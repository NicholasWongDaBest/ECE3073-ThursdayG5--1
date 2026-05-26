// core 1
#include "system.h"
#include <stdio.h>
#include <stdint.h>
#include <system.h>
#include <io.h>
#include "altera_avalon_spi.h"
#include "altera_avalon_spi_regs.h"
#include <stddef.h>
#include "sys/alt_stdio.h"
#include "altera_avalon_pio_regs.h"
#include "alt_types.h"
#include <string.h>
#include "altera_up_avalon_accelerometer_spi.h"
#include "sys/alt_irq.h"
#include "unistd.h"
#include "includes.h"

volatile int rotation = 0;

OS_EVENT *accel_sem;
OS_EVENT *music_sem;

int* DRAM_WRADDR     = (int*) 0x01E00000;
int* ACCE_WRADDR     = (int*) 0x01E12C00;
int* CORE0_WRADDR    = (int*) 0x01E12C0C;
int* CORE2_WRADDR    = (int*) 0x01E12C20;
int* CORE1_WRADDR    = (int*) 0x01E12C40;
int* SCORE_WRADDR    = (int*) 0x01E12C50;
int* WORD_WRADDR     = (int*) 0x01E12C80;
int* SETTINGS_WRADDR = (int*) 0x01E12CA0;
// SETTINGS_WRADDR layout:
//   byte 0 = rot_en      (Core 2 writes, Core 1 reads)
//   byte 1 = spin_en     (Core 2 writes, Core 0 reads)
//   byte 2 = cam_active  (Core 2 writes, Core 1 reads) ← camera page flag
#define SPI_BASE 0x84001040

// IRQ bit assignments
// 0x80000000  camera frame complete  (Core1 → Core0+Core2)
// 0x40000000  camera ON  (explicit)  (Core1 → Core2)
// 0x08000000  camera OFF (explicit)  (Core1 → Core2)
// 0x20000000  KEY1 next page/skip setting (Core1 → Core2)
// 0x10000000  KEY0 prev page/confirm setting (Core1 → Core2)
// 0x00007000  cube mode toggle (Core1 → Core0; Core2 also handles: resets is_cube)
// 0x00008000  accel data ready (Core1 → Core0)
// 0x00010001  emergency ON (Core0 → Core1+Core2)
// 0x00020002  emergency OFF

volatile int edge_capture   = 0;
volatile int start_spi      = 0;
volatile int emergency_stop = 0;
volatile int scroll_flag    = 0;
volatile int video_mode     = 0;

alt_up_accelerometer_spi_dev *accel;

void display_6chars(char* msg, int offset, int len);
unsigned char get_seg7(char c);
void play_tone();

unsigned char seg7_alpha[]   = {0x88,0x83,0xC6,0xA1,0x86,0x8E,0xC2,0x89,0xF9,0xE1,0x89,0xC7,0xC8,0xAB,0xC0,0x8C,0x98,0xAF,0x92,0x87,0xC1,0xC1,0xC1,0x89,0x91,0xA4,0xFF};
unsigned char seg7_numbers[] = {0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90};

static void handle_rx_interrupts(void* context, alt_u32 id) {
    int edge = IORD_ALTERA_AVALON_PIO_EDGE_CAP(IRQ_CORE1_RX_BASE);
    if (edge & 0x00010000) emergency_stop = 1;
    if (edge & 0x00020000) emergency_stop = 0;
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(IRQ_CORE1_RX_BASE, edge);
}

static void handle_key_interrupts(void* context, alt_u32 id) {
    int edge = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTONS_BASE);
    if (edge & 0x1) {  // KEY0 → prev page / confirm setting
        IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE1_TX_BASE, 0x10000000);
        IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE1_TX_BASE, 0x00000000);
    }
    if (edge & 0x2) {  // KEY1 → next page / skip setting
        IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE1_TX_BASE, 0x20000000);
        IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE1_TX_BASE, 0x00000000);
    }
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTONS_BASE, 0);
}

static void handle_timer_interrupts(void* context, alt_u32 id) {
    int edge = IORD_ALTERA_AVALON_PIO_EDGE_CAP(CLOCK_INTERRUPT_BASE);
    if (edge & 0x1) scroll_flag = 1;
    if (edge & 0x2) OSSemPost(accel_sem);
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(CLOCK_INTERRUPT_BASE, 0);
}

static void init_rx() {
    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(IRQ_CORE1_RX_BASE, 0xFFFFFFFF);
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(IRQ_CORE1_RX_BASE, 0);
    alt_irq_register(IRQ_CORE1_RX_IRQ, (void*)&edge_capture, handle_rx_interrupts);
}
static void init_key_pio() {
    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTONS_BASE, 0x3);
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTONS_BASE, 0);
    alt_irq_register(PUSH_BUTTONS_IRQ, (void*)&edge_capture, handle_key_interrupts);
}
static void init_timer() {
    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(CLOCK_INTERRUPT_BASE, 0x3);
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(CLOCK_INTERRUPT_BASE, 0);
    alt_irq_register(CLOCK_INTERRUPT_IRQ, NULL, handle_timer_interrupts);
}

unsigned char get_seg7(char c) {
    if (c>='A'&&c<='Z') return seg7_alpha[c-'A'];
    if (c>='a'&&c<='z') return seg7_alpha[c-'a'];
    if (c>='0'&&c<='9') return seg7_numbers[c-'0'];
    if (c==' ') return 0xFF; if (c=='-') return 0xBF;
    if (c=='.') return 0x7F; return 0xFF;
}
void display_6chars(char* msg, int offset, int len) {
    char chars[6];
    for (int i=0; i<6; i++) chars[i]=msg[(offset+i)%len];
    unsigned int hex012=(get_seg7(chars[3])<<16)|(get_seg7(chars[4])<<8)|get_seg7(chars[5]);
    unsigned int hex345=(get_seg7(chars[0])<<16)|(get_seg7(chars[1])<<8)|get_seg7(chars[2]);
    IOWR_ALTERA_AVALON_PIO_DATA(HEX012_BASE, hex012);
    IOWR_ALTERA_AVALON_PIO_DATA(HEX345_BASE, hex345);
}

#define NOTE_REST 0
#define NOTE_D4  10
#define NOTE_E4  12
#define NOTE_F4  13
#define NOTE_G4  15
#define NOTE_A4  17
#define NOTE_AS4 18
#define NOTE_C5  20

void play_tone() {
    IOWR_ALTERA_AVALON_PIO_DATA(BUZZER_BASE,NOTE_G4);  OSTimeDlyHMSM(0,0,0,900);
    IOWR_ALTERA_AVALON_PIO_DATA(BUZZER_BASE,NOTE_A4);  OSTimeDlyHMSM(0,0,0,150);
    IOWR_ALTERA_AVALON_PIO_DATA(BUZZER_BASE,NOTE_AS4); OSTimeDlyHMSM(0,0,0,150);
    IOWR_ALTERA_AVALON_PIO_DATA(BUZZER_BASE,NOTE_G4);  OSTimeDlyHMSM(0,0,0,600);
    IOWR_ALTERA_AVALON_PIO_DATA(BUZZER_BASE,NOTE_F4);  OSTimeDlyHMSM(0,0,0,550);
    IOWR_ALTERA_AVALON_PIO_DATA(BUZZER_BASE,NOTE_REST);OSTimeDlyHMSM(0,0,0,100);
    IOWR_ALTERA_AVALON_PIO_DATA(BUZZER_BASE,NOTE_F4);  OSTimeDlyHMSM(0,0,0,550);
    IOWR_ALTERA_AVALON_PIO_DATA(BUZZER_BASE,NOTE_D4);  OSTimeDlyHMSM(0,0,0,600);
    IOWR_ALTERA_AVALON_PIO_DATA(BUZZER_BASE,NOTE_E4);  OSTimeDlyHMSM(0,0,1,200);
    IOWR_ALTERA_AVALON_PIO_DATA(BUZZER_BASE,NOTE_G4);  OSTimeDlyHMSM(0,0,0,900);
    IOWR_ALTERA_AVALON_PIO_DATA(BUZZER_BASE,NOTE_A4);  OSTimeDlyHMSM(0,0,0,150);
    IOWR_ALTERA_AVALON_PIO_DATA(BUZZER_BASE,NOTE_AS4); OSTimeDlyHMSM(0,0,0,150);
    IOWR_ALTERA_AVALON_PIO_DATA(BUZZER_BASE,NOTE_G4);  OSTimeDlyHMSM(0,0,0,600);
    IOWR_ALTERA_AVALON_PIO_DATA(BUZZER_BASE,NOTE_F4);  OSTimeDlyHMSM(0,0,0,550);
    IOWR_ALTERA_AVALON_PIO_DATA(BUZZER_BASE,NOTE_REST);OSTimeDlyHMSM(0,0,0,100);
    IOWR_ALTERA_AVALON_PIO_DATA(BUZZER_BASE,NOTE_F4);  OSTimeDlyHMSM(0,0,0,550);
    IOWR_ALTERA_AVALON_PIO_DATA(BUZZER_BASE,NOTE_AS4); OSTimeDlyHMSM(0,0,0,600);
    IOWR_ALTERA_AVALON_PIO_DATA(BUZZER_BASE,NOTE_C5);  OSTimeDlyHMSM(0,0,1,200);
    IOWR_ALTERA_AVALON_PIO_DATA(BUZZER_BASE,NOTE_REST);
}

// ─────────────────────────────────────────────────────────────────────────────
// TASK 1: CAMERA SPI
// Camera ON/OFF is now controlled by SETTINGS_WRADDR[2] written by Core 2
// when it enters/exits page 3. No SW6 needed.
// ─────────────────────────────────────────────────────────────────────────────
#define PAYLOAD_BYTES 60
#define SPI_WORDS ((PAYLOAD_BYTES + 4) / 4)

volatile char detected_msg[11] = "NO IFR ";
volatile char word_msg[17]     = "NO WORD   ";
volatile int  word_new_flag    = 0;
int timeout = 0;

void camera_spi_task(void* pdata) {
    int offset_write  = 0;
    int write_len     = 0;
    uint32_t buffer[SPI_WORDS];
    int trigger_sent  = 0;
    int prev_cam_flag = 0;   // tracks last known SETTINGS_WRADDR[2]

    while (1) {
        // ── Poll camera-page flag written by Core 2 ──────────────────────
        uint8_t cam_flag = IORD_8DIRECT(SETTINGS_WRADDR, 2);
        if ((int)cam_flag != prev_cam_flag) {
            prev_cam_flag = cam_flag;
            start_spi     = cam_flag ? 1 : 0;
            trigger_sent  = 0;   // reset state machine on mode change
            if (start_spi) {
                // Tell Core 2 camera is ON
                IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE1_TX_BASE, 0x40000000);
                IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE1_TX_BASE, 0x00000000);
            } else {
                // Tell Core 2 camera is OFF
                IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE1_TX_BASE, 0x08000000);
                IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE1_TX_BASE, 0x00000000);
                write_len    = 0;
                offset_write = 0;
            }
        }
        video_mode = start_spi ? 1 : 0;

        // ── Send trigger ─────────────────────────────────────────────────
        if (trigger_sent == 0) {
            IOWR_ALTERA_AVALON_SPI_SLAVE_SEL(SPI_BASE, 0x1);
            if (start_spi) {
                uint8_t rot_en = IORD_8DIRECT(SETTINGS_WRADDR, 0);
                int rot_val    = rot_en ? (rotation & 0x3) : 0;
                IOWR_ALTERA_AVALON_SPI_TXDATA(SPI_BASE, 0x01 | (rot_val << 2));
            } else {
                IOWR_ALTERA_AVALON_SPI_TXDATA(SPI_BASE, 0x02);
            }
            while (!(IORD_ALTERA_AVALON_SPI_STATUS(SPI_BASE) & 0x80));
            IORD_ALTERA_AVALON_SPI_RXDATA(SPI_BASE);
            IOWR_ALTERA_AVALON_SPI_SLAVE_SEL(SPI_BASE, 0x0);

            // Score-only: give ESP32 time to return from slave.wait() and call slave.queue().
            // Camera mode doesn't need this — its SPI packets are queued inside sendImage()
            // so the ESP32 is always ahead of the master there.
            if (!start_spi) {
                OSTimeDlyHMSM(0, 0, 0, 20);  // 20ms >> ESP32 queue latency (~1-5µs)
            }

            trigger_sent = 1;
        }

        // ── SPI transaction ───────────────────────────────────────────────
        OSSchedLock();
        IOWR_ALTERA_AVALON_SPI_CONTROL(SPI_BASE, 0x400);
        IOWR_ALTERA_AVALON_SPI_SLAVE_SEL(SPI_BASE, 0x1);
        for (uint8_t bi=0; bi<SPI_WORDS; bi++) {
            while (!(IORD_ALTERA_AVALON_SPI_STATUS(SPI_BASE) & 0x40));
            IOWR_ALTERA_AVALON_SPI_TXDATA(SPI_BASE, 0x00);
            while (!(IORD_ALTERA_AVALON_SPI_STATUS(SPI_BASE) & 0x80));
            buffer[bi] = IORD_ALTERA_AVALON_SPI_RXDATA(SPI_BASE);
        }
        IOWR_ALTERA_AVALON_SPI_SLAVE_SEL(SPI_BASE, 0x0);
        IOWR_ALTERA_AVALON_SPI_CONTROL(SPI_BASE, 0x000);
        OSSchedUnlock();

        // ── State machine (same for camera + idle) ────────────────────────
        if (buffer[0]==0xFF00FF00 && buffer[1]==0xFF00FF00) {
            offset_write=0; write_len=0;
            OSTimeDlyHMSM(0,0,0,1); continue;
        }
        if (buffer[0]==0xEE11EE11 && buffer[1]==0xEE11EE11) {
            timeout=0;
            if (write_len==76800) {
                IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE1_TX_BASE, 0x80000000);
                IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE1_TX_BASE, 0x00000000);
                IOWR_ALTERA_AVALON_PIO_DATA(ONBOARD_LEDS_BASE, 0x0);
            } else {
                IOWR_ALTERA_AVALON_PIO_DATA(ONBOARD_LEDS_BASE, 0x200);
            }
            write_len=0; trigger_sent=0;
            OSTimeDlyHMSM(0,0,0,1); continue;
        } else {
            if (timeout>=1500){trigger_sent=0; timeout=0;} timeout++;
        }

        if ((buffer[0]>>16)==0xC0C0 && (buffer[SPI_WORDS-1]&0xFFFF)==0xC0C0) {
            char lbl[7];
            lbl[0]=(buffer[0]>>8)&0xFF; lbl[1]=buffer[0]&0xFF;
            lbl[2]=(buffer[1]>>24)&0xFF;lbl[3]=(buffer[1]>>16)&0xFF;
            lbl[4]=(buffer[1]>>8)&0xFF; lbl[5]=buffer[1]&0xFF; lbl[6]='\0';
            for(int i=5;i>=0;i--){if(lbl[i]=='\0'||lbl[i]==' ')lbl[i]='\0';else break;}
            uint8_t avg_conf=(buffer[2]>>24)&0xFF;
            snprintf((char*)detected_msg,11,"%-2s-%03d  ",lbl,avg_conf);
        }
        else if ((buffer[0]>>16)==0xA0A0 && (buffer[SPI_WORDS-1]&0xFFFF)==0xA0A0) {
            if (start_spi) {
                int pw=PAYLOAD_BYTES/4;
                for(int i=0;i<pw;i++){
                    uint32_t word=(buffer[i]<<16)|(buffer[i+1]>>16);
                    IOWR_32DIRECT(DRAM_WRADDR,offset_write,word);
                    offset_write+=4; write_len+=4;
                }
            }
        }
        else if ((buffer[0]>>16)==0xD0D0 && (buffer[SPI_WORDS-1]&0xFFFF)==0xD0D0) {
            uint8_t flag=(buffer[4]>>16)&0xFF;
            if (flag==0x01) {
                char tmp[17]={0};
                tmp[0]=(buffer[0]>>8)&0xFF;  tmp[1]=buffer[0]&0xFF;
                tmp[2]=(buffer[1]>>24)&0xFF; tmp[3]=(buffer[1]>>16)&0xFF;
                tmp[4]=(buffer[1]>>8)&0xFF;  tmp[5]=buffer[1]&0xFF;
                tmp[6]=(buffer[2]>>24)&0xFF; tmp[7]=(buffer[2]>>16)&0xFF;
                tmp[8]=(buffer[2]>>8)&0xFF;  tmp[9]=buffer[2]&0xFF;
                tmp[10]=(buffer[3]>>24)&0xFF;tmp[11]=(buffer[3]>>16)&0xFF;
                tmp[12]=(buffer[3]>>8)&0xFF; tmp[13]=buffer[3]&0xFF;
                tmp[14]=(buffer[4]>>24)&0xFF;tmp[15]=' ';tmp[16]='\0';
                strncpy((char*)word_msg,tmp,16);
                word_msg[16]='\0'; word_new_flag=1;
            }
        }
        else if ((buffer[0]>>16)==0xE0E0 && (buffer[SPI_WORDS-1]&0xFFFF)==0xE0E0) {
            uint8_t raw[64];
            for(int w=0;w<SPI_WORDS;w++){
                raw[w*4+0]=(buffer[w]>>24)&0xFF; raw[w*4+1]=(buffer[w]>>16)&0xFF;
                raw[w*4+2]=(buffer[w]>>8)&0xFF;  raw[w*4+3]=buffer[w]&0xFF;
            }
            for(int i=0;i<36;i++) IOWR_8DIRECT(SCORE_WRADDR,i,raw[2+i]);
            for(int i=0;i<16;i++) IOWR_8DIRECT(WORD_WRADDR, i,raw[38+i]);
            if (raw[53]==0x01){
                for(int i=0;i<15;i++) word_msg[i]=(char)raw[38+i];
                word_msg[15]=' '; word_msg[16]='\0'; word_new_flag=1;
            }
            if (!start_spi) trigger_sent=0;
        }

        OSTimeDlyHMSM(0,0,0,start_spi?1:10);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// TASK 2: ACCELEROMETER
// ─────────────────────────────────────────────────────────────────────────────
void accel_task(void* pdata) {
    INT8U err;
    alt_32 x,y,z;
    int emergency_led=0;
    while (1) {
        OSSemPend(accel_sem,0,&err);
        alt_up_accelerometer_spi_read_x_axis(accel,&x);
        alt_up_accelerometer_spi_read_y_axis(accel,&y);
        alt_up_accelerometer_spi_read_z_axis(accel,&z);
        IOWR_32DIRECT(ACCE_WRADDR,0,(uint32_t)x);
        IOWR_32DIRECT(ACCE_WRADDR,4,(uint32_t)y);
        IOWR_32DIRECT(ACCE_WRADDR,8,(uint32_t)z);
        IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE1_TX_BASE,0x00008000);
        IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE1_TX_BASE,0x00000000);
        if (z<-208){
            IOWR_ALTERA_AVALON_PIO_DATA(ONBOARD_LEDS_BASE,emergency_led?0x3FF:0x000);
            emergency_led=!emergency_led;
        } else {
            if      (x<-250&&y>-200) rotation=1;
            else if (x> 230&&y>-200) rotation=2;
            else if (        y<-240) rotation=3;
            else                     rotation=0;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// TASK 3: UI (7-seg scroll, SW5 cube, SW4 music, SW10 LED clear)
// SW6 removed — camera is now page 3
// ─────────────────────────────────────────────────────────────────────────────
void ui_task(void* pdata) {
    int     scroll_offset=0;
    char    cpu_message[7]="C0-000";
    uint8_t cpu_idx=0;
    int     prev_sw4=0, prev_sw5=0;

    while (1) {
        OSTimeDly(2);

        char* message=(char*)detected_msg;
        int   len=strlen(message);
        if (len==0) len=1;

        int sw   =IORD_ALTERA_AVALON_PIO_DATA(SWITCHES_BASE);
        int sw1  =(sw>>0)&0x1;
        int sw2  =(sw>>1)&0x1;
        int sw3  =(sw>>2)&0x1;
        int sw4  =(sw>>3)&0x1;
        int sw5  =(sw>>4)&0x1;  // cube mode toggle
        int sw10 =(sw>>9)&0x1;

        if (!emergency_stop) {
            if (sw4&&!prev_sw4) OSSemPost(music_sem);

            // Cube mode — SW5 rising edge
            if (sw5&&!prev_sw5) {
                // If camera was on (start_spi managed by DRAM flag),
                // camera SPI task will detect the flag change and send OFF.
                // We just toggle cube here.
                IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE1_TX_BASE,0x00007000);
                IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE1_TX_BASE,0x00000000);
            }
            // Cube mode — SW5 falling edge (toggle off)
            if (!sw5&&prev_sw5) {
                IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE1_TX_BASE,0x00007000);
                IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE1_TX_BASE,0x00000000);
            }

            prev_sw4=sw4; prev_sw5=sw5;
            if (sw10) IOWR_ALTERA_AVALON_PIO_DATA(ONBOARD_LEDS_BASE,0x0);

            if (!scroll_flag){continue;} scroll_flag=0;

            if (!sw1) {
                scroll_offset=0;
                IOWR_ALTERA_AVALON_PIO_DATA(HEX012_BASE,0xFFFFFF);
                IOWR_ALTERA_AVALON_PIO_DATA(HEX345_BASE,0xFFFFFF);
            } else if (sw1&&sw2) {
                if (sw3) {
                    int core_state=cpu_idx/4;
                    if      (core_state==0){int v=IORD_32DIRECT(CORE0_WRADDR,0);snprintf(cpu_message,7,"C0-%03d",v);}
                    else if (core_state==1){snprintf(cpu_message,7,"C1-%03d",OSCPUUsage);}
                    else if (core_state==2){int v=IORD_32DIRECT(CORE2_WRADDR,0);snprintf(cpu_message,7,"C2-%03d",v);}
                    display_6chars(cpu_message,0,strlen(cpu_message));
                    if (++cpu_idx>=12) cpu_idx=0;
                } else {
                    char* wm=(char*)word_msg; int wlen=strlen(wm);
                    if (wlen==0) wlen=1;
                    display_6chars(wm,scroll_offset,wlen);
                    scroll_offset=(scroll_offset+1)%wlen;
                }
            } else {
                display_6chars(message,scroll_offset,len);
            }
        }
        IOWR_32DIRECT(CORE1_WRADDR,0,OSCPUUsage);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// TASK 4: MUSIC
// ─────────────────────────────────────────────────────────────────────────────
void music_task(void* pdata) {
    INT8U err;
    while (1){
        OSSemPend(music_sem,0,&err);
        while(OSSemAccept(music_sem)>0);
        play_tone();
    }
}

#define TASK_STACKSIZE 1024
OS_STK camera_task_stk [TASK_STACKSIZE];
OS_STK accel_task_stk  [TASK_STACKSIZE];
OS_STK ui_task_stk     [TASK_STACKSIZE];
OS_STK startup_task_stk[TASK_STACKSIZE];
OS_STK music_task_stk  [TASK_STACKSIZE];

#define CAMERA_PRIORITY 1
#define ACCEL_PRIORITY  2
#define UI_PRIORITY     3
#define MUSIC_PRIORITY  4

void startup_task(void* pdata) {
    OSStatInit();
    OSTaskCreateExt(camera_spi_task,NULL,(void*)&camera_task_stk[TASK_STACKSIZE-1],CAMERA_PRIORITY,CAMERA_PRIORITY,camera_task_stk,TASK_STACKSIZE,NULL,0);
    OSTaskCreateExt(accel_task,     NULL,(void*)&accel_task_stk [TASK_STACKSIZE-1],ACCEL_PRIORITY, ACCEL_PRIORITY, accel_task_stk, TASK_STACKSIZE,NULL,0);
    OSTaskCreateExt(ui_task,        NULL,(void*)&ui_task_stk    [TASK_STACKSIZE-1],UI_PRIORITY,    UI_PRIORITY,    ui_task_stk,    TASK_STACKSIZE,NULL,0);
    OSTaskCreateExt(music_task,     NULL,(void*)&music_task_stk [TASK_STACKSIZE-1],MUSIC_PRIORITY, MUSIC_PRIORITY, music_task_stk, TASK_STACKSIZE,NULL,0);
    OSTaskDel(OS_PRIO_SELF);
}

int main(void) {
    alt_printf("=========================================================\n");
    alt_printf("CPU Core 1 Alive\n");
    alt_printf("=========================================================\n");
    accel=alt_up_accelerometer_spi_open_dev("/dev/accelerometer_spi_0");
    if (!accel){alt_printf("Failed to open accelerometer\n"); return 1;}
    IOWR_ALTERA_AVALON_PIO_DATA(ONBOARD_LEDS_BASE,0x0);
    accel_sem=OSSemCreate(0);
    music_sem=OSSemCreate(0);
    init_rx(); init_key_pio(); init_timer();
    OSTaskCreateExt(startup_task,NULL,(void*)&startup_task_stk[TASK_STACKSIZE-1],0,0,startup_task_stk,TASK_STACKSIZE,NULL,0);
    OSStart();
    return 0;
}
