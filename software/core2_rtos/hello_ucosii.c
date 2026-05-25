// core 2
#include <image.h>
#include <emergencyIMG2.h>
#include "system.h"
#include <stdio.h>
#include <stdint.h>
#include <system.h>
#include <io.h>
#include <stddef.h>
#include "sys/alt_stdio.h"
#include "altera_avalon_pio_regs.h"
#include "alt_types.h"
#include <string.h>
#include "sys/alt_irq.h"
#include "unistd.h"
#include "includes.h"

OS_EVENT *vga_trigger_sem;

int* CORE0_WRADDR    = (int*) 0x01E12C0C;
int* CORE2_WRADDR    = (int*) 0x01E12C20;
int* CORE1_WRADDR    = (int*) 0x01E12C40;
int* DRAM_WRADDR     = (int*) 0x01E00000;
int* ACCE_WRADDR     = (int*) 0x01E12C00;
int* EMER_ADDR       = (int*) 0x01E12C14;
int* SCORE_WRADDR    = (int*) 0x01E12C50;
int* WORD_WRADDR     = (int*) 0x01E12C80;
int* SETTINGS_WRADDR = (int*) 0x01E12CA0;

volatile int edge_capture      = 0;
volatile int rx_buffer         = 0;
volatile int emergency_stop    = 0;
volatile int emergency_counter = 0;
volatile int is_cube           = 0;
volatile int is_spi            = 0;
volatile int key0_click        = 0;
volatile int key1_click        = 0;

static int current_page    = 0;
static int settings_cursor = 0;
static int settings_rot    = 1;
static int settings_spin   = 1;

static uint8_t cube_buf[320 * 240];

static void init_rx();
static void handle_rx_interrupts(void* context, alt_u32 id);
static void draw_ui_page(int page);
void write_pixel_array(const uint8_t* ADDRESS);

static void init_rx() {
    void* p = (void*) &edge_capture;
    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(IRQ_CORE2_RX_BASE, 0xFFFFFFFF);
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(IRQ_CORE2_RX_BASE, 0);
    alt_irq_register(IRQ_CORE2_RX_IRQ, p, handle_rx_interrupts);
}

// ============================================================================
// ISR — all mode transitions handled explicitly, no toggles
// ============================================================================
static void handle_rx_interrupts(void* context, alt_u32 id) {
    volatile int* edge_capture_ptr = (volatile int*) context;
    int edge = IORD_ALTERA_AVALON_PIO_EDGE_CAP(IRQ_CORE2_RX_BASE);

    if (edge & 0x00000001) emergency_stop = 1;
    if (edge & 0x00000002) emergency_stop = 0;
    *edge_capture_ptr = edge;

    // Camera frame complete (from Core 1) — display it
    if (edge & 0x80000000) {
        rx_buffer |= 0x80000000;
        OSSemPost(vga_trigger_sem);
    }
    // Camera ON  (explicit, from Core 1 SW6 rising)
    if (edge & 0x40000000) {
        is_spi  = 1;
        is_cube = 0;
        OSSemPost(vga_trigger_sem);
    }
    // Camera OFF (explicit, from Core 1 SW6 falling OR SW5 cube-start)
    if (edge & 0x08000000) {
        is_spi = 0;
        OSSemPost(vga_trigger_sem);
    }
    // Cube mode toggled (from Core 1 SW5)
    // Core 0 handles the actual render toggle; we just clear cube display state
    if (edge & 0x00007000) {
        is_cube   = 0;
        rx_buffer &= ~0x00000004;
        OSSemPost(vga_trigger_sem);
    }
    // KEY0 — prev page / confirm setting
    if (edge & 0x10000000) {
        key0_click = 1;
        OSSemPost(vga_trigger_sem);
    }
    // KEY1 — next page / skip setting
    if (edge & 0x20000000) {
        key1_click = 1;
        OSSemPost(vga_trigger_sem);
    }
    // Cube frame ready (from Core 0)
    if (edge & 0x00000004) {
        rx_buffer |= 0x00000004;
        is_cube = 1;
        OSSemPost(vga_trigger_sem);
    }
    // Emergency signals
    else if (edge & 0x00000003) {
        OSSemPost(vga_trigger_sem);
    }

    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(IRQ_CORE2_RX_BASE, edge);
}

// ============================================================================
// UI PRIMITIVES
// ============================================================================
static const char UI_FONT_CHARS[] = " -:./0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ%<>";
static const uint8_t UI_FONT_DATA[43][6] = {
    {0x0,0x0,0x0,0x0,0x0,0x0},{0x0,0x0,0xF,0x0,0x0,0x0},{0x0,0x6,0x6,0x0,0x6,0x6},
    {0x0,0x0,0x0,0x0,0x0,0xF},{0x1,0x2,0x4,0x8,0x4,0x2},{0x6,0x9,0x9,0x9,0x9,0x6},
    {0x2,0x6,0x2,0x2,0x2,0x7},{0x6,0x9,0x1,0x2,0x4,0xF},{0xE,0x1,0x6,0x1,0x1,0xE},
    {0x9,0x9,0xF,0x1,0x1,0x1},{0xF,0x8,0xE,0x1,0x1,0xE},{0x6,0x8,0xE,0x9,0x9,0x6},
    {0xF,0x1,0x2,0x4,0x4,0x4},{0x6,0x9,0x6,0x9,0x9,0x6},{0x6,0x9,0x7,0x1,0x9,0x6},
    {0x6,0x9,0x9,0xF,0x9,0x9},{0xE,0x9,0xE,0x9,0x9,0xE},{0x6,0x9,0x8,0x8,0x9,0x6},
    {0xE,0x9,0x9,0x9,0x9,0xE},{0xF,0x8,0xE,0x8,0x8,0xF},{0xF,0x8,0xE,0x8,0x8,0x8},
    {0x6,0x9,0x8,0xB,0x9,0x6},{0x9,0x9,0xF,0x9,0x9,0x9},{0xE,0x4,0x4,0x4,0x4,0xE},
    {0x3,0x1,0x1,0x1,0x9,0x6},{0x9,0xA,0xC,0xC,0xA,0x9},{0x8,0x8,0x8,0x8,0x8,0xF},
    {0x9,0xF,0x9,0x9,0x9,0x9},{0x9,0xD,0xB,0x9,0x9,0x9},{0x6,0x9,0x9,0x9,0x9,0x6},
    {0xE,0x9,0x9,0xE,0x8,0x8},{0x6,0x9,0x9,0x9,0xD,0x6},{0xE,0x9,0x9,0xE,0xA,0x9},
    {0x7,0x8,0x6,0x1,0x1,0xE},{0xF,0x4,0x4,0x4,0x4,0x4},{0x9,0x9,0x9,0x9,0x9,0x6},
    {0x9,0x9,0x9,0x9,0x6,0x6},{0x9,0x9,0xF,0xF,0x9,0x9},{0x9,0x9,0x6,0x6,0x9,0x9},
    {0x9,0x9,0x6,0x4,0x4,0x4},{0xF,0x1,0x2,0x4,0x8,0xF},{0x1,0xA,0x2,0x4,0x5,0x8},
    {0x2,0x4,0x8,0x4,0x2,0x0},{0x4,0x2,0x1,0x2,0x4,0x0},
};

#define UI_BLACK  0x00
#define UI_WHITE  0xFF
#define UI_YELLOW 0xFC
#define UI_CYAN   0x1F
#define UI_GREEN  0x1C
#define UI_RED    0xE0
#define UI_LGREY  0xB6
#define UI_DGREY  0x49
#define UI_ORANGE 0xF4
#define UI_W 320
#define UI_H 240

static uint8_t ui_buf[UI_W * UI_H];

static void ui_fill(uint8_t c) { memset(ui_buf, c, sizeof(ui_buf)); }
static void ui_hline(int x, int y, int len, uint8_t c) {
    for (int i=x; i<x+len && i<UI_W; i++) if (y>=0&&y<UI_H) ui_buf[y*UI_W+i]=c;
}
static void ui_vline(int x, int y, int len, uint8_t c) {
    for (int i=y; i<y+len && i<UI_H; i++) if (x>=0&&x<UI_W) ui_buf[i*UI_W+x]=c;
}
static void ui_rect(int x, int y, int w, int h, uint8_t c) {
    ui_hline(x,y,w,c); ui_hline(x,y+h-1,w,c);
    ui_vline(x,y,h,c); ui_vline(x+w-1,y,h,c);
}
static void ui_fill_rect(int x, int y, int w, int h, uint8_t c) {
    for (int r=y; r<y+h&&r<UI_H; r++) ui_hline(x,r,w,c);
}
static void ui_char(int x, int y, char c, uint8_t col) {
    if (c>='a'&&c<='z') c=c-'a'+'A';
    const char* p=strchr(UI_FONT_CHARS,c); if (!p) return;
    const uint8_t* g=UI_FONT_DATA[p-UI_FONT_CHARS];
    for (int row=0;row<6;row++) for (int co=0;co<4;co++) {
        if (!(g[row]&(0x8>>co))) continue;
        int px=x+co, py=y+row;
        if (px>=0&&px<UI_W&&py>=0&&py<UI_H) ui_buf[py*UI_W+px]=col;
    }
}
static void ui_str(int x, int y, const char* s, uint8_t c) {
    while (*s) { ui_char(x,y,*s++,c); x+=5; }
}
static void ui_char2x(int x, int y, char c, uint8_t col) {
    if (c>='a'&&c<='z') c=c-'a'+'A';
    const char* p=strchr(UI_FONT_CHARS,c); if (!p) return;
    const uint8_t* g=UI_FONT_DATA[p-UI_FONT_CHARS];
    for (int row=0;row<6;row++) for (int co=0;co<4;co++) {
        if (!(g[row]&(0x8>>co))) continue;
        for (int dy=0;dy<2;dy++) for (int dx=0;dx<2;dx++) {
            int px=x+co*2+dx, py=y+row*2+dy;
            if (px>=0&&px<UI_W&&py>=0&&py<UI_H) ui_buf[py*UI_W+px]=col;
        }
    }
}
static void ui_str2x(int x, int y, const char* s, uint8_t c) {
    while (*s) { ui_char2x(x,y,*s++,c); x+=10; }
}
static void ui_bar(int x, int y, int mw, int h, int val, uint8_t fg, uint8_t bg) {
    int f=(val*mw)/100;
    ui_fill_rect(x,y,f,h,fg); ui_fill_rect(x+f,y,mw-f,h,bg);
    ui_rect(x-1,y-1,mw+2,h+2,UI_DGREY);
}

// ============================================================================
// HEADER + STATUS BAR
// ============================================================================
static void ui_draw_header(int page) {
    char tmp[8];
    ui_fill_rect(0,0,UI_W,16,UI_DGREY);
    ui_str2x(4,2,"SIGN",UI_YELLOW); ui_str2x(44,2,"AI",UI_CYAN);
    snprintf(tmp,sizeof(tmp),"PG %d/3",page+1);
    ui_str(UI_W-36,5,tmp,UI_LGREY);
    const char* names[]={"DASHBOARD","INFERENCE","SETTINGS"};
    int nx=UI_W/2-(int)(strlen(names[page])*5/2);
    ui_str(nx,5,names[page],UI_WHITE);
    ui_hline(0,16,UI_W,UI_CYAN);
}

static void ui_draw_status(void) {
    ui_fill_rect(0,UI_H-13,UI_W,13,UI_DGREY);
    ui_hline(0,UI_H-14,UI_W,UI_CYAN);
    ui_str(4,  UI_H-10,"CAM:",UI_LGREY);
    ui_str(24, UI_H-10,is_spi        ?"ON ":"OFF",is_spi        ?UI_GREEN:UI_RED);
    ui_str(52, UI_H-10,"CUBE:",UI_LGREY);
    ui_str(77, UI_H-10,is_cube        ?"ON ":"OFF",is_cube        ?UI_GREEN:UI_LGREY);
    ui_str(108,UI_H-10,"EMRG:",UI_LGREY);
    ui_str(133,UI_H-10,emergency_stop?"ON ":"OFF",emergency_stop?UI_RED:UI_GREEN);
    ui_str(163,UI_H-10,"ROT:",UI_LGREY);
    ui_str(183,UI_H-10,settings_rot  ?"ON ":"OFF",settings_rot  ?UI_CYAN:UI_LGREY);
    ui_str(210,UI_H-10,"K0<  K1>",UI_DGREY);
}

// ============================================================================
// PAGE 0 — DASHBOARD
// ============================================================================
static void draw_page_dashboard(void) {
    char tmp[24];
    int c0=IORD_32DIRECT(CORE0_WRADDR,0);
    int c1=IORD_32DIRECT(CORE1_WRADDR,0);
    int c2=IORD_32DIRECT(CORE2_WRADDR,0);
    int ax=(int32_t)IORD_32DIRECT(ACCE_WRADDR,0);
    int ay=(int32_t)IORD_32DIRECT(ACCE_WRADDR,4);
    int az=(int32_t)IORD_32DIRECT(ACCE_WRADDR,8);
    int lx=4, cy=22; uint8_t ccol;

    ui_str(lx,cy,"CPU USAGE",UI_LGREY); ui_hline(lx,cy+8,110,UI_DGREY); cy+=12;

    ccol=c0>80?UI_RED:UI_GREEN;
    ui_str(lx,cy,"C0",UI_WHITE); snprintf(tmp,sizeof(tmp),"%3d%",c0); ui_str(lx+14,cy,tmp,ccol);
    ui_bar(lx,cy+8,108,5,c0,ccol,UI_DGREY); cy+=18;
    ccol=c1>80?UI_RED:UI_GREEN;
    ui_str(lx,cy,"C1",UI_WHITE); snprintf(tmp,sizeof(tmp),"%3d%",c1); ui_str(lx+14,cy,tmp,ccol);
    ui_bar(lx,cy+8,108,5,c1,ccol,UI_DGREY); cy+=18;
    ccol=c2>80?UI_RED:UI_GREEN;
    ui_str(lx,cy,"C2",UI_WHITE); snprintf(tmp,sizeof(tmp),"%3d%",c2); ui_str(lx+14,cy,tmp,ccol);
    ui_bar(lx,cy+8,108,5,c2,ccol,UI_DGREY); cy+=16;

    ui_hline(lx,cy,110,UI_DGREY); cy+=6;
    ui_str(lx,cy,"ACCELEROMETER",UI_LGREY); ui_hline(lx,cy+8,110,UI_DGREY); cy+=12;
    snprintf(tmp,sizeof(tmp),"X %5d",ax); ui_str(lx,cy,tmp,UI_CYAN); cy+=9;
    snprintf(tmp,sizeof(tmp),"Y %5d",ay); ui_str(lx,cy,tmp,UI_CYAN); cy+=9;
    snprintf(tmp,sizeof(tmp),"Z %5d",az); ui_str(lx,cy,tmp,UI_CYAN);

    int tx=lx+55, ty=cy-18;
    ui_rect(tx,ty,50,28,UI_DGREY);
    int dot_x=tx+25+(ax*20/512), dot_y=ty+14-(ay*11/512);
    if (dot_x<tx+1) dot_x=tx+1; if (dot_x>tx+48) dot_x=tx+48;
    if (dot_y<ty+1) dot_y=ty+1; if (dot_y>ty+26) dot_y=ty+26;
    ui_fill_rect(dot_x-1,dot_y-1,3,3,UI_YELLOW);
    ui_hline(tx+1,ty+14,48,UI_DGREY); ui_vline(tx+25,ty+1,26,UI_DGREY);

    ui_vline(120,20,UI_H-35,UI_DGREY);
    int rx=126;
    ui_str(rx,22,"CONTROLS",UI_LGREY); ui_hline(rx,30,190,UI_DGREY);
    const char* ck[]={"KEY0","KEY1","SW6 ","SW5 ","SW4 ","SW1 ","SW1-2","SW10"};
    const char* cd[]={"PREV PG/CONFIRM","NEXT PG/SKIP","CAMERA ON/OFF","CUBE MODE","PLAY MUSIC","SCROLL INFER","SCROLL WORD","LED CLEAR"};
    for (int i=0;i<8;i++) {
        int ky=34+i*12;
        ui_str(rx,   ky,ck[i],UI_YELLOW);
        ui_str(rx+32,ky,cd[i],UI_WHITE);
    }
}

// ============================================================================
// PAGE 1 — INFERENCE
// ============================================================================
static void draw_page_inference(void) {
    uint8_t alpha_s[26], num_s[10];
    for (int i=0;i<26;i++) alpha_s[i]=IORD_8DIRECT(SCORE_WRADDR,i);
    for (int i=0;i<10;i++) num_s[i]  =IORD_8DIRECT(SCORE_WRADDR,26+i);

    ui_str(4,22,"LETTER SCORES",UI_LGREY); ui_hline(0,30,UI_W,UI_DGREY);

    #define INF_BASE 160
    #define INF_H     28
    #define INF_BW     5
    #define INF_BG     1

    for (int i=0;i<26;i++) {
        int bx=4+i*(INF_BW+INF_BG);
        int bh=(int)((uint32_t)alpha_s[i]*INF_H/100);
        uint8_t col=alpha_s[i]>70?UI_GREEN:alpha_s[i]>30?UI_YELLOW:UI_DGREY;
        ui_fill_rect(bx,INF_BASE-INF_H,INF_BW,INF_H,UI_DGREY);
        if (bh>0) ui_fill_rect(bx,INF_BASE-bh,INF_BW,bh,col);
        ui_rect(bx-1,INF_BASE-INF_H-1,INF_BW+2,INF_H+2,0x24);
        ui_char(bx,INF_BASE+2,'A'+i,alpha_s[i]>30?col:UI_LGREY);
    }
    int nx0=4+26*(INF_BW+INF_BG)+6;
    for (int i=0;i<10;i++) {
        int bx=nx0+i*(INF_BW+INF_BG);
        int bh=(int)((uint32_t)num_s[i]*INF_H/100);
        uint8_t col=num_s[i]>70?UI_GREEN:num_s[i]>30?UI_YELLOW:UI_DGREY;
        ui_fill_rect(bx,INF_BASE-INF_H,INF_BW,INF_H,UI_DGREY);
        if (bh>0) ui_fill_rect(bx,INF_BASE-bh,INF_BW,bh,col);
        ui_rect(bx-1,INF_BASE-INF_H-1,INF_BW+2,INF_H+2,0x24);
        ui_char(bx,INF_BASE+2,'0'+i,num_s[i]>30?col:UI_LGREY);
    }
    ui_str(4,   INF_BASE-INF_H-10,"A-Z",UI_LGREY);
    ui_str(nx0, INF_BASE-INF_H-10,"0-9",UI_LGREY);

    int wy=INF_BASE+12;
    ui_hline(0,wy,UI_W,UI_DGREY); wy+=4;

    char wbuf[17]={0};
    for (int i=0;i<16;i++) wbuf[i]=(char)IORD_8DIRECT(WORD_WRADDR,i);
    wbuf[16]='\0';
    int wlen=0; for (int i=0;i<16;i++) if (wbuf[i]>' ') wlen=i+1;
    ui_str(4,wy,"DETECTED:",UI_LGREY);
    if (wlen>0) ui_str2x(60,wy-2,wbuf,UI_YELLOW);
    else        ui_str(60,wy,"---",UI_DGREY);
}

// ============================================================================
// PAGE 2 — SETTINGS
// ============================================================================
static void draw_setting_row(int sy, const char* label, int sel, int val) {
    uint8_t lc=sel?UI_YELLOW:UI_LGREY;
    if (sel) { ui_fill_rect(0,sy-1,UI_W-1,14,0x09); ui_str(2,sy+3,">",UI_YELLOW); }
    ui_str(12,sy+3,label,lc);
    int oa=(val==1), fa=(val==0);
    ui_fill_rect(160,sy,32,12,oa?UI_CYAN :UI_DGREY); ui_rect(160,sy,32,12,oa?UI_WHITE:UI_LGREY); ui_str(166,sy+3,"ON", oa?0x00:UI_LGREY);
    ui_fill_rect(196,sy,36,12,fa?UI_RED  :UI_DGREY); ui_rect(196,sy,36,12,fa?UI_WHITE:UI_LGREY); ui_str(200,sy+3,"OFF",fa?0x00:UI_LGREY);
}

static void draw_page_settings(void) {
    char tmp[40]; int sy=24;
    ui_str(4,sy,"SYSTEM SETTINGS",UI_LGREY); ui_hline(0,sy+9,UI_W,UI_DGREY); sy+=16;
    ui_str(4,sy,"KEY0:CONFIRM   KEY1:SKIP",UI_DGREY); sy+=14;
    draw_setting_row(sy,"VIDEO ROTATION",settings_cursor==0,settings_rot); sy+=20;
    ui_hline(4,sy,UI_W-8,0x12); sy+=6;
    draw_setting_row(sy,"CUBE AUTO-SPIN",settings_cursor==1,settings_spin); sy+=20;
    ui_hline(4,sy,UI_W-8,0x12); sy+=10;
    ui_rect(4,sy,UI_W-8,32,UI_DGREY);
    ui_str(8,sy+4,"ACTIVE CONFIG:",UI_LGREY);
    snprintf(tmp,sizeof(tmp),"ROT:%s   CUBE-SPIN:%s",
             settings_rot?"ON ":"OFF", settings_spin?"ON ":"OFF");
    ui_str(8,sy+14,tmp,UI_CYAN); sy+=40;
    ui_str(4,sy,"KEY0",UI_ORANGE); ui_str(24,sy,"TOGGLE HIGHLIGHTED + ADVANCE",UI_WHITE); sy+=10;
    ui_str(4,sy,"KEY1",UI_LGREY);  ui_str(24,sy,"SKIP   HIGHLIGHTED + ADVANCE",UI_LGREY);
}

static void draw_ui_page(int page) {
    ui_fill(UI_BLACK);
    ui_draw_header(page);
    switch (page) {
        case 0: draw_page_dashboard(); break;
        case 1: draw_page_inference(); break;
        case 2: draw_page_settings();  break;
    }
    ui_draw_status();
    write_pixel_array((const uint8_t*)ui_buf);
}

// ============================================================================
// PIXEL WRITE
// ============================================================================
void write_pixel_color(int* ADDRESS) {
    int img_addr=0;
    for (int i=0;i<19200;i++) {
        uint32_t word=IORD_32DIRECT(ADDRESS,i*4);
        IOWR_ALTERA_AVALON_PIO_DATA(IMG_ADDR_BASE,img_addr++); IOWR_ALTERA_AVALON_PIO_DATA(PIXEL_BASE,(uint8_t)(word>>24)); IOWR_ALTERA_AVALON_PIO_DATA(WREN_BASE,1); IOWR_ALTERA_AVALON_PIO_DATA(WREN_BASE,0);
        IOWR_ALTERA_AVALON_PIO_DATA(IMG_ADDR_BASE,img_addr++); IOWR_ALTERA_AVALON_PIO_DATA(PIXEL_BASE,(uint8_t)(word>>16)); IOWR_ALTERA_AVALON_PIO_DATA(WREN_BASE,1); IOWR_ALTERA_AVALON_PIO_DATA(WREN_BASE,0);
        IOWR_ALTERA_AVALON_PIO_DATA(IMG_ADDR_BASE,img_addr++); IOWR_ALTERA_AVALON_PIO_DATA(PIXEL_BASE,(uint8_t)(word>>8));  IOWR_ALTERA_AVALON_PIO_DATA(WREN_BASE,1); IOWR_ALTERA_AVALON_PIO_DATA(WREN_BASE,0);
        IOWR_ALTERA_AVALON_PIO_DATA(IMG_ADDR_BASE,img_addr++); IOWR_ALTERA_AVALON_PIO_DATA(PIXEL_BASE,(uint8_t)word);       IOWR_ALTERA_AVALON_PIO_DATA(WREN_BASE,1); IOWR_ALTERA_AVALON_PIO_DATA(WREN_BASE,0);
    }
}
void write_pixel_array(const uint8_t* ADDRESS) {
    for (int i=0;i<UI_W*UI_H;i++) {
        IOWR_ALTERA_AVALON_PIO_DATA(IMG_ADDR_BASE,i);
        IOWR_ALTERA_AVALON_PIO_DATA(PIXEL_BASE,ADDRESS[i]);
        IOWR_ALTERA_AVALON_PIO_DATA(WREN_BASE,1); IOWR_ALTERA_AVALON_PIO_DATA(WREN_BASE,0);
    }
}

// ============================================================================
// KEY HANDLER
// ============================================================================
static void handle_key(int is_confirm) {
    if (current_page == 2) {
        if (is_confirm) {
            if (settings_cursor == 0) { settings_rot  = !settings_rot;  IOWR_8DIRECT(SETTINGS_WRADDR,0,settings_rot); }
            else                      { settings_spin = !settings_spin; IOWR_8DIRECT(SETTINGS_WRADDR,1,settings_spin); }
        }
        if (++settings_cursor >= 2) { settings_cursor = 0; current_page = 0; }
    } else {
        current_page = is_confirm ? (current_page+2)%3 : (current_page+1)%3;
        if (current_page == 2) settings_cursor = 0;
    }
}

// ============================================================================
// TASKS
// ============================================================================
#define TASK_STACKSIZE 1024
OS_STK vga_task_stk        [TASK_STACKSIZE];
OS_STK startup_task_stk    [TASK_STACKSIZE];
OS_STK cpu_monitor_task_stk[TASK_STACKSIZE];
#define VGA_PRIORITY     1
#define CPU_MONITOR_PRIO 2

void vga_task(void* pdata) {
    INT8U err;
    IOWR_8DIRECT(SETTINGS_WRADDR, 0, settings_rot);
    IOWR_8DIRECT(SETTINGS_WRADDR, 1, settings_spin);

    while (1) {
        OSSemPend(vga_trigger_sem, OS_TICKS_PER_SEC / 2, &err);

        if (!emergency_stop) {
            emergency_counter = 0;

            if (key0_click) { key0_click = 0; handle_key(1); }
            if (key1_click) { key1_click = 0; handle_key(0); }

            if (rx_buffer & 0x00000004) {
                // New cube frame: copy DRAM → cube_buf → VGA
                for (int i=0; i<(UI_W*UI_H)/4; i++) {
                    uint32_t w=IORD_32DIRECT(DRAM_WRADDR,i*4);
                    cube_buf[i*4+0]=(w>>24)&0xFF; cube_buf[i*4+1]=(w>>16)&0xFF;
                    cube_buf[i*4+2]=(w>> 8)&0xFF; cube_buf[i*4+3]= w     &0xFF;
                }
                write_pixel_array(cube_buf);
                rx_buffer &= ~0x00000004;
                rx_buffer &= ~0x80000000;
                // Signal Core 0 that Core 2 is done with this frame
                IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE2_TX_BASE, 0x80000000);
                IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE2_TX_BASE, 0x0);

            } else if (rx_buffer & 0x80000000) {
                // New camera frame
                write_pixel_color(DRAM_WRADDR);
                rx_buffer &= ~0x80000000;

            } else if (!is_spi && !is_cube) {
                // Idle — draw current UI page
                draw_ui_page(current_page);
            }
            // If is_spi=1 or is_cube=1 but no new frame yet:
            // do nothing — VGA retains the last frame written
        }

        if (emergency_stop) {
            const uint8_t* img = (emergency_counter%2==0)
                                 ? (uint8_t*)emergency_image
                                 : (uint8_t*)emergencyIMG2;
            write_pixel_array(img);
            emergency_counter++;
        }
    }
}

void cpu_monitor_task(void* pdata) {
    while (1) {
        IOWR_32DIRECT(CORE2_WRADDR, 0, OSCPUUsage);
        OSTimeDlyHMSM(0,0,0,500);
    }
}
void startup_task(void* pdata) {
    OSStatInit();
    OSTaskCreateExt(vga_task,         NULL,(void*)&vga_task_stk        [TASK_STACKSIZE-1],VGA_PRIORITY,    VGA_PRIORITY,    vga_task_stk,        TASK_STACKSIZE,NULL,0);
    OSTaskCreateExt(cpu_monitor_task, NULL,(void*)&cpu_monitor_task_stk[TASK_STACKSIZE-1],CPU_MONITOR_PRIO,CPU_MONITOR_PRIO,cpu_monitor_task_stk,TASK_STACKSIZE,NULL,0);
    OSTaskDel(OS_PRIO_SELF);
}
int main(void) {
    alt_printf("CPU Core 2 Alive\n");
    vga_trigger_sem = OSSemCreate(0);
    init_rx();
    OSTaskCreateExt(startup_task,NULL,(void*)&startup_task_stk[TASK_STACKSIZE-1],0,0,startup_task_stk,TASK_STACKSIZE,NULL,0);
    OSStart();
    return 0;
}
