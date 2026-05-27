//core 0
#include <math.h>
#include "system.h"
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
#include <stdio.h>
#include "includes.h"

// --- RTOS GLOBALS ---
OS_EVENT *rx_trigger_sem;

volatile int rx_buffer      = 0;
volatile int edge_capture   = 0;
volatile int emergency_stop = 0;
volatile int cube_mode      = 0;
volatile int core2_fin      = 1;

static void init_rx();
static void handle_rx_interrupts(void* context, alt_u32 id);

// --- MEMORY MAP ---
int* EMER_ADDR       = (int*) 0x01E12C14;
int* ACCE_WRADDR     = (int*) 0x01E12C00;
int* CORE0_WRADDR    = (int*) 0x01E12C0C;
int* SETTINGS_WRADDR = (int*) 0x01E12CA0; // byte0=rot_en, byte1=spin_en

// --- RENDER CONSTANTS ---
#define LENGTH 320
#define WIDTH  240

static float cube_size   = 15.0f;
static float scale_const = 40.0f;
static float focal       = 100.0f;
static float light_x = 0.0f, light_y = 0.0f, light_z = 1.0f;

static float X = 0.0f;
static float Y = 0.0f;
static float Z = 0.0f;

static float*   buffer_z  = (float*)   0x01E25C00;
static uint8_t* buffer_2d = (uint8_t*) 0x01E00000;

static alt_32 last_ax = 0, last_ay = 0;

// --- ROTATION MATRIX ---
static float r00, r01, r02;
static float r10, r11, r12;
static float r20, r21, r22;

// --- TRANSLATE POINT ---
static void translate_point(float px, float py, float pz,
                             float nx, float ny, float nz) {
    float rx = r00*px + r01*py + r02*pz;
    float ry = r10*px + r11*py + r12*pz;
    float rz = r20*px + r21*py + r22*pz + focal;

    if (rz <= 0.0f) return;

    float rnx = r00*nx + r01*ny + r02*nz;
    float rny = r10*nx + r11*ny + r12*nz;
    float rnz = r20*nx + r21*ny + r22*nz;

    float brightness = rnx*light_x + rny*light_y + rnz*light_z;
    if (brightness < 0.0f) brightness = 0.0f;
    if (brightness > 1.0f) brightness = 1.0f;

    float ooz = 1.0f / rz;
    int xp = (int)(LENGTH/2 + scale_const * rx * ooz * 2);
    int yp = (int)(WIDTH/2  + scale_const * ry * ooz);

    if (xp < 0 || xp >= LENGTH || yp < 0 || yp >= WIDTH) return;

    int idx = xp + LENGTH * yp;
    if (ooz <= buffer_z[idx]) return;
    buffer_z[idx] = ooz;

    uint8_t v = (uint8_t)(brightness * 200.0f + 55.0f);
    uint8_t r3, g3, b2;

    float anx = rnx < 0 ? -rnx : rnx;
    float any = rny < 0 ? -rny : rny;
    float anz = rnz < 0 ? -rnz : rnz;

    if (anx >= any && anx >= anz) {
        r3 = v >> 5; g3 = 0; b2 = 0;          // X face → red
    } else if (any >= anx && any >= anz) {
        r3 = 0; g3 = v >> 5; b2 = 0;          // Y face → green
    } else {
        r3 = 0; g3 = 0; b2 = v >> 6;          // Z face → blue
    }

    buffer_2d[idx] = (r3 << 5) | (g3 << 2) | b2;
}

// --- RENDER CUBE ---
#define ACCEL_MAX 512.0f
static float anim_angle = 0.0f;

static void render_cube(alt_32 ax, alt_32 ay) {
    while (!core2_fin) OSTimeDly(1);
    core2_fin = 0;

    float fax = (float)ax;
    float fay = (float)ay;
    if (fax >  ACCEL_MAX) fax =  ACCEL_MAX;
    if (fax < -ACCEL_MAX) fax = -ACCEL_MAX;
    if (fay >  ACCEL_MAX) fay =  ACCEL_MAX;
    if (fay < -ACCEL_MAX) fay = -ACCEL_MAX;

    int spin_enabled = IORD_8DIRECT(SETTINGS_WRADDR, 1);
    int near_zero    = (ax > -30 && ax < 30 && ay > -30 && ay < 30);

    if (near_zero && spin_enabled) {
        anim_angle += 0.05f;
        if (anim_angle > 6.2832f) anim_angle -= 6.2832f;
        X = 0.4f;
        Y = anim_angle;
        Z = 0.0f;
    } else {
        anim_angle = 0.0f;
        Y =  (fax / ACCEL_MAX) * 1.5708f;
        X = -(fay / ACCEL_MAX) * 1.5708f;
        Z =  0.0f;
    }

    float cx = cosf(X), sx = sinf(X);
    float cy = cosf(Y), sy = sinf(Y);
    float cz = cosf(Z), sz = sinf(Z);

    r00 = cx*cy;  r01 = cx*sy*sz - sx*cz;  r02 = cx*sy*cz + sx*sz;
    r10 = sx*cy;  r11 = sx*sy*sz + cx*cz;  r12 = sx*sy*cz - cx*sz;
    r20 = -sy;    r21 = cy*sz;              r22 = cy*cz;

    memset((void*)buffer_2d, 0, LENGTH * WIDTH * sizeof(uint8_t));
    memset((void*)buffer_z,  0, LENGTH * WIDTH * sizeof(float));

    float lo = -cube_size;
    float hi =  cube_size - 1.0f;

    for (float ci = lo; ci <= hi; ci += 1.0f) {
        for (float cj = lo; cj <= hi; cj += 1.0f) {
            translate_point(ci, cj, lo,  0,  0, -1);  // front
            translate_point(ci, cj, hi,  0,  0,  1);  // back
            translate_point(ci, lo, cj,  0, -1,  0);  // bottom
            translate_point(ci, hi, cj,  0,  1,  0);  // top
            translate_point(lo, ci, cj, -1,  0,  0);  // left
            translate_point(hi, ci, cj,  1,  0,  0);  // right
        }
    }

    // Crosshair
    int cx_px = LENGTH / 2;
    int cy_px = WIDTH  / 2;
    for (int d = -3; d <= 3; d++) {
        int px, py, idx;
        px = cx_px + d; py = cy_px;
        if (px >= 0 && px < LENGTH && py >= 0 && py < WIDTH) {
            idx = py * LENGTH + px; buffer_2d[idx] = 0xE0;
        }
        px = cx_px; py = cy_px + d;
        if (px >= 0 && px < LENGTH && py >= 0 && py < WIDTH) {
            idx = py * LENGTH + px; buffer_2d[idx] = 0xE0;
        }
    }

    IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE0_TX_BASE, 0x00000004);
    IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE0_TX_BASE, 0x00000000);

    last_ax = ax;
    last_ay = ay;
}

// --- ISR ---
static void handle_rx_interrupts(void* context, alt_u32 id) {
    volatile int* edge_capture_ptr = (volatile int*) context;
    int edge = IORD_ALTERA_AVALON_PIO_EDGE_CAP(IRQ_CORE0_RX_BASE);
    *edge_capture_ptr = edge;
    rx_buffer = edge;
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(IRQ_CORE0_RX_BASE, edge);

    if (edge & 0x00007000) {
        cube_mode = !cube_mode;
        if (cube_mode) rx_buffer = 0x00008000;
    } else if (edge & 0x80000000) {
        core2_fin = 1;
    }

    OSSemPost(rx_trigger_sem);
}

static void init_rx() {
    void* edge_capture_ptr = (void*) &edge_capture;
    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(IRQ_CORE0_RX_BASE, 0xFFFFFFFF);
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(IRQ_CORE0_RX_BASE, 0);
    alt_irq_register(IRQ_CORE0_RX_IRQ, edge_capture_ptr, handle_rx_interrupts);
}

// --- TASKS ---
#define TASK_STACKSIZE 1024
OS_STK accel_task_stk  [TASK_STACKSIZE];
OS_STK startup_task_stk[TASK_STACKSIZE];

#define ACCEL_PRIORITY 1

void accel_task(void* pdata) {
    INT8U err;
    while (1) {
        OSSemPend(rx_trigger_sem, 0, &err);

        if (rx_buffer & 0x00008000) {
            int32_t received_x = (int32_t) IORD_32DIRECT(ACCE_WRADDR, 0);
            int32_t received_y = (int32_t) IORD_32DIRECT(ACCE_WRADDR, 4);
            int32_t received_z = (int32_t) IORD_32DIRECT(ACCE_WRADDR, 8);
            rx_buffer = 0x0;

            if (received_z < -208 && !emergency_stop) {
                IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE0_TX_BASE, 0x00010001);
                IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE0_TX_BASE, 0x00000000);
                emergency_stop = 1;
                alt_printf("DEVICE UPSIDE DOWN\n");
            }
            if (received_z > -208 && emergency_stop) {
                IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE0_TX_BASE, 0x00020002);
                IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE0_TX_BASE, 0x00000000);
                emergency_stop = 0;
                alt_printf("DEVICE BACK ALIVE\n");
            }

            IOWR_32DIRECT(CORE0_WRADDR, 0, OSCPUUsage);

            if (cube_mode) {
                alt_32 ax = (received_x != 0 || last_ax == 0) ? received_x : last_ax;
                alt_32 ay = (received_y != 0 || last_ay == 0) ? received_y : last_ay;
                render_cube(ax, ay);
            }
        }
    }
}

void startup_task(void* pdata) {
    OSStatInit();
    OSTaskCreateExt(accel_task, NULL, (void*)&accel_task_stk[TASK_STACKSIZE-1],
                    ACCEL_PRIORITY, ACCEL_PRIORITY, accel_task_stk, TASK_STACKSIZE, NULL, 0);
    OSTaskDel(OS_PRIO_SELF);
}

int main(void) {
    alt_printf("=========================================================\n");
    alt_printf("CPU Core 0 Alive\n");
    alt_printf("=========================================================\n");
    rx_trigger_sem = OSSemCreate(0);
    init_rx();
    OSTaskCreateExt(startup_task, NULL, (void*)&startup_task_stk[TASK_STACKSIZE-1],
                    0, 0, startup_task_stk, TASK_STACKSIZE, NULL, 0);
    OSStart();
    return 0;
}
