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

OS_EVENT *rx_trigger_sem;

volatile int rx_buffer      = 0;
volatile int edge_capture   = 0;
volatile int emergency_stop = 0;
volatile int cube_mode      = 0;
volatile int core2_fin      = 1;
volatile alt_32 last_ax = 0;
volatile alt_32 last_ay = 0;

static void init_rx();
static void handle_rx_interrupts(void* context, alt_u32 id);

int* EMER_ADDR       = (int*) 0x01E12C14;
int* ACCE_WRADDR     = (int*) 0x01E12C00;
int* CORE0_WRADDR    = (int*) 0x01E12C0C;
int* SETTINGS_WRADDR = (int*) 0x01E12CA0;

#define SWAP(a, b) { int t = a; a = b; b = t; }

// Framebuffer in local memory (320x240)
static uint8_t render_fb[320 * 240];

typedef struct { float x, y, z; } Vec3;
typedef struct { int x, y; } Vec2;

// 8 Vertices of a cube
const Vec3 base_vertices[8] = {
    {-1, -1, -1}, { 1, -1, -1}, { 1,  1, -1}, {-1,  1, -1},
    {-1, -1,  1}, { 1, -1,  1}, { 1,  1,  1}, {-1,  1,  1}
};

// 12 Triangles (2 per face) - Defined by vertex indices
const int faces[12][3] = {
    {0, 1, 2}, {0, 2, 3}, // Front
    {1, 5, 6}, {1, 6, 2}, // Right
    {5, 4, 7}, {5, 7, 6}, // Back
    {4, 0, 3}, {4, 3, 7}, // Left
    {3, 2, 6}, {3, 6, 7}, // Top
    {4, 5, 1}, {4, 1, 0}  // Bottom
};

// Fast Horizontal Line for the Triangle Filler
static void draw_hline(int x0, int x1, int y, uint8_t color) {
    if (y < 0 || y >= 240) return;
    if (x0 > x1) SWAP(x0, x1);
    if (x0 < 0) x0 = 0;
    if (x1 > 319) x1 = 319;

    int offset = y * 320;
    for (int x = x0; x <= x1; x++) {
        render_fb[offset + x] = color;
    }
}

// Integer-based Scanline Triangle Rasterizer
static void fill_triangle(Vec2 v0, Vec2 v1, Vec2 v2, uint8_t color) {
    // Sort vertices by Y
    if (v0.y > v1.y) { SWAP(v0.x, v1.x); SWAP(v0.y, v1.y); }
    if (v0.y > v2.y) { SWAP(v0.x, v2.x); SWAP(v0.y, v2.y); }
    if (v1.y > v2.y) { SWAP(v1.x, v2.x); SWAP(v1.y, v2.y); }

    int total_height = v2.y - v0.y;
    if (total_height == 0) return;

    for (int i = 0; i < total_height; i++) {
        int second_half = i > v1.y - v0.y || v1.y == v0.y;
        int segment_height = second_half ? v2.y - v1.y : v1.y - v0.y;
        if (segment_height == 0) continue;

        // Calculate X boundaries using purely integer interpolation
        int ax = v0.x + (v2.x - v0.x) * i / total_height;
        int bx = second_half
            ? v1.x + (v2.x - v1.x) * (i - (v1.y - v0.y)) / segment_height
            : v0.x + (v1.x - v0.x) * i / segment_height;

        draw_hline(ax, bx, v0.y + i, color);
    }
}

// ---------------------------------------------------------
// Main Render Function (Call this in your loop)
// ---------------------------------------------------------
void render_cube(alt_32 ax, alt_32 ay) {
    // 1. Clear the local framebuffer
    memset(render_fb, 0, sizeof(render_fb));

    // Convert accelerometer data to rotation angles
    float angle_y = ax * 0.005f;
    float angle_x = ay * 0.005f;

    Vec3 transformed[8];
    Vec2 projected[8];

    float sin_x = sin(angle_x), cos_x = cos(angle_x);
    float sin_y = sin(angle_y), cos_y = cos(angle_y);

    // 2. Rotate and Project Vertices
    for (int i = 0; i < 8; i++) {
        Vec3 v = base_vertices[i];

        // Rotate X
        float y1 = v.y * cos_x - v.z * sin_x;
        float z1 = v.y * sin_x + v.z * cos_x;
        // Rotate Y
        float x2 = v.x * cos_y + z1 * sin_y;
        float z2 = -v.x * sin_y + z1 * cos_y;

        // Push cube away from camera
        z2 += 3.5f;
        transformed[i] = (Vec3){x2, y1, z2};

        // Perspective Projection to 2D Screen (320x240)
        float fov = 150.0f;
        projected[i].x = (int)(x2 * fov / z2) + 160;
        projected[i].y = (int)(y1 * fov / z2) + 120;
    }

    // Light source coming from the camera
    Vec3 light = {0.0f, 0.0f, -1.0f};

    // 3. Calculate Faces, Shading, and Render
    for (int i = 0; i < 12; i++) {
        Vec3 t0 = transformed[faces[i][0]];
        Vec3 t1 = transformed[faces[i][1]];
        Vec3 t2 = transformed[faces[i][2]];

        // Calculate Surface Normal using Cross Product
        // N = (t1 - t0) x (t2 - t0)
        Vec3 line1 = {t1.x - t0.x, t1.y - t0.y, t1.z - t0.z};
        Vec3 line2 = {t2.x - t0.x, t2.y - t0.y, t2.z - t0.z};

        Vec3 normal = {
            line1.y * line2.z - line1.z * line2.y,
            line1.z * line2.x - line1.x * line2.z,
            line1.x * line2.y - line1.y * line2.x
        };

        // Normalize the vector
        float length = sqrt(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
        normal.x /= length; normal.y /= length; normal.z /= length;

        // Dot product with light source (Flat Shading)
        float dot = normal.x * light.x + normal.y * light.y + normal.z * light.z;

        // Back-face Culling: Only draw if facing the camera (dot > 0)
        if (dot > 0.0f) {
            // Generate an 8-bit green color shade based on light intensity
            // 8-bit RGB332 format: Green is controlled by bits 2, 3, and 4.
            int intensity = (int)(dot * 7.0f); // 0 to 7
            if (intensity < 1) intensity = 1;  // Ambient light minimum
            uint8_t color = (intensity << 2);

            fill_triangle(
                projected[faces[i][0]],
                projected[faces[i][1]],
                projected[faces[i][2]],
                color
            );
        }
    }

    // 4. Fast 32-bit copy from local array to Shared SDRAM
    uint32_t* src = (uint32_t*)render_fb;
    uint32_t* dst = (uint32_t*)0x01E00000; // Your DRAM_WRADDR
    for (int i = 0; i < (320 * 240) / 4; i++) {
        dst[i] = src[i];
    }
}

static void handle_rx_interrupts(void* context, alt_u32 id) {
    volatile int* edge_capture_ptr = (volatile int*) context;
    int edge = IORD_ALTERA_AVALON_PIO_EDGE_CAP(IRQ_CORE0_RX_BASE);
    *edge_capture_ptr = edge;
    rx_buffer = edge;
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(IRQ_CORE0_RX_BASE, edge);

    if (edge & 0x00007000) {
        cube_mode = !cube_mode;
        // If cube just enabled, trigger immediate render using last known accel
        if (cube_mode) rx_buffer = 0x00008000;
        // If cube just disabled: do NOT send fake frame pulse.
        // Core 2's 0x00007000 handler already resets is_cube and force_redraws.
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

#define TASK_STACKSIZE 1024
OS_STK accel_task_stk  [TASK_STACKSIZE];
OS_STK cpu_monitor_task_stk[TASK_STACKSIZE];
OS_STK startup_task_stk[TASK_STACKSIZE];
#define ACCEL_PRIORITY 1
#define CPU_MONITOR_PRIORITY 2

void cpu_monitor_task(void* pdata) {
    while (1) {
        IOWR_32DIRECT(CORE0_WRADDR, 0, OSCPUUsage);
        OSTimeDlyHMSM(0, 0, 0, 500);
    }
}

void accel_task(void* pdata) {
    INT8U err;
    while (1) {
        OSSemPend(rx_trigger_sem, 1, &err);

        if (err == OS_ERR_NONE) {
            if (rx_buffer & 0x00008000) {
                int32_t received_x = (int32_t)IORD_32DIRECT(ACCE_WRADDR, 0);
                int32_t received_y = (int32_t)IORD_32DIRECT(ACCE_WRADDR, 4);
                int32_t received_z = (int32_t)IORD_32DIRECT(ACCE_WRADDR, 8);
                rx_buffer = 0x0;

                if (received_z < -208 && !emergency_stop) {
                    IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE0_TX_BASE, 0x00010001);
                    IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE0_TX_BASE, 0x00000000);
                    emergency_stop = 1;
                }
                if (received_z > -208 && emergency_stop) {
                    IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE0_TX_BASE, 0x00020002);
                    IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE0_TX_BASE, 0x00000000);
                    emergency_stop = 0;
                }

                last_ax = (received_x != 0 || last_ax == 0) ? received_x : last_ax;
                last_ay = (received_y != 0 || last_ay == 0) ? received_y : last_ay;

                IOWR_32DIRECT(CORE0_WRADDR, 0, OSCPUUsage);
            }
        }

        if (cube_mode || IORD_8DIRECT(SETTINGS_WRADDR, 3)) {
            render_cube(last_ax, last_ay);
        }
    }
}

void startup_task(void* pdata) {
    OSStatInit();
    OSTaskCreateExt(accel_task, NULL, (void*)&accel_task_stk[TASK_STACKSIZE-1],
                    ACCEL_PRIORITY, ACCEL_PRIORITY, accel_task_stk, TASK_STACKSIZE, NULL, 0);
    // NEW: Create the CPU monitor task
    OSTaskCreateExt(cpu_monitor_task, NULL, (void*)&cpu_monitor_task_stk[TASK_STACKSIZE-1],
                    CPU_MONITOR_PRIORITY, CPU_MONITOR_PRIORITY, cpu_monitor_task_stk, TASK_STACKSIZE, NULL, 0);
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
