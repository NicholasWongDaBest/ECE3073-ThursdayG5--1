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
#include <stdlib.h>

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
#define ABS(x) ((x) < 0 ? -(x) : (x))

// Framebuffer in local memory (320x240)
static uint8_t render_fb[320 * 240];

typedef struct { float x, y, z; } Vec3;
typedef struct { int x, y; } Vec2;

#define PEBBLE_R 0.13f
#define NUM_PEBBLES 15
typedef struct { float x, y, z; float vx, vy, vz; } Pebble;
Pebble pebbles[NUM_PEBBLES];
int pebbles_initialized = 0;

// Auto-spin state variables
static float auto_spin_x = 0.0f;
static float auto_spin_y = 0.0f;

// Define your spin increments per frame
#define SPIN_SPEED_X 1.5f
#define SPIN_SPEED_Y 2.0f

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

static inline float Q_rsqrt(float number) {
    union { float f; uint32_t i; } conv = { number };
    conv.i  = 0x5f3759df - (conv.i >> 1);
    conv.f *= 1.5f - (number * 0.5f * conv.f * conv.f);
    return conv.f;
}

// Bresenham's Line Algorithm for Sharp Outlines
static void draw_line(int x0, int y0, int x1, int y1, uint8_t color) {
    int dx = ABS(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -ABS(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    while (1) {
        if (x0 >= 0 && x0 < 320 && y0 >= 0 && y0 < 240) {
            render_fb[y0 * 320 + x0] = color;
        }
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

// Fast Horizontal Scanline with 8.8 Fixed-Point Shading & Dithered Transparency
static void draw_hline_gouraud(int x0, int x1, int y, int c0, int c1, int is_backface) {
    if (y < 0 || y >= 240) return;
    if (x0 > x1) {
        SWAP(x0, x1);
        SWAP(c0, c1);
    }

    if (x0 < 0) {
        int dx = x1 - x0;
        if (dx > 0) {
            int slope_c = (c1 - c0) / dx;
            c0 += slope_c * (0 - x0);
        }
        x0 = 0;
    }
    if (x1 > 319) x1 = 319;
    if (x0 > x1) return;

    int dx = x1 - x0;
    int slope_c = (dx > 0) ? (c1 - c0) / dx : 0;

    int offset = y * 320;
    int current_c = c0;

    for (int x = x0; x <= x1; x++) {
        int draw_pixel = 0;

        // Transparent Masking Grid Patterns
        if (is_backface) {
            // Sparser 25% dot pattern for background surfaces
            if (((x + y) & 1) == 0 && (x & 1) == 0) draw_pixel = 1;
        } else {
            // 50% checkerboard dither pattern for foreground surfaces
            if ((x + y) & 1) draw_pixel = 1;
        }

        if (draw_pixel) {
            int intensity = current_c >> 8;
            if (intensity < 1) intensity = 1;
            if (intensity > 7) intensity = 7;
            render_fb[offset + x] = (intensity << 2); // Map directly to RGB332 Green Channel bits
        }
        current_c += slope_c;
    }
}

// Scanline Triangle Rasterizer with Vertices Intensity Interpolations
static void fill_triangle_gouraud(Vec2 v0, Vec2 v1, Vec2 v2, int c0, int c1, int c2, int is_backface) {
    if (v0.y > v1.y) { SWAP(v0.x, v1.x); SWAP(v0.y, v1.y); SWAP(c0, c1); }
    if (v0.y > v2.y) { SWAP(v0.x, v2.x); SWAP(v0.y, v2.y); SWAP(c0, c2); }
    if (v1.y > v2.y) { SWAP(v1.x, v2.x); SWAP(v1.y, v2.y); SWAP(c1, c2); }

    int total_height = v2.y - v0.y;
    if (total_height == 0) return;

    int slope_x02 = ((v2.x - v0.x) << 8) / total_height;
    int slope_c02 = ((c2 - c0) << 8) / total_height;

    // ── Upper Section: v0 → v1 ───────────────────────────────────────────
    int seg_h1 = v1.y - v0.y;
    if (seg_h1 > 0) {
        int slope_x01 = ((v1.x - v0.x) << 8) / seg_h1;
        int slope_c01 = ((c1 - c0) << 8) / seg_h1;
        int ax = v0.x << 8;
        int bx = v0.x << 8;
        int ac = c0 << 8;
        int bc = c0 << 8;
        for (int y = v0.y; y < v1.y; y++) {
            draw_hline_gouraud(ax >> 8, bx >> 8, y, ac, bc, is_backface);
            ax += slope_x02; bx += slope_x01;
            ac += slope_c02; bc += slope_c01;
        }
    }

    // ── Lower Section: v1 → v2 ───────────────────────────────────────────
    int seg_h2 = v2.y - v1.y;
    if (seg_h2 > 0) {
        int slope_x12 = ((v2.x - v1.x) << 8) / seg_h2;
        int slope_c12 = ((c2 - c1) << 8) / seg_h2;
        int ax = (v0.x << 8) + slope_x02 * seg_h1;
        int bx = v1.x << 8;
        int ac = (c0 << 8) + slope_c02 * seg_h1;
        int bc = c1 << 8;
        for (int y = v1.y; y <= v2.y; y++) {
            draw_hline_gouraud(ax >> 8, bx >> 8, y, ac, bc, is_backface);
            ax += slope_x02; bx += slope_x12;
            ac += slope_c02; bc += slope_c12;
        }
    }
}

// ---------------------------------------------------------
// Main Render Function with Conditional Auto-Spin
// ---------------------------------------------------------
void render_cube(alt_32 ax, alt_32 ay, int auto_spin_enabled) {
    // 1. Static persistent rotation registers for the automatic spin path
    static float auto_angle_x = 0.0f;
    static float auto_angle_y = 0.0f;

    float angle_y;
    float angle_x;

    Vec3 transformed[8];
    Vec2 projected[8];
    int vertex_intensity[8];

    float sin_x, cos_x;
    float sin_y, cos_y;

    float gx, gy, gz;
    int p, q;
    int i, idx;
    float avg_z[12];
    int tri_indices[12];

    const int edges[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0},
        {4, 5}, {5, 6}, {6, 7}, {7, 4},
        {0, 4}, {1, 5}, {2, 6}, {3, 7}
    };

    // 2. Clear local memory frame buffer before executing math logic
    memset(render_fb, 0, sizeof(render_fb));

    // 3. SEPARATE ACCELEROMETER VS AUTOMATIC SPIN EXCLUSIVELY
    if (auto_spin_enabled) {
        // Mode A: Auto-spin is ON. Update and use ONLY the automatic angles.
        auto_angle_x += 0.015f;
        auto_angle_y += 0.025f;

        angle_x = auto_angle_x;
        angle_y = auto_angle_y;
    } else {
        // Mode B: Auto-spin is OFF. Freeze and reset auto registers.
        auto_angle_x = 0.0f;
        auto_angle_y = 0.0f;

        // Scale raw board metrics safely into an absolute tilt window (Radians bounds ~ -1.5 to +1.5)
        // This ensures if the board stays still, the cube completely stops spinning and holds position.
        angle_x = (float)ay * 0.005f;
        angle_y = (float)ax * 0.005f;
    }

    sin_x = sin(angle_x); cos_x = cos(angle_x);
    sin_y = sin(angle_y); cos_y = cos(angle_y);

    // --- PEBBLE PHYSICS ENGINE ---
    if (!pebbles_initialized) {
        for(p = 0; p < NUM_PEBBLES; p++) {
            pebbles[p].x = (float)(rand() % 160) / 100.0f - 0.8f;
            pebbles[p].y = (float)(rand() % 160) / 100.0f - 0.8f;
            pebbles[p].z = (float)(rand() % 160) / 100.0f - 0.8f;
            pebbles[p].vx = 0; pebbles[p].vy = 0; pebbles[p].vz = 0;
        }
        pebbles_initialized = 1;
    }

    // INVERTED gravity vectors to fix the accelerometer mapping
    gx = -sin_y * 0.02f;
    gy =  cos_x * cos_y * 0.02f;
    gz = -sin_x * 0.02f;

    for(p = 0; p < NUM_PEBBLES; p++) {
        // Apply gravity and drag
        pebbles[p].vx = (pebbles[p].vx + gx) * 0.98f;
        pebbles[p].vy = (pebbles[p].vy + gy) * 0.98f;
        pebbles[p].vz = (pebbles[p].vz + gz) * 0.98f;

        pebbles[p].x += pebbles[p].vx;
        pebbles[p].y += pebbles[p].vy;
        pebbles[p].z += pebbles[p].vz;

        // Wall collisions
        float bound = 1.0f - PEBBLE_R;
        float bounce = -0.65f;
        if(pebbles[p].x > bound)  { pebbles[p].x = bound;  pebbles[p].vx *= bounce; }
        if(pebbles[p].x < -bound) { pebbles[p].x = -bound; pebbles[p].vx *= bounce; }
        if(pebbles[p].y > bound)  { pebbles[p].y = bound;  pebbles[p].vy *= bounce; }
        if(pebbles[p].y < -bound) { pebbles[p].y = -bound; pebbles[p].vy *= bounce; }
        if(pebbles[p].z > bound)  { pebbles[p].z = bound;  pebbles[p].vz *= bounce; }
        if(pebbles[p].z < -bound) { pebbles[p].z = -bound; pebbles[p].vz *= bounce; }
    }

    // PARTICLE-TO-PARTICLE COLLISIONS
    for(p = 0; p < NUM_PEBBLES; p++) {
        for(q = p + 1; q < NUM_PEBBLES; q++) {
            float dx = pebbles[q].x - pebbles[p].x;
            float dy = pebbles[q].y - pebbles[p].y;
            float dz = pebbles[q].z - pebbles[p].z;

            float dist_sq = dx*dx + dy*dy + dz*dz;
            float min_dist = PEBBLE_R * 2.0f;

            if (dist_sq > 0.00001f && dist_sq < (min_dist * min_dist)) {
                float inv_dist = Q_rsqrt(dist_sq);
                float dist = dist_sq * inv_dist;

                float nx = dx * inv_dist;
                float ny = dy * inv_dist;
                float nz = dz * inv_dist;

                float overlap = 0.5f * (min_dist - dist);
                pebbles[p].x -= nx * overlap; pebbles[p].y -= ny * overlap; pebbles[p].z -= nz * overlap;
                pebbles[q].x += nx * overlap; pebbles[q].y += ny * overlap; pebbles[q].z += nz * overlap;

                float dvx = pebbles[q].vx - pebbles[p].vx;
                float dvy = pebbles[q].vy - pebbles[p].vy;
                float dvz = pebbles[q].vz - pebbles[p].vz;

                float dot = dvx*nx + dvy*ny + dvz*nz;

                if (dot < 0.0f) {
                    float restitution = 0.75f;
                    float impulse = -(1.0f + restitution) * dot * 0.5f;

                    pebbles[p].vx -= impulse * nx; pebbles[p].vy -= impulse * ny; pebbles[p].vz -= impulse * nz;
                    pebbles[q].vx += impulse * nx; pebbles[q].vy += impulse * ny; pebbles[q].vz += impulse * nz;
                }
            }
        }
    }

    // Rotate and Project Vertices
    for (i = 0; i < 8; i++) {
        Vec3 v = base_vertices[i];

        float y1 = v.y * cos_x - v.z * sin_x;
        float z1 = v.y * sin_x + v.z * cos_x;
        float x2 = v.x * cos_y + z1 * sin_y;
        float z2 = -v.x * sin_y + z1 * cos_y;

        float nx = x2, ny = y1, nz = z2;
        float inv_vlen = Q_rsqrt(nx*nx + ny*ny + nz*nz);
        nx *= inv_vlen; ny *= inv_vlen; nz *= inv_vlen;

        float v_dot = -(nz);
        float intensity_factor = (v_dot + 1.0f) * 0.5f;
        int intensity = (int)(intensity_factor * 6.0f) + 1;
        vertex_intensity[i] = intensity;

        z2 += 3.5f;
        transformed[i] = (Vec3){x2, y1, z2};

        float fov = 150.0f;
        projected[i].x = (int)(x2 * fov / z2) + 160;
        projected[i].y = (int)(y1 * fov / z2) + 120;
    }

    // Painters sorting array
    for (i = 0; i < 12; i++) {
        tri_indices[i] = i;
        avg_z[i] = (transformed[faces[i][0]].z + transformed[faces[i][1]].z + transformed[faces[i][2]].z) * 0.3333f;
    }

    for (i = 0; i < 11; i++) {
        for (q = i + 1; q < 12; q++) {
            if (avg_z[tri_indices[i]] < avg_z[tri_indices[q]]) {
                int temp = tri_indices[i];
                tri_indices[i] = tri_indices[q];
                tri_indices[q] = temp;
            }
        }
    }

    // Rasterize facets
    for (idx = 0; idx < 12; idx++) {
        i = tri_indices[idx];

        Vec3 t0 = transformed[faces[i][0]];
        Vec3 t1 = transformed[faces[i][1]];
        Vec3 t2 = transformed[faces[i][2]];

        Vec3 line1 = {t1.x - t0.x, t1.y - t0.y, t1.z - t0.z};
        Vec3 line2 = {t2.x - t0.x, t2.y - t0.y, t2.z - t0.z};
        Vec3 normal = {
            line1.y * line2.z - line1.z * line2.y,
            line1.z * line2.x - line1.x * line2.z,
            line1.x * line2.y - line1.y * line2.x
        };

        float inv_len = Q_rsqrt(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
        normal.z *= inv_len;

        int is_backface = ((-normal.z) <= 0.0f);

        int c0 = vertex_intensity[faces[i][0]];
        int c1 = vertex_intensity[faces[i][1]];
        int c2 = vertex_intensity[faces[i][2]];

        fill_triangle_gouraud(projected[faces[i][0]], projected[faces[i][1]], projected[faces[i][2]], c0, c1, c2, is_backface);
    }

    // Render pebbles
    for(p = 0; p < NUM_PEBBLES; p++) {
        float px = pebbles[p].x;
        float py = pebbles[p].y;
        float pz = pebbles[p].z;

        float y1 = py * cos_x - pz * sin_x;
        float z1 = py * sin_x + pz * cos_x;
        float x2 = px * cos_y + z1 * sin_y;
        float z2 = -px * sin_y + z1 * cos_y;

        z2 += 3.5f;

        float fov = 150.0f;
        int screen_x = (int)(x2 * fov / z2) + 160;
        int screen_y = (int)(y1 * fov / z2) + 120;

        int dy, dx;

        uint8_t pcolor = 0xE0;
        int p_rad = 4;

        for(dy = -p_rad; dy <= p_rad; dy++) {
            for(dx = -p_rad; dx <= p_rad; dx++) {
                if (dx*dx + dy*dy <= p_rad*p_rad) {     // ← circle test
                    int draw_x = screen_x + dx;
                    int draw_y = screen_y + dy;
                    if (draw_x >= 0 && draw_x < 320 && draw_y >= 0 && draw_y < 240) {
                        render_fb[draw_y * 320 + draw_x] = pcolor;
                    }
                }
            }
        }
    }

    // Wireframe lines
    for (i = 0; i < 12; i++) {
        Vec2 p0 = projected[edges[i][0]];
        Vec2 p1 = projected[edges[i][1]];
        draw_line(p0.x, p0.y, p1.x, p1.y, (7 << 2));
    }

    // Fast copy from local array to Shared SDRAM Framebuffer
    // ONLY push to SDRAM if cube page is active — don't clobber camera
    int cube_page = (int)IORD_8DIRECT((alt_u32)SETTINGS_WRADDR, 3);
    if (cube_page) {
        uint32_t* src = (uint32_t*)render_fb;
        uint32_t* dst = (uint32_t*)0x01E00000;
        for (i = 0; i < (320 * 240) / 4; i++) {
            dst[i] = src[i];
        }
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

#define TASK_STACKSIZE 1024
OS_STK accel_task_stk  [TASK_STACKSIZE];
OS_STK cpu_monitor_task_stk[TASK_STACKSIZE];
OS_STK startup_task_stk[TASK_STACKSIZE];
#define ACCEL_PRIORITY 1
#define CPU_MONITOR_PRIORITY 2

void cpu_monitor_task(void* pdata) {
    while (1) {
        INT8U usage = (OSCPUUsage > 100) ? 100 : (INT8U)OSCPUUsage;
        IOWR_32DIRECT(CORE0_WRADDR, 0, usage);
        OSTimeDlyHMSM(0, 0, 0, 500);
    }
}

//void accel_task(void* pdata) {
//    INT8U err;
//    while (1) {
//        OSSemPend(rx_trigger_sem, 1, &err);
//
//        if (err == OS_ERR_NONE) {
//            if (rx_buffer & 0x00008000) {
//                int32_t received_x = (int32_t)IORD_32DIRECT(ACCE_WRADDR, 0);
//                int32_t received_y = (int32_t)IORD_32DIRECT(ACCE_WRADDR, 4);
//                int32_t received_z = (int32_t)IORD_32DIRECT(ACCE_WRADDR, 8);
//                rx_buffer = 0x0;
//
//                if (received_z < -208 && !emergency_stop) {
//                    IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE0_TX_BASE, 0x00010001);
//                    IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE0_TX_BASE, 0x00000000);
//                    emergency_stop = 1;
//                }
//                if (received_z > -208 && emergency_stop) {
//                    IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE0_TX_BASE, 0x00020002);
//                    IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE0_TX_BASE, 0x00000000);
//                    emergency_stop = 0;
//                }
//
//                last_ax = (received_x != 0 || last_ax == 0) ? received_x : last_ax;
//                last_ay = (received_y != 0 || last_ay == 0) ? received_y : last_ay;
//
//                IOWR_32DIRECT(CORE0_WRADDR, 0, OSCPUUsage);
//            }
//        }
//
//        if (cube_mode || IORD_8DIRECT(SETTINGS_WRADDR, 3)) {
//            render_cube(last_ax, last_ay);
//        }
//    }
//}

void accel_task(void* pdata) {
    INT8U err;
    while (1) {
        OSSemPend(rx_trigger_sem, 20, &err);

        // Always read accelerometer every iteration, not just on interrupt flag
        int32_t received_x = (int32_t)IORD_32DIRECT(ACCE_WRADDR, 0);
        int32_t received_y = (int32_t)IORD_32DIRECT(ACCE_WRADDR, 4);
        int32_t received_z = (int32_t)IORD_32DIRECT(ACCE_WRADDR, 8);

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

        INT8U usage = (OSCPUUsage > 100) ? 100 : (INT8U)OSCPUUsage;
        IOWR_32DIRECT(CORE0_WRADDR, 0, usage);
        rx_buffer = 0x0;

        int spin_on   = (int)IORD_8DIRECT((alt_u32)SETTINGS_WRADDR, 1); // auto-spin toggle
        int cube_page = (int)IORD_8DIRECT((alt_u32)SETTINGS_WRADDR, 3); // page 4 active

        if (cube_mode || cube_page) {
            render_cube(last_ax, last_ay, spin_on);
        }
    }
}

void startup_task(void* pdata) {
    // 1. Run OS calibration FIRST, in complete silence
    OSStatInit();

    // 2. Enable the hardware interrupts ONLY AFTER calibration is done!
    init_rx();

    OSTaskCreateExt(accel_task, NULL, (void*)&accel_task_stk[TASK_STACKSIZE-1],
                    ACCEL_PRIORITY, ACCEL_PRIORITY, accel_task_stk, TASK_STACKSIZE, NULL, 0);
    OSTaskCreateExt(cpu_monitor_task, NULL, (void*)&cpu_monitor_task_stk[TASK_STACKSIZE-1],
                    CPU_MONITOR_PRIORITY, CPU_MONITOR_PRIORITY, cpu_monitor_task_stk, TASK_STACKSIZE, NULL, 0);
    OSTaskDel(OS_PRIO_SELF);
}

int main(void) {
    alt_printf("=========================================================\n");
    alt_printf("CPU Core 0 Alive\n");
    alt_printf("=========================================================\n");

    rx_trigger_sem = OSSemCreate(0);

    // init_rx(); <--- REMOVE THIS LINE FROM MAIN

    OSTaskCreateExt(startup_task, NULL, (void*)&startup_task_stk[TASK_STACKSIZE-1],
                    10, 10, startup_task_stk, TASK_STACKSIZE, NULL, 0);  // was: 0, 0
    OSStart();
    return 0;
}
