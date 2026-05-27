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
static int settings_spin   = 0;
#define NUM_PAGES 6

static int last_page    = -1;
static int force_redraw =  1;

static uint8_t cube_buf[320 * 240];

static void init_rx();
static void handle_rx_interrupts(void* context, alt_u32 id);
static void draw_ui_page(int page);
static void draw_ui_page_partial(void);
void write_pixel_array(const uint8_t* ADDRESS);

static void set_cube_page(int active) {
    IOWR_8DIRECT(SETTINGS_WRADDR, 3, active ? 1 : 0);
}

static void set_camera_page(int active) {
    IOWR_8DIRECT(SETTINGS_WRADDR, 2, active ? 1 : 0);
}

static void init_rx() {
    void* p = (void*) &edge_capture;
    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(IRQ_CORE2_RX_BASE, 0xFFFFFFFF);
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(IRQ_CORE2_RX_BASE, 0);
    alt_irq_register(IRQ_CORE2_RX_IRQ, p, handle_rx_interrupts);
}

static void handle_rx_interrupts(void* context, alt_u32 id) {
    volatile int* edge_capture_ptr = (volatile int*) context;
    int edge = IORD_ALTERA_AVALON_PIO_EDGE_CAP(IRQ_CORE2_RX_BASE);
    if (edge & 0x00000001) emergency_stop = 1;
    if (edge & 0x00000002) emergency_stop = 0;
    *edge_capture_ptr = edge;
    if (edge & 0x80000000) { rx_buffer |= 0x80000000; OSSemPost(vga_trigger_sem); }
    if (edge & 0x40000000) { is_spi=1; is_cube=0; OSSemPost(vga_trigger_sem); }
    if (edge & 0x08000000) { is_spi=0; OSSemPost(vga_trigger_sem); }
    if (edge & 0x00007000) { is_cube=0; rx_buffer&=~0x00000004; force_redraw=1; OSSemPost(vga_trigger_sem); }
    if (edge & 0x10000000) { key0_click=1; OSSemPost(vga_trigger_sem); }
    if (edge & 0x20000000) { key1_click=1; OSSemPost(vga_trigger_sem); }
    if (edge & 0x00000004) { rx_buffer|=0x00000004; is_cube=1; OSSemPost(vga_trigger_sem); }
    else if (edge & 0x00000003) { OSSemPost(vga_trigger_sem); }
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(IRQ_CORE2_RX_BASE, edge);
}

// ============================================================================
// FONT
// ============================================================================
static const char UI_FONT_CHARS[] = " -:./0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ%<>";
static const uint8_t UI_FONT_DATA[44][6] = {
    {0x0,0x0,0x0,0x0,0x0,0x0},{0x0,0x0,0xF,0x0,0x0,0x0},{0x0,0x6,0x6,0x0,0x6,0x6},
	{0x1, 0x2, 0x2, 0x4, 0x4, 0x8},{0x1,0x2,0x4,0x8,0x4,0x2},{0x6,0x9,0x9,0x9,0x9,0x6},
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

#define C_BLACK   0x00   // background
#define C_WHITE   0xFF
#define C_YELLOW  0xFC   // logo accent / highlight
#define C_CYAN    0x1F   // page accent / active
#define C_GREEN   0x1C   // OK / positive
#define C_RED     0xE0   // error / negative
#define C_ORANGE  0xF4   // warning / key label
#define C_BLUE    0x03   // panel backgrounds
#define C_PURPLE  0x83   // inference panel accent
#define C_TEAL    0x17   // subtle accent
#define C_LGREY   0xB6   // body text
#define C_MGREY   0x92   // subdued text
#define C_DGREY   0x49   // dividers / borders
#define C_DBLUE   0x01   // dark card background
#define C_DCYAN   0x0A   // dark teal card
#define C_CARD    0x12   // card fill colour

// ============================================================================
// ============================================================================
// F1 THEME COLOUR PALETTE (RGB332)
// ============================================================================
#define C_BLACK    0x00   // Carbon Fiber / Deep Asphalt
#define C_WHITE    0xFF   // Primary Telemetry Text
#define C_F1_RED   0xE0   // F1 Brand Red / Redline Accents
#define C_YELLOW   0xFC   // Safety Car (SC) / Warnings
#define C_ORANGE   0xF4   // McLaren Papaya
#define C_GREEN    0x1C   // DRS Active / Sector Green
#define C_TEAL     0x1E   // Mercedes Petronas Teal
#define C_LGREY    0xB6   // Silver / Secondary Text
#define C_MGREY    0x92   // Inactive Text
#define C_DGREY    0x49   // Dark Asphalt / Borders
#define C_CARD     0x24   // Dark Grey Panel Fill (Replaces Blue)

// ============================================================================
// ULTRA-MODERN MOTORSPORT PALETTE (RGB332)
// ============================================================================
#define C_BLACK      0x00  // Pure Black Canvas
#define C_WHITE      0xFF  // Primary Data Readings
#define C_RED        0xE0  // F1 Neon Red / Critical Alerts
#define C_GREEN      0x1C  // Sector Green / Optimal Range
#define C_TEAL       0x1E  // Petronas Teal / System Active
#define C_YELLOW     0xFC  // Safety Flag / Attention
#define C_ORANGE     0xF4  // McLaren Papaya / Mode Selection
#define C_VAL_GREY   0xD6  // Crisp Silver for Mid-level labels
#define C_WIRE       0x49  // Razor Thin Grid / Wireframe Lines
#define C_DIM_GREY   0x24  // Inactive Element Backgrounds

#define UI_W 320
#define UI_H 240

static uint8_t ui_buf[UI_W * UI_H];

// ============================================================================
// DRAW PRIMITIVES
// ============================================================================
static void ui_pixel(int x,int y,uint8_t c){
    if(x>=0&&x<UI_W&&y>=0&&y<UI_H) ui_buf[y*UI_W+x]=c;}
static void ui_fill(uint8_t c){memset(ui_buf,c,sizeof(ui_buf));}
static void ui_hline(int x,int y,int len,uint8_t c){
    for(int i=x;i<x+len&&i<UI_W;i++) if(y>=0&&y<UI_H) ui_buf[y*UI_W+i]=c;}
static void ui_vline(int x,int y,int len,uint8_t c){
    for(int i=y;i<y+len&&i<UI_H;i++) if(x>=0&&x<UI_W) ui_buf[i*UI_W+x]=c;}
static void ui_rect(int x,int y,int w,int h,uint8_t c){
    ui_hline(x,y,w,c);ui_hline(x,y+h-1,w,c);
    ui_vline(x,y,h,c);ui_vline(x+w-1,y,h,c);}
static void ui_fill_rect(int x,int y,int w,int h,uint8_t c){
    for(int r=y;r<y+h&&r<UI_H;r++) ui_hline(x,r,w,c);}

// Rounded-rect: corners replaced with a single pixel inset
static void ui_rrect(int x,int y,int w,int h,uint8_t border,uint8_t fill){
    if(fill!=0xFF) ui_fill_rect(x+1,y+1,w-2,h-2,fill);
    ui_hline(x+1,y,w-2,border); ui_hline(x+1,y+h-1,w-2,border);
    ui_vline(x,y+1,h-2,border); ui_vline(x+w-1,y+1,h-2,border);
}

static void ui_char(int x,int y,char c,uint8_t col){
    if(c>='a'&&c<='z') c=c-'a'+'A';
    const char* p=strchr(UI_FONT_CHARS,c); if(!p) return;
    const uint8_t* g=UI_FONT_DATA[p-UI_FONT_CHARS];
    for(int row=0;row<6;row++) for(int co=0;co<4;co++){
        if(!(g[row]&(0x8>>co))) continue;
        int px=x+co,py=y+row;
        if(px>=0&&px<UI_W&&py>=0&&py<UI_H) ui_buf[py*UI_W+px]=col;}}
static void ui_str(int x,int y,const char* s,uint8_t c){
    while(*s){ui_char(x,y,*s++,c);x+=5;}}
static void ui_char2x(int x,int y,char c,uint8_t col){
    if(c>='a'&&c<='z') c=c-'a'+'A';
    const char* p=strchr(UI_FONT_CHARS,c); if(!p) return;
    const uint8_t* g=UI_FONT_DATA[p-UI_FONT_CHARS];
    for(int row=0;row<6;row++) for(int co=0;co<4;co++){
        if(!(g[row]&(0x8>>co))) continue;
        for(int dy=0;dy<2;dy++) for(int dx=0;dx<2;dx++){
            int px=x+co*2+dx,py=y+row*2+dy;
            if(px>=0&&px<UI_W&&py>=0&&py<UI_H) ui_buf[py*UI_W+px]=col;}}}
static void ui_str2x(int x,int y,const char* s,uint8_t c){
    while(*s){ui_char2x(x,y,*s++,c);x+=10;}}

// Centred string helper
static void ui_str_cx(int cx,int y,const char* s,uint8_t c){
    int len=0; const char* p=s; while(*p++) len++;
    ui_str(cx - len*5/2, y, s, c);}

// Filled and hollow Bresenham circles
static void ui_fill_circle(int xc,int yc,int r,uint8_t c){
    int x=0,y=r,d=3-2*r;
    while(y>=x){
        ui_hline(xc-x,yc+y,2*x+1,c); ui_hline(xc-x,yc-y,2*x+1,c);
        ui_hline(xc-y,yc+x,2*y+1,c); ui_hline(xc-y,yc-x,2*y+1,c);
        x++; if(d>0){y--;d=d+4*(x-y)+10;}else{d=d+4*x+6;}}}
static void ui_circle(int xc,int yc,int r,uint8_t c){
    int x=0,y=r,d=3-2*r;
    while(y>=x){
        ui_pixel(xc+x,yc+y,c);ui_pixel(xc-x,yc+y,c);
        ui_pixel(xc+x,yc-y,c);ui_pixel(xc-x,yc-y,c);
        ui_pixel(xc+y,yc+x,c);ui_pixel(xc-y,yc+x,c);
        ui_pixel(xc+y,yc-x,c);ui_pixel(xc-y,yc-x,c);
        x++; if(d>0){y--;d=d+4*(x-y)+10;}else{d=d+4*x+6;}}}

// CPU / score progress bar with accent stripe
static void ui_bar(int x,int y,int mw,int h,int val,uint8_t fg,uint8_t bg){
    int f=(val*mw)/100;
    ui_fill_rect(x,y,mw,h,bg);
    ui_fill_rect(x,y,f,h,fg);}

// F1 RPM-style segmented progress bar (Green -> Yellow -> Red -> Teal flash)
static void ui_rev_bar(int x, int y, int mw, int h, int val) {
    ui_fill_rect(x, y, mw, h, C_DGREY); // Background track
    int f = (val * mw) / 100;

    // Draw segmented vertical LEDs
    for(int i = 0; i < f; i++) {
        uint8_t col = C_GREEN;          // Low RPM
        if(i > mw * 0.6) col = C_YELLOW;// Mid RPM
        if(i > mw * 0.8) col = C_F1_RED;// High RPM/Redline
        if(i > mw * 0.95) col = C_TEAL; // Shift Flash!

        // Skip every 4th pixel to create individual "LED" segments
        if (i % 4 != 3) {
            ui_vline(x + i, y, h, col);
        }
    }
}

// Minimalist Integrated Shift Light Bar (Continuous Segment Array)
static void ui_render_shift_strip(int x, int y, int w, int h, int percent) {
    // Structural wireframe housing
    ui_rect(x, y, w, h, C_WIRE);

    int fill_width = (percent * (w - 2)) / 100;
    if (fill_width <= 0) return;

    for (int i = 0; i < fill_width; i++) {
        uint8_t segment_color = C_GREEN;
        // Accurate F1 rev progression color map
        if (i > (w * 50) / 100)  segment_color = C_YELLOW;
        if (i > (w * 75) / 100)  segment_color = C_RED;
        if (i > (w * 92) / 100)  segment_color = C_TEAL; // Shift Point Flash

        ui_vline(x + 1 + i, y + 1, h - 2, segment_color);
    }
}

// ============================================================================
// PIXEL WRITE
// ============================================================================
void write_pixel_color(int* ADDRESS){
    int img_addr=0;
    for(int i=0;i<19200;i++){
        uint32_t word=IORD_32DIRECT(ADDRESS,i*4);
        IOWR_ALTERA_AVALON_PIO_DATA(IMG_ADDR_BASE,img_addr++);IOWR_ALTERA_AVALON_PIO_DATA(PIXEL_BASE,(uint8_t)(word>>24));IOWR_ALTERA_AVALON_PIO_DATA(WREN_BASE,1);IOWR_ALTERA_AVALON_PIO_DATA(WREN_BASE,0);
        IOWR_ALTERA_AVALON_PIO_DATA(IMG_ADDR_BASE,img_addr++);IOWR_ALTERA_AVALON_PIO_DATA(PIXEL_BASE,(uint8_t)(word>>16));IOWR_ALTERA_AVALON_PIO_DATA(WREN_BASE,1);IOWR_ALTERA_AVALON_PIO_DATA(WREN_BASE,0);
        IOWR_ALTERA_AVALON_PIO_DATA(IMG_ADDR_BASE,img_addr++);IOWR_ALTERA_AVALON_PIO_DATA(PIXEL_BASE,(uint8_t)(word>>8)); IOWR_ALTERA_AVALON_PIO_DATA(WREN_BASE,1);IOWR_ALTERA_AVALON_PIO_DATA(WREN_BASE,0);
        IOWR_ALTERA_AVALON_PIO_DATA(IMG_ADDR_BASE,img_addr++);IOWR_ALTERA_AVALON_PIO_DATA(PIXEL_BASE,(uint8_t)word);      IOWR_ALTERA_AVALON_PIO_DATA(WREN_BASE,1);IOWR_ALTERA_AVALON_PIO_DATA(WREN_BASE,0);
    }}
void write_pixel_array(const uint8_t* ADDRESS){
    for(int i=0;i<UI_W*UI_H;i++){
        IOWR_ALTERA_AVALON_PIO_DATA(IMG_ADDR_BASE,i);
        IOWR_ALTERA_AVALON_PIO_DATA(PIXEL_BASE,ADDRESS[i]);
        IOWR_ALTERA_AVALON_PIO_DATA(WREN_BASE,1);IOWR_ALTERA_AVALON_PIO_DATA(WREN_BASE,0);}}
static void write_region(int x,int y,int w,int h){
    for(int row=y;row<y+h&&row<UI_H;row++)
        for(int col=x;col<x+w&&col<UI_W;col++){
            int idx=row*UI_W+col;
            IOWR_ALTERA_AVALON_PIO_DATA(IMG_ADDR_BASE,idx);
            IOWR_ALTERA_AVALON_PIO_DATA(PIXEL_BASE,ui_buf[idx]);
            IOWR_ALTERA_AVALON_PIO_DATA(WREN_BASE,1);IOWR_ALTERA_AVALON_PIO_DATA(WREN_BASE,0);}}

// ============================================================================
// SHARED HEADER  — gradient-style title bar with page tabs
// ============================================================================
// ============================================================================
// SHARED HEADER  — F1 Telemetry Banner
// ============================================================================
// ============================================================================
// WIREFRAME TIMING HEADER
// ============================================================================
static void ui_draw_header(int page) {
    // Clean upper border split
    ui_hline(0, 0, UI_W, C_WIRE);
    ui_hline(0, 16, UI_W, C_WIRE);

    // Left Identity
    ui_str(6, 4, "LIVE TELEMETRY", C_VAL_GREY);
    ui_vline(100, 1, 14, C_WIRE);

    // Center Dynamic Screen Focus
    const char* pages[] = {"SYS_CORE_DASH", "PIT_WALL_", "CTRL_CONFIG", "CAM_ONBOARD", "AERO_3D_MODEL", "TEAM_MARKET_PRICE"};
    ui_str(110, 4, pages[page], C_WHITE);

    // Right Status Identifier Block
    char pos_str[8]; snprintf(pos_str, sizeof(pos_str), "MODE %02d", page + 1);
    ui_str(UI_W - 42, 4, pos_str, C_ORANGE);
}

// ============================================================================
// SHARED STATUS BAR  — pill-style indicators
// ============================================================================
// ============================================================================
// SHARED STATUS BAR  — Steering Wheel Dash Indicators
// ============================================================================
// ============================================================================
// DASHBOARD BOTTOM FLAG MATRIX
// ============================================================================
static void ui_draw_status(void) {
    ui_hline(0, UI_H - 16, UI_W, C_WIRE);
    ui_fill_rect(0, UI_H - 15, UI_W, 15, C_BLACK);

    struct { const char* flag; int state; uint8_t color; } telemetry_flags[] = {
        {"DRS",  is_spi,               C_GREEN},
        {"ERS",  (current_page == 4),  C_TEAL},
        {"FLG",  emergency_stop,       C_YELLOW},
        {"BIAS", settings_rot,         C_ORANGE}
    };

    int current_x = 6;
    for (int i = 0; i < 4; i++) {
        if (telemetry_flags[i].state) {
            // Bright solid fill when flag is tripped
            ui_fill_rect(current_x, UI_H - 13, 28, 11, telemetry_flags[i].color);
            ui_str(current_x + 4, UI_H - 11, telemetry_flags[i].flag, C_BLACK);
        } else {
            // Understated wireframe outline when idle
            ui_rect(current_x, UI_H - 13, 28, 11, C_WIRE);
            ui_str(current_x + 4, UI_H - 11, telemetry_flags[i].flag, C_DIM_GREY);
        }
        current_x += 34;
    }

    // Right aligned team radio notification channel
    ui_str(UI_W - 75, UI_H - 11, "RADIO STABLE", C_VAL_GREY);
}

// ============================================================================
// PAGE 0 — DASHBOARD
// Layout: left 116px = live data cards, right 200px = controls list
// ============================================================================
// ============================================================================
// PAGE 0 — TELEMETRY (DASHBOARD)
// ============================================================================
// ============================================================================
// RACING DATA MODULE — MAIN TELEMETRY INDEX
// ============================================================================
static void draw_page_dashboard(void) {
    char data_string[32];
    int core_0 = IORD_32DIRECT(CORE0_WRADDR, 0);
    int core_1 = IORD_32DIRECT(CORE1_WRADDR, 0);
    int core_2 = IORD_32DIRECT(CORE2_WRADDR, 0);
    int acc_x  = (int32_t)IORD_32DIRECT(ACCE_WRADDR, 0);
    int acc_y  = (int32_t)IORD_32DIRECT(ACCE_WRADDR, 4);
    int acc_z  = (int32_t)IORD_32DIRECT(ACCE_WRADDR, 8);

    // Clear main viewport to deep asphalt black
    ui_fill_rect(0, 17, UI_W, UI_H - 33, C_BLACK);

    // ── LEFT COLUMN: MGU COMPONENT SYSTEM LOADS ─────────────────────────────
    int segment_y = 24;
    ui_str(6, segment_y, "POWERTRAIN THERMALS", C_ORANGE);

    int core_metrics[] = {core_0, core_1, core_2};
    const char* component_tags[] = {"CORE 0", "CORE 1", "CORE 2"};

    for (int i = 0; i < 3; i++) {
        int item_y = segment_y + 14 + (i * 18);

        // Component label string
        ui_str(6, item_y, component_tags[i], C_VAL_GREY);

        // Linear dashboard telemetry strip
        ui_render_shift_strip(48, item_y + 1, 64, 6, core_metrics[i]);

        // Micro scale value readout
        snprintf(data_string, sizeof(data_string), "%3d%%", core_metrics[i]);
        ui_str(116, item_y, data_string, C_WHITE);
    }

    // ── LEFT COLUMN LOWER: PRECISION G-FORCE CROSSHAIR ──────────────────────
    int g_base_y = 88;
    ui_hline(0, g_base_y, 142, C_WIRE);
    ui_str(6, g_base_y + 6, "LATERAL DYNAMICS", C_TEAL);

    // Render Clean Vector Target Crosshair
    int cross_center_x = 36;
    int cross_center_y = g_base_y + 36;
    ui_rect(cross_center_x - 24, cross_center_y - 24, 48, 48, C_DIM_GREY);
    ui_hline(cross_center_x - 20, cross_center_y, 40, C_WIRE);
    ui_vline(cross_center_x, cross_center_y - 20, 40, C_WIRE);

    // Dynamic Tracking Indicator Node
    int vector_x = cross_center_x + (acc_x * 20 / 512);
    int vector_y = cross_center_y - (acc_y * 20 / 512);
    // Boundary structural clamping loops
    if (vector_x < cross_center_x - 22) vector_x = cross_center_x - 22;
    if (vector_x > cross_center_x + 22) vector_x = cross_center_x + 22;
    if (vector_y < cross_center_y - 22) vector_y = cross_center_y - 22;
    if (vector_y > cross_center_y + 22) vector_y = cross_center_y + 22;

    ui_fill_rect(vector_x - 2, vector_y - 2, 5, 5, C_RED);

    // Readout values directly adjacent to crosshair visualization
    int numerical_text_x = 72;
    snprintf(data_string, sizeof(data_string), "X: %+.2fG", (float)acc_x / 256.0f);
    ui_str(numerical_text_x, cross_center_y - 14, data_string, C_WHITE);
    snprintf(data_string, sizeof(data_string), "Y: %+.2fG", (float)acc_y / 256.0f);
    ui_str(numerical_text_x, cross_center_y - 2,  data_string, C_WHITE);
    snprintf(data_string, sizeof(data_string), "Z: %+.2fG", (float)acc_z / 256.0f);
    ui_str(numerical_text_x, cross_center_y + 10, data_string, C_VAL_GREY);

    // ── RIGHT COLUMN: RACING MATRIX MAP & INPUT SCHEDULER ───────────────────
    int right_column_x = 148;
    ui_vline(right_column_x, 17, UI_H - 33, C_WIRE);

    ui_str(right_column_x + 8, 24, "STEERING WHEEL MAP", C_VAL_GREY);
    ui_hline(right_column_x, 34, UI_W - right_column_x, C_WIRE);

    struct { const char* key_label; const char* operational_tag; uint8_t text_color; } mappings[] = {
        {"DOWNSHIFT", "PREV MODE (KEY0)",  C_VAL_GREY},
        {"UPSHIFT",   "NEXT MODE (KEY1)",  C_WHITE},
        {"MODE_05",  "AERO SIMULATE", C_GREEN},
        {"MODE_06",  "DELTA UPDATE",  C_TEAL},
        {"RADIO",    "COMMS (SW4)",   // Active highlighted system setting
#if 1
                                     C_ORANGE
#endif
        },
        {"ROTARY",   "LOCATION",   C_YELLOW},
        {"WIPER",    "RESET DISP (SW10)",   C_RED},
        {"MODE_04",  "VISOR CAM",  C_VAL_GREY}
    };

    for (int i = 0; i < 8; i++) {
        int list_item_y = 40 + (i * 17);

        // Stripped row separation lines for an open architectural UI matrix look
        if (i > 0) {
            ui_hline(right_column_x + 4, list_item_y - 3, UI_W - right_column_x - 8, C_DIM_GREY);
        }

        ui_str(right_column_x + 8,  list_item_y, mappings[i].key_label,        C_WHITE);
        ui_str(right_column_x + 64, list_item_y, mappings[i].operational_tag,   mappings[i].text_color);
    }
}

// ============================================================================
// PAGE 1 — INFERENCE
// Top half: detected word (large) + top scores strip
// Bottom half: A-Z and 0-9 bar chart
// ============================================================================
static void draw_page_inference(void) {
    uint8_t alpha_s[26], num_s[10];
    for(int i=0;i<26;i++) alpha_s[i]=IORD_8DIRECT(SCORE_WRADDR,i);
    for(int i=0;i<10;i++) num_s[i]  =IORD_8DIRECT(SCORE_WRADDR,26+i);

    // ── Word display card ──────────────────────────────────────────────────
    ui_fill_rect(0, 20, UI_W, 44, C_DBLUE);
    ui_hline(0, 20, UI_W, C_RED);
    ui_hline(0, 21, UI_W, C_RED);
    ui_str(4, 23, "DETECTED WORD", C_YELLOW);

    char wbuf[17]={0};
    for(int i=0;i<16;i++) wbuf[i]=(char)IORD_8DIRECT(WORD_WRADDR,i);
    wbuf[16]='\0';
    int wlen=0; for(int i=0;i<16;i++) if(wbuf[i]>' ') wlen=i+1;

    if(wlen>0){
        // Render large word — centred
        int wx = UI_W/2 - wlen*10/2;
        if(wx<4) wx=4;
        ui_str2x(wx, 31, wbuf, C_YELLOW);
    } else {
        ui_str_cx(UI_W/2, 36, "---", C_DGREY);
    }

    // ── Top-3 score strip ─────────────────────────────────────────────────
    // Find top 3 overall (A-Z + 0-9)
    uint8_t top_score[3]={0}; char top_lbl[3]={'?','?','?'};
    for(int i=0;i<26;i++){
        if(alpha_s[i]>top_score[0]){top_score[2]=top_score[1];top_lbl[2]=top_lbl[1];
            top_score[1]=top_score[0];top_lbl[1]=top_lbl[0];
            top_score[0]=alpha_s[i];  top_lbl[0]='A'+i;}
        else if(alpha_s[i]>top_score[1]){top_score[2]=top_score[1];top_lbl[2]=top_lbl[1];
            top_score[1]=alpha_s[i];  top_lbl[1]='A'+i;}
        else if(alpha_s[i]>top_score[2]){top_score[2]=alpha_s[i];  top_lbl[2]='A'+i;}
    }
    for(int i=0;i<10;i++){
        if(num_s[i]>top_score[0]){top_score[2]=top_score[1];top_lbl[2]=top_lbl[1];
            top_score[1]=top_score[0];top_lbl[1]=top_lbl[0];
            top_score[0]=num_s[i];    top_lbl[0]='0'+i;}
        else if(num_s[i]>top_score[1]){top_score[2]=top_score[1];top_lbl[2]=top_lbl[1];
            top_score[1]=num_s[i];    top_lbl[1]='0'+i;}
        else if(num_s[i]>top_score[2]){top_score[2]=num_s[i];     top_lbl[2]='0'+i;}
    }

    // Three podium cards: gold / silver / bronze
    uint8_t medal_col[] = {C_YELLOW, C_LGREY, C_ORANGE};
    char sc_tmp[4];
    for(int i=0;i<3;i++){
        int bx = 4 + i*106;
        ui_rrect(bx, 66, 100, 28, medal_col[i], C_CARD);
        // rank label
        const char* rank_lbl[] = {"TOP","2ND","3RD"};
        ui_str(bx+4, 69, rank_lbl[i], medal_col[i]);
        // big letter
        ui_char2x(bx+32, 68, top_lbl[i], C_WHITE);
        // score
        snprintf(sc_tmp,sizeof(sc_tmp),"%3d",top_score[i]);
        ui_str(bx+62, 69, sc_tmp, medal_col[i]);
        // small bar
        ui_fill_rect(bx+2, 88, 96, 4, C_DGREY);
        ui_fill_rect(bx+2, 88, (top_score[i]*96)/100, 4, medal_col[i]);
    }

    // ── A-Z bar chart ──────────────────────────────────────────────────────
    ui_fill_rect(0, 98, UI_W, 4, C_DGREY);
    ui_str(4,  105, "A-Z", C_LGREY);
    ui_str(172,105, "0-9", C_LGREY);

    #define BAR_BASE  140
    #define BAR_H      22
    #define BAR_BW      5
    #define BAR_BG      1

    for(int i=0;i<26;i++){
        int bx=4+i*(BAR_BW+BAR_BG);
        int bh=(int)((uint32_t)alpha_s[i]*BAR_H/100);
        uint8_t col = alpha_s[i]>70 ? C_GREEN
                    : alpha_s[i]>40 ? C_CYAN
                    : alpha_s[i]>15 ? C_PURPLE
                    :                 C_DGREY;
        ui_fill_rect(bx,BAR_BASE-BAR_H,BAR_BW,BAR_H,C_DGREY);
        if(bh>0) ui_fill_rect(bx,BAR_BASE-bh,BAR_BW,bh,col);
        // top cap on active bars
        if(bh>1) ui_hline(bx,BAR_BASE-bh,BAR_BW,C_WHITE);
        ui_char(bx,BAR_BASE+2,'A'+i, alpha_s[i]>15?col:C_MGREY);
    }
    int nx0=4+26*(BAR_BW+BAR_BG)+6;
    for(int i=0;i<10;i++){
        int bx=nx0+i*(BAR_BW+BAR_BG);
        int bh=(int)((uint32_t)num_s[i]*BAR_H/100);
        uint8_t col = num_s[i]>70 ? C_GREEN
                    : num_s[i]>40 ? C_ORANGE
                    : num_s[i]>15 ? C_YELLOW
                    :               C_DGREY;
        ui_fill_rect(bx,BAR_BASE-BAR_H,BAR_BW,BAR_H,C_DGREY);
        if(bh>0) ui_fill_rect(bx,BAR_BASE-bh,BAR_BW,bh,col);
        if(bh>1) ui_hline(bx,BAR_BASE-bh,BAR_BW,C_WHITE);
        ui_char(bx,BAR_BASE+2,'0'+i, num_s[i]>15?col:C_MGREY);
    }

    // baseline
    ui_hline(0,BAR_BASE,UI_W,C_MGREY);
}

// ============================================================================
// PAGE 2 — SETTINGS
// ============================================================================
static void draw_settings_toggle(int x, int y, int w, int on, uint8_t on_col){
    // Track
    uint8_t track_col = on ? on_col : C_DGREY;
    ui_rrect(x, y, w, 10, track_col, on ? on_col : C_CARD);
    // Knob
    int kx = on ? x+w-11 : x+1;
    ui_fill_circle(kx+4, y+5, 4, C_WHITE);
}

static void draw_page_settings(void) {
    int sy = 22;

    // Header card
    ui_fill_rect(0, sy, UI_W, 12, C_DBLUE);
    ui_hline(0, sy, UI_W, C_ORANGE);
    ui_str(4, sy+3, "SYSTEM SETTINGS", C_ORANGE);
    sy += 14;
    ui_str(4, sy, "KEY0:CONFIRM   KEY1:SKIP SETTING", C_MGREY);
    sy += 12;

    // ── Setting rows ──────────────────────────────────────────────────────
    struct { const char* label; const char* desc; int val; uint8_t acc; } rows[] = {
        {"VIDEO ROTATION",  "Rotate camera feed based on device tilt", settings_rot,  C_CYAN},
        {"CUBE AUTO-SPIN",  "Spin cube when device held flat",         settings_spin, C_PURPLE},
    };

    for(int i=0;i<2;i++){
        int sel = (settings_cursor==i);
        // Row card background
        uint8_t bg = sel ? C_DCYAN : C_CARD;
        ui_fill_rect(0, sy, UI_W, 30, bg);
        ui_hline(0, sy, UI_W, sel ? rows[i].acc : C_DGREY);

        // Cursor arrow
        if(sel){
            ui_str(2, sy+4, ">", rows[i].acc);
            ui_str(2, sy+11,"_", rows[i].acc);
        }

        // Label + desc
        ui_str(12, sy+4,  rows[i].label, sel ? rows[i].acc : C_WHITE);
        ui_str(12, sy+13, rows[i].desc,  C_MGREY);

        // Toggle switch (right side)
        draw_settings_toggle(UI_W-52, sy+10, 46, rows[i].val, rows[i].acc);
        ui_str(UI_W-48, sy+12,
               rows[i].val ? "ON " : "OFF",
               rows[i].val ? rows[i].acc : C_MGREY);

        sy += 32;
        ui_hline(0, sy, UI_W, C_DGREY);
        sy += 2;
    }

    // ── Active config summary card ─────────────────────────────────────────
    sy += 4;
    ui_rrect(4, sy, UI_W-8, 28, C_DGREY, C_DBLUE);
    ui_str(8, sy+4,  "ACTIVE CONFIG", C_LGREY);
    char tmp[40];
    snprintf(tmp,sizeof(tmp),"ROTATION: %-3s    AUTO-SPIN: %-3s",
             settings_rot?"ON":"OFF", settings_spin?"ON":"OFF");
    ui_str(8, sy+14, tmp, C_CYAN);
    sy += 32;

    // ── Key legend ─────────────────────────────────────────────────────────
    ui_rrect(4, sy, 150, 14, C_ORANGE, C_CARD);
    ui_str(8, sy+4, "KEY0", C_ORANGE); ui_str(30, sy+4, "TOGGLE + ADVANCE", C_WHITE);
    ui_rrect(158, sy, 154, 14, C_DGREY, C_CARD);
    ui_str(162, sy+4, "KEY1", C_LGREY); ui_str(184, sy+4, "SKIP   + ADVANCE", C_MGREY);
}

// ============================================================================
// PAGE 3 — CAMERA (waiting / live indicator)
// ============================================================================
static void draw_page_camera(void) {
    // Decorative corner brackets
    int m=12;
    ui_hline(m,     20+m, 20, C_GREEN); ui_vline(m,     20+m, 20, C_GREEN);
    ui_hline(UI_W-m-20,20+m,20,C_GREEN);ui_vline(UI_W-m-1,20+m,20,C_GREEN);
    ui_hline(m,     UI_H-m-14-20,20,C_GREEN);ui_vline(m,UI_H-m-14-20-19,20,C_GREEN);
    ui_hline(UI_W-m-20,UI_H-m-14-20,20,C_GREEN);ui_vline(UI_W-m-1,UI_H-m-14-20-19,20,C_GREEN);

    // Centred status
    int cy = UI_H/2 - 20;
    ui_fill_circle(UI_W/2, cy, 8, is_spi ? C_GREEN : C_DGREY);
    if(is_spi){
        ui_str_cx(UI_W/2, cy+14, "CAMERA ACTIVE", C_GREEN);
        ui_str_cx(UI_W/2, cy+24, "AI INFERENCE RUNNING", C_LGREY);
    } else {
        ui_str_cx(UI_W/2, cy+14, "STARTING CAMERA...", C_ORANGE);
        ui_str_cx(UI_W/2, cy+24, "PLEASE WAIT", C_MGREY);
    }

    // Bottom hint
    ui_fill_rect(0, UI_H-28, UI_W, 14, C_DBLUE);
    ui_hline(0, UI_H-28, UI_W, C_DGREY);
    ui_str_cx(UI_W/2, UI_H-24, "KEY0 OR KEY1 TO LEAVE", C_MGREY);
}

// ============================================================================
// PAGE 4 — CUBE
// ============================================================================

static void draw_page_cube(void) {
    // 65% of the 320x240 screen is roughly 208x156
    int w = 208;
    int h = 156;

    // Center the viewport
    int start_x = (UI_W - w) / 2;
    int start_y = (UI_H - h) / 2;

    // Draw a stylized card background/border
    ui_fill_rect(start_x - 2, start_y - 2, w + 4, h + 4, C_DGREY);
    ui_fill_rect(start_x, start_y, w, h, C_BLACK);

    // Pointer to the shared SDRAM where Core 0 renders the 320x240 cube
    uint8_t* sdram = (uint8_t*)0x01E00000;

    // Sample the center window from SDRAM and draw it to ui_buf
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            // Calculate corresponding center offset in the 320x240 SDRAM buffer
            int src_x = (320 - w) / 2 + x;
            int src_y = (240 - h) / 2 + y;

            uint8_t pixel = sdram[src_y * 320 + src_x];

            // Only draw non-black pixels to blend the 3D model into the UI
            if (pixel != 0) {
                ui_pixel(start_x + x, start_y + y, pixel);
            }
        }
    }

    // Add a small overlay tag inside the viewport
    ui_str(start_x + 6, start_y + 6, "3D ACCEL ENGINE", C_YELLOW);
    ui_str(start_x + 6, start_y + 16, "CORE 0 RENDER", C_MGREY);
}

// ============================================================================
// PAGE 3 — CAMERA (waiting / live indicator)
// ============================================================================
#define NUM_STOCKS 4
#define HIST_LEN   15  // Number of candles rendered on the chart

typedef struct {
    int open;
    int close;
    int high;
    int low;
} Candle;

// Distinct historical data storage for each individual asset
static Candle stock_history[NUM_STOCKS][HIST_LEN];
static int history_initialized = 0;
static int update_ticks[NUM_STOCKS] = {0, 0, 0, 0};

static void draw_page_market(void) {
    // Read detected word from AI
    char wbuf[17]={0};
    for(int i=0;i<16;i++) wbuf[i]=(char)IORD_8DIRECT(WORD_WRADDR,i);
    wbuf[16]='\0';

    // Draw the Terminal UI Card
    ui_fill_rect(10, 26, UI_W-20, UI_H-46, C_DBLUE);
    ui_rrect(10, 26, UI_W-20, UI_H-46, C_CYAN, 0xFF);
    ui_str_cx(UI_W/2, 34, "LIVE STOCK EXCHANGE", C_CYAN);
    ui_hline(10, 46, UI_W-20, C_DGREY);

    // Identify stock index and core base price in cents
    int base_cents = 0;
    int stock_idx = -1;
    if (strncmp(wbuf, "RDB", 4) == 0)      { base_cents = 23224; stock_idx = 0; }
    else if (strncmp(wbuf, "MERC", 4) == 0) { base_cents = 41550; stock_idx = 1; }
    else if (strncmp(wbuf, "FERR", 4) == 0) { base_cents = 17410; stock_idx = 2; }
    else if (strncmp(wbuf, "MCL", 4) == 0) { base_cents = 21245; stock_idx = 3; }

    // Initialize distinct random-walk chart histories once on startup
    if (!history_initialized) {
        int bases[4] = {23224, 41550, 17410, 21245};
        for (int s = 0; s < NUM_STOCKS; s++) {
            int current = bases[s];
            for (int i = 0; i < HIST_LEN; i++) {
                stock_history[s][i].open = current;

                // Formulate a distinct baseline pattern path unique to each market index
                int delta = ((i * 23 + s * 43) % 260) - 120;
                stock_history[s][i].close = current + delta;

                int max_oc = (stock_history[s][i].open > stock_history[s][i].close) ? stock_history[s][i].open : stock_history[s][i].close;
                int min_oc = (stock_history[s][i].open < stock_history[s][i].close) ? stock_history[s][i].open : stock_history[s][i].close;

                stock_history[s][i].high = max_oc + 30 + (i % 4) * 10;
                stock_history[s][i].low  = min_oc - 30 - (i % 3) * 10;
                if (stock_history[s][i].low < 0) stock_history[s][i].low = 0;

                current = stock_history[s][i].close;
            }
        }
        history_initialized = 1;
    }

    if (stock_idx != -1) {
        // Read accelerometer to act as our live source
        int ax = (int32_t)IORD_32DIRECT(ACCE_WRADDR, 0);
        int ay = (int32_t)IORD_32DIRECT(ACCE_WRADDR, 4);

        // Calculate dynamic real-time live price fluctuation
        int fluctuation = (ax * 3) + (ay * 2);
        int current_cents = base_cents + fluctuation;
        if(current_cents < 0) current_cents = 0;

        // Split into Dollars and Cents
        int dollars = current_cents / 100;
        int cents = current_cents % 100;
        if(cents < 0) cents = -cents;

        // Assign live calculated ticks to the active candle frame (the latest array index)
        int live_idx = HIST_LEN - 1;
        stock_history[stock_idx][live_idx].close = current_cents;

        int live_max_oc = (stock_history[stock_idx][live_idx].open > current_cents) ? stock_history[stock_idx][live_idx].open : current_cents;
        int live_min_oc = (stock_history[stock_idx][live_idx].open < current_cents) ? stock_history[stock_idx][live_idx].open : current_cents;

        stock_history[stock_idx][live_idx].high = live_max_oc + 35;
        stock_history[stock_idx][live_idx].low  = live_min_oc - 35;
        if(stock_history[stock_idx][live_idx].low < 0) stock_history[stock_idx][live_idx].low = 0;

        // Shift history left periodically to create moving candles
        update_ticks[stock_idx]++;
        if (update_ticks[stock_idx] >= 25) {  // Shift candle interval spacing threshold
            update_ticks[stock_idx] = 0;
            for (int i = 0; i < HIST_LEN - 1; i++) {
                stock_history[stock_idx][i] = stock_history[stock_idx][i+1];
            }
            // Next candle's open matches the closed candle's terminal value
            stock_history[stock_idx][HIST_LEN - 1].open = current_cents;
        }

        // Render Asset Info & Live Price Header
        ui_str2x(25, 52, wbuf, C_WHITE);

        char pstr[16];
        snprintf(pstr, sizeof(pstr), "$%d.%02d", dollars, cents);
        uint8_t pcol = (fluctuation >= 0) ? C_GREEN : C_RED;
        ui_str2x(125, 52, pstr, pcol);

        // Status Tag Indicator
        if (fluctuation >= 0) ui_str(250, 58, "+ LIVE UP", C_GREEN);
        else                  ui_str(250, 58, "- LIVE DN", C_RED);

        // Render Volume Data
        char vstr[32];
        snprintf(vstr, sizeof(vstr), "VOL: %d,000", 1400 + (abs(ax) % 500));
        ui_str(25, 74, vstr, C_LGREY);

        // ====================================================================
        // CANDLESTICK CHART LAYOUT ENGINE
        // ====================================================================
        int chart_x = 25;
        int chart_y = 92;
        int chart_w = 255; // 15 candles * 17 horizontal pixels spacing = 255
        int chart_h = 96;

        // Draw background canvas border lines
        ui_rect(chart_x, chart_y, chart_w, chart_h, C_DGREY);
        ui_hline(chart_x + 1, chart_y + (chart_h / 2), chart_w - 2, C_DGREY); // Center baseline

        // Calculate maximum and minimum range bounds inside the active history screen frame
        int global_min = 99999999;
        int global_max = 0;
        for (int i = 0; i < HIST_LEN; i++) {
            if (stock_history[stock_idx][i].low < global_min)  global_min = stock_history[stock_idx][i].low;
            if (stock_history[stock_idx][i].high > global_max) global_max = stock_history[stock_idx][i].high;
        }
        int price_range = global_max - global_min;
        if (price_range <= 0) price_range = 1;

        // Draw side bounding reference tags
        char max_lbl[12], min_lbl[12];
        snprintf(max_lbl, sizeof(max_lbl), "$%d", global_max / 100);
        snprintf(min_lbl, sizeof(min_lbl), "$%d", global_min / 100);
        ui_str(chart_x + chart_w + 4, chart_y, max_lbl, C_MGREY);
        ui_str(chart_x + chart_w + 4, chart_y + chart_h - 8, min_lbl, C_MGREY);

        // Loop and render individual candlesticks
        int slot_w = 17;
        int body_w = 9;
        for (int i = 0; i < HIST_LEN; i++) {
            int cx = chart_x + (i * slot_w) + (slot_w / 2); // Center line coordinate for wick
            int bx = cx - (body_w / 2);                     // Left side pixel index for candle body

            // Internal padding height alignment constraints
            int inner_h = chart_h - 12;
            int y_top_bound = chart_y + 6;
            int y_bot_bound = chart_y + chart_h - 6;

            // Scale target data entries directly into vertical pixel lines
            int y_open  = y_bot_bound - ((stock_history[stock_idx][i].open  - global_min) * inner_h / price_range);
            int y_close = y_bot_bound - ((stock_history[stock_idx][i].close - global_min) * inner_h / price_range);
            int y_high  = y_bot_bound - ((stock_history[stock_idx][i].high  - global_min) * inner_h / price_range);
            int y_low   = y_bot_bound - ((stock_history[stock_idx][i].low   - global_min) * inner_h / price_range);

            uint8_t candle_col = (stock_history[stock_idx][i].close >= stock_history[stock_idx][i].open) ? C_GREEN : C_RED;

            // 1. Draw Wick Line Component
            int wick_len = y_low - y_high + 1;
            if (wick_len > 0) {
                ui_vline(cx, y_high, wick_len, candle_col);
            }

            // 2. Draw Candlestick Solid Center Body Rectangle
            if (stock_history[stock_idx][i].close >= stock_history[stock_idx][i].open) {
                // Bullish Candle (Green Fill)
                int body_h = y_open - y_close + 1;
                ui_fill_rect(bx, y_close, body_w, body_h, C_GREEN);
            } else {
                // Bearish Candle (Red Fill)
                int body_h = y_close - y_open + 1;
                ui_fill_rect(bx, y_open, body_w, body_h, C_RED);
            }
        }

        // Adjusted asset track info string further down to avoid overlaying graph frame
        wbuf[4] = '\0';
        char bot_prompt[32];
        snprintf(bot_prompt, sizeof(bot_prompt), "TRACKING ASSET: %s", wbuf);
        ui_str_cx(UI_W/2, 202, bot_prompt, C_YELLOW);

    } else {
        // Idle State when no known company is detected
        ui_str_cx(UI_W/2, 100, "WAITING FOR SYMBOL...", C_ORANGE);
        ui_str_cx(UI_W/2, 120, "SCAN: RDB, MERC, FERR, MCL", C_MGREY);
    }
}
// ============================================================================
// FULL PAGE DISPATCH
// ============================================================================
static void draw_ui_page(int page) {
    ui_fill(C_BLACK);
    ui_draw_header(page);
    switch(page){
        case 0: draw_page_dashboard(); break;
        case 1: draw_page_inference(); break;
        case 2: draw_page_settings();  break;
        case 3: draw_page_camera();    break;
        case 4: draw_page_cube(); break;
        case 5: draw_page_market();    break;
    }
    ui_draw_status();
    write_pixel_array((const uint8_t*)ui_buf);
}

// PARTIAL UPDATE — dashboard left panel + status bar (~2.6× faster)
static void draw_ui_page_partial(void) {
    for(int r=20;r<UI_H-14;r++) ui_hline(0,r,117,C_BLACK);
    draw_page_dashboard();
    ui_draw_status();
    write_region(0, 20,      117, UI_H-34);
    write_region(0, UI_H-14, UI_W, 14);
}

// ============================================================================
// KEY HANDLER
// ============================================================================
static void handle_key(int is_confirm) {
    int prev_page = current_page;
    if(current_page==2){
        if(is_confirm){
            if(settings_cursor==0){settings_rot=!settings_rot;  IOWR_8DIRECT(SETTINGS_WRADDR,0,settings_rot);}
            else                  {settings_spin=!settings_spin;IOWR_8DIRECT(SETTINGS_WRADDR,1,settings_spin);}
        }
        if(++settings_cursor>=2){settings_cursor=0; current_page=(current_page+1)%NUM_PAGES;}
        force_redraw=1;
    } else {
        current_page = is_confirm
            ? (current_page+NUM_PAGES-1)%NUM_PAGES
            : (current_page+1)%NUM_PAGES;
        if(current_page==2) settings_cursor=0;
    }

    // Camera page transitions
    if(prev_page==3 && current_page!=3) set_camera_page(0);
    if(prev_page!=3 && current_page==3) {
        set_camera_page(1);
        rx_buffer &= ~0x00000004;
        is_cube = 0;
    }

    // Cube page transitions — tells Core 0 to start/stop rendering
    if(prev_page==4 && current_page!=4) set_cube_page(0);
    if(prev_page!=4 && current_page==4) {
        set_cube_page(1);
        force_redraw = 1;
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

void vga_task(void* pdata){
    INT8U err;
    IOWR_8DIRECT(SETTINGS_WRADDR,0,settings_rot);
    IOWR_8DIRECT(SETTINGS_WRADDR,1,settings_spin);
    IOWR_8DIRECT(SETTINGS_WRADDR,3,0);

    while(1){
        OSSemPend(vga_trigger_sem, OS_TICKS_PER_SEC/20, &err);

        if(!emergency_stop){
            emergency_counter=0;
            if(key0_click){key0_click=0;handle_key(1);}
            if(key1_click){key1_click=0;handle_key(0);}

            if ((rx_buffer & 0x00000004) && current_page == 4) {
                // Cube frame — only display if NOT on camera page
                for(int i=0;i<(UI_W*UI_H)/4;i++){
                    uint32_t w=IORD_32DIRECT(DRAM_WRADDR,i*4);
                    cube_buf[i*4+0]=(w>>24)&0xFF;cube_buf[i*4+1]=(w>>16)&0xFF;
                    cube_buf[i*4+2]=(w>> 8)&0xFF;cube_buf[i*4+3]= w     &0xFF;}
                write_pixel_array(cube_buf);
                rx_buffer&=~0x00000004; rx_buffer&=~0x80000000;
                IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE2_TX_BASE,0x80000000);
                IOWR_ALTERA_AVALON_PIO_DATA(IRQ_CORE2_TX_BASE,0x0);

            } else if(rx_buffer & 0x80000000){
                write_pixel_color(DRAM_WRADDR);
                rx_buffer&=~0x80000000;

            } else if(!is_spi && !is_cube){
                int page_changed=(current_page!=last_page);
                if(page_changed||force_redraw){
                    draw_ui_page(current_page);
                    last_page=current_page; force_redraw=0;
                } else if(current_page==0){
                    draw_ui_page_partial();
                } else {
                    draw_ui_page(current_page);
                }
            } else if(is_spi && current_page==3 && !(rx_buffer&0x80000000)){
                if(force_redraw||last_page!=3){
                    draw_ui_page(3); last_page=3; force_redraw=0;}
            }
        }

        if(emergency_stop){
            const uint8_t* img=(emergency_counter%2==0)
                               ?(uint8_t*)emergencyIMG:(uint8_t*)emergencyIMG2;
            write_pixel_array(img);
            emergency_counter++;
        }
    }
}

void cpu_monitor_task(void* pdata){
    while(1){ IOWR_32DIRECT(CORE2_WRADDR,0,OSCPUUsage); OSTimeDlyHMSM(0,0,0,500); }}
void startup_task(void* pdata){
    OSStatInit();
    OSTaskCreateExt(vga_task,         NULL,(void*)&vga_task_stk        [TASK_STACKSIZE-1],VGA_PRIORITY,    VGA_PRIORITY,    vga_task_stk,        TASK_STACKSIZE,NULL,0);
    OSTaskCreateExt(cpu_monitor_task, NULL,(void*)&cpu_monitor_task_stk[TASK_STACKSIZE-1],CPU_MONITOR_PRIO,CPU_MONITOR_PRIO,cpu_monitor_task_stk,TASK_STACKSIZE,NULL,0);
    OSTaskDel(OS_PRIO_SELF);}
int main(void){
    alt_printf("CPU Core 2 Alive\n");
    vga_trigger_sem=OSSemCreate(0);
    init_rx();
    OSTaskCreateExt(startup_task,NULL,(void*)&startup_task_stk[TASK_STACKSIZE-1],0,0,startup_task_stk,TASK_STACKSIZE,NULL,0);
    OSStart(); return 0;}
