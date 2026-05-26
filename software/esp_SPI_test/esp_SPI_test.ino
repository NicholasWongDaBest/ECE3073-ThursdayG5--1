// ESP32-S3 dual-core SPI slave + SSCMA camera
// Core 0: continuous AI inference + image decode → front/back buffer swap
// Core 1: waits for trigger → snapshots front_bitmap → sends over SPI

#include <ESP32SPISlave.h>
#include "ImageTransform.h"
#include <Seeed_Arduino_SSCMA.h>

#define PIN_MOSI  10
#define PIN_MISO  9
#define PIN_SCLK  8
#define PIN_CS    7
#define SPI_MODE  SPI_MODE3

static constexpr uint32_t PAYLOAD_BYTES = 60;
static constexpr uint32_t BUFFER_SIZE   = PAYLOAD_BYTES + 4;  // 64
static constexpr uint32_t QUEUE_SIZE    = 2;

const uint8_t START_MAGIC_PATTERN[8] = {0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00}; // 0xFF00FF00
const uint8_t STOP_MAGIC_PATTERN[8]  = {0xEE, 0x11, 0xEE, 0x11, 0xEE, 0x11, 0xEE, 0x11}; // 0xEE11EE11

// DMA-safe buffers in internal DRAM
static DRAM_ATTR uint8_t tx_buf[BUFFER_SIZE] __attribute__((aligned(32)));
static DRAM_ATTR uint8_t rx_buf[BUFFER_SIZE] __attribute__((aligned(32)));


ESP32SPISlave slave;
SSCMA AI;

const char* my_labels[] = {
    "0", "1", "A", "B", "C", "D", "E", "F", "G", "H",
    "I", "J", "2", "K", "L", "M", "N", "O", "P", "Q",
    "R", "S", "T", "3" , "U", "V", "W", "X", "Y", "Z",
    "4", "  5", "6", "7", "8", "9"
};
const int label_count = sizeof(my_labels) / sizeof(my_labels[0]);
void sendScores();

// ---------------------------------------------------------------------------
// Core 0: inference loop
// Decodes into back_bitmap, then swaps front/back.
// No locking needed — Core 1 reads only from send_buf which Core 0 never touches.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Preset word matcher — add after renderSidebars(), before rotateCenter()
// ---------------------------------------------------------------------------

// In ESP32 — replace PRESET_WORDS array:
const char* PRESET_WORDS[] = {
    "AAPL ", "MSFT ", "GOOG ", "TSLA ",
    "AMZN ", "NVDA ", "META ", "NFLX ",
    "HELLO ", "YELLOW ", "GREEN ", "RED "
};
#define PRESET_COUNT  (sizeof(PRESET_WORDS) / sizeof(PRESET_WORDS[0]))
#define MATCH_THRESH    0.6f
#define COOLDOWN_FRAMES 15
#define MIN_DETECTIONS  2    // ignore frames with only 1 box (noise)

static char committed_word[16]  = {0};
static bool word_committed      = false;
static int  cooldown_counter    = 0;

static int popcount32(uint32_t v) {
    int c = 0; while (v) { c += v & 1; v >>= 1; } return c;
}

static uint32_t word_mask(const char* w) {
    uint32_t m = 0;
    while (*w) {
        char c = *w++;
        if (c >= 'A' && c <= 'Z') m |= (1u << (c - 'A'));
        if (c >= 'a' && c <= 'z') m |= (1u << (c - 'a'));
    }
    return m;
}

void updateWordMatcher(const std::vector<boxes_t>& boxes,
                       const char** labels, int label_count_in) {
    if (cooldown_counter > 0) { cooldown_counter--; return; }
    if ((int)boxes.size() < MIN_DETECTIONS) return;

    uint32_t frame_mask = 0;
    for (int i = 0; i < (int)boxes.size(); i++) {
        int idx = boxes[i].target;
        if (idx < 0 || idx >= label_count_in) continue;
        const char* lbl = labels[idx];
        char c = lbl[0];
        if (c >= 'A' && c <= 'Z') frame_mask |= (1u << (c - 'A'));
        if (c >= 'a' && c <= 'z') frame_mask |= (1u << (c - 'a'));
    }

    const char* best_word  = nullptr;
    float       best_score = 0.0f;

    for (int w = 0; w < (int)PRESET_COUNT; w++) {
        uint32_t wm   = word_mask(PRESET_WORDS[w]);
        int total     = popcount32(wm);
        if (total == 0) continue;
        int matched   = popcount32(frame_mask & wm);
        float score   = (float)matched / (float)total;
        if (score > best_score) { best_score = score; best_word = PRESET_WORDS[w]; }
    }

    if (best_score >= MATCH_THRESH && best_word != nullptr) {
        strncpy(committed_word, best_word, 15);
        committed_word[15] = '\0';
        word_committed     = true;
        cooldown_counter   = COOLDOWN_FRAMES;
        Serial.printf("[WordMatch] \"%s\" score=%.2f boxes=%d\n",
                      committed_word, best_score, (int)boxes.size());
    }
}

// ---------------------------------------------------------------------------
// Inference stats — tracks appearance count and best score per label
// ---------------------------------------------------------------------------
static int  label_count_tracker[66] = {0};   // appearance count per label
static int  label_best_score[66]    = {0};   // best score seen per label
static int label_score_sum[66]     = {0}; 
static uint8_t alpha_scores[26] = {0};  // A-Z in order
static uint8_t num_scores[10]   = {0};  // 0-9 in order

static const int ALPHA_IDX[] = {2,3,4,5,6,7,8,9,10,11,13,14,15,16,17,18,19,20,21,22,24,25,26,27,28,29};
static const int NUM_IDX[]   = {0,1,12,23,30,31,32,33,34,35};

// Global top1 result — written by inference, read by sendImage
static char  top1_label[8]  = "?";
static uint8_t top1_avg     = 0;

volatile uint8_t current_rotation = 0;

void compute_top1() {
    int best_idx   = -1;
    int best_count = 0;
    int best_score = 0;

    for (int i = 0; i < label_count; i++) {
        if (label_count_tracker[i] == 0) continue;
        if (label_count_tracker[i] > best_count ||
           (label_count_tracker[i] == best_count && label_best_score[i] > best_score)) {
            best_idx   = i;
            best_count = label_count_tracker[i];
            best_score = label_best_score[i];
        }
    }

    if (best_idx >= 0) {
        // Compute average confidence for top1
        int avg = label_score_sum[best_idx] / label_count_tracker[best_idx];
        strncpy(top1_label, my_labels[best_idx], 7);
        top1_label[7] = '\0';
        top1_avg = (uint8_t)avg;
        Serial.printf("Top1: %s  count=%d  avg=%d\n",
            top1_label, best_count, avg);
    } else {
        strncpy(top1_label, "?", 7);
        top1_avg = 0;
    }
}

void inference_task(void* pdata) {
    Serial.println("[Core0] Inference task started");

    while (1) {
        int status = -1;
        while (status != 0) {
            status = AI.invoke(1, false, true);
            if (status != 0) { Serial.printf("[Core0] AI error %d\n", status); delay(100); }
        }

        for (int i = 0; i < (int)AI.boxes().size(); i++) {
            int idx = AI.boxes()[i].target;
            if (idx >= 0 && idx < label_count) {
                label_count_tracker[idx]++;
                label_score_sum[idx]  += AI.boxes()[i].score;
                if (AI.boxes()[i].score > label_best_score[idx])
                    label_best_score[idx] = AI.boxes()[i].score;
            }
        }

        // --- Capture sidebar data BEFORE resetting trackers ---
        InferEntry sidebar_entries[MAX_SIDEBAR_ENTRIES];
        int num_sidebar = 0;
        for (int i = 0; i < label_count && num_sidebar < MAX_SIDEBAR_ENTRIES; i++) {
            if (label_count_tracker[i] == 0) continue;
            strncpy(sidebar_entries[num_sidebar].label, my_labels[i], 7);
            sidebar_entries[num_sidebar].label[7] = '\0';
            sidebar_entries[num_sidebar].count    = label_count_tracker[i];
            sidebar_entries[num_sidebar].avg_conf =
                (uint8_t)(label_score_sum[i] / label_count_tracker[i]);
            num_sidebar++;
        }
        updateWordMatcher(AI.boxes(), my_labels, label_count);
        compute_top1();
        
        // Capture per-letter scores BEFORE reset
        for (int i = 0; i < 26; i++) {
            int idx = ALPHA_IDX[i];
            alpha_scores[i] = (label_count_tracker[idx] > 0)
                ? (uint8_t)(label_score_sum[idx] / label_count_tracker[idx]) : 0;
        }
        for (int i = 0; i < 10; i++) {
            int idx = NUM_IDX[i];
            num_scores[i] = (label_count_tracker[idx] > 0)
                ? (uint8_t)(label_score_sum[idx] / label_count_tracker[idx]) : 0;
        }

        memset(label_count_tracker, 0, sizeof(label_count_tracker));
        memset(label_score_sum,     0, sizeof(label_score_sum));
        memset(label_best_score,    0, sizeof(label_best_score));

        String b64 = AI.last_image();
        if (b64.length() == 0) continue;
        if (!processBase64Jpeg(b64.c_str())) continue;

        // --- Render sidebar onto back_bitmap before swap ---
        renderSidebars(back_bitmap, sidebar_entries, num_sidebar, committed_word);

        uint8_t* tmp = front_bitmap;
        front_bitmap = back_bitmap;
        back_bitmap  = tmp;
        final_bitmap = front_bitmap;
        frame_ready  = true;
    }
}

// ---------------------------------------------------------------------------
// sendImage — sends from send_buf (private snapshot, Core 0 never touches it)
// ---------------------------------------------------------------------------
void sendImage() {
    // 1. Send START Marker
    memset(tx_buf, 0, BUFFER_SIZE);
    memcpy(tx_buf, START_MAGIC_PATTERN, 8);
    slave.queue(tx_buf, NULL, BUFFER_SIZE);
    slave.wait();

    // 2. Send Image Data Packets (Unchanged)
    uint32_t total = OUT_BUFFER_SIZE;
    for (uint32_t j = 0; j < total; j += PAYLOAD_BYTES) {
        memset(tx_buf, 0, BUFFER_SIZE);
        tx_buf[0] = 0xA0; tx_buf[1] = 0xA0;
        uint32_t chunk = ((j + PAYLOAD_BYTES) <= total) ? PAYLOAD_BYTES : (total - j);
        memcpy(&tx_buf[2], &send_buf[j], chunk);
        tx_buf[BUFFER_SIZE-2] = 0xA0;
        tx_buf[BUFFER_SIZE-1] = 0xA0;
        slave.queue(tx_buf, NULL, BUFFER_SIZE);
        slave.wait();
    }

    // 3. Send Inference Packet
    memset(tx_buf, 0, BUFFER_SIZE);
    tx_buf[0] = 0xC0; tx_buf[1] = 0xC0;   
    strncpy((char*)&tx_buf[2], top1_label, 6);
    tx_buf[8]  = top1_avg;
    tx_buf[BUFFER_SIZE-2] = 0xC0;
    tx_buf[BUFFER_SIZE-1] = 0xC0;
    slave.queue(tx_buf, NULL, BUFFER_SIZE);
    slave.wait();

    // 3b. Send word-match packet
    memset(tx_buf, 0, BUFFER_SIZE);
    tx_buf[0] = 0xD0; tx_buf[1] = 0xD0;
    strncpy((char*)&tx_buf[2], committed_word, 15);
    tx_buf[2 + 15] = word_committed ? 0x01 : 0x00;  // flag byte
    tx_buf[BUFFER_SIZE-2] = 0xD0;
    tx_buf[BUFFER_SIZE-1] = 0xD0;
    slave.queue(tx_buf, NULL, BUFFER_SIZE);
    slave.wait();
    
    // 3c. Send combined score+word packet
    sendScores();


    // Clear committed flag after sending
    word_committed = false;

    // 4. Send DISTINCT STOP Marker
    memset(tx_buf, 0, BUFFER_SIZE);
    memcpy(tx_buf, STOP_MAGIC_PATTERN, 8);
    slave.queue(tx_buf, NULL, BUFFER_SIZE);
    slave.wait();
}

void sendScores() {
    // Single packet: scores + word combined
    // [0-1]=0xE0E0 marker, [2-27]=A-Z, [28-37]=0-9,
    // [38-52]=word(15), [53]=word_committed, [62-63]=0xE0E0
    Serial.println("sent score");
    memset(tx_buf, 0, BUFFER_SIZE);
    tx_buf[0] = 0xE0; tx_buf[1] = 0xE0;
    for (int i = 0; i < 26; i++) tx_buf[2  + i] = alpha_scores[i];
    for (int i = 0; i < 10; i++) tx_buf[28 + i] = num_scores[i];
    strncpy((char*)&tx_buf[38], committed_word, 15);
    tx_buf[53] = word_committed ? 0x01 : 0x00;
    tx_buf[BUFFER_SIZE-2] = 0xE0;
    tx_buf[BUFFER_SIZE-1] = 0xE0;
    slave.queue(tx_buf, NULL, BUFFER_SIZE);
    slave.wait();
    word_committed = false;
}

// ---------------------------------------------------------------------------
// Core 1: SPI task
// On trigger: snapshot front_bitmap into send_buf, then send.
// Core 0 swaps freely the whole time — no locks needed.
// ---------------------------------------------------------------------------
void spi_task(void* pdata) {
    Serial.println("[Core1] SPI task started");

    while (1) {
        // Wait for trigger from Nios — exactly 4 bytes
        memset(rx_buf, 0, 4);
        slave.queue(NULL, rx_buf, 4);
        slave.wait();

        bool score_only = (rx_buf[3] == 0x02);
        bool triggered  = (rx_buf[3] & 0x01);
        
        if (score_only) {
            // Idle poll — send scores without waiting for frame
            sendScores();
            continue;
        }
        if (!triggered) continue;

        current_rotation = (rx_buf[3] >> 2) & 0x3;  // extract rotation
        Serial.printf("[Core1] Trigger received, rotation=%d\n", current_rotation);

        // Wait for Core 0 to have at least one fresh frame ready
        int wait_ms = 0;
        while (!frame_ready && wait_ms < 2000) {
            delay(10);
            wait_ms += 10;
        }

        if (!frame_ready) {
            Serial.println("[Core1] Frame timeout — skipping");
            continue;  // go back to listening for next trigger
        }
        
        frame_ready = false;

        // Snapshot front_bitmap into send_buf (~1ms for 76800 bytes from PSRAM)
        // After this point Core 0 can swap front/back as many times as it likes
        // without affecting our transmission — we read only from send_buf
        memcpy(send_buf, front_bitmap, OUT_BUFFER_SIZE);

        rotateCenter(send_buf, current_rotation);

        Serial.println("[Core1] Sending frame...");
        sendImage();
        Serial.println("[Core1] Frame sent");
    }
}

// ---------------------------------------------------------------------------
// setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.printf("PSRAM size: %u bytes\n", ESP.getPsramSize());
    if (ESP.getPsramSize() == 0) {
        Serial.println("FATAL: PSRAM not detected — set Tools->PSRAM->OPI PSRAM");
        while (1);
    }

    if (!initFinalBitmap()) {
        Serial.println("FATAL: buffer init failed");
        while (1);
    }

    Serial.printf("front_bitmap @ %p\n", front_bitmap);
    Serial.printf("back_bitmap  @ %p\n", back_bitmap);
    Serial.printf("send_buf     @ %p\n", send_buf);
    Serial.printf("Free internal RAM: %u bytes\n",
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    slave.setDataMode(SPI_MODE);
    slave.setQueueSize(QUEUE_SIZE);
    slave.begin();                                                                                                                                                                  

    AI.begin();

    xTaskCreatePinnedToCore(inference_task, "inference", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(spi_task,       "spi",       4096, NULL, 1, NULL, 1);

    Serial.println("[ESP32] Both tasks launched — ready");
}

void loop() {
    delay(1000);
}