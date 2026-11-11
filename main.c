#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/flash.h"

// TinyUSB
#include "bsp/board_api.h"
#include "tusb.h"
#include "usb_descriptors.h"

// OLED display
#include "ssd1306.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ================== MODE ==================
#define FIXED_POWER_CW_MODE     0
#define FIXED_TX_POWER_DBM      (13)
// ==========================================

// ================== TEST MODE ==================
#define USE_TEST_TONE       0
#define USE_TWO_TONE_TEST   1

#define TEST_TONE_HZ        1000.0f
#define TEST_TONE2_HZ       1900.0f

#define TEST_TONE_AMPL      0.35f
#define TEST_BLOCK_SAMPLES  8000u
// ===============================================

// ================== DITHER SPEED-UP ==================
#define DITHER_SUBSTEPS     4
// ===============================================

// ================== BUFFERING ==================
#define BLOCK_SAMPLES       256u
#define NUM_BLOCKS          8u   // increased from 2 to prevent underruns
// ===============================================

// ================== UNDERRUN DIAGNOSTICS ==================
#define UNDERRUN_LED_ENABLE   1
#define UNDERRUN_LED_PULSE_MS 20u
// ===============================================

// ================== AUDIO SHAPING (default values) ==================
#define AUDIO_ENABLE_BANDPASS       1
#define AUDIO_BP_LO_HZ              50.0f
#define AUDIO_BP_HI_HZ              2700.0f
#define AUDIO_BP_MAX_STAGES         10      // Max stages (compile-time allocation)
#define AUDIO_BP_DEFAULT_STAGES     7       // Default stages (runtime adjustable, 1-10)
// Each stage = 12 dB/octave, so 7 stages = 84 dB/oct, 10 stages = 120 dB/oct

#define AUDIO_ENABLE_EQ             1
#define EQ_LOW_SHELF_HZ             190.0f
#define EQ_LOW_SHELF_DB             (-2.0f)
#define EQ_HIGH_SHELF_HZ            1700.0f
#define EQ_HIGH_SHELF_DB            (13.5f)
// ===============================================

// ================== COMPRESSION (default values) ==================
#define AUDIO_ENABLE_COMPRESSOR     1
#define COMP_THRESHOLD_DB           (-2.5f)
#define COMP_RATIO                  (6.1f)
#define COMP_ATTACK_MS              (41.1f)
#define COMP_RELEASE_MS             (1595.0f)
#define COMP_MAKEUP_DB              (0.0f)
#define COMP_KNEE_DB                (16.5f)
#define COMP_OUTPUT_LIMIT           (0.940f)
// ===============================================

// ================== MODULE VARIANT ==================
// Set to 1 if using LoRa1280F27-TCXO module
// Set to 0 if using LoRa1280F27 or LoRa1281F27 (standard crystal)
#define USE_TCXO_MODULE     1
// ====================================================

// ---------------- Pin mapping ----------------
static const uint32_t PIN_MISO  = 16;
static const uint32_t PIN_MOSI  = 19;
static const uint32_t PIN_SCK   = 18;
static const uint32_t PIN_NSS   = 17;

static const uint32_t PIN_RX_EN = 14;
static const uint32_t PIN_TX_EN = 15;

static const uint32_t PIN_RESET = 20;
static const uint32_t PIN_BUSY  = 21;

#if USE_TCXO_MODULE
static const uint32_t PIN_TCXO_EN = 22;
#endif

// ---------------- OLED I2C pins ----------------
#define OLED_I2C       i2c1
#define OLED_I2C_BAUD  400000
static const uint32_t PIN_OLED_SDA = 6;
static const uint32_t PIN_OLED_SCL = 7;

// ---------------- Encoder + buttons ----------------
static const uint32_t PIN_ENC_A   = 2;   // Encoder phase A
static const uint32_t PIN_ENC_B   = 3;   // Encoder phase B
static const uint32_t PIN_ENC_OK  = 4;   // Encoder push button
static const uint32_t PIN_PTT_KEY = 5;   // PTT / CW key

// ---------------- ADC microphone input ----------------
static const uint32_t PIN_ADC_MIC = 26;  // ADC0 = GPIO26

// ---------------- SPI config ----------------
#define SX_SPI spi0
static const uint32_t SX_SPI_BAUD = 18000000;

// ---------------- SX1280 opcodes ----------------
#define OPCODE_SET_STANDBY         0x80
#define OPCODE_SET_PACKET_TYPE     0x8A
#define OPCODE_SET_RF_FREQUENCY    0x86
#define OPCODE_SET_TX_PARAMS       0x8E
#define OPCODE_SET_TX_CW           0xD1

// ---------------- RF/audio params ----------------
#define BASE_FREQ_HZ        2400400000u
#define WAV_SAMPLE_RATE     8000u

#define PWR_MAX_DBM         (13)
#define PWR_MIN_DBM         (-18)

#define AMP_GAIN            2.9f
#define AMP_MIN_A           0.000002f

#define RAMP_TIME           0xE0     // 20 us

// --- Runtime RF config (adjustable via CDC) ---
// Frequency stored as double for sub-Hz precision; automatically split into PLL steps + fine DSP offset
static volatile double g_target_freq_hz = (double)BASE_FREQ_HZ;
static volatile float g_ppm_correction = 0.0f;
static volatile uint8_t g_cw_test_mode = 0;  // 1 = CW test active (blocks normal Core1 operation)
static volatile int8_t g_tx_power_max_dbm = PWR_MAX_DBM;  // Runtime TX power limit
static volatile uint8_t g_tx_enabled = 0;  // TX enable flag (for GUI TX button), default OFF
static volatile uint8_t g_tx_mode = 0;     // 0 = USB (SSB), 1 = CW, 2 = FM
static volatile uint8_t g_tune_active = 0; // 1 = TUNE carrier active
static volatile uint8_t g_ptt_key = 0;     // 1 = PTT/KEY pressed (live)
static volatile uint8_t g_audio_src = 0;   // 0 = PC (USB audio), 1 = MIC (ADC0)

// Guard window after any mode change — suppresses TX output on both
// cores while the carrier state machine settles and Core1 resumes
// cleanly.  Prevents clicks when the user scrolls the mode menu.
#define MODE_CHANGE_GUARD_MS   120u
static volatile uint32_t g_mode_change_at_ms = 0;

static inline bool tx_mode_guard_active(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    return (now - g_mode_change_at_ms) < MODE_CHANGE_GUARD_MS;
}

// Debug counters — surfaced on OLED to find why TX doesn't engage
static volatile uint32_t g_dbg_prod_txon = 0;   // samples with tx_on=1 in last prod block
static volatile uint32_t g_dbg_core1_txcw = 0;  // total SetTxCW commands sent by Core1
static volatile uint32_t g_dbg_prod_blocks = 0; // total blocks produced by Core0
static volatile uint32_t g_dbg_cons_blocks = 0; // total blocks consumed by Core1
static volatile uint8_t  g_dbg_core1_alive = 0; // 1 = Core1 reached main loop
static volatile uint32_t g_dbg_core1_iters = 0; // Core1 while(true) iterations
static volatile uint32_t g_dbg_busy_timeouts = 0; // sx_wait_busy() timeouts
static volatile uint32_t g_dbg_core1_bc = 0; // breadcrumb: last location Core1 was at
static volatile int      g_dbg_save_rc = 99;  // last flash_safe_execute return code (99=never tried)
static volatile uint32_t g_dbg_save_ok = 0;   // number of successful saves

// TX mode symbolic constants
#define TXM_USB     0
#define TXM_CW      1
#define TXM_FM      2

// --- FM mode parameters ---
static volatile float g_fm_deviation_hz = 2500.0f;  // FM deviation in Hz (±, default NBFM)
static volatile float g_ctcss_freq = 0.0f;          // CTCSS tone freq (0 = off)
static volatile uint8_t g_roger_beep = 0;           // Roger beep on end of FM TX (0=off, 1=on)

// --- Tune-digit cursor on main screen ---
// Tune step table: 100 Hz, 1 kHz, 10 kHz, 100 kHz, 1 MHz.  Index selects
// which digit of the displayed frequency (in kHz, one decimal) is underlined.
static const double g_tune_steps_hz[] = {
    100.0,         // digit: 0.1 kHz  (tenths of kHz)
    1000.0,        // digit:   1  kHz
    10000.0,       // digit:  10  kHz
    100000.0,      // digit: 100  kHz
    1000000.0,     // digit:   1  MHz
};
#define TUNE_STEP_COUNT  ((int)(sizeof(g_tune_steps_hz) / sizeof(g_tune_steps_hz[0])))

static volatile uint8_t g_tune_digit_idx = 1; // default = 1 kHz per click

// --- Encoder UI state ---
// Two-mode UI:
//   TUNE — main screen with underlined digit; encoder changes frequency
//          by the selected step, short click advances cursor to the next
//          digit, long press (≥2 s) enters the menu.
//   MENU — vertical scrollable menu; encoder scrolls cursor or edits an
//          item in-place, short click toggles edit/confirm, long press
//          exits back to TUNE.
typedef enum {
    UI_STATE_TUNE = 0,
    UI_STATE_MENU,
} ui_state_t;

typedef enum {
    MENU_MODE = 0,       // USB / CW / FM
    MENU_TX,             // TX on/off
    MENU_TUNE,           // TUNE carrier on/off
    MENU_SRC,            // PC / MIC
    MENU_POWER,          // dBm
    MENU_PPM,            // PPM correction
    MENU_FM_DEV,         // FM deviation Hz (only shown when mode = FM)
    MENU_CTCSS,          // CTCSS Hz (only shown when mode = FM)
    MENU_ROGER_BEEP,     // Roger beep on/off (only shown when mode = FM)
    MENU_SAVE,           // Save settings now
    MENU_EXIT,           // Exit menu back to TUNE
    MENU_ITEM_COUNT
} menu_item_t;

static volatile ui_state_t  g_ui_state = UI_STATE_TUNE;
static volatile menu_item_t g_menu_cursor = MENU_MODE;
static volatile uint8_t     g_menu_editing = 0;  // 1 when encoder edits selected item
static volatile uint32_t    g_menu_scroll_top = 0; // top visible menu row

// Forward decl (used by CDC help output well before the UI section)
static const char *mode_label(uint8_t m);

// --- Hilbert ---
#define HILBERT_TAPS        247

// --- PLL step ---
static const float PLL_STEP_HZ =
    (float)(52000000.0 / (double)(1u << 18)); // ~198.364 Hz

// --- FM modulation ---
#define FM_DEVIATION_HZ     2500.0f  // ±2.5 kHz deviation (NBFM)

#define F_OFF_LIMIT_HZ      3500.0f
#define SILENCE_SECONDS     2u

#define GATE_A_REF          0.01f   // Noise gate threshold - higher with compressor
#define GATE_SHAPE          1

#define IQ_GAIN_CORR        1.00f
#define IQ_PHASE_CORR_DEG   0.0f

// ==========================================================
// Persistent configuration in last flash sector
// ==========================================================
// We use the last 4 KB sector of external flash.  PICO_FLASH_SIZE_BYTES
// is defined by the board file and gives the usable flash size (bytes).
// The offset is relative to the XIP base; flash_range_erase / program
// take offsets relative to flash start, and reading goes through XIP.
#define CFG_FLASH_OFFSET   (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define CFG_MAGIC          0x53523132u    // 'SR12' (LE)
#define CFG_VERSION        1u

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    double   freq_hz;
    uint8_t  tx_mode;          // TXM_* constants
    int8_t   tx_power_dbm;
    uint8_t  audio_src;        // 0 = PC, 1 = MIC
    uint8_t  tune_digit_idx;   // which digit the cursor sits on in TUNE state
    float    ppm_correction;
    float    fm_deviation_hz;
    float    ctcss_freq;
    uint8_t  _reserved0;       // was: freedv_mode
    uint8_t  roger_beep;       // 0=off 1=on (FM only)
    uint8_t  _reserved[2];
    uint32_t crc32;            // CRC32 over everything above
} persist_cfg_t;

_Static_assert(sizeof(persist_cfg_t) <= FLASH_PAGE_SIZE,
               "persist_cfg_t must fit in one flash page");

static volatile uint8_t  g_persist_dirty       = 0;   // set when anything worth saving changed
static volatile uint32_t g_persist_dirty_since = 0;   // ms of last change

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
    crc ^= 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int k = 0; k < 8; k++) {
            uint32_t mask = -(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

static inline uint32_t persist_cfg_crc(const persist_cfg_t *c) {
    return crc32_update(0, (const uint8_t *)c,
                        offsetof(persist_cfg_t, crc32));
}

static void persist_collect(persist_cfg_t *c) {
    memset(c, 0, sizeof(*c));
    c->magic           = CFG_MAGIC;
    c->version         = CFG_VERSION;
    c->freq_hz         = g_target_freq_hz;
    c->tx_mode         = g_tx_mode;
    c->tx_power_dbm    = g_tx_power_max_dbm;
    c->audio_src       = g_audio_src;
    c->tune_digit_idx  = g_tune_digit_idx;
    c->ppm_correction  = g_ppm_correction;
    c->fm_deviation_hz = g_fm_deviation_hz;
    c->ctcss_freq      = g_ctcss_freq;
    c->roger_beep      = g_roger_beep;
    c->crc32           = persist_cfg_crc(c);
}

static void persist_apply(const persist_cfg_t *c) {
    // Defensive clamping — reject nonsense values silently
    double f = c->freq_hz;
    if (f < 2300000000.0) f = 2300000000.0;
    if (f > 2450000000.0) f = 2450000000.0;
    g_target_freq_hz = f;

    g_tx_mode = (c->tx_mode <= TXM_FM) ? c->tx_mode : TXM_USB;

    int8_t p = c->tx_power_dbm;
    if (p < PWR_MIN_DBM) p = PWR_MIN_DBM;
    if (p > PWR_MAX_DBM) p = PWR_MAX_DBM;
    g_tx_power_max_dbm = p;

    g_audio_src = (c->audio_src != 0) ? 1 : 0;

    uint8_t ti = c->tune_digit_idx;
    if (ti >= TUNE_STEP_COUNT) ti = 0;
    g_tune_digit_idx = ti;

    float ppm = c->ppm_correction;
    if (ppm < -50.0f) ppm = -50.0f;
    if (ppm >  50.0f) ppm =  50.0f;
    g_ppm_correction = ppm;

    float dev = c->fm_deviation_hz;
    if (dev < 200.0f)     dev = 200.0f;
    if (dev > 100000.0f)  dev = 100000.0f;
    g_fm_deviation_hz = dev;

    float ct = c->ctcss_freq;
    if (ct < 0.0f || ct > 300.0f) ct = 0.0f;
    g_ctcss_freq = ct;

    g_roger_beep = (c->roger_beep != 0) ? 1 : 0;
}

static bool persist_load(void) {
    const persist_cfg_t *p =
        (const persist_cfg_t *)(XIP_BASE + CFG_FLASH_OFFSET);
    if (p->magic != CFG_MAGIC) return false;
    if (p->version != CFG_VERSION) return false;
    if (persist_cfg_crc(p) != p->crc32) return false;
    persist_apply(p);
    return true;
}

// Flash operation callback — runs with IRQs disabled on Core0 and with
// Core1 parked in the lockout handler (no XIP reads) thanks to
// flash_safe_execute() from pico/flash.h.
// MUST live in RAM: XIP is disabled while this runs, so if the callback
// (or anything it calls) tried to fetch instructions from flash the
// device would lock up.  flash_range_erase / flash_range_program are
// already RAM-resident in the SDK.
static void __not_in_flash_func(persist_flash_op)(void *param) {
    const uint8_t *page = (const uint8_t *)param;
    flash_range_erase(CFG_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(CFG_FLASH_OFFSET, page, FLASH_PAGE_SIZE);
}

// Save takes ~20 ms (erase) + ~1 ms (program).  Uses the SDK's
// flash_safe_execute() which handles Core1 lockout + IRQ disable
// correctly (our old manual sequence could hang if Core1 or DMA
// touched XIP during the erase window).
static void persist_save_now(void) {
    persist_cfg_t c;
    persist_collect(&c);

    static uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));
    memcpy(page, &c, sizeof(c));

    int r = flash_safe_execute(persist_flash_op, page, 100);
    g_dbg_save_rc = r;
    if (r == PICO_OK) {
        g_persist_dirty = 0;
        g_dbg_save_ok++;
    } else {
        // Save failed (timeout, not permitted, etc.) — don't keep trying
        // back-to-back.  Push the dirty timestamp forward so the autosave
        // window waits another 5 s before retrying; otherwise every main
        // loop iteration would burn another 100 ms in flash_safe_execute
        // and the UI would feel totally frozen.
        g_persist_dirty_since = to_ms_since_boot(get_absolute_time());
    }
}

// Mark config dirty; the main loop autosaves after an idle period so
// rotations of the knob don't repeatedly erase flash.
static inline void persist_mark_dirty(void) {
    g_persist_dirty = 1;
    g_persist_dirty_since = to_ms_since_boot(get_absolute_time());
}

// Call from main loop.  If dirty and no changes for 5 s, save to flash.
// Suppresses saves while the user is actively navigating the menu, to
// avoid 20 ms flash-erase stalls during interaction.
static void persist_maybe_autosave(void) {
    if (!g_persist_dirty) return;
    if (g_ui_state != UI_STATE_TUNE) return;   // Only save when idle
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if ((now - g_persist_dirty_since) < 5000u) return;
    persist_save_now();
}


// ---------------- Command buffer ----------------
typedef struct {
    int32_t  freq_steps;
    int8_t   p_dbm;
    uint8_t  tx_on;
} sample_cmd_t;

static sample_cmd_t g_blocks[NUM_BLOCKS][BLOCK_SAMPLES];

static volatile uint32_t g_prod_block = 0;
static volatile uint32_t g_cons_block = 0;
static volatile uint8_t  g_block_ready[NUM_BLOCKS] = {0};
static volatile uint32_t g_underruns = 0;
static volatile uint8_t  g_core1_start = 0; 

// ==========================================================
// USB AUDIO IN (from PC) -> ringbuffer -> resampler to 8k mono
// ==========================================================

typedef struct { int16_t l, r; } stereo16_t;

// Power-of-two
#ifndef USB_RB_FRAMES
#define USB_RB_FRAMES 8192u
#endif

#if (USB_RB_FRAMES & (USB_RB_FRAMES - 1u)) != 0
#error "USB_RB_FRAMES must be power-of-two"
#endif

static stereo16_t g_usb_rb[USB_RB_FRAMES];
static volatile uint32_t g_usb_w = 0;
static volatile uint32_t g_usb_r = 0;

static volatile uint32_t g_usb_sample_rate_hz = 48000u; // current host SR

static inline uint32_t usb_rb_next(uint32_t x) { return (x + 1u) & (USB_RB_FRAMES - 1u); }

static inline bool usb_rb_push(stereo16_t s) {
    uint32_t w = g_usb_w;
    uint32_t n = usb_rb_next(w);
    if (n == g_usb_r) return false; // full
    g_usb_rb[w] = s;
    g_usb_w = n;
    return true;
}

static inline bool usb_rb_pop(stereo16_t *out) {
    uint32_t r = g_usb_r;
    if (r == g_usb_w) return false; // empty
    *out = g_usb_rb[r];
    g_usb_r = usb_rb_next(r);
    return true;
}

static inline int16_t clamp16(int32_t x) {
    if (x > 32767) return 32767;
    if (x < -32768) return -32768;
    return (int16_t)x;
}

// ==========================================================
// MIC ADC ring buffer + 8 kHz hardware timer
// Timer ISR reads ADC, removes DC, pushes int16_t to mic_rb.
// Main loop pops samples non-blocking (like USB path).
// ==========================================================
#define MIC_RB_SIZE 1024u  // Must be power-of-two
#if (MIC_RB_SIZE & (MIC_RB_SIZE - 1u)) != 0
#error "MIC_RB_SIZE must be power-of-two"
#endif

static int16_t g_mic_rb[MIC_RB_SIZE];
static volatile uint32_t g_mic_w = 0;
static volatile uint32_t g_mic_r = 0;

static inline uint32_t mic_rb_next(uint32_t x) { return (x + 1u) & (MIC_RB_SIZE - 1u); }

static inline bool mic_rb_push(int16_t s) {
    uint32_t w = g_mic_w;
    uint32_t n = mic_rb_next(w);
    if (n == g_mic_r) return false; // full — drop
    g_mic_rb[w] = s;
    g_mic_w = n;
    return true;
}

static inline bool mic_rb_pop(int16_t *out) {
    uint32_t r = g_mic_r;
    if (r == g_mic_w) return false; // empty
    *out = g_mic_rb[r];
    g_mic_r = mic_rb_next(r);
    return true;
}

// Timer ISR: lightweight ADC read + DC removal at 8 kHz
static bool mic_timer_callback(struct repeating_timer *t) {
    (void)t;

    // DC removal state (lives only in ISR context)
    static float dc_prev_x = 0.0f;
    static float dc_prev_y = 0.0f;
    const float DC_ALPHA = 0.995f;

    adc_select_input(0);
    uint16_t raw = adc_read();
    float x = ((float)raw - 2048.0f) / 2048.0f;  // Normalize to ±1.0

    // DC removal (single-pole IIR HPF)
    float y = x - dc_prev_x + DC_ALPHA * dc_prev_y;
    dc_prev_x = x;
    dc_prev_y = y;

    // Scale to int16 and push to ring buffer
    int16_t s = clamp16((int32_t)(y * 32767.0f));
    mic_rb_push(s);

    return true;  // keep repeating
}

static struct repeating_timer g_mic_timer;
static volatile bool g_mic_timer_running = false;

static void mic_timer_start(void) {
    if (g_mic_timer_running) return;
    // Flush ring buffer
    g_mic_w = 0;
    g_mic_r = 0;
    // Negative period = exact interval (accounts for callback duration)
    add_repeating_timer_us(-125, mic_timer_callback, NULL, &g_mic_timer);
    g_mic_timer_running = true;
}

static void mic_timer_stop(void) {
    if (!g_mic_timer_running) return;
    cancel_repeating_timer(&g_mic_timer);
    g_mic_timer_running = false;
}

// Read host audio (PCM16LE stereo), push to ringbuffer
static void usb_audio_pump(void);

typedef enum {
    AUDIO_MODULATION_SSB = 0,
    AUDIO_MODULATION_DIGITAL
} audio_modulation_t;

static volatile audio_modulation_t g_audio_modulation = AUDIO_MODULATION_SSB;

static int16_t interpolate_stereo_to_mono(const uint32_t phase_q16, 
                                        const stereo16_t s0, 
                                        const stereo16_t s1,
                                        const stereo16_t s2,
                                        const stereo16_t sm1) {
    // Cubic Hermite interpolation for smoother audio
    float t = (float)phase_q16 / 65536.0f;
    float t2 = t * t;
    float t3 = t2 * t;
    
    // Hermite basis functions
    float h00 = 2*t3 - 3*t2 + 1;
    float h10 = t3 - 2*t2 + t;
    float h01 = -2*t3 + 3*t2;
    float h11 = t3 - t2;
    
    // Left channel
    float m0_l = (float)(s1.l - sm1.l) * 0.5f;
    float m1_l = (float)(s2.l - s0.l) * 0.5f;
    float l = h00 * s0.l + h10 * m0_l + h01 * s1.l + h11 * m1_l;
    
    // Right channel
    float m0_r = (float)(s1.r - sm1.r) * 0.5f;
    float m1_r = (float)(s2.r - s0.r) * 0.5f;
    float r = h00 * s0.r + h10 * m0_r + h01 * s1.r + h11 * m1_r;

    float mono = (l + r) * 0.5f;
    return clamp16((int32_t)mono);
}

static void update_resampler_phase_and_samples(uint32_t *phase_q16,
                                        const uint32_t step_q16,
                                        stereo16_t *s0,
                                        stereo16_t *s1,
                                        stereo16_t *s2,
                                        stereo16_t *sm1) {

    *phase_q16 += step_q16;
    while (*phase_q16 >= (1u << 16)) {
        *phase_q16 -= (1u << 16);
        *sm1 = *s0;
        *s0 = *s1;
        *s1 = *s2;
        if (!usb_rb_pop(s2)) *s2 = *s1;  // Hold last value if empty
    }
}
static uint32_t calculate_resampling_step(const uint32_t base_step_q16,
                                        const uint32_t usb_w,
                                        const uint32_t usb_r,
                                        uint32_t *smooth_step_q16) {
    // *** Adaptive rate with heavy smoothing ***

    const uint32_t fill = (usb_w >= usb_r) ? (usb_w - usb_r) : (USB_RB_FRAMES - usb_r + usb_w);

    const uint32_t target_fill = USB_RB_FRAMES / 2;
    uint32_t target_step = base_step_q16;
    
    if (fill > target_fill) {
        uint32_t excess = fill - target_fill;
        uint32_t adj = (base_step_q16 * excess) / (USB_RB_FRAMES * 10);
        target_step = base_step_q16 + adj;
    } else if (fill < target_fill) {
        uint32_t deficit = target_fill - fill;
        uint32_t adj = (base_step_q16 * deficit) / (USB_RB_FRAMES * 10);
        target_step = base_step_q16 - adj;
    }
    
    // Heavy smoothing: move only 1/256 of the way to target each sample
    // This prevents audible pitch wobble
    if (*smooth_step_q16 < target_step) {
        uint32_t diff = target_step - *smooth_step_q16;
        *smooth_step_q16 += (diff >> 8) + 1;
        if (*smooth_step_q16 > target_step) *smooth_step_q16 = target_step;
    } else if (*smooth_step_q16 > target_step) {
        uint32_t diff = *smooth_step_q16 - target_step;
        *smooth_step_q16 -= (diff >> 8) + 1;
        if (*smooth_step_q16 < target_step) *smooth_step_q16 = target_step;
    }
    return *smooth_step_q16;
}

static int16_t usb_audio_get_mono_8k_int(const audio_modulation_t modulation) {
    static uint32_t src_rate = 48000u;
    static uint32_t base_step_q16 = 0;
    static uint32_t smooth_step_q16 = 0;  // Smoothed step for gradual changes
    static uint32_t phase_q16 = 0;

    static stereo16_t s0 = {0,0};
    static stereo16_t s1 = {0,0};
    static stereo16_t s2 = {0,0};  // Extra sample for cubic interpolation
    static stereo16_t sm1 = {0,0}; // s[-1] for cubic
    static bool primed = false;

    uint32_t sr = g_usb_sample_rate_hz;
    if (!sr) sr = 48000u;

    if (sr != src_rate || base_step_q16 == 0) {
        src_rate = sr;
        base_step_q16 = (uint32_t)(((uint64_t)src_rate << 16) / (uint32_t)WAV_SAMPLE_RATE);
        smooth_step_q16 = base_step_q16;
    }

    uint32_t step_q16;
    
    if (modulation == AUDIO_MODULATION_DIGITAL) {
        step_q16 = base_step_q16;
        smooth_step_q16 = base_step_q16;
    } else {
        //Adaptive resampling for non-digi
        step_q16 = calculate_resampling_step(base_step_q16, g_usb_w, g_usb_r, &smooth_step_q16);
    }

    if (!primed) {
        if (!usb_rb_pop(&sm1)) sm1 = (stereo16_t){0,0};
        if (!usb_rb_pop(&s0)) s0 = (stereo16_t){0,0};
        if (!usb_rb_pop(&s1)) s1 = (stereo16_t){0,0};
        if (!usb_rb_pop(&s2)) s2 = (stereo16_t){0,0};
        phase_q16 = 0;
        primed = true;
    }

    update_resampler_phase_and_samples(&phase_q16, step_q16, &s0, &s1, &s2, &sm1);

    return interpolate_stereo_to_mono(phase_q16, s0, s1, s2, sm1);
}

// Resampler: host SR stereo -> 8 kHz mono
// With smoothed adaptive rate to prevent buffer overflow and pitch artifacts
static int16_t usb_audio_get_mono_8k(void) {
    return usb_audio_get_mono_8k_int(g_audio_modulation);
}

// ==========================================================
// MIC audio: pop from timer-driven ring buffer + apply AGC/gate.
// Analogous to usb_audio_get_mono_8k() but for ADC microphone.
// DC removal is done in timer ISR; AGC + gate are done here
// so they can use runtime-configurable params from cfg_local.
// Returns float in ±1.0 range, or 0.0 if no sample available.
// ==========================================================
static float adc_mic_get_sample(float agc_target, float agc_max_gain,
                                float agc_attack, float agc_release,
                                float gate_thresh) {
    // AGC: envelope follower + gain (persistent across calls)
    static float agc_env = 0.0f;
    static float agc_gain = 1.0f;

    const float AGC_MIN_GAIN = 0.1f;

    // Pop from mic ring buffer (filled by timer ISR)
    int16_t raw16;
    if (!mic_rb_pop(&raw16)) {
        return 0.0f;  // No sample available — return silence
    }

    float y = (float)raw16 / 32767.0f;

    // AGC envelope follower (peak detector)
    float abs_y = (y >= 0.0f) ? y : -y;
    if (abs_y > agc_env) {
        agc_env += agc_attack * (abs_y - agc_env);
    } else {
        agc_env += agc_release * (abs_y - agc_env);
    }

    // Noise gate: if envelope is below threshold, output silence
    if (agc_env < gate_thresh) {
        return 0.0f;
    }

    // Compute gain from envelope
    if (agc_env > 1e-6f) {
        agc_gain = agc_target / agc_env;
        if (agc_gain > agc_max_gain) agc_gain = agc_max_gain;
        if (agc_gain < AGC_MIN_GAIN) agc_gain = AGC_MIN_GAIN;
    }

    float out = y * agc_gain;

    // Hard limiter to prevent clipping
    if (out > 1.0f) out = 1.0f;
    if (out < -1.0f) out = -1.0f;

    return out;
}

// ==========================================================
// TinyUSB UAC1 control callbacks (sampling freq + feature unit)
// ==========================================================

// Minimal feature unit state (żeby Windows nie marudził)
static uint8_t  g_mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];
static int16_t  g_volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1]; // 1/256 dB

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void)rhport;
    (void)p_request;
    return true;
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void)rhport;
    (void)p_request;
    return true;
}

// EP sampling freq (UAC1)
bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
    (void)rhport;
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);

    if (ctrlSel == AUDIO10_EP_CTRL_SAMPLING_FREQ &&
        p_request->bRequest == AUDIO10_CS_REQ_SET_CUR) {
        TU_VERIFY(p_request->wLength == 3);
        uint32_t sr = tu_unaligned_read32(pBuff) & 0x00FFFFFF;
        if (sr) g_usb_sample_rate_hz = sr;
        return true;
    }
    return false;
}

bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);

    if (ctrlSel == AUDIO10_EP_CTRL_SAMPLING_FREQ &&
        p_request->bRequest == AUDIO10_CS_REQ_GET_CUR) {
        uint32_t sr = g_usb_sample_rate_hz;
        uint8_t freq[3] = {
            (uint8_t)(sr & 0xFF),
            (uint8_t)((sr >> 8) & 0xFF),
            (uint8_t)((sr >> 16) & 0xFF)
        };
        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, freq, sizeof(freq));
    }
    return false;
}

// Feature Unit (mute/volume) – UAC1 entity callbacks
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf) {
    (void)rhport;

    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel    = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID   = TU_U16_HIGH(p_request->wIndex);

    if (entityID != UAC1_ENTITY_FEATURE_UNIT) return false;

    switch (ctrlSel) {
        case AUDIO10_FU_CTRL_MUTE:
            if (p_request->bRequest == AUDIO10_CS_REQ_SET_CUR) {
                TU_VERIFY(p_request->wLength == 1);
                g_mute[channelNum] = buf[0];
                return true;
            }
            return false;

        case AUDIO10_FU_CTRL_VOLUME:
            if (p_request->bRequest == AUDIO10_CS_REQ_SET_CUR) {
                TU_VERIFY(p_request->wLength == 2);
                g_volume[channelNum] = (int16_t) tu_unaligned_read16(buf); // 1/256 dB
                return true;
            }
            return false;

        default:
            return false;
    }
}

bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel    = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID   = TU_U16_HIGH(p_request->wIndex);

    if (entityID != UAC1_ENTITY_FEATURE_UNIT) return false;

    switch (ctrlSel) {
        case AUDIO10_FU_CTRL_MUTE:
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &g_mute[channelNum], 1);

        case AUDIO10_FU_CTRL_VOLUME:
            switch (p_request->bRequest) {
                case AUDIO10_CS_REQ_GET_CUR:
                    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &g_volume[channelNum], sizeof(g_volume[channelNum]));
                case AUDIO10_CS_REQ_GET_MIN: {
                    int16_t min = (int16_t)(-90 * 256);
                    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &min, sizeof(min));
                }
                case AUDIO10_CS_REQ_GET_MAX: {
                    int16_t max = (int16_t)(0 * 256);
                    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &max, sizeof(max));
                }
                case AUDIO10_CS_REQ_GET_RES: {
                    int16_t res = (int16_t)(1 * 256);
                    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &res, sizeof(res));
                }
                default:
                    return false;
            }

        default:
            return false;
    }
}

#if CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
// Feedback method (dla UAC1 speaker+feedback)
void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf, audio_feedback_params_t *feedback_param) {
    (void)func_id;
    (void)alt_itf;
    feedback_param->method = AUDIO_FEEDBACK_METHOD_FIFO_COUNT;
    feedback_param->sample_freq = g_usb_sample_rate_hz ? g_usb_sample_rate_hz : 48000u;
}
#endif

// ==========================================================
// SX1280 low-level radio I/O (CORE1 ONLY)
// ==========================================================
static inline void cs_select(void)   { gpio_put(PIN_NSS, 0); }
static inline void cs_deselect(void) { gpio_put(PIN_NSS, 1); }

static inline void sx_wait_busy(void) {
    // Bounded wait — if BUSY stays high for >10 ms something is very wrong
    // (normally commands take <100 us).  Without a timeout, Core1 would hang
    // forever and the whole audio pipeline freezes.
    uint32_t guard = 0;
    while (gpio_get(PIN_BUSY)) {
        tight_loop_contents();
        if (++guard > 2000000u) {   // ~10 ms at 200 MHz busy-wait
            extern volatile uint32_t g_dbg_busy_timeouts;
            g_dbg_busy_timeouts++;
            return;
        }
    }
}

// Forward declarations for CDC functions
static void cdc_printf(const char *fmt, ...);
static void cdc_write_str(const char *s);

static void sx_write_cmd(uint8_t opcode, const uint8_t *params, size_t len) {
    extern volatile uint32_t g_dbg_core1_bc;
    uint32_t save_bc = g_dbg_core1_bc;
    g_dbg_core1_bc = save_bc + 100;
    sx_wait_busy();
    g_dbg_core1_bc = save_bc + 110;
    cs_select();
    spi_write_blocking(SX_SPI, &opcode, 1);
    g_dbg_core1_bc = save_bc + 120;
    if (len && params) {
        spi_write_blocking(SX_SPI, params, len);
    }
    g_dbg_core1_bc = save_bc + 130;
    cs_deselect();
    sx_wait_busy();
    g_dbg_core1_bc = save_bc;
}

static inline void sx_set_standby_rc(void) {
    uint8_t cfg = 0x00;  // STDBY_RC
    sx_write_cmd(OPCODE_SET_STANDBY, &cfg, 1);
}

static inline void sx_set_standby_xosc(void) {
    uint8_t cfg = 0x01;  // STDBY_XOSC - required for TCXO module
    sx_write_cmd(OPCODE_SET_STANDBY, &cfg, 1);
}

static inline void sx_set_packet_type_gfsk(void) {
    uint8_t pt = 0x00; // GFSK
    sx_write_cmd(OPCODE_SET_PACKET_TYPE, &pt, 1);
}

static inline uint8_t sx_encode_power_dbm(int32_t dbm) {
    if (dbm > 13)  dbm = 13;
    if (dbm < -18) dbm = -18;
    return (uint8_t)(dbm + 18); // 0..31
}

static inline void sx_set_tx_params_dbm(int32_t power_dbm) {
    uint8_t pwr = sx_encode_power_dbm(power_dbm);
    uint8_t p[2] = { pwr, RAMP_TIME };
    sx_write_cmd(OPCODE_SET_TX_PARAMS, p, 2);
}

static inline void sx_start_tx_continuous_wave(void) {
    sx_write_cmd(OPCODE_SET_TX_CW, NULL, 0);
}

static inline void sx_set_rf_frequency_steps(uint32_t steps) {
    uint8_t p[3] = {
        (uint8_t)(steps >> 16),
        (uint8_t)(steps >> 8),
        (uint8_t)(steps)
    };
    sx_write_cmd(OPCODE_SET_RF_FREQUENCY, p, 3);
}

static inline uint32_t hz_to_steps(uint32_t freq_hz) {
    return (uint32_t)((double)freq_hz / (double)PLL_STEP_HZ);
}

// Calculate corrected frequency with PPM
static inline double get_corrected_freq_hz(void) {
    return g_target_freq_hz * (1.0 + (double)g_ppm_correction / 1000000.0);
}

// Get base PLL steps (integer part)
static inline uint32_t get_base_steps(void) {
    double corrected_hz = get_corrected_freq_hz();
    return (uint32_t)(corrected_hz / (double)PLL_STEP_HZ);
}

// Get fine tune offset in Hz (fractional part that PLL can't reach)
static inline float get_fine_tune_hz(void) {
    double corrected_hz = get_corrected_freq_hz();
    uint32_t base_steps = (uint32_t)(corrected_hz / (double)PLL_STEP_HZ);
    double base_hz = (double)base_steps * (double)PLL_STEP_HZ;
    return (float)(corrected_hz - base_hz);
}

// Get SX1280 status byte
static uint8_t sx_get_status(void) {
    sx_wait_busy();
    cs_select();
    uint8_t cmd = 0xC0;  // GetStatus opcode
    uint8_t status;
    spi_write_blocking(SX_SPI, &cmd, 1);
    spi_read_blocking(SX_SPI, 0x00, &status, 1);
    cs_deselect();
    return status;
}

// Diagnostic: print SX1280 state via CDC
static void sx_print_diag(void) {
#if CFG_TUD_CDC
    uint8_t status = sx_get_status();
    uint8_t mode = (status >> 5) & 0x07;
    
    const char *mode_str = "UNKNOWN";
    switch (mode) {
        case 2: mode_str = "STDBY_RC"; break;
        case 3: mode_str = "STDBY_XOSC"; break;
        case 4: mode_str = "FS"; break;
        case 5: mode_str = "RX"; break;
        case 6: mode_str = "TX"; break;
    }
    
    cdc_printf("\r\n=== SX1280 Diagnostics ===\r\n");
    cdc_printf("Status: 0x%02X (mode=%d: %s)\r\n", status, mode, mode_str);
    cdc_printf("BUSY pin: %d\r\n", gpio_get(PIN_BUSY));
    cdc_printf("TX_EN pin: %d\r\n", gpio_get(PIN_TX_EN));
    cdc_printf("RX_EN pin: %d\r\n", gpio_get(PIN_RX_EN));
#if USE_TCXO_MODULE
    cdc_printf("TCXO_EN pin: %d\r\n", gpio_get(PIN_TCXO_EN));
#endif
    cdc_printf("Base freq: %lu Hz\r\n", (unsigned long)BASE_FREQ_HZ);
    cdc_printf("TX power max: %d dBm\r\n", g_tx_power_max_dbm);
    
    // Buffer diagnostics
    uint32_t prod = g_prod_block;
    uint32_t cons = g_cons_block;
    uint32_t ready_count = 0;
    for (uint32_t i = 0; i < NUM_BLOCKS; i++) {
        if (g_block_ready[i]) ready_count++;
    }
    cdc_printf("Blocks: prod=%lu cons=%lu ready=%lu/%lu\r\n", 
               (unsigned long)prod, (unsigned long)cons, 
               (unsigned long)ready_count, (unsigned long)NUM_BLOCKS);
    cdc_printf("Underruns: %lu\r\n", (unsigned long)g_underruns);
    
    // USB audio buffer
    uint32_t usb_w = g_usb_w;
    uint32_t usb_r = g_usb_r;
    uint32_t usb_fill = (usb_w >= usb_r) ? (usb_w - usb_r) : (USB_RB_FRAMES - usb_r + usb_w);
    cdc_printf("USB ringbuf: %lu/%lu frames\r\n", (unsigned long)usb_fill, (unsigned long)USB_RB_FRAMES);
    cdc_printf("==========================\r\n");
#endif
}

// Apply current freq + power to SX1280 while in CW/TUNE mode.
// Uses fractional-step dithering for sub-PLL-step precision (same as SSB).
// Call repeatedly from carrier_poll() — each call advances the dither accumulator.
static float g_cw_freq_acc = 0.0f;  // Fractional step accumulator for CW dithering

static void tune_apply_settings(void) {
    uint32_t base = get_base_steps();
    float fine_hz = get_fine_tune_hz();

    // Dither: accumulate fractional PLL step, toggle +1 when it overflows
    float want_frac = fine_hz / PLL_STEP_HZ;
    int32_t Nf = (int32_t)floorf(want_frac);
    float ffrac = want_frac - (float)Nf;

    g_cw_freq_acc += ffrac;
    int32_t chosen = Nf;
    if (g_cw_freq_acc >= 1.0f) {
        chosen = Nf + 1;
        g_cw_freq_acc -= 1.0f;
    }

    sx_set_rf_frequency_steps((uint32_t)((int32_t)base + chosen));
    sx_set_tx_params_dbm(g_tx_power_max_dbm);
}

// Pump USB while waiting (keep USB alive during short delays)
static void usb_aware_delay_ms(uint32_t ms) {
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - start) < ms) {
        tud_task();
    }
}

// Smoothly ramp the transmitter power between two levels by stepping
// sx_set_tx_params_dbm() with a raised-cosine envelope.  This prevents
// key-click in CW and PTT-click in FM.  Runs on Core0 and blocks for
// `ms` milliseconds total.  Callers must already own SPI (carrier mode
// or Core1 idled).
#define CW_RAMP_MS       8u       // ramp duration for CW key-down/up
#define CW_RAMP_STEPS    10
static void sx_carrier_ramp_power(int8_t from_dbm, int8_t to_dbm, uint32_t ms) {
    if (ms == 0) { sx_set_tx_params_dbm(to_dbm); return; }
    uint32_t us_per_step = (ms * 1000u) / CW_RAMP_STEPS;
    int32_t  span = (int32_t)to_dbm - (int32_t)from_dbm;
    for (int i = 1; i <= CW_RAMP_STEPS; i++) {
        float frac = (float)i / (float)CW_RAMP_STEPS;
        // Raised-cosine 0→1 (smooth start and end, no dc-step)
        float env = 0.5f * (1.0f - cosf((float)M_PI * frac));
        int32_t dbm = (int32_t)((float)from_dbm + env * (float)span + 0.5f);
        if (dbm < PWR_MIN_DBM) dbm = PWR_MIN_DBM;
        if (dbm > PWR_MAX_DBM) dbm = PWR_MAX_DBM;
        sx_set_tx_params_dbm(dbm);
        busy_wait_us_32(us_per_step);
    }
    sx_set_tx_params_dbm(to_dbm);
}

// Start CW/TUNE carrier (Core0 only).
// PRECONDITION: g_cw_test_mode must already be 1 and Core1 must be idle.
static void sx_start_carrier(void) {
    // Ensure TCXO is on (pump USB while waiting)
#if USE_TCXO_MODULE
    gpio_put(PIN_TCXO_EN, 1);
    usb_aware_delay_ms(5);
#endif

    // Full clean init: standby -> packet type -> freq -> power -> CW
#if USE_TCXO_MODULE
    sx_set_standby_xosc();
#else
    sx_set_standby_rc();
#endif

    sx_set_packet_type_gfsk();

    uint32_t steps = get_base_steps();
    sx_set_rf_frequency_steps(steps);
    // Start at minimum power; the raised-cosine ramp below takes us
    // from silence up to the user-set target, eliminating CW key-click.
    sx_set_tx_params_dbm(PWR_MIN_DBM);

    gpio_put(PIN_TX_EN, 1);
    gpio_put(PIN_RX_EN, 0);

    sx_start_tx_continuous_wave();
    usb_aware_delay_ms(2);

    // Safety: re-apply frequency AFTER CW start
    sx_set_rf_frequency_steps(steps);

    // Smooth envelope ramp up to target power
    sx_carrier_ramp_power(PWR_MIN_DBM, g_tx_power_max_dbm, CW_RAMP_MS);
}

// Stop CW carrier and restore for SSB (Core0 only)
static void sx_stop_carrier(void) {
    // Smoothly ramp power down before killing the carrier — this
    // suppresses the classic CW key-up click/thump.
    sx_carrier_ramp_power(g_tx_power_max_dbm, PWR_MIN_DBM, CW_RAMP_MS);

#if USE_TCXO_MODULE
    sx_set_standby_xosc();
#else
    sx_set_standby_rc();
#endif

    // Re-initialize radio for normal SSB operation
    sx_set_packet_type_gfsk();
    sx_set_rf_frequency_steps(get_base_steps());
    sx_set_tx_params_dbm((int32_t)g_tx_power_max_dbm);

    gpio_put(PIN_TX_EN, 1);
    gpio_put(PIN_RX_EN, 0);

    // Resume normal Core1 operation
    g_cw_test_mode = 0;
    __compiler_memory_barrier();
}

#define CW_ARM_WAIT_MS  35  // Time to wait for Core1 to finish SPI (> 1 block = 32ms)

// Start CW/TUNE transmission (legacy wrapper with CDC logging)
// This is a blocking call (used by CDC "cw" command and TUNE encoder action).
// It idles Core1, waits (USB-aware), then starts carrier.
static void sx_test_cw(void) {
#if CFG_TUD_CDC
    cdc_printf("\r\n*** Starting CW/TUNE ***\r\n");
#endif
    // Idle Core1 and wait (USB-aware, not blocking)
    if (!g_cw_test_mode) {
        g_cw_test_mode = 1;
        __compiler_memory_barrier();
        usb_aware_delay_ms(CW_ARM_WAIT_MS);
    }
    sx_start_carrier();
#if CFG_TUD_CDC
    uint8_t status = sx_get_status();
    cdc_printf("TUNE: freq=%.1f Hz, steps=%lu, pwr=%d dBm, status=0x%02X\r\n",
               get_corrected_freq_hz(), (unsigned long)get_base_steps(),
               g_tx_power_max_dbm, status);
#endif
}

// Stop CW transmission and restore normal SSB mode (legacy wrapper)
static void sx_stop_cw(void) {
    sx_stop_carrier();
#if CFG_TUD_CDC
    cdc_printf("TX stopped, radio re-initialized for SSB\r\n");
#endif
}

// ==========================================================
// Unified carrier state machine (runs on Core0 in polling loop)
//
// Inputs:  g_tx_mode (0=USB/SSB, 1=CW, 2=FM), g_tune_active, g_ptt_key
// Outputs: g_cw_test_mode, SPI carrier on/off
//
// Logic:
//   need_idle  = (g_tx_mode==1) || g_tune_active     → Core1 must idle
//   need_carrier = g_tune_active || (g_tx_mode==1 && g_ptt_key)  → CW on
//   FM mode (g_tx_mode==2) behaves like SSB — Core1 owns SPI, blocks flow.
//
// States:
//   IDLE    → need_idle? set g_cw_test_mode=1, go ARMING
//   ARMING  → wait 35ms for Core1, go ARMED
//   ARMED   → need_carrier? sx_start_carrier → CARRIER_ON
//   CARRIER_ON → !need_carrier? standby → ARMED
//   any     → !need_idle? stop carrier if on, clear g_cw_test_mode → IDLE
// ==========================================================
typedef enum {
    CR_ST_IDLE = 0,     // Normal SSB mode — Core1 owns SPI
    CR_ST_ARMING,       // g_cw_test_mode=1 set, waiting for Core1 to idle
    CR_ST_ARMED,        // Core1 is idle, no carrier (standby)
    CR_ST_CARRIER_ON    // CW carrier active
} carrier_state_t;

static carrier_state_t g_cr_state = CR_ST_IDLE;
static uint32_t        g_cr_arm_start_ms = 0;

static void carrier_poll(void) {
    bool mode_cw     = (g_tx_mode == 1);
    bool tune        = (bool)g_tune_active;
    bool key         = (bool)g_ptt_key;

    bool need_idle    = mode_cw || tune;
    bool need_carrier = tune || (mode_cw && key);

    // Guard window after mode change: force carrier off so CW/TUNE
    // doesn't briefly latch while the state machine settles.
    if (tx_mode_guard_active()) need_carrier = false;

    uint32_t now = to_ms_since_boot(get_absolute_time());

    switch (g_cr_state) {

    case CR_ST_IDLE:
        if (need_idle) {
            g_cw_test_mode = 1;
            __compiler_memory_barrier();
            g_cr_arm_start_ms = now;
            g_cr_state = CR_ST_ARMING;
        }
        break;

    case CR_ST_ARMING:
        if (!need_idle) {
            // No longer need idle — cancel
            g_cw_test_mode = 0;
            __compiler_memory_barrier();
            g_cr_state = CR_ST_IDLE;
            break;
        }
        if ((now - g_cr_arm_start_ms) >= CW_ARM_WAIT_MS) {
            // Core1 is now idle — force radio to known standby state
            // (Core1 may have left it in TX from SSB)
#if USE_TCXO_MODULE
            sx_set_standby_xosc();
#else
            sx_set_standby_rc();
#endif
            g_cr_state = CR_ST_ARMED;
        }
        break;

    case CR_ST_ARMED:
        if (!need_idle) {
            // Return to SSB — restore Core1
            g_cw_test_mode = 0;
            __compiler_memory_barrier();
            g_cr_state = CR_ST_IDLE;
            break;
        }
        if (need_carrier) {
            g_cw_freq_acc = 0.0f;  // Reset dither accumulator
            sx_start_carrier();
            g_cr_state = CR_ST_CARRIER_ON;
        }
        break;

    case CR_ST_CARRIER_ON:
        if (!need_idle) {
            // Leaving CW/TUNE entirely — full stop + restore SSB
            sx_stop_carrier();   // clears g_cw_test_mode
            g_cr_state = CR_ST_IDLE;
            break;
        }
        if (!need_carrier) {
            // Carrier off but stay armed (e.g. CW key released, or TUNE off
            // but still CW mode).  Ramp down first to avoid key-up click.
            sx_carrier_ramp_power(g_tx_power_max_dbm, PWR_MIN_DBM, CW_RAMP_MS);
#if USE_TCXO_MODULE
            sx_set_standby_xosc();
#else
            sx_set_standby_rc();
#endif
            g_cr_state = CR_ST_ARMED;
        } else {
            // Carrier on — dither freq at ~8 kHz (125 µs), same rate as SSB
            static uint64_t next_dither_us = 0;
            uint64_t now_us = time_us_64();
            if (now_us >= next_dither_us) {
                tune_apply_settings();
                next_dither_us = now_us + 125;  // 125 µs = 8 kHz
            }
        }
        break;
    }
}

// ==========================================================
// DSP blocks (biquads + compressor + hilbert)
// ==========================================================
typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float z1, z2;
} biquad_t;

static inline void biquad_reset(biquad_t *q) { q->z1 = 0.0f; q->z2 = 0.0f; }

static inline float biquad_process(biquad_t *q, float x) {
    float y = q->b0 * x + q->z1;
    q->z1 = q->b1 * x - q->a1 * y + q->z2;
    q->z2 = q->b2 * x - q->a2 * y;
    return y;
}

static void biquad_init_lowpass_bw2(biquad_t *q, float fc, float fs) {
    const float K  = tanf((float)M_PI * fc / fs);
    const float K2 = K * K;
    const float s2 = 1.41421356f;
    const float norm = 1.0f / (1.0f + s2 * K + K2);

    q->b0 = K2 * norm;
    q->b1 = 2.0f * q->b0;
    q->b2 = q->b0;

    q->a1 = 2.0f * (K2 - 1.0f) * norm;
    q->a2 = (1.0f - s2 * K + K2) * norm;

    biquad_reset(q);
}

static void biquad_init_highpass_bw2(biquad_t *q, float fc, float fs) {
    const float K  = tanf((float)M_PI * fc / fs);
    const float K2 = K * K;
    const float s2 = 1.41421356f;
    const float norm = 1.0f / (1.0f + s2 * K + K2);

    q->b0 = 1.0f * norm;
    q->b1 = -2.0f * q->b0;
    q->b2 = q->b0;

    q->a1 = 2.0f * (K2 - 1.0f) * norm;
    q->a2 = (1.0f - s2 * K + K2) * norm;

    biquad_reset(q);
}

static void biquad_init_low_shelf(biquad_t *q, float fc, float fs, float gain_db) {
    const float A = powf(10.0f, gain_db / 40.0f);
    const float w0 = 2.0f * (float)M_PI * fc / fs;
    const float cw = cosf(w0);
    const float sw = sinf(w0);
    const float alpha = sw * 0.5f * 1.41421356f;

    float b0 =    A*((A+1.0f) - (A-1.0f)*cw + 2.0f*sqrtf(A)*alpha);
    float b1 =  2.0f*A*((A-1.0f) - (A+1.0f)*cw);
    float b2 =    A*((A+1.0f) - (A-1.0f)*cw - 2.0f*sqrtf(A)*alpha);
    float a0 =        (A+1.0f) + (A-1.0f)*cw + 2.0f*sqrtf(A)*alpha;
    float a1 =   -2.0f*((A-1.0f) + (A+1.0f)*cw);
    float a2 =        (A+1.0f) + (A-1.0f)*cw - 2.0f*sqrtf(A)*alpha;

    q->b0 = b0 / a0; q->b1 = b1 / a0; q->b2 = b2 / a0;
    q->a1 = a1 / a0; q->a2 = a2 / a0;
    biquad_reset(q);
}

static void biquad_init_high_shelf(biquad_t *q, float fc, float fs, float gain_db) {
    const float A = powf(10.0f, gain_db / 40.0f);
    const float w0 = 2.0f * (float)M_PI * fc / fs;
    const float cw = cosf(w0);
    const float sw = sinf(w0);
    const float alpha = sw * 0.5f * 1.41421356f;

    float b0 =    A*((A+1.0f) + (A-1.0f)*cw + 2.0f*sqrtf(A)*alpha);
    float b1 = -2.0f*A*((A-1.0f) + (A+1.0f)*cw);
    float b2 =    A*((A+1.0f) + (A-1.0f)*cw - 2.0f*sqrtf(A)*alpha);
    float a0 =        (A+1.0f) - (A-1.0f)*cw + 2.0f*sqrtf(A)*alpha;
    float a1 =    2.0f*((A-1.0f) - (A+1.0f)*cw);
    float a2 =        (A+1.0f) - (A-1.0f)*cw - 2.0f*sqrtf(A)*alpha;

    q->b0 = b0 / a0; q->b1 = b1 / a0; q->b2 = b2 / a0;
    q->a1 = a1 / a0; q->a2 = a2 / a0;
    biquad_reset(q);
}

typedef struct {
    float env;
    float a_att, a_rel;
    float thr_db, ratio, makeup_lin, knee_db;
} compressor_t;

static inline float compressor_gain_db(const compressor_t *c, float in_db) {
    const float thr = c->thr_db;
    const float r = c->ratio;

    if (c->knee_db <= 0.0f) {
        if (in_db <= thr) return 0.0f;
        float out_db = thr + (in_db - thr) / r;
        return out_db - in_db;
    }

    const float k = c->knee_db;
    const float x0 = thr - k * 0.5f;
    const float x1 = thr + k * 0.5f;

    if (in_db <= x0) return 0.0f;
    if (in_db >= x1) {
        float out_db = thr + (in_db - thr) / r;
        return out_db - in_db;
    }

    const float t = (in_db - x0) / (x1 - x0);
    float out1 = thr + (x1 - thr) / r;
    float g1 = out1 - x1;
    return g1 * t * t;
}

static inline float compressor_process(compressor_t *c, float x) {
    float ax = fabsf(x);
    if (ax > c->env) c->env = c->a_att * c->env + (1.0f - c->a_att) * ax;
    else            c->env = c->a_rel * c->env + (1.0f - c->a_rel) * ax;

    float env = fmaxf(c->env, 1e-8f);
    float in_db = 20.0f * log10f(env);

    float g_db = compressor_gain_db(c, in_db);
    float g_lin = powf(10.0f, g_db / 20.0f) * c->makeup_lin;

    return x * g_lin;
}

// ==========================================================
// Runtime-configurable DSP settings (over USB CDC)
// ==========================================================
typedef struct {
    uint8_t enable_bandpass;
    uint8_t enable_eq;
    uint8_t enable_comp;

    float bp_lo_hz;
    float bp_hi_hz;
    uint8_t bp_stages;      // 1-10, each stage = 12 dB/octave

    float eq_low_hz;
    float eq_low_db;
    float eq_high_hz;
    float eq_high_db;

    float comp_thr_db;
    float comp_ratio;
    float comp_attack_ms;
    float comp_release_ms;
    float comp_makeup_db;
    float comp_knee_db;
    float comp_out_limit;

    float amp_gain;
    float amp_min_a;

    // MIC AGC (for ADC microphone input)
    float mic_agc_target;    // Target envelope level (0..1)
    float mic_agc_max_gain;  // Maximum gain (prevents noise pumping in silence)
    float mic_agc_attack;    // Attack coefficient (0..1, higher = faster)
    float mic_agc_release;   // Release coefficient (0..1, higher = faster)
    float mic_gate_thresh;   // Noise gate threshold — below this, output is zero
} audio_cfg_t;

static volatile audio_cfg_t g_cfg = {
    .enable_bandpass = AUDIO_ENABLE_BANDPASS,
    .enable_eq       = AUDIO_ENABLE_EQ,
    .enable_comp     = AUDIO_ENABLE_COMPRESSOR,

    .bp_lo_hz  = AUDIO_BP_LO_HZ,
    .bp_hi_hz  = AUDIO_BP_HI_HZ,
    .bp_stages = AUDIO_BP_DEFAULT_STAGES,

    .eq_low_hz  = EQ_LOW_SHELF_HZ,
    .eq_low_db  = EQ_LOW_SHELF_DB,
    .eq_high_hz = EQ_HIGH_SHELF_HZ,
    .eq_high_db = EQ_HIGH_SHELF_DB,

    .comp_thr_db     = COMP_THRESHOLD_DB,
    .comp_ratio      = COMP_RATIO,
    .comp_attack_ms  = COMP_ATTACK_MS,
    .comp_release_ms = COMP_RELEASE_MS,
    .comp_makeup_db  = COMP_MAKEUP_DB,
    .comp_knee_db    = COMP_KNEE_DB,
    .comp_out_limit  = COMP_OUTPUT_LIMIT,

    .amp_gain  = AMP_GAIN,
    .amp_min_a = AMP_MIN_A,

    .mic_agc_target   = 0.75f,
    .mic_agc_max_gain = 1.0f,
    .mic_agc_attack   = 0.01f,
    .mic_agc_release  = 0.0001f,
    .mic_gate_thresh  = 0.005f,   // Below this envelope, output is silenced
};
static volatile uint8_t g_cfg_dirty = 1;

static void compressor_reconfig(compressor_t *c, float fs, const audio_cfg_t *cfg) {
    c->env = 0.0f;

    const float att_s = cfg->comp_attack_ms  * 0.001f;
    const float rel_s = cfg->comp_release_ms * 0.001f;
    c->a_att = expf(-1.0f / (fmaxf(att_s, 1e-4f) * fs));
    c->a_rel = expf(-1.0f / (fmaxf(rel_s, 1e-4f) * fs));

    c->thr_db = cfg->comp_thr_db;
    c->ratio  = fmaxf(cfg->comp_ratio, 1.0f);
    c->makeup_lin = powf(10.0f, cfg->comp_makeup_db / 20.0f);
    c->knee_db    = fmaxf(cfg->comp_knee_db, 0.0f);
}

static void cfg_sanitize(audio_cfg_t *c, float fs) {
    if (c->bp_lo_hz < 50.0f) c->bp_lo_hz = 50.0f;
    float max_hi = fs * 0.45f;
    if (c->bp_hi_hz > max_hi) c->bp_hi_hz = max_hi;
    if (c->bp_hi_hz <= c->bp_lo_hz + 50.0f) c->bp_hi_hz = c->bp_lo_hz + 50.0f;

    if (c->eq_low_hz < 50.0f) c->eq_low_hz = 50.0f;
    if (c->eq_low_hz > fs * 0.45f) c->eq_low_hz = fs * 0.45f;

    if (c->eq_high_hz < 50.0f) c->eq_high_hz = 50.0f;
    if (c->eq_high_hz > fs * 0.45f) c->eq_high_hz = fs * 0.45f;

    if (c->comp_ratio < 1.0f) c->comp_ratio = 1.0f;
    if (c->comp_attack_ms < 0.1f) c->comp_attack_ms = 0.1f;
    if (c->comp_release_ms < 1.0f) c->comp_release_ms = 1.0f;

    if (c->comp_out_limit < 0.05f) c->comp_out_limit = 0.05f;
    if (c->comp_out_limit > 0.999f) c->comp_out_limit = 0.999f;

    if (c->amp_gain < 0.01f) c->amp_gain = 0.01f;
    if (c->amp_min_a < 1e-9f) c->amp_min_a = 1e-9f;

    // MIC AGC
    if (c->mic_agc_target < 0.01f) c->mic_agc_target = 0.01f;
    if (c->mic_agc_target > 1.0f) c->mic_agc_target = 1.0f;
    if (c->mic_agc_max_gain < 1.0f) c->mic_agc_max_gain = 1.0f;
    if (c->mic_agc_max_gain > 200.0f) c->mic_agc_max_gain = 200.0f;
    if (c->mic_agc_attack < 0.0001f) c->mic_agc_attack = 0.0001f;
    if (c->mic_agc_attack > 0.5f) c->mic_agc_attack = 0.5f;
    if (c->mic_agc_release < 0.00001f) c->mic_agc_release = 0.00001f;
    if (c->mic_agc_release > 0.1f) c->mic_agc_release = 0.1f;
    if (c->mic_gate_thresh < 0.0f) c->mic_gate_thresh = 0.0f;
    if (c->mic_gate_thresh > 0.5f) c->mic_gate_thresh = 0.5f;

    // Clamp bp_stages to valid range
    if (c->bp_stages < 1) c->bp_stages = 1;
    if (c->bp_stages > AUDIO_BP_MAX_STAGES) c->bp_stages = AUDIO_BP_MAX_STAGES;
}

static void apply_cfg_if_dirty(float Fs,
                              biquad_t *bp_hpf, biquad_t *bp_lpf,
                              biquad_t *eq_low, biquad_t *eq_high,
                              compressor_t *comp,
                              audio_cfg_t *out_cfg)
{
    if (!g_cfg_dirty) return;

    audio_cfg_t tmp;
    __compiler_memory_barrier();
    memcpy(&tmp, (const void*)&g_cfg, sizeof(tmp));
    __compiler_memory_barrier();

    cfg_sanitize(&tmp, Fs);

#if AUDIO_BP_MAX_STAGES
    for (int i = 0; i < AUDIO_BP_MAX_STAGES; i++) {
        biquad_init_highpass_bw2(&bp_hpf[i], tmp.bp_lo_hz, Fs);
        biquad_init_lowpass_bw2 (&bp_lpf[i], tmp.bp_hi_hz, Fs);
    }
#else
    (void)bp_hpf; (void)bp_lpf;
#endif

    biquad_init_low_shelf (eq_low,  tmp.eq_low_hz,  Fs, tmp.eq_low_db);
    biquad_init_high_shelf(eq_high, tmp.eq_high_hz, Fs, tmp.eq_high_db);

    compressor_reconfig(comp, Fs, &tmp);

    *out_cfg = tmp;

    __compiler_memory_barrier();
    g_cfg_dirty = 0;
    __compiler_memory_barrier();
}
// ==========================================================
// Simple USB CDC command interface (enabled only if CDC exists)
// ==========================================================
static int streqi(const char *a, const char *b) {
    if (!a || !b) return 0;
    while (*a && *b) {
        char ca = *a++, cb = *b++;
        if (ca >= 'A' && ca <= 'Z') ca = (char)(ca - 'A' + 'a');
        if (cb >= 'A' && cb <= 'Z') cb = (char)(cb - 'A' + 'a');
        if (ca != cb) return 0;
    }
    return (*a == 0 && *b == 0);
}

static void cdc_write_str(const char *s) {
#if CFG_TUD_CDC
    if (!tud_cdc_connected()) return;
    tud_cdc_write_str(s);
    tud_cdc_write_flush();
#else
    (void)s;
#endif
}

static void cdc_printf(const char *fmt, ...) {
#if CFG_TUD_CDC
    if (!tud_cdc_connected()) return;
    char b[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    tud_cdc_write_str(b);
    tud_cdc_write_flush();
#else
    (void)fmt;
#endif
}

static bool parse_bool(const char *s, uint8_t *out) {
    if (!s) return false;
    if (streqi(s, "1") || streqi(s, "on") || streqi(s, "true"))  { *out = 1; return true; }
    if (streqi(s, "0") || streqi(s, "off")|| streqi(s, "false")) { *out = 0; return true; }
    return false;
}

static bool parse_f(const char *s, float *out) {
    if (!s) return false;
    char *e = NULL;
    float v = strtof(s, &e);
    if (e == s) return false;
    *out = v;
    return true;
}

// Format large Hz value as "XXXXXXXXXX.X" into caller's buffer.
// Needed because newlib-nano's printf uses scientific notation for big doubles.
static void fmt_freq(char *buf, size_t len, double hz) {
    uint64_t i = (uint64_t)hz;
    uint32_t f = (uint32_t)((hz - (double)i) * 10.0 + 0.5);
    if (f >= 10) { i++; f = 0; }
    snprintf(buf, len, "%llu.%u", (unsigned long long)i, (unsigned)f);
}

static void cfg_print(void) {
    audio_cfg_t c;
    __compiler_memory_barrier();
    memcpy(&c, (const void*)&g_cfg, sizeof(c));
    __compiler_memory_barrier();

    double corrected = get_corrected_freq_hz();
    float fine = get_fine_tune_hz();

    char freq_str[24], corr_str[24];
    fmt_freq(freq_str, sizeof(freq_str), g_target_freq_hz);
    fmt_freq(corr_str, sizeof(corr_str), corrected);

    cdc_printf(
        "CFG:\r\n"
        "  freq=%s Hz (target)  ppm=%.3f  tx=%s  txpwr=%d dBm\r\n"
        "  mode=%s  tune=%s\r\n"
        "  corrected=%s Hz  base_steps=%lu  fine=%.1f Hz (auto)\r\n",
        freq_str, g_ppm_correction, g_tx_enabled ? "ON" : "OFF", g_tx_power_max_dbm,
        mode_label(g_tx_mode),
        g_tune_active ? "ON" : "OFF",
        corr_str, (unsigned long)get_base_steps(), fine);
    cdc_printf(
        "  audio_mode=%s resampler=%s\r\n",
        g_audio_modulation == AUDIO_MODULATION_DIGITAL ? "DIGITAL" : "SSB",
        g_audio_modulation == AUDIO_MODULATION_DIGITAL ? "FIXED" : "ADAPTIVE"
    );        
    cdc_printf(
        "  enable bp=%u eq=%u comp=%u\r\n"
        "  bp_lo=%.1f bp_hi=%.1f bp_stages=%u (%u dB/oct)\r\n"
        "  eq_low_hz=%.1f eq_low_db=%.1f\r\n"
        "  eq_high_hz=%.1f eq_high_db=%.1f\r\n"
        "  comp_thr=%.1f ratio=%.2f att=%.2fms rel=%.2fms makeup=%.1f knee=%.1f outlim=%.3f\r\n"
        "  amp_gain=%.3f amp_min_a=%.9f\r\n",
        c.enable_bandpass, c.enable_eq, c.enable_comp,
        c.bp_lo_hz, c.bp_hi_hz, c.bp_stages, c.bp_stages * 12,
        c.eq_low_hz, c.eq_low_db,
        c.eq_high_hz, c.eq_high_db,
        c.comp_thr_db, c.comp_ratio, c.comp_attack_ms, c.comp_release_ms, c.comp_makeup_db, c.comp_knee_db, c.comp_out_limit,
        c.amp_gain, c.amp_min_a
    );
    cdc_printf(
        "  mic_agc_target=%.3f mic_agc_max_gain=%.1f mic_agc_attack=%.4f mic_agc_release=%.5f\r\n"
        "  mic_gate_thresh=%.4f  src=%s\r\n"
        "  fm_dev=%.0f Hz  ctcss=%.1f Hz\r\n",
        c.mic_agc_target, c.mic_agc_max_gain, c.mic_agc_attack, c.mic_agc_release,
        c.mic_gate_thresh, g_audio_src ? "MIC" : "PC",
        g_fm_deviation_hz, g_ctcss_freq
    );
}

static void cmd_help(void) {
    cdc_write_str(
        "Commands:\r\n"
        "  help\r\n"
        "  get\r\n"
        "  diag          - show SX1280 status\r\n"
        "  tx 0|1        - enable/disable TX (SSB modulation)\r\n"
        "  mode usb|cw|fm - set modulation mode\r\n"
        "  src pc|mic    - audio source (PC=USB audio, MIC=ADC0)\r\n"
        "  tune 0|1      - toggle TUNE carrier\r\n"
        "  cw            - start CW test transmission\r\n"
        "  stop          - stop CW transmission\r\n"
        "  freq <Hz>     - set frequency with sub-Hz precision (e.g. freq 2400100050.5)\r\n"
        "  ppm <value>   - set PPM correction (e.g. ppm -0.5)\r\n"    
        "  txpwr <-18..13> - set max TX power in dBm\r\n"
        "  enable <bp|eq|comp> <0|1|on|off>\r\n"
        "  audio_mode ssb|digital - USB audio resampling mode\r\n"
        "  set bp_lo <Hz>\r\n"
        "  set bp_hi <Hz>\r\n"
        "  set bp_stages <1-10>  (filter steepness: 12dB/oct per stage)\r\n"
        "  set eq_low_hz <Hz>\r\n"
        "  set eq_low_db <dB>\r\n"
        "  set eq_high_hz <Hz>\r\n"
        "  set eq_high_db <dB>\r\n"
        "  set comp_thr <dB>\r\n"
        "  set comp_ratio <R>\r\n"
        "  set comp_att <ms>\r\n"
        "  set comp_rel <ms>\r\n"
        "  set comp_makeup <dB>\r\n"
        "  set comp_knee <dB>\r\n"
        "  set comp_outlim <0..1>\r\n"
        "  set amp_gain <float>\r\n"
        "  set amp_min_a <float>\r\n"
        "  set mic_agc_target <0..1>   (MIC AGC target level)\r\n"
        "  set mic_agc_max_gain <1..200> (MIC AGC max gain)\r\n"
        "  set mic_agc_attack <coeff>  (MIC AGC attack speed)\r\n"
        "  set mic_agc_release <coeff> (MIC AGC release speed)\r\n"
        "  set mic_gate <0..0.5>       (noise gate threshold, 0=off)\r\n"
        "  set fm_dev <200..100000>    (FM deviation in Hz)\r\n"
        "  set ctcss <freq|0>          (CTCSS tone Hz, 0=off)\r\n"
        "\r\n"
        "Frequency is automatically split into PLL steps + fine DSP offset.\r\n"
    );
}

static void cfg_commit(const audio_cfg_t *c) {
    __compiler_memory_barrier();
    memcpy((void*)&g_cfg, c, sizeof(*c));
    g_cfg_dirty = 1;
    __compiler_memory_barrier();
}

// Periodic status push to CDC for GUI synchronization.
// Sends a compact line that the GUI can parse to update its widgets.
// Only sends when something changed, at most every 250 ms.
#if CFG_TUD_CDC
static void cdc_status_push_ex(bool force) {
    static uint32_t last_push_ms = 0;
    static uint8_t  last_mode = 0xFF;
    static uint8_t  last_tune = 0xFF;
    static uint8_t  last_tx   = 0xFF;
    static uint8_t  last_src  = 0xFF;
    static int8_t   last_pwr  = 127;
    static float    last_ppm  = 9999.0f;
    static double   last_freq = 0.0;
    static float    last_fm_dev = -1.0f;
    static float    last_ctcss  = -1.0f;

    if (!tud_cdc_connected()) return;

    uint8_t  cur_mode = g_tx_mode;
    uint8_t  cur_tune = g_tune_active;
    uint8_t  cur_tx   = g_tx_enabled;
    uint8_t  cur_src  = g_audio_src;
    int8_t   cur_pwr  = g_tx_power_max_dbm;
    float    cur_ppm  = g_ppm_correction;
    double   cur_freq = (double)g_target_freq_hz;
    float    cur_fm_dev = g_fm_deviation_hz;
    float    cur_ctcss  = g_ctcss_freq;

    if (!force) {
        // Check if anything changed
        bool changed = (cur_mode != last_mode) || (cur_tune != last_tune) ||
                       (cur_tx != last_tx) || (cur_src != last_src) ||
                       (cur_pwr != last_pwr) ||
                       (cur_ppm != last_ppm) || (cur_freq != last_freq) ||
                       (cur_fm_dev != last_fm_dev) || (cur_ctcss != last_ctcss);

        if (!changed) return;

        uint32_t now = to_ms_since_boot(get_absolute_time());
        if ((now - last_push_ms) < 100) return;
    }

    // Format freq with fmt_freq to avoid newlib-nano scientific notation
    char freq_str[24];
    fmt_freq(freq_str, sizeof(freq_str), cur_freq);

    // PPM: format as fixed-point (4 decimals) to avoid float formatting issues
    int ppm_neg = (cur_ppm < 0.0f);
    float ppm_abs = ppm_neg ? -cur_ppm : cur_ppm;
    uint32_t ppm_int = (uint32_t)ppm_abs;
    uint32_t ppm_frac = (uint32_t)((ppm_abs - (float)ppm_int) * 10000.0f + 0.5f);
    if (ppm_frac >= 10000) { ppm_int++; ppm_frac = 0; }

    char status_buf[160];
    // fm_dev: format as integer (no fractional Hz needed)
    uint32_t fm_dev_int = (uint32_t)(cur_fm_dev + 0.5f);
    // ctcss: format as X.Y (one decimal place)
    uint32_t ctcss_int = (uint32_t)cur_ctcss;
    uint32_t ctcss_frac = (uint32_t)((cur_ctcss - (float)ctcss_int) * 10.0f + 0.5f);
    if (ctcss_frac >= 10) { ctcss_int++; ctcss_frac = 0; }

    snprintf(status_buf, sizeof(status_buf),
             "!S mode=%u tune=%u tx=%u src=%u pwr=%d ppm=%s%lu.%04lu freq=%s fm_dev=%lu ctcss=%lu.%lu\r\n",
             cur_mode, cur_tune, cur_tx, cur_src, cur_pwr,
             ppm_neg ? "-" : "", (unsigned long)ppm_int, (unsigned long)ppm_frac,
             freq_str,
             (unsigned long)fm_dev_int,
             (unsigned long)ctcss_int, (unsigned long)ctcss_frac);
    cdc_write_str(status_buf);

    last_mode = cur_mode;
    last_tune = cur_tune;
    last_tx   = cur_tx;
    last_src  = cur_src;
    last_pwr  = cur_pwr;
    last_ppm  = cur_ppm;
    last_freq = cur_freq;
    last_fm_dev = cur_fm_dev;
    last_ctcss  = cur_ctcss;
    last_push_ms = to_ms_since_boot(get_absolute_time());
}

static inline void cdc_status_push(void) { cdc_status_push_ex(false); }
#endif

static void cdc_handle_line(char *line) {
    char *argv[6] = {0};
    int argc = 0;

    for (char *t = strtok(line, " \t\r\n"); t && argc < 6; t = strtok(NULL, " \t\r\n")) {
        argv[argc++] = t;
    }
    if (argc == 0) return;

    if (streqi(argv[0], "help")) { cmd_help(); return; }
    if (streqi(argv[0], "get"))  { cfg_print(); return; }
    if (streqi(argv[0], "status")) { cdc_status_push_ex(true); return; }
    if (streqi(argv[0], "diag")) { sx_print_diag(); return; }
    if (streqi(argv[0], "cw"))   { g_tune_active = 1; cdc_printf("OK tune=ON (carrier_poll handles SPI)\r\n"); return; }
    if (streqi(argv[0], "stop")) { g_tune_active = 0; cdc_printf("OK tune=OFF\r\n"); return; }

    // Mode: mode usb|cw|fm
    if (streqi(argv[0], "mode") && argc >= 2) {
        uint8_t new_mode = g_tx_mode;
        if (streqi(argv[1], "usb") || streqi(argv[1], "ssb")) {
            new_mode = 0;
            cdc_printf("OK mode=USB\r\n");
        } else if (streqi(argv[1], "cw")) {
            new_mode = 1;
            cdc_printf("OK mode=CW\r\n");
        } else if (streqi(argv[1], "fm")) {
            new_mode = 2;
            cdc_printf("OK mode=FM\r\n");
        } else {
            cdc_write_str("ERR: mode usb|cw|fm\r\n");
            return;
        }
        if (new_mode != g_tx_mode) {
            g_tx_mode = new_mode;
            g_mode_change_at_ms = to_ms_since_boot(get_absolute_time());
        }
        return;
    }

    // Audio source: src pc|mic|adc
    if (streqi(argv[0], "src") && argc >= 2) {
        if (streqi(argv[1], "pc") || streqi(argv[1], "usb")) {
            g_audio_src = 0;
            mic_timer_stop();
            cdc_printf("OK src=PC\r\n");
        } else if (streqi(argv[1], "mic") || streqi(argv[1], "adc")) {
            g_audio_src = 1;
            mic_timer_start();
            cdc_printf("OK src=MIC\r\n");
        } else {
            cdc_write_str("ERR: src pc|mic\r\n");
        }
        return;
    }

    // Tune carrier: tune 0|1
    if (streqi(argv[0], "tune") && argc >= 2) {
        uint8_t v;
        if (!parse_bool(argv[1], &v)) {
            cdc_write_str("ERR: tune 0|1|on|off\r\n");
            return;
        }
        g_tune_active = v;
        // carrier_poll() will handle SPI transitions
        cdc_printf("OK tune=%s\r\n", g_tune_active ? "ON" : "OFF");
        return;
    }

    // TX enable/disable: tx 0|1
    if (streqi(argv[0], "tx") && argc >= 2) {
        uint8_t v;
        if (!parse_bool(argv[1], &v)) { 
            cdc_write_str("ERR: tx 0|1|on|off\r\n"); 
            return; 
        }
        g_tx_enabled = v;
        cdc_printf("OK tx=%s\r\n", g_tx_enabled ? "ON" : "OFF");
        return;
    }

    // Frequency command: freq <Hz> (supports decimal for sub-Hz precision)
    if (streqi(argv[0], "freq") && argc >= 2) {
        char *e = NULL;
        double f = strtod(argv[1], &e);
        if (e == argv[1] || f < 2300000000.0 || f > 2450000000.0) {
            cdc_write_str("ERR: freq must be 2300000000-2450000000 Hz\r\n");
            return;
        }
        g_target_freq_hz = f;
        double corrected = get_corrected_freq_hz();
        float fine = get_fine_tune_hz();
        cdc_printf("OK freq=%.1f Hz (corrected=%.1f, steps=%lu, fine=%.1f Hz)\r\n", 
                   g_target_freq_hz, corrected,
                   (unsigned long)get_base_steps(), fine);
        if (g_tune_active) tune_apply_settings();
        return;
    }

    // PPM correction command: ppm <value>
    if (streqi(argv[0], "ppm") && argc >= 2) {
        float ppm;
        if (!parse_f(argv[1], &ppm)) { 
            cdc_write_str("ERR: bad PPM value\r\n"); 
            return; 
        }
        if (ppm < -100.0f || ppm > 100.0f) {
            cdc_write_str("ERR: ppm must be -100 to +100\r\n");
            return;
        }
        g_ppm_correction = ppm;
        double corrected = get_corrected_freq_hz();
        float fine = get_fine_tune_hz();
        cdc_printf("OK ppm=%.3f (corrected=%.1f Hz, steps=%lu, fine=%.1f Hz)\r\n", 
                   g_ppm_correction, corrected,
                   (unsigned long)get_base_steps(), fine);
        if (g_tune_active) tune_apply_settings();
        return;
    }

    // TX power command: txpwr <-18..13>
    if (streqi(argv[0], "txpwr") && argc >= 2) {
        float pwr;
        if (!parse_f(argv[1], &pwr)) { 
            cdc_write_str("ERR: bad txpwr value\r\n"); 
            return; 
        }
        if (pwr < (float)PWR_MIN_DBM) pwr = (float)PWR_MIN_DBM;
        if (pwr > (float)PWR_MAX_DBM) pwr = (float)PWR_MAX_DBM;
        g_tx_power_max_dbm = (int8_t)pwr;
        cdc_printf("OK txpwr=%d dBm\r\n", g_tx_power_max_dbm);
        if (g_tune_active) tune_apply_settings();
        return;
    }

    //audio_mode command: SSB|DIGITAL
    // audio_mode ssb|digital
    if (streqi(argv[0], "audio_mode")) {
        
        if (argc == 1) {
            cdc_printf("audio_mode=%s\r\n", g_audio_modulation  == AUDIO_MODULATION_DIGITAL ? "DIGITAL" : "SSB");
            return;
        }
        if (argc !=2) {
            cdc_write_str("ERR: audio_mode |ssb|digital\r\n"); 
            return; 
        }
        if (streqi(argv[1], "ssb")) {
            g_audio_modulation = AUDIO_MODULATION_SSB;
        } else if(streqi(argv[1], "digital")) {
            g_audio_modulation = AUDIO_MODULATION_DIGITAL;
        } else {
            cdc_write_str("ERR: audio_mode ssb|digital\r\n"); 
            return;           
        }
        
        cdc_printf("OK audio_mode=%s\r\n", g_audio_modulation  == AUDIO_MODULATION_DIGITAL ? "DIGITAL" : "SSB");
        return;
    }


    audio_cfg_t c;
    __compiler_memory_barrier();
    memcpy(&c, (const void*)&g_cfg, sizeof(c));
    __compiler_memory_barrier();

    if (streqi(argv[0], "enable") && argc >= 3) {
        uint8_t v;
        if (!parse_bool(argv[2], &v)) { cdc_write_str("ERR: bad bool\r\n"); return; }

        if (streqi(argv[1], "bp")) c.enable_bandpass = v;
        else if (streqi(argv[1], "eq")) c.enable_eq = v;
        else if (streqi(argv[1], "comp")) c.enable_comp = v;
        else { cdc_write_str("ERR: enable bp|eq|comp\r\n"); return; }

        cfg_commit(&c);
        cdc_write_str("OK\r\n");
        return;
    }

    if (streqi(argv[0], "set") && argc >= 3) {
        float f;
        if (!parse_f(argv[2], &f)) { cdc_write_str("ERR: bad number\r\n"); return; }

        // FM deviation and CTCSS are volatile globals, not in audio_cfg_t
        if (streqi(argv[1], "fm_dev")) {
            if (f < 200.0f) f = 200.0f;
            if (f > 100000.0f) f = 100000.0f;
            g_fm_deviation_hz = f;
            cdc_printf("OK fm_dev=%.0f Hz\r\n", g_fm_deviation_hz);
            return;
        }
        if (streqi(argv[1], "ctcss")) {
            if (f < 0.0f) f = 0.0f;
            if (f > 300.0f) f = 300.0f;
            g_ctcss_freq = f;
            cdc_printf("OK ctcss=%.1f Hz\r\n", g_ctcss_freq);
            return;
        }
        if (streqi(argv[1], "roger")) {
            g_roger_beep = (f != 0.0f) ? 1 : 0;
            cdc_printf("OK roger=%s\r\n", g_roger_beep ? "on" : "off");
            return;
        }

        if      (streqi(argv[1], "bp_lo"))       c.bp_lo_hz = f;
        else if (streqi(argv[1], "bp_hi"))       c.bp_hi_hz = f;
        else if (streqi(argv[1], "bp_stages"))   c.bp_stages = (uint8_t)f;
        else if (streqi(argv[1], "eq_low_hz"))   c.eq_low_hz = f;
        else if (streqi(argv[1], "eq_low_db"))   c.eq_low_db = f;
        else if (streqi(argv[1], "eq_high_hz"))  c.eq_high_hz = f;
        else if (streqi(argv[1], "eq_high_db"))  c.eq_high_db = f;
        else if (streqi(argv[1], "comp_thr"))    c.comp_thr_db = f;
        else if (streqi(argv[1], "comp_ratio"))  c.comp_ratio = f;
        else if (streqi(argv[1], "comp_att"))    c.comp_attack_ms = f;
        else if (streqi(argv[1], "comp_rel"))    c.comp_release_ms = f;
        else if (streqi(argv[1], "comp_makeup")) c.comp_makeup_db = f;
        else if (streqi(argv[1], "comp_knee"))   c.comp_knee_db = f;
        else if (streqi(argv[1], "comp_outlim")) c.comp_out_limit = f;
        else if (streqi(argv[1], "amp_gain"))    c.amp_gain = f;
        else if (streqi(argv[1], "amp_min_a"))   c.amp_min_a = f;
        else if (streqi(argv[1], "mic_agc_target"))   c.mic_agc_target = f;
        else if (streqi(argv[1], "mic_agc_max_gain")) c.mic_agc_max_gain = f;
        else if (streqi(argv[1], "mic_agc_attack"))   c.mic_agc_attack = f;
        else if (streqi(argv[1], "mic_agc_release"))  c.mic_agc_release = f;
        else if (streqi(argv[1], "mic_gate"))         c.mic_gate_thresh = f;
        else { cdc_write_str("ERR: unknown key\r\n"); return; }

        cfg_commit(&c);
        cdc_write_str("OK\r\n");
        return;
    }

    cdc_write_str("ERR: unknown command (type 'help')\r\n");
}

static void cdc_task(void) {
#if CFG_TUD_CDC
    static char line[128];
    static uint32_t pos = 0;

    if (!tud_cdc_connected()) return;

    while (tud_cdc_available()) {
        char ch = (char)tud_cdc_read_char();

        if (ch == '\r' || ch == '\n') {
            if (pos > 0) {
                line[pos] = 0;
                cdc_handle_line(line);
                pos = 0;
            }
        } else {
            if (pos < sizeof(line) - 1) {
                line[pos++] = ch;
            }
        }
    }
#endif
}

// ==========================================================
// OLED display — simple text, DMA transfer
// ==========================================================
#define QO100_DOWNLINK_OFFSET_HZ  8089500000.0

// Helper: right-justify a 1x string ending at x_right edge
static void draw_string_right(int x_right, int page, const char *str) {
    int len = 0;
    const char *p = str;
    while (*p++) len++;
    int x = x_right - len * 6;
    ssd1306_draw_string(x, page, str);
}

// Draw a 2x-height arrow-up glyph into framebuf at (x, page..page+1)
static void draw_arrow_up_2x(int x, int page) {
    static const uint8_t src[] = {0x04, 0x02, 0x7F, 0x02, 0x04};
    uint8_t *fb = ssd1306_get_framebuf();
    for (int c = 0; c < 5; c++) {
        uint16_t expanded = 0;
        for (int b = 0; b < 7; b++)
            if (src[c] & (1 << b)) expanded |= (3u << (b * 2));
        uint8_t lo = (uint8_t)(expanded & 0xFF);
        uint8_t hi = (uint8_t)((expanded >> 8) & 0xFF);
        for (int dx = 0; dx < 2; dx++) {
            int px = x + c * 2 + dx;
            if (px >= 0 && px < SSD1306_WIDTH) {
                fb[page * SSD1306_WIDTH + px] = lo;
                fb[(page + 1) * SSD1306_WIDTH + px] = hi;
            }
        }
    }
}

// Draw a 2x-height arrow-down glyph into framebuf at (x, page..page+1)
static void draw_arrow_down_2x(int x, int page) {
    static const uint8_t src[] = {0x10, 0x20, 0x7F, 0x20, 0x10};
    uint8_t *fb = ssd1306_get_framebuf();
    for (int c = 0; c < 5; c++) {
        uint16_t expanded = 0;
        for (int b = 0; b < 7; b++)
            if (src[c] & (1 << b)) expanded |= (3u << (b * 2));
        uint8_t lo = (uint8_t)(expanded & 0xFF);
        uint8_t hi = (uint8_t)((expanded >> 8) & 0xFF);
        for (int dx = 0; dx < 2; dx++) {
            int px = x + c * 2 + dx;
            if (px >= 0 && px < SSD1306_WIDTH) {
                fb[page * SSD1306_WIDTH + px] = lo;
                fb[(page + 1) * SSD1306_WIDTH + px] = hi;
            }
        }
    }
}

// Draw radio-on icon at pixel (x, y): dot + 2 arcs, 7px tall, ~9px wide
static void draw_radio_on_xy(int x, int y) {
    // Dot 2x2 at center height (y+2, y+3)
    ssd1306_set_pixel(x,   y+2, true); ssd1306_set_pixel(x,   y+3, true);
    ssd1306_set_pixel(x+1, y+2, true); ssd1306_set_pixel(x+1, y+3, true);
    // Arc 1
    ssd1306_set_pixel(x+3, y+1, true);
    ssd1306_set_pixel(x+3, y+2, true); ssd1306_set_pixel(x+3, y+3, true);
    ssd1306_set_pixel(x+3, y+4, true);
    // Arc 2
    ssd1306_set_pixel(x+5, y+0, true);
    ssd1306_set_pixel(x+5, y+1, true);
    ssd1306_set_pixel(x+5, y+2, true); ssd1306_set_pixel(x+5, y+3, true);
    ssd1306_set_pixel(x+5, y+4, true);
    ssd1306_set_pixel(x+5, y+5, true);
}

// Draw radio-off icon at pixel (x, y): just a dot, 7px tall, 2px wide
static void draw_radio_off_xy(int x, int y) {
    ssd1306_set_pixel(x,   y+2, true); ssd1306_set_pixel(x,   y+3, true);
    ssd1306_set_pixel(x+1, y+2, true); ssd1306_set_pixel(x+1, y+3, true);
}

// Forward declaration — defined below, after OLED drawing helpers
static void oled_prepare_frame(void);

// ==========================================================
// Central OLED refresh — single rate-limited entry point.
// Call this from any poll loop; it internally ensures:
//  - DMA is finished before touching framebuffer
//  - At most ~5 fps (200 ms between redraws)
// ==========================================================
static void oled_poll(void) {
    static absolute_time_t oled_next = {0};
    if (ssd1306_dma_busy()) return;
    if (absolute_time_diff_us(get_absolute_time(), oled_next) > 0) return;
    oled_prepare_frame();
    ssd1306_display_dma(OLED_I2C);
    oled_next = make_timeout_time_ms(200);
}

// ==========================================================
// Small UI helpers
// ==========================================================
static const char *mode_label(uint8_t m) {
    switch (m) {
        case TXM_CW:     return "CW";
        case TXM_FM:     return "FM";
        default:         return "USB";
    }
}

// Return true when `item` should be visible for the current configuration.
// (some menu items depend on the selected TX mode)
static bool menu_item_visible(menu_item_t item) {
    switch (item) {
        case MENU_FM_DEV:
        case MENU_CTCSS:
        case MENU_ROGER_BEEP:
            return (g_tx_mode == TXM_FM);
        default:
            return true;
    }
}

// Build the text displayed next to a menu item.
static void menu_value_str(menu_item_t item, char *out, size_t n) {
    switch (item) {
        case MENU_MODE:
            snprintf(out, n, "%s", mode_label(g_tx_mode)); break;
        case MENU_TX:
            snprintf(out, n, "%s", g_tx_enabled ? "ON" : "OFF"); break;
        case MENU_TUNE:
            snprintf(out, n, "%s", g_tune_active ? "ON" : "OFF"); break;
        case MENU_SRC:
            snprintf(out, n, "%s", g_audio_src ? "MIC" : "PC"); break;
        case MENU_POWER:
            snprintf(out, n, "%+ddBm", g_tx_power_max_dbm); break;
        case MENU_PPM:
            snprintf(out, n, "%+.2f", (double)g_ppm_correction); break;
        case MENU_FM_DEV:
            snprintf(out, n, "%uHz", (unsigned)g_fm_deviation_hz); break;
        case MENU_CTCSS:
            if (g_ctcss_freq <= 0.0f) snprintf(out, n, "off");
            else snprintf(out, n, "%.1f", (double)g_ctcss_freq);
            break;
        case MENU_ROGER_BEEP:
            snprintf(out, n, "%s", g_roger_beep ? "ON" : "OFF"); break;
        case MENU_SAVE: out[0] = 0; break;
        case MENU_EXIT: out[0] = 0; break;
        default: out[0] = 0; break;
    }
}

static const char *menu_label(menu_item_t item) {
    switch (item) {
        case MENU_MODE:         return "Mode";
        case MENU_TX:           return "TX";
        case MENU_TUNE:         return "Tune";
        case MENU_SRC:          return "Src";
        case MENU_POWER:        return "Pwr";
        case MENU_PPM:          return "Ppm";
        case MENU_FM_DEV:       return "Dev";
        case MENU_CTCSS:        return "CTCSS";
        case MENU_ROGER_BEEP:   return "RogerBp";
        case MENU_SAVE:         return "[Save]";
        case MENU_EXIT:         return "[Exit]";
        default:                return "?";
    }
}

// Step cursor/scroll by +1 or -1, skipping hidden items.  Never infinite.
static menu_item_t menu_step(menu_item_t cur, int dir) {
    int c = (int)cur;
    for (int guard = 0; guard < (int)MENU_ITEM_COUNT; guard++) {
        c += dir;
        if (c < 0) c = MENU_ITEM_COUNT - 1;
        if (c >= (int)MENU_ITEM_COUNT) c = 0;
        if (menu_item_visible((menu_item_t)c)) return (menu_item_t)c;
    }
    return cur;
}

// ==========================================================
// Prepare framebuffer content (fast, no I2C)
// ==========================================================
static void oled_prepare_frame(void) {
    ssd1306_clear();

    if (g_ui_state == UI_STATE_TUNE) {
        // =====================================================
        // TUNE screen: big frequency, underlined digit, status
        // =====================================================
        double freq = g_target_freq_hz;
        double downlink = freq + QO100_DOWNLINK_OFFSET_HZ;

        // --- Row 0 (pages 0-1): uplink freq, 2x font ---
        draw_arrow_up_2x(0, 0);
        char buf[20];
        uint32_t khz_total = (uint32_t)(freq / 1000.0);
        uint32_t frac = (uint32_t)((freq - (double)khz_total * 1000.0) / 100.0 + 0.5);
        if (frac >= 10) { frac -= 10; khz_total++; }
        snprintf(buf, sizeof(buf), "%lu.%lu", (unsigned long)khz_total, (unsigned long)frac);
        int len = 0; { const char *p = buf; while (*p++) len++; }
        int x_start = 128 - len * 12;
        ssd1306_draw_string_2x(x_start, 0, buf);

        // Underline the digit currently being tuned.  Digit indexes (LSB-first):
        //   idx 0 = 0.1 kHz (tenths, last char after dot)
        //   idx 1 = 1   kHz (ones)
        //   idx 2..5 = higher kHz decades (skipping the dot char)
        uint8_t di = g_tune_digit_idx;
        if (di >= TUNE_STEP_COUNT) di = 0;
        int char_k;
        if (di == 0) {
            char_k = len - 1;                 // tenths-kHz char
        } else {
            char_k = len - 2 - (int)di;       // above the dot
        }
        if (char_k >= 0 && char_k < len) {
            int ux0 = x_start + char_k * 12;
            int ux1 = ux0 + 11;
            // Draw underline two pixels thick for visibility
            ssd1306_hline(ux0, ux1, 16);
            ssd1306_hline(ux0, ux1, 17);
        }

        // --- Row 1 (pages 3-4): downlink freq when on QO-100 NB ---
        if (freq >= 2400000000.0 && freq <= 2400500000.0) {
            draw_arrow_down_2x(0, 3);
            uint32_t dkhz = (uint32_t)(downlink / 1000.0);
            uint32_t dfrac = (uint32_t)((downlink - (double)dkhz * 1000.0) / 100.0 + 0.5);
            if (dfrac >= 10) { dfrac -= 10; dkhz++; }
            char dbuf[20];
            snprintf(dbuf, sizeof(dbuf), "%lu.%lu", (unsigned long)dkhz, (unsigned long)dfrac);
            int dlen = 0; { const char *p = dbuf; while (*p++) dlen++; }
            int dx = 128 - dlen * 12;
            ssd1306_draw_string_2x(dx, 3, dbuf);
        }

        // --- Separator above status strip ---
        ssd1306_hline(0, 127, 54);

        // --- Page 6 (y=48): per-mode parameter summary ---
        {
            char pbuf[32];
            if (g_tx_mode == TXM_FM) {
                char ct[12];
                if (g_ctcss_freq > 0.0f) snprintf(ct, sizeof(ct), "%.1f", (double)g_ctcss_freq);
                else                     snprintf(ct, sizeof(ct), "off");
                snprintf(pbuf, sizeof(pbuf), "D%u CT%s RB%s",
                         (unsigned)g_fm_deviation_hz, ct,
                         g_roger_beep ? "+" : "-");
            } else {
                snprintf(pbuf, sizeof(pbuf), "PPM%+.2f", (double)g_ppm_correction);
            }
            ssd1306_draw_string(0, 6, pbuf);
        }

        // --- Status strip at page 7 (y=56): MODE  TX  PWR  SRC ---
        char sbuf[64];
        const char *tx_str;
        if (g_tune_active)                          tx_str = "TUNE";
        else if (g_ptt_key)                         tx_str = "KEY";
        else if (g_tx_enabled)                      tx_str = "TX";
        else                                        tx_str = "rx";
        snprintf(sbuf, sizeof(sbuf), "%s %s %+ddBm %s",
                 mode_label(g_tx_mode), tx_str,
                 g_tx_power_max_dbm,
                 g_audio_src ? "MIC" : "PC");
        ssd1306_draw_string(0, 7, sbuf);

        return;
    }

    // =========================================================
    // MENU screen: scrollable vertical list
    // =========================================================
    ssd1306_draw_string(0, 0, "MENU");
    {
        char hint[20];
        snprintf(hint, sizeof(hint), g_menu_editing ? "edit" : "hold=exit");
        draw_string_right(128, 0, hint);
    }
    ssd1306_hline(0, 127, 9);

    // Build a flattened list of visible items (max MENU_ITEM_COUNT).
    menu_item_t visible[MENU_ITEM_COUNT];
    int vis_n = 0;
    for (int i = 0; i < (int)MENU_ITEM_COUNT; i++) {
        if (menu_item_visible((menu_item_t)i)) visible[vis_n++] = (menu_item_t)i;
    }
    // Find cursor's visible index
    int cur_vi = 0;
    for (int i = 0; i < vis_n; i++)
        if (visible[i] == g_menu_cursor) { cur_vi = i; break; }

    // Visible rows: 6 rows starting at y=12, 9 px stride → last y = 12 + 5*9 = 57
    const int ROWS = 6;
    int top = (int)g_menu_scroll_top;
    if (top > cur_vi) top = cur_vi;
    if (top + ROWS <= cur_vi) top = cur_vi - ROWS + 1;
    if (top < 0) top = 0;
    if (top > vis_n - ROWS) top = (vis_n > ROWS) ? vis_n - ROWS : 0;
    g_menu_scroll_top = (uint32_t)top;

    for (int r = 0; r < ROWS; r++) {
        int vi = top + r;
        if (vi >= vis_n) break;
        menu_item_t it = visible[vi];
        int y = 12 + r * 9;

        char line[28];
        char val[16];
        menu_value_str(it, val, sizeof(val));
        if (val[0])
            snprintf(line, sizeof(line), "%-7s %s", menu_label(it), val);
        else
            snprintf(line, sizeof(line), "%s", menu_label(it));

        bool is_cursor = (it == g_menu_cursor);
        if (is_cursor && g_menu_editing) {
            // full-line inverted while editing
            ssd1306_draw_string_bold_y_inv(0, y, line, 128);
        } else if (is_cursor) {
            // frame around the row
            ssd1306_draw_string_bold_y(4, y, line);
            ssd1306_hline(0, 127, y - 1);
            ssd1306_hline(0, 127, y + 8);
            ssd1306_vline(0,   y - 1, y + 8);
            ssd1306_vline(127, y - 1, y + 8);
        } else {
            ssd1306_draw_string_bold_y(4, y, line);
        }
    }
}

// ==========================================================
// Encoder + Button polling (Core0, called from idle loop)
// ==========================================================

// Encoder state machine (Gray code quadrature)
static uint8_t enc_last_ab = 0;
static int8_t  enc_accum = 0;

// Button debounce / long-press state
static uint8_t  ok_raw_state    = 0;    // last debounced raw state (1 = pressed)
static uint32_t ok_debounce_ms  = 0;
static uint32_t ok_press_started_ms = 0;
static uint8_t  ok_long_fired   = 0;    // 1 once long-press action triggered this press
static uint32_t ptt_debounce_ms = 0;
static uint8_t  ptt_last_state  = 0;

#define DEBOUNCE_MS        5
#define LONG_PRESS_MS      500u

// Well-known CTCSS tones (EIA standard set, subset).  0 = off.
static const float CTCSS_TONES[] = {
    0.0f,
    67.0f,  71.9f,  74.4f,  77.0f,  79.7f,  82.5f,  85.4f,  88.5f,
    91.5f,  94.8f,  97.4f, 100.0f, 103.5f, 107.2f, 110.9f, 114.8f,
    118.8f, 123.0f, 127.3f, 131.8f, 136.5f, 141.3f, 146.2f, 151.4f,
    156.7f, 162.2f, 167.9f, 173.8f, 179.9f, 186.2f, 192.8f, 203.5f,
    210.7f, 218.1f, 225.7f, 233.6f, 241.8f, 250.3f
};
#define CTCSS_COUNT ((int)(sizeof(CTCSS_TONES)/sizeof(CTCSS_TONES[0])))

static int ctcss_find_index(float f) {
    int best = 0; float bestd = 1e9f;
    for (int i = 0; i < CTCSS_COUNT; i++) {
        float d = CTCSS_TONES[i] - f;
        if (d < 0) d = -d;
        if (d < bestd) { bestd = d; best = i; }
    }
    return best;
}

// Handle an encoder click (+1 or -1) while in an editing context for a menu item.
static void menu_edit_apply(menu_item_t item, int step) {
    switch (item) {
        case MENU_MODE: {
            int m = (int)g_tx_mode + step;
            if (m < 0) m = TXM_FM;
            if (m > TXM_FM) m = 0;
            if ((uint8_t)m != g_tx_mode) {
                g_tx_mode = (uint8_t)m;
                // Arm the guard so any currently-playing carrier drops
                // cleanly and no new TX activates until things settle.
                g_mode_change_at_ms = to_ms_since_boot(get_absolute_time());
            }
            break;
        }
        case MENU_TX:
            g_tx_enabled = g_tx_enabled ? 0 : 1;
            break;
        case MENU_TUNE:
            g_tune_active = g_tune_active ? 0 : 1;
            break;
        case MENU_SRC:
            g_audio_src = g_audio_src ? 0 : 1;
            if (g_audio_src) mic_timer_start(); else mic_timer_stop();
            break;
        case MENU_POWER: {
            int p = g_tx_power_max_dbm + step;
            if (p < PWR_MIN_DBM) p = PWR_MIN_DBM;
            if (p > PWR_MAX_DBM) p = PWR_MAX_DBM;
            g_tx_power_max_dbm = (int8_t)p;
            if (g_tune_active) tune_apply_settings();
            break;
        }
        case MENU_PPM: {
            float ppm = g_ppm_correction + (float)step * 0.01f;
            if (ppm < -50.0f) ppm = -50.0f;
            if (ppm >  50.0f) ppm =  50.0f;
            g_ppm_correction = ppm;
            if (g_tune_active) tune_apply_settings();
            break;
        }
        case MENU_FM_DEV: {
            float d = g_fm_deviation_hz + (float)step * 100.0f;
            if (d < 200.0f) d = 200.0f;
            if (d > 100000.0f) d = 100000.0f;
            g_fm_deviation_hz = d;
            break;
        }
        case MENU_CTCSS: {
            int i = ctcss_find_index(g_ctcss_freq) + step;
            if (i < 0) i = CTCSS_COUNT - 1;
            if (i >= CTCSS_COUNT) i = 0;
            g_ctcss_freq = CTCSS_TONES[i];
            break;
        }
        case MENU_ROGER_BEEP:
            g_roger_beep = g_roger_beep ? 0 : 1;
            break;
        default: return;
    }
    persist_mark_dirty();
}

static void encoder_poll(void) {
    // Read encoder pins (active LOW with pull-up)
    uint8_t a = gpio_get(PIN_ENC_A) ? 0 : 1;
    uint8_t b = gpio_get(PIN_ENC_B) ? 0 : 1;
    uint8_t ab = (a << 1) | b;

    if (ab == enc_last_ab) return;

    static const int8_t enc_table[16] = {
         0, -1, +1,  0,
        +1,  0,  0, -1,
        -1,  0,  0, +1,
         0, +1, -1,  0
    };
    int8_t dir = enc_table[(enc_last_ab << 2) | ab];
    enc_last_ab = ab;
    if (dir == 0) return;

    enc_accum += dir;
    int step = 0;
    if (enc_accum >= 4)       { step = +1; enc_accum = 0; }
    else if (enc_accum <= -4) { step = -1; enc_accum = 0; }
    else                        return;

    if (g_ui_state == UI_STATE_TUNE) {
        uint8_t di = g_tune_digit_idx;
        if (di >= TUNE_STEP_COUNT) di = 0;
        double inc = g_tune_steps_hz[di] * (double)step;
        double f = g_target_freq_hz + inc;
        if (f < 2300000000.0) f = 2300000000.0;
        if (f > 2450000000.0) f = 2450000000.0;
        g_target_freq_hz = f;
        if (g_tune_active) tune_apply_settings();
        persist_mark_dirty();
        return;
    }

    // MENU state
    if (g_menu_editing) {
        menu_edit_apply(g_menu_cursor, step);
    } else {
        g_menu_cursor = menu_step(g_menu_cursor, step);
    }
}

static void button_poll(void) {
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    // --- OK button (encoder push), active LOW ---
    uint8_t ok_raw = gpio_get(PIN_ENC_OK) ? 0 : 1;

    if (ok_raw != ok_raw_state && (now_ms - ok_debounce_ms) >= DEBOUNCE_MS) {
        ok_debounce_ms = now_ms;

        if (ok_raw && !ok_raw_state) {
            // Falling → pressed
            ok_press_started_ms = now_ms;
            ok_long_fired = 0;
        } else if (!ok_raw && ok_raw_state) {
            // Rising → released
            if (!ok_long_fired) {
                // Short click behaviour
                if (g_ui_state == UI_STATE_TUNE) {
                    // Advance the underlined digit
                    g_tune_digit_idx = (g_tune_digit_idx + 1) % TUNE_STEP_COUNT;
                    persist_mark_dirty();
                } else {
                    // MENU: action on some items, otherwise toggle edit
                    switch (g_menu_cursor) {
                        case MENU_SAVE:
                            persist_save_now();
                            break;
                        case MENU_EXIT:
                            g_menu_editing = 0;
                            g_ui_state = UI_STATE_TUNE;
                            break;
                        case MENU_TX:
                        case MENU_TUNE:
                            // Toggle instantly on click
                            menu_edit_apply(g_menu_cursor, +1);
                            break;
                        default:
                            g_menu_editing = g_menu_editing ? 0 : 1;
                            break;
                    }
                }
            }
        }
        ok_raw_state = ok_raw;
    }

    // Long-press detection while still held (edge triggered)
    if (ok_raw_state && !ok_long_fired &&
        (now_ms - ok_press_started_ms) >= LONG_PRESS_MS) {
        ok_long_fired = 1;
        if (g_ui_state == UI_STATE_TUNE) {
            g_ui_state      = UI_STATE_MENU;
            g_menu_cursor   = MENU_MODE;
            g_menu_editing  = 0;
            g_menu_scroll_top = 0;
        } else {
            // In MENU: long press = exit (auto-save via autosave timer)
            g_menu_editing = 0;
            g_ui_state = UI_STATE_TUNE;
        }
    }

    // --- PTT/KEY button — proper debouncing ---
    // We want *stability* based debouncing: the raw input must hold its
    // new value for N ms before we commit.  Use asymmetric thresholds —
    // press latches quickly so CW/PTT feel responsive, but release waits
    // longer so contact bounce at release does not re-trigger the roger
    // beep or clip CW tails.
    #define PTT_DEBOUNCE_PRESS_MS    6u
    #define PTT_DEBOUNCE_RELEASE_MS  25u
    uint8_t ptt_raw = gpio_get(PIN_PTT_KEY) ? 0 : 1;
    if (ptt_raw != ptt_last_state) {
        // Candidate changed — (re)start stability timer
        ptt_debounce_ms = now_ms;
        ptt_last_state  = ptt_raw;
    } else if (ptt_raw != g_ptt_key) {
        // Candidate == raw, differs from committed state: check dwell
        uint32_t need = ptt_raw ? PTT_DEBOUNCE_PRESS_MS
                                : PTT_DEBOUNCE_RELEASE_MS;
        if ((now_ms - ptt_debounce_ms) >= need) {
            g_ptt_key = ptt_raw;
        }
    }
}

// ==========================================================
// Hilbert
// ==========================================================
static float hilb_h[HILBERT_TAPS];
static float hilb_buf[HILBERT_TAPS];
static uint32_t hilb_idx = 0;

static void hilbert_reset(void) {
    for (int i = 0; i < HILBERT_TAPS; i++) hilb_buf[i] = 0.0f;
    hilb_idx = 0;
}

static void hilbert_init(void) {
    const int M = (HILBERT_TAPS - 1) / 2;

    for (int n = 0; n < HILBERT_TAPS; n++) {
        int k = n - M;

        float h = 0.0f;
        if (k != 0 && (k & 1)) h = 2.0f / ((float)M_PI * (float)k);

        float w = 0.54f - 0.46f *
                  cosf(2.0f * (float)M_PI * (float)n /
                       (float)(HILBERT_TAPS - 1));

        hilb_h[n] = h * w;
        hilb_buf[n] = 0.0f;
    }
}

static inline float hilbert_process(float x, float *i_delayed) {
    const int M = (HILBERT_TAPS - 1) / 2;

    hilb_buf[hilb_idx] = x;

    float y = 0.0f;
    uint32_t idx = hilb_idx;
    for (int n = 0; n < HILBERT_TAPS; n++) {
        y += hilb_h[n] * hilb_buf[idx];
        if (idx == 0) idx = HILBERT_TAPS - 1;
        else idx--;
    }

    uint32_t id = (hilb_idx + HILBERT_TAPS - (uint32_t)M) % HILBERT_TAPS;
    *i_delayed = hilb_buf[id];

    hilb_idx++;
    if (hilb_idx >= HILBERT_TAPS) hilb_idx = 0;

    return y;
}

static inline float duty_from_A(float A) {
    if (A <= 0.0f) return 0.0f;
    float r = A / GATE_A_REF;
    if (r >= 1.0f) return 1.0f;
#if GATE_SHAPE == 2
    return r * r;
#else
    return r;
#endif
}

// ==========================================================
// CORE1: timed radio apply loop
// ==========================================================
static void core1_radio_apply_loop(void) {
    const uint32_t sample_period_us = 1000000u / WAV_SAMPLE_RATE;
    const uint32_t substeps = (DITHER_SUBSTEPS <= 1) ? 1u : (uint32_t)DITHER_SUBSTEPS;
    const uint32_t sub_period_us = (substeps == 1) ? sample_period_us : (sample_period_us / substeps);

    // Register this core with the pico_flash safety helper so Core0 can
    // safely call flash_safe_execute() during config saves.  Internally
    // this calls multicore_lockout_victim_init().  Without it, Core0 would
    // time out trying to park Core1 and the save would fail (and previously
    // the manual lockout sequence would hang forever).
    flash_safe_execute_core_init();

#if UNDERRUN_LED_ENABLE
    const uint led_pin = PICO_DEFAULT_LED_PIN;
    if (led_pin != (uint)-1) {
        gpio_init(led_pin);
        gpio_set_dir(led_pin, GPIO_OUT);
        gpio_put(led_pin, 0);
    }
    uint32_t last_und = 0;
    absolute_time_t led_off_time = get_absolute_time();
#endif

    int32_t last_steps = 0x7FFFFFFF;
    int32_t last_p_dbm = 9999;
    bool last_tx_on = false;  // Start with TX off
    bool tx_en_activated = false;  // Track if we've enabled the PA

    g_dbg_core1_alive = 1;
    while (true) {
        g_dbg_core1_iters++;
        g_dbg_core1_bc = 1;
        // === CW test / CW mode / TUNE: Core0 owns SPI, Core1 idles ===
        if (g_cw_test_mode) {
            g_dbg_core1_bc = 11;  // drain branch
            last_tx_on = false;
            last_steps = 0x7FFFFFFF;
            last_p_dbm = 9999;
            // Drain all blocks so Core0 doesn't stall
            for (;;) {
                uint32_t b = g_cons_block;
                if (!g_block_ready[b]) break;
                __compiler_memory_barrier();
                g_block_ready[b] = 0;
                __compiler_memory_barrier();
                g_cons_block = (b + 1u) % NUM_BLOCKS;
            }
            sleep_ms(10);
            continue;
        }

        // === SSB MODE: normal audio processing ===
        // Pre-buf gating removed — Core1 simply waits for block_ready[b]
        // below (via bc=13 underrun path).  The old g_core1_start flag was
        // an unnecessary extra handshake that could deadlock.
        (void)g_core1_start;

        uint32_t b = g_cons_block;

        if (!g_block_ready[b]) {
            g_dbg_core1_bc = 13;   // block not ready
            g_underruns++;

#if UNDERRUN_LED_ENABLE
            uint32_t und = g_underruns;
            if (und != last_und) {
                last_und = und;
                if (PICO_DEFAULT_LED_PIN != (uint)-1) {
                    gpio_put(PICO_DEFAULT_LED_PIN, 1);
                    led_off_time = make_timeout_time_ms(UNDERRUN_LED_PULSE_MS);
                }
            }
#endif

            uint64_t t0 = time_us_64();
            while (time_us_64() - t0 < sample_period_us) tight_loop_contents();
            continue;
        }

        // Enable TX_EN on first valid block (USB is now stable)
        if (!tx_en_activated) {
            gpio_put(PIN_TX_EN, 1);
            tx_en_activated = true;
            sleep_ms(1);  // Short delay for PA to stabilize
        }

        g_dbg_core1_bc = 2;
        sample_cmd_t *blk = g_blocks[b];
        uint64_t next_us = time_us_64();

        for (uint32_t i = 0; i < BLOCK_SAMPLES; i++) {
            next_us += sample_period_us;
            g_dbg_core1_bc = 3;

            for (uint32_t k = 0; k < substeps; k++) {
                sample_cmd_t c = blk[i];

                if ((bool)c.tx_on != last_tx_on) {
                    g_dbg_core1_bc = 4;
                    if (c.tx_on) { sx_start_tx_continuous_wave(); g_dbg_core1_txcw++; }
#if USE_TCXO_MODULE
                    else         sx_set_standby_xosc();
#else
                    else         sx_set_standby_rc();
#endif
                    last_tx_on = (bool)c.tx_on;
                    g_dbg_core1_bc = 40;  // after tx toggle
                }

                if (c.freq_steps != last_steps) {
                    g_dbg_core1_bc = 5;
                    sx_set_rf_frequency_steps((uint32_t)c.freq_steps);
                    last_steps = c.freq_steps;
                    g_dbg_core1_bc = 50;
                }

                if ((int32_t)c.p_dbm != last_p_dbm) {
                    g_dbg_core1_bc = 6;
                    sx_set_tx_params_dbm((int32_t)c.p_dbm);
                    last_p_dbm = (int32_t)c.p_dbm;
                    g_dbg_core1_bc = 60;
                }

                if (sub_period_us > 0) {
                    g_dbg_core1_bc = 7;
                    uint64_t target = next_us - (uint64_t)(sample_period_us - (k + 1u) * sub_period_us);
                    // Sanity: if target ended up wildly in the future
                    // (clock mismatch / underflow), skip the wait.
                    uint64_t now = time_us_64();
                    if (target > now && (target - now) < 1000u) {
                        busy_wait_us_32((uint32_t)(target - now));
                    }
                }
            }

            g_dbg_core1_bc = 8;
            {
                uint64_t now = time_us_64();
                if (next_us > now && (next_us - now) < 1000u) {
                    busy_wait_us_32((uint32_t)(next_us - now));
                } else if (next_us <= now) {
                    // We're behind — resync to avoid chasing forever.
                    next_us = now;
                }
            }

#if UNDERRUN_LED_ENABLE
            if (PICO_DEFAULT_LED_PIN != (uint)-1) {
                if (absolute_time_diff_us(get_absolute_time(), led_off_time) <= 0) {
                    gpio_put(PICO_DEFAULT_LED_PIN, 0);
                }
            }
#endif
        }

        __compiler_memory_barrier();
        g_block_ready[b] = 0;
        __compiler_memory_barrier();

        g_cons_block = (b + 1u) % NUM_BLOCKS;
        g_dbg_cons_blocks++;
    }
}

// ==========================================================
// USB audio pump (TinyUSB task + UAC RX read)
// ==========================================================
static void usb_audio_pump(void) {
    // utrzymuj TinyUSB — but rate-limit when no host is connected
    // to avoid wasting CPU cycles polling dead USB PHY
    static absolute_time_t tud_next = {0};
    if (tud_connected()) {
        tud_task();
        cdc_task();
    } else {
        // No USB host — call tud_task() at reduced rate (every 10ms)
        if (absolute_time_diff_us(get_absolute_time(), tud_next) <= 0) {
            tud_task();
            tud_next = make_timeout_time_ms(10);
        }
        return;  // No audio to read without USB host
    }

    const uint32_t frame_bytes =
        (uint32_t)CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX *
        (uint32_t)CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX;

    if (!frame_bytes) return;

    uint32_t avail = tud_audio_available();
    if (avail < frame_bytes) return;

    static uint8_t tmp[512];

    uint32_t to_read = avail;
    if (to_read > sizeof(tmp)) to_read = sizeof(tmp);
    to_read = (to_read / frame_bytes) * frame_bytes;
    if (!to_read) return;

    uint32_t got = tud_audio_read(tmp, (uint16_t)to_read);
    if (!got) return;

    uint32_t frames = got / frame_bytes;
    const uint8_t *p = tmp;

    for (uint32_t i = 0; i < frames; i++) {
        int16_t l = (int16_t)(p[0] | (p[1] << 8));
        int16_t r = l;
        if (CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX >= 2) {
            r = (int16_t)(p[2] | (p[3] << 8));
        }
        p += frame_bytes;
        (void) usb_rb_push((stereo16_t){ .l = l, .r = r }); // drop if full
    }
}

// ==========================================================
// PIO Frequency Counter for TCXO on GP26
// Uses PIO state machine to count edges in 1-second window
// ==========================================================

// ==========================================================
// ==========================================================
// MAIN (CORE0): init + DSP producer
// ==========================================================
int main(void) {
    bool ok = set_sys_clock_khz(250000, false);
    if (!ok) set_sys_clock_khz(200000, true);

    // Load persisted configuration (freq, mode, power, ppm, etc.).
    // Done early so SX1280 setup below picks up the saved frequency.
    persist_load();

    // ---- USB device init ----
    board_init();
    tusb_rhport_init_t dev_init = {
        .role  = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL  // UAC1-only: Full-Speed dla Windows/Linux
    };
    tusb_init(BOARD_TUD_RHPORT, &dev_init);
    board_init_after_tusb();

    // ---- SX1280 GPIO/SPI init ----
    // CRITICAL FOR TCXO MODULE: Enable TCXO FIRST, before any SPI/reset!
#if USE_TCXO_MODULE
    gpio_init(PIN_TCXO_EN);
    gpio_set_dir(PIN_TCXO_EN, GPIO_OUT);
    gpio_put(PIN_TCXO_EN, 1);  // Enable TCXO FIRST!
    sleep_ms(5);               // Wait for TCXO to stabilize (min 3ms)
    printf("[SX1280] TCXO enabled (GPIO%d=HIGH)\n", PIN_TCXO_EN);
#endif

    gpio_init(PIN_NSS);   gpio_set_dir(PIN_NSS, GPIO_OUT);   gpio_put(PIN_NSS, 1);
    gpio_init(PIN_RX_EN); gpio_set_dir(PIN_RX_EN, GPIO_OUT); gpio_put(PIN_RX_EN, 0);
    gpio_init(PIN_TX_EN); gpio_set_dir(PIN_TX_EN, GPIO_OUT); gpio_put(PIN_TX_EN, 0);  // Start with TX disabled!
    gpio_init(PIN_RESET); gpio_set_dir(PIN_RESET, GPIO_OUT); gpio_put(PIN_RESET, 1);
    gpio_init(PIN_BUSY);  gpio_set_dir(PIN_BUSY, GPIO_IN);

    // --- Encoder + button GPIO init (input with pull-up, active LOW) ---
    gpio_init(PIN_ENC_A);   gpio_set_dir(PIN_ENC_A, GPIO_IN);   gpio_pull_up(PIN_ENC_A);
    gpio_init(PIN_ENC_B);   gpio_set_dir(PIN_ENC_B, GPIO_IN);   gpio_pull_up(PIN_ENC_B);
    gpio_init(PIN_ENC_OK);  gpio_set_dir(PIN_ENC_OK, GPIO_IN);  gpio_pull_up(PIN_ENC_OK);
    gpio_init(PIN_PTT_KEY); gpio_set_dir(PIN_PTT_KEY, GPIO_IN); gpio_pull_up(PIN_PTT_KEY);
    // Initialize encoder last state
    enc_last_ab = ((gpio_get(PIN_ENC_A) ? 0 : 1) << 1) | (gpio_get(PIN_ENC_B) ? 0 : 1);

    // --- ADC init for microphone input (ADC0 = GPIO26) ---
    adc_init();
    adc_gpio_init(PIN_ADC_MIC);
    adc_select_input(0);  // ADC0

    spi_init(SX_SPI, SX_SPI_BAUD);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);

    // --- OLED I2C init ---
    i2c_init(OLED_I2C, OLED_I2C_BAUD);
    gpio_set_function(PIN_OLED_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_OLED_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_OLED_SDA);
    gpio_pull_up(PIN_OLED_SCL);
    ssd1306_init(OLED_I2C);
    ssd1306_clear();
    ssd1306_draw_string(0, 0, "SX1280 SSB TX");
    ssd1306_draw_string(0, 2, "Booting...");
    ssd1306_display(OLED_I2C);

    // Hardware reset SX1280
    printf("[SX1280] Resetting...\n");
    gpio_put(PIN_RESET, 0); sleep_ms(2);
    gpio_put(PIN_RESET, 1); sleep_ms(10);
    printf("[SX1280] Reset complete, BUSY=%d\n", gpio_get(PIN_BUSY));

#if USE_TCXO_MODULE
    // For TCXO module, use STDBY_XOSC mode
    sx_set_standby_xosc();
    printf("[SX1280] Set STDBY_XOSC mode (for TCXO)\n");
#else
    sx_set_standby_rc();
    printf("[SX1280] Set STDBY_RC mode\n");
#endif

    sx_set_packet_type_gfsk();

    // Use runtime-configurable frequency with PPM correction
    uint32_t init_base_steps = get_base_steps();
    sx_set_rf_frequency_steps(init_base_steps);

    sx_set_tx_params_dbm((int32_t)PWR_MIN_DBM);  // Start with minimum power
    // TX_EN stays LOW - will be controlled by Core1 or CDC commands
    // Use 'cw' command via CDC to test transmission
    // DON'T start TX yet - wait for USB to enumerate and audio to start
    // sx_start_tx_continuous_wave();  -- moved to core1 when audio starts

#if FIXED_POWER_CW_MODE
    // For CW mode, wait for USB then start TX
    while (!tud_ready()) { tud_task(); sleep_ms(10); }
    sleep_ms(500);  // Extra delay for USB stability
    gpio_put(PIN_TX_EN, 1);
    sx_start_tx_continuous_wave();
    while (true) { tud_task(); tight_loop_contents(); }
#endif

    // *** Start Core1 early so it can idle and drain blocks immediately ***
    multicore_launch_core1(core1_radio_apply_loop);

    // Wait for USB — or timeout after 3 seconds (for powerbank / MIC-only use).
    // Meanwhile handle encoder, buttons, OLED, CW keying.
    {
        printf("[BOOT] Waiting for USB connection (max 3s, encoder+CW active)...\n");
        absolute_time_t usb_deadline = make_timeout_time_ms(3000);

        while (!tud_ready()) {
            tud_task();
            encoder_poll();
            button_poll();
            carrier_poll();
            oled_poll();

            // Timeout: proceed without USB (powerbank / MIC-only mode)
            if (absolute_time_diff_us(get_absolute_time(), usb_deadline) <= 0) {
                printf("[BOOT] USB timeout — starting without USB host (MIC mode available)\n");
                if (g_audio_src == 0) {
                    g_audio_src = 1;  // Auto-switch to MIC if USB not present
                    mic_timer_start();
                    printf("[BOOT] Auto-switched to MIC audio source\n");
                }
                break;
            }

            sleep_ms(1);
        }
        if (tud_ready()) {
            printf("[BOOT] USB connected, starting normal SSB mode\n");
        }
    }

    // Honour persisted audio source: if MIC was saved, start the mic timer
    // regardless of how USB negotiation went above.
    if (g_audio_src == 1 && !g_mic_timer_running) {
        mic_timer_start();
    }

    hilbert_init();

    const float Fs = (float)WAV_SAMPLE_RATE;

    float theta_prev = 0.0f;
    float f_acc = 0.0f;
    float fine_tune_phase = 0.0f;  // Phase accumulator for fine frequency tuning
    float ctcss_phase = 0.0f;     // Phase accumulator for CTCSS tone generator

    // --- Roger beep (FM mode) state ---
    // When TX de-asserts while g_roger_beep is enabled, we emit a short
    // tone by holding tx_on=1 and overriding the audio sample for
    // ROGER_BEEP_SAMPLES ticks (~100 ms @ 8 kHz).
    #define ROGER_BEEP_SAMPLES  800u     // 100 ms
    #define ROGER_BEEP_FREQ_HZ  1000.0f
    uint32_t roger_beep_left = 0;         // samples remaining
    float    roger_beep_phase = 0.0f;
    uint8_t  fm_prev_tx_req  = 0;         // previous "user wants TX" state

    // --- FM TX envelope ramping (anti-click) ---
    // When tx_on flips we smoothly ramp power + modulation depth with a
    // raised-cosine envelope so the carrier fades in/out instead of
    // hard-switching (which produces an audible "thump" in receivers).
    #define FM_RAMP_SAMPLES  40u   // 40 samples @ 8 kHz = 5 ms
    uint32_t fm_ramp_pos    = 0;        // 0..FM_RAMP_SAMPLES
    int8_t   fm_ramp_dir    = 0;        // +1 = ramp up, -1 = ramp down, 0 = idle
    uint8_t  fm_carrier_on  = 0;        // 1 while carrier is currently emitting

    float p_acc = 0.0f;
    float tx_acc = 0.0f;

    const float phi = (float)IQ_PHASE_CORR_DEG * (float)M_PI / 180.0f;
    const float cphi = cosf(phi);
    const float sphi = sinf(phi);

#if AUDIO_ENABLE_BANDPASS
    biquad_t bp_hpf[AUDIO_BP_MAX_STAGES];
    biquad_t bp_lpf[AUDIO_BP_MAX_STAGES];
    // init via apply_cfg_if_dirty()
#endif

#if AUDIO_ENABLE_EQ
    biquad_t eq_low, eq_high;
    // init via apply_cfg_if_dirty()
#endif

#if AUDIO_ENABLE_COMPRESSOR
    compressor_t comp;
    // init via apply_cfg_if_dirty()
#endif

#if USE_TEST_TONE
    float sine_phase1 = 0.0f;
    const float sine_inc1 = 2.0f * (float)M_PI * (float)TEST_TONE_HZ / Fs;
#if USE_TWO_TONE_TEST
    float sine_phase2 = 0.0f;
    const float sine_inc2 = 2.0f * (float)M_PI * (float)TEST_TONE2_HZ / Fs;
#endif
#endif

    audio_cfg_t cfg_local;
    memset(&cfg_local, 0, sizeof(cfg_local));

    // PRE-BUFFERING: fill half of the blocks before starting consumer
    // This prevents underruns at startup
    const uint32_t prebuf_blocks = NUM_BLOCKS / 2;
    
    // silence reset counter
    const uint32_t silence_samples = WAV_SAMPLE_RATE * SILENCE_SECONDS;
    uint32_t silence_ctr = 0;

    // greet once if CDC is connected later
    uint8_t greeted = 0;

    // *** Pre-fill some blocks before signaling Core1 to start ***
    const uint32_t prebuf_target = NUM_BLOCKS / 2;  // Fill half the buffer
    uint32_t prebuf_count = 0;

    while (true) {
        uint32_t b = g_prod_block;

        // === CW/TUNE active: Core1 just drains blocks, no DSP needed ===
        // Run a tight poll loop instead of producing audio blocks.
        // Without this, Core1 drains so fast that the wait loop below
        // never executes, starving encoder/button/cw_keying polls.
        if (g_cw_test_mode) {
            usb_audio_pump();
            oled_poll();

            encoder_poll();
            button_poll();
            carrier_poll();
            persist_maybe_autosave();

#if CFG_TUD_CDC
            cdc_task();
            cdc_status_push();
#endif
            tight_loop_contents();
            continue;   // Skip block production entirely
        }

        while (g_block_ready[b]) {
            usb_audio_pump();
            oled_poll();

            // Poll encoder + buttons + carrier state machine
            encoder_poll();
            button_poll();
            carrier_poll();
            persist_maybe_autosave();

#if CFG_TUD_CDC
            cdc_status_push();
#endif

            tight_loop_contents();
        }

#if CFG_TUD_CDC
        if (!greeted && tud_cdc_connected()) {
            greeted = 1;
            cdc_write_str("\r\nSX1280_SDR control ready. Type 'help'.\r\n");
            cfg_print();
        }
#endif

        // Apply pending cfg on block boundary
#if AUDIO_ENABLE_BANDPASS || AUDIO_ENABLE_EQ || AUDIO_ENABLE_COMPRESSOR
        apply_cfg_if_dirty(
            Fs,
#if AUDIO_ENABLE_BANDPASS
            bp_hpf, bp_lpf,
#else
            NULL, NULL,
#endif
#if AUDIO_ENABLE_EQ
            &eq_low, &eq_high,
#else
            NULL, NULL,
#endif
#if AUDIO_ENABLE_COMPRESSOR
            &comp,
#else
            NULL,
#endif
            &cfg_local
        );
#else
        // still snapshot cfg (amp, etc.)
        __compiler_memory_barrier();
        memcpy(&cfg_local, (const void*)&g_cfg, sizeof(cfg_local));
        __compiler_memory_barrier();
#endif

        // Get current base steps (with freq and PPM correction) at block boundary
        int32_t base_steps = (int32_t)get_base_steps();

        sample_cmd_t *blk = g_blocks[b];

        for (uint32_t n = 0; n < BLOCK_SAMPLES; n++) {
            if ((n & 0x07u) == 0u) usb_audio_pump();

            float x = 0.0f;

#if USE_TEST_TONE
#if USE_TWO_TONE_TEST
            // Two-tone test: sum of two sinusoids
            x = TEST_TONE_AMPL * (sinf(sine_phase1) + sinf(sine_phase2));
            sine_phase1 += sine_inc1;
            sine_phase2 += sine_inc2;
            if (sine_phase1 > 2.0f * (float)M_PI) sine_phase1 -= 2.0f * (float)M_PI;
            if (sine_phase2 > 2.0f * (float)M_PI) sine_phase2 -= 2.0f * (float)M_PI;
#else
            x = TEST_TONE_AMPL * sinf(sine_phase1);
            sine_phase1 += sine_inc1;
            if (sine_phase1 > 2.0f * (float)M_PI) sine_phase1 -= 2.0f * (float)M_PI;
#endif
#else
            // Audio source: USB (PC) or ADC (MIC)
            if (g_audio_src == 0) {
                // PC mode: USB audio → downsample → mono
                int16_t s = usb_audio_get_mono_8k();
                x = (float)s / 32768.0f;
            } else {
                // MIC mode: wait for sample from timer-driven ring buffer.
                // Timer ISR fills mic_rb at 8 kHz; we block here until a sample
                // is available — this naturally paces Core0 at 8 kHz.
                // While waiting, poll UI + refresh OLED so everything stays responsive.
                // Also check g_audio_src: if user switches to PC mid-block,
                // break out immediately to avoid deadlock (timer is stopped).
                while (g_mic_r == g_mic_w) {
                    if (g_audio_src == 0) break;  // Source switched — bail out
                    usb_audio_pump();
                    encoder_poll();
                    button_poll();
                    carrier_poll();
                    oled_poll();
#if CFG_TUD_CDC
                    cdc_status_push();
#endif
                }
                // If source changed mid-block, fill rest with silence
                // If source changed mid-block, fill rest with silence
                if (g_audio_src == 0) {
                    x = 0.0f;
                } else {
                    x = adc_mic_get_sample(
                        cfg_local.mic_agc_target,
                        cfg_local.mic_agc_max_gain,
                        cfg_local.mic_agc_attack,
                        cfg_local.mic_agc_release,
                        cfg_local.mic_gate_thresh);
                }
            }

            if (fabsf(x) < 1e-5f) {
                if (silence_ctr < silence_samples) silence_ctr++;
            } else {
                silence_ctr = 0;
            }

            if (silence_ctr == silence_samples) {
                hilbert_reset();
                theta_prev = 0.0f;
                f_acc = 0.0f;
                fine_tune_phase = 0.0f;
                p_acc = 0.0f;
                tx_acc = 0.0f;

#if AUDIO_ENABLE_BANDPASS
                for (int i = 0; i < AUDIO_BP_MAX_STAGES; i++) {
                    biquad_reset(&bp_hpf[i]);
                    biquad_reset(&bp_lpf[i]);
                }
#endif
#if AUDIO_ENABLE_EQ
                biquad_reset(&eq_low);
                biquad_reset(&eq_high);
#endif
#if AUDIO_ENABLE_COMPRESSOR
                comp.env = 0.0f;
#endif
                silence_ctr = silence_samples + 1u;
            }
#endif

#if AUDIO_ENABLE_EQ
            if (cfg_local.enable_eq) {
                x = biquad_process(&eq_low,  x);
                x = biquad_process(&eq_high, x);
            }
#endif

#if AUDIO_ENABLE_COMPRESSOR
            if (cfg_local.enable_comp) {
                x = compressor_process(&comp, x);
                // output limiter
                if (x > cfg_local.comp_out_limit) x = cfg_local.comp_out_limit;
                if (x < -cfg_local.comp_out_limit) x = -cfg_local.comp_out_limit;
            }
#endif

#if AUDIO_ENABLE_BANDPASS
            if (cfg_local.enable_bandpass) {
                for (int i = 0; i < cfg_local.bp_stages; i++) x = biquad_process(&bp_hpf[i], x);
                for (int i = 0; i < cfg_local.bp_stages; i++) x = biquad_process(&bp_lpf[i], x);
            }
#endif

        after_dsp:
            // ==================== FM MODE ====================
            // Direct frequency modulation: audio sample → frequency offset.
            // No Hilbert transform, no SSB I/Q, no amplitude shaping.
            // Constant power, constant TX on (when gated).
            if (g_tx_mode == TXM_FM) {
                // User's "want TX" request (independent of roger-beep)
                uint8_t tx_req = (g_tx_enabled || g_ptt_key) ? 1 : 0;
                if (tx_mode_guard_active()) {
                    tx_req = 0;
                    roger_beep_left = 0;  // cancel any pending beep
                }

                // Detect falling edge of TX request → start roger beep
                if (g_roger_beep && fm_prev_tx_req && !tx_req) {
                    roger_beep_left = ROGER_BEEP_SAMPLES;
                    roger_beep_phase = 0.0f;
                }
                fm_prev_tx_req = tx_req;

                // "Keep carrier up" request — includes roger-beep tail
                uint8_t want_carrier = tx_req || (roger_beep_left > 0);

                // Manage envelope ramp state machine
                if (want_carrier && !fm_carrier_on && fm_ramp_dir <= 0) {
                    // Start ramp-up
                    fm_ramp_dir = +1;
                    fm_ramp_pos = 0;
                    fm_carrier_on = 1;
                } else if (!want_carrier && fm_carrier_on && fm_ramp_dir >= 0) {
                    // Start ramp-down
                    fm_ramp_dir = -1;
                    fm_ramp_pos = FM_RAMP_SAMPLES;  // start from full
                }

                // Compute envelope value 0..1 for this sample
                float env;
                if (fm_ramp_dir > 0) {
                    fm_ramp_pos++;
                    if (fm_ramp_pos >= FM_RAMP_SAMPLES) {
                        fm_ramp_pos = FM_RAMP_SAMPLES;
                        fm_ramp_dir = 0;   // done
                    }
                    float frac = (float)fm_ramp_pos / (float)FM_RAMP_SAMPLES;
                    env = 0.5f * (1.0f - cosf((float)M_PI * frac));
                } else if (fm_ramp_dir < 0) {
                    if (fm_ramp_pos > 0) fm_ramp_pos--;
                    float frac = (float)fm_ramp_pos / (float)FM_RAMP_SAMPLES;
                    env = 0.5f * (1.0f - cosf((float)M_PI * frac));
                    if (fm_ramp_pos == 0) {
                        fm_ramp_dir = 0;
                        fm_carrier_on = 0;
                    }
                } else {
                    env = fm_carrier_on ? 1.0f : 0.0f;
                }

                uint8_t tx_on = fm_carrier_on ? 1 : 0;

                if (tx_on) {
                    if (roger_beep_left > 0) {
                        // Override audio with a sine tone, keep carrier up
                        x = 0.7f * sinf(roger_beep_phase);
                        roger_beep_phase += 2.0f * (float)M_PI * ROGER_BEEP_FREQ_HZ / Fs;
                        if (roger_beep_phase >= 2.0f * (float)M_PI) roger_beep_phase -= 2.0f * (float)M_PI;
                        roger_beep_left--;
                    } else if (tx_req && g_ctcss_freq > 0.0f) {
                        // Add CTCSS sub-audible tone if enabled
                        float ctcss_amp = 0.15f;
                        x = x * (1.0f - ctcss_amp) + ctcss_amp * sinf(ctcss_phase);
                        ctcss_phase += 2.0f * (float)M_PI * g_ctcss_freq / Fs;
                        if (ctcss_phase >= 2.0f * (float)M_PI) ctcss_phase -= 2.0f * (float)M_PI;
                    }
                    // Fade modulation depth with envelope so deviation
                    // also grows/shrinks smoothly, not just RF amplitude.
                    x *= env;
                } else {
                    x = 0.0f;
                }

                float fm_offset_hz = x * g_fm_deviation_hz;
                float fm_steps = fm_offset_hz / PLL_STEP_HZ;
                int32_t fm_int = (int32_t)floorf(fm_steps);
                float fm_frac = fm_steps - (float)fm_int;

                // Sigma-delta dithering for fractional step
                f_acc += fm_frac;
                int32_t fm_chosen = fm_int;
                if (f_acc >= 1.0f)       { fm_chosen += 1; f_acc -= 1.0f; }
                else if (f_acc <= -1.0f)  { fm_chosen -= 1; f_acc += 1.0f; }

                int32_t cur_steps = base_steps + fm_chosen;

                // Apply fine frequency tuning (sub-PLL-step correction)
                float fine_hz = get_fine_tune_hz();
                if (fine_hz != 0.0f) {
                    float fine_steps = fine_hz / PLL_STEP_HZ;
                    cur_steps += (int32_t)roundf(fine_steps);
                }

                // Power envelope: map env (0..1) from PWR_MIN..target linearly in dB
                int8_t target_dbm = g_tx_power_max_dbm;
                int8_t pwr_dbm;
                if (tx_on) {
                    float dbm_f = (float)PWR_MIN_DBM +
                                  env * ((float)target_dbm - (float)PWR_MIN_DBM);
                    int32_t pi = (int32_t)(dbm_f + 0.5f);
                    if (pi < PWR_MIN_DBM) pi = PWR_MIN_DBM;
                    if (pi > PWR_MAX_DBM) pi = PWR_MAX_DBM;
                    pwr_dbm = (int8_t)pi;
                } else {
                    pwr_dbm = PWR_MIN_DBM;
                }

                blk[n].freq_steps = cur_steps;
                blk[n].p_dbm      = pwr_dbm;
                blk[n].tx_on      = tx_on;
                continue;  // Skip SSB path below
            }

            // ==================== SSB MODE ====================
            float I;
            float Q = hilbert_process(x, &I);

            float Iq = I;
            float Qq = Q * (float)IQ_GAIN_CORR;

            float I2 = Iq * cphi - Qq * sphi;
            float Q2 = Iq * sphi + Qq * cphi;

            // Apply fine frequency tuning via complex carrier multiplication
            // Fine tune is calculated automatically from fractional Hz that PLL can't reach
            float fine_hz = get_fine_tune_hz();  // Auto-calculated from target freq + PPM
            if (fine_hz != 0.0f) {
                float fine_cos = cosf(fine_tune_phase);
                float fine_sin = sinf(fine_tune_phase);
                float I3 = I2 * fine_cos - Q2 * fine_sin;
                float Q3 = I2 * fine_sin + Q2 * fine_cos;
                I2 = I3;
                Q2 = Q3;
                fine_tune_phase += 2.0f * (float)M_PI * fine_hz / Fs;
                // Keep phase in [-π, π] to avoid precision loss
                if (fine_tune_phase > (float)M_PI)   fine_tune_phase -= 2.0f * (float)M_PI;
                if (fine_tune_phase < -(float)M_PI) fine_tune_phase += 2.0f * (float)M_PI;
            }

            float A = sqrtf(I2 * I2 + Q2 * Q2);

            float theta = atan2f(Q2, I2);

            float dtheta = theta - theta_prev;
            if (dtheta > (float)M_PI)   dtheta -= 2.0f * (float)M_PI;
            if (dtheta < -(float)M_PI) dtheta += 2.0f * (float)M_PI;
            theta_prev = theta;

            float f_off = dtheta * Fs / (2.0f * (float)M_PI);
            if (f_off > (float)F_OFF_LIMIT_HZ)  f_off = (float)F_OFF_LIMIT_HZ;
            if (f_off < -(float)F_OFF_LIMIT_HZ) f_off = -(float)F_OFF_LIMIT_HZ;

            float want_steps = f_off / PLL_STEP_HZ;
            int32_t Nf = (int32_t)floorf(want_steps);
            float ffrac = want_steps - (float)Nf;

            f_acc += ffrac;
            int32_t f_chosen = Nf;
            if (f_acc >= 1.0f) { f_chosen = Nf + 1; f_acc -= 1.0f; }

            int32_t cur_steps = base_steps + f_chosen;

            float duty = duty_from_A(A);

            int32_t p_chosen = PWR_MIN_DBM;
            uint8_t tx_on = 1;

            if (duty < 1.0f) {
                p_chosen = PWR_MIN_DBM;
                tx_acc += duty;
                if (tx_acc >= 1.0f) { tx_on = 1; tx_acc -= 1.0f; }
                else                { tx_on = 0; }
            } else {
                tx_on = 1;

                int8_t pwr_max = g_tx_power_max_dbm;  // Local copy for this sample
                float Aeff = A * cfg_local.amp_gain;
                if (Aeff < cfg_local.amp_min_a) Aeff = cfg_local.amp_min_a;

                float p_raw = (float)pwr_max + 20.0f * log10f(Aeff);

                float p_des = p_raw;
                if (p_des > (float)pwr_max) p_des = (float)pwr_max;
                if (p_des < (float)PWR_MIN_DBM) p_des = (float)PWR_MIN_DBM;

                int32_t p_low  = (int32_t)floorf(p_des);
                int32_t p_high = p_low + 1;

                if (p_low  < PWR_MIN_DBM) p_low  = PWR_MIN_DBM;
                if (p_high > pwr_max) p_high = pwr_max;

                float frac = p_des - (float)p_low;
                if (frac < 0.0f) frac = 0.0f;
                if (frac > 1.0f) frac = 1.0f;

                p_acc += frac;
                p_chosen = p_low;
                if (p_acc >= 1.0f && p_high != p_low) { p_chosen = p_high; p_acc -= 1.0f; }
            }

            // SSB TX gating: transmit if GUI TX=ON *or* PTT pressed.
            // Either source alone is sufficient (OR logic).
            // CW mode PTT is handled separately by carrier_poll.
            // Applies to USB-SSB (mode 0) only.
            if (g_tx_mode == TXM_USB && !g_tx_enabled && !g_ptt_key) {
                tx_on = 0;
            }
            // Hard-gate TX during mode-change guard window to avoid clicks
            if (tx_mode_guard_active()) tx_on = 0;

            blk[n].freq_steps = cur_steps;
            blk[n].p_dbm      = (int8_t)p_chosen;
            blk[n].tx_on      = tx_on;
        }

        // Diagnostic: count samples in this block that asked for TX
        {
            uint32_t cnt = 0;
            for (uint32_t n = 0; n < BLOCK_SAMPLES; n++) if (blk[n].tx_on) cnt++;
            g_dbg_prod_txon = cnt;
        }
        g_dbg_prod_blocks++;

        __compiler_memory_barrier();
        g_block_ready[b] = 1;
        __compiler_memory_barrier();

        g_prod_block = (b + 1u) % NUM_BLOCKS;

        // *** Signal Core1 to start after pre-buffering ***
        if (!g_core1_start) {
            prebuf_count++;
            if (prebuf_count >= prebuf_target) {
                __compiler_memory_barrier();
                g_core1_start = 1;
                __compiler_memory_barrier();
            }
        }
    }
}

// ==========================================================
// TinyUSB HID callbacks (stubs) – only compiled when HID is enabled
// ==========================================================
#if CFG_TUD_HID
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type,
                               uint8_t* buffer, uint16_t reqlen)
{
    (void)instance; (void)report_id; (void)report_type; (void)buffer; (void)reqlen;
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type,
                           uint8_t const* buffer, uint16_t bufsize)
{
    (void)instance; (void)report_id; (void)report_type; (void)buffer; (void)bufsize;
}
#endif /* CFG_TUD_HID */
