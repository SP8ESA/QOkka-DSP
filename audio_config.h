#ifndef AUDIO_CONFIG_H
#define AUDIO_CONFIG_H
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

#endif