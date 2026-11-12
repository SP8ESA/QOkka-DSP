/*
 * tusb_config.h - UAC1 + CDC (ACM), Full-Speed
 */
#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------+
// Board Specific Configuration
//--------------------------------------------------------------------+

#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT      0
#endif

#ifndef BOARD_TUD_MAX_SPEED
#define BOARD_TUD_MAX_SPEED   OPT_MODE_FULL_SPEED
#endif

//--------------------------------------------------------------------+
// Common Configuration
//--------------------------------------------------------------------+

#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined
#endif

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS           OPT_OS_NONE
#endif

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG        0
#endif

#define CFG_TUD_ENABLED       1

#define CFG_TUD_MAX_SPEED     OPT_MODE_FULL_SPEED

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN        __attribute__ ((aligned(4)))
#endif

//--------------------------------------------------------------------+
// DEVICE CONFIGURATION
//--------------------------------------------------------------------+

#ifndef CFG_AUDIO_DEBUG
#define CFG_AUDIO_DEBUG           0
#endif

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE    64
#endif

#define CFG_TUD_HID_EP_BUFSIZE    64

//------------- CLASS -------------//
#define CFG_TUD_AUDIO             1

#if CFG_AUDIO_DEBUG
#define CFG_TUD_HID               1
#else
#define CFG_TUD_HID               0
#endif

// >>> WŁĄCZAMY CDC (Serial po USB)
#define CFG_TUD_CDC               1

#define CFG_TUD_MSC               0
#define CFG_TUD_MIDI              0
#define CFG_TUD_VENDOR            0

//--------------------------------------------------------------------+
// CDC CLASS DRIVER CONFIGURATION
//--------------------------------------------------------------------+

#ifndef CFG_TUD_CDC_RX_BUFSIZE
#define CFG_TUD_CDC_RX_BUFSIZE    512
#endif

#ifndef CFG_TUD_CDC_TX_BUFSIZE
#define CFG_TUD_CDC_TX_BUFSIZE    2048
#endif

//--------------------------------------------------------------------+
// AUDIO CLASS DRIVER CONFIGURATION (UAC1 Speaker OUT + Feedback)
//--------------------------------------------------------------------+

#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX              2
#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX      2
#define CFG_TUD_AUDIO_FUNC_1_RESOLUTION_RX              16

// UAC1 Full-Speed endpoint size: 48kHz * 2ch * 2bytes / 1000 = 192 bytes per frame
#define CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE_FS     48000
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_FS           192

// Provide compatibility define expected by newer audio_device.h
#ifndef CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_FS
#endif

// FIFO / SW buffer
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ       (4 * CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_FS)

// Enable OUT EP, DISABLE feedback EP (Windows compatible)
#define CFG_TUD_AUDIO_ENABLE_EP_OUT                 1
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP            0

#ifdef __cplusplus
}
#endif

#endif /* TUSB_CONFIG_H_ */
