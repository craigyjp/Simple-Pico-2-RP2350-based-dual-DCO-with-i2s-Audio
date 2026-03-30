/*
 * i2s_audio.h
 *
 * Stereo I2S audio output for RP2350 stamp
 * Target: PCM5101A DAC
 * DCO1 -> Left channel, DCO2 -> Right channel
 *
 * Arduino-Pico I2S pin mapping:
 *   BCLK = GPIO9
 *   LRCK = GPIO10 (always BCLK+1 in Arduino-Pico)
 *   DATA = GPIO11
 */

#ifndef I2S_AUDIO_H_
#define I2S_AUDIO_H_

#include <stdint.h>
#include <Arduino.h>

/* --------------------------------------------------------
 * Callback - implement in main application
 * out1: DCO1 (left)  -1.0 to +1.0
 * out2: DCO2 (right) -1.0 to +1.0
 * -------------------------------------------------------- */
void I2S_CB_FillBuffer(float *out1, float *out2, int len);

/* --------------------------------------------------------
 * API
 * Note: lrckPin is ignored - always bclkPin+1 in Arduino-Pico
 * -------------------------------------------------------- */
void I2SAudio_Init(uint8_t bclkPin, uint8_t lrckPin, uint8_t dataPin,
                   uint32_t sample_rate);
void I2SAudio_Process(void);

#endif /* I2S_AUDIO_H_ */
