/*
 * i2s_audio.cpp
 *
 * Stereo I2S audio output for RP2350 using Arduino-Pico I2S library
 * PCM5101A DAC: 16-bit stereo, 48kHz
 * DCO1 -> Left channel, DCO2 -> Right channel
 *
 * Arduino-Pico I2S: BCLK pin and DATA pin specified,
 * LRCK is always BCLK+1 automatically.
 */

#include "i2s_audio.h"
#include <Arduino.h>
#include <I2S.h>

/* --------------------------------------------------------
 * Configuration
 * -------------------------------------------------------- */
#define BUFFER_SIZE     256

/* --------------------------------------------------------
 * I2S instance - must be global, constructed before use
 * -------------------------------------------------------- */
static I2S i2sOut(OUTPUT);

/* --------------------------------------------------------
 * Internal buffers
 * -------------------------------------------------------- */
static float dco1Buf[BUFFER_SIZE];
static float dco2Buf[BUFFER_SIZE];

/* --------------------------------------------------------
 * API
 * -------------------------------------------------------- */
void I2SAudio_Init(uint8_t bclkPin, uint8_t lrckPin, uint8_t dataPin,
                   uint32_t sample_rate)
{
    (void)lrckPin;  /* LRCK is always bclkPin+1 in Arduino-Pico */

    i2sOut.setBCLK(bclkPin);
    i2sOut.setDATA(dataPin);
    i2sOut.setBitsPerSample(16);
    i2sOut.begin(sample_rate);

    /* pre-fill first buffer */
    I2S_CB_FillBuffer(dco1Buf, dco2Buf, BUFFER_SIZE);
}

void I2SAudio_Process(void)
{
    /* write current buffer to I2S as interleaved stereo */
    for (int n = 0; n < BUFFER_SIZE; n++)
    {
        float l = dco1Buf[n];
        float r = dco2Buf[n];
        if (l >  1.0f) l =  1.0f;
        if (l < -1.0f) l = -1.0f;
        if (r >  1.0f) r =  1.0f;
        if (r < -1.0f) r = -1.0f;

        int16_t left  = (int16_t)(l * 32767.0f);
        int16_t right = (int16_t)(r * 32767.0f);

        i2sOut.write(left);
        i2sOut.write(right);
    }

    /* refill buffer for next call */
    I2S_CB_FillBuffer(dco1Buf, dco2Buf, BUFFER_SIZE);
}
