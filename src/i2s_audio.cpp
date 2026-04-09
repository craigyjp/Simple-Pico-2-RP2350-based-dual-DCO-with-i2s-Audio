/*
 * i2s_audio.cpp
 *
 * Stereo I2S audio output for RP2350 using Arduino-Pico I2S library
 * PCM5101A DAC: 16-bit stereo, 48kHz
 * DCO1 -> Left channel, DCO2 -> Right channel
 *
 * Uses onTransmit callback to fill buffers at correct audio rate
 * ensuring DCO_ProcessBoth runs exactly once per DMA buffer
 */

#include "i2s_audio.h"
#include <Arduino.h>
#include <I2S.h>

#define BUFFER_SIZE     256

static I2S i2sOut(OUTPUT);

static float dco1Buf[BUFFER_SIZE];
static float dco2Buf[BUFFER_SIZE];
static volatile bool needsFill = false;

static void onTransmit()
{
    /* called when DMA buffer consumed - signal main loop to refill */
    needsFill = true;
}

void I2SAudio_Init(uint8_t bclkPin, uint8_t lrckPin, uint8_t dataPin,
                   uint32_t sample_rate)
{
    (void)lrckPin;

    i2sOut.setBCLK(bclkPin);
    i2sOut.setDATA(dataPin);
    i2sOut.setBitsPerSample(16);
    i2sOut.setBuffers(4, BUFFER_SIZE);
    i2sOut.onTransmit(onTransmit);
    i2sOut.begin(sample_rate);

    /* fill initial buffer */
    I2S_CB_FillBuffer(dco1Buf, dco2Buf, BUFFER_SIZE);
    for (int n = 0; n < BUFFER_SIZE; n++)
    {
        int16_t l = (int16_t)(dco1Buf[n] * 32767.0f);
        int16_t r = (int16_t)(dco2Buf[n] * 32767.0f);
        i2sOut.write(l);
        i2sOut.write(r);
    }
}

void I2SAudio_Process(void)
{
    if (!needsFill) return;
    needsFill = false;

    I2S_CB_FillBuffer(dco1Buf, dco2Buf, BUFFER_SIZE);

    for (int n = 0; n < BUFFER_SIZE; n++)
    {
        float l = dco1Buf[n];
        float r = dco2Buf[n];
        if (l >  1.0f) l =  1.0f;
        if (l < -1.0f) l = -1.0f;
        if (r >  1.0f) r =  1.0f;
        if (r < -1.0f) r = -1.0f;

        i2sOut.write((int16_t)(l * 32767.0f));
        i2sOut.write((int16_t)(r * 32767.0f));
    }
}
