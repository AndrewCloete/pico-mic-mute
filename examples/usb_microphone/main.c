/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * This examples creates a USB Microphone device using the TinyUSB
 * library and captures data from a PDM microphone using a sample
 * rate of 16 kHz, to be sent the to PC.
 *
 * The USB microphone code is based on the TinyUSB audio_test example.
 *
 * https://github.com/hathach/tinyusb/tree/master/examples/device/audio_test
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/analog_microphone.h"
#include "usb_microphone.h"

#define PTT_PIN 14
#define LATCH_PIN 15
#define LED_PIN_1 1
#define LED_PIN_2 2

// configuration
const struct analog_microphone_config config = {
    // GPIO to use for input, must be ADC compatible (GPIO 26 - 28)
    .gpio = 26,

    // bias voltage of microphone in volts
    .bias_voltage = 1.25,

    // sample rate in Hz
    .sample_rate = SAMPLE_RATE,

    // number of samples to buffer
    .sample_buffer_size = SAMPLE_BUFFER_SIZE,
};

// variables
uint16_t sample_buffer[SAMPLE_BUFFER_SIZE];
uint16_t mute_buffer[SAMPLE_BUFFER_SIZE];
bool latch_state = false;
bool pin_state = false;
bool pin_ack = false;

// callback functions
void on_analog_samples_ready();
void on_usb_microphone_tx_ready();

void red(void)
{
  gpio_put(LED_PIN_1, 1);
  gpio_put(LED_PIN_2, 0);
}

void green(void)
{
  gpio_put(LED_PIN_1, 0);
  gpio_put(LED_PIN_2, 1);
}

void indicate(bool state)
{
  if (state)
  {
    red();
  }
  else
  {
    green();
  }
}

int main(void)
{
  stdio_init_all();
  gpio_init(LATCH_PIN);
  gpio_set_dir(LATCH_PIN, GPIO_IN);
  gpio_pull_up(LATCH_PIN);
  gpio_init(LED_PIN_1);
  gpio_set_dir(LED_PIN_1, GPIO_OUT);
  gpio_init(LED_PIN_2);
  gpio_set_dir(LED_PIN_2, GPIO_OUT);
  indicate(latch_state);

      // initialize and start the Analog microphone
      analog_microphone_init(&config);
  analog_microphone_set_samples_ready_handler(on_analog_samples_ready);
  analog_microphone_start();

  // initialize the USB microphone interface
  usb_microphone_init();
  usb_microphone_set_tx_ready_handler(on_usb_microphone_tx_ready);

  while (1)
  {
    // run the USB microphone task continuously
    usb_microphone_task();
    pin_state = gpio_get(LATCH_PIN);
    if (pin_state)
    {
      pin_ack = false;
    }
    if (!pin_state && !pin_ack)
    {
      sleep_ms(50); // debounce
      if (!gpio_get(LATCH_PIN))
      {
        latch_state = !latch_state;
        indicate(latch_state);
        pin_ack = true;
      }
    }
  }

  return 0;
}

void on_analog_samples_ready()
{
  // callback from library when all the samples in the library
  // internal sample buffer are ready for reading
  analog_microphone_read(sample_buffer, SAMPLE_BUFFER_SIZE);
}

void on_usb_microphone_tx_ready()
{
  // Callback from TinyUSB library when all data is ready
  // to be transmitted.
  //
  // Write local buffer to the USB microphone
  if (latch_state)
  {
    usb_microphone_write(sample_buffer, sizeof(sample_buffer));
  }
  else
  {
    usb_microphone_write(mute_buffer, sizeof(mute_buffer));
  }
}
