/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Konrad Beckmann
 */

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hardware/flash.h"
#include "hardware/irq.h"
#include "pico/multicore.h"
#include "tusb.h"

#include "pico/stdlib.h"

#if PICO_SDK_VERSION_MAJOR >= 2 ||                                             \
    (PICO_SDK_VERSION_MAJOR == 1 &&                                            \
     (PICO_SDK_VERSION_MINOR > 6 ||                                            \
      PICO_SDK_VERSION_MINOR == 6 && PICO_SDK_VERSION_REVISION >= 2))
/* https://github.com/raspberrypi/pico-sdk/issues/712 */
#include "hardware/clocks.h"
#endif

// #include "stdio_async_uart.h"

#include "git_info.h"
#include "n64_cic.h"
#include "n64_pi_task.h"
#include "picocart64_pins.h"
#include "sram.h"
#include "utils.h"

#define ENABLE_N64_PI 1

// Priority 0 = lowest, 3 = highest
#define CIC_TASK_PRIORITY (3UL)
#define SECOND_TASK_PRIORITY (1UL)

static StaticTask_t cic_task;
static StaticTask_t second_task;
static StackType_t cic_task_stack[4 * 1024 / sizeof(StackType_t)];
static StackType_t second_task_stack[4 * 1024 / sizeof(StackType_t)];

uint32_t g_flash_jedec_id;

// FreeRTOS boilerplate
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize) {
  static StaticTask_t xTimerTaskTCB;
  static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

  *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

static inline uint16_t bswap16(uint16_t x) { return (uint16_t)((x << 8) | (x >> 8)); }
static inline uint32_t bswap32(uint32_t x) {
  return ((x & 0x000000FFu) << 24) |
         ((x & 0x0000FF00u) <<  8) |
         ((x & 0x00FF0000u) >>  8) |
         ((x & 0xFF000000u) >> 24);
}

//Convert N64-word order -> USB little-endian bytes 
static void words_to_usb_le_bytes(uint8_t out22[22], const uint32_t words6[USB_COMM_WORDS]) {
  uint8_t tmp24[24];
  for (int i = 0; i < USB_COMM_WORDS; i++) {
    uint32_t w = words6[i];
    //N64 big-endian bytes 
    tmp24[i*4+0] = (uint8_t)(w >> 24);
    tmp24[i*4+1] = (uint8_t)(w >> 16);
    tmp24[i*4+2] = (uint8_t)(w >>  8);
    tmp24[i*4+3] = (uint8_t)(w >>  0);
  }

  //header bytes as-is 
  out22[0] = tmp24[0];
  out22[1] = tmp24[1];
  out22[2] = tmp24[2];
  out22[3] = tmp24[3];

  //3x float words (bytes 4..15): swap 32-bit endianness 
  for (int k = 0; k < 3; k++) {
    uint32_t be =
      ((uint32_t)tmp24[4 + k*4 + 0] << 24) |
      ((uint32_t)tmp24[4 + k*4 + 1] << 16) |
      ((uint32_t)tmp24[4 + k*4 + 2] <<  8) |
      ((uint32_t)tmp24[4 + k*4 + 3] <<  0);
    uint32_t le = bswap32(be);
    memcpy(out22 + 4 + k*4, &le, 4);
  }

  //buttons u16 (bytes 16..17): swap 16-bit endianness 
  {
    uint16_t be = (uint16_t)((tmp24[16] << 8) | tmp24[17]);
    uint16_t le = bswap16(be);
    memcpy(out22 + 16, &le, 2);
  }

  //sticks (18,19) and remaining (20,21)
  out22[18] = tmp24[18];
  out22[19] = tmp24[19];
  out22[20] = tmp24[20];
  out22[21] = tmp24[21];
}

//Convert USB little-endian bytes -> N64 words (big-endian word meaning)
static void usb_le_bytes_to_words(uint32_t out_words6[USB_COMM_WORDS], const uint8_t in22[22]) {
  uint8_t tmp24[24] = {0};
  memcpy(tmp24, in22, 22);

  //floats: input is little-endian 32-bit; convert back to big-endian bytes 
  for (int k = 0; k < 3; k++) {
    uint32_t le;
    memcpy(&le, in22 + 4 + k*4, 4);
    uint32_t be = bswap32(le);
    tmp24[4 + k*4 + 0] = (uint8_t)(be >> 24);
    tmp24[4 + k*4 + 1] = (uint8_t)(be >> 16);
    tmp24[4 + k*4 + 2] = (uint8_t)(be >>  8);
    tmp24[4 + k*4 + 3] = (uint8_t)(be >>  0);
  }

  //buttons: input little-endian u16 -> big-endian bytes 
  {
    uint16_t le;
    memcpy(&le, in22 + 16, 2);
    uint16_t be = bswap16(le);
    tmp24[16] = (uint8_t)(be >> 8);
    tmp24[17] = (uint8_t)(be & 0xFF);
  }

  //Pack bytes into 6 words (big-endian meaning) 
  for (int i = 0; i < USB_COMM_WORDS; i++) {
    out_words6[i] =
      ((uint32_t)tmp24[i*4+0] << 24) |
      ((uint32_t)tmp24[i*4+1] << 16) |
      ((uint32_t)tmp24[i*4+2] <<  8) |
      ((uint32_t)tmp24[i*4+3] <<  0);
  }
}


void cic_task_entry(__unused void *params) {
  printf("cic_task_entry\n");

  sram_load_from_flash();

  n64_cic_hw_init();

  // TODO: Performing the write to flash in a separate task is the way to go
  n64_cic_task(sram_save_to_flash);
}


#define USB_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define USB_TASK_PRIORITY (tskIDLE_PRIORITY + 1)


StaticTask_t incoming_usb_task;
StackType_t incoming_usb_task_stack[USB_TASK_STACK_SIZE];

void incoming_usb_task_entry(void *pvParameters) {
  uint8_t accum[22];
  int acc_n = 0;

  while (true) {
    while (!tud_cdc_connected()) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }

    while (tud_cdc_available()) {
      uint8_t tmp[64];
      uint32_t n = tud_cdc_read(tmp, sizeof(tmp));

      for (uint32_t i = 0; i < n; i++) {
        accum[acc_n++] = tmp[i];

        if (acc_n == 22) {
          uint32_t words[USB_COMM_WORDS];
          usb_le_bytes_to_words(words, accum);

          uint32_t next = (usb_tx_seq + 1u) & 1u;
          for (int w = 0; w < USB_COMM_WORDS; w++) {
            usb_tx_words_buf[next][w] = words[w];
          }

          usb_tx_seq++;

          acc_n = 0;
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}



StaticTask_t outgoing_usb_task;
StackType_t outgoing_usb_task_stack[USB_TASK_STACK_SIZE];

void outgoing_usb_task_entry(void *pvParameters) {
  uint32_t last_seq = 0;

  while (true) {
    while (!tud_cdc_connected()) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }

    uint32_t seq = usb_rx_seq;
    if (seq != last_seq) {
      uint32_t snap[USB_COMM_WORDS];
      uint32_t seq2;

      /* seqlock-ish snapshot: copy, recheck */
      do {
        seq = usb_rx_seq;
        for (int i = 0; i < USB_COMM_WORDS; i++) snap[i] = usb_rx_words[i];
        seq2 = usb_rx_seq;
      } while (seq != seq2);

      uint8_t pkt[22];
      words_to_usb_le_bytes(pkt, snap);

      tud_cdc_write(pkt, sizeof(pkt));
      tud_cdc_write_flush();

      last_seq = seq2;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void vLaunch(void) {
  xTaskCreateStatic(cic_task_entry, "CICThread", configMINIMAL_STACK_SIZE, NULL,
                    CIC_TASK_PRIORITY, cic_task_stack, &cic_task);

  stdio_init_all();
  xTaskCreateStatic(incoming_usb_task_entry, "IncomingUSBThread",
                    USB_TASK_STACK_SIZE, NULL, USB_TASK_PRIORITY,
                    incoming_usb_task_stack, &incoming_usb_task);

  xTaskCreateStatic(outgoing_usb_task_entry, "OutgoingUSBThread",
                    USB_TASK_STACK_SIZE, NULL, USB_TASK_PRIORITY,
                    outgoing_usb_task_stack, &outgoing_usb_task);

  /* Start the tasks and timer running. */
  vTaskStartScheduler();
}

#include "rom_vars.h"

uint32_t flash_get_jedec_id(void) {
  const uint8_t read_jedec_id = 0x9f;
  uint8_t txbuf[4] = {read_jedec_id};
  uint8_t rxbuf[4] = {0};
  txbuf[0] = read_jedec_id;
  flash_do_cmd(txbuf, rxbuf, 4);

  return rxbuf[1] | (rxbuf[2] << 8) | (rxbuf[3] << 16);
}

int main(void) {

  // First, let's probe the Flash ID
  g_flash_jedec_id = flash_get_jedec_id();

  // Overclock!
  // The external flash should be rated to 133MHz,
  // but since it's used with a 2x clock divider,
  // 266 MHz is safe in this regard.

  // make sure to overclock BEFORE setting up USB init or you'll have a bad time
  set_sys_clock_khz(CONFIG_CPU_FREQ_MHZ * 1000, true);


  // Init GPIOs before starting the second core and FreeRTOS
  for (int i = 0; i <= 27; i++) {
    gpio_init(i);
    gpio_set_dir(i, GPIO_IN);
    gpio_set_pulls(i, false, false);
  }

  // Set up ROM mapping table
  if (memcmp(picocart_header, "picocartcompress", 16) == 0) {
    // Copy rom compressed map from flash into RAM
    memcpy(rom_mapping, flash_rom_mapping,
           MAPPING_TABLE_LEN * sizeof(uint16_t));
  } else {
    for (int i = 0; i < MAPPING_TABLE_LEN; i++) {
      rom_mapping[i] = i;
    }
  }

  // Enable pull up on N64_CIC_DIO since there is no external one.
  gpio_pull_up(N64_CIC_DIO);

  printf("PicoCart64 Boot (git rev %08x)\r\n", GIT_REV);
  printf("  CPU_FREQ_MHZ=%d\n", CONFIG_CPU_FREQ_MHZ);
  printf("  ROM_HEADER_OVERRIDE=%08lX\n", CONFIG_ROM_HEADER_OVERRIDE);

#if ENABLE_N64_PI
  // Launch the N64 PI implementation in the second core
  // Note! You have to power reset the pico after flashing it with a jlink,
  //       otherwise multicore doesn't work properly.
  //       Alternatively, attach gdb to openocd, run `mon reset halt`, `c`.
  //       It seems this works around the issue as well.
  multicore_launch_core1(n64_pi_run);
#endif

  // Start FreeRTOS on Core0
  vLaunch();

  return 0;
}
