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
#include "pico/bootrom.h"
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

static bool is_bootsel_command(const uint8_t pkt[22]) {
  static const uint8_t magic[22] = {
    0x50, 0x43, 0x36, 0x34, 0x42, 0x4f, 0x4f, 0x54, 0x53, 0x45, 0x4c,
    0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21
  };
  for (int i = 0; i < 22; i++) {
    if (pkt[i] != magic[i]) {
      return false;
    }
  }
  return true;
}

static bool bootsel_ring_match(const uint8_t ring[22], int head) {
  static const uint8_t magic[22] = {
    0x50, 0x43, 0x36, 0x34, 0x42, 0x4f, 0x4f, 0x54, 0x53, 0x45, 0x4c,
    0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21
  };
  for (int i = 0; i < 22; i++) {
    if (ring[(head + i) % 22] != magic[i]) {
      return false;
    }
  }
  return true;
}

// Convert N64-word order -> USB little-endian bytes
static void words_to_usb_le_bytes(uint8_t out30[USB_COMM_PKT_BYTES],
                                  const uint32_t words[USB_COMM_WORDS]) {
  uint8_t tmp32[USB_COMM_BYTES];
  for (int i = 0; i < USB_COMM_WORDS; i++) {
    uint32_t w = words[i];
    //N64 big-endian bytes 
    tmp32[i*4+0] = (uint8_t)(w >> 24);
    tmp32[i*4+1] = (uint8_t)(w >> 16);
    tmp32[i*4+2] = (uint8_t)(w >>  8);
    tmp32[i*4+3] = (uint8_t)(w >>  0);
  }

  //header bytes as-is 
  out30[0] = tmp32[0];
  out30[1] = tmp32[1];
  out30[2] = tmp32[2];
  out30[3] = tmp32[3];

  //3x float words (bytes 4..15): swap 32-bit endianness 
  for (int k = 0; k < 3; k++) {
    uint32_t be =
      ((uint32_t)tmp32[4 + k*4 + 0] << 24) |
      ((uint32_t)tmp32[4 + k*4 + 1] << 16) |
      ((uint32_t)tmp32[4 + k*4 + 2] <<  8) |
      ((uint32_t)tmp32[4 + k*4 + 3] <<  0);
    uint32_t le = bswap32(be);
    memcpy(out30 + 4 + k*4, &le, 4);
  }

  // s16 fields (bytes 16..23) + buttons (24..25): swap 16-bit endianness
  for (int i = 16; i <= 24; i += 2) {
    uint16_t be = (uint16_t)((tmp32[i] << 8) | tmp32[i + 1]);
    uint16_t le = bswap16(be);
    memcpy(out30 + i, &le, 2);
  }

  // sticks (26,27) + reserved (28,29)
  out30[26] = tmp32[26];
  out30[27] = tmp32[27];
  out30[28] = tmp32[28];
  out30[29] = tmp32[29];
}

// Convert USB little-endian bytes -> N64 words (big-endian word meaning)
static void usb_le_bytes_to_words(uint32_t out_words[USB_COMM_WORDS],
                                  const uint8_t in30[USB_COMM_PKT_BYTES]) {
  uint8_t tmp32[USB_COMM_BYTES] = {0};
  memcpy(tmp32, in30, USB_COMM_PKT_BYTES);

  // floats: input is little-endian 32-bit; convert back to big-endian bytes
  for (int k = 0; k < 3; k++) {
    uint32_t le;
    memcpy(&le, in30 + 4 + k*4, 4);
    uint32_t be = bswap32(le);
    tmp32[4 + k*4 + 0] = (uint8_t)(be >> 24);
    tmp32[4 + k*4 + 1] = (uint8_t)(be >> 16);
    tmp32[4 + k*4 + 2] = (uint8_t)(be >>  8);
    tmp32[4 + k*4 + 3] = (uint8_t)(be >>  0);
  }

  // s16 fields (bytes 16..23) + buttons (24..25): input little-endian -> big-endian bytes
  for (int i = 16; i <= 24; i += 2) {
    uint16_t le;
    memcpy(&le, in30 + i, 2);
    uint16_t be = bswap16(le);
    tmp32[i] = (uint8_t)(be >> 8);
    tmp32[i + 1] = (uint8_t)(be & 0xFF);
  }

  // Pack bytes into words (big-endian meaning)
  for (int i = 0; i < USB_COMM_WORDS; i++) {
    out_words[i] =
      ((uint32_t)tmp32[i*4+0] << 24) |
      ((uint32_t)tmp32[i*4+1] << 16) |
      ((uint32_t)tmp32[i*4+2] <<  8) |
      ((uint32_t)tmp32[i*4+3] <<  0);
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
  uint8_t accum[USB_COMM_PKT_BYTES];
  int acc_n = 0;
  uint8_t bootsel_ring[22];
  int bootsel_head = 0;
  int bootsel_count = 0;

  while (true) {
    while (!tud_cdc_connected()) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }

    while (tud_cdc_available()) {
      uint8_t tmp[64];
      uint32_t n = tud_cdc_read(tmp, sizeof(tmp));

      for (uint32_t i = 0; i < n; i++) {
        uint8_t byte = tmp[i];
        accum[acc_n++] = byte;
        bootsel_ring[bootsel_head] = byte;
        bootsel_head = (bootsel_head + 1) % 22;
        if (bootsel_count < 22) {
          bootsel_count++;
        }
        if (bootsel_count == 22 && bootsel_ring_match(bootsel_ring, bootsel_head)) {
          reset_usb_boot(0, 0);
        }

        if (acc_n == USB_COMM_PKT_BYTES) {
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

      uint8_t pkt[USB_COMM_PKT_BYTES];
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
