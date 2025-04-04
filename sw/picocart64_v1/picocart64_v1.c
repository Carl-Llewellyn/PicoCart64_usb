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

  while (true) {
    // Wait until USB is connected
    while (!tud_cdc_connected()) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Check for available data
    if (tud_cdc_available()) {
      usb_bytes_received = tud_cdc_read(usb_buffer, USB_BUFFER_SIZE);
      if (usb_bytes_received >= sizeof(uint32_t)) {
        incoming_usb_store_word = __builtin_bswap32(*(uint32_t *)usb_buffer);
      }
    }
      // Yield to other tasks
      vTaskDelay(pdMS_TO_TICKS(10));
  }

}


StaticTask_t outgoing_usb_task;
StackType_t outgoing_usb_task_stack[USB_TASK_STACK_SIZE];

void outgoing_usb_task_entry(void *pvParameters) {

  uint32_t lastXSentData = 0;
  uint32_t lastYSentData = 0;
  uint32_t lastZSentData = 0;
  uint32_t debugSentData = 0;
  uint32_t crashDebugSentData = 0;

  while (true) {
    // Wait until USB is connected
    while (!tud_cdc_connected()) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (lastXSentData != outgoing_usb_x_word || lastYSentData != outgoing_usb_y_word || lastZSentData != outgoing_usb_z_word) {
     // printf("X: %f, Y: %f, Z: %f\n", ((float*)&outgoing_usb_x_word), ((float*)&outgoing_usb_y_word), ((float*)&outgoing_usb_z_word));//bitwise representation of a f32 in u32
      printf("X:  0x%08X, Y:  0x%08X, Z:  0x%08X\n", outgoing_usb_x_word, outgoing_usb_y_word, outgoing_usb_z_word);
      lastXSentData = outgoing_usb_x_word;
      lastYSentData = outgoing_usb_y_word;
      lastZSentData = outgoing_usb_z_word;
    }

    if (debugSentData != alt_usb_debug_word) {
      printf("D: 0x%08X.\n", alt_usb_debug_word);
      debugSentData = alt_usb_debug_word;
    }

    if (crashDebugSentData != crash_alt_usb_debug_word) {
      printf("CR: 0x%08X.\n", crash_alt_usb_debug_word);
      crashDebugSentData = crash_alt_usb_debug_word;
    }
      // Yield to other tasks
      vTaskDelay(pdMS_TO_TICKS(10));
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
