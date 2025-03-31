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

#if 0
static void second_task_entry(__unused void *params)
{
	uint32_t count = 0;

	printf("second_task_entry\n");

	while (true) {
		vTaskDelay(1000);
		count++;

		// Set to 1 to print stack watermarks.
		// Printing is synchronous and interferes with the CIC emulation.
#if 0
		// printf("Second task heartbeat: %d\n", count);
		// vPortYield();

		if (count > 10) {
			printf("watermark: %d\n", uxTaskGetStackHighWaterMark(NULL));
			vPortYield();

			printf("watermark second_task: %d\n", uxTaskGetStackHighWaterMark((TaskHandle_t) & second_task));
			vPortYield();

			printf("watermark cic_task: %d\n", uxTaskGetStackHighWaterMark((TaskHandle_t) & cic_task));
			vPortYield();
		}
#endif

	}
}
#endif

#define USB_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define USB_TASK_PRIORITY (tskIDLE_PRIORITY + 1)

// Static allocation for USB task
StaticTask_t usb_task;
StackType_t usb_task_stack[USB_TASK_STACK_SIZE];

uint32_t swap_endianness(uint32_t value) {
  return ((value >> 24) & 0x000000FF) | ((value >> 8) & 0x0000FF00) |
         ((value << 8) & 0x00FF0000) | ((value << 24) & 0xFF000000);
}

void usb_task_entry(void *pvParameters) {

  while (true) {
    // Wait until USB is connected
    while (!tud_cdc_connected()) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Check for available data
    if (tud_cdc_available()) {
      usb_bytes_received = tud_cdc_read(usb_buffer, USB_BUFFER_SIZE);
      if (usb_bytes_received >= sizeof(uint32_t)) {
          read_word = __builtin_bswap32(*(uint32_t *)usb_buffer);
      }
    }
  }

  // Yield to other tasks
  vTaskDelay(pdMS_TO_TICKS(10));
}

void vLaunch(void) {
  xTaskCreateStatic(cic_task_entry, "CICThread", configMINIMAL_STACK_SIZE, NULL,
                    CIC_TASK_PRIORITY, cic_task_stack, &cic_task);
  // xTaskCreateStatic(second_task_entry, "SecondThread",
  // configMINIMAL_STACK_SIZE, NULL, SECOND_TASK_PRIORITY, second_task_stack,
  // &second_task);

  xTaskCreateStatic(usb_task_entry, "USBThread", USB_TASK_STACK_SIZE, NULL,
                    USB_TASK_PRIORITY, usb_task_stack, &usb_task);

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

  stdio_init_all();
  //  sleep_ms(2000); // Add a delay here
  printf("Hello, world!\n");
  //   sleep_ms(2000); // Add a delay here
  //  printf("Hello, world!\n");

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
