/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Konrad Beckmann
 */

#include "utils.h"

#include <stdio.h>
#include "FreeRTOS.h"
#include <stdatomic.h>


#include "hardware/watchdog.h"

static uint8_t __attribute__((aligned(16))) usb_buffer[USB_BUFFER_SIZE] = {0};
uint32_t usb_bytes_received = 0;
_Atomic uint32_t incoming_usb_store_word = 0;
_Atomic uint32_t outgoing_usb_x_word = 0;
_Atomic uint32_t outgoing_usb_y_word = 0;
_Atomic uint32_t outgoing_usb_z_word = 0;
_Atomic uint32_t alt_usb_debug_word = 0;
_Atomic uint32_t crash_alt_usb_debug_word = 0;


void assert_handler(char *file, int line, char *statement)
{
	printf("Assert failed %s:%d:\n  %s\n", file, line, statement);

	// Reboot after 5s
	watchdog_reboot(0, 0, 5000);

	while (true) {
		tight_loop_contents();
	}
}
