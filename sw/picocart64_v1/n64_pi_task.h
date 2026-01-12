/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Konrad Beckmann
 */
#include "pc64_regs.h"
#pragma once

/**
 * @file n64_pi_task.h
 * @brief Header file for the N64 PI task
 */

/**
 * @brief Executes the N64 PI task.
 */
void n64_pi_run(void);

/* Written by PI task (core1), read by USB tasks (core0) */
extern volatile uint32_t usb_rx_words[USB_COMM_WORDS];
extern volatile uint32_t usb_rx_seq;

/* Written by USB tasks (core0), read by PI task (core1) */
extern volatile uint32_t usb_tx_words_buf[2][USB_COMM_WORDS];
extern volatile uint32_t usb_tx_seq;