/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Konrad Beckmann
 */

#pragma once

// PicoCart64 Address space

// [READ/WRITE]: Scratch memory used for various functions
#define PC64_BASE_ADDRESS_START    0x81000000
#define PC64_BASE_ADDRESS_LENGTH   0x00001000
#define PC64_BASE_ADDRESS_END      (PC64_BASE_ADDRESS_START + PC64_BASE_ADDRESS_LENGTH - 1)

// [READ]: Returns pseudo-random values.
//         Address does not matter.
//         Each returned 16-bit word generates a new random value.
//         PC64_REGISTER_RESET_RAND resets the random seed.
#define PC64_RAND_ADDRESS_START    0x82000000
#define PC64_RAND_ADDRESS_LENGTH   0x01000000
#define PC64_RAND_ADDRESS_END      (PC64_RAND_ADDRESS_START + PC64_RAND_ADDRESS_LENGTH - 1)

// [READ/WRITE]: Command address space. See register definitions below for details.
#define PC64_CIBASE_ADDRESS_START  0x83000000
#define PC64_CIBASE_ADDRESS_LENGTH 0x00001000
#define PC64_CIBASE_ADDRESS_END    (PC64_CIBASE_ADDRESS_START + PC64_CIBASE_ADDRESS_LENGTH - 1)

// [READ]: Returns PC64_MAGIC
#define PC64_REGISTER_MAGIC        0x00000000
#define PC64_MAGIC                 0xDEAD6400

// [WRITE]: Write number of bytes to print from TX buffer
#define PC64_REGISTER_UART_TX      0x00000004

// [WRITE]: Set the random seed to a 32-bit value
#define PC64_REGISTER_RAND_SEED    0x00000008

// [READ]: Returns the JEDEC ID of the flash chip
#define PC64_REGISTER_FLASH_JEDEC_ID 0x0000000C

// [READ] RX BUFF
#define PC64_REGISTER_UART_RX 0x00000010


// OOT uses separate 32-byte SRAM windows for each packet direction.
// 0xA8007A00: N64 writes its outgoing packet here.
// 0xA8007A20: N64 reads the latest incoming packet here.
#define USB_COMM_BYTES          32u
#define USB_COMM_WORDS          (USB_COMM_BYTES / 4u)  // 8
#define USB_COMM_PKT_BYTES      30u

#define USB_COMM_N64_WRITE_BASE (CART_SRAM_START + 0x00007A00u)
#define USB_COMM_N64_WRITE_END  (USB_COMM_N64_WRITE_BASE + USB_COMM_BYTES - 1u)

#define USB_COMM_N64_READ_BASE  (CART_SRAM_START + 0x00007A20u)
#define USB_COMM_N64_READ_END   (USB_COMM_N64_READ_BASE + USB_COMM_BYTES - 1u)
