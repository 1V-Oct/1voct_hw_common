#pragma once
#include "app_config.h"

#if WITH_NOR_FLASH
#ifdef __cplusplus
 extern "C" {
#endif


enum {
  SFLASH_CMD_READ = 0x03,      // Single Read
  SFLASH_CMD_FAST_READ = 0x0B, // Fast Read
  SFLASH_CMD_QUAD_READ = 0x6B, // 1 line address, 4 line data

  SFLASH_CMD_READ_JEDEC_ID = 0x9f,

  SFLASH_CMD_PAGE_PROGRAM = 0x02,
  SFLASH_CMD_QUAD_PAGE_PROGRAM = 0x32, // 1 line address, 4 line data

  SFLASH_CMD_READ_STATUS = 0x05,
  SFLASH_CMD_READ_STATUS2 = 0x35,

  SFLASH_CMD_WRITE_STATUS = 0x01,
  SFLASH_CMD_WRITE_STATUS2 = 0x31,

  SFLASH_CMD_ENABLE_RESET = 0x66,
  SFLASH_CMD_RESET = 0x99,

  SFLASH_CMD_WRITE_ENABLE = 0x06,
  SFLASH_CMD_WRITE_DISABLE = 0x04,

  SFLASH_CMD_ERASE_SECTOR = 0x20,
  SFLASH_CMD_ERASE_BLOCK = 0xD8,
  SFLASH_CMD_ERASE_CHIP = 0xC7,

  SFLASH_CMD_4_BYTE_ADDR = 0xB7,
  SFLASH_CMD_3_BYTE_ADDR = 0xE9,

  SFLASH_CMD_READ_UNIQUE_ID = 0x4b,
};


typedef struct {
  uint32_t total_size;
  uint16_t start_up_time_us;

  // Three response bytes to 0x9f JEDEC ID command.
  uint8_t manufacturer_id;
  uint8_t memory_type;
  uint8_t capacity;

  // Max clock speed for all operations and the fastest read mode.
  uint8_t max_clock_speed_mhz;

  // Bitmask for Quad Enable bit if present. 0x00 otherwise. This is for the
  // highest byte in the status register.
  uint8_t quad_enable_bit_mask;

  uint8_t has_sector_protection : 1;

  // Supports the 0x0b fast read command with 8 dummy cycles.
  uint8_t supports_fast_read : 1;

  // Supports the fast read, quad output command 0x6b with 8 dummy cycles.
  uint8_t supports_qspi : 1;

  // Supports the quad input page program command 0x32. This is known as 1-1-4
  // because it only uses all four lines for data.
  uint8_t supports_qspi_writes : 1;

  // Requires a separate command 0x31 to write to the second byte of the status
  // register. Otherwise two byte are written via 0x01.
  uint8_t write_status_register_split : 1;

  // True when the status register is a single byte. This implies the Quad
  // Enable bit is in the first byte and the Read Status Register 2 command
  // (0x35) is unsupported.
  uint8_t single_status_byte : 1;

  // Fram does not need/support erase and has much simpler WRITE operation
  uint8_t is_fram : 1;

} SPIFlash_Device_t;

// Settings for the Adesto Tech AT25DF081A 1MiB SPI flash. Its on the SAMD21
// Xplained board.
// Datasheet: https://www.adestotech.com/wp-content/uploads/doc8715.pdf
#define AT25DF081A                                                             \
  {                                                                            \
    .total_size = (1 << 20), /* 1 MiB */                                       \
        .start_up_time_us = 10000, .manufacturer_id = 0x1f,                    \
    .memory_type = 0x45, .capacity = 0x01, .max_clock_speed_mhz = 85,          \
    .quad_enable_bit_mask = 0x00, .has_sector_protection = true,               \
    .supports_fast_read = true, .supports_qspi = false,                        \
    .supports_qspi_writes = false, .write_status_register_split = false,       \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Gigadevice GD25Q16C 2MiB SPI flash.
// Datasheet: http://www.gigadevice.com/datasheet/gd25q16c/
#define GD25Q16C                                                               \
  {                                                                            \
    .total_size = (1 << 21), /* 2 MiB */                                       \
        .start_up_time_us = 5000, .manufacturer_id = 0xc8,                     \
    .memory_type = 0x40, .capacity = 0x15,                                     \
    .max_clock_speed_mhz =                                                     \
        104, /* if we need 120 then we can turn on high performance mode */    \
        .quad_enable_bit_mask = 0x02, .has_sector_protection = false,          \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Gigadevice GD25Q64C 8MiB SPI flash.
// Datasheet:
// http://www.elm-tech.com/en/products/spi-flash-memory/gd25q64/gd25q64.pdf
#define GD25Q64C                                                               \
  {                                                                            \
    .total_size = (1 << 23), /* 8 MiB */                                       \
        .start_up_time_us = 5000, .manufacturer_id = 0xc8,                     \
    .memory_type = 0x40, .capacity = 0x17,                                     \
    .max_clock_speed_mhz =                                                     \
        104, /* if we need 120 then we can turn on high performance mode */    \
        .quad_enable_bit_mask = 0x02, .has_sector_protection = false,          \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = true,         \
    .single_status_byte = false, .is_fram = false,                             \
  }

// https://www.fujitsu.com/uk/Images/MB85RS64V.pdf
#define MB85RS64V                                                              \
  {                                                                            \
    .total_size = 8 * 1024, /* 8 KiB */                                        \
        .start_up_time_us = 5000, .manufacturer_id = 0x04,                     \
    .memory_type = 0x7F, .capacity = 0x03, .max_clock_speed_mhz = 20,          \
    .quad_enable_bit_mask = 0x00, .has_sector_protection = false,              \
    .supports_fast_read = false, .supports_qspi = false,                       \
    .supports_qspi_writes = false, .write_status_register_split = false,       \
    .single_status_byte = true, .is_fram = true,                               \
  }

// https://www.fujitsu.com/uk/Images/MB85RS1MT.pdf
#define MB85RS1MT                                                              \
  {                                                                            \
    .total_size = (1 << 17), /* 128 KiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0x04,                     \
    .memory_type = 0x7F, .capacity = 0x27, .max_clock_speed_mhz = 40,          \
    .quad_enable_bit_mask = 0x00, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = false,                        \
    .supports_qspi_writes = false, .write_status_register_split = false,       \
    .single_status_byte = true, .is_fram = true,                               \
  }

// https://www.fujitsu.com/uk/Images/MB85RS2MTA.pdf
#define MB85RS2MTA                                                             \
  {                                                                            \
    .total_size = (1 << 18), /* 256 KiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0x04,                     \
    .memory_type = 0x7F, .capacity = 0x48, .max_clock_speed_mhz = 40,          \
    .quad_enable_bit_mask = 0x00, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = false,                        \
    .supports_qspi_writes = false, .write_status_register_split = false,       \
    .single_status_byte = true, .is_fram = true,                               \
  }

// https://www.fujitsu.com/uk/Images/MB85RS4MT.pdf
#define MB85RS4MT                                                              \
  {                                                                            \
    .total_size = (1 << 19), /* 512 KiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0x04,                     \
    .memory_type = 0x7F, .capacity = 0x49, .max_clock_speed_mhz = 40,          \
    .quad_enable_bit_mask = 0x00, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = false,                        \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = true, .is_fram = true,                               \
  }

// Settings for the Macronix MX25L1606 2MiB SPI flash.
// Datasheet:
#define MX25L1606                                                              \
  {                                                                            \
    .total_size = (1 << 21), /* 2 MiB */                                       \
        .start_up_time_us = 5000, .manufacturer_id = 0xc2,                     \
    .memory_type = 0x20, .capacity = 0x15, .max_clock_speed_mhz = 8,           \
    .quad_enable_bit_mask = 0x40, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = true, .is_fram = false,                              \
  }

// Settings for the Macronix MX25L3233F 4MiB SPI flash.
// Datasheet:
// http://www.macronix.com/Lists/Datasheet/Attachments/7426/MX25L3233F,%203V,%2032Mb,%20v1.6.pdf
#define MX25L3233F                                                             \
  {                                                                            \
    .total_size = (1 << 22), /* 4 MiB */                                       \
        .start_up_time_us = 5000, .manufacturer_id = 0xc2,                     \
    .memory_type = 0x20, .capacity = 0x16, .max_clock_speed_mhz = 133,         \
    .quad_enable_bit_mask = 0x40, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = true, .is_fram = false,                              \
  }

// Settings for the Macronix MX25R6435F 8MiB SPI flash.
// Datasheet:
// http://www.macronix.com/Lists/Datasheet/Attachments/7428/MX25R6435F,%20Wide%20Range,%2064Mb,%20v1.4.pdf
// By default its in lower power mode which can only do 8mhz. In high power mode
// it can do 80mhz.
#define MX25R6435F                                                             \
  {                                                                            \
    .total_size = (1 << 23), /* 8 MiB */                                       \
        .start_up_time_us = 5000, .manufacturer_id = 0xc2,                     \
    .memory_type = 0x28, .capacity = 0x17, .max_clock_speed_mhz = 8,           \
    .quad_enable_bit_mask = 0x40, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = true, .is_fram = false,                              \
  }

// Settings for the Cypress (was Spansion) S25FL064L 8MiB SPI flash.
// Datasheet: http://www.cypress.com/file/316661/download
#define S25FL064L                                                              \
  {                                                                            \
    .total_size = (1 << 23), /* 8 MiB */                                       \
        .start_up_time_us = 300, .manufacturer_id = 0x01, .memory_type = 0x60, \
    .capacity = 0x17, .max_clock_speed_mhz = 108,                              \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Cypress (was Spansion) S25FL116K 2MiB SPI flash.
// Datasheet: http://www.cypress.com/file/196886/download
#define S25FL116K                                                              \
  {                                                                            \
    .total_size = (1 << 21), /* 2 MiB */                                       \
        .start_up_time_us = 10000, .manufacturer_id = 0x01,                    \
    .memory_type = 0x40, .capacity = 0x15, .max_clock_speed_mhz = 108,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = false, .write_status_register_split = false,       \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Cypress (was Spansion) S25FL216K 2MiB SPI flash.
// Datasheet: http://www.cypress.com/file/197346/download
#define S25FL216K                                                              \
  {                                                                            \
    .total_size = (1 << 21), /* 2 MiB */                                       \
        .start_up_time_us = 10000, .manufacturer_id = 0x01,                    \
    .memory_type = 0x40, .capacity = 0x15, .max_clock_speed_mhz = 65,          \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = false,                        \
    .supports_qspi_writes = false, .write_status_register_split = false,       \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Winbond W25Q80DL 1MiB SPI flash.
// https://www.winbond.com/resource-files/w25q80dv%20dl_revh_10022015.pdf
#define W25Q80DL                                                               \
  {                                                                            \
    .total_size = (1 << 20), /* 1 MiB */                                       \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x60, .capacity = 0x14, .max_clock_speed_mhz = 104,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = false, .write_status_register_split = false,       \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Winbond W25Q16FW 2MiB SPI flash.
// Datasheet:
// https://www.winbond.com/resource-files/w25q16fw%20revj%2005182017%20sfdp.pdf
#define W25Q16FW                                                               \
  {                                                                            \
    .total_size = (1 << 21), /* 2 MiB */                                       \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x60, .capacity = 0x15, .max_clock_speed_mhz = 133,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Winbond W25Q16JV-IQ 2MiB SPI flash. Note that JV-IM has a
// different .memory_type (0x70) Datasheet:
// https://www.winbond.com/resource-files/w25q16jv%20spi%20revf%2005092017.pdf
#define W25Q16JV_IQ                                                            \
  {                                                                            \
    .total_size = (1 << 21), /* 2 MiB */                                       \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x40, .capacity = 0x15, .max_clock_speed_mhz = 133,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Winbond W25Q16JV-IM 2MiB SPI flash. Note that JV-IQ has a
// different .memory_type (0x40) Datasheet:
// https://www.winbond.com/resource-files/w25q16jv%20spi%20revf%2005092017.pdf
#define W25Q16JV_IM                                                            \
  {                                                                            \
    .total_size = (1 << 21), /* 2 MiB */                                       \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x70, .capacity = 0x15, .max_clock_speed_mhz = 133,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
  }

// Settings for the Winbond W25Q32BV 4MiB SPI flash.
// Datasheet:
// https://www.winbond.com/resource-files/w25q32bv_revi_100413_wo_automotive.pdf
#define W25Q32BV                                                               \
  {                                                                            \
    .total_size = (1 << 22), /* 4 MiB */                                       \
        .start_up_time_us = 10000, .manufacturer_id = 0xef,                    \
    .memory_type = 0x60, .capacity = 0x16, .max_clock_speed_mhz = 104,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = false, .write_status_register_split = false,       \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Winbond W25Q32FV 4MiB SPI flash.
// Datasheet:http://www.winbond.com/resource-files/w25q32fv%20revj%2006032016.pdf?__locale=en
#define W25Q32FV                                                               \
  {                                                                            \
    .total_size = (1 << 22), /* 4 MiB */                                       \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x40, .capacity = 0x16, .max_clock_speed_mhz = 104,         \
    .quad_enable_bit_mask = 0x00, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = false,                        \
    .supports_qspi_writes = false, .write_status_register_split = false,       \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Winbond W25Q32JV-IM 4MiB SPI flash.
// https://www.winbond.com/resource-files/w25q32jv%20revg%2003272018%20plus.pdf
#define W25Q32JV_IM                                                            \
  {                                                                            \
    .total_size = (1 << 22), /* 4 MiB */                                       \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x70, .capacity = 0x16, .max_clock_speed_mhz = 133,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
  }

// Settings for the Winbond W25Q64JV-IM 8MiB SPI flash. Note that JV-IQ has a
// different .memory_type (0x40) Datasheet:
// http://www.winbond.com/resource-files/w25q64jv%20revj%2003272018%20plus.pdf
#define W25Q64JV_IM                                                            \
  {                                                                            \
    .total_size = (1 << 23), /* 8 MiB */                                       \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x70, .capacity = 0x17, .max_clock_speed_mhz = 133,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Winbond W25Q64JV-IQ 8MiB SPI flash. Note that JV-IM has a
// different .memory_type (0x70) Datasheet:
// http://www.winbond.com/resource-files/w25q64jv%20revj%2003272018%20plus.pdf
#define W25Q64JV_IQ                                                            \
  {                                                                            \
    .total_size = (1 << 23), /* 8 MiB */                                       \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x40, .capacity = 0x17, .max_clock_speed_mhz = 133,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Winbond W25Q128JV-SQ 16MiB SPI flash. Note that JV-IM has a
// different .memory_type (0x70) Datasheet:
// https://www.winbond.com/resource-files/w25q128jv%20revf%2003272018%20plus.pdf
#define W25Q128JV_SQ                                                           \
  {                                                                            \
    .total_size = (1 << 24), /* 16 MiB */                                      \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x40, .capacity = 0x18, .max_clock_speed_mhz = 133,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Winbond W25Q128JV-PM 16MiB SPI flash. Note that JV-IM has a
// different .memory_type (0x70) Datasheet:
// https://www.winbond.com/resource-files/w25q128jv%20revf%2003272018%20plus.pdf
#define W25Q128JV_PM                                                           \
  {                                                                            \
    .total_size = (1 << 24), /* 16 MiB */                                      \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x70, .capacity = 0x18, .max_clock_speed_mhz = 133,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Winbond W25Q256JV 32MiB SPI flash.
// https://www.winbond.com/resource-files/w25q256jv%20spi%20revg%2008032017.pdf
#define W25Q256JV                                                              \
  {                                                                            \
    .total_size = (1 << 25), /* 32 MiB */                                      \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x40, .capacity = 0x19, .max_clock_speed_mhz = 133,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = false, .is_fram = false,                             \
  }




uint8_t nor_init(void);

void nor_erase_chip(void);
// void nor_read(uint8_t *data, uint32_t addr, uint32_t len);

int32_t nor_read_memory(uint32_t addr, void *data, uint32_t len);
int32_t nor_write_memory(uint32_t addr, void *data, uint32_t len);
void nor_erase_sector(uint32_t addr);

#ifdef __cplusplus
}
#endif

#endif