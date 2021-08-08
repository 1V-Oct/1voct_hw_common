//#pragma GCC optimize("Os")
#include "nor_flash.h"
#include "vo_string.h"
#include "vo_logger.h"
#include "gpio.h"

static const uint8_t _addr_len = 3;
static uint8_t _cmd_read = SFLASH_CMD_READ;


#define NOR_CS_LO gpio_reset_pin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);
#define NOR_CS_HI gpio_set_pin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);

void nor_fill_address(uint8_t *buf, uint32_t addr) {
  switch (_addr_len) {
  case 4:
    *buf++ = (addr >> 24) & 0xFF;
    __attribute__((fallthrough)); // ESP32 doesn't support this attribute yet
    // fall through

  case 3:
    *buf++ = (addr >> 16) & 0xFF;
    __attribute__((fallthrough)); // ESP32 doesn't support this attribute yet
    // fall through

  case 2:
  default:
    *buf++ = (addr >> 8) & 0xFF;
    *buf++ = addr & 0xFF;
    break;
  }
}

uint8_t nor_send_recv(uint8_t data) {
  uint8_t	ret;
    __HAL_SPI_ENABLE(&hspi4);

	if (HAL_SPI_TransmitReceive(&hspi4, &data, &ret, 1, 100) != HAL_OK) {
    LOGE("ERR");
  }
	return ret;	
}

void nor_recv(uint8_t *data, uint32_t len) {
  if (HAL_SPI_Receive(&hspi4, data, len, HAL_MAX_DELAY) != HAL_OK) {
    LOGE("NOR Receive Error");
  }
}

void nor_send(uint8_t *data, uint32_t len) {
  if (HAL_SPI_Transmit(&hspi4, data, len, HAL_MAX_DELAY) != HAL_OK) {
    LOGE("NOR Transmit Error");
  }
}


void nor_cmd(uint8_t cmd) {
  gpio_reset_pin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);
  nor_send(&cmd, 1);
  // uint8_t buf;
  // buf = command;
  // HAL_SPI_Transmit(&hspi4, &buf, 1, 1);
  gpio_set_pin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);
}

void nor_read_cmd(uint8_t cmd, uint8_t *data, uint32_t len) {
  gpio_reset_pin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);
  nor_send(&cmd, 1);
  nor_recv(data, len);
  gpio_set_pin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);
}

void not_write_cmd(uint8_t cmd, uint8_t *data, uint32_t len) {
  gpio_reset_pin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);
  nor_send(&cmd, 1);
  nor_send(data, len);
  gpio_set_pin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);
}

void nor_cmd_with_address(uint8_t cmd, uint32_t addr) {
  uint8_t buf[5];
  buf[0] = cmd;
  nor_fill_address(buf + 1, addr);
  nor_send(buf, 5);
}

static void nor_read(uint8_t *data, uint32_t addr, uint32_t len) {
  nor_cmd_with_address(SFLASH_CMD_READ, addr);
  nor_recv(data, len);
}

static void nor_read_cmd_with_address(uint8_t cmd, uint8_t *data, uint32_t addr, uint32_t len) {
  nor_cmd_with_address(cmd, addr);
  nor_recv(data, len);
}

void nor_wait_for_write(void) {
  uint8_t res;
  do {
    nor_read_cmd(SFLASH_CMD_READ_STATUS, &res, 1);
  } while(res & 0x01);

}


void nor_erase_chip(void) {
  LOGI("ERASE CHIP");
  nor_cmd(SFLASH_CMD_WRITE_ENABLE);
  nor_cmd(SFLASH_CMD_ERASE_CHIP);
  nor_cmd(SFLASH_CMD_WRITE_DISABLE);
  LOGI("Wait for write completed");
  nor_wait_for_write();
  LOGI("*COMPLETED*");
}



/**
 * @brief Erases sector that contains given address
 * 
 * @param addr address within sector to be erased
 */
void nor_erase_sector(uint32_t addr) {
  uint8_t cmd_with_addr[6];
  cmd_with_addr[0] = SFLASH_CMD_ERASE_SECTOR;
  nor_fill_address(cmd_with_addr + 1, addr);
  gpio_reset_pin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);
  nor_send(cmd_with_addr, 1 + _addr_len);
  gpio_set_pin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);
}

#if 1
void nor_read_memory(uint32_t addr, void *data, uint32_t len) {
  uint8_t cmd_with_addr[6];

  gpio_reset_pin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);

  cmd_with_addr[0] = _cmd_read;
  nor_fill_address(cmd_with_addr + 1, addr);

  // Fast Read has 1 extra dummy byte
  uint8_t const cmd_len =
      1 + _addr_len + (SFLASH_CMD_FAST_READ == _cmd_read ? 1 : 0);
  nor_send(cmd_with_addr, cmd_len);
  nor_recv(data, len);

  gpio_set_pin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);
}

void nor_write_memory(uint32_t addr, void *data, uint32_t len) {
  uint8_t cmd_with_addr[5];
  uint8_t status;

  // nor_read_cmd(SFLASH_CMD_READ_STATUS, &status, 1);
  // LOGI("STATUS %02x", status);

  nor_cmd(SFLASH_CMD_WRITE_ENABLE);

  nor_read_cmd(SFLASH_CMD_READ_STATUS, &status, 1);
  LOGI("STATUS %02x", status);


  cmd_with_addr[0] = SFLASH_CMD_PAGE_PROGRAM;
  nor_fill_address(cmd_with_addr + 1, addr);

  uint8_t const cmd_len =1 + _addr_len;
  gpio_reset_pin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);
  nor_send(cmd_with_addr, cmd_len);
  nor_send(data, len);
  gpio_set_pin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);
  
  // nor_wait_for_write();
  // LOGI("COMPLETED");

  nor_cmd(SFLASH_CMD_WRITE_DISABLE);

}
#endif

void nor_init(void) {
  uint8_t jedec_ids[4];
  nor_cmd(SFLASH_CMD_RESET);
  nor_read_cmd(SFLASH_CMD_READ_JEDEC_ID, jedec_ids, 3);
  LOGI("NOR FLASH: %02x %02x %02x", jedec_ids[0], jedec_ids[1], jedec_ids[2]);
#if 1
  uint8_t uid[16];
  NOR_CS_LO;
  nor_read_cmd_with_address(SFLASH_CMD_READ_UNIQUE_ID, uid, 0, 16);
  NOR_CS_HI;
  int i;
  for(i = 0 ; i < 16 ; i++) {
    uart_printf("%02x", uid[i]);
  }
  uart_printf("\n");
  uart_printf("%s\n", uid);
#endif
}