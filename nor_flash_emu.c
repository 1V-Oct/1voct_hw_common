#include "nor_flash.h"

#if WITH_NOR_FLASH_EMU

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include "vo_logger.h"
#include "vo_memory.h"

#define NOR_FLASH_IMAGE_PATH "nor_flash.img"

// Using GD25Q64C as a reference from nor_flash.h, which is 8MiB.
#define NOR_FLASH_SIZE (1 << 23)
#define NOR_SECTOR_SIZE 4096 // 4KB sector size is common

static FILE *nor_image_file = NULL;
static bool is_initialized = false;

/**
 * @brief Initializes the emulated NOR flash.
 *
 * @return uint8_t 1 on success, 0 on failure.
 */
uint8_t nor_init(void) {
  if (is_initialized) {
    return 1;
  }

  nor_image_file = fopen(NOR_FLASH_IMAGE_PATH, "r+b");
  if (nor_image_file == NULL) {
    LOGI("Creating NOR flash image file: %s", NOR_FLASH_IMAGE_PATH);
    nor_image_file = fopen(NOR_FLASH_IMAGE_PATH, "w+b");
    if (nor_image_file == NULL) {
      LOGE("Failed to create NOR flash image file.");
      return 0;
    }

    // Set the file size and initialize with 0xFF (erased state)
    if (ftruncate(fileno(nor_image_file), NOR_FLASH_SIZE) != 0) {
        LOGE("Failed to set NOR flash image size.");
        fclose(nor_image_file);
        nor_image_file = NULL;
        return 0;
    }
    nor_erase_chip();
  }

  is_initialized = true;
  LOGI("NOR Flash emulation initialized.");
  return 1;
}

/**
 * @brief Erases the entire emulated flash chip.
 */
void nor_erase_chip(void) {
  if (!nor_image_file) return;

  LOGI("Erasing entire chip...");
  fseek(nor_image_file, 0, SEEK_SET);
  uint8_t *erase_buffer = vo_malloc(NOR_SECTOR_SIZE);
  if (!erase_buffer) {
      LOGE("Failed to allocate buffer for chip erase");
      return;
  }
  memset(erase_buffer, 0xFF, NOR_SECTOR_SIZE);

  for (size_t i = 0; i < NOR_FLASH_SIZE / NOR_SECTOR_SIZE; ++i) {
    fwrite(erase_buffer, 1, NOR_SECTOR_SIZE, nor_image_file);
  }
  fflush(nor_image_file);
  vo_free(erase_buffer);
  LOGI("Chip erase complete.");
}

/**
 * @brief Erases a sector in the emulated flash.
 *
 * @param addr Address within the sector to be erased.
 */
void nor_erase_sector(uint32_t addr) {
  if (!nor_image_file) return;

  uint32_t sector_start = addr - (addr % NOR_SECTOR_SIZE);
  if (sector_start >= NOR_FLASH_SIZE) return;

  fseek(nor_image_file, sector_start, SEEK_SET);

  uint8_t *erase_buffer = vo_malloc(NOR_SECTOR_SIZE);
   if (!erase_buffer) {
      LOGE("Failed to allocate buffer for sector erase");
      return;
  }
  memset(erase_buffer, 0xFF, NOR_SECTOR_SIZE);
  fwrite(erase_buffer, 1, NOR_SECTOR_SIZE, nor_image_file);
  fflush(nor_image_file);
  vo_free(erase_buffer);
}

/**
 * @brief Reads from the emulated NOR flash memory.
 *
 * @param addr Start address to read from.
 * @param data Buffer to store read data.
 * @param len Number of bytes to read.
 * @return int32_t Number of bytes read, or -1 on error.
 */
int32_t nor_read_memory(uint32_t addr, void *data, uint32_t len) {
  if (!nor_image_file || (addr + len) > NOR_FLASH_SIZE) {
    return -1;
  }

  fseek(nor_image_file, addr, SEEK_SET);
  size_t read_bytes = fread(data, 1, len, nor_image_file);

  if (read_bytes != len) {
    LOGE("NOR Read Error: requested %u, got %zu", len, read_bytes);
    return -1;
  }

  return read_bytes;
}

/**
 * @brief Writes to the emulated NOR flash memory.
 *
 * @param addr Start address to write to.
 * @param data Buffer containing data to write.
 * @param len Number of bytes to write.
 * @return int32_t Number of bytes written, or -1 on error.
 */
int32_t nor_write_memory(uint32_t addr, void *data, uint32_t len) {
    if (!nor_image_file || (addr + len) > NOR_FLASH_SIZE) {
        return -1;
    }

    // Simple file write for emulation. A more accurate emulation would
    // read, AND the new data, and then write back.
    fseek(nor_image_file, addr, SEEK_SET);
    size_t written_bytes = fwrite(data, 1, len, nor_image_file);
    fflush(nor_image_file);

    if (written_bytes != len) {
        LOGE("NOR Write Error: requested %u, wrote %zu", len, written_bytes);
        return -1;
    }

    return written_bytes;
}

#endif // WITH_NOR_FLASH_EMU