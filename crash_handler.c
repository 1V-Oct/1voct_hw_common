#include "crash_handler.h"
#include "ff.h"
#include "mod.h"
#include "mod_user.h"
#include "sto_common.h"
#include "storage.h"
#include "uart_io.h"
#include "vo_chunk.h"
#include "vo_logger.h"
#include "vo_memory.h"
#include "vo_string.h"
#include "vo_system.h"

#define CRASH_ID MAKE_CHUNK_ID('C', 'R', 'X', 'X')
static const char *crash_dir = "crash_dump";

#define RAM_D2_PTR      0x30000000
#define CRX_LOG_BUF_PTR RAM_D2_PTR
#define CRX_DATA_PTR    (RAM_D2_PTR + 0x1000)

static char *log_buf = (char *)0;

void crx_crash_handler(uint32_t *sp) {
  /* These are volatile to try and prevent the compiler/linker optimising them
away as the variables never actually get used.  If the debugger won't show the
values of the variables, make them global my moving their declaration outside
of this function. */
  volatile uint32_t lr;  /* Link register. */
  volatile uint32_t pc;  /* Program counter. */
  volatile uint32_t psr; /* Program status register. */

  uint32_t table[32];

  table[0]  = sp[0];
  table[1]  = sp[1];
  table[2]  = sp[2];
  table[3]  = sp[3];
  table[12] = sp[12];

  lr        = sp[5];
  pc        = sp[6];
  psr       = sp[7];

  asm("str r5, [%0, 20]"
      "\n\t"
      "str r6, [%0, 24]"
      "\n\t"
      "str r4, [%0, 16]"
      "\n\t"
      "str r7, [%0, 28]"
      "\n\t"
      "str r8, [%0, 32]"
      "\n\t"
      "str r9, [%0, 36]"
      "\n\t"
      "str r10, [%0, 40]"
      "\n\t"
      "str r11, [%0, 44]"
      "\n\t"
      :
      : "r"(&table[0]), "r"(&table[6])
      : "memory");

  //\033c\033[0;0f
  uart_printf("The Centre (C) 1V/Oct Ltd. %d build %d\n", (uint32_t)&__BUILD_DATE, (uint32_t)&__BUILD_NUMBER);

  uart_printf("PC = %08x       LR = %08x       PSR = %08x\n", pc, lr, psr);
  int i, j;
  for (i = 0; i < 4; i++) {
    for (j = 0; j < 3; j++) {
      uart_printf("r%02d = %08x     ", i + j * 4, table[i + j * 4]);
    }
    uart_printf("\n");
  }
  uart_printf("CCR: %08x SHCR: %08x\n", *((uint32_t *)0xe000ed14), *((uint32_t *)0xe000ed24));
  uart_printf("HFSR %08x\n", *((uint32_t *)0xe000ed2c));
  uart_printf("MMFSR %02x\n", *((uint8_t *)0xe000ed28));
  uart_printf("MMFAR %08x\n", *((uint32_t *)0xe000ed34));
  uart_printf("BFSR %02x\n", *((uint8_t *)0xe000ed29));
  uart_printf("BFAR %08x\n", *((uint32_t *)0xe000ed38));
  uart_printf("UFSR %02x\n", *((uint8_t *)0xe000ed2a));
  uart_printf("AFSR %08x\n", *((uint32_t *)0xe000ed3c));

  uint32_t *p = (uint32_t *)CRX_DATA_PTR;
  *p++        = 0;

  *p++        = 0;
  *p++        = (uint32_t)&__BUILD_NUMBER;

  module_t *m = g_patch[0].mod;
  while (m) {
    p = vo_memcpy(p, m->cs, sizeof(mod_static_t));
    p = vo_memcpy(p, m, sizeof(module_t));
    p = vo_memcpy(p, "INP:", 4);
    if (m->inp)
      p = vo_memcpy(p, m->inp, m->cs->num_inputs * sizeof(input_quad_t));
    p = vo_memcpy(p, "INV:", 4);
    if (m->inp_val)
      p = vo_memcpy(p, m->inp_val, m->cs->num_inputs * sizeof(val_quad_f_t));
    p = vo_memcpy(p, "OUT:", 4);
    if (m->out)
      p = vo_memcpy(p, m->out, m->num_out * sizeof(out_mix_t));
    p = vo_memcpy(p, "SET:", 4);
    if (m->settings)
      p = vo_memcpy(p, m->settings, m->cs->num_settings);
    m = m->next;
  }
  uint32_t data_size = (uint32_t)p - CRX_DATA_PTR;

  p                  = (uint32_t *)CRX_DATA_PTR;
  *p++               = CRASH_ID;
  *p++               = data_size;

  SCB_CleanDCache();

#if (DEBUG_BUILD == 1)
  __ASM volatile("BKPT #01");
  // while(1) ;
#endif
  vo_reset_system();
}

void crx_enable_log(void) {
  log_buf = (char *)CRX_LOG_BUF_PTR;
}

/**
 * @brief Check if Crash Dump available
 *
 * @return uint8_t Returns 1 if crashed last time
 */
uint8_t crx_has_crashed(void) {
  uint32_t *p = (uint32_t *)CRX_DATA_PTR;
  // LOGE("%08x %08x", p, *p);
  if (*p == CRASH_ID) {
    return 1;
  }
  return 0;
}

void crx_log(char c) {
  if (log_buf) {
    *log_buf++ = c;
    log_buf    = (char *)(((uint32_t)log_buf) & 0xffff0fff);
    // uart_tx(UART_PUTS_PORT_ID, '#');
  }
}

void crx_save_crash_dump(void) {
  uint32_t *p               = (uint32_t *)0x30000000;
  *(uint32_t *)CRX_DATA_PTR = 0;
  SCB_CleanDCache();

  LOGE("Saving Crash Dump");
  HAL_Delay(1500);
  if (sto_sd_status != STO_CARD_MOUNTED) {
    LOGE("No SD!");
    return;
  }
  FILINFO fn;
  FRESULT res;

  sto_chmkdir(crash_dir);

  char filename[30];
  int  i = 0;
  do {
    i++;
    vo_sprintf(filename, "crash_dump-%04x.bin", i);
    res = f_stat(filename, &fn);
  } while (res == FR_OK);

  FIL   fp;
  UINT  br;
  char *buf = vo_malloc(4096);
  vo_memcpy(buf, p, 4096);
  res = f_open(&fp, filename, FA_WRITE | FA_CREATE_ALWAYS);
  f_write(&fp, p, 0x1000, &br);
  p = (uint32_t *)CRX_DATA_PTR;
  f_write(&fp, p, p[1], &br);
  f_close(&fp);
  vo_free(buf);
  HAL_Delay(1500);
}

void HardFaultHandler(void) __attribute__((naked, aligned(8)));

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {
  // .align 8 is necessary for all handlers, can be done at function declaration with __attribute
  __asm volatile(
    " .align 8 \n"
    " tst lr, #4                                                \n"
    " ite eq                                                    \n"
    " mrseq r0, msp                                             \n"
    " mrsne r0, psp                                             \n"
    " ldr r1, [r0, #24]                                         \n"
    " ldr r2, handler2_address_const                            \n"
    " bx r2                                                     \n"
    " handler2_address_const: .word crx_crash_handler           \n");
  /* USER CODE BEGIN HardFault_IRQn 0 */
  // con_printf("HardFault_Handler");
  /* USER CODE END HardFault_IRQn 0 */
  while (1) {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}
