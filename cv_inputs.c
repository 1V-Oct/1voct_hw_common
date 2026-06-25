#include "cv_inputs.h"
#include "adc_cv.h"
#include "pots.h"
#include "vo_string.h"
#include "vo_logger.h"
 
void cv_init(void) {
  // to which module the cv belongs
  // LOGI("CV INIT");
  pots_init();
  adc_cv_init();
  adc_dma_done = 1;
}


void cv_scan(void) {
  // LOGI("CV SCAN");;
  if (adc_dma_done == 0) return;
  adc_dma_done = 0;
  adc_cv_scan();
  // while(!adc_dma_done) {}
  // adc_dma_done = 0;
  adc_pots_scan(); 
}
