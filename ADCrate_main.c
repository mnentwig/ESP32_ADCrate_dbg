#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_continuous.h"
#include "soc/soc_caps.h" // SOC_ADC_SAMPLE_FREQ_THRES_LOW; SOC_ADC_SAMPLE_FREQ_THRES_HIGH
#include "esp_timer.h"

#define MAXTRANSFER_BYTES 1024
#define NCHAN 1	
static uint8_t transferBuf[MAXTRANSFER_BYTES];
static adc_continuous_handle_t adcHandle;
static uint32_t nOverflow;
static uint32_t nCapt;
static uint64_t lastCaptBegin;
static uint64_t lastCaptEnd;
#define RATE_FUDGE_FACTOR 0.8181818f // as fraction: 3^2/11
static const char* TAG = "ADCrate";

static bool IRAM_ATTR cbConvDone(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data){
  nCapt += edata->size / sizeof(adc_digi_output_data_t);
  return 0;
}

static bool IRAM_ATTR cbOverflow(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data){
  ++nOverflow;
  return 0;
}

void app_main(){
  nCapt = 0;
  lastCaptBegin = 0;
  lastCaptEnd = 1;

  // === ADC handle ===
  adc_continuous_handle_cfg_t adc_config = {
    .max_store_buf_size = 4*1024,
    .conv_frame_size = MAXTRANSFER_BYTES,
  };
  ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adcHandle));

  // === set callbacks ===
  adc_continuous_evt_cbs_t cbs = {
    .on_conv_done = cbConvDone,
    .on_pool_ovf = cbOverflow
  };
  ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adcHandle, &cbs, /*user_data*/NULL));

  int resCount = 0;
  for (float convRate_Hz = (float)SOC_ADC_SAMPLE_FREQ_THRES_LOW; convRate_Hz < SOC_ADC_SAMPLE_FREQ_THRES_HIGH; convRate_Hz *= 1.2f){    
    // === digi ctrl config ===
    adc_continuous_config_t dig_cfg = {
      .sample_freq_hz = convRate_Hz,
	.conv_mode = ADC_CONV_SINGLE_UNIT_1,
	.format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
	.pattern_num = NCHAN
    };
    
    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    adc_pattern[0].atten = ADC_ATTEN_DB_6;
    adc_pattern[0].channel = ADC_CHANNEL_6 & 0x7;
    adc_pattern[0].unit = ADC_UNIT_1;
    adc_pattern[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    adc_pattern[1].atten = ADC_ATTEN_DB_6;
    adc_pattern[1].channel = ADC_CHANNEL_7 & 0x7;
    adc_pattern[1].unit = ADC_UNIT_1;
    adc_pattern[1].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(adcHandle, &dig_cfg));
    
    // === start conversion ===
    nOverflow = 0;
    nCapt = 0;
    ESP_ERROR_CHECK(adc_continuous_flush_pool(adcHandle));
    ESP_ERROR_CHECK(adc_continuous_start(adcHandle));
    lastCaptBegin = esp_timer_get_time();

    float tCapt_s = 1.0f;    
    uint32_t nSamples = (uint32_t)(tCapt_s*convRate_Hz+0.5);
    uint32_t nBytes = nSamples * 2; // 16 bit response
    
    // round data amount up to full transfer buffer for accurate time measurement
    nBytes += (uint32_t)(MAXTRANSFER_BYTES-1);
    nBytes &= ~(uint32_t)(MAXTRANSFER_BYTES-1);

    // === get data ===  
    while (nBytes){
      uint32_t nTryRead = nBytes < MAXTRANSFER_BYTES ? nBytes : MAXTRANSFER_BYTES;
      uint32_t nActualRead;
      esp_err_t ret = adc_continuous_read(adcHandle, transferBuf, nTryRead, &nActualRead, /*blocking*/ADC_MAX_DELAY);
      if (ret != ESP_OK) {
	ESP_LOGE(TAG, "adc_continuous_read");
	ESP_ERROR_CHECK(ESP_FAIL);
      }
      
      nBytes -= nActualRead;
      if (!nBytes) break;
    }
    lastCaptEnd = esp_timer_get_time();
    
    // === stop conversion ===
    ESP_ERROR_CHECK(adc_continuous_stop(adcHandle));
    
    float duration_s = (float)(lastCaptEnd - lastCaptBegin)*1e-6f;
    float rate_SPS = (float)nCapt / duration_s; // callback is triggered by full buffers => use multiple of buffer size, not set amount
    if (!resCount++)
      printf("rConf/Hz\trMeas_Hz\tratio\tnOvf\n");
    printf("%1.3f\t%1.3f\t%1.5f\t%lu\n", convRate_Hz, rate_SPS, rate_SPS/convRate_Hz, nOverflow);
  } // for convRate
  ESP_ERROR_CHECK(adc_continuous_stop(adcHandle));
  ESP_ERROR_CHECK(adc_continuous_deinit(adcHandle));

  while (1){
    vTaskDelay(1000/portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "zzz");	
  }
}
