#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "ssd1306.h"
#include "oledDisplay.h"
#include "driver/gpio.h"
#include "driver/dac_cosine.h"
#include "esp_vfs_fat.h"
#include "driver/spi_master.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "driver/gptimer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#include "hal/adc_types.h"


static const char *TAG = "TAG";

//OLED
#define I2C_SCL 22
#define I2C_SDA 21
#define I2C_FREQ 400000
#define I2C_PORT 0
#define I2C_ADDRESS_1 0x3C
#define I2C_ADDRESS_2 0x3D

static ssd1306_handle_t display1 = NULL;
static ssd1306_handle_t display2 = NULL;

uint8_t display1X = 0;
uint8_t display1Y = 0;
uint8_t display2X = 0;
uint8_t display2Y = 0;


//Menu and Menu Buttons
#define UP_BUTTON GPIO_NUM_26
#define DOWN_BUTTON GPIO_NUM_33
#define SELECT_BUTTON GPIO_NUM_32

volatile uint8_t menuIndex = 0; //0 is top, menuSize-1 is bottom
uint8_t menuSize = 1;
char currentMenu[10][100];
char menuHeader[100];
volatile bool menuChangeFlag = false;
volatile bool menuSelectFlag = false;

volatile uint64_t lastInterruptTime = 0;
uint64_t debounceDelay = 200;


//DAC Variables
#define DAC_CHANNEL DAC_CHAN_0 //GPIO 25

uint16_t lowFreq[1] = {1000};
uint16_t medFreq[3] = {100,500,1000};
uint16_t highFreq[7] ={10,50,100,500,1000,2000,5000};
uint16_t currentFreq = 1000;

dac_cosine_config_t dac_conf;
dac_cosine_handle_t dac_handle;


//SD Variables
#define MOSI GPIO_NUM_23
#define MISO GPIO_NUM_19
#define CLK GPIO_NUM_18
#define CS GPIO_NUM_5

#define MOUNT_POINT "/sdcard"

sdmmc_card_t *card = NULL;

//ADC Tester and Relay
#define ADC_SLOPE 1232
#define ADC_INTERCEPT -142

#define ADC_OVERVOLTAGE 1000

#define ADC_TEST ADC_CHANNEL_3 //gpio 39
#define RELAY GPIO_NUM_0

gptimer_handle_t ADCTimer = NULL;

int adcTestReading = 0;
volatile bool overVoltageFlag = false;
adc_oneshot_unit_handle_t ADCTest = NULL;

//ADC Continuous
#define ADC_1 ADC_CHANNEL_0 //gpio 36
#define ADC_2 ADC_CHANNEL_6 //gpio 34
#define ADC_RATE 20000

#define FRAME_SIZE 1024
#define POOL_SIZE 8192
#define BUFFER_SIZE 16384

adc_continuous_handle_t adc_handle = NULL;
static TaskHandle_t s_task_handle;

//File Variables
uint8_t fileBuffer[BUFFER_SIZE];
size_t bufferPos = 0;
FILE *currentFile = NULL;
uint16_t currentLogNumber = 0;



//OLED Initialization
static esp_err_t i2c_master_init(void){
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ,
    };
    esp_err_t err = i2c_param_config(I2C_PORT, &conf);
    if(err != ESP_OK)
        return err;
    return i2c_driver_install(I2C_PORT,I2C_MODE_MASTER,0,0,0);
}



//GPIO
void IRAM_ATTR downButtonInterrupt(){
  uint64_t currentTime = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
  if(currentTime - lastInterruptTime > debounceDelay){
    menuIndex = (menuIndex + 1) % menuSize;
    lastInterruptTime = currentTime;
    menuChangeFlag = true;
  }
}

void IRAM_ATTR upButtonInterrupt(){
  uint64_t currentTime = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
  if(currentTime - lastInterruptTime > debounceDelay){
    menuIndex = (menuIndex + menuSize - 1) % menuSize;
    lastInterruptTime = currentTime;
    menuChangeFlag = true;
  }
}

void IRAM_ATTR selectButtonInterrupt(){
  uint64_t currentTime = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
  if(currentTime - lastInterruptTime > debounceDelay){
    lastInterruptTime = currentTime;
    menuSelectFlag = true;
  }
}

void buttonsInitialization(){
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = ((1ULL << UP_BUTTON) | 
                            (1ULL << DOWN_BUTTON) |
                            (1ULL << SELECT_BUTTON)),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };

    gpio_config(&io_conf);

    gpio_install_isr_service(0);

    gpio_isr_handler_add(UP_BUTTON, upButtonInterrupt, NULL);
    gpio_isr_handler_add(DOWN_BUTTON, downButtonInterrupt, NULL);
    gpio_isr_handler_add(SELECT_BUTTON, selectButtonInterrupt, NULL);
}

void displayMenu(){
    ssd1306_clear_screen(display1,0);
    display1X = 0;
    display1Y = 0;
    ssd1306_printLineN(display1, &display1X, &display1Y, (const uint8_t *) menuHeader);
    display1Y += 4;
    for(int i = 0; i<menuSize; i++){
        if(i == menuIndex){
            ssd1306_printLine(display1, &display1X, &display1Y, (const uint8_t *) ">");
        }
        ssd1306_printLineN(display1, &display1X, &display1Y, (const uint8_t *) currentMenu[i]);
    }
    ESP_ERROR_CHECK(ssd1306_refresh_gram(display1));
}

uint8_t getMenuChoice(char *mHeader, uint8_t mSize, char cMenu[][100]){
  menuSelectFlag = false;
  menuSize = mSize;
  strcpy(menuHeader,mHeader);
  for(int i = 0; i<menuSize; i++){
    strcpy(currentMenu[i],cMenu[i]);
  }
  menuIndex = 0; 
  displayMenu();
  while(!menuSelectFlag){
    if(menuChangeFlag){
      displayMenu();
      menuChangeFlag = false;
    }
  }
  menuSelectFlag = false;
  return menuIndex;
}



//DAC
void dacInitialization(){
  dac_conf.chan_id = DAC_CHANNEL;
  dac_conf.freq_hz = 1000;
  dac_conf.clk_src = DAC_COSINE_CLK_SRC_DEFAULT;
  dac_conf.offset = 0;
  dac_conf.phase = DAC_COSINE_PHASE_0;
  dac_conf.atten = DAC_COSINE_ATTEN_DB_6; //0 -> 3.3/2
  dac_conf.flags.force_set_freq = true;
}

esp_err_t DACStart(){
  dac_conf.freq_hz = currentFreq;
  esp_err_t err = dac_cosine_new_channel(&dac_conf,&dac_handle);
  if(err != ESP_OK)
    return err;
  return dac_cosine_start(dac_handle);
}

esp_err_t DACStop(){
  esp_err_t err = dac_cosine_stop(dac_handle);
  if(err != ESP_OK)
    return err;
  return dac_cosine_del_channel(dac_handle);
}



//SD Card Initialization
void sdInitialization(){
  ESP_LOGI(TAG, "Starting SD initialization...");

  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.slot = SPI2_HOST;
  host.max_freq_khz = SDMMC_FREQ_DEFAULT; 

  spi_bus_config_t bus_conf = {
    .mosi_io_num = MOSI,
    .miso_io_num = MISO,
    .sclk_io_num = CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4000,
  };

  sdspi_device_config_t device_conf = SDSPI_DEVICE_CONFIG_DEFAULT();
  device_conf.gpio_cs = CS;
  device_conf.host_id = host.slot;
  
  esp_vfs_fat_sdmmc_mount_config_t mount_conf = {
    .max_files = 5,
    .allocation_unit_size = 16 * 1024,
    .format_if_mount_failed = false,
  };
  
  const char mountPoint[] = MOUNT_POINT;

  spi_bus_free(host.slot);
  esp_err_t ret = spi_bus_initialize(host.slot,&bus_conf,SPI_DMA_CH_AUTO);
  ESP_LOGI(TAG, "SPI init result: %s", esp_err_to_name(ret));

  ESP_LOGI(TAG, "Attempting to mount SD card...");

  esp_err_t err = esp_vfs_fat_sdspi_mount(mountPoint,&host,&device_conf,&mount_conf,&card);

  if (err == ESP_OK) {
      ESP_LOGI(TAG, "Card mounted. Name: %s", card->cid.name);
  } else {
      ESP_LOGE(TAG, "Mount failed: %s", esp_err_to_name(err));
  }
}

void findCurrentLogNumber(){
  DIR *dir = opendir(MOUNT_POINT);
  struct dirent *entry;

  while((entry = readdir(dir)) != NULL){
    if(strncmp(entry->d_name, "log", 3) == 0){
      uint16_t logNum;
      if(sscanf(entry->d_name,"log%hd_%*d.csv",&logNum) > 0){
        if(logNum > currentLogNumber)
          currentLogNumber = logNum;
      }
    }
  }

  closedir(dir);
}



//ADC Initialization and ADC Test Control
void adcTestRead(){
  adc_oneshot_read(ADCTest,ADC_TEST,&adcTestReading);
}

bool IRAM_ATTR checkRelay(gptimer_handle_t t, const gptimer_alarm_event_data_t *edata, void *user_ctx){
  adcTestRead();

  if(adcTestReading > ADC_OVERVOLTAGE){
    overVoltageFlag = true;
  }

  gpio_set_level(RELAY,!overVoltageFlag);

  return false;
}

void ADCTimerStart(){
  overVoltageFlag = false;
  gptimer_start(ADCTimer);
}

void ADCTimerStop(){
  gptimer_stop(ADCTimer);
}

volatile int isr_count = 0;

bool IRAM_ATTR adc_callback(adc_continuous_handle_t handle,
                              const adc_continuous_evt_data_t *edata,
                              void *user_data) {
  BaseType_t taskWoken = pdFALSE;
  isr_count++;
  vTaskNotifyGiveFromISR(s_task_handle, &taskWoken);
  return taskWoken == pdTRUE;
}

void ADCInitialization(){
  //ADC Test Timer
  gptimer_config_t timer_conf = {
    .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 1000000, //1 MHz
  };
  gptimer_new_timer(&timer_conf,&ADCTimer);

  gptimer_alarm_config_t alarm_conf = {
    .alarm_count = 100000, //100ms
    .reload_count = 0,
    .flags.auto_reload_on_alarm = true,
  };
  gptimer_set_alarm_action(ADCTimer,&alarm_conf);

  gptimer_event_callbacks_t callback = {
    .on_alarm = checkRelay,
  };
  gptimer_register_event_callbacks(ADCTimer,&callback,NULL);
  gptimer_enable(ADCTimer);

  //ADC Test
  adc_oneshot_unit_init_cfg_t oneshot_conf = {
    .unit_id = ADC_UNIT_1,
  };
  adc_oneshot_new_unit(&oneshot_conf, &ADCTest);

  adc_oneshot_chan_cfg_t chan_conf = {
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_12,
  };
  adc_oneshot_config_channel(ADCTest, ADC_TEST, &chan_conf);

  gpio_reset_pin(RELAY);
  gpio_set_direction(RELAY, GPIO_MODE_OUTPUT);
  gpio_set_level(RELAY, 0);

  //Continous ADC Initialization
  adc_continuous_handle_cfg_t continuous_conf = {
    .conv_frame_size = FRAME_SIZE,
    .max_store_buf_size = POOL_SIZE,
  };
  adc_continuous_new_handle(&continuous_conf, &adc_handle);

  adc_continuous_config_t channel_conf = {
    .sample_freq_hz = ADC_RATE,
    .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
  };

  static adc_digi_pattern_config_t adc_pattern[2] = {
    {
      .atten = ADC_ATTEN_DB_12,
      .channel = ADC_1,
      .unit = ADC_UNIT_1,
      .bit_width = ADC_BITWIDTH_12,
    },
    {
      .atten = ADC_ATTEN_DB_12,
      .channel = ADC_2,
      .unit = ADC_UNIT_1,
      .bit_width = ADC_BITWIDTH_12,
    },
  };
  channel_conf.adc_pattern = adc_pattern;
  channel_conf.pattern_num = 2;

  esp_err_t ret = adc_continuous_config(adc_handle, &channel_conf);
  if(ret != ESP_OK)
    ESP_LOGE(TAG,"Initilization: &d", ret);
}


//File Writing Functions
void flushBuffer(){
  if(bufferPos > 0 && currentFile != NULL){
    fwrite(fileBuffer,1,bufferPos,currentFile);
    bufferPos = 0;
  }
}

void writeToCSV(const char *data, size_t len){
  if(bufferPos + len >= BUFFER_SIZE){
    flushBuffer();
  }
  memcpy(fileBuffer + bufferPos,data,len);
  bufferPos += len;
}

void logADCContinuous(){

  char title[100] = {0};
  sprintf(title, "log%d_%d.csv",currentLogNumber,currentFreq);
  currentFile = fopen(title,"w");

  char *header = "ADC1,ADC2\n";
  writeToCSV(header,strlen(header));

  uint8_t result[FRAME_SIZE] = {0};
  uint32_t retNumber = 0;
  char lineBuffer[64];

  uint32_t samplesLogged = 0;
  uint32_t maxSamples = ADC_RATE * 1;

  s_task_handle = xTaskGetCurrentTaskHandle();
  adc_continuous_evt_cbs_t cbs = {
    .on_conv_done = adc_callback,
  };
  adc_continuous_register_event_callbacks(adc_handle,&cbs,NULL);
    
  esp_err_t ret = adc_continuous_start(adc_handle);
  if(ret != ESP_OK)
    ESP_LOGE(TAG,"Start: %d", ret);
  
  ESP_LOGI(TAG, "ADC started successfully");
  
  while(samplesLogged < maxSamples){
    if(isr_count % 10 == 0){
      ESP_LOGI(TAG, "isr count: %d", isr_count);
    }

    uint32_t notif = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));
    if(notif == 0){
      ESP_LOGW(TAG, "skipping: %d", samplesLogged);
      //continue;
    }

    ret = adc_continuous_read(adc_handle, result, FRAME_SIZE, &retNumber, 0);
    if(ret != ESP_OK)
      ESP_LOGE(TAG,"Read: %d", ret);

    uint32_t adc1_val = 0;
    uint32_t adc2_val = 0;
    bool adc1_next = true;
    
    if(ret == ESP_OK){
      uint32_t data;
      
      for(int i = 0; i < retNumber; i += SOC_ADC_DIGI_RESULT_BYTES){
        adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];

        data = p->type1.data;

        if(adc1_next){
          adc1_val = data;
          adc1_next = false;
        }else{
          adc2_val = data;
          adc1_next = true;

          size_t len = snprintf(lineBuffer,sizeof(lineBuffer),"%ld,%ld\n",adc1_val,adc2_val);
          writeToCSV(lineBuffer,len);

          samplesLogged++;
        }
      }
    }else{
      ESP_LOGE(TAG,"Read Error: %d", ret);
    }
    
  }
  flushBuffer();
  ret = adc_continuous_stop(adc_handle);
  if(ret != ESP_OK){
    ESP_LOGW(TAG, "ADC stop returned: %d", ret);
  }
  
  fclose(currentFile);
  ESP_LOGI(TAG, "ADC Done - Logged %ld samples", samplesLogged);
}

//Requires 
void fullLogData(){

}

void adc_log_task(void *pvParameters){
  logADCContinuous();
  vTaskDelete(NULL);
}


void app_main(void)
{
    //OLED Initialization
    ESP_ERROR_CHECK(i2c_master_init());
    display1 = ssd1306_create(I2C_PORT,I2C_ADDRESS_1);
    display2 = ssd1306_create(I2C_PORT,I2C_ADDRESS_2);

    if(display1 == NULL || display2 == NULL){
        ESP_LOGE(TAG,"OLED Failed");
        return;
    }
    ESP_ERROR_CHECK(ssd1306_refresh_gram(display1));
    ESP_ERROR_CHECK(ssd1306_refresh_gram(display2));

    ssd1306_clear_screen(display1,0);
    ssd1306_clear_screen(display2,0);
    ESP_LOGI(TAG, "Oled initialized");


    //Menu Initialization
    buttonsInitialization();
    ESP_LOGI(TAG, "Menu Buttons Initialized");

    
    //DAC Initialization
    dacInitialization();
    ESP_ERROR_CHECK(dac_cosine_new_channel(&dac_conf,&dac_handle));
    ESP_ERROR_CHECK(dac_cosine_del_channel(dac_handle));

    ESP_LOGI(TAG,"DAC Initialized");

    
    //SD Card Initialization
    sdInitialization();
    //findCurrentLogNumber();
    ESP_LOGI(TAG,"SD Initialized");    
    
    //ADC Initialization
    ADCInitialization();
    ESP_LOGI(TAG,"ADC Test Initialized");


    char getFrequencyMenu[3][100] = {"1 Frequency", "3 Frequencies", "7 Frequencies"};
    uint8_t numFreq = getMenuChoice("Choose Freq Count:", 3, getFrequencyMenu);

    //Initialize all the components

    //Display all the errors

    //Prompt For Battery

    //Check for Overvoltage - Add a bool that says switch can be open, Add 0 function

    //Ask for frequency number

    //Test and log for all frequencies

    //Save to SD card

    //Analyze data

    //Display Data

    //Prompt to display other data and retest

    //xTaskCreatePinnedToCore(adc_log_task, "adc_log", 8192, NULL, 7, NULL, 1);
    
    DACStart();
    vTaskDelay(pdMS_TO_TICKS(200000));
    DACStop();
}