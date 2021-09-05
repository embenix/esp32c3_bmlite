#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/cdefs.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_attr.h"
#include "led_strip.h"
#include "driver/rmt.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"


#include "bmlite_if.h"
#include "hcp_tiny.h"
#include "platform.h"
#include "bmlite_hal.h"

static const char *TAG = "fpc_bmlite";
#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define EXAMPLE_CHASE_SPEED_MS (100)

/**********************
 *  STATIC PROTOTYPES
 **********************/
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b);
void task_led_indicator(void * pvParameters);
void task_fpc_bmlite(void * pvParameters);

// void bmlite_on_error(bmlite_error_t error, int32_t value);
// // void bmlite_on_start_capture();
// // void bmlite_on_finish_capture();
// void bmlite_on_start_enroll();
// void bmlite_on_finish_enroll();
// void bmlite_on_start_enrollcapture();
// void bmlite_on_finish_enrollcapture(uint8_t capCount);
// void bmlite_on_identify_start();
// void bmlite_on_identify_finish();


#define FPC_HOST    SPI2_HOST
#define DMA_CHAN    SPI_DMA_CH_AUTO  //FPC_HOST
spi_device_handle_t spi;
#define DATA_BUFFER_SIZE (1024*5)
static uint8_t hcp_txrx_buffer[MTU];
static uint8_t hcp_data_buffer[DATA_BUFFER_SIZE];
static char version[100];
static uint16_t template_id;
static uint32_t current_id = 10;
static bool match;
static fpc_bep_result_t res;
static HCP_comm_t hcp_chain;
static bool ledState = false;

static SemaphoreHandle_t BMLITE_IS_BUSY;

struct RGB_Message
{
 uint32_t ucMessageID;
 uint32_t ucRGBdata[3];
 uint32_t ucDelayRate;  //ms | value must be minimum 10ms
} xMsg_RGB;

TaskHandle_t xHandle_led = NULL;
TaskHandle_t xHandle_bmlite = NULL;
QueueHandle_t xQueue_RGB_data, xQueue2;

void app_main(void)
{

    struct RGB_Message *pxMessage;
    xQueue_RGB_data = xQueueCreate( 10, sizeof( struct RGB_Message * ) );
    // uint32_t red = 0;
    // uint32_t green = 0;
    // uint32_t blue = 0;
    // uint16_t hue = 0;
    // uint16_t start_rgb = 0;
    BMLITE_IS_BUSY = xSemaphoreCreateBinary();
    //xTaskCreatePinnedToCore(task_led_indicator, "TASK_LED_INDICATOR", 4096, NULL, tskIDLE_PRIORITY, NULL, 0);
    xTaskCreate(task_led_indicator, "TASK_LED_INDICATOR", 1024*2, NULL, 2, &xHandle_led);
    xTaskCreate(task_fpc_bmlite, "TASK_LED_INDICATOR", 4096, NULL, tskIDLE_PRIORITY, &xHandle_bmlite);

    //Lock the Resource
    //xSemaphoreGive(BMLITE_IS_BUSY);

    while (true) {
        //gpio_set_level(CONFIG_FPC_CS_GPIO, 0);
        //ESP_LOGI(TAG, "-> BM-Lite IRQ signal: %u", gpio_get_level(CONFIG_FPC_IRQ_GPIO));
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}


/**
 * @brief FPC BM-Lite task to handle the communication between
 *  ESP32 module and thte Sensor module
 */
void task_fpc_bmlite(void * pvParameters)
{
    struct RGB_Message *pxMessage;

    //Initialize the BM-Lite IO pins
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = ((1ULL << CONFIG_FPC_CS_GPIO) | (1ULL << CONFIG_FPC_RSTN_GPIO)); //GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;    
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE; //GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = ((1ULL << CONFIG_FPC_IRQ_GPIO)); //GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE; //GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = ((1ULL << 9)); //GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    esp_err_t ret;
    
    spi_bus_config_t buscfg={
        .miso_io_num=CONFIG_FPC_MISO_GPIO,
        .mosi_io_num=CONFIG_FPC_MOSI_GPIO,
        .sclk_io_num=CONFIG_FPC_SCK_GPIO,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=1000,
    };

    spi_device_interface_config_t devcfg={
        .clock_speed_hz=10*1000*1000,           //Clock out at 16 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=-1, //CONFIG_FPC_CS_GPIO,       //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        //.pre_cb=NULL,    //lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(FPC_HOST, &buscfg, DMA_CHAN);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(FPC_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    
    // //Initialize the LCD
    // lcd_init(spi);
    // //Initialize the effect displayed
    // ret=pretty_effect_init();
    // ESP_ERROR_CHECK(ret);

    // //Go do nice stuff.
    // display_pretty_colors(spi);

     //Initialize the FPC BM-Lite Module components
    hcp_chain.read = platform_bmlite_spi_receive;
    hcp_chain.write = platform_bmlite_spi_send;
    hcp_chain.pkt_buffer = hcp_data_buffer;
    hcp_chain.txrx_buffer = hcp_txrx_buffer;
    hcp_chain.pkt_size = 0;
    hcp_chain.pkt_size_max = sizeof(hcp_data_buffer);
    hcp_chain.phy_rx_timeout = 2000;

    platform_init(NULL);
    // These two lines for debug purpose only 
    memset(version, 0, 100);
    res = bep_version(&hcp_chain, version, 99);
    if (res == FPC_BEP_RESULT_OK){
        ESP_LOGI(TAG, "-> FPC - Version Read OK!");
        ESP_LOGI(TAG, "-> FPC - Version: %s", version); 
        //Serial.println(version);
        //u8g2log.println(version);
        //u8g2log.println("> FPC BM-Lite .... OK");
    }

    //Print just to get some notification
    ESP_LOGI(TAG, "-> BM-Lite method running.");

    //res = bep_template_remove_all(&hcp_chain);
    while(1){
        //Lock the Resource
        //xSemaphoreTake(BMLITE_IS_BUSY, portMAX_DELAY);
        
        res = -1;
        template_id = -1;

        res = sensor_wait_finger_not_present(&hcp_chain, 0);
        if (res == FPC_BEP_RESULT_OK){
            ESP_LOGI(TAG, "-> BM-Lite - Finger UP.");
            if (xQueue_RGB_data != 0){
                xMsg_RGB.ucRGBdata[0] = 0;
                xMsg_RGB.ucRGBdata[1] = 0;
                xMsg_RGB.ucRGBdata[2] = 5;
                xMsg_RGB.ucDelayRate = 250; //( rand() % (240)) + 10;
                pxMessage = & xMsg_RGB;
                //xQueueGenericSend( xQueue_RGB_data, ( void * ) &pxMessage, ( TickType_t ) 0, queueSEND_TO_BACK );
                xQueueSend( xQueue_RGB_data, ( void * ) &pxMessage, ( TickType_t ) 2 );
            }
        }

        //vTaskDelay(pdMS_TO_TICKS(100));

        //Register the finger on the location 0
        if (gpio_get_level(9) == 0){
            if (res == FPC_BEP_RESULT_OK){
                ESP_LOGI(TAG, "-> BM-Lite - Finger UP.");
                if (xQueue_RGB_data != 0){
                    xMsg_RGB.ucRGBdata[0] = 0;
                    xMsg_RGB.ucRGBdata[1] = 0;
                    xMsg_RGB.ucRGBdata[2] = 5;
                    xMsg_RGB.ucDelayRate = 50; //( rand() % (240)) + 10;
                    pxMessage = & xMsg_RGB;
                    //xQueueGenericSend( xQueue_RGB_data, ( void * ) &pxMessage, ( TickType_t ) 0, queueSEND_TO_BACK );
                    xQueueSend( xQueue_RGB_data, ( void * ) &pxMessage, ( TickType_t ) 2 );
                }
            }

            vTaskDelay(pdMS_TO_TICKS(2000));
            res = bep_template_remove(&hcp_chain, 5);
            res = bep_enroll_finger(&hcp_chain);
            ESP_LOGW(TAG, "-> BM-Lite - Enroll Finger Res: %i.", res);
            res = bep_template_save(&hcp_chain, 5);
            ESP_LOGW(TAG, "-> BM-Lite - Save Template Res: %i.", res);
            res = sensor_wait_finger_not_present(&hcp_chain, 0);
            
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        //hal_bmlite_get_status();
        res = sensor_wait_finger_present(&hcp_chain, 0);
        ESP_LOGW(TAG, "-> BM-Lite - Finger Wait Res: %i.", res);

        if (res == FPC_BEP_RESULT_TIMEOUT || res == FPC_BEP_RESULT_IO_ERROR) {
                platform_bmlite_reset();
                continue;
        } else if (res != FPC_BEP_RESULT_OK) {
                continue;
        }
        
        if ((res == FPC_BEP_RESULT_OK)){
            ESP_LOGI(TAG, "-> BM-Lite - Finger DOWN.");
            //vTaskDelay(pdMS_TO_TICKS(10));
            res = bep_identify_finger(&hcp_chain, 500, &template_id, &match);
            ESP_LOGW(TAG, "-> BM-Lite - Finger Wait Res: %i.", res);
            if (res == FPC_BEP_RESULT_OK){
                if (match == true){
                    ESP_LOGI(TAG, "-> BM-Lite - Match Found. Template ID: %i", template_id);
                    xMsg_RGB.ucRGBdata[0] = 0;
                    xMsg_RGB.ucRGBdata[1] = 10;
                    xMsg_RGB.ucRGBdata[2] = 0;
                }
                else {
                    ESP_LOGI(TAG, "-> BM-Lite - No Match Found. Template ID: %i", template_id);
                    xMsg_RGB.ucRGBdata[0] = 10;
                    xMsg_RGB.ucRGBdata[1] = 0;
                    xMsg_RGB.ucRGBdata[2] = 0;
                }

                if (xQueue_RGB_data != 0){
                    xMsg_RGB.ucDelayRate = 0; //( rand() % (240)) + 10;
                    pxMessage = & xMsg_RGB;
                    //xQueueGenericSend( xQueue_RGB_data, ( void * ) &pxMessage, ( TickType_t ) 0, queueSEND_TO_BACK );
                    xQueueSend( xQueue_RGB_data, ( void * ) &pxMessage, ( TickType_t ) 2 );
                }
            }
        }

        //Lock the Resource
        //xSemaphoreGive(BMLITE_IS_BUSY);
        // else {
        //     ESP_LOGI(TAG, "-> BM-Lite Finger DOWN.");
        //     if (xQueue_RGB_data != 0){
        //         xMsg_RGB.ucRGBdata[0] = 0;
        //         xMsg_RGB.ucRGBdata[1] = 10;
        //         xMsg_RGB.ucRGBdata[2] = 0;
        //         xMsg_RGB.ucDelayRate = 0; //( rand() % (240)) + 10;
        //         pxMessage = & xMsg_RGB;
        //         //xQueueGenericSend( xQueue_RGB_data, ( void * ) &pxMessage, ( TickType_t ) 0, queueSEND_TO_BACK );
        //         xQueueSend( xQueue_RGB_data, ( void * ) &pxMessage, ( TickType_t ) 2 );
        //     }
        // }

        //vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief RGB LED tasks to show the specific color
 *
 */
void task_led_indicator(void * pvParameters)
{
    uint32_t delayRate = 0;
    uint32_t r = 0, g = 0, b = 0;
    
    struct RGB_Message *pxRxedMessage;
    
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(CONFIG_RMT_TX_GPIO, RMT_TX_CHANNEL);
    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(CONFIG_STRIP_LED_NUMBER, (led_strip_dev_t)config.channel);
    led_strip_t *strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }
    else {
        // Show simple rainbow chasing pattern
        ESP_LOGI(TAG, "LED Rainbow Chase Start");
    }
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));

    while (strip)
    {
        //Lock the Resource
        //xSemaphoreTake(BMLITE_IS_BUSY, portMAX_DELAY);

        //Check if the RGB data is received in the queue
        if( xQueueReceive( xQueue_RGB_data, &( pxRxedMessage ), ( TickType_t ) 10 ) )
        {
            ESP_LOGI(TAG, "new RGB data: R=%u G=%u B=%u | Delay=%ums",
                pxRxedMessage->ucRGBdata[0], pxRxedMessage->ucRGBdata[1],
                pxRxedMessage->ucRGBdata[2], pxRxedMessage->ucDelayRate);

            r = pxRxedMessage->ucRGBdata[0];
            g = pxRxedMessage->ucRGBdata[1];
            b = pxRxedMessage->ucRGBdata[2];
            delayRate = pxRxedMessage->ucDelayRate;

            //Set the single LED RGB value.
            ESP_ERROR_CHECK(strip->set_pixel(strip, 0, r, g, b));
            // Flush RGB values to LEDs
            ESP_ERROR_CHECK(strip->refresh(strip, 100));
        }

        //Lock the Resource
        //xSemaphoreGive(BMLITE_IS_BUSY);

        if (delayRate >= 10){
            vTaskDelay(pdMS_TO_TICKS(delayRate));
            strip->clear(strip, 50);

            vTaskDelay(pdMS_TO_TICKS(delayRate));
            //Set the single LED RGB value.
            ESP_ERROR_CHECK(strip->set_pixel(strip, 0, r, g, b));
            // Flush RGB values to LEDs
            ESP_ERROR_CHECK(strip->refresh(strip, 100));
        }
        else {
            //Idle delay just to waste some ticks
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        
        // r = ( rand() % (50)) + 1;
        // ESP_ERROR_CHECK(strip->set_pixel(strip, 0, r, g, b));
        // // Flush RGB values to LEDs
        // ESP_ERROR_CHECK(strip->refresh(strip, 100));
        // vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));

        // g = ( rand() % (50)) + 1;
        // ESP_ERROR_CHECK(strip->set_pixel(strip, 0, r, g, b));
        // // Flush RGB values to LEDs
        // ESP_ERROR_CHECK(strip->refresh(strip, 100));
        // vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));

        // b = ( rand() % (50)) + 1;
        // ESP_ERROR_CHECK(strip->set_pixel(strip, 0, r, g, b));
        // // Flush RGB values to LEDs
        // ESP_ERROR_CHECK(strip->refresh(strip, 100));
        // vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));

    }

    ESP_LOGE(TAG, "-> task_led_indicator() deleted!");
    vTaskDelete(xHandle_led);
    
}

/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}


/***********************************************************
 * BM-Lite HAL Methods Implementation
 * 
 * 
 * 
 * *********************************************************/

/*
 * @brief Control BM-Lite Reset pin
 * @param[in] True  - Activate RESET
 *            False - Deactivate RESET
 */
void hal_bmlite_reset(bool state)
{
    gpio_set_level(CONFIG_FPC_RSTN_GPIO, state);
    return;
}

/*
 * @brief SPI write-read
 * @param[in] Write buffer
 * @param[in] Read buffer
 * @param[in] Size
 * @param[in] Leave CS asserted
 * @return ::fpc_bep_result_t
 */
fpc_bep_result_t hal_bmlite_spi_write_read(uint8_t *write, uint8_t *read, size_t size, bool leave_cs_asserted)
{
    esp_err_t ret;
    spi_transaction_t t;

    if (size==0) return 0;          //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    
    gpio_set_level(CONFIG_FPC_CS_GPIO, 0);

    t.length=size*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=write;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    //t.tx_data = write;
    t.rx_buffer=read;
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.

    //ESP_LOGW(TAG, "-> SPI return Code: %i", ret);

    if (!leave_cs_asserted){
        gpio_set_level(CONFIG_FPC_CS_GPIO, 1);
    }

    return 0;
}


/*
 * @brief Check if BM-Lite IRQ pin is set
 * @return ::bool
 */
bool hal_bmlite_get_status(void)
{
    //int irq_status = gpio_get_level(CONFIG_FPC_IRQ_GPIO);
    //ESP_LOGI(TAG, "-> FPC - BM-Lite get IRQ status | IRQ: %i", irq_status);
    return gpio_get_level(CONFIG_FPC_IRQ_GPIO); //irq_status;
}

/**
 * @brief Reads the system tick counter.
 *
 * @return Tick count since hal_timebase_init() call. [ms]
 */
hal_tick_t hal_timebase_get_tick(void)
{
    int tickCount = pdTICKS_TO_MS(xTaskGetTickCount());
    //ESP_LOGI(TAG, "-> FPC - Timebase get tickcount. %i", tickCount);
    return tickCount; //xTaskGetTickCount();
}

/**
 * @brief Busy wait.
 *
 * @param[in] ms  Time to wait [ms].
 * 0 => return immediately
 * 1 => wait at least 1ms etc.
 */
void hal_timebase_busy_wait(uint32_t ms)
{
    //ESP_LOGI(TAG, "-> FPC - Timebase busy wait. | Delay: %u", ms);
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/**
 *  Optional functions for Buttons & Leds control
 */

/**
 * @brief Get button press time (msec)
 * 
 * @return ::uint32_t
 */
uint32_t hal_get_button_press_time(void){
    //ESP_LOGI(TAG, "-> FPC - Get button press time.");
    return 0;
}

/**
 * @brief Check if button was pressed and released.
 *
 * @return Button press time in milli seconds.
 */
uint32_t hal_check_button_pressed(void)
{
    //ESP_LOGI(TAG, "-> FPC - Check button pressed.");
    return 0;
}

/**
 * @brief Set LED(s) status
 * @param[in] Status
 * @param[in] Status modifier
 */
void hal_set_leds(platform_led_status_t status, uint16_t mode)
{
    ESP_LOGI(TAG, "-> FPC - Set LED with Status:%u | Mode:%u", status, mode);
}



//---------------------------------------------------------------------
//Other helping functions.
//---------------------------------------------------------------------

//BM-Lite Helping Functions
void bmlite_on_start_enroll()
{
    ESP_LOGI(TAG, "-> FPC - Enroll Started.");
    // if (Serial) Serial.println(">> Start Enroll!");
    // u8g2log.print("\f");  //Clear the Screen log
    // u8g2log.println("> START ENROLLING,");
    // delay(100);
}

/**
 * Callback when the enrolling is done.
 */
void bmlite_on_finish_enroll(void)
{
    ESP_LOGI(TAG, "-> FPC - Enroll Finished.");

    // u8g2log.println("> ENROLL FINISH .. OK");
    // //u8g2log.println("> FPC BM-Lite ____ OK");
    // // set the effect to play
    // haptic.setWaveform(0, 76);  // play effect 
    // haptic.setWaveform(1, 0);       // end waveform
    // // play the effect!
    // haptic.go();

    // //Notify the Base unit
    // if (Bluefruit.connected()){
    //       bleuart.printf("#03010100\r\n");
    // }
    // delay(100);
}

/**
 * Callback invoked when Enroll capture has started
 */
void bmlite_on_start_enrollcapture(void)
{
    ESP_LOGI(TAG, "-> FPC - Enrollcapture Started.");
    // if (Serial) Serial.println(">> Start Enroll Capture!");
    // u8g2log.println("> PLACE YOUR FINGER..");
    // // set the effect to play
    // haptic.setWaveform(0, 42);  // play effect 
    // haptic.setWaveform(1, 0);       // end waveform
    // // play the effect!
    // haptic.go();
    // delay(100);
}

/**
 * Callback invoked when a connection is dropped
 * @param capCount gives the number of captures completed! Usually it takes 3 captures the most.
 */
void bmlite_on_finish_enrollcapture()
{
    ESP_LOGI(TAG, "-> FPC - Enrollcapture Finished.");
    // Serial.println(">> Finish Enroll Capture!");

    // u8g2log.print("\f");  //Clear the Screen log
    // u8g2log.printf("> CAPTURE %d   .... OK\n\r", (capCount+1));
    
    // // set the effect to play
    // haptic.setWaveform(0, 52);  // play effect 
    // haptic.setWaveform(1, 0);       // end waveform
    //   // play the effect!
    // haptic.go();
    // delay(100);
}

/**
 * Callback invoked when the Finger identification has started.
 */
void bmlite_on_identify_start(void)
{
    ESP_LOGI(TAG, "-> FPC - Identification Started.");
    // if (Serial) Serial.println(">> Start Identify!");
    
    // u8g2log.print("\f");  //Clear the Screen log
    // u8g2log.println("> IDENTIFY,");
    // u8g2log.println("> PLACE YOUR FINGER..");
    // delay(100);
}

/**
 * Callback invoked when the finger identification process has finished
 */
void bmlite_on_identify_finish(void)
{
    ESP_LOGI(TAG, "-> FPC - Identification Finished.");
    // if (Serial) Serial.println(">> Finished Identify!");
        
    // //u8g2log.print("\f");  //Clear the Screen log
    // u8g2log.println("> IDENTIFY FINISH!");
    // delay(100);
}

/**
 * Callback invoked when an error has encountered during the finger reading
 * @param error shows the error code related to the commands which can be found in the 'bmlite_if_callbacks.h'
 * @param value is a descriptive reason code which is defined in 'fpc_bep_types.h'
 */
void bmlite_on_error(bmlite_error_t error, int32_t value)
{
    ESP_LOGE(TAG, "-> FPC - Error No. %i | Code: %i", error, value);
    // if (value != FPC_BEP_RESULT_TIMEOUT) {
    //     //hal_set_leds(BMLITE_LED_STATUS_ERROR, false);
    //     //u8g2log.printf("> FPC ERROR: %d | %d\n\r", error, value);
    // } else {
    //     // Timeout - not really an error here
    //     //hal_set_leds(BMLITE_LED_STATUS_ERROR, true);
    //     //u8g2log.println("> FPC TIMEOUT ERROR!");
    // }
}