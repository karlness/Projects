#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "./ADXL343.h"
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_log.h"
#include "soc/uart_struct.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "driver/gpio.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "freertos/FreeRTOS.h"
//for the wifi
#define UDP_SERVER_IP   "192.168.1.10" // Replace it with your server's IP address
#define UDP_SERVER_PORT 3334           
#define BLINK_GPIO GPIO_NUM_13
#define SAMPLE_NUM 64
#define DEFAULT_VREF 1100
#define DELAY 2000           // Sensor read delays (i.e. how long between readings)
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_0;

#define MASTER_SCL_IO                22
#define MASTER_SDA_IO                23
#define I2C_MASTER_NUM               I2C_NUM_0
#define I2C_MASTER_FREQ_HZ           100000
#define WRITE_BIT                    I2C_MASTER_WRITE
#define READ_BIT                     I2C_MASTER_READ
#define ACK_CHECK                    true
#define NACK_CHECK                   false
#define ACK_VALUE                    0x00
#define NACK_VALUE                   0xFF
#define SLAVE_ADDRESS                ADXL343_ADDRESS
// Define constants for LED Controller
#define LEDC_HS_TIMER         LEDC_TIMER_0
#define LEDC_HS_MODE          LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO      (13)
#define LEDC_HS_CH0_CHANNEL   LEDC_CHANNEL_0
#define LEDC_LS_TIMER         LEDC_TIMER_1
#define LEDC_LS_MODE          LEDC_LOW_SPEED_MODE


/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      "FreshTomato" //replace it with your ssid
#define EXAMPLE_ESP_WIFI_PASS      "Smart25$"    //replace it with your wifi password
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";

static int s_retry_num = 0;

static int blink_rate_ms = 500; // Default to 500ms

// Task handle for the blink task
static TaskHandle_t blink_task_handle = NULL;

// Function prototype
static void udp_client_send(void);


static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        udp_client_send(); //send a UDP packet after connection

    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}


//ignore these function
void init_blink_gpio() {
     esp_rom_gpio_pad_select_gpio(BLINK_GPIO);
    // Set the GPIO as a push/pull output
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

void blink_led(int interval_ms) {
    // Configure the pin as an output
    esp_rom_gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while(true) {
        gpio_set_level(BLINK_GPIO, 1); // LED on
        vTaskDelay(interval_ms / portTICK_PERIOD_MS);

        gpio_set_level(BLINK_GPIO, 0); // LED off
        vTaskDelay(interval_ms / portTICK_PERIOD_MS);
    }
}


// Function to blink LED at a specified interval
void blink_task(void *pvParameter) {
    esp_rom_gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        // Check the global blink_rate_ms
        gpio_set_level(BLINK_GPIO, 1); // LED on
        vTaskDelay(blink_rate_ms / portTICK_PERIOD_MS);

        gpio_set_level(BLINK_GPIO, 0); // LED off
        vTaskDelay(blink_rate_ms / portTICK_PERIOD_MS);
    }
}


static void udp_client_send(void) {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
        return;
    }
    ESP_LOGI(TAG, "Socket created");

    // Configure the destination address
    struct sockaddr_in dest_addr = {
        .sin_addr.s_addr = inet_addr(UDP_SERVER_IP),
        .sin_family = AF_INET,
        .sin_port = htons(UDP_SERVER_PORT),
    };

    // Send a message to the server
    char payload[] = "request_blink_rate"; // Changed to the payload that requests the blink rate
    int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        close(sock); // Close the socket if sendind is fails
        return;
    }
    ESP_LOGI(TAG, "Message sent");

    // Receive response from the server
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);
    char recv_buf[128];
    // Wait for the response (addind a timeout here if needed)
    int len = recvfrom(sock, recv_buf, sizeof(recv_buf) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

    if (len > 0) {
        recv_buf[len] = 0; // Null-terminate the received data
        ESP_LOGI(TAG, "Received response: %s", recv_buf);
        
        int new_blink_rate;
        // parse the received data to extract the blink rate
        if (sscanf(recv_buf, "blink_rate:%d", &new_blink_rate) == 1) {
            ESP_LOGI(TAG, "New blink rate received: %d ms", new_blink_rate);
            blink_rate_ms = new_blink_rate;
            //Use new_blink_rate to adjust the blink pattern
            if (blink_task_handle == NULL) {
        xTaskCreate(blink_task, "Blink Task", 1024, NULL, 5, &blink_task_handle);
           }
        } else {
            ESP_LOGE(TAG, "Failed to parse blink rate");
        }
    } else if (len < 0) {
        ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
    }

    close(sock); // Close the socket now that we're done
    ESP_LOGI(TAG, "Socket closed");
}


// Function prototypes
void initialize_uart();
void initialize_ledc();
void adjust_led_intensity(int duty);
void cycle_led_intensity();


// Initialize UART for communication
void initialize_uart() {
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    esp_vfs_dev_uart_use_driver(UART_NUM_0);
}

// Adjust LED intensity based on duty value
void adjust_led_intensity(int duty) {
    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_HS_CH0_CHANNEL,
        .duty = duty,
        .gpio_num = LEDC_HS_CH0_GPIO,
        .speed_mode = LEDC_HS_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_HS_TIMER
    };
    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, duty);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
    printf("LEDC set duty = %d\n", duty);
}

// Cycle LED intensity up and down
void cycle_led_intensity() {
    int duty = 0;
    int intensity = 0;

    // Increase the intensity
    while (duty < 5000) {
        printf("Intensity %d\n", intensity);
        adjust_led_intensity(duty);
        vTaskDelay(25 / portTICK_PERIOD_MS);
        duty += 500;
        intensity++;
    }

    // Decrease the intensity
    vTaskDelay(25 / portTICK_PERIOD_MS);
    duty = 4000;
    intensity = 8;
    while (duty >= 0) {
        printf("Intensity %d\n", intensity);
        adjust_led_intensity(duty);
        vTaskDelay(25 / portTICK_PERIOD_MS);
        duty -= 500;
        intensity--;
    }
}

// Initialize I2C as master
static void init_i2c_master() {
    printf("\n>> I2C Configuration\n");

    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MASTER_SDA_IO,
        .scl_io_num = MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };

    // Setting I2C parameters
    if (i2c_param_config(I2C_MASTER_NUM, &i2c_config) == ESP_OK) {
        printf("- Parameters: OK\n");
    }

    // Installing I2C driver
    if (i2c_driver_install(I2C_MASTER_NUM, i2c_config.mode, 0, 0, 0) == ESP_OK) {
        printf("- Initialized: Yes\n");
    }

    // Set data mode
    i2c_set_data_mode(I2C_MASTER_NUM, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}



// Test connection to I2C device
int test_connection(uint8_t devAddr, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | WRITE_BIT, ACK_CHECK);
    i2c_master_stop(cmd);
    int err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, timeout / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// Scan for I2C devices
static void scan_i2c_devices() {
    printf("\n>> Scanning I2C bus...\n");
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; ++i) {
        if (test_connection(i, 1000) == ESP_OK) {
            printf("- Device found at address: 0x%X\n", i);
            ++count;
        }
    }
    if (count == 0) {
        printf("- No I2C devices found!\n");
    }
}


int getDeviceID(uint8_t *data) {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDRESS << 1 ) | WRITE_BIT, ACK_CHECK);
  i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDRESS << 1 ) | READ_BIT, ACK_CHECK);
  i2c_master_read_byte(cmd, data, ACK_CHECK);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}


// Write one byte to register
void writeRegister(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDRESS << 1 ) | WRITE_BIT, ACK_CHECK);
    i2c_master_write_byte(cmd, reg, ACK_CHECK);
    i2c_master_write_byte(cmd, data, ACK_CHECK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

// Read register
uint8_t readRegister(uint8_t reg) {
    uint8_t value;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    //start command
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDRESS << 1 ) | WRITE_BIT, ACK_CHECK);
    //register pointer sent
    i2c_master_write_byte(cmd, reg, ACK_CHECK);
    i2c_master_start(cmd);
    //slave followed by read bit
    i2c_master_write_byte(cmd, ( SLAVE_ADDRESS << 1 ) | READ_BIT, ACK_CHECK);
    //place data from register into bus
    i2c_master_read_byte(cmd, &value, ACK_CHECK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return value;
}

// Read 16 bits
int16_t read16(uint8_t reg) {
    uint8_t val1;
    uint8_t val2;
    val1 = readRegister(reg);
    if (reg == 41) {
        val2 = 0;
    } else {
        val2 = readRegister(reg+1);
    }
    return (((int16_t)val2 << 8) | val1);
}



void setRange(range_t range) {
  // Read the data format register to preserve bits
  uint8_t format = readRegister(ADXL343_REG_DATA_FORMAT);

  // Update the data rate
  format &= ~0x0F;
  format |= range;

  
  format |= 0x08;

  // Write the register back to the IC
  writeRegister(ADXL343_REG_DATA_FORMAT, format);

}

range_t getRange(void) {
  // Read the data format register to preserve bits
  return (range_t)(readRegister(ADXL343_REG_DATA_FORMAT) & 0x03);
}

dataRate_t getDataRate(void) {
  return (dataRate_t)(readRegister(ADXL343_REG_BW_RATE) & 0x0F);
}

void getAccel(float *xp, float *yp, float *zp) {
    int16_t xRaw = read16(ADXL343_REG_DATAX0);
    int16_t yRaw = read16(ADXL343_REG_DATAY0);
    int16_t zRaw = read16(ADXL343_REG_DATAZ0);
    printf("Raw X: %d, Raw Y: %d, Raw Z: %d\n", xRaw, yRaw, zRaw);

    *xp = xRaw * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    *yp = yRaw * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    *zp = zRaw * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
}


// Calculate print roll and pitch
void calcRP(float x, float y, float z){
  float roll = atan2(y , z) * 57.3;
  float pitch = atan2((-1*x) , sqrt(y*y + z*z)) * 57.3;
  printf("roll: %.2f \t pitch: %.2f \n", roll, pitch);
}

//Pull the acceleration and calculate roll and pitch
static void test_adxl343() {
  printf("\n>> Polling ADAXL343\n");
  while (1) {
    float xVal, yVal, zVal;
    getAccel(&xVal, &yVal, &zVal);
    calcRP(xVal, yVal, zVal);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}


void init_thermistor() {
  //configure ADC
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(channel, atten);
  adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
}

float gettemp() {
  // Initialize reading variable
  uint32_t val = 0;
  for (int i = 0; i < SAMPLE_NUM; i++) {
    val += adc1_get_raw(channel);
  }
  val /= SAMPLE_NUM; // Average ADC value
  printf("Val: %ld\n", val);

  // Calculate thermistor resistance
  float rtherm;
  const float R_DIVIDER = 2000;
  rtherm = (R_DIVIDER * val) / (4096 - val);  //4096.0
  printf("rtherm: %.2f\n", rtherm);

  // Calculate temperature using Steinhart-Hart equation
  const float B_CONSTANT = 3950;  //3950
  const float REF_RESISTANCE = 2000.0; // Reference resistance 2000Ω at 25°C
  const float REF_TEMPERATURE = 298.15; // Reference temperature in Kelvin (25°C)
  float temp;
  temp = rtherm / REF_RESISTANCE; // (R/Ro)
  //temp = log(temp); // ln(R/Ro)
  temp /= B_CONSTANT; // 1/B * ln(R/Ro)
  temp += 1.0 / REF_TEMPERATURE; // + (1/To)
  temp = 1.0 / temp;
  temp -= 273.15; // Convert from Kelvin to Celsius
 

  return temp;
}


void pwm_task(void *pvParameters) {
    char buf[10];
    uint8_t deviceID;
    float xVal, yVal, zVal; // Accelerometer values

    // Initialize ADXL343 if detected
    if (getDeviceID(&deviceID) == ESP_OK && deviceID == 0xE5) {
        ESP_LOGI(TAG, "ADXL343 detected.");
        writeRegister(ADXL343_REG_INT_ENABLE, 0);  // Disable interrupts
        setRange(ADXL343_RANGE_2_G);               // Set range
        writeRegister(ADXL343_REG_POWER_CTL, 0x08); // Begin measuring
    } else {
        ESP_LOGI(TAG, "ADXL343 not detected. Check your wiring.");
    }

    // Main task loop
    while (1) {
        // User input for LED intensity
        ESP_LOGI(TAG, "Enter LED Intensity (0-9) or 'cycle': ");
        // This should be a non-blocking read in the actual implementation
        int numRead = scanf("%s", buf);

        // If user entered 'cycle', activate cycling mode
        if (numRead > 0 && strcmp(buf, "cycle") == 0) {
            cycle_led_intensity();
        }
        // If user entered a number, adjust LED intensity accordingly
        else if (numRead > 0) {
            int intensity = atoi(buf);
            if (intensity >= 0 && intensity <= 9) {
                int duty = intensity * (8191 / 9); // Calculate the duty based on intensity
                adjust_led_intensity(duty);        // Set the new LED duty
            } else {
                ESP_LOGI(TAG, "Invalid input. Enter a value between 0 and 9 or 'cycle'.");
            }
        }

        // Get and print thermistor temperature
        ESP_LOGI(TAG, "Temp: %.2f°C", gettemp());

        // Get and print accelerometer data if present
        if (deviceID == 0xE5) {
            getAccel(&xVal, &yVal, &zVal); // Get current accelerometer readings
            calcRP(xVal, yVal, zVal);      // Calculate and print roll and pitch
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // Delay to avoid spamming
    }
}


void pwm_task(void *pvParameters);

void initialize_ledc(void) {
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // Set PWM duty resolution to 13 bits
        .freq_hz = 5000, // Set frequency at 5 kHz
        .speed_mode = LEDC_HS_MODE, // Use high-speed mode
        .timer_num = LEDC_HS_TIMER, // Use timer 0
        .clk_cfg = LEDC_AUTO_CLK, // Auto select the source clock
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer)); // Initialize timer

    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_HS_CH0_CHANNEL,
        .duty = 0, // Start with a duty cycle of 0
        .gpio_num = LEDC_HS_CH0_GPIO,
        .speed_mode = LEDC_HS_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_HS_TIMER
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel)); // Initialize channel
}


void adjust_led_brightness(int brightness) {
    uint32_t max_duty = (1 << 13) - 1; // For 13-bit resolution
    uint32_t duty = (brightness * max_duty) / 9; // Scale brightness to duty cycle

    ledc_set_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL, duty);
    ledc_update_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL);
    printf("LED duty set to %ld for brightness level %d\n", duty, brightness);
}



void send_sensor_data(int sock, struct sockaddr_in si_other) {
    float temperature = gettemp();
    float xAccel, yAccel, zAccel;
    getAccel(&xAccel, &yAccel, &zAccel); //Updates xAccel, yAccel, zAccel
    int ledDuty = ledc_get_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL);

    char sensor_data[256];
    sprintf(sensor_data, "Temperature:%.2f,LED Duty:%d,Accel X:%.2f,Accel Y:%.2f,Accel Z:%.2f",
            temperature, ledDuty, xAccel, yAccel, zAccel);
    sendto(sock, sensor_data, strlen(sensor_data), 0, (struct sockaddr *)&si_other, sizeof(si_other));
    ESP_LOGI(TAG, "Sensor data sent: %s", sensor_data);
}



// Function Prototypes
static void udp_server_task(void *pvParameters);
char addr_str[INET_ADDRSTRLEN];

static void udp_server_task(void *pvParameters) {
    struct sockaddr_in si_me, si_other;
    char rx_buffer[128];
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return;
    }
    ESP_LOGI(TAG, "Socket created");

    memset(&si_me, 0, sizeof(si_me));
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(UDP_SERVER_PORT);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    int err = bind(sock, (struct sockaddr *)&si_me, sizeof(si_me));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(sock);
        return;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", UDP_SERVER_PORT);

    while (1) {
        socklen_t socklen = sizeof(si_other);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&si_other, &socklen);
        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            continue;
        }
        rx_buffer[len] = 0; // Null-terminate whatever we received

        // Parse the received buffer for a brightness value
        int brightness;
        if (sscanf(rx_buffer, "%d", &brightness) == 1) {
            adjust_led_brightness(brightness); 

            // After setting LED, now send sensor data
            send_sensor_data(sock, si_other);
        } else {
            ESP_LOGE(TAG, "Received invalid data: %s", rx_buffer);
        }
    }

    if (sock != -1) {
        ESP_LOGI(TAG, "Shutting down socket and restarting...");
        shutdown(sock, 0);
        close(sock);
    }
}



void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    printf("NVS Flash initialized.\n");
    
    int16_t xAccel = read16(ADXL343_REG_DATAX0);
    printf("X-axis acceleration: %d\n", xAccel);
    wifi_init_sta();
    initialize_ledc();
    init_thermistor();
    init_i2c_master();
    scan_i2c_devices();

    // Create a UDP server task
    xTaskCreate(udp_server_task, "UDP Server", 4096, NULL, 5, NULL);
}