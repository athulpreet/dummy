/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define GPS_BUFFER_SIZE 256
#define MAX_LINES 10
char GPS_log[256];

char gpsRxBuffer[GPS_BUFFER_SIZE];
uint16_t gpsRxIndex = 0;








#define MAX_LOG_ENTRY_SIZE 100
#define METADATA_FILE "logmeta.txt"
#define DATALOG_FILE "datalog.txt"

typedef struct {
    uint32_t current_position;
    uint32_t total_entries_written;
    uint8_t buffer_full;
} LogMetadata;

typedef struct {
    char line[512];
} LogEntry;

LogEntry circularBuffer[MAX_LINES];
uint32_t writeIndex = 0;
uint32_t sequence_num = 1;

typedef struct {
    float latitude;
    float longitude;
    float speed_knots;
    float course_degrees;
    float altitude;
    char time[12];
    bool is_valid;
} gps_data_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEBUG_ENABLED 1 // Set to 1 to enable debug prints, 0 to disable
#define GPS_DEBUG_ENABLED 0 // Separate GPS debug to reduce interference

#define MAX_PERSISTENT_ERRORS 8  // Increased from 5
#define MODEM_PWR_RST_Pin GPIO_PIN_6
#define MODEM_PWR_RST_GPIO_Port GPIOC
#define AUTHORIZED_SENDER "+919110470625"
#define GPS_DMA_RX_BUFFER_SIZE 512
#define MAX_NMEA_SENTENCE_LEN 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#if DEBUG_ENABLED
#define DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif

#if GPS_DEBUG_ENABLED
#define GPS_DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
#define GPS_DEBUG_PRINTF(...)
#endif

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* Definitions for defaultTask */

/* USER CODE BEGIN PV */
// RTOS Handles
SemaphoreHandle_t g_modem_mutex;
SemaphoreHandle_t g_gps_data_mutex;
SemaphoreHandle_t g_gps_data_ready_sem;


// GPS DMA Buffer
uint8_t g_gps_dma_rx_buffer[GPS_DMA_RX_BUFFER_SIZE];
volatile uint16_t g_gps_bytes_received = 0;


// Global Data
gps_data_t g_gps_data = {0};


// CIRCULAR BUFFER GLOBALS - ADD THESE
FIL circularLogFile;
LogMetadata logMeta;


// Global Configuration Variables
volatile uint32_t g_mqtt_publish_interval_ms = 5000;
volatile uint32_t g_led_blink_interval_ms = 1000;


// MQTT Config
char g_mqtt_broker_ip[40] = "3.109.116.92";
char g_mqtt_broker_port[6] = "1883";
char g_mqtt_client_id[32] = "spring-client";
char g_mqtt_username[32] = "Thinture";
char g_mqtt_password[32] = "Thinture24";
char g_mqtt_topic[32] = "Test1";

// Status tracking
volatile uint32_t g_session_start_time = 0;
volatile uint32_t g_successful_publishes = 0;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

int __io_putchar(int ch);
void vLedHeartbeatTask(void *pvParameters);
void vMqttTask(void *pvParameters);
void vSmsTask(void *pvParameters);
void vGpsTask(void *pvParameters);
void RTC_SD_Log_Task_Polling(void *pvParameters);

void SD_Test_Task(void *pvParameters);
void perform_modem_power_cycle(void);
bool send_sms(const char* recipient, const char* message);
bool send_and_wait_for_response(const char* cmd, const char* expected_response, uint32_t timeout_ms, uint8_t* error_counter);
void parse_gnrmc(char* gnrmc_sentence);
void parse_gngga(char* gngga_sentence);
float nmea_to_decimal(float nmea_coord, char direction);
void process_gps_buffer(uint8_t* buffer, uint16_t size);
void flush_modem_uart_buffer(void);





/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        g_gps_bytes_received = Size;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(g_gps_data_ready_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}


void flush_modem_uart_buffer(void)
{
    uint8_t flush_char;
    uint16_t flush_count = 0;


    while (HAL_UART_Receive(&huart1, &flush_char, 1, 10) == HAL_OK && flush_count < 100) {
        flush_count++;
    }

    if (flush_count > 0) {
        DEBUG_PRINTF("Flushed %d bytes from modem buffer\r\n", flush_count);
    }
}


float nmea_to_decimal(float nmea_coord, char direction)
{
    if (nmea_coord == 0.0 || isnan(nmea_coord)) {
        return 0.0;
    }

    int degrees = (int)(nmea_coord / 100.0);
    double minutes = nmea_coord - (degrees * 100.0);
    double decimal_degrees = degrees + (minutes / 60.0);

    if (direction == 'S' || direction == 'W') {
        decimal_degrees = -decimal_degrees;
    }

    return decimal_degrees;
}


void parse_gngga(char* gngga_sentence)
{
    char temp_sentence[MAX_NMEA_SENTENCE_LEN];

    strncpy(temp_sentence, gngga_sentence, sizeof(temp_sentence) - 1);
    temp_sentence[sizeof(temp_sentence) - 1] = '\0';

    char* fields[15] = {NULL};
    int field_count = 0;
    char* start = temp_sentence;

    fields[field_count++] = start;
    for (int i = 0; temp_sentence[i] != '\0' && field_count < 15; i++) {
        if (temp_sentence[i] == ',' || temp_sentence[i] == '*') {
            temp_sentence[i] = '\0';
            if (temp_sentence[i+1] != '\0' && field_count < 14) {
                fields[field_count++] = &temp_sentence[i+1];
            }
        }
    }

    if (field_count >= 10 && fields[9] != NULL && strlen(fields[9]) > 0) {
        float altitude = atof(fields[9]);

        if (altitude >= -100.0 && altitude <= 2000.0) {
            if (xSemaphoreTake(g_gps_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                g_gps_data.altitude = altitude;
                xSemaphoreGive(g_gps_data_mutex);
                GPS_DEBUG_PRINTF("Altitude: %.1fm\r\n", altitude);
            }
        }
    }
}


/**
 * @brief Parses a GNRMC NMEA sentence to update GPS data.
 * @note  This is the corrected version that properly handles loss of GPS fix.
 * @param gnrmc_sentence Pointer to the null-terminated GNRMC sentence string.
 */
void parse_gnrmc(char* gnrmc_sentence)
{
    // A temporary sentence buffer to avoid modifying the original DMA buffer
    char temp_sentence[MAX_NMEA_SENTENCE_LEN];
    // A temporary struct to hold newly parsed data
    gps_data_t new_data = {0};

    // Safely copy the sentence for processing
    strncpy(temp_sentence, gnrmc_sentence, sizeof(temp_sentence) - 1);
    temp_sentence[sizeof(temp_sentence) - 1] = '\0';

    // Array to hold pointers to each field in the sentence
    char* fields[15] = {NULL};
    int field_count = 0;
    char* start = temp_sentence;

    // Manually tokenize the sentence by replacing commas with null terminators
    fields[field_count++] = start;
    for (int i = 0; temp_sentence[i] != '\0' && field_count < 15; i++) {
        if (temp_sentence[i] == ',' || temp_sentence[i] == '*') {
            temp_sentence[i] = '\0';
            if (temp_sentence[i+1] != '\0' && field_count < 14) {
                fields[field_count++] = &temp_sentence[i+1];
            }
        }
    }

    // Ensure we have enough fields to parse a valid RMC sentence
    if (field_count >= 9) {
        // Field 1: Time (UTC)
        if (fields[1] != NULL && strlen(fields[1]) > 0 && strlen(fields[1]) < sizeof(new_data.time)) {
            strncpy(new_data.time, fields[1], sizeof(new_data.time) - 1);
            new_data.time[sizeof(new_data.time) - 1] = '\0';
        }

        // Field 2: Status ('A' = Active/Valid, 'V' = Void/Invalid)
        if (fields[2] != NULL && strlen(fields[2]) > 0) {
            new_data.is_valid = (fields[2][0] == 'A');
        }

        // Fields 3,4: Latitude and Direction
        if (new_data.is_valid && fields[3] != NULL && fields[4] != NULL && strlen(fields[3]) > 0 && strlen(fields[4]) > 0) {
            float lat_temp = atof(fields[3]);
            if (lat_temp > 0.0f && lat_temp < 9000.0f) {
                new_data.latitude = nmea_to_decimal(lat_temp, fields[4][0]);
                if (new_data.latitude < -90.0f || new_data.latitude > 90.0f) {
                    new_data.latitude = 0.0f; // Sanity check failed
                }
            }
        }

        // Fields 5,6: Longitude and Direction
        if (new_data.is_valid && fields[5] != NULL && fields[6] != NULL && strlen(fields[5]) > 0 && strlen(fields[6]) > 0) {
            float lon_temp = atof(fields[5]);
            if (lon_temp > 0.0f && lon_temp < 18000.0f) {
                new_data.longitude = nmea_to_decimal(lon_temp, fields[6][0]);
                if (new_data.longitude < -180.0f || new_data.longitude > 180.0f) {
                    new_data.longitude = 0.0f; // Sanity check failed
                }
            }
        }

        // Field 7: Speed over ground in knots
        if (fields[7] != NULL && strlen(fields[7]) > 0) {
            new_data.speed_knots = atof(fields[7]);
            if (new_data.speed_knots < 0.0f || new_data.speed_knots > 1000.0f) {
                new_data.speed_knots = 0.0f; // Sanity check
            }
        }

        // Field 8: Course over ground in degrees
        if (fields[8] != NULL && strlen(fields[8]) > 0) {
            new_data.course_degrees = atof(fields[8]);
            if (new_data.course_degrees < 0.0f || new_data.course_degrees > 360.0f) {
                new_data.course_degrees = 0.0f; // Sanity check
            }
        }
    }

    // --- FIX STARTS HERE ---
    // Safely update the global GPS data structure
    if (xSemaphoreTake(g_gps_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {

        if (new_data.is_valid) {
            // If the newly parsed data is valid, update the global structure.
            // A sanity check for non-zero coordinates adds extra robustness.
            if (new_data.latitude != 0.0f && new_data.longitude != 0.0f) {
                 // Preserve altitude, which comes from the separate GNGGA sentence
                 new_data.altitude = g_gps_data.altitude;
                 g_gps_data = new_data; // Copy the entire valid structure
                 GPS_DEBUG_PRINTF("GPS FIXED: %.6f, %.6f\r\n", g_gps_data.latitude, g_gps_data.longitude);
            }
        } else {
            // If the newly parsed data is NOT valid, ONLY update the status flag.
            // This is the critical change that fixes the bug.
            g_gps_data.is_valid = false;
        }

        xSemaphoreGive(g_gps_data_mutex);
    }
}


void process_gps_buffer(uint8_t* buffer, uint16_t size)
{
    static char nmea_sentence[MAX_NMEA_SENTENCE_LEN];
    static uint16_t sentence_index = 0;

    if (buffer == NULL || size == 0) {
        return;
    }

    for (uint16_t i = 0; i < size; i++) {
        char ch = (char)buffer[i];

        if (ch == 0) continue;

        if (ch == '$') {
            sentence_index = 0;
        }

        if (sentence_index < (sizeof(nmea_sentence) - 2)) {
            nmea_sentence[sentence_index++] = ch;
        } else {
            sentence_index = 0;
            continue;
        }

        if (ch == '\n') {
            nmea_sentence[sentence_index] = '\0';

            if (nmea_sentence[0] == '$' && sentence_index > 10) {
                if (strncmp(nmea_sentence, "$GNRMC", 6) == 0) {
                    GPS_DEBUG_PRINTF("GNRMC found\r\n");
                    parse_gnrmc(nmea_sentence);
                } else if (strncmp(nmea_sentence, "$GNGGA", 6) == 0) {
                    GPS_DEBUG_PRINTF("GNGGA found\r\n");
                    parse_gngga(nmea_sentence);
                }
            }

            sentence_index = 0;
        }
    }
}


void vGpsTask(void *pvParameters)
{
    static uint32_t callback_count = 0;
    static uint32_t last_status_print = 0;

    GPS_DEBUG_PRINTF("\r\n--- GPS Starting Enhanced GPS Task (DMA Mode) ---\r\n");
    GPS_DEBUG_PRINTF("GPS UART3 configured at 38400 baud\r\n");

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_gps_dma_rx_buffer, GPS_DMA_RX_BUFFER_SIZE) != HAL_OK) {
       GPS_DEBUG_PRINTF("ERROR: Failed to start GPS DMA reception!\r\n");
    } else {
        GPS_DEBUG_PRINTF("GPS DMA reception started successfully\r\n");
    }

    while(1)
    {
        if (xSemaphoreTake(g_gps_data_ready_sem, pdMS_TO_TICKS(5000)) == pdTRUE)
        {
            callback_count++;

            // Only check if modem is NOT busy
            //if (xSemaphoreTake(g_modem_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                // Modem not busy, can process GPS
                uint16_t bytes_received = g_gps_bytes_received;

                if (bytes_received > 0) {
                    process_gps_buffer(g_gps_dma_rx_buffer, bytes_received);
                }


                if ((HAL_GetTick() - last_status_print) > 10000) {
                    gps_data_t current_gps_data = {0};
                    if (xSemaphoreTake(g_gps_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                        current_gps_data = g_gps_data;
                        xSemaphoreGive(g_gps_data_mutex);
                    }

                    if (current_gps_data.is_valid) {
                        GPS_DEBUG_PRINTF("GPS STATUS: %.6f,%.6f Alt:%.1fm Spd:%.1fkn\r\n",
                               current_gps_data.latitude, current_gps_data.longitude,
                               current_gps_data.altitude, current_gps_data.speed_knots);
                    } else {
                        GPS_DEBUG_PRINTF("GPS: No Fix (%lu callbacks)\r\n", callback_count);
                    }
                    last_status_print = HAL_GetTick();
                }

               // xSemaphoreGive(g_modem_mutex);
            //}
            // If modem is busy, skip GPS processing this cycle

            memset(g_gps_dma_rx_buffer, 0, GPS_DMA_RX_BUFFER_SIZE);
            HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_gps_dma_rx_buffer, GPS_DMA_RX_BUFFER_SIZE);

            vTaskDelay(pdMS_TO_TICKS(50));
        }
        else
        {
            GPS_DEBUG_PRINTF("GPS Timeout - restarting DMA\r\n");
            HAL_UART_AbortReceive(&huart3);
            vTaskDelay(pdMS_TO_TICKS(100));
            HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_gps_dma_rx_buffer, GPS_DMA_RX_BUFFER_SIZE);
        }
    }
}


bool send_and_wait_for_response(const char* cmd, const char* expected_response, uint32_t timeout_ms, uint8_t* error_counter)
{
    char rx_buffer[512] = {0};
    uint16_t rx_index = 0;
    uint8_t rx_char;
    uint32_t start_tick = HAL_GetTick();


    flush_modem_uart_buffer();
    vTaskDelay(pdMS_TO_TICKS(50));

    DEBUG_PRINTF("CMD >> %s", cmd);

    if (huart1.Instance != USART1) {
        DEBUG_PRINTF("ERROR: USART1 not initialized!\r\n");
        return false;
    }

    HAL_StatusTypeDef tx_status = HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
    if (tx_status != HAL_OK) {
        DEBUG_PRINTF("ERROR: UART transmit failed with status %d\r\n", tx_status);
        return false;
    }

    // Extended timeout for high message counts
    uint32_t adjusted_timeout = timeout_ms;
    if (g_successful_publishes > 50) {
        adjusted_timeout = timeout_ms + 5000; // Add 5 seconds after 50 messages
    }

    while ((HAL_GetTick() - start_tick) < adjusted_timeout)
    {
        HAL_StatusTypeDef rx_status = HAL_UART_Receive(&huart1, &rx_char, 1, 100);
        if (rx_status == HAL_OK)
        {
            if (rx_index < sizeof(rx_buffer) - 1)
            {
                rx_buffer[rx_index++] = rx_char;
            }


            if (strstr(rx_buffer, expected_response) != NULL)
            {
                DEBUG_PRINTF("RSP << %s\r\n", rx_buffer);
                DEBUG_PRINTF("SUCCESS: Found '%s' (Runtime: %lums, Msgs: %lu)\r\n",
                           expected_response, HAL_GetTick() - g_session_start_time, g_successful_publishes);
                if (error_counter != NULL) *error_counter = 0;
                return true;
            }
        }
        else if (rx_status != HAL_TIMEOUT) {
            DEBUG_PRINTF("UART RX Error: %d\r\n", rx_status);
        }
    }

    DEBUG_PRINTF("RSP << %s\r\n", rx_buffer);
    DEBUG_PRINTF("ERROR: Timeout after %lums. Expected '%s' (Runtime: %lums, Msgs: %lu)\r\n",
                HAL_GetTick() - start_tick, expected_response,
                HAL_GetTick() - g_session_start_time, g_successful_publishes);


    if (rx_index == 0) {
        DEBUG_PRINTF("!!! NO RESPONSE FROM MODEM !!!\r\n");
    } else if (rx_index < 5) {
        DEBUG_PRINTF("Partial response - possible interference\r\n");
    } else if (strstr(rx_buffer, "ERROR") != NULL) {
        DEBUG_PRINTF("Modem returned ERROR response\r\n");
    } else {
        DEBUG_PRINTF("Unexpected response format\r\n");
    }

    if (error_counter != NULL)
    {
        (*error_counter)++;
        DEBUG_PRINTF("Error count now: %d/%d\r\n", *error_counter, MAX_PERSISTENT_ERRORS);

        if (*error_counter >= MAX_PERSISTENT_ERRORS)
        {
            DEBUG_PRINTF("!!! MAX PERSISTENT ERRORS REACHED !!!\r\n");
            DEBUG_PRINTF("Session runtime: %lums, Successful publishes: %lu\r\n",
                        HAL_GetTick() - g_session_start_time, g_successful_publishes);
            DEBUG_PRINTF("Performing system reset...\r\n");
            vTaskDelay(pdMS_TO_TICKS(500));
            HAL_NVIC_SystemReset();
        }
    }
    return false;
}

void perform_modem_power_cycle(void)
{
    DEBUG_PRINTF("--- Performing Modem Power Cycle ---\r\n");

    // Stop GPS processing during power cycle
    HAL_UART_AbortReceive(&huart3);

    HAL_GPIO_WritePin(MODEM_PWR_RST_GPIO_Port, MODEM_PWR_RST_Pin, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(250));
    DEBUG_PRINTF("Powering modem OFF...\r\n");
    HAL_GPIO_WritePin(MODEM_PWR_RST_GPIO_Port, MODEM_PWR_RST_Pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(2000));
    HAL_GPIO_WritePin(MODEM_PWR_RST_GPIO_Port, MODEM_PWR_RST_Pin, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(5000));
    DEBUG_PRINTF("Powering modem ON...\r\n");
    HAL_GPIO_WritePin(MODEM_PWR_RST_GPIO_Port, MODEM_PWR_RST_Pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(2000));
    HAL_GPIO_WritePin(MODEM_PWR_RST_GPIO_Port, MODEM_PWR_RST_Pin, GPIO_PIN_SET);
    DEBUG_PRINTF("--- Modem Power Cycle Complete. Waiting for boot... ---\r\n");
    vTaskDelay(pdMS_TO_TICKS(12000));

    // Restart GPS processing
    memset(g_gps_dma_rx_buffer, 0, GPS_DMA_RX_BUFFER_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_gps_dma_rx_buffer, GPS_DMA_RX_BUFFER_SIZE);
}


void vMqttTask(void *pvParameters)
{
    char command_buffer[256];
    static uint8_t mqtt_error_count = 0;
    bool is_mqtt_connected = false;
    static uint32_t last_keepalive = 0;
    static uint32_t connection_attempts = 0;

    vTaskDelay(pdMS_TO_TICKS(5000));
    g_session_start_time = HAL_GetTick();

    // Enhanced modem hardware test with less GPS interference
    DEBUG_PRINTF("\r\n=== TESTING MODEM HARDWARE ===\r\n");

    // Temporarily stop GPS to test modem cleanly
    HAL_UART_AbortReceive(&huart3);
    vTaskDelay(pdMS_TO_TICKS(500));

    HAL_GPIO_WritePin(MODEM_PWR_RST_GPIO_Port, MODEM_PWR_RST_Pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(100));
    HAL_GPIO_WritePin(MODEM_PWR_RST_GPIO_Port, MODEM_PWR_RST_Pin, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Enhanced UART test
    flush_modem_uart_buffer();
    DEBUG_PRINTF("Testing modem communication...\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)"AT\r\n", 4, HAL_MAX_DELAY);

    uint8_t test_rx[100] = {0};
    uint16_t test_idx = 0;
    uint32_t test_start = HAL_GetTick();

    while ((HAL_GetTick() - test_start) < 3000 && test_idx < 99) {
        uint8_t test_char;
        if (HAL_UART_Receive(&huart1, &test_char, 1, 100) == HAL_OK) {
            test_rx[test_idx++] = test_char;
        }
    }

    test_rx[test_idx] = '\0';
    DEBUG_PRINTF("Modem test response (%d bytes): %s\r\n", test_idx, (char*)test_rx);

    if (test_idx == 0) {
        DEBUG_PRINTF("!!! NO MODEM RESPONSE - CHECK HARDWARE !!!\r\n");
    } else {
        DEBUG_PRINTF("Modem responding - continuing...\r\n");
    }

    DEBUG_PRINTF("=== END MODEM TEST ===\r\n\r\n");


    memset(g_gps_dma_rx_buffer, 0, GPS_DMA_RX_BUFFER_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_gps_dma_rx_buffer, GPS_DMA_RX_BUFFER_SIZE);

    while(1)
    {
        if (xSemaphoreTake(g_modem_mutex, portMAX_DELAY) == pdTRUE)
        {
            if (!is_mqtt_connected)
            {
                connection_attempts++;
                DEBUG_PRINTF("--- MQTT Connection Attempt #%lu ---\r\n", connection_attempts);


                if (mqtt_error_count > 0 && mqtt_error_count % 3 == 0) {
                     perform_modem_power_cycle();
                }

                bool setup_ok = true;

                // Basic modem test
                if (!send_and_wait_for_response("AT\r\n", "OK", 3000, &mqtt_error_count)) setup_ok = false;
                if(setup_ok) vTaskDelay(pdMS_TO_TICKS(800)); // Increased delays

                // Network attachment
                if (setup_ok) { if (!send_and_wait_for_response("AT+CGATT=1\r\n", "OK", 10000, &mqtt_error_count)) setup_ok = false; }
                if(setup_ok) vTaskDelay(pdMS_TO_TICKS(800));

                // Deactivate context
                if (setup_ok) { send_and_wait_for_response("AT+CGACT=0,1\r\n", "OK", 8000, NULL); }
                if(setup_ok) vTaskDelay(pdMS_TO_TICKS(800));

                // Set APN
                if (setup_ok) {
                    snprintf(command_buffer, sizeof(command_buffer), "AT+CGDCONT=1,\"IP\",\"%s\"\r\n", "internet");
                    if (!send_and_wait_for_response(command_buffer, "OK", 5000, &mqtt_error_count)) setup_ok = false;
                }
                if(setup_ok) vTaskDelay(pdMS_TO_TICKS(800));

                // Activate context
                if (setup_ok) { if (!send_and_wait_for_response("AT+CGACT=1,1\r\n", "OK", 15000, &mqtt_error_count)) setup_ok = false; }
                if(setup_ok) vTaskDelay(pdMS_TO_TICKS(1000));

                // Open MQTT connection
                if (setup_ok) {
                    snprintf(command_buffer, sizeof(command_buffer), "AT+QMTOPEN=0,\"%s\",%s\r\n", g_mqtt_broker_ip, g_mqtt_broker_port);
                    if (!send_and_wait_for_response(command_buffer, "+QMTOPEN: 0,0", 25000, &mqtt_error_count)) setup_ok = false;
                }
                if(setup_ok) vTaskDelay(pdMS_TO_TICKS(1000));

                // Connect to MQTT broker
                if (setup_ok) {
                    snprintf(command_buffer, sizeof(command_buffer), "AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"\r\n",
                           g_mqtt_client_id, g_mqtt_username, g_mqtt_password);
                    if (!send_and_wait_for_response(command_buffer, "+QMTCONN: 0,0,0", 20000, &mqtt_error_count)) setup_ok = false;
                }

                if (setup_ok) {
                    is_mqtt_connected = true;
                    mqtt_error_count = 0;
                    last_keepalive = HAL_GetTick();
                    DEBUG_PRINTF("--- MQTT Setup Complete (Attempt #%lu) ---\r\n", connection_attempts);
                } else {
                    DEBUG_PRINTF("--- MQTT Setup Failed, will retry in 30 seconds ---\r\n");
                    is_mqtt_connected = false;
                    xSemaphoreGive(g_modem_mutex);
                    vTaskDelay(pdMS_TO_TICKS(30000));
                    continue;
                }
            }

            if (is_mqtt_connected)
            {
                // Send keep-alive periodically to prevent broker timeout
            /*    if ((HAL_GetTick() - last_keepalive) > 120000) { // Every 2 minutes
                    if (send_and_wait_for_response("AT+QMTPING=0\r\n", "+QMTPING: 0,0", 10000, NULL)) {
                        DEBUG_PRINTF("MQTT Keep-alive successful\r\n");
                    } else {
                        DEBUG_PRINTF("MQTT Keep-alive failed - connection may be lost\r\n");
                        is_mqtt_connected = false;
                    }
                    last_keepalive = HAL_GetTick();
                }*/

                if (is_mqtt_connected)
                {
                    static int message_count = 0;
                    char mqtt_payload[350];
                    gps_data_t current_gps_data = {0};

                    if (xSemaphoreTake(g_gps_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
                    {
                        current_gps_data = g_gps_data;
                        xSemaphoreGive(g_gps_data_mutex);
                    }

                    if (current_gps_data.is_valid)
                    {
                        snprintf(mqtt_payload, sizeof(mqtt_payload),
                                 "{\"device\":\"%s\",\"msg_id\":%d,\"status\":\"GPS FIXED\",\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.1f,\"spd\":%.1f,\"crs\":%.1f,\"time\":\"%s\",\"session_time\":%lu}",
                                 g_mqtt_client_id, ++message_count,
                                 current_gps_data.latitude, current_gps_data.longitude, current_gps_data.altitude,
                                 current_gps_data.speed_knots, current_gps_data.course_degrees, current_gps_data.time,
                                 (HAL_GetTick() - g_session_start_time) / 1000);
                    }
                    else
                    {
                        snprintf(mqtt_payload, sizeof(mqtt_payload),
                                 "{\"device\":\"%s\",\"msg_id\":%d,\"status\":\"GPS NOT FIXED\",\"session_time\":%lu}",
                                 g_mqtt_client_id, ++message_count,
                                 (HAL_GetTick() - g_session_start_time) / 1000);
                    }

                    snprintf(command_buffer, sizeof(command_buffer), "AT+QMTPUBEX=0,0,0,0,\"%s\",%d\r\n",
                           g_mqtt_topic, strlen(mqtt_payload));

                    // Enhanced publish with better error handling
                    bool publish_success = false;
                    if (send_and_wait_for_response(command_buffer, ">", 8000, &mqtt_error_count))
                    {
                        vTaskDelay(pdMS_TO_TICKS(100)); // Small delay before payload

                        if (send_and_wait_for_response(mqtt_payload, "+QMTPUBEX: 0,0,0", 10000, &mqtt_error_count)) {
                            publish_success = true;
                            g_successful_publishes++; // Track successful publishes
                            mqtt_error_count = 0; // CRITICAL: Reset error counter on success
                            DEBUG_PRINTF("PUBLISH SUCCESS #%lu (Total: %lu)\r\n", message_count, g_successful_publishes);
                        } else {
                            DEBUG_PRINTF("PUBLISH PAYLOAD FAILED - connection lost\r\n");
                            is_mqtt_connected = false;
                        }
                    }
                    else
                    {
                        DEBUG_PRINTF("PUBLISH COMMAND FAILED - connection lost\r\n");
                        is_mqtt_connected = false;
                    }


                    if (message_count % 10 == 0 && publish_success) {
                        DEBUG_PRINTF("Performing periodic buffer flush (Msg #%d)\r\n", message_count);
                        flush_modem_uart_buffer();
                        vTaskDelay(pdMS_TO_TICKS(500));
                    }
                }
            }
            xSemaphoreGive(g_modem_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(g_mqtt_publish_interval_ms));
    }
}


void vSmsTask(void *pvParameters)
{
    char sms_buffer[512];
    char command_buffer[100];

    vTaskDelay(pdMS_TO_TICKS(10000));


    if (xSemaphoreTake(g_modem_mutex, portMAX_DELAY) == pdTRUE)
    {
        DEBUG_PRINTF("=== STARTUP: Clearing all SMS messages ===\r\n");
        flush_modem_uart_buffer();
        vTaskDelay(pdMS_TO_TICKS(200));

        send_and_wait_for_response("AT+CMGF=1\r\n", "OK", 3000, NULL);
        send_and_wait_for_response("AT+CMGDA=\"DEL ALL\"\r\n", "OK", 5000, NULL);

        DEBUG_PRINTF("All SMS messages cleared on startup\r\n");
        xSemaphoreGive(g_modem_mutex);
    }

    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(15000));

        if (xSemaphoreTake(g_modem_mutex, portMAX_DELAY) == pdTRUE)
        {
            DEBUG_PRINTF("--- Checking for SMS Commands ---\r\n");


            flush_modem_uart_buffer();
            vTaskDelay(pdMS_TO_TICKS(200));

            if (!send_and_wait_for_response("AT+CMGF=1\r\n", "OK", 3000, NULL))
            {
                xSemaphoreGive(g_modem_mutex);
                continue;
            }

            memset(sms_buffer, 0, sizeof(sms_buffer));
            HAL_UART_Transmit(&huart1, (uint8_t*)"AT+CMGL=\"ALL\"\r\n", strlen("AT+CMGL=\"ALL\"\r\n"), HAL_MAX_DELAY);

            uint16_t rx_index = 0;
            uint8_t rx_char;
            uint32_t start_tick = HAL_GetTick();
            while((HAL_GetTick() - start_tick) < 8000) // Increased timeout
            {
                if (HAL_UART_Receive(&huart1, &rx_char, 1, 200) == HAL_OK)
                {
                    if (rx_index < sizeof(sms_buffer) - 1)
                    {
                        sms_buffer[rx_index++] = rx_char;
                        if (strstr(sms_buffer, "\r\nOK\r\n") != NULL || strstr(sms_buffer, "\r\nERROR\r\n") != NULL)
                        {
                            break;
                        }
                    }
                }
            }
            DEBUG_PRINTF("SMS RSP << %s\r\n", sms_buffer);

            char* current_msg_ptr = strstr(sms_buffer, "+CMGL:");
            while (current_msg_ptr != NULL)
            {
                int msg_index;
                char sender_number[20] = {0};
                char msg_content[161] = {0};

                sscanf(current_msg_ptr, "+CMGL: %d,\"%*[^\"]\",\"%[^\"]\"", &msg_index, sender_number);
                DEBUG_PRINTF("Found SMS at index %d from %s\n", msg_index, sender_number);

                char* content_start = strchr(current_msg_ptr, '\n');
                if (content_start != NULL)
                {
                    content_start++;
                    char* content_end = strchr(content_start, '\n');
                    if (content_end != NULL)
                    {
                        size_t content_len = content_end - content_start;
                        if (content_len < sizeof(msg_content))
                        {
                            strncpy(msg_content, content_start, content_len);
                            if (content_len > 0 && msg_content[content_len - 1] == '\r')
                            {
                                msg_content[content_len - 1] = '\0';
                            }
                        }
                    }
                }

                if (strcmp(sender_number, AUTHORIZED_SENDER) == 0)
                {
                    DEBUG_PRINTF("Authorized message: '%s'\r\n", msg_content);
                    char ack_message[120] = "ACK: Unknown Command";
                    char* value_ptr = strchr(msg_content, ':');
                    if (value_ptr != NULL)
                    {
                        *value_ptr = '\0';
                        value_ptr++;
                    }

                    if (strcmp(msg_content, "SET_MQTT_INTERVAL") == 0 && value_ptr != NULL) {
                        g_mqtt_publish_interval_ms = atoi(value_ptr);
                        snprintf(ack_message, sizeof(ack_message), "ACK: MQTT Interval set to %lums", g_mqtt_publish_interval_ms);
                    }
                    else if (strcmp(msg_content, "SET_LED_INTERVAL") == 0 && value_ptr != NULL) {
                        g_led_blink_interval_ms = atoi(value_ptr);
                        snprintf(ack_message, sizeof(ack_message), "ACK: LED Interval set to %lums", g_led_blink_interval_ms);
                    }
                    else if (strcmp(msg_content, "SET_MQTT_BROKER") == 0 && value_ptr != NULL) {
                        char port[6];
                        sscanf(value_ptr, "%[^,],%s", g_mqtt_broker_ip, port);
                        strcpy(g_mqtt_broker_port, port);
                        snprintf(ack_message, sizeof(ack_message), "ACK: Broker set to %s:%s", g_mqtt_broker_ip, g_mqtt_broker_port);
                    }
                    else if (strcmp(msg_content, "SET_MQTT_CLIENT") == 0 && value_ptr != NULL) {
                        strncpy(g_mqtt_client_id, value_ptr, sizeof(g_mqtt_client_id) - 1);
                        snprintf(ack_message, sizeof(ack_message), "ACK: Client ID set");
                    }
                    else if (strcmp(msg_content, "SET_MQTT_USER") == 0 && value_ptr != NULL) {
                        strncpy(g_mqtt_username, value_ptr, sizeof(g_mqtt_username) - 1);
                        snprintf(ack_message, sizeof(ack_message), "ACK: User set");
                    }
                    else if (strcmp(msg_content, "SET_MQTT_PASS") == 0 && value_ptr != NULL) {
                        strncpy(g_mqtt_password, value_ptr, sizeof(g_mqtt_password) - 1);
                        snprintf(ack_message, sizeof(ack_message), "ACK: Password set");
                    }
                    else if (strcmp(msg_content, "SET_MQTT_TOPIC") == 0 && value_ptr != NULL) {
                        strncpy(g_mqtt_topic, value_ptr, sizeof(g_mqtt_topic) - 1);
                        snprintf(ack_message, sizeof(ack_message), "ACK: Topic set");
                    }
                    else if (strcmp(msg_content, "GPS_STATUS") == 0) {
                        gps_data_t current_gps_data = {0};
                        if (xSemaphoreTake(g_gps_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                            current_gps_data = g_gps_data;
                            xSemaphoreGive(g_gps_data_mutex);
                        }
                        if (current_gps_data.is_valid) {
                            snprintf(ack_message, sizeof(ack_message), "GPS: %.5f,%.5f Alt:%.1fm Spd:%.1fkn Msgs:%lu",
                                   current_gps_data.latitude, current_gps_data.longitude,
                                   current_gps_data.altitude, current_gps_data.speed_knots, g_successful_publishes);
                        } else {
                            snprintf(ack_message, sizeof(ack_message), "GPS: No Fix. Msgs:%lu Runtime:%lus",
                                   g_successful_publishes, (HAL_GetTick() - g_session_start_time) / 1000);
                        }
                    }
                    else if (strcmp(msg_content, "STATUS") == 0) {
                        snprintf(ack_message, sizeof(ack_message), "Runtime:%lus Msgs:%lu Connected:OK",
                               (HAL_GetTick() - g_session_start_time) / 1000, g_successful_publishes);
                    }
                    else if (strcmp(msg_content, "REBOOT") == 0)
                    {

                        snprintf(command_buffer, sizeof(command_buffer), "AT+CMGD=%d\r\n", msg_index);
                        send_and_wait_for_response(command_buffer, "OK", 3000, NULL);


                        vTaskDelay(pdMS_TO_TICKS(500));


                        snprintf(ack_message, sizeof(ack_message), "ACK: Rebooting after %lu messages...", g_successful_publishes);
                        send_sms(AUTHORIZED_SENDER, ack_message);


                        vTaskDelay(pdMS_TO_TICKS(2000));

                        DEBUG_PRINTF("REBOOT command executed - resetting system\r\n");
                        xSemaphoreGive(g_modem_mutex);
                        HAL_NVIC_SystemReset();
                    }
                    send_sms(AUTHORIZED_SENDER, ack_message);
                }
                else
                {
                    DEBUG_PRINTF("Ignoring message from unauthorized sender: %s\r\n", sender_number);
                }

                snprintf(command_buffer, sizeof(command_buffer), "AT+CMGD=%d\r\n", msg_index);
                send_and_wait_for_response(command_buffer, "OK", 3000, NULL);

                current_msg_ptr = strstr(current_msg_ptr + 1, "+CMGL:");
            }
            xSemaphoreGive(g_modem_mutex);
        }
    }
}




// SD Task: sample GPS every 5 seconds and keep only last 60 lines






/**
 * @brief A simple task to test writing "Hello" to an SD card and reading it back.
 */
void SD_Test_Task(void *pvParameters)
{
    FRESULT fres;         // FatFs result code
    FATFS FatFs;          // File system object
    FIL fil;              // File object
    UINT bytesWrote, bytesRead;
    char wtext[] = "Hello"; // The text to write
    char rtext[20];       // A buffer to read the text back into

    // A short delay to let the system stabilize after startup
    vTaskDelay(pdMS_TO_TICKS(1000));
    DEBUG_PRINTF("\r\n--- SD Card Simple Read/Write Test ---\r\n");

    // 1. Mount the SD card
    // The "1" tells it to mount immediately.
    fres = f_mount(&FatFs, "", 1);
    if (fres != FR_OK)
    {
        DEBUG_PRINTF("ERROR: SD card mount failed (%d)\r\n", fres);
        vTaskDelete(NULL); // End this task if mount fails
        return;
    }
    DEBUG_PRINTF("✅ Step 1: SD card mounted.\r\n");


    // ========== WRITE "HELLO" ==========

    // 2. Open file for writing.
    // FA_CREATE_ALWAYS: Creates a new file. If it already exists, it will be blank.
    // FA_WRITE: We need to write to it.
    fres = f_open(&fil, "hello.txt", FA_CREATE_ALWAYS | FA_WRITE);
    if (fres != FR_OK)
    {
        DEBUG_PRINTF("ERROR: Could not open 'hello.txt' for writing (%d)\r\n", fres);
        vTaskDelete(NULL);
        return;
    }

    // 3. Write the text "Hello" to the file.
    fres = f_write(&fil, wtext, strlen(wtext), &bytesWrote);
    if (fres != FR_OK)
    {
        DEBUG_PRINTF("ERROR: Could not write to file (%d)\r\n", fres);
        f_close(&fil); // Close file before exiting
        vTaskDelete(NULL);
        return;
    }
    DEBUG_PRINTF("✅ Step 2: Wrote '%s' to hello.txt.\r\n", wtext);


    // 4. Close the file. THIS IS CRITICAL TO SAVE THE DATA.
    f_close(&fil);


    // ========== READ "HELLO" ==========

    // 5. Open the same file for reading.
    fres = f_open(&fil, "hello.txt", FA_READ);
    if (fres != FR_OK)
    {
        DEBUG_PRINTF("ERROR: Could not open 'hello.txt' for reading (%d)\r\n", fres);
        vTaskDelete(NULL);
        return;
    }

    // 6. Read the data from the file into our buffer 'rtext'.
    fres = f_read(&fil, rtext, sizeof(rtext) - 1, &bytesRead);
    if (fres != FR_OK)
    {
        DEBUG_PRINTF("ERROR: Could not read from file (%d)\r\n", fres);
        f_close(&fil);
        vTaskDelete(NULL);
        return;
    }

    // 7. Add a null terminator to make it a proper string.
    rtext[bytesRead] = '\0';
    DEBUG_PRINTF("✅ Step 3: Read '%s' from hello.txt.\r\n", rtext);

    // 8. Close the file.
    f_close(&fil);


    // ========== VERIFY THE RESULT ==========
    if (strcmp(wtext, rtext) == 0) {
        DEBUG_PRINTF("\r\nSUCCESS! The text read matches the text written. 🎉\r\n");
    } else {
        DEBUG_PRINTF("\r\nFAILURE! Data does not match. ❌\r\n");
    }

    // Unmount the card and delete the task to finish.
    f_mount(NULL, "", 0);
    DEBUG_PRINTF("--- Test complete. ---\r\n");
    vTaskDelete(NULL);
}






bool send_sms(const char* recipient, const char* message)
{
    char cmd_buf[200];
    char ctrl_z = 26;

    DEBUG_PRINTF("--- Sending SMS to %s ---\r\n", recipient);


    flush_modem_uart_buffer();
    vTaskDelay(pdMS_TO_TICKS(200));

    if (!send_and_wait_for_response("AT+CMGF=1\r\n", "OK", 3000, NULL)) return false;

    snprintf(cmd_buf, sizeof(cmd_buf), "AT+CMGS=\"%s\"\r\n", recipient);
    if (!send_and_wait_for_response(cmd_buf, ">", 8000, NULL)) return false;

    vTaskDelay(pdMS_TO_TICKS(200));

    snprintf(cmd_buf, sizeof(cmd_buf), "%s%c", message, ctrl_z);
    if (!send_and_wait_for_response(cmd_buf, "OK", 15000, NULL)) return false;

    DEBUG_PRINTF("SMS sent successfully.\r\n");
    return true;
}





//RTC ********************************************************************************

//RTC - CIRCULAR BUFFER IMPLEMENTATION ****************************************************

//RTC - CIRCULAR BUFFER IMPLEMENTATION ****************************************************

// =================================================================================
// == DS3231 RTC and Circular Buffer SD Logging Task (72 Hours of Data)
// =================================================================================

// -- Part 1: DS3231 Definitions and Basic Functions (unchanged) --

#define DS3231_I2C_ADDR  0x68 << 1
#define DS3231_REG_SECONDS   0x00
#define DS3231_REG_CONTROL   0x0E
#define DS3231_REG_STATUS    0x0F

// -- Part 2: Circular Buffer Configuration --

#define LOG_INTERVAL_SECONDS 5
#define HOURS_TO_STORE 24
#define TOTAL_LOG_ENTRIES ((uint32_t)((HOURS_TO_STORE * 3600UL) / LOG_INTERVAL_SECONDS))  // 51,840 entries
//#define TOTAL_LOG_ENTRIES 100
#define MAX_LOG_ENTRY_SIZE 100  // Maximum size of each log entry in bytes
#define METADATA_FILE "logmeta.txt"
#define DATALOG_FILE "datalog.txt"

// RTC Time structure for DS3231
typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hour;
    uint8_t day_of_week;
    uint8_t day_of_month;
    uint8_t month;
    uint8_t year;
} RTC_Time;

// DS3231 Helper Functions
static uint8_t decToBcd(int val) { return (uint8_t)((val / 10 * 16) + (val % 10)); }
static int bcdToDec(uint8_t val) { return (int)((val / 16 * 10) + (val % 16)); }

void DS3231_SetTime(RTC_Time *time) {
    uint8_t buf[7];
    buf[0] = decToBcd(time->seconds);
    buf[1] = decToBcd(time->minutes);
    buf[2] = decToBcd(time->hour);
    buf[3] = decToBcd(time->day_of_week);
    buf[4] = decToBcd(time->day_of_month);
    buf[5] = decToBcd(time->month);
    buf[6] = decToBcd(time->year);
    HAL_I2C_Mem_Write(&hi2c1, DS3231_I2C_ADDR, DS3231_REG_SECONDS, 1, buf, 7, HAL_MAX_DELAY);
}

void DS3231_GetTime(RTC_Time *time) {
    uint8_t buf[7];
    HAL_I2C_Mem_Read(&hi2c1, DS3231_I2C_ADDR, DS3231_REG_SECONDS, 1, buf, 7, HAL_MAX_DELAY);
    time->seconds = bcdToDec(buf[0]);
    time->minutes = bcdToDec(buf[1]);
    time->hour = bcdToDec(buf[2]);
    time->day_of_week = bcdToDec(buf[3]);
    time->day_of_month = bcdToDec(buf[4]);
    time->month = bcdToDec(buf[5]);
    time->year = bcdToDec(buf[6]);
}

// -- Part 3: Circular Buffer Management Functions --

/**
 * @brief Load metadata from SD card
 * @param metadata Pointer to metadata structure
 * @return true if metadata loaded successfully, false otherwise
 */
bool load_log_metadata(LogMetadata *metadata) {
    FIL metaFile;
    FRESULT fres;
    UINT bytesRead;

    fres = f_open(&metaFile, METADATA_FILE, FA_READ);
    if (fres != FR_OK) {
        // File doesn't exist, initialize with defaults
        metadata->current_position = 0;
        metadata->total_entries_written = 0;
        metadata->buffer_full = 0;
        DEBUG_PRINTF("LOG_META: No metadata file found, starting fresh\r\n");
        return false;
    }

    fres = f_read(&metaFile, metadata, sizeof(LogMetadata), &bytesRead);
    f_close(&metaFile);

    if (fres == FR_OK && bytesRead == sizeof(LogMetadata)) {
        DEBUG_PRINTF("LOG_META: Loaded - Pos:%lu, Total:%lu, Full:%d\r\n",
                    metadata->current_position, metadata->total_entries_written, metadata->buffer_full);
        return true;
    } else {
        // Corrupted metadata, reset
        metadata->current_position = 0;
        metadata->total_entries_written = 0;
        metadata->buffer_full = 0;
        DEBUG_PRINTF("LOG_META: Corrupted metadata, resetting\r\n");
        return false;
    }
}

/**
 * @brief Save metadata to SD card
 * @param metadata Pointer to metadata structure
 * @return true if saved successfully, false otherwise
 */
bool save_log_metadata(LogMetadata *metadata) {
    FIL metaFile;
    FRESULT fres;
    UINT bytesWritten;

    DEBUG_PRINTF("LOG_META: Opening metadata file for writing...\r\n");
    fres = f_open(&metaFile, METADATA_FILE, FA_WRITE | FA_CREATE_ALWAYS);
    if (fres != FR_OK) {
        DEBUG_PRINTF("LOG_META: Failed to open metadata file for writing, error: %d\r\n", fres);
        return false;
    }

    DEBUG_PRINTF("LOG_META: Writing metadata (size: %d bytes)...\r\n", sizeof(LogMetadata));
    fres = f_write(&metaFile, metadata, sizeof(LogMetadata), &bytesWritten);
    if (fres != FR_OK) {
        DEBUG_PRINTF("LOG_META: Write failed, error: %d\r\n", fres);
        f_close(&metaFile);
        return false;
    }

    DEBUG_PRINTF("LOG_META: Wrote %d bytes, syncing to SD card...\r\n", bytesWritten);
    f_sync(&metaFile);  // Ensure data is written to SD card
    f_close(&metaFile);

    if (bytesWritten == sizeof(LogMetadata)) {
        DEBUG_PRINTF("LOG_META: Metadata saved successfully (%d bytes)\r\n", bytesWritten);
        return true;
    } else {
        DEBUG_PRINTF("LOG_META: Size mismatch - expected: %d, written: %d\r\n",
                    sizeof(LogMetadata), bytesWritten);
        return false;
    }
}

/**
 * @brief Initialize or open the circular log file
 * @param logFile Pointer to FIL structure
 * @param metadata Pointer to metadata structure
 * @return true if successful, false otherwise
 */
bool init_circular_log_file(FIL *logFile, LogMetadata *metadata) {
    FRESULT fres;

    // Try to open existing file
    fres = f_open(logFile, DATALOG_FILE, FA_WRITE | FA_READ);

    if (fres != FR_OK) {
        // File doesn't exist, create it
        DEBUG_PRINTF("LOG_INIT: Creating new log file\r\n");
        fres = f_open(logFile, DATALOG_FILE, FA_WRITE | FA_CREATE_NEW);
        if (fres != FR_OK) {
            DEBUG_PRINTF("LOG_INIT: Failed to create log file! Error: %d\r\n", fres);
            return false;
        }

        // Pre-allocate space for the circular buffer by writing dummy data
        // This prevents fragmentation and ensures consistent performance
        char dummy_entry[MAX_LOG_ENTRY_SIZE];
        memset(dummy_entry, ' ', MAX_LOG_ENTRY_SIZE - 2);
        dummy_entry[MAX_LOG_ENTRY_SIZE - 2] = '\r';
        dummy_entry[MAX_LOG_ENTRY_SIZE - 1] = '\n';

        DEBUG_PRINTF("LOG_INIT: Pre-allocating space for %lu entries...\r\n", (unsigned long)TOTAL_LOG_ENTRIES);
        for (uint32_t i = 0; i < TOTAL_LOG_ENTRIES; i++) {
            f_write(logFile, dummy_entry, MAX_LOG_ENTRY_SIZE, NULL);
            if (i % 1000 == 0) {
                f_sync(logFile);  // Periodic sync to prevent timeout
                DEBUG_PRINTF("LOG_INIT: Allocated %lu/%lu entries\r\n", (unsigned long)i, (unsigned long)TOTAL_LOG_ENTRIES);
            }
        }
        f_sync(logFile);
        DEBUG_PRINTF("LOG_INIT: Pre-allocation complete\r\n");

        // Reset metadata for new file
        metadata->current_position = 0;
        metadata->total_entries_written = 0;
        metadata->buffer_full = 0;
    } else {
        DEBUG_PRINTF("LOG_INIT: Opened existing log file\r\n");

        // Validate file size
        FSIZE_t file_size = f_size(logFile);
        FSIZE_t expected_size = TOTAL_LOG_ENTRIES * MAX_LOG_ENTRY_SIZE;

        if (file_size != expected_size) {
            DEBUG_PRINTF("LOG_INIT: File size mismatch! Expected: %lu, Actual: %lu\r\n",
                        (uint32_t)expected_size, (uint32_t)file_size);
            f_close(logFile);
            // Delete corrupted file and recreate
            f_unlink(DATALOG_FILE);
            return init_circular_log_file(logFile, metadata);
        }
    }

    return true;
}

/**
 * @brief Write a log entry to the circular buffer
 * @param logFile Pointer to FIL structure
 * @param metadata Pointer to metadata structure
 * @param log_entry String containing the log entry
 * @return true if successful, false otherwise
 */
bool write_circular_log_entry(FIL *logFile, LogMetadata *metadata, const char *log_entry) {
    FRESULT fres;
    UINT bytesWritten;
    char padded_entry[MAX_LOG_ENTRY_SIZE];

    // Calculate file position for this entry
    FSIZE_t file_position = metadata->current_position * MAX_LOG_ENTRY_SIZE;

    // Prepare padded entry (pad with spaces and ensure it ends with \r\n)
    memset(padded_entry, ' ', MAX_LOG_ENTRY_SIZE);
    size_t entry_len = strlen(log_entry);
    if (entry_len > MAX_LOG_ENTRY_SIZE - 2) {
        entry_len = MAX_LOG_ENTRY_SIZE - 2;  // Leave space for \r\n
    }

    memcpy(padded_entry, log_entry, entry_len);
    padded_entry[MAX_LOG_ENTRY_SIZE - 2] = '\r';
    padded_entry[MAX_LOG_ENTRY_SIZE - 1] = '\n';

    // Seek to the correct position
    fres = f_lseek(logFile, file_position);
    if (fres != FR_OK) {
        DEBUG_PRINTF("LOG_WRITE: Seek failed at position %lu\r\n", (uint32_t)file_position);
        return false;
    }

    // Write the entry
    fres = f_write(logFile, padded_entry, MAX_LOG_ENTRY_SIZE, &bytesWritten);
    if (fres != FR_OK || bytesWritten != MAX_LOG_ENTRY_SIZE) {
        DEBUG_PRINTF("LOG_WRITE: Write failed! Error: %d, Bytes: %d\r\n", fres, bytesWritten);
        return false;
    }

    // Force write to SD card
    f_sync(logFile);

    // Update metadata
    metadata->total_entries_written++;
    metadata->current_position++;

    // Handle circular buffer wraparound
    if (metadata->current_position >= TOTAL_LOG_ENTRIES) {
        metadata->current_position = 0;
        metadata->buffer_full = 1;
        DEBUG_PRINTF("LOG_CIRCULAR: Buffer wrapped around - starting overwrite cycle\r\n");
    }

    return true;
}

// -- Part 4: Main Circular Logging Task --

void RTC_SD_Log_Task_Circular(void *pvParameters) {
    RTC_Time currentTime;
    gps_data_t localGpsData;
    char logBuffer[MAX_LOG_ENTRY_SIZE];
    FATFS FatFs;
    FRESULT fres;

    // Variables to prevent duplicate logging
    static uint8_t lastLoggedSecond = 255;
    static uint32_t lastLoggedMinute = 0;
    static uint32_t lastMetadataSave = 0;

    vTaskDelay(pdMS_TO_TICKS(1000));
    DEBUG_PRINTF("\r\n--- Circular Buffer RTC+GPS Logging Task Started ---\r\n");
    DEBUG_PRINTF("LOG_CONFIG: 72 hours storage, %lu total entries, %d bytes per entry\r\n",
                (unsigned long)TOTAL_LOG_ENTRIES, MAX_LOG_ENTRY_SIZE);

    // Mount the SD card
    fres = f_mount(&FatFs, "", 1);
    if (fres != FR_OK) {
        DEBUG_PRINTF("LOG_TASK: SD card mount failed (%d)!\r\n", fres);
        DEBUG_PRINTF("LOG_TASK: Task will exit!\r\n");
        vTaskDelete(NULL);
        return;
    }
    DEBUG_PRINTF("LOG_TASK: SD card mounted successfully\r\n");

    // Load metadata
    load_log_metadata(&logMeta);

    // Initialize circular log file
    if (!init_circular_log_file(&circularLogFile, &logMeta)) {
        DEBUG_PRINTF("LOG_TASK: Failed to initialize circular log file!\r\n");
        vTaskDelete(NULL);
        return;
    }

    // Set the time once when first programming the board
    RTC_Time initial_time = {
        .seconds = 00, .minutes = 25, .hour = 17,
        .day_of_week = 4, .day_of_month = 14, .month = 8, .year = 25
    };
   // DS3231_SetTime(&initial_time);
    DEBUG_PRINTF("RTC time set.\r\n");

    // Save initial metadata immediately after initialization
    if (save_log_metadata(&logMeta)) {
        DEBUG_PRINTF("LOG_META: Initial metadata saved successfully\r\n");
    } else {
        DEBUG_PRINTF("LOG_META: Failed to save initial metadata\r\n");
    }

    DEBUG_PRINTF("LOG_TASK: Circular logging initialized successfully\r\n");
    DEBUG_PRINTF("LOG_STATUS: Position: %lu/%lu, Total written: %lu, Buffer full: %s\r\n",
                logMeta.current_position, TOTAL_LOG_ENTRIES, logMeta.total_entries_written,
                logMeta.buffer_full ? "YES" : "NO");

    while (1) {
        // Read current time from RTC
        DS3231_GetTime(&currentTime);

        // Check if we need to log (every 5 seconds)
        bool shouldLog = (currentTime.seconds % LOG_INTERVAL_SECONDS == 0);

        // Prevent duplicate logging
        bool isDuplicate = (currentTime.seconds == lastLoggedSecond) &&
                          (currentTime.minutes == lastLoggedMinute);

        if (shouldLog && !isDuplicate) {
            DEBUG_PRINTF("LOG_CIRCULAR: Logging at %02d:%02d:%02d [Pos: %lu/%lu]\r\n",
                        currentTime.hour, currentTime.minutes, currentTime.seconds,
                        logMeta.current_position, TOTAL_LOG_ENTRIES);

            // Get GPS data with minimal mutex hold time
            bool gpsDataValid = false;
            if (xSemaphoreTake(g_gps_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                localGpsData = g_gps_data;
                gpsDataValid = true;
                xSemaphoreGive(g_gps_data_mutex);
            } else {
                DEBUG_PRINTF("LOG_CIRCULAR: Failed to get GPS mutex\r\n");
                localGpsData.is_valid = false;
                gpsDataValid = true;  // Continue with invalid GPS data
            }

            // Format log entry
            if (gpsDataValid && localGpsData.is_valid) {
                char lat_dir = (localGpsData.latitude >= 0) ? 'N' : 'S';
                float lat_val = (localGpsData.latitude >= 0) ? localGpsData.latitude : -localGpsData.latitude;
                char lon_dir = (localGpsData.longitude >= 0) ? 'E' : 'W';
                float lon_val = (localGpsData.longitude >= 0) ? localGpsData.longitude : -localGpsData.longitude;

                snprintf(logBuffer, sizeof(logBuffer),
                         "20%02d/%02d/%02d\t%02d:%02d:%02d\t%.1f\t%.4f° %c\t%.4f° %c",
                         currentTime.year, currentTime.month, currentTime.day_of_month,
                         currentTime.hour, currentTime.minutes, currentTime.seconds,
                         localGpsData.speed_knots, lat_val, lat_dir, lon_val, lon_dir);
            } else {
                snprintf(logBuffer, sizeof(logBuffer),
                         "20%02d/%02d/%02d\t%02d:%02d:%02d\tGPS NOT FIXED",
                         currentTime.year, currentTime.month, currentTime.day_of_month,
                         currentTime.hour, currentTime.minutes, currentTime.seconds);
            }

            // Write to circular buffer
            if (write_circular_log_entry(&circularLogFile, &logMeta, logBuffer)) {
                DEBUG_PRINTF("LOG_SUCCESS: Entry %lu written\r\n", logMeta.total_entries_written);
            } else {
                DEBUG_PRINTF("LOG_ERROR: Failed to write entry!\r\n");
            }

            // Update last logged time
            lastLoggedSecond = currentTime.seconds;
            lastLoggedMinute = currentTime.minutes;

            // Save metadata periodically (every 2 minutes for testing, change back to 10 minutes later)
            if ((HAL_GetTick() - lastMetadataSave) > 120000) {  // 2 minutes for testing
                DEBUG_PRINTF("LOG_META: Attempting to save metadata...\r\n");
                if (save_log_metadata(&logMeta)) {
                    DEBUG_PRINTF("LOG_META: Metadata saved successfully\r\n");
                } else {
                    DEBUG_PRINTF("LOG_META: Failed to save metadata\r\n");
                }
                lastMetadataSave = HAL_GetTick();
            }

            // Status report every hour
            if (logMeta.total_entries_written % 720 == 0) {  // 720 entries = 1 hour
                float hours_stored = logMeta.buffer_full ? HOURS_TO_STORE :
                                   (logMeta.total_entries_written * LOG_INTERVAL_SECONDS) / 3600.0f;
                DEBUG_PRINTF("LOG_STATUS: %.1f hours of data stored, Position: %lu/%lu\r\n",
                            hours_stored, logMeta.current_position, TOTAL_LOG_ENTRIES);
            }
        }

        // Sleep for 500ms to check twice per second
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

//RTC - CIRCULAR BUFFER END ******************************************************************
//RTC - END **************************************************************************








/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */







  DEBUG_PRINTF("System Initialized. Starting FreeRTOS tasks.\r\n");

  g_modem_mutex = xSemaphoreCreateMutex();
  g_gps_data_mutex = xSemaphoreCreateMutex();
  g_gps_data_ready_sem = xSemaphoreCreateBinary();


  if (g_modem_mutex == NULL || g_gps_data_mutex == NULL || g_gps_data_ready_sem == NULL)
  {
      DEBUG_PRINTF("FATAL ERROR: Failed to create RTOS objects!\r\n");
      Error_Handler();
  }

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */


  /* USER CODE BEGIN RTOS_THREADS */
  //xTaskCreate(vLedHeartbeatTask, "LED_Heartbeat", 256, NULL, 1, NULL);
    xTaskCreate(vGpsTask, "GPS_Task", 1024, NULL, 2, NULL);  // Higher priority
    xTaskCreate(vMqttTask, "MQTT_Task", 1536, NULL, 3, NULL); // Higher priority, larger stack
    xTaskCreate(vSmsTask, "SMS_Task", 1024, NULL, 1, NULL);
    //xTaskCreate(SD_Task, "SDTask", 1024, NULL, 1, NULL);
   // xTaskCreate(SD_Test_Task, "SD_Test", 1024, NULL, 1, NULL);
    //xTaskCreate(RTC_SD_Log_Task_Critical, "Crit_Log_Task", 2048, NULL, 2, NULL);
    xTaskCreate(RTC_SD_Log_Task_Circular, "RTC_Circular_Log", 2048, NULL, 1, NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}




int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */
  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */
  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */
  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */
  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  // REMOVED INTERRUPT CONFIGURATION - Now just a regular input pin
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 SD_CS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // REMOVED EXTI interrupt configuration

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
