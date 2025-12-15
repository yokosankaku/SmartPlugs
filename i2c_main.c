/* ======================================== INCLUDES LIBRARY ====================================== */
#include "iot_config.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Demo includes */
#include "aws_demo.h"
#include "aws_dev_mode_key_provisioning.h"

/* AWS System includes. */
#include "bt_hal_manager.h"
#include "iot_system_init.h"
#include "iot_logging_task.h"
#include "nvs_flash.h"
#include "FreeRTOS_Sockets.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_interface.h"
#include "driver/uart.h"
#include "aws_application_version.h"
#include "iot_network_manager_private.h"
#include "aws_iot_ota_agent.h"
#include "iot_init.h"
#include "platform/iot_network.h"
#include "platform/iot_network_freertos.h"
#include "types/iot_mqtt_types.h"
#include "iot_wifi.h"
#include "iot_mqtt.h"
#include "aws_iot_network_config.h"
#include "aws_clientcredential.h"
#include "iot_default_root_certificates.h"
#include "iot_error.h"
#include "iot_config_common.h"
#include "driver/gpio.h"
#include "esp_image_format.h"
#include "esp_ota_ops.h"
#include "esp_spi_flash.h"
#include "esp_partition.h"
#include "projdefs.h"
#include "driver/adc.h"
#include "esp_timer.h"
#include "driver/i2c.h"

#include "ws2812.h"
#include "cJSON.h"

/* Standart library C */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <math.h>

/* ======================================== DEFINE =============================================== */
/* Logging Task Defines. */
#define mainLOGGING_MESSAGE_QUEUE_LENGTH    ( 32 )
#define mainLOGGING_TASK_STACK_SIZE         ( configMINIMAL_STACK_SIZE * 4 )
#define mainDEVICE_NICK_NAME                "Espressif_Demo"

#define MY_WIFI_SSID clientcredentialWIFI_SSID
#define MY_WIFI_PASSWORD clientcredentialWIFI_PASSWORD

#define RELAY_PIN GPIO_NUM_13

#define WIFI_INDICATOR GPIO_NUM_23

#define SENSOR_BUTTON GPIO_NUM_19

#define WS2812_PIN  17

#define I2C_SCL_PIN GPIO_NUM_22
#define I2C_SDA_PIN GPIO_NUM_21

#define ADC_SENSOR_RD_PIN GPIO_NUM_18

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */

/* =================================== GLOBAL VARIABLE ============================================ */
static IotMqttNetworkInfo_t my_networkInfo = IOT_MQTT_NETWORK_INFO_INITIALIZER;
static IotMqttConnectInfo_t my_connectInfo = IOT_MQTT_CONNECT_INFO_INITIALIZER;
static IotMqttSerializer_t my_mqttSerializer = IOT_MQTT_SERIALIZER_INITIALIZER;
static IotNetworkCredentials_t my_Credentials = AWS_IOT_NETWORK_CREDENTIALS_AFR_INITIALIZER;
static IotNetworkServerInfo_t my_serverInfo = AWS_IOT_NETWORK_SERVER_INFO_AFR_INITIALIZER;
static IotMqttConnection_t pub_sub_InfoData_mqttConnection = IOT_MQTT_CONNECTION_INITIALIZER;

static char buffer_json_tx[ 2048 ] = { 0 };
static char buffer_json_rx[ 2048 ] = { 0 };
static char temp_buffer_rx[ 2048 ] = { 0 };
bool flag_accepted_comand = false;
__NOINIT_ATTR int state_relay_module;
__NOINIT_ATTR uint32_t cur_calibr_value;
__NOINIT_ATTR int check_reset_reason;
static bool flag_wifi_connection = false;
bool flag_reset_reason = false;

float current_value = 0.00;
float voltage_value = 0.00;
float watts_value = 0.00;

int64_t time_tr_server = 0;
int delta_time = 0;

/* =================================== Prototype func ============================================ */
static void prvMiscInitialization( void );
static void my_init( void );
bool iot_netwokr_init();
static void reset_iot_module();
void task_take_action_function( void * pvParameters );
void MQTT_pub_sub_and_ota_function( void * pvParameters );
void clear_char_massiv(char* massiv_inp, int size_massiv);
void create_json_servers_aws();
bool msg_search_existence(char* input_msg, char* search_msg, int start_add_scan);
int msg_search_start_add(char* input_msg, char* search_msg, int start_add_scan);
void task_measure_and_current_function( void * pvParameters );
void delay_us(uint32_t value_time);
float map_float(float map, float min_1, float max_1, float min_2, float max_2);

#define BH1750_SENSOR_ADDR 0x23

/* =================================== FUNC MAIN ================================================= */

int app_main( void )
{
  prvMiscInitialization();
  my_init();

  if(check_reset_reason != 0x696f74)
  {
    state_relay_module = 1;
    flag_reset_reason = true;
    check_reset_reason = 0x696f74;
  }

  i2c_config_t i2c_config = 
  {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_SDA_PIN,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = I2C_SCL_PIN,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000
  };

  i2c_param_config(I2C_NUM_0, &i2c_config);
  i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

  uint8_t data_H;
  uint8_t data_L;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, 0x10, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  while(1)
  {
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data_H, ACK_VAL);
    i2c_master_read_byte(cmd, &data_L, NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    printf("%d\n", (uint32_t)(data_H << 8 | data_L));
    delay_us(30000);
  }


  /* ------------------- ACTION TASK */
  /*if(xTaskCreatePinnedToCore(task_take_action_function, "task_take_action_function", 10000, NULL, 1, NULL, 1) != pdPASS)
  {
    reset_iot_module();
    printf("Create action task error\n");
  }
  printf("Create action task ok\n");*/

  /* ------------------- Measure task */
  /*if(xTaskCreatePinnedToCore(task_measure_and_current_function, "task_measure_and_current_function", 10000, NULL, 1, NULL, 1) != pdPASS)
  {
    reset_iot_module();
    printf("Create measure task error\n");
  }
  printf("Create measure task ok\n");*/
  /*
  if( SYSTEM_Init() == pdPASS )
  {
    vDevModeKeyProvisioning();
    
    if(iot_netwokr_init() == true)
    {
      if(xTaskCreatePinnedToCore(MQTT_pub_sub_and_ota_function, "MQTT_pub_sub_and_ota_function", ( 6 * configMINIMAL_STACK_SIZE ), NULL, ( tskIDLE_PRIORITY + 5 ), NULL, 0) != pdPASS)
      {
        reset_iot_module();
        printf("Create PUB-SUB and ota task error\n");
      }
      printf("Create PUB-SUB and ota task ok\n");
      flag_wifi_connection = true;
    }
    else
    {
      reset_iot_module();
    }
  }*/

  /* vTaskStartScheduler(); */
  return 0;
}

/* =================================== MY FUNCTIONS ============================================== */

float map_float(float map, float min_1, float max_1, float min_2, float max_2)
{
    float Out = map;
    Out = min_2 + (Out - min_1) * ((max_2 - min_2) / (max_1 - min_1));
    return Out;
}

int msg_search_start_add(char* input_msg, char* search_msg, int start_add_scan)
{
  bool flag_return = false;
  int return_add = 0;
  if(strlen(input_msg) <= 1) return flag_return;
  if(strlen(input_msg) < strlen(search_msg)) return flag_return;
  if(start_add_scan >= strlen(input_msg)) return flag_return;
  
  for(int i = start_add_scan; i < strlen(input_msg); i += 1)
  {
    if(input_msg[i] == search_msg[0] && i <= (strlen(input_msg) - strlen(search_msg)))
    {
      flag_return = true;
      if(strlen(search_msg) > 1)
      {
        for(int y = 1; y < strlen(search_msg); y += 1)
        {
          if(search_msg[y] != input_msg[y + i])
          {
            flag_return = false;
            break;
          }
        }
      }
      if(flag_return == true) 
      {
        return_add = i;
        break;
      }
    }
  }
  return return_add;
}

bool msg_search_existence(char* input_msg, char* search_msg, int start_add_scan)
{
  bool flag_return = false;
  if(strlen(input_msg) <= 1) return flag_return;
  if(strlen(input_msg) < strlen(search_msg)) return flag_return;
  if(start_add_scan >= strlen(input_msg)) return flag_return;
  
  for(int i = start_add_scan; i < strlen(input_msg); i += 1)
  {
    if(input_msg[i] == search_msg[0] && i <= (strlen(input_msg) - strlen(search_msg)))
    {
      flag_return = true;
      if(strlen(search_msg) > 1)
      {
        for(int y = 1; y < strlen(search_msg); y += 1)
        {
          if(search_msg[y] != input_msg[y + i])
          {
            flag_return = false;
            break;
          }
        }
      }
      if(flag_return == true) break;
    }
  }
  return flag_return;
}

void clear_char_massiv(char* massiv_inp, int size_massiv)
{
  for(int i = 0; i < size_massiv; i += 1)
  {
    massiv_inp[i] = 0x00;
  }
}

void create_json_servers_aws()
{
  cJSON *json_obj_main;
  cJSON *json_obj_state;
  cJSON *json_obj_reported;
  cJSON *json_obj_desired;
  json_obj_state = cJSON_CreateObject();
  json_obj_reported = cJSON_CreateObject();
  json_obj_main = cJSON_CreateObject();
  json_obj_desired = cJSON_CreateObject();
  if(flag_accepted_comand == true)
  {
    json_obj_desired = cJSON_CreateNull();
    flag_accepted_comand = false;
  }
  delta_time = (esp_timer_get_time() - time_tr_server) / 1000;
  time_tr_server = esp_timer_get_time();
  cJSON_AddNumberToObject(json_obj_reported, "stateRelay", state_relay_module);
  cJSON_AddNumberToObject(json_obj_reported, "deltaTime", delta_time);
  if(state_relay_module == 1) cJSON_AddNumberToObject(json_obj_reported, "wattsValue", watts_value);
  else if(state_relay_module == 0) cJSON_AddNumberToObject(json_obj_reported, "wattsValue", 0);
  cJSON_AddItemToObject(json_obj_state, "reported", json_obj_reported);
  cJSON_AddItemToObject(json_obj_state, "desired", json_obj_desired);
  cJSON_AddItemToObject(json_obj_main, "state", json_obj_state);
  char *msg_json = cJSON_Print(json_obj_main);
  clear_char_massiv(buffer_json_tx, sizeof(buffer_json_tx));
  for(int i = 0; i < strlen(msg_json); i++)
  {
    buffer_json_tx[i] = msg_json[i];
  }
  cJSON_Delete(json_obj_main);
}

void delay_us(uint32_t value_time)
{
  int64_t timers = esp_timer_get_time();
  while((esp_timer_get_time() - timers) <= value_time);
}

/* -------------------------------------- Task functions */

/* -------- OTA update app */
static void App_OTACompleteCallback_iot( OTA_JobEvent_t eEvent )
{
    OTA_Err_t xErr = kOTA_Err_Uninitialized;

    if( eEvent == eOTA_JobEvent_Activate )
    {
        configPRINTF( ( "Received eOTA_JobEvent_Activate callback from OTA Agent.\r\n" ) );
        OTA_SetImageState( eOTA_ImageState_Accepted );
        IotMqtt_Disconnect(pub_sub_InfoData_mqttConnection , 0 );
        OTA_ActivateNewImage();
    }
    else if( eEvent == eOTA_JobEvent_Fail )
    {
        configPRINTF( ( "Received eOTA_JobEvent_Fail callback from OTA Agent.\r\n" ) );
    }
    else if( eEvent == eOTA_JobEvent_StartTest )
    {
        configPRINTF( ( "Received eOTA_JobEvent_StartTest callback from OTA Agent.\r\n" ) );
        xErr = OTA_SetImageState( eOTA_ImageState_Accepted );

        if( xErr != kOTA_Err_None )
        {
            OTA_LOG_L1( " Error! Failed to set image state as accepted.\r\n" );
        }
    }
}

/* -------- take action */

void task_measure_and_current_function( void * pvParameters )
{
  int cur_analog_value = 0;
  uint32_t sum_current = 0;
  int cur_calibr_sq_value;
  int64_t time_measure_cur;
  int numer_measure;
  float primary_settlement_cur = 0.00;
  float coefficient_increase = 0.00;

  adc1_config_channel_atten( ADC1_CHANNEL_5, ADC_ATTEN_DB_2_5 ); // current GPIO 33

  adc1_config_width(ADC_WIDTH_BIT_12);

  if(flag_reset_reason == true)
  {
    cur_calibr_value = 0;
    for(int i = 0; i < 50; i += 1)
    {
      cur_analog_value = adc1_get_raw(ADC1_CHANNEL_5);
      cur_calibr_value += cur_analog_value;
      delay_us(10000);
    }
    cur_calibr_value /= 50;
    flag_reset_reason = false;
  }


  while(true)
  {
    time_measure_cur = esp_timer_get_time();
    numer_measure = 0;
    sum_current = 0;
    while((esp_timer_get_time() - time_measure_cur) <= 100000)
    {
      cur_analog_value = adc1_get_raw(ADC1_CHANNEL_5);
      cur_calibr_sq_value = cur_calibr_value - cur_analog_value;
      sum_current += cur_calibr_sq_value * cur_calibr_sq_value;
      numer_measure += 1;
    }
    primary_settlement_cur = sqrt(sum_current / numer_measure);

    // 5.2385 ====== 0.01469136082918898696217097283255
    // 3.52 ======== 0.01332390061732811127273911007699
    // 0.725 ======= 0.01179694096371407404345927964461
    // 0 =========== 0.011395

    if(primary_settlement_cur >= 0 && primary_settlement_cur < 61.45)
    {
      coefficient_increase = map_float(primary_settlement_cur, 0.00, 61.45, 0.011395, 0.01179694096371407404345927964461);
      current_value = primary_settlement_cur * coefficient_increase;
    }
    else if(primary_settlement_cur >= 61.45 && primary_settlement_cur < 264.1869)
    {
      coefficient_increase = map_float(primary_settlement_cur, 61.45, 264.1869, 0.01179694096371407404345927964461, 0.01332390061732811127273911007699);
      current_value = primary_settlement_cur * coefficient_increase;
    }
    else if(primary_settlement_cur >= 264.1869 && primary_settlement_cur < 356.5701)
    {
      coefficient_increase = map_float(primary_settlement_cur, 264.1869, 356.5701, 0.01332390061732811127273911007699, 0.01469136082918898696217097283255);
      current_value = primary_settlement_cur * coefficient_increase;
    }
    else if(primary_settlement_cur >= 356.5701 && primary_settlement_cur < 1390)
    {
      coefficient_increase = 0.01469136082918898696217097283255;
      current_value = primary_settlement_cur * coefficient_increase;
    }
    if(current_value < 0.318) current_value = 0;

    watts_value = current_value * 220;
    /*
    printf("Current:");
    printf("%f", current_value);
    printf(" || ");
    printf("Watts:");
    printf("%f\n", watts_value);
    */
    vTaskDelay(pdMS_TO_TICKS( 100UL ));

  }
}

void task_take_action_function( void * pvParameters )
{
  uint8_t color_R_led = 0;
  uint8_t color_G_led = 0;
  uint8_t color_B_led = 0;

  uint8_t color_R_led_old = 0;
  uint8_t color_G_led_old = 0;
  uint8_t color_B_led_old = 0;

  int64_t timer_toogle = 0;
  uint8_t toogle_switch = 0;

  ws2812_init(WS2812_PIN);
  rgbVal color_RGB;
  rgbVal *pixels;
  pixels = malloc(sizeof(rgbVal) * 2);

  color_RGB.r = 0x00;
  color_RGB.g = 0x96;
  color_RGB.b = 0x00;
  pixels[0] = color_RGB;
  ws2812_setColors(1, pixels); 

  while(true)
  {
    if(gpio_get_level(SENSOR_BUTTON) == 1)
    {
      while((bool)gpio_get_level(SENSOR_BUTTON));
      if(state_relay_module == 1)
      {
        state_relay_module = 0;
      }
      else if(state_relay_module == 0)
      {
        state_relay_module = 1;
      }
    }

    if(state_relay_module == 0)
    {
      gpio_set_level(RELAY_PIN, 1);
    }
    else if(state_relay_module == 1)
    {
      gpio_set_level(RELAY_PIN, 0);
    }

    if(flag_wifi_connection == true)
    {
      gpio_set_level(WIFI_INDICATOR, 0);
    }
    else
    {
      if((esp_timer_get_time() - timer_toogle) >= 1000000)
      {
        switch (toogle_switch)
        {
          case 0:
                  gpio_set_level(WIFI_INDICATOR, 1);
                  toogle_switch = 1;
                  break;
          case 1:
                  gpio_set_level(WIFI_INDICATOR, 0);
                  toogle_switch = 0;
                  break;
        }
        timer_toogle = esp_timer_get_time();
      }
    }

    if(state_relay_module == 1)
    {
      color_R_led = 0x00;
      color_G_led = 0x96;
      color_B_led = 0x00;
    }
    else if(state_relay_module == 0)
    {
      color_R_led = 0x00;
      color_G_led = 0x00;
      color_B_led = 0x96;
    }

    if(color_R_led != color_R_led_old || color_G_led != color_G_led_old || color_B_led != color_B_led_old)
    {
      color_RGB.r = color_R_led;
      color_RGB.g = color_G_led;
      color_RGB.b = color_B_led;
      pixels[0] = color_RGB;
      ws2812_setColors(1, pixels);
      color_R_led_old = color_R_led;
      color_G_led_old = color_G_led;
      color_B_led_old = color_B_led;
    }

    vTaskDelay(pdMS_TO_TICKS( 10UL ));
  }
}

/* -------- mqtt pub-sub func */
static void func_MQTT_SUB( void * pvParameter, IotMqttCallbackParam_t * const pxPublish )
{
  clear_char_massiv(buffer_json_rx, sizeof(buffer_json_rx));
  strcpy (buffer_json_rx, (char*)pxPublish->u.message.info.pPayload);
  if(msg_search_existence(buffer_json_rx, "desired", 0) != true) return;
  int add_desired = msg_search_start_add(buffer_json_rx, "desired", 0);
  int add_reported = msg_search_start_add(buffer_json_rx, "reported", add_desired);
  clear_char_massiv(temp_buffer_rx, sizeof(temp_buffer_rx));
  for(int i = 0; i < add_reported; i += 1)
  {
    temp_buffer_rx[i] = buffer_json_rx[add_desired + i];
  }
  if(msg_search_existence(temp_buffer_rx, "stateRelay", 0) != true) return;

  cJSON *json_processing = cJSON_Parse(buffer_json_rx);
  if(json_processing != NULL)
  {
    cJSON *state = cJSON_GetObjectItem(json_processing, "state");
    cJSON *desired = cJSON_GetObjectItem(state, "desired");

    state_relay_module = cJSON_GetObjectItem(desired,"stateRelay")->valueint;
    if(state_relay_module > 1) state_relay_module = 1;
    else if(state_relay_module < 0) state_relay_module = 0;

    cJSON_Delete(json_processing);
    flag_accepted_comand = true;
  }

}

void MQTT_pub_sub_and_ota_function( void * pvParameters )
{
  IotMqttError_t MQTT_Status;
  uint8_t sequence_operations = 0;
  uint8_t sequence_pub_msg = 0;
  OTA_State_t eState;

  IotMqttSubscription_t my_Subscription = IOT_MQTT_SUBSCRIPTION_INITIALIZER;
  my_Subscription.pTopicFilter = ( const char * ) "$aws/things/"clientcredentialIOT_THING_NAME"/shadow/get/accepted";
  my_Subscription.topicFilterLength = strlen("$aws/things/"clientcredentialIOT_THING_NAME"/shadow/get/accepted");
  my_Subscription.qos = IOT_MQTT_QOS_0;
  my_Subscription.callback.function = func_MQTT_SUB;

  IotMqttPublishInfo_t my_PublishInfo = IOT_MQTT_PUBLISH_INFO_INITIALIZER;
  my_PublishInfo.qos = IOT_MQTT_QOS_0;

  while(true)
  {
    switch(sequence_operations)
    {
      case 0:
            MQTT_Status = IotMqtt_Connect( &my_networkInfo, &my_connectInfo, 15000, &pub_sub_InfoData_mqttConnection );
            if(MQTT_Status == IOT_MQTT_SUCCESS)
            {
              printf("MQTT connect ok\n");
              sequence_operations = 1;
            }
            else
            {
              printf("MQTT connect error\n");
              reset_iot_module();
            }
            break;
      case 1:
            MQTT_Status = IotMqtt_TimedSubscribe( pub_sub_InfoData_mqttConnection, &my_Subscription, 1, 0, 5000 );
            eState = OTA_AgentInit( pub_sub_InfoData_mqttConnection, ( const uint8_t * ) ( clientcredentialIOT_THING_NAME ), App_OTACompleteCallback_iot, ( TickType_t ) ~0 );

            if(MQTT_Status == IOT_MQTT_SUCCESS && eState == eOTA_AgentState_Ready)
            {
              printf("MQTT sub ok\n");
              sequence_operations = 2;
            }
            else
            {
              printf("MQTT sub error\n");
              sequence_operations = 0;
            }
            break;
      case 2:
            eState = OTA_GetAgentState();
            if(eState == eOTA_AgentState_Active)
            {
              printf("Update app come\n");
              sequence_operations = 3;
            }

            switch(sequence_pub_msg)
            {
              case 0:
                      create_json_servers_aws();
                      my_PublishInfo.pPayload = ( const void * )buffer_json_tx;
                      my_PublishInfo.payloadLength = strlen(buffer_json_tx);
                      my_PublishInfo.pTopicName = ( const char * ) "$aws/things/"clientcredentialIOT_THING_NAME"/shadow/update";
                      my_PublishInfo.topicNameLength = strlen("$aws/things/"clientcredentialIOT_THING_NAME"/shadow/update");
                      MQTT_Status = IotMqtt_TimedPublish( pub_sub_InfoData_mqttConnection, &my_PublishInfo, 0, 5000 );
                      sequence_pub_msg = 1;
                      break;
              case 1:
                      my_PublishInfo.pPayload = NULL;
                      my_PublishInfo.payloadLength = 0;
                      my_PublishInfo.pTopicName = ( const char * ) "$aws/things/"clientcredentialIOT_THING_NAME"/shadow/get";
                      my_PublishInfo.topicNameLength = strlen("$aws/things/"clientcredentialIOT_THING_NAME"/shadow/get");
                      MQTT_Status = IotMqtt_TimedPublish( pub_sub_InfoData_mqttConnection, &my_PublishInfo, 0, 5000 );
                      sequence_pub_msg = 0;
                      break;
            }
            if(MQTT_Status == IOT_MQTT_SUCCESS)
            {
              printf("MQTT pub ok\n");
            }
            else
            {
              printf("MQTT pub error\n");
              sequence_operations = 0;
            }
            break;
      case 3:
            if(eState != eOTA_AgentState_Active)
            {
              IotMqtt_Disconnect( pub_sub_InfoData_mqttConnection, false );
              sequence_operations = 0;
            }
            break;
    }
    if(WIFI_IsConnected() != pdTRUE)
    {
      flag_wifi_connection = false;
      reset_iot_module();
    }
    vTaskDelay(pdMS_TO_TICKS( 1000UL ));
  }
}

/* -------------------------------------- Reset module */
static void reset_iot_module()
{
  for(int i = 0; i < 60; i += 1)
  {
    printf("Seconds until the next reboot: ");
    printf("%d\n", 60 - i);
    vTaskDelay(pdMS_TO_TICKS( 1000UL ));
  }
  esp_restart();
}

/* --------------------------------------- IOT network init func */
bool iot_netwokr_init()
{
  bool flag_return_network_init = false;

  if( IotSdk_Init() != true)
  {
    printf("Error sdk init\n");
    return flag_return_network_init;
  }
  printf("sdk init ok\n");

  if( AwsIotNetworkManager_Init() != pdTRUE )
  {
    printf("aws network manager init error\n");
    return flag_return_network_init;
  }
  printf("aws network manager init ok\n");

  if(AwsIotNetworkManager_EnableNetwork(AWSIOT_NETWORK_TYPE_WIFI) != AWSIOT_NETWORK_TYPE_WIFI)
  {
    printf("Wifi connect none\n");
    return flag_return_network_init;
  }
  printf("Wifi STA connect ok\n");

  /* ---------------------------------------- Certificates info ---------------*/
  my_Credentials.pAlpnProtos = NULL;
  my_Credentials.disableSni = true;
  my_Credentials.pRootCa = (char *)tlsATS1_ROOT_CERTIFICATE_PEM;;
  my_Credentials.rootCaSize = ( size_t )tlsATS1_ROOT_CERTIFICATE_LENGTH;
  my_Credentials.maxFragmentLength = ( size_t )4096;

  /* ---------------------------------------- Interface network ----------------*/
  my_networkInfo.createNetworkConnection = true;
  my_networkInfo.u.setup.pNetworkServerInfo = &my_serverInfo;
  my_networkInfo.u.setup.pNetworkCredentialInfo = &my_Credentials;
  my_networkInfo.pNetworkInterface = IOT_NETWORK_INTERFACE_AFR;
  my_networkInfo.pMqttSerializer = &my_mqttSerializer;

  my_connectInfo.awsIotMqttMode = true;
  my_connectInfo.cleanSession = true;
  my_connectInfo.keepAliveSeconds = 1200;
  my_connectInfo.pClientIdentifier = (const char *)clientcredentialIOT_THING_NAME;
  my_connectInfo.clientIdentifierLength = ( uint16_t ) strlen( (const char *) clientcredentialIOT_THING_NAME );

  if(IotMqtt_Init() != IOT_MQTT_SUCCESS)
  {
    printf("MQTT init error\n");
    return flag_return_network_init;
  }
  printf("MQTT init ok\n");

  flag_return_network_init = true;
  return flag_return_network_init;
}

/* --------------------------------------- my init */
static void my_init( void )
{
  /* GPIO setting */
  gpio_pad_select_gpio(RELAY_PIN);
  gpio_set_direction(RELAY_PIN, GPIO_MODE_OUTPUT);

  gpio_pad_select_gpio(WIFI_INDICATOR);
  gpio_set_direction(WIFI_INDICATOR, GPIO_MODE_OUTPUT);

  gpio_pad_select_gpio(SENSOR_BUTTON);
  gpio_set_direction(SENSOR_BUTTON, GPIO_MODE_INPUT);
  gpio_pad_pulldown(SENSOR_BUTTON);

  /* UART0 setting */
  uart_config_t uart_config = 
  {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(UART_NUM_0, &uart_config);
  uart_set_pin(UART_NUM_0, GPIO_NUM_1, GPIO_NUM_3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_NUM_0, 1024 * 2, 0, 0, NULL, 0);
}

/* =============================== AWS FREERTOS FUNCTIONS ======================================== */

/*-----------------------------------------------------------*/
extern void vApplicationIPInit( void );
static void prvMiscInitialization( void )
{
    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();

    if( ( ret == ESP_ERR_NVS_NO_FREE_PAGES ) || ( ret == ESP_ERR_NVS_NEW_VERSION_FOUND ) )
    {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK( ret );

    /* Create tasks that are not dependent on the WiFi being initialized. */
    xLoggingTaskInitialize( mainLOGGING_TASK_STACK_SIZE,
                            tskIDLE_PRIORITY + 5,
                            mainLOGGING_MESSAGE_QUEUE_LENGTH );

    vApplicationIPInit();
}

/*-----------------------------------------------------------*/

extern void esp_vApplicationTickHook();
void IRAM_ATTR vApplicationTickHook()
{
    esp_vApplicationTickHook();
}

/*-----------------------------------------------------------*/
extern void esp_vApplicationIdleHook();
void vApplicationIdleHook()
{
    esp_vApplicationIdleHook();
}

/*-----------------------------------------------------------*/

void vApplicationDaemonTaskStartupHook( void )
{
}

/*-----------------------------------------------------------*/

void vApplicationIPNetworkEventHook( eIPCallbackEvent_t eNetworkEvent )
{
    uint32_t ulIPAddress, ulNetMask, ulGatewayAddress, ulDNSServerAddress;
    system_event_t evt;

    if( eNetworkEvent == eNetworkUp )
    {
        /* Print out the network configuration, which may have come from a DHCP
         * server. */
        FreeRTOS_GetAddressConfiguration(
            &ulIPAddress,
            &ulNetMask,
            &ulGatewayAddress,
            &ulDNSServerAddress );

        evt.event_id = SYSTEM_EVENT_STA_GOT_IP;
        evt.event_info.got_ip.ip_changed = true;
        evt.event_info.got_ip.ip_info.ip.addr = ulIPAddress;
        evt.event_info.got_ip.ip_info.netmask.addr = ulNetMask;
        evt.event_info.got_ip.ip_info.gw.addr = ulGatewayAddress;
        esp_event_send( &evt );
    }
}