/**************************************************************
#															 
#	Project: LightDNA - Device code of ESP8266   
#	Author: Anderson Ignacio da Silva						 
#   Date: 13/07/15											 
#	Target: ESP-8266							 
#	Inf.: http://www.esplatforms.blogspot.com.br 			 
#													 
**************************************************************/
#define DEBUG_OLED

#include "ets_sys.h"
#include "driver/uart.h"
#include "osapi.h"
#include "mqtt.h"
#include "wifi.h"
#include "config.h"
#include "debug.h"
#include "gpio.h"
#include "user_interface.h"
#include "mem.h"
#include "driver/gpio16.h"
#ifdef	DEBUG_OLED
	#include "driver/i2c.h"
	#include "driver/i2c_oled.h"
#endif

#define TOPIC_DEFAULT	"HomeStark/room/door_%08X"
#define TOPIC_STATUS	"HomeStark/room/door_%08X/status"
#define MESSAGE_OFFLINE	"HomeStark device %08X it's offline!"

#define LED_1(x)	gpio16_output_set(x) 
#define LED_2(x)	GPIO_OUTPUT_SET(GPIO_ID_PIN(14), x) 
#define LED_3(x)	GPIO_OUTPUT_SET(GPIO_ID_PIN(12), x) 
#define RELE(x)		GPIO_OUTPUT_SET(GPIO_ID_PIN(5), x)
#define BUZZER(x)	GPIO_OUTPUT_SET(GPIO_ID_PIN(4), x) 

MQTT_Client mqttClient;
uint8_t topic_dimmer[64],
		topic_current[64],
		topic_status[64],
		topic_pir[64],
		topic_temp[64],
		topic_ldr[64],
		topic_default[64],
		message_dead[64],
		device_id_hw[8];

typedef enum states
{
  FIRST_IDT,
  DATA_B,
  COMPLETE
}states_buffer;

states_buffer serial_s = FIRST_IDT;
static ETSTimer StatusTimer, DoorTimer;
uint8 buffer_serial[10],i = 0;
bool	OLED, CleanOLED = false,blinkS;

void PrepareTopics(){
	os_sprintf(topic_default, TOPIC_DEFAULT, system_get_chip_id());
	// os_sprintf(topic_dimmer, TOPIC_DIMMER, system_get_chip_id());
	// os_sprintf(topic_current, TOPIC_CURRENT, system_get_chip_id());
	os_sprintf(topic_status, TOPIC_STATUS, system_get_chip_id());
	// os_sprintf(topic_temp, TOPIC_TEMP, system_get_chip_id());
	// os_sprintf(topic_pir, TOPIC_PIR, system_get_chip_id());
	// os_sprintf(topic_ldr, TOPIC_LDR, system_get_chip_id());

	os_sprintf(message_dead, MESSAGE_OFFLINE, system_get_chip_id());
	os_sprintf(device_id_hw, "%08X", system_get_chip_id());
}

void DoorTimerCall(){
	#ifdef	DEBUG_OLED
		OLED_CLS();
		OLED_Print(0,0, "System Started...", 1);
		OLED_Print(0,1, "HomeStark device v1.0", 1);
		// OLED_Print(0,3, "Topics:", 1);
		// OLED_Print(1,4, "lights QoS=1", 1);
		// OLED_Print(1,5, "lights/", 1);
		OLED_Print(2,3,device_id_hw,1);
		// OLED_Print(17,5, "/*", 1);
		// OLED_Print(0,6, "QoS=1", 1);
	#endif
	RELE(0);
}

void open_door(){
	RELE(1);
	MQTT_Publish(&mqttClient, topic_default, "F-OPEN", sizeof("F-OPEN"), 1, 0);
	os_timer_disarm(&DoorTimer);
	os_timer_setfn(&DoorTimer, DoorTimerCall, NULL);
	os_timer_arm(&DoorTimer, 1000, 0);
}

void wifiConnectCb(uint8_t status){
	if(status == STATION_GOT_IP){
		MQTT_Connect(&mqttClient);
	} else {
		MQTT_Disconnect(&mqttClient);
	}
}

void mqttConnectedCb(uint32_t *args){
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Connected\r\n");

	// Topics that each device must be subscribe
	// All has QoS lvl. 1 - At least once
	// lights - Broadcast topic to get current of leds
	// lights/device_address - unicast topic

	MQTT_Subscribe(client, topic_status, 1); 
	// MQTT_Subscribe(client, topic_default, 1);
	//MQTT_Subscribe(client, topic_status, 1);
	//MQTT_Subscribe(client, topic_current, 1);	
}

void mqttDisconnectedCb(uint32_t *args){
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Disconnected\r\n");
}

void mqttPublishedCb(uint32_t *args){
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Published\r\n");
}

void mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len){
	char *topicBuf = (char*)os_zalloc(topic_len+1),
			*dataBuf = (char*)os_zalloc(data_len+1);

	MQTT_Client* client = (MQTT_Client*)args;

	os_memcpy(topicBuf, topic, topic_len);
	topicBuf[topic_len] = 0;

	os_memcpy(dataBuf, data, data_len);
	dataBuf[data_len] = 0;

	// uint8_t limits =   (*(dataBuf)-0x30)*100+(*(dataBuf+1)-0x30)*10+(*(dataBuf+2)-0x30);
	
	// if(limits > 100 && os_strstr(topicBuf,"dimmer")){
	// 	INFO("Receive topic: dimmer, data: 100\r\n");
	// 	INFO("@100@");
	// }
	// else{
	// 	INFO("Receive topic: %s, data: %s \r\n", topicBuf, dataBuf);	
	// 	// INFO("%s",dataBuf);
	// 	// INFO("@");
	// 	INFO(dataBuf);
	// 	// INFO("@");
	// }
	INFO(dataBuf);
	if(os_strstr(dataBuf,"F-OPEN")){
		#ifdef	DEBUG_OLED
			OLED_CLS();
			OLED_Print(0,2, "[BROKER]Force OPEN", 1);
		#endif
		open_door();
	}

	os_free(topicBuf);
	os_free(dataBuf);
}

void PrintCurrent(uint8_t *current){
	OLED_Print(1,6, "Current:", 1);
	OLED_Print(9,6, "               ", 1);
	OLED_Print(9,6, current, 1);
}

void SendTMQTT(){
  i = 0;
  //INFO(buffer_serial);

  switch(buffer_serial[0]) {
  	case 'D':
		buffer_serial[0] = ' ';
  		MQTT_Publish(&mqttClient, topic_dimmer, buffer_serial, sizeof(buffer_serial), 1, 0);
  	break;
  	case 'C':
  		buffer_serial[0] = ' ';
  		MQTT_Publish(&mqttClient, topic_current, buffer_serial, sizeof(buffer_serial), 1, 0);
  		#ifdef DEBUG_OLED
  			PrintCurrent(buffer_serial);
  		#endif
  	break;
  	case 'P':
  		buffer_serial[0] = ' ';
  		MQTT_Publish(&mqttClient, topic_pir, buffer_serial, sizeof(buffer_serial), 1, 0);
  	break;
  	case 'T':
  		buffer_serial[0] = ' ';
  		MQTT_Publish(&mqttClient, topic_temp, buffer_serial, sizeof(buffer_serial), 1, 0);
  	break;
  	case 'L':
  		buffer_serial[0] = ' ';
  		MQTT_Publish(&mqttClient, topic_ldr, buffer_serial, sizeof(buffer_serial), 1, 0);
  	break;
  }
  serial_s = FIRST_IDT;
}

void RecChar(uint8 CharBuffer){
  //OLEDprintf(&CharBuffer);
  switch(serial_s)
  {
    case FIRST_IDT:
      	if(CharBuffer == '#')
      	{
        	serial_s = DATA_B;
        	for(i=0;i<10;i++)
        		buffer_serial[i] = '\0';
        	i = 0;
    	}
    break;
    case DATA_B:
        if(CharBuffer != '#' && i < 10)
        {
          buffer_serial[i] = CharBuffer;
          i++;
        }
        else
        {
          	serial_s = COMPLETE; 
        }
    break;
  }

  if(serial_s == COMPLETE)    SendTMQTT();
}



void interruptHandler(){
	uint32 gpio_status;
	bool status_reset_button;
	gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);

	if(gpio_status & BIT(13))
	{
		//clear interrupt status
		gpio_pin_intr_state_set(GPIO_ID_PIN(13), GPIO_PIN_INTR_DISABLE);
		GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status & BIT(13));
		gpio_pin_intr_state_set(GPIO_ID_PIN(13), GPIO_PIN_INTR_POSEDGE);
		//ResetToAP();
		open_door();
		// if(status_b == true)
		// {
		// 	//BUZZER(1);
		// 	//RELE(1);
		// 	status_b = false;
		// }
		// else
		// {
		// 	status_b = true;
		// 	//BUZZER(0);
		// 	//RELE(0);
		// }
		//if(status_reset_button)
			//INFO("\nz\rPUSH BUTTON PRESSED");
		//else
		//	INFO("\n\rPUSH BUTTON NOT PRESSED");

		
	}
}

void ConfigureGPIO() {
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U,  FUNC_GPIO4);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U,  FUNC_GPIO5);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U,   FUNC_GPIO12);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, 	 FUNC_GPIO14);
	gpio16_output_conf();

	//Setting interrupt in pin GPIO13 - Button Reset AP
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13);
	PIN_PULLDWN_EN(PERIPHS_IO_MUX_MTCK_U);

	ETS_GPIO_INTR_ATTACH(interruptHandler, NULL);
	ETS_GPIO_INTR_DISABLE();
	GPIO_DIS_OUTPUT(GPIO_ID_PIN(13));
	gpio_register_set(GPIO_ID_PIN(13), GPIO_PIN_INT_TYPE_SET(GPIO_PIN_INTR_DISABLE)
                    | GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_DISABLE)
                    | GPIO_PIN_SOURCE_SET(GPIO_AS_PIN_SOURCE));
	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, BIT(13));
  	gpio_pin_intr_state_set(GPIO_ID_PIN(13), GPIO_PIN_INTR_POSEDGE);
  	ETS_GPIO_INTR_ENABLE();
}

void BlinkStatusCB() {
	if(blinkS){
		LED_1(1);
		LED_2(1);
		LED_3(1);
		// GPIO_OUTPUT_SET(GPIO_ID_PIN(13), 0);
		blinkS = false;
	}
	else{
		LED_1(0);
		LED_2(0);
		LED_3(0);
		// GPIO_OUTPUT_SET(GPIO_ID_PIN(13), 1);
		blinkS = true;
	}
  	os_timer_disarm(&StatusTimer);
	os_timer_setfn(&StatusTimer, BlinkStatusCB, NULL);
	os_timer_arm(&StatusTimer, 500, 0);
}

void user_init(void){
	uart_init(BIT_RATE_115200, BIT_RATE_115200);
	os_delay_us(1000000);

	#ifdef	DEBUG_OLED
		//Initialize I2C Oled Display
		i2c_init();
	  	OLED = OLED_Init();
	 #endif
	ConfigureGPIO();

	os_timer_disarm(&StatusTimer);
	os_timer_setfn(&StatusTimer, BlinkStatusCB, NULL);
	os_timer_arm(&StatusTimer, 500, 0);
  	//Load define configurations
	CFG_Load();

	PrepareTopics(); //Copy device address to variable's topic MQTT

	MQTT_InitConnection(&mqttClient, sysCfg.mqtt_host, sysCfg.mqtt_port, sysCfg.security);
	MQTT_InitClient(&mqttClient, sysCfg.device_id, sysCfg.mqtt_user, sysCfg.mqtt_pass, sysCfg.mqtt_keepalive, 1);
	MQTT_InitLWT(&mqttClient, topic_status , message_dead, 0, 0);   //Last Will Teastment - Indicate that device has been dead, /lights/device_addres/status(Topic) Device device_address it's offline!(message)
	//MQTT_InitLWT(&mqttClient, "/lwt", "offline", 0, 0);
	MQTT_OnConnected(&mqttClient, mqttConnectedCb);
	MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
	MQTT_OnPublished(&mqttClient, mqttPublishedCb);
	MQTT_OnData(&mqttClient, mqttDataCb);

	WIFI_Connect(sysCfg.sta_ssid, sysCfg.sta_pwd, wifiConnectCb);

	INFO("\r\nSystem started ...\r\n");
	#ifdef	DEBUG_OLED
		OLED_Print(0,0, "System Started...", 1);
		OLED_Print(0,1, "HomeStark device v1.0", 1);
		// OLED_Print(0,3, "Topics:", 1);
		// OLED_Print(1,4, "lights QoS=1", 1);
		// OLED_Print(1,5, "lights/", 1);
		OLED_Print(2,3,device_id_hw,1);
		// OLED_Print(17,5, "/*", 1);
		// OLED_Print(0,6, "QoS=1", 1);
	#endif
}
