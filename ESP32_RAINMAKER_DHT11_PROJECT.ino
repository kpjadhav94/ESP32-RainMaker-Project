/*
 * Project Aim: Interfacing of DHT11 Sensor and controlling load with Relay using ESP32 and ESP RainMaker App
 * Demonstration by: Mr. Ketan Jadhav
 */

/*---------------------------- Including Necessary Header Files ---------------------------------*/
#include "RMaker.h"       // This library enable remote control and monitoring of ESP32 in cloud
#include "WiFi.h"         // This library allow ESP32 to connect with Wi-Fi network
#include "WiFiProv.h"     // This library allow ESP32 to control Wi-Fi provisioning service for receiving and configuring Wi-Fi credintials over
#include "DHT.h"          // This library use to interface DHT11 Sensor with ESP32
#include <SimpleTimer.h>  // This library use to use Timer
#include <wifi_provisioning/manager.h>  // This library use to initialize provisioning manager istance

/*---------------------------- Set Default Value (i.e. using Macro) -----------------------------*/
/*--------- These values are initialised at the beginning before execution of project -----------*/
#define DEFAULT_RELAY_MODE true
#define DEFAULT_Temperature 0
#define DEFAULT_Humidity 0

/*--------------------------------- Set Bluetooth Credentials -----------------------------------*/
/*----------- These credentials will be used when we want to pair ESP32 with Mobile -------------*/
const char *service_name = "Swami";
const char *pop = "swami@123";

/*---------------------------------- Asign GPIO Pins of ESP32 -----------------------------------*/
/*----- These pins will be assigned to ESP32 Module and remain static throughout the program ----*/
static uint8_t gpio_reset = 0;  // Assign GPIO0 pin for gpio_reset
static uint8_t DHTPIN = 19;     // ESP32 pin GPIO19 connected to DHT11 sensor
static uint8_t relay = 21;      // ESP32 pin GPIO21 connected to Relay
bool relay_state = true;        // Default Relay status is true. For bool datatype only 2 value can be assigned i.e. True (1) or False(0)
bool wifi_connected = 0;        // Default status of Wi-Fi connection is 0 i.e. False

DHT dht(DHTPIN, DHT11);         // Initialization of "DHT" sensor object called "dht" on Pin(i.e. GIOP19) and Sensor Type (i.e. DHT11)

SimpleTimer Timer;              // Create object "Timer" of "SimpleTimer" class. This instance is used to create and manage timer intervals

//------------------------------------------- Declaring Devices -----------------------------------------------------//

//The framework provides some standard device types like switch, lightbulb, fan, temperature sensor.
static TemperatureSensor temperature("Temperature");
static TemperatureSensor humidity("Humidity");
static Switch my_switch("Relay", &relay);

/*----------- Declare and define function to handle various System Provisioning Events -----------*/
void sysProvEvent(arduino_event_t *sys_event)
{
  switch (sys_event->event_id) {    // Reading system event id in switch case and executing system provisioning accordingly
    case ARDUINO_EVENT_PROV_START:  // It is an event that is triggered when provisioning of device is starts. It is a constant defined in "WiFiProv.h" library
#if CONFIG_IDF_TARGET_ESP32         // Check whether ESP32 board is selected in Arduino IDE
      Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on BLE\n", service_name, pop);  // Print BLE credentials 
      printQR(service_name, pop, "ble");    // call function to print QR code. QR Code should be generated with BLE provisioning information.
#else
      Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on SoftAP\n", service_name, pop);
      printQR(service_name, pop, "softap"); // call function to print QR code. QR Code should be generated with BLE provisioning information.
#endif
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:  // This event is defined in "WiFiProv.h" library. This event triggered when ESP32 is connected with Wi-Fi network
      Serial.printf("\nConnected to Wi-Fi!\n"); // Print Wi-Fi connection status
      wifi_connected = 1;                   // set value of variable wifi_connected = 1 which indicates Wi-Fi is connected to ESP32 
      delay(500);
      break;
    case ARDUINO_EVENT_PROV_CRED_RECV: {    // This event receives Wi-Fi credentials during the provisioning process
        Serial.println("\nReceived Wi-Fi credentials");
        Serial.print("\tSSID : ");
        Serial.println((const char *) sys_event->event_info.prov_cred_recv.ssid); // print SSID on serial monitor
        Serial.print("\tPassword : ");
        Serial.println((char const *) sys_event->event_info.prov_cred_recv.password); // print Password on serial monitor
        break;
      }
    case ARDUINO_EVENT_PROV_INIT:     // This event handles Wi-Fi provisioning manager by calling the wifi_prov_mgr_disable_auto_stop() function and passing in a timeout value of 10,000 milliseconds (10 seconds)
      wifi_prov_mgr_disable_auto_stop(10000);
      break;
    case ARDUINO_EVENT_PROV_CRED_SUCCESS:     // This event is triggered when the ESP32 successfully receives and saves the Wi-Fi credentials during the provisioning process
      Serial.println("Stopping Provisioning!!!");
      wifi_prov_mgr_stop_provisioning();      // This function is use to stop the Wi-Fi provisioning process
      break;
  }
}

/* --------------------------------------- Define write_callback function ------------------------------------------ */
void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx)
{
  const char *device_name = device->getDeviceName();  // read device name and store in const char pointer
  Serial.println(device_name);                        // print device name
  const char *param_name = param->getParamName();     // read parameter name and store in const char pointer

  if (strcmp(device_name, "Relay") == 0)              // compare device name with "Relay". If condition become true go inside
  {
    if (strcmp(param_name, "Power") == 0)             // compare parameter name with "Power". If condition become true go inside
    {
      // print device name and parameter name on serial monitor and read Relay status
      Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      relay_state = val.val.b;
      (relay_state == false) ? digitalWrite(relay, LOW) : digitalWrite(relay, HIGH);    // if relay_state is false send LOW signal to relay pin else send HIGH signal to relay pin
      param->updateAndReport(val);    // upload value of variable "val" on cloud
    }
  }
}


void setup()
{
  Serial.begin(115200);     // initialize serial communication between ESP32 and Computer at a 115200 baud rate
  Serial.printf("\nStarting ESP-RainMaker\n");

  /*---------------------------------------- Configure ESP32 GPIO Pins -----------------------------------------*/
  pinMode(gpio_reset, INPUT); // gpio_reset pin (GPIO0) is configured to receive INPUT
  pinMode(relay, OUTPUT);     // relay pin (GPIO2) is configured to send OUTPUT
  digitalWrite(relay, DEFAULT_RELAY_MODE);  // default status of realy is configured. here we put default state HIGH

  // call begin() function by dht to begin it's operation
  dht.begin();

  /*---------------------------------- Declare NODE and Standard Swith Device -----------------------------------*/
  Node my_node;
  my_node = RMaker.initNode("Ketan");

  //Standard switch device
  my_switch.addCb(write_callback);

  /*------------------------------------------ Adding Devices in Node -------------------------------------------*/
  my_node.addDevice(temperature);
  my_node.addDevice(humidity);
  my_node.addDevice(my_switch);


  /*------------------------------------------ Call RMaker OTA Service ------------------------------------------*/
  RMaker.enableOTA(OTA_USING_PARAMS); // It enables OTA (Over-the-air) service using OTA Parameters
  //If you want to enable scheduling, set time zone for your region using setTimeZone().
  //The list of available values are provided here https://rainmaker.espressif.com/docs/time-service.html
  // RMaker.setTimeZone("Asia/Shanghai"); 
  // Alternatively, enable the Timezone service and let the phone apps set the appropriate timezone
  RMaker.enableTZService();           // This function will set timezone. In our case it will be "Asia/Shanghai"
  RMaker.enableSchedule();            // This function will enables the scheduling service for the node

  Serial.printf("\nStarting ESP-RainMaker\n");  // Print project name on serial monitor
  RMaker.start();                               // call start function to start operation of RMaker.start();

  /*---------------------- Configure Timer to send sensor data with interval of 3 Sec --------------------------*/
  Timer.setInterval(3000);    

  WiFi.onEvent(sysProvEvent); // call WiFi.onEvent () function and system provision event as parameter

/*---- check whether target device is ESP32 ? ------*/
  /* if target device is ESP32 then code calls begin provision of "WiFiProv" object with parameters - */
  /* 1. WIFI_PROV_SCHEME_BLE - This specifies the provisioning scheme to be used for BLE (Bluetooth Low Energy) connections */
  /* 2. WIFI_PROV_SCHEME_HANDLER_FREE_BTDM - This specifies the provisioning handler to be used for BLE connections. In this case, the "FREE_BTDM" handler is used */
  /* 3. WIFI_PROV_SECURITY_1 - This specifies the security level to be used for the provisioning process. In this case, "1" represents WPA2-PSK security */
  /* 4. pop - This is a pointer to a character array containing the pre-shared key (PSK) for the WiFi network being provisioned */
  /* 5. service_name - This is a pointer to a character array containing the service name to be used during the provisioning process */
  /* if target device is not ESP32 then code calls begin provision of "beginProvision" method with different parameters - */
  /* 1. WIFI_PROV_SCHEME_SOFTAP - This specifies the provisioning scheme to be used for SoftAP connections */
  /* 2. WIFI_PROV_SCHEME_HANDLER_NONE - This specifies that no provisioning handler is required for SoftAP connections */
  /* 3. WIFI_PROV_SECURITY_1 - This specifies the security level to be used for the provisioning process. In this case, "1" represents WPA2-PSK security */
  /* 4. pop - This is a pointer to a character array containing the pre-shared key (PSK) for the WiFi network being provisioned */
  /* 5. service_name - This is a pointer to a character array containing the service name to be used during the provisioning process */
#if CONFIG_IDF_TARGET_ESP32
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, WIFI_PROV_SECURITY_1, pop, service_name);
#else
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE, WIFI_PROV_SECURITY_1, pop, service_name);
#endif

}


void loop()
{

  if (Timer.isReady() && wifi_connected) {  // Check is timer ready with "isReady()" function and wifi_connected ? If YES execute below statements
    Serial.println("Sending Sensor's Data");// prompt message which process is going on 
    Send_Sensor();                          // call Send_Sensor() function to send reading received from DHT11 and Relay Status on cloud
    Timer.reset();                          // Reset a timer. It will wait till 3 seconds
  }



  /* --------------------------------------  Logic to Reset RainMaker ------------------------------------------------- */

  // Read GPIO0 Pin status (i.e. external button to reset device / onboard button)

  if (digitalRead(gpio_reset) == LOW) { // if Push button pressed execute following statement present inside
    Serial.printf("Reset Button Pressed!\n"); // prompt message which process is going on
    // Key debounce handling
    delay(100);
    int startTime = millis();           // create variable to store start time in miliseconds
    while (digitalRead(gpio_reset) == LOW) delay(50);
    int endTime = millis();             // create variable to store end time in miliseconds

    if ((endTime - startTime) > 10000) {
      // if difference between start time and end time is more than 10 second, then do factory reset all
      Serial.printf("Reset to factory.\n");
      wifi_connected = 0;
      RMakerFactoryReset(2);            // call FactoryReset () function
    } else if ((endTime - startTime) > 3000) {  // if difference between start time and end time is in between from 3 to 10 seconds, then reset Wi-Fi
      Serial.printf("Reset Wi-Fi.\n");
      wifi_connected = 0;
      // If key pressed for more than 3secs, but less than 10, reset Wi-Fi
      RMakerWiFiReset(2);               // call WiFiReset() function
    }
  }
  delay(100);
}

/* ---------------------------------------- Define Send_Sensor function -------------------------------------------- */
void Send_Sensor()
{
  float h = dht.readHumidity();     // create 'h' variable to store value of humidity. Use readHumidity() function to read value of humidity
  float t = dht.readTemperature();  // create 't' variable to store value of temperature. Use readTemperature() function to read value of temperature

  Serial.print("Temperature - "); Serial.println(t);  // print value of temperature on serial monitor
  Serial.print("Humidity - "); Serial.println(h);     // print value of humidity on serial monitor

  temperature.updateAndReportParam("Temperature", t); // update value of temperature on cloud
  humidity.updateAndReportParam("Temperature", h);    // update value of humidity on cloud
}
