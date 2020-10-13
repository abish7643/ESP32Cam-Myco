# ESP32CAM With Adafruit.io

## Dependencies
- Hardware
  - AI Thinker ESP32 CAM
  - USB UART Programmer (CP2102 or FTDI)
  - BME 280 Sensor
  - SGP30 Sensor
- Libraries
  - PubSubClient@2.7
  - ArduinoJson@6.15.1
  - Adafruit_BME280_Library (Modified)
  - Adafruit_BusIO (Modified)
  - Adafruit_SGP30_Sensor (Modified)
  - Adafruit_Unified_Sensor (Modified)

`Note : Use the attached library (modified) rather than the original one since it has been slightly modified to work with ESP32CAM.`

## Contents
- Publish to Adafruit.IO
  - Using Platform IO
  - Using Arduino IDE
- Customizing The Credentials
- NTP Server For Setting Intial Time
- Peripheral Connections
- Run Time & MQTT Publishing 
- Expected Output
- Subscribe to Topic Using Python

## Publish to Adafruit.IO
1. ### Using Platform IO

    In the file platformio.ini, before the library dependencies, add a build flag to modify the MQTT Packet Size to the required size. The default packet size defined in PubSubClient Library is 128 Bytes. It's increased to 36000 Bytes inorder to satisfy the requirements.

    ```ini
    build_flags = -DMQTT_MAX_PACKET_SIZE=38000

    lib_deps =
       PubSubClient@2.7
       ArduinoJson@6.15.1
    ```
    For additional information, go to [Platform IO build_flags](https://docs.platformio.org/en/latest/projectconf/section_env_build.html#build-flags).

    `Note : The modified Adafruit_BME280_Library, Adafruit_BusIO, Adafruit_SGP30_Sensor, Adafruit_Unified_Sensor Libraries has to be replaced in the `src` directory in the project folder.`

2. ### Using Arduino IDE

    Install the PubSubClient Library from Tools -> Manage Libraries. We used the 2.7 version of the same. After installing the library go to the directory of Libraries related to Arduino, and navigate to PubSubClient Library.

    * #### In Linux
        ```bash
        user@user:~/Arduino/libraries/PubSubClient$ pwd
        /home/user/Arduino/libraries/PubSubClient
        ```
    * #### In Windows
    
        Navigate to `user/Documents/Arduino/libraries/PubSubClient`

    Go to PubSubClient.h residing in the src folder, Find the following codeblock.

    ```cpp
    // MQTT_MAX_PACKET_SIZE : Maximum packet size
    #ifndef MQTT_MAX_PACKET_SIZE
    #define MQTT_MAX_PACKET_SIZE 128
    #endif
    ```
    And modify according to requirements.

    ```cpp
    // MQTT_MAX_PACKET_SIZE : Maximum packet size
    #ifndef MQTT_MAX_PACKET_SIZE
    #define MQTT_MAX_PACKET_SIZE 38000
    #endif
    ```
    `Note : The modified Adafruit_BME280_Library, Adafruit_BusIO, Adafruit_SGP30_Sensor, Adafruit_Unified_Sensor Libraries has to be replaced in the `user/Documents/Arduino/libraries`  (Windows) or  `Home/Arduino/libs` (Linux) directory.`

## Adding The Credentials

In the main.cpp file (If Arduino IDE, ESP32Cam_MQTT.ino),

1. #### Replace the variables with your SSID/Password combination

    ```cpp
    const char *ssid = "WiFi_SSID";
    const char *password = "WiFi_Password";
    ```

2. #### Add MQTT Credentials.
   
   ```cpp
   const char *mqtt_server = "io.adafruit.com";
   const char *mqtt_clientid = "mqtt_clientid";
   const char *mqtt_username = "mqtt_username";
   const char *mqtt_password = "mqtt_password";
   
   const char *mqtt_publish_topic_camera = "username/feeds/camera";
   const char *mqtt_publish_topic_temperature = "username/feeds/temperature";
   const char *mqtt_publish_topic_humidity = "username/feeds/humidity";
   const char *mqtt_publish_topic_co2content = "username/feeds/co2content";
   const char *mqtt_publish_topic_pressure = "username/feeds/pressure";
   const char *mqtt_publish_topic_altitude = "username/feeds/altitude";
   const char *mqtt_publish_topic_ledstate = "username/feeds/ledstate";
   const char *mqtt_publish_topic_dehumidifier = "username/feeds/dehumidifier";
   const char *mqtt_publish_topic_fanstate = "username/feeds/fanstate";
   ```
   - Replace mqtt_clientid with a unique random id.
   - Replace mqtt_username with the Adafruit Username.
   - Replace mqtt_password with the Adafruit AIO Key.
   - Replace "username/" from all the topics to your Adafruit Username.
  
## NTP Server For Setting Intial Time

Adding Specific NTP Server.

```cpp
const char *ntpServer = "asia.pool.ntp.org"; // Asia Time
const long gmtOffset_sec = 270 * 60;         // India Offset (+4:30 hr)
const int daylightOffset_sec = 60 * 60; // Daylight Saving Time (+1:00 hr)

// Africa — africa.pool.ntp.org
// Antarctica — antarctica.pool.ntp.org
// Asia — asia.pool.ntp.org
// Europe — europe.pool.ntp.org
// North America — north-america.pool.ntp.org
// Oceania — oceania.pool.ntp.org
// South America — south-america.pool.ntp.org
```
If a person is from India, `asia.pool.ntp.org` is the NTP server. An offset of +5:30hr (330 Minutes) is required for India. This should be added in terms of seconds. Therefore `gmtOffset_sec` is made 270*60 seconds and `daylightOffset_sec` is made (1hr) 60*60 seconds (Daylight Saving Time).

Note : Time is only taken on the first loop, then it'll be used as a reference to the local time.

```cpp
 configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
```

## Peripheral Connections
   
1. ### Connect Sensors to the Custom I2C Bus.
   
      ```cpp
      #define I2C_SDA 14 // SDA Connected to GPIO 14
      #define I2C_SCL 15 // SCL Connected to GPIO 15
      TwoWire I2CSensors = TwoWire(0);
      ```

2. ### BME 280 Sensor `(Use Address 0x77 if 0x76 doesn't work)` and SGP30 Sensor

      ```cpp
      void setup()
      {
         ...

        if (!bme.begin(0x76, &I2CSensors))
        {
           Serial.println("Could not find a valid BME280 sensor, check wiring!");
           while (1);
        }
        else
        {
           Serial.println("BME280 Sensor Found");
        }

        ...

        if (!sgp.begin(&I2CSensors))
        {
           Serial.println("Could not find SGP30 sensor");
           while (1);
        }
        else
        {
           Serial.print("Found SGP30 serial #");
           Serial.print(sgp.serialnumber[0], HEX);
           Serial.print(sgp.serialnumber[1], HEX);
           Serial.println(sgp.serialnumber[2], HEX);
        }

        ...
      }
      ```
   `NOTE : Alititude, Pressure are published even though there's no conditional logic added for the values. Feel free to comment out the respective publishing if not required.`

   ```cpp
      publishSensors(mqtt_publish_topic_pressure, String(pressure).c_str()); // Pressure
      publishSensors(mqtt_publish_topic_altitude, String(altitude).c_str()); // Humidity
   ```

3. ### Other Hardware Connections

   An external LED is connected on GPIO 33. If Inbuilt LED only need to be used, comment out the lines with LEDPin. (Reverse Logic -> Inbuilt LED has to be driven low to make it ON)
   
   ```cpp
   
   #define ESP32CAM_LED_INBUILT 33 // BUILTIN LED Internally Connected to GPIO 33 (Reverse Logic)
   #define ESP32CAM_LED_FLASH 4    // BUILTIN FLASH Internally Connected to GPIO 4

   ...

   #define LEDPin 2           // LED Pin Connected to GPIO 2
   #define DeHumidifierPin 12 // DeHumidifier Connected to GPIO 12
   #define FanPin 13          // Fan Connected to GPIO 13

   ```
   > Feel free to customise the GPIO Pins if required.

   `NOTE : DeHumidifier, LED, and Fan States are published to MQTT. States are either 1 (ON) or 0 (OFF). Feel free to comment out the respective publishing lines if not required. Adafruit has "indicator" block which can represent state using conditional logic. This can show which devices are in ON State.`

   ```cpp
   publishSensors(mqtt_publish_topic_ledstate, "1");
   publishSensors(mqtt_publish_topic_fanstate, "1");
   publishSensors(mqtt_publish_topic_dehumidifier, "1");
   ```
4. ### SGP30 Calibration

   SGP30 is calibrated using the baseline value obtained from the library. It is saved to SPIFFS if the value changes. This helps to `self calibrate`. Refer SGP30 Documentation for more info.

   ```cpp
   // Set Baseline Values to 'Self-Calibrate' SGP30
   sgp.setIAQBaseline(eCO2_base, TVOC_base);

   ```

   For getting the baseline values, `getIAQBaseline` method is used. If the values new values are different from the previous baseline values, these is written to SPIFFS.

   ```cpp
      // Get New Baseline Values
       uint16_t new_eCO2_base, new_TVOC_base;

       if (!sgp.getIAQBaseline(&new_eCO2_base, &new_TVOC_base))
       {
         Serial.println("Failed to get baseline readings");
         return;
       }
   ```

   While the device starts, it looks for the `Config File` containing the baseline values. If it doesn't exist, the file is created.

   ```cpp
   if (SPIFFS.exists("/config.json"))
       {
         // Reads Config If The File Exists
         readConfigFile();

         ....

         // Set Baseline Values to 'Self-Calibrate' SGP30
         sgp.setIAQBaseline(eCO2_base, TVOC_base);
       }
       else
       {
         // Creates Config File If The File Doesn't Exist
         Serial.println("Creating new config file");
         writeConfigFile();
       }
   ```

## Run Time & MQTT Publishing

   Adjust the runtime for publishing sensor values, device states each and every 5 minute or modify 5 to anything you require. If a sensor value goes beyond the predefined limit the state of device and sensor values will be instantaneously publishing using MQTT.

   ```cpp
   int RunTime = 5 * 60; // 5 Minutes
   ```

   Set the predefined Trigger limits for conditional activation of devices.

   ```cpp
   // Trigger Limits
   float HumidityLimit = 90.00; // Relative Humidity = 90%
   int CO2ContentLimit = 900;   // CO2 Content Limit = 900PPM
   ```

## Expected Output

```bash
   MYCO CAMERA V1

   SPIFFS mounted successfully
   reading config file
   opened config file
   {"CO2Base":17251,"TVOCBase":26946}
   parsed json
   Baseline values: eCO2: 0x4363 & TVOC: 0x6942

   Connecting to Dhanish
   ..
   WiFi connected
   IP address: 
   192.168.1.13
   Tuesday, October 13 2020 22:11:10
   Hour : 22
   ------------------------
   Attempting MQTT connection...connected
   Pressure = 1021.64 hPa
   Approx. Altitude = 3.45 m
   Temperature = 30.76 *C
   Humidity = 73.32 %
   eCO2 1002 ppm
   ------------------------
   Publishing...Published To abishvijayan/feeds/fanstate | 1
   ------------------------
   -------Fan Turned ON--------
   ------------------------
   Publishing...Published To abishvijayan/feeds/temperature | 30.76
   ------------------------
   ------------------------
   Publishing...Published To abishvijayan/feeds/humidity | 73.32
   ------------------------
   ------------------------
   Publishing...Published To abishvijayan/feeds/co2content | 1002
   ------------------------
   ------------------------
   Publishing...Published To abishvijayan/feeds/pressure | 1021.64
   ------------------------
   ------------------------
   Publishing...Published To abishvijayan/feeds/altitude | 3.45
   ------------------------
   ------------------------
   Publishing...Published To abishvijayan/feeds/ledstate | 0
   ------------------------
   -----LED is OFF-----
   ------------------------
   Publishing...Published To abishvijayan/feeds/dehumidifier | 0
   ------------------------
   Camera Captured
   Buffer Length: 
   20828
   Publishing...Published Image to abishvijayan/feeds/camera
   ------------------------
   Tuesday, October 13 2020 22:11:12
   Hour : 22
   ------------------------
   
   ...
```

## Subscribe to Topic Using Python (Only Subscribes to Image)

A basic python code to subscribe to the topic `username/feeds/camera` and updates realtime. (Replace `username` with Adafruit username)

#### Requirements:
- Python3
- Paho MQTT Client
- PIL (Python Imaging Library)
- PyGame

## Adding Credentials

1. #### Subscribe to Camera Topic in the `on_connect` function,
    ```python
    def on_connect(client, userdata, flags, rc):
        print("Connected with result code "+str(rc))
        client.subscribe("username/feeds/camera")
    ```

2. #### Replace MQTT Credentials,
    ```python
    client.username_pw_set(username="mqtt_username",password="mqtt_password"
    client.connect("io.adafruit.com", 1883, 60)
    ```

3. #### Run using python3
    ```bash
    python3 test_subscriber.py 
    ```