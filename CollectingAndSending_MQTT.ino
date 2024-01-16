#include "Arduino_SensorKit.h"
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include "private_info.h"

///////enter SSID name and WIFI password in the private_info.h file

char ssid[] = SECRET_SSID; // your network SSID (name)
char pass[] = SECRET_PASS; // your WIFI password

WiFiClient wifiClient;

MqttClient mqttClient(wifiClient);

const char broker[] = "167.71.39.204";
int port = 1883;
const char *mqttUser = "<username>";
const char *mqttPassword = "<password>";

//*************************TOPICS********************************
const char topic1[] = "AIPL/ComfortPie_Room108/temperature";
const char topic2[] = "AIPL/ComfortPie_Room108/humidity";
const char topic3[] = "AIPL/ComfortPie_Room108/flameSensor";
const char topic4[] = "AIPL/ComfortPie_Room108/soundSensor";
const char topic5[] = "AIPL/ComfortPie_Room108/lightIntensity";
const char topic6[] = "AIPL/ComfortPie_Room108/carbonmonoxide";

const char topic7[] = "AIPL/ComfortPie_Room109/temperature";
const char topic8[] = "AIPL/ComfortPie_Room109/humidity";
const char topic9[] = "AIPL/ComfortPie_Room109/flameSensor";
const char topic10[] = "AIPL/ComfortPie_Room109/soundSensor";
const char topic11[] = "AIPL/ComfortPie_Room109/lightIntensity";
const char topic12[] = "AIPL/ComfortPie_Room109/carbonmonoxide";

const char topic13[] = "AIPL/ComfortPie_Room110/temperature";
const char topic14[] = "AIPL/ComfortPie_Room110/humidity";
const char topic15[] = "AIPL/ComfortPie_Room110/flameSensor";
const char topic16[] = "AIPL/ComfortPie_Room110/soundSensor";
const char topic17[] = "AIPL/ComfortPie_Room110/lightIntensity";
const char topic18[] = "AIPL/ComfortPie_Room110/carbonmonoxide";

const char topic19[] = "AIPL/ComfortPie_Room111/temperature";
const char topic20[] = "AIPL/ComfortPie_Room111/humidity";
const char topic21[] = "AIPL/ComfortPie_Room111/flameSensor";
const char topic22[] = "AIPL/ComfortPie_Room111/soundSensor";
const char topic23[] = "AIPL/ComfortPie_Room111/lightIntensity";
const char topic24[] = "AIPL/ComfortPie_Room111/carbonmonoxide";

const char topic25[] = "AIPL/ComfortPie_Room112/temperature";
const char topic26[] = "AIPL/ComfortPie_Room112/humidity";
const char topic27[] = "AIPL/ComfortPie_Room112/flameSensor";
const char topic28[] = "AIPL/ComfortPie_Room112/soundSensor";
const char topic29[] = "AIPL/ComfortPie_Room112/lightIntensity";
const char topic30[] = "AIPL/ComfortPie_Room112/carbonmonoxide";

const char topic31[] = "AIPL/ComfortPie_Room215/temperature";
const char topic32[] = "AIPL/ComfortPie_Room215/humidity";
const char topic33[] = "AIPL/ComfortPie_Room215/flameSensor";
const char topic34[] = "AIPL/ComfortPie_Room215/soundSensor";
const char topic35[] = "AIPL/ComfortPie_Room215/lightIntensity";
const char topic36[] = "AIPL/ComfortPie_Room215/carbonmonoxide";

const char topic37[] = "AIPL/ComfortPie_Room216/temperature";
const char topic38[] = "AIPL/ComfortPie_Room216/humidity";
const char topic39[] = "AIPL/ComfortPie_Room216/flameSensor";
const char topic40[] = "AIPL/ComfortPie_Room216/soundSensor";
const char topic41[] = "AIPL/ComfortPie_Room216/lightIntensity";
const char topic42[] = "AIPL/ComfortPie_Room216/carbonmonoxide";

bool fire = 0;
bool gas = 0;
int delay_counter = 1; // to modify how often it reads temp and humidity values and send them
long Sound_values_sum = 0;
long sound_counter = 0;
const float sensitivity_dBV_Pa = -57; // sensitivity value for Grove Sound Sensor
const float Vref = 5.0;               // Reference voltage for the ADC

// Pins:
int light_sensor = A3;
int sound_sensor = A2;
int buzzer = 5;
int led = 6;
int flame_sensor = A5;
int co_sensor = A0;

const long interval = 5000; // interval for reading sensors and sending messages (milliseconds)
unsigned long previousMillis = 0;

void Connect_to_broker()
{
  while (!mqttClient.connect(broker, port))
  {

    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    Serial.println("Retrying...");
    delay(1000);
  }

  Serial.println("Connected to the MQTT broker!");
  Serial.println();
  return;
}

void setup()
{

  // Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // attempt to connect to Wifi network:

  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);

  while (WiFi.begin(ssid, pass) != WL_CONNECTED)
  {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }

  Serial.println("Connected to the network");
  Serial.println();
  mqttClient.setUsernamePassword(mqttUser, mqttPassword);

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  Connect_to_broker();

  pinMode(buzzer, OUTPUT);
  pinMode(led, OUTPUT);

  Environment.begin(); // Initialising the Temperature and Humidity sensor
}

void loop()
{

  if (WiFi.status() != WL_CONNECTED)
  {                          // Is the WIFI disconnected?
    digitalWrite(led, HIGH); // Activating LED to indicate the WIFI connection failed
    Serial.print("Attempting to reconnect to SSID: ");
    Serial.println(ssid);

    while (WiFi.begin(ssid, pass) != WL_CONNECTED)
    {
      Serial.print(".");
      // wait 5 seconds for connection:
      delay(5000);
    }

    Serial.println("Reconnected to the network");
    Serial.println();
    Serial.print("Attempting to reconnect to the MQTT broker: ");
    Serial.println(broker);

    Connect_to_broker();

    digitalWrite(led, LOW); // Deactivating LED after connection reset
  }

  // call poll() regularly to allow the library to send MQTT keep alive which
  // avoids being disconnected by the broker
  mqttClient.poll();

  // Reading sound sensor constantly to calculate the average afterwards:
  int raw_sound = analogRead(sound_sensor);
  Sound_values_sum += raw_sound;
  sound_counter++;

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {
    // save the last time a message was sent
    previousMillis = currentMillis;

    // Reading light sensor:
    int raw_light = analogRead(light_sensor);    // read the raw value from light_sensor pin (A3)
    int light = map(raw_light, 0, 1023, 0, 100); // map the value from 0, 1023 to 0, 100

    // Processing sound samples:
    float sound = Sound_values_sum / sound_counter; // Finding the average of the samples
    float voltage = (sound / 1023.0) * Vref;
    float soundPressurePa = pow(10, (voltage + sensitivity_dBV_Pa) / 20.0);
    float soundPressurePaRef = 20.0e-6; // Reference sound pressure in Pa for 0 dB SPL
    float sounddB = 20 * log10(soundPressurePa / soundPressurePaRef);

    // Reinitializing to take more samples:
    Sound_values_sum = 0;
    sound_counter = 0;

    // Reading flame sensor:
    int flame = analogRead(flame_sensor);

    // Reading CO sensor:
    int co = analogRead(co_sensor);

    //***ALARMS*** Checking if there's fire or a dangerous CO level:
    Serial.println(flame);
    if (flame >= 300)
      fire = 1;
    else
      fire = 0;
    if (co > 23)
      gas = 1;
    else
      gas = 0;

    if (fire || gas)
    {
      // Activate the Buzzer and the LED:
      digitalWrite(led, HIGH); // Sets the voltage to high
      tone(buzzer, 95);        // Set the voltage to high and makes a noise
      delay(2000);             // Waits for 2000 milliseconds
      noTone(buzzer);          // Sets the voltage to low and makes no noise
      digitalWrite(led, LOW);  // Sets the voltage to low
    };

    //***********************Sending the messages for each topic***********************
    if (--delay_counter == 0)
    { // to modify how often it reads the temperature and humidity values and send them
      delay_counter = 1;
      // Reading temperature:
      float temperature = Environment.readTemperature();
      // Reading humidity:
      float humidity = Environment.readHumidity();

      if (!mqttClient.connected())
      { // checking to be properly connected to the broker before sending the messages
        Connect_to_broker();
      }

      mqttClient.beginMessage(topic1);
      mqttClient.print(temperature);
      mqttClient.endMessage();

      mqttClient.beginMessage(topic2);
      mqttClient.print(humidity);
      mqttClient.endMessage();

      //*****TEMP AND HUMIDITY (DUMMY DATA)*********
      mqttClient.beginMessage(topic7);
      mqttClient.print(temperature + 0.5);
      mqttClient.endMessage();

      mqttClient.beginMessage(topic8);
      mqttClient.print(humidity - 1);
      mqttClient.endMessage();

      mqttClient.beginMessage(topic13);
      mqttClient.print(temperature + 1);
      mqttClient.endMessage();

      mqttClient.beginMessage(topic14);
      mqttClient.print(humidity + 1.5);
      mqttClient.endMessage();

      mqttClient.beginMessage(topic19);
      mqttClient.print(temperature - 0.5);
      mqttClient.endMessage();

      mqttClient.beginMessage(topic20);
      mqttClient.print(humidity + 1);
      mqttClient.endMessage();

      mqttClient.beginMessage(topic25);
      mqttClient.print(temperature - 1);
      mqttClient.endMessage();

      mqttClient.beginMessage(topic26);
      mqttClient.print(humidity - 1.5);
      mqttClient.endMessage();

      mqttClient.beginMessage(topic31);
      mqttClient.print(temperature + 1.5);
      mqttClient.endMessage();

      mqttClient.beginMessage(topic32);
      mqttClient.print(humidity - 0.5);
      mqttClient.endMessage();

      mqttClient.beginMessage(topic37);
      mqttClient.print(temperature - 1.5);
      mqttClient.endMessage();

      mqttClient.beginMessage(topic38);
      mqttClient.print(humidity + 0.5);
      mqttClient.endMessage();
    }

    //*****Sending the info of the rest of the sensors (real and dummy data)*********
    mqttClient.beginMessage(topic3);
    mqttClient.print(fire);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic4);
    mqttClient.print(sounddB);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic5);
    mqttClient.print(light);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic6);
    mqttClient.print(gas);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic9);
    mqttClient.print(0);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic10);
    mqttClient.print(sounddB + 10);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic11);
    mqttClient.print(int(random(5, 96)));
    mqttClient.endMessage();

    mqttClient.beginMessage(topic12);
    mqttClient.print(0);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic15);
    mqttClient.print(0);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic16);
    mqttClient.print(sounddB + 12);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic17);
    mqttClient.print(int(random(5, 96)));
    mqttClient.endMessage();

    mqttClient.beginMessage(topic18);
    mqttClient.print(0);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic21);
    mqttClient.print(0);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic22);
    mqttClient.print(sounddB - 2);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic23);
    mqttClient.print(int(random(5, 96)));
    mqttClient.endMessage();

    mqttClient.beginMessage(topic24);
    mqttClient.print(0);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic27);
    mqttClient.print(0);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic28);
    mqttClient.print(sounddB - 5);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic29);
    mqttClient.print(int(random(5, 96)));
    mqttClient.endMessage();

    mqttClient.beginMessage(topic30);
    mqttClient.print(0);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic33);
    mqttClient.print(0);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic34);
    mqttClient.print(sounddB + 8);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic35);
    mqttClient.print(int(random(5, 96)));
    mqttClient.endMessage();

    mqttClient.beginMessage(topic36);
    mqttClient.print(0);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic39);
    mqttClient.print(0);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic40);
    mqttClient.print(sounddB + 11);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic41);
    mqttClient.print(int(random(5, 96)));
    mqttClient.endMessage();

    mqttClient.beginMessage(topic42);
    mqttClient.print(0);
    mqttClient.endMessage();
  }
}