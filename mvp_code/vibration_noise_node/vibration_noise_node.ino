#include <Arduino_LSM6DSOX.h>
#include <PDM.h>
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include "secrets.h" 

#define ACC 0
#define MIC 1

// sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;      // your network SSID (name)
char pass[] = SECRET_PASS;      // your network password 
int status = WL_IDLE_STATUS;    // the WiFi radio's status
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const uint16_t samples = 128;
double samplingFrequency = 100;   //Hz, set in setup()
unsigned int sampling_period_us;
unsigned long microseconds;
double ms;

//MIC
// default number of output channels
static const char channels = 1;
// default PCM output frequency
static const int frequency = 20000;
// Buffer to read samples into, each sample is 16-bits
short sampleBuffer[512];
// Number of audio samples read
volatile int samplesRead;

int state = 0;

void setup() {
  Serial.begin(9600);
  //while (!Serial);
  
  // attempt to connect to WiFi network:
  Serial.println("Connecting to wifi...");
  while (status != WL_CONNECTED) {
    status = WiFi.begin(ssid, pass);
    Serial.print(".");
    delay(1000);
  }

  Serial.println("Connecting to the MQTT broker - ThingsBoard");
  mqttClient.setUsernamePassword(TOKEN, "password");
  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    while (1);
  }
  Serial.println("all connected!");

  // Configure the data receive callback
  PDM.onReceive(onPDMdata);

  // Optionally set the gain
  // Defaults to 20 on the BLE Sense and -10 on the Portenta Vision Shield
  // PDM.setGain(30);
  // Initialize PDM with: one channel (mono mode)
  if (!PDM.begin(channels, frequency)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }

  // Initialize ACC
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
  //Serial.print(IMU.accelerationSampleRate());Serial.println(" Hz");
  //Serial.println("time[s],value");
  samplingFrequency = IMU.accelerationSampleRate();
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  ms = millis();
}

void sendData(int var, float to_send){
  int cnt = 0;
  while(WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, pass);
    delay(500);
    cnt++;
    if(cnt == 3){
      Serial.println("Unable to connect to wifi");
      return;
    }
  }
  if (!mqttClient.connected()) {
    if (!mqttClient.connect(broker, port)) {
      Serial.print("MQTT connection failed! Error code = ");
      Serial.println(mqttClient.connectError());
      return;
    }
  }
  
  // send message, the Print interface can be used to set the message contents
  if(var == MIC){
    mqttClient.beginMessage(topic);
    mqttClient.print("{""mic"":");
    mqttClient.print(to_send);
    mqttClient.print("}");
    mqttClient.endMessage(); 
  } else if(var == ACC){
    mqttClient.beginMessage(topic);
    mqttClient.print("{""value"":");
    mqttClient.print(to_send,5);
    mqttClient.print("}");
    mqttClient.endMessage(); 
  }
}

void mic_function(){
  long int sum = 0;
  int cnt = 0;
  while(cnt < 300){
    if(samplesRead){
      for (int i = 0; i < samplesRead; i++) {
        sum = sum + abs(sampleBuffer[i]);
      }
      samplesRead = 0;  // Clear the read count
      cnt++;
    }
  }

  sendData(MIC, (float) sum/1000000);
}

void acc_function(){
  float x_val, y_val, z_val, last_x, last_y, last_z;
  float xyz = 0;
  microseconds = micros();
  for(int i=0; i<samples; i++){
    if (IMU.accelerationAvailable())
      IMU.readAcceleration(x_val, y_val, z_val);
    if(i > 0){
      xyz = xyz + abs(x_val-last_x) + abs(y_val-last_y) + abs(z_val-last_z);
    }
    last_x = x_val;
    last_y = y_val;
    last_z = z_val;

    while(micros() - microseconds < sampling_period_us){
      //empty loop
    }
    microseconds += sampling_period_us;
  }
  
  sendData(ACC, xyz);
}

void loop(){
  // call poll() regularly to allow the library to send MQTT keep alives which
  // avoids being disconnected by the broker
  mqttClient.poll();
  if(state == 0){
    acc_function();
    state = 1; 
  } else{
    mic_function();
    state = 0;
  }
}

/**
 * Callback function to process the data from the PDM microphone.
 * NOTE: This callback is executed as part of an ISR.
 * Therefore using `Serial` to print messages inside this function isn't supported.
 * */
void onPDMdata() {
  // Query the number of available bytes
  int bytesAvailable = PDM.available();

  // Read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}
