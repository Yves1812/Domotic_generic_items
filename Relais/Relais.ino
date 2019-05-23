// Connect to best known wifi and sends sensor data on a regular basis to MQTT Broker

#include <ESP8266WiFi.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>
#include <Wifi_selector.h>


// Change log
// 2019-05 Created by Yves Bonnefont
// 2019-05 added wifi selector
// 2019-05 rewrite with sensor and actuator classes

// Mode
bool debug = true;  //Affiche sur la console si True

#define WIFI_CONNECT_TIMEOUT 8000 // in millisec
WiFiAutoSelector wifiAutoSelector(WIFI_CONNECT_TIMEOUT);  

//AP definitions
#define wifi_ssid_A "Home_Etage"
#define wifi_ssid_B "Home_wifi"
#define wifi_password_A "mariepascale"
#define wifi_password_B "mariepascale"
#define wifi_ssid_C "La_Clancheuse"
#define wifi_password_C "laclancheuse"
#define wifi_ssid_D "Penn_Ty_Breizh"
#define wifi_password_D "marie-pascale"

// Sensors and actuators
class Sensor {
  public:
    char myName[20];
    byte myId;
    byte myPin;
    char myTopic[50];
    String myType;

    int begin(char* Name, byte Id, byte Pin, char* Topic, String Type);
    int senseAndPublish();
};

class Actuator {
  public:
    char myName[50];
    byte myId;
    byte myPin;
    char myTopic[50];
    String myType;

    int begin(char* Name, byte Id, byte Pin, char* Topic, String Type);
    int act(char* message);
};


float read_ds18B20(int);

// *********************************************************************** Creation of MQTT const and objects *****************************************// 
// MQTT HW
#define mqtt_server "192.168.0.60"
#define CLIENT_NAME "ESP_"

// MQTT buffers
char message_buff_payload[100];
// MQTT topics
char ESP_topic[50]; // /ESP_MAC, built in set-up
// manages periodic publish
unsigned long last_sent = 0;
int send_period = 30; // in seconds
// Header of callback function
void callback(char* topic, byte* payload, unsigned int length);
void sendMQTT(char *topic, float payload);



// ************************************************************************ Creation of top level objects ***********************************************//
WiFiClient espClient; // Wifi client
PubSubClient MQTTclient(mqtt_server, 1883, callback, espClient); // MQTT client
Sensor sensors[2]; // Sensors
Actuator actuators[2]; // Actuators


// ************************************************************************ Classes member functions **************************************************//
int Sensor::begin(char* Name, byte Id, byte Pin, char* Topic, String Type){
  // create topic
  strcpy(myName,Name);
  myId=Id;
  myPin=Pin;
  strcpy(myTopic,ESP_topic);
  strcat(myTopic,"/datas/");
  strcat(myTopic, Topic);
  myType=Type;
  return 0;
}

int Sensor::senseAndPublish(void){
  char pub_topic[80];
  
  Serial.print(myType);
  if(myType=="DHT22") {
    //read DHT
    DHT dht(myPin, DHT22);
    delay(200);
    float temp_f=dht.readTemperature();
    float humi_f= dht.readHumidity();
    if (debug) {
      Serial.println("Reading data");
      Serial.print("Temperature lue: ");
      Serial.println(temp_f);
      Serial.print("Humidite lue: ");
      Serial.println(humi_f);
    }  
    //Send data to MQTT broker
    sprintf(pub_topic, "%s%s",myTopic,"/temperature");
    sendMQTT(pub_topic, temp_f);
    sprintf(pub_topic, "%s%s",myTopic,"/humidite");
    sendMQTT(pub_topic, humi_f);
    return 0;
  }
  if(myType=="DS18B20") {
    sprintf(pub_topic, "%s%s",myTopic,"/temperature");
    sendMQTT(pub_topic, read_ds18B20 (myPin));
    return 0;
  }
  return -1; // sensor type has not been recogniezd in switch statment
}

int Actuator::begin(char* Name, byte Id, byte Pin, char* Topic, String Type){
  // create topic
  strcpy(myName,Name);
  myId=Id;
  myPin=Pin;
  strcpy(myTopic,ESP_topic);
  strcat(myTopic,"/orders/");
  strcat(myTopic,Topic);
  myType=Type;
  return 0;
}

int Actuator::act(char* message){
  if(myType=="relay") {
    if (strcmp(message,"1.0")==0) {
      digitalWrite(myPin,HIGH);
    } else {
      digitalWrite(myPin,LOW);
    }
    return 0;
  }
  return -1; // sensor type has not been recogniezd in switch statment

}

//*************************************************************************** Key functions *******************************************************//
//Connexion to best known WiFi 
int setup_wifi(){
  if(WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting wifi ");
    if(-1 < wifiAutoSelector.scanAndConnect()) {
      int connectedIndex = wifiAutoSelector.getConnectedIndex();
      Serial.print("to '");
      Serial.print(wifiAutoSelector.getSSID(connectedIndex));
      Serial.println("'. Done.");
      return 0;
    }else{
      Serial.println("failed.");
      return -1;
    }
  }
}

//Connexion - Reconnexion MQTT
void MQTTreconnect() {
  int tries = 0;
  char orders_topic[50];
  //Boucle jusqu'à obtenir une reconnexion
  Serial.print("Connexion au serveur MQTT...");
  while (!MQTTclient.connected() && tries < 5) {
    Serial.print(".");
    if (!MQTTclient.connect(ESP_topic)) {
      tries = tries + 1;
      delay(1000);
    }
  }
  if (tries < 5) {
    Serial.print("Connected as ");
    Serial.println(ESP_topic);
  }
  else {
    Serial.print("KO, erreur : ");
    Serial.print(MQTTclient.state());
  }
  
  // Subscribe to orders
  strcpy(orders_topic,ESP_topic);
  strcat(orders_topic,"/orders/#");
  MQTTclient.subscribe(orders_topic);
}


// Déclenche les actions à la récetion d'un message
void callback(char* topic, byte* payload, unsigned int length) {
  int i = 0;
  if ( debug ) {
    Serial.println("Message recu =>  topic: " + String(topic));
    Serial.print(length);
    Serial.println(" | longueur: " + String(length, DEC));
  }
  // create character buffer with ending null terminator (string)
  for (i = 0; i < length; i++) {
    message_buff_payload[i] = payload[i];
  }
  message_buff_payload[i] = '\0';

  String msgString = String(message_buff_payload);
  String topicString = String(topic);
  if ( debug ) {
    Serial.println("Payload: " + msgString);
  }
  // Any actions other than printing the message to stdout to be inserted here
  if(debug){
    Serial.println(sizeof(actuators)/sizeof(Actuator));
  }
  for (i=0; i < (sizeof(actuators)/sizeof(Actuator)) - 1; i++){
    if(strcmp(topic,actuators[i].myTopic)==0){
      actuators[i].act(message_buff_payload);
      return;
    }
  }
}

void sendMQTT(char *topic, float payload)
{
  char message_buffer[20];
  char* message;

  message = dtostrf(payload, 3, 1, message_buffer); //
  if (debug){
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(message);
  }
  Serial.println(MQTTclient.publish(topic, message));
}

void setup() {
  char MAC_buffer[18];
  Serial.begin(9600);
  Serial.println("Going through set-up process");

  // Start the Ethernet connection
  // List known wifi
  wifiAutoSelector.add(wifi_ssid_B, wifi_password_B);
  wifiAutoSelector.add(wifi_ssid_A, wifi_password_A);
  wifiAutoSelector.add(wifi_ssid_C, wifi_password_C);
  wifiAutoSelector.add(wifi_ssid_D, wifi_password_D);
  delay(100);

  if (debug) {
    for (int i=0; i<wifiAutoSelector.getCount();i++){
      Serial.print(wifiAutoSelector.getSSID(i));
      Serial.print(" ");
      Serial.println(wifiAutoSelector.getRSSI(i));
    }
  }

  // Connect to best wifi among available
  if (setup_wifi()==0) {
    delay(500);

    // Build MQTT topics
    // ESP core topic client id using last Character of MAC address
    strcat(ESP_topic,CLIENT_NAME);
    WiFi.macAddress().toCharArray(MAC_buffer,18);
    strcat(ESP_topic, MAC_buffer+9);
        
    // Create Sensors & Actuators
    sensors[0].begin("DHT",0,4,"DHT",String("DHT22")); // name, id, pin, topic, type - known types DHT22, DS18B20
    //sensors[1].begin("DS18B20",1,2,"DS18B20",String("DS18B20")); // name, id, pin, topic, type - known types DHT22, DS18B20

    actuators[0].begin("relay",0,16,"relay",String("relay")); // name, id, pin, topic, type - known types relay

    // Connect to MQTT broker
    MQTTclient.setServer(mqtt_server, 1883);    //Configuration de la connexion au serveur MQTT
    MQTTclient.setCallback(callback);  //La fonction de callback qui est executée à chaque réception de message
    MQTTreconnect();
    MQTTclient.loop();

    // Blink success
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
    delay(500);                       // wait for a second
    digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage LOW
    delay(500);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
    delay(500);                       // wait for a second
    digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage LOW

  }
  else {
    // blink failure
    Serial.println("Unable to connect to a known wifi network");
    pinMode(LED_BUILTIN, OUTPUT);
    while (true){
      digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
      delay(100);                       // wait for a second
      digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage LOW
      delay(250);                       // wait for a second
      digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
      delay(100);                       // wait for a second
      digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage LOW
      delay(500);                       // wait for a second
    }
  }
}

void loop() {
  int i;
  // If publish interval is completed
  if (millis() - last_sent > send_period * 1000)
  {
    // reconnect to wifi if needed to correct for HW errors
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Reconnecting to Wifi ater drop");
      setup_wifi();
    }
    // Reconnect to MQTT broker and re-subscribe
    if (!MQTTclient.connected()) {
      Serial.println("Reconnecting MQTT after drop");
      MQTTreconnect();
    }
    // sense and publish all available sensors
    for (i=0; i<(sizeof(sensors)/sizeof(Sensor)); i++){
      Serial.print("Working sensor #: ");
      Serial.println(i);
      if (sensors[i].senseAndPublish()==0){
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.println(" published.");
      }
      else {
        Serial.println("Sensor not recognized");
      }
    }
    last_sent = millis();
  }
  if (last_sent > millis()) { // means millis has been reset => reset last_sent
    last_sent = millis();
  }
  MQTTclient.loop();
}


/*********************************** Sensors code *********************************/

/* DS18B20 on one_wire bus*/
float read_ds18B20 (int oneWirePin){
  float temptemp;
  OneWire oneWire(oneWirePin);
  DallasTemperature sensors(&oneWire);
  do {
  sensors.requestTemperatures(); // Send the command to get temperatures
  temptemp = sensors.getTempCByIndex(0);
  Serial.print("Temperature: ");
  Serial.println(temptemp);
  } while (temptemp == 85.0 || temptemp == (-127.0));
  return temptemp;
}
