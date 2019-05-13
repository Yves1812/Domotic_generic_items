// Connect to best known wifi and sends sensor data on a regular basis to MQTT Broker

#include <ESP8266WiFi.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <Wifi_selector.h>


// Change log
// 2019-05 Created by Yves Bonnefont
// 219-05 added wifi selector

#define WIFI_CONNECT_TIMEOUT 8000 // in millisec
WiFiAutoSelector wifiAutoSelector(WIFI_CONNECT_TIMEOUT);  

// Mode
bool debug = true;  //Affiche sur la console si True

//AP definitions
#define wifi_ssid_A "Home_Etage"
#define wifi_ssid_B "Home_wifi"
#define wifi_password_A "mariepascale"
#define wifi_password_B "mariepascale"
#define wifi_ssid_C "La_Clancheuse"
#define wifi_password_C "laclancheuse"
#define wifi_ssid_D "Penn_Ty_Breizh"
#define wifi_password_D "marie-pascale"

#define mqtt_server "192.168.0.60"
#define CLIENT_NAME "DHT22_"

// DHT22
#define DHTPIN 2     // what digital pin the DHT22 is conected to
#define DHTTYPE DHT22   // there are multiple kinds of DHT sensors

//Buffer qui permet de décoder les messages MQTT reçus
char message_buff_payload[100];

// manages periodic publish
unsigned long last_sent = 0;
int send_period = 600; // in seconds
char publishing_topic[50];

// Header of callback function
void callback(char* topic, byte* payload, unsigned int length);

//Création des objets
WiFiClient espClient;
PubSubClient MQTTclient(mqtt_server, 1883, callback, espClient);

//Connexion au réseau WiFi
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
  //Boucle jusqu'à obtenir une reconnexion
  Serial.print("Connexion au serveur MQTT...");
  while (!MQTTclient.connected() && tries < 5) {
    Serial.print(".");
    if (!MQTTclient.connect(publishing_topic)) {
      tries = tries + 1;
      delay(1000);
    }
  }
  if (tries < 5) {
    Serial.print("Connected as ");
    Serial.println(publishing_topic);
  }
  else {
    Serial.print("KO, erreur : ");
    Serial.print(MQTTclient.state());
  }
  MQTTclient.subscribe("Cuve_fioul");
}


// Déclenche les actions à la réception d'un message
void callback(char* topic, byte* payload, unsigned int length) {
  int i = 0;
  if ( debug ) {
    Serial.println("Message recu =>  topic: " + String(topic));
    Serial.print(" | longueur: " + String(length, DEC));
  }
  // create character buffer with ending null terminator (string)
  for (i = 0; i < length; i++) {
    message_buff_payload[i] = payload[i];
  }
  message_buff_payload[i] = '\0';

  String msgString = String(message_buff_payload);
  if ( debug ) {
    Serial.println("Payload: " + msgString);
  }
  // Any actions other than printing the message to stdout to be inserted here
}

void sendMQTT(char *topic, float payload)
{
  char message_buffer[20];
  char* message;

  message = dtostrf(payload, 3, 1, message_buffer); //
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(message);
  Serial.println(MQTTclient.publish(topic, message));
}

void setup() {
  int i;
  char MAC_buffer[18];
  Serial.begin(9600);

  // Start the Ethernet connection
  // List known wifi
  wifiAutoSelector.add(wifi_ssid_B, wifi_password_B);
  wifiAutoSelector.add(wifi_ssid_A, wifi_password_A);
  wifiAutoSelector.add(wifi_ssid_C, wifi_password_C);
  wifiAutoSelector.add(wifi_ssid_D, wifi_password_D);
  
  // Connect to best wifi among available
  if (setup_wifi()==0) {
    delay(500);

    // Build MQTT client id using last Character of MAC address
    strcat(publishing_topic,CLIENT_NAME);
    WiFi.macAddress().toCharArray(MAC_buffer,18);
    strcat(publishing_topic, MAC_buffer+9);

  //  for (i=0; i<wifiAutoSelector.getCount();i++){
  //    Serial.print(wifiAutoSelector.getSSID(i));
  //    Serial.print(" ");
  //    Serial.println(wifiAutoSelector.getRSSI(i));
  //  }

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
    Serial.println("Unable to connect to a known wifi network");
    // blink failure
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
  char pub_topic[50];
  float temp_f, humi_f;

  // If publish interval is completed
  if (millis() - last_sent > send_period * 1000)
  {
    // reconnect to wifi if needed to correct for HW errors
    if (WiFi.status() != WL_CONNECTED) {
      setup_wifi();
    }
    // Reconnect to MQTT broker and re-subscribe
    if (!MQTTclient.connected()) {
      MQTTreconnect();
    }

    //read DHT
    DHT dht(DHTPIN, DHTTYPE);
    Serial.println("Reading data");
    // Read current temperature
    temp_f=dht.readTemperature();
    humi_f= dht.readHumidity();
    Serial.print("Temperature lue: ");
    Serial.println(temp_f);
    Serial.print("Humidite lue: ");
    Serial.println(humi_f);
  
    //Send new temperature to MQTT broker
    sprintf(pub_topic, "%s%s",publishing_topic,"/temperature");
    sendMQTT(pub_topic, temp_f);
    sprintf(pub_topic, "%s%s",publishing_topic,"/humidite");
    sendMQTT(pub_topic, humi_f);

    last_sent = millis();
  }
  if (last_sent > millis()) { // means millis has been reset => reset last_sent
    last_sent = millis();
  }
  MQTTclient.loop();
}
