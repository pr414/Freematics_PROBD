/*************************************************************************
* OBD-II/MEMS/GPS Data Logger Sketch for Freematics ONE+
* Distributed under BSD license
* Visit http://freematics.com/products/freematics-one-plus for more information
* Developed by Stanley Huang <stanley@freematics.com.au>
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.

* FORK MADE BY PAOLO RIVA - 723854 UNIVERSITA' DEGLI STUDI
* DI BRESCIA - DII ELUX LABORATORY - PROJECT SUPERVISION BY PHD. PAOLO BELLAGENTE
*************************************************************************/

#include "FreematicsOBD.h"
#include "config.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Time.h> //NON USATA PER ORA
#include <ArduinoJson.h>
//#include <fstream>
//#include <string>
#include <SPIFFS.h>
#include <vector>
#include <iostream>

using namespace std;
//#define TORQUE_CONVERSION 200

uint16_t MMDD = 0;
uint32_t UTC = 0;
uint32_t startTime = 0;
uint32_t pidErrors = 0;

COBDSPI obd;
//PID OBD di cui fare il Logging
const int numOfPIDs = 4;
//byte pids[]= {PID_RPM, PID_SPEED, PID_THROTTLE, PID_ENGINE_REF_TORQUE, PID_ENGINE_TORQUE_DEMANDED, PID_HYBRID_BATTERY_PERCENTAGE, PID_ENGINE_TORQUE_PERCENTAGE};
//char *stamps[7]= {"rpm", "speed", "throttle", "torque_reference_erogation", "torque_demanded", "battery_percentage", "torque_erogated"};
//int values[4] = {0,0,0,0,0,0,0};
byte pids[]= {PID_RPM, PID_SPEED, PID_THROTTLE, PID_ENGINE_REF_TORQUE};
char *stamps[4]= {"rpm", "speed", "throttle", "torque_reference_erogation"};
int values[4] = {0,0,0,0};

//JSON Document
const int capacity = JSON_OBJECT_SIZE(numOfPIDs*2+2); //bytes richiesti per avere "numOfPIDs" valori da serializzare + timestamp
DynamicJsonDocument jsondoc(capacity);

//Credenziali di accesso all'Hotspot
const char* ssid = "OnePlus-Paolo";
const char* wifipassword = "Woahcarlo$#";

//Indirizzo e parametri del Broker MQTT
const char* client_name = "PRivaClient";
const char* mqtt_server = "lab-elux.unibs.it";
const int port = 50009;
const char* topicToPublish = "/telemetry/obd/car1";
const char* username = "priva";
const char* password = "KObM2u96t%&M#e%%ShZ#H!5Ls$0UEN^wXLuegI@*1rudAPeQE"; //CHECK %
const char* certificate_path = "/intermediate_ca.pem";

//char* PROGMEM mqtt_certificate = "";
//FILE *ca = fopen(ca_certificate, "rb");
const char *x509CA PROGMEM = R"EOF("
-----BEGIN CERTIFICATE-----
MIIENjCCAx6gAwIBAgIUGFQHYrPCIetdZ2Qy2cUrSeuX/n0wDQYJKoZIhvcNAQEL
BQAwgaAxCzAJBgNVBAYTAklUMRAwDgYDVQQHEwdCcmVzY2lhMR4wHAYDVQQKExVV
bml2ZXJzaXR5IG9mIEJyZXNjaWExQDA+BgNVBAsTN2VMVVggTGFib3JhdG9yeSAt
IERlcGFydG1lbnQgb2YgSW5mb3JtYXRpb24gRW5naW5lZXJpbmcxHTAbBgNVBAMT
FGVMVVgtSW50ZXJtZWRpYXRlLUNBMB4XDTE4MDYwODA5MzYwMFoXDTIzMDQyMzA5
MzYwMFowgaAxCzAJBgNVBAYTAklUMRAwDgYDVQQHEwdCcmVzY2lhMR4wHAYDVQQK
ExVVbml2ZXJzaXR5IG9mIEJyZXNjaWExQDA+BgNVBAsTN2VMVVggTGFib3JhdG9y
eSAtIERlcGFydG1lbnQgb2YgSW5mb3JtYXRpb24gRW5naW5lZXJpbmcxHTAbBgNV
BAMTFGVMVVgtSW50ZXJtZWRpYXRlLUNBMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8A
MIIBCgKCAQEAvKpFuiUfCBrvTFzEZR3E6xjAUyvo02ZxP1HT3kHEcaU3BRB/CBL3
pVbDxemwkBlXKjHMfmiCpRX3aVSgR7qgNRjuMgG9XrNXpapMPIzEQas6m/2vSHZ+
3gMIUemz8uLFNQFWcYNAYWU+cYaOcKYdL3EKLxCd8mE3ufkZoWBoYvV33/Ef8aox
3X+JYb0J5H1qGLje4dvtfcIdAOXUO81K3XQgdi1VGE3KOdhhsjYTtEbKrQS5P9yO
HLpNDvJctLZWEBjR5kdsux21TCXqrbVF3whrPWQJVuVZYbnRkVpmrMvX1Lbnwk0V
1YuKllU1iG1jpgaEwNqjTzxVdfvUylwxGQIDAQABo2YwZDAOBgNVHQ8BAf8EBAMC
AQYwEgYDVR0TAQH/BAgwBgEB/wIBAjAdBgNVHQ4EFgQU7iyn4WKaJ0lyxpNPAht6
Pc1EKSYwHwYDVR0jBBgwFoAU7iyn4WKaJ0lyxpNPAht6Pc1EKSYwDQYJKoZIhvcN
AQELBQADggEBAE3kbrlrPpeCFfOvZQzx2fpHsjCqhG904Tj9rg4idP+UmzSsgcn8
wGy5UFrYkyRAAryuPhAS47Uefigkk4mVIslU7Dl3xxAw2TCpTZs5eTOExLGAY0lH
g6ff3e/8iRwWl8DS/yx1WFQUeLYVK7da24H7zyow4TELue+QjabsHL7dQlK7bfbI
r4nca5jejPNDTwK4D3zoQP9xBDqflJatjuU9/3MjqUKXh4RTXuEQpj+N16VgEsMp
BUzevFCF+JJfJW/AtmchNh8YvS2EkXIpdi3y1brkzj4wNxKYQ3AMY4E6D9SWbhbe
YhAvMkixXS8TJE2JNWTLHW+lbk1euAU79Qs=
-----END CERTIFICATE-----
")EOF";

//Variabili per WiFi/MQTT
WiFiClientSecure espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

//Variabili per Freematics ONE+
bool connected = false;
unsigned long pack_count = 0;

void setup_wifi() {
  delay(10);

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, wifipassword);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(150);
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("Local IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
    }
    else if(messageTemp == "off"){
      Serial.println("off");
    }
  }
}

void setup_mqtt() {
  //Lettura Certificato CA in "certificate"
  int c;
  int i=0;
  File cert_file = SPIFFS.open(certificate_path, FILE_READ);

  vector<String> v;
  if (!cert_file) {
    Serial.println("Failed to open CA Certificate");
    return;
  } else {
    delay(100);
    //v.push_back("\"\n");
    while (cert_file.available()) {
      v.push_back(cert_file.readString());
      //v.push_back("\n");
      delay(100);
    }
    //v.push_back("\"");
    cert_file.close();
  }

  String cert_toString = "";
  for (String s : v) {
    cert_toString.concat(s);
  }
  
  
  const char* cert_content = cert_toString.c_str();
  /*
    while (cert_file.available()) {
      cert_content[i++] = cert_file.read();
    } 
  */

  Serial.println("\nCertificate Content appended: ");
  Serial.printf("%x", *cert_content);
  Serial.println();

  //Setting del Certificato SSL
  //espClient.setCACert(cert_content);

  espClient.setCACert(x509CA);
  //Setting del Server MQTT e Callback sul topic
  client.setServer(mqtt_server, port);
  client.setCallback(callback);
}

void retryOBD()
    {
        if (obd.init()) return;
        // try to re-connect to OBD
        // turn off GPS power
    }

void reconnect() {
  // Loop fino alla riconnessione
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Tentativo di Connessione
    if (client.connect(client_name, username, password)) {
      Serial.println("connected");
      // Subscription al topic
      client.subscribe("/telemetry/obd/car1");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 2 seconds");
      // Attesa di 2 secondi
      delay(2000);
    }
  }
}

void setup()
{

  delay(2000);

  //USB serial initialization
    Serial.begin(115200);
    Serial.print("ESP32 ");
    Serial.print(ESP.getCpuFreqMHz());
    Serial.print("MHz ");
    
  //SPI FLASH FILE SYSTEM Setup
  if(!SPIFFS.begin(true)) {
    Serial.println("Error Initializating SPI File System");
    return;
  }

  //WiFi Setup
  setup_wifi();
  
  //MQTT Param Setup
  setup_mqtt();

  //OBD Setup
  byte ver = obd.begin();
  Serial.print("Firmware Ver. ");
  Serial.println(ver);

  Serial.print("OBD ");
  if (obd.init()) {
    Serial.println("Initialization OK");
    // retrieve VIN
    char buffer[128];
    if (obd.getVIN(buffer, sizeof(buffer))) {
      Serial.print("VIN:");
      Serial.println(buffer);
    }
  } else {
    Serial.println("Initialization ERR");
    retryOBD();
  }

  

}

void loop() {

  long timestamp = millis();
  
  for (byte i = 0; i < sizeof(pids) / sizeof(pids[0]); i++) {

    Serial.print('[');
    Serial.print(timestamp);
    Serial.print("] #");
    Serial.print(pack_count++);
    Serial.print(" ");

    int value_read;
    byte pid = pids[i];
    if (obd.readPID(pid, value_read)) {
      Serial.println(value_read);
    } else {
      pidErrors++;
      Serial.print("PID errors: ");
      Serial.println(pidErrors);
      if (obd.errors >= 3) {
        obd.reset();
      }
    }

    values[i] = value_read;
    //delay(1000);
  }
  //delay(1000);

  //MQTT Client-to-Broker Connection
  
  if (!client.connected()) {
    reconnect(); //Connette o Riconnette il Client al Broker specificato
  }
  client.loop(); //Processing In-Out of the MQTT Data


  //Inserimento dati nel JSON Document Pre-Serializzazione
  jsondoc["timestamp"] = timestamp;
  for (byte i = 0; i < sizeof(pids) / sizeof(pids[0]); i++) {
    jsondoc[stamps[i]] = values[i];
  }

  if (timestamp - lastMsg > 100) { //100 (0.1s) è l'intervallo di invio pacchetti MQTT
    lastMsg = timestamp;
    
    char jsonString[512]; //CHECK DIMENSIONE
    serializeJson(jsondoc, jsonString);
    //dtostrf(VALOREDACONVERTIRE, 1, 2, tempString); //Convert the value to a char array
    Serial.println(jsonString);
    client.publish(topicToPublish, jsonString);
  }

}



