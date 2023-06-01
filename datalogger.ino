/*************************************************************************
* THIS CODE IS BASED ON OBD-II/MEMS/GPS Data Logger Sketch for Freematics ONE+
* Distributed under BSD license
* Visit http://freematics.com/products/freematics-one-plus for more information
* Originally Developed by Stanley Huang <stanley@freematics.com.au>
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.

* THIS FORK WAS CREATED BY PAOLO RIVA - 723854 UNIVERSITA' DEGLI STUDI DI BRESCIA,
* THE CODE WAS WRITTEN ALONG A BACHELOR'S DEGREE THESIS PROJECT DEVELOPED IN EARLY 2023
* PROJECT SUPERVISION PROVIDED BY PHD. PAOLO BELLAGENTE
*
* THIS FIRMWARE READS VEHICLE DATA, FORMATS IT AND SENDS IT TO AN MQTT BROKER
* THE CODE IS SPECIFICALLY INTENDED FOR FREEMATICS ONE+ DEVICES
*
* COMMENTS IN ITALIAN
*************************************************************************/

#include "FreematicsOBD.h"
#include "config.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <vector>
#include <iostream>
using namespace std;

//Variabili d'ambiente
COBDSPI obd;

//PID OBD di cui fare il Logging
const int numOfPIDs = 4;
byte pids[]= {PID_RPM, PID_SPEED, PID_THROTTLE, PID_ENGINE_REF_TORQUE};
char *stamps[4]= {"rpm", "speed", "throttle", "torque_reference_erogation"};
int values[4] = {0,0,0,0};

//JSON Document
const int capacity = JSON_OBJECT_SIZE(numOfPIDs*2+2); //bytes richiesti per avere "numOfPIDs" valori da serializzare + timestamp + margine
DynamicJsonDocument jsondoc(capacity);
DynamicJsonDocument jsondocwifi(4096);

//Credenziali di accesso all'Hotspot
//const char* ssid = "OnePlus-Paolo";
//const char* wifipassword = "Woahcarlo$#";
const char* ssid = "";
const char* wifipassword = "";
const char* networks_path = "/wifi_networks.json";

//Parametri del Broker MQTT
const char* client_name = "PRivaClient";
const char* mqtt_server = "lab-elux.unibs.it";
const int port = 50009;
const char* topicToPublish = "/telemetry/obd/car1";
const char* username = "priva";
const char* password = "KObM2u96t%&M#e%%ShZ#H!5Ls$0UEN^wXLuegI@*1rudAPeQE";
const char* certificate_path = "/intermediate_ca.pem";
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
")EOF"; //Certificato SSL richiesto dal broker

//Variabili per WiFi/MQTT
WiFiClientSecure espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

//Variabili per Freematics ONE+
bool connected = false;
unsigned long pack_count = 0;
uint32_t pidErrors = 0;

/************************************
* Funzione di configurazione Wi-Fi 
* Funzione che accede ad un file JSON contenente tutte le credenziali Wi-Fi conosciute,
* per effettuare un tentativo di accesso ciclico ad ognuna di esse. 
* La funzione necessita di esser testata.
************************************/
void setup_wifi() {
  delay(10);
  int ctr = 0;
  
  //Apertura del file JSON dal SPIFFS contenente tutte le credenziali Wi-Fi conosciute
  File wifi_file = SPIFFS.open(networks_path, FILE_READ);
  if (!wifi_file) {
    Serial.println("Failed to open Wi-Fi Credential File");
    return;
  } else {
    deserializeJson(jsondocwifi, wifi_file); //Deserializzazione delle credenziali in jsondocwifi
  }

  //Tentativo di accesso ad una delle credenziali Wi-Fi conosciute
  while (true) {
    for (JsonObject credential : jsondocwifi.as<JsonArray>()) {
      ssid = credential["ssid"].as<const char*>();
      password = credential["password"].as<const char*>();

      Serial.println();
      Serial.print("Connecting to ");
      Serial.println(ssid);

      //Accesso al Wi-Fi con 4 tentativi ciclici su tutte le coppie SSID/password conosciute
      for (int i=0; i<4; i++) {
        WiFi.begin(ssid, wifipassword);
        Serial.printf("Attempt %d\n", i);
        delay(200);
        if (WiFi.status() == WL_CONNECTED) {
          Serial.println("");
          Serial.println("WiFi connected");
          Serial.println("Local IP address: ");
          Serial.println(WiFi.localIP());
          wifi_file.close();
          return;
        }
      }
    }
  }
}


/************************************
* Funzione di callback MQTT 
* Funzione che definisce il comportamento del client alla ricezione di un messaggio sul
* topic a cui risulta iscritto.
************************************/
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

  // Se il topic scelto è "esp32/output", è possibile controllare lo stato del client solamente inviando stringhe di controllo su di esso da un altro client (in questo caso, ciò non è stato usato)
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

/************************************
* Funzione di setup MQTT 
* Funzione che apre il certificato inserito staticamente come variabile globale, e che lo imposta come certificato SSL per effettuare la connessione al broker
* topic a cui risulta iscritto.
************************************/
void setup_mqtt() {
  //Lettura Certificato CA in "certificate"
  int c;
  int i=0;

  
  espClient.setCACert(x509CA);
  //Setting del Server MQTT e Callback sul topic
  client.setServer(mqtt_server, port);
  client.setCallback(callback);
}

/*************************************************************************
* Funzione di setup MQTT 
* Funzione che apre il certificato caricato all'interno del SPIFFS, e che lo imposta come certificato SSL per effettuare la connessione al broker
* topic a cui risulta iscritto. Funzionante, ma presenta instabilità che non consente la lettura 
**************************************************************************
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
      
        while (cert_file.available()) {
          cert_content[i++] = cert_file.read();
        } 
    Serial.println("\nCertificate Content appended: ");
    Serial.printf("%x", *cert_content);
    Serial.println();
  

  //Setting del Certificato SSL
  //espClient.setCACert(cert_content);
  
  //Setting del Server MQTT e Callback sul topic
  client.setServer(mqtt_server, port);
  client.setCallback(callback);
}
*************************************************************/

/*************************************************************************
* Funzione di riconnessione al veicolo tramite OBD 
* Funzione che reinvoca la funzione init() dell'oggetto OBD per ritentare il collegamento con il veicolo
**************************************************************************/
void retryOBD()
    {
        if (obd.init()) return;
    }

/*************************************************************************
* Funzione di connessione al broker MQTT
* Funzione che effettua tentativi ciclici di riconnessione al broker MQTT i cui parametri sono specificati come 
* variabili globali
**************************************************************************/
void connect_mqtt() {
  // Loop fino alla riconnessione
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Tentativo di Connessione
    if (client.connect(client_name, username, password)) {
      Serial.println("connected");
      // Subscription al topic per ricezione eventuale di comandi
      //client.subscribe("/telemetry/obd/car1"); CHECK se Necessario
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 2 seconds");
      // Attesa di 2 secondi
      delay(2000);
    }
  }
}

/*************************************************************************
* Funzione di setup del framework Arduino 
* Invocata a livello di framework durante il primo ciclo di esecuzione del dispositivo, ed eseguita una singola volta
**************************************************************************/

void setup()
{
  //Attesa di 2 secondi per il corretto power-up del dispositivo 
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
  
  //MQTT Broker Setup
  setup_mqtt();

  //OBD Setup
  byte ver = obd.begin();
  Serial.print("Firmware Ver. ");
  Serial.println(ver);

  Serial.print("OBD ");
  if (obd.init()) {
    Serial.println("Initialization OK");

    //Per verificare l'istanziazione corretta dell'oggetto OBD, viene richiesto il Vehicle Identification Number (VIN) per verificare che l'OBD risulti accessibile
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

/*************************************************************************
* Funzione di loop del framework Arduino 
* Invocata ciclicamente a livello di framework durante tutto il periodo di power-on del dispositivo
**************************************************************************/
void loop() {

  long timestamp = millis();
  
  for (byte i = 0; i < sizeof(pids) / sizeof(pids[0]); i++) {

    //Print del timestamp e del numero di pacchetto sulla seriale
    Serial.print('[');
    Serial.print(timestamp);
    Serial.print("] #");
    Serial.print(pack_count++);
    Serial.print(" ");

    //Lettura dei valori secondo la mappa di elementi della vettura definiti dall'array pids[]
    int value_read;
    byte pid = pids[i];
    if (obd.readPID(pid, value_read)) {
      Serial.println(value_read);
    } else { //Conteggio degli errori in lettura, il cui numero può invocare un reset della connessione OBD
      pidErrors++;
      Serial.print("PID errors: ");
      Serial.println(pidErrors);
      if (obd.errors >= 3) {
        obd.reset();
      }
    }
    values[i] = value_read;
  }

  //MQTT Client-to-Broker Connection
  if (!client.connected()) {
    connect_mqtt(); //Connette o Riconnette il Client al Broker specificato
  }
  client.loop(); //Processing In-Out dei pacchetti MQTT


  //Inserimento dati nel JSON Document Pre-Serializzazione
  jsondoc["timestamp"] = timestamp;
  for (byte i = 0; i < sizeof(pids) / sizeof(pids[0]); i++) {
    jsondoc[stamps[i]] = values[i];
  }

  //Serializzazione ed invio dei pacchetti al broker MQTT sul topic specificato come variabile globale
  if (timestamp - lastMsg > 100) { //100 (0.1s) è l'intervallo di invio pacchetti MQTT
    lastMsg = timestamp;
    char jsonString[512]; //CHECK DIMENSIONE
    serializeJson(jsondoc, jsonString);
    Serial.println(jsonString);
    client.publish(topicToPublish, jsonString);
  }

}



