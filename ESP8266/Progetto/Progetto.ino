#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include "settings.h"
#include <ThingSpeak.h>

String st;
String content;
int statusCode;
bool data_available = false;  

WiFiClient client;

//Establishing Local server at port 80 whenever required
ESP8266WebServer server(80);

unsigned long myChannelNumber = CH_ID;
const char * myWriteAPIKey = APIKEY;

/* holds the current serial line */
String currentLine;

int lastTemp;
int lastHumidity;
unsigned long lastSendTime = 0;


void read_serial_packet();
void send_to_thingsspeak();
void connect_to_wifi(String ssid, String pass);
bool testWifi();
void launchWeb();
void setupAP();
void createWebServer();

void setup() {
  //debug serial (TX) and data receive serial (RX)
  //when programming the ESP8266, remove the connection
  //from the STM32 to the ESP8266's RX pin!
  //can also be handled with .swap() so that RX and TX pins
  //get swapped to 2 different pins, but this way we can't
  //get the debug output from a NodeMCU
  Serial.begin(115200);
  WiFi.disconnect();
  EEPROM.begin(512); //Initialasing EEPROM
  delay(10);


  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  pinMode(2, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // Led spento
  digitalWrite(2, HIGH);
  
  //wait at max 1 second for a string on the serial
  Serial.setTimeout(1000);
  Serial.println("Firwmare start!");

  Serial.println("Reading EEPROM ssid");
  String esid;
  for (int i = 0; i < 32; ++i){
    esid += char(EEPROM.read(i));
  }
  Serial.println();
  Serial.print("SSID: ");
  Serial.println(esid);


  Serial.println("Reading EEPROM pass");
  String epass = "";
  for (int i = 32; i < 96; ++i){
    epass += char(EEPROM.read(i));
  }
  Serial.print("PASS: ");
  Serial.println(epass);

  WiFi.mode(WIFI_STA);
	ThingSpeak.begin(client); // Initialize ThingSpeak
  connect_to_wifi(esid.c_str(), epass.c_str());
}

void loop() {
  read_serial_packet();
  if(data_available){
    send_to_thingsspeak();
    data_available = false;
  } 
}

bool testWifi(void){
  int c = 0;
  Serial.println("Waiting for Wifi to connect");
  while ( c < 20 ) {
    if (WiFi.status() == WL_CONNECTED){
      return true;
    }
    delay(500);
    Serial.print("*");
    c++;
  }
  Serial.println("");
  Serial.println("Connect timed out, opening AP");
  return false;
}

void read_serial_packet() {
  if(Serial.available()) {
    data_available = true;
    currentLine = Serial.readStringUntil('\n');

    //Serial.println("GOT LINE");
    //Serial.println(currentLine);

    int commaSplitIndex = currentLine.indexOf(',');
    if (commaSplitIndex > 0) {
      String tempStr = currentLine.substring(0, commaSplitIndex);
      String humString = currentLine.substring(commaSplitIndex + 1);

      //Serial.println("[Update] Temp: " + tempStr + " Hum: " + humString);

      lastTemp = tempStr.toInt();
      lastHumidity = humString.toInt();
    }
  }

}

void connect_to_wifi(String ssid, String pass) {
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);
    
  WiFi.begin(ssid, pass); // Connect to WPA/WPA2 network. Change this line if using open or WEP network
  delay(1000);
  if (testWifi()){
    Serial.println("Succesfully Connected!!!");
    digitalWrite(LED_BUILTIN, LOW); // led acceso
    
    digitalWrite(2, LOW); 
    //delay(100); 
    digitalWrite(2, HIGH);         
  }else{
    Serial.println("Turning the HotSpot On");
    //launchWeb();
    setupAP();// Setup HotSpot
    
    Serial.println();
    Serial.println("Waiting...");
    while ((WiFi.status() != WL_CONNECTED)){
      delay(20);
      server.handleClient();
    }
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(2, LOW); 
    //delay(100); 
    digitalWrite(2, HIGH);  
  }
}

void launchWeb(){
 /* Serial.println("");
  if (WiFi.status() == WL_CONNECTED)
    Serial.println("WiFi connected");
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("SoftAP IP: ");
  Serial.println(WiFi.softAPIP());
  */
  createWebServer();
  // Start the server
  server.begin();
  //Serial.println("Server started");
}

void setupAP(void){
  //WiFi.mode(WIFI_STA); ??
  WiFi.disconnect();
  delay(100);
  int n = WiFi.scanNetworks();
  //Serial.println("scan done");
  if (n == 0)
    Serial.println("no networks found");
  /*else
  {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i)
    {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " " : "*");
      delay(10);
    }
  }
  Serial.println("");
  */
  st = "<ol>";
  for (int i = 0; i < n; ++i){
    // Print SSID and RSSI for each network found
    st += "<li>";
    st += WiFi.SSID(i);
    st += " (";
    st += WiFi.RSSI(i);

    st += ")";
    st += (WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " " : "*";
    st += "</li>";
  }
  st += "</ol>";
  delay(100);
  WiFi.softAP("ESP8266", "");
  //Serial.println("Initializing_softap_for_wifi credentials_modification");
  launchWeb();
  //Serial.println("over");
}

void createWebServer(){
  {
    server.on("/", []() {
      content = "<!DOCTYPE HTML>\r\n<html>";
      content += "\r\nReti disponibili:\r\n";
      content += "<p>";
      content += st;
      content += "</p><form method='get' action='setting'><label>SSID: </label><input name='ssid' length=32><br><label>PASSWORD: </label><input name='pass' length=64><input type='submit'></form>";
      content += "<form action=\"/scan\" method=\"POST\"><input type=\"submit\" value=\"scan\"></form>";
      content += "</html>";
      server.send(200, "text/html", content);
    });

    server.on("/scan", []() {
      setupAP();
      IPAddress ip = WiFi.softAPIP();
      String ipStr = String(ip[0]) + '.' + String(ip[1]) + '.' + String(ip[2]) + '.' + String(ip[3]);

      content = "<!DOCTYPE HTML>\r\n<html>go back";
      server.send(200, "text/html", content);
    });

    server.on("/setting", []() {
      String qsid = server.arg("ssid");
      String qpass = server.arg("pass");
      if (qsid.length() > 0 && qpass.length() > 0) {
        //Serial.println("clearing eeprom");
        for (int i = 0; i < 96; ++i) {
          EEPROM.write(i, 0);
        }
        Serial.println(qsid);
        Serial.println("");
        Serial.println(qpass);
        Serial.println("");

        Serial.println("writing eeprom ssid:");
        for (int i = 0; i < qsid.length(); ++i)
        {
          EEPROM.write(i, qsid[i]);
          Serial.print("Wrote: ");
          Serial.println(qsid[i]);
        }
        Serial.println("writing eeprom pass:");
        for (int i = 0; i < qpass.length(); ++i)
        {
          EEPROM.write(32 + i, qpass[i]);
          Serial.print("Wrote: ");
          Serial.println(qpass[i]);
        }
        EEPROM.commit();

        content = "{\"Success\":\"saved to eeprom... reset to boot into new wifi\"}";
        statusCode = 200;
        ESP.reset();
      } else {
        content = "{\"Error\":\"404 not found\"}";
        statusCode = 404;
        Serial.println("Sending 404");
      }
      server.sendHeader("Access-Control-Allow-Origin", "*");
      server.send(statusCode, "application/json", content);

    });
  }}


void send_to_thingsspeak() {
  //update field 1 (temperature) and 2 (humidity)
  ThingSpeak.setField(1, lastTemp);
  ThingSpeak.setField(2, lastHumidity);
  //send update via HTTPS REST call
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if (x == 200) {
    Serial.println("Channel update successful.");
  } else {
    Serial.println(
        "Problem updating channel. HTTP error code " + String(x));
  }
  Serial.println("I'm awake, but I'm going into deep sleep mode until RESET pin is connected to a LOW signal");
  ESP.deepSleep(0); 
}
