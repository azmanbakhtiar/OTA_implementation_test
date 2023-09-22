#include <WiFi.h>                         // provides support for the ESP32's WiFi functionality
#include <Firebase_ESP_Client.h>          // library to communicate with Firebase
#include <NTPClient.h>                    // library to retrieve time from an NTP server
#include "time.h"                         // library to support time-related functions
#include <EEPROM.h>                       // library to access the ESP32's built-in EEPROM memory
#include <WebServer.h>                    //library to set up a web server
#include <ATM90E32.h>                     //library for the ATM90E32 energy monitoring IC
#include "addons/TokenHelper.h"           // Provide the token generation process info.
#include "addons/RTDBHelper.h"            // Provide the RTDB payload printing info and other helper functions.

//added by azman
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>


//added by azman
const char * rootCACertificate = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh\n" \
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
"d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD\n" \
"QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT\n" \
"MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n" \
"b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG\n" \
"9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB\n" \
"CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97\n" \
"nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt\n" \
"43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P\n" \
"T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4\n" \
"gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO\n" \
"BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR\n" \
"TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw\n" \
"DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr\n" \
"hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg\n" \
"06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF\n" \
"PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls\n" \
"YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk\n" \
"CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=\n" \
"-----END CERTIFICATE-----\n";


String FirmwareVer = "3.7.0";
#define URL_fw_Version "https://raw.githubusercontent.com/azmanbakhtiar/azman/main/bin_version.txt"
#define URL_fw_Bin "https://raw.githubusercontent.com/azmanbakhtiar/azman/main/fw.bin"


unsigned long previousMillis = 0; // will store last time LED was updated
unsigned long previousMillis_2 = 0;
const long interval = 60;
const long mini_interval = 10;



#define wifiled 25
// the chip select pins for three ATM90E32 IC
const int CS_pin1 = 5;
const int CS_pin2 = 4;

//the gain values for the voltage and current channels
unsigned short VoltageGain1 = 57465;     
unsigned short VoltageGain2 = 57465;       

unsigned short CurrentGainCT1 = 31256;  
unsigned short CurrentGainCT2 = 31256; 
unsigned short CurrentGainCT3 = 31256; 
unsigned short CurrentGainCT4 = 31256;
unsigned short CurrentGainCT5 = 31256; 
unsigned short CurrentGainCT6 = 31256; 
/*
unsigned short CurrentGainCT1 = 31633;  
unsigned short CurrentGainCT2 = 31633; 
unsigned short CurrentGainCT3 = 31633; 
unsigned short CurrentGainCT4 = 31633;
unsigned short CurrentGainCT5 = 31633; 
unsigned short CurrentGainCT6 = 31633; 
*/
//the line frequency values for three IC
unsigned short LineFreq1 = 135;         
unsigned short LineFreq2 = 135;         

// the PGA gain value for three ics
unsigned short PGAGain1 = 21;            
unsigned short PGAGain2 = 21;            

// Variables to store voltages of different phases for
// Grid, Inverter and Home 
float Acin_voltageA, Acout_voltageA, Home_voltageA;
float Acin_voltageB, Acout_voltageB, Home_voltageB;
float Acin_voltageC, Acout_voltageC, Home_voltageC;

// Variables to store currents of different phases for
// Grid, Inverter and Home 
float Acin_currentA, Acout_currentA,Home_currentA;
float Acin_currentB, Acout_currentB,Home_currentB;
float Acin_currentC, Acout_currentC,Home_currentC;

// Variables to store power of different phases for
// Grid, Inverter and Home 
float Acin_realPower, Acout_realPower, Acin_powerFactor,Acout_powerFactor;
float Home_realPower, Home_powerFactor, temp, Acin_freq, Acout_freq;
float Acin_importEnergy, Acout_importEnergy, Home_importEnergy, Acin_exportEnergy;
float Acin_realPower1, Acin_realPower2, Acin_realPower3;
ATM90E32 eic1{}; //initialize the IC class
ATM90E32 eic2{}; //initialize the IC class

// variables used to keep track of the imported and exported energy values in different phases.

float Acin_importunits;
float Acin_exportunits;
float Acout_importunits;
float Home_importunits;

int address = 130;                 // The values of units are stored in EEPROM memory starting at address 130
float values[4];                   // "values" array stores the imported and exported energy values in different phases

//The "ButtonStyle" variable holds a CSS string for styling the appearance of a button on a web page.
String ButtonStyle = "input[type=Scan] {background-color: #1F62B9; color: white;padding: 12px 20px; border: none; border-radius: 4px;cursor: pointer;text-align: center;}";
int statusCode;  // The "statusCode" variable holds the HTTP status code returned from an HTTP request.

//The "ssid" and "passphrase" variables hold the credentials for the WiFi network that the ESP device is connecting to
const char* ssid = "text";
const char* passphrase = "text";

String esid;  // string to store wifi ssid
String epass = ""; // string to store wifi password
String eemail = ""; // string to store user login email
String eupass = ""; // string to store user login password
// The "st" variable is a string variable, while the "content" variable holds the content of an HTTP response.
String st;
String content;

unsigned long lastConnectionTime = 0;
const unsigned long connectionInterval = 5 * 60 * 1000; // 5 minutes in milliseconds

// Define NTP Client to get time
WiFiUDP ntpUDP;                   //The NTP client is initialized with a WiFiUDP object "ntpUDP".
NTPClient timeClient(ntpUDP);     //  NTP client used to retrieve the current date and time from a network time server

// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;

int firebaseFlag = 0;
int esprestartflag = 0;
// Insert Firebase project API Key
#define API_KEY "AIzaSyCemsMy4pYFofKX1KYIuekIr8vPWyzmfD4"

// Insert Authorized Email and Corresponding Password
#define USER_EMAIL "abraralishah00@gmail.com"
#define USER_PASSWORD "asdf1234"

// Insert RTDB URLefine the RTDB URL
#define DATABASE_URL "https://greenwend-1d975-default-rtdb.asia-southeast1.firebasedatabase.app"

// Define Firebase objects
FirebaseData fbdo;  // provides methods for reading and writing data to the Firebase Realtime Database.
FirebaseAuth auth; // provides methods for handling authentication.
FirebaseConfig config; // provides methods for configuring the connection to the Firebase Realtime Database.

const int BUTTON_PIN = 13; //  pin number of the button
const int LONG_PRESS_TIME = 1500; //  represents the time (in milliseconds) that the button needs to be held down before it is considered a long press.

volatile unsigned long pressedTime = 0; //"pressedTime" and "releasedTime" are used to store
volatile unsigned long releasedTime = 0; //the time (in milliseconds) when a button is pressed and released, respectively.
volatile boolean buttonPressed = false;// indicates whether the button is currently pressed or not.

// variables to store attempts made for UID
#define MAX_UID_GENERATION_ATTEMPTS 30  
int UID_GENERATION_ATTEMPTS = 0;
String uid;         // Variable to save USER UID

String databasePath; // Database main path (to be updated in setup with the user UID)
// Database child nodes
String GVAPath = "/Grid_voltage_A ";
String GVBPath = "/Grid_voltage_B";
String GVCPath = "/Grid_voltage_C";
String GIAPath = "/Grid_current_A";
String GIBPath = "/Grid_current_B";
String GICPath = "/Grid_current_C";
String GPPath = "/Grid_power";
String GpFPath = "/Grid_Power_Factor";
String IVAPath = "/Inverter_voltage_A";
String IVBPath = "/Inverter_voltage_B";
String IVCPath = "/Inverter_voltage_C";
String IIAPath = "/Inverter_current_A";
String IIBPath = "/Inverter_current_B";
String IICPath = "/Inverter_current_C";
String IPPath = "/Inverter_power";
String IpFPath = "/Inverter_Power_Factor";
String eUPath = "/Export_Units";
String iUPath = "/Import_Units";
String uGPath = "/Units_Generated";
String GFPath = "/Frequency";
String IFPath = "/Frequency";
String tPath = "/Temperature";
String HVAPath = "/Home_voltage_A";
String HVBPath = "/Home_voltage_B";
String HVCPath = "/Home_voltage_C";
String HIAPath = "/Home_current_A";
String HIBPath = "/Home_current_B";
String HICPath = "/Home_current_C";
String hPPath = "/Home_Power";
String hCPath = "/Home_Consumption";
String bVPath = "/Battery_Voltage";
String bIPath = "/Battery_Current";
String bPPath = "/Battery_Power";
// Parent Node (to be updated in every loop)
String parentPath;

FirebaseJson json; // the line creates an instance of the FirebaseJson class named json which allows you to parse and manipulate JSON data.

//A pointer to a constant character array (string) that contains the name of the NTP server to use for time synchronization. In this case, the server is "pool.ntp.org".
const char* ntpServer = "pool.ntp.org";
// A long integer that represents the time difference in seconds between the local time zone and Greenwich Mean Time (GMT). In this case, the offset is 18000 seconds, or 5 hours.
const long  gmtOffset_sec = 18000;
// An integer that represents the number of seconds to add to the GMT offset during daylight saving time. In this case, the offset is 3600 seconds, or 1 hour.
const int   daylightOffset_sec = 3600;

//Function Decalration
//Function launch web is used to display webpage to setup Wifi SSID and password
void launchWeb(void);
// Function setupAP to turn on hotspot of esp32.
void setupAP(void);

int counter= 0;


//Establishing Local server at port 80 whenever required
WebServer server(80);



void repeatedCall() {
  static int num=0;
  unsigned long currentMillis = millis();
  if ((currentMillis - previousMillis) >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    if (FirmwareVersionCheck()) {
      firmwareUpdate();
    }
  }
  if ((currentMillis - previousMillis_2) >= mini_interval) {
    previousMillis_2 = currentMillis;
    Serial.print("idle loop...");
    Serial.print(num++);
    Serial.print(" Active fw version:");
    Serial.println(FirmwareVer);
   if(WiFi.status() == WL_CONNECTED) 
   {
       Serial.println("wifi connected");
   }
   else
   {
      Serial.println("Waiting for WiFi");
      
  WiFi.begin(esid.c_str(), epass.c_str());
  //while (WiFi.status() != WL_CONNECTED) 
    delay(500);
    Serial.print(".");
  

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
   }
  }
}

void firmwareUpdate(void) {
  WiFiClientSecure client;
  client.setCACert(rootCACertificate);
  
  t_httpUpdate_return ret = httpUpdate.update(client, URL_fw_Bin);

  switch (ret) {
  case HTTP_UPDATE_FAILED:
    Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
    break;

  case HTTP_UPDATE_NO_UPDATES:
    Serial.println("HTTP_UPDATE_NO_UPDATES");
    break;

  case HTTP_UPDATE_OK:
    Serial.println("HTTP_UPDATE_OK");
    break;
  }
}
int FirmwareVersionCheck(void) {
  String payload;
  int httpCode;
  String fwurl = "";
  fwurl += URL_fw_Version;
  fwurl += "?";
  fwurl += String(rand());
  Serial.println(fwurl);
  WiFiClientSecure * client = new WiFiClientSecure;

  if (client) 
  {
    client -> setCACert(rootCACertificate);

    // Add a scoping block for HTTPClient https to make sure it is destroyed before WiFiClientSecure *client is 
    HTTPClient https;

    if (https.begin( * client, fwurl)) 
    { // HTTPS      
      Serial.print("[HTTPS] GET...\n");
      // start connection and send HTTP header
      delay(100);
      httpCode = https.GET();
      delay(100);
      if (httpCode == HTTP_CODE_OK) // if version received
      {
        payload = https.getString(); // save received version
      } else {
        Serial.print("error in downloading version file:");
        Serial.println(httpCode);
      }
      https.end();
    }
    delete client;
  }
      
  if (httpCode == HTTP_CODE_OK) // if version received
  {
    payload.trim();
    if (payload.equals(FirmwareVer)) 
    {
      Serial.printf("\nDevice already on latest firmware version:%s\n", FirmwareVer);
      return 0;
    } 
    else 
    {
      Serial.println(payload);
      Serial.println("New firmware detected");
      return 1;
    }
  } 
  return 0;  
}






/*
This function retrieves the current time as the number of seconds
since January 1, 1970 (known as the Unix epoch). 
It uses the getLocalTime function to obtain the current time,
and if it fails to do so, it returns 0. 
The retrieved time is stored in the variable now of type time_t.*/
unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}

/* 
The function launchWeb is used to set up a local web server.
 First, it prints the status of the WiFi connection and the local and 
 soft access point IP addresses. Then, it calls the function createWebServer 
 and starts the web server using the server.begin() method. 
 This function is used to create a web interface for 
 accessing and managing the device.*/

void launchWeb()
{
  Serial.println("");
  if (WiFi.status() == WL_CONNECTED)
    Serial.println("WiFi connected");
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("SoftAP IP: ");
  Serial.println(WiFi.softAPIP());
  createWebServer();
  // Start the server
  server.begin();
  Serial.println("Server started");
}

/*
The code sets up a soft access point ("Energy Meter" with password "123456789") 
for the ESP device, which enables it to connect to a WiFi network when no other 
WiFi network is available. The access point is launched by calling the launchWeb() 
function.
*/


void setupAP(void)
{
  WiFi.mode(WIFI_STA); // The setupAP() function first sets the WiFi mode to station (WIFI_STA) 
  WiFi.disconnect(); // and disconnects from any existing network.
  delay(100);
  int n = WiFi.scanNetworks(); //  It then scans for available WiFi networks 
  Serial.println("scan done");
  if (n == 0)
    Serial.println("no networks found");
  else
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
      delay(10);
    }
  }
  Serial.println("");
 // stores the network details (SSID and RSSI) in the string st which is later used 
// to create a web page to display the available networks.
  st = "<ol>";
  for (int i = 0; i < n; ++i)
  {
    // Print SSID and RSSI for each network found
    st += "<li>";
    st += WiFi.SSID(i);
    st += " (";
    st += WiFi.RSSI(i);

    st += ")";
    st += "</li>";
  }
  st += "</ol>";
  delay(100);
  WiFi.softAP("Energy Meter", "123456789");
  Serial.println("softap");
  launchWeb(); //The launchWeb() function is  called to create the web server, 
               //start it, and display the local IP address of the access point.
  Serial.println("over");
}
/*
This is an Arduino sketch that creates a web server using the ESP32 WiFi module.
The server provides a HTML web page for entering and submitting Wi-Fi credentials
(SSID and password), and user email and password. 
The HTML page is generated dynamically, and the CSS is included inline in the HTML
code to style the page. The IP address of the web server is displayed on the page.
When the submit button is pressed, the form data is sent to the server, 
which can then be retrieved by the code and used to connect to the Wi-Fi network
and set up a user account.
*/
void createWebServer()
{
  {
    server.on("/", []() {

      IPAddress ip = WiFi.softAPIP();
      String ipStr = String(ip[0]) + '.' + String(ip[1]) + '.' + String(ip[2]) + '.' + String(ip[3]);
      content = "<!DOCTYPE HTML>\r\n<html><h1>GREENWEND ENERGY </h1> ";
      content += "<head>\n"
                 "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
                 "<style>\n"
                 "body {font-family: Arial, Helvetica, sans-serif;}\n"
                 "* {box-sizing: border-box;}\n"
                 "\n"
                 "input[type=text], select, textarea {\n"
                 "  width: 100%;\n"
                 "  padding: 12px;\n"
                 "  border: 1px solid #ccc;\n"
                 "  border-radius: 4px;\n"
                 "  box-sizing: border-box;\n"
                 "  margin-top: 6px;\n"
                 "  margin-bottom: 16px;\n"
                 "  resize: vertical;\n"
                 "}\n"
                 "input[type=password], select, textarea {\n"
                 "  width: 100%;\n"
                 "  padding: 12px;\n"
                 "  border: 1px solid #ccc;\n"
                 "  border-radius: 4px;\n"
                 "  box-sizing: border-box;\n"
                 "  margin-top: 6px;\n"
                 "  margin-bottom: 16px;\n"
                 "  resize: vertical;\n"
                 "}\n"
                 "input[type=submit] {\n"
                 "  background-color: #04AA6D;\n"
                 "  color: white;\n"
                 "  padding: 12px 20px;\n"
                 "  border: none;\n"
                 "  border-radius: 4px;\n"
                 "  cursor: pointer;\n"
                 "}\n"
                 "\n"
                 "input[type=submit]:hover {\n"
                 "  background-color: #45a049;\n"
                 "}\n"
                 "\n"
                 "input[type=Scan] {\n"
                 "  background-color: #1F62B9;\n"
                 "  color: white;\n"
                 "  padding: 12px 20px;\n"
                 "  border: none;\n"
                 "  border-radius: 4px;\n"
                 "  cursor: pointer;\n"
                 "  text-align: center;\n"
                 "}\n"
                 "\n"
                 "input[type=Scan]:hover {\n"
                 "  background-color: #1F62B9;\n"
                 "}\n"
                 "\n"
                 "\n"
                 ".container {\n"
                 "  border-radius: 5px;\n"
                 "  background-color: #f2f2f2;\n"
                 "  padding: 20px;\n"
                 "  width: 600px;\n"
                 "}\n"
                 "</style>\n"
                 "</head>";

      content += ipStr;
      content += "<p>";
      content += st;
      content += " <input type=\"submit\" value=\"Scan\">";
      content += "<body>\n"
                 "\n"
                 "\n"
                 "<div class=\"container\">\n"
                 "   \n"
                 "  <form action=\"/setting\" method=\"get\">\n"
                 "    <label for=\"fname\">SSID</label>\n"
                 "    <input type=\"text\" id=\"fname\"  placeholder=\"Enter SSID\" autocomplete=\"current-password\" required=\"\" autofocus=\"\" name='ssid' length=32 >\n"
                 "\n"
                 "    <label for=\"lname\">PASSWORD </label>\n"
                 "    <input type=\"password\" id=\"lname\"  placeholder=\"Enter Password\" autocomplete=\"current-password\" required=\"\"name='pass' length=32>\n"
                 "\n"
                 "    <label for=\"subject\">USER EMAIL</label>\n"
                 "   <input type=\"text\" id=\"lname\" placeholder=\"Enter Email\" autocomplete=\"current-password\" required=\"\" name='email' length=32>\n"
                 "\n"
                 "    <label for=\"subject\">USER PASSWORD</label>\n"
                 "   <input type=\"text\" id=\"lname\" placeholder=\"Enter Password\" autocomplete=\"current-password\" required=\"\" name='upass' length=32>\n"
                 "\n"
                 "    <input type=\"submit\" value=\"Submit\">\n"
                 "  </form>\n"
                 "</div>\n"
                 "\n"
                 "</body>";
      content += "</html>";
      server.send(200, "text/html", content);
    });
    server.on("/scan", []() {
      //setupAP();
      IPAddress ip = WiFi.softAPIP();
      String ipStr = String(ip[0]) + '.' + String(ip[1]) + '.' + String(ip[2]) + '.' + String(ip[3]);

      content = "<!DOCTYPE HTML>\r\n<html>go back";
      server.send(200, "text/html", content);
    });

    server.on("/setting", []() {
      String qsid = server.arg("ssid");
      String qpass = server.arg("pass");
      String qemail = server.arg("email");
      String qupass = server.arg("upass");
      if ((qsid.length() > 0) && (qpass.length() > 0) && (qemail.length() > 0) && (qupass.length() > 0)) {
        Serial.println("clearing eeprom");
        for (int i = 0; i < 128; ++i) {
          EEPROM.write(i, 0);
        }
        Serial.println(qsid);
        Serial.println("");
        Serial.println(qpass);
        Serial.println("");
        Serial.println(qemail);
        Serial.println("");
        Serial.println(qupass);
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
        Serial.println("writing eeprom email:");
        for (int i = 0; i < qemail.length(); ++i)
        {
          EEPROM.write(64 + i, qemail[i]);
          Serial.print("Wrote: ");
          Serial.println(qemail[i]);
        }
        Serial.println("writing eeprom user password:");
        for (int i = 0; i < qupass.length(); ++i)
        {
          EEPROM.write(96 + i, qupass[i]);
          Serial.print("Wrote: ");
          Serial.println(qupass[i]);
        }
        EEPROM.commit();

        content = "{\"Success\":\"saved to eeprom... reset to boot into new wifi\"}";
        statusCode = 200;
        ESP.restart();
      } else {
        content = "{\"Error\":\"404 not found\"}";
        statusCode = 404;
        Serial.println("Sending 404");
      }
      server.sendHeader("Access-Control-Allow-Origin", "*");
      server.send(statusCode, "application/json", content);

    });
  }
}

/*
This code defines an interrupt service routine (ISR) function for a button press event. 
When the button is pressed, it will trigger the interrupt, which will execute this 
function.
*/
void buttonISR() {
  if (digitalRead(BUTTON_PIN) == LOW) { //  checks the state of the button by reading the digital signal on the BUTTON_PIN pin. If the signal is low (indicating that the button is pressed),
    pressedTime = millis(); // the current time in milliseconds is saved in the pressedTime variable.
  } else {// If the signal is high (indicating that the button has been released), 
    releasedTime = millis(); //  the current time in milliseconds is saved in the releasedTime variable
    buttonPressed = true; // buttonPressed flag is set to true.
  }
}




void setup() 
{  // This code is the setup() function in esp32 programming.
  Serial.begin(115200); // The function initializes the serial communication at a baud rate of 115200
    Serial.print("Active firmware version:");
  Serial.println(FirmwareVer);
  Serial.println("Start ATM90E32"); // prints "Start ATM90E32" to the serial output
  //  initializes the eic objects with the specified pins and gains
  eic1.begin(CS_pin1, LineFreq1, PGAGain1, VoltageGain1, CurrentGainCT1, CurrentGainCT2, CurrentGainCT3);
  eic2.begin(CS_pin2, LineFreq2, PGAGain2, VoltageGain2, CurrentGainCT4, CurrentGainCT5, CurrentGainCT6);
  delay(1000); // A delay of 1000 milliseconds is introduced.
  pinMode(wifiled, OUTPUT);

  // The next line prints "Disconnecting previously connected WiFi" to the serial output
  Serial.println("Disconnecting previously connected WiFi");
  WiFi.disconnect(); // disconnect from any previously connected WiFi network. 
  EEPROM.begin(512); //EEPROM is initialized with a size of 512 bytes.
  delay(10);
  pinMode(BUTTON_PIN, INPUT_PULLUP); // It sets the button pin as an input with an internal pull-up resistor 
  // The attachInterrupt function is then used to attach the ISR function to the 
  // button pin and trigger the ISR function on a change
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, CHANGE);
  Serial.println();
  Serial.println();
  Serial.println("Startup");
  Serial.println("Reading EEPROM ssid");
  // The code reads the SSID, password, email, and user password saved in EEPROM.
  for (int i = 0; i < 32; ++i)
  {
    esid += char(EEPROM.read(i));
  }
  Serial.println();
  Serial.print("SSID: ");
  Serial.println(esid);
  Serial.println("Reading EEPROM pass");

  
  for (int i = 32; i < 64; ++i)
  {
    epass += char(EEPROM.read(i));
  }
  Serial.print("PASS: ");
  Serial.println(epass);

  Serial.println("Reading EEPROM email");


  for (int i = 64; i < 96; ++i)
  {
    eemail += char(EEPROM.read(i));
  }
  Serial.print("EMAIL: ");
  Serial.println(eemail);
  Serial.println("Reading EEPROM pass");


  for (int i = 96; i < 128; ++i)
  {
    eupass += char(EEPROM.read(i));
  }
  Serial.print("USER PASS: ");
  Serial.println(eupass);
  // it begins a WiFi connection with the read credentials. 
  // The status of the WiFi connection is displayed through the Serial monitor.
  WiFi.begin(esid.c_str(), epass.c_str());
  Serial.print("Connecting to WiFi ..");
  Serial.println(WiFi.localIP());
  
 Serial.println();
  Serial.println("Waiting.");
 int connectionAttempts = 0;
  while ((WiFi.status() != WL_CONNECTED)) // while loop will repeatedly check the status of the Wi-Fi connection
  {
    connectionAttempts++; //  connectionAttempts counter is incremented with each iteration
    Serial.print("."); //  a dot is printed to the serial monitor to indicate that the code is still attempting to connect to Wi-Fi.
    delay(1000); // pause for one second before checking the Wi-Fi status again.
    server.handleClient();
    if (connectionAttempts == 15){
      break;
    }
  }
  
  /*
  If a connection is established before connectionAttempts reaches 15,
  the code will print "internet" to the serial monitor, 
  indicating that the Wi-Fi connection was successful.
  */
  if (connectionAttempts < 15){
    Serial.println("internet");
    
  }
  /*
  If connectionAttempts reaches 15 without establishing a connection,
  the break statement will exit the loop, and the code will print "no internet"
  */
  if (connectionAttempts == 15){
    Serial.println("no internet");
   
  }
  Serial.println();

  // This code is checking if the WiFi is connected
  if (WiFi.status() == WL_CONNECTED)
  {
    digitalWrite(wifiled, HIGH);
    configTime(0, 0, ntpServer); //  if it is, it configures the time using the NTP server 

  // Assign the api key (required)
  config.api_key = API_KEY;

  // Assign the user sign in credentials
  auth.user.email = eemail.c_str();
  auth.user.password = eupass.c_str();

  // Assign the RTDB URL (required)

  config.database_url = DATABASE_URL;

  Firebase.reconnectWiFi(true);
  fbdo.setResponseSize(4096); // The response size of the Firebase is set to 4096 bytes

  // Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

  // Assign the maximum retry of token generation
  config.max_token_generation_retry = 5;

  /*
  initializes the Firebase library with a configuration object and an authentication
  object. The configuration object contains information such as the database URL, 
  while the authentication object contains the credentials necessary to authenticate
  with the Firebase service. */
  Firebase.begin(&config, &auth);
  Serial.println("Getting User UID");
  /*
  waits for the user UID to be generated. The user UID is a unique identifier
  for the user that is currently authenticated with the Firebase service.
  The loop waits until the auth.token.uid field is not an empty string.
   */
  while ((auth.token.uid) == "") {
    UID_GENERATION_ATTEMPTS++; // increments the UID_GENERATION_ATTEMPTS  
    Serial.print('.');
    delay(1000);
    //If the number of attempts reaches the maximum number of attempts
    // specified by MAX_UID_GENERATION_ATTEMPTS, the loop breaks out.
    if (UID_GENERATION_ATTEMPTS == MAX_UID_GENERATION_ATTEMPTS)
    {
      break;
    }
  }
  UID_GENERATION_ATTEMPTS = 0; //After the loop, the UID_GENERATION_ATTEMPTS is set to 0
  // Print user UID
  uid = auth.token.uid.c_str();
  Serial.print("User UID: ");
  Serial.println(uid);
  Serial.println(uid.length());
  // Update database path
  databasePath = "/UsersData/" + uid + "/readings";
  Serial.println(databasePath);  
  // Initialize a NTPClient to get time
  timeClient.begin();
  // Set offset time in seconds to adjust for your timezone, for example:
  // GMT +1 = 3600
  // GMT +8 = 28800
  // GMT -1 = -3600
  // GMT 0 = 0
  timeClient.setTimeOffset(18000); // The offset time is set to 18000 seconds which is equal to 5 hours.
  firebaseFlag = 1;
  }
   
  // Reading Import, export, Generated units and consumption units from eeprom
  for (int i = 0; i < 4; i++) {
    EEPROM.get(address + i * sizeof(float), values[i]);
  }

  for (int i = 0; i < 4; i++) {
    Serial.println(values[i]);
  }
  Acin_importunits = values[0];
  Acout_importunits = values[1];
  Acin_exportunits = values[2];
  Home_importunits = values[3];

   Serial.println("Firmware update Starting..");
    repeatedCall ();

   
}

void loop() {
  unsigned long currentMillis = millis();  // returns the number of milliseconds that have elapsed since the esp was last reset.
  Acin_voltageA = eic1.GetLineVoltageA(); // Grid Phase A voltage
  Acin_voltageB = eic1.GetLineVoltageB(); // Grid Phase B voltage
  Acin_voltageC = eic1.GetLineVoltageC(); // Grid Phase B voltage
  
  Acout_voltageA = eic2.GetLineVoltageA(); // Inverter Phase A voltage
  Acout_voltageB = eic2.GetLineVoltageB(); // Inverter Phase B voltage
  Acout_voltageC = eic2.GetLineVoltageC(); // Inverter Phase C voltage

  Home_voltageA = eic1.GetLineVoltageA(); // Home Phase A voltage
  Home_voltageB = eic1.GetLineVoltageB(); // Home Phase B voltage
  Home_voltageC = eic1.GetLineVoltageC(); // Home Phase C voltage

  Acin_currentA = eic1.GetLineCurrentA(); // Grid Phase A current
  Acin_currentB = eic1.GetLineCurrentB(); // Grid Phase B current
  Acin_currentC = eic1.GetLineCurrentC(); // Grid Phase C current
  
  Acout_currentA = eic2.GetLineCurrentA(); // Inverter Phase A current
  Acout_currentB = eic2.GetLineCurrentB(); // Inverter Phase B current
  Acout_currentC = eic2.GetLineCurrentC(); // Inverter Phase C current

  
  Acin_realPower = eic1.GetTotalActivePower(); // Grid Power
  Acin_realPower1 = eic1.GetActivePowerA();
  Acin_realPower2 = eic1.GetActivePowerB();
  Acin_realPower3 = eic1.GetActivePowerC();
  Acout_realPower = eic2.GetTotalActivePower(); // Inverter Power
  Home_realPower = Acin_realPower + Acout_realPower; // Home Power
  
  Acin_powerFactor = eic1.GetTotalPowerFactor(); // Grid Power Factor
  Acout_powerFactor = eic2.GetTotalPowerFactor();  // Inverter Power Factor
  Home_powerFactor = (Acin_powerFactor + Acout_powerFactor)/2;  // Home Power Factor
  
  
  Acin_importEnergy = eic1.GetImportEnergy(); // Import Units
  Acin_importunits = Acin_importunits + Acin_importEnergy;
  Acout_importEnergy = eic2.GetImportEnergy(); // Units Generated
  Acout_importunits = Acout_importunits + Acout_importEnergy;
  Acin_exportEnergy = eic1.GetExportEnergy(); // Export Units
  Acin_exportunits = Acin_exportunits + Acin_exportEnergy;
  Home_importunits = Acout_importunits + Acin_importunits - Acin_exportunits;
  
  temp = eic1.GetTemperature();
  Acin_freq = eic1.GetFrequency();
  Acout_freq = eic2.GetFrequency();
  if (Acin_realPower1 < 0){
    Home_currentA = Acout_currentA - Acin_currentA;
  }
  if (Acin_realPower1 > 0){
    Home_currentA = Acin_currentA + Acout_currentA;
  }
  if (Acin_realPower2 < 0){
    Home_currentB = Acout_currentB - Acin_currentB;
  }
  if (Acin_realPower2 > 0){
    Home_currentB = Acin_currentB + Acout_currentB;
  }
  if (Acin_realPower3 < 0){
    Home_currentC = Acout_currentC - Acin_currentC ;
  }
  if (Acin_realPower3 > 0){
    Home_currentC = Acin_currentC + Acout_currentC;
  }
 
  if (Acout_realPower < 0){
    Acout_realPower = 0;
    Home_realPower = Acin_realPower;
  }
  // Saving Units Data to Eeprom
  values[0] = Acin_importunits;
  values[1] = Acout_importunits;
  values[2] = Acin_exportunits;
  values[3] = Home_importunits;

  for (int i = 0; i < 4; i++) {
      EEPROM.put(address + i * sizeof(float), values[i]);
    }
    // Commit the data to the EEPROM
    EEPROM.commit();
  
  if (buttonPressed) { //  If the button is pressed, the code will execute the following block of code.
    buttonPressed = false; // buttonPressed is set to false to avoid continuously executing the block of code.
    /* the duration of the button press is calculated by subtracting
     the time the button was released (releasedTime) from the time 
     the button was pressed (pressedTime).
    The result is stored in the variable pressDuration. */
    long pressDuration = releasedTime - pressedTime;
    //  if the duration of the button press
    // is greater than a constant value LONG_PRESS_TIME
    if (pressDuration > LONG_PRESS_TIME) {
    
      Serial.println("A long press is detected");
      Serial.println("Turning the HotSpot On");
      WiFi.disconnect(); // Disconnects from the current WiFi network
      WiFi.begin("Error", "error");
      setupAP();// Setup HotSpot
      // The code then enters a loop
      // that waits for the device to connect to the new WiFi network. 
       while ((WiFi.status() != WL_CONNECTED))
       {
        Serial.print(".");
        delay(100);
        server.handleClient();
        }
    }
  }
   
  // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
  if (currentMillis - lastConnectionTime >= connectionInterval) {
    lastConnectionTime = currentMillis;
    if (WiFi.status() != WL_CONNECTED) {
      digitalWrite(wifiled, LOW);
      esprestartflag++;
      Serial.println("WiFi disconnected. Reconnecting...");
      WiFi.disconnect();
      WiFi.begin(esid.c_str(), epass.c_str());
      int count = 0;
      while (WiFi.status() != WL_CONNECTED) {
         digitalWrite(wifiled, LOW);
        
        count++;
        delay(1000);
        Serial.print(".");
        if (count == 10){
      break;
    }
      }
      if (esprestartflag == 4){
        ESP.restart();
      }
      
    }
  }
  if (WiFi.status() == WL_CONNECTED){
     digitalWrite(wifiled, HIGH);
    esprestartflag = 0;
      if (firebaseFlag == 0){
         configTime(0, 0, ntpServer);

  // Assign the api key (required)
  config.api_key = API_KEY;

  // Assign the user sign in credentials
  auth.user.email = eemail.c_str();
  auth.user.password = eupass.c_str();

  // Assign the RTDB URL (required)
  config.database_url = DATABASE_URL;

  Firebase.reconnectWiFi(true);
  fbdo.setResponseSize(4096);

  // Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

  // Assign the maximum retry of token generation
  config.max_token_generation_retry = 5;

  // Initialize the library with the Firebase authen and config
  Firebase.begin(&config, &auth);
  Serial.println("Getting User UID");
  while ((auth.token.uid) == "") {
    UID_GENERATION_ATTEMPTS++;
    Serial.print('.');
    delay(1000);
    if (UID_GENERATION_ATTEMPTS == MAX_UID_GENERATION_ATTEMPTS)
    {
      break;
    }
  }
  UID_GENERATION_ATTEMPTS = 0;
      // Print user UID
  uid = auth.token.uid.c_str();
  Serial.print("User UID: ");
  Serial.println(uid);
  Serial.println(uid.length());
  // Update database path
  databasePath = "/UsersData/" + uid + "/readings";
  // Initialize a NTPClient to get time
  Serial.println(databasePath);
  timeClient.begin();
  // Set offset time in seconds to adjust for your timezone, for example:
  // GMT +1 = 3600
  // GMT +8 = 28800
  // GMT -1 = -3600
  // GMT 0 = 0
  timeClient.setTimeOffset(18000);
  firebaseFlag = 1;
      }
      if (uid.length()<28)
      {
        while ((auth.token.uid) == "") {
        UID_GENERATION_ATTEMPTS++;
        Serial.print('.');
        delay(1000);
        if (UID_GENERATION_ATTEMPTS == MAX_UID_GENERATION_ATTEMPTS )
        {
          break;
        }
      }
    UID_GENERATION_ATTEMPTS  = 0;
        // Print user UID
    uid = auth.token.uid.c_str();
    Serial.print("User UID: ");
    Serial.println(uid);
    Serial.println(uid.length());
    // Update database path
    databasePath = "/UsersData/" + uid + "/readings";
    // Initialize a NTPClient to get time
    Serial.println(databasePath);
    }
    if (uid.length()>=28)
      {
      counter ++;
    if (counter >= 8)
    {
    if (Firebase.ready()){
      while(!timeClient.update()) {
      timeClient.forceUpdate();
    }
    // The formattedDate comes with the following format:
    // 2018-05-28T16:00:13Z
    // We need to extract date and time
    formattedDate = timeClient.getFormattedDate();

    // Extract date
    int splitT = formattedDate.indexOf("T");
    dayStamp = formattedDate.substring(0, splitT);
    Serial.print("DATE: ");
    Serial.println(dayStamp);
    // Extract time
    timeStamp = formattedDate.substring(splitT+1, formattedDate.length()-1);
    Serial.print("HOUR: ");
    Serial.println(timeStamp);


      parentPath= databasePath + "/" + String(dayStamp)+ "/" + String(timeStamp);
      json.set(GVAPath.c_str(), String(Acin_voltageA));
      json.set(GVBPath.c_str(), String(Acin_voltageB));
      json.set(GVCPath.c_str(), String(Acin_voltageC));
      json.set(GIAPath.c_str(), String(Acin_currentA));
      json.set(GIBPath.c_str(), String(Acin_currentB));
      json.set(GICPath.c_str(), String(Acin_currentC));
      json.set(GPPath.c_str(), String(Acin_realPower));    
      json.set(IVAPath.c_str(), String(Acout_voltageA));
      json.set(IVBPath.c_str(), String(Acout_voltageB));
      json.set(IVCPath.c_str(), String(Acout_voltageC));  
      json.set(IIAPath.c_str(), String(Acout_currentA));
      json.set(IIBPath.c_str(), String(Acout_currentB));
      json.set(IICPath.c_str(), String(Acout_currentC));
      json.set(IPPath.c_str(), String(Acout_realPower));
      json.set(GpFPath.c_str(), String(Acin_powerFactor));
      json.set(IpFPath.c_str(), String(Acout_powerFactor));
      json.set(eUPath.c_str(), String(Acin_exportunits));
      json.set(iUPath.c_str(), String(Acin_importunits));  
      json.set(uGPath.c_str(), String(Acout_importunits));
      json.set(HVAPath.c_str(), String(Home_voltageA));
      json.set(HVBPath.c_str(), String(Home_voltageB));
      json.set(HVCPath.c_str(), String(Home_voltageC));
      json.set(HIAPath.c_str(), String(Home_currentA));
      json.set(HIBPath.c_str(), String(Home_currentB));
      json.set(HICPath.c_str(), String(Home_currentC));
      json.set(IFPath.c_str(), String(Acout_freq));
      json.set(GFPath.c_str(), String(Acin_freq));
      json.set(tPath.c_str(), String(temp));
      json.set(hPPath.c_str(), String(Home_realPower));
      json.set(hCPath.c_str(), String(Home_importunits));
      json.set(bVPath.c_str(), String("-"));
      json.set(bIPath.c_str(), String("-"));
      json.set(bPPath.c_str(), String("-"));

      //Serial.printf("Set json...\n", Firebase.RTDB.setJSON(&fbdo, parentPath.c_str(), &json) ?   counter= 0 : counter = 8);
          if (Firebase.RTDB.setJSON(&fbdo, parentPath.c_str(), &json)){
        Serial.println("PASSED");
        counter=0;
      Acin_importunits= 0;
      Acout_importunits= 0;
      Acin_exportunits = 0;
      Home_importunits = 0;
        Serial.println("PATH: " + fbdo.dataPath());
        Serial.println("TYPE: " + fbdo.dataType());
      }
      else {
        Serial.println("FAILED");
        Serial.println("REASON: " + fbdo.errorReason());
      }
    }
     
    }   
  }
    }
    delay(14000); 
}
