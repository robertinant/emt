#include <cc3200/libraries/WiFi/WiFi.h>
#include <cc3200/libraries/WiFi/WiFiClient.h>
#include <cc3200/libraries/WiFi/WiFiServer.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC3200.h>

//#include "WiFi.h"
#include <Wire.h>

// your network name also called SSID
char ssid[] = "TINK-NET";
// your network password
char password[] = "";

// Initialize the Wifi client library
WiFiClient client;

// server address:
char server[] = "energia.nu";

extern void lpds_init(void (*restore)(void));
extern void goto_sleep(uint32_t time);
uint32_t lastConnected = 0;
uint32_t totalSleepTime = 0;
uint32_t myMillis = 0;
#define SLEEP_INTERVAL_SEC 10
#define CONNECTION_INTERVAL_MS 20000 // Connect every 10 secs

void setupPeripherals();
void connectToWiFi();
void disconnectFromWiFi();
boolean httpRequest();

void lp3p0_back_up_soc_data();

int lpSleep()
{
//    digitalWrite(RED_LED, 0);
    lp3p0_back_up_soc_data();
    return (Power_NOTIFYDONE);
}

int lpAwake()
{
//    digitalWrite(RED_LED, 1);

    /* Trigger JTAG */
//    if (!(HWREG(0x4402DC30) & 0x2)) {
//        HWREG(0x4402DC30) = 0x1;
//    }

    return (Power_NOTIFYDONE);
}

static Power_NotifyObj powerSleepNotifyObj;
static Power_NotifyObj powerAwakeNotifyObj;

// ----------------------------
void lp_setup() {
//  digitalWrite(RED_LED, 1);
  Serial.begin(115200);
  setupPeripherals();
  WiFi.init();
  Serial.print("FW Version: ");
  Serial.println(WiFi.firmwareVersion());
  connectToWiFi();
  disconnectFromWiFi();
  lpds_init(setupPeripherals);
  Power_registerNotify(&powerSleepNotifyObj,
            PowerCC3200_ENTERING_LPDS,
            (Power_NotifyFxn)lpSleep, (uintptr_t) NULL);
  Power_registerNotify(&powerAwakeNotifyObj,
            PowerCC3200_AWAKE_LPDS,
            (Power_NotifyFxn)lpAwake, (uintptr_t) NULL);
}

// ----------------------------
void lp_loop() {    
  Serial.println("Going to sleep!!!!");
  // Small delay to allow the Serial buffer to be flushed before going to sleep
  delay(10);
  Serial.end();

  delay(SLEEP_INTERVAL_SEC * 1000);  

  Serial.begin(115200);
  Serial.println("Wakeup!!");

  myMillis = millis();
  
  if(myMillis - lastConnected > CONNECTION_INTERVAL_MS) {
    lastConnected = myMillis;
Serial.println(lastConnected);
    connectToWiFi();
//    httpRequest();
    disconnectFromWiFi();
  }
}

// ----------------------------
void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

// ----------------------------
boolean httpRequest() {
  boolean ret;
  
  // close any connection before send a new request.
  // This will free the socket on the WiFi shield
  client.stop();

  // if there's a successful connection:
  Serial.println("connecting...\n");

  if (client.connect(server, 80)) {
    // send the HTTP PUT request:
    client.println("GET /hello.txt HTTP/1.1");
    client.println("Host: energia.nu");
    client.println("User-Agent: Energia/1.1");
    client.println("Connection: close");
    client.println();

    uint32_t now = millis();
    while(!client.available()) {
      if(millis() - now > 10000) {
        Serial.println("Timeout waiting for data");
        break;
      }
    }
  
    while (client.available()) {
      char c = client.read();
      Serial.write(c);
    }

    client.stop();    
    ret = true;
  } else {
    // if you couldn't make a connection:
    Serial.println("connection failed");
    ret = false;
  }

  return ret;  
}

// ----------------------------
void setupPeripherals()
{
  Serial.begin(115200);
  Serial.println("Setting up peripherals\n");
}

// ----------------------------
void connectToWiFi() {
//  sl_WlanPolicySet(SL_POLICY_CONNECTION , SL_CONNECTION_POLICY(1,1,0,0,0), 0, 0);
    // attempt to connect to Wifi network:
//  digitalWrite(YELLOW_LED, 1);
  Serial.print("Attempting to connect to Network named: ");
  // print the network name (SSID);
  Serial.println(ssid);
  // Connect to WPA/WPA2 network.
  if (password[0] != 0) {
      WiFi.begin((char *)ssid, (char *)password);
  }
  else {
      WiFi.begin((char *)ssid);
  }
//  digitalWrite(YELLOW_LED, 0);
//  digitalWrite(GREEN_LED, 1);
  while ( WiFi.status() != WL_CONNECTED) {
    // print dots while we wait to connect
    Serial.print(".");
    delay(300);
  }

  Serial.println("\nYou're connected to the network");
  Serial.println("Waiting for an ip address");
  
  while (WiFi.localIP() == INADDR_NONE) {
    // print dots while we wait for an ip addresss
    Serial.print(".");
    delay(300);
  }

  Serial.println("\nIP Address obtained");
  // We are connected and have an IP address.
  // Print the WiFi status.
  printWifiStatus();
}

// ----------------------------
void disconnectFromWiFi() {
//  digitalWrite(GREEN_LED, 0);

  WiFi.end();

  /* the Serial port being active keeps us from going into lpds in this loop */
  while(WiFi.status() == WL_CONNECTED) {
    delay(10);
  }
  delay(10);
}

