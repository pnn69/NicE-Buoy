/*|-----------------------------------------------------------|*/
/*|SKETCH FOR ESP32 UDP DATAGRAMS CONTROL USING PACKET SENDER |*/
/*|Author: MARTIN CHLEBOVEC                                   |*/
/*|Web: http://arduino.clanweb.eu/udp-control-esp32.php       |*/
/*|Board: ESP32 DevkitC v4 / ESP32 Devkit v1                  |*/
/*|Buy me coffee: paypal.me/chlebovec                         |*/
/*|Send ON or OFF string from Packet Sender for change of pin |*/
/*|-----------------------------------------------------------|*/

#include "WiFi.h"
#include "AsyncUDP.h"
const char *ssid = "BUOY_1";
const char *pass = "";
const int led = 2; // D pin (Build-in LED for Devkit V1)
AsyncUDP udp;
bool ap = false;
void setup()
{
  Serial.begin(115200);
  pinMode(led, OUTPUT);
  Serial.begin(115200);
  // Serial.println("\r\nscan for AP");
  // int n = WiFi.scanNetworks();
  // Serial.println("scan done");
  // for (int i = 0; i < n; ++i)
  // {
  //   if (WiFi.SSID(i) == "NicE_WiFi_tralal")
  //   {
  //     ap = false;
  //     WiFi.mode(WIFI_STA);
  //     Serial.print("AP NicE_WiFi foud, logging in...");
  //     WiFi.begin("NicE_WiFi", "!Ni1001100110");
  //     while (WiFi.status() != WL_CONNECTED)
  //     {
  //       delay(50);
  //       Serial.print(".");
  //     }
  //     Serial.print(".\r\n");
  //     Serial.print("Loggend in to SSIS: ");
  //     Serial.println(ssid);
  //     Serial.print("IP address: ");
  //     Serial.println(WiFi.localIP());
  //   }
  // }
  if (ap == false)
  {
    WiFi.mode(WIFI_AP);
    // WiFi.softAP("BUOY_1", "");
    Serial.print("Accespoint setu as: BUOY_1\r\n");
    IPAddress AP_LOCAL_IP(192, 168, 1, 1);
    IPAddress AP_GATEWAY_IP(192, 168, 1, 254);
    IPAddress AP_NETWORK_MASK(255, 255, 255, 0);
    WiFi.softAPConfig(AP_LOCAL_IP, AP_GATEWAY_IP, AP_NETWORK_MASK);
    WiFi.softAPsetHostname("BUOY_1");

    // To initiate the Soft AP, pause the program if the initialization process encounters an error.
    if (!WiFi.softAP(ssid, ""))
    {
      Serial.println("Soft AP creation failed.");
      while (1)
        ;
    }
    Serial.print("AP Name SSID: ");
    Serial.println(WiFi.softAPSSID());
    Serial.print("AP IP Address: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("AP Hostname: ");
    Serial.println(WiFi.softAPgetHostname());
    Serial.print("AP Mac Address: ");
    Serial.println(WiFi.softAPmacAddress());
    Serial.print("AP Subnet: ");
    Serial.println(WiFi.softAPSubnetCIDR());
  }

  if (udp.listen(1001))
  {
    Serial.print("UDP server na IP: ");
    Serial.print(WiFi.softAPIP());
    Serial.println(" port: 1001");
    udp.onPacket([](AsyncUDPPacket packet)
                 {
                   Serial.print("Type of UDP datagram: ");
                   Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast"
                                                                                          : "Unicast");
                   Serial.print(", Sender: ");
                   Serial.print(packet.remoteIP());
                   Serial.print(":");
                   Serial.print(packet.remotePort());
                   Serial.print(", Receiver: ");
                   Serial.print(packet.localIP());
                   Serial.print(":");
                   Serial.print(packet.localPort());
                   Serial.print(", Message length: ");
                   Serial.print(packet.length()); // dlzka spravy
                   Serial.print(", Payload: ");
                   Serial.write(packet.data(), packet.length());
                   Serial.println();
                   String myString = (const char *)packet.data();
                   //  if (myString.startsWith("ON"))
                   //  {
                   //    Serial.println("Action: Turning ON relay");
                   //    digitalWrite(led, HIGH);
                   //  }
                   //  else if (myString.startsWith("OFF"))
                   //  {
                   //    Serial.println("Action: Turning OFF relay");
                   //    digitalWrite(led, LOW);
                   //  }
                   // packet.printf("ESP32 received %u bytes of data", packet.length()); // odpoved odosielatelovi
                 });
  }
}

void loop()
{
  delay(100);
  //udp.broadcast("Top says: Hello world");
  //Serial.print("Num of Connected Clients : ");
  //Serial.println(WiFi.softAPgetStationNum());
}
