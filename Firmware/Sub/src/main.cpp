#include <Arduino.h>
#include <AsyncUDP.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "../../RobobuoyDependency\RobobuoyVersion.h"
#include "../../RobobuoyDependency\RobobuoyMsg.h"
#include "../../RobobuoyDependency\RobobuoyDefaults.h"
#include "../../RobobuoyDependency\RobobuoyCalc.h"

AsyncUDP udp;

char packetBuffer[255];

unsigned int localPort = 9999;

const char *ssid = "buoy_100";

IPAddress ipServidor(192, 168, 4, 1); // Declaration of default IP for server
/*
 *  The ip address of the client has to be different to the server
 *  other wise it will conflict because the client tries to connect
 *  to it self.
 */
IPAddress ipCliente(192, 168, 4, 10); // Different IP than server
IPAddress Subnet(255, 255, 255, 0);

void setup()
{
  Serial.begin(115200);
  Serial.println("\r\nscan for AP");
  int n = WiFi.scanNetworks();
  Serial.println("scan done");
  for (int i = 0; i < n; ++i)
  {
    if (WiFi.SSID(i) == "BUOY_1")
    {
      WiFi.mode(WIFI_STA);
      Serial.print("AP NicE_WiFi foud, logging in...");
      WiFi.begin("BUOY_1","");
      while (WiFi.status() != WL_CONNECTED)
      {
        delay(50);
        Serial.print(".");
      }
      Serial.print(".\r\n");
      Serial.print("Loggend in to SSIS: ");
      Serial.println(ssid);
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
    }
  }

  if (udp.listen(1001))
  {
    Serial.print("UDP server na IP: ");
    Serial.print(WiFi.localIP());
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
                   // packet.printf("ESP32 received %u bytes of data", packet.length()); // odpoved odosielatelovi
                 });
  }
}

void loop()
{
  delay(700);
  udp.broadcast("Sub says: Hello world");
  udp.broadcastTo("Sub says: Hello world unicast", 1001);
}
