/*****************************************************************************************
 *     Serial Bluetooth communication between 2 ESP32 boards - OK1TK www.ok1tk.com        *
 *                   The SERVER (Master) part code v1.32 - 04 May 2023                    *
 ******************************************************************************************/
#include "BluetoothSerial.h" // BT: Include the Serial bluetooth library
#include <stdio.h>
#include <stdlib.h>
unsigned long previousMillisReconnect; // BT: Variable used for comparing millis counter for the reconnection timer
bool SlaveConnected;                   // BT: Variable used to store the current connection state (true=connected/false=disconnected)

String myName = "ESP32-BT-Master";                         // BT: Variable used to store the SERVER(Master) bluetooth device name; just for prinitng
String slaveName = "ALFEN-AP-WXLRIN";                      // BT: Variable used to store the CLIENT(Slave) bluetooth device name; just for prinitng; just for printing in this case
String MACadd = "E4:60:17:0F:72:D2";                       // BT: Variable used to store the CLIENT(Slave) bluetooth device Mac address; just for prinitng; just for printing in this case
uint8_t address[6] = {0xE4, 0x60, 0x17, 0x0F, 0x72, 0xD2}; // BT: Variable used to store the CLIENT(Slave) MAC address used for the connection; Use your own andress in the same format
int ctel = 0;
String str = "";

BluetoothSerial SerialBT; // BT: Set the Object SerialBT

// BT: Bt_Status callback function
void Bt_Status(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
  if (event == ESP_SPP_OPEN_EVT)
  {                                     // BT: Checks if the SPP connection is open, the event comes// event == Client connected
    Serial.println("Client Connected"); // BT: Write to the serial monitor
    SlaveConnected = true;              // BT: Set the variable true = CLIENT is connected to the SERVER
  }
  else if (event == ESP_SPP_CLOSE_EVT)
  {                                        // BT: event == Client disconnected
    Serial.println("Client Disconnected"); // BT: Write to the serial monitor
    SlaveConnected = false;                // BT: Set the variable false = CLIENT connection lost
  }
}

void SlaveConnect()
{                               // BT: This function connects/reconnects to the CLIENT(Slave)
  SerialBT.end();               // BT: Close the bluetooth device
  SerialBT.begin(myName, true); // BT: Starts the bluetooth device with the name stored in the myName variable as SERVER(Master)
  Serial.printf("The device \"%s\" started in master mode, make sure slave BT device is on!\n", myName.c_str());
  Serial.printf("Connecting to slave BT device named \"%s\" and MAC address \"%s\" is started.\n", slaveName.c_str(), MACadd.c_str()); // BT: Write to the serial monitor
  SerialBT.connect(address);                                                                                                           // BT: Establishing the connection with the CLIENT(Slave) with the Mac address stored in the address variable
}

void setup()
{
  SlaveConnected = false;                // BT: Set the variable false = CLIENT is not connected
  Serial.begin(115200);                  // Sets the data rate in bits per second (baud) for serial data transmission
  SerialBT.register_callback(Bt_Status); //
  SerialBT.begin(myName, true);          // Start Bluetooth as master device
  Serial.printf("The device \"%s\" started in master mode, make sure slave BT device is on!\n", myName.c_str());
  SlaveConnect(); // BT: Calls the bluetooth connection function to cnnect to the CLIENT(Slave)
}

void loop()
{
  if (!SlaveConnected)
  { // BT: Condition to evalute if the connection is established/lost
    if (millis() - previousMillisReconnect >= 10000)
    {                                     // BT: Check that 10000ms is passed
      previousMillisReconnect = millis(); // BT: Set previousMillisReconnect to current millis
      SlaveConnect();                     // BT: Calls the bluetooth connection function to cnnect to the CLIENT(Slave)
    }
  }
  // BT: Data send/receive via bluetooth
  if (Serial.available())
  {
    if (SerialBT.connected())
    {                                // BT: Checks if there are data from the serial monitor available
      SerialBT.write(Serial.read()); // BT: Sends the data via bluetooth
    }
  }
  if (SerialBT.connected())
  {
    double minn;
    double maxx;
    double vbatt;
    double bb;
    double sb;
    double dir;
    //"'ID'DEST'msgID'GSA'MSG'"
    switch (ctel)
    {
    case 1:
      minn = 21.0;
      maxx = 23.5;
      vbatt = minn + ((double)rand() / RAND_MAX) * (maxx - minn);
      SerialBT.printf("<1><7><24><1><%1.2lf,89>", vbatt );
      break;
    case 2:
      minn = 150.0;
      maxx = 160;
      dir = minn + ((double)rand() / RAND_MAX) * (maxx - minn);
      SerialBT.printf("<1><7><23><1><52.32038000,4.96563000,10,1,0,0,%d>", (int)dir);
      break;
    case 3:
      minn = 80.0;
      maxx = -40;
      bb = minn + ((double)rand() / RAND_MAX) * (maxx - minn);
      sb = minn + ((double)rand() / RAND_MAX) * (maxx - minn);
      minn = 150.0;
      maxx = 160;
      dir = minn + ((double)rand() / RAND_MAX) * (maxx - minn);
      SerialBT.printf("<1><7><18><1><97.00,60.02,76,%d,%d,%d>", (int)bb, (int)sb, (int)dir);
      break;
    case 4:
      minn = 80.0;
      maxx = -40;
      bb = minn + ((double)rand() / RAND_MAX) * (maxx - minn);
      sb = minn + ((double)rand() / RAND_MAX) * (maxx - minn);
      minn = 150.0;
      maxx = 160;
      dir = minn + ((double)rand() / RAND_MAX) * (maxx - minn);
      SerialBT.printf("<1><7><20><1><97,60.02,76,%dbb,%d,%d>", (int)bb, (int)sb, (int)dir);
      break;
    case 5:
      minn = 15.0;
      maxx = 1;
      vbatt = minn + ((double)rand() / RAND_MAX) * (maxx - minn);
      minn = 150.0;
      maxx = 160;
      dir = minn + ((double)rand() / RAND_MAX) * (maxx - minn);
      SerialBT.printf("<1><7><32><1><%10lf,%1.0lf>", dir,vbatt);
      break;
    default:
      break;
    }
    if (ctel >= 6)
    {
      ctel = 0;
    }
    else
    {
      ctel++;
    }
    delay(1000);
  }

  if (SlaveConnected == true && SerialBT.connected() == false)
  {
    SlaveConnected = false;
    Serial.println("Disconnected\r\n");
  }
}