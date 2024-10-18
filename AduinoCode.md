## Power Meter with ESP32 and PZEM-004T

This project uses an ESP32 to communicate with a PZEM-004T energy meter using the Modbus protocol. It connects to an MQTT broker and publishes energy data such as voltage, current, power, and frequency.

### Libraries used:
- WiFi.h
- Time.h
- ModbusMaster.h
- PubSubClient.h
- ArduinoJson.h

### Wiring
- MAX485 module is used for Modbus communication with the PZEM-004T.
- DE pin connected to GPIO 21
- RE pin connected to GPIO 20

### Code

```cpp
#include <WiFi.h>
#include <Time.h>
#include <ModbusMaster.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>  

#define MAX485_DE 21  
#define MAX485_RE 20 

static uint8_t pzemSlaveAddr = 0x01;
HardwareSerial chat(1); 

ModbusMaster node;
float PZEMVoltage = 0;
float PZEMCurrent = 0;
float PZEMPower = 0;
float PZEMEnergy = 0;
float PZEMHz = 0;
float PZEMPf = 0;

const char* ssid = "Phet";     
const char* password = "ppkk3612";  
const char* mqtt_server = "broker.mqtt.cool";  

WiFiClient espClient;
PubSubClient client(espClient);

int timezone = 7 * 3600; // Timezone for Thailand
int dst = 0; // Daylight Savings Time

void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
}

void preTransmission() {
    digitalWrite(MAX485_RE, 1);
    digitalWrite(MAX485_DE, 1);
    delay(1);
}

void postTransmission() {
    delay(3);
    digitalWrite(MAX485_RE, 0);
    digitalWrite(MAX485_DE, 0);
}

void resetEnergy(uint8_t slaveAddr) {
    uint16_t u16CRC = 0xFFFF;
    static uint8_t resetCommand = 0x42;
    u16CRC = crc16_update(u16CRC, slaveAddr);
    u16CRC = crc16_update(u16CRC, resetCommand);
    preTransmission();

    chat.write(slaveAddr);
    chat.write(resetCommand);
    chat.write(lowByte(u16CRC));
    chat.write(highByte(u16CRC));
}

void reconnect() {
    // Loop until we're reconnected
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP32Client")) {
            Serial.println("connected");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void setup() {
    /* General */
    Serial.begin(115200);  
    Serial.setDebugOutput(true);
    chat.begin(9600, SERIAL_8N1, 16, 17); 

    setup_wifi();
    client.setServer(mqtt_server, 1883); 
    configTime(timezone, dst, "pool.ntp.org", "time.nist.gov");
    Serial.println("\nLoading time");
    while (!time(nullptr)) {
        Serial.print("*");
        delay(1000);
    }
    Serial.println("");

    node.begin(pzemSlaveAddr, chat);
    pinMode(MAX485_RE, OUTPUT);
    pinMode(MAX485_DE, OUTPUT);
    digitalWrite(MAX485_RE, 0);  
    digitalWrite(MAX485_DE, 0);  

    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);
    delay(1000);
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    configTime(timezone, dst, "pool.ntp.org", "time.nist.gov"); 
    time_t now = time(nullptr);
    struct tm* p_tm = localtime(&now);

    Serial.print(p_tm->tm_hour);
    Serial.print(":");
    Serial.print(p_tm->tm_min);
    Serial.print(":");
    Serial.print(p_tm->tm_sec);
    Serial.println("");
    delay(1000);

    if ((p_tm->tm_sec) <= 1) {
        Serial.print("I LOVE SOLARDUINO\n");
        delay(1000);
    }

    // Modbus communication
    uint8_t result = node.readInputRegisters(0x0000, 9);
    if (result == node.ku8MBSuccess) {
        uint32_t tempdouble = 0x00000000;

        PZEMVoltage = node.getResponseBuffer(0x0000) / 10.0;
        tempdouble = (node.getResponseBuffer(0x0002) << 16) + node.getResponseBuffer(0x0001);
        PZEMCurrent = tempdouble / 1000.0;

        tempdouble = (node.getResponseBuffer(0x0004) << 16) + node.getResponseBuffer(0x0003);
        PZEMPower = tempdouble / 10.0;

        tempdouble = (node.getResponseBuffer(0x0006) << 16) + node.getResponseBuffer(0x0005);
        PZEMEnergy = tempdouble;

        PZEMHz = node.getResponseBuffer(0x0007) / 10.0;
        PZEMPf = node.getResponseBuffer(0x0008) / 100.00;
    }

    // Create a JSON object to hold the data
    StaticJsonDocument<200> jsonDoc;
    jsonDoc["voltage"] = PZEMVoltage;
    jsonDoc["current"] = PZEMCurrent;
    jsonDoc["power"] = PZEMPower;
    jsonDoc["energy"] = PZEMEnergy;
    jsonDoc["frequency"] = PZEMHz;
    jsonDoc["powerFactor"] = PZEMPf;

    // Convert the JSON object to a string
    char buffer[256];
    size_t n = serializeJson(jsonDoc, buffer);

    // Publish the JSON string under the topic "power/meter"
    client.publish("power/meter", buffer);

    Serial.print("POWER IS ");
    Serial.print(PZEMPower);
    Serial.println(" WATTS");

    Serial.print("Voltage IS ");
    Serial.print(PZEMVoltage);
    Serial.println(" Volts");

    Serial.print("CURRENT IS ");
    Serial.print(PZEMCurrent);
    Serial.println(" Amps");

    Serial.print("ENERGY IS ");
    Serial.print(PZEMEnergy);
    Serial.println(" Wh");

    Serial.print("FREQUENCY IS ");
    Serial.print(PZEMHz);
    Serial.println(" Hz");

    Serial.print("POWER FACTOR IS ");
    Serial.print(PZEMPf);
    Serial.println(" Pf");

    delay(5000);  // Publish data every 5 seconds
}
