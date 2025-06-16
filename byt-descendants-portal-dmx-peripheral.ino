/*

  DMX Read

  This sketch allows you to read DMX from a DMX controller using a standard DMX
  shield, such SparkFun ESP32 Thing Plus DMX to LED Shield. This sketch was
  made for the Arduino framework!

  Created 9 September 2021
  By Mitch Weisbrod

  https://github.com/someweisguy/esp_dmx

*/
#include <Arduino.h>
#include <esp_dmx.h>
#include "Wire.h"
#include "CRC32.h"
#include <SPI.h>
#include "SerialTransfer.h"

/* First, lets define the hardware pins that we are using with our ESP32. We
  need to define which pin is transmitting data and which pin is receiving data.
  DMX circuits also often need to be told when we are transmitting and when we
  are receiving data. We can do this by defining an enable pin. */
const int transmitPin = 17;
const int receivePin = 16;
const int enablePin = 21;
/* Make sure to double-check that these pins are compatible with your ESP32!
  Some ESP32s, such as the ESP32-WROVER series, do not allow you to read or
  write data on pins 16 or 17, so it's always good to read the manuals. */

/* Next, lets decide which DMX port to use. The ESP32 has either 2 or 3 ports.
  Port 0 is typically used to transmit serial data back to your Serial Monitor,
  so we shouldn't use that port. Lets use port 1! */
const dmx_port_t dmxPort = 1;

const uint8_t I2C_DEV_ADDR = 0x55;

const unsigned long DMX_FORWARD_PERIOD_MSEC = 5ul;

const int DMX_CH_COUNT_PER_PACKET = 16;

// 2 bytes for start channel, 1 byte for channel count
const int I2C_PACKET_HEADER_BYTES = sizeof(uint16_t) + sizeof(uint8_t);

const int DMX_UNIVERSE_SIZE = 512;

/* The last two variables will allow us to know if DMX has been connected and
  also to update our packet and print to the Serial Monitor at regular
  intervals. */
bool dmxIsConnected = false;
unsigned long lastUpdate = millis();

/* Now we want somewhere to store our DMX data. Since a single packet of DMX
  data can be up to 513 bytes long, we want our array to be at least that long.
  This library knows that the max DMX packet size is 513, so we can fill in the
  array size with `DMX_PACKET_SIZE`. */
byte data[DMX_PACKET_SIZE];

CRC32 myCrc;

//HardwareSerial b2bSerial(2);
//SerialTransfer b2bSerialTransfer;

SPIClass mySPI(HSPI); // or HSPI
#define CS_PIN A5

void setup()
{
  Serial.begin(115200);
  Serial.println("============== HELLO DMX ==============");

  //b2bSerial.begin(115200, SERIAL_8E2, 27, 26);
  //b2bSerial.println("DMX Forwarder started");
  //b2bSerialTransfer.begin(b2bSerial);

  pinMode(CS_PIN, OUTPUT); // Set the CS pin for SPI
  mySPI.begin(SCK, MISO, MOSI, CS_PIN);

  //Wire.begin();
  //Wire.setClock(400000);
  //Wire.setTimeout(200);

  /* Now we will install the DMX driver! We'll tell it which DMX port to use,
    what device configuration to use, and what DMX personalities it should have.
    If you aren't sure which configuration to use, you can use the macros
    `DMX_CONFIG_DEFAULT` to set the configuration to its default settings.
    This device is being setup as a DMX responder so it is likely that it should
    respond to DMX commands. It will need at least one DMX personality. Since
    this is an example, we will use a default personality which only uses 1 DMX
    slot in its footprint. */
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_personality_t personalities[] = {
    {1, "Default Personality"}
  };
  int personality_count = 1;
  //dmx_driver_install(dmxPort, &config, personalities, personality_count);

  /* Now set the DMX hardware pins to the pins that we want to use and setup
    will be complete! */
  //dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);
}

typedef struct
{
  uint16_t StartMarker; // 0xCAFE
  uint8_t MessageType;
  uint16_t PayloadLength;
  crc_size_t CrcValue;
} MessageHeader;

typedef struct DmxPacketFragment
{
  uint16_t startChannel; // Start channel (1-based index)
  uint8_t channelCount; // Number of channels in this fragment
  uint8_t data[DMX_CH_COUNT_PER_PACKET]; // DMX data for the channels
} dmx_packet_fragment_t;

uint8_t seq = 0;

void loop()
{
  /* We need a place to store information about the DMX packets we receive. We
    will use a dmx_packet_t to store that packet information.  */
  //dmx_packet_t packet;

  /* And now we wait! The DMX standard defines the amount of time until DMX
    officially times out. That amount of time is converted into ESP32 clock
    ticks using the constant `DMX_TIMEOUT_TICK`. If it takes longer than that
    amount of time to receive data, this if statement will evaluate to false. */
    delay(5);

    //Serial.println("SPI send");
    digitalWrite(CS_PIN, LOW); // Set CS low to select the SPI device
    mySPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    mySPI.transfer(seq++);
    mySPI.transfer(0xDE);
    mySPI.transfer(0xAD);
    mySPI.transfer(0xBE);
    mySPI.transfer(0xEF);
    mySPI.transfer(0xCA);
    mySPI.transfer(0xFE);
    mySPI.transfer(0xF0);
    mySPI.transfer(0x0D);
    for (uint8_t i = 0; i < 128; i++)
    {
      mySPI.transfer(i);
    }
    mySPI.endTransaction();
    digitalWrite(CS_PIN, HIGH); // Set CS high to deselect the SPI device

#ifdef DMX

  //if (dmx_receive(dmxPort, &packet, DMX_TIMEOUT_TICK))
  //{
    /* If this code gets called, it means we've received DMX data! */

    /* Get the current time since boot in milliseconds so that we can find out
      how long it has been since we last updated data and printed to the Serial
      Monitor. */
    unsigned long now = millis();

    /* We should check to make sure that there weren't any DMX errors. */
    //if (!packet.err)
    //{
      /* If this is the first DMX data we've received, lets log it! */
      if (!dmxIsConnected)
      {
        Serial.println("DMX is connected!");
        dmxIsConnected = true;
      }

      // Set up fake DMX data for testing
      for (int i = 0; i < DMX_PACKET_SIZE; i++)
      {
        data[i] = i;
      }

      if (now - lastUpdate > DMX_FORWARD_PERIOD_MSEC)
      {
          if (data[0] == 0) // Expecting DMX NULL start code
          {
            uint16_t chCount = DMX_CH_COUNT_PER_PACKET;

            for (uint16_t chStartIdx = 1; chStartIdx <= DMX_UNIVERSE_SIZE; chStartIdx += chCount)
            {
              delayMicroseconds(10000); // Small delay to avoid overwhelming the I2C bus

              chCount = min(DMX_CH_COUNT_PER_PACKET, DMX_UNIVERSE_SIZE - chStartIdx + 1);

              dmx_packet_fragment_t fragment;
              fragment.startChannel = chStartIdx; // 1-based index
              fragment.channelCount = chCount;
              memcpy(fragment.data, data + chStartIdx - 1, chCount); // Copy DMX data

              MessageHeader header;
              header.StartMarker = 0xCAFE; // Custom start marker
              header.MessageType = 0x01; // DMX packet type
              header.PayloadLength = sizeof(fragment);
              header.CrcValue = 0; // Placeholder for CRC

              myCrc.reset();
              myCrc.add((uint8_t*)&header, sizeof(header));
              myCrc.add((uint8_t*)&fragment, sizeof(fragment));
              header.CrcValue = myCrc.calc(); // Calculate CRC value

              uint16_t sendSize = 0;
              //sendSize = b2bSerialTransfer.txObj(header, sendSize);
              //sendSize = b2bSerialTransfer.txObj(fragment, sendSize);
              //b2bSerialTransfer.sendData(sendSize);

              // size_t bytesWritten = b2bSerial.write((uint8_t*)&header, sizeof(header));
              // bytesWritten += b2bSerial.write((uint8_t*)&fragment, sizeof(fragment));

              // if (bytesWritten != sizeof(header) + sizeof(fragment))
              // {
              //   Serial.println("Error writing to serial port");
              // }
              // else
              // {
              //   Serial.printf("DMX Fragment: Start Channel: %d, Count: %d\n", fragment.startChannel, fragment.channelCount);
              // }
            }

            lastUpdate = now;
          }
      }

      #endif
}
