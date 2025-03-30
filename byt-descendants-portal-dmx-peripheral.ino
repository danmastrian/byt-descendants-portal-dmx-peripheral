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
const int I2C_PACKET_METADATA_BYTES = sizeof(uint16_t) + sizeof(uint8_t);

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

CRC32 crc;

void setup()
{
  Serial.begin(115200);
  Serial.println("============== HELLO DMX ==============");

  Wire.begin();
  //Wire.setClock(400000);
  Wire.setTimeout(200);

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
  dmx_driver_install(dmxPort, &config, personalities, personality_count);

  /* Now set the DMX hardware pins to the pins that we want to use and setup
    will be complete! */
  dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);
}

void loop()
{
  /* We need a place to store information about the DMX packets we receive. We
    will use a dmx_packet_t to store that packet information.  */
  dmx_packet_t packet;

  /* And now we wait! The DMX standard defines the amount of time until DMX
    officially times out. That amount of time is converted into ESP32 clock
    ticks using the constant `DMX_TIMEOUT_TICK`. If it takes longer than that
    amount of time to receive data, this if statement will evaluate to false. */
  if (dmx_receive(dmxPort, &packet, DMX_TIMEOUT_TICK))
  {
    /* If this code gets called, it means we've received DMX data! */

    /* Get the current time since boot in milliseconds so that we can find out
      how long it has been since we last updated data and printed to the Serial
      Monitor. */
    unsigned long now = millis();

    /* We should check to make sure that there weren't any DMX errors. */
    if (!packet.err)
    {
      /* If this is the first DMX data we've received, lets log it! */
      if (!dmxIsConnected)
      {
        Serial.println("DMX is connected!");
        dmxIsConnected = true;
      }

      // Read the DMX data into the buffer
      dmx_read(dmxPort, data, packet.size);

      if (now - lastUpdate > DMX_FORWARD_PERIOD_MSEC)
      {
          if (data[0] == 0) // Expecting DMX NULL start code
          {
            uint16_t chCount = DMX_CH_COUNT_PER_PACKET;

            for (uint16_t chStartIdx = 1; chStartIdx <= DMX_UNIVERSE_SIZE; chStartIdx += chCount)
            {
              //delayMicroseconds(2000); // Small delay to avoid overwhelming the I2C bus
              chCount = min(DMX_CH_COUNT_PER_PACKET, DMX_UNIVERSE_SIZE - chStartIdx + 1);

              uint8_t sendBuffer[chCount + I2C_PACKET_METADATA_BYTES + sizeof(crc_size_t)];

              sendBuffer[0] = (chStartIdx >> 8) & 0xFF; // start channel high byte
              sendBuffer[1] = chStartIdx & 0xFF; // start channel low byte
              sendBuffer[2] = chCount & 0xFF;

              memcpy(
                sendBuffer + I2C_PACKET_METADATA_BYTES,
                data + chStartIdx,
                chCount);

              crc.reset();
              crc.add(sendBuffer, sizeof(sendBuffer) - sizeof(crc_size_t)); // Exclude CRC bytes
              crc_size_t crcValue = crc.calc();

              memcpy(
                sendBuffer + I2C_PACKET_METADATA_BYTES + chCount,
                (void*)&crcValue,
                sizeof(crc_size_t));
              
              Wire.beginTransmission(I2C_DEV_ADDR);

              //Wire.printf("DMX @ %6lu |%03d|%03d|%03d|%03d|", i++, data[1], data[2], data[3], data[4]);
              size_t bytesWritten = Wire.write(sendBuffer, sizeof(sendBuffer));
              
              uint8_t error = Wire.endTransmission(true);
              if (error == 0)
              {
                /*
                Serial.printf(
                  "endTransmission: startIdx %u, chCount %u, code %u, bytes written %u, CRC %08X\n",
                  chStartIdx,
                  chCount,
                  error,
                  bytesWritten,
                  crcValue);
                  */
              }
              else
              {
                Serial.printf(
                  "endTransmission: startIdx %u, chCount %u, code %u, bytes written %u, CRC %08X\n",
                  chStartIdx,
                  chCount,
                  error,
                  bytesWritten,
                  crcValue);
              }
            }

            lastUpdate = now;
          }
      }
    }
    else
    {
      /* Oops! A DMX error occurred! Don't worry, this can happen when you first
        connect or disconnect your DMX devices. If you are consistently getting
        DMX errors, then something may have gone wrong with your code or
        something is seriously wrong with your DMX transmitter. */
      Serial.println("A DMX error occurred.");
    }
  }
  else if (dmxIsConnected)
  {
    /* If DMX times out after having been connected, it likely means that the
      DMX cable was unplugged. When that happens in this example sketch, we'll
      uninstall the DMX driver. */
    Serial.println("DMX was disconnected.");
    dmxIsConnected = false;
    delay(250);
  }
}
