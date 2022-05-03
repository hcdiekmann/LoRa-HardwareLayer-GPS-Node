/* Author:        Hans Christian Diekmann
 * Last update:   03.05.2022
 *
 * Function:
 * 1. HTCC-AB01 board, as a battery powered GPS node can read its battery voltage and transmits the reading via LoRa (chirp spread spectrum).
 * 2. Acquire the current GPS coordinates, altitude and speed and tansmitt the values via LoRa peer-to-peer.
 * 
 * Description:
 * 1. Only hardware layer communication (P2P), no LoRaWAN protocol;
 * 2. Communication with any other LoRa device possible, eg. ESP32 LoRa, STM32 LoRa, etc. 
 *
 *
 * */
#include "Arduino.h"
#include "LoRaWan_APP.h"
#include <TinyGPSPlus.h>
#include <softSerial.h>

#ifndef RGB
#define RGB 0
#endif

#define RF_FREQUENCY                                868000000 // 868KHz for EU
#define TX_OUTPUT_POWER                             14        // dBm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       8         // [SF7..SF12]
#define LORA_CODINGRATE                             4         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 80 // Define the payload size here
#define GPS_BAUD                                    9600

TinyGPSPlus gps;
softSerial gpsSerial(GPIO3 /*TX pin*/, GPIO2 /*RX pin*/);
static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );

typedef enum
{
    LOWPOWER,
    ReadVoltage,
    GetLocation,
    TX
}States_t;
States_t state;

char txPacket[BUFFER_SIZE];
bool sleepMode = false;
int16_t rssi,rxSize;
uint16_t voltage,latitude,longitude,altitude,velocity;

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(GPS_BAUD); // gps UART (GPIO 2 & 3)
    voltage = 0;
    rssi=0;

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    state=ReadVoltage;
}



void loop()
{
  Main();
  Radio.IrqProcess();
}

void Main(){
    switch(state)
    {
      case TX:
      {
        sprintf(txPacket,"Bat:%d,Lat:%d,Lon:%d,Alt:%d,Vel:%d", voltage,latitude,longitude,altitude,velocity);
        turnOnRGB(COLOR_SEND,0);
        Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",txPacket, strlen(txPacket));
        Radio.Send( (uint8_t *)txPacket, strlen(txPacket) );
        state = LOWPOWER;
        break;
      }
      case LOWPOWER:
      {
        lowPowerHandler();
        delay(100);
        turnOffRGB();
        delay(2000);  //LowPower time
        state = ReadVoltage; 
        break;
      }
      case ReadVoltage:
      {
        pinMode(VBAT_ADC_CTL,OUTPUT);   //Board, BoardPlus, etc. variants have external 10K VDD pullup resistor connected to GPIO7 (USER_KEY / VBAT_ADC_CTL) pin
        digitalWrite(VBAT_ADC_CTL,LOW);
        voltage = analogRead(ADC)*2;
        pinMode(VBAT_ADC_CTL, INPUT);
        state = GetLocation;
        break;
      }
      case GetLocation:
      {
        while (gpsSerial.available())     // check for gps data
        {
          if (gps.encode(gpsSerial.read()))   // encode gps data
          {
            Serial.print("SATS: ");
            Serial.println(gps.satellites.value());
            Serial.print("LAT: ");
            latitude = gps.location.lat();
            Serial.println(gps.location.lat(), 6);
            Serial.print("LONG: ");
            Serial.println(gps.location.lng(), 6);
            Serial.print("ALT: ");
            Serial.println(gps.altitude.meters());
            Serial.print("SPEED: ");
            Serial.println(gps.speed.mps());
      
            Serial.print("Date: ");
            Serial.print(gps.date.day()); Serial.print("/");
            Serial.print(gps.date.month()); Serial.print("/");
            Serial.println(gps.date.year());
      
            Serial.print("Hour: ");
            Serial.print(gps.time.hour()); Serial.print(":");
            Serial.print(gps.time.minute()); Serial.print(":");
            Serial.println(gps.time.second());
            Serial.println("---------------------------");
            delay(5000);
          }
        }
        state = TX;
        break;
      }
      default:
        break;
    }
}

void OnTxDone( void )
{
  Serial.print("TX done!");
  turnOnRGB(0,0);
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    Serial.print("TX Timeout......");
    state = ReadVoltage;
}