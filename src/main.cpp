/* Author:        Hans Christian Diekmann
 * Last update:   04.05.2022
 * Initial start: 03.05.2022
 *
 * Function:
 * 1. HTCC-AB01 board, as a battery powered GPS node can read its battery voltage and transmits the reading via LoRa peer-to-peer.
 * 2. Acquire the current GPS coordinates, altitude and velocity and tansmitt the values via LoRa (chirp spread spectrum).
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
#define BUFFER_SIZE                                 60        // set payload size here
#define GPS_BAUD                                    9600

typedef enum
{
    LOWPOWER,
    ReadVoltage,
    GetLocation,
    TX
}States_t;

States_t state = ReadVoltage;
TinyGPSPlus gps;
static softSerial gpsSerial(GPIO3 /*TX pin*/, GPIO2 /*RX pin*/);
static RadioEvents_t RadioEvents;

uint32_t BoardGetBatteryVoltage( void );
void OnTxDone( void );
void OnTxTimeout( void );
void Main( );

char txPacket[BUFFER_SIZE];
bool sleepMode = false;
int16_t rssi;
uint16_t voltage;
uint8_t noOfSatellites;
float latitude,longitude,alti,velocity;

void setup() {
    Serial.begin(115200);
    Serial.println("Starting NODE");
    gpsSerial.begin(GPS_BAUD);
    pinMode(GPIO6, OUTPUT);
    digitalWrite(GPIO6, HIGH);
    voltage = 0;
    rssi = 0;

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

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
        turnOnRGB(COLOR_RXWINDOW2, 0); //yellow LED when starting TX
        char str_lat[8],str_lon[8],str_alt[8],str_vel[8];
        /* mininum width 5, 2 is dec precision, float value is copied into str_var*/
        dtostrf(latitude, 5, 2, str_lat);
        dtostrf(longitude, 5, 2, str_lon);
        dtostrf(alti, 5, 2, str_alt);
        dtostrf(velocity, 5, 2, str_vel);        
        sprintf(txPacket,"Bat:%d,Lat:%s,Lon:%s,Alt:%s,Vel:%s", voltage,str_lat,str_lon,str_alt,str_vel);
        Serial.printf("\r\nSending packet: \"%s\" , Length: %d\r\n",txPacket, strlen(txPacket));
        Radio.Send( (uint8_t *)txPacket, strlen(txPacket) );
        state = LOWPOWER;
        break;
      }
      case LOWPOWER:
      {
        lowPowerHandler();
        delay(100);
        turnOffRGB();
        delay(5000);  // LowPower time
        state = ReadVoltage;
        break;
      }
      case ReadVoltage:
      {
        voltage = BoardGetBatteryVoltage();
        Serial.printf("\nBattery: %d\n", voltage);
        state = GetLocation;
        break;
      }
      case GetLocation:
      {
        while (gpsSerial.available())     // check serial for new gps data
        {
          gps.encode(gpsSerial.read()); // encode gps data
          if (gps.location.isUpdated())   
          {
            noOfSatellites = gps.satellites.value();
            latitude = gps.location.lat();
            longitude = gps.location.lng();
            alti = gps.altitude.meters();
            velocity = gps.speed.mps();

            Serial.print("SATS: ");
            Serial.println(noOfSatellites);
            Serial.print("LAT: ");
            Serial.println(latitude);
            Serial.print("LONG: ");
            Serial.println(longitude);
            Serial.print("ALT: ");
            Serial.println(alti);
            Serial.print("SPEED: ");
            Serial.println(velocity);
            Serial.println("---------------------------");
          }
        }
        state = TX;
        break;
      }
      default:
        break;
    }
}

/*  get the BatteryVoltage in mV. */
uint32_t BoardGetBatteryVoltage(void)
{
    float temp = 0;
    uint16_t volt;
    uint8_t pin;
    pin = ADC;
#if defined(CubeCell_Board) || defined(CubeCell_Capsule) || defined(CubeCell_BoardPlus) || defined(CubeCell_BoardPRO) || defined(CubeCell_GPS) || defined(CubeCell_HalfAA)
    //Board, BoardPlus, etc. variants have external 10K VDD pullup resistor connected to GPIO7 (USER_KEY / VBAT_ADC_CTL) pin
    pinMode(VBAT_ADC_CTL, OUTPUT);
    digitalWrite(VBAT_ADC_CTL, LOW);
#endif
    for (int i = 0; i < 50; i++) // read 50 times and get average
        temp += analogReadmV(pin);
    volt = temp / 50;
#if defined(CubeCell_Board) || defined(CubeCell_Capsule) || defined(CubeCell_BoardPlus) || defined(CubeCell_BoardPRO) || defined(CubeCell_GPS) || defined(CubeCell_HalfAA)
    pinMode(VBAT_ADC_CTL, INPUT);
#endif
    volt = volt * 2;
    return volt;
}

void OnTxDone( void )
{
  Serial.print("Payload sent");
  turnOnRGB(COLOR_RECEIVED,0); // green LED when sent
}

void OnTxTimeout( void )
{
    turnOnRGB(COLOR_SEND,0); // red LED when timeout occurred
    Radio.Sleep( );
    Serial.print("TX Timeout");
    state = ReadVoltage;
}