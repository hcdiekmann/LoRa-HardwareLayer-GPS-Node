/* Author:        Hans Christian Diekmann
 * Last update:   14.07.2022
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

#include "LoRaWan_APP.h"
#include <TinyGPSPlus.h>
#include <softSerial.h>


// #define LORA_NSS    22 //or 39/P4.3 SPI slave select
// #define LORA_RST    27 //or P5.7 SPI reset
// #define LORA_BUSY   26 //or P4.7

#ifndef NODE_ID
#define NODE_ID     2 // to do: write the node id into EEPROM
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
    StandbyRecharge,
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
float latitude,longitude,alti,velocity;

void setup() {
    Serial.begin(115200);
    Serial.println("Starting NODE: " + NODE_ID);
    gpsSerial.begin(GPS_BAUD);
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
        char str_alt[8],str_vel[8];
        String str_lat = String (latitude, 6);
        String str_lon = String (longitude, 6);
        /* mininum width 5, 2 is dec precision, float value is copied into str_var*/
        dtostrf(alti, 5, 2, str_alt);
        dtostrf(velocity, 5, 2, str_vel);
        uint8_t batPercent = map(voltage,3250,4250,0,100);      
        sprintf(txPacket,"Id:%d,Bat:%d,Lat:%s,Lon:%s,Alt:%s,Vel:%s", NODE_ID, batPercent, str_lat.c_str(), str_lon.c_str(), str_alt, str_vel);
        Serial.printf("\r\nSending packet: \"%s\" , Length: %d\r\n",txPacket, strlen(txPacket));
        Radio.Send( (uint8_t *)txPacket, strlen(txPacket) );
        state = LOWPOWER;
        break;
      }

      case LOWPOWER:
      {
        States_t prevState = state;
        lowPowerHandler();
        turnOffRGB();
        delay(30000);  // sleep interval time
        state = (prevState == StandbyRecharge) ? StandbyRecharge : GetLocation;
        break;
      }

      case GetLocation:
      {
        while (gpsSerial.available()) // check serial for new gps data
        {
         // encode gps data
          if ( gps.encode(gpsSerial.read()))   
          {
            //Serial.print("Getting GPS data: ");
            uint8_t noOfSatellites = gps.satellites.value();
            latitude = gps.location.lat();
            longitude = gps.location.lng();
            alti = gps.altitude.meters();
            velocity = gps.speed.mps();

        /*  Serial.print("SATS: ");
            Serial.println(noOfSatellites);
            Serial.print("LAT: ");
            Serial.println(latitude);
            Serial.print("LONG: ");
            Serial.println(longitude, 6);
            Serial.print("ALT: ");
            Serial.println(alti);
            Serial.print("SPEED: ");
            Serial.println(velocity);
            Serial.println("---------------------------"); */
          }
        }
        state = ReadVoltage;
        break;
      }

      case ReadVoltage:
      {
        voltage = BoardGetBatteryVoltage();
        //Serial.printf("\nBattery: %d\n", voltage);
        state = (voltage < 3250) ? StandbyRecharge : TX;
        break;
      }
      case StandbyRecharge:
      {
        voltage = BoardGetBatteryVoltage();
        state = (voltage > 3500) ? GetLocation : LOWPOWER;
        break;
      }
      default:
        break;
  }
}

// get the BatteryVoltage in mV
uint32_t BoardGetBatteryVoltage(void)
{
    float temp = 0;
    uint16_t volt;
    uint8_t pin = ADC;
    pinMode(VBAT_ADC_CTL, OUTPUT);
    digitalWrite(VBAT_ADC_CTL, LOW);

    for (int i = 0; i < 50; i++) // read 50 times and get average
        temp += analogReadmV(pin);
    volt = temp / 50;
    pinMode(VBAT_ADC_CTL, INPUT);
    volt = volt * 2;
    return volt;
}

// automatically called on successful transmit
void OnTxDone(void)
{
  Serial.print("Payload sent");
  turnOnRGB(COLOR_RECEIVED,0); // green LED when sent
}

// automatically called on transmit timeout
void OnTxTimeout(void)
{
  Serial.print("TX Timeout");
  turnOnRGB(COLOR_SEND,0); // red LED when timeout occurred
  Radio.Sleep( );
}