/* 
 *   
 *  Project:          IoT Energy Meter with C/C++, Java/Spring, TypeScript/Angular and Dart/Flutter;
 *  About:            End-to-end implementation of a LoRaWAN network for monitoring electrical quantities;
 *  Version:          1.0;
 *  Backend Mote:     ATmega328P/ESP32/ESP8266/ESP8285/STM32;
 *  Radios:           RFM95w and LoRaWAN EndDevice Radioenge Module: RD49C;
 *  Sensors:          Peacefair PZEM-004T 3.0 Version TTL-RTU kWh Meter;
 *  Backend API:      Java with Framework: Spring Boot;
 *  LoRaWAN Stack:    IBM Long Range Signaling and Control (LMiC: LoRaWAN-MAC-in-C) version 1.5;
 *  Activation mode:  Activation by Personalization (ABP) or Over-the-Air Activation (OTAA);
 *  Author:           Adail dos Santos Silva;
 *  
 *  WARNINGS:
 *  Permission is hereby granted, free of charge, to any person obtaining a copy of
 *  this software and associated documentation files (the “Software”), to deal in
 *  the Software without restriction, including without limitation the rights to
 *  use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 *  the Software, and to permit persons to whom the Software is furnished to do so,
 *  subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 *  FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 *  COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 *  IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 *  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *  
 */

/********************************************************************
 _____              __ _                       _   _             
/  __ \            / _(_)                     | | (_)            
| /  \/ ___  _ __ | |_ _  __ _ _   _ _ __ __ _| |_ _  ___  _ __  
| |    / _ \| '_ \|  _| |/ _` | | | | '__/ _` | __| |/ _ \| '_ \ 
| \__/\ (_) | | | | | | | (_| | |_| | | | (_| | |_| | (_) | | | |
 \____/\___/|_| |_|_| |_|\__, |\__,_|_|  \__,_|\__|_|\___/|_| |_|
                          __/ |                                  
                         |___/                                   
********************************************************************/

/* Includes */
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_Sensor.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"

/* Definitions */
// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

/* Initialize OLED */
SSD1306AsciiAvrI2c oled;

/* Encryption keys to AES128 */
/* 
 *  This EUI must be in little-endian format, so least-significant-byte
 *  first. When copying an EUI from ttnctl output, this means to reverse 
 *  the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3, 0x70.
 */
static const u1_t PROGMEM APPEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

/* This should also be in little endian format, see above. */
static const u1_t PROGMEM DEVEUI[8] = {0x58, 0x1a, 0x10, 0x47, 0xc7, 0xd1, 0xb6, 0x72};
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

/* 
 *  This key should be in big endian format (or, since it is not really a
 *  number but a block of memory, endianness does not really apply).
 *  In practice, a key taken from ttnctl can be copied as-is.
 *  The key shown here is the semtech default key.
 */
static const u1_t PROGMEM APPKEY[16] = {0x80, 0x93, 0xb9, 0x7a, 0xf0, 0xb6, 0x9b, 0xa4, 0x90, 0x2c, 0xb5, 0x15, 0x5a, 0xd5, 0xdb, 0x5a};
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

static uint8_t btn_activated[1] = {0x01};
static osjob_t blinkjob;

static osjob_t sendjob;

/* Schedule TX every this many seconds (might become longer due to duty cycle limitations). */
const unsigned TX_INTERVAL = 5;

/* Pin mapping */
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, LMIC_UNUSED_PIN},
};

/* Define proper RST_PIN if required */
#define RST_PIN -1

#define LED_YELLOW 8
#define LED_GREEN 6

#define DHT_PIN 7
#define BTN_PIN 9

// DHT11 or DHT22:
#define DHTTYPE DHT11

/* Initialize DHT */
DHT dht(DHT_PIN, DHTTYPE);

/* Variables */
int buttonState = 0;     /* current state of the button. */
int lastButtonState = 0; /* previous state of the button. */
int joinstatus = 0;      /* store the join status.  0 = not joined, 1 = joined. */

int lineCounter = 0; /* OLED display line counter. Count the number of lines used. */

/*****************************
 _____           _      
/  __ \         | |     
| /  \/ ___   __| | ___ 
| |    / _ \ / _` |/ _ \
| \__/\ (_) | (_| |  __/
 \____/\___/ \__,_|\___|
*****************************/

/* Functions */
// During the join process, the LED_YELLOW blinks.
// If joined, the LED_YELLOW stops blinking.
void blinkfunc (osjob_t* job) {
    // Toggle LED
    joinstatus = !joinstatus;
    
    switchLed(joinstatus);
    
    // reschedule blink job
    os_setTimedCallback(job, os_getTime()+ms2osticks(100), blinkfunc);
}

void switchLed(int status){
    if (status == 1) {
      digitalWrite(LED_YELLOW, HIGH);
    } else {
      digitalWrite(LED_YELLOW, LOW);
    }
}

void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev)
    {
    case EV_SCAN_TIMEOUT:
        printlnOLED("EV_SCAN_TIMEOUT");
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        printlnOLED("EV_BEACON_FOUND");
        Serial.println(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        printlnOLED("EV_BEACON_MISSED");
        Serial.println(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        printlnOLED("EV_BEACON_TRACKED");
        Serial.println(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        printlnOLED("EV_JOINING");
        Serial.println(F("EV_JOINING"));

//        LMIC_startJoining();
//        LMIC.txChnl = 8;
//        LMIC_setDrTxpow(DR_SF9, 20);
        
        /* Start blinking */
        blinkfunc(&blinkjob);

        break;
    case EV_JOINED:
        printlnOLED("EV_JOINED");
        Serial.println(F("EV_JOINED"));

        showInformation();

        /* Switch Led off. */
        switchLed(0);

        /* Cancel blink job */
        os_clearCallback(&blinkjob);

        LMIC.adrTxPow = 0;
        //LMIC.rxDelay = 1;
        LMIC.dn2Dr = DR_SF9;
        LMIC_setAdrMode(false);

        /*  Valores possíveis (influe no tamanho do Payload):
             *  DR0=SF12, DR1=SF11, DR2=SF10, DR3=SF9, DR4=SF8 e DR5=SF7.
             */
        //LMIC_setDrTxpow(5, 20);
        LMIC_setDrTxpow(DR_SF10, 20);

        /* Stop listening for downstream data (periodical reception) */
        LMIC_stopPingable();

        /* Disable beacon tracking */
        LMIC_disableTracking();

        /* 
         *  Disable link check validation (automatically enable
         *  during join, but because slow data rates change max TX
         *  and not supported by TTN at this time). size, we don't use it in this example.
         */
        LMIC_setLinkCheckMode(false);
        break;
    case EV_RFU1:
        printlnOLED("EV_RFU1");
        Serial.println(F("EV_RFU1"));
        break;
    case EV_JOIN_FAILED:
        printlnOLED("EV_JOIN_FAILED");
        Serial.println(F("EV_JOIN_FAILED"));
        break;
    case EV_REJOIN_FAILED:
        printlnOLED("EV_REJOIN_FAILED");
        Serial.println(F("EV_REJOIN_FAILED"));
        break;
    case EV_TXCOMPLETE:
        printlnOLED("TXCOMPLETE");
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));

        /* Check if we have a downlink on either Rx1 or Rx2 windows */
        if (LMIC.txrxFlags)
        {
            if (TXRX_DNW1)
            {
                printlnOLED("TXRX_DNW1");
                Serial.println(F("Rx1 window"));
            }
            else if (TXRX_DNW2)
            {
                printlnOLED("TXRX_DNW2");
                Serial.println(F("Rx2 window"));
            }
            else if (TXRX_ACK)
            {
                Serial.println(F("Received ack"));
                printlnOLED("TXRX_ACK");
            }
        }

        if (LMIC.dataLen)
        {
            Serial.print(F("Received "));
            Serial.print(LMIC.dataLen);
            Serial.println(F(" bytes of payload."));

            printlnOLED("Rx " + String(LMIC.dataLen) + " bytes payload");

            if (LMIC.dataLen == 1)
            {
                uint8_t result = LMIC.frame[LMIC.dataBeg + 0];
                printlnOLED("Button val: " + String(result));

                if (result == 0)
                {
                    //Serial.println("RESULT 0");
                    digitalWrite(LED_YELLOW, LOW);
                    digitalWrite(LED_GREEN, LOW);
                }
                if (result == 1)
                {
                    //Serial.println("RESULT 1");
                    digitalWrite(LED_YELLOW, HIGH);
                    digitalWrite(LED_GREEN, LOW);
                }
                if (result == 2)
                {
                    //Serial.println("RESULT 2");
                    digitalWrite(LED_YELLOW, LOW);
                    digitalWrite(LED_GREEN, HIGH);
                }
                if (result == 3)
                {
                    //Serial.println("RESULT 3");
                    digitalWrite(LED_YELLOW, HIGH);
                    digitalWrite(LED_GREEN, HIGH);
                }
            }
            Serial.println();
        }

        /* Schedule next transmission */
        showInformation();

        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
        break;
    case EV_LOST_TSYNC:
        printlnOLED("EV_LOST_TSYNC");
        Serial.println(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        printlnOLED("EV_RESET");
        Serial.println(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        printlnOLED("EV_RXCOMPLETE");
        /* Data received in ping slot */
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        printlnOLED("EV_LINK_DEAD");
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        printlnOLED("EV_LINK_ALIVE");
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    default:
        printlnOLED("Unknown event");
        Serial.println(F("Unknown event"));
        break;
    }
}

void do_send(osjob_t *j)
{
    /* Check if there is not a current TX/RX job running */
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else
    {

        uint32_t humidity = dht.readHumidity(false) * 100;
        uint32_t temperature = dht.readTemperature(false) * 100;

        Serial.println("Humidity: " + String(humidity));
        Serial.println("Temperature: " + String(temperature));

        byte payload[4];
        payload[0] = highByte(humidity);
        payload[1] = lowByte(humidity);
        payload[2] = highByte(temperature);
        payload[3] = lowByte(temperature);
        
        /* Prepare upstream data transmission at the next possible time */
        printlnOLED("Tx data:" + String(humidity / 100) + "%/" + String(temperature / 100) + "C");
        LMIC_setTxData2(1, payload, sizeof(payload), 0);

        Serial.println(F("Packet queued"));
    }
    /* Next TX is scheduled after TX_COMPLETE event */
}

/*****************************
 _____      _               
/  ___|    | |              
\ `--.  ___| |_ _   _ _ __  
 `--. \/ _ \ __| | | | '_ \ 
/\__/ /  __/ |_| |_| | |_) |
\____/ \___|\__|\__,_| .__/ 
                     | |    
                     |_|    
******************************/

void setup()
{
    Serial.begin(9600);
    Serial.println(F("Starting"));

#if RST_PIN >= 0
    oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else  /* RST_PIN >= 0 */
    oled.begin(&Adafruit128x64, I2C_ADDRESS);
#endif /* RST_PIN >= 0 */
    /* Call oled.setI2cClock(frequency) to change from the default frequency */

    oled.setFont(System5x7);

#if INCLUDE_SCROLLING == 0
#error INCLUDE_SCROLLING must be non-zero.  Edit SSD1306Ascii.h
#endif /* INCLUDE_SCROLLING */
    /* Set auto scrolling at end of window */
    oled.setScrollMode(SCROLL_MODE_AUTO);

    pinMode(LED_YELLOW, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(BTN_PIN, INPUT);

    digitalWrite(BTN_PIN, LOW);

    dht.begin();

#ifdef VCC_ENABLE
    /* For Pinoccio Scout boards */
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
#endif

    /* LMIC init */
    os_init();

    /* Reset the MAC state. Session and pending data transfers will be discarded. */
    LMIC_reset();

    /* Channels Control */
    for (int b = 0; b < 8; ++b)
    {
        LMIC_disableSubBand(b);
    }

    for (int channel = 0; channel < 72; ++channel)
    {
        LMIC_disableChannel(channel);
    }

    /* Rede ATC ou ChirpStack */
    //LMIC_enableChannel(0);
    //LMIC_enableChannel(1);
    //LMIC_enableChannel(2);
    //LMIC_enableChannel(3);
    //LMIC_enableChannel(4);
    //LMIC_enableChannel(5);
    //LMIC_enableChannel(6);
    //LMIC_enableChannel(7);

    /* Rede TTN ou ChirpStack */
    LMIC_enableChannel(8);
    LMIC_enableChannel(9);
    LMIC_enableChannel(10);
    LMIC_enableChannel(11);
    LMIC_enableChannel(12);
    LMIC_enableChannel(13);
    LMIC_enableChannel(14);
    LMIC_enableChannel(15);

    //LMIC.rxDelay = 1;
    //LMIC.dn2Dr = DR_SF9;

    /* 
     * Use with Arduino Pro Mini ATmega328P 3.3V 8 MHz 
     * Let LMIC compensate for +/- 1% clock error
     * This tells LMIC to increase the reception windows,
     * if your watch is 1% faster or slower.
     */
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

    LMIC_startJoining();
    LMIC.txChnl = 8;
    LMIC_setDrTxpow(DR_SF9, 20);

    /* Start job (sending automatically starts OTAA too) */
    do_send(&sendjob);
}

/*****************************
 _                       
| |                      
| |     ___   ___  _ __  
| |    / _ \ / _ \| '_ \ 
| |___| (_) | (_) | |_) |
\_____/\___/ \___/| .__/ 
                  | |    
                  |_|    
*****************************/

void loop()
{
    /* Read the state of the button value */
    buttonState = digitalRead(BTN_PIN);

    /* Compare the buttonState to its previous state */
    if (buttonState != lastButtonState)
    {

        if (buttonState == HIGH)
        {
            printlnOLED("Tx button press");

            /* If the current state is HIGH then the button went from off to on */
            LMIC_setTxData2(101, btn_activated, sizeof(btn_activated), 0);
            Serial.println(F("Button On"));
        }
        else
        {
            /* If the current state is LOW then the button went from on to off */
            Serial.println(F("Button Off"));
        }
        /* Delay a little bit to avoid bouncing */
        delay(50);
    }

    /* Save the current state as the last state, for next time through the loop */
    lastButtonState = buttonState;

    os_runloop_once();
}

void printlnOLED(String msg)
{
    if (lineCounter == 10)
    {
        oled.clear();
    }

    oled.println(msg);
    delay(500);

    lineCounter = lineCounter + 1;
}

void showInformation()
{
    Serial.println("netid = " + String(LMIC.netid));
    Serial.println("devaddr = " + String(LMIC.devaddr));
    Serial.println("freq = " + String(LMIC.freq));
    Serial.println("txpow = " + String(LMIC.txpow));
    Serial.println("datarate = " + String(LMIC.datarate));
    Serial.println("rps = " + String(LMIC.rps));

    /* WARNING: 
   * The calculated RSSI and SNR does not correspond 100%
   * to the Netswork Servers RSSI and SNR console values. 
   */
    float freq = LMIC.freq / 1000000.0;
    int16_t rssi = (LMIC.rssi - 74);
    int snr = LMIC.snr * 0.25;

    printlnOLED("freq/pow/dr/rssi/snr");
    printlnOLED(String(freq, 3) + "/" + String(LMIC.txpow) + "/" + String(LMIC.datarate) + "/" + String(rssi) + "/" + String(snr));
}
