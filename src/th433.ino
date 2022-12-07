//
// Temp / humidity sensor with emulated OregonScientific 433MHz protocol, 3V powered, 60µA
//

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "WlessOregonV2.h"


// BME280 library
// ================================================================================================================

Adafruit_BME280 bme;


// OregonScientific protocol emulator
// ================================================================================================================

const byte TX_PIN       = 2;               // Tha transmitter data PIN number
const byte channel      = 0x20;            // Not sure it is good idea to change it
const byte sensorID     = 0xBB;            // The ID of the sensor (byte) can be changed if there are several sensors exist
const byte sendHumidity = true;            // Whether the sensor can send humidity
OregonSensor os(TX_PIN, channel, sensorID, sendHumidity);


// Temperature calibration value
// ================================================================================================================

const float tempCalibration = 0.0;


// Sleep and WDT
// ================================================================================================================

volatile uint8_t f_wdt = 1;

int count = 0;

ISR(WDT_vect)
{
    if( f_wdt == 0 ) f_wdt = 1;
}

void enterSleep(void)
{
    // Turn off ADC during sleep
    ADCSRA &= ~(1 << ADEN);

    // Enable the Watchdog interrupt before going to sleep (no WDE)
    WDTCSR |= _BV(WDIE);

    // Use power down sleep mode, the lowest power one
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    // Get ready to disable all clock oscillators and brown out detection
    cli();
    sleep_enable();
    sleep_bod_disable();
    sei();

    // Sleep the CPU
    sleep_cpu();

    // Continue execution here after the WDT fired, wake up the CPU now
    sleep_disable();

    // Enable interrupts
    sei();

    // Re-enable the peripherals
    power_all_enable();

    // Reenable ADC
    ADCSRA |= (1 << ADEN);
}


// Main setup and loop
// ================================================================================================================

void setup() 
{
    // Setup the WDT

    // Disable interrupts while configuring the watchdog
    cli();

    // Clear the reset flag
    MCUSR &= ~(1<<WDRF);

    // In order to change WDE or the prescaler, set WDCE (This will allow updates for 4 clock cycles)
    WDTCSR |= (1<<WDCE) | (1<<WDE);

    // Set new watchdog timeout prescaler value to 8s, do not set WDE (no system reset)
    WDTCSR = 1<<WDP0 | 1<<WDP3;

    // Interrupts back on
    sei();

    // Init OregonScientific protocol emulator
    os.init();

    // This blinks when data is transmitted
    pinMode(13, OUTPUT);

    // Init BME over I2C
    if( !bme.begin(0x77) ) {
        //Serial.println("Could not find a valid BME280 sensor !");
    }

    // Set BME to forced mode (take one sample and go to sleep)
    bme.setSampling(Adafruit_BME280::MODE_FORCED);
}
 
void loop() 
{
    if( f_wdt == 1 ) {

        if( ++count > 40 ) {        // Every 336 seconds (5:36)
      
            count = 0;
  
            // Read BME data, BME goes back to sleep after that
            bme.takeForcedMeasurement();
            float t = bme.readTemperature();
            float h = bme.readHumidity();

            // Apply calibration
            t += tempCalibration;

            // Send the data over RF using the OS protocol
            digitalWrite(13, HIGH);
            os.sendTempHumidity((int)(t * 10.0), (byte)h, true);
            digitalWrite(13, LOW);

        }

        f_wdt = 0;
    
        enterSleep();
    }
}
