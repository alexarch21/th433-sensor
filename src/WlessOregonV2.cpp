#include <Arduino.h>
#include "WlessOregonV2.h"

//----------------------------------- Send the temp & humidity using oregon v2.1 protocol -------------
OregonSensor::OregonSensor(uint8_t txPin, uint8_t channel, uint8_t sensorID, bool Humidity) {
    tx_pin          = txPin;
    tx_bit          = digitalPinToBitMask(tx_pin);
    tx_port         = digitalPinToPort(tx_pin);

    existsHumidity = Humidity;
    uint16_t type = 0xEA4C;								// by default emulate THGN132N
    if (existsHumidity)  type = 0x1A2D;                 // emulate THGR2228N
        
    buffer[0] = type >> 8;
    buffer[1] = type & 0xFF;
    buffer[2] = channel;
    buffer[3] = sensorID;
}

inline void OregonSensor::sendOne(void) {
    volatile uint8_t *tx_out = portOutputRegister(tx_port);
    *tx_out &= ~tx_bit;                                 // digitalWrite(tx_pin, LOW);
    delayMicroseconds(TIME);
    *tx_out |= tx_bit;                                  // digitalWrite(tx_pin, HIGH);
    delayMicroseconds(TWOTIME);
    *tx_out &= ~tx_bit;                                 // digitalWrite(tx_pin, LOW);
    delayMicroseconds(TIME);
}

inline void OregonSensor::sendZero(void) {
    volatile uint8_t *tx_out = portOutputRegister(tx_port);
    *tx_out |= tx_bit;                                  // digitalWrite(tx_pin, HIGH);
    delayMicroseconds(TIME);
    *tx_out &= ~tx_bit;                                 // digitalWrite(tx_pin, LOW);
    delayMicroseconds(TWOTIME);
    *tx_out |= tx_bit;                                  // digitalWrite(tx_pin, HIGH);
    delayMicroseconds(TIME);
}

bool OregonSensor::init(void) {
    if (tx_port == NOT_A_PORT)
        return false;
    pinMode(tx_pin, OUTPUT);
    digitalWrite(tx_pin, LOW);
    return true;
}

void OregonSensor::sendData(const uint8_t *data, uint8_t size) {
    for(uint8_t i = 0; i < size; ++i) {
        uint8_t m = 1;
        uint8_t d = data[i];
        (d & m)? sendOne(): sendZero(); m <<= 1;
        (d & m)? sendOne(): sendZero(); m <<= 1;
        (d & m)? sendOne(): sendZero(); m <<= 1;
        (d & m)? sendOne(): sendZero(); m <<= 1;
        (d & m)? sendOne(): sendZero(); m <<= 1;
        (d & m)? sendOne(): sendZero(); m <<= 1;
        (d & m)? sendOne(): sendZero(); m <<= 1;
        (d & m)? sendOne(): sendZero();
    }
}

void OregonSensor::sendOregon(void) {
    sendPreamble();
	//?sendSync();
    uint8_t size = 8;
    if (existsHumidity) size = 9;
    sendData(buffer, size);
    sendPostamble();
    digitalWrite(tx_pin, LOW);
}

void OregonSensor::sendPreamble(void) {
    uint8_t PREAMBLE[] = {0xFF,0xFF};
    sendData(PREAMBLE, 2);
}

void OregonSensor::sendPostamble(void) {
    sendZero();
    sendZero();
    sendZero();
    sendZero();
    if (!existsHumidity) return;                        // TNHN132N
    sendZero();
    sendZero();
    sendZero();
    sendZero();
}

void OregonSensor::sendSync(void) {
    uint8_t data = 0xA;
    (data & 1)? sendOne(): sendZero(); data >>= 1;
    (data & 1)? sendOne(): sendZero(); data >>= 1;
    (data & 1)? sendOne(): sendZero(); data >>= 1;
    (data & 1)? sendOne(): sendZero();
}

int OregonSensor::sum(uint8_t count) {
    int s = 0;
 
    for(uint8_t i = 0; i < count; i++) {
        s += (buffer[i]&0xF0) >> 4;
        s += (buffer[i]&0xF);
    }
 
    if(int(count) != count)
        s += (buffer[count]&0xF0) >> 4;
 
    return s;
}

void OregonSensor::calculateAndSetChecksum(void) {
    if (!existsHumidity) {
        int s = ((sum(6) + (buffer[6]&0xF) - 0xa) & 0xff);
        buffer[6] |=  (s&0x0F) << 4;     buffer[7] =  (s&0xF0) >> 4;
    } else {
        buffer[8] = ((sum(8) - 0xa) & 0xFF);
    }
}

void OregonSensor::sendTempHumidity(int temp, uint8_t humm, bool battery) {  // temperature centegrees * 10
    if(!battery) buffer[4] = 0x0C; else buffer[4] = 0x00;

    if(temp < 0) {
        buffer[6] = 0x08;
        temp *= -1; 
    } else {
        buffer[6] = 0x00;
    }
    uint8_t d3 = temp % 10;                              // Set temperature decimal part
    buffer[4] |= d3 << 4;
    temp /= 10;
    uint8_t d1 = temp / 10;                              // 1st decimal digit of the temperature
    uint8_t d2 = temp % 10;                              // 2nd deciaml digit of the temperature
    buffer[5] = d1 << 4;
    buffer[5] |= d2;

    if (existsHumidity) {                               // THGR2228N
        buffer[7] = humm / 10;
        buffer[6] |= (humm % 10) << 4;
    }
    calculateAndSetChecksum();

    sendOregon();                                       // The v2.1 protocol send the message two times
    delayMicroseconds(TWOTIME*8);
    sendOregon();
}

//------------------------------------------ class DecodeOOK ------------------------------------------------------
volatile int16_t wl_pulse;

void DecodeOOK::begin(uint8_t int_num) {
    attachInterrupt(int_num, DecodeOOK::interuptHandler, CHANGE);
}

void DecodeOOK::interuptHandler(void) {
    static int16_t last;
    // determine the pulse length in microseconds, for either polarity
    wl_pulse = micros() - last;
    last += wl_pulse;
}

bool DecodeOOK::nextPulse(int16_t width) {
    if (state != DONE)
        switch (decode(width)) {
            case -1:
                resetDecoder();
                break;
            case 1:
                done();
                break;
        }
    return isDone();
}

void DecodeOOK::resetDecoder(void) {
    total_bits = bits = pos = flip = 0;
    state = UNKNOWN;
}

void DecodeOOK::gotBit(int8_t value) {
    total_bits++;
    uint8_t *ptr = data + pos;
    *ptr = (*ptr >> 1) | (value << 7);
 
    if (++bits >= 8) {
        bits = 0;
        if (++pos >= sizeof data) {
            resetDecoder();
            return;
        }
    }
    state = OK;
}

void DecodeOOK::manchester(int8_t value) {
    flip ^= value;                                      // manchester code, long pulse flips the bit
    gotBit(flip);
}

void DecodeOOK::alignTail(uint8_t max) {
    // align bits
    if (bits != 0) {
        data[pos] >>= 8 - bits;
        for (uint8_t i = 0; i < pos; ++i)
            data[i] = (data[i] >> bits) | (data[i+1] << (8 - bits));
        bits = 0;
    }
    // optionally shift bytes down if there are too many of 'em
    if (max > 0 && pos > max) {
        uint8_t n = pos - max;
        pos = max;
        for (uint8_t i = 0; i < pos; ++i)
            data[i] = data[i+n];
    }
}

void DecodeOOK::reverseBits() {
    for (uint8_t i = 0; i < pos; ++i) {
        uint8_t b = data[i];
        for (uint8_t j = 0; j < 8; ++j) {
            data[i] = (data[i] << 1) | (b & 1);
            b >>= 1;
        }
    }
}

void DecodeOOK::reverseNibbles () {
    for (uint8_t i = 0; i < pos; ++i)
        data[i] = (data[i] << 4) | (data[i] >> 4);
}

void DecodeOOK::done () {
    while (bits)
        gotBit(0);                                      // padding
    state = DONE;
}

void DecodeOOK::attachDebugCallback(debugCB cb) {
    debug_cb_func = cb;
}

//------------------------------------------ class OregonDecoderV2, decode Oregon station signals ----------------
void OregonDecoderV2::gotBit(int8_t value) {
    if(!(total_bits & 0x01)) {
        data[pos] = (data[pos] >> 1) | (value ? 0x80 : 00);
    }
    total_bits++;
    pos = total_bits >> 4;
    if (pos >= sizeof data) {
        resetDecoder();
        return;
    }
    state = OK;
}

int OregonDecoderV2::decode(int16_t width) {
    if (200 <= width && width < 1200) {
        uint8_t w = width >= 700;
 
        switch (state) {
            case UNKNOWN:
                if (w != 0) {                           // Long pulse
                  ++flip;
                } else if (w == 0 && 24 <= flip) {      // Short pulse, start bit
                  flip = 0;
                  state = T0;
                } else {                                // Reset decoder
                    return -1;
                }
                break;
            case OK:
                if (w == 0) {                           // Short pulse
                    state = T0;
                } else {                                // Long pulse
                    manchester(1);
                }
                break;
            case T0:
                if (w == 0) {                           // Second short pulse
                    manchester(0);
                } else {                                // Reset decoder
                    return -1;
                }
                break;
        }

    } else if (width >= 2500  && pos >= 8) {
        return 1;
    } else {
        return -1;
    }
    return 0;
}
bool OregonDecoderV2::receiveData(uint8_t& channel, uint8_t& sensorID, int16_t& temp, uint8_t& hum, bool& battOK) {
    cli();
    int16_t p = wl_pulse;
    wl_pulse = 0;
    sei();
 
    if (p != 0) {
        if (nextPulse(p)) {
            if (debug_cb_func != NULL) {
                (*debug_cb_func)(data, pos);
            }
            if (decodeTempHumidity(temp, hum, battOK)) {
                channel  = data[2];
                sensorID = data[3];
                return true;
            } else {
                return false;
            }
        }
    }
    return false;  
}

bool OregonDecoderV2::decodeTempHumidity(int16_t& temp, uint8_t& hum, bool& battOK) {
    bool is_summ_ok = false;
    if (pos >= 8) {
        uint16_t Type = (data[0] << 8) | data[1];
        is_summ_ok = isSummOK(Type);
        if (is_summ_ok) {
            int16_t t = data[5] >> 4;                   // 1st decimal digit
            t *= 10;
            t += data[5] & 0x0F;                        // 2nd decimal digit
            t *= 10;
            t += data[4] >> 4;                          // 3rd decimal digit
            if (data[6] & 0x08) t *= -1;
            temp = t;
            hum = 0;
            battOK = !(data[4] & 0x0C);
            if (Type == 0x1A2D) {                       // THGR2228N
                hum  = data[7] & 0xF;
                hum *= 10;
                hum += data[6] >> 4;
            }
        }
    }
    resetDecoder();
    return is_summ_ok;
}

uint8_t OregonDecoderV2::sum(uint8_t count, const uint8_t* data) {
    uint16_t s = 0;
 
    for(uint8_t i = 0; i < count; ++i) {
        s += (data[i]&0xF0) >> 4;
        s += (data[i]&0xF);
    }
 
    if(int16_t(count) != count)
        s += (data[count]&0xF0) >> 4;
 
    return uint8_t(s);
}

bool OregonDecoderV2::isSummOK(uint16_t sensorType) {
    uint8_t s1 = 0;
    uint8_t s2 = 0;

    switch (sensorType) {
        case 0x1A2D:                                // THGR2228N
            s1 = (sum(8, data) - 0xa) & 0xFF;
            return (data[8] == s1);
        case 0xEA4C:                                // TNHN132N
            s1 = (sum(6, data) + (data[6]&0xF) - 0xa) & 0xff;
            s2 = (s1 & 0xF0) >> 4;
            s1 = (s1 & 0x0F) << 4;
            return ((s1 == data[6]) && (s2 == data[7]));
        default:
            break;
    }
    return false;
}
 