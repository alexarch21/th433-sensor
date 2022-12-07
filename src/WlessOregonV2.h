#ifndef WlessOregonV2_h
#define WlessOregonV2_h

// Oregon V2 decoder modfied - Olivier Lebrun
// Oregon V2 decoder added - Dominique Pierre
// New code to decode OOK signals from weather sensors, etc.
// 2010-04-11 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: ookDecoder.pde 5331 2010-04-17 10:45:17Z jcw $


//----------------------------------- Send the temp & humidity using oregon v2.1 protocol -------------
class OregonSensor {
    public:
        OregonSensor(uint8_t txPin, uint8_t channel, uint8_t sensorID, bool Humidity = false);
        bool init(void);
        void sendTempHumidity(int temp, uint8_t humm, bool battery);
    private:
        inline void sendOne(void);
        inline void sendZero(void);
        void sendData(const uint8_t *data, uint8_t size); // Send data buffer
        void sendPreamble(void);                        // Send preamble
        void sendPostamble(void);                       // Send postamble
        void sendOregon(void);                          // Send preamble, data, postamble
        void sendSync(void);
        int  sum(uint8_t count);                        // Count the buffer summ
        void calculateAndSetChecksum(void);
        bool existsHumidity;                            // Weither THGR2228N (send Humidity)
        uint8_t buffer[9];                              // The buffer to send the sensor data
        uint8_t tx_pin;                                 // Radio transmit pin number
        uint8_t tx_port, tx_bit;                        // Radio transmit port pinter (PORTB, PORTD, etc) and bit mask
        const unsigned long TIME    = 512;
        const unsigned long TWOTIME = TIME*2;
};

//------------------------------------------ class DecodeOOK ------------------------------------------------------
typedef void (*debugCB)(const uint8_t *raw_data, const uint8_t count);

class DecodeOOK {
    public:
        enum { UNKNOWN, T0, T1, T2, T3, OK, DONE };
 
        DecodeOOK(void)                                 { resetDecoder(); }
        void            begin(uint8_t int_num);
        virtual void    gotBit(int8_t value);           // add one bit to the packet data buffer
        bool            isDone (void) const             { return state == DONE; }
        const uint8_t*  getData(uint8_t& count) const   { count = pos;  return data; }
        bool            nextPulse(int16_t width);
        void            resetDecoder(void);
        void            manchester(int8_t value);       // store a bit using Manchester encoding
        void            alignTail(uint8_t max = 0);     // move bits to the front so that all the bits are aligned to the end
        void            reverseBits(void);
        void            reverseNibbles (void);
        void            done(void);
        void            attachDebugCallback(debugCB cb);
    protected:
        virtual int     decode(int16_t width) = 0;
        static void     interuptHandler(void);
        debugCB         debug_cb_func;
        uint8_t total_bits, bits, flip, state, pos, data[25];
};

//------------------------------------------ class OregonDecoderV2, decode Oregon station signals ----------------
class OregonDecoderV2 : public DecodeOOK {
    public:   
        OregonDecoderV2()                               { debug_cb_func = 0; }
        virtual void    gotBit(int8_t value);           // add one bit to the packet data buffer
        virtual int     decode(int16_t width);
        // Check and retrieve the useful data
        bool            receiveData(uint8_t& cnannel, uint8_t& sensorID, int16_t& temp, uint8_t& hum, bool& battOK);
    private:
        bool            decodeTempHumidity(int16_t& temp, uint8_t& hum, bool& battOK);
        uint8_t         sum(uint8_t count, const uint8_t* data);
        bool            isSummOK(uint16_t sensorType);
};

#endif