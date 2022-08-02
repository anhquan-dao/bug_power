#pragma once
#include <Arduino.h>
#include <Wire.h>

#define MAX_HEADER 20
#define MAX_ERROR  20

class BaseHeader
{
    public:
        BaseHeader() : BaseHeader(20) {}
        BaseHeader(uint8_t header_lim_) : header_lim(header_lim_)
        {
            if(header_lim > MAX_HEADER)
            {
                invalid_param = true;
            }
        }
        ~BaseHeader(){}

        virtual int8_t operator()(uint8_t header_c_)
        {   
            if(invalid_param)
            {
                return -1;
            }
            return header_code[header_c_];
        }


        uint8_t header_code[MAX_HEADER];
        uint8_t header_lim;

        boolean invalid_param;


};

class BaseError
{
    public:
        BaseError() : BaseError(20){}
        BaseError(uint8_t error_lim_) : error_lim(error_lim_)
        {
            if(error_lim > MAX_ERROR)
            {
                invalid_param = true;
            }
        }
        ~BaseError(){}

        virtual uint8_t operator()(uint8_t error_c_)
        {
            if(invalid_param)
            {
                return -1;
            }
            return error_code[error_c_];
        }

        uint8_t error_code[MAX_HEADER];
        uint8_t error_lim;

        boolean invalid_param;
};

class BaseInterface
{
    public:
        BaseInterface() = delete;
        BaseInterface(uint8_t id_);
        BaseInterface(uint8_t id_, BaseHeader *header_, BaseError *error_);
        ~BaseInterface();

        enum state_t
        {
            INIT,
            NORMAL,
            SHUTDOWN,
            ERROR
        };
        

        // Message Definition:
        // HEADER | MSG_CNT | STATE | ERROR | DATA
        // 8byte  | 8byte   | 8byte | 8byte | ____

        void writeMessage(uint8_t header_c_, state_t state_, uint8_t error_c_, uint8_t *data, uint8_t size)
        {   
            tx_msg_cnt++;
            writeByte(id);
            writeHeader(header_c_);
            writeByte(tx_msg_cnt);
            writeByte(state_);
            writeError(error_c_);
            writeByte(data, size);
#ifdef SERIAL_DEBUG
            Serial.println();
#endif
        }
        void writeMessage(uint8_t header_c_, state_t state_, uint8_t error_c_, float data)
        {   
            tx_msg_cnt++;
            writeByte(id);
            writeHeader(header_c_);
            writeByte(tx_msg_cnt);
            writeByte(state_);
            writeError(error_c_);
            writeFloat(data);
#ifdef SERIAL_DEBUG
            Serial.println();
#endif
        }

        void writeHeader(uint8_t header_);
        void writeError(uint8_t error_);

        void writeByte(uint8_t *data, uint8_t size);
        void writeByte(uint8_t data)
        {
            writeByte(&data, 1);     
        }
        void writeFloat(float data);

        uint8_t id;
        BaseHeader *header;
        uint8_t header_code;
        BaseError *error;
        uint8_t error_code;

        uint8_t tx_msg_cnt;

        state_t state;

        

        

};

