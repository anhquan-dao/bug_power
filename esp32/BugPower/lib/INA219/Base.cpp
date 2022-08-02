#include "Base.h"

BaseInterface::BaseInterface(uint8_t id_) : id(id_)
{   
    header = new BaseHeader();
    error = new BaseError();
}

BaseInterface::BaseInterface(uint8_t id_, BaseHeader *header_, BaseError *error_) : id(id_)
{
    header = header_;
    error = error_;
}

BaseInterface::~BaseInterface()
{
    if(header!=NULL)
    {
        delete header;
    }
    if(error != NULL)
    {
        delete error;
    }
}

void BaseInterface::writeHeader(uint8_t header_)
{
    writeByte(header->operator()(header_));
}

void BaseInterface::writeError(uint8_t error_)
{
    writeByte(error->operator()(error_));
}

void BaseInterface::writeFloat(float data)
{
    uint8_t float_to_byte_arr[4];
    memcpy(float_to_byte_arr, &data, 4);
    writeByte(float_to_byte_arr, 4);
}

void BaseInterface::writeByte(uint8_t *data, uint8_t size)
{
    for(int i=0; i<size; i++)
    {
#ifdef SERIAL_DEBUG
        Serial.print("0x");
        if(*(data+i)<16) Serial.print("0");
        Serial.print(*(data+i), HEX);
        Serial.print(" ");
#else
        Serial.write(*(data+i));
#endif
    }
}
