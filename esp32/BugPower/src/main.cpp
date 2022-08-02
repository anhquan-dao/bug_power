#include "Arduino.h"
#include "Wire.h"
#include "INA219.h"
#include "Base.h"

#define SDA 7
#define SCL 8
#define I2C_CLOCK 100000
#define INT_PIN 6
#define MPU_HZ 50

#define INCLUDE_xTaskDelayUntil 1

uint8_t i2c_addr = 0x40;

class INAHeader : public BaseHeader
{   
    public:
        using BaseHeader::BaseHeader;
};

BaseHeader    *test_header;
BaseError     *test_error;
BaseInterface *test_interface;

INA219 *device;

void setup()
{       
    Serial.begin(115200);
    Serial.println("OK");    
    
    device = new INA219(SDA, SCL, I2C_CLOCK, i2c_addr);
    device->configure(INA219::NO_RESET, INA219::BRNG_32V, INA219::PGA_1, INA219::BADC_8AVG, INA219::SADC_8AVG, INA219::CNT_SHUNT_BUS);
    device->calibrate(0.00387, 0.075, 20);

    test_header = new INAHeader(20);
    test_error  = new BaseError();
    test_interface = new BaseInterface(0x79, test_header, test_error);

    test_header->header_code[0] = 0x97;
    test_header->header_code[1] = 0x96;
    test_header->header_code[2] = 0x95;
    test_header->header_code[3] = 0x94;
}

void loop()
{   
    static uint16_t start_loop_time;
    static uint16_t loop_time;

    start_loop_time = millis();
    device->read_voltage();
    device->read_shunt_voltage();
    device->read_current();
    
    test_interface->writeMessage(0, BaseInterface::NORMAL, 0, device->voltage);
    test_interface->writeMessage(1, BaseInterface::NORMAL, 0, device->current);
    test_interface->writeMessage(2, BaseInterface::NORMAL, 0, device->shunt_voltage);
    test_interface->writeMessage(3, BaseInterface::NORMAL, 0, device->capacity);


#ifdef SERIAL_DEBUG

    Serial.println("-------------------");
    Serial.print("Bus voltage:   "); Serial.println(device->voltage,12);
    Serial.print("Current:       "); Serial.println(device->current,12);
    Serial.print("Shunt voltage: "); Serial.println(device->shunt_voltage,12);
    Serial.print("Capacity voltage: "); Serial.println(device->capacity,12);

    Serial.print("Configuration stored in INA219: ");
    device->i2c_read(INA_CONFIGURATION_REGISTER, 2);
    Serial.print("0x");
    if(device->rx_buffer[0] < 16) Serial.print("0");
    Serial.print(device->rx_buffer[0], HEX);
    if(device->rx_buffer[1] < 16) Serial.print("0");
    Serial.print(device->rx_buffer[1], HEX);
    Serial.println();

    Serial.print("Calibration stored in INA219: ");
    device->i2c_read(INA_CALIBRATION_REGISTER, 2);
    Serial.print("0x");
    if(device->rx_buffer[0] < 16) Serial.print("0");
    Serial.print(device->rx_buffer[0], HEX);
    if(device->rx_buffer[1] < 16) Serial.print("0");
    Serial.print(device->rx_buffer[1], HEX);
    Serial.println();
#endif

    loop_time = millis() - start_loop_time;
    if(loop_time < 100)
    {
        delay(100 - loop_time);
    }
}