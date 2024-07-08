#pragma once
#include <stdint.h>
#include <Arduino.h>
#include <esp_adc_cal.h>
#include <esp32-hal-adc.h>
#include "Sensors/vrailmonitor.h"

class SiC43x
{

public:
    /**
     * @brief Class to manage SiC43x buck converter devices
     *
     * @author Andrei Paduraru
     *
     * @param PGood PowerGood pin number
     * @param EN Enable pin number
     * @param defaultEN To determine whether the converter should default on or off
     * @param invertEN To determine whether EN output is inverted, i.e. if writing the pin HIGH would cause the converter to turn off
     * @param VRead Voltage output analog read pin
     * @param HighResistor Resistance value of pontential divider resistor connected to the top side
     * @param LowResistor Resistance value of potential divider resistor connected to the low side
     */

    SiC43x(int8_t PGood = -1, int8_t EN = -1, bool defaultEN = 0, bool invertEN = 0, int8_t VRead = -1, float HighResistor = 0, float LowResistor = 1) : _PGoodPin(PGood),
                                                                                                                                                         _ENPin(EN),
                                                                                                                                                         _defaultEN(defaultEN),
                                                                                                                                                         _invertEN(invertEN),
                                                                                                                                                         _VReadPin(VRead),
                                                                                                                                                         _servoVoltage("Servo Voltage", VRead, HighResistor, LowResistor){};

    void setup()
    {
        if (_PGoodPin >= 0)
        {
            pinMode(_PGoodPin, INPUT);
        }
        if (_ENPin >= 0)
        {
            pinMode(_ENPin, OUTPUT);
            setEN(_defaultEN); 
        }
        if (_VReadPin >= 0){
            _servoVoltage.setup(12,5,4);
        }
    };

    /**
     * @brief Method enables and disables buck converter chip based on level param and invertEN attribute
     *
     * @param Bucklevel Desired status of the buck, true - enabled; false - disabled.
     */
    void setEN(bool Bucklevel)
    {
        bool pinLevel = _invertEN ^ Bucklevel;
        digitalWrite(_ENPin, pinLevel);
    };

    /**
     * @brief Method to update output voltage and PGOOD values. Also handles the looping for the restart method to make it non blocking
     */
    void update()
    {
        if(_VReadPin >= 0){
            _servoVoltage.update(OutputV);
        }
        
        if (_PGoodPin >= 0)
        {
            PGOOD = digitalRead(_PGoodPin);
        }

        if (_restartFlag == 1)
        {
            restart(_offTime);
        }
    };

    void restart(uint32_t offTime)
    {

        if (_restartFlag == 0)
        {
            _prevtime = millis(); // time taken at the start of a restart routine, i.e. when the buck is just on normally
            _offTime = offTime; /*I don't like that this has to be stored but I can't figure out
            a different way to make this non blocking so it is what it is*/
        }

        _restartFlag = 1; //set the flag to 1 so the method gets called again during update
        
        setEN(false); //turn the buck off

        if (millis() - _prevtime > offTime)
        {
            setEN(true); //turn it back on after the offtime has passed
            _restartFlag = 0; //set the flag back to 0 to avoid restart being called again during update
            return;
        }
        return;
    };

    float getOutputV() { return OutputV; };
    bool getPGOOD() { return PGOOD; };

private:
    float OutputV;
    bool PGOOD;

    // Pin Definitions
    int8_t _PGoodPin = -1;
    int8_t _ENPin = -1;
    bool _defaultEN = 0;
    bool _invertEN = 0;
    int8_t _VReadPin = -1;

    // Potential divider setup. Default is no divider.
    float _RHighS = 0;
    float _RLowS = 1;

    bool _restartFlag = 0;

    uint32_t _prevtime;
    uint32_t _offTime;

    static constexpr int ADCMax = 4095;

    VRailMonitor _servoVoltage;
};
