#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <string>
#include "oledscreen.h"


#include "Config/types.h"
#include "Config/systemflags_config.h"


void OLEDScreen::setupScreen(){

    m_wire.setTimeout(5); // to stop the stupid lag
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally (from test code)
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        //_systemstatus.newFlag(SYSTEM_FLAG::ERROR_DISPLAY);

        m_wire.setTimeout(1); // to stop the stupid lag
        
    }
    else{
        timeEnteredSetup = millis();
    }

}

void OLEDScreen::displayLogos(){
        if (millis() - timeEnteredSetup < iclrLogoDelay){
            displayICLRLogo();
        }

        if (millis() - timeEnteredSetup > iclrLogoDelay && millis() - timeEnteredSetup < setupDelay){
            displayNimbusLogo();
        }
}

void OLEDScreen::displayICLRLogo(){

    display.clearDisplay();
    display.drawBitmap(48, 0, iclr_logo.data(), 32, 32, WHITE);
    display.display();

}

void OLEDScreen::displayNimbusLogo(){

    display.clearDisplay();
    display.drawBitmap(0, 2, nimbus_logo.data(), 128, 32, WHITE);
    display.display();
}

void OLEDScreen::updateBattV(float battV, float maxbattV){
    _battV = battV;
    _maxbattV = maxbattV;
}

void OLEDScreen::updateDepV(float depV){
    _depV = depV;
}

void OLEDScreen::updateLogicV(float logicV){
    _logicV = logicV;
}

void OLEDScreen::updateState(std::string systemstatus){
    _systemstatus = systemstatus;
}


void OLEDScreen::updateScreen(){

    if (millis() - timeEnteredSetup < setupDelay)
    {
        displayLogos();
    }

    else{

        display.clearDisplay();
        display.setTextSize(1);             // Normal 1:1 pixel scale
        display.setTextColor(SSD1306_WHITE);        // Draw white text
        display.setCursor(0,0);             // Start at top-left corner


        display.println(("STATE: " + _systemstatus).c_str());
        display.println(("BATTERY: " + std::to_string(static_cast<uint16_t>(_battV)) + "mV").c_str());
        display.println(("DEPLOYMENT: " + std::to_string(static_cast<uint16_t>(_depV)) + "mV").c_str());
        display.println(("LOGIC: " + std::to_string(static_cast<uint16_t>(_logicV)) + "mV").c_str());

        //Battery icon in top right corner
        display.drawRect(113, 0, 15, 6, WHITE);
        display.drawRect(128, 3, 1, 2, WHITE);
        chargebars = (_battV/_maxbattV)*11;  //number of bars of charge out of 11
        display.fillRect(CHARGE_AREA_START_X, CHARGE_AREA_START_Y, chargebars, 4, WHITE);
        //filling in battery icon with rectangular bars


        display.display();

        // Serial.println("Default screen updated successfully");
    }
}
