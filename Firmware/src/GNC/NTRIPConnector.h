#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <base64.h>

#include "RTKTelemetryPacket.h"
#include <librnp/rnp_networkmanager.h>
#include "Config/pinmap_config.h"
#include "esp_wifi.h"

class NTRIPConnector {
    public:
        NTRIPConnector(RnpNetworkManager& networkmanager) : m_networkmanager(networkmanager) {};
        void setup();
        void update();

    private:
        const char* m_ssid = "McNaughty";
        // const char* m_ssid = "Jack";
        const char* m_password = "harveyhouse25";
        // const char* m_password = "password1";
        const char* m_casterHost = "3.143.243.81";
        const int   m_casterPort = 2101;
        const char* m_mountpoint = "ICLR_GNC";
        const char* m_ntripUser  = "jackx.crane@hotmail.com";
        const char* m_ntripPass  = "WEEK2374";
        String m_lastGPGGA = "";

        const int m_dataDelta = 1000;
        const int m_GPGGADelta = 5000;

        unsigned long m_prev_timestamp_1 = 0;
        unsigned long m_prev_timestamp_2 = 0;
        unsigned long m_lastGGAGottenMs = 0;
        unsigned long m_lastGGASentMs = 0;


        uint32_t m_lastRtcmRxMs = 0;
        uint32_t m_lastStatMs   = 0;
        uint32_t m_rtcmBytesSec = 0;
        uint32_t m_rtcmBytesAcc = 0;
        uint32_t m_lastReconnectAttemptMs = 0;
        
        WiFiClient client;
        HardwareSerial GNSSserial = HardwareSerial(1);
        RnpNetworkManager& m_networkmanager;

        void connectWIFI();
        void connectNTRIP();
        void updateCorrectionData();
        void connectUART();
        void getGPNTR();
        void parseGPNTR(char *nmea);
        void getGNGGA();
        void parseGNGGA(char *nmea);
        void getNewData();
        void getGPGGA();
        void parseGPGGA(char *nmea);
        void sendGPGGA();
        void sendData();

        char lineBuf[128];
        uint8_t linePos = 0;

        float m_x = 1.0f; // North offset
        float m_y = 2.0f; // East  offset
        float m_z = 3.0f; // Up    offset
};