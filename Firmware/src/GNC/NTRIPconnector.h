#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <base64.h>

#include "Commands/packets/RTKtelemetrypacket.h"
#include <librnp/rnp_networkmanager.h>
#include "Config/pinmap_config.h"
#include "esp_wifi.h"

class NTRIPConnector {
    public:
        NTRIPConnector(RnpNetworkManager& networkmanager) : m_networkmanager(networkmanager) {};
        void setup();
        void update();

    private:
        // const char* m_ssid = "Olis phone ";
        // const char* m_password = "Oliver1234";

        const char* m_ssid = "McNaughty";
        const char* m_password = "harveyhouse25";
        

        //connection to imperial RTK base station
        const char* m_casterHost = "euref-ip.net";
        const int   m_casterPort = 2101;
        const char* m_mountpoint = "LICC00GBR0";
        const char* m_ntripUser  = "ICLR_GNC";
        const char* m_ntripPass  = "iclr_GNC1";


        // connection to standalone RTK base station
        // const char* m_casterHost = "3.143.243.81";
        // const int   m_casterPort = 2101;
        // const char* m_mountpoint = "ICLR_GNC";
        // const char* m_ntripUser  = "jackx.crane@hotmail.com";
        // const char* m_ntripPass  = "WEEK2374";
        String m_lastGPGGA = "";

        static constexpr int kRTKSampleRateHz = 20;
        static constexpr int kRTKSampleDeltaMs = 1000 / kRTKSampleRateHz;
        static constexpr size_t kGNSSRxBufferSize = 4096;

        const int m_dataDelta = 2000;
        const int m_GPGGADelta = 1000;
        const int m_GNSSPollDelta = kRTKSampleDeltaMs;
        const int m_telemetryDelta = kRTKSampleDeltaMs;

        unsigned long m_prev_timestamp_1 = 0;
        unsigned long m_prev_timestamp_2 = 0;
        unsigned long m_lastGGAGottenMs = 0;
        unsigned long m_lastGGASentMs = 0;
        unsigned long m_lastVTGGottenMs = 0;
        unsigned long m_lastTelemetrySentMs = 0;


        uint32_t m_lastRtcmRxMs = 0;
        uint32_t m_lastStatMs   = 0;
        uint32_t m_rtcmBytesSec = 0;
        uint32_t m_rtcmBytesAcc = 0;
        uint32_t m_lastReconnectAttemptMs = 0;
        uint32_t m_lastWifiStatMs = 0;
        uint32_t m_lastPacketDebugMs = 0;
        
        WiFiClient client;
        HardwareSerial GNSSserial = HardwareSerial(1);
        RnpNetworkManager& m_networkmanager;

        double m_latitudeDeg = 0.0;
        double m_longitudeDeg = 0.0;
        double m_altitudeM = 0.0;
        double m_originLatitudeDeg = 0.0;
        double m_originLongitudeDeg = 0.0;
        double m_originAltitudeM = 0.0;
        double m_velocityEastMs = 0.0;
        double m_velocityNorthMs = 0.0;
        double m_velocityUpMs = 0.0;
        uint8_t m_fixQuality = 0;
        uint32_t m_gnssTimeOfDayMs = 0;
        bool m_hasPosition = false;
        bool m_hasNedOrigin = false;
        bool m_hasVelocity = false;
        bool m_hasGNSSTime = false;
        uint32_t m_lastVTGParseMs = 0;
        char m_lastVTGParseStatus[32] = "none";
        char m_lastRawGGA[256] = "";
        char m_lastRawVTG[256] = "";
        char m_lastRawNTR[256] = "";

        void connectWIFI();
        void connectNTRIP();
        void updateCorrectionData();
        void connectUART();
        void requestGPGGA();
        void requestVTG();
        void readGNSSData();
        void printTelemetryStatus();
        void parseNMEALine(char *nmea);
        void getGPNTR();
        void parseGPNTR(char *nmea);
        void getGNGGA();
        void parseGNGGA(char *nmea);
        void getNewData();
        void getGPGGA();
        void parseGPGGA(char *nmea);
        void parseVTG(char *nmea);
        void sendGPGGA();
        void sendData();
        void printWiFiScan();

        char lineBuf[256];
        uint8_t linePos = 0;
};
