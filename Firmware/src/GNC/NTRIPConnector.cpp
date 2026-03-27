#include "NTRIPConnector.h"

void NTRIPConnector::setup()
{
	// // Initialize WiFi
	// connectWIFI();

	// // Connect to NTRIP caster
	// connectNTRIP();

	// Set up UART for UM980 Receiver
	connectUART();

	// First connect to um980 receiver to get GPGGA data
	// Wait for good data
	// Connect to ntrip caster
}

void NTRIPConnector::update() {

	// getNewData();
	getGPNTR();
	// getGNGGA();

} 

void NTRIPConnector::getNewData() {
	// getGPNTR();
	getGPGGA();
	sendGPGGA();
	// getGPNTR();
	updateCorrectionData();
}

void NTRIPConnector::connectWIFI()
{
	Serial.println("Connecting to WiFi...");
	WiFi.begin(m_ssid, m_password);
	WiFi.persistent(false);
	WiFi.setAutoReconnect(true);
	WiFi.setSleep(false);                  // disable modem sleep
	esp_wifi_set_ps(WIFI_PS_NONE);         // IDF API, no power-save
	esp_wifi_set_max_tx_power(78); 
};

void NTRIPConnector::connectNTRIP()
{
	if (WiFi.status() != WL_CONNECTED) return;

	Serial.println("Connecting to NTRIP caster...");

	if (client.connect(m_casterHost, m_casterPort))
	{
		
		String auth = base64::encode(String(m_ntripUser) + ":" + String(m_ntripPass));
		String req = "GET /" + String(m_mountpoint) + " HTTP/1.0\r\n";

		req += "User-Agent: NTRIP ESP32 1.0\r\n";
		req += "Authorization: Basic " + String(auth) + "\r\n\r\n";
		client.write((const uint8_t *)req.c_str(), req.length());
		client.flush(); // make sure it leaves the buffer

		Serial.println("Sent header:\n" + req);

		while (client.connected() && !client.available())
		{
			delay(100);
		}

		// Read the response header

		String header = "";
		while (client.available())
		{
			char c = client.read();
			header += c;
			if (header.endsWith("\r\n\r\n"))
				break;
		}

		Serial.println("Caster Response Header:");
		Serial.println(header);

		if (!header.startsWith("ICY 200 OK") && header.indexOf("200 OK") == -1)
		{
			Serial.println("NTRIP Caster did not accept connection.");
			client.stop();
			return;
		}

		Serial.println("Receiving RTCM/NMEA data...");
	}
	else
	{
		Serial.println("Failed to connect to NTRIP caster.");
		return;
	};
};

void NTRIPConnector::updateCorrectionData()
{
    // Reconnect with a simple backoff (e.g., 3s) so we don't hammer the caster
    if (!client.connected()) {
        uint32_t now = millis();
        if (now - m_lastReconnectAttemptMs >= 3000) {
            connectNTRIP();
            m_lastReconnectAttemptMs = now;
        }
        return;
    }

    // 1) Pull RTCM from socket in chunks
    while (client.available() > 0) {
        uint8_t buf[512];                            // chunk buffer
        int n = client.read(buf, sizeof(buf));       // read as much as available (non-blocking)
        if (n <= 0) break;

        // 2) Forward to GNSS UART, handling partial writes
        int sent = 0;
        while (sent < n) {
            // throttle by UART TX FIFO capacity if available
            int room = GNSSserial.availableForWrite();
            if (room <= 0) { yield(); continue; }    // give RTOS/WiFi time

            int toSend = (n - sent);
            if (toSend > room) toSend = room;

            int w = GNSSserial.write(buf + sent, toSend);
            if (w > 0) sent += w;
            else yield(); // avoid a tight loop if UART is momentarily busy
        }

        // 3) Stats
        m_rtcmBytesAcc += (uint32_t)n;
        m_lastRtcmRxMs  = millis();
    }

    // 4) Lightweight 1 Hz debug (bytes/sec + last-rtcm age)
    uint32_t now = millis();
    if (now - m_lastStatMs >= 1000) {
        m_rtcmBytesSec = m_rtcmBytesAcc;
        m_rtcmBytesAcc = 0;
        uint32_t ageMs = (m_lastRtcmRxMs == 0) ? 0xFFFFFFFF : (now - m_lastRtcmRxMs);

        // Comment this out if you want it silent:
        Serial.printf("RTCM: %lu B/s, last %lu ms ago\n",
                      (unsigned long)m_rtcmBytesSec,
                      (unsigned long)((ageMs == 0xFFFFFFFF) ? 0 : ageMs));

        m_lastStatMs = now;
    }
}

void NTRIPConnector::connectUART()
{
	Serial.println("Connecting to UM980 Receiver via UART...");

	// Initialize the serial port for the GNSS receiver
	GNSSserial.begin(115200, SERIAL_8N1, PinMap::NTRIP_RX, PinMap::NTRIP_TX);

	if (GNSSserial) {
		Serial.println("UM980 Receiver connected successfully.");
	} else {
		Serial.println("Failed to connect to UM980 Receiver.");
	}
}

void NTRIPConnector::getGPNTR() // this is polling data from um980
{
	unsigned long currentTime = millis();
	if (currentTime - m_prev_timestamp_1 >= m_dataDelta) {
		GNSSserial.println(F("GPNTR\r\n"));  
		m_prev_timestamp_1 = currentTime;
	}

	if (GNSSserial.available() > 0) {
		char c = GNSSserial.read();

		if (c == '\r') {
			// skip CR
		}
		else if (c == '\n') {
			// end-of-line — terminate & parse
			lineBuf[linePos] = '\0';
			linePos = 0;
			parseGPNTR(lineBuf);
		}
		else {
			// accumulate, but avoid overflow
			if (linePos < sizeof(lineBuf) - 1) {
				lineBuf[linePos++] = c;
			}
		}
	}
}

void NTRIPConnector::parseGPNTR(char *nmea) {
    // Quick header check
    if (strncmp(nmea, "$GPNTR,", 7) != 0) return;

    // Tokenize into fields
    const int MAXTOK = 16;
    char *tokens[MAXTOK];
    int   tokCount = 0;
    tokens[tokCount++] = nmea;
    for (char *p = nmea; *p && tokCount < MAXTOK; p++) {
        if (*p == ',') {
            *p = '\0';
            tokens[tokCount++] = p + 1;
        }
    }

    // Parse fields
    float utcTime = atof(tokens[1]);       // hhmmss.ss
	int   qual    = atoi(tokens[2]); 
    float north_m = atof(tokens[3]);       // North offset
    float east_m  = atof(tokens[4]);       // East  offset
    float up_m    = atof(tokens[5]);       // Up    offset

    // Break UTC into H:M:S
    int   hh = int(utcTime / 10000);
    int   mm = int((utcTime - hh * 10000) / 100);
    float ss = utcTime - hh * 10000 - mm * 100;

    // Single CSV-style print: UTC, north, east, up
	Serial.printf(
	"%d,%02d:%02d:%05.2f,%.3f,%.3f,%.3f\n",
	qual, hh, mm, ss,
	north_m, east_m, up_m
	);
	m_x = north_m; // Update offsets
	m_y = east_m;
	m_z = up_m;
	sendData();
    // Yield to Wi-Fi/RTOS so we don't starve the stack
    yield();
}

void NTRIPConnector::getGNGGA() 
{
	unsigned long currentTime = millis();
	if (currentTime - m_prev_timestamp_2 >= m_dataDelta) {
		GNSSserial.println(F("GNGGA\r\n"));  
		m_prev_timestamp_2 = currentTime;
	}

	if (GNSSserial.available() > 0) {
		char c = GNSSserial.read();

		if (c == '\r') {
			// skip CR
		}
		else if (c == '\n') {
			// end-of-line — terminate & parse
			lineBuf[linePos] = '\0';
			linePos = 0;
			parseGNGGA(lineBuf);
		}
		else {
			// accumulate, but avoid overflow
			if (linePos < sizeof(lineBuf) - 1) {
				lineBuf[linePos++] = c;
			}
		}
	}
}

void NTRIPConnector::parseGNGGA(char *nmea) {
	// Quick check for GGA header
	if (strncmp(nmea, "$GNGGA,", 7) != 0) return;

		// Split on commas into up to 15 tokens
	const int MAXTOK = 15;
	char *tokens[MAXTOK];
	int   tokCount = 0;
	tokens[tokCount++] = nmea;
	for (char *p = nmea; *p && tokCount < MAXTOK; p++) {
		if (*p == ',') {
			*p = '\0';
			tokens[tokCount++] = p+1;
		}
	}

	// tokens[] now holds:
	// [0] = "$GNGGA"
	// [1] = UTC time (hhmmss.ss)
	// [2] = latitude  (ddmm.mmmm)
	// [3] = N/S
	// [4] = longitude (dddmm.mmmm)
	// [5] = E/W
	// [6] = fix quality
	// [7] = num satellites
	// [8] = HDOP
	// [9] = altitude
	// [10]= "M"
	// [11]= geoid sep
	// [12]= "M"
	// …rest omitted

	// Convert strings to numbers
	float  utcTime  = atof(tokens[1]);
	float  latRaw   = atof(tokens[2]);
	char   latHemi  = tokens[3][0];
	float  lonRaw   = atof(tokens[4]);
	char   lonHemi  = tokens[5][0];
	int    fixQ     = atoi(tokens[6]);
	int    sats     = atoi(tokens[7]);
	float  hdop     = atof(tokens[8]);
	float  altitude = atof(tokens[9]);
	float  geoid    = atof(tokens[11]);

	// // Convert ddmm.mmmm to decimal degrees
	// int   latDeg = int(latRaw / 100);
	// float latMin = latRaw - latDeg * 100;
	// float latitude = latDeg + latMin / 60.0;
	// if (latHemi == 'S') latitude = -latitude;

	// int   lonDeg = int(lonRaw / 100);
	// float lonMin = lonRaw - lonDeg * 100;
	// float longitude = lonDeg + lonMin / 60.0;
	// if (lonHemi == 'W') longitude = -longitude;

	// // UTC hour/min/sec
	// int hour = int(utcTime / 10000);
	// int minute = int((utcTime - hour * 10000) / 100);
	// float second = utcTime - hour * 10000 - minute * 100;

	Serial.printf(
		"%02d:%02d:%05.2f,%09.5f,%c,%010.5f,%c,%d,%d,%.1f,%.3f,%.3f\n",
		int(utcTime / 10000),
		int((utcTime - int(utcTime / 10000) * 10000) / 100),
		utcTime - int(utcTime / 10000) * 10000 - int((utcTime - int(utcTime / 10000) * 10000) / 100) * 100,
		latRaw, latHemi,
		lonRaw, lonHemi,
		fixQ, sats, hdop,
		altitude, geoid
	);
}

void NTRIPConnector::getGPGGA() {
	unsigned long currentTime = millis();
	if (currentTime - m_lastGGAGottenMs >= m_GPGGADelta) {
		GNSSserial.println(F("GPGGA\r\n"));  
		m_lastGGAGottenMs = currentTime;
	}

	if (GNSSserial.available() > 0) {
		char c = GNSSserial.read();

		if (c == '\r') {
			// skip CR
		}
		else if (c == '\n') {
			// end-of-line — terminate & parse
			lineBuf[linePos] = '\0';
			linePos = 0;
			parseGPGGA(lineBuf);
		}
		else {
			// accumulate, but avoid overflow
			if (linePos < sizeof(lineBuf) - 1) {
				lineBuf[linePos++] = c;
			}
		}
	}
}

void NTRIPConnector::parseGPGGA(char *nmea) {
    // Find '*'
    char *star = strrchr(nmea, '*');
    if (!star || strlen(star) < 3) return; // malformed

    // Compute checksum over chars between '$' and '*'
    uint8_t cs = 0;
    for (char *p = nmea + 1; p < star; ++p) cs ^= (uint8_t)*p;

    // Parse reported checksum
    char h1 = star[1], h2 = star[2];
    auto hexVal = [](char h) -> int {
        if (h >= '0' && h <= '9') return h - '0';
        if (h >= 'A' && h <= 'F') return 10 + (h - 'A');
        if (h >= 'a' && h <= 'f') return 10 + (h - 'a');
        return -1;
    };
    int hi = hexVal(h1), lo = hexVal(h2);
    if (hi < 0 || lo < 0 || ((uint8_t)((hi << 4) | lo) != cs)) return; // checksum fail

    // Normalize talker ID to GP
    if (strncmp(nmea + 1, "GNGGA", 5) == 0) {
        nmea[1] = 'G';
        nmea[2] = 'P';
    }

    // Recompute checksum (in case we changed talker ID)
    cs = 0;
    for (char *p = nmea + 1; p < star; ++p) cs ^= (uint8_t)*p;
    char csHex[3];
    snprintf(csHex, sizeof(csHex), "%02X", cs);

    // Store in lastGGA_ with CRLF
    *star = '\0';
    m_lastGPGGA = String(nmea) + "*" + csHex + "\r\n";

}

void NTRIPConnector::sendGPGGA() {
    if (!client.connected() || m_lastGPGGA.length() == 0) return;

    uint32_t now = millis();
    if (now - m_lastGGASentMs >= m_GPGGADelta) {
        client.print(m_lastGPGGA);  // must include \r\n
        m_lastGGASentMs = now;

        // Optional debug
        Serial.print("Sent GPGGA to caster: ");
        Serial.print(m_lastGPGGA);
    }
}

void NTRIPConnector::sendData() {
	RTKTelemetryPacket telemetry;

	telemetry.header.type = static_cast<uint8_t>(120);
	telemetry.header.source = 103;
	telemetry.header.source_service = 1; // does this work if it isnt source service 1
	telemetry.header.destination = 2;
	telemetry.header.destination_service = 6;
	telemetry.header.uid = 1;
    telemetry.x_input = m_x;
    telemetry.y_input = m_y;
    telemetry.z_input = m_z;
    telemetry.u_input = 4;
    telemetry.v_input = 5;
    telemetry.w_input = 6;

	m_networkmanager.sendPacket(telemetry);
}