#include "NTRIPconnector.h"

namespace {

bool verifyNMEAChecksum(const char *nmea)
{
    const char *star = strrchr(nmea, '*');
    if (nmea[0] != '$' || star == nullptr || strlen(star) < 3) return false;

    uint8_t cs = 0;
    for (const char *p = nmea + 1; p < star; ++p) cs ^= static_cast<uint8_t>(*p);

    auto hexVal = [](char h) -> int {
        if (h >= '0' && h <= '9') return h - '0';
        if (h >= 'A' && h <= 'F') return 10 + (h - 'A');
        if (h >= 'a' && h <= 'f') return 10 + (h - 'a');
        return -1;
    };

    int hi = hexVal(star[1]);
    int lo = hexVal(star[2]);
    return hi >= 0 && lo >= 0 && static_cast<uint8_t>((hi << 4) | lo) == cs;
}

double nmeaPositionToDecimal(double raw, char hemi)
{
    int degrees = static_cast<int>(raw / 100.0);
    double minutes = raw - static_cast<double>(degrees * 100);
    double decimal = static_cast<double>(degrees) + (minutes / 60.0);
    if (hemi == 'S' || hemi == 'W') decimal *= -1.0;
    return decimal;
}

bool parseDoubleField(const char *field, double &value)
{
	if (field == nullptr || field[0] == '\0') return false;

	char *end = nullptr;
	value = strtod(field, &end);
	return end != field && *end == '\0';
}

bool parseGGAUtcTimeOfDayMs(const char *field, uint32_t &timeOfDayMs)
{
	double utcTime = 0.0;
	if (!parseDoubleField(field, utcTime)) return false;

	int hour = static_cast<int>(utcTime / 10000.0);
	int minute = static_cast<int>((utcTime - (hour * 10000.0)) / 100.0);
	double second = utcTime - (hour * 10000.0) - (minute * 100.0);

	if (hour < 0 || hour > 23) return false;
	if (minute < 0 || minute > 59) return false;
	if (second < 0.0 || second >= 60.0) return false;

	double msOfDay = (((hour * 60.0) + minute) * 60.0 + second) * 1000.0;
	timeOfDayMs = static_cast<uint32_t>(msOfDay + 0.5);
	if (timeOfDayMs >= 86400000UL) timeOfDayMs = 86399999UL;
	return true;
}

const char *decodeVTGVelocity(const char *nmea,
                              double &velocityNorthMs,
                              double &velocityEastMs,
                              double &velocityUpMs)
{
	velocityNorthMs = 0.0;
	velocityEastMs = 0.0;
	velocityUpMs = 0.0;

	if (nmea == nullptr || nmea[0] == '\0') return "none";
	if (nmea[0] != '$' || strncmp(nmea + 3, "VTG,", 4) != 0) return "not_vtg";
	if (!verifyNMEAChecksum(nmea)) return "checksum";

	char copy[256];
	snprintf(copy, sizeof(copy), "%s", nmea);

	char *star = strrchr(copy, '*');
	if (star == nullptr) return "missing_star";
	*star = '\0';

	const int MAXTOK = 10;
	char *tokens[MAXTOK];
	int tokCount = 0;
	tokens[tokCount++] = copy;
	for (char *p = copy; *p && tokCount < MAXTOK; p++) {
		if (*p == ',') {
			*p = '\0';
			tokens[tokCount++] = p + 1;
		}
	}

	if (tokCount < 9) return "too_few_fields";

	bool hasModeField = tokCount > 9 && tokens[9][0] != '\0';
	if (hasModeField && tokens[9][0] == 'N') return "mode_N";

	double courseDeg = 0.0;
	if (!(tokCount > 2 && tokens[2][0] == 'T' && parseDoubleField(tokens[1], courseDeg))) {
		return "no_course";
	}

	double speedMs = 0.0;
	bool hasSpeedKmh = tokCount > 8 && tokens[8][0] == 'K' && parseDoubleField(tokens[7], speedMs);
	if (hasSpeedKmh) {
		speedMs /= 3.6;
	}
	else {
		double speedKnots = 0.0;
		bool hasSpeedKnots = tokCount > 6 && tokens[6][0] == 'N' && parseDoubleField(tokens[5], speedKnots);
		if (!hasSpeedKnots) return "no_speed";
		speedMs = speedKnots * 0.514444;
	}

	if (speedMs < 0.0) return "negative_speed";

	double courseRad = courseDeg * DEG_TO_RAD;
	velocityEastMs = speedMs * sin(courseRad);
	velocityNorthMs = speedMs * cos(courseRad);
	velocityUpMs = 0.0;
	return nullptr;
}

void geodeticToNed(double latitudeDeg,
                   double longitudeDeg,
                   double altitudeM,
                   double originLatitudeDeg,
                   double originLongitudeDeg,
                   double originAltitudeM,
                   double &northM,
                   double &eastM,
                   double &downM)
{
    static constexpr double wgs84SemiMajorAxisM = 6378137.0;
    static constexpr double wgs84Flattening = 1.0 / 298.257223563;
    static constexpr double wgs84EccentricitySquared =
        wgs84Flattening * (2.0 - wgs84Flattening);

    auto geodeticToEcef = [](double latDeg,
                             double lonDeg,
                             double altM,
                             double &xM,
                             double &yM,
                             double &zM) {
        double latRad = latDeg * DEG_TO_RAD;
        double lonRad = lonDeg * DEG_TO_RAD;
        double sinLat = sin(latRad);
        double cosLat = cos(latRad);
        double sinLon = sin(lonRad);
        double cosLon = cos(lonRad);
        double primeVerticalRadiusM =
            wgs84SemiMajorAxisM / sqrt(1.0 - wgs84EccentricitySquared * sinLat * sinLat);

        xM = (primeVerticalRadiusM + altM) * cosLat * cosLon;
        yM = (primeVerticalRadiusM + altM) * cosLat * sinLon;
        zM = (primeVerticalRadiusM * (1.0 - wgs84EccentricitySquared) + altM) * sinLat;
    };

    double originXM = 0.0;
    double originYM = 0.0;
    double originZM = 0.0;
    double currentXM = 0.0;
    double currentYM = 0.0;
    double currentZM = 0.0;

    geodeticToEcef(originLatitudeDeg,
                   originLongitudeDeg,
                   originAltitudeM,
                   originXM,
                   originYM,
                   originZM);
    geodeticToEcef(latitudeDeg,
                   longitudeDeg,
                   altitudeM,
                   currentXM,
                   currentYM,
                   currentZM);

    double dxM = currentXM - originXM;
    double dyM = currentYM - originYM;
    double dzM = currentZM - originZM;

    double originLatitudeRad = originLatitudeDeg * DEG_TO_RAD;
    double originLongitudeRad = originLongitudeDeg * DEG_TO_RAD;
    double sinLat = sin(originLatitudeRad);
    double cosLat = cos(originLatitudeRad);
    double sinLon = sin(originLongitudeRad);
    double cosLon = cos(originLongitudeRad);

    northM = (-sinLat * cosLon * dxM) +
             (-sinLat * sinLon * dyM) +
             (cosLat * dzM);
    eastM = (-sinLon * dxM) + (cosLon * dyM);
    downM = (-cosLat * cosLon * dxM) +
            (-cosLat * sinLon * dyM) +
            (-sinLat * dzM);
}

const char *fixQualityLabel(uint8_t fixQuality)
{
    switch (fixQuality) {
        case 0: return "no fix";
        case 1: return "gnss";
        case 2: return "dgps";
        case 4: return "rtk fixed";
        case 5: return "rtk float";
        default: return "unknown";
    }
}

const char *wifiStatusLabel(wl_status_t status)
{
    switch (status) {
        case WL_CONNECTED: return "connected";
        case WL_NO_SSID_AVAIL: return "ssid not found";
        case WL_CONNECT_FAILED: return "connect failed";
        case WL_CONNECTION_LOST: return "connection lost";
        case WL_DISCONNECTED: return "disconnected";
        case WL_IDLE_STATUS: return "idle";
        default: return "unknown";
    }
}

const char *wifiEncryptionLabel(wifi_auth_mode_t encryptionType)
{
    switch (encryptionType) {
        case WIFI_AUTH_OPEN: return "open";
        case WIFI_AUTH_WEP: return "wep";
        case WIFI_AUTH_WPA_PSK: return "wpa";
        case WIFI_AUTH_WPA2_PSK: return "wpa2";
        case WIFI_AUTH_WPA_WPA2_PSK: return "wpa/wpa2";
        case WIFI_AUTH_WPA2_ENTERPRISE: return "wpa2 enterprise";
        case WIFI_AUTH_WPA3_PSK: return "wpa3";
        case WIFI_AUTH_WPA2_WPA3_PSK: return "wpa2/wpa3";
        default: return "unknown";
    }
}

[[maybe_unused]] void printPacketHex(const std::vector<uint8_t> &bytes)
{
    Serial.print("RTK packet bytes:");
    for (uint8_t byte : bytes) {
        Serial.printf(" %02X", byte);
    }
    Serial.println();
}

}

void NTRIPConnector::setup()
{
	Serial.println("UM980 telemetry mode: receiving GNSS data only; corrections are handled externally.");
	connectUART();
}

void NTRIPConnector::update() {

	getNewData();

} 

void NTRIPConnector::getNewData() {
	readGNSSData();
	printTelemetryStatus();
	sendData();
}

void NTRIPConnector::connectWIFI()
{
	Serial.println("Connecting to WiFi...");
    printWiFiScan();
	WiFi.begin(m_ssid, m_password);
	esp_wifi_set_ps(WIFI_PS_NONE);
	WiFi.setAutoReconnect(true);
	WiFi.persistent(false);
};

void NTRIPConnector::printWiFiScan()
{
    Serial.println("WiFi scan: starting...");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(false, true);
    delay(3000);

    bool foundConfiguredSsid = false;

    for (int pass = 1; pass <= 3; pass++) {
        Serial.printf("WiFi scan: pass %d/3...\n", pass);

        int networkCount = WiFi.scanNetworks();
        Serial.printf("WiFi scan: pass %d found %d network(s)\n", pass, networkCount);

        for (int i = 0; i < networkCount; i++) {
            String ssid = WiFi.SSID(i);
            if (ssid == m_ssid) foundConfiguredSsid = true;

            Serial.printf("WiFi scan: %d.%d: ssid=\"%s\", rssi=%ld dBm, channel=%ld, encryption=%s%s\n",
                          pass,
                          i + 1,
                          ssid.c_str(),
                          WiFi.RSSI(i),
                          WiFi.channel(i),
                          wifiEncryptionLabel(static_cast<wifi_auth_mode_t>(WiFi.encryptionType(i))),
                          (ssid == m_ssid) ? " <-- configured SSID" : "");
        }

        WiFi.scanDelete();
        delay(500);
    }

    Serial.printf("WiFi scan: configured ssid=\"%s\" %s\n",
                  m_ssid,
                  foundConfiguredSsid ? "was found" : "was NOT found");
}

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
        wl_status_t wifiStatus = WiFi.status();

        if (now - m_lastWifiStatMs >= 1000) {
            Serial.printf("WiFi: status=%s (%d), ntrip=disconnected\n",
                          wifiStatusLabel(wifiStatus),
                          static_cast<int>(wifiStatus));
            m_lastWifiStatMs = now;
        }

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

        bool hasRecentRtcm = (m_lastRtcmRxMs != 0) && (ageMs < 2000);

        double speedMs = m_hasVelocity
            ? sqrt((m_velocityEastMs * m_velocityEastMs) +
                   (m_velocityNorthMs * m_velocityNorthMs) +
                   (m_velocityUpMs * m_velocityUpMs))
            : 0.0;

        Serial.printf("RTK: correction=%s, rtcm=%lu B/s, last=%lu ms, fix=%u (%s), vel=%s, speed=%.3f m/s, east=%.3f m/s, north=%.3f m/s, up=%.3f m/s\n",
                      hasRecentRtcm ? "yes" : "no",
                      (unsigned long)m_rtcmBytesSec,
                      (unsigned long)((ageMs == 0xFFFFFFFF) ? 0 : ageMs),
                      m_fixQuality,
                      fixQualityLabel(m_fixQuality),
                      m_hasVelocity ? "yes" : "no",
                      speedMs,
                      m_hasVelocity ? m_velocityEastMs : 0.0,
                      m_hasVelocity ? m_velocityNorthMs : 0.0,
                      m_hasVelocity ? m_velocityUpMs : 0.0);

        m_lastStatMs = now;
    }
}

void NTRIPConnector::connectUART()
{
	Serial.println("Connecting to UM980 Receiver via UART...");

	// Initialize the serial port for the GNSS receiver
	GNSSserial.setRxBufferSize(kGNSSRxBufferSize);
	GNSSserial.begin(115200, SERIAL_8N1, PinMap::NTRIP_RX, PinMap::NTRIP_TX);

	if (GNSSserial) {
		Serial.println("UM980 Receiver connected successfully.");
	} else {
		Serial.println("Failed to connect to UM980 Receiver.");
	}
}

void NTRIPConnector::requestGPGGA()
{
	unsigned long currentTime = millis();
	if (currentTime - m_lastGGAGottenMs >= m_GPGGADelta) {
		GNSSserial.println(F("GPGGA\r\n"));
		m_lastGGAGottenMs = currentTime;
	}
}

void NTRIPConnector::requestVTG()
{
	unsigned long currentTime = millis();
	if (currentTime - m_lastVTGGottenMs >= m_GNSSPollDelta) {
		GNSSserial.println(F("GNVTG\r\n"));
		m_lastVTGGottenMs = currentTime;
	}
}

void NTRIPConnector::readGNSSData()
{
	while (GNSSserial.available() > 0) {
		char c = GNSSserial.read();

		if (c == '\r') {
			continue;
		}
		else if (c == '\n') {
			lineBuf[linePos] = '\0';
			linePos = 0;
			parseNMEALine(lineBuf);
		}
		else if (linePos < sizeof(lineBuf) - 1) {
			lineBuf[linePos++] = c;
		}
		else {
			linePos = 0;
		}
	}
}

void NTRIPConnector::printTelemetryStatus()
{
	uint32_t now = millis();
	if (now - m_lastStatMs < 1000) return;

	Serial.printf("RTK raw: GGA=%s\n", m_lastRawGGA[0] ? m_lastRawGGA : "none");
	Serial.printf("RTK raw: VTG=%s\n", m_lastRawVTG[0] ? m_lastRawVTG : "none");
	Serial.printf("RTK parsed: VTG valid=%s, status=%s, age=%lu ms\n",
	              m_hasVelocity ? "yes" : "no",
	              m_lastVTGParseStatus,
	              m_lastVTGParseMs == 0 ? 0UL : static_cast<unsigned long>(now - m_lastVTGParseMs));
	Serial.printf("RTK raw: NTR=%s\n", m_lastRawNTR[0] ? m_lastRawNTR : "none");

	m_lastStatMs = now;
}

void NTRIPConnector::parseNMEALine(char *nmea)
{
	if (nmea[0] == '$' && strncmp(nmea + 3, "GGA,", 4) == 0) {
		snprintf(m_lastRawGGA, sizeof(m_lastRawGGA), "%s", nmea);
		parseGPGGA(nmea);
	}
	else if (nmea[0] == '$' && strncmp(nmea + 3, "VTG,", 4) == 0) {
		snprintf(m_lastRawVTG, sizeof(m_lastRawVTG), "%s", nmea);
		parseVTG(nmea);
	}
	else if (strncmp(nmea, "$GPNTR,", 7) == 0) {
		snprintf(m_lastRawNTR, sizeof(m_lastRawNTR), "%s", nmea);
		parseGPNTR(nmea);
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
    if (strncmp(nmea, "$GPNTR,", 7) != 0) return;
	snprintf(m_lastRawNTR, sizeof(m_lastRawNTR), "%s", nmea);
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
    if (!verifyNMEAChecksum(nmea)) return;
	char *star = strrchr(nmea, '*');
	if (star == nullptr) return;

    // Normalize GGA talker ID to GP for the old NTRIP path.
    if (nmea[0] == '$' && strncmp(nmea + 3, "GGA", 3) == 0) {
        nmea[1] = 'G';
        nmea[2] = 'P';
    }

    // Recompute checksum (in case we changed talker ID)
    uint8_t cs = 0;
    for (char *p = nmea + 1; p < star; ++p) cs ^= (uint8_t)*p;
    char csHex[3];
    snprintf(csHex, sizeof(csHex), "%02X", cs);

    // Store in lastGGA_ with CRLF
    *star = '\0';
    m_lastGPGGA = String(nmea) + "*" + csHex + "\r\n";

	const int MAXTOK = 15;
	char *tokens[MAXTOK];
	int tokCount = 0;
	tokens[tokCount++] = nmea;
	for (char *p = nmea; *p && tokCount < MAXTOK; p++) {
		if (*p == ',') {
			*p = '\0';
			tokens[tokCount++] = p + 1;
		}
	}

	if (tokCount < 10) return;

	uint32_t gnssTimeOfDayMs = 0;
	m_hasGNSSTime = parseGGAUtcTimeOfDayMs(tokens[1], gnssTimeOfDayMs);
	m_gnssTimeOfDayMs = m_hasGNSSTime ? gnssTimeOfDayMs : 0;

	double latRaw = strtod(tokens[2], nullptr);
	char latHemi = tokens[3][0];
	double lonRaw = strtod(tokens[4], nullptr);
	char lonHemi = tokens[5][0];
	int fixQ = atoi(tokens[6]);
	double altitude = strtod(tokens[9], nullptr);
	m_fixQuality = static_cast<uint8_t>(fixQ);

	if (fixQ <= 0 || latRaw == 0.0 || lonRaw == 0.0) return;

	m_latitudeDeg = nmeaPositionToDecimal(latRaw, latHemi);
	m_longitudeDeg = nmeaPositionToDecimal(lonRaw, lonHemi);
	m_altitudeM = altitude;

	if (!m_hasNedOrigin) {
		m_originLatitudeDeg = m_latitudeDeg;
		m_originLongitudeDeg = m_longitudeDeg;
		m_originAltitudeM = m_altitudeM;
		m_hasNedOrigin = true;
	}

	m_hasPosition = true;
}

void NTRIPConnector::parseVTG(char *nmea) {
	m_velocityEastMs = 0.0;
	m_velocityNorthMs = 0.0;
	m_velocityUpMs = 0.0;
	m_hasVelocity = false;
	m_lastVTGParseMs = millis();

	double velocityNorthMs = 0.0;
	double velocityEastMs = 0.0;
	double velocityUpMs = 0.0;
	const char *rejectReason = decodeVTGVelocity(nmea, velocityNorthMs, velocityEastMs, velocityUpMs);
	if (rejectReason != nullptr) {
		snprintf(m_lastVTGParseStatus, sizeof(m_lastVTGParseStatus), "%s", rejectReason);
		return;
	}

	m_velocityNorthMs = velocityNorthMs;
	m_velocityEastMs = velocityEastMs;
	m_velocityUpMs = velocityUpMs;
	m_hasVelocity = true;
	snprintf(m_lastVTGParseStatus, sizeof(m_lastVTGParseStatus), "ok");
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
	uint32_t now = millis();
	if (now - m_lastTelemetrySentMs < m_telemetryDelta) return;
	m_lastTelemetrySentMs = now;

	// if (!m_hasPosition) {
	// 	Serial.printf(
	// 		"RTK packet sending without valid position, fix=%u (%s), has_velocity=%s, "
	// 		"lat=%.8f deg, lon=%.8f deg, alt=%.3f m\n",
	// 		m_fixQuality,
	// 		fixQualityLabel(m_fixQuality),
	// 		m_hasVelocity ? "yes" : "no",
	// 		m_latitudeDeg,
	// 		m_longitudeDeg,
	// 		m_altitudeM);
	// }

	RTKTelemetryPacket telemetry;

	telemetry.header.type = static_cast<uint8_t>(120);
	telemetry.header.source = m_networkmanager.getAddress();
	telemetry.header.source_service = 1; // does this work if it isnt source service 1
	telemetry.header.destination = 2;
	telemetry.header.destination_service = 6;
	telemetry.header.uid = 1;

	double northM = 0.0;
	double eastM = 0.0;
	double downM = 0.0;
	if (m_hasNedOrigin) {
		geodeticToNed(m_latitudeDeg,
					  m_longitudeDeg,
					  m_altitudeM,
					  m_originLatitudeDeg,
					  m_originLongitudeDeg,
					  m_originAltitudeM,
					  northM,
					  eastM,
					  downM);
	}

	double packetVelocityNorthMs = 0.0;
	double packetVelocityEastMs = 0.0;
	double packetVelocityUpMs = 0.0;
	const char *packetVTGRejectReason = decodeVTGVelocity(m_lastRawVTG,
														  packetVelocityNorthMs,
														  packetVelocityEastMs,
														  packetVelocityUpMs);
	bool packetHasVelocity = packetVTGRejectReason == nullptr;
	if (packetHasVelocity) {
		m_velocityNorthMs = packetVelocityNorthMs;
		m_velocityEastMs = packetVelocityEastMs;
		m_velocityUpMs = packetVelocityUpMs;
		m_hasVelocity = true;
		snprintf(m_lastVTGParseStatus, sizeof(m_lastVTGParseStatus), "ok");
	}
	else {
		m_velocityNorthMs = 0.0;
		m_velocityEastMs = 0.0;
		m_velocityUpMs = 0.0;
		m_hasVelocity = false;
		if (m_lastRawVTG[0] != '\0') {
			snprintf(m_lastVTGParseStatus, sizeof(m_lastVTGParseStatus), "%s", packetVTGRejectReason);
		}
	}

	telemetry.x_input = static_cast<float>(northM);
	telemetry.y_input = static_cast<float>(eastM);
	telemetry.z_input = static_cast<float>(downM);
	telemetry.u_input = static_cast<float>(packetHasVelocity ? packetVelocityNorthMs : 0.0);
	telemetry.v_input = static_cast<float>(packetHasVelocity ? packetVelocityEastMs : 0.0);
	telemetry.w_input = static_cast<float>(packetHasVelocity ? -packetVelocityUpMs : 0.0);
	telemetry.fix_quality = m_fixQuality;
	telemetry.wifi_connected = (WiFi.status() == WL_CONNECTED) ? 1 : 0;
	telemetry.gnss_time_of_day_ms = m_hasGNSSTime ? m_gnssTimeOfDayMs : 0;

	m_networkmanager.sendPacket(telemetry);

	if (now - m_lastPacketDebugMs >= 1000) {
		std::vector<uint8_t> serializedTelemetry;
		telemetry.serialize(serializedTelemetry);

		Serial.printf(
			"RTK packet sent: node_addr=%u, start=0x%02X, type=%u, uid=%u, "
			"payload_len=%u B, serialized_len=%u B, src_addr=%u, src_service=%u, "
			"dst_addr=%u, dst_service=%u, hops=%u\n",
			m_networkmanager.getAddress(),
			telemetry.header.start_byte,
			telemetry.header.type,
			telemetry.header.uid,
			telemetry.header.packet_len,
			static_cast<unsigned int>(serializedTelemetry.size()),
			telemetry.header.source,
			telemetry.header.source_service,
			telemetry.header.destination,
			telemetry.header.destination_service,
			telemetry.header.hops);

		Serial.printf(
			"RTK packet fields: north=%.6f m, east=%.6f m, down=%.6f m, "
			"vel=%s, vtg_status=%s, north=%.6f m/s, east=%.6f m/s, down=%.6f m/s, "
			"fix=%u, wifi=%u, gnss_time_ms=%lu\n",
			telemetry.x_input,
			telemetry.y_input,
			telemetry.z_input,
			packetHasVelocity ? "yes" : "no",
			packetHasVelocity ? "ok" : packetVTGRejectReason,
			telemetry.u_input,
			telemetry.v_input,
			telemetry.w_input,
			telemetry.fix_quality,
			telemetry.wifi_connected,
			static_cast<unsigned long>(telemetry.gnss_time_of_day_ms));
		printPacketHex(serializedTelemetry);
		m_lastPacketDebugMs = now;
	}
}
