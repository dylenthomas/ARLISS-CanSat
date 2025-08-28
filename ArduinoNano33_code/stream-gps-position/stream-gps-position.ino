#include "Neo6M_UBXParser.h"
#include "ArduinoBLE.h"

constexpr bool debugging = false;

// BLE Initalization ------------------------------------------------------------------------------
const char* uuidService = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* uuidNotify = "19b10001-e8f2-537e-4f6c-d104768a1214";
const char* uuidCommands = "19b10002-e8f2-537e-4f6c-d104768a1214";

const char* BLEname = "Nano33BLE";

BLEService nanoService(uuidService);
BLECharacteristic notifier(uuidNotify, BLENotify, 64);
BLEByteCharacteristic commands(uuidCommands, BLEWrite);
BLEDevice central;
// ------------------------------------------------------------------------------------------------

// GPS Initialization -----------------------------------------------------------------------------
GPSParser gps;
GPSParser::posllhData GPS_posllh = gps.getPOSLLH();
GPSParser::statusData GPS_status = gps.getSTATUS();
GPSParser::velnedData GPS_velned = gps.getVELNED();
// ------------------------------------------------------------------------------------------------

void log(String data) {
	Serial.println(data);
	notifier.writeValue(data.c_str(), data.length());
}

void log(const char* data) {
	log(String(data));
}

void setup() {
    Serial.begin(115200);
    while (!Serial && debugging);

    Serial1.begin(9600);

    if (!BLE.begin()) {
        Serial.println("Bluetooth failed to initialize.");
        while (1);
    }
    else {
        Serial.println("Bluetooth started");
    }

    /* configure bluetooth */
	BLE.setDeviceName(BLEname);
	BLE.setLocalName(BLEname);
	BLE.setAdvertisedService(nanoService);
	nanoService.addCharacteristic(notifier);
	nanoService.addCharacteristic(commands);
	BLE.addService(nanoService);

	/* initialize characteristics */
	notifier.writeValue((uint8_t)0);
	commands.writeValue(0);

	Serial.println("Started advertising bluetooth.");

	/* wait for central device to connect */
	while (!central) {
		central = BLE.central();
		BLE.advertise();
	}

	Serial.print("Connected to central: ");
	Serial.println(central.address());

	/* wait for central to connect to notifications */
	while (!notifier.subscribed()) {
		central = BLE.central();
	}

	log("<CONNECTED>");

	/* wait for a valid initial location from gps */
	log("Watiting for initial GPS position...");
	while (!gps.posllhChanged() || !GPS_status.gpsFixOk) {
		log("POSLLH Changed: " + String(gps.posllhChanged()) + ", gpsFixOk: " + String(GPS_status.gpsFixOk));
		while (Serial1.available()) {
			unsigned char c = Serial1.read();
			gps.addByte(c);
		}

		if (gps.statusChanged()) {
			GPS_status = gps.getSTATUS();
		}
	}

	log("Initial GPS position found.");
	delay(250);
	log("<READY>");
}

void loop() {
  while (Serial1.available()) {
		unsigned char c = Serial1.read();
		gps.addByte(c);
	}

  if (gps.posllhChanged()) {
    GPS_posllh = gps.getPOSLLH();
    log("Lat:" + String(GPS_posllh.lat, 5) + ",Lon:" + String(GPS_posllh.lon, 5) + ",h:" + String(GPS_posllh.height, 3));
  }
}
