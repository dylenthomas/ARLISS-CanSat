#include "Motor_PWM_Nano33.h"
#include "ArduinoBLE.h"

#define DEBUGGING false

constexpr float data_samplerate = 10; // Hz
constexpr int current_sensor_pin = A1;
constexpr float current_sensitivity = 66.0/1000.0; // V/A

constexpr int mR_reverse_pin = 3;
constexpr int mR_forward_pin = 9;
constexpr int mL_reverse_pin = 6;
constexpr int mL_forward_pin = 5;
constexpr int PWMFreq = 25000; // Hz

Motor_Control motorRight(mR_forward_pin, mR_reverse_pin);
Motor_Control motorLeft(mL_forward_pin, mL_reverse_pin);

const char* uuidService = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* uuidNotify = "19b10001-e8f2-537e-4f6c-d104768a1214";
const char* uuidCommands = "19b10002-e8f2-537e-4f6c-d104768a1214";

const char* BLEname = "Nano33BLE";

BLEService nanoService(uuidService);
BLECharacteristic notifier(uuidNotify, BLENotify, 64);
BLEByteCharacteristic commands(uuidCommands, BLEWrite);
BLEDevice central;

void log(String data) {
	Serial.println(data);
	notifier.writeValue(data.c_str(), data.length());
}

void log(const char* data) {
	log(String(data));
}

bool valid_motor_command(String command, float* u_m1, float* u_m2) {
  /* FORMAT:
    x.x_y.y

    where-
      x.x is a float between -1 and 1 specifying duty cycle and direction of the first motor
      y.y is a float between -1 and 1 specifying duty cycle and direction of the second motor
  */

  int underscore_ind;
  int x = 0;

  for (int i = 0; i < command.length(); i++) {
    char c = command.charAt(i);
    if (c == '_') {
      underscore_ind = i;
      x++;
    }
  }

  if (x != 1) {
    return false;
  }

  float motorOne = command.substring(0, underscore_ind).toFloat();
  float motorTwo = command.substring(underscore_ind + 1).toFloat();

  if (motorOne < -1.0 || motorOne > 1.0) {
    return false;
  }
  if (motorTwo < -1.0 || motorTwo > 1.0) {
    return false;
  }

  *u_m1 = motorOne;
  *u_m2 = motorTwo;
  return true;
}

float read_current() {
  float sensor_val = analogRead(current_sensor_pin);
  float offset = 3.3 / 2.0; // voltage at zero amps

  return (offset - (sensor_val * (3.3 / 1024.0))) / current_sensitivity;
}

void setup() {
  Serial.begin(115200);
  while(!Serial && DEBUGGING);

  if (!BLE.begin()) {
    Serial.println("Bluetooth failed to initalize");
    while(1);
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
}


void loop() {
  central = BLE.central();

  String ble_string = "";
  bool ble_terminated = false;
  bool commandRecieved = false;

  unsigned long lastprint = millis();

  float u_mr;
  float u_ml;
  int right_dir;
  int left_dir;

  while (central.connected()) {
    if (commands.written()) {
      char c = commands.value();

      if (c == '\0') {
        ble_terminated = true;
      }
      else {
        ble_string += c;
      }
    }

    if (ble_terminated) {
      if (ble_string == "Start data collection") {
        while (true) {
          if (millis() - lastprint >= 1/data_samplerate * 1000) {
            log(String(millis()) + "," + String(read_current()));
            lastprint = millis();
          }
        }
      }
      else if (valid_motor_command(ble_string, &u_mr, &u_ml)) {
        if (u_mr < 0.0) {
          right_dir = REVERSE;
        }
        else {
          right_dir = FORWARD;
        }

        if (u_ml < 0.0) {
          left_dir = REVERSE;
        }
        else {
          left_dir = FORWARD;
        }

        motorRight.setFreqAndDuty(right_dir, PWMFreq, abs(u_mr));
        motorLeft.setFreqAndDuty(left_dir, PWMFreq, abs(u_ml));
      }
      else {
        log("Invalid command.");
      }
    }
  }
}
