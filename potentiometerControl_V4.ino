
// Wiring
// Connect ADC_rotation_value potentiometer to both A0 and A1
// Connect ADC_multiplier_value potentiometer to A5


#include <util/delay.h>
#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
#define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
#define DEBUG_SERIAL SerialUSB
#else
#define DEBUG_SERIAL Serial
#endif

uint8_t DXL_ID = 0;
const float DXL_PROTOCOL_VERSION = 2.0;
float ADC_rotation_value = 0;
float ADC_multiplier_value = 0;
float multiplier = 4;
float previous_ADC_rotation_value = 0;
float angle = 0;

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:

  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  _delay_ms(100);

  DEBUG_SERIAL.print("PROTOCOL ");
  DEBUG_SERIAL.print(DXL_PROTOCOL_VERSION, 1);

  // Loop through the ID and find the motor
  for (DXL_ID = 0; DXL_ID < 256; DXL_ID++) {
    if (dxl.ping(DXL_ID) == true) {
      DEBUG_SERIAL.print(", ID ");
      DEBUG_SERIAL.print(DXL_ID);
      DEBUG_SERIAL.print(": ");
      DEBUG_SERIAL.print("ping succeeded!");
      DEBUG_SERIAL.print(", Model Number: ");
      DEBUG_SERIAL.println(dxl.getModelNumber(DXL_ID));
      break;
    }
    // Enter an endless while loop if the no valid motor ID is found
    if (DXL_ID == 255) {
      DEBUG_SERIAL.println(", ping failed! Quiting program...");
      exit(0);
    }
  }

  // Current Based Postion Contorl Mode in protocol2.0 (except MX28, XL430)
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT_BASED_POSITION);
  dxl.torqueOn(DXL_ID);
  dxl.setGoalCurrent(DXL_ID, 80, UNIT_PERCENT);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A5, INPUT);
  
  multiplier -= 1; // Adjustment for multiplier
}

void loop() {

  // Get ADC_rotation_value and averages it to remove noise
  ADC_rotation_value = 0;
  for (int i = 0; i < 40; i++) {
    ADC_rotation_value += ((float)analogRead(A0) + (float)analogRead(A1)) / 80;
  }

  // Get ADC_multiplier_value and averages it to remove noise
  ADC_multiplier_value = 0;
  for (int i = 0; i < 40; i++) {
    ADC_multiplier_value += ((float)analogRead(A5) / 1023 * multiplier + 1) / 40;
  }

  // Don't move the motor if previous ADC read value is very close to the present value, this is done to reduce twitching
  if (abs(ADC_rotation_value - previous_ADC_rotation_value) >= 2) {
    previous_ADC_rotation_value = ADC_rotation_value;
    angle = (float)ADC_rotation_value / 1023 * 4096 * ADC_multiplier_value; // 4096 encoder count is equivalent to one rotation
  }

  // Set motor posiiton
  if (dxl.setGoalPosition(DXL_ID, angle)) {
    // to do?
  }

}
