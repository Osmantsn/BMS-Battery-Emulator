#include <Arduino.h>
#include <Wire.h>

#include <HardwareSerial.h>

// -------------------- Nextion UART (ESP32 <-> Nextion) --------------------
// IMPORTANT: change these pins to match your wiring.
// Nextion TX -> ESP32 NEXTION_RX  (use level shifting/divider if Nextion is 5V UART)
// Nextion RX <- ESP32 NEXTION_TX
#define NEXTION_BAUD 9600
#define NEXTION_RX   18
#define NEXTION_TX   17

HardwareSerial Nex(2);

static inline void nexEnd() {
  Nex.write(0xFF); Nex.write(0xFF); Nex.write(0xFF);
}

// Update a Nextion Xfloat by scaling a float into an integer.
// Example: decimals=2, value=4.47 -> sends x4.val=447  (must set x4.vvs1=2 in Nextion editor)
static void nexSetXFloat(const char* obj, float value, int decimals) {
  int scale = 1;
  for (int i = 0; i < decimals; i++) scale *= 10;
  int scaled = (int)roundf(value * (float)scale);

  Nex.print(obj);
  Nex.print(".val=");
  Nex.print(scaled);
  nexEnd();
}

// AD5282 Definitions
#define WIRE Wire
#define SLAVE_ADDRESS_1 47 // Address for Cell Board 1  2F
#define SLAVE_ADDRESS_2 44 // Address for Cell Board 2  2C
#define SLAVE_ADDRESS_3 46 // Address for Cell Board 3  2E  Maybe switch?
#define SLAVE_ADDRESS_4 45 // Address for Cell Board 4  2D  Maybe switch?

#define PIN_SDA 48        // SDA connected to ESP32
#define PIN_SCL 47        // SCL connected to ESP32

//  Maybe impliment hard current limit?

// MUX Definitions
#define EN_PIN 45         // Enable pin on ESP32
#define A0_PIN 11         // Address pin A0 on ESP32
#define A1_PIN 12         // Address pin A1 on ESP32
#define A2_PIN 13         // Address pin A2 on ESP32
#define A3_PIN 10         // Address pin A3 on ESP32
#define DOUT_PIN 9        // Common OUT for MUX
#define SAMPLE_COUNT 128  // Bits for averaging

//  Cellboard Enable pins
#define Aenable 6         //  Change pin number
#define Benable 6         //  Change pin number
#define Cenable 6         //  Change pin number
#define Denable 6         //  Change pin number

// Constants for MCP9700 temperature sensor
const float V0C = 0.16;   // Voltage at 0°C in volts
const float TC = 0.01;    // Temperature coefficient in volts/°C (10 mV/°C)

void setup() {
  // Initialize MUX pins
  pinMode(EN_PIN, OUTPUT);
  pinMode(A0_PIN, OUTPUT);
  pinMode(A1_PIN, OUTPUT);
  pinMode(A2_PIN, OUTPUT);
  pinMode(A3_PIN, OUTPUT);
  pinMode(DOUT_PIN, INPUT);

  // Enable the multiplexer
  digitalWrite(EN_PIN, HIGH); 
  
  //  Begin communication over UART
  //  USB Serial monitor (debug). Set your IDE monitor to 115200.
  Serial.begin(115200);
  Nex.begin(NEXTION_BAUD, SERIAL_8N1, NEXTION_RX, NEXTION_TX);
  Serial.println("Enter command format:");
  Serial.println("  Minimum: va[0..255]ca[60..255]");
  Serial.println("  Optional: vb[0..255]cb[60..255] vc[0..255]cc[60..255] vd[0..255]cd[60..255]");
  Serial.println("Example (A only): va128ca255");
  Serial.println("Example (all):    va128ca255vb0cb60vc0cc60vd0cd60");

  // Initialize I2C for AD5282
  WIRE.setPins(PIN_SDA,PIN_SCL);
  WIRE.begin(); // maybe set frequency? 100000
  
}


static void handleCommand(String input) {
  input.trim();
  if (input.length() == 0) return;

  // Parse commands for all four boards
  int vaIndex = input.indexOf("va");
  int caIndex = input.indexOf("ca");
  int vbIndex = input.indexOf("vb");
  int cbIndex = input.indexOf("cb");
  int vcIndex = input.indexOf("vc");
  int ccIndex = input.indexOf("cc");
  int vdIndex = input.indexOf("vd");
  int cdIndex = input.indexOf("cd");

  // Accept either:
  //  - Full format: va..ca..vb..cb..vc..cc..vd..cd..
  //  - Channel A only: va..ca..
  // Other channels are optional, but must come in voltage/current pairs (vb+cb, vc+cc, vd+cd).
  if (vaIndex == -1 || caIndex == -1) {
    Serial.println("Invalid command format. Must include at least: va[voltage1]ca[current1]");
    Serial.println("Optional: vb[voltage2]cb[current2]vc[voltage3]cc[current3]vd[voltage4]cd[current4]");
    return;
  }
  if ((vbIndex != -1) != (cbIndex != -1)) {
    Serial.println("Invalid command format. If you use vb you must also use cb (and vice versa).");
    return;
  }
  if ((vcIndex != -1) != (ccIndex != -1)) {
    Serial.println("Invalid command format. If you use vc you must also use cc (and vice versa).");
    return;
  }
  if ((vdIndex != -1) != (cdIndex != -1)) {
    Serial.println("Invalid command format. If you use vd you must also use cd (and vice versa).");
    return;
  }

  // Defaults for channels not provided
  int voltage1 = 0, current1 = 60;
  int voltage2 = 0, current2 = 60;
  int voltage3 = 0, current3 = 60;
  int voltage4 = 0, current4 = 60;

  // Extract channel A
  voltage1 = input.substring(vaIndex + 2, caIndex).toInt();
  int aEnd = (vbIndex != -1) ? vbIndex : input.length();
  current1 = input.substring(caIndex + 2, aEnd).toInt();

  // Extract channel B (optional)
  if (vbIndex != -1) {
    voltage2 = input.substring(vbIndex + 2, cbIndex).toInt();
    int bEnd = (vcIndex != -1) ? vcIndex : input.length();
    current2 = input.substring(cbIndex + 2, bEnd).toInt();
  }

  // Extract channel C (optional)
  if (vcIndex != -1) {
    voltage3 = input.substring(vcIndex + 2, ccIndex).toInt();
    int cEnd = (vdIndex != -1) ? vdIndex : input.length();
    current3 = input.substring(ccIndex + 2, cEnd).toInt();
  }

  // Extract channel D (optional)
  if (vdIndex != -1) {
    voltage4 = input.substring(vdIndex + 2, cdIndex).toInt();
    current4 = input.substring(cdIndex + 2).toInt();
  }

  // Validate ranges
  if (!(voltage1 >= 0 && voltage1 <= 255 && current1 >= 60 && current1 <= 255 &&
        voltage2 >= 0 && voltage2 <= 255 && current2 >= 60 && current2 <= 255 &&
        voltage3 >= 0 && voltage3 <= 255 && current3 >= 60 && current3 <= 255 &&
        voltage4 >= 0 && voltage4 <= 255 && current4 >= 60 && current4 <= 255)) {
    Serial.println("Invalid values. Voltage must be 0..255 and current must be 60..255.");
    return;
  }

  Serial.println("-------------------------------------");

  // ---------------- Cell 1 ----------------
  Serial.println("Cell 1:");
  setVoltage(SLAVE_ADDRESS_1, voltage1);
  delay(100);
  float v1 = readMuxVoltageBoard1(voltage1);

  setCurrent(SLAVE_ADDRESS_1, current1);
  delay(100);
  float t1 = 0;
  float i1 = readMuxCurrentBoard1(current1, t1);

  // Push Cell 1 measurements to Nextion fields (ChannelA page)
  // Nextion Xfloat scaling:
  //  - set x4.vvs1=2 for voltage (V)
  //  - set x3.vvs1=2 for current (A)
  //  - set x2.vvs1=2 (or 1) for temperature (°C)
  nexSetXFloat("x4", v1, 2);  // Voltage
  nexSetXFloat("x3", i1, 2);  // Current
  nexSetXFloat("x2", t1, 2);  // Temp

  // ---------------- Cell 2 ----------------
  Serial.println("\nCell 2:");
  setVoltage(SLAVE_ADDRESS_2, voltage2);
  delay(100);
  readMuxVoltageBoard2(voltage2);

  setCurrent(SLAVE_ADDRESS_2, current2);
  delay(100);
  float t2 = 0;
  readMuxCurrentBoard2(current2, t2);

  // ---------------- Cell 3 ----------------
  Serial.println("\nCell 3:");
  setVoltage(SLAVE_ADDRESS_3, voltage3);
  delay(100);
  readMuxVoltageBoard3(voltage3);

  setCurrent(SLAVE_ADDRESS_3, current3);
  delay(100);
  float t3 = 0;
  readMuxCurrentBoard3(current3, t3);

  // ---------------- Cell 4 ----------------
  Serial.println("\nCell 4:");
  setVoltage(SLAVE_ADDRESS_4, voltage4);
  delay(100);
  readMuxVoltageBoard4(voltage4);

  setCurrent(SLAVE_ADDRESS_4, current4);
  delay(100);
  float t4 = 0;
  readMuxCurrentBoard4(current4, t4);

  Serial.println("\nEnter next command:");
  Serial.println("-------------------------------------");
}

void loop() {
  // USB Serial (PC)
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    handleCommand(input);
  }

  // Nextion UART (robust line receiver)
  // Nextion touch events / "Send Component ID" generate binary packets that are NOT newline-terminated.
  // We only start capturing when we see a 'v' (command begins with "va...") and we complete on '\n'.
  static String nexLine;
  static bool capturing = false;
  while (Nex.available()) {
    uint8_t b = (uint8_t)Nex.read();

    if (!capturing) {
      if (b == 'v') {
        capturing = true;
        nexLine = "v";
      }
      continue;
    }

    // capturing
    if (b == '\r') continue;
    if (b == '\n') {
  capturing = false;

  Serial.print("Nextion RX: ");
  Serial.println(nexLine);

  handleCommand(nexLine);
  nexLine = "";
  continue;
}


    // Keep only printable ASCII to avoid stray binary bytes
    if (b >= 32 && b <= 126) {
      // basic size guard
      if (nexLine.length() < 120) {
        nexLine += (char)b;
      } else {
        // too long -> reset
        capturing = false;
        nexLine = "";
      }
    }
  }
}


// Function to set Voltage
void setVoltage(uint8_t slaveAddress, uint8_t value) {
  //  Frame 1
  WIRE.beginTransmission(slaveAddress);
  //  Frame 2
  WIRE.write(0); // Choose Wiper 1 (Voltage control)
  //  Frame 3
  WIRE.write(value);
  WIRE.endTransmission();
}

// Function to set Current
void setCurrent(uint8_t slaveAddress, uint8_t value) {
  WIRE.beginTransmission(slaveAddress);
  WIRE.write(128); // Choose Wiper 2 (Current control)
  WIRE.write(value);
  WIRE.endTransmission();
}

//  Mux functions are hard coded. which isnt pretty!
// Function to read temperature from Cell 1
float readTemperatureBoard1() {
  digitalWrite(A3_PIN, LOW);
  digitalWrite(A2_PIN, LOW);
  digitalWrite(A1_PIN, LOW);
  digitalWrite(A0_PIN, HIGH);
  delay(100);

  int adcValue = analogRead(DOUT_PIN);
  float voltage = adcValue * (3.3f / 4095.0f);
  float temp = 1.25f * ((voltage - V0C) / TC);

  Serial.print(temp, 2);
  Serial.println(" °C");
  return temp;
}


// Function to read temperature from Cell 2
float readTemperatureBoard2() {
  digitalWrite(A3_PIN, LOW);
  digitalWrite(A2_PIN, HIGH);
  digitalWrite(A1_PIN, LOW);
  digitalWrite(A0_PIN, HIGH);
  delay(100);

  int adcValue = analogRead(DOUT_PIN);
  float voltage = adcValue * (3.3f / 4095.0f);
  float temp = 1.25f * ((voltage - V0C) / TC);

  Serial.print(temp, 2);
  Serial.println(" °C");
  return temp;
}


// Function to read temperature from Cell 3
float readTemperatureBoard3() {
  digitalWrite(A3_PIN, HIGH);
  digitalWrite(A2_PIN, HIGH);
  digitalWrite(A1_PIN, LOW);
  digitalWrite(A0_PIN, LOW);
  delay(100);

  int adcValue = analogRead(DOUT_PIN);
  float voltage = adcValue * (3.3f / 4095.0f);
  float temp = 1.25f * ((voltage - V0C) / TC);

  Serial.print(temp, 2);
  Serial.println(" °C");
  return temp;
}


// Function to read temperature from Cell 4
float readTemperatureBoard4() {
  digitalWrite(A3_PIN, HIGH);
  digitalWrite(A2_PIN, LOW);
  digitalWrite(A1_PIN, LOW);
  digitalWrite(A0_PIN, HIGH);
  delay(100);

  int adcValue = analogRead(DOUT_PIN);
  float voltage = adcValue * (3.3f / 4095.0f);
  float temp = 1.25f * ((voltage - V0C) / TC);

  Serial.print(temp, 2);
  Serial.println(" °C");
  return temp;
}


// Function to read and display voltage from Cell 1 MUX
float readMuxVoltageBoard1(int dPotStep) {
  digitalWrite(A3_PIN, LOW);
  digitalWrite(A2_PIN, LOW);
  digitalWrite(A1_PIN, HIGH);
  digitalWrite(A0_PIN, LOW);
  delay(100);

  long totalAdcValue = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    totalAdcValue += analogRead(DOUT_PIN);
    delay(1);
  }
  int averagedAdcValue = totalAdcValue / SAMPLE_COUNT;
  float voltage = ((((averagedAdcValue * (3.3f / 4095.0f)) * 2.0f) / 0.65f) - 0.05f);

  Serial.print("dPot. step: ");
  Serial.print(dPotStep);
  Serial.print("   |  ");
  Serial.print(voltage, 2);
  Serial.println(" V");

  return voltage;
}


// Function to read and display voltage from Cell 2 MUX
float readMuxVoltageBoard2(int dPotStep) {
  digitalWrite(A3_PIN, LOW);
  digitalWrite(A2_PIN, HIGH);
  digitalWrite(A1_PIN, LOW);
  digitalWrite(A0_PIN, LOW);
  delay(100);

  long totalAdcValue = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    totalAdcValue += analogRead(DOUT_PIN);
    delay(1);
  }
  int averagedAdcValue = totalAdcValue / SAMPLE_COUNT;
  float voltage = ((((averagedAdcValue * (3.3f / 4095.0f)) * 2.0f) / 0.65f) - 0.05f);

  Serial.print("dPot. step: ");
  Serial.print(dPotStep);
  Serial.print("   |  ");
  Serial.print(voltage, 2);
  Serial.println(" V");

  return voltage;
}


// Function to read and display voltage from Cell 3 MUX
float readMuxVoltageBoard3(int dPotStep) {
  digitalWrite(A3_PIN, LOW);
  digitalWrite(A2_PIN, HIGH);
  digitalWrite(A1_PIN, HIGH);
  digitalWrite(A0_PIN, HIGH);
  delay(100);

  long totalAdcValue = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    totalAdcValue += analogRead(DOUT_PIN);
    delay(1);
  }
  int averagedAdcValue = totalAdcValue / SAMPLE_COUNT;
  float voltage = ((((averagedAdcValue * (3.3f / 4095.0f)) * 2.0f) / 0.65f) - 0.05f);

  Serial.print("dPot. step: ");
  Serial.print(dPotStep);
  Serial.print("   |  ");
  Serial.print(voltage, 2);
  Serial.println(" V");

  return voltage;
}


// Function to read and display voltage from Cell 4 MUX
float readMuxVoltageBoard4(int dPotStep) {
  digitalWrite(A3_PIN, HIGH);
  digitalWrite(A2_PIN, LOW);
  digitalWrite(A1_PIN, HIGH);
  digitalWrite(A0_PIN, LOW);
  delay(100);

  long totalAdcValue = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    totalAdcValue += analogRead(DOUT_PIN);
    delay(1);
  }
  int averagedAdcValue = totalAdcValue / SAMPLE_COUNT;
  float voltage = ((((averagedAdcValue * (3.3f / 4095.0f)) * 2.0f) / 0.65f) - 0.05f);

  Serial.print("dPot. step: ");
  Serial.print(dPotStep);
  Serial.print("   |  ");
  Serial.print(voltage, 2);
  Serial.println(" V");

  return voltage;
}


// Function to read and display current from Cell 1 MUX
float readMuxCurrentBoard1(int dPotStep, float &tempOut) {
  digitalWrite(A3_PIN, LOW);
  digitalWrite(A2_PIN, LOW);
  digitalWrite(A1_PIN, HIGH);
  digitalWrite(A0_PIN, HIGH);
  delay(100);

  float currentSum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    int adcValue = analogRead(DOUT_PIN);
    float current = ((((adcValue * (3.3f / 4095.0f)) * 2.0f) / 3.3f) / 0.5f);
    currentSum += current;
  }
  float currentAverage = currentSum / SAMPLE_COUNT;

  Serial.print("dPot. step: ");
  Serial.print(dPotStep);
  Serial.print("    |  ");
  Serial.print(currentAverage, 2);
  Serial.print(" A  |   ");

  tempOut = readTemperatureBoard1();
  return currentAverage;
}


// Function to read and display current from Cell 2 MUX
float readMuxCurrentBoard2(int dPotStep, float &tempOut) {
  digitalWrite(A3_PIN, LOW);
  digitalWrite(A2_PIN, LOW);
  digitalWrite(A1_PIN, LOW);
  digitalWrite(A0_PIN, LOW);
  delay(100);

  float currentSum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    int adcValue = analogRead(DOUT_PIN);
    float current = ((((adcValue * (3.3f / 4095.0f)) * 2.0f) / 3.3f) / 0.5f);
    currentSum += current;
  }
  float currentAverage = currentSum / SAMPLE_COUNT;

  Serial.print("dPot. step: ");
  Serial.print(dPotStep);
  Serial.print("    |  ");
  Serial.print(currentAverage, 2);
  Serial.print(" A  |   ");

  tempOut = readTemperatureBoard2();
  return currentAverage;
}


// Function to read and display current from Cell 3 MUX
float readMuxCurrentBoard3(int dPotStep, float &tempOut) {
  digitalWrite(A3_PIN, LOW);
  digitalWrite(A2_PIN, HIGH);
  digitalWrite(A1_PIN, HIGH);
  digitalWrite(A0_PIN, LOW);
  delay(100);

  float currentSum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    int adcValue = analogRead(DOUT_PIN);
    float current = ((((adcValue * (3.3f / 4095.0f)) * 2.0f) / 3.3f) / 0.5f);
    currentSum += current;
  }
  float currentAverage = currentSum / SAMPLE_COUNT;

  Serial.print("dPot. step: ");
  Serial.print(dPotStep);
  Serial.print("    |  ");
  Serial.print(currentAverage, 2);
  Serial.print(" A  |   ");

  tempOut = readTemperatureBoard3();
  return currentAverage;
}


// Function to read and display current from Cell 4 MUX
float readMuxCurrentBoard4(int dPotStep, float &tempOut) {
  digitalWrite(A3_PIN, HIGH);
  digitalWrite(A2_PIN, LOW);
  digitalWrite(A1_PIN, HIGH);
  digitalWrite(A0_PIN, HIGH);
  delay(100);

  float currentSum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    int adcValue = analogRead(DOUT_PIN);
    float current = ((((adcValue * (3.3f / 4095.0f)) * 2.0f) / 3.3f) / 0.5f);
    currentSum += current;
  }
  float currentAverage = currentSum / SAMPLE_COUNT;

  Serial.print("dPot. step: ");
  Serial.print(dPotStep);
  Serial.print("    |  ");
  Serial.print(currentAverage, 2);
  Serial.print(" A  |   ");

  tempOut = readTemperatureBoard4();
  return currentAverage;
}
