#include <Arduino.h>
#include <Wire.h>

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
  Serial.begin(9600);
  Serial.println("Enter command in the format 'va[voltage1]ca[current1]vb[voltage2]cb[current2]vc[voltage3]cc[current3]vd[voltage4]cd[current4]'.");
  Serial.println("Example: va128ca10vb128cb10vc128cc10vd128cd10");

  // Initialize I2C for AD5282
  WIRE.setPins(PIN_SDA,PIN_SCL);
  WIRE.begin(); // maybe set frequency? 100000
  
}

void loop() {
  if (Serial.available()) 
  {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove extra spaces or newline characters
    delay(100);
    // Parse commands for all four boards
    int vaIndex = input.indexOf("va");
    int caIndex = input.indexOf("ca");
    int vbIndex = input.indexOf("vb");
    int cbIndex = input.indexOf("cb");
    int vcIndex = input.indexOf("vc");
    int ccIndex = input.indexOf("cc");
    int vdIndex = input.indexOf("vd");
    int cdIndex = input.indexOf("cd");
    int ResetCom = input.indexOf("reset");

    //  New sections for controlling power state for each cell
    // int Astatus = input.indexOf("A");
    // int Bstatus = input.indexOf("B");
    // int Cstatus = input.indexOf("C");
    // int Dstatus = input.indexOf("D");

    // if (Astatus == 1)
    // {
    //   digitalWrite(Aenable,1);
    // }
    // if (Astatus == 0)
    // {
    //   digitalWrite(Aenable,0);
    // }
    
    if (vaIndex != -1 && caIndex != -1 && vbIndex != -1 && cbIndex != -1 && vcIndex != -1 && ccIndex != -1 && vdIndex != -1 && cdIndex != -1) 
    // This was within if statement, no idea why:   caIndex > vaIndex && vbIndex > caIndex && cbIndex > vbIndex && vcIndex > cbIndex && ccIndex > vcIndex && vdIndex > ccIndex && cdIndex > vdIndex
    {
      
      // Extract values for all four boards
      int voltage1 = input.substring(vaIndex + 2, caIndex).toInt();
      int current1 = input.substring(caIndex + 2, vbIndex).toInt();
      int voltage2 = input.substring(vbIndex + 2, cbIndex).toInt();
      int current2 = input.substring(cbIndex + 2, vcIndex).toInt();
      int voltage3 = input.substring(vcIndex + 2, ccIndex).toInt();
      int current3 = input.substring(ccIndex + 2, vdIndex).toInt();
      int voltage4 = input.substring(vdIndex + 2).toInt();
      int current4 = input.substring(cdIndex + 2).toInt();

      // Validate values for all four cells
      if (voltage1 >= 0 && voltage1 <= 255 && current1 >= 60 && current1 <= 255 &&
          voltage2 >= 0 && voltage2 <= 255 && current2 >= 60 && current2 <= 255 &&
          voltage3 >= 0 && voltage3 <= 255 && current3 >= 60 && current3 <= 255 &&
          voltage4 >= 0 && voltage4 <= 255 && current4 >= 60 && current4 <= 255) {
        
        Serial.println("-------------------------------------");
        
        // Cell 1
        Serial.println("Cell 1:");
        setVoltage(SLAVE_ADDRESS_1, voltage1);
        delay(100);
        readMuxVoltageBoard1(voltage1);

        setCurrent(SLAVE_ADDRESS_1, current1);
        delay(100);
        readMuxCurrentBoard1(current1);

        // Cell 2
        Serial.println("\nCell 2:");
        setVoltage(SLAVE_ADDRESS_2, voltage2);
        delay(100);
        readMuxVoltageBoard2(voltage2);

        setCurrent(SLAVE_ADDRESS_2, current2);
        delay(100);
        readMuxCurrentBoard2(current2);

                // Cell 3
        Serial.println("\nCell 3:");
        setVoltage(SLAVE_ADDRESS_3, voltage3);
        delay(100);
        readMuxVoltageBoard3(voltage3);

        setCurrent(SLAVE_ADDRESS_3, current3);
        delay(100);
        readMuxCurrentBoard3(current3);

        // Cell 4
        Serial.println("\nCell 4:");
        setVoltage(SLAVE_ADDRESS_4, voltage4);
        delay(100);
        readMuxVoltageBoard4(voltage4);

        setCurrent(SLAVE_ADDRESS_4, current4);
        delay(100);
        readMuxCurrentBoard4(current4);

        Serial.println("\nEnter next command:");
        Serial.println("-------------------------------------");
      } 
      else 
      {
        //  User input is outside range (limited)
        Serial.println("Invalid values. Voltage & current must be within range");
      }
    }
    //  else if for reset function, currently no reset function 
    else
    {
      Serial.println("Invalid command format. Use 'va[voltage1]ca[current1]vb[voltage2]cb[current2]vc[voltage3]cc[current3]vd[voltage4]cd[current4]' or ResetCom[Value].");
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
void readTemperatureBoard1() {
  digitalWrite(A3_PIN, LOW);
  digitalWrite(A2_PIN, LOW);
  digitalWrite(A1_PIN, LOW);
  digitalWrite(A0_PIN, HIGH);
  delay(100);

  int adcValue = analogRead(DOUT_PIN);
  float voltage = adcValue * (3.3 / 4095.0);
  float temp = 1.25 * ((voltage - V0C) / TC);

  Serial.print(temp, 2);
  Serial.println(" °C");
}

// Function to read temperature from Cell 2
void readTemperatureBoard2() {
  digitalWrite(A3_PIN, LOW);
  digitalWrite(A2_PIN, HIGH);
  digitalWrite(A1_PIN, LOW);
  digitalWrite(A0_PIN, HIGH);
  delay(100);

  int adcValue = analogRead(DOUT_PIN);
  float voltage = adcValue * (3.3 / 4095.0);
  float temp = 1.25 * ((voltage - V0C) / TC);

  Serial.print(temp, 2);
  Serial.println(" °C");
}

// Function to read temperature from Cell 3
void readTemperatureBoard3() {
  digitalWrite(A3_PIN, HIGH);
  digitalWrite(A2_PIN, HIGH);
  digitalWrite(A1_PIN, LOW);
  digitalWrite(A0_PIN, LOW);
  delay(100);

  int adcValue = analogRead(DOUT_PIN);
  float voltage = adcValue * (3.3 / 4095.0);
  float temp = 1.25 * ((voltage - V0C) / TC);

  Serial.print(temp, 2);
  Serial.println(" °C");
}

// Function to read temperature from Cell 4
void readTemperatureBoard4() {
  digitalWrite(A3_PIN, HIGH);
  digitalWrite(A2_PIN, LOW);
  digitalWrite(A1_PIN, LOW);
  digitalWrite(A0_PIN, HIGH);
  delay(100);

  int adcValue = analogRead(DOUT_PIN);
  float voltage = adcValue * (3.3 / 4095.0);
  float temp = 1.25 * ((voltage - V0C) / TC);

  Serial.print(temp, 2);
  Serial.println(" °C");
}

// Function to read and display voltage from Cell 1 MUX
void readMuxVoltageBoard1(int dPotStep) {
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
  float voltage = ((((averagedAdcValue * (3.3 / 4095)) * 2) / 0.65) - 0.05);

  Serial.print("dPot. step: ");
  Serial.print(dPotStep);
  Serial.print("   |  ");
  Serial.print(voltage, 2);
  Serial.println(" V");
}

// Function to read and display voltage from Cell 2 MUX
void readMuxVoltageBoard2(int dPotStep) {
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
  float voltage = ((((averagedAdcValue * (3.3 / 4095)) * 2) / 0.65) - 0.05);

  Serial.print("dPot. step: ");
  Serial.print(dPotStep);
  Serial.print("   |  ");
  Serial.print(voltage, 2);
  Serial.println(" V");
}

// Function to read and display voltage from Cell 3 MUX
void readMuxVoltageBoard3(int dPotStep) {
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
  float voltage = ((((averagedAdcValue * (3.3 / 4095)) * 2) / 0.65) - 0.05);

  Serial.print("dPot. step: ");
  Serial.print(dPotStep);
  Serial.print("   |  ");
  Serial.print(voltage, 2);
  Serial.println(" V");
}

// Function to read and display voltage from Cell 4 MUX
void readMuxVoltageBoard4(int dPotStep) {
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
  float voltage = ((((averagedAdcValue * (3.3 / 4095)) * 2) / 0.65) - 0.05);

  Serial.print("dPot. step: ");
  Serial.print(dPotStep);
  Serial.print("   |  ");
  Serial.print(voltage, 2);
  Serial.println(" V");
}

// Function to read and display current from Cell 1 MUX
void readMuxCurrentBoard1(int dPotStep) {
  digitalWrite(A3_PIN, LOW);
  digitalWrite(A2_PIN, LOW);
  digitalWrite(A1_PIN, HIGH);
  digitalWrite(A0_PIN, HIGH);
  delay(100);

  float currentSum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    int adcValue = analogRead(DOUT_PIN);
    float current = ((((adcValue * (3.3 / 4095)) * 2) / 3.3) / 0.5);
    currentSum += current;
  }
  float currentAverage = currentSum / SAMPLE_COUNT;

  Serial.print("dPot. step: ");
  Serial.print(dPotStep);
  Serial.print("    |  ");
  Serial.print(currentAverage, 2);
  Serial.print(" A  |   ");
  readTemperatureBoard1();
}

// Function to read and display current from Cell 2 MUX
void readMuxCurrentBoard2(int dPotStep) {
  digitalWrite(A3_PIN, LOW);
  digitalWrite(A2_PIN, LOW);
  digitalWrite(A1_PIN, LOW);
  digitalWrite(A0_PIN, LOW);
  delay(100);

  float currentSum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    int adcValue = analogRead(DOUT_PIN);
    float current = ((((adcValue * (3.3 / 4095)) * 2) / 3.3) / 0.5);
    currentSum += current;
  }
  float currentAverage = currentSum / SAMPLE_COUNT;

  Serial.print("dPot. step: ");
  Serial.print(dPotStep);
  Serial.print("    |  ");
  Serial.print(currentAverage, 2);
  Serial.print(" A  |   ");
  readTemperatureBoard2();
}

// Function to read and display current from Cell 3 MUX
void readMuxCurrentBoard3(int dPotStep) {
  digitalWrite(A3_PIN, LOW);
  digitalWrite(A2_PIN, HIGH);
  digitalWrite(A1_PIN, HIGH);
  digitalWrite(A0_PIN, LOW);
  delay(100);

  float currentSum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    int adcValue = analogRead(DOUT_PIN);
    float current = ((((adcValue * (3.3 / 4095)) * 2) / 3.3) / 0.5);
    currentSum += current;
  }
  float currentAverage = currentSum / SAMPLE_COUNT;

  Serial.print("dPot. step: ");
  Serial.print(dPotStep);
  Serial.print("    |  ");
  Serial.print(currentAverage, 2);
  Serial.print(" A  |   ");
  readTemperatureBoard3();
}

// Function to read and display current from Cell 4 MUX
void readMuxCurrentBoard4(int dPotStep) {
  digitalWrite(A3_PIN, HIGH);
  digitalWrite(A2_PIN, LOW);
  digitalWrite(A1_PIN, HIGH);
  digitalWrite(A0_PIN, HIGH);
  delay(100);

  float currentSum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    int adcValue = analogRead(DOUT_PIN);
    float current = ((((adcValue * (3.3 / 4095)) * 2) / 3.3) / 0.5);
    currentSum += current;
  }
  float currentAverage = currentSum / SAMPLE_COUNT;

  Serial.print("dPot. step: ");
  Serial.print(dPotStep);
  Serial.print("    |  ");
  Serial.print(currentAverage, 2);
  Serial.print(" A  |   ");
  readTemperatureBoard4();
}