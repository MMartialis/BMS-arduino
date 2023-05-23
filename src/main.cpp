#include <Arduino.h>

// setting the constants
const int8_t MAIN_SWITCH = 21;
const int8_t MUX_1 = 23;
const int8_t MUX_2 = 22;
const int8_t MUX_3 = 19;
const int8_t MUX_EN = 18;
const int8_t FAN_PWM = 16;
const int8_t VOLTAGE = 36;
const int8_t CURRENT = 35;
const int8_t NTC_1 = 34;
const int8_t NTC_2 = 39;
const int8_t EN[6] = {14, 27, 26, 25, 33, 32}; // [EN_1, EN_2, EN_3, EN_4, EN_5, EN_6]

// other constants
const int32_t MUX_TO_SETTLE = 50; // given in milliseconds
const int32_t FET_TO_SETTLE = 50; // given in milliseconds
const int32_t READ_VOLTAGE_DELAY = 100; // delay between reading two cells, given in milliseconds
const int32_t READ_CURRENT_DELAY = 100; // delay between current readings, given in milliseconds
const int32_t READ_TEMPERATURE_DELAY = 5000; // delay between temperature readings, given in milliseconds
const float VOLTAGE_MULTIPLIERS[6] = {0.001611722, 0.003223443, 0.004835165, 0.006446886, 0.008058608, 0.00967033}; // {2.0, 4.0, 6.0, 8.0, 10.0, 12.0}, *3.3/4095
const float VOLTAGE_TOLERANCE = 0.2; // given in volts
const float VOLTAGE_UNRECOVERABLE = 3; // given in volts, the voltage at which the battery is considered unrecoverable and the system shuts down
const float CURRENT_MULTIPLIER = 0.001611722; // given in amps // FIXME: this value is probably wrong
const float TEMPERATURE_MULTIPLIER = 0.001611722; // given in degrees celsius
const float CURRENT_SOFT_LIMIT = 30; // given in amps
const float CURRENT_HARD_LIMIT = 35; // given in amps
const float TEMPERATURE_SOFT_LIMIT_1 = 50; // given in degrees celsius
const float TEMPERATURE_SOFT_LIMIT_2 = 60; // given in degrees celsius
const float TEMPERATURE_HARD_LIMIT_1 = 60; // given in degrees celsius
const float TEMPERATURE_HARD_LIMIT_2 = 80; // given in degrees celsius
const float FAN_COEFFICIENT = 0.1; // given in percent per degree celsius

// turn on or off the console output for certain features
const bool VERBOSE_MUX = 1;
const bool VERBOSE_RAW_VOLTAGE = 1;
const bool VERBOSE_VOLTAGE = 1;
const bool VERBOSE_CURRENT = 0;
const bool VERBOSE_RAW_TEMPERATURE = 0;
const bool VERBOSE_TEMPERATURE = 0;
const bool VERBOSE_FAN = 0;

// global variables
bool systemEnabled = false;
uint64_t nextCurrentReadTime = 0; // declares when we should run the next current reading, millis()
uint64_t nextVoltageReadTime = 0; // declares when we should run the next voltage reading, millis()
uint64_t nextTemperatureReadTime = 0; // declares when we should run the next temperature reading, millis()
int8_t lastCellRead = 5;
float voltages[6] = {4.2, 4.2, 4.2, 4.2, 4.2, 4.2};

// setting pin types
void setupPins() {
  pinMode(MUX_1, OUTPUT);
  pinMode(MUX_2, OUTPUT);
  pinMode(MUX_3, OUTPUT);
  pinMode(MUX_EN, OUTPUT);
  pinMode(FAN_PWM, OUTPUT);
  pinMode(VOLTAGE, INPUT_PULLDOWN);
  pinMode(CURRENT, INPUT_PULLDOWN);
  pinMode(NTC_1, INPUT_PULLDOWN);
  pinMode(NTC_2, INPUT_PULLDOWN);
  pinMode(EN[0], OUTPUT);
  pinMode(EN[1], OUTPUT);
  pinMode(EN[2], OUTPUT);
  pinMode(EN[3], OUTPUT);
  pinMode(EN[4], OUTPUT);
  pinMode(EN[5], OUTPUT);
}

// task handles
TaskHandle_t currentDutyHandle;

// function for selecting the mux channel
void selectMuxChannel(int8_t channel) {
  digitalWrite(MUX_EN, LOW);
  // delay for the mux to settle
  delay(MUX_TO_SETTLE);
  // change mux channel pins to be able to read the correct battery cell
  digitalWrite(MUX_1, channel & 0x01);
  digitalWrite(MUX_2, channel & 0x02);
  digitalWrite(MUX_3, channel & 0x04);
  // delay for the mux to settle
  delay(MUX_TO_SETTLE);
  digitalWrite(MUX_EN, HIGH);
  delay(FET_TO_SETTLE);
}

// function for reading the voltage
int16_t readVoltage(int8_t channel, int8_t sample_number) {
  uint16_t voltage_buffer[sample_number] = {};
  selectMuxChannel(channel);
  nextVoltageReadTime = millis() + READ_VOLTAGE_DELAY; // set the next time to read the voltage
  for (int8_t i = 0; i < sample_number; i++) {
    voltage_buffer[i] = analogRead(VOLTAGE);
  }
  // calculate the average voltage
  uint32_t voltage_sum = 0;
  for (int8_t i = 0; i < sample_number; i++) {
    voltage_sum += voltage_buffer[i];
  }
  return voltage_sum / sample_number;
}

// function for reading the current
int16_t readCurrent() {
  nextCurrentReadTime = millis() + READ_CURRENT_DELAY;
  return analogRead(CURRENT);
}

// function for reading the temperature
int16_t readTemperature(int8_t ntc) {
  if (ntc == 1) {
    return analogRead(NTC_1);
  } else {
    return analogRead(NTC_2);
  }
}

// function shut down the system and shut off power
void systemShutdown(uint64_t fanTime /*in milliseconds*/){
  //turn off main switch
  digitalWrite(MAIN_SWITCH,LOW);
  //turn off balancing everywhere
  for (byte i = 0; i < 6; i++)
  {
    digitalWrite(EN[i],LOW);
  }
  //if cooling is needed, cool, otherwise turn off fan
  if (fanTime == 0){
    analogWrite(FAN_PWM,0);
  }
  else{
    analogWrite(FAN_PWM,255);
    uint64_t timeToShutOffFan = millis()+fanTime;
    while (millis()<timeToShutOffFan or systemEnabled){}
    analogWrite(FAN_PWM,0);
  }
}

// interpreting and acting on the cell voltage readings
void evaluateVoltages() {
  float minVoltage = 50;
  // translate the voltages to the correct values
  if (VERBOSE_RAW_VOLTAGE)
  {
    VERBOSE_RAW_VOLTAGE ? Serial.print("Raw voltages: ") : 0;
    for (int8_t i = 0; i < 6; i++)
    {
      Serial.print(voltages[i]);
      Serial.print("\t");
    }
  }
  for (int8_t i = 5; i > 0; i--)
  {
    voltages[i] = voltages[i] * VOLTAGE_MULTIPLIERS[i] - voltages[i-1] * VOLTAGE_MULTIPLIERS[i-1];
    minVoltage = minVoltage < voltages[i] ? minVoltage : voltages[i];
  }
  voltages[0] = voltages[0] * VOLTAGE_MULTIPLIERS[0];
  minVoltage = minVoltage < voltages[0] ? minVoltage : voltages[0];

  // shut down if any cell is below the minimum voltage
  if (minVoltage < VOLTAGE_UNRECOVERABLE)
  {
    systemShutdown(0);
  }

  VERBOSE_VOLTAGE ? Serial.print("Voltages: ") : 0;
  // turn on balancing for every cell that is above the minimum voltage plus the tolerance
  for (int8_t i = 0; i < 6; i++)
  {
    if (VERBOSE_VOLTAGE)
    {
      Serial.print(voltages[i]);
      Serial.print("\t");
    }
    if (voltages[i] - minVoltage > VOLTAGE_TOLERANCE)
    {
      digitalWrite(EN[i], HIGH);
    } else {
      digitalWrite(EN[i], LOW);
    }
  }
  VERBOSE_VOLTAGE || VERBOSE_RAW_VOLTAGE || VERBOSE_MUX ? Serial.print("\n") : 0;
}

// interpreting and acting on the current readings
void evaluateCurrent() {
  float current = readCurrent()*CURRENT_MULTIPLIER;
  if (VERBOSE_CURRENT)
  {
    Serial.print("Current: ");
    Serial.print(current);
    Serial.print("\t");
  }
  if (current > CURRENT_HARD_LIMIT)
  {
    systemShutdown(180000); //turn off everything and cool for 3 minutes
  }
}

// interpreting and acting on the temperature readings
void evaluateTemperature() {
  nextTemperatureReadTime = millis() + READ_TEMPERATURE_DELAY;
  float temperature1 = readTemperature(1)*TEMPERATURE_MULTIPLIER;
  float temperature2 = readTemperature(2)*TEMPERATURE_MULTIPLIER;
  
  if (VERBOSE_TEMPERATURE)
  {
    Serial.print("Temperature: ");
    Serial.print(temperature1);
    Serial.print("\t");
    Serial.print(temperature2);
    Serial.print("\t");
  }
  // checking hard limits
  if (temperature1 > TEMPERATURE_HARD_LIMIT_1 || temperature2 > TEMPERATURE_HARD_LIMIT_2)
  {
    systemShutdown(300000); //turn off everything and cool for 5 minutes
  }
  // controlling the fan
  float fanSpeed = 0;
  // linear function to control the fan speed based on tepmerature1 and temperature2
  int fanSpeedMagic = 0x3F800000 + (int) (temperature1) + (int) (temperature2);
  fanSpeed = *(float*) &fanSpeedMagic;


  if (VERBOSE_FAN)
  {
    Serial.print(fanSpeedMagic);
    Serial.print("\t");
    Serial.print(fanSpeed);
    Serial.print("\n");
  }

  // changing the fan speed
  analogWrite(FAN_PWM, fanSpeed); //FIXME: not gonna be in range

}

// create a task that can be maually assigned to a CPU core
void currentDuty(void * parameters) {
  while (!systemEnabled){} // if the system is disabled, enter infinite loop until it is reset
  for (;;){
    if (millis() >= nextCurrentReadTime){
      evaluateCurrent();
    }
  }
}

//*********************************************************************************************************************
void setup() {
  setupPins();
  Serial.begin(115200);

  xTaskCreatePinnedToCore(
    currentDuty, /* Function to implement the task */
    "currentDutyTask", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    1,  /* Priority of the task */
    &currentDutyHandle,  /* Task handle. */
    1); /* Core where the task should run */
  delay(500);
}

void loop(){
  while (!systemEnabled){} // if the system is disabled, enter infinite loop until it is reset

  if (millis() >= nextTemperatureReadTime){
    evaluateTemperature();
  }
  if (millis() >= nextVoltageReadTime){
    lastCellRead++;
    if (lastCellRead > 5) // if we have read all the cells, reset the counter and evaluate the voltages
    {
      lastCellRead = 0;
      VERBOSE_MUX && (VERBOSE_RAW_VOLTAGE || VERBOSE_VOLTAGE) ? Serial.print("\t") : 0;
      evaluateVoltages();
    }
    VERBOSE_MUX ? Serial.print(lastCellRead) : 0;
    voltages[lastCellRead] = readVoltage(lastCellRead, 10);
  }
}
