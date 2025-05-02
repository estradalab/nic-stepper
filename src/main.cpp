/// Needle-induced Cavitation Stepper Firmware by Joseph Beckett (Experimental Soft Mechanics Laboratory, University of Michigan)
///
/// Designed for use with ESP32-S3-DevKitC-1 v1.1 and BTT TMC2209 V1.3 stepper driver

#include <Arduino.h>
#include <cmath>
#include <vector>
#include <stdint.h>
#include "esp_heap_caps.h"

#include <TMC2209.h>
#include <FastAccelStepper.h>
#include <Adafruit_NeoPixel.h>

// TODO - allow more parameters to be changed via serial command, and print off all adjustable parameters when anything is changed
// TODO - make a single place for all pre-allocations (globals.h/.cpp)
// TODO make a formal header file for all of these supporting functions
// TODO - MAYBE ask TMC 2209 how many steps it took and compare it with how many stepper thought it sent
// TODO - determine max rate that TMC 2209 can take in pulses and if it is microstepping dependent 
// TODO - make a config.cpp/.h file and a config_example.cpp/.h file for all of the settings that are not going to be changed in the code
// TODO - naming conventions CONSTANT_VARIABLE, normalVariable, 

//~// Software Inputs //~//
/// Mode conditions
String MODE_CONTROL_END = "stretch"; // "volume", "stretch", "time" are options for setting injection end conditions
bool MODE_CRE = true; // [true] (false) --> Injection run at a constant rate of [radial expansion] (volumetric flow)
// TODO - make MODE_CRE more generalizable (e.g.,"cve", "cre", "sine", "v_sweep") -- see if this is even necessary
bool MODE_USE_REV_S_FOR_CVE = true; // [true] (false) --> [REV_PER_S] (NL_PER_S) used to calculate step rate for CVE
uint8_t MODE_INJECT = 1; // 1 --> normal injection (default), 2--> mini injection
// TODO - see if MODE_INJECT is still even necessary and clean-up related variables
/// End conditions
float _T_INJECT = 15, _T_INJECT_0 = 0; // s (used in time-control mode)
float _V_INJECT = 40000, _V_INJECT_0 = 0; // nL (used in volume-control mode)
float _STRETCH_INJECT = 10, _STRETCH_INJECT_0 = 0; // mm/mm (used in stretch-control mode)
/// Rates
float REV_PER_S = 0.2; // rev/s for CVE (if set too high w/o acceleration control motor may misfunction)
// TODO - print off an estimated strain rate
float NL_PER_S = 5.0; // nL/s for CVE (if set too high w/o acceleration control motor may misfunction)
float RDOT = 0.1;// mm/s for CRE

//~// Hardware Inputs //~//
const float A_SYRINGE = PI*pow(1.03f/2.f,2); // mm^2
const float ID_NEEDLE = 0.337f; // mm 
const uint16_t MICROSTEPPING = 256; // µsteps/step (can be within 2^x, x ∈ 0:8)
// TODO - allow MICROSTEPPING to be adjusted for very high-rate testing (this can even be precalculated just looking at final rate)
const uint16_t STEPS_PER_REV = 400; // steps/rev (17HM19-1684D; 0.9°/step)
const float PITCH = 5.f; // mm/rev 
const uint16_t RMS_MOTOR_CURRENT = 2000;//1650; // mA
  // BTT TMC 2209 V1.3 limited at 2A RMS - do not exceed this
  // 17HM19-1684D has max phase of 1.68A RMS
const float RSENSE = 0.11; // Ω (BTT TMC 2209 V1.3; two R110 visible on board)
#define STEP_PIN 1
#define DIR_PIN 2
#define ENA_PIN 42
#define RX1_PIN 15 // warning: these are the correct pins, but this must be 15
#define TX1_PIN 16 // warning: these are the correct pins, but this must be 16
#define RGB_PIN 38
const long SERIAL_BAUD_RATE_USB = 115200; // bps (based on default setting of TMC2209 library 115200)
const long SERIAL_BAUD_RATE_TMC2209 = 115200; // bps (based on default setting of TMC2209 library 115200)

//~// Problem Set-up //~//
/// Define motion resolution constants
const uint32_t MICROSTEPS_PER_REV = MICROSTEPPING*STEPS_PER_REV; // µsteps/rev
const float NL_PER_MICROSTEP = A_SYRINGE*PITCH/MICROSTEPS_PER_REV*1000.f; // nL/µsteps
/// Declare system state variables and motion control parameters
float T_INJECT, T_INJECT_0, V_INJECT, V_INJECT_0, STRETCH_INJECT, STRETCH_INJECT_0; // same units as above, respectively
float R_INJECT, R_INJECT_0; // mm
int32_t MICROSTEPS_PER_PERIOD_CVE; // for CRE
float MICROSTEPS_PER_SECOND_CVE; // for CRE
float STRETCH_RATE_CRE; // 1/s
uint32_t FREE_HEAP_MEMORY_32BIT; // bytes
uint32_t MAX_MOTION_TABLE_LENGTH; // number of uint64_t/int64_t pairs for steps/net ticks
uint32_t MOTION_N_SEGMENTS;
std::vector<uint64_t> MOTION_t; // net ticks of FastAccelStepper timer
// TODO - ideally time would probably be a uint32_t just storing dt values instead of total time, but this is easier to read and understand for now
std::vector<int64_t> MOTION_steps; // µsteps in this segment
// TODO - couldn't this just be a uint8_t or conservatively uint16_t? (if so this effects the calculation for MAX_MOTION_TABLE_LENGTH)
unsigned long startTime;
/// Initialize stepper driver, stepper engine, and built-in RGB LED
uint8_t STATUS_DRIVER = 5; // 0 --> not communicating (red), 1 -> communicating but not setup (red), 2 -> setup and communicating (green), 
                           //3 -> Serial not available (magenta), 4 -> pumping (blue), 5 -> unverified (white), 6 -> error (yellow)
TMC2209 stepper_driver; // create TMC2209 stepper driver object
FastAccelStepperEngine engine = FastAccelStepperEngine(); // create stepper engine instance
FastAccelStepper *stepper = NULL; // declare pointer to stepper object
struct MotionQueueState {
  uint32_t segment; // segment number from CRE_TABLE
  uint64_t time; // ticks (total duration of motion) 
  uint32_t drift;  // ticks (time deviation)
};
struct MotionQueueState motionState = {.segment = 0, .time = 0, .drift = 0}; // create control structure for stepper engine
Adafruit_NeoPixel LED_rgb(1, RGB_PIN, NEO_GRB + NEO_KHZ800); // initialize built-in RGB LED to ESP32 chip
/// Function declarations
uint32_t getFree32BitHeapMemory();
float t2V_CVE(float t); float V2t_CVE(float t); float t2V_CRE(float t); float V2t_CRE(float t);
float Vdot2t_CRE(float Vdot); float Vdot2V_CRE(float Vdot); float V2Vdot_CRE(float V);
float V2R(float V); float R2V(float R); 
float stretch2V(float stretch, float R0_factor = 1.f); 
float V2stretch(float V, float R0_factor = 1.f);
float R2stretch(float R, float R0_factor = 1.f);
bool configInjectionEndConditions();
bool makeCRETable(std::vector<uint64_t>& motion_t, std::vector<int64_t>& motion_steps);
void runCRE(FastAccelStepper *stepper, struct MotionQueueState *motionState);
void checkTMC2209Status(HardwareSerial& serialPort);
void printTMC2209Settings();
void printELNICheader();
void configureTMC2209(HardwareSerial& serialPort);
// TODO - ensure all of these functions actually need forward declarations

void setup() {
  /// Initialize built-in NeoPixel RGB LED
  LED_rgb.begin(); // initialize LED
  LED_rgb.setBrightness(10); 
  LED_rgb.setPixelColor(0, LED_rgb.Color(255,255,255)); // set LED color to white
  LED_rgb.show(); // update physical LED state

  /// Set-up Serial Communication and program TMC2209 settings via UART
  Serial.begin(SERIAL_BAUD_RATE_USB); // Initialize USB serial communication with PC
  Serial1.begin(SERIAL_BAUD_RATE_TMC2209,SERIAL_8N1,RX1_PIN,TX1_PIN); // Initialize UART communication with TMC2209 driver
  configureTMC2209(Serial1); // update custom settings to TMC2209 driver

  /// Set-up stepper engine
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (!stepper) {
    LED_rgb.setPixelColor(0, LED_rgb.Color(255,255,0)); LED_rgb.show(); // set LED color to yellow
    while (true) {
      Serial.println("Error: Cannot initialize steppers using FastAccelStepper library!"); 
      delay(10000);
    }
  }
  stepper->setDirectionPin(DIR_PIN);
  stepper->setEnablePin(ENA_PIN); // defaults to low active enables stepper
  stepper->setAutoEnable(false);
  // TODO - make sure that my handling of enable is not causing jerking upon startup

  /// Set-up CVE rate
  if (MODE_USE_REV_S_FOR_CVE) 
    NL_PER_S = REV_PER_S*NL_PER_MICROSTEP*MICROSTEPS_PER_REV; // nL/s
  else 
    REV_PER_S = NL_PER_S/(NL_PER_MICROSTEP*MICROSTEPS_PER_REV); // rev/s
  MICROSTEPS_PER_SECOND_CVE = MICROSTEPS_PER_REV*REV_PER_S;
  MICROSTEPS_PER_PERIOD_CVE = round(MICROSTEPS_PER_SECOND_CVE*1.3981013333333334); // default µsteps/period for TMC 2209 at default clock speed
  // TODO - verify that this is actually correct and if so where it was derived from when I first wrote it

  /// Set-up injection motion profiles
  // TODO it might be nice to make LED function that accepts color and brightness as inputs, also would be cool it if could blink/pulse
  if (!configInjectionEndConditions()) {
    LED_rgb.setPixelColor(0, LED_rgb.Color(255,255,0)); LED_rgb.show(); // set LED color to yellow
    while (true) {
      Serial.println("Error: Invalid value for MODE_CONTROL_END. Expected 'volume', 'stretch', or 'time'. Current value: " + String(MODE_CONTROL_END));
      delay(10000);
    }
  }
  // TODO verify that the memory stuff here is actually helpful
  STRETCH_RATE_CRE = R2stretch(RDOT); // 1/s
  FREE_HEAP_MEMORY_32BIT = getFree32BitHeapMemory(); // bytes
  MAX_MOTION_TABLE_LENGTH = floor(0.95*FREE_HEAP_MEMORY_32BIT/16); // uint64_t/int64 pairs that can be stored in heap memory (16 bytes each pair) (5% safety factor added)
  if (!makeCRETable(MOTION_t,MOTION_steps)) {
    LED_rgb.setPixelColor(0, LED_rgb.Color(255,255,0)); LED_rgb.show(); // set LED color to yellow
    while (true) {
      Serial.println("Error: makeCRETable() failed! Check relevant inputs for CRE.");
      delay(10000);
    }
  } // create motion reference table for CRE
  MOTION_N_SEGMENTS = MOTION_t.size(); // TODO - print off CRE related stuff like this somewhere
  if (MOTION_N_SEGMENTS > MAX_MOTION_TABLE_LENGTH) {
    Serial.println("Error: Not enough heap memory to store CRE table!"); // This is experimental
  }
  // TODO - see if this is redundant

  /// Print settings to Serial Monitor
  printELNICheader();
  Serial.println("---------- NIC Stepper Controller Firmware Settings ----------");
  Serial.println(MODE_CRE ? "\nInjection mode: CRE" : "\nInjection mode: CVE");
  Serial.println("End condition mode: " + MODE_CONTROL_END);
  if (MODE_CRE) {
    Serial.println("Injection rate: " + String(RDOT, 3) + " mm/s"); // TODO - µsteps/s limits on CRE mode related serial print
    Serial.println("Injection stretch rate (estimate): " + String(STRETCH_RATE_CRE, 6) + " 1/s");
    Serial.println("Injection segements: " + String(MOTION_N_SEGMENTS));
  } else {
    Serial.println("Injection rate: " + String(NL_PER_S, 3) + " nL/s"  + " (µsteps/s: " + String(MICROSTEPS_PER_SECOND_CVE, 3) + ")");
  }
  Serial.println("Injection duration: " + String(T_INJECT, 3) + " s");
  Serial.println("Injection effective radius: " + String(R_INJECT, 3) + " mm ");
  Serial.println("Injection volume: " + String(V_INJECT, 3) + " nL ");
  Serial.println("Injection microstep resolution: " + String(NL_PER_MICROSTEP, 6) + " nL");
  Serial.println("\n---------- NIC stepper controller set-up and awaiting for injection command ----------\n");
  Serial.println("Warning: there are no safeguards in place for limiting the motion of of the linear actuator!!! BE CAREFUL!!!\n");

  /// Verify TMC2209 driver is set-up and communicating
  checkTMC2209Status(Serial1);
}

void loop() {
  checkTMC2209Status(Serial1);
  if (Serial.available() > 0) {  // Check if data is available on Serial
    String command = Serial.readStringUntil('\n');  // Read incoming data until newline
    command.trim();  // Remove any whitespace or newline characters
    if (command == "start") { // If the received command is "start"
        stepper_driver.moveAtVelocity(MICROSTEPS_PER_PERIOD_CVE); // Start motor movement
        startTime = millis();  // Record start time
    }
    else if (command == "start_0") { // If the received command is "start"
        stepper_driver.moveAtVelocity(MICROSTEPS_PER_PERIOD_CVE); // Start motor movement
        startTime = millis();  // Record start time
        MODE_INJECT = 2; // set mode to mini inject
    }
    else if (command == "stop") { // Optional: Allow stopping via serial
        stepper_driver.moveAtVelocity(0); // Stop the motor
    }
    else if (command.startsWith("set_pulse_rate ")) { 
      MICROSTEPS_PER_SECOND_CVE = command.substring(15).toFloat(); 
      MICROSTEPS_PER_PERIOD_CVE = round(MICROSTEPS_PER_SECOND_CVE * 1.3981013333333334);
      Serial.println("\nµsteps/s: " + String(MICROSTEPS_PER_SECOND_CVE,3));
    }
    else if (command.startsWith("set_t_inject ")){
      _T_INJECT = abs(command.substring(13).toInt()); // ms
      Serial.println("\nt_inject: " + String(_T_INJECT) + " ms");
    }
    else if (command.startsWith("set_t_inject_0 ")){
      _T_INJECT_0 = abs(command.substring(15).toInt()); // ms
      Serial.println("\nt_inject: " + String(_T_INJECT_0) + " ms");
    }
    // else if (command.startsWith("rdot ")){
    //   RDOT = abs(command.substring(5).toFloat()); // mm/s
    //   STRETCH_RATE_CRE = R2stretch(RDOT); // 1/s
    //   Serial.println("\nRDOT: " + String(RDOT,3) + " mm/s");
    // }
    else if (command.startsWith("set_v_inject ")){
      _V_INJECT = abs(command.substring(13).toInt()); // nL
      Serial.println("\nv_inject: " + String(_V_INJECT) + " nL");
    }
    else if (command.startsWith("set_v_inject_0 ")){
    _V_INJECT_0 = abs(command.substring(15).toInt()); // nL
      Serial.println("\nv_inject: " + String(_V_INJECT_0) + " nL");
    }
    else if (command.startsWith("set_stretch_inject ")){
      _STRETCH_INJECT = abs(command.substring(18).toFloat()); // mm/mm
      Serial.println("\nstretch_inject: " + String(_STRETCH_INJECT,3) + " mm/mm");
    }
    else if (command.startsWith("set_stretch_inject_0 ")){
      _STRETCH_INJECT_0 = abs(command.substring(20).toFloat()); // mm/mm
      Serial.println("\nstretch_inject: " + String(_STRETCH_INJECT_0,3) + " mm/mm");
    }
    else if (command.startsWith("set_mode_control_end ")) {
      MODE_CONTROL_END = command.substring(21); // string
      configInjectionEndConditions(); // update end conditions based on new mode
      Serial.println("\nMode control end: " + MODE_CONTROL_END);
    }
    else if (command.startsWith("setrdot ")) {
      RDOT = abs(command.substring(9).toFloat()); // mm/s
      STRETCH_RATE_CRE = R2stretch(RDOT); // 1/s
      Serial.println("\nRDOT: " + String(RDOT,3) + " mm/s"); 
      MOTION_t.clear(); MOTION_steps.clear();
      if (!makeCRETable(MOTION_t,MOTION_steps)) {
          Serial.println("Error: makeCRETable() failed! Check relevant inputs for CRE."); // TODO - make this wait here until an acceptable value is provided
      } else {
        Serial.println("CRE table updated!");
      }
    } 
    else if (command.startsWith("crestart")){ // "start_CRE"
      Serial.println("Starting CRE");
      // TODO - make sure STATUS_DRIVER is actually being used elsewhere in the code
      STATUS_DRIVER = 4; 
      LED_rgb.setPixelColor(0, LED_rgb.Color(0,0,255)); LED_rgb.show(); // set LED color to blue
      runCRE(stepper, &motionState); // run CRE motion
      Serial.println("CRE motion completed!");
    } else {
      Serial.println("Received unrecognized command: " + command);  // print unrecognized command
    }
  }
  if (startTime != 0 && millis() - startTime >= T_INJECT && MODE_INJECT == 1) { // Stop the motor automatically after 10 seconds
      stepper_driver.moveAtVelocity(0);  // Stop the motor
      Serial.println(String("Motor stopped after ") + (T_INJECT / 1000.0) + " seconds");
      startTime = 0;  // Reset startTime to prevent repeated stops
  }
  else if (startTime != 0 && millis() - startTime >= T_INJECT_0 && MODE_INJECT == 2) { // Stop the motor automatically after 10 seconds
      stepper_driver.moveAtVelocity(0);  // Stop the motor
      Serial.println(String("Motor stopped after ") + (T_INJECT_0 / 1000.0) + " seconds");
      startTime = 0;  // Reset startTime to prevent repeated stops
      MODE_INJECT = 1; // reset mode to normal inject
  }
}

float t2V_CVE(float t) { // s
  return t*NL_PER_S; // nL
}
float V2t_CVE(float V) { // nL
  return V/NL_PER_S; // s
}
float t2V_CRE(float t) { // s
  // unsimplified: 4*PI/3*pow(t*RDOT,3)*1000
  return 4188.79020479f*pow(t*RDOT,3); // nL
  // TODO convert relevant pows to powf
}
float V2t_CRE(float V) { // nL
  // unsimplified: cbrtf((3/(4*PI))*V/1000)/RDOT
  return cbrtf(0.238732f*V)/(RDOT*10.f); // s
}
float Vdot2t_CRE(float Vdot) { // nL/s
  return sqrtf(Vdot/(4000.f*PI*pow(RDOT,3)));
}
float Vdot2V_CRE(float Vdot) { // nL/s
  return t2V_CRE(Vdot2t_CRE(Vdot)); // nL
}
float V2Vdot_CRE(float V) { // nL
  return 4000.f*PI*pow(RDOT,3)*pow(V2t_CRE(V),2); // nL/s
}
float V2R(float V) { // nL
  return cbrtf(3.f/(4.f*PI)*V/1000.f); // mm
}
float R2V(float R) { // mm
  return 4.f*PI/3.f*powf(R,3)*1000.f; // nL
}
float stretch2V(float stretch, float R0_factor) { // mm/mm
  return R2V(stretch*(ID_NEEDLE/2.f)*R0_factor); // nL
}
float V2stretch(float V, float R0_factor) { // nL
  return V2R(V)/((ID_NEEDLE/2.f)*R0_factor); // mm/mm
}
float R2stretch(float R, float R0_factor) { // mm
  return R/((ID_NEEDLE/2.f)*R0_factor); // mm/mm
}

bool configInjectionEndConditions() {
  if (MODE_CONTROL_END == "volume") {
    V_INJECT = _V_INJECT, V_INJECT_0 = _V_INJECT_0; // nL
  } else if (MODE_CONTROL_END == "stretch") {
    STRETCH_INJECT = _STRETCH_INJECT, STRETCH_INJECT_0 = _STRETCH_INJECT_0; // mm/mm
    V_INJECT = stretch2V(STRETCH_INJECT), V_INJECT_0 = stretch2V(STRETCH_INJECT_0); // nL
  } else if (MODE_CONTROL_END == "time") {
    T_INJECT = _T_INJECT, T_INJECT_0 = _T_INJECT_0; // s
  } else {
    return false;
  }
  if (MODE_CONTROL_END != "time") {
    if (MODE_CRE) {
      T_INJECT = V2t_CRE(V_INJECT), T_INJECT_0 = V2t_CRE(V_INJECT_0); // s
    } else {
      T_INJECT = V2t_CVE(V_INJECT), T_INJECT_0 = V2t_CVE(V_INJECT_0); // s
    }
  } else if (MODE_CONTROL_END == "time") {
    if (MODE_CRE) {
      V_INJECT = t2V_CRE(T_INJECT), V_INJECT_0 = t2V_CRE(T_INJECT_0); // nL 
    } else {
      V_INJECT = t2V_CVE(T_INJECT), V_INJECT_0 = t2V_CVE(T_INJECT_0); // nL 
    }
  }
  if (MODE_CONTROL_END != "stretch") {
    STRETCH_INJECT = V2stretch(V_INJECT), STRETCH_INJECT_0 = V2stretch(V_INJECT_0); // mm/mm
  }
  R_INJECT = V2R(V_INJECT), R_INJECT_0 = V2R(V_INJECT_0); // mm
  return true; 
}

uint32_t getFree32BitHeapMemory() { // (bytes) get free heap memory in 32-bit capable regions
  return heap_caps_get_free_size(MALLOC_CAP_32BIT); // bytes
}

void makeCRETable_old(uint64_t* motion_t, uint64_t* motion_steps) {
  // uint64_t tmp1 = 65535*255; // eventually set so that it's not always dtick = 16
  float t_firstTick = V2t_CRE(NL_PER_MICROSTEP)*TICKS_PER_S; // ticks
  float t = 0.f; // ticks
  float t_new = 0.f; // ticks JGB these seems like it should be a unint32_t or something like that
  uint64_t V = 0; // µsteps
  uint64_t V_new = 0; // µsteps
  uint64_t V_f = ceil(V_INJECT*NL_PER_MICROSTEP); // µsteps
  for (int i = 0; i < MOTION_N_SEGMENTS; i++) {
    if (i == (MOTION_N_SEGMENTS - 1)) {
      if ((V_f - V) > 16) {
        Serial.println("Error 1: this should be less than 16!!");
      }
      V_new = V_f; // µsteps
    } else {
      V_new += 16; // µsteps  ***** THIS IS SOMETHING I CAN STILL PLAY WITH ----- it can be done much more adaptively *****
    }
    t_new = ceil(V2t_CRE(V_new*NL_PER_MICROSTEP)*TICKS_PER_S - t_firstTick); // ticks
    motion_t[i] = t_new; // ticks
    motion_steps[i] = V_new - V; // µsteps
    t = t_new; // ticks
    V = V_new; // µsteps
  }

  // For testing
  for (int i = 0; i < MOTION_N_SEGMENTS; i++){
    motion_t[i] = ceil((i+1)*(65535*24.0*0.99)); // ticks (safety factor 0.99 added to avoid drift related overflow)
    motion_steps[i] = 2; // µsteps
    if (i == 0) {
      Serial.println("µsteps/s " + String((float)motion_steps[i]/motion_t[i]*TICKS_PER_S)); // µsteps/s
      Serial.println("revs: " + String((float)motion_steps[i]*MOTION_N_SEGMENTS/MICROSTEPS_PER_REV)); // rev/s
      Serial.println("time: " + String(MOTION_N_SEGMENTS*motion_t[i]/TICKS_PER_S)); // s
    }
  }
}

// TODO - make a way to deal with the fact that buffer may fill up if settings are too low (active calc mode seems unnessary - considering that tabled values should be able to last a long time)
bool makeCRETable(std::vector<uint64_t>& motion_t, std::vector<int64_t>& motion_steps) {
  // Breakdown of how table is calculated
  // 1. Calculate volume when step rate hits 10 Hz.
  // 2. Calculate volume of 0.70*STRETCH_EST_INJECT point (we assume that up to this point the step rate is not super important)
  // 3. Ensure that step rate exceed 10Hz prior to point of 0.70*STRETCH_EST_INJECT
  // 4. Fill at rate of 10 Hz until calc 2 is reached
  // 5. Do rest of injection following this rule
  //       a. If rate is less than 100 Hz, each queued command should have just 1 step in it (10 Hz -> 100 Hz)
  //       b. 100 Hz < rate < 250 Hz, each queued command should have 2 steps in it (50Hz -> 125Hz)
  //       c. 250 Hz < rate < 20k Hz, each queued command should have 10 steps in it (25 Hz -> 2 kHz)
  //       d. 20k Hz < rate < 195k Hz, each queued command should have 100 steps in it (200 Hz -> 2 kHz)
  //       e. No rate may be over 195k Hz
  // TODO - update these limits with the max limits that I figured out
  uint64_t V_10Hz = ceil(Vdot2V_CRE(10.f*NL_PER_MICROSTEP)/NL_PER_MICROSTEP); // µsteps
  uint64_t V_0_70_stretch_estimate = ceil(4000.f*PI/3.f*pow(0.7*(ID_NEEDLE/2.f),3)/NL_PER_MICROSTEP); // µsteps
  Serial.println("V_10Hz: " + String(Vdot2V_CRE(10.f*NL_PER_MICROSTEP)/NL_PER_MICROSTEP) + " µsteps");
  Serial.println("V_0_70_stretch_estimate: " + String(V_0_70_stretch_estimate) + " µsteps");
  if (V_10Hz > V_0_70_stretch_estimate) {
    Serial.println("Error: system unable to produce sufficiently slow rate for stretches near 1 for this radial expansion rate!");
    return false;
  }
  uint32_t segment = 0; 
  uint64_t t = 0; uint64_t t_new = 0; // ticks
  uint64_t V = 0; uint64_t V_new = 0; // µsteps
  uint64_t t_firstTick = ceil(V2t_CRE(NL_PER_MICROSTEP)*TICKS_PER_S); // ticks
  uint64_t V_f = ceil(V_INJECT/NL_PER_MICROSTEP); // µsteps
  float tmp_freq; // Hz
  
  struct StepRule {
    float freq_min; // Hz
    float freq_max; // Hz
    uint16_t nstep; // µsteps
  };

  // TODO - add a general acceleration rule to table to prevent cavitation
  // TODO - make more rules to maximize continuity / duration of motion
  // TODO - potentially try to make it also compatible with Arduino boards
  StepRule stepRules[] = { // Bounds set to provide a continuous stream of values compatible with moveTimed() limitations on ESP32 chip
    {10.0f, 100.0f, 1}, 
    {100.0f, 250.0f, 2},
    {250.0f, 2e4f, 20}, // formerly 10 but it caused issues
    {2e4f, 1.95e5f, 100}
  };
  uint8_t nstepRules = sizeof(stepRules) / sizeof(stepRules[0]); 
  Serial.println("nstepRules: " + String(nstepRules));

  // Serial.println("10Hz region entered");
  while (V < V_10Hz && (segment < MAX_MOTION_TABLE_LENGTH) && V < V_f) {
    V_new += 1; // µsteps
    t_new = round(V_new*TICKS_PER_S/10); // ticks 
    motion_t.push_back(t_new); // ticks
    motion_steps.push_back(V_new - V); // µsteps
    segment += 1;
    V = V_new; // µsteps
    t = t_new; // ticks
  }
  Serial.println("after 10 Hz region V = " + String(V*NL_PER_MICROSTEP,4) + " nL");
  Serial.println("segments after 10Hz region: " + String(segment) + " segments");
  int64_t t_correction = V2t_CRE(V*NL_PER_MICROSTEP)*TICKS_PER_S - t; // ticks (t_CRE(V) > current time 't' because 10 Hz was faster than true CRE rate)
  float t_correction_s = static_cast<float>(t_correction)/TICKS_PER_S; // s

  while ((V < V_f) && (segment < MAX_MOTION_TABLE_LENGTH)) {
    // for (const auto& rule : rules) {
    for (size_t i = 0; i < (nstepRules + 1); ++i) {
      // Serial.println(String(i));
      const StepRule& rule = stepRules[i];
      tmp_freq = rule.nstep/(V2t_CRE((V+rule.nstep)*NL_PER_MICROSTEP) - static_cast<float>(t+t_correction)/TICKS_PER_S); // step/tick
      // if (i > 5384) {
      //   Serial.println("temp_freq: " + String(tmp_freq, 4) + " Hz");
      //   Serial.println("rule.freq_min: " + String(rule.freq_min, 4) + " Hz");
      //   Serial.println("rule.freq_max: " + String(rule.freq_max, 4) + " Hz");
      //   if (tmp_freq >= rule.freq_min && tmp_freq < rule.freq_max) {
      //     Serial.println("it is true!");
      //   }
      // }
      if (tmp_freq >= rule.freq_min && tmp_freq < rule.freq_max) {
        // if (rule.nstep == 100) {
        //   Serial.println("WE TRIED TO INCREMENT BY 100!!");
        // }
        V_new += rule.nstep; // µsteps
        t_new += ceil(V2t_CRE(V_new * NL_PER_MICROSTEP)*TICKS_PER_S - (t + t_correction)); // ticks
        motion_t.push_back(t_new); // ticks
        motion_steps.push_back(V_new - V); // µsteps
        V = V_new; // µsteps
        t = t_new; // ticks
        segment += 1;
        // if (tmp_freq > 1.9e4f)
        // Serial.println("[Segment " + String(segment) + "] V: " + String(V*NL_PER_MICROSTEP, 4) + " nL, t: " + String(t*1e-6, 4) + " s, freq: " + String(tmp_freq, 4) + " Hz, rule.nstep: " + String(rule.nstep));
        break;
      }
      if (tmp_freq > 195000) { 
        // Serial.println("i:" + String(i));
        // Serial.println("segment: " + String(segment));
        // Serial.println("tmp_freq: " + String(tmp_freq, 4) + " Hz");
        // Serial.println("freq_min: " + String(rule.freq_min, 4) + " Hz");
        // Serial.println("freq_max: " + String(rule.freq_max, 4) + " Hz");
        Serial.println("Error: CRE motion table assebmly failed! Requested frequency (" + String(tmp_freq, 4) + " Hz) is not within the bounds of the step rules!");
        return false;
      }
    }
    // TODO - fix issue where the script will always overshoot the desired end volume due to set-up of the while loop
  }
  if (V < V_f) {
    Serial.println("Error: V > V_f! This should not happen!");
    return false;
  }
  Serial.println("Total volume in motion queue: " + String(V*NL_PER_MICROSTEP,4) + " nL");
  Serial.println("Total time in motion queue (something is wrong here!! -- seamingless it works right tho): " + String((t-t_correction)/static_cast<float>(TICKS_PER_S),4) + " s");
  Serial.println("Total number of segments: " + String(segment) + " segments");
  return true;
}

void _handleMoveTimedResult(int8_t rc, MotionQueueState* motionState, uint32_t dt, uint32_t dt_actual, FastAccelStepper *stepper) {
  switch (rc) {
    case MOVE_TIMED_EMPTY:
      if (motionState->segment != 0) { // This is not an error if it's the first segment in the motion sequence
        Serial.print("WARNING: The queue has run out of commands, but the move has been appended!");
        LED_rgb.setBrightness(100);  LED_rgb.show();
        // TODO - add a light change that needs to be manually reset prior to next injection following this error and others too
      }
      /* fallthrough */
    case MOVE_TIMED_OK:
      motionState->segment += 1;
      motionState->drift = dt - dt_actual; // ticks;
      motionState->time += dt_actual; // ticks;
      break;
    case MOVE_TIMED_BUSY:
      if (stepper->queueEntries() <= 1) { // in this case a queue entry is trying to be added that is 31 segments long (proper table calculations should not allow this)
        Serial.print("Attempting to append motion, but the queue is full!");
        LED_rgb.setBrightness(100);  LED_rgb.show();
      }
      break;
    case MOVE_TIMED_TOO_LARGE_ERROR:
      Serial.println("Too many ticks in the move request!");
      LED_rgb.setBrightness(100);  LED_rgb.show();
      break;
    case AQE_ERROR_TICKS_TOO_LOW:
      Serial.print("Tick rate is too high to execute!");
      LED_rgb.setBrightness(100);  LED_rgb.show();
      break;
    default:
      Serial.println(String("Other return code from moveTimed(): ") + rc);
      LED_rgb.setBrightness(100);  LED_rgb.show();
      break;
  }
}
void runCRE(FastAccelStepper *stepper, struct MotionQueueState *motionState) {
  memset(motionState, 0, sizeof(MotionQueueState)); // reset motionState (allows for re-triggering of motion sequence)

  uint32_t dt_actual = 0; // ticks
  uint32_t dt = 0; // ticks
  int8_t rc; // return code from moveTimed function
  stepper->enableOutputs(); // enable stepper outputs (returns true if enabled)
  for (int i = 0; i < ((MOTION_N_SEGMENTS < 32) ? MOTION_N_SEGMENTS : 32); i++) { // Begin motion sequence queue 
    dt = MOTION_t[motionState->segment] - motionState->time + motionState->drift; // ticks
    rc = stepper->moveTimed(MOTION_steps[motionState->segment],dt,&dt_actual,false);
    if (i == 0 && rc == MOVE_TIMED_EMPTY) {
      Serial.println("First segment took up queueEntries(): " + String(stepper->queueEntries()));
    }
    if (rc == MOVE_TIMED_BUSY && motionState->segment != 0) {
      Serial.println("Queue is full. " + String(motionState->segment) + " segments queued.");
      Serial.println("stepper->queueEntries() = " + String(stepper->queueEntries()));
      break;
    }
    _handleMoveTimedResult(rc,motionState,dt,dt_actual,stepper); // handle return code from moveTimed function
  }
  // TODO - add a trigger wait step here to either trigger the actual start of the injection with a digital signal from DAQ or a Serial command
  // while (digitalRead(TRIGGER_PIN) == LOW) { // wait for trigger signal to start injection
  // Serial.println("Starting queue...");
  //stepper->moveTimed(0,0,NULL,true); // trigger step to begin motion sequence
  stepper->addQueueEntry(NULL, true); // trigger step to begin motion sequence
  while (motionState->segment < MOTION_N_SEGMENTS) {
    if (!stepper->isQueueFull()){
      // Serial.println("segment: " + String(motionState->segment));
      // Serial.println("dt: " + String(dt) + " ticks");
      // Serial.println("steps: " + String(MOTION_steps[motionState->segment]) + " µsteps");
      // delay(100);
      dt = MOTION_t[motionState->segment] - motionState->time + motionState->drift; // ticks
      rc = stepper->moveTimed(MOTION_steps[motionState->segment],dt,&dt_actual,true); // re-trigger start just in case the queue temporaily fills up
      _handleMoveTimedResult(rc,motionState,dt,dt_actual,stepper); // handle return code from moveTimed function
    }
  }
  while (stepper->isRunning() || !stepper->isQueueEmpty()) { // wait for the queue to finish running
    delay(10); 
  }
  // Serial.println("CRE injection completed!");
  stepper->enableOutputs(); // disable stepper outputs (returns false if disabled)
}

void checkTMC2209Status(HardwareSerial& serialPort) {
  if (Serial) {
    if (stepper_driver.isSetupAndCommunicating()) {
      if (STATUS_DRIVER != 2) {
        stepper_driver.enable(); // software enable stepper driver
        // Serial.println("Stepper driver is setup and communicating!");
        LED_rgb.setPixelColor(0, LED_rgb.Color(0,255,0)); LED_rgb.show(); // set LED color to green
        STATUS_DRIVER = 2;
      }
    } else if (stepper_driver.isCommunicatingButNotSetup()) {
      // Serial.println("Stepper driver is communicating but not setup! Attempting fresh driver setup.");
      stepper_driver.setup(serialPort);
      if (STATUS_DRIVER != 1) {
        LED_rgb.setPixelColor(0, LED_rgb.Color(255,0,0)); LED_rgb.show(); // set LED color to red
        STATUS_DRIVER = 1;
      }
    } else {
      // Serial.println("Stepper driver is not communicating! Ensure motor power supply is on and check all connections.");
      stepper_driver.setup(serialPort);
      if (STATUS_DRIVER != 0) {
        LED_rgb.setPixelColor(0, LED_rgb.Color(255,0,0)); LED_rgb.show(); // set LED color to red
        STATUS_DRIVER = 0;
      }
    }
  } else {
    // Serial.println("Serial not available! Check USB connection to PC.");
    if (STATUS_DRIVER != 3) {
      LED_rgb.setPixelColor(0, LED_rgb.Color(255,0,255)); LED_rgb.show(); // set LED color to magenta
      STATUS_DRIVER = 3;
    }
  }
}

void configureTMC2209(HardwareSerial& serialPort) {
  stepper_driver.setup(serialPort); // Link TMC2209 to UART
  stepper_driver.setRMSCurrent(RMS_MOTOR_CURRENT, RSENSE); // Set motor RMS current
  stepper_driver.enableInverseMotorDirection(); // Define injection direction as positive
  stepper_driver.setMicrostepsPerStep(MICROSTEPPING); // Set microstepping resolution
  stepper_driver.enableStealthChop(); // Enable quiet mode (optional)
  // TODO - investigae whether or not this is helping or hurting performance
  //stepper_driver.enableCoolStep();
  // I could probably also add something related to Stallguard
}

void printTMC2209Settings() {
  TMC2209::Settings settings = stepper_driver.getSettings();
  Serial.println("TMC2209 Settings:");
  Serial.print("Is Communicating: "); Serial.println(settings.is_communicating);
  Serial.print("Is Setup: "); Serial.println(settings.is_setup);
  Serial.print("Software Enabled: "); Serial.println(settings.software_enabled);
  Serial.print("Microsteps Per Step: "); Serial.println(settings.microsteps_per_step);
  Serial.print("Inverse Motor Direction Enabled: "); Serial.println(settings.inverse_motor_direction_enabled);
  Serial.print("Stealth Chop Enabled: "); Serial.println(settings.stealth_chop_enabled);
  Serial.print("Standstill Mode: "); Serial.println(settings.standstill_mode);
  Serial.print("IRun Percent: "); Serial.println(settings.irun_percent);
  Serial.print("IRun Register Value: "); Serial.println(settings.irun_register_value);
  Serial.print("IHold Percent: "); Serial.println(settings.ihold_percent);
  Serial.print("IHold Register Value: "); Serial.println(settings.ihold_register_value);
  Serial.print("IHoldDelay Percent: "); Serial.println(settings.iholddelay_percent);
  Serial.print("IHoldDelay Register Value: "); Serial.println(settings.iholddelay_register_value);
  Serial.print("Automatic Current Scaling Enabled: "); Serial.println(settings.automatic_current_scaling_enabled);
  Serial.print("Automatic Gradient Adaptation Enabled: "); Serial.println(settings.automatic_gradient_adaptation_enabled);
  Serial.print("PWM Offset: "); Serial.println(settings.pwm_offset);
  Serial.print("PWM Gradient: "); Serial.println(settings.pwm_gradient);
  Serial.print("Cool Step Enabled: "); Serial.println(settings.cool_step_enabled);
  Serial.print("Analog Current Scaling Enabled: "); Serial.println(settings.analog_current_scaling_enabled);
  Serial.print("Internal Sense Resistors Enabled: "); Serial.println(settings.internal_sense_resistors_enabled);
}

void printELNICheader() {
  Serial.println(R"(

 _____    _                 _           _           _         _   _ _____ _____ 
|  ___|  | |               | |         | |         | |       | \ | |_   _/  __ \
| |__ ___| |_ _ __ __ _  __| | __ _    | |     __ _| |__     |  \| | | | | /  \/
|  __/ __| __| '__/ _` |/ _` |/ _` |   | |    / _` | '_ \    | . ` | | | | |    
| |__\__ | |_| | | (_| | (_| | (_| |   | |___| (_| | |_) |   | |\  |_| |_| \__/\
\____|___/\__|_|  \__,_|\__,_|\__,_|   \_____/\__,_|_.__/    \_| \_/\___/ \____/
  
    )");
}