// Dubloon Descrier Demo 3 Code
// Created by the Dubloon Descrier Group
// March 14, 2021

/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

// Include the AccelStepper library:
#include <AccelStepper.h>
// Include the MiltiStepper library:
//#include <MultiStepper.h>
// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define dirPin1   22
#define stepPin1  23
#define dirPin2   24
#define stepPin2  25
#define dirPin3   26
#define stepPin3  27
#define dirPin4   28
#define stepPin4  29
#define motorInterfaceType 1

// Four pins are required to operate the ULN2003 stepper driver
#define motorPin1  30      // IN1 on the ULN2003 driver
#define motorPin2  31      // IN2 on the ULN2003 driver
#define motorPin3  32      // IN3 on the ULN2003 driver
#define motorPin4  33      // IN4 on the ULN2003 driver
// Motor steps to complete one 360 degree rotation
// This is tuned for a 28BYJ-48 Step Motor @ 5v + ULN2003 Driver
#define dropperStepsPerRotation 4096
// Motor max steps per second
// Any number over 1000 on an Arduino has been found to be unstable with ULN2003 driver
#define dropperMaxSpeed 1000
// Motor max acceleration
// Acceleration to prevent missed steps, in steps per second per second
#define dropperMaxAcceleration 500
// Define the AccelStepper interface type; 4 wire motor in half step mode:
#define MotorType 8
// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper library with 28BYJ-48 stepper motor:
AccelStepper dropper = AccelStepper(MotorType, motorPin1, motorPin3, motorPin2, motorPin4);

// Set values for distances
// Wheel diameter 62mm
#define runStraight 8000   //16000 = time required to perform 1024 steps (32 steps using 32 subdivisions)
#define turn90 8000  //75000 = time required to perform a 90° turn on wheels with a 16" center to center length
// Set pins for switches
#define leftTurn  A0  //button that represents a "left turn" signal
#define rightTurn A1  //button that represents a "right turn" signal
#define dropFlag  A2  //button that represents a "drop flag" signal

// Create a new instance of the AccelStepper class:
AccelStepper FL = AccelStepper(motorInterfaceType, stepPin1, dirPin1); //Front Left motor
AccelStepper FR = AccelStepper(motorInterfaceType, stepPin2, dirPin2); //Front Right motor
AccelStepper RL = AccelStepper(motorInterfaceType, stepPin3, dirPin3); //Rear Left motor
AccelStepper RR = AccelStepper(motorInterfaceType, stepPin4, dirPin4); //Rear Right motor

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         0
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  // Set the maximum speed in steps per second:
  FL.setMaxSpeed(2000);
  FR.setMaxSpeed(2000);
  RL.setMaxSpeed(2000);
  RR.setMaxSpeed(2000);
  
  // Set up the maximum speed and acceleration for the dropper
  dropper.setMaxSpeed(dropperMaxSpeed);           // set max motor speed
  dropper.setAcceleration(dropperMaxAcceleration);// set max motor acceleration
  
  // initialize the switch pins as inputs:
  pinMode(leftTurn, INPUT);
  pinMode(rightTurn, INPUT);
  pinMode(dropFlag, INPUT);
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  /* Got a packet! */
  // printHex(packetbuffer, len);
  int dropMarkerFlag = LOW;
  int driveStraightFlag = LOW;
  int leftTurnFlag = LOW;
  int rightTurnFlag = LOW;
  int reverseFlag = LOW;
  
  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed) {
      switch (buttnum) {
        case 1:
          dropMarkerFlag = HIGH;
          break;
        case 5:
          driveStraightFlag = HIGH;
          break;
        case 6:
          reverseFlag = HIGH;
          break;
        case 7:
          leftTurnFlag = HIGH;
          break;
        case 8:
          rightTurnFlag = HIGH;
        default:
          break;
      }
      Serial.println(" pressed");
    } else {
      dropMarkerFlag = LOW;
      driveStraightFlag = LOW;
      leftTurnFlag = LOW;
      rightTurnFlag = LOW;
      reverseFlag = LOW;
    }
  }

  if (driveStraightFlag == HIGH) { //the flag to drive straight has been set
    long i = 0;
    do{ 
      FL.setSpeed(2000);  //FL motor turns clockwise
      RL.setSpeed(2000);  //RL motor turns clockwise
      FR.setSpeed(-2000); //FR motor turns counterclockwise
      RR.setSpeed(-2000); //RR motor turns counterclockwise
      FL.runSpeed();
      RL.runSpeed();
      FR.runSpeed();
      RR.runSpeed();
      i++;
    } while (i < runStraight);
  }
  
  if (reverseFlag == HIGH) { //the flag to reverse has been set
    long i = 0;
    do{ 
      FL.setSpeed(-2000);  //FL motor turns clockwise
      RL.setSpeed(-2000);  //RL motor turns clockwise
      FR.setSpeed(2000); //FR motor turns counterclockwise
      RR.setSpeed(2000); //RR motor turns counterclockwise
      FL.runSpeed();
      RL.runSpeed();
      FR.runSpeed();
      RR.runSpeed();
      i++;
    } while (i < runStraight);
    }
    
    if (dropMarkerFlag == HIGH) { //the flag to drop marker has been set
    dropper.enableOutputs();              //initialize the output pins
    dropper.move(dropperStepsPerRotation);  //set the target position relative to current position
    dropper.runToPosition();             //move to the target position
    dropper.disableOutputs();     // turns off motor locking
  }
 
  if (leftTurnFlag == HIGH) { //flag to turn left has been set
    long i = 0;
    do{ 
    FL.setSpeed(-2000); //FL motor turns counterclockwise
    RL.setSpeed(-2000); //RL motor turns counterclockwise
    FR.setSpeed(-2000); //FR motor turns counterclockwise
    RR.setSpeed(-2000); //RR motor turns counterclockwise
    FL.runSpeed();
    RL.runSpeed();
    FR.runSpeed();
    RR.runSpeed();
    i++;
    } while (i < turn90);
  }
    if (rightTurnFlag == HIGH) { //flag to turn right has been set
    long i = 0;
    do{ 
    FL.setSpeed(2000);  //FL motor turns clockwise
    RL.setSpeed(2000);  //RL motor turns clockwise
    FR.setSpeed(2000);  //FR motor turns clockwise
    RR.setSpeed(2000);  //RR motor turns clockwise
    FL.runSpeed();
    RL.runSpeed();
    FR.runSpeed();
    RR.runSpeed();
    i++;
    } while (i < turn90);
  }
}
