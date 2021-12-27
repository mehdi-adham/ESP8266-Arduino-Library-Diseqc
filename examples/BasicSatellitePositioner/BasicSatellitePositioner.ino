#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include "Positioner.h"
#include "Diseqc.h"


//LOG
String Poslog = "";
uint8_t logCount = 20;

//eeprom
ManageEEPROM MyEEPROM(512);

//DISEQC AND POSITIONER
uint16_t RECV_PIN = D2;   //GPIO4
uint16_t SENSOR_PIN = D1; //GPIO5
uint16_t EAST_PIN = D3;   //GPIO0
uint16_t WEST_PIN = D4;   //GPIO2
enum Error PositionerError = Error::none;
decode_results results;

Diseqc diseqc(RECV_PIN, 1024, kTimeoutMs);
Positioner positioner(SENSOR_PIN, EAST_PIN, WEST_PIN);



void setup() { 
  Serial.begin(9600);
  diseqc.begin(); 
}

void loop() {
  
  //running
  if (positioner.GetPosState() != haltState)
  {
    /******************/
    mycounter();
    /******************/
    //error check
    if (positioner.CheckMotorTimout())
    {
      PositionerError = Error::SensorConnectorORHardwareLimit;
    }
    else
    {
      PositionerError = Error::none;
    }
  }

  //store current SRAM status in eeprom if not match.
  if (positioner.GetStatusFromSRAM() != positioner.GetStatus())
  {
    positioner.SetStatus(positioner.GetStatusFromSRAM());
  }

//decode diseqc message.
  if (diseqc.decode(&results))
  {
    char str[20];
    sprintf(str, "%02X %02X %02X %06X ", results.framming, results.address, results.command, results.data);
    Poslog = (String)str;

    if ((results.framming == Master_First_transmission_No_reply ||
         results.framming == Master_Repeated_First_transmission_No_reply) &&
        (results.address == Any_Positioner ||
         results.address == Polar_Azimuth_Positioner))
    {
      switch (results.command)
      {
      case Reset: //Reset DiSEqCä microcontroller
        ESP.restart();
        Poslog += "restart";
        break;

      case Clr_Reset: // Clear the “Reset” flag
        positioner.ClearFlag();
        Poslog += "ClearFlag";
        break;

      case Standby: // Stand by :Switch peripheral power supply off
        //ESP.deepSleep(0); //deep sleep mode until RESET pin is connected to a LOW signal
        Poslog += "Standby";
        break;

      case Power_on: // Power on :Switch peripheral power supply on
        Poslog += "Poweron";
        break;

      case Halt: //Stop Positioner movement
        positioner.StopMovement();
        Poslog += "StopMovement";
        break;

      case Limits_Off: //Disable Limits
        positioner.DisableLimits();
        Poslog += "DisableLimits";
        break;

      case Limit_E: //Set East Limit (& Enable recommended)
        positioner.SetEastLimit(positioner.GetStatusFromSRAM());
        Poslog += "SetEastLimit";
        positioner.EnableLimits();
        break;

      case Limit_W: //Set West Limit (& Enable recommended)
        positioner.SetWestLimit(positioner.GetStatusFromSRAM());
        Poslog += "SetWestLimit";
        positioner.EnableLimits();
        break;

      case Drive_East: //Drive Motor East (with optional timeout/steps)
        positioner.MoveToEast();
        Poslog += "MoveToEast";
        break;

      case Drive_West: //Drive Motor West (with optional timeout/steps)
        positioner.MoveToWest();
        Poslog += "MoveToWest";
        break;

      case Store_nn: //Store Satellite Position & Enable Limits
        if (results.data == 0)
        {
          positioner.EnableLimits();
          Poslog += "EnableLimits";
        }
        else
        {
          uint16_t status = positioner.GetStatusFromSRAM();
          positioner.StoreSatellite(results.data, status);
          positioner.SetCurrentSat(results.data);
          Poslog += "Store" + (String)results.data;
        }
        break;

      case Goto_nn: //Drive Motor to Satellite Position nn
        positioner.MoveToSatellite(results.data);
        Poslog += "Goto" + (String)results.data;
        break;

      case Set_Posns: //(Re-) Calculate Satellite Positions
        positioner.ReCalculatePositions(results.data);
        Poslog += "ReCalculate" + (String)results.data;
        break;

      default:
        Poslog += "NotSupportCMD";
        break;
      }

      Serial.println(Poslog);
    }

    diseqc.resume();

  }
}
