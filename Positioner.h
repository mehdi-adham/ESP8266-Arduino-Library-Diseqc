#ifndef POSITIONER_H_
#define POSITIONER_H_

#ifndef UNIT_TEST
#include <Arduino.h>
#endif
#include <stddef.h>
#define __STDC_LIMIT_MACROS
#include <stdint.h>
#include <EEPROM.h>

// Constants

// define's

//eeprom address for positioner value

#define LIMIT_ADD 0       // 1 BYTE 0
#define EAST_LIMIT_ADD 1  // 2 BYTE (1-3)
#define WEST_LIMIT_ADD 3  // 2 BYTE (3-5)
#define STATUS_ADD 5      // 2 BYTE (5-7)
#define CURRENT_SAT_ADD 7 //1 BYTE 7
//8-9 RESERVE for default_factory? (first config)
#define RESET_FACTORY_ADD 8                      //1 BYTE 8
#define SAT_STATUS_ADD(SATnn) (10 + (SATnn * 2)) // 99 Satellite x 2 BYTE (10-206)

//eeprom address for wifi config value

#define RESET_WIFICONFIG_ADD 207
#define APSSID_ADD 208
#define APPSK_ADD (208 + (1 * 14))
#define ws_username_ADD (208 + (2 * 14))
#define ws_password_ADD (208 + (3 * 14))


#define POSITINER_VERSION 1
#define MOTOR_TIMEOUT 2 //2s
#define East_Range 4000
#define West_Range 4000
#define Offset 32768


// Positioner states
const uint8_t EastState = 8;
const uint8_t WestState = 9;
const uint8_t haltState = 10;

//const uint16_t kStateSizeMax = 53;

const uint16_t ZeroStatus = Offset;

// Types

typedef struct
{
    uint8_t sensorpin;
    uint8_t eastpin;
    uint8_t westpin;

    uint8_t posstate; // positioner state
    uint16_t status;  // sensor counter
    uint16_t GotoStatus;
    uint16_t GotoSat;
    uint8_t currentSat;
    uint16_t EastLimit;
    uint16_t WestLimit;
    bool ISLimit;
    bool outOfrang;
} PositionerParams_t;

enum Error
{
    none = 0,
    SensorConnectorORHardwareLimit,
    SoftLimit,
    HardwareLimit,
    SensorConnector,
    OutOfRange
};

//Function's
void mycounter();
void Stop();

// Classes

/// Class for EEPROM.
class ManageEEPROM
{
public:
    explicit ManageEEPROM(int size = 512)
    {
        EEPROM.begin(512);
    } // Constructor

    ~ManageEEPROM(void){
        
    } // Destructor
};

/// Class for Positioner.
class Positioner
{
public:
    explicit Positioner(const uint16_t sensorpin, const uint16_t eastpin,
                        const uint16_t westpin); // Constructor

    ~Positioner(void); // Destructor

    void enableSensorIn(const bool pullup = false);
    void disableSensorIn();

    void MoveToEast();
    void MoveToWest();
    void StopMovement();
    void DisableLimits();
    void EnableLimits();
    void StoreSatellite(uint8_t SATnn, uint16_t status);
    void MoveToSatellite(uint8_t SATnn);
    void ReCalculatePositions(uint16_t data);
    void ClearFlag();
    void ResetFactory();

    uint16_t GetEastLimit();
    uint16_t GetWestLimit();
    uint8_t GetLimit();
    uint8_t GetPosState();
    uint8_t GetCurrentSat();
    uint16_t GetStatusFromSRAM();
    uint16_t GetStatus();
    uint16_t GetStatusSatnn(uint8_t SATnn);

    void SetEastLimit(uint16_t status);
    void SetWestLimit(uint16_t status);
    void SetCurrentSat(uint8_t SATnn);
    void SetStatus(uint16_t status);
    void SetPosState(uint16_t PosState);

    bool CheckMotorTimout();
};
#endif // POSITIONER_H_
