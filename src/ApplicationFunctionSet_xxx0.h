/* Application behavior for the sumo robot and line tracking mode. */
#ifndef _ApplicationFunctionSet_xxx0_H_
#define _ApplicationFunctionSet_xxx0_H_

#include <Arduino.h>

class ApplicationFunctionSet
{
public:
  void ApplicationFunctionSet_Init(void);
  void ApplicationFunctionSet_SensorDataUpdate(void);
  void ApplicationFunctionSet_KeyCommand(void);
  void ApplicationFunctionSet_RGB(void);
  void ApplicationFunctionSet_Follow(void);   // Sumo: search, attack, and escape tape.
  void ApplicationFunctionSet_Tracking(void); // Line tracking.
  void ApplicationFunctionSet_Standby(void);
  void ApplicationFunctionSet_IRrecv(void); // Remote button starts sumo mode.

  boolean Car_LeaveTheGround = true;
  uint16_t TrackingDetection_V = 950; // All three sensors above this means lifted.

private:
  volatile float VoltageData_V;
  volatile int TrackingData_L;
  volatile int TrackingData_M;
  volatile int TrackingData_R;
  boolean VoltageDetectionStatus = false;
  const float VoltageDetection = 7.00;

  bool Sumo_EscapeFromTapeSmart(void);
};

extern ApplicationFunctionSet Application_FunctionSet;
#endif
