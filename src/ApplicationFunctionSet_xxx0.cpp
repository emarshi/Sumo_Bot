/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2021-01-05 09:30:14
 * @LastEditors: Changhua
 * @Description: Smart Robot Car V4.0
 * @FilePath:
 */
#include <stdlib.h>
#include "ApplicationFunctionSet_xxx0.h"
#include "DeviceDriverSet_xxx0.h"

#include "MPU6050_getdata.h"

ApplicationFunctionSet Application_FunctionSet;

/*Hardware device object list*/
MPU6050_getdata AppMPU6050getdata;
DeviceDriverSet_RBGLED AppRBG_LED;
DeviceDriverSet_Key AppKey;
DeviceDriverSet_ITR20001 AppITR20001;
DeviceDriverSet_Voltage AppVoltage;

DeviceDriverSet_Motor AppMotor;
DeviceDriverSet_ULTRASONIC AppULTRASONIC;
DeviceDriverSet_Servo AppServo;
DeviceDriverSet_IRrecv AppIRrecv;

/*f(x) int */
static boolean function_xxx(long x, long s, long e) // f(x)
{
  if (s <= x && x <= e)
    return true;
  else
    return false;
}

static inline bool tracking_hysteresis_low(int value, bool was_on, uint16_t on_threshold,
                                           uint16_t off_threshold, uint16_t high_limit)
{
  // Enter "line detected" at a higher threshold, but keep it latched
  // until a slightly lower threshold to reduce jitter at edges.
  if (was_on)
  {
    return (off_threshold <= value && value <= high_limit);
  }
  return (on_threshold <= value && value <= high_limit);
}

/*Movement Direction Control List*/
enum SmartRobotCarMotionControl
{
  Forward,    //(1)
  Backward,   //(2)
  Left,       //(3)
  Right,      //(4)
  stop_it = 8 // Stop; retain the original enum value.
};

// The retained modes are standby, line tracking, and sumo (Follow_mode).
enum SmartRobotCarFunctionalModel
{
  Standby_mode = 0,
  TraceBased_mode = 1,
  Follow_mode = 3
};

struct Application_xxx
{
  SmartRobotCarFunctionalModel Functional_Mode;
};

Application_xxx Application_SmartRobotCarxxx0;

static bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void);
static void
ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction,
                                                        uint8_t directionRecord, uint8_t speed,
                                                        uint8_t Kp, uint16_t UpperLimit);
static void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction,
                                                              uint8_t is_speed);
static bool Sumo_ScanOpponentDirection_ServoControls(uint16_t chase_cm, uint16_t lost_cm,
                                                     uint8_t ang_left_deg, uint8_t ang_center_deg,
                                                     uint8_t ang_right_deg, int8_t &dir_out,
                                                     uint16_t &best_cm_out);

void ApplicationFunctionSet::ApplicationFunctionSet_Init(void)
{
  Serial.begin(9600);
  AppVoltage.DeviceDriverSet_Voltage_Init();
  AppMotor.DeviceDriverSet_Motor_Init();
  AppServo.DeviceDriverSet_Servo_Init(90);
  AppKey.DeviceDriverSet_Key_Init();
  AppRBG_LED.DeviceDriverSet_RBGLED_Init(20);
  AppIRrecv.DeviceDriverSet_IRrecv_Init();
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Init();
  AppITR20001.DeviceDriverSet_ITR20001_Init();
  AppMPU6050getdata.MPU6050_dveInit();
  AppMPU6050getdata.MPU6050_calibration();

  Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
}

/*ITR20001 Check if the car leaves the ground*/
static bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void)
{
  if (AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R() >
          Application_FunctionSet.TrackingDetection_V &&
      AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M() >
          Application_FunctionSet.TrackingDetection_V &&
      AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L() >
          Application_FunctionSet.TrackingDetection_V)
  {
    Application_FunctionSet.Car_LeaveTheGround = false;
    return false;
  }
  else
  {
    Application_FunctionSet.Car_LeaveTheGround = true;
    return true;
  }
}

/*
  Straight line movement control：For dual-drive motors, due to frequent motor coefficient deviations and many external interference factors,
  it is difficult for the car to achieve relative Straight line movement. For this reason, the feedback of the yaw control loop is added.
  direction：only forward/backward
  directionRecord：Used to update the direction and position data (Yaw value) when entering the function for the first time.
  speed：the speed range is 0~255
  Kp：Position error proportional constant（The feedback of improving location resuming status，will be modified according to different mode），improve damping control.
  UpperLimit：Maximum output upper limit control
*/
static void
ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction,
                                                        uint8_t directionRecord, uint8_t speed,
                                                        uint8_t Kp, uint16_t UpperLimit)
{
  static float Yaw; // Yaw
  static float yaw_So = 0;
  static uint8_t en = 110;
  static unsigned long is_time;
  if (en != directionRecord || millis() - is_time > 10)
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                           /*direction_B*/ direction_void, /*speed_B*/ 0,
                                           /*controlED*/ control_enable); // Motor control
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
    is_time = millis();
  }
  if (en != directionRecord || Application_FunctionSet.Car_LeaveTheGround == false)
  {
    en = directionRecord;
    yaw_So = Yaw;
  }
  // Add proportional constant Kp to increase rebound effect
  int R = (Yaw - yaw_So) * Kp + speed;
  if (R > UpperLimit)
  {
    R = UpperLimit;
  }
  else if (R < 10)
  {
    R = 10;
  }
  int L = (yaw_So - Yaw) * Kp + speed;
  if (L > UpperLimit)
  {
    L = UpperLimit;
  }
  else if (L < 10)
  {
    L = 10;
  }
  if (direction == Forward) // Forward
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ R,
                                           /*direction_B*/ direction_just, /*speed_B*/ L,
                                           /*controlED*/ control_enable);
  }
  else if (direction == Backward) // Backward
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ L,
                                           /*direction_B*/ direction_back, /*speed_B*/ R,
                                           /*controlED*/ control_enable);
  }
}

/*
  Movement Direction Control:
  Input parameters: direction (Forward, Backward, Left, Right, or stop_it)
                    and speed (0--255).
*/
static void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction,
                                                              uint8_t is_speed)
{
  static uint8_t directionRecord = 0;
  uint8_t Kp;
  uint16_t UpperLimit;
  uint8_t speed = is_speed;
  // Control mode that requires straight line movement adjustment（Car will has movement offset easily in the below mode，the movement cannot achieve the effect of a relatively straight direction
  // so it needs to add control adjustment）
  switch (Application_SmartRobotCarxxx0.Functional_Mode)
  {
    case Follow_mode:
      Kp = 2;
      UpperLimit = 255; // 8-bit PWM max; avoid overflow/truncation
      break;
    default:
      Kp = 10;
      UpperLimit = 255;
      break;
  }
  switch (direction)
  {
    case /* constant-expression */
        Forward:
      /* code */
      if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
      {
        AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                               /*direction_B*/ direction_just, /*speed_B*/ speed,
                                               /*controlED*/ control_enable); // Motor control
      }
      else
      { // When moving forward, enter the direction and position approach control loop processing
        ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Forward, directionRecord, speed, Kp,
                                                                UpperLimit);
        directionRecord = 1;
      }

      break;
    case /* constant-expression */ Backward:
      /* code */
      if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
      {
        AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                               /*direction_B*/ direction_back, /*speed_B*/ speed,
                                               /*controlED*/ control_enable); // Motor control
      }
      else
      { // When moving backward, enter the direction and position approach control loop processing
        ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Backward, directionRecord, speed,
                                                                Kp, UpperLimit);
        directionRecord = 2;
      }

      break;
    case /* constant-expression */ Left:
      /* code */
      directionRecord = 3;
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                             /*direction_B*/ direction_back, /*speed_B*/ speed,
                                             /*controlED*/ control_enable); // Motor control
      break;
    case /* constant-expression */ Right:
      /* code */
      directionRecord = 4;
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                             /*direction_B*/ direction_just, /*speed_B*/ speed,
                                             /*controlED*/ control_enable); // Motor control
      break;
    case /* constant-expression */ stop_it:
      /* code */
      directionRecord = 9;
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                             /*direction_B*/ direction_void, /*speed_B*/ 0,
                                             /*controlED*/ control_enable); // Motor control

      break;
    default:
      directionRecord = 10;
      break;
  }
}

/*
 Robot car update sensors' data:Partial update (selective update)
*/
void ApplicationFunctionSet::ApplicationFunctionSet_SensorDataUpdate(void)
{

  { /*Battery voltage status update*/
    static unsigned long VoltageData_time = 0;
    static int VoltageData_number = 1;
    if (millis() - VoltageData_time > 10) // read and update the data per 10ms
    {
      VoltageData_time = millis();
      VoltageData_V = AppVoltage.DeviceDriverSet_Voltage_getAnalogue();
      if (VoltageData_V < VoltageDetection)
      {
        VoltageData_number++;
        if (VoltageData_number == 500) // Continuity to judge the latest voltage value multiple
        {
          VoltageDetectionStatus = true;
          VoltageData_number = 0;
        }
      }
      else
      {
        VoltageDetectionStatus = false;
      }
    }
  }

  { /*value updation for the IR sensors on the line tracking module：for the line tracking mode*/
    TrackingData_R = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R();
    TrackingData_M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
    TrackingData_L = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
    // ITR20001 Check if the car leaves the ground
    ApplicationFunctionSet_SmartRobotCarLeaveTheGround();
  }
}

/*RBG_LED set*/
void ApplicationFunctionSet::ApplicationFunctionSet_RGB(void)
{
  static unsigned long getAnalogue_time = 0;
  FastLED.clear(true);
  if (true == VoltageDetectionStatus) // Act on low power state？
  {
    if ((millis() - getAnalogue_time) > 3000)
    {
      getAnalogue_time = millis();
    }
  }
  unsigned long temp = millis() - getAnalogue_time;
  if (function_xxx((temp), 0, 500) && VoltageDetectionStatus == true)
  {
    switch (temp)
    {
      case /* constant-expression */ 0 ... 49:
        /* code */
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
        break;
      case /* constant-expression */ 50 ... 99:
        /* code */
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
        break;
      case /* constant-expression */ 100 ... 149:
        /* code */
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
        break;
      case /* constant-expression */ 150 ... 199:
        /* code */
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
        break;
      case /* constant-expression */ 200 ... 249:
        /* code */
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
        break;
      case /* constant-expression */ 250 ... 299:
        /* code */
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
        break;
      case /* constant-expression */ 300 ... 349:
        /* code */
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
        break;
      case /* constant-expression */ 350 ... 399:
        /* code */
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
        break;
      case /* constant-expression */ 400 ... 449:
        /* code */
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
        break;
      case /* constant-expression */ 450 ... 499:
        /* code */
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
        break;
      default:
        break;
    }
  }
  else if (((function_xxx((temp), 500, 3000)) && VoltageDetectionStatus == true) ||
           VoltageDetectionStatus == false)
  {
    switch (Application_SmartRobotCarxxx0.Functional_Mode) // Act on mode control sequence
    {
      case /* constant-expression */ Standby_mode:
        /* code */
        {
          if (VoltageDetectionStatus == true)
          {
            AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/,
                                                  CRGB::Red);
            delay(30);
            AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/,
                                                  CRGB::Black);
            delay(30);
          }
          else
          {
            static uint8_t setBrightness = 0;
            static boolean et = false;
            static unsigned long time = 0;

            if ((millis() - time) > 10)
            {
              time = millis();
              if (et == false)
              {
                setBrightness += 1;
                if (setBrightness == 100)
                  et = true;
              }
              else if (et == true)
              {
                setBrightness -= 1;
                if (setBrightness == 0)
                  et = false;
              }
            }
            // AppRBG_LED.leds[1] = CRGB::Blue;
            AppRBG_LED.leds[0] = CRGB::Violet;
            FastLED.setBrightness(setBrightness);
            FastLED.show();
          }
        }
        break;
      case /* constant-expression */ TraceBased_mode:
        /* code */
        {
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/,
                                                CRGB::Green);
        }
        break;
      case /* constant-expression */ Follow_mode:
        /* code */
        {
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Blue);
        }
        break;
      default:
        break;
    }
  }
}

/*Line tracking mode*/
void ApplicationFunctionSet::ApplicationFunctionSet_Tracking(void)
{
  // Tracking tunables for black line centered between white tape.
  // Based on your measurements: black is typically >= 700.
  const uint8_t TRACK_SPEED_FWD = 100;
  const uint8_t TRACK_SPEED_TURN = 125;       // normal line-correction turns
  const uint8_t TRACK_SPEED_BLIND_TURN = 110; // slower blind-search turns reduce overshoot
  const uint16_t TRACK_DETECT_ON = 680;       // enter line-detected state
  const uint16_t TRACK_DETECT_OFF = 620;      // leave line-detected state (hysteresis)
  const uint16_t TRACK_DETECT_HIGH = 980;     // reject saturated/noisy highs if needed

  static boolean timestamp = true;
  static boolean BlindDetection = true;
  static unsigned long MotorRL_time = 0;
  static bool lineL_on = false;
  static bool lineM_on = false;
  static bool lineR_on = false;

  if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
  {
    if (Car_LeaveTheGround == false) // Check if the car leaves the ground
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      return;
    }

    const uint16_t detect_on = TRACK_DETECT_ON;
    const uint16_t detect_off = TRACK_DETECT_OFF;
    const uint16_t detect_hi = TRACK_DETECT_HIGH;

    lineL_on = tracking_hysteresis_low(TrackingData_L, lineL_on, detect_on, detect_off, detect_hi);
    lineM_on = tracking_hysteresis_low(TrackingData_M, lineM_on, detect_on, detect_off, detect_hi);
    lineR_on = tracking_hysteresis_low(TrackingData_R, lineR_on, detect_on, detect_off, detect_hi);

    if (lineM_on)
    {
      /*Achieve straight and uniform speed movement*/
      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, TRACK_SPEED_FWD);
      timestamp = true;
      BlindDetection = true;
    }
    else if (lineR_on)
    {
      /*Turn right*/
      ApplicationFunctionSet_SmartRobotCarMotionControl(Right, TRACK_SPEED_TURN);
      timestamp = true;
      BlindDetection = true;
    }
    else if (lineL_on)
    {
      /*Turn left*/
      ApplicationFunctionSet_SmartRobotCarMotionControl(Left, TRACK_SPEED_TURN);
      timestamp = true;
      BlindDetection = true;
    }
    else ////The car is not on the black line. execute Blind scan
    {
      if (timestamp == true) // acquire timestamp
      {
        timestamp = false;
        MotorRL_time = millis();
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      }
      /*Blind Detection*/
      if ((function_xxx((millis() - MotorRL_time), 0, 200) ||
           function_xxx((millis() - MotorRL_time), 1600, 2000)) &&
          BlindDetection == true)
      {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Right, TRACK_SPEED_BLIND_TURN);
      }
      else if (((function_xxx((millis() - MotorRL_time), 200, 1600))) && BlindDetection == true)
      {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Left, TRACK_SPEED_BLIND_TURN);
      }
      else if ((function_xxx((millis() - MotorRL_time), 3000, 3500))) // Blind Detection ...s ?
      {
        BlindDetection = false;
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      }
    }
  }
  else if (false == timestamp)
  {
    BlindDetection = true;
    timestamp = true;
    MotorRL_time = 0;
    lineL_on = false;
    lineM_on = false;
    lineR_on = false;
  }
}

/*
  Following mode：
*/
// The scan helper uses DeviceDriverSet_Servo_controls_NoWait(Servo, Position_angle).
// Your servo driver expects "Position_angle" in *steps* (1..17) and writes 10*step degrees.
// So we pass steps, not degrees.
//
// Assumption: your ultrasonic is on Servo_z => Servo channel 1.
// If it's on Servo_y instead, change SERVO_ULTRASONIC from 1 -> 2.
// Control priority in this mode:
// 1) Safety (picked up) -> stop
// 2) Edge escape -> immediate override
// 3) Opponent engage/attack
// 4) Search pattern when no target
void ApplicationFunctionSet::ApplicationFunctionSet_Follow(void)
{
  // --- Tunables ---
  const uint16_t OPPONENT_ATTACK_CM = 60;                    // strict attack threshold requested
  const uint16_t OPPONENT_TRACK_CM = OPPONENT_ATTACK_CM + 6; // small jitter margin
  const uint16_t ATTACK_CM = OPPONENT_ATTACK_CM;
  const uint16_t CHASE_CM = OPPONENT_TRACK_CM;
  const uint16_t LOST_CM = 150;     // treat larger as "nothing"
  const uint8_t SPEED_ATTACK = 255; // max PWM
  const uint8_t SPEED_CHASE = 200;
  const uint8_t SPEED_SEARCH = 20;
  const uint8_t ATTACK_BIAS_PCT = 88; // 88%/100% keeps steering with high push power

  // Servo scan angles
  const uint8_t ANG_LEFT = 150;
  const uint8_t ANG_CENTER = 80;
  const uint8_t ANG_RIGHT = 20;

  const unsigned long LOST_MEMORY_MS = 550;
  const unsigned long SEARCH_TURN_MS = 520;
  const unsigned long SEARCH_DRIVE_MS = 160;
  const unsigned long TARGET_SEEN_HOLD_MS = 700; // keep track lock through brief dropouts
  const unsigned long ATTACK_HOLD_MS = 360;      // keep pushing through brief sensor dropouts

  // Persistent state for non-blocking behavior across loop calls.
  static unsigned long last_seen_ms = 0;
  static unsigned long search_t0 = 0;
  static bool search_turn_right = true;
  static uint8_t search_phase = 0; // 0=turn, 1=forward
  static bool was_follow_mode = false;
  static unsigned long seen_hold_until_ms = 0;
  static unsigned long attack_hold_until_ms = 0;
  static int8_t last_dir = 0;

  // Reset local follow-mode state when leaving Follow_mode.
  if (Application_SmartRobotCarxxx0.Functional_Mode != Follow_mode)
  {
    was_follow_mode = false;
    return;
  }

  // One-time init when Follow_mode is first entered.
  if (!was_follow_mode)
  {
    was_follow_mode = true;
    search_t0 = millis();
    search_phase = 0;
    search_turn_right = true;
    last_seen_ms = 0;
    seen_hold_until_ms = 0;
    attack_hold_until_ms = 0;
    last_dir = 0;
  }

  // Refresh sensor cache used by this cycle.
  ApplicationFunctionSet_SensorDataUpdate();

  // Safety: stop if picked up
  if (Car_LeaveTheGround == false)
  {
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    return;
  }

  // Ring edge escape (tape)
  if (Sumo_EscapeFromTapeSmart())
  {
    return;
  }

  // Update directional estimate and nearest distance from servo+ultrasonic scan.
  int8_t dir = 0;
  uint16_t best_cm = LOST_CM;
  (void)Sumo_ScanOpponentDirection_ServoControls(CHASE_CM, LOST_CM, ANG_LEFT, ANG_CENTER, ANG_RIGHT,
                                                 dir, best_cm);

  const unsigned long now = millis();
  if (dir != 0)
  {
    // Keep last non-zero direction so attack can continue briefly through noisy reads.
    last_dir = dir;
  }

  // Raw thresholds from the current frame.
  const bool opponent_track_raw = (best_cm <= CHASE_CM);
  const bool opponent_close_raw = (best_cm <= ATTACK_CM);

  if (opponent_track_raw)
  {
    // Extend target lock window while target is within tracking range.
    seen_hold_until_ms = now + TARGET_SEEN_HOLD_MS;
    last_seen_ms = now;
  }

  const bool opponent_seen =
      opponent_track_raw || (seen_hold_until_ms != 0 && now < seen_hold_until_ms);

  if (opponent_close_raw)
  {
    // Extend attack hold window while target is within attack range.
    attack_hold_until_ms = now + ATTACK_HOLD_MS;
    last_seen_ms = now;
  }

  const bool opponent_close =
      opponent_close_raw || (attack_hold_until_ms != 0 && now < attack_hold_until_ms);

  if (opponent_close)
  {
    // Apply a high-power forward push with a small wheel bias for steering.
    int8_t attack_dir = (dir != 0) ? dir : last_dir;
    const uint8_t bias_speed = (uint16_t)SPEED_ATTACK * ATTACK_BIAS_PCT / 100;
    if (attack_dir > 0)
      AppMotor.DeviceDriverSet_Motor_control(
          /*direction_A*/ direction_just, /*speed_A*/ bias_speed,
          /*direction_B*/ direction_just, /*speed_B*/ SPEED_ATTACK,
          /*controlED*/ control_enable);
    else if (attack_dir < 0)
      AppMotor.DeviceDriverSet_Motor_control(
          /*direction_A*/ direction_just, /*speed_A*/ SPEED_ATTACK,
          /*direction_B*/ direction_just, /*speed_B*/ bias_speed,
          /*controlED*/ control_enable);
    else
      AppMotor.DeviceDriverSet_Motor_control(
          /*direction_A*/ direction_just, /*speed_A*/ SPEED_ATTACK,
          /*direction_B*/ direction_just, /*speed_B*/ SPEED_ATTACK,
          /*controlED*/ control_enable);
    return;
  }

  if (opponent_seen)
  {
    // Target tracked but not in close-attack range: steer to align.
    if (dir > 0)
      ApplicationFunctionSet_SmartRobotCarMotionControl(Right, SPEED_CHASE);
    else if (dir < 0)
      ApplicationFunctionSet_SmartRobotCarMotionControl(Left, SPEED_CHASE);
    else
      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, SPEED_CHASE);
    return;
  }

  // Recently lost target: continue moving forward to re-acquire quickly.
  if (last_seen_ms != 0 && (now - last_seen_ms) <= LOST_MEMORY_MS)
  {
    ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, SPEED_CHASE);
    return;
  }

  // No opponent: active search pattern while servo keeps scanning.
  if (search_phase == 0)
  {
    // Phase 0: in-place sweep turn.
    ApplicationFunctionSet_SmartRobotCarMotionControl(search_turn_right ? Right : Left,
                                                      SPEED_SEARCH);
    if (now - search_t0 >= SEARCH_TURN_MS)
    {
      search_phase = 1;
      search_t0 = now;
    }
  }
  else
  {
    // Phase 1: short forward burst between sweeps.
    ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, SPEED_SEARCH);
    if (now - search_t0 >= SEARCH_DRIVE_MS)
    {
      search_phase = 0;
      search_turn_right = !search_turn_right;
      search_t0 = now;
    }
  }
}

bool ApplicationFunctionSet::Sumo_EscapeFromTapeSmart(void)
{
  // Tape thresholds (same as your tuned values)
  const uint16_t TAPE_L = 735;
  const uint16_t TAPE_M = 855;
  const uint16_t TAPE_R = 815;

  const bool tapeL = (uint16_t)TrackingData_L >= TAPE_L;
  const bool tapeM = (uint16_t)TrackingData_M >= TAPE_M;
  const bool tapeR = (uint16_t)TrackingData_R >= TAPE_R;

  static uint8_t phase = 0; // 0=idle, 1=back, 2=turn
  static unsigned long t0 = 0;
  static uint8_t turn_dir = 0; // 0=left, 1=right
  static bool flip = false;

  // Any tape sensor high means "edge risk now".
  const bool tape_now = tapeL || tapeM || tapeR;

  if (phase == 0)
  {
    if (!tape_now)
      return false;

    // Decide direction once per trigger
    if (tapeL && !tapeR)
      turn_dir = 1; // tape left -> turn right
    else if (tapeR && !tapeL)
      turn_dir = 0; // tape right -> turn left
    else
    {
      flip = !flip;
      turn_dir = flip ? 0 : 1;
    }

    phase = 1;
    t0 = millis();
  }

  if (phase == 1)
  {
    // Back away first to create clearance from the edge.
    const unsigned long back_ms = tapeM ? 400 : 300;
    ApplicationFunctionSet_SmartRobotCarMotionControl(Backward, 255);

    if (millis() - t0 >= back_ms)
    {
      phase = 2;
      t0 = millis();
    }
    return true;
  }

  // phase == 2
  // Then turn away to re-enter the arena before returning control to follow mode.
  ApplicationFunctionSet_SmartRobotCarMotionControl(turn_dir ? Right : Left, 255);

  if (millis() - t0 >= rand() % 141 + 250) // Keep the existing randomized turn timing.
  {
    phase = 0;
  }
  return true;
}

static inline uint8_t deg_to_step_10(uint8_t deg)
{
  // Maps degrees to 10-degree steps expected by DeviceDriverSet_Servo_controls.
  // e.g. 80deg -> 8, 150deg -> 15, 20deg -> 2
  uint8_t step = (deg + 5) / 10; // round to nearest
  if (step < 1)
    step = 1;
  if (step > 17)
    step = 17;
  return step;
}

static bool Sumo_ScanOpponentDirection_ServoControls(uint16_t chase_cm, uint16_t lost_cm,
                                                     uint8_t ang_left_deg, uint8_t ang_center_deg,
                                                     uint8_t ang_right_deg, int8_t &dir_out,
                                                     uint16_t &best_cm_out)
{
  // Returns the nearest observed distance among left/center/right samples and
  // a direction hint: -1 left, 0 center, +1 right.
  const uint8_t SERVO_ULTRASONIC = 1; // 1=Servo_z, 2=Servo_y

  // Non-blocking scan sequence: center -> right -> center -> left.
  static uint8_t scan_idx = 0; // 0=center, 1=right, 2=center, 3=left
  static unsigned long servo_t0 = 0;

  static uint16_t d_center = 999;
  static uint16_t d_right = 999;
  static uint16_t d_left = 999;

  const unsigned long STEP_MS = 180;
  const unsigned long SERVO_SETTLE_MS = 60;

  // Step servo position on schedule.
  if (millis() - servo_t0 >= STEP_MS)
  {
    servo_t0 = millis();
    scan_idx = (scan_idx + 1) & 0x03;

    uint8_t target_deg = (scan_idx == 1)   ? ang_right_deg
                         : (scan_idx == 3) ? ang_left_deg
                                           : ang_center_deg;

    const uint8_t step10 = deg_to_step_10(target_deg);
    AppServo.DeviceDriverSet_Servo_controls_NoWait(SERVO_ULTRASONIC, step10);
  }

  // Give servo/sensor a short settle window before sampling this angle.
  if (millis() - servo_t0 >= SERVO_SETTLE_MS)
  {
    uint16_t cm = 0;
    AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&cm /*out*/);
    if (cm == 0 || cm > lost_cm)
      cm = lost_cm;

    if (scan_idx == 1)
      d_right = cm;
    else if (scan_idx == 3)
      d_left = cm;
    else
      d_center = cm;
  }

  // Choose direction based on the nearest sampled distance.
  uint16_t best = d_center;
  int8_t best_dir = 0;
  if (d_right < best)
  {
    best = d_right;
    best_dir = +1;
  }
  if (d_left < best)
  {
    best = d_left;
    best_dir = -1;
  }

  best_cm_out = best;
  dir_out = best_dir;

  return (best <= chase_cm);
}

/*Standby mode*/
void ApplicationFunctionSet::ApplicationFunctionSet_Standby(void)
{
  static bool is_ED = true;
  static uint8_t cout = 0;
  if (Application_SmartRobotCarxxx0.Functional_Mode == Standby_mode)
  {
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    if (true ==
        is_ED) // Used to zero yaw raw data(Make sure the car is placed on a stationary surface!)
    {
      static unsigned long timestamp; // acquire timestamp
      if (millis() - timestamp > 20)
      {
        timestamp = millis();
        if (ApplicationFunctionSet_SmartRobotCarLeaveTheGround() /* condition */)
        {
          cout += 1;
        }
        else
        {
          cout = 0;
        }
        if (cout > 10)
        {
          is_ED = false;
          AppMPU6050getdata.MPU6050_calibration();
        }
      }
    }
  }
}

/*Key command*/
void ApplicationFunctionSet::ApplicationFunctionSet_KeyCommand(void)
{
  uint8_t get_keyValue;
  static uint8_t temp_keyValue = keyValue_Max;
  AppKey.DeviceDriverSet_key_Get(&get_keyValue);

  if (temp_keyValue != get_keyValue)
  {
    temp_keyValue = get_keyValue;

    Serial.println(get_keyValue);

    switch (get_keyValue)
    {
      case /* constant-expression */ 1:
        /* code */
        Application_SmartRobotCarxxx0.Functional_Mode = TraceBased_mode;
        break;
      case /* constant-expression */ 2:
        /* code */
        Application_SmartRobotCarxxx0.Functional_Mode = Follow_mode;
        break;
      case /* constant-expression */ 3:
        /* code */
        Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
        break;
      default:

        break;
    }
  }
}

/*Infrared remote control*/
void ApplicationFunctionSet::ApplicationFunctionSet_IRrecv(void)
{
  uint8_t IRrecv_button = 0;
  if (!AppIRrecv.DeviceDriverSet_IRrecv_Get(&IRrecv_button /*out*/))
  {
    return;
  }

  // The DOWN button (code 2) starts sumo mode.
  // All other IR buttons are ignored; the onboard button can select standby.
  // Change this value for a second car using a different remote button.
  // Decoded buttons: UP=1, DOWN=2, LEFT=3, RIGHT=4, OK=5, digits 1..9=6..14.
  // These codes are mapped in DeviceDriverSet_IRrecv_Get() in DeviceDriverSet_xxx0.cpp.
  const uint8_t SUMO_START_IR_BUTTON = 2; // DOWN on the supplied remote.
  if (IRrecv_button == SUMO_START_IR_BUTTON)
  {
    Application_SmartRobotCarxxx0.Functional_Mode = Follow_mode;
  }
  AppIRrecv.IR_PreMillis = millis();
}
