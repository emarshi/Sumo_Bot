/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-12-29 16:07:48
 * @LastEditors: Changhua
 * @Description: Smart Robot Car V4.0
 * @FilePath:
 */
#include "DeviceDriverSet_xxx0.h"
#include <avr/wdt.h>

static void delay_xxx(uint16_t _ms)
{
  wdt_reset();
  for (unsigned long i = 0; i < _ms; i++)
  {
    delay(1);
  }
}

/*RBG LED*/
void DeviceDriverSet_RBGLED::DeviceDriverSet_RBGLED_xxx(uint16_t Duration, uint8_t Traversal_Number,
                                                        CRGB colour)
{
  if (NUM_LEDS < Traversal_Number)
  {
    Traversal_Number = NUM_LEDS;
  }
  for (int Number = 0; Number < Traversal_Number; Number++)
  {
    leds[Number] = colour;
    FastLED.show();
    delay_xxx(Duration);
  }
}

void DeviceDriverSet_RBGLED::DeviceDriverSet_RBGLED_Init(uint8_t set_Brightness)
{
  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(set_Brightness);
}

/*Key*/
uint8_t DeviceDriverSet_Key::keyValue = 0;

static void attachPinChangeInterrupt_GetKeyValue(void)
{
  DeviceDriverSet_Key Key;
  static uint32_t keyValue_time = 0;
  static uint8_t keyValue_temp = 0;
  if ((millis() - keyValue_time) > 500)
  {
    keyValue_temp++;
    keyValue_time = millis();
    if (keyValue_temp > keyValue_Max)
    {
      keyValue_temp = 0;
    }
    Key.keyValue = keyValue_temp;
  }
}

void DeviceDriverSet_Key::DeviceDriverSet_Key_Init(void)
{
  pinMode(PIN_Key, INPUT_PULLUP);
  attachInterrupt(0, attachPinChangeInterrupt_GetKeyValue, FALLING);
}

void DeviceDriverSet_Key::DeviceDriverSet_key_Get(uint8_t *get_keyValue)
{
  *get_keyValue = keyValue;
}

/*ITR20001 Detection*/
bool DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_Init(void)
{
  pinMode(PIN_ITR20001xxxL, INPUT);
  pinMode(PIN_ITR20001xxxM, INPUT);
  pinMode(PIN_ITR20001xxxR, INPUT);
  return false;
}

int DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_getAnaloguexxx_L(void)
{
  return analogRead(PIN_ITR20001xxxL);
}

int DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_getAnaloguexxx_M(void)
{
  return analogRead(PIN_ITR20001xxxM);
}

int DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_getAnaloguexxx_R(void)
{
  return analogRead(PIN_ITR20001xxxR);
}

/*Voltage Detection*/
void DeviceDriverSet_Voltage::DeviceDriverSet_Voltage_Init(void)
{
  pinMode(PIN_Voltage, INPUT);
}

float DeviceDriverSet_Voltage::DeviceDriverSet_Voltage_getAnalogue(void)
{
  float Voltage = (analogRead(PIN_Voltage) * 0.0375);
  Voltage = Voltage + (Voltage * 0.08); //Compensation 8%
  return Voltage;
}

/*Motor control*/
void DeviceDriverSet_Motor::DeviceDriverSet_Motor_Init(void)
{
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
}

/*
 Motor_control：AB / movement direction and speed
*/
void DeviceDriverSet_Motor::DeviceDriverSet_Motor_control(
    boolean direction_A, uint8_t speed_A, //Group A motor parameters
    boolean direction_B, uint8_t speed_B, //Group B motor parameters
    boolean controlED                     //AB enable setting (true)
    )                                     //Motor control
{

  if (controlED == control_enable) //Enable motot control？
  {
    digitalWrite(PIN_Motor_STBY, HIGH);
    { //A...Right

      switch (direction_A) //movement direction control
      {
        case direction_just:
          digitalWrite(PIN_Motor_AIN_1, HIGH);
          analogWrite(PIN_Motor_PWMA, speed_A);
          break;
        case direction_back:

          digitalWrite(PIN_Motor_AIN_1, LOW);
          analogWrite(PIN_Motor_PWMA, speed_A);
          break;
        case direction_void:
          analogWrite(PIN_Motor_PWMA, 0);
          digitalWrite(PIN_Motor_STBY, LOW);
          break;
        default:
          analogWrite(PIN_Motor_PWMA, 0);
          digitalWrite(PIN_Motor_STBY, LOW);
          break;
      }
    }

    { //B...Left
      switch (direction_B)
      {
        case direction_just:
          digitalWrite(PIN_Motor_BIN_1, HIGH);

          analogWrite(PIN_Motor_PWMB, speed_B);
          break;
        case direction_back:
          digitalWrite(PIN_Motor_BIN_1, LOW);
          analogWrite(PIN_Motor_PWMB, speed_B);
          break;
        case direction_void:
          analogWrite(PIN_Motor_PWMB, 0);
          digitalWrite(PIN_Motor_STBY, LOW);
          break;
        default:
          analogWrite(PIN_Motor_PWMB, 0);
          digitalWrite(PIN_Motor_STBY, LOW);
          break;
      }
    }
  }
  else
  {
    digitalWrite(PIN_Motor_STBY, LOW);
    return;
  }
}

/*ULTRASONIC*/
void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Init(void)
{
  pinMode(ECHO_PIN, INPUT); //Ultrasonic module initialization
  pinMode(TRIG_PIN, OUTPUT);
}

void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Get(uint16_t *ULTRASONIC_Get /*out*/)
{
  unsigned int tempda_x = 0;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // Bound blocking time so control loop can keep servicing edge detection.
  // 12000us is enough for ~200cm round trip; this code clamps to 150cm anyway.
  tempda_x = ((unsigned int)pulseIn(ECHO_PIN, HIGH, 12000) / 58);

  if (tempda_x > 150)
  {
    *ULTRASONIC_Get = 150;
  }
  else
  {
    *ULTRASONIC_Get = tempda_x;
  }
}

/*Servo*/

Servo myservo; // create servo object to control a servo

void DeviceDriverSet_Servo::DeviceDriverSet_Servo_Init(unsigned int Position_angle)
{
  myservo.attach(PIN_Servo_z, 500, 2400); //500: 0 degree  2400: 180 degree
  myservo.attach(PIN_Servo_z);
  myservo.write(Position_angle); //sets the servo position according to the 90（middle）
  delay_xxx(500);

  myservo.attach(PIN_Servo_y, 500, 2400); //500: 0 degree  2400: 180 degree
  myservo.attach(PIN_Servo_y);
  myservo.write(Position_angle); //sets the servo position according to the 90（middle）
  delay_xxx(500);
  myservo.detach();
}

void DeviceDriverSet_Servo::DeviceDriverSet_Servo_controls_NoWait(uint8_t Servo,
                                                                  unsigned int Position_angle)
{
  if (Servo == 1 || Servo == 3) //Servo_z
  {
    if (Position_angle <= 1)
      Position_angle = 1;
    if (Position_angle >= 17)
      Position_angle = 17;
    myservo.attach(PIN_Servo_z);
    myservo.write(10 * Position_angle);
  }
  if (Servo == 2 || Servo == 3) //Servo_y
  {
    if (Position_angle <= 3)
      Position_angle = 3;
    if (Position_angle >= 11)
      Position_angle = 11;
    myservo.attach(PIN_Servo_y);
    myservo.write(10 * Position_angle);
  }
}

/*IRrecv*/
IRrecv irrecv(RECV_PIN); //  Create an infrared receive drive object
decode_results results;  //  Create decoding object

void DeviceDriverSet_IRrecv::DeviceDriverSet_IRrecv_Init(void)
{
  irrecv.enableIRIn(); //Enable infrared communication NEC
}

bool DeviceDriverSet_IRrecv::DeviceDriverSet_IRrecv_Get(uint8_t *IRrecv_Get /*out*/)
{
  if (irrecv.decode(&results))
  {
    IR_PreMillis = millis();
    switch (results.value)
    {
      case /* constant-expression */ aRECV_upper:
      case /* constant-expression */ bRECV_upper:
        /* code */ *IRrecv_Get = 1;
        break;
      case /* constant-expression */ aRECV_lower:
      case /* constant-expression */ bRECV_lower:
        /* code */ *IRrecv_Get = 2;
        break;
      case /* constant-expression */ aRECV_Left:
      case /* constant-expression */ bRECV_Left:
        /* code */ *IRrecv_Get = 3;
        break;
      case /* constant-expression */ aRECV_right:
      case /* constant-expression */ bRECV_right:
        /* code */ *IRrecv_Get = 4;
        break;
      case /* constant-expression */ aRECV_ok:
      case /* constant-expression */ bRECV_ok:
        /* code */ *IRrecv_Get = 5;
        break;

      case /* constant-expression */ aRECV_1:
      case /* constant-expression */ bRECV_1:
        /* code */ *IRrecv_Get = 6;
        break;
      case /* constant-expression */ aRECV_2:
      case /* constant-expression */ bRECV_2:
        /* code */ *IRrecv_Get = 7;
        break;
      case /* constant-expression */ aRECV_3:
      case /* constant-expression */ bRECV_3:
        /* code */ *IRrecv_Get = 8;
        break;
      case /* constant-expression */ aRECV_4:
      case /* constant-expression */ bRECV_4:
        /* code */ *IRrecv_Get = 9;
        break;
      case /* constant-expression */ aRECV_5:
      case /* constant-expression */ bRECV_5:
        /* code */ *IRrecv_Get = 10;
        break;
      case /* constant-expression */ aRECV_6:
      case /* constant-expression */ bRECV_6:
        /* code */ *IRrecv_Get = 11;
        break;
      case /* constant-expression */ aRECV_7:
      case /* constant-expression */ bRECV_7:
        /* code */ *IRrecv_Get = 12;
        break;
      case /* constant-expression */ aRECV_8:
      case /* constant-expression */ bRECV_8:
        /* code */ *IRrecv_Get = 13;
        break;
      case /* constant-expression */ aRECV_9:
      case /* constant-expression */ bRECV_9:
        /* code */ *IRrecv_Get = 14;
        break;
      default:
        irrecv.resume();
        return false;
        break;
    }
    irrecv.resume();
    return true;
  }
  else
  {
    return false;
  }
}
