/******************************************************************************
 *  Ang_Robo.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Implementation of the Class Ang_Robo
 *  Author: Kazuhiro.Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "Ang_Robo.h"
#include "Clock.h"

//補助輪モード
//#define DEBUG_STOP
//#define DEBUG_LINETRACE
//#define DEBUG_LINETRACE_BALANCER

using ev3api::Clock;

Clock*       robo_Clock;


// 定数宣言
const int Ang_Robo::LOW    = 30;    // 低速
const int Ang_Robo::NORMAL = 50;    // 通常
const int Ang_Robo::HIGH   = 70;    // 高速

/**
 * コンストラクタ
 * @param gyroSensor ジャイロセンサ
 * @param leftWheel  左モータ
 * @param rightWheel 右モータ
 * @param balancer   バランサ
 */
Ang_Robo::Ang_Robo(const ev3api::GyroSensor& gyroSensor,
                                 ev3api::Motor& leftWheel,
                                 ev3api::Motor& rightWheel,
                                 ev3api::Motor& tail_motor,
                                 Balancer* balancer)
    : mGyroSensor(gyroSensor),
      mLeftWheel(leftWheel),
      mRightWheel(rightWheel),
      mTail_Motor(tail_motor),
      mBalancer(balancer),
      mForward(LOW),
      mTurn(LOW),

     tail_motor_pwm(0)
 {
}


void Ang_Robo::init() {
  offset = mGyroSensor.getAnglerVelocity();  // ジャイロセンサ値
    
  robo_Clock       = new Clock();        

  mLeftWheel.reset();
  mRightWheel.reset();
  
  mBalancer->init(offset);
  balance_mode = true; 


  gTail_pwm->init_pid(0, 0, 0, 1); //it would be chanbed bellow 
  //  gTail_pwm->init_pid(1, 0.005, 0.001, dT_4ms);    

  Anago_Mode = Ang_Balance;

  Stand_Mode = Balance_Mode;
}

/**
 * PWM値を設定する
 * @param forward 前進値
 * @param turn    旋回値
 */
void Ang_Robo::setCommand(int forward, float yawratecmd, signed int tail_ang_req, float yawrate, bool tail_stand_mode, bool tail_lug_mode) {
  mForward         = forward;
  mYawratecmd      = yawratecmd;
  mTail_ang_req    = tail_ang_req;
  mYawrate         = yawrate;
  mTail_stand_mode = tail_stand_mode;
  mTail_lug_mode   = tail_lug_mode;

}
/** not completed **/
void Ang_Robo::run_anago_run() {
  int16_t angle         = mGyroSensor.getAnglerVelocity();  // ジャイロセンサ値
  int     rightWheelEnc = mRightWheel.getCount();            // 右モータ回転角度
  int     leftWheelEnc  = mLeftWheel.getCount();             // 左モータ回転角度
  int     battery       = ev3_battery_voltage_mV();

  static float   ref_tail_angle;
  static int32_t clock_start;

  switch(Anago_Mode){

  case Ang_Balance:

    ref_tail_angle  =  TAIL_ANGLE_RUN;
    balance_off_en  = false;
    pre_balancer_on = false;
    balance_mode    = true; 
    ref_tail_angle =  TAIL_ANGLE_RUN;

    mTurn = YawrateController(mYawrate, mYawratecmd);

    /*
    mBalancer->setCommand(mForward, mTurn);
    mBalancer->update(angle, rightWheelEnc, leftWheelEnc, battery);

    mLeftWheel.setPWM(mBalancer->getPwmLeft());
    mRightWheel.setPWM(mBalancer->getPwmRight());

    tail_control(ref_tail_angle);
    */
    break;

  case Tail_Down:
    mForward = 0;
    mTurn    = 0;
    
    if(ref_tail_angle <= TAIL_ANGLE_DANSA){
      ref_tail_angle = ref_tail_angle + 0.5;
    }
    if(mTail_Motor.getCount() >= TAIL_ANGLE_DANSA){
      //      Stand_Mode = Tail_On;
      clock_start = robo_Clock->now();
    }
    tail_control(ref_tail_angle);
    balance_off_en = false;
    break;

  default:
    mForward       = 0;
    mTurn          = 0;
    balance_off_en = false;
    break;
  }

  if(balance_mode == true){
    mBalancer->setCommand(mForward, mTurn);
    mBalancer->update(angle, rightWheelEnc, leftWheelEnc, battery);

    mLeftWheel.setPWM(mBalancer->getPwmLeft());
    mRightWheel.setPWM(mBalancer->getPwmRight());

    tail_control(ref_tail_angle);
  }

}

void Ang_Robo::run() {
    int16_t angle         = mGyroSensor.getAnglerVelocity();  // ジャイロセンサ値
    int     rightWheelEnc = mRightWheel.getCount();            // 右モータ回転角度
    int     leftWheelEnc  = mLeftWheel.getCount();             // 左モータ回転角度

	//アクティブヨーレート();
    mTurn = YawrateController(mYawrate, mYawratecmd);

    if(mTail_stand_mode == true){
      tail_stand_from_balance();
    }else if((mTail_stand_mode == false) && (Stand_Mode != Balance_Mode)){
      if(Stand_Mode == Tail_Stand){
	Stand_Mode = Stand_Vert;
      }
      tail_stand_from_balance();

    }else{
      tail_control(mTail_ang_req);
      balance_off_en = false;
    }

    int battery = ev3_battery_voltage_mV();
    
    if((Stand_Mode == Stand_to_Balance)&&(log_left_pwm < -10)){
      mBalancer->setCommand(mForward, mTurn);
      mBalancer->update(10, rightWheelEnc, leftWheelEnc, battery);
      
    }else if((Stand_Mode == Stand_to_Balance)&&(log_left_pwm > 10)){
      mBalancer->setCommand(mForward, mTurn);
      mBalancer->update(-10, rightWheelEnc, leftWheelEnc, battery);
    }else{
      mBalancer->setCommand(mForward, mTurn);
      mBalancer->update(angle, rightWheelEnc, leftWheelEnc, battery);
    }

    log_forward         = mForward;
    log_turn            = mTurn;
    log_gyro            = angle;
    log_left_wheel_enc  = leftWheelEnc;
    log_right_wheel_enc = rightWheelEnc;
    log_battery         = battery;
    log_left_pwm        = mBalancer->getPwmLeft();
    log_right_pwm       = mBalancer->getPwmRight();

    if((balance_off_en == true) && (mTail_Motor.getCount() >  70)){
      TailMode(mForward, mTurn);
      mLeftWheel.setPWM(mtail_mode_pwm_l);
      mRightWheel.setPWM(mtail_mode_pwm_r);
      balance_mode = false; 
      
    }else{
      mLeftWheel.setPWM(mBalancer->getPwmLeft());
      mRightWheel.setPWM(mBalancer->getPwmRight());
      balance_mode = true; 
    }
}


//2017.07.28 k-ota copy from 3-apex
//*****************************************************************************
// 関数名 : tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
void Ang_Robo::tail_control(signed int angle)
{

  //  tail_motor_pwm = (float)(angle - mTail_Motor.getCount()*P_GAIN); /* 比例制御 */
  tail_motor_pwm = gTail_pwm->calc_pid(angle, mTail_Motor.getCount());
  tail_motor_pwm = tail_motor_pwm*0.1;
  /* PWM出力飽和処理 */
  if (tail_motor_pwm > PWM_ABS_MAX)
    {
      tail_motor_pwm = PWM_ABS_MAX;
    }
  else if (tail_motor_pwm < -PWM_ABS_MAX)
    {
      tail_motor_pwm = -PWM_ABS_MAX;
    }

  if (tail_motor_pwm == 0)
    {
      //17.07.28 kota modify//        ev3_motor_stop(tail_motor, true);
      mTail_Motor.stop();
    }
  else
    {
      //17.07.28 kota modify//        ev3_motor_set_power(tail_motor, (signed char)pwm);
      mTail_Motor.setPWM((signed int)tail_motor_pwm);
    }


}

//170816 ota add tail control

void Ang_Robo::tail_reset(){
  int32_t angle    = 0;
  int32_t angle_1d = 0;

  mTail_Motor.setPWM(-10);
  angle = 0;
  angle_1d = 1;

  while(1){
    if(angle == angle_1d){
      mTail_Motor.stop();
      mTail_Motor.reset();
      break;
    }
    else{
      angle_1d = angle;
      tslp_tsk(1000);
      angle = mTail_Motor.getCount();
    }
  }
  mTail_Motor.stop();
  mTail_Motor.reset();
}

void Ang_Robo::tail_stand_up(){
    while(1){
      if(mTail_Motor.getCount() == TAIL_ANGLE_STAND_UP){
	mTail_Motor.stop();
	break;
      }
      else{
	mTail_Motor.setPWM(5);
      }
    }
    mTail_Motor.stop();
} //tail for gyro reset and color sensor calibration


void Ang_Robo::tail_stand_from_balance(){
  static float   target_tail_angle;
  static int32_t clock_start;

  switch(Stand_Mode){
  case Balance_Mode:
    mForward = 0;
    mTurn = 0;
    target_tail_angle =  TAIL_ANGLE_RUN;
    Stand_Mode = Tail_Down;
    balance_off_en = false;
    pre_balancer_on = false;
    break;

  case Tail_Down:
    mForward = 0;
    mTurn = 0;
    if(target_tail_angle <= TAIL_ANGLE_DANSA){
      target_tail_angle = target_tail_angle + 0.5;
    }
    if(mTail_Motor.getCount() >= TAIL_ANGLE_DANSA){
      Stand_Mode = Tail_On;
      clock_start = robo_Clock->now();
    }
    tail_control(target_tail_angle);
    balance_off_en = false;
    break;

  case Tail_On:
    mForward = 0;
    mTurn = 0;
    if((robo_Clock->now() - clock_start) < 3500){
      mForward = 0;
      mTurn = 0;
      balance_off_en = false;
    }
    else if((robo_Clock->now() - clock_start) > 5000){
      mForward = 0;
      mTurn = 0;
      balance_off_en = true;
      Stand_Mode = Tail_Stand;
      clock_start = robo_Clock->now();
    }else{
      mForward = -20;
      mTurn = 0;
      balance_off_en = false;
    }
    break;

  case Tail_Stand:
    if((robo_Clock->now() - clock_start) < 500){
      mForward = 0;
      mTurn = 0;
      balance_off_en = true;
    }else{
      

      balance_off_en = true;
    }
    break;

  case Stand_Vert:
    mForward = 0;
    mTurn    = 0; 
    balance_off_en = true;
    if(mTail_Motor.getCount() >= 95){
      tail_control(96);
      clock_start = robo_Clock->now();
      Stand_Mode = Stand_to_Balance;
    }

    if(mTail_Motor.getCount() < 96){
      target_tail_angle = target_tail_angle + 0.02;
      tail_control(target_tail_angle);
    }else{
      tail_control(96);
      clock_start = robo_Clock->now();
      Stand_Mode = Stand_to_Balance;
    }
    break;
      
  case Stand_to_Balance:
    mForward = 0;
    mTurn    = 0; 
    balance_off_en = true;
    tail_control(96);

    if((robo_Clock->now() - clock_start) > 1000){
      //    if((log_left_pwm >= -20) && (log_left_pwm <= 20) && ((robo_Clock->now() - clock_start) > 1000)){
      if((log_left_pwm >= -10) && (log_left_pwm <= 10)){
	mForward       = 0;
	mTurn          = 0; 
	//	balance_off_en = false;
	balance_off_en = true;
	Stand_Mode     = Tail_for_Run;
	tail_control(96);
      }
    }
    break;

  case Tail_for_Run:
    mForward = 0;
    mTurn    = 0; 
    tail_control(98);
    balance_off_en = true;

    if(mTail_Motor.getCount() >=  97){
      Stand_Mode     = Balance_Mode;
      balance_off_en = false;
    }
    break;



  default:
    mForward = 0;
    mTurn = 0;
    balance_off_en = false;
    break;
  }
}



//2017/08/06多田さんヨーレートコントローラー
float Ang_Robo::YawrateController(float yawrate, float yawrate_cmd)
{
	r_yaw_rate = yawrate_cmd*I_gain1 - r_yaw_rate_ud;
	r_yaw_rate_ud =(r_yaw_rate*I_gain2 + r_yaw_rate_ud*I_gain3);
	F_out = F_controller((float)r_yaw_rate);
	E_out = E_controller((float)r_yaw_rate);
	C_out = C_controller(E_out, (float)yawrate, S_out);
	S_out = S_controller(C_out);
	turn_tmp = F_out + C_out;
	if(turn_tmp > 100) turn_tmp = 100;
	if(turn_tmp < -100) turn_tmp = -100;


	return turn_tmp;//制御出力

}

float Ang_Robo::F_controller(float r_yaw_rate)
{
	F_in = r_yaw_rate;

	F_out = F_in * F_gain;

	return F_out;
}

float Ang_Robo::E_controller(float r_yaw_rate)
{
	E_in_dddddd = E_in_ddddd;
	E_in_ddddd = E_in_dddd;
	E_in_dddd = E_in_ddd;
	E_in_ddd = E_in_dd;
	E_in_dd = E_in_d;
	E_in_d = E_in;
	E_in = (r_yaw_rate);
	E_out = E_ud1;
	E_ud1 = (E_in_dddddd * E_gain1) + E_ud1 * E_gain2;

	return E_out;
}

float Ang_Robo::C_controller(float E_out, float yawrate, float S_out)
{
//	C_in = (E_out - yawrate - S_out);
	C_in = (E_out + yawrate - S_out);//0816
//	C_in = (E_out - yawrate);
	C_out = C_ud1;
	C_ud1 = C_in * C_gain + (C_out * 1.0);


	return C_out;
}

float Ang_Robo::S_controller(float C_out)
{
	S_in_dddddd = S_in_ddddd;
	S_in_ddddd = S_in_dddd;
	S_in_dddd = S_in_ddd;
	S_in_ddd = S_in_dd;
	S_in_dd = S_in_d;
	S_in_d = S_in;
	S_in = C_out*S_gain1;
	S_out = Pn_ud1 - Pd_ud1;
	Pn_ud1 = S_in*Pn_gain1 + Pn_ud1*Pn_gain2;
	Pd_ud1 = S_in_dddddd*Pd_gain1 + Pd_ud1*Pd_gain2;

	return S_out;
}

void Ang_Robo::TailMode(int mForward, float mTurn){
	mtail_mode_pwm_l = 0.5*mForward + 1.0*mTurn ;
	mtail_mode_pwm_r = 0.5*mForward - 1.0*mTurn;
}

