#include "ang_eye.h"
#include "math.h"

//#define DEBUG_ODOMETRY
#define DEBUG_EYE_DEBUG
//#define DEBUG_NANSYO

// 定数宣言
const int8_t Ang_Eye::INITIAL_WHITE_THRESHOLD = 40;  // 黒色の光センサ値
const int8_t Ang_Eye::INITIAL_BLACK_THRESHOLD = 20;  // 黒色の光センサ値

/**
 * コンストラクタ
 * @param colorSensor カラーセンサ
 */
Ang_Eye::Ang_Eye(const ev3api::ColorSensor& colorSensor,
		 ev3api::Motor& leftWheel,
		 ev3api::Motor& rightWheel,
		 ev3api::GyroSensor& gyro)
  : 
    mColorSensor(colorSensor),
    mLeftWheel(leftWheel),
    mRightWheel(rightWheel),
    mGyro(gyro),

    mWhite(INITIAL_WHITE_THRESHOLD),
    mBlack(INITIAL_BLACK_THRESHOLD),
    mWhite_slant(INITIAL_WHITE_THRESHOLD),
    mBlack_slant(INITIAL_BLACK_THRESHOLD)
{
}

void Ang_Eye::init(){
  mGyro.reset();
  enum_Mode = CALIB_ANGLE;
#ifdef DEBUG_NANSYO
  enum_Mode = DET_MOVEMENT;
#endif
  robo_stop       = 1;
  robo_forward    = 0;
  robo_back       = 0;
  robo_turn_left  = 0;
  robo_turn_right = 0;

}


void Ang_Eye::set_White_Black_Threshold(int8_t white, int8_t black, int8_t white_slant, int8_t black_slant) {
  mWhite       = white;
  mBlack       = black;
  mWhite_slant = white_slant;
  mBlack_slant = black_slant;
}

void Ang_Eye::det_Line_Value() {

  float k;
  float adj_brightness;

  k= 100.0/(mWhite-mBlack);
  adj_brightness = k*(mColorSensor.getBrightness()-mBlack);

  if(adj_brightness < 0){
    adj_brightness = 0;
  }
  else if(adj_brightness > 100){
    adj_brightness = 100;
  }
  linevalue = 100-adj_brightness;
}


void Ang_Eye::WheelOdometry(float dT) {
  static float odo_prev;
  static float velocity_input;
  static float velocity_prev;
  //LPF 10[rad/s]/////////////////////////////////////
  static float Alpfd = 0.9391; // LPF
  static float Blpfd = 1; // LPF
  static float Clpfd = 0.0609; // LPF
  static float Dlpfd = 0; // LPF
  //////////////////////////////////////////
  static float old_rel_angle;     //過去のYaw角[rad]
  int   WheelAngRdeg = mRightWheel.getCount();  //右モータ回転角度[deg]
  int   WheelAngLdeg = mLeftWheel.getCount();   //右モータ回転角度[deg]
  int i;

  odo            = ((float)WheelAngLdeg + (float)WheelAngRdeg)/2.0 * RAD_1_DEG * WHEEL_R; //[mm]
  velocity_input = (odo - odo_prev)/dT;
  velocity       = Clpfd * velocity_prev + Dlpfd * velocity_input;
  velocity_prev  = Alpfd * velocity_prev + Blpfd * velocity_input;
  
//  xvalue = xvalue+(-1)*(odo-odo_prev)*sin(yawangle);
//  yvalue = yvalue+(odo-odo_prev)*cos(yawangle);    

  relative_angle =  ((float)WheelAngRdeg - (float)WheelAngLdeg) * RAD_1_DEG * WHEEL_R / RoboTread; //ロボのYaw角[rad]
  relative_angle = relative_angle + correction_angle;
//  abs_angle      = relative_angle + RAD_90_DEG + correction_angle;
  abs_angle      = relative_angle * 1.0 + RAD_90_DEG + correction_angle;
  xvalue = xvalue+(odo-odo_prev)*cos(abs_angle); //0902 tada
  yvalue = yvalue+(odo-odo_prev)*sin(abs_angle); //0902 tada

#ifdef DEBUG_NANSYO
  abs_angle      = relative_angle;
#endif
  yawrate  =(relative_angle-old_rel_angle)/dT;           //ロボのYawレート[rad/s]

  old_rel_angle=relative_angle;         //過去のYaw角[rad]
  odo_prev = odo;

  switch(enum_Mode){
  case CALIB_ANGLE:  
    robo_stop       = 0;
    robo_forward    = 1;
    robo_back       = 0;
    robo_turn_left  = 0;
    robo_turn_right = 0;
    //Correct absolute angle
    if(odo > 500 && odo <= 1500 && cap_cnt < cap_size){
      //cap_dat[cap_cnt] = abs_angle;
      angle_sum_dat = angle_sum_dat + abs_angle;
      cap_cnt++;
    }else if(odo > 1500 && odo <= 1505){
      angle_ave_dat = angle_sum_dat/cap_cnt;
      correction_angle = RAD_90_DEG - angle_ave_dat;
//      xvalue = 0.0;
    	xvalue = 735.96;
    	yvalue = 415.74 + odo * sin(angle_ave_dat);//0910 tada

      enum_Mode = DET_MOVEMENT;
      angle_sum_dat = 0;
      cap_cnt = 0;
    }else{
      cap_cnt = 0;
      angle_sum_dat = 0.0;
      angle_ave_dat = 0.0;
    }
    break;

  case DET_MOVEMENT:

    if(cap_cnt == 125){
      cap_cnt = 0;
    }
    angle_dat_500ms[cap_cnt] = abs_angle;
    velocity_dat_500ms[cap_cnt] = velocity;
    cap_cnt++;

    angle_sum_dat = 0;    
    velocity_sum_dat = 0;
    for (i=0; i<125; i++){
      angle_sum_dat    = angle_sum_dat    + angle_dat_500ms[i];
      velocity_sum_dat = velocity_sum_dat + velocity_dat_500ms[i];
    }
    angle_ave_dat    = angle_sum_dat/125;
    velocity_ave_dat = velocity_sum_dat/125;

    dif_angle_ave_dat    = angle_ave_dat - old_angle_ave_dat;
    dif_velocity_ave_dat = velocity_ave_dat - old_velocity_ave_dat;

    old_angle_ave_dat    = angle_ave_dat;
    old_velocity_ave_dat = velocity_ave_dat;

    if (dif_angle_ave_dat > -0.001 && dif_angle_ave_dat < 0.001){

      if(velocity_ave_dat > 0){
	robo_stop       = 0;
	robo_forward    = 1;
	robo_back       = 0;
	robo_turn_left  = 0;
	robo_turn_right = 0;
      }else if (velocity_ave_dat < 0){
	robo_stop       = 0;
	robo_forward    = 0;
	robo_back       = 1;
	robo_turn_left  = 0;
	robo_turn_right = 0;
      }else{
	robo_stop       = 1;
	robo_forward    = 0;
	robo_back       = 0;
	robo_turn_left  = 0;
	robo_turn_right = 0;
      }

    }else if(dif_angle_ave_dat >= 0.001){
      robo_stop       = 0;
      robo_forward    = 0;
      robo_back       = 0;
      robo_turn_left  = 1;
      robo_turn_right = 0;
    }else if(dif_angle_ave_dat <= -0.001){
      robo_stop       = 0;
      robo_forward    = 0;
      robo_back       = 0;
      robo_turn_left  = 0;
      robo_turn_right = 1;
    }else {
      robo_stop       = 0;
      robo_forward    = 0;
      robo_back       = 0;
      robo_turn_left  = 0;
      robo_turn_right = 0;
    }
    
break;

  default:
    break;
  }

#ifdef DEBUG_EYE_DEBUG
  saveData( );
#endif

}

//170815
void Ang_Eye::det_Dansa( ) {
  static int gyro_250d[250];
  int cnt;
  gyro_250d[0] = mGyro.getAnglerVelocity();

  for(cnt =9; cnt > 0; cnt--){
    gyro_250d[cnt] = gyro_250d[cnt-1];
  }
 
  for(cnt = 0; cnt < 250; cnt++){
    if(gyro_250d[cnt] < -100 || gyro_250d[cnt] > 100){
      dansa = 1;
      cnt = 250;
    }else{
      dansa = 0;
    }
  }
}


#ifdef DEBUG_EYE_DEBUG

void Ang_Eye::saveData( ){

  log_dat_00[log_cnt]  = robo_forward;
  log_dat_01[log_cnt]  = robo_turn_left;
  log_dat_02[log_cnt]  = robo_turn_right;
  log_fdat_00[log_cnt] = velocity;
  log_fdat_01[log_cnt] = abs_angle;
  log_fdat_02[log_cnt] = angle_ave_dat;

  log_cnt++;
  if (log_cnt == log_size){
    log_cnt  = 0;
//    stop_sys = 1;
  }
}

void Ang_Eye::export_dat( ){
    FILE* file_id;
    file_id = fopen( "eye_dat.csv" ,"w");
    fprintf(file_id, "forward,left,right,velocity,angle,angle_ave_dat\n");
    int cnt;

    for(cnt = 0; cnt < log_size ; cnt++){
      fprintf(file_id, "%d,%d,%d,%f,%f,%f\n",log_dat_00[cnt],log_dat_01[cnt], log_dat_02[cnt],log_fdat_00[cnt],log_fdat_01[cnt], log_fdat_02[cnt]);
    }
    fclose(file_id);

}

#endif
