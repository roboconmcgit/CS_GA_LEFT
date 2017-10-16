/******************************************************************************
 *  ang_brain.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2017/07/29
 *  Implementation of the Class RandomWalker
 *  Author: Keiichi Tomii
 *****************************************************************************/

#include "Strategy_Det.h"
#include "math.h"
//#define LINETRACE_MODE
//#define STEP_DEBUG

StrategyDet::StrategyDet(){

}

void StrategyDet::Det_run(float x_value, float y_value, float yawangle){

	x_value = x_value + 50.0*cos(yawangle);// 0902 tada
	y_value = y_value + 50.0*sin(yawangle);// 0902 tada
	RunningStrategyDet(x_value,y_value, yawangle); // 0828 tada
	VirtualGateDet(x_value,y_value,yawangle); // 0828 tada

}

void StrategyDet::RunningStrategyDet(float x_value, float y_value, float yawangle){

  if (Robo_Area_Estimator(StartArea[0],StartArea[1],StartArea[2],StartArea[3],x_value,y_value,yawangle)){
    Strategy=LineTrace1;
    Max_Forward = 70;
    Max_Yawrate = 1.5;
    Min_Yawrate = -1.5;
  }
  /*
  else if (Robo_Area_Estimator(First_Straight[0],First_Straight[1],First_Straight[2],First_Straight[3],x_value,y_value,yawangle)){
    Strategy=LineTrace1;
    Max_Forward = 100;
    Max_Yawrate = 1.5;
    Min_Yawrate = -1.5;
  }

  else if (Robo_Area_Estimator(First_Corner[0],First_Corner[1],First_Corner[2],First_Corner[3],x_value,y_value,yawangle)){
    Strategy=LineTrace1;
    Max_Forward = 100;
    Max_Yawrate = 5;
    Min_Yawrate = -5;
  }
  else if (Robo_Area_Estimator(Second_Corner[0],Second_Corner[1],Second_Corner[2],Second_Corner[3],x_value,y_value,yawangle)){
    Strategy=LineTrace1;
    Max_Forward = 100;
    Max_Yawrate = 5;
    Min_Yawrate = -5;
  }
  else if (Robo_Area_Estimator(Second_Straight[0],Second_Straight[1],Second_Straight[2],Second_Straight[3],x_value,y_value,yawangle)){
    Strategy=LineTrace1;
    Max_Forward = 100;
    Max_Yawrate = 1.5;
    Min_Yawrate = -1.5;
  }
  */
  /*
  else if(Robo_Area_Estimator(GoalArea[0],GoalArea[1],GoalArea[2],GoalArea[3],x_value,y_value,yawangle)){
    //Strategy=Goal;
    Strategy=LineTrace1;
    Max_Forward =   70;
    Max_Yawrate =  5.0;
    Min_Yawrate =  -5.0;
  }

  else if(Robo_Area_Estimator(Goal_to_Step[0],Goal_to_Step[1],Goal_to_Step[2],Goal_to_Step[3],x_value,y_value,yawangle)){
    Strategy=Goal2Step;
    Max_Forward =   40;
    Max_Yawrate =  5.0;
    Min_Yawrate =  -5.0;
  }
*/

  else if(Robo_Area_Estimator(LineTrace1Area[0],LineTrace1Area[1],LineTrace1Area[2],LineTrace1Area[3],x_value,y_value,yawangle)){
    Strategy=LineTrace1;
    Max_Forward = 100;
    Max_Yawrate = 3.0;
    Min_Yawrate = -3.0;
  }
  else if(Robo_Area_Estimator(MapTraceArea1[0],MapTraceArea1[1],MapTraceArea1[2],MapTraceArea1[3],x_value,y_value,  yawangle)){
    Strategy=MapTrace1;
    Max_Forward = 100;
    Max_Yawrate = 3.0;
    Min_Yawrate = -3.0;
  }
  else if(Robo_Area_Estimator(MapTraceArea2[0],MapTraceArea2[1],MapTraceArea2[2],MapTraceArea2[3],x_value,y_value,  yawangle)){
    Strategy=MapTrace2;
    Max_Forward = 100;
    Max_Yawrate = 3.0;
    Min_Yawrate = -3.0;
  }
  else if(Robo_Area_Estimator(MapTraceArea3[0],MapTraceArea3[1],MapTraceArea3[2],MapTraceArea3[3],x_value,y_value,  yawangle)){
    Strategy=MapTrace3;
    Max_Forward = 100;
    Max_Yawrate = 3.0;
    Min_Yawrate = -3.0;
  }
  else if(Robo_Area_Estimator(MapTraceArea4[0],MapTraceArea4[1],MapTraceArea4[2],MapTraceArea4[3],x_value,y_value,  yawangle)){
    Strategy=MapTrace4;
    Max_Forward = 100;
    Max_Yawrate = 3.0;
    Min_Yawrate = -3.0;
  }
  else if(Robo_Area_Estimator(MapTraceArea5[0],MapTraceArea5[1],MapTraceArea5[2],MapTraceArea5[3],x_value,y_value,  yawangle)){
    Strategy=MapTrace5;
    Max_Forward = 100;
    Max_Yawrate = 3.0;
    Min_Yawrate = -3.0;
  }
  else if(Robo_Area_Estimator(MapTraceArea6[0],MapTraceArea6[1],MapTraceArea6[2],MapTraceArea6[3],x_value,y_value,  yawangle)){
    Strategy=MapTrace6;
    Max_Forward = 100;
    Max_Yawrate = 3.0;
    Min_Yawrate = -3.0;
  }
  else if(Robo_Area_Estimator(MapTraceArea7[0],MapTraceArea7[1],MapTraceArea7[2],MapTraceArea7[3],x_value,y_value,  yawangle)){
    Strategy=MapTrace7;
    Max_Forward = 100;
    Max_Yawrate = 3.0;
    Min_Yawrate = -3.0;
  }
  else if(Robo_Area_Estimator(MapTraceArea8[0],MapTraceArea8[1],MapTraceArea8[2],MapTraceArea8[3],x_value,y_value,  yawangle)){
    Strategy=MapTrace8;
    Max_Forward = 100;
    Max_Yawrate = 0.20;
    Min_Yawrate = -0.20;
  }
/*
  else if(Robo_Area_Estimator(StepArea[0],StepArea[1],StepArea[2],StepArea[3],x_value,y_value,yawangle)){
    Strategy=Step;
  }
  else if(Robo_Area_Estimator(LookUpGateArea[0],LookUpGateArea[1],LookUpGateArea[2],LookUpGateArea[3],x_value,y_value,yawangle)){
    Strategy=LookUpGate;
  }
  else if(Robo_Area_Estimator(GarageArea[0],GarageArea[1],GarageArea[2],GarageArea[3],x_value,y_value,yawangle)){
    Strategy=Garage;
  }
  else if(Robo_Area_Estimator(StopArea[0],StopArea[1],StopArea[2],StopArea[3],x_value,y_value,yawangle)){
    Strategy=Stop;
  }
  */
  else{
//    Strategy=LineTrace1;
    Strategy = Goal;
    Max_Forward =   50;
    Max_Yawrate =  3.0;
    Min_Yawrate = -3.0;
  }

#ifdef LINETRACE_MODE
  Strategy=LineTrace1;
#endif

#ifdef STEP_DEBUG
  Strategy=Step;
#endif

  StrategyNum=static_cast<int>(Strategy);

}
void StrategyDet::VirtualGateDet(float x_value, float y_value, float yawangle){

	if (Robo_Area_Estimator(Gate12Area[0],Gate12Area[1],Gate12Area[2],Gate12Area[3],x_value,y_value,  yawangle)){
		VirtualGate=Gate12;
	}
	else if(Robo_Area_Estimator(Gate23Area[0],Gate23Area[1],Gate23Area[2],Gate23Area[3],x_value,y_value,  yawangle)){
		VirtualGate=Gate23;
	}
	else if(Robo_Area_Estimator(Gate34Area[0],Gate34Area[1],Gate34Area[2],Gate34Area[3],x_value,y_value,  yawangle)){
		VirtualGate=Gate34;
	}
	else if(Robo_Area_Estimator(Gate45Area[0],Gate45Area[1],Gate45Area[2],Gate45Area[3],x_value,y_value,  yawangle)){
		VirtualGate=Gate45;
	}
	else if(Robo_Area_Estimator(Gate56Area[0],Gate56Area[1],Gate56Area[2],Gate56Area[3],x_value,y_value,  yawangle)){
		VirtualGate=Gate56;
	}
	else if(Robo_Area_Estimator(Gate67Area[0],Gate67Area[1],Gate67Area[2],Gate67Area[3],x_value,y_value,  yawangle)){
		VirtualGate=Gate67;
	}
	else if(Robo_Area_Estimator(Gate78Area[0],Gate78Area[1],Gate78Area[2],Gate78Area[3],x_value,y_value,  yawangle)){
		VirtualGate=Gate78;
	}
	else if(Robo_Area_Estimator(Gate89Area[0],Gate89Area[1],Gate89Area[2],Gate89Area[3],x_value,y_value,  yawangle)){
		VirtualGate=Gate89;
	}else{
		VirtualGate=None;
	}// 0827 tada

	#ifdef LINETRACE_MODE
	VirtualGate=None;
	#endif

	VirtualGateNum=static_cast<int>(VirtualGate);
}


//bool StrategyDet::Robo_Area_Estimator(float x_left, float x_right, float y_top, float y_under, float x_value, float y_value){
bool StrategyDet::Robo_Area_Estimator(float x_left, float x_right, float y_under, float y_top, float x_value, float y_value, float yawangle){  
  if(x_left < x_value && x_value <= x_right && y_under < y_value && y_value <= y_top){
    return true;
  }else{
    return false;
  }

}

