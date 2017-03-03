///////////////////////////////////////////
// MSU Borealis Payload Motion Simulator //
// Created by Jared Kamp 12/29/2016      //
// Version 2.0.10                         //
// Last Edited by Jared Kamp 3/2/2017   //
///////////////////////////////////////////
// + Version 2 uses hours, minutes, and seconds
// which were previously unavailable in earlier
// IMU measurement data files.
// + The number of measurements per unit time
// has also been increased from 12 measurements
// per minute to 1 measurement per 52 ms **approximately
// + 2.0.3 - Code added for calculating degrees of rotation.
// + 2.0.4 - Code added for counting number of times the packages tipped to one side or another 45 degrees or more.
// + 2.0.5 - Updated serial print to track accleration for identifying the moment ascent changes to descent.
// + 2.0.6 - Added functionality for counting degrees of pitch and roll and eliminated the 3D objects interdependance. All objects move completely independantly now.
// + 2.0.7 - Divided the yaw rotation into three parts; clockwise, counterclockwise, and combined. This done to illustrate rocking (oscillations) vs. spinning (circular).
// + 2.0.8 - Rewrote the yaw rotation calculation to improve accuracy. Added tracking of positive and negative pitch and roll.
// + 2.0.9 - Changed the file extension for IMU data from txt to csv. Moved the file path strings to an easier location to view/edit. Rewrote the pitch and roll calculations.
// + 2.0.10 - Added backwards compatibility with data from Turbulence Datalogger version 1.

import java.awt.datatransfer.*;
import java.awt.Toolkit;
import processing.opengl.*;
import saito.objloader.*;
import g4p_controls.*;
import java.io.FileReader;
import java.io.FileNotFoundException;

//File Paths
//White Box FPs
String IRD_Data_FP = "C:/Users/Jared/Desktop/MSGC/IMU/Processing/payloadSpin/data/NovTwoBalloon/White Box/ascent_IMUIRD00.CSV";
String STL_Data_FP = "C:/Users/Jared/Desktop/MSGC/IMU/Processing/payloadSpin/data/NovTwoBalloon/White Box/ascent_IMUSTL01.CSV";
String VID_Data_FP = "C:/Users/Jared/Desktop/MSGC/IMU/Processing/payloadSpin/data/NovTwoBalloon/White Box/ascent_IMUVID01.CSV";
//Red Box FPs
//String IRD_Data_FP = "C:/Users/Jared/Desktop/MSGC/IMU/Processing/payloadSpin/data/NovTwoBalloon/Red Box/ascent_IMUIRD00.CSV";
//String STL_Data_FP = "C:/Users/Jared/Desktop/MSGC/IMU/Processing/payloadSpin/data/NovTwoBalloon/Red Box/ascent_IMUSTL02.CSV";
//String VID_Data_FP = "";//No video package for red payload.
//Testing FPs
//String IRD_Data_FP = "C:/Users/Jared/Desktop/MSGC/IMU/Processing/payloadSpin/Data/Test/PitchPlus90_Minus90/IMUGLD02.CSV";
//String STL_Data_FP = "C:/Users/Jared/Desktop/MSGC/IMU/Processing/payloadSpin/Data/Test/PitchPlus90_Minus90/IMUWHT02.CSV";
//String VID_Data_FP = "C:/Users/Jared/Desktop/MSGC/IMU/Processing/payloadSpin/Data/Test/PitchPlus90_Minus90/IMUBLU02.CSV";
String White_Model_FP = "C:/Users/Jared/Desktop/MSGC/IMU/Processing/payloadSpin/data/whitebox.obj";
String Red_Model_FP = "C:/Users/Jared/Desktop/MSGC/IMU/Processing/payloadSpin/data/redbox.obj";
String Model_FP;

//TurbulenceDatalogger Version
int turbulenceVer = 1;

//IMU sensitivity threshold adjustments
int yawSensitivity = 0;
int pitchSensitivity = 5;
int rollSensitivity = 5;

//Red Payload vs. White Payload & Variable Ascent Rates (added in 2.0.10)
boolean isRedPayload = false;
int first_5k_increments;
int loopsPer5k;

//CSV files to ArrayLists
ArrayList IRD_sensorData;
ArrayList IRD_BattVolt;
ArrayList IRD_Temp;
ArrayList IRD_AcclX;
ArrayList IRD_AcclY;
ArrayList IRD_AcclZ;
ArrayList IRD_DegX;
ArrayList IRD_DegY;
ArrayList IRD_DegZ;
ArrayList IRD_OrientX;
ArrayList IRD_OrientY;
ArrayList IRD_OrientZ;
ArrayList IRD_SysCalib;
ArrayList IRD_GyroCalib;
ArrayList IRD_AcclCalib;
ArrayList IRD_MagCalib;
ArrayList IRD_Time;
ArrayList IRD_Time_Hr;
ArrayList IRD_Time_Min;
ArrayList IRD_Time_Sec;

ArrayList STL_sensorData;
ArrayList STL_BattVolt;
ArrayList STL_Temp;
ArrayList STL_AcclX;
ArrayList STL_AcclY;
ArrayList STL_AcclZ;
ArrayList STL_DegX;
ArrayList STL_DegY;
ArrayList STL_DegZ;
ArrayList STL_OrientX;
ArrayList STL_OrientY;
ArrayList STL_OrientZ;
ArrayList STL_SysCalib;
ArrayList STL_GyroCalib;
ArrayList STL_AcclCalib;
ArrayList STL_MagCalib;
ArrayList STL_Time;
ArrayList STL_Time_Hr;
ArrayList STL_Time_Min;
ArrayList STL_Time_Sec;

ArrayList VID_sensorData;
ArrayList VID_BattVolt;
ArrayList VID_Temp;
ArrayList VID_AcclX;
ArrayList VID_AcclY;
ArrayList VID_AcclZ;
ArrayList VID_DegX;
ArrayList VID_DegY;
ArrayList VID_DegZ;
ArrayList VID_OrientX;
ArrayList VID_OrientY;
ArrayList VID_OrientZ;
ArrayList VID_SysCalib;
ArrayList VID_GyroCalib;
ArrayList VID_AcclCalib;
ArrayList VID_MagCalib;
ArrayList VID_Time;
ArrayList VID_Time_Hr;
ArrayList VID_Time_Min;
ArrayList VID_Time_Sec;

//IMU data variables
float roll  = 0.0F;
float pitch = 0.0F;
float yaw   = 0.0F;
float temp  = 0.0F;
float alt   = 0.0F;

//Iridium package variables
int totalClockwiseDegYaw_IRD = 0;
int totalCounterClockwiseDegYaw_IRD = 0;
int totalDegYaw_IRD = 0;
float prevYaw_IRD_1;
float prevYaw_IRD_2;
float prevYaw_IRD_3;
int totalNegRoll_IRD = 0;
int totalPosRoll_IRD = 0;
int totalDegRoll_IRD = 0;
float prevRoll_IRD_1;
float prevRoll_IRD_2;
float prevRoll_IRD_3;
int totalNegPitch_IRD = 0;
int totalPosPitch_IRD = 0;
int totalDegPitch_IRD = 0;
float prevPitch_IRD_1;
float prevPitch_IRD_2;
float prevPitch_IRD_3;

//Still Image package variables
int totalClockwiseDegYaw_STL = 0;
int totalCounterClockwiseDegYaw_STL = 0;
int totalDegYaw_STL = 0;
float prevYaw_STL_1;
float prevYaw_STL_2;
float prevYaw_STL_3;
int totalNegRoll_STL = 0;
int totalPosRoll_STL = 0;
int totalDegRoll_STL = 0;
float prevRoll_STL_1;
float prevRoll_STL_2;
float prevRoll_STL_3;
int totalNegPitch_STL = 0;
int totalPosPitch_STL = 0;
int totalDegPitch_STL = 0;
float prevPitch_STL_1;
float prevPitch_STL_2;
float prevPitch_STL_3;

//Video package variables
int totalClockwiseDegYaw_VID = 0;
int totalCounterClockwiseDegYaw_VID = 0;
int totalDegYaw_VID = 0;
float prevYaw_VID_1;
float prevYaw_VID_2;
float prevYaw_VID_3;
int totalNegRoll_VID = 0;
int totalPosRoll_VID = 0;
int totalDegRoll_VID = 0;
float prevRoll_VID_1;
float prevRoll_VID_2;
float prevRoll_VID_3;
int totalNegPitch_VID = 0;
int totalPosPitch_VID = 0;
int totalDegPitch_VID = 0;
float prevPitch_VID_1;
float prevPitch_VID_2;
float prevPitch_VID_3;

//Time-based altitude tracking variables for printing summations
int lineNum = 0;
int lineCount = 0;
int markCount = 0;
int lastYawClockwise_IRD = 0;
int lastYawCounterClockwise_IRD = 0;
int lastYawCombined_IRD = 0;
int lastPosPitch_IRD = 0;
int lastNegPitch_IRD = 0;
int lastPitch_IRD = 0;
int lastPosRoll_IRD = 0;
int lastNegRoll_IRD = 0;
int lastRoll_IRD = 0;
int lastYawClockwise_STL = 0;
int lastYawCounterClockwise_STL = 0;
int lastYawCombined_STL = 0;
int lastPosPitch_STL = 0;
int lastNegPitch_STL = 0;
int lastPitch_STL = 0;
int lastPosRoll_STL = 0;
int lastNegRoll_STL = 0;
int lastRoll_STL = 0;
int lastYawClockwise_VID = 0;
int lastYawCounterClockwise_VID = 0;
int lastYawCombined_VID = 0;
int lastPosPitch_VID = 0;
int lastNegPitch_VID = 0;
int lastPitch_VID = 0;
int lastPosRoll_VID = 0;
int lastNegRoll_VID = 0;
int lastRoll_VID = 0;
boolean printed = false;

OBJModel IRD_model;
OBJModel STL_model;
OBJModel VID_model;

// UI controls.
GPanel    configPanel;
GDropList serialList;
GLabel    serialLabel;
GLabel    calLabel;
GCheckbox printSerialCheckbox;

void setup()
{
  size(1280, 960, OPENGL);
  frameRate(15);
  IRD_model = new OBJModel(this);
  STL_model = new OBJModel(this);
  VID_model = new OBJModel(this);
  if(!isRedPayload){
    Model_FP = White_Model_FP;
  }else{
    Model_FP = Red_Model_FP;
  }
  
  IRD_sensorData=new ArrayList();
  IRD_BattVolt=new ArrayList();
  IRD_Temp=new ArrayList();
  IRD_AcclX=new ArrayList();
  IRD_AcclY=new ArrayList();
  IRD_AcclZ=new ArrayList();
  IRD_DegX=new ArrayList();
  IRD_DegY=new ArrayList();
  IRD_DegZ=new ArrayList();
  IRD_OrientX=new ArrayList();
  IRD_OrientY=new ArrayList();
  IRD_OrientZ=new ArrayList();
  IRD_SysCalib=new ArrayList();
  IRD_GyroCalib=new ArrayList();
  IRD_AcclCalib=new ArrayList();
  IRD_MagCalib=new ArrayList();
  IRD_Time=new ArrayList(); //in minutes (Added in 2.0.10)
  IRD_Time_Hr=new ArrayList(); 
  IRD_Time_Min=new ArrayList(); 
  IRD_Time_Sec=new ArrayList(); 
  IRD_readData(IRD_Data_FP); //CSV File Path
  IRD_model.load(Model_FP);
  IRD_model.scale(1);
  
  STL_sensorData=new ArrayList();
  STL_BattVolt=new ArrayList();
  STL_Temp=new ArrayList();
  STL_AcclX=new ArrayList();
  STL_AcclY=new ArrayList();
  STL_AcclZ=new ArrayList();
  STL_DegX=new ArrayList();
  STL_DegY=new ArrayList();
  STL_DegZ=new ArrayList();
  STL_OrientX=new ArrayList();
  STL_OrientY=new ArrayList();
  STL_OrientZ=new ArrayList();
  STL_SysCalib=new ArrayList();
  STL_GyroCalib=new ArrayList();
  STL_AcclCalib=new ArrayList();
  STL_MagCalib=new ArrayList();
  STL_Time=new ArrayList(); //in minutes (Added in 2.0.10)
  STL_Time_Hr=new ArrayList(); 
  STL_Time_Min=new ArrayList(); 
  STL_Time_Sec=new ArrayList(); 
  STL_readData(STL_Data_FP); // CSV File Path
  STL_model.load(Model_FP);
  STL_model.scale(1);
  
  if(!isRedPayload){
    VID_sensorData=new ArrayList();
    VID_BattVolt=new ArrayList();
    VID_Temp=new ArrayList();
    VID_AcclX=new ArrayList();
    VID_AcclY=new ArrayList();
    VID_AcclZ=new ArrayList();
    VID_DegX=new ArrayList();
    VID_DegY=new ArrayList();
    VID_DegZ=new ArrayList();
    VID_OrientX=new ArrayList();
    VID_OrientY=new ArrayList();
    VID_OrientZ=new ArrayList();
    VID_SysCalib=new ArrayList();
    VID_GyroCalib=new ArrayList();
    VID_AcclCalib=new ArrayList();
    VID_MagCalib=new ArrayList();
    VID_Time=new ArrayList(); //in minutes (Added in 2.0.10)
    VID_Time_Hr=new ArrayList(); 
    VID_Time_Min=new ArrayList();
    VID_Time_Sec=new ArrayList();
    VID_readData(VID_Data_FP); //CSV File Path
    VID_model.load(Model_FP);
    VID_model.scale(1);
  }
  
  // Simple 3 point lighting for dramatic effect.
  // Slightly red light in upper right, slightly blue light in upper left, and white light from behind.
  pointLight(255, 200, 200,  400, 400,  500);
  pointLight(200, 200, 255, -400, 400,  500);
  pointLight(255, 255, 255,    0,   0, -500);
  
  //Variable Ascent Rates (added in 2.0.10)
  if(!isRedPayload){
    first_5k_increments = 9;
    loopsPer5k = 38;
  }else{
    first_5k_increments = 8;
    loopsPer5k = 45;
  }
}
void draw()
{ 
  //debug print
  //println("loop " + lineNum + " ");
  
  // Set a new co-ordinate space
  background(165, 200, 255);
  pushMatrix();
  
  if(!isRedPayload){
    translate(600, 200, -200);
  }else{
    translate(600, 300, -200);
  }
  IRD_crunchNumbers();
  float c1 = cos(radians(roll));
  float s1 = sin(radians(roll));
  float c2 = cos(radians(pitch)); // intrinsic rotation
  float s2 = sin(radians(pitch));
  float c3 = cos(radians(yaw));
  float s3 = sin(radians(yaw));
  applyMatrix( c2*c3, s1*s3+c1*c3*s2, c3*s1*s2-c1*s3, 0,
               -s2, c1*c2, c2*s1, 0,
               c2*s3, c1*s2*s3-c3*s1, c1*c3+s1*s2*s3, 0,
               0, 0, 0, 1);
  pushMatrix();
  noStroke();
  IRD_model.draw();
  popMatrix();
  popMatrix();
  pushMatrix();
  if(!isRedPayload){
    translate(600, 550, -200);
  }else{
    translate(600, 800, -200);
  }
  STL_crunchNumbers();
  c1 = cos(radians(roll));
  s1 = sin(radians(roll));
  c2 = cos(radians(pitch)); // intrinsic rotation
  s2 = sin(radians(pitch));
  c3 = cos(radians(yaw));
  s3 = sin(radians(yaw));
  applyMatrix( c2*c3, s1*s3+c1*c3*s2, c3*s1*s2-c1*s3, 0,
               -s2, c1*c2, c2*s1, 0,
               c2*s3, c1*s2*s3-c3*s1, c1*c3+s1*s2*s3, 0,
               0, 0, 0, 1);
  pushMatrix();
  noStroke();
  STL_model.draw();
  popMatrix();
  popMatrix();
  if(!isRedPayload){
    pushMatrix();
    translate(600, 900, -200);
    VID_crunchNumbers();
    c1 = cos(radians(roll));
    s1 = sin(radians(roll));
    c2 = cos(radians(pitch)); // intrinsic rotation
    s2 = sin(radians(pitch));
    c3 = cos(radians(yaw));
    s3 = sin(radians(yaw));
    applyMatrix( c2*c3, s1*s3+c1*c3*s2, c3*s1*s2-c1*s3, 0,
                 -s2, c1*c2, c2*s1, 0,
                 c2*s3, c1*s2*s3-c3*s1, c1*c3+s1*s2*s3, 0,
                 0, 0, 0, 1);
    pushMatrix();
    noStroke();
    VID_model.draw();
    popMatrix();
    popMatrix();
  }
  //delay(50);  //5000 ~= real time
  fiveThousandMark();
  lineNum++;
}
void IRD_crunchNumbers() 
{
  try{
    //print("IRD Accelleration: x = " + IRD_AcclX.get(lineNum));
    //print(", y = " + IRD_AcclY.get(lineNum));
    //print(", z = " + IRD_AcclZ.get(lineNum));
    //print(", time = " + IRD_Time_Hr.get(lineNum));
    //print(":" + IRD_Time_Min.get(lineNum));
    //println(":" + IRD_Time_Sec.get(lineNum));
    
    roll = float(IRD_OrientZ.get(lineNum).toString());
    pitch = float(IRD_OrientY.get(lineNum).toString());
    yaw = float(IRD_OrientX.get(lineNum).toString());
    //wait until we have enough data...
    if(lineNum > 4){
      prevYaw_IRD_1 = float(IRD_OrientX.get(lineNum - 1).toString());
      prevYaw_IRD_2 = float(IRD_OrientX.get(lineNum - 2).toString());
      prevYaw_IRD_3 = float(IRD_OrientX.get(lineNum - 3).toString());
      prevPitch_IRD_1 = float(IRD_OrientY.get(lineNum - 1).toString());
      prevPitch_IRD_2 = float(IRD_OrientY.get(lineNum - 2).toString());
      prevPitch_IRD_3 = float(IRD_OrientY.get(lineNum - 3).toString());
      prevRoll_IRD_1 = float(IRD_OrientZ.get(lineNum - 1).toString());
      prevRoll_IRD_2 = float(IRD_OrientZ.get(lineNum - 2).toString());
      prevRoll_IRD_3 = float(IRD_OrientZ.get(lineNum - 3).toString());
      
      //Yaw Degree Counting
      //We don't want or care to count every miniscule rotation, only the rotations >= yawSensitivity degrees
      if(prevYaw_IRD_2 < prevYaw_IRD_3 - yawSensitivity){
        //We should be spinning counterclockwise.
        if(prevYaw_IRD_1 <= prevYaw_IRD_2){
          //This suggests that we were spinning counterclockwise.
          if(yaw <= prevYaw_IRD_1){
            //We can be fairly certain we're spinning counterclockwise
            totalCounterClockwiseDegYaw_IRD += (prevYaw_IRD_1 - yaw);
            totalDegYaw_IRD += (prevYaw_IRD_1 - yaw);
          }else if(yaw > prevYaw_IRD_1 + 270){
            //This suggests that we're spinning counterclockwise and just passed the threshold (0 - 359).
            totalCounterClockwiseDegYaw_IRD += ((359 - yaw) + prevYaw_IRD_1);
            totalDegYaw_IRD += ((359 - yaw) + prevYaw_IRD_1);
          }
        }
      }else if(prevYaw_IRD_2 > prevYaw_IRD_3 + yawSensitivity){
        //We should be spinning clockwise.
        if(prevYaw_IRD_1 > prevYaw_IRD_2){
          //This suggests that we were spinning clockwise.
          if(yaw >= prevYaw_IRD_1){
            //We can be fairly certain we're spinning clockwise
            totalClockwiseDegYaw_IRD += (yaw - prevYaw_IRD_1);
            totalDegYaw_IRD += (yaw - prevYaw_IRD_1);
          }else if(yaw < prevYaw_IRD_1 - 270){
            //This suggests that we're spinning clockwise and just passed the threshold (359 - 0).
            totalClockwiseDegYaw_IRD += (yaw + (359 - prevYaw_IRD_1));
            totalDegYaw_IRD += (yaw + (359 - prevYaw_IRD_1));
          }
        }
      }
      
      //Pitch Degree Counting
      //We only want to count the maximum values and only once.
      if(prevPitch_IRD_3 <  - pitchSensitivity){
        if(prevPitch_IRD_2 < prevPitch_IRD_3){
          //We should be decreasing.
          if(prevPitch_IRD_1 <= prevPitch_IRD_2){
            //This suggests that we were decreasing.
            if(pitch <= prevPitch_IRD_1){
              //We can be fairly certain we're decreasing.
              totalNegPitch_IRD += (pitch - prevPitch_IRD_1) * -1;
              totalDegPitch_IRD += (pitch - prevPitch_IRD_1) * -1;
            }
          }
        }
      }else if(prevPitch_IRD_3 > pitchSensitivity){ 
        if(prevPitch_IRD_2 > prevPitch_IRD_3){
          //We should be increasing.
          if(prevPitch_IRD_1 > prevPitch_IRD_2){
            //This suggests that we were increasing.
            if(pitch >= prevPitch_IRD_1){
              //We can be fairly certain we're increasing.
              totalPosPitch_IRD += (pitch - prevPitch_IRD_1);
              totalDegPitch_IRD += (pitch - prevPitch_IRD_1);
            }
          }
        }
      }
      
      //Roll Degree Counting
      //We only want to count the maximum values and only once.
      if(prevRoll_IRD_3 < -rollSensitivity){ 
        if(prevRoll_IRD_2 < prevRoll_IRD_3){
          //We should be decreasing.
          if(prevRoll_IRD_1 <= prevRoll_IRD_2){
            //This suggests that we were decreasing.
            if(roll <= prevRoll_IRD_1){
              //We can be fairly certain we're decreasing.
              totalNegRoll_IRD += (roll - prevRoll_IRD_1) * -1;
              totalDegRoll_IRD += (roll - prevRoll_IRD_1) * -1;
            }
          }
        }
      }else if(prevRoll_IRD_3 > rollSensitivity){ 
        if(prevRoll_IRD_2 > prevRoll_IRD_3){
          //We should be increasing.
          if(prevRoll_IRD_1 > prevRoll_IRD_2){
            //This suggests that we were increasing.
            if(roll >= prevRoll_IRD_1){
              //We can be fairly certain we're increasing.
              totalPosRoll_IRD += (roll - prevRoll_IRD_1);
              totalDegRoll_IRD += (roll - prevRoll_IRD_1);
            }
          }
        }
      }
    }
  }
  catch(IndexOutOfBoundsException ex){
    finish();
  }
}
void STL_crunchNumbers() 
{
  try{
    //print("STL Accelleration: x = " + STL_AcclX.get(lineNum));
    //print(", y = " + STL_AcclY.get(lineNum));
    //print(", z = " + STL_AcclZ.get(lineNum));
    //print(", time = " + STL_Time_Hr.get(lineNum));
    //print(":" + STL_Time_Min.get(lineNum));
    //println(":" + STL_Time_Sec.get(lineNum));
    
    roll = float(STL_OrientZ.get(lineNum).toString());
    pitch = float(STL_OrientY.get(lineNum).toString());
    yaw = float(STL_OrientX.get(lineNum).toString());
    //wait until we have enough data...
    if(lineNum > 4){
      prevYaw_STL_1 = float(STL_OrientX.get(lineNum - 1).toString());
      prevYaw_STL_2 = float(STL_OrientX.get(lineNum - 2).toString());
      prevYaw_STL_3 = float(STL_OrientX.get(lineNum - 3).toString());
      prevPitch_STL_1 = float(STL_OrientY.get(lineNum - 1).toString());
      prevPitch_STL_2 = float(STL_OrientY.get(lineNum - 2).toString());
      prevPitch_STL_3 = float(STL_OrientY.get(lineNum - 3).toString());
      prevRoll_STL_1 = float(STL_OrientZ.get(lineNum - 1).toString());
      prevRoll_STL_2 = float(STL_OrientZ.get(lineNum - 2).toString());
      prevRoll_STL_3 = float(STL_OrientZ.get(lineNum - 3).toString());
      
      //Yaw Degree Counting
      //We don't want or care to count every miniscule rotation, only the rotations >= yawSensitivity degrees
      if(prevYaw_STL_2 < prevYaw_STL_3 - yawSensitivity){
        //We should be spinning counterclockwise.
        if(prevYaw_STL_1 <= prevYaw_STL_2){
          //This suggests that we were spinning counterclockwise.
          if(yaw <= prevYaw_STL_1){
            //We can be fairly certain we're spinning counterclockwise
            totalCounterClockwiseDegYaw_STL += (prevYaw_STL_1 - yaw);
            totalDegYaw_STL += (prevYaw_STL_1 - yaw);
          }else if(yaw > prevYaw_STL_1 + 270){
            //This suggests that we're spinning counterclockwise and just passed the threshold (0 - 359).
            totalCounterClockwiseDegYaw_STL += ((359 - yaw) + prevYaw_STL_1);
            totalDegYaw_STL += ((359 - yaw) + prevYaw_STL_1);
          }
        }
      }else if(prevYaw_STL_2 > prevYaw_STL_3 + yawSensitivity){
        //We should be spinning clockwise.
        if(prevYaw_STL_1 > prevYaw_STL_2){
          //This suggests that we were spinning clockwise.
          if(yaw >= prevYaw_STL_1){
            //We can be fairly certain we're spinning clockwise
            totalClockwiseDegYaw_STL += (yaw - prevYaw_STL_1);
            totalDegYaw_STL += (yaw - prevYaw_STL_1);
          }else if(yaw < prevYaw_STL_1 - 270){
            //This suggests that we're spinning clockwise and just passed the threshold (359 - 0).
            totalClockwiseDegYaw_STL += (yaw + (359 - prevYaw_STL_1));
            totalDegYaw_STL += (yaw + (359 - prevYaw_STL_1));
          }
        }
      }
      
      //Pitch Degree Counting
      //We only want to count the maximum values and only once.
      if(prevPitch_STL_3 <  - pitchSensitivity){
        if(prevPitch_STL_2 < prevPitch_STL_3){
          //We should be decreasing.
          if(prevPitch_STL_1 <= prevPitch_STL_2){
            //This suggests that we were decreasing.
            if(pitch <= prevPitch_STL_1){
              //We can be fairly certain we're decreasing.
              totalNegPitch_STL += (pitch - prevPitch_STL_1) * -1;
              totalDegPitch_STL += (pitch - prevPitch_STL_1) * -1;
            }
          }
        }
      }else if(prevPitch_STL_3 > pitchSensitivity){ 
        if(prevPitch_STL_2 > prevPitch_STL_3){
          //We should be increasing.
          if(prevPitch_STL_1 > prevPitch_STL_2){
            //This suggests that we were increasing.
            if(pitch >= prevPitch_STL_1){
              //We can be fairly certain we're increasing.
              totalPosPitch_STL += (pitch - prevPitch_STL_1);
              totalDegPitch_STL += (pitch - prevPitch_STL_1);
            }
          }
        }
      }
      
      //Roll Degree Counting
      //We only want to count the maximum values and only once.
      if(prevRoll_STL_3 < -rollSensitivity){ 
        if(prevRoll_STL_2 < prevRoll_STL_3){
          //We should be decreasing.
          if(prevRoll_STL_1 <= prevRoll_STL_2){
            //This suggests that we were decreasing.
            if(roll <= prevRoll_STL_1){
              //We can be fairly certain we're decreasing.
              totalNegRoll_STL += (roll - prevRoll_STL_1) * -1;
              totalDegRoll_STL += (roll - prevRoll_STL_1) * -1;
            }
          }
        }
      }else if(prevRoll_STL_3 > rollSensitivity){ 
        if(prevRoll_STL_2 > prevRoll_STL_3){
          //We should be increasing.
          if(prevRoll_STL_1 > prevRoll_STL_2){
            //This suggests that we were increasing.
            if(roll >= prevRoll_STL_1){
              //We can be fairly certain we're increasing.
              totalPosRoll_STL += (roll - prevRoll_STL_1);
              totalDegRoll_STL += (roll - prevRoll_STL_1);
            }
          }
        }
      }
    }
  }
  catch(IndexOutOfBoundsException ex){
    finish();
  }
}
void VID_crunchNumbers() 
{
  try{
    //print("VID Accelleration: x = " + VID_AcclX.get(lineNum));
    //print(", y = " + VID_AcclY.get(lineNum));
    //print(", z = " + VID_AcclZ.get(lineNum));
    //print(", time = " + VID_Time_Hr.get(lineNum));
    //print(":" + VID_Time_Min.get(lineNum));
    //println(":" + VID_Time_Sec.get(lineNum));
    
    roll = float(VID_OrientZ.get(lineNum).toString());
    pitch = float(VID_OrientY.get(lineNum).toString());
    yaw = float(VID_OrientX.get(lineNum).toString());
    //wait until we have enough data...VID
    if(lineNum > 4){
      prevYaw_VID_1 = float(VID_OrientX.get(lineNum - 1).toString());
      prevYaw_VID_2 = float(VID_OrientX.get(lineNum - 2).toString());
      prevYaw_VID_3 = float(VID_OrientX.get(lineNum - 3).toString());
      prevPitch_VID_1 = float(VID_OrientY.get(lineNum - 1).toString());
      prevPitch_VID_2 = float(VID_OrientY.get(lineNum - 2).toString());
      prevPitch_VID_3 = float(VID_OrientY.get(lineNum - 3).toString());
      prevRoll_VID_1 = float(VID_OrientZ.get(lineNum - 1).toString());
      prevRoll_VID_2 = float(VID_OrientZ.get(lineNum - 2).toString());
      prevRoll_VID_3 = float(VID_OrientZ.get(lineNum - 3).toString());
      
      //Yaw Degree Counting
      //We don't want or care to count every miniscule rotation, only the rotations >= yawSensitivity degrees
      if(prevYaw_VID_2 < prevYaw_VID_3 - yawSensitivity){
        //We should be spinning counterclockwise.
        if(prevYaw_VID_1 <= prevYaw_VID_2){
          //This suggests that we were spinning counterclockwise.
          if(yaw <= prevYaw_VID_1){
            //We can be fairly certain we're spinning counterclockwise
            totalCounterClockwiseDegYaw_VID += (prevYaw_VID_1 - yaw);
            totalDegYaw_VID += (prevYaw_VID_1 - yaw);
          }else if(yaw > prevYaw_VID_1 + 270){
            //This suggests that we're spinning counterclockwise and just passed the threshold (0 - 359).
            totalCounterClockwiseDegYaw_VID += ((359 - yaw) + prevYaw_VID_1);
            totalDegYaw_VID += ((359 - yaw) + prevYaw_VID_1);
          }
        }
      }else if(prevYaw_VID_2 > prevYaw_VID_3 + yawSensitivity){
        //We should be spinning clockwise.
        if(prevYaw_VID_1 > prevYaw_VID_2){
          //This suggests that we were spinning clockwise.
          if(yaw >= prevYaw_VID_1){
            //We can be fairly certain we're spinning clockwise
            totalClockwiseDegYaw_VID += (yaw - prevYaw_VID_1);
            totalDegYaw_VID += (yaw - prevYaw_VID_1);
          }else if(yaw < prevYaw_VID_1 - 270){
            //This suggests that we're spinning clockwise and just passed the threshold (359 - 0).
            totalClockwiseDegYaw_VID += (yaw + (359 - prevYaw_VID_1));
            totalDegYaw_VID += (yaw + (359 - prevYaw_VID_1));
          }
        }
      }
      
      //Pitch Degree Counting
      //We only want to count the maximum values and only once.
      if(prevPitch_VID_3 <  - pitchSensitivity){
        if(prevPitch_VID_2 < prevPitch_VID_3){
          //We should be decreasing.
          if(prevPitch_VID_1 <= prevPitch_VID_2){
            //This suggests that we were decreasing.
            if(pitch <= prevPitch_VID_1){
              //We can be fairly certain we're decreasing.
              totalNegPitch_VID += (pitch - prevPitch_VID_1) * -1;
              totalDegPitch_VID += (pitch - prevPitch_VID_1) * -1;
            }
          }
        }
      }else if(prevPitch_VID_3 > pitchSensitivity){ 
        if(prevPitch_VID_2 > prevPitch_VID_3){
          //We should be increasing.
          if(prevPitch_VID_1 > prevPitch_VID_2){
            //This suggests that we were increasing.
            if(pitch >= prevPitch_VID_1){
              //We can be fairly certain we're increasing.
              totalPosPitch_VID += (pitch - prevPitch_VID_1);
              totalDegPitch_VID += (pitch - prevPitch_VID_1);
            }
          }
        }
      }
      
      //Roll Degree Counting
      //We only want to count the maximum values and only once.
      if(prevRoll_VID_3 < -rollSensitivity){ 
        if(prevRoll_VID_2 < prevRoll_VID_3){
          //We should be decreasing.
          if(prevRoll_VID_1 <= prevRoll_VID_2){
            //This suggests that we were decreasing.
            if(roll <= prevRoll_VID_1){
              //We can be fairly certain we're decreasing.
              totalNegRoll_VID += (roll - prevRoll_VID_1) * -1;
              totalDegRoll_VID += (roll - prevRoll_VID_1) * -1;
            }
          }
        }
      }else if(prevRoll_VID_3 > rollSensitivity){ 
        if(prevRoll_VID_2 > prevRoll_VID_3){
          //We should be increasing.
          if(prevRoll_VID_1 > prevRoll_VID_2){
            //This suggests that we were increasing.
            if(roll >= prevRoll_VID_1){
              //We can be fairly certain we're increasing.
              totalPosRoll_VID += (roll - prevRoll_VID_1);
              totalDegRoll_VID += (roll - prevRoll_VID_1);
            }
          }
        }
      }
    }
  }
  catch(IndexOutOfBoundsException ex){
    finish();
  }
}
void IRD_readData(String myFileName){
  File file=new File(myFileName);
  BufferedReader br=null;
  try{
    br=new BufferedReader(new FileReader(file));
    String text=null;
    while((text=br.readLine())!=null){
      String [] subtext = splitTokens(text,",");
      IRD_BattVolt.add(float(subtext[0]));
      IRD_Temp.add(int(subtext[1]));
      IRD_AcclX.add(int(subtext[2]));
      IRD_AcclY.add(int(subtext[3]));
      IRD_AcclZ.add(int(subtext[4]));
      IRD_DegX.add(int(subtext[5]));
      IRD_DegY.add(int(subtext[6]));
      IRD_DegZ.add(int(subtext[7]));
      IRD_OrientX.add(int(subtext[8]));
      IRD_OrientY.add(int(subtext[9]));
      IRD_OrientZ.add(int(subtext[10]));
      IRD_SysCalib.add(int(subtext[11]));
      IRD_GyroCalib.add(int(subtext[12]));
      IRD_AcclCalib.add(int(subtext[13]));
      IRD_MagCalib.add(int(subtext[14]));
      if(turbulenceVer == 1){
        IRD_Time.add(int(subtext[15]));
      }else{
        IRD_Time_Hr.add(int(subtext[15]));
        IRD_Time_Min.add(int(subtext[16]));
        IRD_Time_Sec.add(int(subtext[17]));
      }
      IRD_sensorData.add(text);
    }
    //println("Done Reading Data");
  }catch(FileNotFoundException e){
    e.printStackTrace();
  }catch(IOException e){
    e.printStackTrace();
  }finally{
    try {
      if (br != null){
        br.close();
      }
    } catch (IOException e) {
      e.printStackTrace();
    }
  }
}
void STL_readData(String myFileName){
  File file=new File(myFileName);
  BufferedReader br=null;
  try{
    br=new BufferedReader(new FileReader(file));
    String text=null;
    while((text=br.readLine())!=null){
      String [] subtext = splitTokens(text,",");
      STL_BattVolt.add(float(subtext[0]));
      STL_Temp.add(int(subtext[1]));
      STL_AcclX.add(int(subtext[2]));
      STL_AcclY.add(int(subtext[3]));
      STL_AcclZ.add(int(subtext[4]));
      STL_DegX.add(int(subtext[5]));
      STL_DegY.add(int(subtext[6]));
      STL_DegZ.add(int(subtext[7]));
      STL_OrientX.add(int(subtext[8]));
      STL_OrientY.add(int(subtext[9]));
      STL_OrientZ.add(int(subtext[10]));
      STL_SysCalib.add(int(subtext[11]));
      STL_GyroCalib.add(int(subtext[12]));
      STL_AcclCalib.add(int(subtext[13]));
      STL_MagCalib.add(int(subtext[14]));
      if(turbulenceVer == 1){
        STL_Time.add(int(subtext[15]));
      }else{
        STL_Time_Hr.add(int(subtext[15]));
        STL_Time_Min.add(int(subtext[16]));
        STL_Time_Sec.add(int(subtext[17]));
      }
      STL_sensorData.add(text);
    }
    //println("Done Reading Data");
  }catch(FileNotFoundException e){
    e.printStackTrace();
  }catch(IOException e){
    e.printStackTrace();
  }finally{
    try {
      if (br != null){
        br.close();
      }
    } catch (IOException e) {
      e.printStackTrace();
    }
  }
}
void VID_readData(String myFileName){
  File file=new File(myFileName);
  BufferedReader br=null;
  try{
    br=new BufferedReader(new FileReader(file));
    String text=null;
    while((text=br.readLine())!=null){
      String [] subtext = splitTokens(text,",");
      VID_BattVolt.add(float(subtext[0]));
      VID_Temp.add(int(subtext[1]));
      VID_AcclX.add(int(subtext[2]));
      VID_AcclY.add(int(subtext[3]));
      VID_AcclZ.add(int(subtext[4]));
      VID_DegX.add(int(subtext[5]));
      VID_DegY.add(int(subtext[6]));
      VID_DegZ.add(int(subtext[7]));
      VID_OrientX.add(int(subtext[8]));
      VID_OrientY.add(int(subtext[9]));
      VID_OrientZ.add(int(subtext[10]));
      VID_SysCalib.add(int(subtext[11]));
      VID_GyroCalib.add(int(subtext[12]));
      VID_AcclCalib.add(int(subtext[13]));
      VID_MagCalib.add(int(subtext[14]));
      if(turbulenceVer == 1){
        VID_Time.add(int(subtext[15]));
      }else{
        VID_Time_Hr.add(int(subtext[15]));
        VID_Time_Min.add(int(subtext[16]));
        VID_Time_Sec.add(int(subtext[17]));
      }
      VID_sensorData.add(text);
    }
    //println("Done Reading Data");
  }catch(FileNotFoundException e){
    e.printStackTrace();
  }catch(IOException e){
    e.printStackTrace();
  }finally{
    try {
      if (br != null){
        br.close();
      }
    } catch (IOException e) {
      e.printStackTrace();
    }
  }
}
void fiveThousandMark(){
  lineCount++; //For printing every 5000 feet or so.
  if(lineCount == loopsPer5k){
    markCount++;
    int printYawClockwise_IRD = (totalClockwiseDegYaw_IRD - lastYawClockwise_IRD);
    int printYawCounterClockwise_IRD = (totalCounterClockwiseDegYaw_IRD - lastYawCounterClockwise_IRD);
    int printYawCombined_IRD = (totalDegYaw_IRD - lastYawCombined_IRD);
    int printPosRoll_IRD = (totalPosRoll_IRD - lastPosRoll_IRD);
    int printNegRoll_IRD = (totalNegRoll_IRD - lastNegRoll_IRD);
    int printRoll_IRD = (totalDegRoll_IRD - lastRoll_IRD);
    int printPosPitch_IRD = (totalPosPitch_IRD - lastPosPitch_IRD);
    int printNegPitch_IRD = (totalNegPitch_IRD - lastNegPitch_IRD);
    int printPitch_IRD = (totalDegPitch_IRD - lastPitch_IRD);
    println("****************************");
    println(5000 * markCount + " approximate feet.");
    //println("Iridium Package Degrees Clockwise Yaw = " + printYawClockwise_IRD);
    //println("Iridium Package Degrees Counterclockwise Yaw = " + printYawCounterClockwise_IRD);
    println("Iridium Package Degrees Total Yaw = " + printYawCombined_IRD);
    //println("Iridium Package Degrees Positive Pitch = " + printPosPitch_IRD);
    //println("Iridium Package Degrees Negative Pitch = " + printNegPitch_IRD);
    println("Iridium Package Degrees Total Pitch = " + printPitch_IRD);
    //println("Iridium Package Degrees Positive Roll = " + printPosRoll_IRD);
    //println("Iridium Package Degrees Negative Roll = " + printNegRoll_IRD);
    println("Iridium Package Degrees Total Roll = " + printRoll_IRD);
    println();
    lastYawClockwise_IRD = totalClockwiseDegYaw_IRD;
    lastYawCounterClockwise_IRD = totalCounterClockwiseDegYaw_IRD;
    lastYawCombined_IRD = totalDegYaw_IRD;
    lastPosPitch_IRD = totalPosPitch_IRD;
    lastNegPitch_IRD = totalNegPitch_IRD;
    lastPitch_IRD = totalDegPitch_IRD;
    lastPosRoll_IRD = totalPosRoll_IRD;
    lastNegRoll_IRD = totalNegRoll_IRD;
    lastRoll_IRD = totalDegRoll_IRD;
    int printYawClockwise_STL = (totalClockwiseDegYaw_STL - lastYawClockwise_STL);
    int printYawCounterClockwise_STL = (totalCounterClockwiseDegYaw_STL - lastYawCounterClockwise_STL);
    int printYawCombined_STL = (totalDegYaw_STL - lastYawCombined_STL);
    int printPosRoll_STL = (totalPosRoll_STL - lastPosRoll_STL);
    int printNegRoll_STL = (totalNegRoll_STL - lastNegRoll_STL);
    int printRoll_STL = (totalDegRoll_STL - lastRoll_STL);
    int printPosPitch_STL = (totalPosPitch_STL - lastPosPitch_STL);
    int printNegPitch_STL = (totalNegPitch_STL - lastNegPitch_STL);
    int printPitch_STL = (totalDegPitch_STL - lastPitch_STL);
    //println("Still Image Package Degrees Clockwise Yaw = " + printYawClockwise_STL);
    //println("Still Image Package Degrees Counterclockwise Yaw = " + printYawCounterClockwise_STL);
    println("Still Image Package Degrees Total Yaw = " + printYawCombined_STL);
    //println("Still Image Package Degrees Positive Pitch = " + printPosPitch_STL);
    //println("Still Image Package Degrees Negative Pitch = " + printNegPitch_STL);
    println("Still Image Package Degrees Total Pitch = " + printPitch_STL);
    //println("Still Image Package Degrees Positive Roll = " + printPosRoll_STL);
    //println("Still Image Package Degrees Negative Roll = " + printNegRoll_STL);
    println("Still Image Package Degrees Total Roll = " + printRoll_STL);
    println();
    lastYawClockwise_STL = totalClockwiseDegYaw_STL;
    lastYawCounterClockwise_STL = totalCounterClockwiseDegYaw_STL;
    lastYawCombined_STL = totalDegYaw_STL;
    lastPosPitch_STL = totalPosPitch_STL;
    lastNegPitch_STL = totalNegPitch_STL;
    lastPitch_STL = totalDegPitch_STL;
    lastPosRoll_STL = totalPosRoll_STL;
    lastNegRoll_STL = totalNegRoll_STL;
    lastRoll_STL = totalDegRoll_STL;
    if(!isRedPayload){
      int printYawClockwise_VID = (totalClockwiseDegYaw_VID - lastYawClockwise_VID);
      int printYawCounterClockwise_VID = (totalCounterClockwiseDegYaw_VID - lastYawCounterClockwise_VID);
      int printYawCombined_VID = (totalDegYaw_VID - lastYawCombined_VID);
      int printPosRoll_VID = (totalPosRoll_VID - lastPosRoll_VID);
      int printNegRoll_VID = (totalNegRoll_VID - lastNegRoll_VID);
      int printRoll_VID = (totalDegRoll_VID - lastRoll_VID);
      int printPosPitch_VID = (totalPosPitch_VID - lastPosPitch_VID);
      int printNegPitch_VID = (totalNegPitch_VID - lastNegPitch_VID);
      int printPitch_VID = (totalDegPitch_VID - lastPitch_VID);
      //println("Video Package Degrees Clockwise Yaw = " + printYawClockwise_VID);
      //println("Video Package Degrees Counterclockwise Yaw = " + printYawCounterClockwise_VID);
      println("Video Package Degrees Total Yaw = " + printYawCombined_VID);
      //println("Video Package Degrees Positive Pitch = " + printPosPitch_VID);
      //println("Video Package Degrees Negative Pitch = " + printNegPitch_VID);
      println("Video Package Degrees Total Pitch = " + printPitch_VID);
      //println("Video Package Degrees Positive Roll = " + printPosRoll_VID);
      //println("Video Package Degrees Negative Roll = " + printNegRoll_VID);
      println("Video Package Degrees Total Roll = " + printRoll_VID);
      lastYawClockwise_VID = totalClockwiseDegYaw_VID;
      lastYawCounterClockwise_VID = totalCounterClockwiseDegYaw_VID;
      lastYawCombined_VID = totalDegYaw_VID;
      lastPosPitch_VID = totalPosPitch_VID;
      lastNegPitch_VID = totalNegPitch_VID;
      lastPitch_VID = totalDegPitch_VID;
      lastPosRoll_VID = totalPosRoll_VID;
      lastNegRoll_VID = totalNegRoll_VID;
      lastRoll_VID = totalDegRoll_VID;
    }
    lineCount = 0;
    
    //The balloon changes ascent rate.
    if(first_5k_increments == 0){
      if(!isRedPayload){
        loopsPer5k = 72;
      }else{
        loopsPer5k = 76;
      }
    }else{
      first_5k_increments -= 1;
    }
  }
}
void finish(){
  if(!printed){
    println("");
    println("****************************");
    println("Simulation Complete");
    println("Iridium Package Total Clockwise Degrees Yaw = " + totalClockwiseDegYaw_IRD);
    println("Iridium Package Total Counterclockwise Degrees Yaw = " + totalCounterClockwiseDegYaw_IRD);
    println("Iridium Package Total Combined Degrees Yaw = " + totalDegYaw_IRD);
    println("Iridium Package Total Degrees Positive Pitch = " + totalPosPitch_IRD);
    println("Iridium Package Total Degrees Negative Pitch = " + totalNegPitch_IRD);
    println("Iridium Package Total Degrees Pitch = " + totalDegPitch_IRD);
    println("Iridium Package Total Degrees Positive Roll = " + totalPosRoll_IRD);
    println("Iridium Package Total Degrees Negative Roll = " + totalNegRoll_IRD);
    println("Iridium Package Total Degrees Roll = " + totalDegRoll_IRD);
    println();
    println("Still Image Package Total Clockwise Degrees Yaw = " + totalClockwiseDegYaw_STL);
    println("Still Image Package Total Counterclockwise Degrees Yaw = " + totalCounterClockwiseDegYaw_STL);
    println("Still Image Package Total Combined Degrees Yaw = " + totalDegYaw_STL);
    println("Still Image Package Total Degrees Positive Pitch = " + totalPosPitch_STL);
    println("Still Image Package Total Degrees Negative Pitch = " + totalNegPitch_STL);
    println("Still Image Package Total Degrees Pitch = " + totalDegPitch_STL);
    println("Still Image Package Total Degrees Positive Roll = " + totalPosRoll_STL);
    println("Still Image Package Total Degrees Negative Roll = " + totalNegRoll_STL);
    println("Still Image Package Total Degrees Roll = " + totalDegRoll_STL);
    println();
    if(!isRedPayload){
      println("Video Package Total Clockwise Degrees Yaw = " + totalClockwiseDegYaw_VID);
      println("Video Package Total Counterclockwise Degrees Yaw = " + totalCounterClockwiseDegYaw_VID);
      println("Video Package Total Combined Degrees Yaw = " + totalDegYaw_VID);
      println("Video Package Total Degrees Positive Pitch = " + totalPosPitch_VID);
      println("Video Package Total Degrees Negative Pitch = " + totalNegPitch_VID);
      println("Video Package Total Degrees Pitch = " + totalDegPitch_VID);
      println("Video Package Total Degrees Positive Roll = " + totalPosRoll_VID);
      println("Video Package Total Degrees Negative Roll = " + totalNegRoll_VID);
      println("Video Package Total Degrees Roll = " + totalDegRoll_VID);
    }
    printed = true;
  }
  exit();  
}