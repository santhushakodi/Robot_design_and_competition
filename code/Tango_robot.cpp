#include <iostream>
#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#define TIME_STEP 16
#define c 1200

using std::string;
#define TIME_STEP 16
#define MAX_SPEED 6.28

double TJ=0;
int flag=1;
int T=0;
int box=0;
int box_count=0;
int radius_count=0;
int first=0;
int p1=0;
int p2=0;
int gate=0;
int error=0;
///////////////pillar count///////////////////
int ramp=0;
int pillars=0;
////////////wall following parameters//////////////////////////////////
long speed[2];
long p=0, i=0, d=0, pid=0;

float kP=0.02; // 0.80
float kI=0.002; // 0.00
float kD=0.001; // 0.02

long pr_error=0;
long total_er=0;
long r_error;
 
long pl_error=0;
long total_el=0;
long l_error;
////////////////////////////////////////////////////////
using namespace webots;
Robot *robot = new Robot();
///////////////////////color detect/////////////////
std::string color(){
int r_count=0,b_count=0,g_count=0,pixels=0;
Camera *cam;
cam=robot->getCamera("camera");
cam->enable(TIME_STEP);
int w=cam->getWidth();
int h=cam->getHeight();
int timeStep = (int)robot->getBasicTimeStep();

while (robot->step(timeStep) != -1) {
   const unsigned char *image = cam->getImage();
   for (int x = 0; x < w; x++){
     for (int y = 0; y < h; y++) {
       int r = cam->imageGetRed(image, w, x, y);
       int g = cam->imageGetGreen(image, w, x, y);
       int b = cam->imageGetBlue(image, w, x, y);       
    if(r >= b && r >= g)
        r_count+=1;       
    if(g >= r && g >= b)
        g_count+=1;    
    if(b >= r && b >= g)
        b_count+=1;}}
    pixels=w*h;
    if ((float)r_count/(float)pixels >0.5){
      std::cout<<"red"<<std::endl;
      return "red";}
    else if((float)g_count/(float)pixels >0.5){
      std::cout<<"green"<<std::endl;
      return "green";}
    else{
      std::cout<<"blue"<<std::endl;
      return "blue";}
  }}
/////////////////////////////////////////////////////////
////////////////////turn functions/////////////////////////
void turn(float alpha,int lr){ 
float radius=0.032; //m
float encoder_unit=(2*3.14*radius)/6.28; //  m/rad
float wheel_distance=0.14 ;  
double ps_values[2]={0.,0.}; //0-left,1-right
double last_ps_values[2]={0.,0.};
double distance[2]={0.,0.};  //0-left,1-right
double diff=0;
double w=0;
float angle=0;
int i=1;
double ps0=0.;
double ps1=0.;  
Motor *leftMotor = robot->getMotor("left_wheel");
Motor *rightMotor = robot->getMotor("right_wheel");
leftMotor->setPosition(INFINITY);
rightMotor->setPosition(INFINITY);
leftMotor->setVelocity(0.0);
rightMotor->setVelocity(0.0);  
int timeStep = (int)robot->getBasicTimeStep();
PositionSensor *left_ps=robot->getPositionSensor("psl");
PositionSensor *right_ps=robot->getPositionSensor("psr");
left_ps->enable(timeStep);
right_ps->enable(timeStep); 
while (robot->step(timeStep) != -1 ) {
    ps_values[0]=left_ps->getValue();
    ps_values[1]=right_ps->getValue();
    if(i==1){
    ps0=ps_values[0];
    ps1=ps_values[1];
    i=2;
    }
    ps_values[0]-=ps0;
    ps_values[1]-=ps1;
    leftMotor->setVelocity(lr*-2.0);
    rightMotor->setVelocity(lr*2.0);
    for (int i = 0; i < 2 ; i++) {
      //diff=ps_values[i]-last_ps_values[i];
     diff=last_ps_values[i]-ps_values[i];
      distance[i]=diff*encoder_unit; //distance in m for this instance 
    }
    w=(distance[0]-distance[1])/wheel_distance ;//angle in clocwise direction
    angle+=w;
     for (int i = 0; i < 2 ; i++) {
       last_ps_values[i]=ps_values[i];  
     }
    if (lr*angle>alpha){
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
    break;
    }
  };
}

void turn_back(){ 
float radius=0.032; //m
float encoder_unit=(2*3.14*radius)/6.28; //  m/rad
float wheel_distance=0.14 ;  
double ps_values[2]={0.,0.}; //0-left,1-right
double last_ps_values[2]={0.,0.};
double distance[2]={0.,0.};  //0-left,1-right
double diff=0;
double w=0;
float angle=0;
int i=1;
double ps0=0.;
double ps1=0.;
Motor *leftMotor = robot->getMotor("left_wheel");
Motor *rightMotor = robot->getMotor("right_wheel");
leftMotor->setPosition(INFINITY);
rightMotor->setPosition(INFINITY);
leftMotor->setVelocity(0.0);
rightMotor->setVelocity(0.0);  
int timeStep = (int)robot->getBasicTimeStep();  
PositionSensor *left_ps=robot->getPositionSensor("psl");
PositionSensor *right_ps=robot->getPositionSensor("psr");
left_ps->enable(timeStep);
right_ps->enable(timeStep);
  
while (robot->step(timeStep) != -1 ) {
    ps_values[0]=left_ps->getValue();
    ps_values[1]=right_ps->getValue();
    if(i==1){
    ps0=ps_values[0];
    ps1=ps_values[1];
    i=2;
    }
    ps_values[0]-=ps0;
    ps_values[1]-=ps1;
    leftMotor->setVelocity(-2.0);
    rightMotor->setVelocity(2.0);
    for (int i = 0; i < 2 ; i++) {
      //diff=ps_values[i]-last_ps_values[i];
     diff=last_ps_values[i]-ps_values[i];
      distance[i]=diff*encoder_unit; //distance in m for this instance  
    }
    w=(distance[0]-distance[1])/wheel_distance ;//angle in clocwise direction
    angle+=w;
     for (int i = 0; i < 2 ; i++) {
       last_ps_values[i]=ps_values[i];  
     }
    if (angle>4.1){
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
    break;
    } 
  };
}

//////////////////move back///////////////////////////////////////
void move_back(float l){
float radius=0.032; //m
float teeta=0.;
int i=1;
teeta=-l/radius;
double ps_values[2]={0.,0.};
double ps0=0.;
double ps1=0.;
Motor *leftMotor = robot->getMotor("left_wheel");
Motor *rightMotor = robot->getMotor("right_wheel");
leftMotor->setPosition(INFINITY);
rightMotor->setPosition(INFINITY);
leftMotor->setVelocity(0.0);
rightMotor->setVelocity(0.0);
int timeStep = (int)robot->getBasicTimeStep();  
PositionSensor *left_ps=robot->getPositionSensor("psl");
PositionSensor *right_ps=robot->getPositionSensor("psr");
left_ps->enable(timeStep);
right_ps->enable(timeStep);

while (robot->step(timeStep) != -1 ) {        
    ps_values[0]=left_ps->getValue();
    ps_values[1]=right_ps->getValue();
    if(i==1){
    ps0=ps_values[0];
    ps1=ps_values[1];
    i=2;
    }
    ps_values[0]-=ps0;
    ps_values[1]-=ps1;
    leftMotor->setVelocity(-2.0);
    rightMotor->setVelocity(-2.0);
    if (ps_values[0]<teeta){
    leftMotor->setVelocity(0.);
    rightMotor->setVelocity(0.);
    break;
    }    
  };
}
////////////////////////////////////////////////////
void move_forward(float l){
float radius=0.032; //m
float teeta=0.;
int i=1;
teeta=l/radius;
double ps_values[2]={0.,0.};
double ps0=0.;
double ps1=0.;
Motor *leftMotor = robot->getMotor("left_wheel");
Motor *rightMotor = robot->getMotor("right_wheel");
leftMotor->setPosition(INFINITY);
rightMotor->setPosition(INFINITY);
leftMotor->setVelocity(0.0);
rightMotor->setVelocity(0.0);
int timeStep = (int)robot->getBasicTimeStep();
PositionSensor *left_ps=robot->getPositionSensor("psl");
PositionSensor *right_ps=robot->getPositionSensor("psr");
left_ps->enable(timeStep);
right_ps->enable(timeStep);

while (robot->step(timeStep) != -1 ) {   
    ps_values[0]=left_ps->getValue();
    ps_values[1]=right_ps->getValue();
    if(i==1){
    ps0=ps_values[0];
    ps1=ps_values[1];
    i=2;
    }
    ps_values[0]-=ps0;
    ps_values[1]-=ps1;
    leftMotor->setVelocity(2.0);
    rightMotor->setVelocity(2.0);
    if (ps_values[0]>teeta){
    leftMotor->setVelocity(0.);
    rightMotor->setVelocity(0.);
    break;
    }
  };
}
////////////////////////////////////////////////////////
void move_fast_forward(float l){
float radius=0.032; //m
float teeta=0.;
int i=1;
teeta=l/radius;
double ps_values[2]={0.,0.};
double ps0=0.;
double ps1=0.;
Motor *leftMotor = robot->getMotor("left_wheel");
Motor *rightMotor = robot->getMotor("right_wheel");
leftMotor->setPosition(INFINITY);
rightMotor->setPosition(INFINITY);
leftMotor->setVelocity(0.0);
rightMotor->setVelocity(0.0);
int timeStep = (int)robot->getBasicTimeStep();  
PositionSensor *left_ps=robot->getPositionSensor("psl");
PositionSensor *right_ps=robot->getPositionSensor("psr");
left_ps->enable(timeStep);
right_ps->enable(timeStep);

while (robot->step(timeStep) != -1 ) {
    ps_values[0]=left_ps->getValue();
    ps_values[1]=right_ps->getValue();
    if(i==1){
    ps0=ps_values[0];
    ps1=ps_values[1];
    i=2;
    }
    ps_values[0]-=ps0;
    ps_values[1]-=ps1;
    leftMotor->setVelocity(20.0);
    rightMotor->setVelocity(20.0);
    if (ps_values[0]>teeta){
    leftMotor->setVelocity(0.);
    rightMotor->setVelocity(0.);
    break;
    }    
  };
}
/////////////motor run function/////////////////////////
void motor_run(int m1,int m2){
    if (m1>10){
        m1=10;
        }
    if (m2>10){
        m2=10;
        }
    if (m1<0){
        m1=2;
        }
    if (m2<0){
        m2=2;
        }
    speed[0]=m1; //right speed
    speed[1]=m2; //left speed    
}

////////////declaration of variable line following////////////
double sensorValues[12];
double initialSpeed = 4.00;
double set = 3500;
double e = 0;
double leftSpeed = 0;
double rightSpeed = 0;

// Convert to binary values
void convert(){
 for (int i = 0; i <10; i++){
   if (sensorValues[i] > 900){
     sensorValues[i] = 1;
   }else{
     sensorValues[i] = 0;
   }
 }

}

//////////////////////////////PID control/////////////////////
double PID_calc(){
double avg = 0;
double total = 0;
for (int i = 0; i < 8 ; i++){ 
   avg += sensorValues[i] * i * 1000;
   total += sensorValues[i]; 
}
double position = avg / total;  //weighted mean
double kp = 0.006;
double kd = 0.00001;
double ki = 0.00000000000001;
double error = position - set;
double P = kp * error;
double D = kd * (error - e);
double I = ki * (error + e);
double offset = (P + D + I);
e = error; 
return offset;
}

//////// To control the speed///////////
double MSpeed(double speed){
if (speed > 0){
   if (speed > 10){
     speed = 10;
   }
}else{
   if (speed < -10){
     speed = -10;
   }
} 
return speed;
}
////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
  Motor *base;
  base=robot->getMotor("base");
  double base_rotate=0.0 ;
  base->setPosition(1);
  
// Initialize & enable the distance sensors  
  DistanceSensor *ir[12];
  char sensorNames[12][15] = {"gs0","gs1","gs2","gs3","gs4","gs5","gs6","gs7","left3","right3","sharp","sharp2"};
  for (int i = 0; i < 12; i++) {
    ir[i] = robot->getDistanceSensor(sensorNames[i]);
    ir[i]->enable(TIME_STEP);
  }
    
//Initialize the Motors 
  Motor *wheels[2];
  char wheels_names[2][15] = {"left_wheel","right_wheel"};
  for (int i = 0; i <2; i++){
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
    
move_forward(0.15);/////begining////////
  
/////////////////Line following loop until the end of maze/////////////////////
  while (robot->step(TIME_STEP) != -1){

// To collect the sensor output
    for (int i = 0; i < 12 ; i++){
      sensorValues[i] = ir[i]->getValue();
    }  
         
    convert(); //Covert sensor values to binary
    
    //Left sidejunction
    if (sensorValues[9]==1 && sensorValues[8]==0 && box==0 && TJ==0){
      wheels[0] ->setVelocity(0.0); 
      wheels[1] ->setVelocity(10.0);
      if (sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0 && sensorValues[7]==0){
        wheels[0] ->setVelocity(0.0); 
        wheels[1] ->setVelocity(10.0);
      }else{
        wheels[0] ->setVelocity(0.0); 
        wheels[1] ->setVelocity(10.0);
      }
    }
    //Right side junction
    else if(sensorValues[9]==0 && sensorValues[8]==1 && box==0 && TJ==0){
      wheels[0] ->setVelocity(10.0); 
      wheels[1] ->setVelocity(0.0);
      if (sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0 && sensorValues[7]==0){
        wheels[0] ->setVelocity(10.0); 
        wheels[1] ->setVelocity(0.0);
      }else{
        wheels[0] ->setVelocity(10.0); 
        wheels[1] ->setVelocity(0.0);
      }
    }
    //T junction
    else if(sensorValues[8]==0 && sensorValues[9]==0 && box==0){
       move_forward(0.1);
       turn(1,-1);  
       TJ+=1;
    //in a radius inside a maze
     }
     else if (sensorValues[0]==0 && sensorValues[1]==0 && sensorValues[2]==0 && sensorValues[3]==0  && TJ>0){
     radius_count+=1;
     T=1;
  if(T==1 && (sensorValues[10]>1.6 && sensorValues[10]<2.8)){
    box_count=radius_count;
    std::cout<<"INNNNNNNNNNN"<<std::endl;
    move_forward(0.1);
    turn(3.2,1);
    move_back(0.06);
    base->setPosition(0);
    wheels[0] ->setVelocity(0); 
    wheels[1] ->setVelocity(0);
    flag=0;
    T=0;
    }
    
    else if(box_count<4){
    move_forward(0.12);}
    
    else if (box==1) {move_forward(0.16);
    turn(2.2,1);
    TJ=0;
    first=1;
    break;}    
  }
  //cheking robot whther robot is ready to exit
  else if (sensorValues[9]==0 && TJ>0 && radius_count>2){
  if (box_count>0){
  move_forward(0.12);
  turn(1.7,-1);
  TJ=0;
  first=1;
  break;
  }
  else if (box_count==0){
  move_forward(0.1);
  turn(0.2,1);
  }
  }
  //general pid for line following
    else{
      double offset = PID_calc();
      double leftSpeed =  initialSpeed-offset;
      double rightSpeed = initialSpeed+offset;
    
      double lefSpeed = MSpeed(leftSpeed) ;
      double rigSpeed = MSpeed(rightSpeed) ;
         
      wheels[0] ->setVelocity(lefSpeed); 
      wheels[1] ->setVelocity(rigSpeed);
    }
//wall following part        
  DistanceSensor *ds[2];
  char dsNames[2][10] = {"ds_right","ds_left"};
  
 for (int i = 0; i < 2 ; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
    }
    
    int r_value=ds[0]->getValue();
    int l_value=ds[1]->getValue();
    
if(r_value<2000 || l_value<2000  ){
    
    while (robot->step(TIME_STEP) != -1) {
    float r_value=ds[0]->getValue();
    float l_value=ds[1]->getValue();
    
   
    if (r_value==2000 and l_value==2000){
        break;
        }
    else if (r_value<2000){
        r_error=r_value-c;
        
        p=r_error*kP;
        d=(r_error-pr_error)*kD;
        i=total_er*kI;
        
        pid=p+d+i;
        
        pr_error=r_error;
        total_er+=r_error;
        
        
        motor_run(5-pid,5+pid);
        wheels[0]->setVelocity(speed[1]);
        wheels[1]->setVelocity(speed[0]);
        
        }
    else{
        l_error=l_value-c;
        
        p=l_error*kP;
        d=(l_error-pl_error)*kD;
        i=total_el*kI;
        
        pid=p+d+i;
        
        pl_error=l_error;
        total_el+=l_error;
        
        
        motor_run(5+pid,5-pid);
        wheels[0]->setVelocity(speed[1]);
        wheels[1]->setVelocity(speed[0]);
        }         
  };        
    }             
//////checking for the box box lifting part
if(TJ>0 && sensorValues[11]>2.65){
    box=1;
    wheels[0] ->setVelocity(0); 
    wheels[1] ->setVelocity(0);
  
  Motor *left;
  left=robot->getMotor("left");
  double left_rotate=0.0 ;
  
  Motor *right;
  right=robot->getMotor("right");
  double right_rotate=0.0 ;
  
  Motor *left_g;
  left_g=robot->getMotor("left_g");
  double left_g_rotate=0.0 ;
  
  Motor *right_g;
  right_g=robot->getMotor("right_g");
  double right_g_rotate=0.0 ;
  
  right->setPosition(-0.4);
  left->setPosition(-0.4);
  
  while (robot->step(TIME_STEP) != -1) {
  
    if (left_rotate<0.03){
    left_rotate+=0.0001;
  }
  left->setPosition(left_rotate);

  
  if (right_rotate<0.03){
    right_rotate+=0.0001;
  }
  right->setPosition(right_rotate);
  if ((left_rotate>-0.35) && (right_rotate>-0.35)){
  break;
  }
  }
  std::string str=color();
  while (robot->step(TIME_STEP) != -1){
  if (base_rotate<1){
    base_rotate+=0.005;
  }
  base->setPosition(base_rotate);
  if (base_rotate>0.6){
    break;
  }
  }  
  while (robot->step(TIME_STEP) != -1) {
  
    if (left_g_rotate<2){
    left_g_rotate+=0.01;
  }
  left_g->setPosition(left_g_rotate);

  
  if (right_g_rotate<2){
    right_g_rotate+=0.01;
  }
  right_g->setPosition(right_g_rotate);
  if ((left_g_rotate>1.7) && (right_g_rotate>1.7)){
  break;
  }
  }
  std::string str_1=color();
  turn_back();
  right->setPosition(-0.5);
  left->setPosition(-0.5);
  if (box_count==4){
  turn(1.0,-1);}
  else{
  turn(3.7,1);}
  if (str=="green"){
  p1=2;}
  else if (str=="blue"){
  p1=3;} 
  else if (str=="red"){
  p1=1;}  
  if (str_1=="green"){
  p2=2;}
  else if (str_1=="blue"){
  p2=3;} 
  else if (str_1=="red"){
  p2=1;}} 
//Print the binary output of sensors
    std::cout<<"left_most = "<<sensorValues[8]<<"  ";
    std::cout<<"left4 = "<<sensorValues[0]<<"  ";
    std::cout<<"left3 = "<<sensorValues[1]<<"  ";
    std::cout<<"left2 = "<<sensorValues[2]<<"  ";
    std::cout<<"left1 = "<<sensorValues[3]<<"  ";
    std::cout<<"right1 = "<<sensorValues[4]<<"  ";
    std::cout<<"right2 = "<<sensorValues[5]<<"  ";
    std::cout<<"right3 = "<<sensorValues[6]<<"  ";
    std::cout<<"right4 = "<<sensorValues[7]<<"  ";
    std::cout<<"right_most = "<<sensorValues[9]<<"  ";
    std::cout<<"CURRENT QUADRANT IS "<<radius_count<<"  "; 
    std::cout<<"--------------------------------------------------------------------------------------"<<std::endl;     
  };
////////line following loop after the maze////  
if (first==1){

while (robot->step(TIME_STEP) != -1){
  for (int i = 0; i < 12 ; i++){
      sensorValues[i] = ir[i]->getValue();
    }    
    convert(); //Covert sensor values to binary
    
    /////white line before the gate
    if (pillars>=1 && sensorValues[1]==0 && sensorValues[6]==0){
    /////if number of pillars are odd, correct the path
     if (pillars%2==1){
     
     turn(5.6,1);
     move_forward(0.21);
     if (abs(p1-p2)%2==0){
     turn(2.85,-1);}
     else{turn(2.85,1);}
     error=1;
     }
    //////robot is in right path now :)  
     else{
     wheels[0] ->setVelocity(0); 
     wheels[1] ->setVelocity(0);
     

     while (robot->step(TIME_STEP) != -1){
     float value = ir[11]->getValue();
     wheels[0] ->setVelocity(0); 
     wheels[1] ->setVelocity(0);
     if(value>2.0){
     break;
     }}
     
     while (robot->step(TIME_STEP) != -1){
     float value = ir[11]->getValue();
     wheels[0] ->setVelocity(0); 
     wheels[1] ->setVelocity(0);
     if(value<0.5){
     break;
     }}
     base->setPosition(1);
     move_forward(0.2);
     move_fast_forward(0.4);
     move_forward(0.6);
     gate=1;
     }    
   }
   
   /////moving back if there is an error
   if (error==1 && sensorValues[11]>2.7){
   move_forward(1.5);
   }
    //Left sidejunction
    if (sensorValues[9]==1 && sensorValues[8]==0){
      wheels[0] ->setVelocity(0.0); 
      wheels[1] ->setVelocity(10.0);
      if (sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0 && sensorValues[7]==0){
        wheels[0] ->setVelocity(0.0); 
        wheels[1] ->setVelocity(10.0);
      }else{
        wheels[0] ->setVelocity(0.0); 
        wheels[1] ->setVelocity(10.0);
      }
    }
    //Right side junction
    else if(sensorValues[9]==0 && sensorValues[8]==1){
      wheels[0] ->setVelocity(10.0); 
      wheels[1] ->setVelocity(0.0);
      if (sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0 && sensorValues[7]==0){
        wheels[0] ->setVelocity(10.0); 
        wheels[1] ->setVelocity(0.0);
      }else{
        wheels[0] ->setVelocity(10.0); 
        wheels[1] ->setVelocity(0.0);
      }
    }
    
 // At T junction turn in the ramp
    else if(sensorValues[8]==0 && sensorValues[9]==0 && ramp==0){
      ramp=1;   
      move_fast_forward(0.13); 
      if (abs(p1-p2)%2==0){ 
       turn(2.8,1);}
       else{turn(2.8,-1);
       }
       move_forward(0.35);
     } 
    ///general line following 
    else{
      double offset = PID_calc();
      double leftSpeed =  initialSpeed-offset;
      double rightSpeed = initialSpeed+offset;
    
      double lefSpeed = MSpeed(leftSpeed) ;
      double rigSpeed = MSpeed(rightSpeed) ;
         
      wheels[0] ->setVelocity(lefSpeed); 
      wheels[1] ->setVelocity(rigSpeed);
   
    }
    
    ////counting pillars
    DistanceSensor *ds[2];
  char dsNames[2][10] = {"ds_right","ds_left"};
    for (int i = 0; i < 2 ; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
    }
    if(ramp==1){
    float r_value=ds[0]->getValue();
    float l_value=ds[1]->getValue();
      if(r_value<2000){
        std::cout<<"pillar"<<std::endl;
        pillars+=1;
        move_forward(0.1);
      }  
      if(l_value<2000){
        std::cout<<"pillar"<<std::endl;
        pillars+=1;
        move_forward(0.1);
      }
    } 
     if (gate==1 && sensorValues[8]==0 && sensorValues[9]==0){
     move_forward(0.25);
     wheels[0] ->setVelocity(0); 
     wheels[1] ->setVelocity(0);
     break;
     }  
     
//Print the binary output of sensors
    std::cout<<"df_left_most = "<<sensorValues[8]<<"  ";
    std::cout<<"df_left4 = "<<sensorValues[0]<<"  ";
    std::cout<<"df_left3 = "<<sensorValues[1]<<"  ";
    std::cout<<"df_left2 = "<<sensorValues[2]<<"  ";
    std::cout<<"df_left1 = "<<sensorValues[3]<<"  ";
    std::cout<<"df_right1 = "<<sensorValues[4]<<"  ";
    std::cout<<"df_right2 = "<<sensorValues[5]<<"  ";
    std::cout<<"df_right3 = "<<sensorValues[6]<<"  ";
    std::cout<<"df_right4 = "<<sensorValues[7]<<"  ";
    std::cout<<"df_right_most = "<<sensorValues[9]<<"  ";
    std::cout<<"pillars_count  "<<pillars<<"  ";
    std::cout<<"--------------------------------------------------------------------------------------"<<std::endl;           
  };} 
  delete robot;
  return 0;
  }