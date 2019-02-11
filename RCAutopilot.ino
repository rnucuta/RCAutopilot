//Libraries
#include <Servo.h>
#include <PID_v1.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!


//for theoretical PID that isn't implemented in this case. the code for it is commented out for the propeller driver
double Setpoint ; // will be the desired value
double Input; // photo sensor
double Output ; //LED
//PID parameters
double Kp=0, Ki=10, Kd=0; 


// i2c imu sensor init
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


//init 2 servo motors and esc prop motor
Servo myservo;
Servo myservoE;
Servo ESC1;


//imu variables
#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5



// You can also use software SPI
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS, LSM9DS1_MCS);
// Or hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);


//all other relevant variables 
//imu variables
double a_x, a_y, a_z, g_x, g_y, g_z, m_x, m_y, m_z;
//past positions of the airplane array. Only stores the first 100 variables then rewrites them
double past_positions[100][3];
//current position
double x=0;
double y=0;
double z=0;

//total time elapsed since beginning
double timee=0;
//change in time since past iteration
double diff_time=0;
//velocity components
double v_x=0, v_y=0, v_z=0;
int counter = 0;
int posAil = 0;   

//array of goal locations you want the airplane to go to
int goal[2][3] = {{0, 100, 50}, {0, 0, 50}};


void setSpeed(int speed){
  int angle = map(speed, 0, 100, 0, 180); //Sets servo positions to different speeds 
  ESC1.write(angle);
}

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}
int goalIndex=1;


void checkAtGoal(){
  if((5>x-goal[goalIndex][0] && -5<x-goal[goalIndex][0])&&(5>y-goal[goalIndex][1] && -5<y-goal[goalIndex][1])&&(5>z-goal[goalIndex][2] && -5<z-goal[goalIndex][2]))
  {
  goalIndex++;
  goalIndex = goalIndex%(sizeof(goal) / sizeof(goal[0]));
  }
}

void setup() 
{
  //setup for all teh different pins
  setupSensor();
  ESC1.attach(9);
  myservo.attach(8);
  myservoE.attach(7);
  myservoE.write(90);myservo.write(90);
  setSpeed(0); //Sets speed variable 
  delay(1000);


//  //if you wanted a pid for the motor, you could set it up here:
//  Setpoint = 75;
//  //Turn the PID on
//  myPID.SetMode(AUTOMATIC);
//  //Adjust PID values
//  myPID.SetTunings(Kp, Ki, Kd);
}

void loop() 
{
  //Controls IMU stuff
  checkAtGoal();
  lsm.read();  /* ask it to read in the data */ 

  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp); 

  a_x = a.acceleration.x;
  a_y = a.acceleration.y;
  a_z = a.acceleration.z;

  m_x = m.magnetic.x;
  m_y = m.magnetic.y;
  m_z = m.magnetic.z;     

  g_x = g.gyro.x;   
  g_y = g.gyro.y;      
  g_z = g.gyro.z;     
  
  delay(200);


  //Countrols math stuff
  
  diff_time = timee-millis();
  timee = millis(); //records time that arduino program has been running into time variable in units of milliseconds
  

  velocity();
  positions();
  recordPosition();
  

  //Controls Rudder Stuff
  rudder_controller(rudd_direction(angle_rudder(), angle_rudder2(angle_rudder())));

  //Controls Elevator stuff
  elevator_controller(elev_direction()*angle_rudder());


  //Controls prop motor stuff
  //speed from 1-100
  setSpeed(75);

}


//throwaway vectors for the vector from the position to the goal locaiton
int v[3] ={0,0,0};
int w[3] ={0,0,0};


//tells the elevator to change angle
void elevator_controller(int pos){
  pos+=90;
  myservoE.write(pos);

}
//figures out if elevator should go up or down
int angle_elevator(){
  double trans[3] = {x-goal[goalIndex][0], y-goal[goalIndex][1], z-goal[goalIndex][2]};
  //go up
  //creates plane of the orientation vector and the vector from the goal location to the current position
  if((-1*trans[2]*v_y -1*trans[1]*v_z)>5){  
    return -1;
  }
  //go down
  else if((-1*trans[2]*v_y -1*trans[1]*v_z)<-5){  
    return 1;
  }
  else {return 0;}
}
//finds the projection of a point onto a plane to find the angle to turn the elevator
int elev_direction(){
  double trans[3] = {x-goal[goalIndex][0], y-goal[goalIndex][1], z-goal[goalIndex][2]};  

  double d = -1*trans[2]*v_y/sqrt(pow(v_z,2)+pow(v_y,2)) -1*trans[1]*v_z/sqrt(pow(v_z,2)+pow(v_y,2));

  double p[3] = {v_x, v_y+d*trans[1], v_z*trans[2]};

  int angle = 57.3*acos((v_x*p[0]+v_y*p[1]+v_z*p[2])/(sqrt(v_x*v_x+v_y*v_y+v_z*v_z)*sqrt(pow(p[0], 2)+pow(p[1], 2)+pow(p[2], 2))));

  return angle;
}

//tells the rudder to turn a certain degree angle from 0-180
void rudder_controller(int pos) {
  pos+=90;
  myservo.write(pos); 
}
//returns angle to turn to if not between a +- 5 degree radius of the goal vecotr
int angle_rudder(){
    w[0]=goal[goalIndex][0]-x;
    w[1]=goal[goalIndex][1]-y;
    int angle = 57.3*acos((v_x*w[0]+v_y*w[1]+v_z*w[2])/(sqrt(v_x*v_x+v_y*v_y)*sqrt(pow(w[0], 2)+pow(w[1], 2))));
    if(angle<5&&angle>-5){return angle;}
    else{return 0;}
}
//rotate orientation vector hypothetically to determine correct way to turn (left or right)
int angle_rudder2(int ang){
    double vx=v_x*cos(1/57.3 * ang);
    double vy=v_y*sin(1/57.3 * ang);
    v[0]=goal[goalIndex][0]-x;
    v[1]=goal[goalIndex][1]-y;
    int angle = 57.3*acos((vx*v[0]+vy*v[1])/(sqrt(vx*vx+vy*vy)*sqrt(v[0]*v[0]+v[1]*v[1])));
    return angle;
}
//tells to turn left or right
int rudd_direction(int a1, int a2){
  if (a1!=0){
    if(a1>a2){
       return a1;
    }
    else{
      return -1*a1;  
    }
  }
  else{return 0;}
}

void velocity(){
  //calculates vector components of current velocity using acceleration 
  v_x = v_x + a_x*diff_time/1000000;
  v_y = v_y + a_y*diff_time/1000000;
  v_z = v_z + a_z*diff_time/1000000;

}



void positions(){
 //stores current position using xf = xi+vit+1/2at^2
 x = x+v_x*diff_time+0.5*a_x*diff_time*diff_time/1000000;
 y = y+v_y*diff_time+0.5*a_y*diff_time*diff_time/1000000;
 z = z+v_z*diff_time+0.5*a_z*diff_time*diff_time/1000000;
}



void recordPosition(){
  //records past positions in a multi-dimensional array called past_positions
  past_positions[counter][0] = x;
  past_positions[counter][1] = y;
  past_positions[counter][2] = z;
  counter++;
  int len_past = sizeof(past_positions)/sizeof(past_positions[0]);
  if(counter==len_past){counter=0;}
}
