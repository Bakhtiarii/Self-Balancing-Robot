#include <PID_v1.h>
#include "I2Cdev.h"
#include "Wire.h"

#include <AccelStepper.h>
#include "MPU6050_6Axis_MotionApps20.h"

AccelStepper stepper(1,3,2); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepper2(1,5,4);

MPU6050 mpu(0x68);

bool dmpReady = false; 
uint8_t deviceStatus; //device status , 0 = success , 
uint16_t packetSize; //expected DMP packet size (defult 42) -- ?
uint16_t fifoCount; //count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO buffer storage 

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float gyro_data;
bool ticket = 1;


//PID Variables
double setpoint = 0;
double Kp = 6;
double Kd = 0.0005;
double Ki = 0;
double input;
double output;
int i;
long long timei;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() 
{
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 48; // 400kHz I2C clock (200kHz .if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  
  Serial.begin(115200);
  setup_gyro();
  
  stepper.setMaxSpeed(100);
  stepper.setSpeed(10);
  stepper2.setMaxSpeed(100);
  stepper2.setSpeed(10);

  //PID Setup
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-100,100);  
  //timei = millis(); 
}

void loop() 
{
  read_gyro();
  pid.Compute();
  
  Serial.print(input);
  Serial.print("====>");
  Serial.println(output); 
  output *= 1; 
  /* 
  if(millis() - timei > 2000)
  {
    i += 10;
    if(i > 100) i=0;
    Serial.println(i);      
    timei = millis();
  }
  stepper.setSpeed(i);
  stepper2.setSpeed(i);      
  stepper.runSpeed();
  stepper2.runSpeed();
 */
  if (gyro_data > 20)
  {
    stepper.setSpeed(-80);
    stepper2.setSpeed(-80);
  }
  else if (gyro_data < -20)
  { 
    stepper.setSpeed(80);
    stepper2.setSpeed(80);
  }
  else if(gyro_data > 2)
  {
    stepper.setSpeed(output);
    stepper2.setSpeed(output);    
  }
  else if(gyro_data < -2)
  {
    stepper.setSpeed(output);
    stepper2.setSpeed(output);    
  }
  else
  { 
    stepper.setSpeed(0);
    stepper2.setSpeed(0);
  }
  
  stepper.runSpeed();
  stepper2.runSpeed();
}

void setup_gyro()
{
  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("MPU6050 connection test successed ") : F("MPU6050 connection test failed"));
  deviceStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(2); 
  mpu.setYGyroOffset(-23); 
  mpu.setZGyroOffset(9);
  mpu.setXAccelOffset(2264);
  mpu.setYAccelOffset(1274);
  mpu.setZAccelOffset(1352);

  if (deviceStatus == 0) 
  {
    //Serial.println("DMP initialization success, now enable DMP for use");
    mpu.setDMPEnabled(true);

    //wait for first interrupt . currently just leave it false automatically  
    dmpReady = true;
    //Serial.println("DMP is ready to use.");
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else 
  {
    //ERROR! , device status !=0 when initializing DMP
    Serial.print("DMP initialization failed when using MPU6050 library:");
    if (deviceStatus == 1)      
      Serial.println(" intial memory load failed");
    else if (deviceStatus == 2)      
      Serial.println(" failed to update DMP configuration");
    else 
    {
      Serial.print(" unknow error with code: ");
      Serial.println(deviceStatus);
    }
  }  
}

void read_gyro()
{
  mpu.resetFIFO();
  //if DMP not ready don't do anything 
  if (!dmpReady) 
  {
    if (ticket == 1)
    {
     Serial.println("MAIN LOOP: DMP disabled");
     ticket = 0 ;
    } 
    else
      return; 
  } 
  else 
  {
    if (fifoCount == 1024) 
    {
      mpu.resetFIFO();
      Serial.println("FIFO overflow");
    } 
    else 
    {
      //waiting until get enough
      while (fifoCount < packetSize) 
        fifoCount = mpu.getFIFOCount();    
      
      mpu.getFIFOBytes(fifoBuffer,packetSize);
      fifoCount -= packetSize ;
  
      if (fifoCount > 2){
          ////// clear fifo buffer
      }
      
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      gyro_data = ypr[1] * 180/M_PI;                   
      input = gyro_data;      
    }
  }
}
