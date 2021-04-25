
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;

int input_YAW;      // channel 4 of the receiver
int input_PITCH;    // channel 2 of the receiver 
int input_ROLL;     // channel 1 of the receiver
int input_THROTTLE; // channel 3 of the receiver
int safety_switch;  // channel 5 of the receiver 
int mode_selection; // channel 6 of the reciever 

/////////////////////// defining Servo function for four ESC //////////////////////////

Servo ESC1;
Servo ESC2; 
Servo ESC3; 
Servo ESC4;


//////////////////////////////// Timer width values of RC Intput Signals //////////////////////////////////

unsigned long timer1,timer2,timer3,timer4,timer5,timer6,time_z;

//////////////////////////////// timer for loop control //////////////////////////////////

float elasped_time,time,timePrev;

//////////////////////////////// input variable to store the previous variable of RC channel signals //////////////////////////////////

byte CH1_state,CH2_state,CH3_state,CH4_state,CH5_state,CH6_state;

//////////////////////////////PD FOR ROLL///////////////////////////

float roll_p=0,roll_d=0;                                 // defining proportional and derivative action input variable 
float roll_PD=0;                                         // defining proportional derivative controller input variable 
float roll_error, roll_previous_error;                   // defining error and previous error of roll angle 
double roll_kp=3.2;                                      // defining proportional gain of PD controller 
double roll_kd=1.4;                                      // defining derivative gain of PD controller
float roll_SP=0;                                         // defining setpoint of roll controller 

//////////////////////////////PD FOR PITCH//////////////////////////

float pitch_p=0,pitch_d=0;                              // defining proportional and derivative action input variable 
float pitch_PD=0;                                        // defining proportional derivative controller input variable 
float pitch_error, Pitch_previous_error;                 // defining error and previous error of pitch angle
double pitch_kp=roll_kp;                                 // defining proportional gain of PD controller 
double pitch_kd=roll_kd;                                 // defining derivative gain of PD controller
float pitch_SP=0;                                        // defining setpoint of pitch controller 

//////////////////////////////PD FOR YAW//////////////////////////
 
float yaw_p=0,yaw_d=0;                                   // defining proportional and derivative action input variable                            
float yaw_PD=0;                                          // defining proportional and derivative action input variable 
float yaw_error, yaw_previous_error;                     // defining error and previous error of yaw angle
double yaw_kp=2;                                         // defining proportional gain of PD controller 
double yaw_kd=0;                                         // defining derivative gain of PD controller
float yaw_SP =0;                                         // defining setpoint of yaw controller                           

/////////////////////////// Gyro variables ////////////////////////

int gyro_error=0;                         // I use this variable to only calculate once the gyro data error
float G_X, G_Y, G_Z;                      // GyroScope Raw Data definations 
float GX_angle,GY_angle,GZ_angle;         // Gyro Angle of X, Y & Z 
float GX_error,GY_error;                  // storing the initial gyro data error

////////////////////////// Accelration variables ///////////////////

int acc_error=0;                         //  I use this variable to only calculate once the Acc data error
float rad_to_deg = 180/3.141592654;      //This value is for pasing from radians to degrees values
float A_X, A_Y, A_Z;                     //Here we store the raw data read 
float AX_angle, AY_angle;                //Here we store the angle value obtained with Acc data
float AX_error, AY_error;                //Here we store the initial Acc data error

///////////////////////////////// Euler Angles defination //////////////////////////////////////////////////

float Roll_angle, Pitch_angle, Yaw_angle; 
float altitude, altitude_SP=0;                   // define altitude variable and altitude setpoint
                   
//////////////////////////////////////////////////////////////////////

void setup() {
  //This part is for the PWM input interruptions  
  
   PCICR |= (1 << PCIE0);   // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);  // set PCINT0 (digital input 8) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT1);  // set PCINT1 (digital input 9)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT2);  // set PCINT2 (digital input 10)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT3);  // set PCINT3 (digital input 11)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT4);  // set PCINT4 (digital input 12)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT5);  // set PCINT5 (digital input 13)to trigger an interrupt on state change
  
  ESC1.attach(4);           // left front motor
  ESC2.attach(5);           // left back motor  
  ESC3.attach(7);           // right front motor  
  ESC4.attach(6);           // right back motor
  
  ////  Setting the ESCS TO ZERO AS TO avoid any ESC entering into config mode ///////

  ESC1.writeMicroseconds(1000); 
  ESC2.writeMicroseconds(1000); 
  ESC3.writeMicroseconds(1000); 
  ESC4.writeMicroseconds(1000); 


  Wire.begin();                           //STARTS I2C comunication
  unsigned status;
  status = bme.begin(0x76); 
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68)              
  Wire.write(0x6B);                       //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(true);             //end the transmission
  //Gyro config
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68) 
  Wire.write(0x1B);                       //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                       //Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);             //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(0x68);           //Start communication with the address found during search.
  Wire.write(0x1C);                       //We want to write to the ACCEL_CONFIG register
  Wire.write(0x10);                       //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true); 

  Serial.begin(9600);                     //Remember to set this same baud rate to the serial monitor  
  time = millis();                        //Start counting time in milliseconds




/*Here we calculate the gyro data error before we start the loop
 * I make the mean of 200 values, that should be enough*/
  if(gyro_error==0)
  {
    for(int i=0; i<200; i++)
    {
      Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68) 
      Wire.write(0x43);                        //First adress of the Gyro data
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,6,true);           //We ask for just 4 registers 
         
      G_X=Wire.read()<<8|Wire.read();          // Read Gyro Raw Data of X-axis 
      G_Y=Wire.read()<<8|Wire.read();          // Read Gyro Raw Data of Y-axis 
      G_Z=Wire.read()<<8|Wire.read();          // Read Gyro Raw Data of Z-axis 
      
      GX_error = GX_error + (G_X/65.5); 
      GY_error = GY_error + (G_Y/65.5);
      GZ_error = GZ_error + (G_Z/65.5);
      if(i==199)
      {
        GX_error = GX_error/200;
        GY_error= GY_error/200;
        gyro_error=1;
      }
    }
   }
}
void loop(){
  if(mode_selection>1900)                                  // if the channel 6 switch is high 
  {
    automatic_mode();
    }
  else
  {
    Manual_Mode();
     }
   }
  
void Manual_Mode() {
  
  timePrev=time;                                                // defining previous time.
  time = millis();                                              // read the current time
  elasped_time = (time - timePrev) / 1000;                       // calculating loop sample time.

 gyro_request(G_X,G_Y,G_Z);                                   //  read GyroScope Value
 acc_request(A_X,A_Y,A_Z);                                    //  read Acceleration value
 
 ////////////////////////////////////////////// Evaluate Accleration ANGLE /////////////////////////////////////////////////
 
 AX_angle = ((atan(-1*(A_Y)/A_Z))*rad_to_deg) - AX_error;                          // Evaluate X Accleration Angle 
 AY_angle = (atan((A_X)/sqrt(pow((A_Y),2) + pow((A_Z),2)))*rad_to_deg) - AY_error; // Evaluate Y Accleration Angle     
 
//////////////////////////////////////////////// GYRO ANGLE ///////////////////////////////////////////////

 GX_angle = G_X*elasped_time;                                                       // Integrating Anglular Rate of X axis;
 GY_angle = G_Y*elasped_time;                                                       // Integrating Anglular Rate of Y axis;
 GZ_angle = G_Z*elasped_time;                                                       // Integrating Anglular Rate of Z axis;
 
 ////////////////////////////////////// QUADCOPTER ACTUAL ANGLE /////////////////////////////////////
 
 Roll_angle = 0.98 *(Roll_angle + GX_angle) + 0.02*AX_angle;                       // ROLL ANGLE EVALUATION
 Pitch_angle = 0.98 *(Pitch_angle + GY_angle) + 0.02*AY_angle;                     // PITCH ANGLE EVALUATION 
 Yaw_angle=GZ_angle;                                                               // YAW ANGLE EVALUATION
  ////////////////////////////////////  LIMITING YAW ANGLE      ///////////////////////////////////////
 if(Yaw_angle>360)
 {Yaw_angle=Yaw_angle-360}
 if(Yaw_angle<360)
 {Yaw_angle=Yaw_angle+360;}
//////////////////////////////////////////////// setpoint identification ////////////////////////////////////////////


roll_SP = map(input_ROLL,1000,2000,-30,30);
pitch_SP = map(input_PITCH,1000,2000,-30,30);
yaw_SP= map(input_YAW,1000,2000,-60,60);

//////////////////////////////////////////////// Error Calculations ////////////////////////////////////


roll_error = roll_SP - Roll_angle;
pitch_error = pitch_SP- Pitch_angle;  
Yaw_error= yaw_SP-Yaw_angle;

/////////////////////////////Proportional Gain ///////////////////////////////////////////////

roll_p = roll_kp*roll_error;
pitch_p = pitch_kp*pitch_error;
yaw_p = yaw_kp*yaw_error;

///////////////////////////////// Derivative Gain Calculations /////////////////////////////////////////////

roll_d = roll_kd*((roll_error - roll_previous_error)/elasped_time);
pitch_d = pitch_kd*((pitch_error - pitch_previous_error)/elasped_time);
yaw_d = yaw_kd*((yaw_error - yaw_previous_error)/elasped_time);

//////////////////////////////////////////////// PD calculations ////////////////////////////////////////////

roll_PD= roll_d +roll_p;
pitch_PD=pitch_d +pitch_p;
yaw_PD=pitch_d +yaw_p;

///////////////////////////////////////////////// LIMITING PID OUTPUT ///////////////////////////////////////
roll_PD=constrain(roll_PID,-300,300)
pitch_PD=constrain(roll_PID,-300,300)
yaw_PD=constrain(roll_PID,-300,300)

////////////////////////////////////////////////// ESC MIXER ////////////////////////////////////////////////

ESC1 = input_THROTTLE + roll_PD - pitch_PD - yaw_PD;
ESC2 = input_THROTTLE + roll_PD + pitch_PD + yaw_PD; 
ESC3 = input_THROTTLE - roll_PD + pitch_PD - yaw_PD;
ESC4 = input_THROTTLE - roll_PD - pitch_PD + yaw_PD;  

///////////////////////////////// LIMITING MOTOR OUTPUT /////////////////////////////////////////
ESC1= constrain(ESC1,1100,2000);
ESC2= constrain(ESC2,1100,2000);
ESC3= constrain(ESC3,1100,2000);
ESC4= constrain(ESC4,1100,2000);
//
//////////////////////////////// STORING PREVIOUS ERROR /////////////////////////////////////////
roll_previous_error = roll_error; 
pitch_previous_error = pitch_error; 
yaw_previous_error= yaw_error;

//////////////////////////////// SAFETY FEATURE ///////////////////////////////////////////////////
if(safety_switch>1900)
{
ESC1.writeMicroseconds(1000);  
ESC2.writeMicroseconds(1000); 
ESC3.writeMicroseconds(1000); 
ESC4.writeMicroseconds(1000); 
}

 if (safety_switch<1100)

  {  
    ESC1.writeMicroseconds(ESC4); 
    ESC2.writeMicroseconds(ESC2); 
    ESC3.writeMicroseconds(ESC3); 
    ESC4.writeMicroseconds(ESC4);   
    
  }

}


void automatic_mode(){
  time_z=millis();
  timePrev=time;                                                // defining previous time.
  time = millis();                                              // read the current time
  elasped_time = (time - timePrev) / 1000;                       // calculating loop sample time.

  ///////////////////////////// Program Instructions for Quadcopter ////////////////////////////
switch(i)
{
  case 1:  roll_SP=0;
           pitch_SP=0;
           yaw_SP=0;
           altitude_SP=500;
           break;
  case 2:  roll_SP=30;
           pitch_SP=0;
           yaw_SP=0;
           altitude_SP=500;
           break;
  case 3:  roll_SP=-30;
           pitch_SP=0;
           yaw_SP=0;
           altitude_SP=500;
           break;                   
  case 4:  roll_SP=0;
           pitch_SP=30;
           yaw_SP=0;
           altitude_SP=500;
           break;
  case 5:  roll_SP=0;
           pitch_SP=-30;
           yaw_SP=0;
           altitude_SP=500; 
           break; 
  case 6:  roll_SP=0;
           pitch_SP=0;
           yaw_SP=30;
           altitude_SP=500; 
           break;
  case 7:  roll_SP=0;
           pitch_SP=0;
           yaw_SP=-30;
           altitude_SP=500;   
           break;
  case 8:  roll_SP=0;
           pitch_SP=0;
           yaw_SP=0;
           altitude_SP=0;  
           break;                                    
}

 altitude=Wire.requestFrom(8,6);                              //  Request current alitutde  From Nano Arduino
 gyro_request(G_X,G_Y,G_Z);                                   //  read GyroScope Value
 acc_request(A_X,A_Y,A_Z);                                    //  read Acceleration value
 
 ////////////////////////////////////////////// Evaluate Accleration ANGLE /////////////////////////////////////////////////
 
 AX_angle = ((atan((A_X)/sqrt(pow((A_Y),2) + pow((A_Z),2)))*rad_to_deg) - AX_error;                          // Evaluate X Accleration Angle 
 AY_angle = (atan((A_Y)/sqrt(pow((A_X),2) + pow((A_Z),2)))*rad_to_deg) - AY_error;                          // Evaluate Y Accleration Angle     
 
//////////////////////////////////////////////// GYRO ANGLE ///////////////////////////////////////////////

 GX_angle = G_X*elasped_time;                                                       // Integrating Anglular Rate of X axis;
 GY_angle = G_Y*elasped_time;                                                       // Integrating Anglular Rate of Y axis;
 GZ_angle = G_Z*elasped_time;                                                       // Integrating Anglular Rate of Z axis;
 
 ////////////////////////////////////// QUADCOPTER ACTUAL ANGLE /////////////////////////////////////
 
 Roll_angle = 0.98 *(Roll_angle + GX_angle) + 0.02*AX_angle;                       // ROLL ANGLE EVALUATION
 Pitch_angle = 0.98 *(Pitch_angle + GY_angle) + 0.02*AY_angle;                     // PITCH ANGLE EVALUATION 
 Yaw_angle=GZ_angle;                                                               // YAW ANGLE EVALUATION
 ////////////////////////////////////  LIMITING YAW ANGLE      ///////////////////////////////////////
 if(Yaw_angle>360)
 {Yaw_angle=Yaw_angle-360}
 if(Yaw_angle<360)
 {Yaw_angle=Yaw_angle+360;}
 
//////////////////////////////////////////////// Mapping inputs ////////////////////////////////////////////
input_THROTTLE= map(altitude,0,500,1000,2000);
input_ROLL = map(roll_SP,-30,30,1000,2000);
input_PITCH = map(pitch_SP,-30,30,1000,2000);
input_YAW = map(yaw_SP;,-60,60,-1000,2000);

//////////////////////////////////////////////// Error Calculations ////////////////////////////////////

roll_error = roll_SP - Roll_angle;
pitch_error = pitch_SP- Pitch_angle;  
Yaw_error= yaw_SP-Yaw_angle;

/////////////////////////////Proportional Gain ///////////////////////////////////////////////

roll_p = roll_kp*roll_error;
pitch_p = pitch_kp*pitch_error;
yaw_p = yaw_kp*yaw_error;

///////////////////////////////// Derivative Gain Calculations /////////////////////////////////////////////

roll_d = roll_kd*((roll_error - roll_previous_error)/elasped_time);
pitch_d = pitch_kd*((pitch_error - pitch_previous_error)/elasped_time);
yaw_d = yaw_kd*((yaw_error - yaw_previous_error)/elasped_time);

//////////////////////////////////////////////// PD calculations ////////////////////////////////////////////

roll_PD= roll_d +roll_p;
pitch_PD=pitch_d +pitch_p;
yaw_PD=pitch_d +yaw_p;

///////////////////////////////////////////////// LIMITING PD OUTPUT ///////////////////////////////////////
roll_PD=constrain(roll_PID,-300,300)
pitch_PD=constrain(roll_PID,-300,300)
yaw_PD=constrain(roll_PID,-300,300)

////////////////////////////////////////////////// ESC MIXER ////////////////////////////////////////////////

ESC1 = input_THROTTLE + roll_PD - pitch_PD - yaw_PD;
ESC2 = input_THROTTLE + roll_PD + pitch_PD + yaw_PD; 
ESC3 = input_THROTTLE - roll_PD + pitch_PD - yaw_PD;
ESC4 = input_THROTTLE - roll_PD - pitch_PD + yaw_PD;  


///////////////////////////////// LIMITING MOTOR OUTPUT /////////////////////////////////////////
ESC1= constrain(ESC1,1100,2000);
ESC2= constrain(ESC2,1100,2000);
ESC3= constrain(ESC3,1100,2000);
ESC4= constrain(ESC4,1100,2000);
//
//////////////////////////////// STORING PREVIOUS ERROR /////////////////////////////////////////
roll_previous_error = roll_error; 
pitch_previous_error = pitch_error; 
yaw_previous_error= yaw_error;
/////////////////////////////////  Weather Sensor       ///////////////////////////////////////////

weather_sensor();

//////////////////////////////// SAFETY FEATURE ///////////////////////////////////////////////////
if(safety_switch>1900)
{
ESC1.writeMicroseconds(1000);  
ESC2.writeMicroseconds(1000); 
ESC3.writeMicroseconds(1000); 
ESC4.writeMicroseconds(1000); 
}

 if (safety_switch<1100)

  {  
    ESC1.writeMicroseconds(ESC4); 
    ESC2.writeMicroseconds(ESC2); 
    ESC3.writeMicroseconds(ESC3); 
    ESC4.writeMicroseconds(ESC4);   
    
  }
  
if(roll_SP == ROLL_angle && pitch_SP==Pitch_angle)
  if(yaw_SP==yaw_angle && altitude_SP=altitude)
   if(millis()-time_z>5000)                // wait for 5 second
   {time_z=millis();               // reset
    i++;}
   
} 
      

}
  ////////////////////////////// IMU GYRO READING ////////////////////////////////

void gyro_request(float Gyr_rawX, float Gyr_rawY, float Gyr_rawZ)
 {
  
  Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68) 
  Wire.write(0x43);                        //First adress of the Gyro data
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true);           // Request 6 register from slave address
 Gyr_rawX=Wire.read()<<8|Wire.read();      // Read Gyro Raw Data of X-axis 
 Gyr_rawY=Wire.read()<<8|Wire.read();      // Read Gyro Raw Data of y-axis   
 Gyr_rawZ=Wire.read()<<8|Wire.read();      // Read Gyro Raw Data of z-axis 
 Gyr_rawX = (Gyr_rawX/65.5) - GX_error;    // convert GyroX Raw data into angular rate of X-axis
 Gyr_rawY = (Gyr_rawY/65.5) - GY_error;    // convert GyroY Raw data into angular rate of y-axis 
 Gyr_rawZ = (Gyr_rawY/65.5) - GZ_error;    // convert GyroZ Raw data into angular rate of z-axis 
 /////////////////////////////// Return Gyro data of three axis ///////////////////////////////////////////////
 return Gyr_rawX,Gyr_rawY,Gyr_rawZ;
  }

  ////// IMU Acceleration READING ////////

void acc_request(float Acc_rawX, float Acc_rawX, float Acc_rawX)
{
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);                          //Ask for the 0x3B register- correspond to AcX
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,6,true);                // Request 6 register from slave address
      Acc_rawX=(Wire.read()<<8|Wire.read())/8,192 ; // Read Acceleration of X-axis
      Acc_rawY=(Wire.read()<<8|Wire.read())/8,192 ; // Read Acceleration of Y-axis
      Acc_rawZ=(Wire.read()<<8|Wire.read())/8,192 ; // Read Acceleration of Z-axis
      //////////////////////// Return Acceleration data of three axis  //////////////////////////////
      return  Acc_rawX;   
      return  Acc_rawY;
      return  Acc_rawZ;      
}
/////////// Reading Weather Condition ////////////
void weather_sensor(){
  
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());            // reading temperature
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bme.readPressure() / 100.0F);      // reading pressure
    Serial.println(" hPa");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());               // reading humidity
    Serial.println(" %");
}

///////// interrupt function ///////

ISR(PCINT0_vect){
  //Channel 1=========================================
  if(CH1_state == 0 && PINB & B00000001 ){         //Input 8 changed from 0 to 1
    CH1_state = 1;                                 //Remember current input state
    timer1 = micros();                                 //Set timer_1 to micros()
  }
  else if(CH1_state == 1 && !(PINB & B00000001)){  //Input 8 changed from 1 to 0
    CH1_state = 0;                                 //Remember current input state
    input_ROLL = micros() - timer1;      //Channel 1 is micros() - timer_1
  }
  //Channel 2=========================================
  if(CH2_state == 0 && PINB & B00000010 ){         //Input 9 changed from 0 to 1
    CH2_state = 1;                                 //Remember current input state
    timer2 = micros();                                 //Set timer_2 to micros()
  }
  else if(CH2_state == 1 && !(PINB & B00000010)){  //Input 9 changed from 1 to 0
    CH2_state = 0;                                 //Remember current input state
    input_PITCH = micros() - timer2;      //Channel 2 is micros() - timer_2
  }
  //Channel 3=========================================
  if(CH3_state == 0 && PINB & B00000100 ){         //Input 10 changed from 0 to 1
    CH3_state= 1;                                 //Remember current input state
    timer3 = micros();                                 //Set timer_3 to micros()
  }
  else if(CH3_state == 1 && !(PINB & B00000100)){  //Input 10 changed from 1 to 0
    CH3_state= 0;                                 //Remember current input state
    input_THROTTLE= micros() - timer3;      //Channel 3 is micros() - timer_3
  }
  //Channel 4=========================================
  if(CH4_state == 0 && PINB & B00001000 ){         //Input 11 changed from 0 to 1
    CH4_state = 1;                                 //Remember current input state
    timer4 = micros();                                 //Set timer_4 to micros()
  }
  else if(CH4_state == 1 && !(PINB & B00001000)){  //Input 11 changed from 1 to 0
    CH4_state = 0;                                 //Remember current input state
    input_YAW = micros() - timer4;      //Channel 4 is micros() - timer4
  }
  //Channel 5=========================================
   if(CH5_state == 0 && PINB & B00010000 ){         //Input 12 changed from 0 to 1
    CH5_state = 1;                                 //Remember current input state
    timer5 = micros();                                 //Set timer5 to micros()
  }
  else if(CH5_state == 1 && !(PINB & B00010000)){  //Input 12 changed from 1 to 0
    CH5_state = 0;                                 //Remember current input state
    safety_switch= micros() - timer5;      //Channel 5 is micros() - timer5
  }
  //Channel 6=========================================
   if(CH6_state == 0 && PINB & B00100000 ){         //Input 13 changed from 0 to 1
    CH6_state = 1;                                 //Remember current input state
    timer6 = micros();                                 //Set timer6 to micros()
  }
  else if(CH6_state == 1 && !(PINB & B00100000)){  //Input 13 changed from 1 to 0
    CH6_state = 0;                                 //Remember current input state
    mode_selection= micros() - timer6;      //Channel 6 is micros() - timer6
  }
}
