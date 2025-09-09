// This file contains the code for the real-time PID controller and MPPT tracking algorithm

#include <PWM.h>
#include <PID_v1.h>
#include <Oversample.h>

float current1 = 0.0;

int mppt_on_off = 0;

byte analogPin = A0;
byte resolution = 16;
byte interval = 1;

Oversample * sampler;

int value = 0;
float v_buck;
float R1 = 22400.0;
float R2 = 18000.0;

float v_boost;
float R3 = 10000.0;
float R4 = 3300.0;

float v_solar;
float R5 = 22400.0;
float R6 = 18000.0;

int duty_cycle = 0;

int pin = 11;                // the pin that the LED is attached to
int32_t frequency = 5000; //frequency (in Hz)

// PID controller for the buck converter
// Input: Input_buck (voltage value from the panel)
// Setpoint_buck: Desired setpoint voltage we wish to get to
// Output_buck: Output variable (goes back to the circuit)
// Kp_buck, Ki_buck, Kd_buck: Calculated PID values for the controller
double Setpoint_buck, Input_buck, Output_buck;
double Kp_buck = 0.2, Ki_buck = 40, Kd_buck = 0;
PID myPID_buck(&Input_buck, &Output_buck, &Setpoint_buck, Kp_buck, Ki_buck, Kd_buck, DIRECT);

// PID controller for the boost converter
// Input: Input_boost (voltage value from the panel)
// Setpoint_boost: Desired setpoint voltage we wish to get to
// Output_boost: Output variable (goes back to the circuit)
// Kp_boost, Ki_boost, Kd_boost: Calculated PID values for the controller
double Setpoint_boost, Input_boost, Output_boost;
double Kp_boost = 0.6, Ki_boost = 40, Kd_boost = 0;
PID myPID_boost(&Input_boost, &Output_boost, &Setpoint_boost, Kp_boost, Ki_boost, Kd_boost, DIRECT);

// Parameters for the MPPT tracking Algorithm
  float current_old=0.0;
  float voltage_old=0.0;
  float power_old=0.0;
  float D_old=0.0;  //initial value 
  float D=0.0;      // initial value
  float Delta=10;  // initial value
  float PWM_value= 255*D;

  int value1 = 0;
float voltage;

void setup()
{
  Serial.begin(9600);
  //initialize all timers except for 0, to save time keeping functions
  InitTimersSafe(); 

  //sets the frequency for the specified pin
  bool success = SetPinFrequencySafe(pin, frequency);
  
  //if the pin frequency was set successfully, turn pin 13 on
  if(success) {
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);    
  }

  Setpoint_buck = 5.0;
  Setpoint_boost = 15.0;
  myPID_buck.SetMode(AUTOMATIC);
  myPID_boost.SetMode(AUTOMATIC);

  sampler = new Oversample(analogPin, resolution);

  byte resolution = sampler->getResolution();
  //sampler->setResolution(16);
  Serial.print("Resolution: ");
  Serial.println(resolution);

  byte prescaler = sampler->getPrescaler();
  //sampler->setPrescaler(7);
  Serial.print("Prescaler: ");
  Serial.println(prescaler); 
}

void loop()
{

  value1 = analogRead(A7);
  int mppt_on_off = value1 * 5.0/1023;
  Serial.print("mppt__= ");
  Serial.println(mppt_on_off);

  // Read the relevant current and voltage values from the circuit first
  value1 = analogRead(A3);
  voltage = value1 * 5.0/1023;
  current1 = -2.165*voltage +36.94;
  Serial.print("Current= ");
  Serial.println(voltage);

  value = analogRead(A2);
  v_solar = value * (5.0/1024)*((R3 + R4)/R4);
  Serial.print("v_solar =");
  Serial.println(v_solar);

  value = analogRead(A1);
  v_boost = value * (5.0/1024)*((R3 + R4)/R4);
  Serial.print("v_boost =");
  Serial.println(v_boost);
  
  value = analogRead(A0);
  v_buck = value * (5.0/1024)*((R1 + R2)/R2);
  Serial.print("v_buck =");
  Serial.println(v_buck);


  if (mppt_on_off < 3) {
    Input_boost = v_boost;
    myPID_boost.Compute();
    duty_cycle = Output_boost;
    duty_cycle = map(duty_cycle, 0, 255, 0 , 230);
    //duty_cycle = 255 - duty_cycle;
    pwmWrite(11, duty_cycle); 
  }

  Input_buck = v_buck;
  myPID_buck.Compute();
  duty_cycle = Output_buck;
  duty_cycle = map(duty_cycle, 0, 255, 0 , 230);
  duty_cycle = 255 - duty_cycle;
  pwmWrite(12, duty_cycle); 

  if (mppt_on_off >= 3) {
    Serial.print("Voltage and Current");
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    float voltage_new = v_boost;
    float current_new = current1;
    // print out the value you read:
    Serial.println(voltage_new);
    Serial.println(current_new); 


    float power_new=voltage_new*current_new;
    Serial.print("Current Power: ");
    Serial.println(power_new);
      Serial.print("Starting PWM: ");
    Serial.println(PWM_value);
    // Apply MPPT 
    if((power_new-power_old)!=0.0)
    {
        Serial.println("MPPT if (1) statment is now active");

      if ((power_new-power_old)>0)
      {
            Serial.println("MPPT if (2) statment is now active");

        if ((voltage_new- voltage_old)>0)
        {
          D = D_old - Delta;
        Serial.println("MPPT if (3) statment is now active");
        }
        else
        {
        D = D_old + Delta; 
          Serial.println("MPPT else (2) statment is now active");

        }
      }
    else
      {
        if ((voltage_new- voltage_old)>0)
        {
          D = D_old + Delta;
          
        Serial.println("MPPT if (4) statment is now active");

        }
        else
        {
        D = D_old - Delta; 
          Serial.println("MPPT else (3) statment is now active");

        }
      }
    }
    else
    {
      Serial.println("MPPT else (1) statment is now active");
      D_old=D;
      }


      // Apply PWM signal
    duty_cycle =  255-  D;
    pwmWrite(11,duty_cycle);
    /// prepare for the next loop
    voltage_old=voltage_new;
      current_old=current_new;
      power_old=power_new;
      D_old=D;
    /// Display the current power
    Serial.println("power is");
    Serial.println(power_old);
    delay(500);

  }
 delay(500);
}

