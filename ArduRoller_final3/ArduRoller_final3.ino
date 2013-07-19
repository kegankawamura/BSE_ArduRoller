// Copyright (c) 2011 Shaun Crampton.

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <stdlib.h>
#include <stdarg.h>				//i.e. void printargs(int arg1, ...) the ellipsis allows for indefinite amount of arguments
#include "Arduino.h"			//debugging
#include "cppboilerplate.h"
#include <avr/interrupt.h>		//For the ISR(TIMER1_COMPA_vect)
#include <util/atomic.h>		//For the ATOMIC_BLOCK in void loop()

#include <CommunicationUtils.h>
#include <DebugUtils.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <FreeSixIMU.h>
#include <Wire.h>;



#define TARGET_LOOP_HZ (1000)
#define TICK_SECONDS (1.0 / TARGET_LOOP_HZ)


#define GYRO_DEG_PER_ADC_UNIT (360.0/255.0)	                		//(Volt/ADC)/[GyroVolt/(Degree*Sec)] =(Degree/ADC)
#define GYRO_RAD_PER_ADC_UNIT (GYRO_DEG_PER_ADC_UNIT * 0.0174532925)			//Convert degrees to radians

// Calculate the scaling factor from ADC readings to g (1g, 2g 1g=9.8m/s/s).  We use the g reading
// from X-axis sensor as an approximation for the tilt angle (since, for small
// angles sin(x) (which the accelerometer measures) is approximately equal to x.
//
// The accelerometer is ratiometric with Vs but it doesn't use the full range
// from GND to Vs.  At 5V, it reads about 1V per g.

#define ACCEL_G_PER_ADC_UNIT (1.0/255.0)					//(Volt/ADC)/(Volt/g) =(g/ADC)

// Pin definitions
const int pwm_a = 3;   // PWM control for motor outputs 1 and 2 is on digital pin 3 (speed)
const int pwm_b = 11;  // PWM control for motor outputs 3 and 4 is on digital pin 11 (speed)
const int dir_a = 12;  // direction control for motor outputs 1 and 2 is on digital pin 12 (clockwise or counter)
const int dir_b = 13;  // direction control for motor outputs 3 and 4 is on digital pin 13  (clockwise or counter)


//#define CALIBRATION

// Allowances for mechanical differences in motors
#define MOTOR_A_FACTOR 1
#define MOTOR_B_FACTOR 1

#define NZEROS 2
#define NPOLES 2

/**
 * Low pass filter. Created via
 * http://www-users.cs.york.ac.uk/~fisher/mkfilter/trad.html
 */
static float filterx(float in)
{
  static float xv[NZEROS+1], yv[NPOLES+1];			//NZEROS=2 NPOLES=2
  xv[0] = xv[1]; xv[1] = xv[2];
  xv[2] = in / 4.143204922e+03;
  yv[0] = yv[1]; yv[1] = yv[2];
  yv[2] =   (xv[0] + xv[2]) + 2 * xv[1]
                       + ( -0.9565436765 * yv[0]) + (  1.9555782403 * yv[1]);
  return yv[2];
}

/**
 *
 */
static float filtergyro(float in)
{
  static float xv[NZEROS+1], yv[NPOLES+1];
  xv[0] = xv[1]; xv[1] = xv[2];
  xv[2] = in / 1.000004443e+00;
  yv[0] = yv[1]; yv[1] = yv[2];
  yv[2] = (xv[0] + xv[2]) - 2 * xv[1]
                          + ( -0.9999911143 * yv[0]) + (  1.9999911142 * yv[1]);
  return yv[2];
}


static int gyro_reading = 0;
static int x_reading = 0;
static int y_reading = 0;

static float last_speed = 0;
static float tilt_rads_estimate = 0;
static float x_filt_gs = 0;
static float tilt_int_rads = 0;
static boolean reset_complete = false;

FreeSixIMU my3IMU = FreeSixIMU();
float values[6];
float angles[3];


void balance()
{
  float y_gs;
  float x_gs;
  float gyro_rads_per_sec;
  float speed = 0;
  long int motor_a_speed = 0;
  long int motor_b_speed = 0;
  static uint16_t counter = 0;
#define NUM_G_SQ 3
  static float total_gs_sq[NUM_G_SQ];
  static int gs_idx = 0;

  // Read the inputs.  Each analog read should take about 0.12 msec.  We can't
  // do too many analog reads per timer tick.

  my3IMU.getValues(values);
  my3IMU.getYawPitchRoll(angles);
  // Gyro rate.
  gyro_reading = values[3];			
  // Accelerometer
  x_reading = values[1];				
  y_reading = values[2];	
  
  Serial.print(y_reading);
  Serial.print(" ");
  Serial.print(x_reading);
  Serial.print(" ");			

  // Convert to sensible units
  //Trimpots allow for manual adjustments of the x_gs and gyro_rads_per_sec without retyping the code (turn the knob)
  
  gyro_rads_per_sec =  GYRO_RAD_PER_ADC_UNIT * (gyro_reading);		//gyro_reading under Gyro Rate. gyro_offset under Convert to sensible units
  x_gs = ACCEL_G_PER_ADC_UNIT * (x_reading);							//x_gs without x_offset (from trimpots)
  y_gs = ACCEL_G_PER_ADC_UNIT * (y_reading);

  x_filt_gs = filterx(x_gs);															//x_gs with filterx			

  total_gs_sq[gs_idx++] = abs(1.0 - (x_gs * x_gs + y_gs * y_gs));
  if (gs_idx == NUM_G_SQ)													//NUM_G_SQ=3. Once gs_idx hits the max number 3, it goes back to 0.
  {
    gs_idx = 0;
  }

  float max_gs_sq = 0;
  Serial.print(y_gs);
  Serial.print(" ");
  Serial.println(x_gs);
  

  if (y_gs < 0.1 && abs(x_gs) > 0.6)		//y_gs max=1. y_gs<0.1 is nearly parallel to the ground
  {
    // We fell over! Shut off the motors.
    speed = 0;
    reset_complete = false;
  }
  else if (!reset_complete)					//true statement if reset_complete=false, which happens in the previous statement and initialization
  {
    // We've never been upright, wait until we're righted by the user.
    if (-0.02 < x_gs && x_gs < 0.02)		//Basically perpendicular to the ground.
    {
      tilt_rads_estimate = x_gs;			//x_gs=(1g)*sin(tilt). At small angles, sin(tilt) is approximately equal to tilt.
      tilt_int_rads = 0;
      digitalWrite(dir_a, x_gs < 0 ? LOW : HIGH);		//If speed is negative, motor direction is LOW	
      digitalWrite(dir_b, x_gs < 0 ? HIGH : LOW);		//If speed is negative, motor direction is LOW
      analogWrite(pwm_a, 1000*x_gs);
      analogWrite(pwm_a, 1000*x_gs);
      reset_complete = true;				//makes this statement false, thus moving onto the next statement
    }
  }
  else											//IMPORTANT
  {
    // The accelerometer isn't trustworthy if it's not reporting about 1G of
    // force (it must be picking up acceleration from the motors).
    for (int i = 0; i < NUM_G_SQ; i++)
    {
      if (total_gs_sq[i] > max_gs_sq)
      {
        max_gs_sq = total_gs_sq[i];						//Sets max_gs_sq equal to the largest total_gs_sq[]
      }
    }
    float g_factor = max(0.0, 1.0 - (15 * max_gs_sq)) * 0.02;			//Keeps the g_factor at least 0.

    tilt_rads_estimate = (1.0 - g_factor) * (angles[2]) +         //changed "tilt_rads_estimate + gyro_rads_per_sec * TICK_SECONDS" to "angles[2]". This is a complementary angle filter.
                         g_factor * x_filt_gs;
    tilt_int_rads += tilt_rads_estimate * TICK_SECONDS;		//TICK_SECONDS=0.001 (1/1000Hz). keeps adding to tilt_int_rads (pseudo integral)

#define D_TILT_FACT 40.0
#define TILT_FACT 12400.0
#define TILT_INT_FACT 4500.0

#define MAX_TILT_INT (0.2)

    if (tilt_int_rads > MAX_TILT_INT)		//Establishes upper and lower bounds
    {
      tilt_int_rads = MAX_TILT_INT;
    }
    if (tilt_int_rads < -MAX_TILT_INT)
    {
      tilt_int_rads = -MAX_TILT_INT;
    }

#ifndef CALIBRATION
    speed = tilt_rads_estimate * TILT_FACT +		//PID controller. Very important
            tilt_int_rads * TILT_INT_FACT +			//Kp=TILT_FACT Ki=TILT_INT_FACT Kd=D_TILT_FACT
            gyro_rads_per_sec * D_TILT_FACT;
           /* Serial.print(angles[2]);
            Serial.print(" ");
            Serial.print(tilt_rads_estimate);
            Serial.print(" ");
            Serial.print(tilt_rads_estimate * TILT_FACT);
            Serial.print(" ");
            /*Serial.print( tilt_int_rads * TILT_INT_FACT);
            Serial.print(" ");
            Serial.print(gyro_rads_per_sec * D_TILT_FACT);
            Serial.print(" ");*/
#endif

  // Set the motor directions
  digitalWrite(dir_a, speed < 0 ? LOW : HIGH);		//If speed is negative, motor direction is LOW	
  digitalWrite(dir_b, speed < 0 ? HIGH : LOW);		//If speed is negative, motor direction is HIGH

  speed = sqrt(abs(speed));
  if (speed > 0xff)
  {
    speed = 0xff;								//ff is the hexadecimal representation for 255 (max speed is 255)
  }
  
  Serial.println(speed);

  motor_a_speed = (long int)(speed * MOTOR_A_FACTOR);	//Stores the motor speed as a long int
  motor_b_speed = (long int)(speed * MOTOR_B_FACTOR);	//Stores the motor speed as a long int

  if (motor_a_speed > 0xff)						
  {
    motor_a_speed = 0xff;						//ff is the hexadecimal representation for 255 (max speed is 255)
  }
  if (motor_b_speed > 0xff)
  {
    motor_b_speed = 0xff;						//ff is the hexadecimal representation for 255 (max speed is 255)
  }

  analogWrite(pwm_a, motor_a_speed);			//voltage is translated into wheel speed
  analogWrite(pwm_b, motor_b_speed);			//voltage is translated into wheel speed
}
}

void setup()
{
  pinMode(pwm_a, OUTPUT);  //Set control pins to be outputs
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);

  analogWrite(pwm_a, 0);
  analogWrite(pwm_b, 0);

  Serial.begin(115200);
  Wire.begin();

  delay(5);
  my3IMU.init();
  delay(5);
}
void loop()
{

  balance();

}
