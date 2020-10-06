#include <math.h>
#include "Arduino.h"
#include <avr/interrupt.h>
#include <avr/sleep.h>

class GuttedCheapStepper
//https://github.com/tyhenry/CheapStepper
//with all timing functionality removed
{

  public:
    GuttedCheapStepper (int in1, int in2, int in3, int in4) : pins( {
      in1, in2, in3, in4
    }) {
      for (int pin = 0; pin < 4; pin++) {
        pinMode(pins[pin], OUTPUT);
      }
    }

    void setTotalSteps (int numSteps) {
      totalSteps = numSteps;
    }

    void step(bool clockwise) {

      if (clockwise) seqCW();
      else seqCCW();
    }

    void off() {
      for (int p = 0; p < 4; p++)
        digitalWrite(pins[p], 0);
    }

  private:

    int pins[4]; // defaults to pins {8,9,10,11} (in1,in2,in3,in4 on the driver board)

    int stepN = 0; // keeps track of step position
    // 0-4095 (4096 mini-steps / revolution) or maybe 4076...
    int totalSteps = 4096;
    int seqN = -1; // keeps track of sequence number

    void seqCW () {
      seqN++;
      if (seqN > 7) seqN = 0; // roll over to A seq
      seq(seqN);

      stepN++; // track miniSteps
      if (stepN >= totalSteps) {
        stepN -= totalSteps; // keep stepN within 0-(totalSteps-1)
      }
    }

    void seqCCW () {
      seqN--;
      if (seqN < 0) seqN = 7; // roll over to DA seq
      seq(seqN);

      stepN--; // track miniSteps
      if (stepN < 0) {
        stepN += totalSteps; // keep stepN within 0-(totalSteps-1)
      }
    }

    void seq (int seqNum) {

      int pattern[4];
      // A,B,C,D HIGH/LOW pattern to write to driver board

      switch (seqNum) {
        case 0:
          {
            pattern[0] = 1;
            pattern[1] = 0;
            pattern[2] = 0;
            pattern[3] = 0;
            break;
          }
        case 1:
          {
            pattern[0] = 1;
            pattern[1] = 1;
            pattern[2] = 0;
            pattern[3] = 0;
            break;
          }
        case 2:
          {
            pattern[0] = 0;
            pattern[1] = 1;
            pattern[2] = 0;
            pattern[3] = 0;
            break;
          }
        case 3:
          {
            pattern[0] = 0;
            pattern[1] = 1;
            pattern[2] = 1;
            pattern[3] = 0;
            break;
          }
        case 4:
          {
            pattern[0] = 0;
            pattern[1] = 0;
            pattern[2] = 1;
            pattern[3] = 0;
            break;
          }
        case 5:
          {
            pattern[0] = 0;
            pattern[1] = 0;
            pattern[2] = 1;
            pattern[3] = 1;
            break;
          }
        case 6:
          {
            pattern[0] = 0;
            pattern[1] = 0;
            pattern[2] = 0;
            pattern[3] = 1;
            break;
          }
        case 7:
          {
            pattern[0] = 1;
            pattern[1] = 0;
            pattern[2] = 0;
            pattern[3] = 1;
            break;
          }
        default:
          {
            pattern[0] = 0;
            pattern[1] = 0;
            pattern[2] = 0;
            pattern[3] = 0;
            break;
          }
      }

      // write pattern to pins
      for (int p = 0; p < 4; p++) {
        digitalWrite(pins[p], pattern[p]);
      }
    }
};


// Lead Screw at arm_radius of 195.906mm from hinge
// Starting angle of 5.1 degrees due to arm thickness
// Actual radius 196.101mm, because of arm thickness
// Using sidereal day of Ts = 86,164.1 sec for 360 degrees
// Lead Screw rotation period is Tr = Ts*lead/(2*radius*pi)
// Tr = 140 sec per revolution at 0 degrees (for 195.906mm radius)
// Actual 139.861 sec per revolution for 196.101mm at 0 degrees
// 140.056 sec per revolution for 196.101mm at 5.1 degrees starting angle

// (27*23*18)/(9*9*9) = 15.333:1 gear reduction


//Wolfram Alpha calculations:
//Calculating speed factor in relation to angle
//  d(sqrt(1-cos(a)))/da, 0 < a <= pi
//Plot required motor speed reduction factor for angle 
//  plot 1/0.70739*sin(a)/(2sqrt(1-cos(a))), 0 < a <= pi/2
//Get Lead screw rotation period at angle 5.1 degrees (starting angle)
//  x/(1/0.70739*sin(a)/(2sqrt(1-cos(a)))), x = 139.861 , a = 35.1/180*pi 

// (unused)   polynomial regression on selected points 
// (unused)     fit {0.01,0.999601},{0.15,0.996804},{0.3,0.988389},{0.45,0.974418},{0.6,0.954967},{0.75,0.930148},{0.9,0.900099},{1.05,0.86499},{1.2,0.825017},{1.35,0.780405},{1.5,0.731406}

//find derivative of time in regards to screw length (a is angular velocity, x is 2*arm_radius^2
//  d(arccos((b^2 -x)/(x))/a)/d(b)
//calculate equation after substituting, units sec/step (s is stepper steps/mm (4096 steps/revolution, 15.33333 gear reduction ratio), 86,164.1 is seconds in sidereal day, 196.101 is arm_radius in mm)
//  (2 b)/(a sqrt(1 - (b^2 - x)^2/x^2) x),  x=2*(196.101*s)^2, a=2*pi/86164.1, s = 4096*15.33333/2
//result: Gives the seconds required for each stepper step, given b steps occured from angle 0
// (27426.9 b)/sqrt(1.51688×10^14 b^2 - b^4)
//polynomial regression on selected points (~12-127mm), in usec
//  fit quadratic {{400000,2228.08},{800000,2231.61},{1200000,2237.55},{1600000,2245.93},{2000000,2256.86},{2400000,2270.43},{2800000,2286.78},{3200000,2306.1},{3600000,2328.6},{4000000,2354.54}}
// quadratic function result, with error under a microsecond:
//  2228.55 - 3.2547×10^-6 x + 8.64583×10^-12 x^2

double f0 = 2228.5;
//double f0 = 900;
double f1 = -3.2547E-6;
double f2 =  8.64583E-12;

double calc_nominal_delay(double steps) {
  return f0 + steps * f1 + steps * steps * f2;
}

double mm_to_steps(float mm) {
  return mm * 4096 * 15.3333 / 2;
}

GuttedCheapStepper stepper (2, 3, 4, 5);

boolean moveClockwise = false;

unsigned long steps = 0;
unsigned long start_time = 0;

double interstep = 0;

unsigned long  init_steps = mm_to_steps(17.5); //Starting at 17.5mm, 5.1 degrees 
//unsigned long max_steps = 10000; //10000 steps for timing_stats(), to find calculation overhead
unsigned long max_steps = mm_to_steps(100); //100mm is a good limit for 150mm leadscrew, to avoid running out of screw and damaging the camera

void timing_stats(){
  unsigned long end_time = micros();
    
  Serial.begin(9600);
  Serial.print((max_steps - init_steps));
  Serial.print(" step run done, started at ");
  Serial.print(init_steps);
  Serial.print(" steps, ");
  Serial.print(calc_nominal_delay(init_steps));
  Serial.println(" delay.");

  Serial.print("Avg exec usec/step:\t");
  double avg_exec_time_per_step = (end_time - start_time) / (max_steps - init_steps);
  Serial.println(avg_exec_time_per_step);
  Serial.print("Avg overhead usec/step:\t");
  Serial.println(avg_exec_time_per_step - (calc_nominal_delay(init_steps)+calc_nominal_delay(steps))/2);
}

void turn_off(){
  stepper.off();
  delay(2000);
  noInterrupts();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();
}

void setup() {
  steps = init_steps;
  max_steps += init_steps;
  start_time = micros();
}

void loop() {
  if (steps < max_steps) {
    stepper.step(moveClockwise);
    
    steps ++;
    interstep = calc_nominal_delay(steps);
    //calculation overhead trimming, timing_stats() helps find this
    //there is also a small delay fluctuation depending on the steps variable, the second part helps remove that
    //modify this trim also to fix any drift due to crystal frequency inaccuracy, by timing the rotation of the lead screw gear
    interstep -= 123.1 - steps*2E-6; 
    
    delayMicroseconds(interstep);
  }
  else {
    
    timing_stats();

    turn_off();
  }
}
