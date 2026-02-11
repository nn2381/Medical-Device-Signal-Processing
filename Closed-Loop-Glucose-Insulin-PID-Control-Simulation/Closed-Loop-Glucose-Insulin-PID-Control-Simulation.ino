/**************************************************************************

ARTIFICIAL PANCREAS — GLUCOSE–INSULIN CLOSED-LOOP CONTROL SIMULATION
BMEN 6000 — Signal Processing & Control for Medical Devices

Author: Neema Nkontchou
Institution: Columbia University | Biomedical Engineering

PROJECT OVERVIEW:
This project implements a physiological glucose–insulin regulatory model
on Arduino to simulate metabolic dynamics in diabetic and non-diabetic
patients and evaluate insulin delivery strategies used in modern insulin
pump systems.

The model simulates:
- Intestinal glucose absorption (Q)
- Blood glucose concentration (G)
- Plasma insulin dynamics (I)

Differential equations derived from established physiological models
are numerically integrated in real time to simulate metabolic response
to a 15 g meal disturbance.

CONTROL STRATEGIES IMPLEMENTED:
• Open-loop bolus insulin dosing (pre/post meal)
• Basal insulin infusion
• Combined bolus + basal strategies
• Closed-loop PID glucose control (artificial pancreas logic)

ENGINEERING OBJECTIVE:
To demonstrate how control systems engineering and physiological
modelling are applied in the design of closed-loop medical devices
such as continuous glucose monitors (CGMs) and insulin pumps.

KEY FEATURES:
• Diabetic vs non-diabetic parameter sets
• Numerical integration of nonlinear ODE system
• Real-time glucose response simulation
• PID-controlled insulin delivery
• Hyperglycemia mitigation and stability analysis
• Serial plotting for dynamic visualization

RELEVANCE:
This project models the core control logic underlying modern
artificial pancreas systems and demonstrates integration of
biomedical modelling, embedded systems, and feedback control.

**************************************************************************/


#include <Wire.h>
#include <SD.h>
#include <SPI.h>



// Set to false for NORMAL, true for DIABETIC
bool isDiabetic = true;   

// Initial conditions
float dQ;
float Q = 0.0;  
float dG;
float G = 0.0 ;
float dI;
float I = 0.0;

int i;
int j;

// PID controller gains
float P = 0.08;
float Deriv = 0.2;
float Integer = 0.0005;
float addIn = 0;
float sumG = 0;
float G_setpoint = 155.0;  
float prevError = 0;

// Meal size
float D = 0.0 ; 
int mealclock = 0;
float glucose_perminute = 1000; 

// Bolus controls 
const float BOLUS_UNITS = 15.0f;   
bool bolusGiven = false;           
const float BASAL_RATE = 0.5f;    

// Non-diabetic model
float beta_normal  = 20.0;
float eta_normal   = 4.086;
float gam_normal   = 40.0;
float R0_normal    = 2.1;
float E_normal     = 0.001;
float S_normal     = 0.00306;
float kq_normal    = 0.098;
float Imax_normal  = 0.28;
float alpha_normal = 1.0e4;
float ki_normal    = 0.01;

// Diabetic model
float beta_diab  = 10.0;
float eta_diab   = 4.641;
float gam_diab   = 25.0;
float R0_diab    = 2.5;
float E_diab     = 0.0025;
float S_diab     = 0.00114;
float kq_diab    = 0.026;
float Imax_diab  = 0.93;
float alpha_diab = 1.0e4;
float ki_diab    = 0.06;

// Active parameter set filled in setup from the chosen model
float beta, eta, gam, R0, E, S, kq, Imax, alpha, ki;

const int chipSelect = 4;
File glucoseSystem;

// timekeeping
float t_min = 0.0f;              // simulated time in minutes
const float dt_min = 1.0f;      // 1 minute per loop 

void setup() {
  
  Serial.begin(115200);
  Serial.println("Initializing...");

  while (!Serial) {
    ; 
  }

  if (!isDiabetic) {
  // Copy normal parameters into the active variables
  beta=beta_normal; eta=eta_normal; gam=gam_normal;
  R0=R0_normal; E=E_normal; S=S_normal;
  kq=kq_normal; Imax=Imax_normal; alpha=alpha_normal; ki=ki_normal;

  // Normal parameter guesses for finding initial conditions
  // Q = 0.2f;    
  // G = 80.0f;
  // I = 10.0f;
  // Serial.println("Model: NORMAL (fasting)");

  // Initial conditions determined after running model with guesses 
   Q = 0.000004f;   
   G = 70.855f;     
   I = 9.359f;      
  Serial.println("MODEL: NORMAL (meal 15 g over 15 min)");
  } 

  else {
  // Copy diabetic parameters into active variables
  beta=beta_diab; eta=eta_diab; gam=gam_diab;
  R0=R0_diab; E=E_diab; S=S_diab;
  kq=kq_diab; Imax=Imax_diab; alpha=alpha_diab; ki=ki_diab;

  // Diabetic parameter guesses for finding initial conditions
  // Q = 0.1f;    
  // G = 125.0f;
  // I = 40.0f;
  //Serial.println("Model: DIABETIC (fasting)");

  // Initial conditions determined after running model with guesses
  Q = 0.000005f;   
  G = 162.995f;    
  I = 11.261f;     
  Serial.println("MODEL: DIABETIC (meal 15 g over 15 min)");
  }

  SD.begin(chipSelect);
  if (!SD.begin(chipSelect)) {
    Serial.println("SD not found ");
    return;
  }
  Serial.println("SD found");

  File glucoseSystem = SD.open("gluSys.txt", FILE_WRITE);
  if (glucoseSystem){
    glucoseSystem.println("Q \t G \t I");
    glucoseSystem.close();
  }
  else {
    Serial.println("error opening gluSys.txt");
    return;
  }
  // header for Serial Plotter 
  Serial.println("G\tI\tQ");
}

void loop() {

 // Add meal "input" (3 pts)
  if (mealclock < 15) {
    D = glucose_perminute;
    mealclock++;
  }
  else {
    D = 0.0f;
  }

  // Bolus control 
  // I comment out the sections I am not using 

  // Bolus at the start of the meal
  if (!bolusGiven && mealclock == 1) {
    I += 15;
    bolusGiven = true;
  }
  // Bolus after the meal 
  if (!bolusGiven && mealclock == 15) {
    I += BOLUS_UNITS;
    bolusGiven = true;
  }
  // Bolus before meal + after meal 
  static bool bolusBeforeGiven  = false;
  static bool bolusAfterGiven   = false;

  if (!bolusBeforeGiven && mealclock == 0) {
    I += BOLUS_UNITS;
    bolusBeforeGiven = true;
  }
  if (!bolusAfterGiven && mealclock == 15) {
    I += BOLUS_UNITS;
    bolusAfterGiven = true;
  }

  // Bolus before meal + basal infusion 
  if (!bolusGiven && mealclock == 0) {     
    I += BOLUS_UNITS;
    bolusGiven = true;
  }
  I += BASAL_RATE * dt_min;                 

  // Differential equations for Q, G, I (12 pts)
  dQ = (-beta * Q + eta * D) / (gam * gam + Q * Q);
  dG = R0 - (E + S * I) * G + kq * Q;  
  dI = Imax * ((G * G) / (alpha + G * G)) - ki * I;

  Q += dQ * dt_min;
  G += dG * dt_min;
  I += dI * dt_min;

  // PID control 
  float error = G - G_setpoint;  // error is positive if glucose is above target so we give more insulin
  sumG += error * dt_min;       // integrate error
  float dError = (error - prevError) / dt_min; // slope of error (rate of change per min)
  addIn = P * error + Integer * sumG + Deriv * dError;

  I += addIn;
  if (I < 0) I = 0;
  prevError = error;                   

  t_min += dt_min;

  // Print to Serial
  Serial.print(G, 3);  Serial.print('\t');
  Serial.print(I, 3);  Serial.print('\t');
  Serial.println(Q, 6);


  // Serial.print(Q);
  // Serial.print("\t");
  // Serial.print(G);
  // Serial.print("\t");
  // Serial.print(I);
  // Serial.println("\t");

//  File glucoseSystem = SD.open("gluSys.txt", FILE_WRITE);
//     if (glucoseSystem) {
//     glucoseSystem.print(Q);
//     glucoseSystem.print("\t");
//     glucoseSystem.print(G);
//     glucoseSystem.print("\t");
//     glucoseSystem.println(I);
//     glucoseSystem.close();
//   }
//   else {
//     Serial.println("error opening gluSys.txt");
//     return;
//   }

  delay(100);

}
