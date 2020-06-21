//*********************************************************************************
//**
//** Project.........: Magnetic Transmitting Loop Antenna Controller
//**
//**
//** Copyright (C) 2015  Loftur E. Jonasson  (tf3lj [at] arrl [dot] net)
//**
//** This program is free software: you can redistribute it and/or modify
//** it under the terms of the GNU General Public License as published by
//** the Free Software Foundation, either version 3 of the License, or
//** (at your option) any later version.
//**
//** This program is distributed in the hope that it will be useful,
//** but WITHOUT ANY WARRANTY; without even the implied warranty of
//** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//** GNU General Public License for more details.
//**
//** You should have received a copy of the GNU General Public License
//** along with this program.  If not, see <http://www.gnu.org/licenses/>.
//**
//** Platform........: Teensy 3.1 & 3.2 (http://www.pjrc.com)
//**                   (It may be possible to adapt this code to other
//**                    Arduino compatible platforms, however this will 
//**                    require extensive rewriting of some portions of
//**                    the code)
//**
//** Initial version.: 0.00, 2012-10-20  Loftur Jonasson, TF3LJ / VE2LJX
//**                   (pre-alpha version)
//**
//**
//*********************************************************************************

/*
// Do this in ML.h:
// Assign Output Pins to Switches and Stepper

const int EnactSW       = 23;
const int UpSW          = 11;
const int DnSW          = 12;

// A pair of Allegro A4975 Stepper controllers
const int PhA           = 22;
const int D2A           = 21;
const int D1A           = 20;
const int D0A           = 19;
const int PhB           = 18;
const int D2B           = 17;
const int D1B           = 16;
const int D0B           = 15;
// or alternately, a Pololu (TI) DRV8825 or (Allegro) A4988 Stepper motor controller carrier board
const int drv8825_dir   = 14;     // Direction pin
const int drv8825_step  = 15;     // Step pin (positive pulse of +1us for each step)
const int drv8825_ms2   = 16;     // Microstepping pin MS2
const int drv8825_ms1   = 17;     // Microstepping pin MS1
const int drv8825_enable= 18;     // Enable pin
*/

//
//--------------------------------------------------------------------
// Multipurpose Pushbutton 
//
// Returns 0 for no push, or:
// SW1: 1 for short push and 2 for long push
// SW4, SW5 and SW6: 4, 5 and 6 consecutively - if pushed
// (routine should be called once every 1 ms)
//--------------------------------------------------------------------
//
uint8_t multipurpose_pushbutton(void)
{
  static uint16_t pushcount;       // Measure push button time (max 65s)
  uint8_t         state;           // 1, 4, 5 or 6 - or 0 for no-push
  static uint8_t  prevstate;       // Used for clumsy debounce
  uint8_t         retstate = 0;    // 1 for short push, 2 for long push
                                   // 4, 5 or 6 for SW4, 5 or 6 push

  #if PSWR_AUTOTUNE
  uint16_t        push_adval;      // Returned AD value of push button
  #define NOTPUSHED  3800          // AD buttons not pushed if EnactSW AD above this value
  #define SW1         512          // SW1 (EnactSW) Pushed if EnactSW AD below this value
                                   // Inbetween values for SW5 (Autotune Request) pushed
  #define SW4        1816          // SW4: 4096*2k2/(2k2+4k7) = 1304
  #define SW5        2417          // SW5: 4096*4k7/4k7 = 2048
                                   // SW6: 4096*10k/(10k+4k7) = 2786,
                                   // is between SW5 and NOTPUSHED

  push_adval = adc->analogRead(EnactSW);

  // Determine state of pushbutton
  if      ((push_adval < NOTPUSHED) && (push_adval > SW5)) state = 6;
  else if ((push_adval < SW5)       && (push_adval > SW4)) state = 5;
  else if ((push_adval < SW4)       && (push_adval > SW1)) state = 4;
  else if  (push_adval < SW1)                              state = 1;
  #else
  if (digitalRead(EnactSW) == LOW) state = 1;    // Pin low = pushed
  #endif    
  else state = 0;

  //-------------------------------------------------------------------    
  // Is this the end of a successful push?
  // Was state stable for a minimum number of consecutive readings?
  if ((state != prevstate) && (pushcount >= ENACT_MIN))
  {
    if (prevstate == 1)                          // Menu/Enact switch
    { 
      if (pushcount >= ENACT_MAX)                // "Long Push"
      {
        retstate = 2;                            // Indicate long push
      }
      else retstate = 1;
    }
    #if PSWR_AUTOTUNE
    else retstate = prevstate;                   // SW 4, 5 or 6
    #endif

    pushcount = 0;                               // set counter to zero
    prevstate = 0;
  }
    
  //-------------------------------------------------------------------    
  // Debounce and measure time of push
  // If state is not stable, then reset to no-push
  else if (state != prevstate)                   // Change of state, seed prevstate and reset counter
  {
    pushcount = 0;
    prevstate = state;
  }
  else if (state == 0)                           // Push too short, reset everything
  {
    pushcount = 0;
    prevstate = 0;
  }
  else if ((state > 0) && (state == prevstate)) pushcount++; // Count upwards if state is stable

  //-------------------------------------------------------------------    
  // Kludge, enter Config mode immediately when long push condition is satisfied
  if ((state == 1) && (pushcount >= ENACT_MAX)) retstate = 2;

  return retstate;
}


//-------------------------------------------------------------------
// Read status of UP and DOWN Switches and also find if current state
// is a change from the last state
// The poll routines should be called once every one to 5 milliseconds
//-------------------------------------------------------------------

//-------------------------------------------------------------------
// Poll the UP Button, including debounce
// Gather both momentary status and toggle status
uint8_t  up_count = 0;
int8_t   up_button;               // State of UP button
int8_t   up_toggle;               // Set once during a push, reset if status polled

void poll_up_pushbutton(void)
{
  static uint8_t up_count = 0;
  if (digitalRead(UpSW) == LOW)               // Physical Switch is being pushed
  {
    if (!up_button)                           // Debounce
    {
      if (up_count < 2) up_count++;           // A clumsy debounce. Ensure at least 2
                                              // consecutive readings before enacting.
                                              // LOOP_RATE is typically
                                              // set between 1 and 5 milliseconds
      else                                    // YES, switch is being pushed
      {
        up_button = true;                     // Mark Switch as pushed (debounce done)
        up_toggle = true;                     // This is a new state
      }
    }
  }
  else                                        // Switch is not being pushed  
  {
    up_button = false;
    up_count = 0;
  }
}
//-------------------------------------------------------------------
// Return Momentary Status
int8_t up_button_push(void)
{
  return up_button; 
}
//-------------------------------------------------------------------
// Return Toggle (New State) Status and reset Toggle
int8_t up_button_toggle(void)
{
  int8_t toggle_status = up_toggle;
  up_toggle = false;
  return toggle_status;
}

//-------------------------------------------------------------------
// Poll the DN Button, including debounce
// Gather both momentary status and toggle status
uint8_t  dn_count = 0;
int8_t   dn_button;               // State of DOWN button
int8_t   dn_toggle;               // Set once during a push, reset if status polled

void poll_dn_pushbutton(void)
{
  static uint8_t dn_count = 0;
  if (digitalRead(DnSW) == LOW)               // Physical Switch is being pushed
  {
    if (!dn_button)                           // Debounce
    {
      if (dn_count < 2) dn_count++;           // A clumsy debounce. Ensure at least 2
                                              // consecutive readings before enacting.
                                              // LOOP_RATE is typically
                                              // set between 1 and 5 milliseconds
      {
        dn_button = true;                     // Mark Switch as pushed (debounce done)
        dn_toggle = true;                     // This is a new state
      }
    }
  }
  else                                        // Switch is not being pushed  
  {
    dn_button = false;
    dn_count = 0;
  }
}
//-------------------------------------------------------------------
// Return Momentary Status
int8_t dn_button_push(void)
{
  return dn_button; 
}
//-------------------------------------------------------------------
// Return Toggle (New State) Status and reset Toggle
int8_t dn_button_toggle(void)
{
  int8_t toggle_status = dn_toggle;
  dn_toggle = false;
  return toggle_status;
}

#if !DRV8825STEPPER    // ML.h selection: A pair of A4975 Stepper Controllers
//
//---------------------------------------------------------------------------------
// Bipolar Stepper Motor Control Routine,
// using 2x Allegro A4975 Microstepping Full-Bridge Motor Drivers
//---------------------------------------------------------------------------------
//
const uint8_t out_seq[]= { 0xcc,0xbd,0xae,0x9f,  // As per A4975 Datasheet
                           0x0f,0x1f,0x2e,0x3d,  // "Table 4 - Step Sequencing"	
                           0x4c,0x5b,0x6a,0x79,  // Order of bits is:
                           0x70,0x71,0x62,0x53,  // b7, b6, b5, b4, b3, b2, b1, b0
                           0x44,0x35,0x26,0x17,  // PhA,D2A,D1A,D0A,PhB,D2B,D1B,D0B
                           0x07,0x97,0xa6,0xb5,  // where A denotes Bridge A
                           0xc4,0xd3,0xe2,0xf1,  // and   B denotes Bridge B
                           0xf0,0xf9,0xea,0xdb };
                       
int8_t phase = 0;	               // tracks the current step out of a total of 32

void a4975_Write(uint8_t out)
{
  if (out & 0x80) digitalWrite(PhA, HIGH);
  else digitalWrite(PhA, LOW);  
  if (out & 0x40) digitalWrite(D2A, HIGH);
  else digitalWrite(D2A, LOW);  
  if (out & 0x20) digitalWrite(D1A, HIGH);
  else digitalWrite(D1A, LOW);  
  if (out & 0x10) digitalWrite(D0A, HIGH);
  else digitalWrite(D0A, LOW);  
  if (out & 0x08) digitalWrite(PhB, HIGH);
  else digitalWrite(PhB, LOW);  
  if (out & 0x04) digitalWrite(D2B, HIGH);
  else digitalWrite(D2B, LOW);  
  if (out & 0x02) digitalWrite(D1B, HIGH);
  else digitalWrite(D1B, LOW);  
  if (out & 0x01) digitalWrite(D0B, HIGH);
  else digitalWrite(D0B, LOW);  
}

//
// Increment Stepper
//
void a4975_Incr(uint8_t res)
{
  phase = phase + pow(2,res);          // 0 for full resolution of 8 microsteps
                                       // 1 for 4 microsteps, 2 for 2 microsteps
                                       // or 3 for no microsteps
  if(phase >= 32) phase = phase - 32;
  a4975_Write(out_seq[phase]);
}

//
// Decrement Stepper
//
void a4975_Decr(uint8_t res)
{
  phase = phase - pow(2,res);          // 0 for full resolution of 8 microsteps
                                       // 1 for 4 microsteps, 2 for 2 microsteps
                                       // or 3 for no microsteps
  if(phase < 0) phase = phase + 32;
  a4975_Write(out_seq[phase]);
}

void a4975_PwrOff(void)
{
  a4975_Write(0); 
}

//
// Init Stepper Outputs
//
void a4975_Init(void)
{
  pinMode(PhA, OUTPUT);      
  pinMode(D2A, OUTPUT);      
  pinMode(D1A, OUTPUT);      
  pinMode(D0A, OUTPUT);      
  pinMode(PhB, OUTPUT);      
  pinMode(D2B, OUTPUT);      
  pinMode(D1B, OUTPUT);      
  pinMode(D0B, OUTPUT);      

  // Ensure Power Off state
  a4975_Write(0); 
}

#else                // ML.h selection: A Pololu, StepStick or similar
                     // (Allegro A4988 / TI DRV8825) Stepper motor controller carrier board
//
//---------------------------------------------------------------------------------
//		Bipolar Stepper Motor Control Routine, using Allegro A4988 
//		or TI DRV8825 Double Full-Bridge Motor Driver
//---------------------------------------------------------------------------------
//
   
//
// Increment Stepper
//
void drv8825_Incr(uint8_t res)
{
  res = 3 - res;                       // Reversed: 0 for no microsteps
                                       // 1 for half step (2 microsteps)
                                       // 2 for quarter step (4 microsteps)
                                       // 3 for eighth step (8 microsteps)

  drv8825_PwrOn();                     // Ensure Power On state
  digitalWrite(drv8825_dir, LOW);      // Clockwise
  digitalWrite(drv8825_ms1, (res&0x01)?HIGH:LOW);  // ... Microstep resolution
  digitalWrite(drv8825_ms2, (res&0x02)?HIGH:LOW);
  digitalWrite(drv8825_step, HIGH);    // Prime for Movement, turn Step pulse on
}
//
// Decrement Stepper
//
void drv8825_Decr(uint8_t res)
{
  res = 3 - res;                       // Reversed: 0 for no microsteps
                                       // 1 for half step (2 microsteps)
                                       // 2 for quarter step (4 microsteps)
                                       // 3 for eighth step (8 microsteps)

  drv8825_PwrOn();                     // Ensure Power On state
  digitalWrite(drv8825_dir, HIGH);     // Counterclockwise
  digitalWrite(drv8825_ms1, (res&0x01)?HIGH:LOW);  // ... Microstep resolution
  digitalWrite(drv8825_ms2, (res&0x02)?HIGH:LOW);
  digitalWrite(drv8825_step, HIGH);    // Prime for Movement, turn Step pulse on
}

//
// Move Stepper (neends >1+ microsecond delay from positive edge)
//
void drv8825_Move(void)
{
  digitalWrite(drv8825_step, LOW);     // Move, turn Step pulse off  
}

//
// Turn the Stepper On
//
void drv8825_PwrOn(void)
{
  //digitalWrite(drv8825_reset, HIGH); // Release Reset, turn Stepper Motor On
  digitalWrite(drv8825_enable, LOW);   // Enable Stepper
}

//
// Turn the Stepper Off
//
void drv8825_PwrOff(void)
{
  //digitalWrite(drv8825_reset, LOW);  // Reset and turn Stepper Motor Off
  digitalWrite(drv8825_enable, HIGH);  // Disable Stepper, retain last state
}

//
// Init Stepper Outputs
//
void drv8825_Init(void)
{
  pinMode(drv8825_dir, OUTPUT);        // Direction Pin    
  pinMode(drv8825_step, OUTPUT);       // Step Pin
  pinMode(drv8825_ms2, OUTPUT);        // MS2 pin
  pinMode(drv8825_ms1, OUTPUT);        // MS1 pin
  pinMode(drv8825_enable, OUTPUT);     // Enable Pin    
  drv8825_PwrOff();                    // Ensure Power Off state
}
#endif
