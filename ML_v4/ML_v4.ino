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

//------------------------------------------------------------------------
// For selection of features such as:
//   (to name a few...)
// - Stepper Motor Controller: 2x A4975 or alternately a DRV8825 or A4988
// - Endstop sensor functionality;
// - Power and SWR meter sensitivity and behaviour;
// - SWR assisted auto-tune; 
// - Default Radio settings;
// - Switchover points if controlling two or three antennas;
// - Rotary Encoder sensitivity (resolution); and
// - Microcontroller pin assignments:
//
//   See ML.h
//

// Other source code files used by this project are:
// ML_Display.ino
// ML_Menu.ino
// ML_PSWR.ino
// ML_Pos_Mgmnt.ino
// ML_SWRtune.ino
// ML_TRX_Frq.ino
// ML_USB.ino
// ML_Switches_and_Stepper.ino
// and
// _EEPROMAnything.h
//

#include <LiquidCrystalFast.h>
#include <Metro.h>
#include <EEPROM.h>
#include <Encoder.h>
#include <ADC.h>                     // Syncrhonous read of the two builtin ADC
#include "_EEPROMAnything.h"
#include "ML.h"

#if WIRE_ENABLED
#include <i2c_t3.h>
#endif

//-----------------------------------------------------------------------------------------
// Global variables
rflags    radio;                     // Radio Operation related Boolean Flags/Signals  (struct defined in ML.h)
flags     flag;                      // All kinds of Boolean Flags/Signals (struct defined in ML.h)

settings  controller_settings;       // Various controller settings to be stored in EEPROM
                                     // (struct defined in ML.h)
                                    
var_track running[3];                // Running Frequency and Position (struct defined in ML.h).  Separate running for two
                                     // (or experimentally three) antennas if this option is used. If one antenna, then
                                     // only one is used.
uint8_t   ant;                       // Which antenna is in use (0, 1 or 2 if three antennas). Used with running and preset
                                     // to define which memory bank to use.
int32_t   ant1_changeover;           // Antenna changeover frequency.  Set by ANT1_CHANGEOVER define if auto changeover
                                     // Set as 0 or as 100000000 by a Switch connected to ChgOvSW (pad 27 underneath Teensy),
                                     // if manual.
int32_t   ant2_changeover;           // Second Antenna changeover frequency.  Can be used if ANT_CHG_2BANKS is 0

var_track preset[MAX_PRESETS];       // Frequency and Position presets - only half the number of presets available in this mode
var_track min_preset[3];             // Contains values from the lowest stored presets, up to three antennas
var_track max_preset[3];             // Contains values from the highest stored presets, up to three antennas
uint8_t   num_presets[3];            // Number of active (stored) presets per antenna
uint8_t   range;                     // Active range (which presets are we in-between?) - Consecutive, ant1 + ant2 + ant3

int32_t   stepper_track[3];          // Current position of Stepper
int32_t   delta_Pos[3];              // delta or difference between position based on presets and
                                     // the actual current position as a result of tuning Encoder
                                     // or Up/Down buttons
int32_t   tmp_delta_Pos;             // Used for temporary storage of delta Position during an
                                     // SWR tune operation

int32_t   tunedFrq;                  // Frequency calculated from the current position of Stepper

int8_t    dir_of_travel = 1;         // Used to determine range in case of inverse motion
                                     // and to facilitate a test whether within a proven range
                                     // and to normalise endstop behaviour based on direction of travel

// The below are used by the ML_Menu functions, and match the similar variables in controller_settings
int8_t    step_rate;                 // Stepper rate, 1 - 15. Actual rate is 100 x step_rate per second
int8_t    step_speedup;              // Stepper Motor variable rate speedup, 0 - 3, equals a speedup of
                                     // 1, 2, 4 or 8 ( pow(2, step_speedup) )
int8_t    microstep_resolution;      // Stepper Motor microstep resolution, 0 - 3, where:
                                     //        0 for full resolution of 8 microsteps
                                     //        1 for 4 microsteps (position tracking increases in quadruple steps)
                                     //        2 for 2 microsteps  (position tracking increases in double steps)
                                     //        or 3 for no microsteps (position tracking increases 8x per step)
int16_t   radio_selection;           // Radio selection, where:
                                     //        0 = ICOM CI-V Auto
                                     //        1 = ICOM CI-V Poll
                                     //        2  = Kenwood TS-440
                                     //        3 = Kenwood TS-480/TS-590/TS-2000
                                     //        4 = Yaesu FT-100
                                     //        5  = Yaesu FT-747 (757,767???)
                                     //        6 = Yaesu FT-8x7 (817,847,857,897)
                                     //        7  = Yaesu FT-990
                                     //        8  = Yaesu FT-1000MP
                                     //        9  = Yaesu FT-1000MP Mk-V
                                     //       10 = Yaesu FT-450/950/1200/2000/3000/5000
                                     //       11 = Elecraft K3/KX3 Auto
                                     //       12 = Elecraft K3/KX3 Poll
                                     //       13 = TenTec binary mode
                                     //       14 = TenTec ascii mode
                                     //       15 = Pseudo VFO mode
                                     
uint8_t   rs232_signals;             // 0 for Normal RS232 signalling, 1 for inverted signalling
                                     // in addition, 2nd bit high for passhthrough between serial and USB.
uint8_t   rs232_rate;                // RS232 Data Rate, (0=1200, 1=2400, 2=4800 ...7=115200)

uint8_t   tx_tune_pwr;               // TX Tune Power level

int16_t   Menu_exit_timer;           // Used for a timed display when returning from Menu
int16_t   encOutput;                 // Output From Encoder

char      print_buf[82];             // Print format buffer for lcd/serial/usb output

// Power and SWR meter globals
mflags    meter;                     // Boolean signals/flags from Power SWR meter (struct defined in ML.h)
uint16_t  fwd;                       // AD input - 12 bit value, v-forward
uint16_t  ref;                       // AD input - 12 bit value, v-reverse
uint8_t   Reverse;                   // BOOL: True if reverse power is greater than forward power
#if PSWR_AUTOTUNE
#if AD8307_INSTALLED
double    fwd_power_mw;              // Calculated forward power in mW
double    ref_power_mw;              // Calculated reflected power in mW
double    power_mw;                  // Calculated power in mW
double    power_mw_pep;              // Calculated PEP power in mW
double    power_mw_pk;               // Calculated 100ms peak power in mW
#else
int32_t   fwd_power_mw;              // Calculated forward power in mW
int32_t   ref_power_mw;              // Calculated reflected power in mW
int32_t   power_mw;                  // Calculated power in mW
int32_t   power_mw_pep;              // Calculated PEP power in mW
int32_t   power_mw_pk;               // Calculated 100ms peak power in mW
#endif
#endif
double    measured_swr=1.0;          // SWR as an absolute value
uint16_t  swr_bar;                   // logarithmic SWR value for bargraph, 1000 equals SWR of 10:1
double    snapshot_swr;              // Best SWR from a successful Auto SWR operation
uint16_t  snapshot_swr_bar;

// SWR tune globals
swrflags  swr;                       // Status and Progress signals/flags for the SWR Tune (struct defined in ML.h)
int16_t   SWRtune_timer;             // Used to monitor whether max time has been reached during SWR tune
int16_t   SWRautoretune_timer;       // Used to prevent an automatic retune before stepper motor has
                                     // settled from last time
#if WIRE_ENABLED
int8_t    ad7991_addr = 0;           // Address of AD7991 I2C connected A/D, 0 if none detected
#endif

// Debug
int32_t   fakepswr_val=1200;         // Used with #define FAKEPSWR - a debug tool to fake Power/SWR meter for
                                     // SWR tune without needing to apply RF from a Transceiver

//-----------------------------------------------------------------------------------------
// Instanciate an ADC Object
ADC *adc = new ADC();

//-----------------------------------------------------------------------------------------
// Instanciate an Encoder Object
Encoder   Enc(EncI, EncQ);

//-----------------------------------------------------------------------------------------
// Timers for various tasks:
elapsedMicros loopTime;              // Stepper Scan Loop timer, in microseconds.  Can take values
                                     // from 666 microseconds (1500 steps per second) up to 10000
                                     // microseconds (100 steps per second).
                                     // This timer is also used to pace the lcd print from the virtual LCD
                                     // to the real LCD, one character per 0.333 milliseconds.

Metro     pushMetro = Metro(1);      // 1 millisecond timer for Pushbutton management

#if PSWR_AUTOTUNE
Metro     pswrMetro = Metro(5);      // 5 millisecond timer for Power and SWR measurements
#endif

Metro     slowMetro = Metro(100);    // 100 millisecond timer for various tasks

Metro     lcd_Metro = Metro(100);    // 100 millisecond timer for LCD print

Metro     trxpollMetro = Metro(1000);// Poll Transceiver for frequency data at a defined rate
                                     // (typical POLL_RATE is 1000 = 1 second. Tbis value is defined
                                     //  separately per transceiver type, see #defines XXXX_POLL_RATE
                                     //  as defined for each transceiver in ML.h)
                                     // Note: The Auto Modes do not poll.

//-----------------------------------------------------------------------------------------
// initialize the LCD
LiquidCrystalFast lcd(LCD_RS, LCD_RW, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

//-----------------------------------------------------------------------------------------
// Define a "Uart" object to access the serial port
//HardwareSerial Uart = HardwareSerial();
#define Uart Serial1

//
//-----------------------------------------------------------------------------------------
// Track frequency and position
//-----------------------------------------------------------------------------------------
//
void track_frq(void)
{  
  #if ENDSTOP_OPT == 1          // Vacuum Capacitor, no end stops
  //
  // Protect capacitor from damage in case of misbehavior by software by
  // blocking automatic tuning if we are outside of stored presets
  //
  // Indicate if frequency is outside of the range of configured presets
  
  #if MOVEMENT_OUTSIDE_FRQ      // If this option selected, inhibit all movement if Frequency outside range
  // Are we withing preset frequency range?
  if ((running[ant].Frq < min_preset[ant].Frq) || (running[ant].Frq > max_preset[ant].Frq))
  {
    flag.frq_xrange = true;     // Frequency is out of range
  }
  else flag.frq_xrange = false; // Frequency is within range
  #endif
  
  // Indicate if capacitor position is outside of preset range 
  if (dir_of_travel == 1)       // Capacitor tunes in an ascending manner, Encoder clockwise = increase freqency
  {
    // Indicate if beyond lowermost/uppermost stored frq/pos
    if (stepper_track[ant] < min_preset[ant].Pos) flag.cap_lower_pos = true;
    else flag.cap_lower_pos = false;
    if (stepper_track[ant] > max_preset[ant].Pos) flag.cap_upper_pos = true;
    else flag.cap_upper_pos = false;

    // Stop any automatic movement beyond this point
    if (stepper_track[ant] < (min_preset[ant].Pos - ENDSTOP_TOLERANCE)) flag.cap_lower_endstop = true;
    else flag.cap_lower_endstop = false;
    if (stepper_track[ant] > (max_preset[ant].Pos + ENDSTOP_TOLERANCE)) flag.cap_upper_endstop = true;
    else flag.cap_upper_endstop = false;
  }
  if (dir_of_travel == -1)      // Capacitor tunes in an descending manner, Encoder counterclockwise = decrease frequency
  {
    // Indicate if beyond lowermost/uppermost stored frq/pos
    if (stepper_track[ant] > min_preset[ant].Pos) flag.cap_upper_pos = true;
    else flag.cap_upper_pos = false;
    if (stepper_track[ant] < max_preset[ant].Pos) flag.cap_lower_pos = true;
    else flag.cap_lower_pos = false;

    // Stop any automatic movement beyond this point
    if (stepper_track[ant] > (min_preset[ant].Pos + ENDSTOP_TOLERANCE)) flag.cap_upper_endstop = true;
    else flag.cap_upper_endstop = false;
    if (stepper_track[ant] < (max_preset[ant].Pos - ENDSTOP_TOLERANCE)) flag.cap_lower_endstop = true;
    else flag.cap_lower_endstop = false;
  }

  //
  // Inside presets, It is OK to do stuff
  //
  if ((num_presets[ant]>=2) && !flag.frq_xrange && !flag.cap_lower_endstop && !flag.cap_upper_endstop)
  //if ((num_presets[ant]>=2) && !flag.frq_xrange && !flag.cap_lower_pos && !flag.cap_upper_pos)
  {
    //
    // Calculate Stepper position to tune to, based on indicated frequency
    //
    determine_active_range(running[ant].Frq);   // Find stored preset immediately above current frequency
                                                // Returns 0 if frequency is lower than lowest or
                                                // higher than highest reference frequency
    running[ant].Pos = derive_pos_from_frq();   // Calculate Stepper Pos based on Frequency
  
    // Make sure we don't move further out if at outermost stored presets, while allowing movements back in

    if (dir_of_travel == 1)       // Capacitor tunes in an ascending manner, Encoder clockwise = increase freqency
    {
      // Make sure we do not go further than to Endstop preset
      // Are we moving down beyond min?
      if (((running[ant].Pos + delta_Pos[ant]) < stepper_track[ant]) && (stepper_track[ant] <= (min_preset[ant].Pos-ENDSTOP_TOLERANCE)))
          running[ant].Pos = min_preset[ant].Pos - delta_Pos[ant];
      // Are we moving up beyond max?
      if (((running[ant].Pos + delta_Pos[ant]) > stepper_track[ant]) && (stepper_track[ant] >= (max_preset[ant].Pos+ENDSTOP_TOLERANCE))) 
          running[ant].Pos = max_preset[ant].Pos - delta_Pos[ant];
    }
    if (dir_of_travel == -1)      // Capacitor tunes in an ascending manner, Encoder counterclockwise = increase freqency
    {   
      // Make sure we do not go further than to Endstop preset
      // Are we moving down beyond min?
      if (((running[ant].Pos + delta_Pos[ant]) > stepper_track[ant]) && (stepper_track[ant] >= (min_preset[ant].Pos+ENDSTOP_TOLERANCE))) 
          running[ant].Pos = min_preset[ant].Pos - delta_Pos[ant];
      // Are we moving up beyond max?
      if (((running[ant].Pos + delta_Pos[ant]) < stepper_track[ant]) && (stepper_track[ant] <= (max_preset[ant].Pos-ENDSTOP_TOLERANCE))) 
          running[ant].Pos = max_preset[ant].Pos - delta_Pos[ant];
    }
  }
  
  tunedFrq = derive_frq_from_pos();        // Calculate Tuned Frequency based on Stepper Pos 
  
  #else  // No EndStop smart logic needs to be applied
  //
  // Calculate Stepper position to tune to, based on indicated frequency
  // (this can only be done if we already have two active presets)
  if (num_presets[ant] >= 2)
  {
    determine_active_range(running[ant].Frq);   // Find stored preset immediately above current frequency
                                                // Returns 0 if less than two presets are available.
                                                // Otherwise returns closest matching range
    //
    // Calculate stepper position to tune to if transceiver frequency
    // is within the range of stored positions
    //
    if (range > 0)
    {
      running[ant].Pos = derive_pos_from_frq(); // Calculate Stepper Pos based on Frequency
    }
    
    tunedFrq = derive_frq_from_pos();      // Calculate Tuned Frequency based on Stepper Pos
  }
  #endif
}


//
//---------------------------------------------------------------------------------
// Use this in lieu of delay(), to ensure USB and UART input is not lost
//---------------------------------------------------------------------------------
//
void delayloop(uint16_t msec)
{
  unsigned long starttime, stoptime;

  starttime = millis();
  stoptime  = starttime + msec;

  while (stoptime > millis())
  {
    usb_read_and_parse();                   // Read and parse anything on the USB serial port
    trx_read_and_parse();                   // Read and parse anything from the Transceiver (TTL or RS232)
  }
}


//
//---------------------------------------------------------------------------------
// Here there be all the heavy lifting
//---------------------------------------------------------------------------------
//
void loop()
{
  static uint16_t frq_store_timer;          // Keep track of when last update, in order to store
                                            // current status if stable
                                            
  static uint16_t stepper_active_timer;     // Keep track of whether stepper motor is active (200ms timer)
  
  static uint8_t  radio_reset_timer;        // Keep track of whether Radio is on-line or not,
                                            // 10 second frequency receive poll time

  static uint8_t  multi_button;             // State of Multipurpose Enact Switch
                                            // (Multipurpose pushbutton)
  static uint8_t  old_multi_button;         // Last state of Multipurpose Enact Switch
  static int8_t   button_rate_reducer;      // Stepper scan rate divisor when Up/Down pushbutton
  
  uint8_t         stepper_loop_rate_divide; // Reduce speed of Stepper by only moving when
                                            // a counter equals this value
  static int8_t   skip_step;                // Used in conjunction with stepper_loop_rate_divide   
  
  uint8_t         microstep_skip;           // Stepper rate increase as a function of reduced
                                            // number of microsteps. 0 = 8 microsteps
                                            // 3 = 0 microsteps
 
  static uint8_t  dn_button_last_down;      // TRUE for backlash, Down button has been released
  static uint8_t  dn_button_backlash_return;// TRUE if recovering from button generated backlash
                                            // These two used to activate backlash upon dn release
  // Bandswitching changeover points controlling two output bits, independent for up to three antennas
  const int32_t bnd1_changeover[3] = {ANT1_BND1_CHANGEOVER, ANT2_BND1_CHANGEOVER, ANT3_BND1_CHANGEOVER};
  const int32_t bnd2_changeover[3] = {ANT1_BND2_CHANGEOVER, ANT2_BND2_CHANGEOVER, ANT3_BND2_CHANGEOVER};
  const int32_t bnd3_changeover[3] = {ANT1_BND3_CHANGEOVER, ANT2_BND3_CHANGEOVER, ANT3_BND3_CHANGEOVER};
    
  #if PSWR_AUTOTUNE
  static uint16_t power_reset_timer;        // Timer to revert back to stepper display if no power
  #endif                                    // (units of 5 ms)
                                            
  //-------------------------------------------------------------------------------
  // Here we do routines which are to be run through as often as possible
  //-------------------------------------------------------------------------------
  
  //-------------------------------------------------------------------
  // Asynchronous management of USB and Serial ports
  //
  usb_read_and_parse();                     // Read and parse anything on the USB serial port
 
  trx_read_and_parse();                     // Read and parse anything from the Transceiver (TTL or RS232)

  //-------------------------------------------------------------------------------
  // Here we do routines in conjunction with Stepper Position Updates, which are done
  // once per 667 microseconds (for max rate of 1500 steps per second) up to
  // uonce per 10 milliseconds (for a slow rate of only 100 steps per second).
  // This timer is also used to pace the lcd print from the virtual LCD,
  // see func virt_LCD_to_real_LCD() in ML_Display                              
  //-------------------------------------------------------------------------------
  // Stepper Scan Loop timer.
  uint16_t stepperloop_time = 10000/step_rate;// Step rate of 15 - 1 (eq 667us - 10ms)
  if (loopTime >= (stepperloop_time))         // Microsecond resolution, variable timer.
  {
    loopTime = loopTime - (stepperloop_time); // Reset timer for next go-around

    //-------------------------------------------------------------------
    // Poll the status of UP and DOWN Switches, including whether there is
    // change from the last state.  Actual states determined with return
    // value from XX_button_push() and XX_button_toggle()
    //-------------------------------------------------------------------
    poll_up_pushbutton();
    poll_dn_pushbutton();
        
    //-------------------------------------------------------------------
    // Config Menu Mode, use Up/Down switches in lieu of Encoder, to step between items
    if (flag.config_menu)
    {
      if (up_button_toggle() )             // UP switch has been pushed
      {
        Enc.write(-ENC_MENURESDIVIDE);     // Decrement Encoder by one, in line with direction of Menu scroll
      }
      if (dn_button_toggle() )             // Down switch has been pushed
      {
        Enc.write(+ENC_MENURESDIVIDE);     // Increment Encoder by one, in line with direction of Menu scroll
      }
    }

    //-------------------------------------------------------------------
    // Pseudo-VFO mode
    //
    //-------------------------------------------------------------------
    // We are in Pseudo VFO Radio mode and Encoder is selected for VFO function
    //
    if (!flag.config_menu && (controller_settings.trx[controller_settings.radioprofile].radio == MAX_RADIO) && controller_settings.pseudo_vfo)
    {
      pseudo_vfo_up_down();                // Pseudo-VFO active, use Up/Down switches to step through frequency bands
                                           // and in 100 kHz increments within bands
      update_pseudo_vfo();                 // Read Encoder as VFO
    }
    // We are no longer in Pseudo VFO Radio mode, need to return Encoder to Stepper function
    if ((controller_settings.trx[controller_settings.radioprofile].radio != MAX_RADIO) && controller_settings.pseudo_vfo)
    {
      pseudovfo_encoder_toggle();          // Return Encoder to Stepper
    }

    //-------------------------------------------------------------------
    // Track active frequency input from Radio
    //
    // To get rid of accumulated tracking offset while Radio information is
    // off line, we zero delta_Pos when the radio gets back on_line
    if(radio.online && (radio.previous != radio.online)) delta_Pos[ant] = 0;
    radio.previous = radio.online;
      
    track_frq();                           // Calculate new position based on
                                           // frequency input.  With End_Stop option 1
                                           // this will not return a new position if
                                           // outside of bounds.

    //-------------------------------------------------------------------
    #if ENDSTOP_OPT == 2                   // End stop sensors implemented
    // Check End Stop Sensors   
    if (digitalRead(EndStopUpper) == LOW)
      flag.endstop_upper = true;           // Set Flag indicating Upper Endstop limit
    else flag.endstop_upper = false;       // Clear Flag indicating Upper Endstop limit
    if (digitalRead(EndStopLower) == LOW)
      flag.endstop_lower = true;           // Set Flag indicating Lower Endstop limit
    else flag.endstop_lower = false;       // Clear Flag indicating Lower Endstop limit
    #endif

    //-------------------------------------------------------------------
    // Derive the Active Stepper Settings - used for instance to determine
    // variable rate based on distance of movement.
    //Returned value contains two variables, microstep_skip in the lower 4 bits
    // and stepper_loop_rate divisor in the upper 4 bits
    microstep_skip = determine_variable_rate();    
    stepper_loop_rate_divide = pow(2,microstep_skip>>4);
    microstep_skip = microstep_skip & 0x07;

    //-------------------------------------------------------------------
    // Work with Stepper Positioning if the stepper_loop_rate divisor has matured
    // 
    if (skip_step >= stepper_loop_rate_divide)
    {
      skip_step = 0;

      //-------------------------------------------------------------------
      // Manually Move Stepper back and forth unless in Config Mode
      // or in Pseudo-VFO mode with the Pseudo-VFO active
      if (!flag.config_menu
         && ((controller_settings.trx[controller_settings.radioprofile].radio != MAX_RADIO)
         || !controller_settings.pseudo_vfo))
      {						
        //-------------------------------------------------------------------
        // Manually Move Stepper back and forth using Encoder
        // This mode has no restrictions of movement when configured for 
        // END_STOP options 1 and 3
        //						
        encOutput = Enc.read();
        #if ENDSTOP_OPT == 2                 // End stop sensors implemented
        if (!flag.endstop_upper && (encOutput/ENC_TUNERESDIVIDE > 0))
        #else
        if (encOutput/ENC_TUNERESDIVIDE > 0)
        #endif
        {						
          delta_Pos[ant]++;                  // Update position
          Enc.write(encOutput - ENC_TUNERESDIVIDE);
          flag.manual_move = true;           // Disable backlash when using Encoder
        }
        #if ENDSTOP_OPT == 2                 // End stop sensors implemented
        else if (!flag.endstop_lower && (encOutput/ENC_TUNERESDIVIDE < 0))
        #else
        else if (encOutput/ENC_TUNERESDIVIDE < 0)
        #endif
        {	
          delta_Pos[ant]--;                  // Update position
          Enc.write(encOutput + ENC_TUNERESDIVIDE);
          flag.manual_move = true;           // Disable backlash when using encoder
        }

        //-------------------------------------------------------------------
        // Move stepper by using UP or DOWN push buttons
        //
        // This function only active if we are receiving data from Radio (not Pseudo-VFO)
        // and if rate reducer has matured
        if (radio.online && (button_rate_reducer == 0) && !swr.tune 
           && ((controller_settings.trx[controller_settings.radioprofile].radio != MAX_RADIO)
           || !controller_settings.pseudo_vfo))
        {          
          //
          // UP Button
          //
          #if ENDSTOP_OPT == 1                 // Vacuum Capacitor, no end stops
          // Movement is only possible if we are inside of known FRQ and CAP range
          //
          // UP switch has been pressed and We're at or above min but below max      
          if (up_button_push() && 
              ((dir_of_travel*stepper_track[ant]) >= (dir_of_travel*min_preset[ant].Pos)) && 
              ((dir_of_travel*stepper_track[ant]) < (dir_of_travel*max_preset[ant].Pos)))
          #elif ENDSTOP_OPT == 2               // End stop sensors implemented
          // End_Stop switches can inhibit movement
          //
          // UP switch has been pushed and we're not at upper limit of range
          if (up_button_push() && !flag.endstop_upper)          
          #elif ENDSTOP_OPT == 3               // Butterfly capacitor, no end stops required
          // No movement inhibits whatsoever
          //
          if (up_button_push() )               // UP switch has been pushed          
          #endif
          {
            delta_Pos[ant]++;                  // Update position
          }
          //
          // DOWN Button
          //
          #if ENDSTOP_OPT == 1                 // Vacuum Capacitor, no end stops
          // Movement is only possible if we are inside of known FRQ and CAP range
          //
          // Down switch has been pressed and We're at or below max but above min      
           else if ((dn_button_push() )
                   && ((dir_of_travel*stepper_track[ant]) > (dir_of_travel*min_preset[ant].Pos))
                   && ((dir_of_travel*stepper_track[ant]) <= (dir_of_travel*max_preset[ant].Pos)))
          #elif ENDSTOP_OPT == 2               // End stop sensors implemented
          // End_Stop switches can inhibit movement
          //
          // DOWN switch has been pushed and we're not at lower limit of range
          else if (dn_button_push() && !flag.endstop_lower) 
          #elif ENDSTOP_OPT == 3               // Butterfly capacitor, no end stops required          
          // No movement inhibits whatsoever
          //
          else if (dn_button_push() )          // DOWN switch has been pushed
          #endif
          {
            delta_Pos[ant]--;                  // Update position
            dn_button_last_down = true;        // Prepare for Backlash Comp
            dn_button_backlash_return = false;
            flag.manual_move = true;           // Disable backlash when using encoder
          }
          //
          // Ugly cludge to trigger a backlash only if DOWN Button has just been released
          // (required because of the rate reducer, othwerwise multible backlashes triggered)
          //
          else if (dn_button_last_down)        // Trigger a backlash
          {
            delta_Pos[ant]-=8;                 // Overshoot to ensure we go past microstep resolution
            dn_button_last_down = false;
            dn_button_backlash_return = true;  // Prep for return from overshoot
          }
          else if (dn_button_backlash_return)  // Return from backlash overshoot
          {
            delta_Pos[ant]+=8;                 // Overshoot to ensure we go past microstep resolution
            dn_button_backlash_return = false; // Prep for return from overshoot
          }
          button_rate_reducer = UP_DOWN_RATE;  // Reduce stepper rate
        }
        
        //-------------------------------------------------------------------
        // Manual Mode Move through stored memory presets
        //
        // This function is only active if we are NOT receiving data from Radio
        if (!radio.online && !swr.tune 
           && ((controller_settings.trx[controller_settings.radioprofile].radio != MAX_RADIO)
           || !controller_settings.pseudo_vfo))
        {
          if (up_button_toggle() )
          {
            // Fetch Frequency from the Next Higher Preset
            #if ANT_CHG_2BANKS                  // Dual antenna mode, two memory banks
            if (range <= ant*MAX_PRESETS/2 + num_presets[ant] -2)  // Make sure we are within set range
            #else                               // Single or Dual antenna mode with non-overlapping frequency ranges,
                                                // one memory bandk and automatic changeover
            if (range <= (num_presets[0] + num_presets[1] + num_presets[2]-2))  // Make sure we are within set range
            #endif  
            {
              antenna_select(preset[range+1].Frq);  // Switch to appropriate antenna, if past a changeover pos
              running[ant].Frq = preset[range+1].Frq;        
            }  
          }
          else if (dn_button_toggle() )
          {
            // Fetch Frequency from the Next Lower Preset
            if (range >= 2)                    // Make sure we are within set range
            {
              antenna_select(preset[range-1].Frq);  // Switch to appropriate antenna, if past a changeover pos
              running[ant].Frq = preset[range-1].Frq;              
            }
          }
        }
        
        #if PSWR_AUTOTUNE       
        //-------------------------------------------------------------------
        // SWR Tune already in progress, Up Tune or Down Tune has been requested
        //
        else if (swr.tune)
        {
          if (up_button_toggle() )
          {
            swr.up_mode_request = true;        // Switch to Up Tune Mode
          }
          else if (dn_button_toggle() )
          {
            swr.down_mode_request = true;      // Switch to Down Tune Mode
          }
        }
        #endif        
      }

      //-------------------------------------------------------------------
      // Rotate stepper based on any changes to position, as calculated by
      // track_frq, or enacted by Encoder or Up/Down pushbuttons
      if (!flag.manual_move)                     // Enable backlash
      {
        rotate_stepper_b(microstep_skip, true);  // Down direction with a backlash
      }
      else
      {
        rotate_stepper_b(microstep_skip, false); // No backlash
      }
      flag.manual_move = false;                  // Enable backlash      

      // Decrease Rate Reducer - used with UP/Down buttons
      //
      if (button_rate_reducer > 0)
      {
        button_rate_reducer--;
      }
    }
    skip_step++;                               // Advance the stepper_loop_rate reducer
  
    #if PSWR_AUTOTUNE                      // See #define PSWR_AUTOTUNE in ML.h
    // We measure SWR before every stepper move, when Stepper is active.
    //  This is to assist SWR tune, both automatic and manual
    if (flag.stepper_active) measure_power_and_swr();
  
    // If SWR Autotune mode and we are stable - check for high SWR
    if (!flag.stepper_active && !swr.tune && controller_settings.swrautotune && (fwd_power_mw >= MIN_PWR_FOR_SWR_CALC))
    {
      // Autotune if SWR is too high and we haven't failed to tune the three last times
      if ((measured_swr > ((controller_settings.swr_ok + 10)/10.0) ) && (swr.fail_counter < 3))
      {
        // Ensure the stepper motor has settled from last time before making an attempt to auto-retune
        if (SWRautoretune_timer == 0)
        {
          swr.tune_request = true;         // Initiate a tune request
          SWRtune_timer = SWRTUNE_TIMEOUT; // Tune has been requested, seed timer
        }
      }
    }
    
    //-------------------------------------------------------------------
    // SWR Tune - when requested
    //
    // Tune has been requested, Radio is set up
    if (swr.tune && (radio.tuneinit == DONE) && (swr.tune_status == WORKING))
    {
      if      (swr.up_mode)   swr.tune_status = SWR_TuneUp();
      else if (swr.down_mode) swr.tune_status = SWR_TuneDown();
      else                    swr.tune_status = SWR_Hunt();
    }
    #endif
    
    //-------------------------------------------------------------------
    // Switch fixed capacitors in/out, based on frequency, if used
    int32_t frq;
    #if BND_CHG_BY_ACTUAL_FRQ
    frq = running[ant].Frq;           // Frequency information from Radio
    #else
    frq = tunedFrq;                   // Frequency informaiton calculated from position of variable capacitor
    #endif
    if (frq >= bnd3_changeover[ant])
    {
      digitalWrite(bnd_bit1,HIGH);    // Bit order can easily be modified by swapping HIGH / LOW
      digitalWrite(bnd_bit2,HIGH);    // Default pins for bnd_select1 and 2 are 24 and 25 (pads underneath microcontroller)
    }
    else if (frq >= bnd2_changeover[ant])
    {
      digitalWrite(bnd_bit1,LOW);
      digitalWrite(bnd_bit2,HIGH);     
    }
    else if (frq >= bnd1_changeover[ant])
    {
      digitalWrite(bnd_bit1,HIGH);
      digitalWrite(bnd_bit2,LOW);
    }
    else
    {
      digitalWrite(bnd_bit1,LOW);
      digitalWrite(bnd_bit2,LOW);
    }

    #if DRV8825STEPPER  // ML.h selection: A Pololu (Allegro) A4988 or (TI) 8825 Stepper motor controller carrier board
    //-------------------------------------------------------------------
    // Finalize stepper Move Pulse
    //
    drv8825_Move();
    #endif

    //-------------------------------------------------------------------
    // Print from virtual LCD to real LCD, approx one char per millisecond
    // (a typical HD44780 LCD is measured to use approx 15us per character)
    //
    virt_LCD_to_real_LCD();

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*    #if DRV8825STEPPER  // ML.h selection: A Pololu (Allegro) A4988 or (TI) 8825 Stepper motor controller carrier board
    //-------------------------------------------------------------------
    // Finalize stepper Move Pulse
    //
    drv8825_Move();
    #endif
*/
  }

  //-------------------------------------------------------------------------------
  // Here we do routines which are to be accessed once every 1 millisecond
  // Multipurpose Pushbutton
  //-------------------------------------------------------------------------------
  if (pushMetro.check() )                    // check if the metro has passed its interval .
  {
    //-------------------------------------------------------------------
    // Multipurpose (Enact/Menu) Pushbutton state stuff
    //    
    multi_button = multipurpose_pushbutton();    
    if (old_multi_button != multi_button)    // A new state of the Multi Purpose Pushbutton
    {
      if (multi_button == 1)                 // Short Push detected
      {
        flag.short_push = true;              // Used with Configuraton Menu functions
                                             // as well as Stepper Recalibration or Pseudo VFO
      }
      else if (multi_button == 2)            // Long Push detected
      {
        flag.config_menu = true;             // Activate Configuration Menu
      }
      
      #if PSWR_AUTOTUNE    
      else if ((multi_button == 5) && !flag.config_menu) // SWR Initiate request
      {
        swr.tune_request = true;             // Request activation of SWR Autotune
        SWRtune_timer = SWRTUNE_TIMEOUT;     // Tune has been requested, seed timer
      }    
      else if ((multi_button == 4) && !flag.config_menu) // Autotune in case of high SWR - On / Off
      {
        controller_settings.swrautotune = ~controller_settings.swrautotune;  // toggle
        EEPROM_writeAnything(1,controller_settings); // write controller settings into eeprom
  
        virt_lcd_clear();                            // Announce on LCD
        virt_lcd_setCursor(0, 1);
        if (controller_settings.swrautotune)
        {
          virt_lcd_print("SWR AutoTune ON!!!");
          swr.fail_counter = 0;                      // Indicate that last tune operation was a success
        }
        else
        {
          virt_lcd_print("SWR AutoTune OFF!!!");
        }
        Menu_exit_timer = 30;                        // Show on LCD for 3 seconds             
        // Force a reprint of Config Menu upon return, if we went here from there
        flag.menu_lcd_upd = false;
      }
      // Stepper Recalibrate or Pseudo-VFO toggle, as per #define SW6_FOR_RECAL
      // Or -
      // Cycle through Radio Profiles, as per #define SW6_FOR_PROFILE
      else if ((multi_button == 6) && !flag.config_menu)
      {
        #if SW6_FOR_RECAL
        flag.stepper_recalibrate = true;             // Request a Stepper Recalibrate
        
        #elif SW6_FOR_PROFILE
        controller_settings.radioprofile++;          // Cycle through Radio Profiles
        if (controller_settings.radioprofile >= 4) controller_settings.radioprofile = 0;
        // Set out bits to indicate which Radio Settings Profile is active
        if (controller_settings.radioprofile & 0x01) digitalWrite(profile_bit1, HIGH);
        virt_lcd_clear();                            // Announce on LCD
        virt_lcd_setCursor(0, 0);
        sprintf(print_buf,"Radio Profile %1u",controller_settings.radioprofile+1);
        virt_lcd_print(print_buf);
        virt_lcd_setCursor(0, 2);
        virt_lcd_print(radiotext[controller_settings.trx[ controller_settings.radioprofile ].radio]);   
        trx_profile_update();                        // Update all settings relevant to the active profile
        flag.profile_upd = true;                     // Flag to indicate profile has been updated (for EEPROM write)
        meter.power_detected = true;                 // Force drop out of screensaver, if active
        Menu_exit_timer = 20;                        // Show on LCD for 2 seconds

        // Toggle Encoder for VFO on or off
        if (controller_settings.trx[controller_settings.radioprofile].radio == MAX_RADIO) pseudovfo_encoder_toggle();
        #endif
      }
      #endif // End PSWR_AUTOTUNE
    }
    old_multi_button = multi_button;
  }

  #if PSWR_AUTOTUNE
  //-------------------------------------------------------------------------------
  // Here we do routines which are to be accessed once every 5 milliseconds
  // Normal Power/SWR meter mode - when stepper not active or SWR tuning
  //-------------------------------------------------------------------------------
  if (pswrMetro.check() )                    // check if the metro has passed its interval .
  {
    //-------------------------------------------------------------------
    // Normal Power/SWR meter mode - when stepper not active or SWR tuning
    //  
    if (!flag.stepper_active)          // We only do this if the Stepper is not in use
    {                                  // Otherwise measured more rapidly, elsewhere
      measure_power_and_swr();         // Measure Forward & Reverse Power and calculate SWR
    } 
    
    // If Power is high enough to be useful, then indicate for display routing
    #if AD8307_INSTALLED
    if (fwd_power_mw > pow(10,controller_settings.idle_thresh-4))
    #else  
    if (fwd_power_mw > MIN_PWR_FOR_METER)
    #endif
    {
      meter.power_detected = true;
      meter.power_timer = true;
    }
    // Reset indication if no new valid meter power for POWER_METER_TIME
    if (meter.power_detected) 
    {
      if (meter.power_timer)
      {
        meter.power_timer = false;     // Clear flag
        power_reset_timer = 0;         // Reset timer
      }
      power_reset_timer++;

      if (power_reset_timer >= POWER_METER_TIME) // Clear flag if no activity for a couple of seconds
      {
        power_reset_timer = 0;         // Reset timer
        meter.power_detected = false;  // Clear flag
      }
    }
  }
  #endif
    
  //-------------------------------------------------------------------------------
  // Here we do timing related routines which are to be accessed once every 1/10th of a second
  //-------------------------------------------------------------------------------
  if (slowMetro.check() )              // check if the metro has passed its interval .
  {
    //-------------------------------------------------------------------
    // The Menu function has 5 seconds lag time precedence
    if (Menu_exit_timer > 0) Menu_exit_timer--;
    if (Menu_exit_timer == 1) virt_lcd_clear();

    //-------------------------------------------------------------------
    // Turn stepper active flag off if no activity for 200ms
    if (flag.stepper_active) 
    {
      if (flag.stepper_timer)
      {
        stepper_active_timer = 0;      // Reset timer
        flag.stepper_timer = false;    // Clear flag
      }
      stepper_active_timer++;

      if (stepper_active_timer >= 2)   // Clear flag if no activity for 200ms
      {
        stepper_active_timer = 0;      // Reset timer
        flag.stepper_active = false;   // Clear flag
      }
    }
    
    //-------------------------------------------------------------------
    // Store changes to Radio Profile in EEPROM
    #if SW6_FOR_PROFILE
    if ((Menu_exit_timer==0) && flag.profile_upd)  // Write new Profile to EEPROM once timer has expired
    {
      flag.profile_upd = false;
      EEPROM_writeAnything(1,controller_settings); 
    }
    #endif
    
    //-------------------------------------------------------------------
    // Store Frequency in EEPROM  and power down the Stepper Motor when stable
    if (flag.frq_store) 
    {
      if (flag.frq_timer)
      {
        frq_store_timer = 0;               // Reset timer
        flag.frq_timer = false;            // Clear flag
      }
      frq_store_timer++;

      if (frq_store_timer >= 50)           // Store current pos in EEPROM if stable for 5 seconds
      {
        frq_store_timer = 0;                // Reset timer
        flag.frq_store = false;             // Clear flag
        EEPROM_writeAnything(100,running);  // Write current frq/position into EEPROM
        EEPROM_writeAnything(124,delta_Pos);// Write current delta Position into EEPROM
                                            // (delta-pos is accumulated offset, by turning
                                            //  of Encoder and Up/Down buttons)
        EEPROM_writeAnything(136,stepper_track); // Write current stepper tracl into EEPROM
                                            // stepper_track may be different from running.Pos
      }                                     // + delta_Pos, if frequency is outside of range 
      if (frq_store_timer >= 10)            // Power down stepper if stable for 1 second
      {                                
        #if DRV8825STEPPER
        drv8825_PwrOff();                   // Power down the stepper
        #else
        a4975_PwrOff();                     // Power down the stepper
        #endif
      }
    }

    //-------------------------------------------------------------------    
    // Manage radio.online indication
    if (radio.online) 
    {
      if (radio.timer)
      {
        radio_reset_timer = 0;           // Reset timer
        radio.timer = false;             // and trigger a count until activity from radio resets
      }
      else
      {
        radio_reset_timer++;
      }
    }
    if (radio_reset_timer >= 100)        // Go into Manual mode if nothing received for 10 seconds
    {
      radio_reset_timer = 0;             // Reset timer
      radio.online = false;              // Indicate Radio Off line
    }
  
    #if PSWR_AUTOTUNE
    //-------------------------------------------------------------------
    // SWR Tune Mode stuff
    //
    swr_tune_functions();
    #endif
    
    //-------------------------------------------------------------------
    // Configuration Menu Mode or normal Running Mode
    //
    if (!swr.tune)
    {
      // Configuration Menu Mode
      if (flag.config_menu && (Menu_exit_timer == 0))
      {
        ConfigMenu();
      }
  
      // Normal running Mode
      //   
      // Short Push of the Push Button
      else if (flag.short_push)
      {
        flag.short_push = false;           // Clear short push flag     
        //
        // Various things to be done if short push... depending on which mode is active and on which feature
        // is enabled in ML.h
        //
        #if PSWR_AUTOTUNE && RECALIBRATE && SW6_FOR_RECAL                                  
        // If SW6 is implemented for recalibration or profile selection, then short push of SW1 can be used to trigger Pseudo VFO toggle
        if (controller_settings.trx[controller_settings.radioprofile].radio == MAX_RADIO) pseudovfo_encoder_toggle();
        #elif PSWR_AUTOTUNE && RECALIBRATE && !SW6_FOR_RECAL
        // If Recalibrate is enabled but SW6 is not implemented:
        // Use short push of Menu/Enact outside of Menu to trigger Stepper Recalibrate
        // unless in Pseudo VFO, then use SW1 to toggle Encoder for VFO
        if (controller_settings.trx[controller_settings.radioprofile].radio == MAX_RADIO) pseudovfo_encoder_toggle();
        else recalibrate_stepper_pos();
        #elif !PSWR_AUTOTUNE && RECALIBRATE
        // If Recalibrate is enabled but SWR_AUTOTUNE is not implemented:
        // Use short push of Menu/Enact outside of Menu to trigger Stepper Recalibrate
        // unless in Pseudo VFO, then use SW1 to toggle Encoder for VFO
        if (controller_settings.trx[controller_settings.radioprofile].radio == MAX_RADIO) pseudovfo_encoder_toggle();
        else recalibrate_stepper_pos();
        #elif !PSWR_AUTOTUNE && !RECALIBRATE 
        // If Recalibrate is disabled and SWR_AUTOTUNE is not implemented:
        // Use short push of Menu/Enact outside of Menu to toggle Encoder for VFO
        if (controller_settings.trx[controller_settings.radioprofile].radio == MAX_RADIO) pseudovfo_encoder_toggle();
        #endif
        // Let short push trigger a radio frequency poll, if Auto Polling is disabled
        if (poll_rate[controller_settings.trx[controller_settings.radioprofile].radio] == 9999) trx_poll();
      }                                  

      #if PSWR_AUTOTUNE && RECALIBRATE && SW6_FOR_RECAL
      // Stepper Recalibrate Request received from SW6
      if (flag.stepper_recalibrate)
      {
        flag.stepper_recalibrate = false;
        recalibrate_stepper_pos();
      }
      #endif
    }
    
    #if PSWR_AUTOTUNE
    usb_cont_report();                     // Report Power and SWR to USB, if in Continuous mode
    #endif

    #if ANT_CHG_2BANKS && !ANT1_CHANGEOVER // Dual memory banks for two Antennas, read Antenna Changeover Switch
    static int32_t ant_previous;
    // Determine which antenna is selected
    if (digitalRead(ChgOvSW) == HIGH) ant1_changeover = 0;
    else ant1_changeover = 100000000;      // An arbitrary number much higher than 30 MHz.
    if (ant_previous != ant1_changeover)
    {
      antenna_select(ant1_changeover);
      ant_previous = ant1_changeover;
    }
    #endif
  }
  //-------------------------------------------------------------------------------
  // Here we do routines to update text on (virtual) LCD, once every 100 ms seems reasonable
  //-------------------------------------------------------------------------------
  if (lcd_Metro.check() )                  // check if the metro has passed its interval .
  {
    lcd_display();
  }

  //-------------------------------------------------------------------------------
  // Here we do routines needed if TRX Serial Comms Protocol requires frequency to be polled
  //-------------------------------------------------------------------------------
  if (trxpollMetro.check() )               // check if the metro has passed its interval .
  {
    // If Poll Rate is 9999, then manual polling only.
    if (poll_rate[controller_settings.trx[controller_settings.radioprofile].radio] != 9999)
    {
      // Poll for frequency  
      if (!swr.tune)                       // Stop polling for frequency while performing Autotune
      {
        trx_poll();
      }
    }
    else                                   // Manual Poll Mode
    {
      radio.timer = true;                  // Indicate that we are receiving frq data from Radio
      radio.online = true;
    }    
  }
}

void setup()
{
  uint8_t coldstart;
  
  #if DRV8825STEPPER  // ML.h selection: A Pololu (Allegro) A4988 or (TI) DRV8825 Stepper motor controller carrier board
  drv8825_Init();
  #else               // ML.h selection: A pair of A4975 Stepper Controllers
  a4975_Init();                            // Initialize Stepper Motor
  #endif
  
  pinMode(EnactSW, INPUT_PULLUP);          // Initalize Swticthes as input
  pinMode(UpSW, INPUT_PULLUP);
  pinMode(DnSW, INPUT_PULLUP);

  #if ENDSTOP_OPT == 2                     // End stop sensors implemented
  pinMode(EndStopUpper, INPUT_PULLUP);
  pinMode(EndStopLower, INPUT_PULLUP);
  #endif

  pinMode(hardware_ptt, OUTPUT);           // Enable Hardware PTT
  digitalWrite(hardware_ptt, LOW);         // and set to LOW = Off

  pinMode(bnd_bit1, OUTPUT);               // Enable Capacitor Select bits
  pinMode(bnd_bit2, OUTPUT);
  digitalWrite(bnd_bit1, LOW);             // and set to LOW = Off
  digitalWrite(bnd_bit2, LOW);             // and set to LOW = Off
   
  pinMode(profile_bit1, OUTPUT);           // Enable Profile Indicator bits
  pinMode(profile_bit2, OUTPUT);

  pinMode(swralarm_bit, OUTPUT);           // Enable an SWR Alarm Output
  
  coldstart = EEPROM.read(0);              // Grab the coldstart byte indicator in EEPROM for
                                           // comparison with the COLDSTART_REFERENCE
  //
  // Initialize frequency and position memories if first upload, COLDSTART_REF has been modified in ML.h
  // since last upload OR if the "Clear All" command has been issued through the Controller Menu functions (0xfe)
  if (coldstart != COLDSTART_REF)
  { 
    running[0].Frq=14000000;                     // Antenna 1, 14 MHz as a starting point (irrelevant)
    running[0].Pos=1000000;                      // Some large round number
    running[1].Frq=14000000;                     // Antenna 2, 14 MHz as a starting point (irrelevant)
    running[1].Pos=1000000;                      // Some large round number
    running[2].Frq=14000000;                     // Antenna 3, 14 MHz as a starting point (irrelevant)
    running[2].Pos=1000000;                      // Some large round number

    delta_Pos[0] = 0;                            // Antenna 1, Position offset is 0.
    delta_Pos[1] = 0;                            // Antenna 2, Position offset is 0.
    delta_Pos[2] = 0;                            // Antenna 3, Position offset is 0.
    stepper_track[0]=1000000;                    // Antenna 1, Some large round number
    stepper_track[1]=1000000;                    // Antenna 2, Some large round number
    stepper_track[2]=1000000;                    // Antenna 3, Some large round number

    init_Presets();                              // 0 Hz and 1000000 as stepper_pos into all unused presets

    EEPROM_writeAnything(100,running);           // write running frequency/pos into eeprom
    EEPROM_writeAnything(124,delta_Pos);         // write running position offset into eeprom
    EEPROM_writeAnything(136,stepper_track);     // write initial Stepper Position
    EEPROM_writeAnything(148,preset);            // write initialized empty frq/pos pairs ...
                                                 // (number of presets = MAX_PRESETS)   
    EEPROM.write(0,COLDSTART_REF);               // COLDSTART_REF in first byte indicates all initialized
  }
  else                                           // EEPROM contains stored data, retrieve the data
  {
    EEPROM_readAnything(100,running);            // read the current position
    EEPROM_readAnything(124,delta_Pos);          // the current Offset (delta_Pos) and
    EEPROM_readAnything(136,stepper_track);      // the current actual Stepper Position
    EEPROM_readAnything(148,preset);             // the Frequency/Position memory presets
  }

  //
  // Initialize controller_settings if first upload, or if COLDSTART_REF has been modified in ML.h
  // since last upload. We do not react to a 0xfe which siginfies a memory "Delete All"
  if ((coldstart != COLDSTART_REF) && (coldstart != 0xfe))
  {
    controller_settings.radioprofile = 0;                        // Default to Radio Profile 0
    for (uint8_t a=0;a<4;a++)                                    // Init all four Radio Profiles
    {
      controller_settings.trx[a].radio = DEFAULT_RADIO;          // Default values for Radio defined in ML.h
      trx_parameters_init(a);                                    // Initialize controller_settings for default
                                                                 // data rate and serial mode for the selected Radio
                                                   
      controller_settings.trx[a].passthrough = false;            // Default serial<->USB passthrough Off      
      controller_settings.trx[a].ICOM_address = CIV_TRX_ADDRESS; // Default ICOM CI-V Address for transceiver
      controller_settings.trx[a].tx_pwrlevel = default_plevel[DEFAULT_RADIO]; // Default Tune Power level for default radio
    }
    tx_tune_pwr = controller_settings.trx[0].tx_pwrlevel;

    controller_settings.step_rate = 5;           // Default to 500 steps/second
                                                 // (150 RPM if no microsteps, but effectively
                                                 //  75 RPM max, with 8 microsteps and
                                                 //  speedup of 4.
    controller_settings.step_speedup = 2;        // Deafault to 4x ( pow(2,2) )
    controller_settings.microsteps = 0;          // Default to 8 microsteps (0)
    controller_settings.backlash_angle = BACKLASH_ANGLE; // Default angle, see ML.h
    controller_settings.swr_ok = ACCEPTABLE_SWR*10-10;   // 0 - 31, for a SWR of 1.0:1 to 4.1:1 
    controller_settings.swrautotune = false;     // Default SWR Autotune Mode Off
    controller_settings.Scale[0] = SCALE_A;      // User definable Scale Ranges, up to 3 ranges per decade
    controller_settings.Scale[1] = SCALE_B;      // e.g. ... 6W 12W 24W 60W 120W 240W ...
    controller_settings.Scale[2] = SCALE_C;      // If all values set as "2", then ... 2W 20W 200W ...
    
    // This is only used if AD8307 installed - otherwise harmless
    controller_settings.cal_AD8307[0] = {
                  CAL1_NOR_VALUE,
                  CALFWD1_RAW_DEFAULT,           // First Calibrate point, Forward direction, db*10 + 2 x AD values					 
                  CALREV1_RAW_DEFAULT };         // First Calibrate point, Reverse direction, db*10 + 2 x AD values
    controller_settings.cal_AD8307[1] = {  
                  CAL2_NOR_VALUE,
                  CALFWD2_RAW_DEFAULT,           // Second Calibrate point, Forward direction, db*10 + 2 x AD values
                  CALREV2_RAW_DEFAULT };         // Second Calibrate point, Reverse direction, db*10 + 2 x AD values
    controller_settings.idle_thresh = METER_IDLE_THRESH; // Power Meter wake up threshold: 1=1uW, 2=10uW ... 5=10mW
    
    // This is only used if no AD8307, otherwise harmless
    controller_settings.meter_cal = METER_CAL*100; // Calibration fudge of diode detector style meter

    controller_settings.PEP_period = PEP_PERIOD; // Default PEP period (1, 2.5 or 5 seconds)
    
    EEPROM_writeAnything(1,controller_settings); // write controller settings into eeprom
  }
  // Else use pre-existing controller_settings
  else
  {
    EEPROM_readAnything(1,controller_settings);  // Read controller settings from EEPROM and test for validity
    radio_selection = controller_settings.trx[controller_settings.radioprofile].radio;
    rs232_signals = controller_settings.trx[controller_settings.radioprofile].sig_mode + (controller_settings.trx[controller_settings.radioprofile].passthrough << 1);
    rs232_rate = controller_settings.trx[controller_settings.radioprofile].rs232rate;  
    tx_tune_pwr = controller_settings.trx[controller_settings.radioprofile].tx_pwrlevel;  
  }
  
  ant2_changeover = ANT2_CHANGEOVER;             
  #if ANT1_CHANGEOVER                            // Automatic Dual or Triple Antenna Changeover Mode
  ant = 0;                                       // Initial default as first antenna
  ant1_changeover = ANT1_CHANGEOVER;
  // Select antenna based on active frequency
  // If in single antenna mode, with ANT1_CHANGEOVER = 0, this will make it default to 
  // the second antenna (ant = 1)
  if (running[0].Frq >= ant2_changeover ) ant = 2;
  else if (running[0].Frq >= ant1_changeover ) ant = 1;
  
  #elif ANT_CHG_2BANKS && !ANT1_CHANGEOVER       // Dual Antenna Changeover mode, Two memory banks and
  pinMode(ChgOvSW, INPUT_PULLUP);                // enable Manual antenna Changeover Switch  
  // Determine which antenna is selected
  if (digitalRead(ChgOvSW) == HIGH) ant1_changeover = 0;
  else ant1_changeover = 100000000;              // An arbitrary number much higher than 30 MHz.
  antenna_select(ant1_changeover);
  #endif
  
  // Initialize an output pin to indicate which antenna is selected, if feature is in use.
  #if ANALOGOUTPIN                               // Pin A14
  analogWriteResolution(8);
  analogWrite(ant1_select, (ant==1)?255:0);      // and set to selected antenna
  #else                                          // Alternate, Pin 27 (defined in ML.h)
  pinMode(ant1_select, OUTPUT);                  // Enable Antenna Select bit
  digitalWrite(ant1_select, (ant==1)?HIGH:LOW);  // and set to selected antenna
  #endif
  pinMode(ant2_select, OUTPUT);                  // Enable Antenna Select bit
  digitalWrite(ant2_select, (ant==2)?HIGH:LOW);  // and set to selected antenna

  // Set out bits to indicate which Radio Settings Profile is active
  if (controller_settings.radioprofile & 0x01) digitalWrite(profile_bit1, HIGH);
  else digitalWrite(profile_bit1, LOW);
  if (controller_settings.radioprofile & 0x02) digitalWrite(profile_bit2, HIGH);
  else digitalWrite(profile_bit2, LOW);

  step_rate = controller_settings.step_rate;
  step_speedup = controller_settings.step_speedup;
  microstep_resolution = controller_settings.microsteps;
  
  tunedFrq = running[ant].Frq;                   // Initialize pseudo VFO, in case it is in use at start
  init_pseudo_vfo();
     
  // Find total number of active presets, if any, determine outer bounds and find out where we are
  determine_preset_bounds();
  determine_active_range(running[ant].Frq);

  trx_parameters_set(controller_settings.radioprofile); // Set up UART serial port, data rate etc... for TRX poll
  Serial.begin(9600);                                   // initialize USB virtual serial serial port
  
  #if PSWR_AUTOTUNE
  #if WIRE_ENABLED
  // Start I2C on port SDA1/SCL1 (pins 29/30) - 400 kHz
  Wire1.begin(I2C_MASTER,0x00,I2C_PINS_29_30,I2C_PULLUP_INT,I2C_RATE_400); 
  uint8_t i2c_status = I2C_Init();               // Initialize I2C comms
  #endif
  
  // Defining the ADC pins as inputs will cause a hysteresis error around 3.3V/2.
  // In any case, it is not necessary to define them, as the ADC function does it.
  //pinMode(EnactSW, INPUT);                       // AD input for Menu/Enact Switch
  //pinMode(Pfwd, INPUT);                          // AD input for Forward Power measurement
  //pinMode(Pref, INPUT);                          // AD input for Reverse Power measurement
  // Set up the two separate ADCs for synchronous read at 12 bit resolution and lowest possible measurement speed (minimal noise)
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED);       // Sampling speed, ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED);   // Conversion speed
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED);
  adc->adc0->setResolution(12);                        // AD resolution, 12 bits
  adc->adc1->setResolution(12);
  adc->adc0->setAveraging(16);                         // Averaging by taking multiple samples.
  adc->adc1->setAveraging(16);                         // 16 samples takes approx 80us per measurement and is just about
  #endif                                               // good enough for 12 bit resolution w/o too much noise on LSB
  
  //------------------------------------------
  // Initialize LCD and Print Version information (5 seconds)
  //------------------------------------------          
  lcd.begin(20, 4);                              // Initialize a 20x4 LCD  
  lcd.noDisplay();                               // The excessive initialize sequence below helps some OLED LCDs
  lcd.clear();
  lcd.leftToRight();
  lcd.noAutoscroll();
  lcd.noBlink();
  lcd.noCursor();
  lcd.display();
  lcd.home();
  lcd.createChar(0,0);
  
  lcd_bargraph_Init();                           // Initialize LCD Bargraph

  lcd.setCursor(0,0);
  lcd.print(F(STARTUPDISPLAY1));
  delay(300);
  sprintf(print_buf,STARTUPDISPLAY2);
  lcd.setCursor(20-strlen(print_buf),1);
  lcd.print(print_buf);
  
  delay(1200);
  lcd.setCursor(0,2);
  lcd.print((char*)radiotext[controller_settings.trx[controller_settings.radioprofile].radio]);
  lcd.setCursor(0,3);
  lcd.print(F(STARTUPDISPLAY4));
  
  delay(2000);
  lcd.setCursor(0,2);
  lcd.print(F(DISP_CALLSIGN));                   // Display your own callsign during startup (defined in ML.h)
  lcd.setCursor(0,3);
  lcd.print(F("        "));
  sprintf(print_buf,"Version: %s", VERSION);
  lcd.setCursor(20-strlen(print_buf),3);
  lcd.print(print_buf);
  delay(1500);

  #if PSWR_AUTOTUNE
  #if WIRE_ENABLED                               // I2C scan report, if enabled
  lcd.setCursor(0,3);
  if      (i2c_status==1) lcd.print(F("AD7991-0 detected   "));
  else if (i2c_status==2) lcd.print(F("AD7991-1 detected   "));
  else                    lcd.print(F("Using built-in A/D  "));
  delay(1000);	
  #endif

  lcd.setCursor(0,3);                            // Display whether SWR Autotune is on or off
  if (controller_settings.swrautotune)
  {
    lcd.print(F("SWR AutoTune ON!!!  "));
  }
  else
  {
    lcd.print(F("SWR AutoTune OFF!!! "));
  }
  delay(500);
  #endif

  //------------------------------------------
  // In case of DRV8825, A4988 the previous microstepping to either side 
  // needs to be pre-loaded, up to 4 to one side, up to 3 to the other side
  //------------------------------------------     
  #if DRV8825STEPPER
  int8_t microstep;
  microstep = stepper_track[ant]%8;
  //Serial.println(microstep);
  if (microstep > 0)
  {
    if (microstep < 5)   // Positive direction
    {
      for (uint8_t i = 0; i < microstep; i++)
      {
        drv8825_Incr(0);
        delay(1);
        drv8825_Move();
      }
    }
    else                 // Negative direction
    {
      microstep = 8 - microstep;
      for (uint8_t i = 0; i < microstep; i++)
      {
        drv8825_Incr(0);
        delay(1);
        drv8825_Move();
      }      
    }
    delay(100);
    drv8825_PwrOff();
  }
  #endif
  
  antenna_select(running[ant].Frq);              // Select antenna based on current frequency
  radio.timer = true;                            // Indicate that we are receiving frq data from Radio
  radio.online = true;  

  virt_lcd_clear();                              // Prep for going live: LCD clear using the Virtual LCD code
                                                 // for paced print, in order not to interfere with stepper timing 
  swr.tune_status = SUCCESS;                     // To prevent indication of a false message if status requeted before first tune
}
