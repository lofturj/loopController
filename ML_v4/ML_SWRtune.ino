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

#if PSWR_AUTOTUNE
//
//-----------------------------------------------------------------------------------------
//      SWR Tune routines
//-----------------------------------------------------------------------------------------
//

// Token passing flagsfor SWR tune process
struct  {
          unsigned Prepare  : 1;    // Do initial preparations
          unsigned Progress : 2;    // Track tune progress 
          unsigned Ask_SWR  : 1;    // Request new SWR measurement - Yaesu special case...
          unsigned InitPWR  : 1;    // Init Power Setting for Tune
          unsigned InitMode : 1;    // Init Mode Setting for Tune
         } tune; 

int32_t sample[SWR_SAMPLEBUF];      // Ringbuffer for SWR averaging
int32_t mid_swr;                    // The SWR at midpoint, SWR_SAMPLEBUF/2 samples ago

//
//-----------------------------------------------------------------------------------------
//      Find Best SWR - by sampling, summing and squaring a number of SWR values and
//      comparing with a previous similar sum-square, advancing one sample at a time.
//      once this sum-square value is larger than the previous one, and midpoint SWR
//      (SWR value measured at sample_buffer_size/2 samples earlier) is at an acceptable
//      level, then return TRUE, else return FALSE
//      Relies on sample[] being seeded with 999 values before initial SWR search.
//-----------------------------------------------------------------------------------------
//
int8_t find_best_swr(void) 
{
  static int32_t sum_sq_swr;           // [SWR x 10] is summed over SWR_SAMPLEBUF number of steps 
  static int32_t old_sum_sq_swr=99999; // Previous sum
  static int16_t a=0;                  // [SWR x 10] ring buffer counter
  int16_t        b;
  double         max_acceptable_swr;
  
  max_acceptable_swr = (controller_settings.swr_ok + 10)/10.0;
  
  // Find average of last SWR_SAMPLEBUF (10 - 50) measurements with two decimal point accuracy
  // and compare with previous average
  sample[a] = measured_swr*100;     // Get precision of 2 subdecimals.
  a++;                              // Advance ringbuffer counter
  if (a >= SWR_SAMPLEBUF) a = 0;

  // Derive the SWR measured SWR_SAMPLEBUF/2 + 1 times ago
  b = a - (SWR_SAMPLEBUF/2 + 1);
  if (b < 0) b = b + SWR_SAMPLEBUF;
  mid_swr = sample[b];
  
  // Add up the measurements of the ringbuffer and find an average 
  sum_sq_swr = 0;
  for (uint16_t b = 0; b < SWR_SAMPLEBUF; b++) 
  {
    // Sum up the squares of the samples
    sum_sq_swr += sample[b]*sample[b];
  }
  
  // Determine if we have found a dip, return FALSE while not
  if ((sum_sq_swr <= old_sum_sq_swr) || (mid_swr >= (max_acceptable_swr * 100)))
  {
    // Debug SWR dip, USB print one out of every SWR_SAMPLEBUF measurements made
    if (meter.debug_swrdip && (a == 0))
    {
      sprintf(print_buf,"%1.01f ",sample[a]/100.0);
      Serial.print(print_buf);
    }

    old_sum_sq_swr = sum_sq_swr;
    return false;
  }
  // We have found a dip, tidy up and report
  else
  {
    // Debug SWR dip, USB print the last SWR_SAMPLEBUF measurements upon success
    if (meter.debug_swrdip)
    {
      sprintf(print_buf,"\r\nmidSWR: %1.02f\r\n",(double) mid_swr/100);
      Serial.print(print_buf);
      b = a - SWR_SAMPLEBUF;
      if (b < 0) b = b + SWR_SAMPLEBUF;
      for (uint16_t x=0; x<SWR_SAMPLEBUF; x++)
      {
        sprintf(print_buf,"%1.02f ",(double) sample[b]/100);
        Serial.print(print_buf);
        b++;
        if (b >= SWR_SAMPLEBUF) b = 0;
      }
      Serial.println();
    }
    snapshot_swr = (double) mid_swr/100.0;   // Indicate best SWR found
    snapshot_swr_bar = 1000.0 * log10(snapshot_swr);   
    old_sum_sq_swr = 99999;                 // Make ready for next time
    return true;                            // Return success
  }
}
      

//
//-----------------------------------------------------------------------------------------
//  SWR Tune functions, to be polled as rapidly as the stepper is serviced
//            return 1 (TRUE) if autotune success (a minimum SWR found)
//            return 2 (FAIL) if no minimum SWR was found
//            return 0 (WORKING) while still hunting
//
// SWR_Hunt() for hunting around current position.  Default for SWR Tune
// SWR_TuneUp()   for finding best SWR while tuning upwards.  Tunes until timeout or endstop
// SWR_TuneDown() for finding best SWR while tuning downward. Tunes until timeout or endstop
//-----------------------------------------------------------------------------------------
//
 
//-----------------------------------------------------------------------------------------
// SWR_Hunt() for hunting around current position.  Default for SWR Tune
//-----------------------------------------------------------------------------------------
int8_t SWR_Hunt(void)
{
  int32_t step_size;                      // Number of Microsteps to advance at a time
                                 
  step_size = pow(2,microstep_resolution);// 1 microstep is our smallest unit.
                                          // 8 microsteps is our largest unit                                     
  // First time run: Set up initial Tune Offset  
  if (!tune.Prepare)
  {
    tune.Prepare = true;
    delta_Pos[ant] = delta_Pos[ant] - SWR_HUNT_RANGE * 8; // Set up start position for SWR Hunt
    tune.Progress = WORKING;
    tune.Ask_SWR = false;
    radio.swr = false;
    // Prepare SWR average buffer for use
    for (uint16_t a = 0; a < SWR_SAMPLEBUF; a++) sample[a] = 99999;
  } 

  #if ENDSTOP_OPT == 1
  // If at lower endstop, then modify the delta_Pos until resolved
  if (flag.cap_lower_pos || (abs(stepper_track[ant] - min_preset[ant].Pos) < step_size))
  {
    delta_Pos[ant] += step_size;          // Increase by one step
  }
  // If at upper endstop, then we have reached the end of the line without resoution
  else if (flag.cap_upper_pos)
  {
    tune.Prepare = false;
    tune.Progress = FAIL;
  }
  else
  #elif ENDSTOP_OPT == 2
  // If at lower endstop, then modify the delta_Pos until resolved
  if (flag.endstop_lower)
  {
    delta_Pos[ant] += step_size;          // Increase by one step
  }
  // If at upper endstop, then we have reached the end of the line without resoution
  else if (flag.endstop_upper)
  {
    tune.Prepare = false;
    tune.Progress = FAIL;
  }
  else
  #endif 
  // Whenever we are at the expected position, then do stuff
  //The below is equivalent of this if(stepper_track == running.Pos + delta_Pos)
  // taking into account microstep settings:
  if (abs(stepper_track[ant] - (running[ant].Pos + delta_Pos[ant])) < step_size)
  {
    if (fwd_power_mw > MIN_PWR_FOR_SWR_CALC)    // Do we have sufficient power from Radio?
    {
      // Advance while unacceptably high SWR and until Old average lower than new average
      // (gone SWR_SAMPLEBUF/2 number of steps beyond best resonance)
      if (!find_best_swr() )                    // Returns TRUE if we find a good SWR dip
      {
        delta_Pos[ant] += step_size;            // Increase by one step
     
        // If we're here, then the tune process failed.
        if (delta_Pos[ant] >= (tmp_delta_Pos + SWR_HUNT_RANGE * 8))
        {
          tune.Prepare = false;                 // Clean up for next time
          tune.Progress = FAIL;
        }
      }
      // We found the the lowest SWR and passed it, back up and exit      
      else
      {
        // Back off by half the size of the averaging buffer to find best tune
        delta_Pos[ant] = (int32_t) delta_Pos[ant] - step_size * (SWR_SAMPLEBUF/2 + 1);              
        tune.Prepare = false;                   // Clean up for next time
        tune.Progress = SUCCESS;
      }
    }
    // FAIL - Not enough Transmit Power to determine SWR
    else
    {
      tune.Prepare = false;                     // Clean up for next time
      tune.Progress = NOPWR;
    }
  }  
  return tune.Progress;   
}

//-----------------------------------------------------------------------------------------
// SWR_TuneUp() for finding best SWR while tuning upwards
//-----------------------------------------------------------------------------------------
int8_t SWR_TuneUp(void)
{
  int32_t step_size;                      // Number of Microsteps to advance at a time
  int32_t butterfly_travel;               // keep tabs of number of steps scanned while tuning,
                                          // used with Butterfly cap to impose max boundary
                                 
  step_size = pow(2,microstep_resolution);// 1 microstep is our smallest unit.
                                          // 8 microsteps is our largest unit                                     
  // First time run: Set up initial Tune Parameters  
  if (!tune.Prepare)
  {
    tune.Prepare = true;
    tune.Progress = WORKING;
    tune.Ask_SWR = false;
    radio.swr = false;
    butterfly_travel = 0;                 // Only used with butterfly caps
    // Prepare SWR average buffer for use
    for (uint16_t a = 0; a < SWR_SAMPLEBUF; a++) sample[a] = 99999;
  } 

  #if ENDSTOP_OPT == 1
  // If at upper endstop, then we have reached the end of the line without resoution
  if (flag.cap_upper_pos || (abs(stepper_track[ant] - max_preset[ant].Pos) < step_size))
  #elif ENDSTOP_OPT == 2
  // If at upper endstop, then we have reached the end of the line without resoution
  if (flag.endstop_upper)
  #elif ENDSTOP_OPT == 3
  // Butterfly, no endstops - but need to add some max distance for sanity
  if (butterfly_travel >= BUTTERFLY_MAX_TRAVEL)
  #endif 
  {
    tune.Prepare = false;
    tune.Progress = FAIL;
  }
  // Whenever we are at the expected position, then do stuff
  //The below is equivalent of this if(stepper_track == running.Pos + delta_Pos)
  // taking into account microstep settings:
  else if (abs(stepper_track[ant] - (running[ant].Pos + delta_Pos[ant])) < step_size)
  {
    if (fwd_power_mw > MIN_PWR_FOR_SWR_CALC)    // Do we have sufficient power from Radio?
    {
      // Advance while unacceptably high SWR and until Old average lower than new average
      // (gone SWR_SAMPLEBUF/2 number of steps beyond best resonance)
      if (!find_best_swr() )                    // Returns TRUE if we find a good SWR dip
      {
        delta_Pos[ant] += step_size;            // Increase by one step     
        butterfly_travel++;                     // Increment tune travel for Butterfly Cap special case
      }
      // We found the the lowest SWR and passed it, back up and exit      
      else
      {
        // Back off by half the size of the averaging buffer to find best tune
        delta_Pos[ant] = (int32_t) delta_Pos[ant] - step_size * (SWR_SAMPLEBUF/2 + 1);              
        tune.Prepare = false;                   // Clean up for next time
        tune.Progress = SUCCESS;
      }
    }
    // FAIL - Not enough Transmit Power to determine SWR
    else
    {
      tune.Prepare = false;                     // Clean up for next time
      tune.Progress = NOPWR;
    }
  }  
  return tune.Progress;   
}

//-----------------------------------------------------------------------------------------
// SWR_TuneDown() for finding best SWR while tuning downward
//-----------------------------------------------------------------------------------------
int8_t SWR_TuneDown(void)
{
  int32_t step_size;                      // Number of Microsteps to advance at a time
  int32_t butterfly_travel;               // keep tabs of number of steps scanned while tuning,
                                          // used with Butterfly cap to impose max boundary
                                 
  step_size = pow(2,microstep_resolution);// 1 microstep is our smallest unit.
                                          // 8 microsteps is our largest unit                                     
  // First time run: Set up initial Tune Parameters 
  if (!tune.Prepare)
  {
    tune.Prepare = true;
    tune.Progress = WORKING;
    tune.Ask_SWR = false;
    radio.swr = false;
    butterfly_travel = 0;                 // Only used with butterfly caps
    // Prepare SWR average buffer for use
    for (uint16_t a = 0; a < SWR_SAMPLEBUF; a++) sample[a] = 99999;
  } 

  #if ENDSTOP_OPT == 1
  // If at lower endstop, then we have reached the end of the line without resoution
  if (flag.cap_lower_pos || (abs(stepper_track[ant] - min_preset[ant].Pos) < step_size))
  #elif ENDSTOP_OPT == 2
  // If at lower endstop, then we have reached the end of the line without resoution
  if (flag.endstop_lower)
  #elif ENDSTOP_OPT == 3
  // Butterfly, no endstops - but need to add some max distance for sanity
  if (butterfly_travel >= BUTTERFLY_MAX_TRAVEL)
  #endif 
  {
    tune.Prepare = false;
    tune.Progress = FAIL;
  }
  // Whenever we are at the expected position, then do stuff
  //The below is equivalent of this if(stepper_track == running.Pos + delta_Pos)
  // taking into account microstep settings:
  else if (abs(stepper_track[ant] - (running[ant].Pos + delta_Pos[ant])) < step_size)
  {
    if (fwd_power_mw > MIN_PWR_FOR_SWR_CALC)    // Do we have sufficient power from Radio?
    {
      // Advance while unacceptably high SWR and until Old average lower than new average
      // (gone SWR_SAMPLEBUF/2 number of steps beyond best resonance)
      if (!find_best_swr() )                    // Returns TRUE if we find a good SWR dip
      {
        delta_Pos[ant] -= step_size;            // Decrease by one step
        butterfly_travel++;                     // Increment tune travel for Butterfly Cap special case
      }
      // We found the the lowest SWR and passed it, back up and exit      
      else
      {
        // Back off by half the size of the averaging buffer to find best tune
        delta_Pos[ant] = (int32_t) delta_Pos[ant] - step_size * (SWR_SAMPLEBUF/2 + 1);              
        tune.Prepare = false;                   // Clean up for next time
        tune.Progress = SUCCESS;
      }
    }
    // FAIL - Not enough Transmit Power to determine SWR
    else
    {
      tune.Prepare = false;                     // Clean up for next time
      tune.Progress = NOPWR;
    }
  }  
  return tune.Progress;   
}


//
//-----------------------------------------------------------------------------------------
//      Setup for SWR tune.
//      If Transceiver is fully controllable, then set it up to transmit a tune carrier
//      Poll this routine once every 100ms until success.
//      Returns 0 (WORKING) while still working, 1 (SUCCESS) when Transceiver is ready
//      to go,and 2 (FAIL) if transceiver did not respond to commands.
//-----------------------------------------------------------------------------------------
//
int8_t Radio_TuneInit(void)
{
  static uint8_t progress;            // progress: 0=WORKING, 1=SUCCESS, 2=FAIL
  static uint16_t timer;              // response timer
  if (!radio.tunefirst)
  {
    radio.tunefirst = true;           // Init everything once
    radio.pwr = false;                // Ensure we do not have stale Power Level data from Radio
    radio.pwr_available = true;       // Init Power Available Command Supported flag
    radio.mode = false;               // Ensure we do not have stale Mode data from Radio
    tune.InitPWR = false;             // Prepare to ask for Power Level data
    tune.InitMode = false;            // Prepare to ask for Mode data
    radio.ack = WORKING;              // Only used with ICOM Radios
    progress = WORKING;               // Indicate that we are working to retrieve data
  } 

  // Check whether Transmitter is already puming out RF
  measure_power_and_swr();            // Ensure we have one measurement available
  if (fwd_power_mw >= MIN_PWR_FOR_SWR_CALC)
  {
    swr.rfactive_mode = true;         // We're good to go, no setup required
    progress = SUCCESS;               // Setup for Autotune is initiated  
  }
  
  if (!swr.rfactive_mode)             // We need to ask the Radio into Tune Mode
  {
    // Request Power level only once
    if (!radio.pwr && !tune.InitPWR)
    {
      tune.InitPWR = true;
      timer = 10;                     // Give max 1 second for command rsponse
      progress = trx_request_pwr();   // Returns FAIL if not implemented for this radio
    }
    // Only ICOM: If Radio does not support the Power request command, then capture this (some older ICOM)
    else if  (tune.InitPWR && (radio.ack == FAIL))
    {
      radio.pwr = true;
      radio.pwr_available = false;
    }
    // Request Mode only once, if Power level has been received
    if (radio.pwr && !radio.mode && !tune.InitMode)
    {
      tune.InitMode = true;
      timer = 10;                     // Give max 1 second for command rsponse
      progress = trx_request_mode();  // Returns FAIL if not implemented for this radio
    }
    
    timer--;
    
    // If we have received Mode data, then we're good to go
    if (radio.mode)
    {
      trx_set_am_mode();              // AM mode
      delayloop(50);
      if (radio.pwr_available) trx_set_min_pwr();  // Set Minimal power
      delayloop(50);
      trx_set_tx();                   // TX mode
      delayloop(250);                 // Give time for Power to come up
      measure_power_and_swr();        // Ensure we have one measurement available
      progress = SUCCESS;             // Setup for Autotune is initiated  
    }
    
    // Timer has expired without receiving response from radio - FAIL
    else if (timer == 0)
    {
      progress = FAIL;
    }
  }
  return progress;
}

//
//-----------------------------------------------------------------------------------------
//      Return Transceiver from Autotune.
//      (No test for radio response)
//-----------------------------------------------------------------------------------------
//
void Radio_TuneEnd(void)
{
  if (!swr.rfactive_mode)          // Only do this if required
  {    
    trx_set_rx();                  // Back to RX mode
    delayloop(50);
    trx_restore_mode();            // Restore previous mode setting
    delayloop(50);
    if (radio.pwr_available) trx_restore_pwr();   // Restore previous Power Level setting
    radio.mode = false;            // Clean up for next Auto Tune
    radio.pwr = false;
  }
  swr.tune_request = false;
  swr.tune = false;                // Drop out of SWR Tune Mode
  swr.up_mode = false;
  swr.down_mode = false;
  swr.rfactive_mode = false;
  
}

//
//-----------------------------------------------------------------------------------------
//      High Level SWR Tune Mode stuff - polled every 100ms
//-----------------------------------------------------------------------------------------
//
void swr_tune_functions(void)
{
  // Used to prevent an automatic retune before stepper motor has settled from last time
  if (SWRautoretune_timer > 0)
  {
    SWRautoretune_timer--;                   // Count down the Autoretune timer
  }

  // New Request, things to do once...
  if (swr.tune_request && !swr.tune)         // New request
  {
    swr.tune = true;                         // Init everything once
    radio.tuneinit = WORKING;                // Flag to indicate we need to Set up Radio
    radio.tunefirst = false;                 // Flag to indicate whether first Radio TuneInit poll
    tmp_delta_Pos = delta_Pos[ant];          // Offset the delta_Pos for autotune scan      
    swr.tune_status = WORKING;               // Indicate SWR tune is in Progress
    virt_lcd_clear();                        // Announce on LCD
    virt_lcd_setCursor(0, 0); 
    virt_lcd_print("SWR Tune!!!");        
    Menu_exit_timer = 0;                     // Zero Menu Display timer, if previously running
    // Force a reprint of Config Menu upon return, if we went here from there
    flag.menu_lcd_upd = false;

  }
  
  // SWR Tune is indicated as being in progress, do stuff once every 100ms
  //
  if (swr.tune)
  {
    // Stuff to do while SWR Tune timer does not indicate timeout
    //
    if (SWRtune_timer > 0)
    {
      SWRtune_timer--;                       // Count down the SWR tune timer
      
      // Radio Tune Init and special cases for Up tune and Down tune
      //
      if (radio.tuneinit == WORKING)         // Setup Transceiver for SWR tune
      {
        // Instruct Radio to go into Tune Mode (Minimum Power Out and AM or similar)
        radio.tuneinit = Radio_TuneInit();   // If radio.tuneinit == SUCCESS, then SWR tuning enabled   
      }       
      else if((radio.tuneinit == SUCCESS) && swr.up_mode_request && !swr.up_mode)
      {
        swr.up_mode_request = false;
        delta_Pos[ant] = tmp_delta_Pos;      // Restore original position
        swr.up_mode = true;                  // Indicate up mode active (normally hunt mode active first)
        swr.down_mode = false;               // Deactivate down mode, if previously active
        virt_lcd_setCursor(0, 0);            // Announce on LCD
        virt_lcd_print("SWR Tune UP!!!");        
        Menu_exit_timer = 0;                 // Zero Menu Display timer, if previously running
      }
      else if((radio.tuneinit == SUCCESS) && swr.down_mode_request && !swr.down_mode)
      {
        swr.down_mode_request = false;
        delta_Pos[ant] = tmp_delta_Pos;      // Restore original position
        swr.down_mode = true;
        swr.up_mode = false;
        virt_lcd_setCursor(0, 0);            // Announce on LCD
        virt_lcd_print("SWR Tune DOWN!!!");
        Menu_exit_timer = 0;                 // Zero Menu Display timer, if previously running
      }
            
      // Radio Tune end, success and fail conditions
      //
      else if (swr.tune_status == SUCCESS)   // Was the SWR tune Successful?
      {
        virt_lcd_setCursor(0, 0);
        virt_lcd_print("SWR Tune Success!!  ");
        swr.lcd_snapshot = true;             // Allow one last print to LCD, even though
                                             // Menu_exit_timer is seeded.        
        Menu_exit_timer = 30;                // Show on LCD for 3 seconds
        lcd_display();                       // Force display of best SWR 
        Radio_TuneEnd();                     // Instruct Radio back to normal mode
        swr.fail_counter = 0;                // Indicate that last tune operation was a success
        SWRautoretune_timer = 20;            // Set SWR Autoretune timer at 2 seconds
      }
      else if (radio.tuneinit == FAIL)       // Radio failed to respond to setup commands
      {
        Radio_TuneEnd();                     // Instruct Radio back to normal mode (just to make sure)
        delta_Pos[ant] = tmp_delta_Pos;      // Restore the original position
        virt_lcd_setCursor(0, 0); 
        if (controller_settings.trx[controller_settings.radioprofile].radio == 10) // Special case, Pseudo-VFO
        {
          virt_lcd_print("Xmit Carrier to Tune");
        }
        else
        {
          virt_lcd_print("Radio not responding");
        }        
        Menu_exit_timer = 30;                // Show on LCD for 3 seconds
        SWRautoretune_timer = 20;            // Set SWR Autoretune timer at 2 seconds
        if (swr.fail_counter < 3) swr.fail_counter++; // Indicate that last tune operation failed
      }
      else if (swr.tune_status == FAIL)      // Did the the SWR tune fail to find a dip?
      {
        Radio_TuneEnd();                     // Instruct Radio back to normal mode
        delta_Pos[ant] = tmp_delta_Pos;      // Restore the original position
        virt_lcd_setCursor(0, 0); 
        virt_lcd_print("SWR Tune Failed!!   ");         
        Menu_exit_timer = 30;                // Show on LCD for 3 seconds
        SWRautoretune_timer = 20;            // Set SWR Autoretune timer at 2 seconds
        if (swr.fail_counter < 3) swr.fail_counter++; // Indicate that last tune operation failed
      }
      else if (swr.tune_status == NOPWR)     // Insufficient power for SWR measurement
      {
        Radio_TuneEnd();                     // Instruct Radio back to normal mode
        delta_Pos[ant] = tmp_delta_Pos;      // Restore the original position
        virt_lcd_setCursor(0, 0); 
        virt_lcd_print("No RF Detected!!    ");         
        SWRautoretune_timer = 20;            // Set SWR Autoretune timer at 2 seconds
        Menu_exit_timer = 30;                // Show on LCD for 3 seconds
        if (swr.fail_counter < 3) swr.fail_counter++; // Indicate that last tune operation failed
      }
    }
    // SWR Tune timer timeout
    //
    else                                     // Timeout - Indicate SWRtune fail
    {
      delta_Pos[ant] = tmp_delta_Pos;        // Restore the original position
      Radio_TuneEnd();
      virt_lcd_setCursor(0, 0); 
      virt_lcd_print("SWR Tune Timeout!!! ");
      Menu_exit_timer = 30;                   // Show on LCD for 3 seconds      
      SWRautoretune_timer = 20;            // Set SWR Autoretune timer at 2 seconds
      if (swr.fail_counter < 3) swr.fail_counter++; // Indicate that last tune operation failed
    }
  }  
}
#endif
