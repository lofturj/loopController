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
//** along with this program.  If not,vv see <http://www.gnu.org/licenses/>.
//**ggvv
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

#include <stdio.h>
#include <string.h>

#if PSWR_AUTOTUNE
//
//-----------------------------------------------------------------------------------------
//      Send Power and SWR measurement data to the Computer
//-----------------------------------------------------------------------------------------
//
//------------------------------------------
// Instantaneous Power (unformatted) and SWR
void usb_poll_data(void)
{
  if (Reverse)   Serial.print("-");
  Serial.print(power_mw/1000,8);
  Serial.print(F(", "));
  print_swr();
  Serial.println(print_buf);
}
//------------------------------------------
// Instantaneous Power (formatted, mW - kW)
void usb_poll_inst(void)
{
  print_p_mw(power_mw);
  Serial.print(print_buf);
  //------------------------------------------
  // SWR indication
  Serial.print(F(", VSWR"));
  print_swr();
  Serial.println(print_buf);
}
//------------------------------------------
// Peak (100ms) Power (formatted, mW - kW)
void usb_poll_pk(void)
{
  // We only have valid PK power if stepper is not moving
  if (!flag.stepper_active)   print_p_mw(power_mw_pk);
  else print_p_mw(power_mw); 

  Serial.print(print_buf);
  //------------------------------------------
  // SWR indication
  Serial.print(F(", VSWR"));
  print_swr();
  Serial.println(print_buf);
}
//------------------------------------------
// PEP (1s) Power (formatted, mW - kW)
void usb_poll_pep(void)
{
  // We only have valid PEP power if stepper is not moving
  if (!flag.stepper_active)   print_p_mw(power_mw_pep);
  else print_p_mw(power_mw); 
  Serial.print(print_buf);
  //------------------------------------------
  // SWR indication
  Serial.print(F(", VSWR"));
  print_swr();
  Serial.println(print_buf);
}
//------------------------------------------
// Long, Human Readable Format
void usb_poll_long(void)
{
  // Power indication, inst, peak (100ms), pep (1s)
  Serial.println(F("Power (inst, peak 100ms, pep 1s, avg 1s):"));
  if (Reverse) Serial.print(F("-"));
  print_p_mw(power_mw);
  Serial.print(print_buf);
  Serial.print(F(", "));
  // We only have valid PK and PEP power if stepper is not moving
  if (!flag.stepper_active)
  {
    if (Reverse) Serial.print(F("-"));
    print_p_mw(power_mw_pk);
    Serial.print(print_buf);
    Serial.print(F(", "));
    if (Reverse) Serial.print(F("-"));
    print_p_mw(power_mw_pep);
    Serial.println(print_buf);
  }
  else Serial.print(F("PK and PEP currently not available\r\n"));	
  //------------------------------------------
  // Forward and Reflected Power indication, instantaneous only
  Serial.print(F("Forward and Reflected Power (inst):\r\n"));
  print_p_mw(fwd_power_mw);
  Serial.print(print_buf);
  Serial.print(F(", "));
  print_p_mw(ref_power_mw);
  Serial.println(print_buf);
  //------------------------------------------
  // SWR indication
  Serial.print(F("VSWR"));
  print_swr();
  Serial.println(print_buf);
  Serial.println();
}
//------------------------------------------
// AD debug report - prints out raw AD values
void usb_poll_ad_debug(void)
{
  Serial.print(F("AD values: "));
  Serial.print(fwd);
  Serial.print(F(", "));
  Serial.println(ref);
}

//------------------------------------------
// Prints the selected PSWR report type on a continuous basis, once every 100 milliseconds
void usb_cont_report(void)
{
  if (meter.usb_report_cont)
  {
    if (meter.usb_report_type == REPORT_DATA) usb_poll_data();
    else if (meter.usb_report_type == REPORT_INST) usb_poll_inst();
    else if (meter.usb_report_type == REPORT_PK) usb_poll_pk();
    else if (meter.usb_report_type == REPORT_PEP) usb_poll_pep();
    else if (meter.usb_report_type == REPORT_LONG) usb_poll_long();
    else if (meter.usb_report_type == REPORT_AD_DEBUG) usb_poll_ad_debug();
  }
}
#endif

//
//-----------------------------------------------------------------------------------------
//      Parse and act upon an incoming USB command; USB port enumerates as a COM port
//-----------------------------------------------------------------------------------------
//
const char helpstring[] PROGMEM = {
            "Available USB commands:\r\n"
            "\r\n"
            "$frqget            Retrieve running frequency.\r\n"
            "\r\n"
            "$frqset mmkkkhhh   Update running frequency (equivalent to Transceiver serial input).\r\n"
            "                   Frequency is entered in Hz, e.g. 14100000 is 14.1 MHz.\r\n"
            "\r\n"
            "$memoryget         Retrieve all memory presets.\r\n"
            "                   Format of retrieved values is:\r\n"
            "                      A FFFFFFFF PPPPPPP\r\n"
            "                        where\r\n"
            "                      A is memory position\r\n"
            "                      F is frequency in Hz\r\n"
            "                      P is stepper motor position (relative value)\r\n"
            "                        e.g.\r\n"
            "                      0 14000000 1000000\r\n"
            "                      1 14100000 1000100\r\n"
            "                      2 14200000 1000200\r\n"
            "                      ...\r\n"
            "\r\n"
            "$memoryset A FFFFFFFF PPPPPPP\r\n"
            "                   Enter new memory preset.\r\n"
            "                   Format is same as in $memoryget, however only one preset\r\n"
            "                   is entered at a time.  The Presets MUST be entered with the\r\n"
            "                   Frequency memories in an ascending order and the highest\r\n"
            "                   preset memory entered MUST not be higher than the maximum\r\n"
            "                   number of presets available.\r\n"
            "\r\n"
            "$version           Report version and date of firmware.\r\n"
            "\r\n"
            "Commands for managing SWR Tuning over USB:\r\n"
            "\r\n"
            "$swrtune           Request an SWR Tune.\r\n"
            "$swrtuneup         Request an SWR Up Tune.\r\n"
            "$swrtunedown       Request an SWR Down Tune.\r\n"
            "$swrtunestatus     Request result of latest SWR Tune command.\r\n"
            "                   Results are:  SWR Tune in progress\r\n"
            "                                 SWR Tune unsuccessful\r\n"
            "                                 SWR Tune unsuccessful, no power\r\n"
            "                                 SWR Tune success\r\n"
            "\r\n"            
            "$toggleautotune    Toggle SWR Autotune On or Off\r\n"
            "$recalibrate       Position Recalibrate\r\n"
            "\r\n"
            "Power and SWR Meter related commands (if enabled at compile time):\r\n"
            "\r\n"
            "$ppoll             Poll for one single USB serial report, inst power (unformatted).\r\n"
            "$pinst             Poll for one single USB serial report, inst power (human readable).\r\n"
            "$ppk               Poll for one single USB serial report, 100ms peak power (human readable).\r\n"
            "$ppep              Poll for one single USB serial report, pep power (human readable).\r\n"
            "$plong             Poll for one single USB serial report, actual power (inst, pep and avg)\r\n"
            "                   as well as fwd power, reflected power and SWR (long form).\r\n"
            "\r\n"
            "$pcont             USB serial reporting in a continuous mode, 10 times per second.\r\n"
            "\r\n"
            "                   $ppoll, $pinst, $ppk, $ppep or $plong entered after $pcont will\r\n"
            "                   switch back to single shot mode.\r\n"
            "\r\n"
            "$sleeppwrset x     Power above the level defined here will turn the display into meter mode.\r\n"
            "                   x = 0.001, 0.01, 0.1, 1 or 10 mW (milliWatts).\r\n"
            "$sleeppwrget       Return current value.\r\n"
            "\r\n"
            "$tuneset x         x = 1.1 to 4.0. SWR tune threshold.\r\n"
            "$tuneget           Return current value.\r\n"
            "\r\n"
            "$pepperiodset x    x = 1, 2.5 or 5 seconds.  PEP sampling period.\r\n"
            "$pepperiodget      Return current value.\r\n"
            "\r\n"
            #if AD8307_INSTALLED
            "$calset cal1 AD1-1 AD2-1 cal2 AD1-2 AD2-2\r\n"
            "                   Write new calibration values to the meter.\r\n"
            "$calget            Retrieve calibration values.\r\n"
            "\r\n"
            "                   Format of calibration values is:\r\n"
            "                   cal1 AD1-1 AD2-1 cal2 AD1-2 AD2-2\r\n"
            "                                 where:\r\n"
            "                   cal1 and cal2 are calibration setpoints 1 and 2 in 10x dBm\r\n"
            "                                 and\r\n"
            "                   ADx-1 and ADx-2 are the corresponding AD values for\r\n"
            "                   AD1 (forward direction) and AD2 (reverse direction).\r\n"
            "                   (\r\n"
            "                      normally the AD1 and AD2 values for each setpoint would be the same,\r\n"
            "                      however by doing reverse calibration through the Controller Menu functions\r\n"
            "                      it is possible to balance any small differences there might be between the\r\n"
            "                      two AD8307 outputs.\r\n"
            "                      Note that I have not found this to be necessary at all :)\r\n"
            "                   )\r\n"
            #else
            "$calset cal        Set calibration multiplier (10 - 250). 100 equals a voltage multiplier of 1.\r\n"
            "$calget            Retrieve calibration multiplier.\r\n"
            #endif
            "\r\n"
            "$scaleget          Retrieve user definable scale ranges.\r\n"
            "$scaleset          Write new  scale ranges.\r\n"
            "\r\n"
            "                   The scale ranges are user definable, up to 3 ranges per decade,\r\n"
            "                   e.g. 6, 12 and 24 gives:\r\n"
            "                   ... 6mW, 12mW, 24mW, 60mW ... 1.2W, 2.4W, 6W 12W 24W 60W 120W ...\r\n"
            "                   If all three values set as \"2\", then\r\n"
            "                   ... 2W, 20W, 200W ...\r\n"
            "                   The third and largest value has to be less than ten times the first value.\r\n"
            "\r\n"
            "All the below debug assist commands are off by default:\r\n"
            "\r\n"
            "$debug             This one may provide unexpected results !!! :-)\r\n"
            "\r\n"
            "$swrdebug          Toggle (On/Off) Print/Debug the last 32 SWR measurements found if SWR Tune success.\r\n"
            "\r\n"
            "$trxdebug          Toggle (On/Off) Debug Radio serial communications, ASCII - send to USB.\r\n"
            "\r\n"
            "$hexdebug          same as above, HEX.\r\n"
            "\r\n"
            "$addebug           read raw AD input - also works with $pcont, same as $ppoll etc...\r\n"
            "\r\n"
            "These two are useful to establish whether serial communicaitons in the direction from the Controller\r\n"
            "                     to Transceiver are working:\r\n"
            "$settx             Command Transceiver into Transmit Mode.\r\n"
            "$setrx             Command Transceiver into Receive Mode.\r\n"
            "\r\n"
            "Select active Transceiver Profile, may be useful for remote operation:\r\n"
            "$profileset x      Select active Transceiver Setup Profile, where x is 1 to 4.\r\n"
            "$profileget        Retrieve active Transceiver Setup Profile.\r\n"
            "\r\n"
            "$memoryclear       clear all frequency/position memories, same as Menu command (4).\r\n"
            "$memorywipe        full EEPROM wipe - clear all frq/pos memories and all settings to default.\r\n"
            "\r\n"
            "$help              Display the above instructions.\r\n"
            "\r\n" };                   
// Debug ################################################# 
//          "$softreset         soft reset of Controller - same as turning off and on. Probably not very useful
//           $fakepswr POS      Toggle Fake Power and SWR for debug purposes, takes one argument, if given: POS,
//                              where POS is 1000 times the stepper pos indication as given on LCD (integer number)
//           $990               Request meter information from a Yaesu FT990 ( <00><00><00><00><f7> )
//-----------------------------------------------------------------------------------------
//
char incoming_command_string[50];			                           // Input from USB Serial

void usb_parse_incoming(void)
{
  uint8_t x;
  char *pEnd;
  int32_t  frq_in;
  #if PSWR_AUTOTUNE
  uint16_t inp_val;
  double inp_double;
  #endif

  if (!strcasecmp("frqget",incoming_command_string))	           // Retrieve calibration values
  {
    Serial.print(running[ant].Frq);
  }
  
  else if (!strncasecmp("frqset",incoming_command_string,6))     // Write new calibration values
  {
    frq_in = strtol(incoming_command_string+6,&pEnd,10);
    antenna_select(frq_in);      // Antenna switchover, if frequency applies to the other antenna  
    running[ant].Frq = frq_in;
    radio.timer = true;	         // Indicate that we are receiving frq data from Radio
    radio.online = true;
  }
  
  else if (!strcasecmp("memoryget",incoming_command_string))	   // Retrieve calibration values
  {
    // If there are no active presets, then there is nothing to do
    #if ANT_CHG_2BANKS                  // Dual antenna mode, dual memory banks
    if (num_presets[ant] == 0)
    #else                               // Single or Dual antenna mode with automatic changeover
    if ((num_presets[0] + num_presets[1] + num_presets[2]) == 0)
    #endif  
    {
      Serial.println(F("Nothing stored"));
    }
    else
    #if ANT_CHG_2BANKS                  // Dual antenna mode, dual memory banks
    {
      for(x=0; x<num_presets[0]; x++)
      {
        sprintf(print_buf,"%u %lu %lu",x,preset[x].Frq,preset[x].Pos);
        Serial.println(print_buf);
      }
      for(x=0; x<num_presets[1]; x++)
      {
        sprintf(print_buf,"%u %lu %lu",x,preset[MAX_PRESETS/2+x].Frq,preset[MAX_PRESETS/2+x].Pos);
        Serial.println(print_buf);
      }
    }
    #else                               // Single, Dual or Triple antenna mode with automatic changeover
    {
      for(x=0; x<(num_presets[0] + num_presets[1] + num_presets[2]); x++)
      {
        sprintf(print_buf,"%u %lu %lu",x,preset[x].Frq,preset[x].Pos);
        Serial.println(print_buf);
      }
    }
    #endif  
  }
  else if (!strncasecmp("memoryset",incoming_command_string,9))  // Write new calibration values to EEPROM
  {
    if (strlen(incoming_command_string) > 20)                    // Only process if string contains data
    {
      x =  strtol(incoming_command_string+9,&pEnd,10);
      preset[x].Frq = strtol(pEnd,&pEnd,10);
      preset[x].Pos = strtol(pEnd,&pEnd,10);
    }
    // Sort all presets in an ascending order, but with empty positions on top
    preset_sort();
    // Recalculate total number of active presets, if any, determine outer bounds and find out where we are

    EEPROM_writeAnything(148,preset);                            // Store the whole block of frequencies and positions in eeprom      
    determine_preset_bounds();                                   // Determine outer bounds
    determine_active_range(running[ant].Frq);                    // Determine where we are

    // Ensure that the Stepper Motor doesn't start moving - Assume that it is correctly tuned
    if (range)                                                   // We are within a range of stored positions
    {
      stepper_track[ant] = derive_pos_from_frq();                // Initialize stepper_track and running Position
      running[ant].Pos = stepper_track[ant];
      delta_Pos[ant] = 0;
    }
    else if (running[ant].Frq < min_preset[ant].Frq)             // We are below lowest preset
    {
      stepper_track[ant] = min_preset[ant].Pos;                  // Initialize stepper_track and running Position
      running[ant].Pos = stepper_track[ant];                     // to the lowest preset
      delta_Pos[ant] = 0;
      tunedFrq =  min_preset[ant].Frq;                           // Indicate which frequency stepper is tuned to
    }
    else                                                         // We are above highest preset
    {
      stepper_track[ant] = max_preset[ant].Pos;                  // Initialize stepper_track and running Position
      running[ant].Pos = stepper_track[ant];                     // to the highest preset
      delta_Pos[ant] = 0;
      tunedFrq =  max_preset[ant].Frq;                           // Indicate which frequency stepper is tuned to
    } 
  } 
     
  #if PSWR_AUTOTUNE
  else if (!strcasecmp("ppoll",incoming_command_string))         // Poll, if Continuous mode, then switch into Polled Mode
  {
    // Disable continuous USB report mode ($pcont) if previously set
    if (meter.usb_report_cont)
    {
      meter.usb_report_cont = false;
    }
    meter.usb_report_type = REPORT_DATA;
    usb_poll_data();									                           // Send data over USB
  }
  else if (!strcasecmp("pinst",incoming_command_string))	       // Poll for one single Human Readable report
  {
    // Disable continuous USB report mode ($pcont) if previously set
     if (meter.usb_report_cont)
    {
      meter.usb_report_cont = false;
    }
    meter.usb_report_type = REPORT_INST;
   usb_poll_inst();
  }
  else if (!strcasecmp("ppk",incoming_command_string))	         // Poll for one single Human Readable report
  {
    // Disable continuous USB report mode ($pcont) if previously set
     if (meter.usb_report_cont)
    {
      meter.usb_report_cont = false;
    }
    meter.usb_report_type = REPORT_PK;
    usb_poll_pk();
  }
  else if (!strcasecmp("ppep",incoming_command_string))	         // Poll for one single Human Readable report
  {
     // Disable continuous USB report mode ($pcont) if previously set
     if (meter.usb_report_cont)
    {
      meter.usb_report_cont = false;
    }
    meter.usb_report_type = REPORT_PEP;
    usb_poll_pep();
  }
  else if (!strcasecmp("plong",incoming_command_string))	       // Poll for one single Human Readable report
  {
     // Disable continuous USB report mode ($pcont) if previously set
     if (meter.usb_report_cont)
    {
      meter.usb_report_cont = false;
    }
    meter.usb_report_type = REPORT_LONG;
    usb_poll_long();
  }
  else if (!strcasecmp("addebug",incoming_command_string))	     // Poll for one single Human Readable report
  {
     // Disable continuous USB report mode ($pcont) if previously set
     if (meter.usb_report_cont)
    {
      meter.usb_report_cont = false;
    }
    meter.usb_report_type = REPORT_AD_DEBUG;
    usb_poll_ad_debug();
  }
  else if (!strcasecmp("pcont",incoming_command_string))		     // Switch into Continuous Mode
  {
    // Enable continuous USB report mode ($pcont), and write to EEPROM, if previously disabled
    if (!meter.usb_report_cont)
    {
      meter.usb_report_cont = true;
    }
  }
  
  #if AD8307_INSTALLED
  else if (!strcasecmp("calget",incoming_command_string))        // Retrieve calibration values
  {
    Serial.print(F("AD8307 Cal: "));
    sprintf(print_buf,"%4d,%4d,%4d,%4d,%4d,%4d", 
        controller_settings.cal_AD8307[0].db10m,controller_settings.cal_AD8307[0].Fwd,controller_settings.cal_AD8307[0].Rev,
        controller_settings.cal_AD8307[1].db10m,controller_settings.cal_AD8307[1].Fwd,controller_settings.cal_AD8307[1].Rev);
    Serial.println(print_buf);
  }
  else if (!strncasecmp("calset",incoming_command_string,6))     // Write new calibration values
  {
    EEPROM_readAnything(1,controller_settings);
    controller_settings.cal_AD8307[0].db10m = strtol(incoming_command_string+6,&pEnd,10);
    controller_settings.cal_AD8307[0].Fwd = strtol(pEnd,&pEnd,10);
    controller_settings.cal_AD8307[0].Rev = strtol(pEnd,&pEnd,10);
    controller_settings.cal_AD8307[1].db10m = strtol(pEnd,&pEnd,10);
    controller_settings.cal_AD8307[1].Fwd = strtol(pEnd,&pEnd,10);
    controller_settings.cal_AD8307[1].Rev = strtol(pEnd,&pEnd,10);
    EEPROM_writeAnything(1,controller_settings);
  }  
  #else
  else if (!strcasecmp("calget",incoming_command_string))        // Retrieve calibration values
  {
    Serial.print(F("Meter Cal: "));
    Serial.println(controller_settings.meter_cal/100.0,2);
  }
  else if (!strncasecmp("calset",incoming_command_string,6))     // Write new calibration values
  {
    inp_val = strtod(incoming_command_string+6,&pEnd) *100;
    if ((inp_val >= 10) && (inp_val < 251))
    {
      EEPROM_readAnything(1,controller_settings);
      controller_settings.meter_cal = inp_val;
      EEPROM_writeAnything(1,controller_settings);
    }
  }
  #endif

  else if (!strcasecmp("scaleget",incoming_command_string))      // Retrieve scale limits
  {
    Serial.print(F("Scale: "));
    sprintf(print_buf,"%4u,%4u,%4u",
        controller_settings.Scale[0],controller_settings.Scale[1],controller_settings.Scale[2]);
    Serial.println(print_buf);
  }
  else if (!strncasecmp("scaleset",incoming_command_string,8))   // Write new scale limits
  {
    uint8_t r1, r2, r3;	
	
    r1 = strtol(incoming_command_string+8,&pEnd,10);
    r2 = strtol(pEnd,&pEnd,10);
    r3 = strtol(pEnd,&pEnd,10);
    // Bounds dependencies check and adjust
    //
    // Scales 2 and 3 cannot ever be larger than 9.9 times Scale 1
    // Scale 2 is equal to or larger than Scale 1
    // Scale 3 is equal to or larger than Scale 2
    // If two scales are equal, then only two Scale Ranges in effect
    // If all three scales are equal, then only one Scale Range is in effect
    // If Scale 1 is being adjusted, Scales 2 and 3 can be pushed up or down as a consequence
    // If Scale 2 is being adjusted up, Scale 3 can be pushed up
    // If Scale 3 is being adjusted down, Scale 2 can be pushed down
    if (r2 >= r1*10) r2 = r1*10 - 1;
    if (r3 >= r1*10) r3 = r1*10 - 1;
    // Ranges 2 and 3 cannot be smaller than range 1
    if (r2 < r1) r2 = r1;
    if (r3 < r1) r3 = r1;
    // Range 2 cannot be larger than range 3
    if (r2 > r3) r3 = r2;

    EEPROM_readAnything(1,controller_settings);
    controller_settings.Scale[0] = r1;
    controller_settings.Scale[1] = r2;
    controller_settings.Scale[2] = r3;
    EEPROM_writeAnything(1,controller_settings);
  }

  //
  // The below are a bit redundant, as they are fully manageable by the rotary encoder:
  //
  //    $sleeppwrset x    Power below the level defined here will put display into screensaver mode.
  //                      x = 0.001, 0.01, 0.1, 1 or 10 mW (milliwatts)
  //    $sleeppwrget      Return current value	
  // Power/SWR meter wake up threshold, 1=1uW, 2=10uW ... 5=10mW
  
  
  else if (!strncasecmp("sleeppwrset",incoming_command_string,11))
  {
    // Write value if valid
    inp_double = strtod(incoming_command_string+11,&pEnd);
    if ((inp_double==0.001)||(inp_double==0.01)||(inp_double==0.1)||(inp_double==1)||(inp_double==10))
    {
      EEPROM_readAnything(1,controller_settings);
      controller_settings.idle_thresh = log10(inp_double) + 4;
      EEPROM_writeAnything(1,controller_settings);
    }
  }
  else if (!strcasecmp("sleeppwrget",incoming_command_string))
  {
    Serial.print(F("MeterDisplayThreshold (mW) (only if AD8307): "));
    Serial.println(pow(10,controller_settings.idle_thresh-4),3);
  }

  //    $tuneset x       x = 1.1 to 4.0, SWR tune threshold
  //    $tuneget         Return current value
  else if (!strncasecmp("tuneset",incoming_command_string,7))
  {
    // Write value if valid
    inp_double = strtod(incoming_command_string+8,&pEnd);
    inp_val = inp_double * 10 - 10;
    if ((inp_val>0) && (inp_val<=31))
    {
      EEPROM_readAnything(1,controller_settings);
      controller_settings.swr_ok = inp_val;
      EEPROM_writeAnything(1,controller_settings);
    }
  }
  else if (!strcasecmp("tuneget",incoming_command_string))
  {
    Serial.print(F("SWR_Tune_Threshold: "));
    Serial.println((controller_settings.swr_ok+10)/10.0,1);
  }

  //    $pepperiodget x    x = 1, 2.5 or 5 seconds.  PEP sampling period
  //    $pepperiodset      Return current value
  else if (!strncasecmp("pepperiodset",incoming_command_string,12))
  {
    // Write value if valid
    inp_double = strtod(incoming_command_string+12,&pEnd);
    inp_val = inp_double*(PEP_BUFFER/5.0);
    if ((inp_val==(PEP_BUFFER/5.0))||(inp_val==(PEP_BUFFER/2.0))||(inp_val==PEP_BUFFER))
    { 
      EEPROM_readAnything(1,controller_settings);
      controller_settings.PEP_period = inp_val;
      EEPROM_writeAnything(1,controller_settings);
    }
  }
  else if (!strcasecmp("pepperiodget",incoming_command_string))
  {
    Serial.print(F("PEP_period (seconds): "));
    Serial.println(controller_settings.PEP_period/(PEP_BUFFER/5.0),1);
  }
  #endif
  
  else if (!strcasecmp("version",incoming_command_string))       // Report the firmware version
  {
    Serial.println(F("TF3LJ/VE2AO Magnetic Loop Controller"));
    Serial.print(F("Version "));
    Serial.print(VERSION);
    Serial.print(F(" "));
    Serial.println(DATE);
  }
  
  // Debug ################################################# 
  else if (!strncasecmp("fakepswr",incoming_command_string,8))   // Debug Radio serial comms - send to USB
  {
    if (strlen(incoming_command_string) > 10)                    // Only process if string contains data
    {
      fakepswr_val =  strtol(incoming_command_string+8,&pEnd,10);
      Serial.print(F("$fakepswr "));
      Serial.println(fakepswr_val);
      meter.debug_fakepswr = true;
    }    
    else
    {
      Serial.print(F("$fakepswr "));
      meter.debug_fakepswr = ~meter.debug_fakepswr;
      if (meter.debug_fakepswr)
      {
        sprintf(print_buf,"%lu On", fakepswr_val);
        Serial.println(print_buf);
      }
      else Serial.println(F("Off "));
    }
  }

  else if (!strcasecmp("trxdebug",incoming_command_string))      // Debug Radio serial comms - send to USB
  {
    Serial.print(F("Transceiver Serial Debug Mode [ASCII] "));
    radio.debug_trx = ~radio.debug_trx;
    radio.debug_hex = false;
    if (radio.debug_trx) Serial.println(F("On "));
    else Serial.println(F("Off "));
  }
  else if (!strcasecmp("hexdebug",incoming_command_string))      // Debug Radio serial comms - send to USB
  {
    Serial.print(F("Transceiver Serial Debug Mode [HEX] "));
    radio.debug_hex = ~radio.debug_hex;
    radio.debug_trx = false;
    if (radio.debug_hex) Serial.println(F("On "));
    else Serial.println(F("Off "));
  }
    
  else if (!strcasecmp("swrdebug",incoming_command_string))      // Debug Radio serial comms - send to USB
  {
    Serial.print(F("$swrdebug "));
    meter.debug_swrdip = ~meter.debug_swrdip;
    if (meter.debug_swrdip) Serial.println(F("On "));
    else Serial.println(F("Off "));
  }

  else if (!strcasecmp("debug",incoming_command_string))	       // Retrieve calibration values
  {    
    if ((num_presets[0] + num_presets[1] + num_presets[2]) == 0) // If no stored presets, then nothing to do
    {
      Serial.println(F("Nothing stored"));
    }
    else
    #if ANT_CHG_2BANKS                  // Dual antenna mode, dual memory banks
    {
      for(x=0; x<(num_presets[0]); x++)
      {
        sprintf(print_buf,"%u %lu %lu", x, preset[x].Frq, preset[x].Pos);
        Serial.println(print_buf);
      }
      for(x=MAX_PRESETS/2; x<(MAX_PRESETS/2+num_presets[1]); x++)
      {
        sprintf(print_buf,"%u %lu %lu", x, preset[x].Frq, preset[x].Pos);
        Serial.println(print_buf);
      }
    }
    #else                               // Single, Dual or Triple antenna mode with automatic changeover
    {
      for(x=0; x<(num_presets[0] + num_presets[1] + num_presets[2]); x++)
      {
        sprintf(print_buf,"%u %lu %lu", x, preset[x].Frq, preset[x].Pos);
        Serial.println(print_buf);
      }
    }
    #endif  

    Serial.println();
    Serial.print(F("Antenna: "));       // Indicate which antenna is in use
    Serial.println(ant+1);
    Serial.print(F("Presets: "));
    Serial.print(num_presets[0]);       // Indicate number of programmed presets for each antenna
    Serial.print(F(", "));
    Serial.print(num_presets[1]);
    Serial.print(F(", "));
    Serial.println(num_presets[2]);
    Serial.println();
    
    #if ((ANT1_CHANGEOVER > 0) || (ANT_CHG_2BANKS))      // Dual or Triple antenna configuration
    #if ANT_CHG_2BANKS                // Dual antenna mode, dual memory banks
    Serial.println(F("Manual Dual Antenna Mode"));
    sprintf(print_buf,"Active Antenna (1 or 2): %u", ant+1);
    Serial.println(print_buf);
    #else                             // Single or Dual antenna mode with automatic changeover
    Serial.println(F("Automatic Dual or Triple Antenna Mode"));
    sprintf(print_buf,"Active Antenna (1, 2 or 3): %u", ant+1);
    Serial.println(print_buf);
    #endif  
    Serial.println();
    Serial.println(F("Antenna 1:"));
    sprintf(print_buf,"Running Frq/Pos: %ld %ld", running[0].Frq, running[0].Pos);
    Serial.println(print_buf);
    sprintf(print_buf,"Stepper Track  : %ld", stepper_track[0]);
    Serial.println(print_buf);
    sprintf(print_buf,"Delta Pos      : %ld", delta_Pos[0]);
    Serial.println(print_buf);
    sprintf(print_buf,"Min F/P        : %ld, %ld", min_preset[0].Frq,min_preset[0].Pos);
    Serial.println(print_buf);
    sprintf(print_buf,"Max F/P        : %ld, %ld", max_preset[0].Frq,max_preset[0].Pos);
    Serial.println(print_buf);
    Serial.println();
    Serial.println(F("Antenna 2:"));
    sprintf(print_buf,"Running Frq/Pos: %ld %ld", running[1].Frq, running[1].Pos);
    Serial.println(print_buf);
    sprintf(print_buf,"Stepper Track  : %ld", stepper_track[1]);
    Serial.println(print_buf);
    sprintf(print_buf,"Delta Pos      : %ld", delta_Pos[1]);
    Serial.println(print_buf);
    sprintf(print_buf,"Min F/P        : %ld, %ld", min_preset[1].Frq,min_preset[1].Pos);
    Serial.println(print_buf);
    sprintf(print_buf,"Max F/P        : %ld, %ld", max_preset[1].Frq,max_preset[1].Pos);
    Serial.println(print_buf);
    Serial.println();
    #if !ANT_CHG_2BANKS
    Serial.println(F("Antenna 3:"));
    sprintf(print_buf,"Running Frq/Pos: %ld %ld", running[2].Frq, running[2].Pos);
    Serial.println(print_buf);
    sprintf(print_buf,"Stepper Track  : %ld", stepper_track[2]);
    Serial.println(print_buf);
    sprintf(print_buf,"Delta Pos      : %ld", delta_Pos[2]);
    Serial.println(print_buf);
    sprintf(print_buf,"Min F/P        : %ld, %ld", min_preset[2].Frq,min_preset[2].Pos);
    Serial.println(print_buf);
    sprintf(print_buf,"Max F/P        : %ld, %ld", max_preset[2].Frq,max_preset[2].Pos);
    Serial.println(print_buf);
    Serial.println();
    sprintf(print_buf,"Presets        : %d+%d+%d=%d", num_presets[0],num_presets[1], num_presets[2], num_presets[0]+num_presets[1]+num_presets[2]);
    Serial.println(print_buf);
    #else
    sprintf(print_buf,"Presets        : %d+%d=%d", num_presets[0],num_presets[1],num_presets[0]+num_presets[1]);
    Serial.println(print_buf);
    #endif
    
    #else                                                // Single antenna configuration (variables for second antenna used)
    Serial.println(F("Single Antenna Mode"));

    Serial.println(F("Antenna Frequency and Stepper data-:"));
    sprintf(print_buf,"Running Frq/Pos: %ld %ld", running[1].Frq, running[1].Pos);
    Serial.println(print_buf);
    sprintf(print_buf,"Stepper Track  : %ld", stepper_track[1]);
    Serial.println(print_buf);
    sprintf(print_buf,"Delta Pos      : %ld", delta_Pos[1]);
    Serial.println(print_buf);
    sprintf(print_buf,"Min F/P        : %ld, %ld", min_preset[1].Frq,min_preset[1].Pos);
    Serial.println(print_buf);
    sprintf(print_buf,"Max F/P        : %ld, %ld", max_preset[1].Frq,max_preset[1].Pos);
    Serial.println(print_buf);
    sprintf(print_buf,"Presets        : %d", num_presets[1]);
    Serial.println(print_buf);
    #endif    
    sprintf(print_buf,"Active Range   : %d", range);
    Serial.println(print_buf);  
    Serial.println();
    
    if (!radio.online) Serial.println(F("Radio is OFF-Line"));  
    else Serial.println(F("Radio is ON-Line"));
    Serial.println();

    Serial.println(F("Radio and Serial Port Settings-:"));
    Serial.print(F("Radio Profile (1 to 4): "));
    Serial.println(controller_settings.radioprofile+1);    
    Serial.print(F("Radio: "));
    Serial.println(radiotext[controller_settings.trx[controller_settings.radioprofile].radio]);
    uint32_t rs232rate;
    if (controller_settings.trx[controller_settings.radioprofile].rs232rate < 6) rs232rate = 1200 * 
        (1 << controller_settings.trx[controller_settings.radioprofile].rs232rate);                      // 1200 - 38200 b/s   
    else if (controller_settings.trx[controller_settings.radioprofile].rs232rate == 6) rs232rate = 57600;// 57600 b/s
    else rs232rate = 115200;                                                                             // 115200 b/s
    sprintf(print_buf,"Serial Rate %lu b/s", rs232rate);
    Serial.println(print_buf);
    if(controller_settings.trx[controller_settings.radioprofile].sig_mode) Serial.print("Serial Mode: RS232");
    else Serial.print(F("Serial Mode: TTL"));
    // The below is stupid - passthru mode doesn't allow USB commands to be entered :-)
    if (controller_settings.trx[controller_settings.radioprofile].passthrough) Serial.print(" + Serial<->USB passthrough");
    Serial.println();
    Serial.println();

    Serial.println(F("Stepper Settings-:"));  
    int8_t rate_div = (3-microstep_resolution) - step_speedup;
    if (rate_div > 0) rate_div = pow(2,rate_div);
    else rate_div = 1;
    #if STEPPER_SPEED_IN_RPM            // Display Stepper Rate in RPM
    sprintf(print_buf,"Speed         :%4.0f RPM",(step_rate*6000.0*STEPPER_ANGLE)/(360.0*rate_div));
    #else                               // Display Stepper Rate in Steps/Second
    sprintf(print_buf,"Speed         :%4u Steps/Second",step_rate*100/rate_div);
    #endif
    Serial.println(print_buf);  
    sprintf(print_buf,"SpeedUp       :%4ux",(uint8_t) pow(2,step_speedup));
    Serial.println(print_buf);  
    sprintf(print_buf,"Microsteps    :%4u",(uint8_t) pow(2,3-microstep_resolution));
    Serial.println(print_buf); 
    //if (stepper_backlash) Serial.println(F("Backlash Compensation Enabled"));  
    //else Serial.println(F("Backlash Compensation Disabled"));
    Serial.print(F("Backlash Angle: "));  
    sprintf(print_buf,"%3u",controller_settings.backlash_angle * 5);
    Serial.print(print_buf); 
    Serial.println(F(" steps"));  
    Serial.println();  
  }
  
  else if (!strcasecmp("memoryclear",incoming_command_string))   // Clear all Frq/Pos memories
  {
    Serial.println(F("$memoryclear"));
    // Clear all Frequency/Position memories - all radio and stepper settings are left intact
    EEPROM.write(0,0xfe);
    SOFT_RESET();
  }
  else if (!strcasecmp("memorywipe",incoming_command_string))    // Full reset of Memory
  {
    Serial.println(F("$memorywipe"));
    // Force a full EEPROM update upon reboot by storing 0xff in the first address
    EEPROM.write(0,0xff);
    SOFT_RESET();
  }

  else if (!strcasecmp("softreset",incoming_command_string))     // Reset Microcontroller
  {
    Serial.println(F("$softreset"));                             // Probably not very useful
    SOFT_RESET();
  }
  
  else if (!strcasecmp("help",incoming_command_string))          // Print out USB command help
  {
    Serial.println(F(helpstring));
  }
  // Debug Stuff
  else if (!strcasecmp("settx",incoming_command_string))         // Set Transceiver Transmit Mode (debug)
  {
    Serial.println(F("Transmit ON"));
    trx_set_tx();
  }
  else if (!strcasecmp("setrx",incoming_command_string))         // Set Transceiver Receive Mode (debug)
  {
    Serial.println(F("Transmit OFF"));
    trx_set_rx();
  }
  else if (!strcasecmp("990",incoming_command_string))           // Yaesu FT990 Poll Meters
  {
    Serial.println(F("FT 990 Poll Meters"));
    ft990_read_meter();
  }

  else if (!strcasecmp("swrtune",incoming_command_string))       // Initiate a SWR Tune upon request
  {
    Serial.println(F("SWR Tune requested"));
    swr.tune_request = true;
    SWRtune_timer = SWRTUNE_TIMEOUT;                             // Tune has been requested, seed timer
  }
  else if (!strcasecmp("swrtuneup",incoming_command_string))     // Initiate a SWR Up Tune upon request
  {
    Serial.println(F("SWR Up Tune requested"));
    swr.tune_request = true;
    swr.up_mode_request = true;
    SWRtune_timer = SWRTUNE_TIMEOUT;                             // Tune has been requested, seed timer
  }
  else if (!strcasecmp("swrtunedown",incoming_command_string))   // Initiate a SWR Down Tune upon request
  {
    Serial.println(F("SWR Down Tune requested"));
    swr.tune_request = true;
    swr.down_mode_request = true;
    SWRtune_timer = SWRTUNE_TIMEOUT;                             // Tune has been requested, seed timer
  }
  else if (!strcasecmp("swrtunestatus",incoming_command_string)) // Initiate a SWR Down Tune upon request
  {
    if (swr.tune_status == WORKING) Serial.println(F("SWR Tune in progress"));
    if (swr.tune_status == FAIL)    Serial.println(F("SWR Tune unsuccessful"));
    if (swr.tune_status == NOPWR)   Serial.println(F("SWR Tune unsuccessful, no power"));
    if (swr.tune_status == SUCCESS) Serial.println(F("SWR Tune success"));
  }
  else if (!strcasecmp("recalibrate",incoming_command_string))   // Recalibrate Position indication
  {
    Serial.println(F("recalibrate"));
    recalibrate_stepper_pos();
  }
  else if (!strcasecmp("toggleautotune",incoming_command_string))// Toggle SWR Autotune On or Off
  {
    controller_settings.swrautotune = ~controller_settings.swrautotune; // toggle
    EEPROM_writeAnything(1,controller_settings);                 // write controller settings into eeprom
 
    if (controller_settings.swrautotune)
    {
      Serial.println(F("SWR AutoTune On"));
      swr.fail_counter = 0;                                      // Indicate that last tune operation was a success
    }
    else
    {
      Serial.println(F("SWR AutoTune OFF"));
    }
  }    
  
  else if (!strncasecmp("profileset",incoming_command_string,10))// Set active Transceiver Profile
  {
    if (strlen(incoming_command_string) > 10)                    // Only process if string contains data
    {
      x = strtol(incoming_command_string+10,&pEnd,10);
      if ((x>=1) && (x<=4)) controller_settings.radioprofile = x-1;// Copy to Radio Profile, if valid data
      EEPROM_writeAnything(1,controller_settings);
      Serial.print(F("$Radio Profile "));
      Serial.print(controller_settings.radioprofile+1);
      sprintf(print_buf,", %s",radiotext[controller_settings.trx[ controller_settings.radioprofile ].radio]);
      Serial.println(print_buf);
      // Update all settings relevant to the active profile
      trx_profile_update();
    }
  }
  else if (!strcasecmp("profileget",incoming_command_string))    // Get active Transceiver Profile
  {
    Serial.print(F("$Radio Profile "));
    Serial.print(controller_settings.radioprofile + 1);
    sprintf(print_buf,", %s",radiotext[controller_settings.trx[ controller_settings.radioprofile ].radio]);
    Serial.println(print_buf);
  }
}	

//
//-----------------------------------------------------------------------------------------
// 			Monitor USB Serial port for an incoming USB command
//-----------------------------------------------------------------------------------------
//
void usb_read_and_parse(void)
{
  static uint8_t a;                     // Indicates number of chars received in an incoming command
  static bool Incoming;
  uint8_t ReceivedChar;
  uint8_t waiting;                      // Number of chars waiting in receive buffer

  waiting = Serial.available();         // Find out how many characters are waiting to be read.

  // CAT Passthrough mode. Pass everything from USB (computer) to serial (radio)
  // Only do this if SWR tune is not in progress 
  if (controller_settings.trx[controller_settings.radioprofile].passthrough && !swr.tune)
  {
    while (waiting > 0)
    {
      ReceivedChar = Serial.read();
      Uart.write(ReceivedChar);
      waiting--;
    }
  }
  // Normal mode, grab and parse commands
  else
  {
    // Scan for a command attention symbol -> '$'
    if (waiting && !Incoming)
    {
      ReceivedChar = Serial.read();
      // A valid incoming message starts with an "attention" symbol = '$'.
      // in other words, any random garbage received on USB port is ignored.
      if (ReceivedChar == '$')          // Start command symbol was received,
      {                                 // we can begin processing input command
        Incoming = true;
        a = 0;
        waiting--;
      }
      //else ***********************ADD UART Receive here
    }	
    // Input command is on its way.  One or more characters are waiting to be read
    // and Incoming flag has been set. Read any available bytes from the USB OUT endpoint
    while (waiting && Incoming)
    {
      ReceivedChar = Serial.read();
      waiting--;
      if (a == sizeof(incoming_command_string)-1)  // Line is too long, discard input
      {
        Incoming = false;
        a = 0;
      }
      // Check for End of line
      else if ((ReceivedChar=='\r') || (ReceivedChar=='\n') || (ReceivedChar==';'))
      {
        incoming_command_string[a] = 0; // Terminate line
        usb_parse_incoming();           // Parse the command
        Incoming = false;
        a = 0;
      }
      else                              // Receive message, char by char
      {
        incoming_command_string[a] = ReceivedChar;
      }
      a++;                                         // String length count++
    }
  }
}
