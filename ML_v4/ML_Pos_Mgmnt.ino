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

//
//---------------------------------------------------------------------------------
// Stepper Position and Memories Management
//---------------------------------------------------------------------------------
//

//
//-----------------------------------------------------------------------------------------
// Initialize Presets
//-----------------------------------------------------------------------------------------
//
void init_Presets(void)
{
  uint8_t i;

  // Scan through all "active" memories
  for (i=0; i<MAX_PRESETS; i++)
  {
    preset[i].Frq = 0;      // An unused frequency preset is Zero Hz.
    preset[i].Pos = 1000000;// An unused Position is 1000000
  }
}


//
//-----------------------------------------------------------------------------------------
// Normalize Stepper Positions, lowest is always 1000000
//-----------------------------------------------------------------------------------------
//
void normalize_stepper_pos(void)
{
  uint8_t i;
  int32_t delta;

  // First antenna
  if (num_presets[0] > 1)
  {
    delta = preset[0].Pos - 1000000;
 
    // Impose some boundaries
    if (running[0].Pos < min_preset[0].Pos)
    {
      running[0].Pos = min_preset[0].Pos - delta;
      stepper_track[0] = min_preset[0].Pos - delta;
    }
    else if (running[0].Pos > max_preset[0].Pos)
    {
      running[0].Pos = max_preset[0].Pos - delta;
      stepper_track[0] = max_preset[0].Pos - delta;
    }
    else
    {
      running[0].Pos = running[0].Pos - delta;
      stepper_track[0] = stepper_track[0] - delta;
    }
 
    preset[0].Pos = 1000000;
    // Scan through all "active" memories for first antenna
    for (i=1; i<num_presets[0]; i++)
    {
      preset[i].Pos = preset[i].Pos - delta;
    }
  } 

  // Second antenna
  if (num_presets[1] > 1)
  {
    #if ANT_CHG_2BANKS                  // Dual antenna mode, dual memory banks
    delta = preset[MAX_PRESETS/2].Pos - 1000000;
    #else                               // Single or Dual antenna mode with automatic changeover
    delta = preset[num_presets[0]].Pos - 1000000;
    #endif  

 
    // Impose some boundaries
    if (running[1].Pos < min_preset[1].Pos)
    {
      running[1].Pos = min_preset[1].Pos - delta;
      stepper_track[1] = min_preset[1].Pos - delta;
    }
    else if (running[1].Pos > max_preset[1].Pos)
    {
      running[1].Pos = max_preset[1].Pos - delta;
      stepper_track[1] = max_preset[1].Pos - delta;
    }
    else
    {
      running[1].Pos = running[1].Pos - delta;
      stepper_track[1] = stepper_track[1] - delta;
    }
    #if ANT_CHG_2BANKS                  // Dual antenna mode, dual memory banks
    preset[MAX_PRESETS/2].Pos = 1000000;
    #else                               // Single or Dual antenna mode with automatic changeover
    preset[num_presets[0]].Pos = 1000000;
    #endif  
    // Scan through all "active" memories for second antenna
    for (i=1; i<num_presets[1]; i++)
    {
      #if ANT_CHG_2BANKS                // Dual antenna mode, dual memory banks
      preset[i + MAX_PRESETS/2].Pos = preset[i + MAX_PRESETS/2].Pos - delta;
      #else                             // Single or Dual antenna mode with automatic changeover
      preset[i + num_presets[0]].Pos = preset[i + num_presets[0]].Pos - delta;
      #endif  
    }
  }
  // Third antenna
  if (num_presets[2] > 1)
  {
    delta = preset[ num_presets[0] + num_presets[1] ].Pos - 1000000;

 
    // Impose some boundaries
    if (running[2].Pos < min_preset[2].Pos)
    {
      running[2].Pos = min_preset[2].Pos - delta;
      stepper_track[2] = min_preset[2].Pos - delta;
    }
    else if (running[2].Pos > max_preset[2].Pos)
    {
      running[2].Pos = max_preset[2].Pos - delta;
      stepper_track[2] = max_preset[2].Pos - delta;
    }
    else
    {
      running[2].Pos = running[2].Pos - delta;
      stepper_track[2] = stepper_track[2] - delta;
    }
    preset[ num_presets[0] + num_presets[1] ].Pos = 1000000;
 
    // Scan through all "active" memories for third antenna
    for (i=1; i<num_presets[2]; i++)
    {
      preset[ i + num_presets[0] + num_presets[1] ].Pos = preset[ i + num_presets[0] + num_presets[1] ].Pos - delta;
    }
  } 

  EEPROM_writeAnything(100,running);             // write latest frequency/pos into eeprom
  EEPROM_writeAnything(124,delta_Pos);           // write running position offset into eeprom
  EEPROM_writeAnything(136,stepper_track);       // write initial Stepper Position
  EEPROM_writeAnything(148,preset);              // Store the whole block of frequencies and positions in eeprom      
}


//----------------------------------------------------------------------
// A super inelegant quick and dirty Sort() function for
// Frequency and Position Presets from lowest and up,
// but with zero values in highest positions when empty
//----------------------------------------------------------------------
void preset_sort(void)
{
  uint8_t x, sort;
  int32_t temp;

  #if ANT_CHG_2BANKS                  // Dual antenna mode, dual memory banks
  // Very slow and inelegant :)
  for (x = 0; x < (num_presets[0] - 1); x++)
  {
    for (sort = 0; sort < (num_presets[0] - 1); sort++)
    {
      // Empty elements go up
      if ((preset[sort].Frq == 0) && (preset[sort+1].Frq > 0))
      {
        preset[sort].Frq = preset[sort+1].Frq;
        preset[sort+1].Frq = 0;
  
        preset[sort].Pos = preset[sort+1].Pos;
        preset[sort+1].Pos = 0;
      }
      // Empty elements are not shifted down
      else if (preset[sort+1].Frq == 0);
      // Shift elements, one by one
      else if (preset[sort].Frq > preset[sort+1].Frq)
      {
        temp = preset[sort].Frq;
        preset[sort].Frq = preset[sort+1].Frq;
        preset[sort+1].Frq = temp;
  
        temp = preset[sort].Pos;
        preset[sort].Pos = preset[sort+1].Pos;
        preset[sort+1].Pos = temp;
      }
    }
  }
  for (x = 0; x < (num_presets[1] - 1); x++)
  {
    for (sort = MAX_PRESETS/2; sort < (MAX_PRESETS/2 + num_presets[1] - 1); sort++)
    {
      // Empty elements go up
      if ((preset[sort].Frq == 0) && (preset[sort+1].Frq > 0))
      {
        preset[sort].Frq = preset[sort+1].Frq;
        preset[sort+1].Frq = 0;
  
        preset[sort].Pos = preset[sort+1].Pos;
        preset[sort+1].Pos = 0;
      }
      // Empty elements are not shifted down
      else if (preset[sort+1].Frq == 0);
      // Shift elements, one by one
      else if (preset[sort].Frq > preset[sort+1].Frq)
      {
        temp = preset[sort].Frq;
        preset[sort].Frq = preset[sort+1].Frq;
        preset[sort+1].Frq = temp;
  
        temp = preset[sort].Pos;
        preset[sort].Pos = preset[sort+1].Pos;
        preset[sort+1].Pos = temp;
      }
    }
  }
  // Set all uncalibrated Preset Positions at same value as first Preset
  // First Antenna
  for (x = 1; x < num_presets[0]; x++)
  {
    if (preset[x].Frq == 0)
      preset[x].Pos = min_preset[0].Pos;
  }    
  // Second Antenna
  for (x = 1; x < num_presets[1]; x++)
  {
    if (preset[x + MAX_PRESETS/2].Frq == 0)
      preset[x + MAX_PRESETS/2].Pos = preset[MAX_PRESETS/2].Pos;
  }   
  #else                               // Single or Dual antenna mode with automatic changeover
  // Very slow and inelegant :)
  for (x = 0; x < (num_presets[0] + num_presets[1] + num_presets[2] - 1); x++)
  {
    for (sort = 0; sort < (num_presets[0] + num_presets[1] + num_presets[2] - 1); sort++)
    {
      // Empty elements go up
      if ((preset[sort].Frq == 0) && (preset[sort+1].Frq > 0))
      {
        preset[sort].Frq = preset[sort+1].Frq;
        preset[sort+1].Frq = 0;
  
        preset[sort].Pos = preset[sort+1].Pos;
        preset[sort+1].Pos = 0;
      }
      // Empty elements are not shifted down
      else if (preset[sort+1].Frq == 0);
      // Shift elements, one by one
      else if (preset[sort].Frq > preset[sort+1].Frq)
      {
        temp = preset[sort].Frq;
        preset[sort].Frq = preset[sort+1].Frq;
        preset[sort+1].Frq = temp;
  
        temp = preset[sort].Pos;
        preset[sort].Pos = preset[sort+1].Pos;
        preset[sort+1].Pos = temp;
      }
    }
  }
  // Set all uncalibrated Preset Positions at same value as first Preset
  // First Antenna
  for (x = 1; x < num_presets[0]; x++)
  {
    if (preset[x].Frq == 0)
      preset[x].Pos = min_preset[0].Pos;
  }    
  // Second Antenna
  for (x = 1; x < num_presets[1]; x++)
  {
    if (preset[x + num_presets[0]].Frq == 0)
      preset[x + num_presets[0]].Pos = preset[num_presets[0]].Pos;
  }   
  // Third Antenna
  for (x = 1; x < num_presets[2]; x++)
  {
    if (preset[ x + num_presets[0] + num_presets[1] ].Frq == 0)
      preset[ x + num_presets[0] + num_presets[1] ].Pos = preset[ num_presets[0] + num_presets[1] ].Pos;
  }   
  #endif  
}


//
//-----------------------------------------------------------------------------------------
// Recalibrate Stepper Position if Short Push
//-----------------------------------------------------------------------------------------
//
void recalibrate_stepper_pos(void)
{
  uint8_t i;

  if (num_presets[ant] >= 2)      // If we have more than one memory preset stored
  {
    if (ant == 0)                 // First antenna
    {
      for (i=0; i<num_presets[0]; i++)   // Scan through all "active" memories
      {
        preset[i].Pos = preset[i].Pos + delta_Pos[ant];
      }
    }
    else if (ant == 1)            // Second antenna
    {
      // Scan through all "active" memories
      #if ANT_CHG_2BANKS          // Dual antenna mode, dual memory banks
      for (i=MAX_PRESETS/2; i<(MAX_PRESETS/2 + num_presets[1]); i++)
      #else                       // Single, Dual or Triple antenna mode with automatic changeover
      for (i=num_presets[0]; i<( num_presets[0] + num_presets[1] ); i++)
      #endif  
      {
        preset[i].Pos = preset[i].Pos + delta_Pos[ant];
      }
    }
    #if !ANT_CHG_2BANKS
    else if (ant == 2)            // Third antenna
    {
      for (i=num_presets[0] + num_presets[1]; i<( num_presets[0] + num_presets[1] + num_presets[2] ); i++)
      {
        preset[i].Pos = preset[i].Pos + delta_Pos[ant];
      }   
    }
    #endif

    determine_preset_bounds();    // Adjust preset bounds in line with the recalibration
    delta_Pos[ant] = 0;           // Reset the position delta (delta_Pos) to Zero

    EEPROM_writeAnything(124,delta_Pos);  // write running position offset into eeprom
    EEPROM_writeAnything(148,preset);     // Update all preset memories in EEPROM
  }
  else                             // Don't have an active range of memories to work with
  {
    stepper_track[ant] = stepper_track[ant] - delta_Pos[ant];
    delta_Pos[ant] = 0;
    EEPROM_writeAnything(124,delta_Pos);  // write running position offset into eeprom
  }
}


//
//-----------------------------------------------------------------------------------------
// Select antenna based on whether we are below or above #define ANT1_CHANGEOVER
// or based on manual Antenna Changeover Switch, if implemented
//-----------------------------------------------------------------------------------------
//
void antenna_select(int32_t frq)
{
  static uint8_t   old_ant;                        // Records previous antenna during automatic antenna switching routine 
      
  // Do we need to switch to third antenna?
  // This test is only made if valid ant2_changeover [ = is larger than ant1_changeover ]
  if ( ((frq >= ant2_changeover) && (ant < 2)) && (ant2_changeover > ant1_changeover) )
  {
    EEPROM_writeAnything(100,running);             // write latest frequency/pos into eeprom
    EEPROM_writeAnything(124,delta_Pos);           // write running position offset into eeprom
    EEPROM_writeAnything(136,stepper_track);       // write initial Stepper Position
    ant = 2;
  }
  // Do we need to switch to second antenna?
  else if ((frq >= ant1_changeover) && (frq < ant2_changeover) && (ant != 1))
  {
    EEPROM_writeAnything(100,running);             // write latest frequency/pos into eeprom
    EEPROM_writeAnything(124,delta_Pos);           // write running position offset into eeprom
    EEPROM_writeAnything(136,stepper_track);       // write initial Stepper Position
    ant = 1;
  }
 // Do we need to switch to first antenna?
  else if ((frq < ant1_changeover) && (ant != 0)) 
  {
    EEPROM_writeAnything(100,running);             // write latest frequency/pos into eeprom
    EEPROM_writeAnything(124,delta_Pos);           // write running position offset into eeprom
    EEPROM_writeAnything(136,stepper_track);       // write initial Stepper Position
    ant = 0;
  }
  if (old_ant != ant)
  {
    #if !DRV8825STEPPER
    a4975_PwrOff();                                  // Cut motor current
    #else
    drv8825_PwrOff();
    #endif    
    delayloop(50);                                   // Delay time for current decay
    #if ANALOGOUTPIN
    analogWrite(ant1_select, (ant==1)?255:0);        // Select 2nd Antenna on/off
    #else
    digitalWrite(ant1_select, (ant==1)?HIGH:LOW);    // Select 2nd Antenna on/off
    #endif
    digitalWrite(ant2_select, (ant==2)?HIGH:LOW);    // Select 3rd Antenna on/off
    delayloop(50);                                   // Delay time for relay settling            
    old_ant = ant;                                   // Update antenna select memory
  } 
}


//
//-----------------------------------------------------------------------------------------
// Determine active range, between which two stored presets are we?
// If Endstop Option 1, then Returns 0 if frequency is out of range for the selected antenna
// If Endstop Options 2 or 3, then always return the nearest range, even when outside
//-----------------------------------------------------------------------------------------
//
void determine_active_range(int32_t frq)
{
  uint8_t i;

  #if ANT_CHG_2BANKS                          // Dual antenna mode, dual memory banks
  #if ENDSTOP_OPT == 1
  // Find preset immediately above current frequency
  // Set Range as 0 if frequency is lower than lowest or
  // higher than highest reference frequency
  range = 0;
  for (i=0; i<MAX_PRESETS/2; i++)
  {
    if (frq <= preset[i+(ant*MAX_PRESETS/2)].Frq)
    {
      range = i+(ant*MAX_PRESETS/2);
      break;
    }
  }
  
  // Set range as 0 if invalid antenna selected.
  if ((ant == 0) && (frq >= ant1_changeover)) // Invalid antenna selection
  {
    range = 0;
  }
  if ((ant == 1) && (frq < ant1_changeover))  // Invalid antenna selection
  {
    range = MAX_PRESETS/2;
  }
  #else // ENDSTOP_OPT 2 or 3
  // Find preset immediately above current frequency
  // Set Range as 0 if less than two presets
  if (num_presets[ant] < 2) range = 0;
  else
  {
    for (i=0; i<MAX_PRESETS/2; i++)
    {
      if (frq <= preset[i+(ant*MAX_PRESETS/2)].Frq)
      {
        range = i+(ant*MAX_PRESETS/2);
        break;
      }
    }
    if (range == 0)
    {
      if (i > 0) range = ant*MAX_PRESETS/2 + num_presets[ant] - 1;
      else range = ant*MAX_PRESETS/2 + 1;
    }
  }
  #endif
  #else                                       // Single, Dual or Triple antenna mode with automatic changeover
  #if ENDSTOP_OPT == 1
  // Find preset immediately above current frequency
  // Set Range as 0 if frequency is lower than lowest or
  // higher than highest reference frequency
  range = 0;
  for (i=0; i<(num_presets[0] + num_presets[1] + num_presets[2]); i++)
  {
    if (frq <= preset[i].Frq)
    {
      range = i;
      break;
    }
  }
  
  // Set range as 0 if invalid antenna selected.
  if ((ant == 0) && (frq >= ant2_changeover)) // Invalid antenna selection
  {
    range = 0;
  }
  if ((ant == 2) && (frq < ant1_changeover))  // Invalid antenna selection
  {
    range = 0;
  }
  #else // ENDSTOP_OPT 2 or 3

  // Find preset immediately above current frequency
  // Set Range as 0 if less than two presets
  if (num_presets[ant] < 2) range = 0;
  else
  {
    for (i=0; i<(num_presets[0] + num_presets[1] + num_presets[2]); i++)
    {
      if (frq <= preset[i].Frq)
      {
        range = i;
        break;
      }
    }
    if (range == 0)
    {
      if (i > 0) range = i - 1;
      else range = 1;
    }
  }  
  #endif
  #endif
}/*running[ant].Frq*/
 
//
//-----------------------------------------------------------------------------------------
// Determine the Preset bounds, uppermost and lowermost stored presets for each of the
// two antennas.  Also determine whether we have two or more active presets for each antenna
//-----------------------------------------------------------------------------------------
//
void determine_preset_bounds(void)
{
  uint8_t i;

  #if ANT_CHG_2BANKS                      // Dual antenna mode, dual memory banks
  // Find total number of active presets

  // First antenna
  for (i = 0; i < MAX_PRESETS/2; i++)
  {
    if (preset[i].Frq == 0) break;
  }
  num_presets[0] = i;

  // Second antenna
  for (i = MAX_PRESETS/2; i < MAX_PRESETS; i++)
  {
    if (preset[i].Frq == 0) break;
  }
  num_presets[1] = i - MAX_PRESETS/2;
  
  // Determine lowest and highest Frq/Pos belonging to each antenna
  if (num_presets[0] > 0)                 // We have an active range for the first antenna
  {
    min_preset[0].Frq = preset[0].Frq;    // Lowest and highest presets
    max_preset[0].Frq = preset[num_presets[0]-1].Frq;
    #if FRQ_CALC_BY_RANGE                 // The highest/lowest positions may not be at the end, search...
    min_preset[0].Pos =  9999999;
    max_preset[0].Pos =  0;
    for (i = 0; i < num_presets[0]; i++)
    {
      if (preset[i].Pos < min_preset[0].Pos) min_preset[0].Pos = preset[i].Pos;
      if (preset[i].Pos > max_preset[0].Pos) max_preset[0].Pos = preset[i].Pos;
    }
    #else
    min_preset[0].Pos = preset[0].Pos;
    max_preset[0].Pos = preset[num_presets[0]-1].Pos;
    #endif  
  }
  else
  {
    min_preset[0].Frq = 14000000;
    max_preset[0].Frq = 14000000;
    min_preset[0].Pos = 1000000;
    max_preset[0].Pos = 1000000;
  }
  if (num_presets[1] > 0)                 // We have an active range for the second antenna
  {
    min_preset[1].Frq = preset[MAX_PRESETS/2].Frq;  // Lowest and highest presets
    max_preset[1].Frq = preset[MAX_PRESETS/2 + num_presets[1]-1].Frq;
    #if FRQ_CALC_BY_RANGE                 // The highest/lowest positions may not be at the end, search...
    min_preset[1].Pos =  9999999;
    max_preset[1].Pos =  0;
    for (i = 0; i < num_presets[0]; i++)
    {
      if (preset[MAX_PRESETS/2 + i].Pos < min_preset[1].Pos) min_preset[1].Pos = preset[MAX_PRESETS/2 + i].Pos;
      if (preset[MAX_PRESETS/2 + i].Pos > max_preset[1].Pos) max_preset[1].Pos = preset[MAX_PRESETS/2 + i].Pos;
    }
    #else
    min_preset[1].Pos = preset[MAX_PRESETS/2].Pos;
    max_preset[1].Pos = preset[MAX_PRESETS/2 + num_presets[1]-1].Pos;
    #endif 
  }
  else
  {
    min_preset[1].Frq = 14000000;
    min_preset[1].Pos = 1000000;
    max_preset[1].Frq = 14000000;
    max_preset[1].Pos = 1000000;
  }
  #else                                   // Single or Dual antenna mode with automatic changeover
  // Find total number of active presets

  // First antenna
  for (i=0; i < MAX_PRESETS; i++)
  {
    if ((preset[i].Frq == 0) || (preset[i].Frq >= ant1_changeover)) break;
  }
  num_presets[0] = i;

  // Second antenna
  for (i=num_presets[0]; i < MAX_PRESETS; i++)
  {
    if ((preset[i].Frq == 0) || (preset[i].Frq >= ant2_changeover)) break;
  }
  num_presets[1] = i - num_presets[0];
  
  // Third antenna
  for (i=num_presets[0] + num_presets[1]; i < MAX_PRESETS; i++)
  {
    if (preset[i].Frq == 0) break;
  }
  num_presets[2] = i - (num_presets[0] + num_presets[1]);

  // Determine lowest and highest Frq/Pos belonging to each antenna
  if (num_presets[0] > 0)                 // We have an active range for the first antenna
  {
    min_preset[0].Frq = preset[0].Frq;    // Lowest and highest presets
    max_preset[0].Frq = preset[num_presets[0]-1].Frq;
    #if FRQ_CALC_BY_RANGE                 // The highest/lowest positions may not be at the end, search...
    min_preset[0].Pos =  9999999;
    max_preset[0].Pos =  0;
    for (i = 0; i < num_presets[0]; i++)
    {
      if (preset[i].Pos < min_preset[0].Pos) min_preset[0].Pos = preset[i].Pos;
      if (preset[i].Pos > max_preset[0].Pos) max_preset[0].Pos = preset[i].Pos;
    }
    #else
    min_preset[0].Pos = preset[0].Pos;
    max_preset[0].Pos = preset[num_presets[0]-1].Pos;
    #endif
  }
  else
  {
    min_preset[0].Frq = 14000000;
    max_preset[0].Frq = 14000000;
    min_preset[0].Pos = 1000000;
    max_preset[0].Pos = 1000000;
  }
  if (num_presets[1] > 0)                 // We have an active range for the second antenna
  {
    min_preset[1].Frq = preset[num_presets[0]].Frq;  // Lowest and highest presets
    max_preset[1].Frq = preset[num_presets[0] + num_presets[1]-1].Frq;
    #if FRQ_CALC_BY_RANGE                 // The highest/lowest positions may not be at the end, search...
    min_preset[1].Pos =  9999999;
    max_preset[1].Pos =  0;
    for (i = 0; i < num_presets[1]; i++)
    {
      if (preset[num_presets[0] + i].Pos < min_preset[1].Pos) min_preset[1].Pos = preset[num_presets[0] + i].Pos;
      if (preset[num_presets[0] + i].Pos > max_preset[1].Pos) max_preset[1].Pos = preset[num_presets[0] + i].Pos;
    }
    #else  
    min_preset[1].Pos = preset[num_presets[0]].Pos;
    max_preset[1].Pos = preset[num_presets[0] + num_presets[1]-1].Pos;
    #endif
  }
  else
  {
    min_preset[1].Frq = 14000000;
    max_preset[1].Frq = 14000000;
    min_preset[1].Pos = 1000000;
    max_preset[1].Pos = 1000000;
  }
  if (num_presets[2] > 0)                 // We have an active range for the third antenna
  {
    min_preset[2].Frq = preset[num_presets[0] + num_presets[1]].Frq;  // Lowest and highest presets
    max_preset[2].Frq = preset[num_presets[0] + num_presets[1] + num_presets[2]-1].Frq;
    #if FRQ_CALC_BY_RANGE                 // The highest/lowest positions may not be at the end, search...
    min_preset[2].Pos =  9999999;
    max_preset[2].Pos =  0;
    for (i = 0; i < num_presets[2]; i++)
    {
      if (preset[num_presets[0] + num_presets[1] + i].Pos < min_preset[2].Pos) min_preset[2].Pos = preset[num_presets[0] + num_presets[1] + i].Pos;
      if (preset[num_presets[0] + num_presets[1] + i].Pos > max_preset[2].Pos) max_preset[2].Pos = preset[num_presets[0] + num_presets[1] + i].Pos;
    }
    #else     
    min_preset[2].Pos = preset[num_presets[0] + num_presets[1]].Pos;
    max_preset[2].Pos = preset[num_presets[0] + num_presets[1] + num_presets[2]-1].Pos;
    #endif
  }
  else
  {
    min_preset[2].Frq = 14000000;
    max_preset[2].Frq = 14000000;
    min_preset[2].Pos = 1000000;
    max_preset[2].Pos = 1000000;
  }
  #endif
}


//
//-----------------------------------------------------------------------------------------
// Derive Stepper Position from Frequency
//-----------------------------------------------------------------------------------------
//
int32_t derive_pos_from_frq(void)
{
  int64_t delta_R, delta_F;
  int64_t delta_calc;
  int32_t pos=1000000;

  if (num_presets[ant] >=2)                           // Pos doesn't change until we have two or more presets
  {
    #if ENDSTOP_OPT == 1
    if (running[ant].Frq < min_preset[ant].Frq)       // We are below lowest preset
    {
      pos = min_preset[ant].Pos;
    }
    else if (running[ant].Frq > max_preset[ant].Frq)  // We are above highest preset
    {
      pos = max_preset[ant].Pos;
    }
    else                                              // We are within presets
    #endif
    {
      determine_active_range(running[ant].Frq);
      // deltas used to derive Stepper Position from Frequency
      delta_R = preset[range].Pos - preset[range-1].Pos;
      delta_F = preset[range].Frq - preset[range-1].Frq;
      // calculate Stepper Position
      delta_calc  = (delta_R * (running[ant].Frq - preset[range-1].Frq))/delta_F;
      pos = delta_calc + preset[range-1].Pos;
    }
  }
  return pos;
}


//
//-----------------------------------------------------------------------------------------
// Derive tuned Frequency from Stepper Position
//-----------------------------------------------------------------------------------------
//
int32_t derive_frq_from_pos(void)
{
  #if !FRQ_CALC_BY_RANGE
  int8_t  i;
  uint8_t pos_range;                 // Somewhat redundant, as we already have range
                                     // calculated based on frequency. should be same.
  #endif
  int64_t delta_R, delta_F;          // Integer arithmetic can be tricky - need 64 bits
  int64_t delta_calc;
  int32_t frq=0;

  //
  // Calculate Tuned Frequency based on current stepper position
  //
  if (num_presets[ant] > 1)          // We need at least 2 presets to calculate
  {
    #if FRQ_CALC_BY_RANGE
    // deltas used to derive Frequency from Active Frequency Range and Stepper Position
    delta_R = preset[range].Pos - preset[range-1].Pos;
    delta_F = preset[range].Frq - preset[range-1].Frq;
    // calculate Tuned Frequency
    delta_calc = (delta_F * (stepper_track[ant] - preset[range-1].Pos))/delta_R;
    frq  = delta_calc + preset[range-1].Frq;

    #else
    // Find preset immediately above current stepper position
    // Returns 0 if stepper position is lower than or higher than the highest
    // stored position
    // (mabye a bit redundant, as we already calculate range based on frequency)
    pos_range = 0;
    // First antenna
    if (ant == 0)
    {
      for (i=0; i<=num_presets[0]; i++)
      {
        if ((stepper_track[ant]*dir_of_travel) <= (preset[i].Pos*dir_of_travel))
        {
          pos_range = i;
          break;
        }
      }
    }
    // Second antenna
    else if (ant == 1)
    {
      #if ANT_CHG_2BANKS                // Dual antenna mode, dual memory banks
      for (i=MAX_PRESETS/2; i<=(MAX_PRESETS/2 + num_presets[1]); i++)
      #else                             // Single or Dual antenna mode with automatic changeover
      for (i=num_presets[0]; i<=(num_presets[0] + num_presets[1]); i++)
      #endif  
      {
        if ((stepper_track[ant]*dir_of_travel) <= (preset[i].Pos*dir_of_travel))
        {
          pos_range = i;
          break;
        }
      }
    }
    // Third antenna
    else if (ant == 2)
    {
      for (i=num_presets[0] + num_presets[1]; i<=(num_presets[0] + num_presets[1] + num_presets[2]); i++)
      {
        if ((stepper_track[ant]*dir_of_travel) <= (preset[i].Pos*dir_of_travel))
        {
          pos_range = i;
          break;
        }
      }
    }
      
    // If we are below lowest preset, use lowest preset interval to calculate
    if ((stepper_track[ant]*dir_of_travel) < (min_preset[ant].Pos*dir_of_travel))
    {
      if (ant == 0) pos_range = 1;
      #if ANT_CHG_2BANKS                // Dual antenna mode, dual memory banks
      else pos_range = MAX_PRESETS/2 +  1;
      #else                             // Single or Dual antenna mode with automatic changeover
      else if (ant == 1) pos_range = num_presets[0] + 1;
      else if (ant == 2) pos_range = num_presets[0] + num_presets[1] + 1;
      #endif  
    }
    // If we are above highest preset, use highest preset interval to calculate
    else if ((stepper_track[ant]*dir_of_travel) > (max_preset[ant].Pos*dir_of_travel))
    {
      if (ant == 0) pos_range = num_presets[0] - 1;
      #if ANT_CHG_2BANKS                // Dual antenna mode, dual memory banks
      else pos_range = MAX_PRESETS/2 + num_presets[1] - 1;
      #else                             // Single or Dual antenna mode with automatic changeover
      else if (ant == 1) pos_range = num_presets[0] + num_presets[1] - 1;
      else if (ant == 2) pos_range = num_presets[0] + num_presets[1] + num_presets[2] - 1;
      #endif  
    }

    // deltas used to derive Frequency from Stepper Position only
    delta_R = preset[pos_range].Pos - preset[pos_range-1].Pos;
    delta_F = preset[pos_range].Frq - preset[pos_range-1].Frq;
    // calculate Tuned Frequency
    delta_calc = (delta_F * (stepper_track[ant] - preset[pos_range-1].Pos))/delta_R;
    frq  = delta_calc + preset[pos_range-1].Frq;
    #endif
  }  
  return frq;
}


//
//-----------------------------------------------------------------------------------------
// Variable rate, increases away from start/stop positions
//-----------------------------------------------------------------------------------------
//
uint8_t determine_variable_rate(void)
{
  static int32_t rampup;         // Used to ramp up the stepper rate when movement starts
  
  static int8_t  rate;           // Stepper rate calculate
                                 // Ultimately this value contains the Microstep resolution
                                 // if used:
                                 // 0 is slowest, full resolution of 8 microsteps
                                 // 1 for 4 microsteps, 2 for 2 microsteps
                                 // or 3 for no microsteps
  
  uint32_t       distance_left;  // Used to keep tabs on how far we are from destination

  int8_t         divisor;        // Loop Rate divisor as pow(2,div), 0 for fastest... 
                                 // opposite of rate
                                 // In order to be able to speed up, we need to start slower :)

  uint16_t rampup_angle;         // Rate Ramp-up Angle as a function of Backlash Angle or a set minimum value
                           
  // Only make changes to the rate when at a full step
  if (stepper_track[ant]%8 == 0) // Modulo division by 8 for finding full step
  {
    // Base rate as determined by User Menu settings - typical value is 0 for 8 microsteps
    rate = controller_settings.microsteps;
  
    // Determine the stepper rate rampup angle
    if (controller_settings.backlash_angle < 2)
      rampup_angle = 200;
    else
      rampup_angle = controller_settings.backlash_angle * 100;
    
    // Determine the Initial Loop Rate Divisor
    if (controller_settings.step_speedup == 3)
    {
      divisor = rate;
    }
    else if   (controller_settings.step_speedup == 2)
    {
      divisor = rate - 1;
      if (divisor < 0) divisor = 0;
    }
    else if   (controller_settings.step_speedup == 1)
    {
      divisor = rate - 2;
      if (divisor < 0) divisor = 0;
    }
    else
    { 
      divisor = rate - 3;
      if (divisor < 0) divisor = 0;   
    }
    
    // Determine total distance we need to move, used for speed determination.
    distance_left = abs(stepper_track[ant] - (running[ant].Pos + delta_Pos[ant]));
    
    // Stay at minimal rate and init rampup if just started or close to desired end position
    if (distance_left <= controller_settings.backlash_angle*10)
    {
      rampup = 0;                // Initialize rampup
    }
    // Otherwise determine the rate, higher as we are further away from origin and destination
    else
    {
      rampup+=8;                 // We are moving, moving, moving
    
      if (controller_settings.step_speedup == 3) // Eight times Rate when tuning "long distance"
      {
        // Eight times the stepper rate if tuning distance is more than 4x ramp-up angle  
        if      ((rampup > (4*rampup_angle)) && (distance_left > (4*rampup_angle)))
        {  
          rate = controller_settings.microsteps + 3;
        }
        // Quadruple the stepper rate if tuning distance is more than 3x ramp-up angle  
        else if ((rampup > (3*rampup_angle)) && (distance_left > (3*rampup_angle)))
        { 
          rate = controller_settings.microsteps + 2;
        }
        // Double the stepper rate if tuning distance is more than 2x rampup_angle
        else if ((rampup > (2*rampup_angle)) && (distance_left > (2*rampup_angle)))
        { 
          rate = controller_settings.microsteps + 1;
        }
      }
      
      else if (controller_settings.step_speedup == 2) // Quadruple Rate when tuning "long distance"
      {
        // Quadruple the stepper rate if tuning distance is more than 3x rampup_angle 
        if      ((rampup > (3*rampup_angle)) && (distance_left > (3*rampup_angle)))
        { 
          rate = controller_settings.microsteps + 2;
        }
        // Double the stepper rate if tuning distance is more than 2x rampup_angle
        else if ((rampup > (2*rampup_angle)) && (distance_left > (2*rampup_angle)))
        { 
          rate = controller_settings.microsteps + 1;
        }
      }
      
      else if (controller_settings.step_speedup == 1) // Double Rate when tuning "long distance"
      {
        // Double the stepper rate if tuning distance is more than 2x rampup_angle 
        if      ((rampup > (2*rampup_angle)) && (distance_left > (2*rampup_angle)))
        { 
          rate = controller_settings.microsteps + 1;
        }
      }
    }
    
    // Distribute Rate Increase between Microstep Reduction and Loop Rate divisor
    if (rate > 3)
    {
      divisor = divisor - (rate - 3);
      rate = 3;
    }
    if (divisor < 0) divisor = 0; // Should not happen, ever
  
    // Return a composite value of Rate (microstep resolution setting)
    // and Divisor (loop rate divisor setting)
    rate = rate + (divisor<<4);   // Divisor stored in upper 4 bits  
  }  
  return rate;  
}


//
//-----------------------------------------------------------------------------------------
// Rotate Stepper Up or Down, based on calculated position vs stepper_track comparison 
//
// Input microstep_rate is inverse of microstep_resolution, 
// where 0 for full resolution of 8 microsteps, 1 for 4 microsteps, 2 for 2 microsteps
// or 3 for no microsteps
//
// Backlash on down if BOOL backlash is TRUE
//-----------------------------------------------------------------------------------------
//
void rotate_stepper_b(uint8_t microstep_rate, uint8_t backlash_comp)
{ 
  int32_t     step_size;                  // Number of Microsteps to advance at a time
  int32_t     angle;                      // Backlash Angle in Steps
  static bool backlash;                   // We are in the middle of backlash comp
                                 
  step_size = pow(2,microstep_rate);      // 1 microstep is our smallest unit.
                                          // 8 microsteps is our largest unit
  angle = controller_settings.backlash_angle * 40; // * 5 * 8 microsteps
  
  // Determine direction of travel, depending on whether second stored memory pos is larger or smaller
  // than first pos
  if ((num_presets[ant] >= 2) && (max_preset[ant].Pos < min_preset[ant].Pos)) dir_of_travel = -1;
  else dir_of_travel = 1;
  
  //
  // Position the Stepper according to Frequency
  //
  // Tune Up
  #if   ENDSTOP_OPT == 1                  // Vacuum Cap, No End stop sensors implemented
  if (!backlash && ((stepper_track[ant] + (step_size-1)) < (running[ant].Pos + delta_Pos[ant])))
  #elif ENDSTOP_OPT == 2                  // End stop sensors implemented
  if (!backlash && !flag.endstop_upper && ((stepper_track[ant] + (step_size-1)) < (running[ant].Pos + delta_Pos[ant])))
  #elif ENDSTOP_OPT == 3                  // No End stop sensors implemented
  if (!backlash && ((stepper_track[ant] + (step_size-1)) < (running[ant].Pos + delta_Pos[ant])))
  #endif
  {
    #if DRV8825STEPPER  // ML.h selection: A Pololu (TI) DRV8825 or (Allegro) A4988 Stepper motor controller carrier board
    drv8825_Incr(microstep_rate);
    #else               // ML.h selection: A pair of A4975 Stepper Controllers
    a4975_Incr(microstep_rate);	
    #endif
    stepper_track[ant] += step_size;      // Increase counter in accordance with step size
    flag.stepper_timer = true;            // Seed stepper active timer
    flag.stepper_active = true;           // Indicate that stepper is in use
    flag.frq_timer = true;                // New frequency, these flags seed timer to monitor if
    flag.frq_store = true;                // frequency has been stable for a while
  }
  
  if (backlash_comp)
  {
    // Backlash Compensation.  Set flag to Tune Down and then back Up
    if (stepper_track[ant] > (running[ant].Pos + delta_Pos[ant]))
    {
      backlash = true;                      // We need to tune town, set Backlash flag
    }
    // Tune Down, overshooting by STEPPER_BACKLASH
    #if   ENDSTOP_OPT == 1                // Vacuum Cap, No End stop sensors implemented
    if (backlash && (stepper_track[ant] > (running[ant].Pos + delta_Pos[ant] - angle))
       && ((dir_of_travel*stepper_track[ant]) > (dir_of_travel*min_preset[ant].Pos)))  
    #elif ENDSTOP_OPT == 2                // End stop sensors implemented  
    if (backlash && !flag.endstop_lower && (stepper_track[ant] > (running[ant].Pos + delta_Pos[ant] - angle)))  
    #elif ENDSTOP_OPT == 3                // No End stop sensors implemented
    if (backlash && (stepper_track[ant] > (running[ant].Pos + delta_Pos[ant] - angle)))  
    #endif
    {
      #if DRV8825STEPPER  // ML.h selection: A Pololu (TI) DRV8825 or (Allegro) A4988 Stepper motor controller carrier board
      drv8825_Decr(microstep_rate);
      #else               // ML.h selection: A pair of A4975 Stepper Controllers
      a4975_Decr(microstep_rate);	
      #endif
      stepper_track[ant] -= step_size;    // Decrease counter in accordance with step size--;
      flag.stepper_timer = true;          // Seed stepper active timer
      flag.stepper_active = true;         // Indicate that stepper is in use
      flag.frq_timer = true;              // New frequency, these flags seed timer to monitor if
      flag.frq_store = true;              // frequency has been stable for a while
    }
    else backlash = false;                // Clear the Backlash bool once we have tuned all the way down
  }
  else
  {
    // Tune Down
    #if   ENDSTOP_OPT == 1                // Vacuum Cap, No End stop sensors implemented
    if (stepper_track[ant] > (running[ant].Pos + delta_Pos[ant]))  
    #elif ENDSTOP_OPT == 2                // End stop sensors implemented  
    if (!flag.endstop_lower && (stepper_track[ant] > (running[ant].Pos + delta_Pos[ant])))  
    #elif ENDSTOP_OPT == 3                // No End stop sensors implemented
    if (stepper_track[ant] > (running[ant].Pos + delta_Pos[ant]))  
    #endif
    {
      #if DRV8825STEPPER  // ML.h selection: A Pololu (TI) DRV8825 or (Allegro) A4988 Stepper motor controller carrier board
      drv8825_Decr(microstep_rate);
      #else               // ML.h selection: A pair of A4975 Stepper Controllers
      a4975_Decr(microstep_rate);  
      #endif
      stepper_track[ant] -= step_size;    // Decrease counter in accordance with step size
      flag.stepper_timer = true;          // Seed stepper active timer
      flag.stepper_active = true;         // Indicate that stepper is in use
      flag.frq_timer = true;              // New frequency, these flags seed timer to monitor if
      flag.frq_store = true;              // frequency has been stable for a while
    }
  }
}
