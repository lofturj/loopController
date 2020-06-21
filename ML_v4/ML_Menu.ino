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
// Mulitipurpose Pushbutton and Rotary Encoder Menu Management Routines
//---------------------------------------------------------------------------------
//

// First Level Menu Items
//
#if AD8307_INSTALLED
const uint8_t level0_menu_size = 19;
#elif PSWR_AUTOTUNE
const uint8_t level0_menu_size = 18;
#else
const uint8_t level0_menu_size = 13;
#endif

const char *level0_menu_items[] =
                        {  "1-New Position",
                           "2-Rewrite Position",
                           "3-Delete Position",
                           "4-Clear All",
                           "5-Stepper Settings",
                           "6-Stepper Backlash",
                           "7-Radio Profile",
                           "8-Radio Type",
                           "9-ICOM CI-V Addr",
                           "10-SerialPort Mode",
                           "11-SerialData Rate",
                           #if PSWR_AUTOTUNE
                           "12-TXtune PowerLvl",
                           "13-SWR Tune Thresh",
                           "14-Meter Scale",
                           "15-Meter Calibrate",
                           "16-Meter PEPperiod",
                           #if AD8307_INSTALLED
                           "17-MeterAwakeSense",
                           "18-Debug Serial",
                           #else
                           "17-Debug Serial",                           
                           #endif
                           #else
                           "12-Debug Serial",                                
                           #endif
                           "0-Exit" };                       
// New Pos Flag
#define NEW_POS_MENU      1

// Manage Pos Flag
#define MANAGE_POS_MENU   2

// Delete Pos Flag
#define DELETE_POS_MENU   3

// Flag for Clear Memories
#define CLEAR_MEMORIES_MENU	10
// Clear Memories menu Items
const uint8_t clear_memories_menu_size = 2;
const char *clear_memories_menu_items[] =
                        {  "1 No  - Exit",
                           "2 Yes - Clear All" };

// Stepper Rate, Speedup and Microsteps
#define STEPPER_SETTINGS_MENU      20
#define STEPPER_RATE_SUBMENU       21
#define STEPPER_SPEEDUP_SUBMENU    22
#define STEPPER_MICROSTEPS_SUBMENU 23
// Stepper Settings Menu
const uint8_t stepper_settings_menu_size = 4;
//
// Nothing here for stepper_rate_submenu()
//
// Stepper Speedup submenu Items
const uint8_t stepper_speedup_submenu_size = 4;
const char *stepper_speedup_submenu_items[] =
                        {  "No Speed Up",
                           "Double Speed",
                           "Quadruple Speed",
                           "Eight Times Speed" };
// Microstep Resolution submenu Items
const uint8_t stepper_microsteps_submenu_size = 4;
const char *stepper_microsteps_submenu_items[] =
                        {  "8 Microsteps",
                           "4 Microsteps",
                           "2 Microsteps",
                           "0 Microsteps" };

// Stepper Backlash Select Flag
#define BACKLASH_SELECT_MENU   25
// Stepper Backlash Select menu Items
const uint8_t backlash_select_menu_size = 2;
const char *backlash_select_menu_items[] =
                        {  "Off",
                           "On" };

// Transceiver Profile Select
#define TRANSCEIVER_PROFILE_MENU  29
// Also uses transceiver_select_menu_items[]

// Transceiver Select
#define TRANSCEIVER_SELECT_MENU   30
// Transceiver Select menu Items
const uint8_t transceiver_select_menu_size = 18;
const char *transceiver_select_menu_items[] =
                        {  "1 ICOM CI-V Auto",
                           "2 ICOM CI-V Poll",
                           "3 Kenwood TS-440",
                           "4 Kenwood TS-870",
                           "5 Kenwood 480/2000",
                           "6 Yaesu FT-100",
                           "7 Yaesu FT-747",
                           "8 Yaesu FT-8X7",
                           "9 Yaesu FT-920",
                           "10 Yaesu FT-990",
                           "11 Yaesu FT-1000MP",
                           "12 Yaesu 1000MPmkV",
                           "13 Yaesu FT-450...",
                           "14 ElecraftK3 Auto",
                           "15 ElecraftK3 Poll",
                           "16 TenTec binary",
                           "17 TenTec ascii",
                           "18 Pseudo-VFO" };

// ICOM CI-V Address select
#define ICOM_CIV_MENU    35

// RS232 Signals Select Flag
#define RS232_SIG_MENU   40
// RS232 Signals menu Items
const uint8_t rs232_sig_menu_size = 4;
const char *rs232_sig_menu_items[] =
                        {  "1 TTL",
                           "2 RS232",
                           "3 TTL   + Passthru",
                           "4 RS232 + Passthru" };

// RS232 Signals Select Flag
#define RS232_RATE_MENU   50
// RS232 Signals menu Items
const uint8_t rs232_rate_menu_size = 8;
const char *rs232_rate_menu_items[] =
                        {  "  1200 b/s",
                           "  2400 b/s",
                           "  4800 b/s",
                           "  9600 b/s",
                           " 19200 b/s",
                           " 38400 b/s",
                           " 57600 b/s",
                           "115200 b/s" };

// Flag for TX Tune PowerLevel adjust menu
#define TX_TUNEPWR_MENU 55


// Flag for SWR tune threshold menu
#define SWR_THRESH_MENU 60

// Flag for Scale Range menu
#define SCALERANGE_MENU 70

// Flags for Scale Range Submenu functions
#define SCALE_SET0_MENU	700
#define SCALE_SET1_MENU	701
#define SCALE_SET2_MENU	702

// Flag for Calibrate menu
#define CAL_MENU        80
#if AD8307_INSTALLED
const uint8_t calibrate_menu_size = 5;
const char *calibrate_menu_items[] =
          {  "1 OneLevelCal(dBm)",
             "2  1st level (dBm)",
             "3  2nd level (dBm)",
             "9 Go Back",
             "0 Exit"  };

// Flags for Calibrate Submenu functions
#define CAL_SET0_MENU         800  // Single level calibration
#define CAL_SET1_MENU         801  // 1st level
#define CAL_SET2_MENU         802  // 2nd level
#endif

// Flag for PEP Sample Period select
#define PEP_MENU              90
// PEP Sample Period select menu Items
const uint8_t pep_menu_size = 3;
const char *pep_menu_items[] =
          {  "1    1s",
             "2  2.5s",
             "3    5s"  };

#if AD8307_INSTALLED
// Flag for Meter awake threshold select menu
#define METER_AWAKE_MENU     95
// Power/SWR meter wakeup threshold select menu Items
const uint8_t meterthresh_menu_size = 5;
const char *meterthresh_menu_items[] =
          {  "1    1uW",
             "2   10uW",
             "3  100uW",
             "4    1mW",
             "5   10mW"  };
#endif		

#define DEBUG_SERIAL_MENU     99

uint16_t   menu_level = 0;  // Keep track of which menu we are in
uint8_t    menu_data = 0;   // Pass data to lower menu


//----------------------------------------------------------------------
// Display a Menu of choices, one line at a time
//
// **menu refers to a pointer array containing the Menu to be printed
//
// menu_size indicates how many pointers (menu items) there are in the array
//
// current_choice indicates which item is currently up for selection if
// pushbutton is pushed
//
// begin row & begin_col are the coordinates for the upper lefthand corner
// of the three or four lines to be printed
//----------------------------------------------------------------------
void lcd_scroll_Menu(char **menu, uint8_t menu_size,
  uint8_t current_choice, uint8_t begin_row, uint8_t begin_col, uint8_t lines)
{
  uint8_t a, x;

  // Clear LCD from begin_col to end of line.
  virt_lcd_setCursor(begin_col, begin_row); 
  for (a = begin_col; a < 20; a++)
    virt_lcd_print(" ");
  if (lines > 1)
  {
    virt_lcd_setCursor(begin_col, begin_row+1); 
    for (a = begin_col; a < 20; a++)
      virt_lcd_print(" ");
  }
  if (lines > 2)
  {
    virt_lcd_setCursor(begin_col, begin_row+2); 
    for (a = begin_col; a < 20; a++)
      virt_lcd_print(" ");
  }
  // Using Menu list pointed to by **menu, preformat for print:
  // First line contains previous choice, second line contains
  // current choice preceded with a '->', and third line contains
  // next choice
  if (current_choice == 0) x = menu_size - 1;
  else x = current_choice - 1;
  if (lines > 1)
  {
    virt_lcd_setCursor(begin_col+2, begin_row); 
    sprintf(print_buf,"%s", *(menu + x));
    virt_lcd_print(print_buf);

    virt_lcd_setCursor(begin_col, begin_row+1); 
    sprintf(print_buf,"->%s", *(menu + current_choice));
    virt_lcd_print(print_buf);
    if (current_choice == menu_size - 1) x = 0;
    else x = current_choice + 1;

    if (lines > 2)
    {
      virt_lcd_setCursor(begin_col+2, begin_row+2);
      sprintf(print_buf,"%s", *(menu + x));
      virt_lcd_print(print_buf);
    }  
  }
  else
  {
    virt_lcd_setCursor(begin_col, begin_row); 
    sprintf(print_buf,"->%s", *(menu + current_choice));
    virt_lcd_print(print_buf);
  }
  // LCD print lines 1 to 3

  // 4 line display.  Preformat and print the fourth line as well
  if (lines == 4)
  {
    if (current_choice == menu_size-1) x = 1;
    else if (current_choice == menu_size - 2 ) x = 0;
    else x = current_choice + 2;
    virt_lcd_setCursor(begin_col, begin_row+3); 
    for (a = begin_col; a < 20; a++)
      virt_lcd_print(" ");
    virt_lcd_setCursor(begin_col+2, begin_row+3); 
    sprintf(print_buf,"  %s", *(menu + x));
    virt_lcd_print(print_buf);
  }
}


//----------------------------------------------------------------------
// Menu functions begin:
//----------------------------------------------------------------------

//
//--------------------------------------------------------------------
// New Pos Menu
//--------------------------------------------------------------------
//
void new_pos_menu(void)
{
  flag.menu_lcd_upd = false;              // Make ready for next time

  #if ANT_CHG_2BANKS                      // Dual antenna mode, dual memory banks
  if (num_presets[ant] == MAX_PRESETS/2)  // Check if no more preset memories available
  #else                                   // Single or Dual antenna mode with automatic changeover
  if ((num_presets[0] + num_presets[1] + num_presets[2]) == MAX_PRESETS) // Check if no more preset memories available
  #endif  
  {
    virt_lcd_setCursor(0, 1); 
    virt_lcd_print("Memory Full!!!");
    Menu_exit_timer = 50;               // Show on LCD for 5 seconds
    menu_level = 0;                     // We're done with this menu level
    flag.config_menu = false;           // We're done
  }
  else
  {
    if (!radio.online)                  // No Frequency information being receiced from Radio
    {
      virt_lcd_clear();
      virt_lcd_setCursor(0, 1); 
      virt_lcd_print("No FRQ information");
      virt_lcd_setCursor(0, 2); 
      virt_lcd_print("received from Radio");
      Menu_exit_timer = 30;             // Show on LCD for 3 seconds
      menu_level = 0;                   // We're done with this menu level
      flag.config_menu = false;         // We're done   
    }
    else
    {
      // Store Current value above the highest existing preset
      #if ANT_CHG_2BANKS                // Dual antenna mode, dual memory banks
      preset[ant*MAX_PRESETS/2 + num_presets[ant]].Frq = running[ant].Frq;
      // Set following positions
      running[ant].Pos = running[ant].Pos + delta_Pos[ant];
      stepper_track[ant] = running[ant].Pos;
      delta_Pos[ant] = 0;
      preset[ant*MAX_PRESETS/2 + num_presets[ant]].Pos = running[ant].Pos;
      #else                             // Single or Dual antenna mode with automatic changeover
      preset[ num_presets[0] + num_presets[1] + num_presets[2] ].Frq = running[ant].Frq;
      // Set following positions
      running[ant].Pos = running[ant].Pos + delta_Pos[ant];
      stepper_track[ant] = running[ant].Pos;
      delta_Pos[ant] = 0;
      preset[ num_presets[0] + num_presets[1] + num_presets[2] ].Pos = running[ant].Pos;
      #endif  
      num_presets[ant]++;

      // Sort all presets in an ascending order, but with empty poitions on top
      preset_sort();
      // Recalculate total number of active presets, if any, determine outer bounds and find out where we are
      determine_preset_bounds();        // Determine outer bounds
      normalize_stepper_pos();          // Normalize (first to 1000000) and store all position memories
      determine_preset_bounds();        // Recalculate outer bounds after normalization
      determine_active_range(running[ant].Frq); // Determine where we are
     
      virt_lcd_clear();
      virt_lcd_setCursor(0, 1); 
      virt_lcd_print("New Preset Stored!!!");
      Menu_exit_timer = 30;             // Show on LCD for 3 seconds
      menu_level = 0;                   // We're done with this menu level
      flag.config_menu = false;         // We're done
    }
  }
}

//--------------------------------------------------------------------
// Overwrite existing Position with new
//--------------------------------------------------------------------
void manage_pos_menu(void)
{
  static int8_t	current_selection;	// Keep track of current menu selection
  
  #if ANT_CHG_2BANKS                // Dual antenna mode, dual memory banks
  if (num_presets[ant] == 0) 
  #else                             // Single Dual or Triple antenna mode with automatic changeover
  if ((num_presets[0] + num_presets[1] + num_presets[2]) == 0) 
  #endif  
  {
    flag.short_push = true;             // Nothing has been stored - nothing to do.
    flag.menu_lcd_upd = true;           // No LCD servicing required
  }
  
  // Selection modified by encoder.  We remember last selection, even if exit and re-entry
  if (Enc.read()/ENC_MENURESDIVIDE != 0)
  {
    if (Enc.read()/ENC_MENURESDIVIDE > 0)
    {
      current_selection++;
    }
    else if (Enc.read()/ENC_MENURESDIVIDE < 0)
    {
      current_selection--;
    }
    // Reset data from Encoder
    Enc.write(0);

    // Indicate that an LCD update is needed
    flag.menu_lcd_upd = false;
  }

  if (!flag.menu_lcd_upd)               // Need to update LCD
  {
    flag.menu_lcd_upd = true;           // We have serviced LCD
  
    // Keep Encoder Selection Within Bounds of the Menu Size
    #if ANT_CHG_2BANKS                  // Dual antenna mode, dual memory banks
    uint8_t menu_size = num_presets[ant] + 1;
    #else                               // Single, Dual or Triple antenna mode with automatic changeover
    uint8_t menu_size = num_presets[0] + num_presets[1]  + num_presets[2] + 1;
    #endif  
    while(current_selection >= menu_size)
      current_selection -= menu_size;
    while(current_selection < 0)
      current_selection += menu_size;

    virt_lcd_clear();
    
    // Print the Menu
    if (current_selection < (num_presets[0] + num_presets[1] + num_presets[2]))
    {
      virt_lcd_print("Overwrite FRQ Preset");
      virt_lcd_setCursor(0,1); 
      sprintf(print_buf,"->%2u ",current_selection);
      virt_lcd_print(print_buf);
      
      #if ANT_CHG_2BANKS                // Dual antenna mode, dual memory banks
      display_frq(preset[ant*MAX_PRESETS/2+current_selection].Frq);
      #else                             // Single or Dual antenna mode with automatic changeover
      display_frq(preset[current_selection].Frq);
      #endif  

      virt_lcd_print(" Hz");
      
      virt_lcd_setCursor(0,2); 
      virt_lcd_print("with ");
      display_frq(running[ant].Frq);
      virt_lcd_print(" Hz");
      virt_lcd_setCursor(0,3); 
      virt_lcd_print("Rotate and select:");
    }    
    else
    {
      virt_lcd_print("No change");
      virt_lcd_setCursor(0,1); 
      virt_lcd_print("-> Exit");  
      virt_lcd_setCursor(0,3); 
      virt_lcd_print("Rotate and select:");
    }
  }

  if (flag.short_push)
  {
    flag.short_push = false;            // Clear pushbutton status
 
    // Nothing has been stored, lets get out of here
    #if ANT_CHG_2BANKS                  // Dual antenna mode, dual memory banks
    if (num_presets[ant] == 0)
    #else                               // Single, Dual or Triple antenna mode with automatic changeover
    if ((num_presets[0] + num_presets[1] + num_presets[2]) == 0)
    #endif  
    {
      virt_lcd_clear();
      virt_lcd_setCursor(0,1); 
      virt_lcd_print("Memory is Empty!!");
      Menu_exit_timer = 30;             // Show on LCD for 3 seconds
      menu_level = 0;                   // We're done with this menu level
      flag.config_menu = false;         // We're done
      flag.menu_lcd_upd = false;        // Make ready for next time     
    }
    #if ANT_CHG_2BANKS                  // Dual antenna mode, dual memory banks
    else if (current_selection < num_presets[ant])
    #else                               // Single, Dual or Triple antenna mode with automatic changeover
    else if (current_selection < (num_presets[0] + num_presets[1] + num_presets[2]))
    #endif  
    {
      #if ANT_CHG_2BANKS                // Dual antenna mode, dual memory banks
      // Store Current value in the selected preset
      preset[ant*MAX_PRESETS/2 + current_selection].Frq = running[ant].Frq;
      // Set following positions
      running[ant].Pos = running[ant].Pos + delta_Pos[ant];
      stepper_track[ant] = running[ant].Pos;
      delta_Pos[ant] = 0;
      preset[ant*MAX_PRESETS/2 + current_selection].Pos = running[ant].Pos;
      #else                             // Single or Dual antenna mode with automatic changeover
      // Store Current value in the selected preset
      preset[current_selection].Frq = running[ant].Frq;
      // Set following positions
      running[ant].Pos = running[ant].Pos + delta_Pos[ant];
      stepper_track[ant] = running[ant].Pos;
      delta_Pos[ant] = 0;
      preset[current_selection].Pos = running[ant].Pos;
      #endif  

      // Sort all presets in an ascending order, but with empty poitions on top
      preset_sort();
      // Recalculate total number of active presets, if any, determine outer bounds and find out where we are
      determine_preset_bounds();        // Determine outer bounds
      normalize_stepper_pos();          // Normalize (first to 1000000) and store all position memories
      determine_preset_bounds();        // Recalculate outer bounds after normalization
      determine_active_range(running[ant].Frq); // Determine where we are
     
      virt_lcd_clear();
      virt_lcd_setCursor(0,3); 
      virt_lcd_print("New value stored");
      Menu_exit_timer = 30;             // Show on LCD for 3 seconds
      menu_level = 0;                   // We're done with this menu level
      flag.config_menu = false;         // We're done
      flag.menu_lcd_upd = false;        // Make ready for next time
    }
    else
    {
      virt_lcd_clear();
      virt_lcd_setCursor(0,3); 
      virt_lcd_print("Return from Menu");
      Menu_exit_timer = 20;             // Show on LCD for 2 seconds
      menu_level = 0;                   // We're done with this menu level
      flag.config_menu = false;         // We're done
      flag.menu_lcd_upd = false;        // Make ready for next time
    }
  }
}

//--------------------------------------------------------------------
// Delete Position
//--------------------------------------------------------------------
void delete_pos_menu(void)
{
  static int8_t	 current_selection;	    // Keep track of current menu selection
  
  #if ANT_CHG_2BANKS                    // Dual antenna mode, dual memory banks
  if (num_presets[ant] == 0)
  #else                                 // Single, Dual or Triple antenna mode with automatic changeover
  if ((num_presets[0] + num_presets[1] + num_presets[2]) == 0)
  #endif  
  {
    flag.short_push = true;             // Nothing has been stored - nothing to do.
    flag.menu_lcd_upd = true;           // No LCD servicing required
  }
 
  // We remember last selection as modified by encoder, even if exit and re-entry
  if (Enc.read()/ENC_MENURESDIVIDE != 0)
  {
    if (Enc.read()/ENC_MENURESDIVIDE > 0)
    {
      current_selection++;
    }
    else if (Enc.read()/ENC_MENURESDIVIDE < 0)
    {
      current_selection--;
    }
    // Reset data from Encoder
    Enc.write(0);

    // Indicate that an LCD update is needed
    flag.menu_lcd_upd = false;
  }

  if (!flag.menu_lcd_upd)               // Need to update LCD
  {
    flag.menu_lcd_upd = true;           // We have serviced LCD

    // Keep Encoder Selection Within Bounds of the Menu Size
    #if ANT_CHG_2BANKS                  // Dual antenna mode, dual memory banks
    uint8_t menu_size = num_presets[ant] + 1;
    #else                               // Single, Dual or Triple antenna mode with automatic changeover
    uint8_t menu_size = num_presets[0] + num_presets[1] + num_presets[2] + 1;
    #endif  
    while(current_selection >= menu_size)
      current_selection -= menu_size;
    while(current_selection < 0)
      current_selection += menu_size;

    virt_lcd_clear();
    
    // Print the Menu
    #if ANT_CHG_2BANKS                  // Dual antenna mode, dual memory banks
    if (current_selection < num_presets[ant])
    #else                               // Single, Dual or Triple antenna mode with automatic changeover
    if (current_selection < (num_presets[0] + num_presets[1] + num_presets[2]))
    #endif  
    {
      virt_lcd_print("Delete FRQ Preset");
      virt_lcd_setCursor(0,1); 
      sprintf(print_buf,"->%2u ",current_selection);
      virt_lcd_print(print_buf);
      #if ANT_CHG_2BANKS                // Dual antenna mode, dual memory banks
      display_frq(preset[ant*MAX_PRESETS/2 + current_selection].Frq);
      #else                             // Single or Dual antenna mode with automatic changeover
      display_frq(preset[current_selection].Frq);
      #endif  
      virt_lcd_print(" Hz");
      
      virt_lcd_setCursor(0,3); 
      virt_lcd_print("Rotate and select:");
    }    
    else
    {
      virt_lcd_print("No change");
      virt_lcd_setCursor(0,1); 
      virt_lcd_print("-> Exit");  
      virt_lcd_setCursor(0,3); 
      virt_lcd_print("Rotate and select:");
    }
  }

  if (flag.short_push)
  {
    flag.short_push = false;            // Clear pushbutton status

    // Nothing has been stored, lets get out of here
    #if ANT_CHG_2BANKS                  // Dual antenna mode, dual memory banks
    if (num_presets[ant] == 0)
    #else                               // Single, Dual or Triple antenna mode with automatic changeover
    if ((num_presets[0] + num_presets[1] + num_presets[2]) == 0)
    #endif  
    {
      virt_lcd_clear();
      virt_lcd_setCursor(0,1); 
      virt_lcd_print("Memory is Empty!!");
      Menu_exit_timer = 30;             // Show on LCD for 3 seconds
      menu_level = 0;                   // We're done with this menu level
      flag.config_menu = false;         // We're done
      flag.menu_lcd_upd = false;        // Make ready for next time     
    }
    #if ANT_CHG_2BANKS                  // Dual antenna mode, dual memory banks
    else if (current_selection < num_presets[ant])
    #else                               // Single, Dual or Triple antenna mode with automatic changeover
    else if (current_selection < (num_presets[0] + num_presets[1] + num_presets[2]))
    #endif  
    {
      // Store a Zero value in the selected preset
      #if ANT_CHG_2BANKS                // Dual antenna mode, dual memory banks
      preset[ant*MAX_PRESETS/2 + current_selection].Frq = 0;
      preset[ant*MAX_PRESETS/2 + current_selection].Pos = 1000000;
      #else                             // Single or Dual antenna mode with automatic changeover
      preset[current_selection].Frq = 0;
      preset[current_selection].Pos = 1000000;
      #endif  

      // Sort all presets in an ascending order, but with empty poitions on top
      preset_sort();
      // Recalculate total number of active presets, if any, determine outer bounds and find out where we are
      determine_preset_bounds();        // Determine outer bounds
      normalize_stepper_pos();          // Normalize (first to 1000000) and store all position memories
      determine_preset_bounds();        // Recalculate outer bounds after normalization
      determine_active_range(running[ant].Frq); // Determine where we are
     
      virt_lcd_clear();
      virt_lcd_setCursor(0,3); 
      virt_lcd_print("MemoryPreset Deleted");
      Menu_exit_timer = 30;             // Show on LCD for 3 seconds
      menu_level = 0;                   // We're done with this menu level
      flag.config_menu = false;         // We're done
      flag.menu_lcd_upd = false;        // Make ready for next time
    }
    else
    {
      virt_lcd_clear();
      virt_lcd_setCursor(0,3); 
      virt_lcd_print("Return from Menu");
      Menu_exit_timer = 20;             // Show on LCD for 2 seconds
      menu_level = 0;                   // We're done with this menu level
      flag.config_menu = false;         // We're done
      flag.menu_lcd_upd = false;        // Make ready for next time
    }
  }
}

//--------------------------------------------------------------------
// Stepper Rate selection Submenu
//--------------------------------------------------------------------
void stepper_rate_submenu(void)
{
  int8_t        current_selection;	// Keep track of current menu selection
  
  current_selection = step_rate;
  
  if (Enc.read()/ENC_MENURESDIVIDE != 0)
  {
    if (Enc.read()/ENC_MENURESDIVIDE > 0)
    {
      current_selection++;
    }
    else if (Enc.read()/ENC_MENURESDIVIDE < 0)
    {
      current_selection--;
    }
    // Reset data from Encoder
    Enc.write(0);

    // Indicate that an LCD update is needed
    flag.menu_lcd_upd = false;
  }

  if (!flag.menu_lcd_upd)               // Need to update LCD
  {
    flag.menu_lcd_upd = true;           // We have serviced LCD

    // Keep Encoder Selection Within Bounds of the Menu Size
    uint8_t menu_size = 15;             // Max value is 15 (for 1500 steps per second)
    while(current_selection >= menu_size)
      current_selection -= menu_size;
    while(current_selection < 1)        // Min value is 1 for (100 steps per second)
      current_selection += menu_size;
      
    // Update with currently selected value
    step_rate = current_selection;
    
    virt_lcd_clear();
    virt_lcd_print("Select Stepper Rate:");
    
    // Print the Rotary Encoder scroll Menu
    virt_lcd_setCursor(0,2);
    // Normally the Stepper Rate is displayed in Steps per Second.
    // Alternately it can be shown in RPM - #define in ML.h
    // Start by calculating max rate as a function of speed_up and microsteps
    int8_t rate_div = (3-microstep_resolution) - step_speedup;
    if (rate_div > 0) rate_div = pow(2,rate_div);
    else rate_div = 1;
    #if STEPPER_SPEED_IN_RPM            // Display Stepper Rate in RPM
    sprintf(print_buf,"->%4.0f RPM",(step_rate*6000.0*STEPPER_ANGLE)/(360.0*rate_div));
    #else                               // Display Stepper Rate in Steps/Second
    sprintf(print_buf,"->%4u Steps/Second",step_rate*100/rate_div);
    #endif
    virt_lcd_print(print_buf);  
  }
  
  // Enact selection
  if (flag.short_push)
  {
    virt_lcd_clear();
    virt_lcd_setCursor(0,1); 
    
    flag.short_push = false;            // Clear pushbutton status
    
    // Check if selected value is not same as the previous one stored in the
    // controller_settings.
    if (controller_settings.step_rate != step_rate)// New Value
    {
      virt_lcd_print("Value Stored");
      controller_settings.step_rate = step_rate;
      EEPROM_writeAnything(1,controller_settings);
    }
    else virt_lcd_print("Nothing Changed");
    
    Menu_exit_timer = 20;               // Show on LCD for 2 seconds
    menu_level = 0;                     // We're done with this menu level
    flag.config_menu = true;            // Go back to Config Menu
    flag.menu_lcd_upd = false;          // Make ready for next time
  }
}

//--------------------------------------------------------------------
// Stepper Speedup selection Submenu
//--------------------------------------------------------------------
void stepper_speedup_submenu(void)
{
  int8_t        current_selection;	// Keep track of current menu selection
  
  current_selection = step_speedup;
  
  if (Enc.read()/ENC_MENURESDIVIDE != 0)
  {
    if (Enc.read()/ENC_MENURESDIVIDE > 0)
    {
      current_selection++;
    }
    else if (Enc.read()/ENC_MENURESDIVIDE < 0)
    {
      current_selection--;
    }
    // Reset data from Encoder
    Enc.write(0);

    // Indicate that an LCD update is needed
    flag.menu_lcd_upd = false;
  }

  if (!flag.menu_lcd_upd)               // Need to update LCD
  {
    flag.menu_lcd_upd = true;           // We have serviced LCD

    // Keep Encoder Selection Within Bounds of the Menu Size
    uint8_t menu_size = stepper_speedup_submenu_size;
    while(current_selection >= menu_size)
      current_selection -= menu_size;
    while(current_selection < 0)
      current_selection += menu_size;

    // Update with currently selected value
    step_speedup = current_selection;
    
    virt_lcd_clear();
    virt_lcd_print("Select Stepper");
    virt_lcd_setCursor(0,1); 
    virt_lcd_print("Speed Up:");
    
    // Print the Rotary Encoder scroll Menu
    lcd_scroll_Menu((char**)stepper_speedup_submenu_items, menu_size, current_selection, 2, 0,1);
  }
  
  // Enact selection
  if (flag.short_push)
  {
    virt_lcd_clear();
    virt_lcd_setCursor(0,1); 
    
    flag.short_push = false;            // Clear pushbutton status
    
    // Check if selected value is not same as the previous one stored in the
    // controller_settings.
    if (controller_settings.step_speedup != current_selection)// New Value
    {
      virt_lcd_print("Value Stored");
      controller_settings.step_speedup = current_selection;
      EEPROM_writeAnything(1,controller_settings);
    }
    else virt_lcd_print("Nothing Changed");
    
    Menu_exit_timer = 20;               // Show on LCD for 2 seconds
    menu_level = 0;                     // We're done with this menu level
    flag.config_menu = true;            // Go back to Config Menu
    flag.menu_lcd_upd = false;          // Make ready for next time
  }
}

//--------------------------------------------------------------------
// Stepper Microstep Resolution selection Submenu
//--------------------------------------------------------------------
//void microstep_res_menu(void)
void stepper_microsteps_submenu(void)
{
  int8_t        current_selection;	// Keep track of current menu selection
  
  current_selection = microstep_resolution;
  
  if (Enc.read()/ENC_MENURESDIVIDE != 0)
  {
    if (Enc.read()/ENC_MENURESDIVIDE > 0)
    {
      current_selection++;
    }
    else if (Enc.read()/ENC_MENURESDIVIDE < 0)
    {
      current_selection--;
    }
    // Reset data from Encoder
    Enc.write(0);

    // Indicate that an LCD update is needed
    flag.menu_lcd_upd = false;
  }

  if (!flag.menu_lcd_upd)               // Need to update LCD
  {
    flag.menu_lcd_upd = true;           // We have serviced LCD

    // Keep Encoder Selection Within Bounds of the Menu Size
    uint8_t menu_size = stepper_microsteps_submenu_size;
    while(current_selection >= menu_size)
      current_selection -= menu_size;
    while(current_selection < 0)
      current_selection += menu_size;

    // Update with currently selected value
    microstep_resolution = current_selection;
    
    virt_lcd_clear();
    virt_lcd_print("Select Stepper");
    virt_lcd_setCursor(0,1); 
    virt_lcd_print("Resolution:");
    
    // Print the Rotary Encoder scroll Menu
    lcd_scroll_Menu((char**)stepper_microsteps_submenu_items, menu_size, current_selection, 2, 0,1);
  }
  
  // Enact selection
  if (flag.short_push)
  {
    virt_lcd_clear();
    virt_lcd_setCursor(0,1); 
    
    flag.short_push = false;            // Clear pushbutton status
    
    // Check if selected threshold is not same as previous Microstep resolution, as stored in the
    // controller_settings. 0 for full resolution of 8 microsteps,
    // 1 for 4 microsteps, 2 for 2 microsteps or 3 for no microsteps
    if (controller_settings.microsteps != current_selection)// New Value
    {
      virt_lcd_print("Value Stored");
      controller_settings.microsteps = current_selection;
      EEPROM_writeAnything(1,controller_settings);
    }
    else virt_lcd_print("Nothing Changed");
    
    Menu_exit_timer = 20;               // Show on LCD for 2 seconds
    menu_level = 0;                     // We're done with this menu level
    //flag.config_menu = false;         // We're done
    flag.config_menu = true;            // Go back to Config Menu
    flag.menu_lcd_upd = false;          // Make ready for next time
  }
}

//--------------------------------------------------------------------
// Stepper Settings Menu
//--------------------------------------------------------------------
void stepper_settings_menu(void)
{
  static int8_t   current_selection;	// Keep track of current menu selection
  
  if (Enc.read()/ENC_MENURESDIVIDE != 0)
  {
    if (Enc.read()/ENC_MENURESDIVIDE > 0)
    {
      current_selection++;
    }
    else if (Enc.read()/ENC_MENURESDIVIDE < 0)
    {
      current_selection--;
    }
    // Reset data from Encoder
    Enc.write(0);

    // Indicate that an LCD update is needed
    flag.menu_lcd_upd = false;
  }

  if (!flag.menu_lcd_upd)               // Need to update LCD
  {
    flag.menu_lcd_upd = true;           // We have serviced LCD

    // Keep Encoder Selection Within Bounds of the Menu Size
    uint8_t menu_size = stepper_settings_menu_size;
    while(current_selection >= menu_size)
      current_selection -= menu_size;
    while(current_selection < 0)
      current_selection += menu_size;
    
    virt_lcd_clear();
    virt_lcd_print("Stepper Settings");
    if (current_selection == 0)
    {
      virt_lcd_setCursor(0,1); virt_lcd_print("->"); 
      virt_lcd_setCursor(0,2); virt_lcd_print("  "); 
      virt_lcd_setCursor(0,3); virt_lcd_print("  ");
    }
    else if (current_selection == 1)
    {
      virt_lcd_setCursor(0,1); virt_lcd_print("  "); 
      virt_lcd_setCursor(0,2); virt_lcd_print("->"); 
      virt_lcd_setCursor(0,3); virt_lcd_print("  ");
    }
    else if (current_selection == 2)
    {
      virt_lcd_setCursor(0,1); virt_lcd_print("  "); 
      virt_lcd_setCursor(0,2); virt_lcd_print("  "); 
      virt_lcd_setCursor(0,3); virt_lcd_print("->");
    }
    if (current_selection < 3)
    {  
      virt_lcd_setCursor(2,1);
      // Normally the Stepper Rate is displayed in Steps per Second.
      // Alternately it can be shown in RPM - #define in ML.h
      int8_t rate_div = (3-microstep_resolution) - step_speedup;
      if (rate_div > 0) rate_div = pow(2,rate_div);
      else rate_div = 1;
      #if STEPPER_SPEED_IN_RPM            // Display Stepper Rate in RPM
      // Start by calculating max rate as a function of speed_up and microsteps
      sprintf(print_buf,"Max Rate:%4.0f RPM",(step_rate*6000.0*STEPPER_ANGLE)/(360.0*rate_div));
      #else                               // Display Stepper Rate in Steps/Second
      sprintf(print_buf,"Max Rate:%4u st/s", step_rate*100/rate_div);
      #endif
      virt_lcd_print(print_buf);
      virt_lcd_setCursor(2,2); 
      uint8_t tmp_step_speedup =  pow(2,step_speedup);
      sprintf(print_buf,"VariableRate: %ux", tmp_step_speedup); 
      virt_lcd_print(print_buf);
      virt_lcd_setCursor(2,3);
      uint8_t tmp_microsteps =  8/pow(2,microstep_resolution);
      if (tmp_microsteps == 1) tmp_microsteps = 0;
      sprintf(print_buf,"Microsteps  : %u",tmp_microsteps); 
      virt_lcd_print(print_buf);
    }
    else
    {
      virt_lcd_setCursor(0,1); virt_lcd_print("  "); 
      virt_lcd_setCursor(0,2); virt_lcd_print("-> Exit Stepper Menu"); 
      virt_lcd_setCursor(0,3); virt_lcd_print("  ");      
    }
  }
  
  // Enact selection
  if (flag.short_push)
  {
    virt_lcd_clear();
    virt_lcd_setCursor(0,1); 
    
    flag.short_push = false;            // Clear pushbutton status
    flag.menu_lcd_upd = false;          // Force LCD reprint    
    switch (current_selection)
    {
      case 0:
        menu_level = STEPPER_RATE_SUBMENU;
        break;

      case 1:
        menu_level = STEPPER_SPEEDUP_SUBMENU;
        break;

      case 2:
        menu_level = STEPPER_MICROSTEPS_SUBMENU;
        break;

      default:
        menu_level = 0;
    }
  }
}


//--------------------------------------------------------------------
// Stepper Backlash Adjust Menu
//--------------------------------------------------------------------
void backlash_select_menu(void)
{
  static int16_t   current_selection;

  // Backlash Angle, 0 - 3200 in steps of 100.
  current_selection = controller_settings.backlash_angle;

  // Selection modified by encoder.  We remember last selection, even if exit and re-entry
  if (Enc.read()/ENC_MENURESDIVIDE != 0)
  {
    if ((Enc.read()/ENC_MENURESDIVIDE > 0) && (current_selection < 80) ) // Angle max is 80 x 5 = 400
    {
      current_selection++;
    }
    if ((Enc.read()/ENC_MENURESDIVIDE < 0) && (current_selection > 0) )   // Angle min is 0 x 5
    {
      current_selection--;
    }
    // Reset data from Encoder
    Enc.write(0);

    controller_settings.backlash_angle = current_selection;
    
    // Indicate that an LCD update is needed
    flag.menu_lcd_upd = false;            // Keep track of LCD update requirements
  }

  // If LCD update is needed
  if (!flag.menu_lcd_upd)
  {
    flag.menu_lcd_upd = true;            // We have serviced LCD

    virt_lcd_clear();

    virt_lcd_setCursor(0,0);
    virt_lcd_print("Stepper Backlash:");
    
    virt_lcd_setCursor(0,1);
    virt_lcd_print("Turn and select->");
    sprintf(print_buf,"%3u",current_selection*5);       
    virt_lcd_print(print_buf);
    
    virt_lcd_setCursor(0,3);
    virt_lcd_print("Range 0 to 400 steps");
  }
    
  // Enact selection by saving in EEPROM
  if (flag.short_push)
  {
    flag.short_push = false;             // Clear pushbutton status

    virt_lcd_clear();
    virt_lcd_setCursor(1,1);
    EEPROM_readAnything(1,controller_settings);
    if (controller_settings.backlash_angle != current_selection)  // New Value
    {
      controller_settings.backlash_angle = current_selection;
      EEPROM_writeAnything(1,controller_settings);
      virt_lcd_print("Value Stored");
    }
    else virt_lcd_print("Nothing Changed");

    Menu_exit_timer = 30;                // Show on LCD for 3 seconds
    flag.config_menu = false;            // We're done
    menu_level = 0;                      // We are done with this menu level
    flag.menu_lcd_upd = false;           // Make ready for next time
  }
}


//--------------------------------------------------------------------
// Transceiver Profile Select Menu (select active profile, one of four)
//--------------------------------------------------------------------
void transceiver_profile_menu(void)
{
  static int8_t   current_selection;  // Keep track of current menu selection
  
  if (Enc.read()/ENC_MENURESDIVIDE != 0)
  {
    if (Enc.read()/ENC_MENURESDIVIDE > 0)
    {
      current_selection++;
    }
    else if (Enc.read()/ENC_MENURESDIVIDE < 0)
    {
      current_selection--;
    }
    // Reset data from Encoder
    Enc.write(0);

    // Indicate that an LCD update is needed
    flag.menu_lcd_upd = false;
  }

  if (!flag.menu_lcd_upd)               // Need to update LCD
  {
    flag.menu_lcd_upd = true;           // We have serviced LCD

    // Keep Encoder Selection Within Bounds of the Menu Size
    uint8_t menu_size = 5;              // Max 4 profiles - fifth pos for instructions
    while(current_selection >= menu_size)
      current_selection -= menu_size;
    while(current_selection < 0)
      current_selection += menu_size;
    
    virt_lcd_clear();
    if (current_selection == 4)         // Instructions & Exit w/o change
    {
      virt_lcd_setCursor(0,0); virt_lcd_print("Select one of four"); 
      virt_lcd_setCursor(0,1); virt_lcd_print("preset TRX Profiles."); 
      virt_lcd_setCursor(0,3); virt_lcd_print("Or push to exit"); 
    }
    else
    {
      virt_lcd_setCursor(0,current_selection);
      virt_lcd_print("->"); 

      for (uint8_t x = 0; x<4; x++)
      {
        virt_lcd_setCursor(2,x);
        virt_lcd_print(transceiver_select_menu_items[controller_settings.trx[ x ].radio]);         
      }
    }
  }
  
  // Enact selection
  if (flag.short_push)
  {
    virt_lcd_clear();
    virt_lcd_setCursor(0,1); 
    
    flag.short_push = false;            // Clear pushbutton status
    flag.menu_lcd_upd = false;          // Force LCD reprint

    // Check if selected value is not same as the previous one stored in the
    // controller_settings.
    if ((current_selection != 4) && (controller_settings.radioprofile != current_selection))// New Value
    {
      virt_lcd_print("TRX Profile Selected");

      controller_settings.radioprofile = current_selection;  // Update the controller_settings
      trx_profile_update();             // Update all settings relevant to the active profile
      EEPROM_writeAnything(1,controller_settings); // and write the current settings to EEPROM
    }
    else virt_lcd_print("Nothing Changed");
    
    Menu_exit_timer = 20;               // Show on LCD for 2 seconds
    menu_level = 0;                     // We're done with this menu level
    flag.config_menu = false;           // We're done
    //flag.config_menu = true;            // Go back to Config Menu
    flag.menu_lcd_upd = false;          // Make ready for next time
  }
}




//--------------------------------------------------------------------
// Transceiver Selection Menu
//--------------------------------------------------------------------
void transceiver_select_menu(void)
{
  int8_t        current_selection;	// Keep track of current menu selection
  
  current_selection = radio_selection;
  
  if (Enc.read()/ENC_MENURESDIVIDE != 0)
  {
    if (Enc.read()/ENC_MENURESDIVIDE > 0)
    {
      current_selection++;
    }
    else if (Enc.read()/ENC_MENURESDIVIDE < 0)
    {
      current_selection--;
    }
    // Reset data from Encoder
    Enc.write(0);

    // Indicate that an LCD update is needed
    flag.menu_lcd_upd = false;
  }

  if (!flag.menu_lcd_upd)               // Need to update LCD
  {
    flag.menu_lcd_upd = true;           // We have serviced LCD

    // Keep Encoder Selection Within Bounds of the Menu Size
    uint8_t menu_size = transceiver_select_menu_size;
    while(current_selection >= menu_size)
      current_selection -= menu_size;
    while(current_selection < 0)
      current_selection += menu_size;

    // Update with currently selected value
    radio_selection = current_selection;
    
    virt_lcd_clear();
    virt_lcd_print("Transceiver Type:");
    
    // Print the Rotary Encoder scroll Menu
    lcd_scroll_Menu((char**)transceiver_select_menu_items, menu_size, current_selection, 1, 0,3);
  }
  
  // Enact selection
  if (flag.short_push)
  {
    virt_lcd_clear();
    virt_lcd_setCursor(0,1); 
    
    flag.short_push = false;            // Clear pushbutton status
    
    // Check if selected value is not same as the previous one stored in the
    // controller_settings.
    if (controller_settings.trx[controller_settings.radioprofile].radio != current_selection)// New Value
    {
      virt_lcd_print("SerialSettingsStored");

      controller_settings.trx[controller_settings.radioprofile].radio = current_selection;  // Update the controller_settings
      EEPROM_writeAnything(1,controller_settings);
      
      trx_parameters_init(controller_settings.radioprofile);// Init serial port parameters for a new Radio
      trx_parameters_set(controller_settings.radioprofile); // and setup the serial port for TRX poll
    }
    else virt_lcd_print("Nothing Changed");
    
    Menu_exit_timer = 20;               // Show on LCD for 2 seconds
    menu_level = 0;                     // We're done with this menu level
    //flag.config_menu = false;           // We're done
    flag.config_menu = true;            // Go back to Config Menu
    flag.menu_lcd_upd = false;          // Make ready for next time
  }
}


//--------------------------------------------------------------------
// ICOM CI-V Address Selection Menu
//--------------------------------------------------------------------
void icom_civ_address_menu(void)
{
  static int16_t   current_selection;

  if (controller_settings.trx[controller_settings.radioprofile].radio > 1)     // Not ICOM selected
  {
    virt_lcd_clear();
    virt_lcd_setCursor(0,0);
    virt_lcd_print("Currently Selected");
    virt_lcd_setCursor(0,1);
    virt_lcd_print("Radio is not an ICOM");    
    virt_lcd_setCursor(0,3);
    virt_lcd_print("->");    
    virt_lcd_print((char*)transceiver_select_menu_items[controller_settings.trx[controller_settings.radioprofile].radio]);
    flag.short_push = false;             // Clear pushbutton status
    Menu_exit_timer = 30;                // Show on LCD for 3 seconds
    flag.config_menu = true;             // We're not done, just back off
    menu_level = 0;                      // We are done with this menu level
    flag.menu_lcd_upd = false;           // Make ready for next time
  }
  else
  {
    // ICOM Address can be any value between 0x01 and 0xef
    current_selection = controller_settings.trx[controller_settings.radioprofile].ICOM_address;
  
    // Selection modified by encoder.  We remember last selection, even if exit and re-entry
    if (Enc.read()/ENC_MENURESDIVIDE != 0)
    {
      if ((Enc.read()/ENC_MENURESDIVIDE > 0) && (current_selection < 0xef) )  // 0xef is the MAX value
      {
        current_selection++;
      }
      if ((Enc.read()/ENC_MENURESDIVIDE < 0) && (current_selection > 0x01) )  // 0x01 is the MIN value
      {
        current_selection--;
      }
      // Reset data from Encoder
      Enc.write(0);
  
      controller_settings.trx[controller_settings.radioprofile].ICOM_address = current_selection;
      
      // Indicate that an LCD update is needed
      flag.menu_lcd_upd = false;            // Keep track of LCD update requirements
    }
  
    // If LCD update is needed
    if (!flag.menu_lcd_upd)
    {
      flag.menu_lcd_upd = true;            // We have serviced LCD
  
      virt_lcd_clear();
  
      virt_lcd_setCursor(0,0);
      virt_lcd_print("ICOM CI-V Address:");
      
      virt_lcd_setCursor(0,1);
      virt_lcd_print("Turn and select->");
      sprintf(print_buf,"%02X$",current_selection);
      virt_lcd_print(print_buf);
   					
      virt_lcd_setCursor(0,3);
      virt_lcd_print("Range is 01$ to EF$");
    }
    	
    // Enact selection by saving in EEPROM
    if (flag.short_push)
    {
      flag.short_push = false;             // Clear pushbutton status
  
      virt_lcd_clear();
      virt_lcd_setCursor(1,1);
      EEPROM_readAnything(1,controller_settings);
      if (controller_settings.trx[controller_settings.radioprofile].ICOM_address != current_selection)  // New Value
      {
        controller_settings.trx[controller_settings.radioprofile].ICOM_address = current_selection;
        EEPROM_writeAnything(1,controller_settings);
        virt_lcd_print("Value Stored");
      }
      else virt_lcd_print("Nothing Changed");
    
      Menu_exit_timer = 30;                // Show on LCD for 3 seconds
      flag.config_menu = true;             // Go back to Config Menu
      menu_level = 0;                      // We are done with this menu level
      flag.menu_lcd_upd = false;           // Make ready for next time
    }
  }
}


//--------------------------------------------------------------------
// RS232 Signals Mode selection Menu
//--------------------------------------------------------------------
void rs232_sig_menu(void)
{
  int8_t        current_selection;	// Keep track of current menu selection
  
  current_selection = rs232_signals;
  
  if (Enc.read()/ENC_MENURESDIVIDE != 0)
  {
    if (Enc.read()/ENC_MENURESDIVIDE > 0)
    {
      current_selection++;
    }
    else if (Enc.read()/ENC_MENURESDIVIDE < 0)
    {
      current_selection--;
    }
    // Reset data from Encoder
    Enc.write(0);

    // Indicate that an LCD update is needed
    flag.menu_lcd_upd = false;
  }

  if (!flag.menu_lcd_upd)               // Need to update LCD
  {
    flag.menu_lcd_upd = true;           // We have serviced LCD

    // Keep Encoder Selection Within Bounds of the Menu Size
    uint8_t menu_size = rs232_sig_menu_size;
    while(current_selection >= menu_size)
      current_selection -= menu_size;
    while(current_selection < 0)
      current_selection += menu_size;

    // Update with currently selected value
    rs232_signals = current_selection;
    
    virt_lcd_clear();
    virt_lcd_print("Serial Port Signals:");
    
    // Print the Rotary Encoder scroll Menu
    lcd_scroll_Menu((char**)rs232_sig_menu_items, menu_size, current_selection, 1, 0,3);
  }
  
  // Enact selection
  if (flag.short_push)
  {
    virt_lcd_clear();
    virt_lcd_setCursor(0,1); 
    
    flag.short_push = false;            // Clear pushbutton status
    
    // Check if selected RS232 signals polaritythreshold is not same as previously selected
    // 0 for TTL Signals Mode, 1 for RS232 Signals Mode
    if ((controller_settings.trx[controller_settings.radioprofile].sig_mode 
         + (controller_settings.trx[controller_settings.radioprofile].passthrough*2))
         != current_selection)// New Value
    {
      virt_lcd_print("SerialSettingsStored");
 
      // Update the controller_settings      
      controller_settings.trx[controller_settings.radioprofile].sig_mode = current_selection & 0x01;
      controller_settings.trx[controller_settings.radioprofile].passthrough = (current_selection/2) & 0x01;

      EEPROM_writeAnything(1,controller_settings);
      trx_parameters_set(controller_settings.radioprofile);             // Enact the serial port setup change
    }
    else virt_lcd_print("Nothing Changed");
    
    Menu_exit_timer = 20;               // Show on LCD for 2 seconds
    menu_level = 0;                     // We're done with this menu level
    //flag.config_menu = false;         // We're done
    flag.config_menu = true;            // Go back to Config Menu
    flag.menu_lcd_upd = false;          // Make ready for next time
  }
}

//--------------------------------------------------------------------
// RS232 Data Rate selection Menu
//--------------------------------------------------------------------
void rs232_rate_menu(void)
{
  int8_t  current_selection;            // Keep track of current menu selection
  
  current_selection = rs232_rate;
  
  if (Enc.read()/ENC_MENURESDIVIDE != 0)
  {
    if (Enc.read()/ENC_MENURESDIVIDE > 0)
    {
      current_selection++;
    }
    else if (Enc.read()/ENC_MENURESDIVIDE < 0)
    {
      current_selection--;
    }
    // Reset data from Encoder
    Enc.write(0);

    // Indicate that an LCD update is needed
    flag.menu_lcd_upd = false;
  }

  if (!flag.menu_lcd_upd)               // Need to update LCD
  {
    flag.menu_lcd_upd = true;           // We have serviced LCD

    // Keep Encoder Selection Within Bounds of the Menu Size
    uint8_t menu_size = rs232_rate_menu_size;
    while(current_selection >= menu_size)
    current_selection -= menu_size;
    while(current_selection < 0)
      current_selection += menu_size;

    // Update with currently selected value
    rs232_rate = current_selection;
    
    virt_lcd_clear();
    virt_lcd_print("Serial Data Rate:");
    
    // Print the Rotary Encoder scroll Menu
    lcd_scroll_Menu((char**)rs232_rate_menu_items, menu_size, current_selection, 2, 0,1);
  }
  
  // Enact selection
  if (flag.short_push)
  {
    virt_lcd_clear();
    virt_lcd_setCursor(0,1); 
    
    flag.short_push = false;            // Clear pushbutton status
    
    // Check if selected data rate is not same as the one stored in the Controller Settings
    // 0 = 1200 b/s, 1 = 2400 b/s, 2 = 4800 b/s, 3 = 9600 b/s,
    // 4 = 19200 b/s, 5 = 38400 b/s -- in other words:
    // Rate = 1200 * (1 << controller_settings.rs232rate)
    // or if 6 thn 57600 and if 7 then 115200
    if (controller_settings.trx[controller_settings.radioprofile].rs232rate != current_selection)// New Value
    {
      virt_lcd_print("SerialSettingsStored");

      controller_settings.trx[controller_settings.radioprofile].rs232rate = current_selection;  // Update the controller_settings
      EEPROM_writeAnything(1,controller_settings);
      trx_parameters_set(controller_settings.radioprofile); // Enact the serial port setup change
    }
    else virt_lcd_print("Nothing Changed");
    
    Menu_exit_timer = 20;               // Show on LCD for 2 seconds
    menu_level = 0;                     // We're done with this menu level
    //flag.config_menu = false;           // We're done
    flag.config_menu = true;            // Go back to Config Menu
    flag.menu_lcd_upd = false;          // Make ready for next time
  }
}


//--------------------------------------------------------------------
// TX Tune Power Level Adjust Menu
//--------------------------------------------------------------------
void tx_tunepwr_menu(void)
{
  static int16_t   current_selection;

  if (tx_tune_pwr == 255)                // Power Leve is not adjustable for the selected radio
  {
    virt_lcd_clear();
    virt_lcd_setCursor(0,0);
    virt_lcd_print("Not supported by");
    virt_lcd_setCursor(0,1);
    virt_lcd_print("the selected Radio");    
    virt_lcd_setCursor(0,3);
    virt_lcd_print("->");    
    virt_lcd_print((char*)transceiver_select_menu_items[controller_settings.trx[controller_settings.radioprofile].radio]);
    flag.short_push = false;             // Clear pushbutton status
    Menu_exit_timer = 30;                // Show on LCD for 3 seconds
    flag.config_menu = true;             // We're not done, just back off
    menu_level = 0;                      // We are done with this menu level
    flag.menu_lcd_upd = false;           // Make ready for next time
  }
  else
  {
    // Power Level can be any value between 0 and 254
    current_selection = tx_tune_pwr;
  
    // Selection modified by encoder.  We remember last selection, even if exit and re-entry
    if (Enc.read()/ENC_MENURESDIVIDE != 0)
    {
      if ((Enc.read()/ENC_MENURESDIVIDE > 0) && (current_selection < 254) )  // 254 is the MAX value
      {
        current_selection++;
      }
      if ((Enc.read()/ENC_MENURESDIVIDE < 0) && (current_selection > 0) )    // 0 is the MIN value
      {
        current_selection--;
      }
      // Reset data from Encoder
      Enc.write(0);
  
      tx_tune_pwr = current_selection;
      
      // Indicate that an LCD update is needed
      flag.menu_lcd_upd = false;            // Keep track of LCD update requirements
    }
  
    // If LCD update is needed
    if (!flag.menu_lcd_upd)
    {
      flag.menu_lcd_upd = true;            // We have serviced LCD
  
      virt_lcd_clear();
  
      virt_lcd_setCursor(0,0);
      virt_lcd_print("TX Tune Power Level:");
      
      virt_lcd_setCursor(0,1);
      virt_lcd_print("Turn and select->");
      sprintf(print_buf,"%03u$",current_selection);
      virt_lcd_print(print_buf);
             
      virt_lcd_setCursor(0,3);
      virt_lcd_print("Range is 0 to 254");
    }
      
    // Enact selection by saving in EEPROM
    if (flag.short_push)
    {
      flag.short_push = false;             // Clear pushbutton status
  
      virt_lcd_clear();
      virt_lcd_setCursor(1,1);
      EEPROM_readAnything(1,controller_settings);
      if (controller_settings.trx[controller_settings.radioprofile].tx_pwrlevel != current_selection)  // New Value
      {
        controller_settings.trx[controller_settings.radioprofile].tx_pwrlevel = current_selection;
        EEPROM_writeAnything(1,controller_settings);
        virt_lcd_print("Value Stored");
      }
      else virt_lcd_print("Nothing Changed");
    
      Menu_exit_timer = 30;                // Show on LCD for 3 seconds
      flag.config_menu = true;             // Go back to Config Menu
      menu_level = 0;                      // We are done with this menu level
      flag.menu_lcd_upd = false;           // Make ready for next time
    }
  }
}



//--------------------------------------------------------------------
// Reset Memoriews to all default values
//--------------------------------------------------------------------
void clear_memories_menu(void)
{
  static int8_t   current_selection;

  if (Enc.read()/ENC_MENURESDIVIDE != 0)
  {
    if (Enc.read()/ENC_MENURESDIVIDE > 0)
    {
      current_selection++;
    }
    else if (Enc.read()/ENC_MENURESDIVIDE < 0)
    {
      current_selection--;
    }
    // Reset data from Encoder
    Enc.write(0);

    // Indicate that an LCD update is needed
    flag.menu_lcd_upd = false;
  }

  // If LCD update is needed
  if (!flag.menu_lcd_upd)
  {
    flag.menu_lcd_upd = true;           // We have serviced LCD

    // Keep Encoder Selection Within Bounds of the Menu Size
    uint8_t menu_size = clear_memories_menu_size;
    while(current_selection >= menu_size)
      current_selection -= menu_size;
    while(current_selection < 0)
      current_selection += menu_size;

    virt_lcd_clear();
    virt_lcd_print("All to default?");

      // Print the Rotary Encoder scroll Menu
    lcd_scroll_Menu((char**)clear_memories_menu_items, menu_size, current_selection, 1, 0,2);
  }

  // Enact selection
  if (flag.short_push)
  {
    flag.short_push = false;            // Clear pushbutton status

    switch (current_selection)
    {
      case 0:
        virt_lcd_clear();
        virt_lcd_setCursor(1,1); 				
        virt_lcd_print("Nothing Changed");
        Menu_exit_timer = 30;           // Show on LCD for 3 seconds
        //flag.config_menu = true;      // We're NOT done, just backing off to Config Menu
        flag.config_menu = false;       // We're done
        menu_level = 0;                 // We are done with this menu level
        flag.menu_lcd_upd = false;      // Make ready for next time
        break;
      case 1: // Clear Memories
        // Force an EEPROM update upon reboot by storing 0xfe in the first address
        EEPROM.write(0,0xfe);
        virt_lcd_clear();
        virt_lcd_setCursor(1,1);				
        virt_lcd_print("Clear all Memories");
        SOFT_RESET();
        //while (1);                   // Bye bye, Death by Watchdog
      default:
        virt_lcd_clear();
        virt_lcd_setCursor(1,1);				
        virt_lcd_print("Nothing Changed");
        Menu_exit_timer = 30;          // Show on LCD for 3 seconds
        flag.config_menu = false;      // We're done
        menu_level = 0;                // We are done with this menu level
        flag.menu_lcd_upd = false;     // Make ready for next time
        break;
    }
  }
}


#if PSWR_AUTOTUNE
//--------------------------------------------------------------------
// SWR Tune Threshold Set Menu
//--------------------------------------------------------------------
void swr_threshold_menu(void)
{
  static int8_t   current_selection;

  // SWR tune threshold, 0 - 31 for SWR of 1.0 - 4.1
  current_selection = controller_settings.swr_ok + 10;

  // Selection modified by encoder.  We remember last selection, even if exit and re-entry
  if (Enc.read()/ENC_MENURESDIVIDE != 0)
  {
    if ((Enc.read()/ENC_MENURESDIVIDE > 0) && (current_selection < 41) )  // SWR 4.1:1 is MAX value
    {
      current_selection++;
    }
    if ((Enc.read()/ENC_MENURESDIVIDE < 0) && (current_selection > 10) )  // SWR of 1.0:1 is MIN value
    {
      current_selection--;
    }
    // Reset data from Encoder
    Enc.write(0);

    controller_settings.swr_ok = current_selection - 10;
    
    // Indicate that an LCD update is needed
    flag.menu_lcd_upd = false;            // Keep track of LCD update requirements
  }

  // If LCD update is needed
  if (!flag.menu_lcd_upd)
  {
    flag.menu_lcd_upd = true;            // We have serviced LCD

    virt_lcd_clear();

    virt_lcd_setCursor(0,0);
    virt_lcd_print("SWR Tune Threshold:");
    virt_lcd_setCursor(0,1);
    virt_lcd_print("Adjust->   ");

    sprintf(print_buf,"%1u.%01u",current_selection/10, current_selection%10);
    virt_lcd_print(print_buf);
 					
    virt_lcd_setCursor(0,2);
    virt_lcd_print("Range is 1.0 to 4.1");
  }
  	
  // Enact selection by saving in EEPROM
  if (flag.short_push)
  {
    flag.short_push = false;             // Clear pushbutton status

    virt_lcd_clear();
    virt_lcd_setCursor(1,1);
    EEPROM_readAnything(1,controller_settings);
    if (controller_settings.swr_ok != current_selection - 10)  // New Value
    {
      controller_settings.swr_ok = current_selection - 10;
      EEPROM_writeAnything(1,controller_settings);
      virt_lcd_print("Value Stored");
    }
    else virt_lcd_print("Nothing Changed");

    Menu_exit_timer = 30;                // Show on LCD for 3 seconds
    flag.config_menu = false;            // We're done
    menu_level = 0;                      // We are done with this menu level
    flag.menu_lcd_upd = false;           // Make ready for next time
  }
}


//--------------------------------------------------------------------
// Scale Range Setup Submenu functions
//--------------------------------------------------------------------
void scalerange_menu_level2(void)
{
  static int16_t	current_selection;      // Keep track of current LCD menu selection

  uint8_t scale_set;                            // Determine whether CAL_SET0, CAL_SET1 or CAL_SET2

  if (menu_level == SCALE_SET0_MENU) scale_set = 0;      // SCALE_SET0_MENU
  else if (menu_level == SCALE_SET1_MENU) scale_set = 1; // SCALE_SET1_MENU
  else scale_set = 2;                                    // SCALE_SET2_MENU

  // Get Current value
  current_selection = controller_settings.Scale[scale_set];

  // Selection modified by encoder.  We remember last selection, even if exit and re-entry
  if (Enc.read()/ENC_MENURESDIVIDE != 0)
  {
    if (Enc.read()/ENC_MENURESDIVIDE > 0)
    {
      current_selection++;
    }
    else if (Enc.read()/ENC_MENURESDIVIDE < 0)
    {
      current_selection--;
    }
    // Reset data from Encoder
    Enc.write(0);
    // Indicate that an LCD update is needed
    flag.menu_lcd_upd = false;
  }

  // If LCD update is needed
  if (!flag.menu_lcd_upd)
  {
    flag.menu_lcd_upd = true;                            // We are about to have serviced LCD

    // Keep Encoder Selection Within Scale Range Bounds
    int16_t max_range = controller_settings.Scale[0] * 10; // Highest permissible Scale Range for ranges 2 and 3,
    if (max_range > 99) max_range = 99;                    // never larger than 10x range 1 and never larger than 99
    int16_t min_range = controller_settings.Scale[0];      // Lowest permissible Scale Range for ranges 2 and 3,
                                                           // never smaller than range 1
    if (scale_set==0)
    {
      // Set bounds for Range 1 adjustments
      if(current_selection > 99) current_selection = 99; // Range 1 can take any value between 1 and 99
      if(current_selection < 1) current_selection = 1;			
    }
    if (scale_set>0)
    {
      // Set bounds for Range 2 and 3 adjustments
      if(current_selection > max_range) current_selection = max_range;
      if(current_selection < min_range) current_selection = min_range;
    }
    // Store Current value in running storage
    controller_settings.Scale[scale_set] = current_selection;

    //
    // Bounds dependencies check and adjust
    //
    // Ranges 2 and 3 cannot ever be larger than 9.9 times Range 1
    // Range 2 is equal to or larger than Range 1
    // Range 3 is equal to or larger than Range 2
    // If two Ranges are equal, then only two Scale Ranges in effect
    // If all three Ranges are equal, then only one Range is in effect
    // If Range 1 is being adjusted, Ranges 2 and 3 can be pushed up or down as a consequence
    // If Range 2 is being adjusted up, Range 3 can be pushed up
    // If Range 3 is being adjusted down, Range 2 can be pushed down
    if (controller_settings.Scale[1] >= controller_settings.Scale[0]*10) 
      controller_settings.Scale[1] = controller_settings.Scale[0]*10 - 1;
    if (controller_settings.Scale[2] >= controller_settings.Scale[0]*10) 
      controller_settings.Scale[2] = controller_settings.Scale[0]*10 - 1;
    // Ranges 2 and 3 cannot be smaller than Range 1			
    if (controller_settings.Scale[1] < controller_settings.Scale[0]) 
      controller_settings.Scale[1] = controller_settings.Scale[0];
    if (controller_settings.Scale[2] < controller_settings.Scale[0]) 
      controller_settings.Scale[2] = controller_settings.Scale[0];

    // Adjustment up of Range 2 can push Range 3 up
    if ((scale_set == 1) && (controller_settings.Scale[1] > controller_settings.Scale[2])) 
      controller_settings.Scale[2] = controller_settings.Scale[1];
    // Adjustment down of Range 3 can push Range 2 down:
    if ((scale_set == 2) && (controller_settings.Scale[2] < controller_settings.Scale[1])) 
      controller_settings.Scale[1] = controller_settings.Scale[2];

    virt_lcd_clear();

    // Populate the Display - including current values selected for scale ranges
    virt_lcd_setCursor(0,0);
    virt_lcd_print("Adjust, Push to Set:");

    virt_lcd_setCursor(6,1);
    sprintf(print_buf,"1st Range = %2u",controller_settings.Scale[0]);
    virt_lcd_print(print_buf);
    virt_lcd_setCursor(6,2);
    sprintf(print_buf,"2nd Range = %2u",controller_settings.Scale[1]);
    virt_lcd_print(print_buf);
    virt_lcd_setCursor(6,3);
    sprintf(print_buf,"3rd Range = %2u",controller_settings.Scale[2]);
    virt_lcd_print(print_buf);

    // Place "===>" in front of the "ScaleRange" currently being adjusted
    virt_lcd_setCursor(0,scale_set+1);
    virt_lcd_print("===>");
  }

  // Enact selection by saving in EEPROM
  if (flag.short_push)
  {
    flag.short_push = false;                             // Clear pushbutton status
    virt_lcd_clear();
    virt_lcd_setCursor(1,1);

    // Save modified value
    // There are so many adjustable values that it is simplest just to assume
    // a value has always been modified.  Save all 3
    EEPROM_writeAnything(1,controller_settings);
    virt_lcd_print("Value Stored");
    Menu_exit_timer = 30;                                // Show on LCD for 3 seconds
    flag.config_menu = true;                             // We're NOT done, just backing off
    menu_level = SCALERANGE_MENU;                        // We are done with this menu level
    flag.menu_lcd_upd = false;                           // Make ready for next time
  }
}



//--------------------------------------------------------------------
// Scale Range Menu functions
//--------------------------------------------------------------------
void scalerange_menu(void)
{
  static int8_t	current_selection;                       // Keep track of current LCD menu selection

  // Selection modified by encoder.  We remember last selection, even if exit and re-entry
  if (Enc.read()/ENC_MENURESDIVIDE != 0)
  {
    if (Enc.read()/ENC_MENURESDIVIDE > 0)
    {
      current_selection++;
    }
    else if (Enc.read()/ENC_MENURESDIVIDE < 0)
    {
      current_selection--;
    }
    // Reset data from Encoder
    Enc.write(0);
    // Indicate that an LCD update is needed
    flag.menu_lcd_upd = false;
  }

  // If LCD update is needed
  if (!flag.menu_lcd_upd)
  {
    flag.menu_lcd_upd = true;                            // We have serviced LCD

    // Keep Encoder Selection Within Bounds of the Menu Size
    uint8_t menu_size = 4;
    while(current_selection >= menu_size)
    current_selection -= menu_size;
    while(current_selection < 0)
    current_selection += menu_size;

    virt_lcd_clear();

    // Populate the Display - including current values selected for scale ranges
    virt_lcd_setCursor(0,0);
    if (current_selection<3)
      virt_lcd_print("Select Scale Range:");
    else
      virt_lcd_print("Turn or Push to Exit");

    virt_lcd_setCursor(6,1);
    sprintf(print_buf,"1st Range = %2u",controller_settings.Scale[0]);
    virt_lcd_print(print_buf);
    virt_lcd_setCursor(6,2);
    sprintf(print_buf,"2nd Range = %2u",controller_settings.Scale[1]);
    virt_lcd_print(print_buf);
    virt_lcd_setCursor(6,3);
    sprintf(print_buf,"3rd Range = %2u",controller_settings.Scale[2]);
    virt_lcd_print(print_buf);

    // Place "->" in front of the relevant "ScaleRange" to be selected with a push
    if (current_selection<3)
    {
      virt_lcd_setCursor(4,current_selection+1);
      virt_lcd_print("->");
    }
  }

  // Enact selection
  if (flag.short_push)
  {
    flag.short_push = false;                             // Clear pushbutton status

    switch (current_selection)
    {
      case 0:
        menu_level = SCALE_SET0_MENU;
        flag.menu_lcd_upd = false;                       // force LCD reprint
        break;
      case 1:
        menu_level = SCALE_SET1_MENU;
        flag.menu_lcd_upd = false;                       // force LCD reprint
        break;
      case 2:
        menu_level = SCALE_SET2_MENU;
        flag.menu_lcd_upd = false;                       // force LCD reprint
        break;
      default:
        virt_lcd_clear();
        virt_lcd_setCursor(0,1);
        virt_lcd_print("Done w. Scale Ranges");
        Menu_exit_timer = 30;                            // Show on LCD for 3 seconds
        flag.config_menu = false;                        // We're done
        menu_level = 0;                                  // We are done with this menu level
        flag.menu_lcd_upd = false;                       // Make ready for next time
    }
  }
}

#if AD8307_INSTALLED
//--------------------------------------------------------------------
// Calibrate Submenu functions
//--------------------------------------------------------------------
void calibrate_menu_level2(void)
{
  static int16_t  current_selection;                     // Keep track of current LCD menu selection

  uint8_t cal_set;                                       // Determine whether CAL_SET0, CAL_SET1 or CAL_SET2

  if (menu_level == CAL_SET2_MENU) cal_set = 1;	         // CAL_SET2_MENU
  else cal_set = 0;                                      // CAL_SET0_MENU or CAL_SET1_MENU

  // These defines to aid readability of code
  #define CAL_BAD	0                                // Input signal of insufficient quality for calibration
  #define CAL_FWD	1                                // Good input signal detected, forward direction
  #define CAL_REV	2                                // Good input signal detected, reverse direction (redundant)
  // Below variable can take one of the three above defined values, based on the
  // detected input "calibration" signal
  static uint8_t cal_sig_direction_quality;
  
  // Get Current value
  current_selection = controller_settings.cal_AD8307[cal_set].db10m;

  // Selection modified by encoder.  We remember last selection, even if exit and re-entry
  if (Enc.read()/ENC_MENURESDIVIDE != 0)
  {
    if (Enc.read()/ENC_MENURESDIVIDE > 0)
    {
      current_selection++;
    }
    else if (Enc.read()/ENC_MENURESDIVIDE < 0)
    {
      current_selection--;
    }
    // Reset data from Encoder
    Enc.write(0);
    // Indicate that an LCD update is needed
    flag.menu_lcd_upd = false;
  }

  // Determine direction and level of calibration signal input
  // Check forward direction and sufficient level
  if (((fwd - ref) > CAL_INP_QUALITY) &&
      (cal_sig_direction_quality != CAL_FWD))
  {
    cal_sig_direction_quality = CAL_FWD;
    flag.menu_lcd_upd = false;                           // Indicate that an LCD update is needed
  }
  // Check reverse direction and sufficient level
  else if (((ref - fwd) > CAL_INP_QUALITY) &&
           (cal_sig_direction_quality != CAL_REV))
  {
    cal_sig_direction_quality = CAL_REV;
    flag.menu_lcd_upd = false;                           // Indicate that an LCD update is needed
  }
  // Check insufficient level
  else if ((ABS((fwd - ref)) <= CAL_INP_QUALITY) &&
           (cal_sig_direction_quality != CAL_BAD))
  {
    cal_sig_direction_quality = CAL_BAD;
    flag.menu_lcd_upd = false;                           // Indicate that an LCD update is needed
  }

  // If LCD update is needed
  if(!flag.menu_lcd_upd)  
  {
    flag.menu_lcd_upd = true;                            // We have serviced LCD

    // Keep Encoder Selection Within Bounds of the Menu Size
    int16_t max_value = 530;                             // Highest permissible Calibration value in dBm * 10
    int16_t min_value = -100;                            // Lowest permissible Calibration value in dBm * 10
    if(current_selection > max_value) current_selection = max_value;
    if(current_selection < min_value) current_selection = min_value;

    // Store Current value in running storage
    controller_settings.cal_AD8307[cal_set].db10m = current_selection;

    virt_lcd_clear();
    virt_lcd_setCursor(0,0);	

    if (menu_level == CAL_SET0_MENU)                     // equals cal_set == 0
    {
      virt_lcd_print("Single Level Cal:");
    }
    else if (menu_level == CAL_SET1_MENU)                // equals cal_set == 1
    {
      virt_lcd_print("First Cal SetPoint:");
    }
    else if (menu_level == CAL_SET2_MENU)
    {
      virt_lcd_print("Second Cal SetPoint:");
    }

    virt_lcd_setCursor(0,1);
    virt_lcd_print("Adjust (dBm)->");
    // Format and print current value
    int16_t val_sub = current_selection;
    int16_t val = val_sub / 10;
    val_sub = val_sub % 10;
    if (current_selection < 0)
    {
      val*=-1;
      val_sub*=-1;
      sprintf(print_buf," -%1u.%01u",val, val_sub);
    }
    else
    {
      sprintf(print_buf," %2u.%01u",val, val_sub);
    }
    virt_lcd_print(print_buf);

    if (cal_sig_direction_quality == CAL_FWD)
    {
      virt_lcd_setCursor(0,2);
      virt_lcd_print(">Push to set<");
      virt_lcd_setCursor(0,3);
      virt_lcd_print("Signal detected");
    }
    else if (cal_sig_direction_quality == CAL_REV)
    {
      virt_lcd_setCursor(0,2);
      virt_lcd_print(">Push to set<");
      virt_lcd_setCursor(0,3);
      virt_lcd_print("Reverse detected");
    }
    else                                                 // cal_sig_direction_quality == CAL_BAD
    {
      virt_lcd_setCursor(0,2);
      virt_lcd_print(">Push to exit<");
      virt_lcd_setCursor(0,3);
      virt_lcd_print("Poor signal quality");
    }
  }

  // Enact selection by saving in EEPROM
  if (flag.short_push)
  {
    flag.short_push = false;                             // Clear pushbutton status
    virt_lcd_clear();
    virt_lcd_setCursor(1,1);

    uint16_t thirty_dB;                                  // Used for single shot calibration
    #if WIRE_ENABLED
    if (ad7991_addr) thirty_dB = 1165;                   // If AD7991 was detected during init
    else 
    #endif
    thirty_dB = 931;
        
    // Save modified value
    // If forward direction, then we calibrate for both, using the measured value for
    // in the forward direction only
    if (cal_sig_direction_quality == CAL_FWD)
    {
      if (menu_level == CAL_SET0_MENU)
      {
        controller_settings.cal_AD8307[0].Fwd = fwd;
        controller_settings.cal_AD8307[0].Rev = fwd;
        // Set second calibration point at 30 dB less, assuming 25mV per dB
        controller_settings.cal_AD8307[1].db10m = controller_settings.cal_AD8307[0].db10m - 300;
        controller_settings.cal_AD8307[1].Fwd = controller_settings.cal_AD8307[0].Fwd - thirty_dB;
        controller_settings.cal_AD8307[1].Rev = controller_settings.cal_AD8307[0].Fwd - thirty_dB;
      }
      else
      {
        controller_settings.cal_AD8307[cal_set].Fwd = fwd;
        controller_settings.cal_AD8307[cal_set].Rev = fwd;
      }
      EEPROM_writeAnything(1,controller_settings);
      virt_lcd_print("Value Stored");
    }
    // If reverse, then we calibrate for reverse direction only
    else if (cal_sig_direction_quality == CAL_REV)
    {
      if (menu_level == CAL_SET0_MENU)
      {
        controller_settings.cal_AD8307[0].Rev = ref;
        // Set second calibration point at 30 dB less, assuming 25mV per dB
        controller_settings.cal_AD8307[1].Rev = controller_settings.cal_AD8307[0].Fwd - thirty_dB;
      }
      else
      {
        controller_settings.cal_AD8307[cal_set].Rev = ref;
      }
      EEPROM_writeAnything(1,controller_settings);
      virt_lcd_print("Value Stored");
    }
    else                                                 // cal_sig_direction_quality == CAL_BAD
    {
      virt_lcd_print("Nothing changed");
    }

    Menu_exit_timer = 30;                                // Show on LCD for 3 seconds
    flag.config_menu = true;                             // We're NOT done, just backing off
    menu_level = CAL_MENU;                               // We are done with this menu level
    flag.menu_lcd_upd = false;                           // Make ready for next time
  }
}


//--------------------------------------------------------------------
// Calibrate Menu functions
//--------------------------------------------------------------------
void calibrate_menu(void)
{
  static int8_t	current_selection;                       // Keep track of current LCD menu selection

  // Selection modified by encoder.  We remember last selection, even if exit and re-entry
  if (Enc.read()/ENC_MENURESDIVIDE != 0)
  {
    if (Enc.read()/ENC_MENURESDIVIDE > 0)
    {
      current_selection++;
    }
    else if (Enc.read()/ENC_MENURESDIVIDE < 0)
    {
      current_selection--;
    }
    // Reset data from Encoder
    Enc.write(0);
    // Indicate that an LCD update is needed
    flag.menu_lcd_upd = false;
  }

  // If LCD update is needed
  if (!flag.menu_lcd_upd)
  {
    flag.menu_lcd_upd = true;                            // We have serviced LCD

    // Keep Encoder Selection Within Bounds of the Menu Size
    uint8_t menu_size = calibrate_menu_size;
    while(current_selection >= menu_size)
      current_selection -= menu_size;
    while(current_selection < 0)
      current_selection += menu_size;

    virt_lcd_clear();

    // Print the Rotary Encoder scroll Menu
    lcd_scroll_Menu((char**)calibrate_menu_items, menu_size, current_selection,1, 0,1);

    switch (current_selection)
    {
      case 0:
        virt_lcd_setCursor(0,2);
        virt_lcd_print("Calibrate using one");
        virt_lcd_setCursor(6,3);
        virt_lcd_print("accurate level");
        break;
      case 1:
        virt_lcd_setCursor(0,2);
        virt_lcd_print("Set higher of two");
        virt_lcd_setCursor(5,3);
        virt_lcd_print("accurate levels");
        break;
      case 2:
        virt_lcd_setCursor(0,2);
        virt_lcd_print("Set lower of two");
        virt_lcd_setCursor(5,3);
        virt_lcd_print("accurate levels");
        break;
    }

    // Indicate Current value stored under the currently selected GainPreset
    // The "stored" value indication changes according to which GainPreset is currently selected.
    virt_lcd_setCursor(0,0);				
    virt_lcd_print("Calibrate");
    if (current_selection <= 2)
    {
      int16_t value=0;

      switch (current_selection)
      {
        case 0:
        case 1:
          value = controller_settings.cal_AD8307[0].db10m;
          break;
        case 2:
          value = controller_settings.cal_AD8307[1].db10m;
          break;
      }
      int16_t val_sub = value;
      int16_t val = val_sub / 10;
      val_sub = val_sub % 10;

      // Print value of currently indicated reference
      virt_lcd_setCursor(16,0);
      if (value < 0)
      {
        val*=-1;
        val_sub*=-1;
        sprintf(print_buf,"-%1u.%01u",val, val_sub);
      }
      else
      {
        sprintf(print_buf,"%2u.%01u",val, val_sub);
      }
      virt_lcd_print(print_buf);
    }
    else
    {
      virt_lcd_setCursor(16,0);
      virt_lcd_print(" --");
    }
  }

  // Enact selection
  if (flag.short_push)
  {
    flag.short_push = false;                             // Clear pushbutton status

    switch (current_selection)
    {
      case 0:
        menu_level = CAL_SET0_MENU;
        flag.menu_lcd_upd = false;                       // force LCD reprint
        break;
      case 1:
        menu_level = CAL_SET1_MENU;
        flag.menu_lcd_upd = false;                       // force LCD reprint
        break;
      case 2:
        menu_level = CAL_SET2_MENU;
        flag.menu_lcd_upd = false;                       // force LCD reprint
        break;
      case 3:
        virt_lcd_clear();
        virt_lcd_setCursor(1,1);				
        virt_lcd_print("Done w. Cal");
        Menu_exit_timer = 30;                            // Show on LCD for 3 seconds
        flag.config_menu = true;                         // We're NOT done, just backing off
        menu_level = 0;                                  // We are done with this menu level
        flag.menu_lcd_upd = false;                       // Make ready for next time
        break;
      default:
        virt_lcd_clear();
        virt_lcd_setCursor(1,1);				
        virt_lcd_print("Done w. Cal");
        Menu_exit_timer = 30;                            // Show on LCD for 3 seconds
        flag.config_menu = false;                        // We're done
        menu_level = 0;                                  // We are done with this menu level
        flag.menu_lcd_upd = false;                       // Make ready for next time
        break;
    }
  }
}
#else
//--------------------------------------------------------------------
// Calibrate Menu functions
//--------------------------------------------------------------------
void calibrate_menu(void)
{
  static uint8_t   current_selection;

  // We want LCD update every time - to show Power measurement
  flag.menu_lcd_upd = false;
 
  // Calibration multiplier for diode detector type Power/SWR meter, 100 = 1.0
  current_selection = controller_settings.meter_cal;

  // Selection modified by encoder.  We remember last selection, even if exit and re-entry
  if (Enc.read()/ENC_MENURESDIVIDE != 0)
  {
    if ((Enc.read()/ENC_MENURESDIVIDE > 0) && (current_selection < 250) )
    {
      current_selection++;
    }
    if ((Enc.read()/ENC_MENURESDIVIDE < 0) && (current_selection > 10) )
    {
      current_selection--;
    }
    // Reset data from Encoder
    Enc.write(0);

    controller_settings.meter_cal = current_selection;   
  }

  virt_lcd_clear();

  virt_lcd_setCursor(0,0);
  virt_lcd_print("Meter Calibrate:");
  virt_lcd_setCursor(0,1);
  virt_lcd_print("Adjust->   ");

  sprintf(print_buf,"%1u.%02u",current_selection/100, current_selection%100);
  virt_lcd_print(print_buf);
 					
  virt_lcd_setCursor(0,2);
  virt_lcd_print("Range is 0.10 - 2.50");
  
  //measure_power_and_swr();
  virt_lcd_setCursor(0,3);
  virt_lcd_print("MeasuredPower:");
  print_p_mw(power_mw);
  virt_lcd_print(print_buf);
  	
  // Enact selection by saving in EEPROM
  if (flag.short_push)
  {
    flag.short_push = false;             // Clear pushbutton status

    virt_lcd_clear();
    virt_lcd_setCursor(1,1);
    EEPROM_readAnything(1,controller_settings);
    if (controller_settings.meter_cal != current_selection)  // New Value
    {
      controller_settings.meter_cal = current_selection;
      EEPROM_writeAnything(1,controller_settings);
      virt_lcd_print("Value Stored");
    }
    else virt_lcd_print("Nothing Changed");
    
    Menu_exit_timer = 30;                // Show on LCD for 3 seconds
    flag.config_menu = false;            // We're done
    menu_level = 0;                      // We are done with this menu level
  }
}
#endif


//--------------------------------------------------------------------
// Peak Envelope Power (PEP) period selection Menu
//--------------------------------------------------------------------
void pep_menu(void)
{
  int8_t        current_selection;
  uint16_t      temp;

  // Get Current value
  if (controller_settings.PEP_period == 500) current_selection = 1;        // 2.5 seconds
  else if (controller_settings.PEP_period == 1000) current_selection = 2;  // 5 seconds
  else current_selection = 0;               // Any other value, other than 1s, is invalid

  // Selection modified by encoder.  We remember last selection, even if exit and re-entry
  if (Enc.read()/ENC_MENURESDIVIDE != 0)
  {
    if (Enc.read()/ENC_MENURESDIVIDE > 0)
    {
      current_selection++;
    }
    else if (Enc.read()/ENC_MENURESDIVIDE < 0)
    {
      current_selection--;
    }
    // Reset data from Encoder
    Enc.write(0);

    // Indicate that an LCD update is needed
    flag.menu_lcd_upd = false;
  }

  // If LCD update is needed
  if (!flag.menu_lcd_upd)
  {
    flag.menu_lcd_upd = true;					// We have serviced LCD

    // Keep Encoder Selection Within Bounds of the Menu Size
    uint8_t menu_size = pep_menu_size;
    while(current_selection >= menu_size)
      current_selection -= menu_size;
    while(current_selection < 0)
      current_selection += menu_size;

    if      (current_selection == 1) controller_settings.PEP_period = 500;
    else if (current_selection == 2) controller_settings.PEP_period = 1000;
    else    controller_settings.PEP_period = 200;

    virt_lcd_clear();
    virt_lcd_setCursor(0,0);
    virt_lcd_print("PEP sampling period:");
    virt_lcd_setCursor(0,1);
    virt_lcd_print("Select");

    // Print the Rotary Encoder scroll Menu
    lcd_scroll_Menu((char**)pep_menu_items, menu_size, current_selection, 1, 6,1);

    virt_lcd_setCursor(0,2);
    virt_lcd_print("Available periods");
    virt_lcd_setCursor(0,3);
    virt_lcd_print("1, 2.5 or 5 seconds");
  }

  // Enact selection
  if (flag.short_push)
  {
    virt_lcd_clear();
    virt_lcd_setCursor(0,1);

    flag.short_push = false;                    // Clear pushbutton status

    // Check if selected threshold is not same as previous
    temp = controller_settings.PEP_period;			
    EEPROM_readAnything(1,controller_settings);
    if (controller_settings.PEP_period != temp)
    {
      controller_settings.PEP_period = temp;
      EEPROM_writeAnything(1,controller_settings);
      virt_lcd_print("Value Stored");
    }
    else virt_lcd_print("Nothing Changed");

    Menu_exit_timer = 30;                       // Show on LCD for 3 seconds
    flag.config_menu = false;                   // We're done with Menu, EXIT
    menu_level = 0;                             // We are done with this menu level
    flag.menu_lcd_upd = false;                  // Make ready for next time
  }
}


#if AD8307_INSTALLED
//--------------------------------------------------------------------
// Power/SWR Meter Awake Threshold Sensitivity selection Menu
//--------------------------------------------------------------------
void meterawake_menu(void)
{
  static int8_t	current_selection;
  uint8_t temp;
	
  // Get Current value
  // 1uW=0.001=>1, 10uW=0.01=>2... 100uW=>3, 1mW=>4, 10mW=>5
  current_selection = controller_settings.idle_thresh - 1;

  // Selection modified by encoder.  We remember last selection, even if exit and re-entry
  if (Enc.read()/ENC_MENURESDIVIDE != 0)
  {
    if (Enc.read()/ENC_MENURESDIVIDE > 0)
    {
      current_selection++;
    }
    else if (Enc.read()/ENC_MENURESDIVIDE < 0)
    {
      current_selection--;
    }
    // Reset data from Encoder
    Enc.write(0);

    // Indicate that an LCD update is needed
    flag.menu_lcd_upd = false;
  }

  // If LCD update is needed
  if (!flag.menu_lcd_upd)
  {
    flag.menu_lcd_upd = true;                   // We have serviced LCD

    // Keep Encoder Selection Within Bounds of the Menu Size
    uint8_t menu_size = meterthresh_menu_size;
    while(current_selection >= menu_size)
    current_selection -= menu_size;
    while(current_selection < 0)
    current_selection += menu_size;

    // Update with currently selected value
    // 0=0, 1uW=0.001=>1, 10uW=0.01=>2... 100uW=>3, 1mW=>4, 10mW=>5
    controller_settings.idle_thresh = current_selection + 1;
 	
    virt_lcd_clear();
    virt_lcd_setCursor(0,0);
    virt_lcd_print("MeterAwake Threshld:");
    virt_lcd_setCursor(0,1);
    virt_lcd_print("Select");

    // Print the Rotary Encoder scroll Menu
    lcd_scroll_Menu((char**)meterthresh_menu_items, menu_size, current_selection, 1, 6,1);

    virt_lcd_setCursor(0,2);
    virt_lcd_print("Available thresholds");
    virt_lcd_setCursor(0,3);
    virt_lcd_print("1uW-10mW,  10x steps");
  }

  // Enact selection
  if (flag.short_push)
  {
    virt_lcd_clear();
    virt_lcd_setCursor(0,1);

    flag.short_push = false;                    // Clear pushbutton status

    // Check if selected threshold is not same as previous
    temp = controller_settings.idle_thresh;			
    EEPROM_readAnything(1,controller_settings);
    if (controller_settings.idle_thresh != temp)
    {
      controller_settings.idle_thresh = temp;
      EEPROM_writeAnything(1,controller_settings);
      virt_lcd_print("Value Stored");
    }
    else virt_lcd_print("Nothing Changed");

    Menu_exit_timer = 30;                       // Show on LCD for 3 seconds
    flag.config_menu = false;                   // We're done, EXIT
    menu_level = 0;                             // We are done with this menu level
    flag.menu_lcd_upd = false;                  // Make ready for next time
  }
}
#endif
#endif

//--------------------------------------------------------------------
// Debug Screen, exit on push
//--------------------------------------------------------------------
void debug_serial_menu(void)
{
  static bool debug_screen = false;    // 
  if (!debug_screen)                   // Announce Menu Mode when entering
  {
    radio.debug_to_lcd = true;         // Activate LCD Serial Debug mode
    virt_lcd_clear();
    virt_lcd_setCursor(0,2);
    virt_lcd_print("Serial Debug Screen:");
    virt_lcd_setCursor(0,3);
    virt_lcd_print("Short push to exit.");
    debug_screen = true;                // Do not print this announcement again
  }

  // Make sure other functions do not trigger virt_lcd_clear()
  //Menu_exit_timer = 255;
  
  // Exit on Button Push
  if (flag.short_push)
  {
    radio.debug_to_lcd = false;         // Deactivate LCD Serial Debug mode
    flag.short_push = false;            // Clear pushbutton status

    virt_lcd_clear();
    virt_lcd_setCursor(0,1);				
    virt_lcd_print("Nothing Changed");
    Menu_exit_timer = 20;               // Show on LCD for 2 seconds
    flag.config_menu = true;            // We're not done, just backing off
    menu_level = 0;                     // We are done with this menu level
    debug_screen = false;               // Make ready for next time
  }
}



//
//--------------------------------------------------------------------
// Manage the first level of Menus
//--------------------------------------------------------------------
//
void menu_level0(void)
{
  static int8_t	current_selection;      // Keep track of current menu selection
  // We remember last selection as modified by encoder, even if exit and re-entry

  if (Enc.read()/ENC_MENURESDIVIDE != 0)
  {
    if (Enc.read()/ENC_MENURESDIVIDE > 0)
    {
      current_selection++;
    }
    else if (Enc.read()/ENC_MENURESDIVIDE < 0)
    {
      current_selection--;   
    }
    // Reset data from Encoder
    Enc.write(0);

    // Indicate that an LCD update is needed
    flag.menu_lcd_upd = false;
  }

  if (!flag.menu_lcd_upd)               // Need to update LCD
  {
    flag.menu_lcd_upd = true;           // We have serviced LCD

    // Keep Encoder Selection Within Bounds of the Menu Size
    uint8_t menu_size = level0_menu_size;
    while(current_selection >= menu_size)
    current_selection -= menu_size;
    while(current_selection < 0)
    current_selection += menu_size;

    virt_lcd_clear();
    virt_lcd_print("Config Menu:");

    // Print the Menu
    lcd_scroll_Menu((char**)level0_menu_items, menu_size, current_selection,1, 0,3);
  }

  if (flag.short_push)
  {
    flag.short_push = false;            // Clear pushbutton status
    flag.menu_lcd_upd = false;          // force LCD reprint
    switch (current_selection)
    {
      case 0: // Nes Position Menu
        menu_level = NEW_POS_MENU;
        break;

      case 1: // Rewrite Position Menu
        menu_level = MANAGE_POS_MENU;
        break;

      case 2: // Delete Position Menu
        menu_level = DELETE_POS_MENU;
        break;

      case 3: // Clear Memories Menu
        menu_level = CLEAR_MEMORIES_MENU;
        break;

      case 4: // Stepper Settings Menu
        menu_level = STEPPER_SETTINGS_MENU;
        break;

      case 5: // Backlash Select Menu
        menu_level = BACKLASH_SELECT_MENU;
        break;

      case 6: // Transceiver Profile Select Menu
        menu_level = TRANSCEIVER_PROFILE_MENU;
        break;

      case 7: // Transceiver Select Menu
        menu_level = TRANSCEIVER_SELECT_MENU;
        break;

      case 8: // ICOM CI-V Address Select Menu
        menu_level = ICOM_CIV_MENU;
        break;

      case 9: // RS232 Signals Menu
        menu_level = RS232_SIG_MENU;
        break;

      case 10:// RS232 Data Rate Menu
        menu_level = RS232_RATE_MENU;
        break;

      #if PSWR_AUTOTUNE
      case 11: // TX Tune Power adjust Menu
        menu_level = TX_TUNEPWR_MENU;
        break;
      
      case 12: // SWR Threshold adjust Menu
        menu_level = SWR_THRESH_MENU;
        break;

      case 13:// Scale Range Set
        menu_level = SCALERANGE_MENU;
        break;

      case 14:// Calibrate
        menu_level = CAL_MENU;
        break;

      case 15:// PEP sampling period select
        menu_level = PEP_MENU;
        break;

      #if AD8307_INSTALLED
      case 16:// Power/SWR Meter Awake Threshold Sensitivity select
        menu_level = METER_AWAKE_MENU;
        break;

      case 17:// Debug Serial
        menu_level = DEBUG_SERIAL_MENU;
        break;
        
      #else
      case 16:// Debug Serial
        menu_level = DEBUG_SERIAL_MENU;
        break;
      #endif
      #else
      case 11:// Debug Serial
        menu_level = DEBUG_SERIAL_MENU;
        break;
      #endif
      
      default:
        // Exit
        virt_lcd_clear();
        virt_lcd_setCursor(1,1);
        Menu_exit_timer = 20;           // Show on LCD for 2 seconds
        virt_lcd_print("Return from Menu");
        flag.config_menu = false;       // We're done
    }
  }
}

//
//--------------------------------------------------------------------
// Scan the Configuraton Menu Status and delegate tasks accordingly
//--------------------------------------------------------------------
//
void ConfigMenu(void)
{
  // Select which menu level to manage
  if (menu_level == 0)                               menu_level0();
  else if (menu_level == NEW_POS_MENU)               new_pos_menu();
  else if (menu_level == MANAGE_POS_MENU)            manage_pos_menu();
  else if (menu_level == DELETE_POS_MENU)            delete_pos_menu();
  else if (menu_level == CLEAR_MEMORIES_MENU)        clear_memories_menu();
  else if (menu_level == STEPPER_SETTINGS_MENU)      stepper_settings_menu();
  else if (menu_level == STEPPER_RATE_SUBMENU)       stepper_rate_submenu();
  else if (menu_level == STEPPER_SPEEDUP_SUBMENU)    stepper_speedup_submenu();
  else if (menu_level == STEPPER_MICROSTEPS_SUBMENU) stepper_microsteps_submenu();
  else if (menu_level == BACKLASH_SELECT_MENU)       backlash_select_menu();
  else if (menu_level == TRANSCEIVER_PROFILE_MENU)   transceiver_profile_menu();
  else if (menu_level == TRANSCEIVER_SELECT_MENU)    transceiver_select_menu();
  else if (menu_level == ICOM_CIV_MENU)              icom_civ_address_menu();
  else if (menu_level == RS232_SIG_MENU)             rs232_sig_menu();
  else if (menu_level == RS232_RATE_MENU)            rs232_rate_menu();
  
  #if PSWR_AUTOTUNE
  else if (menu_level == TX_TUNEPWR_MENU)            tx_tunepwr_menu();
  else if (menu_level == SWR_THRESH_MENU)            swr_threshold_menu();
  else if (menu_level == SCALERANGE_MENU)            scalerange_menu();
  else if (menu_level == SCALE_SET0_MENU)            scalerange_menu_level2();
  else if (menu_level == SCALE_SET1_MENU)            scalerange_menu_level2();
  else if (menu_level == SCALE_SET2_MENU)            scalerange_menu_level2();
  else if (menu_level == CAL_MENU)                   calibrate_menu();
  #if AD8307_INSTALLED
  else if (menu_level == CAL_SET0_MENU)              calibrate_menu_level2();
  else if (menu_level == CAL_SET1_MENU)              calibrate_menu_level2();
  else if (menu_level == CAL_SET2_MENU)              calibrate_menu_level2();
  #endif
  else if (menu_level == PEP_MENU)                   pep_menu();
  #if AD8307_INSTALLED
  else if (menu_level == METER_AWAKE_MENU)           meterawake_menu();
  #endif
  #endif
  else if (menu_level == DEBUG_SERIAL_MENU)          debug_serial_menu();
}
