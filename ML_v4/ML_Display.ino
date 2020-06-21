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

//*********************************************************************************
//**
//**  The Bargraph function lcdProgressBarPeak() below relies on a direct swipe
//**  from the AVRLIB lcd.c/h by Pascal Stang.
//**
//**  The original Bargraph function has been modified as follows:
//**
//**  A new Peak Bar (sticky bar) functionality has been added.
//**  A different set of symbols to construct the bars, simulating an N8LP style bargraph.
//**  Adaptation for use with the LiquidCrystalFast library and the virt_lcd_write function
//**
//**                  See the AVRLIB copyright notice below.
//**
// Copy/Paste of copyright notice from AVRLIB lcd.h:
//*****************************************************************************
//
// File Name	: 'lcd.h'
// Title		: Character LCD driver for HD44780/SED1278 displays
//					(usable in mem-mapped, or I/O mode)
// Author		: Pascal Stang
// Created		: 11/22/2000
// Revised		: 4/30/2002
// Version		: 1.1
// Target MCU	: Atmel AVR series
// Editor Tabs	: 4
//
///	\ingroup driver_hw
/// \defgroup lcd Character LCD Driver for HD44780/SED1278-based displays (lcd.c)
/// \code #include "lcd.h" \endcode
/// \par Overview
///		This display driver provides an interface to the most common type of
///	character LCD, those based on the HD44780 or SED1278 controller chip
/// (about 90% of character LCDs use one of these chips).† The display driver
/// can interface to the display through the CPU memory bus, or directly via
/// I/O port pins.† When using the direct I/O port mode, no additional
/// interface hardware is needed except for a contrast potentiometer.
///†Supported functions include initialization, clearing, scrolling, cursor
/// positioning, text writing, and loading of custom characters or icons
/// (up to 8).† Although these displays are simple, clever use of the custom
/// characters can allow you to create animations or simple graphics.† The
/// "progress bar" function that is included in this driver is an example of
/// graphics using limited custom-chars.
///
/// \Note The driver now supports both 8-bit and 4-bit interface modes.
///
/// \Note For full text output functionality, you may wish to use the rprintf
/// functions along with this driver
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************



//
//-----------------------------------------------------------------------------------------
// 20x4 LCD Print Routines
//
// LCD Printing is done as a two step process:
//
// 1) Data is printed to a virtual LCD consisting of an 80 character long string; 
//
// 2) The string is then printed piecemeal, in a round robin fashion, to the real 20x4 LCD,
//    one character per 333 microseconds approximately.
//
// The piecemeal print is to ensure no undue delays associated with servicing the LCD,
// such delays would translate to uneven Stepper Motor rate
//-----------------------------------------------------------------------------------------
//

char    virt_lcd[81];     // Character array representing what is pending for LCD
uint8_t virt_x, virt_y;   // x and y coordinates for LCD
//
//-----------------------------------------------------------------------------------------
//      Move characters to LCD from virtual LCD
//      This function is called once every time the Stepper is serviced, up to once every 667us/
//
//      3 character are transferred per one millisecond, approximately, using step_rate to pace
//      the transfer (30/step_rate ~ step_rate 10 equals 1ms, step_rate 15 equals 667us)
//-----------------------------------------------------------------------------------------
//
void virt_LCD_to_real_LCD(void)
{
  static char    real_lcd[81];                      // Character array representing what is visible on LCD
  static uint8_t character;                         // Character position on LCD, as 0 - 79

  for (uint8_t i = 0; i < 15/step_rate; i++)        // Print two or more chars to LCD (one per 667 microseconds)
  {                                                 // (did try to increase this significantly, no significant
                                                    //  affect on performance until at 100 chars or higher (1500))
    if (virt_lcd[character] != real_lcd[character]) // We have a new character to write out
    {
      lcd.setCursor(character%20, character/20);    // Set cursor to position of new char
      real_lcd[character] = virt_lcd[character];
      lcd.print(real_lcd[character]);               // Write new char
    }
    character++;                                    // Advance by one character in the 80 char long string
    if (character >= 80) character = 0;
  }
}

//
//-----------------------------------------------------------------------------------------
//      Print to a Virtual LCD - 80 character long string representing a 20x4 LCD
//-----------------------------------------------------------------------------------------
//
void virt_lcd_write(char ch)
{
  uint8_t virt_pos;
  
  // Print to the 20x4 virtual LCD
  virt_pos = virt_x + 20*virt_y;          // Determine position on virt LCD
  virt_lcd[virt_pos++] = ch;              // Place character on virt LCD
  if (virt_pos >= 80) virt_pos = 0;       // At end, wrap around to beginning 

  // After print, derive new x,y coordinates in our 20 by 4 matrix
  virt_x = virt_pos;
  for(virt_y = 0; virt_y < 4 && virt_x >= 20;virt_y++)         
  {
    virt_x -=20;
  }
}
void virt_lcd_print(const char *ch_in)
{
  //uint8_t virt_pos;
  uint8_t virt_len;
  
  // Print to the 20x4 virtual LCD
  virt_len = strlen(ch_in);
  for (uint8_t i = 0; i < virt_len; i++)
  {
    virt_lcd_write(ch_in[i]);
  }
}

//
//-----------------------------------------------------------------------------------------
//      Print in lowest line and scroll older data up
//-----------------------------------------------------------------------------------------
//
void virt_lcd_print_scroll(const char *ch_in)
{
  uint8_t i, j;
  
  // Copy everything up one line
  for (i = 0; i< 60; i++) virt_lcd[i] = virt_lcd[i+20];

  // Clear the last line  
  for (i = 60; i < 80; i++) virt_lcd[i] = ' ';

  // Truncate input at length of line
  j = strlen(ch_in);  
  if (j > 20) j = 20;
  
  // Copy input to last line
  for (i = 0; i < j; i++) virt_lcd[60 + i] = ch_in[i];
}
//
//-----------------------------------------------------------------------------------------
//      Set Cursor virtual LCD
//-----------------------------------------------------------------------------------------
//
void virt_lcd_setCursor(uint8_t x, uint8_t y)
{
  virt_x = x;
  virt_y = y;
}

//
//-----------------------------------------------------------------------------------------
//      Clear virtual LCD
//-----------------------------------------------------------------------------------------
//
void virt_lcd_clear(void)
{
  for (uint8_t i = 0; i < 80; i++) virt_lcd[i] = ' ';  // Print a lot of spaces to virt LCD
  virt_x = 0;
  virt_y = 0;
}

//-----------------------------------------------------------------------------------------
// Display Bargraph - including Peak Bar, if relevant
//
// "length" indicates length of bargraph in characters 
// (max 16 on a 16x2 display or max 20 on a 20x4 display)
//
// (each character consists of 6 bars, thereof only 5 visible)
//
// "maxprogress" indicates full scale (16 bit unsigned integer)
//
// "progress" shown as a proportion of "maxprogress"  (16 bit unsigned integer)
//
// if "prog_peak" (16 bit unsigned integer) is larger than "progress",
// then Peak Bar is shown in the middle of that character position
//-----------------------------------------------------------------------------------------
void lcdProgressBarPeak(uint16_t progress, uint16_t prog_peak, uint16_t maxprogress, uint8_t length)
{

  # define PROGRESSPIXELS_PER_CHAR 6                    // progress bar defines
  uint16_t i;
  uint16_t pixelprogress;
  uint8_t c;

  if (progress >= maxprogress) progress = maxprogress;  // Clamp the upper bound to prevent funky readings

  // draw a progress bar displaying (progress / maxprogress)
  // starting from the current cursor position
  // with a total length of "length" characters
  // ***note, LCD chars 0-6 must be programmed as the bar characters
  // char 0 = empty ... char 5 = full, char 6 = peak bar - disabled if maxprogress set as 0 (or lower than progress)

  // total pixel length of bargraph equals length*PROGRESSPIXELS_PER_CHAR;
  // pixel length of bar itself is
  pixelprogress = ((uint32_t)progress*(length*PROGRESSPIXELS_PER_CHAR)/maxprogress);
	
  // print exactly "length" characters
  for(i=0; i<length; i++)
  {
    // check if this is a full block, or partial or empty
    if( ((i*PROGRESSPIXELS_PER_CHAR)+PROGRESSPIXELS_PER_CHAR) > pixelprogress )
    {
      // this is a partial or empty block
      if( ((i*PROGRESSPIXELS_PER_CHAR)) > pixelprogress )
      {
        // If an otherwise empty block contains previous "Peak", then print peak char
        // If this function is not desired, simply set prog_peak at 0 (or as equal to progress)
        if(i == ((uint32_t)length * prog_peak)/maxprogress)
        {
          c = 6;
        }				
        // othwerwise this is an empty block
        else
        {
          c = 0;
        }
      }
      else
      {
        // this is a partial block
        c = pixelprogress % PROGRESSPIXELS_PER_CHAR;
      }
    }
    else
    {
      // this is a full block
      c = 5;
    }

    // write character to display
    virt_lcd_write(c);
  }
}


//-----------------------------------------------------------------------------------------
// custom LCD characters for Bargraph
const uint8_t LcdCustomChar[7][8] =
      {
        // N8LP LP-100 look alike bargraph
        { 0x00, 0x00, 0x00, 0x15, 0x00, 0x00, 0x00, 0x00 }, // 0. 0/5 full progress block
        { 0x00, 0x10, 0x10, 0x15, 0x10, 0x10, 0x00, 0x00 }, // 1. 1/5 full progress block
        { 0x00, 0x18, 0x18, 0x1d, 0x18, 0x18, 0x00, 0x00 }, // 2. 2/5 full progress block
        { 0x00, 0x1c, 0x1c, 0x1d, 0x1C, 0x1c, 0x00, 0x00 }, // 3. 3/5 full progress block
        { 0x00, 0x1e, 0x1e, 0x1E, 0x1E, 0x1e, 0x00, 0x00 }, // 4. 4/5 full progress block
        { 0x00, 0x1f, 0x1f, 0x1F, 0x1F, 0x1f, 0x00, 0x00 }, // 5. 5/5 full progress block
        { 0x06, 0x06, 0x06, 0x16, 0x06, 0x06, 0x06, 0x06 }  // 6. Peak Bar
      };

//-----------------------------------------------------------------------------------------
// Initialize LCD for bargraph display - Load 6 custom bargraph symbols and a PeakBar symbol
//-----------------------------------------------------------------------------------------
void lcd_bargraph_Init(void)
{
  for (uint8_t i=0; i<7; i++)
  {
    lcd.createChar(i, (uint8_t*) LcdCustomChar[i]);
  }
}


#if PSWR_AUTOTUNE
//
//-----------------------------------------------------------------------------
//      Print Format SWR, returns string in print_buf
//-----------------------------------------------------------------------------
//
void print_swr(void)
{
  if (measured_swr < 2.0)
    sprintf(print_buf," %1.02f",measured_swr);
  else if (measured_swr <= 10.0)
    sprintf(print_buf,"  %1.01f",measured_swr);
  else if (measured_swr <= 10000.0)
    sprintf(print_buf,"%5u",(uint16_t) measured_swr);
  else
    //sprintf(print_buf,"     ");
    sprintf(print_buf,"  N/A");
}

//-----------------------------------------------------------------------------
// AD8307 specific Print Formatting, allowing extremely low values to be displayed
#if AD8307_INSTALLED  
//
//-----------------------------------------------------------------------------
//			Print Power, input value is in milliWatts, 
//			returns string in print_buf
//-----------------------------------------------------------------------------
//
void print_p_mw(double mw)
{
  uint32_t p_calc;
  uint16_t power_sub, power;

  if(mw >= 1000000.0)                      // 1kW
  {
    p_calc = mw;
    power = p_calc / 1000;
    sprintf(print_buf," %4uW",power);
  }
  else if(mw >= 100000.0)                  // 100W
  {
    p_calc = mw;
    power = p_calc / 1000;
    sprintf(print_buf,"  %3uW",power);
  }
  else if(mw >= 10000.0)                   // 10W
  {
    p_calc = mw;
    power = p_calc / 1000;
    power_sub = (p_calc % 1000)/100;
    sprintf(print_buf," %2u.%01uW",power, power_sub);
  }
  else if(mw >= 1000.0)                    // 1W
  {
    p_calc = mw;
    power = p_calc / 1000;
    power_sub = (p_calc % 1000)/10;
    sprintf(print_buf," %1u.%02uW",power, power_sub);
  }
  else if(mw >= 100.0)                     // 100mW
  {
    sprintf(print_buf," %3umW",(uint16_t)mw);
  }
  else if(mw >= 10.0)                      // 10mW
  {
    p_calc = mw * 10;
    power = p_calc / 10;
    power_sub = p_calc % 10;
    sprintf(print_buf,"%2u.%01umW",power, power_sub);
  }
  else if(mw >= 1.0)                       // 1mW
  {
    p_calc = mw * 100;
    power = p_calc / 100;
    power_sub = p_calc % 100;
    sprintf(print_buf,"%1u.%02umW",power, power_sub);
  }
  else if(mw >= 0.1)                       // 100uW
  {
    power = mw * 1000;
    sprintf(print_buf," %3uuW",power);
  }
  else if(mw >= 0.01)                      // 10uW
  {
    p_calc = mw * 10000;
    power = p_calc / 10;
    power_sub = p_calc % 10;
    sprintf(print_buf,"%2u.%01uuW",power, power_sub);
  }
  else if(mw >= 0.001)                     // 1 uW
  {
    p_calc = mw * 100000;
    power = p_calc / 100;
    power_sub = p_calc % 100;
    sprintf(print_buf,"%1u.%02uuW",power, power_sub);
  }
  else if(mw >= 0.0001)                    // 100nW
  {
    power = mw * 1000000;
    sprintf(print_buf," %3unW",power);
  }
  else if(mw >= 0.00001)                   // 10nW
  {
    p_calc = mw * 10000000;
    power = p_calc / 10;
    power_sub = p_calc % 10;
    sprintf(print_buf,"%2u.%01unW",power, power_sub);
  }
  else if(mw >= 0.000001)                  // 1nW
  {
    p_calc = mw * 100000000;
    power = p_calc / 100;
    power_sub = p_calc % 100;
    sprintf(print_buf,"%1u.%02unW",power, power_sub);
  }
  else if(mw >= 0.0000001)                 // 100pW
  {
    power = mw * 1000000000;
    sprintf(print_buf," %3upW",power);
  }
  else if(mw >= 0.00000001)                // 10pW
  {
    p_calc = mw * 10000000000.0;
    power = p_calc / 10;
    power_sub = p_calc % 10;
    sprintf(print_buf,"%2u.%01upW",power, power_sub);
  }
  else if(mw >= 0.000000001)               // 1pW
  {
    p_calc = mw * 100000000000.0;
    power = p_calc / 100;
    power_sub = p_calc % 100;
    sprintf(print_buf,"%1u.%02upW",power, power_sub);
  }
  else if(mw >= 0.0000000001)              // 100fW
  {
    power = mw * 1000000000000.0;
    sprintf(print_buf," %3ufW",power);
  }
  else if(mw >= 0.00000000001)             // 10fW
  {
    p_calc = mw * 10000000000000.0;
    power = p_calc / 10;
    power_sub = p_calc % 10;
    sprintf(print_buf,"%2u.%01ufW",power, power_sub);
  }
  else                                     // 1fW
  {
    //p_calc = mw * 100000000000000;
    //power = p_calc / 100;
    //power_sub = p_calc % 100;
    //sprintf(print_buf,"%1u.%02ufW",power, power_sub);
    sprintf(print_buf," 0.00W");
  }
}

//
//-----------------------------------------------------------------------------
//			Print Power, reduced resolution, input value is in milliWatts, 
//			returns string in print_buf
//-----------------------------------------------------------------------------
//
void print_p_reduced(double mw)
{
  uint32_t p_calc;
  uint16_t power_sub, power;

  if(mw >= 1000000.0)                      // 1kW
  {
    p_calc = mw;
    power = p_calc / 1000;
    sprintf(print_buf,"%4uW",power);
  }
  else if(mw >= 100000.0)                  // 100W
  {
    p_calc = mw;
    power = p_calc / 1000;
    sprintf(print_buf,"%3uW",power);
  }
  else if(mw >= 10000.0)                   // 10W
  {
    p_calc = mw;
    power = p_calc / 1000;
    sprintf(print_buf," %2uW",power);
  }
  else if(mw >= 1000.0)                    // 1W
  {
    p_calc = mw;
    power = p_calc / 1000;
    power_sub = (p_calc % 1000)/10;
    sprintf(print_buf,"%1u.%01uW",power, power_sub);
  }
  else if(mw >= 100.0)                     // 100mW
  {
    sprintf(print_buf,"%3umW",(uint16_t)mw);
  }
  else if(mw >= 10.0)                      // 10mW
  {
    sprintf(print_buf," %2umW",(uint16_t)mw);
  }
  else                                     // 1mW
  {
    sprintf(print_buf,"  %1umW",(uint16_t)mw);
  }
}


//-----------------------------------------------------------------------------
// Diode detector specific Print Formatting, allowing values down to 1mW to be displayed
#else
//
//-----------------------------------------------------------------------------
//      Print Format Power, input value is in milliWatts, 
//      returns string of 6 characters in print_buf
//-----------------------------------------------------------------------------
//
void print_p_mw(int32_t mw)
{
  uint32_t p_calc;
  uint32_t power_sub, power;

  if(mw >= 1000000.0)            // 1kW
  {
    p_calc = mw;
    power = p_calc / 1000;
    sprintf(print_buf," %4luW",power);
  }
  else if(mw >= 100000.0)        // 100W
  {
    p_calc = mw;
    power = p_calc / 1000;
    sprintf(print_buf,"  %3luW",power);
  }
  else if(mw >= 10000.0)         // 10W
  {
    p_calc = mw;
    power = p_calc / 1000;
    power_sub = (p_calc % 1000)/100;
    sprintf(print_buf," %2lu.%01luW",power, power_sub);
  }
  else if(mw >= 1000.0)          // 1W
  {
    p_calc = mw;
    power = p_calc / 1000;
    power_sub = (p_calc % 1000)/10;
    sprintf(print_buf," %1lu.%02luW",power, power_sub);  
  }
  else                           // 100 mw or less
  {
    sprintf(print_buf," %3lumW", mw);
  }
}


//
//-----------------------------------------------------------------------------
//      Print Format Power, reduced resolution, input value is in milliWatts, 
//      returns string in print_buf
//-----------------------------------------------------------------------------
//
void print_p_reduced(int32_t mw)
{
  uint32_t p_calc;
  uint32_t power_sub, power;
  
  if(mw >= 1000000.0)            // 1kW
  {
    p_calc = mw;
    power = p_calc / 1000;
    sprintf(print_buf,"%4luW",power);
  }
  else if(mw >= 100000.0)        // 100W
  {
    p_calc = mw;
    power = p_calc / 1000;
    sprintf(print_buf,"%3luW",power);
  }
  else if(mw >= 10000.0)         // 10W
  {
    p_calc = mw;
    power = p_calc / 1000;
    sprintf(print_buf," %2luW",power);
  }
  else if(mw >= 1000.0)          // 1W
  {
    p_calc = mw;
    power = p_calc / 1000;
    power_sub = (p_calc % 1000)/10;
    sprintf(print_buf,"%1lu.%01luW",power, power_sub);
  }
  else                           // 100mW or less
  {
    sprintf(print_buf,"%3lumW", mw);
  }
}
#endif

//
//-----------------------------------------------------------------------------------------
//      Scale Bargraph based on highest instantaneous power reading during last 10 seconds
//      (routine is called once every 1/10th of a second
//
//      The scale ranges are user definable, up to 3 ranges per decade
//      e.g. 6, 12 and 24 gives:
//      ... 6mW, 12mW, 24mW, 60mW ... 1.2W, 2.4W, 6W 12W 24W 60W 120W ...
//      If all three values set as "2", then: 
//      ... 2W, 20W, 200W ...
//      The third and largest value has to be less than ten times the first value 
//-----------------------------------------------------------------------------------------
//
uint32_t scale_BAR(uint32_t pow )           // power in milliwatts
{
  #define  SCALE_BUFFER  100                // Keep latest 100 power readings in buffer

  // For measurement of peak and average power
  static int32_t peak_buff[SCALE_BUFFER];   // 10 second window
  int32_t max_val;                          // Find max value
  #if AD8307_INSTALLED
  int32_t decade = 1;                       // Determine range decade, lowest is 1mW
  #else
  int32_t decade = 100;                     // Determine range decade, lowest is 100mW
  #endif
  uint32_t scale;                           // Scale output

  static uint8_t a;                         // Entry counter

  // Store PEP value in ring buffer
  peak_buff[a] = pow;
  a++;
  if (a == SCALE_BUFFER) a = 0;

  // Retrieve the max value out of the measured window
  max_val = 0;                              // Set lowest scale at or above 100mw
  for (uint8_t b = 0; b < SCALE_BUFFER; b++)
  {
    if (max_val < peak_buff[b]) max_val = peak_buff[b];
  }

  // Determine range decade
  while ((decade * controller_settings.Scale[2]) < (max_val))
  {
    decade = decade * 10;
  }

  // And determine scale limit to use, within the decade previously determined
  if      (max_val >= (decade * controller_settings.Scale[1]))  scale = decade * controller_settings.Scale[2];
  else if (max_val >= (decade * controller_settings.Scale[0]))  scale = decade * controller_settings.Scale[1];
  else    scale = decade * controller_settings.Scale[0];

  return scale;                             // Return value is in mW
}


//
//-----------------------------------------------------------------------------------------
//      Scale Values to fit Power Bargraph (16 bit integer)
//-----------------------------------------------------------------------------------------
//
void scale_PowerBarInpValues(uint32_t fullscale, double bar, double peak_bar, 
                            uint16_t *out_fullscale, uint16_t *out_bar, uint16_t *out_peak)
{
  uint32_t baradj;
  uint32_t peakadj;

  // Convert variables to integer and scale them to fit 16bit integers
  // used in the input variables to lcdProgressBarPeak()

  // If we have 10 mW or more...	
  if (fullscale >= 10)
  {
    baradj = (uint32_t) bar;
    peakadj = (uint32_t) peak_bar;

    // Scale down the bar graph resolution to fit 16 bit integer
    while (fullscale >= 0x10000)
    {
      fullscale = fullscale/0x10;
      baradj = baradj/0x10;
      peakadj = peakadj/0x10;
    }
  }

   // If power is under 10mW, then crank the bar graph resolution up by 0x10
  else
  {
    fullscale = fullscale * 0x10;
    baradj = (uint32_t) (bar * 0x10);
    peakadj = (uint32_t) (peak_bar * 0x10);
  }
  
  // Copy adjusted values into 16 bit variables
  *out_fullscale = fullscale;
  *out_bar = baradj;
  *out_peak = peakadj;	
}
#endif

//
//-----------------------------------------------------------------------------------------
//      Display Frequency as XX.XXX.XXX
//-----------------------------------------------------------------------------------------
//
void display_frq(int32_t frq)
{
  int32_t f1, f2, f3;                         // Used to format frequency printout
  int8_t sign = 1;
  
  if (frq < 0) sign = -1;                     // Format negative frequency correctly	
  f1 = frq/1000000;                           // (although this makes limited sense :)
  f3 = sign*frq%1000000;
  f2 = f3/1000;
  f3 = f3%1000;
  if (f1 <= -10) virt_lcd_print("-");
  else if ((frq < 0) && (f1 > -10)) virt_lcd_print(" -");
  else if (f1 < 10) virt_lcd_print("  ");
  else virt_lcd_print(" ");
  sprintf(print_buf,"%u.%03u.%03u",(uint16_t)(sign*f1),(uint16_t)f2,(uint16_t)f3);
  virt_lcd_print(print_buf);
}

//
//-----------------------------------------------------------------------------
//      Display Stepper Position
//      Indicate number of revolutions above lowest preset
//-----------------------------------------------------------------------------
//
void display_stepper_pos(int32_t pos)
{
  float pos_rev;
  // 8 * 360 / STEPPER_ANGLE = number of [8x] microsteps per revolution
  pos_rev = dir_of_travel * (pos - min_preset[ant].Pos)/(8 * 360/STEPPER_ANGLE);
  sprintf(print_buf,"%7.3f",pos_rev);
  virt_lcd_print(print_buf);
}


//
//-----------------------------------------------------------------------------
//		Screensaver display
//-----------------------------------------------------------------------------
//
void screensaver(void)
{
  static uint8_t count = SCREENSAVE_UPDATE;       // Force screensaver print when run first time
  char MHz_string[21];                            // String with frequency information to print

  if (count++ == SCREENSAVE_UPDATE)               // We reprint the output every fiftieth time (approx 5 sec)
  {
    #if SCREENSAVE_CUST                           // Custom Screensave Message
    strcpy(MHz_string,SCREENSAVE_CUST_TEXT);      // Alternate text to display, defined in ML.h
    #else                                         // Normal Screensave Message
    double frq = tunedFrq/1000000.0;              // Used to format frequency printout
    if (frq == 0) sprintf(MHz_string,"ZZzzz..."); // Display this if no frequency/pos data has been stored
    else sprintf(MHz_string,"%3.03f MHz",frq);    // Else display the currently tuned frequency
    #endif
    count = 0;
    virt_lcd_clear();
    virt_lcd_setCursor(rand() % (21 - strlen(MHz_string)), rand() % 4) ;   
    virt_lcd_print(MHz_string);
  }
}

#if PSWR_AUTOTUNE
//
//-----------------------------------------------------------------------------
//      Display: SWR Tune progress
//-----------------------------------------------------------------------------
//
void lcd_display_swr_tune(void)
{
  //------------------------------------------
  // SWR Bargraph
  virt_lcd_setCursor(0,1);
  lcdProgressBarPeak(swr_bar,0,1000, 15);
  //------------------------------------------
  // SWR Printout
  print_swr();                       // Print the "SWR value"
  virt_lcd_print(print_buf);
}

//
//-----------------------------------------------------------------------------
//      Display: Bargraph, Power in Watts, SWR & PEP Power
//      PEP Power always displayed and used for scale selection
//      power variable can be anything passed to function (power_mv_avg, power_mw_pk, etc...)
//-----------------------------------------------------------------------------
//
#if AD8307_INSTALLED
void lcd_display_power_and_swr(const char * power_display_indicator, double power)
#else
void lcd_display_power_and_swr(const char * power_display_indicator, int32_t power)
#endif
{
  int32_t scale;                          // Progress bar scale

  // Used to scale/fit progressbar inputs into 16 bit variables, when needed
  uint16_t bar_power=0, bar_power_pep=0, bar_scale=0;	
  
  // Determine scale setting, also used to determine if level above useful threshold
  if (power_mw_pep >= power_mw) scale = scale_BAR(power_mw_pep);  
  else scale = scale_BAR(power_mw);        // In certain cases the pep is not calculated
                                           
  #if PSWR_AUTOTUNE
  //----------------------------------------------
  // Display Power if level is above useful threshold
  //
  // If input power above a minimum relevant value is detected (see PM.h)
  // or if scale indicates a value higher than 10mW
  // (= significant power within last 10 seconds)
  #if AD8307_INSTALLED
  if ((power_mw > pow(10,controller_settings.idle_thresh-4)) || (scale > 10))
  #else  
  if ((power_mw > MIN_PWR_FOR_METER) || (scale > 10))
  #endif  
  {
    //------------------------------------------
    // Power indication and Bargraph
    virt_lcd_setCursor(0,0);

    //------------------------------------------
    // Scale variables to fit 16bit input to lcdProgressBarPeak()
    scale_PowerBarInpValues(scale, power, power_mw_pep, &bar_scale, &bar_power, &bar_power_pep);
    //------------------------------------------
    // Power Bargraph
    lcdProgressBarPeak(bar_power,bar_power_pep,bar_scale, 20);

    //------------------------------------------
    // SWR Bargraph
    virt_lcd_setCursor(0,1);
    lcdProgressBarPeak(swr_bar,0,1000, 20);

    //------------------------------------------
    // SWR Printout
    virt_lcd_setCursor(7,2);          // Clear junk in line, if any
    virt_lcd_print("   ");
    virt_lcd_setCursor(0,2);
    virt_lcd_print("SWR ");
    print_swr();                      // and print the "SWR value"
    virt_lcd_print(print_buf);

    //------------------------------------------
    // Scale Indication
    virt_lcd_setCursor(8,3);          // Ensure last couple of chars in line are clear
    virt_lcd_print("  ");
    virt_lcd_setCursor(0,3);
    virt_lcd_print("Scale");
    #if AD8307_INSTALLED
    print_p_reduced((double)scale);   // Scale Printout
    #else
    print_p_reduced(scale);           // Scale Printout
    #endif
    virt_lcd_print(print_buf);

    //------------------------------------------
    // Power Indication
    virt_lcd_setCursor(19,2);         // Clear junk in line, if any
    virt_lcd_print(" ");
    virt_lcd_setCursor(10,2);         // Clear junk in line, if any
    virt_lcd_print(" ");
    virt_lcd_print(power_display_indicator);
    if (Reverse)                      // If reverse power, then indicate
    {
      virt_lcd_setCursor(13,2);
      virt_lcd_print("-");
    }
    virt_lcd_setCursor(14,2);
    print_p_mw(power);
    virt_lcd_print(print_buf);

    //------------------------------------------
    // Power indication, PEP
    virt_lcd_setCursor(19,3);         // Clear junk in line, if any
    virt_lcd_print(" ");
    virt_lcd_setCursor(10,3);         // Clear junk in line, if any
    virt_lcd_print(" pep");
    virt_lcd_setCursor(14,3);
    print_p_mw(power_mw_pep);
    virt_lcd_print(print_buf);
  }
  #endif
}


//
//-----------------------------------------------------------------------------
//      Display Power and SWR during Stepper Move Operations other than SWR autotune
//      Bargraph, Power in Watts and SWR in the first two lines
//      Display stepper stuff in the second two lines
//-----------------------------------------------------------------------------
//
void lcd_display_power_and_swr_during_man_tune(void)  
{
  uint32_t scale;                          // Progress bar scale

  // Used to scale/fit progressbar inputs into 16 bit variables, when needed
  uint16_t bar_power=0, bar_power_pep=0, bar_scale=0;	

  scale = scale_BAR(power_mw);             // Determine scale setting, also used to
                                           // determine if level above useful threshold
  //----------------------------------------------
  // Display Power if level is above useful threshold
  //
  // If input power above a minimum relevant value is detected (see ML.h)
  // or if scale indicates a value higher than 100mW
  // (= significant power within last 10 seconds)
  #if AD8307_INSTALLED
  if ((power_mw > pow(10,controller_settings.idle_thresh-4)) || (scale > 100))
  #else  
  if ((power_mw > MIN_PWR_FOR_METER) || (scale > 100))
  #endif
  {
    //------------------------------------------
    // Power indication and Bargraph
    virt_lcd_setCursor(0,0);

    //------------------------------------------
    // Scale variables to fit 16bit input to lcdProgressBarPeak()
    scale_PowerBarInpValues(scale, power_mw, power_mw, &bar_scale, &bar_power, &bar_power_pep);
    //------------------------------------------
    // Power Bargraph and Power Indication
    lcdProgressBarPeak(bar_power,0 /* no pep */,bar_scale, 14);
    print_p_mw(power_mw);
    virt_lcd_print(print_buf);
    //------------------------------------------
    // SWR Bargraph and SWR Indication
    virt_lcd_setCursor(0,1);
    lcdProgressBarPeak(swr_bar,0,1000, 14);
    virt_lcd_print(" ");
    print_swr();                      // and print the "SWR value"
    virt_lcd_print(print_buf);

    //-----------------------------------
    //      Print Stepper Position data, range and status in lower two lines of LCD
    //----------------------------------- 
    lcd_display_stepper_data();
  }
}
#endif

//
//-----------------------------------------------------------------------------
//      Print Stepper Position data, range and status in lower two lines of LCD
//-----------------------------------------------------------------------------
//
void lcd_display_stepper_data(void)
{
  // Indicate capacitor position (number of revolutions above lowest preset)
  //
  virt_lcd_setCursor(0,2);
  virt_lcd_print("StepP: ");
  virt_lcd_setCursor(6,2);
  display_stepper_pos(running[ant].Pos);			
  virt_lcd_setCursor(13,2);
  display_stepper_pos(stepper_track[ant]);			

  // Indicate whether Stepper is Active or not
  //
  virt_lcd_setCursor(10,3);
  virt_lcd_print(" Motor:");
  virt_lcd_setCursor(17,3);
  if (flag.frq_store)
    virt_lcd_print(" On");
  else 
    virt_lcd_print("Off");
      
  // Indicate which Antenna and frq/pos memory (range) we're using
  //
  virt_lcd_setCursor(0,3);
  // Double antenna configuration
  #if (ANT1_CHANGEOVER || ANT_CHG_2BANKS)   // Dual antenna mode enabled
  sprintf(print_buf,"Mem: %u-%02u ", ant+1, range);
  #else                                    // Single antenna configuration
  sprintf(print_buf,"Range: %2u ", range);
  #endif
  virt_lcd_print(print_buf);

  //-----------------------------------
  // Error Conditions - overwrite stuff in fouth line
  //-----------------------------------
  #if ENDSTOP_OPT == 1                     // Vacuum Capacitor, no end stops
  // Display Frequency out of range condition
  //
  if (flag.frq_xrange)
  {
    virt_lcd_setCursor(0,3); 
    virt_lcd_print("FRQ out of range");
  }

  // Display Capacitor out of range condition
  //
  if (flag.cap_lower_pos || flag.cap_upper_pos)
  {
    virt_lcd_setCursor(0,3);
    virt_lcd_print("CAP out of range");
  }    
  #endif
  #if ENDSTOP_OPT == 2                     // End stop sensors implemented
  // Display Lower EndStop condition
  //
  if (flag.endstop_lower)
  {
    virt_lcd_setCursor(0,3);
    virt_lcd_print("Lower EndStop   ");
  }
    
  // Display Upper Endstop condition
  //
  if (flag.endstop_upper)
  {
    virt_lcd_setCursor(0,3);
    virt_lcd_print("Upper EndStop   ");
  }    
  #endif
  #if ENDSTOP_OPT == 3                     // Butterfly capacitor, no end stops required
  if (0);                                  // Nothing here
  #endif
}


//
//-----------------------------------------------------------------------------
//      Indicate Radio Frequency and Tuned
//      Frequency in first two lines
//-----------------------------------------------------------------------------
//
void lcd_display_radio_frequency_data(void)
{   
  // Is input from Radio inactive?
  if (!radio.online)                     // Activity flag indicates inactive
  {
    virt_lcd_setCursor(0,0);
    virt_lcd_print("Radio:  Manual Mode ");
  }
  //
  // Updates from Radio are active, print to LCD
  //
  else
  {
    //-----------------------------------
    // Indicate Radio Frequency
    //-----------------------------------
    virt_lcd_setCursor(0,0);
    virt_lcd_print("Radio:");
    virt_lcd_setCursor(17,0);
    virt_lcd_print(" Hz");
    virt_lcd_setCursor(6,0);
    // Pseudo-VFO mode
    if ((controller_settings.trx[controller_settings.radioprofile].radio == 10) && controller_settings.pseudo_vfo)
      display_frq(pseudo_vfo());
    else display_frq(running[ant].Frq);
  }
   
  //-----------------------------------
  // Indicate Tuned Frequency
  //-----------------------------------
  virt_lcd_setCursor(0,1);
  virt_lcd_print("Tuned:");
  virt_lcd_setCursor(17,1);
  virt_lcd_print(" Hz");
  virt_lcd_setCursor(6,1);
  display_frq(tunedFrq);
}

//
//-----------------------------------------------------------------------------
//      Manage Display
//-----------------------------------------------------------------------------
//
void lcd_display(void)
{
  static uint16_t screensave_timer;
  static int32_t  old_frq;

  //-----------------------------------
  // Update Screensaver timer and
  // turn Screensaver flag on and off 
  // based on Stepper activity
  //-----------------------------------
  //
  // Inhibit and reset Screensaver counting if deactivated
  if (SCREENSAVE_ACTIVATE == 0) 
  {
    screensave_timer = 0;
    flag.screensaver = false;
  }
  // Inhibit and reset Screensaver counting if in Menu
  else if (flag.config_menu) 
  {
    screensave_timer = 0;
    flag.screensaver = false;
  }
  // Reset Screensaver counter if new frequency
  else if (running[ant].Frq != old_frq)
  {
    screensave_timer = 0;
    flag.screensaver = false;
    old_frq = running[ant].Frq;    
  }
  // Is stepper inactive and screensaver off? 
  else if (!flag.frq_store && !flag.screensaver && !meter.power_detected)
  {
    // Count screensave timer and turn screensaver flag on if mature
    screensave_timer++;
    if (screensave_timer == SCREENSAVE_ACTIVATE)
    {
      flag.screensaver = true;
    }
  }
  // Turn screensave flag off and reset screensave_timer if Stepper goes active or power detected
  else if ( flag.screensaver && (flag.frq_store || meter.power_detected)) 
  {
    screensave_timer = 0;
    flag.screensaver = false;
  }

  //-----------------------------------
  // Print stuff to the LCD
  //-----------------------------------

  //-----------------------------------
  // Do nothing if currently in Menu
  // or displaying an announcement
  // ...unless swr.lcd_snapshot
  //-----------------------------------
  if ((flag.config_menu || (Menu_exit_timer != 0)) && !swr.lcd_snapshot)
  {
  }

  //-----------------------------------
  // Display Screensaver
  //-----------------------------------  
  else if (flag.screensaver)
  {
    screensaver();                                                  
  }
  
  #if PSWR_AUTOTUNE
  //-----------------------------------
  // Display Normal Power & SWR Meter
  //-----------------------------------  
  else if (meter.power_detected && !swr.tune && !flag.frq_store)
  { 
    lcd_display_power_and_swr("pk ", power_mw_pk);
  }

  //-----------------------------------
  // Display Power & SWR Meter during Manual Stepper Tune
  //----------------------------------- 
  else if (meter.power_detected && !swr.tune && flag.frq_store)
  { 
    lcd_display_power_and_swr_during_man_tune();    
  }
  #endif

  //-----------------------------------
  // Display Loop Controller data
  //-----------------------------------
  else
  {
    #if PSWR_AUTOTUNE
    //-----------------------------------
    // Print SWR Bargraph information in second line during SWR tune
    //-----------------------------------
    // If this is the last display after a successful SWR Tune, then show best values
    if (swr.lcd_snapshot)
    {
      measured_swr = snapshot_swr;
      swr_bar = snapshot_swr_bar;
    }
    if (swr.tune)                            // Business as usual
    {
      lcd_display_swr_tune();
    }
    #endif
    
    //-----------------------------------
    // Indicate Radio Frequency and Tuned Frequency in first two lines
    //----------------------------------- 
    if (!swr.tune)                           // Only print first two lines if not in Autotune
    {
      lcd_display_radio_frequency_data();
    }
    //-----------------------------------
    // Print Stepper Position data, range and status in lower two lines of LCD
    //----------------------------------- 
    lcd_display_stepper_data();
  }
  
  swr.lcd_snapshot = false;
}
