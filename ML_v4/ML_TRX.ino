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
// Transceiver Communiations  Routines, using UART (TTL or RS232)
//
// including:     Polling for Frequency
//                Get and Set Power Level
//                Get and Set Mode
//                TX on / TX off
//
// Used in conjunction with SWR-tune, to get current Power Level, get current Mode
// and then set a Low Power Level, AM-Mode and swithc to AM while tuning for low SWR
// and finally, restore to previous Power Level and mode.
//
// Works with:    ICOM CI-V
//                Elecraft K3/KX3
//                Kenwood TS440
//                Kenwood TS480/590/2000...
//                Yaeso FT-100
//                Yaesu FT-7x7  (not verified yet)
//                Yaesu FT-8x7
//                Yaesu FT-990
//                FT-1000MP
//                FT-1000MP Mk-V
//                Yaesu FT-450/950/1200/2000/3000/5000
//                TenTec, recent models
//                Pseudo-VFO.  No communications with an external radio. SW6 is
//                             used to swithc Encoder between Normal and Pseudo-VFO
//---------------------------------------------------------------------------------
//

// Globals for Transceiver
uint16_t trx_pwr=0;     // Power Control Leve, typically 0 - 255, taken over and lowered while SWR tune
uint16_t trx_mode=0;    // LSB, USB, CW, AM etc... AM while SWR tune

// Various Radio selection setups, where:
// 0  = ICOM CI-V Auto
// 1  = ICOM CI-V Poll
// 2  = Kenwood TS-440
// 3 =  Kenwood TS-870
//----------------------------
// 4  = Kenwood TS-480/TS-590/TS-2000
// 5  = Yaesu FT-100
// 6  = Yaesu FT-7X7 (747...)
// 7  = Yaesu FT-8x7 (817,847,857,897)
//---------------------------
// 8  = Yaesu FT-920
// 9  = Yaesu FT-990
// 10 = Yaesu FT-1000MP
// 11 = Yaesu FT-1000MP Mk-V
// 12 = Yaesu FT-450/950/1200/2000/3000/5000
// 13 = Elecraft K3/KX3 Auto
// 14 = Elecraft K3/KX3 Poll
// 15 = TenTec binary mode
// 16 = TenTec ascii mode
// 17 = Pseudo-VFO mode
                           
// The various default settings are fetched from ML.h
 
 const uint16_t default_uart_baud[] =
                        {  DEFAULT_ICOM_BAUD,
                           DEFAULT_ICOM_BAUD,
                           DEFAULT_KENWOOD_BAUD,
                           DEFAULT_KENWOOD_BAUD,
                           DEFAULT_KENWOOD_BAUD,
                           DEFAULT_FT100_BAUD,
                           DEFAULT_FT7X7_BAUD,
                           DEFAULT_FT8X7_BAUD,
                           DEFAULT_FT920_BAUD,
                           DEFAULT_FT990_BAUD,
                           DEFAULT_FT1000MP_BAUD,
                           DEFAULT_FT1000MPMKV_BAUD,
                           DEFAULT_FT450_BAUD,
                           DEFAULT_ELECRAFT_BAUD,
                           DEFAULT_ELECRAFT_BAUD,
                           DEFAULT_TENTEC_BAUD,
                           DEFAULT_TENTEC_BAUD,
                           DEFAULT_PSEUDOVFO_BAUD };
                           
  const uint16_t default_uart_mode[] =
                        {  DEFAULT_ICOM_MODE,
                           DEFAULT_ICOM_MODE,
                           DEFAULT_KENWOOD_MODE,
                           DEFAULT_KENWOOD_MODE,
                           DEFAULT_KENWOOD_MODE,
                           DEFAULT_FT100_MODE,
                           DEFAULT_FT7X7_MODE,
                           DEFAULT_FT8X7_MODE,
                           DEFAULT_FT920_MODE,
                           DEFAULT_FT990_MODE,
                           DEFAULT_FT1000MP_MODE,
                           DEFAULT_FT1000MPMKV_MODE,
                           DEFAULT_FT450_MODE,
                           DEFAULT_ELECRAFT_MODE,
                           DEFAULT_ELECRAFT_MODE,
                           DEFAULT_TENTEC_MODE,
                           DEFAULT_TENTEC_MODE,
                           DEFAULT_PSEUDOVFO_MODE };
                           
 const uint16_t valid_uart_config[] =
                        {  ICOM_CONFIG,
                           ICOM_CONFIG,
                           KENWOOD8N2_CONFIG,
                           KENWOOD_CONFIG,
                           KENWOOD_CONFIG,
                           FT100_CONFIG,
                           FT7X7_CONFIG,
                           FT8X7_CONFIG,
                           FT920_CONFIG,
                           FT990_CONFIG,
                           FT1000MP_CONFIG,
                           FT1000MPMKV_CONFIG,
                           FT450_CONFIG,
                           ELECRAFT_CONFIG,
                           ELECRAFT_CONFIG,
                           TENTEC_CONFIG,
                           TENTEC_CONFIG,
                           PSEUDOVFO_CONFIG };

 const uint16_t valid_uart_config_inv[] =
                        {  ICOM_CONFIG_INV,
                           ICOM_CONFIG_INV,
                           KENWOOD8N2_CONFIG_INV,
                           KENWOOD_CONFIG_INV,
                           KENWOOD_CONFIG_INV,
                           FT100_CONFIG_INV,
                           FT7X7_CONFIG_INV,
                           FT8X7_CONFIG_INV,
                           FT920_CONFIG_INV,
                           FT990_CONFIG_INV,
                           FT1000MP_CONFIG_INV,
                           FT1000MPMKV_CONFIG_INV,
                           FT450_CONFIG_INV,
                           ELECRAFT_CONFIG_INV,
                           ELECRAFT_CONFIG_INV,
                           TENTEC_CONFIG_INV,
                           TENTEC_CONFIG_INV,
                           PSEUDOVFO_CONFIG_INV };
                           
// BOOL for TRX serial port - TRUE = Asynchronous serial read mode
const int8_t async[] =
                        {  true,     // Icom
                           true,     // Icom
                           true,     // Kenwood TS440
                           true,     // Kenwood TS870
                           true,     // Kenwood TS480/590/2000...
                           false,    // Yaesu FT100       = Sync mode
                           false,    // Yaesu FT7x7       = Sync mode
                           false,    // Yaesu FT8x7       = Sync mode
                           false,    // Yaesu FT920       = Sync mode
                           false,    // Yaesu FT990       = Sync mode
                           false,    // Yaesu FT1000MP    = Sync mode
                           false,    // Yaesu FT1000MPmkV = Sync mode
                           true,     // Yaesu FT450+
                           true,     // Elecraft K3/KX3
                           true,     // Elecraft K3/KX3
                           false,    // Tentec Binary     = Sync mode
                           true,     // Tentec Ascii
                           false };
                           
// TRX serial port Poll Rate
const uint16_t poll_rate[] =
                        {  ICOM_POLL_RATE,
                           ICOM_POLL_RATE,
                           KENWOOD_POLL_RATE,
                           KENWOOD_POLL_RATE,
                           KENWOOD_POLL_RATE,
                           FT100_POLL_RATE,
                           FT7X7_POLL_RATE,
                           FT8X7_POLL_RATE,
                           FT920_POLL_RATE,
                           FT990_POLL_RATE,
                           FT1000MP_POLL_RATE,
                           FT1000MPMKV_POLL_RATE,
                           FT450_POLL_RATE,
                           ELECRAFT_POLL_RATE,
                           ELECRAFT_POLL_RATE,
                           TENTEC_POLL_RATE,
                           TENTEC_POLL_RATE,
                           PSEUDOVFO_POLL_RATE };

// Default TRX tune PowerLevel
const uint16_t default_plevel[] =
                        {  ICOM_TUNE_PWR,
                           ICOM_TUNE_PWR,
                           255,      // Not used
                           TS870_TUNE_PWR,
                           TS2000_TUNE_PWR,
                           255,      // Not used
                           255,      // Not used
                           255,      // Not used
                           255,      // Not used
                           255,      // Not used
                           255,      // Not used
                           255,      // Not used
                           FT2000_TUNE_PWR,
                           ELECRAFT_TUNE_PWR,
                           ELECRAFT_TUNE_PWR,
                           TENTEC_TUNE_PWR,
                           TENTEC_TUNE_PWR,
                           255    }; // Not used

// Message 'End" symbol
const char end_delimeter[] =
                        {  0xfd,     // ICOM end delimeter <fd>
                           0xfd,
                            ';',     // Kenwood - ;
                            ';',     // Kenwood - ;
                            ';',     // Kenwood - ;
                              0,     // Yaesu FT100       - Sync mode, not used
                              0,     // Yaesu FT7x7       - Sync mode, not used
                              0,     // Yaesu FT8x7       - Sync mode, not used
                              0,     // Yaesu FT920       - Sync mode, not used
                              0,     // Yaesu FT990       - Sync mode, not used
                              0,     // Yaesu FT1000MP    - Sync mode, not used
                              0,     // Yaesu FT1000MPmkV - Sync mode, not used
                            ';',     // Yaesu FT450... - ;
                            ';',     // Elecraft - ;
                            ';',
                              0,     // TenTec binary     - Sync mode, not used
                            0x0d,    // TenTec ascii      - <0d> = Carriage Return
                              0   };
                              
// TRX serial style, ascii = FALSE or HEX = TRUE.  Used for debug print to LCD
const uint16_t serial_hex[] =
                        {  ICOM_DEBUG,
                           ICOM_DEBUG,
                           KENWOOD_DEBUG,
                           KENWOOD_DEBUG,
                           KENWOOD_DEBUG,
                           FT100_DEBUG,
                           FT7X7_DEBUG,
                           FT8X7_DEBUG,
                           FT920_DEBUG,
                           FT990_DEBUG,
                           FT1000MP_DEBUG,
                           FT1000MPMKV_DEBUG,
                           FT450_DEBUG,
                           ELECRAFT_DEBUG,
                           ELECRAFT_DEBUG,
                           TENTEC_BIN_DEBUG,
                           TENTEC_ASCII_DEBUG,
                           PSEUDOVFO_DEBUG };

int8_t    async_uart_read;           // BOOL for TRX serial port - TRUE = Asynchronous serial read mode
int32_t   pseudovfo_frq;             // Pseudo VFO frequency
char      lcd_debug_line[82];        // Used to accumulate debug data to print to lcd
uint8_t   lcd_debug_pos = 0;         // Current position within lcd_debug_line


//
//-----------------------------------------------------------------------------------------
//      Print Newline to Serial Debug
//-----------------------------------------------------------------------------------------
//
void debug_println_serial(void)
{
  if (radio.debug_trx || radio.debug_hex)
  {
    Serial.print("\r\n");
  }    
}
//
//-----------------------------------------------------------------------------------------
//      Print Debug data to serial port
//-----------------------------------------------------------------------------------------
//
void debug_print_serial(const char *ch_in)
{
  if (radio.debug_trx) Serial.print(ch_in);       // ASCII style   
  if (radio.debug_hex)                            // HEX style
  {
    for (uint8_t x = 0; x< strlen(ch_in); x++)
    {
      sprintf(print_buf,"<%02x>", ch_in[x]);
      Serial.print(print_buf);
    }
  }  
}

//
//-----------------------------------------------------------------------------------------
//      Write Debug data to serial port
//-----------------------------------------------------------------------------------------
//
void debug_write_serial(char c)
{
  if (radio.debug_trx) Serial.print(c);          // ASCII style   
  if (radio.debug_hex)                           // HEX style
  {
     sprintf(print_buf,"<%02x>", c);
     Serial.print(print_buf);
  }  
}
//
//-----------------------------------------------------------------------------------------
//      Print Debug data to LCD
//-----------------------------------------------------------------------------------------
//
void debug_print_lcd(const char *ch_in)
{
  if (radio.debug_to_lcd)
  {
    if(serial_hex[controller_settings.trx[controller_settings.radioprofile].radio])
    {
      for (uint8_t x = 0; x< strlen(ch_in); x++)
      {
        sprintf(lcd_debug_line+2*lcd_debug_pos,"%02x", ch_in[x]);
        lcd_debug_pos++;
        if (lcd_debug_pos >= 10) lcd_debug_pos = 10;   // Go no further than end of line 
      }
      virt_lcd_print_scroll(lcd_debug_line);
    }
    else
    {
      virt_lcd_print_scroll(ch_in);
    }
  }
}
//
//-----------------------------------------------------------------------------------------
//      Write Debug data to LCD (accumulate and print on newline)
//-----------------------------------------------------------------------------------------
//
void debug_write_lcd(char in)
{
  if (radio.debug_to_lcd)
  {
    if(serial_hex[controller_settings.trx[controller_settings.radioprofile].radio])
    {
      sprintf(lcd_debug_line+2*lcd_debug_pos,"%02x", in);
      lcd_debug_pos++;
      if (lcd_debug_pos >= 10) lcd_debug_pos = 10; // Go no further than end of line 
    }
    else
    {
      lcd_debug_line[lcd_debug_pos++] = in;        // Add to string
      lcd_debug_line[lcd_debug_pos] = 0x00;        // End of current string      
      if (lcd_debug_pos >= 20) lcd_debug_pos = 20; // Go no further than end of line      
    }
  }
}
void debug_writeln_lcd(void)
{
  if (radio.debug_to_lcd)
  {
    virt_lcd_print_scroll(lcd_debug_line);
    lcd_debug_pos = 0;
  }
}

//
//---------------------------------------------------------------------------------
// Serial out wrappers - to facilitate redirection to USB, or
// USB debug of the communications between Controller and Transceiver
//---------------------------------------------------------------------------------
//
//
// Write a string to Transceiver.  If Debug, then write a string and newline to debug
void trx_print(const char *in_string)
{
  Uart.print(in_string);
  debug_print_serial(in_string);
  debug_println_serial();        // New line
  debug_print_lcd(in_string);    // Each call for LCD print automatically ends with a newline
}
//
// Write a char to Transceiver
//void trx_write(uint8_t in)
void trx_write(char in)
{
  Uart.write(in);
  debug_write_serial(in);
  debug_write_lcd(in);  
}
//
// Write a char to Transceiver.  If Debug, then write a char and newline to debug
void trx_writeln(char in)
{
  Uart.write(in);
  debug_write_serial(in);
  debug_println_serial();        // New line
  debug_write_lcd(in);  
  debug_writeln_lcd();   
}


char transceiver_in_string[2048]; // Incoming serial string to parse
//
//---------------------------------------------------------------------------------
// Some transceivers use binary data protocols which do not have any obvious 
// beginning and end of each message.  Collect incoming traffic to a circular buffer.
// The Transceiver poll functions have responsibility to zero the outstanding
// byte count in the circular buffer before each poll for new data.
// This also hopefully provides for better behaviour when in passthru mode
// when all sorts of traffic may be present.
//---------------------------------------------------------------------------------
//
//  
uint16_t char_pointer_in  = 0;    // Circular buffer pointer, last read character
uint16_t char_pointer_out = 0;    // Circular buffer pointer, last processed character
void transceiver_sync_read(void)
{
  while(Uart.available()>0)       // Are incoming characters waiting to be processed?
  {
    transceiver_in_string[char_pointer_in++] = Uart.read(); // Read the character from UART buffer
    if (char_pointer_in == 2048) char_pointer_in = 0;       // Wrap around at end.
  }
}
// Return number of chars available from Transceiver
uint16_t trx_available(void)
{
  uint16_t avail;
  if (char_pointer_in >= char_pointer_out) avail = char_pointer_in - char_pointer_out;
  else avail = char_pointer_in + (2048 - char_pointer_out);
  return avail;
}
// Clear incoming buffer
void trx_clear(void)
{
  char_pointer_out = char_pointer_in;
}

// Read chars from incoming circular buffer
uint8_t trx_read(void)
{
  char i;
  i = transceiver_in_string[char_pointer_out++];
  if (char_pointer_out == 2048) char_pointer_out = 0;       // Wrap around at end.

  // CAT Passthrough mode. Pass everything from serial (radio) to USB (computer)
  // Only do this if SWR tune is not in progress
  if (!swr.tune && controller_settings.trx[controller_settings.radioprofile].passthrough)
  {
    Serial.write(i);
  }
  // Debug stuff - Print received serial data from TRX to USB port  
  else                             // Cannot sent debug info to USB if passthrough
  {
   debug_write_serial(i);
  }
  debug_write_lcd(i); 
  return i;
}
//
// Read chars from Transceiver.  If Debug, then write newline to debug
uint8_t trx_readln(void)
{
  char i;
  i = transceiver_in_string[char_pointer_out++];
  if (char_pointer_out == 2048) char_pointer_out = 0;       // Wrap around at end.

  // CAT Passthrough mode. Pass everything from serial (radio) to USB (computer)
  // Only do this if SWR tune is not in progress
  if (!swr.tune && controller_settings.trx[controller_settings.radioprofile].passthrough)
  {
    Serial.write(i);
  }
  // Debug stuff - Print received serial data from TRX to USB port 
  else                             // Cannot sent debug info to USB if passthrough
  { 
    debug_write_serial(i);
    debug_println_serial();        // New line
  }
  debug_write_lcd(i);  
  debug_writeln_lcd();
  return i;
}


//
//---------------------------------------------------------------------------------
// Some Transceivers use ASCII protocols or protocols otherwise easy to parse,
// having a clear header/footer for each message.  However these transceivers
// may be broadcasting data on the serial port at any time. 
// Hence incoming traffic from these transceivers must be handled
// in an asynchronous manner, rather than just assuming that anything
// received is in response to a polled query by the Controller.
// Examples of this include ICOM CI-V, Elecraft K3/KX3...
// Also - if passthru mode, then all sorts of traffic may be present.
//---------------------------------------------------------------------------------
//
//  
int8_t transceiver_async_read(void)
{
  static uint16_t i;                            // message character count
  int8_t  Incoming;                             // Flag - Is there is a message ready to parse
  
  Incoming = false;
  
  while((Uart.available()>0) && !Incoming)      // Are incoming characters waiting to be processed?
  {
    if (i>= 255)                                // A long string of garbage detected
    {
      i = 0;                                    // set character count to zero
      transceiver_in_string[0] = 0;             // Terminate string with a zero
    }

    else                                        // Read the incoming message
    {
      transceiver_in_string[i] = Uart.read();   // Read the character from UART buffer
      transceiver_in_string[i+1] = 0;           // Terminate string with a zero

      // CAT Passthrough mode. Pass everything from serial (radio) to USB (computer)
      // Only do this if SWR tune is not in progress
      if (!swr.tune && controller_settings.trx[controller_settings.radioprofile].passthrough)
      {
        Serial.write(transceiver_in_string[i]);
      }
      // Debug stuff - Print received serial data from TRX to USB port  
      else                             // Cannot sent debug info to USB if passthrough
      {
        debug_write_serial(transceiver_in_string[i]);
      }
      debug_write_lcd(transceiver_in_string[i]);    
   
      // End of string - Indicate that we have stuff to process
      if (transceiver_in_string[i] == end_delimeter[controller_settings.trx[controller_settings.radioprofile].radio])
      {
        i = 0;                                  // set character count to zero
        Incoming = true;                        // Indicate that we have a new message to parse
        // Debug stuff - Print Newline to debug to denote end of message received
        debug_println_serial();
        debug_writeln_lcd();
      }
      else i++;                                 // character count +1
    }
  }
  return Incoming;
}


uint8_t icom_filter;                            // RX Filter setting, specific to ICOM
//---------------------------------------------------------------------------------
// ICOM style CI-V Read for either polled or auto
//---------------------------------------------------------------------------------
//
// This function uses input from uart_async_read()
void icom_parse_serial_input(void)
{
  uint32_t civ_value;	// CI-V data as a 32bit integer

  // Check for valid beginning of an incoming message
  // starts with <fe><fe><e0> (normal mode) or <fe><fe><00> (CIV Transceive Mode)
  if ( ((transceiver_in_string[0]==0xfe) && (transceiver_in_string[1]==0xfe)) &&
       ((transceiver_in_string[2]==0xe0) || (transceiver_in_string[2]==0x00)) )
  {                                          // start parsing
    // Check if this is indicated as a Frequency message
    // 0x00=Transfer operating frq, 0x03/0x05=Read/Write operating frq
    if ((transceiver_in_string[4] == 0x00) || (transceiver_in_string[4] == 0x03) || (transceiver_in_string[4] == 0x05))
    {
      // Convert BCD to value
      civ_value = (transceiver_in_string[5] & 0x0f);                                  // Hz
      civ_value += 10 * ((transceiver_in_string[5] & 0xf0) >> 4);                     // 10 x Hz

      civ_value += 100 *  (transceiver_in_string[6] & 0x0f);                          // 100 x Hz
      civ_value += 1000 * ((transceiver_in_string[6] & 0xf0) >> 4);                   // kHz

      civ_value += (uint32_t) 10000 * (transceiver_in_string[7] & 0x0f);              // 10 x kHz
      civ_value += (uint32_t) 100000 * ((transceiver_in_string[7] & 0xf0) >> 4);      // 100 x kHz

      civ_value += (uint32_t) 1000000 * (transceiver_in_string[8] & 0x0f);            // MHz
      civ_value += (uint32_t) 10000000 * ((transceiver_in_string[8] & 0xf0) >> 4);    // 10 x MHz

      civ_value += (uint32_t) 100000000 * (transceiver_in_string[9] & 0x0f);          // 100 x MHz
      civ_value += (uint32_t) 1000000000 * ((transceiver_in_string[9] & 0xf0) >> 4);  // GHz

      //
      // Update running frequency of the application
      //
      if ((civ_value > 1000000) && (civ_value < 31000000)) // Impose some sane boundaries
      {
        antenna_select(civ_value);                         // Antenna switchover, if frequency applies to the other antenna  
        running[ant].Frq = civ_value;
        radio.timer = true;	                               // Indicate that we are receiving frq data from Radio
        radio.online = true;  
      }
      radio.ack = true;
    }
    // Check if this is an incoming Power Level Indication message
    else if ((transceiver_in_string[4] == 0x14) && (transceiver_in_string[5] == 0x0a))
    {
      trx_pwr = 100 * transceiver_in_string[6];
      trx_pwr += transceiver_in_string[7];
      radio.pwr = true;                              // Indicate that we have successfully read power control level
      radio.ack = true;
    }
    // Check if this is an incoming Mode and Filter Indication message
    else if (transceiver_in_string[4] == 0x04)       // Read active Mode (LSB, USB etc)
    {
      trx_mode = transceiver_in_string[5];           // 0=LSB 1=USB 2=AM 3=CW... 
      icom_filter = transceiver_in_string[6];
      radio.mode = true;                             // Indicate that we have successfully read active mode
      radio.ack = true;
    }
    // Check if this is a Positive acknowlegement message
    else if (transceiver_in_string[4] == 0xfb)       // OK
    {
      radio.ack = true;
    }
    // Check if this is a Negative acknowlegement message
    else if (transceiver_in_string[4] == 0xfa)       // OK
    {
      radio.ack = false;
    }
  }
}
//
//---------------------------------------------------------------------------------
// ICOM CI-V style polling for frequency
//---------------------------------------------------------------------------------
//
void icom_request_frequency(void)
{
  // Transmit a request to read operating frequency
  trx_write(0xfe);
  trx_write(0xfe);
  trx_write(controller_settings.trx[controller_settings.radioprofile].ICOM_address);
  trx_write(0xe0);
  trx_write(0x03);
  trx_writeln(0xfd);
}
//
//---------------------------------------------------------------------------------
// Autotune stuff
//
//---------------------------------------------------------------------------------
// ICOM CI-V style Request Power
//---------------------------------------------------------------------------------
//
void icom_request_pwr(void)
{
 // Transmit a request to read RF Power Level setting
  trx_write(0xfe);
  trx_write(0xfe);
  trx_write(controller_settings.trx[controller_settings.radioprofile].ICOM_address);
  trx_write(0xe0);
  trx_write(0x14);    // Request Level
  trx_write(0x0a);    // level type = RF power
  trx_writeln(0xfd);
  delayloop(9);
}
//
//---------------------------------------------------------------------------------
// ICOM CI-V style Request Mode and Filter
//---------------------------------------------------------------------------------
//
void icom_request_mode(void)
{
 // Transmit a request to read ModeRF Power Level setting
  trx_write(0xfe);
  trx_write(0xfe);
  trx_write(controller_settings.trx[controller_settings.radioprofile].ICOM_address);
  trx_write(0xe0);
  trx_write(0x04);    // 04 = Request Mode and Filter
  trx_writeln(0xfd);
  delayloop(9);
}
//
//---------------------------------------------------------------------------------
// ICOM CI-V style Set Power
//---------------------------------------------------------------------------------
//
void icom_set_pwr(uint16_t pwr)
{
 // Set RF Power Level setting
  trx_write(0xfe);
  trx_write(0xfe);
  trx_write(controller_settings.trx[controller_settings.radioprofile].ICOM_address);
  trx_write(0xe0);
  trx_write(0x14);    // Set Level
  trx_write(0x0a);    // level type = RF power
  trx_write(pwr/100); // High byte (bcd)
  trx_write(pwr%100); // Low byte (bcd)
  trx_writeln(0xfd);
  delayloop(9);
}
//
//---------------------------------------------------------------------------------
// ICOM CI-V style Set Mode
//---------------------------------------------------------------------------------
//
void icom_set_mode(uint8_t mode, uint8_t filter)
{
 // Set RF Power Level setting
  trx_write(0xfe);
  trx_write(0xfe);
  trx_write(controller_settings.trx[controller_settings.radioprofile].ICOM_address);
  trx_write(0xe0);
  trx_write(0x01);    // 01 = Set Mode and FilterLevel
  trx_write(mode);    // Mode
  trx_write(filter);  // Filter
  trx_writeln(0xfd);
  delayloop(9);
}
//
//---------------------------------------------------------------------------------
// ICOM CI-V style Set Transmit Mode
//---------------------------------------------------------------------------------
//
void icom_set_tx(uint8_t tx)
{
 // Set Transmit on or off
  trx_write(0xfe);
  trx_write(0xfe);
  trx_write(controller_settings.trx[controller_settings.radioprofile].ICOM_address);
  trx_write(0xe0);
  trx_write(0x1c);    // Set Transmit on/off
  trx_write(0x00);
  trx_write(tx);      // 1 = TX, 0 = RX
  trx_writeln(0xfd);
  delayloop(9);
}


//
//---------------------------------------------------------------------------------
// Yaesu FT-100 style polling for frequency and Mode+Filter
//---------------------------------------------------------------------------------
//
void ft100_parse_serialinput(void)
{
  uint32_t incoming;
  
  //char *pEnd;
  uint16_t waiting;                     // Number of chars waiting in receive buffer

  // Find out how many characters are waiting to be read. 
  waiting = trx_available();
  if (waiting >= 16)                    // Incoming string of correct length detected
  {
    trx_read();                         // Flush the first byte (band information)
    
    // Decode frequency content of the incoming message
    incoming =  trx_read()<<24;
    incoming += trx_read()<<16;
    incoming += trx_read()<<8;
    incoming += trx_read();
    incoming =  incoming * 1.25;
    
    if ((incoming > 1000000) && (incoming < 31000000))// Impose some sane boundaries
    {  
      trx_mode = trx_readln();          // Read Transceiver Mode (LSB=0... AM=4...) and Filter information
      radio.mode = true;                // Indicate that we have successfully read active mode

      antenna_select(incoming);         // Antenna switchover, if frequency applies to the other antenna  
      running[ant].Frq = incoming;
      radio.timer = true;               // Indicate that we are receiving frq data from Radio
      radio.online = true;  
    }
  }
  trx_clear();                          // Flush/Discard any received data that has not been read.
}

void ft100_request_frequency(void)
{
  ft100_parse_serialinput();            // Grab input from latest poll

  // Transmit a Status request to Yaesu FT100
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_writeln(0x10);
}
//
//---------------------------------------------------------------------------------
// Autotune stuff
//
//  void ft100_request_pwr(void)    // Not available
//  void ft100_set_pwr(uint8_t pwr) // Not available
//
//---------------------------------------------------------------------------------
// Yaesu FT100 style Set Mode and Filter
//---------------------------------------------------------------------------------
//
void ft100_set_mode(uint8_t mode)
{
  // Transmit a Set Mode command to Yaesu FT100
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(mode & 0x0f);
  trx_writeln(0x0c);
  delayloop(100);
  // Transmit a Set Filter command to Yaesu FT100
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write((mode & 0xf0)>>4);
  trx_writeln(0x8c);
  delayloop(100);
}
//
//---------------------------------------------------------------------------------
// Yaesu FT100 style Set Transmit Mode
//---------------------------------------------------------------------------------
//
void ft100_set_tx(void)
{
  // Switch to Transmit (PTT On)
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x01);
  trx_writeln(0x0f);
  delayloop(100);
}
void ft100_set_rx(void)
{
  // Switch to Receive (PTT Off)
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_writeln(0x0f);
  delayloop(100);
}


//
//---------------------------------------------------------------------------------
// Yaesu FT-7x7 style polling for frequency and Mode+Filter
//---------------------------------------------------------------------------------
//
void ft7x7_parse_serialinput(void)
{
  uint8_t i, x;
  uint32_t incoming;
  
  //char *pEnd;
  uint16_t waiting;                     // Number of chars waiting in receive buffer

  // Find out how many characters are waiting to be read. 
  waiting = trx_available();
  if (waiting >= 300)                   // Incoming string of correct length detected (345 bytes)
  {
    trx_read();                         // Flush the first byte (status information)
    x = trx_read();
    if (x == 0)                         // Should contain a zero
    {
    
      // Decode frequency content of the incoming message
      x = trx_read();
      incoming = 10000000 * ((x & 0xf0) >> 4); // 10 x MHz
      incoming += 1000000 * (x & 0x0f);        // 1 x MHz    
      x = trx_read();
      incoming += 100000 * ((x & 0xf0) >> 4);  // 100 x kHz
      incoming += 10000 * (x & 0x0f);          // 10 x kHz
      x = trx_read();
      incoming += 1000 * ((x & 0xf0) >> 4);    // 1 x kHz
      incoming += 100 * (x & 0x0f);            // 100 x Hz
      x = trx_read();
      incoming += 10 * ((x & 0xf0) >> 4);      // 10 x Hz
      incoming += 1 * (x & 0x0f);              // 1 x Hz
      
      if ((incoming > 1000000) && (incoming < 31000000))// Impose some sane boundaries
      {  
        for (i = 6; i < 22; i++) trx_read();   // We have valid data, read until mode
        trx_mode = trx_read();    // Read Transceiver Mode and Filter information (FM=1,AM=2,CW=4,USB=8,LSB=16
                                  // and Narrow Filter mode is 128)
        radio.mode = true;        // Indicate that we have successfully read active mode


        antenna_select(incoming);              // Antenna switchover, if frequency applies to the other antenna  
        running[ant].Frq = incoming;
        radio.timer = true;                    // Indicate that we are receiving frq data from Radio
        radio.online = true;  
      }
    }
  }
  trx_clear();                                // Flush/Discard any received data that has not been read.
}

void ft7x7_request_status(void)
{
  ft7x7_parse_serialinput();            // Grab input from latest poll

  // Transmit a Status request to Yaesu FT7x7
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_writeln(0x10);
}
//
//---------------------------------------------------------------------------------
// Autotune stuff
//
//  void ft7x7_request_pwr(void)    // Not available
//  void ft7x7_set_pwr(uint8_t pwr) // Not available
//
//---------------------------------------------------------------------------------
// Yaesu FT7x7 style Set Mode and Filter
//---------------------------------------------------------------------------------
//
void ft7x7_set_mode(uint8_t mode)
{
  uint8_t mode_out = 0;
  // mode structure is:FM=1,AM=2,CW=4,USB=8,LSB=16 and Narrow Filter mode is 128
  // has to be translated to: 00=LSB,01=USB,02=CWWide,03=CWNarrow,04=AMWide,05=AMNarrow,06=FMWide,07=FMNarrow
  if      ((mode & 0x3f) == 1)  mode_out = 6;  // FM Wide
  else if ((mode & 0x3f) == 2)  mode_out = 4;  // AM Wide
  else if ((mode & 0x3f) == 4)  mode_out = 2;  // CW Wide
  else if ((mode & 0x3f) == 8)  mode_out = 1;  // USB
  else if ((mode & 0x3f) == 16) mode_out = 0;  // LSB
  
  // Switch to Narrow mode for CW, AM and FM if Filter is Narrow
  if ((mode & 0x80) && (mode_out > 2)) mode_out += 1;  
  
  // Transmit a Set Mode command to Yaesu FT7x7
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(mode_out);
  trx_writeln(0x0c);
  delayloop(100);
}
//
//---------------------------------------------------------------------------------
// Yaesu FT7x7 style Set Transmit Mode
//---------------------------------------------------------------------------------
//
void ft7x7_set_tx(void)
{
  // Switch to Transmit (PTT On)
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x01);
  trx_writeln(0x0f);
  delayloop(100);
}
void ft7x7_set_rx(void)
{
  // Switch to Receive (PTT Off)
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_writeln(0x0f);
  delayloop(100);
}


//
//---------------------------------------------------------------------------------
// Yaesu FT-817 / FT-847 / FT-857 / FT-897 style polling for frequency and mode
//---------------------------------------------------------------------------------
//
void ft8x7_parse_serialinput(void)
{
  uint8_t x;
  uint32_t incoming;

  uint16_t waiting;                     // Number of chars waiting in receive buffer

  // Find out how many characters are waiting to be read. 
  waiting = trx_available();
  if (waiting >= 5)                     // Incoming string of correct length detected
  { 
    // Decode frequency content of the incoming message
    x = trx_read();
    incoming = 100000000 * ((x & 0xf0) >> 4); // 100 x MHz
    incoming += 10000000 * (x & 0x0f);        // 10 x MHz    
    x = trx_read();
    incoming += 1000000 * ((x & 0xf0) >> 4);  // 1 x MHz
    incoming += 100000 * (x & 0x0f);          // 100 x kHz
    x = trx_read();
    incoming += 10000 * ((x & 0xf0) >> 4);    // 10 x kHz
    incoming += 1000 * (x & 0x0f);            // 1 x kHz
    x = trx_read();
    incoming += 100 * ((x & 0xf0) >> 4);      // 100 x Hz
    incoming += 10 * (x & 0x0f);              // 10 x Hz
    
    
    if ((incoming > 1000000) && (incoming < 31000000))  // Impose some sane boundaries
    {
      trx_mode = trx_read();                  // Read Transceiver Mode (LSB=0... AM=4...)
      radio.mode = true;                      // Indicate that we have successfully read active mode
    
      antenna_select(incoming);               // Antenna switchover, if frequency applies to the other antenna  
      running[ant].Frq = incoming;
      radio.timer = true;                      // Indicate that we are receiving frq data from Radio
      radio.online = true;  
    }
  }
  trx_clear();                                // Flush/Discard any received data that has not been read.
}

void ft8x7_request_frequency(void)
{
  ft8x7_parse_serialinput();            // Grab input from latest poll

  // Transmit a Status request to Yaesu FT8x7
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_writeln(0x03);
}
//
//---------------------------------------------------------------------------------
// Autotune stuff
//
//  void ft8x7_request_pwr(void)    // Not available
//  void ft8x7_set_pwr(uint8_t pwr) // Not available
//
//---------------------------------------------------------------------------------
// Yaesu FT8x7 style Set Mode
//---------------------------------------------------------------------------------
//
void ft8x7_set_mode(uint8_t mode)
{
  // Transmit a Set Mode command to Yaesu FT8x7
  trx_write(mode);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_writeln(0x07);
  delayloop(100);
}
//
//---------------------------------------------------------------------------------
// Yaesu FT8x7 style Set Transmit Mode
//---------------------------------------------------------------------------------
//
void ft8x7_set_tx(void)
{
  // Switch to Transmit (PTT On)
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_writeln(0x08);
  delayloop(100);
}
void ft8x7_set_rx(void)
{
  // Switch to Receive (PTT Off)
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_writeln(0x88);
  delayloop(100);
}


//
//---------------------------------------------------------------------------------
// Yaesu FT-920 style polling for frequency and Mode+Filter
//---------------------------------------------------------------------------------
//
void ft920_parse_serialinput(void)
{
  uint8_t  x;
  uint32_t incoming;
  uint8_t  byte5;                       // Mode and Filter information to piece
                                        // together a trx_mode information  
  //char *pEnd;
  uint16_t waiting;                     // Number of chars waiting in receive buffer

  // Find out how many characters are waiting to be read. 
  waiting = trx_available();
  if (waiting >= 28)                   // Incoming string of correct length detected (2x14 bytes)
  {
    trx_read();                         // Flush the first byte (status information)
    
    // Decode frequency content of the incoming message
    x = trx_read();
    incoming = 10000000 * ((x & 0xf0) >> 4); // 10 x MHz
    incoming += 1000000 * (x & 0x0f);        // 1 x MHz    
    x = trx_read();
    incoming += 100000 * ((x & 0xf0) >> 4);  // 100 x kHz
    incoming += 10000 * (x & 0x0f);          // 10 x kHz
    x = trx_read();
    incoming += 1000 * ((x & 0xf0) >> 4);    // 1 x kHz
    incoming += 100 * (x & 0x0f);            // 100 x Hz
    x = trx_read();
    incoming += 10 * ((x & 0xf0) >> 4);      // 10 x Hz
    incoming += 1 * (x & 0x0f);              // 1 x Hz
    
    if ((incoming > 1000000) && (incoming < 31000000))// Impose some sane boundaries
    {        
      trx_read();               // Read past Clarifier data and until mode
      trx_read();

      byte5 = trx_read();       // Read Transceiver Mode and Filter information (SSB=0b000, CWF=0b001, AM=0b010, FM=0b011,
                                // Data LSB=0b100, Data USB=0b101, Data FM=0b110
                                // USB=0x40
                                // and Narrow Filter mode is 0x80)

      // Decipher above into a trx_mode byte compatible with the set-op-mode command (0x0c)
      if ((byte5 & 0b111) == 0b000)     // LSB and USB
      {
        if (byte5 & 0x40)               // USB
          trx_mode = 0x01;
        else trx_mode = 0x00;           // LSB
        if (byte5 & 0x80)               // Wide filter
          trx_mode += 0x80;
      }
      else if ((byte5 & 0b111) == 0b001)// CW
      {
        if (byte5 & 0x40)               // CW USB
          trx_mode = 0x02;
        else trx_mode = 0x03;           // CW LSB
        if (byte5 & 0x80)               // Wide filter
          trx_mode += 0x80;
      }
      else if ((byte5 & 0b111) == 0b010)// AM
      {
        trx_mode = 0x04;                // AM
        if (byte5 & 0x80)               // Wide filter
          trx_mode += 0x80;
      }
      else if ((byte5 & 0b111) == 0b011)// FM
      {
        trx_mode = 0x07;                // FM narrow
        if (byte5 & 0x80)               // Wide filter
        {
          trx_mode = 0x06;              // FM wide
          trx_mode += 0x80;
        }
      }
      else if ((byte5 & 0b111) == 0b100)// Data LSB
      {
        trx_mode = 0x09;                // Data LSB
        if (byte5 & 0x80)               // Wide filter
          trx_mode += 0x80;
      }
      else if ((byte5 & 0b111) == 0b101)// Data USB
      {
        trx_mode = 0x0a;                // Data USB
        if (byte5 & 0x80)               // Wide filter
          trx_mode += 0x80;
      }            
      else if ((byte5 & 0b111) == 0b110)// Data FM
      {
        trx_mode = 0x0b;                // Data FM
        if (byte5 & 0x80)               // Wide filter
          trx_mode += 0x80;
      }          
                                
      radio.mode = true;        // Indicate that we have successfully read active mode

      antenna_select(incoming);              // Antenna switchover, if frequency applies to the other antenna  
      running[ant].Frq = incoming;
      radio.timer = true;                    // Indicate that we are receiving frq data from Radio
      radio.online = true;  
    }
  }
  trx_clear();                                // Flush/Discard any received data that has not been read.
}

void ft920_request_status(void)
{
  ft920_parse_serialinput();            // Grab input from latest poll

  // Transmit a Status request to Yaesu FT920
  trx_write(0x00);  // P4
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x02);  // P1
  trx_writeln(0x10);
}
//
//---------------------------------------------------------------------------------
// Autotune stuff
//
//  void ft920_request_pwr(void)    // Not available
//  void ft920_set_pwr(uint8_t pwr) // Not available
//
//---------------------------------------------------------------------------------
// Yaesu FT920 style Set Mode and Filter
//---------------------------------------------------------------------------------
//
void ft920_set_mode(uint8_t mode)
{
  // Transmit a Set Mode command to Yaesu FT920
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(mode & 0x0f);
  trx_writeln(0x0c);
  delayloop(100);
  // Transmit a Set Filter command to Yaesu FT920
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write((mode & 0x80)>>7);
  trx_writeln(0x8c);
  delayloop(100);
}

//
//---------------------------------------------------------------------------------
// Yaesu FT920 style Set Transmit Mode (undocumented ???)
//---------------------------------------------------------------------------------
//
void ft920_set_tx(void)
{
  // Switch to Transmit (PTT On)
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x01);
  trx_writeln(0x0f);
  delayloop(100);
}
void ft920_set_rx(void)
{
  // Switch to Receive (PTT Off)
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_writeln(0x0f);
  delayloop(100);
}


//
//---------------------------------------------------------------------------------
// Yaesu FT-990 style polling for frequency and Mode+Filter
//---------------------------------------------------------------------------------
//
void ft990_parse_serialinput(void)
{
  uint32_t incoming;
  uint16_t waiting;                     // Number of chars waiting in receive buffer
  uint8_t  byte7;                       // Mode and Filter information to piece
  uint8_t  byte8;                       // together a trx_mode information
  //uint8_t  byte9;                     // As per FT990 Manual, page 42
  //uint8_t  byte10;
  //uint8_t  byte11;
  //uint8_t  byte12;

  // Find out how many characters are waiting to be read. 
  waiting = trx_available();
  if (waiting >= 16)                    // Incoming string of correct length detected
  {
    for (uint8_t x=0;x<5;x++)           // Flush the first 5 bytes (flags and band information)
      trx_read();                         // Flush the first byte (band information)
    
    //trx_read();                         // Flush the first byte (band information, Rom v 1.3 and higher)
                                          // if command is <00><00><00><02><10>, see ft990_parse_serialinput()
          
    // Decode frequency content contained in bytes 1 - 3 of the incoming message
    incoming =  trx_read()<<16;
    incoming += trx_read()<<8;
    incoming += trx_read();
    incoming =  incoming * 10;
    
    if ((incoming > 1000000) && (incoming < 31000000))// Impose some sane boundaries
    {  
      trx_read();                       // Skip Clarifier data
      trx_read();
      trx_read();
      byte7  = trx_read();              // Read Transceiver Mode (LSB=0... AM=4...) and Filter information
      byte8  = trx_read();              // Read (current?) IF Filter data
      trx_read();                       // Read last selected SSB filter
      trx_read();                       // Read last selected CW filter (not used)
      trx_read();                       // Read last selected RTTY filter (not used)
      trx_read();                       // Read last selected PKT filter (not used)

      // Decipher above into a trx_mode byte compatible with the set-op-mode command (0x0c)
      if (byte7 <= 0x01)                // 0 and 1 = LSB and USB
      {
        trx_mode = byte7 + (byte8<<8);
      }
      else if (byte7 == 0x02)           // CW
      {
        if (byte8 & 0x02)               // bit 2 set, either 500 or 250Hz filter
        {
          trx_mode = 0x03;
        }
        else trx_mode = 0x02;
        trx_mode = trx_mode + (byte8<<8);
      }
      else if (byte7 == 0x03)           // AM
      {
        if (byte8 == 0x00) trx_mode = 0x05; // AM 2.4kHz
        else trx_mode = 0x06;               // AM 6 kHz
        trx_mode = trx_mode + (byte8<<8);
      }
      else if (byte7 == 0x04)           // FM
      {
        if (byte8 == 0x06)              // 6 kHz filter
        {
          trx_mode = 0x07;              // FM Wide (wild guess)
        }
        else trx_mode = 0x06;
        trx_mode = trx_mode + (byte8<<8);
      }
      else if (byte7 == 0x05)           // RTTY
      {
        if (byte8 & 0x80)               // Reverse RTTY - not sure if this is correct
        {
          trx_mode = 0x09;              // RTTYusb
        }
        else trx_mode = 0x08;           // RTTYlsb
       trx_mode = trx_mode + (byte8<<8);
      }
      else if (byte7 == 0x06)           // PKT
      {
        if (byte8 & 0x80)               // Reverse PKT - not sure if this is correct
        {
          trx_mode = 0x0b;              // PKTfm
        }
        else trx_mode = 0x0a;           // PKTlsb
       trx_mode = trx_mode + (byte8<<8);
      }      

      radio.mode = true;                // Indicate that we have successfully read active mode

      antenna_select(incoming);         // Antenna switchover, if frequency applies to the other antenna  
      running[ant].Frq = incoming;
      radio.timer = true;                // Indicate that we are receiving frq data from Radio
      radio.online = true;  
    }
    // Debug lasdjflaskjflas;jfla;sdjflaskjflaskjflaskjals;kjfalsjkfaslkjfasldkjasljfasldkjfasljfsadljfasdljfasldkfjasldjfalsdjflasdjfsladkjflasdjklfsadfjals
    else
    {
      Serial.print("Error Decode ");
      Serial.println(incoming);
    }
  }
  trx_clear();                          // Flush/Discard any received data that has not been read.
}

void ft990_request_frequency(void)
{
  // If not Manual Mode
  if (poll_rate[controller_settings.trx[controller_settings.radioprofile].radio] != 9999)
    ft990_parse_serialinput();          // Grab input from latest poll

  // Transmit a Status request to Yaesu FT990
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  //trx_write(0x02);                      // Request Op Data (16 bytes, only ROM version 1.3 and above)
  trx_write(0x00);                      // Request Op Data (1508 bytes)
  trx_writeln(0x10);                    // Request Status Update from TRX

  // If Manual Mode
  if (poll_rate[controller_settings.trx[controller_settings.radioprofile].radio] == 9999)
  {
    delayloop(4000);
    ft990_parse_serialinput();          // Grab input from latest poll    
  }
}
//
//---------------------------------------------------------------------------------
// Autotune stuff
//
//  void ft990_request_pwr(void)    // Not available
//  void ft990_set_pwr(uint8_t pwr) // Not available
//
//---------------------------------------------------------------------------------
// Yaesu FT990 style Set Mode and Filter
//---------------------------------------------------------------------------------
//
void ft990_set_mode(uint16_t mode)
{
  // Transmit a Set Mode command to Yaesu FT990
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(mode & 0x00ff);
  trx_writeln(0x0c);
  delayloop(100);
  // Transmit a Set Filter command to Yaesu FT990
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write((mode & 0xff00)>>8);
  trx_writeln(0x8c);
  delayloop(100);
}
//
//---------------------------------------------------------------------------------
// Yaesu FT990 style Set Transmit Mode
//---------------------------------------------------------------------------------
//
void ft990_set_tx(void)
{
  // Switch to Transmit (PTT On)
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x01);
  trx_writeln(0x0f);
  delayloop(100);
}
void ft990_set_rx(void)
{
  // Switch to Receive (PTT Off)
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_writeln(0x0f);
  delayloop(100);
}
//
//---------------------------------------------------------------------------------
// Yaesu FT990 debug stuff
//---------------------------------------------------------------------------------
//
void ft990_read_meter(void)
{
  // Debug Command - to trigger a response
  uint16_t waiting;
  trx_clear();
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_writeln(0xf7);

  delayloop(300);

  waiting = trx_available();
  for (uint16_t x = 0; x<waiting; x++)
  {
    sprintf(print_buf,"<%02x>", trx_read());
    Serial.print(print_buf);
  }
}



//
//---------------------------------------------------------------------------------
// Yaesu FT-1000MP style polling for frequency and Mode+Filter
//---------------------------------------------------------------------------------
//
void ft1000MP_parse_serialinput(void)
{
  uint8_t  x;
  uint32_t incoming;
  uint16_t waiting;                        // Number of chars waiting in receive buffer
  uint8_t  byte7;                          // Mode and Filter information to piece
  uint8_t  byte8;                          // together a trx_mode information
  bool     Reverse;                        // Decipher if Reverse mode (f.instance LSB above 10 MHz)

  // Find out how many characters are waiting to be read. 
  waiting = trx_available();
  if (waiting >= 16)                       // Incoming string of correct length detected
  {
    trx_read();                            // Flush the first byte (band information)
    
    // Decode frequency content contained in bytes 1 - 4 of the incoming message
    x = trx_read();
    incoming = 100 * ((x & 0xf0) >> 4);       // 100 x Hz
    incoming += 10 * (x & 0x0f);              // 10 x Hz
    x = trx_read();
    incoming += 10000 * ((x & 0xf0) >> 4);    // 10 x kHz
    incoming += 1000 * (x & 0x0f);            // 1 x kHz    
    x = trx_read();
    incoming += 1000000 * ((x & 0xf0) >> 4);  // 1 x MHz
    incoming += 100000 * (x & 0x0f);          // 100 x kHz
    x = trx_read();
    incoming += 100000000 * ((x & 0xf0) >> 4);// 100 x MHz
    incoming += 10000000 * (x & 0x0f);        // 10 x MHz    
    
    if ((incoming > 1000000) && (incoming < 31000000))// Impose some sane boundaries
    {  
      trx_read();                          // Skip Clarifier data in bytes 5 and 6
      trx_read();
      byte7  = trx_read() & 0x07;          // Read Transceiver Mode (LSB=0... AM=4...) and Filter information
      byte8  = trx_read();                 // Read IF Filter data
      
      // Decipher above into a trx_mode byte compatible with the set-op-mode command (0x0c)
      if (byte7 <= 0x01)                   // 0 and 1 = LSB and USB
      {
        trx_mode = byte7 + (byte8<<8);
      }
      else if (byte7 == 0x02)              // CW
      {
        // Determine if Reverse CW mode
        if      (((byte8 & 0x80)>0) && (incoming >= 10000000)) Reverse = true;
        else if (((byte8 & 0x80)>0) && (incoming <  10000000)) Reverse = true;
        else Reverse = false;

        if (Reverse) trx_mode = 0x03;      // CWR
        else trx_mode = 0x02;              // CW
        trx_mode = trx_mode + (byte8<<8);
      }
      else if (byte7 == 0x03)              // AM
      {
        if (byte8 & 0x80) trx_mode = 0x05; // AM Sync
        else trx_mode = 0x04;              // AM
        trx_mode = trx_mode + (byte8<<8);
      }
      else if (byte7 == 0x04)              // FM
      {
        if ((byte8 & 0x77) == 0) trx_mode = 0x07; // FM Wide (wild guess)
        else trx_mode = 0x06;
        trx_mode = trx_mode + (byte8<<8);
      }
      else if (byte7 == 0x05)              // RTTY
      {
        if (byte8 & 0x80) trx_mode = 0x08; // RTTYlsb
        else trx_mode = 0x09;              // RTTYusb
        trx_mode = trx_mode + (byte8<<8);
      }
      else if (byte7 == 0x06)              // PKT
      {
        if (byte8 & 0x80) trx_mode = 0x0b; // PKTfm
        else trx_mode = 0x0a;              // PKTlsb
        trx_mode = trx_mode + (byte8<<8);
      }      

      radio.mode = true;                   // Indicate that we have successfully read active mode

      antenna_select(incoming);            // Antenna switchover, if frequency applies to the other antenna  
      running[ant].Frq = incoming;
      radio.timer = true;                  // Indicate that we are receiving frq data from Radio
      radio.online = true;  
    }
  }
  trx_clear();                             // Flush/Discard any received data that has not been read.
}

void ft1000MP_request_frequency(void)
{
  ft1000MP_parse_serialinput();         // Grab input from latest poll

  // Transmit a Status request to Yaesu FT100
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x02);                      // Request subcategory Op Data (16 bytes)
  trx_writeln(0x10);                    // Request Status Update from TRX
}
//
//---------------------------------------------------------------------------------
// Autotune stuff
//
//  void ft1000MP_request_pwr(void)     // Not available
//  void ft1000MP_set_pwr(uint8_t pwr)  // Not available
//
//---------------------------------------------------------------------------------
// Yaesu FT1000MP style Set Mode and Filter
//---------------------------------------------------------------------------------
//
void ft1000MP_set_mode(uint16_t mode)
{
  uint8_t tmp;
  uint8_t if8m2, if455;   // Intermediate frequency filters at 8.2 MHz and 455 kHz
  uint8_t fltbyte=0;      // filter information to send to transceiver
  
  // Transmit a Set Mode command to Yaesu FT1000
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(mode & 0x00ff);
  trx_writeln(0x0c);
  delayloop(100);

  // Transmit a Set Filter command to Yaesu FT1000MP for 455 kHz IF
  tmp = (mode & 0xff00)>>8;
  if455 = tmp & 0x0f;     // Isolate 455kHz info
  if      (if455 == 0x00) fltbyte = 0x84; // Thru / 6 kHz
  else if (if455 == 0x01) fltbyte = 0x80; // 2.4 kHz
  else if (if455 == 0x02) fltbyte = 0x81; // 2.0 kHz
  else if (if455 == 0x03) fltbyte = 0x82; // 500 Hz
  else if (if455 == 0x04) fltbyte = 0x83; // 250 Hz
  trx_write(0x01);        // Write 455 kHz filter info to Radio
  trx_write(0x00);
  trx_write(0x00);
  trx_write(fltbyte);
  trx_writeln(0x8c);
  delayloop(100);
  
  // Transmit a Set Filter command to Yaesu FT1000MP  for 8.2 MHz IF
  tmp = (mode & 0xff00)>>12;
  if8m2 = tmp & 0x0f;       // Isolate 8.2 MHz filter info
  if      (if8m2 == 0x00) fltbyte = 0x04; // Thru / 6 kHz
  else if (if8m2 == 0x01) fltbyte = 0x00; // 2.4 kHz
  else if (if8m2 == 0x02) fltbyte = 0x01; // 2.0 kHz
  else if (if8m2 == 0x03) fltbyte = 0x02; // 500 Hz
  else if (if8m2 == 0x04) fltbyte = 0x03; // 250 Hz
  trx_write(0x01);      // Write 8.2 MHz filter info to Radio
  trx_write(0x00);
  trx_write(0x00);
  trx_write(fltbyte);
  trx_writeln(0x8c);
  delayloop(100);
}
//
//---------------------------------------------------------------------------------
// Yaesu FT1000MP style Set Transmit Mode
//---------------------------------------------------------------------------------
//
void ft1000MP_set_tx(void)
{
  // Switch to Transmit (PTT On)
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x01);
  trx_writeln(0x0f);
  delayloop(100);
}
void ft1000MP_set_rx(void)
{
  // Switch to Receive (PTT Off)
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_writeln(0x0f);
  delayloop(100);
}


//
//---------------------------------------------------------------------------------
// Yaesu FT-1000MP MK-V style polling for frequency and Mode+Filter
//---------------------------------------------------------------------------------
//
void ft1000MPmkV_parse_serialinput(void)
{
  uint32_t incoming;
  uint16_t waiting;                        // Number of chars waiting in receive buffer
  uint8_t  byte7;                          // Mode and Filter information to piece
  uint8_t  byte8;                          // together a trx_mode information
  bool     Reverse;                        // Decipher if Reverse mode (f.instance LSB above 10 MHz)

  // Find out how many characters are waiting to be read. 
  waiting = trx_available();
  if (waiting >= 16)                       // Incoming string of correct length detected
  {
    trx_read();                            // Flush the first byte (band information)
    
    // Decode frequency content contained in bytes 1 - 4 of the incoming message
    incoming =  trx_read()<<24;
    incoming += trx_read()<<16;
    incoming += trx_read()<<8;
    incoming += trx_read();
    
    // Whoops, description in Mk-V manual is incorrect, see discussion at:
    //                 http://www.dxzone.com/cgi-bin/dir/jump2.cgi?ID=4588
    incoming =  incoming / 0x10;           // Shift down by 4 bits. Confirmed by user report
    
    incoming =  incoming * 10;             // Original resolution in 10Hz, we want 1Hz
    
    if ((incoming > 1000000) && (incoming < 31000000)) // Impose some sane boundaries
    {  
      trx_read();                          // Skip Clarifier data in bytes 5 and 6
      trx_read();
      byte7  = trx_read() & 0x07;          // Read Transceiver Mode (LSB=0... AM=4...) and Filter information
      byte8  = trx_read();                 // Read IF Filter data
      
      // Decipher above into a trx_mode byte compatible with the set-op-mode command (0x0c)
      if (byte7 <= 0x01)                   // 0 and 1 = LSB and USB
      {
        trx_mode = byte7 + (byte8<<8);
      }
      else if (byte7 == 0x02)              // CW
      {
        // Determine if Reverse CW mode
        if      (((byte8 & 0x80)>0) && (incoming >= 10000000)) Reverse = true;
        else if (((byte8 & 0x80)>0) && (incoming <  10000000)) Reverse = true;
        else Reverse = false;

        if (Reverse) trx_mode = 0x03;      // CWR
        else trx_mode = 0x02;              // CW
        trx_mode = trx_mode + (byte8<<8);
      }
      else if (byte7 == 0x03)              // AM
      {
        if (byte8 & 0x80) trx_mode = 0x05; // AM Sync
        else trx_mode = 0x04;              // AM
        trx_mode = trx_mode + (byte8<<8);
      }
      else if (byte7 == 0x04)              // FM
      {
        if ((byte8 & 0x77) == 0) trx_mode = 0x07; // FM Wide (wild guess)
        else trx_mode = 0x06;
        trx_mode = trx_mode + (byte8<<8);
      }
      else if (byte7 == 0x05)              // RTTY
      {
        if (byte8 & 0x80) trx_mode = 0x08; // RTTYlsb
        else trx_mode = 0x09;              // RTTYusb
        trx_mode = trx_mode + (byte8<<8);
      }
      else if (byte7 == 0x06)              // PKT
      {
        if (byte8 & 0x80) trx_mode = 0x0b; // PKTfm
        else trx_mode = 0x0a;              // PKTlsb
        trx_mode = trx_mode + (byte8<<8);
      }      

      radio.mode = true;                   // Indicate that we have successfully read active mode

      antenna_select(incoming);            // Antenna switchover, if frequency applies to the other antenna  
      running[ant].Frq = incoming;
      radio.timer = true;                  // Indicate that we are receiving frq data from Radio
      radio.online = true;  
    }
  }
  trx_clear();                             // Flush/Discard any received data that has not been read.
}

void ft1000MPmkV_request_frequency(void)
{
  ft1000MPmkV_parse_serialinput();      // Grab input from latest poll

  // Transmit a Status request to Yaesu FT100
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x02);                      // Request subcategory Op Data (16 bytes)
  trx_writeln(0x10);                    // Request Status Update from TRX
}
//
//---------------------------------------------------------------------------------
// Autotune stuff
//
//  void ft1000MPmkV_request_pwr(void)    // Not available
//  void ft1000MPmkV_set_pwr(uint8_t pwr) // Not available
//
//---------------------------------------------------------------------------------
// Yaesu FT1000MP MK-V style Set Mode and Filter
//---------------------------------------------------------------------------------
//
void ft1000MPmkV_set_mode(uint16_t mode)
{
  uint8_t tmp;
  uint8_t if8m2, if455;   // Intermediate frequency filters at 8.2 MHz and 455 kHz
  uint8_t fltbyte=0;      // filter information to send to transceiver
  
  // Transmit a Set Mode command to Yaesu FT1000
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(mode & 0x00ff);
  trx_writeln(0x0c);
  delayloop(100);

  // Transmit a Set Filter command to Yaesu FT1000MP MK-V for 455 kHz IF
  tmp = (mode & 0xff00)>>8;
  if455 = tmp & 0x0f;     // Isolate 455kHz info
  if      (if455 == 0x00) fltbyte = 0x04; // Thru / 6 kHz
  else if (if455 == 0x01) fltbyte = 0x00; // 2.4 kHz
  else if (if455 == 0x02) fltbyte = 0x01; // 2.0 kHz
  else if (if455 == 0x03) fltbyte = 0x02; // 500 Hz
  else if (if455 == 0x04) fltbyte = 0x03; // 250 Hz
  trx_write(0x02);        // Write 455 kHz filter info to Radio
  trx_write(0x00);
  trx_write(0x00);
  trx_write(fltbyte);
  trx_writeln(0x8c);
  delayloop(100);
  
  // Transmit a Set Filter command to Yaesu FT1000MP MK-V  for 8.2 MHz IF
  tmp = (mode & 0xff00)>>12;
  if8m2 = tmp & 0x0f;       // Isolate 8.2 MHz filter info
  if      (if8m2 == 0x00) fltbyte = 0x04; // Thru / 6 kHz
  else if (if8m2 == 0x01) fltbyte = 0x00; // 2.4 kHz
  else if (if8m2 == 0x02) fltbyte = 0x01; // 2.0 kHz
  else if (if8m2 == 0x03) fltbyte = 0x02; // 500 Hz
  else if (if8m2 == 0x04) fltbyte = 0x03; // 250 Hz
  trx_write(0x01);      // Write 8.2 MHz filter info to Radio
  trx_write(0x00);
  trx_write(0x00);
  trx_write(fltbyte);
  trx_writeln(0x8c);
  delayloop(100);
}
//
//---------------------------------------------------------------------------------
// Yaesu FT1000MP MK-V style Set Transmit Mode
//---------------------------------------------------------------------------------
//
void ft1000MPmkV_set_tx(void)
{
  // Switch to Transmit (PTT On)
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x01);
  trx_writeln(0x0f);
  delayloop(100);
}
void ft1000MPmkV_set_rx(void)
{
  // Switch to Receive (PTT Off)
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_write(0x00);
  trx_writeln(0x0f);
  delayloop(100);
}


//
//---------------------------------------------------------------------------------
// Yaesu FT-2000 / FT-950 / FT-450 style polling for frequency
//---------------------------------------------------------------------------------
//
// Globals for FT2000
uint8_t ft2000vfo=0;   // 0 = VFO A, 1 = VFO B
uint8_t ft2000swr=0;   // SWR, 0 - 255. 100 equals 3:1, 65 2:1 and 14 1.5:1

// This function uses input from uart_async_read()
void ft2000_parse_serial_input(void)
{
  char *pEnd;
  int32_t frq_in;

  // Grab frequency if header corresponds to VFO A or VFO B information being received
  if (((!strncmp("FA",transceiver_in_string,2)) && (ft2000vfo == 0)) || 
      ((!strncmp("FB",transceiver_in_string,2)) && (ft2000vfo == 1)))
  {
    frq_in = strtol(transceiver_in_string+2,&pEnd,10);

    antenna_select(frq_in);                               // Antenna switchover, if frequency applies to the other antenna  
    running[ant].Frq = frq_in;
    radio.timer = true;                                    // Indicate that we are receiving frq data from Radio
    radio.online = true;  
  }
  else if (!strncmp("VS",transceiver_in_string,2))        // Find out which VFO is in use
  {
    ft2000vfo = strtol(transceiver_in_string+2,&pEnd,10); // 0 received = VFO A, 1 received = VFO B
    radio.timer = true;                                   // Indicate that we are receiving frq data from Radio
    radio.online = true;
  }
  else if (!strncmp("PC",transceiver_in_string,2))        // Read Power Control level
  {
    trx_pwr = strtol(transceiver_in_string+2,&pEnd,10);   // 0 - 255
    radio.pwr = true;                                     // Indicate that we have successfully read power control level
  }
  else if (!strncmp("MD",transceiver_in_string,2))        // Read active Mode (LSB, USB etc)
  {
    trx_mode = strtol(transceiver_in_string+3,&pEnd,10);  // 1=LSB 2=USB 3=CW 4=FM 5=AM 6=FSK 7=CWR 8=PKT-L 9=FSK-R... 
    radio.mode = true;                                    // Indicate that we have successfully read active mode
  }
  else if (!strncmp("RM6",transceiver_in_string,3))       // Read SWR meter
  {
    ft2000swr = strtol(transceiver_in_string+3,&pEnd,10); // SWR, 0 - 255. 100 equals 3:1, 65 2:1 and 14 1.5:1 
    radio.swr = true;                                     // Indicate that we have successfully read SWR
  }
}
//
//---------------------------------------------------------------------------------
// Yaesu FT2000 style Request Frequency
//---------------------------------------------------------------------------------
//
void ft2000_request_frequency(void)
{
  static uint8_t which = 0;
  
  if (which == 0)                                 // Poll for frequency information
  {
    if (ft2000vfo == 0)
    {
      trx_print("FA;");
    }
    else
    {
      trx_print("FB;");
    }
  }
  else                                            // Poll which VFO (A or B) is in use
  {
    trx_print("VS;");
  }
  which++;
  if (which >= 2) which = 0;
}
//
//---------------------------------------------------------------------------------
// Autotune stuff
//
//---------------------------------------------------------------------------------
// Yaesu FT2000 style Request Power
//---------------------------------------------------------------------------------
//
void ft2000_request_pwr(void)
{
  trx_print("PC;");                              // Query status of Power Control
}
//
//---------------------------------------------------------------------------------
// Yaesu FT2000 style Request Mode
//---------------------------------------------------------------------------------
//
void ft2000_request_mode(void)
{
  sprintf(print_buf,"MD%01u;", ft2000vfo);          // Query which operating mode is in use
  trx_print(print_buf);
}
//
//---------------------------------------------------------------------------------
// Yaesu FT2000 style Request SWR
//---------------------------------------------------------------------------------
//
void ft2000_request_swr(void)
{
  trx_print("RM6;");                             // Query SWR information
}
//
//---------------------------------------------------------------------------------
// Yaesu FT2000 style Set Power
//---------------------------------------------------------------------------------
//
void ft2000_set_pwr(uint8_t pwr)
{
  sprintf(print_buf,"PC%03u;", pwr);
  trx_print(print_buf);
}
//
//---------------------------------------------------------------------------------
// Yaesu FT2000 style Set Mode
//---------------------------------------------------------------------------------
//
void ft2000_set_mode(uint8_t mode)
{
  sprintf(print_buf,"MD%01u%01u;", ft2000vfo,mode); // Set Mode, using current VFO
  trx_print(print_buf);
}
//
//---------------------------------------------------------------------------------
// Yaesu FT2000 style Set Transmit Mode
//---------------------------------------------------------------------------------
//
void ft2000_set_tx(uint8_t tx)
{
  sprintf(print_buf,"TX%01u;", tx);                 // 0 for RX, 1 for TX
  trx_print(print_buf);
  if (tx == 0) delayloop(200);                        // Switching to RX, need to give some
                                                  // turnaround time when connected 
                                                  // through an external hub such as
                                                  // the MicroHam microKeyer II
}


//
//---------------------------------------------------------------------------------
// Elecraft K3 or KX3 Read for either polled or auto
//---------------------------------------------------------------------------------
//
// This function uses input from uart_async_read()
void elecraft_parse_serial_input(void)
{
  char *pEnd;
  int32_t frq_in;

  if (!strncmp("IF",transceiver_in_string,2))            // Grab frequency if header is correct
  {
    frq_in = strtol(transceiver_in_string+2,&pEnd,10);

    antenna_select(frq_in);                              // Antenna switchover, if frequency applies to the other antenna  
    running[ant].Frq = frq_in;
    radio.timer = true;    	                             // Indicate that we are receiving frq data from Radio
    radio.online = true;    
  }
  else if (!strncmp("PC",transceiver_in_string,2))       // Read Power Control level
  {
    trx_pwr = strtol(transceiver_in_string+2,&pEnd,10);  // 0 - 255
    radio.pwr = true;                                    // Indicate that we have successfully read power control level
  }
  else if (!strncmp("MD",transceiver_in_string,2))       // Read active Mode (LSB, USB etc)
  {
    trx_mode = strtol(transceiver_in_string+2,&pEnd,10); // 1=LSB 2=USB 3=CW 4=FM 5=AM 6=FSK 7=CWR 8=PKT-L 9=FSK-R... 
    radio.mode = true;                                   // Indicate that we have successfully read active mode
  }
}
//
//---------------------------------------------------------------------------------
// Elecraft K3 or KX3 style polling for frequency
//---------------------------------------------------------------------------------
//
void elecraft_request_frequency(void)
{
  trx_print("IF;");                    // Transmit a frequency poll request to transceiver
}
//
//---------------------------------------------------------------------------------
// Autotune stuff
//
//---------------------------------------------------------------------------------
// Elecraft K3 or KX3 style Request Power Level
//---------------------------------------------------------------------------------
//
void elecraft_request_pwr(void)
{
  trx_print("PC;");
}

//
//---------------------------------------------------------------------------------
// Elecraft K3 or KX3 style Request Mode
//---------------------------------------------------------------------------------
//
void elecraft_request_mode(void)
{
  trx_print("MD;");
}
//
//---------------------------------------------------------------------------------
// Elecraft K3 or KX3 style Set Power
//---------------------------------------------------------------------------------
//
void elecraft_set_pwr(uint8_t pwr)
{
  sprintf(print_buf,"PC%03u;", pwr);
  trx_print(print_buf);
}
//
//---------------------------------------------------------------------------------
// Elecraft K3 or KX3 style Set Mode
//---------------------------------------------------------------------------------
//
void elecraft_set_mode(uint8_t mode)
{
  sprintf(print_buf,"MD%01u;", mode);               // Set Mode, using current VFO
  trx_print(print_buf);
}
//
//---------------------------------------------------------------------------------
// Elecraft K3 or KX3 style Set Transmit Mode
//---------------------------------------------------------------------------------
//
void elecraft_set_tx(void)
{
  trx_print("TX;");
  delayloop(100);}
void elecraft_set_rx(void)
{
  trx_print("RX;");
  delayloop(100);
}


//
//---------------------------------------------------------------------------------
// Kenwood TS-440, TS-480 / TS-2000 Parse input
//---------------------------------------------------------------------------------
//
// This function uses input from serial_async_read()
void ts2000_parse_serial_input(void)
{
  char *pEnd;
  char frqstring[12];
  int32_t  frq_in;
  
  // Grab frequency and Mode if header is correct and length is consistent with an IF response
  if ((!strncmp("IF",transceiver_in_string,2)) && (strlen(transceiver_in_string) > 30)) 
  {
    strncpy(frqstring, transceiver_in_string+2,11);
    frq_in = strtol(frqstring,&pEnd,10);

    antenna_select(frq_in);                              // Antenna switchover, if frequency applies to the other antenna  
    running[ant].Frq = frq_in;
    //trx_mode = strtol(transceiver_in_string+2,&pEnd,10); // 1=LSB 2=USB 3=CW 4=FM 5=AM 6=FSK 7=CWR 8=PKT-L 9=FSK-R... 
    //radio.mode = true;
    radio.timer = true;	                                 // Indicate that we are receiving frq data from Radio
    radio.online = true;
  }
  else if (!strncmp("PC",transceiver_in_string,2))       // Read Power Control level
  {
    trx_pwr = strtol(transceiver_in_string+2,&pEnd,10);  // 0 - 255
    radio.pwr = true;                                    // Indicate that we have successfully read power control level
  }
  else if (!strncmp("MD",transceiver_in_string,2))       // Read active Mode (LSB, USB etc)
  {
    trx_mode = strtol(transceiver_in_string+2,&pEnd,10); // 1=LSB 2=USB 3=CW 4=FM 5=AM 6=FSK 7=CWR 8=PKT-L 9=FSK-R... 
    radio.mode = true;                                   // Indicate that we have successfully read active mode
  }
}
//
//---------------------------------------------------------------------------------
// Kenwood TS-440, TS-480 / TS-2000 style Request Frequency and Mode
//---------------------------------------------------------------------------------
//
void ts2000_request_frequency(void)
{
  trx_print("IF;");                               // Transmit a frequency poll request to transceiver
}
//
//---------------------------------------------------------------------------------
// Autotune stuff
//
//---------------------------------------------------------------------------------
// Kenwood TS-480 / TS-2000 style Request Power
//---------------------------------------------------------------------------------
//
void ts2000_request_pwr(void)
{
  trx_print("PC;");                              // Query status of Power Control
  delayloop(10);
}
//
//---------------------------------------------------------------------------------
// Kenwood TS-480 / TS-2000 style Request Mode
//---------------------------------------------------------------------------------
//
void ts2000_request_mode(void)
{
  trx_print("MD;");                              // Query which operating mode is in use
  delayloop(10);
}
//
//---------------------------------------------------------------------------------
// Kenwood TS-480 / TS-2000 style Set Power
//---------------------------------------------------------------------------------
//
void ts2000_set_pwr(uint8_t pwr)
{
  sprintf(print_buf,"PC%03u;", pwr);
  trx_print(print_buf);
  delayloop(100);
}
//
//---------------------------------------------------------------------------------
// Kenwood TS-440, TS-480 / TS-2000 style Set Mode
//---------------------------------------------------------------------------------
//
void ts2000_set_mode(uint8_t mode)
{
  sprintf(print_buf,"MD%01u;", mode);
  trx_print(print_buf);
  delayloop(100);
}
//
//---------------------------------------------------------------------------------
// Kenwood TS-440, TS-480 / TS-2000 style Set Transmit Mode
//---------------------------------------------------------------------------------
//
void ts2000_set_tx(void)
{
  #if KENWOOD_TX_KLUDGE
  trx_print("TX0;");
  #else
  trx_print("TX;");
  #endif
  delayloop(100);
}
void ts2000_set_rx(void)
{
  trx_print("RX;");
  delayloop(250);
}


//
//---------------------------------------------------------------------------------
// Kenwood TS-870 Parse input
//---------------------------------------------------------------------------------
//
// This function uses input from serial_async_read()
void ts870_parse_serial_input(void)
{
  char *pEnd;
  char frqstring[12];
  int32_t  frq_in;
  
  // Grab frequency and Mode if header is correct and length is consistent with an IF response
  if ((!strncmp("IF",transceiver_in_string,2)) && (strlen(transceiver_in_string) >= 28)) 
  {
    strncpy(frqstring, transceiver_in_string+2,11);
    frq_in = strtol(frqstring,&pEnd,10);

    antenna_select(frq_in);                              // Antenna switchover, if frequency applies to the other antenna  
    running[ant].Frq = frq_in;
    //trx_mode = strtol(transceiver_in_string+2,&pEnd,10); // 1=LSB 2=USB 3=CW 4=FM 5=AM 6=FSK 7=CWR 8=PKT-L 9=FSK-R... 
    //radio.mode = true;
    radio.timer = true;                                   // Indicate that we are receiving frq data from Radio
    radio.online = true;
  }
  else if (!strncmp("PC",transceiver_in_string,2))       // Read Power Control level
  {
    trx_pwr = strtol(transceiver_in_string+2,&pEnd,10);  // 0 - 255
    radio.pwr = true;                                    // Indicate that we have successfully read power control level
  }
  else if (!strncmp("MD",transceiver_in_string,2))       // Read active Mode (LSB, USB etc)
  {
    trx_mode = strtol(transceiver_in_string+2,&pEnd,10); // 1=LSB 2=USB 3=CW 4=FM 5=AM 6=FSK 7=CWR 8=PKT-L 9=FSK-R... 
    radio.mode = true;                                   // Indicate that we have successfully read active mode
  }
}
//
//---------------------------------------------------------------------------------
// Kenwood TS-870 style Request Frequency and Mode
//---------------------------------------------------------------------------------
//
void ts870_request_frequency(void)
{
  trx_print("IF;");                               // Transmit a frequency poll request to transceiver
}
//
//---------------------------------------------------------------------------------
// Autotune stuff
//
//---------------------------------------------------------------------------------
// Kenwood TS-870 style Request Power
//---------------------------------------------------------------------------------
//
void ts870_request_pwr(void)
{
  trx_print("PC;");                              // Query status of Power Control
  delayloop(10);
}
//
//---------------------------------------------------------------------------------
// Kenwood TS-870 style Request Mode
//---------------------------------------------------------------------------------
//
void ts870_request_mode(void)
{
  trx_print("MD;");                              // Query which operating mode is in use
  delayloop(10);
}
//
//---------------------------------------------------------------------------------
// Kenwood TS-870 style Set Power
//---------------------------------------------------------------------------------
//
void ts870_set_pwr(uint8_t pwr)
{
  sprintf(print_buf,"PC%03u;", pwr);
  trx_print(print_buf);
  delayloop(100);
}
//
//---------------------------------------------------------------------------------
// Kenwood TS-870 style Set Mode
//---------------------------------------------------------------------------------
//
void ts870_set_mode(uint8_t mode)
{
  sprintf(print_buf,"MD%01u;", mode);
  trx_print(print_buf);
  delayloop(100);
}
//
//---------------------------------------------------------------------------------
// Kenwood TS-870 style Set Transmit Mode
//---------------------------------------------------------------------------------
//
void ts870_set_tx(void)
{
  trx_print("TX;");
  delayloop(100);
}
void ts870_set_rx(void)
{
  trx_print("RX;");
  delayloop(250);
}



//
//---------------------------------------------------------------------------------
// TenTec binary style polling for frequency
//---------------------------------------------------------------------------------
//
void tentec_binary_parse_serialinput(void)
{
  uint32_t incoming;
  uint8_t type;                       // Message type identifier
  
  uint16_t waiting;                   // Number of chars waiting in receive buffer

  // Find out how many characters are waiting to be read. 
  waiting = trx_available();

  if (waiting >= 4)                   // Minimum sized message
  {
    type = trx_read();                // Read first char of incoming message to identify which type
    
    // Is this a frequency Poll response?
    if((type == 'A') && (waiting >= 6))
    {
      // Decode frequency content of the incoming message
      incoming =  trx_read()<<24;
      incoming += trx_read()<<16;
      incoming += trx_read()<<8;
      incoming += trx_read();
      trx_readln();                   // Read the terminating <cr>

      if ((incoming > 1000000) && (incoming < 54000000))// Impose some sane boundaries
      {
        antenna_select(incoming);                       // Antenna switchover, if frequency applies to the other antenna  
        running[ant].Frq = incoming;
        radio.timer = true;	                            // Indicate that we are receiving frq data from Radio
        radio.online = true;  
      }
    }
    // Is this a Mode poll response?
    else if (type == 'M')
    {
      trx_mode =  trx_read()<<8;
      trx_mode += trx_read();
      trx_readln();

      radio.mode = true;
    }
    // Is this a Power output query response?
    else if (type == 'C')
    {
      if (trx_read() == '1')
      {
        if (trx_read() == 'X')
        {
          trx_pwr = trx_read();
          trx_readln();
          radio.pwr = true;                             // Indicate that we have successfully read power control level
        }  
      }
    }
  }
  trx_clear();                                          // Flush/Discard any received data that has not been read.
}
void tentec_binary_request_frequency(void)
{
  tentec_binary_parse_serialinput();    // Grab input from latest poll

  // Transmit a Frequency request to the TenTec
  trx_print("?A\r");                    // Transmit a frequency poll request to transceiver
}
//
//---------------------------------------------------------------------------------
// Autotune stuff
//
//
//---------------------------------------------------------------------------------
// TenTec binary style Request Power setting
//---------------------------------------------------------------------------------
void tentec_binary_request_pwr(void)
{
  tentec_binary_parse_serialinput();    // Grab and flush any pending input

  trx_print("?C1X\r");                  // Transmit a Power level request
  
  // Try for up to 1 second to get a power level reading
  for (uint8_t x = 0; (x < 5) && !radio.pwr; x++)
  {
    delayloop(100);                     // wait for 200ms 
    tentec_binary_parse_serialinput();  // Grab and flush any pending input
  }
}
//---------------------------------------------------------------------------------
// TenTec binary style Request Mode setting
//---------------------------------------------------------------------------------
void tentec_binary_request_mode(void)
{
  tentec_binary_parse_serialinput();    // Grab and flush any pending input

  trx_print("?M\r");                    // Transmit a Mode request
  
  // Try for up to 1 second to get a power level reading
  for (uint8_t x = 0; (x < 5) && !radio.mode; x++)
  {
    delayloop(100);                         // wait for 200ms 
    tentec_binary_parse_serialinput();  // Grab and flush any pending input
  }
}
//
//---------------------------------------------------------------------------------
// TenTec binary style Set Power
//---------------------------------------------------------------------------------
//
void tentec_binary_set_pwr(uint8_t pwr)
{
  trx_print("*C1X");
  trx_write(pwr);
  trx_print("\r");
  delayloop(100);
}
//
//---------------------------------------------------------------------------------
// TenTec binary style Set Mode
//---------------------------------------------------------------------------------
//
void tentec_binary_set_mode(uint16_t mode)
{
  trx_print("*M");
  trx_write((mode & 0xff00)>>8);
  trx_write((uint8_t) mode & 0x00ff);
  trx_print("\r");
  delayloop(100);
}
//
//---------------------------------------------------------------------------------
// TenTec binary style Set Transmit Mode
//---------------------------------------------------------------------------------
//
void tentec_binary_set_tx(void)
{
  // Switch to Transmit (PTT On)
  trx_print("*T");
  trx_write(0x04);
  trx_write(0x00);
  trx_print("\r");
  delayloop(100);
}
void tentec_binary_set_rx(void)
{
  // Switch to Receive (PTT Off)
  trx_print("*T");
  trx_write(0x00);
  trx_write(0x00);
  trx_print("\r");
  delayloop(100);
}


//
//---------------------------------------------------------------------------------
// TenTec ascii style parse and poll
//---------------------------------------------------------------------------------
//
// This function uses input from uart_async_read()
void tentec_ascii_parse_serial_input(void)
{
  char *pEnd;
  uint32_t  frq_in;

  if (!strncmp("@AF",transceiver_in_string,3))           // Grab frequency if header is correct
  {
    frq_in = strtol(transceiver_in_string+3,&pEnd,10);

    antenna_select(frq_in);                              // Antenna switchover, if frequency applies to the other antenna  
    running[ant].Frq = frq_in;
    radio.timer = true;	                                 // Indicate that we are receiving frq data from Radio
    radio.online = true;  
  }

  else if (!strncmp("@TP",transceiver_in_string,3))      // Read Power Control level
  {
    trx_pwr = strtol(transceiver_in_string+3,&pEnd,10);  // 0 - 255
    radio.pwr = true;                                    // Indicate that we have successfully read power control level
  }
  else if (!strncmp("@RMM",transceiver_in_string,4))     // Read active Mode (LSB, USB etc)
  {
    trx_mode = strtol(transceiver_in_string+4,&pEnd,10); // 0=USB 1=LSB 2=CW 3=CW 4=AM 5=FM 
    radio.mode = true;                                   // Indicate that we have successfully read active mode
  }
}
void tentec_ascii_request_frequency(void)
{
  trx_print("?AF\r");                  // Transmit a frequency poll request to transceiver
}
//
//---------------------------------------------------------------------------------
// Autotune stuff
//
//---------------------------------------------------------------------------------
// TenTec ascii style Request Power Level
//---------------------------------------------------------------------------------
//
void tentec_ascii_request_pwr(void)
{
  trx_print("?TP\r");
}

//
//---------------------------------------------------------------------------------
// TenTec ascii style Request Mode
//---------------------------------------------------------------------------------
//
void tentec_ascii_request_mode(void)
{
  trx_print("?RMM\r");
}
//
//---------------------------------------------------------------------------------
// TenTec ascii style Set Power
//---------------------------------------------------------------------------------
//
void tentec_ascii_set_pwr(uint8_t pwr)
{
  sprintf(print_buf,"*TP%u\r", pwr);
  trx_print(print_buf);
}
//
//---------------------------------------------------------------------------------
// TenTec ascii style Set Mode
//---------------------------------------------------------------------------------
//
void tentec_ascii_set_mode(uint8_t mode)
{
  sprintf(print_buf,"*RMM%01u\r", mode);
  trx_print(print_buf);
}
//
//---------------------------------------------------------------------------------
// TenTec ascii style Set Transmit Mode
//---------------------------------------------------------------------------------
//
void tentec_ascii_set_tx(void)
{
  trx_print("*TK\r");
}
void tentec_ascii_set_rx(void)
{
  trx_print("*TU\r");
}


//
//---------------------------------------------------------------------------------
// Pseudo-VFO style update frequency (no parse and poll)
//---------------------------------------------------------------------------------
//
void pseudovfo_update_frq(void)
{
  antenna_select(pseudovfo_frq);   // Antenna switchover, if frequency applies to the other antenna  
  running[ant].Frq = pseudovfo_frq;
  radio.timer = true;	             // Indicate that we are receiving frq data from Radio
  radio.online = true;
}
// Pseudo-VFO active, Update Pseudo VFO frequency using Encoder 
void update_pseudo_vfo(void)
{
  pseudovfo_frq = 100*(pseudovfo_frq/100 + Enc.read());
  Enc.write(0);
}
// Initialize pseudo VFO
void init_pseudo_vfo(void)
{
  if (tunedFrq != 0)               // Ensure we are not resetting to 0 in case no positions have been stored 
  {
    pseudovfo_frq = tunedFrq;  
  }
  else if (tunedFrq == 0)
  {
    pseudovfo_frq = 14000000;      // Should not be needed - belt and braces   
  }
}
// Return current pseudo-VFO frequency for LCD print
int32_t pseudo_vfo(void)
{
  return pseudovfo_frq;
}
// Toggle Encoder use between Pseudo VFO and Stepper
void pseudovfo_encoder_toggle(void)
{
  controller_settings.pseudo_vfo ^= 1;         // toggle
  EEPROM_writeAnything(1,controller_settings); // write controller settings into eeprom

  virt_lcd_clear();                            // Announce on LCD
  virt_lcd_setCursor(0, 1);
  if (controller_settings.pseudo_vfo)
  {
    virt_lcd_print("Pseudo-VFO ON!!!");
    init_pseudo_vfo();
  }
  else
  {
    virt_lcd_print("Pseudo-VFO OFF!!!");
    radio.timer  = false;          // Instant indication of no FRQ data
    radio.online = false;
  }
  Menu_exit_timer = 10;            // Show on LCD for 1 second             
  // Force a reprint of Config Menu upon return, if we went here from there
  flag.menu_lcd_upd = false;
}

// Pseudo-VFO active, use Up/Down switches to step through frequency bands
// and in 100 kHz increments within bands
void pseudo_vfo_up_down(void)
{
  if (up_button_toggle() )
  {
    // Switch to a higher frequency band
    if (pseudovfo_frq < 1800000) pseudovfo_frq = 1800000;
    else if (pseudovfo_frq < 1900000) pseudovfo_frq = 1900000;
    else if (pseudovfo_frq < 2000000) pseudovfo_frq = 2000000;
    else if (pseudovfo_frq < 3500000) pseudovfo_frq = 3500000;
    else if (pseudovfo_frq < 3600000) pseudovfo_frq = 3600000;
    else if (pseudovfo_frq < 3700000) pseudovfo_frq = 3700000;
    else if (pseudovfo_frq < 3800000) pseudovfo_frq = 3800000;
    else if (pseudovfo_frq < 3900000) pseudovfo_frq = 3900000;
    else if (pseudovfo_frq < 4000000) pseudovfo_frq = 4000000;
    else if (pseudovfo_frq < 7000000) pseudovfo_frq = 7000000;
    else if (pseudovfo_frq < 7100000) pseudovfo_frq = 7100000;
    else if (pseudovfo_frq < 7200000) pseudovfo_frq = 7200000;
    else if (pseudovfo_frq < 7300000) pseudovfo_frq = 7300000;
    else if (pseudovfo_frq < 10100000) pseudovfo_frq = 10100000;
    else if (pseudovfo_frq < 10150000) pseudovfo_frq = 10150000;
    else if (pseudovfo_frq < 14000000) pseudovfo_frq = 14000000;
    else if (pseudovfo_frq < 14100000) pseudovfo_frq = 14100000;
    else if (pseudovfo_frq < 14200000) pseudovfo_frq = 14200000;
    else if (pseudovfo_frq < 14300000) pseudovfo_frq = 14300000;
    else if (pseudovfo_frq < 14350000) pseudovfo_frq = 14350000;
    else if (pseudovfo_frq < 18068000) pseudovfo_frq = 18068000;
    else if (pseudovfo_frq < 18168000) pseudovfo_frq = 18168000;
    else if (pseudovfo_frq < 21000000) pseudovfo_frq = 21000000;
    else if (pseudovfo_frq < 21100000) pseudovfo_frq = 21100000;
    else if (pseudovfo_frq < 21200000) pseudovfo_frq = 21200000;
    else if (pseudovfo_frq < 21300000) pseudovfo_frq = 21300000;
    else if (pseudovfo_frq < 21400000) pseudovfo_frq = 21400000;
    else if (pseudovfo_frq < 21450000) pseudovfo_frq = 21450000;
    else if (pseudovfo_frq < 24890000) pseudovfo_frq = 24890000;
    else if (pseudovfo_frq < 24990000) pseudovfo_frq = 24990000;
    else if (pseudovfo_frq < 28000000) pseudovfo_frq = 28000000;
    else if (pseudovfo_frq < 28100000) pseudovfo_frq = 28100000;
    else if (pseudovfo_frq < 28200000) pseudovfo_frq = 28200000;
    else if (pseudovfo_frq < 28300000) pseudovfo_frq = 28300000;
    else if (pseudovfo_frq < 28400000) pseudovfo_frq = 28400000;
    else if (pseudovfo_frq < 28500000) pseudovfo_frq = 28500000;
    else if (pseudovfo_frq < 28600000) pseudovfo_frq = 28600000;
    else if (pseudovfo_frq < 28700000) pseudovfo_frq = 28700000;
    else if (pseudovfo_frq < 28800000) pseudovfo_frq = 28800000;
    else if (pseudovfo_frq < 28900000) pseudovfo_frq = 28900000;
    else if (pseudovfo_frq < 29000000) pseudovfo_frq = 29000000;
    else if (pseudovfo_frq < 29100000) pseudovfo_frq = 29100000;
    else if (pseudovfo_frq < 29200000) pseudovfo_frq = 29200000;
    else if (pseudovfo_frq < 29300000) pseudovfo_frq = 29300000;
    else if (pseudovfo_frq < 29400000) pseudovfo_frq = 29400000;
    else if (pseudovfo_frq < 29500000) pseudovfo_frq = 29500000;
    else if (pseudovfo_frq < 29600000) pseudovfo_frq = 29600000;
    else if (pseudovfo_frq < 29700000) pseudovfo_frq = 29700000;
  }
  else if (dn_button_toggle() )
  {
    // Switch to a lower frequency band
    if (pseudovfo_frq > 29700000) pseudovfo_frq = 29700000;
    else if (pseudovfo_frq > 29600000) pseudovfo_frq = 29600000;
    else if (pseudovfo_frq > 29500000) pseudovfo_frq = 29500000;
    else if (pseudovfo_frq > 29400000) pseudovfo_frq = 29400000;
    else if (pseudovfo_frq > 29300000) pseudovfo_frq = 29300000;
    else if (pseudovfo_frq > 29200000) pseudovfo_frq = 29200000;
    else if (pseudovfo_frq > 29100000) pseudovfo_frq = 29100000;
    else if (pseudovfo_frq > 29000000) pseudovfo_frq = 29000000;
    else if (pseudovfo_frq > 28900000) pseudovfo_frq = 28900000;
    else if (pseudovfo_frq > 28800000) pseudovfo_frq = 28800000;
    else if (pseudovfo_frq > 28700000) pseudovfo_frq = 28700000;
    else if (pseudovfo_frq > 28600000) pseudovfo_frq = 28600000;
    else if (pseudovfo_frq > 28500000) pseudovfo_frq = 28500000;
    else if (pseudovfo_frq > 28400000) pseudovfo_frq = 28400000;
    else if (pseudovfo_frq > 28300000) pseudovfo_frq = 28300000;
    else if (pseudovfo_frq > 28200000) pseudovfo_frq = 28200000;
    else if (pseudovfo_frq > 28100000) pseudovfo_frq = 28100000;
    else if (pseudovfo_frq > 28000000) pseudovfo_frq = 28000000;
    else if (pseudovfo_frq > 24990000) pseudovfo_frq = 24990000;
    else if (pseudovfo_frq > 24890000) pseudovfo_frq = 24890000;
    else if (pseudovfo_frq > 21450000) pseudovfo_frq = 21450000;
    else if (pseudovfo_frq > 21400000) pseudovfo_frq = 21400000;
    else if (pseudovfo_frq > 21300000) pseudovfo_frq = 21300000;
    else if (pseudovfo_frq > 21200000) pseudovfo_frq = 21200000;
    else if (pseudovfo_frq > 21100000) pseudovfo_frq = 21100000;
    else if (pseudovfo_frq > 21000000) pseudovfo_frq = 21000000;
    else if (pseudovfo_frq > 18168000) pseudovfo_frq = 18168000;
    else if (pseudovfo_frq > 18068000) pseudovfo_frq = 18068000;
    else if (pseudovfo_frq > 14350000) pseudovfo_frq = 14350000;
    else if (pseudovfo_frq > 14300000) pseudovfo_frq = 14300000;
    else if (pseudovfo_frq > 14200000) pseudovfo_frq = 14200000;
    else if (pseudovfo_frq > 14100000) pseudovfo_frq = 14100000;
    else if (pseudovfo_frq > 14000000) pseudovfo_frq = 14000000;
    else if (pseudovfo_frq > 10150000) pseudovfo_frq = 10150000;
    else if (pseudovfo_frq > 10100000) pseudovfo_frq = 10100000;
    else if (pseudovfo_frq > 7300000) pseudovfo_frq = 7300000;
    else if (pseudovfo_frq > 7200000) pseudovfo_frq = 7200000;
    else if (pseudovfo_frq > 7100000) pseudovfo_frq = 7100000;
    else if (pseudovfo_frq > 7000000) pseudovfo_frq = 7000000;
    else if (pseudovfo_frq > 4000000) pseudovfo_frq = 4000000;
    else if (pseudovfo_frq > 3900000) pseudovfo_frq = 3900000;
    else if (pseudovfo_frq > 3800000) pseudovfo_frq = 3800000;
    else if (pseudovfo_frq > 3700000) pseudovfo_frq = 3700000;
    else if (pseudovfo_frq > 3600000) pseudovfo_frq = 3600000;
    else if (pseudovfo_frq > 3500000) pseudovfo_frq = 3500000;
    else if (pseudovfo_frq > 1800000) pseudovfo_frq = 1800000;
    else if (pseudovfo_frq > 1700000) pseudovfo_frq = 1700000;
    else if (pseudovfo_frq > 1600000) pseudovfo_frq = 1600000;
  }
}


//---------------------------------------------------------------------------------
// Asynchronous Read and parse anything from the Transceiver (TTL or RS232)
// Addresses appropriate transceivers, based on which one is selected
//---------------------------------------------------------------------------------
void trx_read_and_parse(void)
{
  if (async_uart_read)             // BOOL for Asynchronous serial read mode for certain radios
  {            
    if (transceiver_async_read() ) // Read incoming to a string.  If a message has been read - then parse it
    {
      // Parse captured input string for Frequency Data
      if (controller_settings.trx[controller_settings.radioprofile].radio == 0)  // ICOM
        icom_parse_serial_input();
      if (controller_settings.trx[controller_settings.radioprofile].radio == 1)  // ICOM
        icom_parse_serial_input();
      if (controller_settings.trx[controller_settings.radioprofile].radio == 2)  // Kenwood TS-440
        ts2000_parse_serial_input();
      if (controller_settings.trx[controller_settings.radioprofile].radio == 3)  // Kenwood TS-870
        ts870_parse_serial_input();
      if (controller_settings.trx[controller_settings.radioprofile].radio == 4)  // Kenwood TS-480 / TS-2000
        ts2000_parse_serial_input();
      if (controller_settings.trx[controller_settings.radioprofile].radio == 12)  // Yaesu FT-450/950/1200/2000...
        ft2000_parse_serial_input();
      if (controller_settings.trx[controller_settings.radioprofile].radio == 13)  // Elecraft K3/KX3
        elecraft_parse_serial_input();
      if (controller_settings.trx[controller_settings.radioprofile].radio == 14)  // Elecraft K3/KX3
        elecraft_parse_serial_input();
      if (controller_settings.trx[controller_settings.radioprofile].radio == 15) // TenTec ascii style
        tentec_ascii_parse_serial_input();
    }
  } 
  else transceiver_sync_read();    // If syncronous serial mode (binary messages which do not have
                                   // any obvious beginning and end, then assemble all incoming data
                                   // onto a circular buffer which is zeroed at each new poll for data.
}

//---------------------------------------------------------------------------------
// Monitor whether Radio is Online - poll the Radio for Frequency information
// Indicate whether radio is online or not with >radio.online< flag
// Frequency requests, when successful, will return the >running.Frq< in Hz
//---------------------------------------------------------------------------------
void trx_poll(void)
{
  if (!swr.tune)                         // Only do this if we are not in the middle of SWR tune
  {
    switch (controller_settings.trx[controller_settings.radioprofile].radio)
    {
      case 0:                            // ICOM
        radio.timer = true;              // Indicate that we are receiving frq data from Radio
        radio.online = true;
        break;                           // broadcast mode, no need to poll
      case 1:
        icom_request_frequency();
        break;
      case 2:                            // Kenwood
        ts2000_request_frequency();
        break;
      case 3:
        ts870_request_frequency();
        break;
      case 4:
        ts2000_request_frequency();
        break;
      case 5:                            // Yaesu FT-100
        ft100_request_frequency();
        break;
      case 6:                            // Yaesu FT-7X7
        ft7x7_request_status();
        break;
      case 7:                            // Yaesu FT-8X7
        ft8x7_request_frequency();
        break;
      case 8:                            // Yaesu FT-920
        ft920_request_status();
        break;
      case 9:                            // Yaesu FT-990
        ft990_request_frequency();
        break;
      case 10:                            // Yaesu FT-1000MP
        ft1000MP_request_frequency();
        break;
      case 11:                            // Yaesu FT-1000MP Mk-V
        ft1000MPmkV_request_frequency();
        break;
      case 12:                           // Yaesu FT-450/950/1200/2000...
        ft2000_request_frequency();
        break;
      case 13:                           // Elecraft K3/KX3
        radio.timer = true;              // Indicate that we are receiving frq data from Radio
        radio.online = true;
        break;                           // broadcast mode, no need to poll
      case 14:
        elecraft_request_frequency();
        break;
      case 15:                           // TenTec binary style
        tentec_binary_request_frequency();
        break;
      case 16:                           // TenTec ascii style
        tentec_ascii_request_frequency();   
        break;
      case 17:
        pseudovfo_update_frq();          // Pseudo-VFO                
    }
  }
}

//---------------------------------------------------------------------------------
// Request Power Level Setting from the Radio
//---------------------------------------------------------------------------------
uint8_t trx_request_pwr(void)
{
  uint8_t implemented = WORKING;       // A means to indicate if command not supported
  
  switch (controller_settings.trx[controller_settings.radioprofile].radio)
  {
    case 0:                            // ICOM
    case 1:
     icom_request_pwr();
     break;
    case 2:                            // Kenwood TS-440
      radio.pwr = true;                // Command not available with TS-440, simply indicate success
      implemented = FAIL;              // Not implemented
      break;
    case 3:                            // Kenwood TS-870
      ts870_request_pwr();
      break;
    case 4:                            // Kenwood
      ts2000_request_pwr();
      break;
    case 5:                            // Yaesu FT-100
      radio.pwr = true;                // Command not available with FT100, simply indicate success
      implemented = FAIL;              // Not implemented
      break;
    case 6:                            // Yaesu FT-7X7
      radio.pwr = true;                // Command not available with FT7x7, simply indicate success
      implemented = FAIL;              // Not implemented
      break;
    case 7:                            // Yaesu FT-8X7
      radio.pwr = true;                // Command not available with FT8x7, simply indicate success
      implemented = FAIL;              // Not implemented
      break;
    case 8:                            // Yaesu FT-920
      radio.pwr = true;                // Command not available with FT990, simply indicate success
      implemented = FAIL;              // Not implemented
      break;
    case 9:                            // Yaesu FT-990
      radio.pwr = true;                // Command not available with FT920, simply indicate success
      implemented = FAIL;              // Not implemented
      break;
    case 10:                            // Yaesu FT-1000MP
      radio.pwr = true;                // Command not available with FT1000MP mk-V, simply indicate success
      implemented = FAIL;              // Not implemented
      break;
    case 11:                            // Yaesu FT-1000MP Mk-V
      radio.pwr = true;                // Command not available with FT1000MP mk-V, simply indicate success
      implemented = FAIL;              // Not implemented
      break;
    case 12:                            // Yaesu FT-450/950/1200/2000...
      ft2000_request_pwr();
      break;
    case 13:                           // Elecraft K3/KX3
    case 14:
      elecraft_request_pwr();
      break;
    case 15:                            // TenTec binary style
      tentec_binary_request_pwr();
      break;
    case 16:                            // TenTec ascii style
      tentec_ascii_request_pwr();   
      break;
    default:;                          // Pseudo-VFO - do nothing  
  }
  return implemented;
}

//---------------------------------------------------------------------------------
// Set Minimum Power Level of the Radio
//---------------------------------------------------------------------------------
void trx_set_min_pwr(void)
{
  switch (controller_settings.trx[controller_settings.radioprofile].radio)
  {
    case 0:                            // ICOM
    case 1:
      icom_set_pwr(tx_tune_pwr);
      break;
    case 2:                            // Kenwood TS-440
      ;//ts440_set_pwr(pwr);           // Not available
      break;
    case 3:                            // Kenwood TS-870
      ts870_set_pwr(tx_tune_pwr);
      break;
    case 4:                            // Kenwood
      ts2000_set_pwr(tx_tune_pwr);
      break;
    case 5:                            // Yaesu FT-100
      ;//ft100_set_pwr(pwr);           // Not available
      break;
    case 6:                            // Yaesu FT-7X7
      ;//ft7x7_set_pwr(pwr);           // Not available
      break;
    case 7:                            // Yaesu FT-8X7
      ;//ft8x7_set_pwr(pwr);           // Not available
      break;
    case 8:                            // Yaesu FT-920
      ;//ft990_set_pwr(pwr);           // Not available
      break;
    case 9:                            // Yaesu FT-990
      ;//ft990_set_pwr(pwr);           // Not available
      break;
    case 10:                            // Yaesu FT-1000MP
      ;//ft1000MP_set_pwr(pwr);        // Not available
      break;
    case 11:                            // Yaesu FT-1000MP Mk-V
      ;//ft1000MPmkV_set_pwr(pwr);     // Not available
      break;
    case 12:                            // Yaesu FT-450/950/1200/2000...
      ft2000_set_pwr(tx_tune_pwr);
      break;
    case 13:                           // Elecraft K3/KX3
    case 14:
      elecraft_set_pwr(tx_tune_pwr);
      break;
    case 15:                           // TenTec binary style
      tentec_binary_set_pwr(tx_tune_pwr);
      break;
    case 16:                           // TenTec ascii style
      tentec_ascii_set_pwr(tx_tune_pwr);   
      break;
    default:;                          // Pseudo-VFO - do nothing  
  }
}

//---------------------------------------------------------------------------------
// Restore Original Power Level Setting of the Radio
//---------------------------------------------------------------------------------
void trx_restore_pwr(void)
{
  switch (controller_settings.trx[controller_settings.radioprofile].radio)
  {
    case 0:                            // ICOM
    case 1:
      icom_set_pwr(trx_pwr);
      break;
    case 2:                            // Kenwood TS-440
      ;//ts440_set_pwr(pwr);           // Not available
      break;
    case 3:                            // Kenwood TS-870
      ts870_set_pwr(trx_pwr);
      break;
    case 4:                            // Kenwood
      ts2000_set_pwr(trx_pwr);
      break;
    case 5:                            // Yaesu FT-100
      ;//ft100_set_pwr(pwr);           // Not available
      break;
    case 6:                            // Yaesu FT-7X7
      ;//ft7x7_set_pwr(pwr);           // Not available
      break;
    case 7:                            // Yaesu FT-8X7
      ;//ft8x7_set_pwr(pwr);           // Not available
      break;
    case 8:                            // Yaesu FT-920
      ;//ft920_set_pwr(pwr);
      break;
    case 9:                            // Yaesu FT-990
      ;//ft990_set_pwr(pwr);
      break;
    case 10:                            // Yaesu FT-1000MP
      ;//ft1000MP_set_pwr(pwr);
      break;
    case 11:                            // Yaesu FT-1000MP Mk-V
      ;//ft1000MPmkV_set_pwr(pwr);
      break;
    case 12:                            // Yaesu FT-450/950/1200/2000...
      ft2000_set_pwr(trx_pwr);
      break;
    case 13:                           // Elecraft K3/KX3
    case 14:
      elecraft_set_pwr(trx_pwr);
      break;
    case 15:                           // TenTec binary style
      tentec_binary_set_pwr(trx_pwr);
      break;
    case 16:                           // TenTec ascii style
      tentec_ascii_set_pwr(trx_pwr);   
      break;
    default:;                          // Pseudo-VFO - do nothing    
  }
}

//---------------------------------------------------------------------------------
// Set Radio to AM Mode (or similar) for Tune
//---------------------------------------------------------------------------------
void trx_set_am_mode(void)
{
  switch (controller_settings.trx[controller_settings.radioprofile].radio)
  {
    case 0:                            // ICOM
    case 1:
      icom_set_mode(2, icom_filter);   // 2 = AM mode
      break;
    case 2:                            // Kenwood
    case 3:                            // Kenwood
    case 4:
      ts2000_set_mode(5);              // 5 = AM mode
      break;
    case 5:                            // Yaesu FT-100
      ft100_set_mode(0x04);            // 4 = AM mode
      break;
    case 6:                            // Yaesu FT-7X7
      ft7x7_set_mode(0x82);            // 82 = AM Narrow mode
      break;
    case 7:                            // Yaesu FT-8X7
      ft8x7_set_mode(0x04);            // 4 = AM mode
      break;
    case 8:                            // Yaesu FT-920
      ft920_set_mode(0x04);            // AM Narrow
      break;
    case 9:                            // Yaesu FT-990
      ft990_set_mode(0x0005);          // AM 2.4 kHz
      break;
    case 10:                            // Yaesu FT-1000MP
      ft1000MP_set_mode(0x0004);       // AM 2.4 kHz
      break;
    case 11:                            // Yaesu FT-1000MP Mk-V
      ft1000MPmkV_set_mode(0x0004);    // AM 2.4 kHz
      break;
    case 12:                            // Yaesu FT-450/950/1200/2000...
      ft2000_set_mode(5);              // 5 = AM mode
      break;
    case 13:                           // Elecraft K3/KX3
    case 14:
      elecraft_set_mode(5);            // 5 = AM mode
      break;
    case 15:                           // TenTec binary style
      tentec_binary_set_mode(0);       // 0 = AM mode
      break;
    case 16:                           // TenTec ascii style
      tentec_ascii_set_mode(4);        // 4 = AM mode
      break;
    default:;                          // Pseudo-VFO - do nothing      
  }
}
//---------------------------------------------------------------------------------
// Restore Radio Mode
//---------------------------------------------------------------------------------
void trx_restore_mode(void)
{
  switch (controller_settings.trx[controller_settings.radioprofile].radio)
  {
    case 0:                            // ICOM
    case 1:
      icom_set_mode(trx_mode, icom_filter);
      break;
    case 2:                            // Kenwood
    case 3:
    case 4:
      ts2000_set_mode(trx_mode);
      break;
    case 5:                            // Yaesu FT-100
      ft100_set_mode(trx_mode);
      break;
    case 6:                            // Yaesu FT-7X7
      ft7x7_set_mode(trx_mode);
      break;
    case 7:                            // Yaesu FT-8X7
      ft8x7_set_mode(trx_mode);
      break;
    case 8:                            // Yaesu FT-920
      ft920_set_mode(trx_mode);
      break;
    case 9:                            // Yaesu FT-990
      ft990_set_mode(trx_mode);
      break;
    case 10:                            // Yaesu FT-1000MP
      ft1000MP_set_mode(trx_mode);
      break;
    case 11:                            // Yaesu FT-1000MP Mk-V
      ft1000MPmkV_set_mode(trx_mode);
      break;
    case 12:                            // Yaesu FT-450/950/1200/2000...
      ft2000_set_mode(trx_mode);
      break;
    case 13:                           // Elecraft K3/KX3
    case 14:
      elecraft_set_mode(trx_mode);
      break;
    case 15:                           // TenTec binary style
      tentec_binary_set_mode(trx_mode);
      break;
    case 16:                           // TenTec ascii style
      tentec_ascii_set_mode(trx_mode);   
      break;
    default:;                          // Pseudo-VFO - do nothing   
  }
}   

//---------------------------------------------------------------------------------
// Switch Radio to Transmit
//---------------------------------------------------------------------------------
void trx_set_tx(void)
{
  digitalWrite(hardware_ptt, HIGH);    // Set Hardware PTT to On  
  
  switch (controller_settings.trx[controller_settings.radioprofile].radio)
  {
    case 0:                            // ICOM
    case 1:
      icom_set_tx(1);
      break;
    case 2:                            // Kenwood
      ts2000_set_tx();
      break;
    case 3:
      ts870_set_tx();
      break;
    case 4:
      ts2000_set_tx();
      break;
    case 5:                            // Yaesu FT-100
      ft100_set_tx();
      break;
    case 6:                            // Yaesu FT-7X7
      ft7x7_set_tx();
      break;
    case 7:                            // Yaesu FT-8X7
      ft8x7_set_tx();
      break;
    case 8:                            // Yaesu FT-920
      ft920_set_tx();
      break;
    case 9:                            // Yaesu FT-990
      ft990_set_tx();
      break;
    case 10:                            // Yaesu FT-1000MP
      ft1000MP_set_tx();
      break;
    case 11:                            // Yaesu FT-1000MP Mk-V
      ft1000MPmkV_set_tx();
      break;
    case 12:                            // Yaesu FT-450/950/1200/2000...
      ft2000_set_tx(1);
      break;
    case 13:                           // Elecraft K3/KX3
    case 14:
      elecraft_set_tx();
      break;
    case 15:                           // TenTec binary style
      tentec_binary_set_tx();
      break;
    case 16:                           // TenTec ascii style
      tentec_ascii_set_tx();   
      break;
    default:;                          // Pseudo-VFO - do nothing
  }
}

//---------------------------------------------------------------------------------
// Switch Radio to Receive
//---------------------------------------------------------------------------------
void trx_set_rx(void)
{
  switch (controller_settings.trx[controller_settings.radioprofile].radio)
  {
    case 0:                            // ICOM
    case 1:
      icom_set_tx(0);
      break;
    case 2:                            // Kenwood
      ts2000_set_rx();
      break;
    case 3:
      ts870_set_rx();
      break;
    case 4:
      ts2000_set_rx();
      break;
    case 5:                            // Yaesu FT-100
      ft100_set_rx();
      break;
    case 6:                            // Yaesu FT-7X7
      ft7x7_set_rx();
      break;
    case 7:                            // Yaesu FT-8X7
      ft8x7_set_rx();
      break;
    case 8:                            // Yaesu FT-920
      ft990_set_rx();
      break;
    case 9:                            // Yaesu FT-990
      ft990_set_rx();
      break;
    case 10:                            // Yaesu FT-1000MP
      ft1000MP_set_rx();
      break;
    case 11:                            // Yaesu FT-1000MP Mk-V
      ft1000MPmkV_set_rx();
      break;
    case 12:                            // Yaesu FT-450/950/1200/2000...
      ft2000_set_tx(0);
      break;
    case 13:                           // Elecraft K3/KX3
    case 14:
      elecraft_set_rx();
      break;
    case 15:                           // TenTec binary style
      tentec_binary_set_rx();
      break;
    case 16:                           // TenTec ascii style
      tentec_ascii_set_rx();   
      break;
    default:;                          // Pseudo-VFO - do nothing
  }  
  digitalWrite(hardware_ptt, LOW);     // Set Hardware PTT to Off
}

//---------------------------------------------------------------------------------
// Request Mode Setting from the Radio
//---------------------------------------------------------------------------------
uint8_t trx_request_mode(void)
{
  uint8_t implemented = WORKING;       // A means to indicate if command not supported
  
  switch (controller_settings.trx[controller_settings.radioprofile].radio)
  {
    case 0:                            // ICOM
    case 1:
      icom_request_mode();
      break;
    case 2:                            // Kenwood TS-440
      radio.mode = true;               // Radio Mode is always available, as it comes with the frequency info
      break;
    case 3:
    case 4:
      ts2000_request_mode();           // Kenwood TS-870 / TS-2000. Maybe redundant, we already have the mode info from the IF command
      break;
    case 5:                            // Yaesu FT-100
      radio.mode = true;               // Radio Mode is always available, as it comes with the frequency info
      break;
    case 6:                            // Yaesu FT-7X7
      radio.mode = true;               // Radio Mode is always available, as it comes with the frequency info
      break;
    case 7:                            // Yaesu FT-8X7
      radio.mode = true;               // Radio Mode is always available, as it comes with the frequency info
      break;
    case 8:                            // Yaesu FT-920
      radio.mode = true;               // Radio Mode is always available, as it comes with the frequency info
      break;
    case 9:                            // Yaesu FT-990
      radio.mode = true;               // Radio Mode is always available, as it comes with the frequency info
      break;
    case 10:                            // Yaesu FT-1000MP
      radio.mode = true;               // Radio Mode is always available, as it comes with the frequency info
      break;
    case 11:                            // Yaesu FT-1000MP MK-V
      radio.mode = true;               // Radio Mode is always available, as it comes with the frequency info
      break;
    case 12:                            // Yaesu FT-450/950/1200/2000...
      ft2000_request_mode();
      break;
    case 13:                           // Elecraft K3/KX3
    case 14:
      elecraft_request_mode();
      break;
    case 15:                           // TenTec binary style
      tentec_binary_request_mode();
      break;
    case 16:                           // TenTec ascii style
      tentec_ascii_request_mode();
      break;
    default:;                          // Pseudo-VFO - do nothing   
  }  
  return implemented;
}


//---------------------------------------------------------------------------------
// Initialize TRX parameters for a particular Transceiver (all to default)
// This is done whenever a new transceiver is selected
//---------------------------------------------------------------------------------
void trx_parameters_init(uint8_t which_trx)
{
  // Default to Radio 1 if invalid value detected
  if (controller_settings.trx[which_trx].radio > MAX_RADIO) controller_settings.trx[which_trx].radio = 1;
  
  // initialize UART serial
  controller_settings.trx[which_trx].sig_mode = default_uart_mode[controller_settings.trx[which_trx].radio];
  controller_settings.trx[which_trx].rs232rate =default_uart_baud[controller_settings.trx[which_trx].radio];
  radio_selection = controller_settings.trx[which_trx].radio;
  rs232_signals = controller_settings.trx[which_trx].sig_mode + (controller_settings.trx[which_trx].passthrough << 1);
  rs232_rate = controller_settings.trx[which_trx].rs232rate;
 
  // Set and Store Default Tune Power level for default radio
  controller_settings.trx[which_trx].tx_pwrlevel = default_plevel[controller_settings.trx[which_trx].radio];
  EEPROM_writeAnything(1,controller_settings);   // write controller settings into eeprom
  tx_tune_pwr = controller_settings.trx[which_trx].tx_pwrlevel;
}

//---------------------------------------------------------------------------------
// Set TRX parameters according to the current controller settings
//---------------------------------------------------------------------------------
void trx_parameters_set(uint8_t which_trx)
{
  // Enforce a valid value (1200 b/s) in case of invalid value detected
  if (controller_settings.trx[which_trx].rs232rate > 7) controller_settings.trx[which_trx].rs232rate = 0; // 7 = max = 115200 b/s
  
  // initialize UART serial rate (1200,2400,4800,9600,19200,38400,57600,115200) and TTL or RS232 mode
  uint32_t rs232rate;
  if (controller_settings.trx[which_trx].rs232rate < 6) rs232rate = 1200 * (1 << controller_settings.trx[which_trx].rs232rate);  // 1200 - 38200 b/s
  else if (controller_settings.trx[which_trx].rs232rate == 6) rs232rate = 57600;      // 57600 b/s
  else rs232rate = 115200;                                             // 115200 b/s
  Uart.begin(rs232rate, (controller_settings.trx[which_trx].sig_mode==0) ? 
         valid_uart_config[controller_settings.trx[which_trx].radio] : valid_uart_config_inv[controller_settings.trx[which_trx].radio]);
         
  // For ICOM Radios set UART output with Open Drain and Pullup
  //
  if (controller_settings.trx[which_trx].radio < 2)
  {
    *portConfigRegister(Uart_TXD) |= PORT_PCR_ODE;                     // Open Drain Enable
    *portConfigRegister(Uart_RXD) |= (PORT_PCR_PE | PORT_PCR_PS);      // Pullup Enable
  }
  // Set UART function normal for all other Radios - Needed if switching back from ICOM
  else
  {
    *portConfigRegister(Uart_TXD) &= ~PORT_PCR_ODE;                   // Open Drain Disable
    *portConfigRegister(Uart_RXD) &= ~(PORT_PCR_PE | PORT_PCR_PS);    // Pullup Disable
  }
  
  // BOOL for TRX serial port - TRUE = Asynchronous serial read mode
  async_uart_read = async[controller_settings.trx[which_trx].radio];
  // Seed metro with the specific settings of the selected Radio
  trxpollMetro = Metro(poll_rate[controller_settings.trx[which_trx].radio]);
}


//---------------------------------------------------------------------------------
// Change active TRX profile - Update all settings relevant to the active profile
//---------------------------------------------------------------------------------
void trx_profile_update(void)
{
  // Set out bits to indicate which Radio Settings Profile is active
  if (controller_settings.radioprofile & 0x01) digitalWrite(profile_bit1, HIGH);
  else digitalWrite(profile_bit1, LOW);
  if (controller_settings.radioprofile & 0x02) digitalWrite(profile_bit2, HIGH);
  else digitalWrite(profile_bit2, LOW);

  radio_selection = controller_settings.trx[controller_settings.radioprofile].radio;
  rs232_signals = controller_settings.trx[controller_settings.radioprofile].sig_mode + (controller_settings.trx[controller_settings.radioprofile].passthrough << 1);
  rs232_rate = controller_settings.trx[controller_settings.radioprofile].rs232rate;  
  tx_tune_pwr = controller_settings.trx[controller_settings.radioprofile].tx_pwrlevel; 
  // Activate the changed profile
  trx_parameters_set(controller_settings.radioprofile);
}
