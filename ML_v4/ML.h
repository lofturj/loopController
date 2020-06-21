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

#include <Arduino.h>

#define  VERSION "4.10"
#define  DATE    "2020-06-21"

//
//-----------------------------------------------------------------------------
// Features Selection
//-----------------------------------------------------------------------------  
//

//-----------------------------------------------------------------------------
// Definitions for Hardware implementation
//
//-----------------------------------------------------------------------------
#define DRV8825STEPPER     1    // Set as 0 if 2x Allegro A4975 stepper drivers
                                // Set as 1 if Pololu or StepStick (TI)n DRV8825
                                // or (Allegro) A4988 stepper controller
//-----------------------------------------------------------------------------
// Enable AD8307 option for Power/SWR meter if using 2x AD8307 log amp detector
#define AD8307_INSTALLED   0    // 0 or 1
//-----------------------------------------------------------------------------
// Experimental - Enable I2C for External AD7991 A/D bridge for Power/SWR meter 
#define WIRE_ENABLED       0    // 0 or 1 (I2C bus is on pins 29 and 30)                          

//-----------------------------------------------------------------------------
// Definitions for End Stop sensing
// If required, there are inputs from end-stop switches.  Here there are three
// potential scenarios, user selectable through the definition below:
//
// 1: Vacuum variable, no end-stop switches.  In this case one has to take care
//    that the stepper motor is just powerful enough to turn the capacitor but
//    not excessively more so. The Up/Down switches will not work beyond the
//    lowest/highest stored frequency/position.  To tune the capacitor beyond
//    an already "proven" range, one needs to turn the capacitor by using
//    the Encoder, and store new frequency/positions to extend the range.  
// 2: Vacuum variable, end-stop switchces.  All as 1) except no "intelligence" to
//    inhibit use of Up/Down buttons.  Can be used in "smart" mode with frequency
//    input from radio, or "dumb" mode with no frequency input.
// 3: Butterfly capacitor, no end-stops.  Otherwise same as 2).
#define ENDSTOP_OPT        1    // 1, 2 or 3

//-----------------------------------------------------------------------------
// Enable Power/SWR meter and SWR-Autotune
// (this will enable a resistor ladder network configuration for Pushbutton 1
//  to enable additional pushbuttons 4, 5 and 6.  It will also enable the
//  Power and SWR meter code, including AD-read of Forward and Reverse voltage
//  inputs.  It also enables the SWR Tune and SWR-Autotune functions)
#define PSWR_AUTOTUNE      0    // 0 or 1

//-----------------------------------------------------------------------------
// Stepper Motor Recalibrate
// The Encoder or Up/Down switches can be used at any time to retune the antenna
// to compensate for changes in the antenna resonance frequency due to changes in 
// temperature.  This however will result in the Radio frequency and the 
// "calculated" frequency derived from the stepper position no longer being in
// agreement. Stepper Motor Recalibration realigns the two.
// When PSWR_AUTOTUNE is not selected, Recalibration is enacted by a short push
// of Menu/Enact (SW1), unless disabled below - or unless in Pseudo VFO mode.
// If disabled, then the equivalent of this function is still available by switching
// the Controller Off and then back On.
// When PSWR_AUTOTUNE is selected, then SW6 may be used for the Stepper Motor
// Recalibration instead of SW1.  
// When Pseudo VFO mode is selected as a Radio, then Encoder toggle for
// Pseudo VFO overrides the Recalibrate function for access to SW1.
#define RECALIBRATE        1    // 0 or 1. Enable Stepper Recalibrate, normally enabled

#define SW6_FOR_RECAL      1    // 0 OR 1 (does nothing unless PSWR_AUTOTUNE = 1)
                                // 1 for Stepper Recalibrate on SW6 and
                                // Pseudo VFO on SW1.  0 for Stepper Recalibrate on
                                // SW1 and Pseudo VFO on SW6.

//-----------------------------------------------------------------------------
// Use SW6 to cycle through the four available Radio Settings Profiles.
// See also Configuration Menu Item 7, "Radio Profiles".
// If this option is set as 1, then SW6_FOR_RECAL (above) should be set as 0
#define SW6_FOR_PROFILE    0    // 0 OR 1 (does nothing unless PSWR_AUTOTUNE = 1)

//-----------------------------------------------------------------------------
// Enable management of two or three antennas
// Two modes are available, selected with ANT_CHG_2BANKS:
// 1) single memory bank mode, for automatic changeover between two or three antennas with
//    non-overlapping frequency ranges.
// 2) dual memory bank mode, for automatic or manual changeover between two antennas.
// While mode 1 has the whole amount of memory available to share between antennas,
// mode 2 is a bit less efficient, splits the frq/pos memory into two equal size portions.
// The potential advantage of mode 2) is that it makes manual changeover possible.
#define ANT_CHG_2BANKS     0        // 0 or 1. Dual Memory Bank Mode
                                    // If ANT_CHG_2BANKS is 1 and ANT1_CHANGEOVER is 0, then
                                    // Manual Antenna Changeover by using a SPDT switch to ground
                                    // pin 27 (pad 27 underneath Teensy).
                                    //
#define ANT1_CHANGEOVER    0        // Default is 0
//#define ANT1_CHANGEOVER 11000000  // Example, 11 MHz
                                    // ANT1_CHANGEOVER is 0 for no changeover (mode 1) or manual
                                    // changeover (mode 2); otherwise it should indicate the 
                                    // automatic changeover frequency in Hz (mode 1 and mode 2).
//
// Second Antenna Changeover Point is only available if ANT_CHG_2BANKS is 0
// If not used, then this has to be set higher than highest frequency, e.g. 100000000 for 100 MHz
#define ANT2_CHANGEOVER 100000000   // Default is 100 MHz - out of harms way
//#define ANT2_CHANGEOVER 22000000  // Example, 22 MHz

//-----------------------------------------------------------------------------
// Bandswitching - Use for switch-in, switch-out of fixed capacitors, feed taps
// or similar, if needed.  This option is always active, harmless if not used.
// This provides band switching information in binary form on pins 24 and 25
// (solderpads underneath the Teensy 3.2 Microcontroller)
// Frequency below BND1_CHANGEOVER will result in :         Pin24 = 0, Pin25 = 0
// Frequency btw BND1 and BND2 CHANGEOVER will result in :  Pin24 = 1, Pin25 = 0
// Frequency btw BND2 and BND3 CHANGEOVER will result in :  Pin24 = 0, Pin25 = 1
// Frequency above BND3_CHANGEOVER will result in:          Pin24 = 1, Pin25 = 1
// Antenna 1 in two or three antenna mode
#define ANT1_BND1_CHANGEOVER  5000000     // Example  5 MHz
#define ANT1_BND2_CHANGEOVER 10000000     // Example 10 MHz
#define ANT1_BND3_CHANGEOVER 20000000     // Example 20 MHz
// This is used in single antenna mode, or Antenna 2 in two or three antenna mode
#define ANT2_BND1_CHANGEOVER  5000000     // Example  5 MHz
#define ANT2_BND2_CHANGEOVER 10000000     // Example 10 MHz
#define ANT2_BND3_CHANGEOVER 20000000     // Example 20 MHz
// Third antenna in three antenna mode
#define ANT3_BND1_CHANGEOVER  5000000     // Example  5 MHz
#define ANT3_BND2_CHANGEOVER 10000000     // Example 10 MHz
#define ANT3_BND3_CHANGEOVER 20000000     // Example 20 MHz
//
// BND_CHG_BY_ACTUAL_FRQ can be set as either 0 or 1 if not using fixed capacitors.
#define BND_CHG_BY_ACTUAL_FRQ  1     // 0 to use "derived" frequency based on VVC position
//                                   // 1 to use actual frequency information from Radio
// Experimental, may still have bugs:
// Both BND_CHG_BY_ACTUAL_FRQ and FRQ_CALC_BY_RANGE need to be set as 1 if using switch-in/out of fixed capacitors.
#define FRQ_CALC_BY_RANGE      0     // 1 to derive frequency from position by using the
                                     // currently active "range" selected by frequency information
                                     // from transceiver rather than by using the "best-fit" position.
                                     // This method is necessary if using switch-in/switch-out fixed
                                     // capacitors.  However, it may cause strange readings for "derived"
                                     // frequency readout during tune.  Normally set as 0 to disable.

//-----------------------------------------------------------------------------
// Definitions for Screen Saver
#define SCREENSAVE_ACTIVATE   300    // 30 seconds for going into screensaver mode
                                     // Set as 0 to deactivate screensaver
#define SCREENSAVE_UPDATE      50    // Update screensaver once every 5 seconds
//
#define SCREENSAVE_CUST         0    // Normally 0 for disabled, 1 to enable.
                                     // Disabled for the last tuned frequency to be shown
#define SCREENSAVE_CUST_TEXT "TF3LJ/VE2AO"  // Custom Screensaver msg, max 20 char

//-----------------------------------------------------------------------------
// Intro Message string to print to the first 2 lines of the LCD at startup
// Keep under 20 charecters per line
#define STARTUPDISPLAY1    "Magnetic Loop..."
#define STARTUPDISPLAY2    "...Controller"

//-----------------------------------------------------------------------------
// Change the below to display your own callsign on the LCD during startup
// (line 3).  Make sure there are 20 characters in total, including spaces.
#define DISP_CALLSIGN      "TF3LJ / VE2AO       "

//-----------------------------------------------------------------------------
// The appropriate LCD indications below are automatically selected, based on
// the ENDSTOP_OPT choice.
// Indicate which mode is active in line four of the LCD upon startup
#if ENDSTOP_OPT == 1
#define STARTUPDISPLAY4    "Vacuum C, No EndStop"
#elif ENDSTOP_OPT == 2
#define STARTUPDISPLAY4    "End Stop Switches"
#elif ENDSTOP_OPT == 3
#define STARTUPDISPLAY4    "No End Stop Switches"
#endif

//-----------------------------------------------------------------------------
// Stepper Rate can be displayed in either RPM or Steps per Second.(full steps)
//
// The Stepper rate indicates the actual rate of movement, unaffected by the 
// current selection of microsteps.  
// Maximum rate of change is 1500 times per second.  This translates to 750 steps
// per second (225RPM) if 2 microsteps, 375 steps/second (112RPM) if 4 microsteps,
// or 188 steps/second (56RPM) if 8 microsteps.
#define STEPPER_SPEED_IN_RPM     1  // 1 = display in RPM, else 0
#define STEPPER_ANGLE          1.8  // Correct Step angle needs to be defined for
                                    // RPM and number of revolutions to display correctly

//-----------------------------------------------------------------------------
// Definitions SWR Autotune
#define SWRTUNE_TIMEOUT        300  // 30 seconds. SWR tune maximum time before fail
                                    // in units of 100ms
#define SWR_HUNT_RANGE         200  // Hunt range, each way around a midpoint - 
                                    // is in units of full steps
#define ACCEPTABLE_SWR         2.5  // Typically 2.0 - 2.5 : 1 - can be set/changed in Menu
#define SWR_SAMPLEBUF           32  // SWR is averaged over this many steps
                                    // for finding best dip in a middle pos
#define BUTTERFLY_MAX_TRAVEL 40000  // Max travel while SWR tune in Up/Down direction
                                    // number of steps (in units of 8 microsteps),
                                    // before giving up. Used with Butterfly Cap only 
#define ICOM_TUNE_PWR           30  // Minimum Power setting for Tune (10%)
#define FT2000_TUNE_PWR          0  // Minimum Power setting for Tune (5W)
#define ELECRAFT_TUNE_PWR       10  // Minimum Power setting for Tune (10W or 1W?)
#define TS870_TUNE_PWR          10  // Minimum Power setting for Tune (5W?) (5-25)
#define TS2000_TUNE_PWR         10  // Minimum Power setting for Tune (5W?) (5-25)
#define TENTEC_TUNE_PWR          5  // Minimum Power setting for Tune (5W?) (0-127)

//-----------------------------------------------------------------------------
// Defs for Power and SWR indication
//-----------------------------------------------------------------------------
// Bruene Bridge or Tandem Match parameters
//
// If Bruene Bridge, then:
// BRIDGE_COUPLING = N_transformer*50ohm/R_transformer/sqrt2
// (uncomment the below if using the Bruene Bridge described in the Building Instructions)
//#define BRIDGE_COUPLING    12.04  // Bruene Bridge [ 16*50/47)/1.414 = 12.04]
//#define METER_CAL           0.90  // Calibration fudge factor, Bruene Bridge
//#define VALUE_R15          18000  // Resistor values of R15 & R16 in voltage divider
//#define VALUE_R17          68000  // Resistor values of R17 & R18 in voltage divider
//#define D_VDROP             0.25  // Voltage Drop over Diode
//
// If Tandem Match, then:
// BRIDGE_COUPLING = N_transformer
// (if Bruene bridge, then comment the below with a "//" in front of each line.)
#define BRIDGE_COUPLING       20.0  // Tandem Match, 20 turns
#define METER_CAL             1.08  // Calibration fudge factor, Tandem Match
#define VALUE_R15            18000  // Resistor values of R15 & R16 in voltage divider
#define VALUE_R17            22000  // Resistor values of R17 & R18 in voltage divider
#define D_VDROP               0.25  // Voltage Drop over Diode

//-----------------------------------------------------------------------------
// AD8307 Power Meter default calibration values - if option enabled
//
//-----------------------------------------------------------------------------
// 20 to 1 Tandem Match with Power and SWR Meter
// (default defines are for a 30 to 1 Tandem Match)
#define TWENTYTOONE              1  // 1 to select
//
#if TWENTYTOONE                     // Defs when using a 20 to 1 coupler
#define CAL1_NOR_VALUE         400  // 40 dBm, default dBm level1 for both AD8307
#define CAL2_NOR_VALUE         100  // 10 dBm, default dBm level2 for both AD8307
#define CALFWD1_RAW_DEFAULT   2627  // Default raw Voltage level1 at  40 dBm
#define CALREV1_RAW_DEFAULT   2627  // Default raw Voltage level1 at  40 dBm
#define CALFWD2_RAW_DEFAULT   1696  // Default raw Voltage level2 at  10 dBm
#define CALREV2_RAW_DEFAULT   1696  // Default raw Voltage level2 at  10 dBm
                                    //   to be allowed to calibrate
#else                               // Defs when using a 30 to 1 coupler
#define CAL1_NOR_VALUE         400  // 40 dBm, default dBm level1 for both AD8307
#define CAL2_NOR_VALUE         100  // 10 dBm, default dBm level2 for both AD8307
#define CALFWD1_RAW_DEFAULT   2517  // Default raw Voltage level1 at  40 dBm
#define CALREV1_RAW_DEFAULT   2517  // Default raw Voltage level1 at  40 dBm
#define CALFWD2_RAW_DEFAULT   1586  // Default raw Voltage level2 at  10 dBm
#define CALREV2_RAW_DEFAULT   1586  // Default raw Voltage level2 at  10 dBm
#endif                              // End coupler selection
#define CAL_INP_QUALITY        300  // 400  // Minimum difference between the raw (12 bit) input voltages
                                    //   to be allowed to calibrate
//-----------------------------------------------------------------------------
// Power & SWR Meter behaviour
#define PEP_PERIOD             500  // Default value in units of 5 ms.  500 = 2.5 seconds
#define POWER_METER_TIME       600  // (units of 5ms) Revert back from Power Meter after
                                    //   3 seconds of no power
#define METER_IDLE_THRESH        3  // Power/SWR meter wake up threshold when AD8307,
                                    //   1=1uW, 2=10uW ... 5=10mW
#if AD8307_INSTALLED  // --------------Used with AD8307:
#define MIN_PWR_FOR_SWR_CALC   0.1  // Minimum Power in mW for SWR calculation and display
#define MIN_PWR_FOR_SWR_SHOW  0.01  // Minimum Power in mW for SWR indication (use recent value)
#else                 // --------------Used if no AD8307:
#define MIN_PWR_FOR_METER       20  // Minimum Power in mW for Power/SWR Meter indication on LCD
#define MIN_PWR_FOR_SWR_CALC    10  // Minimum Power in mW for SWR calculation and display
#define MIN_PWR_FOR_SWR_SHOW    10  // Minimum Power in mW for SWR indication (use recent value)
#endif
                                    
//-----------------------------------------------------------------------------
// PEP and PEAK sample buffer sizes. Sampling rate is 5ms, hence 200 samples are 1 second
#define	PEP_BUFFER            1000  // PEP Buffer size, 1000 for Max 5 second PEP measurement
#define	BUF_SHORT               10  // Buffer size for 50ms Peak (to compensate for very small caps in bridge)
// DEFS for Power and SWR Meter User Definable Scale Ranges
#define SCALE_A                  6  // User definable Scale Ranges, up to 3 ranges per decade                   
#define SCALE_B                 12  // e.g. ... 6W 12W 24W 60W 120W 240W ...
#define SCALE_C                 24  // If all values set as "2", then ... 2W 20W 200W ...
// Debug - A fake Power and SWR meter can be activated by USB command $fakepswr
#define FAKEPSWR                 1  // SWR is then generared on basis of Stepper Position


//-----------------------------------------------------------------------------
// Additional definitions for End Stop Sense Option 1, Vacuum variable, no end-stop switches
//
// EndStop Tolerance - Automatic stepper movements back into "known range"
// are allowed, if we are no further out than this number of steps
#define ENDSTOP_TOLERANCE      800  // 800 microsteps = 100 full steps
// Inhibit all movement if Frequency outside range:
// If inhibited, then no stepper movement at all when outside range. Useful to 
// inhibit unecessary movement when switching to a band outside the range of the antenna
#define MOVEMENT_OUTSIDE_FRQ     1  // 1 to inhibit movement, 0 to allow


//-----------------------------------------------------------------------------
// Definitions for the Radio connection for Frqeuency Input Information
//
// Usable Definitions for USART serial rate and configuration are available here:
// http://www.pjrc.com/teensy/td_uart.html
//
// Radio selection, where:
// (selection can be changed using the Menu function)
// 0  = ICOM CI-V Auto
// 1  = ICOM CI-V Poll
// 2  = Kenwood TS-440
// 3 =  Kenwood TS-870
// 4  = Kenwood TS-480/TS-590/TS-2000
// 5  = Yaesu FT-100
// 6  = Yaesu FT-7X7 (747...)
// 7  = Yaesu FT-8x7 (817,847,857,897)
// 8  = Yaesu FT-920
// 9  = Yaesu FT-990
// 10 = Yaesu FT-1000MP
// 11 = Yaesu FT-1000MP Mk-V
// 12 = Yaesu FT-450/950/1200/2000/3000/5000
// 13 = Elecraft K3/KX3 Auto
// 14 = Elecraft K3/KX3 Poll
// 15 = TenTec binary mode
// 16 = TenTec ascii mode
// 17 = Pseudo-VFO mode. SW1 or SW6 switches the Encoder between Normal mode and Pseudo-VFO mode
#define DEFAULT_RADIO             1  // Set to any value between 0 and 17, as per above.
#define MAX_RADIO                17  // Do not modify unless updating the ML_TRX code.

// Default Serial Port Rates
// (valid rates are 0=1200, 1=2400, 2=4800, 3=9600, 4=19200, 5=38400, 6=57600 and 7=115200)
#define DEFAULT_ICOM_BAUD         3  // Default for ICOM is 9600 b/s
#define DEFAULT_KENWOOD_BAUD      3  // Default for Kenwood is 9600 b/s
#define DEFAULT_FT100_BAUD        2  // Default for Yaesu FT-100 is 4800 b/s
#define DEFAULT_FT7X7_BAUD        2  // Default for Yaesu FT-7x7 is 4800 b/s
#define DEFAULT_FT8X7_BAUD        2  // Default for Yaesu FT-8x7 is 4800 b/s
#define DEFAULT_FT920_BAUD        2  // Default for Yaesu FT-920 is 4800 b/s
#define DEFAULT_FT990_BAUD        2  // Default for Yaesu FT-990 is 4800 b/s
#define DEFAULT_FT1000MP_BAUD     2  // Default for Yaesu FT-1000MP      is 4800 b/s
#define DEFAULT_FT1000MPMKV_BAUD  2  // Default for Yaesu FT-1000MP Mk-V is 4800 b/s
#define DEFAULT_FT450_BAUD        2  // Default for Yaesu FT-450/900/dx1200/2000... is 4800 b/s
#define DEFAULT_ELECRAFT_BAUD     2  // Default for Elecraft K3/KX3 is 4800 b/s
#define DEFAULT_TENTEC_BAUD       7  // Default for TenTec is 115200 b/s
#define DEFAULT_PSEUDOVFO_BAUD    7  // Not used, but need to have something here
//
// Default ICOM CI-V Address IC756proII ... only relevant if ICOM
#define CIV_TRX_ADDRESS       0x64
//
// Default Serial Port Parameters
// 0 = TTL polarity, 1 = Reverse polarity Serial (if RS232 without MAX232)
#define DEFAULT_ICOM_MODE         0  // TTL Mode
#define DEFAULT_KENWOOD_MODE      1  // RS232 Mode
#define DEFAULT_FT100_MODE        0  // TTL Mode
#define DEFAULT_FT7X7_MODE        0  // TTL Mode
#define DEFAULT_FT8X7_MODE        0  // TTL Mode
#define DEFAULT_FT920_MODE        1  // RS232 Mode
#define DEFAULT_FT990_MODE        0  // TTL Mode
#define DEFAULT_FT1000MP_MODE     1  // RS232 Mode
#define DEFAULT_FT1000MPMKV_MODE  1  // RS232 Mode
#define DEFAULT_FT450_MODE        1  // RS232 Mode
#define DEFAULT_ELECRAFT_MODE     1  // RS232 Mode
#define DEFAULT_TENTEC_MODE       1  // RS232 Mode
#define DEFAULT_PSEUDOVFO_MODE    1  // Not used, but need to have something here
//
// Valid Serial Port Parameters
#define ICOM_CONFIG            SERIAL_8N1              // TTL Polarity
#define ICOM_CONFIG_INV        SERIAL_8N1_RXINV_TXINV  // RS232 Polarity
#define KENWOOD8N2_CONFIG      SERIAL_8N2              // If Kenwood rate is 4800 b/s or lower,
#define KENWOOD8N2_CONFIG_INV  SERIAL_8N2_RXINV_TXINV  // then use 8N2
#define KENWOOD_CONFIG         SERIAL_8N1
#define KENWOOD_CONFIG_INV     SERIAL_8N1_RXINV_TXINV
#define FT100_CONFIG           SERIAL_8N2
#define FT100_CONFIG_INV       SERIAL_8N2_RXINV_TXINV
#define FT7X7_CONFIG           SERIAL_8N2
#define FT7X7_CONFIG_INV       SERIAL_8N2_RXINV_TXINV
#define FT8X7_CONFIG           SERIAL_8N2
#define FT8X7_CONFIG_INV       SERIAL_8N2_RXINV_TXINV
#define FT920_CONFIG           SERIAL_8N2
#define FT920_CONFIG_INV       SERIAL_8N2_RXINV_TXINV
#define FT990_CONFIG           SERIAL_8N2
#define FT990_CONFIG_INV       SERIAL_8N2_RXINV_TXINV
#define FT1000MP_CONFIG        SERIAL_8N2
#define FT1000MP_CONFIG_INV    SERIAL_8N2_RXINV_TXINV
#define FT1000MPMKV_CONFIG     SERIAL_8N2
#define FT1000MPMKV_CONFIG_INV SERIAL_8N2_RXINV_TXINV
#define FT450_CONFIG           SERIAL_8N2
#define FT450_CONFIG_INV       SERIAL_8N2_RXINV_TXINV
#define ELECRAFT_CONFIG        SERIAL_8N1
#define ELECRAFT_CONFIG_INV    SERIAL_8N1_RXINV_TXINV
#define TENTEC_CONFIG          SERIAL_8N1
#define TENTEC_CONFIG_INV      SERIAL_8N1_RXINV_TXINV
#define PSEUDOVFO_CONFIG       SERIAL_8N1  // Not used, but need to have something here
#define PSEUDOVFO_CONFIG_INV   SERIAL_8N1_RXINV_TXINV
//
// Poll the radio for frequency information every XXX milliseconds
// Set as 9999 for Manual Polling by pushing Enact Switch
#define ICOM_POLL_RATE         1000  // Poll once per second
#define KENWOOD_POLL_RATE      1000
#define FT100_POLL_RATE        1000
#define FT7X7_POLL_RATE        1000
#define FT8X7_POLL_RATE        1000
#define FT920_POLL_RATE        1000
#define FT990_POLL_RATE        9999  // Manual polling by push of enact Switch
#define FT1000MP_POLL_RATE     1000
#define FT1000MPMKV_POLL_RATE  1000
#define FT450_POLL_RATE         500  // Every other poll checks for which VFO is in use
#define ELECRAFT_POLL_RATE     1000
#define TENTEC_POLL_RATE       1000
#define PSEUDOVFO_POLL_RATE     200  // Pseudo VFO is polled like real radio, although a bit faster,
                                     // otherwise Stepper backlash will be rather over the top when
                                     // tuning VFO in downward direction
                                     // If no backlash, then set OK to set this at 5, for smooth steppertuning.

// Default LCD Serial Debug Style, TRUE for HEX, FALSE for ascii]
#define ICOM_DEBUG             true
#define KENWOOD_DEBUG          false
#define FT100_DEBUG            true
#define FT7X7_DEBUG            true
#define FT8X7_DEBUG            true
#define FT920_DEBUG            true
#define FT990_DEBUG            true
#define FT1000MP_DEBUG         true
#define FT1000MPMKV_DEBUG      true
#define FT450_DEBUG            false
#define ELECRAFT_DEBUG         false
#define TENTEC_BIN_DEBUG       true
#define TENTEC_ASCII_DEBUG     false
#define PSEUDOVFO_DEBUG        false // Not used, but need to have something here

//
// Third line LCD indication at startup
const char *radiotext[] = { "ICOM generic CI-V",
                            "ICOM CI-V poll",
                            "Kenwood TS-440",
                            "Kenwood TS-870",
                            "Kenwood TS-480/2000",
                            "Yaesu FT-100",
                            "Yaesu FT-747",
                            "Yaesu FT-8X7",
                            "Yaesu FT-920",
                            "Yaesu FT-990",
                            "Yaesu FT-1000MP",
                            "Yaesu FT-1000MP Mk-V",
                            "Yaesu FT-450/950...",
                            "Elecraft K3/KX3 Auto",
                            "Elecraft K3/KX3 Poll",
                            "TenTec binary mode",
                            "TenTec ascii mode",
                            "Pseudo-VFO mode"  };
                           
// Kenwood Transmit Mode Kludge: 
// For Newer versions of the TS-2000 as well as TS-480 and TS-590, the CAT transmit command is specified as "TX0;"
// However older versions of Kenwood TS-2000 only recognize "TX;".  The below kludge enables the long form, if required
#define KENWOOD_TX_KLUDGE    0    // 1 for "TX0;", 0 for "TX;"

//-----------------------------------------------------------------------------
// Definitions for Rotary Encoder and Pushbuttons
#define  ENC_MENURESDIVIDE  16    // Encoder resolution reduction when in Menu
#define  ENC_TUNERESDIVIDE   2    // Encoder resolution reduction when turning
                                  // stepper motor, 1 = no reduction.

#define  UP_DOWN_RATE       20    // Stepper rate reducer when using Up/Down switches
                                  // 0 = full speed, 4 = 1/4 speed, etc...
                                  
#define  ENACT_MIN          20    // Minimum Menu/Enact push for "short push" (x 1 ms)
#define  ENACT_MAX        1000    // Minimum Menu/Enact push for Menu Mode (x 1 ms)

//-----------------------------------------------------------------------------
// Definitions for Stepper Motor Backlash
// In order to counter Backlash or Slop in the coupling between the Stepper Motor
// and the Capacitor, always reposition the capacitor in an upwards tuning
// direction.  When tuning down, start by overshooting by BACKLASH number of steps,
// then tune back up by the same amount.  Number is in [steps x 5], so 5 equals 25
// steps.  Max is 80 which then equals 400 steps.
#define BACKLASH_ANGLE        5   // 5 equals 25 steps, which equals 360 * 25/200 = 45 degrees
                                  // There is surprisingly much backlash in a Vacuum Capacitor
                                  // even if the coupling mechanism is otherwise very rigid.
                                  // Typical required step value is 25 to 35 (5 - 7),
                                  // or even more if a geared stepper motor is used.        

//-----------------------------------------------------------------------------
// 68 + 8*NUM_PRESETS needs to be less than the total amount of EEPROM available 
#define MAX_PRESETS        200    // Max number of preset memories
//
// EEPROM settings Serial Number. Increment this number when firmware mods necessitate
// the clearing of all Frequency/Position memories stored in EEPROM at first boot after
// an upgrade
#define COLDSTART_REF      0x12   // When started, the firmware examines this "Serial Number
                                  // and enforces factory reset to clear all Controller
                                  // settings, as well as frequency and position memories.
                                  // To roll this value is useful if there is chance of a
                                  // mismatch due to restructuring of the EEPROM.
                                  // Else - the stepper may take on on epic journey... :)
                                  // COLDSTART_REF can be any unique number between
                                  // 0x01 and 0xfd.  0xfe is reserved for use by the firmware 
                                  // to clear frequency/position memory data while retaining
                                  // controller_settings.  Ref: Menu function "Clear All"
//------------------------------------------------------------------------
// EEPROM structure:
//
// Addr 0, COLDSTART_REF or Rewrite BYTE (0xfe indicates a "factory reset" rewrite all
// "memories" with default values upon reboot
// Addr  1-34, various controller settings - such as Radio type, backlash on/off etc...
// ... a lot of empty space ...
// Addr 100-103, antenna 1, active FRQ (int32_t)
// Addr 104-107, antenna 1, active POS (int32_t)
// Addr 108-111, antenna 2, active FRQ (int32_t)
// Addr 112-115, antenna 2, active POS (int32_t)
// Addr 116-119, antenna 3, active FRQ (int32_t)
// Addr 120-123, antenna 3, active POS (int32_t)
// Addr 124-127, antenna 1, delta_Pos (int32_t)
// Addr 128-131, antenna 2, delta_Pos (int32_t)
// Addr 132-135, antenna 3, delta_Pos (int32_t)
// Addr 136-139, antenna 1, stepper_track (int32_t)
// Addr 140-143, antenna 2, stepper_track (int32_t)
// Addr 144-147, antenna 3, stepper_track (int32_t)
// Addr 148-(8xNUM_PRESETS) preset.Frq and preset.Pos pairs - both antennas
// (each pair is 8 bytes, 2 x int32_t)
//------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// USB Virtual Serial Port definitions
//-----------------------------------------------------------------------------
//
// The below copied out of the Teensyduino library
//
// You can change these to have your device annunciate itself to the 
// operating system using its own unique name.  In case of the Windows
// operating system, these are only used before an INF file
// (driver install) is loaded.
#define STR_MANUFACTURER  L"TF3LJ/VE2AO"
#define STR_PRODUCT       L"loopController"

// Some operating systems, especially Microsoft Windows, may cache
// USB device info.  Changes to the device name may not update on
// the same computer unless the vendor or product ID numbers change,
// or the "bcdDevice" revision code is increased.

// All USB serial devices are supposed to have a serial number
// (according to Microsoft).  With Windows, a new COM port is created
// for every unique serial/vendor/product number combination.  If
// you program 2 identical boards with 2 different serial numbers
// and they are assigned COM7 and COM8, each will always get the
// same COM port number because Windows remembers serial numbers.
//
// On Mac OS-X, a device file is created automatically which
// incorperates the serial number, eg, /dev/cu-usbmodem12341
//
// Linux by default ignores the serial number, and creates device
// files named /dev/ttyACM0, /dev/ttyACM1... in the order connected.
// Udev rules (in /etc/udev/rules.d) can define persistent device
// names linked to this serial number, as well as permissions, owner
// and group settings.
#define STR_SERIAL_NUMBER L"12345"

// Mac OS-X and Linux automatically load the correct drivers.  On
// Windows, even though the driver is supplied by Microsoft, an
// INF file is needed to load the driver.  These numbers need to
// match the INF file.
#define VENDOR_ID         0x16C0
#define PRODUCT_ID        0x0483


//-----------------------------------------------------------------------------  
// Assign pins to inputs and outputs (Arduino style)
//-----------------------------------------------------------------------------  
//
// Switch input pins
// UART (serial port) uses Pins 0 and 1
const int8_t Uart_RXD      = 0;
const int8_t Uart_TXD      = 1;
#if PSWR_AUTOTUNE
const int EnactSW          = A9;  // Analog Input 9, Equals pin 23
#else
const int EnactSW          = 23;
#endif
const int UpSW             = 11;
const int DnSW             = 12;
// End Stop Input Pins
#if ENDSTOP_OPT == 2
const int EndStopLower     = 21;
const int EndStopUpper     = 22;
#endif
// Rotary Encoder pins
const int EncI             =  9;
const int EncQ             = 10;
// LCD pins
const int LCD_D4           =  5;
const int LCD_D5           =  6;
const int LCD_D6           =  7;
const int LCD_D7           =  8;
const int LCD_RS           =  2;
const int LCD_RW           =  3;
const int LCD_E            =  4;
// AD inputs for Forward and Reflected Power (SWR measurement)
const int Pfwd             = A10;
const int Pref             = A11;


// Two alternate Stepper Motor configurations
// A pair of Allegro A4975, or a Pololu (Texas Instruments) DRV8825 or (Allegro) A4988 
#if !DRV8825STEPPER
// Assign Output Pins to a pair of A4975 Steppers
const int PhA           = 20;
const int D2A           = 19;
const int D1A           = 18;
const int D0A           = 17;
const int PhB           = 16;
const int D2B           = 15;
const int D1B           = 14;
const int D0B           = 13;     // Pin 13 is the ledPin
#else
// Assign Output Pins to DRV8825, also applies for the A4988 Stepper
const int drv8825_dir   = 14;     // Direction pin
const int drv8825_step  = 15;     // Step pin (positive pulse of +1us for each step)
const int drv8825_ms2   = 16;     // Microstepping pin MS2
const int drv8825_ms1   = 17;     // Microstepping pin MS1
const int drv8825_enable= 18;     // Enable pin
// Note that if using DRV8825 or A4988 Stepper driver, then the 
// following additional pins are available for other use:
// 13, 19, 20
#endif

//-----------------------------------------------------------------------------
// TXD output, if anyone needs it.  Useful with some older ICOM Radios
// which do not implement PTT control over CI-V
const int hardware_ptt= 28;

//-----------------------------------------------------------------------------
// Antenna select output - if anyone needs it.
//
#define ANALOGOUTPIN     1    // 1 for pin A14, 0 for pin 27
//
#if ANALOGOUTPIN              // Normal configuration:
const int ant1_select =  A14; // Analog output pin used as a digital ouput pin
//
#else                         // Alternate configuration:
const int ant1_select =  27;  // Pad underneath the Teensy 3.1/3.2
#endif
const int ant2_select =  26;  // Pad underneath the Teensy 3.1/3.2

#if ANT_CHG_2BANKS && !ANT1_CHANGEOVER && ANALOGOUTPIN // 2 Memory banks, Manual Mode
const int ChgOvSW     =  27;  // Antenna Changeover, pad underneath the Teensy 3.1/3.2
#endif
const int bnd_bit1    =  24;  // Band switching signals, pads underneath the Teensy 3.1/3.2,
const int bnd_bit2    =  25;  // two binary signal pins for four bands.
const int profile_bit1=  31;  // Radio Profile switching  signals, pads underneath the Teensy 3.1/3.2,
const int profile_bit2=  32;  // two binary signal pins for four profiles.
const int swralarm_bit=  33;  // SWR alarm output whenever SWR is higher than Menu Preset

//
//-----------------------------------------------------------------------------
// Don't touch any of the stuff below - unless you really know what you're doing
//
// Miscellaneous software defines, functions and variables
//-----------------------------------------------------------------------------
//

//-----------------------------------------------------------------------------
// All kinds of operaion mode boolean flags - OK some have several states

typedef struct  {
          unsigned short_push          : 1; // Short Push Button Action
          unsigned stepper_timer       : 1; // stepper_timer and stepper_Active indicate whether tge
          unsigned stepper_active      : 1; // Stepper is active (goes low after 200ms of non-activity)
          unsigned stepper_recalibrate : 1; // Stepper recalibrate request received
          unsigned frq_timer           : 1; // FRQ_TIMER and FRQ_STORE are used to update EEPROM if
          unsigned frq_store           : 1; // Frequency is stable for a while after a recent change
                                            // (works similarly to stepper_active, 5 second timer.
          unsigned manual_move         : 1; // Move by manual up/down buttons, disable backlash
          #if ENDSTOP_OPT == 1              // --> Vacuum Capacitor mode , no end stops
          unsigned frq_xrange          : 1; // Frequency is out of range
          unsigned cap_lower_pos       : 1; // Lower capacitor at lowest programmed frq/pos
          unsigned cap_lower_endstop   : 1; // Lower capacitor beyond lowest programmed frq/pos, soft endstop activated
          unsigned cap_upper_pos       : 1; // Upper capacitor at lowest programmed frq/pos
          unsigned cap_upper_endstop   : 1; // Upper capacitor beyond highest programmed frq/pos, soft endstop activated
          #else                             // --> End Stop Switch mode
          unsigned endstop_lower       : 1; // Lower Endstop has been reached
          unsigned endstop_upper       : 1; // Upper Endstop has been reached
          #endif
          unsigned config_menu         : 1; // Configuration Menu Mode
          unsigned screensaver         : 1; // Screensaver
          unsigned screensaver_refresh : 1; // Refresh Screensaver
          unsigned menu_lcd_upd        : 1; // Refresh/Update LCD when in Menu Mode
          unsigned profile_upd         : 1; // Radio Profile Update.  Flag used together with Menu Exit timer
                                            // to write profile info to EEPROM - avoids unnecessary writes when toggling.
                } flags;

typedef struct  {
          unsigned tune_request        : 1; // SWR Tune requested
          unsigned tune                : 1; // SWR Tune in progress
          unsigned tune_status         : 2; // SWR Tune Progress Status, WORKING/SUCCESS/FAIL
          unsigned rfactive_mode       : 1; // SWR Tune requested while RF active, no need to set up Radio
          unsigned hunt_mode           : 1; // Hunt Mode active, hunts around a centerpoint
          unsigned up_mode_request     : 1; // Request Up mode
          unsigned up_mode             : 1; // Up Mode active, tunes up until endstop or timeout
          unsigned down_mode_request   : 1; // Request Up mode
          unsigned down_mode           : 1; // Down Mode active, tunes down until endstop or timeout
          unsigned fail_counter        : 2; // Indicate whether last 3 tune operations failed, prevent multiple autotunes
          unsigned lcd_snapshot        : 1; // Display on LCD the best SWR from an SWR tune operation
                } swrflags;
                
typedef struct  {
          unsigned timer               : 1; // Frequency information received from Radio
          unsigned online              : 1; // Receiving regular frequency updates from radio
          unsigned new_frq             : 1; // Indicates if frequency from Radio has changed (for screensaver)
          unsigned previous            : 1; // Monitor whether Radio Information just got back online
          unsigned pwr                 : 1; // Used with SWR Auto-Tune, radio power level setting
          unsigned mode                : 1; // Used with SWR Auto-Tune, active radio mode setting
          unsigned swr                 : 1; // Used with SWR Auto-Tune, measured swr by radio
          unsigned tuneinit            : 2; // Radio being initialized into SWR Auto-Tune mode
          unsigned tunefirst           : 1; // First iteration of Radio initialization
          unsigned ack                 : 1; // pos or neg acknowlegement from radio (used with some) 
          unsigned pwr_available       : 1; // Indicates FAIL if Radio responded with "command not recognized"
          unsigned debug_trx           : 1; // Send all Transceiver serial comms to USB port for debug
          unsigned debug_hex           : 1; // Same as above, Hexadecimal mode
          unsigned debug_to_lcd        : 1; // Print all Transceiver serial comms to LCD (hex if needed)
                } rflags;
                
 typedef struct  {
          unsigned power_detected      : 1; // For LCD. Power above threshold detected by Power Meter
          unsigned power_timer         : 1; // Used in conjunction with powr_detected to measure time
          unsigned usb_report_poll     : 1; // Poll Mode USB Serial Reporting of Power and SWR
          unsigned usb_report_cont     : 1; // Continuous USB Serial Reporting of Power and SWR
          unsigned usb_report_type     : 3; // Which type of Power report is active?
                 #define  REPORT_DATA     1 // Report Instantaneous Power (raw format) and SWR to USB
                 #define  REPORT_INST     2 // Report Instantaneous Power (formatted) and SWR to USB
                 #define  REPORT_PK       3 // Report Peak (100ms) Power and SWR to USB
                 #define  REPORT_PEP      4 // Report PEP (1s) Power and SWR to USB
                 #define  REPORT_LONG     5 // Report Power and SWR to USB, long Human Readable format
                 #define  REPORT_AD_DEBUG 6 // Report raw AD values
          //Debug
          unsigned debug_fakepswr      : 1; // Provide Power and SWR measurement simulation upon USB command request
          unsigned debug_swrdip        : 1; // USB print the last 32 SWR measurements found if SWR Tune success
                 } mflags; 
                 

//-----------------------------------------------------------------------------
// Calibration settings if using AD8307 option
typedef struct {
          int16_t  db10m;                     // Calibrate, value in dBm x 10
          int16_t  Fwd;                       // corresponding A/D value for AD8307 Forward output
          int16_t  Rev;                       // corresponding A/D value for AD8307 Reverse output
               }  cal_t;                      // (48 bits in total)

//-----------------------------------------------------------------------------
// Various controller settings to be stored in EEPROM
typedef struct {
          unsigned radio       : 5;      // 5 bits contain radio selection (max 32 radios)
          unsigned sig_mode    : 1;      // Normal or Inverted RS232 signals
          unsigned rs232rate   : 3;      // 3 bits, RS232 Data Rate, (0=1200, 1=2400, 2=4800 ...7=115200)
          unsigned passthrough : 1;      // Pass all data from USB (computer) to serial (radio) and vice versa
          uint8_t  ICOM_address;         // ICOM CI-V Address, only relevant for ICOM transceivers
          uint8_t  tx_pwrlevel;          // Power Level of Radio during Tune command
                } radio_settings;        // (26 bits in total)

typedef struct  {
          uint8_t  Scale[3];             // User settable Scal ranges, up to 3 ranges per decade
          cal_t    cal_AD8307[2];        // Calibration settings if using AD8307 option
          uint8_t  meter_cal;            // Calibration multiplier for diode detector option - 100=1.0
          uint16_t PEP_period;           // PEP envelope sampling time in 5ms increments (200 = 1 second)
          unsigned step_rate   : 4;      // four bits for Stepper rate (100 x step_rate per second)
          unsigned step_speedup: 2;      // two bits for Stepper variable rate speedup
          unsigned microsteps  : 2;      // two bits contain Stepper Motor microstep resolution
          unsigned NotInUse    : 1;      // Not Used
          unsigned swr_ok      : 5;      // Acceptable SWR level, 0 = 1.0, 31 = 4.1
          unsigned swrautotune : 1;      // On/Off, Automatically initiates a Hunt Tune if high SWR
          unsigned idle_thresh : 3;      // Power/SWR meter wake up threshold, 1=1uW, 2=10uW ... 5=10mW
          unsigned pseudo_vfo  : 1;      // Encoder as pseudo-VFO, if corresponding .radio setting matches

          unsigned radioprofile: 2;      // Selected profile (4 profiles, 0 - 3)
          radio_settings trx[4];         // Settings for the four Radio profiles
          uint8_t  backlash_angle;       // Backlash Compensation travel in steps times 10. (0 - 2500)
          // 277 bits in use - Just keeping tabs on some space for growth
                } settings;
                
//-----------------------------------------------------------------------------
// A structure of two 32 bit integers to hold running and memorized Frq and Pos information
typedef struct  {
          int32_t  Frq;                  // Frequency information in Hz        
          int32_t  Pos;                  // Position Information, referenced at 1000000
                } var_track;             // 64 bits, 8 bytes

//-----------------------------------------------------------------------------
// Bool stuff
#define WORKING    0 
#define DONE       1
#define SUCCESS    1
#define FAIL       2
#define NOPWR      3

//-----------------------------------------------------------------------------
// Soft Reset Teensy 3 style
#define RESTART_ADDR       0xE000ED0C
#define RESTART_VAL        0x5FA0004
#define SOFT_RESET()       ((*(volatile uint32_t *)RESTART_ADDR) = (RESTART_VAL))

//-----------------------------------------------------------------------------
// Macros
#ifndef SQR
#define SQR(x) ((x)*(x))
#endif
#ifndef ABS
#define ABS(x) ((x>0)?(x):(-x))
#endif

extern const uint16_t poll_rate[];
extern const uint16_t default_plevel[];
