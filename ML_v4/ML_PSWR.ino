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
// 			Power and SWR Meter routines
//-----------------------------------------------------------------------------------------
//

#if WIRE_ENABLED
//
//-----------------------------------------------------------------------------------------
//                    Scan I2C bus for AD7991 - 4x 12bit AD
//
// Return which type detected (AD7991-0 or AD7991-1) or 0 if nothing found
//-----------------------------------------------------------------------------------------
//
#define AD7991_0  0x28                // I2C Address of AD7991-0
#define AD7991_1  0x29                // I2C Address of AD7991-1

int8_t I2C_Init(void)
{
  int8_t found;                       // Returns status of I2C bus scan

  // Scan for AD7991-0 or AD7991-1
  ad7991_addr = AD7991_1;             // We start testing for an AD7991-1
  found = 2;						//  Assume it will be found
  Wire1.beginTransmission(ad7991_addr);
  if(Wire1.endTransmission() != 0)    // If error
  {
    ad7991_addr = AD7991_0;           // AD7991-0 was not detected
    found = 1;                        // We may have an AD7991-0	
  }
  Wire1.beginTransmission(ad7991_addr);
  if(Wire1.endTransmission() != 0)    // If error
  {
    ad7991_addr = 0;                  // AD7991-1 was not detected
    found = 0;                        // No I2C capable device was detected
  }

  // If Power & SWR Code and AD7991 12 bit ADC, the program it to use a 2.6V reference connected
  // to ADC channel 4 and only read ADC channels 1 and 2 consecutively.
  //
  if (found)                          // If AD7991 ADC was found, then configure it...
  {
    //uint8_t writePacket = 0x38;     // Set ADCs 1 and 2 for consecutive reads and REF_SEL = external
    //TWI_WritePacket(ad7991_addr,10,&writePacket,0,&writePacket,1);
    Wire1.beginTransmission(ad7991_addr);
    Wire1.write(0x38);                 // Set ADCs 1 and 2 for consecutive reads and REF_SEL = external
    Wire1.endTransmission();  
  }  
  return found;
}
#endif

//-----------------------------------------------------------------------------------------
//                            Poll the AD7991 2 x ADC chip
//                                  or alternately
//              use built-in 10 bit A/D converters if I2C device not present
//-----------------------------------------------------------------------------------------
//
// This function reads the A/D inputs
void adc_poll(void)
{
  ADC::Sync_result result;

  #if WIRE_ENABLED  
  uint16_t adc_in[4];
  uint8_t  read_B[4];
  uint8_t  i=0;
  
  //-----------------------------------------------------------------------------
  // use I2C connected AD7991 12-bit AD converter, if it was detected during init
  if (ad7991_addr)
  {  
    Wire1.requestFrom(ad7991_addr, 4);
    while (Wire1.available()) read_B[i++] = Wire1.readByte();

    // The output of the 12bit ADCs is contained in two consecutive byte pairs
    // read from the AD7991.  In theory, the second could be read before the first.
    // Each of the AD7991 four builtin ADCs has an address identifier (0 to 3)
    // in the uppermost 4 bits of the first byte.  The lowermost 4 bits of the first
    // byte are bits 8-12 of the A/D output.
    // In this routine we only read the two first ADCs, as set up in the I2C_Init()
    adc_in[(read_B[0] >> 4) & 0x03] = (read_B[0] & 0x0f) * 0x100 + read_B[1];
    adc_in[(read_B[2] >> 4) & 0x03] = (read_B[2] & 0x0f) * 0x100 + read_B[3];
    fwd = adc_in[0] * 2.6/3.25;  // My AD7991 implementation uses 2.6V reference
    ref = adc_in[1] * 2.6/3.25;  // This is a bit crude, loses some precision
  }
  else
  #endif
  //----------------------------------------------------------------------------
  // If no I2C, then use builtin A/D converters and convert to 12 bit resolution
  {
    result = adc->analogSynchronizedRead(Pref, Pfwd);  // ref=ADC0, fwd=ADC1
    if( (result.result_adc0 !=ADC_ERROR_VALUE) && (result.result_adc1 !=ADC_ERROR_VALUE) )
    {
      fwd = result.result_adc1;                   // We have good data from the ADs
      ref = result.result_adc0;
    }
    else  // error
    {
      fwd = -1;                                   // Should never happen
      ref = -1;
    }	
  }
}


//
//-----------------------------------------------------------------------------------------
// 			Calculate SWR if we have sufficient input power
//-----------------------------------------------------------------------------------------
//
void calculate_SWR(double v_fwd, double v_ref)
{
  // Only calculate SWR if meaningful power
  if (fwd_power_mw > MIN_PWR_FOR_SWR_CALC)
  {
    // Calculate SWR
    measured_swr = (1+(v_ref/v_fwd))/(1-(v_ref/v_fwd));
    if (measured_swr < 0) measured_swr = 9999;

    // prepare SWR bargraph value as a logarithmic integer value between 0 and 1000
    if (measured_swr < 10.0)
      swr_bar = 1000.0 * log10(measured_swr);
    else
      swr_bar = 1000;		// Show as maxed out above SWR 10:1
  }
  // If some power present, but not enough for an accurate SWR reading, then use
  // recent measured value
  else if (fwd_power_mw > MIN_PWR_FOR_SWR_SHOW)
  {
    // Do nothing, in other words, swr and swr_bar remain the same
  }
  else
  {
    // No power present, set SWR to high
    measured_swr = 9999;
    swr_bar = 0; // But don't show any panic on bargraph
  }

  // Assert SWR Alarm bit if SWR is above acceptable
  if (measured_swr > (controller_settings.swr_ok + 10)/10.0)
    digitalWrite(swralarm_bit, HIGH);  // Assert SWR Alarm 
  else 
    digitalWrite(swralarm_bit, LOW);   // DeAssert SWR Alarm 
}


//-----------------------------------------------------------------------------
// AD8307 specific Power measurement functions
#if AD8307_INSTALLED
//
//-----------------------------------------------------------------------------------------
//                Convert Voltage Reading into Power
//-----------------------------------------------------------------------------------------
//
void pswr_determine_dBm(double *FdBm, double *RdBm)
{
  double  delta_db;

  int16_t delta_F, delta_R;
  double  delta_Fdb, delta_Rdb;
  double  temp;

  // Calculate the slope gradient between the two calibration points:
  //
  // (dB_Cal1 - dB_Cal2)/(V_Cal1 - V_Cal2) = slope_gradient
  //
  delta_db = (double)((controller_settings.cal_AD8307[1].db10m - controller_settings.cal_AD8307[0].db10m)/10.0);
  delta_F = controller_settings.cal_AD8307[1].Fwd - controller_settings.cal_AD8307[0].Fwd;
  delta_Fdb = delta_db/delta_F;
  delta_R = controller_settings.cal_AD8307[1].Rev - controller_settings.cal_AD8307[0].Rev;
  delta_Rdb = delta_db/delta_R;
  //
  // measured dB values are: (V - V_Cal1) * slope_gradient + dB_Cal1
  *FdBm = (fwd - controller_settings.cal_AD8307[0].Fwd) * delta_Fdb + controller_settings.cal_AD8307[0].db10m/10.0;
  *RdBm = (ref - controller_settings.cal_AD8307[0].Rev) * delta_Rdb + controller_settings.cal_AD8307[0].db10m/10.0;

  // Test for direction of power - Always designate the higher power as "forward"
  // while setting the "Reverse" flag on reverse condition.
  if (*FdBm > *RdBm)                    // Forward direction
  {
    Reverse = false;
  }
  else                                  // Reverse direction
  {
    temp = *RdBm;
    *RdBm = *FdBm;
    *FdBm = temp;
    Reverse = true;
  }
}

//
//-----------------------------------------------------------------------------------------
// 			Calculate all kinds of power
//-----------------------------------------------------------------------------------------
//
void calculate_pep_and_pk(void)
{
  // For measurement of peak and average power
  static int16_t db_buff[PEP_BUFFER];           // dB voltage information in a one second window
  static int16_t db_buff_short[BUF_SHORT];      // dB voltage information in a 100 ms window
  static uint16_t a=0;                          // PEP ring buffer counter
  static uint8_t b=0;                           // 100ms ring buffer counter
  int16_t max=-32767, pk=-32767;                // Keep track of Max (1s) and Peak (100ms) dB voltage
  double power_db;                              // Calculated power in dBm
  double power_db_pk;                           // Calculated 100ms peak power in dBm
  double power_db_pep;                          // Calculated PEP power in dBm

  power_db = 10 * log10(power_mw);

  // Find peaks and averages
  // Multiply by 100 to make suitable for integer value
  // Store dB value in two ring buffers
  db_buff[a] = db_buff_short[b] = 100 * power_db;
  // Advance PEP (1, 2.5 or 5s) and 100ms ringbuffer counters
  a++, b++;
  if (a >= controller_settings.PEP_period) a = 0;
  if (b == BUF_SHORT) b = 0;

  // Retrieve Max Value within a 1 second sliding window
  for (uint16_t x = 0; x < controller_settings.PEP_period; x++)
  {
    if (max < db_buff[x]) max = db_buff[x];
  }

  // Find Peak value within a 100ms sliding window
  for (uint8_t x = 0; x < BUF_SHORT; x++)
  {
    if (pk < db_buff_short[x]) pk = db_buff_short[x];
  }

  // PEP
  power_db_pep = max / 100.0;
  power_mw_pep = pow(10,power_db_pep/10.0);

  // Peak (100 milliseconds)
  power_db_pk = pk / 100.0;
  power_mw_pk = pow(10,power_db_pk/10.0);
}

//
//-----------------------------------------------------------------------------------------
// Measure Forward and Reflected power and process
// Polled once every 5ms normally.  However, when stepper is in use,
// then polled once for each step progressed
//-----------------------------------------------------------------------------------------
//
void measure_power_and_swr(void)
{
  double f_inst;                                // Calculated Forward voltage
  double r_inst;                                // Calculated Reverse voltage
  double ad8307_FdBm;                           // Measured AD8307 forward voltage in dBm
  double ad8307_RdBm;                           // Measured AD8307 reverse current in dBm

  adc_poll();
  pswr_determine_dBm(&ad8307_FdBm, &ad8307_RdBm);
  
  // Instantaneous forward voltage and power, milliwatts and dBm
  f_inst = pow(10,ad8307_FdBm/20.0);		// (We use voltage later on, for SWR calc)
  fwd_power_mw = SQR(f_inst);			// P_mw = (V*V) (current and resistance have already been factored in

  // Instantaneous reverse voltage and power
  r_inst = pow(10,(ad8307_RdBm)/20.0);
  ref_power_mw = SQR(r_inst);

  // Instantaneous Real Power Output
  power_mw = fwd_power_mw - ref_power_mw;

  if (!flag.stepper_active)            // We only do this if the Stepper is not in use
  {                                    // If in use, then PSWR is measured at a different 
    calculate_pep_and_pk();            // rate, hence this would make limited sense
  }
  
  calculate_SWR(f_inst, r_inst);       // Calculate measured_swr based on forward and reflected voltages
}


//-----------------------------------------------------------------------------
// Diode detector specific Power measurement functions
#else
//
//-----------------------------------------------------------------------------------------
// 			Calculate all kinds of power
//-----------------------------------------------------------------------------------------
//
void calculate_pep_and_pk(int32_t p)
{
  // For measurement of peak (100ms pep) and pep (1s pep) power
  static int32_t buff[PEP_BUFFER];              // voltage information in a one second window
  static int32_t buff_short[BUF_SHORT];         // voltage information in a 100 ms window
  static uint16_t a=0;                          // PEP ring buffer counter
  static uint8_t b=0;                           // 100ms ring buffer counter
  int32_t mx=0, pk=0;                           // Keep track of Max (1s) and Peak (100ms) voltage

  // Find peaks and averages
  // Store power level (mw) in two ring buffers
  buff[a] = buff_short[b] = p;
  // Advance PEP [1] and Peak [100ms] ringbuffer counters
  a++, b++;
  if (a >= controller_settings.PEP_period) a = 0;
  if (b >= BUF_SHORT) b = 0;

  // Retrieve Max Value within a [1, 2.5 or 5] second sliding window
  for (uint16_t x = 0; x < controller_settings.PEP_period; x++)
  {
    if (mx < buff[x]) mx = buff[x];
  }

  // Find Peak value within a [100ms] sliding window
  for (uint8_t x = 0; x < BUF_SHORT; x++)
  {
    if (pk < buff_short[x]) pk = buff_short[x];
  }

  power_mw_pep = mx;  // PEP  (largest value measured in a sliding window of typically 1 sec)
  power_mw_pk = pk;   // Peak (typically 100 milliseconds)
}

//
//---------------------------------------------------------------------------------
// Measure Forward and Reflected power and process
// Polled once every 5ms normally.  However, when stepper is in use,
// then polled once for each step progressed
//---------------------------------------------------------------------------------
//

void measure_power_and_swr(void)
{
  double v_fwd;                                 // Calculated Forward voltage
  double v_ref;                                 // Calculated Reflected voltage
  uint16_t temp;

  adc_poll();                                   // Measure AD inputs
  
  // Test for direction of power - Always designate the higher power as "forward"
  // while setting the "Reverse" flag on reverse condition.
  if (fwd > ref)                                // Forward direction
  {
    Reverse = false;
  }
  else                                          // Reverse direction
  {
    temp = ref;
    ref = fwd;
    fwd = temp;
    Reverse = true;
  }

  // Instantaneous forward voltage and power, milliwatts
  //
  // Establish actual measured voltage at diode
  v_fwd = fwd * 3.3/4096.0 * (VALUE_R15 + VALUE_R17)/VALUE_R15;
  // Convert to VRMS in Bridge
  if (v_fwd >= D_VDROP) v_fwd = 1/1.4142135 * (v_fwd - D_VDROP) + D_VDROP;
  // Take Bridge Coupling into account
  v_fwd = v_fwd * BRIDGE_COUPLING * controller_settings.meter_cal/100.0;
  // Convert into milliwatts
  fwd_power_mw = 1000 * v_fwd*v_fwd/50.0;
  	
  // Instantaneous reflected voltage and power
  //
  // Establish actual measured voltage at diode
  v_ref = ref * 3.3/4096.0 * (VALUE_R15 + VALUE_R17)/VALUE_R15;
  // Convert to VRMS in Bridge
  if (v_ref >= D_VDROP) v_ref = 1/1.4142135 * (v_ref - D_VDROP) + D_VDROP;
  // Take Bridge Coupling into account
  v_ref = v_ref * BRIDGE_COUPLING * controller_settings.meter_cal/100.0;
  // Convert into milliwatts
  ref_power_mw = 1000 * v_ref*v_ref/50.0;
	
  // Instantaneous Real Power Output
  power_mw = fwd_power_mw - ref_power_mw;
  if (power_mw <  0) power_mw = power_mw * -1;

  if (!flag.stepper_active)            // We only do this if the Stepper is not in use
  {                                    // If in use, then PSWR is measured at a different 
    calculate_pep_and_pk(power_mw);    // rate, hence this would make limited sense
  }
  
  calculate_SWR(v_fwd, v_ref);         // Calculate measured_swr based on forward and reflected voltages

  // Debug
  // A fake Power and SWR reading, where SWR is generared on basis of a Stepper Position
  #if FAKEPSWR
  int32_t fake_pos;
  if (meter.debug_fakepswr)
  {
    fwd_power_mw = 10000;
    power_mw = 9500 + random(0, 50);
    calculate_pep_and_pk(power_mw);
    // Change fakepswr_val into an absolute position
    fake_pos = dir_of_travel*1.6*fakepswr_val + min_preset[ant].Pos;
    // and fake an SWR dip around this position using a simple parabolic
    measured_swr = 3 * abs(stepper_track[ant] - fake_pos);
    if (measured_swr > 400) measured_swr = 400;
    else
    {
      measured_swr = measured_swr/50;
      measured_swr = 1.2 + measured_swr*measured_swr;
    }
    swr_bar = 1000.0 * log10(measured_swr);    
  }
  #endif
}
#endif
#endif
