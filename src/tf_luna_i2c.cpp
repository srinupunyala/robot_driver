#include "tf_luna_i2c.h"

#include <sensor_msgs/Range.h>
#include <ros/console.h>

TFLuna::TFLuna(ros::NodeHandle& nh): m_range(200), m_signal_strength(6000), m_chip_temperate(0),
        m_nh(nh), m_base_frame_id("base_link"), m_range_frame_id("ir_range") {
    Set_Trig_Mode();
    m_range_publisher = m_nh.advertise<sensor_msgs::Range>("ir_range", 10);
}

bool TFLuna::readFromSensor() {
    Set_Trigger();
    int16_t range=0;
    if (getData(range)) {
      m_range = range/100.0;
      return true;
    }
    return false;
}

void TFLuna::publish() {
    sensor_msgs::Range new_range;
    new_range.header.frame_id = m_range_frame_id;
    new_range.header.stamp = ros::Time::now();
    new_range.radiation_type = sensor_msgs::Range::INFRARED;
    new_range.max_range = 2.0;
    new_range.min_range = 0.0;
    new_range.range = (m_range > new_range.max_range) ? new_range.max_range :
                      ((m_range < new_range.min_range) ? new_range.min_range : m_range);
    new_range.field_of_view = 0.0349066;
    m_range_publisher.publish(new_range);   
}

void TFLuna::spinOnce(const ros::TimerEvent&) {
    if (readFromSensor())
        publish();
}

// - - - - - - - - - - - - - - - - - - - - - - - - - -
//             GET DATA FROM THE DEVICE
// - - - - - - - - - - - - - - - - - - - - - - - - - -
bool TFLuna::getData( int16_t &dist, int16_t &flux, int16_t &temp)
{
    tfStatus = TFL_READY;    // clear status of any error condition

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 1 - Use `readReg` to fill the six byte
    // `dataArray` from the contiguous sequence of registers `TFL_DIST_LO`
    // to `TFL_TEMP_HI` that declared in the header file 'TFLuna.h`.
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    for (uint8_t reg = TFL_DIST_LO; reg <= TFL_TEMP_HI; reg++)
    {
      if( !readReg(reg)) return false;
          else dataArray[reg] = regReply;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 2 - Shift data from read array into the three variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    dist = dataArray[ 0] + ( dataArray[ 1] << 8);
    flux = dataArray[ 2] + ( dataArray[ 3] << 8);
    temp = dataArray[ 4] + ( dataArray[ 5] << 8);

/*
    // Convert temperature from hundredths
    // of a degree to a whole number
    temp = int16_t( temp / 100);
    // Then convert Celsius to degrees Fahrenheit
    temp = uint8_t( temp * 9 / 5) + 32;
*/

    // - - Evaluate Abnormal Data Values - -
    // Signal strength <= 100
    if( flux < (int16_t)100)
    {
      tfStatus = TFL_WEAK;
      return false;
    }
    // Signal Strength saturation
    else if( flux == (int16_t)0xFFFF)
    {
      tfStatus = TFL_STRONG;
      return false;
    }
    else
    {
      tfStatus = TFL_READY;
      return true;
    }
}

// Get Data short version
bool TFLuna::getData( int16_t &dist)
{
  return getData( dist, m_signal_strength, m_chip_temperate);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - -
//              EXPLICIT COMMANDS
// - - - - - - - - - - - - - - - - - - - - - - - - - -

//  = =  GET DEVICE TIME (in milliseconds) = = =
//  Pass back time as an unsigned 16-bit variable
// bool TFLuna::Get_Time( uint16_t &tim, uint8_t adr)
// {
//     // Recast the address of the unsigned integer `tim`
//     // as a pointer to an unsigned byte `p_tim`...
//     uint8_t * p_tim = (uint8_t *) &tim;

//     // ... then address the pointer as an array.
//     if( !readReg( TFL_TICK_LO, adr)) return false;
//         else p_tim[ 0] = regReply;  // Read into `tim` array
//     if( !readReg( TFL_TICK_HI, adr)) return false;
//         else p_tim[ 1] = regReply;  // Read into `tim` array
//     return true;
// }

// //  = =  GET PRODUCTION CODE (Serial Number) = = =
// // When you pass an array as a parameter to a function
// // it decays into a pointer to the first element of the array.
// // The 14 byte array variable `tfCode` declared in the example
// // sketch decays to the array pointer `p_cod`.
// bool TFLuna::Get_Prod_Code( uint8_t * p_cod, uint8_t adr)
// {
//    for (uint8_t i = 0; i < 14; ++i)
//     {
//       if( !readReg( ( 0x10 + i), adr)) return false;
//         else p_cod[ i] = regReply;  // Read into product code array
//     }
//     return true;
// }

// //  = = = =    GET FIRMWARE VERSION   = = = =
// // The 3 byte array variable `tfVer` declared in the
// // example sketch decays to the array pointer `p_ver`.
// bool TFLuna::Get_Firmware_Version( uint8_t * p_ver, uint8_t adr)
// {
//     for (uint8_t i = 0; i < 3; ++i)
//     {
//       if( !readReg( ( 0x0A + i), adr)) return false;
//         else p_ver[ i] = regReply;  // Read into version array
//     }
//     return true;
// }

//  = = = = =    SAVE SETTINGS   = = = = =
// bool TFLuna::Save_Settings()
// {
//     return( writeReg( TFL_SAVE_SETTINGS, 1));
// }

// //  = = = =   SOFT (SYSTEM) RESET   = = = =
// bool TFLuna::Soft_Reset( uint8_t adr)
// {
//     return( writeReg( TFL_SOFT_RESET, adr, 2));
// }

// //  = = = = = =    SET I2C ADDRESS   = = = = = =
// // Range: 0x08, 0x77. Must reboot to take effect.
// bool TFLuna::Set_I2C_Addr( uint8_t adrNew, uint8_t adr)
// {
//     return( writeReg( TFL_SET_I2C_ADDR, adr, adrNew));
// }

// //  = = = = =   SET ENABLE   = = = = =
// bool TFLuna::Set_Enable( uint8_t adr)
// {
//     return( writeReg( TFL_DISABLE, adr, 1));
// }

// //  = = = = =   SET DISABLE   = = = = =
// bool TFLuna::Set_Disable( uint8_t adr)
// {
//     return( writeReg( TFL_DISABLE, adr, 0));
// }

//  = = = = = =    SET FRAME RATE   = = = = = =
// bool TFLuna::Set_Frame_Rate(uint16_t &frm)
// {
//     // Recast the address of the unsigned integer `frm`
//     // as a pointer to an unsigned byte `p_frm` ...
//     uint8_t * p_frm = (uint8_t *) &frm;

//     // ... then address the pointer as an array.
//     if( !writeReg( ( TFL_FPS_LO), p_frm[ 0])) return false;
//     if( !writeReg( ( TFL_FPS_HI), p_frm[ 1])) return false;
//     return true;
// }

// //  = = = = = =    GET FRAME RATE   = = = = = =
// bool TFLuna::Get_Frame_Rate( uint16_t &frm, uint8_t adr)
// {
//     uint8_t * p_frm = (uint8_t *) &frm;
//     if( !readReg( TFL_FPS_LO, adr)) return false;
//         else p_frm[ 0] = regReply;  // Read into `frm` array
//     if( !readReg( TFL_FPS_HI, adr)) return false;
//         else p_frm[ 1] = regReply;  // Read into `frm` array
//     return true;
// }

// //  = = = =   HARD RESET to Factory Defaults  = = = =
// bool TFLuna::Hard_Reset( uint8_t adr)
// {
//     return( writeReg( TFL_HARD_RESET, adr, 1));
// }

// //  = = = = = =   SET CONTINUOUS MODE   = = = = = =
// // Sample LiDAR chip continuously at Frame Rate
// bool TFLuna::Set_Cont_Mode( uint8_t adr)
// {
//     return( writeReg( TFL_SET_TRIG_MODE, adr, 0));
// }

// //  = = = = = =   SET TRIGGER MODE   = = = = = =
// Device will sample only once when triggered
bool TFLuna::Set_Trig_Mode()
{
    return( writeReg( TFL_SET_TRIG_MODE, 1));
}

//  = = = = = =   SET TRIGGER   = = = = = =
// Trigger device to sample once
bool TFLuna::Set_Trigger()
{
    return( writeReg( TFL_TRIGGER, 1));
}
//
// = = = = = = = = = = = = = = = = = = = = = = = =

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//       READ OR WRITE A GIVEN REGISTER OF THE SLAVE DEVICE
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
bool TFLuna::readReg( uint8_t nmbr)
{
  if (m_tfl_control.writeData(&nmbr, 1) == 1) {
    if (m_tfl_control.readBytes(&regReply, 1) == 1) {
        return true;
    } else {
        tfStatus = TFL_I2CREAD;
        return false;
    }
  } else {
    tfStatus = TFL_I2CWRITE;
    return false;
  }
}

bool TFLuna::writeReg( uint8_t nmbr, uint8_t data)
{
    if (m_tfl_control.writeData(&nmbr, 1) == 1 && m_tfl_control.writeData(&data, 1) == 1) {
        return true;
    } else {
        tfStatus = TFL_I2CWRITE;
        return false;
    }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - -    The following is for testing purposes    - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// Called by either `printFrame()` or `printReply()`
// Print status condition either `READY` or error type
// void TFLuna::printStatus()
// {
//     Serial.print("Status: ");
//     if( tfStatus == TFL_READY)          Serial.print( "READY");
//     else if( tfStatus == TFL_SERIAL)    Serial.print( "SERIAL");
//     else if( tfStatus == TFL_HEADER)    Serial.print( "HEADER");
//     else if( tfStatus == TFL_CHECKSUM)  Serial.print( "CHECKSUM");
//     else if( tfStatus == TFL_TIMEOUT)   Serial.print( "TIMEOUT");
//     else if( tfStatus == TFL_PASS)      Serial.print( "PASS");
//     else if( tfStatus == TFL_FAIL)      Serial.print( "FAIL");
//     else if( tfStatus == TFL_I2CREAD)   Serial.print( "I2C-READ");
//     else if( tfStatus == TFL_I2CWRITE)  Serial.print( "I2C-WRITE");
//     else if( tfStatus == TFL_I2CLENGTH) Serial.print( "I2C-LENGTH");
//     else if( tfStatus == TFL_WEAK)      Serial.print( "Signal weak");
//     else if( tfStatus == TFL_STRONG)    Serial.print( "Signal strong");
//     else if( tfStatus == TFL_FLOOD)     Serial.print( "Ambient light");
//     else if( tfStatus == TFL_INVALID)   Serial.print( "No Command");
//     else Serial.print( "OTHER");
// }


// // Print error type and HEX values
// // of each byte in the data frame
// void TFLuna::printDataArray()
// {
//     printStatus();
//     // Print the Hex value of each byte of data
//     Serial.print(" Data:");
//     for( uint8_t i = 0; i < 6; i++)
//     {
//       Serial.print(" ");
//       Serial.print( dataArray[ i] < 16 ? "0" : "");
//       Serial.print( dataArray[ i], HEX);
//     }
//     Serial.println();
// }
