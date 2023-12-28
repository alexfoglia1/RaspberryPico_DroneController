/*------------------------------------------------------------------------------
--                   HERCCULES On-Board Software Components                   --
--                                                                            --
--                           EQUIPMENT  HANDLERS                              --
--                                                                            --
--                            BNO055_IMU Source                               --
--                                                                            --
--            Copyright (C) 2022 Universidad Politécnica de Madrid            --
--                                                                            --
-- HERCCULES was developed by the Real-Time Systems Group at  the Universidad --
-- Politécnica de Madrid.                                                     --
--                                                                            --
-- HERCCULES is free software: you can redistribute it and/or modify it under --
-- the terms of the GNU General Public License as published by the Free Soft- --
-- ware Foundation, either version 3 of the License,  or (at your option) any --
-- later version.                                                             --
--                                                                            --
-- HERCCULES is distributed  in the hope that  it will be useful  but WITHOUT --
-- ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FIT- --
-- NESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more --
-- details. You should have received a copy of the GNU General Public License --
-- along with HERCCULES. If not, see <https://www.gnu.org/licenses/>.         --
--                                                                            --
------------------------------------------------------------------------------*/
#include "bno055.h"

#include <cstdio> // perror
#include <cmath>  // pow
#include <time.h> // sleep

// Registers
// - - - - -
namespace {
    // The value expected to be returned from Device_Id
    const uint8_t I_AM_BNO055 = 0xA0U;
    const uint8_t I_AM_GYR    = 0x0FU;
    const uint8_t I_AM_ACC    = 0xFBU;
    const uint8_t I_AM_MGT    = 0x32U;

    //  masks for unit selections with the BNO055_UNIT_SEL register
    const uint8_t AccelerationUnitsMask   = 0b00000001U;
    const uint8_t Angular_Rate_Units_Mask = 0b00000010U;
    const uint8_t Euler_Angle_Units_Mask  = 0b00000100U;
    const uint8_t Temperature_Units_Mask  = 0b00010000U;
    const uint8_t Pitch_Rotation_Convention_Mask = 0b10000000U;

    const uint8_t Operating_Mode_Mask = 0b00001111U;

    //  Page0 register definition start
    const uint8_t BNO055_CHIP_ID_ADDR       = 0x00U;
    const uint8_t BNO055_ACCEL_REV_ID_ADDR  = 0x01U;
    const uint8_t BNO055_MAG_REV_ID_ADDR    = 0x02U;
    const uint8_t BNO055_GYRO_REV_ID_ADDR   = 0x03U;
    const uint8_t BNO055_SW_REV_ID_LSB_ADDR = 0x04U;
    const uint8_t BNO055_SW_REV_ID_MSB_ADDR = 0x05U;
    const uint8_t BNO055_BL_REV_ID_ADDR     = 0x06U;

    // Page id register definition
    const uint8_t BNO055_PAGE_ID_ADDR = 0x07U;

    //  Accel data register
    const uint8_t BNO055_ACCEL_DATA_X_LSB_ADDR = 0x08U;
    const uint8_t BNO055_ACCEL_DATA_X_MSB_ADDR = 0x09U;
    const uint8_t BNO055_ACCEL_DATA_Y_LSB_ADDR = 0x0AU;
    const uint8_t BNO055_ACCEL_DATA_Y_MSB_ADDR = 0x0BU;
    const uint8_t BNO055_ACCEL_DATA_Z_LSB_ADDR = 0x0CU;
    const uint8_t BNO055_ACCEL_DATA_Z_MSB_ADDR = 0x0DU;

    // Mag data register
    const uint8_t BNO055_MAG_DATA_X_LSB_ADDR = 0x0EU;
    const uint8_t BNO055_MAG_DATA_X_MSB_ADDR = 0x0FU;
    const uint8_t BNO055_MAG_DATA_Y_LSB_ADDR = 0x10U;
    const uint8_t BNO055_MAG_DATA_Y_MSB_ADDR = 0x11U;
    const uint8_t BNO055_MAG_DATA_Z_LSB_ADDR = 0x12U;
    const uint8_t BNO055_MAG_DATA_Z_MSB_ADDR = 0x13U;

    // Gyro data registers
    const uint8_t BNO055_GYRO_DATA_X_LSB_ADDR = 0x14U;
    const uint8_t BNO055_GYRO_DATA_X_MSB_ADDR = 0x15U;
    const uint8_t BNO055_GYRO_DATA_Y_LSB_ADDR = 0x16U;
    const uint8_t BNO055_GYRO_DATA_Y_MSB_ADDR = 0x17U;
    const uint8_t BNO055_GYRO_DATA_Z_LSB_ADDR = 0x18U;
    const uint8_t BNO055_GYRO_DATA_Z_MSB_ADDR = 0x19U;

    // Euler data registers
    const uint8_t BNO055_EULER_H_LSB_ADDR = 0x1AU;
    const uint8_t BNO055_EULER_H_MSB_ADDR = 0x1BU;
    const uint8_t BNO055_EULER_R_LSB_ADDR = 0x1CU;
    const uint8_t BNO055_EULER_R_MSB_ADDR = 0x1DU;
    const uint8_t BNO055_EULER_P_LSB_ADDR = 0x1EU;
    const uint8_t BNO055_EULER_P_MSB_ADDR = 0x1FU;

    // Quaternion data registers
    const uint8_t BNO055_QUATERNION_DATA_W_LSB_ADDR = 0x20U;
    const uint8_t BNO055_QUATERNION_DATA_W_MSB_ADDR = 0x21U;
    const uint8_t BNO055_QUATERNION_DATA_X_LSB_ADDR = 0x22U;
    const uint8_t BNO055_QUATERNION_DATA_X_MSB_ADDR = 0x23U;
    const uint8_t BNO055_QUATERNION_DATA_Y_LSB_ADDR = 0x24U;
    const uint8_t BNO055_QUATERNION_DATA_Y_MSB_ADDR = 0x25U;
    const uint8_t BNO055_QUATERNION_DATA_Z_LSB_ADDR = 0x26U;
    const uint8_t BNO055_QUATERNION_DATA_Z_MSB_ADDR = 0x27U;

    // Linear acceleration data registers
    const uint8_t BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0x28U;
    const uint8_t BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR = 0x29U;
    const uint8_t BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR = 0x2AU;
    const uint8_t BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR = 0x2BU;
    const uint8_t BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR = 0x2CU;
    const uint8_t BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR = 0x2DU;

    // Gravity data registers
    const uint8_t BNO055_GRAVITY_DATA_X_LSB_ADDR = 0x2EU;
    const uint8_t BNO055_GRAVITY_DATA_X_MSB_ADDR = 0x2FU;
    const uint8_t BNO055_GRAVITY_DATA_Y_LSB_ADDR = 0x30U;
    const uint8_t BNO055_GRAVITY_DATA_Y_MSB_ADDR = 0x31U;
    const uint8_t BNO055_GRAVITY_DATA_Z_LSB_ADDR = 0x32U;
    const uint8_t BNO055_GRAVITY_DATA_Z_MSB_ADDR = 0x33U;

    // Temperature data register
    const uint8_t BNO055_TEMP_ADDR = 0x34U;

    // Status registers
    const uint8_t BNO055_CALIB_STAT_ADDR      = 0x35U;
    const uint8_t BNO055_SELFTEST_RESULT_ADDR = 0x36U;
    const uint8_t BNO055_INTR_STAT_ADDR       = 0x37U;

    const uint8_t BNO055_SYS_CLK_STAT_ADDR = 0x38U;
    const uint8_t BNO055_SYS_STAT_ADDR     = 0x39U;
    const uint8_t BNO055_SYS_ERR_ADDR      = 0x3AU;

    // Unit selection register
    const uint8_t BNO055_UNIT_SEL_ADDR    = 0x3BU;
    const uint8_t BNO055_DATA_SELECT_ADDR = 0x3CU;

    // Mode registers
    const uint8_t BNO055_OPR_MODE_ADDR = 0x3DU;
    const uint8_t BNO055_PWR_MODE_ADDR = 0x3EU;

    const uint8_t BNO055_SYS_TRIGGER_ADDR = 0x3FU;
    const uint8_t BNO055_TEMP_SOURCE_ADDR = 0x40U;

    // Axis remap registers
    const uint8_t BNO055_AXIS_MAP_CONFIG_ADDR = 0x41U;
    const uint8_t BNO055_AXIS_MAP_SIGN_ADDR   = 0x42U;

    // SIC registers
    const uint8_t BNO055_SIC_MATRIX_0_LSB_ADDR = 0x43U;
    const uint8_t BNO055_SIC_MATRIX_0_MSB_ADDR = 0x44U;
    const uint8_t BNO055_SIC_MATRIX_1_LSB_ADDR = 0x45U;
    const uint8_t BNO055_SIC_MATRIX_1_MSB_ADDR = 0x46U;
    const uint8_t BNO055_SIC_MATRIX_2_LSB_ADDR = 0x47U;
    const uint8_t BNO055_SIC_MATRIX_2_MSB_ADDR = 0x48U;
    const uint8_t BNO055_SIC_MATRIX_3_LSB_ADDR = 0x49U;
    const uint8_t BNO055_SIC_MATRIX_3_MSB_ADDR = 0x4AU;
    const uint8_t BNO055_SIC_MATRIX_4_LSB_ADDR = 0x4BU;
    const uint8_t BNO055_SIC_MATRIX_4_MSB_ADDR = 0x4CU;
    const uint8_t BNO055_SIC_MATRIX_5_LSB_ADDR = 0x4DU;
    const uint8_t BNO055_SIC_MATRIX_5_MSB_ADDR = 0x4EU;
    const uint8_t BNO055_SIC_MATRIX_6_LSB_ADDR = 0x4FU;
    const uint8_t BNO055_SIC_MATRIX_6_MSB_ADDR = 0x50U;
    const uint8_t BNO055_SIC_MATRIX_7_LSB_ADDR = 0x51U;
    const uint8_t BNO055_SIC_MATRIX_7_MSB_ADDR = 0x52U;
    const uint8_t BNO055_SIC_MATRIX_8_LSB_ADDR = 0x53U;
    const uint8_t BNO055_SIC_MATRIX_8_MSB_ADDR = 0x54U;

    // Accelerometer Offset registers
    const uint8_t ACCEL_OFFSET_X_LSB_ADDR = 0x55U;
    const uint8_t ACCEL_OFFSET_X_MSB_ADDR = 0x56U;
    const uint8_t ACCEL_OFFSET_Y_LSB_ADDR = 0x57U;
    const uint8_t ACCEL_OFFSET_Y_MSB_ADDR = 0x58U;
    const uint8_t ACCEL_OFFSET_Z_LSB_ADDR = 0x59U;
    const uint8_t ACCEL_OFFSET_Z_MSB_ADDR = 0x5AU;

    // Magnetometer Offset registers
    const uint8_t MAG_OFFSET_X_LSB_ADDR = 0x5BU;
    const uint8_t MAG_OFFSET_X_MSB_ADDR = 0x5CU;
    const uint8_t MAG_OFFSET_Y_LSB_ADDR = 0x5DU;
    const uint8_t MAG_OFFSET_Y_MSB_ADDR = 0x5EU;
    const uint8_t MAG_OFFSET_Z_LSB_ADDR = 0x5FU;
    const uint8_t MAG_OFFSET_Z_MSB_ADDR = 0x60U;

    // Gyroscope Offset registers
    const uint8_t GYRO_OFFSET_X_LSB_ADDR = 0x61U;
    const uint8_t GYRO_OFFSET_X_MSB_ADDR = 0x62U;
    const uint8_t GYRO_OFFSET_Y_LSB_ADDR = 0x63U;
    const uint8_t GYRO_OFFSET_Y_MSB_ADDR = 0x64U;
    const uint8_t GYRO_OFFSET_Z_LSB_ADDR = 0x65U;
    const uint8_t GYRO_OFFSET_Z_MSB_ADDR = 0x66U;

    // Radius registers
    const uint8_t ACCEL_RADIUS_LSB_ADDR = 0x67U;
    const uint8_t ACCEL_RADIUS_MSB_ADDR = 0x68U;
    const uint8_t MAG_RADIUS_LSB_ADDR   = 0x69U;
    const uint8_t MAG_RADIUS_MSB_ADDR   = 0x6AU;
}


// Auxiliary operations
// - - - - - - - - - - -
namespace {

    void delayMilliSecs(int msecs) {
        sleep_ms(msecs);
    }

    constexpr equipment_handlers::BNO055_IMU::CalibrationLevels toCalibLevelFrom (uint8_t raw) {
        if (raw == 0U) {
            return equipment_handlers::BNO055_IMU::UNCALIBRATED;
        } else if (raw == 1U || raw == 2U) {
            return equipment_handlers::BNO055_IMU::PARTIALLY_CALIBRATED;
        } else {
            return equipment_handlers::BNO055_IMU::FULLY_CALIBRATED;
        }
    }

    constexpr void rawTo3AxisMeasurement
        (uint8_t buffer [6],
         float   lsb,
         equipment_handlers::BNO055_IMU::SensorData &data)
    {
        auto newX {static_cast<int16_t>(buffer[0] | (buffer[1] << 8U))};
        auto newY {static_cast<int16_t>(buffer[2] | (buffer[3] << 8U))};
        auto newZ {static_cast<int16_t>(buffer[4] | (buffer[5] << 8U))};

        data.X = static_cast<float>(newX) / lsb;
        data.Y = static_cast<float>(newY) / lsb;
        data.Z = static_cast<float>(newZ) / lsb;
    }

    constexpr void rawTo4AxisMeasurement
        (uint8_t buffer [8],
         float   lsb,
         equipment_handlers::BNO055_IMU::Quaternion &data)
    {
        int16_t newW = static_cast<int16_t>(buffer[0] | (buffer[1] << 8U));
        int16_t newX = static_cast<int16_t>(buffer[2] | (buffer[3] << 8U));
        int16_t newY = static_cast<int16_t>(buffer[4] | (buffer[5] << 8U));
        int16_t newZ = static_cast<int16_t>(buffer[6] | (buffer[7] << 8U));

        data.W = static_cast<float>(newW) / lsb;
        data.X = static_cast<float>(newX) / lsb;
        data.Y = static_cast<float>(newY) / lsb;
        data.Z = static_cast<float>(newZ) / lsb;
    }
}

namespace equipment_handlers {

    // --  Initialization & reset ops  -----------------------------------------

    // ----------------
    // -- Initialize --
    // ----------------

    bool BNO055_IMU::initialize
        (uint8_t i2cAddress,
         i2c_inst_t* i2cBusID,
         BNO055_IMU::PowerModes powerMode,
         bool useExternalCrystal)
    {
        this->i2cAddress = i2cAddress;
        this->i2cBusID = i2cBusID;

        bool success = setMode(BNO055_IMU::OperatingModes::CONFIG);

        success = success && reset();

        // Set power register normal
        success = success &&
                  (i2cWriteByteToRegister(i2cBusID, i2cAddress, BNO055_PWR_MODE_ADDR, powerMode) != PICO_ERROR_GENERIC);
        delayMilliSecs(500);

        // Set register map page zero:
        success = success &&
                  (i2cWriteByteToRegister(i2cBusID, i2cAddress, BNO055_PAGE_ID_ADDR, 0U) != PICO_ERROR_GENERIC);
        delayMilliSecs(500);

        // Set trigger map page zero:
        success = success &&
                  (i2cWriteByteToRegister(i2cBusID, i2cAddress, BNO055_PAGE_ID_ADDR, 0U) != PICO_ERROR_GENERIC);
        delayMilliSecs(500);


        // An External clock can be selected by setting bit CLK_SEL in the SYSTEM_TRIGGER
        // register.
        uint8_t clkSel = useExternalCrystal ? 0x80U : 0x00U;
        success = success && (i2cWriteByteToRegister(i2cBusID, i2cAddress, BNO055_SYS_TRIGGER_ADDR, clkSel) != PICO_ERROR_GENERIC);
        delayMilliSecs(500);

        success = success && setMode(NDOF);

        success = success && testWhoAmI();

        return success;
    }

    // -----------
    // -- Reset --
    // -----------

    bool BNO055_IMU::reset() const {
        const uint8_t resetBit = 0x20U;
        bool success = i2c_write_blocking(i2cBusID, i2cAddress, &resetBit, 1, false) != PICO_ERROR_GENERIC;

        // Power-on-reset time for the BNO055 is 650 ms. Give it time to start.
        delayMilliSecs(650);
        return success;
    }

    // -------------------
    // -- Test who am I --
    // -------------------

    bool BNO055_IMU::testWhoAmI() const {
        uint8_t chipId;
        uint8_t accelId;
        uint8_t gyroId;
        uint8_t mgtlId;

        chipId = i2cReadByteFromRegister(i2cBusID, i2cAddress, BNO055_CHIP_ID_ADDR);
        accelId = i2cReadByteFromRegister(i2cBusID, i2cAddress, BNO055_ACCEL_REV_ID_ADDR);
        gyroId = i2cReadByteFromRegister(i2cBusID, i2cAddress, BNO055_GYRO_REV_ID_ADDR);
        mgtlId = i2cReadByteFromRegister(i2cBusID, i2cAddress, BNO055_MAG_REV_ID_ADDR);

        return  (chipId  == I_AM_BNO055)    &&
                   (accelId == I_AM_ACC)    &&
                   (gyroId  == I_AM_GYR)    &&
                   (mgtlId  == I_AM_MGT);
    }

    // --  Calibration Operations  ---------------------------------------------

    BNO055_IMU::CalibrationStates BNO055_IMU::getSensorsCalibration() const {
        uint8_t data;
        CalibrationStates result {};
        data = i2cReadByteFromRegister(i2cBusID, i2cAddress, BNO055_CALIB_STAT_ADDR);
        
        const uint8_t mask = 0b0011U;
        result.Platform      = toCalibLevelFrom((data >> 6U) & mask);
        result.Gyroscope     = toCalibLevelFrom((data >> 4U) & mask);
        result.Accelerometer = toCalibLevelFrom((data >> 2U) & mask);
        result.Magnetometer  = toCalibLevelFrom((data >> 0U) & mask);
        
        return result;
    }

    bool BNO055_IMU::calibrationComplete() const {
        CalibrationStates states = getSensorsCalibration();
        return states.Platform == FULLY_CALIBRATED
            && states.Gyroscope == FULLY_CALIBRATED
            && states.Accelerometer == FULLY_CALIBRATED
            && states.Magnetometer == FULLY_CALIBRATED;
    }

    BNO055_IMU::SensorOffsetValues BNO055_IMU::getSensorOffsets() {
        auto previous_mode  {getCurrentMode()};
        auto success {setMode(OperatingModes::CONFIG)};
        delayMilliSecs(25);

         BNO055_IMU::SensorOffsetValues offsets {};
        if (success && getCurrentMode() == OperatingModes::CONFIG) {
            offsets.accel_x = value_at(ACCEL_OFFSET_X_LSB_ADDR);
            offsets.accel_y = value_at(ACCEL_OFFSET_Y_LSB_ADDR);
            offsets.accel_z = value_at(ACCEL_OFFSET_Z_LSB_ADDR);

            offsets.gyro_x = value_at(GYRO_OFFSET_X_LSB_ADDR);
            offsets.gyro_y = value_at(GYRO_OFFSET_Y_LSB_ADDR);
            offsets.gyro_z = value_at(GYRO_OFFSET_Z_LSB_ADDR);

            offsets.mgm_x = value_at(MAG_OFFSET_X_LSB_ADDR);
            offsets.mgm_y = value_at(MAG_OFFSET_Y_LSB_ADDR);
            offsets.mgm_z = value_at(MAG_OFFSET_Z_LSB_ADDR);

            offsets.accel_radius = value_at(ACCEL_RADIUS_LSB_ADDR);
            offsets.mgm_radius   = value_at(MAG_RADIUS_LSB_ADDR);
        }

        setMode(previous_mode);

        return offsets;
    }

    bool BNO055_IMU::SetSensorsOffsets(const BNO055_IMU::SensorOffsetValues &offsets) {
        auto previous_mode  {getCurrentMode()};
        bool success = setMode(OperatingModes::CONFIG);
        delayMilliSecs(25);

        
        success = success && (i2cWriteWordToRegister(i2cBusID, i2cAddress, ACCEL_OFFSET_X_LSB_ADDR, offsets.accel_x) != PICO_ERROR_GENERIC);
        success = success && (i2cWriteWordToRegister(i2cBusID, i2cAddress, ACCEL_OFFSET_Y_LSB_ADDR, offsets.accel_y) != PICO_ERROR_GENERIC);
        success = success && (i2cWriteWordToRegister(i2cBusID, i2cAddress, ACCEL_OFFSET_Z_LSB_ADDR, offsets.accel_z) != PICO_ERROR_GENERIC);

        success = success && (i2cWriteWordToRegister(i2cBusID, i2cAddress, GYRO_OFFSET_X_LSB_ADDR, offsets.gyro_x) != PICO_ERROR_GENERIC);
        success = success && (i2cWriteWordToRegister(i2cBusID, i2cAddress, GYRO_OFFSET_Y_LSB_ADDR, offsets.gyro_y) != PICO_ERROR_GENERIC);
        success = success && (i2cWriteWordToRegister(i2cBusID, i2cAddress, GYRO_OFFSET_Z_LSB_ADDR, offsets.gyro_z) != PICO_ERROR_GENERIC);

        success = success && (i2cWriteWordToRegister(i2cBusID, i2cAddress, MAG_OFFSET_X_LSB_ADDR, offsets.mgm_x) != PICO_ERROR_GENERIC);
        success = success && (i2cWriteWordToRegister(i2cBusID, i2cAddress, MAG_OFFSET_Y_LSB_ADDR, offsets.mgm_y) != PICO_ERROR_GENERIC);
        success = success && (i2cWriteWordToRegister(i2cBusID, i2cAddress, MAG_OFFSET_Z_LSB_ADDR, offsets.mgm_z) != PICO_ERROR_GENERIC);

        success = success && (i2cWriteWordToRegister(i2cBusID, i2cAddress, ACCEL_RADIUS_LSB_ADDR, offsets.accel_radius) != PICO_ERROR_GENERIC);
        success = success && (i2cWriteWordToRegister(i2cBusID, i2cAddress, MAG_RADIUS_LSB_ADDR, offsets.mgm_radius) != PICO_ERROR_GENERIC);

        setMode(previous_mode);

        return success;
    }

    bool BNO055_IMU::RemapAxes(const BNO055_IMU::AxesRemapping remapping) {
        auto previous_mode  {getCurrentMode()};
        auto success {setMode(OperatingModes::CONFIG)};
        delayMilliSecs(25);

        // Note: ordering is MSb --> |Z|Y|X| <-- LSb
        uint8_t new_mapping {0U};
        new_mapping = new_mapping
                    | remapping.x
                    | remapping.y << 2U
                    | remapping.z << 4U;

        success = success && i2cWriteByteToRegister(i2cBusID, i2cAddress, BNO055_AXIS_MAP_CONFIG_ADDR, new_mapping);
        success = success && setMode(previous_mode);

        return success;
    }

    bool BNO055_IMU::RemapSigns(const BNO055_IMU::AxesSigns remapping) {
        auto previous_mode  {getCurrentMode()};
        auto success {setMode(OperatingModes::CONFIG)};
        delayMilliSecs(25);

        // Note: ordering is MSb --> |X|Y|Z| <-- LSb
        uint8_t new_mapping {0U};
        new_mapping = new_mapping
                    | remapping.z
                    | remapping.y << 1U
                    | remapping.x << 2U;

        success = success && i2cWriteByteToRegister(i2cBusID, i2cAddress, BNO055_AXIS_MAP_SIGN_ADDR, new_mapping);
        success = success && setMode(previous_mode);

        return success;
    }

    BNO055_IMU::AxesConfiguration BNO055_IMU::getAxesConfiguration() {                
        uint8_t sign = i2cReadByteFromRegister(i2cBusID, i2cAddress, BNO055_AXIS_MAP_SIGN_ADDR);
        uint8_t mapping = i2cReadByteFromRegister(i2cBusID, i2cAddress, BNO055_AXIS_MAP_CONFIG_ADDR);

        BNO055_IMU::AxesConfiguration result;
        result.x = {
            .sign      = static_cast<AxisSignSelection>((sign & 0b1'0'0U) >> 2U),
            .remapping = static_cast<AxisRemappingSelection>(mapping & 0b00'00'11U)
        };
        result.y = {
            .sign      = static_cast<AxisSignSelection>((sign & 0b0'1'0U) >> 1U),
            .remapping = static_cast<AxisRemappingSelection>((mapping & 0b00'11'00U) >> 2U)
        };
        result.z = {
            .sign      = static_cast<AxisSignSelection>(sign & 0b0'0'1U),
            .remapping = static_cast<AxisRemappingSelection>((mapping & 0b11'00'00U) >> 4U)
        };

        return result;
    }

    int16_t BNO055_IMU::value_at(uint8_t source) {
        uint16_t rxBuffer {0U};
        rxBuffer = i2cReadWordFromRegister(i2cBusID, i2cAddress, source);
        return static_cast<int16_t>(rxBuffer);
    }

    // --  Operating Modes Configuration  --------------------------------------
    
    // ----------------------
    // -- Get Current Mode --
    // ----------------------

    BNO055_IMU::OperatingModes BNO055_IMU::getCurrentMode() const {
        return this->mode;
    }

    // --------------------
    // -- Set Mode Power --
    // --------------------

    bool BNO055_IMU::setMode(PowerModes mode) const {
        // Select register map page zero:
        bool success = i2cWriteByteToRegister(i2cBusID, i2cAddress, BNO055_PAGE_ID_ADDR, 0U);
        delayMilliSecs(500);

        while (i2cReadByteFromRegister(i2cBusID, i2cAddress, BNO055_PAGE_ID_ADDR) != uint8_t(mode))
        {
            success = success &&  i2cWriteByteToRegister(i2cBusID, i2cAddress, BNO055_PWR_MODE_ADDR, mode);
        }

        return success;
    }

    // ------------------------
    // -- Set Mode Operating --
    // ------------------------


    bool BNO055_IMU::setMode(BNO055_IMU::OperatingModes operatingMode) {

        bool success = i2cWriteByteToRegister(i2cBusID, i2cAddress, BNO055_OPR_MODE_ADDR, operatingMode);
        delayMilliSecs(500);
        
        while (i2cReadByteFromRegister(i2cBusID, i2cAddress, BNO055_OPR_MODE_ADDR) != operatingMode)
        {
            success = success && i2cWriteByteToRegister(i2cBusID, i2cAddress, BNO055_OPR_MODE_ADDR, operatingMode);
        };

        this->mode = success ?
                     operatingMode :
                     this->mode;

        return success;
    }

    // --  Measurements Units Operations  --------------------------------------

    bool BNO055_IMU::setAccelerationUnits(BNO055_IMU::AccelerationUnits units) {
        return setUnitsRegister(units, AccelerationUnitsMask);
    }

    BNO055_IMU::AccelerationUnits BNO055_IMU::getAccelerationUnits() const {
        auto units = static_cast<AccelerationUnits>(getUnitsRegister(AccelerationUnitsMask));
        return units;
    }

    bool BNO055_IMU::setAngularRateUnits(BNO055_IMU::AngularRateUnits units) {
        return setUnitsRegister(units, Angular_Rate_Units_Mask);
    }

    BNO055_IMU::AngularRateUnits BNO055_IMU::getAngularRateUnits() const {
        auto units = static_cast<AngularRateUnits>(getUnitsRegister(Angular_Rate_Units_Mask));
        return units;
    }

    bool BNO055_IMU::setEulerAngleUnits(BNO055_IMU::EulerAngleUnits units) {
        return setUnitsRegister(units, Euler_Angle_Units_Mask);
    }

    BNO055_IMU::EulerAngleUnits BNO055_IMU::getEulerAngleUnits() const {
        auto units = static_cast<EulerAngleUnits>(getUnitsRegister(Euler_Angle_Units_Mask));
        return units;
    }

    bool BNO055_IMU::setTemperatureUnits(BNO055_IMU::TemperatureUnits units) {
        return setUnitsRegister(units, Temperature_Units_Mask);
    }

    BNO055_IMU::TemperatureUnits BNO055_IMU::getTemperatureUnits() const {
        auto units = static_cast<TemperatureUnits>(getUnitsRegister(Temperature_Units_Mask));
        return units;
    }

    bool BNO055_IMU::setPitchRotation(BNO055_IMU::PitchRotationConventions convention) {
        return setUnitsRegister(convention, Pitch_Rotation_Convention_Mask);
    }

    BNO055_IMU::PitchRotationConventions BNO055_IMU::getPitchRotationConvention() const {
        auto units = static_cast<PitchRotationConventions>(getUnitsRegister(Pitch_Rotation_Convention_Mask));
        return units;
    }

    // --  Measurement Reading Operations  -------------------------------------

    void BNO055_IMU::readSensorData(BNO055_IMU::MagField & data) {
        read3AxisMeasurement(BNO055_MAG_DATA_X_LSB_ADDR, 16.0F, data);
    }

    void BNO055_IMU::readSensorData(BNO055_IMU::Acceleration &data) {
        float lsb {
            getAccelerationUnits() == MILLIGRAVITY ? 1.0F : 100.0F
        };
        read3AxisMeasurement(BNO055_ACCEL_DATA_X_LSB_ADDR, lsb, data);
    }

    void BNO055_IMU::readSensorData(BNO055_IMU::AngularVelocity &data) {
        read3AxisMeasurement(BNO055_GYRO_DATA_X_LSB_ADDR, 900.0F, data);
    }

    void BNO055_IMU::readFusedData(Gravity &data) {
        read3AxisMeasurement(BNO055_GRAVITY_DATA_X_LSB_ADDR, 100.0F, data);
    }

    void BNO055_IMU::readFusedData(EulerOrientation &data) {
        float lsb {
            getEulerAngleUnits() == DEGREES ? 16.0F : 900.0F
        };
        read3AxisMeasurement(BNO055_EULER_H_LSB_ADDR, lsb, data);
    }

    void BNO055_IMU::readFusedData(LinearAcceleration &data) {
        read3AxisMeasurement(BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, 100.0F, data);
    }

    void BNO055_IMU::read3AxisMeasurement
        (uint8_t source, float lsb, SensorData &data)
    {
        data = {0.0F, 0.0F, 0.0F};
        uint8_t rxBuffer [6];

        //  By reading multiple UInt8s, the device ensures data consistency,
        //  whereas reading single UInt8s individually does not have that
        //  guarantee. See the Datasheet, section 3.7 "Data register shadowing"
        i2cReadDataFromRegister(i2cBusID, i2cAddress, source, rxBuffer, 6U);

        auto newX {static_cast<int16_t>(rxBuffer[0] | (rxBuffer[1] << 8U))};
        auto newY {static_cast<int16_t>(rxBuffer[2] | (rxBuffer[3] << 8U))};
        auto newZ {static_cast<int16_t>(rxBuffer[4] | (rxBuffer[5] << 8U))};

        data.X = static_cast<float>(newX) / lsb;
        data.Y = static_cast<float>(newY) / lsb;
        data.Z = static_cast<float>(newZ) / lsb;
    }

    void BNO055_IMU::readFusedData(Quaternion & data) const {
        uint8_t rxBuffer [8];
        data = {.0F, .0F, .0F, .0F};

        //  By reading multiple UInt8s, the device ensures data consistency,
        //  whereas reading single UInt8s individually does not have that
        //  guarantee. See the Datasheet, section 3.7 "Data register shadowing"
        i2cReadDataFromRegister(i2cBusID, i2cAddress, BNO055_QUATERNION_DATA_W_LSB_ADDR, rxBuffer, 8U);

        int16_t newW = static_cast<int16_t>(rxBuffer[0] | (rxBuffer[1] << 8U));
        int16_t newX = static_cast<int16_t>(rxBuffer[2] | (rxBuffer[3] << 8U));
        int16_t newY = static_cast<int16_t>(rxBuffer[4] | (rxBuffer[5] << 8U));
        int16_t newZ = static_cast<int16_t>(rxBuffer[6] | (rxBuffer[7] << 8U));

        const float SCALE = 1.0F / static_cast<float>(pow(2.0, 14.0));
        data.W = static_cast<float>(newW) * SCALE;
        data.X = static_cast<float>(newX) * SCALE;
        data.Y = static_cast<float>(newY) * SCALE;
        data.Z = static_cast<float>(newZ) * SCALE;
        
    }

    void BNO055_IMU::readAllFusedData(FusedData &data) {
        data.eulerOrientation = {};
        data.gravity = {};
        data.linearAcceleration = {};
        data.quat = {};

        constexpr size_t buffSize {6U + 8U + 6U + 6U};
        uint8_t rxBuffer [buffSize];

        i2cReadDataFromRegister(i2cBusID, i2cAddress, BNO055_EULER_H_LSB_ADDR, rxBuffer, buffSize);

        float lsb = getEulerAngleUnits() == DEGREES ? 16.0F : 900.0F;
        rawTo3AxisMeasurement(&rxBuffer[0U], lsb, data.eulerOrientation);

        lsb = static_cast<float>(pow(2.0, 14.0));
        rawTo4AxisMeasurement(&rxBuffer[6U], lsb, data.quat);

        rawTo3AxisMeasurement(&rxBuffer[14U], 100.0F, data.linearAcceleration);

        rawTo3AxisMeasurement(&rxBuffer[20U], 100.0F, data.gravity);
    }

    void BNO055_IMU::readTemperatures(BNO055_IMU::Temperatures &data) {
        data.from_gyroscope     = readTemperatureFrom(BNO055_IMU::GYROSCOPE);
        data.from_accelerometer = readTemperatureFrom(BNO055_IMU::ACCELEROMETER);
    }

    int8_t BNO055_IMU::readTemperatureFrom(BNO055_IMU::TemperatureSource source) {
        (void) i2cWriteByteToRegister(i2cBusID, i2cAddress, BNO055_TEMP_SOURCE_ADDR, source);
        uint8_t buff = i2cReadByteFromRegister(i2cBusID, i2cAddress, BNO055_TEMP_ADDR);

        int8_t result = static_cast<int8_t>(buff);
        if (getTemperatureUnits() == FAHRENHEIT) {
            result = result * 2; // LSB = 2 for Fahrinheit and 1 for Celsius
        }
        return result;
    }

    bool BNO055_IMU::setUnitsRegister(uint8_t newValue, uint8_t mask) {
        const OperatingModes previousMode = this->mode;

        bool inPreviousMode;
        if (previousMode != OperatingModes::CONFIG) {
            inPreviousMode = setMode(OperatingModes::CONFIG);
        } else {
            inPreviousMode = true;
        }

        bool success = false;
        if (inPreviousMode) {
            uint8_t value;
            value = i2cReadByteFromRegister(i2cBusID, i2cAddress, BNO055_UNIT_SEL_ADDR);

            value = value & (~mask);  // Clear bits
            value = value | newValue; // Set new bits

            i2cWriteByteToRegister(i2cBusID, i2cAddress, BNO055_UNIT_SEL_ADDR, value);
            delayMilliSecs(10);

            if (previousMode != OperatingModes::CONFIG) {
                success = setMode(previousMode);
            }
        }

        return success;
    }

    uint8_t BNO055_IMU::getUnitsRegister(uint8_t unitsMask) const {
        uint8_t unitsRegister;
        unitsRegister= i2cReadByteFromRegister(i2cBusID, i2cAddress, BNO055_UNIT_SEL_ADDR);

        return (static_cast<uint8_t>(unitsRegister & unitsMask));
    }
}