#include "IMU.hpp"

#include "Common.hpp"
#include "esp_log.h"
#include <cstdint>

class IMU {
    private:
        // Overhead //
        const AngularRange angRange;      // Stores which range of operation its in
        const AccelRange acclRange;       // Stores which range of operation its in
        // Data //
        vec<uint16_t> raw_angvel;
        vec<uint16_t> raw_accel;           
        vec<float> angvel;       // Most recent angvel values
        vec<float> angvel_zero;
        vec<float> accel;        // Most recent accel values
        vec<float> accel_zero;

        // EFFECTS: Updates angvel and accel by converting
        // raw_angvel and raw_accel
        vec<float> binary_to_float() {
            angvel.x = raw_angvel.x     * ang_conversion()  - angvel_zero.x;
            angvel.y = raw_angvel.y     * ang_conversion()  - angvel_zero.y;
            angvel.z = raw_angvel.z     * ang_conversion()  - angvel_zero.z;
            accel.x  = raw_accel.x      * accl_conversion() - accel_zero.x;
            accel.y  = raw_accel.y      * accl_conversion() - accel_zero.y;
            accel.z  = raw_accel.z      * accl_conversion() - accel_zero.z;
        }

        uint8_t angRange_to_bin() const {
            switch (angRange) {
                case AngularRange::plus_minus_250dps: return (uint8_t) 0b00;
                case AngularRange::plus_minus_500dps: return (uint8_t) (0b01 << 3);
                case AngularRange::plus_minus_1000dps: return (uint8_t) (0b10 << 3);
                case AngularRange::plus_minus_2000dps: return (uint8_t) (0b11 << 3);
            }
        }

        uint8_t accelRange_to_bin() const {
            switch (acclRange) {
                case AccelRange::plus_minus_2g: return (uint8_t) 0b00;
                case AccelRange::plus_minus_4g: return (uint8_t) (0b01 << 3);
                case AccelRange::plus_minus_8g: return (uint8_t) (0b10 << 3);
                case AccelRange::plus_minus_16g: return (uint8_t) (0b11 << 3);
            }
        }

        float ang_conversion() const {
            switch(angRange) {
                case AngularRange::plus_minus_250dps: return 500.0f/65536.0f;
                case AngularRange::plus_minus_500dps: return 1000.0f/65536.0f;
                case AngularRange::plus_minus_1000dps: return 2000.0f/65536.0f;
                case AngularRange::plus_minus_2000dps: return 4000.0f/65536.0f;
            }
        }

        float accl_conversion() const {
            switch(acclRange) {
                case AccelRange::plus_minus_2g: return 4.0f/65536.0f;
                case AccelRange::plus_minus_4g: return 8.0f/65536.0f;
                case AccelRange::plus_minus_8g: return 16.0f/65536.0f;
                case AccelRange::plus_minus_16g: return 32.0f/65536.0f;
            }
        }

        void config_IMU() const {
            // clear the sleep bit to turn on the device
            write_to_reg(I2C_NUM_0, IMU_ADDR, PWR_MGMT_1, 0x00);
            // configure the gyroscope to +/- 500 dps
            write_to_reg(I2C_NUM_0, IMU_ADDR, GYRO_CONFIG, angRange_to_bin());
            // configure the accelerometer +/- 4g
            write_to_reg(I2C_NUM_0, IMU_ADDR, ACCEL_CONFIG, accelRange_to_bin());
        }

    public:
        
        // EFFECTS: Contsructs IMU object, turns on the IMU,
        // AND configures the IMU range
        // NOTE: it is critical that accel_zero and angvel_zero
        // are initialized to be 0 in order for calibrate() to function
        // properly due to the implementation of binary_to_float
        IMU(AngularRange angRng, AccelRange acclRng)
            : angRange(angRng), acclRange(acclRng),
              raw_angvel(0), raw_accel(0), angvel(0.0f), angvel_zero(0.0f),
              accel(0.0f), accel_zero(0.0f) {
            config_IMU();
        }

        // MODIFIES: angvel_zero and accel_zero
        // EFFECTS: When called, updates the IMU readings, then stores
        // the readings immediately after the update as the zero values
        // for accel and angvel
        void calibrate() {
            update();
            angvel_zero = angvel;
            accel_zero = accel;
        }

        // MODIFIES: raw and converted values
        void update() {
            i2c_cmd_handle_t ptr;
            uint8_t data[14];
            
            // configure the register pointer
            ptr = i2c_cmd_link_create();
            i2c_master_start(ptr);
            i2c_master_write_byte(ptr, IMU_ADDR << 1, true);
            i2c_master_write_byte(ptr, 0x3B, true);
            i2c_master_stop(ptr);
            i2c_master_cmd_begin(I2C_NUM_0, ptr, pdMS_TO_TICKS(50));
            i2c_cmd_link_delete(ptr);
            
            // read from the IMU
            ptr = i2c_cmd_link_create();
            i2c_master_start(ptr);
            i2c_master_write_byte(ptr, (IMU_ADDR << 1) + 1, true);
            for (uint8_t i = 0; i < 13; ++i) {
                if(i2c_master_read_byte(ptr, data + i, I2C_MASTER_ACK) != ESP_OK) {
                    ESP_LOGI("ERROR", "Unable to read from IMU");
                }
            }
            i2c_master_read_byte(ptr, data + 13, I2C_MASTER_NACK);
            i2c_master_stop(ptr);
            i2c_master_cmd_begin(I2C_NUM_0, ptr, pdMS_TO_TICKS(50));
            i2c_cmd_link_delete(ptr);

            // Parse the data from the IMU
            raw_accel.x   = ((uint16_t)data[0] << 8U) + (uint16_t)data[1];
            raw_accel.y   = ((uint16_t)data[2] << 8U) + (uint16_t)data[3];
            raw_accel.z   = ((uint16_t)data[4] << 8U) + (uint16_t)data[5];
            raw_angvel.x  = ((uint16_t)data[8] << 8U) + (uint16_t)data[9];
            raw_angvel.y  = ((uint16_t)data[10] << 8U) + (uint16_t)data[11];
            raw_angvel.z  = ((uint16_t)data[12] << 8U) + (uint16_t)data[13];

            // Decipher the data
            binary_to_float();
        }

        vec<float> get_angvel() const {
            return angvel;
        }

        vec<float> get_accel() const {
            return accel;
        }

        // NOTE: no setters because only IMU sets values //
};