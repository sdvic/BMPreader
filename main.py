import time
from smbus2 import SMBus

# Constants for BMP280
BMP280_I2C_ADDRESS = 0x76  # Default I2C address of the BMP280
REG_ID = 0xD0  # Register for chip ID
REG_CTRL_MEAS = 0xF4  # Control measurement register
REG_CONFIG = 0xF5  # Configuration register
REG_PRESSURE_MSB = 0xF7  # Pressure MSB register
REG_CALIB_START = 0x88  # Start of calibration coefficients
REG_CALIB_END = 0xA1  # End of calibration coefficients

# Read calibration coefficients
def read_calibration(bus):
    calib = bus.read_i2c_block_data(BMP280_I2C_ADDRESS, REG_CALIB_START, REG_CALIB_END - REG_CALIB_START + 1)
    dig_T1 = calib[0] | (calib[1] << 8)
    dig_T2 = (calib[2] | (calib[3] << 8)) - 65536 if calib[3] & 0x80 else calib[2] | (calib[3] << 8)
    dig_T3 = (calib[4] | (calib[5] << 8)) - 65536 if calib[5] & 0x80 else calib[4] | (calib[5] << 8)
    dig_P1 = calib[6] | (calib[7] << 8)
    dig_P2 = (calib[8] | (calib[9] << 8)) - 65536 if calib[9] & 0x80 else calib[8] | (calib[9] << 8)
    dig_P3 = (calib[10] | (calib[11] << 8)) - 65536 if calib[11] & 0x80 else calib[10] | (calib[11] << 8)
    dig_P4 = (calib[12] | (calib[13] << 8)) - 65536 if calib[13] & 0x80 else calib[12] | (calib[13] << 8)
    dig_P5 = (calib[14] | (calib[15] << 8)) - 65536 if calib[15] & 0x80 else calib[14] | (calib[15] << 8)
    dig_P6 = (calib[16] | (calib[17] << 8)) - 65536 if calib[17] & 0x80 else calib[16] | (calib[17] << 8)
    dig_P7 = (calib[18] | (calib[19] << 8)) - 65536 if calib[19] & 0x80 else calib[18] | (calib[19] << 8)
    dig_P8 = (calib[20] | (calib[21] << 8)) - 65536 if calib[21] & 0x80 else calib[20] | (calib[21] << 8)
    dig_P9 = (calib[22] | (calib[23] << 8)) - 65536 if calib[23] & 0x80 else calib[22] | (calib[23] << 8)
    return {
        'T1': dig_T1, 'T2': dig_T2, 'T3': dig_T3,
        'P1': dig_P1, 'P2': dig_P2, 'P3': dig_P3, 'P4': dig_P4,
        'P5': dig_P5, 'P6': dig_P6, 'P7': dig_P7, 'P8': dig_P8, 'P9': dig_P9
    }

# Initialize the sensor
def init_bmp280(bus):
    chip_id = bus.read_byte_data(BMP280_I2C_ADDRESS, REG_ID)
    print(f"Detected chip ID: {chip_id:#x}")
    if chip_id != 0x58:
        raise Exception("Device is not a BMP280!")

    # Set the sensor to normal mode with low oversampling and basic filtering
    bus.write_byte_data(BMP280_I2C_ADDRESS, REG_CTRL_MEAS, 0x27)  # Temp x1, pressure x1, normal mode
    bus.write_byte_data(BMP280_I2C_ADDRESS, REG_CONFIG, 0x04)  # Filter coefficient 2, standby 0.5ms
    print("BMP280 initialized with basic configuration.")

# Set sensor to forced mode for a single measurement
def set_forced_mode(bus):
    bus.write_byte_data(BMP280_I2C_ADDRESS, REG_CTRL_MEAS, 0x25)  # Forced mode

# Read raw pressure data
def read_pressure_raw(bus):
    set_forced_mode(bus)  # Trigger a new measurement
    time.sleep(0.5)  # Wait for measurement to complete
    try:
        data = bus.read_i2c_block_data(BMP280_I2C_ADDRESS, REG_PRESSURE_MSB, 3)
        adc_p = (data[0] << 16) | (data[1] << 8) | data[2]  # Combine 3 bytes
        adc_p >>= 4  # Adjust for 20-bit resolution
        return adc_p
    except IOError as e:
        print(f"I2C Read Error: {e}")
        return None

# Compute true pressure using calibration coefficients
def compensate_pressure(adc_p, calib):
    var1 = ((adc_p >> 3) - (calib['P1'] << 1)) * calib['P2'] >> 11
    var2 = (((adc_p >> 4) - calib['P1']) * ((adc_p >> 4) - calib['P1']) >> 12) * calib['P3'] >> 14
    var1 += (var2 + calib['P4'])
    if var1 == 0:
        return 0  # Avoid division by zero
    true_pressure = (((1048576 - adc_p) - (var2 >> 12)) * 3125) << 1
    return true_pressure >> 8

# Main function
if __name__ == '__main__':
    print('BMP280 Debugging Script with Calibration version 250117H')
    bus = SMBus(1)  # Use I2C bus 1 (Raspberry Pi default)
    try:
        init_bmp280(bus)
        calib = read_calibration(bus)
        print("Calibration Coefficients:", calib)
        old_pressure = 999999 # old_pressure is set to a large value for reading on the first loop
        while True:
            raw_pressure = read_pressure_raw(bus)
            if raw_pressure is not None:
                true_pressure = compensate_pressure(raw_pressure, calib)
                lsb_pressure = true_pressure & 0xFFFF  # Extract the least significant 16 bits
                new_pressure = raw_pressure
                pdiff = new_pressure - old_pressure
                print(f"{old_pressure}  {pdiff}")
                old_pressure = new_pressure
            else:
                print("Failed to read pressure data.")
            time.sleep(1)  # Delay for 1 second between measurements
            new_pressure = lsb_pressure
    except Exception as e:
        print(f"Error: {e}")
    finally:
        bus.close()
