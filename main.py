import time
from multiprocessing.spawn import old_main_modules
from smbus2 import SMBus
version = '250120F'
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
    dig_P1 = calib[6] | (calib[7] << 8)
    dig_P2 = (calib[8] | (calib[9] << 8)) - 65536 if calib[9] & 0x80 else calib[8] | (calib[9] << 8)
    return {
        'P1': dig_P1, 'P2': dig_P2
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
# Read raw pressure data
def read_pressure_raw(bus):
    try:
        data = bus.read_i2c_block_data(BMP280_I2C_ADDRESS, REG_PRESSURE_MSB, 3)
        adc_p = (data[0] << 16) | (data[1] << 8) | data[2]  # Combine 3 bytes
        adc_p >>= 4  # Adjust for 20-bit resolution
        return adc_p
    except IOError as e:
        print(f"I2C Read Error: {e}")
        return None
# Main function
if __name__ == '__main__':
    print(f"version {version}")
    old_pressure = 999999999  # Initialize old pressure with a large value
    print(f'BMP280 Fast Pressure Readings Script version: {version}')
    bus = SMBus(1)  # Use I2C bus 1 (Raspberry Pi default)
    try:
        init_bmp280(bus)
        calib = read_calibration(bus)
        print("Calibration Coefficients:", calib)
        pressure_readings = []
        while True:
            new_pressure = read_pressure_raw(bus)
            pdiff = new_pressure - old_pressure
            if new_pressure is not None:
                    timestamp = time.strftime('%M:%S')
                    pressure_readings.append((timestamp, pdiff))
                    print(f"pdiff at {timestamp}: {pdiff}")
            else:
                print("Failed to read pressure data.")
            old_pressure = new_pressure
            time.sleep(1)  # Delay to match your sensor reading frequency
    except Exception as e:
        print(f"Error: {e}")
    finally:
        bus.close()

