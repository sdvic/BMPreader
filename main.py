import time
from datetime import datetime, timedelta
from smbus2 import SMBus

version = '250123J'

# Constants for BMP280
BMP280_I2C_ADDRESS = 0x76  # Default I2C address of the BMP280
REG_ID = 0xD0  # Register for chip ID
REG_CTRL_MEAS = 0xF4  # Control measurement register
REG_CONFIG = 0xF5  # Configuration register
REG_PRESSURE_MSB = 0xF7  # Pressure MSB register
REG_TEMP_MSB = 0xFA  # Temperature MSB register
REG_CALIB_START = 0x88  # Start of calibration coefficients
REG_CALIB_END = 0xA1  # End of calibration coefficients

# Read calibration coefficients
def read_calibration(bus):
    calib = bus.read_i2c_block_data(BMP280_I2C_ADDRESS, REG_CALIB_START, REG_CALIB_END - REG_CALIB_START + 1)
    dig_T1 = calib[0] | (calib[1] << 8)
    dig_T2 = calib[2] | (calib[3] << 8)
    dig_T2 = dig_T2 - 65536 if dig_T2 & 0x8000 else dig_T2
    dig_T3 = calib[4] | (calib[5] << 8)
    dig_T3 = dig_T3 - 65536 if dig_T3 & 0x8000 else dig_T3

    dig_P1 = calib[6] | (calib[7] << 8)
    dig_P2 = calib[8] | (calib[9] << 8)
    dig_P2 = dig_P2 - 65536 if dig_P2 & 0x8000 else dig_P2
    dig_P3 = calib[10] | (calib[11] << 8)
    dig_P3 = dig_P3 - 65536 if dig_P3 & 0x8000 else dig_P3
    dig_P4 = calib[12] | (calib[13] << 8)
    dig_P4 = dig_P4 - 65536 if dig_P4 & 0x8000 else dig_P4
    dig_P5 = calib[14] | (calib[15] << 8)
    dig_P5 = dig_P5 - 65536 if dig_P5 & 0x8000 else dig_P5
    dig_P6 = calib[16] | (calib[17] << 8)
    dig_P6 = dig_P6 - 65536 if dig_P6 & 0x8000 else dig_P6
    dig_P7 = calib[18] | (calib[19] << 8)
    dig_P7 = dig_P7 - 65536 if dig_P7 & 0x8000 else dig_P7
    dig_P8 = calib[20] | (calib[21] << 8)
    dig_P8 = dig_P8 - 65536 if dig_P8 & 0x8000 else dig_P8
    dig_P9 = calib[22] | (calib[23] << 8)
    dig_P9 = dig_P9 - 65536 if dig_P9 & 0x8000 else dig_P9

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

# Read raw temperature and pressure data
def read_raw_data(bus):
    try:
        data = bus.read_i2c_block_data(BMP280_I2C_ADDRESS, REG_PRESSURE_MSB, 6)
        adc_p = (data[0] << 16) | (data[1] << 8) | data[2]
        adc_p >>= 4  # Adjust for 20-bit resolution

        adc_t = (data[3] << 16) | (data[4] << 8) | data[5]
        adc_t >>= 4  # Adjust for 20-bit resolution

        return adc_t, adc_p
    except IOError as e:
        print(f"I2C Read Error: {e}")
        return None, None

# Compensate temperature and pressure readings
def compensate_readings(adc_t, adc_p, calib):
    if adc_t is None or adc_p is None:
        return None, None

    # Temperature compensation
    var1 = (adc_t / 16384.0 - calib['T1'] / 1024.0) * calib['T2']
    var2 = ((adc_t / 131072.0 - calib['T1'] / 8192.0) ** 2) * calib['T3']
    t_fine = var1 + var2
    temperature = t_fine / 5120.0

    # Pressure compensation
    var1 = t_fine / 2.0 - 64000.0
    var2 = var1 * var1 * calib['P6'] / 32768.0
    var2 = var2 + var1 * calib['P5'] * 2.0
    var2 = var2 / 4.0 + calib['P4'] * 65536.0
    var1 = (calib['P3'] * var1 * var1 / 524288.0 + calib['P2'] * var1) / 524288.0
    var1 = (1.0 + var1 / 32768.0) * calib['P1']
    if var1 == 0:
        pressure = 0  # Avoid division by zero
    else:
        pressure = 1048576.0 - adc_p
        pressure = (pressure - var2 / 4096.0) * 6250.0 / var1
        var1 = calib['P9'] * pressure * pressure / 2147483648.0
        var2 = pressure * calib['P8'] / 32768.0
        pressure = pressure + (var1 + var2 + calib['P7']) / 16.0

    return temperature, pressure / 100.0  # Pressure in hPa

# Main function
if __name__ == '__main__':
    print(f"version {version}")
    bus = SMBus(1)  # Use I2C bus 1 (Raspberry Pi default)
    old_pressure = 9999  # Initialize old pressure
    old_time = datetime.now()  # Initialize old_time as datetime
    try:
        init_bmp280(bus)
        calib = read_calibration(bus)
        print("Calibration Coefficients:", calib)

        while True:
            adc_t, adc_p = read_raw_data(bus)
            temperature, new_pressure = compensate_readings(adc_t, adc_p, calib)
            new_time = datetime.now()
            tdiff = (new_time - old_time).total_seconds()  # Get time difference in seconds
            pdiff = new_pressure - old_pressure
            if abs(pdiff) > .05:
                print(f" {new_pressure} pdiff: {pdiff:.6f} hPa, tdiff: {tdiff:.6f} seconds")
            old_pressure = new_pressure
            old_time = new_time  # Update old_time
            time.sleep(1)  # Delay between readings
    except Exception as e:
        print(f"Error: {e}")
    finally:
        bus.close()