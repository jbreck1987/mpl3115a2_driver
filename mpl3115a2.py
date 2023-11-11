"""
Device driver for the MPL3115A2 Barometer/Altimeter/Temp. Sensor.

Author: Josh Breckenridge
Date: 11/11/2023
"""
from micropython import const
from machine import I2C, Pin

# Register Definitions

DR_STATUS = const(0x00)
# In FIFO mode, 0x00 refers to the F_STATUS register, which gives data about the
# FIFO subsystem.
F_STATUS = const(0x00)
OUT_P_MSB = const(0x01)
# In FIFO mode, 0x01 refers to F_DATA which is the only way to extract the Pressure/Sensor data,
# registers 0x02 - 0x05 are disabled. Actual register is at 0x0E,
# this is a useful alias to help reduce number of reads.
F_DATA = const(0x01)
OUT_P_CSB = const(0x02)
OUT_P_LSB = const(0x03)
OUT_T_MSB = const(0x04)
OUT_T_LSB = const(0x05)
DR_STATUS = const(0x06)
OUT_P_DELTA_MSB = const(0x07)
OUT_P_DELTA_CSB = const(0x08)
OUT_P_DELTA_LSB = const(0x09)
OUT_T_DELTA_MSB = const(0x0A)
OUT_T_DELTA_LSB = const(0x0B)
WHO_AM_I = const(0x0C)
F_STATUS = const(0x0D)
F_SETUP = const(0x0F)
TIME_DLY = const(0x10)
SYSMOD = const(0x11)
INT_SOURCE = const(0x12)
PT_DATA_CFG = const(0x13)
BAR_IN_MSB = const(0x14)
BAR_IN_LSB = const(0x15)
P_TGT_MSB = const(0x16)
P_TGT_LSB = const(0x17)
T_TGT = const(0x18)
P_WND_MSB = const(0x19)
P_WND_LSB = const(0x1A)
T_WND = const(0x1B)
P_MIN_MSB = const(0x1C)
P_MIN_CSB = const(0x1D)
P_MIN_LSB = const(0x1E)
T_MIN_MSB = const(0x1F)
T_MIN_LSB = const(0x20)
P_MAX_MSB = const(0x21)
P_MAX_CSB = const(0x22)
P_MAX_LSB = const(0x23)
T_MAX_MSB = const(0x24)
T_MAX_LSB = const(0x25)
CTRL_REG1 = const(0x26)
CTRL_REG2 = const(0x27)
CTRL_REG3 = const(0x28)
CTRL_REG4 = const(0x29)
CTRL_REG5 = const(0x2A)
OFF_P = const(0x2B)
OFF_T = const(0x2C)
OFF_H = const(0x2D)

# Device Constants
DEVICE_ID = const(0xC4)
SLAVE_ID = const(0xC0)

class Buffer():
    """
    Simple buffer using a bytearray. If the buffer is full,
    any new data needing to be added to the buffer is thrown away.
    """
    def __init__(self, size: int) -> None:
        self.data = bytearray(size)
        self.temp = bytearray(size)
        self.write_idx = 0


    def read(self, num_bytes: int):
        # Clear out the temp buffer
        for i in range(0, len(self.temp)):
            self.temp[i] = 0
        
        # Move data to temp array and erase the data for the next
        # use.
        for i in range(0, num_bytes):
            self.temp[i] = self.data[i]
            self.data[i] = 0
        
        return self.temp



class MPL3115A2():
    """
    Object representing a single MPL3115A2 device. The device can run as a Barometer or
    Altimeter. Within these modes, the device presents data using non-FIFO mode
    or FIFO mode, either using polling or interrupts. The user will need to determine which
    configuration the device will use, see Pg. 13 in the datasheet. By default, the device
    will be in pressure sensor, non-FIFO, polling mode.
    """
    def __init__(self, sda: Pin, scl: Pin, i2c_clk: int) -> None:
        self.i2c = I2C(id=DEVICE_ID, scl=scl,  sda=sda, freq=i2c_clk)

        # Create a miscellaneous and data buffer for storing return values.
        # The miscellaneous buffer is circular with 16 bits being more
        # than enough for all control logic.
        self._tx_buff = bytearray(12)
        # There are 5 registers that need to be read
        # concurrently to get a single pressure and temp reading.
        self._rx_buff = bytearray(12)

        # Make sure device is reachable and has the correct
        # Device ID
        self._read_reg(WHO_AM_I, 1)
        if self._rx_buff[0] != DEVICE_ID:
            raise MemoryError(f'Device ID is not ')
        


    def _read_reg(self, reg_addr: int, num_bytes: int) -> None:
        """
        Reads data from register(s) into the rx buffer.
        """
        # Reset the rx buffer to zero
        for i in range(0, len(self._rx_buff)):
            self._rx_buff[i] = 0
        
        # Read the register(s) and store in rx_buff
        for i in range(0, num_bytes):
            if i == num_bytes:
                # Release the I2C bus
                self._rx_buff[i] = self.i2c.readfrom(reg_addr + i, 1)
            else:
                # Don't release the I2C bus
                self._rx_buff[i] = self.i2c.readfrom(reg_addr + i, 1, stop=False)
    
    def _write_reg(self, reg_addr: int, data: bytearray) -> None:
        """
        Writes data to register(s).
        """
        self.i2c.writeto(reg_addr, data)


        
        