"""
Device driver for the MPL3115A2 Barometer/Altimeter/Temp. Sensor.

Author: Josh Breckenridge
Date: 11/11/2023
"""
from micropython import const

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
DEVICE_ID = const(0x0C)
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