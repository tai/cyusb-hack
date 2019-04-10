#!/usr/bin/env python3
# -*- coding: utf-8-unix -*-
"""
A port of CyUSB Serial Library in pure python.
"""

import usb1
from usb1 import USBContext
from usb1.libusb1 import *
from enum import Enum, IntEnum
from contextlib import contextmanager
from IPython import embed

VID = 0x04b4
PID = 0x0004

EP_BULK = 2
EP_INTR = 3

EP_OUT = 0x00
EP_IN = 0x80

CY_VENDOR_REQUEST_DEVICE_TO_HOST = 0xC0
CY_VENDOR_REQUEST_HOST_TO_DEVICE = 0x40
CY_VENDOR_REQUEST = 0x40

CY_CLASS_INTERFACE_REQUEST_DEVICE_TO_HOST = 0xA1
CY_CLASS_INTERFACE_REQUEST_HOST_TO_DEVICE = 0x21

class CY_TYPE(IntEnum):
    DISABLED = 0
    UART     = 1
    SPI      = 2
    I2C      = 3
    JTAG     = 4
    MFG      = 5

class CY_CLASS(IntEnum):
    DISABLED = 0x00 # None or the interface is disabled
    CDC      = 0x02 # CDC ACM class
    PHDC     = 0x0F # PHDC class
    VENDOR   = 0xFF #

class CY_VENDOR_CMDS(IntEnum):
    CY_GET_VERSION_CMD     = 0xB0
    CY_GET_SIGNATURE_CMD   = 0xBD

    CY_UART_GET_CONFIG_CMD = 0xC0
    CY_UART_SET_CONFIG_CMD = 0xC1
    CY_SPI_GET_CONFIG_CMD  = 0xC2
    CY_SPI_SET_CONFIG_CMD  = 0xC3
    CY_I2C_GET_CONFIG_CMD  = 0xC4
    CY_I2C_SET_CONFIG_CMD  = 0xC5

    CY_I2C_WRITE_CMD       = 0xC6
    CY_I2C_READ_CMD        = 0xC7
    CY_I2C_GET_STATUS_CMD  = 0xC8
    CY_I2C_RESET_CMD       = 0xC9

    CY_SPI_READ_WRITE_CMD  = 0xCA
    CY_SPI_RESET_CMD       = 0xCB
    CY_SPI_GET_STATUS_CMD  = 0xCC

    CY_JTAG_ENABLE_CMD     = 0xD0
    CY_JTAG_DISABLE_CMD    = 0xD1
    CY_JTAG_READ_CMD       = 0xD2
    CY_JTAG_WRITE_CMD      = 0xD3

    CY_GPIO_GET_CONFIG_CMD = 0xD8
    CY_GPIO_SET_CONFIG_CMD = 0xD9
    CY_GPIO_GET_VALUE_CMD  = 0xDA
    CY_GPIO_SET_VALUE_CMD  = 0xDB

    CY_PROG_USER_FLASH_CMD = 0xE0
    CY_READ_USER_FLASH_CMD = 0xE1

    CY_DEVICE_RESET_CMD    = 0xE3

CY_SCB_INDEX_POS = 15

# I2C related macros
class CY_I2C(IntEnum):
    CONFIG_LENGTH = 16
    WRITE_COMMAND_POS = 3
    WRITE_COMMAND_LEN_POS = 4
    GET_STATUS_LEN = 3
    MODE_WRITE = 1
    MODE_READ = 0
    ERROR_BIT = (1)
    ARBITRATION_ERROR_BIT = (1 << 1)
    NAK_ERROR_BIT = (1 << 2)
    BUS_ERROR_BIT = (1 << 3)
    STOP_BIT_ERROR = (1 << 4)
    BUS_BUSY_ERROR = (1 << 5)
    ENABLE_PRECISE_TIMING = 1
    EVENT_NOTIFICATION_LEN = 3

# SPI related Macros
class CY_SPI(IntEnum):
    CONFIG_LEN = 16
    EVENT_NOTIFICATION_LEN = 2
    READ_BIT = (1)
    WRITE_BIT = (1 << 1)
    SCB_INDEX_BIT = (1 << 15)
    GET_STATUS_LEN = 4
    UNDERFLOW_ERROR = (1)
    BUS_ERROR = (1 << 1)

# Vendor UART related macros
CY_SET_LINE_CONTROL_STATE_CMD = 0x22
class CY_UART(IntEnum):
    SET_FLOW_CONTROL_CMD = 0x60
    SEND_BREAK_CMD = 0x23
    CONFIG_LEN = 16
    EVENT_NOTIFICATION_LEN = 10

    SERIAL_STATE_CARRIER_DETECT = 1
    SERIAL_STATE_TRANSMISSION_CARRIER = (1 << 1)
    SERIAL_STATE_BREAK_DETECTION = (1<< 2)
    SERIAL_STATE_RING_SIGNAL_DETECTION = (1 << 3)
    SERIAL_STATE_FRAMING_ERROR = (1 << 4)
    SERIAL_STATE_PARITY_ERROR = (1 << 5)
    SERIAL_STATUE_OVERRUN = (1 << 6)

# Bootloader related macros
CY_BOOT_CONFIG_SIZE = 64
CY_DEVICE_CONFIG_SIZE = 512
CY_FIRMWARE_BREAKUP_SIZE = 4096
CY_GET_SILICON_ID_LEN = 4
CY_GET_FIRMWARE_VERSION_LEN = 8
CY_GET_SIGNATURE_LEN = 4

# PHDC related macros
class CY_PHDC(IntEnum):
    SET_FEATURE = 0x03
    CLR_FEATURE = 0x01
    GET_DATA_STATUS = 0x00

# JTAG related Macros
CY_JTAG_OUT_EP = 0x04
CY_JTAG_IN_EP = 0x85

# GPIO related Macros
CY_GPIO_GET_LEN = 2
CY_GPIO_SET_LEN = 1

# PHDC related macros
CY_PHDC_GET_STATUS_LEN = 2
CY_PHDC_CLR_FEATURE_WVALUE = 0x1
CY_PHDC_SET_FEATURE_WVALUE = 0x0101

def find_device(context, vid=None, pid=None):
    for dev in context.getDeviceList(skip_on_error=True):
        if vid and dev.getVendorID()  != vid: continue
        if pid and dev.getProductID() != pid: continue
        yield dev

def get_type(us):
    if us.getClass() == CY_CLASS.VENDOR:
        return CY_TYPE(us.getSubClass())
    return CY_TYPE.DISABLED

def find_node(ux, func):
    """Helper to scan through class structure"""
    if func(ux): yield ux
    try:
        for ux_child in ux:
            yield from find_node(ux_child, func)
    except TypeError:
        pass

def find_setting(ud, cy_type=None):
    def check(ux):
        if not isinstance(ux, usb1.USBInterfaceSetting):
            return False
        if get_type(ux) == cy_type or cy_type == None:
            return True
        return False
    yield from find_node(ud, check)

def list_ep(us):
    for ue in us:
        if ue.getAttributes() == EP_BULK:
            addr = ue.getAddress()
            edir = "IN" if addr & EP_IN else "OUT"
            print("EP %d: %s" % (addr, edir))
        if ue.getAttributes() == EP_INTR:
            addr = ue.getAddress()
            print("EP %d: INTR" % (addr))

def get_config(ud, cy_type):
    """Return USB interface/configuration in (index, obj) form"""
    for uc in ud:
        for ui_index, ui in enumerate(uc):
            for us in ui:
                if get_type(us) == cy_type:
                    return (ui_index, uc, ui, us)
    return None

class CyUSB(object):
    @staticmethod
    def open(ud, cy_type, timeout=1000):
        found = get_config(ud, cy_type)
        if not found:
            raise Exception("Device with given type not found")

        # create instance
        self = CyUSB()

        # setup parameters
        self.if_num, uc, ui, us = found
        self.uc_num = uc.getConfigurationValue()
        self.us_num = us.getNumber()
        self.timeout = timeout

        # scan EPs
        for ep in us:
            ep_attr = ep.getAttributes()
            ep_addr = ep.getAddress()
            if ep_attr == EP_BULK:
                if ep_addr & EP_IN:
                    self.ep_in = ep_addr
                else:
                    self.ep_out = ep_addr
            elif ep_attr == EP_INTR:
                self.ep_intr = ep_addr

        # open USBDeviceHandle
        self.dev = ud.open()

        # detach kernel driver to gain access (need root access)
        #if dev.kernelDriverActive(if_num): dev.detachKernelDriver(if_num)
        self.dev.setAutoDetachKernelDriver(True)

        #
        # NOTE:
        # Windows and others seems to differ in expected order of
        # when to claim interface and when to set configuration.
        #
        self.dev.setConfiguration(self.uc_num)
        if self.us_num > 0:
            self.dev.setInterfaceAltSetting(self.if_num, self.us_num)
        return self

    def close(self):
        if self.dev:
            self.dev.close()
        self.dev = None

    def __enter__(self):
        self.dev.claimInterface(self.if_num)
        return self

    def __exit__(self, err_type, err_value, tb):
        self.dev.releaseInterface(self.if_num)
        self.close()

    ######################################################################

    def CyGetSpiConfig(self):
        dev = self.dev

        scbIndex = 1 if self.if_num > 0 else 0
        bmRequestType = CY_VENDOR_REQUEST
        bmRequest = CY_VENDOR_CMDS.CY_SPI_GET_CONFIG_CMD
        wValue = (scbIndex << CY_SCB_INDEX_POS)
        wIndex = 0
        wLength = CY_SPI.CONFIG_LEN

        ret = dev.controlRead(bmRequestType, bmRequest,
                              wValue, wIndex, wLength, self.timeout)
        return ret

    def CySetSpiConfig(self, config):
        dev = self.dev

        scbIndex = 1 if self.if_num > 0 else 0
        bmRequestType = CY_VENDOR_REQUEST
        bmRequest = CY_VENDOR_CMDS.CY_SPI_SET_CONFIG_CMD
        wValue = (scbIndex << CY_SCB_INDEX_POS)
        wIndex = 0
        wLength = CY_SPI.CONFIG_LEN
        wBuffer = bytearray()

        ret = dev.controlWrite(bmRequestType, bmRequest,
                               wValue, wIndex, wBuffer, self.timeout)
        return ret

    def CySpiReset(self):
        dev = self.dev

        scbIndex = 1 if self.if_num > 0 else 0
        bmRequestType = CY_VENDOR_REQUEST
        bmRequest = CY_VENDOR_CMDS.CY_SPI_RESET_CMD
        wValue = (scbIndex << CY_SCB_INDEX_POS)
        wIndex = 0
        wLength = 0

        ret = dev.controlRead(bmRequestType, bmRequest,
                              wValue, wIndex, wLength, self.timeout)
        return ret
        
    def CySpiRead(self, size):
        return self.dev.bulkRead(self.ep_in, size, timeout=self.timeout)

    def CySpiWrite(self, buff):
        return self.dev.bulkWrite(self.ep_out, buff, timeout=self.timeout)

    def CyGetSpiStatus(self):
        scbIndex = 1 if self.if_num > 0 else 0
        bmRequestType = CY_VENDOR_REQUEST | EP_OUT
        bmRequest = CY_VENDOR_CMDS.CY_SPI_GET_STATUS_CMD
        wValue = ((scbIndex << CY_SCB_INDEX_POS))
        wIndex = 0
        wLength = CY_SPI.GET_STATUS_LEN

        ret = self.dev.controlRead(bmRequestType, bmRequest,
                                   wValue, wIndex, wLength, self.timeout)
        return ret

    def CySpiReadWrite(self, wrbuff, rdsize):
        spiTransferMode = 0
        wIndex = 0
        if len(wrbuff) > 0:
            spiTransferMode |= CY_SPI.WRITE_BIT
            wIndex = len(wrbuff)
        if rdsize > 0:
            spiTransferMode |= CY_SPI.READ_BIT
            wIndex = rdsize
        
        scbIndex = 1 if self.if_num > 0 else 0
        bmRequestType = CY_VENDOR_REQUEST | EP_OUT
        bmRequest = CY_VENDOR_CMDS.CY_SPI_READ_WRITE_CMD
        wValue = (scbIndex << CY_SCB_INDEX_POS) | spiTransferMode
        wBuffer = bytearray(0)

        ret = self.dev.controlWrite(bmRequestType, bmRequest,
                                    wValue, wIndex, wBuffer, self.timeout)

        if not spiTransferMode & CY_SPI.READ_BIT:
            return self.CySpiWrite(wrbuff)

        if not spiTransferMode & CY_SPI.WRITE_BIT:
            return self.CySpiRead(rdsize)

        # FIXME: Not sure what Cypress is doing in read-write case

        return ret

    def CyGetI2cConfig(self):
        scbIndex = 1 if self.if_num > 0 else 0
        bmRequestType = CY_VENDOR_REQUEST | EP_OUT
        bmRequest = CY_VENDOR_CMDS.CY_I2C_GET_CONFIG_CMD
        wValue = ((scbIndex << CY_SCB_INDEX_POS))
        wIndex = 0
        wLength = CY_I2C.CONFIG_LENGTH

        ret = self.dev.controlRead(bmRequestType, bmRequest,
                                   wValue, wIndex, wLength, self.timeout)
        return ret

    def CySetI2cConfig(self, config):
        scbIndex = 1 if self.if_num > 0 else 0
        bmRequestType = CY_VENDOR_REQUEST | EP_IN
        bmRequest = CY_VENDOR_CMDS.CY_I2C_SET_CONFIG_CMD
        wValue = ((scbIndex << CY_SCB_INDEX_POS))
        wIndex = 0
        wLength = CY_I2C.CONFIG_LENGTH

        ret = self.dev.controlRead(bmRequestType, bmRequest,
                                   wValue, wIndex, wLength, self.timeout)
        return ret

    def CyI2cRead(self, config, size):
        scbIndex = 1 if self.if_num > 0 else 0
        bmRequestType = CY_VENDOR_REQUEST | EP_OUT
        bmRequest = CY_VENDOR_CMDS.CY_I2C_READ_CMD
        wValue = ((scbIndex << 7) | (0x7F & config.slaveAddress)) << 8
        wValue |= config.isStopBit | (config.isNakBtit << 1)
        wIndex = size
        wBuffer = bytearray(0)

        ret = self.dev.controlWrite(bmRequestType, bmRequest,
                                    wValue, wIndex, wBuffer, self.timeout)
        
        ret = self.dev.bulkRead(self.ep_in, size, timeout=self.timeout)

        return ret

    def CyI2cWrite(self, buff):
        scbIndex = 1 if self.if_num > 0 else 0
        bmRequestType = CY_VENDOR_REQUEST | EP_OUT
        bmRequest = CY_VENDOR_CMDS.CY_I2C_WRITE_CMD
        wValue = ((scbIndex << 7) | (0x7F & config.slaveAddress)) << 8
        wValue |= config.isStopBit
        wIndex = len(buff)
        wBuffer = bytearray(0)

        ret = self.dev.controlWrite(bmRequestType, bmRequest,
                                    wValue, wIndex, wBuffer, self.timeout)

        ret = dev.bulkWrite(self.ep_out, buff, timeout=self.timeout)
        return ret

    def CyI2cGetStatus(self, mode=0):
        dev = self.dev

        scbIndex = 1 if self.if_num > 0 else 0
        bmRequestType = CY_VENDOR_REQUEST | EP_IN
        bmRequest = CY_VENDOR_CMDS.CY_I2C_GET_STATUS_CMD
        wValue = (scbIndex << CY_SCB_INDEX_POS) | mode
        wIndex = 0
        wLength = CY_I2C.GET_STATUS_LEN

        ret = dev.controlRead(bmRequestType, bmRequest,
                              wValue, wIndex, wLength, self.timeout)
        return ret

    def CyI2cReset(self, mode=0):
        dev = self.dev

        scbIndex = 1 if self.if_num > 0 else 0
        bmRequestType = CY_VENDOR_REQUEST | EP_OUT
        bmRequest = CY_VENDOR_CMDS.CY_I2C_RESET_CMD
        wValue = (scbIndex << CY_SCB_INDEX_POS) | mode
        wIndex = 0
        wBuffer = bytearray(0)

        ret = dev.controlWrite(bmRequestType, bmRequest,
                               wValue, wIndex, wBuffer, self.timeout)
        return ret

    def CyGetUartConfig(self):
        scbIndex = 1 if self.if_num > 0 else 0
        bmRequestType = CY_VENDOR_REQUEST | EP_IN
        bmRequest = CY_VENDOR_CMDS.CY_UART_GET_CONFIG_CMD
        wValue = (scbIndex << CY_SCB_INDEX_POS)
        wIndex = 0
        wLength = CY_UART.CONFIG_LEN

        ret = self.dev.controlRead(bmRequestType, bmRequest,
                                   wValue, wIndex, wLength, self.timeout)
        return ret

    def CySetUartConfig(self):
        dev = self.dev

        scbIndex = 1 if self.if_num > 0 else 0
        bmRequestType = CY_VENDOR_REQUEST | EP_OUT
        bmRequest = CY_VENDOR_CMDS.CY_UART_SET_CONFIG_CMD
        wValue = (scbIndex << CY_SCB_INDEX_POS)
        wIndex = 0
        wBuffer = bytearray(CY_UART.CONFIG_LEN)

        ret = dev.controlWrite(bmRequestType, bmRequest,
                               wValue, wIndex, wBuffer, self.timeout)
        return ret

    def CyUartWrite(self, buff):
        dev = self.dev
        ret = dev.bulkWrite(self.ep_out, buff, timeout=self.timeout)
        return ret

    def CyUartRead(self, size):
        dev = self.dev

        # FIXME: need to loop and append buffer until full size is read
        ret = dev.bulkRead(self.ep_in, size, timeout=self.timeout)
        return ret

    def CyUartSetHwFlowControl(self, mode):
        self.uart_flowcontrol_mode = mode

        bmRequestType = CY_VENDOR_REQUEST | EP_OUT
        bmRequest = CY_VENDOR_CMDS.CY_UART_SET_FLOW_CONTROL_CMD
        wValue = mode
        wIndex = self.if_num
        wBuffer = bytearray(0)

        ret = self.dev.controlWrite(bmRequestType, bmRequest,
                                    wValue, wIndex, wBuffer, self.timeout)
        return ret

    def CyUartGetHwFlowControl(self):
        return self.uart_flowcontrol_mode

    def CyUartSetBreak(self, ms):
        bmRequestType = CY_VENDOR_REQUEST | EP_OUT
        bmRequest = CY_VENDOR_CMDS.CY_UART_SEND_BREAK_CMD
        wValue = ms
        wIndex = self.if_num
        wBuffer = bytearray(0)

        ret = self.dev.controlWrite(bmRequestType, bmRequest,
                                    wValue, wIndex, wBuffer, self.timeout)
        return ret

    def CyUartSetRts(self):
        bmRequestType = CY_VENDOR_REQUEST | EP_OUT
        bmRequest = CY_VENDOR_CMDS.CY_SET_LINE_CONTROL_STATE_CMD
        wValue = (1<<1) | self.dtrValue
        wIndex = self.if_num
        wBuffer = bytearray(0)

        ret = self.dev.controlWrite(bmRequestType, bmRequest,
                                    wValue, wIndex, wBuffer, self.timeout)
        self.rtsValue = 1
        return ret

    def CyUartClearRts(self):
        bmRequestType = CY_VENDOR_REQUEST | EP_OUT
        bmRequest = CY_VENDOR_CMDS.CY_SET_LINE_CONTROL_STATE_CMD
        wValue = self.dtrValue
        wIndex = self.if_num
        wBuffer = bytearray(0)

        ret = self.dev.controlWrite(bmRequestType, bmRequest,
                                    wValue, wIndex, wBuffer, self.timeout)
        self.rtsValue = 0
        return ret

    def CyUartSetDtr(self):
        bmRequestType = CY_VENDOR_REQUEST | EP_OUT
        bmRequest = CY_VENDOR_CMDS.CY_SET_LINE_CONTROL_STATE_CMD
        wValue = (self.rtsValue << 1) | 1
        wIndex = self.if_num
        wBuffer = bytearray(0)

        ret = self.dev.controlWrite(bmRequestType, bmRequest,
                                    wValue, wIndex, wBuffer, self.timeout)
        self.dtrValue = 1
        return ret

    def CyUartClearDtr(self):
        bmRequestType = CY_VENDOR_REQUEST | EP_OUT
        bmRequest = CY_VENDOR_CMDS.CY_SET_LINE_CONTROL_STATE_CMD
        wValue = (self.rtsValue << 1)
        wIndex = self.if_num
        wBuffer = bytearray(0)

        ret = self.dev.controlWrite(bmRequestType, bmRequest,
                                    wValue, wIndex, wBuffer, self.timeout)
        self.dtrValue = 0
        return ret

    def CyJtagEnable(self):
        bmRequestType = CY_VENDOR_REQUEST | EP_OUT
        bmRequest = CY_VENDOR_CMDS.CY_JTAG_ENABLE_CMD
        wValue = 0
        wIndex = 0
        wBuffer = bytearray(0)

        ret = self.dev.controlWrite(bmRequestType, bmRequest,
                                    wValue, wIndex, wBuffer, self.timeout)
        return ret
        
    def CyJtagDisable(self):
        bmRequestType = CY_VENDOR_REQUEST | EP_OUT
        bmRequest = CY_VENDOR_CMDS.CY_JTAG_DISABLE_CMD
        wValue = 0
        wIndex = 0
        wBuffer = bytearray(0)

        ret = self.dev.controlWrite(bmRequestType, bmRequest,
                                    wValue, wIndex, wBuffer, self.timeout)
        return ret

    def CyJtagWrite(self, buff):
        bmRequestType = CY_VENDOR_REQUEST | EP_OUT
        bmRequest = CY_VENDOR_CMDS.CY_JTAG_WRITE_CMD
        wValue = len(buff)
        wIndex = 0
        wBuffer = bytearray(0)

        ret = self.dev.controlWrite(bmRequestType, bmRequest,
                                    wValue, wIndex, wBuffer, self.timeout)

        ret = dev.bulkWrite(CY_JTAG_OUT_EP, buff, timeout=self.timeout)

    def CyJtagRead(self, size):
        bmRequestType = CY_VENDOR_REQUEST | EP_OUT
        bmRequest = CY_VENDOR_CMDS.CY_JTAG_READ_CMD
        wValue = size
        wIndex = 0
        wBuffer = bytearray(0)

        ret = self.dev.controlWrite(bmRequestType, bmRequest,
                                    wValue, wIndex, wBuffer, self.timeout)

        ret = dev.bulkRead(CY_JTAG_IN_EP, size, timeout=self.timeout)
        return ret

    def CyPhdcClrFeature(self):
        bmRequestType = CY_VENDOR_REQUEST | EP_OUT
        bmRequest = CY_VENDOR_CMDS.CY_PHDC_CLR_FEATURE
        wValue = CY_PHDC_CLR_FEATURE_WVALUE
        wIndex = self.if_num
        wBuffer = bytearray(0)

        ret = self.dev.controlWrite(bmRequestType, bmRequest,
                                    wValue, wIndex, wBuffer, self.timeout)
        return ret

    def CyPhdcSetFeature(self):
        bmRequestType = CY_VENDOR_REQUEST | EP_OUT
        bmRequest = CY_VENDOR_CMDS.CY_PHDC_SET_FEATURE
        wValue = CY_PHDC_SET_FEATURE_WVALUE
        wIndex = self.if_num
        wBuffer = bytearray(0)

        ret = self.dev.controlWrite(bmRequestType, bmRequest,
                                    wValue, wIndex, wBuffer, self.timeout)
        return ret

    def CyPhdcGetStatus(self):
        bmRequestType = CY_VENDOR_REQUEST | EP_IN
        bmRequest = CY_VENDOR_CMDS.CY_PHDC_GET_DATA_STATUS
        wValue = 0
        wIndex = self.if_num
        wLength = CY_PHDC_GET_STATUS_LEN

        ret = self.dev.controlRead(bmRequestType, bmRequest,
                                   wValue, wIndex, wLength, self.timeout)
        return ret

    def CyGetFirmwareVersion(self):
        bmRequestType = CY_VENDOR_REQUEST | EP_IN
        bmRequest = CY_VENDOR_CMDS.CY_GET_VERSION_CMD
        wValue = 0
        wIndex = 0
        wLength = CY_GET_FIRMWARE_VERSION_LEN

        ret = self.dev.controlRead(bmRequestType, bmRequest,
                                   wValue, wIndex, wLength, self.timeout)
        return ret

    def CyResetDevice(self):
        bmRequestType = CY_VENDOR_REQUEST | EP_IN
        bmRequest = CY_VENDOR_CMDS.CY_DEVICE_RESET_CMD
        wValue = 0xA6B6
        wIndex = 0xADBA
        wLength = 0

        ret = self.dev.controlRead(bmRequestType, bmRequest,
                                   wValue, wIndex, wLength, self.timeout)
        return ret

    def CySetGpioValue(self, gpio, value):
        bmRequestType = CY_VENDOR_REQUEST | EP_IN
        bmRequest = CY_VENDOR_CMDS.CY_GPIO_SET_VALUE_CMD
        wValue = gpio
        wIndex = value
        wLength = 0

        ret = self.dev.controlRead(bmRequestType, bmRequest,
                                   wValue, wIndex, wLength, self.timeout)
        return ret
        
    def CyGetGpioValue(self, gpio):
        bmRequestType = CY_VENDOR_REQUEST | EP_IN
        bmRequest = CY_VENDOR_CMDS.CY_GPIO_GET_VALUE_CMD
        wValue = gpio
        wIndex = 0
        wLength = CY_GPIO_GET_LEN

        ret = self.dev.controlRead(bmRequestType, bmRequest,
                                   wValue, wIndex, wLength, self.timeout)
        return ret

    def CyProgUserFlash(self, addr, buff):
        bmRequestType = CY_VENDOR_REQUEST | EP_OUT
        bmRequest = CY_VENDOR_CMDS.CY_PROG_USER_FLASH_CMD
        wValue = 0
        wIndex = addr
        wBuffer = buff

        ret = self.dev.controlWrite(bmRequestType, bmRequest,
                                    wValue, wIndex, wBuffer, self.timeout)
        return ret

    def CyReadUserFlash(self, addr, size):
        bmRequestType = CY_VENDOR_REQUEST | EP_IN
        bmRequest = CY_VENDOR_CMDS.CY_READ_USER_FLASH_CMD
        wValue = 0
        wIndex = addr
        wLength = size

        ret = self.dev.controlRead(bmRequestType, bmRequest,
                                   wValue, wIndex, wLength, self.timeout)
        return ret
        
    def CyGetSignature(self):
        bmRequestType = CY_VENDOR_REQUEST | EP_IN
        bmRequest = CY_VENDOR_CMDS.CY_GET_SIGNATURE_CMD
        wValue = 0
        wIndex = 0
        wLength = CY_GET_SIGNATURE_LEN

        ret = self.dev.controlRead(bmRequestType, bmRequest,
                                   wValue, wIndex, wLength, self.timeout)
        return ret

    def ping(self):
        """Send whatever USCU sends on startup"""
        bmRequestType = CY_VENDOR_REQUEST | EP_OUT
        bmRequest = 203
        wValue = 0
        wIndex = 0
        wBuffer = bytearray(0)

        ret = self.dev.controlWrite(bmRequestType, bmRequest,
                                    wValue, wIndex, wBuffer, self.timeout)
        return ret

    def probe0(self):
        """Send whatever USCU sends on startup - some signature?"""
        bmRequestType = CY_VENDOR_REQUEST | EP_IN
        bmRequest = 177
        wValue = 0
        wIndex = 0
        wLength = 4

        ret = self.dev.controlRead(bmRequestType, bmRequest,
                                   wValue, wIndex, wLength, self.timeout)
        return ret

    def probe1(self):
        """Send whatever USCU sends on startup - firmware version?"""
        bmRequestType = CY_VENDOR_REQUEST | EP_IN
        bmRequest = 176
        wValue = 0
        wIndex = 0
        wLength = 8

        ret = self.dev.controlRead(bmRequestType, bmRequest,
                                   wValue, wIndex, wLength, self.timeout)
        return ret
        
    def connect(self):
        """Send whatever USCU sends on connect"""
        bmRequestType = CY_VENDOR_REQUEST | EP_OUT
        bmRequest = 226
        wValue = 0xa6bc
        wIndex = 0xb1b0
        wBuffer = bytearray(0)

        ret = self.dev.controlWrite(bmRequestType, bmRequest,
                                    wValue, wIndex, wBuffer, self.timeout)
        return ret

    def disconnect(self):
        """Send whatever USCU sends on disconnect"""
        bmRequestType = CY_VENDOR_REQUEST | EP_OUT
        bmRequest = 226
        wValue = 0xa6bc
        wIndex = 0xb9b0
        wBuffer = bytearray(0)

        ret = self.dev.controlWrite(bmRequestType, bmRequest,
                                    wValue, wIndex, wBuffer, self.timeout)
        return ret

    def read_config(self):
        """Send whatever USCU sends on config read"""
        bmRequestType = CY_VENDOR_REQUEST | EP_IN
        bmRequest = 181
        wValue = 0
        wIndex = 0
        wLength = 512

        ret = self.dev.controlRead(bmRequestType, bmRequest,
                                   wValue, wIndex, wLength, self.timeout)
        return ret

    def write_config(self, buff):
        """Send whatever USCU sends on config write"""
        bmRequestType = CY_VENDOR_REQUEST | EP_OUT
        bmRequest = 182
        wValue = 0
        wIndex = 0

        if len(buff) != 512:
            print("FIXME: need to have valid buffer")
            sys.exit(0)
        wBuffer = buff

        ret = self.dev.controlWrite(bmRequestType, bmRequest,
                                    wValue, wIndex, wBuffer, self.timeout)
        return ret

def main():
    with USBContext() as context:
        # ux == USB Device/Configuration/Interface/Setting/Endpoint
        ud = list(find_device(context, VID, PID))[0]
        uc = ud[0]
        ui = uc[0]
        us = ui[0]
        ue = us[0]

        with CyUSB.open(ud, CY_TYPE.I2C) as cyd:
            embed()

if __name__ == '__main__':
    main()
