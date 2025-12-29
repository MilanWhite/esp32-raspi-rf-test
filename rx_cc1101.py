#!/usr/bin/env python3
# CC1101 RX over /dev/spidev0.0 (userspace only). Python 3.7+.

import time
import spidev

# ---- CC1101 register map ----
IOCFG2    = 0x00
IOCFG0    = 0x02
PKTCTRL1  = 0x07
PKTCTRL0  = 0x08
FSCTRL1   = 0x0B
FREQ2     = 0x0D
FREQ1     = 0x0E
FREQ0     = 0x0F
MDMCFG4   = 0x10
MDMCFG3   = 0x11
MDMCFG2   = 0x12
MDMCFG1   = 0x13
MDMCFG0   = 0x14
DEVIATN   = 0x15
FREND1    = 0x21
FREND0    = 0x22
MCSM0     = 0x18
FOCCFG    = 0x19
BSCFG     = 0x1A
AGCCTRL2  = 0x1B
AGCCTRL1  = 0x1C
AGCCTRL0  = 0x1D
FSCAL3    = 0x23
FSCAL2    = 0x24
FSCAL1    = 0x25
FSCAL0    = 0x26
FSTEST    = 0x29
TEST2     = 0x2C
TEST1     = 0x2D
TEST0     = 0x2E
FIFO      = 0x3F

# ---- Status regs ----
PARTNUM   = 0x30
VERSION   = 0x31
RXBYTES   = 0x3B

# ---- Strobes ----
SRES      = 0x30
SIDLE     = 0x36
SRX       = 0x34
SFRX      = 0x3A

READ_SINGLE = 0x80
READ_BURST  = 0xC0
WRITE_BURST = 0x40

def mhz_to_freq_regs(mhz, fxosc_hz=26_000_000):
    w = int((mhz * 1_000_000.0) * (1 << 16) / fxosc_hz)
    return (w >> 16) & 0xFF, (w >> 8) & 0xFF, w & 0xFF

def is_status_reg(addr):
    # CC1101 status regs are 0x30..0x3D (incl. PARTNUM/VERSION/RXBYTES)
    return 0x30 <= addr <= 0x3D

class CC1101:
    def __init__(self, bus=0, dev=0, speed_hz=500_000):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, dev)
        self.spi.max_speed_hz = speed_hz
        self.spi.mode = 0

    def close(self):
        self.spi.close()

    def strobe(self, cmd):
        self.spi.xfer2([cmd])

    def write_reg(self, addr, val):
        self.spi.xfer2([addr & 0x3F, val & 0xFF])

    def read_reg(self, addr):
        # Config regs: 0x80|addr
        # Status regs: must use 0xC0|addr (burst bit set) even for 1 byte
        cmd = (READ_BURST if is_status_reg(addr) else READ_SINGLE) | (addr & 0x3F)
        resp = self.spi.xfer2([cmd, 0x00])
        return resp[1]

    def read_burst(self, addr, n):
        resp = self.spi.xfer2([addr | READ_BURST] + [0x00] * n)
        return resp[1:]

    def reset(self):
        time.sleep(0.01)
        self.strobe(SRES)
        time.sleep(0.05)

    def config_rx(self, mhz=915.0):
        self.reset()

        # Prove SPI+CS works
        part = self.read_reg(PARTNUM)
        ver  = self.read_reg(VERSION)
        print("PARTNUM=0x%02X VERSION=0x%02X" % (part, ver), flush=True)

        # Base (common stable defaults)
        self.write_reg(FSCTRL1, 0x06)
        self.write_reg(IOCFG2, 0x0B)
        self.write_reg(IOCFG0, 0x06)

        self.write_reg(PKTCTRL1, 0x04)  # APPEND_STATUS=1 (RSSI/LQI bytes appended)
        # IMPORTANT: your ESP32 library has no setCRC(); assume CRC OFF for compatibility
        self.write_reg(PKTCTRL0, 0x01)  # variable length, CRC OFF

        f2, f1, f0 = mhz_to_freq_regs(mhz)
        self.write_reg(FREQ2, f2)
        self.write_reg(FREQ1, f1)
        self.write_reg(FREQ0, f0)

        # Modem settings (2-FSK + sync). Must match ESP32 defaults to decode.
        self.write_reg(MDMCFG4, 0xCA)
        self.write_reg(MDMCFG3, 0x83)
        self.write_reg(MDMCFG2, 0x13)
        self.write_reg(MDMCFG1, 0x22)
        self.write_reg(MDMCFG0, 0xF8)
        self.write_reg(DEVIATN, 0x35)

        self.write_reg(FREND1, 0x56)
        self.write_reg(FREND0, 0x10)
        self.write_reg(MCSM0,  0x18)
        self.write_reg(FOCCFG, 0x16)
        self.write_reg(BSCFG,  0x1C)
        self.write_reg(AGCCTRL2, 0xC7)
        self.write_reg(AGCCTRL1, 0x00)
        self.write_reg(AGCCTRL0, 0xB2)
        self.write_reg(FSCAL3, 0xE9)
        self.write_reg(FSCAL2, 0x2A)
        self.write_reg(FSCAL1, 0x00)
        self.write_reg(FSCAL0, 0x1F)
        self.write_reg(FSTEST, 0x59)
        self.write_reg(TEST2,  0x81)
        self.write_reg(TEST1,  0x35)
        self.write_reg(TEST0,  0x09)

        self.strobe(SIDLE)
        self.strobe(SFRX)
        self.strobe(SRX)

    def recv_packet(self):
        rxbytes = self.read_reg(RXBYTES) & 0x7F
        if rxbytes == 0:
            return None

        length = self.read_burst(FIFO, 1)[0]
        if length == 0 or length > 61:
            self.strobe(SIDLE); self.strobe(SFRX); self.strobe(SRX)
            return None

        payload = bytes(self.read_burst(FIFO, length))

        # appended status bytes (RSSI, LQI) still arrive if APPEND_STATUS=1
        _status = self.read_burst(FIFO, 2)

        self.strobe(SIDLE); self.strobe(SFRX); self.strobe(SRX)
        return payload

def main():
    radio = CC1101(bus=0, dev=0, speed_hz=500_000)  # CE0 -> /dev/spidev0.0
    try:
        radio.config_rx(mhz=915.0)
        print("CC1101 RX ready", flush=True)

        last = time.time()
        while True:
            pkt = radio.recv_packet()
            if pkt:
                msg = pkt.split(b"\x00", 1)[0].decode("utf-8", errors="replace")
                print("RX:", msg, flush=True)

            if time.time() - last > 2:
                print("alive", flush=True)
                last = time.time()

            time.sleep(0.01)
    finally:
        radio.close()

if __name__ == "__main__":
    main()
