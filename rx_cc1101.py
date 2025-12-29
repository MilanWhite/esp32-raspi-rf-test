#!/usr/bin/env python3
# Works on Python 3.7+
# CC1101 RX (packet mode, variable length, CRC) over /dev/spidev0.0

import time
import spidev

# ---- CC1101 register map (config regs) ----
IOCFG2    = 0x00
IOCFG0    = 0x02
PKTCTRL1  = 0x07
PKTCTRL0  = 0x08
ADDR      = 0x09
CHANNR    = 0x0A
FSCTRL1   = 0x0B
FSCTRL0   = 0x0C
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
PATABLE   = 0x3E
FIFO      = 0x3F

# ---- CC1101 status regs ----
LQI       = 0x33
RSSI      = 0x34
RXBYTES   = 0x3B

# ---- CC1101 strobes ----
SRES      = 0x30
SIDLE     = 0x36
SRX       = 0x34
SFRX      = 0x3A

READ_SINGLE = 0x80
READ_BURST  = 0xC0
WRITE_BURST = 0x40

def mhz_to_freq_regs(mhz, fxosc_hz=26_000_000):
    # CC1101: Freq = (FREQ / 2^16) * fxosc
    # => FREQ = mhz*1e6 * 2^16 / fxosc
    freq_word = int((mhz * 1_000_000.0) * (1 << 16) / fxosc_hz)
    return (freq_word >> 16) & 0xFF, (freq_word >> 8) & 0xFF, freq_word & 0xFF

class CC1101:
    def __init__(self, bus=0, dev=0, speed_hz=4_000_000):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, dev)
        self.spi.max_speed_hz = speed_hz
        self.spi.mode = 0

    def close(self):
        try:
            self.spi.close()
        except Exception:
            pass

    def strobe(self, cmd):
        self.spi.xfer2([cmd])

    def write_reg(self, addr, val):
        self.spi.xfer2([addr & 0x3F, val & 0xFF])

    def read_reg(self, addr):
        # Works for both config regs and status regs
        resp = self.spi.xfer2([addr | READ_SINGLE, 0x00])
        return resp[1]

    def write_burst(self, addr, data):
        self.spi.xfer2([addr | WRITE_BURST] + [b & 0xFF for b in data])

    def read_burst(self, addr, n):
        resp = self.spi.xfer2([addr | READ_BURST] + [0x00] * n)
        return resp[1:]

    def reset(self):
        # Typical userspace reset (CS handled by spidev)
        time.sleep(0.01)
        self.strobe(SRES)
        time.sleep(0.05)

    def apply_config_like_common_senddata(self, mhz=915.0):
        """
        Packet mode (FIFO), variable length, CRC enabled, append status (RSSI/LQI).
        Register choices here are aligned with common Arduino CC1101 "SendData()/ReceiveData()" setups.
        """
        self.reset()

        # Base settings (these values match widely-used ELECHOUSE/SmartRC-style defaults)
        self.write_reg(FSCTRL1, 0x06)

        # GDOs: not required for this polling receiver, but set to common "packet" values
        self.write_reg(IOCFG2, 0x0B)
        self.write_reg(IOCFG0, 0x06)

        # Packet automation
        self.write_reg(PKTCTRL1, 0x04)  # APPEND_STATUS = 1
        self.write_reg(PKTCTRL0, 0x05)  # PKT_FORMAT=0, CRC_EN=1, LENGTH_CONFIG=1 (variable)

        self.write_reg(ADDR, 0x00)
        self.write_reg(CHANNR, 0x00)

        # Frequency
        f2, f1, f0 = mhz_to_freq_regs(mhz)
        self.write_reg(FREQ2, f2)
        self.write_reg(FREQ1, f1)
        self.write_reg(FREQ0, f0)

        # Modem config (conservative, common stable settings)
        # If your ESP32 library uses different modulation/data rate, these must match on both ends.
        self.write_reg(MDMCFG4, 0xCA)
        self.write_reg(MDMCFG3, 0x83)
        self.write_reg(MDMCFG2, 0x13)  # 2-FSK + 16/16 sync
        self.write_reg(MDMCFG1, 0x22)
        self.write_reg(MDMCFG0, 0xF8)
        self.write_reg(DEVIATN, 0x35)

        # Front end / calibration / tests (common TI recommended-like)
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

        # Enter RX
        self.strobe(SIDLE)
        self.strobe(SFRX)
        self.strobe(SRX)

    def recv_packet(self):
        # returns (payload_bytes, rssi_raw, lqi_raw, crc_ok) or None
        rxbytes = self.read_reg(RXBYTES) & 0x7F
        if rxbytes == 0:
            return None

        # First byte in FIFO is payload length in variable-length mode
        length = self.read_burst(FIFO, 1)[0]
        if length == 0 or length > 61:
            # Flush on nonsense; adjust 61 if you expect longer payloads
            self.strobe(SIDLE)
            self.strobe(SFRX)
            self.strobe(SRX)
            return None

        payload = bytes(self.read_burst(FIFO, length))

        # Two appended status bytes: RSSI, LQI/CRC_OK (CRC_OK is bit 7 of LQI)
        status = self.read_burst(FIFO, 2)
        rssi_raw = status[0]
        lqi_raw = status[1]
        crc_ok = (lqi_raw & 0x80) != 0

        # Flush RX FIFO + return to RX
        self.strobe(SIDLE)
        self.strobe(SFRX)
        self.strobe(SRX)

        return payload, rssi_raw, lqi_raw, crc_ok

def main():
    radio = CC1101(bus=0, dev=0, speed_hz=4_000_000)
    try:
        radio.apply_config_like_common_senddata(mhz=915.0)
        print("CC1101 RX ready on 915.0 MHz (polling RXFIFO). Ctrl+C to stop.")

        while True:
            pkt = radio.recv_packet()
            if pkt is None:
                time.sleep(0.01)
                continue

            payload, rssi_raw, lqi_raw, crc_ok = pkt
            # Your ESP32 sends a null-terminated C string, so decode safely:
            msg = payload.split(b"\x00", 1)[0].decode("utf-8", errors="replace")
            print("CRC_OK=%s RSSI_RAW=0x%02X LQI=0x%02X  MSG=%s" % (crc_ok, rssi_raw, lqi_raw, msg))
    finally:
        radio.close()

if __name__ == "__main__":
    main()
