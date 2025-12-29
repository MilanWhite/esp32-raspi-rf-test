#!/usr/bin/env python3

from cc1101 import CC1101
import time

# -------- CONFIG --------
SPI_BUS = 0          # SPI bus 0
SPI_DEVICE = 0       # CE0 -> /dev/spidev0.0
FREQ_MHZ = 915.0     # MUST match ESP32
# ------------------------

# IMPORTANT:
# CC1101(bus, device)
radio = CC1101(SPI_BUS, SPI_DEVICE)

radio.reset()
radio.set_frequency_mhz(FREQ_MHZ)
radio.set_crc(True)
radio.set_packet_mode()
radio.rx()

print("CC1101 RX ready")

while True:
    if radio.packet_available():
        pkt = radio.receive_packet()
        if pkt:
            try:
                print("RX:", pkt.decode("utf-8", errors="ignore"))
            except Exception:
                print("RX raw:", pkt)
    time.sleep(0.05)
