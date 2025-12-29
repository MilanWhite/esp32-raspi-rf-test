#!/usr/bin/env python3

from cc1101 import CC1101
import time

# ---------- CONFIG (must match ESP32) ----------
FREQ_MHZ = 915.0        # 433.92 / 868.0 / 915.0
SPI_BUS = 0
SPI_DEVICE = 0          # CE0 -> /dev/spidev0.0
GDO0_GPIO = 25          # optional but recommended
# -----------------------------------------------

radio = CC1101(
    spi_bus=SPI_BUS,
    spi_device=SPI_DEVICE,
    gdo0=GDO0_GPIO
)

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
