#!/usr/bin/env python3
from time import sleep
from binascii import hexlify

from cc1101.config import RXConfig, Modulation
from cc1101 import CC1101

DEVICE = "/dev/cc1101.0.0"

# MUST match your ESP32 radio settings
FREQUENCY_MHZ = 915        # 433 / 868 / 915
MODULATION = Modulation.FSK_2
BAUD_KBAUD = 1             # data rate in kBaud
PACKET_LENGTH = 64         # fixed number of bytes the driver will read per packet
SYNC_WORD = 0x0000         # 0x0000 disables sync word (see docs)

rx_config = RXConfig.new(
    frequency=FREQUENCY_MHZ,
    modulation=MODULATION,
    baud_rate=BAUD_KBAUD,
    sync_word=SYNC_WORD,
    packet_length=PACKET_LENGTH,
)

radio = CC1101(DEVICE, rx_config, blocking=True)

print("Listening on {} MHz, {}, {} kBaud, len={}".format(
    FREQUENCY_MHZ, MODULATION.name, BAUD_KBAUD, PACKET_LENGTH
))

while True:
    packets = radio.receive()
    for p in packets:
        # show hex always
        print("RX hex:", hexlify(p).decode("ascii"))

        # also try ASCII (strip trailing nulls)
        try:
            s = p.split(b"\x00", 1)[0].decode("utf-8", errors="ignore")
            if s:
                print("RX txt:", s)
        except Exception:
            pass

    sleep(0.05)
