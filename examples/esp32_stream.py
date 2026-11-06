"""LRF22VB / LRF50VB / H-series stream on an ESP32 (UART2: TX=GPIO17, RX=GPIO16)."""
import time
from erdi_lrf import open_uart

lrf = open_uart("LRF22VB", uart_id=2, tx=17, rx=16)
while True:
    d = lrf.read_latest()
    print("out of range / no frame yet" if d is None else "%.3f m" % d)
    time.sleep_ms(50)
