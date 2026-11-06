"""LR1000E2 / LRF3000A1 / SPD1200N* single-shot ranging (family A, 115200 baud)."""
import time
from erdi_lrf import open_uart

lrf = open_uart("LRF3000A1", uart_id=2, tx=17, rx=16)
while True:
    try:
        d = lrf.measure()
        print("no valid echo" if d is None else "%.1f m" % d)
    except OSError as e:
        print(e)
    time.sleep(1)
