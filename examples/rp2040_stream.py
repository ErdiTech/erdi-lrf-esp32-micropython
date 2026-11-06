"""Same driver on a Raspberry Pi Pico (UART1: TX=GP4, RX=GP5)."""
import time
from erdi_lrf import open_uart

lrf = open_uart("LRF50VB", uart_id=1, tx=4, rx=5)
while True:
    print(lrf.read_latest())
    time.sleep_ms(50)
