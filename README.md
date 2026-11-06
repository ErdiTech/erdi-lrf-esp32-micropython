# ERDI laser rangefinder on ESP32 with MicroPython

[![ci](https://github.com/ErdiTech/erdi-lrf-esp32-micropython/actions/workflows/ci.yml/badge.svg)](https://github.com/ErdiTech/erdi-lrf-esp32-micropython/actions/workflows/ci.yml)

A single-file **MicroPython driver for ERDI 905 nm laser rangefinder modules on ESP32**, Raspberry Pi Pico (RP2040) and STM32 boards. It reads the **LRF22VB / LRF50VB / LRF50H-LRF1500H** 460800-baud `5C` stream and polls the **LR1000E2 ... LR2000E4, LRF1200A1, LRF3000A1, SPD1200N\*** `55 AA` single-shot modules. Use it for a **laser distance sensor on ESP32**, a **UART lidar rangefinder** with MicroPython, drone altitude hold experiments, robot obstacle sensing and level measurement.

- One file, no dependencies: [`erdi_lrf.py`](erdi_lrf.py)
- Pure-Python frame parsing, unit-tested under CPython (`pytest`)
- Units handled per model: **H series = centimetres, VB series = millimetres, family A = metres x 10**

## Install

```python
# on the board, with Wi-Fi up
import mip
mip.install("github:ErdiTech/erdi-lrf-esp32-micropython")
```

or copy `erdi_lrf.py` to the board with `mpremote cp erdi_lrf.py :`.

## Wiring (ESP32, UART2)

| Module | ESP32 |
|---|---|
| TX | GPIO16 (RX2) |
| RX | GPIO17 (TX2) |
| GND | GND |
| VCC | its own supply per the model manual |

Check the module's TX logic level against 3.3 V before connecting. LRF22VB is a 3.0-3.6 V part; LRF50VB 3.3-5 V; H series 9-36 V supply (UART level per manual). Supply voltage is not the UART voltage: use a level shifter if they differ. Model pages and datasheets: <https://erdilrf.com/models/?utm_source=github&utm_medium=readme&utm_campaign=erdi-lrf-esp32-micropython>.

## Quick start

```python
from erdi_lrf import open_uart
lrf = open_uart("LRF22VB", uart_id=2, tx=17, rx=16)   # 460800 baud picked from the model table
print(lrf.read_latest())                               # metres, or None if out of range
```

Single-shot modules (family A):

```python
lrf = open_uart("LRF3000A1")        # 115200 baud
print(lrf.measure())                # metres, None if no valid echo, OSError on timeout
```

More: [`examples/esp32_stream.py`](examples/esp32_stream.py), [`examples/esp32_single_shot_a.py`](examples/esp32_single_shot_a.py), [`examples/rp2040_stream.py`](examples/rp2040_stream.py).

## Frame formats (from the ERDI manuals)

| Family | Frame | Check | Baud |
|---|---|---|---|
| B (stream) | `5C lo hi chk` (LRF1500H: `5C lo mid hi chk`) | `~(sum of distance bytes)` | 460800 |
| A (poll) | request `55 AA 88 FF FF FF FF 84`, reply `55 AA 88 STA FF DH DL SUM` | reply sum includes the two header bytes, request sum does not | 115200 |

`5C 02 11 EC` is **43.54 m** on an H-series module and **4.354 m** on a VB-series module, so the driver never infers the unit from the family. Full protocol notes: [ErdiTech/erdi-lrf-drivers](https://github.com/ErdiTech/erdi-lrf-drivers).

## Tests

```bash
pip install pytest
python -m pytest -q
```

The tests use the golden frames printed in the manuals. They do not replace a bench test with your module, cable and supply.

## Related projects

- [ErdiTech/erdi-lrf-drivers](https://github.com/ErdiTech/erdi-lrf-drivers): Python, Arduino and ROS 2 drivers, protocol documents
- [laser-rangefinder-module-database](https://github.com/ErdiTech/laser-rangefinder-module-database): open dataset of laser rangefinder module specifications
- [lrf-uart-protocol-decoder](https://github.com/ErdiTech/lrf-uart-protocol-decoder): browser and Python UART frame decoder

## Support / 打赏

If this project saved you time, you can buy the author a coffee in crypto. Thank you!
如果这个项目帮到了你，欢迎用加密货币打赏支持，谢谢！

| Network | Asset | Address |
|---|---|---|
| TRON (TRC20) | USDT | `TNGdTwE6opH6rmUyoG1JUR7FSyEb4k273R` |
| Ethereum / BNB Chain / Polygon | USDT | `0x59c43144a862b351fa1086ae842d1327c9d91613` |
| Ethereum / Arbitrum / Base | ETH | `0x59c43144a862b351fa1086ae842d1327c9d91613` |
| Bitcoin | BTC | `1RS6tWXHregLfEsiPzVUCUBuhgDg5uTSV` |
| Solana | USDT | `HZguEXfTtLChNdJTxAYCbcGscaqikz8JYrEpUhEyeVd9` |

Please send only the listed asset on the listed network — other tokens or networks may be lost.
请按表中对应的网络转对应的币种，转错币种或网络可能无法找回。

Built by tommyapi.com

## License

MIT — see [LICENSE](LICENSE).
