
# EDGE_XL Board Standalone Self-Test Quick Guide (USB Version)

For: Factory operators (use only one USB cable to quickly check self-test results).

——
## What You Need
- One EDGE_XL board to be tested
- One USB Micro cable
- A computer (Windows or Mac) with any serial terminal tool

——
## Connection & Setup
1) Connect the EDGE_XL board to your computer using the USB cable.
2) Open your serial terminal tool and select the newly appeared “USB‑Serial‑JTAG” port (check Device Manager for the exact COM port).
3) Set serial parameters: 115200 baud rate, 8 data bits, no parity, 1 stop bit (115200, 8N1).

——
## Viewing Results (Auto-refresh every ~3 seconds)
The terminal will continuously print a concise summary, for example:
```
================= SELFTEST RESULT =================
EG915       : PASS/FAIL
Motion      : PASS/FAIL (|a|=x.xxxg)
RS485       : PASS/FAIL (written=8, rx=N)
CAN         : PASS/FAIL (inited=1, started=1, state=S)
GNSS UART   : PASS/FAIL (bytes=N)
Battery/ADC : PASS/FAIL (V=X.XX)
IGN Opto    : PASS/FAIL
---------------------------------------------------
OVERALL     : PASS/FAIL
```
Refer to “OVERALL” for the final judgment. PASS = qualified; FAIL = needs recheck.

——
## Judgment Criteria (USB Only)
- Must PASS: EG915, Motion, GNSS, Battery/ADC
- May FAIL due to missing external conditions: RS485, CAN, IGN (can be ignored for standalone test)
  - If only these items FAIL, and all “Must PASS” items are PASS, the board can be considered basically qualified.

——
## Common Issues
- Can’t find the serial port:
  - Replug the USB cable; check Device Manager for the USB‑Serial‑JTAG port.
- No output for a long time:
  - Press the reset button on the board or replug the USB cable, then wait 3–5 seconds.
- Still FAIL:
  - Record the log and ask technical staff to recheck.

——
## Flashing the Firmware (How to Burn)

### 1. Prepare
- Make sure you have the correct firmware files (usually in the `build` folder, e.g. `pe_board.bin`, `bootloader.bin`, `partition-table.bin`).
- Install Python and [esptool](https://github.com/espressif/esptool) if not already installed:
  ```bash
  pip install esptool
  ```

### 2. Find the USB Serial Port
- Plug the EDGE_XL board into your computer via USB.
- On Windows: Check Device Manager for a port like `COMx`.
- On Mac: Use `ls /dev/tty.*` to find something like `/dev/tty.usbserial-xxxx`.

### 3. Flash Command Example
- Open a terminal and run (replace the port and file paths as needed):
  ```bash
  esptool.py --chip esp32s3 --port <your_port> --baud 921600 write_flash -z 0x0 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin 0x10000 build/pe_board.bin
  ```
- Example for Windows:
  ```bash
  esptool.py --chip esp32s3 --port COM5 --baud 921600 write_flash -z 0x0 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin 0x10000 build/pe_board.bin
  ```
- Example for Mac:
  ```bash
  esptool.py --chip esp32s3 --port /dev/tty.usbserial-1234 --baud 921600 write_flash -z 0x0 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin 0x10000 build/pe_board.bin
  ```

### 4. After Flashing
- The board will automatically reboot and start self-testing.
- You can now follow the USB viewing steps above to check the results.

——
## Notes
- No test jig or external equipment required.
- No need to send any commands; self-test runs and outputs results automatically after power-on.
