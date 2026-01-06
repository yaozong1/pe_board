## System Overview

```mermaid
graph TB
    PC["PC (Windows)<br/>Factory Test GUI (Python)"]
    PC -->|USB<br/>Control Port<br/>USB-Serial-JTAG| Jig
    PC -->|USB<br/>Flash Port<br/>CH340| DUT
    
    Jig["Jig (Test Fixture)<br/>- RS485 Loopback<br/>- CAN Loopback<br/>- IGN Generator"]
    DUT["DUT (Device Under Test)<br/>- RS485 Test<br/>- CAN Test<br/>- IGN Input"]
    
    Jig <-->|Test Cables| DUT
    
    style PC fill:#e1f5ff
    style Jig fill:#fff4e1
    style DUT fill:#e8f5e9
```

**Components:**
- **PC**: Runs test GUI software
- **Jig (Test Fixture)**: Provides RS485/CAN loopback and IGN signal
- **DUT (Device Under Test)**: PE board to be tested
- **Test Cables**: RS485, CAN, IGN connection cables

---

## Hardware Setup

### 1. Connect Test Cables (Jig ↔ DUT)

```mermaid
graph LR
    subgraph Jig["Jig (Test Fixture)"]
        J1[RS485_A]
        J2[RS485_B]
        J3[CAN_H]
        J4[CAN_L]
        J5[IGN_OUT]
        J6[GND]
    end
    
    subgraph DUT["DUT (Device Under Test)"]
        D1[RS485_A]
        D2[RS485_B]
        D3[CAN_H]
        D4[CAN_L]
        D5[IGN_IN]
        D6[GND]
    end
    
    J1 -->|Test Cable| D1
    J2 -->|Test Cable| D2
    J3 -->|Test Cable| D3
    J4 -->|Test Cable| D4
    J5 -->|Test Cable| D5
    J6 -->|Test Cable| D6
    
    style Jig fill:#fff4e1
    style DUT fill:#e8f5e9
```

### 2. Connect USB Cables (PC ↔ Jig/DUT)

| Device | USB Type | Port Identification | Purpose |
|--------|----------|---------------------|---------|
| **Jig** | USB-Serial-JTAG | "USB Serial" / "JTAG" | Control Port (Jig Control) |
| **DUT** | CH340 | "CH340" / "CH343" / "CH9102" | Flash Port (Flashing/Testing) |

### 3. Power Check

- ✅ Jig powered normally (LED indicator)
- ✅ DUT powered normally
- ✅ All cables securely connected

---

## Software Setup

### Install Python Dependencies (First time only)

```bash
pip install PySide6 pyserial esptool
```

Detailed installation guide: [GUI_SETUP.md](tools/factory_gui/GUI_SETUP.md)

### Build Firmware (Only when code updates)

```bash
cd pe_board
idf.py build
```

---

## Test Procedure

### Test Flow Diagram

```mermaid
flowchart TD
    Start(["1. Launch GUI<br/>python tools\factory_gui\factory_gui.py"])
    Port(["2. Select Ports (Auto-detect)<br/>Control Port: Jig (USB-Serial-JTAG)<br/>Flash Port: DUT (CH340)"])
    Flash(["3. Flash Firmware<br/>Click [Flash DUT]<br/>Wait 30-60 seconds"])
    Test(["4. Start Test<br/>Click [Start Test]"])
    
    ReqJig["GUI → Jig<br/>!GUI_REQUEST_DATA<br/>(every 500ms)"]
    JigResp["Jig → GUI<br/>JIG PAYLOAD: {...}<br/>(RS485/CAN/IGN ready)"]
    TrigDUT["GUI → DUT<br/>!GUI_SELFTEST_ACK<br/>(trigger test)"]
    
    DUTTest["DUT Self-Test (15-20s)<br/>├─ EG915 Module<br/>├─ Motion Sensor<br/>├─ RS485 (4s timeout)<br/>├─ CAN (4s timeout)<br/>├─ Battery/IBL ADC<br/>├─ GNSS UART<br/>└─ IGN Optocoupler"]
    
    DUTWait["DUT → GUI<br/>Waiting for ACK"]
    SendACK["GUI → DUT<br/>!GUI_SELFTEST_ACK<br/>(every 500ms)"]
    DUTOutput["DUT → GUI<br/>SELFTEST SUMMARY: {...}<br/>(JSON format)"]
    
    Result(["5. View Results<br/>✅ Green: Pass<br/>❌ Red: Fail"])
    
    Start --> Port --> Flash --> Test
    Test --> ReqJig --> JigResp --> TrigDUT --> DUTTest
    DUTTest --> DUTWait --> SendACK --> DUTOutput --> Result
    
    style Start fill:#e1f5ff
    style Port fill:#e1f5ff
    style Flash fill:#fff4e1
    style Test fill:#e8f5e9
    style Result fill:#e8f5e9
    style DUTTest fill:#ffe1f5
```

---

## Serial Command Protocol

### GUI → Jig (Control Port)

```
Command: !GUI_REQUEST_DATA
Purpose: Request Jig to send test ready status
Frequency: Every 500ms (20s timeout)
Response: JIG PAYLOAD: {"rs485_loopback": true, "can_loopback": true, ...}
```

### GUI → DUT (Flash Port)

```
Command: !GUI_SELFTEST_ACK
Purpose: ① Start DUT self-test  ② Trigger result output
Frequency: Every 500ms (20s timeout)
Response: SELFTEST SUMMARY: {"eg915_ok": true, "motion": {...}, ...}
```

---

## Test Items

| Item | Test Content | Pass Criteria | Timeout |
|------|--------------|---------------|---------|
| **EG915** | AT command comm., IMEI, ICCID | AT response OK | 2s |
| **Motion** | Magnetometer reading | mag > 0.1 | 1s |
| **RS485** | Loopback test (send 8 bytes) | Receive 8 bytes match | 4s |
| **CAN** | Loopback test (send 8 bytes) | Receive 8 bytes match | 4s |
| **Battery** | Battery voltage ADC | 0.441V ~ 0.539V | 1s |
| **IBL** | IO2 voltage ADC | > 0 mV | 1s |
| **GNSS** | UART data reception | > 0 bytes | 800ms |
| **IGN** | Optocoupler level change | HIGH→LOW→HIGH | 1s |

**Total Test Time**: 15-25 seconds

---

## Troubleshooting

### ❌ Port Not Detected

**Symptom**: No serial ports in dropdown menu

**Solution**:
1. Check USB cable connection
2. Click "Refresh Ports" button
3. Verify COM port in Device Manager
4. Install CH340 driver

---

### ❌ Flash Failed

**Symptom**: "Flash failed" error

**Solution**:
1. Confirm DUT is powered on
2. Replace USB cable (not charge-only cable)
3. Select correct Flash Port
4. Hold BOOT button and reconnect

---

### ❌ RS485/CAN Test Failed

**Symptom**: RS485 or CAN shows FAIL

**Solution**:
1. **If 4-second timeout**: 
   - Check cable connections between Jig and DUT
   - Confirm GND is connected
   - Test if Jig is working properly

2. **If hardware damaged**:
   - DUT will still complete test after 4 seconds
   - Result shows FAIL but doesn't affect other tests
   - Other test items execute normally

---

### ❌ Test Timeout

**Symptom**: "Timeout waiting for test results"

**Solution**:
1. Confirm all test cables connected (RS485/CAN/IGN)
2. Confirm Jig is powered and running
3. Re-flash DUT firmware
4. Restart GUI and retry test

---