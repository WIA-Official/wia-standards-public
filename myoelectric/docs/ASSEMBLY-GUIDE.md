# WIA Myoelectric Prosthetic Assembly Guide

Complete guide for assembling a WIA-compatible EMG-controlled prosthetic hand.

## Overview

Total cost: **~$100**
Build time: **4-8 hours**
Skill level: **Intermediate** (basic soldering and 3D printing)

## Bill of Materials

### EMG Electronics (~$48)

| Component | Quantity | Approx. Cost |
|-----------|----------|--------------|
| ESP32-S3 Dev Board | 1 | $8 |
| AD8232 ECG Module | 2 | $12 |
| Ag/AgCl Electrodes (pack of 50) | 1 | $8 |
| 3.5mm Audio Jacks | 2 | $2 |
| LiPo Battery 3.7V 1000mAh | 1 | $6 |
| TP4056 Charging Module | 1 | $2 |
| Toggle Switch | 1 | $1 |
| Prototype PCB | 1 | $3 |
| Wires, connectors | - | $6 |

### Prosthetic Hand (~$40)

| Component | Quantity | Approx. Cost |
|-----------|----------|--------------|
| SG90 Micro Servos | 5 | $15 |
| PLA/PETG Filament | 200g | $5 |
| Braided Fishing Line (50lb) | 1 | $5 |
| Elastic Cord (1mm) | 2m | $2 |
| M3 Screws & Nuts Kit | 1 | $5 |
| Velcro Straps | 2 | $3 |
| Foam Padding | 1 | $3 |
| Heat Shrink Tubing | 1 | $2 |

### Tools Required

- 3D Printer (or access to one)
- Soldering iron + solder
- Wire strippers
- Screwdrivers (Phillips #1, #2)
- Needle-nose pliers
- Hot glue gun
- Multimeter
- Computer with Arduino IDE

## Step 1: Print the Prosthetic Hand

### Recommended Designs

1. **Ada Hand** (OpenBionics) - Best for beginners
   - Download: openbionics.com/ada
   - Print time: ~20 hours
   - Material: PLA or PETG

2. **Brunel Hand** (OpenBionics) - More advanced
   - Download: openbionics.com/brunel
   - Print time: ~30 hours
   - Material: PETG recommended

3. **Raptor Reloaded** (e-NABLE) - Simplest
   - Download: hub.e-nable.org
   - Print time: ~15 hours
   - Material: PLA

### Print Settings

```
Layer Height: 0.2mm
Infill: 20-30%
Supports: Yes (for fingers)
Material: PETG (recommended) or PLA
Nozzle: 0.4mm
```

### Post-Processing

1. Remove supports carefully
2. Sand contact surfaces (220-400 grit)
3. Test fit all joints before assembly
4. Apply light lubricant to pivot points

## Step 2: Assemble the Hand Mechanism

### For Servo-Based Hands (Brunel-style)

1. Insert servos into finger housings
2. Attach servo horns with screws
3. Connect servo wires (note positions)
4. Test each servo moves freely

### For Cable-Based Hands (Ada/Raptor-style)

1. Thread fishing line through finger channels
2. Create loops at fingertips (secure with knots + glue)
3. Route cables to central point
4. Connect to servo or linear actuator

### Wiring Diagram

```
                    ┌─────────────┐
                    │   ESP32-S3  │
                    └─────────────┘
                      │ │ │ │ │
         ┌────────────┼─┼─┼─┼─┼────────────┐
         │            │ │ │ │ │            │
     [Servo 1]    [2] [3] [4] [5]     [EMG Module]
      (Thumb)   (Index)(Mid)(Ring)(Pinky)    ↓
                                        [Electrodes]
```

## Step 3: Build the EMG Electronics

### PCB Assembly

1. Solder ESP32-S3 socket to prototype board
2. Add AD8232 modules (2 channels)
3. Connect power rails (3.3V, GND)
4. Wire 3.5mm electrode jacks
5. Add battery connector and switch

### Wiring Connections

```c
// ESP32-S3 Pin Assignments
#define EMG_CH1_PIN   34   // Analog input channel 1
#define EMG_CH2_PIN   35   // Analog input channel 2
#define SERVO_1       13   // Thumb
#define SERVO_2       12   // Index
#define SERVO_3       14   // Middle
#define SERVO_4       27   // Ring
#define SERVO_5       26   // Pinky
```

### AD8232 Connection

| AD8232 Pin | ESP32 Pin |
|------------|-----------|
| 3.3V | 3.3V |
| GND | GND |
| OUTPUT | GPIO34 (CH1) or GPIO35 (CH2) |
| LO+ | Not connected (optional) |
| LO- | Not connected (optional) |

## Step 4: Electrode Placement

### Forearm Muscle Groups

```
        DORSAL (back of arm)
    ┌─────────────────────────┐
    │    [E1] Extensors       │
    │     ○────○────○         │
    │                         │
    ├─────────────────────────┤
    │    [E2] Flexors         │
    │     ○────○────○         │
    │                         │
    └─────────────────────────┘
        VENTRAL (inner arm)
```

### Electrode Positions

**Channel 1 (Extensors)**
- Active: 3cm distal to lateral epicondyle
- Reference: 4cm distal to active
- Ground: Bony prominence (elbow)

**Channel 2 (Flexors)**
- Active: 3cm distal to medial epicondyle
- Reference: 4cm distal to active
- Ground: Shared with Channel 1

### Application Tips

1. Clean skin with alcohol wipe
2. Let skin dry completely
3. Apply electrode gel if using dry electrodes
4. Secure electrodes with medical tape
5. Avoid hair (shave if necessary)
6. Check impedance < 10kΩ

## Step 5: Upload Firmware

### Install Arduino IDE

1. Download from arduino.cc
2. Add ESP32 board support:
   - File → Preferences → Additional Boards Manager URLs
   - Add: `https://dl.espressif.com/dl/package_esp32_index.json`
3. Tools → Board → Boards Manager → ESP32

### Upload WIA Firmware

1. Open `hardware/firmware/esp32/wia_myoelectric/main.cpp`
2. Select Board: "ESP32-S3 Dev Module"
3. Select Port: (your ESP32 port)
4. Click Upload

### Test EMG Signal

1. Open Serial Monitor (115200 baud)
2. You should see raw EMG values
3. Flex muscles - values should increase
4. Relax - values return to baseline

## Step 6: Calibration

### Run Calibration Routine

1. Open WIA Mobile App (or Serial Monitor)
2. Select "Calibrate"
3. Follow prompts:
   - Rest (10 seconds)
   - Hand Open (5 repetitions)
   - Hand Close (5 repetitions)
   - Pinch (5 repetitions)
4. Save calibration profile

### Adjust Sensitivity

If gestures aren't detected reliably:
- Increase EMG gain in firmware
- Reposition electrodes
- Clean electrode contacts
- Check for 50/60Hz interference

## Step 7: Final Assembly

### Mount Electronics

1. Place ESP32 in enclosure
2. Secure battery with foam tape
3. Route electrode cables
4. Add strain relief to cables

### Attach to Socket

1. Measure residual limb
2. Create custom socket (or use commercial)
3. Mount hand to socket
4. Add padding for comfort
5. Test fit and adjust

### Safety Checklist

- [ ] All electrical connections secure
- [ ] No exposed wires
- [ ] Battery protected from shorts
- [ ] Emergency release mechanism works
- [ ] Grip force within safe limits
- [ ] No pinch points

## Troubleshooting

### No EMG Signal
- Check electrode contact
- Verify AD8232 wiring
- Test with multimeter

### Erratic Servo Movement
- Add capacitors to power rails
- Check for loose connections
- Reduce servo speed

### Short Battery Life
- Reduce BLE advertising interval
- Use low-power mode when idle
- Check for current leaks

### Gesture Misclassification
- Recalibrate
- Check electrode placement
- Reduce movement during signals

## Next Steps

1. Practice basic grip patterns
2. Gradually increase gesture complexity
3. Join the WIA community for support
4. Consider custom socket fitting with prosthetist

## Resources

- WIA GitHub: github.com/wia-standards
- OpenBionics: openbionics.com
- e-NABLE: enablingthefuture.org
- EMG Tutorial: advancertechnologies.com

---

**홍익인간 (弘益人間)**: $75,000 → $100
*Broadly benefiting humanity through accessible technology*
