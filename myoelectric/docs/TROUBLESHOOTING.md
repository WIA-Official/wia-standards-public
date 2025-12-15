# WIA Myoelectric Troubleshooting Guide

Quick reference for diagnosing and fixing common issues with
WIA EMG prosthetic systems.

## Quick Diagnosis Flowchart

```
Problem: System not working
           │
           ▼
    ┌──────────────────┐
    │ Power LED on?    │
    └──────────────────┘
           │
    No ────┴──── Yes
    │              │
    ▼              ▼
 Check         ┌──────────────────┐
 battery       │ BLE connected?   │
               └──────────────────┘
                      │
               No ────┴──── Yes
               │              │
               ▼              ▼
            Check         ┌──────────────────┐
            pairing       │ EMG signal OK?   │
                          └──────────────────┘
                                 │
                          No ────┴──── Yes
                          │              │
                          ▼              ▼
                       Check         ┌──────────────────┐
                       electrodes    │ Gestures work?   │
                                     └──────────────────┘
                                            │
                                     No ────┴──── Yes
                                     │              │
                                     ▼              ▼
                                  Recalibrate    All OK!
```

## Power Issues

### System won't turn on

**Symptoms**: No LED, no response

| Check | Solution |
|-------|----------|
| Battery depleted | Charge for 2+ hours |
| Switch broken | Test with multimeter, replace |
| Fuse blown | Check/replace fuse |
| Loose connection | Reseat battery connector |

### Short battery life

**Expected**: 8-12 hours typical use

| Cause | Solution |
|-------|----------|
| Old battery | Replace LiPo cell |
| Constant BLE advertising | Reduce advertising interval |
| Servos always active | Enable sleep mode |
| Short circuit | Check for exposed wires |

### Battery not charging

| Check | Solution |
|-------|----------|
| USB cable | Try different cable |
| TP4056 LED | Should be red while charging |
| Battery connectors | Clean/tighten connections |
| Battery damage | Replace if swollen |

## Bluetooth Issues

### Device not found

**When scanning, WIA device doesn't appear**

1. Ensure device is powered on
2. Check if already connected to another device
3. Restart Bluetooth on phone
4. Reset ESP32 (press EN button)
5. Verify firmware has BLE enabled

### Connection drops frequently

| Cause | Solution |
|-------|----------|
| Distance too far | Stay within 5m |
| Interference | Move away from WiFi routers |
| Low battery | Charge device |
| Firmware bug | Update to latest version |

### Paired but no data

1. Check characteristic UUIDs match
2. Verify notifications are enabled
3. Restart app and reconnect
4. Check Serial Monitor for errors

## EMG Signal Issues

### No signal (flat line)

```
Symptom: EMG value constant, no change when flexing
```

| Check | Solution |
|-------|----------|
| Electrodes connected | Verify all 3 electrodes attached |
| AD8232 powered | Check 3.3V at VCC pin |
| Correct pins | Verify OUTPUT connected to ADC |
| Electrode contact | Reapply with fresh gel |

### Very noisy signal

```
Symptom: Signal jumps randomly, hard to see muscle activity
```

| Cause | Solution |
|-------|----------|
| 50/60Hz interference | Enable notch filter |
| Poor electrode contact | Clean skin, fresh electrodes |
| Long electrode cables | Shorten or shield cables |
| Ground not connected | Verify ground electrode |
| Nearby electronics | Move away from power supplies |

### Signal saturated (stuck at max/min)

```
Symptom: Value stuck at 1023 or 0
```

| Cause | Solution |
|-------|----------|
| Gain too high | Reduce AD8232 gain |
| DC offset | Check electrode connections |
| Short circuit | Inspect wiring |

### Weak signal

```
Symptom: Very small changes when flexing
```

| Cause | Solution |
|-------|----------|
| Gain too low | Increase amplifier gain |
| Wrong muscle | Reposition electrodes |
| Dry electrodes | Use conductive gel |
| Thick skin/hair | Prepare skin properly |

## Gesture Recognition Issues

### Wrong gestures detected

| Cause | Solution |
|-------|----------|
| Poor calibration | Recalibrate all gestures |
| Similar patterns | Use more distinct gestures |
| Electrode moved | Reposition and recalibrate |
| Model overfitting | Collect more varied samples |

### Gestures not detected

| Cause | Solution |
|-------|----------|
| Threshold too high | Lower confidence threshold |
| Weak contraction | Practice stronger activation |
| Wrong classifier | Try different algorithm |

### Delayed response

**Target**: < 200ms from intention to action

| Cause | Solution |
|-------|----------|
| Large window size | Reduce to 150-200ms |
| High overlap | Reduce to 50% |
| Slow classifier | Use LDA instead of CNN |
| BLE latency | Use connection interval 7.5ms |

### Frequent false activations

| Cause | Solution |
|-------|----------|
| Low threshold | Increase confidence threshold |
| Noise misclassified | Add noise to training data |
| Motion artifacts | Secure electrodes firmly |

## Servo/Motor Issues

### Servo not moving

| Check | Solution |
|-------|----------|
| Power to servo | Measure 5V at servo VCC |
| Signal present | Check PWM with oscilloscope |
| Servo damaged | Test with known-good servo |
| Wrong pin | Verify pin assignment |

### Servo jittering

| Cause | Solution |
|-------|----------|
| Weak power supply | Add capacitor (470µF+) |
| Signal noise | Filter PWM signal |
| Mechanical binding | Lubricate joints |
| Load too high | Use stronger servo |

### Servo overheating

| Cause | Solution |
|-------|----------|
| Continuous stall | Add current limiting |
| Mechanical jam | Clear obstruction |
| Overcurrent | Use appropriate servo |
| No rest periods | Implement duty cycle limits |

### Grip too weak/strong

| Adjustment | Method |
|------------|--------|
| Increase force | Higher position value |
| Decrease force | Lower position value |
| Speed control | Adjust movement speed |
| Proportional | Enable EMG proportional mode |

## Mechanical Issues

### Fingers don't close fully

| Cause | Solution |
|-------|----------|
| Cable slack | Tighten cable tension |
| Servo range | Extend servo angle range |
| Obstruction | Remove debris from joints |
| String stretched | Replace fishing line |

### Fingers don't open

| Cause | Solution |
|-------|----------|
| Elastic broken | Replace elastic cord |
| Joint friction | Lubricate with silicone |
| Servo stuck | Check servo operation |

### Hand falls off socket

| Cause | Solution |
|-------|----------|
| Socket too loose | Add padding |
| Weak attachment | Reinforce mounting |
| Sweat | Use moisture-wicking liner |

## Software/Firmware Issues

### Firmware won't upload

| Error | Solution |
|-------|----------|
| Port not found | Install USB drivers |
| Upload timeout | Hold BOOT, press EN |
| Wrong board | Select ESP32-S3 |

### Crashes/reboots

| Cause | Solution |
|-------|----------|
| Stack overflow | Increase stack size |
| Memory leak | Check for malloc without free |
| Brownout | Improve power supply |
| Watchdog | Feed watchdog timer |

### App crashes

1. Update to latest version
2. Clear app cache
3. Reinstall app
4. Check phone compatibility

## Environmental Issues

### Problems in cold weather

| Issue | Solution |
|-------|----------|
| Stiff servos | Warm up before use |
| Poor electrode contact | Use warmer gel |
| Battery capacity drop | Keep battery warm |

### Problems in hot/humid weather

| Issue | Solution |
|-------|----------|
| Excessive sweating | Use antiperspirant |
| Electrode sliding | Secure with tape |
| Electronics overheating | Add ventilation |

## Error Codes

| Code | Meaning | Action |
|------|---------|--------|
| E01 | Battery low | Charge immediately |
| E02 | EMG signal lost | Check electrodes |
| E03 | BLE disconnected | Reconnect |
| E04 | Calibration invalid | Recalibrate |
| E05 | Servo timeout | Check servo wiring |
| E06 | Overtemperature | Let cool down |
| E07 | Classifier error | Reset device |
| E08 | Memory full | Clear logs |

## Maintenance Schedule

### Daily
- [ ] Check electrode adhesion
- [ ] Verify battery level
- [ ] Test basic grip function

### Weekly
- [ ] Clean electrodes/contacts
- [ ] Check cable connections
- [ ] Lubricate finger joints
- [ ] Full calibration

### Monthly
- [ ] Deep clean prosthetic
- [ ] Inspect all wiring
- [ ] Replace electrodes
- [ ] Check battery health
- [ ] Update firmware

## Getting Help

### Self-Diagnosis

Run self-test:
```
Command: self_test
Output: Shows all system status
```

### Community Support

- GitHub Issues: github.com/wia-standards/issues
- Discord: WIA Prosthetics Community
- Forum: openbionics.org/community

### Information to Include

When asking for help, provide:
1. Hardware version
2. Firmware version
3. Error messages
4. Steps to reproduce
5. Photos if relevant

---

*Most problems have simple solutions.
Check connections first, calibrate second, ask for help third.*
