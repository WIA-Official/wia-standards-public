# WIA-SEMI-012: Calibration Procedures

## Accelerometer Calibration

### Six-Position Method
1. Place sensor X-axis up → Record [0, 0, +1g]
2. Place sensor X-axis down → Record [0, 0, -1g]
3. Place sensor Y-axis up → Record [0, +1g, 0]
4. Place sensor Y-axis down → Record [0, -1g, 0]
5. Place sensor Z-axis up → Record [+1g, 0, 0]
6. Place sensor Z-axis down → Record [-1g, 0, 0]

### Calibration Matrix
```
a_calibrated = S × (a_raw - b)
Where S is 3×3 scale matrix, b is bias vector
```

## Magnetometer Calibration

### Hard-Iron and Soft-Iron Correction
- Rotate device in figure-8 pattern
- Collect 100+ data points
- Fit ellipsoid: Center = hard-iron offset
- Transform to sphere: Soft-iron correction matrix

## Temperature Compensation

- Measure sensor output across temperature range
- Fit polynomial: Output(T) = a₀ + a₁×T + a₂×T²
- Store coefficients in EEPROM
- Apply correction in firmware

---
© 2025 SmileStory Inc. / WIA
