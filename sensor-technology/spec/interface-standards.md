# WIA-SEMI-012: Interface Standards

## I2C Communication

### Standard Register Map
```
0x00: WHO_AM_I (Read-only, device ID)
0x01-0x0F: Configuration registers
0x10-0x1F: Control registers
0x20-0x3F: Data output registers
0x40-0x5F: FIFO access
0x60-0x7F: Interrupt configuration
```

### Power Modes
- Normal: Full performance
- Low-power: Reduced ODR, lower current
- Sleep: Wake on interrupt
- Shutdown: Complete power-off

## SPI Communication

### Timing Requirements
- Setup time: Minimum 10 ns
- Hold time: Minimum 10 ns
- Clock frequency: Up to 10 MHz
- Mode: 0 or 3

---
© 2025 SmileStory Inc. / WIA
