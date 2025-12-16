# WIA Smart Wheelchair CAN Gateway Firmware

CAN 버스와 ROS2 간의 브릿지 역할을 하는 펌웨어입니다.

## Supported Platforms

| Platform | MCU | CAN Interface |
|----------|-----|---------------|
| STM32F4 | STM32F405/407 | Native bxCAN |
| ESP32 | ESP32-WROOM | MCP2515 (SPI) |
| Teensy 4.0 | IMXRT1062 | Native FlexCAN |

## Building

### Requirements
- [PlatformIO](https://platformio.org/)

### Build Commands

```bash
# Build for STM32F4 (default)
pio run -e stm32f4

# Build for ESP32
pio run -e esp32

# Build for Teensy 4.0
pio run -e teensy40

# Upload
pio run -e stm32f4 --target upload
```

## Serial Protocol

게이트웨이는 USB/Serial을 통해 호스트와 통신합니다.

### Packet Format

```
[START][LEN][CMD][ID_H][ID_L][DLC][DATA...][CHECKSUM][END]
```

| Field | Size | Description |
|-------|------|-------------|
| START | 1 | 0xAA |
| LEN | 1 | Payload length (CMD to DATA) |
| CMD | 1 | Command code |
| ID_H | 1 | CAN ID high byte |
| ID_L | 1 | CAN ID low byte |
| DLC | 1 | Data length (0-8) |
| DATA | 0-8 | CAN data |
| CHECKSUM | 1 | XOR of payload |
| END | 1 | 0x55 |

### Commands

| Code | Command | Description |
|------|---------|-------------|
| 0x01 | CAN_SEND | Host → Gateway: Send CAN message |
| 0x02 | CAN_RECV | Gateway → Host: Received CAN message |
| 0x10 | HEARTBEAT | Gateway heartbeat |
| 0x20 | SET_MODE | Set gateway mode |
| 0x21 | GET_STATUS | Get gateway status |
| 0xF0 | ERROR | Error response |

## Hardware Connection

### STM32F4 (CAN1)

| Pin | Function |
|-----|----------|
| PB8 | CAN1_RX |
| PB9 | CAN1_TX |

### ESP32 + MCP2515

| ESP32 | MCP2515 |
|-------|---------|
| GPIO5 | CS |
| GPIO18 | SCK |
| GPIO19 | MISO |
| GPIO23 | MOSI |
| GPIO21 | INT |

### Teensy 4.0

| Pin | Function |
|-----|----------|
| 22 | CAN1_TX |
| 23 | CAN1_RX |

## License

Apache-2.0
