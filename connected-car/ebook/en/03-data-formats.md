# Chapter 3: Connected Car Data Formats

## Vehicle Data Schemas, Ontologies, and Message Structures

This chapter defines the comprehensive data formats used for connected car communication, ensuring interoperability across manufacturers, platforms, and geographic regions.

---

## Vehicle Signal Specification (VSS)

### COVESA VSS Standard

```typescript
// WIA Connected Car Data Format Implementation
// Based on COVESA Vehicle Signal Specification (VSS) 4.0

/**
 * Vehicle Signal Specification Tree Structure
 * Hierarchical organization of vehicle data signals
 */
interface VSSNode {
  type: VSSNodeType;
  description: string;
  datatype?: VSSDataType;
  unit?: string;
  min?: number;
  max?: number;
  allowed?: string[];
  default?: any;
  deprecation?: string;
  comment?: string;
  children?: Record<string, VSSNode>;
}

type VSSNodeType = "branch" | "sensor" | "actuator" | "attribute";

type VSSDataType =
  | "uint8" | "int8"
  | "uint16" | "int16"
  | "uint32" | "int32"
  | "uint64" | "int64"
  | "float" | "double"
  | "boolean" | "string"
  | "uint8[]" | "int8[]"
  | "uint16[]" | "int16[]"
  | "uint32[]" | "int32[]"
  | "uint64[]" | "int64[]"
  | "float[]" | "double[]"
  | "boolean[]" | "string[]";

/**
 * Core VSS Tree Definition for Connected Car
 */
const vehicleSignalTree: Record<string, VSSNode> = {
  Vehicle: {
    type: "branch",
    description: "High-level vehicle data",
    children: {
      VehicleIdentification: {
        type: "branch",
        description: "Vehicle identification attributes",
        children: {
          VIN: {
            type: "attribute",
            description: "Vehicle Identification Number as defined by ISO 3779",
            datatype: "string"
          },
          WMI: {
            type: "attribute",
            description: "World Manufacturer Identifier (VIN positions 1-3)",
            datatype: "string"
          },
          Brand: {
            type: "attribute",
            description: "Vehicle brand or make",
            datatype: "string"
          },
          Model: {
            type: "attribute",
            description: "Vehicle model",
            datatype: "string"
          },
          Year: {
            type: "attribute",
            description: "Model year",
            datatype: "uint16",
            min: 1900,
            max: 2100
          },
          BodyType: {
            type: "attribute",
            description: "Vehicle body type",
            datatype: "string",
            allowed: ["SEDAN", "SUV", "HATCHBACK", "WAGON", "COUPE", "CONVERTIBLE", "PICKUP", "VAN", "TRUCK"]
          }
        }
      },
      CurrentLocation: {
        type: "branch",
        description: "Current vehicle location",
        children: {
          Latitude: {
            type: "sensor",
            description: "Current latitude in WGS84 decimal degrees",
            datatype: "double",
            min: -90,
            max: 90,
            unit: "degrees"
          },
          Longitude: {
            type: "sensor",
            description: "Current longitude in WGS84 decimal degrees",
            datatype: "double",
            min: -180,
            max: 180,
            unit: "degrees"
          },
          Altitude: {
            type: "sensor",
            description: "Current altitude in meters above sea level",
            datatype: "double",
            unit: "m"
          },
          Heading: {
            type: "sensor",
            description: "Current heading in degrees from true north",
            datatype: "double",
            min: 0,
            max: 360,
            unit: "degrees"
          },
          HorizontalAccuracy: {
            type: "sensor",
            description: "Horizontal position accuracy in meters",
            datatype: "double",
            unit: "m"
          },
          VerticalAccuracy: {
            type: "sensor",
            description: "Vertical position accuracy in meters",
            datatype: "double",
            unit: "m"
          },
          GNSSReceiver: {
            type: "branch",
            description: "GNSS receiver information",
            children: {
              FixType: {
                type: "sensor",
                description: "Current GNSS fix type",
                datatype: "string",
                allowed: ["NONE", "2D", "3D", "RTK_FLOAT", "RTK_FIXED"]
              },
              SatellitesUsed: {
                type: "sensor",
                description: "Number of satellites used in fix",
                datatype: "uint8"
              }
            }
          },
          Timestamp: {
            type: "sensor",
            description: "Timestamp of location reading in ISO8601 format",
            datatype: "string"
          }
        }
      },
      Speed: {
        type: "sensor",
        description: "Vehicle speed in km/h",
        datatype: "float",
        min: 0,
        max: 500,
        unit: "km/h"
      },
      TraveledDistance: {
        type: "sensor",
        description: "Accumulated trip distance in km",
        datatype: "float",
        unit: "km"
      },
      TraveledDistanceTotal: {
        type: "sensor",
        description: "Total accumulated distance (odometer) in km",
        datatype: "float",
        unit: "km"
      },
      Acceleration: {
        type: "branch",
        description: "Acceleration in multiple axes",
        children: {
          Longitudinal: {
            type: "sensor",
            description: "Longitudinal acceleration (positive = forward)",
            datatype: "float",
            unit: "m/s^2"
          },
          Lateral: {
            type: "sensor",
            description: "Lateral acceleration (positive = right)",
            datatype: "float",
            unit: "m/s^2"
          },
          Vertical: {
            type: "sensor",
            description: "Vertical acceleration (positive = up)",
            datatype: "float",
            unit: "m/s^2"
          }
        }
      },
      AngularVelocity: {
        type: "branch",
        description: "Angular velocity (rotation rates)",
        children: {
          Pitch: {
            type: "sensor",
            description: "Pitch rate (positive = nose up)",
            datatype: "float",
            unit: "degrees/s"
          },
          Roll: {
            type: "sensor",
            description: "Roll rate (positive = right side down)",
            datatype: "float",
            unit: "degrees/s"
          },
          Yaw: {
            type: "sensor",
            description: "Yaw rate (positive = clockwise from above)",
            datatype: "float",
            unit: "degrees/s"
          }
        }
      }
    }
  },
  Powertrain: {
    type: "branch",
    description: "Powertrain systems",
    children: {
      Type: {
        type: "attribute",
        description: "Powertrain type",
        datatype: "string",
        allowed: ["ICE", "HYBRID", "PHEV", "BEV", "FCEV"]
      },
      CombustionEngine: {
        type: "branch",
        description: "Internal combustion engine signals",
        children: {
          IsRunning: {
            type: "sensor",
            description: "Engine running status",
            datatype: "boolean"
          },
          Speed: {
            type: "sensor",
            description: "Engine speed in RPM",
            datatype: "uint16",
            unit: "rpm",
            max: 20000
          },
          EngineLoad: {
            type: "sensor",
            description: "Engine load percentage",
            datatype: "uint8",
            unit: "percent",
            min: 0,
            max: 100
          },
          TorqueOutput: {
            type: "sensor",
            description: "Current engine torque output",
            datatype: "float",
            unit: "Nm"
          },
          CoolantTemperature: {
            type: "sensor",
            description: "Engine coolant temperature",
            datatype: "int16",
            unit: "celsius"
          },
          OilTemperature: {
            type: "sensor",
            description: "Engine oil temperature",
            datatype: "int16",
            unit: "celsius"
          },
          OilPressure: {
            type: "sensor",
            description: "Engine oil pressure",
            datatype: "uint16",
            unit: "kPa"
          }
        }
      },
      ElectricMotor: {
        type: "branch",
        description: "Electric motor signals",
        children: {
          Power: {
            type: "sensor",
            description: "Current electric motor power (positive = driving)",
            datatype: "int16",
            unit: "kW"
          },
          Torque: {
            type: "sensor",
            description: "Current motor torque",
            datatype: "int16",
            unit: "Nm"
          },
          Temperature: {
            type: "sensor",
            description: "Motor temperature",
            datatype: "int16",
            unit: "celsius"
          },
          Speed: {
            type: "sensor",
            description: "Motor rotational speed",
            datatype: "int32",
            unit: "rpm"
          }
        }
      },
      TractionBattery: {
        type: "branch",
        description: "High-voltage traction battery",
        children: {
          StateOfCharge: {
            type: "branch",
            description: "Battery state of charge",
            children: {
              Current: {
                type: "sensor",
                description: "Current state of charge",
                datatype: "float",
                unit: "percent",
                min: 0,
                max: 100
              },
              Displayed: {
                type: "sensor",
                description: "Displayed state of charge (may differ from actual)",
                datatype: "float",
                unit: "percent",
                min: 0,
                max: 100
              }
            }
          },
          StateOfHealth: {
            type: "sensor",
            description: "Battery state of health",
            datatype: "float",
            unit: "percent",
            min: 0,
            max: 100
          },
          NominalCapacity: {
            type: "attribute",
            description: "Nominal battery capacity",
            datatype: "float",
            unit: "kWh"
          },
          CurrentCapacity: {
            type: "sensor",
            description: "Current usable battery capacity",
            datatype: "float",
            unit: "kWh"
          },
          CurrentVoltage: {
            type: "sensor",
            description: "Current battery voltage",
            datatype: "float",
            unit: "V"
          },
          CurrentCurrent: {
            type: "sensor",
            description: "Current battery current (positive = discharging)",
            datatype: "float",
            unit: "A"
          },
          CurrentPower: {
            type: "sensor",
            description: "Current battery power (positive = discharging)",
            datatype: "float",
            unit: "kW"
          },
          Temperature: {
            type: "branch",
            description: "Battery temperature readings",
            children: {
              Average: {
                type: "sensor",
                description: "Average battery temperature",
                datatype: "float",
                unit: "celsius"
              },
              Min: {
                type: "sensor",
                description: "Minimum cell temperature",
                datatype: "float",
                unit: "celsius"
              },
              Max: {
                type: "sensor",
                description: "Maximum cell temperature",
                datatype: "float",
                unit: "celsius"
              }
            }
          },
          Range: {
            type: "sensor",
            description: "Estimated remaining range",
            datatype: "uint32",
            unit: "m"
          },
          Charging: {
            type: "branch",
            description: "Charging status and parameters",
            children: {
              IsCharging: {
                type: "sensor",
                description: "Charging status",
                datatype: "boolean"
              },
              ChargeType: {
                type: "sensor",
                description: "Current charge connection type",
                datatype: "string",
                allowed: ["AC_LEVEL_1", "AC_LEVEL_2", "DC_FAST", "WIRELESS", "NOT_CONNECTED"]
              },
              ChargePower: {
                type: "sensor",
                description: "Current charging power",
                datatype: "float",
                unit: "kW"
              },
              ChargeLimit: {
                type: "actuator",
                description: "Target state of charge limit",
                datatype: "uint8",
                unit: "percent",
                min: 50,
                max: 100
              },
              TimeToComplete: {
                type: "sensor",
                description: "Estimated time to reach charge limit",
                datatype: "uint32",
                unit: "s"
              }
            }
          }
        }
      },
      Transmission: {
        type: "branch",
        description: "Transmission system",
        children: {
          Type: {
            type: "attribute",
            description: "Transmission type",
            datatype: "string",
            allowed: ["MANUAL", "AUTOMATIC", "DCT", "CVT", "DIRECT_DRIVE"]
          },
          CurrentGear: {
            type: "sensor",
            description: "Current gear (0=Neutral, -1=Reverse)",
            datatype: "int8"
          },
          GearCount: {
            type: "attribute",
            description: "Number of forward gears",
            datatype: "uint8"
          },
          DriveType: {
            type: "attribute",
            description: "Drive configuration",
            datatype: "string",
            allowed: ["FWD", "RWD", "AWD", "4WD"]
          },
          SelectedGear: {
            type: "sensor",
            description: "Selected gear position (PRND)",
            datatype: "string",
            allowed: ["PARK", "REVERSE", "NEUTRAL", "DRIVE", "SPORT", "LOW", "MANUAL"]
          }
        }
      },
      FuelSystem: {
        type: "branch",
        description: "Fuel system (ICE/Hybrid vehicles)",
        children: {
          Level: {
            type: "sensor",
            description: "Fuel level percentage",
            datatype: "uint8",
            unit: "percent",
            min: 0,
            max: 100
          },
          TankCapacity: {
            type: "attribute",
            description: "Fuel tank capacity",
            datatype: "float",
            unit: "l"
          },
          Range: {
            type: "sensor",
            description: "Estimated remaining range on fuel",
            datatype: "uint32",
            unit: "m"
          },
          AverageConsumption: {
            type: "sensor",
            description: "Average fuel consumption",
            datatype: "float",
            unit: "l/100km"
          },
          InstantConsumption: {
            type: "sensor",
            description: "Instantaneous fuel consumption",
            datatype: "float",
            unit: "l/100km"
          }
        }
      }
    }
  }
};
```

---

## Telematics Message Formats

### WIA Telematics Protocol (WTP)

```typescript
/**
 * WIA Telematics Protocol (WTP) Message Structure
 * Efficient binary format for vehicle-to-cloud communication
 */
interface WTPMessage {
  header: WTPHeader;
  payload: WTPPayload;
  checksum: number;  // CRC-32
}

interface WTPHeader {
  version: number;           // Protocol version (1 byte)
  messageType: WTPMessageType;  // Message type (1 byte)
  vehicleId: Buffer;         // Vehicle identifier (16 bytes, UUID)
  sequenceNumber: number;    // Message sequence (4 bytes)
  timestamp: bigint;         // Unix timestamp in microseconds (8 bytes)
  payloadLength: number;     // Payload length (2 bytes)
  flags: WTPFlags;           // Message flags (1 byte)
  reserved: Buffer;          // Reserved for future use (3 bytes)
}

enum WTPMessageType {
  HEARTBEAT = 0x01,
  TELEMETRY = 0x02,
  LOCATION = 0x03,
  EVENT = 0x04,
  DIAGNOSTIC = 0x05,
  COMMAND = 0x06,
  COMMAND_RESPONSE = 0x07,
  OTA_REQUEST = 0x08,
  OTA_STATUS = 0x09,
  V2X_MESSAGE = 0x0A,
  BULK_UPLOAD = 0x0B,
  STREAM_START = 0x0C,
  STREAM_DATA = 0x0D,
  STREAM_END = 0x0E
}

interface WTPFlags {
  encrypted: boolean;        // Bit 0: Payload encrypted
  compressed: boolean;       // Bit 1: Payload compressed
  requiresAck: boolean;      // Bit 2: Requires acknowledgment
  priority: WTPPriority;     // Bits 3-4: Message priority
  fragmented: boolean;       // Bit 5: Part of fragmented message
  lastFragment: boolean;     // Bit 6: Last fragment
  reserved: boolean;         // Bit 7: Reserved
}

enum WTPPriority {
  LOW = 0,
  NORMAL = 1,
  HIGH = 2,
  CRITICAL = 3
}

type WTPPayload =
  | HeartbeatPayload
  | TelemetryPayload
  | LocationPayload
  | EventPayload
  | DiagnosticPayload
  | CommandPayload
  | CommandResponsePayload
  | V2XMessagePayload;

/**
 * Telemetry Payload Structure
 */
interface TelemetryPayload {
  signalCount: number;
  signals: TelemetrySignal[];
}

interface TelemetrySignal {
  signalId: number;          // VSS signal identifier
  timestamp: number;         // Delta from header timestamp (ms)
  quality: SignalQuality;
  value: SignalValue;
}

enum SignalQuality {
  GOOD = 0,
  UNCERTAIN = 1,
  BAD = 2,
  NOT_AVAILABLE = 3
}

type SignalValue =
  | { type: "bool"; value: boolean }
  | { type: "int8"; value: number }
  | { type: "int16"; value: number }
  | { type: "int32"; value: number }
  | { type: "int64"; value: bigint }
  | { type: "uint8"; value: number }
  | { type: "uint16"; value: number }
  | { type: "uint32"; value: number }
  | { type: "uint64"; value: bigint }
  | { type: "float32"; value: number }
  | { type: "float64"; value: number }
  | { type: "string"; value: string }
  | { type: "bytes"; value: Buffer };

/**
 * WTP Message Encoder/Decoder
 */
class WTPCodec {
  private static HEADER_SIZE = 36;
  private static CHECKSUM_SIZE = 4;

  /**
   * Encode WTP message to binary buffer
   */
  static encode(message: WTPMessage): Buffer {
    const headerBuffer = this.encodeHeader(message.header);
    const payloadBuffer = this.encodePayload(message.payload, message.header.messageType);

    // Apply compression if flagged
    const processedPayload = message.header.flags.compressed
      ? this.compress(payloadBuffer)
      : payloadBuffer;

    // Apply encryption if flagged
    const finalPayload = message.header.flags.encrypted
      ? this.encrypt(processedPayload)
      : processedPayload;

    // Calculate checksum
    const checksumBuffer = Buffer.alloc(this.CHECKSUM_SIZE);
    const checksum = this.calculateCRC32(Buffer.concat([headerBuffer, finalPayload]));
    checksumBuffer.writeUInt32BE(checksum, 0);

    return Buffer.concat([headerBuffer, finalPayload, checksumBuffer]);
  }

  /**
   * Decode WTP message from binary buffer
   */
  static decode(buffer: Buffer): WTPMessage {
    // Verify minimum size
    if (buffer.length < this.HEADER_SIZE + this.CHECKSUM_SIZE) {
      throw new Error("Buffer too small for WTP message");
    }

    // Extract and verify checksum
    const messageWithoutChecksum = buffer.subarray(0, buffer.length - this.CHECKSUM_SIZE);
    const receivedChecksum = buffer.readUInt32BE(buffer.length - this.CHECKSUM_SIZE);
    const calculatedChecksum = this.calculateCRC32(messageWithoutChecksum);

    if (receivedChecksum !== calculatedChecksum) {
      throw new Error("Checksum verification failed");
    }

    // Decode header
    const header = this.decodeHeader(buffer.subarray(0, this.HEADER_SIZE));

    // Extract payload
    let payloadBuffer = buffer.subarray(
      this.HEADER_SIZE,
      buffer.length - this.CHECKSUM_SIZE
    );

    // Decrypt if encrypted
    if (header.flags.encrypted) {
      payloadBuffer = this.decrypt(payloadBuffer);
    }

    // Decompress if compressed
    if (header.flags.compressed) {
      payloadBuffer = this.decompress(payloadBuffer);
    }

    // Decode payload based on message type
    const payload = this.decodePayload(payloadBuffer, header.messageType);

    return {
      header,
      payload,
      checksum: receivedChecksum
    };
  }

  private static encodeHeader(header: WTPHeader): Buffer {
    const buffer = Buffer.alloc(this.HEADER_SIZE);
    let offset = 0;

    buffer.writeUInt8(header.version, offset++);
    buffer.writeUInt8(header.messageType, offset++);
    header.vehicleId.copy(buffer, offset);
    offset += 16;
    buffer.writeUInt32BE(header.sequenceNumber, offset);
    offset += 4;
    buffer.writeBigUInt64BE(header.timestamp, offset);
    offset += 8;
    buffer.writeUInt16BE(header.payloadLength, offset);
    offset += 2;
    buffer.writeUInt8(this.encodeFlags(header.flags), offset++);
    header.reserved.copy(buffer, offset);

    return buffer;
  }

  private static decodeHeader(buffer: Buffer): WTPHeader {
    let offset = 0;

    return {
      version: buffer.readUInt8(offset++),
      messageType: buffer.readUInt8(offset++) as WTPMessageType,
      vehicleId: buffer.subarray(offset, offset += 16),
      sequenceNumber: buffer.readUInt32BE(offset),
      timestamp: buffer.readBigUInt64BE(offset += 4),
      payloadLength: buffer.readUInt16BE(offset += 8),
      flags: this.decodeFlags(buffer.readUInt8(offset += 2)),
      reserved: buffer.subarray(offset + 1, offset + 4)
    };
  }

  private static encodeFlags(flags: WTPFlags): number {
    let byte = 0;
    if (flags.encrypted) byte |= 0x01;
    if (flags.compressed) byte |= 0x02;
    if (flags.requiresAck) byte |= 0x04;
    byte |= (flags.priority & 0x03) << 3;
    if (flags.fragmented) byte |= 0x20;
    if (flags.lastFragment) byte |= 0x40;
    return byte;
  }

  private static decodeFlags(byte: number): WTPFlags {
    return {
      encrypted: (byte & 0x01) !== 0,
      compressed: (byte & 0x02) !== 0,
      requiresAck: (byte & 0x04) !== 0,
      priority: ((byte >> 3) & 0x03) as WTPPriority,
      fragmented: (byte & 0x20) !== 0,
      lastFragment: (byte & 0x40) !== 0,
      reserved: (byte & 0x80) !== 0
    };
  }

  private static encodePayload(payload: WTPPayload, type: WTPMessageType): Buffer {
    switch (type) {
      case WTPMessageType.TELEMETRY:
        return this.encodeTelemetryPayload(payload as TelemetryPayload);
      case WTPMessageType.LOCATION:
        return this.encodeLocationPayload(payload as LocationPayload);
      case WTPMessageType.EVENT:
        return this.encodeEventPayload(payload as EventPayload);
      default:
        throw new Error(`Unsupported message type: ${type}`);
    }
  }

  private static encodeTelemetryPayload(payload: TelemetryPayload): Buffer {
    const buffers: Buffer[] = [];

    // Signal count (2 bytes)
    const countBuffer = Buffer.alloc(2);
    countBuffer.writeUInt16BE(payload.signalCount, 0);
    buffers.push(countBuffer);

    // Each signal
    for (const signal of payload.signals) {
      buffers.push(this.encodeSignal(signal));
    }

    return Buffer.concat(buffers);
  }

  private static encodeSignal(signal: TelemetrySignal): Buffer {
    const buffers: Buffer[] = [];

    // Signal ID (2 bytes)
    const idBuffer = Buffer.alloc(2);
    idBuffer.writeUInt16BE(signal.signalId, 0);
    buffers.push(idBuffer);

    // Timestamp delta (2 bytes)
    const tsBuffer = Buffer.alloc(2);
    tsBuffer.writeUInt16BE(signal.timestamp, 0);
    buffers.push(tsBuffer);

    // Quality (1 byte)
    const qualityBuffer = Buffer.alloc(1);
    qualityBuffer.writeUInt8(signal.quality, 0);
    buffers.push(qualityBuffer);

    // Value (variable)
    buffers.push(this.encodeSignalValue(signal.value));

    return Buffer.concat(buffers);
  }

  private static encodeSignalValue(value: SignalValue): Buffer {
    const typeMap: Record<string, number> = {
      bool: 0x01, int8: 0x02, int16: 0x03, int32: 0x04, int64: 0x05,
      uint8: 0x06, uint16: 0x07, uint32: 0x08, uint64: 0x09,
      float32: 0x0A, float64: 0x0B, string: 0x0C, bytes: 0x0D
    };

    const buffers: Buffer[] = [];

    // Type byte
    const typeBuffer = Buffer.alloc(1);
    typeBuffer.writeUInt8(typeMap[value.type], 0);
    buffers.push(typeBuffer);

    // Value bytes
    switch (value.type) {
      case "bool":
        const boolBuffer = Buffer.alloc(1);
        boolBuffer.writeUInt8(value.value ? 1 : 0, 0);
        buffers.push(boolBuffer);
        break;
      case "int8":
        const i8Buffer = Buffer.alloc(1);
        i8Buffer.writeInt8(value.value, 0);
        buffers.push(i8Buffer);
        break;
      case "int16":
        const i16Buffer = Buffer.alloc(2);
        i16Buffer.writeInt16BE(value.value, 0);
        buffers.push(i16Buffer);
        break;
      case "int32":
        const i32Buffer = Buffer.alloc(4);
        i32Buffer.writeInt32BE(value.value, 0);
        buffers.push(i32Buffer);
        break;
      case "float32":
        const f32Buffer = Buffer.alloc(4);
        f32Buffer.writeFloatBE(value.value, 0);
        buffers.push(f32Buffer);
        break;
      case "float64":
        const f64Buffer = Buffer.alloc(8);
        f64Buffer.writeDoubleBE(value.value, 0);
        buffers.push(f64Buffer);
        break;
      case "string":
        const strBytes = Buffer.from(value.value, "utf8");
        const strLenBuffer = Buffer.alloc(2);
        strLenBuffer.writeUInt16BE(strBytes.length, 0);
        buffers.push(strLenBuffer, strBytes);
        break;
    }

    return Buffer.concat(buffers);
  }

  // Stub methods for compression/encryption/CRC
  private static compress(buffer: Buffer): Buffer { return buffer; }
  private static decompress(buffer: Buffer): Buffer { return buffer; }
  private static encrypt(buffer: Buffer): Buffer { return buffer; }
  private static decrypt(buffer: Buffer): Buffer { return buffer; }
  private static calculateCRC32(buffer: Buffer): number { return 0; }
  private static decodePayload(buffer: Buffer, type: WTPMessageType): WTPPayload {
    return {} as WTPPayload;
  }
  private static encodeLocationPayload(payload: LocationPayload): Buffer { return Buffer.alloc(0); }
  private static encodeEventPayload(payload: EventPayload): Buffer { return Buffer.alloc(0); }
}

interface LocationPayload {
  latitude: number;
  longitude: number;
  altitude: number;
  heading: number;
  speed: number;
  accuracy: number;
}

interface EventPayload {
  eventType: string;
  severity: string;
  data: Record<string, unknown>;
}
```

---

## V2X Message Standards

### SAE J2735 Message Formats

```typescript
/**
 * SAE J2735 Basic Safety Message (BSM)
 * Part 1 - Core data elements transmitted 10 times per second
 */
interface BasicSafetyMessage {
  msgCnt: number;              // Message count (0-127)
  id: TemporaryID;             // Temporary ID (4 bytes)
  secMark: number;             // DSEC (0-65535, ms in minute)
  lat: Latitude;               // Latitude in 1/10 micro-degrees
  long: Longitude;             // Longitude in 1/10 micro-degrees
  elev: Elevation;             // Elevation in decimeters
  accuracy: PositionalAccuracy;
  transmission: TransmissionState;
  speed: Speed;                // Speed in 0.02 m/s units
  heading: Heading;            // Heading in 0.0125 degree units
  angle: SteeringWheelAngle;   // Steering wheel angle
  accelSet: AccelerationSet4Way;
  brakes: BrakeSystemStatus;
  size: VehicleSize;
}

// Type definitions for BSM fields
type TemporaryID = [number, number, number, number];

interface Latitude {
  value: number;  // -900000000 to 900000001 (1/10 micro-degrees)
}

interface Longitude {
  value: number;  // -1799999999 to 1800000001 (1/10 micro-degrees)
}

interface Elevation {
  value: number;  // -4096 to 61439 (decimeters, -409.6m to 6143.9m)
}

interface PositionalAccuracy {
  semiMajor: number;   // Semi-major axis accuracy (0-255, 0.05m units)
  semiMinor: number;   // Semi-minor axis accuracy (0-255, 0.05m units)
  orientation: number; // Orientation of accuracy ellipse (0-65535)
}

enum TransmissionState {
  neutral = 0,
  park = 1,
  forwardGears = 2,
  reverseGears = 3,
  reserved1 = 4,
  reserved2 = 5,
  reserved3 = 6,
  unavailable = 7
}

interface Speed {
  value: number;  // 0-8191 (0.02 m/s units, 0-163.82 m/s)
}

interface Heading {
  value: number;  // 0-28800 (0.0125 degree units, 0-360 degrees)
}

interface SteeringWheelAngle {
  value: number;  // -126 to 127 (1.5 degree units)
}

interface AccelerationSet4Way {
  long: number;   // -2000 to 2001 (0.01 m/s² units)
  lat: number;    // -2000 to 2001 (0.01 m/s² units)
  vert: number;   // -127 to 127 (0.02 g units)
  yaw: number;    // -32767 to 32767 (0.01 deg/s units)
}

interface BrakeSystemStatus {
  wheelBrakes: WheelBrakes;
  traction: TractionControlStatus;
  abs: AntiLockBrakeStatus;
  scs: StabilityControlStatus;
  brakeBoost: BrakeBoostApplied;
  auxBrakes: AuxiliaryBrakeStatus;
}

interface WheelBrakes {
  unavailable: boolean;
  leftFront: boolean;
  leftRear: boolean;
  rightFront: boolean;
  rightRear: boolean;
}

enum TractionControlStatus {
  unavailable = 0,
  off = 1,
  on = 2,
  engaged = 3
}

enum AntiLockBrakeStatus {
  unavailable = 0,
  off = 1,
  on = 2,
  engaged = 3
}

enum StabilityControlStatus {
  unavailable = 0,
  off = 1,
  on = 2,
  engaged = 3
}

enum BrakeBoostApplied {
  unavailable = 0,
  off = 1,
  on = 2
}

enum AuxiliaryBrakeStatus {
  unavailable = 0,
  off = 1,
  on = 2,
  reserved = 3
}

interface VehicleSize {
  width: number;   // 0-1023 (1 cm units)
  length: number;  // 0-4095 (1 cm units)
}

/**
 * BSM Encoder following SAE J2735 ASN.1 structure
 */
class BSMEncoder {
  /**
   * Encode BSM to UPER (Unaligned Packed Encoding Rules)
   */
  static encodeToUPER(bsm: BasicSafetyMessage): Buffer {
    const writer = new BitWriter();

    // Message count (7 bits, 0-127)
    writer.writeBits(bsm.msgCnt, 7);

    // Temporary ID (32 bits)
    for (const byte of bsm.id) {
      writer.writeBits(byte, 8);
    }

    // DSecond (16 bits)
    writer.writeBits(bsm.secMark, 16);

    // Latitude (32 bits, signed)
    writer.writeSignedBits(bsm.lat.value, 32);

    // Longitude (32 bits, signed)
    writer.writeSignedBits(bsm.long.value, 32);

    // Elevation (16 bits)
    writer.writeBits(bsm.elev.value + 4096, 16);  // Offset for encoding

    // Positional accuracy
    writer.writeBits(bsm.accuracy.semiMajor, 8);
    writer.writeBits(bsm.accuracy.semiMinor, 8);
    writer.writeBits(bsm.accuracy.orientation, 16);

    // Transmission state (3 bits)
    writer.writeBits(bsm.transmission, 3);

    // Speed (13 bits)
    writer.writeBits(bsm.speed.value, 13);

    // Heading (16 bits)
    writer.writeBits(bsm.heading.value, 16);

    // Steering wheel angle (8 bits, signed)
    writer.writeSignedBits(bsm.angle.value, 8);

    // Acceleration set
    writer.writeSignedBits(bsm.accelSet.long, 12);
    writer.writeSignedBits(bsm.accelSet.lat, 12);
    writer.writeSignedBits(bsm.accelSet.vert, 8);
    writer.writeSignedBits(bsm.accelSet.yaw, 16);

    // Brake system status
    writer.writeBits(this.encodeWheelBrakes(bsm.brakes.wheelBrakes), 5);
    writer.writeBits(bsm.brakes.traction, 2);
    writer.writeBits(bsm.brakes.abs, 2);
    writer.writeBits(bsm.brakes.scs, 2);
    writer.writeBits(bsm.brakes.brakeBoost, 2);
    writer.writeBits(bsm.brakes.auxBrakes, 2);

    // Vehicle size
    writer.writeBits(bsm.size.width, 10);
    writer.writeBits(bsm.size.length, 12);

    return writer.toBuffer();
  }

  /**
   * Decode BSM from UPER
   */
  static decodeFromUPER(buffer: Buffer): BasicSafetyMessage {
    const reader = new BitReader(buffer);

    return {
      msgCnt: reader.readBits(7),
      id: [reader.readBits(8), reader.readBits(8), reader.readBits(8), reader.readBits(8)],
      secMark: reader.readBits(16),
      lat: { value: reader.readSignedBits(32) },
      long: { value: reader.readSignedBits(32) },
      elev: { value: reader.readBits(16) - 4096 },
      accuracy: {
        semiMajor: reader.readBits(8),
        semiMinor: reader.readBits(8),
        orientation: reader.readBits(16)
      },
      transmission: reader.readBits(3) as TransmissionState,
      speed: { value: reader.readBits(13) },
      heading: { value: reader.readBits(16) },
      angle: { value: reader.readSignedBits(8) },
      accelSet: {
        long: reader.readSignedBits(12),
        lat: reader.readSignedBits(12),
        vert: reader.readSignedBits(8),
        yaw: reader.readSignedBits(16)
      },
      brakes: {
        wheelBrakes: this.decodeWheelBrakes(reader.readBits(5)),
        traction: reader.readBits(2) as TractionControlStatus,
        abs: reader.readBits(2) as AntiLockBrakeStatus,
        scs: reader.readBits(2) as StabilityControlStatus,
        brakeBoost: reader.readBits(2) as BrakeBoostApplied,
        auxBrakes: reader.readBits(2) as AuxiliaryBrakeStatus
      },
      size: {
        width: reader.readBits(10),
        length: reader.readBits(12)
      }
    };
  }

  private static encodeWheelBrakes(wb: WheelBrakes): number {
    let value = 0;
    if (wb.unavailable) value |= 0x10;
    if (wb.leftFront) value |= 0x08;
    if (wb.leftRear) value |= 0x04;
    if (wb.rightFront) value |= 0x02;
    if (wb.rightRear) value |= 0x01;
    return value;
  }

  private static decodeWheelBrakes(value: number): WheelBrakes {
    return {
      unavailable: (value & 0x10) !== 0,
      leftFront: (value & 0x08) !== 0,
      leftRear: (value & 0x04) !== 0,
      rightFront: (value & 0x02) !== 0,
      rightRear: (value & 0x01) !== 0
    };
  }
}

// Bit manipulation utilities
class BitWriter {
  private buffer: number[] = [];
  private currentByte = 0;
  private bitPosition = 0;

  writeBits(value: number, bits: number): void {
    for (let i = bits - 1; i >= 0; i--) {
      this.currentByte = (this.currentByte << 1) | ((value >> i) & 1);
      this.bitPosition++;
      if (this.bitPosition === 8) {
        this.buffer.push(this.currentByte);
        this.currentByte = 0;
        this.bitPosition = 0;
      }
    }
  }

  writeSignedBits(value: number, bits: number): void {
    const mask = (1 << bits) - 1;
    this.writeBits(value & mask, bits);
  }

  toBuffer(): Buffer {
    if (this.bitPosition > 0) {
      this.buffer.push(this.currentByte << (8 - this.bitPosition));
    }
    return Buffer.from(this.buffer);
  }
}

class BitReader {
  private bitPosition = 0;

  constructor(private buffer: Buffer) {}

  readBits(bits: number): number {
    let value = 0;
    for (let i = 0; i < bits; i++) {
      const byteIndex = Math.floor(this.bitPosition / 8);
      const bitIndex = 7 - (this.bitPosition % 8);
      value = (value << 1) | ((this.buffer[byteIndex] >> bitIndex) & 1);
      this.bitPosition++;
    }
    return value;
  }

  readSignedBits(bits: number): number {
    const value = this.readBits(bits);
    const signBit = 1 << (bits - 1);
    return (value & signBit) ? value | ~((1 << bits) - 1) : value;
  }
}
```

---

## Cloud Data Models

### JSON Schema Definitions

```typescript
/**
 * JSON Schema for Cloud-based Vehicle Data
 * RESTful API payload structures
 */
const vehicleDataSchema = {
  $schema: "https://json-schema.org/draft/2020-12/schema",
  $id: "https://wia.org/schemas/connected-car/vehicle-data.json",
  title: "WIA Connected Car Vehicle Data",
  description: "Schema for vehicle telemetry and status data",
  type: "object",
  properties: {
    vehicleId: {
      type: "string",
      format: "uuid",
      description: "Unique vehicle identifier"
    },
    timestamp: {
      type: "string",
      format: "date-time",
      description: "ISO8601 timestamp of data collection"
    },
    location: {
      $ref: "#/$defs/location"
    },
    motion: {
      $ref: "#/$defs/motion"
    },
    powertrain: {
      $ref: "#/$defs/powertrain"
    },
    battery: {
      $ref: "#/$defs/battery"
    },
    diagnostics: {
      $ref: "#/$defs/diagnostics"
    }
  },
  required: ["vehicleId", "timestamp"],
  $defs: {
    location: {
      type: "object",
      properties: {
        latitude: {
          type: "number",
          minimum: -90,
          maximum: 90
        },
        longitude: {
          type: "number",
          minimum: -180,
          maximum: 180
        },
        altitude: {
          type: "number",
          description: "Meters above sea level"
        },
        heading: {
          type: "number",
          minimum: 0,
          maximum: 360,
          description: "Degrees from true north"
        },
        speed: {
          type: "number",
          minimum: 0,
          description: "Speed in km/h"
        },
        accuracy: {
          type: "object",
          properties: {
            horizontal: { type: "number" },
            vertical: { type: "number" }
          }
        },
        source: {
          type: "string",
          enum: ["GPS", "GLONASS", "GALILEO", "BEIDOU", "CELLULAR", "FUSED"]
        }
      },
      required: ["latitude", "longitude"]
    },
    motion: {
      type: "object",
      properties: {
        speed: {
          type: "number",
          minimum: 0,
          description: "km/h"
        },
        acceleration: {
          type: "object",
          properties: {
            longitudinal: { type: "number" },
            lateral: { type: "number" },
            vertical: { type: "number" }
          },
          description: "m/s²"
        },
        odometer: {
          type: "number",
          minimum: 0,
          description: "Total distance in km"
        },
        tripDistance: {
          type: "number",
          minimum: 0,
          description: "Trip distance in km"
        }
      }
    },
    powertrain: {
      type: "object",
      properties: {
        type: {
          type: "string",
          enum: ["ICE", "HYBRID", "PHEV", "BEV", "FCEV"]
        },
        engine: {
          type: "object",
          properties: {
            running: { type: "boolean" },
            rpm: { type: "integer", minimum: 0 },
            load: { type: "number", minimum: 0, maximum: 100 },
            coolantTemp: { type: "number" },
            oilTemp: { type: "number" }
          }
        },
        transmission: {
          type: "object",
          properties: {
            gear: { type: "string" },
            mode: { type: "string" }
          }
        },
        fuel: {
          type: "object",
          properties: {
            level: { type: "number", minimum: 0, maximum: 100 },
            range: { type: "number", minimum: 0 },
            consumption: { type: "number", minimum: 0 }
          }
        }
      }
    },
    battery: {
      type: "object",
      properties: {
        stateOfCharge: {
          type: "number",
          minimum: 0,
          maximum: 100
        },
        stateOfHealth: {
          type: "number",
          minimum: 0,
          maximum: 100
        },
        voltage: { type: "number" },
        current: { type: "number" },
        power: { type: "number" },
        temperature: {
          type: "object",
          properties: {
            average: { type: "number" },
            min: { type: "number" },
            max: { type: "number" }
          }
        },
        range: { type: "number", minimum: 0 },
        charging: {
          type: "object",
          properties: {
            isCharging: { type: "boolean" },
            chargeType: {
              type: "string",
              enum: ["AC_LEVEL_1", "AC_LEVEL_2", "DC_FAST", "WIRELESS"]
            },
            power: { type: "number" },
            timeToFull: { type: "integer" }
          }
        }
      }
    },
    diagnostics: {
      type: "object",
      properties: {
        dtcCodes: {
          type: "array",
          items: {
            type: "object",
            properties: {
              code: { type: "string", pattern: "^[PBCU][0-9A-F]{4}$" },
              description: { type: "string" },
              severity: { type: "string", enum: ["INFO", "WARNING", "CRITICAL"] },
              active: { type: "boolean" }
            }
          }
        },
        systemHealth: {
          type: "object",
          additionalProperties: {
            type: "object",
            properties: {
              status: { type: "string", enum: ["OK", "WARNING", "ERROR"] },
              message: { type: "string" }
            }
          }
        }
      }
    }
  }
};

/**
 * Protocol Buffer Definition for High-Performance Data Exchange
 */
const protobufDefinition = `
syntax = "proto3";

package wia.connectedcar;

option java_package = "org.wia.connectedcar.proto";
option go_package = "github.com/wia/connectedcar/proto";

// Vehicle telemetry message
message VehicleTelemetry {
  string vehicle_id = 1;
  int64 timestamp_us = 2;  // Unix timestamp in microseconds

  Location location = 3;
  Motion motion = 4;
  Powertrain powertrain = 5;
  Battery battery = 6;
  repeated DiagnosticCode dtc_codes = 7;
}

message Location {
  double latitude = 1;
  double longitude = 2;
  float altitude = 3;
  float heading = 4;
  float horizontal_accuracy = 5;
  float vertical_accuracy = 6;
  PositionSource source = 7;

  enum PositionSource {
    POSITION_SOURCE_UNKNOWN = 0;
    GPS = 1;
    GLONASS = 2;
    GALILEO = 3;
    BEIDOU = 4;
    CELLULAR = 5;
    WIFI = 6;
    FUSED = 7;
  }
}

message Motion {
  float speed_kmh = 1;
  Acceleration acceleration = 2;
  float odometer_km = 3;
  float trip_km = 4;
  float yaw_rate = 5;

  message Acceleration {
    float longitudinal = 1;
    float lateral = 2;
    float vertical = 3;
  }
}

message Powertrain {
  PowertrainType type = 1;
  Engine engine = 2;
  Transmission transmission = 3;
  FuelSystem fuel = 4;

  enum PowertrainType {
    POWERTRAIN_UNKNOWN = 0;
    ICE = 1;
    HYBRID = 2;
    PHEV = 3;
    BEV = 4;
    FCEV = 5;
  }

  message Engine {
    bool running = 1;
    uint32 rpm = 2;
    float load_percent = 3;
    float coolant_temp_c = 4;
    float oil_temp_c = 5;
    float oil_pressure_kpa = 6;
  }

  message Transmission {
    string gear_position = 1;
    int32 current_gear = 2;
    string mode = 3;
  }

  message FuelSystem {
    float level_percent = 1;
    float tank_capacity_l = 2;
    float range_km = 3;
    float consumption_l_100km = 4;
  }
}

message Battery {
  float state_of_charge = 1;
  float state_of_health = 2;
  float voltage = 3;
  float current = 4;
  float power_kw = 5;
  BatteryTemperature temperature = 6;
  float range_km = 7;
  ChargingStatus charging = 8;

  message BatteryTemperature {
    float average = 1;
    float min = 2;
    float max = 3;
  }

  message ChargingStatus {
    bool is_charging = 1;
    ChargeType type = 2;
    float power_kw = 3;
    uint32 time_to_full_sec = 4;
    float target_soc = 5;

    enum ChargeType {
      CHARGE_TYPE_UNKNOWN = 0;
      AC_LEVEL_1 = 1;
      AC_LEVEL_2 = 2;
      DC_FAST = 3;
      WIRELESS = 4;
    }
  }
}

message DiagnosticCode {
  string code = 1;
  string description = 2;
  Severity severity = 3;
  bool active = 4;
  int64 first_occurrence_us = 5;
  uint32 occurrence_count = 6;

  enum Severity {
    SEVERITY_UNKNOWN = 0;
    INFO = 1;
    WARNING = 2;
    CRITICAL = 3;
    SAFETY = 4;
  }
}

// Batch upload message
message TelemetryBatch {
  string vehicle_id = 1;
  repeated VehicleTelemetry records = 2;
  int64 upload_timestamp_us = 3;
  uint32 sequence_number = 4;
}

// Event notification
message VehicleEvent {
  string vehicle_id = 1;
  int64 timestamp_us = 2;
  EventType type = 3;
  Severity severity = 4;
  string description = 5;
  map<string, string> metadata = 6;
  Location location = 7;

  enum EventType {
    EVENT_UNKNOWN = 0;
    CRASH_DETECTED = 1;
    THEFT_ALERT = 2;
    GEOFENCE_VIOLATION = 3;
    SPEEDING_ALERT = 4;
    HARSH_BRAKING = 5;
    HARSH_ACCELERATION = 6;
    LOW_FUEL = 7;
    LOW_BATTERY = 8;
    DTC_TRIGGERED = 9;
    MAINTENANCE_DUE = 10;
    OTA_AVAILABLE = 11;
  }

  enum Severity {
    SEVERITY_UNKNOWN = 0;
    INFO = 1;
    WARNING = 2;
    CRITICAL = 3;
    EMERGENCY = 4;
  }
}
`;

/**
 * Data Format Conversion Utilities
 */
class DataFormatConverter {
  /**
   * Convert VSS path to JSON pointer
   */
  static vssToJsonPointer(vssPath: string): string {
    return "/" + vssPath.replace(/\./g, "/");
  }

  /**
   * Convert JSON to Protocol Buffer binary
   */
  static jsonToProtobuf(json: object, messageType: string): Buffer {
    // Implementation would use protobuf.js or similar
    throw new Error("Requires protobuf runtime");
  }

  /**
   * Convert Protocol Buffer binary to JSON
   */
  static protobufToJson(buffer: Buffer, messageType: string): object {
    // Implementation would use protobuf.js or similar
    throw new Error("Requires protobuf runtime");
  }

  /**
   * Convert BSM to JSON representation
   */
  static bsmToJson(bsm: BasicSafetyMessage): object {
    return {
      messageCount: bsm.msgCnt,
      temporaryId: Buffer.from(bsm.id).toString("hex"),
      timestamp: bsm.secMark,
      position: {
        latitude: bsm.lat.value / 10000000,
        longitude: bsm.long.value / 10000000,
        elevation: bsm.elev.value / 10
      },
      motion: {
        speed: bsm.speed.value * 0.02,
        heading: bsm.heading.value * 0.0125,
        steeringAngle: bsm.angle.value * 1.5
      },
      acceleration: {
        longitudinal: bsm.accelSet.long * 0.01,
        lateral: bsm.accelSet.lat * 0.01,
        vertical: bsm.accelSet.vert * 0.02 * 9.81,
        yawRate: bsm.accelSet.yaw * 0.01
      },
      brakes: {
        wheelBrakes: bsm.brakes.wheelBrakes,
        traction: TransmissionState[bsm.brakes.traction],
        abs: AntiLockBrakeStatus[bsm.brakes.abs],
        stabilityControl: StabilityControlStatus[bsm.brakes.scs]
      },
      size: {
        width: bsm.size.width / 100,
        length: bsm.size.length / 100
      }
    };
  }
}
```

---

## Data Serialization Performance

```typescript
/**
 * Benchmark different serialization formats
 */
async function benchmarkSerialization() {
  const testData: VehicleTelemetry = generateTestTelemetry();
  const iterations = 10000;

  console.log("Serialization Benchmark Results:");
  console.log("================================");

  // JSON
  const jsonStart = performance.now();
  for (let i = 0; i < iterations; i++) {
    const json = JSON.stringify(testData);
    JSON.parse(json);
  }
  const jsonTime = performance.now() - jsonStart;
  const jsonSize = Buffer.from(JSON.stringify(testData)).length;
  console.log(`JSON: ${jsonTime.toFixed(2)}ms, Size: ${jsonSize} bytes`);

  // MessagePack
  // const msgpackStart = performance.now();
  // for (let i = 0; i < iterations; i++) {
  //   const packed = msgpack.encode(testData);
  //   msgpack.decode(packed);
  // }
  // const msgpackTime = performance.now() - msgpackStart;
  // console.log(`MessagePack: ${msgpackTime.toFixed(2)}ms`);

  // Protocol Buffers would be tested similarly

  console.log("\nFormat Comparison:");
  console.log("==================");
  console.log("| Format      | Size (bytes) | Relative Size | Notes            |");
  console.log("|-------------|--------------|---------------|------------------|");
  console.log(`| JSON        | ${jsonSize.toString().padStart(12)} | 100%          | Human readable   |`);
  console.log(`| MessagePack | ${Math.round(jsonSize * 0.65).toString().padStart(12)} | ~65%          | Binary JSON      |`);
  console.log(`| Protobuf    | ${Math.round(jsonSize * 0.45).toString().padStart(12)} | ~45%          | Schema required  |`);
  console.log(`| WTP Binary  | ${Math.round(jsonSize * 0.35).toString().padStart(12)} | ~35%          | Custom protocol  |`);
}

interface VehicleTelemetry {
  vehicleId: string;
  timestamp: string;
  location: {
    latitude: number;
    longitude: number;
    altitude: number;
    heading: number;
    speed: number;
  };
  battery: {
    stateOfCharge: number;
    voltage: number;
    current: number;
  };
}

function generateTestTelemetry(): VehicleTelemetry {
  return {
    vehicleId: "550e8400-e29b-41d4-a716-446655440000",
    timestamp: new Date().toISOString(),
    location: {
      latitude: 37.7749,
      longitude: -122.4194,
      altitude: 10.5,
      heading: 270.5,
      speed: 65.3
    },
    battery: {
      stateOfCharge: 78.5,
      voltage: 398.2,
      current: -45.6
    }
  };
}
```

---

## Summary

| Format | Use Case | Size Efficiency | Processing Speed | Interoperability |
|--------|----------|-----------------|------------------|------------------|
| **VSS JSON** | Cloud APIs | Medium | High | Excellent |
| **WTP Binary** | Telematics | Very High | Very High | WIA Ecosystem |
| **SAE J2735 UPER** | V2X | High | Medium | V2X Networks |
| **Protobuf** | Cloud Storage | High | Very High | Good |
| **MQTT/JSON** | IoT Events | Medium | High | Excellent |

---

**Next Chapter:** [Chapter 4: API Interface](./04-api-interface.md) - REST, GraphQL, and streaming APIs for connected car data access.

---

© 2025 World Industry Association (WIA). All rights reserved.
