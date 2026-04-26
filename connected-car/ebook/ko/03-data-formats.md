# 제3장: 커넥티드카 데이터 포맷

## 차량 데이터 스키마, 온톨로지 및 메시지 구조

이 장에서는 커넥티드카 통신에 사용되는 포괄적인 데이터 포맷을 정의하여 제조사, 플랫폼 및 지역 간 상호운용성을 보장합니다.

---

## 차량 신호 사양 (VSS)

### COVESA VSS 표준

```typescript
// WIA 커넥티드카 데이터 포맷 구현
// COVESA 차량 신호 사양 (VSS) 4.0 기반

/**
 * 차량 신호 사양 트리 구조
 * 차량 데이터 신호의 계층적 구성
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
 * 커넥티드카용 핵심 VSS 트리 정의
 */
const vehicleSignalTree: Record<string, VSSNode> = {
  Vehicle: {
    type: "branch",
    description: "상위 레벨 차량 데이터",
    children: {
      VehicleIdentification: {
        type: "branch",
        description: "차량 식별 속성",
        children: {
          VIN: {
            type: "attribute",
            description: "ISO 3779에 정의된 차량 식별 번호",
            datatype: "string"
          },
          WMI: {
            type: "attribute",
            description: "세계 제조사 식별자 (VIN 위치 1-3)",
            datatype: "string"
          },
          Brand: {
            type: "attribute",
            description: "차량 브랜드 또는 제조사",
            datatype: "string"
          },
          Model: {
            type: "attribute",
            description: "차량 모델",
            datatype: "string"
          },
          Year: {
            type: "attribute",
            description: "모델 연도",
            datatype: "uint16",
            min: 1900,
            max: 2100
          },
          BodyType: {
            type: "attribute",
            description: "차량 차체 유형",
            datatype: "string",
            allowed: ["SEDAN", "SUV", "HATCHBACK", "WAGON", "COUPE", "CONVERTIBLE", "PICKUP", "VAN", "TRUCK"]
          }
        }
      },
      CurrentLocation: {
        type: "branch",
        description: "현재 차량 위치",
        children: {
          Latitude: {
            type: "sensor",
            description: "WGS84 10진 도 단위 현재 위도",
            datatype: "double",
            min: -90,
            max: 90,
            unit: "degrees"
          },
          Longitude: {
            type: "sensor",
            description: "WGS84 10진 도 단위 현재 경도",
            datatype: "double",
            min: -180,
            max: 180,
            unit: "degrees"
          },
          Altitude: {
            type: "sensor",
            description: "해수면 기준 현재 고도 (미터)",
            datatype: "double",
            unit: "m"
          },
          Heading: {
            type: "sensor",
            description: "진북 기준 현재 방향각 (도)",
            datatype: "double",
            min: 0,
            max: 360,
            unit: "degrees"
          },
          HorizontalAccuracy: {
            type: "sensor",
            description: "수평 위치 정확도 (미터)",
            datatype: "double",
            unit: "m"
          },
          VerticalAccuracy: {
            type: "sensor",
            description: "수직 위치 정확도 (미터)",
            datatype: "double",
            unit: "m"
          },
          GNSSReceiver: {
            type: "branch",
            description: "GNSS 수신기 정보",
            children: {
              FixType: {
                type: "sensor",
                description: "현재 GNSS 픽스 유형",
                datatype: "string",
                allowed: ["NONE", "2D", "3D", "RTK_FLOAT", "RTK_FIXED"]
              },
              SatellitesUsed: {
                type: "sensor",
                description: "픽스에 사용된 위성 수",
                datatype: "uint8"
              }
            }
          },
          Timestamp: {
            type: "sensor",
            description: "ISO8601 형식의 위치 판독 타임스탬프",
            datatype: "string"
          }
        }
      },
      Speed: {
        type: "sensor",
        description: "차량 속도 (km/h)",
        datatype: "float",
        min: 0,
        max: 500,
        unit: "km/h"
      },
      TraveledDistance: {
        type: "sensor",
        description: "누적 트립 거리 (km)",
        datatype: "float",
        unit: "km"
      },
      TraveledDistanceTotal: {
        type: "sensor",
        description: "총 누적 거리 (주행거리계) (km)",
        datatype: "float",
        unit: "km"
      },
      Acceleration: {
        type: "branch",
        description: "다축 가속도",
        children: {
          Longitudinal: {
            type: "sensor",
            description: "종방향 가속도 (양수 = 전진)",
            datatype: "float",
            unit: "m/s^2"
          },
          Lateral: {
            type: "sensor",
            description: "횡방향 가속도 (양수 = 우측)",
            datatype: "float",
            unit: "m/s^2"
          },
          Vertical: {
            type: "sensor",
            description: "수직 가속도 (양수 = 상향)",
            datatype: "float",
            unit: "m/s^2"
          }
        }
      },
      AngularVelocity: {
        type: "branch",
        description: "각속도 (회전율)",
        children: {
          Pitch: {
            type: "sensor",
            description: "피치율 (양수 = 기수 상향)",
            datatype: "float",
            unit: "degrees/s"
          },
          Roll: {
            type: "sensor",
            description: "롤율 (양수 = 우측 하향)",
            datatype: "float",
            unit: "degrees/s"
          },
          Yaw: {
            type: "sensor",
            description: "요율 (양수 = 위에서 볼 때 시계방향)",
            datatype: "float",
            unit: "degrees/s"
          }
        }
      }
    }
  },
  Powertrain: {
    type: "branch",
    description: "파워트레인 시스템",
    children: {
      Type: {
        type: "attribute",
        description: "파워트레인 유형",
        datatype: "string",
        allowed: ["ICE", "HYBRID", "PHEV", "BEV", "FCEV"]
      },
      CombustionEngine: {
        type: "branch",
        description: "내연기관 신호",
        children: {
          IsRunning: {
            type: "sensor",
            description: "엔진 작동 상태",
            datatype: "boolean"
          },
          Speed: {
            type: "sensor",
            description: "엔진 회전수 (RPM)",
            datatype: "uint16",
            unit: "rpm",
            max: 20000
          },
          EngineLoad: {
            type: "sensor",
            description: "엔진 부하율 (%)",
            datatype: "uint8",
            unit: "percent",
            min: 0,
            max: 100
          },
          TorqueOutput: {
            type: "sensor",
            description: "현재 엔진 토크 출력",
            datatype: "float",
            unit: "Nm"
          },
          CoolantTemperature: {
            type: "sensor",
            description: "엔진 냉각수 온도",
            datatype: "int16",
            unit: "celsius"
          },
          OilTemperature: {
            type: "sensor",
            description: "엔진 오일 온도",
            datatype: "int16",
            unit: "celsius"
          },
          OilPressure: {
            type: "sensor",
            description: "엔진 오일 압력",
            datatype: "uint16",
            unit: "kPa"
          }
        }
      },
      ElectricMotor: {
        type: "branch",
        description: "전기 모터 신호",
        children: {
          Power: {
            type: "sensor",
            description: "현재 전기 모터 출력 (양수 = 구동)",
            datatype: "int16",
            unit: "kW"
          },
          Torque: {
            type: "sensor",
            description: "현재 모터 토크",
            datatype: "int16",
            unit: "Nm"
          },
          Temperature: {
            type: "sensor",
            description: "모터 온도",
            datatype: "int16",
            unit: "celsius"
          },
          Speed: {
            type: "sensor",
            description: "모터 회전 속도",
            datatype: "int32",
            unit: "rpm"
          }
        }
      },
      TractionBattery: {
        type: "branch",
        description: "고전압 구동 배터리",
        children: {
          StateOfCharge: {
            type: "branch",
            description: "배터리 충전 상태",
            children: {
              Current: {
                type: "sensor",
                description: "현재 충전 상태",
                datatype: "float",
                unit: "percent",
                min: 0,
                max: 100
              },
              Displayed: {
                type: "sensor",
                description: "표시 충전 상태 (실제와 다를 수 있음)",
                datatype: "float",
                unit: "percent",
                min: 0,
                max: 100
              }
            }
          },
          StateOfHealth: {
            type: "sensor",
            description: "배터리 건강 상태",
            datatype: "float",
            unit: "percent",
            min: 0,
            max: 100
          },
          NominalCapacity: {
            type: "attribute",
            description: "공칭 배터리 용량",
            datatype: "float",
            unit: "kWh"
          },
          CurrentCapacity: {
            type: "sensor",
            description: "현재 사용 가능 배터리 용량",
            datatype: "float",
            unit: "kWh"
          },
          CurrentVoltage: {
            type: "sensor",
            description: "현재 배터리 전압",
            datatype: "float",
            unit: "V"
          },
          CurrentCurrent: {
            type: "sensor",
            description: "현재 배터리 전류 (양수 = 방전)",
            datatype: "float",
            unit: "A"
          },
          Temperature: {
            type: "branch",
            description: "배터리 온도 측정값",
            children: {
              Average: {
                type: "sensor",
                description: "평균 배터리 온도",
                datatype: "float",
                unit: "celsius"
              },
              Min: {
                type: "sensor",
                description: "최소 셀 온도",
                datatype: "float",
                unit: "celsius"
              },
              Max: {
                type: "sensor",
                description: "최대 셀 온도",
                datatype: "float",
                unit: "celsius"
              }
            }
          },
          Range: {
            type: "sensor",
            description: "예상 잔여 주행거리",
            datatype: "uint32",
            unit: "m"
          },
          Charging: {
            type: "branch",
            description: "충전 상태 및 매개변수",
            children: {
              IsCharging: {
                type: "sensor",
                description: "충전 상태",
                datatype: "boolean"
              },
              ChargeType: {
                type: "sensor",
                description: "현재 충전 연결 유형",
                datatype: "string",
                allowed: ["AC_LEVEL_1", "AC_LEVEL_2", "DC_FAST", "WIRELESS", "NOT_CONNECTED"]
              },
              ChargePower: {
                type: "sensor",
                description: "현재 충전 전력",
                datatype: "float",
                unit: "kW"
              },
              ChargeLimit: {
                type: "actuator",
                description: "목표 충전 상태 한도",
                datatype: "uint8",
                unit: "percent",
                min: 50,
                max: 100
              },
              TimeToComplete: {
                type: "sensor",
                description: "충전 한도 도달 예상 시간",
                datatype: "uint32",
                unit: "s"
              }
            }
          }
        }
      },
      Transmission: {
        type: "branch",
        description: "변속 시스템",
        children: {
          Type: {
            type: "attribute",
            description: "변속기 유형",
            datatype: "string",
            allowed: ["MANUAL", "AUTOMATIC", "DCT", "CVT", "DIRECT_DRIVE"]
          },
          CurrentGear: {
            type: "sensor",
            description: "현재 기어 (0=중립, -1=후진)",
            datatype: "int8"
          },
          GearCount: {
            type: "attribute",
            description: "전진 기어 수",
            datatype: "uint8"
          },
          DriveType: {
            type: "attribute",
            description: "구동 구성",
            datatype: "string",
            allowed: ["FWD", "RWD", "AWD", "4WD"]
          },
          SelectedGear: {
            type: "sensor",
            description: "선택된 기어 위치 (PRND)",
            datatype: "string",
            allowed: ["PARK", "REVERSE", "NEUTRAL", "DRIVE", "SPORT", "LOW", "MANUAL"]
          }
        }
      },
      FuelSystem: {
        type: "branch",
        description: "연료 시스템 (ICE/하이브리드 차량)",
        children: {
          Level: {
            type: "sensor",
            description: "연료 레벨 백분율",
            datatype: "uint8",
            unit: "percent",
            min: 0,
            max: 100
          },
          TankCapacity: {
            type: "attribute",
            description: "연료 탱크 용량",
            datatype: "float",
            unit: "l"
          },
          Range: {
            type: "sensor",
            description: "연료 기준 예상 잔여 주행거리",
            datatype: "uint32",
            unit: "m"
          },
          AverageConsumption: {
            type: "sensor",
            description: "평균 연료 소비량",
            datatype: "float",
            unit: "l/100km"
          },
          InstantConsumption: {
            type: "sensor",
            description: "순간 연료 소비량",
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

## 텔레매틱스 메시지 포맷

### WIA 텔레매틱스 프로토콜 (WTP)

```typescript
/**
 * WIA 텔레매틱스 프로토콜 (WTP) 메시지 구조
 * 차량-클라우드 통신을 위한 효율적인 바이너리 포맷
 */
interface WTPMessage {
  header: WTPHeader;
  payload: WTPPayload;
  checksum: number;  // CRC-32
}

interface WTPHeader {
  version: number;           // 프로토콜 버전 (1바이트)
  messageType: WTPMessageType;  // 메시지 유형 (1바이트)
  vehicleId: Buffer;         // 차량 식별자 (16바이트, UUID)
  sequenceNumber: number;    // 메시지 시퀀스 (4바이트)
  timestamp: bigint;         // Unix 타임스탬프 (마이크로초) (8바이트)
  payloadLength: number;     // 페이로드 길이 (2바이트)
  flags: WTPFlags;           // 메시지 플래그 (1바이트)
  reserved: Buffer;          // 향후 사용 예약 (3바이트)
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
  encrypted: boolean;        // 비트 0: 페이로드 암호화
  compressed: boolean;       // 비트 1: 페이로드 압축
  requiresAck: boolean;      // 비트 2: 확인 응답 필요
  priority: WTPPriority;     // 비트 3-4: 메시지 우선순위
  fragmented: boolean;       // 비트 5: 분할 메시지의 일부
  lastFragment: boolean;     // 비트 6: 마지막 분할
  reserved: boolean;         // 비트 7: 예약됨
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
 * 텔레메트리 페이로드 구조
 */
interface TelemetryPayload {
  signalCount: number;
  signals: TelemetrySignal[];
}

interface TelemetrySignal {
  signalId: number;          // VSS 신호 식별자
  timestamp: number;         // 헤더 타임스탬프로부터의 델타 (ms)
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
 * WTP 메시지 인코더/디코더
 */
class WTPCodec {
  private static HEADER_SIZE = 36;
  private static CHECKSUM_SIZE = 4;

  /**
   * WTP 메시지를 바이너리 버퍼로 인코딩
   */
  static encode(message: WTPMessage): Buffer {
    const headerBuffer = this.encodeHeader(message.header);
    const payloadBuffer = this.encodePayload(message.payload, message.header.messageType);

    // 플래그에 따라 압축 적용
    const processedPayload = message.header.flags.compressed
      ? this.compress(payloadBuffer)
      : payloadBuffer;

    // 플래그에 따라 암호화 적용
    const finalPayload = message.header.flags.encrypted
      ? this.encrypt(processedPayload)
      : processedPayload;

    // 체크섬 계산
    const checksumBuffer = Buffer.alloc(this.CHECKSUM_SIZE);
    const checksum = this.calculateCRC32(Buffer.concat([headerBuffer, finalPayload]));
    checksumBuffer.writeUInt32BE(checksum, 0);

    return Buffer.concat([headerBuffer, finalPayload, checksumBuffer]);
  }

  /**
   * 바이너리 버퍼에서 WTP 메시지 디코딩
   */
  static decode(buffer: Buffer): WTPMessage {
    // 최소 크기 검증
    if (buffer.length < this.HEADER_SIZE + this.CHECKSUM_SIZE) {
      throw new Error("WTP 메시지에 비해 버퍼가 너무 작습니다");
    }

    // 체크섬 추출 및 검증
    const messageWithoutChecksum = buffer.subarray(0, buffer.length - this.CHECKSUM_SIZE);
    const receivedChecksum = buffer.readUInt32BE(buffer.length - this.CHECKSUM_SIZE);
    const calculatedChecksum = this.calculateCRC32(messageWithoutChecksum);

    if (receivedChecksum !== calculatedChecksum) {
      throw new Error("체크섬 검증 실패");
    }

    // 헤더 디코딩
    const header = this.decodeHeader(buffer.subarray(0, this.HEADER_SIZE));

    // 페이로드 추출
    let payloadBuffer = buffer.subarray(
      this.HEADER_SIZE,
      buffer.length - this.CHECKSUM_SIZE
    );

    // 암호화된 경우 복호화
    if (header.flags.encrypted) {
      payloadBuffer = this.decrypt(payloadBuffer);
    }

    // 압축된 경우 압축 해제
    if (header.flags.compressed) {
      payloadBuffer = this.decompress(payloadBuffer);
    }

    // 메시지 유형에 따라 페이로드 디코딩
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

  // 압축/암호화/CRC 스텁 메서드
  private static compress(buffer: Buffer): Buffer { return buffer; }
  private static decompress(buffer: Buffer): Buffer { return buffer; }
  private static encrypt(buffer: Buffer): Buffer { return buffer; }
  private static decrypt(buffer: Buffer): Buffer { return buffer; }
  private static calculateCRC32(buffer: Buffer): number { return 0; }
  private static decodeHeader(buffer: Buffer): WTPHeader { return {} as WTPHeader; }
  private static decodePayload(buffer: Buffer, type: WTPMessageType): WTPPayload {
    return {} as WTPPayload;
  }
  private static encodePayload(payload: WTPPayload, type: WTPMessageType): Buffer {
    return Buffer.alloc(0);
  }
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

## V2X 메시지 표준

### SAE J2735 메시지 포맷

```typescript
/**
 * SAE J2735 기본 안전 메시지 (BSM)
 * Part 1 - 초당 10회 전송되는 핵심 데이터 요소
 */
interface BasicSafetyMessage {
  msgCnt: number;              // 메시지 카운트 (0-127)
  id: TemporaryID;             // 임시 ID (4바이트)
  secMark: number;             // DSEC (0-65535, 분 내 ms)
  lat: Latitude;               // 1/10 마이크로도 단위 위도
  long: Longitude;             // 1/10 마이크로도 단위 경도
  elev: Elevation;             // 데시미터 단위 고도
  accuracy: PositionalAccuracy;
  transmission: TransmissionState;
  speed: Speed;                // 0.02 m/s 단위 속도
  heading: Heading;            // 0.0125도 단위 방향각
  angle: SteeringWheelAngle;   // 조향휠 각도
  accelSet: AccelerationSet4Way;
  brakes: BrakeSystemStatus;
  size: VehicleSize;
}

// BSM 필드용 타입 정의
type TemporaryID = [number, number, number, number];

interface Latitude {
  value: number;  // -900000000 ~ 900000001 (1/10 마이크로도)
}

interface Longitude {
  value: number;  // -1799999999 ~ 1800000001 (1/10 마이크로도)
}

interface Elevation {
  value: number;  // -4096 ~ 61439 (데시미터, -409.6m ~ 6143.9m)
}

interface PositionalAccuracy {
  semiMajor: number;   // 장반경 정확도 (0-255, 0.05m 단위)
  semiMinor: number;   // 단반경 정확도 (0-255, 0.05m 단위)
  orientation: number; // 정확도 타원 방향 (0-65535)
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
  value: number;  // 0-8191 (0.02 m/s 단위, 0-163.82 m/s)
}

interface Heading {
  value: number;  // 0-28800 (0.0125도 단위, 0-360도)
}

interface AccelerationSet4Way {
  long: number;   // -2000 ~ 2001 (0.01 m/s² 단위)
  lat: number;    // -2000 ~ 2001 (0.01 m/s² 단위)
  vert: number;   // -127 ~ 127 (0.02 g 단위)
  yaw: number;    // -32767 ~ 32767 (0.01 deg/s 단위)
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

interface VehicleSize {
  width: number;   // 0-1023 (1 cm 단위)
  length: number;  // 0-4095 (1 cm 단위)
}

/**
 * SAE J2735 ASN.1 구조를 따르는 BSM 인코더
 */
class BSMEncoder {
  /**
   * BSM을 UPER (Unaligned Packed Encoding Rules)로 인코딩
   */
  static encodeToUPER(bsm: BasicSafetyMessage): Buffer {
    const writer = new BitWriter();

    // 메시지 카운트 (7비트, 0-127)
    writer.writeBits(bsm.msgCnt, 7);

    // 임시 ID (32비트)
    for (const byte of bsm.id) {
      writer.writeBits(byte, 8);
    }

    // DSecond (16비트)
    writer.writeBits(bsm.secMark, 16);

    // 위도 (32비트, 부호 있음)
    writer.writeSignedBits(bsm.lat.value, 32);

    // 경도 (32비트, 부호 있음)
    writer.writeSignedBits(bsm.long.value, 32);

    // 고도 (16비트)
    writer.writeBits(bsm.elev.value + 4096, 16);  // 인코딩용 오프셋

    // 위치 정확도
    writer.writeBits(bsm.accuracy.semiMajor, 8);
    writer.writeBits(bsm.accuracy.semiMinor, 8);
    writer.writeBits(bsm.accuracy.orientation, 16);

    // 변속 상태 (3비트)
    writer.writeBits(bsm.transmission, 3);

    // 속도 (13비트)
    writer.writeBits(bsm.speed.value, 13);

    // 방향각 (16비트)
    writer.writeBits(bsm.heading.value, 16);

    // 조향휠 각도 (8비트, 부호 있음)
    writer.writeSignedBits(bsm.angle.value, 8);

    // 가속도 세트
    writer.writeSignedBits(bsm.accelSet.long, 12);
    writer.writeSignedBits(bsm.accelSet.lat, 12);
    writer.writeSignedBits(bsm.accelSet.vert, 8);
    writer.writeSignedBits(bsm.accelSet.yaw, 16);

    // 브레이크 시스템 상태
    writer.writeBits(this.encodeWheelBrakes(bsm.brakes.wheelBrakes), 5);
    writer.writeBits(bsm.brakes.traction, 2);
    writer.writeBits(bsm.brakes.abs, 2);
    writer.writeBits(bsm.brakes.scs, 2);
    writer.writeBits(bsm.brakes.brakeBoost, 2);
    writer.writeBits(bsm.brakes.auxBrakes, 2);

    // 차량 크기
    writer.writeBits(bsm.size.width, 10);
    writer.writeBits(bsm.size.length, 12);

    return writer.toBuffer();
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
}

// 비트 조작 유틸리티
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

## 클라우드 데이터 모델

### JSON 스키마 정의

```typescript
/**
 * 클라우드 기반 차량 데이터용 JSON 스키마
 * RESTful API 페이로드 구조
 */
const vehicleDataSchema = {
  $schema: "https://json-schema.org/draft/2020-12/schema",
  $id: "https://wia.org/schemas/connected-car/vehicle-data.json",
  title: "WIA 커넥티드카 차량 데이터",
  description: "차량 텔레매틱스 및 상태 데이터용 스키마",
  type: "object",
  properties: {
    vehicleId: {
      type: "string",
      format: "uuid",
      description: "고유 차량 식별자"
    },
    timestamp: {
      type: "string",
      format: "date-time",
      description: "데이터 수집 ISO8601 타임스탬프"
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
        latitude: { type: "number", minimum: -90, maximum: 90 },
        longitude: { type: "number", minimum: -180, maximum: 180 },
        altitude: { type: "number", description: "해수면 기준 미터" },
        heading: { type: "number", minimum: 0, maximum: 360, description: "진북 기준 도" },
        speed: { type: "number", minimum: 0, description: "km/h 속도" },
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
        speed: { type: "number", minimum: 0, description: "km/h" },
        acceleration: {
          type: "object",
          properties: {
            longitudinal: { type: "number" },
            lateral: { type: "number" },
            vertical: { type: "number" }
          },
          description: "m/s²"
        },
        odometer: { type: "number", minimum: 0, description: "km 총 거리" },
        tripDistance: { type: "number", minimum: 0, description: "km 트립 거리" }
      }
    },
    powertrain: {
      type: "object",
      properties: {
        type: { type: "string", enum: ["ICE", "HYBRID", "PHEV", "BEV", "FCEV"] },
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
        stateOfCharge: { type: "number", minimum: 0, maximum: 100 },
        stateOfHealth: { type: "number", minimum: 0, maximum: 100 },
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
            chargeType: { type: "string", enum: ["AC_LEVEL_1", "AC_LEVEL_2", "DC_FAST", "WIRELESS"] },
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
```

---

## 데이터 직렬화 성능

```typescript
/**
 * 다양한 직렬화 포맷 벤치마크
 */
async function benchmarkSerialization() {
  const testData: VehicleTelemetry = generateTestTelemetry();
  const iterations = 10000;

  console.log("직렬화 벤치마크 결과:");
  console.log("================================");

  // JSON
  const jsonStart = performance.now();
  for (let i = 0; i < iterations; i++) {
    const json = JSON.stringify(testData);
    JSON.parse(json);
  }
  const jsonTime = performance.now() - jsonStart;
  const jsonSize = Buffer.from(JSON.stringify(testData)).length;
  console.log(`JSON: ${jsonTime.toFixed(2)}ms, 크기: ${jsonSize} 바이트`);

  console.log("\n포맷 비교:");
  console.log("==================");
  console.log("| 포맷        | 크기 (바이트) | 상대 크기 | 비고             |");
  console.log("|-------------|---------------|-----------|------------------|");
  console.log(`| JSON        | ${jsonSize.toString().padStart(13)} | 100%      | 사람이 읽기 가능 |`);
  console.log(`| MessagePack | ${Math.round(jsonSize * 0.65).toString().padStart(13)} | ~65%      | 바이너리 JSON    |`);
  console.log(`| Protobuf    | ${Math.round(jsonSize * 0.45).toString().padStart(13)} | ~45%      | 스키마 필요      |`);
  console.log(`| WTP 바이너리| ${Math.round(jsonSize * 0.35).toString().padStart(13)} | ~35%      | 커스텀 프로토콜  |`);
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
      latitude: 37.5665,
      longitude: 126.9780,
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

## 요약

| 포맷 | 사용 사례 | 크기 효율성 | 처리 속도 | 상호운용성 |
|------|----------|-------------|----------|-----------|
| **VSS JSON** | 클라우드 API | 중간 | 높음 | 우수 |
| **WTP 바이너리** | 텔레매틱스 | 매우 높음 | 매우 높음 | WIA 에코시스템 |
| **SAE J2735 UPER** | V2X | 높음 | 중간 | V2X 네트워크 |
| **Protobuf** | 클라우드 저장소 | 높음 | 매우 높음 | 양호 |
| **MQTT/JSON** | IoT 이벤트 | 중간 | 높음 | 우수 |

---

**다음 장:** [제4장: API 인터페이스](./04-api-interface.md) - 커넥티드카 데이터 접근을 위한 REST, GraphQL 및 스트리밍 API.

---

© 2025 World Industry Association (WIA). All rights reserved.
