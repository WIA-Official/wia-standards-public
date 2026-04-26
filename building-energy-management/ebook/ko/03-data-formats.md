# 제3장: 데이터 포맷 및 스키마 (Phase 1)

## 서론

WIA-BEMS의 첫 번째 단계(Phase 1)는 표준화된 데이터 기반을 구축하는 것입니다. 빌딩 에너지 관리의 핵심은 정확하고 일관된 데이터입니다. 본 장에서는 WIA-BEMS에서 정의하는 데이터 모델, JSON 스키마, 그리고 데이터 품질 관리 방법론을 상세히 설명합니다.

---

## 3.1 WIA-BEMS 데이터 모델 개요

### 3.1.1 데이터 아키텍처

#### 계층적 데이터 구조

```typescript
// WIA-BEMS 데이터 아키텍처
interface WIABEMSDataArchitecture {
  hierarchicalStructure: {
    level1_portfolio: {
      name: '포트폴리오';
      description: '여러 건물의 집합';
      attributes: ['소유자 정보', '관리 조직', '집계 데이터'];
    };
    level2_building: {
      name: '건물';
      description: '개별 건축물';
      attributes: ['위치', '용도', '면적', '준공년도', '설비 목록'];
    };
    level3_system: {
      name: '시스템';
      description: '건물 내 설비 시스템';
      types: ['HVAC', '조명', '전력', '급수', '소방', '승강기'];
    };
    level4_equipment: {
      name: '설비';
      description: '개별 설비 기기';
      examples: ['AHU', '냉동기', '보일러', 'VAV', '조명기구'];
    };
    level5_point: {
      name: '포인트';
      description: '센서, 설정값, 상태값';
      types: ['analog_input', 'analog_output', 'binary_input', 'binary_output'];
    };
  };

  dataCategories: {
    staticData: {
      name: '정적 데이터';
      description: '시간에 따라 변하지 않거나 드물게 변하는 데이터';
      examples: [
        '건물 기본 정보',
        '설비 사양',
        '도면 정보',
        '유지보수 이력'
      ];
      updateFrequency: '비정기 (변경 시)';
    };
    dynamicData: {
      name: '동적 데이터';
      description: '실시간으로 변화하는 운영 데이터';
      examples: [
        '센서 측정값',
        '에너지 사용량',
        '설비 상태',
        '환경 조건'
      ];
      updateFrequency: '초~분 단위';
    };
    eventData: {
      name: '이벤트 데이터';
      description: '특정 시점에 발생하는 사건';
      examples: [
        '알람',
        '설비 기동/정지',
        '설정값 변경',
        '사용자 조작'
      ];
      characteristics: '비정형, 불규칙 발생';
    };
    derivedData: {
      name: '파생 데이터';
      description: '원시 데이터로부터 계산된 데이터';
      examples: [
        'KPI 지표',
        '에너지 효율',
        '이상 점수',
        '예측값'
      ];
      computationFrequency: '배치 또는 실시간';
    };
  };
}
```

### 3.1.2 데이터 모델 설계 원칙

```typescript
// 데이터 모델 설계 원칙
interface DataModelPrinciples {
  principle1_standardization: {
    name: '표준화';
    description: '일관된 데이터 구조와 명명 규칙 적용';
    implementation: {
      namingConvention: {
        format: 'snake_case 또는 camelCase 일관 사용';
        examples: ['supply_air_temperature', 'supplyAirTemperature'];
        rules: [
          '약어 사용 최소화',
          '단위 포함 금지 (별도 필드)',
          '영문 사용 권장'
        ];
      };
      dataTypes: {
        temperature: 'number (Celsius)';
        pressure: 'number (Pascal)';
        flow: 'number (L/s or m³/h)';
        power: 'number (Watt or kW)';
        energy: 'number (kWh)';
        timestamp: 'ISO 8601 format';
        boolean: 'true/false';
        enum: 'predefined string values';
      };
    };
  };

  principle2_extensibility: {
    name: '확장성';
    description: '새로운 데이터 유형 추가 용이';
    implementation: {
      additionalProperties: 'JSON Schema의 additionalProperties 활용';
      customAttributes: '커스텀 속성을 위한 별도 필드';
      versioning: '스키마 버전 관리';
    };
  };

  principle3_interoperability: {
    name: '상호운용성';
    description: '다양한 시스템과의 데이터 교환 지원';
    implementation: {
      standardFormats: ['JSON', 'CSV', 'XML'];
      protocols: ['REST', 'MQTT', 'WebSocket'];
      mappings: ['BACnet 매핑', 'Haystack 매핑', 'Brick 매핑'];
    };
  };

  principle4_quality: {
    name: '품질 보장';
    description: '데이터 정확성, 완전성, 적시성 확보';
    implementation: {
      validation: '입력 시 스키마 검증';
      qualityScoring: '데이터 품질 점수 산출';
      monitoring: '품질 지표 대시보드';
    };
  };
}
```

---

## 3.2 핵심 데이터 스키마

### 3.2.1 건물 정보 스키마

```typescript
// 건물 정보 JSON 스키마
const BuildingSchema = {
  $schema: 'https://json-schema.org/draft/2020-12/schema',
  $id: 'https://wia.org/schemas/bems/building.json',
  title: 'WIA-BEMS Building Schema',
  description: '건물 기본 정보를 정의하는 스키마',
  type: 'object',
  required: ['buildingId', 'name', 'location', 'buildingType', 'grossFloorArea'],

  properties: {
    buildingId: {
      type: 'string',
      pattern: '^BLD-[A-Z0-9]{8}$',
      description: '건물 고유 식별자'
    },
    name: {
      type: 'string',
      minLength: 1,
      maxLength: 200,
      description: '건물 명칭'
    },
    alternateNames: {
      type: 'array',
      items: { type: 'string' },
      description: '대체 명칭 목록'
    },

    location: {
      type: 'object',
      required: ['address', 'coordinates'],
      properties: {
        address: {
          type: 'object',
          required: ['country', 'city'],
          properties: {
            country: { type: 'string', description: '국가' },
            postalCode: { type: 'string', description: '우편번호' },
            province: { type: 'string', description: '시/도' },
            city: { type: 'string', description: '시/군/구' },
            district: { type: 'string', description: '읍/면/동' },
            streetAddress: { type: 'string', description: '도로명 주소' },
            lotAddress: { type: 'string', description: '지번 주소' }
          }
        },
        coordinates: {
          type: 'object',
          required: ['latitude', 'longitude'],
          properties: {
            latitude: {
              type: 'number',
              minimum: -90,
              maximum: 90,
              description: '위도'
            },
            longitude: {
              type: 'number',
              minimum: -180,
              maximum: 180,
              description: '경도'
            },
            elevation: {
              type: 'number',
              description: '해발 고도 (m)'
            }
          }
        },
        timezone: {
          type: 'string',
          pattern: '^[A-Za-z]+/[A-Za-z_]+$',
          description: 'IANA 타임존',
          examples: ['Asia/Seoul', 'America/New_York']
        },
        climateZone: {
          type: 'string',
          enum: ['1A', '2A', '2B', '3A', '3B', '3C', '4A', '4B', '4C', '5A', '5B', '5C', '6A', '6B', '7', '8'],
          description: 'ASHRAE 기후 구역'
        }
      }
    },

    buildingType: {
      type: 'object',
      required: ['primaryUse'],
      properties: {
        primaryUse: {
          type: 'string',
          enum: [
            'office', 'retail', 'hotel', 'hospital', 'school', 'university',
            'residential', 'industrial', 'warehouse', 'datacenter',
            'mixed_use', 'laboratory', 'convention_center', 'sports_facility'
          ],
          description: '주용도'
        },
        secondaryUses: {
          type: 'array',
          items: { type: 'string' },
          description: '부속 용도'
        },
        buildingClass: {
          type: 'string',
          enum: ['A', 'B', 'C'],
          description: '건물 등급 (오피스)'
        }
      }
    },

    physicalCharacteristics: {
      type: 'object',
      properties: {
        grossFloorArea: {
          type: 'number',
          minimum: 0,
          description: '연면적 (㎡)'
        },
        netLettableArea: {
          type: 'number',
          minimum: 0,
          description: '임대 면적 (㎡)'
        },
        conditionedArea: {
          type: 'number',
          minimum: 0,
          description: '공조 면적 (㎡)'
        },
        numberOfFloors: {
          type: 'object',
          properties: {
            aboveGround: { type: 'integer', minimum: 0 },
            belowGround: { type: 'integer', minimum: 0 }
          }
        },
        buildingHeight: {
          type: 'number',
          minimum: 0,
          description: '건물 높이 (m)'
        },
        footprint: {
          type: 'number',
          minimum: 0,
          description: '건축 면적 (㎡)'
        },
        envelopeArea: {
          type: 'number',
          minimum: 0,
          description: '외피 면적 (㎡)'
        },
        windowToWallRatio: {
          type: 'number',
          minimum: 0,
          maximum: 1,
          description: '창면적비'
        }
      }
    },

    constructionInfo: {
      type: 'object',
      properties: {
        yearBuilt: {
          type: 'integer',
          minimum: 1800,
          maximum: 2100,
          description: '준공 연도'
        },
        yearLastRenovated: {
          type: 'integer',
          description: '최근 리모델링 연도'
        },
        structureType: {
          type: 'string',
          enum: ['steel_frame', 'reinforced_concrete', 'composite', 'wood', 'masonry'],
          description: '구조 유형'
        },
        facadeMaterial: {
          type: 'string',
          description: '외장재'
        }
      }
    },

    operationalInfo: {
      type: 'object',
      properties: {
        operatingHours: {
          type: 'object',
          properties: {
            weekday: { $ref: '#/$defs/timeRange' },
            saturday: { $ref: '#/$defs/timeRange' },
            sunday: { $ref: '#/$defs/timeRange' },
            holidays: { $ref: '#/$defs/timeRange' }
          }
        },
        typicalOccupancy: {
          type: 'integer',
          minimum: 0,
          description: '평균 재실 인원'
        },
        maxOccupancy: {
          type: 'integer',
          minimum: 0,
          description: '최대 수용 인원'
        },
        occupancyDensity: {
          type: 'number',
          minimum: 0,
          description: '재실 밀도 (인/㎡)'
        }
      }
    },

    certifications: {
      type: 'array',
      items: {
        type: 'object',
        properties: {
          type: {
            type: 'string',
            enum: ['LEED', 'BREEAM', 'WELL', 'G-SEED', 'ZEB', 'ENERGY_STAR', 'NABERS'],
            description: '인증 유형'
          },
          level: { type: 'string', description: '인증 등급' },
          score: { type: 'number', description: '인증 점수' },
          certificationDate: { type: 'string', format: 'date' },
          expirationDate: { type: 'string', format: 'date' },
          certificateNumber: { type: 'string' }
        }
      }
    },

    energyProfile: {
      type: 'object',
      properties: {
        annualEnergyConsumption: {
          type: 'object',
          properties: {
            electricity: { type: 'number', description: 'kWh/년' },
            gas: { type: 'number', description: '㎥/년' },
            districtHeating: { type: 'number', description: 'Gcal/년' },
            districtCooling: { type: 'number', description: 'RTH/년' }
          }
        },
        eui: {
          type: 'number',
          description: '에너지 사용 강도 (kWh/㎡/년)'
        },
        peakDemand: {
          type: 'object',
          properties: {
            electric: { type: 'number', description: 'kW' },
            cooling: { type: 'number', description: 'RT' },
            heating: { type: 'number', description: 'kW' }
          }
        },
        baselineYear: { type: 'integer' }
      }
    },

    contacts: {
      type: 'array',
      items: {
        type: 'object',
        properties: {
          role: {
            type: 'string',
            enum: ['owner', 'property_manager', 'facility_manager', 'energy_manager', 'tenant']
          },
          name: { type: 'string' },
          organization: { type: 'string' },
          email: { type: 'string', format: 'email' },
          phone: { type: 'string' }
        }
      }
    },

    metadata: {
      type: 'object',
      properties: {
        createdAt: { type: 'string', format: 'date-time' },
        updatedAt: { type: 'string', format: 'date-time' },
        createdBy: { type: 'string' },
        version: { type: 'string' },
        source: { type: 'string' }
      }
    }
  },

  $defs: {
    timeRange: {
      type: 'object',
      properties: {
        start: { type: 'string', pattern: '^([01][0-9]|2[0-3]):[0-5][0-9]$' },
        end: { type: 'string', pattern: '^([01][0-9]|2[0-3]):[0-5][0-9]$' }
      }
    }
  }
};

// 건물 데이터 예시
const buildingDataExample = {
  buildingId: 'BLD-ABCD1234',
  name: '스마트타워',
  alternateNames: ['Smart Tower', '스마트 빌딩'],

  location: {
    address: {
      country: '대한민국',
      postalCode: '06141',
      province: '서울특별시',
      city: '강남구',
      district: '삼성동',
      streetAddress: '테헤란로 427'
    },
    coordinates: {
      latitude: 37.5088,
      longitude: 127.0631,
      elevation: 35
    },
    timezone: 'Asia/Seoul',
    climateZone: '4A'
  },

  buildingType: {
    primaryUse: 'office',
    secondaryUses: ['retail', 'parking'],
    buildingClass: 'A'
  },

  physicalCharacteristics: {
    grossFloorArea: 85000,
    netLettableArea: 62000,
    conditionedArea: 75000,
    numberOfFloors: {
      aboveGround: 35,
      belowGround: 6
    },
    buildingHeight: 165,
    windowToWallRatio: 0.45
  },

  constructionInfo: {
    yearBuilt: 2018,
    structureType: 'composite',
    facadeMaterial: 'curtain_wall'
  },

  operationalInfo: {
    operatingHours: {
      weekday: { start: '07:00', end: '22:00' },
      saturday: { start: '08:00', end: '18:00' },
      sunday: { start: '00:00', end: '00:00' },
      holidays: { start: '00:00', end: '00:00' }
    },
    typicalOccupancy: 4500,
    maxOccupancy: 6000,
    occupancyDensity: 0.073
  },

  certifications: [
    {
      type: 'LEED',
      level: 'Platinum',
      score: 82,
      certificationDate: '2019-03-15',
      certificateNumber: 'LEED-1234567'
    },
    {
      type: 'G-SEED',
      level: '최우수',
      certificationDate: '2018-12-01'
    }
  ],

  energyProfile: {
    annualEnergyConsumption: {
      electricity: 12500000,
      gas: 180000
    },
    eui: 165,
    peakDemand: {
      electric: 4800,
      cooling: 2400
    },
    baselineYear: 2019
  }
};
```

### 3.2.2 에너지 데이터 스키마

```typescript
// 에너지 데이터 JSON 스키마
const EnergyDataSchema = {
  $schema: 'https://json-schema.org/draft/2020-12/schema',
  $id: 'https://wia.org/schemas/bems/energy-data.json',
  title: 'WIA-BEMS Energy Data Schema',
  description: '에너지 사용 데이터를 정의하는 스키마',
  type: 'object',
  required: ['meterId', 'timestamp', 'readings'],

  properties: {
    meterId: {
      type: 'string',
      pattern: '^MTR-[A-Z0-9]{8}$',
      description: '미터 고유 식별자'
    },

    buildingId: {
      type: 'string',
      description: '건물 식별자'
    },

    location: {
      type: 'object',
      properties: {
        floor: { type: 'string' },
        zone: { type: 'string' },
        panel: { type: 'string' },
        circuit: { type: 'string' }
      },
      description: '미터 위치'
    },

    meterType: {
      type: 'string',
      enum: [
        'main_electric', 'submeter_electric', 'solar_production', 'ess_charge', 'ess_discharge',
        'main_gas', 'submeter_gas',
        'thermal_heating', 'thermal_cooling',
        'water_main', 'water_chilled', 'water_hot', 'water_condenser',
        'steam'
      ],
      description: '미터 유형'
    },

    timestamp: {
      type: 'string',
      format: 'date-time',
      description: '측정 시간 (ISO 8601)'
    },

    interval: {
      type: 'integer',
      enum: [1, 5, 15, 30, 60],
      description: '측정 간격 (분)'
    },

    readings: {
      type: 'object',
      properties: {
        // 전력 데이터
        activeEnergy: {
          type: 'number',
          description: '유효 전력량 (kWh)'
        },
        reactiveEnergy: {
          type: 'number',
          description: '무효 전력량 (kVARh)'
        },
        apparentEnergy: {
          type: 'number',
          description: '피상 전력량 (kVAh)'
        },
        activePower: {
          type: 'number',
          description: '유효 전력 (kW)'
        },
        reactivePower: {
          type: 'number',
          description: '무효 전력 (kVAR)'
        },
        apparentPower: {
          type: 'number',
          description: '피상 전력 (kVA)'
        },
        powerFactor: {
          type: 'number',
          minimum: -1,
          maximum: 1,
          description: '역률'
        },
        voltage: {
          type: 'object',
          properties: {
            phaseA: { type: 'number' },
            phaseB: { type: 'number' },
            phaseC: { type: 'number' },
            average: { type: 'number' },
            lineToLine: { type: 'number' }
          },
          description: '전압 (V)'
        },
        current: {
          type: 'object',
          properties: {
            phaseA: { type: 'number' },
            phaseB: { type: 'number' },
            phaseC: { type: 'number' },
            neutral: { type: 'number' },
            average: { type: 'number' }
          },
          description: '전류 (A)'
        },
        frequency: {
          type: 'number',
          description: '주파수 (Hz)'
        },
        thd: {
          type: 'object',
          properties: {
            voltage: { type: 'number' },
            current: { type: 'number' }
          },
          description: '고조파 왜곡률 (%)'
        },

        // 가스 데이터
        gasVolume: {
          type: 'number',
          description: '가스 사용량 (㎥)'
        },
        gasEnergy: {
          type: 'number',
          description: '가스 열량 (MJ)'
        },

        // 열량 데이터
        thermalEnergy: {
          type: 'number',
          description: '열량 (kWh 또는 Gcal)'
        },
        flowRate: {
          type: 'number',
          description: '유량 (㎥/h)'
        },
        supplyTemperature: {
          type: 'number',
          description: '공급 온도 (°C)'
        },
        returnTemperature: {
          type: 'number',
          description: '환수 온도 (°C)'
        },
        deltaTemperature: {
          type: 'number',
          description: '온도차 (°C)'
        },

        // 수량 데이터
        waterVolume: {
          type: 'number',
          description: '수량 (㎥)'
        },
        waterFlowRate: {
          type: 'number',
          description: '수량 유량 (㎥/h)'
        }
      }
    },

    demand: {
      type: 'object',
      properties: {
        current: { type: 'number', description: '현재 수요 (kW)' },
        predicted: { type: 'number', description: '예측 수요 (kW)' },
        peak: { type: 'number', description: '피크 수요 (kW)' },
        peakTimestamp: { type: 'string', format: 'date-time' }
      }
    },

    quality: {
      type: 'object',
      properties: {
        status: {
          type: 'string',
          enum: ['valid', 'estimated', 'interpolated', 'suspect', 'missing'],
          description: '데이터 상태'
        },
        qualityScore: {
          type: 'number',
          minimum: 0,
          maximum: 100,
          description: '품질 점수'
        },
        flags: {
          type: 'array',
          items: { type: 'string' },
          description: '품질 플래그'
        }
      }
    },

    cost: {
      type: 'object',
      properties: {
        energyCost: { type: 'number', description: '에너지 비용 (원)' },
        demandCost: { type: 'number', description: '수요 비용 (원)' },
        totalCost: { type: 'number', description: '총 비용 (원)' },
        tariff: { type: 'string', description: '적용 요금제' }
      }
    },

    carbon: {
      type: 'object',
      properties: {
        emissions: { type: 'number', description: '탄소 배출량 (kgCO2e)' },
        emissionFactor: { type: 'number', description: '배출 계수' },
        scope: {
          type: 'string',
          enum: ['scope1', 'scope2_location', 'scope2_market']
        }
      }
    }
  }
};

// 에너지 데이터 시계열 스키마
const EnergyTimeSeriesSchema = {
  $schema: 'https://json-schema.org/draft/2020-12/schema',
  $id: 'https://wia.org/schemas/bems/energy-timeseries.json',
  title: 'Energy Time Series Data',
  type: 'object',
  required: ['meterId', 'startTime', 'endTime', 'interval', 'data'],

  properties: {
    meterId: { type: 'string' },
    startTime: { type: 'string', format: 'date-time' },
    endTime: { type: 'string', format: 'date-time' },
    interval: { type: 'integer' },
    unit: { type: 'string' },
    aggregation: {
      type: 'string',
      enum: ['raw', 'sum', 'average', 'min', 'max', 'delta']
    },
    data: {
      type: 'array',
      items: {
        type: 'object',
        properties: {
          timestamp: { type: 'string', format: 'date-time' },
          value: { type: 'number' },
          quality: { type: 'string' }
        }
      }
    }
  }
};
```

### 3.2.3 환경 센서 데이터 스키마

```typescript
// 환경 데이터 JSON 스키마
const EnvironmentalDataSchema = {
  $schema: 'https://json-schema.org/draft/2020-12/schema',
  $id: 'https://wia.org/schemas/bems/environmental-data.json',
  title: 'WIA-BEMS Environmental Data Schema',
  description: '환경 센서 데이터를 정의하는 스키마',
  type: 'object',
  required: ['sensorId', 'timestamp', 'measurements'],

  properties: {
    sensorId: {
      type: 'string',
      pattern: '^SNS-[A-Z0-9]{8}$',
      description: '센서 고유 식별자'
    },

    buildingId: { type: 'string' },

    location: {
      type: 'object',
      properties: {
        type: {
          type: 'string',
          enum: ['indoor', 'outdoor', 'duct', 'pipe']
        },
        floor: { type: 'string' },
        zone: { type: 'string' },
        room: { type: 'string' },
        equipment: { type: 'string' },
        coordinates: {
          type: 'object',
          properties: {
            x: { type: 'number' },
            y: { type: 'number' },
            z: { type: 'number' }
          }
        }
      }
    },

    timestamp: {
      type: 'string',
      format: 'date-time'
    },

    measurements: {
      type: 'object',
      properties: {
        // 온열 환경
        temperature: {
          type: 'object',
          properties: {
            value: { type: 'number', description: '온도 (°C)' },
            setpoint: { type: 'number' },
            dewPoint: { type: 'number' },
            wetBulb: { type: 'number' },
            operative: { type: 'number', description: '작용 온도' },
            meanRadiant: { type: 'number', description: '평균 복사 온도' }
          }
        },
        humidity: {
          type: 'object',
          properties: {
            relative: { type: 'number', minimum: 0, maximum: 100, description: '상대습도 (%)' },
            absolute: { type: 'number', description: '절대습도 (g/kg)' },
            setpoint: { type: 'number' }
          }
        },

        // 공기질
        co2: {
          type: 'object',
          properties: {
            value: { type: 'number', description: 'CO2 농도 (ppm)' },
            level: {
              type: 'string',
              enum: ['excellent', 'good', 'fair', 'poor', 'hazardous']
            }
          }
        },
        co: {
          type: 'number',
          description: 'CO 농도 (ppm)'
        },
        pm: {
          type: 'object',
          properties: {
            pm1: { type: 'number', description: 'PM1.0 (μg/㎥)' },
            pm25: { type: 'number', description: 'PM2.5 (μg/㎥)' },
            pm10: { type: 'number', description: 'PM10 (μg/㎥)' },
            aqi: { type: 'integer', description: '대기질 지수' }
          }
        },
        voc: {
          type: 'object',
          properties: {
            tvoc: { type: 'number', description: '총 VOC (ppb)' },
            formaldehyde: { type: 'number', description: '포름알데히드 (ppb)' }
          }
        },
        ozone: { type: 'number', description: '오존 (ppb)' },
        radon: { type: 'number', description: '라돈 (Bq/㎥)' },

        // 조명
        illuminance: {
          type: 'object',
          properties: {
            value: { type: 'number', description: '조도 (lux)' },
            setpoint: { type: 'number' },
            daylight: { type: 'number', description: '주광 조도' }
          }
        },
        colorTemperature: {
          type: 'number',
          description: '색온도 (K)'
        },

        // 음환경
        noise: {
          type: 'object',
          properties: {
            level: { type: 'number', description: '소음 레벨 (dBA)' },
            peak: { type: 'number' },
            nc: { type: 'integer', description: 'NC 등급' }
          }
        },

        // 압력
        pressure: {
          type: 'object',
          properties: {
            atmospheric: { type: 'number', description: '대기압 (hPa)' },
            differential: { type: 'number', description: '차압 (Pa)' }
          }
        },

        // 기류
        airVelocity: {
          type: 'number',
          description: '기류 속도 (m/s)'
        }
      }
    },

    comfortIndices: {
      type: 'object',
      properties: {
        pmv: {
          type: 'number',
          minimum: -3,
          maximum: 3,
          description: '예상 평균 투표 (PMV)'
        },
        ppd: {
          type: 'number',
          minimum: 0,
          maximum: 100,
          description: '예상 불만족율 (PPD %)'
        },
        adaptiveComfort: {
          type: 'object',
          properties: {
            acceptableRange: {
              type: 'object',
              properties: {
                lower: { type: 'number' },
                upper: { type: 'number' }
              }
            },
            deviation: { type: 'number' }
          }
        },
        iaq: {
          type: 'object',
          properties: {
            index: { type: 'number', minimum: 0, maximum: 100 },
            level: {
              type: 'string',
              enum: ['excellent', 'good', 'fair', 'poor', 'hazardous']
            }
          }
        }
      }
    },

    quality: {
      type: 'object',
      properties: {
        status: { type: 'string' },
        batteryLevel: { type: 'number' },
        signalStrength: { type: 'number' },
        lastCalibration: { type: 'string', format: 'date' }
      }
    }
  }
};

// 환경 데이터 예시
const environmentalDataExample = {
  sensorId: 'SNS-ENV12345',
  buildingId: 'BLD-ABCD1234',
  location: {
    type: 'indoor',
    floor: '15F',
    zone: 'ZONE-A',
    room: '회의실 1501'
  },
  timestamp: '2024-11-15T14:30:00+09:00',

  measurements: {
    temperature: {
      value: 23.5,
      setpoint: 24.0,
      dewPoint: 12.8,
      operative: 23.8
    },
    humidity: {
      relative: 48,
      absolute: 8.2,
      setpoint: 50
    },
    co2: {
      value: 650,
      level: 'good'
    },
    pm: {
      pm25: 12,
      pm10: 25,
      aqi: 45
    },
    voc: {
      tvoc: 180
    },
    illuminance: {
      value: 520,
      setpoint: 500,
      daylight: 280
    },
    noise: {
      level: 42,
      nc: 35
    }
  },

  comfortIndices: {
    pmv: 0.2,
    ppd: 6,
    iaq: {
      index: 85,
      level: 'good'
    }
  },

  quality: {
    status: 'valid',
    batteryLevel: 78,
    signalStrength: -65
  }
};
```

---

## 3.3 설비 및 재실 데이터 스키마

### 3.3.1 설비 상태 데이터

```typescript
// 설비 데이터 스키마
const EquipmentDataSchema = {
  $schema: 'https://json-schema.org/draft/2020-12/schema',
  $id: 'https://wia.org/schemas/bems/equipment-data.json',
  title: 'WIA-BEMS Equipment Data Schema',
  type: 'object',
  required: ['equipmentId', 'equipmentType', 'timestamp'],

  properties: {
    equipmentId: {
      type: 'string',
      pattern: '^EQP-[A-Z0-9]{8}$'
    },

    equipmentType: {
      type: 'string',
      enum: [
        'AHU', 'RTU', 'FCU', 'VAV', 'CAHU',
        'chiller', 'boiler', 'heat_pump', 'cooling_tower',
        'pump', 'fan', 'compressor',
        'transformer', 'ups', 'generator',
        'elevator', 'escalator'
      ]
    },

    timestamp: { type: 'string', format: 'date-time' },

    status: {
      type: 'object',
      properties: {
        operatingMode: {
          type: 'string',
          enum: ['auto', 'manual', 'off', 'standby', 'test', 'emergency']
        },
        runStatus: {
          type: 'string',
          enum: ['running', 'stopped', 'starting', 'stopping', 'fault']
        },
        enabled: { type: 'boolean' },
        occupied: { type: 'boolean' },
        alarmActive: { type: 'boolean' },
        maintenanceMode: { type: 'boolean' }
      }
    },

    // AHU/RTU 특화 데이터
    ahuData: {
      type: 'object',
      properties: {
        supplyAirTemperature: { type: 'number' },
        supplyAirTemperatureSetpoint: { type: 'number' },
        returnAirTemperature: { type: 'number' },
        mixedAirTemperature: { type: 'number' },
        outsideAirTemperature: { type: 'number' },
        supplyAirHumidity: { type: 'number' },
        returnAirHumidity: { type: 'number' },
        staticPressure: { type: 'number' },
        staticPressureSetpoint: { type: 'number' },
        supplyFanSpeed: { type: 'number' },
        returnFanSpeed: { type: 'number' },
        supplyFanStatus: { type: 'boolean' },
        returnFanStatus: { type: 'boolean' },
        outsideAirDamperPosition: { type: 'number' },
        returnAirDamperPosition: { type: 'number' },
        exhaustAirDamperPosition: { type: 'number' },
        coolingValvePosition: { type: 'number' },
        heatingValvePosition: { type: 'number' },
        filterDifferentialPressure: { type: 'number' },
        economizer: {
          type: 'object',
          properties: {
            enabled: { type: 'boolean' },
            mode: { type: 'string', enum: ['full', 'partial', 'minimum'] }
          }
        },
        airflow: {
          type: 'object',
          properties: {
            supply: { type: 'number' },
            return: { type: 'number' },
            outsideAir: { type: 'number' },
            exhaust: { type: 'number' }
          }
        }
      }
    },

    // VAV 특화 데이터
    vavData: {
      type: 'object',
      properties: {
        zoneTemperature: { type: 'number' },
        zoneTemperatureSetpoint: { type: 'number' },
        coolingSetpoint: { type: 'number' },
        heatingSetpoint: { type: 'number' },
        damperPosition: { type: 'number' },
        airflowRate: { type: 'number' },
        airflowSetpoint: { type: 'number' },
        minAirflow: { type: 'number' },
        maxAirflow: { type: 'number' },
        reheatValvePosition: { type: 'number' },
        dischargeAirTemperature: { type: 'number' },
        occupancyStatus: { type: 'boolean' },
        co2Level: { type: 'number' }
      }
    },

    // 냉동기 특화 데이터
    chillerData: {
      type: 'object',
      properties: {
        chilledWaterSupplyTemperature: { type: 'number' },
        chilledWaterSupplySetpoint: { type: 'number' },
        chilledWaterReturnTemperature: { type: 'number' },
        chilledWaterFlowRate: { type: 'number' },
        condenserWaterSupplyTemperature: { type: 'number' },
        condenserWaterReturnTemperature: { type: 'number' },
        condenserWaterFlowRate: { type: 'number' },
        compressorSpeed: { type: 'number' },
        loadPercentage: { type: 'number' },
        powerConsumption: { type: 'number' },
        cop: { type: 'number' },
        refrigerantPressure: {
          type: 'object',
          properties: {
            suction: { type: 'number' },
            discharge: { type: 'number' }
          }
        },
        oilPressure: { type: 'number' },
        oilTemperature: { type: 'number' },
        runHours: { type: 'number' },
        startCount: { type: 'integer' }
      }
    },

    // 보일러 특화 데이터
    boilerData: {
      type: 'object',
      properties: {
        hotWaterSupplyTemperature: { type: 'number' },
        hotWaterSupplySetpoint: { type: 'number' },
        hotWaterReturnTemperature: { type: 'number' },
        hotWaterFlowRate: { type: 'number' },
        firingRate: { type: 'number' },
        fuelConsumption: { type: 'number' },
        flueGasTemperature: { type: 'number' },
        oxygenLevel: { type: 'number' },
        efficiency: { type: 'number' },
        steamPressure: { type: 'number' },
        waterLevel: { type: 'number' }
      }
    },

    // 펌프 특화 데이터
    pumpData: {
      type: 'object',
      properties: {
        speed: { type: 'number' },
        flowRate: { type: 'number' },
        differentialPressure: { type: 'number' },
        suctionPressure: { type: 'number' },
        dischargePressure: { type: 'number' },
        powerConsumption: { type: 'number' },
        motorTemperature: { type: 'number' },
        vibration: { type: 'number' },
        runHours: { type: 'number' }
      }
    },

    alarms: {
      type: 'array',
      items: {
        type: 'object',
        properties: {
          code: { type: 'string' },
          message: { type: 'string' },
          severity: { type: 'string', enum: ['info', 'warning', 'alarm', 'critical'] },
          timestamp: { type: 'string', format: 'date-time' },
          acknowledged: { type: 'boolean' }
        }
      }
    },

    efficiency: {
      type: 'object',
      properties: {
        current: { type: 'number' },
        rated: { type: 'number' },
        target: { type: 'number' },
        trend: { type: 'string', enum: ['improving', 'stable', 'degrading'] }
      }
    }
  }
};
```

### 3.3.2 재실 데이터 스키마

```typescript
// 재실 데이터 스키마
const OccupancyDataSchema = {
  $schema: 'https://json-schema.org/draft/2020-12/schema',
  $id: 'https://wia.org/schemas/bems/occupancy-data.json',
  title: 'WIA-BEMS Occupancy Data Schema',
  type: 'object',
  required: ['zoneId', 'timestamp'],

  properties: {
    zoneId: {
      type: 'string',
      pattern: '^ZON-[A-Z0-9]{8}$'
    },

    buildingId: { type: 'string' },

    location: {
      type: 'object',
      properties: {
        floor: { type: 'string' },
        area: { type: 'string' },
        room: { type: 'string' },
        spaceType: {
          type: 'string',
          enum: ['office', 'meeting_room', 'lobby', 'cafeteria', 'restroom', 'corridor', 'parking']
        }
      }
    },

    timestamp: { type: 'string', format: 'date-time' },

    occupancy: {
      type: 'object',
      properties: {
        isOccupied: {
          type: 'boolean',
          description: '재실 여부'
        },
        occupantCount: {
          type: 'integer',
          minimum: 0,
          description: '재실 인원수'
        },
        maxCapacity: {
          type: 'integer',
          description: '최대 수용 인원'
        },
        utilizationPercent: {
          type: 'number',
          minimum: 0,
          maximum: 100,
          description: '공간 활용률 (%)'
        },
        density: {
          type: 'number',
          description: '재실 밀도 (인/㎡)'
        }
      }
    },

    schedule: {
      type: 'object',
      properties: {
        scheduledOccupancy: {
          type: 'string',
          enum: ['occupied', 'unoccupied', 'warmup', 'cooldown']
        },
        scheduleOverride: {
          type: 'boolean'
        },
        nextScheduleChange: {
          type: 'string',
          format: 'date-time'
        }
      }
    },

    detection: {
      type: 'object',
      properties: {
        method: {
          type: 'string',
          enum: ['pir', 'ultrasonic', 'camera', 'wifi', 'badge', 'co2_inference', 'schedule']
        },
        confidence: {
          type: 'number',
          minimum: 0,
          maximum: 1
        },
        lastDetectionTime: {
          type: 'string',
          format: 'date-time'
        },
        dwellTime: {
          type: 'number',
          description: '체류 시간 (분)'
        }
      }
    },

    analytics: {
      type: 'object',
      properties: {
        averageOccupancy: {
          type: 'object',
          properties: {
            hourly: { type: 'number' },
            daily: { type: 'number' },
            weekly: { type: 'number' }
          }
        },
        peakOccupancy: {
          type: 'object',
          properties: {
            count: { type: 'integer' },
            timestamp: { type: 'string', format: 'date-time' }
          }
        },
        occupancyPattern: {
          type: 'string',
          enum: ['regular', 'irregular', 'sparse', 'dense']
        }
      }
    },

    privacyCompliance: {
      type: 'object',
      properties: {
        dataAnonymized: { type: 'boolean' },
        retentionPeriod: { type: 'string' },
        consentObtained: { type: 'boolean' }
      }
    }
  }
};
```

---

## 3.4 데이터 품질 관리

### 3.4.1 데이터 검증 프레임워크

```typescript
// 데이터 품질 관리 시스템
interface DataQualityFramework {
  dimensions: {
    accuracy: {
      name: '정확성';
      description: '데이터가 실제 값을 정확히 반영하는 정도';
      metrics: ['절대 오차', '상대 오차', 'RMSE'];
      validationMethods: [
        '센서 교정 확인',
        '참조 측정과 비교',
        '물리적 범위 검증'
      ];
    };
    completeness: {
      name: '완전성';
      description: '필수 데이터의 존재 여부';
      metrics: ['결측률', '필수 필드 존재율'];
      thresholds: {
        excellent: '> 99%',
        good: '95-99%',
        fair: '90-95%',
        poor: '< 90%'
      };
    };
    timeliness: {
      name: '적시성';
      description: '데이터가 적시에 수집되고 사용 가능한 정도';
      metrics: ['데이터 지연', '수집 주기 준수율'];
      requirements: {
        realtime: '< 1분',
        nearRealtime: '< 15분',
        batch: '< 24시간'
      };
    };
    consistency: {
      name: '일관성';
      description: '데이터 간 모순이 없는 정도';
      validationRules: [
        '시계열 연속성',
        '관련 포인트 간 상관관계',
        '에너지 균형'
      ];
    };
    validity: {
      name: '유효성';
      description: '데이터가 정의된 형식과 규칙을 준수하는 정도';
      validations: [
        '스키마 검증',
        '범위 검증',
        '형식 검증',
        '비즈니스 규칙'
      ];
    };
  };
}

// 데이터 검증 엔진
class DataValidationEngine {
  private schemas: Map<string, JSONSchema>;
  private rules: ValidationRule[];

  async validateData(data: any, schemaId: string): Promise<ValidationResult> {
    const results: ValidationIssue[] = [];

    // 1. 스키마 검증
    const schemaValidation = await this.validateSchema(data, schemaId);
    if (!schemaValidation.valid) {
      results.push(...schemaValidation.issues);
    }

    // 2. 범위 검증
    const rangeValidation = this.validateRanges(data);
    results.push(...rangeValidation);

    // 3. 물리적 일관성 검증
    const consistencyValidation = this.validateConsistency(data);
    results.push(...consistencyValidation);

    // 4. 시계열 검증
    const timeSeriesValidation = this.validateTimeSeries(data);
    results.push(...timeSeriesValidation);

    // 5. 비즈니스 규칙 검증
    const businessValidation = this.validateBusinessRules(data);
    results.push(...businessValidation);

    // 품질 점수 계산
    const qualityScore = this.calculateQualityScore(results);

    return {
      valid: results.filter(r => r.severity === 'error').length === 0,
      qualityScore,
      issues: results,
      summary: this.generateSummary(results)
    };
  }

  private validateRanges(data: any): ValidationIssue[] {
    const issues: ValidationIssue[] = [];

    // 온도 범위 검증
    if (data.measurements?.temperature?.value !== undefined) {
      const temp = data.measurements.temperature.value;
      if (temp < -50 || temp > 80) {
        issues.push({
          field: 'temperature',
          value: temp,
          severity: 'error',
          message: `온도 ${temp}°C가 유효 범위(-50~80)를 벗어남`,
          rule: 'range_check'
        });
      } else if (temp < 10 || temp > 40) {
        issues.push({
          field: 'temperature',
          value: temp,
          severity: 'warning',
          message: `온도 ${temp}°C가 일반적 범위(10~40)를 벗어남`,
          rule: 'soft_range_check'
        });
      }
    }

    // CO2 범위 검증
    if (data.measurements?.co2?.value !== undefined) {
      const co2 = data.measurements.co2.value;
      if (co2 < 350 || co2 > 10000) {
        issues.push({
          field: 'co2',
          value: co2,
          severity: 'error',
          message: `CO2 ${co2}ppm이 유효 범위(350~10000)를 벗어남`,
          rule: 'range_check'
        });
      }
    }

    // 습도 범위 검증
    if (data.measurements?.humidity?.relative !== undefined) {
      const rh = data.measurements.humidity.relative;
      if (rh < 0 || rh > 100) {
        issues.push({
          field: 'relative_humidity',
          value: rh,
          severity: 'error',
          message: `상대습도 ${rh}%가 유효 범위(0~100)를 벗어남`,
          rule: 'range_check'
        });
      }
    }

    // 전력 검증
    if (data.readings?.activePower !== undefined) {
      const power = data.readings.activePower;
      if (power < 0) {
        // 역송전 가능성 확인
        if (data.meterType !== 'solar_production' && data.meterType !== 'ess_discharge') {
          issues.push({
            field: 'activePower',
            value: power,
            severity: 'warning',
            message: `음수 전력(${power}kW)이 감지됨. 역송전 확인 필요`,
            rule: 'negative_power_check'
          });
        }
      }
    }

    return issues;
  }

  private validateConsistency(data: any): ValidationIssue[] {
    const issues: ValidationIssue[] = [];

    // AHU 데이터 일관성 검증
    if (data.ahuData) {
      const ahu = data.ahuData;

      // 혼합공기 온도 검증 (OA-MA-RA 관계)
      if (ahu.outsideAirTemperature && ahu.mixedAirTemperature && ahu.returnAirTemperature) {
        const oat = ahu.outsideAirTemperature;
        const mat = ahu.mixedAirTemperature;
        const rat = ahu.returnAirTemperature;

        const minExpected = Math.min(oat, rat) - 2;
        const maxExpected = Math.max(oat, rat) + 2;

        if (mat < minExpected || mat > maxExpected) {
          issues.push({
            field: 'mixedAirTemperature',
            value: mat,
            severity: 'warning',
            message: `혼합공기 온도(${mat}°C)가 OAT(${oat}°C)와 RAT(${rat}°C) 사이에 없음`,
            rule: 'ma_temp_consistency'
          });
        }
      }

      // 공기량 균형 검증
      if (ahu.airflow) {
        const supply = ahu.airflow.supply || 0;
        const oa = ahu.airflow.outsideAir || 0;
        const returnAir = ahu.airflow.return || 0;
        const exhaust = ahu.airflow.exhaust || 0;

        const balance = supply - (oa + returnAir);
        if (Math.abs(balance) > supply * 0.1) {
          issues.push({
            field: 'airflow_balance',
            value: balance,
            severity: 'warning',
            message: `공기량 불균형 감지. SA=${supply}, OA=${oa}, RA=${returnAir}`,
            rule: 'airflow_balance'
          });
        }
      }
    }

    // 냉동기 데이터 일관성 검증
    if (data.chillerData) {
      const chiller = data.chillerData;

      // 냉동기 운전 중 COP 검증
      if (chiller.cop && chiller.loadPercentage > 10) {
        if (chiller.cop < 2 || chiller.cop > 10) {
          issues.push({
            field: 'cop',
            value: chiller.cop,
            severity: 'warning',
            message: `COP(${chiller.cop})가 일반적 범위(2~10)를 벗어남`,
            rule: 'cop_range'
          });
        }
      }
    }

    return issues;
  }

  private calculateQualityScore(issues: ValidationIssue[]): number {
    const weights = {
      error: 10,
      warning: 3,
      info: 1
    };

    let deductions = 0;
    for (const issue of issues) {
      deductions += weights[issue.severity] || 0;
    }

    return Math.max(0, 100 - deductions);
  }
}

// 결측치 처리 엔진
class MissingDataHandler {
  handleMissingData(
    data: TimeSeriesData,
    method: InterpolationMethod
  ): TimeSeriesData {
    switch (method) {
      case 'linear':
        return this.linearInterpolation(data);
      case 'cubic':
        return this.cubicInterpolation(data);
      case 'forward_fill':
        return this.forwardFill(data);
      case 'backward_fill':
        return this.backwardFill(data);
      case 'mean':
        return this.meanImputation(data);
      case 'seasonal':
        return this.seasonalImputation(data);
      default:
        return data;
    }
  }

  private linearInterpolation(data: TimeSeriesData): TimeSeriesData {
    const result = [...data.values];

    for (let i = 0; i < result.length; i++) {
      if (result[i].value === null || result[i].value === undefined) {
        // 앞뒤 유효한 값 찾기
        let prevIdx = i - 1;
        let nextIdx = i + 1;

        while (prevIdx >= 0 && (result[prevIdx].value === null || result[prevIdx].value === undefined)) {
          prevIdx--;
        }
        while (nextIdx < result.length && (result[nextIdx].value === null || result[nextIdx].value === undefined)) {
          nextIdx++;
        }

        if (prevIdx >= 0 && nextIdx < result.length) {
          // 선형 보간
          const ratio = (i - prevIdx) / (nextIdx - prevIdx);
          result[i] = {
            ...result[i],
            value: result[prevIdx].value + ratio * (result[nextIdx].value - result[prevIdx].value),
            quality: 'interpolated'
          };
        }
      }
    }

    return { ...data, values: result };
  }

  private seasonalImputation(data: TimeSeriesData): TimeSeriesData {
    // 계절성을 고려한 결측치 대체
    const seasonalPattern = this.extractSeasonalPattern(data);
    const result = [...data.values];

    for (let i = 0; i < result.length; i++) {
      if (result[i].value === null || result[i].value === undefined) {
        const timestamp = new Date(result[i].timestamp);
        const seasonalIndex = this.getSeasonalIndex(timestamp, data.interval);
        result[i] = {
          ...result[i],
          value: seasonalPattern[seasonalIndex],
          quality: 'seasonal_imputed'
        };
      }
    }

    return { ...data, values: result };
  }

  private extractSeasonalPattern(data: TimeSeriesData): number[] {
    // 시간대별/요일별 평균 패턴 추출
    const pattern: { [key: number]: number[] } = {};

    for (const point of data.values) {
      if (point.value !== null && point.value !== undefined) {
        const timestamp = new Date(point.timestamp);
        const index = this.getSeasonalIndex(timestamp, data.interval);

        if (!pattern[index]) {
          pattern[index] = [];
        }
        pattern[index].push(point.value);
      }
    }

    // 평균 계산
    const avgPattern: number[] = [];
    for (const index in pattern) {
      const values = pattern[index];
      avgPattern[parseInt(index)] = values.reduce((a, b) => a + b, 0) / values.length;
    }

    return avgPattern;
  }

  private getSeasonalIndex(timestamp: Date, interval: number): number {
    // 15분 간격 기준 하루 96개 인덱스
    const minutesFromMidnight = timestamp.getHours() * 60 + timestamp.getMinutes();
    return Math.floor(minutesFromMidnight / interval);
  }
}
```

---

## 3.5 장 요약

### 핵심 데이터 스키마 요약

| 스키마 | 용도 | 핵심 필드 |
|--------|------|-----------|
| Building | 건물 정보 | 위치, 면적, 용도, 인증, 에너지 프로파일 |
| Energy Data | 에너지 계측 | 전력량, 수요, 역률, 비용, 탄소 |
| Environmental | 환경 센서 | 온도, 습도, CO2, PM, VOC, 조도 |
| Equipment | 설비 상태 | AHU, VAV, 냉동기, 보일러 운전 데이터 |
| Occupancy | 재실 정보 | 인원수, 활용률, 체류 시간, 패턴 |

### 데이터 품질 관리 핵심

- **정확성**: 물리적 범위 검증, 센서 교정
- **완전성**: 결측률 99% 이상 목표
- **적시성**: 실시간 데이터 1분 이내
- **일관성**: 에너지 균형, 물리적 관계 검증
- **유효성**: JSON 스키마 검증, 비즈니스 규칙

### 다음 장 미리보기

제4장에서는 WIA-BEMS Phase 2인 API 인터페이스에 대해 다룹니다. RESTful API 설계, 인증/인가, WebSocket 실시간 스트리밍, SDK 개발 방법을 학습합니다.

---

© 2025 World Certification Industry Association (WIA)
弘益人間 (홍익인간) - 널리 인간을 이롭게 한다
