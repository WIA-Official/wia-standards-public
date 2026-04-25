# 제4장: 데이터 형식 및 구조

## 가뭄 모니터링 데이터를 위한 표준화된 스키마

---

## 4.1 Palmer 가뭄 심각도 지수(PDSI) 데이터 형식

### PDSI 개요

Palmer 가뭄 심각도 지수는 전 세계적으로 가장 널리 사용되는 가뭄 지수 중 하나로, 2계층 토양 물 수지 모델을 기반으로 수분 상태의 표준화된 척도를 제공합니다. WIA-ENV-003 표준은 PDSI 값, 메타데이터 및 품질 정보에 대한 포괄적인 데이터 형식을 지정합니다.

### PDSI 데이터 스키마

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA-ENV-003 PDSI 데이터 스키마",
  "type": "object",
  "required": ["header", "data"],
  "properties": {
    "header": {
      "type": "object",
      "required": ["standard_id", "version", "index_type", "generation_time"],
      "properties": {
        "standard_id": {
          "type": "string",
          "const": "WIA-ENV-003"
        },
        "version": {
          "type": "string",
          "pattern": "^\\d+\\.\\d+\\.\\d+$"
        },
        "index_type": {
          "type": "string",
          "enum": ["PDSI", "SC-PDSI", "PHDI", "PMDI"]
        },
        "generation_time": {
          "type": "string",
          "format": "date-time"
        },
        "source_institution": {
          "type": "string"
        },
        "spatial_coverage": {
          "type": "object",
          "properties": {
            "type": {"type": "string", "enum": ["global", "regional", "national", "local"]},
            "bounds": {
              "type": "object",
              "properties": {
                "north": {"type": "number"},
                "south": {"type": "number"},
                "east": {"type": "number"},
                "west": {"type": "number"}
              }
            }
          }
        }
      }
    },
    "data": {
      "type": "array",
      "items": {
        "$ref": "#/definitions/pdsi_record"
      }
    }
  }
}
```

### PDSI 분류 참조 표

| PDSI 값 | 등급 | 분류 | 설명 |
|--------|------|------|------|
| ≥ 4.0 | W4 | 극심한 습윤 | 주요 홍수 조건 가능 |
| 3.0 ~ 3.99 | W3 | 매우 습윤 | 상당한 수분 잉여 |
| 2.0 ~ 2.99 | W2 | 적당히 습윤 | 평년 이상 수분 |
| 1.0 ~ 1.99 | W1 | 약간 습윤 | 경미한 수분 잉여 |
| -0.99 ~ 0.99 | N | 거의 정상 | 정상 수분 상태 |
| -1.0 ~ -1.99 | D0 | 비정상적 건조 | 단기 건조 |
| -2.0 ~ -2.99 | D1 | 중간 가뭄 | 일부 작물 피해 |
| -3.0 ~ -3.99 | D2 | 심한 가뭄 | 작물 손실 가능 |
| -4.0 ~ -4.99 | D3 | 극심한 가뭄 | 주요 작물 손실 |
| ≤ -5.0 | D4 | 예외적 가뭄 | 예외적 손실 |

### 한국 맥락의 PDSI 데이터 예시

```json
{
  "header": {
    "standard_id": "WIA-ENV-003",
    "version": "1.0.0",
    "index_type": "SC-PDSI",
    "generation_time": "2025-01-15T12:00:00Z",
    "source_institution": "기상청",
    "spatial_coverage": {
      "type": "national",
      "bounds": {
        "north": 38.6,
        "south": 33.1,
        "east": 131.9,
        "west": 124.6
      }
    }
  },
  "data": [
    {
      "location": {
        "location_id": "KR-CB-001",
        "name": "충청북도",
        "coordinates": {
          "latitude": 36.6,
          "longitude": 127.5
        },
        "administrative": {
          "country": "대한민국",
          "state_province": "충청북도",
          "county": null
        }
      },
      "period": {
        "year": 2025,
        "month": 1,
        "start_date": "2025-01-01",
        "end_date": "2025-01-31"
      },
      "value": -2.35,
      "classification": {
        "category": "D1",
        "description": "중간 가뭄"
      },
      "quality": {
        "flag": 0,
        "confidence": 0.93,
        "missing_inputs": []
      }
    }
  ]
}
```

---

## 4.2 표준화 강수지수(SPI) 스키마

### SPI 개요

표준화 강수지수는 여러 시간 척도에 걸쳐 강수량 이상을 정량화하여 다양한 가뭄 유형을 모니터링하는 데 유연하게 사용됩니다. WIA-ENV-003 표준은 1개월에서 48개월까지의 시간 척도를 지원하는 SPI 형식을 지정합니다.

### SPI 데이터 스키마

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA-ENV-003 SPI 데이터 스키마",
  "type": "object",
  "required": ["header", "data"],
  "properties": {
    "header": {
      "type": "object",
      "required": ["standard_id", "version", "index_type", "time_scale"],
      "properties": {
        "standard_id": {"const": "WIA-ENV-003"},
        "version": {"type": "string"},
        "index_type": {
          "type": "string",
          "enum": ["SPI", "SPEI"]
        },
        "time_scale": {
          "type": "object",
          "properties": {
            "value": {"type": "integer", "minimum": 1, "maximum": 48},
            "unit": {"type": "string", "enum": ["month", "week"]}
          }
        },
        "distribution": {
          "type": "string",
          "enum": ["gamma", "pearson-III", "log-logistic"]
        },
        "baseline_period": {
          "type": "object",
          "properties": {
            "start_year": {"type": "integer"},
            "end_year": {"type": "integer"}
          }
        }
      }
    }
  }
}
```

### SPI 분류 표

| SPI 값 | 등급 | 확률 | 설명 |
|-------|------|------|------|
| ≥ 2.0 | 극심한 습윤 | 2.3% | 예외적인 수분 잉여 |
| 1.5 ~ 1.99 | 매우 습윤 | 4.4% | 상당한 잉여 |
| 1.0 ~ 1.49 | 적당히 습윤 | 9.2% | 평년 이상 |
| -0.99 ~ 0.99 | 거의 정상 | 68.3% | 정상 상태 |
| -1.0 ~ -1.49 | 적당히 건조 | 9.2% | 평년 이하 |
| -1.5 ~ -1.99 | 심하게 건조 | 4.4% | 상당한 부족 |
| ≤ -2.0 | 극심하게 건조 | 2.3% | 예외적 부족 |

### 다중 시간 척도 SPI 예시

```json
{
  "header": {
    "standard_id": "WIA-ENV-003",
    "version": "1.0.0",
    "index_type": "SPI",
    "distribution": "gamma",
    "baseline_period": {
      "start_year": 1991,
      "end_year": 2020
    }
  },
  "data": [
    {
      "location": {
        "location_id": "KR-SEOUL",
        "coordinates": {"latitude": 37.5665, "longitude": 126.9780}
      },
      "multi_scale_values": [
        {
          "time_scale_months": 1,
          "value": 0.35,
          "percentile": 63.7,
          "classification": {"category": "거의 정상"}
        },
        {
          "time_scale_months": 3,
          "value": -1.15,
          "percentile": 12.5,
          "classification": {"category": "적당히 건조"}
        },
        {
          "time_scale_months": 6,
          "value": -1.72,
          "percentile": 4.3,
          "classification": {"category": "심하게 건조"}
        },
        {
          "time_scale_months": 12,
          "value": -2.05,
          "percentile": 2.0,
          "classification": {"category": "극심하게 건조"}
        }
      ],
      "period": {
        "end_date": "2025-01-31"
      }
    }
  ]
}
```

---

## 4.3 토양 수분 데이터 표현

### 토양 수분 변수

토양 수분은 여러 방식으로 표현될 수 있습니다. WIA-ENV-003 표준은 모든 일반적인 표현을 지원합니다:

| 변수 | 단위 | 설명 | 일반적인 범위 |
|------|------|------|-------------|
| 체적 함수율 | m³/m³ | 토양 부피당 물 부피 | 0.05-0.50 |
| 포화도 | 분율 | 채워진 공극 공간의 비율 | 0.0-1.0 |
| 식물 이용 가능 수분 | mm | 식물이 이용할 수 있는 물 | 0-200 mm |
| 토양 수분 백분위 | % | 이력 대비 백분위 | 0-100 |
| 토양 수분 이상 | mm | 평년에서의 편차 | -100 ~ +100 |

### 토양 수분 데이터 스키마

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA-ENV-003 토양 수분 스키마",
  "type": "object",
  "required": ["header", "data"],
  "properties": {
    "header": {
      "type": "object",
      "properties": {
        "standard_id": {"const": "WIA-ENV-003"},
        "version": {"type": "string"},
        "data_source": {
          "type": "string",
          "enum": ["in_situ", "satellite", "model", "blended"]
        },
        "soil_model": {
          "type": "string",
          "description": "토양 모델 또는 센서 유형"
        }
      }
    },
    "data": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "location": {
            "type": "object",
            "properties": {
              "location_id": {"type": "string"},
              "coordinates": {
                "type": "object",
                "properties": {
                  "latitude": {"type": "number"},
                  "longitude": {"type": "number"}
                }
              },
              "elevation_m": {"type": "number"}
            }
          },
          "timestamp": {
            "type": "string",
            "format": "date-time"
          },
          "soil_layers": {
            "type": "array",
            "items": {
              "type": "object",
              "properties": {
                "layer_id": {"type": "string"},
                "depth_top_cm": {"type": "number"},
                "depth_bottom_cm": {"type": "number"},
                "volumetric_water_content": {"type": "number"},
                "percentile": {"type": "number"},
                "anomaly_mm": {"type": "number"},
                "quality_flag": {"type": "integer"}
              }
            }
          },
          "integrated_column": {
            "type": "object",
            "properties": {
              "total_depth_cm": {"type": "number"},
              "total_water_mm": {"type": "number"},
              "percentile": {"type": "number"},
              "drought_category": {"type": "string"}
            }
          }
        }
      }
    }
  }
}
```

### 토양 수분 기록 예시

```json
{
  "header": {
    "standard_id": "WIA-ENV-003",
    "version": "1.0.0",
    "data_source": "blended",
    "soil_model": "농촌진흥청 토양수분 모델"
  },
  "data": [
    {
      "location": {
        "location_id": "KR-AGR-2001",
        "coordinates": {
          "latitude": 36.5892,
          "longitude": 127.3845
        },
        "elevation_m": 85
      },
      "timestamp": "2025-01-15T06:00:00Z",
      "soil_layers": [
        {
          "layer_id": "0-10cm",
          "depth_top_cm": 0,
          "depth_bottom_cm": 10,
          "volumetric_water_content": 0.16,
          "percentile": 18,
          "anomaly_mm": -6.5,
          "quality_flag": 0
        },
        {
          "layer_id": "10-30cm",
          "depth_top_cm": 10,
          "depth_bottom_cm": 30,
          "volumetric_water_content": 0.22,
          "percentile": 24,
          "anomaly_mm": -12.3,
          "quality_flag": 0
        },
        {
          "layer_id": "30-100cm",
          "depth_top_cm": 30,
          "depth_bottom_cm": 100,
          "volumetric_water_content": 0.26,
          "percentile": 28,
          "anomaly_mm": -18.5,
          "quality_flag": 0
        }
      ],
      "integrated_column": {
        "total_depth_cm": 100,
        "total_water_mm": 235,
        "percentile": 22,
        "drought_category": "D1"
      }
    }
  ]
}
```

---

## 4.4 정규화 식생지수(NDVI) 형식

### NDVI 개요

NDVI는 농업 가뭄 모니터링을 위한 주요 위성 유래 식생 지수입니다. 값은 -1에서 +1까지이며, 건강한 식생은 일반적으로 0.3에서 0.8 사이의 값을 보입니다.

### NDVI 데이터 형식 사양

| 형식 | 사용 사례 | 장점 | 파일 확장자 |
|------|----------|------|-----------|
| GeoTIFF | 데스크톱 GIS, 아카이브 | 범용 호환성 | .tif |
| Cloud Optimized GeoTIFF | 웹 스트리밍, 클라우드 | 범위 요청, 효율적 | .tif |
| NetCDF | 시계열 분석 | 자기 기술, CF 규약 | .nc |
| JSON (포인트) | API 응답 | 가벼움, 웹 친화적 | .json |

### 가뭄 평가를 위한 NDVI 분류

| NDVI 범위 | 식생 상태 | 가뭄 의미 |
|----------|----------|----------|
| < 0.0 | 비식생 (물, 나지) | 해당 없음 |
| 0.0 - 0.15 | 나지/희소 식생 | 심각한 스트레스 또는 비식생 |
| 0.15 - 0.25 | 스트레스/불건강 식생 | 가뭄 스트레스 가능 |
| 0.25 - 0.40 | 적당한 식생 | 스트레스 가능 |
| 0.40 - 0.60 | 건강한 식생 | 정상 상태 |
| 0.60 - 0.80 | 매우 건강, 밀집 식생 | 평년 이상 활력 |
| > 0.80 | 예외적으로 건강 | 최적 상태 |

### NDVI 이상 계산

```python
def calculate_ndvi_anomaly(ndvi_current, ndvi_historical):
    """
    역사적 기준선 대비 NDVI 이상 계산.

    매개변수:
    -----------
    ndvi_current : float 또는 배열
        현재 NDVI 값
    ndvi_historical : dict
        'mean', 'std' 키를 가진 역사적 통계

    반환값:
    --------
    dict: 이상 지표
    """
    mean = ndvi_historical['mean']
    std = ndvi_historical['std']

    # 절대 이상
    absolute_anomaly = ndvi_current - mean

    # 표준화 이상 (z-점수)
    standardized_anomaly = (ndvi_current - mean) / std

    # 평년 대비 백분율
    percent_of_normal = (ndvi_current / mean) * 100

    # 식생 상태 지수 (0-100 척도)
    ndvi_min = ndvi_historical.get('min', mean - 2*std)
    ndvi_max = ndvi_historical.get('max', mean + 2*std)
    vci = ((ndvi_current - ndvi_min) / (ndvi_max - ndvi_min)) * 100
    vci = max(0, min(100, vci))  # 0-100으로 제한

    return {
        'absolute_anomaly': absolute_anomaly,
        'standardized_anomaly': standardized_anomaly,
        'percent_of_normal': percent_of_normal,
        'vegetation_condition_index': vci
    }
```

---

## 4.5 증발산 데이터 구조

### ET 변수 및 단위

| 변수 | 기호 | 단위 | 설명 |
|------|------|------|------|
| 기준 ET | ET₀ | mm/일 | 잔디 기준 증발산 |
| 실제 ET | ETa | mm/일 | 실제 증발산 |
| 잠재 ET | PET | mm/일 | 잠재 증발산 |
| 작물 ET | ETc | mm/일 | 작물별 증발산 |
| ET 이상 | ETa-anom | mm/일 | 평년에서의 편차 |
| 증발 스트레스 지수 | ESI | 무차원 | 표준화된 ET 이상 |

### 증발산 스키마

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA-ENV-003 증발산 스키마",
  "type": "object",
  "properties": {
    "header": {
      "type": "object",
      "properties": {
        "standard_id": {"const": "WIA-ENV-003"},
        "version": {"type": "string"},
        "et_method": {
          "type": "string",
          "enum": [
            "FAO56_PM",
            "Hargreaves",
            "Priestley_Taylor",
            "SEBAL",
            "METRIC",
            "SSEBop"
          ]
        },
        "temporal_resolution": {
          "type": "string",
          "enum": ["hourly", "daily", "weekly", "monthly"]
        }
      }
    },
    "data": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "location": {
            "type": "object",
            "properties": {
              "location_id": {"type": "string"},
              "coordinates": {
                "type": "object",
                "properties": {
                  "latitude": {"type": "number"},
                  "longitude": {"type": "number"}
                }
              }
            }
          },
          "timestamp": {"type": "string", "format": "date-time"},
          "et_reference": {
            "type": "object",
            "properties": {
              "value_mm": {"type": "number"},
              "method": {"type": "string"}
            }
          },
          "et_actual": {
            "type": "object",
            "properties": {
              "value_mm": {"type": "number"},
              "source": {"type": "string"}
            }
          },
          "et_fraction": {
            "type": "number",
            "description": "ETa/ET0 비율"
          },
          "anomaly": {
            "type": "object",
            "properties": {
              "absolute_mm": {"type": "number"},
              "percent_of_normal": {"type": "number"}
            }
          }
        }
      }
    }
  }
}
```

---

## 4.6 기상 관측소 데이터 통합

### 기상 관측 스키마

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA-ENV-003 기상 관측소 스키마",
  "type": "object",
  "properties": {
    "header": {
      "type": "object",
      "properties": {
        "standard_id": {"const": "WIA-ENV-003"},
        "station_metadata": {
          "type": "object",
          "properties": {
            "station_id": {"type": "string"},
            "station_name": {"type": "string"},
            "network": {"type": "string"},
            "coordinates": {
              "type": "object",
              "properties": {
                "latitude": {"type": "number"},
                "longitude": {"type": "number"},
                "elevation_m": {"type": "number"}
              }
            },
            "installation_date": {"type": "string", "format": "date"}
          }
        }
      }
    },
    "observations": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "timestamp": {"type": "string", "format": "date-time"},
          "temperature": {
            "type": "object",
            "properties": {
              "air_temp_c": {"type": "number"},
              "max_temp_c": {"type": "number"},
              "min_temp_c": {"type": "number"}
            }
          },
          "precipitation": {
            "type": "object",
            "properties": {
              "total_mm": {"type": "number"},
              "intensity_mmhr": {"type": "number"}
            }
          },
          "humidity": {
            "type": "object",
            "properties": {
              "relative_humidity_percent": {"type": "number"}
            }
          },
          "wind": {
            "type": "object",
            "properties": {
              "speed_ms": {"type": "number"},
              "direction_degrees": {"type": "number"}
            }
          },
          "radiation": {
            "type": "object",
            "properties": {
              "solar_wm2": {"type": "number"}
            }
          }
        }
      }
    }
  }
}
```

### 한국 관측소 네트워크 통합 표

| 네트워크 | 커버리지 | 변수 | 업데이트 빈도 | 형식 |
|---------|---------|------|-------------|------|
| 기상청 AWS | 전국 | 기온, 강수, 풍속, 습도 | 분 단위 | API |
| 농업기상관측망 | 농업지역 | 토양, 기상 | 시간별 | API |
| 산림기상관측망 | 산림지역 | 기상, 산불위험 | 시간별 | API |
| 수문관측소 | 하천유역 | 수위, 유량 | 실시간 | API |

---

## 4.7 위성 영상 메타데이터 표준

### STAC 카탈로그 구조

표준은 위성 데이터 검색을 위해 SpatioTemporal Asset Catalog(STAC)를 채택합니다:

```json
{
  "type": "Feature",
  "stac_version": "1.0.0",
  "id": "GK2A_L2_AMI_20250115_NDVI",
  "geometry": {
    "type": "Polygon",
    "coordinates": [[[124.6, 33.1], [131.9, 33.1], [131.9, 38.6], [124.6, 38.6], [124.6, 33.1]]]
  },
  "bbox": [124.6, 33.1, 131.9, 38.6],
  "properties": {
    "datetime": "2025-01-15T03:00:00Z",
    "platform": "GK2A",
    "instruments": ["AMI"],
    "eo:cloud_cover": 8.5,
    "processing:level": "L2",
    "wia:standard": "WIA-ENV-003",
    "wia:drought_product": "NDVI"
  },
  "assets": {
    "ndvi": {
      "href": "s3://drought-data/ndvi/GK2A_20250115_NDVI.tif",
      "type": "image/tiff; application=geotiff; profile=cloud-optimized",
      "title": "NDVI"
    },
    "metadata": {
      "href": "s3://drought-data/ndvi/GK2A_20250115_NDVI.json",
      "type": "application/json",
      "title": "Metadata"
    }
  }
}
```

---

## 4.8 시계열 데이터 관리

### 시계열 데이터베이스 스키마

```sql
-- 가뭄 시계열을 위한 TimescaleDB 스키마
CREATE TABLE drought_indices (
    time TIMESTAMPTZ NOT NULL,
    location_id TEXT NOT NULL,
    index_type TEXT NOT NULL,
    value DOUBLE PRECISION,
    quality_flag INTEGER,
    confidence DOUBLE PRECISION
);

SELECT create_hypertable('drought_indices', 'time');

-- 효율적인 쿼리를 위한 인덱스 생성
CREATE INDEX idx_location_time ON drought_indices (location_id, time DESC);
CREATE INDEX idx_index_type ON drought_indices (index_type, time DESC);

-- 일반적인 쿼리를 위한 연속 집계
CREATE MATERIALIZED VIEW monthly_drought_summary
WITH (timescaledb.continuous) AS
SELECT
    time_bucket('1 month', time) AS month,
    location_id,
    index_type,
    AVG(value) as mean_value,
    MIN(value) as min_value,
    MAX(value) as max_value,
    COUNT(*) as observation_count
FROM drought_indices
GROUP BY month, location_id, index_type;
```

### 시계열 API 응답 형식

```json
{
  "timeseries": {
    "location_id": "KR-CB-001",
    "index_type": "PDSI",
    "start_date": "2020-01-01",
    "end_date": "2025-01-31",
    "interval": "monthly",
    "data": [
      {"date": "2020-01-01", "value": -0.75, "quality": 0},
      {"date": "2020-02-01", "value": -1.12, "quality": 0},
      {"date": "2020-03-01", "value": -1.45, "quality": 0}
    ],
    "statistics": {
      "mean": -1.35,
      "median": -1.22,
      "std_dev": 1.05,
      "min": -3.85,
      "max": 1.65,
      "trend": {
        "slope": -0.018,
        "p_value": 0.04,
        "interpretation": "통계적으로 유의한 건조 추세"
      }
    }
  }
}
```

---

## 4.9 복습 문제 및 핵심 요점

### 복습 문제

1. **PDSI 스키마**: WIA-ENV-003 PDSI 데이터 레코드에서 필수 필드는 무엇입니까? 각 필드가 가뭄 모니터링 응용에 왜 중요합니까?

2. **SPI 시간 척도**: 예시는 다른 이야기를 전하는 1, 3, 6, 12개월 시간 척도의 SPI 값을 보여줍니다(거의 정상에서 극심하게 건조). 이 패턴이 드러내는 가뭄 역학을 설명하세요.

3. **토양 수분 계층**: 표준이 단일 값 대신 여러 토양 계층을 지정하는 이유는 무엇입니까? 다른 계층이 다른 농업 응용과 어떻게 관련됩니까?

4. **NDVI 품질**: 위성 NDVI 제품이 광범위한 구름 오염을 보여줍니다. 어떤 품질 플래그를 설정해야 하며, 다운스트림 응용이 이 데이터를 어떻게 처리해야 합니까?

5. **ET 방법**: 스키마는 여러 ET 추정 방법을 지원합니다. 어떤 상황에서 FAO-56 Penman-Monteith 대신 SSEBop과 같은 위성 기반 방법을 선택하시겠습니까?

6. **데이터 통합**: 필드 규모 가뭄 평가를 위해 포인트 기반 기상 관측소 데이터와 격자 위성 NDVI 제품을 어떻게 통합하시겠습니까?

7. **STAC 채택**: 전통적인 파일 명명 규칙에 비해 위성 데이터 카탈로그에 STAC을 사용하는 장점은 무엇입니까?

8. **시계열 분석**: 제공된 TimescaleDB 스키마를 사용하여 최소 6개월 연속 PDSI가 -2.0 이하인 위치를 식별하는 쿼리를 작성하세요.

### 핵심 요점

1. **포괄적인 지수 커버리지**: 표준은 모든 주요 가뭄 지수—PDSI, SPI/SPEI, 토양 수분, NDVI 및 ET—에 대한 형식을 지정하여 전체적인 가뭄 평가를 가능하게 합니다.

2. **계층적 데이터 구조**: 데이터 스키마는 모든 데이터 유형에 걸쳐 일관된 계층 구조에 위치, 시간, 측정 및 품질 구성요소를 포함합니다.

3. **품질 정보 통합**: 모든 데이터 형식에 품질 플래그와 신뢰도 지표가 포함되어 사용자가 신뢰성에 따라 데이터를 적절하게 가중할 수 있습니다.

4. **다중 형식 지원**: 표준은 다양한 맥락에 적합한 형식을 지원합니다—API용 JSON, 기후 데이터용 NetCDF, 영상용 GeoTIFF/COG.

5. **표준화된 분류**: 가뭄 심각도 분류는 지수 간에 일관되게 매핑되어 의미 있는 비교와 소통을 가능하게 합니다.

6. **토양 수분 깊이 프로파일**: 다중 계층 토양 수분 표현은 얕은 뿌리 작물에서 깊은 뿌리 다년생 및 지하수 충전까지의 응용을 지원합니다.

7. **위성 제품 통합**: STAC 준수 메타데이터는 위성 유래 가뭄 제품에 대한 효율적인 검색과 접근을 가능하게 합니다.

8. **시계열 최적화**: 시계열에 최적화된 데이터베이스 스키마는 수십 년간의 가뭄 이력에 걸친 효율적인 쿼리를 가능하게 합니다.

9. **메타데이터 완전성**: 처리 이력, 기준선 기간 및 알고리즘 버전을 포함한 포괄적인 메타데이터는 데이터 재현성을 보장합니다.

10. **확장성**: 스키마 설계는 이전 버전과의 호환성을 유지하면서 새로운 지수와 데이터 유형에 대한 확장을 허용합니다.

---

## 장 요약

이 장은 WIA-ENV-003 표준의 핵심에 있는 데이터 형식 사양을 상세히 설명했습니다. 기본적인 PDSI와 SPI 지수부터 토양 수분, 식생 및 증발산 데이터까지, 각 형식은 가뭄 모니터링 응용을 지원하는 포괄적인 스키마를 제공합니다.

형식은 공통 설계 원칙을 공유합니다: 계층적 구조, 포괄적인 메타데이터, 품질 플래그 통합 및 표준화된 분류 매핑. 이러한 원칙은 다양한 소스의 데이터가 일관된 가뭄 평가로 통합될 수 있도록 보장합니다.

JSON 스키마는 공식적인 검증 사양을 제공하고, 예시 레코드는 실용적인 데이터 표현을 보여줍니다. 지원 표는 지수 값을 가뭄 분류와 식생 상태에 매핑하여 구현 전반에 걸쳐 일관된 해석을 가능하게 합니다.

형식은 여러 출력 옵션을 지원합니다—API용 JSON, 기후 아카이브용 NetCDF, 영상용 GeoTIFF—다양한 응용이 다른 요구사항을 가지고 있음을 인식합니다. 위성 데이터 카탈로그를 위한 STAC 채택은 지구 관측 커뮤니티의 신흥 표준과 일치합니다.

시계열 관리 사양은 수십 년간의 가뭄 관측을 효율적으로 저장하고 쿼리하는 과제를 해결합니다. 시간 데이터에 최적화된 데이터베이스 스키마는 간단한 쿼리에서 복잡한 추세 감지까지의 분석을 가능하게 합니다.

이러한 데이터 형식 사양은 API 인터페이스(5장), 처리 프로토콜(6장) 및 시스템 통합(7장)의 기반을 제공합니다. 일관되고 잘 문서화된 데이터 형식은 상호 운용 가능한 가뭄 모니터링 시스템의 전제 조건입니다.

---

**다음 장: [제5장: API 인터페이스 및 서비스](05-api-interface.md)**
