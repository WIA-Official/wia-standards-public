# Chapter 4: Data Formats and Structures

## Standardized Schemas for Drought Monitoring Data

---

## 4.1 Palmer Drought Severity Index (PDSI) Data Format

### Overview of PDSI

The Palmer Drought Severity Index remains one of the most widely used drought indices globally, providing a standardized measure of moisture conditions based on a two-layer soil water balance model. The WIA-ENV-003 standard specifies comprehensive data formats for PDSI values, metadata, and quality information.

### PDSI Data Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA-ENV-003 PDSI Data Schema",
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
                "north": {"type": "number", "minimum": -90, "maximum": 90},
                "south": {"type": "number", "minimum": -90, "maximum": 90},
                "east": {"type": "number", "minimum": -180, "maximum": 180},
                "west": {"type": "number", "minimum": -180, "maximum": 180}
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
  },
  "definitions": {
    "pdsi_record": {
      "type": "object",
      "required": ["location", "period", "value", "quality"],
      "properties": {
        "location": {
          "type": "object",
          "properties": {
            "location_id": {"type": "string"},
            "name": {"type": "string"},
            "coordinates": {
              "type": "object",
              "properties": {
                "latitude": {"type": "number"},
                "longitude": {"type": "number"}
              }
            },
            "administrative": {
              "type": "object",
              "properties": {
                "country": {"type": "string"},
                "state_province": {"type": "string"},
                "county": {"type": "string"}
              }
            }
          }
        },
        "period": {
          "type": "object",
          "properties": {
            "year": {"type": "integer"},
            "month": {"type": "integer", "minimum": 1, "maximum": 12},
            "start_date": {"type": "string", "format": "date"},
            "end_date": {"type": "string", "format": "date"}
          }
        },
        "value": {
          "type": "number",
          "minimum": -10,
          "maximum": 10
        },
        "classification": {
          "type": "object",
          "properties": {
            "category": {
              "type": "string",
              "enum": ["W4", "W3", "W2", "W1", "N", "D0", "D1", "D2", "D3", "D4"]
            },
            "description": {"type": "string"}
          }
        },
        "quality": {
          "type": "object",
          "properties": {
            "flag": {"type": "integer", "minimum": 0, "maximum": 3},
            "confidence": {"type": "number", "minimum": 0, "maximum": 1},
            "missing_inputs": {"type": "array", "items": {"type": "string"}}
          }
        }
      }
    }
  }
}
```

### PDSI Classification Reference Table

| PDSI Value | Category | Classification | Description |
|------------|----------|----------------|-------------|
| ≥ 4.0 | W4 | Extremely Wet | Major flood conditions likely |
| 3.0 to 3.99 | W3 | Very Wet | Significant moisture surplus |
| 2.0 to 2.99 | W2 | Moderately Wet | Above normal moisture |
| 1.0 to 1.99 | W1 | Slightly Wet | Minor moisture surplus |
| -0.99 to 0.99 | N | Near Normal | Normal moisture conditions |
| -1.0 to -1.99 | D0 | Abnormally Dry | Short-term dryness |
| -2.0 to -2.99 | D1 | Moderate Drought | Some crop damage |
| -3.0 to -3.99 | D2 | Severe Drought | Crop losses likely |
| -4.0 to -4.99 | D3 | Extreme Drought | Major crop losses |
| ≤ -5.0 | D4 | Exceptional Drought | Exceptional losses |

### Example PDSI Data Record

```json
{
  "header": {
    "standard_id": "WIA-ENV-003",
    "version": "1.0.0",
    "index_type": "SC-PDSI",
    "generation_time": "2025-01-15T12:00:00Z",
    "source_institution": "NOAA NCEI",
    "spatial_coverage": {
      "type": "national",
      "bounds": {
        "north": 49.0,
        "south": 24.0,
        "east": -66.0,
        "west": -125.0
      }
    }
  },
  "data": [
    {
      "location": {
        "location_id": "US-KS-CD05",
        "name": "Kansas Climate Division 5",
        "coordinates": {
          "latitude": 38.5,
          "longitude": -98.5
        },
        "administrative": {
          "country": "United States",
          "state_province": "Kansas",
          "county": null
        }
      },
      "period": {
        "year": 2025,
        "month": 1,
        "start_date": "2025-01-01",
        "end_date": "2025-01-31"
      },
      "value": -3.45,
      "classification": {
        "category": "D2",
        "description": "Severe Drought"
      },
      "quality": {
        "flag": 0,
        "confidence": 0.95,
        "missing_inputs": []
      }
    }
  ]
}
```

---

## 4.2 Standardized Precipitation Index (SPI) Schema

### SPI Overview

The Standardized Precipitation Index quantifies precipitation anomalies across multiple time scales, making it versatile for monitoring different drought types. The WIA-ENV-003 standard specifies SPI formats supporting time scales from 1 to 48 months.

### SPI Data Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA-ENV-003 SPI Data Schema",
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
    },
    "data": {
      "type": "array",
      "items": {
        "$ref": "#/definitions/spi_record"
      }
    }
  },
  "definitions": {
    "spi_record": {
      "type": "object",
      "required": ["location", "period", "value"],
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
        "period": {
          "type": "object",
          "properties": {
            "end_date": {"type": "string", "format": "date"},
            "accumulation_period": {"type": "string"}
          }
        },
        "value": {
          "type": "number",
          "description": "SPI value in standard deviations"
        },
        "percentile": {
          "type": "number",
          "minimum": 0,
          "maximum": 100
        },
        "precipitation": {
          "type": "object",
          "properties": {
            "observed_mm": {"type": "number"},
            "normal_mm": {"type": "number"},
            "anomaly_percent": {"type": "number"}
          }
        },
        "classification": {
          "type": "object",
          "properties": {
            "category": {"type": "string"},
            "probability": {"type": "number"}
          }
        }
      }
    }
  }
}
```

### SPI Classification Table

| SPI Value | Category | Probability | Description |
|-----------|----------|-------------|-------------|
| ≥ 2.0 | Extremely Wet | 2.3% | Exceptional moisture surplus |
| 1.5 to 1.99 | Very Wet | 4.4% | Significant surplus |
| 1.0 to 1.49 | Moderately Wet | 9.2% | Above normal |
| -0.99 to 0.99 | Near Normal | 68.3% | Normal conditions |
| -1.0 to -1.49 | Moderately Dry | 9.2% | Below normal |
| -1.5 to -1.99 | Severely Dry | 4.4% | Significant deficit |
| ≤ -2.0 | Extremely Dry | 2.3% | Exceptional deficit |

### Multi-Time Scale SPI Example

```json
{
  "header": {
    "standard_id": "WIA-ENV-003",
    "version": "1.0.0",
    "index_type": "SPI",
    "distribution": "gamma",
    "baseline_period": {
      "start_year": 1981,
      "end_year": 2010
    }
  },
  "data": [
    {
      "location": {
        "location_id": "STATION-001",
        "coordinates": {"latitude": 39.7392, "longitude": -104.9903}
      },
      "multi_scale_values": [
        {
          "time_scale": {"value": 1, "unit": "month"},
          "value": 0.45,
          "percentile": 67.3,
          "classification": {"category": "Near Normal"}
        },
        {
          "time_scale": {"value": 3, "unit": "month"},
          "value": -1.23,
          "percentile": 10.9,
          "classification": {"category": "Moderately Dry"}
        },
        {
          "time_scale": {"value": 6, "unit": "month"},
          "value": -1.87,
          "percentile": 3.1,
          "classification": {"category": "Severely Dry"}
        },
        {
          "time_scale": {"value": 12, "unit": "month"},
          "value": -2.15,
          "percentile": 1.6,
          "classification": {"category": "Extremely Dry"}
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

## 4.3 Soil Moisture Data Representation

### Soil Moisture Variables

Soil moisture can be expressed in multiple ways. The WIA-ENV-003 standard supports all common representations:

| Variable | Unit | Description | Typical Range |
|----------|------|-------------|---------------|
| Volumetric Water Content | m³/m³ | Volume of water per volume of soil | 0.05-0.50 |
| Degree of Saturation | fraction | Fraction of pore space filled | 0.0-1.0 |
| Plant Available Water | mm | Water available to plants | 0-200 mm |
| Soil Moisture Percentile | % | Percentile relative to history | 0-100 |
| Soil Moisture Anomaly | mm | Departure from normal | -100 to +100 |

### Soil Moisture Data Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA-ENV-003 Soil Moisture Schema",
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
          "description": "Soil model or sensor type"
        }
      }
    },
    "data": {
      "type": "array",
      "items": {
        "$ref": "#/definitions/soil_moisture_record"
      }
    }
  },
  "definitions": {
    "soil_moisture_record": {
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
              "volumetric_water_content": {
                "type": "number",
                "minimum": 0,
                "maximum": 1
              },
              "saturation_degree": {"type": "number"},
              "plant_available_water_mm": {"type": "number"},
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
            "plant_available_water_mm": {"type": "number"},
            "percentile": {"type": "number"},
            "drought_category": {"type": "string"}
          }
        },
        "soil_properties": {
          "type": "object",
          "properties": {
            "texture_class": {"type": "string"},
            "field_capacity": {"type": "number"},
            "wilting_point": {"type": "number"},
            "porosity": {"type": "number"}
          }
        }
      }
    }
  }
}
```

### Example Soil Moisture Record

```json
{
  "header": {
    "standard_id": "WIA-ENV-003",
    "version": "1.0.0",
    "data_source": "blended",
    "soil_model": "NASA SMAP-NLDAS Blended"
  },
  "data": [
    {
      "location": {
        "location_id": "SCAN-2001",
        "coordinates": {
          "latitude": 34.2548,
          "longitude": -89.8735
        },
        "elevation_m": 105
      },
      "timestamp": "2025-01-15T06:00:00Z",
      "soil_layers": [
        {
          "layer_id": "0-10cm",
          "depth_top_cm": 0,
          "depth_bottom_cm": 10,
          "volumetric_water_content": 0.18,
          "saturation_degree": 0.42,
          "plant_available_water_mm": 12.5,
          "percentile": 15,
          "anomaly_mm": -8.2,
          "quality_flag": 0
        },
        {
          "layer_id": "10-40cm",
          "depth_top_cm": 10,
          "depth_bottom_cm": 40,
          "volumetric_water_content": 0.24,
          "saturation_degree": 0.55,
          "plant_available_water_mm": 45.0,
          "percentile": 22,
          "anomaly_mm": -15.3,
          "quality_flag": 0
        },
        {
          "layer_id": "40-100cm",
          "depth_top_cm": 40,
          "depth_bottom_cm": 100,
          "volumetric_water_content": 0.28,
          "saturation_degree": 0.62,
          "plant_available_water_mm": 85.0,
          "percentile": 28,
          "anomaly_mm": -22.5,
          "quality_flag": 0
        }
      ],
      "integrated_column": {
        "total_depth_cm": 100,
        "total_water_mm": 248,
        "plant_available_water_mm": 142.5,
        "percentile": 20,
        "drought_category": "D2"
      },
      "soil_properties": {
        "texture_class": "silt_loam",
        "field_capacity": 0.34,
        "wilting_point": 0.12,
        "porosity": 0.45
      }
    }
  ]
}
```

---

## 4.4 Normalized Difference Vegetation Index (NDVI) Format

### NDVI Overview

NDVI is the primary satellite-derived vegetation index for agricultural drought monitoring. Values range from -1 to +1, with healthy vegetation typically showing values between 0.3 and 0.8.

### NDVI Data Format Specifications

| Format | Use Case | Advantages | File Extension |
|--------|----------|------------|----------------|
| GeoTIFF | Desktop GIS, archive | Universal compatibility | .tif |
| Cloud Optimized GeoTIFF | Web streaming, cloud | Range requests, efficient | .tif |
| NetCDF | Time series analysis | Self-describing, CF conventions | .nc |
| JSON (Point) | API responses | Lightweight, web-friendly | .json |

### NDVI Raster Metadata Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA-ENV-003 NDVI Raster Metadata",
  "type": "object",
  "properties": {
    "standard_id": {"const": "WIA-ENV-003"},
    "product_type": {
      "type": "string",
      "enum": ["NDVI", "NDVI_ANOMALY", "VCI", "VHI"]
    },
    "satellite_platform": {
      "type": "string",
      "enum": ["MODIS_TERRA", "MODIS_AQUA", "LANDSAT_8", "LANDSAT_9", "SENTINEL_2", "VIIRS"]
    },
    "temporal_info": {
      "type": "object",
      "properties": {
        "acquisition_start": {"type": "string", "format": "date-time"},
        "acquisition_end": {"type": "string", "format": "date-time"},
        "composite_period_days": {"type": "integer"},
        "composite_method": {
          "type": "string",
          "enum": ["maximum_value", "median", "mean", "closest_to_nadir"]
        }
      }
    },
    "spatial_info": {
      "type": "object",
      "properties": {
        "crs": {"type": "string"},
        "resolution_m": {"type": "number"},
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
    },
    "processing_info": {
      "type": "object",
      "properties": {
        "atmospheric_correction": {
          "type": "string",
          "enum": ["6S", "LEDAPS", "LaSRC", "Sen2Cor", "none"]
        },
        "cloud_masking": {
          "type": "string",
          "enum": ["Fmask", "CFMask", "QA_band", "s2cloudless", "none"]
        },
        "processing_version": {"type": "string"}
      }
    },
    "bands": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "band_number": {"type": "integer"},
          "name": {"type": "string"},
          "description": {"type": "string"},
          "scale_factor": {"type": "number"},
          "add_offset": {"type": "number"},
          "no_data_value": {"type": "number"},
          "valid_range": {
            "type": "object",
            "properties": {
              "min": {"type": "number"},
              "max": {"type": "number"}
            }
          }
        }
      }
    },
    "quality_layer": {
      "type": "object",
      "properties": {
        "band_number": {"type": "integer"},
        "bit_flags": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "bit_position": {"type": "integer"},
              "description": {"type": "string"},
              "values": {"type": "object"}
            }
          }
        }
      }
    }
  }
}
```

### NDVI Classification for Drought Assessment

| NDVI Range | Vegetation Condition | Drought Implication |
|------------|---------------------|---------------------|
| < 0.0 | Non-vegetation (water, bare soil) | N/A |
| 0.0 - 0.15 | Bare/sparse vegetation | Severe stress or non-vegetated |
| 0.15 - 0.25 | Stressed/unhealthy vegetation | Drought stress likely |
| 0.25 - 0.40 | Moderate vegetation | Possible stress |
| 0.40 - 0.60 | Healthy vegetation | Normal conditions |
| 0.60 - 0.80 | Very healthy, dense vegetation | Above normal vigor |
| > 0.80 | Exceptionally healthy | Optimal conditions |

### NDVI Anomaly Calculation

```python
def calculate_ndvi_anomaly(ndvi_current, ndvi_historical):
    """
    Calculate NDVI anomaly relative to historical baseline.

    Parameters:
    -----------
    ndvi_current : float or array
        Current NDVI value(s)
    ndvi_historical : dict
        Historical statistics with keys 'mean', 'std'

    Returns:
    --------
    dict with anomaly metrics
    """
    mean = ndvi_historical['mean']
    std = ndvi_historical['std']

    # Absolute anomaly
    absolute_anomaly = ndvi_current - mean

    # Standardized anomaly (z-score)
    standardized_anomaly = (ndvi_current - mean) / std

    # Percent of normal
    percent_of_normal = (ndvi_current / mean) * 100

    # Vegetation Condition Index (0-100 scale)
    ndvi_min = ndvi_historical.get('min', mean - 2*std)
    ndvi_max = ndvi_historical.get('max', mean + 2*std)
    vci = ((ndvi_current - ndvi_min) / (ndvi_max - ndvi_min)) * 100
    vci = max(0, min(100, vci))  # Clamp to 0-100

    return {
        'absolute_anomaly': absolute_anomaly,
        'standardized_anomaly': standardized_anomaly,
        'percent_of_normal': percent_of_normal,
        'vegetation_condition_index': vci
    }
```

---

## 4.5 Evapotranspiration Data Structures

### ET Variables and Units

| Variable | Symbol | Unit | Description |
|----------|--------|------|-------------|
| Reference ET | ET₀ | mm/day | Grass reference evapotranspiration |
| Actual ET | ETa | mm/day | Actual evapotranspiration |
| Potential ET | PET | mm/day | Potential evapotranspiration |
| Crop ET | ETc | mm/day | Crop-specific evapotranspiration |
| ET Anomaly | ETa-anom | mm/day | Departure from normal |
| Evaporative Stress Index | ESI | unitless | Standardized ET anomaly |

### Evapotranspiration Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA-ENV-003 Evapotranspiration Schema",
  "type": "object",
  "required": ["header", "data"],
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
            "SSEBop",
            "DisALEXI"
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
            "minimum": 0,
            "maximum": 1.5,
            "description": "ETa/ET0 ratio"
          },
          "anomaly": {
            "type": "object",
            "properties": {
              "absolute_mm": {"type": "number"},
              "percent_of_normal": {"type": "number"},
              "evaporative_stress_index": {"type": "number"}
            }
          },
          "meteorological_inputs": {
            "type": "object",
            "properties": {
              "temperature_max_c": {"type": "number"},
              "temperature_min_c": {"type": "number"},
              "relative_humidity_percent": {"type": "number"},
              "wind_speed_ms": {"type": "number"},
              "solar_radiation_mjm2": {"type": "number"}
            }
          }
        }
      }
    }
  }
}
```

---

## 4.6 Weather Station Data Integration

### Weather Observation Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA-ENV-003 Weather Station Schema",
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
            "installation_date": {"type": "string", "format": "date"},
            "equipment": {"type": "array", "items": {"type": "string"}}
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
              "min_temp_c": {"type": "number"},
              "dewpoint_c": {"type": "number"},
              "soil_temp_5cm_c": {"type": "number"},
              "soil_temp_10cm_c": {"type": "number"}
            }
          },
          "precipitation": {
            "type": "object",
            "properties": {
              "total_mm": {"type": "number"},
              "intensity_mmhr": {"type": "number"},
              "type": {"type": "string", "enum": ["rain", "snow", "mixed", "none"]}
            }
          },
          "humidity": {
            "type": "object",
            "properties": {
              "relative_humidity_percent": {"type": "number"},
              "vapor_pressure_kpa": {"type": "number"}
            }
          },
          "wind": {
            "type": "object",
            "properties": {
              "speed_ms": {"type": "number"},
              "direction_degrees": {"type": "number"},
              "gust_ms": {"type": "number"}
            }
          },
          "radiation": {
            "type": "object",
            "properties": {
              "solar_wm2": {"type": "number"},
              "net_radiation_wm2": {"type": "number"},
              "par_umol": {"type": "number"}
            }
          },
          "pressure": {
            "type": "object",
            "properties": {
              "station_pressure_hpa": {"type": "number"},
              "sea_level_pressure_hpa": {"type": "number"}
            }
          },
          "quality_flags": {
            "type": "object",
            "additionalProperties": {"type": "integer"}
          }
        }
      }
    }
  }
}
```

### Station Network Integration Table

| Network | Coverage | Variables | Update Frequency | Format |
|---------|----------|-----------|------------------|--------|
| GHCN-Daily | Global | T, P | Daily | CSV, API |
| ISD | Global | Full met | Hourly | Fixed-width |
| SCAN | US | Soil, met | Hourly | API |
| USCRN | US | High-quality | 5-min | NetCDF |
| AWDN | US Midwest | Agricultural | Hourly | API |
| Mesonets | Regional | Variable | 5-60 min | Variable |

---

## 4.7 Satellite Imagery Metadata Standards

### STAC Catalog Structure

The standard adopts SpatioTemporal Asset Catalog (STAC) for satellite data discovery:

```json
{
  "type": "Feature",
  "stac_version": "1.0.0",
  "stac_extensions": [
    "https://stac-extensions.github.io/eo/v1.0.0/schema.json",
    "https://stac-extensions.github.io/processing/v1.0.0/schema.json"
  ],
  "id": "LC08_L2SP_027033_20250115_20250118_02_T1",
  "geometry": {
    "type": "Polygon",
    "coordinates": [[[-99.5, 38.0], [-99.5, 40.0], [-97.0, 40.0], [-97.0, 38.0], [-99.5, 38.0]]]
  },
  "bbox": [-99.5, 38.0, -97.0, 40.0],
  "properties": {
    "datetime": "2025-01-15T16:45:00Z",
    "platform": "landsat-8",
    "instruments": ["oli", "tirs"],
    "eo:cloud_cover": 5.2,
    "processing:level": "L2SP",
    "wia:standard": "WIA-ENV-003",
    "wia:drought_product": "NDVI"
  },
  "assets": {
    "ndvi": {
      "href": "s3://drought-data/ndvi/LC08_027033_20250115_NDVI.tif",
      "type": "image/tiff; application=geotiff; profile=cloud-optimized",
      "title": "NDVI",
      "eo:bands": [{"name": "NDVI", "common_name": "ndvi"}]
    },
    "ndvi_qa": {
      "href": "s3://drought-data/ndvi/LC08_027033_20250115_NDVI_QA.tif",
      "type": "image/tiff; application=geotiff",
      "title": "NDVI Quality"
    },
    "metadata": {
      "href": "s3://drought-data/ndvi/LC08_027033_20250115_NDVI.json",
      "type": "application/json",
      "title": "Metadata"
    }
  },
  "links": [
    {"rel": "self", "href": "..."},
    {"rel": "collection", "href": "..."},
    {"rel": "parent", "href": "..."}
  ]
}
```

---

## 4.8 Time Series Data Management

### Time Series Database Schema

```sql
-- TimescaleDB schema for drought time series
CREATE TABLE drought_indices (
    time TIMESTAMPTZ NOT NULL,
    location_id TEXT NOT NULL,
    index_type TEXT NOT NULL,
    value DOUBLE PRECISION,
    quality_flag INTEGER,
    confidence DOUBLE PRECISION
);

SELECT create_hypertable('drought_indices', 'time');

-- Create indexes for efficient querying
CREATE INDEX idx_location_time ON drought_indices (location_id, time DESC);
CREATE INDEX idx_index_type ON drought_indices (index_type, time DESC);

-- Continuous aggregates for common queries
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

### Time Series API Response Format

```json
{
  "timeseries": {
    "location_id": "US-KS-CD05",
    "index_type": "PDSI",
    "start_date": "2020-01-01",
    "end_date": "2025-01-31",
    "interval": "monthly",
    "data": [
      {"date": "2020-01-01", "value": -0.85, "quality": 0},
      {"date": "2020-02-01", "value": -1.23, "quality": 0},
      {"date": "2020-03-01", "value": -1.56, "quality": 0}
    ],
    "statistics": {
      "mean": -1.45,
      "median": -1.32,
      "std_dev": 1.12,
      "min": -4.25,
      "max": 1.85,
      "trend": {
        "slope": -0.02,
        "p_value": 0.03,
        "interpretation": "Significant drying trend"
      }
    }
  }
}
```

---

## 4.9 Review Questions and Key Takeaways

### Review Questions

1. **PDSI Schema**: What are the required fields in a WIA-ENV-003 PDSI data record? Why is each field important for drought monitoring applications?

2. **SPI Time Scales**: The example shows SPI values at 1, 3, 6, and 12-month time scales that tell different stories (near normal to extremely dry). Explain what drought dynamics this pattern reveals.

3. **Soil Moisture Layers**: Why does the standard specify multiple soil layers rather than a single value? How do different layers relate to different agricultural applications?

4. **NDVI Quality**: A satellite NDVI product shows widespread cloud contamination. What quality flags should be set, and how should downstream applications handle this data?

5. **ET Methods**: The schema supports multiple ET estimation methods. Under what circumstances might you choose FAO-56 Penman-Monteith versus a satellite-based method like SSEBop?

6. **Data Integration**: How would you integrate point-based weather station data with gridded satellite NDVI products for field-scale drought assessment?

7. **STAC Adoption**: What are the advantages of using STAC for satellite data cataloging compared to traditional file naming conventions?

8. **Time Series Analysis**: Using the TimescaleDB schema provided, write a query to identify locations that have experienced at least 6 consecutive months of PDSI below -2.0.

### Key Takeaways

1. **Comprehensive Index Coverage**: The standard specifies formats for all major drought indices—PDSI, SPI/SPEI, soil moisture, NDVI, and ET—enabling holistic drought assessment.

2. **Hierarchical Data Structures**: Data schemas include location, temporal, measurement, and quality components in a consistent hierarchical structure across all data types.

3. **Quality Information Integration**: Every data format includes quality flags and confidence metrics, enabling users to appropriately weight data based on reliability.

4. **Multi-Format Support**: The standard supports appropriate formats for different contexts—JSON for APIs, NetCDF for climate data, GeoTIFF/COG for imagery.

5. **Standardized Classifications**: Drought severity classifications map consistently across indices, enabling meaningful comparisons and communication.

6. **Soil Moisture Depth Profiles**: Multi-layer soil moisture representation supports applications from shallow-rooted crops to deep-rooted perennials and groundwater recharge.

7. **Satellite Product Integration**: STAC-compliant metadata enables efficient discovery and access to satellite-derived drought products.

8. **Time Series Optimization**: Database schemas optimized for time series enable efficient queries across decades of drought history.

9. **Metadata Completeness**: Comprehensive metadata including processing history, baseline periods, and algorithm versions ensures data reproducibility.

10. **Extensibility**: Schema design allows extension for new indices and data types while maintaining backward compatibility.

---

## Chapter Summary

This chapter has detailed the data format specifications at the core of the WIA-ENV-003 standard. From the foundational PDSI and SPI indices through soil moisture, vegetation, and evapotranspiration data, each format provides a comprehensive schema supporting drought monitoring applications.

The formats share common design principles: hierarchical structure, comprehensive metadata, quality flag integration, and standardized classification mapping. These principles ensure that data from diverse sources can be integrated into coherent drought assessments.

JSON schemas provide formal validation specifications, while example records demonstrate practical data representation. Supporting tables map index values to drought classifications and vegetation conditions, enabling consistent interpretation across implementations.

The formats support multiple output options—JSON for APIs, NetCDF for climate archives, GeoTIFF for imagery—recognizing that different applications have different requirements. STAC adoption for satellite data catalogs aligns with emerging standards in the earth observation community.

Time series management specifications address the challenge of efficiently storing and querying decades of drought observations. Database schemas optimized for temporal data enable analyses from simple queries to complex trend detection.

These data format specifications provide the foundation for API interfaces (Chapter 5), processing protocols (Chapter 6), and system integration (Chapter 7). Consistent, well-documented data formats are the prerequisite for interoperable drought monitoring systems.

---

**Next Chapter: [Chapter 5: API Interfaces and Services](05-api-interface.md)**
