# WIA Ecosystem Monitoring Standard - Phase 3: Protocol Specification v1.0

**Status:** Official Release  
**Version:** 1.0.0  
**Date:** December 26, 2025  
**License:** CC BY 4.0

## 1. Introduction

Phase 3 defines communication protocols, quality assurance procedures, and calibration standards.

## 2. Sensor Communication Protocols

### 2.1 Hardware Interfaces

Supported standards:
- **SDI-12**: Serial Data Interface for environmental sensors
- **Modbus RTU/TCP**: Industrial sensor protocol
- **I2C/SPI**: Digital sensor buses
- **4-20mA**: Analog current loop

### 2.2 Network Transport

Protocols by use case:
- **MQTT**: Low-bandwidth IoT sensor networks
- **CoAP**: Constrained devices
- **HTTP/HTTPS**: Standard connectivity
- **WebSocket**: Real-time bidirectional
- **LoRaWAN**: Long-range low-power wireless

### 2.3 Data Packet Structure

```json
{
  "packet_version": "1.0",
  "packet_id": "UUID",
  "sensor_id": "string",
  "timestamp": "ISO 8601",
  "sequence_number": "integer",
  "readings": [ /* array of measurements */ ],
  "battery_voltage": "number",
  "signal_strength": "number",
  "checksum": "CRC32 hash"
}
```

### 2.4 Error Detection

Requirements:
- CRC32 checksums for all packets
- Sequence numbers for ordering/duplicate detection
- Acknowledgments with retry mechanism
- Local buffering until transmission confirmed
- NTP/GPS time synchronization (±1 second accuracy)

## 3. Calibration Protocols

### 3.1 Calibration Frequency

Minimum requirements:

| Sensor Type | Interval | Method |
|------------|----------|--------|
| Temperature | 6 months | Ice point check |
| pH | 2 weeks | Two-point buffer |
| Dissolved Oxygen | 1 month | Water-saturated air |
| Conductivity | 3 months | Standard solution |
| Turbidity | 1 month | Formazin standards |
| Nutrients | Each run | Standard curve |

### 3.2 Calibration Documentation

Required metadata:
```json
{
  "calibration_id": "string",
  "sensor_id": "string",
  "calibration_date": "ISO 8601",
  "technician": "string",
  "method": "string",
  "standards_used": [
    {
      "value": "number",
      "lot_number": "string",
      "expiry_date": "ISO 8601"
    }
  ],
  "pre_calibration_readings": "array",
  "post_calibration_readings": "array",
  "adjustment_applied": "boolean",
  "drift_detected": "number",
  "pass_fail": "enum",
  "next_calibration_due": "ISO 8601",
  "certificate_url": "URL"
}
```

## 4. Quality Assurance Protocols

### 4.1 Field QA/QC

Requirements:
- **Field blanks**: 5% of samples
- **Equipment blanks**: Before/after sampling events
- **Field replicates**: 10% of samples
- **Positive controls**: For presence/absence methods

### 4.2 Laboratory QA/QC

Requirements:
- **Method blanks**: Each analytical batch
- **Calibration verification**: Every 10 samples
- **Spike recovery**: 10% of samples
- **Duplicate analysis**: 10% of samples
- **Certified reference materials**: Each batch
- **Blind QC samples**: 5% of samples

Acceptance criteria:
- Blank contamination: < detection limit
- Spike recovery: 75-125%
- Duplicate RPD: < 20%
- Reference material: Within ±2 standard deviations

### 4.3 Automated Data Validation

Required checks:
1. **Range checks**: Values within physical limits
2. **Rate-of-change**: Maximum change rates
3. **Flatline detection**: Minimum variability thresholds
4. **Spike detection**: Isolated extreme values
5. **Statistical outliers**: Beyond 3-4 standard deviations
6. **Cross-parameter**: Consistency between related variables

## 5. Field Sampling Protocols

### 5.1 Species Observations

**Point Count Protocol**:
- Fixed location for 5-10 minutes
- Record all detections within specified radius
- Note distance bands for detectability
- Conduct during appropriate season/time
- Multiple visits for detection probability
- Document environmental conditions

**Transect Protocol**:
- Predetermined route or random walk
- Defined width or distance bands
- Perpendicular distance measurement
- Consistent speed and timing
- Multiple observers when possible

### 5.2 Water Sampling

Standard procedure:
1. Approach from downstream
2. Rinse bottle 3× with ambient water
3. Collect mid-stream, mid-depth
4. Avoid sediment disturbance
5. Fill completely (no headspace for DO)
6. Preserve immediately if required
7. Label with metadata
8. Document field parameters
9. Maintain chain of custody
10. Transport on ice

### 5.3 Soil Sampling

Standard procedure:
1. Clear surface debris
2. Sample to specified depth
3. Composite multiple cores if required
4. Use clean, dedicated equipment
5. Store in appropriate containers
6. Keep cool until analysis
7. Process within specified time
8. Document horizon/depth precisely

## 6. Data Management Protocols

### 6.1 Data Entry

Requirements:
- Double data entry for critical datasets
- Real-time validation during entry
- Immediate backup of raw data
- Version control for edits
- Separation of raw/processed data

### 6.2 Metadata Documentation

Minimum required metadata (following EML/ISO 19115):
- Dataset title, abstract, keywords
- Authors, contacts, funding
- Temporal/spatial/taxonomic coverage
- Detailed methods and protocols
- QA/QC procedures
- Known limitations
- Access restrictions and licenses
- Related publications/datasets

### 6.3 Long-term Preservation

Requirements:
- Format migration to current standards
- Geographic distribution of backups
- Checksum verification
- Deposit in discipline repositories
- DOI assignment for citation

## 7. Change Management

### 7.1 Protocol Modifications

When changing methods:
1. Document what changed, when, why
2. Version control protocol documents
3. Update metadata
4. Flag data with method used
5. Conduct overlap studies (≥1 year)
6. Develop conversion factors
7. Document precision/bias differences

## 8. Proficiency Testing

### 8.1 Laboratory Proficiency

Requirements:
- Quarterly proficiency samples
- External provider or internal blind samples
- Z-score calculation and tracking
- Corrective action for |Z| > 2

### 8.2 Observer Calibration

Requirements:
- Multiple observers, same area
- Annual or when observers change
- Expert verification of identifications
- Training for discrepancies
- Document inter-observer variability
