# Phase 4 Research: Ecosystem Integration

**Date**: 2025-12-14
**Status**: Research Complete

---

## 1. Overview

Phase 4 focuses on integrating WIA Physics Standard with external systems:
- Control systems (EPICS, TANGO, OPC UA)
- Data archives (HDF5, ROOT, InfluxDB)
- Visualization (Grafana, web dashboards)
- External services (CERN Open Data, DOI)

---

## 2. Control System Integration

### 2.1 EPICS (Experimental Physics and Industrial Control System)

**Usage**: Most particle accelerators, fusion reactors, synchrotrons

**Key Features**:
- Channel Access (CA) protocol - default port 5064
- pvAccess (PVA) protocol - newer, more efficient
- Process Variables (PVs) for real-time data

**Integration Approach**:
```
WIA Physics Protocol (WPP) ←→ EPICS Gateway ←→ EPICS IOCs
```

**Data Mapping**:
| EPICS Type | WIA Physics Type |
|------------|------------------|
| DBR_DOUBLE | Measurement.value |
| DBR_TIME_DOUBLE | Measurement + timestamp |
| DBR_CTRL_DOUBLE | Measurement + limits |
| DBR_ALARM | ErrorEvent |

### 2.2 TANGO Controls

**Usage**: Synchrotrons (ESRF, SOLEIL), telescopes

**Key Features**:
- Device-oriented architecture
- CORBA/ZeroMQ transport
- Attribute/Command/Property model

**Data Mapping**:
| TANGO Type | WIA Physics Type |
|------------|------------------|
| DevDouble | Measurement.value |
| DevVarDoubleArray | Vec<Measurement> |
| DevState | DeviceStatus |

### 2.3 OPC UA (Unified Architecture)

**Usage**: Industrial physics applications

**Key Features**:
- Information model with namespaces
- Secure communication
- Discovery services

---

## 3. Data Archive Integration

### 3.1 HDF5 (Hierarchical Data Format)

**Usage**: Most physics experiments for large datasets

**Key Features**:
- Hierarchical group structure
- Efficient compression
- Parallel I/O
- Self-describing metadata

**Proposed Structure**:
```
/fusion/
  /plasma/
    temperature     [dataset, float64]
    density         [dataset, float64]
    timestamps      [dataset, int64]
  /magnetics/
    toroidal_field  [dataset, float64]
  /metadata         [group]
    experiment_id   [attribute, string]
```

### 3.2 ROOT (CERN)

**Usage**: Particle physics, high-energy physics

**Key Features**:
- TTree for event data
- TH1/TH2 for histograms
- Native C++ with Python bindings

**Integration**:
- Export WIA Physics data to ROOT files
- Import ROOT data into WIA format

### 3.3 Time-Series Databases

**Options**:
| Database | Best For |
|----------|----------|
| InfluxDB | Metrics, monitoring |
| TimescaleDB | PostgreSQL compatibility |
| QuestDB | High-performance ingestion |

---

## 4. Visualization Integration

### 4.1 Grafana

**Features**:
- Real-time dashboards
- Alert rules
- Plugin ecosystem

**Integration**:
- WIA Physics → InfluxDB → Grafana
- Custom data source plugin

### 4.2 Web Dashboards

**Technologies**:
- React/Vue for UI
- Three.js for 3D visualization
- D3.js for scientific plots
- WebGL for GPU-accelerated rendering

**Use Cases**:
- Tokamak cross-section visualization
- Particle detector event displays
- Real-time beam monitoring

### 4.3 Jupyter Integration

**Features**:
- Interactive analysis
- Matplotlib/Plotly visualization
- Python bindings for WIA Physics

---

## 5. External Services

### 5.1 CERN Open Data Portal

**URL**: https://opendata.cern.ch
**Format**: ROOT files, CSV, JSON
**Integration**: Export WIA Physics → CERN format

### 5.2 EOSC (European Open Science Cloud)

**Features**:
- Federated data infrastructure
- FAIR data principles
- DOI assignment

### 5.3 Data Citation

**Standard**: DataCite DOI
**Metadata**: Dublin Core + domain-specific

---

## 6. Recommended Implementation

### 6.1 Adapter Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    WIA Physics Data                         │
│                  (Phase 1-3 Output)                         │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    IntegrationManager                        │
│                   (Central Controller)                       │
├──────────────┬──────────────┬──────────────┬────────────────┤
│ EPICSAdapter │ TANGOAdapter │  HDF5Adapter │ GrafanaAdapter │
├──────────────┼──────────────┼──────────────┼────────────────┤
│  EPICS IOC   │ TANGO Device │  HDF5 File   │ InfluxDB/Graf  │
└──────────────┴──────────────┴──────────────┴────────────────┘
```

### 6.2 Priority Adapters

| Priority | Adapter | Reason |
|----------|---------|--------|
| **P0** | HDF5 | Universal physics format |
| **P0** | EPICS | Most accelerator/fusion facilities |
| **P1** | InfluxDB/Grafana | Monitoring dashboards |
| **P1** | ROOT | Particle physics analysis |
| **P2** | TANGO | Synchrotron facilities |
| **P2** | OPC UA | Industrial applications |

### 6.3 Rust Crates to Use

| Crate | Purpose |
|-------|---------|
| `hdf5` | HDF5 file I/O |
| `influxdb` | InfluxDB client |
| `tokio-postgres` | TimescaleDB |
| `root-io` | ROOT file reading |
| `tonic` | gRPC (for custom protocols) |

---

## 7. References

1. EPICS Documentation: https://epics.anl.gov/
2. TANGO Controls: https://www.tango-controls.org/
3. HDF5: https://www.hdfgroup.org/
4. ROOT: https://root.cern/
5. CERN Open Data: https://opendata.cern.ch/
6. OPC UA: https://opcfoundation.org/

---

**Conclusion**: Phase 4 should implement a modular adapter system supporting HDF5 and EPICS as core integrations, with visualization via Grafana/InfluxDB for monitoring dashboards.
