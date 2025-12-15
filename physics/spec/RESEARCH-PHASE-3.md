# WIA Physics Standard - Phase 3 Research

**Version**: 1.0.0
**Date**: 2025-12-14
**Status**: Research Complete

---

## 1. Executive Summary

This document presents research on communication protocols used in physics experiments and scientific instrumentation. The goal is to design a unified protocol for the WIA Physics Standard that enables real-time data streaming, device control, and interoperability across different physics domains.

---

## 2. Existing Physics Control Systems

### 2.1 EPICS (Experimental Physics and Industrial Control System)

**Overview**: The most widely used control system in physics facilities worldwide.

| Aspect | Details |
|--------|---------|
| **Adoption** | Particle accelerators, fusion reactors (ITER), synchrotrons |
| **Scale** | 40,000 to 10+ million Process Variables (PVs) per facility |
| **Protocols** | Channel Access (CA), pvAccess |
| **Transport** | TCP/IP |

**Key Features**:
- Distributed control system architecture
- Process Variables (PVs) as unique identifiers for data
- Real-time data streaming with event-driven updates
- Mature ecosystem with 30+ years of development

**Protocols**:

1. **Channel Access (CA)** - Legacy protocol
   - Application layer over TCP/IP
   - Simple get/put/monitor operations
   - Widely deployed, proven reliability

2. **pvAccess** - Modern protocol (EPICS 7+)
   - Higher performance than CA
   - Structured data types
   - Better support for complex data (images, arrays)

**2024 Developments**:
- Integration with container orchestration (Docker, Kubernetes)
- Zero-trust cybersecurity implementation
- Python-based PVAccess servers (p4p library)
- Machine learning integration

**References**:
- [EPICS Official Site](https://epics-controls.org/)
- [EPICS at ANL](https://epics.anl.gov/)

---

### 2.2 TANGO Controls

**Overview**: Object-oriented distributed control system framework.

| Aspect | Details |
|--------|---------|
| **Adoption** | Synchrotrons (ESRF, SOLEIL, ALBA, DESY), telescopes |
| **Architecture** | Device-oriented (unique vs signal-based systems) |
| **Protocols** | CORBA (omniorb), ZeroMQ |
| **Languages** | C++, Java, Python |

**Key Features**:
- Device and device class abstraction
- Synchronous, asynchronous, and event-driven communication
- CORBA for sync/async, ZeroMQ for events (since v8)
- SCADA system building toolkit

**Communication Patterns**:
- Synchronous: Request-response
- Asynchronous: Non-blocking calls
- Event-driven: Publish-subscribe via ZeroMQ

**References**:
- [TANGO Controls](https://www.tango-controls.org/)
- [TANGO Documentation](https://tango-controls.readthedocs.io/)

---

### 2.3 ITER CODAC

**Overview**: Control, Data Access and Communication system for ITER fusion reactor.

| Aspect | Details |
|--------|---------|
| **Scale** | 170+ local control systems, ~1,000,000 signals |
| **Suppliers** | 101 different suppliers |
| **Base** | Built on EPICS |
| **Networks** | PON, TCN, SDN, DAN |

**Network Architecture**:

| Network | Purpose |
|---------|---------|
| **PON** (Plant Operation Network) | General control |
| **TCN** (Time Communication Network) | Precision timing (IEEE 1588 PTP) |
| **SDN** (Synchronous Databus Network) | Real-time data |
| **DAN** (Data Archiving Network) | Long-term storage |

**Standards**:
- Plant Control Design Handbook (PCDH)
- CODAC Core System software distribution
- Nominal Device Support (NDS) drivers

**Milestone**: First CODAC command issued December 2024.

**References**:
- [ITER CODAC](https://www.iter.org/machine/supporting-systems/codac)
- [Cosylab CODAC Case Study](https://cosylab.com/stories/how-iter-standardized-large-system-integration-with-codac-case-study/)

---

### 2.4 XRootD (CERN/LHC)

**Overview**: High-performance data access protocol for LHC experiments.

| Aspect | Details |
|--------|---------|
| **Use Case** | Petabyte-scale physics data access |
| **Adoption** | All LHC experiments, WLCG sites |
| **Integration** | EOS, CASTOR storage systems |
| **Features** | Streaming, authentication, federation |

**Key Features**:
- Remote data streaming without full download
- Federated storage across global sites
- Integration with ROOT file format
- Continuous improvement for reliability and security

**Data Format**: ROOT files (columnar data format for HEP)

**References**:
- [CERN Open Data](https://opendata.cern.ch/)
- [ROOT at CERN](https://root.cern/)

---

## 3. Modern Communication Technologies

### 3.1 gRPC + Protocol Buffers

| Aspect | Details |
|--------|---------|
| **Performance** | 7-10x faster than REST |
| **Serialization** | Binary (Protocol Buffers) |
| **Transport** | HTTP/2 |
| **Streaming** | Bi-directional support |

**Advantages**:
- Low latency, high throughput
- Strong typing with code generation
- Native streaming support
- Multi-language support

**Limitations for Physics**:
- Less efficient for large multi-dimensional arrays
- Protocol Buffers not ideal for Fortran/IDL
- Specialized formats (FITS, HDF5) may be better for certain data

**Use Cases**:
- Microservices communication
- Real-time IoT data
- Video streaming

---

### 3.2 WebSocket

| Aspect | Details |
|--------|---------|
| **Standard** | RFC 6455 |
| **Transport** | TCP (upgraded from HTTP) |
| **Direction** | Full-duplex, bi-directional |
| **Format** | Text or binary frames |

**Advantages**:
- Browser-native support
- Low overhead after handshake
- Real-time updates
- Wide adoption

**Physics Applications**:
- Web-based monitoring dashboards
- Remote experiment control
- Live data visualization

---

### 3.3 MQTT

| Aspect | Details |
|--------|---------|
| **Pattern** | Publish-Subscribe |
| **QoS Levels** | 0 (at most once), 1 (at least once), 2 (exactly once) |
| **Transport** | TCP, WebSocket |
| **Payload** | Agnostic (JSON, binary) |

**Advantages**:
- Lightweight protocol
- Ideal for IoT and sensor networks
- Built-in QoS
- Retained messages

**Physics Applications**:
- Distributed sensor networks
- Environmental monitoring
- Low-bandwidth scenarios

---

### 3.4 ZeroMQ

| Aspect | Details |
|--------|---------|
| **Pattern** | Multiple (PUB-SUB, REQ-REP, PUSH-PULL) |
| **Transport** | TCP, IPC, inproc |
| **Performance** | Very high throughput |
| **Language** | 40+ language bindings |

**Advantages**:
- No broker required
- Flexible patterns
- High performance
- Used in TANGO Controls

---

## 4. Data Streaming Frameworks

### 4.1 PvaPy Streaming Framework

**Description**: Python-based real-time analysis of X-ray detector data via EPICS pvAccess.

**Features**:
- Direct streaming to Python applications
- Distributed analysis workflows
- Integration with areaDetector
- Automatic serialization/networking

**Reference**: [PvaPy Framework (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC12067317/)

---

### 4.2 Streaming Readout (SRO) DAQ

**Description**: Next-generation DAQ replacing hardware triggers with software.

**Features**:
- Software-based triggers
- AI-supported real-time analysis
- More flexible than traditional DAQ
- Proven superior to triggered systems

**Application**: Electron scattering experiments, particle physics

---

## 5. Protocol Comparison

| Protocol | Latency | Throughput | Streaming | Browser | Physics Adoption |
|----------|---------|------------|-----------|---------|-----------------|
| **EPICS CA** | Medium | High | Yes | No | Very High |
| **pvAccess** | Low | Very High | Yes | No | High |
| **TANGO** | Medium | High | Yes | No | High |
| **gRPC** | Very Low | Very High | Yes | Limited | Growing |
| **WebSocket** | Low | Medium | Yes | Yes | Medium |
| **MQTT** | Low | Medium | Yes | Yes | Low |
| **ZeroMQ** | Very Low | Very High | Yes | No | Medium |

---

## 6. WIA Physics Protocol Design Decisions

### 6.1 Requirements

Based on research, the WIA Physics Protocol must support:

1. **Real-time streaming**: Physics data often requires continuous streaming
2. **Request-response**: For device control and queries
3. **Event-driven updates**: For state changes and alerts
4. **Multiple transports**: WebSocket (web), TCP (native), potentially gRPC
5. **JSON compatibility**: For interoperability with Phase 1/2 schemas
6. **Binary option**: For high-volume data streams

### 6.2 Design Choices

| Aspect | Decision | Rationale |
|--------|----------|-----------|
| **Message Format** | JSON (primary), Binary (optional) | Compatibility with schemas |
| **Primary Transport** | WebSocket | Browser support, wide adoption |
| **Secondary Transport** | TCP (native) | High performance |
| **Future Transport** | gRPC | For performance-critical |
| **Streaming Model** | Pub-Sub + Request-Response | Flexibility |
| **Compression** | Optional (LZ4) | For high-volume data |

### 6.3 Protocol Layers

```
┌─────────────────────────────────────────┐
│       WIA Physics Application           │
├─────────────────────────────────────────┤
│       Protocol Handler                  │
│  (Message Building, Parsing, Routing)   │
├─────────────────────────────────────────┤
│       Transport Abstraction             │
│  (WebSocket, TCP, gRPC adapters)        │
├─────────────────────────────────────────┤
│       Network Layer                     │
│  (WebSocket/TCP/HTTP2)                  │
└─────────────────────────────────────────┘
```

### 6.4 Message Types

| Type | Direction | Description |
|------|-----------|-------------|
| `connect` | Client → Server | Connection request |
| `connect_ack` | Server → Client | Connection acknowledgment |
| `disconnect` | Both | Graceful disconnect |
| `subscribe` | Client → Server | Subscribe to data stream |
| `unsubscribe` | Client → Server | Unsubscribe from stream |
| `data` | Server → Client | Physics data (Phase 1 format) |
| `command` | Client → Server | Device control command |
| `response` | Server → Client | Command response |
| `event` | Server → Client | Asynchronous event |
| `error` | Both | Error notification |
| `ping` | Client → Server | Keepalive |
| `pong` | Server → Client | Keepalive response |

---

## 7. Conclusions

### 7.1 Key Findings

1. **EPICS dominance**: Most physics facilities use EPICS or EPICS-compatible systems
2. **Streaming essential**: Real-time data streaming is fundamental to physics instrumentation
3. **Multiple transports needed**: No single transport fits all use cases
4. **JSON viable**: JSON performance is acceptable for most physics applications
5. **Abstraction critical**: Transport abstraction enables flexibility

### 7.2 Recommendations

1. **Design transport-agnostic protocol** with adapters for WebSocket, TCP, gRPC
2. **Use JSON as primary format** for compatibility with existing schemas
3. **Support streaming and request-response** patterns
4. **Include optional binary mode** for high-volume data
5. **Align with EPICS concepts** (PV-like addressing, event-driven updates)

---

## 8. References

- [EPICS Official](https://epics-controls.org/)
- [TANGO Controls](https://www.tango-controls.org/)
- [ITER CODAC](https://www.iter.org/machine/supporting-systems/codac)
- [gRPC](https://grpc.io/)
- [WebSocket RFC 6455](https://tools.ietf.org/html/rfc6455)
- [MQTT](https://mqtt.org/)
- [ZeroMQ](https://zeromq.org/)
- [PvaPy Framework](https://pmc.ncbi.nlm.nih.gov/articles/PMC12067317/)

---

**弘益人間** 🤟

---

*Research conducted: 2025-12-14*
*Next: Protocol Specification (PHASE-3-PROTOCOL.md)*
