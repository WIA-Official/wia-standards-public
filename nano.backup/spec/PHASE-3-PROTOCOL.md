# WIA Nano Protocol (WNP) Specification

**Version**: 1.0.0
**Status**: Draft
**Phase**: 3 of 4
**Last Updated**: 2025-12-16

---

## 1. Overview

WIA Nano Protocol (WNP) defines the communication standards for nanoscale systems. It enables interoperability between nanorobots, nanosensors, nanomachines, and external controllers.

### 1.1 Goals

- Standardize message formats for nano-to-nano and nano-to-macro communication
- Support multiple transport methods (diffusion, guided, direct)
- Enable swarm coordination and quorum sensing
- Provide bio-nano interface communication patterns

### 1.2 Scope

This specification covers:
- Protocol message format and structure
- Message types and payloads
- Transport layer abstraction
- Molecular signaling protocols
- Swarm coordination mechanisms
- Error handling and recovery

---

## 2. Terminology

| Term | Definition |
|------|-----------|
| **Node** | Any entity capable of sending or receiving WNP messages |
| **Nanorobot** | Autonomous nanoscale device capable of locomotion |
| **Nanosensor** | Nanoscale device for environmental sensing |
| **Gateway** | Bridge between nano-network and external systems |
| **Autoinducer** | Signaling molecule for quorum sensing |
| **TTL** | Time-to-live, message expiration time |
| **Swarm** | Group of coordinated nano devices |

---

## 3. Message Format

### 3.1 Base Message Structure

```json
{
  "protocol": "wia-nano",
  "version": "1.0.0",
  "messageId": "uuid-v4",
  "timestamp": 1702483200000,
  "type": "message_type",
  "source": {
    "id": "source-node-id",
    "type": "nanorobot",
    "location_nm": {"x": 0, "y": 0, "z": 0}
  },
  "destination": {
    "id": "dest-node-id",
    "type": "cell",
    "broadcast": false
  },
  "transport": {
    "method": "diffusion",
    "parameters": {}
  },
  "payload": {},
  "ttl": 3600,
  "checksum": "crc32"
}
```

### 3.2 Field Descriptions

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `protocol` | string | Yes | Protocol identifier, always "wia-nano" |
| `version` | string | Yes | Protocol version (semver) |
| `messageId` | string | Yes | Unique message identifier (UUID v4) |
| `timestamp` | number | Yes | Unix timestamp in milliseconds |
| `type` | string | Yes | Message type (see Section 4) |
| `source` | object | Yes | Source node information |
| `destination` | object | Yes | Destination node information |
| `transport` | object | Yes | Transport method and parameters |
| `payload` | object | Yes | Message-type-specific data |
| `ttl` | number | No | Time-to-live in seconds (default: 3600) |
| `checksum` | string | No | CRC32 checksum for integrity |

### 3.3 Node Information

```json
{
  "id": "node-unique-id",
  "type": "nanorobot",
  "location_nm": {
    "x": 100.5,
    "y": 200.3,
    "z": 50.0
  }
}
```

#### Node Types

| Type | Description | Example |
|------|-------------|---------|
| `nanorobot` | Autonomous nano device | DNA origami robot |
| `nanomachine` | Molecular machine | ATP synthase |
| `nanosensor` | Sensing device | Quantum dot sensor |
| `controller` | External controller | Magnetic field controller |
| `cell` | Biological cell | Cancer cell target |
| `gateway` | Network gateway | Body-external bridge |

---

## 4. Message Types

### 4.1 Signal Messages

For molecular signaling and quorum sensing.

```json
{
  "type": "signal",
  "payload": {
    "signal_type": "quorum_sensing",
    "molecule": "AHL",
    "concentration_nm": 100,
    "threshold_nm": 50,
    "action_triggered": true,
    "collective_behavior": "biofilm_formation"
  }
}
```

### 4.2 Command Messages

For device control and task assignment.

```json
{
  "type": "command",
  "payload": {
    "command_id": "cmd-001",
    "command_type": "move",
    "parameters": {
      "target_position": {"x": 500, "y": 300, "z": 100},
      "speed_nm_per_s": 10
    },
    "timeout_ms": 60000
  }
}
```

#### Command Types

- `move` - Navigate to position
- `stop` - Halt current operation
- `release` - Release payload/cargo
- `sense` - Perform sensing operation
- `configure` - Update configuration
- `shutdown` - Power down device

### 4.3 Telemetry Messages

For status and sensor data reporting.

```json
{
  "type": "telemetry",
  "payload": {
    "sensor_readings": {
      "ph": 7.4,
      "temperature_k": 310,
      "glucose_mm": 5.5
    },
    "device_status": {
      "energy_level": 0.85,
      "operational_state": "active",
      "error_code": null
    },
    "position_nm": {"x": 150, "y": 220, "z": 75}
  }
}
```

### 4.4 Acknowledgment Messages

For delivery confirmation.

```json
{
  "type": "acknowledgment",
  "payload": {
    "ack_message_id": "original-msg-uuid",
    "status": "received",
    "execution_result": "success"
  }
}
```

### 4.5 Coordination Messages

For swarm synchronization.

```json
{
  "type": "coordination",
  "payload": {
    "swarm_id": "swarm-001",
    "action": "formation",
    "formation_type": "circle",
    "center_nm": {"x": 1000, "y": 1000, "z": 500},
    "radius_nm": 200,
    "member_count": 50
  }
}
```

### 4.6 Emergency Messages

For urgent alerts and safety-critical events.

```json
{
  "type": "emergency",
  "payload": {
    "emergency_type": "toxin_detected",
    "severity": 9,
    "location_nm": {"x": 500, "y": 600, "z": 300},
    "detected_substance": "reactive_oxygen_species",
    "concentration": "critical",
    "recommended_action": "evacuate_area"
  }
}
```

---

## 5. Transport Methods

### 5.1 Diffusion-Based Transport

Primary method for nano-to-nano communication in fluid environments.

```json
{
  "transport": {
    "method": "diffusion",
    "parameters": {
      "carrier_molecule": "acetylcholine",
      "diffusion_coefficient_m2_per_s": 4e-10,
      "medium_viscosity_pa_s": 0.001,
      "temperature_k": 310
    }
  }
}
```

#### Physical Model

- **Fick's First Law**: J = -D × (∂C/∂x)
- **Mean Square Displacement**: <x²> = 2Dt (1D)
- **Diffusion Time**: t = d² / (2D)

### 5.2 Guided Transport

For directed communication using external fields.

```json
{
  "transport": {
    "method": "guided",
    "parameters": {
      "guidance_type": "magnetic_field",
      "field_strength_tesla": 0.1,
      "gradient_t_per_m": 0.01,
      "frequency_hz": 10
    }
  }
}
```

#### Guidance Types

- `magnetic_field` - Magnetic field guidance
- `acoustic` - Ultrasound-based guidance
- `optical` - Light-based guidance (optical tweezers)
- `chemical_gradient` - Chemotaxis-like guidance

### 5.3 Direct Transfer

For contact-based communication.

```json
{
  "transport": {
    "method": "direct",
    "parameters": {
      "mechanism": "gap_junction",
      "channel_conductance_ps": 100,
      "contact_duration_ms": 50
    }
  }
}
```

#### Direct Transfer Mechanisms

- `gap_junction` - Cell-cell gap junctions
- `nanotube` - Carbon nanotube channels
- `conjugation` - Bacterial conjugation-like transfer
- `vesicle` - Vesicle-mediated transfer

---

## 6. Molecular Signaling Protocol

### 6.1 Quorum Sensing Protocol

Enables population density-dependent collective behavior.

#### Message Flow

```
1. Nano devices continuously produce signal molecules
2. Local concentration increases with device density
3. When concentration exceeds threshold:
   - Trigger collective behavior
   - Broadcast coordination message
4. All nearby devices synchronize action
```

#### Quorum Sensing Message

```json
{
  "type": "signal",
  "payload": {
    "signal_type": "quorum_sensing",
    "molecule": "AHL",
    "concentration_nm": 150,
    "threshold_nm": 100,
    "action_triggered": true,
    "collective_behavior": "target_attack",
    "participants": 47
  }
}
```

### 6.2 DNA-Based Logic

For computational signaling.

```json
{
  "type": "command",
  "payload": {
    "logic_operation": "AND",
    "input_strands": [
      {"name": "input_A", "sequence": "ATCGATCG"},
      {"name": "input_B", "sequence": "GCTAGCTA"}
    ],
    "output_strand": {
      "name": "output_C",
      "sequence": "TTAACCGG"
    },
    "reaction_time_sec": 60,
    "expected_success_rate": 0.95
  }
}
```

---

## 7. Bio-Nano Interface

### 7.1 Cell Targeting Message

```json
{
  "type": "signal",
  "destination": {
    "type": "cell",
    "cell_type": "cancer_cell",
    "marker": "folate_receptor"
  },
  "payload": {
    "targeting": {
      "ligand": "folate",
      "affinity_kd_nm": 0.1,
      "binding_site": "cell_surface",
      "expected_binding_time_s": 30
    },
    "cargo": {
      "type": "therapeutic",
      "drug": "doxorubicin",
      "dose_molecules": 1000,
      "release_trigger": "ph_6.5"
    }
  }
}
```

### 7.2 Endocytosis Notification

```json
{
  "type": "telemetry",
  "payload": {
    "event": "endocytosis_initiated",
    "mechanism": "clathrin_mediated",
    "cell_id": "target-cell-001",
    "estimated_internalization_time_s": 120,
    "cargo_status": "intact"
  }
}
```

---

## 8. Nano Network

### 8.1 Network Topology

WNP supports multiple network topologies:

| Topology | Description | Use Case |
|----------|-------------|----------|
| Star | Gateway-centered | External control |
| Mesh | Peer-to-peer | Swarm operations |
| Tree | Hierarchical | Leader-follower |
| Ad-hoc | Dynamic | Exploration |

### 8.2 Routing

#### Broadcast Routing

```json
{
  "destination": {
    "id": null,
    "broadcast": true,
    "range_nm": 1000
  }
}
```

#### Geographic Routing

```json
{
  "destination": {
    "id": null,
    "broadcast": false,
    "target_region": {
      "center_nm": {"x": 500, "y": 500, "z": 250},
      "radius_nm": 200
    }
  }
}
```

---

## 9. Error Handling

### 9.1 Error Codes

| Range | Category | Description |
|-------|----------|-------------|
| 1000-1999 | Diffusion | Transport errors |
| 2000-2999 | Molecular | Signal molecule errors |
| 3000-3999 | Network | Routing/connectivity |
| 4000-4999 | Bio-Interface | Cell interaction errors |
| 5000-5999 | System | Device/system errors |

### 9.2 Error Message Format

```json
{
  "type": "error",
  "payload": {
    "error_code": 1001,
    "error_category": "diffusion",
    "error_message": "Signal attenuation exceeded threshold",
    "details": {
      "expected_concentration_nm": 50,
      "actual_concentration_nm": 5,
      "distance_nm": 5000
    },
    "recoverable": true,
    "suggested_action": "increase_signal_strength"
  }
}
```

### 9.3 Common Error Codes

| Code | Name | Description |
|------|------|-------------|
| 1001 | `SIGNAL_ATTENUATED` | Signal too weak |
| 1002 | `DIFFUSION_TIMEOUT` | Diffusion time exceeded |
| 2001 | `MOLECULE_DEGRADED` | Signal molecule degraded |
| 2002 | `RECEPTOR_SATURATED` | Receptor binding sites full |
| 3001 | `NODE_UNREACHABLE` | Cannot reach destination |
| 3002 | `SWARM_DISCONNECTED` | Lost swarm connection |
| 4001 | `CELL_MEMBRANE_BLOCKED` | Cannot penetrate cell |
| 4002 | `IMMUNE_RESPONSE` | Immune system detected |
| 5001 | `ENERGY_DEPLETED` | Device out of energy |
| 5002 | `COMPONENT_FAILURE` | Hardware failure |

---

## 10. Security Considerations

### 10.1 Message Authentication

For sensitive operations, messages should include authentication:

```json
{
  "security": {
    "authentication": "dna_signature",
    "signature_sequence": "ATCGATCGATCG",
    "encryption": "molecular_key",
    "integrity_check": "crc32"
  }
}
```

### 10.2 Threat Model

- **Spoofing**: Malicious molecules mimicking valid signals
- **Jamming**: Flooding environment with noise molecules
- **Eavesdropping**: Intercepting molecular signals
- **Replay**: Re-transmitting captured signals

---

## 11. Examples

### 11.1 Drug Delivery Scenario

```json
{
  "protocol": "wia-nano",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": 1702483200000,
  "type": "command",
  "source": {
    "id": "controller-001",
    "type": "gateway"
  },
  "destination": {
    "id": "nanobot-fleet-A",
    "type": "nanorobot",
    "broadcast": true
  },
  "transport": {
    "method": "guided",
    "parameters": {
      "guidance_type": "magnetic_field",
      "field_strength_tesla": 0.05
    }
  },
  "payload": {
    "command_id": "deliver-001",
    "command_type": "release",
    "parameters": {
      "target_cell_type": "tumor_cell",
      "release_trigger": "ph_below_6.5",
      "cargo": "doxorubicin"
    }
  },
  "ttl": 7200
}
```

### 11.2 Swarm Formation

```json
{
  "protocol": "wia-nano",
  "version": "1.0.0",
  "messageId": "550e8400-e29b-41d4-a716-446655440001",
  "timestamp": 1702483260000,
  "type": "coordination",
  "source": {
    "id": "leader-bot-001",
    "type": "nanorobot",
    "location_nm": {"x": 1000, "y": 1000, "z": 500}
  },
  "destination": {
    "broadcast": true
  },
  "transport": {
    "method": "diffusion",
    "parameters": {
      "carrier_molecule": "AHL",
      "diffusion_coefficient_m2_per_s": 4.9e-10
    }
  },
  "payload": {
    "swarm_id": "swarm-alpha",
    "action": "formation",
    "formation_type": "sphere",
    "center_nm": {"x": 1000, "y": 1000, "z": 500},
    "radius_nm": 500,
    "spacing_nm": 50
  },
  "ttl": 300
}
```

---

## 12. References

1. IEEE 1906.1-2015 - Recommended Practice for Nanoscale and Molecular Communication Framework
2. Pierobon, M. & Akyildiz, I.F. "Fundamentals of Diffusion-Based Molecular Communication"
3. Douglas, S.M. et al. "A Logic-Gated Nanorobot for Targeted Transport of Molecular Payloads"
4. Bassler, B.L. "How Bacteria Talk to Each Other"
5. Nature Nanotechnology - "Key principles for studying endocytosis of nanoparticle therapeutics"

---

## Appendix A: JSON Schema References

- Message Schema: `/spec/schemas/wnp-message.schema.json`
- Error Schema: `/spec/schemas/wnp-error.schema.json`

## Appendix B: Rust API Reference

- Protocol Module: `/api/rust/src/protocol/`
- Transport Module: `/api/rust/src/transport/`

---

**WIA Nano Protocol v1.0.0**
**弘益人間 - Benefit All Humanity**
