# WIA Livestock Tracking Protocol Standard
## Phase 3 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**License**: MIT

---

## Table of Contents

1. [Communication Protocols](#communication-protocols)
2. [RFID Standards](#rfid-standards)
3. [LoRaWAN Integration](#lorawan-integration)
4. [Data Security](#data-security)
5. [Interoperability](#interoperability)

---

## Communication Protocols

### 1.1 Supported Protocols

| Protocol | Use Case | Range |
|----------|----------|-------|
| RFID (ISO 11784/11785) | Animal identification | 0-1m |
| LoRaWAN | GPS tracking | 2-15km |
| Bluetooth 5.0 | Local monitoring | 0-100m |
| MQTT | Real-time data | Internet |
| HTTPS/REST | API communication | Internet |

### 1.2 MQTT Topics

```
livestock/{farm_id}/{animal_id}/location
livestock/{farm_id}/{animal_id}/health
livestock/{farm_id}/{animal_id}/alerts
livestock/{farm_id}/herd/summary
```

**Example MQTT Message:**
```json
{
  "topic": "livestock/FARM-KR-001/CATTLE-KR-2025-001234/location",
  "payload": {
    "lat": 37.566535,
    "lng": 126.977969,
    "timestamp": "2025-01-15T14:22:00Z"
  },
  "qos": 1,
  "retain": false
}
```

---

## RFID Standards

### 2.1 ISO 11784/11785 Compliance

**Frequency:** 134.2 kHz (LF)
**Encoding:** FDX-B (Full Duplex)
**Read Range:** 0-30 cm

**Tag Structure:**
```
┌─────────────┬───────────────┬─────────────┐
│ Country Code│ Manufacturer  │  Animal ID  │
│   3 digits  │   3 digits    │  9 digits   │
└─────────────┴───────────────┴─────────────┘
Example: 982-000-123456789
```

### 2.2 EPC Gen2 (UHF RFID)

**Frequency:** 860-960 MHz
**Encoding:** EPC Gen2
**Read Range:** 1-10 meters

**EPC Code Structure:**
```
┌──────┬────────┬──────────┬────────────┐
│Header│ Manager│  Object  │   Serial   │
│ 8bit │ 28bit  │  24bit   │   36bit    │
└──────┴────────┴──────────┴────────────┘
```

---

## LoRaWAN Integration

### 3.1 Device Configuration

**LoRaWAN Parameters:**
```json
{
  "dev_eui": "0018B20000000001",
  "app_eui": "70B3D57ED0000001",
  "app_key": "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX",
  "frequency_plan": "AS923-1",
  "join_method": "OTAA",
  "class": "A",
  "data_rate": "SF7BW125"
}
```

### 3.2 GPS Payload Format

**Uplink Message (GPS Location):**
```
Byte 0: Message Type (0x01 = GPS)
Byte 1-4: Latitude (signed 32-bit)
Byte 5-8: Longitude (signed 32-bit)
Byte 9-10: Altitude (signed 16-bit meters)
Byte 11: Battery (0-100%)
Byte 12: Temperature (-128 to +127°C)
```

**Example:**
```
01 16 5A 0E 4B 78 9A BC DE 01 F4 55 12
```

### 3.3 Downlink Commands

**Command: Update Interval**
```
Byte 0: 0x02 (Command Type)
Byte 1-2: Interval in minutes (16-bit)
```

---

## Data Security

### 4.1 Encryption

- **Transport:** TLS 1.3
- **At Rest:** AES-256-GCM
- **LoRaWAN:** AES-128 (Network + Application Session Keys)

### 4.2 Authentication

```
API Key: SHA-256 hashed
JWT Tokens: RS256 algorithm
Expiry: 1 hour (access), 30 days (refresh)
```

### 4.3 Blockchain Integration

**Smart Contract (Solidity):**
```solidity
pragma solidity ^0.8.0;

contract LivestockRegistry {
    struct Animal {
        string animalId;
        string rfidTag;
        uint256 birthDate;
        address owner;
        bytes32 currentHash;
    }

    mapping(string => Animal) public animals;

    event AnimalRegistered(string animalId, address owner);
    event AnimalTransferred(string animalId, address from, address to);

    function registerAnimal(
        string memory animalId,
        string memory rfidTag,
        uint256 birthDate
    ) public {
        animals[animalId] = Animal(animalId, rfidTag, birthDate, msg.sender, bytes32(0));
        emit AnimalRegistered(animalId, msg.sender);
    }

    function updateHash(string memory animalId, bytes32 hash) public {
        require(animals[animalId].owner == msg.sender, "Not owner");
        animals[animalId].currentHash = hash;
    }
}
```

---

## Interoperability

### 5.1 GS1 Standards

**GTIN (Global Trade Item Number):**
```
Barcode: 8801234567890
Format: GS1-13
```

**EPCIS (Electronic Product Code Information Services):**
```xml
<epcis:EPCISDocument>
  <EPCISBody>
    <EventList>
      <ObjectEvent>
        <eventTime>2025-01-15T14:22:00Z</eventTime>
        <epcList>
          <epc>urn:epc:id:sgtin:8801234.056789.001234</epc>
        </epcList>
        <action>OBSERVE</action>
        <bizStep>urn:epcglobal:cbv:bizstep:feeding</bizStep>
        <readPoint>
          <id>urn:epc:id:sgln:8801234.00001.0</id>
        </readPoint>
      </ObjectEvent>
    </EventList>
  </EPCISBody>
</epcis:EPCISDocument>
```

### 5.2 ICAR (International Committee for Animal Recording)

**ICAR Animal ID Format:**
```
Country: KOR (Korea)
Herd Book: 01
Animal Number: 123456789
Full ID: KOR-01-123456789
```

### 5.3 Integration Points

| System | Protocol | Data Format |
|--------|----------|-------------|
| Farm Management Software | REST API | JSON |
| Veterinary Systems | HL7 FHIR | JSON/XML |
| Slaughterhouse | EDI | X12/EDIFACT |
| Retail | GS1 EPCIS | XML |
| Government Database | SOAP/REST | XML/JSON |

---

**弘益人間 (Hongik Ingan) - Benefit All Humanity**

*WIA-AGRI-009 Livestock Tracking Standard*
*© 2025 WIA - MIT License*
