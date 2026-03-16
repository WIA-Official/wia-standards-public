# WIA-LANG PHASE 1: Data Format Specification

## Version 1.0 | 弘益人間 · Benefit All Humanity

## 1. Overview

This specification defines the standardized data formats for language preservation, documentation, and analysis within the WIA-LANG framework.

## 2. Core Data Structures

### 2.1 Language Metadata

```json
{
  "standard": "WIA-LANG-001",
  "version": "1.0",
  "language": {
    "name": "Example Language",
    "iso639-3": "exa",
    "family": "Language Family",
    "endangerment": "vulnerable|definitely|severely|critically|extinct",
    "speakers": {
      "native": 1500,
      "l2": 300,
      "lastUpdated": "2025-12-27"
    },
    "regions": ["Region A", "Region B"]
  }
}
```

### 2.2 Audio Recording Format

- **Container**: FLAC, WAV, MP3
- **Sample Rate**: 44.1kHz - 96kHz
- **Bit Depth**: 16-bit minimum, 24-bit recommended
- **Channels**: Mono or Stereo
- **Metadata**: Embedded EXIF/ID3 tags

### 2.3 Textual Data

- **Encoding**: UTF-8
- **Phonetic**: IPA (International Phonetic Alphabet)
- **Orthographic**: Native script or romanization
- **Glossing**: Leipzig Glossing Rules

## 3. File Naming Convention

```
{LANGUAGE-CODE}_{TYPE}_{DATE}_{SEQUENCE}.{EXT}
Example: ain_recording_20251227_001.flac
```

## 4. Storage Requirements

- **Redundancy**: Minimum 3 copies
- **Geographic Distribution**: 2+ locations
- **Encryption**: AES-256 at rest
- **Backup Frequency**: Daily incremental, weekly full

## 5. Interchange Formats

- JSON-LD for metadata
- CSV for tabular data
- XML (TEI) for annotated texts
- Protocol Buffers for streaming

## 6. Compliance

All implementations MUST support:
- UTF-8 encoding
- ISO 8601 timestamps
- RFC 3986 URI references
- JSON Schema validation

---
© 2025 SmileStory Inc. / WIA · Licensed under CC BY-SA 4.0
