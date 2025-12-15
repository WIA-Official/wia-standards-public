# WIA AAC Signal Format Validators

Validation scripts for WIA AAC Signal Format messages.

## Overview

This directory contains validation tools in TypeScript and Python for verifying that JSON messages conform to the WIA AAC Signal Format Standard.

## Installation

### TypeScript

```bash
cd examples/validators
npm install
```

### Python

```bash
cd examples/validators
pip install -r requirements.txt
```

## Usage

### TypeScript

```bash
# Validate a single file
npx ts-node validate.ts ../sample-data/eye-tracker-sample.json

# Validate all sample files
npm run validate:all

# Or directly
npx ts-node validate.ts ../sample-data/*.json
```

### Python

```bash
# Validate a single file
python validate.py ../sample-data/eye-tracker-sample.json

# Validate all sample files
python validate.py ../sample-data/*.json
```

## Output

The validators provide colored output indicating validation status:

```
WIA AAC Signal Format Validator
================================

✅ VALID: eye-tracker-sample.json
   Type: eye_tracker

✅ VALID: switch-sample.json
   Type: switch

❌ INVALID: broken-sample.json
   Type: unknown
   Errors:
    - /version: 'version' is a required property
    - /type: must be one of: eye_tracker, switch, ...

================================
Results: 2/3 files valid
```

## Exit Codes

- `0`: All files validated successfully
- `1`: One or more files failed validation

## Schema Files

The validators use JSON Schema files located in `/spec/schemas/`:

| File | Description |
|------|-------------|
| `wia-aac-signal-v1.schema.json` | Base message schema |
| `eye-tracker.schema.json` | Eye tracker data schema |
| `switch.schema.json` | Switch data schema |
| `muscle-sensor.schema.json` | Muscle sensor (EMG) data schema |
| `brain-interface.schema.json` | Brain interface (EEG/BCI) data schema |
| `breath.schema.json` | Breath (sip-and-puff) data schema |
| `head-movement.schema.json` | Head movement data schema |

## Programmatic Usage

### TypeScript

```typescript
import { WiaAacValidator } from './validate';

const validator = new WiaAacValidator();
const result = validator.validate({
  version: "1.0.0",
  type: "switch",
  timestamp: { unix_ms: Date.now() },
  device: { manufacturer: "Test", model: "Test" },
  data: { switch_id: 1, state: "pressed" }
});

console.log(result.valid); // true or false
console.log(result.errors); // array of error objects
```

### Python

```python
from validate import WiaAacValidator

validator = WiaAacValidator()
result = validator.validate({
    "version": "1.0.0",
    "type": "switch",
    "timestamp": {"unix_ms": 1702468800000},
    "device": {"manufacturer": "Test", "model": "Test"},
    "data": {"switch_id": 1, "state": "pressed"}
})

print(result.valid)   # True or False
print(result.errors)  # list of error strings
```

## License

MIT License - WIA / SmileStory Inc.
