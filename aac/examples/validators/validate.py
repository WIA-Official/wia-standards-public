#!/usr/bin/env python3
"""
WIA AAC Signal Format Validator

Python implementation for validating WIA AAC Signal messages
against JSON Schema.

Usage:
    python validate.py <file1.json> [file2.json] ...
    python validate.py ../sample-data/*.json

License: MIT
Author: WIA / SmileStory Inc.
"""

import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
from dataclasses import dataclass

try:
    from jsonschema import Draft7Validator, ValidationError
    from jsonschema.exceptions import SchemaError
except ImportError:
    print("Error: jsonschema library not found.")
    print("Please install it: pip install jsonschema>=4.20.0")
    sys.exit(1)


# Schema directory path
SCHEMA_DIR = Path(__file__).parent.parent.parent / "spec" / "schemas"

# Sensor type to schema file mapping
SENSOR_SCHEMA_MAP: Dict[str, str] = {
    "eye_tracker": "eye-tracker.schema.json",
    "switch": "switch.schema.json",
    "muscle_sensor": "muscle-sensor.schema.json",
    "brain_interface": "brain-interface.schema.json",
    "breath": "breath.schema.json",
    "head_movement": "head-movement.schema.json",
    "custom": "",
}


@dataclass
class ValidationResult:
    """Validation result container"""
    valid: bool
    file: str
    sensor_type: Optional[str] = None
    errors: Optional[List[str]] = None


class WiaAacValidator:
    """WIA AAC Signal Format Validator"""

    def __init__(self):
        self.base_schema: Optional[Dict[str, Any]] = None
        self.sensor_schemas: Dict[str, Dict[str, Any]] = {}
        self._load_schemas()

    def _load_schemas(self) -> None:
        """Load all JSON schemas"""
        # Load base schema
        base_schema_path = SCHEMA_DIR / "wia-aac-signal-v1.schema.json"
        if base_schema_path.exists():
            with open(base_schema_path, "r", encoding="utf-8") as f:
                self.base_schema = json.load(f)
                # Remove allOf for separate validation
                if "allOf" in self.base_schema:
                    del self.base_schema["allOf"]

        # Load sensor-specific schemas
        for sensor_type, schema_file in SENSOR_SCHEMA_MAP.items():
            if not schema_file:
                continue
            schema_path = SCHEMA_DIR / schema_file
            if schema_path.exists():
                with open(schema_path, "r", encoding="utf-8") as f:
                    self.sensor_schemas[sensor_type] = json.load(f)

    def validate(self, message: Any, file_path: str = "unknown") -> ValidationResult:
        """
        Validate a single message

        Args:
            message: The message to validate (parsed JSON)
            file_path: Path to the source file (for error reporting)

        Returns:
            ValidationResult with validation status and errors
        """
        result = ValidationResult(valid=False, file=file_path, errors=[])

        # Check if message is a dict
        if not isinstance(message, dict):
            result.errors = ["Message must be a JSON object"]
            return result

        # Get sensor type
        sensor_type = message.get("type")
        result.sensor_type = sensor_type

        # Validate against base schema
        if self.base_schema:
            try:
                validator = Draft7Validator(self.base_schema)
                base_errors = list(validator.iter_errors(message))
                if base_errors:
                    result.errors = [
                        f"{'/'.join(str(p) for p in err.absolute_path) or '/'}: {err.message}"
                        for err in base_errors
                    ]
                    return result
            except SchemaError as e:
                result.errors = [f"Schema error: {e.message}"]
                return result

        # Validate against sensor-specific schema
        if sensor_type in self.sensor_schemas:
            data = message.get("data", {})
            sensor_schema = self.sensor_schemas[sensor_type]

            try:
                validator = Draft7Validator(sensor_schema)
                sensor_errors = list(validator.iter_errors(data))
                if sensor_errors:
                    result.errors = [
                        f"data/{'/'.join(str(p) for p in err.absolute_path) or ''}: {err.message}"
                        for err in sensor_errors
                    ]
                    return result
            except SchemaError as e:
                result.errors = [f"Sensor schema error: {e.message}"]
                return result

        result.valid = True
        result.errors = None
        return result

    def validate_file(self, file_path: str) -> ValidationResult:
        """
        Validate a JSON file

        Args:
            file_path: Path to the JSON file

        Returns:
            ValidationResult with validation status and errors
        """
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                message = json.load(f)
            return self.validate(message, file_path)
        except json.JSONDecodeError as e:
            return ValidationResult(
                valid=False,
                file=file_path,
                errors=[f"JSON parse error: {e.msg} at line {e.lineno}"]
            )
        except FileNotFoundError:
            return ValidationResult(
                valid=False,
                file=file_path,
                errors=[f"File not found: {file_path}"]
            )
        except Exception as e:
            return ValidationResult(
                valid=False,
                file=file_path,
                errors=[f"Error reading file: {str(e)}"]
            )


def format_errors(errors: Optional[List[str]]) -> str:
    """Format validation errors for display"""
    if not errors:
        return ""
    return "\n".join(f"    - {err}" for err in errors)


def main() -> int:
    """Main function"""
    args = sys.argv[1:]

    if not args:
        print("WIA AAC Signal Format Validator")
        print("================================")
        print("")
        print("Usage: python validate.py <file1.json> [file2.json] ...")
        print("")
        print("Examples:")
        print("  python validate.py sample.json")
        print("  python validate.py ../sample-data/*.json")
        return 0

    validator = WiaAacValidator()
    all_valid = True
    total_files = 0
    valid_files = 0

    print("WIA AAC Signal Format Validator")
    print("================================")
    print()

    for file_path in args:
        total_files += 1
        result = validator.validate_file(file_path)
        file_name = Path(file_path).name

        if result.valid:
            valid_files += 1
            print(f"✅ VALID: {file_name}")
            print(f"   Type: {result.sensor_type}")
        else:
            all_valid = False
            print(f"❌ INVALID: {file_name}")
            print(f"   Type: {result.sensor_type or 'unknown'}")
            print(f"   Errors:")
            print(format_errors(result.errors))
        print()

    print("================================")
    print(f"Results: {valid_files}/{total_files} files valid")

    return 0 if all_valid else 1


if __name__ == "__main__":
    sys.exit(main())
