"""
WIA AAC Mock Adapter
For testing and development purposes
"""

import random
import asyncio
from typing import Optional, List, Dict, Any

from .base_adapter import BaseSensorAdapter
from ..types import (
    SensorConfig,
    SensorOptionDescriptor
)


class MockAdapter(BaseSensorAdapter):
    """Mock adapter for testing"""

    def __init__(
        self,
        sensor_type: str = "eye_tracker",
        simulate_signals: bool = False,
        signal_interval: float = 0.1
    ):
        super().__init__()
        self._sensor_type = sensor_type
        self._simulate_signals = simulate_signals
        self._signal_interval = signal_interval
        self._simulation_task: Optional[asyncio.Task] = None

    @property
    def type(self) -> str:
        return self._sensor_type

    async def connect(self, config: SensorConfig) -> None:
        await super().connect(config)

        if self._simulate_signals:
            self._start_simulation()

    async def disconnect(self) -> None:
        self._stop_simulation()
        await super().disconnect()

    def get_supported_options(self) -> List[SensorOptionDescriptor]:
        return [
            SensorOptionDescriptor(
                name="sample_rate",
                type="number",
                description="Sample rate in Hz",
                default=60,
                min=1,
                max=1000
            ),
            SensorOptionDescriptor(
                name="dwell_time",
                type="number",
                description="Dwell time for selection in ms",
                default=1000,
                min=100,
                max=5000
            )
        ]

    def emit_mock_signal(self, data: Optional[Dict[str, Any]] = None) -> None:
        """Manually emit a mock signal"""
        signal_data = self._generate_signal_data()
        if data:
            signal_data.update(data)
        signal = self._create_signal(self._sensor_type, signal_data)
        self._emit_signal(signal)

    def _start_simulation(self) -> None:
        """Start signal simulation"""
        async def simulate():
            while self._is_connected:
                self.emit_mock_signal()
                await asyncio.sleep(self._signal_interval)

        try:
            loop = asyncio.get_event_loop()
            self._simulation_task = loop.create_task(simulate())
        except RuntimeError:
            pass  # No event loop running

    def _stop_simulation(self) -> None:
        """Stop signal simulation"""
        if self._simulation_task:
            self._simulation_task.cancel()
            self._simulation_task = None

    def _generate_signal_data(self) -> Dict[str, Any]:
        """Generate mock signal data based on sensor type"""
        if self._sensor_type == "eye_tracker":
            return self._generate_eye_tracker_data()
        elif self._sensor_type == "switch":
            return self._generate_switch_data()
        elif self._sensor_type == "muscle_sensor":
            return self._generate_muscle_sensor_data()
        elif self._sensor_type == "brain_interface":
            return self._generate_brain_interface_data()
        elif self._sensor_type == "breath":
            return self._generate_breath_data()
        elif self._sensor_type == "head_movement":
            return self._generate_head_movement_data()
        else:
            return {"custom_type": "mock", "custom_data": {}}

    def _generate_eye_tracker_data(self) -> Dict[str, Any]:
        return {
            "gaze": {
                "x": random.random(),
                "y": random.random(),
                "z": None
            },
            "fixation": {
                "active": random.random() > 0.7,
                "duration_ms": int(random.random() * 2000),
                "target_id": "mock_target"
            },
            "pupil": {
                "left_diameter_mm": 3 + random.random(),
                "right_diameter_mm": 3 + random.random()
            },
            "blink": {
                "detected": random.random() > 0.95,
                "duration_ms": 0
            },
            "eye_validity": {
                "left": True,
                "right": True
            }
        }

    def _generate_switch_data(self) -> Dict[str, Any]:
        return {
            "switch_id": 1,
            "channel": "primary",
            "state": "released",
            "duration_ms": 0,
            "pressure": None,
            "repeat_count": 1
        }

    def _generate_muscle_sensor_data(self) -> Dict[str, Any]:
        return {
            "channel_id": 1,
            "muscle_group": "cheek_left",
            "activation": random.random(),
            "raw_uv": random.random() * 200,
            "envelope_uv": random.random() * 100,
            "threshold_exceeded": random.random() > 0.8,
            "gesture": None
        }

    def _generate_brain_interface_data(self) -> Dict[str, Any]:
        return {
            "channel_count": 8,
            "sample_rate_hz": 250,
            "channels": [
                {"id": "Fp1", "value_uv": random.random() * 50 - 25},
                {"id": "Fp2", "value_uv": random.random() * 50 - 25},
                {"id": "C3", "value_uv": random.random() * 50 - 25},
                {"id": "C4", "value_uv": random.random() * 50 - 25},
                {"id": "P3", "value_uv": random.random() * 50 - 25},
                {"id": "P4", "value_uv": random.random() * 50 - 25},
                {"id": "O1", "value_uv": random.random() * 50 - 25},
                {"id": "O2", "value_uv": random.random() * 50 - 25}
            ],
            "bands": {
                "delta": 0.2,
                "theta": 0.2,
                "alpha": 0.3,
                "beta": 0.2,
                "gamma": 0.1
            },
            "classification": {
                "intent": "rest",
                "confidence": 0.7
            }
        }

    def _generate_breath_data(self) -> Dict[str, Any]:
        return {
            "action": "neutral",
            "pressure_kpa": 101.3,
            "pressure_normalized": 0.5,
            "duration_ms": 0,
            "intensity": "soft",
            "baseline_kpa": 101.3
        }

    def _generate_head_movement_data(self) -> Dict[str, Any]:
        return {
            "position": {
                "x": 0.5 + (random.random() - 0.5) * 0.1,
                "y": 0.5 + (random.random() - 0.5) * 0.1
            },
            "rotation": {
                "pitch": random.random() * 10 - 5,
                "yaw": random.random() * 10 - 5,
                "roll": random.random() * 5 - 2.5
            },
            "velocity": {
                "x": 0,
                "y": 0
            },
            "gesture": "none",
            "dwell_time_ms": 0,
            "face_detected": True
        }
