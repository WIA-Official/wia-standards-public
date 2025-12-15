"""
WIA Haptic Standard - Device Adapters
"""

import asyncio
import struct
import time
from typing import Optional, List, Any
from dataclasses import dataclass

from .types import (
    BodyLocation,
    ActuatorType,
    DeviceType,
    ConnectionState,
    HapticErrorCode,
    WaveformType,
)
from .device import (
    BaseHapticDevice,
    HapticCapabilities,
    DeviceConfig,
    HapticDeviceEvent,
    HapticError,
)
from .patterns import (
    HapticPattern,
    HapticPrimitive,
    HapticSequence,
    SpatialPattern,
    resolve_envelope,
)


# ============================================================================
# Bluetooth Adapter Configuration
# ============================================================================

@dataclass
class BluetoothConfig(DeviceConfig):
    """Bluetooth device configuration"""
    device_address: Optional[str] = None
    haptic_device_type: DeviceType = DeviceType.WRISTBAND
    service_uuid: str = "0000ff00-0000-1000-8000-00805f9b34fb"
    command_char_uuid: str = "0000ff01-0000-1000-8000-00805f9b34fb"
    response_char_uuid: str = "0000ff02-0000-1000-8000-00805f9b34fb"


# ============================================================================
# Message Types for Bluetooth Protocol
# ============================================================================

class MessageType:
    QUERY_CAPABILITIES = 0x01
    CAPABILITIES_RESPONSE = 0x02
    PLAY_PRIMITIVE = 0x10
    PLAY_SEQUENCE = 0x11
    STOP = 0x20
    SET_INTENSITY = 0x30
    PULSE = 0x31
    VIBRATE = 0x32
    ACK = 0xF0
    NACK = 0xF1


# ============================================================================
# Bluetooth Adapter
# ============================================================================

class BluetoothAdapter(BaseHapticDevice):
    """
    Bluetooth adapter for haptic devices.

    Requires `bleak` package for BLE communication:
        pip install bleak

    Example:
        adapter = BluetoothAdapter(BluetoothConfig(
            device_address="AA:BB:CC:DD:EE:FF"
        ))
        await adapter.connect()
        await adapter.play(STANDARD_PRIMITIVES["tick"])
    """

    def __init__(self, config: Optional[BluetoothConfig] = None):
        super().__init__(
            device_type=config.haptic_device_type if config else DeviceType.WRISTBAND,
            config=config or BluetoothConfig()
        )
        self._bt_config = config or BluetoothConfig()
        self._client: Any = None  # BleakClient instance
        self._response_event: Optional[asyncio.Event] = None
        self._response_data: Optional[bytes] = None

    async def connect(self) -> None:
        """Connect to Bluetooth device"""
        self._state = ConnectionState.CONNECTING

        try:
            # Import bleak here to make it optional
            try:
                from bleak import BleakClient, BleakScanner
            except ImportError:
                raise HapticError(
                    "bleak package required: pip install bleak",
                    HapticErrorCode.DEVICE_NOT_FOUND
                )

            # Find device
            address = self._bt_config.device_address
            if not address:
                # Scan for devices with WIA haptic service
                devices = await BleakScanner.discover(timeout=5.0)
                for device in devices:
                    if self._bt_config.service_uuid in (device.metadata.get("uuids", []) or []):
                        address = device.address
                        break

            if not address:
                raise HapticError(
                    "No haptic device found",
                    HapticErrorCode.DEVICE_NOT_FOUND
                )

            # Connect
            self._client = BleakClient(address)
            await asyncio.wait_for(
                self._client.connect(),
                timeout=self._config.connection_timeout / 1000
            )

            # Set up notifications
            await self._client.start_notify(
                self._bt_config.response_char_uuid,
                self._handle_notification
            )

            # Query capabilities
            self._capabilities = await self._query_capabilities()
            self._state = ConnectionState.CONNECTED

            self._emit(HapticDeviceEvent(
                type="connected",
                timestamp=time.time(),
                data={"address": address}
            ))

        except asyncio.TimeoutError:
            self._state = ConnectionState.ERROR
            raise HapticError(
                "Connection timeout",
                HapticErrorCode.CONNECTION_TIMEOUT
            )
        except Exception as e:
            self._state = ConnectionState.ERROR
            self._emit(HapticDeviceEvent(
                type="error",
                timestamp=time.time(),
                data=str(e)
            ))
            raise

    async def disconnect(self) -> None:
        """Disconnect from Bluetooth device"""
        self._state = ConnectionState.DISCONNECTING

        if self._client:
            try:
                await self._client.disconnect()
            except Exception:
                pass
            self._client = None

        self._capabilities = None
        self._state = ConnectionState.DISCONNECTED

        self._emit(HapticDeviceEvent(
            type="disconnected",
            timestamp=time.time()
        ))

    def _handle_notification(self, sender: Any, data: bytes) -> None:
        """Handle notification from device"""
        self._response_data = data
        if self._response_event:
            self._response_event.set()

    async def _send_command(
        self,
        message_type: int,
        payload: bytes = b""
    ) -> bytes:
        """Send command and wait for response"""
        if not self._client:
            raise HapticError("Not connected", HapticErrorCode.NOT_CONNECTED)

        # Build message
        message = self._build_message(message_type, payload)

        # Set up response event
        self._response_event = asyncio.Event()
        self._response_data = None

        # Send
        await self._client.write_gatt_char(
            self._bt_config.command_char_uuid,
            message
        )

        # Wait for response
        try:
            await asyncio.wait_for(self._response_event.wait(), timeout=5.0)
        except asyncio.TimeoutError:
            raise HapticError(
                "Command timeout",
                HapticErrorCode.CONNECTION_TIMEOUT
            )

        return self._response_data or b""

    def _build_message(self, message_type: int, payload: bytes) -> bytes:
        """Build protocol message"""
        header = struct.pack("<BBH", 0x01, message_type, len(payload))
        message = header + payload

        # Checksum (XOR of all bytes)
        checksum = 0
        for byte in message:
            checksum ^= byte

        return message + bytes([checksum])

    async def _query_capabilities(self) -> HapticCapabilities:
        """Query device capabilities"""
        response = await self._send_command(MessageType.QUERY_CAPABILITIES)

        # Parse response (simplified)
        # In real implementation, parse binary protocol
        return HapticCapabilities(
            actuator_type=ActuatorType.LRA,
            frequency_range=(30, 200),
            locations=["wrist_left_dorsal"],
            max_intensity=1.0,
            latency=30,
            actuator_count=1,
            supports_custom_waveforms=False,
            supports_amplitude_modulation=True,
            supports_frequency_modulation=False,
        )

    def _encode_primitive(self, primitive: HapticPrimitive) -> bytes:
        """Encode primitive to binary format"""
        envelope = resolve_envelope(primitive.envelope)
        intensity = self._scale_intensity(primitive.intensity)

        waveform_map = {
            WaveformType.SINE: 0,
            WaveformType.SQUARE: 1,
            WaveformType.TRIANGLE: 2,
            WaveformType.SAWTOOTH: 3,
            WaveformType.NOISE: 4,
        }

        waveform = primitive.waveform
        if hasattr(waveform, 'type'):
            waveform_type = waveform_map.get(waveform.type, 0)
        else:
            waveform_type = waveform_map.get(waveform, 0)

        # Pack binary data
        return struct.pack(
            "<BHBHHHBHf",
            waveform_type,
            int(primitive.frequency),
            int(intensity * 255),
            int(primitive.duration),
            int(envelope.attack),
            int(envelope.decay),
            int(envelope.sustain * 255),
            int(envelope.release),
            primitive.frequency  # Extra for compatibility
        )

    async def play(self, pattern: HapticPattern) -> None:
        """Play a haptic pattern"""
        self._assert_connected()
        self._is_playing = True

        try:
            if isinstance(pattern, HapticPrimitive):
                payload = self._encode_primitive(pattern)
                await self._send_command(MessageType.PLAY_PRIMITIVE, payload)
                await asyncio.sleep(pattern.duration / 1000)

            elif isinstance(pattern, HapticSequence):
                for step in pattern.steps:
                    if self._is_paused:
                        break

                    if step.delay_before > 0:
                        await asyncio.sleep(step.delay_before / 1000)

                    if step.primitive:
                        scaled_primitive = HapticPrimitive(
                            id=step.primitive.id,
                            name=step.primitive.name,
                            waveform=step.primitive.waveform,
                            envelope=step.primitive.envelope,
                            frequency=step.primitive.frequency,
                            intensity=step.primitive.intensity * step.intensity_scale,
                            duration=step.primitive.duration,
                            metadata=step.primitive.metadata,
                        )

                        for _ in range(step.repeat_count):
                            if self._is_paused:
                                break
                            await self.play(scaled_primitive)
                            if step.repeat_delay > 0:
                                await asyncio.sleep(step.repeat_delay / 1000)

            elif isinstance(pattern, SpatialPattern):
                for actuation in pattern.actuations:
                    if actuation.pattern:
                        if isinstance(actuation.pattern, (HapticPrimitive, HapticSequence)):
                            await self.play(actuation.pattern)

            self._emit(HapticDeviceEvent(
                type="pattern_complete",
                timestamp=time.time()
            ))

        finally:
            self._is_playing = False

    def stop(self) -> None:
        """Stop playback"""
        self._is_playing = False
        self._is_paused = False
        if self._client:
            asyncio.create_task(
                self._send_command(MessageType.STOP)
            )

    def set_intensity(self, location: BodyLocation, intensity: float) -> None:
        """Set intensity for a location"""
        self._assert_connected()
        scaled = self._scale_intensity(intensity)
        payload = struct.pack("<BB", 0, int(scaled * 255))
        asyncio.create_task(
            self._send_command(MessageType.SET_INTENSITY, payload)
        )

    def pulse(
        self,
        location: BodyLocation,
        duration: float,
        intensity: float = 0.5
    ) -> None:
        """Trigger a pulse"""
        self._assert_connected()
        scaled = self._scale_intensity(intensity)
        payload = struct.pack("<BHB", 0, int(duration), int(scaled * 255))
        asyncio.create_task(
            self._send_command(MessageType.PULSE, payload)
        )

    def vibrate(
        self,
        location: BodyLocation,
        frequency: float,
        intensity: float
    ) -> None:
        """Start continuous vibration"""
        self._assert_connected()
        scaled = self._scale_intensity(intensity)
        payload = struct.pack("<BHB", 0, int(frequency), int(scaled * 255))
        asyncio.create_task(
            self._send_command(MessageType.VIBRATE, payload)
        )


# ============================================================================
# Serial Adapter Configuration
# ============================================================================

@dataclass
class SerialConfig(DeviceConfig):
    """Serial device configuration"""
    port: Optional[str] = None
    baudrate: int = 115200
    haptic_device_type: DeviceType = DeviceType.WRISTBAND


# ============================================================================
# Serial Adapter
# ============================================================================

class SerialAdapter(BaseHapticDevice):
    """
    Serial adapter for haptic devices.

    Requires `pyserial` package:
        pip install pyserial

    Example:
        adapter = SerialAdapter(SerialConfig(port="/dev/ttyUSB0"))
        await adapter.connect()
        await adapter.play(STANDARD_PRIMITIVES["tick"])
    """

    def __init__(self, config: Optional[SerialConfig] = None):
        super().__init__(
            device_type=config.haptic_device_type if config else DeviceType.WRISTBAND,
            config=config or SerialConfig()
        )
        self._serial_config = config or SerialConfig()
        self._serial: Any = None

    async def connect(self) -> None:
        """Connect to serial device"""
        self._state = ConnectionState.CONNECTING

        try:
            import serial
            import serial.tools.list_ports
        except ImportError:
            raise HapticError(
                "pyserial package required: pip install pyserial",
                HapticErrorCode.DEVICE_NOT_FOUND
            )

        try:
            port = self._serial_config.port
            if not port:
                # Auto-detect
                ports = serial.tools.list_ports.comports()
                for p in ports:
                    if "haptic" in p.description.lower() or "wia" in p.description.lower():
                        port = p.device
                        break

            if not port:
                raise HapticError(
                    "No serial device found",
                    HapticErrorCode.DEVICE_NOT_FOUND
                )

            self._serial = serial.Serial(
                port,
                self._serial_config.baudrate,
                timeout=1.0
            )

            # Query capabilities
            self._capabilities = await self._query_capabilities()
            self._state = ConnectionState.CONNECTED

            self._emit(HapticDeviceEvent(
                type="connected",
                timestamp=time.time(),
                data={"port": port}
            ))

        except Exception as e:
            self._state = ConnectionState.ERROR
            raise HapticError(str(e), HapticErrorCode.CONNECTION_FAILED)

    async def disconnect(self) -> None:
        """Disconnect from serial device"""
        self._state = ConnectionState.DISCONNECTING

        if self._serial:
            self._serial.close()
            self._serial = None

        self._capabilities = None
        self._state = ConnectionState.DISCONNECTED

        self._emit(HapticDeviceEvent(
            type="disconnected",
            timestamp=time.time()
        ))

    async def _query_capabilities(self) -> HapticCapabilities:
        """Query device capabilities via serial"""
        # Send query command
        self._serial.write(b"\x01\x01\x00\x00\x00")
        await asyncio.sleep(0.1)

        # Read response (simplified)
        # Real implementation would parse actual response
        return HapticCapabilities(
            actuator_type=ActuatorType.ERM,
            frequency_range=(50, 200),
            locations=["wrist_left_dorsal"],
            max_intensity=1.0,
            latency=20,
            actuator_count=1,
            supports_custom_waveforms=False,
            supports_amplitude_modulation=True,
            supports_frequency_modulation=False,
        )

    async def play(self, pattern: HapticPattern) -> None:
        """Play a haptic pattern via serial"""
        self._assert_connected()
        self._is_playing = True

        try:
            if isinstance(pattern, HapticPrimitive):
                intensity = self._scale_intensity(pattern.intensity)
                # Send simple vibrate command
                cmd = struct.pack(
                    "<BBHBH",
                    0x10,  # Play primitive
                    int(pattern.frequency),
                    int(pattern.duration),
                    int(intensity * 255),
                    0  # Padding
                )
                self._serial.write(cmd)
                await asyncio.sleep(pattern.duration / 1000)

            elif isinstance(pattern, HapticSequence):
                for step in pattern.steps:
                    if step.delay_before > 0:
                        await asyncio.sleep(step.delay_before / 1000)
                    if step.primitive:
                        await self.play(step.primitive)

            self._emit(HapticDeviceEvent(
                type="pattern_complete",
                timestamp=time.time()
            ))

        finally:
            self._is_playing = False

    def stop(self) -> None:
        """Stop playback"""
        self._is_playing = False
        if self._serial:
            self._serial.write(b"\x20\x00\x00\x00\x00")  # Stop command

    def set_intensity(self, location: BodyLocation, intensity: float) -> None:
        """Set intensity"""
        if self._serial:
            scaled = self._scale_intensity(intensity)
            cmd = struct.pack("<BBB", 0x30, 0, int(scaled * 255))
            self._serial.write(cmd)

    def pulse(
        self,
        location: BodyLocation,
        duration: float,
        intensity: float = 0.5
    ) -> None:
        """Trigger a pulse"""
        if self._serial:
            scaled = self._scale_intensity(intensity)
            cmd = struct.pack("<BBHB", 0x31, 0, int(duration), int(scaled * 255))
            self._serial.write(cmd)

    def vibrate(
        self,
        location: BodyLocation,
        frequency: float,
        intensity: float
    ) -> None:
        """Start continuous vibration"""
        if self._serial:
            scaled = self._scale_intensity(intensity)
            cmd = struct.pack("<BBHB", 0x32, 0, int(frequency), int(scaled * 255))
            self._serial.write(cmd)
