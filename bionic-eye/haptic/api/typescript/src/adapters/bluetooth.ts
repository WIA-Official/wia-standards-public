/**
 * WIA Haptic Standard - Bluetooth Adapter
 * @version 1.0.0
 *
 * Generic adapter for Bluetooth-connected haptic devices
 * Supports both BLE (Bluetooth Low Energy) and Classic Bluetooth
 */

import { BodyLocation, DeviceType } from '../types';
import {
  BaseHapticDevice,
  HapticCapabilities,
  DeviceConfig,
  HapticError,
} from '../device';
import {
  HapticPattern,
  HapticPrimitive,
  HapticSequence,
  resolveEnvelope,
  resolveWaveform,
} from '../patterns';

// ============================================================================
// Bluetooth Protocol Types
// ============================================================================

/**
 * WIA Haptic Bluetooth Protocol Message
 */
interface WiaHapticMessage {
  version: number;       // Protocol version
  type: MessageType;
  payload: Uint8Array;
  checksum: number;
}

enum MessageType {
  QUERY_CAPABILITIES = 0x01,
  CAPABILITIES_RESPONSE = 0x02,
  PLAY_PRIMITIVE = 0x10,
  PLAY_SEQUENCE = 0x11,
  STOP = 0x20,
  SET_INTENSITY = 0x30,
  PULSE = 0x31,
  VIBRATE = 0x32,
  ACK = 0xF0,
  NACK = 0xF1,
  BATTERY_STATUS = 0xE0,
}

// ============================================================================
// Bluetooth Service UUIDs
// ============================================================================

const WIA_HAPTIC_SERVICE_UUID = '0000ff00-0000-1000-8000-00805f9b34fb';
const WIA_HAPTIC_COMMAND_CHAR_UUID = '0000ff01-0000-1000-8000-00805f9b34fb';
const WIA_HAPTIC_RESPONSE_CHAR_UUID = '0000ff02-0000-1000-8000-00805f9b34fb';
const WIA_HAPTIC_BATTERY_CHAR_UUID = '0000ff03-0000-1000-8000-00805f9b34fb';

// ============================================================================
// Bluetooth Device Configuration
// ============================================================================

export interface BluetoothDeviceConfig extends DeviceConfig {
  /** Bluetooth device address or name */
  deviceAddress?: string;

  /** Device type for capability inference */
  hapticDeviceType?: DeviceType;

  /** Custom service UUID */
  serviceUuid?: string;

  /** Custom command characteristic UUID */
  commandCharUuid?: string;

  /** Custom response characteristic UUID */
  responseCharUuid?: string;

  /** Use BLE instead of classic Bluetooth */
  useBLE?: boolean;

  /** MTU size for BLE */
  mtuSize?: number;
}

// ============================================================================
// Web Bluetooth Types
// ============================================================================

interface BluetoothDevice {
  id: string;
  name?: string;
  gatt?: BluetoothRemoteGATTServer;
}

interface BluetoothRemoteGATTServer {
  connected: boolean;
  connect(): Promise<BluetoothRemoteGATTServer>;
  disconnect(): void;
  getPrimaryService(uuid: string): Promise<BluetoothRemoteGATTService>;
}

interface BluetoothRemoteGATTService {
  getCharacteristic(uuid: string): Promise<BluetoothRemoteGATTCharacteristic>;
}

interface BluetoothRemoteGATTCharacteristic {
  writeValue(value: BufferSource): Promise<void>;
  readValue(): Promise<DataView>;
  startNotifications(): Promise<BluetoothRemoteGATTCharacteristic>;
  addEventListener(type: string, listener: (event: any) => void): void;
}

declare const navigator: {
  bluetooth?: {
    requestDevice(options: any): Promise<BluetoothDevice>;
  };
};

// ============================================================================
// Bluetooth Haptic Adapter
// ============================================================================

export class BluetoothHapticAdapter extends BaseHapticDevice {
  private device: BluetoothDevice | null = null;
  private gattServer: BluetoothRemoteGATTServer | null = null;
  private commandChar: BluetoothRemoteGATTCharacteristic | null = null;
  private responseChar: BluetoothRemoteGATTCharacteristic | null = null;
  private batteryChar: BluetoothRemoteGATTCharacteristic | null = null;
  private btConfig: BluetoothDeviceConfig;

  constructor(config: BluetoothDeviceConfig = {}) {
    super(config.hapticDeviceType ?? 'wristband', {
      name: 'Bluetooth Haptic Device',
      ...config,
    });

    this.btConfig = {
      useBLE: true,
      mtuSize: 512,
      serviceUuid: WIA_HAPTIC_SERVICE_UUID,
      commandCharUuid: WIA_HAPTIC_COMMAND_CHAR_UUID,
      responseCharUuid: WIA_HAPTIC_RESPONSE_CHAR_UUID,
      ...config,
    };
  }

  // -------------------------------------------------------------------------
  // Connection
  // -------------------------------------------------------------------------

  async connect(): Promise<void> {
    this._state = 'connecting';

    try {
      if (!navigator.bluetooth) {
        throw new HapticError(
          'Web Bluetooth not available',
          'BLUETOOTH_DISABLED'
        );
      }

      // Request device
      this.device = await navigator.bluetooth.requestDevice({
        filters: this.btConfig.deviceAddress
          ? [{ name: this.btConfig.deviceAddress }]
          : [{ services: [this.btConfig.serviceUuid!] }],
        optionalServices: [this.btConfig.serviceUuid!],
      });

      if (!this.device.gatt) {
        throw new HapticError('GATT not available', 'CONNECTION_FAILED');
      }

      // Connect to GATT server
      this.gattServer = await this.device.gatt.connect();

      // Get service and characteristics
      const service = await this.gattServer.getPrimaryService(
        this.btConfig.serviceUuid!
      );

      this.commandChar = await service.getCharacteristic(
        this.btConfig.commandCharUuid!
      );

      this.responseChar = await service.getCharacteristic(
        this.btConfig.responseCharUuid!
      );

      // Set up notifications for responses
      await this.responseChar.startNotifications();
      this.responseChar.addEventListener(
        'characteristicvaluechanged',
        this.handleResponse.bind(this)
      );

      // Try to get battery characteristic
      try {
        this.batteryChar = await service.getCharacteristic(
          WIA_HAPTIC_BATTERY_CHAR_UUID
        );
      } catch {
        // Battery characteristic optional
      }

      // Query capabilities
      this._capabilities = await this.queryCapabilities();
      this._state = 'connected';

      this.emit({
        type: 'connected',
        timestamp: Date.now(),
        data: { deviceId: this.device.id, deviceName: this.device.name },
      });
    } catch (error) {
      this._state = 'error';
      this.emit({
        type: 'error',
        timestamp: Date.now(),
        data: error,
      });
      throw error;
    }
  }

  async disconnect(): Promise<void> {
    this._state = 'disconnecting';

    if (this.gattServer?.connected) {
      this.gattServer.disconnect();
    }

    this.device = null;
    this.gattServer = null;
    this.commandChar = null;
    this.responseChar = null;
    this.batteryChar = null;
    this._capabilities = null;
    this._state = 'disconnected';

    this.emit({
      type: 'disconnected',
      timestamp: Date.now(),
    });
  }

  private async queryCapabilities(): Promise<HapticCapabilities> {
    const response = await this.sendCommand(MessageType.QUERY_CAPABILITIES, new Uint8Array(0));

    // Parse capabilities from response
    const caps = this.parseCapabilitiesResponse(response);

    // Get battery level if available
    let batteryLevel: number | null = null;
    if (this.batteryChar) {
      try {
        const batteryData = await this.batteryChar.readValue();
        batteryLevel = batteryData.getUint8(0);
      } catch {
        // Ignore battery read errors
      }
    }

    return {
      ...caps,
      batteryLevel,
    };
  }

  private parseCapabilitiesResponse(data: Uint8Array): Omit<HapticCapabilities, 'batteryLevel'> {
    // Default capabilities for unknown devices
    const defaults: Omit<HapticCapabilities, 'batteryLevel'> = {
      actuatorType: 'erm',
      frequencyRange: { min: 50, max: 200 },
      locations: ['wrist_left_dorsal'],
      maxIntensity: 1.0,
      latency: 30,
      actuatorCount: 1,
      supportsCustomWaveforms: false,
      supportsAmplitudeModulation: true,
      supportsFrequencyModulation: false,
    };

    if (data.length < 10) {
      return defaults;
    }

    // Parse binary capabilities response
    // Format: [actuatorType, freqMin(2), freqMax(2), locationCount, locations..., flags]
    const view = new DataView(data.buffer);

    const actuatorTypes: Record<number, HapticCapabilities['actuatorType']> = {
      0: 'erm',
      1: 'lra',
      2: 'piezo',
      3: 'voice_coil',
    };

    const actuatorType = actuatorTypes[data[0]] ?? 'erm';
    const freqMin = view.getUint16(1, true);
    const freqMax = view.getUint16(3, true);
    const locationCount = data[5];

    const locationIds = Array.from(data.slice(6, 6 + locationCount));
    const locations = this.mapLocationIds(locationIds);

    const flagsByte = data[6 + locationCount] ?? 0;
    const supportsCustomWaveforms = (flagsByte & 0x01) !== 0;
    const supportsAmplitudeModulation = (flagsByte & 0x02) !== 0;
    const supportsFrequencyModulation = (flagsByte & 0x04) !== 0;

    return {
      actuatorType,
      frequencyRange: { min: freqMin, max: freqMax },
      locations,
      maxIntensity: 1.0,
      latency: 30,
      actuatorCount: locationCount,
      supportsCustomWaveforms,
      supportsAmplitudeModulation,
      supportsFrequencyModulation,
    };
  }

  private mapLocationIds(ids: number[]): BodyLocation[] {
    const locationMap: Record<number, BodyLocation> = {
      0: 'wrist_left_dorsal',
      1: 'wrist_right_dorsal',
      2: 'palm_left',
      3: 'palm_right',
      4: 'chest_center',
      5: 'back_upper_center',
      // ... extend as needed
    };

    return ids
      .map(id => locationMap[id])
      .filter((loc): loc is BodyLocation => loc !== undefined);
  }

  // -------------------------------------------------------------------------
  // Communication
  // -------------------------------------------------------------------------

  private async sendCommand(type: MessageType, payload: Uint8Array): Promise<Uint8Array> {
    if (!this.commandChar) {
      throw new HapticError('Not connected', 'NOT_CONNECTED');
    }

    const message = this.buildMessage(type, payload);
    await this.commandChar.writeValue(message);

    // Wait for response (simplified - real implementation would use promises)
    return new Promise((resolve, reject) => {
      const timeout = setTimeout(() => {
        reject(new HapticError('Command timeout', 'CONNECTION_TIMEOUT'));
      }, 5000);

      this.pendingResponse = { resolve, reject, timeout };
    });
  }

  private pendingResponse: {
    resolve: (data: Uint8Array) => void;
    reject: (error: Error) => void;
    timeout: NodeJS.Timeout;
  } | null = null;

  private handleResponse(event: any): void {
    const value = event.target.value as DataView;
    const data = new Uint8Array(value.buffer);

    if (this.pendingResponse) {
      clearTimeout(this.pendingResponse.timeout);
      this.pendingResponse.resolve(data);
      this.pendingResponse = null;
    }
  }

  private buildMessage(type: MessageType, payload: Uint8Array): Uint8Array {
    const message = new Uint8Array(4 + payload.length);
    message[0] = 0x01; // Protocol version
    message[1] = type;
    message[2] = payload.length & 0xFF;
    message[3] = (payload.length >> 8) & 0xFF;
    message.set(payload, 4);

    // Calculate checksum (simple XOR)
    let checksum = 0;
    for (const byte of message) {
      checksum ^= byte;
    }

    const finalMessage = new Uint8Array(message.length + 1);
    finalMessage.set(message);
    finalMessage[finalMessage.length - 1] = checksum;

    return finalMessage;
  }

  // -------------------------------------------------------------------------
  // Pattern Encoding
  // -------------------------------------------------------------------------

  private encodePrimitive(primitive: HapticPrimitive): Uint8Array {
    const envelope = resolveEnvelope(primitive.envelope);
    const waveform = resolveWaveform(primitive.waveform);
    const intensity = this.scaleIntensity(primitive.intensity);

    // Encode primitive to binary format
    // Format: [waveformType, frequency(2), intensity, duration(2), attack(2), decay(2), sustain, release(2)]
    const buffer = new ArrayBuffer(14);
    const view = new DataView(buffer);

    const waveformTypes: Record<string, number> = {
      sine: 0, square: 1, triangle: 2, sawtooth: 3, noise: 4,
    };

    view.setUint8(0, waveformTypes[waveform.type] ?? 0);
    view.setUint16(1, primitive.frequency, true);
    view.setUint8(3, Math.round(intensity * 255));
    view.setUint16(4, primitive.duration, true);
    view.setUint16(6, envelope.attack, true);
    view.setUint16(8, envelope.decay, true);
    view.setUint8(10, Math.round(envelope.sustain * 255));
    view.setUint16(11, envelope.release, true);

    return new Uint8Array(buffer);
  }

  private encodeSequence(sequence: HapticSequence): Uint8Array {
    const parts: Uint8Array[] = [];

    // Header: [stepCount, loop, loopCount]
    const header = new Uint8Array(3);
    header[0] = sequence.steps.length;
    header[1] = sequence.loop ? 1 : 0;
    header[2] = sequence.loopCount ?? 0;
    parts.push(header);

    // Encode each step
    for (const step of sequence.steps) {
      // Step header: [delayBefore(2), repeatCount, repeatDelay(2), intensityScale]
      const stepHeader = new Uint8Array(6);
      const stepView = new DataView(stepHeader.buffer);
      stepView.setUint16(0, step.delayBefore ?? 0, true);
      stepView.setUint8(2, step.repeatCount ?? 1);
      stepView.setUint16(3, step.repeatDelay ?? 0, true);
      stepView.setUint8(5, Math.round((step.intensityScale ?? 1) * 255));
      parts.push(stepHeader);

      // Encode primitive if present
      if (step.primitive) {
        const primitiveData = this.encodePrimitive(step.primitive);
        parts.push(primitiveData);
      }
    }

    // Concatenate all parts
    const totalLength = parts.reduce((sum, p) => sum + p.length, 0);
    const result = new Uint8Array(totalLength);
    let offset = 0;
    for (const part of parts) {
      result.set(part, offset);
      offset += part.length;
    }

    return result;
  }

  // -------------------------------------------------------------------------
  // Playback
  // -------------------------------------------------------------------------

  async play(pattern: HapticPattern): Promise<void> {
    this.assertConnected();

    let payload: Uint8Array;
    let messageType: MessageType;

    if ('waveform' in pattern) {
      payload = this.encodePrimitive(pattern as HapticPrimitive);
      messageType = MessageType.PLAY_PRIMITIVE;
    } else if ('steps' in pattern) {
      payload = this.encodeSequence(pattern as HapticSequence);
      messageType = MessageType.PLAY_SEQUENCE;
    } else {
      // Spatial pattern - encode first actuation
      const spatial = pattern as any;
      if (spatial.actuations?.[0]?.pattern) {
        return this.play(spatial.actuations[0].pattern);
      }
      return;
    }

    await this.sendCommand(messageType, payload);
    this._isPlaying = true;

    const duration = this.estimateDuration(pattern);
    setTimeout(() => {
      this._isPlaying = false;
      this.emit({
        type: 'pattern_complete',
        timestamp: Date.now(),
      });
    }, duration);
  }

  stop(): void {
    if (this.commandChar) {
      this.sendCommand(MessageType.STOP, new Uint8Array(0)).catch(() => {});
    }
    this._isPlaying = false;
  }

  // -------------------------------------------------------------------------
  // Real-time Control
  // -------------------------------------------------------------------------

  setIntensity(location: BodyLocation, intensity: number): void {
    this.assertConnected();

    const payload = new Uint8Array(2);
    payload[0] = this.locationToId(location);
    payload[1] = Math.round(this.scaleIntensity(intensity) * 255);

    this.sendCommand(MessageType.SET_INTENSITY, payload).catch(() => {});
  }

  pulse(location: BodyLocation, duration: number, intensity: number = 0.5): void {
    this.assertConnected();

    const payload = new Uint8Array(4);
    const view = new DataView(payload.buffer);
    payload[0] = this.locationToId(location);
    view.setUint16(1, duration, true);
    payload[3] = Math.round(this.scaleIntensity(intensity) * 255);

    this.sendCommand(MessageType.PULSE, payload).catch(() => {});
  }

  vibrate(location: BodyLocation, frequency: number, intensity: number): void {
    this.assertConnected();

    const payload = new Uint8Array(4);
    const view = new DataView(payload.buffer);
    payload[0] = this.locationToId(location);
    view.setUint16(1, frequency, true);
    payload[3] = Math.round(this.scaleIntensity(intensity) * 255);

    this.sendCommand(MessageType.VIBRATE, payload).catch(() => {});
  }

  // -------------------------------------------------------------------------
  // Utility
  // -------------------------------------------------------------------------

  private locationToId(location: BodyLocation): number {
    const locationMap: Record<BodyLocation, number> = {
      wrist_left_dorsal: 0,
      wrist_right_dorsal: 1,
      palm_left: 2,
      palm_right: 3,
      chest_center: 4,
      back_upper_center: 5,
      // ... extend as needed
    } as Record<BodyLocation, number>;

    return locationMap[location] ?? 0;
  }

  private estimateDuration(pattern: HapticPattern): number {
    if ('duration' in pattern) {
      return (pattern as HapticPrimitive).duration;
    }
    if ('steps' in pattern) {
      const seq = pattern as HapticSequence;
      let duration = 0;
      for (const step of seq.steps) {
        duration += step.delayBefore ?? 0;
        if (step.primitive) {
          duration += step.primitive.duration * (step.repeatCount ?? 1);
          duration += (step.repeatDelay ?? 0) * ((step.repeatCount ?? 1) - 1);
        }
      }
      return duration;
    }
    return 100;
  }
}
