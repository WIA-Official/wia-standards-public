/**
 * WIA AAC API Tests
 */

import { WiaAac, MockAdapter, SensorType, WiaAacSignal, ConnectionState } from '../src';

describe('WiaAac', () => {
  let aac: WiaAac;

  beforeEach(() => {
    aac = new WiaAac({ logLevel: 'none' });
  });

  afterEach(async () => {
    if (aac.isConnected()) {
      await aac.disconnect();
    }
  });

  describe('Connection', () => {
    it('should start disconnected', () => {
      expect(aac.isConnected()).toBe(false);
      expect(aac.getConnectionState()).toBe(ConnectionState.DISCONNECTED);
    });

    it('should connect with mock adapter', async () => {
      const mockAdapter = new MockAdapter({ type: 'eye_tracker' });
      aac.useAdapter(mockAdapter);

      await aac.connect({ type: 'eye_tracker' });

      expect(aac.isConnected()).toBe(true);
      expect(aac.getConnectionState()).toBe(ConnectionState.CONNECTED);
    });

    it('should disconnect properly', async () => {
      const mockAdapter = new MockAdapter({ type: 'eye_tracker' });
      aac.useAdapter(mockAdapter);

      await aac.connect({ type: 'eye_tracker' });
      await aac.disconnect();

      expect(aac.isConnected()).toBe(false);
      expect(aac.getConnectionState()).toBe(ConnectionState.DISCONNECTED);
    });

    it('should emit connected event', async () => {
      const mockAdapter = new MockAdapter({ type: 'switch' });
      aac.useAdapter(mockAdapter);

      const connectedPromise = new Promise<void>((resolve) => {
        aac.once('connected', () => resolve());
      });

      await aac.connect({ type: 'switch' });
      await connectedPromise;

      expect(aac.isConnected()).toBe(true);
    });

    it('should emit disconnected event', async () => {
      const mockAdapter = new MockAdapter({ type: 'switch' });
      aac.useAdapter(mockAdapter);

      await aac.connect({ type: 'switch' });

      const disconnectedPromise = new Promise<void>((resolve) => {
        aac.once('disconnected', () => resolve());
      });

      await aac.disconnect();
      await disconnectedPromise;

      expect(aac.isConnected()).toBe(false);
    });
  });

  describe('Events', () => {
    it('should handle signal events', async () => {
      const mockAdapter = new MockAdapter({ type: 'eye_tracker' });
      aac.useAdapter(mockAdapter);

      await aac.connect({ type: 'eye_tracker' });

      const signalPromise = new Promise<WiaAacSignal>((resolve) => {
        aac.on('signal', (signal) => resolve(signal));
      });

      // Emit a mock signal
      mockAdapter.emitMockSignal();

      const signal = await signalPromise;
      expect(signal.type).toBe('eye_tracker');
      expect(signal.data).toBeDefined();
    });

    it('should support once listener', async () => {
      const mockAdapter = new MockAdapter({ type: 'switch' });
      aac.useAdapter(mockAdapter);

      await aac.connect({ type: 'switch' });

      let callCount = 0;
      aac.once('signal', () => {
        callCount++;
      });

      mockAdapter.emitMockSignal();
      mockAdapter.emitMockSignal();

      // Small delay to ensure events are processed
      await new Promise((r) => setTimeout(r, 10));

      expect(callCount).toBe(1);
    });

    it('should support off to unsubscribe', async () => {
      const mockAdapter = new MockAdapter({ type: 'switch' });
      aac.useAdapter(mockAdapter);

      await aac.connect({ type: 'switch' });

      let callCount = 0;
      const handler = () => {
        callCount++;
      };

      aac.on('signal', handler);
      mockAdapter.emitMockSignal();

      aac.off('signal', handler);
      mockAdapter.emitMockSignal();

      await new Promise((r) => setTimeout(r, 10));

      expect(callCount).toBe(1);
    });
  });

  describe('Signal Buffer', () => {
    it('should store signals in buffer', async () => {
      const mockAdapter = new MockAdapter({ type: 'eye_tracker' });
      aac.useAdapter(mockAdapter);

      await aac.connect({ type: 'eye_tracker' });

      expect(aac.getLastSignal()).toBeNull();

      mockAdapter.emitMockSignal();

      expect(aac.getLastSignal()).not.toBeNull();
      expect(aac.getSignalBuffer().length).toBe(1);
    });

    it('should limit buffer size', async () => {
      const smallBufferAac = new WiaAac({
        logLevel: 'none',
        signalBufferSize: 5
      });

      const mockAdapter = new MockAdapter({ type: 'switch' });
      smallBufferAac.useAdapter(mockAdapter);

      await smallBufferAac.connect({ type: 'switch' });

      // Emit 10 signals
      for (let i = 0; i < 10; i++) {
        mockAdapter.emitMockSignal();
      }

      expect(smallBufferAac.getSignalBuffer().length).toBe(5);

      await smallBufferAac.disconnect();
    });
  });

  describe('Configuration', () => {
    it('should get config after connect', async () => {
      const mockAdapter = new MockAdapter({ type: 'breath' });
      aac.useAdapter(mockAdapter);

      expect(aac.getConfig()).toBeNull();

      await aac.connect({
        type: 'breath',
        options: { sipThreshold: 0.5 }
      });

      const config = aac.getConfig();
      expect(config).not.toBeNull();
      expect(config?.type).toBe('breath');
    });
  });
});

describe('MockAdapter', () => {
  it('should generate correct signal types', () => {
    const types: Array<'eye_tracker' | 'switch' | 'muscle_sensor' | 'brain_interface' | 'breath' | 'head_movement'> = [
      'eye_tracker',
      'switch',
      'muscle_sensor',
      'brain_interface',
      'breath',
      'head_movement'
    ];

    for (const type of types) {
      const adapter = new MockAdapter({ type });
      let receivedSignal: WiaAacSignal | null = null;

      adapter.onSignal((signal) => {
        receivedSignal = signal;
      });

      adapter.emitMockSignal();

      expect(receivedSignal).not.toBeNull();
      expect(receivedSignal!.type).toBe(type);
      expect(receivedSignal!.version).toBe('1.0.0');
      expect(receivedSignal!.data).toBeDefined();
    }
  });

  it('should simulate continuous signals', async () => {
    const adapter = new MockAdapter({
      type: 'eye_tracker',
      simulateSignals: true,
      signalInterval: 50
    });

    const signals: WiaAacSignal[] = [];
    adapter.onSignal((signal) => signals.push(signal));

    await adapter.connect({ type: 'eye_tracker' });

    // Wait for some signals
    await new Promise((r) => setTimeout(r, 200));

    await adapter.disconnect();

    expect(signals.length).toBeGreaterThan(2);
  });
});
