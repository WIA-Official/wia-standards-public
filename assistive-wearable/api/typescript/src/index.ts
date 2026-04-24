/**
 * WIA Assistive Wearable Standard - SDK Implementation
 *
 * @packageDocumentation
 * @module wia-assistive-wearable
 */

import { EventEmitter } from 'events';
import * as types from './types';

// Re-export all types
export * from './types';

/**
 * Main Assistive Wearable class
 * Provides unified interface for assistive wearable devices
 *
 * @example
 * ```typescript
 * const device = new WIAAssistiveWearable({
 *   device: {
 *     id: 'device-001',
 *     name: 'Smart Hearing Aid',
 *     category: DeviceCategory.HearingAid,
 *     ...
 *   },
 *   autoConnect: true,
 *   emergencyDetection: true
 * });
 *
 * await device.connect();
 *
 * device.on('alert', (alert) => {
 *   console.log('Alert:', alert.message);
 * });
 * ```
 */
export class WIAAssistiveWearable extends EventEmitter {
  private config: types.AssistiveWearableConfig;
  private isConnected: boolean = false;
  private deviceStatus?: types.DeviceStatus;
  private features: Map<string, types.AccessibilityFeature> = new Map();
  private gestures: Map<string, types.Gesture> = new Map();
  private hapticPatterns: Map<string, types.HapticPattern> = new Map();
  private sensorDataBuffer: types.SensorData[] = [];
  private healthMetrics: types.HealthMetric[] = [];
  private alerts: Map<string, types.Alert> = new Map();

  /**
   * Create a new Assistive Wearable instance
   * @param config - Configuration options
   */
  constructor(config: types.AssistiveWearableConfig) {
    super();
    this.config = config;
    this.initializeDefaults();
  }

  /**
   * Initialize default features and patterns
   */
  private initializeDefaults(): void {
    // Default haptic patterns
    const defaultPatterns: types.HapticPattern[] = [
      {
        id: 'alert',
        name: 'Alert',
        meaning: 'General alert notification',
        sequence: [{ duration: 200, intensity: 100, pauseAfter: 100 }, { duration: 200, intensity: 100 }]
      },
      {
        id: 'emergency',
        name: 'Emergency',
        meaning: 'Emergency alert',
        sequence: [{ duration: 500, intensity: 100, pauseAfter: 200 }, { duration: 500, intensity: 100, pauseAfter: 200 }, { duration: 500, intensity: 100 }]
      },
      {
        id: 'navigation_left',
        name: 'Turn Left',
        meaning: 'Navigation instruction to turn left',
        sequence: [{ duration: 100, intensity: 80 }]
      },
      {
        id: 'navigation_right',
        name: 'Turn Right',
        meaning: 'Navigation instruction to turn right',
        sequence: [{ duration: 100, intensity: 80, pauseAfter: 50 }, { duration: 100, intensity: 80 }]
      }
    ];

    for (const pattern of defaultPatterns) {
      this.hapticPatterns.set(pattern.id, pattern);
    }
  }

  /**
   * Connect to the device
   */
  async connect(): Promise<boolean> {
    console.log(`Connecting to ${this.config.device.name}...`);

    // Simulate connection
    await new Promise(resolve => setTimeout(resolve, 1000));

    this.isConnected = true;
    this.deviceStatus = {
      deviceId: this.config.device.id,
      connected: true,
      battery: {
        level: 85,
        charging: false,
        health: 95
      },
      mode: types.AssistanceMode.Automatic,
      activeFeatures: [],
      lastSync: new Date()
    };

    this.emit('connected', this.deviceStatus);
    return true;
  }

  /**
   * Disconnect from the device
   */
  async disconnect(): Promise<void> {
    console.log('Disconnecting...');
    this.isConnected = false;
    if (this.deviceStatus) {
      this.deviceStatus.connected = false;
    }
    this.emit('disconnected');
  }

  /**
   * Get device status
   */
  getDeviceStatus(): types.DeviceStatus | undefined {
    return this.deviceStatus;
  }

  /**
   * Set user profile
   * @param profile - User profile
   */
  setUserProfile(profile: types.UserProfile): void {
    this.config.userProfile = profile;
    this.applyPreferences(profile.preferences);
    this.emit('profile-updated', profile);
  }

  /**
   * Apply user preferences
   */
  private applyPreferences(preferences: types.UserPreferences): void {
    // Apply preferences to device
    console.log('Applying user preferences:', preferences);
    this.emit('preferences-applied', preferences);
  }

  /**
   * Register accessibility feature
   * @param feature - Feature definition
   */
  registerFeature(feature: types.AccessibilityFeature): void {
    this.features.set(feature.id, feature);
    this.emit('feature-registered', feature);
  }

  /**
   * Enable feature
   * @param featureId - Feature ID
   */
  enableFeature(featureId: string): boolean {
    const feature = this.features.get(featureId);
    if (feature) {
      feature.enabled = true;
      if (this.deviceStatus) {
        this.deviceStatus.activeFeatures.push(featureId);
      }
      this.emit('feature-enabled', feature);
      return true;
    }
    return false;
  }

  /**
   * Disable feature
   * @param featureId - Feature ID
   */
  disableFeature(featureId: string): boolean {
    const feature = this.features.get(featureId);
    if (feature) {
      feature.enabled = false;
      if (this.deviceStatus) {
        this.deviceStatus.activeFeatures = this.deviceStatus.activeFeatures.filter(f => f !== featureId);
      }
      this.emit('feature-disabled', feature);
      return true;
    }
    return false;
  }

  /**
   * Get enabled features
   */
  getEnabledFeatures(): types.AccessibilityFeature[] {
    return Array.from(this.features.values()).filter(f => f.enabled);
  }

  /**
   * Configure hearing aid
   * @param settings - Hearing aid settings
   */
  async configureHearingAid(settings: types.HearingAidSettings): Promise<void> {
    if (this.config.device.category !== types.DeviceCategory.HearingAid &&
        this.config.device.category !== types.DeviceCategory.CochlearImplant) {
      throw new Error('Device is not a hearing aid');
    }

    // Apply settings
    console.log('Configuring hearing aid:', settings);
    this.emit('hearing-aid-configured', settings);
  }

  /**
   * Configure vision aid
   * @param settings - Vision aid settings
   */
  async configureVisionAid(settings: types.VisionAidSettings): Promise<void> {
    if (this.config.device.category !== types.DeviceCategory.VisionAid &&
        this.config.device.category !== types.DeviceCategory.SmartGlasses) {
      throw new Error('Device is not a vision aid');
    }

    console.log('Configuring vision aid:', settings);
    this.emit('vision-aid-configured', settings);
  }

  /**
   * Configure exoskeleton
   * @param settings - Exoskeleton settings
   */
  async configureExoskeleton(settings: types.ExoskeletonSettings): Promise<void> {
    if (this.config.device.category !== types.DeviceCategory.Exoskeleton) {
      throw new Error('Device is not an exoskeleton');
    }

    console.log('Configuring exoskeleton:', settings);
    this.emit('exoskeleton-configured', settings);
  }

  /**
   * Submit sensor data
   * @param data - Sensor data
   */
  async submitSensorData(data: types.SensorData): Promise<void> {
    this.sensorDataBuffer.push(data);

    // Check for emergency conditions
    if (this.config.emergencyDetection) {
      this.checkEmergencyConditions(data);
    }

    this.emit('sensor-data', data);
  }

  /**
   * Check for emergency conditions
   */
  private checkEmergencyConditions(data: types.SensorData): void {
    // Fall detection using accelerometer
    if (data.type === types.SensorType.Accelerometer) {
      const magnitude = Math.sqrt(
        (data.value as number[])[0] ** 2 +
        (data.value as number[])[1] ** 2 +
        (data.value as number[])[2] ** 2
      );

      if (magnitude > 25) { // High acceleration threshold
        this.triggerAlert({
          id: `fall-${Date.now()}`,
          type: 'fall',
          severity: 'critical',
          message: 'Potential fall detected',
          data: { acceleration: magnitude },
          timestamp: new Date(),
          acknowledged: false
        });
      }
    }

    // Heart rate monitoring
    if (data.type === types.SensorType.HeartRate) {
      const hr = data.value as number;
      if (hr < 40 || hr > 180) {
        this.triggerAlert({
          id: `hr-${Date.now()}`,
          type: 'health',
          severity: hr < 40 ? 'critical' : 'warning',
          message: `Abnormal heart rate: ${hr} bpm`,
          data: { heartRate: hr },
          timestamp: new Date(),
          acknowledged: false
        });
      }
    }
  }

  /**
   * Trigger an alert
   * @param alert - Alert to trigger
   */
  triggerAlert(alert: types.Alert): void {
    this.alerts.set(alert.id, alert);
    this.emit('alert', alert);

    // Send haptic feedback
    if (alert.severity === 'emergency' || alert.severity === 'critical') {
      this.playHapticPattern('emergency');
    } else {
      this.playHapticPattern('alert');
    }

    // Notify emergency contacts for critical alerts
    if (alert.severity === 'emergency' && this.config.userProfile) {
      this.notifyEmergencyContacts(alert);
    }
  }

  /**
   * Notify emergency contacts
   */
  private notifyEmergencyContacts(alert: types.Alert): void {
    if (!this.config.userProfile?.emergencyContacts) return;

    for (const contact of this.config.userProfile.emergencyContacts) {
      console.log(`Notifying ${contact.name} (${contact.phone}): ${alert.message}`);
      this.emit('emergency-contact-notified', { contact, alert });
    }
  }

  /**
   * Acknowledge alert
   * @param alertId - Alert ID
   */
  acknowledgeAlert(alertId: string): boolean {
    const alert = this.alerts.get(alertId);
    if (alert) {
      alert.acknowledged = true;
      this.emit('alert-acknowledged', alert);
      return true;
    }
    return false;
  }

  /**
   * Register gesture
   * @param gesture - Gesture definition
   */
  registerGesture(gesture: types.Gesture): void {
    this.gestures.set(gesture.id, gesture);
    this.emit('gesture-registered', gesture);
  }

  /**
   * Process gesture
   * @param gestureId - Gesture ID
   */
  processGesture(gestureId: string): void {
    const gesture = this.gestures.get(gestureId);
    if (gesture) {
      this.emit('gesture-detected', gesture);
      this.executeAction(gesture.action, gesture.parameters);
    }
  }

  /**
   * Execute action
   */
  private executeAction(action: string, parameters?: Record<string, unknown>): void {
    console.log(`Executing action: ${action}`, parameters);
    this.emit('action-executed', { action, parameters });
  }

  /**
   * Play haptic pattern
   * @param patternId - Pattern ID
   */
  async playHapticPattern(patternId: string): Promise<void> {
    const pattern = this.hapticPatterns.get(patternId);
    if (pattern) {
      console.log(`Playing haptic pattern: ${pattern.name}`);
      this.emit('haptic-feedback', pattern);
    }
  }

  /**
   * Speak text
   * @param request - Speech request
   */
  async speak(request: types.SpeechRequest): Promise<void> {
    console.log(`Speaking: ${request.text}`);
    this.emit('speech-started', request);

    // Simulate speech duration
    const duration = request.text.length * 50 / (request.speed || 1);
    await new Promise(resolve => setTimeout(resolve, duration));

    this.emit('speech-completed', request);
  }

  /**
   * Start speech recognition
   */
  async startSpeechRecognition(): Promise<void> {
    this.emit('speech-recognition-started');
  }

  /**
   * Stop speech recognition
   */
  async stopSpeechRecognition(): Promise<types.SpeechRecognitionResult> {
    const result: types.SpeechRecognitionResult = {
      transcript: 'Simulated speech recognition result',
      confidence: 0.95,
      isFinal: true
    };
    this.emit('speech-recognition-result', result);
    return result;
  }

  /**
   * Provide navigation instruction
   * @param instruction - Navigation instruction
   */
  async provideNavigationInstruction(instruction: types.NavigationInstruction): Promise<void> {
    // Speak the instruction
    await this.speak({
      text: instruction.text,
      language: this.config.userProfile?.preferences.language || 'en',
      priority: 'high'
    });

    // Provide haptic feedback for direction
    if (instruction.direction === 'left') {
      await this.playHapticPattern('navigation_left');
    } else if (instruction.direction === 'right') {
      await this.playHapticPattern('navigation_right');
    }

    this.emit('navigation-instruction', instruction);
  }

  /**
   * Record health metric
   * @param metric - Health metric
   */
  recordHealthMetric(metric: types.HealthMetric): void {
    this.healthMetrics.push(metric);
    this.emit('health-metric', metric);

    if (metric.status !== 'normal') {
      this.triggerAlert({
        id: `health-${Date.now()}`,
        type: 'health',
        severity: metric.status === 'critical' ? 'critical' : 'warning',
        message: `${metric.type}: ${metric.value} ${metric.unit}`,
        data: { metric },
        timestamp: new Date(),
        acknowledged: false
      });
    }
  }

  /**
   * Get health metrics
   * @param type - Metric type filter
   */
  getHealthMetrics(type?: string): types.HealthMetric[] {
    if (type) {
      return this.healthMetrics.filter(m => m.type === type);
    }
    return [...this.healthMetrics];
  }

  /**
   * Check WIA compliance
   * @param targetLevel - Target certification level
   */
  checkCompliance(targetLevel: types.CertificationLevel): types.ComplianceReport {
    const tests: types.TestResult[] = [];

    // Test 1: Device configuration
    tests.push({
      testName: 'Device Configuration',
      passed: this.config.device.id !== undefined && this.config.device.category !== undefined,
      notes: 'Device ID and category must be defined'
    });

    // Test 2: Accessibility features
    tests.push({
      testName: 'Accessibility Features',
      passed: this.features.size > 0,
      notes: 'At least one accessibility feature required'
    });

    // Test 3: Emergency detection for Silver+
    if (targetLevel !== types.CertificationLevel.Bronze) {
      tests.push({
        testName: 'Emergency Detection',
        passed: this.config.emergencyDetection === true,
        notes: 'Emergency detection required for Silver/Gold'
      });
    }

    // Test 4: User profile and emergency contacts for Gold
    if (targetLevel === types.CertificationLevel.Gold) {
      tests.push({
        testName: 'User Profile & Emergency Contacts',
        passed: this.config.userProfile !== undefined &&
                (this.config.userProfile?.emergencyContacts?.length || 0) > 0,
        notes: 'User profile with emergency contacts required for Gold'
      });
    }

    const passed = tests.every(t => t.passed);

    return {
      standard: 'WIA-ASSISTIVE-WEARABLE',
      testDate: new Date().toISOString(),
      config: this.config,
      targetLevel,
      tests,
      passed,
      achievedLevel: passed ? targetLevel : undefined
    };
  }
}

/**
 * Default export for convenience
 */
export default {
  WIAAssistiveWearable
};
