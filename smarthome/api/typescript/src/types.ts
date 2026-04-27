/**
 * WIA Smart Home Type Definitions
 * 弘益人間 - Benefit All Humanity
 *
 * Comprehensive type system for smart home automation with accessibility features
 */

// ============================================================================
// Input/Output Modalities
// ============================================================================

export type InputModality =
  | 'voice'
  | 'touch'
  | 'switch'
  | 'gaze'
  | 'gesture'
  | 'bci'
  | 'sip_puff'
  | 'keyboard'
  | 'remote';

export type OutputModality =
  | 'visual_screen'
  | 'visual_led'
  | 'audio_tts'
  | 'audio_tone'
  | 'haptic'
  | 'braille';

// ============================================================================
// Disability Types
// ============================================================================

export type DisabilityType =
  | 'visual_blind'
  | 'visual_low_vision'
  | 'visual_color_blind'
  | 'hearing_deaf'
  | 'hearing_hard'
  | 'motor_limited'
  | 'motor_tremor'
  | 'motor_paralysis'
  | 'cognitive_learning'
  | 'cognitive_memory'
  | 'cognitive_attention'
  | 'speech'
  | 'multiple';

export type SeverityLevel = 'mild' | 'moderate' | 'severe' | 'profound';

export type WcagLevel = 'A' | 'AA' | 'AAA';

// ============================================================================
// Assistive Technology
// ============================================================================

export type AssistiveTechType =
  | 'screen_reader'
  | 'magnifier'
  | 'braille_display'
  | 'hearing_aid'
  | 'cochlear_implant'
  | 'switch_device'
  | 'eye_tracker'
  | 'head_tracker'
  | 'sip_puff_device'
  | 'bci_device'
  | 'voice_amplifier';

export type Platform = 'ios' | 'android' | 'windows' | 'macos' | 'linux' | 'universal';

export interface AssistiveTechnology {
  type: AssistiveTechType;
  name?: string;
  platform?: Platform;
  version?: string;
  configuration?: Record<string, unknown>;
}

// ============================================================================
// Accessibility Requirements
// ============================================================================

export interface VisualNeeds {
  screen_reader_required?: boolean;
  magnification_required?: boolean;
  high_contrast_required?: boolean;
  audio_descriptions_required?: boolean;
}

export interface AuditoryNeeds {
  visual_alerts_required?: boolean;
  captions_required?: boolean;
  sign_language_preferred?: boolean;
  hearing_aid_compatible?: boolean;
}

export interface MotorNeeds {
  voice_control_required?: boolean;
  switch_access_required?: boolean;
  dwell_selection_required?: boolean;
  large_targets_required?: boolean;
  reduced_motion_required?: boolean;
}

export interface CognitiveNeeds {
  simplified_interface_required?: boolean;
  consistent_navigation_required?: boolean;
  error_prevention_required?: boolean;
  reading_assistance_required?: boolean;
}

export interface SpecificNeeds {
  visual?: VisualNeeds;
  auditory?: AuditoryNeeds;
  motor?: MotorNeeds;
  cognitive?: CognitiveNeeds;
}

export interface RequiredModalities {
  input: InputModality[];
  output: OutputModality[];
}

export interface AccessibilityRequirements {
  primary_disabilities?: DisabilityType[];
  severity_levels?: Record<string, SeverityLevel>;
  assistive_technologies?: AssistiveTechnology[];
  required_modalities?: RequiredModalities;
  wcag_level?: WcagLevel;
  specific_needs?: SpecificNeeds;
}

// ============================================================================
// User Profile
// ============================================================================

export interface VoiceSettings {
  wake_word?: string;
  speech_rate?: number;
  pitch?: number;
  voice_id?: string;
}

export interface VisualSettings {
  high_contrast?: boolean;
  large_text?: boolean;
  text_scale?: number;
  reduce_motion?: boolean;
}

export interface TimingSettings {
  response_timeout_ms?: number;
  dwell_time_ms?: number;
  confirmation_required?: boolean;
}

export interface InteractionPreferences {
  preferred_input_modalities?: InputModality[];
  preferred_output_modalities?: OutputModality[];
  voice_settings?: VoiceSettings;
  visual_settings?: VisualSettings;
  timing_settings?: TimingSettings;
}

export interface QuietHours {
  enabled?: boolean;
  start_time?: string;
  end_time?: string;
}

export interface EmergencyContact {
  name: string;
  phone: string;
  relationship?: string;
  notify_on_emergency?: boolean;
  email?: string;
}

export interface PriorityOverrides {
  critical_always_notify?: boolean;
  emergency_contacts?: EmergencyContact[];
}

export interface NotificationPreferences {
  quiet_hours?: QuietHours;
  priority_overrides?: PriorityOverrides;
}

export interface DataCollectionSettings {
  usage_analytics?: boolean;
  voice_recording?: boolean;
  location_history?: boolean;
}

export interface DataSharingSettings {
  with_caregivers?: boolean;
  with_healthcare?: boolean;
}

export interface PrivacySettings {
  data_collection?: DataCollectionSettings;
  data_sharing?: DataSharingSettings;
}

export interface PersonalInfo {
  name?: string;
  preferred_language?: string;
  timezone?: string;
}

export interface UserProfile {
  profile_id: string;
  version: string;
  created_at?: string;
  updated_at?: string;
  personal_info?: PersonalInfo;
  accessibility_requirements: AccessibilityRequirements;
  interaction_preferences?: InteractionPreferences;
  notification_preferences?: NotificationPreferences;
  privacy_settings?: PrivacySettings;
}

// ============================================================================
// Device Types
// ============================================================================

export type DeviceType =
  | 'light'
  | 'light_dimmer'
  | 'light_color'
  | 'switch'
  | 'outlet'
  | 'thermostat'
  | 'fan'
  | 'air_purifier'
  | 'humidifier'
  | 'lock'
  | 'doorbell'
  | 'camera'
  | 'alarm'
  | 'blind'
  | 'curtain'
  | 'garage_door'
  | 'motion_sensor'
  | 'contact_sensor'
  | 'temperature_sensor'
  | 'humidity_sensor'
  | 'leak_sensor'
  | 'smoke_detector'
  | 'co_detector'
  | 'speaker'
  | 'tv'
  | 'media_player'
  | 'vacuum'
  | 'washer'
  | 'dryer'
  | 'refrigerator'
  | 'oven'
  | 'hub'
  | 'bridge'
  | 'other';

export type DeviceStatus = 'online' | 'offline' | 'error' | 'updating';

export interface DeviceCapabilities {
  on_off?: boolean;
  dimming?: boolean;
  color_control?: boolean;
  color_temperature?: boolean;
  temperature_sensing?: boolean;
  humidity_sensing?: boolean;
  motion_sensing?: boolean;
  contact_sensing?: boolean;
  lock_control?: boolean;
  thermostat?: boolean;
  fan_control?: boolean;
  window_covering?: boolean;
  media_playback?: boolean;
  camera?: boolean;
  doorbell?: boolean;
}

export interface VoiceCommand {
  command: string;
  aliases?: string[];
  action: string;
  parameters?: Record<string, unknown>;
  confirmation_phrase?: string;
}

export interface AudioFeedback {
  enabled?: boolean;
  volume?: number;
  tts_voice?: string;
  tones?: Record<string, string>;
}

export interface VisualFeedback {
  led_indicators?: boolean;
  high_contrast?: boolean;
  large_icons?: boolean;
}

export interface HapticFeedback {
  enabled?: boolean;
  pattern?: string;
  intensity?: number;
}

export interface DeviceTimingSettings {
  response_timeout_ms?: number;
  confirmation_required?: boolean;
  dwell_time_ms?: number;
}

export interface DeviceAccessibilityFeatures {
  supported_inputs?: InputModality[];
  supported_outputs?: OutputModality[];
  voice_commands?: VoiceCommand[];
  audio_feedback?: AudioFeedback;
  visual_feedback?: VisualFeedback;
  haptic_feedback?: HapticFeedback;
  timing?: DeviceTimingSettings;
}

export interface DeviceMetadata {
  manufacturer?: string;
  model?: string;
  serial_number?: string;
  installation_date?: string;
  last_maintenance?: string;
}

export interface Device {
  device_id: string;
  matter_node_id?: string;
  vendor_id?: string;
  product_id?: string;
  device_type: DeviceType;
  name: string;
  zone_id?: string;
  home_id?: string;
  status?: DeviceStatus;
  firmware_version?: string;
  capabilities?: DeviceCapabilities;
  current_state?: Record<string, unknown>;
  accessibility_features?: DeviceAccessibilityFeatures;
  metadata?: DeviceMetadata;
}

// ============================================================================
// Protocol Types (Matter/Thread/Zigbee/Z-Wave)
// ============================================================================

export type ProtocolType = 'matter' | 'thread' | 'zigbee' | 'zwave' | 'wifi' | 'bluetooth' | 'hybrid';

export interface MatterConfig {
  fabric_id?: string;
  node_id?: string;
  vendor_id: string;
  product_id: string;
  device_type_id?: string;
  commissioning_data?: string;
}

export interface ThreadConfig {
  network_key?: string;
  pan_id?: string;
  extended_pan_id?: string;
  channel?: number;
}

export interface ZigbeeConfig {
  network_key?: string;
  pan_id?: string;
  extended_pan_id?: string;
  channel?: number;
  ieee_address?: string;
}

export interface ZWaveConfig {
  home_id?: string;
  node_id?: number;
  security_level?: 'none' | 's0' | 's2_unauthenticated' | 's2_authenticated' | 's2_access_control';
}

export interface ProtocolInfo {
  protocol: ProtocolType;
  matter?: MatterConfig;
  thread?: ThreadConfig;
  zigbee?: ZigbeeConfig;
  zwave?: ZWaveConfig;
  wifi?: {
    ssid?: string;
    ip_address?: string;
    mac_address?: string;
  };
}

// ============================================================================
// Home and Zone
// ============================================================================

export type HubType = 'matter' | 'zigbee' | 'zwave' | 'wifi' | 'hybrid';

export interface HubInfo {
  hub_type: HubType;
  hub_id?: string;
  firmware_version?: string;
  ip_address?: string;
}

export interface Address {
  street?: string;
  city?: string;
  state?: string;
  postal_code?: string;
  country?: string;
  latitude?: number;
  longitude?: number;
}

export interface DefaultModalities {
  input?: string[];
  output?: string[];
}

export interface TtsSettings {
  default_voice?: string;
  default_language?: string;
  speech_rate?: number;
}

export type CaregiverPermission = 'view_status' | 'control_devices' | 'modify_settings' | 'receive_alerts';

export interface CaregiverAccess {
  enabled?: boolean;
  caregiver_profiles?: string[];
  permissions?: CaregiverPermission[];
}

export interface HomeAccessibilitySettings {
  default_modalities?: DefaultModalities;
  global_volume?: number;
  global_brightness?: number;
  tts_settings?: TtsSettings;
  emergency_contacts?: EmergencyContact[];
  caregiver_access?: CaregiverAccess;
}

export interface Home {
  home_id: string;
  name: string;
  address?: Address;
  timezone?: string;
  owner_profile_id: string;
  member_profiles?: string[];
  zones?: string[];
  devices?: string[];
  hub_info?: HubInfo;
  accessibility_settings?: HomeAccessibilitySettings;
  created_at?: string;
  updated_at?: string;
}

export type ZoneType =
  | 'living_room'
  | 'bedroom'
  | 'kitchen'
  | 'bathroom'
  | 'office'
  | 'garage'
  | 'garden'
  | 'entrance'
  | 'hallway'
  | 'basement'
  | 'attic'
  | 'other';

export interface Zone {
  zone_id: string;
  home_id: string;
  name: string;
  zone_type: ZoneType;
  devices?: string[];
  accessibility_overrides?: Record<string, unknown>;
  created_at?: string;
  updated_at?: string;
}

// ============================================================================
// Automation and Scenes
// ============================================================================

export type TriggerType =
  | 'time'
  | 'sunrise'
  | 'sunset'
  | 'device_state'
  | 'sensor_value'
  | 'location'
  | 'voice_command'
  | 'manual'
  | 'schedule';

export interface Trigger {
  type: TriggerType;
  config?: Record<string, unknown>;
}

export type ConditionType =
  | 'time_range'
  | 'day_of_week'
  | 'device_state'
  | 'sensor_value'
  | 'zone_occupancy'
  | 'user_presence'
  | 'sun_position'
  | 'weather';

export type ConditionOperator = 'and' | 'or' | 'not';

export interface Condition {
  type: ConditionType;
  config?: Record<string, unknown>;
  operator?: ConditionOperator;
}

export type ActionType =
  | 'device_control'
  | 'scene_activate'
  | 'notification'
  | 'delay'
  | 'condition_check'
  | 'webhook'
  | 'voice_announcement';

export interface Action {
  type: ActionType;
  device_id?: string;
  command?: string;
  parameters?: Record<string, unknown>;
  delay_ms?: number;
}

export type OverridePriority = 'low' | 'medium' | 'high' | 'critical';

export type SafeModeBehavior = 'execute' | 'skip' | 'confirm';

export interface AutomationAccessibilitySettings {
  announce_activation?: boolean;
  announcement_text?: string;
  confirmation_required?: boolean;
  override_priority?: OverridePriority;
  safe_mode_behavior?: SafeModeBehavior;
}

export type DayOfWeek = 'mon' | 'tue' | 'wed' | 'thu' | 'fri' | 'sat' | 'sun';

export interface AutomationSchedule {
  active_days?: DayOfWeek[];
  active_start_time?: string;
  active_end_time?: string;
  respect_quiet_hours?: boolean;
}

export interface Automation {
  automation_id: string;
  name: string;
  description?: string;
  enabled?: boolean;
  home_id: string;
  zone_ids?: string[];
  trigger: Trigger;
  conditions?: Condition[];
  actions: Action[];
  accessibility_settings?: AutomationAccessibilitySettings;
  schedule?: AutomationSchedule;
  created_at?: string;
  updated_at?: string;
  last_triggered_at?: string;
  trigger_count?: number;
}

export interface Scene {
  scene_id: string;
  name: string;
  description?: string;
  home_id: string;
  icon?: string;
  actions: Action[];
  accessibility_settings?: AutomationAccessibilitySettings;
  created_at?: string;
  updated_at?: string;
}

// ============================================================================
// Security System
// ============================================================================

export type SecurityMode = 'disarmed' | 'armed_home' | 'armed_away' | 'armed_night' | 'triggered';

export interface SecurityZone {
  zone_id: string;
  name: string;
  sensors: string[];
  entry_delay_ms?: number;
  exit_delay_ms?: number;
}

export interface SecuritySystem {
  system_id: string;
  home_id: string;
  mode: SecurityMode;
  zones: SecurityZone[];
  alarm_triggered?: boolean;
  triggered_at?: string;
  triggered_zone?: string;
  access_code_required?: boolean;
}

// ============================================================================
// Energy Monitoring
// ============================================================================

export interface EnergyReading {
  timestamp: string;
  power_watts: number;
  voltage?: number;
  current?: number;
  energy_kwh?: number;
}

export interface EnergyMonitoring {
  device_id: string;
  current_reading?: EnergyReading;
  daily_usage_kwh?: number;
  monthly_usage_kwh?: number;
  cost_per_kwh?: number;
  estimated_daily_cost?: number;
  peak_usage_time?: string;
}

// ============================================================================
// Voice Assistant Integration
// ============================================================================

export type VoiceAssistantType = 'alexa' | 'google' | 'siri' | 'wia';

export interface VoiceAssistantConfig {
  type: VoiceAssistantType;
  enabled: boolean;
  account_linked?: boolean;
  skills_enabled?: string[];
  wake_word?: string;
  voice_id?: string;
  language?: string;
}

export interface VoiceIntentResult {
  intent: string;
  confidence: number;
  slots?: Record<string, string>;
  device_id?: string;
  action?: string;
  parameters?: Record<string, unknown>;
}

// ============================================================================
// Notification
// ============================================================================

export type NotificationType = 'alert' | 'warning' | 'info' | 'reminder' | 'emergency';

export type NotificationPriority = 'critical' | 'high' | 'medium' | 'low';

export interface NotificationSource {
  device_id?: string;
  device_type?: string;
  device_name?: string;
  zone_id?: string;
  zone_name?: string;
  automation_id?: string;
}

export type FlashPattern = 'none' | 'pulse' | 'blink' | 'strobe';

export interface AudioDelivery {
  tts_text?: string;
  tts_language?: string;
  tts_voice?: string;
  tts_rate?: number;
  tone?: string;
  volume?: number;
  repeat_count?: number;
  repeat_interval_ms?: number;
}

export interface VisualDelivery {
  title?: string;
  body?: string;
  icon?: string;
  image_url?: string;
  color?: string;
  flash_pattern?: FlashPattern;
  duration_ms?: number;
}

export type HapticPattern = 'short_tap' | 'double_tap' | 'long_buzz' | 'pulse' | 'escalating';

export interface HapticDelivery {
  pattern: HapticPattern;
  intensity?: number;
  repeat?: number;
}

export interface NotificationDelivery {
  modalities: OutputModality[];
  audio?: AudioDelivery;
  visual?: VisualDelivery;
  haptic?: HapticDelivery;
}

export interface NotificationAction {
  id: string;
  label: string;
  label_i18n?: Record<string, string>;
  command: string;
  parameters?: Record<string, unknown>;
  confirmation_required?: boolean;
  icon?: string;
}

export interface NotificationTargeting {
  user_ids?: string[];
  zone_ids?: string[];
  all_users?: boolean;
}

export interface NotificationScheduling {
  deliver_at?: string;
  expires_at?: string;
  respect_quiet_hours?: boolean;
  snooze_options?: number[];
}

export interface Notification {
  notification_id: string;
  type: NotificationType;
  priority: NotificationPriority;
  source?: NotificationSource;
  message: Record<string, string>;
  delivery: NotificationDelivery;
  actions?: NotificationAction[];
  targeting?: NotificationTargeting;
  scheduling?: NotificationScheduling;
  timestamp?: string;
  read_at?: string;
  dismissed_at?: string;
}

// ============================================================================
// Accessibility Event
// ============================================================================

export type EventType =
  | 'voice_command_received'
  | 'voice_command_executed'
  | 'voice_command_failed'
  | 'switch_activated'
  | 'gesture_detected'
  | 'eye_gaze_selection'
  | 'bci_command'
  | 'device_state_changed'
  | 'automation_triggered'
  | 'automation_executed'
  | 'automation_failed'
  | 'scene_activated'
  | 'emergency_triggered'
  | 'emergency_resolved'
  | 'fall_detected'
  | 'inactivity_alert'
  | 'medication_reminder'
  | 'caregiver_alert_sent'
  | 'caregiver_responded'
  | 'safe_mode_activated'
  | 'safe_mode_deactivated'
  | 'quiet_hours_started'
  | 'quiet_hours_ended'
  | 'user_entered_zone'
  | 'user_left_zone'
  | 'accessibility_settings_changed'
  | 'device_accessibility_error'
  | 'system_announcement';

export type EventSourceType = 'device' | 'user' | 'automation' | 'system' | 'caregiver';

export interface EventSource {
  type: EventSourceType;
  id: string;
  name?: string;
}

export type EventTargetType = 'device' | 'zone' | 'home' | 'user';

export interface EventTarget {
  type: EventTargetType;
  id: string;
}

export type EventSeverity = 'info' | 'warning' | 'alert' | 'emergency';

export interface AccessibilityContext {
  modalities_used?: InputModality[];
  assistive_tech?: string;
  response_time_ms?: number;
  accessibility_features_active?: string[];
}

export interface NotificationSent {
  notification_id: string;
  recipient_id: string;
  channel: OutputModality;
  delivered: boolean;
}

export interface AccessibilityEvent {
  event_id: string;
  event_type: EventType;
  timestamp: string;
  source: EventSource;
  target?: EventTarget;
  home_id?: string;
  zone_id?: string;
  user_id?: string;
  severity?: EventSeverity;
  data?: Record<string, unknown>;
  accessibility_context?: AccessibilityContext;
  notifications_sent?: NotificationSent[];
  requires_acknowledgment?: boolean;
  acknowledged_at?: string;
  acknowledged_by?: string;
}

// ============================================================================
// API Configuration and Options
// ============================================================================

export interface WIASmartHomeConfig {
  api_endpoint?: string;
  api_key?: string;
  home_id?: string;
  user_profile_id?: string;
  enable_local_discovery?: boolean;
  enable_cloud_sync?: boolean;
  default_protocol?: ProtocolType;
  timeout_ms?: number;
}

export interface DeviceControlOptions {
  confirm?: boolean;
  announce?: boolean;
  timeout_ms?: number;
}

export interface DiscoveryOptions {
  protocol?: ProtocolType;
  timeout_ms?: number;
  filter_device_types?: DeviceType[];
}
