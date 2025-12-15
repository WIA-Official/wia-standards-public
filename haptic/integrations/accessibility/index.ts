/**
 * WIA Haptic Standard - Accessibility Platform Integrations
 *
 * Platform-specific accessibility integrations:
 * - iOS VoiceOver (Swift)
 * - Android TalkBack (Kotlin)
 * - Web Vibration API (TypeScript)
 */

// Web integration exports
export {
  WebVibrationHaptic,
  GamepadHaptic,
  WebAccessibilityHaptic,
  WEB_HAPTIC_PATTERNS,
  checkWebHapticCapabilities,
} from './web-vibration';

export type {
  WebHapticCapabilities,
  WebAccessibilityEvent,
} from './web-vibration';

// Note: iOS and Android integrations are in their respective languages
// - ios-voiceover.swift: VoiceOverHaptic class
// - android-talkback.kt: TalkBackHapticService class
