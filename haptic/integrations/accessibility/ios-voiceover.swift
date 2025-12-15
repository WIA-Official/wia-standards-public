/**
 * WIA Haptic Standard - iOS VoiceOver Integration
 *
 * Swift implementation for integrating WIA haptic feedback with iOS VoiceOver.
 */

import UIKit
import CoreHaptics

// MARK: - Haptic Pattern Types

/// WIA haptic waveform types
public enum WIAWaveformType: String, Codable {
    case sine
    case square
    case triangle
    case sawtooth
    case noise
}

/// WIA haptic primitive
public struct WIAHapticPrimitive: Codable {
    let waveform: WIAWaveformType
    let frequency: Double   // Hz
    let intensity: Double   // 0-1
    let duration: Double    // ms
    let delay: Double?      // ms
}

/// WIA haptic pattern
public struct WIAHapticPattern: Codable {
    let id: String
    let name: String
    let description: String?
    let primitives: [WIAHapticPrimitive]
    let totalDuration: Double
}

// MARK: - VoiceOver Haptic Patterns

/// Pre-defined patterns for VoiceOver events
public struct VoiceOverHapticPatterns {

    /// Focus moved to new element
    public static let focusChange = WIAHapticPattern(
        id: "vo.focus.change",
        name: "Focus Change",
        description: "VoiceOver focus moved to new element",
        primitives: [
            WIAHapticPrimitive(waveform: .sine, frequency: 150, intensity: 0.4, duration: 30, delay: nil)
        ],
        totalDuration: 30
    )

    /// Focus on button
    public static let focusButton = WIAHapticPattern(
        id: "vo.focus.button",
        name: "Button Focus",
        description: "Focus on interactive button",
        primitives: [
            WIAHapticPrimitive(waveform: .square, frequency: 180, intensity: 0.5, duration: 40, delay: nil)
        ],
        totalDuration: 40
    )

    /// Focus on text field
    public static let focusTextField = WIAHapticPattern(
        id: "vo.focus.textfield",
        name: "Text Field Focus",
        description: "Focus on editable text field",
        primitives: [
            WIAHapticPrimitive(waveform: .sine, frequency: 120, intensity: 0.4, duration: 50, delay: nil),
            WIAHapticPrimitive(waveform: .sine, frequency: 140, intensity: 0.5, duration: 50, delay: 30)
        ],
        totalDuration: 130
    )

    /// Focus on link
    public static let focusLink = WIAHapticPattern(
        id: "vo.focus.link",
        name: "Link Focus",
        description: "Focus on hyperlink",
        primitives: [
            WIAHapticPrimitive(waveform: .sine, frequency: 200, intensity: 0.5, duration: 40, delay: nil)
        ],
        totalDuration: 40
    )

    /// Focus on heading
    public static let focusHeading = WIAHapticPattern(
        id: "vo.focus.heading",
        name: "Heading Focus",
        description: "Focus on heading element",
        primitives: [
            WIAHapticPrimitive(waveform: .sine, frequency: 160, intensity: 0.6, duration: 60, delay: nil)
        ],
        totalDuration: 60
    )

    /// Focus on image
    public static let focusImage = WIAHapticPattern(
        id: "vo.focus.image",
        name: "Image Focus",
        description: "Focus on image element",
        primitives: [
            WIAHapticPrimitive(waveform: .triangle, frequency: 130, intensity: 0.4, duration: 50, delay: nil)
        ],
        totalDuration: 50
    )

    /// Element activated
    public static let activate = WIAHapticPattern(
        id: "vo.action.activate",
        name: "Activate",
        description: "Element activated (double-tap)",
        primitives: [
            WIAHapticPrimitive(waveform: .square, frequency: 200, intensity: 0.7, duration: 60, delay: nil),
            WIAHapticPrimitive(waveform: .square, frequency: 200, intensity: 0.5, duration: 40, delay: 40)
        ],
        totalDuration: 140
    )

    /// Escape gesture
    public static let escape = WIAHapticPattern(
        id: "vo.action.escape",
        name: "Escape",
        description: "Two-finger scrub gesture",
        primitives: [
            WIAHapticPrimitive(waveform: .sine, frequency: 150, intensity: 0.5, duration: 80, delay: nil),
            WIAHapticPrimitive(waveform: .sine, frequency: 100, intensity: 0.3, duration: 80, delay: 50)
        ],
        totalDuration: 210
    )

    /// Magic tap
    public static let magicTap = WIAHapticPattern(
        id: "vo.action.magictap",
        name: "Magic Tap",
        description: "Two-finger double-tap",
        primitives: [
            WIAHapticPrimitive(waveform: .square, frequency: 180, intensity: 0.8, duration: 50, delay: nil),
            WIAHapticPrimitive(waveform: .square, frequency: 220, intensity: 0.9, duration: 50, delay: 30)
        ],
        totalDuration: 130
    )

    /// Rotor changed
    public static let rotorChange = WIAHapticPattern(
        id: "vo.rotor.change",
        name: "Rotor Change",
        description: "Rotor setting changed",
        primitives: [
            WIAHapticPrimitive(waveform: .sine, frequency: 140, intensity: 0.4, duration: 40, delay: nil),
            WIAHapticPrimitive(waveform: .sine, frequency: 180, intensity: 0.5, duration: 40, delay: 30)
        ],
        totalDuration: 110
    )

    /// Screen edge reached
    public static let screenEdge = WIAHapticPattern(
        id: "vo.boundary.edge",
        name: "Screen Edge",
        description: "Reached edge of screen",
        primitives: [
            WIAHapticPrimitive(waveform: .square, frequency: 100, intensity: 0.6, duration: 80, delay: nil)
        ],
        totalDuration: 80
    )

    /// Error / invalid action
    public static let error = WIAHapticPattern(
        id: "vo.error",
        name: "Error",
        description: "Invalid action or error",
        primitives: [
            WIAHapticPrimitive(waveform: .square, frequency: 80, intensity: 0.7, duration: 100, delay: nil),
            WIAHapticPrimitive(waveform: .square, frequency: 80, intensity: 0.7, duration: 100, delay: 80)
        ],
        totalDuration: 280
    )
}

// MARK: - VoiceOver Haptic Integration

/// VoiceOver haptic integration delegate
public protocol VoiceOverHapticDelegate: AnyObject {
    func voiceOverHaptic(_ haptic: VoiceOverHaptic, didPlayPattern pattern: WIAHapticPattern)
    func voiceOverHaptic(_ haptic: VoiceOverHaptic, didFailWithError error: Error)
}

/// Main VoiceOver haptic integration class
@available(iOS 13.0, *)
public class VoiceOverHaptic: NSObject {

    // MARK: - Properties

    public weak var delegate: VoiceOverHapticDelegate?

    /// Configuration
    public var isEnabled: Bool = true
    public var intensityMultiplier: Float = 1.0
    public var screenCurtainHapticsEnabled: Bool = true

    /// Core Haptics engine
    private var hapticEngine: CHHapticEngine?
    private var isEngineRunning: Bool = false

    /// Notification observers
    private var observers: [NSObjectProtocol] = []

    // MARK: - Initialization

    public override init() {
        super.init()
        setupHapticEngine()
        setupVoiceOverObservers()
    }

    deinit {
        observers.forEach { NotificationCenter.default.removeObserver($0) }
        stopEngine()
    }

    // MARK: - Setup

    private func setupHapticEngine() {
        guard CHHapticEngine.capabilitiesForHardware().supportsHaptics else {
            print("Device does not support haptics")
            return
        }

        do {
            hapticEngine = try CHHapticEngine()
            hapticEngine?.resetHandler = { [weak self] in
                self?.restartEngine()
            }
            hapticEngine?.stoppedHandler = { [weak self] reason in
                self?.isEngineRunning = false
            }
        } catch {
            print("Failed to create haptic engine: \(error)")
        }
    }

    private func setupVoiceOverObservers() {
        // VoiceOver status change
        let voStatusObserver = NotificationCenter.default.addObserver(
            forName: UIAccessibility.voiceOverStatusDidChangeNotification,
            object: nil,
            queue: .main
        ) { [weak self] _ in
            self?.handleVoiceOverStatusChange()
        }
        observers.append(voStatusObserver)

        // Screen curtain
        if #available(iOS 14.0, *) {
            // Note: Screen curtain detection would be implemented here
        }

        // Focus element change
        let focusObserver = NotificationCenter.default.addObserver(
            forName: UIAccessibility.elementFocusedNotification,
            object: nil,
            queue: .main
        ) { [weak self] notification in
            self?.handleFocusChange(notification)
        }
        observers.append(focusObserver)
    }

    // MARK: - Engine Control

    public func startEngine() {
        guard let engine = hapticEngine, !isEngineRunning else { return }

        do {
            try engine.start()
            isEngineRunning = true
        } catch {
            print("Failed to start haptic engine: \(error)")
        }
    }

    public func stopEngine() {
        hapticEngine?.stop()
        isEngineRunning = false
    }

    private func restartEngine() {
        guard isEnabled else { return }
        startEngine()
    }

    // MARK: - Event Handlers

    private func handleVoiceOverStatusChange() {
        if UIAccessibility.isVoiceOverRunning {
            startEngine()
        } else {
            stopEngine()
        }
    }

    private func handleFocusChange(_ notification: Notification) {
        guard isEnabled, UIAccessibility.isVoiceOverRunning else { return }

        let userInfo = notification.userInfo
        let element = userInfo?[UIAccessibility.focusedElementUserInfoKey]

        // Determine element type and play appropriate pattern
        if let view = element as? UIView {
            playPatternForElement(view)
        }
    }

    // MARK: - Pattern Playback

    private func playPatternForElement(_ element: UIView) {
        let pattern: WIAHapticPattern

        switch element {
        case is UIButton:
            pattern = VoiceOverHapticPatterns.focusButton
        case is UITextField, is UITextView:
            pattern = VoiceOverHapticPatterns.focusTextField
        case is UIImageView:
            pattern = VoiceOverHapticPatterns.focusImage
        default:
            if element.accessibilityTraits.contains(.link) {
                pattern = VoiceOverHapticPatterns.focusLink
            } else if element.accessibilityTraits.contains(.header) {
                pattern = VoiceOverHapticPatterns.focusHeading
            } else {
                pattern = VoiceOverHapticPatterns.focusChange
            }
        }

        playPattern(pattern)
    }

    /// Play a WIA haptic pattern
    public func playPattern(_ pattern: WIAHapticPattern) {
        guard isEnabled, isEngineRunning else { return }
        guard let engine = hapticEngine else { return }

        // Check screen curtain mode
        if !screenCurtainHapticsEnabled && UIAccessibility.isVoiceOverRunning {
            // Skip if screen curtain is active and setting disabled
            // Note: Actual screen curtain detection would go here
        }

        do {
            let events = try convertToHapticEvents(pattern)
            let hapticPattern = try CHHapticPattern(events: events, parameters: [])
            let player = try engine.makePlayer(with: hapticPattern)
            try player.start(atTime: CHHapticTimeImmediate)

            delegate?.voiceOverHaptic(self, didPlayPattern: pattern)
        } catch {
            delegate?.voiceOverHaptic(self, didFailWithError: error)
        }
    }

    /// Convert WIA pattern to Core Haptics events
    private func convertToHapticEvents(_ pattern: WIAHapticPattern) throws -> [CHHapticEvent] {
        var events: [CHHapticEvent] = []
        var currentTime: TimeInterval = 0

        for primitive in pattern.primitives {
            if let delay = primitive.delay {
                currentTime += delay / 1000.0
            }

            let intensity = Float(primitive.intensity) * intensityMultiplier
            let sharpness = mapWaveformToSharpness(primitive.waveform)

            let event = CHHapticEvent(
                eventType: .hapticContinuous,
                parameters: [
                    CHHapticEventParameter(parameterID: .hapticIntensity, value: intensity),
                    CHHapticEventParameter(parameterID: .hapticSharpness, value: sharpness)
                ],
                relativeTime: currentTime,
                duration: primitive.duration / 1000.0
            )

            events.append(event)
            currentTime += primitive.duration / 1000.0
        }

        return events
    }

    /// Map WIA waveform to Core Haptics sharpness
    private func mapWaveformToSharpness(_ waveform: WIAWaveformType) -> Float {
        switch waveform {
        case .sine:
            return 0.3
        case .square:
            return 0.9
        case .triangle:
            return 0.5
        case .sawtooth:
            return 0.7
        case .noise:
            return 1.0
        }
    }

    // MARK: - Public Actions

    /// Manually trigger haptic for activation
    public func playActivation() {
        playPattern(VoiceOverHapticPatterns.activate)
    }

    /// Manually trigger haptic for escape
    public func playEscape() {
        playPattern(VoiceOverHapticPatterns.escape)
    }

    /// Manually trigger haptic for magic tap
    public func playMagicTap() {
        playPattern(VoiceOverHapticPatterns.magicTap)
    }

    /// Manually trigger haptic for rotor change
    public func playRotorChange() {
        playPattern(VoiceOverHapticPatterns.rotorChange)
    }

    /// Manually trigger haptic for screen edge
    public func playScreenEdge() {
        playPattern(VoiceOverHapticPatterns.screenEdge)
    }

    /// Manually trigger haptic for error
    public func playError() {
        playPattern(VoiceOverHapticPatterns.error)
    }
}

// MARK: - UIAccessibilityElement Extension

extension UIAccessibilityElement {

    /// Play haptic feedback for this element's type
    @available(iOS 13.0, *)
    public func playAccessibilityHaptic(using haptic: VoiceOverHaptic) {
        if accessibilityTraits.contains(.button) {
            haptic.playPattern(VoiceOverHapticPatterns.focusButton)
        } else if accessibilityTraits.contains(.link) {
            haptic.playPattern(VoiceOverHapticPatterns.focusLink)
        } else if accessibilityTraits.contains(.header) {
            haptic.playPattern(VoiceOverHapticPatterns.focusHeading)
        } else if accessibilityTraits.contains(.image) {
            haptic.playPattern(VoiceOverHapticPatterns.focusImage)
        } else {
            haptic.playPattern(VoiceOverHapticPatterns.focusChange)
        }
    }
}
