# WIA Smart Home Phase 3: Communication Protocol

**Version**: 1.0.0
**Status**: Draft
**Last Updated**: 2025-01-01

---

## 1. Overview

This specification defines the communication protocol for WIA Smart Home devices, built on top of the Matter protocol with accessibility extensions.

### 1.1 Goals

- Matter protocol compatibility
- Accessibility-aware messaging
- Multi-modal notification delivery
- Secure device communication

### 1.2 Scope

- Device discovery and commissioning
- Message format and serialization
- Cluster definitions (standard + accessibility)
- Security requirements

---

## 2. Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   WIA Smart Home App                     │
├─────────────────────────────────────────────────────────┤
│                  WIA Protocol Layer                      │
│  ┌─────────────┬─────────────┬─────────────────────┐   │
│  │  Discovery  │   Message   │   Accessibility     │   │
│  │   (mDNS)    │   Handler   │    Extensions       │   │
│  └─────────────┴─────────────┴─────────────────────┘   │
├─────────────────────────────────────────────────────────┤
│                   Matter Protocol                        │
│  ┌─────────────┬─────────────┬─────────────────────┐   │
│  │  Clusters   │   Binding   │     Security        │   │
│  │  (Standard) │   Model     │    (PAKE/TLS)       │   │
│  └─────────────┴─────────────┴─────────────────────┘   │
├─────────────────────────────────────────────────────────┤
│              Transport (IPv6/UDP/TCP)                    │
└─────────────────────────────────────────────────────────┘
```

---

## 3. Matter Cluster Mapping

### 3.1 Standard Clusters

| Cluster ID | Name | Description |
|------------|------|-------------|
| 0x0006 | On/Off | Basic on/off control |
| 0x0008 | Level Control | Brightness/volume levels |
| 0x0300 | Color Control | RGB/HSV color |
| 0x0101 | Door Lock | Lock/unlock operations |
| 0x0201 | Thermostat | Temperature control |
| 0x0202 | Fan Control | Fan speed control |
| 0x0102 | Window Covering | Blinds/curtains |

### 3.2 WIA Accessibility Clusters (Custom)

| Cluster ID | Name | Description |
|------------|------|-------------|
| 0xWIA1 | Voice Command | Voice control interface |
| 0xWIA2 | Audio Feedback | TTS and tone feedback |
| 0xWIA3 | Visual Feedback | LED and screen feedback |
| 0xWIA4 | Haptic Feedback | Vibration patterns |
| 0xWIA5 | Switch Access | External switch input |
| 0xWIA6 | Dwell Control | Gaze/dwell selection |

---

## 4. Message Format

### 4.1 WIA Protocol Message

```rust
struct WiaMessage {
    header: MessageHeader,
    payload: MessagePayload,
    signature: Option<Vec<u8>>,
}

struct MessageHeader {
    version: u8,           // Protocol version (1)
    message_type: u16,     // Message type ID
    message_id: u32,       // Unique message ID
    source_node: u64,      // Source node ID
    dest_node: u64,        // Destination node ID
    timestamp: u64,        // Unix timestamp (ms)
    flags: u8,             // Message flags
}

struct MessagePayload {
    cluster_id: u16,       // Target cluster
    command_id: u8,        // Command within cluster
    data: Vec<u8>,         // Serialized command data
    accessibility: Option<AccessibilityContext>,
}

struct AccessibilityContext {
    user_profile_id: Uuid,
    input_modality: InputModality,
    preferred_outputs: Vec<OutputModality>,
    response_timeout_ms: u32,
    confirmation_required: bool,
}
```

### 4.2 Message Types

| Type ID | Name | Description |
|---------|------|-------------|
| 0x0001 | Command | Device command |
| 0x0002 | Response | Command response |
| 0x0003 | Event | Device event |
| 0x0004 | Subscribe | Event subscription |
| 0x0005 | Unsubscribe | Cancel subscription |
| 0x0010 | Discovery | Device discovery |
| 0x0011 | Announce | Device announcement |
| 0x0020 | AccessibilityFeedback | Feedback message |

---

## 5. Device Discovery

### 5.1 mDNS Service Type

```
_wia-smarthome._tcp.local
_matter._tcp.local
```

### 5.2 TXT Records

| Key | Description |
|-----|-------------|
| `DI` | Device ID |
| `DN` | Device Name |
| `DT` | Device Type |
| `VD` | Vendor ID |
| `PD` | Product ID |
| `AC` | Accessibility Features (bitmask) |
| `V` | Protocol Version |

### 5.3 Accessibility Features Bitmask

| Bit | Feature |
|-----|---------|
| 0 | Voice Control |
| 1 | Audio Feedback |
| 2 | Visual Feedback |
| 3 | Haptic Feedback |
| 4 | Switch Access |
| 5 | Dwell Control |
| 6 | BCI Support |
| 7 | Korean TTS |

---

## 6. Accessibility Extensions

### 6.1 Voice Command Cluster (0xWIA1)

**Attributes:**
- `SupportedCommands`: List of voice commands
- `ActiveLanguage`: Current language (default: ko-KR)
- `WakeWord`: Wake word for activation

**Commands:**
- `ExecuteVoiceCommand(text: string) -> Result`
- `GetSupportedCommands() -> List<VoiceCommand>`
- `SetLanguage(language: string)`

### 6.2 Audio Feedback Cluster (0xWIA2)

**Attributes:**
- `Volume`: Current volume (0-100)
- `TTSEnabled`: TTS enabled flag
- `TTSVoice`: Selected TTS voice
- `TTSRate`: Speech rate (0.5-2.0)

**Commands:**
- `Speak(text: string, language: string)`
- `PlayTone(tone_id: string)`
- `SetVolume(level: u8)`
- `Mute() / Unmute()`

### 6.3 Visual Feedback Cluster (0xWIA3)

**Attributes:**
- `LEDSupported`: LED indicator support
- `ScreenSupported`: Screen display support
- `HighContrastEnabled`: High contrast mode

**Commands:**
- `SetLED(color: RGB, pattern: LEDPattern)`
- `DisplayMessage(title: string, body: string)`
- `Flash(pattern: FlashPattern, duration_ms: u32)`

### 6.4 Haptic Feedback Cluster (0xWIA4)

**Attributes:**
- `HapticSupported`: Haptic support flag
- `Intensity`: Default intensity (0-100)

**Commands:**
- `Vibrate(pattern: HapticPattern, intensity: u8)`
- `SetDefaultIntensity(intensity: u8)`

---

## 7. Security

### 7.1 Authentication

- SPAKE2+ (Password Authenticated Key Exchange)
- Node Operational Credentials (NOC)
- Access Control Lists (ACL)

### 7.2 Encryption

- AES-128-CCM for message encryption
- TLS 1.3 for TCP connections
- DTLS 1.3 for UDP connections

### 7.3 Privacy Considerations

- User profile data encrypted at rest
- Voice recordings not stored by default
- Accessibility preferences locally cached

---

## 8. Error Codes

| Code | Name | Description |
|------|------|-------------|
| 0x00 | SUCCESS | Operation successful |
| 0x01 | UNSUPPORTED | Feature not supported |
| 0x02 | INVALID_PARAM | Invalid parameter |
| 0x03 | NOT_FOUND | Resource not found |
| 0x04 | TIMEOUT | Operation timed out |
| 0x05 | BUSY | Device busy |
| 0x06 | OFFLINE | Device offline |
| 0x07 | AUTH_FAILED | Authentication failed |
| 0x08 | ACCESS_DENIED | Permission denied |
| 0x10 | ACCESSIBILITY_ERROR | Accessibility feature error |

---

## 9. Example Flows

### 9.1 Voice Command Flow

```
User -> App: "거실 불 켜"
App -> WIA Protocol: VoiceCommand("거실 불 켜")
WIA Protocol -> Discovery: FindDevice("거실 조명")
WIA Protocol -> Device: OnOff.On()
Device -> WIA Protocol: Response(SUCCESS)
WIA Protocol -> App: AudioFeedback.Speak("거실 조명을 켰습니다")
App -> User: TTS Output
```

### 9.2 Switch Access Flow

```
User -> Switch Device: Press
Switch Device -> WIA Protocol: SwitchActivated(switch_id)
WIA Protocol -> App: ProcessSwitchInput(switch_id)
App -> WIA Protocol: ExecuteAction(mapped_action)
WIA Protocol -> Device: Command
Device -> WIA Protocol: Response
WIA Protocol -> App: HapticFeedback.Vibrate(SHORT_TAP)
```

---

## 10. References

- [Matter Specification 1.0](https://csa-iot.org/developer-resource/specifications-download-request/)
- [RFC 6762 - Multicast DNS](https://datatracker.ietf.org/doc/html/rfc6762)
- [RFC 6763 - DNS-Based Service Discovery](https://datatracker.ietf.org/doc/html/rfc6763)
- [WCAG 2.1 Guidelines](https://www.w3.org/WAI/WCAG21/quickref/)

---

弘益人間 - Benefit All Humanity
