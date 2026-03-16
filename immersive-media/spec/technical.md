# WIA-EDU-021: Technical Specification

**Version:** 1.0.0
**Last Updated:** 2025-01-01

## 360° Video Specifications

### Supported Formats

#### Resolution Requirements
- **Minimum:** 4K (3840×2160) monoscopic
- **Recommended:** 5.7K (5760×2880) or 8K (7680×4320) monoscopic
- **Stereoscopic:** Minimum 5.7K per eye, recommended 8K per eye

#### Projection Types
- **Equirectangular:** 2:1 aspect ratio, primary format
- **Cubemap:** 6 square faces, alternative for real-time rendering
- **Fisheye:** Circular projection, specialized use cases

#### Video Codecs
- **H.264 (AVC):** Universal baseline, good compatibility
- **H.265 (HEVC):** Recommended for 8K+, better compression
- **VP9:** Open format, YouTube compatible
- **AV1:** Future-proofing, emerging standard

#### Frame Rates
- **Minimum:** 30 FPS
- **Recommended:** 60 FPS
- **VR Optimal:** 90+ FPS for rendered content

### Metadata Requirements
```json
{
  "video": {
    "projection": "equirectangular",
    "stereoMode": "mono" | "top-bottom" | "left-right",
    "resolution": "7680x4320",
    "frameRate": 60,
    "codec": "h265",
    "bitrate": "100Mbps"
  }
}
```

## 3D Model Specifications

### File Formats

#### Primary Format: glTF 2.0
- **Extension:** .glb (binary) or .gltf (JSON + bins)
- **Features:** PBR materials, animations, morph targets, skins
- **Compression:** Draco mesh compression recommended
- **Texture:** KTX2 with Basis Universal compression

#### Alternative Formats
- **FBX:** For tools without glTF export
- **OBJ:** Simple static meshes only
- **USDZ:** iOS AR Quick Look support

### Geometry Requirements

#### Polygon Budgets
- **Mobile AR:** 50K-100K triangles total scene
- **Standalone VR:** 100K-300K triangles total
- **PC VR:** 500K-2M triangles total
- **Per-Object:** Proportional to screen importance

#### Level of Detail (LOD)
- **LOD0 (High):** Full detail, < 2m viewing distance
- **LOD1 (Medium):** 50% polygons, 2-10m distance
- **LOD2 (Low):** 25% polygons, > 10m distance

### Materials and Textures

#### PBR Material Properties
- **Base Color:** RGB texture, sRGB color space
- **Metallic-Roughness:** Packed texture (metallic in B, roughness in G)
- **Normal Map:** Tangent-space, OpenGL format
- **Occlusion:** Ambient occlusion in red channel
- **Emissive:** Optional self-illumination

#### Texture Specifications
- **Dimensions:** Power of 2 (512, 1024, 2048, 4096)
- **Maximum Size:** 2048×2048 for mobile, 4096×4096 for PC
- **Format:** PNG or JPEG for source, compressed for runtime
- **Compression:** KTX2/Basis Universal for cross-platform

## Spatial Audio Specifications

### Audio Formats

#### Ambisonic Audio
- **Order:** First-order (4 channels) minimum
- **Higher Orders:** Second (9 ch) and third (16 ch) recommended
- **Format:** AmbiX (ACN channel ordering, SN3D normalization)
- **Codec:** Opus, AAC, or FLAC

#### Binaural Audio
- **Channels:** 2 (stereo with HRTF processing)
- **Sample Rate:** 48 kHz recommended
- **Bit Depth:** 24-bit for production, 16-bit for delivery

#### Object-Based Audio
- **Sources:** Individual positioned audio objects
- **Attenuation:** Inverse distance law default
- **Occlusion:** Ray-casting for blocked audio paths

### Technical Requirements
- **Sample Rate:** 44.1 kHz minimum, 48 kHz recommended
- **Bit Depth:** 16-bit minimum, 24-bit for mastering
- **Dynamic Range:** > 60 dB
- **Latency:** < 20ms for interactive audio

## Haptic Feedback Specifications

### Haptic Protocols

#### Vibrotactile Feedback
- **Amplitude:** 0.0 to 1.0 normalized intensity
- **Frequency:** 10-300 Hz range
- **Duration:** 10ms to 5000ms
- **Pattern:** Array of amplitude/duration pairs

#### Force Feedback
- **Force Range:** 0-10 Newtons typical
- **Update Rate:** 1000 Hz minimum
- **Precision:** 0.1 Newton resolution

### Device APIs
- **Web Vibration API:** Basic vibration for web/mobile
- **Unity Haptics:** XR Input Haptics
- **Platform SDKs:** Oculus, SteamVR, PlayStation specific APIs

## Performance Requirements

### Frame Rate Targets
- **VR:** 90 FPS minimum, 120 FPS recommended
- **Mobile AR:** 30 FPS minimum, 60 FPS recommended
- **Desktop XR:** 90 FPS minimum

### Latency Limits
- **Motion-to-Photon:** < 20ms for VR
- **Audio Latency:** < 20ms
- **Input Response:** < 16ms
- **Network Latency:** < 100ms for real-time multi-user

### Resource Budgets
- **Draw Calls:** < 500 per frame (mobile), < 2000 (PC)
- **Memory:** < 2 GB (mobile), < 4 GB (PC VR)
- **Download Size:** < 500 MB (mobile apps), < 2 GB (PC apps)

## Network and Streaming

### Bandwidth Requirements
- **4K 360° Video:** 25-50 Mbps
- **5.7K 360° Video:** 50-75 Mbps
- **8K 360° Video:** 100-200 Mbps
- **Interactive 3D:** 5-10 Mbps

### Adaptive Streaming
- **Protocol:** HLS or DASH for video
- **Quality Levels:** Minimum 3 (low, medium, high)
- **Segment Duration:** 2-10 seconds

## Accessibility Requirements

### Visual Accessibility
- **Text Size:** Minimum 18pt equivalent in VR
- **Contrast Ratio:** WCAG AA (4.5:1) minimum
- **Colorblind Modes:** Deuteranopia, Protanopia, Tritanopia support
- **Subtitles:** Available for all audio content

### Audio Accessibility
- **Captions:** Synchronized text for speech
- **Audio Description:** Narration of visual elements
- **Visual Indicators:** Alternative to audio-only cues

### Interaction Accessibility
- **Multiple Input Methods:** Support gaze, voice, controller, hand tracking
- **Adjustable Speed:** Slow motion options
- **Seated Mode:** All content accessible without standing
- **One-Handed:** Support single hand operation

## Security and Privacy

### Data Collection
- **Minimum Collection:** Only pedagogically necessary data
- **Consent:** Clear opt-in for any tracking beyond basic analytics
- **Anonymization:** Remove PII before storage
- **Retention:** Maximum 2 years unless legally required

### Encryption
- **Transport:** TLS 1.2+ for all network communication
- **Storage:** Encrypted local storage for user data
- **Biometric:** Never store raw eye-tracking or facial data

## Testing and Validation

### Device Testing Matrix
- **VR:** Meta Quest 3, PlayStation VR2, PC VR (SteamVR)
- **AR:** iPhone (ARKit), Android flagship (ARCore), HoloLens 2
- **Mobile:** iOS 16+, Android 12+

### Performance Testing
- **Frame Rate Profiling:** 90th percentile must meet targets
- **Thermal Testing:** Sustained performance over 30 minutes
- **Battery Impact:** < 30% drain per 30 minutes (standalone VR)

### User Testing
- **Sample Size:** Minimum 20 representative users
- **Diversity:** Age, ability, prior experience distribution
- **Metrics:** Completion rate, error rate, satisfaction score

---

© 2025 SmileStory Inc. / WIA
