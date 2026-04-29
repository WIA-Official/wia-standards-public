# WIA-SEMI-017: 3D Touch Standard

## Force Touch, Haptic Feedback & Pressure-Sensitive Touch Interfaces

[![WIA Standard](https://img.shields.io/badge/WIA-SEMI--017-06B6D4)](https://wiabooks.store/tag/wia-3d-touch/)
[![Version](https://img.shields.io/badge/version-1.0-green)](./spec/wia-semi-017-v1.0.md)
[![License](https://img.shields.io/badge/license-CC%20BY--SA%204.0-blue)](https://creativecommons.org/licenses/by-sa/4.0/)

---

## Overview

The WIA-SEMI-017 standard defines comprehensive specifications for **3D Touch technology**, covering:

- **Force Sensing**: Capacitive, strain gauge, and piezoelectric force detection
- **Pressure Sensing**: Multi-level force detection and calibration
- **Haptic Feedback**: LRA, ERM, and piezoelectric haptic actuators
- **Gesture Recognition**: Peek, Pop, and force-based interaction patterns
- **Integration**: iOS, Android, and web platform implementation

**Philosophy:** 홍익인간 (弘益人間) - Benefit All Humanity

---

## Quick Links

- 🌐 **[Landing Page](./index.html)** - Interactive overview with 4-phase implementation
- 🧪 **[Simulator](./simulator/index.html)** - 5-tab force sensing calculator (99 languages)
- 📚 **[Ebook](https://wiabooks.store/tag/wia-3d-touch/)** - Complete guide (English & Korean)
- 📖 **[Specification](./spec/wia-semi-017-v1.0.md)** - Formal standard document
- 💻 **[TypeScript SDK](./api/typescript/)** - Force sensing and haptic SDK
- 🔧 **[GitHub](https://github.com/WIA-Official/wia-standards)** - Source code repository

---

## What is 3D Touch?

3D Touch adds **pressure** as a third dimension to traditional touch interfaces (X-Y position + force/pressure). Users can apply different force levels to trigger distinct actions:

**Traditional Touch:** Tap, swipe, pinch (2D - position only)

**3D Touch:** Light press (Peek), deep press (Pop), force-sensitive drawing (3D - position + force)

### Key Capabilities

**Multi-Level Force Detection:**
- Light touch: 0-50 gf
- Normal tap: 50-150 gf
- Peek gesture: 150-300 gf (preview content)
- Pop gesture: 300-500 gf (open content)

**Haptic Feedback:**
- Tactile confirmation of force gestures
- Click, thud, tick patterns
- Latency: <20ms for natural feel

**Applications:**
- Smartphones: Quick actions, content preview
- Laptops: Force Touch trackpads
- Automotive: Haptic touchscreens (eyes-on-road safety)
- Gaming: Adaptive triggers (PlayStation 5)
- Industrial: Force-controlled robotics

---

## Technology Highlights

### Force Sensing Technologies

**1. Capacitive Force Sensing** (Apple 3D Touch, Huawei Force Touch)
- Grid of electrodes beneath display
- Deformable dielectric layer compresses under force
- Capacitance change measures force
- Multi-point force detection
- Thin form factor (+0.5mm display stack)

**2. Strain Gauge Force Sensing** (Synaptics ClearForce)
- Strain gauges at display corners/edges
- Mechanical deformation measured
- High accuracy (±0.5-2%)
- Linear response
- Lower cost than capacitive

**3. Piezoelectric Force Sensing** (TDK PowerHap)
- Piezo material generates charge under force
- Ultra-fast response (<1ms)
- Dual-use: sensing + haptic actuation
- High voltage driver required (100-200V)

### Haptic Actuator Technologies

**1. Linear Resonant Actuator (LRA)**
- Apple Taptic Engine, AAC actuators
- Operates at resonant frequency (150-250 Hz)
- Sharp, precise haptic feel
- Low power (100-300 mW)
- Response time: 10-15ms

**2. Eccentric Rotating Mass (ERM)**
- Traditional vibration motors
- Off-center weight on motor shaft
- Low cost ($0.10-0.50)
- Slower response (40-80ms)
- Mushy feel

**3. Piezoelectric Actuators**
- TDK PowerHap, Boreas BOS1211
- Ultra-fast response (<1ms)
- Wide frequency range (DC-10kHz)
- Thin form factor
- Dual-use (sensing + actuation)

**4. Voice Coil Actuator (VCA)**
- PlayStation 5 DualSense adaptive triggers
- Variable resistance (0-8N force range)
- Simulates weapon recoil, bowstring tension
- Higher power (200-800 mW)

---

## File Structure

```
standards/3d-touch/
├── index.html                    # Landing page with dark theme
├── simulator/
│   └── index.html                # 5-tab simulator (99 languages)
├── ebook/
│   ├── en/                       # 9 English chapters (15KB+ each)
│   │   ├── 01-cover.md
│   │   ├── 02-market-analysis.md
│   │   ├── 03-force-touch-technology.md
│   │   ├── 04-haptic-feedback.md
│   │   ├── 05-pressure-sensing.md
│   │   ├── 06-force-gestures.md
│   │   ├── 07-latency-performance.md
│   │   ├── 08-ui-integration.md
│   │   └── 09-future-trends.md
│   └── ko/                       # 9 Korean chapters (15KB+ each)
│       └── 01-09-chapter.md
├── spec/                         # Specification documents
│   ├── wia-semi-017-v1.0.md     # Main specification (15KB)
│   ├── force-calibration.md     # Calibration procedures
│   ├── haptic-design.md         # Haptic feedback design
│   └── gesture-protocols.md     # Gesture recognition
├── api/
│   └── typescript/              # TypeScript SDK
│       ├── package.json
│       ├── tsconfig.json
│       └── src/
│           ├── types.ts         # Type definitions
│           └── index.ts         # Force3DController class
└── README.md                    # This file
```

---

## Quick Start

### 1. Explore the Simulator

Open **[simulator/index.html](./simulator/index.html)** in a web browser to try:

**Tab 1: Force Sensing Specifications**
- Calculate force resolution, dynamic range, sensitivity
- Visualize force levels in real-time

**Tab 2: Pressure Level Calculations**
- Configure Peek/Pop thresholds
- Calculate pressure in kPa
- Generate force threshold code

**Tab 3: Touch Protocols**
- Calculate total touch latency
- Optimize scanning and processing delays
- Event debouncing configuration

**Tab 4: UI Framework Integration**
- Generate code for React, Vue, Swift, Android, Flutter
- Haptic feedback integration examples
- Force threshold configuration

**Tab 5: Force Calibration Testing**
- Multi-point calibration calculator
- Temperature compensation
- Linearity error analysis

### 2. Read the Ebook

Download the comprehensive guide from **[WIA Books](https://wiabooks.store/tag/wia-3d-touch/)**:

**Topics covered:**
- Force sensing physics and architectures
- Haptic actuator technologies (LRA, ERM, piezo, VCA)
- Calibration algorithms and procedures
- Gesture recognition (Peek, Pop, long press)
- Market analysis and adoption trends
- iOS, Android, web integration
- Future trends (ultrasonic haptics, AI-driven force sensing)

### 3. Use the TypeScript SDK

Install the SDK:

```bash
npm install @wia/3d-touch-sdk
```

**Basic usage:**

```typescript
import { Force3DController, createDefaultConfig, ForceGestureType, HapticPresets } from '@wia/3d-touch-sdk';

// Initialize controller
const config = createDefaultConfig();
const force3D = new Force3DController(config);
await force3D.initialize();

// Listen for Peek gesture
force3D.on(ForceGestureType.PEEK, (event) => {
  console.log('Peek gesture detected:', event.force.force, 'gf');

  // Trigger haptic feedback
  force3D.triggerHaptic(HapticPresets.click());

  // Show preview UI
  showPreview(event);
});

// Listen for Pop gesture
force3D.on(ForceGestureType.POP, (event) => {
  console.log('Pop gesture detected:', event.force.force, 'gf');

  // Trigger stronger haptic
  force3D.triggerHaptic(HapticPresets.thud());

  // Open full content
  openContent(event);
});

// Update force measurement (called from touch event handler)
touchElement.addEventListener('touchmove', (e) => {
  const touch = e.touches[0];
  const force = touch.force * 500; // Normalize to gf

  force3D.updateForce({
    force: force,
    normalizedForce: touch.force,
    timestamp: Date.now(),
    x: touch.clientX,
    y: touch.clientY
  });
});
```

**Advanced: Custom calibration**

```typescript
// Perform multi-point calibration
const calibrationPoints = [
  { force: 0, rawValue: 0, temperature: 25 },
  { force: 50, rawValue: 512, temperature: 25 },
  { force: 150, rawValue: 1536, temperature: 25 },
  { force: 250, rawValue: 2560, temperature: 25 },
  { force: 400, rawValue: 4096, temperature: 25 }
];

await force3D.calibrate(calibrationPoints);

// Adjust thresholds for user preference
force3D.updateThresholds({
  peek: 180,  // Easier to trigger (default: 200)
  pop: 380    // Easier to trigger (default: 400)
});
```

### 4. iOS Integration (Swift)

```swift
import UIKit

class ForceViewController: UIViewController {
    override func touchesMoved(_ touches: Set<UITouch>, with event: UIEvent?) {
        guard let touch = touches.first else { return }

        // Measure force
        let force = touch.force / touch.maximumPossibleForce
        let forceGrams = force * 500  // Assume 500gf max

        // Detect gestures
        if forceGrams >= 400 {
            handlePop()
        } else if forceGrams >= 200 {
            handlePeek()
        }
    }

    func handlePeek() {
        // Haptic feedback
        let generator = UIImpactFeedbackGenerator(style: .light)
        generator.impactOccurred()

        // Show preview
        showPreview()
    }

    func handlePop() {
        // Stronger haptic
        let generator = UIImpactFeedbackGenerator(style: .medium)
        generator.impactOccurred()

        // Open content
        openContent()
    }
}
```

### 5. Android Integration (Kotlin)

```kotlin
import android.view.MotionEvent
import android.view.HapticFeedbackConstants

override fun onTouchEvent(event: MotionEvent): Boolean {
    val pressure = event.pressure
    val maxPressure = event.device.getMotionRange(MotionEvent.AXIS_PRESSURE).max
    val normalizedForce = pressure / maxPressure
    val forceGrams = normalizedForce * 500

    when {
        forceGrams >= 400 -> {
            performHapticFeedback(HapticFeedbackConstants.LONG_PRESS)
            handlePop()
        }
        forceGrams >= 200 -> {
            performHapticFeedback(HapticFeedbackConstants.VIRTUAL_KEY)
            handlePeek()
        }
    }

    return true
}
```

---

## Specification Highlights

### Force Sensing Requirements

**Accuracy:**
- Absolute accuracy: ±2-5% of full scale
- Repeatability: ±1-2%
- Linearity: ±3% maximum deviation

**Resolution:**
- Minimum: 256 levels (8-bit)
- Recommended: 1024 levels (10-bit)
- High-end: 4096 levels (12-bit)

**Force Range:**
- Minimum detectable: ≤1 gf
- Maximum force: 400-500 gf
- Dynamic range: ≥60 dB

### Calibration Requirements

**Factory calibration:**
- Minimum 5 force levels
- Spatial calibration at 5-9 screen positions
- Temperature calibration at 0°C, 25°C, 40°C, 60°C
- Data stored in non-volatile memory

**Runtime calibration:**
- Auto-zero when device idle
- Temperature compensation (±0.1% per °C)
- User-adjustable sensitivity (Light, Medium, Firm)

### Haptic Feedback Requirements

**Latency:**
- Force detection: <8ms
- Total response (force to haptic): <20ms

**LRA specifications:**
- Resonant frequency: 150-250 Hz
- Acceleration: 1-4 G
- Response time: <15ms
- Lifetime: ≥5 million cycles

**Waveforms:**
- Click: 10-15ms duration, 3-5 G peak
- Thud: 25-35ms duration, 2-3 G peak
- Tick: 8-12ms duration, 1-2 G peak

### Testing Requirements

**Durability:**
- 1 million force sensor activations
- 5 million haptic actuator cycles
- <10% calibration drift

**Environmental:**
- Operating: 0°C to 60°C
- Storage: -20°C to 70°C
- Humidity: 10-90% RH non-condensing
- Drop test: 1.5m onto hard surface

---

## Market Overview

### Global Market Size (2025)

- Force-sensitive displays: $8.2 billion
- Haptic actuators: $12.6 billion
- Force sensors: $6.0 billion
- **Total market: $28.3 billion**

### Key Applications

**Smartphones (Declining):**
- Apple discontinued 3D Touch in 2019 (cost, complexity)
- Shifted to Haptic Touch (software long-press)
- Android manufacturers minimal adoption

**Automotive (Growing):**
- 45 million vehicles with haptic touchscreens by 2025
- Replacing physical buttons (cost parity at $200-250/vehicle)
- Regulatory driver: "eyes-on-road" safety requirements

**Gaming (Strong Growth):**
- PlayStation 5 DualSense adaptive triggers
- Force feedback in VR controllers
- Haptic gloves for immersive experiences

**Industrial & Medical (Steady):**
- Surgical robot force feedback
- Collaborative robot (cobot) safety
- Precision assembly force control

### Leading Manufacturers

**Force Sensing:**
- Bosch Sensortec (automotive)
- Texas Instruments (controller ICs)
- Synaptics (ClearForce)
- Cirrus Logic (integrated haptic + force)

**Haptic Actuators:**
- AAC Technologies (38% market share)
- Nidec Corporation (22%)
- TDK Corporation (15% - piezoelectric)
- Alps Alpine (12%)

---

## Future Trends

### Ultrasonic Haptics (Mid-Air Tactile)

**Technology:** Ultrasonic phased arrays create tactile sensations in mid-air
**Leaders:** Ultraleap (UK), Actronika (France)
**Applications:** Automotive HMI, AR displays, public kiosks
**Market:** $3.2 billion by 2030

### Flexible Display Force Sensing

**Challenge:** Foldable displays need flexible force sensors
**Solutions:** PVDF piezo film, printed strain gauges, optical sensing
**Status:** Early research, not yet commercial

### AI-Enhanced Force Detection

**Approach:** Neural networks predict force from multiple sensor modalities
**Inputs:** Touch area, accelerometer, gyroscope, historical patterns
**Benefits:** Improved accuracy without hardware cost, user-adaptive thresholds

---

## Accessibility

### Adjustable Force Thresholds

Users can customize force sensitivity:
- **Light:** 0.7× thresholds (easier to trigger)
- **Medium:** 1.0× thresholds (default)
- **Firm:** 1.4× thresholds (harder to trigger)

### Alternative Interaction Methods

All force gestures must have alternatives:
- Long-press (500-800ms) for Peek
- Double-long-press for Pop
- Context menu buttons
- AssistiveTouch shortcuts

### Haptic Feedback Control

Users can:
- Disable haptic feedback
- Adjust intensity (0-100%)
- Choose patterns (subtle, standard, pronounced)

---

## Contributing

We welcome contributions to the WIA-SEMI-017 standard!

**How to contribute:**

1. **Fork the repository:**
   ```bash
   git clone https://github.com/WIA-Official/wia-standards.git
   cd wia-standards/standards/3d-touch
   ```

2. **Create a feature branch:**
   ```bash
   git checkout -b feature/your-improvement
   ```

3. **Make changes:**
   - Update specifications in `spec/`
   - Improve SDK in `api/typescript/`
   - Add simulator features in `simulator/`
   - Enhance ebook chapters in `ebook/`

4. **Submit pull request:**
   - Include detailed description
   - Reference related issues
   - Follow coding style guidelines

**Areas needing contribution:**
- Additional language translations (simulator, ebook)
- Platform-specific SDKs (Python, Java, C++)
- Hardware reference implementations
- Test fixtures and calibration tools
- Example applications (iOS, Android, web)

---

## License

**Content License:** [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/)
**Code License:** MIT (SDK and simulator code)

You are free to:
- Share: Copy and redistribute
- Adapt: Remix, transform, build upon
- Commercial use: Use in commercial products

Under these terms:
- **Attribution:** Credit WIA-SEMI-017 standard
- **ShareAlike:** Distribute adaptations under same license
- **No additional restrictions:** Cannot apply legal/technical measures that restrict others

---

## Contact & Support

**WIA Organization:**
- Website: https://wiabooks.store
- GitHub: https://github.com/WIA-Official
- Email: contact@wia.org

**Resources:**
- Standard specification: [spec/wia-semi-017-v1.0.md](./spec/wia-semi-017-v1.0.md)
- Ebook: https://wiabooks.store/tag/wia-3d-touch/
- Simulator: [simulator/index.html](./simulator/index.html)
- TypeScript SDK: [api/typescript/](./api/typescript/)

**Report issues:**
- GitHub Issues: https://github.com/WIA-Official/wia-standards/issues
- Label: `WIA-SEMI-017`

---

## Acknowledgments

The WIA-SEMI-017 standard was developed with input from:

- Force sensing hardware engineers
- Haptic feedback researchers
- Mobile device manufacturers
- Automotive HMI designers
- Gaming hardware developers
- Accessibility advocates
- Open-source community

Special thanks to the pioneers:
- Apple (3D Touch, Force Touch innovation)
- Sony (PlayStation DualSense)
- TDK (PowerHap piezoelectric actuators)
- Ultraleap (mid-air haptics)

---

## Citation

If you use WIA-SEMI-017 in research or products, please cite:

```bibtex
@techreport{wia-semi-017,
  title = {WIA-SEMI-017: 3D Touch Standard},
  author = {WIA Technical Committee},
  institution = {World Certification Industry Association},
  year = {2025},
  version = {1.0},
  url = {https://github.com/WIA-Official/wia-standards/tree/main/standards/3d-touch}
}
```

---

**홍익인간 (弘益人間)** · Benefit All Humanity

© 2025 SmileStory Inc. / WIA
All rights reserved.

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
