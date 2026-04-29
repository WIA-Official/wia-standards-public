# Chapter 4: Haptic Feedback Systems

## Linear Resonant Actuators, Waveform Synthesis, and Tactile Design

### Introduction

Haptic feedback transforms digital interactions into physical sensations, providing tactile confirmation of user actions and creating immersive experiences. This chapter explores the actuator technologies (LRA, ERM, piezoelectric), driver electronics, waveform synthesis techniques, and design principles for creating compelling haptic experiences in smartphones, wearables, gaming controllers, and automotive interfaces.

---

## Haptic Actuator Technologies

### Linear Resonant Actuator (LRA)

**Principle:**
Electromagnetic actuator operating at its mechanical resonant frequency for maximum efficiency and amplitude.

**Construction:**
- **Moving mass**: Permanent magnet or coil assembly
- **Spring system**: Mechanical springs provide restoring force
- **Electromagnetic coil**: Drives oscillation
- **Housing**: Constrains linear motion

**Operating principle:**
```
F = B × I × L

Where:
F = force (N)
B = magnetic flux density (Tesla)
I = coil current (Amperes)
L = coil wire length (meters)
```

**Key specifications:**
- **Resonant frequency**: 150-250 Hz typical (optimized for human tactile perception)
- **Acceleration**: 1-4 G (9.81-39.2 m/s²)
- **Response time**: 10-20 ms (rise time to peak amplitude)
- **Power consumption**: 100-300 mW during activation
- **Lifetime**: 5-10 million cycles
- **Size**: 7×7×2mm (small), 10×10×3mm (medium), 13×11×5mm (large)

**Advantages:**
- Sharp, precise haptic feel
- Low power consumption (resonant operation)
- Programmable waveforms
- Compact size

**Disadvantages:**
- Limited frequency range (sharp resonance peak)
- Requires matching to resonant frequency
- More expensive than ERM
- Z-axis (out of display) or X-axis (lateral) orientation

**Examples:**
- **Apple Taptic Engine**: Z-axis LRA in iPhone, Apple Watch
- **AAC 1030-series**: X-axis LRA for Android phones
- **Nidec 1040**: High-acceleration LRA for gaming

### Eccentric Rotating Mass (ERM)

**Principle:**
Off-center weight on motor shaft creates unbalanced rotational force.

**Construction:**
- **DC motor**: Brushed or brushless
- **Eccentric mass**: Asymmetric weight on shaft
- **Housing**: Contains motor

**Operating characteristics:**
- **Frequency range**: 50-200 Hz (varies with voltage)
- **Acceleration**: 0.5-2 G typical
- **Response time**: 40-80 ms (motor spin-up time)
- **Power consumption**: 60-150 mW continuous
- **Lifetime**: 1-5 million cycles (brushed motor wear)
- **Size**: 4-10mm diameter, cylindrical or coin-type

**Advantages:**
- Very low cost ($0.10-0.50 vs. $2-3 for LRA)
- Simple driver circuit (PWM control)
- Omnidirectional vibration
- Wide availability

**Disadvantages:**
- Mushy, less precise feel
- Slow response time (momentum)
- Higher power consumption
- Limited waveform control
- Mechanical wear (brushes)

**Applications:**
- Budget smartphones (<$200 price point)
- Basic vibration alerts
- Older gaming controllers
- Pagers and notification devices

### Piezoelectric Actuators

**Principle:**
Piezoelectric material deforms when voltage is applied, creating displacement and force.

**Materials:**
- **PZT (Lead Zirconate Titanate)**: High strain, widely used
- **Lead-free alternatives**: BaTiO₃, KNN (regulatory compliance)
- **PVDF (Polyvinylidene Fluoride)**: Flexible polymer film

**Types:**

**1. Bending Actuators (Unimorph, Bimorph)**
- PZT layer bonded to flexible substrate
- Voltage creates bending motion
- Displacement: 0.1-2 mm typical
- Force: 0.1-5 N

**2. Stack Actuators**
- Multiple PZT layers stacked
- High force, low displacement
- Force: 10-1000 N
- Displacement: 10-100 μm

**Key specifications:**
- **Response time**: <1 ms (ultra-fast)
- **Frequency range**: DC to 10 kHz+
- **Driving voltage**: 100-200V typical (requires boost converter)
- **Power consumption**: 10-100 mW (capacitive load)
- **Size**: 10×10×0.5mm (bending), 5×5×5mm (stack)

**Advantages:**
- Ultra-fast response (microseconds)
- Precise control
- Wide frequency range
- Thin form factor
- Dual-use (sensing + actuation)

**Disadvantages:**
- High voltage driver required
- Brittle (mechanical shock sensitive)
- Limited displacement (stack type)
- Higher cost than LRA/ERM

**Applications:**
- **TDK PowerHap**: Laptop trackpads, automotive touchscreens
- **Boreas BOS1211**: Wideband HD haptics
- **PI Ceramic**: Industrial precision haptics

### Voice Coil Actuator (VCA)

**Principle:**
Electromagnetic coil moves against permanent magnet (similar to speaker).

**Construction:**
- **Permanent magnet**: Provides magnetic field
- **Voice coil**: Moves within magnetic gap
- **Spring return**: Centers coil
- **Linear bearing**: Guides motion

**Characteristics:**
- **Force**: 0.5-10 N typical
- **Displacement**: 1-5 mm
- **Frequency response**: 10-1000 Hz
- **Response time**: 5-15 ms
- **Power**: 200-800 mW

**Applications:**
- **PlayStation 5 DualSense adaptive triggers**: Variable resistance
- **VR haptic gloves**: Finger force feedback
- **Force feedback steering wheels**: Resistance simulation

---

## Haptic Driver Electronics

### LRA Driver Architecture

**Components:**

**1. H-Bridge Driver**
Bidirectional current control for AC waveform generation

```
        +V
         |
       Q1  Q2
         | / |
Coil  ----●----
         | \ |
       Q3  Q4
         |
        GND

PWM control: Q1+Q4 on → current forward
             Q2+Q3 on → current reverse
```

**2. Resonant Frequency Tracking**
LRA resonant frequency varies with temperature, aging, mechanical load

**Auto-resonance detection:**
- Apply frequency sweep (140-260 Hz)
- Measure back-EMF or impedance
- Identify resonant peak
- Lock to resonant frequency for maximum efficiency

**3. Braking Circuit**
Active braking stops vibration quickly after waveform ends

**Methods:**
- **Short coil terminals**: Electromagnetic braking
- **Reverse current pulse**: Active damping
- **Optimized decay waveform**: Shaped deceleration

### ERM Driver Circuit

**Simple PWM driver:**
```
PWM → MOSFET → ERM Motor → GND
               (Flyback diode for inductive kickback)
```

**Frequency control:**
PWM duty cycle controls motor speed → vibration frequency

**Typical:** 50-100% duty cycle → 100-200 Hz vibration

### Piezoelectric Driver

**Boost Converter:**
Convert 3.3V/5V battery to 100-200V for piezo

**Topology:**
- Flyback converter or boost converter
- High voltage MOSFET (200-300V rating)
- Feedback control for voltage regulation

**Drive signal:**
- AC square wave or sine wave
- Frequency: DC to 10 kHz depending on application
- Amplitude modulation for intensity control

---

## Haptic Waveform Synthesis

### Basic Waveform Types

**1. Click (Transient)**
Short, sharp haptic pulse simulating button click

```
Amplitude
   ^
   |     ╱╲
   |    ╱  ╲
   |   ╱    ╲___
   |  ╱
   +─────────────> Time
   0   5   10  15 ms

Rise time: 2-5 ms
Peak: 3-5 G
Decay: 8-12 ms
Total duration: 10-15 ms
```

**2. Thud (Soft Impact)**
Longer, softer haptic pulse

```
Amplitude
   ^
   |    ╱‾‾╲
   |   ╱    ╲
   |  ╱      ╲__
   | ╱
   +─────────────> Time
   0   10  20  30 ms

Rise time: 5-10 ms
Peak: 2-3 G
Decay: 15-20 ms
Total duration: 25-35 ms
```

**3. Continuous Vibration**
Sustained vibration for alerts

```
Amplitude
   ^
   |  ╱╲╱╲╱╲╱╲╱╲
   | ╱  ┴  ┴  ┴ ╲
   |╱            ╲
   +─────────────────> Time
   0     500ms    1s

Envelope: Fade in/out
Frequency: 150-200 Hz (LRA resonance)
Duration: 100ms - several seconds
```

**4. Ramp (Gradual Transition)**
Smoothly increasing or decreasing intensity

```
Amplitude
   ^
   |           ╱
   |         ╱
   |       ╱
   |     ╱
   |   ╱
   +─────────────> Time
   0      200  ms

Linear or exponential ramp
Applications: Force feedback, volume control
```

### Advanced Waveform Techniques

**Frequency Modulation:**
Vary vibration frequency over time for richer haptic textures

```python
def frequency_sweep(duration_ms, f_start, f_end):
    samples = []
    for t in range(duration_ms):
        freq = f_start + (f_end - f_start) * (t / duration_ms)
        amplitude = sin(2 * pi * freq * t / 1000)
        samples.append(amplitude)
    return samples
```

**Amplitude Modulation:**
Vary vibration intensity to create complex patterns

**Example: Pulsing effect**
```
Base frequency: 200 Hz (LRA resonance)
Modulation frequency: 10 Hz (pulse rate)
Result: 10 pulses per second sensation
```

**Multi-Actuator Patterns:**
Coordinate multiple LRAs for spatial haptic effects

**Example: Directional feedback**
- Left LRA activates → tactile sensation on left side
- Right LRA activates → sensation on right side
- Sequential activation → perceived motion across device

### Haptic Design Patterns (Apple Guidelines)

**UIFeedbackGenerator Types (iOS):**

**1. Impact Feedback**
- **Light**: Soft tap (e.g., keyboard key press)
- **Medium**: Standard tap (e.g., button press)
- **Heavy**: Firm impact (e.g., deletion, dismissal)

**2. Selection Feedback**
- Used when scrolling through lists
- Rapid, subtle ticks as each item passes

**3. Notification Feedback**
- **Success**: Task completed (e.g., sent message)
- **Warning**: Attention needed (e.g., low battery)
- **Error**: Action failed (e.g., invalid input)

**Implementation:**
```swift
let generator = UIImpactFeedbackGenerator(style: .medium)
generator.prepare()  // Pre-load actuator for minimal latency
generator.impactOccurred()  // Trigger haptic
```

**Best practices:**
- **Prepare before expected trigger**: Reduces latency from 15ms to 5ms
- **Don't overuse**: Haptics for meaningful events only (not every touch)
- **Match intensity to importance**: Subtle for minor events, strong for critical
- **Respect user settings**: Honor "Haptic Feedback" toggle in system settings

---

## Latency Optimization

### Latency Budget Breakdown

**Total perceived latency**: Touch to haptic feedback

**Components:**

**1. Touch detection**: 5-8 ms
- Capacitive touch scan: 3-5 ms (120-240 Hz scan rate)
- Touch processing: 2-3 ms (filtering, gesture recognition)

**2. Force measurement**: 3-8 ms
- Force sensor scan: 2-5 ms (120-1000 Hz sampling)
- Force calculation: 1-3 ms (linearization, calibration)

**3. Event propagation**: 1-5 ms
- Interrupt handling: 0.5-1 ms
- OS event queue: 0.5-2 ms
- Application callback: 0-2 ms (if app-triggered)

**4. Haptic driver response**: 5-15 ms
- LRA spin-up: 10-15 ms
- Piezo response: <1 ms
- ERM spin-up: 40-80 ms

**Total latency:**
- **LRA**: 14-36 ms (optimized: 14-20 ms)
- **Piezoelectric**: 9-22 ms
- **ERM**: 49-101 ms

**Target latency:** <20 ms for natural feel

### Latency Reduction Techniques

**1. Prepare Actuator (Pre-bias)**
Apply small DC bias current to LRA to overcome static friction

```c
// Pre-load LRA before expected touch
void prepare_haptic() {
    lra_set_bias_current(5_mA);  // Small current, no vibration
    delay_ms(2);  // Allow settling
}

// Now trigger happens with reduced latency
void trigger_haptic() {
    lra_play_waveform(CLICK_WAVEFORM);  // Faster response
}
```

**Latency improvement:** 10-15 ms → 5-8 ms

**2. Predictive Triggering**
Anticipate user action and pre-load haptic

**Example: Button press**
- Detect finger approach via proximity sensor or hover
- Prepare haptic actuator
- When touch occurs, instant haptic response

**3. Concurrent Processing**
Pipeline touch detection, force measurement, and haptic triggering

```
Cycle 1: [Touch Scan]
Cycle 2: [Touch Scan] [Force Measure] 
Cycle 3: [Touch Scan] [Force Measure] [Haptic Trigger]
         ↑ Pipelined - results from Cycle 1
```

**Latency improvement:** Sequential (15ms) → Pipelined (8ms)

**4. Hardware Acceleration**
Dedicated haptic coprocessor (e.g., Cirrus Logic CS40L26)

**Features:**
- Waveform memory (pre-loaded patterns)
- Trigger from GPIO/I2C command
- No CPU intervention required
- Latency: 2-5 ms from trigger to output

---

## Haptic Texture Synthesis

### Surface Texture Simulation

**Concept:**
Generate haptic patterns that simulate touching different materials

**Examples:**

**1. Rough Surface (e.g., sandpaper)**
```python
def rough_texture():
    # Random vibration bursts
    for i in range(20):  # 20 random ticks
        amplitude = random(0.5, 1.0)
        duration_ms = random(5, 15)
        trigger_haptic(amplitude, duration_ms)
        delay_ms(random(10, 30))
```

**2. Smooth Surface (e.g., silk)**
```python
def smooth_texture():
    # Gentle, continuous low-frequency vibration
    frequency = 80  # Hz
    amplitude = 0.3  # Low intensity
    duration_ms = 500
    play_continuous_vibration(frequency, amplitude, duration_ms)
```

**3. Bumpy Surface (e.g., braille dots)**
```python
def bumpy_texture(bump_positions):
    for pos in bump_positions:
        if finger_position == pos:
            trigger_haptic(amplitude=0.8, duration_ms=10)
        delay_ms(5)
```

**Applications:**
- **Touchscreen keyboards**: Different key textures
- **VR/AR**: Material interaction
- **Accessibility**: Tactile patterns for visually impaired
- **Automotive**: Confirm button presses without looking

### Dynamic Haptic Effects

**Inertia Simulation:**
Haptic feedback proportional to scrolling speed

```python
def scroll_haptics(velocity):
    # Faster scroll → stronger haptic ticks
    tick_strength = min(0.3 + velocity * 0.1, 1.0)
    tick_frequency = max(5, velocity * 2)  # Hz (ticks per second)
    
    for tick in range(int(tick_frequency)):
        trigger_haptic(amplitude=tick_strength, duration_ms=5)
        delay_ms(int(1000 / tick_frequency))
```

**Boundary/Limit Feedback:**
Strong haptic when reaching end of scrollable content

```python
def check_scroll_boundary(position, max_position):
    if position >= max_position:
        # Hit boundary - strong haptic
        trigger_haptic(amplitude=1.0, duration_ms=20, waveform=THUD)
```

**Elastic Rebound:**
Haptic feedback simulating spring-back effect

```
User drags past boundary → Content stretches (visual)
                         → Release triggers rebound
                         → Haptic matches rebound timing
```

---

## Haptic Design for Accessibility

### Haptic Substitutes for Visual/Audio Cues

**1. Navigation Feedback**
Haptic patterns indicate location in UI hierarchy

**Pattern language:**
- **Single tap**: Item selected
- **Double tap**: Entered sub-menu
- **Triple tap**: Reached end of list
- **Long vibration**: Error or invalid action

**2. Notification Types**
Distinct haptic signatures for different notifications

**Message types:**
- **Email**: Short pulse
- **Text message**: Double short pulse
- **Calendar alert**: Rising intensity ramp
- **Warning**: Three rapid pulses

**Example (Android):**
```java
// Custom vibration pattern for accessibility
long[] pattern = {0, 100, 50, 100};  // [delay, vibrate, pause, vibrate]
VibrationEffect effect = VibrationEffect.createWaveform(pattern, -1);
vibrator.vibrate(effect);
```

**3. Force-Based Interaction for Motor Impairments**
Force sensing can **reduce** precision requirements:

**Traditional touch:**
- Requires precise tap on small target
- Difficult for tremor or limited dexterity

**Force-enhanced touch:**
- Large touch area + force level distinguishes intent
- Light touch → Preview
- Firm press → Activate
- Easier for users with motor challenges

---

## Gaming Haptics

### Adaptive Triggers (PlayStation 5 DualSense)

**Technology:**
Voice coil actuator (VCA) in L2/R2 triggers provides variable resistance

**Force range:**
- **Minimum resistance**: 0.1 N (free trigger pull)
- **Maximum resistance**: 8 N (trigger fully locked)
- **Positions**: 256 discrete positions (0-255)

**Modes:**

**1. Variable Resistance**
Resistance increases with trigger pull

```c
// Bowstring tension simulation
void bowstring_haptics(uint8_t trigger_position) {
    // Resistance increases exponentially
    float resistance = pow(trigger_position / 255.0, 2) * 8.0;  // 0-8N
    set_trigger_resistance(resistance);
}
```

**2. Vibration**
High-frequency vibration in trigger

**Example: Gunfire**
```
Trigger pulled → 10-50 Hz vibration pulses
Simulates weapon recoil
```

**3. Trigger Locking**
Trigger becomes stiff at specific position

**Example: Two-stage trigger (gun safety)**
- First stage: Light pull (10% resistance)
- Safety detent at 50% → Increased resistance
- Full pull after detent: Fire

**4. Weapon Jam Simulation**
Trigger completely locks, cannot be pulled further

```c
void weapon_jam() {
    set_trigger_resistance(8.0);  // Maximum resistance
    set_trigger_position_lock(current_position);
    trigger_vibration(frequency=100, duration=200);
}
```

### HD Rumble (Nintendo Switch)

**Technology:**
Linear resonant actuators (LRA) in each Joy-Con

**Capabilities:**
- Wide frequency range (80-600 Hz with envelope modulation)
- Precise amplitude control
- Synchronized dual-actuator effects

**Example effects:**

**1. Ice Cubes in a Glass**
```python
def ice_cubes_effect():
    # Random taps simulating ice hitting glass
    for i in range(random(3, 8)):
        frequency = random(300, 500)  # Hz
        amplitude = random(0.3, 0.8)
        duration = random(5, 15)  # ms
        trigger_lra(frequency, amplitude, duration)
        delay_ms(random(50, 200))
```

**2. Ball Rolling**
```python
def rolling_ball(position_x):
    # Left Joy-Con vibrates when ball on left side
    left_amplitude = max(0, 1.0 - position_x)
    right_amplitude = max(0, position_x)
    
    set_lra_amplitude(LEFT_JOYCON, left_amplitude)
    set_lra_amplitude(RIGHT_JOYCON, right_amplitude)
```

---

## Automotive Haptic Touchscreens

### Design Requirements

**Safety critical:**
- Driver must receive tactile confirmation without looking
- Response time <20ms for "eyes on road" interaction
- Haptic feedback distinct enough to feel through gloves

**Environmental challenges:**
- Operating temperature: -40°C to +85°C
- Vibration from vehicle (engine, road)
- Variable ambient noise (need tactile, not audio cues)

**Haptic Actuator Placement:**

**1. Distributed Actuators**
Multiple LRAs or piezo actuators behind touchscreen

**Typical configuration:**
- 4-8 actuators in grid pattern
- Each actuator covers 100-200 cm² area
- Localized haptic feedback indicates touched zone

**2. Edge-Mounted Actuators**
Piezoelectric actuators on display perimeter

**Propagation:**
Bending waves propagate through glass
User feels haptic at touch point

**Example: TDK PowerHap**
- 4 piezo actuators on display edges
- Waveform tuning creates localized sensation
- <5ms latency from touch to haptic

### Haptic Feedback Patterns for HMI

**Climate control buttons:**
- Light click: Temperature adjust (±1°C)
- Medium click: Fan speed change
- Long vibration: Mode switch (heat/cool/auto)

**Volume control slider:**
- Tick every 5% (18-20 ticks from 0-100%)
- Stronger tick at 50% (midpoint reference)
- Long pulse at 0% and 100% (limits)

**Safety-critical alerts:**
- **Lane departure**: Steering wheel vibration (left/right directional)
- **Forward collision**: Strong haptic pulse + audible alert
- **Blind spot**: Gentle directional haptic when turn signal active

---

## Conclusion

Haptic feedback transforms cold glass touchscreens into expressive, tactile interfaces. Effective haptic design requires understanding actuator physics (LRA resonance, piezo response), driver electronics (resonant tracking, waveform synthesis), latency optimization, and human perception.

**Key principles:**

1. **Match actuator to application**: LRA for mobile, piezo for automotive, VCA for gaming
2. **Minimize latency**: <20ms total for natural feel
3. **Design meaningful patterns**: Haptics for important events, not every touch
4. **Consider accessibility**: Haptic patterns can substitute for visual/audio cues
5. **Test with real users**: Haptic perception is subjective and context-dependent

**Next Chapter Preview:** Chapter 5 explores pressure sensing in depth, covering multi-level force detection, calibration algorithms, and force threshold optimization for consistent user experiences across varied usage scenarios.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
