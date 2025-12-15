# WIA Myoelectric Calibration Guide

Proper calibration is essential for accurate gesture recognition. This guide
covers initial setup, routine calibration, and optimization techniques.

## Overview

Calibration creates a personalized model of your unique muscle signals.
Every user has different:
- Muscle strength and control
- Electrode contact quality
- Skin conductivity
- Signal baseline levels

Plan for **5-10 minutes** of calibration per session.

## Pre-Calibration Setup

### Environment

1. **Quiet location** - Minimize electrical interference
2. **Comfortable seating** - Arm supported, muscles relaxed
3. **Room temperature** - Avoid sweating (affects electrodes)
4. **Remove watches/jewelry** - Can cause signal artifacts

### Electrode Preparation

1. Clean forearm with alcohol wipe
2. Wait 30 seconds for skin to dry
3. Apply fresh electrodes
4. Ensure firm, complete contact
5. Allow 2-3 minutes for impedance to stabilize

### Verify Signal Quality

Before calibration, check:

```
Serial Monitor Output:
CH1: 512 ±20 (at rest)
CH2: 510 ±20 (at rest)
```

Good signal: Stable baseline, clear response to muscle activation
Bad signal: Noisy, drifting, or no response to activation

## Calibration Procedure

### Step 1: Baseline Recording (30 seconds)

**Goal**: Establish your rest signal level

1. Relax completely
2. Keep arm still
3. Breathe normally
4. Don't think about moving

System records:
- Average rest amplitude
- Noise floor
- Baseline drift

### Step 2: Maximum Voluntary Contraction (MVC)

**Goal**: Establish your maximum signal level

For each gesture:

1. **Hand Open MVC**
   - Extend all fingers forcefully
   - Hold 3 seconds
   - Repeat 3 times

2. **Hand Close MVC**
   - Make strongest fist possible
   - Hold 3 seconds
   - Repeat 3 times

3. **Wrist Flexion MVC**
   - Flex wrist toward palm
   - Hold 3 seconds
   - Repeat 3 times

4. **Wrist Extension MVC**
   - Extend wrist away from palm
   - Hold 3 seconds
   - Repeat 3 times

### Step 3: Gesture Training (5-10 reps each)

**Goal**: Teach the classifier your gesture patterns

For each gesture:

1. Start from rest position
2. Wait for "BEGIN" prompt
3. Perform gesture at **moderate intensity** (50-70% MVC)
4. Hold for 2 seconds
5. Return to rest
6. Wait for "NEXT" prompt
7. Repeat 5-10 times

**Important**: Be consistent! Same speed, same intensity each time.

### Step 4: Validation

System tests calibration accuracy:

1. Random gesture prompts appear
2. Perform the requested gesture
3. System verifies classification
4. Target: >90% accuracy

If accuracy is low:
- Recalibrate problematic gestures
- Check electrode placement
- Ensure consistent gesture execution

## Gesture Execution Guide

### Hand Open (Gesture 1)

```
Starting Position: Relaxed, fingers slightly curved

Execution:
1. Spread all fingers wide
2. Extend wrist slightly
3. Hold position firmly

Muscles: Extensor digitorum, extensor pollicis

Tip: Imagine pushing against a wall
```

### Hand Close (Gesture 2)

```
Starting Position: Relaxed, fingers slightly curved

Execution:
1. Curl all fingers into palm
2. Thumb wraps over fingers
3. Squeeze firmly

Muscles: Flexor digitorum, flexor pollicis

Tip: Imagine gripping a ball firmly
```

### Wrist Flexion (Gesture 3)

```
Starting Position: Neutral wrist

Execution:
1. Bend wrist toward palm
2. Keep fingers relaxed
3. Avoid rotating forearm

Muscles: Flexor carpi radialis/ulnaris

Tip: As if looking at watch on inner wrist
```

### Wrist Extension (Gesture 4)

```
Starting Position: Neutral wrist

Execution:
1. Bend wrist away from palm
2. Keep fingers relaxed
3. Avoid rotating forearm

Muscles: Extensor carpi radialis/ulnaris

Tip: As if pushing door open with back of hand
```

### Pinch (Gesture 5)

```
Starting Position: Fingers extended

Execution:
1. Touch thumb to index fingertip
2. Apply moderate pressure
3. Other fingers remain extended

Muscles: Flexor pollicis, first dorsal interosseous

Tip: As if picking up a pin
```

### Tripod (Gesture 6)

```
Starting Position: Fingers extended

Execution:
1. Touch thumb to index and middle
2. Form a stable tripod
3. Ring and pinky remain extended

Muscles: Combination of flexors/intrinsics

Tip: As if holding a pen for writing
```

## Advanced Calibration

### Co-Contraction Detection

For users with limited independent muscle control:

1. Calibrate sequential co-contractions
2. Example: Flex then extend = gesture switch
3. Adjust timing thresholds as needed

### Proportional Control Tuning

For smooth grip force control:

1. Record force at 25%, 50%, 75%, 100% MVC
2. Create linear or logarithmic mapping
3. Adjust dead zone (typically 5-15%)
4. Test with grip force sensor

### Fatigue Compensation

EMG signals decrease with fatigue:

1. Enable adaptive thresholds
2. Recalibrate every 30-60 minutes of use
3. Or use auto-calibration feature

## Troubleshooting Calibration

### Problem: Gestures confused with each other

**Cause**: Similar muscle activation patterns

**Solutions**:
1. Use more distinct gestures
2. Adjust electrode positions
3. Add more training samples
4. Try different classifier (CNN vs LDA)

### Problem: False activations at rest

**Cause**: Threshold too low or noisy signal

**Solutions**:
1. Increase noise floor threshold
2. Check electrode contact
3. Move away from electrical interference
4. Add shielding to cables

### Problem: Gestures not detected

**Cause**: Threshold too high or weak signal

**Solutions**:
1. Lower detection threshold
2. Increase EMG gain
3. Check electrode placement
4. Perform MVC recalibration

### Problem: Delayed response

**Cause**: Processing latency too high

**Solutions**:
1. Reduce window size
2. Decrease overlap percentage
3. Use simpler classifier
4. Check for buffer issues

## Daily Calibration Routine

### Quick Start (2 minutes)

1. Apply electrodes
2. Wait 1 minute for stabilization
3. Perform 3 reps of each gesture
4. Verify with quick test
5. Begin use

### Full Recalibration (10 minutes)

Perform weekly or when:
- Accuracy drops below 85%
- Changed electrode positions
- Significant weather change (humidity)
- After illness or injury

## Saving and Loading Profiles

### Save Calibration

```
Command: save_cal <profile_name>
Example: save_cal morning_routine
```

### Load Calibration

```
Command: load_cal <profile_name>
Example: load_cal morning_routine
```

### Profile Management

- Keep 2-3 profiles for different conditions
- Morning vs. evening (fatigue levels)
- With vs. without prosthetic socket
- Different electrode sets

## Performance Metrics

After calibration, review:

| Metric | Target | Action if Below |
|--------|--------|-----------------|
| Rest Accuracy | >99% | Increase rest samples |
| Gesture Accuracy | >90% | More training data |
| Confusion Rate | <5% | Distinguish similar gestures |
| Response Time | <200ms | Reduce processing |
| False Positive Rate | <2% | Raise thresholds |

## Tips for Best Results

1. **Consistency is key** - Same electrode placement daily
2. **Mark electrode positions** - Use permanent marker dots
3. **Practice offline** - Train muscles before wearing prosthetic
4. **Start simple** - Master 2-3 gestures before adding more
5. **Rest breaks** - Prevent fatigue-induced errors
6. **Keep electrodes fresh** - Replace gel electrodes daily

---

*Calibration transforms raw signals into reliable control.
Take time to calibrate well - it pays off in everyday use.*
