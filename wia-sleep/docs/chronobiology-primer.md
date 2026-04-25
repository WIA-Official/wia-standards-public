# Chronobiology Primer for WIA-SLEEP

## Understanding the Science Behind the Standard

---

## 1. What is Chronobiology?

Chronobiology is the study of biological rhythms and their effects on physiology and behavior. The most well-known rhythm is the **circadian rhythm** - an approximately 24-hour cycle that regulates sleep-wake patterns, hormone release, body temperature, and many other biological processes.

---

## 2. The Circadian System

### 2.1 The Master Clock

The **suprachiasmatic nucleus (SCN)** in the hypothalamus serves as the body's master clock. It:

- Contains ~20,000 neurons with autonomous rhythmicity
- Synchronizes peripheral clocks throughout the body
- Receives light input directly from the retina
- Has an intrinsic period (tau) of ~24.2 hours in humans

### 2.2 Entrainment

**Entrainment** is the process by which the circadian clock synchronizes to external time cues (zeitgebers):

| Zeitgeber | Strength | Notes |
|-----------|----------|-------|
| Light | Strongest | Primary synchronizer |
| Feeding | Moderate | Affects peripheral clocks |
| Physical activity | Moderate | Exercise timing matters |
| Social cues | Weak-Moderate | Social rhythms |
| Temperature | Weak | Environmental temperature |

### 2.3 Light and the Circadian System

Light enters through specialized retinal cells called **intrinsically photosensitive retinal ganglion cells (ipRGCs)** containing **melanopsin**:

- Peak sensitivity: ~480 nm (blue light)
- Projects directly to SCN via retinohypothalamic tract
- Suppresses melatonin production
- Shifts circadian phase depending on timing

---

## 3. Key Circadian Markers

### 3.1 Dim Light Melatonin Onset (DLMO)

DLMO is the gold standard marker for circadian phase:

- Marks the beginning of the biological night
- Occurs ~2-3 hours before habitual sleep onset
- Measured in dim light (<10 lux) to prevent suppression
- Typical range: 7 PM - 11 PM in healthy adults

### 3.2 Core Body Temperature (CBT)

CBT follows a predictable daily rhythm:

```
6 PM  ████████████████████████  37.2°C (Peak)
      ↓
12 AM ████████████████          36.8°C
      ↓
4 AM  ████████                  36.2°C (Nadir)
      ↓
8 AM  ████████████              36.6°C
      ↓
12 PM ████████████████          36.9°C
```

- **CBT nadir** occurs ~7 hours after DLMO
- Optimal sleep onset: 2-3 hours after DLMO (descending temperature)
- Wake optimal: ascending temperature phase

### 3.3 Cortisol Rhythm

Cortisol follows a characteristic pattern:

- **Cortisol Awakening Response (CAR)**: 50-75% increase in first 30-45 minutes after waking
- Peak: 30-45 minutes after waking
- Nadir: Around midnight
- CAR blunted in depression, chronic stress

---

## 4. Chronotypes

### 4.1 Definition

**Chronotype** refers to individual differences in the timing of circadian rhythms and sleep-wake preferences. It has:

- **Genetic basis**: ~50% heritable (PER3, CLOCK genes)
- **Age-related changes**: Delays in adolescence, advances in older age
- **Environmental modulation**: Light exposure patterns

### 4.2 Munich Chronotype Questionnaire (MCTQ)

The MCTQ assesses chronotype through:

1. **Sleep timing on work days** (socially constrained)
2. **Sleep timing on free days** (closer to biological preference)
3. **MSFsc**: Corrected mid-sleep on free days = chronotype marker

### 4.3 Chronotype Spectrum

```
     ◄─────────────────────────────────────────────────────►
     Extreme     Moderate     Intermediate     Moderate     Extreme
     Early       Early                         Late         Late
     ("Lark")                                               ("Owl")

     DLMO:      DLMO:        DLMO:            DLMO:        DLMO:
     ~19:00     ~20:00       ~21:00           ~22:00       ~23:00+
```

---

## 5. Social Jetlag

### 5.1 Definition

**Social jetlag** is the discrepancy between biological and social clocks:

```
Social Jetlag = |MSF - MSW|
```

Where:
- MSF = Mid-sleep on free days
- MSW = Mid-sleep on work days

### 5.2 Health Implications

Social jetlag >2 hours is associated with:

- Metabolic dysfunction
- Obesity risk
- Cardiovascular issues
- Mood disturbances
- Cognitive impairment

---

## 6. Phase Response Curves (PRCs)

### 6.1 Light PRC

The timing of light exposure determines the direction of phase shift:

```
                    Delay              Advance
                      ↓                  ↓
Phase     |    +     |         |    -     |
Shift     |   ███    |         |   ███    |
(hours)   |  █████   |         |  █████   |
          ├──█████───┼─────────┼──█████───┤
          |         CBT               |
          |        Nadir              |
          -12h      0      +6h      +12h
                Relative to CBT Nadir
```

- **Delay zone**: Light before CBT nadir → delays clock
- **Advance zone**: Light after CBT nadir → advances clock
- **Dead zone**: Mid-day light has minimal effect

### 6.2 Melatonin PRC

Melatonin has an opposite phase response:

- Before DLMO: Advances phase
- After DLMO: Delays phase (rarely used clinically)

---

## 7. Sleep Architecture

### 7.1 Sleep Stages

| Stage | EEG Characteristics | Function |
|-------|---------------------|----------|
| Wake | Alpha, Beta | Consciousness |
| N1 | Theta, vertex waves | Light transition |
| N2 | Sleep spindles, K-complexes | Memory consolidation |
| N3/SWS | Delta (slow waves) | Physical restoration |
| REM | Mixed, REMs, low EMG | Emotional processing, memory |

### 7.2 Sleep Cycles

```
Hour: 0    1    2    3    4    5    6    7    8
      ├────┼────┼────┼────┼────┼────┼────┼────┤
Wake  |                                        |
REM   |    ░░   ▒▒▒   ███   ████  █████ |
N1    |░░                                      |
N2    |████ ██ ████ ████ ████ ████     |
N3    | ████ ████ ██   █                 |
      └────────────────────────────────────────┘
        Cycle 1  Cycle 2  Cycle 3  Cycle 4  Cycle 5
        ~90 min each
```

- Early night: More N3 (deep sleep)
- Late night: More REM
- Cycles: Typically 4-6 per night

### 7.3 Circadian Influence on Sleep

- **Sleep propensity gate**: Opens ~2h after DLMO
- **Wake maintenance zone**: 2-4h before DLMO (hard to fall asleep)
- **REM regulation**: Circadian-driven, peaks in early morning
- **SWS regulation**: Homeostatic (sleep pressure driven)

---

## 8. The Two-Process Model

### 8.1 Process S (Homeostatic)

Sleep pressure accumulates during wakefulness:

- Builds up exponentially while awake
- Dissipates during sleep
- Mediated by adenosine
- "Sleep debt" concept

### 8.2 Process C (Circadian)

Circadian alertness rhythm:

- Promotes wakefulness during biological day
- Allows sleep during biological night
- Creates "forbidden zones" and "sleep gates"

### 8.3 Interaction

```
Alertness = Process C - Process S

High alertness when:
- Circadian drive high (afternoon)
- Sleep pressure low (early in day)

Sleepiness when:
- Circadian drive low (night)
- Sleep pressure high (extended waking)
```

---

## 9. Clinical Applications

### 9.1 Circadian Rhythm Sleep Disorders

| Disorder | Characteristic | Treatment |
|----------|---------------|-----------|
| DSWPD | Delayed phase | Morning light, evening melatonin |
| ASWPD | Advanced phase | Evening light, morning darkness |
| Non-24 | Free-running | Timed melatonin, light |
| Shift work | Social misalignment | Strategic light, scheduled sleep |
| Jet lag | Rapid timezone change | Timed light/dark, melatonin |

### 9.2 Light Therapy

Effective for:
- Phase disorders
- Seasonal affective disorder
- Non-seasonal depression
- Shift work optimization

Parameters:
- Intensity: 2,500-10,000 lux
- Duration: 30-60 minutes
- Timing: Relative to CBT nadir
- Spectrum: Blue-enriched more effective

---

## 10. WIA-SLEEP Applications

### 10.1 Chronotype Assessment

WIA-SLEEP uses MCTQ methodology to:
1. Calculate MSFsc (chronotype marker)
2. Classify into five categories
3. Estimate DLMO from behavioral data
4. Calculate social jetlag

### 10.2 Circadian Phase Tracking

WIA-SLEEP tracks phase through:
1. Estimated DLMO (from chronotype or measurement)
2. Activity rhythms (actigraphy)
3. HRV patterns
4. Temperature data (if available)

### 10.3 Personalized Optimization

WIA-SLEEP generates recommendations for:
1. Optimal sleep window based on chronotype
2. Light exposure timing and intensity
3. Melatonin administration timing
4. Meal and exercise scheduling

---

## 11. Glossary

| Term | Definition |
|------|------------|
| Circadian | Approximately 24-hour rhythm |
| Chronotype | Individual circadian preference |
| DLMO | Dim Light Melatonin Onset |
| Entrainment | Synchronization to external cues |
| MSFsc | Corrected mid-sleep free days |
| PRC | Phase Response Curve |
| SCN | Suprachiasmatic Nucleus |
| Social jetlag | Bio-social clock mismatch |
| SWS | Slow Wave Sleep (N3) |
| Tau | Intrinsic circadian period |
| Zeitgeber | Time-giver (environmental cue) |

---

## 12. Further Reading

1. Roenneberg, T. (2012). *Internal Time: Chronotypes, Social Jet Lag, and Why You're So Tired*
2. Foster, R. & Kreitzman, L. (2017). *Circadian Rhythms: A Very Short Introduction*
3. Walker, M. (2017). *Why We Sleep*
4. AASM (2020). *AASM Scoring Manual Version 2.6*

---

**弘益人間 (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
