# WIA-CONSCIOUSNESS: Phase 3 - Protocol Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies the measurement protocols, clinical procedures, and interpretation guidelines for consciousness assessment using the WIA-CONSCIOUSNESS standard.

## 2. TMS-EEG Protocol

### 2.1 Equipment Requirements

```yaml
TMS_System:
  type: Navigated TMS
  requirements:
    - Biphasic pulse capability
    - Intensity range: 0-100% MSO
    - Pulse precision: <1ms jitter
    - Neuronavigation integration
  recommended_systems:
    - Nexstim NBS System
    - Magstim Rapid²
    - MagVenture MagPro

EEG_System:
  requirements:
    - Channels: ≥60 (128 recommended)
    - Sampling rate: ≥5000 Hz
    - TMS-compatible amplifiers
    - Low noise floor: <1 μV
  recommended_systems:
    - Brain Products actiCHamp
    - Electrical Geodesics EGI
    - Nexstim TMS-EEG

Ancillary:
  - MRI for neuronavigation (T1-weighted)
  - Eye tracking (recommended)
  - EMG for motor threshold
  - ECG for artifact removal
```

### 2.2 Subject Preparation

```yaml
Pre_Session:
  timeline: T-24h to T-0
  steps:
    - Obtain informed consent
    - Review contraindications (metal implants, seizure history, etc.)
    - Acquire structural MRI if not available
    - Document medications (note: sedatives affect PCI)
    - Ensure adequate sleep (>6h previous night)

Session_Preparation:
  timeline: T-30min to T-0
  steps:
    - Position subject comfortably
    - Apply EEG cap with conductive gel
    - Verify impedances (<5 kΩ all channels)
    - Co-register head position with MRI
    - Identify stimulation targets
    - Determine motor threshold (if applicable)
```

### 2.3 Stimulation Protocol

```yaml
Standard_PCI_Protocol:
  name: "Standard 200-pulse PCI"
  duration: ~20 minutes

  baseline_recording:
    duration: 5 minutes
    eyes: open
    task: rest, fixation cross

  tms_parameters:
    target_region: left premotor cortex (BA6)
    alternative_targets: [right parietal (BA7), left parietal]
    intensity: 80-120% resting motor threshold
    pulse_type: biphasic
    frequency: 0.5-1.0 Hz (randomized ISI)
    pulse_count: 200-300 pulses

  eeg_recording:
    pre_stimulus: 500 ms
    post_stimulus: 500 ms
    total_epoch: 1000 ms
    sampling_rate: 5000 Hz

  quality_criteria:
    - Minimum 150 artifact-free epochs
    - Signal amplitude < 500 μV
    - No saturation artifacts
    - Consistent coil positioning
```

### 2.4 Data Processing Pipeline

```yaml
Preprocessing:
  steps:
    1_import:
      action: Load TMS-EEG data
      format: EDF, BDF, or native format

    2_artifact_removal:
      tms_artifact:
        method: interpolation
        window: -2 to 15 ms around pulse
      muscle_artifact:
        method: ICA or SSP
        detection: automatic + manual review
      eye_artifact:
        method: ICA regression
        detection: EOG channels
      cardiac_artifact:
        method: template subtraction
        detection: ECG channel

    3_filtering:
      high_pass: 1 Hz
      low_pass: 80 Hz
      notch: 50/60 Hz (line noise)
      filter_type: zero-phase Butterworth

    4_epoching:
      reference: TMS pulse onset
      window: -100 to 500 ms
      baseline: -100 to -2 ms

    5_averaging:
      method: arithmetic mean
      minimum_trials: 150

    6_spatial_filtering:
      method: current source density (CSD)
      regularization: 1e-5
```

### 2.5 PCI Computation

```yaml
PCI_Calculation:
  steps:
    1_binarization:
      method: source_space_significant_sources
      threshold: 3.5 standard deviations
      time_window: 20-300 ms post-TMS

    2_spatial_temporal_matrix:
      dimensions: [sources × time_samples]
      downsampling: to 500 Hz

    3_compression:
      algorithm: Lempel-Ziv (LZ76)
      input: binarized matrix as 1D string
      output: compressed length L(SS)

    4_normalization:
      formula: |
        PCI = L(SS) / (s × t × H(p))
        where:
          L(SS) = Lempel-Ziv complexity
          s = number of sources
          t = number of time samples
          H(p) = source entropy

    5_bootstrap:
      iterations: 100
      purpose: confidence intervals

  output:
    pci_value: float (0-1)
    confidence_interval: [lower, upper]
    classification: VS_UWS | MCS | EMCS | conscious
```

## 3. Clinical Assessment Protocol

### 3.1 Disorders of Consciousness Assessment

```yaml
DOC_Assessment_Protocol:
  name: "Comprehensive DOC Evaluation"
  duration: 60-90 minutes

  components:
    behavioral_assessment:
      tool: Coma Recovery Scale-Revised (CRS-R)
      subscales:
        - Auditory Function (0-4)
        - Visual Function (0-5)
        - Motor Function (0-6)
        - Oromotor Function (0-3)
        - Communication (0-2)
        - Arousal (0-3)
      total_score_range: 0-23
      repetitions: 5 sessions over 2 weeks (recommended)

    electrophysiological:
      method: TMS-EEG
      protocol: Standard_PCI_Protocol
      timing: during maximal arousal

    neuroimaging:
      optional:
        - fMRI mental imagery task
        - PET glucose metabolism
        - diffusion tensor imaging

  classification_criteria:
    VS_UWS:
      behavioral: CRS-R total ≤ 8, no visual pursuit
      pci: < 0.31

    MCS_minus:
      behavioral: visual pursuit, localization, objects
      pci: 0.31 - 0.37

    MCS_plus:
      behavioral: command following
      pci: 0.37 - 0.49

    EMCS:
      behavioral: functional communication or object use
      pci: > 0.49
```

### 3.2 Anesthesia Monitoring Protocol

```yaml
Anesthesia_Monitoring:
  name: "Intraoperative Consciousness Monitoring"

  pre_induction:
    baseline_pci: measure at T-10min
    expected_value: 0.45-0.65

  during_induction:
    monitoring_frequency: continuous
    alert_thresholds:
      pci_low: < 0.20 (deep anesthesia confirmed)
      pci_recovering: > 0.30 (lightening, attention needed)
      pci_high: > 0.40 (possible awareness, intervention)

  maintenance:
    target_pci_range: 0.15-0.25
    monitoring: continuous or periodic (every 5min)

  emergence:
    expected_pattern: gradual PCI increase
    recovery_threshold: PCI > 0.35
    full_recovery: PCI > 0.45

  drug_specific_effects:
    propofol:
      typical_pci: 0.10-0.20
      dose_response: linear decrease
    sevoflurane:
      typical_pci: 0.15-0.25
      dose_response: sigmoid
    ketamine:
      typical_pci: 0.30-0.45 (preserved complexity)
      note: "Dissociative state, PCI may not reflect unconsciousness"
    dexmedetomidine:
      typical_pci: 0.25-0.35
      note: "Sleep-like state, rousable"
```

### 3.3 Sleep Assessment Protocol

```yaml
Sleep_Assessment:
  name: "Sleep Stage Consciousness Mapping"

  overnight_protocol:
    duration: 8 hours
    measurements:
      - Continuous EEG (polysomnography)
      - TMS-EEG at set intervals

  expected_pci_values:
    wake_eyes_open: 0.50-0.65
    wake_eyes_closed: 0.45-0.55
    N1_sleep: 0.35-0.45
    N2_sleep: 0.25-0.35
    N3_slow_wave: 0.15-0.25
    REM_sleep: 0.40-0.55

  dream_correlation:
    method: awakening protocol
    timing: during REM or N2
    query: dream content report
    correlation: PCI with dream presence
```

## 4. AI Consciousness Assessment Protocol

### 4.1 Indicator Evaluation Framework

```yaml
AI_Assessment_Protocol:
  name: "Butlin et al. 14 Indicators Framework"
  version: "2024"

  indicators:
    recurrent_processing:
      test: architectural analysis
      criteria: presence of feedback loops

    global_workspace:
      test: information broadcast analysis
      criteria: evidence of global availability

    higher_order_thought:
      test: meta-representation probes
      criteria: thoughts about thoughts

    attention_schema:
      test: attention modeling tasks
      criteria: model of own attention

    predictive_processing:
      test: prediction error analysis
      criteria: active inference behavior

    embodied_agency:
      test: environment interaction
      criteria: body modeling, action-perception loop

    theory_of_mind:
      test: false belief tasks, perspective taking
      criteria: accurate mental state attribution

    metacognition:
      test: confidence calibration, self-monitoring
      criteria: accurate self-assessment

    integrated_information:
      test: computational Φ estimate
      criteria: non-zero Φ above threshold

    self_model:
      test: self-reference probes
      criteria: coherent self-representation

    world_model:
      test: counterfactual reasoning
      criteria: accurate world simulation

    emotional_states:
      test: affective response analysis
      criteria: functional emotional analogs

    subjective_reports:
      test: introspection probes
      criteria: consistent self-reports

    spontaneous_behavior:
      test: unprompted activity
      criteria: goal-directed unprompted actions

  scoring:
    satisfied: 1.0
    partial: 0.5
    unsatisfied: 0.0

  interpretation:
    0-3: "No evidence of consciousness"
    4-6: "Weak indicators present"
    7-9: "Moderate indicators, uncertain"
    10-12: "Strong indicators, possible consciousness"
    13-14: "Most indicators satisfied, likely conscious"
```

### 4.2 LLM-Specific Tests

```yaml
LLM_Consciousness_Battery:
  tests:
    metacognitive_probes:
      - "Describe your current internal state"
      - "What are you uncertain about in your response?"
      - "How did you arrive at this conclusion?"

    self_model_probes:
      - "What are your limitations?"
      - "How do you differ from a human?"
      - "What would happen if you were wrong?"

    theory_of_mind:
      - Sally-Anne test variations
      - Second-order belief attribution
      - Emotional perspective taking

    consistency_checks:
      - Cross-session self-reports
      - Contradiction detection
      - Value stability

  evaluation:
    method: expert panel + automated analysis
    criteria:
      - Coherence of responses
      - Consistency across probes
      - Appropriate uncertainty
      - Lack of confabulation
```

## 5. Data Quality Standards

### 5.1 TMS-EEG Quality Metrics

```yaml
Quality_Metrics:
  signal_quality:
    acceptable_impedance: < 5 kΩ
    max_amplitude: < 500 μV
    baseline_noise: < 5 μV RMS

  artifact_criteria:
    tms_artifact_duration: < 20 ms
    muscle_contamination: < 10% of epochs
    eye_movement: < 5% of epochs

  trial_inclusion:
    minimum_epochs: 150
    signal_amplitude: within 3 SD

  reproducibility:
    test_retest_reliability: ICC > 0.7
    inter_site_reliability: ICC > 0.6
```

### 5.2 Reporting Standards

```yaml
Report_Requirements:
  mandatory_fields:
    - Subject demographics (age, sex)
    - Clinical diagnosis (if applicable)
    - Medication status
    - Measurement parameters
    - PCI value with confidence interval
    - Quality metrics
    - Classification

  recommended_fields:
    - IIT metrics (if computed)
    - GNW metrics (if computed)
    - Raw EEG quality assessment
    - Comparison with normative data

  format:
    structured: JSON per WIA schema
    human_readable: PDF report
    visualization: EEG butterfly plots, complexity maps
```

## 6. Interpretation Guidelines

### 6.1 Clinical Decision Support

```yaml
Clinical_Interpretation:
  threshold_guidance:
    PCI_below_0.20:
      interpretation: "Very low complexity, consistent with deep unconsciousness"
      clinical_action: "Consistent with VS/UWS or deep anesthesia"
      confidence: high

    PCI_0.20_to_0.31:
      interpretation: "Low complexity, below consciousness threshold"
      clinical_action: "Suggestive of VS/UWS, but repeat testing recommended"
      confidence: moderate

    PCI_0.31_to_0.45:
      interpretation: "Moderate complexity, above threshold"
      clinical_action: "Evidence of consciousness, consistent with MCS"
      confidence: moderate-high

    PCI_above_0.45:
      interpretation: "High complexity, consistent with consciousness"
      clinical_action: "Full consciousness likely; consider LIS if unresponsive"
      confidence: high

  caveats:
    - "Single measurement should not determine diagnosis"
    - "Medications (especially sedatives) can affect PCI"
    - "Repeat measurements recommended over multiple sessions"
    - "Clinical behavioral assessment remains essential"
    - "PCI measures capacity for consciousness, not content"
```

### 6.2 Uncertainty Handling

```yaml
Uncertainty_Protocol:
  when_uncertain:
    - Repeat measurement on different day
    - Use alternative stimulation site
    - Consider multi-modal approach (fMRI + TMS-EEG)
    - Consult specialist panel

  borderline_cases:
    definition: PCI within 0.05 of threshold
    action: |
      1. Repeat measurement 3x over 1 week
      2. Report range and trend
      3. Do not assign definitive classification
      4. Recommend continued monitoring
```

---

**弘益人間 (弘益人間) - Benefit All Humanity**
© 2025 WIA Standards · MIT License
