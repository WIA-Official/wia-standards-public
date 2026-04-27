# WIA-prosthetic-control PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-prosthetic-control
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for WIA-prosthetic-control.
The standard covers control of externally powered upper-limb and lower-limb
prostheses driven by user-generated control signals (surface or
intramuscular EMG, force-myography, inertial residual-limb motion, eye
tracking, peripheral-nerve signals, brain-computer-interface signals) and
the actuator commands and sensory-feedback streams that close the loop
back to the user. The format captures the signal-acquisition record, the
classifier or regressor that maps signals to intent, the motor commands
issued to the prosthetic actuators, the sensory feedback returned to the
user, the clinical fitting and calibration session, and the safety and
risk-management evidence that medical-device regulators require.

References (CITATION-POLICY ALLOW only):

- ISO 22523:2006 (external limb prostheses and external orthoses —
  requirements and test methods)
- ISO 13485:2016 (medical devices — quality management systems)
- ISO 14971:2019 (medical devices — application of risk management)
- ISO 14155:2020 (clinical investigation of medical devices)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- ISO 8601 (date and time)
- ISO 11073-10101 (medical device communication — nomenclature)
- IEC 60601-1:2005+AMD1:2012+AMD2:2020 (medical electrical equipment —
  general requirements for basic safety and essential performance)
- IEC 62366-1:2015+AMD1:2020 (usability engineering for medical devices)
- IEEE 11073-10406 (Personal Health Devices — basic ECG; cited only as a
  pattern reference for signal-encoding envelopes)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- HL7 FHIR R5 (DeviceUseStatement, Observation, ClinicalImpression)

---

## §1 Scope

This PHASE document defines the persistent shapes for the records that
flow during fitting, daily operation, clinical follow-up, and adverse-event
reporting of an externally powered prosthesis. Implementations covered
include:

- Signal-acquisition front-ends (EMG amplifiers, IMU clusters,
  peripheral-nerve interfaces, BCI front-ends).
- Intent-decoding software (pattern classifiers, regressors, posture
  controllers, finite-state machines).
- Prosthetic actuator firmware that consumes motor commands.
- Sensory-feedback drivers (vibrotactile arrays, transcutaneous
  electrical nerve stimulation, peripheral-nerve stimulation).
- Clinical-fitting workstations used by certified prosthetists.
- Post-market surveillance systems that aggregate device-use data.

Cosmetic prostheses without active control, passive orthoses, and
fully-implanted neurostimulators with no external limb interface are out
of scope.

## §2 Subject Identifier

Each fitted user is bound to a stable Subject Identifier (`subjectId`)
held in the operating clinic's electronic health record. The
DATA-FORMAT layer never carries the user's clinical identifiers; it
references the subject through an opaque token derived from the EHR
identifier.

```
subjectId         : string (uuidv7, opaque token; never a national ID)
deviceId          : string (uuidv7; serial number is held separately)
fitDate           : string (ISO 8601 date)
deviceClass       : enum  ("transradial" | "transhumeral" | "shoulder-disart"
                       | "transtibial" | "transfemoral" | "hip-disart" |
                       "partial-hand" | "partial-foot")
controlMode       : enum  ("direct-emg" | "pattern-recognition" |
                       "regression" | "fsm" | "tmr-emg" | "bci-eeg" |
                       "bci-ecog" | "peripheral-nerve")
indication        : enum  ("congenital" | "trauma" | "vascular-amputation" |
                       "oncological-amputation" | "infection-amputation")
```

The clinic EHR mediates between `subjectId` and the user's clinical
record per the binding rules in PHASE-4 §6.

## §3 Signal-Acquisition Record

```
acquisition:
  acquisitionId   : string (uuidv7)
  subjectId       : string (uuidv7)
  channels        : array of Channel
  sampleRateHz    : integer (channel-uniform; per-channel rates use the
                       extension below)
  resolutionBits  : integer
  filterChain     : array of FilterStage
  startedAt       : string (ISO 8601 / RFC 3339)
  durationS       : number
  artefactRef     : string (content-addressed URI of the raw archive,
                       encoded as EDF+ or HDF5 unless legacy)

Channel:
  channelId       : string
  modality        : enum ("semg" | "iemg" | "fmg" | "imu-accel" | "imu-gyro" |
                       "imu-mag" | "force" | "torque" | "pressure" |
                       "ecog" | "eeg" | "peripheral-nerve")
  electrodeMapRef : string (content-addressed URI; placement diagram per
                       ISO 11073-10101 nomenclature where applicable)
  inputImpedanceOhm: number (front-end input impedance)
  cmrrDb          : number (common-mode rejection ratio)
  notchFilters    : array of integer (Hz; e.g. [50, 60] for power-line
                       suppression by jurisdiction)
```

Acquisition records that span multiple sessions (a 30-day home-use
window, for example) MUST be split into per-session records of bounded
duration so that consent, calibration drift, and electrode-placement
revisions can be attributed unambiguously to the subset of data they
apply to.

## §4 Intent-Decoder Record

```
decoder:
  decoderId       : string (uuidv7)
  subjectId       : string (uuidv7)
  family          : enum ("lda" | "qda" | "svm" | "random-forest" |
                       "gradient-boosting" | "regression-mlp" |
                       "convnet" | "transformer" | "fsm" | "rule-based")
  inputs          : array of FeatureRef (references into acquisition
                       records or feature streams)
  outputs         : array of OutputDoF (degrees of freedom this decoder
                       drives; e.g. "wrist-flexion", "thumb-MCP-flexion")
  trainingSet     : TrainingSetRef (links to the labelled session set
                       used to train this decoder, see §6)
  validation:
    holdoutClassifAccuracyPct : number (cross-validated, optional)
    confusionMatrixRef        : string (URI; absent for regression
                                   decoders)
    nrmseRef                  : string (URI; present for regression)
    realtimeLatencyMs         : number (95th percentile end-to-end
                                   decode latency under bench test)
  modelArtefactRef: string (content-addressed URI of the model file,
                       in ONNX or a successor portable format)
  trainedAt       : string (ISO 8601 / RFC 3339)
  retrainPolicy   : enum ("manual" | "session-end-update" |
                       "incremental-online")
```

Decoders that update online during use (incremental adaptation) MUST
publish a continuous adaptation log so that downstream regulators can
audit the trajectory of the decoder over time.

## §5 Motor-Command Record

Motor commands describe the actuator setpoints the decoder issues to the
prosthesis. The format records the commanded position, velocity, and
torque per actuator and the timestamp at which each command was issued.

```
motorCommand:
  commandId       : string (uuidv7)
  acquisitionId   : string (uuidv7, source of intent)
  decoderId       : string (uuidv7, decoder that produced the command)
  issuedAt        : string (ISO 8601 / RFC 3339, sub-millisecond precision)
  setpoints       : array of ActuatorSetpoint
  safetyState     : enum ("normal" | "limit-engaged" | "fault-detected" |
                       "graceful-shutdown" | "user-stop")

ActuatorSetpoint:
  actuatorId      : string (joint or compound-actuator identifier;
                       "wrist", "thumb-MCP", "knee-flex", etc.)
  positionRad     : number (commanded position in radians; absent for
                       velocity- or torque-mode actuators)
  velocityRadPerS : number (commanded velocity)
  torqueNm        : number (commanded torque)
  modeOfControl   : enum ("position" | "velocity" | "torque" | "impedance")
```

Implementations MUST persist the motor-command stream at the highest
control-loop rate that the actuator implements; downsampling for storage
is permitted but the original sample rate MUST be recorded so that
downstream reproducibility analysis is correct.

## §6 Calibration Session Record

Calibration sessions are conducted by certified prosthetists at fitting,
follow-up, and after device repair. The session record carries the
prescribed protocol (e.g. "screen-guided four-direction wrist
calibration"), the trials run, and the operator's qualitative assessment.

```
calibrationSession:
  sessionId       : string (uuidv7)
  subjectId       : string (uuidv7)
  prosthetistId   : string (institutional identifier; PII held in clinic
                       HR system, not here)
  protocolRef     : string (content-addressed URI of the prescribed
                       protocol)
  trials          : array of CalibrationTrial
  outcome         : enum ("accepted" | "needs-rework" | "device-fault")
  notes           : string (clinical free text; redacted on export)
```

## §7 Sensory-Feedback Record

Sensory-feedback streams (vibrotactile arrays, electro-tactile arrays,
transcutaneous electrical nerve stimulation, peripheral-nerve
stimulation) close the loop back to the user. The record captures the
modality, the per-channel waveform parameters, and the safety limits
that the device enforces.

```
sensoryFeedback:
  feedbackId      : string (uuidv7)
  subjectId       : string (uuidv7)
  modality        : enum ("vibrotactile" | "electrotactile" | "tens" |
                       "peripheral-nerve-stim")
  channels        : array of FeedbackChannel
  perChannelLimits:
    maxAmplitude  : number (modality-specific units)
    maxDutyCyclePct: number
  startedAt       : string (ISO 8601)
  durationS       : number
```

Stimulation-based feedback channels (TENS, peripheral-nerve) MUST emit
charge-balanced waveforms; the format records the charge balance per
channel as a non-zero invariant that the implementation MUST satisfy
during operation.

## §8 Adverse-Event Record

```
adverseEvent:
  eventId         : string (uuidv7)
  subjectId       : string (uuidv7)
  occurredAt      : string (ISO 8601 / RFC 3339)
  category        : enum ("skin-irritation" | "muscle-fatigue" |
                       "joint-stress" | "fall" | "device-fault" |
                       "stimulation-overcurrent" | "thermal-event" |
                       "battery-fire-prevention-trigger")
  severity        : enum ("non-serious" | "serious" | "life-threatening")
  reportedToAuthority: string (regulator reference; opaque)
  recoveryNotes   : string (clinical free text; redacted on export)
```

Adverse events with `severity` of `serious` or `life-threatening` MUST
be reported to the relevant national medical-device authority within
the period the authority requires; the regulator-reference field
captures the report identifier returned by the authority.

## §9 Risk-Management Linkage

Every device fitting MUST reference the device's ISO 14971-aligned risk
file. The reference is a content-addressed URI of the risk file at the
manufacturer's quality system; the operating clinic does not own the
risk file but consumes it.

## §10 Activity-of-Daily-Living Outcome Record

Outcome measures captured during fitting and follow-up are recorded as
structured observations alongside the device-use stream so that
clinical effect can be tracked over time. The outcome catalogue
follows the operating clinic's choice of validated measures.

```
outcome:
  outcomeId       : string (uuidv7)
  subjectId       : string (uuidv7)
  capturedAt      : string (ISO 8601)
  instrument      : enum ("box-and-block" | "jebsen-taylor-hand" |
                       "amputee-mobility-predictor" |
                       "promis-pa" | "promis-fatigue" |
                       "trinity-amputation-prosthesis-experience" |
                       "six-minute-walk" | "timed-up-and-go" |
                       "user-defined")
  value           : number (instrument-specific units)
  trialConditions : object (e.g. with-prosthesis vs without; standing
                       vs seated; at fitting vs at week 8)
  rater           : string (clinician identifier; PII held in clinic
                       HR system)
```

Trial-cohort subjects emit outcome records at the schedule prescribed
by the investigation protocol; post-market subjects emit outcomes at
the cadence the clinic and the user agree on, with at least one
outcome capture per follow-up visit per active outcome instrument.

## §11 Device Configuration Snapshot

The device's loaded configuration at any point in time is captured
as a content-addressed snapshot bundle that includes the active
decoder model, the active feedback-channel limits, the firmware
version, and the calibration state.

```
configurationSnapshot:
  snapshotId      : string (uuidv7)
  subjectId       : string (uuidv7)
  capturedAt      : string (ISO 8601)
  decoderRef      : string (decoderId)
  feedbackRef     : string (feedbackId)
  firmwareVersion : string (Semantic Versioning 2.0.0)
  bootloaderVersion: string
  configurationHash: string (SHA-256 of the bundled configuration)
```

Snapshots are emitted at every clinic visit, after every firmware
update, and on user-initiated re-configuration; they are the recovery
anchor when a configuration causes degraded performance and the user
requests a rollback.

## §12 Conformance

Implementations claiming PHASE-1 conformance emit each record listed
above for every fitted subject and honour the immutability and
append-only constraints. Failure modes return Problem-Details (RFC 9457)
with `urn:wia:prosthetic-control` types defined in PHASE-2 §10.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-prosthetic-control
- **Last Updated:** 2026-04-27
