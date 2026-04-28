# WIA-interstellar-communication PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-interstellar-communication
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-interstellar-communication. The standard covers the
persistent record shapes for radio and optical observation
campaigns that listen for putative interstellar artificial
signals, for probe-to-Earth telemetry from precursor
interstellar probes (Voyager-class spacecraft now in the
heliosheath and beyond, future small-probe missions), and
for any deliberate transmissions that the operator's
governance permits. The format is consumed by radio-astronomy
observatories conducting SETI-style surveys, by the operators
of precursor probes, by the IAU-affiliated post-detection
review committees, and by the regulators that license the
operator's spectrum and hardware.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID)
- ISO/IEC 27001:2022 (information security management)
- ISO 19115-1 (geographic information — metadata; cited as
  the metadata pattern for sky-pointing observations)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- ITU-R RA.769 (protection criteria for radio astronomy
  measurements)
- ITU-R RA.1513 (loss of the data due to interference for
  radio-astronomical observations)
- ITU-R RR Article 29 (radio astronomy service)
- IAU Working Group on Cartographic Coordinates and Rotational
  Elements report
- IAU Standards of Fundamental Astronomy (SOFA)
- IVOA (International Virtual Observatory Alliance) standards
  — VOTable, ObsCore, ObsTAP, UCD1+ (cited as the canonical
  metadata vocabulary for observation records)
- IAU Resolution B1 (XXVII GA, 2009) on radio frequency
  protection (cited as a reference policy that operators
  recognise in observation campaigns)
- CCSDS 301.0-B (Time Code Formats; cited for probe-to-Earth
  link records)

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts a
WIA-interstellar-communication operator manages.
Implementations covered include:

- Radio-astronomy SETI-style search programmes operating at
  L-band (1-2 GHz), S-band (2-4 GHz), C-band (4-8 GHz),
  X-band (8-12 GHz), Ka-band (26-40 GHz), and the broader
  bands that protective regulations permit.
- Optical SETI search programmes operating at visible and
  near-infrared wavelengths.
- Precursor interstellar probe operators (Voyager 1/2 in
  the interstellar medium, hypothetical near-future small
  probes).
- Post-detection review platforms that adjudicate candidate-
  signal claims under the IAA SETI Permanent Committee's
  protocols.
- Deliberate-transmission governance reviews that adjudicate
  any operator-proposed active SETI ("METI") transmissions.

Crewed interstellar travel and faster-than-light speculative
communications are out of scope; this PHASE addresses the
records of presently-feasible search and probe-telemetry
programmes.

## §2 Programme Identifier

```
programmeId         : string (uuidv7)
programmeOperator   : string (institutional identifier of the
                        observatory, university group, or
                        agency operating the programme)
programmeRegistered : string (ISO 8601 / RFC 3339)
programmeKind       : enum ("radio-search-passive" |
                        "optical-search-passive" |
                        "precursor-probe-link" |
                        "deliberate-transmission" |
                        "candidate-followup")
spectrumLicensingRef : string (operator's spectrum licence
                        reference under the operating
                        jurisdiction's radio regulations)
postDetectionAffiliationRef : string (operator's affiliation
                        with the IAA SETI Permanent
                        Committee post-detection-protocol
                        community, where the operator
                        adheres to the protocol)
programmeStatus     : enum ("design" | "operating" |
                        "suspended" | "concluded" |
                        "archived")
```

## §3 Sky Pointing and Target Catalogue Record

```
target:
  targetId            : string (uuidv7)
  programmeId         : string (uuidv7)
  designationKind     : enum ("hd-catalog" | "hipparcos" |
                          "gaia-dr3" | "tic-tess" |
                          "kic-kepler" | "exoplanet-host" |
                          "user-defined-pointing")
  designation         : string (catalogue identifier)
  rightAscensionDeg   : number (J2000 equatorial right
                          ascension, decimal degrees)
  declinationDeg      : number (J2000 declination)
  distanceParsec      : number (best-known distance, parsecs)
  spectralType        : string (Morgan-Keenan spectral type
                          where known)
  selectionRationale  : string (why the target is on the
                          list — e.g. "G-type within 100 pc",
                          "TESS exoplanet host", "anomalous
                          flux variability")
```

## §4 Observation Record

Observation records follow IVOA ObsCore semantics so that
WIA-native observations interoperate with the broader
astronomy data ecosystem.

```
observation:
  observationId       : string (uuidv7)
  programmeId         : string (uuidv7)
  targetRef           : string (target UUID)
  startEpoch          : string (ISO 8601 / RFC 3339)
  endEpoch            : string (ISO 8601)
  facilityRef         : string (observing facility identifier;
                          e.g. "GBT", "FAST", "VLA", "ATA",
                          "MeerKAT", "ALLEN-TELESCOPE-ARRAY",
                          "JWST" for optical SETI piggyback)
  instrumentRef       : string (instrument identifier on the
                          facility)
  bandKind            : enum ("L-band" | "S-band" | "C-band"
                          | "X-band" | "Ku-band" | "Ka-band"
                          | "optical-visible" |
                          "optical-nir" | "user-defined")
  centreFrequencyHz   : number (Hz; for radio observations)
  bandwidthHz         : number (Hz; for radio observations)
  centreWavelengthNm  : number (nm; for optical observations;
                          absent for radio)
  integrationTimeS    : number (seconds)
  pointingMode        : enum ("on-source-only" |
                          "on-off-nodding" |
                          "drift-scan" |
                          "raster-mosaic" |
                          "transit-monitoring")
  observationArtefactRef: string (content-addressed URI of
                          the observation data archive)
  observationDigest   : string (SHA-256)
```

## §5 Candidate-Signal Record

```
candidate:
  candidateId         : string (uuidv7)
  observationId       : string (uuidv7)
  detectedAt          : string (ISO 8601)
  pipelineRef         : string (signal-detection pipeline
                          identifier and version, e.g.
                          "TURBO_SETI v2.3", "BL-SCRUNCH v1.4")
  signalKind          : enum ("narrow-band-drift" |
                          "wide-band-broadband" |
                          "pulsed-modulation" |
                          "anomalous-spectral-line" |
                          "optical-nanosecond-flash" |
                          "user-defined")
  centreFrequencyHz   : number
  driftRateHzPerS     : number (frequency drift, Hz/s; absent
                          for non-drifting candidates)
  significance        : number (operator's pipeline-specific
                          significance metric; the metric
                          definition is recorded against the
                          pipeline)
  reproducibilityState: enum ("unverified" |
                          "reproduced-on-source" |
                          "rfi-classified" |
                          "natural-source-classified" |
                          "follow-up-pending" |
                          "post-detection-escalated")
  postDetectionTokenRef: string (URI of the post-detection
                          escalation record; absent unless
                          escalated)
```

The reproducibility-state lifecycle is governed by the
post-detection protocol described in PHASE-3 §5.

## §6 Radio-Frequency-Interference (RFI) Catalogue Record

Most candidates that survive initial screening turn out to be
RFI; the RFI catalogue tracks known interference sources so
that downstream reproducibility checks can rule them out
quickly.

```
rfiEntry:
  rfiId               : string (uuidv7)
  programmeId         : string (uuidv7)
  catalogedAt         : string (ISO 8601)
  centreFrequencyHz   : number
  sourceKind          : enum ("local-microwave" |
                          "mobile-handset" |
                          "satellite-downlink" |
                          "weather-radar" |
                          "aircraft-transponder" |
                          "wifi-bluetooth" |
                          "facility-internal" |
                          "user-defined")
  citationRef         : string (URI of the operator's RFI
                          investigation report)
```

## §7 Probe-Telemetry Link Record (Precursor Probes)

```
probeTelemetryLink:
  linkId              : string (uuidv7)
  programmeId         : string (uuidv7)
  probeRef            : string (probe identifier; e.g.
                          "VOYAGER-1", "VOYAGER-2", "PIONEER-10",
                          "NEW-HORIZONS-EXTENDED",
                          "BREAKTHROUGH-STARSHOT-N")
  trackingFacilityRef : string (deep-space tracking facility
                          carrying the link)
  startEpoch          : string (ISO 8601 / CCSDS 301.0-B)
  endEpoch            : string (ISO 8601)
  oneWayLightTimeS    : number (light-time at window midpoint,
                          seconds)
  bitRateBitsPerS     : number
  artefactRef         : string (URI of the captured
                          telemetry archive)
```

## §8 Deliberate-Transmission Record

Operators that propose any deliberate transmission ("active
SETI", "METI", or precursor-probe uplink commands) carry a
deliberate-transmission record per transmission.

```
deliberateTransmission:
  transmissionId      : string (uuidv7)
  programmeId         : string (uuidv7)
  proposedAt          : string (ISO 8601)
  governanceReviewRef : string (URI of the governance review
                          that adjudicated the proposal;
                          PHASE-3 §6)
  approvalState       : enum ("proposed" |
                          "under-governance-review" |
                          "approved" | "rejected" |
                          "withdrawn" | "transmitted")
  transmittedAt       : string (ISO 8601; absent until
                          transmission)
  payloadRef          : string (URI of the transmission
                          payload archive)
  targetRef           : string (target UUID)
  transmitterRef      : string (transmitter facility
                          identifier and the operator's
                          declared EIRP)
```

## §9 Synthetic-Signal Injection Test Record

Pipeline qualification (PHASE-3 §7) requires synthetic-signal
injection tests so that the pipeline's recovery rate, false-
positive rate, and per-significance-bin behaviour are
measured against a known truth set.

```
syntheticInjection:
  injectionId         : string (uuidv7)
  programmeId         : string (uuidv7)
  pipelineRef         : string (signal-detection pipeline
                          identifier and version)
  injectedAt          : string (ISO 8601)
  signalKind          : enum (matches §5 signalKind)
  injectedFrequencyHz : number
  injectedSignificance: number (the ground-truth significance
                          per the operator's pipeline metric
                          definition)
  recovered           : boolean (true when the pipeline
                          recovered the injected signal)
  recoveredCandidateRef : string (URI of the candidate that
                          the pipeline emitted in response;
                          absent when not recovered)
  recoveryDelta       : number (frequency / significance
                          difference between injected truth
                          and recovered candidate)
```

## §10 Observation Campaign Record

Multi-observation campaigns (decade-scale surveys of a
nearby-star catalog, multi-band coordinated campaigns)
carry a campaign record that aggregates the constituent
observations.

```
campaign:
  campaignId          : string (uuidv7)
  programmeId         : string (uuidv7)
  campaignName        : string (operator's display name)
  startedAt           : string (ISO 8601)
  endedAt             : string (ISO 8601; absent until ended)
  scientificRationale : string (one-paragraph rationale;
                          rendered in the public catalogue)
  observationCount    : integer (number of observations
                          composing the campaign)
  candidateCount      : integer (number of candidates
                          emitted across the campaign)
  postDetectionEscalationCount : integer (number of
                          post-detection escalations across
                          the campaign; typically 0)
```

## §11 Equipment-Calibration Record

Receiver calibration is the bedrock of candidate
significance: a poorly-calibrated receiver attributes thermal
or instrument noise to apparent signal. The calibration
record documents the receiver chain's measured behaviour at
the time of each observation campaign.

```
calibration:
  calibrationId       : string (uuidv7)
  facilityRef         : string (facility identifier)
  receiverChainRef    : string (operator identifier of the
                          calibrated receiver chain)
  capturedAt          : string (ISO 8601)
  systemTemperatureK  : number (system temperature in kelvin)
  bandwidthHz         : number (calibrated bandwidth)
  gainDb              : number (calibrated gain)
  flatnessHz          : number (passband flatness FWHM)
  calibrationArtefactRef: string (URI of the calibration
                          dataset)
  validUntil          : string (ISO 8601; cadence at which
                          re-calibration is required)
```

## §12 Conformance

Implementations claiming PHASE-1 conformance emit each of
the records defined above for every observation, candidate,
and (where applicable) probe-link or deliberate-transmission,
and honour the post-detection protocol's reproducibility-
state semantics in §5.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-interstellar-communication
- **Last Updated:** 2026-04-28
