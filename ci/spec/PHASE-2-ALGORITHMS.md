# Phase 2: CI Algorithm Specification

## WIA-CI Octave Enhancement Algorithms

**Version**: 1.0.0
**Date**: 2025-12-16
**Status**: Draft

---

## 1. 개요

WIA-CI 알고리즘은 기존 CI 처리의 한계를 극복하고,
옥타브/피치 정보를 보존하여 음악 인식을 향상시킵니다.

### 1.1 기존 CI의 문제점

```
기존 CI 처리:
Audio → FFT → Envelope → Stimulation

문제점:
1. Temporal Fine Structure (TFS) 손실
2. 옥타브 정보 손실 (C4 vs C5 구분 불가)
3. 하모닉 구조 파괴
4. 음악/음성 구분 없는 단일 처리
```

### 1.2 WIA-CI 해결책

```
WIA-CI 처리:
Audio → [Octave Detection] → [TFS Encoding] → Enhanced Stimulation
              ↓
        하모닉 분석
        F0 추정
        옥타브 식별
```

---

## 2. Octave Detection Algorithm

### 2.1 F0 Estimation (기본 주파수 추정)

```typescript
interface F0Estimator {
  // 다중 알고리즘 앙상블
  algorithms: {
    yin: YINAlgorithm;           // 시간 영역
    pyin: PYINAlgorithm;         // 확률적 YIN
    swipe: SWIPEAlgorithm;       // 스펙트럼 기반
    crepe?: CREPEAlgorithm;      // 딥러닝 (선택)
  };

  // 앙상블 결합
  ensemble: {
    method: 'voting' | 'weighted' | 'confidence';
    weights?: number[];
  };
}

// YIN 알고리즘 구현
function yinEstimateF0(
  signal: Float32Array,
  sampleRate: number,
  options: YINOptions
): F0Result {
  const { minF0 = 50, maxF0 = 600, threshold = 0.1 } = options;

  const minPeriod = Math.floor(sampleRate / maxF0);
  const maxPeriod = Math.floor(sampleRate / minF0);

  // Step 1: Difference function
  const diff = computeDifference(signal, maxPeriod);

  // Step 2: Cumulative mean normalized difference
  const cmndf = computeCMNDF(diff);

  // Step 3: Absolute threshold
  let period = findPeriod(cmndf, minPeriod, maxPeriod, threshold);

  // Step 4: Parabolic interpolation
  period = parabolicInterpolation(cmndf, period);

  return {
    f0: sampleRate / period,
    confidence: 1 - cmndf[Math.round(period)],
    period: period
  };
}
```

### 2.2 Harmonic Analysis (하모닉 분석)

```typescript
interface HarmonicAnalyzer {
  // 하모닉 검출 설정
  config: {
    maxHarmonics: number;       // 최대 하모닉 수 (권장: 10)
    minAmplitude: number;       // 최소 진폭 (-60dB)
    toleranceHz: number;        // 주파수 허용 오차 (±5Hz)
  };
}

function analyzeHarmonics(
  spectrum: Float32Array,
  f0: number,
  sampleRate: number
): HarmonicInfo[] {
  const harmonics: HarmonicInfo[] = [];
  const binWidth = sampleRate / spectrum.length;

  for (let h = 1; h <= 10; h++) {
    const expectedFreq = f0 * h;
    if (expectedFreq > sampleRate / 2) break;

    const expectedBin = Math.round(expectedFreq / binWidth);

    // Peak search around expected bin
    const { peakBin, peakAmp } = findPeak(
      spectrum,
      expectedBin - 3,
      expectedBin + 3
    );

    const actualFreq = peakBin * binWidth;
    const deviation = Math.abs(actualFreq - expectedFreq);

    harmonics.push({
      harmonic: h,
      frequency: actualFreq,
      expectedFrequency: expectedFreq,
      amplitude: peakAmp,
      deviation: deviation,
      present: deviation < 5 && peakAmp > -60,
      phase: getPhase(spectrum, peakBin)
    });
  }

  return harmonics;
}
```

### 2.3 Octave Classification (옥타브 분류)

```typescript
interface OctaveClassifier {
  // 옥타브 경계 (A4 = 440Hz 기준)
  octaveBoundaries: number[];  // [27.5, 55, 110, 220, 440, 880, 1760, 3520]

  // 노트 매핑
  noteNames: string[];  // ['C', 'C#', 'D', ...]
}

function classifyOctave(f0: number): OctaveInfo {
  // A0 = 27.5Hz 기준
  const A0 = 27.5;

  // 옥타브 계산: O = log2(f / A0)
  const octaveFloat = Math.log2(f0 / A0);
  const octaveNumber = Math.floor(octaveFloat);

  // 노트 계산 (A = 0, A# = 1, ..., G# = 11)
  const noteFloat = (octaveFloat - octaveNumber) * 12;
  const noteNumber = Math.round(noteFloat) % 12;

  // A 기준을 C 기준으로 변환
  const noteFromC = (noteNumber + 3) % 12;
  const notes = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B'];

  // C 기준 옥타브 (A4 = 440Hz일 때 C4 = 261.63Hz)
  const cOctave = noteFromC < 3 ? octaveNumber : octaveNumber + 1;

  return {
    octaveNumber: cOctave,
    noteName: notes[noteFromC],
    noteNumber: noteFromC,
    cents: (noteFloat - Math.round(noteFloat)) * 100,
    midiNote: 12 * (octaveNumber + 1) + noteNumber + 21
  };
}
```

---

## 3. Temporal Fine Structure (TFS) Encoding

### 3.1 TFS 추출

```typescript
interface TFSExtractor {
  // 필터뱅크 설정
  filterBank: {
    type: 'gammatone' | 'butterworth';
    channels: number;           // 22
    minFreq: number;            // 250 Hz
    maxFreq: number;            // 8000 Hz
  };

  // TFS 추출 방식
  extraction: 'hilbert' | 'zero_crossing' | 'phase';
}

function extractTFS(
  signal: Float32Array,
  sampleRate: number,
  channel: number
): TFSResult {
  // 1. Gammatone 필터 적용
  const filtered = applyGammatone(signal, channel);

  // 2. Hilbert 변환
  const analytic = hilbertTransform(filtered);

  // 3. Envelope 추출
  const envelope = computeEnvelope(analytic);

  // 4. Fine Structure 추출 (순시 위상)
  const phase = computeInstantaneousPhase(analytic);
  const frequency = computeInstantaneousFrequency(phase, sampleRate);

  return {
    envelope: envelope,
    fineStructure: frequency,
    phase: phase,

    // TFS 통계
    tfsEnergy: computeTFSEnergy(frequency, envelope),
    dominantTFS: findDominantTFS(frequency)
  };
}

// Hilbert 변환
function hilbertTransform(signal: Float32Array): Complex[] {
  const N = signal.length;

  // FFT
  const spectrum = fft(signal);

  // Hilbert 마스크 적용
  // H(k) = 2 for k in [1, N/2-1]
  // H(k) = 1 for k = 0 or N/2
  // H(k) = 0 for k in [N/2+1, N-1]
  for (let k = 1; k < N / 2; k++) {
    spectrum[k].re *= 2;
    spectrum[k].im *= 2;
  }
  for (let k = Math.floor(N / 2) + 1; k < N; k++) {
    spectrum[k].re = 0;
    spectrum[k].im = 0;
  }

  // IFFT
  return ifft(spectrum);
}
```

### 3.2 TFS → Electrode 인코딩

```typescript
interface TFSEncoder {
  // 인코딩 전략
  strategy: 'phase_locking' | 'rate_place' | 'hybrid';

  // Phase Locking 파라미터
  phaseLocking: {
    maxRate: number;            // 최대 위상 고정 레이트 (500 Hz)
    channels: number[];         // 적용 채널 (저주파 채널)
  };
}

function encodeTFSToStimulation(
  tfs: TFSResult,
  electrode: number,
  strategy: TFSEncodingStrategy
): StimulationPattern {
  const { fineStructure, envelope } = tfs;

  if (strategy === 'phase_locking' && electrode > 15) {
    // 저주파 채널 (apex): Phase Locking 적용
    return encodeWithPhaseLocking(fineStructure, envelope, electrode);
  } else {
    // 고주파 채널 (base): Rate-Place 유지
    return encodeWithRatePlace(envelope, electrode);
  }
}

function encodeWithPhaseLocking(
  tfs: Float32Array,
  envelope: Float32Array,
  electrode: number
): StimulationPattern {
  const pulses: StimulationPulse[] = [];

  // Zero-crossing 기반 펄스 타이밍
  for (let i = 1; i < tfs.length; i++) {
    // Positive zero-crossing 검출
    if (tfs[i - 1] < 0 && tfs[i] >= 0) {
      // 보간으로 정확한 crossing 시점 계산
      const fraction = -tfs[i - 1] / (tfs[i] - tfs[i - 1]);
      const crossingTime = (i - 1 + fraction) / SAMPLE_RATE;

      // Envelope로 amplitude 결정
      const amp = interpolateEnvelope(envelope, i);

      pulses.push({
        time: crossingTime,
        electrode: electrode,
        amplitude: amp * MAX_AMPLITUDE,
        phase: 'biphasic'
      });
    }
  }

  return { pulses, electrode };
}
```

---

## 4. Envelope Extraction with Octave Preservation

### 4.1 Enhanced Envelope Extractor

```typescript
interface EnhancedEnvelopeExtractor {
  // 기본 Envelope 추출
  baseExtraction: {
    method: 'hilbert' | 'rectify_lowpass' | 'rms';
    smoothing: number;          // ms
  };

  // 옥타브 보존 확장
  octavePreservation: {
    enabled: boolean;
    modulationRate: number;     // Hz (권장: 4-16 Hz)
    depth: number;              // 0.0-1.0
  };
}

function extractEnhancedEnvelope(
  signal: Float32Array,
  f0: number,
  octaveInfo: OctaveInfo,
  config: EnhancedEnvelopeConfig
): EnhancedEnvelope {
  // 1. 기본 Envelope 추출
  const baseEnvelope = extractBaseEnvelope(signal, config.smoothing);

  // 2. 옥타브 변조 신호 생성
  const octaveModulation = generateOctaveModulation(
    signal.length,
    octaveInfo.octaveNumber,
    config.modulationRate,
    config.depth
  );

  // 3. Envelope에 옥타브 정보 임베딩
  const enhancedEnvelope = new Float32Array(signal.length);
  for (let i = 0; i < signal.length; i++) {
    enhancedEnvelope[i] = baseEnvelope[i] * (1 + octaveModulation[i]);
  }

  return {
    base: baseEnvelope,
    enhanced: enhancedEnvelope,
    octaveModulation: octaveModulation,
    f0: f0,
    octaveInfo: octaveInfo
  };
}

// 옥타브별 고유 변조 패턴 생성
function generateOctaveModulation(
  length: number,
  octaveNumber: number,
  rate: number,
  depth: number
): Float32Array {
  const modulation = new Float32Array(length);

  // 옥타브별 고유 변조 주파수
  // Octave 3: 4Hz, Octave 4: 6Hz, Octave 5: 8Hz, ...
  const modFreq = 4 + (octaveNumber - 3) * 2;

  // 옥타브별 고유 위상 오프셋
  const phaseOffset = (octaveNumber * Math.PI) / 4;

  for (let i = 0; i < length; i++) {
    const t = i / SAMPLE_RATE;
    modulation[i] = depth * Math.sin(2 * Math.PI * modFreq * t + phaseOffset);
  }

  return modulation;
}
```

---

## 5. Channel Selection (n-of-m)

### 5.1 Enhanced ACE Strategy

```typescript
interface EnhancedACE {
  // 기본 n-of-m 파라미터
  maxChannels: number;          // m = 22
  selectedChannels: number;     // n = 8-12

  // 옥타브 인식 선택
  octaveAware: {
    enabled: boolean;
    prioritizeHarmonics: boolean;
    preserveF0Channel: boolean;
  };
}

function selectChannelsWithOctave(
  envelopes: Float32Array[],    // 22채널
  harmonics: HarmonicInfo[],
  config: EnhancedACEConfig
): number[] {
  const scores: { channel: number; score: number }[] = [];

  for (let ch = 0; ch < 22; ch++) {
    let score = envelopes[ch].reduce((a, b) => a + b, 0);  // 기본: 에너지

    // 하모닉 채널 보너스
    if (config.prioritizeHarmonics) {
      for (const h of harmonics) {
        if (h.present && isChannelInRange(ch, h.frequency)) {
          score *= 1.5;  // 50% 보너스
          break;
        }
      }
    }

    // F0 채널 보너스
    if (config.preserveF0Channel) {
      const f0Channel = frequencyToChannel(harmonics[0]?.frequency || 0);
      if (ch === f0Channel) {
        score *= 2.0;  // 100% 보너스 (항상 선택되도록)
      }
    }

    scores.push({ channel: ch, score });
  }

  // 상위 n개 선택
  scores.sort((a, b) => b.score - a.score);
  return scores.slice(0, config.selectedChannels).map(s => s.channel);
}
```

### 5.2 Spectral Peak Preservation

```typescript
function preserveSpectralPeaks(
  spectrum: Float32Array,
  selectedChannels: number[],
  minPeaks: number
): number[] {
  // 스펙트럼 피크 검출
  const peaks = findSpectralPeaks(spectrum);

  // 피크 채널이 선택에 포함되어 있는지 확인
  const peakChannels = peaks.map(p => frequencyToChannel(p.frequency));

  let selected = [...selectedChannels];

  for (const peakCh of peakChannels) {
    if (!selected.includes(peakCh) && selected.length < 22) {
      // 가장 낮은 스코어 채널 교체
      selected = selected.slice(0, -1);
      selected.push(peakCh);
    }
  }

  return selected;
}
```

---

## 6. Stimulation Pattern Generation

### 6.1 Octave-Modulated Stimulation

```typescript
interface OctaveModulatedStimulation {
  // 자극 기본 파라미터
  pulseRate: number;            // pps (pulses per second)
  pulseWidth: number;           // μs

  // 옥타브 변조
  octaveModulation: {
    type: 'amplitude' | 'rate' | 'timing' | 'combined';
    pattern: OctavePattern;
  };
}

function generateStimulationPattern(
  envelope: EnhancedEnvelope,
  electrode: number,
  config: StimulationConfig
): StimulationSequence {
  const sequence: StimulationCommand[] = [];

  const pulsePeriod = 1000000 / config.pulseRate;  // μs

  for (let t = 0; t < envelope.enhanced.length; t += pulsePeriod) {
    const envValue = interpolate(envelope.enhanced, t);

    // 옥타브 기반 변조 적용
    let amplitude = envValue * config.maxAmplitude;
    let timing = t;

    switch (config.octaveModulation.type) {
      case 'amplitude':
        // 옥타브에 따른 amplitude 변조
        amplitude *= getOctaveAmplitudeScale(envelope.octaveInfo);
        break;

      case 'timing':
        // 옥타브에 따른 미세 타이밍 조절
        timing += getOctaveTimingOffset(envelope.octaveInfo, t);
        break;

      case 'rate':
        // 옥타브에 따른 펄스 레이트 조절
        // (다음 펄스 시점 조절로 구현)
        break;

      case 'combined':
        amplitude *= getOctaveAmplitudeScale(envelope.octaveInfo);
        timing += getOctaveTimingOffset(envelope.octaveInfo, t);
        break;
    }

    sequence.push({
      timestamp: timing,
      electrode: electrode,
      amplitude: clamp(amplitude, 0, MAX_AMPLITUDE),
      pulseWidth: config.pulseWidth,
      interphaseGap: 8  // μs
    });
  }

  return { commands: sequence, electrode };
}
```

### 6.2 Interleaving Strategy

```typescript
// CIS 인터리빙 (동시 자극 방지)
function interleavePulses(
  patterns: StimulationSequence[]
): StimulationFrame[] {
  // 모든 펄스를 타임라인에 배치
  const allPulses: TimedPulse[] = [];

  for (const pattern of patterns) {
    for (const cmd of pattern.commands) {
      allPulses.push({
        time: cmd.timestamp,
        electrode: cmd.electrode,
        command: cmd
      });
    }
  }

  // 시간순 정렬
  allPulses.sort((a, b) => a.time - b.time);

  // 프레임 생성 (각 프레임 = 하나의 자극 사이클)
  const frames: StimulationFrame[] = [];
  const frameInterval = 1000000 / (22 * 900);  // ~50μs per electrode at 900pps

  let currentFrame: StimulationFrame = {
    timestamp: 0,
    stimulations: [],
    interleaved: true
  };

  for (const pulse of allPulses) {
    // 같은 시간에 동일 전극이 이미 있으면 다음 슬롯으로
    const conflicting = currentFrame.stimulations.some(
      s => s.electrode === pulse.electrode
    );

    if (conflicting || currentFrame.stimulations.length >= 1) {
      frames.push(currentFrame);
      currentFrame = {
        timestamp: pulse.time,
        stimulations: [],
        interleaved: true
      };
    }

    currentFrame.stimulations.push(pulse.command);
  }

  if (currentFrame.stimulations.length > 0) {
    frames.push(currentFrame);
  }

  return frames;
}
```

---

## 7. Music Mode Processing

### 7.1 Music Detection

```typescript
interface MusicDetector {
  // 분류 특성
  features: {
    spectralFlux: number;
    zeroCrossingRate: number;
    spectralCentroid: number;
    harmonicity: number;
  };

  // 분류기
  classifier: 'threshold' | 'svm' | 'neural';
}

function detectMusic(
  signal: Float32Array,
  sampleRate: number
): MusicDetectionResult {
  // 특성 추출
  const features = extractMusicFeatures(signal, sampleRate);

  // 간단한 규칙 기반 분류
  const isMusicScore =
    (features.harmonicity > 0.5 ? 0.3 : 0) +
    (features.spectralFlux < 0.3 ? 0.2 : 0) +
    (features.zeroCrossingRate < 0.1 ? 0.2 : 0) +
    (features.spectralCentroid > 500 && features.spectralCentroid < 2000 ? 0.3 : 0);

  return {
    isMusic: isMusicScore > 0.5,
    confidence: isMusicScore,
    features: features
  };
}
```

### 7.2 Music-Optimized Processing

```typescript
function processMusicMode(
  signal: Float32Array,
  config: MusicProcessingConfig
): ProcessedMusicFrame {
  // 1. F0 추정 (앙상블)
  const f0Result = estimateF0Ensemble(signal, config.sampleRate);

  // 2. 옥타브 분류
  const octaveInfo = classifyOctave(f0Result.f0);

  // 3. 하모닉 분석
  const spectrum = computeSpectrum(signal);
  const harmonics = analyzeHarmonics(spectrum, f0Result.f0, config.sampleRate);

  // 4. TFS 추출 (저주파 채널만)
  const tfsResults: TFSResult[] = [];
  for (let ch = 16; ch < 22; ch++) {  // 채널 17-22 (저주파)
    tfsResults.push(extractTFS(signal, config.sampleRate, ch));
  }

  // 5. 옥타브 보존 Envelope
  const enhancedEnvelopes: EnhancedEnvelope[] = [];
  for (let ch = 0; ch < 22; ch++) {
    const channelSignal = applyBandpass(signal, ch);
    enhancedEnvelopes.push(
      extractEnhancedEnvelope(channelSignal, f0Result.f0, octaveInfo, config)
    );
  }

  // 6. 채널 선택 (하모닉 인식)
  const selectedChannels = selectChannelsWithOctave(
    enhancedEnvelopes.map(e => e.enhanced),
    harmonics,
    config.aceConfig
  );

  // 7. 자극 패턴 생성
  const stimPatterns: StimulationSequence[] = [];
  for (const ch of selectedChannels) {
    const pattern = generateStimulationPattern(
      enhancedEnvelopes[ch],
      ch + 1,  // electrode 1-22
      config.stimConfig
    );
    stimPatterns.push(pattern);
  }

  // 8. TFS 인코딩 추가 (저주파 채널)
  for (let i = 0; i < tfsResults.length; i++) {
    const ch = 16 + i;  // 채널 17-22
    if (selectedChannels.includes(ch)) {
      const tfsPattern = encodeTFSToStimulation(
        tfsResults[i],
        ch + 1,
        'phase_locking'
      );
      // 기존 패턴과 병합
      stimPatterns.find(p => p.electrode === ch + 1)!.commands.push(
        ...tfsPattern.pulses.map(p => ({
          timestamp: p.time * 1000000,
          electrode: p.electrode,
          amplitude: p.amplitude,
          pulseWidth: 25,
          interphaseGap: 8
        }))
      );
    }
  }

  // 9. 인터리빙
  const frames = interleavePulses(stimPatterns);

  return {
    f0: f0Result,
    octaveInfo: octaveInfo,
    harmonics: harmonics,
    selectedChannels: selectedChannels,
    frames: frames
  };
}
```

---

## 8. Speech Mode Processing

### 8.1 Speech-Optimized (기존 호환)

```typescript
function processSpeechMode(
  signal: Float32Array,
  config: SpeechProcessingConfig
): ProcessedSpeechFrame {
  // 기존 CIS/ACE와 호환되는 처리

  // 1. 필터뱅크
  const channelSignals = applyFilterBank(signal, 22);

  // 2. Envelope 추출 (기본)
  const envelopes = channelSignals.map(ch =>
    extractBaseEnvelope(ch, config.smoothing)
  );

  // 3. 채널 선택 (에너지 기반)
  const selectedChannels = selectTopChannels(
    envelopes,
    config.selectedChannels
  );

  // 4. 자극 패턴 (기본)
  const stimPatterns = selectedChannels.map(ch =>
    generateBasicStimulation(envelopes[ch], ch + 1, config)
  );

  // 5. 인터리빙
  const frames = interleavePulses(stimPatterns);

  return {
    selectedChannels: selectedChannels,
    frames: frames
  };
}
```

---

## 9. Adaptive Mode Selection

### 9.1 자동 모드 전환

```typescript
interface AdaptiveProcessor {
  // 현재 모드
  currentMode: 'speech' | 'music' | 'mixed';

  // 전환 히스테리시스
  hysteresis: {
    speechToMusic: number;      // 연속 n 프레임
    musicToSpeech: number;
  };

  // 전환 스무딩
  transitionFrames: number;     // 부드러운 전환
}

function adaptiveProcess(
  signal: Float32Array,
  processor: AdaptiveProcessor
): ProcessedFrame {
  // 모드 검출
  const musicDetection = detectMusic(signal, SAMPLE_RATE);

  // 모드 결정 (히스테리시스 적용)
  const newMode = determineMode(
    musicDetection,
    processor.currentMode,
    processor.hysteresis
  );

  // 처리
  if (newMode === 'music') {
    return processMusicMode(signal, MUSIC_CONFIG);
  } else if (newMode === 'speech') {
    return processSpeechMode(signal, SPEECH_CONFIG);
  } else {
    // Mixed: 두 모드의 가중 평균
    const musicFrame = processMusicMode(signal, MUSIC_CONFIG);
    const speechFrame = processSpeechMode(signal, SPEECH_CONFIG);
    return blendFrames(musicFrame, speechFrame, musicDetection.confidence);
  }
}
```

---

## 10. Performance Requirements

### 10.1 실시간 처리 요구사항

| 알고리즘 | 최대 지연 | 목표 지연 | 복잡도 |
|---------|----------|----------|--------|
| F0 Estimation | 20ms | 10ms | O(N log N) |
| Harmonic Analysis | 5ms | 2ms | O(N log N) |
| TFS Extraction | 5ms | 2ms | O(N) |
| Envelope + Octave | 3ms | 1ms | O(N) |
| Channel Selection | 1ms | 0.5ms | O(1) |
| Stimulation Gen | 2ms | 1ms | O(N) |
| **Total** | **36ms** | **16.5ms** | - |

### 10.2 최적화 가이드라인

```typescript
// 1. FFT 캐싱
const fftCache = new FFTCache(FRAME_SIZE);

// 2. 룩업 테이블
const frequencyToChannelLUT = precomputeFrequencyLUT();
const octaveModulationLUT = precomputeOctaveModulation();

// 3. SIMD 최적화 (WebAssembly)
const simdEnvelope = new SIMDEnvelopeExtractor();

// 4. 병렬 처리
const workerPool = new WorkerPool(4);  // 4 threads
```

---

## 11. Validation

### 11.1 테스트 신호

```typescript
// 옥타브 분리 테스트
function testOctaveSeparation(): void {
  const testTones = [
    { note: 'C4', freq: 261.63 },
    { note: 'C5', freq: 523.25 },
    { note: 'C6', freq: 1046.50 }
  ];

  for (const tone of testTones) {
    const signal = generateSineWave(tone.freq, 1.0, SAMPLE_RATE);
    const f0 = estimateF0Ensemble(signal, SAMPLE_RATE);
    const octave = classifyOctave(f0.f0);

    console.assert(
      octave.noteName === 'C',
      `Expected C, got ${octave.noteName}`
    );
    console.assert(
      Math.abs(f0.f0 - tone.freq) < 5,
      `F0 error: ${Math.abs(f0.f0 - tone.freq)} Hz`
    );
  }
}
```

### 11.2 음악 샘플 테스트

```typescript
// 피아노 옥타브 테스트
async function testPianoOctaves(): Promise<TestResult> {
  const samples = [
    'piano_c3.wav', 'piano_c4.wav', 'piano_c5.wav',
    'piano_c6.wav', 'piano_c7.wav'
  ];

  const results: OctaveTestResult[] = [];

  for (const sample of samples) {
    const audio = await loadAudio(sample);
    const processed = processMusicMode(audio, MUSIC_CONFIG);

    results.push({
      sample: sample,
      detectedOctave: processed.octaveInfo.octaveNumber,
      expectedOctave: parseInt(sample.match(/c(\d)/)?.[1] || '0'),
      correct: processed.octaveInfo.noteName === 'C'
    });
  }

  return {
    total: results.length,
    correct: results.filter(r => r.correct).length,
    accuracy: results.filter(r => r.correct).length / results.length
  };
}
```

---

**Document ID**: WIA-CI-PHASE2-001
**Version**: 1.0.0
**Last Updated**: 2025-12-16
**Copyright**: © 2025 WIA - MIT License
