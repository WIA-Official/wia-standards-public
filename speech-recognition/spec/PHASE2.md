# WIA-AI-022 PHASE 2: Advanced Features

## Overview

PHASE 2 extends PHASE 1 with advanced ASR features including speaker diarization, noise reduction, punctuation restoration, and enhanced multilingual support. These features serve 弘益人間 by improving accuracy and usability in real-world conditions.

**Status**: ✅ Stable
**Version**: 1.0.0
**Prerequisites**: PHASE 1 complete

## Advanced Capabilities

### 2.1 Speaker Diarization

**REQUIRED** speaker identification features:

```typescript
interface SpeakerDiarizationConfig {
  enabled: boolean;
  numSpeakers?: number | 'auto';
  minSpeakers?: number;
  maxSpeakers?: number;
}

interface SpeakerSegment {
  speakerId: string;
  startTime: number;
  endTime: number;
  text: string;
  confidence: number;
}

interface DiarizationResult {
  segments: SpeakerSegment[];
  speakers: Speaker[];
}

interface Speaker {
  id: string;
  name?: string;
  embedding?: number[];
}
```

**Performance Requirements**:
- Diarization Error Rate (DER) <20%
- Speaker identification accuracy >80%
- Support for 2-10 concurrent speakers

### 2.2 Noise Robustness

**REQUIRED** noise handling:

1. **Noise Reduction**
   - Spectral subtraction
   - Wiener filtering
   - Deep learning-based denoising

2. **Supported Noise Types**
   - Stationary (white noise, hum)
   - Non-stationary (traffic, music)
   - Reverberation

```python
class NoiseReduction:
    """Noise reduction module"""

    def __init__(self, method: str = 'spectral'):
        self.method = method

    def reduce(self, audio: np.ndarray, sr: int = 16000) -> np.ndarray:
        """Apply noise reduction"""
        if self.method == 'spectral':
            return self.spectral_subtraction(audio)
        elif self.method == 'wiener':
            return self.wiener_filter(audio)
        elif self.method == 'deep':
            return self.deep_denoise(audio)

    def spectral_subtraction(self, audio: np.ndarray) -> np.ndarray:
        """Spectral subtraction noise reduction"""
        # Estimate noise spectrum from first 0.5s
        noise_frames = int(0.5 * 16000)
        noise_spectrum = np.mean(np.abs(np.fft.fft(audio[:noise_frames])))

        # Subtract noise spectrum
        signal_spectrum = np.fft.fft(audio)
        clean_spectrum = np.maximum(
            np.abs(signal_spectrum) - noise_spectrum,
            0.1 * np.abs(signal_spectrum)
        )

        # Reconstruct
        clean_audio = np.fft.ifft(
            clean_spectrum * np.exp(1j * np.angle(signal_spectrum))
        ).real

        return clean_audio
```

**Performance Requirements**:
- SNR improvement: >10 dB
- Maintain WER <15% in SNR 10dB conditions
- Real-time processing (RTF <0.5)

### 2.3 Punctuation & Formatting

**REQUIRED** text formatting features:

```typescript
interface FormattingConfig {
  punctuation: boolean;
  capitalization: boolean;
  numbers: 'digits' | 'words' | 'auto';
  dates: 'numeric' | 'text' | 'auto';
  currency: boolean;
}

interface FormattedResult {
  rawText: string;
  formattedText: string;
  formatting: FormattingAnnotation[];
}

interface FormattingAnnotation {
  type: 'punctuation' | 'capitalization' | 'number' | 'date';
  position: number;
  value: string;
  confidence: number;
}
```

**Example**:
```
Input:  "hello world how are you today is january first twenty twenty five"
Output: "Hello world. How are you? Today is January 1st, 2025."
```

### 2.4 Enhanced Language Support

**REQUIRED** PHASE 2 language capabilities:

- Support for 20+ languages
- Low-resource language support via transfer learning
- Code-switching detection (optional)
- Dialect recognition (optional)

```typescript
interface MultilingualConfig {
  primaryLanguage: LanguageCode;
  secondaryLanguages?: LanguageCode[];
  autoDetect: boolean;
  codeSwitching: boolean;
  dialectDetection: boolean;
}

interface LanguageDetectionResult {
  language: LanguageCode;
  confidence: number;
  alternatives: Array<{
    language: LanguageCode;
    confidence: number;
  }>;
}
```

### 2.5 Confidence Scoring

**REQUIRED** confidence estimation:

```typescript
interface ConfidenceConfig {
  level: 'word' | 'sentence' | 'utterance';
  threshold: number;  // 0.0 to 1.0
}

interface ConfidenceResult {
  overall: number;
  words: Array<{
    word: string;
    confidence: number;
    alternatives?: Array<{
      word: string;
      confidence: number;
    }>;
  }>;
}
```

**Accuracy Requirements**:
- Confidence calibration error <10%
- High-confidence (>0.9) predictions: WER <5%
- Low-confidence (<0.5) flagging: recall >90%

### 2.6 Custom Vocabulary

**REQUIRED** vocabulary customization:

```typescript
interface CustomVocabulary {
  words: string[];
  phrases: string[];
  boostedTerms: Array<{
    term: string;
    boost: number;  // 1.0 = normal, >1.0 = boosted
  }>;
}

interface VocabularyConfig {
  custom: CustomVocabulary;
  domain?: 'medical' | 'legal' | 'technical' | 'general';
}
```

**Example**:
```typescript
const medicalVocab: CustomVocabulary = {
  words: ['acetaminophen', 'hypertension', 'arrhythmia'],
  phrases: ['blood pressure', 'heart rate', 'CT scan'],
  boostedTerms: [
    { term: 'diagnosis', boost: 2.0 },
    { term: 'prescription', boost: 1.5 }
  ]
};
```

### 2.7 N-best Hypotheses

**REQUIRED** multiple hypothesis generation:

```typescript
interface NBestConfig {
  n: number;  // Number of hypotheses (1-10)
  includeScores: boolean;
}

interface NBestResult {
  hypotheses: Array<{
    text: string;
    score: number;
    words: WordInfo[];
  }>;
}
```

## Performance Benchmarks

### PHASE 2 Performance Targets

| Metric | Minimum | Target | Excellent |
|--------|---------|--------|-----------|
| WER (clean) | <10% | <5% | <3% |
| WER (noisy, SNR 10dB) | <25% | <15% | <10% |
| Diarization Error Rate | <25% | <15% | <10% |
| Language Detection Accuracy | >85% | >90% | >95% |
| Punctuation F1 Score | >80% | >90% | >95% |
| Real-time Factor | <0.7 | <0.4 | <0.2 |

## Implementation Example

```python
from wia_speech import ASREngine, PHASE2Config

# Initialize with PHASE 2 features
config = PHASE2Config(
    # Speaker diarization
    diarization=SpeakerDiarizationConfig(
        enabled=True,
        numSpeakers='auto',
        minSpeakers=2,
        maxSpeakers=5
    ),

    # Noise reduction
    noiseReduction=NoiseReductionConfig(
        enabled=True,
        method='deep',
        aggressiveness=0.7
    ),

    # Formatting
    formatting=FormattingConfig(
        punctuation=True,
        capitalization=True,
        numbers='auto'
    ),

    # Multilingual
    multilingual=MultilingualConfig(
        primaryLanguage='en-US',
        autoDetect=True,
        codeSwitching=True
    ),

    # Custom vocabulary
    vocabulary=VocabularyConfig(
        domain='medical'
    )
)

asr = ASREngine(config)

# Transcribe with advanced features
result = asr.transcribe_file('meeting.wav')

print(f"Formatted: {result.formattedText}")
print(f"Speakers: {len(result.speakers)}")

for segment in result.segments:
    print(f"[{segment.speakerId}] {segment.text}")
```

## Testing & Validation

### PHASE 2 Test Suite

```python
class PHASE2Validator:
    """Validate PHASE 2 compliance"""

    def validate_diarization(self, engine: ASREngine) -> bool:
        """Test speaker diarization"""
        result = engine.transcribe_file(
            'multi_speaker.wav',
            diarization=True
        )

        # Check DER
        der = self.calculate_der(result.segments, ground_truth)
        return der < 0.25

    def validate_noise_robustness(self, engine: ASREngine) -> bool:
        """Test noise handling"""
        # Test with various SNR levels
        for snr in [20, 15, 10, 5]:
            noisy_audio = self.add_noise(clean_audio, snr)
            result = engine.transcribe_buffer(noisy_audio)
            wer = calculate_wer(reference, result.text)

            if snr == 10 and wer > 0.25:
                return False

        return True

    def validate_punctuation(self, engine: ASREngine) -> bool:
        """Test punctuation restoration"""
        result = engine.transcribe_file(
            'test.wav',
            formatting={'punctuation': True}
        )

        # Calculate F1 score
        f1 = self.calculate_punctuation_f1(
            result.formattedText,
            reference_with_punctuation
        )

        return f1 > 0.80
```

## Summary

PHASE 2 adds:

✅ Speaker diarization (2-10 speakers, DER <25%)
✅ Noise reduction (SNR +10dB, WER <15% at SNR 10dB)
✅ Automatic punctuation & formatting
✅ 20+ language support with auto-detection
✅ Confidence scoring & calibration
✅ Custom vocabulary & domain adaptation
✅ N-best hypothesis generation

**Next**: [PHASE 3 - Intelligence](PHASE3.md)

---

**弘益人間 (Hongik Ingan)** - *Benefit All Humanity*

PHASE 2 enhances ASR for real-world use with advanced features that improve accuracy and usability for all users.
