# WIA-AI-022 PHASE 1: Foundation

## Overview

PHASE 1 establishes the foundational specifications for speech recognition systems, covering basic ASR functionality, audio processing, and core components. This phase ensures all implementations provide essential speech-to-text capabilities that serve 弘益人間 (Benefit All Humanity).

**Status**: ✅ Stable
**Version**: 1.0.0
**Last Updated**: 2025-01-XX

## Core Requirements

### 1.1 Audio Input Specifications

**REQUIRED** implementations MUST support:

- **Sample Rates**: 8kHz, 16kHz, 44.1kHz, 48kHz
- **Bit Depth**: 16-bit PCM (minimum), 24-bit and 32-bit float (recommended)
- **Channels**: Mono (required), Stereo (optional)
- **Formats**: WAV (required), FLAC, OGG, MP3 (optional)

```typescript
interface AudioConfig {
  sampleRate: 8000 | 16000 | 44100 | 48000;
  bitDepth: 16 | 24 | 32;
  channels: 1 | 2;
  format: 'wav' | 'flac' | 'ogg' | 'mp3';
}
```

### 1.2 Feature Extraction

**REQUIRED** feature extraction methods:

1. **Mel-Frequency Cepstral Coefficients (MFCCs)**
   - Default: 13 coefficients
   - Frame length: 25ms
   - Frame shift: 10ms
   - Include delta and delta-delta features

2. **Mel Spectrogram**
   - Number of mel bins: 40-80
   - FFT size: 512 or 1024
   - Window: Hamming or Hann

```python
# Example MFCC extraction
def extract_mfcc(audio: np.ndarray, sample_rate: int = 16000) -> np.ndarray:
    """
    Extract MFCC features

    Args:
        audio: Audio samples
        sample_rate: Sample rate in Hz

    Returns:
        MFCC features (time, n_mfcc)
    """
    import librosa

    mfcc = librosa.feature.mfcc(
        y=audio,
        sr=sample_rate,
        n_mfcc=13,
        n_fft=512,
        hop_length=160  # 10ms at 16kHz
    )

    # Add delta features
    delta = librosa.feature.delta(mfcc)
    delta2 = librosa.feature.delta(mfcc, order=2)

    # Stack features
    features = np.vstack([mfcc, delta, delta2])

    return features.T  # (time, features)
```

### 1.3 Voice Activity Detection (VAD)

**REQUIRED** VAD capabilities:

- Energy-based detection (minimum)
- WebRTC VAD or equivalent (recommended)
- Frame-level speech/non-speech classification
- Configurable sensitivity (0.0 to 1.0)

```typescript
interface VADConfig {
  algorithm: 'energy' | 'webrtc' | 'ml';
  sensitivity: number;  // 0.0 (lenient) to 1.0 (strict)
  frameLengthMs: 10 | 20 | 30;
}

interface VADResult {
  isSpeech: boolean;
  confidence: number;
  startTime: number;
  endTime: number;
}
```

### 1.4 Basic ASR Interface

**REQUIRED** ASR API methods:

```typescript
interface ASREngine {
  /**
   * Transcribe audio file
   */
  transcribeFile(
    filePath: string,
    options?: TranscriptionOptions
  ): Promise<TranscriptionResult>;

  /**
   * Transcribe audio buffer
   */
  transcribeBuffer(
    audioBuffer: ArrayBuffer,
    options?: TranscriptionOptions
  ): Promise<TranscriptionResult>;

  /**
   * Real-time streaming transcription
   */
  createStream(
    options?: StreamOptions
  ): TranscriptionStream;
}

interface TranscriptionResult {
  text: string;
  confidence: number;
  words?: WordInfo[];
  duration: number;
  language?: string;
}

interface WordInfo {
  word: string;
  startTime: number;
  endTime: number;
  confidence: number;
}
```

### 1.5 Language Support

**REQUIRED** minimum language support:

- English (en-US, en-GB)
- At least 3 additional languages from different families:
  - Germanic: German (de-DE)
  - Romance: Spanish (es-ES), French (fr-FR)
  - East Asian: Mandarin (zh-CN), Japanese (ja-JP), Korean (ko-KR)

**RECOMMENDED**: Support for 20+ languages

```typescript
type LanguageCode =
  | 'en-US' | 'en-GB'
  | 'es-ES' | 'fr-FR' | 'de-DE'
  | 'zh-CN' | 'ja-JP' | 'ko-KR'
  | 'ar-SA' | 'hi-IN' | 'pt-BR'
  | 'ru-RU' | 'it-IT' | 'nl-NL'
  | 'tr-TR' | 'pl-PL' | 'sv-SE';

interface LanguageConfig {
  code: LanguageCode;
  acousticModel: string;
  languageModel: string;
  vocabulary: string[];
}
```

### 1.6 Performance Benchmarks

**REQUIRED** minimum performance:

| Metric | Minimum | Target | Excellent |
|--------|---------|--------|-----------|
| Word Error Rate (WER) | <20% | <10% | <5% |
| Latency (offline) | <5s | <2s | <1s |
| Real-time Factor (RTF) | <1.0 | <0.5 | <0.3 |
| Throughput | 10x real-time | 20x | 50x |

```python
def calculate_wer(reference: str, hypothesis: str) -> float:
    """Calculate Word Error Rate"""
    import editdistance

    ref_words = reference.split()
    hyp_words = hypothesis.split()

    distance = editdistance.eval(ref_words, hyp_words)
    wer = distance / len(ref_words) if len(ref_words) > 0 else 0

    return wer

def measure_rtf(audio_duration: float, processing_time: float) -> float:
    """Calculate Real-Time Factor"""
    return processing_time / audio_duration
```

### 1.7 Output Format

**REQUIRED** output formats:

1. **Plain Text**
   ```json
   {
     "text": "Hello world",
     "confidence": 0.95
   }
   ```

2. **Detailed JSON**
   ```json
   {
     "text": "Hello world",
     "confidence": 0.95,
     "words": [
       {"word": "Hello", "start": 0.0, "end": 0.5, "confidence": 0.96},
       {"word": "world", "start": 0.6, "end": 1.0, "confidence": 0.94}
     ],
     "language": "en-US",
     "duration": 1.0,
     "metadata": {
       "model": "wia-asr-v1",
       "timestamp": "2025-01-XX"
     }
   }
   ```

3. **WebVTT** (for subtitles)
   ```
   WEBVTT

   00:00:00.000 --> 00:00:00.500
   Hello

   00:00:00.600 --> 00:00:01.000
   world
   ```

## Quality Assurance

### 2.1 Testing Requirements

**REQUIRED** test coverage:

- Unit tests for all public APIs
- Integration tests for end-to-end pipelines
- Performance benchmarks on standard datasets
- Regression tests for accuracy

**Standard Test Sets**:
- LibriSpeech (English)
- Common Voice (multilingual)
- Custom domain-specific datasets

### 2.2 Validation Metrics

```python
class ASRValidator:
    """Validate ASR implementation against PHASE 1 requirements"""

    def validate_audio_formats(self, engine: ASREngine) -> bool:
        """Test supported audio formats"""
        formats = ['wav', 'flac']
        for fmt in formats:
            try:
                result = engine.transcribe_file(f'test.{fmt}')
                assert result.text is not None
            except:
                return False
        return True

    def validate_languages(self, engine: ASREngine) -> bool:
        """Test minimum language support"""
        required_langs = ['en-US', 'es-ES', 'zh-CN']
        for lang in required_langs:
            try:
                result = engine.transcribe_file('test.wav', language=lang)
                assert result.language == lang
            except:
                return False
        return True

    def validate_performance(self, engine: ASREngine) -> Dict[str, bool]:
        """Test performance requirements"""
        results = {}

        # WER test
        wer = self.measure_wer(engine)
        results['wer'] = wer < 0.20

        # Latency test
        latency = self.measure_latency(engine)
        results['latency'] = latency < 5.0

        # RTF test
        rtf = self.measure_rtf(engine)
        results['rtf'] = rtf < 1.0

        return results
```

## Security & Privacy

### 3.1 Data Protection

**REQUIRED** security measures:

1. **Audio Encryption**
   - TLS 1.3 for transmission
   - AES-256 for storage
   - Secure key management

2. **PII Protection**
   - Automatic redaction of sensitive data (optional)
   - Configurable retention policies
   - GDPR/CCPA compliance support

```typescript
interface SecurityConfig {
  encryption: {
    inTransit: 'tls1.2' | 'tls1.3';
    atRest: 'aes256' | 'aes128';
  };
  pii: {
    redactEmails: boolean;
    redactPhoneNumbers: boolean;
    redactCreditCards: boolean;
  };
  retention: {
    audioRetentionDays: number;
    transcriptRetentionDays: number;
  };
}
```

### 3.2 Access Control

**REQUIRED** authentication:

- API key authentication (minimum)
- OAuth 2.0 / JWT (recommended)
- Rate limiting per user/API key
- Audit logging

## Compliance

### 4.1 Accessibility

**REQUIRED** accessibility features:

- WCAG 2.1 AA compliance for web interfaces
- Screen reader compatibility
- Keyboard navigation support
- High contrast themes

### 4.2 Internationalization

**REQUIRED** i18n support:

- UTF-8 text encoding
- Right-to-left (RTL) language support
- Locale-aware number/date formatting
- Translatable UI elements

## Summary

PHASE 1 establishes the foundation:

✅ Basic audio processing (8kHz-48kHz, multiple formats)
✅ Feature extraction (MFCC, mel spectrogram)
✅ Voice Activity Detection
✅ Core ASR API (file, buffer, stream)
✅ Minimum 4 languages
✅ Performance: WER <20%, RTF <1.0
✅ Security & privacy protections
✅ Accessibility & i18n support

**Next**: [PHASE 2 - Advanced Features](PHASE2.md)

---

**弘益人間 (Hongik Ingan)** - *Benefit All Humanity*

PHASE 1 ensures baseline ASR functionality is accessible, secure, and performant for all users worldwide.
