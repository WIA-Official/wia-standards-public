# Chapter 2: ASR Fundamentals

## Introduction to ASR Architecture

Automatic Speech Recognition (ASR) systems transform continuous audio signals into discrete text sequences. This chapter explores the fundamental architecture, signal processing techniques, and core algorithms that make modern ASR possible.

Understanding these fundamentals is essential for building robust, accurate speech recognition systems that serve the principle of 弘益人間 (Benefit All Humanity) by making technology accessible through natural voice interaction.

## Audio Signal Processing

### Digital Audio Representation

Audio is converted from analog (continuous) to digital (discrete) form through sampling:

```python
import numpy as np
import matplotlib.pyplot as plt
from scipy.io import wavfile

class AudioSignal:
    def __init__(self, file_path):
        self.sample_rate, self.samples = wavfile.read(file_path)
        self.duration = len(self.samples) / self.sample_rate
        self.channels = self.samples.shape[1] if len(self.samples.shape) > 1 else 1

    def to_mono(self):
        """Convert stereo to mono"""
        if self.channels == 2:
            self.samples = np.mean(self.samples, axis=1)
            self.channels = 1
        return self

    def normalize(self):
        """Normalize audio to [-1, 1] range"""
        max_val = np.max(np.abs(self.samples))
        if max_val > 0:
            self.samples = self.samples / max_val
        return self

    def resample(self, target_rate):
        """Resample audio to different sample rate"""
        from scipy.signal import resample

        num_samples = int(len(self.samples) * target_rate / self.sample_rate)
        self.samples = resample(self.samples, num_samples)
        self.sample_rate = target_rate
        return self

    def get_frames(self, frame_length_ms=25, frame_shift_ms=10):
        """Split audio into overlapping frames"""
        frame_length = int(self.sample_rate * frame_length_ms / 1000)
        frame_shift = int(self.sample_rate * frame_shift_ms / 1000)

        frames = []
        for i in range(0, len(self.samples) - frame_length, frame_shift):
            frame = self.samples[i:i + frame_length]
            frames.append(frame)

        return np.array(frames)
```

**Key Concepts:**

- **Sample Rate**: Number of samples per second (typical: 8kHz, 16kHz, 44.1kHz, 48kHz)
- **Bit Depth**: Number of bits per sample (typical: 16-bit, 24-bit, 32-bit float)
- **Nyquist Theorem**: Sample rate must be at least 2× highest frequency
- **Quantization**: Converting continuous amplitude to discrete levels

### Pre-emphasis

Pre-emphasis boosts high frequencies to balance the frequency spectrum:

```python
def preemphasis(signal, alpha=0.97):
    """
    Apply pre-emphasis filter to signal

    H(z) = 1 - alpha * z^-1
    """
    return np.append(signal[0], signal[1:] - alpha * signal[:-1])

# Example usage
audio = AudioSignal('speech.wav')
audio.to_mono().normalize()

emphasized = preemphasis(audio.samples)
```

**Why Pre-emphasis?**
- Speech has more energy in lower frequencies
- Pre-emphasis balances the spectrum
- Improves numerical stability in subsequent processing
- Typical alpha values: 0.95 to 0.97

### Windowing

Windowing reduces spectral leakage when analyzing short audio segments:

```python
def apply_window(frames, window_type='hamming'):
    """Apply window function to frames"""
    frame_length = frames.shape[1]

    if window_type == 'hamming':
        window = np.hamming(frame_length)
    elif window_type == 'hanning':
        window = np.hanning(frame_length)
    elif window_type == 'blackman':
        window = np.blackman(frame_length)
    else:
        window = np.ones(frame_length)

    return frames * window

# Common window functions
def hamming_window(N):
    """Hamming window: w(n) = 0.54 - 0.46*cos(2πn/(N-1))"""
    n = np.arange(N)
    return 0.54 - 0.46 * np.cos(2 * np.pi * n / (N - 1))

def hanning_window(N):
    """Hanning window: w(n) = 0.5 - 0.5*cos(2πn/(N-1))"""
    n = np.arange(N)
    return 0.5 - 0.5 * np.cos(2 * np.pi * n / (N - 1))
```

## Feature Extraction

### Mel-Frequency Cepstral Coefficients (MFCCs)

MFCCs are the most widely used features in speech recognition:

```python
import librosa
import numpy as np

class MFCCExtractor:
    def __init__(self, sample_rate=16000, n_mfcc=13, n_fft=512,
                 hop_length=160, n_mels=40):
        self.sample_rate = sample_rate
        self.n_mfcc = n_mfcc
        self.n_fft = n_fft
        self.hop_length = hop_length
        self.n_mels = n_mels

    def extract(self, audio):
        """Extract MFCC features from audio"""
        # Compute MFCCs
        mfccs = librosa.feature.mfcc(
            y=audio,
            sr=self.sample_rate,
            n_mfcc=self.n_mfcc,
            n_fft=self.n_fft,
            hop_length=self.hop_length,
            n_mels=self.n_mels
        )

        # Compute delta and delta-delta
        delta = librosa.feature.delta(mfccs)
        delta2 = librosa.feature.delta(mfccs, order=2)

        # Stack features
        features = np.vstack([mfccs, delta, delta2])

        return features.T  # Transpose to (time, features)

    def extract_from_file(self, audio_path):
        """Extract MFCCs from audio file"""
        audio, _ = librosa.load(audio_path, sr=self.sample_rate)
        return self.extract(audio)

# Detailed MFCC computation
def compute_mfcc_detailed(audio, sample_rate=16000, n_mfcc=13):
    """Detailed MFCC computation steps"""

    # Step 1: Pre-emphasis
    emphasized = preemphasis(audio)

    # Step 2: Framing
    frame_length = int(0.025 * sample_rate)  # 25ms
    frame_shift = int(0.010 * sample_rate)   # 10ms

    frames = []
    for i in range(0, len(emphasized) - frame_length, frame_shift):
        frames.append(emphasized[i:i + frame_length])
    frames = np.array(frames)

    # Step 3: Windowing
    windowed = frames * np.hamming(frame_length)

    # Step 4: FFT
    fft_size = 512
    magnitude_spectrum = np.abs(np.fft.rfft(windowed, fft_size))
    power_spectrum = (magnitude_spectrum ** 2) / fft_size

    # Step 5: Mel filterbank
    n_mels = 40
    mel_filters = create_mel_filterbank(sample_rate, fft_size, n_mels)
    mel_spectrum = np.dot(power_spectrum, mel_filters.T)

    # Step 6: Log
    log_mel = np.log(mel_spectrum + 1e-10)

    # Step 7: DCT (Discrete Cosine Transform)
    from scipy.fftpack import dct
    mfcc = dct(log_mel, type=2, axis=1, norm='ortho')[:, :n_mfcc]

    return mfcc

def create_mel_filterbank(sample_rate, n_fft, n_mels):
    """Create mel-scale filterbank"""
    def hz_to_mel(hz):
        return 2595 * np.log10(1 + hz / 700)

    def mel_to_hz(mel):
        return 700 * (10 ** (mel / 2595) - 1)

    # Frequency range
    low_freq = 0
    high_freq = sample_rate / 2

    # Mel scale points
    low_mel = hz_to_mel(low_freq)
    high_mel = hz_to_mel(high_freq)
    mel_points = np.linspace(low_mel, high_mel, n_mels + 2)
    hz_points = mel_to_hz(mel_points)

    # FFT bin indices
    bin_points = np.floor((n_fft + 1) * hz_points / sample_rate).astype(int)

    # Create filterbank
    filters = np.zeros((n_mels, n_fft // 2 + 1))

    for i in range(1, n_mels + 1):
        left = bin_points[i - 1]
        center = bin_points[i]
        right = bin_points[i + 1]

        # Left slope
        for j in range(left, center):
            filters[i - 1, j] = (j - left) / (center - left)

        # Right slope
        for j in range(center, right):
            filters[i - 1, j] = (right - j) / (right - center)

    return filters
```

**MFCC Pipeline:**
1. Pre-emphasis
2. Framing
3. Windowing
4. FFT
5. Mel filterbank
6. Logarithm
7. DCT

### Filter Bank Features

An alternative to MFCCs that preserves more information:

```python
def extract_filterbank_features(audio, sample_rate=16000, n_filters=40):
    """Extract log mel-filterbank features"""

    # Pre-process
    emphasized = preemphasis(audio)

    # Get frames
    frame_length = int(0.025 * sample_rate)
    frame_shift = int(0.010 * sample_rate)

    frames = librosa.util.frame(
        emphasized,
        frame_length=frame_length,
        hop_length=frame_shift
    ).T

    # Apply window
    windowed = frames * np.hamming(frame_length)

    # Compute power spectrum
    power_spectrum = np.abs(np.fft.rfft(windowed, 512)) ** 2

    # Apply mel filterbank
    mel_filters = create_mel_filterbank(sample_rate, 512, n_filters)
    mel_spectrum = np.dot(power_spectrum, mel_filters.T)

    # Log
    log_mel = np.log(mel_spectrum + 1e-10)

    return log_mel
```

### Spectrogram Features

Raw spectrograms are increasingly used in deep learning models:

```python
def compute_spectrogram(audio, sample_rate=16000, n_fft=512,
                        hop_length=160, power=2.0):
    """Compute power spectrogram"""

    # STFT
    stft = librosa.stft(
        audio,
        n_fft=n_fft,
        hop_length=hop_length,
        window='hann'
    )

    # Power spectrogram
    power_spec = np.abs(stft) ** power

    # Convert to dB
    db_spec = librosa.power_to_db(power_spec, ref=np.max)

    return db_spec

# Mel spectrogram
def compute_mel_spectrogram(audio, sample_rate=16000, n_mels=80):
    """Compute mel spectrogram"""

    mel_spec = librosa.feature.melspectrogram(
        y=audio,
        sr=sample_rate,
        n_mels=n_mels,
        n_fft=512,
        hop_length=160
    )

    log_mel_spec = librosa.power_to_db(mel_spec, ref=np.max)

    return log_mel_spec
```

## Voice Activity Detection (VAD)

VAD identifies speech vs. non-speech segments to improve efficiency:

```python
class VoiceActivityDetector:
    def __init__(self, sample_rate=16000, frame_length_ms=30,
                 energy_threshold=0.1, zero_crossing_threshold=0.3):
        self.sample_rate = sample_rate
        self.frame_length = int(sample_rate * frame_length_ms / 1000)
        self.energy_threshold = energy_threshold
        self.zcr_threshold = zero_crossing_threshold

    def detect(self, audio):
        """Detect voice activity in audio"""
        frames = self._get_frames(audio)

        # Compute features
        energy = self._compute_energy(frames)
        zcr = self._compute_zero_crossing_rate(frames)

        # Normalize
        energy_norm = (energy - np.min(energy)) / (np.max(energy) - np.min(energy) + 1e-10)
        zcr_norm = (zcr - np.min(zcr)) / (np.max(zcr) - np.min(zcr) + 1e-10)

        # Detect voice
        voice_frames = (energy_norm > self.energy_threshold) & \
                       (zcr_norm < self.zcr_threshold)

        return voice_frames

    def _get_frames(self, audio):
        """Split audio into frames"""
        frames = []
        for i in range(0, len(audio) - self.frame_length, self.frame_length // 2):
            frames.append(audio[i:i + self.frame_length])
        return np.array(frames)

    def _compute_energy(self, frames):
        """Compute frame energy"""
        return np.sum(frames ** 2, axis=1)

    def _compute_zero_crossing_rate(self, frames):
        """Compute zero crossing rate"""
        signs = np.sign(frames)
        crossings = np.abs(np.diff(signs, axis=1))
        return np.sum(crossings, axis=1) / frames.shape[1]

    def get_speech_segments(self, audio):
        """Extract speech segments from audio"""
        voice_frames = self.detect(audio)

        segments = []
        in_speech = False
        start = 0

        for i, is_voice in enumerate(voice_frames):
            if is_voice and not in_speech:
                start = i * (self.frame_length // 2)
                in_speech = True
            elif not is_voice and in_speech:
                end = i * (self.frame_length // 2)
                segments.append((start, end))
                in_speech = False

        return segments

# Advanced WebRTC VAD
class WebRTCVAD:
    """More sophisticated VAD based on WebRTC"""

    def __init__(self, aggressiveness=3):
        """
        aggressiveness: 0-3, higher = more aggressive filtering
        """
        import webrtcvad
        self.vad = webrtcvad.Vad(aggressiveness)

    def detect(self, audio, sample_rate=16000):
        """Detect voice activity"""
        # WebRTC VAD requires 10, 20, or 30ms frames
        frame_duration_ms = 30
        frame_length = int(sample_rate * frame_duration_ms / 1000)

        frames = []
        for i in range(0, len(audio) - frame_length, frame_length):
            frame = audio[i:i + frame_length]
            # Convert to 16-bit PCM
            frame_bytes = (frame * 32768).astype(np.int16).tobytes()

            is_speech = self.vad.is_speech(frame_bytes, sample_rate)
            frames.append(is_speech)

        return np.array(frames)
```

## Phonetic Representation

### Phonemes

Phonemes are the smallest units of sound that distinguish words:

```python
# English phoneme set (ARPAbet)
ARPABET_PHONEMES = {
    # Vowels
    'AA': 'odd',      'AE': 'at',       'AH': 'hut',
    'AO': 'ought',    'AW': 'cow',      'AY': 'hide',
    'EH': 'Ed',       'ER': 'hurt',     'EY': 'ate',
    'IH': 'it',       'IY': 'eat',      'OW': 'oat',
    'OY': 'toy',      'UH': 'hood',     'UW': 'two',

    # Consonants
    'B': 'be',        'CH': 'cheese',   'D': 'dee',
    'DH': 'thee',     'F': 'fee',       'G': 'green',
    'HH': 'he',       'JH': 'gee',      'K': 'key',
    'L': 'lee',       'M': 'me',        'N': 'knee',
    'NG': 'ping',     'P': 'pee',       'R': 'read',
    'S': 'sea',       'SH': 'she',      'T': 'tea',
    'TH': 'theta',    'V': 'vee',       'W': 'we',
    'Y': 'yield',     'Z': 'zee',       'ZH': 'seizure'
}

class Phoneme:
    def __init__(self, symbol, stress=None):
        self.symbol = symbol
        self.stress = stress  # 0, 1, or 2 for vowels

    def __repr__(self):
        if self.stress is not None:
            return f"{self.symbol}{self.stress}"
        return self.symbol

# Pronunciation dictionary
class PronunciationDict:
    def __init__(self):
        self.dict = {}

    def load_cmudict(self, path='cmudict.txt'):
        """Load CMU Pronouncing Dictionary"""
        with open(path, 'r') as f:
            for line in f:
                if line.startswith(';;;'):
                    continue

                parts = line.strip().split()
                if len(parts) < 2:
                    continue

                word = parts[0].lower()
                phonemes = parts[1:]

                self.dict[word] = [Phoneme(p[:-1] if p[-1].isdigit() else p,
                                          int(p[-1]) if p[-1].isdigit() else None)
                                  for p in phonemes]

    def lookup(self, word):
        """Get phonemes for word"""
        return self.dict.get(word.lower(), None)
```

### Grapheme-to-Phoneme (G2P)

Converting text to phonemes for words not in the dictionary:

```python
class GraphemeToPhoneme:
    """Neural G2P model"""

    def __init__(self, model_path):
        import torch
        self.model = torch.load(model_path)
        self.grapheme_vocab = self._load_vocab('graphemes.txt')
        self.phoneme_vocab = self._load_vocab('phonemes.txt')

    def convert(self, word):
        """Convert word to phoneme sequence"""
        # Encode graphemes
        grapheme_ids = [self.grapheme_vocab.get(c, 0) for c in word.lower()]

        # Run model
        with torch.no_grad():
            phoneme_ids = self.model.predict(grapheme_ids)

        # Decode phonemes
        phonemes = [self.phoneme_vocab.inverse[p] for p in phoneme_ids]

        return phonemes
```

## Alignment

Aligning audio with phonemes or words is crucial for training:

```python
def forced_alignment(audio_features, phoneme_sequence, acoustic_model):
    """
    Align audio features with phoneme sequence using Viterbi algorithm

    Returns: List of (phoneme, start_frame, end_frame) tuples
    """
    T = len(audio_features)  # Number of frames
    N = len(phoneme_sequence)  # Number of phonemes

    # Initialize DP table
    dp = np.zeros((T, N))
    backpointer = np.zeros((T, N), dtype=int)

    # Initialization
    dp[0, 0] = acoustic_model.score(audio_features[0], phoneme_sequence[0])

    # Forward pass
    for t in range(1, T):
        for s in range(N):
            # Stay in same phoneme
            stay_score = dp[t-1, s]

            # Transition to this phoneme
            trans_score = dp[t-1, s-1] if s > 0 else -np.inf

            # Choose best
            if stay_score > trans_score:
                dp[t, s] = stay_score
                backpointer[t, s] = s
            else:
                dp[t, s] = trans_score
                backpointer[t, s] = s - 1

            # Add acoustic score
            dp[t, s] += acoustic_model.score(audio_features[t], phoneme_sequence[s])

    # Backtrack
    alignment = []
    s = N - 1
    end = T - 1

    for t in range(T - 1, -1, -1):
        if backpointer[t, s] != s:
            # State transition occurred
            alignment.append((phoneme_sequence[s], t, end))
            end = t
            s = backpointer[t, s]

    alignment.append((phoneme_sequence[0], 0, end))
    alignment.reverse()

    return alignment
```

## Decoding Fundamentals

### Beam Search

Beam search efficiently finds likely word sequences:

```python
class BeamSearchDecoder:
    def __init__(self, beam_width=10):
        self.beam_width = beam_width

    def decode(self, acoustic_scores, language_model, vocabulary):
        """
        Decode using beam search

        acoustic_scores: (T, vocab_size) log probabilities
        language_model: LM providing P(word|context)
        vocabulary: List of words
        """
        T = len(acoustic_scores)

        # Initial beam: [(text, score)]
        beam = [('', 0.0)]

        for t in range(T):
            candidates = []

            for text, score in beam:
                # Get acoustic scores for all words
                for word_id, word in enumerate(vocabulary):
                    acoustic_score = acoustic_scores[t, word_id]

                    # Get language model score
                    lm_score = language_model.score(text, word)

                    # Combined score
                    new_score = score + acoustic_score + 0.5 * lm_score
                    new_text = text + ' ' + word if text else word

                    candidates.append((new_text, new_score))

            # Keep top-k candidates
            beam = sorted(candidates, key=lambda x: x[1], reverse=True)[:self.beam_width]

        # Return best hypothesis
        return beam[0][0]
```

## Summary

This chapter covered the fundamental components of ASR systems:

- **Audio Processing**: Sampling, pre-emphasis, windowing, and framing
- **Feature Extraction**: MFCCs, filter banks, and spectrograms
- **Voice Activity Detection**: Identifying speech segments
- **Phonetic Representation**: Phonemes and pronunciation dictionaries
- **Alignment**: Matching audio to phoneme sequences
- **Decoding**: Beam search for finding likely transcriptions

These fundamentals form the foundation for modern ASR systems. Understanding these concepts is essential for building systems that serve 弘益人間 by making speech recognition accessible and reliable for all users.

## Review Questions

1. Why is pre-emphasis applied to speech signals?
2. What is the purpose of windowing in speech processing?
3. Describe the seven steps of MFCC computation.
4. What is the difference between MFCCs and filter bank features?
5. How does Voice Activity Detection improve ASR efficiency?
6. What are phonemes and why are they important?
7. Explain the concept of forced alignment.
8. How does beam search balance acoustic and language model scores?
9. What is the Nyquist theorem and why is it relevant?
10. Compare energy-based and WebRTC VAD approaches.

## Practical Exercises

1. **MFCC Implementation**: Implement MFCC extraction from scratch without libraries
2. **VAD Evaluation**: Compare different VAD algorithms on noisy audio
3. **Alignment Visualization**: Visualize forced alignment results
4. **Beam Search**: Implement beam search decoder with configurable width
5. **Feature Analysis**: Analyze how different features perform on various accents

---

**弘益人間 (Hongik Ingan)** - *Benefit All Humanity*

By mastering these fundamentals, we build the foundation for ASR systems that work reliably across languages, accents, and conditions - ensuring that speech recognition technology truly benefits all of humanity.

---

*Previous: [Introduction](chapter1-introduction.md) | Next: [Acoustic Models](chapter3-acoustic-models.md)*
