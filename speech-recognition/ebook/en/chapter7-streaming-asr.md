# Chapter 7: Streaming ASR

## Introduction

Streaming ASR processes audio in real-time, essential for interactive applications like voice assistants, live captioning, and conversational AI. This chapter explores techniques for low-latency speech recognition that serves 弘익人間 (Benefit All Humanity) through responsive, natural interaction.

## Real-Time Requirements

### Latency Constraints

```python
class LatencyRequirements:
    """Latency requirements for different applications"""

    REQUIREMENTS = {
        'Voice Assistant': {
            'max_latency_ms': 300,
            'target_latency_ms': 150,
            'description': 'User expects immediate response'
        },
        'Live Captioning': {
            'max_latency_ms': 1000,
            'target_latency_ms': 500,
            'description': 'Keep up with speaker'
        },
        'Transcription': {
            'max_latency_ms': 5000,
            'target_latency_ms': 2000,
            'description': 'Near real-time acceptable'
        },
        'Phone System': {
            'max_latency_ms': 500,
            'target_latency_ms': 200,
            'description': 'Natural conversation flow'
        }
    }

    @staticmethod
    def check_latency(application, measured_latency_ms):
        """Check if latency meets requirements"""
        req = LatencyRequirements.REQUIREMENTS.get(application)

        if not req:
            return None

        if measured_latency_ms <= req['target_latency_ms']:
            return 'EXCELLENT'
        elif measured_latency_ms <= req['max_latency_ms']:
            return 'ACCEPTABLE'
        else:
            return 'UNACCEPTABLE'
```

### Latency Components

```python
import time

class LatencyProfiler:
    """Profile latency in streaming ASR pipeline"""

    def __init__(self):
        self.timings = {}

    def measure_component(self, name, func, *args, **kwargs):
        """Measure execution time of component"""
        start = time.time()
        result = func(*args, **kwargs)
        end = time.time()

        latency_ms = (end - start) * 1000
        self.timings[name] = latency_ms

        return result, latency_ms

    def profile_pipeline(self, audio_chunk):
        """Profile entire ASR pipeline"""
        # Audio buffering
        buffered, buffer_latency = self.measure_component(
            'Audio Buffering',
            self._buffer_audio,
            audio_chunk
        )

        # Feature extraction
        features, feature_latency = self.measure_component(
            'Feature Extraction',
            self._extract_features,
            buffered
        )

        # Model inference
        logits, inference_latency = self.measure_component(
            'Model Inference',
            self._run_model,
            features
        )

        # Decoding
        text, decode_latency = self.measure_component(
            'Decoding',
            self._decode,
            logits
        )

        # Calculate total
        total_latency = sum(self.timings.values())

        return {
            'text': text,
            'latency_breakdown': self.timings,
            'total_latency_ms': total_latency
        }

    def _buffer_audio(self, chunk):
        """Buffer audio (simulated)"""
        time.sleep(0.001)  # 1ms
        return chunk

    def _extract_features(self, audio):
        """Extract features (simulated)"""
        time.sleep(0.010)  # 10ms
        return audio

    def _run_model(self, features):
        """Run model inference (simulated)"""
        time.sleep(0.050)  # 50ms
        return features

    def _decode(self, logits):
        """Decode to text (simulated)"""
        time.sleep(0.005)  # 5ms
        return "recognized text"
```

## Streaming Model Architectures

### Online RNN

Causal models that don't look ahead:

```python
import torch
import torch.nn as nn

class OnlineRNNASR(nn.Module):
    """Online RNN for streaming ASR"""

    def __init__(self, input_dim, vocab_size, hidden_dim=512):
        super().__init__()

        # Unidirectional LSTM (no future context)
        self.lstm = nn.LSTM(
            input_dim,
            hidden_dim,
            num_layers=4,
            batch_first=True,
            bidirectional=False  # Causal!
        )

        # Output projection
        self.fc = nn.Linear(hidden_dim, vocab_size)

        # Stateful processing
        self.hidden_state = None

    def forward(self, features, reset_state=False):
        """
        Process features incrementally

        features: (batch, time, input_dim) - can be single frame
        reset_state: Reset hidden state for new utterance
        """
        if reset_state or self.hidden_state is None:
            self.hidden_state = None

        # Process with saved state
        output, self.hidden_state = self.lstm(features, self.hidden_state)

        # Project to vocabulary
        logits = self.fc(output)

        return torch.log_softmax(logits, dim=2)

    def process_chunk(self, audio_chunk):
        """Process single audio chunk"""
        # Extract features
        features = self.extract_features(audio_chunk)

        # Run model
        log_probs = self.forward(features, reset_state=False)

        return log_probs

# Usage
model = OnlineRNNASR(input_dim=80, vocab_size=5000)

# Process stream
model.forward(chunk1, reset_state=True)  # Start of utterance
model.forward(chunk2)  # Continue
model.forward(chunk3)  # Continue
# ...
```

### Streaming Transformer

Chunk-wise self-attention:

```python
class StreamingTransformer(nn.Module):
    """Transformer for streaming ASR with limited context"""

    def __init__(self, input_dim, vocab_size, d_model=512,
                 chunk_size=40, left_context=30, right_context=0):
        """
        chunk_size: Process this many frames at once
        left_context: Look back this many chunks
        right_context: Look ahead (0 for truly online)
        """
        super().__init__()

        self.chunk_size = chunk_size
        self.left_context = left_context
        self.right_context = right_context

        # Input projection
        self.input_proj = nn.Linear(input_dim, d_model)

        # Encoder layers
        self.encoder_layers = nn.ModuleList([
            StreamingTransformerLayer(d_model, nhead=8)
            for _ in range(12)
        ])

        # Output projection
        self.output_proj = nn.Linear(d_model, vocab_size)

        # Context cache
        self.context_cache = None

    def forward(self, features):
        """
        Process audio features with limited context

        features: (batch, time, input_dim)
        """
        # Project input
        x = self.input_proj(features)

        # Split into chunks
        chunks = self._split_chunks(x)

        outputs = []
        for chunk in chunks:
            # Get context
            context = self._get_context(chunk)

            # Process chunk with context
            for layer in self.encoder_layers:
                chunk = layer(chunk, context)

            outputs.append(chunk)

            # Update cache
            self._update_cache(chunk)

        # Concatenate chunks
        output = torch.cat(outputs, dim=1)

        # Project to vocabulary
        logits = self.output_proj(output)

        return torch.log_softmax(logits, dim=2)

    def _split_chunks(self, x):
        """Split sequence into chunks"""
        batch_size, time, dim = x.size()
        num_chunks = time // self.chunk_size

        chunks = []
        for i in range(num_chunks):
            start = i * self.chunk_size
            end = start + self.chunk_size
            chunks.append(x[:, start:end, :])

        return chunks

    def _get_context(self, chunk):
        """Get left context from cache"""
        if self.context_cache is None:
            return None

        # Return last left_context chunks
        return self.context_cache[:, -self.left_context*self.chunk_size:, :]

    def _update_cache(self, chunk):
        """Update context cache"""
        if self.context_cache is None:
            self.context_cache = chunk
        else:
            self.context_cache = torch.cat([self.context_cache, chunk], dim=1)

            # Limit cache size
            max_cache = (self.left_context + 1) * self.chunk_size
            if self.context_cache.size(1) > max_cache:
                self.context_cache = self.context_cache[:, -max_cache:, :]

class StreamingTransformerLayer(nn.Module):
    """Single streaming transformer layer"""

    def __init__(self, d_model, nhead=8):
        super().__init__()

        self.self_attn = nn.MultiheadAttention(d_model, nhead)
        self.ff = nn.Sequential(
            nn.Linear(d_model, d_model * 4),
            nn.ReLU(),
            nn.Linear(d_model * 4, d_model)
        )

        self.norm1 = nn.LayerNorm(d_model)
        self.norm2 = nn.LayerNorm(d_model)

    def forward(self, chunk, context=None):
        """
        chunk: (batch, chunk_size, d_model)
        context: (batch, context_size, d_model) or None
        """
        # Combine chunk with context
        if context is not None:
            combined = torch.cat([context, chunk], dim=1)
        else:
            combined = chunk

        # Self-attention (only attend to context + chunk)
        attn_out, _ = self.self_attn(
            chunk.transpose(0, 1),
            combined.transpose(0, 1),
            combined.transpose(0, 1)
        )
        attn_out = attn_out.transpose(0, 1)

        # Residual + norm
        chunk = self.norm1(chunk + attn_out)

        # Feed-forward
        ff_out = self.ff(chunk)

        # Residual + norm
        chunk = self.norm2(chunk + ff_out)

        return chunk
```

### RNN-Transducer

Streaming model with alignment-free training:

```python
class RNNTransducer(nn.Module):
    """RNN-Transducer for streaming ASR"""

    def __init__(self, input_dim, vocab_size, enc_dim=512, pred_dim=512, joint_dim=512):
        super().__init__()

        # Encoder (processes audio)
        self.encoder = nn.LSTM(
            input_dim, enc_dim,
            num_layers=6,
            batch_first=True
        )

        # Prediction network (language model)
        self.predictor = nn.LSTM(
            vocab_size, pred_dim,
            num_layers=2,
            batch_first=True
        )

        # Joint network
        self.joint = nn.Sequential(
            nn.Linear(enc_dim + pred_dim, joint_dim),
            nn.Tanh(),
            nn.Linear(joint_dim, vocab_size)
        )

    def forward(self, features, targets):
        """
        features: (batch, time, input_dim)
        targets: (batch, target_len)
        """
        # Encode audio
        enc_out, _ = self.encoder(features)  # (batch, time, enc_dim)

        # Embed and predict
        # One-hot encode targets
        target_onehot = torch.nn.functional.one_hot(
            targets, num_classes=self.predictor.input_size
        ).float()

        pred_out, _ = self.predictor(target_onehot)  # (batch, target_len, pred_dim)

        # Joint network: combine encoder and predictor
        # Expand dimensions for broadcasting
        enc_expanded = enc_out.unsqueeze(2)  # (batch, time, 1, enc_dim)
        pred_expanded = pred_out.unsqueeze(1)  # (batch, 1, target_len, pred_dim)

        # Concatenate and pass through joint
        combined = torch.cat([
            enc_expanded.expand(-1, -1, pred_out.size(1), -1),
            pred_expanded.expand(-1, enc_out.size(1), -1, -1)
        ], dim=-1)

        logits = self.joint(combined)  # (batch, time, target_len, vocab_size)

        return torch.log_softmax(logits, dim=-1)

    def greedy_decode(self, features, max_length=100):
        """Greedy decoding for inference"""
        self.eval()

        with torch.no_grad():
            # Encode
            enc_out, _ = self.encoder(features)

            batch_size = features.size(0)
            time_steps = enc_out.size(1)

            # Initialize
            predictions = [[] for _ in range(batch_size)]
            pred_state = None

            # Decode
            blank = 0
            current_token = torch.zeros(batch_size, 1, dtype=torch.long)

            for t in range(time_steps):
                enc_t = enc_out[:, t:t+1, :]

                while True:
                    # Predict
                    token_onehot = torch.nn.functional.one_hot(
                        current_token, num_classes=self.predictor.input_size
                    ).float()

                    pred_out, pred_state = self.predictor(token_onehot, pred_state)

                    # Joint
                    combined = torch.cat([enc_t.squeeze(1), pred_out.squeeze(1)], dim=-1)
                    logits = self.joint(combined)

                    # Get token
                    token = torch.argmax(logits, dim=-1)

                    if (token == blank).all():
                        # Emit blank, move to next frame
                        break
                    else:
                        # Emit token
                        for b in range(batch_size):
                            if token[b] != blank:
                                predictions[b].append(token[b].item())

                        current_token = token.unsqueeze(1)

            return predictions
```

## Incremental Decoding

### Prefix Beam Search

Streaming-compatible beam search:

```python
class PrefixBeamSearch:
    """Prefix beam search for streaming ASR"""

    def __init__(self, vocab_size, beam_width=10):
        self.vocab_size = vocab_size
        self.beam_width = beam_width
        self.blank_id = 0

    def decode(self, log_probs):
        """
        Incremental decoding

        log_probs: (time, vocab_size) from streaming model
        """
        # Initialize beam: {prefix -> (prob_blank, prob_non_blank)}
        beam = {(): (1.0, 0.0)}  # Empty prefix

        # Process each time step
        for t in range(len(log_probs)):
            new_beam = {}

            # Extend each hypothesis
            for prefix, (p_b, p_nb) in beam.items():
                # Blank: stay in same prefix
                new_p_b = (p_b + p_nb) * log_probs[t, self.blank_id]

                if prefix in new_beam:
                    new_beam[prefix] = (
                        new_beam[prefix][0] + new_p_b,
                        new_beam[prefix][1]
                    )
                else:
                    new_beam[prefix] = (new_p_b, 0.0)

                # Non-blank tokens
                for c in range(1, self.vocab_size):
                    prob = log_probs[t, c]

                    if len(prefix) > 0 and prefix[-1] == c:
                        # Repeated character: need blank
                        new_p_nb = p_b * prob
                    else:
                        # New character
                        new_p_nb = (p_b + p_nb) * prob

                    new_prefix = prefix + (c,)

                    if new_prefix in new_beam:
                        new_beam[new_prefix] = (
                            new_beam[new_prefix][0],
                            new_beam[new_prefix][1] + new_p_nb
                        )
                    else:
                        new_beam[new_prefix] = (0.0, new_p_nb)

            # Prune beam
            beam = self._prune_beam(new_beam)

        # Return best hypothesis
        best_prefix = max(beam.items(), key=lambda x: sum(x[1]))
        return best_prefix[0]

    def _prune_beam(self, beam):
        """Keep top-k hypotheses"""
        # Sort by total probability
        sorted_beam = sorted(
            beam.items(),
            key=lambda x: sum(x[1]),
            reverse=True
        )

        # Keep top-k
        return dict(sorted_beam[:self.beam_width])
```

## Voice Activity Detection

Essential for streaming to detect speech boundaries:

```python
class StreamingVAD:
    """Streaming Voice Activity Detection"""

    def __init__(self, sample_rate=16000, frame_ms=30):
        self.sample_rate = sample_rate
        self.frame_length = int(sample_rate * frame_ms / 1000)
        self.buffer = []

        # Thresholds
        self.energy_threshold = 0.01
        self.speech_frames_required = 3
        self.silence_frames_required = 10

        # State
        self.is_speech = False
        self.speech_frame_count = 0
        self.silence_frame_count = 0

    def process_chunk(self, audio_chunk):
        """
        Process audio chunk and detect speech

        Returns: ('SPEECH', 'SILENCE', 'SPEECH_START', 'SPEECH_END')
        """
        # Add to buffer
        self.buffer.extend(audio_chunk)

        events = []

        # Process complete frames
        while len(self.buffer) >= self.frame_length:
            frame = self.buffer[:self.frame_length]
            self.buffer = self.buffer[self.frame_length:]

            # Detect speech in frame
            is_speech_frame = self._is_speech_frame(frame)

            if is_speech_frame:
                self.speech_frame_count += 1
                self.silence_frame_count = 0

                if not self.is_speech and self.speech_frame_count >= self.speech_frames_required:
                    # Speech start
                    self.is_speech = True
                    events.append('SPEECH_START')

            else:
                self.silence_frame_count += 1
                self.speech_frame_count = 0

                if self.is_speech and self.silence_frame_count >= self.silence_frames_required:
                    # Speech end
                    self.is_speech = False
                    events.append('SPEECH_END')

            # Current state
            if self.is_speech:
                events.append('SPEECH')
            else:
                events.append('SILENCE')

        return events

    def _is_speech_frame(self, frame):
        """Detect if frame contains speech"""
        import numpy as np

        # Energy-based detection
        energy = np.sum(np.array(frame) ** 2) / len(frame)

        return energy > self.energy_threshold
```

## Latency Optimization

### Model Optimization

```python
def optimize_for_latency(model):
    """Optimize model for low latency"""

    # 1. Quantization (INT8)
    quantized_model = torch.quantization.quantize_dynamic(
        model,
        {nn.LSTM, nn.Linear},
        dtype=torch.qint8
    )

    # 2. Pruning
    import torch.nn.utils.prune as prune

    for module in model.modules():
        if isinstance(module, nn.Linear):
            prune.l1_unstructured(module, name='weight', amount=0.3)

    # 3. Knowledge distillation (train smaller model)
    # (Requires training loop)

    return quantized_model
```

## Summary

Streaming ASR enables real-time applications:

- **Online Models**: Causal architectures without future context
- **Chunk Processing**: Process audio incrementally
- **Prefix Beam Search**: Streaming-compatible decoding
- **VAD**: Detect speech boundaries
- **Optimization**: Reduce latency through quantization and pruning

These techniques serve 弘益人間 by enabling natural, responsive voice interaction for all users.

## Review Questions

1. What are the main latency requirements for voice assistants?
2. How do online RNNs differ from bidirectional RNNs?
3. Explain chunk-wise processing in streaming Transformers
4. What is RNN-Transducer and why is it good for streaming?
5. How does prefix beam search work incrementally?
6. Why is VAD important for streaming ASR?
7. What are the main sources of latency in ASR?
8. How does quantization reduce latency?
9. Compare greedy decoding and beam search for streaming
10. What tradeoffs exist between latency and accuracy?

---

**弘益人間** - Streaming ASR enables natural, real-time interaction, making technology more responsive for everyone.

---

*Previous: [Multilingual ASR](chapter6-multilingual-asr.md) | Next: [Production Deployment](chapter8-production-deployment.md)*
