# Chapter 5: End-to-End ASR Systems

## Introduction

End-to-end (E2E) systems directly map audio to text without intermediate representations like phonemes, revolutionizing speech recognition. This chapter explores modern E2E architectures that simplify training and improve performance while serving 弘益人間 (Benefit All Humanity) through more accessible and multilingual ASR.

## Sequence-to-Sequence Models

### Basic Encoder-Decoder Architecture

```python
import torch
import torch.nn as nn

class Encoder(nn.Module):
    """Audio encoder"""

    def __init__(self, input_dim, hidden_dim, num_layers=3):
        super().__init__()

        self.lstm = nn.LSTM(
            input_dim,
            hidden_dim,
            num_layers,
            batch_first=True,
            bidirectional=True
        )

        self.projection = nn.Linear(hidden_dim * 2, hidden_dim)

    def forward(self, features, lengths):
        """
        features: (batch, time, feature_dim)
        lengths: (batch,) actual sequence lengths
        """
        # Pack sequence
        packed = nn.utils.rnn.pack_padded_sequence(
            features, lengths, batch_first=True, enforce_sorted=False
        )

        # Encode
        outputs, (hidden, cell) = self.lstm(packed)

        # Unpack
        outputs, _ = nn.utils.rnn.pad_packed_sequence(outputs, batch_first=True)

        # Project to hidden_dim
        outputs = self.projection(outputs)

        return outputs, (hidden, cell)

class AttentionDecoder(nn.Module):
    """Attention-based decoder"""

    def __init__(self, vocab_size, embed_dim, hidden_dim, num_layers=2):
        super().__init__()

        self.embedding = nn.Embedding(vocab_size, embed_dim)

        self.lstm = nn.LSTM(
            embed_dim + hidden_dim,  # Input + context
            hidden_dim,
            num_layers,
            batch_first=True
        )

        self.attention = BahdanauAttention(hidden_dim)
        self.output_projection = nn.Linear(hidden_dim, vocab_size)

    def forward(self, encoder_outputs, targets, encoder_lengths):
        """
        encoder_outputs: (batch, enc_time, hidden_dim)
        targets: (batch, dec_time) target tokens
        """
        batch_size = encoder_outputs.size(0)
        dec_length = targets.size(1)

        # Embed targets
        embedded = self.embedding(targets)

        # Initialize decoder hidden state
        hidden = None
        outputs = []

        # Initial context (zeros)
        context = torch.zeros(batch_size, encoder_outputs.size(2))

        for t in range(dec_length):
            # Concatenate embedding and context
            decoder_input = torch.cat([embedded[:, t:t+1, :], context.unsqueeze(1)], dim=2)

            # Decoder step
            output, hidden = self.lstm(decoder_input, hidden)

            # Attention
            context, attention_weights = self.attention(
                output.squeeze(1), encoder_outputs, encoder_lengths
            )

            # Project to vocabulary
            logits = self.output_projection(output.squeeze(1))
            outputs.append(logits)

        outputs = torch.stack(outputs, dim=1)
        return torch.log_softmax(outputs, dim=2)

class BahdanauAttention(nn.Module):
    """Bahdanau (additive) attention mechanism"""

    def __init__(self, hidden_dim):
        super().__init__()

        self.query_projection = nn.Linear(hidden_dim, hidden_dim)
        self.key_projection = nn.Linear(hidden_dim, hidden_dim)
        self.energy_projection = nn.Linear(hidden_dim, 1)

    def forward(self, query, keys, key_lengths):
        """
        query: (batch, hidden_dim) decoder state
        keys: (batch, time, hidden_dim) encoder outputs
        key_lengths: (batch,) actual key lengths
        """
        # Project query and keys
        query_proj = self.query_projection(query).unsqueeze(1)  # (batch, 1, hidden)
        keys_proj = self.key_projection(keys)  # (batch, time, hidden)

        # Compute energy
        energy = self.energy_projection(torch.tanh(query_proj + keys_proj))  # (batch, time, 1)
        energy = energy.squeeze(2)  # (batch, time)

        # Create mask for padding
        mask = self._create_mask(keys.size(1), key_lengths)
        energy = energy.masked_fill(mask, float('-inf'))

        # Compute attention weights
        attention_weights = torch.softmax(energy, dim=1)  # (batch, time)

        # Compute context vector
        context = torch.sum(keys * attention_weights.unsqueeze(2), dim=1)  # (batch, hidden)

        return context, attention_weights

    def _create_mask(self, max_len, lengths):
        """Create padding mask"""
        batch_size = lengths.size(0)
        mask = torch.arange(max_len).expand(batch_size, max_len)
        mask = mask >= lengths.unsqueeze(1)
        return mask

class Seq2SeqASR(nn.Module):
    """Complete Sequence-to-Sequence ASR"""

    def __init__(self, input_dim, vocab_size, hidden_dim=512):
        super().__init__()

        self.encoder = Encoder(input_dim, hidden_dim)
        self.decoder = AttentionDecoder(vocab_size, 256, hidden_dim)

    def forward(self, features, feature_lengths, targets):
        """Training forward pass"""
        # Encode audio
        encoder_outputs, _ = self.encoder(features, feature_lengths)

        # Decode to text
        outputs = self.decoder(encoder_outputs, targets, feature_lengths)

        return outputs

    def greedy_decode(self, features, feature_lengths, max_length=100):
        """Greedy decoding for inference"""
        self.eval()

        with torch.no_grad():
            # Encode
            encoder_outputs, _ = self.encoder(features, feature_lengths)

            batch_size = features.size(0)
            decoded = torch.zeros(batch_size, max_length, dtype=torch.long)
            decoded[:, 0] = 1  # <SOS> token

            for t in range(1, max_length):
                # Decode one step
                outputs = self.decoder(encoder_outputs, decoded[:, :t], feature_lengths)

                # Get most likely token
                next_token = torch.argmax(outputs[:, -1, :], dim=1)
                decoded[:, t] = next_token

                # Stop if all sequences produced <EOS>
                if (next_token == 2).all():  # <EOS> token
                    break

            return decoded
```

### Beam Search Decoding

```python
class BeamSearchDecoder:
    """Beam search for sequence-to-sequence models"""

    def __init__(self, model, beam_width=10, max_length=100):
        self.model = model
        self.beam_width = beam_width
        self.max_length = max_length

    def decode(self, features, feature_lengths):
        """Beam search decoding"""
        self.model.eval()

        with torch.no_grad():
            # Encode
            encoder_outputs, _ = self.model.encoder(features, feature_lengths)

            batch_size = features.size(0)
            results = []

            for b in range(batch_size):
                # Beam search for single example
                beam = self._beam_search_single(
                    encoder_outputs[b:b+1],
                    feature_lengths[b:b+1]
                )
                results.append(beam[0][1])  # Best hypothesis

            return results

    def _beam_search_single(self, encoder_outputs, encoder_lengths):
        """Beam search for single example"""
        # Initial beam: [(score, sequence, hidden)]
        beam = [(0.0, [1], None)]  # Start with <SOS> token

        for t in range(self.max_length):
            candidates = []

            for score, sequence, hidden in beam:
                # Stop if sequence ended
                if sequence[-1] == 2:  # <EOS>
                    candidates.append((score, sequence, hidden))
                    continue

                # Decode one step
                targets = torch.tensor([sequence])
                outputs = self.model.decoder(
                    encoder_outputs, targets, encoder_lengths
                )

                # Get top-k tokens
                log_probs = outputs[0, -1, :]
                topk_log_probs, topk_tokens = torch.topk(log_probs, self.beam_width)

                # Add to candidates
                for log_prob, token in zip(topk_log_probs, topk_tokens):
                    new_score = score + log_prob.item()
                    new_sequence = sequence + [token.item()]
                    candidates.append((new_score, new_sequence, None))

            # Keep top-k candidates
            beam = sorted(candidates, key=lambda x: x[0], reverse=True)[:self.beam_width]

            # Stop if all beams ended
            if all(seq[-1] == 2 for _, seq, _ in beam):
                break

        return beam
```

## Listen, Attend and Spell (LAS)

```python
class LASModel(nn.Module):
    """Listen, Attend and Spell architecture"""

    def __init__(self, input_dim, vocab_size):
        super().__init__()

        # Listener (encoder with pyramidal structure)
        self.listener = PyramidalListener(input_dim, hidden_dim=256)

        # Speller (decoder with attention)
        self.speller = AttentionSpeller(vocab_size, embed_dim=512, hidden_dim=512)

    def forward(self, features, feature_lengths, targets):
        # Listen
        encoded, encoded_lengths = self.listener(features, feature_lengths)

        # Spell
        outputs, attention = self.speller(encoded, encoded_lengths, targets)

        return outputs, attention

class PyramidalListener(nn.Module):
    """Pyramidal BiLSTM encoder for time reduction"""

    def __init__(self, input_dim, hidden_dim):
        super().__init__()

        self.layers = nn.ModuleList([
            nn.LSTM(input_dim, hidden_dim, bidirectional=True, batch_first=True),
            nn.LSTM(hidden_dim * 4, hidden_dim, bidirectional=True, batch_first=True),
            nn.LSTM(hidden_dim * 4, hidden_dim, bidirectional=True, batch_first=True),
        ])

    def forward(self, features, lengths):
        """
        Apply pyramidal structure (reduce time by 2 at each layer)
        """
        outputs = features

        for layer in self.layers:
            outputs, _ = layer(outputs)

            # Reduce time dimension by factor of 2
            batch_size, time, dim = outputs.size()
            if time % 2 == 1:
                outputs = outputs[:, :-1, :]  # Remove last frame if odd
                time -= 1

            # Reshape: concatenate pairs
            outputs = outputs.reshape(batch_size, time // 2, dim * 2)
            lengths = lengths // 2

        return outputs, lengths

class AttentionSpeller(nn.Module):
    """Attention-based decoder (speller)"""

    def __init__(self, vocab_size, embed_dim, hidden_dim):
        super().__init__()

        self.embedding = nn.Embedding(vocab_size, embed_dim)
        self.lstm_cells = nn.ModuleList([
            nn.LSTMCell(embed_dim + hidden_dim, hidden_dim),
            nn.LSTMCell(hidden_dim, hidden_dim)
        ])

        self.attention = MultiHeadAttention(hidden_dim, num_heads=4)
        self.output_projection = nn.Linear(hidden_dim * 2, vocab_size)

    def forward(self, encoder_outputs, encoder_lengths, targets):
        batch_size = encoder_outputs.size(0)
        max_length = targets.size(1)

        # Initialize states
        states = [(
            torch.zeros(batch_size, cell.hidden_size),
            torch.zeros(batch_size, cell.hidden_size)
        ) for cell in self.lstm_cells]

        context = torch.zeros(batch_size, encoder_outputs.size(2))
        outputs = []
        attentions = []

        # Embed targets
        embedded = self.embedding(targets)

        for t in range(max_length):
            # Input: embedding + previous context
            decoder_input = torch.cat([embedded[:, t], context], dim=1)

            # LSTM layers
            for i, cell in enumerate(self.lstm_cells):
                h, c = states[i]
                h, c = cell(decoder_input if i == 0 else h, (h, c))
                states[i] = (h, c)

            # Attention
            query = states[-1][0]
            context, attention_weights = self.attention(
                query, encoder_outputs, encoder_outputs, encoder_lengths
            )

            # Output
            combined = torch.cat([states[-1][0], context], dim=1)
            output = self.output_projection(combined)

            outputs.append(output)
            attentions.append(attention_weights)

        outputs = torch.stack(outputs, dim=1)
        attentions = torch.stack(attentions, dim=1)

        return torch.log_softmax(outputs, dim=2), attentions

class MultiHeadAttention(nn.Module):
    """Multi-head attention mechanism"""

    def __init__(self, hidden_dim, num_heads=8):
        super().__init__()

        self.hidden_dim = hidden_dim
        self.num_heads = num_heads
        self.head_dim = hidden_dim // num_heads

        self.query_proj = nn.Linear(hidden_dim, hidden_dim)
        self.key_proj = nn.Linear(hidden_dim, hidden_dim)
        self.value_proj = nn.Linear(hidden_dim, hidden_dim)
        self.output_proj = nn.Linear(hidden_dim, hidden_dim)

    def forward(self, query, keys, values, key_lengths=None):
        batch_size = query.size(0)

        # Project and split into heads
        Q = self.query_proj(query).view(batch_size, 1, self.num_heads, self.head_dim)
        K = self.key_proj(keys).view(batch_size, -1, self.num_heads, self.head_dim)
        V = self.value_proj(values).view(batch_size, -1, self.num_heads, self.head_dim)

        # Transpose for attention
        Q = Q.transpose(1, 2)  # (batch, heads, 1, head_dim)
        K = K.transpose(1, 2)  # (batch, heads, time, head_dim)
        V = V.transpose(1, 2)  # (batch, heads, time, head_dim)

        # Scaled dot-product attention
        scores = torch.matmul(Q, K.transpose(-2, -1)) / math.sqrt(self.head_dim)

        # Apply mask if provided
        if key_lengths is not None:
            mask = self._create_mask(K.size(2), key_lengths)
            scores = scores.masked_fill(mask.unsqueeze(1).unsqueeze(2), float('-inf'))

        # Attention weights
        attention_weights = torch.softmax(scores, dim=-1)

        # Apply attention to values
        context = torch.matmul(attention_weights, V)

        # Concatenate heads
        context = context.transpose(1, 2).contiguous()
        context = context.view(batch_size, -1, self.hidden_dim)

        # Output projection
        output = self.output_projection(context.squeeze(1))

        return output, attention_weights.squeeze(1).squeeze(1)

    def _create_mask(self, max_len, lengths):
        batch_size = lengths.size(0)
        mask = torch.arange(max_len).expand(batch_size, max_len)
        return mask >= lengths.unsqueeze(1)
```

## Transformer-based ASR

```python
class TransformerASR(nn.Module):
    """Transformer for speech recognition"""

    def __init__(self, input_dim, vocab_size, d_model=512, nhead=8,
                 num_encoder_layers=12, num_decoder_layers=6):
        super().__init__()

        # Input projection
        self.input_projection = nn.Linear(input_dim, d_model)

        # Positional encoding
        self.pos_encoding = PositionalEncoding(d_model)

        # Transformer
        self.transformer = nn.Transformer(
            d_model=d_model,
            nhead=nhead,
            num_encoder_layers=num_encoder_layers,
            num_decoder_layers=num_decoder_layers,
            dim_feedforward=2048,
            dropout=0.1
        )

        # Output embedding
        self.tgt_embedding = nn.Embedding(vocab_size, d_model)

        # Output projection
        self.output_projection = nn.Linear(d_model, vocab_size)

    def forward(self, src, tgt, src_key_padding_mask=None):
        """
        src: (batch, src_len, input_dim)
        tgt: (batch, tgt_len)
        """
        # Project input features
        src = self.input_projection(src)
        src = self.pos_encoding(src)
        src = src.transpose(0, 1)  # (src_len, batch, d_model)

        # Embed target
        tgt = self.tgt_embedding(tgt)
        tgt = self.pos_encoding(tgt)
        tgt = tgt.transpose(0, 1)  # (tgt_len, batch, d_model)

        # Create masks
        tgt_mask = self._generate_square_subsequent_mask(tgt.size(0))

        # Transform
        output = self.transformer(
            src, tgt,
            tgt_mask=tgt_mask,
            src_key_padding_mask=src_key_padding_mask
        )

        # Project to vocabulary
        output = output.transpose(0, 1)  # (batch, tgt_len, d_model)
        logits = self.output_projection(output)

        return torch.log_softmax(logits, dim=2)

    def _generate_square_subsequent_mask(self, sz):
        mask = torch.triu(torch.ones(sz, sz), diagonal=1)
        return mask.masked_fill(mask == 1, float('-inf'))
```

## Conformer

State-of-the-art combining convolution and self-attention:

```python
class Conformer(nn.Module):
    """Conformer model for speech recognition"""

    def __init__(self, input_dim, vocab_size, d_model=512, num_layers=16):
        super().__init__()

        # Subsampling layer
        self.subsampling = ConvSubsampling(input_dim, d_model)

        # Conformer blocks
        self.layers = nn.ModuleList([
            ConformerBlock(d_model) for _ in range(num_layers)
        ])

        # Output layer
        self.output_layer = nn.Linear(d_model, vocab_size)

    def forward(self, features, feature_lengths):
        # Subsample
        x, lengths = self.subsampling(features, feature_lengths)

        # Conformer blocks
        for layer in self.layers:
            x = layer(x)

        # Output
        logits = self.output_layer(x)

        return torch.log_softmax(logits, dim=2), lengths

class ConformerBlock(nn.Module):
    """Single Conformer block"""

    def __init__(self, d_model, num_heads=8, conv_kernel_size=31):
        super().__init__()

        # Feed-forward module 1
        self.ff1 = FeedForwardModule(d_model)

        # Multi-head self-attention
        self.mhsa = nn.MultiheadAttention(d_model, num_heads)

        # Convolution module
        self.conv = ConvolutionModule(d_model, conv_kernel_size)

        # Feed-forward module 2
        self.ff2 = FeedForwardModule(d_model)

        # Layer norms
        self.norm_ff1 = nn.LayerNorm(d_model)
        self.norm_mhsa = nn.LayerNorm(d_model)
        self.norm_conv = nn.LayerNorm(d_model)
        self.norm_ff2 = nn.LayerNorm(d_model)
        self.norm_out = nn.LayerNorm(d_model)

    def forward(self, x):
        # Feed-forward 1
        x = x + 0.5 * self.ff1(self.norm_ff1(x))

        # Multi-head self-attention
        x_norm = self.norm_mhsa(x)
        attn_out, _ = self.mhsa(x_norm, x_norm, x_norm)
        x = x + attn_out

        # Convolution
        x = x + self.conv(self.norm_conv(x))

        # Feed-forward 2
        x = x + 0.5 * self.ff2(self.norm_ff2(x))

        # Final norm
        x = self.norm_out(x)

        return x

class ConvolutionModule(nn.Module):
    """Convolution module in Conformer"""

    def __init__(self, d_model, kernel_size):
        super().__init__()

        self.pointwise_conv1 = nn.Conv1d(d_model, d_model * 2, 1)
        self.depthwise_conv = nn.Conv1d(
            d_model, d_model, kernel_size,
            padding=kernel_size // 2, groups=d_model
        )
        self.batch_norm = nn.BatchNorm1d(d_model)
        self.pointwise_conv2 = nn.Conv1d(d_model, d_model, 1)

    def forward(self, x):
        # Transpose for conv1d
        x = x.transpose(1, 2)

        # GLU activation
        x = self.pointwise_conv1(x)
        x = nn.functional.glu(x, dim=1)

        # Depthwise convolution
        x = self.depthwise_conv(x)
        x = self.batch_norm(x)
        x = nn.functional.silu(x)

        # Pointwise convolution
        x = self.pointwise_conv2(x)

        # Transpose back
        return x.transpose(1, 2)

class FeedForwardModule(nn.Module):
    """Feed-forward module in Conformer"""

    def __init__(self, d_model, expansion_factor=4):
        super().__init__()

        self.linear1 = nn.Linear(d_model, d_model * expansion_factor)
        self.linear2 = nn.Linear(d_model * expansion_factor, d_model)
        self.dropout = nn.Dropout(0.1)

    def forward(self, x):
        x = self.linear1(x)
        x = nn.functional.silu(x)
        x = self.dropout(x)
        x = self.linear2(x)
        x = self.dropout(x)
        return x
```

## Summary

End-to-end ASR systems simplify the pipeline and improve performance:

- **Seq2Seq**: Direct audio-to-text mapping with attention
- **LAS**: Pyramidal encoder for efficiency
- **Transformer**: Self-attention for long-range dependencies
- **Conformer**: State-of-the-art combining convolution and attention

These systems embody 弘익人間 by making ASR more accessible and multilingual through simplified architectures.

## Review Questions

1. What advantages do E2E systems have over traditional ASR?
2. Explain attention mechanism in Seq2Seq models
3. How does pyramidal BiLSTM reduce computation?
4. What is the difference between encoder and decoder in LAS?
5. Why are Transformers effective for ASR?
6. How does Conformer combine CNN and self-attention?
7. Compare greedy decoding and beam search
8. What is the role of positional encoding?
9. How do E2E systems handle multiple languages?
10. What are the training challenges for E2E models?

---

**弘益人間** - End-to-end systems democratize ASR development, enabling better multilingual support.

---

*Previous: [Language Models](chapter4-language-models.md) | Next: [Multilingual ASR](chapter6-multilingual-asr.md)*
