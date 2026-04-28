# Chapter 6: Multilingual ASR

## Introduction

Multilingual ASR systems recognize speech in multiple languages, essential for serving 弘益人間 (Benefit All Humanity) by making technology accessible to all 7000+ world languages. This chapter explores techniques for building ASR systems that transcend linguistic boundaries.

## Challenges in Multilingual ASR

### Phonetic Diversity

Different languages have vastly different sound inventories:

```python
# Phoneme inventories vary dramatically
PHONEME_COUNTS = {
    'Hawaiian': 13,      # Smallest
    'English': 44,
    'Korean': 40,
    'Mandarin': 25,
    'Arabic': 34,
    'Russian': 42,
    '!Xóõ': 122,        # Largest (Khoisan language)
}

# Example: Language-specific phonemes
LANGUAGE_PHONEMES = {
    'Korean': {
        'ㄱ': 'k/g',
        'ㄴ': 'n',
        'ㄷ': 't/d',
        'ㄹ': 'r/l',
        'ㅁ': 'm',
        'ㅂ': 'p/b',
        'ㅅ': 's',
        'ㅇ': 'ng/silent',
        'ㅈ': 'ch/j',
        'ㅎ': 'h',
    },
    'Mandarin': {
        'b': 'b (unaspirated)',
        'p': 'p (aspirated)',
        'zh': 'retroflex j',
        'x': 'voiceless palatal fricative',
    }
}
```

### Writing Systems

Multiple scripts and orthographies:

```python
class MultiScriptProcessor:
    """Handle multiple writing systems"""

    SCRIPTS = {
        'Latin': 'English, Spanish, French, Vietnamese, etc.',
        'Cyrillic': 'Russian, Ukrainian, Bulgarian, etc.',
        'Arabic': 'Arabic, Persian, Urdu, etc.',
        'Devanagari': 'Hindi, Marathi, Nepali, etc.',
        'Han': 'Chinese, Japanese Kanji',
        'Hangul': 'Korean',
        'Hiragana/Katakana': 'Japanese',
        'Thai': 'Thai',
        'Hebrew': 'Hebrew',
    }

    def __init__(self):
        self.tokenizers = {}

    def detect_script(self, text):
        """Detect writing system"""
        import unicodedata

        scripts = {}
        for char in text:
            if char.isspace():
                continue

            script = unicodedata.name(char).split()[0]
            scripts[script] = scripts.get(script, 0) + 1

        return max(scripts.items(), key=lambda x: x[1])[0]

    def tokenize(self, text, language):
        """Language-specific tokenization"""
        script = self.detect_script(text)

        if script in ['CJK', 'Han']:
            # Character-based for Chinese
            return list(text.replace(' ', ''))
        elif language == 'Japanese':
            # Use MeCab or similar
            return self._tokenize_japanese(text)
        elif language == 'Korean':
            # Use KoNLPy or similar
            return self._tokenize_korean(text)
        else:
            # Space-based for most languages
            return text.split()
```

### Data Imbalance

Resource availability varies dramatically:

```python
LANGUAGE_RESOURCES = {
    'High-resource': {
        'languages': ['English', 'Mandarin', 'Spanish', 'French'],
        'hours': 10000+,
        'quality': 'High',
    },
    'Mid-resource': {
        'languages': ['Korean', 'Japanese', 'Arabic', 'Russian'],
        'hours': 1000-10000,
        'quality': 'Medium',
    },
    'Low-resource': {
        'languages': ['Swahili', 'Tagalog', 'Vietnamese'],
        'hours': 100-1000,
        'quality': 'Variable',
    },
    'Very-low-resource': {
        'languages': ['Indigenous languages', 'Minority languages'],
        'hours': '<100',
        'quality': 'Limited',
    }
}
```

## Multilingual Model Architectures

### Shared Encoder, Language-Specific Decoder

```python
import torch
import torch.nn as nn

class MultilingualASR(nn.Module):
    """Multilingual ASR with shared encoder"""

    def __init__(self, input_dim, languages, d_model=512):
        super().__init__()

        # Shared audio encoder
        self.encoder = nn.LSTM(
            input_dim,
            d_model,
            num_layers=6,
            bidirectional=True,
            batch_first=True
        )

        # Language-specific decoders
        self.decoders = nn.ModuleDict({
            lang: LanguageDecoder(d_model * 2, vocab_size)
            for lang, vocab_size in languages.items()
        })

        # Language identification
        self.lang_classifier = nn.Linear(d_model * 2, len(languages))

    def forward(self, features, language, targets=None):
        """
        features: (batch, time, input_dim)
        language: str, target language
        """
        # Shared encoding
        encoded, _ = self.encoder(features)

        # Language-specific decoding
        if targets is not None:
            outputs = self.decoders[language](encoded, targets)
        else:
            outputs = self.decoders[language].decode(encoded)

        return outputs

    def identify_language(self, features):
        """Identify spoken language"""
        encoded, _ = self.encoder(features)

        # Pool over time
        pooled = torch.mean(encoded, dim=1)

        # Classify
        logits = self.lang_classifier(pooled)
        return torch.softmax(logits, dim=1)

class LanguageDecoder(nn.Module):
    """Language-specific decoder"""

    def __init__(self, encoder_dim, vocab_size):
        super().__init__()

        self.attention_decoder = nn.TransformerDecoder(
            nn.TransformerDecoderLayer(encoder_dim, nhead=8),
            num_layers=6
        )

        self.embedding = nn.Embedding(vocab_size, encoder_dim)
        self.output_projection = nn.Linear(encoder_dim, vocab_size)

    def forward(self, encoded, targets):
        # Embed targets
        tgt_embedded = self.embedding(targets)

        # Decode
        decoded = self.attention_decoder(
            tgt_embedded.transpose(0, 1),
            encoded.transpose(0, 1)
        ).transpose(0, 1)

        # Project
        logits = self.output_projection(decoded)

        return torch.log_softmax(logits, dim=2)
```

### Universal Multilingual Model

Single model for all languages:

```python
class UniversalMultilingualASR(nn.Module):
    """Universal multilingual ASR"""

    def __init__(self, input_dim, vocab_size, num_languages, d_model=512):
        super().__init__()

        # Language embeddings
        self.language_embedding = nn.Embedding(num_languages, d_model)

        # Shared encoder
        self.encoder = nn.TransformerEncoder(
            nn.TransformerEncoderLayer(d_model, nhead=8),
            num_layers=12
        )

        # Input projection
        self.input_projection = nn.Linear(input_dim, d_model)

        # Shared decoder
        self.decoder = nn.TransformerDecoder(
            nn.TransformerDecoderLayer(d_model, nhead=8),
            num_layers=6
        )

        # Token embeddings (shared vocabulary)
        self.token_embedding = nn.Embedding(vocab_size, d_model)

        # Output projection
        self.output_projection = nn.Linear(d_model, vocab_size)

    def forward(self, features, language_id, targets):
        """
        features: (batch, time, input_dim)
        language_id: (batch,) language indices
        targets: (batch, target_len)
        """
        batch_size = features.size(0)

        # Project input
        x = self.input_projection(features)

        # Add language embedding
        lang_emb = self.language_embedding(language_id)
        x = x + lang_emb.unsqueeze(1)

        # Encode
        encoded = self.encoder(x.transpose(0, 1)).transpose(0, 1)

        # Embed targets
        tgt_embedded = self.token_embedding(targets)

        # Decode
        decoded = self.decoder(
            tgt_embedded.transpose(0, 1),
            encoded.transpose(0, 1)
        ).transpose(0, 1)

        # Output
        logits = self.output_projection(decoded)

        return torch.log_softmax(logits, dim=2)
```

## Cross-Lingual Transfer Learning

### Transfer from High to Low Resource

```python
class TransferLearningASR:
    """Transfer learning for low-resource languages"""

    def __init__(self, base_model, target_language):
        self.base_model = base_model
        self.target_language = target_language

    def adapt(self, target_data, strategy='fine-tuning'):
        """Adapt model to target language"""

        if strategy == 'fine-tuning':
            return self._fine_tune(target_data)
        elif strategy == 'adapter':
            return self._adapter_tuning(target_data)
        elif strategy == 'freezing':
            return self._freeze_encoder(target_data)

    def _fine_tune(self, target_data):
        """Fine-tune entire model"""
        optimizer = torch.optim.Adam(
            self.base_model.parameters(),
            lr=1e-5  # Lower learning rate
        )

        for epoch in range(10):
            for batch in target_data:
                loss = self._train_step(batch, optimizer)

        return self.base_model

    def _adapter_tuning(self, target_data):
        """Add and train adapter modules"""
        # Freeze base model
        for param in self.base_model.parameters():
            param.requires_grad = False

        # Add adapters
        adapters = self._add_adapters()

        # Train only adapters
        optimizer = torch.optim.Adam(adapters.parameters(), lr=1e-3)

        for epoch in range(20):
            for batch in target_data:
                loss = self._train_step(batch, optimizer)

        return self.base_model

    def _add_adapters(self):
        """Add adapter modules to model"""
        adapters = nn.ModuleList()

        for layer in self.base_model.encoder.layers:
            adapter = AdapterModule(layer.d_model)
            layer.add_module('adapter', adapter)
            adapters.append(adapter)

        return adapters

class AdapterModule(nn.Module):
    """Lightweight adapter for transfer learning"""

    def __init__(self, d_model, bottleneck_dim=64):
        super().__init__()

        self.down_project = nn.Linear(d_model, bottleneck_dim)
        self.up_project = nn.Linear(bottleneck_dim, d_model)
        self.activation = nn.ReLU()

    def forward(self, x):
        # Bottleneck
        h = self.down_project(x)
        h = self.activation(h)
        h = self.up_project(h)

        # Residual connection
        return x + h
```

## Language Identification

Automatically detect the spoken language:

```python
class LanguageIdentifier(nn.Module):
    """Spoken language identification"""

    def __init__(self, input_dim, num_languages):
        super().__init__()

        # Feature encoder
        self.encoder = nn.Sequential(
            nn.Conv1d(input_dim, 128, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool1d(2),

            nn.Conv1d(128, 256, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool1d(2),

            nn.Conv1d(256, 512, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.AdaptiveAvgPool1d(1)
        )

        # Classifier
        self.classifier = nn.Sequential(
            nn.Linear(512, 256),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(256, num_languages)
        )

    def forward(self, features):
        """
        features: (batch, time, input_dim)
        """
        # Transpose for Conv1d
        x = features.transpose(1, 2)

        # Encode
        x = self.encoder(x)
        x = x.squeeze(2)

        # Classify
        logits = self.classifier(x)

        return torch.softmax(logits, dim=1)

# Combined ASR with LID
class MultilingualASRWithLID(nn.Module):
    """ASR with automatic language identification"""

    def __init__(self, input_dim, languages):
        super().__init__()

        self.lid = LanguageIdentifier(input_dim, len(languages))
        self.asr = UniversalMultilingualASR(
            input_dim,
            vocab_size=50000,
            num_languages=len(languages)
        )

        self.languages = languages

    def forward(self, features, language_id=None):
        """Automatic language detection and recognition"""

        if language_id is None:
            # Detect language
            lang_probs = self.lid(features)
            language_id = torch.argmax(lang_probs, dim=1)

        # Recognize
        # Note: In training, targets would be provided
        return self.asr.encode(features, language_id)
```

## Code-Switching

Handle mixed-language speech:

```python
class CodeSwitchingASR(nn.Module):
    """ASR for code-switching scenarios"""

    def __init__(self, input_dim, languages, vocab_size):
        super().__init__()

        # Shared encoder
        self.encoder = nn.LSTM(
            input_dim, 512, num_layers=6,
            bidirectional=True, batch_first=True
        )

        # Frame-level language detector
        self.frame_lid = nn.Linear(512 * 2, len(languages))

        # Shared decoder
        self.decoder = nn.TransformerDecoder(
            nn.TransformerDecoderLayer(512 * 2, nhead=8),
            num_layers=6
        )

        # Token embeddings
        self.token_embedding = nn.Embedding(vocab_size, 512 * 2)

        # Output projection
        self.output_projection = nn.Linear(512 * 2, vocab_size)

    def forward(self, features, targets=None):
        # Encode
        encoded, _ = self.encoder(features)

        # Detect language at each frame
        frame_lang_logits = self.frame_lid(encoded)
        frame_lang_probs = torch.softmax(frame_lang_logits, dim=2)

        # Decode with language information
        if targets is not None:
            tgt_embedded = self.token_embedding(targets)

            # Add language information
            tgt_embedded = tgt_embedded + frame_lang_probs.mean(dim=1, keepdim=True)

            decoded = self.decoder(
                tgt_embedded.transpose(0, 1),
                encoded.transpose(0, 1)
            ).transpose(0, 1)

            logits = self.output_projection(decoded)

            return torch.log_softmax(logits, dim=2), frame_lang_probs

        return None, frame_lang_probs
```

## Multilingual Training Strategies

### Data Sampling

Balance languages during training:

```python
class MultilingualDataSampler:
    """Sample data from multiple languages"""

    def __init__(self, datasets, sampling_strategy='proportional'):
        """
        datasets: Dict[language -> dataset]
        sampling_strategy: 'proportional', 'uniform', 'temperature'
        """
        self.datasets = datasets
        self.sampling_strategy = sampling_strategy

        # Calculate sampling probabilities
        self.probs = self._calculate_sampling_probs()

    def _calculate_sampling_probs(self):
        """Calculate sampling probability for each language"""
        sizes = {lang: len(ds) for lang, ds in self.datasets.items()}
        total = sum(sizes.values())

        if self.sampling_strategy == 'proportional':
            # Proportional to dataset size
            return {lang: size / total for lang, size in sizes.items()}

        elif self.sampling_strategy == 'uniform':
            # Equal probability for all languages
            num_langs = len(self.datasets)
            return {lang: 1.0 / num_langs for lang in self.datasets}

        elif self.sampling_strategy == 'temperature':
            # Temperature-based sampling (boost low-resource)
            import numpy as np
            T = 0.5  # Temperature

            sizes_arr = np.array(list(sizes.values()))
            probs = sizes_arr ** (1.0 / T)
            probs = probs / np.sum(probs)

            return {lang: p for lang, p in zip(self.datasets.keys(), probs)}

    def sample_batch(self, batch_size):
        """Sample a batch from multiple languages"""
        # Sample language
        import random
        language = random.choices(
            list(self.datasets.keys()),
            weights=list(self.probs.values())
        )[0]

        # Sample batch from that language
        dataset = self.datasets[language]
        indices = random.sample(range(len(dataset)), batch_size)
        batch = [dataset[i] for i in indices]

        return batch, language
```

## Evaluation

Multilingual ASR evaluation:

```python
def evaluate_multilingual_asr(model, test_sets, languages):
    """Evaluate on multiple languages"""
    results = {}

    for lang in languages:
        wer_total = 0
        num_samples = 0

        for features, targets in test_sets[lang]:
            # Recognize
            predictions = model(features, language_id=lang)

            # Calculate WER
            wer = calculate_wer(predictions, targets)
            wer_total += wer
            num_samples += 1

        results[lang] = {
            'WER': wer_total / num_samples,
            'num_samples': num_samples
        }

    # Calculate average
    avg_wer = sum(r['WER'] for r in results.values()) / len(results)
    results['average'] = avg_wer

    return results

def calculate_wer(hypothesis, reference):
    """Calculate Word Error Rate"""
    import editdistance

    # Tokenize
    hyp_words = hypothesis.split()
    ref_words = reference.split()

    # Edit distance
    distance = editdistance.eval(hyp_words, ref_words)

    # WER
    wer = distance / len(ref_words) if len(ref_words) > 0 else 0

    return wer
```

## Summary

Multilingual ASR extends speech recognition to all languages:

- **Shared Encoders**: Universal audio representations
- **Language-Specific Decoders**: Handle unique characteristics
- **Transfer Learning**: Leverage high-resource languages
- **Language Identification**: Automatic language detection
- **Code-Switching**: Mixed-language recognition

These techniques embody 弘益人間 by making ASR accessible to speakers of all languages worldwide.

## Review Questions

1. What are the main challenges in multilingual ASR?
2. Compare shared vs. language-specific components
3. How does transfer learning help low-resource languages?
4. What is code-switching and why is it challenging?
5. Explain temperature-based sampling
6. How do adapters enable efficient transfer learning?
7. What is the role of language identification?
8. Compare proportional and uniform sampling strategies
9. How do you evaluate multilingual systems fairly?
10. What are the benefits of universal multilingual models?

---

**弘益人間** - Multilingual ASR ensures technology serves all humanity, transcending linguistic barriers.

---

*Previous: [End-to-End Systems](chapter5-end-to-end-systems.md) | Next: [Streaming ASR](chapter7-streaming-asr.md)*
