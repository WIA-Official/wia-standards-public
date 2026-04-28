# Chapter 4: Language Models

## Introduction to Language Modeling

Language models predict the probability of word sequences, essential for disambiguating acoustically similar phrases and improving ASR accuracy. This chapter explores statistical and neural language modeling techniques that enable speech recognition systems to understand context and produce coherent transcriptions.

In alignment with 弘益人間 (Benefit All Humanity), language models must handle diverse linguistic structures across all languages and domains, ensuring accurate recognition for everyone.

## N-gram Language Models

### Fundamentals

N-gram models estimate probabilities based on word history:

```python
from collections import defaultdict
import math

class NgramLanguageModel:
    def __init__(self, n=3):
        """
        Initialize n-gram language model

        n: Order of n-gram (1=unigram, 2=bigram, 3=trigram, etc.)
        """
        self.n = n
        self.counts = defaultdict(int)
        self.context_counts = defaultdict(int)
        self.vocab = set()
        self.total_words = 0

    def train(self, corpus):
        """
        Train on text corpus

        corpus: List of sentences (each sentence is a list of words)
        """
        for sentence in corpus:
            # Add start/end tokens
            words = ['<s>'] * (self.n - 1) + sentence + ['</s>']

            self.vocab.update(words)
            self.total_words += len(sentence)

            # Count n-grams
            for i in range(len(words) - self.n + 1):
                ngram = tuple(words[i:i + self.n])
                context = ngram[:-1]

                self.counts[ngram] += 1
                if len(context) > 0:
                    self.context_counts[context] += 1

    def probability(self, word, context):
        """
        Calculate P(word | context)

        context: Tuple of previous (n-1) words
        """
        if len(context) != self.n - 1:
            raise ValueError(f"Context must have {self.n-1} words")

        ngram = context + (word,)

        if self.n == 1:
            # Unigram: P(word)
            return self.counts[(word,)] / self.total_words
        else:
            # N-gram: P(word | context)
            context_count = self.context_counts[context]
            if context_count == 0:
                return 0.0

            return self.counts[ngram] / context_count

    def log_probability(self, word, context):
        """Calculate log P(word | context)"""
        prob = self.probability(word, context)
        return math.log(prob) if prob > 0 else float('-inf')

    def sentence_probability(self, sentence):
        """Calculate probability of entire sentence"""
        words = ['<s>'] * (self.n - 1) + sentence + ['</s>']

        log_prob = 0.0
        for i in range(self.n - 1, len(words)):
            context = tuple(words[i - self.n + 1:i])
            word = words[i]

            log_prob += self.log_probability(word, context)

        return math.exp(log_prob)

    def perplexity(self, test_corpus):
        """
        Calculate perplexity on test set

        Lower perplexity = better model
        """
        total_log_prob = 0.0
        total_words = 0

        for sentence in test_corpus:
            words = ['<s>'] * (self.n - 1) + sentence + ['</s>']

            for i in range(self.n - 1, len(words)):
                context = tuple(words[i - self.n + 1:i])
                word = words[i]

                total_log_prob += self.log_probability(word, context)
                total_words += 1

        avg_log_prob = total_log_prob / total_words
        return math.exp(-avg_log_prob)

# Example usage
corpus = [
    ['the', 'cat', 'sat', 'on', 'the', 'mat'],
    ['the', 'dog', 'sat', 'on', 'the', 'log'],
    ['the', 'cat', 'and', 'dog', 'played'],
]

lm = NgramLanguageModel(n=3)
lm.train(corpus)

# Query probability
context = ('the', 'cat')
word = 'sat'
prob = lm.probability(word, context)
print(f"P({word} | {context}) = {prob:.4f}")

# Sentence probability
sentence = ['the', 'cat', 'sat']
prob = lm.sentence_probability(sentence)
print(f"P(sentence) = {prob:.6f}")
```

### Smoothing Techniques

Handling unseen n-grams:

```python
class SmoothedNgramLM(NgramLanguageModel):
    """N-gram LM with smoothing"""

    def __init__(self, n=3, smoothing='add-k', k=1.0):
        super().__init__(n)
        self.smoothing = smoothing
        self.k = k

    def probability(self, word, context):
        """Calculate smoothed probability"""
        if self.smoothing == 'add-k':
            return self._additive_smoothing(word, context)
        elif self.smoothing == 'kneser-ney':
            return self._kneser_ney(word, context)
        else:
            return super().probability(word, context)

    def _additive_smoothing(self, word, context):
        """
        Add-k smoothing (Laplace smoothing when k=1)

        P(word | context) = (count(context, word) + k) /
                           (count(context) + k * |V|)
        """
        ngram = context + (word,)
        vocab_size = len(self.vocab)

        numerator = self.counts[ngram] + self.k
        denominator = self.context_counts[context] + self.k * vocab_size

        return numerator / denominator if denominator > 0 else 0.0

    def _kneser_ney(self, word, context):
        """
        Kneser-Ney smoothing (simplified)

        Uses absolute discounting and continuation probability
        """
        discount = 0.75
        ngram = context + (word,)

        # Absolute discounting
        ngram_count = max(self.counts[ngram] - discount, 0)
        context_count = self.context_counts[context]

        if context_count == 0:
            # Backoff to lower-order model
            if len(context) > 0:
                return self.probability(word, context[1:])
            else:
                return 1.0 / len(self.vocab)

        # Number of unique words following context
        unique_following = sum(
            1 for ng in self.counts
            if ng[:-1] == context and self.counts[ng] > 0
        )

        # Interpolation weight
        lambda_weight = (discount * unique_following) / context_count

        # Continuation probability
        continuation = self._continuation_prob(word)

        return (ngram_count / context_count) + lambda_weight * continuation

    def _continuation_prob(self, word):
        """Calculate continuation probability for Kneser-Ney"""
        # Number of unique contexts word appears in
        unique_contexts = sum(
            1 for ng in self.counts
            if ng[-1] == word and self.counts[ng] > 0
        )

        # Total number of unique bigrams
        total_unique = len([ng for ng in self.counts if len(ng) == 2])

        return unique_contexts / total_unique if total_unique > 0 else 0.0
```

### Backoff Models

Backing off to lower-order n-grams for unseen sequences:

```python
class BackoffNgramLM:
    """N-gram language model with backoff"""

    def __init__(self, max_n=3):
        self.max_n = max_n
        self.models = {n: NgramLanguageModel(n) for n in range(1, max_n + 1)}
        self.backoff_weights = {}

    def train(self, corpus):
        """Train all n-gram orders"""
        for n in range(1, self.max_n + 1):
            self.models[n].train(corpus)

        self._calculate_backoff_weights()

    def _calculate_backoff_weights(self):
        """Calculate backoff weights using Good-Turing"""
        # Simplified: uniform backoff weights
        for n in range(2, self.max_n + 1):
            self.backoff_weights[n] = 0.4  # Default backoff weight

    def probability(self, word, context):
        """
        Calculate probability with backoff

        Try highest order first, back off if needed
        """
        # Start with longest context
        n = min(len(context) + 1, self.max_n)

        while n > 0:
            if n == 1:
                # Unigram (always available)
                return self.models[1].probability(word, ())

            # Try n-gram
            ctx = context[-(n-1):] if len(context) >= n-1 else context
            prob = self.models[n].probability(word, ctx)

            if prob > 0:
                return prob
            else:
                # Back off to lower order
                backoff_weight = self.backoff_weights.get(n, 0.4)
                n -= 1

        return 1.0 / len(self.models[1].vocab)
```

## Neural Language Models

### Feedforward Neural Networks

Simple neural LMs with fixed context:

```python
import torch
import torch.nn as nn

class FeedForwardNLM(nn.Module):
    """Feedforward Neural Language Model"""

    def __init__(self, vocab_size, embedding_dim, context_size, hidden_dim):
        """
        vocab_size: Size of vocabulary
        embedding_dim: Dimension of word embeddings
        context_size: Number of context words
        hidden_dim: Hidden layer dimension
        """
        super().__init__()

        self.context_size = context_size

        # Word embeddings
        self.embeddings = nn.Embedding(vocab_size, embedding_dim)

        # Hidden layers
        self.fc1 = nn.Linear(context_size * embedding_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, vocab_size)

        self.relu = nn.ReLU()
        self.dropout = nn.Dropout(0.2)

    def forward(self, context):
        """
        context: (batch_size, context_size) word indices
        returns: (batch_size, vocab_size) log probabilities
        """
        # Embed context words
        embeds = self.embeddings(context)  # (batch, context, embed_dim)

        # Flatten
        embeds = embeds.view(embeds.size(0), -1)  # (batch, context*embed_dim)

        # Hidden layers
        h1 = self.dropout(self.relu(self.fc1(embeds)))
        h2 = self.dropout(self.relu(self.fc2(h1)))

        # Output
        logits = self.fc3(h2)

        return torch.log_softmax(logits, dim=1)

# Training
def train_neural_lm(model, train_data, vocab, epochs=10):
    """Train neural language model"""
    optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
    criterion = nn.NLLLoss()

    for epoch in range(epochs):
        total_loss = 0

        for context, target in train_data:
            # Convert to indices
            context_ids = torch.tensor([vocab[w] for w in context])
            target_id = torch.tensor([vocab[target]])

            # Forward pass
            log_probs = model(context_ids.unsqueeze(0))
            loss = criterion(log_probs, target_id)

            # Backward pass
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            total_loss += loss.item()

        print(f"Epoch {epoch+1}: Loss = {total_loss/len(train_data):.4f}")
```

### Recurrent Neural Networks

RNNs handle variable-length context:

```python
class RNNLanguageModel(nn.Module):
    """RNN-based Language Model"""

    def __init__(self, vocab_size, embedding_dim, hidden_dim, num_layers=2):
        super().__init__()

        self.embedding = nn.Embedding(vocab_size, embedding_dim)

        self.rnn = nn.LSTM(
            embedding_dim,
            hidden_dim,
            num_layers,
            batch_first=True,
            dropout=0.2
        )

        self.fc = nn.Linear(hidden_dim, vocab_size)

    def forward(self, input_seq, hidden=None):
        """
        input_seq: (batch_size, seq_len) word indices
        hidden: Previous hidden state
        returns: (batch_size, seq_len, vocab_size) log probabilities
        """
        # Embed
        embeds = self.embedding(input_seq)

        # RNN
        output, hidden = self.rnn(embeds, hidden)

        # Project to vocabulary
        logits = self.fc(output)

        return torch.log_softmax(logits, dim=2), hidden

    def generate(self, start_token, max_length=50, temperature=1.0):
        """Generate text"""
        self.eval()

        generated = [start_token]
        hidden = None

        with torch.no_grad():
            for _ in range(max_length):
                # Get last token
                input_token = torch.tensor([[generated[-1]]])

                # Forward pass
                log_probs, hidden = self.forward(input_token, hidden)

                # Sample from distribution
                probs = torch.exp(log_probs[0, -1] / temperature)
                next_token = torch.multinomial(probs, 1).item()

                if next_token == 0:  # End token
                    break

                generated.append(next_token)

        return generated
```

### Transformer Language Models

Self-attention for long-range dependencies:

```python
class TransformerLM(nn.Module):
    """Transformer-based Language Model"""

    def __init__(self, vocab_size, d_model=512, nhead=8, num_layers=6,
                 dim_feedforward=2048, max_seq_len=512):
        super().__init__()

        self.d_model = d_model
        self.max_seq_len = max_seq_len

        # Token embedding
        self.embedding = nn.Embedding(vocab_size, d_model)

        # Positional encoding
        self.pos_encoding = PositionalEncoding(d_model, max_seq_len)

        # Transformer decoder layers
        decoder_layer = nn.TransformerDecoderLayer(
            d_model, nhead, dim_feedforward, dropout=0.1
        )
        self.transformer = nn.TransformerDecoder(decoder_layer, num_layers)

        # Output projection
        self.fc = nn.Linear(d_model, vocab_size)

    def forward(self, src, tgt_mask=None):
        """
        src: (batch_size, seq_len) input tokens
        """
        # Embed and add positional encoding
        src = self.embedding(src) * math.sqrt(self.d_model)
        src = self.pos_encoding(src)

        # Create causal mask
        if tgt_mask is None:
            tgt_mask = self._generate_square_subsequent_mask(src.size(1))

        # Transformer
        output = self.transformer(src, src, tgt_mask=tgt_mask)

        # Project to vocabulary
        logits = self.fc(output)

        return torch.log_softmax(logits, dim=2)

    def _generate_square_subsequent_mask(self, sz):
        """Generate causal attention mask"""
        mask = torch.triu(torch.ones(sz, sz), diagonal=1)
        mask = mask.masked_fill(mask == 1, float('-inf'))
        return mask

class PositionalEncoding(nn.Module):
    """Positional encoding for Transformer"""

    def __init__(self, d_model, max_len=5000):
        super().__init__()

        # Create positional encoding matrix
        pe = torch.zeros(max_len, d_model)
        position = torch.arange(0, max_len, dtype=torch.float).unsqueeze(1)
        div_term = torch.exp(
            torch.arange(0, d_model, 2).float() * (-math.log(10000.0) / d_model)
        )

        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)

        pe = pe.unsqueeze(0)
        self.register_buffer('pe', pe)

    def forward(self, x):
        """Add positional encoding to input"""
        return x + self.pe[:, :x.size(1)]
```

## Domain Adaptation

Adapting language models to specific domains:

```python
class DomainAdaptedLM:
    """Domain adaptation for language models"""

    def __init__(self, base_model, adaptation_method='interpolation'):
        self.base_model = base_model
        self.domain_model = None
        self.adaptation_method = adaptation_method
        self.interpolation_weight = 0.5

    def adapt(self, domain_corpus):
        """Adapt to domain using small corpus"""
        if self.adaptation_method == 'interpolation':
            self._interpolation_adaptation(domain_corpus)
        elif self.adaptation_method == 'fine-tuning':
            self._fine_tuning_adaptation(domain_corpus)

    def _interpolation_adaptation(self, domain_corpus):
        """Linear interpolation of base and domain models"""
        # Train domain-specific model
        self.domain_model = NgramLanguageModel(n=3)
        self.domain_model.train(domain_corpus)

    def probability(self, word, context):
        """Combined probability"""
        base_prob = self.base_model.probability(word, context)

        if self.domain_model:
            domain_prob = self.domain_model.probability(word, context)
            return (1 - self.interpolation_weight) * base_prob + \
                   self.interpolation_weight * domain_prob
        else:
            return base_prob

    def _fine_tuning_adaptation(self, domain_corpus):
        """Fine-tune neural model on domain data"""
        # For neural models
        optimizer = torch.optim.Adam(self.base_model.parameters(), lr=0.0001)
        criterion = nn.NLLLoss()

        for epoch in range(5):  # Few epochs for adaptation
            for batch in domain_corpus:
                # Training step
                loss = self._train_step(batch, criterion, optimizer)
```

## Evaluation Metrics

### Perplexity

Standard metric for language models:

```python
def calculate_perplexity(model, test_data):
    """
    Calculate perplexity on test set

    Lower is better
    """
    total_log_prob = 0.0
    total_words = 0

    model.eval()
    with torch.no_grad():
        for sentence in test_data:
            log_probs, _ = model(sentence)

            for i in range(len(sentence) - 1):
                total_log_prob += log_probs[0, i, sentence[i+1]].item()
                total_words += 1

    avg_log_prob = total_log_prob / total_words
    perplexity = math.exp(-avg_log_prob)

    return perplexity
```

## Summary

Language models are essential for accurate speech recognition:

- **N-grams**: Simple, effective statistical models
- **Smoothing**: Handling unseen sequences
- **Neural LMs**: Deep learning for better context modeling
- **Transformers**: State-of-the-art with self-attention
- **Domain Adaptation**: Customizing for specific applications

By combining acoustic and language models, ASR systems achieve high accuracy across diverse domains, serving 弘益人間 by enabling natural communication for all.

## Review Questions

1. What is the difference between bigram and trigram models?
2. Why is smoothing necessary in n-gram models?
3. Explain Kneser-Ney smoothing.
4. How do neural LMs differ from n-gram models?
5. What advantage do Transformers have over RNNs?
6. What is perplexity and how is it calculated?
7. How does domain adaptation improve ASR accuracy?
8. Explain the backoff strategy in language modeling.
9. What is the purpose of positional encoding in Transformers?
10. How do you handle out-of-vocabulary words?

## Practical Exercises

1. **N-gram Training**: Build trigram model on large corpus
2. **Smoothing Comparison**: Compare different smoothing techniques
3. **Neural LM**: Implement and train RNN language model
4. **Perplexity Analysis**: Evaluate models on different domains
5. **Domain Adaptation**: Adapt general LM to medical domain

---

**弘益人間 (Hongik Ingan)** - *Benefit All Humanity*

Language models enable ASR systems to understand context and produce coherent transcriptions, making technology more natural and accessible for all languages and cultures.

---

*Previous: [Acoustic Models](chapter3-acoustic-models.md) | Next: [End-to-End Systems](chapter5-end-to-end-systems.md)*
