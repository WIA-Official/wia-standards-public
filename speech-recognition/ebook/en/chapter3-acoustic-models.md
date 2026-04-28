# Chapter 3: Acoustic Models

## Introduction to Acoustic Modeling

Acoustic models are the heart of speech recognition systems, learning the complex relationship between audio signals and linguistic units (phonemes, characters, or words). This chapter explores traditional and modern acoustic modeling techniques that enable accurate speech-to-text conversion across diverse languages and speakers.

In service of 弘益人間 (Benefit All Humanity), acoustic models must be robust, adaptable, and capable of handling the rich diversity of human speech across all languages and dialects.

## Hidden Markov Models (HMMs)

### HMM Fundamentals

Hidden Markov Models dominated ASR for decades, modeling speech as a sequence of states with probabilistic transitions:

```python
import numpy as np

class HiddenMarkovModel:
    def __init__(self, n_states, n_observations):
        """
        Initialize HMM

        n_states: Number of hidden states
        n_observations: Number of observable symbols
        """
        self.n_states = n_states
        self.n_observations = n_observations

        # Initialize parameters randomly
        self.initial_prob = np.random.dirichlet(np.ones(n_states))
        self.transition_prob = np.random.dirichlet(np.ones(n_states), n_states)
        self.emission_prob = np.random.dirichlet(np.ones(n_observations), n_states)

    def forward(self, observations):
        """
        Forward algorithm: compute P(observations|model)

        Returns: (alpha, log_prob)
        """
        T = len(observations)
        alpha = np.zeros((T, self.n_states))

        # Initialization
        alpha[0, :] = self.initial_prob * self.emission_prob[:, observations[0]]

        # Recursion
        for t in range(1, T):
            for j in range(self.n_states):
                alpha[t, j] = np.sum(alpha[t-1, :] * self.transition_prob[:, j]) * \
                             self.emission_prob[j, observations[t]]

        # Termination
        log_prob = np.log(np.sum(alpha[-1, :]) + 1e-10)

        return alpha, log_prob

    def backward(self, observations):
        """
        Backward algorithm

        Returns: beta
        """
        T = len(observations)
        beta = np.zeros((T, self.n_states))

        # Initialization
        beta[-1, :] = 1.0

        # Recursion
        for t in range(T-2, -1, -1):
            for i in range(self.n_states):
                beta[t, i] = np.sum(
                    self.transition_prob[i, :] *
                    self.emission_prob[:, observations[t+1]] *
                    beta[t+1, :]
                )

        return beta

    def viterbi(self, observations):
        """
        Viterbi algorithm: find most likely state sequence

        Returns: (states, log_prob)
        """
        T = len(observations)
        delta = np.zeros((T, self.n_states))
        psi = np.zeros((T, self.n_states), dtype=int)

        # Initialization
        delta[0, :] = np.log(self.initial_prob + 1e-10) + \
                     np.log(self.emission_prob[:, observations[0]] + 1e-10)

        # Recursion
        for t in range(1, T):
            for j in range(self.n_states):
                # Find best previous state
                values = delta[t-1, :] + np.log(self.transition_prob[:, j] + 1e-10)
                psi[t, j] = np.argmax(values)
                delta[t, j] = np.max(values) + \
                             np.log(self.emission_prob[j, observations[t]] + 1e-10)

        # Backtrack
        states = np.zeros(T, dtype=int)
        states[-1] = np.argmax(delta[-1, :])

        for t in range(T-2, -1, -1):
            states[t] = psi[t+1, states[t+1]]

        log_prob = np.max(delta[-1, :])

        return states, log_prob

    def baum_welch(self, observations, max_iter=100, tolerance=1e-6):
        """
        Baum-Welch algorithm (EM) for training HMM

        observations: List of observation sequences
        """
        prev_log_prob = -np.inf

        for iteration in range(max_iter):
            # E-step: compute expected counts
            total_initial = np.zeros(self.n_states)
            total_transition = np.zeros((self.n_states, self.n_states))
            total_emission = np.zeros((self.n_states, self.n_observations))

            total_log_prob = 0

            for obs in observations:
                alpha, log_prob = self.forward(obs)
                beta = self.backward(obs)
                total_log_prob += log_prob

                T = len(obs)

                # Compute gamma (state probabilities)
                gamma = alpha * beta
                gamma /= np.sum(gamma, axis=1, keepdims=True)

                # Compute xi (transition probabilities)
                xi = np.zeros((T-1, self.n_states, self.n_states))
                for t in range(T-1):
                    for i in range(self.n_states):
                        for j in range(self.n_states):
                            xi[t, i, j] = alpha[t, i] * \
                                         self.transition_prob[i, j] * \
                                         self.emission_prob[j, obs[t+1]] * \
                                         beta[t+1, j]
                    xi[t, :, :] /= np.sum(xi[t, :, :])

                # Accumulate counts
                total_initial += gamma[0, :]

                for i in range(self.n_states):
                    for j in range(self.n_states):
                        total_transition[i, j] += np.sum(xi[:, i, j])

                for t in range(T):
                    total_emission[:, obs[t]] += gamma[t, :]

            # M-step: update parameters
            self.initial_prob = total_initial / len(observations)
            self.transition_prob = total_transition / \
                                  np.sum(total_transition, axis=1, keepdims=True)
            self.emission_prob = total_emission / \
                                np.sum(total_emission, axis=1, keepdims=True)

            # Check convergence
            avg_log_prob = total_log_prob / len(observations)
            print(f"Iteration {iteration+1}: Log Prob = {avg_log_prob:.4f}")

            if avg_log_prob - prev_log_prob < tolerance:
                print("Converged!")
                break

            prev_log_prob = avg_log_prob
```

### GMM-HMM Systems

Gaussian Mixture Models extend HMMs to continuous observations:

```python
from scipy.stats import multivariate_normal

class GaussianMixtureModel:
    def __init__(self, n_components, n_features):
        """GMM for acoustic modeling"""
        self.n_components = n_components
        self.n_features = n_features

        # Initialize parameters
        self.weights = np.ones(n_components) / n_components
        self.means = np.random.randn(n_components, n_features)
        self.covariances = np.array([np.eye(n_features) for _ in range(n_components)])

    def pdf(self, x):
        """Compute probability density"""
        probs = np.zeros(self.n_components)

        for k in range(self.n_components):
            probs[k] = self.weights[k] * multivariate_normal.pdf(
                x, mean=self.means[k], cov=self.covariances[k]
            )

        return np.sum(probs)

    def log_likelihood(self, data):
        """Compute log-likelihood of data"""
        return np.sum([np.log(self.pdf(x) + 1e-10) for x in data])

    def em_step(self, data):
        """One EM iteration"""
        N = len(data)

        # E-step: compute responsibilities
        responsibilities = np.zeros((N, self.n_components))

        for i, x in enumerate(data):
            for k in range(self.n_components):
                responsibilities[i, k] = self.weights[k] * multivariate_normal.pdf(
                    x, mean=self.means[k], cov=self.covariances[k]
                )

        responsibilities /= np.sum(responsibilities, axis=1, keepdims=True)

        # M-step: update parameters
        Nk = np.sum(responsibilities, axis=0)

        self.weights = Nk / N

        for k in range(self.n_components):
            self.means[k] = np.sum(responsibilities[:, k:k+1] * data, axis=0) / Nk[k]

            diff = data - self.means[k]
            self.covariances[k] = np.dot(
                (responsibilities[:, k:k+1] * diff).T, diff
            ) / Nk[k]

class GMMHMM:
    """GMM-HMM for continuous speech recognition"""

    def __init__(self, n_states, n_components, n_features):
        self.n_states = n_states
        self.hmm = HiddenMarkovModel(n_states, 0)  # Placeholder
        self.gmms = [GaussianMixtureModel(n_components, n_features)
                     for _ in range(n_states)]

    def score(self, features, state):
        """Compute acoustic score for state given features"""
        return self.gmms[state].pdf(features)

    def train(self, feature_sequences, state_sequences):
        """Train GMM-HMM"""
        # Train each GMM separately
        for state in range(self.n_states):
            # Collect features for this state
            state_features = []

            for features, states in zip(feature_sequences, state_sequences):
                state_mask = (states == state)
                state_features.extend(features[state_mask])

            # Train GMM
            state_features = np.array(state_features)
            for _ in range(10):  # EM iterations
                self.gmms[state].em_step(state_features)
```

## Deep Neural Network Acoustic Models

### DNN-HMM Hybrid

Deep Neural Networks replace GMMs for better modeling:

```python
import torch
import torch.nn as nn

class DNNAcousticModel(nn.Module):
    """Deep Neural Network for acoustic modeling"""

    def __init__(self, input_dim, hidden_dims, output_dim):
        """
        input_dim: Feature dimension (e.g., 40 for MFCCs)
        hidden_dims: List of hidden layer dimensions
        output_dim: Number of phoneme states
        """
        super().__init__()

        layers = []
        prev_dim = input_dim

        for hidden_dim in hidden_dims:
            layers.extend([
                nn.Linear(prev_dim, hidden_dim),
                nn.ReLU(),
                nn.BatchNorm1d(hidden_dim),
                nn.Dropout(0.2)
            ])
            prev_dim = hidden_dim

        layers.append(nn.Linear(prev_dim, output_dim))
        layers.append(nn.LogSoftmax(dim=1))

        self.network = nn.Sequential(*layers)

    def forward(self, features):
        """
        features: (batch_size, feature_dim)
        returns: (batch_size, num_states) log probabilities
        """
        return self.network(features)

# Training DNN acoustic model
class DNNTrainer:
    def __init__(self, model, learning_rate=0.001):
        self.model = model
        self.optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
        self.criterion = nn.NLLLoss()

    def train_epoch(self, train_loader):
        """Train for one epoch"""
        self.model.train()
        total_loss = 0

        for features, labels in train_loader:
            # Forward pass
            outputs = self.model(features)
            loss = self.criterion(outputs, labels)

            # Backward pass
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()

            total_loss += loss.item()

        return total_loss / len(train_loader)

    def evaluate(self, val_loader):
        """Evaluate on validation set"""
        self.model.eval()
        correct = 0
        total = 0

        with torch.no_grad():
            for features, labels in val_loader:
                outputs = self.model(features)
                predictions = torch.argmax(outputs, dim=1)

                correct += (predictions == labels).sum().item()
                total += labels.size(0)

        return correct / total
```

### Time-Delay Neural Networks (TDNN)

TDNNs capture temporal context:

```python
class TDNNLayer(nn.Module):
    """Time-Delay Neural Network Layer"""

    def __init__(self, input_dim, output_dim, context_size=5):
        super().__init__()
        self.context_size = context_size
        self.conv = nn.Conv1d(
            input_dim,
            output_dim,
            kernel_size=context_size,
            padding=context_size // 2
        )
        self.bn = nn.BatchNorm1d(output_dim)
        self.relu = nn.ReLU()

    def forward(self, x):
        """
        x: (batch_size, input_dim, time_steps)
        """
        return self.relu(self.bn(self.conv(x)))

class TDNNAcousticModel(nn.Module):
    """TDNN for acoustic modeling"""

    def __init__(self, input_dim, output_dim):
        super().__init__()

        self.tdnn1 = TDNNLayer(input_dim, 512, context_size=5)
        self.tdnn2 = TDNNLayer(512, 512, context_size=3)
        self.tdnn3 = TDNNLayer(512, 512, context_size=3)
        self.tdnn4 = TDNNLayer(512, 512, context_size=1)
        self.tdnn5 = TDNNLayer(512, 1500, context_size=1)

        # Statistics pooling
        self.stats_pool = StatsPooling()

        # Output layers
        self.fc1 = nn.Linear(3000, 512)
        self.bn = nn.BatchNorm1d(512)
        self.fc2 = nn.Linear(512, output_dim)

    def forward(self, x):
        """
        x: (batch_size, feature_dim, time_steps)
        """
        x = self.tdnn1(x)
        x = self.tdnn2(x)
        x = self.tdnn3(x)
        x = self.tdnn4(x)
        x = self.tdnn5(x)

        # Pool over time
        x = self.stats_pool(x)

        # Output layers
        x = self.fc1(x)
        x = self.bn(x)
        x = torch.relu(x)
        x = self.fc2(x)

        return torch.log_softmax(x, dim=1)

class StatsPooling(nn.Module):
    """Statistical pooling layer"""

    def forward(self, x):
        """
        x: (batch_size, channels, time_steps)
        returns: (batch_size, 2*channels)
        """
        mean = torch.mean(x, dim=2)
        std = torch.std(x, dim=2)
        return torch.cat([mean, std], dim=1)
```

## Recurrent Neural Networks

### LSTM Acoustic Models

Long Short-Term Memory networks handle sequential dependencies:

```python
class LSTMAcousticModel(nn.Module):
    """LSTM-based acoustic model"""

    def __init__(self, input_dim, hidden_dim, num_layers, output_dim):
        super().__init__()

        self.lstm = nn.LSTM(
            input_dim,
            hidden_dim,
            num_layers,
            batch_first=True,
            bidirectional=True,
            dropout=0.2
        )

        self.fc = nn.Linear(hidden_dim * 2, output_dim)  # *2 for bidirectional

    def forward(self, features, lengths=None):
        """
        features: (batch_size, time_steps, feature_dim)
        lengths: (batch_size,) actual sequence lengths
        """
        if lengths is not None:
            # Pack padded sequence
            packed = nn.utils.rnn.pack_padded_sequence(
                features, lengths, batch_first=True, enforce_sorted=False
            )
            outputs, _ = self.lstm(packed)
            outputs, _ = nn.utils.rnn.pad_packed_sequence(
                outputs, batch_first=True
            )
        else:
            outputs, _ = self.lstm(features)

        # Project to output dimension
        logits = self.fc(outputs)

        return torch.log_softmax(logits, dim=2)

### Bidirectional LSTM with Attention

class BLSTMAttention(nn.Module):
    """Bidirectional LSTM with attention mechanism"""

    def __init__(self, input_dim, hidden_dim, num_layers, output_dim):
        super().__init__()

        self.blstm = nn.LSTM(
            input_dim,
            hidden_dim,
            num_layers,
            batch_first=True,
            bidirectional=True
        )

        self.attention = nn.MultiheadAttention(
            hidden_dim * 2,
            num_heads=8,
            batch_first=True
        )

        self.fc = nn.Linear(hidden_dim * 2, output_dim)

    def forward(self, features):
        # LSTM encoding
        lstm_out, _ = self.blstm(features)

        # Self-attention
        attended, _ = self.attention(lstm_out, lstm_out, lstm_out)

        # Residual connection
        combined = lstm_out + attended

        # Output projection
        logits = self.fc(combined)

        return torch.log_softmax(logits, dim=2)
```

## Convolutional Neural Networks

### CNN Acoustic Features

CNNs extract hierarchical features:

```python
class CNNAcousticModel(nn.Module):
    """CNN for acoustic modeling"""

    def __init__(self, input_height, output_dim):
        """
        input_height: Feature dimension (e.g., 80 for mel spectrogram)
        """
        super().__init__()

        self.conv_layers = nn.Sequential(
            # Conv block 1
            nn.Conv2d(1, 64, kernel_size=3, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(),
            nn.MaxPool2d(2),

            # Conv block 2
            nn.Conv2d(64, 128, kernel_size=3, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(),
            nn.MaxPool2d(2),

            # Conv block 3
            nn.Conv2d(128, 256, kernel_size=3, padding=1),
            nn.BatchNorm2d(256),
            nn.ReLU(),
            nn.MaxPool2d(2),

            # Conv block 4
            nn.Conv2d(256, 512, kernel_size=3, padding=1),
            nn.BatchNorm2d(512),
            nn.ReLU(),
        )

        # Calculate flattened dimension
        self.flat_dim = 512 * (input_height // 8)

        self.fc_layers = nn.Sequential(
            nn.Linear(self.flat_dim, 1024),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(1024, output_dim)
        )

    def forward(self, x):
        """
        x: (batch_size, 1, time_steps, feature_dim)
        """
        # Convolutional layers
        x = self.conv_layers(x)

        # Flatten spatial dimensions but keep time
        batch_size, channels, time_steps, height = x.size()
        x = x.permute(0, 2, 1, 3).contiguous()
        x = x.view(batch_size, time_steps, -1)

        # Fully connected layers
        x = self.fc_layers(x)

        return torch.log_softmax(x, dim=2)
```

## Connectionist Temporal Classification (CTC)

CTC enables end-to-end training without frame-level alignment:

```python
class CTCAcousticModel(nn.Module):
    """CTC-based acoustic model"""

    def __init__(self, input_dim, hidden_dim, num_layers, vocab_size):
        super().__init__()

        # Encoder
        self.encoder = nn.LSTM(
            input_dim,
            hidden_dim,
            num_layers,
            batch_first=True,
            bidirectional=True
        )

        # CTC output layer (vocab + blank)
        self.fc = nn.Linear(hidden_dim * 2, vocab_size + 1)

    def forward(self, features, feature_lengths):
        """
        features: (batch_size, time_steps, feature_dim)
        feature_lengths: (batch_size,)
        """
        # Encode
        encoded, _ = self.encoder(features)

        # Project to vocabulary
        logits = self.fc(encoded)

        # Log softmax for CTC
        log_probs = torch.log_softmax(logits, dim=2)

        # Permute for CTC loss: (time_steps, batch_size, vocab_size+1)
        log_probs = log_probs.permute(1, 0, 2)

        return log_probs

# CTC Training
class CTCTrainer:
    def __init__(self, model, learning_rate=0.001):
        self.model = model
        self.optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
        self.ctc_loss = nn.CTCLoss(blank=0, zero_infinity=True)

    def train_step(self, features, feature_lengths, labels, label_lengths):
        """Single training step"""
        self.model.train()

        # Forward pass
        log_probs = self.model(features, feature_lengths)

        # Compute CTC loss
        loss = self.ctc_loss(log_probs, labels, feature_lengths, label_lengths)

        # Backward pass
        self.optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(self.model.parameters(), 5.0)
        self.optimizer.step()

        return loss.item()

    def decode_greedy(self, log_probs):
        """Greedy CTC decoding"""
        # Get best path
        best_path = torch.argmax(log_probs, dim=2)

        # Remove blanks and repeated tokens
        decoded = []
        for sequence in best_path.permute(1, 0):  # (batch, time)
            prev = -1
            result = []
            for token in sequence:
                if token != 0 and token != prev:  # 0 is blank
                    result.append(token.item())
                prev = token
            decoded.append(result)

        return decoded
```

## Summary

This chapter explored acoustic modeling techniques from traditional HMMs to modern deep learning:

- **HMMs**: Statistical foundation of classical ASR
- **GMM-HMMs**: Continuous observation modeling
- **DNNs**: Deep neural networks for improved accuracy
- **TDNNs**: Temporal context modeling
- **LSTMs**: Sequential dependency handling
- **CNNs**: Hierarchical feature learning
- **CTC**: End-to-end training without alignment

Modern acoustic models leverage deep learning to achieve unprecedented accuracy while maintaining the goal of 弘益人間 - making speech recognition accessible and reliable for all humanity.

## Review Questions

1. What are the three key components of an HMM?
2. Explain the forward-backward algorithm.
3. How do GMMs extend HMMs to continuous observations?
4. What advantages do DNNs have over GMMs?
5. How do TDNNs capture temporal context?
6. Why are bidirectional LSTMs useful for ASR?
7. What problem does CTC solve?
8. Compare frame-level and sequence-level training.
9. What is the role of batch normalization in acoustic models?
10. How does attention improve LSTM models?

## Practical Exercises

1. **HMM Implementation**: Train an HMM on synthetic data
2. **DNN Training**: Build and train a DNN acoustic model
3. **CTC Decoding**: Implement beam search for CTC
4. **Model Comparison**: Compare HMM, DNN, and LSTM accuracy
5. **Feature Analysis**: Visualize learned features in different layers

---

**弘益人間 (Hongik Ingan)** - *Benefit All Humanity*

Acoustic models are the foundation that enables machines to understand human speech. By continually improving these models, we make technology more accessible to everyone, regardless of language or accent.

---

*Previous: [ASR Fundamentals](chapter2-asr-fundamentals.md) | Next: [Language Models](chapter4-language-models.md)*
