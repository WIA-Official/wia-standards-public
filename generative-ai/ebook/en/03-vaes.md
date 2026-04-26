# Chapter 3: Variational Autoencoders (VAEs)

## 3.1 Introduction to VAEs

Variational Autoencoders (VAEs) represent a fundamentally different approach to generative modeling compared to GANs. Instead of adversarial training, VAEs combine ideas from deep learning and Bayesian inference to learn a probabilistic latent representation of data.

Introduced by Kingma and Welling in 2013, VAEs provide a principled probabilistic framework for generative modeling. They offer several advantages:
- Explicit likelihood modeling
- Stable training (no adversarial dynamics)
- Interpretable latent space
- Theoretical grounding in variational inference

### The Autoencoder Foundation

To understand VAEs, we first examine traditional autoencoders:

**Autoencoder Components**:
- **Encoder**: Compresses input data x into a lower-dimensional latent representation z
- **Decoder**: Reconstructs the original data from the latent representation
- **Training objective**: Minimize reconstruction error

```python
class Autoencoder(nn.Module):
    def __init__(self, input_dim, latent_dim):
        super(Autoencoder, self).__init__()

        # Encoder
        self.encoder = nn.Sequential(
            nn.Linear(input_dim, 512),
            nn.ReLU(),
            nn.Linear(512, 256),
            nn.ReLU(),
            nn.Linear(256, latent_dim)
        )

        # Decoder
        self.decoder = nn.Sequential(
            nn.Linear(latent_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 512),
            nn.ReLU(),
            nn.Linear(512, input_dim),
            nn.Sigmoid()
        )

    def forward(self, x):
        z = self.encoder(x)
        x_reconstructed = self.decoder(z)
        return x_reconstructed
```

While autoencoders learn useful representations, they have limitations for generation:
- Latent space may have "holes" where decoding produces unrealistic outputs
- No probabilistic interpretation
- Deterministic encoding doesn't capture uncertainty

VAEs address these limitations by treating the latent representation probabilistically.

## 3.2 VAE Theory and Mathematics

VAEs are built on solid theoretical foundations in variational inference and probabilistic modeling.

### Probabilistic Formulation

VAEs model the generative process as:

1. Sample latent variable z from prior distribution p(z)
2. Generate data x from conditional distribution p(x|z)

The goal is to learn:
- **Decoder**: p_θ(x|z) - the generative model
- **Encoder**: q_φ(z|x) - the inference model (approximates true posterior p(z|x))

### The Evidence Lower Bound (ELBO)

The key insight is to maximize the log-likelihood of data, but since computing p(x) directly is intractable, we maximize a lower bound instead.

The ELBO objective:

```
ELBO = E_q[log p_θ(x|z)] - KL(q_φ(z|x) || p(z))
```

Where:
- First term: **Reconstruction loss** - how well the decoder reconstructs data
- Second term: **KL divergence** - regularization keeping learned distribution close to prior

The full derivation:

```
log p(x) = ELBO + KL(q_φ(z|x) || p_θ(z|x))

Since KL ≥ 0:
log p(x) ≥ ELBO

Maximizing ELBO gives:
ELBO = E_z~q[log p_θ(x|z)] - KL(q_φ(z|x) || p(z))
     = -Reconstruction_Loss - KL_Divergence
```

### The Reparameterization Trick

A crucial innovation enabling VAE training is the reparameterization trick, which allows backpropagation through stochastic sampling.

Instead of sampling z ~ q_φ(z|x) = N(μ, σ²) directly:

```python
# Non-differentiable (can't backpropagate through sampling)
z = mu + sigma * random_normal()
```

We reparameterize:

```python
# Differentiable
epsilon = torch.randn_like(sigma)  # Sample from N(0,1)
z = mu + sigma * epsilon  # Deterministic transformation
```

This makes the sampling operation differentiable with respect to μ and σ.

### Standard VAE Architecture

```python
class VAE(nn.Module):
    def __init__(self, input_dim, hidden_dim, latent_dim):
        super(VAE, self).__init__()

        # Encoder
        self.encoder_hidden = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU()
        )

        self.fc_mu = nn.Linear(hidden_dim, latent_dim)
        self.fc_logvar = nn.Linear(hidden_dim, latent_dim)

        # Decoder
        self.decoder = nn.Sequential(
            nn.Linear(latent_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, input_dim),
            nn.Sigmoid()
        )

    def encode(self, x):
        h = self.encoder_hidden(x)
        mu = self.fc_mu(h)
        logvar = self.fc_logvar(h)
        return mu, logvar

    def reparameterize(self, mu, logvar):
        std = torch.exp(0.5 * logvar)
        eps = torch.randn_like(std)
        return mu + eps * std

    def decode(self, z):
        return self.decoder(z)

    def forward(self, x):
        mu, logvar = self.encode(x)
        z = self.reparameterize(mu, logvar)
        x_recon = self.decode(z)
        return x_recon, mu, logvar
```

### Loss Function

```python
def vae_loss(x, x_recon, mu, logvar, beta=1.0):
    # Reconstruction loss (binary cross-entropy for binary data)
    recon_loss = F.binary_cross_entropy(x_recon, x, reduction='sum')

    # KL divergence
    # KL(N(μ,σ²) || N(0,1)) = -0.5 * sum(1 + log(σ²) - μ² - σ²)
    kl_divergence = -0.5 * torch.sum(1 + logvar - mu.pow(2) - logvar.exp())

    # Total loss
    return recon_loss + beta * kl_divergence
```

## 3.3 Training VAEs

Training VAEs involves optimizing the ELBO objective while managing the balance between reconstruction and regularization.

### Training Loop

```python
def train_vae(model, train_loader, optimizer, num_epochs, device):
    model.train()

    for epoch in range(num_epochs):
        total_loss = 0
        total_recon = 0
        total_kl = 0

        for batch_idx, (data, _) in enumerate(train_loader):
            data = data.to(device)

            # Forward pass
            optimizer.zero_grad()
            recon_batch, mu, logvar = model(data)

            # Calculate loss
            recon_loss = F.binary_cross_entropy(recon_batch, data, reduction='sum')
            kl_loss = -0.5 * torch.sum(1 + logvar - mu.pow(2) - logvar.exp())
            loss = recon_loss + kl_loss

            # Backward pass
            loss.backward()
            optimizer.step()

            # Track metrics
            total_loss += loss.item()
            total_recon += recon_loss.item()
            total_kl += kl_loss.item()

        # Epoch statistics
        avg_loss = total_loss / len(train_loader.dataset)
        avg_recon = total_recon / len(train_loader.dataset)
        avg_kl = total_kl / len(train_loader.dataset)

        print(f'Epoch {epoch}: Loss: {avg_loss:.4f}, '
              f'Recon: {avg_recon:.4f}, KL: {avg_kl:.4f}')

    return model
```

### Beta-VAE

Beta-VAE introduces a hyperparameter β to control the trade-off between reconstruction and disentanglement:

```python
loss = recon_loss + beta * kl_loss
```

**Effects of β**:
- β < 1: Prioritize reconstruction, may produce entangled representations
- β = 1: Standard VAE
- β > 1: Stronger regularization, encourages disentangled representations

Higher β values tend to produce more disentangled latent representations where individual dimensions correspond to interpretable factors of variation.

### Annealing Strategies

To improve training stability, we can gradually increase the weight of the KL term:

```python
def get_kl_weight(epoch, total_epochs, annealing_epochs):
    if epoch < annealing_epochs:
        return epoch / annealing_epochs
    return 1.0

# In training loop:
kl_weight = get_kl_weight(epoch, num_epochs, annealing_epochs=10)
loss = recon_loss + kl_weight * kl_loss
```

This prevents the KL term from overwhelming early training when the reconstruction capability is still developing.

## 3.4 Advanced VAE Architectures

Several variants of VAEs have been developed to address specific challenges and applications.

### Convolutional VAE

For image data, convolutional layers are more effective:

```python
class ConvVAE(nn.Module):
    def __init__(self, latent_dim=128, img_channels=3):
        super(ConvVAE, self).__init__()

        # Encoder
        self.encoder = nn.Sequential(
            nn.Conv2d(img_channels, 32, 4, stride=2, padding=1),  # 64 -> 32
            nn.ReLU(),
            nn.Conv2d(32, 64, 4, stride=2, padding=1),  # 32 -> 16
            nn.ReLU(),
            nn.Conv2d(64, 128, 4, stride=2, padding=1),  # 16 -> 8
            nn.ReLU(),
            nn.Conv2d(128, 256, 4, stride=2, padding=1),  # 8 -> 4
            nn.ReLU(),
        )

        self.fc_mu = nn.Linear(256 * 4 * 4, latent_dim)
        self.fc_logvar = nn.Linear(256 * 4 * 4, latent_dim)

        # Decoder
        self.fc_decode = nn.Linear(latent_dim, 256 * 4 * 4)

        self.decoder = nn.Sequential(
            nn.ConvTranspose2d(256, 128, 4, stride=2, padding=1),  # 4 -> 8
            nn.ReLU(),
            nn.ConvTranspose2d(128, 64, 4, stride=2, padding=1),  # 8 -> 16
            nn.ReLU(),
            nn.ConvTranspose2d(64, 32, 4, stride=2, padding=1),  # 16 -> 32
            nn.ReLU(),
            nn.ConvTranspose2d(32, img_channels, 4, stride=2, padding=1),  # 32 -> 64
            nn.Sigmoid()
        )

    def encode(self, x):
        h = self.encoder(x)
        h = h.view(h.size(0), -1)
        return self.fc_mu(h), self.fc_logvar(h)

    def decode(self, z):
        h = self.fc_decode(z)
        h = h.view(h.size(0), 256, 4, 4)
        return self.decoder(h)
```

### Conditional VAE (CVAE)

CVAEs condition both encoder and decoder on additional information (labels, attributes):

```python
class ConditionalVAE(nn.Module):
    def __init__(self, input_dim, latent_dim, num_classes):
        super(ConditionalVAE, self).__init__()

        self.label_embedding = nn.Embedding(num_classes, num_classes)

        # Encoder takes input + label
        self.encoder = nn.Sequential(
            nn.Linear(input_dim + num_classes, 512),
            nn.ReLU(),
            nn.Linear(512, 256),
            nn.ReLU()
        )

        self.fc_mu = nn.Linear(256, latent_dim)
        self.fc_logvar = nn.Linear(256, latent_dim)

        # Decoder takes latent + label
        self.decoder = nn.Sequential(
            nn.Linear(latent_dim + num_classes, 256),
            nn.ReLU(),
            nn.Linear(256, 512),
            nn.ReLU(),
            nn.Linear(512, input_dim),
            nn.Sigmoid()
        )

    def encode(self, x, labels):
        label_emb = self.label_embedding(labels)
        inputs = torch.cat([x, label_emb], dim=1)
        h = self.encoder(inputs)
        return self.fc_mu(h), self.fc_logvar(h)

    def decode(self, z, labels):
        label_emb = self.label_embedding(labels)
        inputs = torch.cat([z, label_emb], dim=1)
        return self.decoder(inputs)
```

Applications:
- Controlled generation (specify class, attribute, or style)
- Semi-supervised learning
- Domain adaptation

### Vector Quantized VAE (VQ-VAE)

VQ-VAE uses discrete latent representations instead of continuous:

**Key Ideas**:
- Encoder outputs continuous vectors
- Vectors are quantized to nearest codebook entry
- Decoder reconstructs from discrete codes
- Enables hierarchical generation and compression

```python
class VectorQuantizer(nn.Module):
    def __init__(self, num_embeddings, embedding_dim, commitment_cost):
        super(VectorQuantizer, self).__init__()

        self.embedding_dim = embedding_dim
        self.num_embeddings = num_embeddings
        self.commitment_cost = commitment_cost

        # Codebook
        self.embeddings = nn.Embedding(num_embeddings, embedding_dim)
        self.embeddings.weight.data.uniform_(-1/num_embeddings, 1/num_embeddings)

    def forward(self, inputs):
        # Flatten input
        flat_input = inputs.view(-1, self.embedding_dim)

        # Calculate distances to codebook entries
        distances = (torch.sum(flat_input**2, dim=1, keepdim=True)
                    + torch.sum(self.embeddings.weight**2, dim=1)
                    - 2 * torch.matmul(flat_input, self.embeddings.weight.t()))

        # Find nearest codebook entry
        encoding_indices = torch.argmin(distances, dim=1).unsqueeze(1)
        encodings = torch.zeros(encoding_indices.shape[0], self.num_embeddings, device=inputs.device)
        encodings.scatter_(1, encoding_indices, 1)

        # Quantize
        quantized = torch.matmul(encodings, self.embeddings.weight).view(inputs.shape)

        # Loss
        e_latent_loss = F.mse_loss(quantized.detach(), inputs)
        q_latent_loss = F.mse_loss(quantized, inputs.detach())
        loss = q_latent_loss + self.commitment_cost * e_latent_loss

        # Straight-through estimator
        quantized = inputs + (quantized - inputs).detach()

        return quantized, loss
```

VQ-VAE enables:
- High-quality image generation
- Audio generation (used in WaveNet and Jukebox)
- Hierarchical representations
- Better compression than continuous VAEs

### Hierarchical VAE

Hierarchical VAEs use multiple levels of latent variables:

```python
class HierarchicalVAE(nn.Module):
    def __init__(self, input_dim, latent_dims=[256, 128, 64]):
        super(HierarchicalVAE, self).__init__()

        self.latent_dims = latent_dims

        # Encoder hierarchy
        self.encoders = nn.ModuleList()
        prev_dim = input_dim
        for latent_dim in latent_dims:
            self.encoders.append(nn.Sequential(
                nn.Linear(prev_dim, latent_dim * 2),
                nn.ReLU()
            ))
            prev_dim = latent_dim

        # Decoder hierarchy (reverse order)
        self.decoders = nn.ModuleList()
        for i in range(len(latent_dims) - 1, -1, -1):
            if i == len(latent_dims) - 1:
                in_dim = latent_dims[i]
            else:
                in_dim = latent_dims[i] + latent_dims[i + 1]

            self.decoders.append(nn.Sequential(
                nn.Linear(in_dim, latent_dims[i - 1] if i > 0 else input_dim),
                nn.ReLU() if i > 0 else nn.Sigmoid()
            ))
```

Benefits:
- Captures hierarchical structure in data
- Different levels model different abstraction levels
- Improved generation quality for complex data

## 3.5 Latent Space Exploration

One of VAE's key advantages is the interpretable, continuous latent space.

### Interpolation

Smoothly interpolate between two datapoints in latent space:

```python
def interpolate(vae, x1, x2, steps=10):
    vae.eval()
    with torch.no_grad():
        # Encode both inputs
        mu1, _ = vae.encode(x1)
        mu2, _ = vae.encode(x2)

        # Interpolate in latent space
        interpolations = []
        for alpha in torch.linspace(0, 1, steps):
            z_interp = alpha * mu2 + (1 - alpha) * mu1
            x_interp = vae.decode(z_interp)
            interpolations.append(x_interp)

    return interpolations
```

### Sampling

Generate new samples by sampling from the prior:

```python
def sample(vae, num_samples, latent_dim, device):
    vae.eval()
    with torch.no_grad():
        # Sample from prior N(0, I)
        z = torch.randn(num_samples, latent_dim).to(device)
        samples = vae.decode(z)
    return samples
```

### Attribute Manipulation

Discover directions in latent space corresponding to attributes:

```python
def find_attribute_direction(vae, data_with_attr, data_without_attr):
    """Find direction in latent space corresponding to an attribute"""
    with torch.no_grad():
        mu_with, _ = vae.encode(data_with_attr)
        mu_without, _ = vae.encode(data_without_attr)

        # Average difference gives attribute direction
        direction = (mu_with.mean(dim=0) - mu_without.mean(dim=0))

    return direction

def apply_attribute(vae, x, direction, strength=1.0):
    """Apply attribute to image by moving in latent space"""
    with torch.no_grad():
        mu, _ = vae.encode(x)
        mu_modified = mu + strength * direction
        x_modified = vae.decode(mu_modified)
    return x_modified
```

### Disentanglement

Disentangled representations have independent dimensions corresponding to distinct factors:

```python
def traverse_latent_dimensions(vae, base_z, dim_index, range_vals):
    """Traverse a single latent dimension while keeping others fixed"""
    traversals = []

    for val in range_vals:
        z = base_z.clone()
        z[0, dim_index] = val
        x = vae.decode(z)
        traversals.append(x)

    return traversals
```

## 3.6 Applications

VAEs excel in applications requiring:
- Probabilistic modeling
- Interpretable representations
- Smooth latent spaces

### Image Generation and Manipulation

- Generate new images by sampling from prior
- Interpolate between images smoothly
- Edit specific attributes (pose, lighting, expression)
- Image compression with learned codecs

### Anomaly Detection

VAEs can identify outliers based on reconstruction error:

```python
def detect_anomalies(vae, data, threshold):
    vae.eval()
    with torch.no_grad():
        recon, mu, logvar = vae(data)
        recon_errors = F.mse_loss(recon, data, reduction='none').mean(dim=[1,2,3])

        anomalies = recon_errors > threshold
    return anomalies, recon_errors
```

Applications:
- Manufacturing defect detection
- Network intrusion detection
- Medical imaging (detecting abnormalities)
- Fraud detection

### Representation Learning

VAEs learn useful representations for:
- Semi-supervised learning (label few samples, use VAE features for rest)
- Transfer learning (pretrain VAE, use encoder for downstream tasks)
- Data visualization (project to 2D/3D latent space)

### Drug Discovery and Molecular Design

VAEs can generate novel molecular structures:

```python
class MolecularVAE(nn.Module):
    """VAE for molecular structures (SMILES strings)"""

    def __init__(self, vocab_size, max_length, latent_dim):
        super(MolecularVAE, self).__init__()

        # Encoder: GRU over SMILES sequence
        self.encoder_rnn = nn.GRU(vocab_size, 256, 3, batch_first=True)
        self.fc_mu = nn.Linear(256, latent_dim)
        self.fc_logvar = nn.Linear(256, latent_dim)

        # Decoder: GRU generating SMILES
        self.fc_decode = nn.Linear(latent_dim, 256)
        self.decoder_rnn = nn.GRU(256, 256, 3, batch_first=True)
        self.fc_out = nn.Linear(256, vocab_size)
```

Applications:
- Generate molecules with desired properties
- Optimize molecules for drug-likeness
- Explore chemical space efficiently

### Data Imputation

Fill in missing values using VAE's generative model:

```python
def impute_missing_values(vae, data_with_missing, missing_mask, num_iterations=10):
    """Iteratively impute missing values"""
    data = data_with_missing.clone()

    for _ in range(num_iterations):
        with torch.no_grad():
            # Reconstruct
            recon, _, _ = vae(data)

            # Fill in missing values with reconstructions
            data = torch.where(missing_mask, recon, data_with_missing)

    return data
```

## 3.7 VAEs vs GANs

Understanding when to use VAEs versus GANs:

### VAE Advantages

**Training Stability**: No adversarial dynamics, stable optimization
**Theoretical Foundation**: Principled probabilistic framework
**Explicit Likelihood**: Can evaluate data likelihood
**Latent Space Quality**: Continuous, smooth, interpretable
**Inference**: Can encode new data into latent space

### VAE Disadvantages

**Sample Quality**: Generally produces blurrier samples than GANs
**Reconstruction Bias**: Tends to average over modes
**Computational Cost**: Requires multiple forward passes for sampling

### When to Choose VAEs

- Need explicit likelihood estimates
- Require stable training
- Want interpretable latent representations
- Need to encode new data
- Working with limited data
- Anomaly detection applications
- Scientific applications requiring uncertainty quantification

### When to Choose GANs

- Prioritize sample quality over diversity
- Have sufficient training data
- Don't need to encode new data
- Can tolerate training instability
- Image generation is primary goal

## Summary

Variational Autoencoders provide a principled approach to generative modeling through probabilistic inference. By learning a structured latent representation and optimizing the ELBO, VAEs enable stable training and interpretable generation.

**Key Takeaways**:
- VAEs combine autoencoders with probabilistic modeling
- ELBO objective balances reconstruction and regularization
- Reparameterization trick enables backpropagation through sampling
- Variants like β-VAE, CVAE, and VQ-VAE extend capabilities
- Latent space is continuous and interpretable
- Applications include generation, anomaly detection, and representation learning
- Trade-off: stability and interpretability vs. sample quality

VAEs remain essential for applications requiring probabilistic modeling, stable training, and interpretable representations, complementing GANs in the generative AI toolkit.

## Review Questions

1. **Theory**
   - Derive the ELBO objective from the log-likelihood
   - Explain the reparameterization trick and why it's necessary
   - What is the role of the KL divergence term in VAE training?

2. **Architecture**
   - Compare standard VAE to β-VAE
   - How do Conditional VAEs differ from standard VAEs?
   - Explain the quantization mechanism in VQ-VAE

3. **Training**
   - What are common challenges in VAE training?
   - How does KL annealing improve training?
   - Why might you see the KL term collapse to zero?

4. **Applications**
   - Describe how VAEs can be used for anomaly detection
   - How can you manipulate specific attributes using VAE latent space?
   - What makes VAEs suitable for molecular design?

5. **Comparison**
   - Compare VAE and GAN sample quality
   - When would you prefer VAE over GAN?
   - Can VAE and GAN techniques be combined? How?

## Practical Exercise

Implement and train a VAE on image data:

1. Build a Convolutional VAE for 64x64 images
2. Implement the ELBO loss with configurable β
3. Train with and without KL annealing, compare results
4. Generate new samples by sampling from prior
5. Interpolate between pairs of real images
6. Identify and traverse interesting latent dimensions
7. Implement anomaly detection using reconstruction error
8. Compare results with different β values (0.5, 1.0, 2.0, 5.0)

Document your findings with visualizations of:
- Training curves (total loss, reconstruction, KL)
- Generated samples over training
- Latent space interpolations
- Latent dimension traversals
- Reconstruction quality

---

**弘益人間 (Benefit All Humanity)** - VAEs exemplify how probabilistic thinking and rigorous theory can guide practical AI development. Their interpretable latent spaces and stable training make them valuable tools for scientific discovery, from designing new molecules to understanding complex data distributions.

---

*Previous: [Chapter 2 - GANs](./02-gans.md) | Next: [Chapter 4 - Diffusion Models](./04-diffusion-models.md)*

---

© 2025 SmileStory Inc. / WIA | WIA-AI-026 Generative AI Standard
