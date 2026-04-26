# Chapter 4: Diffusion Models

## 4.1 Introduction to Diffusion Models

Diffusion models represent one of the most significant recent breakthroughs in generative AI, achieving state-of-the-art results in image generation and increasingly in other modalities. Unlike GANs that learn through adversarial training or VAEs that learn through variational inference, diffusion models learn to reverse a gradual noising process.

The core idea is remarkably elegant: if we can learn to progressively denoise random noise into coherent data, we have effectively learned the data distribution. This approach draws inspiration from non-equilibrium thermodynamics and provides a stable, scalable framework for high-quality generation.

### Historical Context

Diffusion models emerged from several research threads:

**2015 - Deep Unsupervised Learning using Nonequilibrium Thermodynamics** (Sohl-Dickstein et al.): Introduced the theoretical framework connecting diffusion processes to deep learning

**2020 - Denoising Diffusion Probabilistic Models (DDPM)** (Ho et al.): Made diffusion models practical and competitive with GANs

**2021 - Diffusion Models Beat GANs on Image Synthesis** (Dhariwal & Nichol): Demonstrated superior performance to GANs

**2022 - Stable Diffusion** (Rombach et al.): Latent diffusion models enabling efficient high-resolution generation

**2023-Present**: Explosive growth with text-to-image, video, audio, and 3D applications

### Why Diffusion Models Matter

Several factors make diffusion models particularly powerful:

**High Sample Quality**: Produce extremely realistic, detailed outputs
**Training Stability**: No adversarial dynamics or mode collapse
**Flexibility**: Work across modalities (image, audio, video, 3D)
**Controllability**: Natural integration with conditioning (text, class, etc.)
**Scalability**: Performance improves predictably with model size and compute
**Theoretical Grounding**: Solid mathematical foundations

## 4.2 The Diffusion Process

Understanding diffusion models requires grasping both the forward diffusion process (adding noise) and the reverse process (denoising).

### Forward Diffusion (Adding Noise)

The forward process gradually adds Gaussian noise to data over T timesteps:

```
q(x_t | x_{t-1}) = N(x_t; √(1-β_t) x_{t-1}, β_t I)
```

Where:
- x_0: Original data
- x_t: Noised data at timestep t
- β_t: Noise schedule (how much noise to add at each step)
- T: Total number of timesteps (typically 1000)

The beauty is that we can sample x_t directly from x_0:

```
q(x_t | x_0) = N(x_t; √(ᾱ_t) x_0, (1-ᾱ_t) I)

where ᾱ_t = ∏(i=1 to t) (1 - β_i)
```

This allows efficient training without simulating the entire chain.

### Reverse Process (Denoising)

The reverse process learns to denoise:

```
p_θ(x_{t-1} | x_t) = N(x_{t-1}; μ_θ(x_t, t), Σ_θ(x_t, t))
```

A neural network learns to predict either:
1. The added noise ε
2. The denoised image x_0
3. The score (gradient of log-probability)

Most modern implementations predict the noise ε.

### Training Objective

The simplified training objective for DDPM:

```
L_simple = E_{t, x_0, ε} [||ε - ε_θ(x_t, t)||²]
```

Where:
- x_t = √(ᾱ_t) x_0 + √(1-ᾱ_t) ε
- ε ~ N(0, I): Random noise
- ε_θ: Neural network predicting noise
- t ~ Uniform(1, T): Random timestep

### Implementation

```python
class DiffusionModel(nn.Module):
    def __init__(self, num_timesteps=1000):
        super().__init__()

        # Noise schedule
        self.num_timesteps = num_timesteps
        self.betas = self.linear_beta_schedule(num_timesteps)

        self.alphas = 1. - self.betas
        self.alphas_cumprod = torch.cumprod(self.alphas, dim=0)
        self.sqrt_alphas_cumprod = torch.sqrt(self.alphas_cumprod)
        self.sqrt_one_minus_alphas_cumprod = torch.sqrt(1. - self.alphas_cumprod)

        # Denoising network (U-Net)
        self.model = UNet(
            in_channels=3,
            out_channels=3,
            channels=128,
            num_res_blocks=2,
            attention_resolutions=[16, 8]
        )

    def linear_beta_schedule(self, timesteps, beta_start=1e-4, beta_end=0.02):
        return torch.linspace(beta_start, beta_end, timesteps)

    def forward_diffusion(self, x_0, t, noise=None):
        """Add noise to x_0 to get x_t"""
        if noise is None:
            noise = torch.randn_like(x_0)

        sqrt_alpha_cumprod = self.sqrt_alphas_cumprod[t].view(-1, 1, 1, 1)
        sqrt_one_minus_alpha_cumprod = self.sqrt_one_minus_alphas_cumprod[t].view(-1, 1, 1, 1)

        x_t = sqrt_alpha_cumprod * x_0 + sqrt_one_minus_alpha_cumprod * noise
        return x_t, noise

    def training_step(self, x_0):
        """Single training step"""
        batch_size = x_0.shape[0]

        # Sample random timesteps
        t = torch.randint(0, self.num_timesteps, (batch_size,), device=x_0.device)

        # Add noise
        x_t, noise = self.forward_diffusion(x_0, t)

        # Predict noise
        noise_pred = self.model(x_t, t)

        # Compute loss
        loss = F.mse_loss(noise_pred, noise)
        return loss

    @torch.no_grad()
    def sampling_step(self, x_t, t):
        """Single reverse diffusion step"""
        # Predict noise
        noise_pred = self.model(x_t, t)

        # Get parameters
        beta_t = self.betas[t]
        alpha_t = self.alphas[t]
        alpha_cumprod_t = self.alphas_cumprod[t]

        # Compute mean
        x_0_pred = (x_t - torch.sqrt(1 - alpha_cumprod_t) * noise_pred) / torch.sqrt(alpha_cumprod_t)
        mean = (x_t - beta_t / torch.sqrt(1 - alpha_cumprod_t) * noise_pred) / torch.sqrt(alpha_t)

        # Add noise (except at t=0)
        if t > 0:
            noise = torch.randn_like(x_t)
            variance = beta_t
            x_t_minus_1 = mean + torch.sqrt(variance) * noise
        else:
            x_t_minus_1 = mean

        return x_t_minus_1

    @torch.no_grad()
    def sample(self, shape, device):
        """Generate samples through reverse diffusion"""
        # Start from pure noise
        x_t = torch.randn(shape, device=device)

        # Progressively denoise
        for t in reversed(range(self.num_timesteps)):
            t_batch = torch.full((shape[0],), t, device=device, dtype=torch.long)
            x_t = self.sampling_step(x_t, t_batch)

        return x_t
```

## 4.3 The U-Net Architecture

Diffusion models typically use U-Net architectures for the denoising network, combining downsampling and upsampling paths with skip connections.

### Basic U-Net Components

```python
class ResidualBlock(nn.Module):
    def __init__(self, in_channels, out_channels, time_emb_dim):
        super().__init__()

        self.conv1 = nn.Conv2d(in_channels, out_channels, 3, padding=1)
        self.conv2 = nn.Conv2d(out_channels, out_channels, 3, padding=1)

        self.time_mlp = nn.Sequential(
            nn.Linear(time_emb_dim, out_channels),
            nn.SiLU()
        )

        self.norm1 = nn.GroupNorm(8, out_channels)
        self.norm2 = nn.GroupNorm(8, out_channels)

        self.act = nn.SiLU()

        if in_channels != out_channels:
            self.residual_conv = nn.Conv2d(in_channels, out_channels, 1)
        else:
            self.residual_conv = nn.Identity()

    def forward(self, x, time_emb):
        residual = x

        x = self.conv1(x)
        x = self.norm1(x)
        x = self.act(x)

        # Add time embedding
        time_emb = self.time_mlp(time_emb)
        x = x + time_emb[:, :, None, None]

        x = self.conv2(x)
        x = self.norm2(x)

        x = x + self.residual_conv(residual)
        x = self.act(x)

        return x

class AttentionBlock(nn.Module):
    def __init__(self, channels, num_heads=4):
        super().__init__()
        self.channels = channels
        self.num_heads = num_heads

        self.norm = nn.GroupNorm(8, channels)
        self.qkv = nn.Conv2d(channels, channels * 3, 1)
        self.proj = nn.Conv2d(channels, channels, 1)

    def forward(self, x):
        residual = x
        x = self.norm(x)

        b, c, h, w = x.shape
        qkv = self.qkv(x)
        qkv = qkv.reshape(b, 3, self.num_heads, c // self.num_heads, h * w)
        q, k, v = qkv[:, 0], qkv[:, 1], qkv[:, 2]

        # Attention
        attn = torch.einsum('bhci,bhcj->bhij', q, k) / (c // self.num_heads) ** 0.5
        attn = F.softmax(attn, dim=-1)

        # Apply attention
        out = torch.einsum('bhij,bhcj->bhci', attn, v)
        out = out.reshape(b, c, h, w)
        out = self.proj(out)

        return out + residual

class UNet(nn.Module):
    def __init__(self, in_channels=3, out_channels=3, channels=128,
                 num_res_blocks=2, attention_resolutions=[16, 8]):
        super().__init__()

        # Time embedding
        time_emb_dim = channels * 4
        self.time_mlp = nn.Sequential(
            SinusoidalPositionEmbedding(channels),
            nn.Linear(channels, time_emb_dim),
            nn.SiLU(),
            nn.Linear(time_emb_dim, time_emb_dim)
        )

        # Encoder (downsampling)
        self.encoder_blocks = nn.ModuleList()
        ch_mult = [1, 2, 2, 4]
        current_channels = channels

        for level, mult in enumerate(ch_mult):
            out_channels_level = channels * mult

            for _ in range(num_res_blocks):
                block = ResidualBlock(current_channels, out_channels_level, time_emb_dim)
                self.encoder_blocks.append(block)
                current_channels = out_channels_level

                # Add attention at specified resolutions
                if 2 ** (len(ch_mult) - level - 1) in attention_resolutions:
                    self.encoder_blocks.append(AttentionBlock(current_channels))

            # Downsample (except last level)
            if level != len(ch_mult) - 1:
                self.encoder_blocks.append(nn.Conv2d(current_channels, current_channels, 3, stride=2, padding=1))

        # Middle
        self.middle = nn.ModuleList([
            ResidualBlock(current_channels, current_channels, time_emb_dim),
            AttentionBlock(current_channels),
            ResidualBlock(current_channels, current_channels, time_emb_dim)
        ])

        # Decoder (upsampling)
        self.decoder_blocks = nn.ModuleList()
        # Implementation continues...

class SinusoidalPositionEmbedding(nn.Module):
    def __init__(self, dim):
        super().__init__()
        self.dim = dim

    def forward(self, time):
        device = time.device
        half_dim = self.dim // 2
        embeddings = math.log(10000) / (half_dim - 1)
        embeddings = torch.exp(torch.arange(half_dim, device=device) * -embeddings)
        embeddings = time[:, None] * embeddings[None, :]
        embeddings = torch.cat((embeddings.sin(), embeddings.cos()), dim=-1)
        return embeddings
```

## 4.4 Noise Schedules

The noise schedule β_t significantly impacts training and generation quality.

### Linear Schedule

```python
def linear_schedule(timesteps, beta_start=1e-4, beta_end=0.02):
    return torch.linspace(beta_start, beta_end, timesteps)
```

Simple but not always optimal.

### Cosine Schedule

```python
def cosine_schedule(timesteps, s=0.008):
    steps = timesteps + 1
    x = torch.linspace(0, timesteps, steps)
    alphas_cumprod = torch.cos(((x / timesteps) + s) / (1 + s) * torch.pi * 0.5) ** 2
    alphas_cumprod = alphas_cumprod / alphas_cumprod[0]
    betas = 1 - (alphas_cumprod[1:] / alphas_cumprod[:-1])
    return torch.clip(betas, 0.0001, 0.9999)
```

Often produces better results, especially for images.

### Learned Schedules

Some approaches learn optimal schedules during training.

## 4.5 Conditional Diffusion

Conditioning enables controlled generation based on text, class labels, or other signals.

### Classifier Guidance

Use a classifier to guide the diffusion process toward desired classes:

```python
def classifier_guidance(x_t, t, class_label, classifier, scale=1.0):
    """Guide diffusion using classifier gradients"""
    with torch.enable_grad():
        x_t = x_t.detach().requires_grad_(True)
        logits = classifier(x_t, t)
        log_probs = F.log_softmax(logits, dim=-1)
        selected_log_probs = log_probs[range(len(logits)), class_label]
        gradient = torch.autograd.grad(selected_log_probs.sum(), x_t)[0]

    return scale * gradient
```

### Classifier-Free Guidance

Train a single model for both conditional and unconditional generation:

```python
class ConditionalUNet(nn.Module):
    def __init__(self, num_classes, class_emb_dim=128):
        super().__init__()

        # Class embedding (None for unconditional)
        self.class_embedding = nn.Embedding(num_classes + 1, class_emb_dim)
        # ... rest of U-Net

    def forward(self, x, t, class_label=None):
        # If class_label is None, use unconditional (class index = num_classes)
        if class_label is None:
            class_label = torch.full((x.shape[0],), self.num_classes, device=x.device)

        class_emb = self.class_embedding(class_label)
        # Combine with time embedding and proceed with U-Net
        # ...

def classifier_free_guidance(model, x_t, t, class_label, guidance_scale=7.5):
    """Classifier-free guidance sampling"""
    # Conditional prediction
    eps_cond = model(x_t, t, class_label)

    # Unconditional prediction
    eps_uncond = model(x_t, t, None)

    # Guided prediction
    eps = eps_uncond + guidance_scale * (eps_cond - eps_uncond)
    return eps
```

### Text Conditioning

For text-to-image models like Stable Diffusion:

```python
class TextConditionedUNet(nn.Module):
    def __init__(self, text_emb_dim=768):
        super().__init__()

        # Cross-attention to text embeddings
        self.cross_attention = CrossAttention(
            query_dim=channels,
            context_dim=text_emb_dim,
            heads=8
        )

    def forward(self, x, t, text_embeddings):
        # ... U-Net processing with cross-attention to text
        x = self.cross_attention(x, context=text_embeddings)
        # ...
```

## 4.6 Latent Diffusion Models

Latent diffusion models (like Stable Diffusion) perform diffusion in a compressed latent space rather than pixel space, dramatically improving efficiency.

### Architecture

```
Original Image → VAE Encoder → Latent Space → Diffusion → VAE Decoder → Generated Image
```

### Implementation Concept

```python
class LatentDiffusionModel(nn.Module):
    def __init__(self, vae, unet, text_encoder):
        super().__init__()

        self.vae = vae  # Pre-trained VAE
        self.unet = unet  # Diffusion U-Net
        self.text_encoder = text_encoder  # Pre-trained text encoder (e.g., CLIP)

    def forward(self, images, text):
        # Encode images to latent space
        with torch.no_grad():
            latents = self.vae.encode(images).latent_dist.sample()
            latents = latents * 0.18215  # Scaling factor

        # Encode text
        text_embeddings = self.text_encoder(text)

        # Diffusion in latent space
        t = torch.randint(0, self.num_timesteps, (latents.shape[0],))
        noise = torch.randn_like(latents)
        noisy_latents = self.add_noise(latents, noise, t)

        # Predict noise
        noise_pred = self.unet(noisy_latents, t, text_embeddings)

        # Loss
        loss = F.mse_loss(noise_pred, noise)
        return loss

    @torch.no_grad()
    def sample(self, text, num_inference_steps=50):
        # Encode text
        text_embeddings = self.text_encoder(text)

        # Start from noise
        latents = torch.randn(1, 4, 64, 64)

        # Diffusion sampling
        for t in self.scheduler.timesteps:
            noise_pred = self.unet(latents, t, text_embeddings)
            latents = self.scheduler.step(noise_pred, t, latents).prev_sample

        # Decode to image
        latents = latents / 0.18215
        images = self.vae.decode(latents).sample

        return images
```

### Benefits

- **Efficiency**: Operate on 64x64 latents instead of 512x512 pixels
- **Speed**: ~10x faster training and inference
- **Memory**: Much lower memory requirements
- **Quality**: Maintain high output quality

## 4.7 Advanced Techniques

### DDIM (Denoising Diffusion Implicit Models)

Accelerates sampling by using deterministic sampling:

```python
def ddim_step(model, x_t, t, t_prev, eta=0.0):
    """DDIM sampling step (can skip timesteps)"""
    noise_pred = model(x_t, t)

    alpha_t = alphas_cumprod[t]
    alpha_t_prev = alphas_cumprod[t_prev]

    # Predict x_0
    x_0_pred = (x_t - torch.sqrt(1 - alpha_t) * noise_pred) / torch.sqrt(alpha_t)

    # Direction pointing to x_t
    dir_xt = torch.sqrt(1 - alpha_t_prev - eta ** 2 * variance) * noise_pred

    # Add noise
    noise = torch.randn_like(x_t)
    x_t_prev = torch.sqrt(alpha_t_prev) * x_0_pred + dir_xt + eta * torch.sqrt(variance) * noise

    return x_t_prev
```

Enables sampling in 50 steps instead of 1000.

### Progressive Distillation

Distill diffusion models into fewer steps:

```python
def progressive_distillation(teacher_model, student_model, x_0):
    """Train student to match teacher in half the steps"""

    # Teacher uses 2 steps
    t1 = sample_timestep()
    t2 = t1 + step_size

    x_t2 = add_noise(x_0, t2)
    with torch.no_grad():
        x_t1_teacher = teacher_step(teacher_model, x_t2, t2, t1)

    # Student uses 1 step
    x_t1_student = student_step(student_model, x_t2, t2, t1)

    # Match teacher output
    loss = F.mse_loss(x_t1_student, x_t1_teacher)
    return loss
```

### Consistency Models

New approach enabling single-step generation while maintaining quality.

## 4.8 Applications

Diffusion models excel across many domains:

### Text-to-Image Generation

- Stable Diffusion, DALL-E 2, Imagen
- High-quality, controllable image synthesis
- Personalization (DreamBooth, Textual Inversion)

### Image Editing

- Inpainting: Fill missing regions
- Outpainting: Extend images beyond borders
- Image-to-image: Guided transformation

```python
def inpainting(model, image, mask, text, num_steps=50):
    """Inpaint masked regions"""
    latent = vae.encode(image)

    for t in reversed(range(num_steps)):
        if t > 0:
            # Diffusion step
            noise_pred = model(latent, t, text)
            latent = scheduler.step(noise_pred, t, latent)

            # Replace unmasked regions with original
            latent = mask * latent + (1 - mask) * vae.encode(image)

    return vae.decode(latent)
```

### Video Generation

- Extend diffusion to temporal dimension
- Generate coherent video sequences
- Applications in animation, VFX

### Audio and Music

- Generate audio waveforms or spectrograms
- Music composition and sound effects
- Voice synthesis

### 3D Generation

- Generate 3D shapes and scenes
- NeRF-based diffusion
- 3D-aware image synthesis

### Super-Resolution

- Enhance low-resolution images
- Restoration of degraded images

### Scientific Applications

- Protein structure prediction
- Molecular generation
- Climate modeling
- Medical imaging synthesis

## Summary

Diffusion models have emerged as a dominant paradigm in generative AI, combining theoretical elegance with practical effectiveness. By learning to reverse a gradual noising process, they achieve state-of-the-art results across modalities.

**Key Takeaways**:
- Diffusion models learn to reverse noise addition through denoising
- Training is stable and scalable
- U-Net architecture with time conditioning is standard
- Conditioning enables text-to-image and controlled generation
- Latent diffusion dramatically improves efficiency
- Applications span image, video, audio, 3D, and scientific domains
- Recent advances enable faster sampling and distillation

Diffusion models represent a major milestone in generative AI, powering many of today's most impressive creative AI tools.

## Review Questions

1. **Fundamentals**
   - Explain the forward and reverse diffusion processes
   - Why can we sample x_t directly from x_0?
   - What does the denoising network predict during training?

2. **Architecture**
   - Why is U-Net architecture commonly used for diffusion models?
   - Explain the role of time embeddings
   - How do attention mechanisms improve diffusion models?

3. **Conditioning**
   - Compare classifier guidance and classifier-free guidance
   - How does text conditioning work in models like Stable Diffusion?
   - What are the trade-offs of guidance scale?

4. **Latent Diffusion**
   - Why perform diffusion in latent space vs. pixel space?
   - What are the components of a latent diffusion model?
   - How does latent diffusion maintain quality while improving efficiency?

5. **Applications**
   - Describe three applications of diffusion models beyond text-to-image
   - How can diffusion models be used for image editing?
   - What makes diffusion models suitable for scientific applications?

## Practical Exercise

Implement a simple diffusion model:

1. Build a DDPM model for 32x32 images
2. Implement the forward diffusion process with cosine schedule
3. Create a simplified U-Net architecture
4. Train on a dataset (CIFAR-10, CelebA, or custom)
5. Implement DDIM sampling for faster generation
6. Add classifier-free guidance for conditional generation
7. Visualize the denoising process step-by-step
8. Compare different guidance scales and sampling steps

Document:
- Training curves and convergence
- Generated samples at different checkpoints
- Effect of guidance scale on output
- Denoising visualization
- Comparison with GAN or VAE if possible

---

**弘益人間 (Benefit All Humanity)** - Diffusion models exemplify how patient, gradual processes can achieve remarkable results. Like many worthwhile endeavors, generation happens through many small steps, each building on the last. May we apply this wisdom not just in our models, but in how we build technology that serves humanity.

---

*Previous: [Chapter 3 - VAEs](./03-vaes.md) | Next: [Chapter 5 - Large Language Models](./05-large-language-models.md)*

---

© 2025 SmileStory Inc. / WIA | WIA-AI-026 Generative AI Standard
