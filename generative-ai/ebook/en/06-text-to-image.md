# Chapter 6: Text-to-Image Generation

## 6.1 The Text-to-Image Revolution

Text-to-image generation represents one of the most visible and impactful applications of generative AI. The ability to create detailed, photorealistic images from natural language descriptions has transformed creative workflows, democratized visual content creation, and sparked both excitement and important conversations about AI's role in art and media.

The journey from text to pixels involves bridging two fundamentally different modalities—discrete symbolic language and continuous visual information. Modern systems achieve this through sophisticated architectures combining language understanding, cross-modal alignment, and powerful image generation.

### Historical Milestones

**2021 - DALL-E (OpenAI)**: First large-scale text-to-image model using discrete VAE and transformer
**2021 - CLIP (OpenAI)**: Contrastive learning connecting images and text
**2022 - DALL-E 2**: Diffusion-based model with CLIP embeddings
**2022 - Imagen (Google)**: Large language model encoders + cascaded diffusion
**2022 - Stable Diffusion**: Open-source latent diffusion model
**2023 - Midjourney v5**: Photorealistic quality, artistic control
**2023 - DALL-E 3**: Improved prompt following and safety
**2024-Present**: Video generation, 3D synthesis, real-time generation

### Key Challenges

**Semantic Understanding**: Interpreting complex, nuanced prompts
**Compositional Generation**: Correctly composing multiple objects with specified attributes
**Consistency**: Maintaining coherent style and lighting
**Fine Details**: Generating realistic textures, hands, text
**Bias and Safety**: Addressing biases, preventing harmful content

## 6.2 Core Architectures

### CLIP: Connecting Vision and Language

CLIP (Contrastive Language-Image Pre-training) learns a shared embedding space for images and text:

```python
class CLIP(nn.Module):
    def __init__(self, vision_encoder, text_encoder, embed_dim=512):
        super().__init__()

        self.vision_encoder = vision_encoder  # e.g., Vision Transformer
        self.text_encoder = text_encoder      # e.g., Transformer

        # Projection heads
        self.vision_projection = nn.Linear(vision_encoder.output_dim, embed_dim)
        self.text_projection = nn.Linear(text_encoder.output_dim, embed_dim)

        # Learnable temperature parameter
        self.temperature = nn.Parameter(torch.ones([]) * np.log(1 / 0.07))

    def forward(self, images, texts):
        # Encode
        image_features = self.vision_encoder(images)
        text_features = self.text_encoder(texts)

        # Project to shared embedding space
        image_embeds = self.vision_projection(image_features)
        text_embeds = self.text_projection(text_features)

        # Normalize
        image_embeds = F.normalize(image_embeds, dim=-1)
        text_embeds = F.normalize(text_embeds, dim=-1)

        return image_embeds, text_embeds

    def contrastive_loss(self, image_embeds, text_embeds):
        """Symmetric cross-entropy loss"""
        # Cosine similarity
        logits = image_embeds @ text_embeds.T * torch.exp(self.temperature)

        # Labels (diagonal matches)
        batch_size = image_embeds.shape[0]
        labels = torch.arange(batch_size, device=image_embeds.device)

        # Symmetric loss
        loss_i2t = F.cross_entropy(logits, labels)
        loss_t2i = F.cross_entropy(logits.T, labels)

        return (loss_i2t + loss_t2i) / 2
```

**Training Process**:
1. Sample image-text pairs from dataset
2. Encode both into embeddings
3. Maximize similarity for matching pairs
4. Minimize similarity for non-matching pairs

**Applications**:
- Zero-shot image classification
- Image retrieval from text
- Text-conditioned image generation
- Multimodal understanding

### DALL-E 2 Architecture

DALL-E 2 combines CLIP with diffusion models:

**Components**:
1. **Text Encoder**: CLIP text encoder produces text embeddings
2. **Prior**: Transforms text embeddings into image embeddings (diffusion or autoregressive)
3. **Decoder**: Diffusion model generates images conditioned on image embeddings

```python
class DALLE2(nn.Module):
    def __init__(self, clip_model, prior_model, decoder_model):
        super().__init__()

        self.clip = clip_model
        self.prior = prior_model    # Text embedding -> Image embedding
        self.decoder = decoder_model  # Image embedding -> Pixel image

    def generate(self, text, num_images=1):
        # Encode text
        with torch.no_grad():
            text_embeds = self.clip.encode_text(text)

        # Prior: predict CLIP image embedding from text embedding
        image_embeds = self.prior.sample(text_embeds, num_samples=num_images)

        # Decoder: generate image from image embedding
        images = self.decoder.sample(image_embeds)

        return images
```

**Prior Network** (Autoregressive):
```python
class AutoregressivePrior(nn.Module):
    def __init__(self, text_embed_dim, image_embed_dim, num_tokens):
        super().__init__()

        # Quantize image embeddings into discrete tokens
        self.vq = VectorQuantizer(num_tokens, image_embed_dim)

        # Transformer to predict image tokens from text
        self.transformer = Transformer(
            input_dim=text_embed_dim,
            output_dim=num_tokens,
            num_layers=24
        )

    def forward(self, text_embeds, image_embeds):
        # Quantize target image embeddings
        image_tokens, _ = self.vq.encode(image_embeds)

        # Predict image tokens autoregressively
        logits = self.transformer(text_embeds, image_tokens)

        return logits
```

**Prior Network** (Diffusion):
```python
class DiffusionPrior(nn.Module):
    def __init__(self, text_embed_dim, image_embed_dim):
        super().__init__()

        self.diffusion = GaussianDiffusion(
            model=UNet1D(
                in_dim=image_embed_dim,
                cond_dim=text_embed_dim
            ),
            timesteps=1000
        )

    def forward(self, text_embeds, image_embeds):
        # Diffusion loss for generating image embeddings
        loss = self.diffusion.training_loss(image_embeds, cond=text_embeds)
        return loss

    @torch.no_grad()
    def sample(self, text_embeds, num_samples=1):
        # Generate image embeddings via denoising
        image_embeds = self.diffusion.sample(
            shape=(num_samples, self.image_embed_dim),
            cond=text_embeds
        )
        return image_embeds
```

### Stable Diffusion Architecture

Stable Diffusion uses latent diffusion with cross-attention to text:

```python
class StableDiffusion(nn.Module):
    def __init__(self, vae, text_encoder, unet, scheduler):
        super().__init__()

        self.vae = vae                  # VAE for latent space
        self.text_encoder = text_encoder  # CLIP text encoder
        self.unet = unet                # U-Net with cross-attention
        self.scheduler = scheduler      # DDIM/DDPM scheduler

    @torch.no_grad()
    def generate(self, prompt, num_inference_steps=50, guidance_scale=7.5,
                 height=512, width=512):
        # Encode prompt
        text_embeddings = self.text_encoder(prompt)

        # Unconditional embeddings for classifier-free guidance
        uncond_embeddings = self.text_encoder("")

        # Concatenate for classifier-free guidance
        text_embeddings = torch.cat([uncond_embeddings, text_embeddings])

        # Initialize latents
        latents = torch.randn(
            1, 4, height // 8, width // 8,
            device=self.device
        )

        # Set scheduler timesteps
        self.scheduler.set_timesteps(num_inference_steps)

        # Denoising loop
        for t in self.scheduler.timesteps:
            # Expand latents for classifier-free guidance
            latent_input = torch.cat([latents] * 2)

            # Predict noise
            noise_pred = self.unet(latent_input, t, text_embeddings)

            # Classifier-free guidance
            noise_pred_uncond, noise_pred_text = noise_pred.chunk(2)
            noise_pred = noise_pred_uncond + guidance_scale * (
                noise_pred_text - noise_pred_uncond
            )

            # Denoise step
            latents = self.scheduler.step(noise_pred, t, latents).prev_sample

        # Decode latents to image
        latents = latents / 0.18215
        images = self.vae.decode(latents).sample

        return images
```

**Cross-Attention in U-Net**:
```python
class CrossAttentionBlock(nn.Module):
    def __init__(self, query_dim, context_dim, num_heads=8):
        super().__init__()

        self.num_heads = num_heads
        self.head_dim = query_dim // num_heads

        self.to_q = nn.Linear(query_dim, query_dim)
        self.to_k = nn.Linear(context_dim, query_dim)
        self.to_v = nn.Linear(context_dim, query_dim)
        self.to_out = nn.Linear(query_dim, query_dim)

    def forward(self, x, context):
        """
        x: image features (batch, height*width, query_dim)
        context: text embeddings (batch, seq_len, context_dim)
        """
        batch_size = x.shape[0]

        # Compute Q, K, V
        q = self.to_q(x)
        k = self.to_k(context)
        v = self.to_v(context)

        # Reshape for multi-head attention
        q = q.reshape(batch_size, -1, self.num_heads, self.head_dim).transpose(1, 2)
        k = k.reshape(batch_size, -1, self.num_heads, self.head_dim).transpose(1, 2)
        v = v.reshape(batch_size, -1, self.num_heads, self.head_dim).transpose(1, 2)

        # Attention
        attn = torch.matmul(q, k.transpose(-2, -1)) / math.sqrt(self.head_dim)
        attn = F.softmax(attn, dim=-1)
        out = torch.matmul(attn, v)

        # Reshape and project
        out = out.transpose(1, 2).reshape(batch_size, -1, self.num_heads * self.head_dim)
        out = self.to_out(out)

        return out
```

## 6.3 Prompt Engineering

Effective prompts dramatically improve generation quality.

### Descriptive Elements

**Subject**: Main focus
```
"A majestic lion"
```

**Style**: Artistic style
```
"A majestic lion, oil painting style"
```

**Details**: Specific attributes
```
"A majestic lion with golden mane, piercing amber eyes, oil painting style"
```

**Setting**: Environment and context
```
"A majestic lion with golden mane, piercing amber eyes, standing on African savanna at sunset, oil painting style"
```

**Lighting**: Illumination
```
"A majestic lion with golden mane, piercing amber eyes, standing on African savanna at sunset, warm golden hour lighting, oil painting style"
```

**Quality Modifiers**: Technical specs
```
"A majestic lion with golden mane, piercing amber eyes, standing on African savanna at sunset, warm golden hour lighting, oil painting style, highly detailed, 8k resolution, masterpiece"
```

### Negative Prompts

Specify what to avoid:

```python
positive_prompt = "Beautiful portrait of a woman, professional photography"

negative_prompt = "blurry, distorted, low quality, ugly, deformed, watermark, text, signature"
```

### Weighted Prompts

Some systems support weight modifiers:

```
"(beautiful sunset:1.5), mountains, lake, (reflections:0.8)"
```

Higher weights emphasize concepts.

### Prompt Templates

**Photography**:
```
"[subject], [shot type], [camera settings], [lighting], professional photography, 8k, highly detailed"
```

**Artwork**:
```
"[subject], [art style], by [artist name], [medium], [mood/atmosphere], masterpiece, trending on artstation"
```

**Concept Art**:
```
"[subject], concept art, [environment], [lighting], [color palette], detailed, matte painting, fantasy art"
```

## 6.4 Advanced Techniques

### ControlNet

Add spatial control through additional inputs:

```python
class ControlNet(nn.Module):
    def __init__(self, unet):
        super().__init__()

        # Copy U-Net encoder
        self.control_encoder = copy.deepcopy(unet.encoder)

        # Zero convolutions for adding control
        self.zero_convs = nn.ModuleList([
            nn.Conv2d(channels, channels, 1)
            for channels in unet.encoder_channels
        ])

        # Initialize zero convs to zero
        for conv in self.zero_convs:
            nn.init.zeros_(conv.weight)
            nn.init.zeros_(conv.bias)

    def forward(self, x, control_image):
        # Process control image through encoder
        control_features = self.control_encoder(control_image)

        # Apply zero convolutions
        control_features = [
            zero_conv(feat)
            for feat, zero_conv in zip(control_features, self.zero_convs)
        ]

        return control_features
```

**Control Types**:
- Canny edges
- Depth maps
- Pose keypoints
- Segmentation maps
- Scribbles

### DreamBooth

Personalize models with few images:

```python
def dreambooth_training(model, subject_images, unique_identifier,
                       class_name, num_epochs=800):
    """
    subject_images: 3-5 images of the subject
    unique_identifier: e.g., "sks" (rare token)
    class_name: e.g., "dog", "person"
    """

    optimizer = torch.optim.AdamW(model.unet.parameters(), lr=1e-6)

    for epoch in range(num_epochs):
        # Instance loss (with unique identifier)
        instance_prompt = f"a photo of {unique_identifier} {class_name}"
        instance_loss = model.training_step(subject_images, instance_prompt)

        # Class loss (regularization)
        class_prompt = f"a photo of a {class_name}"
        generated_class_images = model.generate(class_prompt, num_samples=4)
        class_loss = model.training_step(generated_class_images, class_prompt)

        # Combined loss
        loss = instance_loss + class_loss

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    return model
```

After training, generate with: `"a photo of sks dog in Paris"`

### Textual Inversion

Learn new tokens for concepts:

```python
class TextualInversion:
    def __init__(self, model, tokenizer, placeholder_token="<new-concept>"):
        self.model = model
        self.tokenizer = tokenizer
        self.placeholder_token = placeholder_token

        # Add placeholder token
        num_added_tokens = tokenizer.add_tokens(placeholder_token)
        token_id = tokenizer.convert_tokens_to_ids(placeholder_token)

        # Initialize embedding (copy existing token)
        model.text_encoder.resize_token_embeddings(len(tokenizer))
        token_embeds = model.text_encoder.get_input_embeddings().weight.data
        token_embeds[token_id] = token_embeds[token_id - 1]  # Copy similar token

        # Only train this embedding
        for param in model.parameters():
            param.requires_grad = False
        model.text_encoder.get_input_embeddings().weight.requires_grad = True

    def train(self, concept_images, num_epochs=5000):
        optimizer = torch.optim.Adam(
            model.text_encoder.get_input_embeddings().parameters(),
            lr=5e-4
        )

        for epoch in range(num_epochs):
            prompt = f"a photo of {self.placeholder_token}"
            loss = self.model.training_step(concept_images, prompt)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
```

### LoRA for Diffusion Models

Efficient fine-tuning:

```python
from peft import LoraConfig, get_peft_model

lora_config = LoraConfig(
    r=4,  # Rank
    lora_alpha=16,
    target_modules=["to_q", "to_k", "to_v", "to_out"],  # Attention layers
    lora_dropout=0.1
)

# Apply LoRA to U-Net
model.unet = get_peft_model(model.unet, lora_config)

# Only ~1% parameters are trainable
trainable_params = sum(p.numel() for p in model.unet.parameters() if p.requires_grad)
print(f"Trainable parameters: {trainable_params:,}")
```

## 6.5 Image Editing

### Inpainting

```python
def inpaint(model, image, mask, prompt, num_inference_steps=50):
    """
    image: Original image
    mask: Binary mask (1 = inpaint, 0 = keep)
    prompt: Text description of desired content
    """

    # Encode image to latent
    latent = model.vae.encode(image).latent_dist.sample()

    # Encode prompt
    text_emb = model.text_encoder(prompt)

    # Initialize with noise in masked region
    noise = torch.randn_like(latent)
    masked_latent = latent * (1 - mask) + noise * mask

    # Diffusion process
    for t in model.scheduler.timesteps:
        # Predict noise
        noise_pred = model.unet(masked_latent, t, text_emb)

        # Denoise
        masked_latent = model.scheduler.step(noise_pred, t, masked_latent).prev_sample

        # Keep unmasked regions from original
        masked_latent = latent * (1 - mask) + masked_latent * mask

    # Decode
    result = model.vae.decode(masked_latent).sample
    return result
```

### Image-to-Image

Transform images based on prompts:

```python
def image_to_image(model, image, prompt, strength=0.8, num_inference_steps=50):
    """
    strength: How much to transform (0 = no change, 1 = complete transformation)
    """

    # Encode image to latent
    latent = model.vae.encode(image).latent_dist.sample()

    # Calculate start step based on strength
    start_step = int(num_inference_steps * (1 - strength))
    model.scheduler.set_timesteps(num_inference_steps)
    timesteps = model.scheduler.timesteps[start_step:]

    # Add noise to latent
    noise = torch.randn_like(latent)
    latent = model.scheduler.add_noise(latent, noise, timesteps[0])

    # Encode prompt
    text_emb = model.text_encoder(prompt)

    # Denoise with prompt guidance
    for t in timesteps:
        noise_pred = model.unet(latent, t, text_emb)
        latent = model.scheduler.step(noise_pred, t, latent).prev_sample

    # Decode
    result = model.vae.decode(latent).sample
    return result
```

## 6.6 Evaluation and Quality

### Automated Metrics

**CLIP Score**: Measures text-image alignment
```python
def clip_score(images, prompts, clip_model):
    image_embeds = clip_model.encode_image(images)
    text_embeds = clip_model.encode_text(prompts)
    scores = (image_embeds * text_embeds).sum(dim=-1)
    return scores.mean()
```

**FID**: Measures visual quality
**Inception Score**: Measures diversity and quality

### Human Evaluation

- **Prompt Adherence**: Does image match description?
- **Realism**: How photorealistic/believable?
- **Composition**: Is layout and arrangement good?
- **Details**: Are fine details correct?
- **Aesthetics**: Overall visual appeal

## Summary

Text-to-image generation combines language understanding with visual synthesis to create images from descriptions. Modern systems use diffusion models with sophisticated text conditioning, enabling unprecedented creative control.

**Key Takeaways**:
- CLIP connects vision and language through contrastive learning
- DALL-E 2 and Stable Diffusion use diffusion with text conditioning
- Effective prompting dramatically improves results
- Advanced techniques (ControlNet, DreamBooth) enable precise control
- Applications span art, design, content creation, and beyond
- Challenges include composition, consistency, and ethical considerations

---

**弘益人間 (Benefit All Humanity)** - Visual creation tools have always amplified human expression. As we build AI that generates images from imagination, let us ensure these tools empower creativity for all, respect artistic integrity, and expand—not replace—human creative potential.

---

*Previous: [Chapter 5 - LLMs](./05-large-language-models.md) | Next: [Chapter 7 - Multimodal Generation](./07-multimodal-generation.md)*

---

© 2025 SmileStory Inc. / WIA | WIA-AI-026 Generative AI Standard
