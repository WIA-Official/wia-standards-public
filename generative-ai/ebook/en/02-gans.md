# Chapter 2: Generative Adversarial Networks (GANs)

## 2.1 The GAN Revolution

In 2014, Ian Goodfellow and colleagues introduced Generative Adversarial Networks (GANs), fundamentally changing how we approach generative modeling. The key insight was beautifully simple yet profoundly powerful: train two neural networks in opposition, with one generating samples and the other evaluating their authenticity.

This adversarial framework mirrors many real-world scenarios—an art forger trying to create convincing fakes while an expert authenticator learns to detect forgeries. As both improve through competition, the forger's work becomes increasingly indistinguishable from genuine art.

### The Core Concept

GANs consist of two components:

**Generator (G)**: Takes random noise as input and produces synthetic data samples. Its goal is to create outputs so realistic that they fool the discriminator.

**Discriminator (D)**: Receives both real data from the training set and synthetic data from the generator. Its goal is to correctly classify each sample as real or fake.

These networks engage in a **minimax game**:
- The generator tries to maximize the discriminator's error rate
- The discriminator tries to minimize its classification error
- Training alternates between updating each network

The mathematical formulation:

```
min_G max_D V(D,G) = E_x~pdata[log D(x)] + E_z~pz[log(1 - D(G(z)))]
```

Where:
- `E_x~pdata[log D(x)]`: Expected log-probability that D correctly identifies real data
- `E_z~pz[log(1 - D(G(z)))]`: Expected log-probability that D correctly identifies generated data as fake
- `z`: Random noise vector (latent code)
- `pdata`: Real data distribution
- `pz`: Noise distribution (typically Gaussian)

## 2.2 GAN Architecture and Training

Understanding how to build and train GANs requires careful consideration of network architecture, loss functions, and training dynamics.

### Basic Architecture

**Generator Network**:
```python
class Generator(nn.Module):
    def __init__(self, latent_dim=100, img_channels=3, img_size=64):
        super(Generator, self).__init__()

        self.init_size = img_size // 16
        self.l1 = nn.Linear(latent_dim, 128 * self.init_size ** 2)

        self.conv_blocks = nn.Sequential(
            nn.BatchNorm2d(128),
            nn.Upsample(scale_factor=2),
            nn.Conv2d(128, 128, 3, stride=1, padding=1),
            nn.BatchNorm2d(128, 0.8),
            nn.LeakyReLU(0.2, inplace=True),

            nn.Upsample(scale_factor=2),
            nn.Conv2d(128, 64, 3, stride=1, padding=1),
            nn.BatchNorm2d(64, 0.8),
            nn.LeakyReLU(0.2, inplace=True),

            nn.Upsample(scale_factor=2),
            nn.Conv2d(64, 32, 3, stride=1, padding=1),
            nn.BatchNorm2d(32, 0.8),
            nn.LeakyReLU(0.2, inplace=True),

            nn.Upsample(scale_factor=2),
            nn.Conv2d(32, img_channels, 3, stride=1, padding=1),
            nn.Tanh()
        )

    def forward(self, z):
        out = self.l1(z)
        out = out.view(out.shape[0], 128, self.init_size, self.init_size)
        img = self.conv_blocks(out)
        return img
```

**Discriminator Network**:
```python
class Discriminator(nn.Module):
    def __init__(self, img_channels=3, img_size=64):
        super(Discriminator, self).__init__()

        def discriminator_block(in_filters, out_filters, bn=True):
            block = [
                nn.Conv2d(in_filters, out_filters, 3, 2, 1),
                nn.LeakyReLU(0.2, inplace=True),
                nn.Dropout2d(0.25)
            ]
            if bn:
                block.append(nn.BatchNorm2d(out_filters, 0.8))
            return block

        self.model = nn.Sequential(
            *discriminator_block(img_channels, 16, bn=False),
            *discriminator_block(16, 32),
            *discriminator_block(32, 64),
            *discriminator_block(64, 128),
        )

        # The height and width of downsampled image
        ds_size = img_size // 2 ** 4
        self.adv_layer = nn.Sequential(
            nn.Linear(128 * ds_size ** 2, 1),
            nn.Sigmoid()
        )

    def forward(self, img):
        out = self.model(img)
        out = out.view(out.shape[0], -1)
        validity = self.adv_layer(out)
        return validity
```

### Training Loop

The GAN training process alternates between updating the discriminator and generator:

```python
def train_gan(generator, discriminator, dataloader, num_epochs, device):
    # Loss function
    adversarial_loss = nn.BCELoss()

    # Optimizers
    optimizer_G = torch.optim.Adam(generator.parameters(), lr=0.0002, betas=(0.5, 0.999))
    optimizer_D = torch.optim.Adam(discriminator.parameters(), lr=0.0002, betas=(0.5, 0.999))

    for epoch in range(num_epochs):
        for i, real_imgs in enumerate(dataloader):
            batch_size = real_imgs.size(0)
            real_imgs = real_imgs.to(device)

            # Adversarial ground truths
            valid = torch.ones(batch_size, 1).to(device)
            fake = torch.zeros(batch_size, 1).to(device)

            # ---------------------
            #  Train Discriminator
            # ---------------------
            optimizer_D.zero_grad()

            # Loss for real images
            real_loss = adversarial_loss(discriminator(real_imgs), valid)

            # Generate fake images
            z = torch.randn(batch_size, latent_dim).to(device)
            gen_imgs = generator(z)

            # Loss for fake images
            fake_loss = adversarial_loss(discriminator(gen_imgs.detach()), fake)

            # Total discriminator loss
            d_loss = (real_loss + fake_loss) / 2
            d_loss.backward()
            optimizer_D.step()

            # -----------------
            #  Train Generator
            # -----------------
            optimizer_G.zero_grad()

            # Generate fake images
            z = torch.randn(batch_size, latent_dim).to(device)
            gen_imgs = generator(z)

            # Generator loss (tries to fool discriminator)
            g_loss = adversarial_loss(discriminator(gen_imgs), valid)
            g_loss.backward()
            optimizer_G.step()

            # Print progress
            if i % 100 == 0:
                print(f"[Epoch {epoch}/{num_epochs}] [Batch {i}/{len(dataloader)}] "
                      f"[D loss: {d_loss.item():.4f}] [G loss: {g_loss.item():.4f}]")
```

### Training Challenges

GAN training is notoriously unstable. Several challenges commonly arise:

**Mode Collapse**: The generator produces limited variety, generating only a few types of outputs regardless of input noise. This happens when the generator finds a single or few outputs that consistently fool the discriminator.

**Vanishing Gradients**: When the discriminator becomes too good, it provides gradients near zero to the generator, preventing learning.

**Training Instability**: The adversarial training process can oscillate without converging, or one network can overpower the other.

**Non-Convergence**: Unlike typical loss minimization, GANs seek an equilibrium between two competing objectives, which doesn't guarantee convergence.

## 2.3 Advanced GAN Architectures

Since the original GAN, numerous variants have been developed to address limitations and extend capabilities.

### Deep Convolutional GAN (DCGAN)

DCGAN introduced architectural guidelines that significantly improved GAN stability:

**Key Innovations**:
- Replace pooling layers with strided convolutions (discriminator) and fractional-strided convolutions (generator)
- Use batch normalization in both networks
- Remove fully connected hidden layers
- Use ReLU activation in generator (except output layer using Tanh)
- Use LeakyReLU activation in discriminator

These guidelines became standard practice for image generation GANs.

### Conditional GAN (cGAN)

cGANs extend GANs to conditional generative modeling by providing additional information (labels, attributes, or other data) to both generator and discriminator.

```python
class ConditionalGenerator(nn.Module):
    def __init__(self, latent_dim, num_classes, img_channels, img_size):
        super(ConditionalGenerator, self).__init__()

        self.label_emb = nn.Embedding(num_classes, latent_dim)

        self.model = nn.Sequential(
            nn.Linear(latent_dim * 2, 128 * (img_size // 16) ** 2),
            nn.ReLU(),
            # ... rest of generator architecture
        )

    def forward(self, noise, labels):
        # Concatenate label embedding and noise
        gen_input = torch.cat((self.label_emb(labels), noise), -1)
        img = self.model(gen_input)
        return img
```

Applications include:
- Generating images of specific classes (e.g., "generate a cat" vs "generate a dog")
- Text-to-image generation
- Image-to-image translation with semantic control

### Wasserstein GAN (WGAN)

WGAN addresses training instability by using the Wasserstein distance instead of Jensen-Shannon divergence:

**Key Changes**:
- Remove sigmoid from discriminator output (now called "critic")
- Use Wasserstein loss instead of binary cross-entropy
- Clip critic weights to enforce Lipschitz constraint
- Train critic more iterations per generator iteration

```python
# WGAN Loss
def wasserstein_loss(y_true, y_pred):
    return torch.mean(y_true * y_pred)

# Training loop modification
for _ in range(n_critic):
    # Train critic
    real_validity = critic(real_imgs)
    fake_validity = critic(gen_imgs.detach())

    critic_loss = -torch.mean(real_validity) + torch.mean(fake_validity)

    # Weight clipping
    for p in critic.parameters():
        p.data.clamp_(-clip_value, clip_value)
```

**WGAN-GP** (Gradient Penalty) improves on WGAN by replacing weight clipping with a gradient penalty term, further stabilizing training.

### StyleGAN and StyleGAN2

StyleGAN revolutionized high-quality image generation through style-based architecture:

**Innovations**:
- **Mapping Network**: Transforms latent code z into intermediate latent space W
- **Adaptive Instance Normalization (AdaIN)**: Controls style at each layer
- **Style Mixing**: Different styles can be applied at different resolutions
- **Stochastic Variation**: Random noise injection for fine details

```python
class StyleGAN_Generator(nn.Module):
    def __init__(self, latent_dim=512, n_mapping=8):
        super().__init__()

        # Mapping network: Z -> W
        layers = []
        for i in range(n_mapping):
            layers.append(nn.Linear(latent_dim, latent_dim))
            layers.append(nn.LeakyReLU(0.2))
        self.mapping = nn.Sequential(*layers)

        # Synthesis network with AdaIN
        self.synthesis = SynthesisNetwork(latent_dim)

    def forward(self, z):
        w = self.mapping(z)
        img = self.synthesis(w)
        return img
```

StyleGAN enables:
- Unprecedented control over generated images
- Smooth interpolation between different attributes
- Fine-grained editing of specific features
- Extremely high-resolution generation (1024x1024 and beyond)

### Progressive GAN

Progressive GAN grows the generator and discriminator progressively, starting from low resolution and gradually adding layers for higher resolution:

**Training Process**:
1. Start with 4x4 resolution
2. Train until stable
3. Add layers for 8x8, fade in gradually
4. Repeat until reaching target resolution (e.g., 1024x1024)

This approach:
- Stabilizes training of high-resolution GANs
- Speeds up training by focusing on coarse features first
- Produces higher quality results

### CycleGAN

CycleGAN performs unpaired image-to-image translation without requiring matched training pairs:

**Architecture**:
- Two generators: G: X→Y and F: Y→X
- Two discriminators: D_X and D_Y
- Cycle consistency loss: ensures F(G(x)) ≈ x and G(F(y)) ≈ y

```python
# Cycle consistency loss
cycle_loss = lambda_cycle * (
    torch.mean(torch.abs(real_X - F(G(real_X)))) +
    torch.mean(torch.abs(real_Y - G(F(real_Y))))
)

# Identity loss (optional)
identity_loss = lambda_identity * (
    torch.mean(torch.abs(real_Y - G(real_Y))) +
    torch.mean(torch.abs(real_X - F(real_X)))
)
```

Applications:
- Style transfer (photo ↔ painting)
- Season transfer (summer ↔ winter)
- Domain adaptation (synthetic ↔ real images)
- Object transfiguration (horses ↔ zebras)

## 2.4 Evaluation Metrics

Evaluating GAN performance is challenging because there's no single ground truth for "good" generation. Several metrics have been developed:

### Inception Score (IS)

Measures both quality and diversity of generated images:

```python
def inception_score(imgs, splits=10):
    # Get Inception predictions
    preds = inception_model(imgs)

    # Calculate score
    scores = []
    for i in range(splits):
        part = preds[i * (len(preds) // splits): (i + 1) * (len(preds) // splits)]
        kl_div = part * (np.log(part) - np.log(np.expand_dims(np.mean(part, 0), 0)))
        kl_div = np.mean(np.sum(kl_div, 1))
        scores.append(np.exp(kl_div))

    return np.mean(scores), np.std(scores)
```

**Properties**:
- Higher is better
- Requires generated images to be recognizable by Inception network
- Doesn't compare to real data distribution

### Fréchet Inception Distance (FID)

Compares distribution of generated images to real images using Inception network features:

```python
def calculate_fid(real_images, generated_images):
    # Get Inception features
    real_features = inception_features(real_images)
    gen_features = inception_features(generated_images)

    # Calculate mean and covariance
    mu_real, sigma_real = real_features.mean(axis=0), np.cov(real_features, rowvar=False)
    mu_gen, sigma_gen = gen_features.mean(axis=0), np.cov(gen_features, rowvar=False)

    # Calculate Fréchet distance
    ssdiff = np.sum((mu_real - mu_gen) ** 2)
    covmean = scipy.linalg.sqrtm(sigma_real.dot(sigma_gen))

    if np.iscomplexobj(covmean):
        covmean = covmean.real

    fid = ssdiff + np.trace(sigma_real + sigma_gen - 2 * covmean)
    return fid
```

**Properties**:
- Lower is better
- More robust to noise than IS
- Correlates well with human judgment
- Most widely used metric for image GANs

### Precision and Recall

Measures both fidelity (precision) and diversity (recall) of generated samples:

**Precision**: Fraction of generated samples that are realistic
**Recall**: Fraction of real data distribution covered by generated samples

This provides more nuanced evaluation than single-number metrics.

## 2.5 Practical Applications

GANs have found applications across numerous domains:

### Image Synthesis and Editing

**Face Generation**: Creating photorealistic faces of non-existent people (ThisPersonDoesNotExist.com)

**Image Super-Resolution**: Enhancing low-resolution images to high resolution

**Image Inpainting**: Filling in missing or corrupted parts of images

**Semantic Image Editing**: Changing image attributes (age, expression, hair color) while preserving identity

### Data Augmentation

Generating synthetic training data for:
- Medical imaging (rare conditions)
- Autonomous vehicles (edge cases)
- Security systems (synthetic scenarios)
- Fashion and retail (product variations)

### Creative Tools

**Art and Design**: Assisting artists with style exploration and concept generation

**Fashion**: Generating clothing designs, virtual try-on systems

**Gaming**: Creating textures, characters, and environments

**Architecture**: Generating building designs and interior layouts

### Scientific Applications

**Drug Discovery**: Generating molecular structures with desired properties

**Materials Science**: Designing new materials with specific characteristics

**Astronomy**: Generating realistic astronomical images for training

### Privacy-Preserving Synthetic Data

Creating synthetic datasets that preserve statistical properties while protecting individual privacy:
- Medical records
- Financial transactions
- User behavior patterns

## 2.6 Challenges and Limitations

Despite their power, GANs have inherent limitations:

### Training Difficulty

- Requires careful hyperparameter tuning
- Sensitive to architectural choices
- No guarantee of convergence
- Computationally expensive

### Mode Collapse

Even with improvements, mode collapse remains a challenge, especially when:
- Dataset is highly diverse
- Training is not properly balanced
- Architecture is insufficient

### Lack of Explicit Likelihood

Unlike VAEs, GANs don't provide explicit probability densities, making it harder to:
- Evaluate sample quality theoretically
- Perform certain types of inference
- Compare models rigorously

### Limited Controllability

While conditional GANs improve control, precisely specifying desired outputs remains challenging, especially for complex attributes.

### Evaluation Challenges

- Metrics don't always align with human perception
- Different metrics may give conflicting assessments
- Difficult to capture all aspects of generation quality

## Summary

Generative Adversarial Networks introduced adversarial training as a powerful paradigm for generative modeling. Through competition between generator and discriminator, GANs can produce remarkably realistic synthetic data across many domains.

**Key Takeaways**:
- GANs use adversarial training between generator and discriminator networks
- Numerous variants address stability, controllability, and application-specific needs
- Training requires careful tuning and monitoring
- Evaluation uses metrics like FID, IS, and precision/recall
- Applications span creative tools, data augmentation, and scientific discovery
- Challenges include training instability, mode collapse, and evaluation difficulty

While GANs have limitations, they remain a cornerstone of generative AI, particularly for high-quality image generation and creative applications.

## Review Questions

1. **Architecture**
   - Explain the roles of the generator and discriminator in a GAN
   - Why is batch normalization important in GAN architectures?
   - What are the key architectural improvements introduced by DCGAN?

2. **Training**
   - Describe the minimax objective function in GANs
   - What is mode collapse and why does it occur?
   - How does WGAN address training instability?

3. **Variants**
   - Compare and contrast conditional GANs with unconditional GANs
   - How does StyleGAN achieve fine-grained control over generated images?
   - What problem does CycleGAN solve that traditional GANs cannot?

4. **Evaluation**
   - Why is evaluating GAN performance challenging?
   - Explain the difference between Inception Score and FID
   - What do precision and recall measure in the context of GANs?

5. **Applications**
   - Describe three practical applications of GANs
   - How can GANs be used for data augmentation?
   - What are the privacy implications of synthetic data generation?

## Practical Exercise

Implement a simple DCGAN to generate images from a dataset of your choice (MNIST, CIFAR-10, or a custom dataset):

1. Build the generator and discriminator architectures
2. Implement the training loop with proper loss functions
3. Train for at least 50 epochs, saving generated samples periodically
4. Visualize the training progress (both losses and generated samples)
5. Calculate FID score comparing generated and real images
6. Experiment with hyperparameters (learning rate, batch size, architecture depth)
7. Document challenges encountered and solutions attempted

**Bonus**: Implement one variant (cGAN, WGAN, or StyleGAN) and compare results.

---

**弘익人間 (Benefit All Humanity)** - GANs exemplify how competition can drive improvement. As we develop these powerful tools, let us ensure they are used to create value—generating synthetic medical data to advance healthcare, creating artistic tools that amplify human creativity, and building systems that respect privacy while advancing science.

---

*Previous: [Chapter 1 - Introduction](./01-introduction.md) | Next: [Chapter 3 - Variational Autoencoders (VAEs)](./03-vaes.md)*

---

© 2025 SmileStory Inc. / WIA | WIA-AI-026 Generative AI Standard
