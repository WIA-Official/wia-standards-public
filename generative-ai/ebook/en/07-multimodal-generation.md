# Chapter 7: Multimodal Generation

## 7.1 Beyond Single Modalities

The future of generative AI lies in multimodal systems that seamlessly understand and generate across text, images, audio, video, and 3D representations. Where earlier models specialized in one modality, modern systems integrate multiple modalities, enabling richer, more natural AI interactions and creative possibilities.

**Multimodal capabilities include**:
- Understanding: Processing inputs across modalities (vision + language)
- Generation: Creating outputs in multiple formats
- Translation: Converting between modalities (text→image, image→text)
- Reasoning: Combining information from different sources

### Why Multimodal Matters

**Natural Communication**: Humans communicate through multiple channels—speech, gestures, images, text. Multimodal AI better matches human communication.

**Richer Context**: Combining modalities provides more complete understanding than any single modality alone.

**Creative Flexibility**: Generate cohesive content across formats—scripts with storyboards, music with visualizations, code with documentation.

**Accessibility**: Transform content between modalities to serve different needs and abilities.

## 7.2 Multimodal Understanding

### Vision-Language Models

Models that jointly understand images and text:

**CLIP (Reviewed)**:
```python
# Zero-shot image classification
image_features = clip.encode_image(image)
text_features = clip.encode_text(["a dog", "a cat", "a bird"])
similarity = image_features @ text_features.T
predicted_class = similarity.argmax()
```

**BLIP (Bootstrapped Language-Image Pretraining)**:
```python
class BLIP(nn.Module):
    def __init__(self, vision_encoder, text_encoder, fusion_encoder):
        super().__init__()

        self.vision_encoder = vision_encoder
        self.text_encoder = text_encoder
        self.fusion_encoder = fusion_encoder  # Multimodal fusion

    def forward(self, image, text):
        # Encode separately
        image_features = self.vision_encoder(image)
        text_features = self.text_encoder(text)

        # Fuse multimodal features
        fused_features = self.fusion_encoder(
            image_features,
            text_features,
            cross_attention=True
        )

        return fused_features

    def caption(self, image):
        """Generate caption for image"""
        image_features = self.vision_encoder(image)
        caption = self.text_decoder.generate(
            encoder_hidden_states=image_features
        )
        return caption

    def vqa(self, image, question):
        """Visual Question Answering"""
        fused = self.forward(image, question)
        answer = self.answer_head(fused)
        return answer
```

**GPT-4V (Vision)**:
Multimodal LLM accepting both images and text:

```python
# Pseudocode for GPT-4V style model
messages = [
    {
        "role": "user",
        "content": [
            {"type": "text", "text": "What's in this image?"},
            {"type": "image_url", "image_url": "https://..."}
        ]
    }
]

response = model.chat(messages)
# "The image shows a golden retriever playing in a park..."
```

### Audio-Language Models

**Whisper (Speech Recognition)**:
```python
class Whisper(nn.Module):
    def __init__(self, audio_encoder, text_decoder):
        super().__init__()

        self.audio_encoder = audio_encoder  # Conv + Transformer
        self.text_decoder = text_decoder    # Transformer decoder

    def transcribe(self, audio):
        # Encode audio to features
        audio_features = self.audio_encoder(audio)

        # Decode to text autoregressively
        text = self.text_decoder.generate(
            encoder_hidden_states=audio_features,
            language="<|en|>",  # Language token
            task="<|transcribe|>"  # Task token
        )

        return text

    def translate(self, audio, target_language="en"):
        audio_features = self.audio_encoder(audio)

        text = self.text_decoder.generate(
            encoder_hidden_states=audio_features,
            language=f"<|{target_language}|>",
            task="<|translate|>"
        )

        return text
```

### Video Understanding

**Video-LLMs**:
```python
class VideoLanguageModel(nn.Module):
    def __init__(self, frame_encoder, temporal_encoder, text_decoder):
        super().__init__()

        self.frame_encoder = frame_encoder        # Per-frame vision encoder
        self.temporal_encoder = temporal_encoder  # Temporal transformer
        self.text_decoder = text_decoder

    def forward(self, video_frames):
        """
        video_frames: (batch, num_frames, channels, height, width)
        """
        batch_size, num_frames = video_frames.shape[:2]

        # Encode each frame
        frame_features = []
        for i in range(num_frames):
            features = self.frame_encoder(video_frames[:, i])
            frame_features.append(features)

        frame_features = torch.stack(frame_features, dim=1)

        # Model temporal relationships
        video_features = self.temporal_encoder(frame_features)

        return video_features

    def caption_video(self, video_frames):
        video_features = self.forward(video_frames)
        caption = self.text_decoder.generate(
            encoder_hidden_states=video_features
        )
        return caption

    def answer_video_question(self, video_frames, question):
        video_features = self.forward(video_frames)
        question_features = self.text_encoder(question)

        # Fuse video and question
        combined = self.fusion(video_features, question_features)

        answer = self.answer_decoder.generate(combined)
        return answer
```

## 7.3 Unified Multimodal Models

### Flamingo Architecture

Freezes pretrained vision and language models, training only cross-attention:

```python
class FlamingoModel(nn.Module):
    def __init__(self, vision_encoder, language_model):
        super().__init__()

        # Frozen pretrained models
        self.vision_encoder = vision_encoder
        self.language_model = language_model

        for param in self.vision_encoder.parameters():
            param.requires_grad = False

        # Learnable cross-attention layers (gated)
        self.cross_attentions = nn.ModuleList([
            GatedCrossAttention(
                lang_dim=language_model.hidden_size,
                vision_dim=vision_encoder.output_dim
            )
            for _ in range(language_model.num_layers)
        ])

        # Perceiver resampler (compress vision features)
        self.perceiver = PerceiverResampler(
            vision_dim=vision_encoder.output_dim,
            num_latents=64
        )

    def forward(self, text_tokens, images=None):
        # Encode images if provided
        if images is not None:
            vision_features = self.vision_encoder(images)
            vision_features = self.perceiver(vision_features)
        else:
            vision_features = None

        # Process text through language model with cross-attention to vision
        hidden_states = self.language_model.embed_tokens(text_tokens)

        for layer_idx, lm_layer in enumerate(self.language_model.layers):
            # Standard self-attention
            hidden_states = lm_layer.self_attention(hidden_states)

            # Cross-attention to vision (if images present)
            if vision_features is not None:
                hidden_states = self.cross_attentions[layer_idx](
                    hidden_states,
                    vision_features
                )

            # Feed-forward
            hidden_states = lm_layer.feed_forward(hidden_states)

        logits = self.language_model.lm_head(hidden_states)
        return logits
```

**Gated Cross-Attention**:
```python
class GatedCrossAttention(nn.Module):
    def __init__(self, lang_dim, vision_dim):
        super().__init__()

        self.cross_attn = nn.MultiheadAttention(
            embed_dim=lang_dim,
            num_heads=8,
            kdim=vision_dim,
            vdim=vision_dim
        )

        # Gating mechanism (start from zero)
        self.gate = nn.Parameter(torch.zeros(1))

    def forward(self, language_features, vision_features):
        # Cross-attend: language queries, vision keys/values
        attended, _ = self.cross_attn(
            query=language_features,
            key=vision_features,
            value=vision_features
        )

        # Gated residual
        output = language_features + torch.tanh(self.gate) * attended

        return output
```

### Gemini Architecture

Google's multimodal model processes text, images, audio, and video natively:

**Key Innovations**:
- Unified tokenization across modalities
- Interleaved multimodal sequences
- Cross-modal attention throughout
- Native multimodal pretraining (not separate then fused)

```python
class GeminiStyle(nn.Module):
    """Conceptual Gemini-style architecture"""

    def __init__(self):
        super().__init__()

        # Modality-specific encoders
        self.text_embedding = nn.Embedding(vocab_size, embed_dim)
        self.image_encoder = VisionTransformer()
        self.audio_encoder = AudioTransformer()

        # Unified transformer
        self.transformer = Transformer(
            embed_dim=embed_dim,
            num_layers=32,
            num_heads=16
        )

        # Modality-specific decoders
        self.text_head = nn.Linear(embed_dim, vocab_size)
        self.image_head = ImageDecoder()
        self.audio_head = AudioDecoder()

    def encode(self, inputs):
        """
        inputs: List of dictionaries with 'type' and 'data'
        [
            {'type': 'text', 'data': 'Describe this:'},
            {'type': 'image', 'data': <image_tensor>},
            {'type': 'text', 'data': 'What do you see?'}
        ]
        """

        embeddings = []

        for item in inputs:
            if item['type'] == 'text':
                emb = self.text_embedding(item['data'])
            elif item['type'] == 'image':
                emb = self.image_encoder(item['data'])
            elif item['type'] == 'audio':
                emb = self.audio_encoder(item['data'])

            embeddings.append(emb)

        # Concatenate all modalities
        combined = torch.cat(embeddings, dim=1)

        return combined

    def forward(self, inputs, output_modality='text'):
        # Encode multimodal input
        encoded = self.encode(inputs)

        # Process through unified transformer
        hidden_states = self.transformer(encoded)

        # Generate in specified modality
        if output_modality == 'text':
            output = self.text_head(hidden_states)
        elif output_modality == 'image':
            output = self.image_head(hidden_states)
        elif output_modality == 'audio':
            output = self.audio_head(hidden_states)

        return output
```

## 7.4 Multimodal Generation Applications

### Video Generation

**Text-to-Video Models**:
```python
class TextToVideo(nn.Module):
    def __init__(self, text_encoder, video_diffusion):
        super().__init__()

        self.text_encoder = text_encoder
        self.video_diffusion = video_diffusion  # 3D U-Net for temporal coherence

    def generate(self, prompt, num_frames=16, height=512, width=512):
        # Encode text
        text_emb = self.text_encoder(prompt)

        # Initialize video latent
        video_latent = torch.randn(1, 4, num_frames, height // 8, width // 8)

        # Diffusion process
        for t in self.scheduler.timesteps:
            # Predict noise (3D convolutions for temporal consistency)
            noise_pred = self.video_diffusion(video_latent, t, text_emb)

            # Denoise step
            video_latent = self.scheduler.step(noise_pred, t, video_latent)

        # Decode to frames
        frames = self.vae_decoder(video_latent)

        return frames
```

**Applications**:
- Marketing video creation
- Animation and VFX
- Educational content
- Social media content

### Audio Generation

**Text-to-Speech (TTS)**:
```python
class TextToSpeech(nn.Module):
    def __init__(self, text_encoder, acoustic_model, vocoder):
        super().__init__()

        self.text_encoder = text_encoder      # Text → phonemes
        self.acoustic_model = acoustic_model  # Phonemes → mel spectrogram
        self.vocoder = vocoder                # Mel → waveform

    def forward(self, text):
        # Encode text
        phonemes = self.text_encoder(text)

        # Generate mel spectrogram
        mel = self.acoustic_model(phonemes)

        # Generate waveform
        audio = self.vocoder(mel)

        return audio
```

**Music Generation**:
```python
class MusicGenerator(nn.Module):
    def __init__(self):
        super().__init__()

        # Transformer for symbolic music (MIDI-like)
        self.symbolic_model = MusicTransformer()

        # Audio codec (compress audio to discrete tokens)
        self.audio_codec = EnCodec()

        # Transformer for audio tokens
        self.audio_model = AudioLM()

    def generate_from_text(self, description):
        """Generate music from text description"""

        # Text → music tokens (via diffusion or autoregressive)
        music_tokens = self.symbolic_model.generate(description)

        # Music tokens → audio tokens
        audio_tokens = self.audio_codec.encode(music_tokens)

        # Audio tokens → waveform
        audio = self.audio_codec.decode(audio_tokens)

        return audio
```

### 3D Generation

**Text-to-3D**:
```python
class TextTo3D(nn.Module):
    def __init__(self, text_encoder, nerf_model, diffusion_guidance):
        super().__init__()

        self.text_encoder = text_encoder
        self.nerf = nerf_model  # Neural Radiance Field
        self.diffusion = diffusion_guidance  # 2D diffusion for guidance

    def optimize_nerf(self, prompt, num_iterations=10000):
        """Optimize NeRF to match text description"""

        text_emb = self.text_encoder(prompt)
        optimizer = torch.optim.Adam(self.nerf.parameters(), lr=1e-3)

        for iteration in range(num_iterations):
            # Random camera viewpoint
            camera_pose = sample_random_camera()

            # Render image from NeRF
            rendered_image = self.nerf.render(camera_pose)

            # Score-Distillation Sampling (SDS) loss
            # Use 2D diffusion model to guide 3D generation
            with torch.no_grad():
                noise = torch.randn_like(rendered_image)
                t = random.randint(0, 1000)
                noisy_image = add_noise(rendered_image, noise, t)

                noise_pred = self.diffusion.unet(noisy_image, t, text_emb)

            # Gradient from diffusion model
            loss = F.mse_loss(noise_pred, noise)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

        return self.nerf

    def export_mesh(self):
        """Extract 3D mesh from trained NeRF"""
        # Marching cubes or similar
        vertices, faces = extract_mesh(self.nerf)
        return vertices, faces
```

## 7.5 Cross-Modal Translation

### Image Captioning

```python
class ImageCaptioner(nn.Module):
    def __init__(self, vision_encoder, text_decoder):
        super().__init__()

        self.vision_encoder = vision_encoder
        self.text_decoder = text_decoder

    def forward(self, image, caption_tokens):
        # Encode image
        image_features = self.vision_encoder(image)

        # Decode caption with cross-attention to image
        logits = self.text_decoder(
            caption_tokens,
            encoder_hidden_states=image_features
        )

        return logits

    @torch.no_grad()
    def generate_caption(self, image, max_length=50):
        image_features = self.vision_encoder(image)

        # Autoregressive generation
        generated = torch.tensor([[tokenizer.bos_token_id]])

        for _ in range(max_length):
            logits = self.text_decoder(
                generated,
                encoder_hidden_states=image_features
            )

            next_token = logits[:, -1].argmax(dim=-1, keepdim=True)
            generated = torch.cat([generated, next_token], dim=1)

            if next_token.item() == tokenizer.eos_token_id:
                break

        caption = tokenizer.decode(generated[0])
        return caption
```

### Visual Question Answering

```python
def visual_qa(image, question, model):
    """Answer questions about images"""

    # Encode image and question
    image_features = model.encode_image(image)
    question_features = model.encode_text(question)

    # Fuse modalities
    fused = model.fusion(image_features, question_features)

    # Generate answer
    answer = model.answer_decoder.generate(fused)

    return answer

# Example
image = load_image("park.jpg")
question = "How many dogs are in the image?"
answer = visual_qa(image, question, model)
# "There are two dogs in the image."
```

### Speech-to-Speech Translation

```python
class SpeechToSpeechTranslation(nn.Module):
    def __init__(self, speech_encoder, translator, speech_synthesizer):
        super().__init__()

        self.speech_encoder = speech_encoder  # Speech → semantic tokens
        self.translator = translator          # Tokens (L1) → Tokens (L2)
        self.synthesizer = speech_synthesizer # Tokens → Speech

    def forward(self, source_audio, target_language):
        # Encode source speech
        semantic_tokens = self.speech_encoder(source_audio)

        # Translate
        translated_tokens = self.translator(
            semantic_tokens,
            target_language=target_language
        )

        # Synthesize target speech
        target_audio = self.synthesizer(
            translated_tokens,
            speaker_embedding=extract_speaker(source_audio)  # Preserve voice
        )

        return target_audio
```

## 7.6 Multimodal Reasoning

**Chain-of-Thought with Vision**:
```python
prompt = """
Image: [Shows a photo of a partially filled glass on a table]

Question: Will the glass tip over if I tilt the table 15 degrees to the right?

Let's think step by step:
1. Observe the glass position and contents
2. Estimate center of mass
3. Calculate stability given the tilt
4. Determine if center of mass stays over base
"""

response = multimodal_model.generate(prompt)
```

**Visual Programming**:
```python
# Model generates Python code to answer visual questions
question = "What is the average height of the red objects in the image?"

generated_code = model.generate_code(image, question)
"""
# Generated code:
import numpy as np

def solve(image):
    # Detect objects
    objects = detect_objects(image)

    # Filter red objects
    red_objects = [obj for obj in objects if is_red(obj)]

    # Calculate average height
    heights = [obj.height for obj in red_objects]
    avg_height = np.mean(heights)

    return avg_height

answer = solve(image)
"""
```

## 7.7 Challenges and Future Directions

### Current Challenges

**Alignment**: Ensuring tight correspondence between modalities
**Consistency**: Maintaining coherence across long sequences
**Efficiency**: Computational cost of multimodal processing
**Evaluation**: Measuring quality across diverse outputs
**Grounding**: Connecting abstract concepts to perceptual reality

### Emerging Directions

**Unified Architectures**: Single model natively processing all modalities
**Interactive Generation**: Real-time, controllable multimodal creation
**Embodied AI**: Integration with robotics and physical world
**Scientific Discovery**: Multimodal reasoning for research
**Accessibility**: Converting between modalities for different abilities

## Summary

Multimodal generation represents the convergence of previously separate generative AI domains, enabling systems that understand and create across text, images, audio, video, and 3D. These capabilities unlock richer, more natural AI interactions and creative possibilities.

**Key Takeaways**:
- Multimodal understanding combines information from multiple sources
- Unified models process diverse inputs through shared representations
- Cross-modal generation enables translation between modalities
- Applications span video creation, audio synthesis, 3D generation
- Challenges include alignment, consistency, and efficiency
- Future: increasingly seamless multimodal AI systems

---

**弘益人間 (Benefit All Humanity)** - As we build AI that operates across the full spectrum of human communication—words, images, sounds, and beyond—let us ensure these tools enhance understanding between people, make knowledge accessible in every form, and serve all of humanity's diverse needs and abilities.

---

*Previous: [Chapter 6 - Text-to-Image](./06-text-to-image.md) | Next: [Chapter 8 - Ethics and Safety](./08-ethics-and-safety.md)*

---

© 2025 SmileStory Inc. / WIA | WIA-AI-026 Generative AI Standard
