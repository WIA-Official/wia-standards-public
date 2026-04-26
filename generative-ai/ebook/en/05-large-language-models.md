# Chapter 5: Large Language Models for Text Generation

## 5.1 The Transformer Revolution

Large Language Models (LLMs) have transformed how we think about AI and text generation. Built on the Transformer architecture introduced in 2017's "Attention Is All You Need" paper, LLMs can generate coherent, contextual, and creative text across virtually any domain.

The progression has been remarkable:
- **2017**: Original Transformer for machine translation
- **2018**: GPT-1 (117M parameters) and BERT
- **2019**: GPT-2 (1.5B parameters) - "too dangerous to release"
- **2020**: GPT-3 (175B parameters) - few-shot learning breakthrough
- **2022**: ChatGPT - mainstream AI adoption
- **2023**: GPT-4, Claude 3, LLaMA 2, Gemini - multimodal reasoning
- **2024-2025**: Specialized models, efficiency improvements, open-source democratization

### What Makes LLMs Different

**Scale**: Billions of parameters trained on trillions of tokens
**Emergence**: Capabilities that emerge only at scale (reasoning, instruction-following)
**Few-Shot Learning**: Learn new tasks from examples without retraining
**Generality**: Single model handles diverse tasks
**Context**: Process and generate long-form, coherent text

## 5.2 Transformer Architecture

The Transformer's key innovation is the self-attention mechanism, allowing models to weigh the importance of different parts of the input when processing each element.

### Self-Attention Mechanism

```python
class SelfAttention(nn.Module):
    def __init__(self, embed_dim, num_heads):
        super().__init__()
        self.embed_dim = embed_dim
        self.num_heads = num_heads
        self.head_dim = embed_dim // num_heads

        assert self.head_dim * num_heads == embed_dim, "embed_dim must be divisible by num_heads"

        self.query = nn.Linear(embed_dim, embed_dim)
        self.key = nn.Linear(embed_dim, embed_dim)
        self.value = nn.Linear(embed_dim, embed_dim)
        self.out = nn.Linear(embed_dim, embed_dim)

    def forward(self, x, mask=None):
        batch_size, seq_len, embed_dim = x.shape

        # Linear projections
        Q = self.query(x)  # (batch, seq_len, embed_dim)
        K = self.key(x)
        V = self.value(x)

        # Reshape for multi-head attention
        Q = Q.view(batch_size, seq_len, self.num_heads, self.head_dim).transpose(1, 2)
        K = K.view(batch_size, seq_len, self.num_heads, self.head_dim).transpose(1, 2)
        V = V.view(batch_size, seq_len, self.num_heads, self.head_dim).transpose(1, 2)

        # Scaled dot-product attention
        scores = torch.matmul(Q, K.transpose(-2, -1)) / math.sqrt(self.head_dim)

        # Apply mask (for autoregressive generation)
        if mask is not None:
            scores = scores.masked_fill(mask == 0, float('-inf'))

        # Softmax and apply to values
        attn_weights = F.softmax(scores, dim=-1)
        attn_output = torch.matmul(attn_weights, V)

        # Concatenate heads
        attn_output = attn_output.transpose(1, 2).contiguous().view(batch_size, seq_len, embed_dim)

        # Final linear layer
        output = self.out(attn_output)
        return output, attn_weights
```

### Feed-Forward Network

```python
class FeedForward(nn.Module):
    def __init__(self, embed_dim, ff_dim, dropout=0.1):
        super().__init__()
        self.linear1 = nn.Linear(embed_dim, ff_dim)
        self.linear2 = nn.Linear(ff_dim, embed_dim)
        self.dropout = nn.Dropout(dropout)

    def forward(self, x):
        x = F.gelu(self.linear1(x))
        x = self.dropout(x)
        x = self.linear2(x)
        return x
```

### Transformer Block

```python
class TransformerBlock(nn.Module):
    def __init__(self, embed_dim, num_heads, ff_dim, dropout=0.1):
        super().__init__()

        self.attention = SelfAttention(embed_dim, num_heads)
        self.feed_forward = FeedForward(embed_dim, ff_dim, dropout)

        self.norm1 = nn.LayerNorm(embed_dim)
        self.norm2 = nn.LayerNorm(embed_dim)

        self.dropout1 = nn.Dropout(dropout)
        self.dropout2 = nn.Dropout(dropout)

    def forward(self, x, mask=None):
        # Self-attention with residual connection and layer norm
        attn_output, _ = self.attention(self.norm1(x), mask)
        x = x + self.dropout1(attn_output)

        # Feed-forward with residual connection and layer norm
        ff_output = self.feed_forward(self.norm2(x))
        x = x + self.dropout2(ff_output)

        return x
```

### Complete GPT-Style Model

```python
class GPTModel(nn.Module):
    def __init__(self, vocab_size, embed_dim=768, num_heads=12, num_layers=12,
                 ff_dim=3072, max_seq_len=1024, dropout=0.1):
        super().__init__()

        self.embed_dim = embed_dim
        self.max_seq_len = max_seq_len

        # Token and position embeddings
        self.token_embedding = nn.Embedding(vocab_size, embed_dim)
        self.position_embedding = nn.Embedding(max_seq_len, embed_dim)

        # Transformer blocks
        self.blocks = nn.ModuleList([
            TransformerBlock(embed_dim, num_heads, ff_dim, dropout)
            for _ in range(num_layers)
        ])

        # Final layer norm and output projection
        self.norm = nn.LayerNorm(embed_dim)
        self.head = nn.Linear(embed_dim, vocab_size, bias=False)

        # Tie weights between token embedding and output projection
        self.head.weight = self.token_embedding.weight

    def forward(self, input_ids, targets=None):
        batch_size, seq_len = input_ids.shape

        # Create position IDs
        position_ids = torch.arange(seq_len, device=input_ids.device).unsqueeze(0)

        # Embeddings
        token_embeds = self.token_embedding(input_ids)
        position_embeds = self.position_embedding(position_ids)
        x = token_embeds + position_embeds

        # Causal mask (prevent attending to future tokens)
        mask = torch.tril(torch.ones(seq_len, seq_len, device=input_ids.device)).unsqueeze(0).unsqueeze(0)

        # Apply transformer blocks
        for block in self.blocks:
            x = block(x, mask)

        # Final layer norm
        x = self.norm(x)

        # Output logits
        logits = self.head(x)

        # Calculate loss if targets provided
        loss = None
        if targets is not None:
            loss = F.cross_entropy(logits.view(-1, logits.size(-1)), targets.view(-1))

        return logits, loss

    @torch.no_grad()
    def generate(self, input_ids, max_new_tokens, temperature=1.0, top_k=None, top_p=None):
        """Generate text autoregressively"""
        for _ in range(max_new_tokens):
            # Crop to max sequence length
            input_ids_cropped = input_ids[:, -self.max_seq_len:]

            # Forward pass
            logits, _ = self.forward(input_ids_cropped)

            # Get logits for last token
            logits = logits[:, -1, :] / temperature

            # Apply top-k filtering
            if top_k is not None:
                v, _ = torch.topk(logits, min(top_k, logits.size(-1)))
                logits[logits < v[:, [-1]]] = float('-inf')

            # Apply top-p (nucleus) filtering
            if top_p is not None:
                sorted_logits, sorted_indices = torch.sort(logits, descending=True)
                cumulative_probs = torch.cumsum(F.softmax(sorted_logits, dim=-1), dim=-1)

                # Remove tokens with cumulative probability above threshold
                sorted_indices_to_remove = cumulative_probs > top_p
                sorted_indices_to_remove[:, 1:] = sorted_indices_to_remove[:, :-1].clone()
                sorted_indices_to_remove[:, 0] = 0

                indices_to_remove = sorted_indices_to_remove.scatter(1, sorted_indices, sorted_indices_to_remove)
                logits[indices_to_remove] = float('-inf')

            # Sample from distribution
            probs = F.softmax(logits, dim=-1)
            next_token = torch.multinomial(probs, num_samples=1)

            # Append to sequence
            input_ids = torch.cat([input_ids, next_token], dim=1)

        return input_ids
```

## 5.3 Training Large Language Models

Training LLMs involves massive-scale optimization with careful engineering.

### Pretraining

```python
def pretrain_llm(model, train_loader, optimizer, num_epochs, device):
    model.train()

    for epoch in range(num_epochs):
        total_loss = 0
        num_batches = 0

        for batch in train_loader:
            input_ids = batch['input_ids'].to(device)
            targets = batch['targets'].to(device)  # Shifted by 1

            # Forward pass
            logits, loss = model(input_ids, targets)

            # Backward pass
            optimizer.zero_grad()
            loss.backward()

            # Gradient clipping
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)

            # Optimizer step
            optimizer.step()

            total_loss += loss.item()
            num_batches += 1

        avg_loss = total_loss / num_batches
        perplexity = math.exp(avg_loss)

        print(f"Epoch {epoch}: Loss = {avg_loss:.4f}, Perplexity = {perplexity:.2f}")

    return model
```

### Tokenization

```python
from transformers import GPT2Tokenizer

# Byte-Pair Encoding (BPE) tokenizer
tokenizer = GPT2Tokenizer.from_pretrained('gpt2')

text = "Large language models are transforming AI."
tokens = tokenizer.encode(text)
decoded = tokenizer.decode(tokens)

print(f"Original: {text}")
print(f"Tokens: {tokens}")
print(f"Decoded: {decoded}")
```

### Training Optimizations

**Mixed Precision Training**:
```python
from torch.cuda.amp import autocast, GradScaler

scaler = GradScaler()

for batch in train_loader:
    optimizer.zero_grad()

    with autocast():
        logits, loss = model(input_ids, targets)

    scaler.scale(loss).backward()
    scaler.unscale_(optimizer)
    torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
    scaler.step(optimizer)
    scaler.update()
```

**Distributed Training**:
```python
import torch.distributed as dist
from torch.nn.parallel import DistributedDataParallel as DDP

# Initialize process group
dist.init_process_group(backend='nccl')

# Wrap model with DDP
model = DDP(model, device_ids=[local_rank])

# Training proceeds as normal, gradients synchronized automatically
```

**Gradient Accumulation**:
```python
accumulation_steps = 4

for i, batch in enumerate(train_loader):
    logits, loss = model(input_ids, targets)
    loss = loss / accumulation_steps  # Normalize loss

    loss.backward()

    if (i + 1) % accumulation_steps == 0:
        optimizer.step()
        optimizer.zero_grad()
```

## 5.4 Instruction Tuning and RLHF

Making LLMs helpful and aligned requires additional training beyond pretraining.

### Supervised Fine-Tuning (SFT)

Fine-tune on instruction-response pairs:

```python
def instruction_tuning(model, instruction_data, optimizer, num_epochs):
    """
    instruction_data format:
    [
        {"instruction": "Translate to French:", "input": "Hello", "output": "Bonjour"},
        {"instruction": "Summarize:", "input": "Long text...", "output": "Summary..."},
        ...
    ]
    """

    for epoch in range(num_epochs):
        for example in instruction_data:
            # Format as prompt
            prompt = f"{example['instruction']}\n\n{example['input']}\n\nResponse:"
            full_text = prompt + example['output']

            # Tokenize
            input_ids = tokenizer.encode(full_text, return_tensors='pt')
            prompt_len = len(tokenizer.encode(prompt))

            # Forward pass (only compute loss on response)
            logits, _ = model(input_ids)
            loss = F.cross_entropy(
                logits[0, prompt_len-1:-1],
                input_ids[0, prompt_len:]
            )

            # Backward pass
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
```

### Reinforcement Learning from Human Feedback (RLHF)

**Phase 1: Train Reward Model**
```python
class RewardModel(nn.Module):
    def __init__(self, base_model):
        super().__init__()
        self.transformer = base_model.transformer
        self.value_head = nn.Linear(base_model.config.hidden_size, 1)

    def forward(self, input_ids):
        outputs = self.transformer(input_ids)
        last_hidden = outputs.last_hidden_state[:, -1]  # Last token
        reward = self.value_head(last_hidden)
        return reward

def train_reward_model(reward_model, comparison_data, optimizer):
    """
    comparison_data format:
    [
        {"prompt": "...", "chosen": "...", "rejected": "..."},
        ...
    ]
    """

    for example in comparison_data:
        # Encode chosen and rejected completions
        chosen_ids = tokenizer.encode(example['prompt'] + example['chosen'])
        rejected_ids = tokenizer.encode(example['prompt'] + example['rejected'])

        # Get rewards
        reward_chosen = reward_model(chosen_ids)
        reward_rejected = reward_model(rejected_ids)

        # Loss: chosen should have higher reward
        loss = -F.logsigmoid(reward_chosen - reward_rejected).mean()

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
```

**Phase 2: PPO Training**
```python
from transformers import PPOTrainer

def rlhf_training(model, reward_model, prompts):
    """Train with Proximal Policy Optimization"""

    ppo_trainer = PPOTrainer(
        model=model,
        ref_model=reference_model,  # Frozen copy for KL penalty
        reward_model=reward_model,
        ppo_config=ppo_config
    )

    for prompt in prompts:
        # Generate response
        input_ids = tokenizer.encode(prompt, return_tensors='pt')
        response_ids = model.generate(input_ids, max_length=100)

        # Get reward
        reward = reward_model(response_ids)

        # PPO update
        stats = ppo_trainer.step([input_ids], [response_ids], [reward])
```

## 5.5 Prompting Techniques

Effective prompting is crucial for getting desired outputs from LLMs.

### Zero-Shot Prompting

```python
prompt = """
Classify the sentiment of the following review:

Review: "This movie was absolutely fantastic! The acting was superb."

Sentiment:
"""
```

### Few-Shot Prompting

```python
prompt = """
Classify the sentiment as Positive, Negative, or Neutral.

Review: "The food was cold and tasteless."
Sentiment: Negative

Review: "It's okay, nothing special."
Sentiment: Neutral

Review: "Absolutely loved it! Will come back."
Sentiment: Positive

Review: "This product exceeded all my expectations."
Sentiment:
"""
```

### Chain-of-Thought Prompting

```python
prompt = """
Question: A store has 15 apples. They sell 7 and then receive a shipment of 20 more. How many apples do they have now?

Let's solve this step by step:
1. Start with 15 apples
2. Sell 7 apples: 15 - 7 = 8 apples
3. Receive 20 more: 8 + 20 = 28 apples

Answer: 28 apples

Question: A train travels 120 miles in 2 hours. At this rate, how far will it travel in 5 hours?

Let's solve this step by step:
"""
```

### Self-Consistency

Generate multiple reasoning paths and take majority vote:

```python
def self_consistency(model, prompt, num_samples=5):
    answers = []

    for _ in range(num_samples):
        response = model.generate(prompt, temperature=0.7)
        answer = extract_final_answer(response)
        answers.append(answer)

    # Return most common answer
    return Counter(answers).most_common(1)[0][0]
```

### ReAct (Reasoning + Acting)

Interleave reasoning and actions:

```python
prompt = """
Question: What is the capital of the country where the Eiffel Tower is located?

Thought: I need to find out which country the Eiffel Tower is in.
Action: Search "Eiffel Tower location"
Observation: The Eiffel Tower is located in Paris, France.

Thought: Now I know the tower is in France. I need to confirm France's capital.
Action: Search "capital of France"
Observation: Paris is the capital of France.

Thought: I now have the answer.
Answer: Paris
"""
```

## 5.6 Efficient LLMs

Making LLMs more efficient through various techniques.

### Quantization

```python
import torch.quantization as quant

# Post-training quantization
model_fp32 = GPTModel(...)  # Standard FP32 model
model_int8 = quant.quantize_dynamic(
    model_fp32,
    {nn.Linear},  # Quantize linear layers
    dtype=torch.qint8
)

# 4x smaller, 2-4x faster inference
```

### LoRA (Low-Rank Adaptation)

Fine-tune efficiently by adding low-rank matrices:

```python
class LoRALinear(nn.Module):
    def __init__(self, in_features, out_features, rank=4, alpha=16):
        super().__init__()

        # Freeze original weights
        self.linear = nn.Linear(in_features, out_features, bias=False)
        self.linear.weight.requires_grad = False

        # Trainable low-rank matrices
        self.lora_A = nn.Parameter(torch.randn(in_features, rank) * 0.01)
        self.lora_B = nn.Parameter(torch.zeros(rank, out_features))

        self.scaling = alpha / rank

    def forward(self, x):
        # Original + low-rank adaptation
        return self.linear(x) + (x @ self.lora_A @ self.lora_B) * self.scaling
```

Enables fine-tuning with <1% of original parameters.

### Flash Attention

Optimized attention computation:

```python
# Standard attention: O(n²) memory
attention = (Q @ K.T / sqrt(d)) @ V

# Flash Attention: O(n) memory, 2-4x faster
from flash_attn import flash_attn_func
attention = flash_attn_func(Q, K, V)
```

### Speculative Decoding

Speed up generation by using smaller model to propose tokens:

```python
def speculative_decoding(large_model, small_model, input_ids, k=4):
    """Generate k tokens with small model, verify with large model"""

    while len(input_ids) < max_length:
        # Small model proposes k tokens
        proposals = small_model.generate(input_ids, max_new_tokens=k)

        # Large model verifies in parallel
        logits_large = large_model(proposals)

        # Accept/reject based on probability ratios
        for i, token in enumerate(proposals[len(input_ids):]):
            p_large = logits_large[i, token]
            p_small = small_model.get_prob(token, i)

            if random.random() < p_large / p_small:
                input_ids.append(token)
            else:
                break  # Reject and resample

    return input_ids
```

## 5.7 Applications

LLMs power a vast range of applications:

### Content Generation

- Blog posts, articles, stories
- Marketing copy and advertisements
- Email and business writing
- Social media content

### Code Generation

```python
# Example: Using LLM for code generation
prompt = """
Write a Python function that:
1. Takes a list of numbers
2. Removes duplicates
3. Sorts in descending order
4. Returns top 5 elements
"""

# LLM generates:
def top_five_unique(numbers):
    """Get top 5 unique numbers in descending order"""
    unique_sorted = sorted(set(numbers), reverse=True)
    return unique_sorted[:5]
```

### Question Answering

- Customer support chatbots
- Educational tutors
- Information retrieval
- Document QA

### Translation and Localization

- Multi-language translation
- Cultural adaptation
- Technical documentation

### Data Analysis

- SQL query generation
- Data interpretation
- Report generation
- Visualization suggestions

### Reasoning and Problem-Solving

- Mathematical problem solving
- Logical reasoning
- Planning and scheduling
- Creative problem solving

## Summary

Large Language Models represent a breakthrough in generative AI, enabling machines to understand and generate human-like text. Built on Transformer architecture and trained at massive scale, LLMs demonstrate emergent capabilities that extend far beyond simple text completion.

**Key Takeaways**:
- Transformers use self-attention to process sequences in parallel
- Scale (parameters and data) unlocks emergent capabilities
- Instruction tuning and RLHF align models with human intent
- Effective prompting dramatically improves outputs
- Efficiency techniques (quantization, LoRA) enable broader deployment
- Applications span content creation, coding, reasoning, and more

LLMs continue to evolve rapidly, with improvements in reasoning, factuality, efficiency, and multimodal capabilities shaping the future of AI.

## Review Questions

1. **Architecture**
   - Explain how self-attention works in Transformers
   - Why is the causal mask necessary for language generation?
   - What role do position embeddings play?

2. **Training**
   - Describe the pretraining objective for autoregressive LLMs
   - What is the purpose of instruction tuning?
   - How does RLHF improve model outputs?

3. **Prompting**
   - Compare zero-shot and few-shot prompting
   - How does chain-of-thought prompting improve reasoning?
   - What is self-consistency and when is it useful?

4. **Efficiency**
   - How does quantization reduce model size?
   - Explain LoRA and its benefits for fine-tuning
   - What makes Flash Attention more efficient?

5. **Applications**
   - Describe three novel applications of LLMs
   - How can LLMs assist with code generation?
   - What are limitations of current LLMs?

## Practical Exercise

Build and experiment with a small LLM:

1. Implement a GPT-style model (6-12 layers, 256-512 embed dim)
2. Train on a text corpus (books, Wikipedia, code)
3. Implement temperature, top-k, and top-p sampling
4. Experiment with different prompt formats
5. Fine-tune on instruction data
6. Implement LoRA for efficient adaptation
7. Compare generation quality across checkpoints
8. Test chain-of-thought prompting

Document:
- Training curves and perplexity
- Generated samples at different temperatures
- Effect of different prompting strategies
- LoRA vs. full fine-tuning results

---

**弘益人間 (Benefit All Humanity)** - Language is humanity's greatest tool for sharing knowledge, expressing creativity, and building understanding. As we build AI systems that master language, let us ensure they amplify human potential, make knowledge accessible to all, and serve as tools for education, creativity, and connection.

---

*Previous: [Chapter 4 - Diffusion Models](./04-diffusion-models.md) | Next: [Chapter 6 - Text-to-Image Generation](./06-text-to-image.md)*

---

© 2025 SmileStory Inc. / WIA | WIA-AI-026 Generative AI Standard
