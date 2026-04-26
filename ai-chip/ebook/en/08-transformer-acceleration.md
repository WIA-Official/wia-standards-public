# Chapter 8: Transformer Acceleration - Optimizing LLMs and Attention Mechanisms

## Hardware and Software Techniques for Transformer Models

Transformers have revolutionized AI, powering large language models (LLMs), vision transformers, and multimodal models. However, their quadratic complexity in the attention mechanism poses unique challenges for AI chips. This chapter explores specialized acceleration techniques for transformer models.

---

## The Transformer Challenge

### Computational Bottlenecks

**Standard Transformer Attention**:
```python
def attention(Q, K, V):
    # Q, K, V: [batch, seq_len, dim]
    scores = Q @ K.T  # [batch, seq_len, seq_len] ← Memory bottleneck!
    scores = scores / sqrt(dim)
    attn = softmax(scores)
    output = attn @ V
    return output

Complexity: O(n²d) where n=seq_len, d=dimension
```

**Problem**:
```
GPT-3 (context length 2048):
- Attention matrix: 2048 × 2048 = 4M elements
- Memory: 4M × 4 bytes (FP32) = 16MB per layer
- 96 layers × 16MB = 1.5GB just for attention

LLaMA 2 (context length 4096):
- Attention matrix: 4096 × 4096 = 16.8M elements
- Memory: 16.8M × 2 bytes (FP16) = 33.6MB per layer
- 80 layers × 33.6MB = 2.7GB for attention

GPT-4 (rumored 32k context):
- Attention matrix: 32k × 32k = 1B elements
- Memory: 1B × 2 bytes = 2GB per layer!
- Doesn't fit in GPU cache → Memory-bound
```

**Inference Characteristics**:
- Memory-bound (not compute-bound)
- Quadratic memory growth with sequence length
- Low arithmetic intensity (FLOPs/byte ratio)

---

## Flash Attention: Memory-Efficient Attention

### The Breakthrough

**Standard Attention** (Naive):
```python
# Pseudocode
Q, K, V = input  # Shape: [batch, seq_len, dim]

# Step 1: Compute attention scores
S = Q @ K.T  # Materialize full attention matrix
S = S / sqrt(dim)

# Step 2: Apply softmax
P = softmax(S)  # Still full matrix in memory

# Step 3: Compute output
O = P @ V

Problem: S and P are [seq_len, seq_len] matrices
         Must fit in GPU memory (typically limited)
```

**Flash Attention** (Optimized):
```python
# Tiled computation - never materialize full attention matrix

# Divide Q, K, V into blocks that fit in SRAM
for i in range(0, seq_len, block_size_q):
    Q_block = Q[i:i+block_size_q]  # Load into SRAM

    # Initialize output and normalization factors
    O_i = zeros(block_size_q, dim)
    l_i = zeros(block_size_q)  # Softmax denominators
    m_i = -inf * ones(block_size_q)  # Softmax max values

    for j in range(0, seq_len, block_size_kv):
        K_block = K[j:j+block_size_kv]  # Load into SRAM
        V_block = V[j:j+block_size_kv]

        # Compute attention for this block
        S_ij = Q_block @ K_block.T / sqrt(dim)

        # Online softmax (streaming algorithm)
        m_ij = max(S_ij, axis=1)
        m_new = max(m_i, m_ij)

        P_ij = exp(S_ij - m_new)

        # Update output incrementally
        l_new = exp(m_i - m_new) * l_i + sum(P_ij, axis=1)
        O_i = (O_i * (l_i * exp(m_i - m_new) / l_new)[:, None]
               + P_ij @ V_block)

        # Update normalization factors
        l_i = l_new
        m_i = m_new

    # Write output block
    O[i:i+block_size_q] = O_i

Result: No full attention matrix in memory!
        Only SRAM needed (much faster than HBM)
```

**Performance Impact**:
| Sequence Length | Standard Attention | Flash Attention | Speedup |
|-----------------|-------------------|-----------------|---------|
| 512 | 1.2ms | 0.6ms | 2.0x |
| 1024 | 4.5ms | 1.8ms | 2.5x |
| 2048 | 18ms | 5.4ms | 3.3x |
| 4096 | 72ms | 15ms | 4.8x |
| 8192 | 290ms | 42ms | 6.9x |

**Memory Savings**:
```
Standard: O(n²) memory for attention matrix
Flash Attention: O(n) memory (only Q, K, V, O)

Example (8k context, 80 layers):
Standard: 8192² × 80 × 2 bytes = 10.7GB
Flash Attention: 8192 × 80 × 128 × 2 bytes = 167MB

64x memory reduction!
```

### Flash Attention 2 (2023)

**Further Optimizations**:
1. **Work Partitioning**: Better GPU thread utilization
2. **Reduced Non-Matmul FLOPs**: Fewer rescaling operations
3. **Optimized Softmax**: Faster exp/log approximations

**Performance** (A100 GPU):
```
BERT (512 tokens): 1.8x faster than Flash Attention 1
GPT-2 (1024 tokens): 2.2x faster
LLaMA (4096 tokens): 2.8x faster

Compared to standard attention:
- 5-9x faster
- Same mathematical output (numerically stable)
```

---

## Hardware Support for Transformers

### NVIDIA H100 Transformer Engine

**Automatic FP8 Precision**:
```
Traditional mixed precision (manual):
def forward(x):
    x = layer1(x).half()  # FP16
    x = layer2(x).float()  # FP32 (manual selection)
    x = layer3(x).half()  # FP16
    return x

Transformer Engine (automatic):
def forward(x):
    x = layer1(x)  # Automatic FP8/FP16/FP32 selection
    x = layer2(x)  # Based on runtime analysis
    x = layer3(x)
    return x

No manual tuning required!
```

**How It Works**:
```
Per-tensor scaling:

1. Analyze tensor value distribution
   min_val, max_val = tensor.min(), tensor.max()

2. Select FP8 format:
   - E4M3 (forward pass): Range ±448
   - E5M2 (gradients): Range ±57,344

3. Compute scaling factor:
   scale = FP8_MAX / max(abs(min_val), abs(max_val))

4. Quantize:
   tensor_fp8 = (tensor * scale).to(fp8)

5. Compute in FP8, accumulate in FP16/FP32

6. Dequantize:
   tensor_out = tensor_fp8.to(fp16) / scale
```

**Performance (GPT-3 175B training)**:
| GPU | Time to Train | Speedup vs A100 |
|-----|---------------|-----------------|
| A100 (FP16) | 100 days | 1.0x |
| H100 (FP16) | 60 days | 1.67x |
| H100 (FP8 Transformer Engine) | 34 days | **2.94x** |

### Google TPU v5p: Sparse Core

**Sparsity Acceleration**:
```
Neural networks are naturally sparse:
- 50-90% of weights can be pruned
- 30-70% of activations are zero (ReLU)

Sparse matrix multiplication:
Dense: C = A × B (all elements)
Sparse: C = A × B (skip zeros)

Example:
A = [[1, 0, 3],     B = [[2, 1],
     [0, 0, 4],          [0, 5],
     [2, 5, 0]]          [1, 0]]

Dense: 3×3 × 3×2 = 18 multiplications
Sparse: Only 6 non-zero products

Speedup: 3x
```

**TPU v5p SparseCore**:
- Dedicated hardware for sparse operations
- 2x-4x speedup for 50-80% sparse models
- Supports structured and unstructured sparsity
- Automatic pruning during training

**Use Cases**:
- Sparse transformers (reducing attention overhead)
- Mixture of Experts (MoE) models
- Pruned LLMs

### AMD MI300X: Memory-First Design

**192GB HBM3 Advantage for LLMs**:
```
Inference Latency = Memory Load Time + Compute Time

For LLMs:
- Compute Time << Memory Load Time (memory-bound)
- Larger memory → Fewer GPUs → Less communication

Example: GPT-3 175B inference (batch size 1)
NVIDIA H100 (80GB): Need 3 GPUs
                    Inter-GPU communication overhead
AMD MI300X (192GB): Fit on 1 GPU
                    No communication overhead

Result: MI300X often faster for LLM inference
        despite lower TFLOPS
```

---

## Software Optimizations

### 1. KV Cache Optimization

**Problem**: Redundant computation in autoregressive generation
```
LLM text generation (autoregressive):
Token 1: Compute attention over 1 token
Token 2: Compute attention over 2 tokens (1 is redundant)
Token 3: Compute attention over 3 tokens (1,2 are redundant)
...
Token N: Compute attention over N tokens (1..N-1 are redundant)

Wasted computation: 50% average
```

**Solution**: Cache key and value vectors
```python
class TransformerWithKVCache:
    def __init__(self):
        self.kv_cache = {}  # Store K, V for each layer

    def forward(self, x, position):
        for layer_idx, layer in enumerate(self.layers):
            Q = layer.q_proj(x)

            if position == 0:  # First token
                K = layer.k_proj(x)
                V = layer.v_proj(x)
                self.kv_cache[layer_idx] = (K, V)
            else:  # Subsequent tokens
                K_new = layer.k_proj(x)
                V_new = layer.v_proj(x)

                # Concatenate with cached K, V
                K_cached, V_cached = self.kv_cache[layer_idx]
                K = torch.cat([K_cached, K_new], dim=1)
                V = torch.cat([V_cached, V_new], dim=1)

                self.kv_cache[layer_idx] = (K, V)

            # Compute attention with full K, V
            attn = attention(Q, K, V)
            x = layer.output(attn)

        return x

Speedup: 2-3x for generation
Memory cost: Cache size = 2 × layers × hidden_dim × seq_len
```

**KV Cache Size**:
```
LLaMA 2 70B:
- Layers: 80
- Hidden dim: 8192
- Context: 4096
- Precision: FP16 (2 bytes)

Cache size = 2 (K+V) × 80 × 8192 × 4096 × 2 bytes
           = 10.7 GB per batch item

For batch size 16: 171 GB!
```

**Optimization: Multi-Query Attention (MQA)**
```
Standard Attention:
- num_heads = 32
- Each head has its own K, V projections
- KV cache: 32 × (K + V) per layer

Multi-Query Attention:
- num_heads = 32 (for Q)
- Single shared K, V across all heads
- KV cache: 1 × (K + V) per layer

Reduction: 32x smaller KV cache!

Example: PaLM uses MQA → 32x KV cache reduction
```

### 2. Operator Fusion

**Problem**: Each operation launches separate GPU kernel
```python
# Unfused (slow)
x = layer_norm(x)  # Kernel 1
q = q_proj(x)      # Kernel 2
k = k_proj(x)      # Kernel 3
v = v_proj(x)      # Kernel 4
attn = attention(q, k, v)  # Kernel 5
out = out_proj(attn)  # Kernel 6

Each kernel:
1. Read from HBM
2. Compute
3. Write to HBM

6 kernels = 12 HBM accesses (6 read + 6 write)
```

**Fused Operator** (optimized):
```python
# Fused (fast)
out = fused_attention_block(x)  # Single kernel

Single kernel:
1. Read from HBM (x)
2. Compute all operations in registers/SRAM
3. Write to HBM (out)

1 kernel = 2 HBM accesses (1 read + 1 write)

Speedup: 3-4x by reducing memory traffic
```

**Common Fusions**:
1. **QKV Projection Fusion**: Fuse Q/K/V linear layers
2. **Attention + Output Fusion**: Combine attention and output projection
3. **LayerNorm + Linear Fusion**: Fuse normalization with next layer
4. **GELU/SwiGLU Fusion**: Fuse activation functions

**Example** (PyTorch JIT):
```python
@torch.jit.script
def fused_gelu(x):
    return x * 0.5 * (1.0 + torch.erf(x / 1.41421))

# Compiled to single CUDA kernel
```

### 3. Parallelism Strategies

**Tensor Parallelism**:
```
Split model weights across GPUs

Example: Linear layer (hidden_dim=8192, num_gpus=8)

Standard:
GPU 0: Entire weight matrix [8192, 8192]

Tensor Parallel:
GPU 0: Weight chunk [8192, 1024] (1/8 of columns)
GPU 1: Weight chunk [8192, 1024]
...
GPU 7: Weight chunk [8192, 1024]

Each GPU computes partial result, then all-reduce

Benefits:
- Distribute memory across GPUs
- Parallel computation
- Lower latency than pipeline parallelism
```

**Pipeline Parallelism**:
```
Split model layers across GPUs

Example: 96-layer transformer, 8 GPUs

GPU 0: Layers 0-11
GPU 1: Layers 12-23
GPU 2: Layers 24-35
...
GPU 7: Layers 84-95

Forward pass:
1. GPU 0 processes batch, sends to GPU 1
2. GPU 1 processes, sends to GPU 2
3. ...
4. GPU 7 produces output

Benefits:
- Handle very deep models
- Memory distributed

Drawbacks:
- GPU idle time (bubbles in pipeline)
- Higher latency than tensor parallelism
```

**Data Parallelism**:
```
Replicate model on each GPU, split data

Example: Batch size 256, 8 GPUs

GPU 0: Batch [0:32]
GPU 1: Batch [32:64]
...
GPU 7: Batch [224:256]

Each GPU:
1. Forward pass on its batch
2. Compute gradients
3. All-reduce gradients across GPUs
4. Update weights (synchronized)

Benefits:
- Simple to implement
- Near-linear scaling
- Standard for data center training
```

**3D Parallelism** (Megatron-LM):
```
Combine all three:
- Data parallel: 4 replicas
- Tensor parallel: 8-way (within node)
- Pipeline parallel: 16 stages

Total GPUs: 4 × 8 × 16 = 512 GPUs

Used to train models like:
- GPT-3 (175B)
- Megatron-Turing NLG (530B)
- PaLM (540B)
```

---

## Model Architecture Optimizations

### 1. Grouped-Query Attention (GQA)

**Evolution**:
```
Multi-Head Attention (MHA):
- 32 heads, each with Q, K, V
- KV cache: Large

Multi-Query Attention (MQA):
- 32 Q heads, 1 shared K, V
- KV cache: 32x smaller
- Quality: Slight degradation

Grouped-Query Attention (GQA):
- 32 Q heads, 4-8 KV groups
- KV cache: 4-8x smaller
- Quality: Similar to MHA

Example: LLaMA 2 70B uses GQA (8 groups)
```

**Implementation**:
```python
class GroupedQueryAttention(nn.Module):
    def __init__(self, dim, num_heads=32, num_kv_groups=8):
        self.num_heads = num_heads
        self.num_kv_groups = num_kv_groups
        self.heads_per_group = num_heads // num_kv_groups

        self.q_proj = nn.Linear(dim, dim)
        self.k_proj = nn.Linear(dim, dim // self.heads_per_group)
        self.v_proj = nn.Linear(dim, dim // self.heads_per_group)

    def forward(self, x):
        Q = self.q_proj(x).reshape(..., self.num_heads, ...)
        K = self.k_proj(x).reshape(..., self.num_kv_groups, ...)
        V = self.v_proj(x).reshape(..., self.num_kv_groups, ...)

        # Repeat K, V for each head in group
        K = K.repeat_interleave(self.heads_per_group, dim=1)
        V = V.repeat_interleave(self.heads_per_group, dim=1)

        # Standard attention
        return attention(Q, K, V)
```

### 2. Sliding Window Attention

**Problem**: Quadratic complexity limits long context
```
Full attention: O(n²)
Context 32k: 32768² = 1B operations per layer

Observation: Distant tokens matter less
```

**Solution**: Attend to local window only
```python
def sliding_window_attention(Q, K, V, window_size=256):
    seq_len = Q.shape[1]
    output = []

    for i in range(seq_len):
        # Attend to window around position i
        start = max(0, i - window_size // 2)
        end = min(seq_len, i + window_size // 2)

        q_i = Q[:, i:i+1, :]  # Query at position i
        k_window = K[:, start:end, :]
        v_window = V[:, start:end, :]

        attn_i = attention(q_i, k_window, v_window)
        output.append(attn_i)

    return torch.cat(output, dim=1)

Complexity: O(n × window_size) = O(n) for fixed window
```

**Example**: Longformer (4096 context with 512 window)
```
Full attention: 4096² = 16.7M operations
Sliding window: 4096 × 512 = 2.1M operations

Speedup: 8x with minimal quality loss
```

### 3. Flash Decoding (2023)

**Optimized for LLM Serving**:
```
Problem: KV cache grows during generation
         Memory access becomes bottleneck

Flash Decoding:
1. Partition KV cache across multiple workers
2. Each worker computes partial attention
3. Reduce results efficiently

Speedup: 2-3x for long-context generation (8k+ tokens)
```

---

## Production Deployment Techniques

### 1. Continuous Batching

**Problem**: Fixed batch size wastes resources
```
Standard batching:
Batch 1: [Request A (50 tokens), Request B (20 tokens)]
Wait for longest (50 tokens) before starting Batch 2

Idle time: Request B finishes at 20 tokens,
           waits 30 tokens for Request A
```

**Continuous batching** (vLLM, TensorRT-LLM):
```
As soon as Request B finishes:
- Remove from batch
- Add Request C to batch
- Continue processing

Result: Higher throughput, lower latency
```

### 2. PagedAttention (vLLM)

**KV Cache Management**:
```
Problem: KV cache requires contiguous memory
         Fragmentation wastes memory

Solution: Page KV cache like OS virtual memory

Virtual KV cache: Appears contiguous to model
Physical memory: Paged blocks (e.g., 16 tokens/page)

Benefits:
- Reduce memory waste (20-30% savings)
- Enable more concurrent requests
- Dynamic memory allocation
```

### 3. Speculative Decoding

**Faster Generation**:
```
Idea: Use small "draft" model to propose tokens,
      large "target" model to verify

1. Draft model generates 5 tokens quickly
2. Target model verifies all 5 in parallel
3. Accept correct prefix, reject rest
4. Repeat

Average: 2-3x faster generation
Quality: Identical to target model (no approximation)
```

---

## Conclusion

Transformer acceleration is multi-faceted:

**Hardware Innovations**:
- Flash Attention (memory-efficient)
- Transformer Engine (automatic FP8)
- Sparse cores (skip zeros)
- High-bandwidth memory (MI300X)

**Software Techniques**:
- KV cache (reuse computations)
- Operator fusion (reduce memory traffic)
- 3D parallelism (scale to thousands of GPUs)

**Architecture Changes**:
- GQA (smaller KV cache)
- Sliding window (local attention)
- MoE (sparse models)

**Production Optimizations**:
- Continuous batching (higher throughput)
- PagedAttention (efficient memory)
- Speculative decoding (faster generation)

**For Practitioners**:
- Use Flash Attention 2 (drop-in replacement)
- Enable KV caching for generation
- Quantize to INT8/INT4 (GPTQ, AWQ)
- Leverage tensor parallelism for large models
- Monitor memory bandwidth (often bottleneck)

**Future**: Longer contexts (100k+ tokens), multimodal transformers, on-device LLMs.

Next chapter: Future trends in AI chips.

---

*© 2025 SmileStory Inc. / WIA*
*弘益人間 (Hongik Ingan) · Benefit All Humanity*
