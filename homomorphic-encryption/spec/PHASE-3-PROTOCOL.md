  }

  return output;
}
```

#### 2.4.3 Convolutional Layer

```typescript
async function encryptedConv2D(
  encInput: Ciphertext[][][],    // [height][width][channels]
  encKernel: Ciphertext[][][][], // [kH][kW][inCh][outCh]
  stride: number = 1,
  padding: number = 0
): Promise<Ciphertext[][][]> {

  const [inH, inW, inC] = [encInput.length, encInput[0].length, encInput[0][0].length];
  const [kH, kW, _, outC] = [encKernel.length, encKernel[0].length, inC, encKernel[0][0][0].length];

  const outH = Math.floor((inH + 2 * padding - kH) / stride) + 1;
  const outW = Math.floor((inW + 2 * padding - kW) / stride) + 1;

  const output: Ciphertext[][][] = [];

  for (let h = 0; h < outH; h++) {
    output[h] = [];
    for (let w = 0; w < outW; w++) {
      output[h][w] = [];
      for (let oc = 0; oc < outC; oc++) {
        let sum = await encrypt(0, publicKey);

        for (let kh = 0; kh < kH; kh++) {
          for (let kw = 0; kw < kW; kw++) {
            for (let ic = 0; ic < inC; ic++) {
              const inputH = h * stride + kh - padding;
              const inputW = w * stride + kw - padding;

              if (inputH >= 0 && inputH < inH && inputW >= 0 && inputW < inW) {
                const encMul = await multiply(
                  encInput[inputH][inputW][ic],
                  encKernel[kh][kw][ic][oc]
                );
                sum = await add(sum, encMul);
              }
            }
          }
        }

        output[h][w][oc] = sum;
      }
    }
  }

  return output;
}
```

### 2.5 Model Architectures

#### 2.5.1 CryptoNets

**Architecture:** CNN designed for encrypted inference
- Input: 28×28 encrypted images (MNIST)
- Conv1: 5×5 filters, stride 2
- Square activation (approximation of ReLU)
- Conv2: 5×5 filters, stride 2
- Fully connected layer
- Softmax approximation

**Performance:**
- Accuracy: ~99% (similar to plaintext)
- Inference time: ~250 seconds per image (early implementations)
- Modern optimizations: ~5-10 seconds

#### 2.5.2 LoLa

**Low Latency CNN for Encrypted Inference**
- Optimized layer scheduling
- Reduced multiplicative depth
- Batch normalization folding

**Key Techniques:**
```typescript
// Fold batch normalization into convolution
function foldBatchNorm(
  convWeights: number[][],
  bnGamma: number[],
  bnBeta: number[],
  bnMean: number[],
  bnVar: number[]
): { weights: number[][], bias: number[] } {

  const epsilon = 1e-5;
  const weights = convWeights.map((w, i) =>
    w.map(v => v * bnGamma[i] / Math.sqrt(bnVar[i] + epsilon))
  );

  const bias = bnGamma.map((g, i) =>
    g * (-bnMean[i]) / Math.sqrt(bnVar[i] + epsilon) + bnBeta[i]
  );

  return { weights, bias };
}
```

### 2.6 Federated Learning Integration

**Concept:** Combine encrypted local models from multiple parties

```typescript
async function federatedAggregation(
  localModels: Map<ParticipantId, Ciphertext[]>,
  weights: Map<ParticipantId, number>
): Promise<Ciphertext[]> {

  const modelSize = localModels.values().next().value.length;
  const globalModel: Ciphertext[] = [];

  for (let i = 0; i < modelSize; i++) {
    let encWeightedSum = await encrypt(0, publicKey);

    for (const [party, model] of localModels.entries()) {
      const weight = weights.get(party) || 1.0;
      const encWeighted = await scalarMultiply(model[i], weight);
      encWeightedSum = await add(encWeightedSum, encWeighted);
    }

    globalModel.push(encWeightedSum);
  }

  return globalModel;
}
```

---

# PHASE 4 - CLOUD DEPLOYMENT & INTEGRATION

## 3. CLOUD ARCHITECTURES

### 3.1 Deployment Models

#### 3.1.1 Client-Server Model

**Architecture:**
```
Client (Data Owner)
  ↓ Encrypt data
  ↓ Send Enc(data)
Cloud Server (Compute)
  ↓ Process Enc(data)
  ↓ Return Enc(result)
Client
  ↓ Decrypt Enc(result)
  ↓ Use result
```

**Implementation:**
```typescript
// Client side
class HEClient {
  private secretKey: SecretKey;
  private publicKey: PublicKey;

  async initialize() {
    const he = new WIAHomomorphic({ scheme: 'CKKS' });
    const keys = await he.generateKeys();
    this.secretKey = keys.secretKey;
    this.publicKey = keys.publicKey;
  }

  async encryptAndSend(data: number[]): Promise<string> {
    const encrypted = await encrypt(data, this.publicKey);
    const serialized = serialize(encrypted);

    const response = await fetch('https://api.example.com/compute', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'X-WIA-Standard': 'SEC-012',
        'X-Public-Key': serialize(this.publicKey)
      },
      body: JSON.stringify({ ciphertext: serialized })
    });

    const encryptedResult = deserialize(await response.text());
    return await decrypt(encryptedResult, this.secretKey);
  }
}

// Server side
class HEServer {
  async processRequest(req: Request): Promise<Response> {
    const { ciphertext } = await req.json();
    const publicKey = deserialize(req.headers.get('X-Public-Key'));
    const encrypted = deserialize(ciphertext);

    // Perform computation on encrypted data
    const result = await performComputation(encrypted);

    return new Response(serialize(result), {
      headers: { 'Content-Type': 'application/json' }
    });
  }

  async performComputation(encrypted: Ciphertext): Promise<Ciphertext> {
    // Example: multiply by 2, add 10
    const doubled = await scalarMultiply(encrypted, 2);
    return await scalarAdd(doubled, 10);
  }
}
```

#### 3.1.2 Multi-Party Cloud Model

**Architecture:**
```
Party A ──→ Enc(dataA) ──┐
Party B ──→ Enc(dataB) ──┼──→ Cloud Compute ──→ Enc(result) ──→ All Parties
Party C ──→ Enc(dataC) ──┘
```

### 3.2 Scalability Strategies

#### 3.2.1 Horizontal Scaling

**Load Balancing:**
```typescript
class HELoadBalancer {
  private workers: WorkerNode[];

  async distributeWork(ciphertexts: Ciphertext[]): Promise<Ciphertext[]> {
    const chunks = this.chunkArray(ciphertexts, this.workers.length);

    const promises = chunks.map((chunk, idx) =>
      this.workers[idx].process(chunk)
    );

    const results = await Promise.all(promises);
    return results.flat();
  }

  private chunkArray<T>(array: T[], chunks: number): T[][] {
    const chunkSize = Math.ceil(array.length / chunks);
    return Array.from({ length: chunks }, (_, i) =>
      array.slice(i * chunkSize, (i + 1) * chunkSize)
    );
  }
}
```

#### 3.2.2 Vertical Scaling

**GPU Acceleration:**
```typescript
interface GPUConfig {
  deviceId: number;
  memoryLimit: number;  // GB
  computeCapability: string;
}

class GPUAcceleratedHE {
  async encryptBatch(
    data: number[],
    publicKey: PublicKey,
    gpuConfig: GPUConfig
  ): Promise<Ciphertext[]> {
    // Transfer data to GPU
    // Perform parallel encryption
    // Return results
  }

  async multiplyBatch(
    ct1: Ciphertext[],
    ct2: Ciphertext[],
    gpuConfig: GPUConfig
  ): Promise<Ciphertext[]> {
    // GPU-accelerated NTT operations
    // Parallel polynomial multiplication
  }
}
```

### 3.3 Storage Optimization

#### 3.3.1 Ciphertext Compression

**Techniques:**
- Remove redundant components
- Delta encoding for similar ciphertexts
- Specialized serialization formats

```typescript
interface CompressedCiphertext {
  version: number;
  scheme: 'BFV' | 'CKKS';
  components: Buffer[];  // Compressed polynomial components
  metadata: {
    polyDegree: number;
    modulusChain: number[];
    scale?: number;
  };
}

async function compress(ct: Ciphertext): Promise<CompressedCiphertext> {
  // Apply compression algorithm
  // Reduce storage by 30-50%
}
```

#### 3.3.2 Caching Strategy

```typescript
interface CacheStrategy {
  storage: 'memory' | 'redis' | 'disk';
  ttl: number;  // seconds
  maxSize: number;  // MB
  evictionPolicy: 'LRU' | 'LFU';
}

class HECache {
  async get(key: string): Promise<Ciphertext | null> {
    // Retrieve from cache
  }

  async set(
    key: string,
    value: Ciphertext,
    options?: { ttl?: number }
  ): Promise<void> {
    // Store in cache with eviction policy
  }
}
```

### 3.4 Monitoring and Observability

#### 3.4.1 Metrics


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.
