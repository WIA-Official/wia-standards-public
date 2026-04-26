
```typescript
interface HEMetrics {
  encryption: {
    throughput: number;      // ops/sec
    latency: {
      p50: number;
      p95: number;
      p99: number;
    };
    errors: number;
  };
  computation: {
    operations: Map<string, number>;  // operation -> count
    noiseBudget: {
      average: number;
      minimum: number;
    };
    bootstraps: number;
  };
  resources: {
    cpuUsage: number;        // percent
    memoryUsage: number;     // MB
    gpuUsage?: number;       // percent
  };
}
```

#### 3.4.2 Logging

```typescript
interface HELogEntry {
  timestamp: string;
  level: 'info' | 'warn' | 'error';
  operation: string;
  duration: number;
  metadata: {
    ciphertextId?: string;
    noiseBudget?: number;
    participantId?: string;
  };
}

class HELogger {
  logOperation(entry: HELogEntry): void {
    // Send to logging service
    // Alert on anomalies
  }
}
```

## 4. INTEGRATION PATTERNS

### 4.1 REST API

```typescript
// Express.js example
import express from 'express';
import { WIAHomomorphic } from '@wia/sec-012';

const app = express();
const he = new WIAHomomorphic({ scheme: 'BFV' });

app.post('/api/v1/encrypt', async (req, res) => {
  try {
    const { data, publicKey } = req.body;
    const encrypted = await he.encrypt(data, deserialize(publicKey));

    res.json({
      success: true,
      ciphertext: serialize(encrypted)
    });
  } catch (error) {
    res.status(400).json({ success: false, error: error.message });
  }
});

app.post('/api/v1/compute', async (req, res) => {
  try {
    const { operation, operands } = req.body;
    const cts = operands.map(deserialize);

    let result;
    switch (operation) {
      case 'add':
        result = await he.add(cts[0], cts[1]);
        break;
      case 'multiply':
        result = await he.multiply(cts[0], cts[1]);
        break;
      default:
        throw new Error('Unsupported operation');
    }

    res.json({
      success: true,
      result: serialize(result)
    });
  } catch (error) {
    res.status(400).json({ success: false, error: error.message });
  }
});
```

### 4.2 GraphQL API

```typescript
import { GraphQLObjectType, GraphQLSchema, GraphQLString } from 'graphql';

const CiphertextType = new GraphQLObjectType({
  name: 'Ciphertext',
  fields: {
    id: { type: GraphQLString },
    data: { type: GraphQLString },
    scheme: { type: GraphQLString },
    noiseBudget: { type: GraphQLString }
  }
});

const schema = new GraphQLSchema({
  query: new GraphQLObjectType({
    name: 'Query',
    fields: {
      getNoiseBudget: {
        type: GraphQLString,
        args: { ciphertextId: { type: GraphQLString } },
        resolve: async (_, { ciphertextId }) => {
          const ct = await getCiphertext(ciphertextId);
          return he.getNoiseBudget(ct).toString();
        }
      }
    }
  }),
  mutation: new GraphQLObjectType({
    name: 'Mutation',
    fields: {
      add: {
        type: CiphertextType,
        args: {
          ct1Id: { type: GraphQLString },
          ct2Id: { type: GraphQLString }
        },
        resolve: async (_, { ct1Id, ct2Id }) => {
          const ct1 = await getCiphertext(ct1Id);
          const ct2 = await getCiphertext(ct2Id);
          const result = await he.add(ct1, ct2);
          return await storeCiphertext(result);
        }
      }
    }
  })
});
```

### 4.3 WebSocket Streaming

```typescript
import WebSocket from 'ws';

const wss = new WebSocket.Server({ port: 8080 });

wss.on('connection', (ws) => {
  ws.on('message', async (message) => {
    const { type, payload } = JSON.parse(message);

    if (type === 'COMPUTE_STREAM') {
      const { ciphertexts, operation } = payload;

      for (const ct of ciphertexts) {
        const result = await performOperation(ct, operation);

        ws.send(JSON.stringify({
          type: 'RESULT',
          payload: { result: serialize(result) }
        }));
      }

      ws.send(JSON.stringify({ type: 'COMPLETE' }));
    }
  });
});
```

### 4.4 Message Queue Integration

```typescript
import { Kafka } from 'kafkajs';

const kafka = new Kafka({ brokers: ['localhost:9092'] });
const consumer = kafka.consumer({ groupId: 'he-workers' });

await consumer.connect();
await consumer.subscribe({ topic: 'encrypted-data' });

await consumer.run({
  eachMessage: async ({ message }) => {
    const { ciphertext, operation } = JSON.parse(message.value.toString());

    const result = await performComputation(
      deserialize(ciphertext),
      operation
    );

    await producer.send({
      topic: 'encrypted-results',
      messages: [{ value: serialize(result) }]
    });
  }
});
```

## 5. BEST PRACTICES

### 5.1 Security Best Practices

1. **Always validate inputs** before encryption
2. **Use authenticated encryption** when possible
3. **Rotate keys regularly** (90-180 days)
4. **Monitor noise budgets** to prevent decryption failures
5. **Implement rate limiting** on API endpoints
6. **Audit all key access** and decryption operations
7. **Use HSMs** for production key storage

### 5.2 Performance Best Practices

1. **Batch operations** whenever possible
2. **Reuse public keys** for multiple encryptions
3. **Cache Galois/relin keys** to avoid regeneration
4. **Pre-compute NTT tables** for faster operations
5. **Use GPU acceleration** for large-scale deployments
6. **Optimize multiplicative depth** in circuit design
7. **Profile regularly** to identify bottlenecks

### 5.3 Development Best Practices

1. **Test with plaintext first** before encrypting
2. **Validate correctness** on small datasets
3. **Benchmark performance** early in development
4. **Document parameter choices** and security levels
5. **Version control schemas** and key formats
6. **Implement comprehensive logging**
7. **Create reproducible test cases**

---

## 6. CASE STUDIES

### 6.1 Healthcare: Private Medical Data Analysis

**Problem:** Hospital consortium wants to analyze patient data across institutions without sharing raw data.

**Solution:**
```typescript
class MedicalDataAnalyzer {
  async analyzeCrossSite(
    hospitals: Hospital[],
    analysisType: 'prevalence' | 'correlation'
  ): Promise<AnalysisResult> {

    const he = new WIAHomomorphic({ scheme: 'BFV' });
    const { publicKey, secretKey } = await he.generateKeys();

    // Each hospital encrypts their data
    const encryptedDatasets = await Promise.all(
      hospitals.map(h => h.encryptPatientData(publicKey))
    );

    // Aggregate encrypted statistics
    const encryptedStats = await this.computeAggregateStats(
      encryptedDatasets,
      analysisType
    );

    // Decrypt only final results
    return await he.decrypt(encryptedStats, secretKey);
  }
}
```

**Results:**
- Privacy preserved for all patients
- No hospital sees other hospitals' data
- Regulatory compliance (HIPAA) maintained
- Insights from combined dataset

### 6.2 Finance: Confidential Portfolio Analysis

**Problem:** Multiple investors want to calculate total portfolio value without revealing individual holdings.

**Solution:** Secure MPC using Paillier encryption for additive homomorphism.

**Performance:**
- 1000 investors: ~2 seconds
- Privacy: Perfect
- Accuracy: Exact (no approximation)

### 6.3 Cloud ML: Encrypted Model Serving

**Problem:** Provide ML predictions without exposing user data to cloud provider.

**Solution:** CryptoNets architecture with CKKS encryption.

**Metrics:**
- Accuracy: 98.5% (vs 99.1% plaintext)
- Latency: 5 seconds per image
- Privacy: Complete

---

## 7. FUTURE DIRECTIONS

### 7.1 Research Areas

- **Faster bootstrapping** algorithms
- **Hardware acceleration** (FPGA, ASIC)
- **Improved polynomial approximations** for activations
- **Hybrid protocols** combining HE with other techniques
- **Standardized benchmarks** and testing frameworks

### 7.2 Emerging Applications

- **Private blockchain** smart contracts
- **Encrypted search** engines
- **Genomic privacy** for DNA analysis
- **IoT security** for edge devices
- **Quantum-safe** communication protocols

---

**Document Control**

| Version | Date       | Author          | Changes                        |
|---------|------------|-----------------|--------------------------------|
| 1.0.0   | 2025-12-25 | WIA Standards   | Initial PHASE 2-3-4 release    |

---

弘益人間 (홍익인간) · Benefit All Humanity

© 2025 SmileStory Inc. / WIA


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

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
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.
