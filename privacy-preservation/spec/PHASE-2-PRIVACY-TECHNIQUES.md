# WIA-SEC-023: Privacy Preservation — Phase 2 Specification

**Standard ID:** WIA-SEC-023
**Version:** 1.0.0
**Status:** Active
**Category:** Security (SEC)

---
# PHASE 2: Advanced Privacy Techniques

## PHASE 2.1: l-Diversity

### 2.1.1 Overview

**l-Diversity** addresses the homogeneity attack on k-anonymity by ensuring that each equivalence class has at least **l** well-represented values for sensitive attributes.

**Problem with k-Anonymity:**
```
k-Anonymous but vulnerable:
Age    ZIP     Disease
30-35  123**   Flu
30-35  123**   Flu
30-35  123**   Flu
→ All have Flu (homogeneity attack)
```

### 2.1.2 l-Diversity Principles

A dataset satisfies **l-diversity** if:
1. It satisfies k-anonymity
2. Each equivalence class has at least l distinct values for each sensitive attribute
3. The distribution of sensitive values is sufficiently diverse

### 2.1.3 Types of l-Diversity

#### Distinct l-Diversity
Each equivalence class has at least l distinct sensitive values.

```
Example (l=3):
Age    ZIP     Disease
30-35  123**   Flu
30-35  123**   COVID
30-35  123**   Diabetes
→ 3 distinct diseases ✓
```

#### Entropy l-Diversity
Entropy of sensitive attribute distribution ≥ log(l)

```
Entropy = -Σ p(v) × log(p(v))

Example:
Disease distribution: {Flu: 0.4, COVID: 0.3, Diabetes: 0.3}
Entropy = -(0.4×log(0.4) + 0.3×log(0.3) + 0.3×log(0.3)) = 1.57
```

#### Recursive (c, l)-Diversity
Most frequent value appears < c × least frequent value

```
c = 2, l = 3
Flu: 40%
COVID: 30%
Diabetes: 30%
→ 40% < 2 × 30% ✓
```

### 2.1.4 Implementation

```typescript
interface LDiversityConfig {
  k: number;
  l: number;
  sensitiveAttributes: string[];
  diversityType: 'distinct' | 'entropy' | 'recursive';
}

class LDiversityAnonymizer {
  constructor(private config: LDiversityConfig) {}

  anonymize(data: Record<string, any>[]): Record<string, any>[] {
    // 1. Achieve k-anonymity first
    let anonymized = this.kAnonymize(data, this.config.k);

    // 2. Check l-diversity
    const groups = this.groupByQuasiIdentifiers(anonymized);

    for (const group of groups) {
      if (!this.satisfiesLDiversity(group)) {
        // Merge with nearby groups or suppress
        anonymized = this.enforceeLDiversity(anonymized, group);
      }
    }

    return anonymized;
  }

  private satisfiesLDiversity(group: Record<string, any>[]): boolean {
    for (const attr of this.config.sensitiveAttributes) {
      const values = group.map(r => r[attr]);
      const distinctCount = new Set(values).size;

      if (distinctCount < this.config.l) {
        return false;
      }

      if (this.config.diversityType === 'entropy') {
        const entropy = this.calculateEntropy(values);
        if (entropy < Math.log(this.config.l)) {
          return false;
        }
      }
    }

    return true;
  }

  private calculateEntropy(values: any[]): number {
    const freq = new Map<any, number>();
    values.forEach(v => freq.set(v, (freq.get(v) || 0) + 1));

    let entropy = 0;
    const total = values.length;

    for (const count of freq.values()) {
      const p = count / total;
      entropy -= p * Math.log2(p);
    }

    return entropy;
  }
}
```

---

## PHASE 2.2: t-Closeness

### 2.2.1 Overview

**t-Closeness** strengthens l-diversity by requiring that the distribution of sensitive attributes in each equivalence class is close to the distribution in the overall dataset.

**Definition:** An equivalence class satisfies t-closeness if the distance between its sensitive attribute distribution and the overall distribution is ≤ t.

### 2.2.2 Distance Measures

#### Earth Mover's Distance (EMD)

```
EMD(P, Q) = minimum cost to transform distribution P to Q

Example:
Overall: {Flu: 30%, COVID: 40%, Diabetes: 30%}
Group:   {Flu: 20%, COVID: 50%, Diabetes: 30%}

EMD = |30-20| + |40-50| + |30-30| = 10% + 10% + 0% = 20%
```

#### Kullback-Leibler Divergence

```
D_KL(P || Q) = Σ P(x) × log(P(x) / Q(x))
```

### 2.2.3 Implementation

```typescript
interface TClosenessConfig {
  k: number;
  l: number;
  t: number;  // Maximum allowed distance
  sensitiveAttributes: string[];
  distanceMeasure: 'emd' | 'kl-divergence';
}

class TClosenessAnonymizer {
  anonymize(data: Record<string, any>[]): Record<string, any>[] {
    const overallDist = this.calculateDistribution(data);
    let anonymized = this.kAnonymize(data, this.config.k);

    const groups = this.groupByQuasiIdentifiers(anonymized);

    for (const group of groups) {
      const groupDist = this.calculateDistribution(group);
      const distance = this.calculateDistance(groupDist, overallDist);

      if (distance > this.config.t) {
        anonymized = this.enforceTCloseness(anonymized, group);
      }
    }

    return anonymized;
  }

  private calculateDistance(
    dist1: Map<string, number>,
    dist2: Map<string, number>
  ): number {
    if (this.config.distanceMeasure === 'emd') {
      return this.earthMoverDistance(dist1, dist2);
    } else {
      return this.klDivergence(dist1, dist2);
    }
  }

  private earthMoverDistance(
    p: Map<string, number>,
    q: Map<string, number>
  ): number {
    let distance = 0;
    const allKeys = new Set([...p.keys(), ...q.keys()]);

    for (const key of allKeys) {
      distance += Math.abs((p.get(key) || 0) - (q.get(key) || 0));
    }

    return distance / 2;  // Normalize
  }
}
```

---

## PHASE 2.3: Homomorphic Encryption

### 2.3.1 Overview

**Homomorphic Encryption (HE)** allows computation on encrypted data without decryption.

**Types:**
- **Partially Homomorphic:** Supports one operation (addition OR multiplication)
- **Somewhat Homomorphic:** Limited number of operations
- **Fully Homomorphic (FHE):** Unlimited arbitrary operations

### 2.3.2 Properties

```
Encryption: E(x)
Addition: E(x) ⊕ E(y) = E(x + y)
Multiplication: E(x) ⊗ E(y) = E(x × y)
```

### 2.3.3 Use Cases

**Privacy-Preserving Analytics:**
```typescript
// Client encrypts data
const encrypted_salary = encrypt(50000, publicKey);

// Server computes on encrypted data
const encrypted_avg = homomorphic_average([
  encrypted_salary1,
  encrypted_salary2,
  encrypted_salary3
]);

// Client decrypts result
const average = decrypt(encrypted_avg, privateKey);
// Individual salaries never revealed to server
```

**Secure Cloud Computing:**
```
User → [Encrypt Data] → Cloud → [Compute on Encrypted] → [Return Result] → User Decrypts
       Private          Server   No access to plaintext
```

### 2.3.4 Implementation with SEAL

```typescript
import { SEAL } from 'node-seal';

class HomomorphicPrivacy {
  private seal: any;
  private context: any;

  async initialize() {
    this.seal = await SEAL();

    // Configure encryption parameters
    const schemeType = this.seal.SchemeType.bfv;
    const securityLevel = this.seal.SecurityLevel.tc128;
    const polyModulusDegree = 4096;
    const bitSizes = [36, 36, 37];

    const encParms = this.seal.EncryptionParameters(schemeType);
    encParms.setPolyModulusDegree(polyModulusDegree);
    encParms.setCoeffModulus(
      this.seal.CoeffModulus.Create(polyModulusDegree, Int32Array.from(bitSizes))
    );
    encParms.setPlainModulus(this.seal.PlainModulus.Batching(polyModulusDegree, 20));

    this.context = this.seal.Context(encParms, true, securityLevel);
  }

  encryptValue(value: number, publicKey: any): any {
    const encoder = this.seal.BatchEncoder(this.context);
    const encryptor = this.seal.Encryptor(this.context, publicKey);

    const plainText = encoder.encode(Int32Array.from([value]));
    const cipherText = encryptor.encrypt(plainText);

    return cipherText;
  }

  computeSum(encrypted1: any, encrypted2: any): any {
    const evaluator = this.seal.Evaluator(this.context);
    const result = evaluator.add(encrypted1, encrypted2);
    return result;
  }

  computeAverage(encryptedValues: any[]): any {
    const evaluator = this.seal.Evaluator(this.context);

    // Sum all encrypted values
    let sum = encryptedValues[0];
    for (let i = 1; i < encryptedValues.length; i++) {
      sum = evaluator.add(sum, encryptedValues[i]);
    }

    // Divide by count (requires additional encoding)
    // Note: Division is complex in HE, often done client-side
    return sum;
  }
}
```

---

## PHASE 2.4: Secure Multi-Party Computation

### 2.4.1 Overview

**Secure Multi-Party Computation (SMPC)** enables multiple parties to jointly compute a function over their inputs while keeping those inputs private.

**Example:** Two companies want to know if their combined revenue exceeds $1M without revealing individual revenues.

### 2.4.2 Secret Sharing

**Shamir's Secret Sharing:**

```
Secret s split into n shares
Any t shares can reconstruct s
Fewer than t shares reveal nothing
```

**Implementation:**
```typescript
class SecretSharing {
  // Split secret into n shares with threshold t
  split(secret: bigint, n: number, t: number, prime: bigint): bigint[] {
    // Random polynomial of degree t-1
    const coefficients = [secret];
    for (let i = 1; i < t; i++) {
      coefficients.push(this.randomBigInt(prime));
    }

    // Evaluate polynomial at points 1, 2, ..., n
    const shares: bigint[] = [];
    for (let x = 1; x <= n; x++) {
      let y = BigInt(0);
      for (let i = 0; i < t; i++) {
        y += coefficients[i] * BigInt(Math.pow(x, i));
      }
      shares.push(y % prime);
    }

    return shares;
  }

  // Reconstruct secret from t shares
  reconstruct(shares: [number, bigint][], prime: bigint): bigint {
    // Lagrange interpolation
    let secret = BigInt(0);

    for (let i = 0; i < shares.length; i++) {
      const [xi, yi] = shares[i];
      let numerator = BigInt(1);
      let denominator = BigInt(1);

      for (let j = 0; j < shares.length; j++) {
        if (i !== j) {
          const [xj] = shares[j];
          numerator *= BigInt(-xj);
          denominator *= BigInt(xi - xj);
        }
      }

      const lagrange = (numerator * this.modInverse(denominator, prime)) % prime;
      secret = (secret + yi * lagrange) % prime;
    }

    return (secret + prime) % prime;
  }

  private modInverse(a: bigint, m: bigint): bigint {
    // Extended Euclidean algorithm
    let [old_r, r] = [a, m];
    let [old_s, s] = [BigInt(1), BigInt(0)];

    while (r !== BigInt(0)) {
      const quotient = old_r / r;
      [old_r, r] = [r, old_r - quotient * r];
      [old_s, s] = [s, old_s - quotient * s];
    }

    return (old_s + m) % m;
  }
}
```

### 2.4.3 SMPC Protocols

**Millionaire's Problem:**
```typescript
class MillionaireProblem {
  // Alice has a, Bob has b
  // Compute a > b without revealing a or b

  async compare(aliceValue: number, bobValue: number): Promise<boolean> {
    const prime = BigInt(1000000007);

    // Alice and Bob each split their values
    const aliceShares = this.secretSharing.split(BigInt(aliceValue), 2, 2, prime);
    const bobShares = this.secretSharing.split(BigInt(bobValue), 2, 2, prime);

    // Compute comparison in encrypted form
    // (Simplified - actual protocol more complex)
    const difference = (aliceShares[0] + aliceShares[1]) -
                       (bobShares[0] + bobShares[1]);

    return difference > BigInt(0);
  }
}
```

---

