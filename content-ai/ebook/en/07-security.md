# Chapter 7: Security Framework

## Content AI Security Architecture

### Overview

The WIA Content AI Security Framework provides comprehensive protection for AI-generated content authentication systems, addressing threats from adversarial attacks, credential forgery, and privacy violations. This chapter details the security architecture required to maintain trust in content provenance systems.

---

## 7.1 Threat Model

### Attack Surface Analysis

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     Content AI Threat Landscape                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                    Content Creation Attacks                          │    │
│  │  • Credential injection   • Watermark removal   • Metadata stripping│    │
│  │  • Signature forgery      • Timestamp tampering • Attribution theft │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                    │                                         │
│                                    ▼                                         │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                    Detection Evasion Attacks                         │    │
│  │  • Adversarial examples   • Detection bypasses  • Model poisoning   │    │
│  │  • Style transfer         • Noise injection     • Compression attack│    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                    │                                         │
│                                    ▼                                         │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                    Infrastructure Attacks                            │    │
│  │  • PKI compromise         • Verification bypass • API abuse         │    │
│  │  • Certificate theft      • Trust anchor attack • DDoS              │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                    │                                         │
│                                    ▼                                         │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                    Privacy Attacks                                   │    │
│  │  • Deanonymization        • Tracking via creds  • Metadata leakage  │    │
│  │  • Creator profiling      • Usage surveillance  • Correlation attack│    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Threat Categories

```typescript
// Threat classification and risk assessment
interface ThreatCategory {
  id: string;
  name: string;
  description: string;
  attackVectors: AttackVector[];
  riskLevel: 'critical' | 'high' | 'medium' | 'low';
  mitigations: Mitigation[];
}

interface AttackVector {
  id: string;
  name: string;
  technique: string;
  difficulty: 'trivial' | 'low' | 'medium' | 'high' | 'expert';
  impact: 'critical' | 'high' | 'medium' | 'low';
  prerequisites: string[];
  indicators: string[];
}

interface Mitigation {
  id: string;
  name: string;
  type: 'preventive' | 'detective' | 'corrective';
  effectiveness: number; // 0-1
  implementation: string;
}

// Comprehensive threat registry
class ThreatRegistry {
  private threats: Map<string, ThreatCategory> = new Map();

  constructor() {
    this.initializeThreats();
  }

  private initializeThreats(): void {
    // Credential forgery threats
    this.registerThreat({
      id: 'THREAT-001',
      name: 'Credential Injection Attack',
      description: 'Attacker injects false provenance credentials into content',
      attackVectors: [
        {
          id: 'AV-001-1',
          name: 'Manifest Injection',
          technique: 'Inject fabricated C2PA manifest into unsigned content',
          difficulty: 'medium',
          impact: 'critical',
          prerequisites: ['Access to content file', 'Knowledge of C2PA format'],
          indicators: ['Invalid signature chain', 'Missing trust anchor']
        },
        {
          id: 'AV-001-2',
          name: 'Signature Replay',
          technique: 'Copy valid signature from one content to another',
          difficulty: 'low',
          impact: 'high',
          prerequisites: ['Access to signed content', 'Similar content type'],
          indicators: ['Hash mismatch', 'Content-signature inconsistency']
        }
      ],
      riskLevel: 'critical',
      mitigations: [
        {
          id: 'MIT-001-1',
          name: 'Cryptographic Binding',
          type: 'preventive',
          effectiveness: 0.95,
          implementation: 'Bind signature to content hash using JUMBF structure'
        },
        {
          id: 'MIT-001-2',
          name: 'Trust Anchor Validation',
          type: 'detective',
          effectiveness: 0.90,
          implementation: 'Verify certificate chain to known trust anchors'
        }
      ]
    });

    // Detection evasion threats
    this.registerThreat({
      id: 'THREAT-002',
      name: 'Adversarial Detection Evasion',
      description: 'AI-generated content modified to evade detection',
      attackVectors: [
        {
          id: 'AV-002-1',
          name: 'Adversarial Perturbation',
          technique: 'Add imperceptible noise to fool detection models',
          difficulty: 'high',
          impact: 'high',
          prerequisites: ['Access to detection model', 'Gradient computation'],
          indicators: ['Unusual pixel patterns', 'Statistical anomalies']
        },
        {
          id: 'AV-002-2',
          name: 'Compression Attack',
          technique: 'Heavy compression/recompression to remove AI artifacts',
          difficulty: 'trivial',
          impact: 'medium',
          prerequisites: ['Basic image editing'],
          indicators: ['Quality degradation', 'Compression artifacts']
        }
      ],
      riskLevel: 'high',
      mitigations: [
        {
          id: 'MIT-002-1',
          name: 'Ensemble Detection',
          type: 'detective',
          effectiveness: 0.85,
          implementation: 'Use multiple diverse detection models'
        },
        {
          id: 'MIT-002-2',
          name: 'Robust Watermarking',
          type: 'preventive',
          effectiveness: 0.80,
          implementation: 'Embed robust invisible watermarks during generation'
        }
      ]
    });
  }

  registerThreat(threat: ThreatCategory): void {
    this.threats.set(threat.id, threat);
  }

  assessRisk(contentType: string, context: SecurityContext): RiskAssessment {
    const applicableThreats = this.getApplicableThreats(contentType, context);

    return {
      overallRisk: this.calculateOverallRisk(applicableThreats),
      threats: applicableThreats.map(t => ({
        threat: t,
        likelihood: this.assessLikelihood(t, context),
        impact: this.assessImpact(t, context),
        residualRisk: this.calculateResidualRisk(t, context)
      })),
      recommendations: this.generateRecommendations(applicableThreats, context)
    };
  }

  private getApplicableThreats(
    contentType: string,
    context: SecurityContext
  ): ThreatCategory[] {
    return Array.from(this.threats.values()).filter(threat =>
      this.isThreatApplicable(threat, contentType, context)
    );
  }

  private isThreatApplicable(
    threat: ThreatCategory,
    contentType: string,
    context: SecurityContext
  ): boolean {
    // Check if threat applies to content type and context
    return true; // Simplified
  }

  private calculateOverallRisk(threats: ThreatCategory[]): number {
    if (threats.length === 0) return 0;

    const riskScores = threats.map(t => {
      switch (t.riskLevel) {
        case 'critical': return 1.0;
        case 'high': return 0.75;
        case 'medium': return 0.5;
        case 'low': return 0.25;
      }
    });

    // Use maximum risk with adjustment for multiple threats
    const maxRisk = Math.max(...riskScores);
    const aggregateFactor = 1 + (threats.length - 1) * 0.1;

    return Math.min(1.0, maxRisk * aggregateFactor);
  }

  private assessLikelihood(
    threat: ThreatCategory,
    context: SecurityContext
  ): number {
    // Assess based on attack difficulty and context
    const baseLikelihood = threat.attackVectors.reduce((max, av) => {
      const difficultyScore = {
        'trivial': 0.9,
        'low': 0.7,
        'medium': 0.5,
        'high': 0.3,
        'expert': 0.1
      }[av.difficulty];
      return Math.max(max, difficultyScore);
    }, 0);

    return baseLikelihood * context.threatEnvironment;
  }

  private assessImpact(
    threat: ThreatCategory,
    context: SecurityContext
  ): number {
    const impactScores = {
      'critical': 1.0,
      'high': 0.75,
      'medium': 0.5,
      'low': 0.25
    };

    return impactScores[threat.riskLevel] * context.assetValue;
  }

  private calculateResidualRisk(
    threat: ThreatCategory,
    context: SecurityContext
  ): number {
    const inherentRisk = this.assessLikelihood(threat, context) *
                         this.assessImpact(threat, context);

    const controlEffectiveness = threat.mitigations.reduce((total, m) =>
      total + m.effectiveness * (context.implementedControls.has(m.id) ? 1 : 0),
      0
    ) / threat.mitigations.length;

    return inherentRisk * (1 - controlEffectiveness);
  }

  private generateRecommendations(
    threats: ThreatCategory[],
    context: SecurityContext
  ): SecurityRecommendation[] {
    const recommendations: SecurityRecommendation[] = [];

    for (const threat of threats) {
      for (const mitigation of threat.mitigations) {
        if (!context.implementedControls.has(mitigation.id)) {
          recommendations.push({
            priority: threat.riskLevel,
            control: mitigation,
            threatAddressed: threat.id,
            estimatedEffort: this.estimateEffort(mitigation),
            businessJustification: `Reduces ${threat.name} risk by ${
              Math.round(mitigation.effectiveness * 100)
            }%`
          });
        }
      }
    }

    return recommendations.sort((a, b) =>
      this.priorityScore(b.priority) - this.priorityScore(a.priority)
    );
  }

  private priorityScore(priority: string): number {
    const scores: Record<string, number> = {
      'critical': 4, 'high': 3, 'medium': 2, 'low': 1
    };
    return scores[priority] || 0;
  }

  private estimateEffort(mitigation: Mitigation): string {
    // Simplified effort estimation
    return mitigation.type === 'preventive' ? 'high' : 'medium';
  }
}

interface SecurityContext {
  threatEnvironment: number; // 0-1
  assetValue: number; // 0-1
  implementedControls: Set<string>;
}

interface RiskAssessment {
  overallRisk: number;
  threats: ThreatAssessment[];
  recommendations: SecurityRecommendation[];
}

interface ThreatAssessment {
  threat: ThreatCategory;
  likelihood: number;
  impact: number;
  residualRisk: number;
}

interface SecurityRecommendation {
  priority: string;
  control: Mitigation;
  threatAddressed: string;
  estimatedEffort: string;
  businessJustification: string;
}
```

---

## 7.2 Cryptographic Security

### Digital Signature Infrastructure

```typescript
import { createSign, createVerify, generateKeyPairSync } from 'crypto';

// Multi-algorithm signing support
class ContentSignatureManager {
  private supportedAlgorithms = ['Ed25519', 'ECDSA-P256', 'RSA-PSS'];

  generateKeyPair(algorithm: string): KeyPair {
    switch (algorithm) {
      case 'Ed25519':
        return generateKeyPairSync('ed25519', {
          publicKeyEncoding: { type: 'spki', format: 'pem' },
          privateKeyEncoding: { type: 'pkcs8', format: 'pem' }
        });

      case 'ECDSA-P256':
        return generateKeyPairSync('ec', {
          namedCurve: 'prime256v1',
          publicKeyEncoding: { type: 'spki', format: 'pem' },
          privateKeyEncoding: { type: 'pkcs8', format: 'pem' }
        });

      case 'RSA-PSS':
        return generateKeyPairSync('rsa-pss', {
          modulusLength: 4096,
          hashAlgorithm: 'sha256',
          saltLength: 32,
          publicKeyEncoding: { type: 'spki', format: 'pem' },
          privateKeyEncoding: { type: 'pkcs8', format: 'pem' }
        });

      default:
        throw new Error(`Unsupported algorithm: ${algorithm}`);
    }
  }

  async signContent(
    content: Buffer,
    privateKey: string,
    algorithm: string
  ): Promise<ContentSignature> {
    const timestamp = Date.now();
    const contentHash = await this.hashContent(content);

    // Create signature payload
    const payload = this.createSignaturePayload(contentHash, timestamp);

    // Sign the payload
    const signature = this.createSignature(payload, privateKey, algorithm);

    return {
      algorithm,
      signature: signature.toString('base64'),
      contentHash,
      timestamp,
      payload
    };
  }

  private async hashContent(content: Buffer): Promise<string> {
    const { subtle } = await import('crypto');
    const hashBuffer = await subtle.digest('SHA-256', content);
    return Buffer.from(hashBuffer).toString('hex');
  }

  private createSignaturePayload(
    contentHash: string,
    timestamp: number
  ): SignaturePayload {
    return {
      version: '1.0',
      contentHash,
      timestamp,
      algorithm: 'SHA-256',
      format: 'C2PA/1.0'
    };
  }

  private createSignature(
    payload: SignaturePayload,
    privateKey: string,
    algorithm: string
  ): Buffer {
    const payloadString = JSON.stringify(payload);

    switch (algorithm) {
      case 'Ed25519':
        const ed25519Sign = createSign('sha512');
        ed25519Sign.update(payloadString);
        return ed25519Sign.sign(privateKey);

      case 'ECDSA-P256':
        const ecdsaSign = createSign('sha256');
        ecdsaSign.update(payloadString);
        return ecdsaSign.sign(privateKey);

      case 'RSA-PSS':
        const rsaSign = createSign('sha256');
        rsaSign.update(payloadString);
        return rsaSign.sign({
          key: privateKey,
          padding: 6, // RSA_PKCS1_PSS_PADDING
          saltLength: 32
        });

      default:
        throw new Error(`Unsupported algorithm: ${algorithm}`);
    }
  }

  verifySignature(
    signature: ContentSignature,
    publicKey: string
  ): SignatureVerificationResult {
    try {
      const payloadString = JSON.stringify(signature.payload);
      const signatureBuffer = Buffer.from(signature.signature, 'base64');

      const verify = createVerify(
        signature.algorithm === 'Ed25519' ? 'sha512' : 'sha256'
      );
      verify.update(payloadString);

      const isValid = verify.verify(publicKey, signatureBuffer);

      return {
        valid: isValid,
        algorithm: signature.algorithm,
        timestamp: new Date(signature.timestamp),
        contentHash: signature.contentHash,
        errors: isValid ? [] : ['Signature verification failed']
      };
    } catch (error) {
      return {
        valid: false,
        algorithm: signature.algorithm,
        timestamp: new Date(signature.timestamp),
        contentHash: signature.contentHash,
        errors: [`Verification error: ${error.message}`]
      };
    }
  }
}

interface KeyPair {
  publicKey: string;
  privateKey: string;
}

interface ContentSignature {
  algorithm: string;
  signature: string;
  contentHash: string;
  timestamp: number;
  payload: SignaturePayload;
}

interface SignaturePayload {
  version: string;
  contentHash: string;
  timestamp: number;
  algorithm: string;
  format: string;
}

interface SignatureVerificationResult {
  valid: boolean;
  algorithm: string;
  timestamp: Date;
  contentHash: string;
  errors: string[];
}
```

### Certificate Chain Validation

```typescript
import * as forge from 'node-forge';

class CertificateChainValidator {
  private trustAnchors: Map<string, forge.pki.Certificate> = new Map();
  private revokedCertificates: Set<string> = new Set();

  constructor(trustAnchors: TrustAnchorConfig[]) {
    for (const anchor of trustAnchors) {
      const cert = forge.pki.certificateFromPem(anchor.certificate);
      this.trustAnchors.set(anchor.id, cert);
    }
  }

  async validateChain(
    certificateChain: string[],
    options: ValidationOptions = {}
  ): Promise<ChainValidationResult> {
    const errors: ValidationError[] = [];
    const warnings: ValidationWarning[] = [];

    // Parse certificates
    const certs = certificateChain.map(pem =>
      forge.pki.certificateFromPem(pem)
    );

    if (certs.length === 0) {
      return {
        valid: false,
        errors: [{ code: 'EMPTY_CHAIN', message: 'Certificate chain is empty' }],
        warnings: [],
        chain: []
      };
    }

    // Validate each certificate in chain
    for (let i = 0; i < certs.length; i++) {
      const cert = certs[i];
      const issuer = i < certs.length - 1 ? certs[i + 1] : null;

      // Check validity period
      const now = new Date();
      if (now < cert.validity.notBefore) {
        errors.push({
          code: 'NOT_YET_VALID',
          message: `Certificate ${i} not yet valid`,
          certificateIndex: i
        });
      }
      if (now > cert.validity.notAfter) {
        errors.push({
          code: 'EXPIRED',
          message: `Certificate ${i} has expired`,
          certificateIndex: i
        });
      }

      // Check signature
      if (issuer) {
        try {
          if (!issuer.verify(cert)) {
            errors.push({
              code: 'INVALID_SIGNATURE',
              message: `Certificate ${i} signature verification failed`,
              certificateIndex: i
            });
          }
        } catch (e) {
          errors.push({
            code: 'SIGNATURE_ERROR',
            message: `Error verifying certificate ${i}: ${e.message}`,
            certificateIndex: i
          });
        }
      }

      // Check revocation status
      const serialNumber = cert.serialNumber;
      if (this.revokedCertificates.has(serialNumber)) {
        errors.push({
          code: 'REVOKED',
          message: `Certificate ${i} has been revoked`,
          certificateIndex: i
        });
      }

      // Check key usage for intermediate/root
      if (i > 0) {
        const keyUsage = cert.getExtension('keyUsage');
        if (keyUsage && !keyUsage.keyCertSign) {
          warnings.push({
            code: 'KEY_USAGE',
            message: `Certificate ${i} missing keyCertSign usage`,
            certificateIndex: i
          });
        }
      }

      // Check basic constraints for CA certificates
      if (i > 0) {
        const basicConstraints = cert.getExtension('basicConstraints');
        if (!basicConstraints || !basicConstraints.cA) {
          errors.push({
            code: 'NOT_CA',
            message: `Certificate ${i} is not a CA certificate`,
            certificateIndex: i
          });
        }
      }
    }

    // Verify chain terminates at trust anchor
    const rootCert = certs[certs.length - 1];
    const trustedAnchor = this.findTrustAnchor(rootCert);

    if (!trustedAnchor) {
      errors.push({
        code: 'NO_TRUST_ANCHOR',
        message: 'Certificate chain does not terminate at a trusted anchor'
      });
    }

    // Check OCSP if enabled
    if (options.checkOCSP) {
      const ocspResults = await this.checkOCSP(certs);
      errors.push(...ocspResults.errors);
      warnings.push(...ocspResults.warnings);
    }

    return {
      valid: errors.length === 0,
      errors,
      warnings,
      chain: certs.map((cert, i) => ({
        subject: cert.subject.getField('CN')?.value || 'Unknown',
        issuer: cert.issuer.getField('CN')?.value || 'Unknown',
        validFrom: cert.validity.notBefore,
        validTo: cert.validity.notAfter,
        serialNumber: cert.serialNumber,
        isTrustAnchor: i === certs.length - 1 && trustedAnchor !== null
      })),
      trustAnchor: trustedAnchor ? {
        id: trustedAnchor.id,
        name: trustedAnchor.name
      } : null
    };
  }

  private findTrustAnchor(
    cert: forge.pki.Certificate
  ): { id: string; name: string } | null {
    for (const [id, anchor] of this.trustAnchors) {
      try {
        // Check if certificate matches or is issued by anchor
        if (anchor.verify(cert) ||
            this.certificatesMatch(anchor, cert)) {
          return {
            id,
            name: anchor.subject.getField('CN')?.value || id
          };
        }
      } catch (e) {
        continue;
      }
    }
    return null;
  }

  private certificatesMatch(
    cert1: forge.pki.Certificate,
    cert2: forge.pki.Certificate
  ): boolean {
    return cert1.serialNumber === cert2.serialNumber &&
           forge.pki.certificateToPem(cert1) === forge.pki.certificateToPem(cert2);
  }

  private async checkOCSP(
    certs: forge.pki.Certificate[]
  ): Promise<{ errors: ValidationError[]; warnings: ValidationWarning[] }> {
    const errors: ValidationError[] = [];
    const warnings: ValidationWarning[] = [];

    for (let i = 0; i < certs.length - 1; i++) {
      const cert = certs[i];
      const issuer = certs[i + 1];

      // Get OCSP responder URL
      const ocspUrl = this.getOCSPUrl(cert);
      if (!ocspUrl) {
        warnings.push({
          code: 'NO_OCSP',
          message: `Certificate ${i} has no OCSP responder URL`,
          certificateIndex: i
        });
        continue;
      }

      try {
        const status = await this.queryOCSP(cert, issuer, ocspUrl);
        if (status === 'revoked') {
          errors.push({
            code: 'OCSP_REVOKED',
            message: `Certificate ${i} is revoked according to OCSP`,
            certificateIndex: i
          });
        } else if (status === 'unknown') {
          warnings.push({
            code: 'OCSP_UNKNOWN',
            message: `OCSP status unknown for certificate ${i}`,
            certificateIndex: i
          });
        }
      } catch (e) {
        warnings.push({
          code: 'OCSP_ERROR',
          message: `OCSP check failed for certificate ${i}: ${e.message}`,
          certificateIndex: i
        });
      }
    }

    return { errors, warnings };
  }

  private getOCSPUrl(cert: forge.pki.Certificate): string | null {
    const aia = cert.getExtension('authorityInfoAccess');
    if (aia && aia.accessDescriptions) {
      for (const desc of aia.accessDescriptions) {
        if (desc.accessMethod === '1.3.6.1.5.5.7.48.1') { // OCSP
          return desc.accessLocation.value;
        }
      }
    }
    return null;
  }

  private async queryOCSP(
    cert: forge.pki.Certificate,
    issuer: forge.pki.Certificate,
    url: string
  ): Promise<'good' | 'revoked' | 'unknown'> {
    // Simplified OCSP query
    // In production, use proper OCSP request/response handling
    return 'good';
  }

  addRevokedCertificate(serialNumber: string): void {
    this.revokedCertificates.add(serialNumber);
  }

  addTrustAnchor(id: string, certificate: string): void {
    const cert = forge.pki.certificateFromPem(certificate);
    this.trustAnchors.set(id, cert);
  }
}

interface TrustAnchorConfig {
  id: string;
  certificate: string;
}

interface ValidationOptions {
  checkOCSP?: boolean;
  allowExpired?: boolean;
  maxChainLength?: number;
}

interface ChainValidationResult {
  valid: boolean;
  errors: ValidationError[];
  warnings: ValidationWarning[];
  chain: CertificateInfo[];
  trustAnchor: { id: string; name: string } | null;
}

interface ValidationError {
  code: string;
  message: string;
  certificateIndex?: number;
}

interface ValidationWarning {
  code: string;
  message: string;
  certificateIndex?: number;
}

interface CertificateInfo {
  subject: string;
  issuer: string;
  validFrom: Date;
  validTo: Date;
  serialNumber: string;
  isTrustAnchor: boolean;
}
```

---

## 7.3 Robust Watermarking

### Neural Watermarking System

```python
import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import Tuple, Optional
import numpy as np

class NeuralWatermarkEncoder(nn.Module):
    """
    Neural network-based invisible watermark encoder.
    Embeds robust watermarks that survive various transformations.
    """

    def __init__(
        self,
        message_length: int = 128,
        image_channels: int = 3,
        hidden_dim: int = 64
    ):
        super().__init__()

        self.message_length = message_length

        # Message preparation network
        self.message_encoder = nn.Sequential(
            nn.Linear(message_length, hidden_dim * 4),
            nn.ReLU(inplace=True),
            nn.Linear(hidden_dim * 4, hidden_dim * 16 * 16),
            nn.ReLU(inplace=True)
        )

        # Image encoder with skip connections
        self.image_encoder = nn.Sequential(
            nn.Conv2d(image_channels, hidden_dim, 3, padding=1),
            nn.BatchNorm2d(hidden_dim),
            nn.ReLU(inplace=True),
            nn.Conv2d(hidden_dim, hidden_dim * 2, 3, stride=2, padding=1),
            nn.BatchNorm2d(hidden_dim * 2),
            nn.ReLU(inplace=True),
            nn.Conv2d(hidden_dim * 2, hidden_dim * 4, 3, stride=2, padding=1),
            nn.BatchNorm2d(hidden_dim * 4),
            nn.ReLU(inplace=True)
        )

        # Fusion network
        self.fusion = nn.Sequential(
            nn.Conv2d(hidden_dim * 4 + hidden_dim, hidden_dim * 4, 3, padding=1),
            nn.BatchNorm2d(hidden_dim * 4),
            nn.ReLU(inplace=True),
            nn.Conv2d(hidden_dim * 4, hidden_dim * 4, 3, padding=1),
            nn.BatchNorm2d(hidden_dim * 4),
            nn.ReLU(inplace=True)
        )

        # Decoder to produce watermarked image
        self.decoder = nn.Sequential(
            nn.ConvTranspose2d(hidden_dim * 4, hidden_dim * 2, 4, stride=2, padding=1),
            nn.BatchNorm2d(hidden_dim * 2),
            nn.ReLU(inplace=True),
            nn.ConvTranspose2d(hidden_dim * 2, hidden_dim, 4, stride=2, padding=1),
            nn.BatchNorm2d(hidden_dim),
            nn.ReLU(inplace=True),
            nn.Conv2d(hidden_dim, image_channels, 3, padding=1),
            nn.Tanh()
        )

        # Strength control
        self.strength = nn.Parameter(torch.tensor(0.1))

    def forward(
        self,
        image: torch.Tensor,
        message: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Embed watermark into image.

        Args:
            image: Input image [B, C, H, W] in range [-1, 1]
            message: Binary message [B, message_length]

        Returns:
            Watermarked image and residual
        """
        batch_size = image.size(0)

        # Encode message
        message_features = self.message_encoder(message.float())
        message_features = message_features.view(batch_size, -1, 16, 16)

        # Encode image
        image_features = self.image_encoder(image)

        # Upsample message features to match image features
        message_features = F.interpolate(
            message_features,
            size=image_features.shape[2:],
            mode='bilinear',
            align_corners=False
        )

        # Fuse image and message
        fused = torch.cat([image_features, message_features], dim=1)
        fused = self.fusion(fused)

        # Decode to residual
        residual = self.decoder(fused)

        # Apply strength-controlled watermark
        watermarked = image + self.strength * residual
        watermarked = torch.clamp(watermarked, -1, 1)

        return watermarked, residual


class NeuralWatermarkDecoder(nn.Module):
    """
    Neural network-based watermark decoder.
    Extracts embedded watermarks with robustness to transformations.
    """

    def __init__(
        self,
        message_length: int = 128,
        image_channels: int = 3,
        hidden_dim: int = 64
    ):
        super().__init__()

        self.message_length = message_length

        # Feature extraction
        self.feature_extractor = nn.Sequential(
            nn.Conv2d(image_channels, hidden_dim, 3, padding=1),
            nn.BatchNorm2d(hidden_dim),
            nn.ReLU(inplace=True),
            nn.Conv2d(hidden_dim, hidden_dim * 2, 3, stride=2, padding=1),
            nn.BatchNorm2d(hidden_dim * 2),
            nn.ReLU(inplace=True),
            nn.Conv2d(hidden_dim * 2, hidden_dim * 4, 3, stride=2, padding=1),
            nn.BatchNorm2d(hidden_dim * 4),
            nn.ReLU(inplace=True),
            nn.Conv2d(hidden_dim * 4, hidden_dim * 8, 3, stride=2, padding=1),
            nn.BatchNorm2d(hidden_dim * 8),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d(1)
        )

        # Message decoder
        self.message_decoder = nn.Sequential(
            nn.Linear(hidden_dim * 8, hidden_dim * 4),
            nn.ReLU(inplace=True),
            nn.Dropout(0.1),
            nn.Linear(hidden_dim * 4, hidden_dim * 2),
            nn.ReLU(inplace=True),
            nn.Linear(hidden_dim * 2, message_length)
        )

    def forward(self, image: torch.Tensor) -> torch.Tensor:
        """
        Extract watermark from image.

        Args:
            image: Potentially watermarked image [B, C, H, W]

        Returns:
            Decoded message logits [B, message_length]
        """
        features = self.feature_extractor(image)
        features = features.view(features.size(0), -1)
        message_logits = self.message_decoder(features)
        return message_logits

    def decode_binary(self, image: torch.Tensor) -> torch.Tensor:
        """Extract binary watermark."""
        logits = self.forward(image)
        return (torch.sigmoid(logits) > 0.5).float()


class RobustWatermarkingSystem:
    """
    Complete watermarking system with training and inference.
    """

    def __init__(
        self,
        message_length: int = 128,
        device: str = 'cuda' if torch.cuda.is_available() else 'cpu'
    ):
        self.device = device
        self.message_length = message_length

        self.encoder = NeuralWatermarkEncoder(message_length).to(device)
        self.decoder = NeuralWatermarkDecoder(message_length).to(device)

        # Differentiable augmentations for training
        self.augmentations = DifferentiableAugmentations()

    def embed_watermark(
        self,
        image: np.ndarray,
        message: bytes
    ) -> np.ndarray:
        """
        Embed watermark into image.

        Args:
            image: RGB image [H, W, 3] in range [0, 255]
            message: Binary message to embed

        Returns:
            Watermarked image [H, W, 3] in range [0, 255]
        """
        self.encoder.eval()

        # Prepare image
        image_tensor = self._preprocess_image(image)

        # Prepare message
        message_bits = self._bytes_to_bits(message)
        message_tensor = torch.tensor(message_bits, dtype=torch.float32)
        message_tensor = message_tensor.unsqueeze(0).to(self.device)

        with torch.no_grad():
            watermarked, _ = self.encoder(image_tensor, message_tensor)

        return self._postprocess_image(watermarked)

    def extract_watermark(
        self,
        image: np.ndarray
    ) -> Tuple[bytes, float]:
        """
        Extract watermark from image.

        Args:
            image: Potentially watermarked image [H, W, 3]

        Returns:
            Extracted message and confidence score
        """
        self.decoder.eval()

        image_tensor = self._preprocess_image(image)

        with torch.no_grad():
            logits = self.decoder(image_tensor)
            probs = torch.sigmoid(logits)
            bits = (probs > 0.5).float()

            # Calculate confidence
            confidence = torch.mean(torch.abs(probs - 0.5) * 2).item()

        message = self._bits_to_bytes(bits.squeeze().cpu().numpy())
        return message, confidence

    def verify_watermark(
        self,
        image: np.ndarray,
        expected_message: bytes,
        threshold: float = 0.9
    ) -> WatermarkVerificationResult:
        """
        Verify if image contains expected watermark.
        """
        extracted, confidence = self.extract_watermark(image)

        # Calculate bit accuracy
        expected_bits = self._bytes_to_bits(expected_message)
        extracted_bits = self._bytes_to_bits(extracted)

        bit_accuracy = np.mean(
            np.array(expected_bits) == np.array(extracted_bits)
        )

        return WatermarkVerificationResult(
            verified=bit_accuracy >= threshold,
            bit_accuracy=float(bit_accuracy),
            confidence=confidence,
            extracted_message=extracted
        )

    def _preprocess_image(self, image: np.ndarray) -> torch.Tensor:
        """Convert numpy image to tensor."""
        # Normalize to [-1, 1]
        tensor = torch.tensor(image, dtype=torch.float32) / 127.5 - 1.0
        tensor = tensor.permute(2, 0, 1).unsqueeze(0)
        return tensor.to(self.device)

    def _postprocess_image(self, tensor: torch.Tensor) -> np.ndarray:
        """Convert tensor to numpy image."""
        tensor = tensor.squeeze(0).permute(1, 2, 0)
        image = ((tensor.cpu().numpy() + 1.0) * 127.5).astype(np.uint8)
        return image

    def _bytes_to_bits(self, data: bytes) -> list:
        """Convert bytes to bit list."""
        bits = []
        for byte in data:
            for i in range(8):
                bits.append((byte >> (7 - i)) & 1)
        # Pad or truncate to message length
        if len(bits) < self.message_length:
            bits.extend([0] * (self.message_length - len(bits)))
        return bits[:self.message_length]

    def _bits_to_bytes(self, bits: np.ndarray) -> bytes:
        """Convert bit array to bytes."""
        bits = bits.astype(int)
        byte_list = []
        for i in range(0, len(bits), 8):
            byte_val = 0
            for j in range(min(8, len(bits) - i)):
                byte_val = (byte_val << 1) | bits[i + j]
            byte_list.append(byte_val)
        return bytes(byte_list)


class DifferentiableAugmentations:
    """
    Differentiable image augmentations for robust watermark training.
    """

    def jpeg_simulation(
        self,
        image: torch.Tensor,
        quality: int = 80
    ) -> torch.Tensor:
        """Differentiable JPEG compression simulation."""
        # Simplified DCT-based compression simulation
        # In production, use proper differentiable JPEG
        noise_level = (100 - quality) / 100.0 * 0.1
        noise = torch.randn_like(image) * noise_level
        return torch.clamp(image + noise, -1, 1)

    def gaussian_blur(
        self,
        image: torch.Tensor,
        kernel_size: int = 5,
        sigma: float = 1.0
    ) -> torch.Tensor:
        """Apply Gaussian blur."""
        channels = image.size(1)

        # Create Gaussian kernel
        x = torch.arange(kernel_size).float() - kernel_size // 2
        kernel_1d = torch.exp(-x ** 2 / (2 * sigma ** 2))
        kernel_1d = kernel_1d / kernel_1d.sum()
        kernel_2d = kernel_1d.unsqueeze(1) @ kernel_1d.unsqueeze(0)
        kernel_2d = kernel_2d.expand(channels, 1, -1, -1)

        kernel_2d = kernel_2d.to(image.device)

        return F.conv2d(
            image,
            kernel_2d,
            padding=kernel_size // 2,
            groups=channels
        )

    def resize(
        self,
        image: torch.Tensor,
        scale: float
    ) -> torch.Tensor:
        """Resize and restore to original size."""
        original_size = image.shape[2:]

        # Downscale
        small = F.interpolate(
            image,
            scale_factor=scale,
            mode='bilinear',
            align_corners=False
        )

        # Upscale back
        restored = F.interpolate(
            small,
            size=original_size,
            mode='bilinear',
            align_corners=False
        )

        return restored

    def random_crop_and_pad(
        self,
        image: torch.Tensor,
        crop_ratio: float = 0.9
    ) -> torch.Tensor:
        """Random crop and pad back to original size."""
        B, C, H, W = image.shape

        new_h = int(H * crop_ratio)
        new_w = int(W * crop_ratio)

        top = torch.randint(0, H - new_h + 1, (1,)).item()
        left = torch.randint(0, W - new_w + 1, (1,)).item()

        cropped = image[:, :, top:top+new_h, left:left+new_w]

        # Pad back to original size
        padded = F.pad(
            cropped,
            (left, W - new_w - left, top, H - new_h - top),
            mode='constant',
            value=0
        )

        return padded


class WatermarkVerificationResult:
    def __init__(
        self,
        verified: bool,
        bit_accuracy: float,
        confidence: float,
        extracted_message: bytes
    ):
        self.verified = verified
        self.bit_accuracy = bit_accuracy
        self.confidence = confidence
        self.extracted_message = extracted_message

    def to_dict(self) -> dict:
        return {
            'verified': self.verified,
            'bit_accuracy': self.bit_accuracy,
            'confidence': self.confidence,
            'extracted_message': self.extracted_message.hex()
        }
```

---

## 7.4 Adversarial Robustness

### Detection Model Hardening

```python
import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import List, Tuple, Optional
import numpy as np

class AdversarialTrainer:
    """
    Adversarial training framework for robust AI detection models.
    """

    def __init__(
        self,
        model: nn.Module,
        attack_types: List[str] = ['pgd', 'fgsm', 'autoattack'],
        epsilon: float = 8/255,
        device: str = 'cuda'
    ):
        self.model = model
        self.attack_types = attack_types
        self.epsilon = epsilon
        self.device = device

        # Attack configurations
        self.attacks = {
            'fgsm': self._fgsm_attack,
            'pgd': self._pgd_attack,
            'autoattack': self._autoattack
        }

    def adversarial_train_step(
        self,
        images: torch.Tensor,
        labels: torch.Tensor,
        optimizer: torch.optim.Optimizer,
        adversarial_ratio: float = 0.5
    ) -> dict:
        """
        Single training step with adversarial examples.
        """
        self.model.train()

        batch_size = images.size(0)
        num_adversarial = int(batch_size * adversarial_ratio)

        # Split batch into clean and adversarial
        clean_images = images[num_adversarial:]
        clean_labels = labels[num_adversarial:]

        adv_images_base = images[:num_adversarial]
        adv_labels = labels[:num_adversarial]

        # Generate adversarial examples
        attack_type = np.random.choice(self.attack_types)
        adv_images = self.attacks[attack_type](
            adv_images_base,
            adv_labels
        )

        # Combine clean and adversarial
        all_images = torch.cat([adv_images, clean_images], dim=0)
        all_labels = torch.cat([adv_labels, clean_labels], dim=0)

        # Forward pass
        optimizer.zero_grad()
        outputs = self.model(all_images)
        loss = F.cross_entropy(outputs, all_labels)

        # Backward pass
        loss.backward()
        optimizer.step()

        # Calculate metrics
        with torch.no_grad():
            predictions = outputs.argmax(dim=1)
            accuracy = (predictions == all_labels).float().mean()

            # Separate accuracies
            adv_accuracy = (predictions[:num_adversarial] == adv_labels).float().mean()
            clean_accuracy = (predictions[num_adversarial:] == clean_labels).float().mean()

        return {
            'loss': loss.item(),
            'accuracy': accuracy.item(),
            'adversarial_accuracy': adv_accuracy.item(),
            'clean_accuracy': clean_accuracy.item(),
            'attack_type': attack_type
        }

    def _fgsm_attack(
        self,
        images: torch.Tensor,
        labels: torch.Tensor
    ) -> torch.Tensor:
        """Fast Gradient Sign Method attack."""
        images = images.clone().requires_grad_(True)

        outputs = self.model(images)
        loss = F.cross_entropy(outputs, labels)
        loss.backward()

        # Generate perturbation
        perturbation = self.epsilon * images.grad.sign()
        adv_images = images + perturbation
        adv_images = torch.clamp(adv_images, 0, 1)

        return adv_images.detach()

    def _pgd_attack(
        self,
        images: torch.Tensor,
        labels: torch.Tensor,
        num_steps: int = 10,
        step_size: float = 2/255
    ) -> torch.Tensor:
        """Projected Gradient Descent attack."""
        adv_images = images.clone()

        for _ in range(num_steps):
            adv_images.requires_grad_(True)

            outputs = self.model(adv_images)
            loss = F.cross_entropy(outputs, labels)
            loss.backward()

            # Update adversarial images
            with torch.no_grad():
                perturbation = step_size * adv_images.grad.sign()
                adv_images = adv_images + perturbation

                # Project back to epsilon ball
                delta = adv_images - images
                delta = torch.clamp(delta, -self.epsilon, self.epsilon)
                adv_images = torch.clamp(images + delta, 0, 1)

        return adv_images.detach()

    def _autoattack(
        self,
        images: torch.Tensor,
        labels: torch.Tensor
    ) -> torch.Tensor:
        """AutoAttack - ensemble of attacks."""
        # Simplified version - in production use official AutoAttack
        # Try multiple attacks and use the most successful

        attacks_to_try = [
            lambda: self._pgd_attack(images, labels, num_steps=20),
            lambda: self._fgsm_attack(images, labels),
            lambda: self._pgd_attack(images, labels, num_steps=40, step_size=1/255)
        ]

        best_adv = None
        best_success_rate = 0

        for attack_fn in attacks_to_try:
            adv_images = attack_fn()

            with torch.no_grad():
                outputs = self.model(adv_images)
                predictions = outputs.argmax(dim=1)
                success_rate = (predictions != labels).float().mean().item()

                if success_rate > best_success_rate:
                    best_success_rate = success_rate
                    best_adv = adv_images

        return best_adv if best_adv is not None else images


class InputPurificationDefense(nn.Module):
    """
    Input purification defense against adversarial examples.
    Uses denoising autoencoder to remove adversarial perturbations.
    """

    def __init__(self, channels: int = 3, hidden_dim: int = 64):
        super().__init__()

        self.encoder = nn.Sequential(
            nn.Conv2d(channels, hidden_dim, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(hidden_dim, hidden_dim * 2, 3, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(hidden_dim * 2, hidden_dim * 4, 3, stride=2, padding=1),
            nn.ReLU(inplace=True)
        )

        self.decoder = nn.Sequential(
            nn.ConvTranspose2d(hidden_dim * 4, hidden_dim * 2, 4, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.ConvTranspose2d(hidden_dim * 2, hidden_dim, 4, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(hidden_dim, channels, 3, padding=1),
            nn.Sigmoid()
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Purify input by denoising."""
        encoded = self.encoder(x)
        decoded = self.decoder(encoded)
        return decoded

    def purify_and_detect(
        self,
        detector: nn.Module,
        images: torch.Tensor,
        num_iterations: int = 3
    ) -> torch.Tensor:
        """
        Iteratively purify and detect.
        """
        purified = images

        for _ in range(num_iterations):
            purified = self.forward(purified)

        return detector(purified)


class EnsembleDefense:
    """
    Ensemble of diverse detection models for robustness.
    """

    def __init__(self, models: List[nn.Module], weights: Optional[List[float]] = None):
        self.models = models
        self.weights = weights or [1.0 / len(models)] * len(models)

    def predict(
        self,
        images: torch.Tensor,
        aggregation: str = 'weighted_average'
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Ensemble prediction with multiple aggregation strategies.
        """
        predictions = []

        for model in self.models:
            model.eval()
            with torch.no_grad():
                pred = model(images)
                predictions.append(F.softmax(pred, dim=1))

        if aggregation == 'weighted_average':
            ensemble_pred = sum(
                w * p for w, p in zip(self.weights, predictions)
            )
        elif aggregation == 'voting':
            votes = torch.stack([p.argmax(dim=1) for p in predictions])
            ensemble_pred = torch.mode(votes, dim=0).values
            return ensemble_pred, torch.ones_like(ensemble_pred).float()
        elif aggregation == 'max_confidence':
            stacked = torch.stack(predictions)
            max_conf, _ = stacked.max(dim=2)
            best_model = max_conf.argmax(dim=0)
            ensemble_pred = torch.stack([
                predictions[best_model[i]][i]
                for i in range(images.size(0))
            ])
        else:
            raise ValueError(f"Unknown aggregation: {aggregation}")

        confidence = ensemble_pred.max(dim=1).values
        return ensemble_pred.argmax(dim=1), confidence

    def detect_adversarial(
        self,
        images: torch.Tensor,
        threshold: float = 0.3
    ) -> torch.Tensor:
        """
        Detect potential adversarial examples via ensemble disagreement.
        """
        predictions = []

        for model in self.models:
            model.eval()
            with torch.no_grad():
                pred = model(images).argmax(dim=1)
                predictions.append(pred)

        # Calculate disagreement
        predictions = torch.stack(predictions)
        mode_pred = torch.mode(predictions, dim=0).values

        agreement = (predictions == mode_pred.unsqueeze(0)).float().mean(dim=0)

        # Low agreement suggests adversarial
        is_adversarial = agreement < (1 - threshold)

        return is_adversarial
```

---

## 7.5 Privacy Protection

### Anonymization Framework

```typescript
// Privacy-preserving content credential system
interface PrivacyConfig {
  anonymizationLevel: 'none' | 'partial' | 'full';
  redactableFields: string[];
  selectiveDisclosure: boolean;
  zkProofSupport: boolean;
}

class PrivacyPreservingCredentials {
  private config: PrivacyConfig;

  constructor(config: PrivacyConfig) {
    this.config = config;
  }

  async createAnonymousCredential(
    content: Buffer,
    creator: CreatorIdentity,
    options: CredentialOptions
  ): Promise<AnonymousCredential> {
    // Generate pseudonymous identifier
    const pseudonym = await this.generatePseudonym(creator, options.context);

    // Create commitment to identity (can be revealed later if needed)
    const identityCommitment = await this.createIdentityCommitment(
      creator,
      options.revealConditions
    );

    // Build credential with selective disclosure support
    const credential = await this.buildCredential({
      pseudonym,
      identityCommitment,
      contentHash: await this.hashContent(content),
      timestamp: Date.now(),
      assertions: options.assertions,
      redactedFields: this.config.redactableFields
    });

    // Generate zero-knowledge proofs if enabled
    let zkProofs: ZKProof[] = [];
    if (this.config.zkProofSupport) {
      zkProofs = await this.generateZKProofs(credential, options.proofRequests);
    }

    return {
      credential,
      zkProofs,
      pseudonym,
      revealToken: identityCommitment.revealToken
    };
  }

  private async generatePseudonym(
    creator: CreatorIdentity,
    context: string
  ): Promise<string> {
    // Context-specific pseudonym prevents cross-platform tracking
    const contextKey = await this.deriveContextKey(creator.privateKey, context);
    const pseudonym = await this.hash(
      `${creator.id}:${context}:${contextKey}`
    );
    return pseudonym;
  }

  private async createIdentityCommitment(
    creator: CreatorIdentity,
    revealConditions: RevealCondition[]
  ): Promise<IdentityCommitment> {
    // Pedersen commitment to identity
    const randomness = crypto.getRandomValues(new Uint8Array(32));
    const commitment = await this.pedersenCommit(
      Buffer.from(creator.id),
      randomness
    );

    // Encrypt reveal token for authorized parties
    const revealToken = await this.encryptRevealToken(
      { identity: creator.id, randomness: Buffer.from(randomness) },
      revealConditions
    );

    return {
      commitment,
      revealToken,
      conditions: revealConditions
    };
  }

  private async pedersenCommit(
    value: Buffer,
    randomness: Uint8Array
  ): Promise<string> {
    // Simplified Pedersen commitment
    // In production, use proper elliptic curve implementation
    const combined = Buffer.concat([value, Buffer.from(randomness)]);
    return await this.hash(combined.toString('hex'));
  }

  private async encryptRevealToken(
    token: { identity: string; randomness: Buffer },
    conditions: RevealCondition[]
  ): Promise<EncryptedRevealToken> {
    // Encrypt token for each authorized revealer
    const encryptedShares: EncryptedShare[] = [];

    for (const condition of conditions) {
      const share = await this.encryptForParty(
        JSON.stringify(token),
        condition.authorizedParty
      );
      encryptedShares.push({
        party: condition.authorizedParty,
        condition: condition.type,
        encryptedData: share
      });
    }

    return { shares: encryptedShares };
  }

  private async encryptForParty(
    data: string,
    partyPublicKey: string
  ): Promise<string> {
    // Use party's public key for encryption
    // Implementation would use proper hybrid encryption
    return Buffer.from(data).toString('base64'); // Simplified
  }

  private async generateZKProofs(
    credential: Credential,
    proofRequests: ProofRequest[]
  ): Promise<ZKProof[]> {
    const proofs: ZKProof[] = [];

    for (const request of proofRequests) {
      switch (request.type) {
        case 'ai_generated':
          proofs.push(await this.proveAIGeneration(credential, request));
          break;
        case 'creator_authorized':
          proofs.push(await this.proveCreatorAuthorization(credential, request));
          break;
        case 'timestamp_range':
          proofs.push(await this.proveTimestampRange(credential, request));
          break;
        case 'content_unmodified':
          proofs.push(await this.proveContentUnmodified(credential, request));
          break;
      }
    }

    return proofs;
  }

  private async proveAIGeneration(
    credential: Credential,
    request: ProofRequest
  ): Promise<ZKProof> {
    // Prove content is AI-generated without revealing generator
    return {
      type: 'ai_generated',
      proof: 'zk_proof_data', // Actual ZK proof would go here
      publicInputs: {
        isAIGenerated: true,
        confidenceThreshold: request.params.minConfidence
      },
      verificationKey: 'vk_ai_generation'
    };
  }

  private async proveCreatorAuthorization(
    credential: Credential,
    request: ProofRequest
  ): Promise<ZKProof> {
    // Prove creator is authorized without revealing identity
    return {
      type: 'creator_authorized',
      proof: 'zk_proof_data',
      publicInputs: {
        isAuthorized: true,
        authorizationLevel: 'verified'
      },
      verificationKey: 'vk_creator_auth'
    };
  }

  private async proveTimestampRange(
    credential: Credential,
    request: ProofRequest
  ): Promise<ZKProof> {
    // Prove timestamp is within range without revealing exact time
    return {
      type: 'timestamp_range',
      proof: 'zk_proof_data',
      publicInputs: {
        afterDate: request.params.minTimestamp,
        beforeDate: request.params.maxTimestamp
      },
      verificationKey: 'vk_timestamp'
    };
  }

  private async proveContentUnmodified(
    credential: Credential,
    request: ProofRequest
  ): Promise<ZKProof> {
    // Prove content hasn't been modified since signing
    return {
      type: 'content_unmodified',
      proof: 'zk_proof_data',
      publicInputs: {
        contentCommitment: credential.contentHash
      },
      verificationKey: 'vk_content_integrity'
    };
  }

  async verifyAnonymousCredential(
    credential: AnonymousCredential,
    content: Buffer
  ): Promise<CredentialVerificationResult> {
    const errors: string[] = [];
    const warnings: string[] = [];

    // Verify content hash
    const contentHash = await this.hashContent(content);
    if (contentHash !== credential.credential.contentHash) {
      errors.push('Content hash mismatch');
    }

    // Verify ZK proofs
    for (const proof of credential.zkProofs) {
      const valid = await this.verifyZKProof(proof);
      if (!valid) {
        errors.push(`Invalid ZK proof: ${proof.type}`);
      }
    }

    // Verify pseudonym format
    if (!this.isValidPseudonym(credential.pseudonym)) {
      warnings.push('Pseudonym format is non-standard');
    }

    return {
      valid: errors.length === 0,
      errors,
      warnings,
      proofResults: credential.zkProofs.map(p => ({
        type: p.type,
        verified: true // Would be actual verification result
      })),
      anonymityLevel: this.assessAnonymityLevel(credential)
    };
  }

  private async verifyZKProof(proof: ZKProof): Promise<boolean> {
    // Verify zero-knowledge proof
    // Implementation would use proper ZK verification
    return true; // Simplified
  }

  private isValidPseudonym(pseudonym: string): boolean {
    return /^[a-f0-9]{64}$/.test(pseudonym);
  }

  private assessAnonymityLevel(credential: AnonymousCredential): string {
    if (credential.zkProofs.length > 0) {
      return 'high';
    }
    if (credential.credential.redactedFields?.length > 0) {
      return 'medium';
    }
    return 'low';
  }

  private async hashContent(content: Buffer): Promise<string> {
    const encoder = new TextEncoder();
    const hashBuffer = await crypto.subtle.digest('SHA-256', content);
    return Buffer.from(hashBuffer).toString('hex');
  }

  private async hash(data: string): Promise<string> {
    const encoder = new TextEncoder();
    const hashBuffer = await crypto.subtle.digest('SHA-256', encoder.encode(data));
    return Buffer.from(hashBuffer).toString('hex');
  }

  private async deriveContextKey(privateKey: string, context: string): Promise<string> {
    return await this.hash(`${privateKey}:${context}`);
  }

  private async buildCredential(params: any): Promise<Credential> {
    return {
      version: '1.0',
      ...params,
      signature: 'signature_placeholder'
    };
  }
}

interface CreatorIdentity {
  id: string;
  privateKey: string;
  publicKey: string;
}

interface CredentialOptions {
  context: string;
  assertions: Assertion[];
  revealConditions: RevealCondition[];
  proofRequests: ProofRequest[];
}

interface RevealCondition {
  type: 'legal_request' | 'platform_policy' | 'creator_consent';
  authorizedParty: string;
  expiresAt?: number;
}

interface ProofRequest {
  type: string;
  params: Record<string, any>;
}

interface AnonymousCredential {
  credential: Credential;
  zkProofs: ZKProof[];
  pseudonym: string;
  revealToken: EncryptedRevealToken;
}

interface ZKProof {
  type: string;
  proof: string;
  publicInputs: Record<string, any>;
  verificationKey: string;
}

interface EncryptedRevealToken {
  shares: EncryptedShare[];
}

interface EncryptedShare {
  party: string;
  condition: string;
  encryptedData: string;
}

interface Credential {
  version: string;
  contentHash: string;
  timestamp: number;
  signature: string;
  redactedFields?: string[];
}

interface CredentialVerificationResult {
  valid: boolean;
  errors: string[];
  warnings: string[];
  proofResults: { type: string; verified: boolean }[];
  anonymityLevel: string;
}

interface Assertion {
  type: string;
  value: any;
}

interface IdentityCommitment {
  commitment: string;
  revealToken: EncryptedRevealToken;
  conditions: RevealCondition[];
}
```

---

## 7.6 Security Monitoring

### Real-time Threat Detection

```typescript
// Security monitoring and incident response
interface SecurityEvent {
  id: string;
  type: SecurityEventType;
  severity: 'critical' | 'high' | 'medium' | 'low' | 'info';
  timestamp: number;
  source: string;
  details: Record<string, any>;
  indicators: string[];
}

type SecurityEventType =
  | 'credential_forgery_attempt'
  | 'detection_evasion'
  | 'certificate_misuse'
  | 'api_abuse'
  | 'trust_anchor_compromise'
  | 'watermark_removal'
  | 'replay_attack'
  | 'anomalous_behavior';

class SecurityMonitor {
  private eventQueue: SecurityEvent[] = [];
  private alertHandlers: Map<string, AlertHandler> = new Map();
  private ruleEngine: SecurityRuleEngine;
  private anomalyDetector: AnomalyDetector;

  constructor(config: SecurityMonitorConfig) {
    this.ruleEngine = new SecurityRuleEngine(config.rules);
    this.anomalyDetector = new AnomalyDetector(config.anomalyConfig);
  }

  async processEvent(event: SecurityEvent): Promise<SecurityResponse> {
    // Enrich event with context
    const enrichedEvent = await this.enrichEvent(event);

    // Evaluate against rules
    const ruleMatches = this.ruleEngine.evaluate(enrichedEvent);

    // Check for anomalies
    const anomalyScore = await this.anomalyDetector.score(enrichedEvent);

    // Determine response
    const response = this.determineResponse(
      enrichedEvent,
      ruleMatches,
      anomalyScore
    );

    // Execute response actions
    await this.executeResponse(response);

    // Store for analysis
    this.eventQueue.push(enrichedEvent);

    return response;
  }

  private async enrichEvent(event: SecurityEvent): Promise<SecurityEvent> {
    return {
      ...event,
      details: {
        ...event.details,
        geoLocation: await this.lookupGeoIP(event.source),
        threatIntelligence: await this.checkThreatIntel(event.indicators),
        relatedEvents: this.findRelatedEvents(event)
      }
    };
  }

  private async lookupGeoIP(ip: string): Promise<GeoLocation | null> {
    // GeoIP lookup implementation
    return null;
  }

  private async checkThreatIntel(
    indicators: string[]
  ): Promise<ThreatIntelResult[]> {
    // Check indicators against threat intelligence feeds
    return [];
  }

  private findRelatedEvents(event: SecurityEvent): SecurityEvent[] {
    // Find related events by source, indicators, or patterns
    return this.eventQueue.filter(e =>
      e.source === event.source ||
      e.indicators.some(i => event.indicators.includes(i))
    ).slice(-10);
  }

  private determineResponse(
    event: SecurityEvent,
    ruleMatches: RuleMatch[],
    anomalyScore: number
  ): SecurityResponse {
    const actions: ResponseAction[] = [];
    let escalate = false;

    // Add actions from rule matches
    for (const match of ruleMatches) {
      actions.push(...match.actions);
      if (match.escalate) escalate = true;
    }

    // Add actions based on anomaly score
    if (anomalyScore > 0.9) {
      actions.push({
        type: 'block',
        target: event.source,
        duration: 3600
      });
      escalate = true;
    } else if (anomalyScore > 0.7) {
      actions.push({
        type: 'rate_limit',
        target: event.source,
        limit: 10
      });
    }

    return {
      eventId: event.id,
      actions,
      escalate,
      severity: this.calculateSeverity(event, ruleMatches, anomalyScore),
      timestamp: Date.now()
    };
  }

  private calculateSeverity(
    event: SecurityEvent,
    ruleMatches: RuleMatch[],
    anomalyScore: number
  ): string {
    if (event.severity === 'critical' || anomalyScore > 0.95) {
      return 'critical';
    }
    if (ruleMatches.some(m => m.severity === 'high') || anomalyScore > 0.8) {
      return 'high';
    }
    if (ruleMatches.length > 0 || anomalyScore > 0.6) {
      return 'medium';
    }
    return 'low';
  }

  private async executeResponse(response: SecurityResponse): Promise<void> {
    for (const action of response.actions) {
      switch (action.type) {
        case 'block':
          await this.blockSource(action.target, action.duration);
          break;
        case 'rate_limit':
          await this.applyRateLimit(action.target, action.limit);
          break;
        case 'alert':
          await this.sendAlert(action.alertConfig);
          break;
        case 'log':
          await this.logIncident(action.logLevel, action.message);
          break;
        case 'quarantine':
          await this.quarantineContent(action.contentId);
          break;
      }
    }

    if (response.escalate) {
      await this.escalateIncident(response);
    }
  }

  private async blockSource(source: string, duration: number): Promise<void> {
    console.log(`Blocking ${source} for ${duration}s`);
  }

  private async applyRateLimit(source: string, limit: number): Promise<void> {
    console.log(`Rate limiting ${source} to ${limit} req/min`);
  }

  private async sendAlert(config: AlertConfig): Promise<void> {
    const handler = this.alertHandlers.get(config.channel);
    if (handler) {
      await handler.send(config);
    }
  }

  private async logIncident(level: string, message: string): Promise<void> {
    console.log(`[${level}] ${message}`);
  }

  private async quarantineContent(contentId: string): Promise<void> {
    console.log(`Quarantining content: ${contentId}`);
  }

  private async escalateIncident(response: SecurityResponse): Promise<void> {
    console.log(`Escalating incident: ${response.eventId}`);
  }

  registerAlertHandler(channel: string, handler: AlertHandler): void {
    this.alertHandlers.set(channel, handler);
  }
}

class SecurityRuleEngine {
  private rules: SecurityRule[];

  constructor(rules: SecurityRule[]) {
    this.rules = rules.sort((a, b) => b.priority - a.priority);
  }

  evaluate(event: SecurityEvent): RuleMatch[] {
    const matches: RuleMatch[] = [];

    for (const rule of this.rules) {
      if (this.matchesRule(event, rule)) {
        matches.push({
          ruleId: rule.id,
          ruleName: rule.name,
          severity: rule.severity,
          actions: rule.actions,
          escalate: rule.escalate
        });

        if (rule.terminal) break;
      }
    }

    return matches;
  }

  private matchesRule(event: SecurityEvent, rule: SecurityRule): boolean {
    // Check event type
    if (rule.eventTypes && !rule.eventTypes.includes(event.type)) {
      return false;
    }

    // Check conditions
    for (const condition of rule.conditions) {
      if (!this.evaluateCondition(event, condition)) {
        return false;
      }
    }

    return true;
  }

  private evaluateCondition(
    event: SecurityEvent,
    condition: RuleCondition
  ): boolean {
    const value = this.getFieldValue(event, condition.field);

    switch (condition.operator) {
      case 'equals':
        return value === condition.value;
      case 'contains':
        return String(value).includes(String(condition.value));
      case 'regex':
        return new RegExp(condition.value).test(String(value));
      case 'greater_than':
        return Number(value) > Number(condition.value);
      case 'less_than':
        return Number(value) < Number(condition.value);
      case 'in':
        return (condition.value as any[]).includes(value);
      default:
        return false;
    }
  }

  private getFieldValue(event: SecurityEvent, field: string): any {
    const parts = field.split('.');
    let value: any = event;

    for (const part of parts) {
      value = value?.[part];
    }

    return value;
  }
}

class AnomalyDetector {
  private baseline: Map<string, BaselineStats> = new Map();

  constructor(config: AnomalyConfig) {
    // Initialize with configuration
  }

  async score(event: SecurityEvent): Promise<number> {
    const features = this.extractFeatures(event);
    const scores: number[] = [];

    for (const [feature, value] of Object.entries(features)) {
      const baseline = this.baseline.get(feature);
      if (baseline) {
        const zscore = Math.abs(value - baseline.mean) / baseline.stddev;
        scores.push(Math.min(zscore / 3, 1)); // Normalize to 0-1
      }
    }

    return scores.length > 0
      ? scores.reduce((a, b) => a + b) / scores.length
      : 0;
  }

  private extractFeatures(event: SecurityEvent): Record<string, number> {
    return {
      eventFrequency: this.getEventFrequency(event.source),
      errorRate: this.getErrorRate(event.source),
      uniqueIndicators: event.indicators.length,
      severityScore: this.severityToNumber(event.severity)
    };
  }

  private getEventFrequency(source: string): number {
    // Get recent event frequency for source
    return 0;
  }

  private getErrorRate(source: string): number {
    // Get error rate for source
    return 0;
  }

  private severityToNumber(severity: string): number {
    const map: Record<string, number> = {
      critical: 5, high: 4, medium: 3, low: 2, info: 1
    };
    return map[severity] || 0;
  }

  updateBaseline(events: SecurityEvent[]): void {
    // Update baseline statistics from historical events
    const features = new Map<string, number[]>();

    for (const event of events) {
      const eventFeatures = this.extractFeatures(event);
      for (const [name, value] of Object.entries(eventFeatures)) {
        if (!features.has(name)) {
          features.set(name, []);
        }
        features.get(name)!.push(value);
      }
    }

    for (const [name, values] of features) {
      const mean = values.reduce((a, b) => a + b) / values.length;
      const variance = values.reduce((sum, v) => sum + (v - mean) ** 2, 0) / values.length;
      const stddev = Math.sqrt(variance);

      this.baseline.set(name, { mean, stddev });
    }
  }
}

interface SecurityMonitorConfig {
  rules: SecurityRule[];
  anomalyConfig: AnomalyConfig;
}

interface SecurityRule {
  id: string;
  name: string;
  priority: number;
  eventTypes?: SecurityEventType[];
  conditions: RuleCondition[];
  actions: ResponseAction[];
  severity: string;
  escalate: boolean;
  terminal: boolean;
}

interface RuleCondition {
  field: string;
  operator: string;
  value: any;
}

interface RuleMatch {
  ruleId: string;
  ruleName: string;
  severity: string;
  actions: ResponseAction[];
  escalate: boolean;
}

interface ResponseAction {
  type: 'block' | 'rate_limit' | 'alert' | 'log' | 'quarantine';
  target?: string;
  duration?: number;
  limit?: number;
  alertConfig?: AlertConfig;
  logLevel?: string;
  message?: string;
  contentId?: string;
}

interface SecurityResponse {
  eventId: string;
  actions: ResponseAction[];
  escalate: boolean;
  severity: string;
  timestamp: number;
}

interface AlertConfig {
  channel: string;
  message: string;
  priority: string;
}

interface AlertHandler {
  send(config: AlertConfig): Promise<void>;
}

interface GeoLocation {
  country: string;
  city: string;
  latitude: number;
  longitude: number;
}

interface ThreatIntelResult {
  indicator: string;
  type: string;
  confidence: number;
  source: string;
}

interface AnomalyConfig {
  windowSize: number;
  thresholds: Record<string, number>;
}

interface BaselineStats {
  mean: number;
  stddev: number;
}
```

---

## 7.7 Security Best Practices

### Implementation Checklist

```yaml
# Security Implementation Checklist
security_checklist:
  cryptography:
    - name: "Use approved algorithms"
      requirement: "Ed25519, ECDSA-P256, or RSA-4096 for signatures"
      verification: "Review signing configuration"
      priority: critical

    - name: "Secure key storage"
      requirement: "HSM or secure enclave for private keys"
      verification: "Audit key management infrastructure"
      priority: critical

    - name: "Certificate pinning"
      requirement: "Pin trust anchor certificates"
      verification: "Test certificate validation"
      priority: high

    - name: "Cryptographic agility"
      requirement: "Support algorithm migration"
      verification: "Test with multiple algorithms"
      priority: medium

  authentication:
    - name: "Multi-factor authentication"
      requirement: "MFA for signing key access"
      verification: "Review authentication flow"
      priority: critical

    - name: "Session management"
      requirement: "Short-lived tokens with rotation"
      verification: "Audit token lifecycle"
      priority: high

    - name: "API authentication"
      requirement: "OAuth 2.0 or API keys with rotation"
      verification: "Test API authentication"
      priority: high

  detection_security:
    - name: "Adversarial robustness"
      requirement: "Trained against adversarial examples"
      verification: "Run adversarial evaluation suite"
      priority: high

    - name: "Model versioning"
      requirement: "Immutable model deployment"
      verification: "Verify deployment pipeline"
      priority: medium

    - name: "Ensemble detection"
      requirement: "Multiple diverse detection models"
      verification: "Test ensemble disagreement"
      priority: medium

  privacy:
    - name: "Data minimization"
      requirement: "Collect only necessary data"
      verification: "Review data collection"
      priority: high

    - name: "Anonymization"
      requirement: "Support pseudonymous credentials"
      verification: "Test anonymization features"
      priority: medium

    - name: "Consent management"
      requirement: "Explicit consent for tracking"
      verification: "Audit consent flows"
      priority: high

  operations:
    - name: "Security monitoring"
      requirement: "Real-time threat detection"
      verification: "Test alerting systems"
      priority: high

    - name: "Incident response"
      requirement: "Documented procedures"
      verification: "Run tabletop exercises"
      priority: high

    - name: "Security logging"
      requirement: "Comprehensive audit trail"
      verification: "Review log completeness"
      priority: high
```

---

## Summary

The WIA Content AI Security Framework provides:

1. **Comprehensive Threat Model** - Systematic analysis of attack vectors and mitigations
2. **Cryptographic Security** - Multi-algorithm signing and certificate chain validation
3. **Robust Watermarking** - Neural watermarks surviving transformations
4. **Adversarial Robustness** - Hardened detection models and ensemble defense
5. **Privacy Protection** - Anonymization and zero-knowledge proofs
6. **Security Monitoring** - Real-time threat detection and response

---

*© 2025 World Industry Association (WIA). All rights reserved.*
*弘益人間 (홍익인간) · Benefit All Humanity*
