//! Post-Quantum Cryptography types

use crate::error::{QuantumError, Result};
use crate::types::WIA_QUANTUM_VERSION;
use serde::{Deserialize, Serialize};
use uuid::Uuid;
use chrono::{DateTime, Utc};

/// PQC algorithm type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PqcKeyType {
    /// Key Encapsulation Mechanism
    #[serde(rename = "kem")]
    Kem,
    /// Digital Signature
    #[serde(rename = "signature")]
    Signature,
}

/// PQC algorithm variants (NIST standards)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PqcAlgorithm {
    // ML-KEM (Kyber) - Key Encapsulation
    #[serde(rename = "ML-KEM-512")]
    MlKem512,
    #[serde(rename = "ML-KEM-768")]
    MlKem768,
    #[serde(rename = "ML-KEM-1024")]
    MlKem1024,

    // ML-DSA (Dilithium) - Digital Signatures
    #[serde(rename = "ML-DSA-44")]
    MlDsa44,
    #[serde(rename = "ML-DSA-65")]
    MlDsa65,
    #[serde(rename = "ML-DSA-87")]
    MlDsa87,

    // SLH-DSA (SPHINCS+) - Hash-based Signatures
    #[serde(rename = "SLH-DSA-SHAKE-128f")]
    SlhDsaShake128f,
    #[serde(rename = "SLH-DSA-SHAKE-128s")]
    SlhDsaShake128s,
    #[serde(rename = "SLH-DSA-SHAKE-192f")]
    SlhDsaShake192f,
    #[serde(rename = "SLH-DSA-SHAKE-256f")]
    SlhDsaShake256f,

    // HQC - Code-based KEM (backup)
    #[serde(rename = "HQC-128")]
    Hqc128,
    #[serde(rename = "HQC-192")]
    Hqc192,
    #[serde(rename = "HQC-256")]
    Hqc256,
}

impl PqcAlgorithm {
    /// Get NIST security level
    pub fn nist_level(&self) -> u8 {
        match self {
            PqcAlgorithm::MlKem512 => 1,
            PqcAlgorithm::MlKem768 => 3,
            PqcAlgorithm::MlKem1024 => 5,
            PqcAlgorithm::MlDsa44 => 2,
            PqcAlgorithm::MlDsa65 => 3,
            PqcAlgorithm::MlDsa87 => 5,
            PqcAlgorithm::SlhDsaShake128f | PqcAlgorithm::SlhDsaShake128s => 1,
            PqcAlgorithm::SlhDsaShake192f => 3,
            PqcAlgorithm::SlhDsaShake256f => 5,
            PqcAlgorithm::Hqc128 => 1,
            PqcAlgorithm::Hqc192 => 3,
            PqcAlgorithm::Hqc256 => 5,
        }
    }

    /// Get key type
    pub fn key_type(&self) -> PqcKeyType {
        match self {
            PqcAlgorithm::MlKem512
            | PqcAlgorithm::MlKem768
            | PqcAlgorithm::MlKem1024
            | PqcAlgorithm::Hqc128
            | PqcAlgorithm::Hqc192
            | PqcAlgorithm::Hqc256 => PqcKeyType::Kem,
            _ => PqcKeyType::Signature,
        }
    }

    /// Get approximate public key size in bytes
    pub fn public_key_size(&self) -> usize {
        match self {
            PqcAlgorithm::MlKem512 => 800,
            PqcAlgorithm::MlKem768 => 1184,
            PqcAlgorithm::MlKem1024 => 1568,
            PqcAlgorithm::MlDsa44 => 1312,
            PqcAlgorithm::MlDsa65 => 1952,
            PqcAlgorithm::MlDsa87 => 2592,
            PqcAlgorithm::SlhDsaShake128f | PqcAlgorithm::SlhDsaShake128s => 32,
            PqcAlgorithm::SlhDsaShake192f => 48,
            PqcAlgorithm::SlhDsaShake256f => 64,
            PqcAlgorithm::Hqc128 => 2249,
            PqcAlgorithm::Hqc192 => 4522,
            PqcAlgorithm::Hqc256 => 7245,
        }
    }
}

/// Algorithm info for serialization
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlgorithmInfo {
    pub name: String,
    pub variant: String,
    pub nist_level: u8,
    #[serde(rename = "type")]
    pub key_type: PqcKeyType,
}

impl From<PqcAlgorithm> for AlgorithmInfo {
    fn from(alg: PqcAlgorithm) -> Self {
        let (name, variant) = match alg {
            PqcAlgorithm::MlKem512 => ("ML-KEM", "ML-KEM-512"),
            PqcAlgorithm::MlKem768 => ("ML-KEM", "ML-KEM-768"),
            PqcAlgorithm::MlKem1024 => ("ML-KEM", "ML-KEM-1024"),
            PqcAlgorithm::MlDsa44 => ("ML-DSA", "ML-DSA-44"),
            PqcAlgorithm::MlDsa65 => ("ML-DSA", "ML-DSA-65"),
            PqcAlgorithm::MlDsa87 => ("ML-DSA", "ML-DSA-87"),
            PqcAlgorithm::SlhDsaShake128f => ("SLH-DSA", "SLH-DSA-SHAKE-128f"),
            PqcAlgorithm::SlhDsaShake128s => ("SLH-DSA", "SLH-DSA-SHAKE-128s"),
            PqcAlgorithm::SlhDsaShake192f => ("SLH-DSA", "SLH-DSA-SHAKE-192f"),
            PqcAlgorithm::SlhDsaShake256f => ("SLH-DSA", "SLH-DSA-SHAKE-256f"),
            PqcAlgorithm::Hqc128 => ("HQC", "HQC-128"),
            PqcAlgorithm::Hqc192 => ("HQC", "HQC-192"),
            PqcAlgorithm::Hqc256 => ("HQC", "HQC-256"),
        };

        Self {
            name: name.to_string(),
            variant: variant.to_string(),
            nist_level: alg.nist_level(),
            key_type: alg.key_type(),
        }
    }
}

/// Key data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KeyData {
    pub format: String,
    pub public_key: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub secret_key: Option<String>,
    pub key_size_bytes: KeySizeBytes,
}

/// Key sizes
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KeySizeBytes {
    pub public: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub secret: Option<usize>,
}

/// Key metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KeyMetadata {
    pub generated_at: DateTime<Utc>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub expires_at: Option<DateTime<Utc>>,
    pub key_id: Uuid,
}

/// PQC Key Pair
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PqcKeyPair {
    pub wia_quantum_version: String,
    #[serde(rename = "type")]
    pub data_type: String,
    pub crypto_type: String,
    pub algorithm: AlgorithmInfo,
    pub key: KeyData,
    pub metadata: KeyMetadata,
}

impl PqcKeyPair {
    /// Generate a new key pair (mock implementation)
    pub fn generate(algorithm: PqcAlgorithm) -> Result<Self> {
        // Note: This is a mock implementation. In production,
        // use a proper PQC library like pqcrypto or liboqs.
        let key_size = algorithm.public_key_size();

        // Generate mock keys (random bytes encoded as base64)
        let mock_public = base64_encode(&vec![0u8; key_size]);
        let mock_secret = base64_encode(&vec![0u8; key_size * 2]);

        Ok(Self {
            wia_quantum_version: WIA_QUANTUM_VERSION.to_string(),
            data_type: "crypto".to_string(),
            crypto_type: "pqc_key".to_string(),
            algorithm: algorithm.into(),
            key: KeyData {
                format: "base64".to_string(),
                public_key: mock_public,
                secret_key: Some(mock_secret),
                key_size_bytes: KeySizeBytes {
                    public: key_size,
                    secret: Some(key_size * 2),
                },
            },
            metadata: KeyMetadata {
                generated_at: Utc::now(),
                expires_at: None,
                key_id: Uuid::new_v4(),
            },
        })
    }

    /// Get public key only
    pub fn public_key(&self) -> PqcKeyPair {
        let mut pk = self.clone();
        pk.key.secret_key = None;
        pk.key.key_size_bytes.secret = None;
        pk
    }

    /// Serialize to JSON
    pub fn to_json(&self) -> Result<String> {
        serde_json::to_string_pretty(self).map_err(Into::into)
    }

    /// Deserialize from JSON
    pub fn from_json(json: &str) -> Result<Self> {
        serde_json::from_str(json).map_err(Into::into)
    }
}

/// PQC Signature
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PqcSignature {
    pub wia_quantum_version: String,
    #[serde(rename = "type")]
    pub data_type: String,
    pub crypto_type: String,
    pub algorithm: AlgorithmInfo,
    pub signature: SignatureData,
    pub message_hash: MessageHash,
    pub public_key_id: Uuid,
    pub signed_at: DateTime<Utc>,
}

/// Signature data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignatureData {
    pub format: String,
    pub value: String,
    pub size_bytes: usize,
}

/// Message hash
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MessageHash {
    pub algorithm: String,
    pub value: String,
}

impl PqcSignature {
    /// Create mock signature (for testing)
    pub fn sign_mock(
        algorithm: PqcAlgorithm,
        message: &[u8],
        key_id: Uuid,
    ) -> Result<Self> {
        if algorithm.key_type() != PqcKeyType::Signature {
            return Err(QuantumError::CryptoError(
                "Algorithm is not a signature algorithm".to_string(),
            ));
        }

        // Mock signature
        let sig_size = match algorithm {
            PqcAlgorithm::MlDsa44 => 2420,
            PqcAlgorithm::MlDsa65 => 3309,
            PqcAlgorithm::MlDsa87 => 4627,
            _ => 1000,
        };

        let mock_sig = base64_encode(&vec![0u8; sig_size]);
        let msg_hash = hex::encode(&sha256_mock(message));

        Ok(Self {
            wia_quantum_version: WIA_QUANTUM_VERSION.to_string(),
            data_type: "crypto".to_string(),
            crypto_type: "pqc_signature".to_string(),
            algorithm: algorithm.into(),
            signature: SignatureData {
                format: "base64".to_string(),
                value: mock_sig,
                size_bytes: sig_size,
            },
            message_hash: MessageHash {
                algorithm: "SHA3-256".to_string(),
                value: msg_hash,
            },
            public_key_id: key_id,
            signed_at: Utc::now(),
        })
    }
}

// Helper functions
fn base64_encode(data: &[u8]) -> String {
    use std::io::Write;
    let mut buf = Vec::new();
    {
        let mut encoder = base64_writer(&mut buf);
        encoder.write_all(data).ok();
    }
    String::from_utf8(buf).unwrap_or_default()
}

fn base64_writer(w: &mut Vec<u8>) -> impl std::io::Write + '_ {
    struct Base64Writer<'a>(&'a mut Vec<u8>);
    impl<'a> std::io::Write for Base64Writer<'a> {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            const ALPHABET: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
            for chunk in buf.chunks(3) {
                let b0 = chunk.first().copied().unwrap_or(0);
                let b1 = chunk.get(1).copied().unwrap_or(0);
                let b2 = chunk.get(2).copied().unwrap_or(0);
                self.0.push(ALPHABET[(b0 >> 2) as usize]);
                self.0.push(ALPHABET[(((b0 & 0x03) << 4) | (b1 >> 4)) as usize]);
                if chunk.len() > 1 {
                    self.0.push(ALPHABET[(((b1 & 0x0f) << 2) | (b2 >> 6)) as usize]);
                }
                if chunk.len() > 2 {
                    self.0.push(ALPHABET[(b2 & 0x3f) as usize]);
                }
            }
            Ok(buf.len())
        }
        fn flush(&mut self) -> std::io::Result<()> { Ok(()) }
    }
    Base64Writer(w)
}

fn sha256_mock(data: &[u8]) -> [u8; 32] {
    // Simple mock hash (not cryptographically secure)
    let mut hash = [0u8; 32];
    for (i, byte) in data.iter().enumerate() {
        hash[i % 32] ^= byte;
    }
    hash
}

mod hex {
    pub fn encode(data: &[u8]) -> String {
        data.iter().map(|b| format!("{:02x}", b)).collect()
    }
}
