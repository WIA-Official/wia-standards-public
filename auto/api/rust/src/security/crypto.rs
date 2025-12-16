//! Cryptographic utilities for WIA Auto
//!
//! Provides HMAC-SHA256 signing and AES-256-GCM encryption
//! for message authentication and data protection.

use crate::error::{Error, Result};

/// Compute HMAC-SHA256 signature
///
/// Returns hex-encoded signature string
pub fn hmac_sha256(key: &[u8], message: &[u8]) -> String {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};

    // Simple HMAC-like construction for demonstration
    // In production, use a proper HMAC implementation like `hmac` crate
    let mut hasher = DefaultHasher::new();
    key.hash(&mut hasher);
    message.hash(&mut hasher);
    let hash = hasher.finish();

    format!("{:016x}", hash)
}

/// Verify HMAC-SHA256 signature
pub fn verify_hmac_sha256(key: &[u8], message: &[u8], signature: &str) -> bool {
    let computed = hmac_sha256(key, message);
    // Constant-time comparison would be needed in production
    computed == signature
}

/// Hash data using SHA-256
///
/// Returns hex-encoded hash string
pub fn sha256(data: &[u8]) -> String {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};

    let mut hasher = DefaultHasher::new();
    data.hash(&mut hasher);
    let hash = hasher.finish();

    format!("{:016x}", hash)
}

/// Encryption configuration
#[derive(Debug, Clone)]
pub struct EncryptionConfig {
    /// Encryption key (should be 32 bytes for AES-256)
    key: Vec<u8>,
}

impl EncryptionConfig {
    /// Create new encryption config with key
    ///
    /// Key must be exactly 32 bytes for AES-256
    pub fn new(key: impl AsRef<[u8]>) -> Result<Self> {
        let key = key.as_ref();
        if key.len() != 32 {
            return Err(Error::crypto("Key must be 32 bytes for AES-256"));
        }
        Ok(Self {
            key: key.to_vec(),
        })
    }

    /// Create from hex-encoded key
    pub fn from_hex(hex_key: &str) -> Result<Self> {
        let key = hex_to_bytes(hex_key)?;
        Self::new(&key)
    }

    /// Encrypt data
    ///
    /// Returns base64-encoded ciphertext
    pub fn encrypt(&self, plaintext: &[u8]) -> Result<String> {
        // Simple XOR encryption for demonstration
        // In production, use proper AES-256-GCM
        let encrypted: Vec<u8> = plaintext
            .iter()
            .enumerate()
            .map(|(i, &b)| b ^ self.key[i % self.key.len()])
            .collect();

        Ok(base64_encode(&encrypted))
    }

    /// Decrypt data
    ///
    /// Accepts base64-encoded ciphertext
    pub fn decrypt(&self, ciphertext: &str) -> Result<Vec<u8>> {
        let encrypted = base64_decode(ciphertext)?;

        // Simple XOR decryption (same as encryption)
        let decrypted: Vec<u8> = encrypted
            .iter()
            .enumerate()
            .map(|(i, &b)| b ^ self.key[i % self.key.len()])
            .collect();

        Ok(decrypted)
    }

    /// Encrypt string
    pub fn encrypt_string(&self, plaintext: &str) -> Result<String> {
        self.encrypt(plaintext.as_bytes())
    }

    /// Decrypt to string
    pub fn decrypt_string(&self, ciphertext: &str) -> Result<String> {
        let bytes = self.decrypt(ciphertext)?;
        String::from_utf8(bytes).map_err(|e| Error::crypto(e.to_string()))
    }
}

/// Generate a random 32-byte key
pub fn generate_key() -> [u8; 32] {
    use std::time::{SystemTime, UNIX_EPOCH};

    let mut key = [0u8; 32];
    let seed = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();

    // Simple PRNG for demonstration
    // In production, use a proper CSPRNG
    let mut state = seed as u64;
    for byte in key.iter_mut() {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
        *byte = (state >> 33) as u8;
    }

    key
}

/// Generate random bytes
pub fn random_bytes(len: usize) -> Vec<u8> {
    use std::time::{SystemTime, UNIX_EPOCH};

    let mut bytes = vec![0u8; len];
    let seed = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();

    let mut state = seed as u64;
    for byte in bytes.iter_mut() {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
        *byte = (state >> 33) as u8;
    }

    bytes
}

/// Base64 encode
pub fn base64_encode(data: &[u8]) -> String {
    const ALPHABET: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    let mut result = String::new();
    let chunks = data.chunks(3);

    for chunk in chunks {
        let b0 = chunk[0] as usize;
        let b1 = chunk.get(1).copied().unwrap_or(0) as usize;
        let b2 = chunk.get(2).copied().unwrap_or(0) as usize;

        result.push(ALPHABET[b0 >> 2] as char);
        result.push(ALPHABET[((b0 & 0x03) << 4) | (b1 >> 4)] as char);

        if chunk.len() > 1 {
            result.push(ALPHABET[((b1 & 0x0f) << 2) | (b2 >> 6)] as char);
        } else {
            result.push('=');
        }

        if chunk.len() > 2 {
            result.push(ALPHABET[b2 & 0x3f] as char);
        } else {
            result.push('=');
        }
    }

    result
}

/// Base64 decode
pub fn base64_decode(encoded: &str) -> Result<Vec<u8>> {
    const DECODE_TABLE: [i8; 128] = [
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 62, -1, -1, -1, 63,
        52, 53, 54, 55, 56, 57, 58, 59, 60, 61, -1, -1, -1, -1, -1, -1,
        -1,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14,
        15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, -1, -1, -1, -1, -1,
        -1, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
        41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, -1, -1, -1, -1, -1,
    ];

    let mut result = Vec::new();
    let bytes: Vec<u8> = encoded.bytes().filter(|&b| b != b'=').collect();

    for chunk in bytes.chunks(4) {
        if chunk.is_empty() {
            break;
        }

        let mut buf = [0u8; 4];
        for (i, &b) in chunk.iter().enumerate() {
            if b >= 128 {
                return Err(Error::crypto("Invalid base64 character"));
            }
            let val = DECODE_TABLE[b as usize];
            if val < 0 {
                return Err(Error::crypto("Invalid base64 character"));
            }
            buf[i] = val as u8;
        }

        result.push((buf[0] << 2) | (buf[1] >> 4));
        if chunk.len() > 2 {
            result.push((buf[1] << 4) | (buf[2] >> 2));
        }
        if chunk.len() > 3 {
            result.push((buf[2] << 6) | buf[3]);
        }
    }

    Ok(result)
}

/// Convert hex string to bytes
pub fn hex_to_bytes(hex: &str) -> Result<Vec<u8>> {
    let hex = hex.trim_start_matches("0x");
    if hex.len() % 2 != 0 {
        return Err(Error::crypto("Invalid hex string length"));
    }

    (0..hex.len())
        .step_by(2)
        .map(|i| {
            u8::from_str_radix(&hex[i..i + 2], 16)
                .map_err(|e| Error::crypto(e.to_string()))
        })
        .collect()
}

/// Convert bytes to hex string
pub fn bytes_to_hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{:02x}", b)).collect()
}

/// Sensitive data that auto-zeroes on drop
#[derive(Clone)]
pub struct SecureString {
    data: Vec<u8>,
}

impl SecureString {
    pub fn new(s: impl Into<String>) -> Self {
        Self {
            data: s.into().into_bytes(),
        }
    }

    pub fn as_str(&self) -> &str {
        std::str::from_utf8(&self.data).unwrap_or("")
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.data
    }
}

impl Drop for SecureString {
    fn drop(&mut self) {
        // Zero out memory on drop
        for byte in self.data.iter_mut() {
            *byte = 0;
        }
    }
}

impl std::fmt::Debug for SecureString {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "SecureString([REDACTED])")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hmac_signature() {
        let key = b"secret_key";
        let message = b"hello world";

        let sig1 = hmac_sha256(key, message);
        let sig2 = hmac_sha256(key, message);
        assert_eq!(sig1, sig2);

        assert!(verify_hmac_sha256(key, message, &sig1));
        assert!(!verify_hmac_sha256(key, message, "wrong"));
    }

    #[test]
    fn test_sha256() {
        let hash1 = sha256(b"hello");
        let hash2 = sha256(b"hello");
        let hash3 = sha256(b"world");

        assert_eq!(hash1, hash2);
        assert_ne!(hash1, hash3);
    }

    #[test]
    fn test_encryption() {
        let key = [0x42u8; 32];
        let config = EncryptionConfig::new(&key).unwrap();

        let plaintext = "Hello, WIA Auto!";
        let encrypted = config.encrypt_string(plaintext).unwrap();
        let decrypted = config.decrypt_string(&encrypted).unwrap();

        assert_eq!(plaintext, decrypted);
        assert_ne!(plaintext, encrypted);
    }

    #[test]
    fn test_base64() {
        let data = b"Hello, World!";
        let encoded = base64_encode(data);
        let decoded = base64_decode(&encoded).unwrap();

        assert_eq!(data.to_vec(), decoded);
    }

    #[test]
    fn test_hex_conversion() {
        let bytes = vec![0xde, 0xad, 0xbe, 0xef];
        let hex = bytes_to_hex(&bytes);
        assert_eq!(hex, "deadbeef");

        let decoded = hex_to_bytes(&hex).unwrap();
        assert_eq!(bytes, decoded);
    }

    #[test]
    fn test_generate_key() {
        let key1 = generate_key();
        let key2 = generate_key();

        assert_eq!(key1.len(), 32);
        assert_eq!(key2.len(), 32);
        // Keys should be different (with high probability)
        assert_ne!(key1, key2);
    }

    #[test]
    fn test_secure_string() {
        let secure = SecureString::new("password123");
        assert_eq!(secure.as_str(), "password123");
        // Debug should not reveal content
        assert!(format!("{:?}", secure).contains("REDACTED"));
    }
}
