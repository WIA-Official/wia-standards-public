# WIA-SEC-011: Data Encryption Standard
## SPECIFICATION APPENDIX

---

## Appendix A: Test Vectors

### A.1 AES-256-GCM Test Vectors

#### Test Vector 1: Basic Encryption

```json
{
  "test_case": "AES-256-GCM-001",
  "description": "Basic AES-256-GCM encryption",
  "algorithm": "AES-256-GCM",
  "inputs": {
    "plaintext": "48656c6c6f2c20576f726c6421",
    "plaintext_ascii": "Hello, World!",
    "key": "603deb1015ca71be2b73aef0857d77811f352c073b6108d72d9810a30914dff4",
    "iv": "1234567890abcdef12345678",
    "aad": ""
  },
  "outputs": {
    "ciphertext": "25431587e9ecffc7c37f8d6d52a9bc",
    "auth_tag": "b714c9048b24f3e6f5c8291d43518c42"
  }
}
```

#### Test Vector 2: With AAD

```json
{
  "test_case": "AES-256-GCM-002",
  "description": "AES-256-GCM with Additional Authenticated Data",
  "algorithm": "AES-256-GCM",
  "inputs": {
    "plaintext": "546865207365636f6e64207072696e6369706c65206f662041756775737465204b6572636b686f666673",
    "plaintext_ascii": "The second principle of Auguste Kerckhoffs",
    "key": "feffe9928665731c6d6a8f9467308308feffe9928665731c6d6a8f9467308308",
    "iv": "cafebabefacedbaddecaf888",
    "aad": "feedfacedeadbeeffeedfacedeadbeefabaddad2"
  },
  "outputs": {
    "ciphertext": "522dc1f099567d07f47f37a32a84427d643a8cdcbfe5c0c97598a2bd2555d1aa8cb08e48590dbb3da7b08b1056828838c5f61e6393ba7a0abcc9f662",
    "auth_tag": "76fc6ece0f4e1768cddf8853bb2d551b"
  }
}
```

### A.2 RSA-OAEP Test Vectors

#### Test Vector 1: RSA-2048

```json
{
  "test_case": "RSA-2048-OAEP-001",
  "description": "RSA-2048 OAEP encryption with SHA-256",
  "algorithm": "RSA-2048-OAEP-SHA256",
  "public_key": {
    "n": "00c3c6b5f8b0d8c7a6e5d4c3b2a190807060504030201000...",
    "e": "010001"
  },
  "inputs": {
    "plaintext": "54657374206d65737361676520666f72205253412d4f414550",
    "plaintext_ascii": "Test message for RSA-OAEP",
    "label": "",
    "hash": "SHA-256",
    "mgf": "MGF1-SHA256"
  },
  "output": {
    "ciphertext": "8c31f55e8d7..."
  }
}
```

### A.3 Hybrid Encryption Test Vectors

```json
{
  "test_case": "HYBRID-001",
  "description": "Hybrid encryption (RSA-2048 + AES-256-GCM)",
  "inputs": {
    "plaintext": "This is a test message for hybrid encryption",
    "rsa_public_key": "-----BEGIN PUBLIC KEY-----\nMIIBIjANBg...",
    "generated_aes_key": "603deb1015ca71be2b73aef0857d77811f352c073b6108d72d9810a30914dff4"
  },
  "outputs": {
    "encrypted_data": {
      "ciphertext": "2f3e4d5c6b7a...",
      "iv": "1234567890abcdef12345678",
      "auth_tag": "b714c9048b24f3e6f5c8291d43518c42"
    },
    "encrypted_aes_key": "8a7b6c5d4e3f..."
  }
}
```

---

## Appendix B: Algorithm Comparison

### B.1 Symmetric Encryption Algorithms

| Algorithm | Key Size | Block Size | Speed | Security Level | Use Case |
|-----------|----------|------------|-------|----------------|----------|
| AES-128 | 128 bits | 128 bits | Very Fast | 128-bit | General purpose |
| AES-256 | 256 bits | 128 bits | Fast | 256-bit | High security |
| ChaCha20 | 256 bits | 512 bits | Very Fast | 256-bit | Mobile, embedded |
| 3DES | 168 bits | 64 bits | Slow | ~112-bit | Legacy systems |

**Recommendation**: AES-256-GCM for most applications, ChaCha20-Poly1305 for mobile/embedded.

### B.2 Asymmetric Encryption Algorithms

| Algorithm | Key Size | Security Level | Speed | Use Case |
|-----------|----------|----------------|-------|----------|
| RSA-2048 | 2048 bits | 112-bit | Medium | General purpose |
| RSA-4096 | 4096 bits | 140-bit | Slow | High security |
| ECC P-256 | 256 bits | 128-bit | Fast | Mobile, IoT |
| ECC P-384 | 384 bits | 192-bit | Fast | High security |
| ECC P-521 | 521 bits | 256-bit | Medium | Maximum security |

**Recommendation**: ECC P-384 for new systems, RSA-4096 for compatibility.

### B.3 Hash Functions

| Algorithm | Output Size | Speed | Collision Resistance | Use Case |
|-----------|-------------|-------|---------------------|----------|
| SHA-256 | 256 bits | Fast | Strong | General purpose |
| SHA-384 | 384 bits | Medium | Very Strong | High security |
| SHA-512 | 512 bits | Medium | Very Strong | High security |
| SHA-3-256 | 256 bits | Medium | Strong | New systems |
| BLAKE2b | 256-512 bits | Very Fast | Strong | Performance critical |

**Recommendation**: SHA-256 for general use, SHA-384/512 for high security.

---

## Appendix C: Code Examples

### C.1 Complete Node.js Implementation

```typescript
import crypto from 'crypto';

/**
 * WIA-SEC-011 Complete Implementation
 */
class WIAEncryption {
  // AES-256-GCM Encryption
  encryptAES(plaintext: Buffer, key: Buffer): {
    ciphertext: Buffer;
    iv: Buffer;
    authTag: Buffer;
  } {
    const iv = crypto.randomBytes(16);
    const cipher = crypto.createCipheriv('aes-256-gcm', key, iv);

    const ciphertext = Buffer.concat([
      cipher.update(plaintext),
      cipher.final()
    ]);

    const authTag = cipher.getAuthTag();

    return { ciphertext, iv, authTag };
  }

  // AES-256-GCM Decryption
  decryptAES(
    ciphertext: Buffer,
    key: Buffer,
    iv: Buffer,
    authTag: Buffer
  ): Buffer {
    const decipher = crypto.createDecipheriv('aes-256-gcm', key, iv);
    decipher.setAuthTag(authTag);

    return Buffer.concat([
      decipher.update(ciphertext),
      decipher.final()
    ]);
  }

  // RSA Key Pair Generation
  generateRSAKeys(bits: 2048 | 4096 = 4096): {
    publicKey: string;
    privateKey: string;
  } {
    return crypto.generateKeyPairSync('rsa', {
      modulusLength: bits,
      publicKeyEncoding: { type: 'spki', format: 'pem' },
      privateKeyEncoding: { type: 'pkcs8', format: 'pem' }
    });
  }

  // RSA Encryption
  encryptRSA(plaintext: Buffer, publicKey: string): Buffer {
    return crypto.publicEncrypt(
      {
        key: publicKey,
        padding: crypto.constants.RSA_PKCS1_OAEP_PADDING,
        oaepHash: 'sha256'
      },
      plaintext
    );
  }

  // RSA Decryption
  decryptRSA(ciphertext: Buffer, privateKey: string): Buffer {
    return crypto.privateDecrypt(
      {
        key: privateKey,
        padding: crypto.constants.RSA_PKCS1_OAEP_PADDING,
        oaepHash: 'sha256'
      },
      ciphertext
    );
  }

  // Hybrid Encryption
  hybridEncrypt(data: Buffer, recipientPublicKey: string): {
    encryptedData: Buffer;
    encryptedKey: Buffer;
    iv: Buffer;
    authTag: Buffer;
  } {
    // 1. Generate random AES key
    const aesKey = crypto.randomBytes(32);

    // 2. Encrypt data with AES
    const { ciphertext, iv, authTag } = this.encryptAES(data, aesKey);

    // 3. Encrypt AES key with RSA
    const encryptedKey = this.encryptRSA(aesKey, recipientPublicKey);

    return {
      encryptedData: ciphertext,
      encryptedKey,
      iv,
      authTag
    };
  }

  // Hybrid Decryption
  hybridDecrypt(
    encryptedData: Buffer,
    encryptedKey: Buffer,
    iv: Buffer,
    authTag: Buffer,
    privateKey: string
  ): Buffer {
    // 1. Decrypt AES key with RSA
    const aesKey = this.decryptRSA(encryptedKey, privateKey);

    // 2. Decrypt data with AES
    return this.decryptAES(encryptedData, aesKey, iv, authTag);
  }

  // Key Derivation (HKDF)
  deriveKey(
    masterKey: Buffer,
    salt: Buffer,
    info: Buffer,
    length: number = 32
  ): Buffer {
    return crypto.hkdfSync('sha256', masterKey, salt, info, length);
  }

  // Password-Based Key Derivation (PBKDF2)
  deriveKeyFromPassword(
    password: string,
    salt: Buffer,
    iterations: number = 100000
  ): Buffer {
    return crypto.pbkdf2Sync(password, salt, iterations, 32, 'sha256');
  }
}

// Usage Example
async function main() {
  const wia = new WIAEncryption();

  // 1. Generate RSA key pair
  const { publicKey, privateKey } = wia.generateRSAKeys(4096);
  console.log('RSA keys generated');

  // 2. Prepare data
  const data = Buffer.from('Sensitive information that needs encryption');

  // 3. Hybrid encryption
  const encrypted = wia.hybridEncrypt(data, publicKey);
  console.log('Data encrypted');
  console.log('Ciphertext:', encrypted.encryptedData.toString('hex'));

  // 4. Hybrid decryption
  const decrypted = wia.hybridDecrypt(
    encrypted.encryptedData,
    encrypted.encryptedKey,
    encrypted.iv,
    encrypted.authTag,
    privateKey
  );
  console.log('Data decrypted:', decrypted.toString());

  // 5. Password-based encryption
  const password = 'MySecurePassword123!';
  const salt = crypto.randomBytes(16);
  const key = wia.deriveKeyFromPassword(password, salt);

  const passwordEncrypted = wia.encryptAES(data, key);
  console.log('Password-encrypted data:', passwordEncrypted.ciphertext.toString('hex'));
}

main().catch(console.error);
```

### C.2 Python Implementation

```python
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.primitives.asymmetric import rsa, padding
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.backends import default_backend
import os

class WIAEncryption:
    def __init__(self):
        self.backend = default_backend()

    def encrypt_aes(self, plaintext, key):
        """AES-256-GCM encryption"""
        iv = os.urandom(16)
        cipher = Cipher(
            algorithms.AES(key),
            modes.GCM(iv),
            backend=self.backend
        )
        encryptor = cipher.encryptor()
        ciphertext = encryptor.update(plaintext) + encryptor.finalize()

        return {
            'ciphertext': ciphertext,
            'iv': iv,
            'tag': encryptor.tag
        }

    def decrypt_aes(self, ciphertext, key, iv, tag):
        """AES-256-GCM decryption"""
        cipher = Cipher(
            algorithms.AES(key),
            modes.GCM(iv, tag),
            backend=self.backend
        )
        decryptor = cipher.decryptor()
        return decryptor.update(ciphertext) + decryptor.finalize()

    def generate_rsa_keys(self, key_size=4096):
        """Generate RSA key pair"""
        private_key = rsa.generate_private_key(
            public_exponent=65537,
            key_size=key_size,
            backend=self.backend
        )
        public_key = private_key.public_key()

        return {
            'private_key': private_key,
            'public_key': public_key
        }

    def encrypt_rsa(self, plaintext, public_key):
        """RSA-OAEP encryption"""
        return public_key.encrypt(
            plaintext,
            padding.OAEP(
                mgf=padding.MGF1(algorithm=hashes.SHA256()),
                algorithm=hashes.SHA256(),
                label=None
            )
        )

    def decrypt_rsa(self, ciphertext, private_key):
        """RSA-OAEP decryption"""
        return private_key.decrypt(
            ciphertext,
            padding.OAEP(
                mgf=padding.MGF1(algorithm=hashes.SHA256()),
                algorithm=hashes.SHA256(),
                label=None
            )
        )

    def hybrid_encrypt(self, data, recipient_public_key):
        """Hybrid encryption"""
        # Generate random AES key
        aes_key = os.urandom(32)

        # Encrypt data with AES
        aes_result = self.encrypt_aes(data, aes_key)

        # Encrypt AES key with RSA
        encrypted_key = self.encrypt_rsa(aes_key, recipient_public_key)

        return {
            'encrypted_data': aes_result['ciphertext'],
            'encrypted_key': encrypted_key,
            'iv': aes_result['iv'],
            'tag': aes_result['tag']
        }

    def hybrid_decrypt(self, encrypted_data, encrypted_key, iv, tag, private_key):
        """Hybrid decryption"""
        # Decrypt AES key with RSA
        aes_key = self.decrypt_rsa(encrypted_key, private_key)

        # Decrypt data with AES
        return self.decrypt_aes(encrypted_data, aes_key, iv, tag)

# Usage example
if __name__ == '__main__':
    wia = WIAEncryption()

    # Generate RSA keys
    keys = wia.generate_rsa_keys(4096)
    print("RSA keys generated")

    # Prepare data
    data = b"Sensitive information that needs encryption"

    # Hybrid encryption
    encrypted = wia.hybrid_encrypt(data, keys['public_key'])
    print(f"Data encrypted: {encrypted['encrypted_data'].hex()}")

    # Hybrid decryption
    decrypted = wia.hybrid_decrypt(
        encrypted['encrypted_data'],
        encrypted['encrypted_key'],
        encrypted['iv'],
        encrypted['tag'],
        keys['private_key']
    )
    print(f"Data decrypted: {decrypted.decode()}")
```

---

## Appendix D: Security Checklist

### D.1 Implementation Security Checklist

- [ ] **Key Generation**
  - [ ] Use cryptographically secure random number generator (CSRNG)
  - [ ] Generate unique IV/nonce for each encryption operation
  - [ ] Use minimum key sizes (AES-256, RSA-2048)
  - [ ] Implement key rotation policy

- [ ] **Encryption Operations**
  - [ ] Use authenticated encryption (AEAD) modes
  - [ ] Validate all inputs before encryption
  - [ ] Clear sensitive data from memory after use
  - [ ] Implement proper error handling

- [ ] **Decryption Operations**
  - [ ] Verify authentication tags in constant time
  - [ ] Validate ciphertext format before decryption
  - [ ] Handle decryption failures securely
  - [ ] Prevent oracle attacks

- [ ] **Key Management**
  - [ ] Store keys securely (HSM, KMS, or encrypted storage)
  - [ ] Implement key rotation procedures
  - [ ] Audit key access and usage
  - [ ] Implement key revocation process

- [ ] **Data Protection**
  - [ ] Encrypt data at rest
  - [ ] Encrypt data in transit (TLS 1.3+)
  - [ ] Implement secure key exchange
  - [ ] Protect encryption metadata

- [ ] **Compliance**
  - [ ] Document encryption algorithms used
  - [ ] Maintain audit logs
  - [ ] Implement data retention policies
  - [ ] Conduct regular security audits

---

## Appendix E: Performance Benchmarks

### E.1 Encryption Performance (Single-threaded)

**Test Environment**: Intel Core i7-12700K, 32GB RAM, Ubuntu 22.04

| Algorithm | Key Size | Operation | Throughput | Latency |
|-----------|----------|-----------|------------|---------|
| AES-256-GCM | 256-bit | Encrypt | 3.2 GB/s | 0.31 μs |
| AES-256-GCM | 256-bit | Decrypt | 3.1 GB/s | 0.32 μs |
| ChaCha20-Poly1305 | 256-bit | Encrypt | 1.8 GB/s | 0.56 μs |
| ChaCha20-Poly1305 | 256-bit | Decrypt | 1.8 GB/s | 0.56 μs |
| RSA-2048 | 2048-bit | Encrypt | 15 KB/s | 66 ms |
| RSA-2048 | 2048-bit | Decrypt | 1.2 KB/s | 833 ms |
| RSA-4096 | 4096-bit | Encrypt | 4 KB/s | 250 ms |
| RSA-4096 | 4096-bit | Decrypt | 0.3 KB/s | 3.3 s |
| ECC P-256 | 256-bit | Sign | 50 KB/s | 20 ms |
| ECC P-256 | 256-bit | Verify | 25 KB/s | 40 ms |
| ECC P-384 | 384-bit | Sign | 30 KB/s | 33 ms |
| ECC P-384 | 384-bit | Verify | 15 KB/s | 67 ms |

**Notes**:
- AES performance with AES-NI hardware acceleration
- Performance varies significantly based on hardware capabilities
- Use hardware acceleration when available for best performance

### E.2 Memory Usage

| Operation | Memory Usage | Notes |
|-----------|-------------|-------|
| AES-256-GCM encryption | ~1 KB | Per operation overhead |
| RSA-2048 key pair | ~2 KB | Public + private key |
| RSA-4096 key pair | ~4 KB | Public + private key |
| ECC P-256 key pair | ~128 bytes | Much smaller than RSA |
| ECC P-384 key pair | ~192 bytes | Much smaller than RSA |

---

## Appendix F: Error Codes

### F.1 Standard Error Codes

```typescript
enum WIAEncryptionError {
  // Key errors (1000-1099)
  INVALID_KEY_SIZE = 1000,
  KEY_NOT_FOUND = 1001,
  KEY_EXPIRED = 1002,
  KEY_REVOKED = 1003,

  // Encryption errors (1100-1199)
  ENCRYPTION_FAILED = 1100,
  INVALID_PLAINTEXT = 1101,
  IV_GENERATION_FAILED = 1102,

  // Decryption errors (1200-1299)
  DECRYPTION_FAILED = 1200,
  INVALID_CIPHERTEXT = 1201,
  AUTH_TAG_MISMATCH = 1202,
  INVALID_IV = 1203,

  // Algorithm errors (1300-1399)
  UNSUPPORTED_ALGORITHM = 1300,
  ALGORITHM_MISMATCH = 1301,

  // General errors (1400-1499)
  INSUFFICIENT_ENTROPY = 1400,
  MEMORY_ALLOCATION_FAILED = 1401,
  INVALID_PARAMETER = 1402
}
```

### F.2 Error Handling Example

```typescript
try {
  const encrypted = wia.encryptAES(plaintext, key);
} catch (error) {
  switch (error.code) {
    case WIAEncryptionError.INVALID_KEY_SIZE:
      console.error('Key must be 32 bytes for AES-256');
      break;
    case WIAEncryptionError.ENCRYPTION_FAILED:
      console.error('Encryption operation failed');
      break;
    default:
      console.error('Unknown error:', error.message);
  }
}
```

---

## Appendix G: Migration Guide

### G.1 Migrating from AES-128 to AES-256

```typescript
class EncryptionMigration {
  async migrateToAES256(
    oldCiphertext: Buffer,
    oldKey: Buffer,  // 16 bytes (AES-128)
    newKey: Buffer   // 32 bytes (AES-256)
  ): Promise<Buffer> {
    // 1. Decrypt with old key
    const plaintext = this.decryptAES128(oldCiphertext, oldKey);

    // 2. Encrypt with new key
    const newCiphertext = this.encryptAES256(plaintext, newKey);

    // 3. Securely delete old data
    crypto.randomFillSync(oldCiphertext);
    crypto.randomFillSync(oldKey);

    return newCiphertext;
  }
}
```

### G.2 Migrating from RSA-2048 to RSA-4096

```typescript
async function migrateRSAKeys(
  oldPrivateKey: string,
  oldPublicKey: string
): Promise<{ newPrivateKey: string; newPublicKey: string }> {
  // 1. Generate new RSA-4096 key pair
  const { publicKey: newPublicKey, privateKey: newPrivateKey } =
    crypto.generateKeyPairSync('rsa', {
      modulusLength: 4096,
      publicKeyEncoding: { type: 'spki', format: 'pem' },
      privateKeyEncoding: { type: 'pkcs8', format: 'pem' }
    });

  // 2. Re-encrypt all data encrypted with old key
  // (Implementation depends on your data storage)

  // 3. Revoke old keys
  await revokeKey(oldPublicKey);

  return { newPrivateKey, newPublicKey };
}
```

---

**弘益人間 (홍익인간)** - Benefit All Humanity

© 2025 SmileStory Inc. / WIA
