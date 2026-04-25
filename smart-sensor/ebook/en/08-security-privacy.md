# Chapter 8: Security, Privacy, and OTA Updates

## Protecting Smart Sensors from Threats

As smart sensors proliferate in critical infrastructure, healthcare, and consumer applications, security becomes paramount. This chapter covers threat models, defense mechanisms, and secure update procedures.

---

## Threat Landscape for Smart Sensors

### Attack Vectors

**Physical Attacks:**
- Device tampering (chip decapping, probing)
- Side-channel analysis (power, EM, timing)
- Fault injection (glitching, laser, EM pulses)
- Reverse engineering firmware

**Network Attacks:**
- Eavesdropping on wireless communication
- Man-in-the-middle attacks
- Replay attacks
- Denial of service (jamming, flooding)

**Software Attacks:**
- Firmware exploitation (buffer overflow, ROP)
- Privilege escalation
- Malware injection
- Supply chain compromise

**Data Attacks:**
- Privacy leakage (inferring user behavior)
- Data exfiltration
- Sensor spoofing (feeding false data)

### Consequences

**Safety:** Malicious control of actuators, medical devices
**Privacy:** Exposure of personal data, location tracking
**Availability:** Denial of service, ransomware
**Financial:** Data theft, service disruption
**Reputation:** Loss of customer trust

---

## Hardware Security

### Secure Boot

**Objective:** Ensure only authentic firmware executes.

**Process:**
1. ROM bootloader verifies Stage 1 bootloader signature
2. Stage 1 verifies Stage 2 (if applicable)
3. Final bootloader verifies application firmware
4. Execution only if all signatures valid

**Implementation:**

```cpp
#include "crypto.h"
#include "flash.h"

#define FLASH_APP_START 0x08010000
#define FLASH_APP_SIZE  (256 * 1024)
#define FLASH_SIG_ADDR  (FLASH_APP_START + FLASH_APP_SIZE)

typedef struct {
    uint8_t signature[64];  // ECDSA signature
    uint8_t hash[32];       // SHA-256 hash
    uint32_t version;
    uint32_t size;
} FirmwareHeader;

bool secure_boot() {
    // Read firmware header
    FirmwareHeader header;
    flash_read(FLASH_SIG_ADDR, (uint8_t*)&header, sizeof(header));

    // Compute hash of application firmware
    uint8_t computed_hash[32];
    sha256_init();
    sha256_update((uint8_t*)FLASH_APP_START, header.size);
    sha256_final(computed_hash);

    // Verify hash matches
    if (memcmp(computed_hash, header.hash, 32) != 0) {
        return false;  // Hash mismatch
    }

    // Verify signature with public key (stored in OTP/ROM)
    const uint8_t public_key[64] = { /* OEM public key */ };
    if (!ecdsa_verify(header.hash, 32, header.signature, public_key)) {
        return false;  // Invalid signature
    }

    // All checks passed
    return true;
}

void bootloader_main() {
    if (secure_boot()) {
        // Jump to application
        void (*app_entry)(void) = *(void**)(FLASH_APP_START + 4);
        app_entry();
    } else {
        // Boot failed, enter recovery mode
        recovery_mode();
    }
}
```

**Key Storage:**
- Public key: In OTP (One-Time Programmable) fuse or ROM
- Private key: Offline, secure HSM (Hardware Security Module)

### Hardware Root of Trust

**ARM TrustZone (Cortex-M33, M55):**
- Secure/Non-Secure world separation
- Secure firmware isolated from application
- Secure storage, crypto operations

**Example: Using TrustZone for Key Storage**

```cpp
// Secure world code
__attribute__((cmse_nonsecure_entry))
int secure_decrypt(uint8_t* ciphertext, int len, uint8_t* plaintext) {
    // AES key stored in secure SRAM (inaccessible from non-secure)
    static const uint8_t aes_key[16] = { /* secret key */ };

    // Decrypt in secure world
    aes_decrypt(ciphertext, len, aes_key, plaintext);

    // Zero key material from stack/registers before returning
    memset_secure(aes_key, 0, sizeof(aes_key));

    return 0;
}

// Non-secure world (application) code
void application_process_data() {
    uint8_t encrypted_data[128];
    uint8_t decrypted_data[128];

    // Receive encrypted sensor data
    receive_data(encrypted_data, sizeof(encrypted_data));

    // Call secure function (key never exposed to application)
    secure_decrypt(encrypted_data, sizeof(encrypted_data), decrypted_data);

    // Process decrypted data
    process(decrypted_data);
}
```

**Benefits:**
- Keys never accessible to application code
- Isolation even if application compromised
- Minimal performance overhead

### Tamper Detection

**Physical Tamper Sensors:**
- Case open detection (switch/hall sensor)
- Mesh layer on PCB (cut detection)
- Temperature/voltage monitors (glitch detection)

**Action on Tamper:**
```cpp
void tamper_irq_handler() {
    // Detected case opening or voltage glitch

    // Erase cryptographic keys
    flash_erase(KEY_STORAGE_ADDR);

    // Zeroize SRAM
    memset((void*)SRAM_START, 0, SRAM_SIZE);

    // Set tamper flag in OTP (permanent)
    otp_write(TAMPER_FLAG_ADDR, 0xDEAD);

    // Reset device
    NVIC_SystemReset();
}
```

### Side-Channel Protection

**Power Analysis Countermeasures:**
- Random delays
- Dummy operations
- Balanced logic (DPA-resistant)
- Hardware crypto accelerators (constant-time)

**Timing Attack Mitigation:**
```cpp
// VULNERABLE: Early exit leaks information
bool constant_time_compare(uint8_t* a, uint8_t* b, int len) {
    for (int i = 0; i < len; i++) {
        if (a[i] != b[i]) {
            return false;  // Early exit reveals position of mismatch!
        }
    }
    return true;
}

// SECURE: Constant-time comparison
bool secure_compare(uint8_t* a, uint8_t* b, int len) {
    uint8_t diff = 0;
    for (int i = 0; i < len; i++) {
        diff |= a[i] ^ b[i];  // Accumulate differences
    }
    return (diff == 0);  // Always compares all bytes
}
```

---

## Communication Security

### Transport Layer Security (TLS)

**TLS 1.3 Benefits:**
- Mandatory encryption
- Forward secrecy (ephemeral keys)
- Reduced handshake (1-RTT or 0-RTT)
- Simplified cipher suites

**Embedded TLS Libraries:**
- **mbedTLS**: Comprehensive, modular (70-200 KB)
- **WolfSSL**: Optimized for embedded (20-100 KB)
- **TinyDTLS**: Minimal for CoAP (< 40 KB)

**Example: DTLS for CoAP**

```cpp
#include "tinydtls.h"

dtls_context_t *dtls_ctx;

void dtls_init() {
    // Pre-shared key (PSK) mode
    const uint8_t psk_identity[] = "sensor_001";
    const uint8_t psk_key[16] = { /* 128-bit key */ };

    dtls_ctx = dtls_new_context(NULL);
    dtls_set_psk(dtls_ctx, psk_identity, sizeof(psk_identity),
                psk_key, sizeof(psk_key));
}

void send_secure_coap(uint8_t* payload, int len) {
    // DTLS handshake (if not established)
    if (!dtls_is_connected(dtls_ctx)) {
        dtls_connect(dtls_ctx, server_addr);
    }

    // Send encrypted CoAP message
    dtls_write(dtls_ctx, payload, len);
}
```

**Power Impact:**
- DTLS handshake: ~100-500 ms, 50-200 mA
- Ongoing encryption: +10-20% power vs. plaintext
- Amortize over session: Use long-lived connections

### BLE Security

**BLE Pairing:**
- **Just Works**: No authentication (MITM vulnerable)
- **Passkey Entry**: 6-digit PIN (better)
- **Numeric Comparison**: Both devices display code (good)
- **Out-of-Band**: NFC, QR code (best for IoT)

**BLE Security Levels:**
- **LE Security Mode 1, Level 1**: No security
- **LE Security Mode 1, Level 2**: Unauthenticated pairing, encryption
- **LE Security Mode 1, Level 3**: Authenticated pairing, encryption
- **LE Security Mode 1, Level 4**: Authenticated LE Secure Connections, AES-128-CCM

**Recommended: Mode 1, Level 3 or 4**

```cpp
void ble_security_init() {
    // Enable authenticated pairing
    ble_gap_sec_params_t sec_params = {
        .bond = 1,           // Store pairing info
        .mitm = 1,           // MITM protection required
        .lesc = 1,           // LE Secure Connections
        .keypress = 0,
        .io_caps = BLE_GAP_IO_CAPS_DISPLAY_ONLY,  // For passkey entry
        .oob = 0,
        .min_key_size = 16,  // Maximum key size
        .max_key_size = 16
    };

    ble_gap_sec_params_set(&sec_params);
}
```

---

## Privacy Protection

### Data Minimization

**Principle:** Collect only necessary data, transmit minimal information.

```cpp
// POOR PRIVACY: Send raw accelerometer data (infers activity, gait, health)
void send_raw_accel() {
    for (int i = 0; i < 1000; i++) {
        send_to_cloud(accel_x[i], accel_y[i], accel_z[i], timestamp[i]);
    }
    // 12 KB transmitted, detailed activity patterns exposed
}

// BETTER: Send aggregated features
void send_activity_summary() {
    ActivitySummary summary = {
        .step_count = 8547,
        .active_minutes = 67,
        .sedentary_minutes = 133,
        .calories = 325
    };
    send_to_cloud(&summary, sizeof(summary));
    // 16 bytes transmitted, minimal privacy leakage
}

// BEST: Process entirely on-device
void on_device_activity_tracking() {
    // ML classification runs locally
    Activity activity = classify_activity(accel_data);

    // Only send summary statistics
    update_local_stats(activity);

    // Periodic sync of aggregated data
    if (time_for_sync()) {
        send_activity_summary();
    }
}
```

### Differential Privacy

**Add calibrated noise to protect individual privacy:**

```cpp
float add_laplace_noise(float value, float sensitivity, float epsilon) {
    // Laplace mechanism for differential privacy
    // epsilon: Privacy budget (smaller = more private)
    float scale = sensitivity / epsilon;

    // Generate Laplace noise
    float u = rand_uniform() - 0.5;
    float noise = -scale * sign(u) * log(1 - 2*fabs(u));

    return value + noise;
}

// Example: Room occupancy sensing
void report_occupancy_count() {
    int actual_count = count_people();

    // Add noise for privacy (epsilon = 0.1)
    float noisy_count = add_laplace_noise(actual_count, 1.0, 0.1);

    send_to_cloud((int)noisy_count);

    // Individual presence cannot be determined, but aggregate trends preserved
}
```

### Anonymization

**Remove or pseudonymize identifying information:**

```cpp
typedef struct {
    uint8_t device_id[16];   // Unique device ID
    uint32_t timestamp;
    float temperature;
} SensorReport;

void anonymize_report(SensorReport* report) {
    // Hash device ID with daily salt (pseudonymization)
    uint8_t salt[16];
    get_daily_salt(salt);

    uint8_t pseudonym[16];
    hmac_sha256(report->device_id, sizeof(report->device_id),
               salt, sizeof(salt), pseudonym);

    memcpy(report->device_id, pseudonym, sizeof(pseudonym));

    // Truncate timestamp to hour (k-anonymity)
    report->timestamp = (report->timestamp / 3600) * 3600;

    // Quantize temperature to reduce precision
    report->temperature = floor(report->temperature * 2) / 2;  // 0.5°C bins
}
```

---

## Over-The-Air (OTA) Updates

### OTA Architecture

```
[Cloud] → [Gateway] → [Sensor]
  ↓
[Firmware Image]
[Signature]
[Metadata]
```

**Requirements:**
- **Atomic updates**: New firmware fully written before activation
- **Rollback**: Ability to revert on failure
- **Integrity**: Cryptographic verification
- **Incremental**: Delta updates to save bandwidth

### Dual-Bank Flash

**Layout:**
```
┌──────────────┐ 0x08000000
│ Bootloader   │
├──────────────┤ 0x08010000
│ Bank A       │ (Active firmware)
│ (256 KB)     │
├──────────────┤ 0x08050000
│ Bank B       │ (Update staging)
│ (256 KB)     │
├──────────────┤ 0x08090000
│ Config/Data  │
└──────────────┘
```

**Update Process:**

```cpp
typedef enum {
    BOOT_BANK_A,
    BOOT_BANK_B
} BootBank;

typedef struct {
    BootBank active_bank;
    uint32_t version_a;
    uint32_t version_b;
    uint8_t update_in_progress;
    uint8_t boot_attempts;
} BootConfig;

void ota_update(uint8_t* new_firmware, uint32_t size, uint8_t* signature) {
    BootConfig cfg;
    load_boot_config(&cfg);

    // Determine inactive bank
    uint32_t inactive_bank_addr = (cfg.active_bank == BOOT_BANK_A) ?
                                  BANK_B_ADDR : BANK_A_ADDR;

    // Write new firmware to inactive bank
    flash_erase(inactive_bank_addr, BANK_SIZE);
    flash_write(inactive_bank_addr, new_firmware, size);

    // Verify signature
    if (!verify_firmware_signature(inactive_bank_addr, size, signature)) {
        return;  // Invalid update
    }

    // Mark update pending
    cfg.update_in_progress = 1;
    if (cfg.active_bank == BOOT_BANK_A) {
        cfg.version_b = get_firmware_version(new_firmware);
    } else {
        cfg.version_a = get_firmware_version(new_firmware);
    }
    save_boot_config(&cfg);

    // Reboot to bootloader
    NVIC_SystemReset();
}

void bootloader_select_bank() {
    BootConfig cfg;
    load_boot_config(&cfg);

    if (cfg.update_in_progress) {
        // Try new firmware
        BootBank new_bank = (cfg.active_bank == BOOT_BANK_A) ?
                           BOOT_BANK_B : BOOT_BANK_A;

        cfg.active_bank = new_bank;
        cfg.boot_attempts = 0;
        save_boot_config(&cfg);
    }

    // Boot from active bank
    uint32_t boot_addr = (cfg.active_bank == BOOT_BANK_A) ?
                         BANK_A_ADDR : BANK_B_ADDR;
    jump_to_application(boot_addr);
}

void application_mark_update_successful() {
    BootConfig cfg;
    load_boot_config(&cfg);

    cfg.update_in_progress = 0;
    cfg.boot_attempts = 0;
    save_boot_config(&cfg);
}
```

**Rollback on Failure:**

```cpp
void application_watchdog_init() {
    // If new firmware hangs or crashes, watchdog resets device

    BootConfig cfg;
    load_boot_config(&cfg);

    if (cfg.update_in_progress) {
        cfg.boot_attempts++;

        if (cfg.boot_attempts >= 3) {
            // Rollback to previous firmware
            cfg.active_bank = (cfg.active_bank == BOOT_BANK_A) ?
                             BOOT_BANK_B : BOOT_BANK_A;
            cfg.update_in_progress = 0;
            cfg.boot_attempts = 0;
            save_boot_config(&cfg);

            NVIC_SystemReset();
        } else {
            save_boot_config(&cfg);
        }
    }

    // Start watchdog
    watchdog_start(30000);  // 30s timeout
}
```

### Delta Updates

**Reduce download size for bandwidth/power-constrained sensors:**

```cpp
// Generate delta (on server/gateway)
void generate_delta(uint8_t* old_fw, uint32_t old_size,
                   uint8_t* new_fw, uint32_t new_size,
                   uint8_t* delta, uint32_t* delta_size) {
    // Use bsdiff or similar binary diff algorithm
    bsdiff(old_fw, old_size, new_fw, new_size, delta, delta_size);

    // Typical compression: 10-30% of full firmware size
}

// Apply delta (on device)
void apply_delta_update(uint8_t* delta, uint32_t delta_size) {
    // Read current firmware from flash
    uint8_t* old_fw = (uint8_t*)BANK_A_ADDR;

    // Apply patch to generate new firmware
    uint8_t new_fw[BANK_SIZE];
    bspatch(old_fw, FIRMWARE_SIZE, delta, delta_size, new_fw);

    // Write to inactive bank and activate (as before)
    ota_update(new_fw, FIRMWARE_SIZE, signature);
}
```

**Savings Example:**
- Full firmware: 256 KB
- Delta update: 25 KB (10×  smaller!)
- LoRaWAN transmission time: 4 hours → 25 minutes

---

## Security Standards and Certifications

### PSA Certified (Platform Security Architecture)

**Levels:**
- **PSA Certified Level 1**: Basic security (chip design review)
- **PSA Certified Level 2**: Advanced security (chip testing)
- **PSA Certified Level 3**: Substantial security (product certification)

**Requirements:**
- Secure boot
- Secure storage
- Cryptographic services
- Secure firmware update
- Attestation

### Common Criteria (CC EAL)

**Evaluation Assurance Levels (EAL 1-7):**
- EAL 1: Functionally tested
- EAL 4: Methodically designed, tested, reviewed (common for smart cards)
- EAL 7: Formally verified design and tested (highest)

**For sensors: EAL 2-4 typical**

### FIPS 140-2/3

**Federal Information Processing Standards for cryptographic modules:**

**Security Levels:**
- Level 1: Basic requirements
- Level 2: Physical tamper evidence
- Level 3: Tamper detection and response
- Level 4: Highest security (environmental protection)

**Applicable to cryptographic processors in high-security sensors**

### IEC 62443

**Industrial automation and control systems security:**
- **SL 1**: Protection against casual or coincidental violation
- **SL 2**: Protection against intentional violation using simple means
- **SL 3**: Protection against intentional violation using sophisticated means
- **SL 4**: Protection against intentional violation using sophisticated means with extended resources

**Target: SL 2-3 for industrial smart sensors**

---

## Best Practices Summary

**Hardware:**
✓ Enable secure boot
✓ Use hardware root of trust (TrustZone, TEE)
✓ Store keys in OTP or secure element
✓ Implement tamper detection
✓ Use hardware crypto accelerators (constant-time)

**Communication:**
✓ Use TLS 1.3 or DTLS
✓ Enforce mutual authentication
✓ Use modern cipher suites (AES-GCM, ChaCha20-Poly1305)
✓ Implement certificate pinning

**Software:**
✓ Minimize attack surface (disable unused features)
✓ Use memory-safe programming (bounds checking)
✓ Regular security audits and penetration testing
✓ Secure update mechanism with rollback

**Privacy:**
✓ Data minimization (transmit only necessary info)
✓ On-device processing where possible
✓ Anonymization/pseudonymization
✓ Differential privacy for aggregate data
✓ Comply with regulations (GDPR, CCPA, HIPAA)

---

## Review Questions

1. **Secure Boot Process**: Describe the three-stage secure boot verification chain (ROM → Stage 1 → Stage 2 → Application). If ECDSA signature verification takes 50 ms and SHA-256 hashing takes 10 ms for 256 KB firmware, calculate total boot time. What happens if verification fails at Stage 2?

2. **ARM TrustZone Isolation**: Explain how TrustZone separates Secure and Non-Secure worlds on Cortex-M33. In the provided example, AES decryption happens in the secure world using a key stored in secure SRAM. Why can't the application (non-secure) directly access this key? What's the overhead of secure function calls?

3. **DTLS Power Impact**: DTLS handshake consumes 100-500 ms @ 50-200 mA, while ongoing encryption adds 10-20% power overhead. For a sensor transmitting 10 bytes every 10 minutes, calculate the amortized power impact over a 24-hour period. Why are long-lived connections critical?

4. **BLE Security Modes**: Compare BLE Security Mode 1 Levels 1-4. For a medical sensor requiring HIPAA compliance, which level would you mandate (Level 3 or 4)? Explain the difference between authenticated pairing and LE Secure Connections (Level 4).

5. **Differential Privacy Trade-off**: The Laplace mechanism adds noise proportional to sensitivity/epsilon. For room occupancy counting with epsilon=0.1, calculate noise scale (sensitivity=1). If actual count is 5, what's the probability the noisy count is within ±2? Explain the privacy-utility trade-off.

6. **OTA Dual-Bank Update**: Describe the rollback mechanism for failed firmware updates. Given 3-boot-attempt threshold before rollback, calculate the maximum time to detect failure if watchdog timeout is 30s. Why is atomic update (dual-bank) preferred over in-place update?

7. **Delta Update Efficiency**: Full firmware is 256 KB, delta update is 25 KB (10× smaller). For LoRaWAN SF7 @ 5.5 kbps, calculate transmission time for full vs. delta. Given 1% duty cycle limit (36s per hour), can you complete a full update in one hour? What about delta?

## Key Takeaways

- **Secure Boot Foundation**: Three-stage verification chain (ROM → bootloader stages → application) with **ECDSA signature + SHA-256 hash** ensures only authentic firmware executes. Public key stored in **OTP/ROM** (immutable), private key in offline **HSM**, preventing unauthorized firmware injection.

- **ARM TrustZone Isolation**: Cortex-M33/M55 TrustZone provides **hardware-enforced separation** between Secure and Non-Secure worlds, enabling crypto keys to reside in secure SRAM inaccessible to application code even if compromised, with **minimal performance overhead** (<5%).

- **TLS 1.3 for Embedded**: Lightweight libraries (mbedTLS 70-200 KB, WolfSSL 20-100 KB, TinyDTLS <40 KB) enable **forward secrecy** and **mandatory encryption**. DTLS handshake (100-500 ms @ 50-200 mA) amortized over long-lived connections adds **10-20% ongoing power** overhead.

- **BLE Security Levels**: LE Security Mode 1, Level 4 provides **authenticated LE Secure Connections with AES-128-CCM**, requiring numeric comparison or out-of-band pairing. Medical/financial sensors must use **Level 3 or 4** to prevent man-in-the-middle attacks.

- **Privacy-Preserving Design**: Data minimization (transmit step count vs. raw accelerometer = **1000× data reduction**), on-device ML processing, differential privacy (Laplace noise with epsilon=0.1), and hourly timestamp truncation enable **aggregate trend analysis** while protecting individual privacy per GDPR/CCPA.

- **OTA Dual-Bank Architecture**: Two flash banks enable **atomic updates** (write to inactive bank, verify signature, swap) with **automatic rollback** after 3 failed boot attempts. Delta updates reduce **256 KB → 25 KB (10×)**, enabling LoRaWAN OTA in **25 minutes vs. 4 hours** for full firmware.

- **PSA Certified Framework**: Platform Security Architecture defines **3 levels** (chip design review → chip testing → product certification) requiring secure boot, secure storage, cryptographic services, secure firmware update, and attestation, providing industry-standard security baseline for IoT devices.

---

**Next Chapter**: Testing, validation, and compliance for smart sensor systems.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
