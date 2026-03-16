# WIA-SEC-013: Secure Enclave - Appendix

**Version:** 1.0.0
**Last Updated:** 2025-12-25

---

## Appendix A: Code Examples

### A.1 Complete Intel SGX Application

```c
// enclave.edl - Enclave Definition Language
enclave {
    trusted {
        public int ecall_process_secret(
            [in, size=len] const uint8_t* data,
            size_t len,
            [out, size=32] uint8_t* result
        );

        public int ecall_seal_data(
            [in, size=len] const uint8_t* plaintext,
            size_t len,
            [out, size=sealed_size] uint8_t* sealed_data,
            size_t sealed_size
        );

        public int ecall_unseal_data(
            [in, size=sealed_len] const uint8_t* sealed_data,
            size_t sealed_len,
            [out, size=max_len] uint8_t* plaintext,
            size_t max_len
        );
    };

    untrusted {
        void ocall_print([in, string] const char* str);
        int ocall_get_time([out] uint64_t* timestamp);
    };
};
```

```c
// enclave.c - Enclave implementation
#include <sgx_tseal.h>
#include <sgx_tcrypto.h>
#include "enclave_t.h"

int ecall_process_secret(
    const uint8_t* data,
    size_t len,
    uint8_t* result
) {
    // Process sensitive data inside enclave
    sgx_sha256_hash_t hash;
    sgx_status_t status = sgx_sha256_msg(data, len, &hash);

    if (status != SGX_SUCCESS) {
        return -1;
    }

    memcpy(result, &hash, 32);
    return 0;
}

int ecall_seal_data(
    const uint8_t* plaintext,
    size_t len,
    uint8_t* sealed_data,
    size_t sealed_size
) {
    sgx_status_t status = sgx_seal_data(
        0,                    // Additional AAD length
        NULL,                 // Additional AAD
        len,                  // Plaintext length
        plaintext,            // Plaintext data
        sealed_size,          // Sealed blob size
        (sgx_sealed_data_t*)sealed_data
    );

    return (status == SGX_SUCCESS) ? 0 : -1;
}

int ecall_unseal_data(
    const uint8_t* sealed_data,
    size_t sealed_len,
    uint8_t* plaintext,
    size_t max_len
) {
    uint32_t plaintext_len = max_len;
    sgx_status_t status = sgx_unseal_data(
        (const sgx_sealed_data_t*)sealed_data,
        NULL,                 // AAD output
        NULL,                 // AAD length
        plaintext,            // Plaintext output
        &plaintext_len        // Plaintext length
    );

    return (status == SGX_SUCCESS) ? plaintext_len : -1;
}
```

```c
// app.c - Untrusted application
#include <stdio.h>
#include "enclave_u.h"

sgx_enclave_id_t global_eid = 0;

int initialize_enclave(void) {
    sgx_launch_token_t token = {0};
    int updated = 0;

    sgx_status_t ret = sgx_create_enclave(
        "enclave.signed.so",
        SGX_DEBUG_FLAG,
        &token,
        &updated,
        &global_eid,
        NULL
    );

    return (ret == SGX_SUCCESS) ? 0 : -1;
}

int main() {
    // Initialize enclave
    if (initialize_enclave() < 0) {
        printf("Failed to initialize enclave\n");
        return -1;
    }

    // Call enclave function
    uint8_t data[] = "Secret data";
    uint8_t result[32];

    sgx_status_t ret = ecall_process_secret(
        global_eid,
        data,
        sizeof(data),
        result
    );

    if (ret != SGX_SUCCESS) {
        printf("ECALL failed\n");
        return -1;
    }

    printf("Result: ");
    for (int i = 0; i < 32; i++) {
        printf("%02x", result[i]);
    }
    printf("\n");

    // Destroy enclave
    sgx_destroy_enclave(global_eid);
    return 0;
}
```

### A.2 ARM TrustZone Trusted Application

```c
// ta/ta.c - Trusted Application
#include <tee_internal_api.h>
#include <tee_internal_api_extensions.h>

#define TA_UUID \
    { 0x8aaaf200, 0x2450, 0x11e4, \
      { 0xab, 0xe2, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b } }

TEE_Result TA_CreateEntryPoint(void) {
    return TEE_SUCCESS;
}

void TA_DestroyEntryPoint(void) {
}

TEE_Result TA_OpenSessionEntryPoint(
    uint32_t param_types,
    TEE_Param params[4],
    void **sess_ctx
) {
    (void)param_types;
    (void)params;
    (void)sess_ctx;
    return TEE_SUCCESS;
}

void TA_CloseSessionEntryPoint(void *sess_ctx) {
    (void)sess_ctx;
}

// Command IDs
#define CMD_ENCRYPT_DATA  1
#define CMD_DECRYPT_DATA  2
#define CMD_SEAL_DATA     3
#define CMD_UNSEAL_DATA   4

TEE_Result TA_InvokeCommandEntryPoint(
    void *sess_ctx,
    uint32_t cmd_id,
    uint32_t param_types,
    TEE_Param params[4]
) {
    (void)sess_ctx;

    switch (cmd_id) {
    case CMD_ENCRYPT_DATA:
        return encrypt_data(param_types, params);
    case CMD_DECRYPT_DATA:
        return decrypt_data(param_types, params);
    case CMD_SEAL_DATA:
        return seal_data(param_types, params);
    case CMD_UNSEAL_DATA:
        return unseal_data(param_types, params);
    default:
        return TEE_ERROR_BAD_PARAMETERS;
    }
}

static TEE_Result encrypt_data(
    uint32_t param_types,
    TEE_Param params[4]
) {
    TEE_OperationHandle op = TEE_HANDLE_NULL;
    TEE_ObjectHandle key = TEE_HANDLE_NULL;
    TEE_Result res;

    // Params: [0] = input, [1] = output, [2] = key
    void *input = params[0].memref.buffer;
    size_t input_len = params[0].memref.size;
    void *output = params[1].memref.buffer;
    size_t *output_len = &params[1].memref.size;

    // Allocate AES operation
    res = TEE_AllocateOperation(
        &op,
        TEE_ALG_AES_GCM,
        TEE_MODE_ENCRYPT,
        256
    );
    if (res != TEE_SUCCESS)
        goto out;

    // Generate random key
    res = TEE_AllocateTransientObject(
        TEE_TYPE_AES,
        256,
        &key
    );
    if (res != TEE_SUCCESS)
        goto out;

    res = TEE_GenerateKey(key, 256, NULL, 0);
    if (res != TEE_SUCCESS)
        goto out;

    res = TEE_SetOperationKey(op, key);
    if (res != TEE_SUCCESS)
        goto out;

    // Encrypt
    uint8_t iv[16];
    TEE_GenerateRandom(iv, sizeof(iv));

    TEE_AEInit(op, iv, sizeof(iv), 128, 0, 0);
    res = TEE_AEEncryptFinal(
        op,
        input,
        input_len,
        output,
        output_len
    );

out:
    if (op != TEE_HANDLE_NULL)
        TEE_FreeOperation(op);
    if (key != TEE_HANDLE_NULL)
        TEE_FreeTransientObject(key);
    return res;
}
```

### A.3 Cross-Platform TEE Library (Rust)

```rust
// lib.rs
use std::fmt;

/// Platform-agnostic TEE abstraction
pub trait TEEPlatform {
    type EnclaveHandle;
    type Error: fmt::Display;

    fn create_enclave(
        &self,
        config: &EnclaveConfig,
    ) -> Result<Self::EnclaveHandle, Self::Error>;

    fn destroy_enclave(
        &self,
        handle: Self::EnclaveHandle,
    ) -> Result<(), Self::Error>;

    fn call_enclave(
        &self,
        handle: &Self::EnclaveHandle,
        function: &str,
        args: &[u8],
    ) -> Result<Vec<u8>, Self::Error>;

    fn get_attestation(
        &self,
        handle: &Self::EnclaveHandle,
        data: &[u8],
    ) -> Result<AttestationQuote, Self::Error>;
}

#[derive(Debug, Clone)]
pub struct EnclaveConfig {
    pub debug: bool,
    pub heap_size: usize,
    pub stack_size: usize,
}

#[derive(Debug, Clone)]
pub struct AttestationQuote {
    pub platform: String,
    pub version: u16,
    pub quote: Vec<u8>,
    pub signature: Vec<u8>,
}

// Intel SGX implementation
#[cfg(feature = "sgx")]
pub mod sgx {
    use super::*;
    use sgx_types::*;
    use sgx_urts::SgxEnclave;

    pub struct SGXPlatform;

    impl TEEPlatform for SGXPlatform {
        type EnclaveHandle = SgxEnclave;
        type Error = sgx_status_t;

        fn create_enclave(
            &self,
            config: &EnclaveConfig,
        ) -> Result<Self::EnclaveHandle, Self::Error> {
            let mut launch_token = [0; 1024];
            let mut updated = 0;
            let debug = if config.debug { 1 } else { 0 };

            let enclave = SgxEnclave::create(
                "enclave.signed.so",
                debug,
                &mut launch_token,
                &mut updated,
            )?;

            Ok(enclave)
        }

        fn destroy_enclave(
            &self,
            handle: Self::EnclaveHandle,
        ) -> Result<(), Self::Error> {
            handle.destroy();
            Ok(())
        }

        fn call_enclave(
            &self,
            handle: &Self::EnclaveHandle,
            function: &str,
            args: &[u8],
        ) -> Result<Vec<u8>, Self::Error> {
            // Implementation-specific ECALL dispatch
            unimplemented!()
        }

        fn get_attestation(
            &self,
            handle: &Self::EnclaveHandle,
            data: &[u8],
        ) -> Result<AttestationQuote, Self::Error> {
            // Generate SGX quote
            unimplemented!()
        }
    }
}

// ARM TrustZone implementation
#[cfg(feature = "trustzone")]
pub mod trustzone {
    use super::*;

    pub struct TrustZonePlatform;

    impl TEEPlatform for TrustZonePlatform {
        type EnclaveHandle = TrustZoneSession;
        type Error = TrustZoneError;

        // Implementation...
    }
}
```

---

## Appendix B: Configuration Examples

### B.1 SGX Enclave Configuration (XML)

```xml
<?xml version="1.0" encoding="UTF-8"?>
<EnclaveConfiguration>
  <ProdID>1</ProdID>
  <ISVSVN>1</ISVSVN>

  <StackMaxSize>0x40000</StackMaxSize>     <!-- 256 KB -->
  <HeapMaxSize>0x100000</HeapMaxSize>      <!-- 1 MB -->

  <TCSNum>10</TCSNum>
  <TCSPolicy>1</TCSPolicy>

  <DisableDebug>0</DisableDebug>

  <MiscSelect>0</MiscSelect>
  <MiscMask>0xFFFFFFFF</MiscMask>

  <EnableKSS>0</EnableKSS>

  <ISVEXTPRODID_H>0</ISVEXTPRODID_H>
  <ISVEXTPRODID_L>0</ISVEXTPRODID_L>

  <ISVFAMILYID_H>0</ISVFAMILYID_H>
  <ISVFAMILYID_L>0</ISVFAMILYID_L>
</EnclaveConfiguration>
```

### B.2 TrustZone TA Manifest

```json
{
  "uuid": "8aaaf200-2450-11e4-abe2-0002a5d5c51b",
  "name": "Secure Storage TA",
  "description": "Trusted Application for secure data storage",
  "version": "1.0.0",

  "entry_points": {
    "create": "TA_CreateEntryPoint",
    "destroy": "TA_DestroyEntryPoint",
    "open_session": "TA_OpenSessionEntryPoint",
    "close_session": "TA_CloseSessionEntryPoint",
    "invoke_command": "TA_InvokeCommandEntryPoint"
  },

  "stack_size": 65536,
  "heap_size": 131072,

  "single_instance": false,
  "multi_session": true,
  "instance_keep_alive": false,

  "flags": [
    "TA_FLAG_USER_MODE",
    "TA_FLAG_EXEC_DDR"
  ],

  "properties": {
    "gpd.ta.appID": "8aaaf200-2450-11e4-abe2-0002a5d5c51b",
    "gpd.ta.singleInstance": false,
    "gpd.ta.multiSession": true,
    "gpd.ta.instanceKeepAlive": false,
    "gpd.ta.dataSize": 131072,
    "gpd.ta.stackSize": 65536
  }
}
```

---

## Appendix C: Testing Frameworks

### C.1 SGX Unit Tests

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_seal_unseal() {
        let plaintext = b"Secret message";

        // Seal data
        let sealed = seal_data(plaintext, KeyPolicy::MRENCLAVE)
            .expect("Failed to seal");

        // Unseal data
        let unsealed = unseal_data(&sealed)
            .expect("Failed to unseal");

        assert_eq!(plaintext.as_ref(), unsealed.as_slice());
    }

    #[test]
    fn test_attestation_generation() {
        let report_data = [0u8; 64];

        let quote = generate_attestation_quote(
            &report_data,
        ).expect("Failed to generate quote");

        assert!(!quote.is_empty());
    }

    #[test]
    fn test_remote_attestation() {
        let quote = generate_test_quote();

        let result = verify_attestation_quote(&quote)
            .expect("Verification failed");

        assert!(result.is_valid);
    }
}
```

### C.2 TrustZone Integration Tests

```c
// test_ta.c
#include <stdio.h>
#include <tee_client_api.h>

#define TA_UUID \
    { 0x8aaaf200, 0x2450, 0x11e4, \
      { 0xab, 0xe2, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b } }

void test_encrypt_decrypt(void) {
    TEEC_Context ctx;
    TEEC_Session sess;
    TEEC_Operation op;
    TEEC_Result res;
    TEEC_UUID uuid = TA_UUID;

    // Initialize context
    res = TEEC_InitializeContext(NULL, &ctx);
    assert(res == TEEC_SUCCESS);

    // Open session
    res = TEEC_OpenSession(&ctx, &sess, &uuid,
                          TEEC_LOGIN_PUBLIC, NULL, NULL, NULL);
    assert(res == TEEC_SUCCESS);

    // Prepare operation
    memset(&op, 0, sizeof(op));
    op.paramTypes = TEEC_PARAM_TYPES(
        TEEC_MEMREF_TEMP_INPUT,
        TEEC_MEMREF_TEMP_OUTPUT,
        TEEC_NONE,
        TEEC_NONE
    );

    char plaintext[] = "Hello, TrustZone!";
    char ciphertext[128];

    op.params[0].tmpref.buffer = plaintext;
    op.params[0].tmpref.size = sizeof(plaintext);
    op.params[1].tmpref.buffer = ciphertext;
    op.params[1].tmpref.size = sizeof(ciphertext);

    // Invoke command
    res = TEEC_InvokeCommand(&sess, CMD_ENCRYPT_DATA, &op, NULL);
    assert(res == TEEC_SUCCESS);

    printf("Encrypted successfully\n");

    // Cleanup
    TEEC_CloseSession(&sess);
    TEEC_FinalizeContext(&ctx);
}
```

---

## Appendix D: Deployment Guides

### D.1 Docker Container with SGX

```dockerfile
# Dockerfile
FROM ubuntu:20.04

# Install SGX dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    ocaml \
    ocamlbuild \
    automake \
    autoconf \
    libtool \
    wget \
    python \
    libssl-dev \
    libcurl4-openssl-dev \
    protobuf-compiler

# Install SGX SDK
RUN wget https://download.01.org/intel-sgx/sgx-linux/2.17/distro/ubuntu20.04-server/sgx_linux_x64_sdk_2.17.100.3.bin \
    && chmod +x sgx_linux_x64_sdk_2.17.100.3.bin \
    && ./sgx_linux_x64_sdk_2.17.100.3.bin --prefix=/opt/intel

# Source SGX environment
RUN echo "source /opt/intel/sgxsdk/environment" >> ~/.bashrc

WORKDIR /app
COPY . .

RUN make

CMD ["./app"]
```

```yaml
# docker-compose.yml
version: '3'
services:
  sgx-app:
    build: .
    devices:
      - /dev/isgx:/dev/isgx
      - /dev/sgx/enclave:/dev/sgx/enclave
      - /dev/sgx/provision:/dev/sgx/provision
    volumes:
      - /var/run/aesmd:/var/run/aesmd
    environment:
      - SGX_MODE=HW
```

### D.2 Kubernetes with Confidential Computing

```yaml
# sgx-daemonset.yaml
apiVersion: apps/v1
kind: DaemonSet
metadata:
  name: sgx-device-plugin
  namespace: kube-system
spec:
  selector:
    matchLabels:
      app: sgx-device-plugin
  template:
    metadata:
      labels:
        app: sgx-device-plugin
    spec:
      containers:
      - name: sgx-device-plugin
        image: intel/intel-device-plugins-for-kubernetes:sgx
        volumeMounts:
        - name: devfs
          mountPath: /dev
        - name: sysfs
          mountPath: /sys
      volumes:
      - name: devfs
        hostPath:
          path: /dev
      - name: sysfs
        hostPath:
          path: /sys
```

```yaml
# sgx-workload.yaml
apiVersion: v1
kind: Pod
metadata:
  name: sgx-workload
spec:
  containers:
  - name: app
    image: myapp:latest
    resources:
      limits:
        sgx.intel.com/epc: "10Mi"
        sgx.intel.com/enclave: 1
        sgx.intel.com/provision: 1
    volumeMounts:
    - name: aesmd
      mountPath: /var/run/aesmd
  volumes:
  - name: aesmd
    hostPath:
      path: /var/run/aesmd
```

---

## Appendix E: Performance Benchmarks

### E.1 Enclave Operation Latencies

| Operation | Intel SGX | ARM TrustZone | AWS Nitro |
|-----------|-----------|---------------|-----------|
| Enclave Entry (ECALL) | ~8,000 cycles | ~1,500 cycles | ~10,000 cycles |
| Enclave Exit (OCALL) | ~10,000 cycles | ~2,000 cycles | ~12,000 cycles |
| Memory Encryption | +5-15% overhead | +10-20% overhead | +8-12% overhead |
| Sealed Storage | ~50 μs | ~30 μs | ~60 μs |
| Attestation Generation | ~100 ms | ~50 ms | ~150 ms |

### E.2 Throughput Measurements

```
AES-128-GCM Encryption (MB/s):
- Native: 2,400 MB/s
- SGX Enclave: 2,100 MB/s (87%)
- TrustZone: 1,800 MB/s (75%)

SHA-256 Hashing (MB/s):
- Native: 850 MB/s
- SGX Enclave: 780 MB/s (92%)
- TrustZone: 720 MB/s (85%)
```

---

**Published by:**
World Certification Industry Association (WIA)

**License:** CC BY-SA 4.0

**弘益人間 · Benefit All Humanity**
