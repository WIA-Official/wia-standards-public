# WIA-AI-011 Phase 2: API Abstraction Layer

**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-01-15

---

## Overview

Phase 2 defines hardware-agnostic APIs for AI accelerator operations, enabling portable code while preserving access to hardware-specific optimizations.

## Core APIs

### Device Management

```c
// Device enumeration
wia_status_t wia_enumerate_devices(wia_device_t** devices, int* count);
wia_status_t wia_get_device_info(wia_device_t device, wia_device_info_t* info);
wia_status_t wia_query_capabilities(wia_device_t device, wia_capability_t* caps);

// Device selection
wia_status_t wia_set_current_device(wia_device_t device);
wia_status_t wia_get_current_device(wia_device_t* device);
```

### Context Management

```c
// Context creation/destruction
typedef struct {
    wia_device_t device;
    size_t memory_pool_size;
    int max_streams;
    bool profiling_enabled;
} wia_context_config_t;

wia_status_t wia_create_context(wia_context_config_t* config, wia_context_t* ctx);
wia_status_t wia_destroy_context(wia_context_t ctx);
wia_status_t wia_context_synchronize(wia_context_t ctx);
```

### Stream Operations

```c
// Stream management
wia_status_t wia_create_stream(wia_context_t ctx, wia_stream_type_t type, wia_stream_t* stream);
wia_status_t wia_destroy_stream(wia_stream_t stream);
wia_status_t wia_stream_synchronize(wia_stream_t stream);
wia_status_t wia_stream_query(wia_stream_t stream, bool* is_idle);

// Event synchronization
wia_status_t wia_create_event(wia_event_t* event);
wia_status_t wia_record_event(wia_event_t event, wia_stream_t stream);
wia_status_t wia_stream_wait_event(wia_stream_t stream, wia_event_t event);
```

### Memory Operations

```c
// Memory allocation
typedef struct {
    size_t size;
    wia_memory_type_t type; // DEVICE, PINNED, UNIFIED, MANAGED
    size_t alignment;
    wia_memory_flags_t flags;
    wia_device_t device_affinity;
} wia_memory_desc_t;

wia_status_t wia_allocate_memory(wia_context_t ctx, wia_memory_desc_t* desc, wia_buffer_t* buffer);
wia_status_t wia_free_memory(wia_buffer_t buffer);

// Data transfer
wia_status_t wia_memcpy(void* dst, const void* src, size_t size);
wia_status_t wia_memcpy_async(void* dst, const void* src, size_t size, wia_stream_t stream);
wia_status_t wia_memset(void* ptr, int value, size_t size);
```

### Tensor Operations

```c
// Tensor allocation
wia_status_t wia_allocate_tensor(wia_context_t ctx, wia_tensor_desc_t* desc, wia_tensor_t* tensor);

// Matrix operations
wia_status_t wia_matmul(wia_context_t ctx, wia_tensor_t A, wia_tensor_t B, wia_tensor_t C,
                        wia_matmul_config_t* config, wia_stream_t stream);

// Convolution
wia_status_t wia_conv2d(wia_context_t ctx, wia_tensor_t input, wia_tensor_t weight,
                        wia_tensor_t bias, wia_tensor_t output,
                        wia_conv2d_config_t* config, wia_stream_t stream);

// Activation functions
wia_status_t wia_relu(wia_context_t ctx, wia_tensor_t input, wia_tensor_t output, wia_stream_t stream);
wia_status_t wia_gelu(wia_context_t ctx, wia_tensor_t input, wia_tensor_t output, wia_stream_t stream);
wia_status_t wia_softmax(wia_context_t ctx, wia_tensor_t input, wia_tensor_t output,
                         wia_softmax_config_t* config, wia_stream_t stream);
```

### Error Handling

```c
typedef enum {
    WIA_SUCCESS = 0,
    WIA_ERROR_INVALID_ARGUMENT,
    WIA_ERROR_OUT_OF_MEMORY,
    WIA_ERROR_DEVICE_NOT_FOUND,
    WIA_ERROR_NOT_SUPPORTED,
    WIA_ERROR_RUNTIME_ERROR
} wia_status_t;

const char* wia_get_error_string(wia_status_t status);
wia_status_t wia_get_last_error(wia_context_t ctx, wia_error_info_t* info);
```

## Capability System

```c
typedef struct {
    // Compute capabilities
    float peak_tflops_fp32;
    float peak_tflops_fp16;
    float peak_tflops_int8;
    
    // Memory capabilities
    size_t total_memory;
    size_t memory_bandwidth_gbps;
    bool supports_unified_memory;
    bool supports_pinned_memory;
    
    // Feature support
    bool supports_fp16;
    bool supports_bfloat16;
    bool supports_int8;
    bool supports_sparse_ops;
    bool supports_dynamic_shapes;
    
    // Limits
    int max_tensor_dims;
    size_t max_tensor_size;
    int max_batch_size;
} wia_capability_t;
```

## Profiling

```c
typedef struct {
    bool profiling_enabled;
    wia_profile_mode_t mode; // SAMPLING, INSTRUMENTATION, HARDWARE_COUNTERS
} wia_profiling_config_t;

wia_status_t wia_start_profiling(wia_context_t ctx, wia_profiling_config_t* config, const char* output_file);
wia_status_t wia_stop_profiling(wia_context_t ctx);
wia_status_t wia_get_profile_results(wia_context_t ctx, wia_profile_results_t* results);
```

## Compliance Levels

### Level 1: Core
- Device enumeration
- Basic context management
- Memory allocation (device memory)
- MatMul, Conv2D
- FLOAT32 support

### Level 2: Extended
- Stream management
- Event synchronization
- Multiple memory types
- Full tensor operation library
- FP16, INT8 support

### Level 3: Advanced
- Multi-device contexts
- Profiling APIs
- Dynamic shapes
- All operations and data types

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 WIA (World Certification Industry Association)
