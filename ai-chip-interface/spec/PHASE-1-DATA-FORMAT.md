# WIA-AI-011 Phase 1: Data Format Standardization

**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-01-15

---

## Overview

Phase 1 establishes foundational data format standards for AI chip interfaces, ensuring tensor data can be seamlessly shared across NPU, TPU, and GPU accelerators without conversion overhead or numerical degradation.

## Goals

1. Define unified tensor descriptor format
2. Standardize memory layouts (NCHW, NHWC, etc.)
3. Specify data type system (FP32, FP16, INT8, etc.)
4. Enable zero-copy data sharing
5. Support quantization metadata

## Tensor Descriptor Specification

### Core Structure

```json
{
  "version": "WIA-AI-011-v1.0",
  "tensor_id": "UUID",
  "name": "string",
  "shape": {
    "dimensions": [int, ...],
    "layout": "NCHW | NHWC | CHWN",
    "symbolic": boolean
  },
  "dtype": {
    "base_type": "FLOAT32 | FLOAT16 | BFLOAT16 | INT8 | INT32 | ...",
    "quantization": null | QuantizationParams,
    "byte_order": "little_endian | big_endian"
  },
  "memory": {
    "strides": [int, ...],
    "offset_bytes": int,
    "total_bytes": int,
    "alignment": int,
    "device": DeviceDescriptor
  },
  "properties": {
    "requires_grad": boolean,
    "is_pinned": boolean,
    "is_contiguous": boolean,
    "zero_copy_compatible": boolean
  },
  "metadata": {}
}
```

### Data Types

#### Floating Point
- **FLOAT64**: IEEE 754 double precision (64-bit)
- **FLOAT32**: IEEE 754 single precision (32-bit) - default for training
- **FLOAT16**: IEEE 754 half precision (16-bit)
- **BFLOAT16**: Brain floating point (16-bit, 8-bit exponent)
- **TF32**: TensorFloat-32 (19-bit, NVIDIA)
- **FP8_E4M3**: 8-bit float (4-bit exp, 3-bit mantissa)
- **FP8_E5M2**: 8-bit float (5-bit exp, 2-bit mantissa)

#### Integer
- **INT64**, **INT32**, **INT16**, **INT8**: Signed integers
- **UINT64**, **UINT32**, **UINT16**, **UINT8**: Unsigned integers
- **INT4**, **UINT4**: 4-bit integers (sub-byte)

### Memory Layouts

#### NCHW (Batch, Channel, Height, Width)
- Preferred by: NVIDIA cuDNN, PyTorch (GPU)
- Best for: Convolution operations
- Memory order: N → C → H → W

#### NHWC (Batch, Height, Width, Channel)
- Preferred by: TensorFlow, ARM, Mobile NPUs
- Best for: Cache locality in certain operations
- Memory order: N → H → W → C

#### Stride Representation
Strides define element spacing for each dimension:
```
element_offset = base_offset + Σ(index[i] * stride[i])
```

### Quantization

#### Symmetric Per-Tensor
```json
{
  "scheme": "per_tensor_symmetric",
  "scale": float,
  "zero_point": 0,
  "range": [int_min, int_max]
}
```

#### Asymmetric Per-Channel
```json
{
  "scheme": "per_channel_asymmetric",
  "scales": [float, ...],
  "zero_points": [int, ...],
  "channel_axis": int
}
```

## Memory Alignment

### Requirements
- **Minimum alignment**: 64 bytes
- **Recommended alignment**: 256 bytes
- **Device-specific**: Query via capability API

### Padding
Padding may be applied to dimensions for optimal performance:
```json
{
  "shape": [32, 11, 224, 224],
  "physical_shape": [32, 16, 224, 224],
  "padding": {
    "dims": [false, true, false, false],
    "values": [0, 5, 0, 0],
    "mode": "zero_fill"
  }
}
```

## Zero-Copy Protocol

### Shared Buffer Handle
```json
{
  "buffer_handle": {
    "type": "dma_buf_fd | cuda_ipc | metal_shared",
    "handle": int,
    "size_bytes": int,
    "permissions": "read_only | read_write"
  },
  "mapping": {
    "device_address": "0xaddress",
    "cpu_address": null | "0xaddress",
    "access_mode": "device_only | cpu_device"
  }
}
```

### Pinned Memory
- Page-locked memory for DMA transfers
- Stable physical addresses
- Flags: `WIA_MEM_PINNED`

## Sparse Tensor Support

### COO (Coordinate) Format
```json
{
  "format": "COO",
  "shape": [int, int],
  "nnz": int,
  "indices": {
    "dtype": "INT32 | INT64",
    "data": [[row, col], ...]
  },
  "values": {
    "dtype": "FLOAT32 | ...",
    "data": [float, ...]
  }
}
```

### CSR (Compressed Sparse Row)
```json
{
  "format": "CSR",
  "shape": [int, int],
  "row_pointers": [int, ...],
  "column_indices": [int, ...],
  "values": [float, ...]
}
```

## Dynamic Shapes

```json
{
  "shape": {
    "dimensions": ["batch", "sequence_length", 768],
    "symbolic": true,
    "constraints": {
      "batch": {"min": 1, "max": 256},
      "sequence_length": {"min": 1, "max": 2048, "multiple_of": 8}
    }
  }
}
```

## Compliance Requirements

### Level 1: Basic
- Support FLOAT32, FLOAT16
- Support NCHW or NHWC (at least one)
- Minimum 64-byte alignment

### Level 2: Standard
- Support FLOAT32, FLOAT16, BFLOAT16, INT8
- Support both NCHW and NHWC
- Stride-based layouts
- Quantization metadata

### Level 3: Advanced
- All data types
- All layouts
- Sparse tensors
- Zero-copy sharing
- Dynamic shapes

---

## Reference Implementation

See `api/typescript/` for reference implementation.

## Testing

Compliance tests available at: [WIA-AI-011-tests](https://github.com/WIA-Official/wia-ai-011-tests)

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 WIA (World Certification Industry Association)
