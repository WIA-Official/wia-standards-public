# WIA-AI-008 PHASE 2: API & Conversion Algorithms

**Standard ID**: WIA-AI-008-P2
**Version**: 1.0.0
**Status**: Active
**Last Updated**: 2025-01-15

## 弘益人間 - Benefit All Humanity

---

## 1. Overview

Phase 2 defines APIs and algorithms for cross-framework model conversion, optimization, and validation. This phase ensures models can be seamlessly transformed between different frameworks while maintaining numerical accuracy and performance characteristics.

## 2. Conversion API Specification

### 2.1 Standard Conversion Interface

All converters MUST implement the following interface:

```python
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional
from pathlib import Path

class ModelConverter(ABC):
    """Base class for model converters"""

    @abstractmethod
    def convert(
        self,
        source_path: Path,
        target_path: Path,
        **kwargs
    ) -> ConversionResult:
        """
        Convert model from source to target format

        Args:
            source_path: Path to source model
            target_path: Path for converted model
            **kwargs: Additional conversion parameters

        Returns:
            ConversionResult with status and metadata
        """
        pass

    @abstractmethod
    def validate(
        self,
        source_path: Path,
        target_path: Path,
        test_inputs: Optional[list] = None,
        tolerance: float = 1e-5
    ) -> ValidationResult:
        """
        Validate conversion accuracy

        Args:
            source_path: Original model
            target_path: Converted model
            test_inputs: Test data for validation
            tolerance: Numerical tolerance

        Returns:
            ValidationResult with metrics
        """
        pass

class ConversionResult:
    """Result of model conversion"""
    success: bool
    target_path: Path
    metadata: Dict[str, Any]
    warnings: list[str]
    errors: list[str]

class ValidationResult:
    """Result of conversion validation"""
    passed: bool
    max_difference: float
    mean_difference: float
    test_cases: int
    failures: list[str]
```

### 2.2 PyTorch to ONNX Converter

```python
import torch
import onnx
from pathlib import Path

class PyTorchToONNXConverter(ModelConverter):
    """Convert PyTorch models to ONNX format"""

    def convert(
        self,
        source_path: Path,
        target_path: Path,
        input_shape: tuple,
        opset_version: int = 15,
        dynamic_axes: Optional[dict] = None,
        **kwargs
    ) -> ConversionResult:
        """
        Convert PyTorch model to ONNX

        Args:
            source_path: Path to .pth or .pt file
            target_path: Path for output .onnx file
            input_shape: Model input shape (e.g., (1, 3, 224, 224))
            opset_version: ONNX opset version
            dynamic_axes: Dynamic dimensions specification

        Returns:
            ConversionResult
        """
        try:
            # Load PyTorch model
            model = torch.load(source_path)
            model.eval()

            # Create dummy input
            dummy_input = torch.randn(*input_shape)

            # Export to ONNX
            torch.onnx.export(
                model,
                dummy_input,
                str(target_path),
                export_params=True,
                opset_version=opset_version,
                do_constant_folding=True,
                input_names=['input'],
                output_names=['output'],
                dynamic_axes=dynamic_axes or {'input': {0: 'batch'}, 'output': {0: 'batch'}}
            )

            # Verify ONNX model
            onnx_model = onnx.load(str(target_path))
            onnx.checker.check_model(onnx_model)

            return ConversionResult(
                success=True,
                target_path=target_path,
                metadata={
                    'opset_version': opset_version,
                    'input_shape': input_shape,
                    'dynamic_axes': dynamic_axes
                },
                warnings=[],
                errors=[]
            )

        except Exception as e:
            return ConversionResult(
                success=False,
                target_path=None,
                metadata={},
                warnings=[],
                errors=[str(e)]
            )
```

### 2.3 TensorFlow to ONNX Converter

```python
import tensorflow as tf
import tf2onnx

class TensorFlowToONNXConverter(ModelConverter):
    """Convert TensorFlow models to ONNX format"""

    def convert(
        self,
        source_path: Path,
        target_path: Path,
        opset_version: int = 15,
        **kwargs
    ) -> ConversionResult:
        """
        Convert TensorFlow SavedModel to ONNX

        Args:
            source_path: Path to SavedModel directory
            target_path: Path for output .onnx file
            opset_version: ONNX opset version

        Returns:
            ConversionResult
        """
        try:
            # Load TensorFlow model
            model = tf.saved_model.load(str(source_path))

            # Convert to ONNX
            spec = (tf.TensorSpec((None, 224, 224, 3), tf.float32, name="input"),)
            model_proto, _ = tf2onnx.convert.from_saved_model(
                str(source_path),
                input_signature=spec,
                opset=opset_version
            )

            # Save ONNX model
            with open(target_path, "wb") as f:
                f.write(model_proto.SerializeToString())

            return ConversionResult(
                success=True,
                target_path=target_path,
                metadata={'opset_version': opset_version},
                warnings=[],
                errors=[]
            )

        except Exception as e:
            return ConversionResult(
                success=False,
                target_path=None,
                metadata={},
                warnings=[],
                errors=[str(e)]
            )
```

## 3. Optimization Algorithms

### 3.1 Quantization API

```python
from enum import Enum

class QuantizationMode(Enum):
    DYNAMIC = "dynamic"
    STATIC = "static"
    QAT = "quantization_aware_training"

class Quantizer(ABC):
    """Base class for model quantization"""

    @abstractmethod
    def quantize(
        self,
        model_path: Path,
        output_path: Path,
        mode: QuantizationMode,
        calibration_data: Optional[Any] = None,
        **kwargs
    ) -> QuantizationResult:
        """
        Quantize model to lower precision

        Args:
            model_path: Path to input model
            output_path: Path for quantized model
            mode: Quantization mode
            calibration_data: Data for calibration (static quant)

        Returns:
            QuantizationResult
        """
        pass

class QuantizationResult:
    success: bool
    output_path: Path
    original_size_mb: float
    quantized_size_mb: float
    compression_ratio: float
    accuracy_before: Optional[float]
    accuracy_after: Optional[float]
```

### 3.2 PyTorch Quantization

```python
import torch.quantization as quant

class PyTorchQuantizer(Quantizer):
    """PyTorch model quantization"""

    def quantize(
        self,
        model_path: Path,
        output_path: Path,
        mode: QuantizationMode,
        calibration_data: Optional[Any] = None,
        **kwargs
    ) -> QuantizationResult:
        """Quantize PyTorch model"""

        model = torch.load(model_path)
        model.eval()

        if mode == QuantizationMode.DYNAMIC:
            # Dynamic quantization
            quantized_model = quant.quantize_dynamic(
                model,
                {torch.nn.Linear, torch.nn.Conv2d},
                dtype=torch.qint8
            )

        elif mode == QuantizationMode.STATIC:
            # Static quantization
            model.qconfig = quant.get_default_qconfig('fbgemm')
            model_prepared = quant.prepare(model)

            # Calibrate
            with torch.no_grad():
                for data in calibration_data:
                    model_prepared(data)

            quantized_model = quant.convert(model_prepared)

        # Save quantized model
        torch.save(quantized_model, output_path)

        # Calculate metrics
        original_size = model_path.stat().st_size / (1024 ** 2)
        quantized_size = output_path.stat().st_size / (1024 ** 2)

        return QuantizationResult(
            success=True,
            output_path=output_path,
            original_size_mb=original_size,
            quantized_size_mb=quantized_size,
            compression_ratio=original_size / quantized_size,
            accuracy_before=None,
            accuracy_after=None
        )
```

## 4. Validation Framework

### 4.1 Numerical Validation

```python
import numpy as np

class NumericalValidator:
    """Validate numerical accuracy of conversions"""

    @staticmethod
    def validate_outputs(
        output1: np.ndarray,
        output2: np.ndarray,
        rtol: float = 1e-5,
        atol: float = 1e-8
    ) -> Dict[str, Any]:
        """
        Compare two model outputs

        Args:
            output1: First output
            output2: Second output
            rtol: Relative tolerance
            atol: Absolute tolerance

        Returns:
            Validation metrics
        """
        # Calculate differences
        abs_diff = np.abs(output1 - output2)
        rel_diff = abs_diff / (np.abs(output1) + 1e-8)

        max_abs_diff = np.max(abs_diff)
        mean_abs_diff = np.mean(abs_diff)
        max_rel_diff = np.max(rel_diff)
        mean_rel_diff = np.mean(rel_diff)

        # Check if within tolerance
        passed = np.allclose(output1, output2, rtol=rtol, atol=atol)

        return {
            'passed': passed,
            'max_absolute_difference': float(max_abs_diff),
            'mean_absolute_difference': float(mean_abs_diff),
            'max_relative_difference': float(max_rel_diff),
            'mean_relative_difference': float(mean_rel_diff),
            'tolerance': {'rtol': rtol, 'atol': atol}
        }
```

### 4.2 Performance Benchmarking

```python
import time
from typing import Callable

class PerformanceBenchmark:
    """Benchmark model inference performance"""

    @staticmethod
    def benchmark(
        inference_fn: Callable,
        input_data: Any,
        num_iterations: int = 100,
        warmup: int = 10
    ) -> Dict[str, float]:
        """
        Benchmark inference performance

        Args:
            inference_fn: Function that runs inference
            input_data: Input data
            num_iterations: Number of benchmark iterations
            warmup: Number of warmup iterations

        Returns:
            Performance metrics
        """
        # Warmup
        for _ in range(warmup):
            _ = inference_fn(input_data)

        # Benchmark
        start_time = time.time()
        for _ in range(num_iterations):
            _ = inference_fn(input_data)
        end_time = time.time()

        total_time = end_time - start_time
        avg_latency_ms = (total_time / num_iterations) * 1000
        throughput_qps = num_iterations / total_time

        return {
            'avg_latency_ms': avg_latency_ms,
            'throughput_qps': throughput_qps,
            'total_time_s': total_time,
            'iterations': num_iterations
        }
```

## 5. Graph Optimization

### 5.1 Operator Fusion

```python
class GraphOptimizer:
    """Optimize computational graphs"""

    @staticmethod
    def fuse_operators(
        model_path: Path,
        output_path: Path,
        fusion_patterns: list[str]
    ) -> OptimizationResult:
        """
        Fuse operators in computational graph

        Supported patterns:
        - 'conv_bn_relu': Conv + BatchNorm + ReLU
        - 'linear_relu': Linear + ReLU
        - 'attention': Multi-head attention fusion

        Args:
            model_path: Input model
            output_path: Optimized model
            fusion_patterns: List of patterns to fuse

        Returns:
            OptimizationResult
        """
        pass
```

## 6. Error Handling

### 6.1 Standard Exceptions

```python
class ModelConversionError(Exception):
    """Base exception for conversion errors"""
    pass

class UnsupportedOperatorError(ModelConversionError):
    """Raised when unsupported operator encountered"""
    pass

class ValidationError(ModelConversionError):
    """Raised when validation fails"""
    pass

class OptimizationError(ModelConversionError):
    """Raised during optimization"""
    pass
```

## 7. CLI Interface

### 7.1 Command-Line Tool

```bash
# Convert models
wia-convert --source model.pth --target model.onnx --format onnx

# Quantize models
wia-quantize --input model.onnx --output model_int8.onnx --mode static --calibration-data data.npz

# Validate conversion
wia-validate --original model.pth --converted model.onnx --tolerance 1e-5

# Benchmark performance
wia-benchmark --model model.onnx --input-shape 1,3,224,224 --iterations 100
```

## 8. Compliance Requirements

Models and tools following Phase 2 MUST:
- Implement standard conversion interfaces
- Provide numerical validation
- Support ONNX as intermediate format
- Document all conversions in model card
- Include conversion metadata in output

---

**弘益人間 - Benefit All Humanity**

© 2025 WIA (World Certification Industry Association)
Licensed under Apache 2.0
