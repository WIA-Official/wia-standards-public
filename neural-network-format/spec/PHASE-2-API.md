# WIA-AI-014: Neural Network Format - Phase 2 API Specification

**Version:** 1.0.0
**Status:** Draft
**Philosophy:** 弘益人間 - Benefit All Humanity

## 1. Overview

This specification defines the programming interfaces for loading, saving, and manipulating WIA-AI-014 compliant neural network models.

## 2. Core API

### 2.1 Model I/O Operations

```typescript
interface WIAModel {
    // Load model from file
    static load(path: string, options?: LoadOptions): Promise<WIAModel>;

    // Save model to file
    save(path: string, options?: SaveOptions): Promise<void>;

    // Run inference
    run(inputs: TensorMap, options?: InferenceOptions): Promise<TensorMap>;

    // Get model metadata
    getMetadata(): ModelMetadata;

    // Get input/output specifications
    getInputs(): ValueInfo[];
    getOutputs(): ValueInfo[];
}
```

### 2.2 Tensor Operations

```typescript
interface Tensor {
    name: string;
    shape: number[];
    dtype: DataType;
    data: TypedArray | Buffer;

    // Create tensor from array
    static from(data: number[], shape: number[], dtype?: DataType): Tensor;

    // Get tensor value at index
    get(...indices: number[]): number;

    // Set tensor value at index
    set(value: number, ...indices: number[]): void;

    // Reshape tensor
    reshape(shape: number[]): Tensor;

    // Convert to different dtype
    astype(dtype: DataType): Tensor;
}
```

### 2.3 Graph Manipulation

```typescript
interface Graph {
    name: string;
    nodes: Node[];
    inputs: ValueInfo[];
    outputs: ValueInfo[];

    // Add node to graph
    addNode(node: Node): void;

    // Remove node from graph
    removeNode(name: string): void;

    // Get node by name
    getNode(name: string): Node | null;

    // Optimize graph
    optimize(level: OptimizationLevel): Graph;

    // Validate graph structure
    validate(): ValidationResult;
}
```

## 3. Conversion API

### 3.1 Framework Conversion

```typescript
class ModelConverter {
    // Convert from PyTorch
    static fromPyTorch(
        modelPath: string,
        inputShapes: Record<string, number[]>
    ): Promise<WIAModel>;

    // Convert from TensorFlow
    static fromTensorFlow(savedModelPath: string): Promise<WIAModel>;

    // Convert from ONNX
    static fromONNX(onnxPath: string): Promise<WIAModel>;

    // Convert to ONNX
    static toONNX(model: WIAModel, outputPath: string): Promise<void>;

    // Convert to TFLite
    static toTFLite(
        model: WIAModel,
        options?: TFLiteOptions
    ): Promise<Buffer>;
}
```

### 3.2 Optimization API

```typescript
class ModelOptimizer {
    // Apply quantization
    static quantize(
        model: WIAModel,
        config: QuantizationConfig
    ): WIAModel;

    // Apply pruning
    static prune(model: WIAModel, sparsity: number): WIAModel;

    // Operator fusion
    static fuseOperators(model: WIAModel): WIAModel;

    // Constant folding
    static foldConstants(model: WIAModel): WIAModel;
}
```

## 4. Runtime API

### 4.1 Inference Session

```typescript
interface InferenceSession {
    // Create session
    static create(
        model: WIAModel,
        options?: SessionOptions
    ): InferenceSession;

    // Run inference
    run(inputs: TensorMap): Promise<TensorMap>;

    // Get profiling info
    getProfilingData(): ProfilingData;

    // Release resources
    dispose(): void;
}

interface SessionOptions {
    executionProviders?: string[];  // ['cpu', 'cuda', 'tensorrt']
    graphOptimizationLevel?: OptimizationLevel;
    enableProfiling?: boolean;
    numThreads?: number;
}
```

### 4.2 Execution Providers

```typescript
interface ExecutionProvider {
    name: string;
    deviceType: 'cpu' | 'gpu' | 'npu' | 'tpu';

    // Check if provider is available
    isAvailable(): boolean;

    // Get device information
    getDeviceInfo(): DeviceInfo;

    // Create kernel for operation
    createKernel(node: Node): Kernel;
}
```

## 5. Metadata API

### 5.1 Model Registry

```typescript
class ModelRegistry {
    // Register model
    register(name: string, version: string, model: WIAModel): void;

    // Get model by name and version
    get(name: string, version?: string): WIAModel;

    // List all models
    list(): ModelInfo[];

    // Delete model
    delete(name: string, version: string): void;

    // Get model metadata
    getMetadata(name: string, version: string): ModelMetadata;
}
```

### 5.2 Versioning API

```typescript
interface ModelVersion {
    major: number;
    minor: number;
    patch: number;

    toString(): string;
    compareTo(other: ModelVersion): number;
}

class VersionManager {
    // Parse version string
    static parse(version: string): ModelVersion;

    // Get latest version
    static getLatest(versions: ModelVersion[]): ModelVersion;

    // Check compatibility
    static isCompatible(v1: ModelVersion, v2: ModelVersion): boolean;
}
```

## 6. Validation API

### 6.1 Model Validation

```typescript
class ModelValidator {
    // Validate model structure
    static validateStructure(model: WIAModel): ValidationResult;

    // Validate operator support
    static validateOperators(
        model: WIAModel,
        targetPlatform: string
    ): ValidationResult;

    // Validate numerical accuracy
    static compareOutputs(
        model1: WIAModel,
        model2: WIAModel,
        testInputs: TensorMap[]
    ): ComparisonResult;
}

interface ValidationResult {
    valid: boolean;
    errors: ValidationError[];
    warnings: ValidationWarning[];
}
```

## 7. Utility API

### 7.1 Model Analysis

```typescript
class ModelAnalyzer {
    // Get model statistics
    static getStats(model: WIAModel): ModelStats;

    // Estimate memory usage
    static estimateMemory(model: WIAModel): MemoryEstimate;

    // Profile inference performance
    static profileInference(
        model: WIAModel,
        inputs: TensorMap
    ): PerformanceMetrics;
}

interface ModelStats {
    numParameters: number;
    numLayers: number;
    modelSizeMB: number;
    flops: number;
}
```

## 8. Error Handling

### 8.1 Exception Types

```typescript
class WIAError extends Error {
    code: string;
    details?: any;
}

class ModelFormatError extends WIAError {}
class ConversionError extends WIAError {}
class InferenceError extends WIAError {}
class ValidationError extends WIAError {}
```

## 9. Example Usage

### 9.1 Load and Run Model

```typescript
// Load model
const model = await WIAModel.load('model.wia');

// Create inference session
const session = InferenceSession.create(model, {
    executionProviders: ['cuda', 'cpu'],
    graphOptimizationLevel: OptimizationLevel.All
});

// Prepare input
const input = Tensor.from(
    [1, 2, 3, 4],
    [1, 4],
    DataType.Float32
);

// Run inference
const outputs = await session.run({ input });

console.log(outputs.output.data);

// Cleanup
session.dispose();
```

### 9.2 Convert and Optimize

```typescript
// Convert from PyTorch
const model = await ModelConverter.fromPyTorch(
    'model.pt',
    { input: [1, 3, 224, 224] }
);

// Optimize
const optimized = ModelOptimizer.quantize(model, {
    type: 'int8',
    calibrationData: calibrationSet
});

// Save
await optimized.save('optimized_model.wia');
```

---

**Copyright © 2025 WIA**
**弘益人間 · Benefit All Humanity**
