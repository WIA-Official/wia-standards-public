# @wia/wia-rust-intermediate

Intermediate Rust programming concepts, error handling, and trait patterns SDK

## Installation

```bash
npm install @wia/wia-rust-intermediate
```

## Features

- **Trait Analysis**: Analyze trait definitions, implementations, and coherence
- **Generic Validation**: Validate type parameters and where clauses
- **Pattern Matching**: Check pattern exhaustiveness and detect unreachable arms
- **Error Handling**: Analyze Result, Option, and custom error patterns
- **Smart Pointers**: Inspect Box, Rc, Arc, and RefCell usage
- **Borrow Checking**: Validate borrow checker rules
- **Code Linting**: Detect common Rust anti-patterns
- **Refactoring Suggestions**: Get recommendations for code improvements

## Quick Start

```typescript
import { createWIARustIntermediateSDK } from '@wia/wia-rust-intermediate';

// Create SDK instance
const sdk = createWIARustIntermediateSDK({
  edition: '2021',
  strictMode: true,
  lintLevel: 'warn'
});

// Analyze traits
const traitAnalysis = await sdk.analyzeTraits(rustCode);
console.log('Traits found:', traitAnalysis.traits);

// Check pattern matching
const patternResult = await sdk.checkPatternMatch(matchArms, 'Option<T>');
if (!patternResult.exhaustive) {
  console.log('Missing patterns:', patternResult.missingPatterns);
}

// Validate borrows
const borrowCheck = await sdk.validateBorrows(rustCode);
if (!borrowCheck.valid) {
  console.error('Borrow errors:', borrowCheck.errors);
}

// Lint code
const lintResults = await sdk.lintCode(rustCode);
console.log('Lint issues:', lintResults.stats);
```

## API Reference

### Core Methods

#### `analyzeTraits(code: string, options?: AnalysisOptions): Promise<TraitAnalysisResult>`

Analyzes trait definitions and implementations in Rust code.

```typescript
const result = await sdk.analyzeTraits(code);
// Returns: { traits, implementations, orphanRules, coherence }
```

#### `validateGenerics(genericType: GenericType): Promise<boolean>`

Validates generic type parameters and constraints.

```typescript
const isValid = await sdk.validateGenerics({
  name: 'MyType',
  parameters: [
    { name: 'T', bounds: ['Clone', 'Debug'] }
  ]
});
```

#### `checkPatternMatch(matchArms: MatchArm[], matchType: string): Promise<PatternMatchResult>`

Checks pattern matching for exhaustiveness.

```typescript
const result = await sdk.checkPatternMatch(arms, 'Result<T, E>');
if (!result.exhaustive) {
  console.log('Add these patterns:', result.missingPatterns);
}
```

#### `analyzeErrorHandling(code: string): Promise<ErrorAnalysis>`

Analyzes error handling patterns.

```typescript
const analysis = await sdk.analyzeErrorHandling(code);
console.log('Results:', analysis.results);
console.log('Options:', analysis.options);
console.log('Recommendations:', analysis.recommendations);
```

#### `inspectSmartPointers(code: string): Promise<SmartPointerAnalysis>`

Inspects smart pointer usage.

```typescript
const analysis = await sdk.inspectSmartPointers(code);
console.log(`Found ${analysis.boxes} Box, ${analysis.arcs} Arc`);
```

#### `validateBorrows(code: string): Promise<BorrowCheckResult>`

Validates borrow checker rules.

```typescript
const result = await sdk.validateBorrows(code);
if (!result.valid) {
  result.errors?.forEach(err => console.error(err.message));
}
```

#### `suggestRefactoring(code: string): Promise<string[]>`

Suggests code refactoring improvements.

```typescript
const suggestions = await sdk.suggestRefactoring(code);
suggestions.forEach(s => console.log('💡', s));
```

#### `lintCode(code: string, options?: AnalysisOptions): Promise<LintResult>`

Lints Rust code for common issues.

```typescript
const lint = await sdk.lintCode(code);
console.log(`Found ${lint.stats.errorCount} errors, ${lint.stats.warningCount} warnings`);
```

### Events

The SDK extends EventEmitter and emits the following events:

```typescript
sdk.on('initialized', (data) => {
  console.log('SDK initialized:', data.config);
});

sdk.on('analysis:started', (data) => {
  console.log('Analysis started:', data.type);
});

sdk.on('analysis:completed', (data) => {
  console.log('Analysis completed:', data.result);
});

sdk.on('analysis:error', (data) => {
  console.error('Analysis error:', data.error);
});

sdk.on('lint:completed', (result) => {
  console.log('Lint completed:', result.stats);
});
```

### Configuration

```typescript
interface RustIntermediateConfig {
  strictMode?: boolean;              // Default: true
  edition?: '2015' | '2018' | '2021' | '2024';  // Default: '2021'
  features?: string[];               // Default: []
  lintLevel?: 'forbid' | 'deny' | 'warn' | 'allow';  // Default: 'warn'
  optimizationLevel?: 0 | 1 | 2 | 3;  // Default: 2
}
```

### Type Definitions

The SDK provides comprehensive TypeScript types for Rust concepts:

- **Error Handling**: `Result<T, E>`, `Option<T>`, `CustomError`
- **Traits**: `TraitDefinition`, `TraitImpl`, `TraitBound`, `DynTrait`
- **Generics**: `GenericType`, `TypeParameter`, `WhereClause`
- **Collections**: `VecConfig`, `HashMapConfig`, `IteratorPattern`
- **Patterns**: `MatchArm`, `Pattern`, `DestructuringPattern`
- **Smart Pointers**: `BoxType`, `RcConfig`, `ArcConfig`, `RefCellConfig`

## Examples

### Trait Analysis

```typescript
const code = `
trait Drawable {
  fn draw(&self);
}

impl Drawable for Circle {
  fn draw(&self) {
    println!("Drawing circle");
  }
}
`;

const result = await sdk.analyzeTraits(code);
console.log('Traits:', result.traits);
console.log('Implementations:', result.implementations);
```

### Generic Validation

```typescript
const generic: GenericType = {
  name: 'Container',
  parameters: [
    { name: 'T', bounds: ['Clone', 'Debug'] }
  ],
  whereClause: [
    { type: 'T', constraint: 'Send + Sync' }
  ]
};

const isValid = await sdk.validateGenerics(generic);
```

### Pattern Matching

```typescript
const matchArms: MatchArm[] = [
  { pattern: { type: 'enum', variant: 'Some' }, body: 'value' },
  { pattern: { type: 'enum', variant: 'None' }, body: 'default' }
];

const result = await sdk.checkPatternMatch(matchArms, 'Option<T>');
console.log('Exhaustive:', result.exhaustive);
```

### Smart Pointer Inspection

```typescript
const code = `
let data = Box::new(vec![1, 2, 3]);
let shared = Arc::new(Mutex::new(data));
let counted = Rc::new(RefCell::new(42));
`;

const analysis = await sdk.inspectSmartPointers(code);
console.log(`Box: ${analysis.boxes}, Arc: ${analysis.arcs}, Rc: ${analysis.rcs}`);
```

## License

MIT

## Contributing

Contributions are welcome! Please see the [WIA Standards](https://github.com/WIA-Official/wia-standards) repository.

## About WIA

**WIA (World Certification Industry Association)**

弘益人間 (홍익인간) - Benefit All Humanity

© 2025 SmileStory Inc. / WIA
