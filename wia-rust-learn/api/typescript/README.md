# WIA-RUST-LEARN TypeScript SDK

TypeScript SDK for the WIA Rust Learning Platform - Interactive Rust tutorials, code execution, and progress tracking.

## Installation

```bash
npm install @wia/wia-rust-learn
# or
yarn add @wia/wia-rust-learn
```

## Quick Start

```typescript
import { createRustLearnSDK } from '@wia/wia-rust-learn';

// Initialize the SDK
const sdk = createRustLearnSDK({
  userId: 'user_123',
  apiKey: 'your_api_key',
});

// Get learning paths
const paths = await sdk.getLearningPaths();

// Start a lesson
const lesson = await sdk.startLesson('ownership-basics');

// Run Rust code
const result = await sdk.runCode('fn main() { println!("Hello!"); }');

// Submit an exercise
const submission = await sdk.submitExercise('ex_001', userCode);

// Track progress
await sdk.trackProgress('lesson_1', 75);
```

## Features

- **Learning Paths**: Access structured Rust curricula
- **Interactive Lessons**: Learn Rust concepts with examples
- **Code Execution**: Run Rust code in a safe sandbox
- **Exercises & Challenges**: Practice with hands-on coding problems
- **Progress Tracking**: Monitor learning progress and achievements
- **Quiz System**: Test understanding with quizzes
- **Playground**: Interactive Rust code playground
- **Event System**: Subscribe to SDK events

## API Reference

### Core Methods

#### `getLearningPaths(filters?)`
Get all available learning paths with optional filtering.

#### `startLesson(lessonId)`
Start a lesson and retrieve its content.

#### `runCode(code, options?)`
Execute Rust code with optional compiler options.

#### `submitExercise(exerciseId, code)`
Submit an exercise solution for validation.

#### `checkSolution(code, expectedOutput)`
Check if a solution produces expected output.

#### `getHint(exerciseId, hintLevel?)`
Get a hint for an exercise.

#### `trackProgress(lessonId, progressPercent)`
Update progress for a lesson.

#### `getUserProgress()`
Get user's overall progress.

#### `getAchievements()`
Retrieve earned achievements.

#### `takeQuiz(quizId, answers)`
Submit quiz answers and get results.

#### `createPlayground(code?, options?)`
Create a new playground session.

#### `sharePlayground(sessionId)`
Generate a share URL for a playground.

## Event System

```typescript
sdk.on('code:success', (result) => {
  console.log('Code executed:', result);
});

sdk.on('exercise:complete', (data) => {
  console.log('Exercise completed:', data);
});

sdk.on('progress:update', (progress) => {
  console.log('Progress updated:', progress);
});
```

## Types

All TypeScript types are exported from the package:

```typescript
import type {
  LearningPath,
  Lesson,
  Exercise,
  CompilerOptions,
  ExecutionResult,
  UserProgress,
  Achievement,
} from '@wia/wia-rust-learn';
```

## Development

```bash
# Install dependencies
npm install

# Build the SDK
npm run build

# Run tests
npm test

# Type check
npm run typecheck

# Lint
npm run lint
```

## License

MIT © WIA Education Research Group

## Contributing

Contributions are welcome! Please read our contributing guidelines before submitting PRs.

---

弘益人間 (Benefit All Humanity)
