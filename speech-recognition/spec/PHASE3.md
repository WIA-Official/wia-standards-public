# WIA-AI-022 PHASE 3: Intelligence & Understanding

## Overview

PHASE 3 adds intelligent features beyond basic transcription including intent recognition, entity extraction, sentiment analysis, and semantic understanding. These capabilities serve 弘益人間 by enabling deeper human-machine interaction.

**Status**: 🚧 In Development
**Version**: 0.9.0
**Prerequisites**: PHASE 1 & 2 complete

## Intelligent Features

### 3.1 Intent Recognition

**REQUIRED** intent classification:

```typescript
interface IntentRecognitionConfig {
  enabled: boolean;
  domains: string[];  // e.g., ['home_automation', 'calendar', 'search']
  confidenceThreshold: number;
}

interface Intent {
  name: string;
  confidence: number;
  domain: string;
  slots: Record<string, any>;
}

interface IntentResult {
  text: string;
  intents: Intent[];
  topIntent: Intent;
}
```

**Example**:
```typescript
Input:  "Set an alarm for 7 AM tomorrow"
Output: {
  intent: {
    name: "SetAlarm",
    domain: "clock",
    confidence: 0.95,
    slots: {
      time: "07:00",
      date: "2025-01-XX",
      recurring: false
    }
  }
}
```

### 3.2 Named Entity Recognition (NER)

**REQUIRED** entity extraction:

```typescript
interface NERConfig {
  entities: EntityType[];
  customEntities?: CustomEntity[];
}

type EntityType =
  | 'PERSON' | 'ORGANIZATION' | 'LOCATION'
  | 'DATE' | 'TIME' | 'MONEY'
  | 'PHONE' | 'EMAIL' | 'URL';

interface Entity {
  type: EntityType;
  text: string;
  start: number;
  end: number;
  confidence: number;
  normalized?: any;
}

interface CustomEntity {
  name: string;
  patterns: string[];
  examples: string[];
}
```

**Example**:
```typescript
Input:  "Call John Smith at 555-1234 tomorrow at 3 PM"
Output: {
  entities: [
    { type: "PERSON", text: "John Smith", start: 5, end: 15, confidence: 0.92 },
    { type: "PHONE", text: "555-1234", start: 19, end: 27, confidence: 0.98 },
    { type: "TIME", text: "3 PM", start: 41, end: 45, confidence: 0.95 }
  ]
}
```

### 3.3 Sentiment Analysis

**REQUIRED** sentiment detection:

```typescript
interface SentimentConfig {
  enabled: boolean;
  granularity: 'document' | 'sentence' | 'aspect';
}

interface Sentiment {
  label: 'positive' | 'negative' | 'neutral' | 'mixed';
  score: number;  // -1.0 (very negative) to +1.0 (very positive)
  confidence: number;
}

interface SentimentResult {
  overall: Sentiment;
  sentences?: Sentiment[];
  aspects?: Array<{
    aspect: string;
    sentiment: Sentiment;
  }>;
}
```

**Example**:
```typescript
Input:  "I love this product but the price is too high"
Output: {
  overall: { label: "mixed", score: 0.2, confidence: 0.85 },
  sentences: [
    { label: "positive", score: 0.8, confidence: 0.90 },
    { label: "negative", score: -0.6, confidence: 0.87 }
  ],
  aspects: [
    { aspect: "product", sentiment: { label: "positive", score: 0.9 }},
    { aspect: "price", sentiment: { label: "negative", score: -0.7 }}
  ]
}
```

### 3.4 Semantic Similarity

**REQUIRED** semantic understanding:

```typescript
interface SemanticConfig {
  embedding: 'sentence-bert' | 'universal-sentence-encoder';
  similarityThreshold: number;
}

interface SemanticResult {
  embedding: number[];
  similar: Array<{
    text: string;
    similarity: number;
  }>;
}
```

**Use Cases**:
- Duplicate detection
- Question answering
- Content recommendation
- Search query understanding

### 3.5 Keyword Extraction

**REQUIRED** key phrase extraction:

```typescript
interface KeywordConfig {
  method: 'tfidf' | 'textrank' | 'rake';
  maxKeywords: number;
  minNgram: number;
  maxNgram: number;
}

interface Keyword {
  phrase: string;
  score: number;
  positions: number[];
}
```

### 3.6 Topic Classification

**REQUIRED** topic identification:

```typescript
interface TopicConfig {
  taxonomy?: 'custom' | 'iab' | 'wikipedia';
  maxTopics: number;
  hierarchical: boolean;
}

interface Topic {
  id: string;
  name: string;
  confidence: number;
  parent?: string;
}
```

**Example Taxonomy**:
```
Technology
├── Artificial Intelligence
│   ├── Machine Learning
│   └── Natural Language Processing
├── Hardware
└── Software
```

### 3.7 Summarization

**REQUIRED** automatic summarization:

```typescript
interface SummarizationConfig {
  method: 'extractive' | 'abstractive';
  maxLength: number;
  ratio?: number;  // 0.0 to 1.0 for extractive
}

interface Summary {
  text: string;
  sentences: string[];
  keywords: Keyword[];
}
```

### 3.8 Question Answering

**OPTIONAL** but RECOMMENDED:

```typescript
interface QAConfig {
  context?: string;
  maxAnswerLength: number;
}

interface Answer {
  text: string;
  confidence: number;
  startPosition: number;
  endPosition: number;
  context?: string;
}
```

## Integration Examples

### Complete Intelligence Pipeline

```python
from wia_speech import ASREngine, PHASE3Config

# Configure intelligence features
config = PHASE3Config(
    # Intent recognition
    intent=IntentRecognitionConfig(
        enabled=True,
        domains=['home_automation', 'calendar', 'communication']
    ),

    # Named entity recognition
    ner=NERConfig(
        entities=['PERSON', 'DATE', 'TIME', 'LOCATION', 'ORGANIZATION']
    ),

    # Sentiment analysis
    sentiment=SentimentConfig(
        enabled=True,
        granularity='sentence'
    ),

    # Semantic understanding
    semantic=SemanticConfig(
        embedding='sentence-bert',
        similarityThreshold=0.75
    ),

    # Topic classification
    topics=TopicConfig(
        taxonomy='custom',
        maxTopics=3,
        hierarchical=True
    )
)

asr = ASREngine(config)

# Process audio with intelligence
result = asr.transcribe_file('voice_command.wav')

print(f"Text: {result.text}")
print(f"Intent: {result.intent.name}")
print(f"Entities: {[e.text for e in result.entities]}")
print(f"Sentiment: {result.sentiment.label}")
print(f"Topics: {[t.name for t in result.topics]}")

# Take action based on intent
if result.intent.name == 'SetReminder':
    time = result.intent.slots['time']
    message = result.intent.slots['message']
    set_reminder(time, message)
```

### Multi-turn Dialogue

```python
class DialogueManager:
    """Manage multi-turn conversations"""

    def __init__(self, asr: ASREngine):
        self.asr = asr
        self.context = []
        self.state = {}

    async def process_turn(self, audio):
        """Process one dialogue turn"""
        # Transcribe with context
        result = await self.asr.transcribe_buffer(
            audio,
            context=self.context
        )

        # Extract intent
        intent = result.intent

        # Update state
        self.state.update(intent.slots)

        # Generate response
        response = self.generate_response(intent, self.state)

        # Update context
        self.context.append({
            'user': result.text,
            'assistant': response,
            'intent': intent.name
        })

        return response

    def generate_response(self, intent, state):
        """Generate contextual response"""
        # Implementation specific to application
        pass
```

## Performance Requirements

### PHASE 3 Benchmarks

| Feature | Metric | Target |
|---------|--------|--------|
| Intent Recognition | F1 Score | >90% |
| NER | F1 Score | >85% |
| Sentiment Analysis | Accuracy | >80% |
| Topic Classification | Accuracy | >85% |
| Keyword Extraction | F1 Score | >75% |
| Semantic Similarity | Correlation | >0.80 |
| Summarization (ROUGE-L) | F1 Score | >40% |

### Latency Requirements

- Intent recognition: <50ms
- NER: <100ms
- Sentiment: <50ms
- Complete pipeline: <300ms

## Testing & Validation

```python
class PHASE3Validator:
    """Validate PHASE 3 intelligence features"""

    def validate_intent_recognition(self, engine: ASREngine) -> bool:
        """Test intent classification"""
        test_cases = [
            ("Set alarm for 7 AM", "SetAlarm"),
            ("What's the weather today", "GetWeather"),
            ("Call mom", "MakeCall")
        ]

        correct = 0
        for text, expected_intent in test_cases:
            result = engine.recognize_intent(text)
            if result.intent.name == expected_intent:
                correct += 1

        accuracy = correct / len(test_cases)
        return accuracy > 0.85

    def validate_ner(self, engine: ASREngine) -> bool:
        """Test named entity recognition"""
        text = "Call John Smith at 555-1234 tomorrow"
        result = engine.extract_entities(text)

        # Check extracted entities
        entity_types = [e.type for e in result.entities]

        required = ['PERSON', 'PHONE', 'DATE']
        return all(et in entity_types for et in required)

    def validate_sentiment(self, engine: ASREngine) -> bool:
        """Test sentiment analysis"""
        test_cases = [
            ("I love this!", "positive"),
            ("This is terrible", "negative"),
            ("It's okay", "neutral")
        ]

        correct = 0
        for text, expected_sentiment in test_cases:
            result = engine.analyze_sentiment(text)
            if result.sentiment.label == expected_sentiment:
                correct += 1

        accuracy = correct / len(test_cases)
        return accuracy > 0.80
```

## Summary

PHASE 3 adds intelligence:

✅ Intent recognition (F1 >90%)
✅ Named Entity Recognition (F1 >85%)
✅ Sentiment analysis (Accuracy >80%)
✅ Semantic similarity
✅ Keyword extraction
✅ Topic classification
✅ Automatic summarization
🔄 Question answering (optional)

**Next**: [PHASE 4 - Ecosystem Integration](PHASE4.md)

---

**弘益人間 (Hongik Ingan)** - *Benefit All Humanity*

PHASE 3 enables intelligent understanding beyond transcription, creating more natural and useful human-machine interaction for all.
