# WIA-EDU-020: Content AI Standard
## Implementation Guide

**Version:** 1.0
**Date:** 2025-01-15

---

## 1. Getting Started

### 1.1 Prerequisites

**Technical Requirements:**
- Node.js 16+ or Python 3.9+
- API credentials (API key and/or OAuth tokens)
- Basic understanding of REST APIs
- (Optional) TypeScript for type safety

**Account Setup:**
1. Sign up at https://api.wia.org
2. Create an application
3. Generate API keys
4. Configure webhooks (optional)

### 1.2 Installation

**TypeScript/JavaScript:**
```bash
npm install @wia/content-ai
# or
yarn add @wia/content-ai
```

**Python:**
```bash
pip install wia-content-ai
```

---

## 2. Basic Implementation

### 2.1 Initialize the Client

**TypeScript:**
```typescript
import { ContentAI } from '@wia/content-ai';

const client = new ContentAI({
  apiKey: process.env.WIA_API_KEY,
  environment: 'production', // or 'sandbox'
  timeout: 30000,
  retries: 3
});
```

**Python:**
```python
from wia_content_ai import ContentAI

client = ContentAI(
    api_key=os.environ['WIA_API_KEY'],
    environment='production',
    timeout=30,
    retries=3
)
```

### 2.2 Generate Your First Lesson

```typescript
async function generateLesson() {
  try {
    const lesson = await client.generateLesson({
      subject: 'mathematics',
      topic: 'quadratic-equations',
      gradeLevel: '8th',
      objectives: [
        'Understand the standard form',
        'Solve using factoring',
        'Apply to real-world problems'
      ],
      duration: 45,
      includeAssessment: true
    });

    console.log('Lesson generated:', lesson.id);
    console.log('Title:', lesson.title);
    console.log('Sections:', lesson.sections.length);

    return lesson;
  } catch (error) {
    console.error('Generation failed:', error.message);
    throw error;
  }
}
```

---

## 3. Advanced Features

### 3.1 Content Personalization

```typescript
async function personalizeForStudent(
  contentId: string,
  studentId: string
) {
  // Get student profile
  const profile = await client.getStudentProfile(studentId);

  // Personalize content
  const personalized = await client.personalizeContent({
    contentId,
    learnerProfile: {
      readingLevel: profile.readingLevel,
      learningStyle: profile.learningStyle,
      pace: profile.pace,
      interests: profile.interests,
      language: profile.preferredLanguage
    },
    adaptations: {
      readingLevel: true,
      learningStyle: true,
      interests: true,
      pace: true
    }
  });

  return personalized;
}
```

### 3.2 Multi-Modal Content

```typescript
async function createMultiModalLesson(topic: string) {
  // Generate base lesson
  const lesson = await client.generateLesson({
    subject: 'science',
    topic,
    gradeLevel: '7th'
  });

  // Generate video explanation
  const video = await client.generateVideo({
    topic,
    script: lesson.sections[1].content,
    duration: 300,
    style: 'animated',
    voiceConfig: {
      gender: 'female',
      accent: 'en-US'
    },
    captions: true
  });

  // Generate interactive simulation
  const simulation = await client.generateInteractive({
    type: 'simulation',
    topic,
    learningObjectives: lesson.objectives,
    difficulty: 0.6
  });

  return {
    lesson,
    video,
    simulation
  };
}
```

### 3.3 Content Translation

```typescript
async function translateCourse(courseId: string) {
  const targetLanguages = ['es', 'fr', 'zh', 'ar', 'ko'];

  const translations = await client.translateContent({
    contentId: courseId,
    sourceLanguage: 'en',
    targetLanguages,
    culturalAdaptation: true,
    preserveFormatting: true
  });

  // Store translations
  for (const translation of translations.translations) {
    await saveTranslation(
      courseId,
      translation.language,
      translation.content
    );
  }

  return translations;
}
```

---

## 4. Integration Patterns

### 4.1 LMS Integration

**Canvas Integration:**
```typescript
import { CanvasAdapter } from '@wia/content-ai';

const canvas = new CanvasAdapter({
  canvasUrl: 'https://canvas.university.edu',
  apiKey: process.env.CANVAS_API_KEY
});

// Generate and publish to Canvas
async function publishToCanvas(courseId: string) {
  const lesson = await client.generateLesson({...});

  // Convert to Canvas module
  const module = await canvas.createModule({
    courseId,
    name: lesson.title,
    content: lesson
  });

  // Create assignments
  for (const assessment of lesson.assessments) {
    await canvas.createAssignment({
      courseId,
      moduleId: module.id,
      assessment
    });
  }

  return module;
}
```

**Moodle Integration:**
```typescript
import { MoodleAdapter } from '@wia/content-ai';

const moodle = new MoodleAdapter({
  moodleUrl: 'https://moodle.school.edu',
  token: process.env.MOODLE_TOKEN
});

async function publishToMoodle(courseId: string) {
  const lesson = await client.generateLesson({...});

  await moodle.createActivity({
    courseId,
    type: 'lesson',
    content: lesson
  });
}
```

### 4.2 Content Management System

**WordPress Integration:**
```typescript
import { WordPressAdapter } from '@wia/content-ai';

const wp = new WordPressAdapter({
  siteUrl: 'https://education.example.com',
  credentials: {
    username: process.env.WP_USER,
    password: process.env.WP_PASSWORD
  }
});

async function publishBlogPost(topic: string) {
  const content = await client.generateContent({
    type: 'article',
    topic,
    format: 'blog-post'
  });

  await wp.createPost({
    title: content.title,
    content: content.body,
    categories: ['education', topic],
    status: 'publish'
  });
}
```

### 4.3 Real-time Collaboration

```typescript
import { CollaborationService } from '@wia/content-ai';

const collab = new CollaborationService({
  apiKey: process.env.WIA_API_KEY
});

// Enable real-time editing
async function enableCollaboration(contentId: string) {
  const session = await collab.createSession({
    contentId,
    participants: ['user-1', 'user-2', 'user-3']
  });

  // Listen for updates
  session.on('update', (update) => {
    console.log('Content updated:', update);
  });

  // Make changes
  await session.update({
    section: 'introduction',
    content: 'Updated introduction text...'
  });
}
```

---

## 5. Best Practices

### 5.1 Error Handling

```typescript
async function robustContentGeneration(params: any) {
  try {
    const content = await client.generateLesson(params);
    return content;
  } catch (error) {
    if (error.code === 'RATE_LIMIT_EXCEEDED') {
      // Wait and retry
      await sleep(error.retryAfter * 1000);
      return robustContentGeneration(params);
    } else if (error.code === 'INVALID_REQUEST') {
      // Fix parameters and retry
      const fixedParams = validateAndFix(params);
      return client.generateLesson(fixedParams);
    } else {
      // Log and notify
      logger.error('Content generation failed', error);
      await notifyAdmins(error);
      throw error;
    }
  }
}
```

### 5.2 Caching Strategy

```typescript
import { Redis } from 'ioredis';

const redis = new Redis(process.env.REDIS_URL);

async function getCachedOrGenerate(params: any) {
  const cacheKey = generateCacheKey(params);

  // Check cache
  const cached = await redis.get(cacheKey);
  if (cached) {
    return JSON.parse(cached);
  }

  // Generate new content
  const content = await client.generateLesson(params);

  // Cache for 24 hours
  await redis.setex(
    cacheKey,
    86400,
    JSON.stringify(content)
  );

  return content;
}
```

### 5.3 Batch Processing

```typescript
async function batchGenerateProblems(
  topics: string[],
  count: number
) {
  const batches = chunk(topics, 10); // Process 10 at a time

  const results = [];
  for (const batch of batches) {
    const promises = batch.map(topic =>
      client.generateProblems({
        topic,
        count,
        difficulty: 0.6
      })
    );

    const batchResults = await Promise.all(promises);
    results.push(...batchResults);

    // Rate limiting: wait between batches
    await sleep(1000);
  }

  return results;
}
```

### 5.4 Quality Assurance

```typescript
async function generateWithQA(params: any) {
  // Generate content
  const content = await client.generateLesson(params);

  // Validate quality
  const validation = await client.validateContent({
    content,
    checks: [
      'grammar',
      'factual-accuracy',
      'bias',
      'accessibility',
      'standards-alignment'
    ]
  });

  // If quality is low, regenerate or fix
  if (validation.overallScore < 0.85) {
    if (validation.overallScore < 0.7) {
      // Quality too low, regenerate
      return generateWithQA(params);
    } else {
      // Apply fixes
      const fixed = await applyFixes(content, validation.issues);
      return fixed;
    }
  }

  return content;
}
```

---

## 6. Performance Optimization

### 6.1 Parallel Generation

```typescript
async function generateCompleteCourse(topics: string[]) {
  // Generate all lessons in parallel
  const lessons = await Promise.all(
    topics.map(topic =>
      client.generateLesson({
        subject: 'mathematics',
        topic,
        gradeLevel: '8th'
      })
    )
  );

  return lessons;
}
```

### 6.2 Streaming Responses

```typescript
async function streamLessonGeneration(params: any) {
  const stream = await client.generateLessonStream(params);

  stream.on('section', (section) => {
    console.log('Section generated:', section.title);
    // Update UI immediately
    updateUI(section);
  });

  stream.on('complete', (lesson) => {
    console.log('Lesson complete:', lesson.id);
  });

  stream.on('error', (error) => {
    console.error('Error:', error);
  });
}
```

---

## 7. Monitoring and Analytics

### 7.1 Usage Tracking

```typescript
import { Analytics } from '@wia/content-ai';

const analytics = new Analytics({
  apiKey: process.env.WIA_API_KEY
});

async function trackUsage() {
  const usage = await analytics.getUsage({
    timeRange: 'last-30-days',
    groupBy: 'day'
  });

  console.log('Total requests:', usage.totalRequests);
  console.log('Content generated:', usage.contentCount);
  console.log('Cost:', usage.estimatedCost);
}
```

### 7.2 Content Performance

```typescript
async function monitorContentPerformance(contentId: string) {
  const analytics = await client.getContentAnalytics({
    contentId,
    timeRange: 'last-7-days'
  });

  // Alert if performance drops
  if (analytics.engagement.completionRate < 0.5) {
    await sendAlert({
      type: 'low-completion',
      contentId,
      completionRate: analytics.engagement.completionRate
    });
  }

  // Log metrics
  logger.info('Content performance', {
    contentId,
    completionRate: analytics.engagement.completionRate,
    satisfaction: analytics.effectiveness.satisfactionScore
  });
}
```

---

## 8. Testing

### 8.1 Unit Tests

```typescript
import { ContentAI } from '@wia/content-ai';
import { jest } from '@jest/globals';

describe('ContentAI', () => {
  let client: ContentAI;

  beforeEach(() => {
    client = new ContentAI({
      apiKey: 'test-api-key',
      environment: 'sandbox'
    });
  });

  it('should generate a lesson', async () => {
    const lesson = await client.generateLesson({
      subject: 'mathematics',
      topic: 'addition',
      gradeLevel: '1st'
    });

    expect(lesson).toBeDefined();
    expect(lesson.title).toBeTruthy();
    expect(lesson.sections).toHaveLength(5);
  });

  it('should handle errors gracefully', async () => {
    await expect(
      client.generateLesson({
        subject: '',
        topic: '',
        gradeLevel: ''
      })
    ).rejects.toThrow('INVALID_REQUEST');
  });
});
```

### 8.2 Integration Tests

```typescript
describe('LMS Integration', () => {
  it('should publish to Canvas', async () => {
    const lesson = await client.generateLesson({...});
    const module = await canvas.createModule({...});

    expect(module.id).toBeDefined();
    expect(module.name).toBe(lesson.title);
  });
});
```

---

## 9. Deployment

### 9.1 Environment Configuration

```typescript
// config.ts
export const config = {
  development: {
    apiKey: process.env.WIA_DEV_API_KEY,
    environment: 'sandbox',
    logLevel: 'debug'
  },
  production: {
    apiKey: process.env.WIA_PROD_API_KEY,
    environment: 'production',
    logLevel: 'info'
  }
};

// Initialize with environment
const client = new ContentAI(
  config[process.env.NODE_ENV || 'development']
);
```

### 9.2 Docker Deployment

```dockerfile
FROM node:18-alpine

WORKDIR /app

COPY package*.json ./
RUN npm ci --production

COPY . .

ENV NODE_ENV=production
ENV WIA_API_KEY=${WIA_API_KEY}

CMD ["node", "dist/index.js"]
```

---

## 10. Troubleshooting

### Common Issues

**Issue: Rate Limit Exceeded**
```typescript
// Solution: Implement retry with backoff
async function retryWithBackoff(fn, retries = 3) {
  for (let i = 0; i < retries; i++) {
    try {
      return await fn();
    } catch (error) {
      if (error.code === 'RATE_LIMIT_EXCEEDED' && i < retries - 1) {
        await sleep(Math.pow(2, i) * 1000);
      } else {
        throw error;
      }
    }
  }
}
```

**Issue: Generation Timeout**
```typescript
// Solution: Increase timeout or use streaming
const client = new ContentAI({
  apiKey: process.env.WIA_API_KEY,
  timeout: 60000 // 60 seconds
});
```

**Issue: Quality Issues**
```typescript
// Solution: Enable validation and regeneration
const content = await generateWithQA(params);
```

---

**弘익人間 (홍익인간) - Benefit All Humanity**

**© 2025 SmileStory Inc. / WIA - World Certification Industry Association**
