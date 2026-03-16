# WIA AI Interoperability Standard
## Phase 4: Integration Specification

**Version**: 1.0
**Status**: Draft
**Last Updated**: 2025-12-25

---

## Philosophy

**弘益人間 (홍익인간)** - *Benefit All Humanity*

Integration is where standards meet reality. By providing clear patterns for connecting existing AI platforms to the WIA standard, we enable a unified AI ecosystem that serves all of humanity, regardless of which AI system they choose to use.

---

## Overview

Phase 4 defines integration patterns for connecting existing AI platforms to the WIA AI Interoperability Standard. This includes:
- Adapters for major AI platforms
- Migration strategies
- Backward compatibility patterns
- Testing and validation
- Production deployment guides

### Supported Platforms

1. **Anthropic Claude**
2. **OpenAI GPT**
3. **Google Gemini**
4. **Meta Llama**
5. **Custom AI Systems**

---

## Schema

### Adapter Interface

All platform adapters must implement this interface:

```typescript
interface WIAAdapter {
  // Initialize adapter
  initialize(config: AdapterConfig): Promise<void>;

  // Convert platform message to WIA format
  toWIA(platformMessage: any): AIMessage;

  // Convert WIA message to platform format
  fromWIA(wiaMessage: AIMessage): any;

  // Send message through platform
  sendMessage(message: AIMessage, options?: SendOptions): Promise<AIMessage>;

  // Stream message through platform
  streamMessage(message: AIMessage, options?: StreamOptions): AsyncIterableIterator<AIMessage>;

  // Get platform capabilities
  getCapabilities(): PlatformCapabilities;

  // Validate message compatibility
  validateMessage(message: AIMessage): ValidationResult;
}
```

### Adapter Configuration

```typescript
interface AdapterConfig {
  platform: 'anthropic' | 'openai' | 'google' | 'meta' | 'custom';
  apiKey: string;
  baseURL?: string;
  version?: string;
  options?: {
    timeout?: number;
    retries?: number;
    defaultModel?: string;
    rateLimits?: RateLimitConfig;
  };
}
```

---

## Fields

### Platform Mapping

#### Anthropic Claude

| WIA Field | Claude API Field | Notes |
|-----------|------------------|-------|
| `message.content.parts` | `messages[].content` | Convert array to text/image blocks |
| `model.modelId` | `model` | Direct mapping |
| `options.maxTokens` | `max_tokens` | Direct mapping |
| `options.temperature` | `temperature` | Direct mapping |
| `options.stopSequences` | `stop_sequences` | Direct mapping |

#### OpenAI GPT

| WIA Field | OpenAI API Field | Notes |
|-----------|------------------|--------|
| `message.content.parts` | `messages[].content` | Convert to role-based format |
| `model.modelId` | `model` | Map WIA IDs to OpenAI IDs |
| `options.maxTokens` | `max_tokens` | Direct mapping |
| `options.temperature` | `temperature` | Direct mapping |
| `options.topP` | `top_p` | Direct mapping |

#### Google Gemini

| WIA Field | Gemini API Field | Notes |
|-----------|------------------|--------|
| `message.content.parts` | `contents[].parts` | Direct mapping |
| `model.modelId` | `model` | Add 'models/' prefix |
| `options.maxTokens` | `generationConfig.maxOutputTokens` | Direct mapping |
| `options.temperature` | `generationConfig.temperature` | Direct mapping |

---

## Validation

### Adapter Validation

Each adapter must:
1. ✅ Implement all interface methods
2. ✅ Handle platform-specific errors gracefully
3. ✅ Convert bidirectionally without data loss
4. ✅ Support streaming if platform allows
5. ✅ Respect rate limits
6. ✅ Provide meaningful error messages
7. ✅ Log integration events for debugging

### Message Compatibility

```typescript
interface ValidationResult {
  valid: boolean;
  errors?: ValidationError[];
  warnings?: ValidationWarning[];
  compatibility: {
    textSupport: boolean;
    imageSupport: boolean;
    audioSupport: boolean;
    videoSupport: boolean;
    toolUseSupport: boolean;
    streamingSupport: boolean;
  };
}
```

---

## Examples

### Example 1: Anthropic Claude Adapter

```typescript
import Anthropic from '@anthropic-ai/sdk';
import { WIAAdapter, AIMessage, AdapterConfig } from '@wia/ai-interop';

export class ClaudeAdapter implements WIAAdapter {
  private client: Anthropic;

  async initialize(config: AdapterConfig): Promise<void> {
    this.client = new Anthropic({
      apiKey: config.apiKey,
    });
  }

  toWIA(claudeMessage: Anthropic.Message): AIMessage {
    return {
      version: '1.0',
      messageId: claudeMessage.id,
      timestamp: new Date().toISOString(),
      sender: {
        type: 'ai',
        id: 'claude_assistant',
        name: 'Claude'
      },
      content: {
        type: claudeMessage.content.some(c => c.type === 'image') ? 'multimodal' : 'text',
        parts: claudeMessage.content.map(block => ({
          type: block.type === 'text' ? 'text' : 'image',
          data: block.type === 'text' ? block.text : block.source,
          metadata: {}
        }))
      },
      model: {
        provider: 'anthropic',
        modelId: claudeMessage.model,
        capabilities: ['text', 'vision', 'tool_use']
      },
      metadata: {
        stopReason: claudeMessage.stop_reason,
        usage: {
          inputTokens: claudeMessage.usage.input_tokens,
          outputTokens: claudeMessage.usage.output_tokens
        }
      }
    };
  }

  fromWIA(wiaMessage: AIMessage): Anthropic.MessageCreateParams {
    return {
      model: wiaMessage.model?.modelId || 'claude-sonnet-4-5',
      max_tokens: 4096,
      messages: [{
        role: wiaMessage.sender.type === 'human' ? 'user' : 'assistant',
        content: wiaMessage.content.parts.map(part => {
          if (part.type === 'text') {
            return { type: 'text', text: part.data as string };
          } else if (part.type === 'image') {
            return {
              type: 'image',
              source: part.data as Anthropic.ImageBlockParam.Source
            };
          }
          throw new Error(`Unsupported part type: ${part.type}`);
        })
      }]
    };
  }

  async sendMessage(message: AIMessage, options?: any): Promise<AIMessage> {
    const claudeParams = this.fromWIA(message);
    const response = await this.client.messages.create({
      ...claudeParams,
      ...options
    });
    return this.toWIA(response);
  }

  async *streamMessage(message: AIMessage, options?: any): AsyncIterableIterator<AIMessage> {
    const claudeParams = this.fromWIA(message);
    const stream = await this.client.messages.create({
      ...claudeParams,
      ...options,
      stream: true
    });

    let accumulatedText = '';
    for await (const event of stream) {
      if (event.type === 'content_block_delta') {
        if (event.delta.type === 'text_delta') {
          accumulatedText += event.delta.text;
          yield {
            ...message,
            messageId: 'streaming_' + Date.now(),
            sender: { type: 'ai', id: 'claude_assistant' },
            content: {
              type: 'text',
              parts: [{ type: 'text', data: accumulatedText }]
            }
          };
        }
      }
    }
  }

  getCapabilities(): PlatformCapabilities {
    return {
      textSupport: true,
      imageSupport: true,
      audioSupport: false,
      videoSupport: false,
      toolUseSupport: true,
      streamingSupport: true,
      maxContextWindow: 200000,
      supportedModels: ['claude-opus-4-5', 'claude-sonnet-4-5', 'claude-haiku-4']
    };
  }

  validateMessage(message: AIMessage): ValidationResult {
    const capabilities = this.getCapabilities();
    const errors: ValidationError[] = [];
    const warnings: ValidationWarning[] = [];

    // Check unsupported content types
    for (const part of message.content.parts) {
      if (part.type === 'audio' && !capabilities.audioSupport) {
        errors.push({
          field: 'content.parts',
          message: 'Audio content not supported by Claude'
        });
      }
      if (part.type === 'video' && !capabilities.videoSupport) {
        errors.push({
          field: 'content.parts',
          message: 'Video content not supported by Claude'
        });
      }
    }

    return {
      valid: errors.length === 0,
      errors: errors.length > 0 ? errors : undefined,
      warnings: warnings.length > 0 ? warnings : undefined,
      compatibility: capabilities
    };
  }
}
```

### Example 2: OpenAI GPT Adapter

```typescript
import OpenAI from 'openai';
import { WIAAdapter, AIMessage, AdapterConfig } from '@wia/ai-interop';

export class OpenAIAdapter implements WIAAdapter {
  private client: OpenAI;

  async initialize(config: AdapterConfig): Promise<void> {
    this.client = new OpenAI({
      apiKey: config.apiKey,
    });
  }

  toWIA(openaiMessage: OpenAI.Chat.ChatCompletion): AIMessage {
    const choice = openaiMessage.choices[0];
    return {
      version: '1.0',
      messageId: openaiMessage.id,
      timestamp: new Date(openaiMessage.created * 1000).toISOString(),
      sender: {
        type: 'ai',
        id: 'gpt_assistant',
        name: 'GPT'
      },
      content: {
        type: 'text',
        parts: [{
          type: 'text',
          data: choice.message.content || '',
          metadata: {}
        }]
      },
      model: {
        provider: 'openai',
        modelId: openaiMessage.model,
        capabilities: ['text', 'code']
      },
      metadata: {
        finishReason: choice.finish_reason,
        usage: {
          inputTokens: openaiMessage.usage?.prompt_tokens,
          outputTokens: openaiMessage.usage?.completion_tokens,
          totalTokens: openaiMessage.usage?.total_tokens
        }
      }
    };
  }

  fromWIA(wiaMessage: AIMessage): OpenAI.Chat.ChatCompletionCreateParams {
    return {
      model: wiaMessage.model?.modelId || 'gpt-4-turbo',
      messages: [{
        role: wiaMessage.sender.type === 'human' ? 'user' : 'assistant',
        content: wiaMessage.content.parts
          .filter(part => part.type === 'text')
          .map(part => part.data as string)
          .join('\n')
      }]
    };
  }

  async sendMessage(message: AIMessage, options?: any): Promise<AIMessage> {
    const openaiParams = this.fromWIA(message);
    const response = await this.client.chat.completions.create({
      ...openaiParams,
      ...options
    });
    return this.toWIA(response);
  }

  async *streamMessage(message: AIMessage, options?: any): AsyncIterableIterator<AIMessage> {
    const openaiParams = this.fromWIA(message);
    const stream = await this.client.chat.completions.create({
      ...openaiParams,
      ...options,
      stream: true
    });

    let accumulatedText = '';
    for await (const chunk of stream) {
      const delta = chunk.choices[0]?.delta?.content;
      if (delta) {
        accumulatedText += delta;
        yield {
          ...message,
          messageId: 'streaming_' + Date.now(),
          sender: { type: 'ai', id: 'gpt_assistant' },
          content: {
            type: 'text',
            parts: [{ type: 'text', data: accumulatedText }]
          }
        };
      }
    }
  }

  getCapabilities(): PlatformCapabilities {
    return {
      textSupport: true,
      imageSupport: true,
      audioSupport: false,
      videoSupport: false,
      toolUseSupport: true,
      streamingSupport: true,
      maxContextWindow: 128000,
      supportedModels: ['gpt-4-turbo', 'gpt-4', 'gpt-3.5-turbo']
    };
  }

  validateMessage(message: AIMessage): ValidationResult {
    return {
      valid: true,
      compatibility: this.getCapabilities()
    };
  }
}
```

### Example 3: Using Adapters in Production

```typescript
import { ClaudeAdapter, OpenAIAdapter } from '@wia/ai-interop-adapters';
import { AIMessage } from '@wia/ai-interop';

// Initialize adapters
const claudeAdapter = new ClaudeAdapter();
await claudeAdapter.initialize({
  platform: 'anthropic',
  apiKey: process.env.ANTHROPIC_API_KEY!
});

const openaiAdapter = new OpenAIAdapter();
await openaiAdapter.initialize({
  platform: 'openai',
  apiKey: process.env.OPENAI_API_KEY!
});

// Create a message
const userMessage: AIMessage = {
  version: '1.0',
  messageId: 'msg_' + Date.now(),
  timestamp: new Date().toISOString(),
  sender: {
    type: 'human',
    id: 'user_123',
    name: 'Alice'
  },
  content: {
    type: 'text',
    parts: [{
      type: 'text',
      data: 'Explain quantum computing in simple terms'
    }]
  }
};

// Send to multiple AI platforms simultaneously
const [claudeResponse, openaiResponse] = await Promise.all([
  claudeAdapter.sendMessage(userMessage),
  openaiAdapter.sendMessage(userMessage)
]);

console.log('Claude says:', claudeResponse.content.parts[0].data);
console.log('GPT says:', openaiResponse.content.parts[0].data);

// Or use the best available platform
async function sendToAnyAI(message: AIMessage) {
  const adapters = [claudeAdapter, openaiAdapter];

  for (const adapter of adapters) {
    try {
      const validation = adapter.validateMessage(message);
      if (validation.valid) {
        return await adapter.sendMessage(message);
      }
    } catch (error) {
      console.error(`Failed with ${adapter.constructor.name}:`, error);
      continue;
    }
  }

  throw new Error('All AI platforms failed');
}
```

### Example 4: Multi-Platform Conversation Router

```typescript
import { WIAAdapter, AIMessage } from '@wia/ai-interop';

class MultiPlatformRouter {
  private adapters: Map<string, WIAAdapter> = new Map();

  registerAdapter(name: string, adapter: WIAAdapter): void {
    this.adapters.set(name, adapter);
  }

  async routeMessage(
    message: AIMessage,
    strategy: 'failover' | 'fastest' | 'consensus' | 'specific',
    target?: string
  ): Promise<AIMessage> {
    switch (strategy) {
      case 'specific':
        if (!target || !this.adapters.has(target)) {
          throw new Error('Target adapter not found');
        }
        return this.adapters.get(target)!.sendMessage(message);

      case 'failover':
        return this.failoverStrategy(message);

      case 'fastest':
        return this.fastestStrategy(message);

      case 'consensus':
        return this.consensusStrategy(message);

      default:
        throw new Error('Unknown strategy');
    }
  }

  private async failoverStrategy(message: AIMessage): Promise<AIMessage> {
    for (const [name, adapter] of this.adapters) {
      try {
        return await adapter.sendMessage(message);
      } catch (error) {
        console.error(`${name} failed, trying next adapter...`);
        continue;
      }
    }
    throw new Error('All adapters failed');
  }

  private async fastestStrategy(message: AIMessage): Promise<AIMessage> {
    const promises = Array.from(this.adapters.values()).map(adapter =>
      adapter.sendMessage(message)
    );
    return Promise.race(promises);
  }

  private async consensusStrategy(message: AIMessage): Promise<AIMessage> {
    const promises = Array.from(this.adapters.values()).map(adapter =>
      adapter.sendMessage(message)
    );
    const responses = await Promise.all(promises);

    // Simple consensus: return most common response
    // In production, use more sophisticated voting
    return responses[0];
  }
}

// Usage
const router = new MultiPlatformRouter();
router.registerAdapter('claude', claudeAdapter);
router.registerAdapter('openai', openaiAdapter);

// Get fastest response
const response = await router.routeMessage(userMessage, 'fastest');
```

---

## Migration Strategies

### Strategy 1: Parallel Running

Run both legacy and WIA systems simultaneously:

```typescript
// Gradual migration
class HybridAIClient {
  async sendMessage(message: any, useWIA: boolean = false) {
    if (useWIA) {
      // New WIA path
      const wiaMessage = convertToWIA(message);
      return wiaAdapter.sendMessage(wiaMessage);
    } else {
      // Legacy path
      return legacyClient.sendMessage(message);
    }
  }
}
```

### Strategy 2: Proxy Pattern

Intercept all AI calls and transparently convert:

```typescript
class AIProxy {
  async chat(legacyRequest: any): Promise<any> {
    // Convert to WIA
    const wiaMessage = this.convertToWIA(legacyRequest);

    // Send via WIA
    const wiaResponse = await wiaAdapter.sendMessage(wiaMessage);

    // Convert back to legacy format
    return this.convertFromWIA(wiaResponse);
  }
}
```

### Strategy 3: Feature Flags

Gradually roll out WIA integration:

```typescript
import { featureFlags } from './config';

async function sendAIMessage(message: any) {
  if (featureFlags.useWIAInterop && featureFlags.rolloutPercentage > Math.random()) {
    return sendViaWIA(message);
  } else {
    return sendViaLegacy(message);
  }
}
```

---

## Testing & Validation

### Integration Tests

```typescript
import { describe, it, expect } from 'vitest';

describe('ClaudeAdapter', () => {
  it('should convert WIA message to Claude format', () => {
    const wiaMessage: AIMessage = { /* ... */ };
    const claudeMessage = adapter.fromWIA(wiaMessage);

    expect(claudeMessage.model).toBe('claude-sonnet-4-5');
    expect(claudeMessage.messages[0].role).toBe('user');
  });

  it('should convert Claude response to WIA format', () => {
    const claudeResponse = { /* ... */ };
    const wiaMessage = adapter.toWIA(claudeResponse);

    expect(wiaMessage.version).toBe('1.0');
    expect(wiaMessage.sender.type).toBe('ai');
  });

  it('should handle streaming correctly', async () => {
    const messages: AIMessage[] = [];
    for await (const msg of adapter.streamMessage(testMessage)) {
      messages.push(msg);
    }

    expect(messages.length).toBeGreaterThan(0);
    expect(messages[messages.length - 1].content.parts[0].data).toBeTruthy();
  });
});
```

---

## Production Deployment

### Checklist

- [ ] All adapters tested with real API keys
- [ ] Error handling implemented for all edge cases
- [ ] Rate limiting configured
- [ ] Monitoring and logging in place
- [ ] Fallback mechanisms tested
- [ ] Performance benchmarks established
- [ ] Security audit completed
- [ ] Documentation updated
- [ ] Team training completed

### Monitoring

```typescript
import { monitor } from './monitoring';

class MonitoredAdapter implements WIAAdapter {
  async sendMessage(message: AIMessage): Promise<AIMessage> {
    const startTime = Date.now();

    try {
      const response = await this.baseAdapter.sendMessage(message);

      monitor.recordSuccess({
        adapter: this.name,
        latency: Date.now() - startTime,
        tokens: response.metadata?.usage?.totalTokens
      });

      return response;
    } catch (error) {
      monitor.recordError({
        adapter: this.name,
        error: error.message,
        latency: Date.now() - startTime
      });
      throw error;
    }
  }
}
```

---

## Best Practices

1. **Always validate**: Check message compatibility before sending
2. **Handle errors gracefully**: Implement retry logic with exponential backoff
3. **Log everything**: Comprehensive logging for debugging
4. **Monitor performance**: Track latency, tokens, errors
5. **Test thoroughly**: Integration tests with real APIs
6. **Document differences**: Platform-specific quirks and limitations
7. **Version adapters**: Keep adapters versioned with platforms
8. **Cache when possible**: Reduce API calls with intelligent caching

---

## Conclusion

Phase 4 completes the WIA AI Interoperability Standard by providing practical integration patterns. With these adapters and strategies, any AI system can join the interoperable ecosystem, enabling seamless communication across platforms for the benefit of all humanity.

**弘익人間 (홍익인간)** - Let's build an AI ecosystem that truly benefits everyone.

---

© 2025 SmileStory Inc. / WIA
**弘益人間 (홍익인간)** · Benefit All Humanity
