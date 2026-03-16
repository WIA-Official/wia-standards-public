# WIA-CORE-007 PHASE 4: Advanced Features and Extensions

**Version:** 1.0
**Status:** APPROVED
**Last Updated:** 2025-12-27
**Authors:** WIA Standards Committee

## 1. Introduction

PHASE 4 defines advanced features and extension mechanisms for WIA-CORE-007 Universal Protocol. This document covers middleware, plugins, advanced patterns, and extensibility features that enable sophisticated use cases while maintaining protocol simplicity.

## 2. Middleware Architecture

### 2.1 Middleware Interface

```typescript
interface Middleware {
  (context: Context, next: NextFunction): Promise<void>;
}

interface Context {
  message: Message;
  metadata: Metadata;
  transport: Transport;
  user?: any;
  [key: string]: any;
}

type NextFunction = () => Promise<void>;
```

### 2.2 Middleware Chain

```typescript
class MiddlewareChain {
  private middleware: Middleware[] = [];

  use(middleware: Middleware): this {
    this.middleware.push(middleware);
    return this;
  }

  async execute(context: Context): Promise<void> {
    let index = 0;

    const next = async (): Promise<void> => {
      if (index >= this.middleware.length) {
        return;
      }

      const middleware = this.middleware[index++];
      await middleware(context, next);
    };

    await next();
  }
}
```

### 2.3 Common Middleware Patterns

**Logging Middleware:**
```typescript
const loggingMiddleware: Middleware = async (context, next) => {
  const startTime = Date.now();

  logger.info('Request started', {
    method: context.message.method,
    id: context.message.id
  });

  try {
    await next();

    logger.info('Request completed', {
      method: context.message.method,
      duration: Date.now() - startTime
    });
  } catch (error) {
    logger.error('Request failed', {
      method: context.message.method,
      error: error.message
    });
    throw error;
  }
};
```

**Authentication Middleware:**
```typescript
const authMiddleware: Middleware = async (context, next) => {
  const token = context.metadata.authorization;

  if (!token) {
    throw new UnauthorizedError('Missing authentication token');
  }

  try {
    const payload = await verifyToken(token);
    context.user = payload;
    await next();
  } catch (error) {
    throw new UnauthorizedError('Invalid token');
  }
};
```

**Rate Limiting Middleware:**
```typescript
const rateLimitMiddleware = (options: RateLimitOptions): Middleware => {
  const limiter = new RateLimiter(options);

  return async (context, next) => {
    const key = context.user?.id || context.metadata['x-client-ip'];

    const allowed = await limiter.checkLimit(key);

    if (!allowed) {
      throw new RateLimitError('Too many requests');
    }

    await next();
  };
};
```

## 3. Plugin System

### 3.1 Plugin Interface

```typescript
interface Plugin {
  name: string;
  version: string;
  install(protocol: UniversalProtocol): void;
  uninstall?(protocol: UniversalProtocol): void;
}
```

### 3.2 Plugin Implementation

```typescript
class TelemetryPlugin implements Plugin {
  name = 'telemetry';
  version = '1.0.0';

  constructor(private options: TelemetryOptions) {}

  install(protocol: UniversalProtocol): void {
    // Add middleware
    protocol.use(this.createMiddleware());

    // Add methods
    protocol.getMetrics = () => this.getMetrics();

    // Subscribe to events
    protocol.on('message', this.handleMessage.bind(this));
  }

  private createMiddleware(): Middleware {
    return async (context, next) => {
      const span = this.tracer.startSpan(context.message.method);

      try {
        await next();
        span.setStatus({ code: 'OK' });
      } catch (error) {
        span.setStatus({ code: 'ERROR', message: error.message });
        throw error;
      } finally {
        span.end();
      }
    };
  }

  private getMetrics(): Metrics {
    return {
      requestCount: this.requestCount,
      errorCount: this.errorCount,
      averageLatency: this.calculateAverageLatency()
    };
  }
}

// Usage
const plugin = new TelemetryPlugin({
  serviceName: 'my-service',
  exportInterval: 60000
});

protocol.use(plugin);
```

### 3.3 Plugin Discovery

```typescript
interface PluginRegistry {
  register(plugin: Plugin): void;
  unregister(name: string): void;
  get(name: string): Plugin | undefined;
  list(): Plugin[];
}

class DefaultPluginRegistry implements PluginRegistry {
  private plugins = new Map<string, Plugin>();

  register(plugin: Plugin): void {
    if (this.plugins.has(plugin.name)) {
      throw new Error(`Plugin ${plugin.name} already registered`);
    }

    this.plugins.set(plugin.name, plugin);
  }

  unregister(name: string): void {
    const plugin = this.plugins.get(name);

    if (plugin?.uninstall) {
      plugin.uninstall(this.protocol);
    }

    this.plugins.delete(name);
  }

  get(name: string): Plugin | undefined {
    return this.plugins.get(name);
  }

  list(): Plugin[] {
    return Array.from(this.plugins.values());
  }
}
```

## 4. Advanced Communication Patterns

### 4.1 Request Batching

```typescript
interface BatchRequest {
  requests: Array<{
    method: string;
    params: any;
    id?: string;
  }>;
}

interface BatchResponse {
  responses: Array<{
    id: string;
    result?: any;
    error?: Error;
  }>;
}

async function batchCall(
  protocol: UniversalProtocol,
  requests: BatchRequest
): Promise<BatchResponse> {
  const response = await protocol.call('$batch', requests);
  return response;
}
```

### 4.2 Request Multiplexing

```typescript
class MultiplexedProtocol {
  private pending = new Map<string, Promise<any>>();

  async call(method: string, params: any): Promise<any> {
    const key = this.generateKey(method, params);

    if (this.pending.has(key)) {
      return this.pending.get(key);
    }

    const promise = this.protocol.call(method, params);
    this.pending.set(key, promise);

    try {
      const result = await promise;
      return result;
    } finally {
      this.pending.delete(key);
    }
  }

  private generateKey(method: string, params: any): string {
    return `${method}:${JSON.stringify(params)}`;
  }
}
```

### 4.3 Request Coalescing

```typescript
class RequestCoalescer {
  private batches = new Map<string, Batch>();

  async add(method: string, params: any): Promise<any> {
    const batch = this.getBatch(method);

    return new Promise((resolve, reject) => {
      batch.add({ params, resolve, reject });

      if (!batch.timer) {
        batch.timer = setTimeout(() => {
          this.flush(method);
        }, this.options.delay);
      }

      if (batch.size >= this.options.maxSize) {
        clearTimeout(batch.timer);
        this.flush(method);
      }
    });
  }

  private async flush(method: string): Promise<void> {
    const batch = this.batches.get(method);
    if (!batch) return;

    this.batches.delete(method);

    try {
      const results = await this.protocol.call(
        method + '.batch',
        batch.items.map(item => item.params)
      );

      batch.items.forEach((item, index) => {
        item.resolve(results[index]);
      });
    } catch (error) {
      batch.items.forEach(item => {
        item.reject(error);
      });
    }
  }
}
```

### 4.4 Circuit Breaker

```typescript
class CircuitBreaker {
  private state: CircuitState = 'CLOSED';
  private failures = 0;
  private nextAttempt = Date.now();

  async execute<T>(fn: () => Promise<T>): Promise<T> {
    if (this.state === 'OPEN') {
      if (Date.now() < this.nextAttempt) {
        throw new CircuitOpenError('Circuit breaker is open');
      }
      this.state = 'HALF_OPEN';
    }

    try {
      const result = await fn();

      if (this.state === 'HALF_OPEN') {
        this.state = 'CLOSED';
        this.failures = 0;
      }

      return result;
    } catch (error) {
      this.failures++;

      if (this.failures >= this.options.threshold) {
        this.state = 'OPEN';
        this.nextAttempt = Date.now() + this.options.timeout;
      }

      throw error;
    }
  }
}

type CircuitState = 'CLOSED' | 'OPEN' | 'HALF_OPEN';
```

## 5. Service Discovery

### 5.1 Service Registry

```typescript
interface Service {
  name: string;
  version: string;
  endpoints: Endpoint[];
  metadata: Record<string, any>;
  health: HealthStatus;
}

interface Endpoint {
  transport: string;
  url: string;
  capabilities: TransportCapabilities;
}

interface ServiceRegistry {
  register(service: Service): Promise<void>;
  unregister(serviceName: string): Promise<void>;
  discover(serviceName: string): Promise<Service[]>;
  watch(serviceName: string, callback: (services: Service[]) => void): void;
}
```

### 5.2 Load Balancing

```typescript
interface LoadBalancer {
  select(services: Service[]): Service;
}

class RoundRobinLoadBalancer implements LoadBalancer {
  private index = 0;

  select(services: Service[]): Service {
    const service = services[this.index % services.length];
    this.index++;
    return service;
  }
}

class WeightedLoadBalancer implements LoadBalancer {
  select(services: Service[]): Service {
    const totalWeight = services.reduce(
      (sum, s) => sum + (s.metadata.weight || 1),
      0
    );

    let random = Math.random() * totalWeight;

    for (const service of services) {
      random -= service.metadata.weight || 1;
      if (random <= 0) {
        return service;
      }
    }

    return services[0];
  }
}
```

## 6. Observability

### 6.1 Distributed Tracing

```typescript
interface Tracer {
  startSpan(name: string, options?: SpanOptions): Span;
  inject(span: Span, carrier: any): void;
  extract(carrier: any): SpanContext | undefined;
}

interface Span {
  setTag(key: string, value: any): void;
  log(fields: Record<string, any>): void;
  finish(): void;
}

// OpenTelemetry integration
const tracingMiddleware: Middleware = async (context, next) => {
  const parentContext = tracer.extract(context.metadata);

  const span = tracer.startSpan(context.message.method, {
    childOf: parentContext,
    tags: {
      'protocol.version': context.metadata.version,
      'protocol.transport': context.transport.type
    }
  });

  try {
    await next();
    span.setTag('status', 'success');
  } catch (error) {
    span.setTag('status', 'error');
    span.setTag('error', true);
    span.log({ event: 'error', message: error.message });
    throw error;
  } finally {
    span.finish();
  }
};
```

### 6.2 Metrics Collection

```typescript
interface MetricsCollector {
  counter(name: string, labels?: Labels): Counter;
  gauge(name: string, labels?: Labels): Gauge;
  histogram(name: string, labels?: Labels): Histogram;
}

// Prometheus integration
const metricsMiddleware: Middleware = async (context, next) => {
  const startTime = Date.now();

  requestCounter.inc({
    method: context.message.method,
    transport: context.transport.type
  });

  try {
    await next();

    requestDuration.observe(
      { method: context.message.method, status: 'success' },
      Date.now() - startTime
    );
  } catch (error) {
    requestDuration.observe(
      { method: context.message.method, status: 'error' },
      Date.now() - startTime
    );

    errorCounter.inc({
      method: context.message.method,
      code: error.code
    });

    throw error;
  }
};
```

## 7. Schema Validation

### 7.1 JSON Schema Validation

```typescript
import Ajv from 'ajv';

const ajv = new Ajv();

interface SchemaRegistry {
  register(method: string, schema: JSONSchema): void;
  validate(method: string, data: any): ValidationResult;
}

class DefaultSchemaRegistry implements SchemaRegistry {
  private schemas = new Map<string, ValidateFunction>();

  register(method: string, schema: JSONSchema): void {
    const validate = ajv.compile(schema);
    this.schemas.set(method, validate);
  }

  validate(method: string, data: any): ValidationResult {
    const validate = this.schemas.get(method);

    if (!validate) {
      return { valid: true };
    }

    const valid = validate(data);

    return {
      valid,
      errors: validate.errors || []
    };
  }
}

// Middleware
const validationMiddleware: Middleware = async (context, next) => {
  const result = schemaRegistry.validate(
    context.message.method,
    context.message.payload
  );

  if (!result.valid) {
    throw new ValidationError('Invalid request', result.errors);
  }

  await next();
};
```

## 8. Retry Strategies

### 8.1 Retry Configuration

```typescript
interface RetryOptions {
  maxAttempts: number;
  initialDelay: number;
  maxDelay: number;
  backoff: 'linear' | 'exponential' | 'constant';
  jitter: boolean;
  retryableErrors: string[];
  shouldRetry?: (error: Error, attempt: number) => boolean;
}
```

### 8.2 Retry Implementation

```typescript
class RetryHandler {
  async execute<T>(
    fn: () => Promise<T>,
    options: RetryOptions
  ): Promise<T> {
    let attempt = 0;

    while (attempt < options.maxAttempts) {
      try {
        return await fn();
      } catch (error) {
        attempt++;

        if (!this.shouldRetry(error, attempt, options)) {
          throw error;
        }

        const delay = this.calculateDelay(attempt, options);
        await sleep(delay);
      }
    }

    throw new MaxRetriesExceededError();
  }

  private calculateDelay(attempt: number, options: RetryOptions): number {
    let delay: number;

    switch (options.backoff) {
      case 'linear':
        delay = options.initialDelay * attempt;
        break;
      case 'exponential':
        delay = options.initialDelay * Math.pow(2, attempt - 1);
        break;
      case 'constant':
        delay = options.initialDelay;
        break;
    }

    delay = Math.min(delay, options.maxDelay);

    if (options.jitter) {
      delay = delay * (0.5 + Math.random() * 0.5);
    }

    return delay;
  }

  private shouldRetry(
    error: Error,
    attempt: number,
    options: RetryOptions
  ): boolean {
    if (attempt >= options.maxAttempts) {
      return false;
    }

    if (options.shouldRetry) {
      return options.shouldRetry(error, attempt);
    }

    return options.retryableErrors.includes(error.code);
  }
}
```

## 9. Multi-Tenancy

### 9.1 Tenant Isolation

```typescript
interface TenantContext {
  tenantId: string;
  database: Database;
  config: TenantConfig;
  features: string[];
}

const multiTenancyMiddleware: Middleware = async (context, next) => {
  const tenantId = context.metadata['x-tenant-id'];

  if (!tenantId) {
    throw new Error('Tenant ID required');
  }

  const tenant = await tenantRegistry.get(tenantId);

  if (!tenant) {
    throw new NotFoundError('Tenant not found');
  }

  context.tenant = tenant;
  context.database = getDatabaseForTenant(tenantId);

  await next();
};
```

## 10. Compliance and Standards

### 10.1 Standards Conformance

Implementations SHOULD conform to:
- ISO/IEC 27001 (Security)
- ISO/IEC 25010 (Software Quality)
- GDPR (Data Protection)
- SOC 2 (Security Controls)

### 10.2 Audit Trail

```typescript
interface AuditLog {
  timestamp: string;
  userId: string;
  action: string;
  resource: string;
  result: 'success' | 'failure';
  metadata: Record<string, any>;
}

const auditMiddleware: Middleware = async (context, next) => {
  const auditLog: AuditLog = {
    timestamp: new Date().toISOString(),
    userId: context.user?.id,
    action: context.message.method,
    resource: context.message.payload?.id,
    result: 'success',
    metadata: {}
  };

  try {
    await next();
  } catch (error) {
    auditLog.result = 'failure';
    auditLog.metadata.error = error.message;
    throw error;
  } finally {
    await auditLogger.log(auditLog);
  }
};
```

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
