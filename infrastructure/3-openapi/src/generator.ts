/**
 * WIA OpenAPI Generator
 *
 * 모든 WIA 표준에서 OpenAPI/Swagger 스펙 자동 생성
 *
 * @example
 * ```typescript
 * import { generateOpenAPI } from '@anthropic/wia-openapi';
 *
 * const spec = generateOpenAPI(['voice', 'auto', 'ci']);
 * console.log(JSON.stringify(spec, null, 2));
 * ```
 *
 * 홍익인간 (弘益人間)
 */

export interface OpenAPISpec {
  openapi: string;
  info: {
    title: string;
    version: string;
    description: string;
    contact: {
      name: string;
      url: string;
    };
  };
  servers: Array<{
    url: string;
    description: string;
  }>;
  paths: Record<string, PathItem>;
  components: {
    schemas: Record<string, SchemaObject>;
    securitySchemes: Record<string, SecurityScheme>;
  };
  tags: Array<{
    name: string;
    description: string;
  }>;
}

export interface PathItem {
  get?: Operation;
  post?: Operation;
  put?: Operation;
  delete?: Operation;
}

export interface Operation {
  tags: string[];
  summary: string;
  description: string;
  operationId: string;
  parameters?: Parameter[];
  requestBody?: RequestBody;
  responses: Record<string, Response>;
}

export interface Parameter {
  name: string;
  in: 'query' | 'path' | 'header';
  required: boolean;
  schema: SchemaObject;
  description: string;
}

export interface RequestBody {
  required: boolean;
  content: Record<string, { schema: SchemaObject }>;
}

export interface Response {
  description: string;
  content?: Record<string, { schema: SchemaObject }>;
}

export interface SchemaObject {
  type: string;
  properties?: Record<string, SchemaObject>;
  items?: SchemaObject;
  required?: string[];
  description?: string;
  example?: unknown;
}

export interface SecurityScheme {
  type: string;
  scheme?: string;
  bearerFormat?: string;
}

// 도메인별 API 정의
const domainAPIs: Record<string, { paths: Record<string, PathItem>; schemas: Record<string, SchemaObject> }> = {
  voice: {
    paths: {
      '/voice/intent': {
        post: {
          tags: ['Voice'],
          summary: 'Parse voice intent',
          description: '음성 명령을 의도로 파싱',
          operationId: 'parseVoiceIntent',
          requestBody: {
            required: true,
            content: {
              'application/json': {
                schema: { $ref: '#/components/schemas/VoiceInput' } as any
              }
            }
          },
          responses: {
            '200': {
              description: 'Intent parsed successfully',
              content: {
                'application/json': {
                  schema: { $ref: '#/components/schemas/IntentResult' } as any
                }
              }
            }
          }
        }
      }
    },
    schemas: {
      VoiceInput: {
        type: 'object',
        properties: {
          audio: { type: 'string', description: 'Base64 encoded audio' },
          language: { type: 'string', description: 'Language code (e.g., ko, en)' }
        },
        required: ['audio']
      },
      IntentResult: {
        type: 'object',
        properties: {
          intent: { type: 'string' },
          confidence: { type: 'number' },
          entities: { type: 'array', items: { type: 'object' } }
        }
      }
    }
  },

  auto: {
    paths: {
      '/auto/route': {
        post: {
          tags: ['Auto'],
          summary: 'Plan autonomous route',
          description: '자율주행 경로 계획',
          operationId: 'planRoute',
          requestBody: {
            required: true,
            content: {
              'application/json': {
                schema: { $ref: '#/components/schemas/RouteRequest' } as any
              }
            }
          },
          responses: {
            '200': {
              description: 'Route planned',
              content: {
                'application/json': {
                  schema: { $ref: '#/components/schemas/RouteResult' } as any
                }
              }
            }
          }
        }
      }
    },
    schemas: {
      RouteRequest: {
        type: 'object',
        properties: {
          start: { type: 'object', properties: { lat: { type: 'number' }, lng: { type: 'number' } } },
          end: { type: 'object', properties: { lat: { type: 'number' }, lng: { type: 'number' } } }
        }
      },
      RouteResult: {
        type: 'object',
        properties: {
          waypoints: { type: 'array', items: { type: 'object' } },
          distance: { type: 'number' },
          duration: { type: 'number' }
        }
      }
    }
  },

  ci: {
    paths: {
      '/ci/octave': {
        post: {
          tags: ['CI'],
          summary: 'Detect octave from audio',
          description: '오디오에서 옥타브 감지 (CI 사용자용)',
          operationId: 'detectOctave',
          requestBody: {
            required: true,
            content: {
              'application/json': {
                schema: { $ref: '#/components/schemas/AudioInput' } as any
              }
            }
          },
          responses: {
            '200': {
              description: 'Octave detected',
              content: {
                'application/json': {
                  schema: { $ref: '#/components/schemas/OctaveResult' } as any
                }
              }
            }
          }
        }
      }
    },
    schemas: {
      AudioInput: {
        type: 'object',
        properties: {
          audio: { type: 'string', description: 'Base64 encoded audio' },
          sampleRate: { type: 'number', description: 'Sample rate in Hz' }
        }
      },
      OctaveResult: {
        type: 'object',
        properties: {
          f0: { type: 'number', description: 'Fundamental frequency' },
          octave: { type: 'number', description: 'Octave number (0-8)' },
          note: { type: 'string', description: 'Note name (C, D, E...)' }
        }
      }
    }
  },

  a11y: {
    paths: {
      '/a11y/score': {
        get: {
          tags: ['A11Y'],
          summary: 'Get accessibility score',
          description: '웹페이지 접근성 점수 (https://a11y.wiabook.com)',
          operationId: 'getA11yScore',
          parameters: [
            {
              name: 'url',
              in: 'query',
              required: true,
              schema: { type: 'string' },
              description: 'URL to check'
            }
          ],
          responses: {
            '200': {
              description: 'Score calculated',
              content: {
                'application/json': {
                  schema: { $ref: '#/components/schemas/A11yScore' } as any
                }
              }
            }
          }
        }
      }
    },
    schemas: {
      A11yScore: {
        type: 'object',
        properties: {
          score: { type: 'number', description: '0-100' },
          grade: { type: 'string', description: 'A, B, C, D, F' },
          wcagLevel: { type: 'string', description: 'A, AA, AAA' },
          issues: { type: 'array', items: { type: 'object' } }
        }
      }
    }
  }
};

/**
 * OpenAPI 스펙 생성
 */
export function generateOpenAPI(domains: string[] = ['voice', 'auto', 'ci', 'a11y']): OpenAPISpec {
  const paths: Record<string, PathItem> = {};
  const schemas: Record<string, SchemaObject> = {};
  const tags: Array<{ name: string; description: string }> = [];

  for (const domain of domains) {
    const api = domainAPIs[domain];
    if (api) {
      Object.assign(paths, api.paths);
      Object.assign(schemas, api.schemas);
      tags.push({
        name: domain.charAt(0).toUpperCase() + domain.slice(1),
        description: `WIA ${domain.toUpperCase()} API`
      });
    }
  }

  return {
    openapi: '3.0.3',
    info: {
      title: 'WIA Standards API',
      version: '1.0.0',
      description: `
# WIA (World Certification Industry Association) API

모든 WIA 표준에 대한 통합 REST API

## 철학
홍익인간 (弘益人間) - Benefit All Humanity

## 접근성 대시보드
https://a11y.wiabook.com (211개 언어 지원!)

## 도메인
${domains.map(d => `- ${d.toUpperCase()}`).join('\n')}
      `.trim(),
      contact: {
        name: 'WIA Official',
        url: 'https://wia.live'
      }
    },
    servers: [
      {
        url: 'https://api.wia.live/v1',
        description: 'Production'
      },
      {
        url: 'http://localhost:3000/v1',
        description: 'Local development'
      }
    ],
    paths,
    components: {
      schemas,
      securitySchemes: {
        bearerAuth: {
          type: 'http',
          scheme: 'bearer',
          bearerFormat: 'JWT'
        }
      }
    },
    tags
  };
}

/**
 * YAML 형식으로 출력
 */
export function toYAML(spec: OpenAPISpec): string {
  // 간단한 YAML 변환 (실제로는 js-yaml 사용)
  return JSON.stringify(spec, null, 2)
    .replace(/"/g, '')
    .replace(/,$/gm, '');
}

/**
 * Swagger UI HTML 생성
 */
export function generateSwaggerUI(spec: OpenAPISpec): string {
  return `
<!DOCTYPE html>
<html>
<head>
  <title>WIA API Documentation</title>
  <link rel="stylesheet" href="https://unpkg.com/swagger-ui-dist/swagger-ui.css">
</head>
<body>
  <div id="swagger-ui"></div>
  <script src="https://unpkg.com/swagger-ui-dist/swagger-ui-bundle.js"></script>
  <script>
    SwaggerUIBundle({
      spec: ${JSON.stringify(spec)},
      dom_id: '#swagger-ui',
      presets: [SwaggerUIBundle.presets.apis],
    });
  </script>
</body>
</html>
  `.trim();
}
