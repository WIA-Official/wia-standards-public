// WIA-FINANCIAL_INCLUSION TypeScript SDK
// Version: 1.0.0
// 弘益人間 (Benefit All Humanity)
//
// References (CITATION-POLICY v1.0 §2.1 ALLOW):
//   - HTTP/1.1 RFC 9112, HTTP/2 RFC 9113, TLS 1.3 RFC 8446
//   - JSON RFC 8259, RFC 9457 (Problem Details)
//   - JWT RFC 7519, OAuth 2.0 RFC 6749, PKCE RFC 7636
//   - ISO 20022 (financial-services messaging)
//   - ISO 8601:2019 (date/time)

import {
  AccountIdentifier,
  HealthStatus,
  ListOptions,
  MoneyAmount,
  RecordList,
  TransactionRecord,
  ValidationResult,
} from './types';

export interface ClientConfig {
  apiKey?: string;
  baseUrl?: string;                      // default: https://api.wia.org/v1
  timeoutMs?: number;                    // default: 15000
  acceptLanguage?: string;               // BCP 47 (RFC 5646)
}

export interface RecordInput {
  origin: TransactionRecord['origin'];
  destination: TransactionRecord['destination'];
  origin_account?: AccountIdentifier;
  destination_account?: AccountIdentifier;
  amount: MoneyAmount;
  category: TransactionRecord['category'];
  metadata?: Record<string, unknown>;
}

export interface WIAClient {
  createRecord(input: RecordInput): Promise<TransactionRecord>;
  getRecord(id: string): Promise<TransactionRecord>;
  listRecords(options?: ListOptions): Promise<RecordList>;
  validate(input: unknown): Promise<ValidationResult>;
  healthCheck(): Promise<HealthStatus>;
}

export function createClient(config: ClientConfig = {}): WIAClient {
  const baseUrl = config.baseUrl ?? 'https://api.wia.org/v1';
  const acceptLanguage = config.acceptLanguage ?? 'en';
  const timeoutMs = config.timeoutMs ?? 15000;

  function headers(extra: Record<string, string> = {}): Headers {
    const h = new Headers({
      'Content-Type': 'application/json',
      'Accept': 'application/json',
      'Accept-Language': acceptLanguage,
      ...extra,
    });
    if (config.apiKey) h.set('Authorization', `Bearer ${config.apiKey}`);
    return h;
  }

  async function call<T>(method: string, path: string, body?: unknown): Promise<T> {
    const ctrl = new AbortController();
    const timer = setTimeout(() => ctrl.abort(), timeoutMs);
    try {
      const resp = await fetch(`${baseUrl}${path}`, {
        method,
        headers: headers(),
        body: body == null ? undefined : JSON.stringify(body),
        signal: ctrl.signal,
      });
      if (!resp.ok) {
        // RFC 9457 Problem Details
        const text = await resp.text();
        let problem: { type?: string; title?: string; status?: number; detail?: string };
        try { problem = JSON.parse(text); } catch { problem = { detail: text }; }
        throw Object.assign(new Error(problem.title ?? `HTTP ${resp.status}`), { problem, status: resp.status });
      }
      return (await resp.json()) as T;
    } finally {
      clearTimeout(timer);
    }
  }

  return {
    createRecord: (input) => call<TransactionRecord>('POST', '/wia-financial-inclusion/records', input),
    getRecord: (id) => call<TransactionRecord>('GET', `/wia-financial-inclusion/records/${encodeURIComponent(id)}`),
    listRecords: (options) => {
      const q = new URLSearchParams();
      if (options?.cursor) q.set('cursor', options.cursor);
      if (options?.limit != null) q.set('limit', String(options.limit));
      const path = `/wia-financial-inclusion/records${q.toString() ? '?' + q.toString() : ''}`;
      return call<RecordList>('GET', path);
    },
    validate: (input) => call<ValidationResult>('POST', '/wia-financial-inclusion/records/validate', input),
    healthCheck: () => call<HealthStatus>('GET', '/wia-financial-inclusion/health'),
  };
}

export type { TransactionRecord, RecordInput, RecordList, ValidationResult, HealthStatus };
