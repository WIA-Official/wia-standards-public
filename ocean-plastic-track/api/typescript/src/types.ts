// types.ts — TypeScript declarations for the WIA ocean-plastic-track standard.
// Mirrors the schema documented under spec/PHASE-1-*.md.
// This file is reference material; production implementations should
// regenerate from the canonical OpenAPI document under spec/.

export interface ResourceRef {
  id: string;
  type: string;
  created_at: string;
}

export interface ListResponse<T> {
  items: T[];
  next_page_token?: string;
}

export interface WiaError {
  type: string;
  title: string;
  status: number;
  code: string;
  detail?: string;
  instance?: string;
  request_id: string;
}
