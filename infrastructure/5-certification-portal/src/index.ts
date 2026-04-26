/**
 * WIA Certification Portal
 *
 * a11y.wiabook.com 연동 인증 포털
 * 211개 언어 지원
 *
 * 홍익인간 (弘益人間) - Benefit All Humanity
 */

// ============================================
// 인증 등급
// ============================================

export enum CertificationLevel {
  CERTIFIED = 'wia-certified',    // 기본 인증
  SILVER = 'wia-silver',          // 실버 등급
  GOLD = 'wia-gold',              // 골드 등급
  PLATINUM = 'wia-platinum',      // 플래티넘 등급
}

export interface CertificationBadge {
  level: CertificationLevel;
  imageUrl: string;
  altText: string;
  issuedAt: string;
  expiresAt: string;
  language: string;
}

// ============================================
// 인증 결과
// ============================================

export interface CertificationResult {
  url: string;
  score: number;
  level: CertificationLevel | null;
  passed: boolean;
  issues: AccessibilityIssue[];
  checkedAt: string;
  language: string;
  badge?: CertificationBadge;
}

export interface AccessibilityIssue {
  code: string;
  message: string;
  severity: 'error' | 'warning' | 'info';
  element?: string;
  suggestion?: string;
}

// ============================================
// 211개 언어 지원 (a11y.wiabook.com)
// ============================================

export const SUPPORTED_LANGUAGES = [
  // 주요 언어
  'ko', 'en', 'ja', 'zh', 'es', 'fr', 'de', 'it', 'pt', 'ru',
  // 아시아
  'hi', 'bn', 'pa', 'jv', 'vi', 'th', 'tr', 'fa', 'ur', 'id',
  'ms', 'tl', 'my', 'km', 'lo', 'ne', 'si', 'ta', 'te', 'kn',
  'ml', 'gu', 'mr', 'or', 'as', 'mn', 'bo', 'dz', 'ug',
  // 유럽
  'pl', 'uk', 'nl', 'sv', 'no', 'da', 'fi', 'cs', 'sk', 'hu',
  'ro', 'bg', 'hr', 'sr', 'sl', 'mk', 'sq', 'el', 'lt', 'lv',
  'et', 'mt', 'ga', 'cy', 'gd', 'eu', 'ca', 'gl', 'is', 'fo',
  // 아프리카
  'ar', 'he', 'am', 'sw', 'ha', 'yo', 'ig', 'zu', 'xh', 'af',
  'so', 'rw', 'ny', 'sn', 'st', 'tn', 'ts', 've', 'nr', 'ss',
  // 태평양
  'mi', 'haw', 'sm', 'to', 'fj', 'ty',
  // 수어 (Sign Languages) - 농인 커뮤니티
  'ase', 'bfi', 'fsl', 'gsg', 'jsl', 'csl', 'ksl', 'asl',
  // ... 총 211개
] as const;

export type SupportedLanguage = typeof SUPPORTED_LANGUAGES[number];

// ============================================
// Certification Portal API
// ============================================

export class CertificationPortal {
  private apiEndpoint: string;
  private language: SupportedLanguage;

  constructor(options?: {
    apiEndpoint?: string;
    language?: SupportedLanguage;
  }) {
    this.apiEndpoint = options?.apiEndpoint || 'https://a11y.wiabook.com/api';
    this.language = options?.language || 'en';
  }

  /**
   * 웹사이트 접근성 검사
   */
  async checkAccessibility(url: string): Promise<CertificationResult> {
    const response = await fetch(
      `${this.apiEndpoint}/check?url=${encodeURIComponent(url)}&lang=${this.language}`
    );
    return response.json();
  }

  /**
   * 인증 배지 생성
   */
  async generateBadge(
    url: string,
    level: CertificationLevel
  ): Promise<CertificationBadge> {
    const response = await fetch(`${this.apiEndpoint}/badge`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ url, level, language: this.language }),
    });
    return response.json();
  }

  /**
   * 인증 배지 HTML 코드
   */
  getBadgeHtml(badge: CertificationBadge): string {
    return `<a href="https://a11y.wiabook.com/verify/${badge.level}" target="_blank" rel="noopener">
  <img src="${badge.imageUrl}" alt="${badge.altText}" width="120" height="40" />
</a>`;
  }

  /**
   * 언어 설정
   */
  setLanguage(lang: SupportedLanguage): void {
    this.language = lang;
  }

  /**
   * 지원 언어 목록
   */
  getSupportedLanguages(): readonly string[] {
    return SUPPORTED_LANGUAGES;
  }

  /**
   * WIA 도메인별 인증 체크
   */
  async checkDomain(
    domain: 'voice' | 'auto' | 'ci' | 'home' | 'quantum' | 'medical' | 'climate',
    targetUrl: string
  ): Promise<CertificationResult> {
    const response = await fetch(
      `${this.apiEndpoint}/domain/${domain}/check?url=${encodeURIComponent(targetUrl)}&lang=${this.language}`
    );
    return response.json();
  }
}

// ============================================
// 배지 URL 헬퍼
// ============================================

export const BadgeUrls = {
  certified: 'https://a11y.wiabook.com/badges/wia-certified.svg',
  silver: 'https://a11y.wiabook.com/badges/wia-silver.svg',
  gold: 'https://a11y.wiabook.com/badges/wia-gold.svg',
  platinum: 'https://a11y.wiabook.com/badges/wia-platinum.svg',
} as const;

// ============================================
// 점수 → 등급 변환
// ============================================

export function scoreToLevel(score: number): CertificationLevel | null {
  if (score >= 95) return CertificationLevel.PLATINUM;
  if (score >= 85) return CertificationLevel.GOLD;
  if (score >= 75) return CertificationLevel.SILVER;
  if (score >= 60) return CertificationLevel.CERTIFIED;
  return null;
}

// ============================================
// Export
// ============================================

export default CertificationPortal;

// 홍익인간 (弘益人間)
// 인류를 널리 이롭게 하라
// Benefit All Humanity
