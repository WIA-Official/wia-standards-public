# 제5장: API 인터페이스

> **弘益人間 (홍익인간)** - 명확하고 일관된 API는 개발자와 사용자 모두를 이롭게 합니다.

## 5.1 API 설계 원칙

### 5.1.1 핵심 원칙

```typescript
/**
 * WIA-LANG-006 API 설계 원칙
 *
 * 1. 단순성 (Simplicity): 복잡한 기능도 간단한 API로
 * 2. 일관성 (Consistency): 모든 언어에서 동일한 패턴
 * 3. 예측가능성 (Predictability): 직관적인 동작
 * 4. 확장성 (Extensibility): 쉬운 기능 확장
 * 5. 안전성 (Safety): 타입 안전 및 오류 처리
 * 6. 성능 (Performance): 최적화된 실행
 */

interface WIADesignPrinciples {
  simplicity: {
    principle: "최소한의 API로 최대 기능";
    example: "t('key') - 가장 간단한 번역";
  };

  consistency: {
    principle: "모든 환경에서 동일한 API";
    frameworks: ["React", "Vue", "Angular", "Svelte", "vanilla"];
  };

  predictability: {
    principle: "명확한 입력과 출력";
    guarantees: [
      "항상 문자열 반환",
      "누락된 키는 키 이름 반환",
      "null/undefined 안전"
    ];
  };

  extensibility: {
    principle: "플러그인과 훅으로 확장";
    mechanisms: ["plugins", "hooks", "middleware"];
  };

  safety: {
    principle: "TypeScript 우선 설계";
    features: ["타입 추론", "자동완성", "컴파일 타임 검증"];
  };

  performance: {
    principle: "지연 로딩과 캐싱";
    optimizations: ["lazy loading", "memoization", "tree shaking"];
  };
}
```

### 5.1.2 REST vs GraphQL

```yaml
api_approaches:
  rest:
    pros:
      - "간단한 구조"
      - "광범위한 도구 지원"
      - "캐싱 용이"
      - "HTTP 표준 활용"
    cons:
      - "Over-fetching/Under-fetching"
      - "여러 요청 필요"
      - "버전 관리 복잡"
    use_cases:
      - "간단한 CRUD 작업"
      - "공개 API"
      - "모바일 앱"

  graphql:
    pros:
      - "정확한 데이터 요청"
      - "단일 엔드포인트"
      - "강력한 타입 시스템"
      - "실시간 구독"
    cons:
      - "복잡한 설정"
      - "캐싱 어려움"
      - "학습 곡선"
    use_cases:
      - "복잡한 데이터 관계"
      - "실시간 업데이트"
      - "다양한 클라이언트"

  wia_recommendation:
    approach: "Both (Hybrid)"
    rationale: |
      REST는 간단한 작업에, GraphQL은 복잡한
      쿼리와 실시간 기능에 사용
```

## 5.2 Core API

### 5.2.1 초기화 및 설정

```typescript
import { createWIALang, WIALangConfig } from '@wia/lang-006';

// 기본 설정
interface WIALangConfig {
  // 필수 설정
  defaultLocale: string;           // 기본 언어
  supportedLocales: string[];      // 지원 언어 목록

  // 선택적 설정
  fallbackLocale?: string;         // 폴백 언어 (기본: defaultLocale)
  namespace?: string | string[];   // 네임스페이스
  debug?: boolean;                 // 디버그 모드

  // 리소스 로딩
  resources?: Resources;           // 미리 로드된 리소스
  loadPath?: string;               // 리소스 경로 패턴
  resourceLoader?: ResourceLoader; // 커스텀 로더

  // 성능 최적화
  lazy?: boolean;                  // 지연 로딩
  preload?: string[];              // 미리 로드할 언어
  cache?: CacheConfig;             // 캐시 설정

  // 동작 설정
  returnEmptyString?: boolean;     // 빈 문자열 반환 (기본: false)
  returnNull?: boolean;            // null 반환 (기본: false)
  returnObjects?: boolean;         // 객체 반환 허용 (기본: false)
  saveMissing?: boolean;           // 누락 키 저장 (기본: false)
  missingKeyHandler?: MissingKeyHandler;

  // 인터폴레이션
  interpolation?: {
    prefix?: string;               // 기본: '{{'
    suffix?: string;               // 기본: '}}'
    escapeValue?: boolean;         // HTML 이스케이프
    format?: FormatFunction;       // 커스텀 포맷
  };

  // 복수형
  pluralization?: {
    resolvers?: PluralResolvers;   // 커스텀 복수형 규칙
  };

  // 후처리
  postProcess?: string | string[]; // 후처리 플러그인
}

// 초기화
const i18n = createWIALang({
  defaultLocale: 'en',
  supportedLocales: ['en', 'ko', 'ja', 'zh', 'ar', 'es', 'fr', 'de'],
  fallbackLocale: 'en',

  // 자동 리소스 로딩
  loadPath: '/locales/{{locale}}/{{namespace}}.json',

  // 성능 최적화
  lazy: true,
  preload: ['en', 'ko'],

  // 캐시 설정
  cache: {
    enabled: true,
    storage: 'localStorage',
    maxAge: 3600000,  // 1 hour
    version: '1.0.0'
  },

  // 디버그 모드
  debug: process.env.NODE_ENV === 'development',

  // 누락 키 처리
  saveMissing: true,
  missingKeyHandler: (locale, namespace, key) => {
    console.warn(`Missing translation: ${locale}:${namespace}:${key}`);
    // 서버에 누락 키 보고
    reportMissingKey({ locale, namespace, key });
  }
});

// 또는 팩토리 함수
const i18n = WIA.Lang({
  defaultLocale: 'ko',
  supportedLocales: ['ko', 'en'],
  loadPath: '/i18n/{{locale}}.json'
});
```

### 5.2.2 기본 번역 API

```typescript
// WIA-LANG-006 Translation API
class WIATranslationAPI {
  /**
   * 기본 번역
   * @param key - 번역 키
   * @param params - 인터폴레이션 매개변수
   * @returns 번역된 문자열
   */
  t(key: string, params?: TranslationParams): string;

  /**
   * 복수형 번역
   * @param key - 번역 키
   * @param count - 개수
   * @param params - 추가 매개변수
   */
  t(key: string, count: number, params?: TranslationParams): string;

  /**
   * 성별 변형 번역
   * @param key - 번역 키
   * @param gender - 성별
   * @param params - 추가 매개변수
   */
  t(key: string, gender: Gender, params?: TranslationParams): string;

  /**
   * 번역 존재 확인
   * @param key - 번역 키
   * @returns 존재 여부
   */
  exists(key: string): boolean;

  /**
   * 현재 언어 가져오기
   */
  getLocale(): string;

  /**
   * 언어 변경
   * @param locale - 새 언어 코드
   */
  setLocale(locale: string): Promise<void>;

  /**
   * 지원 언어 목록
   */
  getSupportedLocales(): string[];

  /**
   * 언어 자동 감지
   */
  detectLocale(): string;
}

// 사용 예제
const greeting = i18n.t('common.greeting');
// => "안녕하세요"

const welcome = i18n.t('common.welcome', { name: '홍길동' });
// => "환영합니다, 홍길동님"

const items = i18n.t('cart.items', { count: 5 });
// => "5개 항목"

const message = i18n.t('user.joined', {
  gender: 'male',
  name: 'John'
});
// => "John님이 가입했습니다"

// 번역 존재 확인
if (i18n.exists('admin.feature')) {
  console.log(i18n.t('admin.feature'));
}

// 언어 변경
await i18n.setLocale('ja');
console.log(i18n.t('common.greeting'));
// => "こんにちは"
```

### 5.2.3 형식화 API

```typescript
// WIA-LANG-006 Formatting API
class WIAFormattingAPI {
  /**
   * 날짜 형식화
   */
  formatDate(
    date: Date | string | number,
    format?: DateFormat | string,
    locale?: string
  ): string;

  /**
   * 시간 형식화
   */
  formatTime(
    time: Date | string | number,
    format?: TimeFormat | string,
    locale?: string
  ): string;

  /**
   * 날짜시간 형식화
   */
  formatDateTime(
    datetime: Date | string | number,
    format?: DateTimeFormat,
    locale?: string
  ): string;

  /**
   * 상대 시간 형식화
   */
  formatRelativeTime(
    date: Date | string | number,
    locale?: string
  ): string;

  /**
   * 숫자 형식화
   */
  formatNumber(
    value: number,
    options?: NumberFormatOptions,
    locale?: string
  ): string;

  /**
   * 통화 형식화
   */
  formatCurrency(
    amount: number,
    currency: string,
    options?: CurrencyFormatOptions,
    locale?: string
  ): string;

  /**
   * 백분율 형식화
   */
  formatPercent(
    value: number,
    decimals?: number,
    locale?: string
  ): string;

  /**
   * 목록 형식화
   */
  formatList(
    items: string[],
    type?: 'conjunction' | 'disjunction',
    locale?: string
  ): string;

  /**
   * 단위 형식화
   */
  formatUnit(
    value: number,
    unit: string,
    style?: 'long' | 'short' | 'narrow',
    locale?: string
  ): string;
}

// 사용 예제
const formatted = {
  // 날짜
  date: i18n.formatDate(new Date(), 'long'),
  // => "2025년 12월 29일"

  // 시간
  time: i18n.formatTime(new Date(), 'short'),
  // => "오후 3:30"

  // 상대 시간
  relative: i18n.formatRelativeTime(new Date('2025-12-28')),
  // => "1일 전"

  // 숫자
  number: i18n.formatNumber(1234567.89),
  // => "1,234,567.89"

  // 통화
  currency: i18n.formatCurrency(50000, 'KRW'),
  // => "₩50,000"

  // 백분율
  percent: i18n.formatPercent(0.855),
  // => "85.5%"

  // 목록
  list: i18n.formatList(['사과', '바나나', '오렌지'], 'conjunction'),
  // => "사과, 바나나 및 오렌지"

  // 단위
  unit: i18n.formatUnit(100, 'kilometer', 'long'),
  // => "100킬로미터"
};
```

## 5.3 프레임워크 통합 API

### 5.3.1 React

```typescript
import { createWIALang } from '@wia/lang-006';
import { WIALangProvider, useWIALang, Trans } from '@wia/lang-006-react';

// 1. Provider 설정
function App() {
  const i18n = createWIALang({
    defaultLocale: 'ko',
    supportedLocales: ['ko', 'en', 'ja'],
    loadPath: '/locales/{{locale}}/{{namespace}}.json'
  });

  return (
    <WIALangProvider i18n={i18n}>
      <MyApp />
    </WIALangProvider>
  );
}

// 2. useWIALang Hook
function MyComponent() {
  const {
    t,                    // 번역 함수
    locale,               // 현재 언어
    setLocale,            // 언어 변경
    formatDate,           // 날짜 형식
    formatNumber,         // 숫자 형식
    formatCurrency,       // 통화 형식
    ready                 // 로딩 완료 여부
  } = useWIALang();

  if (!ready) {
    return <Loading />;
  }

  return (
    <div>
      <h1>{t('welcome.title')}</h1>
      <p>{t('welcome.message', { name: 'User' })}</p>

      <p>{formatDate(new Date(), 'long')}</p>
      <p>{formatCurrency(99.99, 'USD')}</p>

      <select value={locale} onChange={(e) => setLocale(e.target.value)}>
        <option value="ko">한국어</option>
        <option value="en">English</option>
        <option value="ja">日本語</option>
      </select>
    </div>
  );
}

// 3. Trans 컴포넌트 (복잡한 번역)
function ComplexTranslation() {
  return (
    <Trans
      i18nKey="agreement.text"
      values={{ email: 'user@example.com' }}
      components={{
        bold: <strong />,
        link: <a href="/terms" />
      }}
    />
  );
}
// 번역: "I agree to the <bold>terms</bold> and will receive emails at <bold>{{email}}</bold>"
// 결과: "I agree to the <strong>terms</strong> and will receive emails at <strong>user@example.com</strong>"

// 4. useTranslation Hook (네임스페이스 지정)
function NamespacedComponent() {
  const { t } = useWIALang('admin');  // 'admin' 네임스페이스

  return (
    <div>
      <h1>{t('dashboard.title')}</h1>
      {/* admin:dashboard.title에서 번역 */}
    </div>
  );
}

// 5. HOC (Higher-Order Component)
import { withWIALang } from '@wia/lang-006-react';

class LegacyComponent extends React.Component {
  render() {
    const { t, locale } = this.props.i18n;
    return <div>{t('legacy.message')}</div>;
  }
}

export default withWIALang()(LegacyComponent);
```

### 5.3.2 Vue

```vue
<script setup>
import { useI18n } from '@wia/lang-006-vue';

const {
  t,
  locale,
  setLocale,
  formatDate,
  formatCurrency
} = useI18n();

const items = ref([]);

const changeLanguage = (lang) => {
  setLocale(lang);
};
</script>

<template>
  <div>
    <h1>{{ t('welcome.title') }}</h1>
    <p>{{ t('welcome.message', { name: 'User' }) }}</p>

    <!-- 복수형 -->
    <p>{{ t('cart.items', { count: items.length }) }}</p>

    <!-- 형식화 -->
    <time>{{ formatDate(new Date(), 'long') }}</time>
    <span>{{ formatCurrency(99.99, 'USD') }}</span>

    <!-- 언어 선택 -->
    <select :value="locale" @change="changeLanguage($event.target.value)">
      <option value="ko">한국어</option>
      <option value="en">English</option>
      <option value="ja">日本語</option>
    </select>

    <!-- Trans 컴포넌트 -->
    <i18n-t keypath="agreement.text" tag="p">
      <template #bold>
        <strong>{{ t('terms') }}</strong>
      </template>
    </i18n-t>
  </div>
</template>

<!-- Options API -->
<script>
export default {
  methods: {
    translate(key, params) {
      return this.$t(key, params);
    }
  }
};
</script>
```

### 5.3.3 Angular

```typescript
import { Component } from '@angular/core';
import { WIALangService } from '@wia/lang-006-angular';

@Component({
  selector: 'app-root',
  template: `
    <div>
      <h1>{{ 'welcome.title' | translate }}</h1>
      <p>{{ 'welcome.message' | translate:{ name: 'User' } }}</p>

      <!-- 복수형 -->
      <p>{{ 'cart.items' | translate:{ count: items.length } }}</p>

      <!-- 형식화 -->
      <time>{{ currentDate | formatDate:'long' }}</time>
      <span>{{ price | formatCurrency:'USD' }}</span>

      <!-- 언어 선택 -->
      <select [(ngModel)]="currentLocale" (change)="changeLanguage()">
        <option value="ko">한국어</option>
        <option value="en">English</option>
        <option value="ja">日本語</option>
      </select>
    </div>
  `
})
export class AppComponent {
  currentLocale: string;
  currentDate = new Date();
  price = 99.99;
  items = [];

  constructor(private i18n: WIALangService) {
    this.currentLocale = i18n.getLocale();
  }

  async changeLanguage() {
    await this.i18n.setLocale(this.currentLocale);
  }

  // 프로그래밍 방식
  translate(key: string, params?: any): string {
    return this.i18n.t(key, params);
  }
}

// Module 설정
import { NgModule } from '@angular/core';
import { WIALangModule } from '@wia/lang-006-angular';

@NgModule({
  imports: [
    WIALangModule.forRoot({
      defaultLocale: 'ko',
      supportedLocales: ['ko', 'en', 'ja'],
      loadPath: '/assets/i18n/{{locale}}/{{namespace}}.json'
    })
  ]
})
export class AppModule {}
```

### 5.3.4 Svelte

```svelte
<script>
import { wiaLang } from '@wia/lang-006-svelte';

const { t, locale, setLocale, formatDate, formatCurrency } = wiaLang();

let items = [];

function changeLanguage(lang) {
  setLocale(lang);
}
</script>

<div>
  <h1>{$t('welcome.title')}</h1>
  <p>{$t('welcome.message', { name: 'User' })}</p>

  <!-- 복수형 -->
  <p>{$t('cart.items', { count: items.length })}</p>

  <!-- 형식화 -->
  <time>{$formatDate(new Date(), 'long')}</time>
  <span>{$formatCurrency(99.99, 'USD')}</span>

  <!-- 언어 선택 -->
  <select bind:value={$locale} on:change={() => changeLanguage($locale)}>
    <option value="ko">한국어</option>
    <option value="en">English</option>
    <option value="ja">日本語</option>
  </select>
</div>
```

## 5.4 서버 사이드 API

### 5.4.1 Node.js

```typescript
import { createWIALang } from '@wia/lang-006';
import express from 'express';

const app = express();

// i18n 인스턴스 생성
const i18n = createWIALang({
  defaultLocale: 'en',
  supportedLocales: ['en', 'ko', 'ja', 'zh'],
  resources: {
    en: { translation: require('./locales/en.json') },
    ko: { translation: require('./locales/ko.json') }
  }
});

// 미들웨어
app.use((req, res, next) => {
  // Accept-Language 헤더에서 언어 감지
  const locale = req.acceptsLanguages(i18n.getSupportedLocales()) || 'en';

  // 요청별 i18n 인스턴스 생성
  req.i18n = i18n.cloneInstance({ locale });

  next();
});

// 라우트
app.get('/api/welcome', (req, res) => {
  res.json({
    message: req.i18n.t('welcome.message', { name: req.query.name })
  });
});

app.get('/api/product/:id', async (req, res) => {
  const product = await getProduct(req.params.id);

  res.json({
    name: req.i18n.t(`products.${product.id}.name`),
    description: req.i18n.t(`products.${product.id}.description`),
    price: req.i18n.formatCurrency(product.price, product.currency),
    availability: req.i18n.t('product.in_stock', { count: product.stock })
  });
});

// 이메일 템플릿
async function sendWelcomeEmail(user: User) {
  const i18n = createWIALang({ locale: user.locale });

  const emailContent = {
    subject: i18n.t('email.welcome.subject'),
    body: i18n.t('email.welcome.body', {
      name: user.name,
      date: i18n.formatDate(new Date(), 'long')
    })
  };

  await sendEmail(user.email, emailContent);
}
```

### 5.4.2 Next.js

```typescript
// pages/_app.tsx
import { WIALangProvider } from '@wia/lang-006-react';
import { createWIALang } from '@wia/lang-006';

function MyApp({ Component, pageProps }) {
  const i18n = createWIALang({
    defaultLocale: pageProps.locale || 'en',
    supportedLocales: ['en', 'ko', 'ja'],
    resources: pageProps.translations
  });

  return (
    <WIALangProvider i18n={i18n}>
      <Component {...pageProps} />
    </WIALangProvider>
  );
}

// pages/index.tsx
import { GetServerSideProps } from 'next';
import { useWIALang } from '@wia/lang-006-react';

export default function Home() {
  const { t } = useWIALang();

  return (
    <div>
      <h1>{t('home.title')}</h1>
      <p>{t('home.description')}</p>
    </div>
  );
}

export const getServerSideProps: GetServerSideProps = async ({ locale }) => {
  // 서버에서 번역 로드
  const translations = await loadTranslations(locale);

  return {
    props: {
      locale,
      translations
    }
  };
};
```

## 5.5 REST API 엔드포인트

### 5.5.1 Translation API

```typescript
/**
 * WIA-LANG-006 REST API Specification
 */

// GET /api/v1/locales
// 지원 언어 목록 조회
interface LocalesResponse {
  locales: Locale[];
  total: number;
}

interface Locale {
  code: string;              // 'ko-KR'
  name: string;              // '한국어'
  englishName: string;       // 'Korean'
  nativeName: string;        // '한국어'
  direction: 'ltr' | 'rtl';
  script: string;            // 'Hang'
  region: string;            // 'KR'
  completeness: number;      // 0-100
  status: 'active' | 'beta' | 'deprecated';
}

// GET /api/v1/translations/{locale}
// 특정 언어의 모든 번역 조회
interface TranslationsResponse {
  locale: string;
  namespace: string;
  translations: Record<string, any>;
  metadata: Metadata;
}

// GET /api/v1/translations/{locale}/{namespace}
// 네임스페이스별 번역 조회
interface NamespaceTranslationsResponse {
  locale: string;
  namespace: string;
  translations: Record<string, any>;
  metadata: Metadata;
}

// GET /api/v1/translations/{locale}/{namespace}/{key}
// 특정 키의 번역 조회
interface TranslationResponse {
  locale: string;
  namespace: string;
  key: string;
  value: string | object;
  metadata: {
    createdAt: string;
    updatedAt: string;
    translator: string;
    status: string;
  };
}

// POST /api/v1/translate
// 번역 요청
interface TranslateRequest {
  key: string;
  locale: string;
  namespace?: string;
  params?: Record<string, any>;
  count?: number;
  gender?: 'male' | 'female' | 'other';
}

interface TranslateResponse {
  key: string;
  locale: string;
  translated: string;
  params: Record<string, any>;
}

// POST /api/v1/translate/batch
// 일괄 번역 요청
interface BatchTranslateRequest {
  keys: string[];
  locale: string;
  namespace?: string;
  params?: Record<string, any>;
}

interface BatchTranslateResponse {
  locale: string;
  translations: Record<string, string>;
  errors: Array<{ key: string; error: string }>;
}

// POST /api/v1/format/date
// 날짜 형식화
interface FormatDateRequest {
  date: string | number;
  locale: string;
  format?: 'short' | 'medium' | 'long' | 'full' | string;
}

interface FormatDateResponse {
  formatted: string;
  locale: string;
  format: string;
}

// POST /api/v1/format/currency
// 통화 형식화
interface FormatCurrencyRequest {
  amount: number;
  currency: string;
  locale: string;
  display?: 'symbol' | 'code' | 'name';
}

interface FormatCurrencyResponse {
  formatted: string;
  amount: number;
  currency: string;
  locale: string;
}

// PUT /api/v1/translations/{locale}/{namespace}/{key}
// 번역 업데이트
interface UpdateTranslationRequest {
  value: string | object;
  translator?: string;
  notes?: string;
}

interface UpdateTranslationResponse {
  success: boolean;
  translation: Translation;
}

// DELETE /api/v1/translations/{locale}/{namespace}/{key}
// 번역 삭제
interface DeleteTranslationResponse {
  success: boolean;
  deletedKey: string;
}
```

### 5.5.2 Express 구현 예제

```typescript
import express from 'express';
import { WIALangService } from '@wia/lang-006';

const router = express.Router();
const i18nService = new WIALangService();

// GET /api/v1/locales
router.get('/locales', async (req, res) => {
  try {
    const locales = await i18nService.getLocales();
    res.json({
      locales,
      total: locales.length
    });
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

// GET /api/v1/translations/:locale
router.get('/translations/:locale', async (req, res) => {
  try {
    const { locale } = req.params;
    const { namespace = 'translation' } = req.query;

    const translations = await i18nService.getTranslations(locale, namespace);

    res.json({
      locale,
      namespace,
      translations,
      metadata: {
        count: Object.keys(translations).length,
        lastUpdated: new Date().toISOString()
      }
    });
  } catch (error) {
    res.status(404).json({ error: 'Locale not found' });
  }
});

// POST /api/v1/translate
router.post('/translate', async (req, res) => {
  try {
    const { key, locale, namespace, params, count, gender } = req.body;

    if (!key || !locale) {
      return res.status(400).json({ error: 'key and locale are required' });
    }

    const i18n = i18nService.getInstance(locale);

    let translated: string;
    if (count !== undefined) {
      translated = i18n.t(key, { count, ...params });
    } else if (gender) {
      translated = i18n.t(key, { gender, ...params });
    } else {
      translated = i18n.t(key, params);
    }

    res.json({
      key,
      locale,
      translated,
      params
    });
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

// POST /api/v1/translate/batch
router.post('/translate/batch', async (req, res) => {
  try {
    const { keys, locale, namespace, params } = req.body;

    if (!keys || !Array.isArray(keys) || !locale) {
      return res.status(400).json({
        error: 'keys (array) and locale are required'
      });
    }

    const i18n = i18nService.getInstance(locale);
    const translations: Record<string, string> = {};
    const errors: Array<{ key: string; error: string }> = [];

    for (const key of keys) {
      try {
        translations[key] = i18n.t(key, params);
      } catch (error) {
        errors.push({ key, error: error.message });
      }
    }

    res.json({
      locale,
      translations,
      errors
    });
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

// POST /api/v1/format/date
router.post('/format/date', async (req, res) => {
  try {
    const { date, locale, format = 'medium' } = req.body;

    if (!date || !locale) {
      return res.status(400).json({ error: 'date and locale are required' });
    }

    const i18n = i18nService.getInstance(locale);
    const formatted = i18n.formatDate(new Date(date), format);

    res.json({
      formatted,
      locale,
      format
    });
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

// POST /api/v1/format/currency
router.post('/format/currency', async (req, res) => {
  try {
    const { amount, currency, locale, display = 'symbol' } = req.body;

    if (amount === undefined || !currency || !locale) {
      return res.status(400).json({
        error: 'amount, currency, and locale are required'
      });
    }

    const i18n = i18nService.getInstance(locale);
    const formatted = i18n.formatCurrency(amount, currency, { display });

    res.json({
      formatted,
      amount,
      currency,
      locale
    });
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

export default router;
```

## 5.6 GraphQL API

### 5.6.1 스키마 정의

```graphql
# WIA-LANG-006 GraphQL Schema

type Query {
  """지원 언어 목록 조회"""
  locales(
    status: LocaleStatus
    search: String
  ): [Locale!]!

  """특정 언어 정보"""
  locale(code: String!): Locale

  """번역 조회"""
  translation(
    locale: String!
    key: String!
    namespace: String
  ): Translation

  """여러 번역 조회"""
  translations(
    locale: String!
    keys: [String!]
    namespace: String
    includeMetadata: Boolean
  ): [Translation!]!

  """번역 검색"""
  searchTranslations(
    query: String!
    locale: String
    namespace: String
    fuzzy: Boolean
    limit: Int
  ): SearchResult!

  """누락된 번역 찾기"""
  missingTranslations(
    baseLocale: String!
    targetLocale: String!
    namespace: String
  ): [String!]!

  """번역 통계"""
  translationStats(
    locale: String!
    namespace: String
  ): TranslationStats!
}

type Mutation {
  """번역 생성"""
  createTranslation(
    input: CreateTranslationInput!
  ): Translation!

  """번역 업데이트"""
  updateTranslation(
    input: UpdateTranslationInput!
  ): Translation!

  """번역 삭제"""
  deleteTranslation(
    locale: String!
    key: String!
    namespace: String
  ): Boolean!

  """번역 일괄 가져오기"""
  importTranslations(
    input: ImportTranslationsInput!
  ): ImportResult!

  """번역 승인"""
  approveTranslation(
    locale: String!
    key: String!
    namespace: String
  ): Translation!
}

type Subscription {
  """번역 변경 감지"""
  translationChanged(
    locale: String!
    namespace: String
  ): TranslationEvent!

  """번역 생성 감지"""
  translationCreated(
    locale: String!
  ): Translation!

  """번역 삭제 감지"""
  translationDeleted(
    locale: String!
  ): String!
}

type Locale {
  code: String!
  name: String!
  englishName: String!
  nativeName: String!
  direction: TextDirection!
  script: String!
  region: String
  pluralRules: [PluralRule!]!
  completeness: Float!
  status: LocaleStatus!
  lastUpdated: DateTime!
}

type Translation {
  key: String!
  value: TranslationValue!
  locale: String!
  namespace: String!
  context: String
  metadata: TranslationMetadata
}

union TranslationValue =
  | SimpleTranslation
  | PluralTranslation
  | GenderTranslation
  | ComplexTranslation

type SimpleTranslation {
  text: String!
}

type PluralTranslation {
  zero: String
  one: String
  two: String
  few: String
  many: String
  other: String!
}

type GenderTranslation {
  male: String!
  female: String!
  other: String
}

type ComplexTranslation {
  rules: [ComplexRule!]!
}

type ComplexRule {
  gender: Gender!
  plural: PluralTranslation!
}

type TranslationMetadata {
  createdAt: DateTime!
  updatedAt: DateTime!
  createdBy: String
  updatedBy: String
  translator: String
  reviewer: String
  status: TranslationStatus!
  quality: Int
  notes: String
}

type TranslationStats {
  total: Int!
  translated: Int!
  missing: Int!
  outdated: Int!
  completeness: Float!
  lastUpdated: DateTime!
}

type SearchResult {
  results: [Translation!]!
  total: Int!
  page: Int!
  limit: Int!
}

type TranslationEvent {
  action: EventAction!
  translation: Translation!
  timestamp: DateTime!
}

type ImportResult {
  success: Boolean!
  imported: Int!
  updated: Int!
  errors: [ImportError!]!
}

type ImportError {
  key: String!
  error: String!
}

input CreateTranslationInput {
  locale: String!
  key: String!
  value: String!
  namespace: String
  context: String
  notes: String
}

input UpdateTranslationInput {
  locale: String!
  key: String!
  value: String!
  namespace: String
  notes: String
}

input ImportTranslationsInput {
  locale: String!
  format: ImportFormat!
  data: String!
  namespace: String
  overwrite: Boolean
}

enum TextDirection {
  LTR
  RTL
}

enum PluralRule {
  ZERO
  ONE
  TWO
  FEW
  MANY
  OTHER
}

enum Gender {
  MALE
  FEMALE
  OTHER
}

enum LocaleStatus {
  ACTIVE
  BETA
  DEPRECATED
}

enum TranslationStatus {
  DRAFT
  REVIEW
  APPROVED
  PUBLISHED
  OUTDATED
}

enum EventAction {
  CREATED
  UPDATED
  DELETED
  APPROVED
}

enum ImportFormat {
  JSON
  YAML
  XLIFF
  PO
  CSV
}

scalar DateTime
```

### 5.6.2 Resolver 구현

```typescript
import { GraphQLResolvers } from './generated/graphql';
import { WIALangService } from '@wia/lang-006';

const i18nService = new WIALangService();

export const resolvers: GraphQLResolvers = {
  Query: {
    locales: async (_, { status, search }) => {
      const locales = await i18nService.getLocales();

      let filtered = locales;

      if (status) {
        filtered = filtered.filter(l => l.status === status);
      }

      if (search) {
        const searchLower = search.toLowerCase();
        filtered = filtered.filter(l =>
          l.name.toLowerCase().includes(searchLower) ||
          l.englishName.toLowerCase().includes(searchLower) ||
          l.code.toLowerCase().includes(searchLower)
        );
      }

      return filtered;
    },

    locale: async (_, { code }) => {
      return await i18nService.getLocale(code);
    },

    translation: async (_, { locale, key, namespace }) => {
      return await i18nService.getTranslation(locale, key, namespace);
    },

    translations: async (_, { locale, keys, namespace }) => {
      return await i18nService.getTranslations(locale, keys, namespace);
    },

    searchTranslations: async (_, { query, locale, namespace, fuzzy, limit }) => {
      return await i18nService.searchTranslations({
        query,
        locale,
        namespace,
        fuzzy,
        limit
      });
    },

    missingTranslations: async (_, { baseLocale, targetLocale, namespace }) => {
      return await i18nService.findMissingKeys(
        baseLocale,
        targetLocale,
        namespace
      );
    },

    translationStats: async (_, { locale, namespace }) => {
      return await i18nService.getStats(locale, namespace);
    }
  },

  Mutation: {
    createTranslation: async (_, { input }) => {
      return await i18nService.createTranslation(input);
    },

    updateTranslation: async (_, { input }) => {
      return await i18nService.updateTranslation(input);
    },

    deleteTranslation: async (_, { locale, key, namespace }) => {
      await i18nService.deleteTranslation(locale, key, namespace);
      return true;
    },

    importTranslations: async (_, { input }) => {
      return await i18nService.importTranslations(input);
    },

    approveTranslation: async (_, { locale, key, namespace }) => {
      return await i18nService.approveTranslation(locale, key, namespace);
    }
  },

  Subscription: {
    translationChanged: {
      subscribe: (_, { locale, namespace }) => {
        return i18nService.subscribeToChanges(locale, namespace);
      }
    },

    translationCreated: {
      subscribe: (_, { locale }) => {
        return i18nService.subscribeToCreation(locale);
      }
    },

    translationDeleted: {
      subscribe: (_, { locale }) => {
        return i18nService.subscribeToDeletion(locale);
      }
    }
  },

  TranslationValue: {
    __resolveType(obj) {
      if (typeof obj === 'string') {
        return 'SimpleTranslation';
      }
      if (obj.$type === 'plural') {
        return 'PluralTranslation';
      }
      if (obj.$type === 'gender') {
        return 'GenderTranslation';
      }
      if (obj.$type === 'complex') {
        return 'ComplexTranslation';
      }
      return null;
    }
  }
};
```

---

## 요약

WIA-LANG-006 API 인터페이스는 다음을 제공합니다:

1. **통일된 Core API**: 모든 환경에서 일관된 인터페이스
2. **프레임워크 통합**: React, Vue, Angular, Svelte 지원
3. **서버 사이드**: Node.js, Next.js, Express 등
4. **REST API**: 표준 HTTP 엔드포인트
5. **GraphQL API**: 강력한 쿼리 및 실시간 구독
6. **타입 안전**: TypeScript 우선 설계

**다음 장**에서는 통신 프로토콜과 동기화 메커니즘을 살펴보겠습니다.

---

**© 2025 World Industry Association**
**弘益人間 (홍익인간) · Benefit All Humanity**

*이 문서는 WIA-LANG-006 Multilingual Interface 표준의 일부입니다.*
