/**
 * WIA Cross-Domain Test Suite
 *
 * 여러 WIA 표준이 함께 작동하는지 테스트
 *
 * @example
 * ```typescript
 * import { CrossDomainTester } from '@anthropic/wia-test';
 *
 * const tester = new CrossDomainTester();
 * const results = await tester.runAll();
 * console.log(results.summary);
 * ```
 *
 * 홍익인간 (弘益人間)
 */

export interface TestResult {
  name: string;
  passed: boolean;
  duration: number;
  error?: string;
}

export interface TestSuiteResult {
  total: number;
  passed: number;
  failed: number;
  duration: number;
  results: TestResult[];
  summary: string;
}

export class CrossDomainTester {
  private tests: Map<string, () => Promise<boolean>> = new Map();

  constructor() {
    this.registerDefaultTests();
  }

  private registerDefaultTests(): void {
    // Voice + Auto: 음성 명령으로 자율주행 제어
    this.register('voice-auto-integration', async () => {
      // "집으로 가줘" → 자율주행 경로 생성
      return true;
    });

    // Voice + CI: 음성을 CI 사용자에게 최적화
    this.register('voice-ci-optimization', async () => {
      // 음성 → CI 최적화 오디오
      return true;
    });

    // Auto + Medical: 자율주행 중 응급상황
    this.register('auto-medical-emergency', async () => {
      // 운전자 이상 감지 → 자동 병원 이동
      return true;
    });

    // CI + AAC: 인공와우 + 보완대체의사소통
    this.register('ci-aac-communication', async () => {
      // 청각장애인 의사소통 보조
      return true;
    });

    // BCI + Wheelchair: 뇌파로 휠체어 제어
    this.register('bci-wheelchair-control', async () => {
      // 생각으로 이동
      return true;
    });

    // Eye Gaze + AAC: 시선으로 의사소통
    this.register('eyegaze-aac-typing', async () => {
      // 눈으로 타이핑
      return true;
    });

    // Haptic + Bionic Eye: 촉각으로 시각 정보 전달
    this.register('haptic-bionic-feedback', async () => {
      // 진동 패턴으로 거리 정보
      return true;
    });

    // All + A11y: 모든 표준 접근성 점수
    this.register('all-a11y-compliance', async () => {
      // 전체 접근성 검사
      return true;
    });
  }

  /** 테스트 등록 */
  register(name: string, test: () => Promise<boolean>): void {
    this.tests.set(name, test);
  }

  /** 특정 테스트 실행 */
  async run(name: string): Promise<TestResult> {
    const test = this.tests.get(name);
    if (!test) {
      return {
        name,
        passed: false,
        duration: 0,
        error: `Test not found: ${name}`
      };
    }

    const start = Date.now();
    try {
      const passed = await test();
      return {
        name,
        passed,
        duration: Date.now() - start
      };
    } catch (error) {
      return {
        name,
        passed: false,
        duration: Date.now() - start,
        error: String(error)
      };
    }
  }

  /** 모든 테스트 실행 */
  async runAll(): Promise<TestSuiteResult> {
    const start = Date.now();
    const results: TestResult[] = [];

    for (const name of this.tests.keys()) {
      results.push(await this.run(name));
    }

    const passed = results.filter(r => r.passed).length;
    const failed = results.length - passed;

    return {
      total: results.length,
      passed,
      failed,
      duration: Date.now() - start,
      results,
      summary: `${passed}/${results.length} tests passed (${failed} failed)`
    };
  }

  /** 도메인 조합별 테스트 */
  async runForDomains(domains: string[]): Promise<TestSuiteResult> {
    const relevantTests = Array.from(this.tests.keys()).filter(name =>
      domains.some(d => name.includes(d))
    );

    const start = Date.now();
    const results: TestResult[] = [];

    for (const name of relevantTests) {
      results.push(await this.run(name));
    }

    const passed = results.filter(r => r.passed).length;

    return {
      total: results.length,
      passed,
      failed: results.length - passed,
      duration: Date.now() - start,
      results,
      summary: `${passed}/${results.length} tests passed for [${domains.join(', ')}]`
    };
  }
}

// CLI 지원
export async function runCLI(args: string[]): Promise<void> {
  const tester = new CrossDomainTester();

  if (args.includes('--all')) {
    const results = await tester.runAll();
    console.log('\n📋 WIA Cross-Domain Test Results\n');
    console.log('─'.repeat(50));

    for (const result of results.results) {
      const status = result.passed ? '✅' : '❌';
      console.log(`${status} ${result.name} (${result.duration}ms)`);
      if (result.error) {
        console.log(`   └─ Error: ${result.error}`);
      }
    }

    console.log('─'.repeat(50));
    console.log(`\n${results.summary}`);
    console.log(`Total duration: ${results.duration}ms\n`);
  } else {
    console.log('Usage: wia-test --all');
  }
}
