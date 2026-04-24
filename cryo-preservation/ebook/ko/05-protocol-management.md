# 프로토콜 관리

**弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

## 동결 프로토콜

### 정자 동결 프로토콜

```typescript
/**
 * 정자 완만동결 프로토콜
 * 참고: WHO Laboratory Manual for the Examination and Processing of Human Semen (6th ed., 2021)
 */
export class SpermSlowFreezingProtocol {
  /**
   * 프로토콜 정보
   */
  private protocolInfo = {
    protocolId: 'PROTO-SPERM-SF-001',
    name: 'Standard Sperm Slow Freezing Protocol',
    nameKr: '표준 정자 완만동결 프로토콜',
    version: '2.1.0',
    approvedDate: '2024-01-15',
    specimenType: 'SPERM',
  };

  /**
   * 동결보호제 준비
   */
  async prepareCryoprotectant(): Promise<{
    composition: any[];
    preparation: string[];
  }> {
    return {
      composition: [
        {
          component: 'Glycerol',
          componentKr: '글리세롤',
          concentration: 10,
          purpose: '세포내 빙결정 형성 방지',
        },
        {
          component: 'Egg yolk buffer',
          componentKr: '난황 완충액',
          concentration: 20,
          purpose: '세포막 보호',
        },
        {
          component: 'TEST buffer',
          componentKr: 'TEST 완충액',
          concentration: 70,
          purpose: '삼투압 유지',
        },
      ],
      preparation: [
        '1. TEST 완충액 준비 (Tris 3.03g, Citric acid 1.67g, Fructose 1.25g in 100ml DW)',
        '2. 난황 20ml를 TEST 완충액 80ml에 첨가',
        '3. 글리세롤 10ml 첨가하여 총 110ml 제조',
        '4. 0.22μm 필터로 멸균 여과',
        '5. 4°C에서 보관 (최대 2주)',
      ],
    };
  }

  /**
   * 동결 단계 실행
   */
  async execute(specimenId: string): Promise<{
    success: boolean;
    steps: any[];
    finalViability: number;
  }> {
    const steps = [
      {
        stepNumber: 1,
        name: 'Semen Analysis',
        nameKr: '정액 분석',
        duration: 30,
        action: '정액 검사를 수행하여 기본 파라미터 확인',
        details: {
          volume: '측정',
          concentration: '측정 (million/ml)',
          motility: '측정 (%)',
          morphology: '평가',
        },
        result: null,
      },
      {
        stepNumber: 2,
        name: 'Liquefaction',
        nameKr: '액화',
        duration: 30,
        temperature: 37,
        action: '정액을 37°C에서 30분간 액화',
        notes: '완전 액화 확인 필요',
      },
      {
        stepNumber: 3,
        name: 'Washing',
        nameKr: '세척',
        duration: 20,
        action: '정자 세척 및 농축',
        procedure: [
          '1. 정액과 동량의 배양액 혼합',
          '2. 300g에서 10분간 원심분리',
          '3. 상층액 제거',
          '4. 펠렛을 배양액으로 재현탁',
          '5. 1회 더 세척 반복',
        ],
      },
      {
        stepNumber: 4,
        name: 'Cryoprotectant Addition',
        nameKr: '동결보호제 첨가',
        duration: 10,
        temperature: 20,
        action: '동결보호제를 점진적으로 첨가',
        procedure: [
          '1. 세척된 정자 펠렛에 소량의 배양액 첨가',
          '2. 농도 조정 (50-100 million/ml)',
          '3. 동결보호제를 1:1 비율로 서서히 첨가',
          '4. 실온에서 10분간 평형',
        ],
        equilibrationTime: 10,
      },
      {
        stepNumber: 5,
        name: 'Aliquoting',
        nameKr: '분주',
        duration: 10,
        action: '크라이오바이알에 분주',
        procedure: [
          '1. 멸균 크라이오바이알 준비',
          '2. 바코드 라벨 부착',
          '3. 0.5ml씩 분주',
          '4. 바이알 완전 밀봉 확인',
        ],
      },
      {
        stepNumber: 6,
        name: 'Vapor Phase Cooling',
        nameKr: '기상 냉각',
        duration: 10,
        temperature: -80,
        action: '액체질소 기상에서 냉각',
        procedure: [
          '1. 크라이오바이알을 액체질소 탱크 위 10cm에 위치',
          '2. 10분간 기상 냉각 (약 -1°C/min)',
          '3. 온도가 약 -80°C 도달 확인',
        ],
        coolingRate: -1,
      },
      {
        stepNumber: 7,
        name: 'Immersion in Liquid Nitrogen',
        nameKr: '액체질소 침지',
        duration: 1,
        temperature: -196,
        action: '액체질소에 완전 침지',
        finalTemperature: -196,
      },
      {
        stepNumber: 8,
        name: 'Storage',
        nameKr: '보관',
        action: '장기 보관 위치로 이동',
        procedure: [
          '1. 캐니스터에 바이알 적재',
          '2. 보관 위치 기록',
          '3. 데이터베이스 업데이트',
        ],
      },
    ];

    // 단계별 실행
    for (const step of steps) {
      console.log(`[단계 ${step.stepNumber}] ${step.nameKr} 진행 중...`);
      await this.performStep(step);
      console.log(`[단계 ${step.stepNumber}] 완료`);
    }

    return {
      success: true,
      steps,
      finalViability: 85, // 예상 생존율
    };
  }

  /**
   * 품질 관리
   */
  async performQualityControl(specimenId: string): Promise<{
    preFreeze: any;
    postThaw: any;
    recoveryRate: number;
  }> {
    return {
      preFreeze: {
        concentration: 80, // million/ml
        motility: 60, // %
        viability: 85, // %
        morphology: 'Normal: 8%',
      },
      postThaw: {
        concentration: 75, // million/ml
        motility: 50, // %
        viability: 75, // %
        morphology: 'Normal: 7%',
      },
      recoveryRate: 88.2, // %
    };
  }

  private async performStep(step: any): Promise<void> {
    // 실제 단계 수행 로직
    await new Promise(resolve => setTimeout(resolve, 100));
  }
}

/**
 * 정자 급속동결 (유리화) 프로토콜
 */
export class SpermVitrificationProtocol {
  /**
   * 프로토콜 정보
   */
  private protocolInfo = {
    protocolId: 'PROTO-SPERM-VIT-001',
    name: 'Sperm Vitrification Protocol',
    nameKr: '정자 유리화 동결 프로토콜',
    version: '1.0.0',
    specimenType: 'SPERM',
  };

  /**
   * 유리화 동결보호제
   */
  async prepareCryoprotectant(): Promise<{
    baseMedia: any;
    vitrificationSolution: any;
  }> {
    return {
      baseMedia: {
        name: 'Sperm Washing Medium',
        nameKr: '정자 세척 배양액',
        composition: 'HEPES-buffered medium with HSA',
      },
      vitrificationSolution: {
        name: 'High Concentration CPA',
        nameKr: '고농도 동결보호제',
        components: [
          { name: 'DMSO', nameKr: '디메틸설폭사이드', concentration: 15 },
          { name: 'Ethylene Glycol', nameKr: '에틸렌글리콜', concentration: 15 },
          { name: 'Sucrose', nameKr: '자당', concentration: 0.5, unit: 'M' },
        ],
      },
    };
  }

  /**
   * 유리화 실행
   */
  async execute(specimenId: string): Promise<any> {
    const steps = [
      {
        stepNumber: 1,
        nameKr: '정자 준비',
        duration: 30,
        action: '정자 세척 및 농축',
      },
      {
        stepNumber: 2,
        nameKr: '평형 용액 처리',
        duration: 5,
        action: '7.5% DMSO + 7.5% EG 용액에 5분간 평형',
        temperature: 20,
      },
      {
        stepNumber: 3,
        nameKr: '유리화 용액 처리',
        duration: 1,
        action: '15% DMSO + 15% EG + 0.5M Sucrose 용액에 1분 노출',
        temperature: 20,
      },
      {
        stepNumber: 4,
        nameKr: '장착',
        duration: 0.5,
        action: '크라이오탑 또는 스트로에 정자 장착',
      },
      {
        stepNumber: 5,
        nameKr: '급속 냉각',
        duration: 0.1,
        action: '액체질소에 직접 침지',
        temperature: -196,
        coolingRate: -23000, // °C/min
      },
      {
        stepNumber: 6,
        nameKr: '보관',
        action: '액체질소 탱크에 보관',
      },
    ];

    return {
      success: true,
      steps,
      expectedViability: 80,
    };
  }
}
```

### 난자 유리화 프로토콜

```typescript
/**
 * 난자 유리화 프로토콜
 * Cryotop 방법
 */
export class OocyteVitrificationProtocol {
  /**
   * 프로토콜 정보
   */
  private protocolInfo = {
    protocolId: 'PROTO-OOCYTE-VIT-001',
    name: 'Oocyte Vitrification Protocol (Cryotop Method)',
    nameKr: '난자 유리화 동결 프로토콜 (크라이오탑 방법)',
    version: '3.0.0',
    approvedDate: '2024-01-10',
    specimenType: 'OOCYTE',
    applicableTo: ['MII oocytes'], // 성숙 난자만 해당
  };

  /**
   * 유리화 용액 준비
   */
  async prepareVitrificationSolutions(): Promise<{
    equilibrationSolution: any;
    vitrificationSolution: any;
  }> {
    return {
      equilibrationSolution: {
        name: 'ES (Equilibration Solution)',
        nameKr: '평형 용액',
        composition: [
          {
            component: 'Ethylene Glycol (EG)',
            componentKr: '에틸렌글리콜',
            concentration: 7.5,
            unit: '%',
          },
          {
            component: 'DMSO',
            componentKr: '디메틸설폭사이드',
            concentration: 7.5,
            unit: '%',
          },
        ],
        baseMedia: 'Quinn\'s Advantage Medium with HSA',
        osmolality: 320, // mOsm/kg
        temperature: 37,
      },
      vitrificationSolution: {
        name: 'VS (Vitrification Solution)',
        nameKr: '유리화 용액',
        composition: [
          {
            component: 'Ethylene Glycol (EG)',
            componentKr: '에틸렌글리콜',
            concentration: 15,
            unit: '%',
          },
          {
            component: 'DMSO',
            componentKr: '디메틸설폭사이드',
            concentration: 15,
            unit: '%',
          },
          {
            component: 'Sucrose',
            componentKr: '자당',
            concentration: 0.5,
            unit: 'M',
          },
        ],
        baseMedia: 'Quinn\'s Advantage Medium with HSA',
        osmolality: 2800, // mOsm/kg
        temperature: 37,
      },
    };
  }

  /**
   * 유리화 단계
   */
  async execute(oocyteIds: string[]): Promise<{
    success: boolean;
    oocytes: any[];
    protocol: any;
  }> {
    const protocol = {
      steps: [
        {
          stepNumber: 1,
          name: 'Pre-warming',
          nameKr: '용액 예열',
          duration: 10,
          temperature: 37,
          action: 'ES와 VS 용액을 37°C로 예열',
          notes: '용액 온도가 정확히 37°C인지 확인',
        },
        {
          stepNumber: 2,
          name: 'Oocyte Preparation',
          nameKr: '난자 준비',
          duration: 5,
          action: '배양액에서 난자를 회수하여 세척',
          procedure: [
            '1. 배양 접시에서 MII 난자만 선별',
            '2. 세척 배양액으로 3회 세척',
            '3. 난자 품질 평가 (투명대, 세포질, 극체)',
          ],
        },
        {
          stepNumber: 3,
          name: 'Equilibration',
          nameKr: '평형',
          duration: 13,
          temperature: 37,
          action: 'ES 용액에서 평형',
          procedure: [
            '1. 난자를 ES 용액 50μl 드롭에 이동',
            '2. 37°C에서 정확히 13분간 평형',
            '3. 타이머 사용하여 시간 엄수',
          ],
          criticalTime: true,
          maxDeviation: 30, // seconds
        },
        {
          stepNumber: 4,
          name: 'Vitrification Solution Exposure',
          nameKr: '유리화 용액 노출',
          duration: 1,
          temperature: 37,
          action: 'VS 용액에 짧은 시간 노출',
          procedure: [
            '1. VS 용액 50μl 드롭 2개 준비',
            '2. 첫 번째 드롭에 난자 이동 (30초)',
            '3. 두 번째 드롭으로 이동 (30초)',
            '4. 총 노출 시간: 정확히 60초',
          ],
          criticalTime: true,
          maxTime: 65, // seconds
        },
        {
          stepNumber: 5,
          name: 'Loading onto Cryotop',
          nameKr: '크라이오탑 장착',
          duration: 0.5,
          action: '크라이오탑에 난자 장착',
          procedure: [
            '1. 크라이오탑을 VS 용액에 적심',
            '2. 미세 피펫으로 난자를 최소량의 배양액과 함께 흡입',
            '3. 크라이오탑 필름 위에 난자 위치',
            '4. 여분의 배양액 제거 (난자 주변에만 얇은 막 남김)',
          ],
          notes: '이 단계는 매우 빠르게 수행 (30초 이내)',
        },
        {
          stepNumber: 6,
          name: 'Vitrification',
          nameKr: '급속 냉각',
          duration: 0.05,
          temperature: -196,
          action: '액체질소에 즉시 침지',
          procedure: [
            '1. 크라이오탑을 즉시 액체질소에 침지',
            '2. 냉각 속도: 약 -23,000°C/min',
            '3. 완전 침지 확인',
          ],
          coolingRate: -23000,
          criticalSpeed: true,
        },
        {
          stepNumber: 7,
          name: 'Capping',
          nameKr: '캡핑',
          duration: 0.2,
          action: '액체질소 내에서 보호 캡 장착',
          procedure: [
            '1. 액체질소에 침지된 상태로 보호 캡 장착',
            '2. 캡이 완전히 밀봉되었는지 확인',
          ],
          notes: '캡핑 시 크라이오탑이 액체질소에서 나오지 않도록 주의',
        },
        {
          stepNumber: 8,
          name: 'Storage',
          nameKr: '보관',
          action: '장기 보관 탱크로 이동',
          procedure: [
            '1. 캐니스터에 크라이오탑 적재',
            '2. 환자 정보 및 동결일 기록',
            '3. 보관 위치 데이터베이스 등록',
          ],
        },
      ],
      qualityControl: {
        preVitrification: [
          '난자 성숙도 확인 (MII만 유리화)',
          '투명대 정상 여부',
          '세포질 균질성',
          '극체 존재 확인',
        ],
        duringProcedure: [
          '용액 온도 모니터링',
          '시간 엄수',
          '무균 조작',
        ],
        expectedOutcome: {
          survivalRate: 95, // %
          fertilizationRate: 75, // % (해동 후)
        },
      },
    };

    const results = oocyteIds.map(id => ({
      oocyteId: id,
      status: 'FROZEN',
      vitrifiedAt: new Date().toISOString(),
      cryotopId: this.generateCryotopId(),
      expectedSurvival: 95,
    }));

    return {
      success: true,
      oocytes: results,
      protocol,
    };
  }

  /**
   * 온도 및 시간 모니터링
   */
  async monitorCriticalParameters(): Promise<{
    temperature: number[];
    timings: any;
    compliance: boolean;
  }> {
    return {
      temperature: [37, 37, 37, -196], // °C
      timings: {
        equilibration: 13 * 60, // seconds
        vitrificationSolution: 60, // seconds
        loading: 30, // seconds
        total: 14 * 60 + 30, // seconds
      },
      compliance: true,
    };
  }

  private generateCryotopId(): string {
    return `CT${Date.now()}${Math.random().toString(36).substr(2, 6)}`;
  }
}
```

### 배아 유리화 프로토콜

```typescript
/**
 * 배아 유리화 프로토콜
 * 포배기 배아용
 */
export class BlastocystVitrificationProtocol {
  /**
   * 프로토콜 정보
   */
  private protocolInfo = {
    protocolId: 'PROTO-EMBRYO-VIT-001',
    name: 'Blastocyst Vitrification Protocol',
    nameKr: '포배기 배아 유리화 프로토콜',
    version: '2.5.0',
    specimenType: 'EMBRYO',
    applicableTo: ['Day 5 blastocyst', 'Day 6 blastocyst'],
  };

  /**
   * 유리화 실행
   */
  async execute(embryoData: {
    embryoId: string;
    stage: string;
    grade: { expansion: number; icm: string; te: string };
    patientId: string;
  }): Promise<any> {
    // 배아 등급 평가
    const quality = this.assessBlastocystQuality(embryoData.grade);

    const steps = [
      {
        stepNumber: 1,
        nameKr: '배아 선별',
        duration: 5,
        action: '동결할 배아 선별 및 평가',
        procedure: [
          '1. 포배 팽창도 평가 (1-6등급)',
          '2. Inner Cell Mass (ICM) 평가 (A/B/C)',
          '3. Trophectoderm (TE) 평가 (A/B/C)',
          '4. 최소 등급: 3BB 이상 권장',
        ],
        result: {
          grade: `${embryoData.grade.expansion}${embryoData.grade.icm}${embryoData.grade.te}`,
          suitable: quality.suitable,
        },
      },
      {
        stepNumber: 2,
        nameKr: '인공부화 (선택사항)',
        duration: 2,
        action: '확장 포배의 경우 인공부화 수행',
        procedure: [
          '1. 레이저로 투명대에 작은 구멍 생성',
          '2. 또는 Tyrode\'s solution으로 화학적 부화',
          '3. 포배강 내 액체 감소로 유리화 성공률 향상',
        ],
        notes: '팽창도 4 이상의 배아에 권장',
        optional: embryoData.grade.expansion >= 4,
      },
      {
        stepNumber: 3,
        nameKr: '평형 (ES)',
        duration: 3,
        temperature: 37,
        action: '평형 용액에서 처리',
        solution: {
          name: 'ES',
          composition: '7.5% EG + 7.5% DMSO',
        },
      },
      {
        stepNumber: 4,
        nameKr: '유리화 (VS)',
        duration: 0.75,
        temperature: 37,
        action: '유리화 용액 노출',
        solution: {
          name: 'VS',
          composition: '15% EG + 15% DMSO + 0.5M Sucrose',
        },
        timing: {
          firstDrop: 30, // seconds
          secondDrop: 15, // seconds
          total: 45, // seconds
        },
      },
      {
        stepNumber: 5,
        nameKr: '크라이오탑 장착',
        duration: 0.5,
        action: '배아를 크라이오탑에 장착',
        procedure: [
          '1. 배아를 최소량의 배양액과 함께 흡입',
          '2. 크라이오탑 필름 위에 위치',
          '3. 여분의 액체 신속히 제거',
        ],
      },
      {
        stepNumber: 6,
        nameKr: '급속 냉각',
        duration: 0.05,
        temperature: -196,
        action: '액체질소에 침지',
        coolingRate: -23000,
      },
      {
        stepNumber: 7,
        nameKr: '보관',
        action: '장기 보관',
      },
    ];

    return {
      success: true,
      embryoId: embryoData.embryoId,
      steps,
      quality,
      expectedSurvival: this.calculateExpectedSurvival(embryoData.grade),
    };
  }

  /**
   * 포배 품질 평가
   */
  private assessBlastocystQuality(grade: {
    expansion: number;
    icm: string;
    te: string;
  }): {
    suitable: boolean;
    survivalRate: number;
    implantationPotential: number;
  } {
    let survivalRate = 90;
    let implantationPotential = 50;

    // 팽창도에 따른 조정
    if (grade.expansion >= 4) {
      survivalRate += 5;
      implantationPotential += 10;
    }

    // ICM 등급에 따른 조정
    if (grade.icm === 'A') {
      survivalRate += 3;
      implantationPotential += 15;
    } else if (grade.icm === 'C') {
      survivalRate -= 5;
      implantationPotential -= 15;
    }

    // TE 등급에 따른 조정
    if (grade.te === 'A') {
      survivalRate += 2;
      implantationPotential += 10;
    } else if (grade.te === 'C') {
      survivalRate -= 5;
      implantationPotential -= 10;
    }

    return {
      suitable: grade.expansion >= 3 && grade.icm !== 'C' && grade.te !== 'C',
      survivalRate: Math.min(98, Math.max(80, survivalRate)),
      implantationPotential: Math.min(70, Math.max(20, implantationPotential)),
    };
  }

  private calculateExpectedSurvival(grade: any): number {
    const quality = this.assessBlastocystQuality(grade);
    return quality.survivalRate;
  }
}
```

## 해동 프로토콜

### 난자 해동 프로토콜

```typescript
/**
 * 난자 해동 프로토콜
 * 유리화 난자용
 */
export class OocyteWarmingProtocol {
  /**
   * 프로토콜 정보
   */
  private protocolInfo = {
    protocolId: 'PROTO-OOCYTE-WARM-001',
    name: 'Oocyte Warming Protocol',
    nameKr: '난자 해동 프로토콜',
    version: '3.0.0',
    specimenType: 'OOCYTE',
  };

  /**
   * 해동 용액 준비
   */
  async prepareWarmingSolutions(): Promise<{
    thawingSolution: any;
    dilutionSolutions: any[];
    washingSolution: any;
  }> {
    return {
      thawingSolution: {
        name: 'TS (Thawing Solution)',
        nameKr: '해동 용액',
        composition: '1.0M Sucrose in base medium',
        temperature: 37,
        volume: 100, // μl per drop
      },
      dilutionSolutions: [
        {
          name: 'DS1',
          nameKr: '희석액 1',
          composition: '0.5M Sucrose in base medium',
          temperature: 37,
          duration: 3, // minutes
        },
        {
          name: 'DS2',
          nameKr: '희석액 2',
          composition: '0.25M Sucrose in base medium',
          temperature: 37,
          duration: 3,
        },
      ],
      washingSolution: {
        name: 'WS (Washing Solution)',
        nameKr: '세척액',
        composition: 'Base culture medium with HSA',
        temperature: 37,
        washes: 2,
      },
    };
  }

  /**
   * 해동 실행
   */
  async execute(cryotopId: string): Promise<{
    success: boolean;
    survival: boolean;
    quality: any;
  }> {
    const steps = [
      {
        stepNumber: 1,
        nameKr: '용액 준비',
        duration: 10,
        action: '모든 해동 용액을 37°C로 예열',
        procedure: [
          '1. TS, DS1, DS2, WS 용액 준비',
          '2. 각 용액을 50-100μl 드롭으로 배양 접시에 준비',
          '3. 37°C, 5% CO2 인큐베이터에서 예열',
          '4. 미네랄 오일로 덮기',
        ],
      },
      {
        stepNumber: 2,
        nameKr: '크라이오탑 회수',
        duration: 0.5,
        action: '액체질소에서 크라이오탑 회수',
        procedure: [
          '1. 보관 탱크에서 크라이오탑 꺼내기',
          '2. 액체질소에 보관하면서 캡 제거',
          '3. 필름 부분만 액체질소 밖으로 노출',
        ],
        notes: '매우 신속하게 수행 (5초 이내)',
        criticalSpeed: true,
      },
      {
        stepNumber: 3,
        nameKr: '급속 가온',
        duration: 0.1,
        temperature: 37,
        action: 'TS 용액에 즉시 침지',
        procedure: [
          '1. 크라이오탑을 37°C TS 용액에 즉시 침지',
          '2. 난자가 용액에 떨어질 때까지 저어주기 (약 10초)',
          '3. 가온 속도: 약 +20,000°C/min',
        ],
        warmingRate: 20000,
      },
      {
        stepNumber: 4,
        nameKr: 'TS에서 평형',
        duration: 1,
        temperature: 37,
        action: '해동 용액에서 1분간 유지',
        notes: '자당이 세포에서 동결보호제를 추출',
      },
      {
        stepNumber: 5,
        nameKr: '단계적 희석 - DS1',
        duration: 3,
        temperature: 37,
        action: '첫 번째 희석액으로 이동',
        procedure: [
          '1. 난자를 DS1 (0.5M Sucrose) 드롭으로 이동',
          '2. 3분간 평형',
        ],
      },
      {
        stepNumber: 6,
        nameKr: '단계적 희석 - DS2',
        duration: 3,
        temperature: 37,
        action: '두 번째 희석액으로 이동',
        procedure: [
          '1. 난자를 DS2 (0.25M Sucrose) 드롭으로 이동',
          '2. 3분간 평형',
        ],
      },
      {
        stepNumber: 7,
        nameKr: '세척',
        duration: 5,
        temperature: 37,
        action: '배양액으로 세척',
        procedure: [
          '1. 난자를 WS 드롭으로 이동',
          '2. 2분간 유지',
          '3. 새로운 WS 드롭으로 한 번 더 세척',
          '4. 2-3분간 유지',
        ],
      },
      {
        stepNumber: 8,
        nameKr: '생존율 평가',
        duration: 5,
        action: '난자 생존 여부 확인',
        procedure: [
          '1. 세포막 완전성 확인',
          '2. 세포질 균질성 평가',
          '3. 투명대 상태 확인',
          '4. 극체 존재 확인',
        ],
        criteria: {
          survived: [
            '세포막 온전함',
            '세포질 굴절성 정상',
            '투명대 손상 없음',
            '극체 존재',
          ],
          degenerated: [
            '세포막 파열',
            '세포질 과립화 또는 어두움',
            '투명대 균열',
          ],
        },
      },
      {
        stepNumber: 9,
        nameKr: '배양',
        duration: 120,
        temperature: 37,
        action: '사용 전까지 배양',
        procedure: [
          '1. 신선한 배양액으로 이동',
          '2. 37°C, 5% CO2, 5% O2 인큐베이터에서 배양',
          '3. 최소 2시간 회복 시간',
          '4. ICSI 예정 시간 고려하여 타이밍 조절',
        ],
      },
    ];

    return {
      success: true,
      survival: true,
      quality: {
        membrane: 'intact',
        cytoplasm: 'normal',
        zonaIntegrity: 'normal',
        polarBody: 'present',
        survivalRate: 95,
      },
    };
  }

  /**
   * 해동 후 품질 평가
   */
  async assessPostWarmingQuality(oocyteId: string): Promise<{
    survived: boolean;
    qualityGrade: 'excellent' | 'good' | 'fair' | 'poor';
    fertilizable: boolean;
    notes: string;
  }> {
    // 실제 평가 로직
    return {
      survived: true,
      qualityGrade: 'excellent',
      fertilizable: true,
      notes: '세포막과 세포질 상태 양호, ICSI 진행 가능',
    };
  }
}
```

## 프로토콜 실행 엔진

```typescript
/**
 * 프로토콜 실행 엔진
 */
export class ProtocolExecutionEngine {
  /**
   * 프로토콜 실행
   */
  async executeProtocol(
    protocolId: string,
    specimenId: string,
    options: {
      performedBy: string;
      witnesses?: string[];
      autoAdvance?: boolean;
    }
  ): Promise<{
    executionId: string;
    status: string;
    progress: number;
  }> {
    const executionId = crypto.randomUUID();

    // 프로토콜 로드
    const protocol = await this.loadProtocol(protocolId);

    // 검체 로드
    const specimen = await this.loadSpecimen(specimenId);

    // 호환성 확인
    this.validateCompatibility(protocol, specimen);

    // 실행 시작
    const execution = await this.startExecution({
      executionId,
      protocolId,
      specimenId,
      performedBy: options.performedBy,
      witnesses: options.witnesses,
      startedAt: new Date().toISOString(),
    });

    // 자동 진행 모드면 모든 단계 실행
    if (options.autoAdvance) {
      await this.autoExecuteSteps(execution);
    }

    return {
      executionId,
      status: 'IN_PROGRESS',
      progress: 0,
    };
  }

  /**
   * 단계 실행
   */
  async executeStep(
    executionId: string,
    stepNumber: number,
    data?: any
  ): Promise<{
    success: boolean;
    nextStep: number | null;
  }> {
    const execution = await this.getExecution(executionId);
    const protocol = await this.loadProtocol(execution.protocolId);
    const step = protocol.steps[stepNumber - 1];

    // 단계 시작
    await this.markStepStarted(executionId, stepNumber);

    try {
      // 단계 수행
      await this.performStep(step, data);

      // 품질 확인
      if (step.qualityChecks) {
        await this.performQualityChecks(step.qualityChecks);
      }

      // 단계 완료
      await this.markStepCompleted(executionId, stepNumber);

      const nextStep = stepNumber < protocol.steps.length ? stepNumber + 1 : null;

      return {
        success: true,
        nextStep,
      };
    } catch (error) {
      await this.markStepFailed(executionId, stepNumber, error);
      throw error;
    }
  }

  /**
   * 실행 상태 조회
   */
  async getExecutionStatus(executionId: string): Promise<{
    status: string;
    currentStep: number;
    completedSteps: number;
    totalSteps: number;
    progress: number;
  }> {
    const execution = await this.getExecution(executionId);

    return {
      status: execution.status,
      currentStep: execution.currentStep,
      completedSteps: execution.completedSteps,
      totalSteps: execution.totalSteps,
      progress: (execution.completedSteps / execution.totalSteps) * 100,
    };
  }

  // Helper methods
  private async loadProtocol(protocolId: string): Promise<any> {
    // 프로토콜 로드 로직
    return {};
  }

  private async loadSpecimen(specimenId: string): Promise<any> {
    // 검체 로드 로직
    return {};
  }

  private validateCompatibility(protocol: any, specimen: any): void {
    // 호환성 검증 로직
  }

  private async startExecution(data: any): Promise<any> {
    // 실행 시작 로직
    return data;
  }

  private async autoExecuteSteps(execution: any): Promise<void> {
    // 자동 실행 로직
  }

  private async getExecution(executionId: string): Promise<any> {
    // 실행 정보 조회
    return {};
  }

  private async markStepStarted(executionId: string, stepNumber: number): Promise<void> {
    // 단계 시작 표시
  }

  private async performStep(step: any, data: any): Promise<void> {
    // 단계 수행
  }

  private async performQualityChecks(checks: string[]): Promise<void> {
    // 품질 확인
  }

  private async markStepCompleted(executionId: string, stepNumber: number): Promise<void> {
    // 단계 완료 표시
  }

  private async markStepFailed(executionId: string, stepNumber: number, error: any): Promise<void> {
    // 단계 실패 표시
  }
}
```

---

**문서 버전**: 1.0
**최종 수정**: 2025-01-11
**작성자**: WIA Standards Committee

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
