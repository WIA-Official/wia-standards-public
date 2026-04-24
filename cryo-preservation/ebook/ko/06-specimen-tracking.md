# 검체 추적 및 관리

**弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

## 관리연속성 (Chain of Custody)

### 관리연속성 시스템

```typescript
/**
 * 관리연속성 (Chain of Custody) 관리 시스템
 *
 * 검체의 전체 생명주기 동안 누가, 언제, 어디서, 무엇을 했는지 추적
 */
export class ChainOfCustodyManager {
  /**
   * 관리연속성 이벤트 유형
   */
  private eventTypes = {
    COLLECTED: {
      code: 'COLLECTED',
      nameKr: '채취됨',
      description: '환자로부터 검체 채취',
    },
    RECEIVED: {
      code: 'RECEIVED',
      nameKr: '수령됨',
      description: '실험실에서 검체 수령',
    },
    PROCESSED: {
      code: 'PROCESSED',
      nameKr: '처리됨',
      description: '검체 전처리 수행',
    },
    TESTED: {
      code: 'TESTED',
      nameKr: '검사됨',
      description: '품질 검사 수행',
    },
    FROZEN: {
      code: 'FROZEN',
      nameKr: '동결됨',
      description: '냉동보존 수행',
    },
    STORED: {
      code: 'STORED',
      nameKr: '보관됨',
      description: '저장 탱크에 보관',
    },
    RELOCATED: {
      code: 'RELOCATED',
      nameKr: '이동됨',
      description: '저장 위치 변경',
    },
    RETRIEVED: {
      code: 'RETRIEVED',
      nameKr: '꺼내짐',
      description: '저장소에서 회수',
    },
    THAWED: {
      code: 'THAWED',
      nameKr: '해동됨',
      description: '해동 프로토콜 수행',
    },
    TRANSFERRED: {
      code: 'TRANSFERRED',
      nameKr: '이식됨',
      description: '환자에게 이식',
    },
    DISPOSED: {
      code: 'DISPOSED',
      nameKr: '폐기됨',
      description: '검체 폐기',
    },
    SHIPPED: {
      code: 'SHIPPED',
      nameKr: '발송됨',
      description: '다른 시설로 발송',
    },
  };

  /**
   * 관리연속성 이벤트 기록
   */
  async recordEvent(data: {
    specimenId: string;
    eventType: string;
    custodian: string;
    custodianName: string;
    location: string;
    action: string;
    actionKr: string;
    witness?: string;
    witnessName?: string;
    notes?: string;
    attachments?: string[];
  }): Promise<{
    eventId: string;
    recorded: boolean;
    timestamp: string;
  }> {
    const event = {
      eventId: crypto.randomUUID(),
      specimenId: data.specimenId,
      timestamp: new Date().toISOString(),
      eventType: data.eventType,
      eventTypeKr: this.eventTypes[data.eventType as keyof typeof this.eventTypes].nameKr,
      custodian: data.custodian,
      custodianName: data.custodianName,
      custodianRole: await this.getCustodianRole(data.custodian),
      location: data.location,
      action: data.action,
      actionKr: data.actionKr,
      witness: data.witness,
      witnessName: data.witnessName,
      notes: data.notes,
      attachments: data.attachments,
      digitalSignature: await this.generateDigitalSignature({
        eventId: data.specimenId,
        custodian: data.custodian,
        timestamp: new Date().toISOString(),
      }),
      verified: false,
    };

    // 데이터베이스에 저장
    await this.saveEvent(event);

    // 블록체인에 해시 기록 (옵션)
    if (this.blockchainEnabled) {
      await this.recordOnBlockchain(event);
    }

    return {
      eventId: event.eventId,
      recorded: true,
      timestamp: event.timestamp,
    };
  }

  /**
   * 관리연속성 조회
   */
  async getChainOfCustody(specimenId: string): Promise<{
    specimenId: string;
    events: any[];
    verified: boolean;
    totalEvents: number;
  }> {
    const events = await this.getEvents(specimenId);

    // 연속성 검증
    const verified = this.verifyChainIntegrity(events);

    return {
      specimenId,
      events: events.map(e => ({
        eventId: e.eventId,
        timestamp: e.timestamp,
        eventType: e.eventType,
        eventTypeKr: e.eventTypeKr,
        custodian: e.custodianName,
        role: e.custodianRole,
        location: e.location,
        action: e.actionKr,
        witness: e.witnessName,
        verified: e.verified,
      })),
      verified,
      totalEvents: events.length,
    };
  }

  /**
   * 관리연속성 타임라인 생성
   */
  async generateTimeline(specimenId: string): Promise<{
    specimenId: string;
    timeline: Array<{
      date: string;
      events: any[];
    }>;
  }> {
    const events = await this.getEvents(specimenId);

    // 날짜별로 그룹화
    const groupedByDate = events.reduce((acc, event) => {
      const date = event.timestamp.split('T')[0];
      if (!acc[date]) {
        acc[date] = [];
      }
      acc[date].push(event);
      return acc;
    }, {} as Record<string, any[]>);

    const timeline = Object.entries(groupedByDate)
      .map(([date, events]) => ({
        date,
        events: events.sort((a, b) =>
          new Date(a.timestamp).getTime() - new Date(b.timestamp).getTime()
        ),
      }))
      .sort((a, b) => new Date(a.date).getTime() - new Date(b.date).getTime());

    return {
      specimenId,
      timeline,
    };
  }

  /**
   * 검체 이동 기록
   */
  async recordTransfer(data: {
    specimenId: string;
    fromLocation: string;
    toLocation: string;
    performedBy: string;
    witness: string;
    reason: string;
  }): Promise<{
    transferId: string;
    success: boolean;
  }> {
    // 이동 전 위치 확인
    const currentLocation = await this.getCurrentLocation(data.specimenId);
    if (currentLocation !== data.fromLocation) {
      throw new Error('현재 위치가 일치하지 않습니다');
    }

    // 이동 이벤트 기록
    await this.recordEvent({
      specimenId: data.specimenId,
      eventType: 'RELOCATED',
      custodian: data.performedBy,
      custodianName: await this.getUserName(data.performedBy),
      location: data.toLocation,
      action: `Moved from ${data.fromLocation} to ${data.toLocation}`,
      actionKr: `${data.fromLocation}에서 ${data.toLocation}으로 이동`,
      witness: data.witness,
      witnessName: await this.getUserName(data.witness),
      notes: data.reason,
    });

    // 검체 위치 업데이트
    await this.updateSpecimenLocation(data.specimenId, data.toLocation);

    return {
      transferId: crypto.randomUUID(),
      success: true,
    };
  }

  /**
   * 관리연속성 검증
   */
  private verifyChainIntegrity(events: any[]): boolean {
    // 1. 시간순 정렬 확인
    for (let i = 1; i < events.length; i++) {
      if (new Date(events[i].timestamp) < new Date(events[i - 1].timestamp)) {
        return false;
      }
    }

    // 2. 필수 이벤트 존재 확인
    const hasCollected = events.some(e => e.eventType === 'COLLECTED');
    const hasStored = events.some(e => e.eventType === 'STORED');
    if (!hasCollected || !hasStored) {
      return false;
    }

    // 3. 디지털 서명 검증
    const allSigned = events.every(e => e.digitalSignature);
    if (!allSigned) {
      return false;
    }

    return true;
  }

  // Helper methods
  private blockchainEnabled = false;

  private async getCustodianRole(userId: string): Promise<string> {
    // 사용자 역할 조회
    return 'Embryologist';
  }

  private async generateDigitalSignature(data: any): Promise<string> {
    // 디지털 서명 생성 (SHA-256 해시)
    const content = JSON.stringify(data);
    return `SIG_${content.length}_${Date.now()}`;
  }

  private async saveEvent(event: any): Promise<void> {
    // 데이터베이스 저장 로직
  }

  private async recordOnBlockchain(event: any): Promise<void> {
    // 블록체인 기록 로직
  }

  private async getEvents(specimenId: string): Promise<any[]> {
    // 이벤트 조회 로직
    return [];
  }

  private async getCurrentLocation(specimenId: string): Promise<string> {
    // 현재 위치 조회
    return 'TANK-01-A-001';
  }

  private async getUserName(userId: string): Promise<string> {
    // 사용자 이름 조회
    return 'Dr. Kim';
  }

  private async updateSpecimenLocation(specimenId: string, location: string): Promise<void> {
    // 검체 위치 업데이트
  }
}
```

## 바코드 및 RFID 시스템

### 바코드 관리

```typescript
/**
 * 바코드 생성 및 관리 시스템
 */
export class BarcodeManager {
  /**
   * 바코드 형식
   * - 유형: Code 128
   * - 구조: [시설코드(3)][검체유형(2)][년도(2)][일련번호(8)]
   * - 예시: SCF-SP-24-00001234
   */
  private barcodeFormat = {
    facilityCode: 3,     // 시설 코드 (예: SCF = Seoul Cryo Facility)
    specimenType: 2,     // 검체 유형 (SP=정자, OO=난자, EM=배아)
    yearCode: 2,         // 연도 (24 = 2024)
    serialNumber: 8,     // 일련번호
  };

  /**
   * 검체 유형 코드
   */
  private specimenTypeCodes = {
    SPERM: 'SP',
    OOCYTE: 'OO',
    EMBRYO: 'EM',
    CORD_BLOOD: 'CB',
    TISSUE: 'TS',
    STEM_CELL: 'SC',
    OVARIAN_TISSUE: 'OV',
    TESTICULAR_TISSUE: 'TE',
    BONE_MARROW: 'BM',
    PERIPHERAL_BLOOD: 'PB',
  };

  /**
   * 바코드 생성
   */
  async generateBarcode(data: {
    facilityCode: string;
    specimenType: string;
    year?: number;
  }): Promise<{
    barcodeId: string;
    barcodeSvg: string;
    qrCode: string;
  }> {
    const year = data.year || new Date().getFullYear();
    const yearCode = year.toString().slice(-2);
    const typeCode = this.specimenTypeCodes[data.specimenType as keyof typeof this.specimenTypeCodes];
    const serialNumber = await this.getNextSerialNumber(data.facilityCode, yearCode);

    const barcodeId = `${data.facilityCode}-${typeCode}-${yearCode}-${serialNumber}`;

    // 바코드 SVG 생성 (Code 128)
    const barcodeSvg = await this.generateBarcodeSvg(barcodeId);

    // QR 코드 생성 (추가 정보 포함)
    const qrCode = await this.generateQRCode({
      barcodeId,
      facilityCode: data.facilityCode,
      specimenType: data.specimenType,
      createdAt: new Date().toISOString(),
    });

    return {
      barcodeId,
      barcodeSvg,
      qrCode,
    };
  }

  /**
   * 바코드 스캔
   */
  async scanBarcode(barcodeId: string): Promise<{
    valid: boolean;
    specimen?: any;
    error?: string;
  }> {
    // 바코드 형식 검증
    if (!this.validateBarcodeFormat(barcodeId)) {
      return {
        valid: false,
        error: '잘못된 바코드 형식',
      };
    }

    // 바코드 파싱
    const parsed = this.parseBarcode(barcodeId);

    // 검체 조회
    const specimen = await this.findSpecimenByBarcode(barcodeId);

    if (!specimen) {
      return {
        valid: false,
        error: '검체를 찾을 수 없습니다',
      };
    }

    // 스캔 이력 기록
    await this.recordScan({
      barcodeId,
      specimenId: specimen.specimenId,
      scannedAt: new Date().toISOString(),
      scannedBy: 'current-user-id',
    });

    return {
      valid: true,
      specimen,
    };
  }

  /**
   * 바코드 인쇄
   */
  async printBarcode(barcodeId: string, options: {
    printerName?: string;
    copies?: number;
    labelSize?: 'small' | 'medium' | 'large';
  }): Promise<{
    success: boolean;
    printedAt: string;
  }> {
    const specimen = await this.findSpecimenByBarcode(barcodeId);

    if (!specimen) {
      throw new Error('검체를 찾을 수 없습니다');
    }

    // 라벨 생성
    const label = await this.generateLabel({
      barcodeId,
      specimenType: specimen.typeKr,
      patientInitials: specimen.patientInfo.initials,
      collectionDate: specimen.collection.collectionDate,
      labelSize: options.labelSize || 'medium',
    });

    // 프린터로 전송
    await this.sendToPrinter({
      label,
      printerName: options.printerName || 'default',
      copies: options.copies || 1,
    });

    return {
      success: true,
      printedAt: new Date().toISOString(),
    };
  }

  /**
   * 배치 바코드 생성
   */
  async generateBatch(data: {
    facilityCode: string;
    specimenType: string;
    count: number;
  }): Promise<{
    barcodes: string[];
    printFile: string;
  }> {
    const barcodes: string[] = [];

    for (let i = 0; i < data.count; i++) {
      const barcode = await this.generateBarcode({
        facilityCode: data.facilityCode,
        specimenType: data.specimenType,
      });
      barcodes.push(barcode.barcodeId);
    }

    // 인쇄 파일 생성
    const printFile = await this.generateBatchPrintFile(barcodes);

    return {
      barcodes,
      printFile,
    };
  }

  // Helper methods
  private async getNextSerialNumber(facilityCode: string, yearCode: string): Promise<string> {
    // 데이터베이스에서 마지막 일련번호 조회 후 +1
    const lastNumber = 1234; // 예시
    return (lastNumber + 1).toString().padStart(8, '0');
  }

  private async generateBarcodeSvg(barcodeId: string): Promise<string> {
    // Code 128 바코드 SVG 생성 로직
    return `<svg>...</svg>`;
  }

  private async generateQRCode(data: any): Promise<string> {
    // QR 코드 생성 로직
    return `QR_${JSON.stringify(data)}`;
  }

  private validateBarcodeFormat(barcodeId: string): boolean {
    // 형식: XXX-XX-XX-XXXXXXXX
    const pattern = /^[A-Z]{3}-[A-Z]{2}-\d{2}-\d{8}$/;
    return pattern.test(barcodeId);
  }

  private parseBarcode(barcodeId: string): {
    facilityCode: string;
    specimenType: string;
    year: number;
    serialNumber: number;
  } {
    const [facilityCode, specimenType, yearCode, serialNumber] = barcodeId.split('-');
    return {
      facilityCode,
      specimenType,
      year: 2000 + parseInt(yearCode),
      serialNumber: parseInt(serialNumber),
    };
  }

  private async findSpecimenByBarcode(barcodeId: string): Promise<any> {
    // 검체 조회 로직
    return null;
  }

  private async recordScan(data: any): Promise<void> {
    // 스캔 이력 기록
  }

  private async generateLabel(data: any): Promise<any> {
    // 라벨 생성
    return {};
  }

  private async sendToPrinter(data: any): Promise<void> {
    // 프린터 전송
  }

  private async generateBatchPrintFile(barcodes: string[]): Promise<string> {
    // 배치 인쇄 파일 생성
    return 'batch_print.pdf';
  }
}
```

### RFID 시스템

```typescript
/**
 * RFID 추적 시스템
 * - 표준: EPC Gen2 (ISO 18000-6C)
 * - 주파수: UHF (860-960 MHz)
 * - 읽기 거리: 최대 5m
 */
export class RFIDTrackingSystem {
  /**
   * RFID 태그 등록
   */
  async registerTag(data: {
    specimenId: string;
    tagId?: string; // 미제공 시 자동 생성
  }): Promise<{
    tagId: string;
    epc: string;
    registered: boolean;
  }> {
    // EPC (Electronic Product Code) 생성
    const epc = await this.generateEPC({
      header: '35', // SGTIN-96
      filter: '1',  // Unit Load
      partition: '3',
      companyPrefix: '0614141', // 예시
      itemReference: data.specimenId.slice(-8),
      serialNumber: Date.now().toString(36).toUpperCase(),
    });

    const tagId = data.tagId || epc;

    // RFID 태그 정보 저장
    await this.saveTagInfo({
      tagId,
      epc,
      specimenId: data.specimenId,
      registeredAt: new Date().toISOString(),
      status: 'active',
    });

    return {
      tagId,
      epc,
      registered: true,
    };
  }

  /**
   * RFID 태그 읽기
   */
  async readTag(tagId: string): Promise<{
    tagId: string;
    specimen: any;
    location: string;
    lastSeen: string;
  }> {
    const tagInfo = await this.getTagInfo(tagId);

    if (!tagInfo) {
      throw new Error('RFID 태그를 찾을 수 없습니다');
    }

    // 마지막 감지 정보 업데이트
    await this.updateLastSeen(tagId);

    return {
      tagId,
      specimen: await this.getSpecimen(tagInfo.specimenId),
      location: await this.getLocationFromReader(tagId),
      lastSeen: new Date().toISOString(),
    };
  }

  /**
   * 자동 위치 추적
   */
  async startAutomaticTracking(config: {
    readers: string[];      // RFID 리더기 ID 목록
    interval: number;       // 스캔 간격 (초)
    alertOnMovement: boolean;
  }): Promise<{
    trackingId: string;
    active: boolean;
  }> {
    const trackingId = crypto.randomUUID();

    // 각 리더기에서 주기적으로 스캔
    for (const readerId of config.readers) {
      this.startReaderScanning(readerId, config.interval, (tags) => {
        // 감지된 태그 처리
        tags.forEach(async (tag) => {
          await this.processTagDetection({
            tagId: tag.tagId,
            readerId,
            timestamp: new Date().toISOString(),
            rssi: tag.rssi, // Signal strength
          });

          // 이동 감지 시 알림
          if (config.alertOnMovement) {
            const moved = await this.detectMovement(tag.tagId);
            if (moved) {
              await this.sendMovementAlert(tag.tagId);
            }
          }
        });
      });
    }

    return {
      trackingId,
      active: true,
    };
  }

  /**
   * 검체 위치 조회
   */
  async locateSpecimen(specimenId: string): Promise<{
    found: boolean;
    location?: string;
    tankId?: string;
    position?: string;
    lastUpdate: string;
  }> {
    const tagInfo = await this.findTagBySpecimen(specimenId);

    if (!tagInfo) {
      return {
        found: false,
        lastUpdate: new Date().toISOString(),
      };
    }

    // 최근 감지 위치 조회
    const lastDetection = await this.getLastDetection(tagInfo.tagId);

    return {
      found: true,
      location: lastDetection.location,
      tankId: lastDetection.tankId,
      position: lastDetection.position,
      lastUpdate: lastDetection.timestamp,
    };
  }

  /**
   * 재고 조사
   */
  async performInventory(tankId: string): Promise<{
    totalExpected: number;
    totalFound: number;
    found: string[];
    missing: string[];
    unexpected: string[];
  }> {
    // 예상 검체 목록
    const expected = await this.getExpectedSpecimens(tankId);

    // RFID 스캔 수행
    const scannedTags = await this.scanTank(tankId);

    // 발견된 검체
    const found = scannedTags.filter(tag =>
      expected.includes(tag.specimenId)
    );

    // 누락된 검체
    const missing = expected.filter(specimenId =>
      !scannedTags.some(tag => tag.specimenId === specimenId)
    );

    // 예상치 못한 검체
    const unexpected = scannedTags.filter(tag =>
      !expected.includes(tag.specimenId)
    );

    return {
      totalExpected: expected.length,
      totalFound: found.length,
      found: found.map(t => t.specimenId),
      missing,
      unexpected: unexpected.map(t => t.specimenId),
    };
  }

  /**
   * 무단 반출 감지
   */
  async detectUnauthorizedRemoval(): Promise<{
    alerts: Array<{
      tagId: string;
      specimenId: string;
      exitPoint: string;
      timestamp: string;
    }>;
  }> {
    // 출입구 RFID 리더기에서 감지된 태그 중
    // 반출 승인이 없는 검체 확인

    const exitReaderIds = await this.getExitReaders();
    const alerts: any[] = [];

    for (const readerId of exitReaderIds) {
      const detections = await this.getRecentDetections(readerId, 60); // 최근 1분

      for (const detection of detections) {
        const authorized = await this.checkRemovalAuthorization(detection.tagId);
        if (!authorized) {
          alerts.push({
            tagId: detection.tagId,
            specimenId: detection.specimenId,
            exitPoint: readerId,
            timestamp: detection.timestamp,
          });

          // 알림 전송
          await this.sendSecurityAlert({
            type: 'UNAUTHORIZED_REMOVAL',
            tagId: detection.tagId,
            location: readerId,
          });
        }
      }
    }

    return { alerts };
  }

  // Helper methods
  private async generateEPC(data: any): Promise<string> {
    // EPC 생성 로직
    return Array.from({ length: 24 }, () =>
      Math.floor(Math.random() * 16).toString(16).toUpperCase()
    ).join('');
  }

  private async saveTagInfo(data: any): Promise<void> {
    // 태그 정보 저장
  }

  private async getTagInfo(tagId: string): Promise<any> {
    // 태그 정보 조회
    return null;
  }

  private async updateLastSeen(tagId: string): Promise<void> {
    // 마지막 감지 시간 업데이트
  }

  private async getSpecimen(specimenId: string): Promise<any> {
    // 검체 조회
    return {};
  }

  private async getLocationFromReader(tagId: string): Promise<string> {
    // 리더기 위치 조회
    return 'TANK-01';
  }

  private startReaderScanning(
    readerId: string,
    interval: number,
    callback: (tags: any[]) => void
  ): void {
    // 리더기 스캔 시작
    setInterval(async () => {
      const tags = await this.scanReader(readerId);
      callback(tags);
    }, interval * 1000);
  }

  private async scanReader(readerId: string): Promise<any[]> {
    // RFID 리더기 스캔
    return [];
  }

  private async processTagDetection(data: any): Promise<void> {
    // 태그 감지 처리
  }

  private async detectMovement(tagId: string): Promise<boolean> {
    // 이동 감지
    return false;
  }

  private async sendMovementAlert(tagId: string): Promise<void> {
    // 이동 알림 전송
  }

  private async findTagBySpecimen(specimenId: string): Promise<any> {
    // 검체로 태그 찾기
    return null;
  }

  private async getLastDetection(tagId: string): Promise<any> {
    // 마지막 감지 정보 조회
    return {};
  }

  private async getExpectedSpecimens(tankId: string): Promise<string[]> {
    // 예상 검체 목록 조회
    return [];
  }

  private async scanTank(tankId: string): Promise<any[]> {
    // 탱크 스캔
    return [];
  }

  private async getExitReaders(): Promise<string[]> {
    // 출입구 리더기 목록
    return [];
  }

  private async getRecentDetections(readerId: string, seconds: number): Promise<any[]> {
    // 최근 감지 목록
    return [];
  }

  private async checkRemovalAuthorization(tagId: string): Promise<boolean> {
    // 반출 승인 확인
    return false;
  }

  private async sendSecurityAlert(data: any): Promise<void> {
    // 보안 알림 전송
  }
}
```

## 위치 추적 시스템

```typescript
/**
 * 저장 위치 관리 시스템
 */
export class StorageLocationManager {
  /**
   * 위치 구조
   * 탱크 > 캐니스터 > 케인 > 위치 (층-행-열)
   */
  private locationStructure = {
    tank: {
      capacity: 12,        // 캐니스터 수
      temperature: -196,   // °C
    },
    canister: {
      capacity: 6,         // 케인 수
      height: 50,          // cm
    },
    cane: {
      capacity: 10,        // 바이알/스트로 수
      length: 13,          // cm
    },
  };

  /**
   * 빈 위치 찾기
   */
  async findAvailableLocation(criteria: {
    tankId?: string;
    specimenType?: string;
    quantity?: number;
  }): Promise<{
    available: boolean;
    locations: Array<{
      tankId: string;
      canisterId: string;
      caneId: string;
      position: string;
    }>;
  }> {
    const quantity = criteria.quantity || 1;
    const availableLocations: any[] = [];

    // 탱크 목록 조회
    const tanks = criteria.tankId
      ? [await this.getTank(criteria.tankId)]
      : await this.getAllTanks();

    for (const tank of tanks) {
      if (availableLocations.length >= quantity) break;

      // 탱크 용량 확인
      const usage = await this.getTankUsage(tank.tankId);
      if (usage.available <= 0) continue;

      // 캐니스터별 확인
      for (const canister of tank.canisters) {
        if (availableLocations.length >= quantity) break;

        // 케인별 확인
        for (const cane of canister.canes) {
          if (availableLocations.length >= quantity) break;

          // 빈 위치 확인
          const emptyPositions = await this.getEmptyPositions(cane.caneId);

          for (const position of emptyPositions) {
            availableLocations.push({
              tankId: tank.tankId,
              canisterId: canister.canisterId,
              caneId: cane.caneId,
              position,
            });

            if (availableLocations.length >= quantity) break;
          }
        }
      }
    }

    return {
      available: availableLocations.length >= quantity,
      locations: availableLocations.slice(0, quantity),
    };
  }

  /**
   * 위치 할당
   */
  async allocateLocation(
    specimenId: string,
    location: {
      tankId: string;
      canisterId: string;
      caneId: string;
      position: string;
    }
  ): Promise<{
    allocated: boolean;
    fullLocation: string;
  }> {
    // 위치 사용 가능 여부 확인
    const available = await this.isLocationAvailable(location);
    if (!available) {
      throw new Error('이미 사용중인 위치입니다');
    }

    // 위치 할당
    await this.assignLocation(specimenId, location);

    const fullLocation = `${location.tankId}-${location.canisterId}-${location.caneId}-${location.position}`;

    return {
      allocated: true,
      fullLocation,
    };
  }

  /**
   * 위치 맵 생성
   */
  async generateLocationMap(tankId: string): Promise<{
    tankId: string;
    map: Array<{
      canisterId: string;
      canes: Array<{
        caneId: string;
        positions: Array<{
          position: string;
          occupied: boolean;
          specimenId?: string;
          specimenType?: string;
        }>;
      }>;
    }>;
  }> {
    const tank = await this.getTank(tankId);
    const map: any[] = [];

    for (const canister of tank.canisters) {
      const canes: any[] = [];

      for (const cane of canister.canes) {
        const positions = await this.getCanePositions(cane.caneId);
        canes.push({
          caneId: cane.caneId,
          positions,
        });
      }

      map.push({
        canisterId: canister.canisterId,
        canes,
      });
    }

    return {
      tankId,
      map,
    };
  }

  /**
   * 검체 검색
   */
  async searchSpecimens(criteria: {
    patientId?: string;
    specimenType?: string;
    dateRange?: { from: Date; to: Date };
    tankId?: string;
  }): Promise<Array<{
    specimenId: string;
    type: string;
    location: string;
    collectionDate: string;
  }>> {
    // 검색 로직
    return [];
  }

  /**
   * 탱크 최적화
   * 조각난 공간을 정리하여 효율적으로 재배치
   */
  async optimizeTank(tankId: string): Promise<{
    before: { occupancy: number; fragmentation: number };
    after: { occupancy: number; fragmentation: number };
    relocations: number;
  }> {
    const beforeStats = await this.getTankStats(tankId);

    // 검체 재배치 계획 수립
    const relocations = await this.planRelocations(tankId);

    // 재배치 실행
    for (const relocation of relocations) {
      await this.relocateSpecimen(
        relocation.specimenId,
        relocation.from,
        relocation.to
      );
    }

    const afterStats = await this.getTankStats(tankId);

    return {
      before: beforeStats,
      after: afterStats,
      relocations: relocations.length,
    };
  }

  // Helper methods
  private async getTank(tankId: string): Promise<any> {
    return { tankId, canisters: [] };
  }

  private async getAllTanks(): Promise<any[]> {
    return [];
  }

  private async getTankUsage(tankId: string): Promise<any> {
    return { total: 100, used: 60, available: 40 };
  }

  private async getEmptyPositions(caneId: string): Promise<string[]> {
    return [];
  }

  private async isLocationAvailable(location: any): Promise<boolean> {
    return true;
  }

  private async assignLocation(specimenId: string, location: any): Promise<void> {
    // 위치 할당 로직
  }

  private async getCanePositions(caneId: string): Promise<any[]> {
    return [];
  }

  private async getTankStats(tankId: string): Promise<any> {
    return { occupancy: 0, fragmentation: 0 };
  }

  private async planRelocations(tankId: string): Promise<any[]> {
    return [];
  }

  private async relocateSpecimen(
    specimenId: string,
    from: any,
    to: any
  ): Promise<void> {
    // 재배치 로직
  }
}
```

---

**문서 버전**: 1.0
**최종 수정**: 2025-01-11
**작성자**: WIA Standards Committee

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
