# 제7장: 시스템 통합

## 학습 목표

이 장을 완료하면 다음을 수행할 수 있습니다:
- 한국 환경부 및 환경공단 시스템과의 연계 구현
- EPR 공제조합 시스템 통합 설계
- 기업 ERP 및 레거시 시스템 연동
- 실시간 데이터 동기화 전략 수립
- 통합 아키텍처 보안 및 모니터링 구현

---

## 7.1 한국 환경 규제 시스템 통합

### 7.1.1 통합 대상 시스템 개요

```typescript
// 한국 환경 규제 시스템 연계 맵
interface KoreaEnvironmentalSystemsMap {
  regulatorySystems: {
    // 올바로 시스템 (폐기물 관리)
    allbaro: {
      operator: "한국환경공단";
      purpose: "폐기물 인계인수 및 처리실적 관리";
      integrationLevel: "mandatory";
      dataExchange: [
        "전자인계서",
        "처리실적",
        "재활용실적",
        "업체정보"
      ];
      protocol: "SOAP Web Service";
      authentication: "공인인증서 + API Key";
    };

    // EPR 통합관리시스템
    eprSystem: {
      operator: "한국환경공단";
      purpose: "생산자책임재활용 의무 관리";
      integrationLevel: "mandatory_for_producers";
      dataExchange: [
        "출고량 신고",
        "재활용실적 보고",
        "분담금 정산",
        "의무이행 현황"
      ];
      protocol: "REST API";
      authentication: "인증서 + OAuth 2.0";
    };

    // 자원순환정보시스템
    resourceCirculation: {
      operator: "환경부";
      purpose: "자원순환 통계 및 정책 지원";
      integrationLevel: "optional";
      dataExchange: [
        "재활용률 통계",
        "물질흐름 분석",
        "순환자원 인정"
      ];
      protocol: "REST API";
      authentication: "API Key";
    };
  };

  industrySystems: {
    // 전자제품자원순환공제조합 (ECOAS)
    ecoas: {
      operator: "전자제품자원순환공제조합";
      purpose: "전자제품 EPR 공동이행";
      integrationLevel: "mandatory_for_members";
      dataExchange: [
        "출고량 보고",
        "수거량 보고",
        "처리위탁",
        "분담금 납부"
      ];
      protocol: "REST API + File Exchange";
      authentication: "API Key + 회원번호";
    };

    // 한국전지재활용협회
    kbrc: {
      operator: "한국전지재활용협회";
      purpose: "전지류 재활용 관리";
      integrationLevel: "conditional";
      dataExchange: [
        "전지 수거량",
        "처리실적",
        "재활용률"
      ];
      protocol: "REST API";
      authentication: "API Key";
    };
  };
}
```

### 7.1.2 올바로 시스템 통합

```typescript
// 올바로 시스템 통합 구현
class AllbaroIntegration {
  private config: AllbaroConfig;
  private certificate: CertificateManager;
  private apiClient: SOAPClient;

  constructor(config: AllbaroConfig) {
    this.config = config;
    this.certificate = new CertificateManager(config.certificatePath);
    this.apiClient = new SOAPClient(config.wsdlUrl);
  }

  // 전자인계서 발행
  async issueElectronicManifest(
    manifestData: AllbaroManifestData
  ): Promise<AllbaroManifestResponse> {
    // 1. 인증서 로드 및 서명 준비
    const signature = await this.certificate.sign(manifestData);

    // 2. SOAP 요청 구성
    const request = {
      header: {
        serviceId: "MANIFEST_ISSUE",
        businessNumber: this.config.businessNumber,
        timestamp: new Date().toISOString(),
        signature: signature
      },
      body: {
        폐기물정보: manifestData.폐기물,
        배출자정보: manifestData.배출자,
        수집운반자정보: manifestData.수집운반자,
        처리자정보: manifestData.처리자,
        인계예정일: manifestData.인계예정일
      }
    };

    // 3. API 호출
    try {
      const response = await this.apiClient.call(
        "issueManifest",
        request,
        { timeout: 30000 }
      );

      // 4. 응답 검증 및 저장
      if (response.resultCode === "SUCCESS") {
        await this.saveManifestRecord({
          manifestNumber: response.manifestNumber,
          issueTimestamp: response.issueTimestamp,
          status: "issued",
          localData: manifestData
        });

        return {
          success: true,
          manifestNumber: response.manifestNumber,
          issueTimestamp: response.issueTimestamp,
          validUntil: response.validUntil
        };
      } else {
        throw new AllbaroError(response.resultCode, response.resultMessage);
      }
    } catch (error) {
      // 5. 오류 처리 및 재시도 로직
      if (this.isRetryableError(error)) {
        return this.retryWithBackoff(() => this.issueElectronicManifest(manifestData));
      }
      throw error;
    }
  }

  // 인계인수 확인
  async confirmHandover(
    manifestNumber: string,
    confirmationType: "collection" | "receipt",
    confirmationData: HandoverConfirmation
  ): Promise<ConfirmationResponse> {
    const signature = await this.certificate.sign({
      manifestNumber,
      confirmationType,
      ...confirmationData
    });

    const request = {
      header: {
        serviceId: confirmationType === "collection"
          ? "MANIFEST_COLLECT_CONFIRM"
          : "MANIFEST_RECEIPT_CONFIRM",
        businessNumber: this.config.businessNumber,
        timestamp: new Date().toISOString(),
        signature: signature
      },
      body: {
        인계서번호: manifestNumber,
        확인일시: confirmationData.confirmTimestamp,
        실제수량: confirmationData.actualQuantity,
        수량차이사유: confirmationData.varianceReason,
        담당자: confirmationData.confirmedBy,
        비고: confirmationData.notes
      }
    };

    const response = await this.apiClient.call("confirmHandover", request);

    // 로컬 기록 업데이트
    await this.updateManifestStatus(manifestNumber, {
      status: confirmationType === "collection" ? "collected" : "received",
      confirmTimestamp: confirmationData.confirmTimestamp,
      confirmedBy: confirmationData.confirmedBy
    });

    return response;
  }

  // 처리완료 보고
  async reportProcessingCompletion(
    manifestNumber: string,
    processingData: ProcessingCompletionData
  ): Promise<CompletionResponse> {
    const request = {
      header: {
        serviceId: "MANIFEST_PROCESSING_COMPLETE",
        businessNumber: this.config.businessNumber,
        timestamp: new Date().toISOString(),
        signature: await this.certificate.sign({ manifestNumber, ...processingData })
      },
      body: {
        인계서번호: manifestNumber,
        처리완료일: processingData.completionDate,
        처리방법: processingData.processingMethod,
        처리량: processingData.processedQuantity,
        잔재물: processingData.residuals.map(r => ({
          종류: r.type,
          수량: r.quantity,
          처분방법: r.disposalMethod
        })),
        재활용산물: processingData.recoveredMaterials.map(m => ({
          종류: m.type,
          수량: m.quantity,
          품질등급: m.qualityGrade
        })),
        처리담당자: processingData.processedBy
      }
    };

    return await this.apiClient.call("reportCompletion", request);
  }

  // 조회 및 현황
  async queryManifests(
    criteria: ManifestQueryCriteria
  ): Promise<ManifestQueryResult> {
    const request = {
      header: {
        serviceId: "MANIFEST_QUERY",
        businessNumber: this.config.businessNumber,
        timestamp: new Date().toISOString()
      },
      body: {
        조회기간: {
          시작일: criteria.startDate,
          종료일: criteria.endDate
        },
        상태: criteria.status,
        역할: criteria.role,  // 배출자/운반자/처리자
        페이지: criteria.page,
        페이지크기: criteria.pageSize
      }
    };

    return await this.apiClient.call("queryManifests", request);
  }

  // 재시도 로직
  private async retryWithBackoff<T>(
    operation: () => Promise<T>,
    maxRetries: number = 3,
    baseDelay: number = 1000
  ): Promise<T> {
    for (let attempt = 1; attempt <= maxRetries; attempt++) {
      try {
        return await operation();
      } catch (error) {
        if (attempt === maxRetries || !this.isRetryableError(error)) {
          throw error;
        }
        const delay = baseDelay * Math.pow(2, attempt - 1);
        await this.sleep(delay);
      }
    }
    throw new Error("Max retries exceeded");
  }

  private isRetryableError(error: any): boolean {
    return error.code === "NETWORK_ERROR" ||
           error.code === "TIMEOUT" ||
           error.code === "SERVICE_UNAVAILABLE";
  }

  private sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}
```

### 7.1.3 EPR 통합관리시스템 연계

```typescript
// EPR 통합관리시스템 클라이언트
class EPRSystemIntegration {
  private config: EPRSystemConfig;
  private oauth: OAuth2Client;
  private httpClient: HTTPClient;

  constructor(config: EPRSystemConfig) {
    this.config = config;
    this.oauth = new OAuth2Client(config.oauthConfig);
    this.httpClient = new HTTPClient(config.baseUrl);
  }

  // 출고량 신고
  async reportShipmentVolume(
    reportData: ShipmentVolumeReport
  ): Promise<ReportResponse> {
    const token = await this.oauth.getAccessToken();

    const payload = {
      reportingPeriod: {
        year: reportData.year,
        quarter: reportData.quarter
      },
      producer: {
        eprRegistrationNumber: reportData.producerId,
        businessNumber: reportData.businessNumber
      },
      shipments: reportData.items.map(item => ({
        productCategory: item.categoryCode,
        productName: item.productName,
        quantity: item.quantity,
        unit: item.unit,
        weightKg: item.weightKg,
        recyclingObligationRate: item.obligationRate
      })),
      declaration: {
        declarantName: reportData.declarant,
        declarationDate: new Date().toISOString(),
        isAccurate: true
      }
    };

    const response = await this.httpClient.post(
      "/api/v1/shipment-reports",
      payload,
      {
        headers: {
          "Authorization": `Bearer ${token}`,
          "Content-Type": "application/json"
        }
      }
    );

    return {
      reportId: response.data.reportId,
      submissionTimestamp: response.data.submissionTimestamp,
      status: response.data.status,
      validationErrors: response.data.validationErrors
    };
  }

  // 재활용실적 보고
  async reportRecyclingPerformance(
    performanceData: RecyclingPerformanceReport
  ): Promise<ReportResponse> {
    const token = await this.oauth.getAccessToken();

    const payload = {
      reportingPeriod: {
        year: performanceData.year,
        type: "annual"
      },
      producer: {
        eprRegistrationNumber: performanceData.producerId
      },
      recyclingResults: performanceData.results.map(result => ({
        productCategory: result.categoryCode,
        obligationQuantity: result.obligationKg,
        selfFulfilledQuantity: result.selfFulfilledKg,
        consortiumFulfilledQuantity: result.consortiumFulfilledKg,
        totalFulfilledQuantity: result.totalFulfilledKg,
        fulfillmentRate: result.fulfillmentRate,
        surplus: result.surplusKg,
        deficit: result.deficitKg
      })),
      processingFacilities: performanceData.facilities.map(facility => ({
        facilityId: facility.id,
        facilityName: facility.name,
        licenseNumber: facility.license,
        processingMethod: facility.method,
        processedQuantity: facility.quantityKg,
        certificateNumber: facility.certificateNumber
      })),
      supporting_documents: performanceData.documents.map(doc => ({
        documentType: doc.type,
        documentNumber: doc.number,
        issueDate: doc.issueDate,
        fileReference: doc.fileId
      }))
    };

    const response = await this.httpClient.post(
      "/api/v1/recycling-reports",
      payload,
      {
        headers: {
          "Authorization": `Bearer ${token}`,
          "Content-Type": "application/json"
        }
      }
    );

    return response.data;
  }

  // 의무이행 현황 조회
  async getObligationStatus(
    producerId: string,
    year: number
  ): Promise<ObligationStatus> {
    const token = await this.oauth.getAccessToken();

    const response = await this.httpClient.get(
      `/api/v1/producers/${producerId}/obligations/${year}`,
      {
        headers: { "Authorization": `Bearer ${token}` }
      }
    );

    return {
      year: response.data.year,
      producer: response.data.producer,
      categories: response.data.categories.map((cat: any) => ({
        categoryCode: cat.categoryCode,
        categoryName: cat.categoryName,
        shipmentQuantity: cat.shipmentQuantity,
        obligationRate: cat.obligationRate,
        obligationQuantity: cat.obligationQuantity,
        fulfilledQuantity: cat.fulfilledQuantity,
        fulfillmentRate: cat.fulfillmentRate,
        status: cat.status  // compliant, non_compliant, pending
      })),
      overallStatus: response.data.overallStatus,
      estimatedPenalty: response.data.estimatedPenalty
    };
  }

  // 분담금 정보 조회
  async getFeeInformation(
    producerId: string,
    year: number
  ): Promise<FeeInformation> {
    const token = await this.oauth.getAccessToken();

    const response = await this.httpClient.get(
      `/api/v1/producers/${producerId}/fees/${year}`,
      {
        headers: { "Authorization": `Bearer ${token}` }
      }
    );

    return {
      year: response.data.year,
      basicFee: response.data.basicFee,
      variableFee: response.data.variableFee,
      additionalCharges: response.data.additionalCharges,
      deductions: response.data.deductions,
      totalFee: response.data.totalFee,
      paidAmount: response.data.paidAmount,
      outstandingAmount: response.data.outstandingAmount,
      dueDate: response.data.dueDate,
      paymentInstructions: response.data.paymentInstructions
    };
  }
}
```

---

## 7.2 공제조합 시스템 통합

### 7.2.1 ECOAS 시스템 연계

```typescript
// 전자제품자원순환공제조합(ECOAS) 시스템 통합
class ECOASIntegration {
  private config: ECOASConfig;
  private apiClient: HTTPClient;

  constructor(config: ECOASConfig) {
    this.config = config;
    this.apiClient = new HTTPClient(config.baseUrl, {
      headers: {
        "X-API-Key": config.apiKey,
        "X-Member-ID": config.memberId
      }
    });
  }

  // 출고량 보고 (분기)
  async reportQuarterlyShipment(
    shipmentData: QuarterlyShipmentReport
  ): Promise<ReportResponse> {
    const payload = {
      회원번호: this.config.memberId,
      보고기간: {
        연도: shipmentData.year,
        분기: shipmentData.quarter
      },
      품목별출고량: shipmentData.items.map(item => ({
        품목코드: item.categoryCode,
        품목명: item.productName,
        모델수: item.modelCount,
        출고수량: item.quantity,
        출고중량_kg: item.weightKg,
        내수: item.domestic,
        수출: item.export
      })),
      신고자: {
        성명: shipmentData.reporter.name,
        연락처: shipmentData.reporter.phone,
        이메일: shipmentData.reporter.email
      }
    };

    const response = await this.apiClient.post(
      "/api/members/shipment-report",
      payload
    );

    return {
      reportId: response.접수번호,
      submissionDate: response.접수일시,
      status: response.상태,
      nextSteps: response.후속조치
    };
  }

  // 수거 요청
  async requestCollection(
    collectionRequest: CollectionRequest
  ): Promise<CollectionRequestResponse> {
    const payload = {
      회원번호: this.config.memberId,
      수거요청: {
        요청일: collectionRequest.requestDate,
        희망수거일: collectionRequest.preferredDate,
        수거장소: {
          주소: collectionRequest.location.address,
          상세주소: collectionRequest.location.detailAddress,
          연락처: collectionRequest.location.contactPhone,
          담당자: collectionRequest.location.contactPerson
        },
        수거품목: collectionRequest.items.map(item => ({
          품목코드: item.categoryCode,
          품목명: item.productName,
          예상수량: item.estimatedQuantity,
          예상중량_kg: item.estimatedWeightKg,
          상태: item.condition,
          특이사항: item.notes
        })),
        차량요청: collectionRequest.vehicleType,
        접근성정보: collectionRequest.accessInfo
      }
    };

    const response = await this.apiClient.post(
      "/api/collections/request",
      payload
    );

    return {
      requestId: response.요청번호,
      status: response.상태,
      scheduledDate: response.배정일,
      assignedCollector: response.수거업체,
      trackingUrl: response.추적URL
    };
  }

  // 수거 실적 조회
  async getCollectionHistory(
    criteria: CollectionHistoryCriteria
  ): Promise<CollectionHistory> {
    const params = {
      시작일: criteria.startDate,
      종료일: criteria.endDate,
      품목코드: criteria.categoryCode,
      상태: criteria.status,
      페이지: criteria.page,
      페이지크기: criteria.pageSize
    };

    const response = await this.apiClient.get(
      "/api/collections/history",
      { params }
    );

    return {
      총건수: response.총건수,
      총중량: response.총중량_kg,
      기록: response.기록.map((record: any) => ({
        수거번호: record.수거번호,
        수거일: record.수거일,
        품목: record.품목,
        수량: record.수량,
        중량: record.중량_kg,
        수거업체: record.수거업체,
        상태: record.상태,
        인증서번호: record.인증서번호
      })),
      페이지정보: response.페이지정보
    };
  }

  // 분담금 현황 조회
  async getFeeStatus(year: number): Promise<FeeStatus> {
    const response = await this.apiClient.get(
      `/api/members/fees/${year}`
    );

    return {
      연도: response.연도,
      회원번호: response.회원번호,
      회사명: response.회사명,
      분담금내역: {
        기본분담금: response.기본분담금,
        변동분담금: response.변동분담금,
        추가부담금: response.추가부담금,
        감면액: response.감면액,
        총분담금: response.총분담금
      },
      납부현황: {
        납부완료액: response.납부완료액,
        미납액: response.미납액,
        납부기한: response.납부기한,
        연체여부: response.연체여부
      },
      납부내역: response.납부내역.map((payment: any) => ({
        납부일: payment.납부일,
        납부금액: payment.납부금액,
        납부방법: payment.납부방법,
        영수증번호: payment.영수증번호
      }))
    };
  }

  // 재활용 인증서 조회
  async getRecyclingCertificates(
    criteria: CertificateCriteria
  ): Promise<RecyclingCertificate[]> {
    const response = await this.apiClient.get(
      "/api/certificates",
      {
        params: {
          시작일: criteria.startDate,
          종료일: criteria.endDate,
          품목코드: criteria.categoryCode
        }
      }
    );

    return response.인증서목록.map((cert: any) => ({
      인증서번호: cert.인증서번호,
      발급일: cert.발급일,
      품목: cert.품목,
      재활용량_kg: cert.재활용량_kg,
      처리업체: cert.처리업체,
      처리방법: cert.처리방법,
      유효기간: cert.유효기간,
      다운로드URL: cert.다운로드URL
    }));
  }
}

// ECOAS 통합 이벤트 핸들러
class ECOASEventHandler {
  private integration: ECOASIntegration;
  private eventEmitter: EventEmitter;

  constructor(integration: ECOASIntegration) {
    this.integration = integration;
    this.eventEmitter = new EventEmitter();
    this.setupWebhookHandlers();
  }

  private setupWebhookHandlers(): void {
    // 수거 완료 이벤트
    this.eventEmitter.on("collection_completed", async (data) => {
      console.log(`수거 완료: ${data.수거번호}`);

      // 로컬 데이터베이스 업데이트
      await this.updateLocalCollectionRecord(data);

      // 재고 시스템 업데이트
      await this.updateInventorySystem(data);
    });

    // 인증서 발급 이벤트
    this.eventEmitter.on("certificate_issued", async (data) => {
      console.log(`인증서 발급: ${data.인증서번호}`);

      // 인증서 다운로드 및 저장
      await this.downloadAndStoreCertificate(data);

      // EPR 시스템에 실적 반영
      await this.syncToEPRSystem(data);
    });

    // 분담금 청구 이벤트
    this.eventEmitter.on("fee_invoice", async (data) => {
      console.log(`분담금 청구: ${data.청구번호}`);

      // 재무 시스템에 전달
      await this.notifyFinanceSystem(data);
    });
  }

  // Webhook 엔드포인트 처리
  async handleWebhook(webhookData: any): Promise<void> {
    const eventType = webhookData.event_type;
    const payload = webhookData.payload;

    // 서명 검증
    if (!this.verifyWebhookSignature(webhookData)) {
      throw new Error("Invalid webhook signature");
    }

    // 이벤트 처리
    this.eventEmitter.emit(eventType, payload);
  }

  private verifyWebhookSignature(data: any): boolean {
    // HMAC 서명 검증 로직
    const expectedSignature = crypto
      .createHmac("sha256", this.integration.config.webhookSecret)
      .update(JSON.stringify(data.payload))
      .digest("hex");

    return data.signature === expectedSignature;
  }

  private async updateLocalCollectionRecord(data: any): Promise<void> {
    // 로컬 DB 업데이트 로직
  }

  private async updateInventorySystem(data: any): Promise<void> {
    // 재고 시스템 연동 로직
  }

  private async downloadAndStoreCertificate(data: any): Promise<void> {
    // 인증서 다운로드 및 저장 로직
  }

  private async syncToEPRSystem(data: any): Promise<void> {
    // EPR 시스템 동기화 로직
  }

  private async notifyFinanceSystem(data: any): Promise<void> {
    // 재무 시스템 알림 로직
  }
}
```

---

## 7.3 기업 시스템 통합

### 7.3.1 ERP 시스템 연동

```typescript
// ERP 시스템 통합 어댑터
interface ERPIntegrationAdapter {
  // 지원 ERP 시스템
  supportedSystems: ("SAP" | "Oracle" | "Microsoft_Dynamics" | "더존" | "영림원")[];

  // 데이터 매핑
  dataMapping: {
    masterData: {
      products: ProductDataMapping;
      vendors: VendorDataMapping;
      customers: CustomerDataMapping;
      materials: MaterialDataMapping;
    };
    transactions: {
      purchaseOrders: PODataMapping;
      salesOrders: SODataMapping;
      inventoryMovements: InventoryDataMapping;
      invoices: InvoiceDataMapping;
    };
  };

  // 동기화 설정
  syncConfiguration: {
    direction: "erp_to_wia" | "wia_to_erp" | "bidirectional";
    frequency: "realtime" | "batch";
    batchSchedule?: string;  // cron expression
    conflictResolution: "erp_wins" | "wia_wins" | "manual";
  };
}

// SAP 통합 구현
class SAPIntegration implements ERPIntegrationAdapter {
  private config: SAPConfig;
  private rfcClient: SAPRFCClient;
  private idocHandler: IDocHandler;

  constructor(config: SAPConfig) {
    this.config = config;
    this.rfcClient = new SAPRFCClient(config.rfcConfig);
    this.idocHandler = new IDocHandler(config.idocConfig);
  }

  // 제품 마스터 동기화
  async syncProductMaster(): Promise<SyncResult> {
    // SAP에서 제품 데이터 조회 (RFC 호출)
    const sapProducts = await this.rfcClient.call("BAPI_MATERIAL_GETLIST", {
      MATNRSELECTION: [{ SIGN: "I", OPTION: "CP", MATNR_LOW: "*" }],
      MATERIALTYPE: "FERT"  // 완제품
    });

    const mappedProducts: WIAProduct[] = [];

    for (const sapProduct of sapProducts.MATNRLIST) {
      // 상세 정보 조회
      const details = await this.rfcClient.call("BAPI_MATERIAL_GET_DETAIL", {
        MATERIAL: sapProduct.MATERIAL
      });

      // WIA 형식으로 매핑
      mappedProducts.push({
        productId: this.generateWIAProductId(sapProduct.MATERIAL),
        sapMaterialNumber: sapProduct.MATERIAL,
        productName: details.MATERIALNAME,
        category: this.mapToWIACategory(details.MATERIALTYPE, details.MATERIALGROUP),
        weight: {
          value: details.GROSSWEIGHT,
          unit: details.WEIGHTUNIT
        },
        materials: await this.getProductMaterialComposition(sapProduct.MATERIAL),
        eprCategory: this.determineEPRCategory(details),
        metadata: {
          sapPlant: details.PLANT,
          sapStorageLocation: details.STORAGELOCATION,
          lastSyncTimestamp: new Date().toISOString()
        }
      });
    }

    // WIA 시스템에 저장
    const saveResult = await this.saveToWIASystem(mappedProducts);

    return {
      processedCount: mappedProducts.length,
      successCount: saveResult.successCount,
      errorCount: saveResult.errorCount,
      errors: saveResult.errors
    };
  }

  // 재고 이동 연동
  async syncInventoryMovement(movement: InventoryMovement): Promise<void> {
    // IDoc 생성 (MBGMCR - 재고 이동)
    const idoc = this.idocHandler.createIDoc("MBGMCR01", {
      E1MBXSH: {
        BLDAT: formatSAPDate(movement.documentDate),
        BUDAT: formatSAPDate(movement.postingDate),
        BWART: this.mapMovementType(movement.movementType),
        XBLNR: movement.referenceDocument
      },
      E1MBXYH: movement.items.map(item => ({
        MATNR: item.materialNumber,
        WERKS: item.plant,
        LGORT: item.storageLocation,
        MENGE: item.quantity,
        MEINS: item.unit,
        KOSTL: item.costCenter,
        GRUND: this.mapMovementReason(item.reason)
      }))
    });

    // IDoc 전송
    await this.idocHandler.send(idoc);
  }

  // 공급업체(처리업체) 마스터 동기화
  async syncVendorMaster(vendor: WIAProcessingFacility): Promise<void> {
    // SAP 공급업체 생성/수정
    const result = await this.rfcClient.call("BAPI_VENDOR_CREATE", {
      COMPANYDATA: {
        BUKRS: this.config.companyCode,
        WAESSION: "KRW"
      },
      GENERALDATA: {
        NAME1: vendor.facilityName,
        SORTL: vendor.facilityId,
        STRAS: vendor.address,
        ORT01: vendor.city,
        LAND1: "KR",
        TELF1: vendor.phone
      },
      PURCHASINGDATA: {
        EKORG: this.config.purchasingOrg,
        WAERS: "KRW"
      }
    });

    if (result.RETURN.TYPE === "E") {
      throw new Error(`SAP Vendor creation failed: ${result.RETURN.MESSAGE}`);
    }
  }

  // 비용 전기
  async postRecyclingCost(costData: RecyclingCostData): Promise<void> {
    const result = await this.rfcClient.call("BAPI_ACC_DOCUMENT_POST", {
      DOCUMENTHEADER: {
        BUS_ACT: "RFBU",
        COMP_CODE: this.config.companyCode,
        DOC_DATE: formatSAPDate(costData.documentDate),
        PSTNG_DATE: formatSAPDate(costData.postingDate),
        DOC_TYPE: "SA",
        REF_DOC_NO: costData.referenceNumber,
        HEADER_TXT: `EPR 재활용비용 ${costData.period}`
      },
      ACCOUNTGL: [
        {
          ITEMNO_ACC: "1",
          GL_ACCOUNT: this.config.recyclingCostAccount,
          COMP_CODE: this.config.companyCode,
          COST_CTR: costData.costCenter,
          AMT_DOCCUR: costData.amount,
          CURRENCY: "KRW"
        }
      ],
      ACCOUNTPAYABLE: [
        {
          ITEMNO_ACC: "2",
          VENDOR_NO: costData.vendorNumber,
          COMP_CODE: this.config.companyCode,
          AMT_DOCCUR: -costData.amount,
          CURRENCY: "KRW"
        }
      ]
    });

    if (result.RETURN.some((r: any) => r.TYPE === "E")) {
      throw new Error("SAP posting failed");
    }
  }

  // 카테고리 매핑
  private mapToWIACategory(materialType: string, materialGroup: string): WasteCategory {
    const mappingTable: Record<string, WasteCategory> = {
      "ELEC01": WasteCategory.IT_EQUIPMENT,
      "ELEC02": WasteCategory.CONSUMER_ELECTRONICS,
      "ELEC03": WasteCategory.LARGE_HOUSEHOLD,
      "ELEC04": WasteCategory.SMALL_HOUSEHOLD,
      "BATT01": WasteCategory.BATTERIES
    };
    return mappingTable[materialGroup] || WasteCategory.OTHER;
  }

  private mapMovementType(type: string): string {
    const mapping: Record<string, string> = {
      "collection": "501",  // 폐기물 수거
      "processing": "502",  // 처리 투입
      "recovery": "503",    // 재활용 산출
      "disposal": "504"     // 최종 처분
    };
    return mapping[type] || "500";
  }
}

// 더존 ERP 통합
class DouzonIntegration implements ERPIntegrationAdapter {
  private config: DouzonConfig;
  private apiClient: HTTPClient;

  constructor(config: DouzonConfig) {
    this.config = config;
    this.apiClient = new HTTPClient(config.apiUrl, {
      headers: {
        "Authorization": `Bearer ${config.apiToken}`,
        "X-Company-Code": config.companyCode
      }
    });
  }

  // 제품 마스터 동기화
  async syncProductMaster(): Promise<SyncResult> {
    const response = await this.apiClient.get("/api/products", {
      params: { page: 1, pageSize: 1000 }
    });

    const products: WIAProduct[] = response.data.items.map((item: any) => ({
      productId: this.generateWIAProductId(item.productCode),
      douzonProductCode: item.productCode,
      productName: item.productName,
      category: this.mapCategory(item.category),
      weight: { value: item.weight, unit: "kg" },
      eprCategory: item.eprCategory,
      metadata: {
        lastSyncTimestamp: new Date().toISOString()
      }
    }));

    return await this.saveToWIASystem(products);
  }

  // 출고 데이터 조회 (EPR 신고용)
  async getShipmentData(startDate: string, endDate: string): Promise<ShipmentData[]> {
    const response = await this.apiClient.get("/api/sales/shipments", {
      params: { startDate, endDate, status: "shipped" }
    });

    return response.data.items.map((item: any) => ({
      shipmentId: item.shipmentNo,
      shipmentDate: item.shipmentDate,
      products: item.products.map((p: any) => ({
        productCode: p.productCode,
        productName: p.productName,
        quantity: p.quantity,
        weight: p.weight,
        eprCategory: p.eprCategory
      })),
      customer: {
        customerCode: item.customerCode,
        customerName: item.customerName,
        isExport: item.exportYn === "Y"
      }
    }));
  }

  // EPR 비용 전표 생성
  async createEPRCostVoucher(costData: EPRCostData): Promise<VoucherResponse> {
    const voucherData = {
      voucherType: "11",  // 일반전표
      voucherDate: costData.voucherDate,
      description: `EPR 분담금 ${costData.period}`,
      lines: [
        {
          accountCode: this.config.eprCostAccount,
          debitAmount: costData.amount,
          creditAmount: 0,
          costCenter: costData.costCenter,
          description: "EPR 재활용분담금"
        },
        {
          accountCode: this.config.payableAccount,
          debitAmount: 0,
          creditAmount: costData.amount,
          vendorCode: costData.consortiumCode,
          description: costData.consortiumName
        }
      ]
    };

    const response = await this.apiClient.post("/api/accounting/vouchers", voucherData);
    return response.data;
  }
}
```

### 7.3.2 레거시 시스템 통합

```typescript
// 레거시 시스템 통합 전략
interface LegacyIntegrationStrategy {
  // 통합 방식
  integrationApproaches: {
    // 1. 파일 기반 통합
    fileBasedIntegration: {
      description: "정기적인 파일 교환을 통한 데이터 동기화";
      formats: ("CSV" | "Excel" | "XML" | "JSON" | "EDI")[];
      transferMethods: ("SFTP" | "FTP" | "SharedFolder" | "Email")[];
      schedule: string;
      pros: string[];
      cons: string[];
    };

    // 2. 데이터베이스 연동
    databaseIntegration: {
      description: "직접 DB 연결을 통한 데이터 접근";
      supportedDatabases: ("Oracle" | "MSSQL" | "MySQL" | "PostgreSQL")[];
      connectionMethods: ("JDBC" | "ODBC" | "NativeDriver")[];
      considerations: string[];
    };

    // 3. API 래퍼
    apiWrapper: {
      description: "레거시 시스템을 위한 REST API 래퍼 구축";
      technology: string[];
      benefits: string[];
    };

    // 4. 메시지 큐
    messageQueue: {
      description: "비동기 메시지 기반 통합";
      platforms: ("RabbitMQ" | "Kafka" | "ActiveMQ")[];
      patterns: ("pub/sub" | "point-to-point" | "request-reply")[];
    };
  };
}

// 파일 기반 통합 구현
class FileBasedIntegration {
  private config: FileIntegrationConfig;
  private fileProcessor: FileProcessor;
  private scheduler: JobScheduler;

  constructor(config: FileIntegrationConfig) {
    this.config = config;
    this.fileProcessor = new FileProcessor();
    this.scheduler = new JobScheduler();
  }

  // 수출 작업 스케줄링
  scheduleExportJob(jobConfig: ExportJobConfig): void {
    this.scheduler.schedule(jobConfig.cronExpression, async () => {
      await this.exportData(jobConfig);
    });
  }

  // 데이터 내보내기
  async exportData(config: ExportJobConfig): Promise<ExportResult> {
    // 1. 데이터 조회
    const data = await this.fetchDataForExport(config.dataQuery);

    // 2. 형식 변환
    const formattedData = await this.formatData(data, config.format);

    // 3. 파일 생성
    const filename = this.generateFilename(config.filenamePattern);
    const filepath = path.join(config.outputDirectory, filename);
    await this.writeFile(filepath, formattedData);

    // 4. 파일 전송
    if (config.transferConfig) {
      await this.transferFile(filepath, config.transferConfig);
    }

    // 5. 결과 기록
    return {
      filename,
      recordCount: data.length,
      fileSize: fs.statSync(filepath).size,
      exportTimestamp: new Date().toISOString(),
      transferred: !!config.transferConfig
    };
  }

  // CSV 출고량 보고서 내보내기
  async exportShipmentReportCSV(
    startDate: string,
    endDate: string
  ): Promise<string> {
    const shipments = await db.query(`
      SELECT
        product_code,
        product_name,
        epr_category,
        SUM(quantity) as total_quantity,
        SUM(weight_kg) as total_weight
      FROM shipments
      WHERE shipment_date BETWEEN ? AND ?
      GROUP BY product_code, product_name, epr_category
    `, [startDate, endDate]);

    const csvContent = [
      ["품목코드", "품목명", "EPR분류", "출고수량", "출고중량(kg)"].join(","),
      ...shipments.map(s => [
        s.product_code,
        `"${s.product_name}"`,
        s.epr_category,
        s.total_quantity,
        s.total_weight
      ].join(","))
    ].join("\n");

    const filename = `shipment_report_${startDate}_${endDate}.csv`;
    const filepath = path.join(this.config.exportDir, filename);

    await fs.promises.writeFile(filepath, "\uFEFF" + csvContent, "utf8");  // BOM for Excel
    return filepath;
  }

  // 가져오기 작업
  async importData(importConfig: ImportConfig): Promise<ImportResult> {
    // 1. 파일 가져오기
    const files = await this.fetchFiles(importConfig.source);

    const results: ImportResult = {
      processedFiles: 0,
      totalRecords: 0,
      successRecords: 0,
      errorRecords: 0,
      errors: []
    };

    for (const file of files) {
      try {
        // 2. 파일 파싱
        const records = await this.parseFile(file, importConfig.format);

        // 3. 데이터 변환 및 검증
        const validatedRecords = await this.validateAndTransform(
          records,
          importConfig.validationRules,
          importConfig.transformations
        );

        // 4. 데이터 저장
        const saveResult = await this.saveRecords(validatedRecords, importConfig.targetTable);

        results.processedFiles++;
        results.totalRecords += records.length;
        results.successRecords += saveResult.successCount;
        results.errorRecords += saveResult.errorCount;
        results.errors.push(...saveResult.errors);

        // 5. 처리 완료 파일 이동
        await this.moveToProcessed(file, importConfig.processedDir);
      } catch (error) {
        results.errors.push({ file: file.name, error: error.message });
        await this.moveToError(file, importConfig.errorDir);
      }
    }

    return results;
  }
}

// 데이터베이스 직접 연동
class DirectDatabaseIntegration {
  private sourcePool: ConnectionPool;
  private targetPool: ConnectionPool;

  constructor(sourceConfig: DBConfig, targetConfig: DBConfig) {
    this.sourcePool = new ConnectionPool(sourceConfig);
    this.targetPool = new ConnectionPool(targetConfig);
  }

  // CDC (Change Data Capture) 기반 동기화
  async setupChangeCapture(tableConfig: CDCTableConfig): Promise<void> {
    // SQL Server CDC 활성화 예시
    await this.sourcePool.query(`
      EXEC sys.sp_cdc_enable_table
        @source_schema = N'${tableConfig.schema}',
        @source_name = N'${tableConfig.table}',
        @role_name = NULL,
        @capture_instance = N'${tableConfig.captureInstance}',
        @supports_net_changes = 1
    `);
  }

  // 변경 데이터 동기화
  async syncChanges(captureInstance: string, lastSyncLSN: string): Promise<SyncResult> {
    // 변경 데이터 조회
    const changes = await this.sourcePool.query(`
      SELECT
        __$operation,
        __$update_mask,
        *
      FROM cdc.fn_cdc_get_all_changes_${captureInstance}(
        ${lastSyncLSN},
        sys.fn_cdc_get_max_lsn(),
        'all'
      )
    `);

    const result: SyncResult = {
      inserts: 0,
      updates: 0,
      deletes: 0,
      errors: []
    };

    for (const change of changes) {
      try {
        switch (change.__$operation) {
          case 2:  // Insert
            await this.handleInsert(change);
            result.inserts++;
            break;
          case 4:  // Update
            await this.handleUpdate(change);
            result.updates++;
            break;
          case 1:  // Delete
            await this.handleDelete(change);
            result.deletes++;
            break;
        }
      } catch (error) {
        result.errors.push({ record: change, error: error.message });
      }
    }

    return result;
  }
}
```

---

## 7.4 실시간 데이터 동기화

### 7.4.1 이벤트 기반 아키텍처

```typescript
// 이벤트 기반 동기화 시스템
class EventDrivenSyncSystem {
  private eventBus: EventBus;
  private subscribers: Map<string, EventSubscriber[]>;

  constructor(eventBusConfig: EventBusConfig) {
    this.eventBus = new EventBus(eventBusConfig);
    this.subscribers = new Map();
    this.setupEventHandlers();
  }

  // 이벤트 발행
  async publishEvent(event: DomainEvent): Promise<void> {
    // 이벤트 저장 (Event Sourcing)
    await this.saveEvent(event);

    // 이벤트 발행
    await this.eventBus.publish(event.type, {
      eventId: event.id,
      timestamp: event.timestamp,
      aggregateId: event.aggregateId,
      aggregateType: event.aggregateType,
      payload: event.payload,
      metadata: event.metadata
    });
  }

  // 이벤트 핸들러 설정
  private setupEventHandlers(): void {
    // 장치 등록 이벤트
    this.eventBus.subscribe("DeviceRegistered", async (event) => {
      // ERP 시스템에 제품 등록
      await this.erpAdapter.registerProduct(event.payload);

      // EPR 시스템에 출고 대상 추가
      await this.eprAdapter.addShippableProduct(event.payload);
    });

    // 수거 완료 이벤트
    this.eventBus.subscribe("CollectionCompleted", async (event) => {
      // 재고 시스템 업데이트
      await this.inventoryAdapter.recordCollection(event.payload);

      // 올바로 시스템 신고
      await this.allbaroAdapter.reportCollection(event.payload);

      // 공제조합 실적 보고
      await this.consortiumAdapter.reportCollection(event.payload);
    });

    // 처리 완료 이벤트
    this.eventBus.subscribe("ProcessingCompleted", async (event) => {
      // 재활용 실적 기록
      await this.recyclingAdapter.recordProcessing(event.payload);

      // 올바로 처리완료 신고
      await this.allbaroAdapter.reportProcessingCompletion(event.payload);

      // 재료 재고 업데이트
      await this.inventoryAdapter.recordRecoveredMaterials(event.payload);

      // 비용 회계 처리
      await this.accountingAdapter.postProcessingCost(event.payload);
    });

    // 인증서 발급 이벤트
    this.eventBus.subscribe("CertificateIssued", async (event) => {
      // 문서 관리 시스템에 저장
      await this.documentAdapter.storeCertificate(event.payload);

      // EPR 실적 반영
      await this.eprAdapter.recordCertifiedRecycling(event.payload);
    });
  }

  // 이벤트 저장
  private async saveEvent(event: DomainEvent): Promise<void> {
    await db.query(`
      INSERT INTO event_store (
        event_id, event_type, aggregate_id, aggregate_type,
        payload, metadata, timestamp
      ) VALUES (?, ?, ?, ?, ?, ?, ?)
    `, [
      event.id,
      event.type,
      event.aggregateId,
      event.aggregateType,
      JSON.stringify(event.payload),
      JSON.stringify(event.metadata),
      event.timestamp
    ]);
  }
}

// Kafka 기반 이벤트 버스
class KafkaEventBus implements EventBus {
  private producer: KafkaProducer;
  private consumer: KafkaConsumer;
  private admin: KafkaAdmin;

  constructor(config: KafkaConfig) {
    const kafka = new Kafka({
      clientId: config.clientId,
      brokers: config.brokers,
      ssl: config.ssl,
      sasl: config.sasl
    });

    this.producer = kafka.producer();
    this.consumer = kafka.consumer({ groupId: config.consumerGroup });
    this.admin = kafka.admin();
  }

  async publish(topic: string, message: any): Promise<void> {
    await this.producer.send({
      topic: `wia-ewaste-${topic}`,
      messages: [
        {
          key: message.aggregateId,
          value: JSON.stringify(message),
          headers: {
            "event-type": topic,
            "timestamp": message.timestamp,
            "correlation-id": message.metadata?.correlationId
          }
        }
      ]
    });
  }

  async subscribe(topic: string, handler: EventHandler): Promise<void> {
    await this.consumer.subscribe({
      topic: `wia-ewaste-${topic}`,
      fromBeginning: false
    });

    await this.consumer.run({
      eachMessage: async ({ topic, partition, message }) => {
        try {
          const event = JSON.parse(message.value.toString());
          await handler(event);

          // 처리 성공 로그
          console.log(`Event processed: ${topic}, offset: ${message.offset}`);
        } catch (error) {
          // 오류 처리 (Dead Letter Queue로 전송)
          await this.sendToDeadLetterQueue(topic, message, error);
        }
      }
    });
  }

  private async sendToDeadLetterQueue(
    originalTopic: string,
    message: any,
    error: Error
  ): Promise<void> {
    await this.producer.send({
      topic: `wia-ewaste-dlq`,
      messages: [
        {
          key: message.key,
          value: message.value,
          headers: {
            "original-topic": originalTopic,
            "error-message": error.message,
            "failed-timestamp": new Date().toISOString()
          }
        }
      ]
    });
  }
}
```

### 7.4.2 데이터 일관성 관리

```typescript
// 분산 트랜잭션 관리 (Saga 패턴)
class RecyclingSaga {
  private sagaId: string;
  private state: SagaState;
  private steps: SagaStep[];

  constructor() {
    this.sagaId = generateUUID();
    this.state = { status: "pending", currentStep: 0, completedSteps: [] };
    this.steps = [];
  }

  // 재활용 처리 Saga 정의
  defineRecyclingProcessSaga(): void {
    this.steps = [
      {
        name: "reserveInventory",
        execute: async (context) => {
          return await this.inventoryService.reserve(context.items);
        },
        compensate: async (context, result) => {
          await this.inventoryService.releaseReservation(result.reservationId);
        }
      },
      {
        name: "createProcessingOrder",
        execute: async (context) => {
          return await this.processingService.createOrder(context);
        },
        compensate: async (context, result) => {
          await this.processingService.cancelOrder(result.orderId);
        }
      },
      {
        name: "updateAllbaro",
        execute: async (context, previousResults) => {
          return await this.allbaroService.reportHandover({
            items: context.items,
            orderId: previousResults.createProcessingOrder.orderId
          });
        },
        compensate: async (context, result) => {
          await this.allbaroService.cancelManifest(result.manifestNumber);
        }
      },
      {
        name: "updateERP",
        execute: async (context, previousResults) => {
          return await this.erpService.createMaterialMovement({
            type: "processing_transfer",
            items: context.items,
            reference: previousResults.createProcessingOrder.orderId
          });
        },
        compensate: async (context, result) => {
          await this.erpService.reverseMaterialMovement(result.documentNumber);
        }
      },
      {
        name: "notifyConsortium",
        execute: async (context, previousResults) => {
          return await this.consortiumService.reportProcessing({
            orderId: previousResults.createProcessingOrder.orderId,
            items: context.items
          });
        },
        compensate: async (context, result) => {
          // 알림 취소 (필요시)
          await this.consortiumService.cancelReport(result.reportId);
        }
      }
    ];
  }

  // Saga 실행
  async execute(context: SagaContext): Promise<SagaResult> {
    const results: Record<string, any> = {};

    try {
      for (let i = 0; i < this.steps.length; i++) {
        const step = this.steps[i];
        this.state.currentStep = i;

        console.log(`Executing step: ${step.name}`);

        const stepResult = await step.execute(context, results);
        results[step.name] = stepResult;

        this.state.completedSteps.push({
          name: step.name,
          result: stepResult,
          timestamp: new Date().toISOString()
        });

        // 중간 상태 저장
        await this.saveSagaState();
      }

      this.state.status = "completed";
      await this.saveSagaState();

      return {
        sagaId: this.sagaId,
        status: "success",
        results
      };
    } catch (error) {
      console.error(`Saga failed at step ${this.steps[this.state.currentStep].name}:`, error);

      // 보상 트랜잭션 실행
      await this.compensate(context, results);

      this.state.status = "compensated";
      await this.saveSagaState();

      return {
        sagaId: this.sagaId,
        status: "failed",
        error: error.message,
        compensated: true
      };
    }
  }

  // 보상 트랜잭션
  private async compensate(context: SagaContext, results: Record<string, any>): Promise<void> {
    // 완료된 단계를 역순으로 보상
    for (let i = this.state.completedSteps.length - 1; i >= 0; i--) {
      const completedStep = this.state.completedSteps[i];
      const step = this.steps.find(s => s.name === completedStep.name);

      if (step?.compensate) {
        try {
          console.log(`Compensating step: ${step.name}`);
          await step.compensate(context, completedStep.result);
        } catch (compensateError) {
          console.error(`Compensation failed for step ${step.name}:`, compensateError);
          // 보상 실패 기록 (수동 개입 필요)
          await this.recordCompensationFailure(step.name, compensateError);
        }
      }
    }
  }

  private async saveSagaState(): Promise<void> {
    await db.query(`
      INSERT INTO saga_state (saga_id, state, updated_at)
      VALUES (?, ?, NOW())
      ON DUPLICATE KEY UPDATE state = ?, updated_at = NOW()
    `, [this.sagaId, JSON.stringify(this.state), JSON.stringify(this.state)]);
  }

  private async recordCompensationFailure(stepName: string, error: Error): Promise<void> {
    await db.query(`
      INSERT INTO compensation_failures (saga_id, step_name, error_message, created_at)
      VALUES (?, ?, ?, NOW())
    `, [this.sagaId, stepName, error.message]);

    // 관리자 알림
    await this.notificationService.alertAdmins({
      type: "COMPENSATION_FAILURE",
      sagaId: this.sagaId,
      stepName,
      error: error.message
    });
  }
}

// 데이터 충돌 해결
class ConflictResolver {
  // 충돌 감지
  detectConflict(localData: any, remoteData: any): Conflict | null {
    if (localData.version !== remoteData.baseVersion) {
      return {
        type: "version_mismatch",
        localVersion: localData.version,
        remoteVersion: remoteData.version,
        localData,
        remoteData,
        conflictingFields: this.identifyConflictingFields(localData, remoteData)
      };
    }
    return null;
  }

  // 충돌 해결 전략
  resolveConflict(conflict: Conflict, strategy: ConflictResolutionStrategy): any {
    switch (strategy) {
      case "local_wins":
        return conflict.localData;

      case "remote_wins":
        return conflict.remoteData;

      case "latest_wins":
        return conflict.localData.updatedAt > conflict.remoteData.updatedAt
          ? conflict.localData
          : conflict.remoteData;

      case "merge":
        return this.mergeData(conflict);

      case "manual":
        throw new ManualResolutionRequired(conflict);

      default:
        throw new Error(`Unknown resolution strategy: ${strategy}`);
    }
  }

  // 데이터 병합
  private mergeData(conflict: Conflict): any {
    const merged = { ...conflict.localData };

    for (const field of conflict.conflictingFields) {
      // 필드별 병합 규칙 적용
      const rule = this.getMergeRule(field);
      merged[field] = rule(conflict.localData[field], conflict.remoteData[field]);
    }

    // 버전 증가
    merged.version = Math.max(conflict.localData.version, conflict.remoteData.version) + 1;
    merged.mergedAt = new Date().toISOString();
    merged.mergeSource = ["local", "remote"];

    return merged;
  }

  private getMergeRule(field: string): MergeRule {
    const rules: Record<string, MergeRule> = {
      quantity: (local, remote) => local + remote,  // 수량은 합산
      status: (local, remote) => {
        // 상태는 진행도가 높은 것 선택
        const statusOrder = ["pending", "in_progress", "completed"];
        return statusOrder.indexOf(local) > statusOrder.indexOf(remote) ? local : remote;
      },
      notes: (local, remote) => `${local}\n---\n${remote}`,  // 노트는 병합
      default: (local, remote) => remote  // 기본은 원격 우선
    };
    return rules[field] || rules.default;
  }
}
```

---

## 7.5 보안 및 모니터링

### 7.5.1 통합 보안

```typescript
// 통합 보안 프레임워크
interface IntegrationSecurityFramework {
  // 인증
  authentication: {
    methods: {
      apiKey: {
        headerName: string;
        rotationPolicy: string;
        scopeBasedAccess: boolean;
      };
      oauth2: {
        authorizationServer: string;
        grantTypes: ("client_credentials" | "authorization_code")[];
        scopes: string[];
        tokenExpiry: number;
      };
      certificate: {
        type: "공인인증서" | "사설인증서" | "TLS_mutual";
        validationMethod: string;
      };
    };
  };

  // 암호화
  encryption: {
    inTransit: {
      protocol: "TLS 1.2" | "TLS 1.3";
      cipherSuites: string[];
    };
    atRest: {
      algorithm: "AES-256-GCM";
      keyManagement: "KMS" | "HSM" | "internal";
      keyRotation: string;
    };
    fieldLevel: {
      encryptedFields: string[];
      algorithm: string;
    };
  };

  // 접근 제어
  accessControl: {
    rbac: {
      roles: {
        roleName: string;
        permissions: string[];
        dataScope: string;
      }[];
    };
    ipWhitelist: string[];
    rateLimit: {
      requestsPerMinute: number;
      burstLimit: number;
    };
  };

  // 감사 로깅
  auditLogging: {
    events: string[];
    retention: string;
    tamperProtection: boolean;
    complianceStandards: string[];
  };
}

// 보안 미들웨어
class SecurityMiddleware {
  // API 키 검증
  async validateApiKey(req: Request, res: Response, next: NextFunction): Promise<void> {
    const apiKey = req.headers["x-api-key"] as string;

    if (!apiKey) {
      res.status(401).json({ error: "API key required" });
      return;
    }

    const keyInfo = await this.apiKeyStore.validate(apiKey);

    if (!keyInfo.valid) {
      res.status(401).json({ error: "Invalid API key" });
      return;
    }

    if (keyInfo.expired) {
      res.status(401).json({ error: "API key expired" });
      return;
    }

    // 범위 확인
    if (!this.hasRequiredScope(keyInfo.scopes, req.path, req.method)) {
      res.status(403).json({ error: "Insufficient permissions" });
      return;
    }

    // 요청 컨텍스트에 인증 정보 추가
    req.auth = {
      clientId: keyInfo.clientId,
      scopes: keyInfo.scopes,
      organizationId: keyInfo.organizationId
    };

    next();
  }

  // 레이트 리미팅
  createRateLimiter(config: RateLimitConfig): RequestHandler {
    return rateLimit({
      windowMs: config.windowMs || 60000,
      max: config.max || 100,
      keyGenerator: (req) => req.auth?.clientId || req.ip,
      handler: (req, res) => {
        res.status(429).json({
          error: "Too many requests",
          retryAfter: Math.ceil(config.windowMs / 1000)
        });
      }
    });
  }

  // 감사 로깅
  auditLog(req: Request, res: Response, next: NextFunction): void {
    const startTime = Date.now();

    res.on("finish", async () => {
      const auditEntry = {
        timestamp: new Date().toISOString(),
        clientId: req.auth?.clientId,
        method: req.method,
        path: req.path,
        query: req.query,
        statusCode: res.statusCode,
        responseTime: Date.now() - startTime,
        ip: req.ip,
        userAgent: req.headers["user-agent"],
        requestId: req.headers["x-request-id"]
      };

      await this.auditLogger.log(auditEntry);
    });

    next();
  }

  // 민감 데이터 마스킹
  maskSensitiveData(data: any): any {
    const sensitiveFields = [
      "사업자등록번호",
      "대표자주민번호",
      "계좌번호",
      "연락처"
    ];

    const masked = JSON.parse(JSON.stringify(data));

    for (const field of sensitiveFields) {
      if (masked[field]) {
        masked[field] = this.mask(masked[field]);
      }
    }

    return masked;
  }

  private mask(value: string): string {
    if (value.length <= 4) return "****";
    return value.substring(0, 2) + "*".repeat(value.length - 4) + value.substring(value.length - 2);
  }
}
```

### 7.5.2 통합 모니터링

```typescript
// 통합 모니터링 대시보드
interface IntegrationMonitoringDashboard {
  // 연결 상태
  connectionStatus: {
    systems: {
      systemName: string;
      status: "healthy" | "degraded" | "down";
      lastCheck: string;
      responseTime: number;
      errorRate: number;
    }[];
  };

  // 동기화 상태
  syncStatus: {
    jobs: {
      jobName: string;
      lastRun: string;
      lastSuccess: string;
      status: "success" | "failed" | "running";
      recordsProcessed: number;
      errors: number;
      nextScheduled: string;
    }[];
  };

  // 데이터 품질
  dataQuality: {
    metrics: {
      metricName: string;
      value: number;
      threshold: number;
      status: "good" | "warning" | "critical";
    }[];
  };

  // 알림
  alerts: {
    activeAlerts: {
      alertId: string;
      severity: "info" | "warning" | "critical";
      message: string;
      timestamp: string;
      acknowledged: boolean;
    }[];
  };
}

// 모니터링 서비스 구현
class IntegrationMonitoringService {
  private healthChecks: Map<string, HealthCheck>;
  private metrics: MetricsCollector;
  private alertManager: AlertManager;

  constructor() {
    this.healthChecks = new Map();
    this.metrics = new MetricsCollector();
    this.alertManager = new AlertManager();
  }

  // 헬스체크 등록
  registerHealthCheck(systemName: string, check: HealthCheck): void {
    this.healthChecks.set(systemName, check);
  }

  // 주기적 헬스체크 실행
  async runHealthChecks(): Promise<HealthCheckResults> {
    const results: HealthCheckResults = {
      timestamp: new Date().toISOString(),
      systems: []
    };

    for (const [systemName, check] of this.healthChecks) {
      const startTime = Date.now();

      try {
        const checkResult = await Promise.race([
          check.execute(),
          this.timeout(check.timeoutMs || 5000)
        ]);

        results.systems.push({
          systemName,
          status: checkResult.healthy ? "healthy" : "degraded",
          responseTime: Date.now() - startTime,
          details: checkResult.details
        });
      } catch (error) {
        results.systems.push({
          systemName,
          status: "down",
          responseTime: Date.now() - startTime,
          error: error.message
        });

        // 알림 발생
        await this.alertManager.createAlert({
          severity: "critical",
          title: `${systemName} 연결 실패`,
          message: error.message,
          source: "health_check"
        });
      }
    }

    // 메트릭 기록
    await this.recordHealthMetrics(results);

    return results;
  }

  // 동기화 작업 모니터링
  async monitorSyncJob(jobId: string, execution: () => Promise<any>): Promise<SyncJobResult> {
    const startTime = Date.now();
    const jobMetrics = {
      jobId,
      startTime: new Date().toISOString(),
      endTime: "",
      duration: 0,
      status: "running" as const,
      recordsProcessed: 0,
      errors: []
    };

    try {
      const result = await execution();

      jobMetrics.endTime = new Date().toISOString();
      jobMetrics.duration = Date.now() - startTime;
      jobMetrics.status = "success";
      jobMetrics.recordsProcessed = result.recordsProcessed || 0;

      // 성공 메트릭 기록
      this.metrics.increment("sync_jobs_success", { jobId });
      this.metrics.histogram("sync_job_duration", jobMetrics.duration, { jobId });
      this.metrics.gauge("sync_records_processed", jobMetrics.recordsProcessed, { jobId });

      return { ...jobMetrics, result };
    } catch (error) {
      jobMetrics.endTime = new Date().toISOString();
      jobMetrics.duration = Date.now() - startTime;
      jobMetrics.status = "failed";
      jobMetrics.errors.push(error.message);

      // 실패 메트릭 및 알림
      this.metrics.increment("sync_jobs_failed", { jobId });

      await this.alertManager.createAlert({
        severity: "warning",
        title: `동기화 작업 실패: ${jobId}`,
        message: error.message,
        source: "sync_monitor"
      });

      throw error;
    }
  }

  // 데이터 품질 모니터링
  async checkDataQuality(): Promise<DataQualityReport> {
    const checks = [
      {
        name: "중복 장치 ID",
        query: "SELECT COUNT(*) as count FROM devices GROUP BY wdid HAVING count > 1",
        threshold: 0,
        severity: "critical"
      },
      {
        name: "누락된 수거 기록",
        query: `SELECT COUNT(*) FROM collections WHERE manifest_id IS NULL AND status = 'completed'`,
        threshold: 10,
        severity: "warning"
      },
      {
        name: "불일치 재고",
        query: `SELECT COUNT(*) FROM inventory WHERE wia_quantity != erp_quantity`,
        threshold: 5,
        severity: "warning"
      },
      {
        name: "미동기화 레코드",
        query: `SELECT COUNT(*) FROM sync_queue WHERE status = 'pending' AND created_at < DATE_SUB(NOW(), INTERVAL 1 HOUR)`,
        threshold: 100,
        severity: "warning"
      }
    ];

    const results: DataQualityMetric[] = [];

    for (const check of checks) {
      const [row] = await db.query(check.query);
      const value = row.count || row[Object.keys(row)[0]];

      const status = value <= check.threshold ? "good" :
                     value <= check.threshold * 2 ? "warning" : "critical";

      results.push({
        name: check.name,
        value,
        threshold: check.threshold,
        status,
        checkedAt: new Date().toISOString()
      });

      if (status !== "good") {
        await this.alertManager.createAlert({
          severity: check.severity,
          title: `데이터 품질 문제: ${check.name}`,
          message: `현재 값: ${value}, 임계값: ${check.threshold}`,
          source: "data_quality"
        });
      }
    }

    return {
      timestamp: new Date().toISOString(),
      metrics: results,
      overallStatus: results.some(r => r.status === "critical") ? "critical" :
                     results.some(r => r.status === "warning") ? "warning" : "good"
    };
  }

  private timeout(ms: number): Promise<never> {
    return new Promise((_, reject) => {
      setTimeout(() => reject(new Error("Health check timeout")), ms);
    });
  }
}
```

---

## 7.6 요약 및 핵심 사항

### 통합 아키텍처 요약

| 통합 대상 | 프로토콜 | 인증 방식 | 데이터 유형 | 동기화 방식 |
|----------|---------|----------|-----------|-----------|
| 올바로 시스템 | SOAP | 공인인증서 | 인계서, 실적 | 실시간 |
| EPR 시스템 | REST | OAuth 2.0 | 출고량, 재활용 | 배치 (분기) |
| ECOAS | REST | API Key | 수거, 분담금 | 실시간+배치 |
| SAP ERP | RFC/IDoc | SAP 인증 | 제품, 재고, 회계 | 양방향 |
| 더존 ERP | REST | API Token | 제품, 출고, 회계 | 양방향 |
| 레거시 | 파일/DB | 다양 | 다양 | 배치 |

### 통합 체크리스트

**설계 단계:**
- [ ] 통합 대상 시스템 식별 및 우선순위 결정
- [ ] 데이터 매핑 및 변환 규칙 정의
- [ ] 인증 및 보안 요구사항 확인
- [ ] 동기화 전략 (실시간/배치) 결정

**구현 단계:**
- [ ] API 클라이언트 개발 및 테스트
- [ ] 오류 처리 및 재시도 로직 구현
- [ ] 데이터 변환 로직 구현
- [ ] 충돌 해결 전략 구현

**운영 단계:**
- [ ] 헬스체크 설정
- [ ] 모니터링 대시보드 구성
- [ ] 알림 규칙 설정
- [ ] 정기 감사 일정 수립

---

## 복습 질문

1. 올바로 시스템과 WIA 전자폐기물 관리 시스템 간의 통합에서 전자인계서 발행 절차를 설명하세요.

2. EPR 통합관리시스템에서 출고량 신고와 재활용실적 보고의 차이점과 각각의 보고 주기는 무엇인가요?

3. ECOAS 공제조합 시스템과의 통합에서 웹훅(Webhook)을 활용한 이벤트 처리 방식을 설명하세요.

4. SAP ERP 시스템과의 통합에서 제품 마스터 동기화 시 데이터 매핑의 주요 고려사항은 무엇인가요?

5. 이벤트 기반 아키텍처에서 Saga 패턴을 사용한 분산 트랜잭션 관리의 장점과 보상 트랜잭션의 역할을 설명하세요.

6. 레거시 시스템 통합 시 파일 기반 통합과 데이터베이스 직접 연동 방식의 장단점을 비교하세요.

7. 통합 모니터링에서 데이터 품질 검사의 주요 항목과 임계값 설정 기준은 무엇인가요?

---

## 다음 장 예고

제8장에서는 **구현 가이드**를 다룹니다. 실제 프로젝트에서 WIA 전자폐기물 관리 표준을 도입하는 단계별 절차와 모범 사례를 학습합니다.

---

*WIA 전자폐기물 관리 표준 - 제7장 완료*
