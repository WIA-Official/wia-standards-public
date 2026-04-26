# 제6장: 리콜 관리 및 위기 대응

**WIA-AGRI-016 전자책 시리즈**

---

## 신속한 리콜의 중요성

식품 안전 사고는 빠르게 확대될 수 있습니다. 효과적인 이력 추적은 영향받은 제품의 신속한 식별 및 제거를 가능하게 하여 공중 보건 위험과 재정적 손해를 최소화합니다.

**통계:**
- 평균 식품 리콜 비용: 100억원
- 브랜드 가치 손실: 심각한 경우 20-30%
- 법적 책임은 1,000억원 초과 가능
- 역추적 시간: 2-7일 (기존) vs. < 2시간 (디지털)

---

## 리콜 분류

### 한국 식품의약품안전처(식약처) 분류

**1등급:** 심각한 건강 상 악영향 또는 사망을 초래할 합리적인 가능성
- 예시: 즉석섭취식품의 리스테리아 오염
- 조치: 즉시 공개 발표, 전체 제품 제거

**2등급:** 일시적 또는 의학적으로 회복 가능한 건강상 악영향
- 예시: 미표시 알레르기 유발 물질
- 조치: 공개 발표, 표적 제거

**3등급:** 건강상 악영향을 초래할 가능성이 없음
- 예시: 라벨 오류
- 조치: 제한적 유통, 시정 조치

---

## 신속한 역추적 시스템

### 구현

```javascript
class RecallManagementSystem {
  async initiateRecall(incidentDetails) {
    const recallId = this.generateRecallId();

    // 1단계: 영향받은 배치 식별 (< 1시간)
    const affectedBatches = await this.identifyAffectedBatches(
      incidentDetails.rootCause
    );

    // 2단계: 모든 현재 위치로 순방향 추적 (< 2시간)
    const distributionMap = await Promise.all(
      affectedBatches.map(batch => this.traceBatchDistribution(batch))
    );

    // 3단계: 모든 이해관계자에게 통보 (즉시)
    await this.notifyStakeholders(recallId, distributionMap);

    // 4단계: 리콜 효과성 추적
    this.trackRecallProgress(recallId);

    return {
      recallId,
      affectedBatches,
      estimatedUnits: this.calculateTotalUnits(distributionMap),
      locationsNotified: this.countLocations(distributionMap),
      initiatedAt: new Date().toISOString()
    };
  }

  async identifyAffectedBatches(rootCause) {
    if (rootCause.type === 'supplier_contamination') {
      // 오염된 성분을 사용한 모든 배치 찾기
      return await this.findBatchesWithIngredient(
        rootCause.supplierBatch,
        rootCause.dateRange
      );
    }

    if (rootCause.type === 'processing_issue') {
      // 영향받은 장비/시설의 모든 배치 찾기
      return await this.findBatchesByProcessing(
        rootCause.facilityGLN,
        rootCause.equipmentId,
        rootCause.dateRange
      );
    }

    // 직접 배치 식별
    return [rootCause.batchId];
  }

  async traceBatchDistribution(batchId) {
    // 모든 출하/수령 이벤트에 대해 EPCIS 쿼리
    const events = await this.epcisQuery.getEventsByBatch(batchId);

    // 유통 트리 구축
    const currentLocations = this.findCurrentLocations(events);
    const customerPurchases = this.findConsumerPurchases(events);

    return {
      batchId,
      productName: events[0].product,
      quantity: events[0].quantity,
      currentLocations,
      customerPurchases,
      distributionPath: this.buildPath(events)
    };
  }

  async notifyStakeholders(recallId, distributionMap) {
    const notifications = [];

    // 물류센터 및 소매업체에 통보
    for (const batch of distributionMap) {
      for (const location of batch.currentLocations) {
        notifications.push(
          this.sendRecallNotification({
            recipient: location.contacts.foodSafety,
            urgency: '긴급',
            recallId,
            batchId: batch.batchId,
            product: batch.productName,
            action: '즉시 진열대에서 제거',
            reportingUrl: `https://recall.wia.org/${recallId}/report`
          })
        );
      }
    }

    // 소비자 통보 (식별 가능한 경우)
    for (const batch of distributionMap) {
      for (const purchase of batch.customerPurchases) {
        if (purchase.customerId) {
          notifications.push(
            this.sendConsumerAlert({
              recipient: purchase.contactInfo,
              urgency: '높음',
              product: batch.productName,
              action: '섭취하지 마십시오. 전액 환불을 위해 반품하십시오.',
              healthRisk: this.getHealthRiskStatement(recallId)
            })
          );
        }
      }
    }

    await Promise.all(notifications);

    return {
      totalNotifications: notifications.length,
      sentAt: new Date().toISOString()
    };
  }
}
```

### 리콜 효과성 추적

```javascript
async function trackRecallEffectiveness(recallId) {
  const recall = await getRecallDetails(recallId);
  const checkIns = await getRecallCheckIns(recallId);

  const effectiveness = {
    recallId,
    totalUnitsAffected: recall.estimatedUnits,
    unitsRecovered: checkIns.reduce((sum, ci) => sum + ci.unitsReturned, 0),
    recoveryRate: 0,
    byChannel: {}
  };

  effectiveness.recoveryRate =
    (effectiveness.unitsRecovered / effectiveness.totalUnitsAffected) * 100;

  // 채널별 분류
  const channels = ['distribution', 'retail', 'consumer'];
  for (const channel of channels) {
    const channelCheckIns = checkIns.filter(ci => ci.channel === channel);
    effectiveness.byChannel[channel] = {
      unitsAffected: recall[`${channel}Units`],
      unitsRecovered: channelCheckIns.reduce((sum, ci) => sum + ci.unitsReturned, 0),
      recoveryRate: 0
    };
    effectiveness.byChannel[channel].recoveryRate =
      (effectiveness.byChannel[channel].unitsRecovered /
       effectiveness.byChannel[channel].unitsAffected) * 100;
  }

  return effectiveness;
}
```

---

## 근본 원인 분석

### 오염원 추적

```javascript
async function performRootCauseAnalysis(affectedBatches) {
  // 영향받은 배치 간 공통 요인 식별
  const commonFactors = {
    sharedSuppliers: [],
    sharedEquipment: [],
    sharedFacilities: [],
    sharedTimeframes: []
  };

  // 공유 공급업체 찾기
  for (const batch of affectedBatches) {
    const ingredients = await getIngredients(batch);
    for (const ingredient of ingredients) {
      if (allBatchesContain(affectedBatches, ingredient.supplierBatch)) {
        commonFactors.sharedSuppliers.push({
          supplier: ingredient.supplier,
          ingredientBatch: ingredient.supplierBatch,
          probability: '높음'
        });
      }
    }
  }

  // 공유 가공 장비 찾기
  for (const batch of affectedBatches) {
    const processing = await getProcessingDetails(batch);
    if (allBatchesProcessedOn(affectedBatches, processing.equipmentId)) {
      commonFactors.sharedEquipment.push({
        equipmentId: processing.equipmentId,
        facility: processing.facility,
        probability: '높음'
      });
    }
  }

  return {
    affectedBatchCount: affectedBatches.length,
    commonFactors,
    likelyRootCause: determineMostLikely(commonFactors)
  };
}
```

---

## 위기 커뮤니케이션

### 다중 채널 통보

```javascript
class CrisisNotificationSystem {
  async sendEmergencyNotification(recall) {
    // 모든 이해관계자에게 이메일
    await this.sendEmailNotifications({
      recipients: recall.stakeholderEmails,
      subject: `긴급 리콜: ${recall.product}`,
      body: this.generateEmailBody(recall)
    });

    // 주요 인력에게 SMS
    await this.sendSMS({
      recipients: recall.emergencyContacts,
      message: `리콜 개시: ${recall.product}. 이메일에서 세부 정보를 확인하세요.`
    });

    // 모바일 앱 푸시 알림
    await this.sendPushNotifications({
      audience: recall.affectedRetailers,
      title: '제품 리콜 경고',
      body: recall.shortDescription,
      data: { recallId: recall.recallId }
    });

    // 공개 웹사이트 발표
    await this.publishWebAnnouncement(recall);

    // 규제 기관에 제출
    await this.submitToRegulators(recall);
  }
}
```

---

## 규제 보고

### 한국 식약처 준수

```javascript
async function generateKFDAReport(recallId) {
  const recall = await getRecallDetails(recallId);

  const report = {
    reportType: 'KFDA_RECALL',
    recallId: recallId,
    firmName: recall.firmName,
    productDescription: recall.product,

    // 중요 추적 이벤트
    criticalTrackingEvents: await getCTEs(recall.affectedBatches),

    // 핵심 데이터 요소
    keyDataElements: {
      traceabilityLotCode: recall.affectedBatches.map(b => b.batchId),
      productQuantity: recall.totalUnitsAffected,
      locationDescription: recall.facilityName,
      originReference: recall.originGLN
    },

    // 즉시 공급업체 및 수령인
    immediateSuppliers: await getImmediateSuppliers(recall.affectedBatches),
    immediateRecipients: await getImmediateRecipients(recall.affectedBatches),

    // 현재 상태
    recallStatus: recall.status,
    unitsRecovered: recall.unitsRecovered,
    effectivenessRate: recall.effectivenessRate
  };

  // 식약처 포털에 제출
  await submitToKFDA(report);

  return report;
}
```

---

## 장 요약

효과적인 리콜 관리에는 다음이 필요합니다:

**신속한 역추적:**
- < 1시간 내 영향받은 배치 식별
- < 2시간 내 완전한 유통 매핑
- 즉시 이해관계자 통보

**근본 원인 분석:**
- 공통 공급업체, 장비, 기간 식별
- 향후 사고 방지

**위기 커뮤니케이션:**
- 다중 채널 통보
- 소비자 안전 우선순위
- 규제 준수

**효과성 추적:**
- 실시간 회수 모니터링
- 채널별 분석

---

## 다음 장

**제7장: 소비자 참여 및 투명성**

소비자 대면 이력 추적 경험 구축 방법을 배웁니다.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
