# 제6장: 통합

## 금융 및 법률 시스템과의 연결

### 소개

WIA Cryo-Asset 표준은 외부 금융 기관, 법률 서비스, 규제 시스템 및 블록체인 네트워크와의 원활한 통합을 요구합니다. 이 장에서는 보안, 신뢰성 및 장기 데이터 접근성을 유지하면서 냉동 보존 자산 관리 플랫폼을 광범위한 금융 및 법률 생태계와 연결하는 데 필요한 통합 패턴, 프로토콜 및 구현을 자세히 설명합니다.

---

## 6.1 통합 아키텍처

### 시스템 통합 개요

```typescript
// 통합 아키텍처 개요
interface IntegrationArchitecture {
  layers: {
    presentation: IntegrationLayer;      // 프레젠테이션 계층
    orchestration: IntegrationLayer;     // 오케스트레이션 계층
    adapter: IntegrationLayer;           // 어댑터 계층
    protocol: IntegrationLayer;          // 프로토콜 계층
  };

  patterns: {
    sync: SyncPattern[];                 // 동기 패턴
    async: AsyncPattern[];               // 비동기 패턴
    realtime: RealtimePattern[];         // 실시간 패턴
  };

  security: {
    authentication: AuthMethod[];        // 인증 방식
    authorization: AuthzModel;           // 권한 부여 모델
    encryption: EncryptionStandard[];    // 암호화 표준
  };

  resilience: {
    circuitBreaker: CircuitBreakerConfig; // 서킷 브레이커
    retry: RetryConfig;                   // 재시도 설정
    fallback: FallbackConfig;             // 폴백 설정
  };
}

// 통합 오케스트레이터 서비스
class IntegrationOrchestrator {
  private adapters: Map<string, IntegrationAdapter>;
  private circuitBreaker: CircuitBreakerService;
  private eventBus: EventBus;

  constructor(config: IntegrationConfig) {
    this.adapters = new Map();
    this.circuitBreaker = new CircuitBreakerService(config.circuitBreaker);
    this.eventBus = new EventBus(config.eventBus);

    this.initializeAdapters(config.adapters);
  }

  // 어댑터 초기화
  private initializeAdapters(adapterConfigs: AdapterConfig[]): void {
    for (const config of adapterConfigs) {
      const adapter = this.createAdapter(config);
      this.adapters.set(config.id, adapter);
    }
  }

  // 어댑터 유형별 생성
  private createAdapter(config: AdapterConfig): IntegrationAdapter {
    switch (config.type) {
      case 'BANKING':
        return new BankingAdapter(config);
      case 'BROKERAGE':
        return new BrokerageAdapter(config);
      case 'INSURANCE':
        return new InsuranceAdapter(config);
      case 'LEGAL':
        return new LegalServicesAdapter(config);
      case 'BLOCKCHAIN':
        return new BlockchainAdapter(config);
      case 'TAX':
        return new TaxAuthorityAdapter(config);
      case 'CRYONICS_ORG':
        return new CryonicsOrgAdapter(config);
      default:
        throw new Error(`알 수 없는 어댑터 유형: ${config.type}`);
    }
  }

  // 복원력 있는 통합 작업 실행
  async execute<T>(
    adapterId: string,
    operation: string,
    params: Record<string, any>
  ): Promise<IntegrationResult<T>> {
    const adapter = this.adapters.get(adapterId);
    if (!adapter) {
      throw new Error(`어댑터를 찾을 수 없음: ${adapterId}`);
    }

    // 서킷 브레이커 상태 확인
    if (!this.circuitBreaker.isAllowed(adapterId)) {
      return {
        success: false,
        error: {
          code: 'CIRCUIT_OPEN',
          message: `${adapterId}에 대한 서킷 브레이커가 열려있습니다`,
        },
        fallbackUsed: true,
        fallbackData: await adapter.getFallbackData(operation, params),
      };
    }

    try {
      const result = await this.executeWithRetry(adapter, operation, params);

      this.circuitBreaker.recordSuccess(adapterId);

      // 성공 이벤트 발생
      this.eventBus.emit('integration:success', {
        adapterId,
        operation,
        timestamp: new Date(),
      });

      return {
        success: true,
        data: result,
      };
    } catch (error) {
      this.circuitBreaker.recordFailure(adapterId);

      // 실패 이벤트 발생
      this.eventBus.emit('integration:failure', {
        adapterId,
        operation,
        error: error.message,
        timestamp: new Date(),
      });

      return {
        success: false,
        error: {
          code: error.code || 'INTEGRATION_ERROR',
          message: error.message,
          details: error.details,
        },
      };
    }
  }

  // 재시도 로직을 포함한 실행
  private async executeWithRetry<T>(
    adapter: IntegrationAdapter,
    operation: string,
    params: Record<string, any>,
    attempt: number = 1
  ): Promise<T> {
    try {
      return await adapter.execute(operation, params);
    } catch (error) {
      if (this.shouldRetry(error, attempt)) {
        const delay = this.calculateBackoff(attempt);
        await this.sleep(delay);
        return this.executeWithRetry(adapter, operation, params, attempt + 1);
      }
      throw error;
    }
  }

  // 재시도 여부 결정
  private shouldRetry(error: any, attempt: number): boolean {
    const maxRetries = 3;
    const retryableErrors = ['TIMEOUT', 'NETWORK_ERROR', 'SERVICE_UNAVAILABLE'];

    return attempt < maxRetries &&
           retryableErrors.includes(error.code);
  }

  // 지수 백오프 계산
  private calculateBackoff(attempt: number): number {
    const baseDelay = 1000; // 1초
    return Math.min(baseDelay * Math.pow(2, attempt - 1), 30000);
  }

  private sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}
```

### 서킷 브레이커 패턴 구현

```typescript
// 서킷 브레이커 서비스
interface CircuitBreakerState {
  status: 'CLOSED' | 'OPEN' | 'HALF_OPEN';
  failureCount: number;
  successCount: number;
  lastFailureTime: Date | null;
  nextAttemptTime: Date | null;
}

class CircuitBreakerService {
  private states: Map<string, CircuitBreakerState>;
  private config: CircuitBreakerConfig;

  constructor(config: CircuitBreakerConfig) {
    this.states = new Map();
    this.config = config;
  }

  // 요청 허용 여부 확인
  isAllowed(serviceId: string): boolean {
    const state = this.getOrCreateState(serviceId);

    switch (state.status) {
      case 'CLOSED':
        return true;

      case 'OPEN':
        // 재시도 시간 확인
        if (state.nextAttemptTime && new Date() >= state.nextAttemptTime) {
          // HALF_OPEN 상태로 전환
          state.status = 'HALF_OPEN';
          state.successCount = 0;
          return true;
        }
        return false;

      case 'HALF_OPEN':
        return true;
    }
  }

  // 성공 기록
  recordSuccess(serviceId: string): void {
    const state = this.getOrCreateState(serviceId);

    if (state.status === 'HALF_OPEN') {
      state.successCount++;

      if (state.successCount >= this.config.successThreshold) {
        // CLOSED 상태로 전환
        state.status = 'CLOSED';
        state.failureCount = 0;
        state.successCount = 0;
      }
    } else if (state.status === 'CLOSED') {
      state.failureCount = 0;
    }
  }

  // 실패 기록
  recordFailure(serviceId: string): void {
    const state = this.getOrCreateState(serviceId);

    state.failureCount++;
    state.lastFailureTime = new Date();

    if (state.status === 'HALF_OPEN') {
      // 즉시 OPEN으로 전환
      state.status = 'OPEN';
      state.nextAttemptTime = new Date(
        Date.now() + this.config.resetTimeout
      );
    } else if (state.failureCount >= this.config.failureThreshold) {
      // OPEN 상태로 전환
      state.status = 'OPEN';
      state.nextAttemptTime = new Date(
        Date.now() + this.config.resetTimeout
      );
    }
  }

  private getOrCreateState(serviceId: string): CircuitBreakerState {
    if (!this.states.has(serviceId)) {
      this.states.set(serviceId, {
        status: 'CLOSED',
        failureCount: 0,
        successCount: 0,
        lastFailureTime: null,
        nextAttemptTime: null,
      });
    }
    return this.states.get(serviceId)!;
  }
}
```

---

## 6.2 뱅킹 통합

### 은행 계좌 연결

```typescript
// Open Banking / Plaid를 통한 뱅킹 통합
interface BankingIntegration {
  provider: 'plaid' | 'yodlee' | 'mx' | 'direct';
  capabilities: BankingCapability[];
  accounts: LinkedAccount[];
  syncStatus: SyncStatus;
}

class BankingAdapter implements IntegrationAdapter {
  private plaidClient: PlaidApi;
  private credentials: Map<string, AccountCredentials>;

  constructor(config: BankingAdapterConfig) {
    this.plaidClient = new PlaidApi({
      basePath: config.environment === 'production'
        ? PlaidEnvironments.production
        : PlaidEnvironments.sandbox,
      baseOptions: {
        headers: {
          'PLAID-CLIENT-ID': config.clientId,
          'PLAID-SECRET': config.secret,
        },
      },
    });
    this.credentials = new Map();
  }

  // 새 은행 계좌 연결
  async linkAccount(request: LinkAccountRequest): Promise<LinkedAccount> {
    // 링크 토큰 생성
    const linkTokenResponse = await this.plaidClient.linkTokenCreate({
      user: { client_user_id: request.userId },
      client_name: 'Cryo-Asset 관리 시스템',
      products: [Products.Auth, Products.Transactions, Products.Balance],
      country_codes: [CountryCode.Us, CountryCode.Kr],
      language: request.language || 'ko',
      account_filters: {
        depository: {
          account_subtypes: [
            AccountSubtype.Checking,
            AccountSubtype.Savings,
            AccountSubtype.MoneyMarket,
            AccountSubtype.Cd,
          ],
        },
      },
    });

    // 프론트엔드가 연결을 완료할 수 있도록 링크 토큰 반환
    return {
      linkToken: linkTokenResponse.data.link_token,
      expiration: linkTokenResponse.data.expiration,
    };
  }

  // 사용자가 링크를 완료한 후 공개 토큰 교환
  async exchangePublicToken(
    publicToken: string,
    userId: string
  ): Promise<AccountConnection> {
    // 공개 토큰을 액세스 토큰으로 교환
    const exchangeResponse = await this.plaidClient.itemPublicTokenExchange({
      public_token: publicToken,
    });

    const accessToken = exchangeResponse.data.access_token;
    const itemId = exchangeResponse.data.item_id;

    // 계좌 상세 정보 가져오기
    const accountsResponse = await this.plaidClient.accountsGet({
      access_token: accessToken,
    });

    // 자격 증명 안전하게 저장
    await this.storeCredentials(userId, itemId, accessToken);

    // 연결된 계좌 생성
    const linkedAccounts: LinkedAccount[] = accountsResponse.data.accounts.map(
      account => ({
        id: generateAccountId(),
        externalId: account.account_id,
        itemId: itemId,
        name: account.name,
        officialName: account.official_name,
        type: this.mapAccountType(account.type),
        subtype: account.subtype,
        mask: account.mask,
        currentBalance: account.balances.current,
        availableBalance: account.balances.available,
        currency: account.balances.iso_currency_code || 'USD',
        institution: accountsResponse.data.item.institution_id,
        status: 'ACTIVE',
        lastSyncedAt: new Date(),
      })
    );

    // 계좌 매핑 저장
    await this.storeAccountMappings(userId, linkedAccounts);

    return {
      itemId,
      institutionId: accountsResponse.data.item.institution_id,
      accounts: linkedAccounts,
    };
  }

  // 계좌 잔액 동기화
  async syncBalances(accountIds: string[]): Promise<BalanceSync[]> {
    const syncs: BalanceSync[] = [];

    // 효율성을 위해 항목별로 그룹화
    const accountsByItem = await this.groupAccountsByItem(accountIds);

    for (const [itemId, accounts] of accountsByItem) {
      const accessToken = await this.getAccessToken(itemId);

      const response = await this.plaidClient.accountsBalanceGet({
        access_token: accessToken,
        options: {
          account_ids: accounts.map(a => a.externalId),
        },
      });

      for (const plaidAccount of response.data.accounts) {
        const account = accounts.find(a => a.externalId === plaidAccount.account_id);

        syncs.push({
          accountId: account.id,
          previousBalance: account.currentBalance,
          currentBalance: plaidAccount.balances.current,
          availableBalance: plaidAccount.balances.available,
          syncedAt: new Date(),
          status: 'SUCCESS',
        });

        // 저장된 잔액 업데이트
        await this.updateAccountBalance(account.id, {
          current: plaidAccount.balances.current,
          available: plaidAccount.balances.available,
        });
      }
    }

    return syncs;
  }

  // 거래 내역 조회
  async getTransactions(
    accountId: string,
    startDate: Date,
    endDate: Date
  ): Promise<BankTransaction[]> {
    const account = await this.getAccount(accountId);
    const accessToken = await this.getAccessToken(account.itemId);

    const transactions: BankTransaction[] = [];
    let hasMore = true;
    let cursor: string | undefined;

    while (hasMore) {
      const response = await this.plaidClient.transactionsSync({
        access_token: accessToken,
        cursor,
        options: {
          include_personal_finance_category: true,
        },
      });

      for (const txn of response.data.added) {
        if (txn.account_id === account.externalId) {
          const txnDate = new Date(txn.date);
          if (txnDate >= startDate && txnDate <= endDate) {
            transactions.push(this.mapTransaction(txn, account));
          }
        }
      }

      hasMore = response.data.has_more;
      cursor = response.data.next_cursor;
    }

    return transactions;
  }

  // 거래 매핑
  private mapTransaction(
    plaidTxn: PlaidTransaction,
    account: LinkedAccount
  ): BankTransaction {
    return {
      id: generateTransactionId(),
      accountId: account.id,
      externalId: plaidTxn.transaction_id,
      date: new Date(plaidTxn.date),
      amount: Math.abs(plaidTxn.amount),
      direction: plaidTxn.amount < 0 ? 'CREDIT' : 'DEBIT',
      description: plaidTxn.name,
      merchantName: plaidTxn.merchant_name,
      category: plaidTxn.personal_finance_category?.primary,
      pending: plaidTxn.pending,
      currency: plaidTxn.iso_currency_code || 'USD',
    };
  }

  // ACH 이체 시작
  async initiateTransfer(request: TransferRequest): Promise<TransferResult> {
    const account = await this.getAccount(request.sourceAccountId);
    const accessToken = await this.getAccessToken(account.itemId);

    // 이체 승인 생성
    const authResponse = await this.plaidClient.transferAuthorizationCreate({
      access_token: accessToken,
      account_id: account.externalId,
      type: TransferType.Debit,
      network: TransferNetwork.Ach,
      amount: request.amount.toString(),
      ach_class: ACHClass.Ppd,
      user: {
        legal_name: request.accountHolderName,
      },
    });

    if (authResponse.data.authorization.decision !== 'approved') {
      return {
        success: false,
        error: {
          code: 'TRANSFER_NOT_AUTHORIZED',
          message: authResponse.data.authorization.decision_rationale?.description,
        },
      };
    }

    // 이체 생성
    const transferResponse = await this.plaidClient.transferCreate({
      access_token: accessToken,
      account_id: account.externalId,
      authorization_id: authResponse.data.authorization.id,
      type: TransferType.Debit,
      network: TransferNetwork.Ach,
      amount: request.amount.toString(),
      description: request.description,
      ach_class: ACHClass.Ppd,
      user: {
        legal_name: request.accountHolderName,
      },
    });

    return {
      success: true,
      transferId: transferResponse.data.transfer.id,
      status: transferResponse.data.transfer.status,
      estimatedSettlement: this.calculateSettlementDate(),
    };
  }

  // 계좌 유형 매핑
  private mapAccountType(plaidType: string): string {
    const typeMap: Record<string, string> = {
      'depository': '예금',
      'investment': '투자',
      'loan': '대출',
      'credit': '신용',
      'other': '기타',
    };
    return typeMap[plaidType] || '기타';
  }
}
```

---

## 6.3 증권사 통합

### 투자 계좌 연결

```typescript
// 투자 계좌용 증권사 통합
class BrokerageAdapter implements IntegrationAdapter {
  private providers: Map<string, BrokerageProvider>;

  constructor(config: BrokerageAdapterConfig) {
    this.providers = new Map();
    this.initializeProviders(config.providers);
  }

  // 포트폴리오 포지션 동기화
  async syncPositions(accountId: string): Promise<PositionSync> {
    const account = await this.getAccount(accountId);
    const provider = this.providers.get(account.providerId);

    const positions = await provider.getPositions(account.externalId);

    const syncedPositions: Position[] = [];

    for (const position of positions) {
      // 현재 시장 가격 조회
      const quote = await this.getQuote(position.symbol);

      syncedPositions.push({
        id: generatePositionId(),
        accountId: accountId,
        symbol: position.symbol,
        securityType: position.securityType,
        quantity: position.quantity,
        costBasis: position.costBasis,
        currentPrice: quote.price,
        currentValue: position.quantity * quote.price,
        unrealizedGain: (position.quantity * quote.price) - position.costBasis,
        unrealizedGainPercent: ((quote.price - (position.costBasis / position.quantity)) /
          (position.costBasis / position.quantity)) * 100,
        lastUpdated: new Date(),
      });
    }

    // 계좌 합계 계산
    const totalValue = syncedPositions.reduce((sum, p) => sum + p.currentValue, 0);
    const totalCost = syncedPositions.reduce((sum, p) => sum + p.costBasis, 0);

    return {
      accountId,
      positions: syncedPositions,
      totalValue,
      totalCost,
      totalGain: totalValue - totalCost,
      syncedAt: new Date(),
      status: 'SUCCESS',
    };
  }

  // 실시간 시세 조회
  async getQuote(symbol: string): Promise<Quote> {
    // 신뢰성을 위해 여러 데이터 소스 시도
    const sources = ['IEX', 'POLYGON', 'ALPHA_VANTAGE'];

    for (const source of sources) {
      try {
        return await this.getQuoteFromSource(symbol, source);
      } catch (error) {
        console.warn(`${source}에서 시세 조회 실패: ${error.message}`);
      }
    }

    throw new Error(`${symbol}에 대한 시세를 가져올 수 없습니다`);
  }

  // 소스별 시세 조회
  private async getQuoteFromSource(symbol: string, source: string): Promise<Quote> {
    switch (source) {
      case 'IEX':
        const iexResponse = await fetch(
          `https://cloud.iexapis.com/stable/stock/${symbol}/quote?token=${process.env.IEX_TOKEN}`
        );
        const iexData = await iexResponse.json();
        return {
          symbol,
          price: iexData.latestPrice,
          change: iexData.change,
          changePercent: iexData.changePercent,
          volume: iexData.volume,
          timestamp: new Date(iexData.latestUpdate),
          source: 'IEX',
        };

      case 'POLYGON':
        const polygonResponse = await fetch(
          `https://api.polygon.io/v2/aggs/ticker/${symbol}/prev?apiKey=${process.env.POLYGON_KEY}`
        );
        const polygonData = await polygonResponse.json();
        return {
          symbol,
          price: polygonData.results[0].c,
          change: polygonData.results[0].c - polygonData.results[0].o,
          changePercent: ((polygonData.results[0].c - polygonData.results[0].o) /
            polygonData.results[0].o) * 100,
          volume: polygonData.results[0].v,
          timestamp: new Date(polygonData.results[0].t),
          source: 'POLYGON',
        };

      default:
        throw new Error(`알 수 없는 시세 소스: ${source}`);
    }
  }

  // 거래 실행
  async executeTrade(order: TradeOrder): Promise<TradeExecution> {
    const account = await this.getAccount(order.accountId);
    const provider = this.providers.get(account.providerId);

    // 주문 검증
    await this.validateOrder(order, account);

    // 주문 제출
    const orderResult = await provider.submitOrder({
      accountId: account.externalId,
      symbol: order.symbol,
      side: order.side,
      quantity: order.quantity,
      orderType: order.orderType,
      limitPrice: order.limitPrice,
      timeInForce: order.timeInForce || 'DAY',
    });

    // 체결 모니터링
    const execution = await this.monitorOrderExecution(
      provider,
      account.externalId,
      orderResult.orderId
    );

    return {
      orderId: orderResult.orderId,
      status: execution.status,
      filledQuantity: execution.filledQuantity,
      averagePrice: execution.averagePrice,
      commission: execution.commission,
      executedAt: execution.executedAt,
    };
  }

  // 표준화된 형식의 계좌 보유 내역 조회
  async getHoldings(accountId: string): Promise<StandardizedHolding[]> {
    const positions = await this.syncPositions(accountId);

    return positions.positions.map(position => ({
      securityId: this.getSecurityId(position.symbol),
      securityType: position.securityType,
      symbol: position.symbol,
      name: position.securityName,
      quantity: position.quantity,
      unitCost: position.costBasis / position.quantity,
      totalCost: position.costBasis,
      currentPrice: position.currentPrice,
      currentValue: position.currentValue,
      unrealizedGain: position.unrealizedGain,
      weightPercent: (position.currentValue / positions.totalValue) * 100,
      assetClass: this.classifyAsset(position.symbol, position.securityType),
    }));
  }

  // 주문 검증
  private async validateOrder(order: TradeOrder, account: Account): Promise<void> {
    // 매수 시 잔액 확인
    if (order.side === 'BUY') {
      const balance = await this.getAccountBalance(account.id);
      const estimatedCost = order.quantity * (order.limitPrice || await this.getQuote(order.symbol).then(q => q.price));

      if (balance.available < estimatedCost) {
        throw new Error('잔액 부족');
      }
    }

    // 매도 시 보유 수량 확인
    if (order.side === 'SELL') {
      const positions = await this.syncPositions(account.id);
      const position = positions.positions.find(p => p.symbol === order.symbol);

      if (!position || position.quantity < order.quantity) {
        throw new Error('보유 수량 부족');
      }
    }
  }
}
```

---

## 6.4 보험 통합

### 생명보험 정책 관리

```typescript
// 생명보험 정책용 보험 통합
class InsuranceAdapter implements IntegrationAdapter {
  private carriers: Map<string, InsuranceCarrier>;

  constructor(config: InsuranceAdapterConfig) {
    this.carriers = new Map();
    this.initializeCarriers(config.carriers);
  }

  // 냉동 보존 자금용 보험 등록
  async registerPolicy(registration: PolicyRegistration): Promise<RegisteredPolicy> {
    const carrier = this.carriers.get(registration.carrierId);

    // 보험이 유효한지 확인
    const policyDetails = await carrier.verifyPolicy(registration.policyNumber);

    if (policyDetails.status !== 'IN_FORCE') {
      throw new Error(`보험 ${registration.policyNumber}이(가) 유효하지 않습니다`);
    }

    // 사망 보험금 금액 확인
    if (policyDetails.deathBenefit < registration.requiredAmount) {
      throw new Error(
        `보험 사망 보험금 $${policyDetails.deathBenefit}이(가) 필요 금액 $${registration.requiredAmount}보다 적습니다`
      );
    }

    // 냉동 보존 기관을 수익자로 등록
    const beneficiaryResult = await carrier.updateBeneficiary({
      policyNumber: registration.policyNumber,
      beneficiaries: [
        {
          type: 'PRIMARY',
          name: registration.cryonicsOrganization.name,
          taxId: registration.cryonicsOrganization.taxId,
          percentage: registration.beneficiaryPercentage,
          relationship: 'OTHER',
          designation: 'CRYONICS_PRESERVATION',
        },
        // 나머지 수익자
        ...registration.remainingBeneficiaries.map(b => ({
          type: b.primary ? 'PRIMARY' : 'CONTINGENT',
          name: b.name,
          percentage: b.percentage,
          relationship: b.relationship,
        })),
      ],
    });

    // 필요시 담보 양도 설정
    let assignmentResult = null;
    if (registration.collateralAssignment) {
      assignmentResult = await carrier.createCollateralAssignment({
        policyNumber: registration.policyNumber,
        assignee: registration.cryonicsOrganization.name,
        amount: registration.collateralAmount,
        purpose: '냉동 보존 자금',
      });
    }

    // 등록된 보험 기록 생성
    const registeredPolicy: RegisteredPolicy = {
      id: generatePolicyId(),
      policyNumber: registration.policyNumber,
      carrierId: registration.carrierId,
      carrierName: carrier.name,
      policyType: policyDetails.type,
      status: 'REGISTERED',

      insured: {
        patientId: registration.patientId,
        name: policyDetails.insuredName,
      },

      coverage: {
        deathBenefit: policyDetails.deathBenefit,
        allocatedToCryonics: registration.beneficiaryPercentage,
        cryonicsBenefit: policyDetails.deathBenefit * (registration.beneficiaryPercentage / 100),
      },

      beneficiaryDesignation: beneficiaryResult,
      collateralAssignment: assignmentResult,

      premium: {
        amount: policyDetails.premiumAmount,
        frequency: policyDetails.premiumFrequency,
        nextDueDate: policyDetails.nextPremiumDate,
        paymentMethod: policyDetails.paymentMethod,
      },

      registeredAt: new Date(),
      lastVerifiedAt: new Date(),
    };

    // 등록 저장
    await this.storePolicyRegistration(registeredPolicy);

    // 모니터링 설정
    await this.setupPolicyMonitoring(registeredPolicy);

    return registeredPolicy;
  }

  // 보험 상태 모니터링
  async monitorPolicies(): Promise<PolicyMonitoringResult[]> {
    const registeredPolicies = await this.getRegisteredPolicies();
    const results: PolicyMonitoringResult[] = [];

    for (const policy of registeredPolicies) {
      try {
        const carrier = this.carriers.get(policy.carrierId);
        const currentStatus = await carrier.getPolicyStatus(policy.policyNumber);

        const result: PolicyMonitoringResult = {
          policyId: policy.id,
          policyNumber: policy.policyNumber,
          checkDate: new Date(),
          status: 'OK',
          alerts: [],
        };

        // 보험 상태 확인
        if (currentStatus.status !== 'IN_FORCE') {
          result.status = 'ALERT';
          result.alerts.push({
            type: 'POLICY_NOT_IN_FORCE',
            severity: 'CRITICAL',
            message: `보험 상태가 ${currentStatus.status}(으)로 변경되었습니다`,
          });
        }

        // 보험료 상태 확인
        if (currentStatus.premiumPastDue) {
          result.status = 'ALERT';
          result.alerts.push({
            type: 'PREMIUM_PAST_DUE',
            severity: 'HIGH',
            message: `${currentStatus.premiumDueDate} 이후 보험료 미납`,
            daysOverdue: currentStatus.daysOverdue,
          });
        }

        // 수익자 상태 확인
        const beneficiaryValid = await this.verifyBeneficiaryStatus(policy, currentStatus);
        if (!beneficiaryValid.valid) {
          result.status = 'ALERT';
          result.alerts.push({
            type: 'BENEFICIARY_ISSUE',
            severity: 'HIGH',
            message: beneficiaryValid.issue,
          });
        }

        // 사망 보험금 금액 확인
        if (currentStatus.deathBenefit < policy.coverage.deathBenefit) {
          result.status = 'WARNING';
          result.alerts.push({
            type: 'DEATH_BENEFIT_REDUCED',
            severity: 'MEDIUM',
            message: `사망 보험금이 $${policy.coverage.deathBenefit}에서 $${currentStatus.deathBenefit}(으)로 감소했습니다`,
          });
        }

        results.push(result);

        // 필요시 알림 전송
        if (result.alerts.length > 0) {
          await this.sendPolicyAlerts(policy, result.alerts);
        }

      } catch (error) {
        results.push({
          policyId: policy.id,
          policyNumber: policy.policyNumber,
          checkDate: new Date(),
          status: 'ERROR',
          error: error.message,
        });
      }
    }

    return results;
  }

  // 사망 청구 처리
  async processDeathClaim(claim: DeathClaimRequest): Promise<ClaimResult> {
    const policy = await this.getPolicy(claim.policyId);
    const carrier = this.carriers.get(policy.carrierId);

    // 사망 청구 제출
    const claimSubmission = await carrier.submitDeathClaim({
      policyNumber: policy.policyNumber,
      dateOfDeath: claim.dateOfDeath,
      causeOfDeath: claim.causeOfDeath,
      claimant: {
        name: claim.cryonicsOrganization.name,
        taxId: claim.cryonicsOrganization.taxId,
        relationship: 'BENEFICIARY',
      },
      documentation: claim.documentation,
    });

    // 청구 상태 추적
    await this.trackClaimStatus(claimSubmission.claimId, policy.id);

    return {
      claimId: claimSubmission.claimId,
      status: claimSubmission.status,
      expectedProcessingTime: claimSubmission.estimatedDays,
      requiredDocuments: claimSubmission.additionalDocumentsNeeded,
      contactInfo: claimSubmission.claimsContact,
    };
  }
}
```

---

## 6.5 법률 서비스 통합

### 신탁 및 유산 서비스

```typescript
// 법률 서비스 통합
class LegalServicesAdapter implements IntegrationAdapter {
  private documentService: DocumentGenerationService;
  private eSignatureService: ESignatureService;
  private courtFilingService: CourtFilingService;

  constructor(config: LegalServicesConfig) {
    this.documentService = new DocumentGenerationService(config.documents);
    this.eSignatureService = new ESignatureService(config.esignature);
    this.courtFilingService = new CourtFilingService(config.courtFiling);
  }

  // 신탁 문서 생성
  async generateTrustDocument(
    trust: CryonicsTrust,
    options: DocumentOptions
  ): Promise<GeneratedDocument> {
    // 적절한 템플릿 선택
    const template = await this.selectTrustTemplate(
      trust.trustType,
      trust.jurisdiction
    );

    // 신탁 데이터로 템플릿 채우기
    const documentData = this.prepareTrustDocumentData(trust);

    // 문서 생성
    const document = await this.documentService.generate({
      templateId: template.id,
      data: documentData,
      format: options.format || 'PDF',
      options: {
        includeExhibits: options.includeExhibits,
        includeSchedules: options.includeSchedules,
        watermark: options.draft ? '초안' : null,
      },
    });

    // 문서 저장
    const stored = await this.storeDocument(document, {
      entityType: 'TRUST',
      entityId: trust.id,
      documentType: 'TRUST_INSTRUMENT',
      version: trust.version,
    });

    return {
      documentId: stored.id,
      filename: stored.filename,
      format: document.format,
      pageCount: document.pageCount,
      generatedAt: new Date(),
      downloadUrl: stored.downloadUrl,
      expiresAt: stored.urlExpiration,
    };
  }

  // 문서 서명 시작
  async initiateDocumentSigning(
    documentId: string,
    signers: SignerInfo[]
  ): Promise<SigningSession> {
    const document = await this.getDocument(documentId);

    // 서명 봉투 생성
    const envelope = await this.eSignatureService.createEnvelope({
      documents: [{
        documentId: document.id,
        name: document.filename,
        fileContent: document.content,
      }],
      recipients: signers.map((signer, index) => ({
        recipientId: signer.id,
        name: signer.name,
        email: signer.email,
        role: signer.role,
        routingOrder: signer.order || index + 1,
        tabs: this.createSignerTabs(signer),
      })),
      emailSubject: `서명 필요: ${document.title}`,
      emailBody: this.generateSigningEmail(document),
      settings: {
        expiresInDays: 30,
        reminderEnabled: true,
        reminderDelayDays: 3,
        reminderFrequencyDays: 3,
      },
    });

    // 서명을 위해 전송
    const sendResult = await this.eSignatureService.sendEnvelope(envelope.envelopeId);

    return {
      sessionId: envelope.envelopeId,
      status: sendResult.status,
      signers: signers.map(signer => ({
        id: signer.id,
        name: signer.name,
        email: signer.email,
        status: 'PENDING',
        signingUrl: sendResult.signerUrls?.[signer.id],
      })),
      expiresAt: sendResult.expiresAt,
    };
  }

  // 법원에 신탁 제출 (필요시)
  async fileWithCourt(
    trustId: string,
    filing: CourtFilingRequest
  ): Promise<FilingResult> {
    const trust = await this.getTrust(trustId);
    const documents = await this.getTrustDocuments(trustId);

    // 제출 패키지 준비
    const filingPackage = await this.courtFilingService.preparePackage({
      caseType: filing.caseType,
      jurisdiction: {
        state: trust.jurisdiction.state,
        county: filing.county,
        court: filing.courtName,
      },
      documents: documents.map(doc => ({
        documentId: doc.id,
        documentType: this.mapToCourtDocType(doc.type),
        filename: doc.filename,
      })),
      parties: this.mapPartiesToFiling(trust.parties),
      filingFee: await this.calculateFilingFee(filing),
    });

    // 제출
    const submission = await this.courtFilingService.submit(filingPackage);

    return {
      filingId: submission.filingId,
      confirmationNumber: submission.confirmationNumber,
      status: submission.status,
      filedAt: submission.filedAt,
      caseNumber: submission.caseNumber,
      estimatedProcessingDays: submission.estimatedDays,
    };
  }

  // 수정 문서 생성
  async generateAmendment(
    trustId: string,
    amendment: TrustAmendment
  ): Promise<GeneratedDocument> {
    const trust = await this.getTrust(trustId);
    const originalDocument = await this.getOriginalTrustDocument(trustId);

    // 수정 유형 및 템플릿 결정
    const template = await this.selectAmendmentTemplate(
      amendment.type,
      trust.jurisdiction
    );

    // 수정 데이터 준비
    const amendmentData = {
      trust: {
        name: trust.name,
        originalDate: trust.createdAt,
        trustType: trust.trustType,
      },
      amendment: {
        number: await this.getNextAmendmentNumber(trustId),
        date: new Date(),
        type: amendment.type,
        changes: amendment.changes,
        reason: amendment.reason,
      },
      grantor: trust.parties.grantor,
      originalDocumentReference: originalDocument.reference,
    };

    // 수정안 생성
    const document = await this.documentService.generate({
      templateId: template.id,
      data: amendmentData,
      format: 'PDF',
    });

    return await this.storeDocument(document, {
      entityType: 'TRUST',
      entityId: trustId,
      documentType: 'AMENDMENT',
      parentDocumentId: originalDocument.id,
    });
  }

  // 신탁 문서 데이터 준비
  private prepareTrustDocumentData(trust: CryonicsTrust): TrustDocumentData {
    return {
      trustName: trust.name,
      trustType: this.translateTrustType(trust.trustType),
      jurisdiction: trust.jurisdiction,
      creationDate: trust.createdAt,

      parties: {
        grantor: {
          ...trust.parties.grantor,
          role: '위탁자',
        },
        trustees: trust.parties.trustees.map(t => ({
          ...t,
          role: '수탁자',
        })),
        beneficiaries: trust.parties.beneficiaries.map(b => ({
          ...b,
          role: '수익자',
        })),
      },

      terms: trust.terms,
      revivalProvisions: trust.revivalProvisions,
      governingLaw: trust.jurisdiction.governingLaw,
    };
  }

  // 신탁 유형 번역
  private translateTrustType(type: string): string {
    const translations: Record<string, string> = {
      'PERSONAL_REVIVAL_TRUST': '개인 소생 신탁',
      'PATIENT_CARE_FUND': '환자 관리 기금',
      'CHARITABLE_REMAINDER': '자선 잔여 신탁',
      'DYNASTY_TRUST': '왕조 신탁',
    };
    return translations[type] || type;
  }
}
```

---

## 6.6 블록체인 통합

### 분산 원장 연결

```typescript
// 자산 등록 및 출처 추적을 위한 블록체인 통합
class BlockchainAdapter implements IntegrationAdapter {
  private providers: Map<string, BlockchainProvider>;
  private defaultNetwork: string;

  constructor(config: BlockchainConfig) {
    this.providers = new Map();
    this.defaultNetwork = config.defaultNetwork;

    // 프로바이더 초기화
    for (const network of config.networks) {
      this.providers.set(network.name, this.createProvider(network));
    }
  }

  // 프로바이더 생성
  private createProvider(network: NetworkConfig): BlockchainProvider {
    switch (network.type) {
      case 'ETHEREUM':
        return new EthereumProvider(network);
      case 'POLYGON':
        return new PolygonProvider(network);
      case 'ARBITRUM':
        return new ArbitrumProvider(network);
      default:
        throw new Error(`지원되지 않는 네트워크: ${network.type}`);
    }
  }

  // 블록체인에 자산 등록
  async registerAsset(
    asset: CryoAsset,
    options: RegistrationOptions = {}
  ): Promise<BlockchainRegistration> {
    const network = options.network || this.defaultNetwork;
    const provider = this.providers.get(network);

    // 자산 해시 생성
    const assetHash = this.createAssetHash(asset);

    // 등록 데이터 준비
    const registrationData = {
      assetHash,
      category: asset.category,
      assetType: this.encodeAssetType(asset.assetType),
      valuationHash: this.hashValuation(asset.currentValuation),
      ipfsHash: await this.storeToIPFS(this.prepareIPFSData(asset)),
      encryptionKeyId: asset.encryptionKeyId,
      organizationId: this.encodeOrganizationId(asset.organizationId),
    };

    // 트랜잭션 제출
    const tx = await provider.executeContract(
      'CryoAssetRegistry',
      'registerAsset',
      [
        registrationData.assetHash,
        registrationData.category,
        registrationData.assetType,
        registrationData.valuationHash,
        registrationData.ipfsHash,
        registrationData.encryptionKeyId,
        registrationData.organizationId,
      ]
    );

    // 확인 대기
    const receipt = await provider.waitForConfirmation(tx.hash, 2);

    return {
      transactionHash: receipt.transactionHash,
      blockNumber: receipt.blockNumber,
      blockHash: receipt.blockHash,
      timestamp: new Date(receipt.timestamp * 1000),
      network,
      contractAddress: provider.getContractAddress('CryoAssetRegistry'),
      assetHash,
      explorerUrl: this.getExplorerUrl(network, receipt.transactionHash),
    };
  }

  // 체인에서 자산 평가 업데이트
  async updateValuation(
    assetId: string,
    valuation: Valuation
  ): Promise<BlockchainUpdate> {
    const asset = await this.getAsset(assetId);
    const network = asset.blockchainRef?.network || this.defaultNetwork;
    const provider = this.providers.get(network);

    const valuationHash = this.hashValuation(valuation);

    const tx = await provider.executeContract(
      'CryoAssetRegistry',
      'updateValuation',
      [
        asset.blockchainRef.assetHash,
        valuationHash,
      ]
    );

    const receipt = await provider.waitForConfirmation(tx.hash);

    return {
      transactionHash: receipt.transactionHash,
      blockNumber: receipt.blockNumber,
      valuationHash,
      previousValuationHash: asset.blockchainRef.valuationHash,
      explorerUrl: this.getExplorerUrl(network, receipt.transactionHash),
    };
  }

  // 자산 등록 확인
  async verifyAsset(assetHash: string, network?: string): Promise<VerificationResult> {
    const targetNetwork = network || this.defaultNetwork;
    const provider = this.providers.get(targetNetwork);

    const record = await provider.callContract(
      'CryoAssetRegistry',
      'getAssetRecord',
      [assetHash]
    );

    if (!record || record.registrationTimestamp === 0) {
      return { verified: false, reason: '블록체인에서 자산을 찾을 수 없습니다' };
    }

    // 평가 내역 가져오기
    const history = await provider.callContract(
      'CryoAssetRegistry',
      'getValuationHistory',
      [assetHash]
    );

    return {
      verified: true,
      record: {
        assetHash: record.assetHash,
        registrationTimestamp: new Date(record.registrationTimestamp * 1000),
        registrar: record.registrar,
        organizationId: this.decodeOrganizationId(record.organizationId),
        category: record.assetCategory,
        currentValuationHash: record.currentValuationHash,
        lastValuationTimestamp: new Date(record.lastValuationTimestamp * 1000),
        valuationCount: history.length,
        isActive: record.isActive,
      },
      network: targetNetwork,
    };
  }

  // IPFS에 데이터 저장
  private async storeToIPFS(data: any): Promise<string> {
    const ipfsClient = create({ url: process.env.IPFS_API_URL });

    const content = JSON.stringify(data);
    const encrypted = await this.encryptForIPFS(content);

    const result = await ipfsClient.add(encrypted);
    return result.cid.toString();
  }

  // 자산 해시 생성
  private createAssetHash(asset: CryoAsset): string {
    const hashInput = {
      id: asset.id,
      portfolioId: asset.portfolioId,
      category: asset.category,
      name: asset.name,
      ownership: asset.ownership,
      createdAt: asset.metadata.createdAt.toISOString(),
    };

    return ethers.keccak256(
      ethers.toUtf8Bytes(JSON.stringify(hashInput))
    );
  }

  // 온체인 저장용 평가 해시
  private hashValuation(valuation: Valuation): string {
    const hashInput = {
      value: valuation.value.toString(),
      currency: valuation.currency,
      date: valuation.valuationDate.toISOString(),
      method: valuation.valuationMethod,
    };

    return ethers.keccak256(
      ethers.toUtf8Bytes(JSON.stringify(hashInput))
    );
  }

  // 탐색기 URL 생성
  private getExplorerUrl(network: string, txHash: string): string {
    const explorers: Record<string, string> = {
      'ETHEREUM': `https://etherscan.io/tx/${txHash}`,
      'POLYGON': `https://polygonscan.com/tx/${txHash}`,
      'ARBITRUM': `https://arbiscan.io/tx/${txHash}`,
    };
    return explorers[network] || '';
  }
}
```

---

## 6.7 냉동 보존 기관 통합

### 기관 직접 연결

```typescript
// 냉동 보존 기관(Alcor, CI 등)과의 통합
class CryonicsOrgAdapter implements IntegrationAdapter {
  private organizations: Map<string, OrgConnection>;

  constructor(config: CryonicsOrgConfig) {
    this.organizations = new Map();
    this.initializeConnections(config.organizations);
  }

  // 환자 회원 자격 확인
  async verifyMembership(
    organizationId: string,
    patientId: string
  ): Promise<MembershipVerification> {
    const org = this.organizations.get(organizationId);

    const membership = await org.api.getMembershipStatus(patientId);

    return {
      verified: membership.active,
      membershipId: membership.id,
      membershipType: membership.type,
      status: membership.status,
      joinDate: membership.joinDate,
      standbyStatus: membership.standbyReady,
      preservationType: membership.preservationType,
      fundingStatus: {
        patientCareFund: membership.patientCareFundStatus,
        preservationFunding: membership.preservationFundingStatus,
      },
      lastVerified: new Date(),
    };
  }

  // 환자 관리 기금 상태 동기화
  async syncPatientCareFund(
    organizationId: string,
    patientId: string
  ): Promise<PatientCareFundStatus> {
    const org = this.organizations.get(organizationId);

    const fundStatus = await org.api.getPatientCareFund(patientId);

    return {
      patientId,
      organizationId,
      fundBalance: fundStatus.balance,
      targetBalance: fundStatus.target,
      fundingRatio: (fundStatus.balance / fundStatus.target) * 100,
      status: this.determineFundingStatus(fundStatus.balance, fundStatus.target),
      investmentAllocation: fundStatus.allocation,
      lastContribution: fundStatus.lastContribution,
      projectedYears: this.calculateProjectedYears(fundStatus),
      annualExpenses: fundStatus.annualExpenses,
      syncedAt: new Date(),
    };
  }

  // 보존 이벤트 보고
  async reportPreservation(
    organizationId: string,
    event: PreservationEvent
  ): Promise<PreservationReport> {
    const org = this.organizations.get(organizationId);

    // 보존 보고서 제출
    const report = await org.api.submitPreservationReport({
      patientId: event.patientId,
      preservationDate: event.preservationDate,
      preservationType: event.preservationType,
      procedureDetails: event.procedureDetails,
      team: event.team,
      location: event.location,
      notes: event.notes,
    });

    // 환자 상태 업데이트
    await this.updatePatientStatus(event.patientId, 'PRESERVED');

    // 자금 이벤트 트리거
    await this.triggerPreservationFunding(event);

    return {
      reportId: report.id,
      status: report.status,
      patientStatus: 'PRESERVED',
      fundingTriggered: true,
      confirmationNumber: report.confirmationNumber,
    };
  }

  // 대기 네트워크 상태 조회
  async getStandbyStatus(
    organizationId: string,
    patientId: string
  ): Promise<StandbyStatus> {
    const org = this.organizations.get(organizationId);

    const standby = await org.api.getStandbyInfo(patientId);

    return {
      patientId,
      standbyReady: standby.ready,
      standbyTeam: standby.assignedTeam,
      responseTime: standby.estimatedResponseTime,
      coverageArea: standby.coverageArea,
      emergencyContacts: standby.emergencyContacts,
      medicalDirective: standby.medicalDirectiveOnFile,
      lastDrillDate: standby.lastDrillDate,
      equipmentChecked: standby.equipmentStatus,
    };
  }

  // 회원 및 자금 데이터 전체 동기화
  async fullSync(
    organizationId: string,
    patientId: string
  ): Promise<FullSyncResult> {
    const [membership, patientCareFund, standby] = await Promise.all([
      this.verifyMembership(organizationId, patientId),
      this.syncPatientCareFund(organizationId, patientId),
      this.getStandbyStatus(organizationId, patientId),
    ]);

    return {
      patientId,
      organizationId,
      syncedAt: new Date(),
      membership,
      patientCareFund,
      standby,
      overallStatus: this.determineOverallStatus(membership, patientCareFund, standby),
      alerts: this.generateAlerts(membership, patientCareFund, standby),
    };
  }

  // 자금 상태 결정
  private determineFundingStatus(balance: number, target: number): string {
    const ratio = balance / target;
    if (ratio >= 1.0) return 'FULLY_FUNDED';
    if (ratio >= 0.75) return 'ADEQUATELY_FUNDED';
    if (ratio >= 0.5) return 'PARTIALLY_FUNDED';
    return 'UNDERFUNDED';
  }

  // 예상 연수 계산
  private calculateProjectedYears(fundStatus: FundStatus): number {
    if (fundStatus.annualExpenses === 0) return Infinity;
    return Math.floor(fundStatus.balance / fundStatus.annualExpenses);
  }

  // 전체 상태 결정
  private determineOverallStatus(
    membership: MembershipVerification,
    fund: PatientCareFundStatus,
    standby: StandbyStatus
  ): string {
    if (!membership.verified) return 'MEMBERSHIP_INVALID';
    if (fund.status === 'UNDERFUNDED') return 'UNDERFUNDED';
    if (!standby.standbyReady) return 'STANDBY_NOT_READY';
    return 'READY';
  }

  // 알림 생성
  private generateAlerts(
    membership: MembershipVerification,
    fund: PatientCareFundStatus,
    standby: StandbyStatus
  ): Alert[] {
    const alerts: Alert[] = [];

    if (!membership.verified) {
      alerts.push({
        type: 'MEMBERSHIP_INVALID',
        severity: 'CRITICAL',
        message: '회원 자격이 유효하지 않습니다',
      });
    }

    if (fund.fundingRatio < 75) {
      alerts.push({
        type: 'FUNDING_LOW',
        severity: fund.fundingRatio < 50 ? 'HIGH' : 'MEDIUM',
        message: `자금 비율이 ${fund.fundingRatio.toFixed(1)}%입니다`,
      });
    }

    if (!standby.standbyReady) {
      alerts.push({
        type: 'STANDBY_NOT_READY',
        severity: 'HIGH',
        message: '대기 상태가 준비되지 않았습니다',
      });
    }

    return alerts;
  }
}
```

---

## 장 요약

이 장에서는 냉동 보존 자산 관리 시스템을 외부 서비스와 연결하는 포괄적인 통합 패턴을 다루었습니다:

1. **통합 아키텍처**: 서킷 브레이커 및 재시도 로직을 갖춘 복원력 있는 패턴
2. **뱅킹 통합**: Open Banking을 통한 계좌 연결, 잔액 동기화, 거래 조회
3. **증권사 통합**: 포트폴리오 포지션 동기화 및 거래 실행
4. **보험 통합**: 생명보험 정책 관리 및 청구 처리
5. **법률 서비스**: 문서 생성, 전자 서명, 법원 제출
6. **블록체인**: 자산 등록 및 출처 추적
7. **냉동 보존 기관**: 보존 기관과의 직접 통합

이러한 통합은 자산 관리 플랫폼과 광범위한 금융 및 법률 생태계 간의 원활한 데이터 흐름을 가능하게 합니다.

---

*다음 장: 보안 - 보호 메커니즘 및 규정 준수 프레임워크*
