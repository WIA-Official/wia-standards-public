# 제4장: CBDC API 명세 및 통합 패턴

## 중앙은행 디지털 화폐 시스템을 위한 종합 API 아키텍처

### 4.1 API 아키텍처 개요

WIA-CBDC API 계층은 토큰 발행부터 국경간 거래까지 모든 CBDC 운영을 위한 표준화된 인터페이스를 제공합니다. 이 아키텍처는 복잡한 쿼리를 위한 GraphQL 지원과 함께 RESTful 원칙을 따릅니다.

```typescript
// API 아키텍처 정의
interface WIACBDCAPIArchitecture {
  version: '1.0.0';
  baseUrl: 'https://api.cbdc.{jurisdiction}.gov/v1';

  layers: {
    public: {
      description: '지갑 앱 및 가맹점을 위한 공개 API';
      authentication: 'OAuth 2.0 / API 키';
      rateLimit: '분당 1,000 요청';
    };
    partner: {
      description: '인가된 금융기관을 위한 API';
      authentication: 'mTLS + JWT';
      rateLimit: '분당 10,000 요청';
    };
    interbank: {
      description: '은행간 결제를 위한 API';
      authentication: 'mTLS + HSM 서명';
      rateLimit: '쿼터와 함께 무제한';
    };
    centralBank: {
      description: '내부 중앙은행 운영';
      authentication: '다자간 인가';
      rateLimit: '정책 통제';
    };
  };

  protocols: {
    rest: '기본 API 인터페이스';
    graphql: '복잡한 쿼리 및 구독';
    websocket: '실시간 알림';
    grpc: '고성능 은행간';
  };

  standards: {
    messaging: 'ISO 20022';
    security: 'OAuth 2.0, FAPI 2.0';
    dataFormats: 'JSON, Protocol Buffers';
  };
}
```

### 4.2 핵심 API 엔드포인트

#### 4.2.1 토큰 관리 API

```typescript
// 토큰 관리 API 명세
interface TokenManagementAPI {
  // 토큰 발행 (중앙은행 전용)
  issuance: {
    endpoint: 'POST /v1/tokens/issue';
    authorization: 'multi-party-auth';

    request: {
      issuanceRequest: {
        requestId: string;           // 멱등성 키
        amount: MonetaryAmount;
        denominationProfile: DenominationProfile;
        recipientAccount: string;
        conditions?: TokenCondition[];
        metadata?: IssuanceMetadata;
      };
      authorization: {
        signatures: AuthorizationSignature[];
        policyReference: string;
      };
    };

    response: {
      issuanceId: string;
      status: IssuanceStatus;
      tokensCreated: number;
      totalAmount: MonetaryAmount;
      tokenIds: string[];
      timestamp: ISO8601DateTime;
    };
  };

  // 토큰 전송
  transfer: {
    endpoint: 'POST /v1/tokens/transfer';
    authorization: 'owner-signature';

    request: {
      transferRequest: {
        requestId: string;
        fromWallet: string;
        toWallet: string;
        amount: MonetaryAmount;
        tokenIds?: string[];         // 특정 토큰 또는 자동 선택
        memo?: string;
        conditions?: TransferCondition[];
      };
      signature: OwnerSignature;
    };

    response: {
      transactionId: string;
      status: TransactionStatus;
      transferredAmount: MonetaryAmount;
      fees?: TransactionFee[];
      timestamp: ISO8601DateTime;
      receipt: TransactionReceipt;
    };
  };

  // 잔액 조회
  balance: {
    endpoint: 'GET /v1/wallets/{walletId}/balance';
    authorization: 'owner';

    response: {
      walletId: string;
      balances: {
        available: MonetaryAmount;   // 사용 가능
        pending: MonetaryAmount;     // 대기 중
        locked: MonetaryAmount;      // 잠김
        total: MonetaryAmount;       // 총계
      };
      tokenCount: number;
      lastUpdated: ISO8601DateTime;
    };
  };
}

// 토큰 API 구현
class TokenAPIController {
  constructor(
    private tokenService: TokenService,
    private authService: AuthorizationService,
    private complianceService: ComplianceService
  ) {}

  @Post('/tokens/transfer')
  @UseGuards(OwnerAuthGuard)
  async transfer(
    @Body() request: TransferRequest,
    @CurrentUser() user: AuthenticatedUser
  ): Promise<TransferResponse> {
    // 요청 검증
    const validation = await this.validateTransferRequest(request);
    if (!validation.valid) {
      throw new BadRequestException(validation.errors);
    }

    // 소유권 확인
    const ownershipVerified = await this.tokenService.verifyOwnership(
      request.fromWallet,
      user.walletId
    );
    if (!ownershipVerified) {
      throw new ForbiddenException('지갑 소유권 확인 실패');
    }

    // 잔액 확인
    const balance = await this.tokenService.getBalance(request.fromWallet);
    if (balance.available.valueInSmallestUnit < request.amount.valueInSmallestUnit) {
      throw new BadRequestException('잔액 부족');
    }

    // 컴플라이언스 심사
    const screeningResult = await this.complianceService.screenTransfer({
      sender: request.fromWallet,
      receiver: request.toWallet,
      amount: request.amount
    });

    if (screeningResult.blocked) {
      throw new ForbiddenException('컴플라이언스에 의해 거래 차단');
    }

    // 전송 실행
    const result = await this.tokenService.executeTransfer({
      ...request,
      complianceResult: screeningResult
    });

    return {
      transactionId: result.transactionId,
      status: result.status,
      transferredAmount: result.amount,
      fees: result.fees,
      timestamp: result.timestamp,
      receipt: this.generateReceipt(result)
    };
  }
}
```

#### 4.2.2 계좌 및 지갑 API

```typescript
// 계좌 관리 API
interface AccountManagementAPI {
  // 계좌 생성
  createAccount: {
    endpoint: 'POST /v1/accounts';
    authorization: 'partner-api';

    request: {
      accountRequest: {
        accountType: AccountType;
        holderInfo: {
          participantType: ParticipantType;
          identityRef: string;       // KYC 신원 참조
          name?: string;
          dateOfBirth?: ISO8601Date;
          nationality?: string;
        };
        initialLimits?: AccountLimits;
        settings?: AccountSettings;
      };
      partnerInfo: {
        partnerId: string;
        partnerAccountRef: string;
      };
    };

    response: {
      accountId: string;
      accountNumber: string;
      status: AccountState;
      createdAt: ISO8601DateTime;
      limits: AccountLimits;
    };
  };

  // 계좌 조회
  getAccount: {
    endpoint: 'GET /v1/accounts/{accountId}';
    authorization: 'owner-or-partner';

    response: {
      account: CBDCAccount;
      wallets: WalletSummary[];
    };
  };

  // 한도 업데이트
  updateLimits: {
    endpoint: 'PATCH /v1/accounts/{accountId}/limits';
    authorization: 'partner-api-elevated';

    request: {
      newLimits: Partial<AccountLimits>;
      kycUpgradeRef?: string;
      reason: string;
    };

    response: {
      accountId: string;
      previousLimits: AccountLimits;
      newLimits: AccountLimits;
      effectiveFrom: ISO8601DateTime;
    };
  };
}

// 지갑 관리 API
interface WalletManagementAPI {
  // 지갑 생성
  createWallet: {
    endpoint: 'POST /v1/wallets';
    authorization: 'account-owner';

    request: {
      walletRequest: {
        accountId: string;
        walletType: WalletType;
        walletName?: string;
        deviceBinding?: DeviceBindingRequest;
        securitySettings: WalletSecuritySettings;
      };
    };

    response: {
      walletId: string;
      walletAddress: string;
      walletType: WalletType;
      publicKey: string;
      createdAt: ISO8601DateTime;
    };
  };

  // 오프라인 토큰 생성
  generateOfflineTokens: {
    endpoint: 'POST /v1/wallets/{walletId}/offline-tokens';
    authorization: 'owner';

    request: {
      amount: MonetaryAmount;
      duration: number;            // 시간
      maxTransactions?: number;
    };

    response: {
      offlineTokens: OfflineToken[];
      validUntil: ISO8601DateTime;
      maxOfflineBalance: MonetaryAmount;
    };
  };
}

// 계좌 API 구현
class AccountAPIController {
  constructor(
    private accountService: AccountService,
    private kycService: KYCService,
    private limitService: LimitService
  ) {}

  @Post('/accounts')
  @UseGuards(PartnerAuthGuard)
  async createAccount(
    @Body() request: CreateAccountRequest,
    @Partner() partner: Partner
  ): Promise<CreateAccountResponse> {
    // KYC 신원 확인
    const kycResult = await this.kycService.verifyIdentity(
      request.holderInfo.identityRef,
      partner.partnerId
    );

    if (!kycResult.verified) {
      throw new BadRequestException('신원 확인 실패');
    }

    // KYC 수준에 따른 한도 결정
    const limits = await this.limitService.determineAccountLimits(
      request.accountType,
      kycResult.kycLevel
    );

    // 계좌 생성
    const account = await this.accountService.createAccount({
      type: request.accountType,
      holder: {
        participantId: kycResult.participantId,
        participantType: request.holderInfo.participantType,
        identityVerificationLevel: kycResult.kycLevel
      },
      limits: request.initialLimits || limits,
      settings: request.settings || this.getDefaultSettings(),
      partnerRef: {
        partnerId: partner.partnerId,
        externalRef: request.partnerInfo.partnerAccountRef
      }
    });

    return {
      accountId: account.accountId,
      accountNumber: account.accountNumber,
      status: account.status.state,
      createdAt: account.status.createdAt,
      limits: account.limits
    };
  }
}
```

#### 4.2.3 거래 API

```typescript
// 거래 API 명세
interface TransactionAPI {
  // 결제 시작
  initiatePayment: {
    endpoint: 'POST /v1/payments';
    authorization: 'payer';

    request: {
      paymentRequest: {
        requestId: string;
        payerWallet: string;
        payeeIdentifier: PayeeIdentifier;
        amount: MonetaryAmount;
        paymentType: PaymentType;
        scheduledTime?: ISO8601DateTime;
        expiresAt?: ISO8601DateTime;
        metadata?: PaymentMetadata;
      };
    };

    response: {
      paymentId: string;
      status: TransactionStatus;
      amount: MonetaryAmount;
      fees: TransactionFee[];
      estimatedCompletion: ISO8601DateTime;
      requiresAction?: RequiredAction;
    };
  };

  // 거래 조회
  getTransaction: {
    endpoint: 'GET /v1/transactions/{transactionId}';
    authorization: 'participant';

    response: {
      transaction: CBDCTransaction;
      proofs?: TransactionProofs;
    };
  };

  // 거래 목록
  listTransactions: {
    endpoint: 'GET /v1/wallets/{walletId}/transactions';
    authorization: 'owner';

    queryParams: {
      startDate?: ISO8601DateTime;
      endDate?: ISO8601DateTime;
      type?: TransactionType[];
      status?: TransactionStatus[];
      minAmount?: number;
      maxAmount?: number;
      limit?: number;
      cursor?: string;
    };

    response: {
      transactions: TransactionSummary[];
      pagination: {
        nextCursor?: string;
        hasMore: boolean;
        total: number;
      };
    };
  };
}

// 결제 유형
interface PayeeIdentifier {
  type: 'WALLET_ADDRESS' | 'ACCOUNT_NUMBER' | 'PHONE' | 'EMAIL' | 'QR_CODE' | 'MERCHANT_ID';
  value: string;
  verified?: boolean;
}

interface PaymentMetadata {
  description?: string;
  merchantInfo?: {
    merchantId: string;
    merchantName: string;
    merchantCategory: string;
    terminalId?: string;
  };
  invoiceRef?: string;
  orderId?: string;
  customFields?: Record<string, string>;
}

enum PaymentType {
  INSTANT = 'INSTANT',            // 즉시
  SCHEDULED = 'SCHEDULED',        // 예약
  RECURRING = 'RECURRING',        // 반복
  CONDITIONAL = 'CONDITIONAL',    // 조건부
  OFFLINE = 'OFFLINE'             // 오프라인
}

// 거래 API 구현
class TransactionAPIController {
  constructor(
    private transactionService: TransactionService,
    private paymentRouter: PaymentRouter,
    private notificationService: NotificationService
  ) {}

  @Post('/payments')
  @UseGuards(WalletOwnerGuard)
  async initiatePayment(
    @Body() request: PaymentRequest,
    @CurrentWallet() wallet: Wallet
  ): Promise<PaymentResponse> {
    // 수취인 확인
    const payee = await this.paymentRouter.resolvePayee(
      request.payeeIdentifier
    );

    if (!payee) {
      throw new BadRequestException('유효하지 않은 수취인 식별자');
    }

    // 한도 확인
    const limitCheck = await this.limitService.checkTransactionLimits(
      wallet.accountId,
      request.amount,
      request.paymentType
    );

    if (!limitCheck.allowed) {
      if (limitCheck.requiresAction) {
        return {
          status: TransactionStatus.PENDING_AUTHORIZATION,
          requiresAction: limitCheck.requiredAction
        };
      }
      throw new BadRequestException(limitCheck.reason);
    }

    // 수수료 계산
    const fees = await this.feeService.calculateFees(
      request.amount,
      request.paymentType,
      wallet.accountId,
      payee.accountId
    );

    // 거래 생성
    const transaction = await this.transactionService.createPayment({
      payer: {
        walletId: wallet.walletId,
        accountId: wallet.accountId
      },
      payee,
      amount: request.amount,
      fees,
      type: request.paymentType,
      metadata: request.metadata,
      scheduledTime: request.scheduledTime
    });

    // 즉시 결제인 경우 실행
    if (request.paymentType === PaymentType.INSTANT) {
      const result = await this.transactionService.executePayment(
        transaction.transactionId
      );

      // 알림 전송
      await this.notificationService.notifyPayment(result);

      return {
        paymentId: result.transactionId,
        status: result.status,
        amount: result.amount,
        fees: result.fees,
        estimatedCompletion: result.settledAt || new Date().toISOString()
      };
    }

    return {
      paymentId: transaction.transactionId,
      status: transaction.status,
      amount: transaction.amount,
      fees,
      estimatedCompletion: request.scheduledTime || this.estimateCompletion()
    };
  }
}
```

### 4.3 WebSocket 실시간 API

```typescript
// 실시간 업데이트를 위한 WebSocket API
interface CBDCWebSocketAPI {
  // 연결
  connect: {
    url: 'wss://api.cbdc.{jurisdiction}.gov/v1/ws';
    authentication: '연결 헤더의 JWT 토큰';
  };

  // 구독
  subscriptions: {
    // 잔액 업데이트
    balanceUpdates: {
      subscribe: {
        type: 'subscribe';
        channel: 'balance';
        walletIds: string[];
      };
      message: {
        type: 'balance_update';
        walletId: string;
        previousBalance: MonetaryAmount;
        newBalance: MonetaryAmount;
        reason: string;
        transactionId?: string;
        timestamp: ISO8601DateTime;
      };
    };

    // 거래 업데이트
    transactionUpdates: {
      subscribe: {
        type: 'subscribe';
        channel: 'transactions';
        walletIds: string[];
        types?: TransactionType[];
      };
      message: {
        type: 'transaction_update';
        transactionId: string;
        walletId: string;
        previousStatus: TransactionStatus;
        newStatus: TransactionStatus;
        details: TransactionSummary;
        timestamp: ISO8601DateTime;
      };
    };

    // 입금 알림
    incomingPayments: {
      subscribe: {
        type: 'subscribe';
        channel: 'incoming';
        walletIds: string[];
      };
      message: {
        type: 'incoming_payment';
        transactionId: string;
        toWallet: string;
        fromWallet: string;
        amount: MonetaryAmount;
        status: TransactionStatus;
        timestamp: ISO8601DateTime;
      };
    };
  };
}

// WebSocket 핸들러 구현
class CBDCWebSocketHandler {
  private connections: Map<string, WebSocket> = new Map();
  private subscriptions: Map<string, Set<string>> = new Map();

  constructor(
    private authService: AuthService,
    private eventBus: EventBus
  ) {
    this.setupEventListeners();
  }

  @WebSocketConnect()
  async handleConnection(
    socket: WebSocket,
    token: string
  ): Promise<void> {
    // 토큰 검증
    const user = await this.authService.validateToken(token);

    if (!user) {
      socket.close(4001, '유효하지 않은 인증');
      return;
    }

    const connectionId = this.generateConnectionId();
    this.connections.set(connectionId, socket);

    socket.send(JSON.stringify({
      type: 'connected',
      connectionId,
      userId: user.id
    }));

    socket.on('message', (data) => this.handleMessage(connectionId, user, data));
    socket.on('close', () => this.handleDisconnect(connectionId));
  }

  private setupEventListeners(): void {
    // 잔액 업데이트
    this.eventBus.on('balance.updated', (event: BalanceUpdateEvent) => {
      this.broadcastToWalletSubscribers('balance', event.walletId, {
        type: 'balance_update',
        walletId: event.walletId,
        previousBalance: event.previousBalance,
        newBalance: event.newBalance,
        reason: event.reason,
        transactionId: event.transactionId,
        timestamp: event.timestamp
      });
    });

    // 거래 업데이트
    this.eventBus.on('transaction.updated', (event: TransactionUpdateEvent) => {
      const walletIds = [event.senderWallet, event.receiverWallet].filter(Boolean);

      walletIds.forEach(walletId => {
        this.broadcastToWalletSubscribers('transactions', walletId, {
          type: 'transaction_update',
          transactionId: event.transactionId,
          walletId,
          previousStatus: event.previousStatus,
          newStatus: event.newStatus,
          details: event.summary,
          timestamp: event.timestamp
        });
      });
    });
  }
}
```

### 4.4 API 보안

```typescript
// API 보안 구성
interface APISecurityConfig {
  authentication: {
    oauth2: {
      authorizationEndpoint: string;
      tokenEndpoint: string;
      scopes: string[];
      clientCredentials: boolean;
      authorizationCode: boolean;
    };
    mtls: {
      required: boolean;
      certificateValidation: 'STRICT' | 'CHAIN' | 'NONE';
      clientCertificateHeader?: string;
    };
    apiKey: {
      enabled: boolean;
      headerName: string;
      rotationPolicy: string;
    };
  };

  authorization: {
    rbac: boolean;  // 역할 기반 접근 제어
    abac: boolean;  // 속성 기반 접근 제어
    policyEngine: string;
  };

  rateLimit: {
    enabled: boolean;
    defaultLimit: number;
    windowMs: number;
    keyBy: 'IP' | 'USER' | 'API_KEY';
  };

  encryption: {
    transportSecurity: 'TLS1.3';
    payloadEncryption: boolean;
    signedRequests: boolean;
  };
}

// 보안 미들웨어
class APISecurityMiddleware {
  async authenticate(req: Request): Promise<AuthResult> {
    // mTLS 인증서 확인
    if (this.config.mtls.required) {
      const cert = req.headers[this.config.mtls.clientCertificateHeader!];
      if (!cert || !await this.validateCertificate(cert)) {
        throw new UnauthorizedError('유효하지 않은 클라이언트 인증서');
      }
    }

    // OAuth 토큰 확인
    const authHeader = req.headers.authorization;
    if (authHeader?.startsWith('Bearer ')) {
      const token = authHeader.substring(7);
      return this.validateOAuthToken(token);
    }

    // API 키 확인
    if (this.config.apiKey.enabled) {
      const apiKey = req.headers[this.config.apiKey.headerName];
      if (apiKey) {
        return this.validateApiKey(apiKey as string);
      }
    }

    throw new UnauthorizedError('유효한 인증이 제공되지 않았습니다');
  }

  async authorize(
    user: AuthenticatedUser,
    resource: string,
    action: string
  ): Promise<boolean> {
    if (this.config.authorization.abac) {
      return this.abacEngine.evaluate({
        subject: user,
        resource,
        action,
        context: this.buildContext()
      });
    }

    if (this.config.authorization.rbac) {
      return this.checkRolePermission(user.roles, resource, action);
    }

    return false;
  }
}
```

### 4.5 요약

WIA-CBDC API 계층이 제공하는 것:

1. **종합적인 토큰 API**: 전체 생명주기 관리
2. **계좌/지갑 API**: 완전한 계좌 운영
3. **거래 API**: 모든 결제 유형 지원
4. **국경간 API**: 다자간 CBDC 코리도 지원
5. **실시간 API**: WebSocket 및 GraphQL 구독
6. **엔터프라이즈 보안**: OAuth 2.0, mTLS, ABAC

---

**WIA-CBDC API 명세**
**버전**: 1.0.0
**최종 업데이트**: 2025

© 2025 WIA (World Interoperability Alliance)
