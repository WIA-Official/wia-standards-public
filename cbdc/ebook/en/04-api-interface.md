# Chapter 4: CBDC API Specifications and Integration Patterns

## Comprehensive API Architecture for Central Bank Digital Currency Systems

### 4.1 API Architecture Overview

The WIA-CBDC API layer provides standardized interfaces for all CBDC operations, from token issuance to cross-border transactions. The architecture follows RESTful principles with GraphQL support for complex queries.

```typescript
// API Architecture Definition
interface WIACBDCAPIArchitecture {
  version: '1.0.0';
  baseUrl: 'https://api.cbdc.{jurisdiction}.gov/v1';

  layers: {
    public: {
      description: 'Public APIs for wallet apps and merchants';
      authentication: 'OAuth 2.0 / API Key';
      rateLimit: '1000 requests/minute';
    };
    partner: {
      description: 'APIs for authorized financial institutions';
      authentication: 'mTLS + JWT';
      rateLimit: '10000 requests/minute';
    };
    interbank: {
      description: 'APIs for interbank settlement';
      authentication: 'mTLS + HSM signatures';
      rateLimit: 'Unlimited with quotas';
    };
    centralBank: {
      description: 'Internal central bank operations';
      authentication: 'Multi-party authorization';
      rateLimit: 'Policy controlled';
    };
  };

  protocols: {
    rest: 'Primary API interface';
    graphql: 'Complex queries and subscriptions';
    websocket: 'Real-time notifications';
    grpc: 'High-performance interbank';
  };

  standards: {
    messaging: 'ISO 20022';
    security: 'OAuth 2.0, FAPI 2.0';
    dataFormats: 'JSON, Protocol Buffers';
  };
}
```

### 4.2 Core API Endpoints

#### 4.2.1 Token Management APIs

```typescript
// Token Management API Specification
interface TokenManagementAPI {
  // Token Issuance (Central Bank Only)
  issuance: {
    endpoint: 'POST /v1/tokens/issue';
    authorization: 'multi-party-auth';

    request: {
      issuanceRequest: {
        requestId: string;           // Idempotency key
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

  // Token Transfer
  transfer: {
    endpoint: 'POST /v1/tokens/transfer';
    authorization: 'owner-signature';

    request: {
      transferRequest: {
        requestId: string;
        fromWallet: string;
        toWallet: string;
        amount: MonetaryAmount;
        tokenIds?: string[];         // Specific tokens or auto-select
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

  // Token Redemption
  redemption: {
    endpoint: 'POST /v1/tokens/redeem';
    authorization: 'owner-signature';

    request: {
      redemptionRequest: {
        requestId: string;
        tokenIds: string[];
        destinationType: 'BANK_ACCOUNT' | 'CASH' | 'DESTRUCTION';
        destinationDetails?: RedemptionDestination;
      };
      signature: OwnerSignature;
    };

    response: {
      redemptionId: string;
      status: RedemptionStatus;
      amount: MonetaryAmount;
      settlementDetails?: SettlementDetails;
      timestamp: ISO8601DateTime;
    };
  };

  // Token Query
  query: {
    endpoint: 'GET /v1/tokens/{tokenId}';
    authorization: 'owner-or-auditor';

    response: {
      token: CBDCToken;
      verificationProof: string;
    };
  };

  // Balance Query
  balance: {
    endpoint: 'GET /v1/wallets/{walletId}/balance';
    authorization: 'owner';

    response: {
      walletId: string;
      balances: {
        available: MonetaryAmount;
        pending: MonetaryAmount;
        locked: MonetaryAmount;
        total: MonetaryAmount;
      };
      tokenCount: number;
      lastUpdated: ISO8601DateTime;
    };
  };
}

// Token API Implementation
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
    // Validate request
    const validation = await this.validateTransferRequest(request);
    if (!validation.valid) {
      throw new BadRequestException(validation.errors);
    }

    // Verify ownership
    const ownershipVerified = await this.tokenService.verifyOwnership(
      request.fromWallet,
      user.walletId
    );
    if (!ownershipVerified) {
      throw new ForbiddenException('Wallet ownership verification failed');
    }

    // Check balance
    const balance = await this.tokenService.getBalance(request.fromWallet);
    if (balance.available.valueInSmallestUnit < request.amount.valueInSmallestUnit) {
      throw new BadRequestException('Insufficient balance');
    }

    // Compliance screening
    const screeningResult = await this.complianceService.screenTransfer({
      sender: request.fromWallet,
      receiver: request.toWallet,
      amount: request.amount
    });

    if (screeningResult.blocked) {
      throw new ForbiddenException('Transaction blocked by compliance');
    }

    // Execute transfer
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

  @Post('/tokens/issue')
  @UseGuards(CentralBankAuthGuard, MultiPartyAuthGuard)
  async issue(
    @Body() request: IssuanceRequest,
    @Authorizations() auths: Authorization[]
  ): Promise<IssuanceResponse> {
    // Verify multi-party authorization
    const authValid = await this.authService.verifyMultiPartyAuth(
      auths,
      'ISSUANCE',
      request.amount
    );

    if (!authValid) {
      throw new ForbiddenException('Insufficient authorization');
    }

    // Validate against monetary policy
    const policyCheck = await this.monetaryPolicyService.validateIssuance(
      request.amount,
      request.denominationProfile
    );

    if (!policyCheck.allowed) {
      throw new BadRequestException(policyCheck.reason);
    }

    // Execute issuance
    const result = await this.tokenService.issueTokens(request);

    // Audit logging
    await this.auditService.logIssuance(result, auths);

    return {
      issuanceId: result.issuanceId,
      status: result.status,
      tokensCreated: result.tokens.length,
      totalAmount: result.totalAmount,
      tokenIds: result.tokens.map(t => t.tokenId),
      timestamp: result.timestamp
    };
  }
}
```

#### 4.2.2 Account and Wallet APIs

```typescript
// Account Management API
interface AccountManagementAPI {
  // Create Account
  createAccount: {
    endpoint: 'POST /v1/accounts';
    authorization: 'partner-api';

    request: {
      accountRequest: {
        accountType: AccountType;
        holderInfo: {
          participantType: ParticipantType;
          identityRef: string;       // Reference to KYC identity
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

  // Get Account
  getAccount: {
    endpoint: 'GET /v1/accounts/{accountId}';
    authorization: 'owner-or-partner';

    response: {
      account: CBDCAccount;
      wallets: WalletSummary[];
    };
  };

  // Update Account Limits
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

  // Freeze Account
  freezeAccount: {
    endpoint: 'POST /v1/accounts/{accountId}/freeze';
    authorization: 'compliance-officer';

    request: {
      reason: FreezeReason;
      duration?: number;           // Hours, undefined = indefinite
      referenceNumber?: string;    // Legal/compliance reference
    };

    response: {
      accountId: string;
      status: 'FROZEN';
      frozenAt: ISO8601DateTime;
      unfreezeAt?: ISO8601DateTime;
    };
  };
}

// Wallet Management API
interface WalletManagementAPI {
  // Create Wallet
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

  // Get Wallet
  getWallet: {
    endpoint: 'GET /v1/wallets/{walletId}';
    authorization: 'owner';

    response: {
      wallet: CBDCWallet;
      balance: WalletBalance;
      recentTransactions: TransactionSummary[];
    };
  };

  // Bind Device
  bindDevice: {
    endpoint: 'POST /v1/wallets/{walletId}/devices';
    authorization: 'owner-with-mfa';

    request: {
      deviceInfo: {
        deviceId: string;
        deviceType: DeviceType;
        deviceFingerprint: string;
        attestation?: DeviceAttestation;
      };
    };

    response: {
      bindingId: string;
      walletId: string;
      deviceId: string;
      boundAt: ISO8601DateTime;
    };
  };

  // Generate Offline Tokens
  generateOfflineTokens: {
    endpoint: 'POST /v1/wallets/{walletId}/offline-tokens';
    authorization: 'owner';

    request: {
      amount: MonetaryAmount;
      duration: number;            // Hours
      maxTransactions?: number;
    };

    response: {
      offlineTokens: OfflineToken[];
      validUntil: ISO8601DateTime;
      maxOfflineBalance: MonetaryAmount;
    };
  };
}

// Account API Implementation
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
    // Verify KYC identity
    const kycResult = await this.kycService.verifyIdentity(
      request.holderInfo.identityRef,
      partner.partnerId
    );

    if (!kycResult.verified) {
      throw new BadRequestException('Identity verification failed');
    }

    // Determine limits based on KYC level
    const limits = await this.limitService.determineAccountLimits(
      request.accountType,
      kycResult.kycLevel
    );

    // Create account
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

  @Get('/accounts/:accountId')
  @UseGuards(AccountAccessGuard)
  async getAccount(
    @Param('accountId') accountId: string,
    @CurrentUser() user: AuthenticatedUser
  ): Promise<AccountResponse> {
    const account = await this.accountService.getAccount(accountId);

    if (!account) {
      throw new NotFoundException('Account not found');
    }

    // Get associated wallets
    const wallets = await this.walletService.getWalletsForAccount(accountId);

    // Apply privacy rules
    const maskedAccount = this.applyPrivacyRules(account, user);

    return {
      account: maskedAccount,
      wallets: wallets.map(w => this.summarizeWallet(w))
    };
  }
}
```

#### 4.2.3 Transaction APIs

```typescript
// Transaction API Specification
interface TransactionAPI {
  // Initiate Payment
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

  // Get Transaction
  getTransaction: {
    endpoint: 'GET /v1/transactions/{transactionId}';
    authorization: 'participant';

    response: {
      transaction: CBDCTransaction;
      proofs?: TransactionProofs;
    };
  };

  // List Transactions
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

  // Cancel Transaction
  cancelTransaction: {
    endpoint: 'POST /v1/transactions/{transactionId}/cancel';
    authorization: 'initiator';

    request: {
      reason: string;
    };

    response: {
      transactionId: string;
      status: 'CANCELLED' | 'CANCELLATION_FAILED';
      refundDetails?: RefundDetails;
    };
  };

  // Request Refund
  requestRefund: {
    endpoint: 'POST /v1/transactions/{transactionId}/refund';
    authorization: 'payee';

    request: {
      refundRequest: {
        amount: MonetaryAmount;     // Full or partial
        reason: string;
        evidence?: RefundEvidence[];
      };
    };

    response: {
      refundId: string;
      status: RefundStatus;
      amount: MonetaryAmount;
      processedAt?: ISO8601DateTime;
    };
  };
}

// Payment Types
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
  INSTANT = 'INSTANT',
  SCHEDULED = 'SCHEDULED',
  RECURRING = 'RECURRING',
  CONDITIONAL = 'CONDITIONAL',
  OFFLINE = 'OFFLINE'
}

interface RequiredAction {
  actionType: 'MFA' | 'DEVICE_APPROVAL' | 'LIMIT_INCREASE' | 'KYC_UPGRADE';
  actionUrl?: string;
  expiresAt: ISO8601DateTime;
}

// Transaction API Implementation
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
    // Resolve payee
    const payee = await this.paymentRouter.resolvePayee(
      request.payeeIdentifier
    );

    if (!payee) {
      throw new BadRequestException('Invalid payee identifier');
    }

    // Check limits
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

    // Calculate fees
    const fees = await this.feeService.calculateFees(
      request.amount,
      request.paymentType,
      wallet.accountId,
      payee.accountId
    );

    // Create transaction
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

    // Execute if instant
    if (request.paymentType === PaymentType.INSTANT) {
      const result = await this.transactionService.executePayment(
        transaction.transactionId
      );

      // Send notifications
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

  @Get('/wallets/:walletId/transactions')
  @UseGuards(WalletOwnerGuard)
  async listTransactions(
    @Param('walletId') walletId: string,
    @Query() query: TransactionQueryParams
  ): Promise<TransactionListResponse> {
    const filters = this.buildFilters(query);

    const result = await this.transactionService.listTransactions(
      walletId,
      filters,
      {
        limit: query.limit || 20,
        cursor: query.cursor
      }
    );

    return {
      transactions: result.items.map(t => this.summarizeTransaction(t)),
      pagination: {
        nextCursor: result.nextCursor,
        hasMore: result.hasMore,
        total: result.total
      }
    };
  }
}
```

### 4.3 Cross-Border APIs

```typescript
// Cross-Border Payment API
interface CrossBorderAPI {
  // Initiate Cross-Border Payment
  initiateCrossBorder: {
    endpoint: 'POST /v1/cross-border/payments';
    authorization: 'payer-with-enhanced-kyc';

    request: {
      crossBorderRequest: {
        requestId: string;
        sourceWallet: string;
        sourceCurrency: string;
        destinationIdentifier: CrossBorderDestination;
        destinationCurrency: string;
        sourceAmount?: MonetaryAmount;
        destinationAmount?: MonetaryAmount;
        purpose: RemittancePurpose;
        corridor: string;
        metadata?: CrossBorderMetadata;
      };
    };

    response: {
      paymentId: string;
      status: CrossBorderTransactionStatus;
      fxQuote: FXQuote;
      fees: CrossBorderFees;
      estimatedArrival: ISO8601DateTime;
      requiredDocuments?: DocumentRequirement[];
    };
  };

  // Get FX Quote
  getFXQuote: {
    endpoint: 'POST /v1/cross-border/quotes';
    authorization: 'authenticated';

    request: {
      sourceCurrency: string;
      destinationCurrency: string;
      amount: MonetaryAmount;
      direction: 'SOURCE' | 'DESTINATION';
      corridor: string;
    };

    response: {
      quoteId: string;
      exchangeRate: number;
      inverseRate: number;
      sourceAmount: MonetaryAmount;
      destinationAmount: MonetaryAmount;
      fees: CrossBorderFees;
      validUntil: ISO8601DateTime;
      rateGuarantee: boolean;
    };
  };

  // Get Supported Corridors
  getCorridors: {
    endpoint: 'GET /v1/cross-border/corridors';
    authorization: 'public';

    response: {
      corridors: Corridor[];
    };
  };

  // Track Cross-Border Payment
  trackPayment: {
    endpoint: 'GET /v1/cross-border/payments/{paymentId}/track';
    authorization: 'participant';

    response: {
      paymentId: string;
      status: CrossBorderTransactionStatus;
      timeline: PaymentTimelineEvent[];
      currentStage: string;
      estimatedCompletion: ISO8601DateTime;
    };
  };
}

interface CrossBorderDestination {
  type: 'CBDC_WALLET' | 'BANK_ACCOUNT' | 'MOBILE_MONEY';
  country: string;
  identifier: string;
  beneficiaryName: string;
  beneficiaryAddress?: Address;
}

interface FXQuote {
  quoteId: string;
  sourceCurrency: string;
  destinationCurrency: string;
  exchangeRate: number;
  sourceAmount: MonetaryAmount;
  destinationAmount: MonetaryAmount;
  timestamp: ISO8601DateTime;
  validUntil: ISO8601DateTime;
  provider: string;
}

interface CrossBorderFees {
  fxSpread: MonetaryAmount;
  networkFee: MonetaryAmount;
  originFee: MonetaryAmount;
  destinationFee: MonetaryAmount;
  totalFees: MonetaryAmount;
}

interface Corridor {
  corridorId: string;
  sourceCurrency: string;
  destinationCurrency: string;
  sourceCountry: string;
  destinationCountry: string;
  minAmount: MonetaryAmount;
  maxAmount: MonetaryAmount;
  estimatedTime: string;
  available: boolean;
  settlementMechanism: string;
}

// Cross-Border API Implementation
class CrossBorderAPIController {
  constructor(
    private crossBorderService: CrossBorderService,
    private fxService: FXService,
    private complianceService: CrossBorderComplianceService
  ) {}

  @Post('/cross-border/payments')
  @UseGuards(EnhancedKYCGuard)
  async initiateCrossBorder(
    @Body() request: CrossBorderRequest,
    @CurrentUser() user: User
  ): Promise<CrossBorderResponse> {
    // Validate corridor
    const corridor = await this.crossBorderService.getCorridor(
      request.sourceCurrency,
      request.destinationCurrency
    );

    if (!corridor || !corridor.available) {
      throw new BadRequestException('Corridor not available');
    }

    // Get FX quote
    const quote = await this.fxService.getQuote({
      sourceCurrency: request.sourceCurrency,
      destinationCurrency: request.destinationCurrency,
      amount: request.sourceAmount || request.destinationAmount!,
      direction: request.sourceAmount ? 'SOURCE' : 'DESTINATION'
    });

    // Cross-border compliance check
    const complianceResult = await this.complianceService.screenCrossBorder({
      sender: user,
      destination: request.destinationIdentifier,
      amount: quote.sourceAmount,
      purpose: request.purpose
    });

    if (complianceResult.blocked) {
      throw new ForbiddenException(complianceResult.reason);
    }

    // Check for required documents
    const documentRequirements = await this.complianceService.getRequiredDocuments(
      quote.sourceAmount,
      request.purpose,
      request.destinationIdentifier.country
    );

    // Create cross-border transaction
    const transaction = await this.crossBorderService.createTransaction({
      sourceWallet: request.sourceWallet,
      destination: request.destinationIdentifier,
      quote,
      purpose: request.purpose,
      metadata: request.metadata,
      complianceResult
    });

    return {
      paymentId: transaction.transactionId,
      status: transaction.status,
      fxQuote: quote,
      fees: transaction.fees,
      estimatedArrival: transaction.estimatedArrival,
      requiredDocuments: documentRequirements
    };
  }

  @Post('/cross-border/quotes')
  async getFXQuote(
    @Body() request: FXQuoteRequest
  ): Promise<FXQuoteResponse> {
    const quote = await this.fxService.getQuote(request);

    return {
      quoteId: quote.quoteId,
      exchangeRate: quote.rate,
      inverseRate: 1 / quote.rate,
      sourceAmount: quote.sourceAmount,
      destinationAmount: quote.destinationAmount,
      fees: await this.feeService.calculateCrossBorderFees(quote),
      validUntil: quote.validUntil,
      rateGuarantee: quote.guaranteed
    };
  }
}
```

### 4.4 WebSocket Real-Time APIs

```typescript
// WebSocket API for Real-Time Updates
interface CBDCWebSocketAPI {
  // Connection
  connect: {
    url: 'wss://api.cbdc.{jurisdiction}.gov/v1/ws';
    authentication: 'JWT token in connection header';
  };

  // Subscriptions
  subscriptions: {
    // Balance updates
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

    // Transaction updates
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

    // Incoming payments
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

    // System notifications
    systemNotifications: {
      subscribe: {
        type: 'subscribe';
        channel: 'system';
      };
      message: {
        type: 'system_notification';
        severity: 'INFO' | 'WARNING' | 'CRITICAL';
        category: string;
        message: string;
        affectedServices?: string[];
        timestamp: ISO8601DateTime;
      };
    };
  };
}

// WebSocket Handler Implementation
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
    // Validate token
    const user = await this.authService.validateToken(token);

    if (!user) {
      socket.close(4001, 'Invalid authentication');
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

  private async handleMessage(
    connectionId: string,
    user: User,
    data: WebSocket.Data
  ): Promise<void> {
    const message = JSON.parse(data.toString());

    switch (message.type) {
      case 'subscribe':
        await this.handleSubscribe(connectionId, user, message);
        break;
      case 'unsubscribe':
        await this.handleUnsubscribe(connectionId, message);
        break;
      case 'ping':
        this.sendToConnection(connectionId, { type: 'pong' });
        break;
    }
  }

  private async handleSubscribe(
    connectionId: string,
    user: User,
    message: SubscribeMessage
  ): Promise<void> {
    // Verify access to requested wallets
    if (message.walletIds) {
      const authorized = await this.authService.verifyWalletAccess(
        user.id,
        message.walletIds
      );

      if (!authorized) {
        this.sendToConnection(connectionId, {
          type: 'error',
          message: 'Unauthorized wallet access'
        });
        return;
      }
    }

    // Add subscription
    const channel = `${message.channel}:${connectionId}`;
    if (!this.subscriptions.has(channel)) {
      this.subscriptions.set(channel, new Set());
    }

    message.walletIds?.forEach(walletId => {
      this.subscriptions.get(channel)!.add(walletId);
    });

    this.sendToConnection(connectionId, {
      type: 'subscribed',
      channel: message.channel,
      walletIds: message.walletIds
    });
  }

  private setupEventListeners(): void {
    // Balance updates
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

    // Transaction updates
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

  private broadcastToWalletSubscribers(
    channel: string,
    walletId: string,
    message: any
  ): void {
    for (const [key, wallets] of this.subscriptions.entries()) {
      if (key.startsWith(channel) && wallets.has(walletId)) {
        const connectionId = key.split(':')[1];
        this.sendToConnection(connectionId, message);
      }
    }
  }
}
```

### 4.5 GraphQL API

```typescript
// GraphQL Schema for Complex Queries
const CBDCGraphQLSchema = `
  type Query {
    # Account queries
    account(id: ID!): Account
    accounts(filter: AccountFilter, pagination: PaginationInput): AccountConnection!

    # Wallet queries
    wallet(id: ID!): Wallet
    wallets(accountId: ID!, filter: WalletFilter): [Wallet!]!

    # Transaction queries
    transaction(id: ID!): Transaction
    transactions(
      walletId: ID!
      filter: TransactionFilter
      pagination: PaginationInput
    ): TransactionConnection!

    # Analytics queries
    transactionAnalytics(
      walletId: ID!
      period: AnalyticsPeriod!
      groupBy: GroupByOption
    ): TransactionAnalytics!

    # Token queries
    token(id: ID!): Token
    tokensByWallet(walletId: ID!, filter: TokenFilter): [Token!]!
  }

  type Mutation {
    # Payment mutations
    initiatePayment(input: PaymentInput!): PaymentResult!
    cancelPayment(paymentId: ID!, reason: String!): CancelResult!

    # Wallet mutations
    createWallet(input: CreateWalletInput!): Wallet!
    updateWalletSettings(walletId: ID!, settings: WalletSettingsInput!): Wallet!

    # Cross-border mutations
    initiateCrossBorderPayment(input: CrossBorderInput!): CrossBorderResult!
    acceptFXQuote(quoteId: ID!): FXQuoteAcceptance!
  }

  type Subscription {
    # Real-time subscriptions
    balanceUpdated(walletIds: [ID!]!): BalanceUpdate!
    transactionUpdated(walletIds: [ID!]!): TransactionUpdate!
    incomingPayment(walletIds: [ID!]!): IncomingPayment!
  }

  # Types
  type Account {
    id: ID!
    accountNumber: String!
    accountType: AccountType!
    holder: ParticipantInfo!
    balances: Balances!
    limits: AccountLimits!
    status: AccountStatus!
    wallets: [Wallet!]!
    createdAt: DateTime!
    lastActivityAt: DateTime
  }

  type Wallet {
    id: ID!
    walletAddress: String!
    walletType: WalletType!
    account: Account!
    balance: WalletBalance!
    tokens(filter: TokenFilter): [Token!]!
    transactions(
      filter: TransactionFilter
      pagination: PaginationInput
    ): TransactionConnection!
    capabilities: WalletCapabilities!
    createdAt: DateTime!
  }

  type Transaction {
    id: ID!
    referenceNumber: String!
    type: TransactionType!
    subType: TransactionSubType
    amount: MonetaryAmount!
    fees: [TransactionFee!]
    sender: TransactionParty
    receiver: TransactionParty
    status: TransactionStatus!
    timing: TransactionTiming!
    metadata: TransactionMetadata
  }

  type Token {
    id: ID!
    serialNumber: String!
    denomination: MonetaryAmount!
    tokenType: TokenType!
    status: TokenStatus!
    conditions: [TokenCondition!]
    issuedAt: DateTime!
    expiresAt: DateTime
    history: [OwnershipRecord!]!
  }

  type TransactionAnalytics {
    period: AnalyticsPeriod!
    totalVolume: MonetaryAmount!
    transactionCount: Int!
    averageTransaction: MonetaryAmount!
    byType: [TypeBreakdown!]!
    byDay: [DailyStats!]!
    topMerchants: [MerchantStats!]
    categoryBreakdown: [CategoryStats!]
  }

  # Input types
  input PaymentInput {
    payerWallet: ID!
    payeeIdentifier: PayeeIdentifierInput!
    amount: MonetaryAmountInput!
    paymentType: PaymentType!
    memo: String
    metadata: JSON
  }

  input TransactionFilter {
    types: [TransactionType!]
    statuses: [TransactionStatus!]
    startDate: DateTime
    endDate: DateTime
    minAmount: Float
    maxAmount: Float
    counterparty: String
  }

  input PaginationInput {
    limit: Int
    cursor: String
    sortBy: String
    sortOrder: SortOrder
  }
`;

// GraphQL Resolvers
const resolvers = {
  Query: {
    account: async (_, { id }, context) => {
      const account = await context.accountService.getAccount(id);
      if (!await context.authService.canAccessAccount(context.user, id)) {
        throw new ForbiddenError('Access denied');
      }
      return account;
    },

    transactions: async (_, { walletId, filter, pagination }, context) => {
      if (!await context.authService.canAccessWallet(context.user, walletId)) {
        throw new ForbiddenError('Access denied');
      }

      return context.transactionService.listTransactions(
        walletId,
        filter,
        pagination
      );
    },

    transactionAnalytics: async (_, { walletId, period, groupBy }, context) => {
      if (!await context.authService.canAccessWallet(context.user, walletId)) {
        throw new ForbiddenError('Access denied');
      }

      return context.analyticsService.getTransactionAnalytics(
        walletId,
        period,
        groupBy
      );
    }
  },

  Mutation: {
    initiatePayment: async (_, { input }, context) => {
      return context.paymentService.initiatePayment(input, context.user);
    },

    initiateCrossBorderPayment: async (_, { input }, context) => {
      return context.crossBorderService.initiatePayment(input, context.user);
    }
  },

  Subscription: {
    balanceUpdated: {
      subscribe: (_, { walletIds }, context) => {
        // Verify access to all wallets
        return context.pubsub.asyncIterator(
          walletIds.map(id => `BALANCE_UPDATED_${id}`)
        );
      }
    },

    transactionUpdated: {
      subscribe: (_, { walletIds }, context) => {
        return context.pubsub.asyncIterator(
          walletIds.map(id => `TRANSACTION_UPDATED_${id}`)
        );
      }
    }
  },

  // Field resolvers
  Account: {
    wallets: async (account, _, context) => {
      return context.walletService.getWalletsForAccount(account.id);
    }
  },

  Wallet: {
    transactions: async (wallet, { filter, pagination }, context) => {
      return context.transactionService.listTransactions(
        wallet.id,
        filter,
        pagination
      );
    },

    tokens: async (wallet, { filter }, context) => {
      return context.tokenService.getTokensByWallet(wallet.id, filter);
    }
  }
};
```

### 4.6 API Security

```typescript
// API Security Configuration
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
    rbac: boolean;
    abac: boolean;
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

// Security Middleware
class APISecurityMiddleware {
  async authenticate(req: Request): Promise<AuthResult> {
    // Check mTLS certificate
    if (this.config.mtls.required) {
      const cert = req.headers[this.config.mtls.clientCertificateHeader!];
      if (!cert || !await this.validateCertificate(cert)) {
        throw new UnauthorizedError('Invalid client certificate');
      }
    }

    // Check OAuth token
    const authHeader = req.headers.authorization;
    if (authHeader?.startsWith('Bearer ')) {
      const token = authHeader.substring(7);
      return this.validateOAuthToken(token);
    }

    // Check API key
    if (this.config.apiKey.enabled) {
      const apiKey = req.headers[this.config.apiKey.headerName];
      if (apiKey) {
        return this.validateApiKey(apiKey as string);
      }
    }

    throw new UnauthorizedError('No valid authentication provided');
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

### 4.7 Summary

The WIA-CBDC API layer provides:

1. **Comprehensive Token APIs**: Full lifecycle management
2. **Account/Wallet APIs**: Complete account operations
3. **Transaction APIs**: All payment types supported
4. **Cross-Border APIs**: Multi-CBDC corridor support
5. **Real-Time APIs**: WebSocket and GraphQL subscriptions
6. **Enterprise Security**: OAuth 2.0, mTLS, ABAC

---

**WIA-CBDC API Specifications**
**Version**: 1.0.0
**Last Updated**: 2025

© 2025 WIA (World Interoperability Alliance)
