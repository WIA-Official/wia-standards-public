# WIA-FIN-008 Phase 4: Integration Specification

**Version:** 1.0.0
**Status:** Final
**Last Updated:** 2025-01-20
**Authors:** WIA Standards Committee

## Table of Contents

1. [Introduction](#introduction)
2. [Custody Integration](#custody-integration)
3. [KYC/AML Provider Integration](#kycaml-provider-integration)
4. [Exchange Integration](#exchange-integration)
5. [Wallet Integration](#wallet-integration)
6. [Banking & Fiat On-Ramps](#banking--fiat-on-ramps)
7. [Traditional Brokerage Integration](#traditional-brokerage-integration)
8. [Data Analytics & Reporting](#data-analytics--reporting)
9. [Tax Reporting](#tax-reporting)
10. [End-to-End Integration Architecture](#end-to-end-integration-architecture)

---

## 1. Introduction

Phase 4 of WIA-FIN-008 defines integration patterns for connecting tokenization platforms with real-world financial infrastructure. These integrations enable custody, KYC/AML verification, trading, fiat on/off-ramps, and reporting.

### Design Principles

- **Interoperability:** Standardized interfaces for third-party services
- **Modularity:** Plug-and-play integrations with multiple providers
- **Redundancy:** Multiple provider options for critical services
- **Security:** End-to-end encryption, API key rotation, rate limiting
- **Compliance:** Regulatory requirements embedded in integrations
- **Monitoring:** Health checks, uptime tracking, error alerting

---

## 2. Custody Integration

### Supported Custody Providers

| Provider | Type | API Docs | Integration Complexity |
|----------|------|----------|------------------------|
| **Fireblocks** | MPC | [docs.fireblocks.com](https://docs.fireblocks.com) | Medium |
| **BitGo** | Multi-sig | [bitgo.github.io/BitGoJS](https://bitgo.github.io/BitGoJS) | Low |
| **Anchorage** | Bank custody | [docs.anchorage.com](https://docs.anchorage.com) | Medium |
| **Copper** | MPC + ClearLoop | [docs.copper.co](https://docs.copper.co) | High |

### Fireblocks Integration

#### Setup

```bash
npm install @fireblocks/ts-sdk
```

#### Configuration

```typescript
import { FireblocksSDK } from '@fireblocks/ts-sdk';
import * as fs from 'fs';

const fireblocks = new FireblocksSDK(
  fs.readFileSync(process.env.FIREBLOCKS_PRIVATE_KEY_PATH, 'utf8'),
  process.env.FIREBLOCKS_API_KEY
);
```

#### Create Investor Wallet

```typescript
interface CreateWalletRequest {
  investorId: string;
  email: string;
  name: string;
}

async function createInvestorWallet(
  request: CreateWalletRequest
): Promise<{ vaultId: string; address: string }> {
  // Create vault account
  const vault = await fireblocks.vaults.create({
    name: `${request.name} (${request.investorId})`,
    hiddenOnUI: false,
    customerRefId: request.investorId
  });

  // Create Ethereum wallet within vault
  const account = await fireblocks.vaults.createAccount({
    vaultAccountId: vault.id,
    assetId: 'ETH'
  });

  return {
    vaultId: vault.id,
    address: account.address
  };
}
```

#### Execute Compliant Transfer

```typescript
interface TransferRequest {
  fromVaultId: string;
  toAddress: string;
  amount: string;
  tokenContractAddress: string;
}

async function executeTransfer(
  request: TransferRequest
): Promise<{ txId: string; txHash: string }> {
  // Encode ERC-20 transfer
  const transferData = ethers.utils.defaultAbiCoder.encode(
    ['address', 'uint256'],
    [request.toAddress, ethers.utils.parseUnits(request.amount, 0)]
  );

  const transaction = await fireblocks.transactions.create({
    assetId: 'ETH',
    source: {
      type: 'VAULT_ACCOUNT',
      id: request.fromVaultId
    },
    destination: {
      type: 'EXTERNAL_WALLET',
      oneTimeAddress: request.tokenContractAddress
    },
    amount: '0', // Gas only
    operation: 'CONTRACT_CALL',
    extraParameters: {
      contractCallData: `0xa9059cbb${transferData.slice(2)}` // transfer(address,uint256)
    }
  });

  // Wait for confirmation
  const status = await waitForTransactionCompletion(transaction.id);

  return {
    txId: transaction.id,
    txHash: status.txHash
  };
}
```

### BitGo Integration

```typescript
import { BitGo } from 'bitgo';

const bitgo = new BitGo({
  env: 'prod', // or 'test'
  accessToken: process.env.BITGO_ACCESS_TOKEN
});

async function createBitGoWallet(
  investorId: string
): Promise<{ walletId: string; address: string }> {
  const wallet = await bitgo.coin('eth').wallets().generateWallet({
    label: `Investor-${investorId}`,
    passphrase: process.env.BITGO_WALLET_PASSPHRASE,
    enterprise: process.env.BITGO_ENTERPRISE_ID
  });

  return {
    walletId: wallet.id,
    address: wallet.receiveAddress()
  };
}
```

---

## 3. KYC/AML Provider Integration

### Supported KYC Providers

| Provider | Coverage | Pricing | API Complexity |
|----------|----------|---------|----------------|
| **Onfido** | 195+ countries | $2-5/check | Medium |
| **Jumio** | 200+ countries | $1.50-4/check | Low |
| **Sumsub** | 220+ countries | $0.50-3/check | Medium |
| **Civic** | 150+ countries | $0.10-1/check | Low |

### Onfido Integration

#### Setup

```bash
npm install @onfido/api
```

#### Create Applicant & Start Verification

```typescript
import { Onfido, Region } from '@onfido/api';

const onfido = new Onfido({
  apiToken: process.env.ONFIDO_API_TOKEN,
  region: Region.US
});

interface KYCRequest {
  investorId: string;
  firstName: string;
  lastName: string;
  email: string;
  dateOfBirth: string; // YYYY-MM-DD
}

async function startKYCVerification(
  request: KYCRequest
): Promise<{ applicantId: string; sdkToken: string }> {
  // Create applicant
  const applicant = await onfido.applicant.create({
    firstName: request.firstName,
    lastName: request.lastName,
    email: request.email,
    dob: request.dateOfBirth
  });

  // Generate SDK token for web/mobile flow
  const sdkToken = await onfido.sdkToken.generate({
    applicantId: applicant.id,
    referrer: 'https://your-platform.com/*'
  });

  // Store mapping
  await db.kycApplications.create({
    investorId: request.investorId,
    applicantId: applicant.id,
    provider: 'Onfido',
    status: 'PENDING',
    createdAt: new Date()
  });

  return {
    applicantId: applicant.id,
    sdkToken: sdkToken.token
  };
}
```

#### Check Verification Status

```typescript
async function checkKYCStatus(
  applicantId: string
): Promise<{
  status: 'pending' | 'verified' | 'rejected';
  reasons?: string[];
}> {
  const checks = await onfido.check.list(applicantId);

  if (checks.length === 0) {
    return { status: 'pending' };
  }

  const latestCheck = checks[0];

  if (latestCheck.result === 'clear') {
    return { status: 'verified' };
  }

  const failureReasons = latestCheck.reports
    .filter(r => r.result !== 'clear')
    .map(r => r.breakdown?.reason || 'Unknown reason');

  return {
    status: 'rejected',
    reasons: failureReasons
  };
}
```

#### Webhook Handler

```typescript
import { createHmac } from 'crypto';

function verifyOnfidoWebhook(
  payload: string,
  signature: string
): boolean {
  const expectedSignature = createHmac('sha256', process.env.ONFIDO_WEBHOOK_SECRET)
    .update(payload)
    .digest('hex');

  return signature === expectedSignature;
}

async function handleOnfidoWebhook(req, res) {
  const signature = req.headers['x-sha2-signature'];
  const payload = JSON.stringify(req.body);

  if (!verifyOnfidoWebhook(payload, signature)) {
    return res.status(401).send('Invalid signature');
  }

  const { payload: webhookPayload } = req.body;
  const { action, object } = webhookPayload;

  if (action === 'check.completed' && object.result === 'clear') {
    // Update investor KYC status
    await updateInvestorKYC(object.id, {
      kycVerified: true,
      kycDate: new Date(),
      kycExpiry: new Date(Date.now() + 365 * 24 * 60 * 60 * 1000)
    });

    // Update on-chain whitelist
    await updateBlockchainWhitelist(object.id);
  }

  res.status(200).send('OK');
}
```

### Accreditation Verification (VerifyInvestor)

```typescript
import axios from 'axios';

interface AccreditationRequest {
  investorId: string;
  method: 'NET_WORTH' | 'INCOME' | 'PROFESSIONAL';
  netWorth?: number;
  annualIncome?: number;
  financialDocuments?: string[];
}

async function verifyAccreditation(
  request: AccreditationRequest
): Promise<{ verificationId: string; status: string }> {
  const response = await axios.post(
    'https://api.verifyinvestor.com/v1/verifications',
    {
      investor: {
        id: request.investorId,
        accreditation_method: request.method,
        net_worth: request.netWorth,
        annual_income: request.annualIncome
      },
      documents: request.financialDocuments
    },
    {
      headers: {
        'Authorization': `Bearer ${process.env.VERIFY_INVESTOR_API_KEY}`,
        'Content-Type': 'application/json'
      }
    }
  );

  return {
    verificationId: response.data.id,
    status: response.data.status
  };
}
```

---

## 4. Exchange Integration

### tZERO ATS Integration

#### API Authentication

```typescript
import axios from 'axios';
import jwt from 'jsonwebtoken';

class TZeroClient {
  private baseUrl = 'https://api.tzero.com/v1';
  private apiKey: string;
  private apiSecret: string;

  constructor(apiKey: string, apiSecret: string) {
    this.apiKey = apiKey;
    this.apiSecret = apiSecret;
  }

  private generateJWT(): string {
    return jwt.sign(
      { apiKey: this.apiKey },
      this.apiSecret,
      { expiresIn: '1h' }
    );
  }

  async submitForListing(token: {
    contractAddress: string;
    name: string;
    symbol: string;
    totalSupply: number;
    regulatoryFramework: string;
    auditReport: string;
  }): Promise<{ listingId: string }> {
    const response = await axios.post(
      `${this.baseUrl}/listings`,
      token,
      {
        headers: {
          'Authorization': `Bearer ${this.generateJWT()}`,
          'Content-Type': 'application/json'
        }
      }
    );

    return { listingId: response.data.listingId };
  }

  async placeLimitOrder(order: {
    tokenAddress: string;
    side: 'buy' | 'sell';
    quantity: number;
    limitPrice: number;
    investorWallet: string;
  }): Promise<{ orderId: string }> {
    const response = await axios.post(
      `${this.baseUrl}/orders`,
      order,
      {
        headers: {
          'Authorization': `Bearer ${this.generateJWT()}`,
          'Content-Type': 'application/json'
        }
      }
    );

    return { orderId: response.data.orderId };
  }

  async getOrderBook(tokenAddress: string): Promise<{
    bids: Array<{ price: number; quantity: number }>;
    asks: Array<{ price: number; quantity: number }>;
  }> {
    const response = await axios.get(
      `${this.baseUrl}/orderbook/${tokenAddress}`,
      {
        headers: {
          'Authorization': `Bearer ${this.generateJWT()}`
        }
      }
    );

    return response.data;
  }
}
```

### FIX Protocol Integration (Institutional)

```typescript
import { FIXClient } from 'quickfix';

class InstitutionalTradingBridge {
  private fixClient: FIXClient;

  constructor(config: {
    host: string;
    port: number;
    senderCompID: string;
    targetCompID: string;
  }) {
    this.fixClient = new FIXClient(config);
  }

  async sendNewOrderSingle(order: {
    symbol: string;
    side: 'BUY' | 'SELL';
    quantity: number;
    price: number;
    orderType: 'LIMIT' | 'MARKET';
  }): Promise<string> {
    const message = this.fixClient.createMessage('NewOrderSingle', {
      ClOrdID: generateOrderId(),
      Symbol: order.symbol,
      Side: order.side === 'BUY' ? '1' : '2',
      TransactTime: new Date().toISOString(),
      OrderQty: order.quantity,
      OrdType: order.orderType === 'LIMIT' ? '2' : '1',
      Price: order.price
    });

    await this.fixClient.send(message);
    return message.ClOrdID;
  }
}
```

---

## 5. Wallet Integration

### MetaMask Integration (Web3Modal)

```typescript
import { createWeb3Modal, defaultWagmiConfig } from '@web3modal/wagmi';
import { mainnet, polygon } from 'wagmi/chains';

const projectId = process.env.WALLETCONNECT_PROJECT_ID;

const metadata = {
  name: 'Asset Tokenization Platform',
  description: 'WIA-FIN-008 Compliant Security Tokens',
  url: 'https://your-platform.com',
  icons: ['https://your-platform.com/logo.png']
};

const chains = [mainnet, polygon];
const wagmiConfig = defaultWagmiConfig({ chains, projectId, metadata });

createWeb3Modal({ wagmiConfig, projectId, chains });
```

### WalletConnect v2 Integration

```typescript
import { useAccount, useWriteContract } from 'wagmi';

function TransferTokens() {
  const { address, isConnected } = useAccount();
  const { writeContract, isPending, isSuccess } = useWriteContract();

  async function transferTokens(to: string, amount: string) {
    if (!isConnected) {
      throw new Error('Wallet not connected');
    }

    writeContract({
      address: '0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb',
      abi: assetTokenABI,
      functionName: 'transfer',
      args: [to, BigInt(amount)]
    });
  }

  return (
    <div>
      {isConnected ? (
        <button onClick={() => transferTokens('0x123...', '1000')}>
          {isPending ? 'Transferring...' : 'Send Tokens'}
        </button>
      ) : (
        <w3m-button />
      )}
    </div>
  );
}
```

### Hardware Wallet Support (Ledger)

```typescript
import TransportWebUSB from '@ledgerhq/hw-transport-webusb';
import Eth from '@ledgerhq/hw-app-eth';

async function signTransactionWithLedger(
  transaction: {
    to: string;
    value: string;
    data: string;
  }
): Promise<string> {
  const transport = await TransportWebUSB.create();
  const eth = new Eth(transport);

  const { rawTransaction } = await eth.signTransaction(
    "44'/60'/0'/0/0", // Derivation path
    transaction.to,
    transaction.value,
    transaction.data
  );

  await transport.close();
  return rawTransaction;
}
```

---

## 6. Banking & Fiat On-Ramps

### Circle USDC Integration

```typescript
import { Circle, CircleEnvironments } from '@circle-fin/circle-sdk';

const circle = new Circle(
  process.env.CIRCLE_API_KEY,
  CircleEnvironments.production
);

async function depositFiat(payment: {
  amount: number;
  currency: 'USD' | 'EUR';
  accountNumber: string;
  routingNumber: string;
}): Promise<{ transferId: string }> {
  const transfer = await circle.transfers.createWireTransfer({
    idempotencyKey: crypto.randomUUID(),
    amount: {
      amount: payment.amount.toString(),
      currency: payment.currency
    },
    source: {
      type: 'wire',
      accountNumber: payment.accountNumber,
      routingNumber: payment.routingNumber
    },
    destination: {
      type: 'wallet',
      id: process.env.CIRCLE_WALLET_ID
    }
  });

  return { transferId: transfer.data.id };
}

async function withdrawFiat(withdrawal: {
  amount: number;
  bankAccount: {
    accountNumber: string;
    routingNumber: string;
  };
}): Promise<{ payoutId: string }> {
  const payout = await circle.payouts.create({
    idempotencyKey: crypto.randomUUID(),
    amount: {
      amount: withdrawal.amount.toString(),
      currency: 'USD'
    },
    destination: {
      type: 'wire',
      accountNumber: withdrawal.bankAccount.accountNumber,
      routingNumber: withdrawal.bankAccount.routingNumber
    },
    source: {
      type: 'wallet',
      id: process.env.CIRCLE_WALLET_ID
    }
  });

  return { payoutId: payout.data.id };
}
```

### Wyre Integration

```typescript
import axios from 'axios';

class WyreClient {
  private baseUrl = 'https://api.sendwyre.com/v3';
  private apiKey: string;
  private secretKey: string;

  async createPaymentLink(order: {
    amount: number;
    currency: string;
    destCurrency: string;
    dest: string; // Wallet address
  }): Promise<{ url: string }> {
    const response = await axios.post(
      `${this.baseUrl}/orders/reserve`,
      {
        amount: order.amount,
        sourceCurrency: order.currency,
        destCurrency: order.destCurrency,
        dest: order.dest,
        redirectUrl: 'https://your-platform.com/payment/success',
        failureRedirectUrl: 'https://your-platform.com/payment/failure'
      },
      {
        headers: this.generateAuthHeaders()
      }
    );

    return { url: response.data.url };
  }

  private generateAuthHeaders() {
    const timestamp = Date.now();
    const signature = crypto
      .createHmac('sha256', this.secretKey)
      .update(`${timestamp}`)
      .digest('hex');

    return {
      'X-Api-Key': this.apiKey,
      'X-Api-Signature': signature,
      'X-Api-Timestamp': timestamp.toString()
    };
  }
}
```

---

## 7. Traditional Brokerage Integration

### Interactive Brokers Integration

```typescript
import { IBApi, EventName } from '@stoqey/ib';

class BrokerageIntegration {
  private ib: IBApi;

  constructor() {
    this.ib = new IBApi({
      clientId: 0,
      host: '127.0.0.1',
      port: 7497 // TWS port
    });
  }

  async placeSecurityTokenOrder(order: {
    symbol: string;
    action: 'BUY' | 'SELL';
    quantity: number;
    orderType: 'LMT' | 'MKT';
    limitPrice?: number;
  }): Promise<{ orderId: number }> {
    const contract = {
      symbol: order.symbol,
      secType: 'STK',
      exchange: 'SMART',
      currency: 'USD'
    };

    const orderId = await this.ib.placeOrder(contract, {
      action: order.action,
      totalQuantity: order.quantity,
      orderType: order.orderType,
      lmtPrice: order.limitPrice
    });

    return { orderId };
  }

  onOrderStatus(callback: (status: any) => void) {
    this.ib.on(EventName.orderStatus, callback);
  }
}
```

---

## 8. Data Analytics & Reporting

### Token Analytics

```typescript
interface TokenAnalytics {
  tokenId: string;
  metrics: {
    totalHolders: number;
    top10HoldingPercent: number;
    avgHoldingSize: number;
    dailyActiveAddresses: number;
    transferVolume24h: number;
    priceChange24h: number;
  };
  distribution: {
    range: string; // e.g., "0-100"
    holders: number;
    percentage: number;
  }[];
}

async function getTokenAnalytics(tokenId: string): Promise<TokenAnalytics> {
  const [holders, transfers, price] = await Promise.all([
    db.balances.groupBy('address').where({ tokenId }),
    db.transfers.where({ tokenId, timestamp: { $gte: Date.now() - 86400000 } }),
    getTokenPrice(tokenId)
  ]);

  const totalHolders = holders.length;
  const sortedHolders = holders.sort((a, b) => b.balance - a.balance);
  const top10Total = sortedHolders.slice(0, 10).reduce((sum, h) => sum + h.balance, 0);
  const totalSupply = await getTokenSupply(tokenId);

  return {
    tokenId,
    metrics: {
      totalHolders,
      top10HoldingPercent: (top10Total / totalSupply) * 100,
      avgHoldingSize: totalSupply / totalHolders,
      dailyActiveAddresses: new Set(transfers.flatMap(t => [t.from, t.to])).size,
      transferVolume24h: transfers.reduce((sum, t) => sum + t.amount, 0),
      priceChange24h: price.change24h
    },
    distribution: calculateDistribution(holders)
  };
}
```

---

## 9. Tax Reporting

### IRS Form 1099 Generation

```typescript
interface Form1099DIV {
  formType: '1099-DIV';
  year: number;
  payer: {
    name: string;
    tin: string;
    address: string;
  };
  recipient: {
    name: string;
    ssn: string;
    address: string;
  };
  box1a: number; // Ordinary dividends
  box1b: number; // Qualified dividends
  box2a: number; // Total capital gain distribution
  box3: number; // Nondividend distributions
}

async function generate1099(investorId: string, year: number): Promise<Form1099DIV> {
  const distributions = await db.distributions.where({
    investorId,
    year
  });

  const investor = await db.investors.findById(investorId);

  const ordinaryDividends = distributions
    .filter(d => d.type === 'RENTAL_INCOME')
    .reduce((sum, d) => sum + d.amount, 0);

  const capitalGains = distributions
    .filter(d => d.type === 'CAPITAL_GAIN')
    .reduce((sum, d) => sum + d.amount, 0);

  return {
    formType: '1099-DIV',
    year,
    payer: {
      name: 'Asset Tokenization Platform LLC',
      tin: '12-3456789',
      address: '123 Main St, New York, NY 10001'
    },
    recipient: {
      name: investor.fullName,
      ssn: investor.ssn,
      address: investor.address
    },
    box1a: ordinaryDividends,
    box1b: 0,
    box2a: capitalGains,
    box3: 0
  };
}
```

---

## 10. End-to-End Integration Architecture

### System Architecture Diagram

```
┌─────────────────────────────────────────────────┐
│        WIA-FIN-008 Tokenization Platform        │
│                                                  │
│  ┌──────────────────────────────────────────┐  │
│  │         Token Issuance (Phase 1)         │  │
│  │    Data Format, Metadata, Validation     │  │
│  └──────────────────────────────────────────┘  │
│                      │                          │
│  ┌──────────────────────────────────────────┐  │
│  │           API Gateway (Phase 2)          │  │
│  │   REST/GraphQL, Auth, Rate Limiting      │  │
│  └──────────────────────────────────────────┘  │
│                      │                          │
│  ┌──────────────────────────────────────────┐  │
│  │       Smart Contracts (Phase 3)          │  │
│  │   ERC-1400, Compliance, Distribution     │  │
│  └──────────────────────────────────────────┘  │
└─────────────────────┬───────────────────────────┘
                      │
        ┌─────────────┴─────────────┐
        │                           │
   ┌────▼────┐                 ┌────▼────┐
   │ Custody │                 │   KYC   │
   │Fireblocks                 │  Onfido │
   │ BitGo   │                 │  Jumio  │
   └────┬────┘                 └────┬────┘
        │                           │
        └─────────┬─────────────────┘
                  │
        ┌─────────▼─────────┐
        │    Blockchain     │
        │ Ethereum/Polygon  │
        └─────────┬─────────┘
                  │
        ┌─────────┴─────────┐
        │                   │
   ┌────▼────┐         ┌────▼────┐
   │Exchange │         │ Wallets │
   │  tZERO  │         │MetaMask │
   │   INX   │         │ Ledger  │
   └────┬────┘         └────┬────┘
        │                   │
        └─────────┬─────────┘
                  │
        ┌─────────▼─────────┐
        │   Fiat Gateway    │
        │   Circle (USDC)   │
        │      Wyre         │
        └─────────┬─────────┘
                  │
        ┌─────────▼─────────┐
        │    Reporting      │
        │ Tax (1099), Analytics │
        └───────────────────┘
```

### Health Monitoring

```typescript
interface IntegrationHealth {
  service: string;
  status: 'healthy' | 'degraded' | 'down';
  latency: number; // ms
  lastCheck: Date;
  errorRate: number; // percentage
}

async function checkIntegrationHealth(): Promise<IntegrationHealth[]> {
  return await Promise.all([
    checkFireblocksHealth(),
    checkOnfidoHealth(),
    checkTZeroHealth(),
    checkCircleHealth()
  ]);
}

async function checkFireblocksHealth(): Promise<IntegrationHealth> {
  const start = Date.now();
  try {
    await fireblocks.vaults.list();
    return {
      service: 'Fireblocks',
      status: 'healthy',
      latency: Date.now() - start,
      lastCheck: new Date(),
      errorRate: 0
    };
  } catch (error) {
    return {
      service: 'Fireblocks',
      status: 'down',
      latency: Date.now() - start,
      lastCheck: new Date(),
      errorRate: 100
    };
  }
}
```

---

© 2025 SmileStory Inc. / WIA
弘益人間 · Benefit All Humanity
