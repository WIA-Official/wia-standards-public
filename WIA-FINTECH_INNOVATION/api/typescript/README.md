# WIA-FINTECH_INNOVATION TypeScript SDK

Official TypeScript/JavaScript SDK for WIA Fintech Innovation platform - embedded finance, open banking, and BNPL.

## Installation

```bash
npm install @wia/fintech-innovation
# or
yarn add @wia/fintech-innovation
```

## Quick Start

```typescript
import { FintechClient } from '@wia/fintech-innovation';

const client = new FintechClient({
  apiKey: process.env.WIA_FINTECH_API_KEY,
  baseUrl: 'https://api.wia-fintech.io/v1' // optional
});

// Create a payment
const payment = await client.createPayment({
  amount: 149.99,
  currency: 'USD',
  payment_method: {
    type: 'card',
    token: 'tok_visa_4242'
  },
  customer: {
    email: 'customer@example.com'
  }
});

console.log(`Payment ${payment.id}: ${payment.status}`);
```

## Features

- **Payments**: Accept card, bank account, and digital wallet payments
- **Open Banking**: Access account information and initiate payments (PSD2 compliant)
- **BNPL**: Buy Now Pay Later integration with flexible plans
- **TypeScript Support**: Full type definitions included
- **Promise-based**: Modern async/await API
- **Error Handling**: Comprehensive error types
- **Validation**: Built-in input validation
- **Retry Logic**: Automatic retries with exponential backoff

## Usage

### Payment Processing

```typescript
// Create payment
const payment = await client.createPayment({
  amount: 599.99,
  currency: 'USD',
  payment_method: {
    type: 'card',
    token: 'tok_visa_4242'
  },
  customer: {
    id: 'cus_1234567890',
    email: 'customer@example.com'
  },
  metadata: {
    order_id: 'ORD-2026-001'
  }
});

// Get payment by ID
const retrieved = await client.getPayment('pmt_abc123');

// Refund payment
const refund = await client.refundPayment('pmt_abc123', {
  amount: 599.99,
  reason: 'requested_by_customer'
});
```

### Open Banking

```typescript
// Get bank accounts
const accounts = await client.getOpenBankingAccounts('consent_id_123');

console.log(`Found ${accounts.length} accounts`);
accounts.forEach(account => {
  console.log(`${account.nickname}: ${account.balance?.amount} ${account.currency}`);
});

// Get transactions
const transactions = await client.getOpenBankingTransactions(
  'consent_id_123',
  'acc_9876543210',
  {
    from: '2025-12-01',
    to: '2026-01-11',
    limit: 100
  }
);
```

### Buy Now Pay Later (BNPL)

```typescript
// Check eligibility
const eligibility = await client.getBNPLEligibility({
  customer_id: 'cus_1234567890',
  amount: 599.99,
  currency: 'USD'
});

if (eligibility.eligible) {
  console.log(`Available plans: ${eligibility.available_plans.length}`);
  eligibility.available_plans.forEach(plan => {
    console.log(`${plan.name}: ${plan.installments} × ${plan.installment_amount}`);
  });
}

// Create BNPL order
const order = await client.createBNPLOrder({
  plan_id: 'plan_pay4',
  customer_id: 'cus_1234567890',
  amount: 599.99,
  currency: 'USD',
  merchant: {
    id: 'mch_9876543210',
    order_id: 'ORD-2026-001'
  },
  items: [
    {
      name: 'Wireless Headphones',
      category: 'Electronics',
      quantity: 1,
      unit_price: 599.99
    }
  ]
});

console.log(`BNPL Order ${order.order_id}: ${order.status}`);
console.log(`First payment: ${order.payment_schedule[0].amount} on ${order.payment_schedule[0].due_date}`);
```

### Validation

```typescript
import { validatePayment, validateIBAN, validateEmail } from '@wia/fintech-innovation';

// Validate payment data
const errors = validatePayment({
  amount: 149.99,
  currency: 'USD',
  customer: {
    email: 'customer@example.com'
  }
});

if (errors.length > 0) {
  console.error('Validation errors:', errors);
}

// Validate IBAN
const ibanError = validateIBAN('GB29NWBK60161331926819');
if (ibanError) {
  console.error(`Invalid IBAN: ${ibanError.message}`);
}
```

### Utilities

```typescript
import {
  formatCurrency,
  calculateInstallmentAmount,
  generatePaymentSchedule,
  calculateProcessingFee
} from '@wia/fintech-innovation';

// Format currency
const formatted = formatCurrency(149.99, 'USD');
// Output: "$149.99"

// Calculate BNPL installments
const { installmentAmount, totalWithInterest } = calculateInstallmentAmount(
  599.99,
  4,
  0  // 0% APR
);
console.log(`4 × ${installmentAmount} = ${totalWithInterest}`);

// Generate payment schedule
const schedule = generatePaymentSchedule(
  new Date('2026-01-11'),
  4,
  149.99,
  'biweekly'
);

// Calculate processing fee
const { feeAmount, netAmount } = calculateProcessingFee(149.99, 'card');
console.log(`Fee: ${feeAmount}, Net: ${netAmount}`);
```

## Error Handling

```typescript
import { FintechClient, FintechError } from '@wia/fintech-innovation';

try {
  const payment = await client.createPayment({ ... });
} catch (error) {
  if (error instanceof FintechError) {
    console.error(`Error ${error.code}: ${error.message}`);
    console.error(`Status: ${error.statusCode}`);
    console.error(`Details:`, error.details);
  } else {
    console.error('Unexpected error:', error);
  }
}
```

## Configuration

```typescript
const client = new FintechClient({
  apiKey: 'your_api_key',
  baseUrl: 'https://api.wia-fintech.io/v1',  // optional
  timeout: 30000,  // 30 seconds (optional)
  retries: 3  // number of retries (optional)
});
```

## TypeScript Types

All types are exported:

```typescript
import {
  Payment,
  PaymentStatus,
  PaymentMethod,
  BankAccount,
  Transaction,
  BNPLPlan,
  BNPLOrder,
  OpenBankingConsent
} from '@wia/fintech-innovation';
```

## API Reference

### FintechClient

#### Payment Methods

- `createPayment(payment: Partial<Payment>): Promise<Payment>`
- `getPayment(paymentId: string): Promise<Payment>`
- `refundPayment(paymentId: string, refund: RefundRequest): Promise<Refund>`
- `listPayments(params?: ListPaymentsParams): Promise<Payment[]>`

#### Open Banking Methods

- `getOpenBankingAccounts(consentId: string): Promise<BankAccount[]>`
- `getOpenBankingTransactions(consentId: string, accountId: string, params?: TransactionParams): Promise<Transaction[]>`
- `getOpenBankingConsent(consentId: string): Promise<OpenBankingConsent>`

#### BNPL Methods

- `getBNPLEligibility(params: EligibilityParams): Promise<EligibilityResponse>`
- `getBNPLPlans(amount: number): Promise<BNPLPlan[]>`
- `createBNPLOrder(order: CreateBNPLOrderRequest): Promise<BNPLOrder>`
- `getBNPLOrder(orderId: string): Promise<BNPLOrder>`

## Webhooks

Handle webhook events:

```typescript
import { verifyWebhookSignature } from '@wia/fintech-innovation';

app.post('/webhooks/fintech', (req, res) => {
  const signature = req.headers['wia-signature'];
  const payload = req.body;

  const isValid = verifyWebhookSignature(
    payload,
    signature,
    process.env.WEBHOOK_SECRET
  );

  if (!isValid) {
    return res.status(400).send('Invalid signature');
  }

  // Handle event
  switch (payload.type) {
    case 'payment.succeeded':
      console.log('Payment succeeded:', payload.data.object);
      break;
    case 'bnpl.installment.due':
      console.log('BNPL installment due:', payload.data.object);
      break;
  }

  res.json({ received: true });
});
```

## Development

```bash
# Install dependencies
npm install

# Build
npm run build

# Run tests
npm test

# Lint
npm run lint
```

## Support

- Documentation: https://docs.wia-fintech.io
- GitHub Issues: https://github.com/WIA-Official/fintech-innovation/issues
- Email: support@wia.io

## License

MIT License - see LICENSE file for details

---

**© 2026 WIA | 弘益人間**
