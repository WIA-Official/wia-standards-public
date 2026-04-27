# 🍴 WIA-IND-010: Restaurant Tech Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-IND-010
> **Version:** 1.0.0
> **Status:** Active
> **Category:** IND / Industry
> **Color:** Indigo (#6366F1)

---

## 🌟 Overview

The WIA-IND-010 standard defines the comprehensive framework for restaurant technology systems, including Point of Sale (POS) systems, reservation management, kitchen display systems (KDS), staff scheduling, inventory management, and customer analytics. This standard provides a unified interface for restaurant operations, enabling seamless integration between different systems and platforms.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard aims to improve restaurant operations worldwide, making dining experiences better for customers, operations more efficient for staff, and business management more effective for owners, ultimately contributing to the thriving food service industry that nourishes communities.

## 🎯 Key Features

- **Point of Sale (POS) Systems**: Order processing, payment handling, receipt generation
- **Reservation Management**: Table booking, waitlist management, customer preferences
- **Kitchen Display Systems (KDS)**: Order routing, preparation tracking, timing optimization
- **Staff Scheduling**: Shift management, labor cost optimization, attendance tracking
- **Inventory Management**: Stock tracking, automatic reordering, waste reduction
- **Customer Analytics**: Dining patterns, preferences, loyalty programs
- **Menu Engineering**: Item profitability, popularity analysis, pricing optimization
- **Multi-location Support**: Chain restaurant management, centralized reporting

## 📊 Core Concepts

### 1. Table Turnover Rate

```
Turnover Rate = Number of Parties Served / Number of Tables / Time Period (hours)
```

Where:
- `Number of Parties` = Total guests served during period
- `Number of Tables` = Available seating capacity
- `Time Period` = Operating hours

### 2. Revenue Per Available Seat Hour (RevPASH)

```
RevPASH = Total Revenue / (Number of Seats × Operating Hours)
```

### 3. Food Cost Percentage

```
Food Cost % = (Cost of Goods Sold / Food Sales Revenue) × 100
```

### 4. Labor Cost Percentage

```
Labor Cost % = (Total Labor Costs / Total Revenue) × 100
```

Optimal range: 25-35% for most restaurants

### 5. Average Check Size

```
Average Check = Total Revenue / Number of Covers
```

### 6. Kitchen Ticket Time

```
Average Ticket Time = Sum of All Ticket Times / Number of Tickets
```

Target: 12-20 minutes for casual dining

## 🔧 Components

### TypeScript SDK

```typescript
import {
  POSSystem,
  ReservationManager,
  KitchenDisplaySystem,
  StaffScheduler,
  InventoryManager,
  CustomerAnalytics
} from '@wia/ind-010';

// Initialize POS system
const pos = new POSSystem({
  restaurantId: 'rest_001',
  location: 'main_dining',
  timezone: 'America/New_York'
});

// Process order
const order = await pos.createOrder({
  tableNumber: 12,
  serverId: 'server_042',
  items: [
    { menuItemId: 'item_101', quantity: 2, modifiers: ['no_onions'] },
    { menuItemId: 'item_205', quantity: 1, modifiers: ['extra_cheese'] }
  ],
  orderType: 'dine_in'
});

// Calculate order totals
const totals = pos.calculateOrderTotal({
  subtotal: 45.50,
  taxRate: 0.08,
  tipPercent: 18,
  discounts: [{ type: 'happy_hour', amount: 5.00 }]
});

console.log(`Total: $${totals.total.toFixed(2)}`);
console.log(`Tax: $${totals.tax.toFixed(2)}`);
console.log(`Tip: $${totals.tip.toFixed(2)}`);

// Manage reservations
const reservation = new ReservationManager({
  restaurantId: 'rest_001'
});

const booking = await reservation.createReservation({
  customerName: 'Jane Smith',
  partySize: 4,
  dateTime: new Date('2025-12-31T19:00:00'),
  specialRequests: 'Window seat preferred, birthday celebration',
  contactEmail: 'jane@example.com',
  contactPhone: '+1-555-0123'
});

// Kitchen display system
const kds = new KitchenDisplaySystem({
  stationId: 'grill_station_1',
  autoRouting: true
});

// Send order to kitchen
await kds.routeOrder({
  orderId: order.id,
  items: order.items.filter(item => item.station === 'grill'),
  priority: 'normal',
  specialInstructions: 'VIP table'
});

// Track preparation time
const ticketTime = kds.getAverageTicketTime({
  station: 'grill_station_1',
  period: 'last_hour'
});

console.log(`Average ticket time: ${ticketTime.toFixed(1)} minutes`);

// Staff scheduling
const scheduler = new StaffScheduler({
  restaurantId: 'rest_001'
});

const schedule = scheduler.optimizeSchedule({
  date: new Date('2025-12-31'),
  expectedCovers: 180,
  laborCostTarget: 0.28, // 28% of projected revenue
  skillRequirements: {
    servers: 6,
    cooks: 3,
    hosts: 2,
    bartenders: 2
  }
});

// Customer analytics
const analytics = new CustomerAnalytics({
  restaurantId: 'rest_001'
});

const insights = await analytics.getCustomerInsights({
  customerId: 'cust_12345',
  timeRange: 'last_year'
});

console.log(`Lifetime value: $${insights.lifetimeValue.toFixed(2)}`);
console.log(`Visit frequency: ${insights.visitFrequency} visits/month`);
console.log(`Favorite items: ${insights.favoriteItems.join(', ')}`);
```

### CLI Tool

```bash
# Process a dine-in order
wia-ind-010 pos order --table 12 --items "burger,fries,soda" --server 042

# Calculate order total with tax and tip
wia-ind-010 pos total --subtotal 45.50 --tax 8 --tip 18

# Create reservation
wia-ind-010 reservation create --name "Jane Smith" --party 4 --date "2025-12-31 19:00"

# Check table availability
wia-ind-010 reservation check --date "2025-12-31 19:00" --party 4

# Track kitchen ticket time
wia-ind-010 kds ticket-time --station grill --period last_hour

# Optimize staff schedule
wia-ind-010 schedule optimize --date "2025-12-31" --covers 180 --labor-target 28

# Calculate table turnover
wia-ind-010 analytics turnover --tables 25 --parties 85 --hours 8

# Calculate RevPASH
wia-ind-010 analytics revpash --revenue 12500 --seats 100 --hours 8

# Food cost analysis
wia-ind-010 analytics food-cost --cogs 3500 --revenue 10000

# Menu item profitability
wia-ind-010 menu analyze --item "signature_burger" --cost 4.50 --price 15.99

# Inventory check
wia-ind-010 inventory check --item "ground_beef" --par-level 50

# Generate sales report
wia-ind-010 report sales --date "2025-12-26" --format json
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-IND-010-v1.0.md](./spec/WIA-IND-010-v1.0.md) | Complete specification with technical details |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-ind-010.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/restaurant-tech

# Run installation script
./install.sh

# Verify installation
wia-ind-010 version
```

### TypeScript SDK Usage

```bash
# Install via npm
npm install @wia/ind-010

# Or install locally
cd api/typescript
npm install
npm run build
npm link
```

### Basic Examples

```typescript
// Quick POS calculation
import { calculateOrderTotal } from '@wia/ind-010';

const total = calculateOrderTotal({
  subtotal: 45.50,
  taxRate: 0.08,
  tipPercent: 18
});

console.log(`Total: $${total.total.toFixed(2)}`); // $55.55

// Quick table turnover calculation
import { calculateTurnoverRate } from '@wia/ind-010';

const turnover = calculateTurnoverRate({
  partiesServed: 85,
  numberOfTables: 25,
  hoursOpen: 8
});

console.log(`Turnover: ${turnover.toFixed(2)} parties/table/hour`); // 0.43
```

## 🏢 Use Cases

### Fast Casual Restaurant
- Quick order processing with POS
- Minimal table management
- High volume, fast turnover
- Simple menu engineering

### Fine Dining Restaurant
- Detailed reservation management
- Course timing coordination via KDS
- Premium customer analytics
- Complex menu with wine pairing

### Restaurant Chain
- Multi-location inventory sync
- Centralized staff scheduling
- Cross-location analytics
- Standardized menu and pricing

### Cloud Kitchen / Ghost Kitchen
- Order aggregation from multiple platforms
- Kitchen efficiency optimization
- Delivery coordination
- Virtual brand management

## 📈 Performance Metrics

### Operational Metrics
- Table turnover rate
- Average ticket time
- Order accuracy rate
- Peak hour capacity utilization

### Financial Metrics
- Revenue per available seat hour (RevPASH)
- Food cost percentage
- Labor cost percentage
- Average check size

### Customer Metrics
- Customer satisfaction score
- Repeat visit rate
- Customer lifetime value
- Online review ratings

## 🔐 Security & Compliance

- **PCI DSS Compliance**: Secure payment processing
- **GDPR Compliance**: Customer data protection
- **Health Code Integration**: Food safety tracking
- **Labor Law Compliance**: Scheduling regulations

## 🌐 Integration Support

- Payment processors (Stripe, Square, etc.)
- Reservation platforms (OpenTable, Resy)
- Delivery services (Uber Eats, DoorDash, Grubhub)
- Accounting systems (QuickBooks, Xero)
- Loyalty programs
- Review platforms (Yelp, Google)

## 📞 Support

- Documentation: [https://wia.dev/standards/ind-010](https://wia.dev/standards/ind-010)
- GitHub Issues: [WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards/issues)
- Email: standards@wia.dev

---

**홍익인간 (弘益人間) - Benefit All Humanity**

© 2025 SmileStory Inc. / WIA - MIT License

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
