# 🚗 WIA-AUTO-025: Mobility-as-a-Service Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-AUTO-025
> **Version:** 1.0.0
> **Status:** Active
> **Category:** AUTO / Mobility
> **Color:** Orange (#F97316)

---

## 🌟 Overview

The WIA-AUTO-025 standard defines a comprehensive framework for Mobility-as-a-Service (MaaS), enabling seamless integration of multiple transportation modes into a unified platform with journey planning, booking, ticketing, and payment services.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard aims to democratize urban mobility by providing accessible, sustainable, and efficient transportation solutions for all people, reducing private vehicle dependency and environmental impact.

## 🎯 Key Features

- **Multimodal Integration**: Unified access to all transport modes (bus, train, bike, car-share, ride-hail)
- **Journey Planning**: Intelligent route optimization across multiple transport modes
- **Unified Payment**: Single payment interface for all mobility services
- **Real-time Information**: Live updates on schedules, delays, and availability
- **Subscription Models**: Flexible pricing with pay-per-use and monthly subscriptions
- **Accessibility Support**: Features for users with disabilities and special needs
- **Carbon Tracking**: Environmental impact monitoring and offsetting

## 📊 Core Concepts

### 1. MaaS Integration Levels

```
Level 0: No Integration
  - Separate services, separate bookings

Level 1: Information Integration
  - Unified journey planning
  - Multiple apps still required

Level 2: Booking & Payment Integration
  - Single app for booking
  - Unified payment interface

Level 3: Service Integration
  - Service packages and subscriptions
  - Bundled mobility credits

Level 4: Societal Integration
  - Policy coordination
  - Urban planning integration
  - Sustainability goals
```

### 2. Multimodal Journey Types

```
First Mile / Last Mile: Home ↔ Transit Station
  - Walking, cycling, e-scooter, ride-share

Core Journey: Long-distance travel
  - Bus, metro, train, BRT

On-demand: Door-to-door service
  - Taxi, ride-hail, car-share

Flexible: Multi-segment trips
  - Combined modes optimized for cost/time/comfort
```

### 3. Pricing Models

- **Pay-per-Use**: Pay only for trips taken
- **Monthly Subscription**: Unlimited or credit-based plans
- **Tiered Plans**: Bronze/Silver/Gold packages
- **Dynamic Pricing**: Demand-based pricing optimization
- **Carbon Credits**: Eco-friendly trip incentives

## 🔧 Components

### TypeScript SDK

```typescript
import {
  MaaSPlatform,
  planJourney,
  bookTrip,
  getMobilityOptions
} from '@wia/auto-025';

// Initialize MaaS platform
const platform = new MaaSPlatform({
  apiKey: 'your-api-key',
  region: 'us-west',
  integrationLevel: 3
});

// Plan a multimodal journey
const journey = await planJourney({
  origin: { lat: 37.7749, lng: -122.4194 }, // San Francisco
  destination: { lat: 37.3861, lng: -122.0839 }, // Mountain View
  departureTime: new Date('2025-01-15T09:00:00'),
  preferences: {
    optimize: 'time', // or 'cost', 'carbon', 'comfort'
    modes: ['bus', 'train', 'bike-share', 'walking'],
    accessibility: {
      wheelchairAccessible: false,
      assistanceRequired: false
    }
  }
});

// Book the journey
const booking = await platform.bookJourney({
  journeyId: journey.id,
  paymentMethod: 'subscription', // or 'credit-card', 'mobile-wallet'
  passengers: 1
});

console.log(booking.status, booking.totalCost);
```

### CLI Tool

```bash
# Plan a journey
wia-auto-025 plan \
  --from "37.7749,-122.4194" \
  --to "37.3861,-122.0839" \
  --time "2025-01-15T09:00:00" \
  --optimize time

# Check available mobility services
wia-auto-025 services --location "37.7749,-122.4194"

# View subscription plans
wia-auto-025 plans --region sf-bay-area

# Book a trip
wia-auto-025 book --journey-id J123456 --payment subscription

# Track carbon footprint
wia-auto-025 carbon --user-id user123 --period month
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-AUTO-025-v1.0.md](./spec/WIA-AUTO-025-v1.0.md) | Complete technical specification |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-auto-025.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/maas

# Run installation script
./install.sh

# Verify installation
wia-auto-025 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/auto-025

# Or yarn
yarn add @wia/auto-025
```

```typescript
import { MaaSPlatform } from '@wia/auto-025';

const platform = new MaaSPlatform({ apiKey: 'your-key' });

// Get real-time mobility options
const options = await platform.getMobilityOptions({
  location: { lat: 37.7749, lng: -122.4194 },
  radius: 1000, // meters
  modes: ['bike-share', 'scooter', 'car-share']
});

console.log(`Found ${options.length} mobility options nearby`);
options.forEach(option => {
  console.log(`${option.mode}: ${option.available} vehicles, ${option.distanceMeters}m away`);
});
```

## 🚌 Transport Modes

| Mode | Description | Typical Use Case |
|------|-------------|------------------|
| Walking | Pedestrian travel | First/last mile (<1km) |
| Cycling | Personal or shared bikes | Short trips (1-5km) |
| E-Scooter | Electric scooters | Urban short trips (1-3km) |
| Bus | Fixed-route public transit | Medium distance (5-20km) |
| Metro/Subway | Urban rail transit | City-wide travel (5-30km) |
| Train | Regional/intercity rail | Long distance (>30km) |
| Tram/Light Rail | Urban rail | City districts (5-15km) |
| Ferry | Water-based transit | Cross-water routes |
| Ride-hail | On-demand car service | Door-to-door (any distance) |
| Car-share | Temporary car rental | Flexible trips (hourly) |
| Taxi | Traditional cab service | On-demand transport |

## 🌍 MaaS Benefits

### For Users
- **Convenience**: One app for all mobility needs
- **Cost Savings**: Optimized transport choices, no car ownership
- **Time Efficiency**: Real-time journey optimization
- **Accessibility**: Inclusive design for all users
- **Sustainability**: Easy access to eco-friendly options

### For Cities
- **Reduced Congestion**: Fewer private vehicles on roads
- **Lower Emissions**: Shift to public and shared transport
- **Better Planning**: Data-driven urban mobility insights
- **Improved Accessibility**: Enhanced mobility for all residents
- **Economic Growth**: New mobility service markets

### For Operators
- **Increased Ridership**: Easy discovery and booking
- **Revenue Growth**: Access to broader customer base
- **Operational Efficiency**: Demand prediction and optimization
- **Integration Benefits**: Network effects with other services
- **Data Insights**: User behavior and demand patterns

## 📱 Data Formats

### Standard Data Formats Supported

| Format | Purpose | Usage |
|--------|---------|-------|
| **NeTEx** | Public transport data | Schedule, routes, stops |
| **GTFS** | General Transit Feed | Real-time updates |
| **SIRI** | Real-time information | Service status, arrivals |
| **MDS** | Mobility Data Spec | Shared mobility vehicles |
| **GBFS** | Bike-share data | Bike availability |
| **TOMP-API** | Transport operator API | Booking, payment |

## 🔐 Privacy & Security

- **Data Minimization**: Collect only necessary user data
- **Anonymization**: Aggregate travel patterns without PII
- **Consent Management**: User control over data sharing
- **Secure Payments**: PCI-DSS compliant transactions
- **Location Privacy**: Opt-in location tracking with controls
- **GDPR Compliance**: Right to access, rectify, delete data

## 🌐 WIA Integration

This standard integrates with:
- **WIA-INTENT**: Intent-based mobility queries ("take me home")
- **WIA-OMNI-API**: Universal API gateway for all transport modes
- **WIA-SOCIAL**: Social mobility features (shared trips, reviews)
- **WIA-PAYMENT**: Unified payment processing
- **WIA-CARBON**: Carbon footprint tracking and offsetting

## 📖 Use Cases

1. **Daily Commute**: Home → Office with optimal mode combination
2. **Airport Transfer**: Multi-leg journey to/from airport
3. **Tourist Exploration**: City-wide sightseeing with day pass
4. **Emergency Travel**: Fast route during service disruptions
5. **Accessible Travel**: Wheelchair-accessible journey planning
6. **Cargo Delivery**: Last-mile delivery integration
7. **Event Transportation**: Mass transit for concerts, sports
8. **Intermodal Tourism**: Long-distance travel combining modes

## 🤝 Contributing

Contributions welcome! Please see our [Contributing Guide](https://github.com/WIA-Official/wia-standards/CONTRIBUTING.md).

## 📄 License

MIT License - see [LICENSE](https://github.com/WIA-Official/wia-standards/LICENSE)

## 🔗 Links

- **Website**: [wiastandards.com](https://wiastandards.com)
- **GitHub**: [github.com/WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)
- **Documentation**: [docs.wiastandards.com](https://docs.wiastandards.com)
- **MaaS Alliance**: [maas-alliance.eu](https://maas-alliance.eu)
- **Certification**: [cert.wiastandards.com](https://cert.wiastandards.com)

---

**홍익인간 (弘益人間) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
