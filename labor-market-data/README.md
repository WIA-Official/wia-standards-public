# WIA-SOC-020: Labor Market Data Standard 👷

> **Employment Statistics • Wage Data • Job Vacancy Tracking • Skills Taxonomy • Workforce Analytics • Labor Mobility**

**홍익인간 (弘益人間) - Benefit All Humanity**

---

## Overview

The WIA-SOC-020 Labor Market Data Standard provides a comprehensive, globally applicable framework for collecting, formatting, sharing, and analyzing labor market information. This standard addresses critical challenges in employment data interoperability, enabling governments, businesses, educational institutions, and workers to make better-informed decisions.

### Key Features

- 📊 **Standardized Data Formats**: JSON schemas for employment records, wage data, skills, and job postings
- 🔌 **Interoperable APIs**: RESTful, GraphQL, and WebSocket interfaces for seamless integration
- 🔒 **Privacy-First Design**: Built-in privacy protection with differential privacy and encryption
- 🌍 **Global Taxonomies**: Harmonizes NAICS/ISIC, SOC/ISCO, O*NET/ESCO classifications
- ⚡ **Real-Time Capable**: Supports both batch data exchange and real-time streaming
- 🏆 **WIA Certified**: Part of the comprehensive WIA standards ecosystem

---

## Quick Start

### Installation

```bash
# TypeScript/JavaScript
npm install @wia/soc-020-labor-market-data

# Python
pip install wia-soc-020

# Go
go get github.com/wia-official/soc-020-go
```

### Basic Usage

```typescript
import { LaborMarketDataClient } from '@wia/soc-020-labor-market-data';

const client = new LaborMarketDataClient({
  baseUrl: 'https://api.wiastandards.com/labor-market/v1',
  apiKey: 'your-api-key'
});

// Get worker profile
const profile = await client.getWorkerProfile('WKR-2025-001234');

// Search job postings
const jobs = await client.searchJobs({
  location: 'US-CA-SF',
  occupation: 'SOC-15-1252',
  salaryMin: 100000
});

// Get wage statistics
const wages = await client.getWageStatistics({
  occupation: 'SOC-15-1252',
  region: 'US-CA',
  period: '2025-Q4'
});
```

---

## Documentation

### 📚 Interactive Guides

- **[Ebook (English)](ebook/en/index.html)** - Complete technical guide with 8 chapters
- **[Ebook (Korean)](ebook/ko/index.html)** - 한국어 완전 기술 가이드
- **[Simulator](simulator/)** - Interactive labor market data simulator

### 📋 Specifications

- **[Phase 1: Data Format](spec/PHASE-1-DATA-FORMAT.md)** - JSON schemas, field definitions, taxonomies
- **[Phase 2: API Interface](spec/PHASE-2-API.md)** - RESTful APIs, GraphQL, authentication
- **[Phase 3: Protocol](spec/PHASE-3-PROTOCOL.md)** - WebSocket, streaming, security
- **[Phase 4: Integration](spec/PHASE-4-INTEGRATION.md)** - WIA ecosystem connectivity

### 💻 API Reference

- **[TypeScript SDK](api/typescript/)** - Full-featured TypeScript/JavaScript client
- **[OpenAPI Specification](spec/openapi.yaml)** - Complete API documentation
- **[GraphQL Schema](spec/schema.graphql)** - GraphQL type definitions

---

## Architecture

### Four-Phase Approach

```
Phase 1: Data Format ─────┐
                          ├──> Basic Data Exchange
Phase 2: API Interface ───┘

Phase 3: Protocol ────────┐
                          ├──> Full Ecosystem Integration
Phase 4: WIA Integration ─┘
```

### Core Components

1. **Worker Profiles**: Employment history, skills, certifications
2. **Employer Data**: Organization information, industry classification
3. **Job Postings**: Vacancy tracking, requirements, compensation
4. **Wage Statistics**: Compensation data with PPP adjustments
5. **Skills Taxonomy**: 50,000+ standardized skills with proficiency levels
6. **Labor Market Analytics**: Employment rates, trends, forecasts

---

## Data Model Example

```json
{
  "standard": "WIA-SOC-020",
  "version": "1.0",
  "workerProfile": {
    "workerId": "WKR-2025-001234",
    "personalInfo": {
      "name": "Jane Smith",
      "location": {"country": "US", "state": "CA", "city": "San Francisco"}
    },
    "skills": [
      {
        "skillId": "SKL-001234",
        "name": "Python Programming",
        "proficiencyLevel": 4,
        "verification": {
          "method": "certification",
          "source": "Python Institute"
        }
      }
    ],
    "experience": [
      {
        "employer": {"name": "Tech Innovations Inc.", "industry": "NAICS-5415"},
        "position": {"title": "Senior Software Engineer", "occupationCode": "SOC-15-1252"},
        "period": {"startDate": "2022-03-15", "isCurrent": true},
        "compensation": {"baseSalary": 145000, "currency": "USD", "period": "annual"}
      }
    ]
  }
}
```

---

## Integration

### Compatible Standards

WIA-SOC-020 integrates seamlessly with:

- **WIA-EDU**: Educational credentials and outcomes
- **WIA-CREDENTIAL**: Verifiable worker credentials
- **WIA-SOCIAL**: Social services coordination
- **WIA-MIGRATION**: Cross-border labor mobility
- **WIA-AI**: AI-powered workforce analytics

### Supported Platforms

- ✅ Government statistical agencies
- ✅ Job boards and recruitment platforms
- ✅ HR management systems
- ✅ Educational institutions
- ✅ Workforce development programs
- ✅ Economic research organizations

---

## Certification

Organizations can obtain WIA-SOC-020 certification at four levels:

| Level | Requirements | Capabilities |
|-------|--------------|--------------|
| **Basic** | Phase 1 compliance | Data format standardization |
| **Standard** | Phases 1-2 | API integration |
| **Advanced** | Phases 1-3 | Real-time streaming |
| **Premium** | Phases 1-4 | Full ecosystem integration |

**Apply for certification:** https://cert.wiastandards.com

---

## Use Cases

### For Governments
- Evidence-based policy development
- Real-time labor market monitoring
- International data comparability
- Workforce planning and forecasting

### For Employers
- Competitive compensation benchmarking
- Talent market insights
- Skills gap identification
- Diversity and inclusion tracking

### For Workers
- Transparent wage information
- Career path guidance
- Skills development recommendations
- Portable employment credentials

### For Educators
- Curriculum alignment with market demands
- Graduate outcomes tracking
- Skills-based program development

---

## Contributing

We welcome contributions from the community! See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

### Development Setup

```bash
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/labor-market-data

# Install dependencies
npm install

# Run tests
npm test

# Build
npm run build
```

---

## Support

- **Documentation**: https://wiastandards.com/soc-020
- **Community Forum**: https://community.wiastandards.com
- **Issue Tracker**: https://github.com/WIA-Official/wia-standards/issues
- **Email**: support@wiastandards.com

---

## License

MIT License - see [LICENSE](LICENSE) file for details

**홍익인간 (弘益人間) - Benefit All Humanity**

© 2025 WIA - World Certification Industry Association

---

## Acknowledgments

This standard was developed through collaboration with:
- International Labour Organization (ILO)
- National statistical agencies worldwide
- Leading employment platforms
- Workforce development organizations
- Academic researchers
- Worker advocacy groups

Special thanks to all contributors who helped make labor market data more accessible, transparent, and useful for everyone.

---

**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2025-12-26
