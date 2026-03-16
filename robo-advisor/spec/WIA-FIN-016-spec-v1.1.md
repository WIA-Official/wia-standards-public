# WIA-FIN-016: Robo-Advisor Standard v1.1

**Status:** Stable  
**Version:** 1.1.0  
**Date:** 2025-03-01  
**Changes:** Enhanced tax optimization, goal-based planning, ESG integration

---

## Changelog from v1.0

### New Features
- Goal-based portfolio construction
- ESG/SRI screening capabilities
- Enhanced tax-loss harvesting with direct indexing
- Multi-account tax coordination
- Fractional share support

### API Enhancements
- `/api/v1/goals` - Goal management endpoints
- `/api/v1/portfolios/{id}/esg-score` - ESG metrics
- `/api/v1/tax/harvest-opportunities` - TLH suggestions

---

## 1. Goal-Based Investment

### 1.1 Goal Schema
```json
{
  "goal": {
    "id": "goal_123",
    "userId": "user_abc",
    "type": "enum [retirement, education, home_purchase, wealth_building]",
    "name": "string",
    "targetAmount": "decimal",
    "currentAmount": "decimal",
    "targetDate": "ISO 8601 date",
    "monthlyContribution": "decimal",
    "riskTolerance": "enum [conservative, moderate, aggressive]",
    "priority": "integer (1-10)",
    "linkedPortfolios": ["portfolio_id"]
  }
}
```

### 1.2 Goal-Based Allocation
```
Glide Path Function:
equity_allocation = base_allocation - (years_to_goal × reduction_factor)

For retirement:
- 30+ years: 90% stocks
- 20 years: 80% stocks  
- 10 years: 60% stocks
- 5 years: 40% stocks
- At retirement: 30% stocks
```

---

## 2. ESG Integration

### 2.1 ESG Screening
```json
{
  "esgProfile": {
    "enabled": "boolean",
    "preferences": {
      "excludeSectors": ["fossil_fuels", "tobacco", "weapons"],
      "prioritizeThemes": ["clean_energy", "gender_diversity"],
      "minimumEsgScore": "integer (0-100)"
    },
    "portfolioEsgScore": {
      "overall": "integer (0-100)",
      "environmental": "integer (0-100)",
      "social": "integer (0-100)",
      "governance": "integer (0-100)"
    }
  }
}
```

### 2.2 ESG ETF Mapping
- ESGU - iShares MSCI USA ESG Optimized
- ESGD - iShares MSCI EAFE ESG Optimized  
- ESGE - iShares MSCI EM ESG Optimized
- EAGG - iShares ESG Aware USD Bond

---

## 3. Enhanced Tax Optimization

### 3.1 Direct Indexing
For accounts $100,000+:
- Hold individual stocks instead of ETFs
- Enable stock-level tax-loss harvesting
- Harvest losses on 500-1,000 positions
- Potential additional alpha: 1-2% annually

### 3.2 Tax Coordination
```json
{
  "taxCoordination": {
    "enabled": "boolean",
    "accounts": [
      {
        "portfolioId": "port_123",
        "accountType": "taxable",
        "assetLocationPreference": ["municipal_bonds", "growth_stocks"]
      },
      {
        "portfolioId": "port_456",  
        "accountType": "traditional_ira",
        "assetLocationPreference": ["reits", "bonds", "dividend_stocks"]
      }
    ],
    "strategy": "maximize_after_tax_returns"
  }
}
```

### 3.3 Tax-Loss Harvesting Enhancements
- Daily harvesting (vs. monthly in v1.0)
- Smart lot selection (HIFO, specific ID)
- Automatic replacement security selection
- Wash sale tracking across all accounts

---

## 4. Fractional Shares

### 4.1 Implementation
```json
{
  "holding": {
    "symbol": "VTI",
    "shares": 15.735421,
    "allowFractional": true,
    "sharesType": "fractional"
  }
}
```

### 4.2 Benefits
- Precise asset allocation
- Ability to invest any dollar amount
- Full dividend reinvestment
- Smart deposit allocation

---

## 5. Performance Enhancements

### 5.1 Advanced Metrics
- Risk-adjusted return (Sharpe, Sortino, Calmar)
- Factor attribution (value, size, momentum)
- Tax alpha calculation
- After-tax returns reporting

### 5.2 Benchmarking
```json
{
  "benchmark": {
    "custom": "boolean",
    "composition": [
      {"index": "SPY", "weight": 60},
      {"index": "AGG", "weight": 40}
    ],
    "performance": {
      "ytd": 8.2,
      "oneYear": 11.5
    }
  }
}
```

---

## 6. Additional APIs

### 6.1 Goal Management
```
POST /api/v1/goals
GET /api/v1/goals/{goalId}
PUT /api/v1/goals/{goalId}
DELETE /api/v1/goals/{goalId}
GET /api/v1/goals/{goalId}/projection
```

### 6.2 ESG Endpoints
```
GET /api/v1/portfolios/{id}/esg-score
PUT /api/v1/portfolios/{id}/esg-preferences
GET /api/v1/securities/{symbol}/esg-rating
```

### 6.3 Tax Optimization
```
GET /api/v1/portfolios/{id}/tax/harvest-opportunities
POST /api/v1/portfolios/{id}/tax/harvest-execute
GET /api/v1/tax/tax-loss-report/{year}
```

---

## 7. Backward Compatibility

All v1.0 APIs remain supported with identical behavior. New features are opt-in via API parameters or configuration flags.

---

© 2025 WIA Standards | MIT License  
弘益人間 · Benefit All Humanity
