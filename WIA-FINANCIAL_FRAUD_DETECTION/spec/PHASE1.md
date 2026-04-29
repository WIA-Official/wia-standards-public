# WIA-FINANCIAL_FRAUD_DETECTION - PHASE 1: Foundation & Architecture
**Version**: 1.0
**Status**: Production Ready
**Last Updated**: 2026-01-11

---

## 📋 Table of Contents

1. [Executive Summary](#executive-summary)
2. [Introduction](#introduction)
3. [Scope & Objectives](#scope--objectives)
4. [System Architecture](#system-architecture)
5. [Core Components](#core-components)
6. [Technology Stack](#technology-stack)
7. [Compliance Framework](#compliance-framework)
8. [References](#references)

---

## 1. Executive Summary

### 1.1 Overview

WIA-FINANCIAL_FRAUD_DETECTION is a comprehensive, real-time fraud detection standard designed to protect financial institutions and payment processors from increasingly sophisticated fraud attacks. With global fraud losses exceeding $32 billion annually in 2025, this standard provides a unified framework for implementing AI-powered fraud detection systems that balance security with user experience.

### 1.2 Key Capabilities

- **Real-Time Detection**: Sub-100ms fraud scoring for payment transactions
- **AI/ML Models**: Hybrid XGBoost + Random Forest achieving F1-score of 0.947 and AUC of 0.994
- **Multi-Vector Analysis**: Combines behavioral, transactional, device, and network signals
- **Privacy-First**: Federated learning with differential privacy (ε-DP) compliance
- **Global Compliance**: Meets PCI DSS v4.0.1, ISO/IEC 27001, GDPR, and regional regulations

### 1.3 Target Use Cases

| Use Case | Description | Priority |
|----------|-------------|----------|
| **Card-Not-Present (CNP) Fraud** | E-commerce, mobile payments, subscription services | Critical |
| **Account Takeover (ATO)** | Unauthorized access to banking/payment accounts | Critical |
| **Money Laundering (AML)** | Transaction pattern analysis for suspicious activity | High |
| **Synthetic Identity Fraud** | Detection of fabricated identities using real+fake data | High |
| **Card Testing/Carding** | Rapid low-value transactions to validate stolen cards | Medium |
| **Friendly Fraud/Chargeback** | Legitimate customer disputes requiring investigation | Medium |

### 1.4 Business Value

- **Fraud Loss Reduction**: 75-85% reduction in fraud-related losses
- **False Positive Rate**: <0.5% (industry average: 2-3%)
- **Customer Experience**: 99.7% of legitimate transactions approved automatically
- **Operational Efficiency**: 90% reduction in manual review workload
- **Compliance Cost Savings**: Automated PCI DSS and regulatory reporting

---

## 2. Introduction

### 2.1 Background

Financial fraud has evolved dramatically over the past decade, driven by:

- **Digital Transformation**: Explosive growth in online/mobile payments
- **Sophisticated Attacks**: AI-powered fraud bots, deep fakes, social engineering
- **Cross-Border Crime**: Organized fraud rings operating globally
- **Data Breaches**: Billions of compromised credentials available on dark web
- **Regulatory Pressure**: Stricter compliance requirements (PSD2, Strong Customer Authentication)

Traditional rule-based fraud detection systems struggle with:
- High false positive rates (2-5%)
- Inability to detect novel fraud patterns
- Slow adaptation to emerging threats
- Limited scalability for real-time processing

### 2.2 The WIA Approach

WIA-FINANCIAL_FRAUD_DETECTION addresses these challenges through:

1. **Hybrid AI Architecture**: Combines supervised (XGBoost, Random Forest) and unsupervised (Isolation Forest, Autoencoder) models
2. **Real-Time Stream Processing**: Apache Kafka + Flink for sub-second decision-making
3. **Federated Learning**: Privacy-preserving collaborative model training across institutions
4. **Explainable AI**: SHAP values and LIME for transparent fraud reasoning
5. **Adaptive Learning**: Continuous model retraining with concept drift detection

### 2.3 Standards Alignment

This specification aligns with:

- **PCI DSS v4.0.1**: Payment Card Industry Data Security Standard (March 2025)
- **ISO/IEC 27001:2022**: Information Security Management
- **ISO 22301:2019**: Business Continuity Management
- **NIST SP 800-53 Rev 5**: Security and Privacy Controls
- **GDPR**: EU General Data Protection Regulation
- **CCPA/CPRA**: California Consumer Privacy Act
- **PSD2**: EU Payment Services Directive (Strong Customer Authentication)
- **FATF**: Financial Action Task Force recommendations for AML/CFT

---

## 3. Scope & Objectives

### 3.1 In Scope

#### 3.1.1 Transaction Types
- Credit/Debit card payments (CNP and Card-Present)
- ACH/Bank transfers
- Wire transfers (domestic and international)
- Mobile wallet transactions (Apple Pay, Google Pay, etc.)
- Cryptocurrency payments and exchanges
- P2P payments (Venmo, Zelle, PayPal, etc.)
- Buy Now Pay Later (BNPL) transactions

#### 3.1.2 Fraud Categories
- **Transaction Fraud**: Unauthorized payments using stolen credentials
- **Identity Fraud**: Account opening with synthetic or stolen identities
- **Account Takeover**: Credential stuffing, phishing, SIM swapping
- **Refund Fraud**: Exploiting return policies for financial gain
- **Merchant Fraud**: Fake merchants, chargeback schemes
- **First-Party Fraud**: Legitimate customers committing fraud
- **Money Laundering**: Structuring, layering, integration detection

#### 3.1.3 Detection Phases
1. **Pre-Authorization**: Risk scoring before payment approval
2. **In-Transaction**: Real-time behavioral biometrics during checkout
3. **Post-Authorization**: Batch analysis for delayed fraud patterns
4. **Dispute Resolution**: Chargeback prediction and evidence gathering

### 3.2 Out of Scope

- Physical card skimming detection (addressed in WIA-PHYSICAL_SECURITY)
- ATM cash-out fraud (separate physical security domain)
- Insider threat detection (covered in WIA-INSIDER_THREAT)
- Tax fraud and insurance fraud (domain-specific standards)

### 3.3 Objectives

#### 3.3.1 Performance Targets

| Metric | Target | Measurement |
|--------|--------|-------------|
| **Detection Rate (Recall)** | ≥95% | True Positives / (TP + False Negatives) |
| **Precision** | ≥90% | True Positives / (TP + False Positives) |
| **F1-Score** | ≥0.92 | Harmonic mean of Precision and Recall |
| **AUC-ROC** | ≥0.98 | Area Under Receiver Operating Characteristic |
| **False Positive Rate** | ≤0.5% | False Positives / Total Legitimate Transactions |
| **Latency (p95)** | ≤100ms | 95th percentile response time |
| **Throughput** | ≥10,000 TPS | Transactions per second per node |
| **Uptime** | 99.99% | Four nines availability (52 min downtime/year) |

#### 3.3.2 Compliance Objectives
- Achieve PCI DSS v4.0.1 compliance for all payment data handling
- Maintain ISO 27001 certification for information security
- GDPR Article 22 compliance for automated decision-making
- Fair Lending Act compliance (avoid discriminatory bias in models)

#### 3.3.3 Business Objectives
- Reduce fraud losses by 80% within 12 months of implementation
- Decrease manual review queues by 85%
- Improve customer satisfaction scores (reduce friction for legitimate users)
- Enable real-time fraud insights for business intelligence

---

## 4. System Architecture

### 4.1 High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    CLIENT APPLICATIONS                          │
│  (Mobile Apps, Web, POS, API Integrations)                     │
└────────────────────────┬───────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│                    API GATEWAY LAYER                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐         │
│  │ Rate Limiter │  │ Auth/AuthZ   │  │ TLS Offload  │         │
│  └──────────────┘  └──────────────┘  └──────────────┘         │
└────────────────────────┬───────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│               FRAUD DETECTION CORE ENGINE                       │
│  ┌──────────────────────────────────────────────────┐          │
│  │  Real-Time Stream Processing (Kafka + Flink)     │          │
│  └────────────┬─────────────────────────────────────┘          │
│               │                                                  │
│       ┌───────┴────────┬──────────────┬──────────────┐         │
│       ▼                ▼              ▼              ▼         │
│  ┌─────────┐   ┌──────────────┐  ┌─────────┐  ┌─────────┐    │
│  │ Feature │   │   ML Models  │  │  Rules  │  │ Scoring │    │
│  │Engineer │   │  Ensemble    │  │ Engine  │  │ Fusion  │    │
│  └─────────┘   └──────────────┘  └─────────┘  └─────────┘    │
│       │             │                  │            │          │
│       │        ┌────┴────┬─────────────┘            │          │
│       │        ▼         ▼                          │          │
│       │   ┌────────┐ ┌────────┐                    │          │
│       │   │XGBoost │ │Random  │                    │          │
│       │   │        │ │Forest  │                    │          │
│       │   └────────┘ └────────┘                    │          │
│       │        │         │                          │          │
│       │   ┌────┴────┬────┴──────┐                  │          │
│       │   ▼         ▼           ▼                  │          │
│       │ ┌──────┐ ┌──────┐ ┌──────────┐            │          │
│       │ │Isolat│ │Auto  │ │Deep      │            │          │
│       │ │Forest│ │encode│ │Learning  │            │          │
│       │ └──────┘ └──────┘ └──────────┘            │          │
│       └──────────────────────────────┬─────────────┘          │
│                                      ▼                         │
│                            ┌──────────────────┐                │
│                            │  Decision Engine │                │
│                            │  (Approve/Review │                │
│                            │   /Block/Alert)  │                │
│                            └──────────────────┘                │
└────────────────────────┬───────────────────────────────────────┘
                         │
        ┌────────────────┼────────────────┐
        ▼                ▼                ▼
┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│  Case Mgmt   │  │  Analytics   │  │  Feedback    │
│  (Manual     │  │  Dashboard   │  │  Loop (Model │
│   Review)    │  │  (Kibana)    │  │  Retraining) │
└──────────────┘  └──────────────┘  └──────────────┘
```

### 4.2 Architecture Principles

#### 4.2.1 Cloud-Native Design
- **Microservices**: Independently deployable components
- **Containerization**: Docker + Kubernetes for orchestration
- **Auto-Scaling**: Horizontal scaling based on transaction volume
- **Multi-Region**: Active-active deployment for global coverage

#### 4.2.2 Data Architecture
- **Event-Driven**: Kafka topics for transaction events
- **Data Lake**: Historical data storage (S3/GCS) for model training
- **Feature Store**: Centralized feature management (Feast/Tecton)
- **Time-Series DB**: InfluxDB for real-time metrics

#### 4.2.3 Security Architecture
- **Zero Trust**: Never trust, always verify
- **Encryption**: Data at rest (AES-256), in transit (TLS 1.3)
- **Secrets Management**: HashiCorp Vault for credentials
- **Network Segmentation**: VPC isolation for sensitive components

### 4.3 Deployment Models

#### 4.3.1 Cloud (Recommended)
- **AWS**: EKS, Kinesis, SageMaker, DynamoDB
- **GCP**: GKE, Pub/Sub, Vertex AI, BigQuery
- **Azure**: AKS, Event Hubs, Azure ML, Cosmos DB

#### 4.3.2 On-Premises
- Kubernetes cluster with bare-metal or VM infrastructure
- Apache Kafka for streaming
- Self-hosted ML infrastructure (Kubeflow, MLflow)

#### 4.3.3 Hybrid
- Cloud-based ML model training and analytics
- On-premises real-time inference for latency/compliance
- Secure VPN/Direct Connect for data synchronization

---

## 5. Core Components

### 5.1 Transaction Collector

**Purpose**: Ingest transaction data from multiple sources in real-time

**Key Features**:
- Multi-protocol support: REST API, gRPC, GraphQL, message queues
- Schema validation and transformation
- Deduplication (idempotency keys)
- Guaranteed delivery (at-least-once semantics)

**Data Sources**:
- Payment gateways (Stripe, Adyen, Authorize.Net)
- Core banking systems
- E-commerce platforms (Shopify, WooCommerce)
- Mobile SDKs (iOS, Android)

**Output**: Standardized transaction events to Kafka topic

### 5.2 Feature Engineering Module

**Purpose**: Transform raw transaction data into ML-ready features

**Feature Categories**:

1. **Transaction Features** (15 features)
   - Amount, currency, merchant category code (MCC)
   - Time of day, day of week, timezone
   - Velocity: transactions per hour/day/week

2. **User Behavioral Features** (25 features)
   - Historical spending patterns (avg, median, std dev)
   - Geographic distribution (home country, travel patterns)
   - Merchant affinity (repeat merchants, new merchants)
   - Time-since-last-transaction

3. **Device/Network Features** (20 features)
   - Device fingerprint (browser, OS, screen resolution)
   - IP geolocation and reputation score
   - VPN/Proxy detection
   - Device velocity (unique devices per user)

4. **Graph Features** (10 features)
   - Social network analysis (connected users, shared addresses)
   - Entity resolution (same device, IP, email across accounts)

5. **External Data Features** (10 features)
   - Email domain reputation (disposable email detection)
   - Phone number type (landline, mobile, VoIP)
   - Address verification (USPS, postal services)
   - Credit bureau data (where permitted)

**Total**: 80+ engineered features

**Technologies**:
- Python: Pandas, NumPy, Scikit-learn
- Distributed: Spark Structured Streaming
- Feature Store: Feast (offline + online stores)

### 5.3 ML Model Ensemble

**Purpose**: Multi-model fraud scoring with ensemble learning

#### 5.3.1 Supervised Models

**A. XGBoost Classifier**
- Primary model for fraud classification
- Handles class imbalance with scale_pos_weight
- Feature importance via SHAP values
- Hyperparameters: max_depth=6, learning_rate=0.1, n_estimators=200

**B. Random Forest Classifier**
- Secondary model for robustness
- Resistant to overfitting with bagging
- Provides confidence intervals
- Hyperparameters: n_estimators=100, max_features='sqrt'

**C. Deep Neural Network (TensorFlow)**
- Captures complex non-linear patterns
- Architecture: 80 → 128 → 64 → 32 → 1 (sigmoid output)
- Dropout (0.3) and L2 regularization
- Trained with focal loss for class imbalance

#### 5.3.2 Unsupervised Models

**A. Isolation Forest**
- Anomaly detection for novel fraud patterns
- Contamination parameter: 0.01 (1% expected fraud rate)
- Useful for zero-day fraud detection

**B. Autoencoder**
- Reconstruction error for outlier detection
- Architecture: 80 → 40 → 20 → 40 → 80
- Threshold: 95th percentile of reconstruction error

#### 5.3.3 Ensemble Strategy

**Weighted Voting**:
```
Fraud_Score = 0.40 * XGBoost
            + 0.30 * Random_Forest
            + 0.15 * DNN
            + 0.10 * Isolation_Forest
            + 0.05 * Autoencoder
```

**Decision Thresholds**:
- Score ≥ 0.90: **Block** (high confidence fraud)
- 0.70 ≤ Score < 0.90: **Manual Review** (suspicious)
- 0.50 ≤ Score < 0.70: **Challenge** (step-up authentication)
- Score < 0.50: **Approve** (legitimate)

### 5.4 Rules Engine

**Purpose**: Deterministic fraud checks and business logic

**Rule Categories**:

1. **Blacklist/Whitelist Rules**
   - Blocked countries, IP ranges, email domains
   - Whitelisted trusted users (low-risk segments)

2. **Velocity Rules**
   - Max 3 transactions per minute per card
   - Max $5,000 total per day for new accounts
   - Max 5 failed authorization attempts per hour

3. **Geolocation Rules**
   - Impossible travel (card used in two distant locations within short time)
   - High-risk countries (based on fraud rate history)

4. **Behavioral Rules**
   - First transaction ≥ $500: trigger review
   - Shipping address ≠ billing address: increase risk score
   - Email domain recently created (< 30 days): suspicious

**Rule Engine Technology**: Drools (open-source business rules engine)

### 5.5 Decision Engine

**Purpose**: Orchestrate model scores, rules, and business logic into final decision

**Decision Logic**:
```
IF (any_blocking_rule_triggered):
    RETURN Block(reason="Rule violation", rule_id)
ELIF (fraud_score >= 0.90):
    RETURN Block(reason="High fraud probability", score)
ELIF (fraud_score >= 0.70):
    RETURN Review(priority="high", score)
ELIF (fraud_score >= 0.50):
    RETURN Challenge(method="3DS2", score)
ELSE:
    RETURN Approve(score)
```

**Explainability**:
- Top 5 contributing features (SHAP values)
- Triggered rule IDs
- Historical user risk profile
- Similar fraud cases (case-based reasoning)

### 5.6 Feedback Loop & Continuous Learning

**Purpose**: Adapt to evolving fraud patterns through continuous model updates

**Feedback Sources**:
1. Manual review decisions (confirmed fraud, false positives)
2. Chargebacks (30-60 day delay)
3. External fraud reports (consortium data)
4. Customer disputes

**Retraining Cadence**:
- **Online Learning**: Incremental updates hourly
- **Batch Retraining**: Full model refresh weekly
- **A/B Testing**: Shadow mode for 48 hours before promotion

**Concept Drift Detection**:
- Monitor model performance metrics (precision, recall) over time
- Trigger retraining if F1-score drops by >5%
- Detect distribution shifts in feature statistics

---

## 6. Technology Stack

### 6.1 Programming Languages
- **Python 3.11+**: ML models, feature engineering, APIs
- **Java 17**: Kafka consumers, rules engine (Drools)
- **TypeScript/Node.js**: API Gateway, SDK
- **Go**: High-performance services (scoring, routing)

### 6.2 Machine Learning
- **Frameworks**: XGBoost, Scikit-learn, TensorFlow 2.x, PyTorch
- **Feature Store**: Feast, Tecton
- **Model Serving**: TensorFlow Serving, TorchServe, Seldon Core
- **Experiment Tracking**: MLflow, Weights & Biases

### 6.3 Data Infrastructure
- **Stream Processing**: Apache Kafka, Apache Flink, Kafka Streams
- **Batch Processing**: Apache Spark (PySpark)
- **Data Lake**: AWS S3, Google Cloud Storage, Azure Blob
- **Databases**:
  - **Transactional**: PostgreSQL 15, Amazon Aurora
  - **NoSQL**: DynamoDB, MongoDB, Cassandra
  - **Time-Series**: InfluxDB, TimescaleDB
  - **Graph**: Neo4j (for network analysis)

### 6.4 Deployment & Orchestration
- **Containers**: Docker 24+
- **Orchestration**: Kubernetes 1.28+
- **Service Mesh**: Istio, Linkerd
- **CI/CD**: GitHub Actions, GitLab CI, Jenkins

### 6.5 Monitoring & Observability
- **Metrics**: Prometheus, Grafana
- **Logging**: ELK Stack (Elasticsearch, Logstash, Kibana), Loki
- **Tracing**: Jaeger, OpenTelemetry
- **APM**: Datadog, New Relic

### 6.6 Security
- **Secrets Management**: HashiCorp Vault, AWS Secrets Manager
- **Identity**: OAuth 2.0, OpenID Connect, SAML 2.0
- **Encryption**: OpenSSL, AWS KMS, Google Cloud KMS
- **WAF**: Cloudflare, AWS WAF, ModSecurity

---

## 7. Compliance Framework

### 7.1 PCI DSS v4.0.1 Compliance

**Key Requirements**:

| Requirement | Implementation |
|-------------|----------------|
| **Req 1-2**: Firewall & Defaults | Network segmentation, VPC isolation, no default passwords |
| **Req 3**: Protect Stored Data | AES-256 encryption, tokenization of PAN (Primary Account Number) |
| **Req 4**: Encrypt Transmission | TLS 1.3 for all API communications |
| **Req 5**: Anti-Malware | Container scanning (Aqua, Twistlock), SAST/DAST |
| **Req 6**: Secure Systems | Patch management, vulnerability scanning (Nessus, Qualys) |
| **Req 7**: Restrict Access | RBAC, principle of least privilege, MFA |
| **Req 8**: Identify Users | SSO, audit logging, password policies |
| **Req 9**: Physical Access | Data center security (out of scope for cloud) |
| **Req 10**: Log & Monitor | Centralized logging, 90-day retention, SIEM integration |
| **Req 11**: Test Security | Quarterly ASV scans, annual penetration testing |
| **Req 12**: Security Policy | ISMS documentation, security awareness training |

**New in v4.0.1** (March 2025):
- Requirement 6.4.3: Web application firewall for internet-facing apps
- Requirement 11.6.1: Automated change detection for critical files
- Requirement 12.10: Incident response plan testing

### 7.2 ISO/IEC 27001:2022 Alignment

**Annex A Controls**:
- **A.8**: Asset Management - inventory of data, systems, and models
- **A.9**: Access Control - authentication, authorization, privileged access
- **A.12**: Operations Security - malware protection, backup, logging
- **A.14**: System Acquisition - secure SDLC for ML models
- **A.16**: Incident Management - fraud incident response procedures
- **A.17**: Business Continuity - disaster recovery, failover strategies

### 7.3 GDPR Compliance

**Key Considerations**:
- **Article 6**: Lawful basis for processing (legitimate interest in fraud prevention)
- **Article 22**: Right to object to automated decision-making
  - Provide human review option for declined transactions
  - Explain fraud decision in plain language
- **Article 25**: Privacy by design
  - Data minimization (only collect necessary features)
  - Pseudonymization/anonymization where possible
- **Article 32**: Security of processing (encryption, access controls)
- **Article 33**: Breach notification (72-hour reporting)

**Differential Privacy**:
- Implement ε-differential privacy (ε=1.0) for federated learning
- Noise injection during model training to protect individual records

### 7.4 AML/KYC Requirements

**FATF Recommendations**:
- **Recommendation 10**: Customer Due Diligence (CDD)
- **Recommendation 16**: Wire transfer information requirements
- **Recommendation 20**: Suspicious Transaction Reports (STRs)

**Transaction Monitoring**:
- Structuring detection (deposits just under $10,000 reporting threshold)
- Layering detection (complex chains of transactions)
- Geographic anomalies (transfers to high-risk jurisdictions)

### 7.5 Fairness & Bias Mitigation

**Fair Lending Act Compliance**:
- Ensure ML models don't discriminate based on protected characteristics:
  - Race, ethnicity, national origin
  - Gender, age, marital status
  - Religion, disability status
- Conduct disparate impact analysis quarterly
- Use fairness metrics: demographic parity, equalized odds

**Bias Mitigation Techniques**:
- Pre-processing: Reweighting, sampling, data augmentation
- In-processing: Adversarial debiasing, regularization
- Post-processing: Threshold optimization per protected group

---

## 8. References

### 8.1 Industry Standards
- PCI Security Standards Council. (2025). *Payment Card Industry Data Security Standard v4.0.1*. https://www.pcisecuritystandards.org/
- ISO/IEC. (2022). *ISO/IEC 27001:2022 - Information Security Management*.
- NIST. (2020). *NIST SP 800-53 Rev 5 - Security and Privacy Controls*.
- FATF. (2023). *International Standards on Combating Money Laundering*.

### 8.2 Research Papers
- Devarakonda, R. R. (2025). *Machine Learning Approach for Fraud Detection in Financial Services*. SSRN. https://papers.ssrn.com/sol3/papers.cfm?abstract_id=5234670
- Vora, R. (2024). *How I Built a Fraud Detection System with XGBoost and SMOTE*. Medium. https://rushhabhh.medium.com/how-i-built-a-fraud-detection-system-with-xgboost-and-smote-13cf2f3f7b18
- Shahidi, M. (2024). *Detecting Credit Card Fraud Using Extreme Gradient Boosting (XGBoost)*. GitHub. https://github.com/MiladShahidi/Fraud-Detection-XGBoost

### 8.3 Technology Documentation
- Apache Kafka Documentation: https://kafka.apache.org/documentation/
- Apache Flink Documentation: https://flink.apache.org/
- XGBoost Documentation: https://xgboost.readthedocs.io/
- TensorFlow Documentation: https://www.tensorflow.org/

### 8.4 Compliance Resources
- GDPR Official Text: https://gdpr-info.eu/
- PSD2 Directive: https://eur-lex.europa.eu/
- CCPA/CPRA: https://oag.ca.gov/privacy/ccpa

### 8.5 Web Sources

**Financial Fraud Detection Standards & ML**:
- [How Financial Technology is Transforming Risk Management in 2026](https://www.sigmainfo.net/blog/how-financial-technology-is-transforming-risk-management-in-2026/)
- [PCI Security Standards Council](https://www.pcisecuritystandards.org/)
- [Advanced fraud detection using machine learning models](https://www.sciencepubco.com/index.php/IJAES/article/view/34013)
- [How AI Is Transforming Fraud Detection in FinTech](https://techbullion.com/how-ai-is-transforming-fraud-detection-in-fintech/)

**XGBoost and Anomaly Detection**:
- [Fraud Detection in Mobile Payment Systems using XGBoost](https://pmc.ncbi.nlm.nih.gov/articles/PMC9560719/)
- [How I Built a Fraud Detection System with XGBoost and SMOTE](https://rushhabhh.medium.com/how-i-built-a-fraud-detection-system-with-xgboost-and-smote-13cf2f3f7b18)
- [A Hybrid Anomaly Detection Framework](https://f1000research.com/articles/14-664)

**Payment API Architecture**:
- [Top 10 Best Payment APIs in 2025](https://apidog.com/blog/top-payment-apis/)
- [Best Fraud Prevention APIs for Lenders in 2025](https://crscreditapi.com/fraud-prevention-apis-lenders/)
- [Fraud Detection API: How It Works and Business Benefits](https://www.wallarm.com/what/the-power-of-fraud-detection-apis-a-comprehensive-guide)

---

## 9. Glossary

| Term | Definition |
|------|------------|
| **AML** | Anti-Money Laundering - regulations to prevent financial crimes |
| **ATO** | Account Takeover - unauthorized access to user accounts |
| **AUC-ROC** | Area Under Receiver Operating Characteristic Curve - model performance metric |
| **CNP** | Card-Not-Present - transactions without physical card |
| **PAN** | Primary Account Number - the card number on payment cards |
| **SHAP** | SHapley Additive exPlanations - method for explaining ML predictions |
| **3DS2** | 3D Secure 2.0 - authentication protocol for online card payments |
| **TPS** | Transactions Per Second - throughput metric |

---

**Document Status**: ✅ **Approved for PHASE 2 Development**
**Next Phase**: [PHASE2.md](./PHASE2.md) - API Specification & Data Models

---

© 2026 WIA (World Certification Industry Association)
**Philosophy**: 弘益人間 (Hongik Ingan) - Benefit All Humanity
