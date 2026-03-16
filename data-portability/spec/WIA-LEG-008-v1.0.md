# WIA-LEG-008: Data Portability Specification v1.0

> **Standard ID:** WIA-LEG-008
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Digital Legacy Working Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Post-Mortem Data Rights Framework](#2-post-mortem-data-rights-framework)
3. [Data Export Formats and Standards](#3-data-export-formats-and-standards)
4. [Cross-Platform Transfer Protocols](#4-cross-platform-transfer-protocols)
5. [Digital Asset Portability](#5-digital-asset-portability)
6. [Service-to-Service Migration](#6-service-to-service-migration)
7. [Consent and Authorization](#7-consent-and-authorization)
8. [Privacy-Preserving Mechanisms](#8-privacy-preserving-mechanisms)
9. [GDPR and Regulatory Compliance](#9-gdpr-and-regulatory-compliance)
10. [Implementation Guidelines](#10-implementation-guidelines)
11. [Security Considerations](#11-security-considerations)
12. [References](#12-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines comprehensive data portability standards for digital legacy, ensuring that individuals and their authorized representatives can seamlessly transfer digital assets, personal data, and account information across platforms and services after death.

### 1.2 Scope

The standard covers:
- Rights to data portability after death
- Standardized export formats (JSON-LD, XML, CSV)
- Cross-platform data transfer protocols
- Digital asset inventory and cataloging
- Service-to-service migration mechanisms
- User consent and executor authorization frameworks
- Privacy-preserving data portability techniques
- Compliance with GDPR, CCPA, and other regulations

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard ensures that digital legacy rights extend beyond life, allowing individuals to maintain control over their data and enabling authorized parties to preserve, transfer, and manage digital assets in accordance with the deceased's wishes.

### 1.4 Terminology

- **Data Subject**: The individual whose data is being transferred (deceased)
- **Executor**: Authorized person with legal rights to access and transfer deceased's data
- **Data Portability Package (DPP)**: Standardized container for exported data
- **Source Platform**: Service from which data is being exported
- **Destination Platform**: Service to which data is being imported
- **Consent Record**: Blockchain-verified record of user authorization
- **Privacy Envelope**: Encrypted wrapper for sensitive data

---

## 2. Post-Mortem Data Rights Framework

### 2.1 Legal Foundation

#### 2.1.1 Rights Recognition

Post-mortem data rights include:

| Right | Description | Legal Basis |
|-------|-------------|-------------|
| Access | Right for executors to access deceased's data | Estate law, GDPR Article 15 |
| Portability | Right to transfer data between services | GDPR Article 20 |
| Preservation | Right to preserve data for posterity | Copyright law, cultural heritage |
| Distribution | Right to distribute data per will instructions | Estate law, testamentary freedom |
| Deletion | Right to delete data posthumously | GDPR Article 17, privacy law |

#### 2.1.2 Executor Authorization

Executors must prove authorization through:

```json
{
  "executor_authorization": {
    "executor_id": "did:wia:executor789",
    "deceased_id": "did:wia:user123",
    "authorization_type": "legal_executor",
    "proof_documents": [
      {
        "type": "death_certificate",
        "issuer": "vital_records_office",
        "document_hash": "sha256:abc123...",
        "verification_url": "https://verify.gov/cert/123"
      },
      {
        "type": "letters_testamentary",
        "issuer": "probate_court",
        "document_hash": "sha256:def456...",
        "court_case": "2025-EST-001234"
      }
    ],
    "valid_from": "2025-01-15T00:00:00Z",
    "valid_until": "2027-01-15T00:00:00Z",
    "granted_permissions": [
      "read_all_data",
      "export_data",
      "transfer_data",
      "delete_data"
    ]
  }
}
```

### 2.2 Jurisdiction Handling

#### 2.2.1 Multi-Jurisdictional Framework

```typescript
interface JurisdictionRules {
  jurisdiction: string; // ISO 3166-1 alpha-2
  postMortemDataRights: {
    recognizedByLaw: boolean;
    maxDataRetentionYears: number;
    executorRequirements: string[];
    restrictedDataCategories: string[];
  };
  complianceFramework: {
    gdpr: boolean;
    ccpa: boolean;
    localRegulations: string[];
  };
}
```

### 2.3 Time-Limited Access

Executor access may be time-limited:

```typescript
interface AccessWindow {
  grantedAt: Date;
  expiresAt: Date;
  renewalAllowed: boolean;
  autoRenewal: boolean;
  maxExtensions: number;
  reasonForExtension?: string;
}
```

---

## 3. Data Export Formats and Standards

### 3.1 JSON-LD Format (Primary)

#### 3.1.1 Data Portability Package Schema

```json
{
  "@context": "https://schema.wiastandards.com/leg-008/v1",
  "@type": "DataPortabilityPackage",
  "id": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",
  "version": "1.0",
  "generated_at": "2025-01-15T10:30:00Z",
  "expires_at": "2025-07-15T10:30:00Z",
  
  "deceased": {
    "@type": "Person",
    "id": "did:wia:123456",
    "name": "John Doe",
    "dateOfBirth": "1980-05-15",
    "dateOfDeath": "2025-01-01",
    "lastKnownLocation": {
      "@type": "Place",
      "address": "123 Main St, City, Country"
    }
  },
  
  "executor": {
    "@type": "LegalExecutor",
    "id": "did:wia:executor789",
    "name": "Jane Doe",
    "relationship": "spouse",
    "contactEmail": "jane@example.com",
    "authorization": {
      "type": "court_appointed",
      "document_ref": "urn:uuid:auth-token-xyz",
      "issued_by": "Superior Court of County",
      "valid_from": "2025-01-15",
      "valid_until": "2027-01-15"
    }
  },
  
  "data_inventory": {
    "total_categories": 8,
    "total_items": 15234,
    "total_size_bytes": 5368709120,
    "categories": {
      "social_media": {
        "item_count": 3456,
        "size_bytes": 1073741824,
        "platforms": ["facebook", "instagram", "twitter"]
      },
      "financial": {
        "item_count": 890,
        "size_bytes": 52428800,
        "platforms": ["bank_a", "investment_firm_b"]
      },
      "creative_works": {
        "item_count": 234,
        "size_bytes": 2147483648,
        "platforms": ["blog", "youtube", "github"]
      },
      "communications": {
        "item_count": 8934,
        "size_bytes": 536870912,
        "platforms": ["gmail", "outlook"]
      },
      "cloud_storage": {
        "item_count": 1234,
        "size_bytes": 1073741824,
        "platforms": ["google_drive", "dropbox"]
      },
      "gaming": {
        "item_count": 56,
        "size_bytes": 104857600,
        "platforms": ["steam", "playstation"]
      },
      "health": {
        "item_count": 123,
        "size_bytes": 10485760,
        "platforms": ["fitbit", "apple_health"]
      },
      "professional": {
        "item_count": 307,
        "size_bytes": 268435456,
        "platforms": ["linkedin", "github"]
      }
    }
  },
  
  "encryption": {
    "algorithm": "AES-256-GCM",
    "key_derivation": "PBKDF2",
    "iterations": 100000,
    "salt": "base64_encoded_salt",
    "iv": "base64_encoded_iv",
    "auth_tag": "base64_encoded_tag"
  },
  
  "data": {
    "social_media": [...],
    "financial": [...],
    "creative_works": [...],
    "communications": [...],
    "cloud_storage": [...],
    "gaming": [...],
    "health": [...],
    "professional": [...]
  },
  
  "metadata": {
    "export_method": "automated",
    "export_tool": "wia-leg-008-sdk v1.0.0",
    "checksum": "sha256:full_package_hash",
    "signature": "executor_digital_signature",
    "audit_trail": [
      {
        "timestamp": "2025-01-15T10:00:00Z",
        "action": "export_initiated",
        "actor": "did:wia:executor789"
      },
      {
        "timestamp": "2025-01-15T10:30:00Z",
        "action": "export_completed",
        "actor": "system"
      }
    ]
  }
}
```

#### 3.1.2 Category-Specific Schemas

**Social Media Data:**

```json
{
  "@type": "SocialMediaData",
  "platform": "facebook",
  "account_id": "user_facebook_id",
  "profile": {
    "username": "johndoe",
    "display_name": "John Doe",
    "bio": "...",
    "profile_picture_url": "https://...",
    "cover_photo_url": "https://..."
  },
  "posts": [
    {
      "id": "post_123",
      "created_at": "2024-12-01T15:30:00Z",
      "content": "Post content...",
      "media": [...],
      "reactions": {...},
      "comments": [...]
    }
  ],
  "photos": [...],
  "videos": [...],
  "friends": [...],
  "messages": [...]
}
```

**Financial Data:**

```json
{
  "@type": "FinancialData",
  "institution": "Bank of Example",
  "account_type": "checking",
  "account_number": "****1234",
  "transactions": [
    {
      "id": "txn_456",
      "date": "2024-12-15",
      "description": "Purchase at Store",
      "amount": -45.67,
      "currency": "USD",
      "category": "groceries"
    }
  ],
  "statements": [...],
  "tax_documents": [...]
}
```

### 3.2 Alternative Formats

#### 3.2.1 XML Format

```xml
<?xml version="1.0" encoding="UTF-8"?>
<DataPortabilityPackage xmlns="https://schema.wiastandards.com/leg-008/v1">
  <id>urn:uuid:550e8400-e29b-41d4-a716-446655440000</id>
  <version>1.0</version>
  <deceased>
    <id>did:wia:123456</id>
    <name>John Doe</name>
  </deceased>
  <!-- ... -->
</DataPortabilityPackage>
```

#### 3.2.2 CSV Format (for tabular data)

Used for specific data categories like transactions, contacts:

```csv
category,platform,item_type,item_id,created_at,content,size_bytes
social_media,facebook,post,post_123,2024-12-01T15:30:00Z,"Post content...",1024
social_media,instagram,photo,photo_456,2024-12-05T10:00:00Z,"",2048000
financial,bank_a,transaction,txn_789,2024-12-15T00:00:00Z,"Purchase at Store",256
```

---

## 4. Cross-Platform Transfer Protocols

### 4.1 Transfer Workflow

```
┌──────────────────────────────────────────────────────────┐
│                  Transfer Initiation                      │
│  1. Executor authenticates                                │
│  2. Select source and destination platforms               │
│  3. Choose data categories to transfer                    │
└──────────────────────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────┐
│                  Authorization Verification               │
│  1. Verify executor credentials                           │
│  2. Check platform ToS compliance                         │
│  3. Validate consent records                              │
└──────────────────────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────┐
│                  Data Export from Source                  │
│  1. Generate DPP from source platform                     │
│  2. Encrypt sensitive data                                │
│  3. Create export manifest                                │
└──────────────────────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────┐
│                  Data Transformation                      │
│  1. Map source schema to destination schema               │
│  2. Convert data formats                                  │
│  3. Validate data integrity                               │
└──────────────────────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────┐
│                  Data Import to Destination               │
│  1. Create import request                                 │
│  2. Upload transformed data                               │
│  3. Verify import success                                 │
└──────────────────────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────┐
│                  Verification & Audit                     │
│  1. Generate transfer completion report                   │
│  2. Update audit trail                                    │
│  3. Notify executor                                       │
└──────────────────────────────────────────────────────────┘
```

### 4.2 API Specifications

#### 4.2.1 Export Initiation

```http
POST /api/v1/export/initiate
Authorization: Bearer {executor_jwt_token}
Content-Type: application/json

{
  "deceased_id": "did:wia:123456",
  "executor_id": "did:wia:executor789",
  "platform": "facebook",
  "categories": ["posts", "photos", "messages"],
  "format": "json-ld",
  "encryption": {
    "enabled": true,
    "algorithm": "AES-256-GCM"
  }
}

Response:
{
  "export_id": "exp_abc123",
  "status": "initiated",
  "estimated_completion": "2025-01-15T12:00:00Z",
  "estimated_size_bytes": 1073741824
}
```

#### 4.2.2 Export Status Check

```http
GET /api/v1/export/{export_id}/status
Authorization: Bearer {executor_jwt_token}

Response:
{
  "export_id": "exp_abc123",
  "status": "in_progress",
  "progress_percent": 45,
  "items_processed": 1560,
  "items_total": 3456,
  "current_category": "photos"
}
```

#### 4.2.3 Export Download

```http
GET /api/v1/export/{export_id}/download
Authorization: Bearer {executor_jwt_token}

Response:
HTTP/1.1 200 OK
Content-Type: application/json+ld
Content-Disposition: attachment; filename="dpp_123456_20250115.json"
Content-Length: 1073741824

{DPP content}
```

#### 4.2.4 Import Initiation

```http
POST /api/v1/import/initiate
Authorization: Bearer {executor_jwt_token}
Content-Type: multipart/form-data

{
  "dpp_file": (binary data),
  "destination_platform": "memorial_service",
  "import_options": {
    "privacy_level": "friends_only",
    "enable_comments": false,
    "notification_preferences": "none"
  }
}

Response:
{
  "import_id": "imp_xyz789",
  "status": "validating",
  "validation_results": {
    "format_valid": true,
    "schema_valid": true,
    "encryption_valid": true
  }
}
```

### 4.3 Transfer Protocols

#### 4.3.1 OAuth 2.0 Integration

```typescript
interface OAuth2Config {
  client_id: string;
  client_secret: string;
  authorization_endpoint: string;
  token_endpoint: string;
  scopes: string[];
  redirect_uri: string;
}

// Example: Facebook OAuth for data export
const fbOAuth: OAuth2Config = {
  client_id: "wia_leg_008_client",
  client_secret: "secret_key",
  authorization_endpoint: "https://www.facebook.com/v18.0/dialog/oauth",
  token_endpoint: "https://graph.facebook.com/v18.0/oauth/access_token",
  scopes: [
    "user_posts",
    "user_photos",
    "user_videos",
    "user_friends",
    "email"
  ],
  redirect_uri: "https://portability.wiastandards.com/callback"
};
```

#### 4.3.2 Data Streaming Protocol

For large datasets:

```typescript
interface StreamConfig {
  chunk_size_bytes: number; // e.g., 10MB
  compression: "gzip" | "brotli";
  checksum_algorithm: "sha256";
  retry_policy: {
    max_retries: number;
    backoff_ms: number;
  };
}

// Stream data in chunks
async function* streamExportData(
  exportId: string,
  config: StreamConfig
): AsyncGenerator<Uint8Array> {
  let offset = 0;
  while (true) {
    const chunk = await fetchChunk(exportId, offset, config.chunk_size_bytes);
    if (chunk.length === 0) break;
    yield chunk;
    offset += chunk.length;
  }
}
```

---

## 5. Digital Asset Portability

### 5.1 Asset Classification

```typescript
enum AssetType {
  SOCIAL_MEDIA_POST = "social_media_post",
  PHOTO = "photo",
  VIDEO = "video",
  AUDIO = "audio",
  DOCUMENT = "document",
  EMAIL = "email",
  MESSAGE = "message",
  FINANCIAL_RECORD = "financial_record",
  HEALTH_RECORD = "health_record",
  CONTACT = "contact",
  CALENDAR_EVENT = "calendar_event",
  BOOKMARK = "bookmark",
  PASSWORD = "password",
  CRYPTOCURRENCY = "cryptocurrency",
  NFT = "nft",
  GAME_SAVE = "game_save",
  VIRTUAL_ITEM = "virtual_item"
}

interface DigitalAsset {
  id: string;
  type: AssetType;
  platform: string;
  created_at: Date;
  modified_at: Date;
  size_bytes: number;
  format: string;
  url?: string;
  local_path?: string;
  metadata: Record<string, any>;
  ownership_proof?: string;
  transferable: boolean;
  transfer_restrictions?: string[];
}
```

### 5.2 Asset Inventory Generation

```typescript
interface AssetInventory {
  inventory_id: string;
  deceased_id: string;
  generated_at: Date;
  total_assets: number;
  total_value_usd?: number;
  categories: {
    [category: string]: {
      count: number;
      total_size_bytes: number;
      assets: DigitalAsset[];
    };
  };
  platforms: {
    [platform: string]: {
      account_id: string;
      last_activity: Date;
      asset_count: number;
      exportable: boolean;
      export_method?: string;
    };
  };
}
```

### 5.3 Media Asset Handling

#### 5.3.1 Photo/Video Metadata Preservation

```json
{
  "@type": "MediaAsset",
  "id": "photo_12345",
  "asset_type": "photo",
  "file_name": "vacation_2024.jpg",
  "size_bytes": 2048000,
  "mime_type": "image/jpeg",
  "dimensions": {
    "width": 1920,
    "height": 1080
  },
  "exif": {
    "camera_make": "Canon",
    "camera_model": "EOS R5",
    "date_taken": "2024-07-15T14:30:00Z",
    "location": {
      "latitude": 37.7749,
      "longitude": -122.4194,
      "altitude": 15.5
    },
    "settings": {
      "iso": 400,
      "aperture": "f/2.8",
      "shutter_speed": "1/250",
      "focal_length": "50mm"
    }
  },
  "people_tagged": [
    {"id": "person_1", "name": "Alice"},
    {"id": "person_2", "name": "Bob"}
  ],
  "albums": ["Vacation 2024", "Family Photos"],
  "caption": "Beautiful sunset at the beach",
  "privacy": "friends",
  "download_url": "https://encrypted-storage.example.com/photo_12345"
}
```

### 5.4 Financial Asset Portability

```typescript
interface FinancialAsset {
  asset_id: string;
  type: "bank_account" | "investment" | "cryptocurrency" | "stock" | "bond";
  institution: string;
  account_number: string; // masked
  balance: {
    amount: number;
    currency: string;
    as_of_date: Date;
  };
  transactions: Transaction[];
  statements: Statement[];
  beneficiaries?: Beneficiary[];
  transfer_instructions?: {
    method: string;
    destination_account?: string;
    executor_access_required: boolean;
  };
}
```

---

## 6. Service-to-Service Migration

### 6.1 Platform Connectors

#### 6.1.1 Connector Interface

```typescript
interface PlatformConnector {
  platform_id: string;
  platform_name: string;
  api_version: string;
  
  // Authentication
  authenticate(credentials: Credentials): Promise<AuthToken>;
  
  // Export
  initiateExport(config: ExportConfig): Promise<ExportJob>;
  getExportStatus(jobId: string): Promise<ExportStatus>;
  downloadExport(jobId: string): Promise<DataPortabilityPackage>;
  
  // Import
  validateImport(dpp: DataPortabilityPackage): Promise<ValidationResult>;
  initiateImport(dpp: DataPortabilityPackage): Promise<ImportJob>;
  getImportStatus(jobId: string): Promise<ImportStatus>;
  
  // Mapping
  mapToStandardSchema(platformData: any): StandardSchema;
  mapFromStandardSchema(standardData: StandardSchema): any;
}
```

#### 6.1.2 Facebook Connector Example

```typescript
class FacebookConnector implements PlatformConnector {
  platform_id = "facebook";
  platform_name = "Facebook";
  api_version = "v18.0";
  
  async authenticate(credentials: Credentials): Promise<AuthToken> {
    // OAuth 2.0 flow
    const response = await fetch(
      `https://graph.facebook.com/${this.api_version}/oauth/access_token`,
      {
        method: 'POST',
        body: JSON.stringify({
          client_id: credentials.client_id,
          client_secret: credentials.client_secret,
          grant_type: 'authorization_code',
          code: credentials.auth_code,
          redirect_uri: credentials.redirect_uri
        })
      }
    );
    return await response.json();
  }
  
  async initiateExport(config: ExportConfig): Promise<ExportJob> {
    // Use Facebook's Data Transfer API
    const response = await this.apiCall('/me/data_transfer', {
      method: 'POST',
      params: {
        destination: 'wia_portability_service',
        scope: config.categories.join(',')
      }
    });
    return {
      job_id: response.transfer_id,
      status: 'initiated',
      estimated_completion: response.estimated_completion_time
    };
  }
  
  mapToStandardSchema(fbData: any): StandardSchema {
    // Map Facebook-specific data to WIA standard schema
    return {
      '@context': 'https://schema.wiastandards.com/leg-008/v1',
      '@type': 'SocialMediaData',
      platform: 'facebook',
      posts: fbData.posts.data.map(post => ({
        id: post.id,
        created_at: post.created_time,
        content: post.message,
        media: post.attachments?.data || [],
        reactions: post.reactions?.summary || {},
        comments: post.comments?.data || []
      }))
    };
  }
}
```

### 6.2 Schema Mapping

#### 6.2.1 Mapping Configuration

```json
{
  "schema_mapping": {
    "version": "1.0",
    "source_platform": "facebook",
    "destination_platform": "memorial_service",
    "field_mappings": [
      {
        "source_field": "posts[].message",
        "destination_field": "memories[].content",
        "transform": "identity"
      },
      {
        "source_field": "posts[].created_time",
        "destination_field": "memories[].date",
        "transform": "iso8601_to_timestamp"
      },
      {
        "source_field": "photos[].images[0].source",
        "destination_field": "gallery[].image_url",
        "transform": "identity"
      },
      {
        "source_field": "friends[].name",
        "destination_field": "connections[].display_name",
        "transform": "identity"
      }
    ],
    "category_mappings": {
      "posts": "memories",
      "photos": "gallery",
      "friends": "connections",
      "videos": "videos"
    }
  }
}
```

#### 6.2.2 Transform Functions

```typescript
type TransformFunction = (input: any) => any;

const transforms: Record<string, TransformFunction> = {
  identity: (x) => x,
  
  iso8601_to_timestamp: (dateString: string) => 
    new Date(dateString).getTime(),
  
  timestamp_to_iso8601: (timestamp: number) => 
    new Date(timestamp).toISOString(),
  
  array_to_csv: (arr: any[]) => arr.join(','),
  
  csv_to_array: (csv: string) => csv.split(','),
  
  url_to_blob: async (url: string) => {
    const response = await fetch(url);
    return await response.blob();
  },
  
  html_to_markdown: (html: string) => {
    // Use library like turndown
    return convertHtmlToMarkdown(html);
  },
  
  markdown_to_html: (markdown: string) => {
    // Use library like marked
    return convertMarkdownToHtml(markdown);
  }
};
```

### 6.3 Automated Migration Pipeline

```typescript
class MigrationPipeline {
  async executeMigration(
    source: PlatformConnector,
    destination: PlatformConnector,
    config: MigrationConfig
  ): Promise<MigrationResult> {
    
    // Step 1: Export from source
    const exportJob = await source.initiateExport({
      categories: config.categories,
      format: 'json-ld'
    });
    
    // Step 2: Wait for export completion
    const dpp = await this.waitForExport(source, exportJob.job_id);
    
    // Step 3: Validate DPP
    const validation = await destination.validateImport(dpp);
    if (!validation.valid) {
      throw new Error(`Validation failed: ${validation.errors.join(', ')}`);
    }
    
    // Step 4: Transform data
    const transformed = await this.transformData(
      dpp,
      config.schema_mapping
    );
    
    // Step 5: Import to destination
    const importJob = await destination.initiateImport(transformed);
    
    // Step 6: Wait for import completion
    const result = await this.waitForImport(destination, importJob.job_id);
    
    // Step 7: Verify migration
    await this.verifyMigration(source, destination, dpp, result);
    
    return {
      migration_id: uuidv4(),
      source_platform: source.platform_id,
      destination_platform: destination.platform_id,
      items_migrated: result.items_imported,
      status: 'completed',
      audit_trail: [...]
    };
  }
}
```

---

## 7. Consent and Authorization

### 7.1 User Consent Framework

#### 7.1.1 Consent Record Structure

```json
{
  "@type": "ConsentRecord",
  "id": "urn:uuid:consent-abc-123",
  "version": "1.0",
  "user_id": "did:wia:123456",
  "granted_at": "2024-01-01T00:00:00Z",
  "expires_at": "2054-01-01T00:00:00Z",
  "consent_type": "post_mortem_data_portability",
  
  "granted_permissions": {
    "data_export": {
      "enabled": true,
      "categories": ["all"],
      "format": ["json-ld", "xml"],
      "encryption_required": true
    },
    "data_transfer": {
      "enabled": true,
      "allowed_destinations": ["memorial_services", "family_archives"],
      "prohibited_destinations": ["commercial_platforms"]
    },
    "data_deletion": {
      "enabled": true,
      "delay_days": 90,
      "preserve_categories": ["creative_works", "financial"]
    }
  },
  
  "authorized_executors": [
    {
      "executor_id": "did:wia:executor789",
      "name": "Jane Doe",
      "relationship": "spouse",
      "email": "jane@example.com",
      "phone": "+1-555-0100",
      "permissions": ["full_access"],
      "priority": 1,
      "alternate_contact": {
        "email": "jane.backup@example.com",
        "phone": "+1-555-0101"
      }
    },
    {
      "executor_id": "did:wia:executor456",
      "name": "John Doe Jr.",
      "relationship": "child",
      "email": "johnjr@example.com",
      "permissions": ["read_only"],
      "priority": 2
    }
  ],
  
  "special_instructions": {
    "social_media": "Transfer all posts to family memorial site",
    "photos": "Create shared family album, send copies to children",
    "financial": "Provide statements to executor only",
    "email": "Delete after 30 days except emails in 'Important' folder",
    "creative_works": "Publish posthumously on personal blog"
  },
  
  "blockchain_proof": {
    "chain": "ethereum",
    "contract_address": "0x1234567890abcdef",
    "transaction_hash": "0xabcdef1234567890",
    "block_number": 12345678,
    "timestamp": "2024-01-01T00:00:00Z"
  },
  
  "signature": {
    "algorithm": "ECDSA",
    "public_key": "base64_encoded_public_key",
    "signature_value": "base64_encoded_signature"
  }
}
```

#### 7.1.2 Consent Verification API

```typescript
interface ConsentVerificationService {
  // Verify executor has valid consent
  verifyExecutorConsent(
    deceasedId: string,
    executorId: string,
    requestedPermissions: string[]
  ): Promise<VerificationResult>;
  
  // Check blockchain proof
  verifyBlockchainProof(
    consentId: string
  ): Promise<BlockchainVerification>;
  
  // Validate consent signature
  verifySignature(
    consent: ConsentRecord
  ): Promise<SignatureVerification>;
}

// Implementation
class ConsentVerifier implements ConsentVerificationService {
  async verifyExecutorConsent(
    deceasedId: string,
    executorId: string,
    requestedPermissions: string[]
  ): Promise<VerificationResult> {
    
    // 1. Retrieve consent record
    const consent = await this.getConsentRecord(deceasedId);
    
    // 2. Check if executor is authorized
    const executor = consent.authorized_executors.find(
      e => e.executor_id === executorId
    );
    
    if (!executor) {
      return {
        valid: false,
        reason: "Executor not authorized"
      };
    }
    
    // 3. Check if consent is still valid
    if (new Date() > new Date(consent.expires_at)) {
      return {
        valid: false,
        reason: "Consent expired"
      };
    }
    
    // 4. Verify requested permissions
    const hasPermissions = requestedPermissions.every(
      perm => executor.permissions.includes(perm) || 
              executor.permissions.includes('full_access')
    );
    
    if (!hasPermissions) {
      return {
        valid: false,
        reason: "Insufficient permissions"
      };
    }
    
    // 5. Verify blockchain proof
    const blockchainValid = await this.verifyBlockchainProof(consent.id);
    
    if (!blockchainValid.valid) {
      return {
        valid: false,
        reason: "Blockchain verification failed"
      };
    }
    
    // 6. Verify signature
    const signatureValid = await this.verifySignature(consent);
    
    if (!signatureValid.valid) {
      return {
        valid: false,
        reason: "Signature verification failed"
      };
    }
    
    return {
      valid: true,
      consent_id: consent.id,
      executor_name: executor.name,
      granted_permissions: executor.permissions
    };
  }
}
```

### 7.2 Multi-Factor Authentication

#### 7.2.1 MFA Requirements

Executor authentication requires:

1. **Knowledge Factor**: Password or PIN
2. **Possession Factor**: Hardware token, mobile device, or security key
3. **Inherence Factor**: Biometric (fingerprint, face recognition)
4. **Legal Documentation**: Death certificate, letters testamentary

```typescript
interface MFAChallenge {
  challenge_id: string;
  required_factors: FactorType[];
  completed_factors: FactorType[];
  status: 'pending' | 'completed' | 'failed';
  expires_at: Date;
}

enum FactorType {
  PASSWORD = 'password',
  TOTP = 'totp',
  SMS = 'sms',
  EMAIL = 'email',
  HARDWARE_TOKEN = 'hardware_token',
  BIOMETRIC = 'biometric',
  LEGAL_DOCUMENT = 'legal_document'
}

class MFAService {
  async initiateMFA(
    executorId: string,
    requiredFactors: FactorType[]
  ): Promise<MFAChallenge> {
    return {
      challenge_id: uuidv4(),
      required_factors: requiredFactors,
      completed_factors: [],
      status: 'pending',
      expires_at: new Date(Date.now() + 15 * 60 * 1000) // 15 min
    };
  }
  
  async verifyFactor(
    challengeId: string,
    factor: FactorType,
    proof: any
  ): Promise<FactorVerificationResult> {
    // Verify the provided factor
    switch (factor) {
      case FactorType.PASSWORD:
        return this.verifyPassword(proof.password);
      
      case FactorType.TOTP:
        return this.verifyTOTP(proof.code);
      
      case FactorType.BIOMETRIC:
        return this.verifyBiometric(proof.biometric_data);
      
      case FactorType.LEGAL_DOCUMENT:
        return this.verifyLegalDocument(proof.document);
      
      default:
        throw new Error(`Unsupported factor: ${factor}`);
    }
  }
}
```

---

## 8. Privacy-Preserving Mechanisms

### 8.1 Encryption Standards

#### 8.1.1 Data Encryption

```typescript
interface EncryptionConfig {
  algorithm: 'AES-256-GCM' | 'ChaCha20-Poly1305';
  key_derivation: 'PBKDF2' | 'Argon2id';
  iterations: number; // e.g., 100000 for PBKDF2
  salt_bytes: number; // e.g., 32
  iv_bytes: number; // e.g., 12 for GCM
}

class DataEncryption {
  async encryptDPP(
    dpp: DataPortabilityPackage,
    password: string,
    config: EncryptionConfig
  ): Promise<EncryptedDPP> {
    
    // 1. Generate salt
    const salt = crypto.getRandomValues(new Uint8Array(config.salt_bytes));
    
    // 2. Derive encryption key
    const key = await this.deriveKey(password, salt, config);
    
    // 3. Generate IV
    const iv = crypto.getRandomValues(new Uint8Array(config.iv_bytes));
    
    // 4. Encrypt data
    const plaintext = JSON.stringify(dpp);
    const ciphertext = await crypto.subtle.encrypt(
      {
        name: 'AES-GCM',
        iv: iv
      },
      key,
      new TextEncoder().encode(plaintext)
    );
    
    // 5. Create encrypted package
    return {
      version: '1.0',
      encryption: {
        algorithm: config.algorithm,
        key_derivation: config.key_derivation,
        iterations: config.iterations,
        salt: this.base64Encode(salt),
        iv: this.base64Encode(iv)
      },
      ciphertext: this.base64Encode(new Uint8Array(ciphertext))
    };
  }
  
  async decryptDPP(
    encrypted: EncryptedDPP,
    password: string
  ): Promise<DataPortabilityPackage> {
    
    // 1. Decode parameters
    const salt = this.base64Decode(encrypted.encryption.salt);
    const iv = this.base64Decode(encrypted.encryption.iv);
    const ciphertext = this.base64Decode(encrypted.ciphertext);
    
    // 2. Derive key
    const key = await this.deriveKey(password, salt, encrypted.encryption);
    
    // 3. Decrypt
    const plaintext = await crypto.subtle.decrypt(
      {
        name: 'AES-GCM',
        iv: iv
      },
      key,
      ciphertext
    );
    
    // 4. Parse JSON
    return JSON.parse(new TextDecoder().decode(plaintext));
  }
}
```

### 8.2 Selective Redaction

#### 8.2.1 Redaction Rules

```typescript
interface RedactionRule {
  field_path: string; // JSON path, e.g., "data.financial[*].account_number"
  redaction_type: 'mask' | 'remove' | 'hash' | 'encrypt';
  mask_pattern?: string; // e.g., "****{last4}"
  conditions?: {
    field: string;
    operator: '==' | '!=' | '>' | '<' | 'contains';
    value: any;
  }[];
}

const redactionRules: RedactionRule[] = [
  {
    field_path: "data.financial[*].account_number",
    redaction_type: "mask",
    mask_pattern: "****{last4}"
  },
  {
    field_path: "data.health[*].ssn",
    redaction_type: "remove"
  },
  {
    field_path: "data.communications[*].message_content",
    redaction_type: "encrypt",
    conditions: [
      {
        field: "privacy_level",
        operator: "==",
        value: "private"
      }
    ]
  }
];

class RedactionEngine {
  applyRedactions(
    dpp: DataPortabilityPackage,
    rules: RedactionRule[]
  ): DataPortabilityPackage {
    
    let redacted = JSON.parse(JSON.stringify(dpp)); // deep clone
    
    for (const rule of rules) {
      const matches = this.findFieldsByPath(redacted, rule.field_path);
      
      for (const match of matches) {
        if (this.evaluateConditions(match, rule.conditions)) {
          match.value = this.applyRedaction(match.value, rule);
        }
      }
    }
    
    return redacted;
  }
  
  private applyRedaction(value: any, rule: RedactionRule): any {
    switch (rule.redaction_type) {
      case 'mask':
        return this.maskValue(value, rule.mask_pattern);
      
      case 'remove':
        return undefined;
      
      case 'hash':
        return this.hashValue(value);
      
      case 'encrypt':
        return this.encryptValue(value);
      
      default:
        return value;
    }
  }
  
  private maskValue(value: string, pattern: string): string {
    // Example: "1234567890" with pattern "****{last4}" -> "****7890"
    const last4 = value.slice(-4);
    return pattern.replace('{last4}', last4);
  }
}
```

### 8.3 Zero-Knowledge Proofs

#### 8.3.1 ZKP for Consent Verification

```typescript
interface ZKProof {
  proof_type: 'executor_authorization' | 'data_ownership' | 'consent_validity';
  proof_data: string; // Base64-encoded proof
  public_inputs: any[];
  verification_key: string;
}

class ZKConsentProver {
  // Prove executor has valid authorization without revealing consent details
  async proveExecutorAuthorization(
    consentRecord: ConsentRecord,
    executorId: string,
    requestedPermissions: string[]
  ): Promise<ZKProof> {
    
    // Private inputs (not revealed)
    const privateInputs = {
      consent_record: consentRecord,
      executor_details: consentRecord.authorized_executors.find(
        e => e.executor_id === executorId
      )
    };
    
    // Public inputs (revealed)
    const publicInputs = {
      deceased_id: consentRecord.user_id,
      executor_id: executorId,
      current_timestamp: Date.now(),
      requested_permissions: requestedPermissions
    };
    
    // Generate proof
    const proof = await zkSNARK.prove(
      this.authorizationCircuit,
      privateInputs,
      publicInputs
    );
    
    return {
      proof_type: 'executor_authorization',
      proof_data: this.base64Encode(proof),
      public_inputs: [publicInputs],
      verification_key: this.authorizationVerificationKey
    };
  }
  
  async verifyProof(proof: ZKProof): Promise<boolean> {
    const proofData = this.base64Decode(proof.proof_data);
    
    return await zkSNARK.verify(
      proof.verification_key,
      proof.public_inputs,
      proofData
    );
  }
}
```

---

## 9. GDPR and Regulatory Compliance

### 9.1 GDPR Article 20 Compliance

#### 9.1.1 Right to Data Portability

The standard ensures full compliance with GDPR Article 20:

| GDPR Requirement | Implementation |
|------------------|----------------|
| Structured format | JSON-LD, XML, CSV formats |
| Commonly used format | Industry-standard formats |
| Machine-readable | All exports are machine-parseable |
| Interoperable | Schema mapping for cross-platform transfer |
| No hindrance | Automated export and transfer processes |
| Direct transmission | Service-to-service migration support |

#### 9.1.2 Post-Mortem Considerations

GDPR Recital 68 states that member states may provide rules for the exercise of data subject rights by deceased individuals:

```typescript
interface GDPRPostMortemRules {
  jurisdiction: string; // ISO 3166-1
  post_mortem_rights_recognized: boolean;
  legal_basis: string; // e.g., "Article 85 domestic law"
  
  executor_requirements: {
    proof_of_death_required: boolean;
    legal_authorization_required: boolean;
    acceptable_documents: string[];
  };
  
  data_retention: {
    max_retention_period_days: number;
    exceptions: string[];
  };
  
  prohibited_transfers: {
    countries: string[];
    categories: string[];
  };
}

// Example: French law (Article 40-1 of the Data Protection Act)
const francePostMortemRules: GDPRPostMortemRules = {
  jurisdiction: "FR",
  post_mortem_rights_recognized: true,
  legal_basis: "Loi n° 2016-1321 Article 40-1",
  
  executor_requirements: {
    proof_of_death_required: true,
    legal_authorization_required: true,
    acceptable_documents: [
      "death_certificate",
      "will_designation",
      "family_authorization"
    ]
  },
  
  data_retention: {
    max_retention_period_days: 365,
    exceptions: ["archival_purposes", "legal_obligations"]
  },
  
  prohibited_transfers: {
    countries: [], // Subject to GDPR Chapter V adequacy decisions
    categories: []
  }
};
```

### 9.2 CCPA Compliance

#### 9.2.1 California Consumer Privacy Act

```typescript
interface CCPACompliance {
  // Consumer right to data portability (CCPA § 1798.110)
  consumer_data_request: {
    categories_of_data: string[];
    sources_of_data: string[];
    business_purposes: string[];
    third_parties_shared_with: string[];
  };
  
  // Format requirements
  format: {
    readily_useable: boolean; // Must be true
    formats_offered: string[]; // e.g., ["json", "csv", "pdf"]
    electronic_transmission: boolean; // Must be true
  };
  
  // Response timeline
  response_timeline_days: 45; // Max 45 days under CCPA
  extension_allowed_days: 45; // Can extend by 45 days if reasonably necessary
  
  // Verification requirements
  verification: {
    two_step_process: boolean;
    match_identity_with_records: boolean;
  };
}
```

### 9.3 Other Regulatory Frameworks

#### 9.3.1 Multi-Jurisdictional Compliance Matrix

| Regulation | Jurisdiction | Data Portability Right | Post-Mortem Provisions |
|------------|--------------|------------------------|------------------------|
| GDPR | EU/EEA | Article 20 | Recital 68 (member state law) |
| CCPA | California, USA | § 1798.110 | Limited (estate executor access) |
| LGPD | Brazil | Article 18, IV | Article 21 (heir rights) |
| PIPA | South Korea | Article 4 | Article 39-8 (family requests) |
| PIPEDA | Canada | Principle 4.9 | Common law (estate rights) |
| PDPA | Singapore | Section 21 | Not explicitly addressed |

#### 9.3.2 Compliance API

```typescript
interface ComplianceEngine {
  // Determine applicable regulations
  getApplicableRegulations(
    userJurisdiction: string,
    platformJurisdictions: string[]
  ): Regulation[];
  
  // Check if export meets regulatory requirements
  validateExport(
    dpp: DataPortabilityPackage,
    regulations: Regulation[]
  ): ComplianceReport;
  
  // Generate compliance documentation
  generateComplianceReport(
    exportId: string
  ): Promise<ComplianceReport>;
}

interface ComplianceReport {
  export_id: string;
  regulations_checked: string[];
  compliant: boolean;
  issues: ComplianceIssue[];
  recommendations: string[];
  generated_at: Date;
}
```

---

## 10. Implementation Guidelines

### 10.1 Platform Integration

#### 10.1.1 For Platform Providers

Platforms should implement the following APIs:

```typescript
// Export API
POST /wia/v1/export/initiate
GET  /wia/v1/export/{id}/status
GET  /wia/v1/export/{id}/download

// Import API
POST /wia/v1/import/validate
POST /wia/v1/import/initiate
GET  /wia/v1/import/{id}/status

// Consent API
GET  /wia/v1/consent/{user_id}
POST /wia/v1/consent/{user_id}/verify

// Metadata API
GET  /wia/v1/metadata/schema
GET  /wia/v1/metadata/categories
```

#### 10.1.2 SDK Usage

```typescript
import { DataPortabilitySDK } from '@wia/leg-008';

// Initialize SDK
const sdk = new DataPortabilitySDK({
  platform: 'my_platform',
  api_key: 'platform_api_key',
  environment: 'production'
});

// Register platform capabilities
await sdk.registerPlatform({
  platform_id: 'my_platform',
  supported_categories: [
    'social_media',
    'photos',
    'videos',
    'documents'
  ],
  export_formats: ['json-ld', 'xml'],
  import_formats: ['json-ld'],
  max_export_size_bytes: 10 * 1024 * 1024 * 1024, // 10GB
  oauth_config: {
    authorization_endpoint: 'https://myplatform.com/oauth/authorize',
    token_endpoint: 'https://myplatform.com/oauth/token',
    scopes: ['read_data', 'export_data']
  }
});

// Handle export request
sdk.onExportRequest(async (request) => {
  const data = await myPlatform.getUserData(request.user_id, request.categories);
  const dpp = await sdk.createDPP(data, request.format);
  return dpp;
});

// Handle import request
sdk.onImportRequest(async (request) => {
  const validation = await sdk.validateDPP(request.dpp);
  if (!validation.valid) {
    throw new Error(validation.errors.join(', '));
  }
  
  const transformed = await sdk.transformDPP(
    request.dpp,
    myPlatform.schema
  );
  
  await myPlatform.importData(request.user_id, transformed);
  
  return {
    import_id: generateId(),
    status: 'completed',
    items_imported: transformed.items.length
  };
});
```

### 10.2 Testing and Validation

#### 10.2.1 Test Scenarios

```typescript
describe('WIA-LEG-008 Compliance Tests', () => {
  
  test('Export generates valid DPP', async () => {
    const dpp = await sdk.exportData({
      user_id: 'test_user',
      categories: ['all']
    });
    
    expect(dpp['@context']).toBe('https://schema.wiastandards.com/leg-008/v1');
    expect(dpp.version).toBe('1.0');
    expect(dpp.deceased).toBeDefined();
    expect(dpp.data_inventory).toBeDefined();
  });
  
  test('Executor authorization verification', async () => {
    const result = await sdk.verifyExecutorConsent(
      'deceased_user_id',
      'executor_id',
      ['export_data']
    );
    
    expect(result.valid).toBe(true);
    expect(result.granted_permissions).toContain('export_data');
  });
  
  test('Cross-platform transfer', async () => {
    const exportResult = await sourcePlatform.export({
      user_id: 'test_user',
      categories: ['posts', 'photos']
    });
    
    const importResult = await destinationPlatform.import(exportResult.dpp);
    
    expect(importResult.status).toBe('completed');
    expect(importResult.items_imported).toBeGreaterThan(0);
  });
  
  test('GDPR compliance check', async () => {
    const compliance = await sdk.checkGDPRCompliance(dpp);
    
    expect(compliance.article_20_compliant).toBe(true);
    expect(compliance.machine_readable).toBe(true);
    expect(compliance.structured_format).toBe(true);
  });
  
  test('Encryption and decryption', async () => {
    const encrypted = await sdk.encryptDPP(dpp, 'password');
    const decrypted = await sdk.decryptDPP(encrypted, 'password');
    
    expect(decrypted).toEqual(dpp);
  });
  
  test('Selective redaction', async () => {
    const redacted = await sdk.applyRedactions(dpp, redactionRules);
    
    expect(redacted.data.financial[0].account_number).toMatch(/\*\*\*\*\d{4}/);
  });
});
```

### 10.3 Performance Optimization

#### 10.3.1 Streaming Large Exports

```typescript
async function streamLargeExport(
  exportId: string,
  chunkSizeMB: number = 10
): AsyncGenerator<Uint8Array> {
  
  const chunkSizeBytes = chunkSizeMB * 1024 * 1024;
  let offset = 0;
  
  while (true) {
    const chunk = await fetchExportChunk(exportId, offset, chunkSizeBytes);
    
    if (chunk.length === 0) break;
    
    // Compress chunk
    const compressed = await compressChunk(chunk, 'gzip');
    
    yield compressed;
    
    offset += chunk.length;
    
    // Rate limiting
    await sleep(100); // 100ms between chunks
  }
}

// Usage
for await (const chunk of streamLargeExport('exp_123', 10)) {
  await writeToFile(chunk);
}
```

#### 10.3.2 Parallel Processing

```typescript
async function parallelExport(
  userId: string,
  categories: string[]
): Promise<DataPortabilityPackage> {
  
  // Export categories in parallel
  const categoryPromises = categories.map(async (category) => {
    return await exportCategory(userId, category);
  });
  
  const categoryData = await Promise.all(categoryPromises);
  
  // Combine into single DPP
  return {
    '@context': 'https://schema.wiastandards.com/leg-008/v1',
    version: '1.0',
    data: Object.fromEntries(
      categories.map((cat, i) => [cat, categoryData[i]])
    )
  };
}
```

---

## 11. Security Considerations

### 11.1 Threat Model

| Threat | Mitigation |
|--------|------------|
| Unauthorized executor access | Multi-factor authentication, legal document verification |
| Data interception during transfer | End-to-end encryption (TLS 1.3+) |
| Man-in-the-middle attack | Certificate pinning, mutual TLS |
| Data tampering | Digital signatures, checksums |
| Replay attacks | Nonce, timestamp validation |
| Credential theft | Short-lived tokens, token rotation |
| Insider threat | Audit logging, least privilege access |
| DDoS on export API | Rate limiting, CAPTCHA |

### 11.2 Encryption Best Practices

```typescript
const securityConfig = {
  // Encryption
  encryption: {
    algorithm: 'AES-256-GCM',
    key_derivation: 'Argon2id',
    iterations: 100000,
    memory_kb: 65536,
    parallelism: 4
  },
  
  // Transport
  transport: {
    tls_version: 'TLS 1.3',
    ciphers: [
      'TLS_AES_256_GCM_SHA384',
      'TLS_CHACHA20_POLY1305_SHA256'
    ],
    certificate_pinning: true
  },
  
  // Authentication
  authentication: {
    token_lifetime_seconds: 900, // 15 minutes
    refresh_token_lifetime_days: 30,
    mfa_required: true,
    session_lifetime_hours: 24
  },
  
  // Rate limiting
  rate_limiting: {
    requests_per_hour: 100,
    export_size_limit_gb: 50,
    concurrent_exports: 3
  }
};
```

### 11.3 Audit Logging

```typescript
interface AuditLog {
  event_id: string;
  timestamp: Date;
  event_type: string;
  actor: {
    id: string;
    type: 'executor' | 'system' | 'platform';
    ip_address: string;
    user_agent: string;
  };
  resource: {
    type: 'dpp' | 'consent' | 'transfer';
    id: string;
  };
  action: string;
  result: 'success' | 'failure' | 'error';
  details: any;
  metadata: {
    session_id: string;
    request_id: string;
    duration_ms: number;
  };
}

class AuditLogger {
  async log(event: AuditLog): Promise<void> {
    // Store in tamper-proof log
    await this.appendToBlockchain(event);
    
    // Also store in database for querying
    await this.database.auditLogs.create(event);
    
    // Real-time monitoring
    await this.monitoringService.emit('audit_event', event);
  }
}
```

---

## 12. References

### 12.1 Standards and Specifications

1. **GDPR** - General Data Protection Regulation (EU) 2016/679
2. **CCPA** - California Consumer Privacy Act of 2018
3. **ISO/IEC 29100** - Privacy framework
4. **ISO/IEC 27001** - Information security management
5. **W3C ODRL** - Open Digital Rights Language
6. **JSON-LD 1.1** - JSON for Linking Data
7. **RFC 7519** - JSON Web Token (JWT)
8. **RFC 8259** - JSON Data Interchange Format

### 12.2 Related WIA Standards

- **WIA-LEG-001**: Digital Will
- **WIA-LEG-002**: Digital Executor
- **WIA-LEG-003**: Digital Asset Inventory
- **WIA-LEG-004**: Digital Memorial
- **WIA-LEG-005**: Digital Inheritance
- **WIA-LEG-006**: Digital Vault
- **WIA-LEG-007**: Digital Commemoration

### 12.3 External Resources

- GDPR Article 20: https://gdpr-info.eu/art-20-gdpr/
- Data Transfer Project: https://datatransferproject.dev/
- Schema.org: https://schema.org/
- DID Core Specification: https://www.w3.org/TR/did-core/

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
