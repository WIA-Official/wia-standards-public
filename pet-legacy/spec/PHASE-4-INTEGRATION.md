# WIA PET-LEGACY PHASE 4: INTEGRATION SPECIFICATION

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-18
**Primary Color:** #F59E0B (Amber - PET Series)

---

## Table of Contents

1. [Overview](#overview)
2. [Veterinary System Integration](#veterinary-system-integration)
3. [Pet Cemetery Integration](#pet-cemetery-integration)
4. [Social Media Integration](#social-media-integration)
5. [Cloud Storage Integration](#cloud-storage-integration)
6. [Payment Processing Integration](#payment-processing-integration)
7. [Analytics Integration](#analytics-integration)
8. [Email & Communication Integration](#email--communication-integration)
9. [AI Services Integration](#ai-services-integration)
10. [Third-Party Platform Integration](#third-party-platform-integration)

---

## 1. Overview

### 1.1 Purpose

The WIA PET-LEGACY Phase 4 specification defines integration standards for connecting the pet memorial platform with external systems, services, and platforms. These integrations enhance functionality, streamline workflows, and provide comprehensive memorial management capabilities.

### 1.2 Integration Architecture

```
┌──────────────────────────────────────────────────────────┐
│              PET-LEGACY Platform Core                    │
│  ┌────────────────────────────────────────────────────┐  │
│  │         Integration Gateway                        │  │
│  │  - Authentication                                  │  │
│  │  - Rate Limiting                                   │  │
│  │  - Protocol Translation                           │  │
│  │  - Error Handling                                 │  │
│  └─────────────┬──────────────────────────────────────┘  │
└────────────────┼─────────────────────────────────────────┘
                 │
     ┌───────────┼───────────────────────┐
     │           │                       │
     ▼           ▼                       ▼
┌─────────┐ ┌──────────┐         ┌──────────────┐
│Veterinary│ │Cemetery  │         │Social Media  │
│ Systems  │ │ Systems  │         │  Platforms   │
└─────────┘ └──────────┘         └──────────────┘
     ▼           ▼                       ▼
┌─────────┐ ┌──────────┐         ┌──────────────┐
│  Cloud  │ │ Payment  │         │   Analytics  │
│ Storage │ │Processors│         │   Services   │
└─────────┘ └──────────┘         └──────────────┘
     ▼           ▼                       ▼
┌─────────┐ ┌──────────┐         ┌──────────────┐
│  Email  │ │    AI    │         │  Third-Party │
│Services │ │ Services │         │   Platforms  │
└─────────┘ └──────────┘         └──────────────┘
```

### 1.3 Integration Types

| Type | Description | Examples |
|------|-------------|----------|
| **Data Sync** | Bidirectional data synchronization | Veterinary records, cemetery info |
| **API Integration** | Direct API calls | Payment processing, analytics |
| **Webhook** | Event-driven notifications | Social media, email triggers |
| **OAuth** | Authentication delegation | Google Drive, Facebook login |
| **Import/Export** | Batch data transfer | Photo imports, memorial exports |
| **Embedded** | Widget/iframe embedding | Memorial displays on websites |

### 1.4 Integration Security Standards

| Security Measure | Requirement | Implementation |
|------------------|-------------|----------------|
| **Authentication** | OAuth 2.0 or API Keys | Token-based auth |
| **Encryption** | TLS 1.3 minimum | All communications |
| **Data Validation** | Input sanitization | Server-side validation |
| **Rate Limiting** | Per-integration limits | API gateway enforcement |
| **Audit Logging** | All integration activities | Centralized logging |
| **Error Handling** | Graceful degradation | Fallback mechanisms |

---

## 2. Veterinary System Integration

### 2.1 Veterinary Practice Management Systems

#### 2.1.1 Supported Systems

| System | Integration Type | Features |
|--------|-----------------|----------|
| **Avimark** | API, File Export | Medical records, client info |
| **Cornerstone** | API, File Export | Patient history, appointments |
| **eVetPractice** | API | Medical records, invoices |
| **VetView** | API, HL7 | Comprehensive medical data |
| **ImproMed** | File Export | Patient records |

#### 2.1.2 Medical Record Import

```http
POST /v1/integrations/veterinary/import
Authorization: Bearer {token}
Content-Type: application/json

{
  "source": "avimark",
  "veterinaryClinic": {
    "clinicId": "clinic_5k7m9n2p4q",
    "name": "Compassionate Care Veterinary",
    "veterinarian": "Dr. Emily Chen"
  },
  "petIdentification": {
    "clinicPatientId": "PAT-12345",
    "microchipId": "985112345678901",
    "name": "Max"
  },
  "recordTypes": [
    "vaccinations",
    "medical_history",
    "prescriptions",
    "visit_notes"
  ],
  "dateRange": {
    "start": "2012-07-15",
    "end": "2024-03-10"
  },
  "linkToMemorial": "550e8400-e29b-41d4-a716-446655440000"
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "importId": "imp_7x9k2m4n6p",
    "status": "processing",
    "recordsFound": 87,
    "estimatedTime": "2 minutes",
    "preview": {
      "vaccinations": 24,
      "visits": 52,
      "prescriptions": 11
    },
    "statusUrl": "/v1/integrations/veterinary/import/imp_7x9k2m4n6p"
  }
}
```

#### 2.1.3 Medical Record Structure

```json
{
  "medicalRecords": {
    "petId": "550e8400-e29b-41d4-a716-446655440000",
    "veterinaryCare": {
      "primaryClinic": {
        "clinicId": "clinic_5k7m9n2p4q",
        "name": "Compassionate Care Veterinary",
        "address": "123 Veterinary Way, Portland, OR",
        "phone": "555-0123"
      },
      "primaryVeterinarian": {
        "name": "Dr. Emily Chen",
        "licenseNumber": "VET-OR-12345",
        "specialization": "General Practice"
      }
    },
    "vaccinations": [
      {
        "vaccineName": "Rabies",
        "date": "2023-05-20",
        "expirationDate": "2026-05-20",
        "manufacturer": "Merck",
        "lotNumber": "123456",
        "administeredBy": "Dr. Emily Chen"
      },
      {
        "vaccineName": "DHPP",
        "date": "2023-05-20",
        "expirationDate": "2024-05-20",
        "manufacturer": "Zoetis",
        "lotNumber": "789012"
      }
    ],
    "medicalHistory": [
      {
        "visitDate": "2024-01-15",
        "visitType": "wellness_exam",
        "chiefComplaint": "Annual checkup",
        "diagnosis": "Healthy",
        "treatment": "None required",
        "notes": "Patient in good health for age. Mild arthritis noted.",
        "veterinarian": "Dr. Emily Chen",
        "weight": {
          "value": 32.5,
          "unit": "kg"
        }
      },
      {
        "visitDate": "2024-02-28",
        "visitType": "urgent_care",
        "chiefComplaint": "Lethargy, decreased appetite",
        "diagnosis": "Age-related decline",
        "treatment": "Supportive care",
        "notes": "Discussed quality of life with guardian",
        "veterinarian": "Dr. Emily Chen"
      }
    ],
    "prescriptions": [
      {
        "medicationName": "Carprofen",
        "dosage": "75mg",
        "frequency": "Twice daily",
        "prescribedDate": "2023-08-10",
        "duration": "Ongoing",
        "purpose": "Arthritis pain management",
        "prescribedBy": "Dr. Emily Chen"
      }
    ],
    "diagnosticTests": [
      {
        "testDate": "2024-01-15",
        "testType": "blood_panel",
        "results": "Within normal limits",
        "laboratory": "In-house lab",
        "interpretedBy": "Dr. Emily Chen"
      }
    ],
    "surgicalHistory": [
      {
        "procedure": "Neutering",
        "date": "2012-09-15",
        "surgeon": "Dr. Robert Smith",
        "complications": "None",
        "notes": "Routine procedure, uneventful recovery"
      }
    ]
  }
}
```

#### 2.1.4 Euthanasia Record Integration

```json
{
  "euthanasiaRecord": {
    "date": "2024-03-10",
    "time": "14:30:00",
    "location": "At home",
    "veterinarian": "Dr. Emily Chen",
    "clinic": "Compassionate Care Veterinary",
    "reason": "Quality of life - age-related decline",
    "guardianConsent": {
      "obtained": true,
      "consentForm": "CONSENT-2024-0310-MAX",
      "signedBy": "Sarah Johnson",
      "date": "2024-03-10"
    },
    "procedure": {
      "sedationGiven": true,
      "sedationMedication": "Acepromazine + Butorphanol",
      "euthanasiaAgent": "Pentobarbital sodium",
      "dosage": "150mg/kg",
      "timeOfDeath": "14:45:00"
    },
    "finalDisposition": {
      "type": "cremation",
      "provider": "Rainbow Bridge Pet Memorial",
      "certificateNumber": "CERT-2024-00127"
    },
    "memorialSupport": {
      "griefResourcesProvided": true,
      "pawPrintTaken": true,
      "furClippingProvided": true
    }
  }
}
```

### 2.2 Veterinary Data Privacy

```json
{
  "privacySettings": {
    "displayMedicalRecords": {
      "enabled": false,
      "visibleTo": "guardian_only",
      "redactedFields": ["cost_information", "insurance_details"]
    },
    "shareWithFamily": {
      "enabled": false,
      "requiresGuardianApproval": true
    },
    "includeInExport": {
      "enabled": true,
      "encrypted": true
    },
    "veterinarianAccess": {
      "allowVetView": true,
      "allowVetUpdate": false,
      "notifyOnAccess": true
    }
  }
}
```

### 2.3 Veterinary Clinic Memorial Program

```json
{
  "clinicMemorialProgram": {
    "clinicId": "clinic_5k7m9n2p4q",
    "programName": "Rainbow Bridge Memorial Partnership",
    "features": {
      "automaticMemorialCreation": {
        "enabled": true,
        "trigger": "euthanasia_record",
        "requiresGuardianConsent": true,
        "templateUsed": "compassionate_care_template"
      },
      "memorialPageWidget": {
        "enabled": true,
        "displayOnClinicWebsite": true,
        "widgetUrl": "https://compassionatecare.com/memorials"
      },
      "tributeDonations": {
        "enabled": true,
        "beneficiary": "Local Animal Shelter",
        "donationUrl": "https://donate.example.org"
      }
    },
    "branding": {
      "includeClinicLogo": true,
      "customMessage": "In loving memory of our cherished patients",
      "clinicColor": "#4A90E2"
    }
  }
}
```

---

## 3. Pet Cemetery Integration

### 3.1 Cemetery Management System Integration

#### 3.1.1 Plot Management

```http
POST /v1/integrations/cemetery/reserve-plot
Authorization: Bearer {token}
Content-Type: application/json

{
  "cemeteryId": "cem_5k7m9n2p4q",
  "memorialId": "550e8400-e29b-41d4-a716-446655440000",
  "serviceType": "burial",
  "plotSelection": {
    "section": "Garden of Memories",
    "plotNumber": "A-127",
    "plotSize": "small",
    "plotType": "individual"
  },
  "intermentDetails": {
    "scheduledDate": "2024-03-12T14:00:00Z",
    "caretakerPresent": true,
    "witnesses": ["Sarah Johnson", "Michael Johnson"]
  },
  "markerDetails": {
    "markerType": "flat_bronze",
    "inscription": "Max\nForever in Our Hearts\n2012 - 2024",
    "customDesign": false
  }
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "reservationId": "res_3k5m7n9p2q",
    "plotNumber": "A-127",
    "status": "reserved",
    "intermentDate": "2024-03-12T14:00:00Z",
    "certificateNumber": "CERT-2024-00127",
    "totalCost": {
      "amount": 1500.00,
      "currency": "USD",
      "breakdown": {
        "plot": 800.00,
        "interment": 400.00,
        "marker": 300.00
      }
    },
    "documentUrls": {
      "contract": "https://cdn.cemetery.com/contracts/res_3k5m7n9p2q.pdf",
      "certificate": "https://cdn.cemetery.com/certs/CERT-2024-00127.pdf"
    }
  }
}
```

#### 3.1.2 Cremation Services

```json
{
  "cremationService": {
    "cemeteryId": "cem_5k7m9n2p4q",
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "cremationType": "individual",
    "scheduledDate": "2024-03-11T10:00:00Z",
    "witnessedCremation": false,
    "urnSelection": {
      "urnType": "wooden_personalized",
      "material": "cherry_wood",
      "size": "medium",
      "engraving": "Max - 2012-2024",
      "customEngraving": true
    },
    "ashesDisposition": {
      "type": "returned_to_family",
      "collectionDate": "2024-03-13T14:00:00Z",
      "collectedBy": "Sarah Johnson"
    },
    "memorialOptions": {
      "pawPrintKeepsake": true,
      "furClipping": true,
      "memorialCandle": true
    },
    "certificateNumber": "CREM-2024-00089"
  }
}
```

#### 3.1.3 Virtual Cemetery Integration

```json
{
  "virtualCemetery": {
    "enabled": true,
    "plotLocation": {
      "section": "Garden of Memories",
      "plotNumber": "A-127",
      "coordinates": {
        "latitude": 45.523064,
        "longitude": -122.676483
      }
    },
    "virtualFeatures": {
      "3dPlotView": {
        "enabled": true,
        "url": "https://virtual.cemetery.com/plot/A-127"
      },
      "virtualVisitation": {
        "enabled": true,
        "activities": [
          "place_virtual_flowers",
          "light_candle",
          "leave_message"
        ]
      },
      "qrCodeMarker": {
        "enabled": true,
        "qrCodeUrl": "https://cdn.cemetery.com/qr/A-127.png",
        "linkedUrl": "https://petlegacy.wia.org/memorial/550e8400"
      }
    },
    "visitationLog": {
      "trackVisits": true,
      "shareWithGuardian": true,
      "privacy": "guardian_only"
    }
  }
}
```

### 3.2 Cemetery Event Scheduling

```json
{
  "memorialService": {
    "cemeteryId": "cem_5k7m9n2p4q",
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "serviceType": "memorial_ceremony",
    "scheduledDate": "2024-03-12T14:00:00Z",
    "duration": 60,
    "location": {
      "name": "Rainbow Bridge Chapel",
      "capacity": 50,
      "amenities": ["audio_visual", "seating", "climate_controlled"]
    },
    "officiant": {
      "name": "Reverend Sarah Miller",
      "type": "interfaith_minister"
    },
    "attendees": {
      "expectedCount": 25,
      "rsvpRequired": true,
      "rsvpDeadline": "2024-03-10T12:00:00Z"
    },
    "program": {
      "eulogy": true,
      "photoSlideshow": true,
      "musicSelection": ["Amazing Grace", "Over the Rainbow"],
      "candleLighting": true
    },
    "liveStreaming": {
      "enabled": true,
      "platform": "zoom",
      "recordingAvailable": true,
      "password": "protected"
    }
  }
}
```

---

## 4. Social Media Integration

### 4.1 Facebook Integration

#### 4.1.1 Facebook Login (OAuth)

```http
GET /v1/auth/facebook
Response: Redirect to Facebook OAuth
```

**Facebook OAuth Flow:**
```
1. User clicks "Login with Facebook"
2. Redirect to Facebook authorization
3. User grants permissions
4. Facebook redirects with auth code
5. Exchange code for access token
6. Create/link user account
```

#### 4.1.2 Share Memorial to Facebook

```http
POST /v1/integrations/facebook/share
Authorization: Bearer {token}
Content-Type: application/json

{
  "memorialId": "550e8400-e29b-41d4-a716-446655440000",
  "shareType": "post",
  "content": {
    "message": "Remembering my beloved Max. Visit his memorial to share your memories.",
    "link": "https://petlegacy.wia.org/memorial/550e8400",
    "privacy": "public"
  },
  "media": {
    "type": "photo",
    "url": "https://cdn.petlegacy.wia.org/share/550e8400-fb.jpg"
  }
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "postId": "10159123456789012",
    "postedAt": "2024-12-18T19:00:00Z",
    "postUrl": "https://www.facebook.com/username/posts/10159123456789012",
    "engagement": {
      "likes": 0,
      "comments": 0,
      "shares": 0
    }
  }
}
```

### 4.2 Instagram Integration

```json
{
  "instagramShare": {
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "shareType": "story",
    "content": {
      "mediaType": "photo",
      "mediaUrl": "https://cdn.petlegacy.wia.org/share/550e8400-ig.jpg",
      "caption": "Forever in our hearts 🐾 #PetMemorial #RainbowBridge",
      "link": "https://petlegacy.wia.org/memorial/550e8400"
    },
    "storySettings": {
      "duration": 24,
      "stickers": [
        {
          "type": "link",
          "url": "https://petlegacy.wia.org/memorial/550e8400",
          "text": "View Memorial"
        }
      ]
    }
  }
}
```

### 4.3 Twitter/X Integration

```json
{
  "twitterShare": {
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "content": {
      "text": "Remembering Max, my beloved Golden Retriever who crossed the rainbow bridge. Visit his memorial: https://pet.wia/m/abc123 #PetMemorial #DogsOfTwitter",
      "media": [
        {
          "type": "photo",
          "url": "https://cdn.petlegacy.wia.org/share/550e8400-tw.jpg",
          "altText": "Max, a Golden Retriever, smiling at the camera"
        }
      ]
    },
    "privacy": "public",
    "allowReplies": true
  }
}
```

### 4.4 Social Media Analytics

```json
{
  "socialAnalytics": {
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "period": {
      "start": "2024-03-15",
      "end": "2024-12-18"
    },
    "platforms": {
      "facebook": {
        "shares": 45,
        "reactions": 342,
        "comments": 67,
        "reach": 8920,
        "clicks": 234
      },
      "instagram": {
        "shares": 28,
        "likes": 489,
        "comments": 52,
        "reach": 6730
      },
      "twitter": {
        "retweets": 12,
        "likes": 98,
        "replies": 15,
        "impressions": 3450
      }
    },
    "totalEngagement": 1170,
    "totalReach": 19100
  }
}
```

---

## 5. Cloud Storage Integration

### 5.1 Google Drive Integration

```http
POST /v1/integrations/google-drive/backup
Authorization: Bearer {token}
Content-Type: application/json

{
  "memorialId": "550e8400-e29b-41d4-a716-446655440000",
  "backupOptions": {
    "includeMedia": true,
    "includeTimeline": true,
    "includeComments": true,
    "organizationStructure": "by_year"
  },
  "googleDrive": {
    "folderId": "1AbC2dEf3GhI4jKl5MnO6pQr7StU8vWx",
    "folderName": "Pet Legacy - Max",
    "createSubfolders": true
  },
  "schedule": {
    "frequency": "weekly",
    "dayOfWeek": "sunday",
    "time": "02:00:00"
  }
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "backupId": "bkp_5k7m9n2p4q",
    "status": "scheduled",
    "nextBackup": "2024-12-22T02:00:00Z",
    "googleDriveFolder": {
      "folderId": "1AbC2dEf3GhI4jKl5MnO6pQr7StU8vWx",
      "folderUrl": "https://drive.google.com/drive/folders/1AbC2dEf3GhI4jKl5MnO6pQr7StU8vWx"
    }
  }
}
```

### 5.2 Dropbox Integration

```json
{
  "dropboxBackup": {
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "dropboxPath": "/Apps/PetLegacy/Max",
    "syncOptions": {
      "bidirectional": false,
      "autoSync": true,
      "conflictResolution": "server_wins"
    },
    "fileOrganization": {
      "structure": "by_type",
      "folders": {
        "photos": "/Photos",
        "videos": "/Videos",
        "documents": "/Documents",
        "timeline": "/Timeline"
      }
    }
  }
}
```

### 5.3 iCloud Integration

```json
{
  "iCloudBackup": {
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "iCloudDrive": {
      "enabled": true,
      "folder": "PetLegacy/Max"
    },
    "iCloudPhotos": {
      "enabled": true,
      "album": "Max Memorial",
      "sharedAlbum": true,
      "subscribers": [
        "michael.j@icloud.com"
      ]
    },
    "syncFrequency": "real_time"
  }
}
```

---

## 6. Payment Processing Integration

### 6.1 Stripe Integration

#### 6.1.1 Payment Processing

```http
POST /v1/integrations/stripe/create-payment
Authorization: Bearer {token}
Content-Type: application/json

{
  "memorialId": "550e8400-e29b-41d4-a716-446655440000",
  "paymentType": "memorial_upgrade",
  "items": [
    {
      "sku": "memorial_pro_annual",
      "description": "Pro Memorial (Annual)",
      "amount": 9900,
      "quantity": 1
    },
    {
      "sku": "storage_addon_50gb",
      "description": "Additional 50GB Storage",
      "amount": 1900,
      "quantity": 1
    }
  ],
  "currency": "USD",
  "customer": {
    "email": "sarah.j@example.com",
    "name": "Sarah Johnson"
  },
  "successUrl": "https://petlegacy.wia.org/payment/success",
  "cancelUrl": "https://petlegacy.wia.org/payment/cancel"
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "paymentIntentId": "pi_3k5m7n9p2q",
    "clientSecret": "pi_3k5m7n9p2q_secret_abc123",
    "amount": 11800,
    "currency": "USD",
    "status": "requires_payment_method",
    "checkoutUrl": "https://checkout.stripe.com/pay/cs_test_abc123"
  }
}
```

#### 6.1.2 Subscription Management

```json
{
  "subscription": {
    "subscriptionId": "sub_5k7m9n2p4q",
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "plan": {
      "id": "plan_pro_annual",
      "name": "Pro Memorial",
      "interval": "year",
      "amount": 9900,
      "currency": "USD"
    },
    "status": "active",
    "currentPeriod": {
      "start": "2024-03-15T00:00:00Z",
      "end": "2025-03-15T00:00:00Z"
    },
    "cancelAtPeriodEnd": false,
    "trialEnd": null,
    "defaultPaymentMethod": {
      "type": "card",
      "last4": "4242",
      "brand": "visa",
      "expMonth": 12,
      "expYear": 2026
    }
  }
}
```

### 6.2 PayPal Integration

```json
{
  "paypalPayment": {
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "orderId": "7X987654321",
    "orderType": "one_time_payment",
    "items": [
      {
        "name": "Memorial Pro Upgrade",
        "unitAmount": 99.00,
        "quantity": 1
      }
    ],
    "totalAmount": 99.00,
    "currency": "USD",
    "status": "completed",
    "payerInfo": {
      "email": "sarah.j@example.com",
      "name": "Sarah Johnson"
    },
    "transactionId": "8XY12345678",
    "completedAt": "2024-12-18T19:30:00Z"
  }
}
```

### 6.3 Donation Processing

```json
{
  "tributeDonation": {
    "memorialId": "550e8400-e29b-41d4-a716-446655440000",
    "donationType": "in_memory_of",
    "amount": 50.00,
    "currency": "USD",
    "beneficiary": {
      "type": "charity",
      "name": "Local Animal Shelter",
      "ein": "12-3456789"
    },
    "donor": {
      "name": "Jennifer Smith",
      "email": "jennifer@example.com",
      "isAnonymous": false,
      "includeMessage": true,
      "message": "In loving memory of Max"
    },
    "paymentMethod": "stripe",
    "transactionId": "txn_9k2m5n7p8q",
    "taxDeductible": true,
    "receiptUrl": "https://donations.example.org/receipt/txn_9k2m5n7p8q"
  }
}
```

---

## 7. Analytics Integration

### 7.1 Google Analytics Integration

```javascript
// Google Analytics 4 Configuration
{
  "ga4": {
    "measurementId": "G-XXXXXXXXXX",
    "events": {
      "memorial_created": {
        "category": "engagement",
        "action": "create_memorial",
        "label": "memorial_id"
      },
      "media_uploaded": {
        "category": "content",
        "action": "upload_media",
        "label": "media_type"
      },
      "memorial_viewed": {
        "category": "engagement",
        "action": "view_memorial",
        "label": "memorial_id"
      },
      "tribute_posted": {
        "category": "engagement",
        "action": "post_tribute",
        "label": "memorial_id"
      }
    },
    "customDimensions": {
      "pet_species": "dimension1",
      "memorial_age": "dimension2",
      "user_role": "dimension3"
    },
    "customMetrics": {
      "photo_count": "metric1",
      "video_count": "metric2",
      "tribute_count": "metric3"
    }
  }
}
```

### 7.2 Mixpanel Integration

```json
{
  "mixpanel": {
    "projectToken": "abc123def456",
    "trackingEvents": [
      {
        "event": "Memorial Created",
        "properties": {
          "memorial_id": "550e8400",
          "pet_species": "dog",
          "pet_breed": "Golden Retriever",
          "memorial_type": "pro"
        }
      },
      {
        "event": "Media Uploaded",
        "properties": {
          "memorial_id": "550e8400",
          "media_type": "photo",
          "file_size": 3457280,
          "upload_source": "mobile_app"
        }
      }
    ],
    "userProperties": {
      "memorials_created": 1,
      "total_photos_uploaded": 324,
      "subscription_tier": "pro",
      "signup_date": "2024-03-15"
    },
    "cohortAnalysis": {
      "enabled": true,
      "cohortBy": "signup_month"
    }
  }
}
```

### 7.3 Segment Integration

```json
{
  "segment": {
    "writeKey": "xyz789abc123",
    "integrations": {
      "google_analytics": true,
      "mixpanel": true,
      "facebook_pixel": true,
      "intercom": true
    },
    "track": {
      "event": "Memorial Shared",
      "properties": {
        "memorial_id": "550e8400",
        "share_platform": "facebook",
        "share_type": "public_post"
      },
      "context": {
        "ip": "192.168.1.100",
        "userAgent": "Mozilla/5.0..."
      }
    },
    "identify": {
      "userId": "660e8400",
      "traits": {
        "email": "sarah.j@example.com",
        "name": "Sarah Johnson",
        "plan": "pro",
        "memorials_count": 1
      }
    }
  }
}
```

---

## 8. Email & Communication Integration

### 8.1 SendGrid Integration

```http
POST /v1/integrations/sendgrid/send
Authorization: Bearer {token}
Content-Type: application/json

{
  "templateId": "memorial_anniversary",
  "to": {
    "email": "sarah.j@example.com",
    "name": "Sarah Johnson"
  },
  "dynamicData": {
    "petName": "Max",
    "anniversaryType": "passing",
    "yearsSince": 1,
    "memorialUrl": "https://petlegacy.wia.org/memorial/550e8400",
    "photoUrl": "https://cdn.petlegacy.wia.org/photos/max-profile.jpg"
  },
  "categories": ["memorial", "anniversary"],
  "sendAt": "2025-03-10T09:00:00Z"
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "messageId": "msg_5k7m9n2p4q",
    "status": "scheduled",
    "scheduledFor": "2025-03-10T09:00:00Z"
  }
}
```

### 8.2 Twilio Integration (SMS/Phone)

```json
{
  "twilioSms": {
    "to": "+15551234567",
    "from": "+15559876543",
    "body": "A new tribute has been posted to Max's memorial. View it here: https://pet.wia/m/abc123",
    "mediaUrl": null,
    "statusCallback": "https://api.petlegacy.wia.org/v1/webhooks/twilio/status"
  }
}
```

### 8.3 Mailchimp Integration

```json
{
  "mailchimp": {
    "listId": "abc123def456",
    "campaign": {
      "type": "regular",
      "subject": "Memorial Anniversary: Remembering Max",
      "fromName": "Pet Legacy",
      "replyTo": "support@petlegacy.wia.org",
      "template": {
        "id": "memorial_anniversary",
        "sections": {
          "pet_name": "Max",
          "memorial_url": "https://petlegacy.wia.org/memorial/550e8400",
          "anniversary_message": "It's been one year since Max crossed the rainbow bridge..."
        }
      }
    },
    "segment": {
      "conditions": [
        {
          "field": "memorial_id",
          "operator": "is",
          "value": "550e8400"
        },
        {
          "field": "role",
          "operator": "in",
          "value": ["guardian", "contributor"]
        }
      ]
    }
  }
}
```

---

## 9. AI Services Integration

### 9.1 OpenAI Integration

```http
POST /v1/integrations/openai/generate-summary
Authorization: Bearer {token}
Content-Type: application/json

{
  "memorialId": "550e8400-e29b-41d4-a716-446655440000",
  "model": "gpt-4",
  "prompt": {
    "type": "memorial_summary",
    "tone": "warm_and_loving",
    "length": "medium",
    "context": {
      "petName": "Max",
      "species": "dog",
      "breed": "Golden Retriever",
      "lifespan": "2012-2024",
      "personality": ["loyal", "gentle", "playful"],
      "favoriteActivities": ["swimming", "fetch", "hiking"],
      "timelineEvents": 52,
      "photos": 324
    }
  }
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "summary": "Max was more than just a pet; he was a cherished member of the family who filled every day with unconditional love and boundless joy. Born in 2012, this gentle Golden Retriever spent 11 beautiful years creating memories that will last a lifetime. Whether he was diving into the water for a swim, chasing after his favorite tennis ball, or exploring new trails on family hikes, Max approached every moment with enthusiasm and a wagging tail. His loyal and playful spirit touched everyone he met, leaving paw prints on countless hearts. The 324 photos and 52 precious moments captured in his memorial tell the story of a life well-lived and deeply loved. Though Max crossed the rainbow bridge in 2024, his memory continues to bring comfort and smiles to all who knew him.",
    "wordCount": 142,
    "tokensUsed": 189,
    "generatedAt": "2024-12-18T20:00:00Z"
  }
}
```

### 9.2 Computer Vision Integration

```json
{
  "visionAI": {
    "provider": "google_cloud_vision",
    "assetId": "770e8400-e29b-41d4-a716-446655440002",
    "analysis": {
      "labels": [
        {"description": "Dog", "score": 0.99},
        {"description": "Golden Retriever", "score": 0.97},
        {"description": "Beach", "score": 0.95},
        {"description": "Ocean", "score": 0.93},
        {"description": "Happy", "score": 0.89}
      ],
      "faces": {
        "detected": 1,
        "emotions": {
          "joy": 0.92,
          "playful": 0.87
        }
      },
      "objects": [
        {"name": "Animal", "score": 0.98},
        {"name": "Water", "score": 0.94},
        {"name": "Sand", "score": 0.91}
      ],
      "colors": [
        {"color": "#F4D03F", "percentage": 35},
        {"color": "#3498DB", "percentage": 30},
        {"color": "#F5B7B1", "percentage": 20}
      ],
      "safeSearch": {
        "adult": "VERY_UNLIKELY",
        "violence": "VERY_UNLIKELY",
        "racy": "UNLIKELY"
      }
    },
    "suggestedTags": ["beach", "ocean", "golden_retriever", "happy", "summer"],
    "suggestedCaption": "Max enjoying a beautiful day at the beach"
  }
}
```

### 9.3 Video Processing Integration

```json
{
  "videoProcessing": {
    "provider": "cloudinary",
    "assetId": "770e8400-e29b-41d4-a716-446655440015",
    "operations": [
      {
        "type": "thumbnail_generation",
        "timestamps": [0, 5, 10, 15, 20],
        "outputFormat": "jpg",
        "quality": "auto:best"
      },
      {
        "type": "transcoding",
        "formats": [
          {
            "codec": "h264",
            "resolution": "1080p",
            "bitrate": "5000k"
          },
          {
            "codec": "h264",
            "resolution": "720p",
            "bitrate": "2500k"
          },
          {
            "codec": "h264",
            "resolution": "480p",
            "bitrate": "1000k"
          }
        ]
      },
      {
        "type": "scene_detection",
        "confidenceThreshold": 0.8
      },
      {
        "type": "highlight_reel",
        "duration": 30,
        "music": "gentle_piano",
        "transitions": "fade"
      }
    ],
    "outputUrls": {
      "1080p": "https://cdn.cloudinary.com/video/max-1080p.mp4",
      "720p": "https://cdn.cloudinary.com/video/max-720p.mp4",
      "480p": "https://cdn.cloudinary.com/video/max-480p.mp4",
      "thumbnails": [
        "https://cdn.cloudinary.com/thumbs/max-00.jpg",
        "https://cdn.cloudinary.com/thumbs/max-05.jpg"
      ],
      "highlightReel": "https://cdn.cloudinary.com/video/max-highlights.mp4"
    }
  }
}
```

---

## 10. Third-Party Platform Integration

### 10.1 Zapier Integration

```json
{
  "zapierIntegration": {
    "triggers": [
      {
        "event": "memorial_created",
        "zapUrl": "https://zapier.com/hooks/catch/123456/abc123/",
        "actions": [
          "Send notification to Slack",
          "Add row to Google Sheets",
          "Create Trello card"
        ]
      },
      {
        "event": "media_uploaded",
        "zapUrl": "https://zapier.com/hooks/catch/123456/def456/",
        "actions": [
          "Backup to Dropbox",
          "Send email notification"
        ]
      }
    ],
    "actions": [
      {
        "trigger": "new_row_google_sheets",
        "action": "create_timeline_event",
        "mapping": {
          "date": "{{column_a}}",
          "title": "{{column_b}}",
          "description": "{{column_c}}"
        }
      }
    ]
  }
}
```

### 10.2 IFTTT Integration

```json
{
  "ifttt": {
    "applet": {
      "name": "Backup memorial photos to Google Photos",
      "trigger": {
        "channel": "pet_legacy",
        "event": "media_uploaded",
        "filter": {
          "type": "photo"
        }
      },
      "action": {
        "channel": "google_photos",
        "command": "upload_photo",
        "parameters": {
          "album": "Max Memorial",
          "caption": "{{title}} - {{date}}"
        }
      }
    }
  }
}
```

### 10.3 WordPress Integration

```json
{
  "wordpressPlugin": {
    "pluginName": "WIA Pet Legacy",
    "version": "1.0.0",
    "features": {
      "shortcode": {
        "enabled": true,
        "syntax": "[pet_memorial id='550e8400' display='timeline']",
        "attributes": {
          "id": "required",
          "display": "timeline|gallery|tribute_wall",
          "theme": "light|dark|custom",
          "maxItems": "integer"
        }
      },
      "widget": {
        "enabled": true,
        "widgetAreas": ["sidebar", "footer"],
        "configuration": {
          "memorialId": "550e8400",
          "displayMode": "featured_photo",
          "showTributeCount": true
        }
      },
      "blockEditor": {
        "enabled": true,
        "blockName": "wia/pet-memorial",
        "category": "widgets"
      }
    }
  }
}
```

### 10.4 Mobile App Integration (iOS/Android)

```json
{
  "mobileSDK": {
    "platform": "ios",
    "version": "1.0.0",
    "features": {
      "authentication": {
        "oauth": true,
        "biometric": true,
        "appleSignIn": true
      },
      "mediaUpload": {
        "photoLibrary": true,
        "camera": true,
        "livePhoto": true,
        "heicSupport": true,
        "backgroundUpload": true
      },
      "pushNotifications": {
        "enabled": true,
        "types": [
          "new_tribute",
          "memorial_anniversary",
          "family_member_activity"
        ]
      },
      "widgets": {
        "homeScreen": true,
        "lockScreen": true,
        "appleWatch": true
      },
      "shortcuts": {
        "siriIntegration": true,
        "quickActions": ["upload_photo", "view_memorial", "light_candle"]
      }
    },
    "deepLinking": {
      "scheme": "petlegacy://",
      "universalLinks": true,
      "associatedDomains": ["petlegacy.wia.org"]
    }
  }
}
```

---

## Appendix A: Integration Testing Checklist

### Authentication & Authorization
- [ ] OAuth 2.0 flow completes successfully
- [ ] API keys are validated correctly
- [ ] Token refresh works properly
- [ ] Permission scopes are enforced
- [ ] Rate limiting functions as expected

### Data Integration
- [ ] Data sync is bidirectional where applicable
- [ ] Conflict resolution works correctly
- [ ] Data validation prevents invalid inputs
- [ ] Error handling is graceful
- [ ] Retry logic handles failures

### Webhook Integration
- [ ] Webhooks are delivered reliably
- [ ] Signature verification works
- [ ] Retry logic handles failures
- [ ] Events are deduplicated
- [ ] Payload format is correct

### Media Integration
- [ ] File uploads complete successfully
- [ ] Media processing works correctly
- [ ] Thumbnails generate properly
- [ ] CDN distribution functions
- [ ] Large files are handled

### Payment Integration
- [ ] Payments process successfully
- [ ] Refunds work correctly
- [ ] Subscriptions renew properly
- [ ] Failed payments are handled
- [ ] Receipts are generated

---

## Appendix B: Integration Rate Limits

| Integration | Requests/Minute | Requests/Hour | Requests/Day |
|-------------|-----------------|---------------|--------------|
| Veterinary Systems | 60 | 1,000 | 10,000 |
| Cemetery Systems | 30 | 500 | 5,000 |
| Social Media | 100 | 3,000 | 25,000 |
| Cloud Storage | 120 | 5,000 | 50,000 |
| Payment Processing | 300 | 10,000 | 100,000 |
| Analytics | 600 | 20,000 | 200,000 |
| Email Services | 1,000 | 30,000 | 250,000 |
| AI Services | 60 | 1,000 | 10,000 |

---

## Appendix C: Integration SLA Requirements

| Service Type | Uptime | Response Time | Support Level |
|--------------|--------|---------------|---------------|
| Critical (Payment, Auth) | 99.9% | < 200ms | 24/7 |
| High Priority (Media, API) | 99.5% | < 500ms | Business hours |
| Standard (Analytics, Social) | 99.0% | < 1000ms | Email support |
| Background (Backup, Sync) | 95.0% | < 5000ms | Best effort |

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
