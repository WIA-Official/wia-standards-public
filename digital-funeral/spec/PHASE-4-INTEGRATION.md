# WIA Digital Funeral Standard - Phase 4: Integration Specification

**Version:** 1.0.0
**Status:** Draft
**Date:** 2025-12-18
**Primary Color:** #64748B (Slate)
**Series:** Digital Death Services

---

## Table of Contents

1. [Introduction](#introduction)
2. [System Architecture](#system-architecture)
3. [Third-Party Integrations](#third-party-integrations)
4. [Platform Integrations](#platform-integrations)
5. [Payment Processing Integration](#payment-processing-integration)
6. [Streaming Service Integration](#streaming-service-integration)
7. [Cemetery Management Integration](#cemetery-management-integration)
8. [Notification Service Integration](#notification-service-integration)
9. [Analytics and Reporting](#analytics-and-reporting)
10. [Security and Compliance](#security-and-compliance)
11. [Deployment Patterns](#deployment-patterns)
12. [Testing and Quality Assurance](#testing-and-quality-assurance)
13. [Migration Strategies](#migration-strategies)
14. [Implementation Examples](#implementation-examples)

---

## 1. Introduction

### 1.1 Purpose

This specification defines integration patterns, best practices, and implementation guidelines for incorporating the WIA Digital Funeral Standard into existing funeral service platforms, third-party applications, and enterprise systems. It provides comprehensive guidance for developers and system architects.

### 1.2 Integration Scenarios

| Scenario | Description | Complexity |
|----------|-------------|------------|
| Greenfield Implementation | New platform built on WIA standard | Low |
| Brownfield Integration | Adding WIA to existing system | Medium |
| Third-Party Integration | External app consuming WIA APIs | Low |
| Enterprise Migration | Legacy system replacement | High |
| Hybrid Deployment | Gradual migration strategy | Medium |

### 1.3 Integration Architecture Layers

```
┌─────────────────────────────────────────────────┐
│          Presentation Layer                     │
│  (Web, Mobile, Digital Signage, Kiosks)        │
└────────────────┬────────────────────────────────┘
                 │
┌────────────────▼────────────────────────────────┐
│          API Gateway Layer                      │
│  (Authentication, Rate Limiting, Routing)       │
└────────────────┬────────────────────────────────┘
                 │
┌────────────────▼────────────────────────────────┐
│          Business Logic Layer                   │
│  (Funeral Management, Memorial Services)        │
└────────────────┬────────────────────────────────┘
                 │
┌────────────────▼────────────────────────────────┐
│          Data Access Layer                      │
│  (WIA Standard Data Models)                     │
└────────────────┬────────────────────────────────┘
                 │
┌────────────────▼────────────────────────────────┐
│          Integration Layer                      │
│  (Payment, Streaming, Cemetery Systems)         │
└─────────────────────────────────────────────────┘
```

---

## 2. System Architecture

### 2.1 Reference Architecture

```
                    ┌──────────────┐
                    │   Clients    │
                    │ Web | Mobile │
                    └───────┬──────┘
                            │
                    ┌───────▼──────────┐
                    │   Load Balancer  │
                    └───────┬──────────┘
                            │
         ┌──────────────────┼──────────────────┐
         │                  │                  │
    ┌────▼────┐      ┌─────▼─────┐     ┌─────▼─────┐
    │ API     │      │ WebSocket │     │  Stream   │
    │ Server  │      │  Server   │     │  Server   │
    └────┬────┘      └─────┬─────┘     └─────┬─────┘
         │                  │                  │
         └──────────────────┼──────────────────┘
                            │
                    ┌───────▼──────────┐
                    │  Message Queue   │
                    │  (Redis/RabbitMQ)│
                    └───────┬──────────┘
                            │
         ┌──────────────────┼──────────────────┐
         │                  │                  │
    ┌────▼────┐      ┌─────▼─────┐     ┌─────▼─────┐
    │Database │      │  Storage  │     │   Cache   │
    │(Primary)│      │   (S3)    │     │  (Redis)  │
    └─────────┘      └───────────┘     └───────────┘
```

### 2.2 Component Responsibilities

| Component | Responsibilities | Technologies |
|-----------|-----------------|--------------|
| API Gateway | Auth, routing, rate limiting | Kong, AWS API Gateway, NGINX |
| Application Server | Business logic, data validation | Node.js, Python, Java, .NET |
| Database | Persistent storage | PostgreSQL, MySQL, MongoDB |
| Cache | Session data, frequent queries | Redis, Memcached |
| Message Queue | Async processing, webhooks | RabbitMQ, AWS SQS, Redis |
| Object Storage | Media files, recordings | AWS S3, Google Cloud Storage |
| CDN | Static assets, video streaming | CloudFlare, AWS CloudFront |
| Search Engine | Full-text search, analytics | Elasticsearch, Algolia |

### 2.3 Scalability Considerations

| Aspect | Strategy | Target Metrics |
|--------|----------|----------------|
| Horizontal Scaling | Auto-scaling groups | 1000 req/sec per instance |
| Database Scaling | Read replicas, sharding | < 100ms query time |
| Cache Strategy | Multi-tier caching | 95%+ cache hit rate |
| CDN Distribution | Global edge locations | < 50ms latency |
| Stream Delivery | Adaptive bitrate, edge servers | 99.9% uptime |

---

## 3. Third-Party Integrations

### 3.1 Funeral Home Management Systems

**Integration Type:** Bidirectional sync

```javascript
// Example: Integrating with FuneralOne
class FuneralOneIntegration {
  constructor(apiKey, apiSecret) {
    this.apiKey = apiKey;
    this.apiSecret = apiSecret;
    this.baseUrl = 'https://api.funeralone.com/v1';
  }

  async syncDeceasedPerson(wiaDeceasedId) {
    // Fetch from WIA system
    const wiaData = await this.getWIADeceasedPerson(wiaDeceasedId);

    // Transform to FuneralOne format
    const funeralOneData = {
      firstName: wiaData.legalName.firstName,
      lastName: wiaData.legalName.lastName,
      dateOfBirth: wiaData.dateOfBirth,
      dateOfDeath: wiaData.dateOfDeath,
      obituary: wiaData.biography.full,
      // Additional mappings...
    };

    // Sync to FuneralOne
    const response = await fetch(`${this.baseUrl}/cases`, {
      method: 'POST',
      headers: {
        'X-API-Key': this.apiKey,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(funeralOneData)
    });

    return response.json();
  }

  async syncService(wiaServiceId) {
    const wiaService = await this.getWIAService(wiaServiceId);

    const funeralOneService = {
      caseId: wiaService.externalIds?.funeralOne,
      serviceType: this.mapServiceType(wiaService.serviceType),
      serviceDate: wiaService.startDateTime,
      location: {
        name: wiaService.venue.name,
        address: wiaService.venue.address
      }
    };

    await fetch(`${this.baseUrl}/services`, {
      method: 'POST',
      headers: {
        'X-API-Key': this.apiKey,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(funeralOneService)
    });
  }

  mapServiceType(wiaType) {
    const mapping = {
      'funeral': 'FUNERAL_SERVICE',
      'memorial': 'MEMORIAL_SERVICE',
      'viewing': 'VISITATION',
      'graveside': 'GRAVESIDE_SERVICE'
    };
    return mapping[wiaType] || 'OTHER';
  }
}
```

### 3.2 Social Media Integration

**Platforms:** Facebook, Instagram, Twitter/X

```javascript
class SocialMediaIntegration {
  async shareObituary(deceasedId, platforms) {
    const deceased = await this.getDeceasedPerson(deceasedId);
    const obituary = await this.getObituary(deceasedId);

    const shareData = {
      title: obituary.title,
      description: obituary.content.text.substring(0, 200) + '...',
      image: deceased.photos.find(p => p.isPrimary)?.url,
      url: `https://memorial.example.com/tribute/${deceasedId}`
    };

    if (platforms.includes('facebook')) {
      await this.shareToFacebook(shareData);
    }

    if (platforms.includes('twitter')) {
      await this.shareToTwitter(shareData);
    }

    if (platforms.includes('instagram')) {
      await this.shareToInstagram(shareData);
    }
  }

  async shareToFacebook(data) {
    // Using Facebook Graph API
    const response = await fetch(
      `https://graph.facebook.com/v18.0/me/feed`,
      {
        method: 'POST',
        headers: { 'Authorization': `Bearer ${this.facebookToken}` },
        body: JSON.stringify({
          message: data.description,
          link: data.url
        })
      }
    );
    return response.json();
  }

  async liveStreamToFacebook(serviceId, streamUrl) {
    // Create Facebook Live video
    const response = await fetch(
      `https://graph.facebook.com/v18.0/me/live_videos`,
      {
        method: 'POST',
        headers: { 'Authorization': `Bearer ${this.facebookToken}` },
        body: JSON.stringify({
          title: 'Memorial Service Live Stream',
          description: 'Join us for a celebration of life',
          status: 'LIVE_NOW',
          stream_url: streamUrl
        })
      }
    );
    return response.json();
  }
}
```

### 3.3 Calendar Integration

**Platforms:** Google Calendar, Outlook, Apple Calendar

```javascript
class CalendarIntegration {
  createICSFile(service) {
    const ics = `BEGIN:VCALENDAR
VERSION:2.0
PRODID:-//WIA Funeral Services//EN
BEGIN:VEVENT
UID:${service.id}@funeral-service.example.com
DTSTAMP:${this.formatICSDate(new Date())}
DTSTART:${this.formatICSDate(new Date(service.startDateTime))}
DTEND:${this.formatICSDate(new Date(service.endDateTime))}
SUMMARY:${this.escapeICS(service.serviceType)} for ${this.escapeICS(service.deceased.name)}
DESCRIPTION:${this.escapeICS(service.specialInstructions || '')}
LOCATION:${this.escapeICS(service.venue.name)}, ${this.escapeICS(service.venue.address.street)}
URL:${service.virtualService?.viewerUrl || ''}
BEGIN:VALARM
TRIGGER:-PT24H
ACTION:DISPLAY
DESCRIPTION:Reminder: Service tomorrow
END:VALARM
END:VEVENT
END:VCALENDAR`;

    return ics;
  }

  async addToGoogleCalendar(service, userToken) {
    const event = {
      summary: `${service.serviceType} for ${service.deceased.name}`,
      description: service.specialInstructions,
      start: {
        dateTime: service.startDateTime,
        timeZone: service.timezone
      },
      end: {
        dateTime: service.endDateTime,
        timeZone: service.timezone
      },
      location: `${service.venue.name}, ${service.venue.address.street}`,
      reminders: {
        useDefault: false,
        overrides: [
          { method: 'email', minutes: 24 * 60 },
          { method: 'popup', minutes: 60 }
        ]
      }
    };

    const response = await fetch(
      'https://www.googleapis.com/calendar/v3/calendars/primary/events',
      {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${userToken}`,
          'Content-Type': 'application/json'
        },
        body: JSON.stringify(event)
      }
    );

    return response.json();
  }

  formatICSDate(date) {
    return date.toISOString().replace(/[-:]/g, '').split('.')[0] + 'Z';
  }

  escapeICS(text) {
    return text.replace(/[,;\\]/g, '\\$&').replace(/\n/g, '\\n');
  }
}
```

---

## 4. Platform Integrations

### 4.1 E-commerce Platforms

**Integration:** Floral arrangements, memorial products

```javascript
class ShopifyIntegration {
  constructor(shopDomain, accessToken) {
    this.shopDomain = shopDomain;
    this.accessToken = accessToken;
    this.apiVersion = '2024-01';
  }

  async createMemorialProduct(deceased) {
    const product = {
      title: `Memorial for ${deceased.legalName.firstName} ${deceased.legalName.lastName}`,
      body_html: deceased.biography.short,
      vendor: 'Memorial Services',
      product_type: 'Memorial Product',
      variants: [
        {
          title: 'Digital Guestbook',
          price: '29.99',
          sku: `MEM-GB-${deceased.id}`
        },
        {
          title: 'Video Tribute',
          price: '99.99',
          sku: `MEM-VT-${deceased.id}`
        }
      ],
      images: deceased.photos.map(photo => ({
        src: photo.url
      }))
    };

    const response = await fetch(
      `https://${this.shopDomain}/admin/api/${this.apiVersion}/products.json`,
      {
        method: 'POST',
        headers: {
          'X-Shopify-Access-Token': this.accessToken,
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({ product })
      }
    );

    return response.json();
  }

  async createFloralOrder(serviceId, floralData) {
    // Create order in Shopify for floral arrangement
    const order = {
      line_items: [
        {
          title: floralData.type,
          price: floralData.cost,
          quantity: 1,
          custom_attributes: [
            { key: 'service_id', value: serviceId },
            { key: 'card_message', value: floralData.cardMessage },
            { key: 'delivery_location', value: floralData.deliveryLocation }
          ]
        }
      ],
      customer: {
        first_name: floralData.sender.name.split(' ')[0],
        last_name: floralData.sender.name.split(' ').slice(1).join(' ')
      },
      shipping_address: {
        address1: floralData.deliveryAddress.street,
        city: floralData.deliveryAddress.city,
        province: floralData.deliveryAddress.state,
        zip: floralData.deliveryAddress.postalCode
      }
    };

    const response = await fetch(
      `https://${this.shopDomain}/admin/api/${this.apiVersion}/orders.json`,
      {
        method: 'POST',
        headers: {
          'X-Shopify-Access-Token': this.accessToken,
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({ order })
      }
    );

    return response.json();
  }
}
```

### 4.2 CRM Integration

**Platform:** Salesforce

```javascript
class SalesforceIntegration {
  constructor(instanceUrl, accessToken) {
    this.instanceUrl = instanceUrl;
    this.accessToken = accessToken;
  }

  async createCase(deceased, service) {
    // Create a case in Salesforce for funeral service
    const caseData = {
      Subject: `Funeral Service: ${deceased.legalName.firstName} ${deceased.legalName.lastName}`,
      Description: deceased.biography.short,
      Status: 'New',
      Priority: 'High',
      Origin: 'Web',
      Type: 'Funeral Service',
      Date_of_Service__c: service.startDateTime,
      Service_Type__c: service.serviceType,
      Deceased_DOB__c: deceased.dateOfBirth,
      Deceased_DOD__c: deceased.dateOfDeath
    };

    const response = await fetch(
      `${this.instanceUrl}/services/data/v59.0/sobjects/Case`,
      {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${this.accessToken}`,
          'Content-Type': 'application/json'
        },
        body: JSON.stringify(caseData)
      }
    );

    return response.json();
  }

  async syncContactsFromFamily(deceased) {
    // Create contacts for family members
    for (const child of deceased.family.children || []) {
      if (child.status === 'surviving') {
        await this.createContact({
          LastName: deceased.legalName.lastName,
          FirstName: child.name.split(' ')[0],
          Relationship__c: 'Child',
          Related_Deceased__c: deceased.id
        });
      }
    }
  }

  async createContact(contactData) {
    const response = await fetch(
      `${this.instanceUrl}/services/data/v59.0/sobjects/Contact`,
      {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${this.accessToken}`,
          'Content-Type': 'application/json'
        },
        body: JSON.stringify(contactData)
      }
    );

    return response.json();
  }
}
```

---

## 5. Payment Processing Integration

### 5.1 Stripe Integration

```javascript
const stripe = require('stripe')(process.env.STRIPE_SECRET_KEY);

class PaymentProcessor {
  async processDonation(donationData) {
    try {
      // Create payment intent
      const paymentIntent = await stripe.paymentIntents.create({
        amount: Math.round(donationData.amount * 100), // Convert to cents
        currency: donationData.currency.toLowerCase(),
        payment_method: donationData.paymentMethod,
        metadata: {
          deceased_id: donationData.deceasedId,
          donor_name: donationData.donorInfo.name,
          recipient_type: donationData.recipient.type,
          in_memory_of: donationData.inMemoryOf
        },
        description: `Memorial donation for ${donationData.inMemoryOf}`,
        confirm: true
      });

      // Create donation record
      const donation = await this.createDonationRecord({
        ...donationData,
        transactionId: paymentIntent.id,
        status: paymentIntent.status === 'succeeded' ? 'completed' : 'pending'
      });

      // Generate receipt
      if (paymentIntent.status === 'succeeded') {
        await this.generateReceipt(donation);
        await this.sendThankYouEmail(donation);
      }

      return donation;

    } catch (error) {
      console.error('Payment processing error:', error);
      throw error;
    }
  }

  async setupRecurringDonation(donationData, frequency) {
    // Create customer
    const customer = await stripe.customers.create({
      email: donationData.donorInfo.email,
      name: donationData.donorInfo.name,
      payment_method: donationData.paymentMethod,
      invoice_settings: {
        default_payment_method: donationData.paymentMethod
      }
    });

    // Create subscription
    const subscription = await stripe.subscriptions.create({
      customer: customer.id,
      items: [
        {
          price_data: {
            currency: donationData.currency.toLowerCase(),
            product_data: {
              name: `Monthly memorial donation for ${donationData.inMemoryOf}`
            },
            unit_amount: Math.round(donationData.amount * 100),
            recurring: {
              interval: frequency // 'month', 'year'
            }
          }
        }
      ],
      metadata: {
        deceased_id: donationData.deceasedId,
        in_memory_of: donationData.inMemoryOf
      }
    });

    return subscription;
  }

  async handleWebhook(event) {
    switch (event.type) {
      case 'payment_intent.succeeded':
        await this.handlePaymentSuccess(event.data.object);
        break;

      case 'payment_intent.payment_failed':
        await this.handlePaymentFailure(event.data.object);
        break;

      case 'charge.refunded':
        await this.handleRefund(event.data.object);
        break;
    }
  }

  async generateReceipt(donation) {
    const receiptData = {
      donationId: donation.id,
      amount: donation.amount,
      currency: donation.currency,
      donorName: donation.donorInfo.name,
      donorEmail: donation.donorInfo.email,
      date: donation.donatedAt,
      recipient: donation.recipient.name,
      taxId: donation.recipient.taxId
    };

    // Generate PDF receipt
    const pdfUrl = await this.createPDFReceipt(receiptData);

    // Update donation with receipt URL
    await this.updateDonation(donation.id, { receiptUrl: pdfUrl });

    return pdfUrl;
  }
}
```

### 5.2 PayPal Integration

```javascript
class PayPalIntegration {
  constructor(clientId, clientSecret, mode = 'sandbox') {
    this.clientId = clientId;
    this.clientSecret = clientSecret;
    this.baseUrl = mode === 'live'
      ? 'https://api.paypal.com'
      : 'https://api.sandbox.paypal.com';
  }

  async createOrder(donationData) {
    const accessToken = await this.getAccessToken();

    const order = {
      intent: 'CAPTURE',
      purchase_units: [
        {
          amount: {
            currency_code: donationData.currency,
            value: donationData.amount.toFixed(2)
          },
          description: `Memorial donation for ${donationData.inMemoryOf}`,
          custom_id: donationData.deceasedId
        }
      ],
      application_context: {
        return_url: `https://memorial.example.com/donation/success`,
        cancel_url: `https://memorial.example.com/donation/cancel`
      }
    };

    const response = await fetch(`${this.baseUrl}/v2/checkout/orders`, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${accessToken}`,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(order)
    });

    return response.json();
  }

  async captureOrder(orderId) {
    const accessToken = await this.getAccessToken();

    const response = await fetch(
      `${this.baseUrl}/v2/checkout/orders/${orderId}/capture`,
      {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${accessToken}`,
          'Content-Type': 'application/json'
        }
      }
    );

    return response.json();
  }

  async getAccessToken() {
    const auth = Buffer.from(`${this.clientId}:${this.clientSecret}`).toString('base64');

    const response = await fetch(`${this.baseUrl}/v1/oauth2/token`, {
      method: 'POST',
      headers: {
        'Authorization': `Basic ${auth}`,
        'Content-Type': 'application/x-www-form-urlencoded'
      },
      body: 'grant_type=client_credentials'
    });

    const data = await response.json();
    return data.access_token;
  }
}
```

---

## 6. Streaming Service Integration

### 6.1 AWS MediaLive Integration

```javascript
const AWS = require('aws-sdk');

class AWSMediaLiveIntegration {
  constructor() {
    this.mediaLive = new AWS.MediaLive({
      region: 'us-east-1'
    });
    this.mediaPackage = new AWS.MediaPackage({
      region: 'us-east-1'
    });
  }

  async createLiveChannel(serviceId) {
    // Create MediaPackage channel
    const packageChannel = await this.mediaPackage.createChannel({
      Id: `funeral-service-${serviceId}`,
      Description: `Live stream for service ${serviceId}`
    }).promise();

    // Create HLS endpoint
    const hlsEndpoint = await this.mediaPackage.createOriginEndpoint({
      ChannelId: packageChannel.Id,
      Id: `${packageChannel.Id}-hls`,
      ManifestName: 'index',
      StartoverWindowSeconds: 86400, // 24 hours
      TimeDelaySeconds: 0,
      HlsPackage: {
        SegmentDurationSeconds: 6,
        PlaylistType: 'EVENT',
        PlaylistWindowSeconds: 60,
        AdMarkers: 'NONE'
      }
    }).promise();

    // Create MediaLive input
    const input = await this.mediaLive.createInput({
      Name: `funeral-service-${serviceId}-input`,
      Type: 'RTMP_PUSH',
      Destinations: [
        { StreamName: `stream-${serviceId}/primary` },
        { StreamName: `stream-${serviceId}/backup` }
      ]
    }).promise();

    // Create MediaLive channel
    const channel = await this.mediaLive.createChannel({
      Name: `funeral-service-${serviceId}`,
      RoleArn: process.env.MEDIALIVE_ROLE_ARN,
      InputAttachments: [
        {
          InputId: input.Input.Id,
          InputSettings: {
            SourceEndBehavior: 'CONTINUE',
            InputFilter: 'AUTO',
            FilterStrength: 1,
            DeblockFilter: 'DISABLED',
            DenoiseFilter: 'DISABLED'
          }
        }
      ],
      Destinations: [
        {
          Id: 'destination1',
          MediaPackageSettings: [
            { ChannelId: packageChannel.Id }
          ]
        }
      ],
      EncoderSettings: this.getEncoderSettings()
    }).promise();

    return {
      channelId: channel.Channel.Id,
      inputUrls: input.Input.Destinations.map(d => d.Url),
      streamKey: input.Input.Destinations[0].StreamName,
      playbackUrl: hlsEndpoint.Url
    };
  }

  getEncoderSettings() {
    return {
      TimecodeConfig: {
        Source: 'SYSTEMCLOCK'
      },
      AudioDescriptions: [
        {
          AudioSelectorName: 'default',
          CodecSettings: {
            AacSettings: {
              Bitrate: 128000,
              CodingMode: 'CODING_MODE_2_0',
              InputType: 'NORMAL',
              Profile: 'LC',
              RateControlMode: 'CBR',
              RawFormat: 'NONE',
              SampleRate: 48000,
              Spec: 'MPEG4'
            }
          }
        }
      ],
      VideoDescriptions: [
        {
          Name: 'video_1080p',
          CodecSettings: {
            H264Settings: {
              Bitrate: 5000000,
              FramerateControl: 'SPECIFIED',
              FramerateNumerator: 30,
              FramerateDenominator: 1,
              GopSize: 2,
              GopSizeUnits: 'SECONDS',
              Profile: 'HIGH',
              Level: 'H264_LEVEL_4_1'
            }
          },
          Height: 1080,
          Width: 1920
        },
        {
          Name: 'video_720p',
          CodecSettings: {
            H264Settings: {
              Bitrate: 2500000,
              FramerateControl: 'SPECIFIED',
              FramerateNumerator: 30,
              FramerateDenominator: 1
            }
          },
          Height: 720,
          Width: 1280
        },
        {
          Name: 'video_480p',
          CodecSettings: {
            H264Settings: {
              Bitrate: 1000000,
              FramerateControl: 'SPECIFIED',
              FramerateNumerator: 30,
              FramerateDenominator: 1
            }
          },
          Height: 480,
          Width: 854
        }
      ],
      OutputGroups: [
        {
          OutputGroupSettings: {
            MediaPackageGroupSettings: {
              Destination: {
                DestinationRefId: 'destination1'
              }
            }
          },
          Outputs: [
            {
              OutputName: '1080p',
              VideoDescriptionName: 'video_1080p',
              AudioDescriptionNames: ['default'],
              OutputSettings: {
                MediaPackageOutputSettings: {}
              }
            },
            {
              OutputName: '720p',
              VideoDescriptionName: 'video_720p',
              AudioDescriptionNames: ['default'],
              OutputSettings: {
                MediaPackageOutputSettings: {}
              }
            },
            {
              OutputName: '480p',
              VideoDescriptionName: 'video_480p',
              AudioDescriptionNames: ['default'],
              OutputSettings: {
                MediaPackageOutputSettings: {}
              }
            }
          ]
        }
      ]
    };
  }

  async startChannel(channelId) {
    await this.mediaLive.startChannel({ ChannelId: channelId }).promise();
  }

  async stopChannel(channelId) {
    await this.mediaLive.stopChannel({ ChannelId: channelId }).promise();
  }
}
```

### 6.2 Zoom Integration

```javascript
class ZoomIntegration {
  constructor(apiKey, apiSecret) {
    this.apiKey = apiKey;
    this.apiSecret = apiSecret;
    this.baseUrl = 'https://api.zoom.us/v2';
  }

  async createMeeting(service) {
    const token = this.generateJWT();

    const meetingData = {
      topic: `Memorial Service for ${service.deceased.name}`,
      type: 2, // Scheduled meeting
      start_time: service.startDateTime,
      duration: Math.ceil(
        (new Date(service.endDateTime) - new Date(service.startDateTime)) / 60000
      ),
      timezone: service.timezone,
      password: this.generateMeetingPassword(),
      settings: {
        host_video: true,
        participant_video: false,
        join_before_host: true,
        mute_upon_entry: true,
        watermark: false,
        use_pmi: false,
        approval_type: 0, // Automatically approve
        audio: 'both',
        auto_recording: 'cloud',
        waiting_room: false,
        allow_multiple_devices: true
      }
    };

    const response = await fetch(`${this.baseUrl}/users/me/meetings`, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(meetingData)
    });

    const meeting = await response.json();

    return {
      meetingId: meeting.id,
      joinUrl: meeting.join_url,
      startUrl: meeting.start_url,
      password: meeting.password
    };
  }

  async getRecording(meetingId) {
    const token = this.generateJWT();

    const response = await fetch(
      `${this.baseUrl}/meetings/${meetingId}/recordings`,
      {
        headers: {
          'Authorization': `Bearer ${token}`
        }
      }
    );

    return response.json();
  }

  generateJWT() {
    const jwt = require('jsonwebtoken');

    const payload = {
      iss: this.apiKey,
      exp: Date.now() + 3600000 // 1 hour
    };

    return jwt.sign(payload, this.apiSecret);
  }

  generateMeetingPassword() {
    return Math.random().toString(36).substring(2, 8).toUpperCase();
  }
}
```

---

## 7. Cemetery Management Integration

### 7.1 PlotBox Integration

```javascript
class PlotBoxIntegration {
  constructor(apiKey, organizationId) {
    this.apiKey = apiKey;
    this.organizationId = organizationId;
    this.baseUrl = 'https://api.plotbox.io/v1';
  }

  async createInterment(intermentData) {
    const plotboxData = {
      organization_id: this.organizationId,
      deceased: {
        first_name: intermentData.deceased.firstName,
        last_name: intermentData.deceased.lastName,
        date_of_birth: intermentData.deceased.dateOfBirth,
        date_of_death: intermentData.deceased.dateOfDeath
      },
      plot: {
        cemetery_id: intermentData.cemetery.externalId,
        section: intermentData.plotLocation.section,
        lot: intermentData.plotLocation.lot,
        grave: intermentData.plotLocation.grave
      },
      interment_date: intermentData.intermentDate,
      interment_type: this.mapIntermentType(intermentData.intermentType)
    };

    const response = await fetch(`${this.baseUrl}/interments`, {
      method: 'POST',
      headers: {
        'X-API-Key': this.apiKey,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(plotboxData)
    });

    return response.json();
  }

  async searchPlots(cemeteryId, criteria) {
    const params = new URLSearchParams({
      cemetery_id: cemeteryId,
      section: criteria.section,
      availability: 'available'
    });

    const response = await fetch(
      `${this.baseUrl}/plots/search?${params}`,
      {
        headers: {
          'X-API-Key': this.apiKey
        }
      }
    );

    return response.json();
  }

  async getMonumentDetails(plotId) {
    const response = await fetch(
      `${this.baseUrl}/plots/${plotId}/monument`,
      {
        headers: {
          'X-API-Key': this.apiKey
        }
      }
    );

    return response.json();
  }

  mapIntermentType(wiaType) {
    const mapping = {
      'full-body': 'FULL_BURIAL',
      'cremains': 'CREMATION',
      'above-ground': 'MAUSOLEUM',
      'columbarium': 'COLUMBARIUM'
    };
    return mapping[wiaType] || 'OTHER';
  }
}
```

---

## 8. Notification Service Integration

### 8.1 SendGrid Email Integration

```javascript
const sgMail = require('@sendgrid/mail');

class EmailNotificationService {
  constructor(apiKey) {
    sgMail.setApiKey(apiKey);
    this.fromEmail = 'notifications@memorial-service.example.com';
  }

  async sendServiceReminder(service, attendees) {
    const messages = attendees.map(attendee => ({
      to: attendee.email,
      from: this.fromEmail,
      subject: `Reminder: Service for ${service.deceased.name}`,
      templateId: 'd-service-reminder-template',
      dynamic_template_data: {
        attendee_name: attendee.name,
        deceased_name: service.deceased.name,
        service_type: service.serviceType,
        service_date: new Date(service.startDateTime).toLocaleDateString(),
        service_time: new Date(service.startDateTime).toLocaleTimeString(),
        venue_name: service.venue.name,
        venue_address: this.formatAddress(service.venue.address),
        stream_url: service.virtualService?.viewerUrl,
        calendar_link: this.generateCalendarLink(service)
      }
    }));

    await sgMail.send(messages);
  }

  async sendDonationReceipt(donation) {
    const msg = {
      to: donation.donorInfo.email,
      from: this.fromEmail,
      subject: 'Thank you for your memorial donation',
      templateId: 'd-donation-receipt-template',
      dynamic_template_data: {
        donor_name: donation.donorInfo.name,
        amount: donation.amount,
        currency: donation.currency,
        in_memory_of: donation.inMemoryOf,
        recipient_name: donation.recipient.name,
        tax_id: donation.recipient.taxId,
        receipt_url: donation.receiptUrl,
        donation_date: new Date(donation.donatedAt).toLocaleDateString()
      }
    };

    await sgMail.send(msg);
  }

  async sendCondolenceNotification(condolence, familyContacts) {
    const messages = familyContacts.map(contact => ({
      to: contact.email,
      from: this.fromEmail,
      subject: 'New condolence message received',
      templateId: 'd-condolence-notification-template',
      dynamic_template_data: {
        contact_name: contact.name,
        deceased_name: condolence.deceasedName,
        author_name: condolence.authorName,
        message: condolence.message,
        view_url: `https://memorial.example.com/condolences/${condolence.id}`
      }
    }));

    await sgMail.send(messages);
  }

  formatAddress(address) {
    return `${address.street}, ${address.city}, ${address.state} ${address.postalCode}`;
  }

  generateCalendarLink(service) {
    // Generate Google Calendar link
    const params = new URLSearchParams({
      action: 'TEMPLATE',
      text: `Service for ${service.deceased.name}`,
      dates: this.formatGoogleCalendarDate(service.startDateTime, service.endDateTime),
      location: service.venue.name,
      details: service.specialInstructions || ''
    });

    return `https://calendar.google.com/calendar/render?${params}`;
  }

  formatGoogleCalendarDate(start, end) {
    const format = (date) => new Date(date).toISOString().replace(/[-:]/g, '').split('.')[0] + 'Z';
    return `${format(start)}/${format(end)}`;
  }
}
```

### 8.2 Twilio SMS Integration

```javascript
const twilio = require('twilio');

class SMSNotificationService {
  constructor(accountSid, authToken, fromNumber) {
    this.client = twilio(accountSid, authToken);
    this.fromNumber = fromNumber;
  }

  async sendServiceReminder(service, phoneNumber) {
    const message = `Reminder: ${service.serviceType} for ${service.deceased.name} tomorrow at ${new Date(service.startDateTime).toLocaleTimeString()} at ${service.venue.name}. ${service.virtualService?.enabled ? `Watch online: ${service.virtualService.viewerUrl}` : ''}`;

    await this.client.messages.create({
      body: message,
      from: this.fromNumber,
      to: phoneNumber
    });
  }

  async sendRSVPConfirmation(rsvp, service) {
    const message = `RSVP confirmed for ${service.deceased.name}'s ${service.serviceType} on ${new Date(service.startDateTime).toLocaleDateString()}. Confirmation code: ${rsvp.confirmationCode}`;

    await this.client.messages.create({
      body: message,
      from: this.fromNumber,
      to: rsvp.attendeeInfo.phone
    });
  }

  async sendBulkNotification(phoneNumbers, messageText) {
    const messages = phoneNumbers.map(phone =>
      this.client.messages.create({
        body: messageText,
        from: this.fromNumber,
        to: phone
      })
    );

    await Promise.all(messages);
  }
}
```

---

## 9. Analytics and Reporting

### 9.1 Google Analytics Integration

```javascript
class AnalyticsService {
  constructor(measurementId) {
    this.measurementId = measurementId;
    this.apiSecret = process.env.GA_API_SECRET;
  }

  async trackEvent(eventName, eventParams) {
    const payload = {
      client_id: this.generateClientId(),
      events: [
        {
          name: eventName,
          params: eventParams
        }
      ]
    };

    await fetch(
      `https://www.google-analytics.com/mp/collect?measurement_id=${this.measurementId}&api_secret=${this.apiSecret}`,
      {
        method: 'POST',
        body: JSON.stringify(payload)
      }
    );
  }

  async trackServiceView(serviceId, deceased) {
    await this.trackEvent('view_service', {
      service_id: serviceId,
      deceased_name: `${deceased.legalName.firstName} ${deceased.legalName.lastName}`,
      service_type: 'memorial'
    });
  }

  async trackDonation(donation) {
    await this.trackEvent('donation', {
      currency: donation.currency,
      value: donation.amount,
      transaction_id: donation.transactionId,
      deceased_id: donation.deceasedId,
      recipient_type: donation.recipient.type
    });
  }

  async trackStreamView(serviceId, duration) {
    await this.trackEvent('stream_view', {
      service_id: serviceId,
      engagement_time_msec: duration * 1000
    });
  }

  generateClientId() {
    return crypto.randomUUID();
  }
}
```

### 9.2 Custom Reporting Dashboard

```javascript
class ReportingService {
  async generateServiceReport(serviceId) {
    const service = await this.getService(serviceId);
    const rsvps = await this.getServiceRSVPs(serviceId);
    const streamStats = await this.getStreamStatistics(serviceId);
    const donations = await this.getServiceDonations(serviceId);
    const condolences = await this.getCondolences(service.deceasedId);

    const report = {
      service: {
        id: serviceId,
        type: service.serviceType,
        date: service.startDateTime,
        venue: service.venue.name
      },
      attendance: {
        rsvpCount: rsvps.length,
        attending: rsvps.filter(r => r.response === 'attending').length,
        notAttending: rsvps.filter(r => r.response === 'not-attending').length,
        virtualOnly: rsvps.filter(r => r.response === 'virtual-only').length,
        totalGuests: rsvps.reduce((sum, r) => sum + r.numberOfGuests, 0)
      },
      streaming: {
        peakViewers: streamStats.peakViewers,
        totalViews: streamStats.totalViews,
        averageWatchTime: streamStats.averageWatchTime,
        recordingViews: streamStats.recordingViews
      },
      donations: {
        count: donations.length,
        totalAmount: donations.reduce((sum, d) => sum + d.amount, 0),
        currency: donations[0]?.currency || 'USD',
        averageDonation: donations.length > 0
          ? donations.reduce((sum, d) => sum + d.amount, 0) / donations.length
          : 0
      },
      engagement: {
        condolenceCount: condolences.length,
        tributeCount: await this.getTributeCount(service.deceasedId),
        photoCount: await this.getPhotoCount(service.deceasedId)
      },
      generatedAt: new Date().toISOString()
    };

    return report;
  }

  async exportToExcel(report) {
    const ExcelJS = require('exceljs');
    const workbook = new ExcelJS.Workbook();

    // Summary sheet
    const summarySheet = workbook.addWorksheet('Summary');
    summarySheet.columns = [
      { header: 'Metric', key: 'metric', width: 30 },
      { header: 'Value', key: 'value', width: 20 }
    ];

    summarySheet.addRows([
      { metric: 'Service Type', value: report.service.type },
      { metric: 'Service Date', value: report.service.date },
      { metric: 'Venue', value: report.service.venue },
      { metric: 'Total RSVPs', value: report.attendance.rsvpCount },
      { metric: 'Attending', value: report.attendance.attending },
      { metric: 'Peak Viewers', value: report.streaming.peakViewers },
      { metric: 'Total Donations', value: `${report.donations.currency} ${report.donations.totalAmount}` },
      { metric: 'Condolences', value: report.engagement.condolenceCount }
    ]);

    // RSVPs sheet
    const rsvpSheet = workbook.addWorksheet('RSVPs');
    const rsvps = await this.getServiceRSVPs(report.service.id);
    rsvpSheet.columns = [
      { header: 'Name', key: 'name', width: 30 },
      { header: 'Email', key: 'email', width: 30 },
      { header: 'Response', key: 'response', width: 15 },
      { header: 'Guests', key: 'guests', width: 10 },
      { header: 'Attendance Type', key: 'type', width: 15 }
    ];
    rsvpSheet.addRows(rsvps.map(r => ({
      name: r.attendeeInfo.name,
      email: r.attendeeInfo.email,
      response: r.response,
      guests: r.numberOfGuests,
      type: r.attendanceType
    })));

    const buffer = await workbook.xlsx.writeBuffer();
    return buffer;
  }
}
```

---

## 10. Security and Compliance

### 10.1 Data Encryption

| Data Type | At Rest | In Transit | Key Management |
|-----------|---------|------------|----------------|
| Personal Information | AES-256 | TLS 1.3 | AWS KMS / HSM |
| Payment Data | PCI DSS compliant | TLS 1.3 | Payment processor |
| Photos/Videos | AES-256 | TLS 1.3 | AWS KMS |
| Chat Messages | AES-256 | WSS (TLS 1.3) | Per-service keys |

### 10.2 GDPR Compliance

```javascript
class GDPRComplianceService {
  async exportUserData(userId) {
    // Gather all user data
    const userData = {
      profile: await this.getUserProfile(userId),
      rsvps: await this.getUserRSVPs(userId),
      condolences: await this.getUserCondolences(userId),
      donations: await this.getUserDonations(userId),
      tributesCreated: await this.getUserTributes(userId)
    };

    // Create exportable JSON
    return {
      requestedAt: new Date().toISOString(),
      userId: userId,
      data: userData
    };
  }

  async deleteUserData(userId, requestType) {
    switch (requestType) {
      case 'full-deletion':
        // Delete all user data
        await this.deleteUserProfile(userId);
        await this.anonymizeUserRSVPs(userId);
        await this.anonymizeUserCondolences(userId);
        await this.deleteUserDonations(userId);
        break;

      case 'anonymize':
        // Keep data but remove PII
        await this.anonymizeUserProfile(userId);
        await this.anonymizeUserRSVPs(userId);
        await this.anonymizeUserCondolences(userId);
        await this.anonymizeDonations(userId);
        break;
    }

    // Log deletion request
    await this.logDataDeletion(userId, requestType);
  }

  async anonymizeUserCondolences(userId) {
    const condolences = await this.getUserCondolences(userId);

    for (const condolence of condolences) {
      await this.updateCondolence(condolence.id, {
        authorName: 'Anonymous',
        authorEmail: null,
        metadata: {
          ...condolence.metadata,
          anonymized: true,
          anonymizedAt: new Date().toISOString()
        }
      });
    }
  }
}
```

### 10.3 Audit Logging

```javascript
class AuditLogger {
  async logEvent(eventType, userId, resourceType, resourceId, details) {
    const auditEntry = {
      timestamp: new Date().toISOString(),
      eventType: eventType,
      userId: userId,
      resourceType: resourceType,
      resourceId: resourceId,
      ipAddress: details.ipAddress,
      userAgent: details.userAgent,
      changes: details.changes,
      result: details.result
    };

    // Write to audit log table
    await this.writeAuditLog(auditEntry);

    // Also stream to SIEM if configured
    if (process.env.SIEM_ENABLED) {
      await this.streamToSIEM(auditEntry);
    }
  }

  async getAuditTrail(resourceType, resourceId, startDate, endDate) {
    return await this.queryAuditLogs({
      resourceType,
      resourceId,
      timestamp: {
        $gte: startDate,
        $lte: endDate
      }
    });
  }
}
```

---

## 11. Deployment Patterns

### 11.1 Containerized Deployment (Docker)

**docker-compose.yml:**
```yaml
version: '3.8'

services:
  api:
    image: funeral-service-api:latest
    ports:
      - "3000:3000"
    environment:
      - NODE_ENV=production
      - DATABASE_URL=postgresql://user:pass@db:5432/funeral_service
      - REDIS_URL=redis://cache:6379
    depends_on:
      - db
      - cache
    deploy:
      replicas: 3
      resources:
        limits:
          cpus: '1'
          memory: 1G

  websocket:
    image: funeral-service-websocket:latest
    ports:
      - "3001:3001"
    environment:
      - REDIS_URL=redis://cache:6379
    depends_on:
      - cache
    deploy:
      replicas: 2

  stream:
    image: funeral-service-stream:latest
    ports:
      - "1935:1935"
      - "8080:8080"
    environment:
      - CDN_URL=https://cdn.example.com
    deploy:
      replicas: 2

  db:
    image: postgres:15
    environment:
      - POSTGRES_DB=funeral_service
      - POSTGRES_USER=user
      - POSTGRES_PASSWORD=pass
    volumes:
      - postgres_data:/var/lib/postgresql/data

  cache:
    image: redis:7
    volumes:
      - redis_data:/data

  nginx:
    image: nginx:latest
    ports:
      - "80:80"
      - "443:443"
    volumes:
      - ./nginx.conf:/etc/nginx/nginx.conf
      - ./ssl:/etc/nginx/ssl
    depends_on:
      - api
      - websocket

volumes:
  postgres_data:
  redis_data:
```

### 11.2 Kubernetes Deployment

**deployment.yaml:**
```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: funeral-service-api
  labels:
    app: funeral-service
    component: api
spec:
  replicas: 3
  selector:
    matchLabels:
      app: funeral-service
      component: api
  template:
    metadata:
      labels:
        app: funeral-service
        component: api
    spec:
      containers:
      - name: api
        image: funeral-service-api:1.0.0
        ports:
        - containerPort: 3000
        env:
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: funeral-service-secrets
              key: database-url
        - name: REDIS_URL
          value: redis://redis-service:6379
        resources:
          requests:
            memory: "512Mi"
            cpu: "500m"
          limits:
            memory: "1Gi"
            cpu: "1000m"
        livenessProbe:
          httpGet:
            path: /health
            port: 3000
          initialDelaySeconds: 30
          periodSeconds: 10
        readinessProbe:
          httpGet:
            path: /ready
            port: 3000
          initialDelaySeconds: 5
          periodSeconds: 5
---
apiVersion: v1
kind: Service
metadata:
  name: funeral-service-api
spec:
  selector:
    app: funeral-service
    component: api
  ports:
  - protocol: TCP
    port: 80
    targetPort: 3000
  type: LoadBalancer
```

### 11.3 Serverless Deployment (AWS Lambda)

```javascript
// serverless.yml
service: funeral-service-api

provider:
  name: aws
  runtime: nodejs18.x
  region: us-east-1
  environment:
    DYNAMODB_TABLE: ${self:service}-${sls:stage}
  iamRoleStatements:
    - Effect: Allow
      Action:
        - dynamodb:Query
        - dynamodb:Scan
        - dynamodb:GetItem
        - dynamodb:PutItem
        - dynamodb:UpdateItem
        - dynamodb:DeleteItem
      Resource: "arn:aws:dynamodb:${aws:region}:*:table/${self:provider.environment.DYNAMODB_TABLE}"

functions:
  getDeceased:
    handler: handlers/deceased.get
    events:
      - http:
          path: deceased/{id}
          method: get
          cors: true
          authorizer: auth

  createService:
    handler: handlers/service.create
    events:
      - http:
          path: services
          method: post
          cors: true
          authorizer: auth

  streamWebSocket:
    handler: handlers/websocket.handler
    events:
      - websocket:
          route: $connect
      - websocket:
          route: $disconnect
      - websocket:
          route: $default

resources:
  Resources:
    FuneralServiceTable:
      Type: AWS::DynamoDB::Table
      Properties:
        TableName: ${self:provider.environment.DYNAMODB_TABLE}
        AttributeDefinitions:
          - AttributeName: pk
            AttributeType: S
          - AttributeName: sk
            AttributeType: S
        KeySchema:
          - AttributeName: pk
            KeyType: HASH
          - AttributeName: sk
            KeyType: RANGE
        BillingMode: PAY_PER_REQUEST
```

---

## 12. Testing and Quality Assurance

### 12.1 Unit Testing

```javascript
// tests/deceased.test.js
const { createDeceasedPerson, getDeceasedPerson } = require('../src/deceased');

describe('Deceased Person Management', () => {
  test('should create deceased person with valid data', async () => {
    const data = {
      legalName: {
        firstName: 'John',
        lastName: 'Doe'
      },
      dateOfBirth: '1950-01-01',
      dateOfDeath: '2025-12-01T10:00:00Z',
      privacy: {
        level: 'public'
      }
    };

    const result = await createDeceasedPerson(data);

    expect(result).toHaveProperty('id');
    expect(result.legalName.firstName).toBe('John');
    expect(result.metadata.verificationStatus).toBe('pending');
  });

  test('should reject invalid date of death', async () => {
    const data = {
      legalName: { firstName: 'John', lastName: 'Doe' },
      dateOfBirth: '1950-01-01',
      dateOfDeath: '2030-12-01T10:00:00Z', // Future date
      privacy: { level: 'public' }
    };

    await expect(createDeceasedPerson(data)).rejects.toThrow('Date of death cannot be in the future');
  });
});
```

### 12.2 Integration Testing

```javascript
// tests/integration/service-flow.test.js
describe('Complete Service Flow', () => {
  let deceasedId, serviceId;

  test('should create deceased person', async () => {
    const response = await request(app)
      .post('/v1/deceased')
      .set('Authorization', `Bearer ${testToken}`)
      .send(deceasedData);

    expect(response.status).toBe(201);
    deceasedId = response.body.data.id;
  });

  test('should create funeral service', async () => {
    const response = await request(app)
      .post('/v1/services')
      .set('Authorization', `Bearer ${testToken}`)
      .send({
        deceasedId,
        serviceType: 'memorial',
        startDateTime: '2025-12-18T14:00:00Z'
      });

    expect(response.status).toBe(201);
    serviceId = response.body.data.id;
  });

  test('should allow guest RSVP', async () => {
    const response = await request(app)
      .post(`/v1/services/${serviceId}/rsvp`)
      .send({
        attendeeInfo: {
          name: 'Test Guest',
          email: 'guest@example.com'
        },
        response: 'attending'
      });

    expect(response.status).toBe(201);
    expect(response.body.data.confirmationCode).toBeDefined();
  });
});
```

### 12.3 Load Testing

```javascript
// k6 load test script
import http from 'k6/http';
import { check, sleep } from 'k6';

export let options = {
  stages: [
    { duration: '2m', target: 100 },
    { duration: '5m', target: 100 },
    { duration: '2m', target: 200 },
    { duration: '5m', target: 200 },
    { duration: '2m', target: 0 }
  ],
  thresholds: {
    http_req_duration: ['p(95)<500'],
    http_req_failed: ['rate<0.01']
  }
};

export default function() {
  // Test service viewing
  let response = http.get('https://api.example.com/v1/services/660e8400-e29b-41d4-a716-446655440001');

  check(response, {
    'status is 200': (r) => r.status === 200,
    'response time < 500ms': (r) => r.timings.duration < 500
  });

  sleep(1);

  // Test stream viewer count
  response = http.get('https://api.example.com/v1/services/660e8400-e29b-41d4-a716-446655440001/stream/status');

  check(response, {
    'stream status 200': (r) => r.status === 200
  });

  sleep(2);
}
```

---

## 13. Migration Strategies

### 13.1 Data Migration from Legacy System

```javascript
class DataMigrationService {
  async migrateLegacyData(legacySystemConnection) {
    console.log('Starting data migration...');

    // Phase 1: Migrate deceased persons
    const legacyDeceased = await this.fetchLegacyDeceased(legacySystemConnection);
    for (const legacy of legacyDeceased) {
      const wiaFormat = this.transformToWIAFormat(legacy);
      await this.createDeceasedPerson(wiaFormat);
    }

    // Phase 2: Migrate services
    const legacyServices = await this.fetchLegacyServices(legacySystemConnection);
    for (const legacy of legacyServices) {
      const wiaFormat = this.transformServiceToWIA(legacy);
      await this.createService(wiaFormat);
    }

    // Phase 3: Migrate condolences
    const legacyCondolences = await this.fetchLegacyCondolences(legacySystemConnection);
    for (const legacy of legacyCondolences) {
      const wiaFormat = this.transformCondolenceToWIA(legacy);
      await this.createCondolence(wiaFormat);
    }

    console.log('Migration complete');
  }

  transformToWIAFormat(legacy) {
    return {
      legalName: {
        firstName: legacy.first_name,
        lastName: legacy.last_name,
        preferredName: legacy.nickname
      },
      dateOfBirth: legacy.birth_date,
      dateOfDeath: legacy.death_date,
      biography: {
        short: legacy.obituary_short,
        full: legacy.obituary_full
      },
      privacy: {
        level: legacy.is_public ? 'public' : 'family-only'
      },
      metadata: {
        legacyId: legacy.id,
        migratedAt: new Date().toISOString()
      }
    };
  }
}
```

---

## 14. Implementation Examples

### 14.1 Complete Full-Stack Application

```javascript
// Backend: Express.js API Server
const express = require('express');
const cors = require('cors');
const helmet = require('helmet');
const rateLimit = require('express-rate-limit');

const app = express();

// Middleware
app.use(helmet());
app.use(cors());
app.use(express.json());

const limiter = rateLimit({
  windowMs: 15 * 60 * 1000,
  max: 100
});
app.use('/api/', limiter);

// Routes
app.use('/v1/deceased', require('./routes/deceased'));
app.use('/v1/services', require('./routes/services'));
app.use('/v1/rsvp', require('./routes/rsvp'));
app.use('/v1/donations', require('./routes/donations'));

// Health check
app.get('/health', (req, res) => {
  res.json({ status: 'healthy', timestamp: new Date().toISOString() });
});

// Error handling
app.use((err, req, res, next) => {
  console.error(err.stack);
  res.status(500).json({
    success: false,
    error: {
      code: 'INTERNAL_ERROR',
      message: err.message
    }
  });
});

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => {
  console.log(`Server running on port ${PORT}`);
});
```

```javascript
// Frontend: React Application
import React, { useState, useEffect } from 'react';
import { FuneralServiceClient } from './api/client';

function ServiceViewer({ serviceId }) {
  const [service, setService] = useState(null);
  const [viewerCount, setViewerCount] = useState(0);
  const [chatMessages, setChatMessages] = useState([]);

  useEffect(() => {
    const client = new FuneralServiceClient(serviceId);

    // Load service details
    client.getService().then(setService);

    // Connect to WebSocket for live updates
    client.connect();

    client.on('viewer_count', (data) => {
      setViewerCount(data.count);
    });

    client.on('chat_message', (data) => {
      setChatMessages(prev => [...prev, data]);
    });

    return () => client.disconnect();
  }, [serviceId]);

  if (!service) return <div>Loading...</div>;

  return (
    <div className="service-viewer">
      <div className="video-container">
        <VideoPlayer url={service.virtualService.streamingUrl} />
        <ViewerCount count={viewerCount} />
      </div>

      <div className="info-panel">
        <ServiceInfo service={service} />
        <ChatPanel messages={chatMessages} serviceId={serviceId} />
        <ReactionBar serviceId={serviceId} />
      </div>
    </div>
  );
}
```

---

## Conclusion

This Phase 4 specification provides comprehensive integration guidance for implementing the WIA Digital Funeral Standard in production environments. It covers architecture patterns, third-party integrations, security, deployment, testing, and migration strategies, enabling developers to build robust, scalable funeral service platforms.

**Implementation Success Factors:**
1. Start with core data models (Phase 1)
2. Implement essential APIs (Phase 2)
3. Add real-time features (Phase 3)
4. Integrate with existing systems (Phase 4)
5. Continuously monitor and optimize

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
