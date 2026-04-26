# WIA 디지털 장례 표준 - Phase 4: Integration 명세서

**버전:** 1.0.0
**상태:** Draft
**날짜:** 2025-12-18
**Primary Color:** #64748B (Slate)
**시리즈:** Digital Death Services

---

## 목차

1. [소개](#소개)
2. [시스템 아키텍처](#시스템-아키텍처)
3. [타사 통합](#타사-통합)
4. [플랫폼 통합](#플랫폼-통합)
5. [결제 처리 통합](#결제-처리-통합)
6. [스트리밍 서비스 통합](#스트리밍-서비스-통합)
7. [묘지 관리 통합](#묘지-관리-통합)
8. [알림 서비스 통합](#알림-서비스-통합)
9. [분석 및 보고](#분석-및-보고)
10. [보안 및 규정 준수](#보안-및-규정-준수)
11. [배포 패턴](#배포-패턴)
12. [테스트 및 품질 보증](#테스트-및-품질-보증)
13. [마이그레이션 전략](#마이그레이션-전략)
14. [구현 예제](#구현-예제)

---

## 1. 소개

### 1.1 목적

본 명세서는 WIA 디지털 장례 표준을 기존 장례 서비스 플랫폼, 타사 애플리케이션 및 엔터프라이즈 시스템에 통합하기 위한 통합 패턴, 모범 사례 및 구현 가이드라인을 정의합니다. 개발자와 시스템 아키텍트를 위한 포괄적인 지침을 제공합니다.

### 1.2 통합 시나리오

| 시나리오 | 설명 | 복잡도 |
|---------|-----|-------|
| Greenfield 구현 | WIA 표준 기반 새 플랫폼 구축 | 낮음 |
| Brownfield 통합 | 기존 시스템에 WIA 추가 | 중간 |
| 타사 통합 | WIA API를 사용하는 외부 앱 | 낮음 |
| 엔터프라이즈 마이그레이션 | 레거시 시스템 교체 | 높음 |
| 하이브리드 배포 | 점진적 마이그레이션 전략 | 중간 |

### 1.3 통합 아키텍처 계층

```
┌─────────────────────────────────────────────────┐
│          표현 계층 (Presentation Layer)          │
│  (Web, Mobile, Digital Signage, Kiosks)        │
└────────────────┬────────────────────────────────┘
                 │
┌────────────────▼────────────────────────────────┐
│          API Gateway 계층                       │
│  (Authentication, Rate Limiting, Routing)       │
└────────────────┬────────────────────────────────┘
                 │
┌────────────────▼────────────────────────────────┐
│          비즈니스 로직 계층                      │
│  (Funeral Management, Memorial Services)        │
└────────────────┬────────────────────────────────┘
                 │
┌────────────────▼────────────────────────────────┐
│          데이터 액세스 계층                      │
│  (WIA Standard Data Models)                     │
└────────────────┬────────────────────────────────┘
                 │
┌────────────────▼────────────────────────────────┐
│          통합 계층                               │
│  (Payment, Streaming, Cemetery Systems)         │
└─────────────────────────────────────────────────┘
```

---

## 2. 시스템 아키텍처

### 2.1 참조 아키텍처

```
                    ┌──────────────┐
                    │   클라이언트  │
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

### 2.2 구성요소 책임

| 구성요소 | 책임 | 기술 |
|---------|-----|------|
| API Gateway | 인증, 라우팅, 요청 제한 | Kong, AWS API Gateway, NGINX |
| Application Server | 비즈니스 로직, 데이터 검증 | Node.js, Python, Java, .NET |
| Database | 영구 저장소 | PostgreSQL, MySQL, MongoDB |
| Cache | 세션 데이터, 자주 사용하는 쿼리 | Redis, Memcached |
| Message Queue | 비동기 처리, webhook | RabbitMQ, AWS SQS, Redis |
| Object Storage | 미디어 파일, 녹화본 | AWS S3, Google Cloud Storage |
| CDN | 정적 자산, 비디오 스트리밍 | CloudFlare, AWS CloudFront |
| Search Engine | 전체 텍스트 검색, 분석 | Elasticsearch, Algolia |

### 2.3 확장성 고려사항

| 측면 | 전략 | 목표 지표 |
|-----|------|---------|
| 수평 확장 | Auto-scaling 그룹 | 인스턴스당 1000 req/sec |
| Database 확장 | 읽기 복제본, 샤딩 | 쿼리 시간 < 100ms |
| Cache 전략 | 다단계 캐싱 | 캐시 적중률 95%+ |
| CDN 분산 | 글로벌 엣지 위치 | 지연 시간 < 50ms |
| Stream 전달 | 적응형 bitrate, 엣지 서버 | 가동 시간 99.9% |

---

## 3. 타사 통합

### 3.1 장례식장 관리 시스템

**통합 유형:** 양방향 동기화

```javascript
// 예제: FuneralOne과 통합
class FuneralOneIntegration {
  constructor(apiKey, apiSecret) {
    this.apiKey = apiKey;
    this.apiSecret = apiSecret;
    this.baseUrl = 'https://api.funeralone.com/v1';
  }

  async syncDeceasedPerson(wiaDeceasedId) {
    // WIA 시스템에서 가져오기
    const wiaData = await this.getWIADeceasedPerson(wiaDeceasedId);

    // FuneralOne 형식으로 변환
    const funeralOneData = {
      firstName: wiaData.legalName.firstName,
      lastName: wiaData.legalName.lastName,
      dateOfBirth: wiaData.dateOfBirth,
      dateOfDeath: wiaData.dateOfDeath,
      obituary: wiaData.biography.full
    };

    // FuneralOne으로 동기화
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

### 3.2 소셜 미디어 통합

**플랫폼:** Facebook, Instagram, Twitter/X

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
  }

  async shareToFacebook(data) {
    // Facebook Graph API 사용
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
    // Facebook Live 비디오 생성
    const response = await fetch(
      `https://graph.facebook.com/v18.0/me/live_videos`,
      {
        method: 'POST',
        headers: { 'Authorization': `Bearer ${this.facebookToken}` },
        body: JSON.stringify({
          title: '추모식 라이브 스트리밍',
          description: '생애 축하 행사에 참여하세요',
          status: 'LIVE_NOW',
          stream_url: streamUrl
        })
      }
    );
    return response.json();
  }
}
```

### 3.3 캘린더 통합

**플랫폼:** Google Calendar, Outlook, Apple Calendar

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
SUMMARY:${this.escapeICS(service.deceased.name)} ${this.escapeICS(service.serviceType)}
DESCRIPTION:${this.escapeICS(service.specialInstructions || '')}
LOCATION:${this.escapeICS(service.venue.name)}
URL:${service.virtualService?.viewerUrl || ''}
BEGIN:VALARM
TRIGGER:-PT24H
ACTION:DISPLAY
DESCRIPTION:알림: 내일 예식
END:VALARM
END:VEVENT
END:VCALENDAR`;

    return ics;
  }

  async addToGoogleCalendar(service, userToken) {
    const event = {
      summary: `${service.deceased.name} ${service.serviceType}`,
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

## 4. 플랫폼 통합

### 4.1 전자상거래 플랫폼

**통합:** 화환, 추모 제품

```javascript
class ShopifyIntegration {
  constructor(shopDomain, accessToken) {
    this.shopDomain = shopDomain;
    this.accessToken = accessToken;
    this.apiVersion = '2024-01';
  }

  async createMemorialProduct(deceased) {
    const product = {
      title: `${deceased.legalName.firstName} ${deceased.legalName.lastName} 추모`,
      body_html: deceased.biography.short,
      vendor: '추모 서비스',
      product_type: '추모 제품',
      variants: [
        {
          title: '디지털 방명록',
          price: '29000',
          sku: `MEM-GB-${deceased.id}`
        },
        {
          title: '비디오 헌사',
          price: '99000',
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
    // 화환 주문을 위한 Shopify 주문 생성
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

### 4.2 CRM 통합

**플랫폼:** Salesforce

```javascript
class SalesforceIntegration {
  constructor(instanceUrl, accessToken) {
    this.instanceUrl = instanceUrl;
    this.accessToken = accessToken;
  }

  async createCase(deceased, service) {
    // 장례 서비스를 위한 Salesforce 케이스 생성
    const caseData = {
      Subject: `장례 서비스: ${deceased.legalName.firstName} ${deceased.legalName.lastName}`,
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
    // 가족 구성원에 대한 연락처 생성
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

## 5. 결제 처리 통합

### 5.1 Stripe 통합

```javascript
const stripe = require('stripe')(process.env.STRIPE_SECRET_KEY);

class PaymentProcessor {
  async processDonation(donationData) {
    try {
      // Payment intent 생성
      const paymentIntent = await stripe.paymentIntents.create({
        amount: Math.round(donationData.amount), // 원화는 정수
        currency: donationData.currency.toLowerCase(),
        payment_method: donationData.paymentMethod,
        metadata: {
          deceased_id: donationData.deceasedId,
          donor_name: donationData.donorInfo.name,
          recipient_type: donationData.recipient.type,
          in_memory_of: donationData.inMemoryOf
        },
        description: `${donationData.inMemoryOf} 추모 기부`,
        confirm: true
      });

      // 기부 기록 생성
      const donation = await this.createDonationRecord({
        ...donationData,
        transactionId: paymentIntent.id,
        status: paymentIntent.status === 'succeeded' ? 'completed' : 'pending'
      });

      // 영수증 생성
      if (paymentIntent.status === 'succeeded') {
        await this.generateReceipt(donation);
        await this.sendThankYouEmail(donation);
      }

      return donation;

    } catch (error) {
      console.error('결제 처리 오류:', error);
      throw error;
    }
  }

  async setupRecurringDonation(donationData, frequency) {
    // 고객 생성
    const customer = await stripe.customers.create({
      email: donationData.donorInfo.email,
      name: donationData.donorInfo.name,
      payment_method: donationData.paymentMethod,
      invoice_settings: {
        default_payment_method: donationData.paymentMethod
      }
    });

    // 구독 생성
    const subscription = await stripe.subscriptions.create({
      customer: customer.id,
      items: [
        {
          price_data: {
            currency: donationData.currency.toLowerCase(),
            product_data: {
              name: `${donationData.inMemoryOf} 월간 추모 기부`
            },
            unit_amount: Math.round(donationData.amount),
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

    // PDF 영수증 생성
    const pdfUrl = await this.createPDFReceipt(receiptData);

    // 영수증 URL로 기부 업데이트
    await this.updateDonation(donation.id, { receiptUrl: pdfUrl });

    return pdfUrl;
  }
}
```

---

## 6. 스트리밍 서비스 통합

### 6.1 AWS MediaLive 통합

```javascript
const AWS = require('aws-sdk');

class AWSMediaLiveIntegration {
  constructor() {
    this.mediaLive = new AWS.MediaLive({
      region: 'ap-northeast-2' // 서울 리전
    });
    this.mediaPackage = new AWS.MediaPackage({
      region: 'ap-northeast-2'
    });
  }

  async createLiveChannel(serviceId) {
    // MediaPackage 채널 생성
    const packageChannel = await this.mediaPackage.createChannel({
      Id: `funeral-service-${serviceId}`,
      Description: `서비스 ${serviceId}를 위한 라이브 스트림`
    }).promise();

    // HLS endpoint 생성
    const hlsEndpoint = await this.mediaPackage.createOriginEndpoint({
      ChannelId: packageChannel.Id,
      Id: `${packageChannel.Id}-hls`,
      ManifestName: 'index',
      StartoverWindowSeconds: 86400, // 24시간
      TimeDelaySeconds: 0,
      HlsPackage: {
        SegmentDurationSeconds: 6,
        PlaylistType: 'EVENT',
        PlaylistWindowSeconds: 60,
        AdMarkers: 'NONE'
      }
    }).promise();

    // MediaLive input 생성
    const input = await this.mediaLive.createInput({
      Name: `funeral-service-${serviceId}-input`,
      Type: 'RTMP_PUSH',
      Destinations: [
        { StreamName: `stream-${serviceId}/primary` },
        { StreamName: `stream-${serviceId}/backup` }
      ]
    }).promise();

    // MediaLive 채널 생성
    const channel = await this.mediaLive.createChannel({
      Name: `funeral-service-${serviceId}`,
      RoleArn: process.env.MEDIALIVE_ROLE_ARN,
      InputAttachments: [
        {
          InputId: input.Input.Id,
          InputSettings: {
            SourceEndBehavior: 'CONTINUE',
            InputFilter: 'AUTO',
            FilterStrength: 1
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
              SampleRate: 48000
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
              FramerateDenominator: 1
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
              AudioDescriptionNames: ['default']
            },
            {
              OutputName: '720p',
              VideoDescriptionName: 'video_720p',
              AudioDescriptionNames: ['default']
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

---

## 7. 묘지 관리 통합

### 7.1 PlotBox 통합

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

## 8. 알림 서비스 통합

### 8.1 SendGrid 이메일 통합

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
      subject: `알림: ${service.deceased.name} 예식`,
      templateId: 'd-service-reminder-template',
      dynamic_template_data: {
        attendee_name: attendee.name,
        deceased_name: service.deceased.name,
        service_type: service.serviceType,
        service_date: new Date(service.startDateTime).toLocaleDateString('ko-KR'),
        service_time: new Date(service.startDateTime).toLocaleTimeString('ko-KR'),
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
      subject: '추모 기부에 감사드립니다',
      templateId: 'd-donation-receipt-template',
      dynamic_template_data: {
        donor_name: donation.donorInfo.name,
        amount: donation.amount.toLocaleString('ko-KR'),
        currency: donation.currency,
        in_memory_of: donation.inMemoryOf,
        recipient_name: donation.recipient.name,
        tax_id: donation.recipient.taxId,
        receipt_url: donation.receiptUrl,
        donation_date: new Date(donation.donatedAt).toLocaleDateString('ko-KR')
      }
    };

    await sgMail.send(msg);
  }

  formatAddress(address) {
    return `${address.street}, ${address.city}, ${address.state} ${address.postalCode}`;
  }

  generateCalendarLink(service) {
    // Google Calendar 링크 생성
    const params = new URLSearchParams({
      action: 'TEMPLATE',
      text: `${service.deceased.name} 예식`,
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

### 8.2 Twilio SMS 통합

```javascript
const twilio = require('twilio');

class SMSNotificationService {
  constructor(accountSid, authToken, fromNumber) {
    this.client = twilio(accountSid, authToken);
    this.fromNumber = fromNumber;
  }

  async sendServiceReminder(service, phoneNumber) {
    const message = `알림: 내일 ${new Date(service.startDateTime).toLocaleTimeString('ko-KR')} ${service.venue.name}에서 ${service.deceased.name}님의 ${service.serviceType}이(가) 있습니다. ${service.virtualService?.enabled ? `온라인 시청: ${service.virtualService.viewerUrl}` : ''}`;

    await this.client.messages.create({
      body: message,
      from: this.fromNumber,
      to: phoneNumber
    });
  }

  async sendRSVPConfirmation(rsvp, service) {
    const message = `${new Date(service.startDateTime).toLocaleDateString('ko-KR')} ${service.deceased.name}님의 ${service.serviceType} RSVP가 확인되었습니다. 확인 코드: ${rsvp.confirmationCode}`;

    await this.client.messages.create({
      body: message,
      from: this.fromNumber,
      to: rsvp.attendeeInfo.phone
    });
  }
}
```

---

## 9. 분석 및 보고

### 9.1 Google Analytics 통합

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

---

## 10. 보안 및 규정 준수

### 10.1 데이터 암호화

| 데이터 유형 | 저장 시 | 전송 시 | Key 관리 |
|-----------|--------|--------|---------|
| 개인 정보 | AES-256 | TLS 1.3 | AWS KMS / HSM |
| 결제 데이터 | PCI DSS 준수 | TLS 1.3 | 결제 프로세서 |
| 사진/동영상 | AES-256 | TLS 1.3 | AWS KMS |
| 채팅 메시지 | AES-256 | WSS (TLS 1.3) | 서비스별 키 |

### 10.2 GDPR 준수

```javascript
class GDPRComplianceService {
  async exportUserData(userId) {
    // 모든 사용자 데이터 수집
    const userData = {
      profile: await this.getUserProfile(userId),
      rsvps: await this.getUserRSVPs(userId),
      condolences: await this.getUserCondolences(userId),
      donations: await this.getUserDonations(userId),
      tributesCreated: await this.getUserTributes(userId)
    };

    // 내보낼 수 있는 JSON 생성
    return {
      requestedAt: new Date().toISOString(),
      userId: userId,
      data: userData
    };
  }

  async deleteUserData(userId, requestType) {
    switch (requestType) {
      case 'full-deletion':
        // 모든 사용자 데이터 삭제
        await this.deleteUserProfile(userId);
        await this.anonymizeUserRSVPs(userId);
        await this.anonymizeUserCondolences(userId);
        await this.deleteUserDonations(userId);
        break;

      case 'anonymize':
        // 데이터는 유지하되 개인 식별 정보 제거
        await this.anonymizeUserProfile(userId);
        await this.anonymizeUserRSVPs(userId);
        await this.anonymizeUserCondolences(userId);
        await this.anonymizeDonations(userId);
        break;
    }

    // 삭제 요청 기록
    await this.logDataDeletion(userId, requestType);
  }

  async anonymizeUserCondolences(userId) {
    const condolences = await this.getUserCondolences(userId);

    for (const condolence of condolences) {
      await this.updateCondolence(condolence.id, {
        authorName: '익명',
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

---

## 11. 배포 패턴

### 11.1 컨테이너화 배포 (Docker)

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

---

## 12. 테스트 및 품질 보증

### 12.1 단위 테스트

```javascript
// tests/deceased.test.js
const { createDeceasedPerson, getDeceasedPerson } = require('../src/deceased');

describe('고인 관리', () => {
  test('유효한 데이터로 고인 생성', async () => {
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

  test('잘못된 사망일자 거부', async () => {
    const data = {
      legalName: { firstName: 'John', lastName: 'Doe' },
      dateOfBirth: '1950-01-01',
      dateOfDeath: '2030-12-01T10:00:00Z', // 미래 날짜
      privacy: { level: 'public' }
    };

    await expect(createDeceasedPerson(data)).rejects.toThrow('사망일자는 미래일 수 없습니다');
  });
});
```

---

## 13. 마이그레이션 전략

### 13.1 레거시 시스템에서 데이터 마이그레이션

```javascript
class DataMigrationService {
  async migrateLegacyData(legacySystemConnection) {
    console.log('데이터 마이그레이션 시작...');

    // 1단계: 고인 마이그레이션
    const legacyDeceased = await this.fetchLegacyDeceased(legacySystemConnection);
    for (const legacy of legacyDeceased) {
      const wiaFormat = this.transformToWIAFormat(legacy);
      await this.createDeceasedPerson(wiaFormat);
    }

    // 2단계: 서비스 마이그레이션
    const legacyServices = await this.fetchLegacyServices(legacySystemConnection);
    for (const legacy of legacyServices) {
      const wiaFormat = this.transformServiceToWIA(legacy);
      await this.createService(wiaFormat);
    }

    // 3단계: 조문 마이그레이션
    const legacyCondolences = await this.fetchLegacyCondolences(legacySystemConnection);
    for (const legacy of legacyCondolences) {
      const wiaFormat = this.transformCondolenceToWIA(legacy);
      await this.createCondolence(wiaFormat);
    }

    console.log('마이그레이션 완료');
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

## 14. 구현 예제

### 14.1 완전한 Full-Stack 애플리케이션

```javascript
// Backend: Express.js API 서버
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

// 상태 확인
app.get('/health', (req, res) => {
  res.json({ status: 'healthy', timestamp: new Date().toISOString() });
});

// 오류 처리
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
  console.log(`서버가 포트 ${PORT}에서 실행 중입니다`);
});
```

```javascript
// Frontend: React 애플리케이션
import React, { useState, useEffect } from 'react';
import { FuneralServiceClient } from './api/client';

function ServiceViewer({ serviceId }) {
  const [service, setService] = useState(null);
  const [viewerCount, setViewerCount] = useState(0);
  const [chatMessages, setChatMessages] = useState([]);

  useEffect(() => {
    const client = new FuneralServiceClient(serviceId);

    // 서비스 세부 정보 로드
    client.getService().then(setService);

    // 실시간 업데이트를 위한 WebSocket 연결
    client.connect();

    client.on('viewer_count', (data) => {
      setViewerCount(data.count);
    });

    client.on('chat_message', (data) => {
      setChatMessages(prev => [...prev, data]);
    });

    return () => client.disconnect();
  }, [serviceId]);

  if (!service) return <div>로딩 중...</div>;

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

## 결론

이 Phase 4 명세서는 프로덕션 환경에서 WIA 디지털 장례 표준을 구현하기 위한 포괄적인 통합 지침을 제공합니다. 아키텍처 패턴, 타사 통합, 보안, 배포, 테스트 및 마이그레이션 전략을 다루어 개발자가 강력하고 확장 가능한 장례 서비스 플랫폼을 구축할 수 있도록 합니다.

**구현 성공 요인:**
1. 핵심 데이터 모델로 시작 (Phase 1)
2. 필수 API 구현 (Phase 2)
3. 실시간 기능 추가 (Phase 3)
4. 기존 시스템과 통합 (Phase 4)
5. 지속적으로 모니터링 및 최적화

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
