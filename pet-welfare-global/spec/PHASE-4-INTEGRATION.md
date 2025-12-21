# WIA Pet Welfare Global Integration Standard
## Phase 4 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-12-18
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #F59E0B (Amber)

---

## Table of Contents

1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Integration Patterns](#integration-patterns)
4. [Shelter Management Systems](#shelter-management-systems)
5. [Government Systems](#government-systems)
6. [Third-Party Integrations](#third-party-integrations)
7. [Data Migration](#data-migration)
8. [Testing & Certification](#testing--certification)
9. [Deployment Strategies](#deployment-strategies)
10. [Code Examples](#code-examples)

---

## Overview

### 1.1 Purpose

The WIA Pet Welfare Global Integration Standard defines comprehensive integration patterns, system architectures, migration strategies, and implementation guidelines for connecting animal welfare systems, shelters, government agencies, NGOs, and third-party services into the global pet welfare network.

**Core Objectives**:
- Enable seamless integration with existing shelter management systems
- Provide standardized connectors for government databases and registries
- Support legacy system migration and data transformation
- Define testing and certification procedures for compliant implementations
- Establish best practices for deployment and operations
- Facilitate interoperability across diverse technology stacks

### 1.2 Integration Scope

| Integration Type | Description | Priority |
|------------------|-------------|----------|
| **Shelter Management** | PetPoint, ShelterLuv, Chameleon, custom systems | High |
| **Government Registries** | National pet databases, microchip registries | High |
| **Veterinary Systems** | Practice management software, health records | High |
| **Transport & Logistics** | IATA systems, airline cargo platforms | High |
| **Payment Processing** | Adoption fees, donations, microchip registration | Medium |
| **Social Media** | Adoption listings, success stories | Medium |
| **Mapping Services** | Location tracking, service areas | Medium |
| **Analytics Platforms** | Business intelligence, welfare trends | Low |

### 1.3 Integration Architecture Layers

| Layer | Components | Standards |
|-------|------------|-----------|
| **Application** | Business logic, workflows | WIA-PET-PROTOCOL |
| **API Gateway** | Authentication, rate limiting, routing | OAuth 2.0, REST |
| **Integration** | Adapters, transformers, connectors | ETL, ESB |
| **Data** | Storage, replication, sync | PostgreSQL, MongoDB |
| **Infrastructure** | Containers, orchestration, monitoring | Docker, Kubernetes |

---

## System Architecture

### 2.1 Reference Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        WIA Pet Welfare Global                        │
│                         Integration Architecture                     │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│                         External Systems                             │
├─────────────────┬───────────────┬───────────────┬───────────────────┤
│ Shelter Systems │   Gov Systems │  Vet Systems  │  Third-Party      │
│  - PetPoint     │   - National  │  - Avimark    │  - Payment        │
│  - ShelterLuv   │     Registries│  - Cornerstone│  - Social Media   │
│  - Chameleon    │   - USDA      │  - eVetPractice│ - Maps/GIS       │
└────────┬────────┴───────┬───────┴───────┬───────┴──────┬────────────┘
         │                │               │              │
         │                │               │              │
┌────────▼────────────────▼───────────────▼──────────────▼────────────┐
│                      API Gateway Layer                               │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                 │
│  │ Auth &      │  │ Rate        │  │ Request     │                 │
│  │ AuthZ       │  │ Limiting    │  │ Routing     │                 │
│  └─────────────┘  └─────────────┘  └─────────────┘                 │
└────────┬──────────────────────────────────────────────────────────┬─┘
         │                                                          │
┌────────▼──────────────────────────────────────────────────────────▼─┐
│                   Integration Services Layer                         │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │
│  │  Adapters    │  │ Transformers │  │  Validators  │              │
│  │  - REST API  │  │ - Data Maps  │  │ - Schema     │              │
│  │  - SOAP      │  │ - Format     │  │   Check      │              │
│  │  - File/FTP  │  │   Convert    │  │ - Business   │              │
│  └──────────────┘  └──────────────┘  └──────────────┘              │
│                                                                       │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │
│  │ Sync Engine  │  │  Event Bus   │  │  Workflow    │              │
│  │ - Real-time  │  │ - Pub/Sub    │  │  Engine      │              │
│  │ - Scheduled  │  │ - Topics     │  │ - Processes  │              │
│  └──────────────┘  └──────────────┘  └──────────────┘              │
└────────┬──────────────────────────────────────────────────────────┬─┘
         │                                                          │
┌────────▼──────────────────────────────────────────────────────────▼─┐
│                      Core Services Layer                             │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │
│  │  Animal      │  │  Adoption    │  │  Transport   │              │
│  │  Management  │  │  Service     │  │  Service     │              │
│  └──────────────┘  └──────────────┘  └──────────────┘              │
│                                                                       │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │
│  │  Health      │  │  Abuse       │  │  Welfare     │              │
│  │  Passport    │  │  Reporting   │  │  Assessment  │              │
│  └──────────────┘  └──────────────┘  └──────────────┘              │
└────────┬──────────────────────────────────────────────────────────┬─┘
         │                                                          │
┌────────▼──────────────────────────────────────────────────────────▼─┐
│                        Data Layer                                    │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │
│  │  Primary DB  │  │  Document    │  │  Cache       │              │
│  │  PostgreSQL  │  │  Store       │  │  Redis       │              │
│  │              │  │  MongoDB     │  │              │              │
│  └──────────────┘  └──────────────┘  └──────────────┘              │
│                                                                       │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │
│  │  Blob        │  │  Search      │  │  Analytics   │              │
│  │  Storage     │  │  Elasticsearch│ │  Warehouse   │              │
│  └──────────────┘  └──────────────┘  └──────────────┘              │
└───────────────────────────────────────────────────────────────────────┘
```

### 2.2 Component Specifications

| Component | Technology Options | Purpose |
|-----------|-------------------|---------|
| **API Gateway** | Kong, AWS API Gateway, Nginx | Request routing, auth |
| **Integration Bus** | Apache Camel, MuleSoft, WSO2 | Service orchestration |
| **Message Queue** | RabbitMQ, Apache Kafka, AWS SQS | Async communication |
| **Primary Database** | PostgreSQL, MySQL | Relational data |
| **Document Store** | MongoDB, CouchDB | Flexible schemas |
| **Cache** | Redis, Memcached | Performance optimization |
| **Search** | Elasticsearch, Apache Solr | Full-text search |
| **Blob Storage** | AWS S3, Azure Blob, MinIO | Media files |
| **Container Runtime** | Docker, containerd | Application packaging |
| **Orchestration** | Kubernetes, Docker Swarm | Container management |

### 2.3 Deployment Topologies

#### 2.3.1 Cloud-Native Deployment

```yaml
deployment_type: cloud_native
infrastructure:
  cloud_provider: aws  # or azure, gcp
  regions:
    - us-east-1
    - eu-west-1
    - ap-southeast-1
  services:
    compute: eks  # Kubernetes
    database: rds_postgresql
    cache: elasticache_redis
    storage: s3
    cdn: cloudfront
    dns: route53
  high_availability: true
  auto_scaling: true
  disaster_recovery:
    rpo_hours: 1
    rto_hours: 4
```

#### 2.3.2 On-Premises Deployment

```yaml
deployment_type: on_premises
infrastructure:
  datacenter: primary_dc
  hardware:
    app_servers: 4
    database_servers: 2
    load_balancers: 2
  networking:
    load_balancer: haproxy
    firewall: pfsense
    vpn: wireguard
  storage:
    type: san
    capacity_tb: 10
  backup:
    type: tape_and_offsite
    frequency: daily
```

#### 2.3.3 Hybrid Deployment

```yaml
deployment_type: hybrid
on_premises:
  services:
    - primary_database
    - sensitive_data_storage
  connectivity: vpn_tunnel
cloud:
  services:
    - api_gateway
    - integration_services
    - analytics
  provider: aws
sync_strategy:
  type: bidirectional
  frequency: real_time
```

---

## Integration Patterns

### 3.1 RESTful API Integration

#### 3.1.1 Basic REST Adapter

```python
import requests
import json
from typing import Dict, List, Optional
from datetime import datetime

class WIARestAdapter:
    """
    RESTful API adapter for WIA Pet Welfare Global integration
    """

    def __init__(self, base_url: str, api_key: str, organization_id: str):
        self.base_url = base_url
        self.api_key = api_key
        self.organization_id = organization_id
        self.session = requests.Session()
        self.session.headers.update({
            'Authorization': f'Bearer {api_key}',
            'Content-Type': 'application/json',
            'X-WIA-Organization-ID': organization_id
        })

    def create_animal(self, animal_data: Dict) -> Dict:
        """Create a new animal profile"""
        response = self.session.post(
            f'{self.base_url}/animals',
            json=animal_data
        )
        response.raise_for_status()
        return response.json()

    def update_animal(self, animal_id: str, updates: Dict) -> Dict:
        """Update existing animal profile"""
        response = self.session.patch(
            f'{self.base_url}/animals/{animal_id}',
            json=updates
        )
        response.raise_for_status()
        return response.json()

    def add_vaccination(self, animal_id: str, vaccination: Dict) -> Dict:
        """Add vaccination record to health passport"""
        response = self.session.post(
            f'{self.base_url}/animals/{animal_id}/health-passport/vaccinations',
            json=vaccination
        )
        response.raise_for_status()
        return response.json()

    def search_animals(self, filters: Dict) -> List[Dict]:
        """Search animals with filters"""
        response = self.session.get(
            f'{self.base_url}/animals/search',
            params=filters
        )
        response.raise_for_status()
        return response.json()['data']['animals']

    def submit_welfare_assessment(self, animal_id: str,
                                   assessment: Dict) -> Dict:
        """Submit welfare assessment"""
        response = self.session.post(
            f'{self.base_url}/animals/{animal_id}/welfare-assessments',
            json=assessment
        )
        response.raise_for_status()
        return response.json()

# Usage example
adapter = WIARestAdapter(
    base_url='https://api.wia.org/pet-welfare-global/v1',
    api_key='your_api_key',
    organization_id='WIA-ORG-US-001234'
)

# Create animal
animal = adapter.create_animal({
    'identifiers': {
        'microchip_id': '900123456789012'
    },
    'basic_info': {
        'name': 'Max',
        'species': 'canis_lupus_familiaris',
        'breed': 'Golden Retriever',
        'sex': 'male',
        'date_of_birth': '2023-06-15'
    }
})
print(f"Created animal: {animal['data']['wia_id']}")
```

### 3.2 Event-Driven Integration

#### 3.2.1 Webhook Consumer

```javascript
const express = require('express');
const crypto = require('crypto');

class WIAWebhookConsumer {
  constructor(webhookSecret) {
    this.webhookSecret = webhookSecret;
    this.app = express();
    this.setupMiddleware();
    this.setupRoutes();
  }

  setupMiddleware() {
    this.app.use(express.json());
  }

  verifySignature(payload, signature) {
    const hmac = crypto.createHmac('sha256', this.webhookSecret);
    const digest = hmac.update(JSON.stringify(payload)).digest('hex');
    return crypto.timingSafeEqual(
      Buffer.from(signature),
      Buffer.from(digest)
    );
  }

  setupRoutes() {
    this.app.post('/webhooks/wia', (req, res) => {
      const signature = req.headers['x-wia-signature'];

      if (!this.verifySignature(req.body, signature)) {
        return res.status(401).send('Invalid signature');
      }

      const { event, data } = req.body;

      try {
        this.handleEvent(event, data);
        res.status(200).send('Webhook processed');
      } catch (error) {
        console.error('Error processing webhook:', error);
        res.status(500).send('Processing error');
      }
    });
  }

  handleEvent(event, data) {
    switch (event) {
      case 'animal.created':
        this.onAnimalCreated(data);
        break;

      case 'animal.updated':
        this.onAnimalUpdated(data);
        break;

      case 'animal.adopted':
        this.onAnimalAdopted(data);
        break;

      case 'adoption.application.submitted':
        this.onAdoptionApplication(data);
        break;

      case 'transport.status.updated':
        this.onTransportUpdate(data);
        break;

      case 'abuse.report.submitted':
        this.onAbuseReport(data);
        break;

      case 'welfare.assessment.completed':
        this.onWelfareAssessment(data);
        break;

      default:
        console.log(`Unhandled event: ${event}`);
    }
  }

  onAnimalCreated(data) {
    console.log(`New animal created: ${data.animal_id}`);
    // Update local database
    // Trigger notifications
  }

  onAnimalUpdated(data) {
    console.log(`Animal updated: ${data.animal_id}`);
    // Sync changes to local system
  }

  onAnimalAdopted(data) {
    console.log(`Animal adopted: ${data.animal_id}`);
    // Update inventory
    // Send congratulations email
    // Update statistics
  }

  onAdoptionApplication(data) {
    console.log(`New adoption application: ${data.application_id}`);
    // Notify adoption coordinators
    // Start review workflow
  }

  onTransportUpdate(data) {
    console.log(`Transport update: ${data.transport_id} - ${data.status}`);
    // Update tracking system
    // Notify relevant parties
  }

  onAbuseReport(data) {
    console.log(`Abuse report submitted: ${data.incident_id}`);
    // Alert investigation team
    // Create case in local system
  }

  onWelfareAssessment(data) {
    console.log(`Welfare assessment completed: ${data.assessment_id}`);
    // Update animal welfare scores
    // Trigger interventions if needed
  }

  start(port = 3000) {
    this.app.listen(port, () => {
      console.log(`Webhook consumer listening on port ${port}`);
    });
  }
}

// Usage
const consumer = new WIAWebhookConsumer('your_webhook_secret');
consumer.start(3000);
```

### 3.3 Batch Integration Pattern

#### 3.3.1 ETL Pipeline

```java
import java.sql.*;
import java.util.*;
import org.json.*;

public class WIABatchIntegration {

    private Connection localDb;
    private String wiaApiEndpoint;
    private String apiKey;

    public WIABatchIntegration(Connection localDb, String wiaApiEndpoint,
                                String apiKey) {
        this.localDb = localDb;
        this.wiaApiEndpoint = wiaApiEndpoint;
        this.apiKey = apiKey;
    }

    /**
     * Extract animals from local shelter management system
     */
    public List<Animal> extractAnimals(Date since) throws SQLException {
        List<Animal> animals = new ArrayList<>();

        String query = "SELECT * FROM animals WHERE updated_at > ? " +
                       "ORDER BY updated_at ASC LIMIT 1000";

        try (PreparedStatement stmt = localDb.prepareStatement(query)) {
            stmt.setTimestamp(1, new Timestamp(since.getTime()));

            ResultSet rs = stmt.executeQuery();
            while (rs.next()) {
                animals.add(extractAnimalFromResultSet(rs));
            }
        }

        return animals;
    }

    /**
     * Transform local animal data to WIA format
     */
    public JSONObject transformAnimal(Animal animal) {
        JSONObject wiaAnimal = new JSONObject();

        // Basic info transformation
        JSONObject basicInfo = new JSONObject();
        basicInfo.put("name", animal.getName());
        basicInfo.put("species", mapSpecies(animal.getSpecies()));
        basicInfo.put("breed", animal.getBreed());
        basicInfo.put("sex", animal.getSex().toLowerCase());
        basicInfo.put("date_of_birth", animal.getDateOfBirth().toString());

        // Identifiers transformation
        JSONObject identifiers = new JSONObject();
        if (animal.getMicrochipId() != null) {
            identifiers.put("microchip_id", animal.getMicrochipId());
        }

        // Current status transformation
        JSONObject currentStatus = new JSONObject();
        currentStatus.put("status", mapStatus(animal.getStatus()));
        currentStatus.put("organization_id", "WIA-ORG-US-001234");

        wiaAnimal.put("basic_info", basicInfo);
        wiaAnimal.put("identifiers", identifiers);
        wiaAnimal.put("current_status", currentStatus);

        return wiaAnimal;
    }

    /**
     * Load transformed data to WIA system
     */
    public void loadAnimals(List<JSONObject> wiaAnimals) throws Exception {
        int successCount = 0;
        int errorCount = 0;

        for (JSONObject animal : wiaAnimals) {
            try {
                createOrUpdateAnimal(animal);
                successCount++;
            } catch (Exception e) {
                errorCount++;
                logError("Failed to sync animal", animal, e);
            }

            // Rate limiting: sleep between requests
            Thread.sleep(100);
        }

        System.out.printf("Batch complete: %d success, %d errors%n",
                          successCount, errorCount);
    }

    /**
     * Create or update animal in WIA system
     */
    private void createOrUpdateAnimal(JSONObject animal) throws Exception {
        String microchipId = animal.getJSONObject("identifiers")
                                   .optString("microchip_id");

        // Check if animal exists by microchip
        String existingId = findAnimalByMicrochip(microchipId);

        if (existingId != null) {
            // Update existing
            updateAnimal(existingId, animal);
        } else {
            // Create new
            createAnimal(animal);
        }
    }

    /**
     * Run full ETL pipeline
     */
    public void runPipeline(Date since) throws Exception {
        System.out.println("Starting ETL pipeline...");

        // Extract
        System.out.println("Extracting animals from local database...");
        List<Animal> localAnimals = extractAnimals(since);
        System.out.printf("Extracted %d animals%n", localAnimals.size());

        // Transform
        System.out.println("Transforming to WIA format...");
        List<JSONObject> wiaAnimals = new ArrayList<>();
        for (Animal animal : localAnimals) {
            wiaAnimals.add(transformAnimal(animal));
        }

        // Load
        System.out.println("Loading to WIA system...");
        loadAnimals(wiaAnimals);

        System.out.println("ETL pipeline complete");
    }

    // Helper methods
    private String mapSpecies(String localSpecies) {
        Map<String, String> speciesMap = new HashMap<>();
        speciesMap.put("Dog", "canis_lupus_familiaris");
        speciesMap.put("Cat", "felis_catus");
        speciesMap.put("Rabbit", "oryctolagus_cuniculus");
        return speciesMap.getOrDefault(localSpecies, localSpecies.toLowerCase());
    }

    private String mapStatus(String localStatus) {
        Map<String, String> statusMap = new HashMap<>();
        statusMap.put("Available", "shelter_care");
        statusMap.put("Adopted", "adopted");
        statusMap.put("Foster", "foster_care");
        statusMap.put("Hold", "hold");
        return statusMap.getOrDefault(localStatus, "shelter_care");
    }

    public static void main(String[] args) throws Exception {
        // Database connection
        Connection db = DriverManager.getConnection(
            "jdbc:postgresql://localhost/shelter_db",
            "user", "password"
        );

        WIABatchIntegration integration = new WIABatchIntegration(
            db,
            "https://api.wia.org/pet-welfare-global/v1",
            "your_api_key"
        );

        // Run pipeline for animals updated in last 24 hours
        Date yesterday = new Date(System.currentTimeMillis() - 86400000);
        integration.runPipeline(yesterday);
    }
}
```

### 3.4 Message Queue Integration

#### 3.4.1 RabbitMQ Integration

```python
import pika
import json
import logging
from typing import Callable, Dict

class WIAMessageQueueIntegration:
    """
    RabbitMQ integration for asynchronous WIA data processing
    """

    def __init__(self, rabbitmq_url: str, exchange: str = 'wia.pet.welfare'):
        self.rabbitmq_url = rabbitmq_url
        self.exchange = exchange
        self.connection = None
        self.channel = None
        self.handlers = {}

    def connect(self):
        """Establish connection to RabbitMQ"""
        parameters = pika.URLParameters(self.rabbitmq_url)
        self.connection = pika.BlockingConnection(parameters)
        self.channel = self.connection.channel()

        # Declare exchange
        self.channel.exchange_declare(
            exchange=self.exchange,
            exchange_type='topic',
            durable=True
        )

    def publish_event(self, routing_key: str, message: Dict):
        """Publish event to message queue"""
        self.channel.basic_publish(
            exchange=self.exchange,
            routing_key=routing_key,
            body=json.dumps(message),
            properties=pika.BasicProperties(
                delivery_mode=2,  # Make message persistent
                content_type='application/json'
            )
        )
        logging.info(f"Published event: {routing_key}")

    def subscribe(self, routing_key: str, handler: Callable):
        """Subscribe to events matching routing key"""
        # Declare queue
        queue_name = f'wia.{routing_key}.queue'
        self.channel.queue_declare(queue=queue_name, durable=True)

        # Bind queue to exchange
        self.channel.queue_bind(
            exchange=self.exchange,
            queue=queue_name,
            routing_key=routing_key
        )

        # Register handler
        self.handlers[routing_key] = handler

        # Start consuming
        self.channel.basic_consume(
            queue=queue_name,
            on_message_callback=self._handle_message,
            auto_ack=False
        )

    def _handle_message(self, ch, method, properties, body):
        """Internal message handler"""
        try:
            message = json.loads(body)
            routing_key = method.routing_key

            if routing_key in self.handlers:
                self.handlers[routing_key](message)

            # Acknowledge message
            ch.basic_ack(delivery_tag=method.delivery_tag)

        except Exception as e:
            logging.error(f"Error handling message: {e}")
            # Reject and requeue message
            ch.basic_nack(delivery_tag=method.delivery_tag, requeue=True)

    def start_consuming(self):
        """Start consuming messages"""
        logging.info("Starting message consumer...")
        self.channel.start_consuming()

    def close(self):
        """Close connection"""
        if self.connection:
            self.connection.close()


# Example usage
def handle_animal_created(message):
    print(f"Animal created: {message['animal_id']}")
    # Process animal creation
    # Sync to local database
    # Send notifications

def handle_adoption_approved(message):
    print(f"Adoption approved: {message['adoption_contract_id']}")
    # Update animal status
    # Notify adopter
    # Schedule transport

def handle_welfare_alert(message):
    print(f"Welfare alert: Animal {message['animal_id']} score: {message['score']}")
    # Alert staff
    # Create intervention task

# Setup integration
mq = WIAMessageQueueIntegration('amqp://localhost')
mq.connect()

# Subscribe to events
mq.subscribe('animal.created', handle_animal_created)
mq.subscribe('adoption.approved', handle_adoption_approved)
mq.subscribe('welfare.alert', handle_welfare_alert)

# Start consuming (blocks)
mq.start_consuming()
```

---

## Shelter Management Systems

### 4.1 PetPoint Integration

#### 4.1.1 PetPoint to WIA Sync

```csharp
using System;
using System.Collections.Generic;
using System.Net.Http;
using System.Text.Json;
using System.Threading.Tasks;

public class PetPointWIAIntegration
{
    private readonly HttpClient _petPointClient;
    private readonly HttpClient _wiaClient;
    private readonly string _wiaApiKey;
    private readonly string _organizationId;

    public PetPointWIAIntegration(string petPointApiKey, string wiaApiKey,
                                   string organizationId)
    {
        _petPointClient = new HttpClient();
        _petPointClient.DefaultRequestHeaders.Add("Authorization",
            $"Bearer {petPointApiKey}");

        _wiaClient = new HttpClient();
        _wiaClient.DefaultRequestHeaders.Add("Authorization",
            $"Bearer {wiaApiKey}");
        _wiaClient.DefaultRequestHeaders.Add("X-WIA-Organization-ID",
            organizationId);

        _wiaApiKey = wiaApiKey;
        _organizationId = organizationId;
    }

    public async Task SyncAnimalsFromPetPoint()
    {
        // Fetch animals from PetPoint
        var petPointAnimals = await FetchPetPointAnimals();

        foreach (var animal in petPointAnimals)
        {
            try
            {
                // Transform to WIA format
                var wiaAnimal = TransformPetPointToWIA(animal);

                // Create or update in WIA
                await CreateOrUpdateWIAAnimal(wiaAnimal);

                Console.WriteLine($"Synced animal: {animal.AnimalId}");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error syncing animal {animal.AnimalId}: {ex.Message}");
            }
        }
    }

    private async Task<List<PetPointAnimal>> FetchPetPointAnimals()
    {
        var response = await _petPointClient.GetAsync(
            "https://api.petpoint.com/v2/animals?status=available");

        response.EnsureSuccessStatusCode();

        var json = await response.Content.ReadAsStringAsync();
        return JsonSerializer.Deserialize<List<PetPointAnimal>>(json);
    }

    private WIAAnimal TransformPetPointToWIA(PetPointAnimal petPointAnimal)
    {
        return new WIAAnimal
        {
            Identifiers = new Identifiers
            {
                MicrochipId = petPointAnimal.MicrochipNumber,
                RegistrationNumber = petPointAnimal.AnimalId
            },
            BasicInfo = new BasicInfo
            {
                Name = petPointAnimal.Name,
                Species = MapSpecies(petPointAnimal.Species),
                Breed = petPointAnimal.PrimaryBreed,
                Sex = petPointAnimal.Sex.ToLower(),
                DateOfBirth = petPointAnimal.DateOfBirth,
                Color = petPointAnimal.Color
            },
            CurrentStatus = new CurrentStatus
            {
                Status = MapStatus(petPointAnimal.Status),
                OrganizationId = _organizationId,
                Since = petPointAnimal.IntakeDate,
                AvailableForAdoption = petPointAnimal.Status == "Available"
            },
            BehaviorProfile = new BehaviorProfile
            {
                Temperament = petPointAnimal.Temperament,
                Socialization = new Socialization
                {
                    GoodWithDogs = petPointAnimal.GoodWithDogs,
                    GoodWithCats = petPointAnimal.GoodWithCats,
                    GoodWithChildren = petPointAnimal.GoodWithKids
                }
            }
        };
    }

    private async Task CreateOrUpdateWIAAnimal(WIAAnimal animal)
    {
        // Check if animal exists by microchip
        var existingId = await FindWIAAnimalByMicrochip(
            animal.Identifiers.MicrochipId);

        var content = new StringContent(
            JsonSerializer.Serialize(animal),
            System.Text.Encoding.UTF8,
            "application/json"
        );

        HttpResponseMessage response;

        if (existingId != null)
        {
            // Update existing
            response = await _wiaClient.PatchAsync(
                $"https://api.wia.org/pet-welfare-global/v1/animals/{existingId}",
                content
            );
        }
        else
        {
            // Create new
            response = await _wiaClient.PostAsync(
                "https://api.wia.org/pet-welfare-global/v1/animals",
                content
            );
        }

        response.EnsureSuccessStatusCode();
    }

    private string MapSpecies(string petPointSpecies)
    {
        return petPointSpecies.ToLower() switch
        {
            "dog" => "canis_lupus_familiaris",
            "cat" => "felis_catus",
            "rabbit" => "oryctolagus_cuniculus",
            _ => petPointSpecies.ToLower()
        };
    }

    private string MapStatus(string petPointStatus)
    {
        return petPointStatus switch
        {
            "Available" => "shelter_care",
            "Adopted" => "adopted",
            "Foster" => "foster_care",
            "Hold" => "hold",
            _ => "shelter_care"
        };
    }
}

// Data models
public class PetPointAnimal
{
    public string AnimalId { get; set; }
    public string Name { get; set; }
    public string Species { get; set; }
    public string PrimaryBreed { get; set; }
    public string Sex { get; set; }
    public string DateOfBirth { get; set; }
    public string Color { get; set; }
    public string MicrochipNumber { get; set; }
    public string Status { get; set; }
    public string IntakeDate { get; set; }
    public string Temperament { get; set; }
    public bool GoodWithDogs { get; set; }
    public bool GoodWithCats { get; set; }
    public bool GoodWithKids { get; set; }
}
```

### 4.2 ShelterLuv Integration

```javascript
const axios = require('axios');

class ShelterLuvWIAIntegration {
  constructor(shelterLuvApiKey, wiaApiKey, organizationId) {
    this.shelterLuvClient = axios.create({
      baseURL: 'https://www.shelterluv.com/api/v1',
      headers: { 'X-Api-Key': shelterLuvApiKey }
    });

    this.wiaClient = axios.create({
      baseURL: 'https://api.wia.org/pet-welfare-global/v1',
      headers: {
        'Authorization': `Bearer ${wiaApiKey}`,
        'X-WIA-Organization-ID': organizationId
      }
    });

    this.organizationId = organizationId;
  }

  async syncAnimalsFromShelterLuv() {
    try {
      // Fetch animals from ShelterLuv
      const response = await this.shelterLuvClient.get('/animals', {
        params: { Status: 'Available' }
      });

      const animals = response.data.animals;

      for (const animal of animals) {
        await this.syncAnimal(animal);
      }

      console.log(`Synced ${animals.length} animals from ShelterLuv`);
    } catch (error) {
      console.error('Error syncing from ShelterLuv:', error.message);
    }
  }

  async syncAnimal(shelterLuvAnimal) {
    const wiaAnimal = this.transformShelterLuvToWIA(shelterLuvAnimal);

    try {
      // Check if animal exists
      const existingId = await this.findWIAAnimalByMicrochip(
        wiaAnimal.identifiers.microchip_id
      );

      if (existingId) {
        await this.wiaClient.patch(`/animals/${existingId}`, wiaAnimal);
        console.log(`Updated animal: ${shelterLuvAnimal.Id}`);
      } else {
        await this.wiaClient.post('/animals', wiaAnimal);
        console.log(`Created animal: ${shelterLuvAnimal.Id}`);
      }
    } catch (error) {
      console.error(`Error syncing animal ${shelterLuvAnimal.Id}:`,
                    error.message);
    }
  }

  transformShelterLuvToWIA(animal) {
    return {
      identifiers: {
        microchip_id: animal.MicrochipNumber,
        registration_number: animal.Id
      },
      basic_info: {
        name: animal.Name,
        species: this.mapSpecies(animal.Type),
        breed: animal.Breed,
        sex: animal.Sex.toLowerCase(),
        date_of_birth: animal.DOBUnix ?
          new Date(animal.DOBUnix * 1000).toISOString().split('T')[0] : null,
        color: animal.Color
      },
      current_status: {
        status: this.mapStatus(animal.Status),
        organization_id: this.organizationId,
        available_for_adoption: animal.Status === 'Available'
      },
      behavior_profile: {
        temperament: animal.Personality,
        socialization: {
          good_with_dogs: animal.GoodWithDogs === 'Yes',
          good_with_cats: animal.GoodWithCats === 'Yes',
          good_with_children: animal.GoodWithKids === 'Yes'
        }
      }
    };
  }

  mapSpecies(type) {
    const speciesMap = {
      'Dog': 'canis_lupus_familiaris',
      'Cat': 'felis_catus',
      'Rabbit': 'oryctolagus_cuniculus'
    };
    return speciesMap[type] || type.toLowerCase();
  }

  mapStatus(status) {
    const statusMap = {
      'Available': 'shelter_care',
      'Adopted': 'adopted',
      'Foster': 'foster_care',
      'Hold': 'hold'
    };
    return statusMap[status] || 'shelter_care';
  }

  async findWIAAnimalByMicrochip(microchipId) {
    if (!microchipId) return null;

    try {
      const response = await this.wiaClient.get(`/microchips/${microchipId}`);
      return response.data.data.animal_id;
    } catch (error) {
      return null;
    }
  }
}

// Usage
const integration = new ShelterLuvWIAIntegration(
  'shelterluv_api_key',
  'wia_api_key',
  'WIA-ORG-US-001234'
);

// Run sync every hour
setInterval(() => {
  integration.syncAnimalsFromShelterLuv();
}, 3600000);
```

---

## Government Systems

### 5.1 National Pet Registry Integration

```go
package main

import (
    "bytes"
    "encoding/json"
    "fmt"
    "net/http"
    "time"
)

type NationalRegistryIntegration struct {
    RegistryAPIKey string
    RegistryURL    string
    WIAAPIKey      string
    WIAURL         string
}

type RegistryAnimal struct {
    MicrochipID    string    `json:"microchip_id"`
    OwnerName      string    `json:"owner_name"`
    OwnerContact   string    `json:"owner_contact"`
    RegisteredDate time.Time `json:"registered_date"`
    AnimalType     string    `json:"animal_type"`
    Breed          string    `json:"breed"`
}

func (nri *NationalRegistryIntegration) RegisterAnimalWithGovernment(
    wiaAnimalID string) error {

    // Fetch animal from WIA
    animal, err := nri.fetchWIAAnimal(wiaAnimalID)
    if err != nil {
        return fmt.Errorf("failed to fetch WIA animal: %w", err)
    }

    // Transform to registry format
    registryAnimal := nri.transformToRegistryFormat(animal)

    // Submit to national registry
    err = nri.submitToRegistry(registryAnimal)
    if err != nil {
        return fmt.Errorf("failed to submit to registry: %w", err)
    }

    fmt.Printf("Successfully registered animal %s with national registry\n",
               wiaAnimalID)
    return nil
}

func (nri *NationalRegistryIntegration) fetchWIAAnimal(animalID string) (
    map[string]interface{}, error) {

    url := fmt.Sprintf("%s/animals/%s", nri.WIAURL, animalID)

    req, err := http.NewRequest("GET", url, nil)
    if err != nil {
        return nil, err
    }

    req.Header.Set("Authorization", "Bearer "+nri.WIAAPIKey)

    client := &http.Client{Timeout: 10 * time.Second}
    resp, err := client.Do(req)
    if err != nil {
        return nil, err
    }
    defer resp.Body.Close()

    var result map[string]interface{}
    if err := json.NewDecoder(resp.Body).Decode(&result); err != nil {
        return nil, err
    }

    return result, nil
}

func (nri *NationalRegistryIntegration) transformToRegistryFormat(
    wiaAnimal map[string]interface{}) RegistryAnimal {

    data := wiaAnimal["data"].(map[string]interface{})
    profile := data["animal_profile"].(map[string]interface{})
    identifiers := profile["identifiers"].(map[string]interface{})
    basicInfo := profile["basic_info"].(map[string]interface{})

    return RegistryAnimal{
        MicrochipID:    identifiers["microchip_id"].(string),
        RegisteredDate: time.Now(),
        AnimalType:     basicInfo["species"].(string),
        Breed:          basicInfo["breed"].(string),
    }
}

func (nri *NationalRegistryIntegration) submitToRegistry(
    animal RegistryAnimal) error {

    jsonData, err := json.Marshal(animal)
    if err != nil {
        return err
    }

    req, err := http.NewRequest("POST",
        nri.RegistryURL+"/register",
        bytes.NewBuffer(jsonData))
    if err != nil {
        return err
    }

    req.Header.Set("Authorization", "Bearer "+nri.RegistryAPIKey)
    req.Header.Set("Content-Type", "application/json")

    client := &http.Client{Timeout: 30 * time.Second}
    resp, err := client.Do(req)
    if err != nil {
        return err
    }
    defer resp.Body.Close()

    if resp.StatusCode != http.StatusOK &&
       resp.StatusCode != http.StatusCreated {
        return fmt.Errorf("registry returned status: %d", resp.StatusCode)
    }

    return nil
}

func main() {
    integration := &NationalRegistryIntegration{
        RegistryAPIKey: "registry_api_key",
        RegistryURL:    "https://nationalpetreg.gov/api/v1",
        WIAAPIKey:      "wia_api_key",
        WIAURL:         "https://api.wia.org/pet-welfare-global/v1",
    }

    err := integration.RegisterAnimalWithGovernment("WIA-PET-2025-000001")
    if err != nil {
        fmt.Printf("Error: %v\n", err)
    }
}
```

---

## Third-Party Integrations

### 6.1 Payment Processing Integration (Stripe)

```python
import stripe
from typing import Dict

class WIAPaymentIntegration:
    """
    Stripe payment integration for adoption fees and donations
    """

    def __init__(self, stripe_api_key: str, wia_api_key: str):
        stripe.api_key = stripe_api_key
        self.wia_api_key = wia_api_key

    def process_adoption_fee(self, adoption_contract_id: str,
                             amount_cents: int,
                             payment_method_id: str) -> Dict:
        """Process adoption fee payment"""

        try:
            # Create payment intent
            intent = stripe.PaymentIntent.create(
                amount=amount_cents,
                currency='usd',
                payment_method=payment_method_id,
                confirm=True,
                metadata={
                    'type': 'adoption_fee',
                    'adoption_contract_id': adoption_contract_id,
                    'wia_standard': 'pet-welfare-global'
                }
            )

            # Update adoption record in WIA
            if intent.status == 'succeeded':
                self.update_adoption_payment_status(
                    adoption_contract_id,
                    'paid',
                    intent.id
                )

            return {
                'success': True,
                'payment_intent_id': intent.id,
                'status': intent.status
            }

        except stripe.error.CardError as e:
            return {
                'success': False,
                'error': str(e)
            }

    def process_donation(self, organization_id: str, amount_cents: int,
                        donor_email: str, payment_method_id: str) -> Dict:
        """Process donation to shelter"""

        try:
            intent = stripe.PaymentIntent.create(
                amount=amount_cents,
                currency='usd',
                payment_method=payment_method_id,
                confirm=True,
                receipt_email=donor_email,
                metadata={
                    'type': 'donation',
                    'organization_id': organization_id,
                    'wia_standard': 'pet-welfare-global'
                }
            )

            # Record donation in WIA
            if intent.status == 'succeeded':
                self.record_donation(
                    organization_id,
                    amount_cents,
                    donor_email,
                    intent.id
                )

            return {
                'success': True,
                'payment_intent_id': intent.id,
                'status': intent.status
            }

        except stripe.error.CardError as e:
            return {
                'success': False,
                'error': str(e)
            }

    def update_adoption_payment_status(self, adoption_contract_id: str,
                                       status: str, payment_id: str):
        """Update payment status in WIA system"""
        # Implementation would call WIA API to update adoption record
        pass

    def record_donation(self, organization_id: str, amount_cents: int,
                       donor_email: str, payment_id: str):
        """Record donation in WIA system"""
        # Implementation would call WIA API to record donation
        pass
```

### 6.2 Social Media Integration

```javascript
const { FacebookAdsApi, Page } = require('facebook-nodejs-business-sdk');
const { TwitterApi } = require('twitter-api-v2');

class SocialMediaIntegration {
  constructor(config) {
    this.facebookPageId = config.facebookPageId;
    this.facebookAccessToken = config.facebookAccessToken;
    this.twitterClient = new TwitterApi(config.twitterBearerToken);
    this.wiaApiKey = config.wiaApiKey;
  }

  async postAdoptableAnimal(animalId) {
    // Fetch animal from WIA
    const animal = await this.fetchAnimal(animalId);

    // Post to Facebook
    await this.postToFacebook(animal);

    // Post to Twitter
    await this.postToTwitter(animal);

    console.log(`Posted animal ${animalId} to social media`);
  }

  async postToFacebook(animal) {
    const page = new Page(this.facebookPageId);
    page.accessToken = this.facebookAccessToken;

    const message = this.createAdoptionMessage(animal);

    await page.createPhoto(
      [],
      {
        url: animal.photos[0].url,
        caption: message
      }
    );
  }

  async postToTwitter(animal) {
    const message = this.createAdoptionMessage(animal, 280);

    await this.twitterClient.v2.tweet({
      text: message
    });
  }

  createAdoptionMessage(animal, maxLength = null) {
    let message = `🐾 Meet ${animal.name}!\n\n`;
    message += `${animal.breed}, ${animal.age} years old\n`;
    message += `${animal.description}\n\n`;
    message += `Adopt today! Visit our website for more information.\n`;
    message += `#AdoptDontShop #RescueDog`;

    if (maxLength && message.length > maxLength) {
      message = message.substring(0, maxLength - 3) + '...';
    }

    return message;
  }

  async fetchAnimal(animalId) {
    // Fetch from WIA API
    const response = await fetch(
      `https://api.wia.org/pet-welfare-global/v1/animals/${animalId}`,
      {
        headers: {
          'Authorization': `Bearer ${this.wiaApiKey}`
        }
      }
    );

    const data = await response.json();
    return data.data.animal_profile;
  }
}
```

---

## Data Migration

### 7.1 Migration Planning

| Phase | Activities | Duration |
|-------|-----------|----------|
| **Assessment** | Data audit, mapping, quality analysis | 2-4 weeks |
| **Design** | Migration strategy, tooling selection | 2-3 weeks |
| **Development** | Build migration scripts, transformers | 4-6 weeks |
| **Testing** | Pilot migration, validation | 2-3 weeks |
| **Execution** | Full migration, monitoring | 1-2 weeks |
| **Validation** | Data verification, reconciliation | 1 week |

### 7.2 Data Mapping Example

| Source Field (Legacy) | Target Field (WIA) | Transformation |
|-----------------------|-------------------|----------------|
| `animal_id` | `identifiers.registration_number` | Direct copy |
| `pet_name` | `basic_info.name` | Direct copy |
| `species_code` | `basic_info.species` | Map via lookup table |
| `breed_name` | `basic_info.breed` | Normalize capitalization |
| `gender` | `basic_info.sex` | Convert to lowercase |
| `birth_date` | `basic_info.date_of_birth` | Format as ISO date |
| `chip_number` | `identifiers.microchip_id` | Validate 15 digits |
| `current_location` | `current_status.organization_id` | Map to WIA org ID |

### 7.3 Migration Validation Checklist

- [ ] All required fields populated
- [ ] Data types correct
- [ ] Referential integrity maintained
- [ ] No duplicate records
- [ ] Microchip IDs validated (15 digits, ISO compliant)
- [ ] Dates formatted correctly (ISO 8601)
- [ ] Country codes valid (ISO 3166-1 alpha-2)
- [ ] Enumerated values within allowed sets
- [ ] Cross-references resolved
- [ ] Photos and documents migrated
- [ ] Legacy IDs preserved in metadata

---

## Testing & Certification

### 8.1 Compliance Testing

```yaml
wia_compliance_test:
  organization: "WIA-ORG-US-001234"
  test_date: "2025-12-18"

  api_tests:
    - name: "Authentication"
      tests:
        - oauth2_flow
        - api_key_validation
        - token_expiration
      status: "passed"

    - name: "Animal Management"
      tests:
        - create_animal_profile
        - update_animal_profile
        - search_animals
        - delete_animal_profile
      status: "passed"

    - name: "Data Format Validation"
      tests:
        - json_schema_validation
        - required_fields_check
        - data_type_validation
        - enum_value_validation
      status: "passed"

  protocol_tests:
    - name: "Cross-Border Coordination"
      tests:
        - organization_handshake
        - data_request_response
        - document_verification
      status: "passed"

    - name: "Real-Time Sync"
      tests:
        - change_notification
        - sync_request_response
        - conflict_resolution
      status: "passed"

  integration_tests:
    - name: "Webhook Delivery"
      tests:
        - webhook_signature_verification
        - event_delivery
        - retry_logic
      status: "passed"

  certification_status: "compliant"
  certification_expires: "2026-12-18"
```

### 8.2 Performance Testing

| Test | Target | Result | Status |
|------|--------|--------|--------|
| API Response Time (p95) | < 200ms | 145ms | ✓ Pass |
| Throughput | > 1000 req/sec | 1250 req/sec | ✓ Pass |
| Database Query Time | < 100ms | 78ms | ✓ Pass |
| Webhook Delivery | < 5 sec | 2.8 sec | ✓ Pass |
| Sync Latency | < 10 sec | 6.2 sec | ✓ Pass |
| File Upload (10MB) | < 30 sec | 22 sec | ✓ Pass |

---

## Deployment Strategies

### 9.1 Blue-Green Deployment

```yaml
blue_green_deployment:
  strategy: blue_green

  blue_environment:
    version: "1.0.0"
    status: "active"
    traffic_percentage: 100
    instances: 5

  green_environment:
    version: "1.1.0"
    status: "ready"
    traffic_percentage: 0
    instances: 5

  deployment_steps:
    - Deploy new version to green environment
    - Run smoke tests on green
    - Route 10% traffic to green
    - Monitor metrics for 1 hour
    - If successful, route 50% traffic
    - Monitor for 30 minutes
    - Route 100% traffic to green
    - Mark blue as standby
```

### 9.2 Canary Deployment

```yaml
canary_deployment:
  strategy: canary

  rollout_plan:
    - phase: 1
      traffic_percentage: 5
      duration: "2 hours"
      success_criteria:
        error_rate: "< 0.1%"
        latency_p95: "< 200ms"

    - phase: 2
      traffic_percentage: 25
      duration: "4 hours"
      success_criteria:
        error_rate: "< 0.1%"
        latency_p95: "< 200ms"

    - phase: 3
      traffic_percentage: 50
      duration: "8 hours"
      success_criteria:
        error_rate: "< 0.1%"
        latency_p95: "< 200ms"

    - phase: 4
      traffic_percentage: 100
      duration: "stable"
      success_criteria:
        error_rate: "< 0.1%"
        latency_p95: "< 200ms"

  rollback_trigger:
    error_rate: "> 0.5%"
    latency_p95: "> 500ms"
    manual: true
```

---

## Code Examples

### Example 1: Complete Integration Setup

```python
from wia_sdk import WIAClient
from legacy_system import LegacyDatabase

class CompleteWIAIntegration:
    def __init__(self):
        self.wia = WIAClient(
            api_key='your_api_key',
            organization_id='WIA-ORG-US-001234'
        )
        self.legacy_db = LegacyDatabase()

    def setup_integration(self):
        # 1. Initial data migration
        print("Starting initial data migration...")
        self.migrate_animals()
        self.migrate_vaccinations()

        # 2. Setup real-time sync
        print("Setting up real-time sync...")
        self.setup_webhooks()
        self.setup_event_listeners()

        # 3. Configure scheduled tasks
        print("Configuring scheduled tasks...")
        self.setup_daily_sync()
        self.setup_weekly_reports()

        print("Integration setup complete!")

    def migrate_animals(self):
        animals = self.legacy_db.get_all_animals()
        for animal in animals:
            wia_animal = self.transform_animal(animal)
            self.wia.create_animal(wia_animal)

    def transform_animal(self, legacy_animal):
        return {
            'identifiers': {
                'microchip_id': legacy_animal.microchip,
                'registration_number': legacy_animal.id
            },
            'basic_info': {
                'name': legacy_animal.name,
                'species': self.map_species(legacy_animal.species),
                'breed': legacy_animal.breed
            }
        }

# Run integration
integration = CompleteWIAIntegration()
integration.setup_integration()
```

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
