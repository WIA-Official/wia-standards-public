# WIA-EDU-024: Museum Digital Archive Standard - Implementation Guide

> **Philosophy:** 弘益人間 (Benefit All Humanity)

## Getting Started

This guide will help you implement the WIA Museum Digital Archive Standard in your institution, whether you're starting from scratch or migrating from an existing system.

## Implementation Paths

Choose the path that best fits your needs:

### Path A: Cloud-Hosted Solution (Fastest)
**Best for:** Small to medium museums, limited IT resources
**Time to launch:** 2-4 weeks
**Cost:** $50-500/month

### Path B: Self-Hosted Open Source (Most Flexible)
**Best for:** Institutions with IT staff, customization needs
**Time to launch:** 3-6 months
**Cost:** Infrastructure only

### Path C: Commercial Platform (Enterprise)
**Best for:** Large institutions, complex requirements
**Time to launch:** 6-12 months
**Cost:** $5,000-50,000/year

### Path D: Hybrid Approach
**Best for:** Existing systems, phased migration
**Time to launch:** 6-18 months
**Cost:** Varies

## Phase 1: Planning & Assessment (Weeks 1-2)

### 1.1 Assess Current State

**Inventory your collection:**
```
Total objects: _______
Objects with digital records: _______
Objects with images: _______
High-resolution images: _______
```

**Evaluate existing systems:**
- Collection management software: _______
- Image storage: _______
- Public website: _______
- Rights management: _______

**Identify gaps:**
- [ ] Missing metadata
- [ ] Low-quality images
- [ ] Incomplete provenance
- [ ] Rights clearance needed
- [ ] Accessibility issues

### 1.2 Define Goals

**Primary objectives** (check all that apply):
- [ ] Increase online visibility
- [ ] Support research access
- [ ] Enable virtual exhibitions
- [ ] Improve education programs
- [ ] Generate revenue (licensing, merchandise)
- [ ] Meet grant requirements
- [ ] Comply with accessibility laws

**Success metrics:**
- Monthly website visitors: Target _______
- API requests: Target _______
- Digital objects: Target _______
- Educational users: Target _______

### 1.3 Assemble Team

**Required roles:**
- Project Manager
- Collections Manager/Registrar
- Digital Asset Manager
- Web Developer (if self-hosting)
- Educator/Curator (for content)

**Optional roles:**
- Photographer
- Metadata Specialist
- UX Designer
- Legal Counsel (rights clearance)

### 1.4 Budget Planning

**Sample budget (medium museum, 10,000 objects):**

| Item | Cost |
|------|------|
| Cloud platform (3 years) | $18,000 |
| Photography equipment | $5,000 |
| Staff time (photography) | $15,000 |
| Staff time (cataloging) | $25,000 |
| Consultant (setup/training) | $10,000 |
| Contingency (20%) | $14,600 |
| **Total** | **$87,600** |

## Phase 2: Infrastructure Setup (Weeks 3-6)

### 2.1 Choose Your Stack

#### Option A: Cloud-Hosted (Recommended for most)

**Providers:**
1. **CollectionSpace Cloud** (Open source based)
   - Pros: Open source, standards-compliant, affordable
   - Cons: Less customization
   - Cost: $100-500/month

2. **Gallery Systems IIIF Cloud**
   - Pros: Museum-specific, full-featured
   - Cons: Higher cost
   - Cost: $500-2,000/month

3. **AWS/Azure/GCP Custom**
   - Pros: Maximum flexibility
   - Cons: Requires technical expertise
   - Cost: $200-1,000/month

#### Option B: Self-Hosted

**Technology stack:**
```yaml
Operating System: Ubuntu 22.04 LTS
Database: PostgreSQL 15
Search: Elasticsearch 8
Cache: Redis 7
Web Server: Nginx
Application: Python/Django or Node.js/Express
IIIF Server: Cantaloupe Image Server
Storage: MinIO (S3-compatible)
```

**Minimum server requirements:**
- CPU: 8 cores
- RAM: 32 GB
- Storage: 2 TB SSD + 10 TB HDD
- Bandwidth: 1 Gbps

### 2.2 Install Core Components

#### Self-Hosted Installation (Ubuntu)

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install PostgreSQL
sudo apt install postgresql-15 postgresql-contrib
sudo -u postgres createdb museum_archive

# Install Elasticsearch
wget https://artifacts.elastic.co/downloads/elasticsearch/elasticsearch-8.11.0-amd64.deb
sudo dpkg -i elasticsearch-8.11.0-amd64.deb
sudo systemctl enable elasticsearch
sudo systemctl start elasticsearch

# Install Redis
sudo apt install redis-server
sudo systemctl enable redis-server

# Install Cantaloupe IIIF Server
wget https://github.com/cantaloupe-project/cantaloupe/releases/download/v5.0.5/cantaloupe-5.0.5.zip
unzip cantaloupe-5.0.5.zip
cd cantaloupe-5.0.5

# Install MinIO
wget https://dl.min.io/server/minio/release/linux-amd64/minio
chmod +x minio
sudo mv minio /usr/local/bin/

# Start MinIO
minio server /data --console-address ":9001"
```

### 2.3 Configure Database Schema

```sql
-- Create core tables
CREATE TABLE objects (
  id VARCHAR(50) PRIMARY KEY,
  title VARCHAR(500) NOT NULL,
  creator VARCHAR(500),
  date_created VARCHAR(100),
  medium VARCHAR(200),
  department VARCHAR(100),
  classification VARCHAR(100),
  description TEXT,
  width_cm DECIMAL(10,2),
  height_cm DECIMAL(10,2),
  depth_cm DECIMAL(10,2),
  credit_line TEXT,
  rights VARCHAR(200),
  on_view BOOLEAN DEFAULT false,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE images (
  id SERIAL PRIMARY KEY,
  object_id VARCHAR(50) REFERENCES objects(id),
  file_path VARCHAR(500),
  width INTEGER,
  height INTEGER,
  format VARCHAR(20),
  iiif_id VARCHAR(200),
  is_primary BOOLEAN DEFAULT false,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE exhibitions (
  id VARCHAR(50) PRIMARY KEY,
  title VARCHAR(500) NOT NULL,
  description TEXT,
  curator VARCHAR(200),
  start_date DATE,
  end_date DATE,
  status VARCHAR(50),
  type VARCHAR(50),
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE exhibition_objects (
  exhibition_id VARCHAR(50) REFERENCES exhibitions(id),
  object_id VARCHAR(50) REFERENCES objects(id),
  sort_order INTEGER,
  PRIMARY KEY (exhibition_id, object_id)
);

-- Create indexes
CREATE INDEX idx_objects_department ON objects(department);
CREATE INDEX idx_objects_creator ON objects(creator);
CREATE INDEX idx_objects_on_view ON objects(on_view);
CREATE INDEX idx_images_object_id ON images(object_id);
CREATE INDEX idx_exhibitions_status ON exhibitions(status);

-- Full-text search
CREATE INDEX idx_objects_search ON objects USING GIN(
  to_tsvector('english', title || ' ' || COALESCE(description, ''))
);
```

## Phase 3: Content Migration (Weeks 7-16)

### 3.1 Prepare Metadata

**Metadata mapping template:**

| Source Field | WIA-EDU-024 Field | Required? | Transformation |
|--------------|-------------------|-----------|----------------|
| Object_ID | identifier | Yes | None |
| Object_Name | name | Yes | None |
| Artist | creator.name | Recommended | Parse into first/last |
| Date_Made | dateCreated | Recommended | Normalize format |
| Medium | artMedium | Recommended | Standardize vocabulary |
| Dimensions | width, height, depth | Recommended | Parse and convert to cm |
| Description | description | Recommended | None |
| Department | department | Yes | Map to standard names |

**Quality checklist:**
- [ ] All required fields populated
- [ ] Dates in ISO 8601 format (YYYY-MM-DD)
- [ ] Dimensions in metric (cm)
- [ ] Controlled vocabularies used
- [ ] Rights statements assigned
- [ ] Spelling checked
- [ ] Special characters escaped

### 3.2 Image Preparation

**Image workflow:**

1. **Capture**
   - Camera: 50MP+ (e.g., Canon 5DS R, Nikon D850)
   - Lighting: Color-balanced (5500K)
   - Color target: Include X-Rite ColorChecker
   - Format: RAW (CR2, NEF)

2. **Process**
   - Software: Adobe Lightroom, Capture One
   - Color correction: Match color target
   - Cropping: Remove background
   - Export: TIFF (48-bit, Adobe RGB)

3. **Generate Derivatives**
   ```bash
   # Archive (TIFF, 48-bit, 15000px width)
   convert original.tif -quality 100 -compress lzw archive.tif

   # High-res (JPEG2000, 5000px width)
   convert original.tif -resize 5000x -quality 95 highres.jp2

   # Standard (JPEG, 2000px width)
   convert original.tif -resize 2000x -quality 90 standard.jpg

   # Thumbnail (JPEG, 400px width)
   convert original.tif -resize 400x -quality 85 thumbnail.jpg
   ```

4. **Extract Metadata**
   ```bash
   exiftool -DateTimeOriginal -Artist -Copyright image.jpg
   ```

### 3.3 Batch Import

**CSV import format:**
```csv
identifier,name,creator,dateCreated,medium,department,image_path
2025.001.01,Still Life with Fruit,Jane Smith,2024-03,Oil on canvas,Contemporary Art,/images/2025.001.01.tif
2025.001.02,Abstract Composition,John Doe,2024-05,Acrylic on canvas,Contemporary Art,/images/2025.001.02.tif
```

**Import script (Python):**
```python
import csv
import json
from pathlib import Path

def import_objects(csv_file):
    with open(csv_file) as f:
        reader = csv.DictReader(f)
        for row in reader:
            object_data = {
                "@context": "https://schema.org",
                "@type": "VisualArtwork",
                "wiaStandard": "WIA-EDU-024",
                "identifier": row['identifier'],
                "name": row['name'],
                "creator": {
                    "@type": "Person",
                    "name": row['creator']
                },
                "dateCreated": row['dateCreated'],
                "artMedium": row['medium'],
                "department": row['department']
            }

            # Save to database
            save_object(object_data)

            # Process image
            if row['image_path']:
                process_image(row['identifier'], row['image_path'])

def process_image(object_id, image_path):
    # Upload to IIIF server
    iiif_id = upload_to_iiif(image_path)

    # Save image metadata
    save_image_metadata(object_id, iiif_id)

    print(f"Processed image for {object_id}: {iiif_id}")

import_objects('objects.csv')
```

## Phase 4: API Setup (Weeks 17-20)

### 4.1 Configure REST API

**FastAPI example:**
```python
from fastapi import FastAPI, Query
from typing import Optional
import uvicorn

app = FastAPI(title="Museum Digital Archive API")

@app.get("/objects")
async def list_objects(
    page: int = Query(1, ge=1),
    pageSize: int = Query(20, ge=1, le=100),
    department: Optional[str] = None,
    hasImage: Optional[bool] = None
):
    # Query database
    objects = db.query_objects(
        page=page,
        page_size=pageSize,
        department=department,
        has_image=hasImage
    )

    return {
        "data": objects,
        "pagination": {
            "page": page,
            "pageSize": pageSize,
            "totalResults": db.count_objects()
        }
    }

@app.get("/objects/{object_id}")
async def get_object(object_id: str):
    obj = db.get_object(object_id)
    if not obj:
        raise HTTPException(status_code=404)
    return obj

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

### 4.2 Setup IIIF Server

**Cantaloupe configuration:**
```properties
# cantaloupe.properties
endpoint.iiif.2.enabled = true
endpoint.iiif.3.enabled = true

# Source
source.static = FilesystemSource
FilesystemSource.lookup_strategy = BasicLookupStrategy
FilesystemSource.BasicLookupStrategy.path_prefix = /data/images/

# Cache
cache.server.derivative.enabled = true
cache.server.derivative = FilesystemCache
FilesystemCache.pathname = /var/cache/cantaloupe

# Image processing
processor.selection_strategy = AutomaticSelectionStrategy
processor.jpg = TurboJpegProcessor
processor.tif = Java2dProcessor
```

### 4.3 Enable CORS

```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Configure appropriately for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

## Phase 5: Public Portal (Weeks 21-26)

### 5.1 Web Portal Setup

**Next.js example:**
```typescript
// pages/objects/[id].tsx
import { GetServerSideProps } from 'next';
import { MuseumObject } from '@/types';

export default function ObjectPage({ object }: { object: MuseumObject }) {
  return (
    <div>
      <h1>{object.name}</h1>
      <img src={object.image.contentUrl} alt={object.name} />
      <p>{object.description}</p>
      <dl>
        <dt>Artist:</dt>
        <dd>{object.creator.name}</dd>
        <dt>Date:</dt>
        <dd>{object.dateCreated}</dd>
        <dt>Medium:</dt>
        <dd>{object.artMedium}</dd>
      </dl>
    </div>
  );
}

export const getServerSideProps: GetServerSideProps = async ({ params }) => {
  const res = await fetch(`https://api.museum.org/v1/objects/${params.id}`);
  const object = await res.json();

  return { props: { object } };
};
```

### 5.2 Implement Search

**Elasticsearch query:**
```python
def search_objects(query, filters):
    body = {
        "query": {
            "bool": {
                "must": [
                    {
                        "multi_match": {
                            "query": query,
                            "fields": ["name^3", "description", "creator.name^2"]
                        }
                    }
                ],
                "filter": []
            }
        },
        "aggs": {
            "departments": {
                "terms": { "field": "department.keyword" }
            },
            "creators": {
                "terms": { "field": "creator.name.keyword" }
            }
        }
    }

    if filters.get('department'):
        body['query']['bool']['filter'].append({
            "term": { "department.keyword": filters['department'] }
        })

    return es.search(index="objects", body=body)
```

## Phase 6: Testing & Launch (Weeks 27-30)

### 6.1 Testing Checklist

**Functional Testing:**
- [ ] All API endpoints return correct data
- [ ] IIIF images load and zoom properly
- [ ] Search returns relevant results
- [ ] Filters work correctly
- [ ] Pagination works
- [ ] Exhibition pages display
- [ ] Educational resources accessible

**Performance Testing:**
- [ ] API response time < 200ms
- [ ] Page load time < 2s
- [ ] Image load time < 500ms
- [ ] Handle 1000 concurrent users
- [ ] Database queries optimized

**Security Testing:**
- [ ] HTTPS enabled
- [ ] API rate limiting works
- [ ] Authentication required for admin
- [ ] SQL injection prevented
- [ ] XSS attacks prevented
- [ ] CORS configured correctly

**Accessibility Testing:**
- [ ] WCAG 2.1 AA compliant
- [ ] Screen reader compatible
- [ ] Keyboard navigation works
- [ ] Color contrast meets standards
- [ ] Alternative text for images
- [ ] Form labels present

### 6.2 Pre-Launch Activities

**Content review:**
- [ ] Metadata accuracy verified
- [ ] Images quality-checked
- [ ] Rights statements correct
- [ ] Spelling/grammar checked
- [ ] Links tested

**Documentation:**
- [ ] API documentation published
- [ ] User guides created
- [ ] FAQs written
- [ ] Contact information provided

**Marketing:**
- [ ] Press release prepared
- [ ] Social media posts scheduled
- [ ] Email announcement drafted
- [ ] Partners notified

### 6.3 Launch

**Soft launch (2 weeks):**
- Enable for beta testers
- Monitor analytics
- Fix critical bugs
- Gather feedback

**Public launch:**
- Announce via press release
- Social media campaign
- Email subscribers
- Update website

## Phase 7: Ongoing Maintenance

### 7.1 Daily Tasks
- Monitor system health
- Check error logs
- Review API usage
- Respond to user feedback

### 7.2 Weekly Tasks
- Update new acquisitions
- Process uploaded images
- Review analytics
- Backup databases

### 7.3 Monthly Tasks
- Security updates
- Performance optimization
- Content review
- User surveys

### 7.4 Quarterly Tasks
- Add new features
- Major content additions
- Accessibility audit
- Security audit

## Support & Resources

### Official Resources
- Documentation: https://docs.wia.org/edu-024
- Community Forum: https://community.wia.org
- GitHub: https://github.com/WIA-Official/wia-standards

### Consulting Services
- Implementation support: consulting@wia.org
- Custom development: dev@wia.org
- Training: training@wia.org

### Certification
- Apply: https://cert.wiastandards.com
- Cost: $500-7,500 (based on level)
- Validity: 3 years

---

**Philosophy:** 弘益人間 (Benefit All Humanity)

*WIA - World Certification Industry Association*

© 2025 MIT License
