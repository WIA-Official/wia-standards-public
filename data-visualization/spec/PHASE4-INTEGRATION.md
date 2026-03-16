# WIA-DATA-011: PHASE 4 - Integration Specification

**Version:** 1.0
**Status:** Active
**Last Updated:** 2025-12-26

## Overview

This specification defines how WIA-DATA-011 compliant visualization systems integrate with external tools, BI platforms, and enterprise systems. It covers embedding, iframe security, CSP policies, and third-party integrations.

## Table of Contents

1. [Embedding Visualizations](#embedding-visualizations)
2. [BI Tool Integration](#bi-tool-integration)
3. [Cloud Platform Integration](#cloud-platform-integration)
4. [Security & Sandboxing](#security--sandboxing)
5. [Plugin Architecture](#plugin-architecture)
6. [Export & Sharing](#export--sharing)

---

## 1. Embedding Visualizations

### 1.1 Iframe Embedding

**Basic Embed:**

```html
<iframe
  src="https://api.example.com/v1/viz/viz-12345/embed"
  width="800"
  height="600"
  frameborder="0"
  allow="fullscreen"
  sandbox="allow-scripts allow-same-origin"
></iframe>
```

**Embed URL Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `theme` | string | Theme (light, dark, auto) |
| `interactive` | boolean | Enable user interactions |
| `toolbar` | boolean | Show toolbar |
| `controls` | string | Comma-separated control names |
| `autoplay` | boolean | Auto-play animations |

**Example:**

```html
<iframe
  src="https://api.example.com/v1/viz/viz-12345/embed?theme=dark&interactive=true&toolbar=false"
  width="100%"
  height="600"
></iframe>
```

### 1.2 JavaScript SDK

**Installation:**

```bash
npm install @wia/viz-sdk
```

**Usage:**

```javascript
import { WIAVisualization } from '@wia/viz-sdk';

const viz = new WIAVisualization({
  container: '#viz-container',
  vizId: 'viz-12345',
  apiKey: 'YOUR_API_KEY',
  config: {
    theme: 'dark',
    responsive: true,
    interactions: {
      zoom: true,
      pan: true,
      tooltip: true
    }
  }
});

viz.render();

// Update data
viz.updateData(newData);

// Listen to events
viz.on('click', (event) => {
  console.log('Data point clicked:', event.data);
});

// Export
const png = await viz.export('png');
```

### 1.3 React Component

```jsx
import { WIAChart } from '@wia/viz-react';

function Dashboard() {
  return (
    <WIAChart
      vizId="viz-12345"
      apiKey={process.env.WIA_API_KEY}
      width="100%"
      height={600}
      theme="dark"
      onDataClick={(data) => console.log(data)}
    />
  );
}
```

### 1.4 Vue Component

```vue
<template>
  <wia-chart
    :viz-id="vizId"
    :api-key="apiKey"
    width="100%"
    :height="600"
    theme="dark"
    @data-click="handleClick"
  />
</template>

<script>
import { WIAChart } from '@wia/viz-vue';

export default {
  components: { WIAChart },
  data() {
    return {
      vizId: 'viz-12345',
      apiKey: process.env.VUE_APP_WIA_API_KEY
    };
  },
  methods: {
    handleClick(data) {
      console.log('Clicked:', data);
    }
  }
};
</script>
```

---

## 2. BI Tool Integration

### 2.1 Tableau Integration

**WIA-DATA-011 Web Data Connector:**

```javascript
// tableau-wia-connector.js
(function() {
  var connector = tableau.makeConnector();

  connector.getSchema = function(schemaCallback) {
    var cols = [
      { id: "date", dataType: tableau.dataTypeEnum.date },
      { id: "sales", dataType: tableau.dataTypeEnum.float },
      { id: "region", dataType: tableau.dataTypeEnum.string }
    ];

    var tableInfo = {
      id: "wia_data",
      alias: "WIA Dataset",
      columns: cols
    };

    schemaCallback([tableInfo]);
  };

  connector.getData = function(table, doneCallback) {
    fetch('https://api.example.com/v1/viz/data/sales-2025', {
      headers: {
        'Authorization': 'Bearer ' + tableau.password
      }
    })
    .then(response => response.json())
    .then(json => {
      var tableData = json.data.map(row => ({
        date: row.date,
        sales: row.sales,
        region: row.region
      }));

      table.appendRows(tableData);
      doneCallback();
    });
  };

  tableau.registerConnector(connector);
})();
```

### 2.2 Power BI Integration

**Custom Connector (Power Query M):**

```m
let
    Source = Json.Document(Web.Contents(
        "https://api.example.com/v1/viz/data/sales-2025",
        [Headers=[Authorization="Bearer " & ApiKey]]
    )),
    Data = Source[data],
    ToTable = Table.FromList(Data, Splitter.SplitByNothing(), null, null, ExtraValues.Error),
    Expanded = Table.ExpandRecordColumn(ToTable, "Column1", {"date", "sales", "region"})
in
    Expanded
```

### 2.3 Google Data Studio Integration

**Community Connector:**

```javascript
function getSchema(request) {
  return {
    schema: [
      { name: 'date', label: 'Date', dataType: 'STRING', semantics: { conceptType: 'DIMENSION' } },
      { name: 'sales', label: 'Sales', dataType: 'NUMBER', semantics: { conceptType: 'METRIC' } },
      { name: 'region', label: 'Region', dataType: 'STRING', semantics: { conceptType: 'DIMENSION' } }
    ]
  };
}

function getData(request) {
  var apiKey = PropertiesService.getUserProperties().getProperty('API_KEY');
  var url = 'https://api.example.com/v1/viz/data/sales-2025';

  var response = UrlFetchApp.fetch(url, {
    headers: {
      'Authorization': 'Bearer ' + apiKey
    }
  });

  var data = JSON.parse(response.getContentText()).data;

  var rows = data.map(function(row) {
    return {
      values: [row.date, row.sales, row.region]
    };
  });

  return {
    schema: getSchema(request).schema,
    rows: rows
  };
}
```

---

## 3. Cloud Platform Integration

### 3.1 AWS Integration

**Lambda Function:**

```javascript
const AWS = require('aws-sdk');
const fetch = require('node-fetch');

exports.handler = async (event) => {
  const response = await fetch('https://api.example.com/v1/viz/data/sales-2025', {
    headers: {
      'Authorization': `Bearer ${process.env.WIA_API_KEY}`
    }
  });

  const data = await response.json();

  // Store in S3
  const s3 = new AWS.S3();
  await s3.putObject({
    Bucket: 'my-viz-bucket',
    Key: `data/${Date.now()}.json`,
    Body: JSON.stringify(data),
    ContentType: 'application/json'
  }).promise();

  return {
    statusCode: 200,
    body: JSON.stringify({ message: 'Data stored successfully' })
  };
};
```

### 3.2 Google Cloud Integration

**Cloud Function:**

```javascript
const { Storage } = require('@google-cloud/storage');
const fetch = require('node-fetch');

exports.syncWIAData = async (req, res) => {
  const response = await fetch('https://api.example.com/v1/viz/data/sales-2025', {
    headers: {
      'Authorization': `Bearer ${process.env.WIA_API_KEY}`
    }
  });

  const data = await response.json();

  const storage = new Storage();
  const bucket = storage.bucket('my-viz-bucket');
  const file = bucket.file(`data/${Date.now()}.json`);

  await file.save(JSON.stringify(data), {
    contentType: 'application/json'
  });

  res.status(200).send({ message: 'Data synced successfully' });
};
```

### 3.3 Azure Integration

**Azure Function:**

```csharp
using System;
using System.Net.Http;
using System.Threading.Tasks;
using Microsoft.Azure.WebJobs;
using Microsoft.Extensions.Logging;
using Azure.Storage.Blobs;

public static class WIADataSync
{
    [FunctionName("SyncWIAData")]
    public static async Task Run(
        [TimerTrigger("0 */5 * * * *")] TimerInfo timer,
        ILogger log)
    {
        var client = new HttpClient();
        client.DefaultRequestHeaders.Add(
            "Authorization",
            $"Bearer {Environment.GetEnvironmentVariable("WIA_API_KEY")}"
        );

        var response = await client.GetStringAsync(
            "https://api.example.com/v1/viz/data/sales-2025"
        );

        var blobClient = new BlobClient(
            Environment.GetEnvironmentVariable("STORAGE_CONNECTION"),
            "viz-data",
            $"data/{DateTime.UtcNow:yyyy-MM-dd-HH-mm-ss}.json"
        );

        await blobClient.UploadAsync(
            BinaryData.FromString(response)
        );

        log.LogInformation($"Data synced at: {DateTime.Now}");
    }
}
```

---

## 4. Security & Sandboxing

### 4.1 Content Security Policy (CSP)

**Recommended CSP Headers:**

```
Content-Security-Policy:
  default-src 'self';
  script-src 'self' 'unsafe-eval' https://api.example.com;
  style-src 'self' 'unsafe-inline';
  img-src 'self' data: https:;
  connect-src 'self' https://api.example.com wss://api.example.com;
  frame-ancestors 'self' https://trusted-domain.com;
  worker-src 'self' blob:;
```

### 4.2 Iframe Sandbox Attributes

```html
<iframe
  sandbox="
    allow-scripts
    allow-same-origin
    allow-popups
    allow-forms
  "
  src="https://api.example.com/v1/viz/viz-12345/embed"
></iframe>
```

**Sandbox Tokens:**

| Token | Description |
|-------|-------------|
| `allow-scripts` | Enable JavaScript |
| `allow-same-origin` | Treat as same origin |
| `allow-popups` | Allow popup windows |
| `allow-forms` | Allow form submission |
| `allow-downloads` | Allow file downloads |

### 4.3 CORS Configuration

**Server Response Headers:**

```
Access-Control-Allow-Origin: https://trusted-domain.com
Access-Control-Allow-Methods: GET, POST, PUT, DELETE, OPTIONS
Access-Control-Allow-Headers: Authorization, Content-Type
Access-Control-Max-Age: 86400
Access-Control-Allow-Credentials: true
```

---

## 5. Plugin Architecture

### 5.1 Plugin Interface

```typescript
interface WIAPlugin {
  name: string;
  version: string;
  initialize(context: PluginContext): void;
  onDataUpdate?(data: any[]): void;
  onRender?(element: HTMLElement): void;
  onDestroy?(): void;
}

interface PluginContext {
  api: WIAAPIClient;
  config: Record<string, any>;
  emit(event: string, data: any): void;
  on(event: string, handler: Function): void;
}
```

### 5.2 Example Plugin

```javascript
class CustomTooltipPlugin {
  constructor() {
    this.name = 'custom-tooltip';
    this.version = '1.0.0';
  }

  initialize(context) {
    this.context = context;

    context.on('datapoint:hover', (data) => {
      this.showTooltip(data);
    });
  }

  showTooltip(data) {
    const tooltip = document.createElement('div');
    tooltip.className = 'custom-tooltip';
    tooltip.innerHTML = `
      <strong>${data.label}</strong><br>
      Value: ${data.value}<br>
      Trend: ${data.trend > 0 ? '↑' : '↓'} ${Math.abs(data.trend)}%
    `;

    document.body.appendChild(tooltip);

    // Position tooltip
    const rect = event.target.getBoundingClientRect();
    tooltip.style.left = rect.left + 'px';
    tooltip.style.top = (rect.top - tooltip.offsetHeight - 10) + 'px';
  }

  onDestroy() {
    // Cleanup
    document.querySelectorAll('.custom-tooltip').forEach(el => el.remove());
  }
}

// Register plugin
viz.use(new CustomTooltipPlugin());
```

---

## 6. Export & Sharing

### 6.1 Export Formats

**Static Images:**

```javascript
// PNG Export
const png = await viz.export('png', {
  width: 1920,
  height: 1080,
  scale: 2,
  backgroundColor: '#ffffff'
});

// SVG Export
const svg = await viz.export('svg', {
  width: 800,
  height: 600
});

// PDF Export
const pdf = await viz.export('pdf', {
  width: 800,
  height: 600,
  format: 'A4',
  orientation: 'landscape'
});
```

**Data Exports:**

```javascript
// CSV Export
const csv = await viz.exportData('csv');

// JSON Export
const json = await viz.exportData('json');

// Excel Export
const xlsx = await viz.exportData('xlsx', {
  sheetName: 'Sales Data',
  includeMetadata: true
});
```

### 6.2 Sharing Links

**Generate Share Link:**

```javascript
const shareLink = await viz.createShareLink({
  permissions: 'view',
  expiration: '7d',
  password: 'optional-password',
  allowDownload: true
});

console.log(shareLink.url);
// https://api.example.com/v1/viz/share/abc123def456
```

### 6.3 Embedding Code Generator

```javascript
const embedCode = viz.getEmbedCode({
  type: 'iframe',
  width: 800,
  height: 600,
  responsive: true,
  theme: 'dark'
});

console.log(embedCode);
/*
<div style="position: relative; padding-bottom: 75%; height: 0;">
  <iframe
    src="https://api.example.com/v1/viz/viz-12345/embed?theme=dark"
    style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"
    frameborder="0"
  ></iframe>
</div>
*/
```

---

## 7. Monitoring & Analytics

### 7.1 Usage Tracking

```javascript
viz.on('render', () => {
  analytics.track('Visualization Rendered', {
    vizId: 'viz-12345',
    chartType: 'line',
    dataPoints: 1000
  });
});

viz.on('interaction', (event) => {
  analytics.track('User Interaction', {
    vizId: 'viz-12345',
    interactionType: event.type,
    timestamp: Date.now()
  });
});
```

### 7.2 Performance Monitoring

```javascript
const observer = new PerformanceObserver((list) => {
  for (const entry of list.getEntries()) {
    console.log('Visualization Performance:', {
      renderTime: entry.duration,
      startTime: entry.startTime
    });
  }
});

observer.observe({ entryTypes: ['measure'] });

performance.mark('viz-render-start');
await viz.render();
performance.mark('viz-render-end');

performance.measure(
  'viz-render',
  'viz-render-start',
  'viz-render-end'
);
```

---

## Appendix A: Integration Checklist

- [ ] Implement iframe embedding with CSP
- [ ] Configure CORS for allowed domains
- [ ] Set up API authentication
- [ ] Test sandbox security
- [ ] Implement export functionality
- [ ] Add sharing capabilities
- [ ] Configure BI tool connectors
- [ ] Set up cloud platform integration
- [ ] Implement plugin system
- [ ] Add usage analytics
- [ ] Performance monitoring
- [ ] Error tracking

---

## Appendix B: Supported Platforms

| Platform | Integration Type | Documentation |
|----------|-----------------|---------------|
| Tableau | Web Data Connector | [Link](#) |
| Power BI | Custom Connector | [Link](#) |
| Google Data Studio | Community Connector | [Link](#) |
| AWS | Lambda + S3 | [Link](#) |
| Google Cloud | Cloud Functions | [Link](#) |
| Azure | Azure Functions | [Link](#) |
| Salesforce | Lightning Component | [Link](#) |
| SharePoint | Web Part | [Link](#) |

---

**© 2025 WIA (World Certification Industry Association)**
**Standard:** WIA-DATA-011
**Phase:** 4 - Integration Specification
