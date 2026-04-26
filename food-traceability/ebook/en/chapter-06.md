# Chapter 6: Recall Management and Crisis Response

**WIA-AGRI-016 eBook Series**

---

## The Critical Importance of Rapid Recall

Food safety incidents can escalate quickly. Effective traceability enables rapid identification and removal of affected products, minimizing public health risk and financial damage.

**Statistics:**
- Average food recall costs: $10 million
- Brand value loss: 20-30% in severe cases
- Legal liability can exceed $100 million
- Time to trace-back: 2-7 days (traditional) vs. < 2 hours (digital)

---

## Recall Classification

### FDA Classifications

**Class I:** Reasonable probability of serious health consequences or death
- Example: Listeria contamination in ready-to-eat foods
- Action: Immediate public announcement, total product removal

**Class II:** Temporary or medically reversible adverse health consequences
- Example: Undeclared allergen
- Action: Public announcement, targeted removal

**Class III:** Not likely to cause adverse health consequences
- Example: Labeling error
- Action: Limited distribution, corrective measures

---

## Rapid Trace-Back System

### Implementation

```javascript
class RecallManagementSystem {
  async initiateRecall(incidentDetails) {
    const recallId = this.generateRecallId();

    // Step 1: Identify affected batches (< 1 hour)
    const affectedBatches = await this.identifyAffectedBatches(
      incidentDetails.rootCause
    );

    // Step 2: Trace forward to all current locations (< 2 hours)
    const distributionMap = await Promise.all(
      affectedBatches.map(batch => this.traceBatchDistribution(batch))
    );

    // Step 3: Notify all stakeholders (immediate)
    await this.notifyStakeholders(recallId, distributionMap);

    // Step 4: Track recall effectiveness
    this.trackRecallProgress(recallId);

    return {
      recallId,
      affectedBatches,
      estimatedUnits: this.calculateTotalUnits(distributionMap),
      locationsNotified: this.countLocations(distributionMap),
      initiatedAt: new Date().toISOString()
    };
  }

  async identifyAffectedBatches(rootCause) {
    if (rootCause.type === 'supplier_contamination') {
      // Find all batches using contaminated ingredient
      return await this.findBatchesWithIngredient(
        rootCause.supplierBatch,
        rootCause.dateRange
      );
    }

    if (rootCause.type === 'processing_issue') {
      // Find all batches from affected equipment/facility
      return await this.findBatchesByProcessing(
        rootCause.facilityGLN,
        rootCause.equipmentId,
        rootCause.dateRange
      );
    }

    // Direct batch identification
    return [rootCause.batchId];
  }

  async traceBatchDistribution(batchId) {
    // Query EPCIS for all shipping/receiving events
    const events = await this.epcisQuery.getEventsByBatch(batchId);

    // Build distribution tree
    const currentLocations = this.findCurrentLocations(events);
    const customerPurchases = this.findConsumerPurchases(events);

    return {
      batchId,
      productName: events[0].product,
      quantity: events[0].quantity,
      currentLocations,
      customerPurchases,
      distributionPath: this.buildPath(events)
    };
  }

  async notifyStakeholders(recallId, distributionMap) {
    const notifications = [];

    // Notify distribution centers and retailers
    for (const batch of distributionMap) {
      for (const location of batch.currentLocations) {
        notifications.push(
          this.sendRecallNotification({
            recipient: location.contacts.foodSafety,
            urgency: 'CRITICAL',
            recallId,
            batchId: batch.batchId,
            product: batch.productName,
            action: 'Remove from shelves immediately',
            reportingUrl: `https://recall.wia.org/${recallId}/report`
          })
        );
      }
    }

    // Notify consumers (if identifiable)
    for (const batch of distributionMap) {
      for (const purchase of batch.customerPurchases) {
        if (purchase.customerId) {
          notifications.push(
            this.sendConsumerAlert({
              recipient: purchase.contactInfo,
              urgency: 'HIGH',
              product: batch.productName,
              action: 'Do not consume. Return for full refund.',
              healthRisk: this.getHealthRiskStatement(recallId)
            })
          );
        }
      }
    }

    await Promise.all(notifications);

    return {
      totalNotifications: notifications.length,
      sentAt: new Date().toISOString()
    };
  }
}
```

### Recall Effectiveness Tracking

```javascript
async function trackRecallEffectiveness(recallId) {
  const recall = await getRecallDetails(recallId);
  const checkIns = await getRecallCheckIns(recallId);

  const effectiveness = {
    recallId,
    totalUnitsAffected: recall.estimatedUnits,
    unitsRecovered: checkIns.reduce((sum, ci) => sum + ci.unitsReturned, 0),
    recoveryRate: 0,
    byChannel: {}
  };

  effectiveness.recoveryRate =
    (effectiveness.unitsRecovered / effectiveness.totalUnitsAffected) * 100;

  // Break down by channel
  const channels = ['distribution', 'retail', 'consumer'];
  for (const channel of channels) {
    const channelCheckIns = checkIns.filter(ci => ci.channel === channel);
    effectiveness.byChannel[channel] = {
      unitsAffected: recall[`${channel}Units`],
      unitsRecovered: channelCheckIns.reduce((sum, ci) => sum + ci.unitsReturned, 0),
      recoveryRate: 0
    };
    effectiveness.byChannel[channel].recoveryRate =
      (effectiveness.byChannel[channel].unitsRecovered /
       effectiveness.byChannel[channel].unitsAffected) * 100;
  }

  return effectiveness;
}
```

---

## Root Cause Analysis

### Contamination Source Tracing

```javascript
async function performRootCauseAnalysis(affectedBatches) {
  // Identify common factors across affected batches
  const commonFactors = {
    sharedSuppliers: [],
    sharedEquipment: [],
    sharedFacilities: [],
    sharedTimeframes: []
  };

  // Find shared suppliers
  for (const batch of affectedBatches) {
    const ingredients = await getIngredients(batch);
    for (const ingredient of ingredients) {
      if (allBatchesContain(affectedBatches, ingredient.supplerBatch)) {
        commonFactors.sharedSuppliers.push({
          supplier: ingredient.supplier,
          ingredientBatch: ingredient.supplierBatch,
          probability: 'HIGH'
        });
      }
    }
  }

  // Find shared processing equipment
  for (const batch of affectedBatches) {
    const processing = await getProcessingDetails(batch);
    if (allBatchesProcessedOn(affectedBatches, processing.equipmentId)) {
      commonFactors.sharedEquipment.push({
        equipmentId: processing.equipmentId,
        facility: processing.facility,
        probability: 'HIGH'
      });
    }
  }

  // Analyze temporal patterns
  const dates = affectedBatches.map(b => new Date(b.productionDate));
  const dateRange = {
    start: new Date(Math.min(...dates)),
    end: new Date(Math.max(...dates))
  };

  commonFactors.sharedTimeframes.push({
    start: dateRange.start.toISOString(),
    end: dateRange.end.toISOString(),
    span: (dateRange.end - dateRange.start) / (1000 * 60 * 60 * 24) // days
  });

  return {
    affectedBatchCount: affectedBatches.length,
    commonFactors,
    likelyRootCause: determineMostLikely(commonFactors)
  };
}
```

---

## Crisis Communication

### Multi-Channel Notification

```javascript
class CrisisNotificationSystem {
  async sendEmergencyNotification(recall) {
    // Email to all stakeholders
    await this.sendEmailNotifications({
      recipients: recall.stakeholderEmails,
      subject: `URGENT RECALL: ${recall.product}`,
      body: this.generateEmailBody(recall)
    });

    // SMS to key personnel
    await this.sendSMS({
      recipients: recall.emergencyContacts,
      message: `RECALL INITIATED: ${recall.product}. Check email for details.`
    });

    // Mobile app push notifications
    await this.sendPushNotifications({
      audience: recall.affectedRetailers,
      title: 'Product Recall Alert',
      body: recall.shortDescription,
      data: { recallId: recall.recallId }
    });

    // Public website announcement
    await this.publishWebAnnouncement(recall);

    // Social media alerts (if public recall)
    if (recall.publicAnnouncement) {
      await this.postSocialMediaAlert(recall);
    }

    // Regulatory submission
    await this.submitToRegulators(recall);
  }
}
```

---

## Regulatory Reporting

### FDA FSMA 204 Compliance

```javascript
async function generateFSMAReport(recallId) {
  const recall = await getRecallDetails(recallId);

  const report = {
    reportType: 'FDA_FSMA_204_RECALL',
    recallId: recallId,
    firmName: recall.firmName,
    productDescription: recall.product,

    // Critical Tracking Events
    criticalTrackingEvents: await getCTEs(recall.affectedBatches),

    // Key Data Elements
    keyDataElements: {
      traceabilityLotCode: recall.affectedBatches.map(b => b.batchId),
      productQuantity: recall.totalUnitsAffected,
      locationDescription: recall.facilityName,
      originReference: recall.originGLN
    },

    // 24-hour traceability
    immediateSuppliers: await getImmediateSuppliers(recall.affectedBatches),
    immediateRecipients: await getImmediateRecipients(recall.affectedBatches),

    // Current status
    recallStatus: recall.status,
    unitsRecovered: recall.unitsRecovered,
    effectivenessRate: recall.effectivenessRate
  };

  // Submit to FDA portal
  await submitToFDA(report);

  return report;
}
```

---

## Chapter Summary

Effective recall management requires:

**Rapid Trace-Back:**
- < 1 hour to identify affected batches
- < 2 hours for complete distribution mapping
- Immediate stakeholder notification

**Root Cause Analysis:**
- Identify common suppliers, equipment, timeframes
- Prevent future incidents

**Crisis Communication:**
- Multi-channel notifications
- Consumer safety prioritization
- Regulatory compliance

**Effectiveness Tracking:**
- Real-time recovery monitoring
- Channel-by-channel analysis

---

## Next Chapter

**Chapter 7: Consumer Engagement and Transparency**

Learn how to build consumer-facing traceability experiences.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
