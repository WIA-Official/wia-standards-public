# WIA-EDU-004 Learning Analytics Standard
## PHASE 3: Integration

**Version:** 1.0.0
**Status:** Complete
**Last Updated:** 2025-01-15

---

## 1. Overview

Phase 3 focuses on integrating the learning analytics system with existing educational technologies, including LMS platforms, SIS systems, assessment tools, and third-party applications.

### 1.1 Integration Goals

- Seamless data flow between systems
- Minimal disruption to existing workflows
- Standards-based interoperability
- Scalable and maintainable integration architecture

---

## 2. LMS Integration

### 2.1 LTI 1.3 Integration

**Deep Linking for Dashboard Embedding:**

```typescript
// LTI launch handler
app.post('/lti/launch', async (req, res) => {
    const platform = await ltijs.validatePlatform(req.body);
    
    if (!platform.valid) {
        return res.status(403).send('Invalid LTI launch');
    }
    
    const userId = platform.user.id;
    const courseId = platform.context.id;
    const role = platform.roles[0];
    
    // Create session and redirect to appropriate dashboard
    const dashboardUrl = `/dashboard?userId=${userId}&courseId=${courseId}&role=${role}`;
    res.redirect(dashboardUrl);
});
```

**Assignment and Grade Services:**

```typescript
// Sync grades to LMS
async function syncGradeToLMS(learnerId, assignmentId, score) {
    const assignment = await getAssignment(assignmentId);
    const ltiScore = {
        userId: learnerId,
        activityProgress: 'Completed',
        gradingProgress: 'FullyGraded',
        scoreGiven: score * assignment.maxScore,
        scoreMaximum: assignment.maxScore,
        timestamp: new Date().toISOString()
    };
    
    await platform.Grade.publishScore(ltiScore);
}
```

### 2.2 Canvas API Integration

```javascript
const canvas = require('canvas-api');

// Initialize Canvas client
const client = new canvas.Client({
    apiUrl: 'https://canvas.university.edu',
    token: process.env.CANVAS_TOKEN
});

// Fetch course analytics
async function fetchCanvasAnalytics(courseId) {
    const [enrollments, submissions, pageViews] = await Promise.all([
        client.get(`/api/v1/courses/${courseId}/enrollments`),
        client.get(`/api/v1/courses/${courseId}/students/submissions`),
        client.get(`/api/v1/courses/${courseId}/analytics/student_summaries`)
    ]);
    
    return transformCanvasData({ enrollments, submissions, pageViews });
}
```

### 2.3 Moodle Integration

```php
// Moodle web service function
function local_wia_analytics_get_course_data($courseid) {
    global $DB;
    
    $course = $DB->get_record('course', array('id' => $courseid));
    $students = get_enrolled_users(context_course::instance($courseid), 'mod/assign:submit');
    
    $analytics = array();
    foreach ($students as $student) {
        $completion = $DB->get_records('course_modules_completion', 
            array('userid' => $student->id, 'coursemoduleid' => $courseid));
        
        $analytics[] = array(
            'userid' => $student->id,
            'completions' => count($completion),
            'grade' => grade_get_grades($courseid, 'user', $student->id)
        );
    }
    
    return $analytics;
}
```

---

## 3. SIS Integration

### 3.1 OneRoster Implementation

```xml
<!-- OneRoster CSV Manifest -->
<manifest xmlns="http://www.imsglobal.org/xsd/imsOneRosterv1p2">
    <version>1.2</version>
    <file>orgs.csv</file>
    <file>academicSessions.csv</file>
    <file>courses.csv</file>
    <file>classes.csv</file>
    <file>users.csv</file>
    <file>enrollments.csv</file>
    <file>demographics.csv</file>
</manifest>
```

**Sync Script:**

```python
import pandas as pd
from datetime import datetime

def sync_oneroster_data(manifest_path):
    # Read CSV files
    users = pd.read_csv(f'{manifest_path}/users.csv')
    enrollments = pd.read_csv(f'{manifest_path}/enrollments.csv')
    courses = pd.read_csv(f'{manifest_path}/courses.csv')
    
    # Transform and load to analytics DB
    for _, user in users.iterrows():
        upsert_learner({
            'id': user['sourcedId'],
            'studentId': user['identifier'],
            'givenName': user['givenName'],
            'familyName': user['familyName'],
            'email': user['email'],
            'role': user['role']
        })
    
    for _, enrollment in enrollments.iterrows():
        upsert_enrollment({
            'userId': enrollment['userId'],
            'classId': enrollment['classId'],
            'role': enrollment['role'],
            'status': enrollment['status'],
            'beginDate': enrollment['beginDate'],
            'endDate': enrollment['endDate']
        })
    
    log_sync_completion(datetime.now())
```

---

## 4. Assessment Platform Integration

### 4.1 QTI (Question and Test Interoperability)

```xml
<!-- QTI 2.1 Assessment Item -->
<assessmentItem xmlns="http://www.imsglobal.org/xsd/imsqti_v2p1"
                identifier="quiz-05-q1"
                title="Machine Learning Basics"
                adaptive="false"
                timeDependent="false">
    <responseDeclaration identifier="RESPONSE" cardinality="single" baseType="identifier">
        <correctResponse>
            <value>ChoiceA</value>
        </correctResponse>
    </responseDeclaration>
    
    <outcomeDeclaration identifier="SCORE" cardinality="single" baseType="float">
        <defaultValue>
            <value>0</value>
        </defaultValue>
    </outcomeDeclaration>
    
    <itemBody>
        <p>Which algorithm is best for classification?</p>
        <choiceInteraction responseIdentifier="RESPONSE" shuffle="true" maxChoices="1">
            <simpleChoice identifier="ChoiceA">Random Forest</simpleChoice>
            <simpleChoice identifier="ChoiceB">K-Means</simpleChoice>
            <simpleChoice identifier="ChoiceC">PCA</simpleChoice>
        </choiceInteraction>
    </itemBody>
    
    <responseProcessing>
        <responseCondition>
            <responseIf>
                <match>
                    <variable identifier="RESPONSE"/>
                    <correct identifier="RESPONSE"/>
                </match>
                <setOutcomeValue identifier="SCORE">
                    <baseValue baseType="float">1.0</baseValue>
                </setOutcomeValue>
            </responseIf>
        </responseCondition>
    </responseProcessing>
</assessmentItem>
```

---

## 5. xAPI and Caliper Integration

### 5.1 Learning Record Store (LRS) Setup

```javascript
// Learning Locker LRS configuration
const lrs = {
    endpoint: 'https://lrs.university.edu/data/xAPI',
    username: process.env.LRS_KEY,
    password: process.env.LRS_SECRET,
    version: '1.0.3'
};

// Send xAPI statement
async function sendStatement(statement) {
    const response = await fetch(`${lrs.endpoint}/statements`, {
        method: 'POST',
        headers: {
            'Authorization': `Basic ${btoa(`${lrs.username}:${lrs.password}`)}`,
            'X-Experience-API-Version': lrs.version,
            'Content-Type': 'application/json'
        },
        body: JSON.stringify(statement)
    });
    
    return response.json();
}
```

### 5.2 Caliper Event Emission

```json
{
  "@context": "http://purl.imsglobal.org/ctx/caliper/v1p2",
  "id": "urn:uuid:3a9eb2b8-25bb-4fef-a2d2-1b38a3d3f567",
  "type": "AssessmentEvent",
  "actor": {
    "id": "https://university.edu/users/554433",
    "type": "Person"
  },
  "action": "Submitted",
  "object": {
    "id": "https://university.edu/terms/201801/courses/CS101/assessments/quiz05",
    "type": "Assessment",
    "name": "Quiz 5: Machine Learning",
    "dateToSubmit": "2025-01-15T23:59:59.000Z",
    "maxScore": 100.0
  },
  "generated": {
    "id": "https://university.edu/terms/201801/courses/CS101/assessments/quiz05/attempts/1",
    "type": "Attempt",
    "assignee": {
      "id": "https://university.edu/users/554433",
      "type": "Person"
    },
    "assignable": {
      "id": "https://university.edu/terms/201801/courses/CS101/assessments/quiz05",
      "type": "Assessment"
    },
    "count": 1,
    "dateCreated": "2025-01-15T14:00:00.000Z",
    "startedAtTime": "2025-01-15T14:00:00.000Z",
    "endedAtTime": "2025-01-15T14:20:00.000Z",
    "duration": "PT20M"
  },
  "eventTime": "2025-01-15T14:20:05.000Z"
}
```

---

## 6. Data Warehouse Integration

### 6.1 ETL Pipeline

```sql
-- Incremental load from staging to warehouse
INSERT INTO warehouse.fact_learning_events
SELECT 
    event_id,
    learner_id,
    course_id,
    activity_id,
    time_key,
    verb_id,
    score_scaled,
    duration_seconds,
    success,
    completion,
    event_timestamp
FROM staging.raw_events
WHERE processed = FALSE
  AND event_timestamp >= (SELECT MAX(event_timestamp) FROM warehouse.fact_learning_events);

UPDATE staging.raw_events
SET processed = TRUE
WHERE processed = FALSE;
```

### 6.2 Data Mart Creation

```sql
-- Course performance data mart
CREATE TABLE mart.course_performance AS
SELECT 
    c.course_id,
    c.course_code,
    c.title,
    t.academic_term,
    COUNT(DISTINCT e.learner_id) as total_students,
    AVG(e.score_scaled) as avg_score,
    SUM(CASE WHEN e.completion = TRUE THEN 1 ELSE 0 END)::FLOAT / COUNT(*) as completion_rate,
    SUM(e.duration_seconds) / 3600.0 as total_hours
FROM warehouse.fact_learning_events e
JOIN warehouse.dim_course c ON e.course_id = c.course_id
JOIN warehouse.dim_time t ON e.time_id = t.time_id
WHERE e.verb_id = 'http://adlnet.gov/expapi/verbs/completed'
GROUP BY c.course_id, c.course_code, c.title, t.academic_term;
```

---

## 7. Third-Party Tool Integration

### 7.1 Video Platform (Kaltura, Panopto)

```javascript
// Webhook handler for video events
app.post('/webhooks/kaltura', async (req, res) => {
    const event = req.body;
    
    if (event.eventType === 'VIDEO_VIEWED') {
        await analytics.track({
            actor: { id: event.userId },
            verb: { id: 'http://adlnet.gov/expapi/verbs/experienced' },
            object: {
                id: event.videoId,
                type: 'http://adlnet.gov/expapi/activities/media'
            },
            result: {
                duration: event.watchDuration,
                completion: event.percentWatched >= 90
            },
            timestamp: event.timestamp
        });
    }
    
    res.sendStatus(200);
});
```

### 7.2 Collaboration Tools (Slack, Teams)

```typescript
// Microsoft Teams bot for analytics alerts
const { TeamsActivityHandler } = require('botbuilder');

class AnalyticsBot extends TeamsActivityHandler {
    async onMessage(context) {
        const message = context.activity.text;
        
        if (message.startsWith('/analytics')) {
            const userId = context.activity.from.aadObjectId;
            const analytics = await getAnalyticsForUser(userId);
            
            const card = CardFactory.adaptiveCard({
                type: 'AdaptiveCard',
                body: [
                    {
                        type: 'TextBlock',
                        text: 'Your Learning Analytics',
                        weight: 'Bolder',
                        size: 'Large'
                    },
                    {
                        type: 'FactSet',
                        facts: [
                            { title: 'Completion Rate', value: `${analytics.completionRate}%` },
                            { title: 'Average Score', value: analytics.averageScore.toFixed(2) },
                            { title: 'Time This Week', value: `${analytics.timeThisWeek} hours` }
                        ]
                    }
                ]
            });
            
            await context.sendActivity({ attachments: [card] });
        }
    }
}
```

---

## 8. API Gateway Pattern

### 8.1 Unified API Gateway

```yaml
# Kong configuration
services:
  - name: analytics-api
    url: http://analytics-backend:8000
    routes:
      - name: analytics-route
        paths:
          - /api/v1/analytics
    plugins:
      - name: rate-limiting
        config:
          minute: 100
      - name: jwt
      - name: cors

  - name: lrs
    url: http://lrs:8080
    routes:
      - name: lrs-route
        paths:
          - /api/v1/lrs
```

---

## 9. Data Synchronization

### 9.1 Sync Strategies

**Full Sync (Weekly):**
- Complete dataset refresh
- Validates data integrity
- Resource-intensive

**Incremental Sync (Hourly):**
- Only changed records
- Efficient bandwidth usage
- Requires change tracking

**Real-time Sync (Continuous):**
- Webhook-driven updates
- Minimal latency
- Higher complexity

### 9.2 Conflict Resolution

```python
def resolve_conflict(local_record, remote_record):
    # Last-write-wins strategy
    if remote_record['updated_at'] > local_record['updated_at']:
        return remote_record
    else:
        return local_record
```

---

## 10. Integration Testing

### 10.1 End-to-End Test

```javascript
describe('LMS to Analytics Integration', () => {
    test('should sync course completion from Canvas', async () => {
        // 1. Create test course in Canvas
        const course = await canvas.createCourse({ name: 'Test Course' });
        
        // 2. Enroll test student
        await canvas.enrollUser(course.id, testStudent.id);
        
        // 3. Complete assignment
        await canvas.submitAssignment(course.id, assignment.id, { score: 95 });
        
        // 4. Wait for sync
        await sleep(5000);
        
        // 5. Verify analytics data
        const analytics = await getAnalytics(testStudent.id, course.id);
        expect(analytics.completionRate).toBe(1.0);
        expect(analytics.averageScore).toBe(0.95);
    });
});
```

---

## 11. Integration Checklist

- [ ] Document all integration points
- [ ] Implement authentication for each system
- [ ] Create data mapping documentation
- [ ] Build and test ETL pipelines
- [ ] Set up webhook handlers
- [ ] Configure API gateway
- [ ] Implement error handling and retries
- [ ] Create monitoring dashboards
- [ ] Test data synchronization
- [ ] Validate data quality post-integration
- [ ] Document troubleshooting procedures
- [ ] Train support staff

---

**Document Status:** ✅ Complete
**Next Phase:** Phase 4 - Optimization

---

© 2025 WIA - World Certification Industry Association
弘益人間 · Benefit All Humanity
