# WIA-DIGITAL_EXECUTOR PHASE 3 — Protocol Specification

**Standard:** WIA-DIGITAL_EXECUTOR
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

"properties": {
          "witnessName": {
            "type": "string"
          },
          "witnessSignature": {
            "type": "string"
          },
          "attestationDate": {
            "type": "string",
            "format": "date"
          },
          "witnessType": {
            "type": "string",
            "enum": ["witness", "notary", "court-official"]
          }
        }
      }
    },
    "amendments": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "amendmentId": {
            "type": "string"
          },
          "amendmentDate": {
            "type": "string",
            "format": "date"
          },
          "description": {
            "type": "string"
          },
          "documentReference": {
            "type": "string"
          }
        }
      }
    }
  }
}
```

---

## 7. Progress Tracking Schema

### 7.1 Progress Report Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Executor Progress Report",
  "type": "object",
  "required": ["reportId", "executorId", "reportPeriod", "generatedAt"],
  "properties": {
    "reportId": {
      "type": "string",
      "format": "uuid"
    },
    "executorId": {
      "type": "string",
      "format": "uuid"
    },
    "reportPeriod": {
      "type": "object",
      "required": ["startDate", "endDate"],
      "properties": {
        "startDate": {
          "type": "string",
          "format": "date"
        },
        "endDate": {
          "type": "string",
          "format": "date"
        }
      }
    },
    "generatedAt": {
      "type": "string",
      "format": "date-time"
    },
    "summary": {
      "type": "object",
      "properties": {
        "totalTasks": {
          "type": "integer"
        },
        "completedTasks": {
          "type": "integer"
        },
        "inProgressTasks": {
          "type": "integer"
        },
        "blockedTasks": {
          "type": "integer"
        },
        "platformsAccessed": {
          "type": "integer"
        },
        "dataDownloaded": {
          "type": "string",
          "description": "Size in bytes"
        },
        "accountsClosed": {
          "type": "integer"
        },
        "assetsTransferred": {
          "type": "integer"
        }
      }
    },
    "taskBreakdown": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "category": {
            "type": "string"
          },
          "total": {
            "type": "integer"
          },
          "completed": {
            "type": "integer"
          },
          "completionRate": {
            "type": "number",
            "minimum": 0,
            "maximum": 100
          }
        }
      }
    },
    "platformProgress": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "platform": {
            "type": "string"
          },
          "status": {
            "type": "string",
            "enum": ["not-started", "in-progress", "completed", "blocked"]
          },
          "tasksCompleted": {
            "type": "integer"
          },
          "totalTasks": {
            "type": "integer"
          },
          "lastActivity": {
            "type": "string",
            "format": "date-time"
          }
        }
      }
    },
    "milestones": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "milestone": {
            "type": "string"
          },
          "targetDate": {
            "type": "string",
            "format": "date"
          },
          "actualDate": {
            "type": "string",
            "format": "date"
          },
          "status": {
            "type": "string",
            "enum": ["pending", "on-track", "at-risk", "delayed", "completed"]
          }
        }
      }
    },
    "issues": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "issueId": {
            "type": "string"
          },
          "severity": {
            "type": "string",
            "enum": ["low", "medium", "high", "critical"]
          },
          "description": {
            "type": "string"
          },
          "impact": {
            "type": "string"
          },
          "resolution": {
            "type": "string"
          },
          "status": {
            "type": "string",
            "enum": ["open", "in-progress", "resolved"]
          }
        }
      }
    },
    "beneficiaryUpdates": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "beneficiaryId": {
            "type": "string"
          },
          "updateSent": {
            "type": "string",
            "format": "date-time"
          },
          "method": {
            "type": "string",
            "enum": ["email", "portal", "mail", "phone"]
          },
          "acknowledged": {
            "type": "boolean"
          }
        }
      }
    },
    "complianceStatus": {
      "type": "object",
      "properties": {
        "legalRequirementsMet": {
          "type": "integer"
        },
        "totalLegalRequirements": {
          "type": "integer"
        },
        "outstandingFilings": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "filing": {
                "type": "string"
              },
              "deadline": {
                "type": "string",
                "format": "date"
              },
              "status": {
                "type": "string"
              }
            }
          }
        }
      }
    },
    "timeTracking": {
      "type": "object",
      "properties": {
        "totalHours": {
          "type": "number"
        },
        "hoursByCategory": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "category": {
                "type": "string"
              },
              "hours": {
                "type": "number"
              }
            }
          }
        }
      }
    },
    "nextSteps": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "action": {
            "type": "string"
          },
          "targetDate": {
            "type": "string",
            "format": "date"
          },
          "assignedTo": {
            "type": "string"
          }
        }
      }
    }
  }
}
```

---

## 8. Communication Schema

### 8.1 Beneficiary Communication Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Beneficiary Communication",
  "type": "object",
  "required": ["communicationId", "fromExecutor", "toBeneficiaries", "subject", "sentAt"],
  "properties": {
    "communicationId": {
      "type": "string",
      "format": "uuid"
    },
    "fromExecutor": {
      "type": "string",
      "format": "uuid"
    },
    "toBeneficiaries": {
      "type": "array",
      "items": {
        "type": "string",
        "format": "uuid"
      }
    },
    "ccExecutors": {
      "type": "array",
      "items": {
        "type": "string",
        "format": "uuid"
      }
    },
    "subject": {
      "type": "string"
    },
    "messageType": {
      "type": "string",
      "enum": ["initial-notification", "progress-update", "asset-distribution", "account-closure", "action-required", "completion-notice"]
    },
    "content": {
      "type": "object",
      "properties": {
        "body": {
          "type": "string"
        },
        "format": {
          "type": "string",
          "enum": ["plain-text", "html", "markdown"]
        },
        "language": {
          "type": "string",
          "pattern": "^[a-z]{2}$"
        }
      }
    },
    "attachments": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "fileId": {
            "type": "string"
          },
          "fileName": {
            "type": "string"
          },
          "fileType": {
            "type": "string"
          },
          "description": {
            "type": "string"
          }
        }
      }
    },
    "sentAt": {
      "type": "string",
      "format": "date-time"
    },
    "deliveryMethod": {
      "type": "string",
      "enum": ["email", "portal", "postal-mail", "sms", "secure-message"]
    },
    "deliveryStatus": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "beneficiaryId": {
            "type": "string",
            "format": "uuid"
          },
          "status": {
            "type": "string",
            "enum": ["sent", "delivered", "read", "acknowledged", "bounced", "failed"]
          },
          "timestamp": {
            "type": "string",
            "format": "date-time"
          }
        }
      }
    },
    "requiresResponse": {
      "type": "boolean"
    },
    "responseDeadline": {
      "type": "string",
      "format": "date-time"
    },
    "responses": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "beneficiaryId": {
            "type": "string",
            "format": "uuid"
          },
          "response": {
            "type": "string"
          },
          "respondedAt": {
            "type": "string",
            "format": "date-time"
          }
        }
      }
    },
    "template": {
      "type": "object",
      "properties": {
        "templateId": {
          "type": "string"
        },
        "variables": {
          "type": "object"
        }
      }
    },
    "compliance": {
      "type": "object",
      "properties": {
        "retentionPeriod": {
          "type": "string",
          "description": "ISO 8601 duration"
        },
        "privacyLevel": {
          "type": "string",
          "enum": ["public", "confidential", "restricted"]
        },
        "archiveRequired": {
          "type": "boolean"
        }
      }
    }
  }
}
```

---

## 9. Audit Trail Schema

### 9.1 Audit Log Entry Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Executor Audit Log Entry",
  "type": "object",
  "required": ["logId", "executorId", "action", "timestamp"],
  "properties": {
    "logId": {
      "type": "string",
      "format": "uuid"
    },
    "executorId": {
      "type": "string",
      "format": "uuid"
    },
    "action": {
      "type": "string",
      "enum": [
        "login",
        "logout",
        "view-asset",
        "download-data",
        "delete-content",
        "transfer-asset",
        "close-account",
        "update-task",
        "send-communication",
        "update-permissions",
        "access-credentials",
        "upload-document",
        "verify-legal-authority",
        "generate-report",
        "delegate-authority"
      ]
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "platform": {
      "type": "string"
    },
    "accountId": {
      "type": "string"
    },
    "details": {
      "type": "object",
      "properties": {
        "description": {
          "type": "string"
        },
        "affectedResources": {
          "type": "array",
          "items": {
            "type": "string"
          }
        },
        "previousState": {
          "type": "object"
        },
        "newState": {
          "type": "object"
        }
      }
    },
    "context": {
      "type": "object",
      "properties": {
        "ipAddress": {
          "type": "string"
        },
        "userAgent": {
          "type": "string"
        },
        "location": {
          "type": "object",
          "properties": {
            "country": {
              "type": "string"
            },
            "city": {
              "type": "string"
            },
            "coordinates": {
              "type": "object",
              "properties": {
                "latitude": {
                  "type": "number"
                },
                "longitude": {
                  "type": "number"
                }
              }
            }
          }
        },
        "sessionId": {
          "type": "string"
        },
        "requestId": {
          "type": "string"
        }
      }
    },
    "result": {
      "type": "string",
      "enum": ["success", "failure", "partial", "blocked"]
    },
    "errorDetails": {
      "type": "object",
      "properties": {
        "errorCode": {
          "type": "string"
        },
        "errorMessage": {
          "type": "string"
        },
        "stackTrace": {
          "type": "string"
        }
      }
    },
    "securityFlags": {
      "type": "array",
      "items": {
        "type": "string",
        "enum": ["suspicious-location", "unusual-time", "rapid-actions", "high-risk-action", "unauthorized-attempt"]
      }
    },
    "approvals": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "approverExecutorId": {
            "type": "string"
          },
          "approvedAt": {
            "type": "string",
            "format": "date-time"
          }
        }
      }
    },
    "relatedLogs": {
      "type": "array",
      "items": {
        "type": "string",
        "format": "uuid"
      }
    },
    "immutabilityProof": {
      "type": "object",
      "properties": {
        "hash": {
          "type": "string"
        },
        "previousHash": {
          "type": "string"
        },
        "signature": {
          "type": "string"
        }
      }
    }
  }
}
```

---

## 10. Data Validation Rules

### 10.1 Validation Requirements

| Data Type | Validation Rules | Error Handling | Example |
|-----------|-----------------|----------------|---------|
| **Executor ID** | UUID v4 format, exists in database | Return 404 if not found | `550e8400-e29b-41d4-a716-446655440000` |
| **Email Address** | RFC 5322 compliant, verified | Reject unverified emails | `executor@example.com` |
| **Phone Number** | E.164 format, verified | Require verification code | `+14155552671` |
| **Date Fields** | ISO 8601, not in future (except deadlines) | Reject invalid dates | `2025-12-18T10:30:00Z` |
| **Legal Documents** | Valid checksum, verified status | Require re-verification | SHA-256 hash |
| **Access Permissions** | Valid role + permission combination | Deny invalid combinations | Primary executor + full access |

### 10.2 Business Logic Validation

```javascript
// Validation Rules Examples
const validationRules = {
  executorAppointment: {
    mustHaveLegalAuthority: true,
    mustBeAdult: true, // Age >= 18
    cannotBeBeneficiaryOnly: false, // Can be both
    requiresBackupExecutor: true,
    minimumExecutors: 1,
    maximumCoExecutors: 5
  },

  taskAssignment: {
    mustMatchExecutorSpecialization: false, // Recommended but not required
    cannotExceedPermissions: true,
    requiresApprovalForCritical: true,
    mustHaveDeadline: true,
    cannotBePastDue: false // Can assign past due tasks
  },

  accessGrant: {
    requiresLegalDocumentation: true,
    mustHaveExpiryDate: true,
    cannotExceedExecutorPermissions: true,
    requiresPlatformApproval: true,
    mustLogAllAccess: true
  },

  dataTransfer: {
    requiresEncryption: true,
    mustVerifyRecipient: true,
    requiresAuditTrail: true,
    mustNotifyStakeholders: true,
    cannotTransferToUnverifiedExecutor: true
  }
};
```

### 10.3 Data Integrity Checks

```javascript
// Data Integrity Verification
class DataIntegrityValidator {
  static validateExecutorProfile(profile) {
    const checks = {
