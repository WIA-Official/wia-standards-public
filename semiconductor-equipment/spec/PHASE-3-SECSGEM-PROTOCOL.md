# WIA-SEMI-019 - Phase 3: SECS/GEM Protocol

> **Version:** 1.0  
> **Status:** Active  
> **Last Updated:** 2025-12-26

## 1. Overview

Phase 3 extends SEMI E5 (SECS-II), E30 (GEM), and E37 (HSMS) with WIA-specific enhancements while maintaining full backward compatibility with existing standards.

## 2. WIA SECS Messages

### 2.1 Stream 9 Extensions (S9F100-S9F199)

#### S9F100/S9F101 - WIA Equipment Specification

```
S9F100 W  // Request
  <L[0]>

S9F101    // Response
  <L[5]
    <A "WIA-SEMI-019">      // Standard
    <A "1.0">               // Version
    <A "ASML">              // Manufacturer
    <A "TWINSCAN-3600D">    // Model
    <A "{...}">             // Capabilities JSON
  >
```

#### S9F110/S9F111 - Enhanced Parameter Subscribe

```
S9F110 W  // Subscribe
  <L[3]
    <L[2]                   // Parameters
      <A "substrate_temp">
      <A "chamber_pressure">
    >
    <U4 100>                // Frequency Hz
    <A "BINARY">            // Format
  >

S9F111    // Response
  <L[2]
    <BOOLEAN TRUE>          // Accepted
    <A "STREAM_12345">      // Stream ID
  >
```

#### S9F120/S9F121 - Recipe Transfer

```
S9F120 W  // Upload Recipe
  <L[4]
    <A "RECIPE_ID">
    <A "1.0">               // Version
    <B ...>                 // Recipe data
    <A "sha256:...">        // Checksum
  >

S9F121    // Response
  <L[3]
    <BOOLEAN TRUE>          // Valid
    <A "STORED">
    <A "/recipes/...">      // Path
  >
```

## 3. Standardized SVID Ranges

| Range | Category | Examples |
|-------|----------|----------|
| 4000-4999 | Temperature | 4001: substrate_temp |
| 5000-5999 | Pressure | 5001: chamber_pressure |
| 6000-6999 | Flow | 6001: N2_flow |
| 7000-7999 | Power/RF | 7001: rf_power_fwd |
| 8000-8999 | Motion | 8001: robot_x |
| 9000-9999 | Metrology | 9001: thickness |

## 4. Enhanced Event Reporting

### 4.1 S6F11 - WIA Event Report

```
S6F11 W  // Event with context
  <L[7]
    <U4 1001>                    // CEID
    <A "WAFER_PROCESS_START">    // Event name
    <A "2025-12-26T15:30:45Z">   // Timestamp
    <L[3]                        // Context
      <L[2] <A "wafer_id"> <A "W123"> >
      <L[2] <A "lot_id"> <A "LOT001"> >
      <L[2] <A "recipe_id"> <A "RCP001"> >
    >
    <L[5]                        // Parameter snapshot
      <L[2] <A "temp"> <F4 425.0> >
      ...
    >
    <A "INFO">                   // Severity
    <A "CORR_12345">             // Correlation ID
  >
```

## 5. HSMS-TLS Security

### 5.1 TLS Configuration

- TLS 1.3 required
- Certificate-based authentication
- Encrypted SECS message payload
- Standard HSMS port 5000 (configurable)

### 5.2 S9F130/S9F131 - Authentication

```
S9F130 W  // Authenticate
  <L[3]
    <A "username">
    <A "role">
    <B ...>                 // Challenge response
  >

S9F131    // Response
  <L[4]
    <BOOLEAN TRUE>          // Success
    <A "SESSION_ABC">       // Token
    <U4 3600>               // Timeout
    <L[3]                   // Permissions
      <A "equipment.read">
      <A "command.start">
      <A "command.stop">
    >
  >
```

## 6. Process Job Management

### 6.1 S16F11/S16F12 - Create Process Job

```
S16F11 W  // Create Job
  <L[6]
    <A "JOB-001">           // Job ID
    <L[25] ... >            // Wafer IDs
    <A "RECIPE_ID">
    <L[5]                   // WIA extensions
      <L[2] <A "lot_id"> <A "LOT001"> >
      <L[2] <A "priority"> <U1 5> >
      <L[2] <A "correlation_id"> <A "MES-123"> >
      <L[2] <A "quality_level"> <A "PRODUCTION"> >
      <L[2] <A "metadata"> <A "{...}"> >
    >
    <BOOLEAN TRUE>          // Start immediate
    <A "2025-12-31T23:59Z"> // Expiration
  >

S16F12    // Response
  <BOOLEAN 0x00>  // Accepted
```

## 7. Alarm Management

### 7.1 S5F1 - Enhanced Alarm Report

```
S5F1 W  // Alarm with diagnostics
  <L[7]
    <U4 5001>                     // ALID
    <A "TEMP_EXCURSION_HIGH">     // Code
    <A "WARNING">                 // Severity
    <A "Temp exceeded by 5C">     // Message
    <A "2025-12-26T15:25:30Z">    // Timestamp
    <L[3]                         // Diagnostic data
      <L[2] <A "current"> <F4 430.2> >
      <L[2] <A "setpoint"> <F4 425.0> >
      <L[2] <A "deviation"> <F4 5.2> >
    >
    <L[2]                         // Actions
      <A "Check heater controller">
      <A "Verify cooling water flow">
    >
  >
```

## 8. Gold Certification Requirements

1. Implement all WIA SECS messages (S9F100-S9F199)
2. Use standardized SVID ranges for common parameters
3. Enhanced event reporting with context and correlation IDs
4. Support HSMS-TLS with certificate authentication
5. Process job management with WIA extensions
6. Enhanced alarm reporting with diagnostic data
7. Full backward compatibility with SEMI E5/E30/E37

---

**弘益人間 · Benefit All Humanity**

*WIA - World Certification Industry Association*  
*© 2025 MIT License*
