# WIA-SEC-022: Ransomware Protection Standard
## APPENDIX - IMPLEMENTATION GUIDES & REFERENCE

**Standard ID:** WIA-SEC-022
**Version:** 1.0.0
**Last Updated:** 2025-12-25

---

## A. Quick Start Implementation Guide

### A.1 30-Day Ransomware Protection Deployment

**Week 1: Assessment & Planning**
```markdown
Day 1-2: Current State Assessment
- [ ] Inventory all critical systems and data
- [ ] Document existing backup solutions
- [ ] Identify security gaps
- [ ] Map network topology

Day 3-4: Design Protection Architecture
- [ ] Define backup strategy (3-2-1-1-0 rule)
- [ ] Select WORM storage solution
- [ ] Design network segmentation
- [ ] Plan endpoint agent deployment

Day 5-7: Stakeholder Alignment
- [ ] Present plan to leadership
- [ ] Budget approval
- [ ] Assign roles and responsibilities
- [ ] Create project timeline
```

**Week 2: Infrastructure Deployment**
```markdown
Day 8-10: Backup Infrastructure
- [ ] Deploy WORM storage (S3 Object Lock / Azure Immutable)
- [ ] Configure encryption (AES-256-GCM)
- [ ] Set retention policies (90 days)
- [ ] Test backup creation

Day 11-12: Network Hardening
- [ ] Implement firewall rules (block workstation-to-workstation)
- [ ] Deploy network segmentation
- [ ] Configure VLANs for critical systems
- [ ] Enable logging on all network devices

Day 13-14: Endpoint Protection
- [ ] Deploy endpoint agents to test group
- [ ] Configure behavioral detection rules
- [ ] Set up centralized management console
- [ ] Validate agent communication
```

**Week 3: Detection & Response**
```markdown
Day 15-17: Behavioral Detection
- [ ] Configure file system monitoring rules
- [ ] Set entropy thresholds (>7.8 = alert)
- [ ] Enable process behavior analysis
- [ ] Configure automated response actions

Day 18-19: SIEM Integration
- [ ] Connect to SIEM platform
- [ ] Configure alert forwarding (CEF/JSON)
- [ ] Create correlation rules
- [ ] Set up dashboards

Day 20-21: Threat Intelligence
- [ ] Integrate threat feeds (Abuse.ch, OTX)
- [ ] Configure IOC matching
- [ ] Enable automatic blocking
- [ ] Test feed updates
```

**Week 4: Testing & Training**
```markdown
Day 22-24: Testing
- [ ] Ransomware simulation test (safe simulator)
- [ ] Verify detection (<100ms)
- [ ] Validate automated response
- [ ] Test backup restoration (verify RTO <15 min)

Day 25-27: User Training
- [ ] Conduct phishing awareness training
- [ ] Explain incident reporting procedures
- [ ] Train IT staff on response playbooks
- [ ] Distribute security guidelines

Day 28-30: Final Validation
- [ ] Full-scale tabletop exercise
- [ ] Review and adjust policies
- [ ] Document procedures
- [ ] Go-live approval
```

---

## B. Sample Configurations

### B.1 AWS S3 Immutable Backup Configuration

```python
import boto3
from datetime import datetime, timedelta

def configure_immutable_backup_bucket():
    """
    Configure S3 bucket with Object Lock for immutable backups
    """
    s3 = boto3.client('s3')

    bucket_name = 'ransomware-protection-backups'

    # Create bucket with Object Lock enabled
    s3.create_bucket(
        Bucket=bucket_name,
        ObjectLockEnabledForBucket=True
    )

    # Configure Object Lock default retention
    s3.put_object_lock_configuration(
        Bucket=bucket_name,
        ObjectLockConfiguration={
            'ObjectLockEnabled': 'Enabled',
            'Rule': {
                'DefaultRetention': {
                    'Mode': 'COMPLIANCE',  # Cannot be deleted even by root
                    'Days': 90
                }
            }
        }
    )

    # Enable versioning (required for Object Lock)
    s3.put_bucket_versioning(
        Bucket=bucket_name,
        VersioningConfiguration={'Status': 'Enabled'}
    )

    # Enable encryption
    s3.put_bucket_encryption(
        Bucket=bucket_name,
        ServerSideEncryptionConfiguration={
            'Rules': [{
                'ApplyServerSideEncryptionByDefault': {
                    'SSEAlgorithm': 'AES256'
                },
                'BucketKeyEnabled': True
            }]
        }
    )

    # Configure lifecycle policy
    s3.put_bucket_lifecycle_configuration(
        Bucket=bucket_name,
        LifecycleConfiguration={
            'Rules': [{
                'ID': 'archive-old-backups',
                'Status': 'Enabled',
                'Transitions': [{
                    'Days': 30,
                    'StorageClass': 'GLACIER'
                }],
                'Expiration': {
                    'Days': 365  # Delete after 1 year
                }
            }]
        }
    )

    print(f"Immutable backup bucket configured: {bucket_name}")

# Upload backup with object lock
def upload_backup(file_path, backup_id):
    s3 = boto3.client('s3')
    bucket_name = 'ransomware-protection-backups'

    retention_date = datetime.now() + timedelta(days=90)

    s3.upload_file(
        file_path,
        bucket_name,
        f'backups/{backup_id}',
        ExtraArgs={
            'ObjectLockMode': 'COMPLIANCE',
            'ObjectLockRetainUntilDate': retention_date,
            'ServerSideEncryption': 'AES256',
            'Metadata': {
                'backup-id': backup_id,
                'created': datetime.now().isoformat(),
                'standard': 'WIA-SEC-022'
            }
        }
    )

    print(f"Backup uploaded with immutability until {retention_date}")
```

---

### B.2 Linux Endpoint Agent Configuration

```bash
#!/bin/bash
# WIA-SEC-022 Linux Endpoint Agent Setup

# Install dependencies
sudo apt-get update
sudo apt-get install -y auditd inotify-tools python3-pip

# Install Python libraries
pip3 install watchdog psutil requests

# Create agent configuration
cat > /etc/wia-sec-022/agent.conf << EOF
[detection]
monitor_paths = /home,/var,/opt
exclude_paths = /proc,/sys,/dev
entropy_threshold = 7.8
file_change_threshold = 100
time_window = 60

[response]
auto_isolate = true
auto_terminate = true
create_snapshot = true
alert_endpoint = https://soc.company.com/api/alerts

[backup]
verify_integrity = true
check_frequency = 3600
immutable_mount = /mnt/backup-worm
EOF

# Create monitoring script
cat > /usr/local/bin/wia-ransomware-monitor.py << 'PYTHON'
#!/usr/bin/env python3
import os
import sys
import time
import hashlib
import subprocess
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import psutil
import requests

class RansomwareDetector(FileSystemEventHandler):
    def __init__(self):
        self.file_changes = {}
        self.alert_threshold = 100
        self.time_window = 60

    def calculate_entropy(self, file_path):
        """Calculate Shannon entropy of file"""
        try:
            with open(file_path, 'rb') as f:
                data = f.read(1024 * 1024)  # Read first 1MB
                if not data:
                    return 0

                entropy = 0
                for x in range(256):
                    p_x = data.count(bytes([x])) / len(data)
                    if p_x > 0:
                        entropy += - p_x * (p_x.bit_length() - 1)
                return entropy
        except:
            return 0

    def on_modified(self, event):
        if event.is_directory:
            return

        current_time = time.time()
        entropy = self.calculate_entropy(event.src_path)

        # Track file changes
        self.file_changes[event.src_path] = {
            'timestamp': current_time,
            'entropy': entropy
        }

        # Clean old entries
        self.file_changes = {
            k: v for k, v in self.file_changes.items()
            if current_time - v['timestamp'] < self.time_window
        }

        # Check for ransomware indicators
        recent_changes = len(self.file_changes)
        avg_entropy = sum(v['entropy'] for v in self.file_changes.values()) / max(recent_changes, 1)

        if recent_changes > self.alert_threshold and avg_entropy > 7.8:
            self.trigger_alert("RANSOMWARE_DETECTED", {
                'files_affected': recent_changes,
                'average_entropy': avg_entropy,
                'time_window': self.time_window
            })

    def trigger_alert(self, alert_type, details):
        """Send alert and initiate response"""
        alert = {
            'type': alert_type,
            'timestamp': time.time(),
            'hostname': os.uname().nodename,
            'details': details
        }

        # Send to SOC
        requests.post('https://soc.company.com/api/alerts', json=alert)

        # Automated response
        self.initiate_response()

    def initiate_response(self):
        """Execute automated response actions"""
        # 1. Create filesystem snapshot
        subprocess.run(['lvcreate', '-L', '10G', '-s', '-n', 'snap-emergency', '/dev/vg0/root'])

        # 2. Network isolation
        subprocess.run(['iptables', '-I', 'OUTPUT', '-j', 'DROP'])

        # 3. Kill suspicious processes
        for proc in psutil.process_iter(['pid', 'name']):
            # Add logic to identify malicious processes
            pass

        print("EMERGENCY RESPONSE EXECUTED")

if __name__ == '__main__':
    detector = RansomwareDetector()
    observer = Observer()
    observer.schedule(detector, '/home', recursive=True)
    observer.schedule(detector, '/var', recursive=True)
    observer.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()
PYTHON

chmod +x /usr/local/bin/wia-ransomware-monitor.py

# Create systemd service
cat > /etc/systemd/system/wia-ransomware-protection.service << EOF
[Unit]
Description=WIA-SEC-022 Ransomware Protection Agent
After=network.target

[Service]
Type=simple
ExecStart=/usr/local/bin/wia-ransomware-monitor.py
Restart=always
User=root

[Install]
WantedBy=multi-user.target
EOF

# Enable and start service
systemctl daemon-reload
systemctl enable wia-ransomware-protection
systemctl start wia-ransomware-protection

echo "WIA-SEC-022 agent installed and running"
```

---

### B.3 Windows Endpoint Agent (PowerShell)

```powershell
# WIA-SEC-022 Windows Ransomware Protection
# Run as Administrator

# Create monitoring directory
New-Item -Path "C:\Program Files\WIA-SEC-022" -ItemType Directory -Force

# Install monitoring script
$MonitorScript = @'
# Ransomware Detection Script
$ErrorActionPreference = "SilentlyContinue"

# Configuration
$MonitorPaths = @("C:\Users", "C:\Data")
$EntropyThreshold = 7.8
$FileChangeThreshold = 100
$TimeWindow = 60

# File change tracking
$FileChanges = @{}

function Calculate-Entropy {
    param([string]$FilePath)

    try {
        $bytes = [System.IO.File]::ReadAllBytes($FilePath)
        if ($bytes.Length -eq 0) { return 0 }

        $freq = @{}
        foreach ($byte in $bytes) {
            if ($freq.ContainsKey($byte)) {
                $freq[$byte]++
            } else {
                $freq[$byte] = 1
            }
        }

        $entropy = 0
        foreach ($count in $freq.Values) {
            $p = $count / $bytes.Length
            $entropy -= $p * [Math]::Log($p, 2)
        }

        return $entropy
    } catch {
        return 0
    }
}

function Trigger-Alert {
    param($AlertData)

    # Send to SIEM
    $json = $AlertData | ConvertTo-Json
    Invoke-WebRequest -Uri "https://soc.company.com/api/alerts" `
        -Method POST `
        -Body $json `
        -ContentType "application/json"

    # Execute response
    Execute-Response
}

function Execute-Response {
    Write-Host "RANSOMWARE DETECTED - EXECUTING EMERGENCY RESPONSE"

    # 1. Create VSS snapshot
    $shadow = (Get-WmiObject -List Win32_ShadowCopy).Create("C:\", "ClientAccessible")

    # 2. Terminate suspicious processes
    Get-Process | Where-Object {
        $_.Path -like "*\AppData\Local\Temp\*" -or
        $_.Path -like "*\Users\*\Downloads\*"
    } | Stop-Process -Force

    # 3. Network isolation
    Disable-NetAdapter -Name "*" -Confirm:$false

    # 4. Alert user
    msg * "RANSOMWARE DETECTED - System isolated. Contact IT Security immediately."
}

# File System Watcher
$watcher = New-Object System.IO.FileSystemWatcher
$watcher.Path = "C:\Users"
$watcher.IncludeSubdirectories = $true
$watcher.EnableRaisingEvents = $true

$action = {
    $path = $Event.SourceEventArgs.FullPath
    $changeType = $Event.SourceEventArgs.ChangeType

    if ($changeType -eq "Changed" -or $changeType -eq "Created") {
        $entropy = Calculate-Entropy -FilePath $path
        $now = Get-Date

        # Track change
        $global:FileChanges[$path] = @{
            Timestamp = $now
            Entropy = $entropy
        }

        # Remove old entries
        $cutoff = $now.AddSeconds(-$TimeWindow)
        $global:FileChanges = $global:FileChanges.GetEnumerator() |
            Where-Object { $_.Value.Timestamp -gt $cutoff } |
            ForEach-Object { @{$_.Key = $_.Value} }

        # Check for ransomware
        $recentChanges = $global:FileChanges.Count
        $avgEntropy = ($global:FileChanges.Values.Entropy | Measure-Object -Average).Average

        if ($recentChanges -gt $FileChangeThreshold -and $avgEntropy -gt $EntropyThreshold) {
            Trigger-Alert @{
                Type = "RANSOMWARE_DETECTED"
                FilesAffected = $recentChanges
                AverageEntropy = $avgEntropy
                Hostname = $env:COMPUTERNAME
                Timestamp = $now
            }
        }
    }
}

Register-ObjectEvent -InputObject $watcher -EventName "Changed" -Action $action
Register-ObjectEvent -InputObject $watcher -EventName "Created" -Action $action

# Keep script running
while ($true) {
    Start-Sleep -Seconds 1
}
'@

Set-Content -Path "C:\Program Files\WIA-SEC-022\monitor.ps1" -Value $MonitorScript

# Create scheduled task to run at startup
$action = New-ScheduledTaskAction -Execute "PowerShell.exe" `
    -Argument "-ExecutionPolicy Bypass -File `"C:\Program Files\WIA-SEC-022\monitor.ps1`""

$trigger = New-ScheduledTaskTrigger -AtStartup

$principal = New-ScheduledTaskPrincipal -UserId "SYSTEM" -LogonType ServiceAccount -RunLevel Highest

Register-ScheduledTask -TaskName "WIA-SEC-022-RansomwareProtection" `
    -Action $action `
    -Trigger $trigger `
    -Principal $principal `
    -Description "WIA-SEC-022 Ransomware Protection Agent"

Write-Host "WIA-SEC-022 agent installed successfully"
```

---

## C. Ransomware Family Reference

### C.1 Major Ransomware Families (2020-2025)

| Family | First Seen | Encryption | C2 Method | Typical IOCs |
|--------|-----------|------------|-----------|--------------|
| **LockBit 3.0** | 2022-06 | AES-256 + RSA-2048 | Tor, I2P | `.lockbit`, `Restore-My-Files.txt` |
| **REvil/Sodinokibi** | 2019-04 | Salsa20 + Curve25519 | Tor | `.sodinokibi`, ransom notes in JSON |
| **Ryuk** | 2018-08 | AES-256 + RSA-4096 | Direct IP | `.RYK`, `RyukReadMe.txt` |
| **Conti** | 2020-02 | ChaCha + RSA-4096 | Tor | `.CONTI`, `readme.txt` |
| **BlackCat/ALPHV** | 2021-11 | AES + RSA (Rust-based) | Tor | Various extensions, multilingual notes |
| **Hive** | 2021-06 | Custom stream cipher | Tor | `.hive`, `HOW_TO_DECRYPT.txt` |
| **DarkSide** | 2020-08 | Salsa20 + RSA-1024 | Tor | Random extensions, professional notes |

---

### C.2 Common Attack Vectors

```markdown
1. Phishing Emails (40% of attacks)
   - Malicious attachments (.doc, .xls with macros)
   - Malicious links to exploit kits
   - Credential harvesting for RDP access

2. RDP Exploitation (30% of attacks)
   - Brute force attacks on exposed RDP
   - Credential stuffing with leaked passwords
   - Exploits (BlueKeep CVE-2019-0708)

3. Software Vulnerabilities (20% of attacks)
   - Unpatched VPN appliances (Fortinet, Pulse Secure)
   - Web application vulnerabilities (SQL injection, RCE)
   - Zero-day exploits

4. Supply Chain Attacks (5% of attacks)
   - Compromised software updates (SolarWinds-style)
   - Trojanized installers
   - Malicious browser extensions

5. Other (5%)
   - USB/removable media
   - Watering hole attacks
   - Insider threats
```

---

## D. Recovery Checklists

### D.1 Immediate Response Checklist

```markdown
⏰ First 5 Minutes (Critical Actions)
- [ ] Confirm ransomware detection (verify alert)
- [ ] Isolate affected systems from network
- [ ] Document current time and observations
- [ ] Notify incident response team
- [ ] Do NOT pay ransom immediately

⏰ First 30 Minutes (Containment)
- [ ] Identify patient zero (initial infection point)
- [ ] Scan entire network for lateral movement
- [ ] Disable compromised accounts
- [ ] Block C2 domains/IPs at firewall
- [ ] Capture memory dumps for forensics
- [ ] Verify backup integrity (check for encrypted backups)

⏰ First Hour (Assessment)
- [ ] Determine scope of infection
- [ ] Identify affected data and systems
- [ ] Classify by business impact
- [ ] Assess backup availability
- [ ] Determine recovery strategy
- [ ] Notify stakeholders (management, legal, PR)

⏰ First 4 Hours (Eradication & Recovery)
- [ ] Remove ransomware from all systems
- [ ] Patch vulnerabilities that allowed entry
- [ ] Restore critical systems from clean backups
- [ ] Verify restored data integrity
- [ ] Implement enhanced monitoring
- [ ] Gradual reconnection to network

⏰ 24-48 Hours (Post-Incident)
- [ ] Complete forensic investigation
- [ ] Document lessons learned
- [ ] Update incident response procedures
- [ ] Enhance security controls
- [ ] Provide status updates to stakeholders
- [ ] Consider regulatory reporting (GDPR breach notification, etc.)
```

---

### D.2 Backup Restoration Checklist

```markdown
Pre-Restoration
- [ ] Verify threat has been eradicated
- [ ] Confirm no backdoors remain
- [ ] Identify clean backup point (before infection)
- [ ] Calculate estimated restoration time
- [ ] Prepare temporary systems if needed

Restoration Process
- [ ] Restore to isolated environment first (not production)
- [ ] Verify file integrity (hash comparison)
- [ ] Scan restored data for malware
- [ ] Test critical applications
- [ ] Verify data completeness
- [ ] Document any data loss (RPO measurement)

Post-Restoration
- [ ] Gradual cutover to production
- [ ] Monitor for 72 hours minimum
- [ ] Watch for re-infection attempts
- [ ] Verify business operations
- [ ] Update documentation
- [ ] Conduct post-mortem review
```

---

## E. Compliance Mapping

### E.1 NIST Cybersecurity Framework Mapping

| NIST Function | NIST Category | WIA-SEC-022 Control |
|---------------|---------------|---------------------|
| **IDENTIFY** | Asset Management (ID.AM) | System inventory, data classification |
| **PROTECT** | Data Security (PR.DS) | Immutable backups, encryption (AES-256) |
| **PROTECT** | Protective Technology (PR.PT) | Endpoint agents, behavioral detection |
| **DETECT** | Anomalies and Events (DE.AE) | Entropy analysis, UEBA |
| **DETECT** | Continuous Monitoring (DE.CM) | Real-time file system monitoring |
| **RESPOND** | Response Planning (RS.RP) | Automated playbooks, SOAR integration |
| **RESPOND** | Mitigation (RS.MI) | Automated isolation, process termination |
| **RECOVER** | Recovery Planning (RC.RP) | RTO/RPO targets, prioritized recovery |
| **RECOVER** | Improvements (RC.IM) | Post-incident analysis, continuous improvement |

---

### E.2 ISO 27001 Controls Mapping

| ISO 27001 Control | Description | WIA-SEC-022 Implementation |
|-------------------|-------------|----------------------------|
| **A.12.3.1** | Information backup | 3-2-1-1-0 rule, immutable WORM storage |
| **A.16.1.2** | Reporting information security events | SIEM integration, automated alerting |
| **A.16.1.4** | Assessment of information security events | Behavioral detection, threat intelligence |
| **A.16.1.5** | Response to information security incidents | Automated playbooks, network isolation |
| **A.17.1.1** | Planning information security continuity | RTO/RPO planning, tabletop exercises |
| **A.17.1.2** | Implementing information security continuity | Backup testing, DR drills |
| **A.18.1.3** | Protection of records | Immutable storage, compliance retention |

---

## F. Additional Resources

### F.1 Recommended Tools

**Backup Solutions:**
- Veeam Backup & Replication (WORM support)
- Commvault Complete Backup & Recovery
- Rubrik Cloud Data Management
- AWS Backup with S3 Object Lock
- Azure Backup with immutable vaults

**Endpoint Protection:**
- CrowdStrike Falcon
- SentinelOne
- Microsoft Defender for Endpoint
- Sophos Intercept X
- Carbon Black

**SIEM Platforms:**
- Splunk Enterprise Security
- IBM QRadar
- Microsoft Sentinel
- Elastic Security
- LogRhythm

**Ransomware Simulators (for testing):**
- KnowBe4 RanSim
- Cymulate Ransomware Simulator
- SafeBreach Ransomware Simulation
- AttackIQ Security Optimization Platform

---

### F.2 Training Resources

**Certifications:**
- GIAC Certified Incident Handler (GCIH)
- Certified Information Systems Security Professional (CISSP)
- EC-Council Certified Incident Handler (ECIH)

**Free Training:**
- CISA Ransomware Guide: https://www.cisa.gov/ransomware
- NIST Ransomware Resources: https://www.nist.gov/ransomware
- SANS Ransomware Summit (annual)

---

### F.3 Incident Response Contacts

**Government Resources:**
- FBI Internet Crime Complaint Center (IC3): https://www.ic3.gov
- CISA Cybersecurity Hotline: 1-888-282-0870
- No More Ransom Project: https://www.nomoreransom.org

**Decryption Tools:**
- Emsisoft Decryptor Tools: https://www.emsisoft.com/ransomware-decryption-tools/
- Kaspersky Free Decryptors: https://www.kaspersky.com/downloads/thank-you/free-ransomware-decryptors
- Avast Free Ransomware Decryption Tools

---

*© 2025 WIA - World Certification Industry Association*
*弘益人間 · Benefit All Humanity*
