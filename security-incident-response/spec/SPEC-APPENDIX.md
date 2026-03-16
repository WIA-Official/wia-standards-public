# WIA-SEC-020: Security Incident Response - Appendix

**Standard ID:** WIA-SEC-020
**Version:** 1.0
**Last Updated:** 2025-12-25

---

## Appendix A: Incident Response Playbooks

### A.1 Ransomware Incident Response

**Immediate Actions (0-15 minutes):**

1. **Isolate Infected Systems**
   ```bash
   # Disable network adapters
   ifconfig eth0 down  # Linux
   Disable-NetAdapter -Name "Ethernet" -Confirm:$false  # Windows PowerShell

   # Block at firewall
   iptables -A INPUT -s <infected_ip> -j DROP
   iptables -A OUTPUT -d <infected_ip> -j DROP
   ```

2. **Preserve Evidence**
   ```bash
   # Capture memory dump (if encryption not yet started)
   sudo ./avml /mnt/evidence/memory_$(hostname)_$(date +%Y%m%d_%H%M%S).lime

   # Document visible ransom note
   screenshot /mnt/evidence/ransom_note.png

   # List encrypted files
   find / -name "*.encrypted" -o -name "*.locked" > /tmp/encrypted_files.txt 2>/dev/null
   ```

3. **Identify Ransomware Variant**
   - Check ransom note for identifying information
   - Use ID Ransomware (id-ransomware.malwarehunterteam.com)
   - Check file extensions
   - Analyze ransom note content

**Investigation Actions (15-60 minutes):**

4. **Determine Patient Zero**
   ```bash
   # Find earliest encrypted file
   find / -name "*.encrypted" -printf '%T+ %p\n' | sort | head -1

   # Check email logs for phishing
   grep -i "attachment" /var/log/mail.log | grep -E "$(date +%Y-%m-%d)"

   # Review authentication logs
   grep "session opened" /var/log/auth.log | tail -100
   ```

5. **Assess Scope**
   - Number of affected systems
   - Types of data encrypted
   - Backup status
   - Network shares affected

**Containment Actions (1-4 hours):**

6. **Network Segmentation**
   - Isolate affected VLANs
   - Block C2 communication
   - Disable remote access temporarily

7. **Account Security**
   ```powershell
   # Disable potentially compromised accounts
   Disable-ADAccount -Identity <username>

   # Force password reset for all users
   Get-ADUser -Filter * | Set-ADUser -ChangePasswordAtLogon $true
   ```

**Recovery Actions (4-24 hours):**

8. **Restore from Backup**
   ```bash
   # Verify backup integrity
   sha256sum backup.tar.gz
   # Compare with known good hash

   # Scan backup for malware
   clamscan -r /mnt/backup/

   # Restore data
   tar -xzvf backup.tar.gz -C /restore/
   ```

9. **Rebuild Compromised Systems**
   - Wipe and reinstall OS
   - Apply all security patches
   - Restore data from clean backup
   - Implement enhanced monitoring

**Post-Incident (1-2 weeks):**

10. **Lessons Learned**
    - How did ransomware enter?
    - Why wasn't it detected earlier?
    - What can prevent recurrence?
    - Update response procedures

**Decision Tree: To Pay or Not to Pay?**

```
Do NOT pay ransom if:
✓ You have clean, recent backups
✓ Decryption tool available (No More Ransom Project)
✓ Data not critically time-sensitive
✓ Prohibited by policy or law

Consider paying ONLY if:
⚠ No backups available
⚠ Mission-critical data
⚠ Lives at risk (hospital)
⚠ After consulting legal/insurance
⚠ Law enforcement notified

Note: Paying does not guarantee:
- You will receive decryption key
- Key will work properly
- Data won't be leaked anyway
- You won't be targeted again
```

---

### A.2 Data Breach Response

**Detection Indicators:**
- Unusual database queries
- Large data exports
- Access from unusual locations
- Privilege escalation attempts
- Failed authorization attempts

**Immediate Response:**

1. **Confirm Breach**
   ```sql
   -- Check for unusual queries (MySQL example)
   SELECT * FROM mysql.general_log
   WHERE command_type = 'Query'
   AND argument LIKE '%SELECT%'
   ORDER BY event_time DESC
   LIMIT 100;

   -- Check for data exports
   SELECT user, query_time, rows_examined, rows_sent
   FROM mysql.slow_log
   WHERE rows_sent > 10000
   ORDER BY query_time DESC;
   ```

2. **Isolate Affected Systems**
   - Restrict database access
   - Block suspicious IP addresses
   - Revoke compromised credentials

3. **Preserve Evidence**
   - Database logs
   - Application logs
   - Network traffic captures
   - Authentication logs

**Investigation:**

4. **Determine Scope**
   - What data was accessed?
   - When did access occur?
   - How much data was exfiltrated?
   - Who was affected?

5. **Root Cause Analysis**
   ```python
   # Analyze access patterns
   import pandas as pd

   # Load database access logs
   logs = pd.read_csv('db_access.log')

   # Identify anomalies
   unusual_queries = logs[
       (logs['rows_returned'] > logs['rows_returned'].quantile(0.95)) |
       (logs['query_time'] > logs['query_time'].quantile(0.95))
   ]

   # Find access from unusual IPs
   legitimate_ips = ['10.0.1.0/24', '10.0.2.0/24']
   suspicious_access = logs[~logs['source_ip'].isin(legitimate_ips)]
   ```

**Notification Requirements:**

| Regulation | Timeframe | Who to Notify |
|------------|-----------|---------------|
| GDPR | 72 hours | Supervisory authority, affected individuals |
| HIPAA | 60 days | HHS, affected individuals, media (if >500) |
| PCI-DSS | Immediately | Card brands, acquiring bank |
| CCPA | Without unreasonable delay | California AG, affected residents |

**Notification Template:**
```
Subject: Important Security Notice - Data Breach Notification

Dear [Name],

We are writing to inform you of a data security incident that may have
affected your personal information.

WHAT HAPPENED:
On [DATE], we discovered that an unauthorized party gained access to
[SYSTEM/DATABASE]. We immediately launched an investigation and took
steps to secure our systems.

WHAT INFORMATION WAS INVOLVED:
The information that may have been accessed includes:
- [List specific data types: names, addresses, SSN, etc.]

WHAT WE ARE DOING:
- We have secured the affected systems
- We engaged cybersecurity experts to investigate
- We notified law enforcement
- We are implementing additional security measures

WHAT YOU CAN DO:
- Monitor your accounts for suspicious activity
- Consider placing a fraud alert or credit freeze
- We are offering [12/24] months of free credit monitoring

FOR MORE INFORMATION:
Contact our dedicated assistance line at [PHONE]
or visit [WEBSITE] for FAQs and resources.

We sincerely apologize for this incident and any inconvenience.

Sincerely,
[Name, Title]
```

---

### A.3 DDoS Attack Response

**Attack Types:**
```
Volume-Based:
- UDP floods
- ICMP floods
- DNS amplification
- NTP amplification

Protocol-Based:
- SYN floods
- Fragmented packet attacks
- Ping of Death
- Smurf attacks

Application-Layer:
- HTTP floods
- Slowloris
- RUDY (Slow POST)
- DNS query floods
```

**Mitigation Steps:**

1. **Immediate Response**
   ```bash
   # Enable SYN cookies (Linux)
   sysctl -w net.ipv4.tcp_syncookies=1

   # Rate limit connections
   iptables -A INPUT -p tcp --dport 80 -m limit --limit 25/minute --limit-burst 100 -j ACCEPT

   # Block specific source
   iptables -A INPUT -s <attacker_ip> -j DROP
   ```

2. **Activate DDoS Mitigation Service**
   - Cloudflare
   - AWS Shield
   - Akamai Prolexic
   - Arbor Networks

3. **Contact ISP**
   - Request upstream filtering
   - Blackhole routing if necessary
   - Coordinate traffic analysis

4. **Scale Infrastructure**
   ```bash
   # Auto-scaling group (AWS example)
   aws autoscaling set-desired-capacity \
     --auto-scaling-group-name web-asg \
     --desired-capacity 20

   # Enable CloudFront CDN
   aws cloudfront create-distribution \
     --origin-domain-name example.com
   ```

**Analysis:**
```python
# Analyze DDoS traffic
from scapy.all import *

def analyze_ddos_pcap(pcap_file):
    packets = rdpcap(pcap_file)

    # Count packets by source IP
    src_ips = {}
    for packet in packets:
        if IP in packet:
            src = packet[IP].src
            src_ips[src] = src_ips.get(src, 0) + 1

    # Identify top attackers
    top_attackers = sorted(src_ips.items(), key=lambda x: x[1], reverse=True)[:20]

    # Identify attack type
    protocols = {}
    for packet in packets:
        if packet.haslayer(TCP):
            protocols['TCP'] = protocols.get('TCP', 0) + 1
        elif packet.haslayer(UDP):
            protocols['UDP'] = protocols.get('UDP', 0) + 1
        elif packet.haslayer(ICMP):
            protocols['ICMP'] = protocols.get('ICMP', 0) + 1

    return {
        'top_attackers': top_attackers,
        'protocols': protocols,
        'total_packets': len(packets)
    }
```

---

## Appendix B: Forensic Procedures

### B.1 Digital Evidence Collection

**Order of Volatility (collect in this order):**

1. **CPU Registers and Cache** (nanoseconds)
   - Usually only via hardware debugger
   - Lost when system powered off

2. **Memory (RAM)** (minutes)
   ```bash
   # Linux Memory Acquisition
   sudo insmod lime.ko "path=/mnt/usb/memory.lime format=lime"
   sha256sum /mnt/usb/memory.lime > /mnt/usb/memory.lime.sha256

   # Windows Memory Acquisition
   FTK Imager: File > Capture Memory
   # or
   winpmem.exe memory.raw
   ```

3. **Network State** (seconds to hours)
   ```bash
   # Capture current connections
   netstat -anp > network_connections.txt  # Linux
   Get-NetTCPConnection | Export-Csv connections.csv  # Windows

   # Active sessions
   w > active_sessions.txt  # Linux
   qwinsta > active_sessions.txt  # Windows

   # ARP cache
   arp -a > arp_cache.txt
   ```

4. **Running Processes** (seconds to hours)
   ```bash
   # Process list with full details
   ps auxww > processes.txt  # Linux
   Get-Process | Export-Csv processes.csv  # Windows

   # Process tree
   pstree -p > process_tree.txt  # Linux

   # Open files by process
   lsof > open_files.txt  # Linux
   ```

5. **Disk/Filesystem** (months to years)
   ```bash
   # Create forensic image
   dd if=/dev/sda of=/mnt/evidence/disk.img bs=64K conv=noerror,sync status=progress

   # Using dc3dd (with hashing)
   dc3dd if=/dev/sda of=/mnt/evidence/disk.img hash=sha256 log=/mnt/evidence/acquisition.log

   # Using FTK Imager (GUI)
   # File > Create Disk Image > Physical Drive
   ```

6. **Logs and Backups** (months to years)
   ```bash
   # Collect system logs
   tar -czf logs_$(hostname)_$(date +%Y%m%d).tar.gz \
     /var/log \
     /var/log/auth.log* \
     /var/log/syslog* \
     /var/log/apache2/* \
     /var/log/nginx/*

   # Windows Event Logs
   wevtutil epl System system.evtx
   wevtutil epl Security security.evtx
   wevtutil epl Application application.evtx
   ```

### B.2 Chain of Custody Form

```
EVIDENCE CUSTODY RECORD

Case Number: ___________________
Incident ID: ___________________
Evidence ID: ___________________

EVIDENCE DESCRIPTION:
Type: [ ] Hard Drive [ ] Memory Dump [ ] Log Files [ ] Network Capture [ ] Other: _____
Description: _________________________________________________________________
Serial Number: _______________________________________________________________
Storage Media: _______________________________________________________________

HASH VALUES:
MD5:    _____________________________________________________________________
SHA256: _____________________________________________________________________

COLLECTION INFORMATION:
Collected By: _________________________ Badge/ID: _________________________
Date/Time: ____________________________ Location: _________________________
Collection Method: ___________________________________________________________
Photographs Taken: [ ] Yes [ ] No   Photo IDs: ____________________________

CHAIN OF CUSTODY:
┌──────────────┬─────────────┬──────────────┬──────────┬──────────────┐
│ Date/Time    │ Released By │ Received By  │ Purpose  │ Location     │
├──────────────┼─────────────┼──────────────┼──────────┼──────────────┤
│              │             │              │          │              │
│              │             │              │          │              │
│              │             │              │          │              │
│              │             │              │          │              │
└──────────────┴─────────────┴──────────────┴──────────┴──────────────┘

EVIDENCE DISPOSITION:
[ ] Returned to Owner  [ ] Destroyed  [ ] Retained
Disposition Date: ___________________
Authorized By: _____________________  Signature: __________________________
```

### B.3 Timeline Analysis

**Super Timeline Creation:**
```bash
# Using log2timeline/plaso
log2timeline.py --storage-file timeline.plaso /mnt/evidence/

# Convert to readable format
psort.py -o l2tcsv -w timeline.csv timeline.plaso

# Create focused timeline
psort.py -o l2tcsv -w focused_timeline.csv timeline.plaso \
  "date > '2025-12-24 00:00:00' AND date < '2025-12-26 00:00:00'"
```

**Timeline Visualization:**
```python
import pandas as pd
import matplotlib.pyplot as plt

# Load timeline
df = pd.read_csv('timeline.csv')
df['datetime'] = pd.to_datetime(df['date'])

# Group by hour
hourly_events = df.groupby(df['datetime'].dt.floor('H')).size()

# Plot
plt.figure(figsize=(15, 5))
hourly_events.plot(kind='bar')
plt.title('Event Timeline')
plt.xlabel('Time')
plt.ylabel('Number of Events')
plt.xticks(rotation=45)
plt.tight_layout()
plt.savefig('timeline_visualization.png')
```

---

## Appendix C: Communication Templates

### C.1 Initial Incident Notification

**Subject:** SECURITY INCIDENT - [SEVERITY] - [INCIDENT ID]

```
INCIDENT ALERT

Incident ID: INC-2025-001234
Severity: CRITICAL
Detected: 2025-12-25 14:30 UTC
Status: INVESTIGATING

SUMMARY:
[Brief description of incident]

AFFECTED SYSTEMS:
- System 1
- System 2

IMPACT:
[Description of business impact]

CURRENT ACTIONS:
- Action 1
- Action 2

INCIDENT RESPONSE TEAM:
Lead: John Smith (john@company.com, +1-555-0100)
Analyst: Jane Doe (jane@company.com, +1-555-0101)

NEXT UPDATE: In 2 hours or when status changes

For questions, contact CSIRT at security@company.com
```

### C.2 Executive Summary

**Subject:** Executive Summary - Security Incident [INCIDENT ID]

```
EXECUTIVE SUMMARY - SECURITY INCIDENT

Incident ID: INC-2025-001234
Date: December 25, 2025
Classification: CONFIDENTIAL

WHAT HAPPENED:
[Non-technical summary of the incident]

BUSINESS IMPACT:
- Systems Affected: [Number/Description]
- Duration of Outage: [Hours/Days]
- Data at Risk: [Yes/No and what type]
- Financial Impact: [Estimated $ or TBD]
- Reputation Impact: [Assessment]

CURRENT STATUS:
[ ] DETECTED  [ ] CONTAINED  [X] UNDER INVESTIGATION  [ ] RESOLVED

ACTIONS TAKEN:
1. [Action]
2. [Action]
3. [Action]

NEXT STEPS:
1. [Step]
2. [Step]

ESTIMATED RESOLUTION: [Date/Time]

LEGAL/REGULATORY:
[Any notification requirements, regulatory considerations]

For more information, contact:
[CISO Name], Chief Information Security Officer
[Email], [Phone]
```

---

## Appendix D: Tools and Resources

### D.1 Essential Tools

**Free/Open Source:**
- **Volatility** - Memory forensics framework
- **Autopsy** - Digital forensics platform
- **Wireshark** - Network protocol analyzer
- **Zeek** - Network security monitor
- **YARA** - Malware identification and classification
- **Ghidra** - Reverse engineering tool
- **TheHive** - Incident response platform
- **MISP** - Threat intelligence platform

**Commercial:**
- **Splunk Enterprise Security** - SIEM
- **CrowdStrike Falcon** - EDR
- **Palo Alto Cortex XSOAR** - SOAR
- **EnCase** - Digital forensics
- **FTK** - Forensic toolkit
- **IDA Pro** - Disassembler

### D.2 Training Resources

**Certifications:**
- GIAC Certified Incident Handler (GCIH)
- GIAC Certified Forensic Analyst (GCFA)
- Certified Computer Security Incident Handler (CSIH)
- Certified Incident Handler (ECIH)

**Training Platforms:**
- SANS Cyber Range
- RangeForce
- Cyber Defenders
- TryHackMe
- HackTheBox

---

## Appendix E: Legal and Regulatory References

### E.1 Breach Notification Laws (US)

**Federal:**
- HIPAA Breach Notification Rule
- GLBA Safeguards Rule
- FISMA Incident Reporting

**State Laws:**
- California: Cal. Civ. Code § 1798.82
- New York: N.Y. Gen. Bus. Law § 899-aa
- All 50 states have breach notification laws

### E.2 International Regulations

**GDPR (EU):**
- Article 33: Notification of breach to supervisory authority (72 hours)
- Article 34: Communication of breach to data subject

**Other Jurisdictions:**
- PIPEDA (Canada)
- Privacy Act 1988 (Australia)
- PDPA (Singapore)
- LGPD (Brazil)

---

**Document Control:**
- Version: 1.0
- Status: ACTIVE
- Last Review: 2025-12-25
- Next Review: 2026-06-25
- Owner: WIA Security Standards Committee

**弘益人間 (홍익인간) - Benefit All Humanity**

© 2025 SmileStory Inc. / WIA
