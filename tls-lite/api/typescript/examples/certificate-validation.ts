/**
 * Certificate Validation Example
 * Demonstrates various certificate validation scenarios
 */

import {
  createTLSLite,
  Certificate,
  CertificateType,
  CertificateValidationOptions
} from '@wia/tls-lite';

async function main() {
  console.log('📜 WIA-TLS-LITE Certificate Validation Example\n');

  const tls = createTLSLite();

  // === Example 1: Valid Certificate ===
  console.log('═══ Test 1: Valid Certificate ═══');
  const validCert: Certificate = {
    type: CertificateType.X509,
    data: Buffer.from('mock-cert-data'),
    subject: 'CN=iot.example.com,O=Example IoT,C=US',
    issuer: 'CN=Example CA,O=Example Inc,C=US',
    notBefore: new Date('2025-01-01'),
    notAfter: new Date('2026-01-01'),
    publicKey: Buffer.from('mock-public-key'),
    fingerprint: 'SHA256:abc123...'
  };

  const isValid1 = await tls.validateCertificate(validCert);
  console.log('✓ Standard validation:', isValid1 ? 'PASS' : 'FAIL');

  // === Example 2: Self-Signed Certificate ===
  console.log('\n═══ Test 2: Self-Signed Certificate ═══');
  const selfSignedCert: Certificate = {
    type: CertificateType.X509,
    data: Buffer.from('mock-cert-data'),
    subject: 'CN=device-001,O=IoT Device',
    issuer: 'CN=device-001,O=IoT Device', // Same as subject
    notBefore: new Date('2025-01-01'),
    notAfter: new Date('2026-01-01'),
    publicKey: Buffer.from('mock-public-key')
  };

  // Reject self-signed by default
  const isValid2a = await tls.validateCertificate(selfSignedCert);
  console.log('✗ Without allowSelfSigned:', isValid2a ? 'PASS' : 'FAIL');

  // Allow self-signed explicitly
  const isValid2b = await tls.validateCertificate(selfSignedCert, {
    allowSelfSigned: true
  });
  console.log('✓ With allowSelfSigned:', isValid2b ? 'PASS' : 'FAIL');

  // === Example 3: Expired Certificate ===
  console.log('\n═══ Test 3: Expired Certificate ═══');
  const expiredCert: Certificate = {
    type: CertificateType.X509,
    data: Buffer.from('mock-cert-data'),
    subject: 'CN=old-device.example.com',
    issuer: 'CN=Example CA',
    notBefore: new Date('2020-01-01'),
    notAfter: new Date('2021-01-01'), // Expired
    publicKey: Buffer.from('mock-public-key')
  };

  const isValid3 = await tls.validateCertificate(expiredCert);
  console.log('✗ Expired certificate:', isValid3 ? 'PASS' : 'FAIL');

  // === Example 4: Custom Validator ===
  console.log('\n═══ Test 4: Custom Validator ═══');

  const customOptions: CertificateValidationOptions = {
    customValidator: async (cert) => {
      console.log('   Running custom validation...');

      // Example: Only allow certificates from specific organization
      const allowedOrg = 'Example IoT';
      const hasOrg = cert.subject?.includes(`O=${allowedOrg}`);

      console.log('   Subject:', cert.subject);
      console.log('   Required org:', allowedOrg);
      console.log('   Has org:', hasOrg);

      return hasOrg ?? false;
    }
  };

  const isValid4a = await tls.validateCertificate(validCert, customOptions);
  console.log('✓ Valid org:', isValid4a ? 'PASS' : 'FAIL');

  const wrongOrgCert: Certificate = {
    ...validCert,
    subject: 'CN=device.example.com,O=Wrong Org,C=US'
  };

  const isValid4b = await tls.validateCertificate(wrongOrgCert, customOptions);
  console.log('✗ Wrong org:', isValid4b ? 'PASS' : 'FAIL');

  // === Example 5: Raw Public Key (No Certificate) ===
  console.log('\n═══ Test 5: Raw Public Key ═══');
  const rawPublicKey: Certificate = {
    type: CertificateType.RAW_PUBLIC_KEY,
    data: Buffer.from('raw-public-key-bytes'),
    publicKey: Buffer.from('raw-public-key-bytes')
  };

  console.log('Type:', rawPublicKey.type);
  console.log('Size:', rawPublicKey.data.length, 'bytes');
  console.log('✓ No certificate overhead - ideal for IoT!');

  // === Example 6: Trust Anchors ===
  console.log('\n═══ Test 6: Trust Anchors ═══');

  const caCert: Certificate = {
    type: CertificateType.X509,
    data: Buffer.from('ca-cert-data'),
    subject: 'CN=Example CA,O=Example Inc',
    issuer: 'CN=Example CA,O=Example Inc', // Self-signed CA
    notBefore: new Date('2020-01-01'),
    notAfter: new Date('2030-01-01'),
    publicKey: Buffer.from('ca-public-key')
  };

  const trustOptions: CertificateValidationOptions = {
    trustAnchors: [caCert],
    checkRevocation: true
  };

  console.log('Trust anchor:', caCert.subject);
  console.log('Certificate issuer:', validCert.issuer);
  console.log('Check revocation:', trustOptions.checkRevocation);

  const isValid6 = await tls.validateCertificate(validCert, trustOptions);
  console.log('✓ Validation result:', isValid6 ? 'PASS' : 'FAIL');

  // === Example 7: Compact X.509 for IoT ===
  console.log('\n═══ Test 7: Compact X.509 (IoT-Optimized) ═══');
  const compactCert: Certificate = {
    type: CertificateType.COMPACT_X509,
    data: Buffer.from('compact-cert-data'),
    subject: 'CN=sensor-node-42',
    publicKey: Buffer.from('compact-public-key')
  };

  console.log('Type:', compactCert.type);
  console.log('Subject:', compactCert.subject);
  console.log('Size:', compactCert.data.length, 'bytes');
  console.log('✓ Reduced size for constrained devices');

  // === Summary ===
  console.log('\n═══ Certificate Types for IoT ═══');
  console.log('\n1. X.509 (Standard)');
  console.log('   ✓ Widely supported');
  console.log('   ✗ Larger size');
  console.log('   Use: General IoT devices');

  console.log('\n2. Raw Public Key');
  console.log('   ✓ Minimal overhead');
  console.log('   ✓ Fast validation');
  console.log('   Use: Constrained devices');

  console.log('\n3. PSK (Pre-Shared Key)');
  console.log('   ✓ No certificates needed');
  console.log('   ✓ Ultra-lightweight');
  console.log('   Use: Memory-limited devices');

  console.log('\n4. Compact X.509');
  console.log('   ✓ Reduced size');
  console.log('   ✓ PKI compatible');
  console.log('   Use: IoT with size constraints');

  console.log('\n✅ Certificate validation examples completed\n');
}

main().catch(console.error);
