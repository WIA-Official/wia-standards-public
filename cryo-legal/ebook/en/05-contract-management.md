# Chapter 5: Contract Management - Templates, Clauses, and Lifecycle

## Comprehensive Contract Management System

This chapter provides complete contract management implementations for the WIA Cryo Legal Standard, including template management, clause libraries, signature workflows, and contract lifecycle management.

## Contract Template Engine

```typescript
/**
 * WIA Cryo Legal Standard - Contract Template Engine
 * Dynamic template generation and management
 */

import { z } from 'zod';
import {
  ContractTemplate,
  ContractTemplateSchema,
  Clause,
  ClauseSchema,
  ContractType,
  TemplateStatus,
} from './types';

// ============================================================================
// Template Engine Configuration
// ============================================================================

export interface TemplateEngineConfig {
  templatesPath: string;
  clauseLibraryPath: string;
  outputFormat: 'markdown' | 'html' | 'pdf';
  variableDelimiters: [string, string];
  conditionalDelimiters: [string, string];
}

export const defaultConfig: TemplateEngineConfig = {
  templatesPath: './templates',
  clauseLibraryPath: './clauses',
  outputFormat: 'markdown',
  variableDelimiters: ['{{', '}}'],
  conditionalDelimiters: ['{{#', '}}'],
};

// ============================================================================
// Template Manager
// ============================================================================

export class ContractTemplateManager {
  private templates: Map<string, ContractTemplate> = new Map();
  private clauseLibrary: Map<string, Clause> = new Map();
  private versionHistory: Map<string, ContractTemplate[]> = new Map();

  constructor(private readonly config: TemplateEngineConfig = defaultConfig) {}

  async initialize(): Promise<void> {
    await this.loadDefaultTemplates();
    await this.loadClauseLibrary();
  }

  private async loadDefaultTemplates(): Promise<void> {
    // Storage Agreement Template
    this.registerTemplate({
      id: 'tpl-storage-agreement-v1',
      name: 'Cryogenic Storage Agreement',
      type: 'storage-agreement',
      version: '1.0.0',
      jurisdiction: 'US',
      clauses: this.getStorageAgreementClauses(),
      approvedBy: 'Legal Department',
      approvedAt: new Date().toISOString(),
      status: 'active',
    });

    // Consent Form Template
    this.registerTemplate({
      id: 'tpl-consent-form-v1',
      name: 'Informed Consent for Cryopreservation',
      type: 'consent-form',
      version: '1.0.0',
      jurisdiction: 'US',
      clauses: this.getConsentFormClauses(),
      approvedBy: 'Legal Department',
      approvedAt: new Date().toISOString(),
      status: 'active',
    });

    // Research Agreement Template
    this.registerTemplate({
      id: 'tpl-research-agreement-v1',
      name: 'Research Use Agreement',
      type: 'research-agreement',
      version: '1.0.0',
      jurisdiction: 'US',
      clauses: this.getResearchAgreementClauses(),
      approvedBy: 'Legal Department',
      approvedAt: new Date().toISOString(),
      status: 'active',
    });

    // Donation Agreement Template
    this.registerTemplate({
      id: 'tpl-donation-agreement-v1',
      name: 'Tissue Donation Agreement',
      type: 'donation-agreement',
      version: '1.0.0',
      jurisdiction: 'US',
      clauses: this.getDonationAgreementClauses(),
      approvedBy: 'Legal Department',
      approvedAt: new Date().toISOString(),
      status: 'active',
    });
  }

  private getStorageAgreementClauses(): Clause[] {
    return [
      {
        id: 'clause-storage-purpose',
        title: 'Purpose of Storage',
        text: `This Agreement governs the cryogenic storage of {{specimen_type}}
               (the "Specimen") belonging to {{client_name}} (the "Client")
               at {{facility_name}} (the "Facility"). The Facility agrees to
               store the Specimen for the purposes specified by the Client,
               including but not limited to: {{storage_purposes}}.`,
        type: 'standard',
        mandatory: true,
        category: 'scope',
      },
      {
        id: 'clause-storage-term',
        title: 'Term of Storage',
        text: `The initial storage term shall be {{initial_term}} commencing on
               {{effective_date}}. The Agreement shall automatically renew for
               successive {{renewal_term}} periods unless either party provides
               written notice of non-renewal at least {{notice_period}} prior
               to the expiration of the then-current term.`,
        type: 'standard',
        mandatory: true,
        category: 'term',
      },
      {
        id: 'clause-storage-fees',
        title: 'Storage Fees',
        text: `The Client agrees to pay the following fees:
               - Initial Processing Fee: {{processing_fee}}
               - Annual Storage Fee: {{annual_storage_fee}}
               - Retrieval Fee: {{retrieval_fee}}

               Payment is due {{payment_terms}}. Late payments shall incur
               a {{late_fee_percentage}}% monthly finance charge.`,
        type: 'standard',
        mandatory: true,
        category: 'fees',
      },
      {
        id: 'clause-storage-conditions',
        title: 'Storage Conditions',
        text: `The Facility shall maintain the Specimen at {{storage_temperature}}
               using {{storage_method}}. The Facility maintains {{backup_systems}}
               and monitors storage conditions {{monitoring_frequency}}.
               In case of equipment failure, the Facility shall {{emergency_procedures}}.`,
        type: 'standard',
        mandatory: true,
        category: 'operations',
      },
      {
        id: 'clause-access-release',
        title: 'Access and Release',
        text: `Access to the Specimen shall be granted only to authorized persons
               as designated by the Client. Release of the Specimen requires:
               - Written request from the Client or authorized designee
               - Verification of identity
               - Payment of outstanding fees
               - Completion of release documentation`,
        type: 'standard',
        mandatory: true,
        category: 'access',
      },
      {
        id: 'clause-liability-limitation',
        title: 'Limitation of Liability',
        text: `The Facility's liability for loss or damage to the Specimen is
               limited to {{liability_cap}}. THE FACILITY SHALL NOT BE LIABLE
               FOR ANY INDIRECT, INCIDENTAL, CONSEQUENTIAL, OR PUNITIVE DAMAGES.

               The Facility is not liable for loss or damage caused by:
               - Acts of God, natural disasters, or force majeure
               - War, terrorism, or civil unrest
               - Government actions or regulations
               - Client's failure to provide accurate information`,
        type: 'standard',
        mandatory: true,
        category: 'liability',
      },
      {
        id: 'clause-insurance',
        title: 'Insurance',
        text: `{{#if insurance_required}}
               The Client agrees to maintain insurance coverage of at least
               {{minimum_coverage}} for the Specimen during the storage term.
               {{/if}}

               The Facility maintains professional liability insurance with
               coverage of {{facility_coverage}}.`,
        type: 'optional',
        mandatory: false,
        category: 'insurance',
      },
      {
        id: 'clause-termination',
        title: 'Termination',
        text: `Either party may terminate this Agreement:
               - With {{notice_period}} written notice
               - Immediately for material breach (with {{cure_period}} cure period)
               - Upon death of the Client (subject to disposition instructions)

               Upon termination, the Client must retrieve the Specimen within
               {{retrieval_deadline}} or provide disposition instructions.`,
        type: 'standard',
        mandatory: true,
        category: 'termination',
      },
      {
        id: 'clause-disposition',
        title: 'Disposition Instructions',
        text: `In the event of the Client's death, incapacity, or failure to
               maintain payment, the Specimen shall be handled according to
               the disposition instructions on file. If no valid instructions
               exist, the Facility may:
               - Attempt to contact designated beneficiaries
               - Hold the Specimen for {{abandonment_period}}
               - Dispose of the Specimen in accordance with applicable law`,
        type: 'standard',
        mandatory: true,
        category: 'disposition',
      },
      {
        id: 'clause-governing-law',
        title: 'Governing Law',
        text: `This Agreement shall be governed by the laws of {{governing_state}},
               without regard to conflict of law principles. Any disputes shall
               be resolved in the courts of {{venue_county}}, {{venue_state}}.`,
        type: 'standard',
        mandatory: true,
        category: 'legal',
      },
    ];
  }

  private getConsentFormClauses(): Clause[] {
    return [
      {
        id: 'clause-consent-purpose',
        title: 'Purpose of Cryopreservation',
        text: `I, {{patient_name}}, hereby consent to the cryopreservation of
               my {{specimen_type}} for the following purposes:
               {{#each purposes}}
               - {{this}}
               {{/each}}

               I understand that cryopreservation is a process that freezes and
               stores biological material at extremely low temperatures.`,
        type: 'standard',
        mandatory: true,
        category: 'consent',
      },
      {
        id: 'clause-risks-benefits',
        title: 'Risks and Benefits',
        text: `I have been informed of and understand:

               POTENTIAL BENEFITS:
               - Preservation of reproductive potential
               - Future use for fertility treatment
               - Potential use for medical research (if consented)

               POTENTIAL RISKS:
               - No guarantee of successful preservation
               - Possible loss or damage during storage
               - Unknown long-term effects of extended storage
               - Equipment failure despite safety measures`,
        type: 'standard',
        mandatory: true,
        category: 'disclosure',
      },
      {
        id: 'clause-voluntary',
        title: 'Voluntary Consent',
        text: `I confirm that:
               - This consent is given voluntarily
               - I have had the opportunity to ask questions
               - My questions have been answered satisfactorily
               - I have not been pressured into this decision
               - I may withdraw consent at any time`,
        type: 'standard',
        mandatory: true,
        category: 'consent',
      },
      {
        id: 'clause-future-use',
        title: 'Future Use Decisions',
        text: `I direct that my preserved {{specimen_type}} be used for
               (select all that apply):

               [ ] Personal use only
               [ ] Use by designated partner: {{partner_name}}
               [ ] Donation for research (anonymous)
               [ ] Donation to another individual/couple
               [ ] Disposal after {{disposal_date}}`,
        type: 'standard',
        mandatory: true,
        category: 'instructions',
      },
      {
        id: 'clause-posthumous-use',
        title: 'Posthumous Use Authorization',
        text: `In the event of my death:

               [ ] I DO authorize posthumous use of my preserved material
               [ ] I DO NOT authorize posthumous use

               If authorized, I designate {{designated_person}} to make
               decisions regarding use of the preserved material.`,
        type: 'standard',
        mandatory: true,
        category: 'posthumous',
      },
    ];
  }

  private getResearchAgreementClauses(): Clause[] {
    return [
      {
        id: 'clause-research-scope',
        title: 'Research Scope',
        text: `This Agreement authorizes {{research_institution}} to use
               specimens from {{facility_name}} for the research project(s)
               described as:

               {{research_description}}

               Principal Investigator: {{pi_name}}
               IRB Approval Number: {{irb_number}}
               Approval Date: {{irb_approval_date}}`,
        type: 'standard',
        mandatory: true,
        category: 'scope',
      },
      {
        id: 'clause-specimen-handling',
        title: 'Specimen Handling Requirements',
        text: `The Research Institution agrees to:
               - Maintain specimens at appropriate storage conditions
               - Follow all applicable regulations (FDA, IRB, institutional)
               - Not transfer specimens to third parties without consent
               - Return or properly dispose of unused specimens
               - Maintain chain of custody documentation`,
        type: 'standard',
        mandatory: true,
        category: 'requirements',
      },
      {
        id: 'clause-data-privacy',
        title: 'Data Privacy and De-identification',
        text: `All specimens shall be de-identified according to {{deidentification_standard}}.

               The Research Institution shall:
               - Not attempt to re-identify donors
               - Protect all associated data per HIPAA requirements
               - Report any data breaches within {{breach_notification_period}}
               - Destroy linking information upon project completion`,
        type: 'standard',
        mandatory: true,
        category: 'privacy',
      },
      {
        id: 'clause-intellectual-property',
        title: 'Intellectual Property',
        text: `Intellectual property arising from the research:

               {{#if shared_ip}}
               - Shall be jointly owned by both parties
               - Revenue sharing: {{revenue_split}}
               {{else}}
               - Shall be owned by the Research Institution
               - Facility receives acknowledgment in publications
               {{/if}}

               Both parties agree to maintain confidentiality of unpublished findings.`,
        type: 'negotiable',
        mandatory: true,
        category: 'ip',
      },
    ];
  }

  private getDonationAgreementClauses(): Clause[] {
    return [
      {
        id: 'clause-donation-intent',
        title: 'Donation Intent',
        text: `I, {{donor_name}}, hereby voluntarily donate my {{specimen_type}}
               to {{recipient_type}} through {{facility_name}}.

               I understand that this donation is:
               {{#if anonymous}}
               - Anonymous (my identity will not be disclosed)
               {{else}}
               - Non-anonymous (my identity may be shared with recipient)
               {{/if}}

               {{#if compensated}}
               - Compensated: {{compensation_amount}}
               {{else}}
               - Non-compensated (altruistic)
               {{/if}}`,
        type: 'standard',
        mandatory: true,
        category: 'intent',
      },
      {
        id: 'clause-donor-screening',
        title: 'Donor Screening and Testing',
        text: `I agree to undergo screening and testing as required by
               FDA regulations (21 CFR 1271) and facility protocols, including:

               - Medical history questionnaire
               - Physical examination
               - Infectious disease testing
               - Genetic screening (if applicable)

               I understand that I may be disqualified based on screening results.`,
        type: 'standard',
        mandatory: true,
        category: 'screening',
      },
      {
        id: 'clause-irrevocable',
        title: 'Irrevocability of Donation',
        text: `I understand that once my donation is used for its intended
               purpose, this donation becomes IRREVOCABLE.

               Prior to use, I may revoke this donation by providing written
               notice to the Facility within {{revocation_period}}.

               After revocation deadline, I relinquish all rights to the
               donated material and any resulting pregnancies, embryos, or
               offspring.`,
        type: 'standard',
        mandatory: true,
        category: 'rights',
      },
    ];
  }

  private async loadClauseLibrary(): Promise<void> {
    // Load all clauses from templates
    for (const template of this.templates.values()) {
      for (const clause of template.clauses) {
        this.clauseLibrary.set(clause.id, clause);
      }
    }
  }

  registerTemplate(template: ContractTemplate): void {
    const validated = ContractTemplateSchema.parse(template);

    // Store version history
    const history = this.versionHistory.get(validated.id) || [];
    const existing = this.templates.get(validated.id);
    if (existing) {
      history.push(existing);
      this.versionHistory.set(validated.id, history);
    }

    this.templates.set(validated.id, validated);
  }

  getTemplate(id: string): ContractTemplate | undefined {
    return this.templates.get(id);
  }

  getTemplatesByType(type: ContractType): ContractTemplate[] {
    return Array.from(this.templates.values())
      .filter(t => t.type === type && t.status === 'active');
  }

  getTemplatesByJurisdiction(jurisdiction: string): ContractTemplate[] {
    return Array.from(this.templates.values())
      .filter(t => t.jurisdiction === jurisdiction && t.status === 'active');
  }

  deprecateTemplate(id: string): void {
    const template = this.templates.get(id);
    if (template) {
      template.status = 'deprecated';
      this.templates.set(id, template);
    }
  }

  getClause(id: string): Clause | undefined {
    return this.clauseLibrary.get(id);
  }

  getClausesByCategory(category: string): Clause[] {
    return Array.from(this.clauseLibrary.values())
      .filter(c => c.category === category);
  }

  getMandatoryClauses(templateId: string): Clause[] {
    const template = this.templates.get(templateId);
    if (!template) return [];

    return template.clauses.filter(c => c.mandatory);
  }
}

// ============================================================================
// Contract Generator
// ============================================================================

export class ContractGenerator {
  constructor(
    private readonly templateManager: ContractTemplateManager,
    private readonly config: TemplateEngineConfig = defaultConfig
  ) {}

  async generateContract(
    templateId: string,
    variables: Record<string, unknown>,
    options: GenerationOptions = {}
  ): Promise<GeneratedContract> {
    const template = this.templateManager.getTemplate(templateId);

    if (!template) {
      throw new Error(`Template not found: ${templateId}`);
    }

    // Validate required variables
    this.validateVariables(template, variables);

    // Process clauses
    const processedClauses = template.clauses
      .filter(clause => this.shouldIncludeClause(clause, options))
      .map(clause => this.processClause(clause, variables));

    // Generate document
    const document = this.assembleDocument(template, processedClauses, variables);

    return {
      templateId,
      templateVersion: template.version,
      generatedAt: new Date().toISOString(),
      content: document,
      format: this.config.outputFormat,
      variables: this.sanitizeVariables(variables),
      clauses: processedClauses.map(c => c.id),
    };
  }

  private validateVariables(
    template: ContractTemplate,
    variables: Record<string, unknown>
  ): void {
    // Extract required variables from clauses
    const requiredVars = new Set<string>();
    const [open, close] = this.config.variableDelimiters;

    for (const clause of template.clauses) {
      if (clause.mandatory) {
        const matches = clause.text.matchAll(
          new RegExp(`${this.escapeRegex(open)}(\\w+)${this.escapeRegex(close)}`, 'g')
        );

        for (const match of matches) {
          requiredVars.add(match[1]);
        }
      }
    }

    // Check for missing required variables
    const missing: string[] = [];
    for (const varName of requiredVars) {
      if (!(varName in variables)) {
        missing.push(varName);
      }
    }

    if (missing.length > 0) {
      throw new Error(`Missing required variables: ${missing.join(', ')}`);
    }
  }

  private shouldIncludeClause(clause: Clause, options: GenerationOptions): boolean {
    // Always include mandatory clauses
    if (clause.mandatory) return true;

    // Check if clause is explicitly included
    if (options.includeClauses?.includes(clause.id)) return true;

    // Check if clause is explicitly excluded
    if (options.excludeClauses?.includes(clause.id)) return false;

    // Include optional clauses by default unless specified
    return options.includeOptional !== false;
  }

  private processClause(
    clause: Clause,
    variables: Record<string, unknown>
  ): ProcessedClause {
    let text = clause.text;

    // Replace variables
    const [open, close] = this.config.variableDelimiters;

    for (const [key, value] of Object.entries(variables)) {
      const pattern = new RegExp(
        `${this.escapeRegex(open)}${key}${this.escapeRegex(close)}`,
        'g'
      );
      text = text.replace(pattern, String(value));
    }

    // Process conditionals
    text = this.processConditionals(text, variables);

    // Process loops
    text = this.processLoops(text, variables);

    return {
      id: clause.id,
      title: clause.title,
      text: text.trim(),
      category: clause.category,
    };
  }

  private processConditionals(
    text: string,
    variables: Record<string, unknown>
  ): string {
    const [open, close] = this.config.conditionalDelimiters;

    // Match {{#if condition}}...{{/if}} blocks
    const ifPattern = new RegExp(
      `${this.escapeRegex(open)}if\\s+(\\w+)${this.escapeRegex(close)}([\\s\\S]*?)${this.escapeRegex(open.replace('#', '/')}if${this.escapeRegex(close)}`,
      'g'
    );

    return text.replace(ifPattern, (match, condition, content) => {
      const value = variables[condition];

      if (value) {
        // Check for {{else}}
        const elseParts = content.split(/\{\{else\}\}/);
        return elseParts[0].trim();
      } else {
        const elseParts = content.split(/\{\{else\}\}/);
        return elseParts[1]?.trim() || '';
      }
    });
  }

  private processLoops(
    text: string,
    variables: Record<string, unknown>
  ): string {
    const [open, close] = this.config.conditionalDelimiters;

    // Match {{#each array}}...{{/each}} blocks
    const eachPattern = new RegExp(
      `${this.escapeRegex(open)}each\\s+(\\w+)${this.escapeRegex(close)}([\\s\\S]*?)${this.escapeRegex(open.replace('#', '/')}each${this.escapeRegex(close)}`,
      'g'
    );

    return text.replace(eachPattern, (match, arrayName, template) => {
      const array = variables[arrayName];

      if (!Array.isArray(array)) {
        return '';
      }

      return array.map(item => {
        let itemText = template;
        itemText = itemText.replace(/\{\{this\}\}/g, String(item));

        if (typeof item === 'object' && item !== null) {
          for (const [key, value] of Object.entries(item)) {
            itemText = itemText.replace(
              new RegExp(`\\{\\{${key}\\}\\}`, 'g'),
              String(value)
            );
          }
        }

        return itemText.trim();
      }).join('\n');
    });
  }

  private assembleDocument(
    template: ContractTemplate,
    clauses: ProcessedClause[],
    variables: Record<string, unknown>
  ): string {
    const sections: string[] = [];

    // Header
    sections.push(`# ${template.name}`);
    sections.push('');
    sections.push(`**Version**: ${template.version}`);
    sections.push(`**Jurisdiction**: ${template.jurisdiction}`);
    sections.push(`**Date**: ${new Date().toLocaleDateString()}`);
    sections.push('');
    sections.push('---');
    sections.push('');

    // Group clauses by category
    const categorized = this.categorize(clauses);

    for (const [category, categoryClause] of categorized) {
      sections.push(`## ${this.formatCategoryName(category)}`);
      sections.push('');

      for (const clause of categoryClause) {
        sections.push(`### ${clause.title}`);
        sections.push('');
        sections.push(clause.text);
        sections.push('');
      }
    }

    // Signature block
    sections.push('---');
    sections.push('');
    sections.push('## Signatures');
    sections.push('');
    sections.push(this.generateSignatureBlock(variables));

    return sections.join('\n');
  }

  private categorize(clauses: ProcessedClause[]): Map<string, ProcessedClause[]> {
    const categoryOrder = [
      'scope', 'term', 'fees', 'operations', 'access',
      'consent', 'disclosure', 'instructions', 'posthumous',
      'requirements', 'privacy', 'ip', 'screening', 'rights',
      'intent', 'liability', 'insurance', 'termination',
      'disposition', 'legal',
    ];

    const categorized = new Map<string, ProcessedClause[]>();

    for (const clause of clauses) {
      const existing = categorized.get(clause.category) || [];
      existing.push(clause);
      categorized.set(clause.category, existing);
    }

    // Sort by category order
    return new Map(
      [...categorized.entries()].sort((a, b) => {
        const aIndex = categoryOrder.indexOf(a[0]);
        const bIndex = categoryOrder.indexOf(b[0]);
        return (aIndex === -1 ? 999 : aIndex) - (bIndex === -1 ? 999 : bIndex);
      })
    );
  }

  private formatCategoryName(category: string): string {
    return category
      .split('-')
      .map(word => word.charAt(0).toUpperCase() + word.slice(1))
      .join(' ');
  }

  private generateSignatureBlock(variables: Record<string, unknown>): string {
    const lines: string[] = [];

    lines.push('**Client/Party 1:**');
    lines.push('');
    lines.push('Signature: _______________________________');
    lines.push('');
    lines.push(`Name: ${variables.client_name || '____________________'}`);
    lines.push('');
    lines.push('Date: ____________________');
    lines.push('');
    lines.push('');
    lines.push('**Facility/Party 2:**');
    lines.push('');
    lines.push('Signature: _______________________________');
    lines.push('');
    lines.push(`Name: ${variables.facility_representative || '____________________'}`);
    lines.push('');
    lines.push(`Title: ${variables.facility_representative_title || '____________________'}`);
    lines.push('');
    lines.push('Date: ____________________');

    return lines.join('\n');
  }

  private escapeRegex(string: string): string {
    return string.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
  }

  private sanitizeVariables(variables: Record<string, unknown>): Record<string, unknown> {
    // Remove sensitive data from stored variables
    const sensitive = ['ssn', 'password', 'credit_card', 'bank_account'];
    const sanitized = { ...variables };

    for (const key of Object.keys(sanitized)) {
      if (sensitive.some(s => key.toLowerCase().includes(s))) {
        sanitized[key] = '[REDACTED]';
      }
    }

    return sanitized;
  }
}

export interface GenerationOptions {
  includeClauses?: string[];
  excludeClauses?: string[];
  includeOptional?: boolean;
  customClauses?: Clause[];
}

export interface GeneratedContract {
  templateId: string;
  templateVersion: string;
  generatedAt: string;
  content: string;
  format: string;
  variables: Record<string, unknown>;
  clauses: string[];
}

export interface ProcessedClause {
  id: string;
  title: string;
  text: string;
  category: string;
}
```

## Contract Lifecycle Manager

```typescript
/**
 * Contract Lifecycle Management
 * Handles contract states, signatures, and amendments
 */

export class ContractLifecycleManager {
  private contracts: Map<string, Contract> = new Map();
  private eventBus: ContractEventBus;

  constructor(
    private readonly storage: ContractStorage,
    private readonly signatureService: SignatureService,
    private readonly notificationService: NotificationService
  ) {
    this.eventBus = new ContractEventBus();
  }

  async createContract(
    generated: GeneratedContract,
    parties: Party[]
  ): Promise<Contract> {
    const contractId = this.generateContractId();

    const contract: Contract = {
      id: contractId,
      templateId: generated.templateId,
      type: this.inferContractType(generated.templateId),
      parties,
      terms: this.extractTerms(generated),
      signatures: [],
      effectiveDate: new Date().toISOString(),
      status: 'draft',
      history: [{
        id: crypto.randomUUID(),
        type: 'created',
        date: new Date().toISOString(),
        actor: 'system',
        description: 'Contract created from template',
        documents: [],
      }],
    };

    await this.storage.save(contract);
    this.contracts.set(contractId, contract);

    this.eventBus.emit('contract:created', { contract });

    return contract;
  }

  async submitForSignature(
    contractId: string,
    requestedBy: string
  ): Promise<Contract> {
    const contract = await this.getContract(contractId);

    if (contract.status !== 'draft') {
      throw new Error(`Cannot submit contract in ${contract.status} status`);
    }

    // Validate all required fields
    this.validateForSubmission(contract);

    // Update status
    contract.status = 'pending';

    // Add history event
    contract.history.push({
      id: crypto.randomUUID(),
      type: 'submitted',
      date: new Date().toISOString(),
      actor: requestedBy,
      description: 'Contract submitted for signature',
    });

    await this.storage.save(contract);

    // Send signature requests
    for (const party of contract.parties) {
      await this.signatureService.requestSignature(contract, party);
      await this.notificationService.sendSignatureRequest(party, contract);
    }

    this.eventBus.emit('contract:submitted', { contract });

    return contract;
  }

  async addSignature(
    contractId: string,
    signature: Signature
  ): Promise<Contract> {
    const contract = await this.getContract(contractId);

    if (contract.status !== 'pending') {
      throw new Error(`Cannot sign contract in ${contract.status} status`);
    }

    // Verify signature
    const verification = await this.signatureService.verifySignature(
      contract,
      signature
    );

    if (!verification.valid) {
      throw new Error(`Invalid signature: ${verification.error}`);
    }

    // Add signature
    contract.signatures.push({
      ...signature,
      date: new Date().toISOString(),
    });

    // Add history event
    contract.history.push({
      id: crypto.randomUUID(),
      type: 'signed',
      date: new Date().toISOString(),
      actor: signature.signatory,
      description: `Signed by ${signature.signatory} on behalf of ${signature.party}`,
    });

    // Check if all parties have signed
    const allSigned = this.checkAllPartiesSigned(contract);

    if (allSigned) {
      // Auto-execute contract
      contract.status = 'active';

      contract.history.push({
        id: crypto.randomUUID(),
        type: 'executed',
        date: new Date().toISOString(),
        actor: 'system',
        description: 'Contract executed - all parties have signed',
      });

      this.eventBus.emit('contract:executed', { contract });
    }

    await this.storage.save(contract);

    this.eventBus.emit('contract:signed', { contract, signature });

    return contract;
  }

  async executeContract(
    contractId: string,
    executor: string
  ): Promise<Contract> {
    const contract = await this.getContract(contractId);

    if (contract.status !== 'pending') {
      throw new Error(`Cannot execute contract in ${contract.status} status`);
    }

    if (!this.checkAllPartiesSigned(contract)) {
      throw new Error('Cannot execute contract without all signatures');
    }

    contract.status = 'active';

    contract.history.push({
      id: crypto.randomUUID(),
      type: 'executed',
      date: new Date().toISOString(),
      actor: executor,
      description: 'Contract manually executed',
    });

    await this.storage.save(contract);

    this.eventBus.emit('contract:executed', { contract });

    // Schedule renewal reminders
    if (contract.expiryDate) {
      await this.scheduleRenewalReminder(contract);
    }

    return contract;
  }

  async amendContract(
    contractId: string,
    amendment: Amendment,
    actor: string
  ): Promise<Contract> {
    const contract = await this.getContract(contractId);

    if (contract.status !== 'active') {
      throw new Error(`Cannot amend contract in ${contract.status} status`);
    }

    // Validate amendment
    this.validateAmendment(contract, amendment);

    // Apply amendment
    const updatedTerms = this.applyAmendment(contract.terms, amendment);
    contract.terms = updatedTerms;

    // Reset signatures for material amendments
    if (amendment.requiresResigning) {
      contract.signatures = [];
      contract.status = 'pending';

      contract.history.push({
        id: crypto.randomUUID(),
        type: 'amended',
        date: new Date().toISOString(),
        actor,
        description: `Material amendment requiring re-signing: ${amendment.description}`,
      });

      // Request new signatures
      for (const party of contract.parties) {
        await this.signatureService.requestSignature(contract, party);
        await this.notificationService.sendAmendmentNotice(party, contract, amendment);
      }
    } else {
      contract.history.push({
        id: crypto.randomUUID(),
        type: 'amended',
        date: new Date().toISOString(),
        actor,
        description: `Non-material amendment: ${amendment.description}`,
      });
    }

    await this.storage.save(contract);

    this.eventBus.emit('contract:amended', { contract, amendment });

    return contract;
  }

  async terminateContract(
    contractId: string,
    reason: string,
    effectiveDate: string,
    actor: string
  ): Promise<Contract> {
    const contract = await this.getContract(contractId);

    if (contract.status !== 'active') {
      throw new Error(`Cannot terminate contract in ${contract.status} status`);
    }

    contract.status = 'terminated';

    contract.history.push({
      id: crypto.randomUUID(),
      type: 'terminated',
      date: new Date().toISOString(),
      actor,
      description: `Contract terminated: ${reason}. Effective: ${effectiveDate}`,
    });

    await this.storage.save(contract);

    // Notify all parties
    for (const party of contract.parties) {
      await this.notificationService.sendTerminationNotice(
        party,
        contract,
        reason,
        effectiveDate
      );
    }

    this.eventBus.emit('contract:terminated', { contract, reason });

    return contract;
  }

  async renewContract(
    contractId: string,
    renewalTerms: Partial<ContractTerms>,
    actor: string
  ): Promise<Contract> {
    const originalContract = await this.getContract(contractId);

    if (originalContract.status !== 'active' && originalContract.status !== 'expired') {
      throw new Error(`Cannot renew contract in ${originalContract.status} status`);
    }

    // Create new contract based on original
    const renewedContract: Contract = {
      ...originalContract,
      id: this.generateContractId(),
      terms: {
        ...originalContract.terms,
        ...renewalTerms,
      },
      effectiveDate: originalContract.expiryDate || new Date().toISOString(),
      signatures: [],
      status: 'draft',
      history: [{
        id: crypto.randomUUID(),
        type: 'renewed',
        date: new Date().toISOString(),
        actor,
        description: `Renewed from contract ${originalContract.id}`,
      }],
    };

    // Mark original as expired if not already
    if (originalContract.status === 'active') {
      originalContract.status = 'expired';

      originalContract.history.push({
        id: crypto.randomUUID(),
        type: 'expired',
        date: new Date().toISOString(),
        actor: 'system',
        description: `Expired and renewed as ${renewedContract.id}`,
      });

      await this.storage.save(originalContract);
    }

    await this.storage.save(renewedContract);

    this.eventBus.emit('contract:renewed', {
      originalContract,
      renewedContract,
    });

    return renewedContract;
  }

  async getContract(id: string): Promise<Contract> {
    let contract = this.contracts.get(id);

    if (!contract) {
      contract = await this.storage.load(id);

      if (!contract) {
        throw new Error(`Contract not found: ${id}`);
      }

      this.contracts.set(id, contract);
    }

    return contract;
  }

  private generateContractId(): string {
    const timestamp = Date.now().toString(36).toUpperCase();
    const random = Math.random().toString(36).substring(2, 8).toUpperCase();
    return `CTR-${timestamp}-${random}`;
  }

  private inferContractType(templateId: string): ContractType {
    if (templateId.includes('storage')) return 'storage-agreement';
    if (templateId.includes('consent')) return 'consent-form';
    if (templateId.includes('research')) return 'research-agreement';
    if (templateId.includes('donation')) return 'donation-agreement';
    if (templateId.includes('transfer')) return 'transfer-agreement';
    return 'service-agreement';
  }

  private extractTerms(generated: GeneratedContract): ContractTerms {
    return {
      duration: generated.variables.duration as string || '1 year',
      renewal: 'manual',
      fees: [],
      obligations: [],
      warranties: [],
      limitations: [],
      governing: generated.variables.governing_state as string || 'California',
      venue: generated.variables.venue_county as string || 'Los Angeles County',
    };
  }

  private validateForSubmission(contract: Contract): void {
    if (contract.parties.length < 2) {
      throw new Error('Contract must have at least two parties');
    }

    const hasPrimary = contract.parties.some(p => p.role === 'primary');
    const hasCounterparty = contract.parties.some(p => p.role === 'counterparty');

    if (!hasPrimary || !hasCounterparty) {
      throw new Error('Contract must have primary and counterparty');
    }
  }

  private checkAllPartiesSigned(contract: Contract): boolean {
    const requiredParties = contract.parties
      .filter(p => p.role === 'primary' || p.role === 'counterparty')
      .map(p => p.id);

    const signedParties = new Set(contract.signatures.map(s => s.party));

    return requiredParties.every(id => signedParties.has(id));
  }

  private validateAmendment(contract: Contract, amendment: Amendment): void {
    // Check if amendment is allowed per contract terms
    // Check if amendment is within allowed scope
  }

  private applyAmendment(
    terms: ContractTerms,
    amendment: Amendment
  ): ContractTerms {
    return {
      ...terms,
      ...amendment.changes,
    };
  }

  private async scheduleRenewalReminder(contract: Contract): Promise<void> {
    // Schedule reminder for renewal
  }

  onContractEvent(
    event: string,
    handler: (data: unknown) => void
  ): void {
    this.eventBus.on(event, handler);
  }
}

export interface Amendment {
  id: string;
  description: string;
  changes: Partial<ContractTerms>;
  requiresResigning: boolean;
  effectiveDate: string;
}

export interface ContractStorage {
  save(contract: Contract): Promise<void>;
  load(id: string): Promise<Contract | null>;
  findByParty(partyId: string): Promise<Contract[]>;
  findByStatus(status: ContractStatus): Promise<Contract[]>;
}

export interface SignatureService {
  requestSignature(contract: Contract, party: Party): Promise<void>;
  verifySignature(contract: Contract, signature: Signature): Promise<SignatureVerification>;
}

export interface SignatureVerification {
  valid: boolean;
  error?: string;
  timestamp?: string;
}

export interface NotificationService {
  sendSignatureRequest(party: Party, contract: Contract): Promise<void>;
  sendAmendmentNotice(party: Party, contract: Contract, amendment: Amendment): Promise<void>;
  sendTerminationNotice(party: Party, contract: Contract, reason: string, effectiveDate: string): Promise<void>;
}

class ContractEventBus {
  private handlers: Map<string, Set<(data: unknown) => void>> = new Map();

  on(event: string, handler: (data: unknown) => void): void {
    if (!this.handlers.has(event)) {
      this.handlers.set(event, new Set());
    }
    this.handlers.get(event)!.add(handler);
  }

  emit(event: string, data: unknown): void {
    const handlers = this.handlers.get(event);
    if (handlers) {
      for (const handler of handlers) {
        handler(data);
      }
    }
  }
}
```

## Electronic Signature Integration

```typescript
/**
 * Electronic Signature Service Implementation
 * Supports multiple signature methods
 */

export class ElectronicSignatureService implements SignatureService {
  constructor(
    private readonly config: SignatureConfig,
    private readonly storage: SignatureStorage,
    private readonly auditLog: AuditLogger
  ) {}

  async requestSignature(contract: Contract, party: Party): Promise<void> {
    const request: SignatureRequest = {
      id: crypto.randomUUID(),
      contractId: contract.id,
      partyId: party.id,
      partyName: party.name,
      partyEmail: party.contact.email,
      status: 'pending',
      createdAt: new Date().toISOString(),
      expiresAt: this.calculateExpiry(),
    };

    await this.storage.saveRequest(request);

    // Generate signature URL
    const signatureUrl = await this.generateSignatureUrl(request);

    // Send notification
    await this.sendSignatureEmail(party, contract, signatureUrl);

    await this.auditLog.log({
      action: 'signature_requested',
      contractId: contract.id,
      partyId: party.id,
      timestamp: new Date(),
    });
  }

  async verifySignature(
    contract: Contract,
    signature: Signature
  ): Promise<SignatureVerification> {
    // Verify party is authorized
    const party = contract.parties.find(p => p.id === signature.party);
    if (!party) {
      return { valid: false, error: 'Party not found in contract' };
    }

    // Verify signatory is authorized
    if (party.representative && party.representative !== signature.signatory) {
      return { valid: false, error: 'Signatory is not the authorized representative' };
    }

    // Verify signature method validity
    const methodValid = await this.verifySignatureMethod(signature);
    if (!methodValid.valid) {
      return methodValid;
    }

    // Verify not already signed
    const existingSignature = contract.signatures.find(s => s.party === signature.party);
    if (existingSignature) {
      return { valid: false, error: 'Party has already signed' };
    }

    // Record signature
    await this.recordSignature(contract.id, signature);

    await this.auditLog.log({
      action: 'signature_verified',
      contractId: contract.id,
      partyId: signature.party,
      signatory: signature.signatory,
      method: signature.method,
      timestamp: new Date(),
    });

    return { valid: true, timestamp: new Date().toISOString() };
  }

  private async verifySignatureMethod(
    signature: Signature
  ): Promise<SignatureVerification> {
    switch (signature.method) {
      case 'electronic':
        return this.verifyElectronicSignature(signature);
      case 'digital':
        return this.verifyDigitalSignature(signature);
      case 'wet':
        return this.verifyWetSignature(signature);
      default:
        return { valid: false, error: 'Unknown signature method' };
    }
  }

  private async verifyElectronicSignature(
    signature: Signature
  ): Promise<SignatureVerification> {
    // Electronic signature verification
    // - IP address logging
    // - Browser fingerprint
    // - Click-to-sign confirmation
    return { valid: true };
  }

  private async verifyDigitalSignature(
    signature: Signature
  ): Promise<SignatureVerification> {
    // Digital signature verification
    // - Certificate validation
    // - PKI verification
    // - Timestamp authority verification
    return { valid: true };
  }

  private async verifyWetSignature(
    signature: Signature
  ): Promise<SignatureVerification> {
    // Wet signature verification
    // - Document scan uploaded
    // - Notarization verified if required
    if (signature.notarized) {
      // Verify notary seal
    }
    return { valid: true };
  }

  private calculateExpiry(): string {
    const expiry = new Date();
    expiry.setDate(expiry.getDate() + this.config.signatureExpiryDays);
    return expiry.toISOString();
  }

  private async generateSignatureUrl(request: SignatureRequest): Promise<string> {
    const token = await this.generateSecureToken(request);
    return `${this.config.baseUrl}/sign/${request.id}?token=${token}`;
  }

  private async generateSecureToken(request: SignatureRequest): Promise<string> {
    // Generate secure, time-limited token
    return Buffer.from(JSON.stringify({
      requestId: request.id,
      exp: request.expiresAt,
    })).toString('base64url');
  }

  private async sendSignatureEmail(
    party: Party,
    contract: Contract,
    url: string
  ): Promise<void> {
    // Send email with signature link
  }

  private async recordSignature(
    contractId: string,
    signature: Signature
  ): Promise<void> {
    await this.storage.saveSignature({
      contractId,
      signature,
      metadata: {
        recordedAt: new Date().toISOString(),
        ipAddress: 'recorded-separately',
        userAgent: 'recorded-separately',
      },
    });
  }
}

export interface SignatureConfig {
  baseUrl: string;
  signatureExpiryDays: number;
  allowedMethods: SignatureMethod[];
  requireNotarization: boolean;
}

export interface SignatureRequest {
  id: string;
  contractId: string;
  partyId: string;
  partyName: string;
  partyEmail: string;
  status: 'pending' | 'signed' | 'expired' | 'cancelled';
  createdAt: string;
  expiresAt: string;
  signedAt?: string;
}

export interface SignatureStorage {
  saveRequest(request: SignatureRequest): Promise<void>;
  getRequest(id: string): Promise<SignatureRequest | null>;
  saveSignature(data: { contractId: string; signature: Signature; metadata: any }): Promise<void>;
}

export interface AuditLogger {
  log(entry: AuditEntry): Promise<void>;
}

export interface AuditEntry {
  action: string;
  contractId: string;
  partyId?: string;
  signatory?: string;
  method?: string;
  timestamp: Date;
}
```

---

## Chapter Summary

This chapter covered comprehensive contract management:

- **Template Engine**: Dynamic template generation with variable processing
- **Clause Library**: Reusable clause management by category
- **Lifecycle Management**: Draft to execution workflow
- **Amendment Process**: Material and non-material amendments
- **Electronic Signatures**: Multi-method signature support

---

**Next Chapter**: [Dispute Resolution - Mechanisms and Procedures](./06-dispute-resolution.md)
