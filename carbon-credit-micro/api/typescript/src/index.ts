/**
 * WIA Carbon Credit Micro Standard - SDK Implementation
 * @packageDocumentation
 * @module wia-carbon-credit-micro
 */

import { EventEmitter } from 'events';
import * as types from './types';

export * from './types';

export class WIACarbonCreditMicro extends EventEmitter {
  private config: types.CarbonCreditMicroConfig;
  private projects: Map<string, types.CarbonProject> = new Map();
  private credits: Map<string, types.MicroCredit> = new Map();
  private wallets: Map<string, types.Wallet> = new Map();
  private emissionFactors: Map<string, types.EmissionFactor> = new Map();

  constructor(config: types.CarbonCreditMicroConfig) {
    super();
    this.config = config;
    this.loadEmissionFactors();
  }

  private loadEmissionFactors(): void {
    const factors: Partial<types.EmissionFactor>[] = [
      { category: types.ActivityCategory.Transport, activity: 'car_km', factor: 0.21, unit: 'kg CO2/km' },
      { category: types.ActivityCategory.Transport, activity: 'flight_km', factor: 0.255, unit: 'kg CO2/km' },
      { category: types.ActivityCategory.Energy, activity: 'electricity_kwh', factor: 0.5, unit: 'kg CO2/kWh' },
      { category: types.ActivityCategory.Food, activity: 'beef_kg', factor: 27, unit: 'kg CO2/kg' },
      { category: types.ActivityCategory.Food, activity: 'chicken_kg', factor: 6.9, unit: 'kg CO2/kg' }
    ];

    factors.forEach((f, i) => {
      const ef: types.EmissionFactor = {
        id: `ef-${i}`,
        category: f.category!,
        activity: f.activity!,
        factor: f.factor!,
        unit: f.unit!,
        source: 'WIA Default',
        validFrom: new Date()
      };
      this.emissionFactors.set(ef.activity, ef);
    });
  }

  async registerProject(project: Omit<types.CarbonProject, 'id'>): Promise<types.CarbonProject> {
    const newProject: types.CarbonProject = { ...project, id: `proj-${Date.now()}` };
    this.projects.set(newProject.id, newProject);
    this.emit('project-registered', newProject);
    return newProject;
  }

  getProject(projectId: string): types.CarbonProject | undefined {
    return this.projects.get(projectId);
  }

  listProjects(filter?: { type?: types.ProjectType; standard?: types.VerificationStandard }): types.CarbonProject[] {
    let projects = Array.from(this.projects.values());
    if (filter?.type) projects = projects.filter(p => p.type === filter.type);
    if (filter?.standard) projects = projects.filter(p => p.standard === filter.standard);
    return projects;
  }

  async issueCredits(projectId: string, amount: number): Promise<types.MicroCredit[]> {
    const project = this.projects.get(projectId);
    if (!project) throw new Error('Project not found');
    if (amount > project.availableCredits) throw new Error('Insufficient credits');

    const credits: types.MicroCredit[] = [];
    for (let i = 0; i < amount; i++) {
      const credit: types.MicroCredit = {
        id: `credit-${Date.now()}-${i}`,
        projectId,
        amount: 1,
        unit: 'kg',
        type: project.type === types.ProjectType.Reforestation ? types.CreditType.Sequestration : types.CreditType.Avoidance,
        vintage: new Date().getFullYear(),
        serialNumber: `WIA-${projectId.slice(-4)}-${Date.now()}-${i}`,
        issuedAt: new Date(),
        price: project.pricePerTonne / 1000,
        currency: project.currency
      };
      this.credits.set(credit.id, credit);
      credits.push(credit);
    }

    project.availableCredits -= amount;
    this.emit('credits-issued', { projectId, credits });
    return credits;
  }

  async getWallet(userId: string): Promise<types.Wallet> {
    let wallet = this.wallets.get(userId);
    if (!wallet) {
      wallet = {
        userId,
        balance: 0,
        currency: this.config.defaultCurrency,
        creditsOwned: [],
        totalOffset: 0,
        transactions: []
      };
      this.wallets.set(userId, wallet);
    }
    return wallet;
  }

  calculateEmissions(activities: Omit<types.Activity, 'id' | 'emissions' | 'emissionFactor' | 'timestamp'>[]): types.Activity[] {
    return activities.map((a, i) => {
      const factor = this.emissionFactors.get(a.name);
      const emissions = (factor?.factor || 0) * a.quantity;
      return {
        ...a,
        id: `act-${Date.now()}-${i}`,
        emissions,
        emissionFactor: factor?.factor || 0,
        timestamp: new Date()
      };
    });
  }

  async calculateFootprint(userId: string, activities: types.Activity[]): Promise<types.CarbonFootprint> {
    const wallet = await this.getWallet(userId);
    const totalEmissions = activities.reduce((sum, a) => sum + a.emissions, 0);

    const byCategory = new Map<types.ActivityCategory, types.Activity[]>();
    activities.forEach(a => {
      const list = byCategory.get(a.category) || [];
      list.push(a);
      byCategory.set(a.category, list);
    });

    const breakdown: types.EmissionBreakdown[] = Array.from(byCategory.entries()).map(([category, acts]) => {
      const emissions = acts.reduce((sum, a) => sum + a.emissions, 0);
      return {
        category,
        emissions,
        percentage: (emissions / totalEmissions) * 100,
        activities: acts
      };
    });

    return {
      userId,
      period: { start: new Date(), end: new Date(), type: 'day' },
      totalEmissions,
      breakdown,
      offset: wallet.totalOffset,
      netEmissions: totalEmissions - wallet.totalOffset,
      calculatedAt: new Date()
    };
  }

  async purchaseCredits(userId: string, creditIds: string[]): Promise<types.OffsetTransaction> {
    const wallet = await this.getWallet(userId);
    const credits = creditIds.map(id => this.credits.get(id)).filter(c => c && !c.retiredAt) as types.MicroCredit[];

    if (credits.length === 0) throw new Error('No valid credits found');

    const totalCost = credits.reduce((sum, c) => sum + c.price, 0);
    const totalAmount = credits.reduce((sum, c) => sum + c.amount, 0);

    const transaction: types.OffsetTransaction = {
      id: `tx-${Date.now()}`,
      userId,
      credits,
      totalAmount,
      totalCost,
      currency: credits[0].currency,
      status: 'completed',
      purchasedAt: new Date()
    };

    credits.forEach(c => {
      c.retiredAt = new Date();
      c.retiredBy = userId;
      wallet.creditsOwned.push(c);
    });

    wallet.totalOffset += totalAmount;
    wallet.transactions.push(transaction);

    this.emit('credits-purchased', transaction);
    return transaction;
  }

  async retireCredits(userId: string, amount: number): Promise<types.Certificate> {
    const wallet = await this.getWallet(userId);
    const available = wallet.creditsOwned.filter(c => !c.retiredAt);

    if (available.length < amount) throw new Error('Insufficient credits');

    const toRetire = available.slice(0, amount);
    toRetire.forEach(c => c.retiredAt = new Date());

    const certificate: types.Certificate = {
      id: `cert-${Date.now()}`,
      transactionId: `retire-${Date.now()}`,
      userId,
      amount,
      projectName: 'Mixed Projects',
      issuedAt: new Date(),
      certificateUrl: `https://wia.org/certificates/cert-${Date.now()}`,
      qrCode: `qr-${Date.now()}`
    };

    this.emit('credits-retired', { userId, amount, certificate });
    return certificate;
  }

  checkCompliance(targetLevel: types.CertificationLevel): types.ComplianceReport {
    const tests: types.TestResult[] = [];

    tests.push({
      testName: 'Configuration Validation',
      passed: this.config.apiEndpoint !== undefined,
      notes: 'API endpoint must be defined'
    });

    tests.push({
      testName: 'Emission Factors',
      passed: this.emissionFactors.size > 0,
      notes: 'Emission factors must be loaded'
    });

    if (targetLevel !== types.CertificationLevel.Bronze) {
      tests.push({
        testName: 'Gamification',
        passed: this.config.enableGamification === true,
        notes: 'Gamification required for Silver/Gold'
      });
    }

    const passed = tests.every(t => t.passed);

    return {
      standard: 'WIA-CARBON-CREDIT-MICRO',
      testDate: new Date().toISOString(),
      config: this.config,
      targetLevel,
      tests,
      passed,
      achievedLevel: passed ? targetLevel : undefined
    };
  }
}

export default { WIACarbonCreditMicro };
