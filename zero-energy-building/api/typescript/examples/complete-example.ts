/**
 * Complete Example: Zero Energy Building Monitoring
 *
 * This example demonstrates a complete workflow for monitoring
 * and certifying a net-zero energy building.
 */

import {
  createZeroEnergyBuilding,
  EnergySourceType,
  ConsumptionCategory,
  ZEBEventType,
  BuildingMetadata
} from '../src';

async function main() {
  console.log('🏢 WIA Zero Energy Building SDK - Complete Example\n');

  // Step 1: Initialize the SDK
  console.log('Step 1: Initializing Zero Energy Building SDK...');
  const zeb = createZeroEnergyBuilding({
    buildingId: 'green-office-001',
    enableRealTimeMonitoring: true,
    monitoringInterval: 60,
    units: 'metric'
  });

  // Step 2: Set up event handlers
  console.log('Step 2: Setting up event handlers...');

  zeb.on(ZEBEventType.ENERGY_SURPLUS, (event) => {
    console.log(`✅ ${event.message}`);
  });

  zeb.on(ZEBEventType.ENERGY_DEFICIT, (event) => {
    console.log(`⚠️  ${event.message}`);
  });

  zeb.on(ZEBEventType.PRODUCTION_ANOMALY, (event) => {
    console.log(`🔧 ${event.message}`);
  });

  zeb.on(ZEBEventType.CONSUMPTION_SPIKE, (event) => {
    console.log(`📈 ${event.message}`);
  });

  // Step 3: Initialize building metadata
  console.log('Step 3: Initializing building metadata...');

  const buildingMetadata: BuildingMetadata = {
    id: 'green-office-001',
    name: 'Green Office Complex',
    type: 'commercial',
    location: {
      address: '123 Sustainability Street, San Francisco, CA',
      latitude: 37.7749,
      longitude: -122.4194,
      climateZone: 'Zone 3C - Warm Marine'
    },
    size: {
      floorArea: 5000, // m²
      volume: 15000, // m³
      stories: 3
    },
    occupancy: 200,
    yearBuilt: 2020,
    yearRetrofitted: 2024
  };

  await zeb.initializeBuilding(buildingMetadata);
  console.log(`✓ Building initialized: ${buildingMetadata.name}\n`);

  // Step 4: Track energy production from multiple sources
  console.log('Step 4: Tracking energy production...');

  // Solar PV production
  await zeb.trackProduction({
    sourceType: EnergySourceType.SOLAR_PV,
    capacity: 150, // kW
    currentOutput: 120, // kW (80% of capacity)
    dailyProduction: 800, // kWh
    monthlyProduction: 24000, // kWh
    yearlyProduction: 288000, // kWh
    efficiency: 85,
    timestamp: new Date(),
    metadata: {
      panelCount: 500,
      panelWattage: 300,
      inverterType: 'String Inverter'
    }
  });
  console.log('  ☀️  Solar PV: 120 kW (85% efficiency)');

  // Wind turbine production
  await zeb.trackProduction({
    sourceType: EnergySourceType.WIND,
    capacity: 50, // kW
    currentOutput: 35, // kW
    dailyProduction: 350, // kWh
    monthlyProduction: 10500, // kWh
    yearlyProduction: 126000, // kWh
    efficiency: 78,
    timestamp: new Date(),
    metadata: {
      turbineCount: 2,
      windSpeed: 8.5 // m/s
    }
  });
  console.log('  💨 Wind Turbine: 35 kW (78% efficiency)');

  // Battery storage (discharging)
  await zeb.trackProduction({
    sourceType: EnergySourceType.BATTERY_STORAGE,
    capacity: 100, // kWh capacity
    currentOutput: 15, // kW
    dailyProduction: 50, // kWh
    monthlyProduction: 1500, // kWh
    yearlyProduction: 18000, // kWh
    efficiency: 95,
    timestamp: new Date(),
    metadata: {
      stateOfCharge: 75,
      batteryType: 'Lithium-Ion'
    }
  });
  console.log('  🔋 Battery Storage: 15 kW (95% efficiency, 75% SoC)\n');

  // Step 5: Track energy consumption by category
  console.log('Step 5: Tracking energy consumption...');

  await zeb.trackConsumption({
    category: ConsumptionCategory.HEATING,
    currentPower: 30,
    dailyConsumption: 250,
    monthlyConsumption: 7500,
    yearlyConsumption: 90000,
    peakDemand: 45,
    timestamp: new Date()
  });
  console.log('  🌡️  Heating: 30 kW');

  await zeb.trackConsumption({
    category: ConsumptionCategory.COOLING,
    currentPower: 40,
    dailyConsumption: 320,
    monthlyConsumption: 9600,
    yearlyConsumption: 115200,
    peakDemand: 60,
    timestamp: new Date()
  });
  console.log('  ❄️  Cooling: 40 kW');

  await zeb.trackConsumption({
    category: ConsumptionCategory.LIGHTING,
    currentPower: 15,
    dailyConsumption: 120,
    monthlyConsumption: 3600,
    yearlyConsumption: 43200,
    peakDemand: 20,
    timestamp: new Date()
  });
  console.log('  💡 Lighting: 15 kW');

  await zeb.trackConsumption({
    category: ConsumptionCategory.APPLIANCES,
    currentPower: 25,
    dailyConsumption: 200,
    monthlyConsumption: 6000,
    yearlyConsumption: 72000,
    peakDemand: 35,
    timestamp: new Date()
  });
  console.log('  🖥️  Appliances: 25 kW');

  await zeb.trackConsumption({
    category: ConsumptionCategory.VENTILATION,
    currentPower: 10,
    dailyConsumption: 80,
    monthlyConsumption: 2400,
    yearlyConsumption: 28800,
    peakDemand: 15,
    timestamp: new Date()
  });
  console.log('  🌬️  Ventilation: 10 kW\n');

  // Step 6: Calculate energy balance
  console.log('Step 6: Calculating energy balance...');

  const dailyBalance = await zeb.calculateEnergyBalance('daily');
  console.log('\n📊 Daily Energy Balance:');
  console.log(`  Production:     ${dailyBalance.totalProduction.toFixed(2)} kWh`);
  console.log(`  Consumption:    ${dailyBalance.totalConsumption.toFixed(2)} kWh`);
  console.log(`  Net Balance:    ${dailyBalance.netBalance.toFixed(2)} kWh`);
  console.log(`  Grid Export:    ${dailyBalance.gridExport.toFixed(2)} kWh`);
  console.log(`  Grid Import:    ${dailyBalance.gridImport.toFixed(2)} kWh`);
  console.log(`  Self-Consumption: ${dailyBalance.selfConsumptionRate.toFixed(1)}%`);
  console.log(`  Self-Sufficiency: ${dailyBalance.selfSufficiencyRate.toFixed(1)}%`);

  const yearlyBalance = await zeb.calculateEnergyBalance('yearly');
  console.log('\n📊 Yearly Energy Balance:');
  console.log(`  Production:     ${yearlyBalance.totalProduction.toLocaleString()} kWh`);
  console.log(`  Consumption:    ${yearlyBalance.totalConsumption.toLocaleString()} kWh`);
  console.log(`  Net Balance:    ${yearlyBalance.netBalance.toLocaleString()} kWh`);
  console.log(`  Self-Sufficiency: ${yearlyBalance.selfSufficiencyRate.toFixed(1)}%\n`);

  // Step 7: Assess certification level
  console.log('Step 7: Assessing certification level...');

  const assessment = await zeb.assessCertification();

  console.log('\n🏆 Certification Assessment:');
  console.log(`  Level:              ${assessment.level.toUpperCase()}`);
  console.log(`  Score:              ${assessment.score.toFixed(1)}/100`);
  console.log(`  Meets Requirements: ${assessment.meetsMinimumRequirements ? 'Yes' : 'No'}`);
  console.log(`  Self-Sufficiency:   ${assessment.selfSufficiencyRate.toFixed(1)}%`);
  console.log(`  Renewable %:        ${assessment.renewablePercentage.toFixed(1)}%`);
  console.log(`  Energy Intensity:   ${assessment.energyIntensity.toFixed(2)} kWh/m²/year`);
  console.log(`  Carbon Footprint:   ${assessment.carbonFootprint.toFixed(0)} kg CO2/year`);
  console.log(`  Valid Until:        ${assessment.validUntil.toLocaleDateString()}`);

  if (assessment.recommendations.length > 0) {
    console.log('\n  Recommendations:');
    assessment.recommendations.forEach((rec, i) => {
      console.log(`    ${i + 1}. ${rec}`);
    });
  }

  // Step 8: Get optimization recommendations
  console.log('\n\nStep 8: Getting optimization recommendations...');

  const recommendations = await zeb.getOptimizationRecommendations();

  console.log(`\n💡 Optimization Recommendations (${recommendations.length} found):\n`);

  recommendations.forEach((rec, index) => {
    console.log(`${index + 1}. ${rec.title} [${rec.priority.toUpperCase()}]`);
    console.log(`   Category: ${rec.category}`);
    console.log(`   ${rec.description}`);
    console.log(`   💰 Estimated Savings: ${rec.estimatedSavings.toLocaleString()} kWh/year`);
    console.log(`   💵 Estimated Cost: $${rec.estimatedCost.toLocaleString()}`);
    console.log(`   ⏱️  Payback Period: ${rec.paybackPeriod.toFixed(1)} years`);
    console.log(`   🌱 Carbon Reduction: ${rec.carbonReduction.toLocaleString()} kg CO2/year`);
    console.log(`   🔧 Complexity: ${rec.implementationComplexity}`);
    console.log('');
  });

  // Step 9: Summary
  console.log('═'.repeat(60));
  console.log('Summary:');
  console.log('═'.repeat(60));
  console.log(`Building: ${buildingMetadata.name}`);
  console.log(`Certification: ${assessment.level.toUpperCase()} (${assessment.score.toFixed(1)}/100)`);
  console.log(`Net Zero Status: ${yearlyBalance.netBalance >= 0 ? '✅ Achieved' : '❌ Not Achieved'}`);
  console.log(`Annual Net Balance: ${yearlyBalance.netBalance.toFixed(0)} kWh`);
  console.log(`Optimization Opportunities: ${recommendations.length} identified`);
  console.log('═'.repeat(60));

  // Cleanup
  zeb.destroy();
  console.log('\n✓ SDK resources cleaned up');
}

// Run the example
main().catch(console.error);
