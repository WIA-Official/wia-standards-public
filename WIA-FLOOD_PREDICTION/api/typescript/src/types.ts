export interface FloodPrediction {
  id: string;
  location: {lat: number; lng: number};
  forecast_date: string;
  risk_level: 'low' | 'medium' | 'high' | 'extreme';
  probability: number;
  predicted_depth_meters: number;
  affected_area_km2: number;
}

export interface RiverGauge {
  id: string;
  name: string;
  location: {lat: number; lng: number};
  current_level: number;
  flood_stage: number;
  forecast: Array<{timestamp: string; level: number}>;
}
