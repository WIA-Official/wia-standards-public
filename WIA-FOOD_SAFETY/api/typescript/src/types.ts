export interface FoodProduct {
  id: string;
  name: string;
  batch_number: string;
  origin: Location;
  current_location: Location;
  temperature_history: TemperatureRecord[];
  certifications: string[];
  status: 'safe' | 'warning' | 'recalled';
}

export interface Location {
  facility_name: string;
  address: string;
  coordinates: {lat: number; lng: number};
  timestamp: string;
}

export interface TemperatureRecord {
  timestamp: string;
  temperature_celsius: number;
  humidity_percent: number;
  sensor_id: string;
}

export interface RecallNotice {
  id: string;
  product_id: string;
  batch_numbers: string[];
  reason: string;
  severity: 'class1' | 'class2' | 'class3';
  issued_date: string;
}
