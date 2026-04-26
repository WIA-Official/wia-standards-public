/**
 * WIA-EDU-024: Museum Digital Archive Standard - TypeScript Type Definitions
 *
 * @description Comprehensive type definitions for museum digital archives
 * @standard WIA-EDU-024
 * @version 1.0.0
 * @philosophy 弘益人間 (Benefit All Humanity)
 */

/**
 * Core Museum Object representing a catalog item
 */
export interface MuseumObject {
  "@context"?: string;
  "@type"?: "VisualArtwork" | "Sculpture" | "Photograph" | "Artifact";
  wiaStandard: "WIA-EDU-024";
  identifier: string;
  name: string;
  alternativeName?: string;
  creator?: Person | Person[];
  dateCreated?: string;
  dateModified?: string;
  artMedium?: string;
  artform?: string;
  artworkSurface?: string;
  width?: QuantitativeValue;
  height?: QuantitativeValue;
  depth?: QuantitativeValue;
  weight?: QuantitativeValue;
  description?: string;
  image?: ImageObject | ImageObject[];
  iiifManifest?: string;
  keywords?: string[];
  locationCreated?: Place;
  department: string;
  classification?: string;
  culture?: string;
  period?: string;
  dynasty?: string;
  reign?: string;
  creditLine?: string;
  license?: string;
  copyrightHolder?: Organization;
  copyrightYear?: number;
  isPublicDomain?: boolean;
  conditionReport?: ConditionReport;
  provenance?: ProvenanceEntry[];
  exhibitions?: Exhibition[];
  bibliography?: string[];
  relatedLink?: RelatedLink[];
  inscriptions?: Inscription[];
  marks?: string[];
  catalogedBy?: Person;
  catalogDate?: string;
  lastModified?: string;
  onView?: boolean;
  currentLocation?: string;
  culturalContext?: string;
  technicalObservations?: string;
}

/**
 * Person entity (artist, curator, etc.)
 */
export interface Person {
  "@type"?: "Person";
  name: string;
  givenName?: string;
  familyName?: string;
  birthDate?: string;
  deathDate?: string;
  birthPlace?: Place;
  deathPlace?: Place;
  nationality?: string;
  gender?: string;
  url?: string;
  sameAs?: string[];
  jobTitle?: string;
  email?: string;
}

/**
 * Organization entity (museum, institution)
 */
export interface Organization {
  "@type"?: "Organization" | "Museum";
  name: string;
  alternateName?: string;
  url?: string;
  logo?: string;
  address?: PostalAddress;
  contactPoint?: ContactPoint;
  sameAs?: string[];
}

/**
 * Postal address
 */
export interface PostalAddress {
  "@type"?: "PostalAddress";
  streetAddress?: string;
  addressLocality?: string;
  addressRegion?: string;
  postalCode?: string;
  addressCountry?: string;
}

/**
 * Contact information
 */
export interface ContactPoint {
  "@type"?: "ContactPoint";
  telephone?: string;
  email?: string;
  contactType?: string;
  availableLanguage?: string[];
}

/**
 * Geographic place
 */
export interface Place {
  "@type"?: "Place";
  name: string;
  address?: PostalAddress;
  geo?: GeoCoordinates;
  sameAs?: string[];
}

/**
 * Geographic coordinates
 */
export interface GeoCoordinates {
  "@type"?: "GeoCoordinates";
  latitude: number;
  longitude: number;
}

/**
 * Quantitative measurement
 */
export interface QuantitativeValue {
  "@type"?: "QuantitativeValue";
  value: number;
  unitCode?: string;
  unitText?: string;
}

/**
 * Image object with IIIF support
 */
export interface ImageObject {
  "@type"?: "ImageObject";
  contentUrl: string;
  encodingFormat?: string;
  width?: number;
  height?: number;
  caption?: string;
  license?: string;
  copyrightHolder?: string;
  thumbnail?: string;
  iiifService?: IIIFService;
}

/**
 * IIIF Image Service
 */
export interface IIIFService {
  "@id": string;
  "@type": "ImageService3";
  profile: "level0" | "level1" | "level2";
  protocol?: string;
}

/**
 * IIIF Presentation Manifest
 */
export interface IIIFManifest {
  "@context": string;
  id: string;
  type: "Manifest";
  wiaStandard: "WIA-EDU-024";
  label: LanguageMap;
  metadata?: MetadataItem[];
  summary?: LanguageMap;
  thumbnail?: ImageObject[];
  items: Canvas[];
  rights?: string;
  requiredStatement?: LabelValuePair;
  seeAlso?: any[];
  rendering?: any[];
}

/**
 * IIIF Canvas
 */
export interface Canvas {
  id: string;
  type: "Canvas";
  label?: LanguageMap;
  width: number;
  height: number;
  items: AnnotationPage[];
}

/**
 * IIIF Annotation Page
 */
export interface AnnotationPage {
  id: string;
  type: "AnnotationPage";
  items: Annotation[];
}

/**
 * IIIF Annotation
 */
export interface Annotation {
  id: string;
  type: "Annotation";
  motivation: string;
  body: any;
  target: string;
}

/**
 * Language map for multilingual content
 */
export interface LanguageMap {
  [languageCode: string]: string[];
}

/**
 * Metadata item
 */
export interface MetadataItem {
  label: LanguageMap;
  value: LanguageMap;
}

/**
 * Label-value pair
 */
export interface LabelValuePair {
  label: LanguageMap;
  value: LanguageMap;
}

/**
 * Condition report for conservation
 */
export interface ConditionReport {
  date: string;
  condition: "excellent" | "good" | "fair" | "poor" | "critical";
  examiner?: Person;
  summary?: string;
  conservationHistory?: ConservationTreatment[];
  nextExamination?: string;
}

/**
 * Conservation treatment record
 */
export interface ConservationTreatment {
  date: string;
  treatment: string;
  conservator: string;
  materials?: string[];
  notes?: string;
  beforeImage?: string;
  afterImage?: string;
}

/**
 * Provenance entry
 */
export interface ProvenanceEntry {
  date?: string;
  dateRange?: string;
  owner?: string;
  location?: string;
  acquisitionMethod?: string;
  price?: MonetaryAmount;
  notes?: string;
  source?: string;
}

/**
 * Monetary amount
 */
export interface MonetaryAmount {
  "@type"?: "MonetaryAmount";
  value: number;
  currency: string;
}

/**
 * Exhibition event
 */
export interface Exhibition {
  "@type"?: "ExhibitionEvent";
  wiaStandard?: "WIA-EDU-024";
  identifier?: string;
  name: string;
  description?: string;
  curator?: Person | Person[];
  startDate?: string;
  endDate?: string;
  location?: Place | Organization;
  virtualLocation?: VirtualLocation;
  workFeatured?: MuseumObject[] | string[];
  numberOfObjects?: number;
  sections?: ExhibitionSection[];
  educationalResources?: EducationalResource[];
  image?: string;
  eventStatus?: "EventScheduled" | "EventPostponed" | "EventCancelled";
  eventType?: "permanent" | "temporary" | "virtual" | "traveling";
  createdAt?: string;
}

/**
 * Virtual location for online exhibitions
 */
export interface VirtualLocation {
  "@type"?: "VirtualLocation";
  url: string;
  description?: string;
}

/**
 * Exhibition section
 */
export interface ExhibitionSection {
  name: string;
  description?: string;
  objects: string[];
  narrativeText?: string;
  order?: number;
}

/**
 * Educational resource
 */
export interface EducationalResource {
  "@type"?: "Course" | "LearningResource";
  name: string;
  description?: string;
  educationalLevel?: string;
  learningResourceType?: string;
  url?: string;
  author?: Person;
  datePublished?: string;
}

/**
 * Related link
 */
export interface RelatedLink {
  "@type"?: "URL";
  name: string;
  url: string;
  description?: string;
}

/**
 * Inscription on artwork
 */
export interface Inscription {
  type: "signature" | "date" | "dedication" | "other";
  location: string;
  content: string;
  language?: string;
}

/**
 * Search query parameters
 */
export interface SearchQuery {
  query?: string;
  filters?: SearchFilters;
  facets?: string[];
  sort?: SortOptions;
  pagination?: PaginationOptions;
  includeFields?: string[];
  searchAlgorithm?: "keyword" | "semantic" | "ai-powered";
}

/**
 * Search filters
 */
export interface SearchFilters {
  department?: string | string[];
  medium?: string | string[];
  creator?: string | string[];
  dateRange?: DateRange;
  culture?: string | string[];
  period?: string | string[];
  onView?: boolean;
  hasImage?: boolean;
  isPublicDomain?: boolean;
  keywords?: string[];
}

/**
 * Date range
 */
export interface DateRange {
  start?: string;
  end?: string;
}

/**
 * Sort options
 */
export interface SortOptions {
  field: "relevance" | "name" | "creator" | "date" | "accession";
  order: "asc" | "desc";
}

/**
 * Pagination options
 */
export interface PaginationOptions {
  page: number;
  pageSize: number;
}

/**
 * Search results
 */
export interface SearchResults {
  query?: SearchQuery;
  results: MuseumObject[];
  facets?: SearchFacets;
  pagination: PaginationInfo;
  totalResults: number;
}

/**
 * Search facets
 */
export interface SearchFacets {
  [facetName: string]: {
    [value: string]: number;
  };
}

/**
 * Pagination information
 */
export interface PaginationInfo {
  page: number;
  pageSize: number;
  totalPages: number;
  totalResults: number;
}

/**
 * API Response wrapper
 */
export interface APIResponse<T> {
  data: T;
  pagination?: PaginationInfo;
  links?: {
    self?: string;
    next?: string;
    prev?: string;
    first?: string;
    last?: string;
  };
  meta?: {
    timestamp?: string;
    requestId?: string;
  };
}

/**
 * API Error response
 */
export interface APIError {
  error: {
    code: string;
    message: string;
    details?: any;
    requestId?: string;
  };
}

/**
 * Client configuration options
 */
export interface ClientConfig {
  baseURL: string;
  apiKey?: string;
  accessToken?: string;
  timeout?: number;
  retries?: number;
  rateLimit?: {
    maxRequests: number;
    perMilliseconds: number;
  };
}

/**
 * Department information
 */
export interface Department {
  id: string;
  name: string;
  description?: string;
  objectCount?: number;
  url?: string;
}

/**
 * Collection statistics
 */
export interface CollectionStats {
  totalObjects: number;
  objectsWithImages: number;
  departments: number;
  creators: number;
  exhibitions: number;
  dateRange?: {
    earliest: string;
    latest: string;
  };
  topDepartments?: Array<{
    name: string;
    count: number;
  }>;
  recentlyAdded?: number;
  lastUpdated?: string;
}

/**
 * Webhook event types
 */
export type WebhookEventType =
  | "object.created"
  | "object.updated"
  | "object.deleted"
  | "exhibition.created"
  | "exhibition.published"
  | "image.uploaded";

/**
 * Webhook payload
 */
export interface WebhookPayload {
  event: WebhookEventType;
  timestamp: string;
  data: any;
  signature?: string;
}

/**
 * Webhook registration
 */
export interface WebhookRegistration {
  url: string;
  events: WebhookEventType[];
  secret?: string;
  active?: boolean;
}

/**
 * Export format types
 */
export type ExportFormat = "json" | "json-ld" | "xml" | "csv" | "rdf";

/**
 * Batch operation result
 */
export interface BatchResult {
  total: number;
  successful: number;
  failed: number;
  errors?: Array<{
    id: string;
    error: string;
  }>;
}
