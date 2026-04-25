# PHASE 2: Algorithm Specification

## WIA-PET-LEGACY Processing Algorithms

> "Hong-ik Ingan: Thoughtful algorithms transform memories into lasting tributes."

---

## 1. Memory Curation

### 1.1 Photo Selection Algorithm

```python
class MemoryPhotoSelector:
    """Selects best photos for memorial gallery."""

    def __init__(self, config: SelectionConfig):
        self.config = config
        self.quality_analyzer = ImageQualityAnalyzer()
        self.pet_detector = PetDetector()
        self.emotion_analyzer = EmotionAnalyzer()

    def select_featured_photos(
        self,
        photos: list[MediaAttachment],
        count: int = 10
    ) -> list[ScoredPhoto]:
        """Select top photos for featured gallery."""
        scored_photos = []

        for photo in photos:
            score = self._calculate_photo_score(photo)
            scored_photos.append(ScoredPhoto(photo, score))

        # Sort by score
        scored_photos.sort(key=lambda x: x.score, reverse=True)

        # Ensure diversity
        selected = self._ensure_diversity(scored_photos, count)

        return selected

    def _calculate_photo_score(
        self,
        photo: MediaAttachment
    ) -> float:
        """Calculate memorial suitability score."""
        score = 0.0

        # Technical quality (0-25)
        quality = self.quality_analyzer.analyze(photo)
        score += quality.sharpness * 10
        score += quality.exposure * 10
        score += quality.composition * 5

        # Pet presence and clarity (0-30)
        pet_analysis = self.pet_detector.analyze(photo)
        if pet_analysis.pet_detected:
            score += 15
            score += pet_analysis.face_visible * 10
            score += pet_analysis.eye_contact * 5

        # Emotional content (0-25)
        emotion = self.emotion_analyzer.analyze(photo)
        positive_emotions = ['happy', 'playful', 'content', 'loving']
        if emotion.detected_emotion in positive_emotions:
            score += 20
        score += emotion.warmth * 5

        # Context value (0-20)
        if photo.caption:
            score += 5
        if photo.event_id:
            score += 5
        if photo.tags:
            score += min(len(photo.tags), 5) * 2

        return score / 100

    def _ensure_diversity(
        self,
        photos: list[ScoredPhoto],
        count: int
    ) -> list[ScoredPhoto]:
        """Ensure selected photos are diverse."""
        selected = []
        seen_dates = set()
        seen_locations = set()

        for photo in photos:
            if len(selected) >= count:
                break

            # Check diversity criteria
            date_key = photo.capture_date.strftime("%Y-%m")

            # Allow some from same month, but prioritize variety
            same_month_count = sum(
                1 for p in selected
                if p.capture_date.strftime("%Y-%m") == date_key
            )

            if same_month_count < 3:
                selected.append(photo)
                seen_dates.add(date_key)

        return selected
```

### 1.2 Timeline Generation

```python
class TimelineGenerator:
    """Generates life timeline from events and media."""

    def generate_timeline(
        self,
        legacy: LegacyProfile
    ) -> Timeline:
        """Generate comprehensive life timeline."""
        events = []

        # Add biographical milestones
        if legacy.pet_info.birth_date:
            events.append(self._create_birth_event(legacy))

        if legacy.pet_info.adoption_date:
            events.append(self._create_adoption_event(legacy))

        # Add user-created events
        events.extend(legacy.life_timeline)

        # Infer events from media
        inferred = self._infer_events_from_media(legacy.media)
        events.extend(inferred)

        # Add passing event if applicable
        if legacy.pet_info.passing_date:
            events.append(self._create_passing_event(legacy))

        # Sort chronologically
        events.sort(key=lambda e: e.date)

        # Group by period
        periods = self._group_into_periods(events)

        return Timeline(
            legacy_id=legacy.legacy_id,
            pet_name=legacy.pet_info.name,
            start_date=events[0].date if events else None,
            end_date=events[-1].date if events else None,
            periods=periods,
            event_count=len(events)
        )

    def _infer_events_from_media(
        self,
        media: list[MediaAttachment]
    ) -> list[LifeEvent]:
        """Infer events from media metadata."""
        inferred_events = []

        # Group photos by date and location
        date_groups = self._group_media_by_date(media)

        for date, group in date_groups.items():
            if len(group) >= 3:  # Enough for a meaningful event
                # Analyze common themes
                themes = self._analyze_themes(group)

                if themes:
                    event = LifeEvent(
                        event_type=LifeEventType.MEMORABLE_MOMENT,
                        category=EventCategory.DAILY_LIFE,
                        title=f"Memories from {date.strftime('%B %Y')}",
                        date=date,
                        media=group[:5],  # Top 5 from group
                        significance=2,
                        auto_generated=True
                    )
                    inferred_events.append(event)

        return inferred_events

    def _group_into_periods(
        self,
        events: list[LifeEvent]
    ) -> list[TimelinePeriod]:
        """Group events into life periods."""
        if not events:
            return []

        periods = []
        current_period = None

        for event in events:
            period_name = self._determine_period(event)

            if current_period is None or current_period.name != period_name:
                if current_period:
                    periods.append(current_period)
                current_period = TimelinePeriod(
                    name=period_name,
                    start_date=event.date,
                    events=[]
                )

            current_period.events.append(event)
            current_period.end_date = event.date

        if current_period:
            periods.append(current_period)

        return periods
```

---

## 2. Memorial Generation

### 2.1 Biography Generator

```python
class BiographyGenerator:
    """Generates narrative biography from structured data."""

    def generate_biography(
        self,
        legacy: LegacyProfile
    ) -> Biography:
        """Generate complete biography narrative."""
        pet = legacy.pet_info
        events = legacy.life_timeline

        sections = []

        # Introduction
        sections.append(self._generate_introduction(pet))

        # Origin story
        if pet.origin:
            sections.append(self._generate_origin_section(pet))

        # Personality section
        sections.append(self._generate_personality_section(pet))

        # Life highlights
        highlights = self._select_highlights(events)
        if highlights:
            sections.append(self._generate_highlights_section(highlights))

        # Favorites section
        sections.append(self._generate_favorites_section(pet.favorites))

        # Closing/remembrance
        sections.append(self._generate_closing(pet))

        return Biography(
            legacy_id=legacy.legacy_id,
            title=f"Remembering {pet.name}",
            sections=sections,
            word_count=sum(len(s.content.split()) for s in sections)
        )

    def _generate_introduction(
        self,
        pet: PetBiography
    ) -> BiographySection:
        """Generate introduction paragraph."""
        age_text = self._format_age(pet)
        traits = ", ".join(pet.personality_traits[:3])

        intro = f"{pet.name} was a {traits} {pet.breed} "
        intro += f"who brought joy to everyone who knew them. "

        if pet.birth_date and pet.passing_date:
            intro += f"Born on {pet.birth_date.strftime('%B %d, %Y')}, "
            intro += f"{pet.name} lived a full life of {age_text}."

        return BiographySection(
            title="Introduction",
            content=intro,
            section_type="introduction"
        )

    def _generate_personality_section(
        self,
        pet: PetBiography
    ) -> BiographySection:
        """Generate personality description."""
        content = f"{pet.name}'s personality was truly special. "

        if pet.temperament:
            content += f"Known for being {pet.temperament}, "

        if pet.personality_traits:
            traits_text = self._format_trait_list(pet.personality_traits)
            content += f"{pet.name} was {traits_text}. "

        if pet.distinctive_features:
            features = pet.distinctive_features[0]
            content += f"One of their most distinctive features was {features}."

        return BiographySection(
            title="Personality",
            content=content,
            section_type="personality"
        )
```

### 2.2 Memorial Page Generator

```python
class MemorialPageGenerator:
    """Generates memorial page HTML content."""

    def generate_memorial_page(
        self,
        legacy: LegacyProfile,
        config: MemorialConfiguration
    ) -> MemorialPage:
        """Generate complete memorial page."""
        sections = []

        for section_config in config.sections:
            if not section_config.enabled:
                continue

            section = self._generate_section(
                section_config,
                legacy,
                config.theme
            )
            sections.append(section)

        # Generate navigation
        navigation = self._generate_navigation(sections)

        # Apply theme
        styled_content = self._apply_theme(sections, config.theme)

        return MemorialPage(
            legacy_id=legacy.legacy_id,
            title=f"In Loving Memory of {legacy.pet_info.name}",
            sections=styled_content,
            navigation=navigation,
            meta=self._generate_meta(legacy)
        )

    def _generate_section(
        self,
        config: MemorialSection,
        legacy: LegacyProfile,
        theme: MemorialTheme
    ) -> GeneratedSection:
        """Generate individual memorial section."""
        generators = {
            SectionType.HERO: self._generate_hero_section,
            SectionType.BIOGRAPHY: self._generate_biography_section,
            SectionType.TIMELINE: self._generate_timeline_section,
            SectionType.GALLERY: self._generate_gallery_section,
            SectionType.TRIBUTES: self._generate_tributes_section,
            SectionType.FAVORITES: self._generate_favorites_section,
            SectionType.CANDLES: self._generate_candles_section
        }

        generator = generators.get(config.section_type)
        if generator:
            return generator(config, legacy, theme)

        return GeneratedSection(
            section_id=config.section_id,
            content=""
        )

    def _generate_hero_section(
        self,
        config: MemorialSection,
        legacy: LegacyProfile,
        theme: MemorialTheme
    ) -> GeneratedSection:
        """Generate hero tribute section."""
        pet = legacy.pet_info

        # Select best hero image
        hero_image = self._select_hero_image(legacy.media)

        content = {
            "image": hero_image,
            "name": pet.name,
            "dates": self._format_life_dates(pet),
            "epitaph": pet.epitaph or f"Forever in our hearts"
        }

        return GeneratedSection(
            section_id=config.section_id,
            section_type=SectionType.HERO,
            content=content
        )
```

---

## 3. Media Processing

### 3.1 Media Archive Processor

```python
class MediaArchiveProcessor:
    """Processes and archives media for long-term storage."""

    def process_for_archive(
        self,
        media: MediaAttachment
    ) -> ArchiveResult:
        """Process media for archival storage."""
        # Generate preservation copies
        preservation = self._create_preservation_copy(media)

        # Generate thumbnails
        thumbnails = self._generate_thumbnails(media)

        # Extract metadata
        metadata = self._extract_metadata(media)

        # Calculate checksums
        checksums = self._calculate_checksums(
            media,
            preservation,
            thumbnails
        )

        # Perform AI analysis
        ai_analysis = self._perform_ai_analysis(media)

        return ArchiveResult(
            original=media,
            preservation=preservation,
            thumbnails=thumbnails,
            metadata=metadata,
            checksums=checksums,
            ai_analysis=ai_analysis
        )

    def _create_preservation_copy(
        self,
        media: MediaAttachment
    ) -> PreservationCopy:
        """Create archival-quality preservation copy."""
        if media.media_type == MediaType.PHOTO:
            # Convert to TIFF for preservation
            preserved = self._convert_to_tiff(media)

            # Also keep high-quality JPEG
            display = self._create_high_quality_jpeg(media)

        elif media.media_type == MediaType.VIDEO:
            # Transcode to preservation format
            preserved = self._transcode_video(
                media,
                format="FFV1",
                container="MKV"
            )

            # Create H.265 for streaming
            display = self._transcode_video(
                media,
                format="H265",
                container="MP4"
            )

        return PreservationCopy(
            preservation_file=preserved,
            display_file=display,
            format_info=self._get_format_info(preserved)
        )

    def _perform_ai_analysis(
        self,
        media: MediaAttachment
    ) -> AIAnalysis:
        """Perform AI analysis on media."""
        analysis = AIAnalysis()

        if media.media_type == MediaType.PHOTO:
            # Detect pets
            pet_detection = self.pet_detector.detect(media)
            analysis.pet_detected = pet_detection.detected
            analysis.pet_species = pet_detection.species
            analysis.pet_confidence = pet_detection.confidence

            # Detect emotions
            if pet_detection.detected:
                emotion = self.emotion_detector.detect(media)
                analysis.emotion = emotion.primary_emotion

            # Scene understanding
            scene = self.scene_analyzer.analyze(media)
            analysis.scene_type = scene.scene_type
            analysis.location_type = scene.location

            # Generate tags
            analysis.auto_tags = self._generate_tags(
                pet_detection,
                emotion,
                scene
            )

        return analysis
```

---

## 4. Memory Book Generation

### 4.1 Book Layout Engine

```python
class MemoryBookGenerator:
    """Generates printable memory book."""

    def generate_book(
        self,
        legacy: LegacyProfile,
        book_config: MemoryBookConfig
    ) -> MemoryBook:
        """Generate complete memory book."""
        pages = []

        # Cover page
        pages.append(self._generate_cover(legacy, book_config))

        # Title page
        pages.append(self._generate_title_page(legacy))

        # Dedication
        if book_config.include_dedication:
            pages.append(self._generate_dedication(legacy))

        # Biography pages
        bio_pages = self._generate_biography_pages(legacy)
        pages.extend(bio_pages)

        # Timeline pages
        timeline_pages = self._generate_timeline_pages(legacy)
        pages.extend(timeline_pages)

        # Photo pages
        photo_pages = self._generate_photo_pages(
            legacy,
            book_config.max_photo_pages
        )
        pages.extend(photo_pages)

        # Tribute pages
        if book_config.include_tributes:
            tribute_pages = self._generate_tribute_pages(legacy)
            pages.extend(tribute_pages)

        # Closing page
        pages.append(self._generate_closing_page(legacy))

        # Generate PDF
        pdf = self._render_to_pdf(pages, book_config)

        return MemoryBook(
            book_id=generate_uuid(),
            legacy_id=legacy.legacy_id,
            pages=pages,
            page_count=len(pages),
            pdf_url=pdf.url,
            generated_at=datetime.now()
        )

    def _generate_photo_pages(
        self,
        legacy: LegacyProfile,
        max_pages: int
    ) -> list[BookPage]:
        """Generate photo gallery pages."""
        # Select best photos
        selector = MemoryPhotoSelector()
        photos = selector.select_featured_photos(
            legacy.media,
            count=max_pages * 4  # 4 photos per page average
        )

        pages = []
        current_page_photos = []

        for photo in photos:
            current_page_photos.append(photo)

            if len(current_page_photos) >= 4:
                layout = self._select_layout(current_page_photos)
                page = BookPage(
                    page_type=PageType.PHOTO_PAGE,
                    layout=layout,
                    content={"photos": current_page_photos}
                )
                pages.append(page)
                current_page_photos = []

        # Handle remaining photos
        if current_page_photos:
            layout = self._select_layout(current_page_photos)
            pages.append(BookPage(
                page_type=PageType.PHOTO_PAGE,
                layout=layout,
                content={"photos": current_page_photos}
            ))

        return pages[:max_pages]
```

---

## 5. Grief Support Integration

### 5.1 Support Resource Matcher

```python
class GriefSupportMatcher:
    """Matches users with appropriate grief support resources."""

    def find_resources(
        self,
        legacy: LegacyProfile,
        user_context: UserContext
    ) -> list[SupportResource]:
        """Find relevant grief support resources."""
        resources = []

        # Determine stage
        stage = self._determine_grief_stage(legacy, user_context)

        # Find stage-appropriate resources
        stage_resources = self._find_stage_resources(stage)
        resources.extend(stage_resources)

        # Local resources
        if user_context.location:
            local = self._find_local_resources(user_context.location)
            resources.extend(local)

        # Pet-specific resources
        pet_specific = self._find_pet_specific_resources(
            legacy.pet_info.species
        )
        resources.extend(pet_specific)

        # Rank by relevance
        ranked = self._rank_resources(resources, user_context)

        return ranked[:10]

    def _determine_grief_stage(
        self,
        legacy: LegacyProfile,
        context: UserContext
    ) -> GriefStage:
        """Estimate grief stage based on timing and activity."""
        if not legacy.pet_info.passing_date:
            return GriefStage.ANTICIPATORY

        days_since = (datetime.now() - legacy.pet_info.passing_date).days

        if days_since < 7:
            return GriefStage.ACUTE
        elif days_since < 30:
            return GriefStage.EARLY
        elif days_since < 180:
            return GriefStage.INTEGRATED
        else:
            return GriefStage.LONG_TERM
```

---

## Document Information

**WIA-PET-LEGACY Phase 2: Algorithms**
**Version**: 1.0.0
**Hong-ik Ingan**
