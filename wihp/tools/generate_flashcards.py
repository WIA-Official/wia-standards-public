#!/usr/bin/env python3
"""
WIHP Flashcard Generator
========================
Generates learning flashcards from WIHP language mapping files.

Output Formats:
- Anki: Tab-separated file for Anki import
- Quizlet: Format compatible with Quizlet import
- CSV: Standard CSV format
- JSON: Structured JSON for custom apps
- HTML: Interactive HTML flashcard deck

Card Types:
- IPA → Hangul (pronunciation cards)
- Hangul → Original (reading cards)
- Sentence cards (full sentences)
- Vocabulary cards (words with context)

Usage:
    python generate_flashcards.py [--language LANG] [--format FORMAT] [--type TYPE]

Examples:
    python generate_flashcards.py --language korean --format anki
    python generate_flashcards.py --tier 1 --format quizlet --type sentences
    python generate_flashcards.py --all --format html
"""

import os
import re
import sys
import json
import csv
import argparse
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, asdict
from collections import defaultdict

@dataclass
class Flashcard:
    """Represents a single flashcard."""
    front: str
    back: str
    hint: Optional[str] = None
    tags: List[str] = None
    language: str = ""
    card_type: str = ""

    def __post_init__(self):
        if self.tags is None:
            self.tags = []


class FlashcardGenerator:
    """Generates flashcards from WIHP mapping files."""

    def __init__(self, wihp_root: Path):
        self.wihp_root = wihp_root
        self.cards: List[Flashcard] = []

    def extract_language_info(self, content: str) -> Dict:
        """Extract language metadata from file content."""
        info = {}

        # Extract language name
        lang_match = re.search(r'\*\*Language\*\*:\s*([^\n]+)', content)
        if lang_match:
            info['language'] = lang_match.group(1).strip()

        # Extract ISO code
        iso_match = re.search(r'\*\*ISO 639-3\*\*:\s*(\w+)', content)
        if iso_match:
            info['iso'] = iso_match.group(1).strip()

        # Extract tier
        tier_match = re.search(r'\*\*WIHP Tier\*\*:\s*(\d+)', content)
        if tier_match:
            info['tier'] = int(tier_match.group(1))

        return info

    def extract_phoneme_cards(self, content: str, language: str) -> List[Flashcard]:
        """Extract IPA → Hangul phoneme cards."""
        cards = []

        # Pattern for consonant/vowel table rows
        table_pattern = re.compile(
            r'\|\s*([^|]+)\s*\|\s*(?:PL|VL)\d{2}-(?:MN|VH)\d{2}-(?:AR|VR)\d{2}\s*\|\s*([^|]+)\s*\|\s*([^|]+)\s*\|'
        )

        for match in table_pattern.finditer(content):
            ipa = match.group(1).strip()
            hangul = match.group(2).strip()
            description = match.group(3).strip()

            if ipa and hangul and ipa != 'IPA':  # Skip header rows
                cards.append(Flashcard(
                    front=f"/{ipa}/",
                    back=hangul,
                    hint=description,
                    tags=[language, 'phoneme', 'ipa-to-hangul'],
                    language=language,
                    card_type='phoneme'
                ))

                # Reverse card: Hangul → IPA
                cards.append(Flashcard(
                    front=hangul,
                    back=f"/{ipa}/ - {description}",
                    tags=[language, 'phoneme', 'hangul-to-ipa'],
                    language=language,
                    card_type='phoneme-reverse'
                ))

        return cards

    def extract_sentence_cards(self, content: str, language: str) -> List[Flashcard]:
        """Extract sentence cards with translations."""
        cards = []

        # Pattern: **Text** /IPA/ → Hangul "Translation"
        sentence_pattern = re.compile(
            r'\*\*([^*]+)\*\*\s*/([^/]+)/\s*→\s*([^\s"]+(?:\s+[^\s"]+)*)\s*"([^"]+)"'
        )

        for match in sentence_pattern.finditer(content):
            original = match.group(1).strip()
            ipa = match.group(2).strip()
            hangul = match.group(3).strip()
            translation = match.group(4).strip()

            # Card 1: Original → Translation
            cards.append(Flashcard(
                front=original,
                back=f"{translation}\n\n/{ipa}/ → {hangul}",
                tags=[language, 'sentence', 'original-to-translation'],
                language=language,
                card_type='sentence'
            ))

            # Card 2: Translation → Original
            cards.append(Flashcard(
                front=f"{translation} ({language})",
                back=f"{original}\n/{ipa}/ → {hangul}",
                tags=[language, 'sentence', 'translation-to-original'],
                language=language,
                card_type='sentence-reverse'
            ))

            # Card 3: Hangul reading practice
            cards.append(Flashcard(
                front=hangul,
                back=f"/{ipa}/\n\n{original} = {translation}",
                tags=[language, 'sentence', 'hangul-reading'],
                language=language,
                card_type='reading'
            ))

        return cards

    def extract_vocabulary_cards(self, content: str, language: str) -> List[Flashcard]:
        """Extract vocabulary cards (numbers, days, colors, family)."""
        cards = []

        # Pattern for vocabulary tables: | Word | IPA | Hangul |
        vocab_pattern = re.compile(
            r'\|\s*([^|]+)\s*\|\s*/([^/]+)/\s*\|\s*([^|]+)\s*\|'
        )

        # Also match tables with meaning: | Word | IPA | Hangul | Meaning |
        vocab_meaning_pattern = re.compile(
            r'\|\s*([^|]+)\s*\|\s*/([^/]+)/\s*\|\s*([^|]+)\s*\|\s*([^|]+)\s*\|'
        )

        for match in vocab_meaning_pattern.finditer(content):
            word = match.group(1).strip()
            ipa = match.group(2).strip()
            hangul = match.group(3).strip()
            meaning = match.group(4).strip()

            # Skip header rows
            if word.lower() in ['word', 'term', language.lower(), 'ipa', '---']:
                continue

            cards.append(Flashcard(
                front=word,
                back=f"{meaning}\n\n/{ipa}/ → {hangul}",
                tags=[language, 'vocabulary'],
                language=language,
                card_type='vocabulary'
            ))

        return cards

    def process_file(self, filepath: Path) -> List[Flashcard]:
        """Process a single mapping file and generate all card types."""
        try:
            content = filepath.read_text(encoding='utf-8')
        except Exception as e:
            print(f"⚠️  Error reading {filepath}: {e}")
            return []

        info = self.extract_language_info(content)
        language = info.get('language', filepath.stem)

        cards = []
        cards.extend(self.extract_phoneme_cards(content, language))
        cards.extend(self.extract_sentence_cards(content, language))
        cards.extend(self.extract_vocabulary_cards(content, language))

        return cards

    def process_tier(self, tier: int) -> List[Flashcard]:
        """Process all files in a tier."""
        tier_path = self.wihp_root / 'wihp' / f'tier{tier}'
        cards = []

        if not tier_path.exists():
            return cards

        for filepath in sorted(tier_path.glob('*.md')):
            if filepath.name != 'README.md':
                file_cards = self.process_file(filepath)
                cards.extend(file_cards)
                print(f"   📝 {filepath.name}: {len(file_cards)} cards")

        return cards

    def process_all(self) -> List[Flashcard]:
        """Process all tiers."""
        all_cards = []

        for tier in range(1, 5):
            print(f"\n📁 Processing Tier {tier}...")
            tier_cards = self.process_tier(tier)
            all_cards.extend(tier_cards)
            print(f"   Total: {len(tier_cards)} cards from Tier {tier}")

        return all_cards


class FlashcardExporter:
    """Exports flashcards to various formats."""

    def __init__(self, cards: List[Flashcard]):
        self.cards = cards

    def to_anki(self, output_path: Path, deck_name: str = "WIHP"):
        """Export to Anki tab-separated format."""
        with open(output_path, 'w', encoding='utf-8') as f:
            # Anki header
            f.write(f"#separator:tab\n")
            f.write(f"#html:true\n")
            f.write(f"#deck:{deck_name}\n")
            f.write(f"#tags column:3\n\n")

            for card in self.cards:
                front = card.front.replace('\n', '<br>')
                back = card.back.replace('\n', '<br>')
                tags = ' '.join(card.tags)
                f.write(f"{front}\t{back}\t{tags}\n")

        print(f"📄 Anki deck saved: {output_path}")

    def to_quizlet(self, output_path: Path):
        """Export to Quizlet import format."""
        with open(output_path, 'w', encoding='utf-8') as f:
            for card in self.cards:
                # Quizlet uses tab separation and newlines between cards
                front = card.front.replace('\n', ' | ')
                back = card.back.replace('\n', ' | ')
                f.write(f"{front}\t{back}\n")

        print(f"📄 Quizlet file saved: {output_path}")

    def to_csv(self, output_path: Path):
        """Export to CSV format."""
        with open(output_path, 'w', encoding='utf-8', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Front', 'Back', 'Hint', 'Language', 'Type', 'Tags'])

            for card in self.cards:
                writer.writerow([
                    card.front,
                    card.back,
                    card.hint or '',
                    card.language,
                    card.card_type,
                    ';'.join(card.tags)
                ])

        print(f"📄 CSV file saved: {output_path}")

    def to_json(self, output_path: Path):
        """Export to JSON format."""
        data = {
            'generated': datetime.now().isoformat(),
            'total_cards': len(self.cards),
            'cards': [asdict(card) for card in self.cards]
        }

        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

        print(f"📄 JSON file saved: {output_path}")

    def to_html(self, output_path: Path):
        """Export to interactive HTML flashcard deck."""
        html_template = '''<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>WIHP Flashcards</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: 'Noto Sans KR', -apple-system, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            min-height: 100vh;
            display: flex;
            flex-direction: column;
            align-items: center;
            padding: 20px;
            color: white;
        }
        h1 { margin-bottom: 20px; }
        .stats { margin-bottom: 20px; opacity: 0.8; }
        .controls { margin-bottom: 20px; display: flex; gap: 10px; flex-wrap: wrap; justify-content: center; }
        select, button {
            padding: 10px 20px;
            border: none;
            border-radius: 8px;
            font-size: 16px;
            cursor: pointer;
        }
        select { background: #2a2a4a; color: white; }
        button { background: #4a4a8a; color: white; transition: all 0.3s; }
        button:hover { background: #6a6aaa; transform: scale(1.05); }
        .card-container {
            perspective: 1000px;
            width: 100%;
            max-width: 600px;
            height: 350px;
        }
        .card {
            width: 100%;
            height: 100%;
            position: relative;
            transform-style: preserve-3d;
            transition: transform 0.6s;
            cursor: pointer;
        }
        .card.flipped { transform: rotateY(180deg); }
        .card-face {
            position: absolute;
            width: 100%;
            height: 100%;
            backface-visibility: hidden;
            display: flex;
            flex-direction: column;
            justify-content: center;
            align-items: center;
            padding: 30px;
            border-radius: 20px;
            text-align: center;
        }
        .card-front { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); }
        .card-back {
            background: linear-gradient(135deg, #11998e 0%, #38ef7d 100%);
            transform: rotateY(180deg);
        }
        .card-content { font-size: 28px; line-height: 1.6; word-break: keep-all; }
        .card-hint { font-size: 14px; opacity: 0.7; margin-top: 15px; }
        .card-meta { font-size: 12px; opacity: 0.5; position: absolute; bottom: 15px; }
        .nav-buttons { display: flex; gap: 20px; margin-top: 20px; }
        .nav-buttons button { font-size: 24px; padding: 15px 30px; }
        .progress { width: 100%; max-width: 600px; margin-top: 20px; }
        .progress-bar {
            height: 6px;
            background: rgba(255,255,255,0.2);
            border-radius: 3px;
            overflow: hidden;
        }
        .progress-fill {
            height: 100%;
            background: #4ade80;
            transition: width 0.3s;
        }
        @keyframes slideIn { from { opacity: 0; transform: translateX(50px); } to { opacity: 1; transform: translateX(0); } }
        .card-container { animation: slideIn 0.3s ease-out; }
    </style>
    <link href="https://fonts.googleapis.com/css2?family=Noto+Sans+KR:wght@400;700&display=swap" rel="stylesheet">
</head>
<body>
    <h1>🇰🇷 WIHP Flashcards</h1>
    <div class="stats">
        <span id="current">1</span> / <span id="total">0</span> cards
    </div>
    <div class="controls">
        <select id="languageFilter" onchange="filterCards()">
            <option value="all">All Languages</option>
        </select>
        <select id="typeFilter" onchange="filterCards()">
            <option value="all">All Types</option>
            <option value="phoneme">Phonemes</option>
            <option value="sentence">Sentences</option>
            <option value="vocabulary">Vocabulary</option>
        </select>
        <button onclick="shuffle()">🔀 Shuffle</button>
    </div>

    <div class="card-container">
        <div class="card" id="flashcard" onclick="flipCard()">
            <div class="card-face card-front">
                <div class="card-content" id="front"></div>
                <div class="card-hint" id="hint"></div>
                <div class="card-meta" id="meta-front"></div>
            </div>
            <div class="card-face card-back">
                <div class="card-content" id="back"></div>
                <div class="card-meta" id="meta-back"></div>
            </div>
        </div>
    </div>

    <div class="nav-buttons">
        <button onclick="prevCard()">⬅️</button>
        <button onclick="nextCard()">➡️</button>
    </div>

    <div class="progress">
        <div class="progress-bar">
            <div class="progress-fill" id="progress"></div>
        </div>
    </div>

    <script>
        const allCards = CARDS_DATA;
        let cards = [...allCards];
        let currentIndex = 0;
        let isFlipped = false;

        // Extract unique languages
        const languages = [...new Set(allCards.map(c => c.language))].sort();
        const langSelect = document.getElementById('languageFilter');
        languages.forEach(lang => {
            const opt = document.createElement('option');
            opt.value = lang;
            opt.textContent = lang;
            langSelect.appendChild(opt);
        });

        function showCard() {
            const card = cards[currentIndex];
            document.getElementById('front').innerHTML = card.front.replace(/\\n/g, '<br>');
            document.getElementById('back').innerHTML = card.back.replace(/\\n/g, '<br>');
            document.getElementById('hint').textContent = card.hint || '';
            document.getElementById('meta-front').textContent = `${card.language} • ${card.card_type}`;
            document.getElementById('meta-back').textContent = card.tags.join(' • ');
            document.getElementById('current').textContent = currentIndex + 1;
            document.getElementById('total').textContent = cards.length;
            document.getElementById('progress').style.width = `${((currentIndex + 1) / cards.length) * 100}%`;

            // Reset flip
            document.getElementById('flashcard').classList.remove('flipped');
            isFlipped = false;
        }

        function flipCard() {
            document.getElementById('flashcard').classList.toggle('flipped');
            isFlipped = !isFlipped;
        }

        function nextCard() {
            currentIndex = (currentIndex + 1) % cards.length;
            showCard();
        }

        function prevCard() {
            currentIndex = (currentIndex - 1 + cards.length) % cards.length;
            showCard();
        }

        function shuffle() {
            for (let i = cards.length - 1; i > 0; i--) {
                const j = Math.floor(Math.random() * (i + 1));
                [cards[i], cards[j]] = [cards[j], cards[i]];
            }
            currentIndex = 0;
            showCard();
        }

        function filterCards() {
            const lang = document.getElementById('languageFilter').value;
            const type = document.getElementById('typeFilter').value;

            cards = allCards.filter(c => {
                const langMatch = lang === 'all' || c.language === lang;
                const typeMatch = type === 'all' || c.card_type.includes(type);
                return langMatch && typeMatch;
            });

            currentIndex = 0;
            if (cards.length > 0) showCard();
        }

        // Keyboard navigation
        document.addEventListener('keydown', (e) => {
            if (e.key === 'ArrowRight') nextCard();
            if (e.key === 'ArrowLeft') prevCard();
            if (e.key === ' ') { e.preventDefault(); flipCard(); }
        });

        // Initial display
        showCard();
    </script>
</body>
</html>'''

        # Convert cards to JSON for embedding
        cards_json = json.dumps([asdict(c) for c in self.cards], ensure_ascii=False)
        html_content = html_template.replace('CARDS_DATA', cards_json)

        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(html_content)

        print(f"📄 HTML flashcards saved: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description='WIHP Flashcard Generator',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--tier', type=int, choices=[1, 2, 3, 4],
                        help='Process specific tier only')
    parser.add_argument('--language', type=str,
                        help='Process specific language only')
    parser.add_argument('--format', type=str, default='all',
                        choices=['anki', 'quizlet', 'csv', 'json', 'html', 'all'],
                        help='Output format')
    parser.add_argument('--type', type=str, default='all',
                        choices=['phoneme', 'sentence', 'vocabulary', 'all'],
                        help='Card type to generate')
    parser.add_argument('--output', type=str, default=None,
                        help='Output directory')
    parser.add_argument('--root', type=str, default=None,
                        help='WIHP root directory')

    args = parser.parse_args()

    # Determine paths
    if args.root:
        wihp_root = Path(args.root)
    else:
        script_dir = Path(__file__).parent
        wihp_root = script_dir.parent

    if args.output:
        output_dir = Path(args.output)
    else:
        output_dir = wihp_root / 'tools' / 'output'

    output_dir.mkdir(parents=True, exist_ok=True)

    print("🃏 WIHP Flashcard Generator")
    print("=" * 40)

    # Generate cards
    generator = FlashcardGenerator(wihp_root)

    if args.tier:
        cards = generator.process_tier(args.tier)
    else:
        cards = generator.process_all()

    # Filter by type if specified
    if args.type != 'all':
        cards = [c for c in cards if args.type in c.card_type]

    print(f"\n✅ Generated {len(cards)} flashcards")

    # Export
    exporter = FlashcardExporter(cards)

    formats = [args.format] if args.format != 'all' else ['anki', 'quizlet', 'csv', 'json', 'html']

    print("\n📤 Exporting...")
    for fmt in formats:
        if fmt == 'anki':
            exporter.to_anki(output_dir / 'wihp_flashcards.txt')
        elif fmt == 'quizlet':
            exporter.to_quizlet(output_dir / 'wihp_quizlet.txt')
        elif fmt == 'csv':
            exporter.to_csv(output_dir / 'wihp_flashcards.csv')
        elif fmt == 'json':
            exporter.to_json(output_dir / 'wihp_flashcards.json')
        elif fmt == 'html':
            exporter.to_html(output_dir / 'wihp_flashcards.html')

    print(f"\n✅ All exports complete! Files saved to: {output_dir}")


if __name__ == '__main__':
    main()
