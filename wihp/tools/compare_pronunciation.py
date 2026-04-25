#!/usr/bin/env python3
"""
WIHP Pronunciation Comparison Tool
===================================
Compares how the same words/phrases are pronounced across different languages.

Features:
- Compare common words (greetings, numbers, colors, family terms)
- Show IPA transcriptions side-by-side
- Generate comparison tables (Markdown, HTML, CSV)
- Highlight phonological similarities and differences
- Cross-language cognate detection

Usage:
    python compare_pronunciation.py [--category CATEGORY] [--languages LANG1,LANG2]

Examples:
    python compare_pronunciation.py --category greetings
    python compare_pronunciation.py --category numbers --languages korean,japanese,chinese
    python compare_pronunciation.py --all --format html
"""

import os
import re
import sys
import json
import argparse
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Set
from collections import defaultdict
from dataclasses import dataclass

@dataclass
class PronunciationEntry:
    """A pronunciation entry for a concept in a specific language."""
    language: str
    iso_code: str
    original: str
    ipa: str
    hangul: str
    tier: int


class PronunciationDatabase:
    """Database of pronunciations extracted from WIHP files."""

    def __init__(self, wihp_root: Path):
        self.wihp_root = wihp_root
        self.data: Dict[str, Dict[str, PronunciationEntry]] = defaultdict(dict)
        # Category -> Concept -> Language -> Entry
        self.categories = {
            'greetings': [
                'hello', 'goodbye', 'good morning', 'good evening',
                'how are you', 'thank you', 'please', 'sorry',
                'yes', 'no', 'see you'
            ],
            'numbers': [
                '1', '2', '3', '4', '5', '6', '7', '8', '9', '10'
            ],
            'colors': [
                'white', 'black', 'red', 'green', 'blue', 'yellow'
            ],
            'family': [
                'mother', 'father', 'brother', 'sister',
                'grandmother', 'grandfather', 'wife', 'husband',
                'son', 'daughter'
            ],
            'days': [
                'monday', 'tuesday', 'wednesday', 'thursday',
                'friday', 'saturday', 'sunday'
            ]
        }

    def extract_from_file(self, filepath: Path, tier: int) -> Dict:
        """Extract pronunciation data from a single file."""
        try:
            content = filepath.read_text(encoding='utf-8')
        except Exception as e:
            print(f"⚠️  Error reading {filepath}: {e}")
            return {}

        data = {'info': {}, 'entries': defaultdict(dict)}

        # Extract language info
        lang_match = re.search(r'\*\*Language\*\*:\s*([^\n(]+)', content)
        iso_match = re.search(r'\*\*ISO 639-3\*\*:\s*(\w+)', content)

        if lang_match:
            data['info']['language'] = lang_match.group(1).strip()
        if iso_match:
            data['info']['iso'] = iso_match.group(1).strip()

        data['info']['tier'] = tier

        # Extract greetings from sentences
        self._extract_greetings(content, data)

        # Extract numbers
        self._extract_numbers(content, data)

        # Extract colors
        self._extract_colors(content, data)

        # Extract family terms
        self._extract_family(content, data)

        # Extract days
        self._extract_days(content, data)

        return data

    def _extract_greetings(self, content: str, data: Dict):
        """Extract greeting phrases."""
        greeting_patterns = [
            (r'\*\*([^*]+)\*\*\s*/([^/]+)/\s*→\s*([^\s"]+(?:\s+[^\s"]+)*)\s*"Hello"', 'hello'),
            (r'\*\*([^*]+)\*\*\s*/([^/]+)/\s*→\s*([^\s"]+(?:\s+[^\s"]+)*)\s*"Goodbye"', 'goodbye'),
            (r'\*\*([^*]+)\*\*\s*/([^/]+)/\s*→\s*([^\s"]+(?:\s+[^\s"]+)*)\s*"Good morning"', 'good morning'),
            (r'\*\*([^*]+)\*\*\s*/([^/]+)/\s*→\s*([^\s"]+(?:\s+[^\s"]+)*)\s*"Thank you"', 'thank you'),
            (r'\*\*([^*]+)\*\*\s*/([^/]+)/\s*→\s*([^\s"]+(?:\s+[^\s"]+)*)\s*"Please"', 'please'),
            (r'\*\*([^*]+)\*\*\s*/([^/]+)/\s*→\s*([^\s"]+(?:\s+[^\s"]+)*)\s*"Sorry"', 'sorry'),
            (r'\*\*([^*]+)\*\*\s*/([^/]+)/\s*→\s*([^\s"]+(?:\s+[^\s"]+)*)\s*"Yes"', 'yes'),
            (r'\*\*([^*]+)\*\*\s*/([^/]+)/\s*→\s*([^\s"]+(?:\s+[^\s"]+)*)\s*"No"', 'no'),
            (r'\*\*([^*]+)\*\*\s*/([^/]+)/\s*→\s*([^\s"]+(?:\s+[^\s"]+)*)\s*"How are you[?]?"', 'how are you'),
        ]

        for pattern, concept in greeting_patterns:
            match = re.search(pattern, content, re.IGNORECASE)
            if match:
                data['entries']['greetings'][concept] = {
                    'original': match.group(1).strip(),
                    'ipa': match.group(2).strip(),
                    'hangul': match.group(3).strip()
                }

    def _extract_numbers(self, content: str, data: Dict):
        """Extract number pronunciations."""
        # Look for number tables
        number_section = re.search(r'## Numbers.*?(?=##|\Z)', content, re.DOTALL)
        if number_section:
            section = number_section.group()
            # Pattern: | Word (N) | /IPA/ | Hangul |
            pattern = re.compile(r'\|\s*[^|]*\((\d+)\)[^|]*\|\s*/([^/]+)/\s*\|\s*([^|]+)\s*\|')
            for match in pattern.finditer(section):
                num = match.group(1)
                data['entries']['numbers'][num] = {
                    'original': num,
                    'ipa': match.group(2).strip(),
                    'hangul': match.group(3).strip()
                }

    def _extract_colors(self, content: str, data: Dict):
        """Extract color pronunciations."""
        color_section = re.search(r'## Colors.*?(?=##|\Z)', content, re.DOTALL)
        if color_section:
            section = color_section.group()
            color_map = {
                'white': ['white', 'whit', 'blanc', 'weiß', 'blanco', '白', 'सफ़ेद', 'ак', 'alb'],
                'black': ['black', 'noir', 'schwarz', 'negro', '黑', 'काला', 'кара', 'negru'],
                'red': ['red', 'rouge', 'rot', 'rojo', '红', 'लाल', 'кызыл', 'roșu'],
                'green': ['green', 'vert', 'grün', 'verde', '绿', 'हरा', 'жашыл'],
                'blue': ['blue', 'bleu', 'blau', 'azul', '蓝', 'नीला', 'көк'],
                'yellow': ['yellow', 'jaune', 'gelb', 'amarillo', '黄', 'पीला', 'сары']
            }

            # Pattern: | Word | /IPA/ | Hangul |
            pattern = re.compile(r'\|\s*([^|]+)\s*\|\s*/([^/]+)/\s*\|\s*([^|]+)\s*\|')
            for match in pattern.finditer(section):
                word = match.group(1).strip().lower()
                for color, keywords in color_map.items():
                    if any(kw in word for kw in keywords):
                        data['entries']['colors'][color] = {
                            'original': match.group(1).strip(),
                            'ipa': match.group(2).strip(),
                            'hangul': match.group(3).strip()
                        }
                        break

    def _extract_family(self, content: str, data: Dict):
        """Extract family term pronunciations."""
        family_section = re.search(r'## Family.*?(?=##|\Z)', content, re.DOTALL)
        if family_section:
            section = family_section.group()
            family_map = {
                'mother': ['mother', 'mom', 'mère', 'madre', '母', 'माँ', 'апа', 'mamă'],
                'father': ['father', 'dad', 'père', 'padre', '父', 'पिता', 'ата', 'tată'],
                'brother': ['brother', 'frère', 'hermano', '兄', 'भाई', 'ага'],
                'sister': ['sister', 'sœur', 'hermana', '姐', 'बहन', 'эже'],
                'grandmother': ['grandmother', 'grandma', 'grand-mère', 'abuela', '祖母', 'दादी', 'чоң эне'],
                'grandfather': ['grandfather', 'grandpa', 'grand-père', 'abuelo', '祖父', 'दादा', 'чоң ата'],
                'wife': ['wife', 'femme', 'esposa', '妻', 'पत्नी', 'аял'],
                'husband': ['husband', 'mari', 'esposo', '夫', 'पति', 'күйөө'],
                'son': ['son', 'fils', 'hijo', '儿子', 'बेटा', 'уул'],
                'daughter': ['daughter', 'fille', 'hija', '女儿', 'बेटी', 'кыз']
            }

            # Pattern: | Word | /IPA/ | Hangul | Meaning |
            pattern = re.compile(r'\|\s*([^|]+)\s*\|\s*/([^/]+)/\s*\|\s*([^|]+)\s*\|\s*([^|]+)\s*\|')
            for match in pattern.finditer(section):
                meaning = match.group(4).strip().lower()
                for term, keywords in family_map.items():
                    if any(kw in meaning for kw in keywords):
                        data['entries']['family'][term] = {
                            'original': match.group(1).strip(),
                            'ipa': match.group(2).strip(),
                            'hangul': match.group(3).strip()
                        }
                        break

    def _extract_days(self, content: str, data: Dict):
        """Extract day of week pronunciations."""
        days_section = re.search(r'## Days.*?(?=##|\Z)', content, re.DOTALL)
        if days_section:
            section = days_section.group()
            days_order = ['monday', 'tuesday', 'wednesday', 'thursday', 'friday', 'saturday', 'sunday']

            pattern = re.compile(r'\|\s*([^|]+)\s*\|\s*/([^/]+)/\s*\|\s*([^|]+)\s*\|')
            matches = list(pattern.finditer(section))

            # Skip header row
            for i, match in enumerate(matches[1:8]):  # 7 days
                if i < len(days_order):
                    data['entries']['days'][days_order[i]] = {
                        'original': match.group(1).strip(),
                        'ipa': match.group(2).strip(),
                        'hangul': match.group(3).strip()
                    }

    def load_all(self):
        """Load pronunciation data from all tiers."""
        for tier in range(1, 5):
            tier_path = self.wihp_root / 'wihp' / f'tier{tier}'
            if not tier_path.exists():
                continue

            for filepath in sorted(tier_path.glob('*.md')):
                if filepath.name == 'README.md':
                    continue

                file_data = self.extract_from_file(filepath, tier)
                if not file_data.get('info'):
                    continue

                lang = file_data['info'].get('language', filepath.stem)
                iso = file_data['info'].get('iso', '')
                tier_num = file_data['info'].get('tier', tier)

                for category, entries in file_data['entries'].items():
                    for concept, entry_data in entries.items():
                        key = f"{category}:{concept}"
                        self.data[key][lang] = PronunciationEntry(
                            language=lang,
                            iso_code=iso,
                            original=entry_data['original'],
                            ipa=entry_data['ipa'],
                            hangul=entry_data['hangul'],
                            tier=tier_num
                        )


class ComparisonGenerator:
    """Generates comparison tables."""

    def __init__(self, db: PronunciationDatabase):
        self.db = db

    def compare_category(self, category: str, languages: Optional[List[str]] = None) -> Dict:
        """Compare all items in a category across languages."""
        concepts = self.db.categories.get(category, [])
        comparison = {}

        for concept in concepts:
            key = f"{category}:{concept}"
            if key in self.db.data:
                entries = self.db.data[key]
                if languages:
                    entries = {l: e for l, e in entries.items() if l in languages}
                if entries:
                    comparison[concept] = entries

        return comparison

    def to_markdown(self, comparison: Dict, title: str = "Comparison") -> str:
        """Generate Markdown comparison table."""
        if not comparison:
            return "No data available."

        # Get all languages
        all_languages = set()
        for entries in comparison.values():
            all_languages.update(entries.keys())
        languages = sorted(all_languages)

        # Build table
        lines = [f"# {title}\n"]
        lines.append("| Concept | " + " | ".join(languages) + " |")
        lines.append("|" + "---|" * (len(languages) + 1))

        for concept, entries in comparison.items():
            row = [f"**{concept}**"]
            for lang in languages:
                if lang in entries:
                    e = entries[lang]
                    row.append(f"{e.original}<br>/{e.ipa}/<br>{e.hangul}")
                else:
                    row.append("-")
            lines.append("| " + " | ".join(row) + " |")

        return "\n".join(lines)

    def to_html(self, comparison: Dict, title: str = "Comparison") -> str:
        """Generate HTML comparison table."""
        if not comparison:
            return "<p>No data available.</p>"

        all_languages = set()
        for entries in comparison.values():
            all_languages.update(entries.keys())
        languages = sorted(all_languages)

        html = f'''<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>WIHP {title}</title>
    <style>
        body {{ font-family: 'Noto Sans KR', sans-serif; padding: 20px; background: #f5f5f5; }}
        h1 {{ color: #333; }}
        table {{ border-collapse: collapse; width: 100%; background: white; box-shadow: 0 2px 8px rgba(0,0,0,0.1); }}
        th, td {{ border: 1px solid #ddd; padding: 12px; text-align: center; }}
        th {{ background: linear-gradient(135deg, #667eea, #764ba2); color: white; }}
        tr:nth-child(even) {{ background: #f9f9f9; }}
        tr:hover {{ background: #f0f0f0; }}
        .original {{ font-weight: bold; font-size: 1.1em; }}
        .ipa {{ color: #666; font-style: italic; }}
        .hangul {{ color: #4a4a8a; font-size: 1.1em; }}
        .concept {{ text-align: left; font-weight: bold; background: #f0f0ff; }}
    </style>
    <link href="https://fonts.googleapis.com/css2?family=Noto+Sans+KR&display=swap" rel="stylesheet">
</head>
<body>
    <h1>🌍 WIHP {title}</h1>
    <table>
        <thead>
            <tr>
                <th>Concept</th>
                {"".join(f"<th>{lang}</th>" for lang in languages)}
            </tr>
        </thead>
        <tbody>
'''
        for concept, entries in comparison.items():
            html += f'<tr><td class="concept">{concept}</td>'
            for lang in languages:
                if lang in entries:
                    e = entries[lang]
                    html += f'''<td>
                        <div class="original">{e.original}</div>
                        <div class="ipa">/{e.ipa}/</div>
                        <div class="hangul">{e.hangul}</div>
                    </td>'''
                else:
                    html += '<td>-</td>'
            html += '</tr>\n'

        html += '''
        </tbody>
    </table>
</body>
</html>'''
        return html

    def to_csv(self, comparison: Dict) -> str:
        """Generate CSV comparison."""
        if not comparison:
            return ""

        all_languages = set()
        for entries in comparison.values():
            all_languages.update(entries.keys())
        languages = sorted(all_languages)

        lines = ["Concept," + ",".join(f"{l}_original,{l}_ipa,{l}_hangul" for l in languages)]

        for concept, entries in comparison.items():
            row = [concept]
            for lang in languages:
                if lang in entries:
                    e = entries[lang]
                    row.extend([f'"{e.original}"', f'"{e.ipa}"', f'"{e.hangul}"'])
                else:
                    row.extend(['', '', ''])
            lines.append(",".join(row))

        return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description='WIHP Pronunciation Comparison Tool')
    parser.add_argument('--category', type=str,
                        choices=['greetings', 'numbers', 'colors', 'family', 'days', 'all'],
                        default='greetings', help='Category to compare')
    parser.add_argument('--languages', type=str,
                        help='Comma-separated list of languages to include')
    parser.add_argument('--format', type=str, default='markdown',
                        choices=['markdown', 'html', 'csv', 'all'],
                        help='Output format')
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

    print("🔍 WIHP Pronunciation Comparison Tool")
    print("=" * 40)

    # Load data
    print("\n📂 Loading pronunciation data...")
    db = PronunciationDatabase(wihp_root)
    db.load_all()

    languages = args.languages.split(',') if args.languages else None
    generator = ComparisonGenerator(db)

    categories = [args.category] if args.category != 'all' else list(db.categories.keys())
    formats = [args.format] if args.format != 'all' else ['markdown', 'html', 'csv']

    for category in categories:
        print(f"\n📊 Comparing: {category}")
        comparison = generator.compare_category(category, languages)

        if not comparison:
            print(f"   ⚠️  No data found for {category}")
            continue

        print(f"   Found {len(comparison)} concepts across {len(set(l for e in comparison.values() for l in e))} languages")

        for fmt in formats:
            if fmt == 'markdown':
                content = generator.to_markdown(comparison, f"{category.title()} Comparison")
                output_file = output_dir / f"comparison_{category}.md"
            elif fmt == 'html':
                content = generator.to_html(comparison, f"{category.title()} Comparison")
                output_file = output_dir / f"comparison_{category}.html"
            elif fmt == 'csv':
                content = generator.to_csv(comparison)
                output_file = output_dir / f"comparison_{category}.csv"

            with open(output_file, 'w', encoding='utf-8') as f:
                f.write(content)
            print(f"   📄 Saved: {output_file}")

    print(f"\n✅ Comparison complete! Files saved to: {output_dir}")


if __name__ == '__main__':
    main()
