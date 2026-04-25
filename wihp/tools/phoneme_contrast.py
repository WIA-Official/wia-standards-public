#!/usr/bin/env python3
"""
WIHP Phoneme Contrast Tool
===========================
Analyzes and contrasts phonological systems across languages.

Features:
- Extract phoneme inventories from all languages
- Compare consonant/vowel inventories
- Identify shared and unique phonemes
- Generate phonological feature matrices
- Create typological comparisons
- Highlight challenging sounds for learners

Usage:
    python phoneme_contrast.py [--languages LANG1,LANG2] [--feature FEATURE]

Examples:
    python phoneme_contrast.py --languages korean,japanese
    python phoneme_contrast.py --feature clicks
    python phoneme_contrast.py --summary
"""

import os
import re
import sys
import json
import argparse
from pathlib import Path
from typing import Dict, List, Set, Tuple, Optional
from collections import defaultdict
from dataclasses import dataclass, field

@dataclass
class PhonemeInventory:
    """Phoneme inventory for a language."""
    language: str
    iso_code: str
    tier: int
    consonants: Dict[str, Dict] = field(default_factory=dict)  # IPA -> {wihp, hangul, description}
    vowels: Dict[str, Dict] = field(default_factory=dict)
    features: Set[str] = field(default_factory=set)  # Special phonological features


# Phoneme feature classifications
MANNER_FEATURES = {
    'MN01': 'plosive',
    'MN02': 'nasal',
    'MN03': 'fricative',
    'MN04': 'affricate',
    'MN05': 'approximant',
    'MN06': 'lateral',
    'MN07': 'flap',
    'MN08': 'trill'
}

PLACE_FEATURES = {
    'PL01': 'bilabial',
    'PL02': 'labiodental',
    'PL03': 'dental',
    'PL04': 'alveolar',
    'PL05': 'retroflex',
    'PL06': 'palatal',
    'PL07': 'velar',
    'PL08': 'uvular',
    'PL09': 'glottal',
    'PL10': 'pharyngeal'
}

AIRSTREAM_FEATURES = {
    'AR01': 'voiceless',
    'AR02': 'aspirated',
    'AR03': 'voiced',
    'AR04': 'breathy',
    'AR05': 'implosive',
    'AR06': 'ejective',
    'AR07': 'click'
}

# Special phonological features to detect
SPECIAL_FEATURES = {
    'clicks': ['ʘ', 'ǀ', 'ǃ', 'ǂ', 'ǁ'],
    'implosives': ['ɓ', 'ɗ', 'ʄ', 'ɠ', 'ʛ'],
    'ejectives': ['pʼ', 'tʼ', 'kʼ', 'sʼ', 'tsʼ'],
    'retroflexes': ['ʈ', 'ɖ', 'ɳ', 'ɽ', 'ʂ', 'ʐ', 'ɻ', 'ɭ'],
    'pharyngeals': ['ħ', 'ʕ'],
    'uvulars': ['q', 'ɢ', 'ɴ', 'ʀ', 'χ', 'ʁ'],
    'vowel_nasalization': ['ã', 'ẽ', 'ĩ', 'õ', 'ũ', 'ɛ̃', 'ɔ̃'],
    'vowel_length': ['ː', 'ˑ'],
    'tones': ['˥', '˦', '˧', '˨', '˩', '́', '̀', '̂', '̌'],
    'voiceless_sonorants': ['m̥', 'n̥', 'l̥', 'r̥', 'ŋ̊'],
    'breathy_voice': ['bʱ', 'dʱ', 'gʱ', 'ɦ']
}


class PhonemeExtractor:
    """Extracts phoneme inventories from WIHP files."""

    def __init__(self, wihp_root: Path):
        self.wihp_root = wihp_root
        self.inventories: Dict[str, PhonemeInventory] = {}

    def extract_from_file(self, filepath: Path, tier: int) -> Optional[PhonemeInventory]:
        """Extract phoneme inventory from a single file."""
        try:
            content = filepath.read_text(encoding='utf-8')
        except Exception as e:
            print(f"⚠️  Error reading {filepath}: {e}")
            return None

        # Extract language info
        lang_match = re.search(r'\*\*Language\*\*:\s*([^\n(]+)', content)
        iso_match = re.search(r'\*\*ISO 639-3\*\*:\s*(\w+)', content)

        language = lang_match.group(1).strip() if lang_match else filepath.stem
        iso_code = iso_match.group(1).strip() if iso_match else ""

        inventory = PhonemeInventory(
            language=language,
            iso_code=iso_code,
            tier=tier
        )

        # Extract consonants
        consonant_pattern = re.compile(
            r'\|\s*([^|]+)\s*\|\s*(PL\d{2}-MN\d{2}-AR\d{2})\s*\|\s*([^|]+)\s*\|\s*([^|]+)\s*\|'
        )

        for match in consonant_pattern.finditer(content):
            ipa = match.group(1).strip()
            wihp = match.group(2).strip()
            hangul = match.group(3).strip()
            description = match.group(4).strip()

            if ipa and ipa != 'IPA' and not ipa.startswith('-'):
                inventory.consonants[ipa] = {
                    'wihp': wihp,
                    'hangul': hangul,
                    'description': description
                }

        # Extract vowels
        vowel_pattern = re.compile(
            r'\|\s*([^|]+)\s*\|\s*(VL\d{2}-VH\d{2}-VR\d{2})\s*\|\s*([^|]+)\s*\|\s*([^|]+)\s*\|'
        )

        for match in vowel_pattern.finditer(content):
            ipa = match.group(1).strip()
            wihp = match.group(2).strip()
            hangul = match.group(3).strip()
            description = match.group(4).strip()

            if ipa and ipa != 'IPA' and not ipa.startswith('-'):
                inventory.vowels[ipa] = {
                    'wihp': wihp,
                    'hangul': hangul,
                    'description': description
                }

        # Detect special features
        all_phonemes = set(inventory.consonants.keys()) | set(inventory.vowels.keys())
        content_lower = content.lower()

        for feature, markers in SPECIAL_FEATURES.items():
            for marker in markers:
                if marker in all_phonemes or marker in content:
                    inventory.features.add(feature)
                    break

        # Also check feature descriptions
        if 'click' in content_lower:
            inventory.features.add('clicks')
        if 'implosive' in content_lower:
            inventory.features.add('implosives')
        if 'ejective' in content_lower:
            inventory.features.add('ejectives')
        if 'retroflex' in content_lower:
            inventory.features.add('retroflexes')
        if 'tone' in content_lower or 'tonal' in content_lower:
            inventory.features.add('tones')
        if 'vowel harmony' in content_lower:
            inventory.features.add('vowel_harmony')
        if 'nasal vowel' in content_lower or 'nasalized' in content_lower:
            inventory.features.add('vowel_nasalization')

        return inventory

    def load_all(self):
        """Load phoneme inventories from all tiers."""
        for tier in range(1, 5):
            tier_path = self.wihp_root / 'wihp' / f'tier{tier}'
            if not tier_path.exists():
                continue

            for filepath in sorted(tier_path.glob('*.md')):
                if filepath.name == 'README.md':
                    continue

                inventory = self.extract_from_file(filepath, tier)
                if inventory:
                    self.inventories[inventory.language] = inventory
                    print(f"   📝 {inventory.language}: {len(inventory.consonants)}C + {len(inventory.vowels)}V")


class PhonemeAnalyzer:
    """Analyzes and compares phoneme inventories."""

    def __init__(self, inventories: Dict[str, PhonemeInventory]):
        self.inventories = inventories

    def compare_two(self, lang1: str, lang2: str) -> Dict:
        """Compare phoneme inventories of two languages."""
        inv1 = self.inventories.get(lang1)
        inv2 = self.inventories.get(lang2)

        if not inv1 or not inv2:
            return {'error': f"Language not found: {lang1 if not inv1 else lang2}"}

        c1 = set(inv1.consonants.keys())
        c2 = set(inv2.consonants.keys())
        v1 = set(inv1.vowels.keys())
        v2 = set(inv2.vowels.keys())

        return {
            'languages': [lang1, lang2],
            'consonants': {
                'shared': sorted(c1 & c2),
                f'unique_to_{lang1}': sorted(c1 - c2),
                f'unique_to_{lang2}': sorted(c2 - c1),
                f'{lang1}_count': len(c1),
                f'{lang2}_count': len(c2)
            },
            'vowels': {
                'shared': sorted(v1 & v2),
                f'unique_to_{lang1}': sorted(v1 - v2),
                f'unique_to_{lang2}': sorted(v2 - v1),
                f'{lang1}_count': len(v1),
                f'{lang2}_count': len(v2)
            },
            'features': {
                'shared': sorted(inv1.features & inv2.features),
                f'unique_to_{lang1}': sorted(inv1.features - inv2.features),
                f'unique_to_{lang2}': sorted(inv2.features - inv1.features)
            }
        }

    def find_languages_with_feature(self, feature: str) -> List[str]:
        """Find all languages with a specific phonological feature."""
        return [lang for lang, inv in self.inventories.items()
                if feature in inv.features]

    def generate_summary(self) -> Dict:
        """Generate overall phonological summary."""
        summary = {
            'total_languages': len(self.inventories),
            'by_tier': defaultdict(list),
            'consonant_stats': {'min': 100, 'max': 0, 'avg': 0},
            'vowel_stats': {'min': 100, 'max': 0, 'avg': 0},
            'feature_distribution': defaultdict(list),
            'all_consonants': set(),
            'all_vowels': set()
        }

        total_c, total_v = 0, 0

        for lang, inv in self.inventories.items():
            summary['by_tier'][inv.tier].append(lang)

            c_count = len(inv.consonants)
            v_count = len(inv.vowels)

            total_c += c_count
            total_v += v_count

            summary['consonant_stats']['min'] = min(summary['consonant_stats']['min'], c_count)
            summary['consonant_stats']['max'] = max(summary['consonant_stats']['max'], c_count)
            summary['vowel_stats']['min'] = min(summary['vowel_stats']['min'], v_count)
            summary['vowel_stats']['max'] = max(summary['vowel_stats']['max'], v_count)

            summary['all_consonants'].update(inv.consonants.keys())
            summary['all_vowels'].update(inv.vowels.keys())

            for feature in inv.features:
                summary['feature_distribution'][feature].append(lang)

        if self.inventories:
            summary['consonant_stats']['avg'] = round(total_c / len(self.inventories), 1)
            summary['vowel_stats']['avg'] = round(total_v / len(self.inventories), 1)

        # Convert sets to sorted lists for JSON serialization
        summary['all_consonants'] = sorted(summary['all_consonants'])
        summary['all_vowels'] = sorted(summary['all_vowels'])
        summary['by_tier'] = dict(summary['by_tier'])
        summary['feature_distribution'] = {k: sorted(v) for k, v in summary['feature_distribution'].items()}

        return summary

    def generate_typology_matrix(self) -> Dict:
        """Generate a typological feature matrix."""
        features = [
            'clicks', 'implosives', 'ejectives', 'retroflexes',
            'pharyngeals', 'uvulars', 'vowel_nasalization',
            'vowel_length', 'tones', 'voiceless_sonorants',
            'breathy_voice', 'vowel_harmony'
        ]

        matrix = {}
        for lang, inv in self.inventories.items():
            matrix[lang] = {f: f in inv.features for f in features}

        return matrix


class ContrastReportGenerator:
    """Generates phoneme contrast reports."""

    def __init__(self, analyzer: PhonemeAnalyzer):
        self.analyzer = analyzer

    def comparison_to_markdown(self, comparison: Dict) -> str:
        """Convert comparison to Markdown."""
        if 'error' in comparison:
            return f"Error: {comparison['error']}"

        langs = comparison['languages']
        lines = [
            f"# Phoneme Contrast: {langs[0]} vs {langs[1]}\n",
            "## Consonants\n",
            f"| Category | Count |",
            f"|----------|-------|",
            f"| {langs[0]} total | {comparison['consonants'][f'{langs[0]}_count']} |",
            f"| {langs[1]} total | {comparison['consonants'][f'{langs[1]}_count']} |",
            f"| Shared | {len(comparison['consonants']['shared'])} |",
            "",
            "### Shared Consonants",
            ", ".join(f"/{c}/" for c in comparison['consonants']['shared']) or "None",
            "",
            f"### Unique to {langs[0]}",
            ", ".join(f"/{c}/" for c in comparison['consonants'][f'unique_to_{langs[0]}']) or "None",
            "",
            f"### Unique to {langs[1]}",
            ", ".join(f"/{c}/" for c in comparison['consonants'][f'unique_to_{langs[1]}']) or "None",
            "",
            "## Vowels\n",
            f"| Category | Count |",
            f"|----------|-------|",
            f"| {langs[0]} total | {comparison['vowels'][f'{langs[0]}_count']} |",
            f"| {langs[1]} total | {comparison['vowels'][f'{langs[1]}_count']} |",
            f"| Shared | {len(comparison['vowels']['shared'])} |",
            "",
            "### Shared Vowels",
            ", ".join(f"/{v}/" for v in comparison['vowels']['shared']) or "None",
            "",
            "## Special Features\n",
            "### Shared Features",
            ", ".join(comparison['features']['shared']) or "None",
            "",
            f"### {langs[0]} Unique Features",
            ", ".join(comparison['features'][f'unique_to_{langs[0]}']) or "None",
            "",
            f"### {langs[1]} Unique Features",
            ", ".join(comparison['features'][f'unique_to_{langs[1]}']) or "None"
        ]

        return "\n".join(lines)

    def summary_to_markdown(self, summary: Dict) -> str:
        """Convert summary to Markdown."""
        lines = [
            "# WIHP Phonological Summary\n",
            f"**Total Languages:** {summary['total_languages']}\n",
            "## Languages by Tier\n"
        ]

        for tier in sorted(summary['by_tier'].keys()):
            langs = summary['by_tier'][tier]
            lines.append(f"### Tier {tier} ({len(langs)} languages)")
            lines.append(", ".join(sorted(langs)))
            lines.append("")

        lines.extend([
            "## Inventory Statistics\n",
            "| Metric | Consonants | Vowels |",
            "|--------|------------|--------|",
            f"| Minimum | {summary['consonant_stats']['min']} | {summary['vowel_stats']['min']} |",
            f"| Maximum | {summary['consonant_stats']['max']} | {summary['vowel_stats']['max']} |",
            f"| Average | {summary['consonant_stats']['avg']} | {summary['vowel_stats']['avg']} |",
            "",
            f"**Total unique consonants:** {len(summary['all_consonants'])}",
            f"**Total unique vowels:** {len(summary['all_vowels'])}",
            "",
            "## Feature Distribution\n",
            "| Feature | Languages |",
            "|---------|-----------|"
        ])

        for feature, langs in sorted(summary['feature_distribution'].items()):
            lines.append(f"| {feature.replace('_', ' ').title()} | {len(langs)} |")

        return "\n".join(lines)

    def typology_to_html(self, matrix: Dict) -> str:
        """Generate HTML typology matrix."""
        features = list(next(iter(matrix.values())).keys())
        languages = sorted(matrix.keys())

        html = '''<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>WIHP Typological Matrix</title>
    <style>
        body { font-family: 'Noto Sans KR', sans-serif; padding: 20px; background: #f5f5f5; }
        h1 { color: #333; }
        table { border-collapse: collapse; background: white; box-shadow: 0 2px 8px rgba(0,0,0,0.1); }
        th, td { border: 1px solid #ddd; padding: 8px; text-align: center; min-width: 30px; }
        th { background: linear-gradient(135deg, #667eea, #764ba2); color: white; font-size: 12px; }
        th.lang { text-align: left; min-width: 120px; }
        td.lang { text-align: left; font-weight: bold; background: #f8f8ff; }
        .yes { background: #4ade80; color: white; }
        .no { background: #fecaca; color: #999; }
        .feature-header { writing-mode: vertical-rl; transform: rotate(180deg); }
    </style>
</head>
<body>
    <h1>🌍 WIHP Typological Feature Matrix</h1>
    <table>
        <thead>
            <tr>
                <th class="lang">Language</th>
'''
        for f in features:
            fname = f.replace('_', ' ').title()
            html += f'<th><span class="feature-header">{fname}</span></th>'

        html += '</tr></thead><tbody>'

        for lang in languages:
            html += f'<tr><td class="lang">{lang}</td>'
            for f in features:
                has_feature = matrix[lang][f]
                cls = 'yes' if has_feature else 'no'
                symbol = '✓' if has_feature else '–'
                html += f'<td class="{cls}">{symbol}</td>'
            html += '</tr>'

        html += '''
        </tbody>
    </table>
    <p style="margin-top: 20px; color: #666;">
        ✓ = Feature present | – = Feature absent
    </p>
</body>
</html>'''
        return html


def main():
    parser = argparse.ArgumentParser(description='WIHP Phoneme Contrast Tool')
    parser.add_argument('--languages', type=str,
                        help='Compare two languages (comma-separated)')
    parser.add_argument('--feature', type=str,
                        help='Find languages with specific feature')
    parser.add_argument('--summary', action='store_true',
                        help='Generate overall phonological summary')
    parser.add_argument('--typology', action='store_true',
                        help='Generate typological feature matrix')
    parser.add_argument('--format', type=str, default='markdown',
                        choices=['markdown', 'html', 'json'],
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

    print("🔬 WIHP Phoneme Contrast Tool")
    print("=" * 40)

    # Load inventories
    print("\n📂 Loading phoneme inventories...")
    extractor = PhonemeExtractor(wihp_root)
    extractor.load_all()

    analyzer = PhonemeAnalyzer(extractor.inventories)
    reporter = ContrastReportGenerator(analyzer)

    if args.languages:
        # Compare two languages
        langs = args.languages.split(',')
        if len(langs) != 2:
            print("❌ Please provide exactly two languages separated by comma")
            sys.exit(1)

        print(f"\n🔍 Comparing: {langs[0]} vs {langs[1]}")
        comparison = analyzer.compare_two(langs[0].strip(), langs[1].strip())

        if args.format == 'markdown':
            content = reporter.comparison_to_markdown(comparison)
            output_file = output_dir / f"contrast_{langs[0]}_{langs[1]}.md"
        elif args.format == 'json':
            content = json.dumps(comparison, indent=2, ensure_ascii=False)
            output_file = output_dir / f"contrast_{langs[0]}_{langs[1]}.json"

        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f"📄 Saved: {output_file}")

    elif args.feature:
        # Find languages with feature
        print(f"\n🔍 Finding languages with feature: {args.feature}")
        langs = analyzer.find_languages_with_feature(args.feature)

        if langs:
            print(f"\n✅ Found {len(langs)} languages with '{args.feature}':")
            for lang in sorted(langs):
                print(f"   • {lang}")
        else:
            print(f"❌ No languages found with feature: {args.feature}")

    elif args.typology:
        # Generate typology matrix
        print("\n📊 Generating typological matrix...")
        matrix = analyzer.generate_typology_matrix()

        if args.format == 'html':
            content = reporter.typology_to_html(matrix)
            output_file = output_dir / "typology_matrix.html"
        else:
            content = json.dumps(matrix, indent=2, ensure_ascii=False)
            output_file = output_dir / "typology_matrix.json"

        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f"📄 Saved: {output_file}")

    else:
        # Default: generate summary
        print("\n📊 Generating phonological summary...")
        summary = analyzer.generate_summary()

        if args.format == 'markdown':
            content = reporter.summary_to_markdown(summary)
            output_file = output_dir / "phoneme_summary.md"
        elif args.format == 'json':
            content = json.dumps(summary, indent=2, ensure_ascii=False)
            output_file = output_dir / "phoneme_summary.json"

        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f"📄 Saved: {output_file}")

    print(f"\n✅ Analysis complete! Output saved to: {output_dir}")


if __name__ == '__main__':
    main()
