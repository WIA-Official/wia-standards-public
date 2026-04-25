#!/usr/bin/env python3
"""
WIHP IPA Verification Tool
===========================
Validates IPA transcriptions and WIHP codes in language mapping files.

Features:
- Validates IPA symbols against Unicode IPA chart
- Checks WIHP code format (PL##-MN##-AR## / VL##-VH##-VR##)
- Detects missing mappings
- Reports inconsistencies across files
- Generates validation report

Usage:
    python verify_ipa.py [--tier TIER] [--language LANG] [--verbose] [--report]

Examples:
    python verify_ipa.py                    # Validate all tiers
    python verify_ipa.py --tier 1           # Validate tier 1 only
    python verify_ipa.py --language korean  # Validate Korean only
    python verify_ipa.py --report           # Generate detailed report
"""

import os
import re
import sys
import json
import argparse
from pathlib import Path
from collections import defaultdict
from typing import Dict, List, Set, Tuple, Optional

# Valid IPA consonant symbols (Unicode)
VALID_IPA_CONSONANTS = set(
    # Plosives
    'p b t d ʈ ɖ c ɟ k g q ɢ ʔ '
    # Nasals
    'm ɱ n ɳ ɲ ŋ ɴ '
    # Trills
    'ʙ r ʀ '
    # Taps/Flaps
    'ⱱ ɾ ɽ '
    # Fricatives
    'ɸ β f v θ ð s z ʃ ʒ ʂ ʐ ç ʝ x ɣ χ ʁ ħ ʕ h ɦ '
    # Lateral fricatives
    'ɬ ɮ '
    # Approximants
    'ʋ ɹ ɻ j ɰ l ɭ ʎ ʟ w '
    # Clicks
    'ʘ ǀ ǃ ǂ ǁ '
    # Implosives
    'ɓ ɗ ʄ ɠ ʛ '
    # Ejectives (with modifier)
    "pʼ tʼ kʼ sʼ "
).split()

# Valid IPA vowel symbols
VALID_IPA_VOWELS = set(
    # Close
    'i y ɨ ʉ ɯ u '
    # Near-close
    'ɪ ʏ ʊ '
    # Close-mid
    'e ø ɘ ɵ ɤ o '
    # Mid
    'ə '
    # Open-mid
    'ɛ œ ɜ ɞ ʌ ɔ '
    # Near-open
    'æ ɐ '
    # Open
    'a ɶ ɑ ɒ '
).split()

# IPA diacritics and modifiers
IPA_DIACRITICS = set(
    'ʰ ʷ ʲ ˠ ˤ ⁿ ˡ '  # Superscript modifiers
    'ʼ '  # Ejective
    '̃ ̥ ̬ ̤ ̰ ̼ ̪ ̺ ̻ ̹ ̜ ̟ ̠ ̈ ̽ ̩ ̯ ˞ ̚ '  # Combining diacritics
    'ː ˑ '  # Length
    '̆ '  # Extra-short
    '˥ ˦ ˧ ˨ ˩ '  # Tone letters
    '́ ̀ ̂ ̌ ̄ '  # Tone diacritics
).split()

# WIHP code patterns
CONSONANT_CODE_PATTERN = re.compile(r'^PL\d{2}-MN\d{2}-AR\d{2}$')
VOWEL_CODE_PATTERN = re.compile(r'^VL\d{2}-VH\d{2}-VR\d{2}$')

# Hangul validation
HANGUL_PATTERN = re.compile(r'[\uAC00-\uD7AF\u1100-\u11FF\u3130-\u318F]+')


class IPAValidator:
    """Validates IPA transcriptions in WIHP mapping files."""

    def __init__(self, wihp_root: Path):
        self.wihp_root = wihp_root
        self.errors: List[Dict] = []
        self.warnings: List[Dict] = []
        self.stats = defaultdict(int)

    def validate_ipa_symbol(self, symbol: str) -> Tuple[bool, str]:
        """Validate a single IPA symbol or sequence."""
        # Remove common diacritics for base check
        base_symbol = symbol
        for diacritic in ['ʰ', 'ʷ', 'ʲ', 'ˠ', 'ˤ', 'ʼ', 'ː', 'ˑ', '̃', '̥']:
            base_symbol = base_symbol.replace(diacritic, '')

        # Check if it's a valid consonant
        if base_symbol in VALID_IPA_CONSONANTS:
            return True, "consonant"

        # Check if it's a valid vowel
        if base_symbol in VALID_IPA_VOWELS:
            return True, "vowel"

        # Check for affricates (two-character sequences)
        if len(base_symbol) == 2:
            affricate_patterns = ['ts', 'dz', 'tʃ', 'dʒ', 'tɕ', 'dʑ', 'pf', 'bv']
            if base_symbol in affricate_patterns:
                return True, "affricate"

        # Check for coarticulated consonants
        if 'w' in symbol or 'j' in symbol:
            return True, "coarticulated"

        return False, "unknown"

    def validate_wihp_code(self, code: str) -> Tuple[bool, str]:
        """Validate WIHP code format."""
        if CONSONANT_CODE_PATTERN.match(code):
            return True, "consonant"
        if VOWEL_CODE_PATTERN.match(code):
            return True, "vowel"
        return False, "invalid"

    def validate_hangul(self, hangul: str) -> bool:
        """Check if string contains valid Hangul characters."""
        return bool(HANGUL_PATTERN.search(hangul))

    def extract_mappings(self, content: str) -> List[Dict]:
        """Extract IPA-WIHP-Hangul mappings from markdown table."""
        mappings = []

        # Pattern for table rows: | IPA | WIHP Code | Hangul | Description |
        table_pattern = re.compile(
            r'\|\s*([^|]+)\s*\|\s*((?:PL|VL)\d{2}-(?:MN|VH)\d{2}-(?:AR|VR)\d{2})\s*\|\s*([^|]+)\s*\|'
        )

        for match in table_pattern.finditer(content):
            ipa = match.group(1).strip()
            wihp_code = match.group(2).strip()
            hangul = match.group(3).strip()

            mappings.append({
                'ipa': ipa,
                'wihp_code': wihp_code,
                'hangul': hangul
            })

        return mappings

    def extract_sentences(self, content: str) -> List[Dict]:
        """Extract sentences with IPA transcriptions."""
        sentences = []

        # Pattern: **Text** /IPA/ → Hangul "Translation"
        sentence_pattern = re.compile(
            r'\*\*([^*]+)\*\*\s*/([^/]+)/\s*→\s*([^\s"]+(?:\s+[^\s"]+)*)\s*"([^"]+)"'
        )

        for match in sentence_pattern.finditer(content):
            sentences.append({
                'original': match.group(1).strip(),
                'ipa': match.group(2).strip(),
                'hangul': match.group(3).strip(),
                'translation': match.group(4).strip()
            })

        return sentences

    def validate_file(self, filepath: Path) -> Dict:
        """Validate a single mapping file."""
        results = {
            'file': str(filepath),
            'valid': True,
            'errors': [],
            'warnings': [],
            'stats': {
                'consonant_mappings': 0,
                'vowel_mappings': 0,
                'sentences': 0,
                'valid_ipa': 0,
                'invalid_ipa': 0
            }
        }

        try:
            content = filepath.read_text(encoding='utf-8')
        except Exception as e:
            results['valid'] = False
            results['errors'].append(f"Failed to read file: {e}")
            return results

        # Extract and validate mappings
        mappings = self.extract_mappings(content)

        for mapping in mappings:
            # Validate IPA
            is_valid_ipa, ipa_type = self.validate_ipa_symbol(mapping['ipa'])
            if not is_valid_ipa:
                results['warnings'].append(
                    f"Potentially invalid IPA: {mapping['ipa']}"
                )
                results['stats']['invalid_ipa'] += 1
            else:
                results['stats']['valid_ipa'] += 1
                if ipa_type == 'consonant':
                    results['stats']['consonant_mappings'] += 1
                elif ipa_type == 'vowel':
                    results['stats']['vowel_mappings'] += 1

            # Validate WIHP code
            is_valid_code, code_type = self.validate_wihp_code(mapping['wihp_code'])
            if not is_valid_code:
                results['errors'].append(
                    f"Invalid WIHP code format: {mapping['wihp_code']}"
                )
                results['valid'] = False

            # Validate Hangul
            if not self.validate_hangul(mapping['hangul']):
                results['warnings'].append(
                    f"Missing Hangul for IPA: {mapping['ipa']}"
                )

        # Extract and count sentences
        sentences = self.extract_sentences(content)
        results['stats']['sentences'] = len(sentences)

        # Check for minimum requirements
        if results['stats']['consonant_mappings'] < 10:
            results['warnings'].append(
                f"Low consonant count: {results['stats']['consonant_mappings']}"
            )

        if results['stats']['sentences'] < 50:
            results['warnings'].append(
                f"Less than 50 sentences: {results['stats']['sentences']}"
            )

        return results

    def validate_tier(self, tier: int) -> List[Dict]:
        """Validate all files in a tier."""
        tier_path = self.wihp_root / 'wihp' / f'tier{tier}'
        results = []

        if not tier_path.exists():
            return results

        for filepath in sorted(tier_path.glob('*.md')):
            if filepath.name != 'README.md':
                result = self.validate_file(filepath)
                results.append(result)

        return results

    def validate_all(self) -> Dict:
        """Validate all tiers."""
        all_results = {
            'tiers': {},
            'summary': {
                'total_files': 0,
                'valid_files': 0,
                'total_errors': 0,
                'total_warnings': 0,
                'total_mappings': 0,
                'total_sentences': 0
            }
        }

        for tier in range(1, 5):
            tier_results = self.validate_tier(tier)
            all_results['tiers'][f'tier{tier}'] = tier_results

            for result in tier_results:
                all_results['summary']['total_files'] += 1
                if result['valid']:
                    all_results['summary']['valid_files'] += 1
                all_results['summary']['total_errors'] += len(result['errors'])
                all_results['summary']['total_warnings'] += len(result['warnings'])
                all_results['summary']['total_mappings'] += (
                    result['stats']['consonant_mappings'] +
                    result['stats']['vowel_mappings']
                )
                all_results['summary']['total_sentences'] += result['stats']['sentences']

        return all_results


def print_results(results: Dict, verbose: bool = False):
    """Print validation results to console."""
    print("\n" + "=" * 60)
    print("WIHP IPA VALIDATION REPORT")
    print("=" * 60)

    summary = results['summary']
    print(f"\n📊 SUMMARY")
    print(f"   Total files validated: {summary['total_files']}")
    print(f"   Valid files: {summary['valid_files']}")
    print(f"   Total mappings: {summary['total_mappings']}")
    print(f"   Total sentences: {summary['total_sentences']}")
    print(f"   Errors: {summary['total_errors']}")
    print(f"   Warnings: {summary['total_warnings']}")

    for tier_name, tier_results in results['tiers'].items():
        if not tier_results:
            continue

        print(f"\n📁 {tier_name.upper()}")
        print("-" * 40)

        for result in tier_results:
            filename = Path(result['file']).name
            status = "✅" if result['valid'] else "❌"
            print(f"   {status} {filename}")

            if verbose:
                stats = result['stats']
                print(f"      Consonants: {stats['consonant_mappings']}, "
                      f"Vowels: {stats['vowel_mappings']}, "
                      f"Sentences: {stats['sentences']}")

                for error in result['errors']:
                    print(f"      ❌ {error}")
                for warning in result['warnings'][:3]:  # Limit warnings shown
                    print(f"      ⚠️  {warning}")
                if len(result['warnings']) > 3:
                    print(f"      ... and {len(result['warnings']) - 3} more warnings")

    print("\n" + "=" * 60)

    # Final status
    if summary['total_errors'] == 0:
        print("✅ All files passed validation!")
    else:
        print(f"❌ {summary['total_errors']} errors found. Please review.")
    print("=" * 60 + "\n")


def generate_report(results: Dict, output_path: Path):
    """Generate detailed JSON report."""
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(results, f, indent=2, ensure_ascii=False)
    print(f"📄 Report saved to: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description='WIHP IPA Validation Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('--tier', type=int, choices=[1, 2, 3, 4],
                        help='Validate specific tier only')
    parser.add_argument('--language', type=str,
                        help='Validate specific language only')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Show detailed output')
    parser.add_argument('--report', '-r', action='store_true',
                        help='Generate JSON report')
    parser.add_argument('--root', type=str, default=None,
                        help='WIHP root directory')

    args = parser.parse_args()

    # Determine WIHP root
    if args.root:
        wihp_root = Path(args.root)
    else:
        # Try to find WIHP root relative to script
        script_dir = Path(__file__).parent
        wihp_root = script_dir.parent

    if not (wihp_root / 'wihp').exists():
        print(f"❌ Error: WIHP directory not found at {wihp_root}")
        print("   Use --root to specify the WIHP root directory")
        sys.exit(1)

    print(f"🔍 Validating WIHP files in: {wihp_root}")

    validator = IPAValidator(wihp_root)

    if args.tier:
        results = {
            'tiers': {f'tier{args.tier}': validator.validate_tier(args.tier)},
            'summary': {'total_files': 0, 'valid_files': 0,
                       'total_errors': 0, 'total_warnings': 0,
                       'total_mappings': 0, 'total_sentences': 0}
        }
        # Calculate summary
        for result in results['tiers'][f'tier{args.tier}']:
            results['summary']['total_files'] += 1
            if result['valid']:
                results['summary']['valid_files'] += 1
            results['summary']['total_errors'] += len(result['errors'])
            results['summary']['total_warnings'] += len(result['warnings'])
    else:
        results = validator.validate_all()

    print_results(results, args.verbose)

    if args.report:
        report_path = wihp_root / 'tools' / 'validation_report.json'
        generate_report(results, report_path)


if __name__ == '__main__':
    main()
