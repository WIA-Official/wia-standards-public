/**
 * WIA-INTENT SDK
 *
 * 의도 기반 프로그래밍 언어 - AI 시대의 새로운 패러다임
 *
 * "어떻게"가 아니라 "무엇을 원하는지"를 표현
 * 코드가 아니라 욕망을 표현
 *
 * 홍익인간 (弘益人間) - Benefit All Humanity
 *
 * @version 1.0.0
 * @author World Certification Industry Association (WIA)
 */

// Type exports
export * from './types';

// Parser exports
export { Lexer } from './parser/lexer';
export { Parser } from './parser/parser';

import { Lexer } from './parser/lexer';
import { Parser } from './parser/parser';
import {
  ProgramNode,
  IntentNode,
  DesireNode,
  IntentResult,
  ExecutionContext,
  CompileTarget,
  CompilationOptions,
  CompilationResult,
  CompilerWarning,
  EthicsViolationError,
  CertaintyError,
} from './types';

// ============================================================
// Main Compiler/Interpreter Class
// ============================================================

export class WiaIntent {
  private options: Partial<CompilationOptions>;

  constructor(options: Partial<CompilationOptions> = {}) {
    this.options = {
      target: 'pseudo',
      optimize: true,
      includeComments: true,
      strictMode: false,
      ethicsEnforcement: 'warn',
      ...options,
    };
  }

  /**
   * Parse WIA-INTENT source code to AST
   */
  parse(source: string): ProgramNode {
    const lexer = new Lexer(source);
    const tokens = lexer.tokenize();
    const parser = new Parser(tokens);
    return parser.parse();
  }

  /**
   * Compile WIA-INTENT to target language
   */
  compile(source: string, target?: CompileTarget): CompilationResult {
    const ast = this.parse(source);
    const targetLang = target || (this.options.target as CompileTarget);

    return this.generateCode(ast, targetLang);
  }

  /**
   * Generate code for target language
   */
  private generateCode(ast: ProgramNode, target: CompileTarget): CompilationResult {
    const warnings: CompilerWarning[] = [];
    let code = '';

    switch (target) {
      case 'pseudo':
        code = this.toPseudoCode(ast);
        break;
      case 'python':
        code = this.toPython(ast);
        break;
      case 'typescript':
        code = this.toTypeScript(ast);
        break;
      default:
        warnings.push({
          message: `Target '${target}' not fully implemented, using pseudo-code`,
          severity: 'warning',
        });
        code = this.toPseudoCode(ast);
    }

    return {
      target,
      code,
      warnings,
      stats: {
        intentCount: ast.declarations.filter(d => d.type === 'Intent').length,
        desireCount: ast.declarations.filter(d => d.type === 'Desire').length,
        constraintCount: this.countConstraints(ast),
        evolutionRulesCount: this.countEvolutionRules(ast),
        estimatedComplexity: this.estimateComplexity(ast),
      },
    };
  }

  /**
   * Generate pseudo-code (human readable)
   */
  private toPseudoCode(ast: ProgramNode): string {
    const lines: string[] = [
      '// WIA-INTENT Pseudo-Code',
      '// Generated: ' + new Date().toISOString(),
      '',
    ];

    for (const decl of ast.declarations) {
      if (decl.type === 'Intent') {
        lines.push(...this.intentToPseudo(decl));
      } else if (decl.type === 'Desire') {
        lines.push(...this.desireToPseudo(decl));
      }
      lines.push('');
    }

    return lines.join('\n');
  }

  private intentToPseudo(intent: IntentNode): string[] {
    const lines: string[] = [];
    lines.push(`INTENT ${intent.name}:`);

    if (intent.given) {
      const inputs = Object.keys(intent.given.inputs).join(', ');
      lines.push(`  GIVEN: ${inputs}`);
    }

    const outputs = Object.keys(intent.want.outputs).join(', ');
    lines.push(`  WANT: ${outputs || 'result'}`);

    if (intent.constraints) {
      lines.push(`  CONSTRAINTS:`);
      for (const c of intent.constraints.constraints) {
        lines.push(`    - ${c.name} ${c.operator} ${this.exprToString(c.value)}`);
      }
    }

    if (intent.certainty) {
      lines.push(`  CERTAINTY: ${intent.certainty.operator} ${intent.certainty.threshold}`);
    }

    if (intent.fallback) {
      lines.push(`  FALLBACK:`);
      for (const rule of intent.fallback.rules) {
        lines.push(`    IF ${this.exprToString(rule.condition)}: ${this.exprToString(rule.action)}`);
      }
    }

    if (intent.evolve) {
      lines.push(`  EVOLVE:`);
      if (intent.evolve.learnFrom) {
        lines.push(`    learn_from: ${intent.evolve.learnFrom.join(', ')}`);
      }
      if (intent.evolve.improve) {
        lines.push(`    improve: ${intent.evolve.improve.metric} via ${intent.evolve.improve.method}`);
      }
    }

    return lines;
  }

  private desireToPseudo(desire: DesireNode): string[] {
    const lines: string[] = [];
    lines.push(`DESIRE ${desire.name}:`);
    lines.push(`  ULTIMATE_GOAL: "${desire.ultimateGoal}"`);

    if (Object.keys(desire.indicators).length > 0) {
      lines.push(`  INDICATORS:`);
      for (const [key, value] of Object.entries(desire.indicators)) {
        lines.push(`    - ${key}: ${this.exprToString(value)}`);
      }
    }

    if (Object.keys(desire.ethics).length > 0) {
      lines.push(`  ETHICS:`);
      for (const [key, value] of Object.entries(desire.ethics)) {
        lines.push(`    - ${key}: ${this.exprToString(value)}`);
      }
    }

    return lines;
  }

  /**
   * Generate Python code
   */
  private toPython(ast: ProgramNode): string {
    const lines: string[] = [
      '"""',
      'WIA-INTENT Generated Python Code',
      `Generated: ${new Date().toISOString()}`,
      '"""',
      '',
      'from dataclasses import dataclass',
      'from typing import Any, Optional, Dict, List',
      'from abc import ABC, abstractmethod',
      '',
      '',
      '# Base Intent class',
      'class Intent(ABC):',
      '    @abstractmethod',
      '    def execute(self, context: Dict[str, Any]) -> Dict[str, Any]:',
      '        pass',
      '',
      '    @abstractmethod',
      '    def get_certainty(self) -> float:',
      '        pass',
      '',
    ];

    for (const decl of ast.declarations) {
      if (decl.type === 'Intent') {
        lines.push(...this.intentToPython(decl));
        lines.push('');
      }
    }

    return lines.join('\n');
  }

  private intentToPython(intent: IntentNode): string[] {
    const className = intent.name;
    const lines: string[] = [];

    lines.push(`class ${className}(Intent):`);
    lines.push(`    """Intent: ${intent.name}"""`);
    lines.push('');

    // Constructor
    lines.push('    def __init__(self):');
    if (intent.certainty) {
      lines.push(`        self.min_certainty = ${intent.certainty.threshold}`);
    } else {
      lines.push('        self.min_certainty = 0.5');
    }
    lines.push('        self.constraints = {}');

    if (intent.constraints) {
      for (const c of intent.constraints.constraints) {
        lines.push(`        self.constraints["${c.name}"] = "${this.exprToString(c.value)}"`);
      }
    }
    lines.push('');

    // Execute method
    lines.push('    def execute(self, context: Dict[str, Any]) -> Dict[str, Any]:');
    lines.push('        # TODO: Implement intent logic');
    lines.push('        result = {}');

    if (intent.given) {
      for (const input of Object.keys(intent.given.inputs)) {
        lines.push(`        ${input} = context.get("${input}")`);
      }
    }

    lines.push('');
    lines.push('        # Apply constraints and compute result');
    lines.push('        # This is where AI would generate the implementation');
    lines.push('');
    lines.push('        return {"result": result, "certainty": self.get_certainty()}');
    lines.push('');

    // Get certainty method
    lines.push('    def get_certainty(self) -> float:');
    lines.push('        # TODO: Compute actual certainty');
    lines.push(`        return ${intent.certainty?.threshold || 0.5}`);

    return lines;
  }

  /**
   * Generate TypeScript code
   */
  private toTypeScript(ast: ProgramNode): string {
    const lines: string[] = [
      '/**',
      ' * WIA-INTENT Generated TypeScript Code',
      ` * Generated: ${new Date().toISOString()}`,
      ' */',
      '',
      'interface IntentResult {',
      '  result: any;',
      '  certainty: number;',
      '  metadata: Record<string, any>;',
      '}',
      '',
      'interface IntentContext {',
      '  [key: string]: any;',
      '}',
      '',
      'abstract class Intent {',
      '  abstract execute(context: IntentContext): Promise<IntentResult>;',
      '  abstract getCertainty(): number;',
      '}',
      '',
    ];

    for (const decl of ast.declarations) {
      if (decl.type === 'Intent') {
        lines.push(...this.intentToTypeScript(decl));
        lines.push('');
      }
    }

    return lines.join('\n');
  }

  private intentToTypeScript(intent: IntentNode): string[] {
    const className = intent.name;
    const lines: string[] = [];

    lines.push(`class ${className} extends Intent {`);
    lines.push(`  private minCertainty: number;`);
    lines.push(`  private constraints: Record<string, any>;`);
    lines.push('');

    // Constructor
    lines.push('  constructor() {');
    lines.push('    super();');
    lines.push(`    this.minCertainty = ${intent.certainty?.threshold || 0.5};`);
    lines.push('    this.constraints = {};');

    if (intent.constraints) {
      for (const c of intent.constraints.constraints) {
        lines.push(`    this.constraints["${c.name}"] = "${this.exprToString(c.value)}";`);
      }
    }
    lines.push('  }');
    lines.push('');

    // Execute method
    lines.push('  async execute(context: IntentContext): Promise<IntentResult> {');
    lines.push('    // TODO: AI-generated implementation');

    if (intent.given) {
      for (const input of Object.keys(intent.given.inputs)) {
        lines.push(`    const ${input} = context["${input}"];`);
      }
    }

    lines.push('');
    lines.push('    // Placeholder result');
    lines.push('    const result = {};');
    lines.push('');
    lines.push('    return {');
    lines.push('      result,');
    lines.push('      certainty: this.getCertainty(),');
    lines.push('      metadata: { constraints: this.constraints }');
    lines.push('    };');
    lines.push('  }');
    lines.push('');

    // Get certainty method
    lines.push('  getCertainty(): number {');
    lines.push(`    return ${intent.certainty?.threshold || 0.5};`);
    lines.push('  }');
    lines.push('}');

    return lines;
  }

  // ============================================================
  // Helper Methods
  // ============================================================

  private exprToString(expr: any): string {
    if (!expr) return 'undefined';

    switch (expr.type) {
      case 'Literal':
        return expr.valueType === 'string' ? `"${expr.value}"` : String(expr.value);
      case 'Identifier':
        return expr.name;
      case 'BinaryOp':
        return `${this.exprToString(expr.left)} ${expr.operator} ${this.exprToString(expr.right)}`;
      case 'MemberAccess':
        return `${this.exprToString(expr.object)}.${expr.member}`;
      case 'Range':
        return `${expr.start}..${expr.end}`;
      case 'List':
        return `[${expr.elements.map((e: any) => this.exprToString(e)).join(', ')}]`;
      default:
        return JSON.stringify(expr);
    }
  }

  private countConstraints(ast: ProgramNode): number {
    let count = 0;
    for (const decl of ast.declarations) {
      if (decl.type === 'Intent' && decl.constraints) {
        count += decl.constraints.constraints.length;
      }
    }
    return count;
  }

  private countEvolutionRules(ast: ProgramNode): number {
    let count = 0;
    for (const decl of ast.declarations) {
      if (decl.type === 'Intent' && decl.evolve) {
        if (decl.evolve.learnFrom) count += decl.evolve.learnFrom.length;
        if (decl.evolve.adaptTo) count += decl.evolve.adaptTo.length;
        if (decl.evolve.improve) count++;
      }
    }
    return count;
  }

  private estimateComplexity(ast: ProgramNode): number {
    // Simple complexity estimation
    let complexity = ast.declarations.length * 10;
    complexity += this.countConstraints(ast) * 5;
    complexity += this.countEvolutionRules(ast) * 15;
    return complexity;
  }
}

// ============================================================
// Convenience Functions
// ============================================================

/**
 * Quick parse function
 */
export function parse(source: string): ProgramNode {
  const wia = new WiaIntent();
  return wia.parse(source);
}

/**
 * Quick compile function
 */
export function compile(
  source: string,
  target: CompileTarget = 'pseudo'
): CompilationResult {
  const wia = new WiaIntent();
  return wia.compile(source, target);
}

/**
 * Version information
 */
export const VERSION = '1.0.0';
export const WIA_INTENT_VERSION = '1.0';

// ============================================================
// Default Export
// ============================================================

export default WiaIntent;
