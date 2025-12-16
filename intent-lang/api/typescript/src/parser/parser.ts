/**
 * WIA-INTENT Parser
 *
 * 의도 기반 언어의 파서 (Recursive Descent)
 */

import {
  Token,
  TokenType,
  ParseError,
  ProgramNode,
  IntentNode,
  DesireNode,
  GivenNode,
  WantNode,
  ConstraintsNode,
  ConstraintNode,
  CertaintyNode,
  FallbackNode,
  EvolveNode,
  SequenceNode,
  ParallelNode,
  ChooseNode,
  WhenNode,
  CollectiveNode,
  AgentNode,
  ExpressionNode,
  LiteralNode,
  IdentifierNode,
  IntentCallNode,
  ListNode,
  MapNode,
  RangeNode,
  TypeExpression,
  MemberAccessNode,
} from '../types';

export class Parser {
  private tokens: Token[];
  private current = 0;

  constructor(tokens: Token[]) {
    this.tokens = tokens;
  }

  parse(): ProgramNode {
    const declarations: (IntentNode | DesireNode | CollectiveNode)[] = [];

    while (!this.isAtEnd()) {
      const decl = this.declaration();
      if (decl) {
        declarations.push(decl);
      }
    }

    return {
      type: 'Program',
      declarations,
    };
  }

  private declaration(): IntentNode | DesireNode | CollectiveNode | null {
    if (this.match(TokenType.INTENT)) {
      return this.intentDeclaration();
    }
    if (this.match(TokenType.DESIRE)) {
      return this.desireDeclaration();
    }
    if (this.match(TokenType.COLLECTIVE)) {
      return this.collectiveDeclaration();
    }

    // Skip unknown tokens
    if (!this.isAtEnd()) {
      this.advance();
    }
    return null;
  }

  // ============================================================
  // Intent Declaration
  // ============================================================

  private intentDeclaration(): IntentNode {
    const name = this.consume(TokenType.IDENTIFIER, 'Expected intent name').value;
    this.consume(TokenType.LBRACE, 'Expected "{" after intent name');

    const intent: IntentNode = {
      type: 'Intent',
      name,
      want: { type: 'Want', outputs: {} },
    };

    while (!this.check(TokenType.RBRACE) && !this.isAtEnd()) {
      if (this.match(TokenType.GIVEN)) {
        this.consume(TokenType.COLON, 'Expected ":" after "given"');
        intent.given = this.parseGiven();
      } else if (this.match(TokenType.WANT)) {
        this.consume(TokenType.COLON, 'Expected ":" after "want"');
        intent.want = this.parseWant();
      } else if (this.match(TokenType.CONSTRAINTS)) {
        intent.constraints = this.parseConstraints();
      } else if (this.match(TokenType.CERTAINTY)) {
        this.consume(TokenType.COLON, 'Expected ":" after "certainty"');
        intent.certainty = this.parseCertainty();
      } else if (this.match(TokenType.FALLBACK)) {
        intent.fallback = this.parseFallback();
      } else if (this.match(TokenType.EVOLVE)) {
        intent.evolve = this.parseEvolve();
      } else if (this.match(TokenType.SEQUENCE)) {
        if (!intent.body) intent.body = [];
        intent.body.push(this.parseSequence());
      } else if (this.match(TokenType.PARALLEL)) {
        if (!intent.body) intent.body = [];
        intent.body.push(this.parseParallel());
      } else if (this.match(TokenType.CHOOSE)) {
        if (!intent.body) intent.body = [];
        intent.body.push(this.parseChoose());
      } else {
        this.advance();
      }
    }

    this.consume(TokenType.RBRACE, 'Expected "}" after intent body');
    return intent;
  }

  private parseGiven(): GivenNode {
    const inputs: Record<string, TypeExpression> = {};

    if (this.check(TokenType.LBRACE)) {
      this.advance();
      while (!this.check(TokenType.RBRACE) && !this.isAtEnd()) {
        const name = this.consume(TokenType.IDENTIFIER, 'Expected input name').value;
        this.consume(TokenType.COLON, 'Expected ":"');
        inputs[name] = this.parseTypeExpression();
        this.match(TokenType.COMMA);
      }
      this.consume(TokenType.RBRACE, 'Expected "}"');
    } else {
      // Single line: given: input1, input2
      do {
        const name = this.consume(TokenType.IDENTIFIER, 'Expected input name').value;
        inputs[name] = { baseType: 'any' };
      } while (this.match(TokenType.COMMA));
    }

    return { type: 'Given', inputs };
  }

  private parseWant(): WantNode {
    const outputs: Record<string, TypeExpression> = {};

    if (this.check(TokenType.LBRACE)) {
      this.advance();
      while (!this.check(TokenType.RBRACE) && !this.isAtEnd()) {
        const name = this.consume(TokenType.IDENTIFIER, 'Expected output name').value;
        this.consume(TokenType.COLON, 'Expected ":"');
        outputs[name] = this.parseTypeExpression();
        this.match(TokenType.COMMA);
      }
      this.consume(TokenType.RBRACE, 'Expected "}"');
    } else {
      // Single expression
      const expr = this.expression();
      outputs['result'] = { baseType: 'inferred' };
    }

    return { type: 'Want', outputs };
  }

  private parseConstraints(): ConstraintsNode {
    this.consume(TokenType.LBRACE, 'Expected "{" after "constraints"');
    const constraints: ConstraintNode[] = [];

    while (!this.check(TokenType.RBRACE) && !this.isAtEnd()) {
      const name = this.consume(TokenType.IDENTIFIER, 'Expected constraint name').value;

      let operator: '>=' | '<=' | '==' | '!=' | ':' = ':';
      if (this.match(TokenType.GTE)) operator = '>=';
      else if (this.match(TokenType.LTE)) operator = '<=';
      else if (this.match(TokenType.EQ)) operator = '==';
      else if (this.match(TokenType.NEQ)) operator = '!=';
      else this.consume(TokenType.COLON, 'Expected operator or ":"');

      const value = this.expression();

      constraints.push({
        type: 'Constraint',
        name,
        operator,
        value,
      });
    }

    this.consume(TokenType.RBRACE, 'Expected "}" after constraints');
    return { type: 'Constraints', constraints };
  }

  private parseCertainty(): CertaintyNode {
    let operator: '>=' | '<=' = '>=';
    if (this.match(TokenType.GTE)) operator = '>=';
    else if (this.match(TokenType.LTE)) operator = '<=';

    const threshold = parseFloat(
      this.consume(TokenType.NUMBER, 'Expected certainty threshold').value
    );

    return { type: 'Certainty', operator, threshold };
  }

  private parseFallback(): FallbackNode {
    this.consume(TokenType.LBRACE, 'Expected "{" after "fallback"');
    const rules: { condition: ExpressionNode; action: ExpressionNode }[] = [];

    while (!this.check(TokenType.RBRACE) && !this.isAtEnd()) {
      if (this.match(TokenType.IF)) {
        const condition = this.expression();
        this.consume(TokenType.COLON, 'Expected ":"');
        const action = this.expression();
        rules.push({ condition, action });
      } else {
        this.advance();
      }
    }

    this.consume(TokenType.RBRACE, 'Expected "}" after fallback');
    return { type: 'Fallback', rules };
  }

  private parseEvolve(): EvolveNode {
    this.consume(TokenType.LBRACE, 'Expected "{" after "evolve"');
    const evolve: EvolveNode = { type: 'Evolve' };

    while (!this.check(TokenType.RBRACE) && !this.isAtEnd()) {
      const key = this.consume(TokenType.IDENTIFIER, 'Expected evolve property').value;
      this.consume(TokenType.COLON, 'Expected ":"');

      switch (key) {
        case 'learn_from':
          evolve.learnFrom = this.parseStringList();
          break;
        case 'adapt_to':
          evolve.adaptTo = this.parseStringList();
          break;
        case 'improve':
          evolve.improve = this.parseImproveBlock();
          break;
        case 'boundaries':
          evolve.boundaries = this.parseBoundariesBlock();
          break;
        default:
          this.expression(); // Skip unknown
      }
    }

    this.consume(TokenType.RBRACE, 'Expected "}" after evolve');
    return evolve;
  }

  private parseStringList(): string[] {
    const items: string[] = [];
    if (this.check(TokenType.LBRACKET)) {
      this.advance();
      while (!this.check(TokenType.RBRACKET) && !this.isAtEnd()) {
        items.push(this.consume(TokenType.IDENTIFIER, 'Expected identifier').value);
        this.match(TokenType.COMMA);
      }
      this.consume(TokenType.RBRACKET, 'Expected "]"');
    } else {
      items.push(this.consume(TokenType.IDENTIFIER, 'Expected identifier').value);
    }
    return items;
  }

  private parseImproveBlock(): { metric: string; method: string; rate: string } {
    this.consume(TokenType.LBRACE, 'Expected "{"');
    const result = { metric: '', method: '', rate: '' };

    while (!this.check(TokenType.RBRACE) && !this.isAtEnd()) {
      const key = this.consume(TokenType.IDENTIFIER, 'Expected key').value;
      this.consume(TokenType.COLON, 'Expected ":"');
      const value = this.consume(TokenType.IDENTIFIER, 'Expected value').value;

      if (key === 'metric') result.metric = value;
      else if (key === 'method') result.method = value;
      else if (key === 'rate') result.rate = value;
    }

    this.consume(TokenType.RBRACE, 'Expected "}"');
    return result;
  }

  private parseBoundariesBlock(): { never: string[]; always: string[] } {
    this.consume(TokenType.LBRACE, 'Expected "{"');
    const result = { never: [] as string[], always: [] as string[] };

    while (!this.check(TokenType.RBRACE) && !this.isAtEnd()) {
      const key = this.consume(TokenType.IDENTIFIER, 'Expected key').value;
      this.consume(TokenType.COLON, 'Expected ":"');

      if (key === 'never') result.never = this.parseStringList();
      else if (key === 'always') result.always = this.parseStringList();
    }

    this.consume(TokenType.RBRACE, 'Expected "}"');
    return result;
  }

  // ============================================================
  // Control Flow
  // ============================================================

  private parseSequence(): SequenceNode {
    this.consume(TokenType.LBRACE, 'Expected "{"');
    const steps: IntentCallNode[] = [];

    while (!this.check(TokenType.RBRACE) && !this.isAtEnd()) {
      // Parse step number (optional): "1."
      if (this.check(TokenType.NUMBER)) {
        this.advance();
        this.match(TokenType.DOT);
      }
      steps.push(this.parseIntentCall());
    }

    this.consume(TokenType.RBRACE, 'Expected "}"');
    return { type: 'Sequence', steps };
  }

  private parseParallel(): ParallelNode {
    this.consume(TokenType.LBRACE, 'Expected "{"');
    const branches: IntentCallNode[] = [];

    while (!this.check(TokenType.RBRACE) && !this.isAtEnd()) {
      branches.push(this.parseIntentCall());
    }

    this.consume(TokenType.RBRACE, 'Expected "}"');
    return { type: 'Parallel', branches };
  }

  private parseChoose(): ChooseNode {
    this.consume(TokenType.LBRACE, 'Expected "{"');
    const conditions: WhenNode[] = [];
    let defaultBranch: IntentCallNode | undefined;

    while (!this.check(TokenType.RBRACE) && !this.isAtEnd()) {
      if (this.match(TokenType.WHEN)) {
        const condition = this.expression();
        this.consume(TokenType.COLON, 'Expected ":"');
        const action = this.parseIntentCall();
        conditions.push({ type: 'When', condition, action });
      } else if (this.match(TokenType.DEFAULT)) {
        this.consume(TokenType.COLON, 'Expected ":"');
        defaultBranch = this.parseIntentCall();
      } else {
        this.advance();
      }
    }

    this.consume(TokenType.RBRACE, 'Expected "}"');
    return { type: 'Choose', conditions, defaultBranch };
  }

  private parseIntentCall(): IntentCallNode {
    const intentName = this.consume(TokenType.IDENTIFIER, 'Expected intent name').value;
    const args: Record<string, ExpressionNode> = {};

    if (this.match(TokenType.LBRACE)) {
      while (!this.check(TokenType.RBRACE) && !this.isAtEnd()) {
        const key = this.consume(TokenType.IDENTIFIER, 'Expected argument name').value;
        this.consume(TokenType.COLON, 'Expected ":"');
        args[key] = this.expression();
        this.match(TokenType.COMMA);
      }
      this.consume(TokenType.RBRACE, 'Expected "}"');
    }

    return { type: 'IntentCall', intentName, args };
  }

  // ============================================================
  // Desire Declaration
  // ============================================================

  private desireDeclaration(): DesireNode {
    const name = this.consume(TokenType.IDENTIFIER, 'Expected desire name').value;
    this.consume(TokenType.LBRACE, 'Expected "{"');

    const desire: DesireNode = {
      type: 'Desire',
      name,
      ultimateGoal: '',
      indicators: {},
      approach: {},
      ethics: {},
    };

    while (!this.check(TokenType.RBRACE) && !this.isAtEnd()) {
      const key = this.consume(TokenType.IDENTIFIER, 'Expected property').value;
      this.consume(TokenType.COLON, 'Expected ":"');

      if (key === 'ultimate_goal') {
        desire.ultimateGoal = this.consume(TokenType.STRING, 'Expected string').value;
      } else if (key === 'indicators') {
        desire.indicators = this.parseExpressionMap();
      } else if (key === 'approach') {
        desire.approach = this.parseExpressionMap();
      } else if (key === 'ethics') {
        desire.ethics = this.parseExpressionMap();
      }
    }

    this.consume(TokenType.RBRACE, 'Expected "}"');
    return desire;
  }

  private parseExpressionMap(): Record<string, ExpressionNode> {
    const map: Record<string, ExpressionNode> = {};
    this.consume(TokenType.LBRACE, 'Expected "{"');

    while (!this.check(TokenType.RBRACE) && !this.isAtEnd()) {
      const key = this.consume(TokenType.IDENTIFIER, 'Expected key').value;
      this.consume(TokenType.COLON, 'Expected ":"');
      map[key] = this.expression();
      this.match(TokenType.COMMA);
    }

    this.consume(TokenType.RBRACE, 'Expected "}"');
    return map;
  }

  // ============================================================
  // Collective Declaration
  // ============================================================

  private collectiveDeclaration(): CollectiveNode {
    const name = this.consume(TokenType.IDENTIFIER, 'Expected collective name').value;
    this.consume(TokenType.LBRACE, 'Expected "{"');

    const collective: CollectiveNode = {
      type: 'Collective',
      name,
      agents: [],
      coordinate: [],
    };

    while (!this.check(TokenType.RBRACE) && !this.isAtEnd()) {
      if (this.match(TokenType.AGENTS)) {
        this.consume(TokenType.LBRACE, 'Expected "{"');
        while (!this.check(TokenType.RBRACE) && !this.isAtEnd()) {
          collective.agents.push(this.parseAgent());
        }
        this.consume(TokenType.RBRACE, 'Expected "}"');
      } else {
        this.advance();
      }
    }

    this.consume(TokenType.RBRACE, 'Expected "}"');
    return collective;
  }

  private parseAgent(): AgentNode {
    const name = this.consume(TokenType.IDENTIFIER, 'Expected agent name').value;
    const agent: AgentNode = { type: 'Agent', name };

    if (this.match(TokenType.LBRACE)) {
      while (!this.check(TokenType.RBRACE) && !this.isAtEnd()) {
        const key = this.consume(TokenType.IDENTIFIER, 'Expected property').value;
        this.consume(TokenType.COLON, 'Expected ":"');

        if (key === 'role') {
          agent.role = this.consume(TokenType.IDENTIFIER, 'Expected role').value;
        } else if (key === 'specializes_in') {
          agent.specializesIn = this.consume(TokenType.IDENTIFIER, 'Expected specialization').value;
        }
      }
      this.consume(TokenType.RBRACE, 'Expected "}"');
    }

    return agent;
  }

  // ============================================================
  // Expressions
  // ============================================================

  private expression(): ExpressionNode {
    return this.comparison();
  }

  private comparison(): ExpressionNode {
    let expr = this.term();

    while (
      this.match(TokenType.GTE) ||
      this.match(TokenType.LTE) ||
      this.match(TokenType.GT) ||
      this.match(TokenType.LT) ||
      this.match(TokenType.EQ) ||
      this.match(TokenType.NEQ)
    ) {
      const operator = this.previous().value;
      const right = this.term();
      expr = {
        type: 'BinaryOp',
        operator,
        left: expr,
        right,
      };
    }

    return expr;
  }

  private term(): ExpressionNode {
    return this.primary();
  }

  private primary(): ExpressionNode {
    // Number
    if (this.check(TokenType.NUMBER)) {
      const value = parseFloat(this.advance().value);

      // Check for range (10..20)
      if (this.match(TokenType.RANGE)) {
        const end = parseFloat(this.consume(TokenType.NUMBER, 'Expected end of range').value);
        let confidence: number | undefined;

        if (this.match(TokenType.IDENTIFIER) && this.previous().value === 'with_confidence') {
          confidence = parseFloat(this.consume(TokenType.NUMBER, 'Expected confidence').value);
        }

        return { type: 'Range', start: value, end, confidence };
      }

      return { type: 'Literal', valueType: 'number', value };
    }

    // String
    if (this.check(TokenType.STRING)) {
      return {
        type: 'Literal',
        valueType: 'string',
        value: this.advance().value,
      };
    }

    // Boolean
    if (this.check(TokenType.BOOLEAN)) {
      return {
        type: 'Literal',
        valueType: 'boolean',
        value: this.advance().value === 'true',
      };
    }

    // List
    if (this.match(TokenType.LBRACKET)) {
      const elements: ExpressionNode[] = [];
      while (!this.check(TokenType.RBRACKET) && !this.isAtEnd()) {
        elements.push(this.expression());
        this.match(TokenType.COMMA);
      }
      this.consume(TokenType.RBRACKET, 'Expected "]"');
      return { type: 'List', elements };
    }

    // Identifier or member access
    if (this.check(TokenType.IDENTIFIER)) {
      let expr: ExpressionNode = {
        type: 'Identifier',
        name: this.advance().value,
      };

      // Handle member access (obj.prop)
      while (this.match(TokenType.DOT)) {
        const member = this.consume(TokenType.IDENTIFIER, 'Expected property name').value;
        expr = {
          type: 'MemberAccess',
          object: expr,
          member,
        };
      }

      return expr;
    }

    // Grouped expression
    if (this.match(TokenType.LPAREN)) {
      const expr = this.expression();
      this.consume(TokenType.RPAREN, 'Expected ")"');
      return expr;
    }

    throw new ParseError(
      `Unexpected token: ${this.peek().value}`,
      this.peek().line,
      this.peek().column
    );
  }

  private parseTypeExpression(): TypeExpression {
    const baseType = this.consume(TokenType.IDENTIFIER, 'Expected type').value;
    const typeExpr: TypeExpression = { baseType };

    // Generic types: list<string>
    if (this.match(TokenType.LT)) {
      typeExpr.genericArgs = [];
      do {
        typeExpr.genericArgs.push(this.parseTypeExpression());
      } while (this.match(TokenType.COMMA));
      this.consume(TokenType.GT, 'Expected ">"');
    }

    // Optional: type?
    if (this.peek().value === '?') {
      this.advance();
      typeExpr.isOptional = true;
    }

    return typeExpr;
  }

  // ============================================================
  // Helper Methods
  // ============================================================

  private match(...types: TokenType[]): boolean {
    for (const type of types) {
      if (this.check(type)) {
        this.advance();
        return true;
      }
    }
    return false;
  }

  private check(type: TokenType): boolean {
    if (this.isAtEnd()) return false;
    return this.peek().type === type;
  }

  private advance(): Token {
    if (!this.isAtEnd()) this.current++;
    return this.previous();
  }

  private isAtEnd(): boolean {
    return this.peek().type === TokenType.EOF;
  }

  private peek(): Token {
    return this.tokens[this.current];
  }

  private previous(): Token {
    return this.tokens[this.current - 1];
  }

  private consume(type: TokenType, message: string): Token {
    if (this.check(type)) return this.advance();
    throw new ParseError(message, this.peek().line, this.peek().column);
  }
}
