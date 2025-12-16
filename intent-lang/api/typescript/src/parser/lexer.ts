/**
 * WIA-INTENT Lexer
 *
 * 의도 기반 언어의 토크나이저
 */

import { Token, TokenType, ParseError } from '../types';

const KEYWORDS: Record<string, TokenType> = {
  'intent': TokenType.INTENT,
  'desire': TokenType.DESIRE,
  'given': TokenType.GIVEN,
  'want': TokenType.WANT,
  'constraints': TokenType.CONSTRAINTS,
  'certainty': TokenType.CERTAINTY,
  'fallback': TokenType.FALLBACK,
  'evolve': TokenType.EVOLVE,
  'sequence': TokenType.SEQUENCE,
  'parallel': TokenType.PARALLEL,
  'choose': TokenType.CHOOSE,
  'when': TokenType.WHEN,
  'default': TokenType.DEFAULT,
  'if': TokenType.IF,
  'then': TokenType.THEN,
  'else': TokenType.ELSE,
  'with': TokenType.WITH,
  'using': TokenType.USING,
  'collective': TokenType.COLLECTIVE,
  'agents': TokenType.AGENTS,
  'true': TokenType.BOOLEAN,
  'false': TokenType.BOOLEAN,
  'maybe': TokenType.PROBABILITY,
};

export class Lexer {
  private source: string;
  private tokens: Token[] = [];
  private start = 0;
  private current = 0;
  private line = 1;
  private column = 1;

  constructor(source: string) {
    this.source = source;
  }

  tokenize(): Token[] {
    while (!this.isAtEnd()) {
      this.start = this.current;
      this.scanToken();
    }

    this.tokens.push({
      type: TokenType.EOF,
      value: '',
      line: this.line,
      column: this.column,
    });

    return this.tokens;
  }

  private scanToken(): void {
    const char = this.advance();

    switch (char) {
      // Single character tokens
      case '{': this.addToken(TokenType.LBRACE); break;
      case '}': this.addToken(TokenType.RBRACE); break;
      case '(': this.addToken(TokenType.LPAREN); break;
      case ')': this.addToken(TokenType.RPAREN); break;
      case '[': this.addToken(TokenType.LBRACKET); break;
      case ']': this.addToken(TokenType.RBRACKET); break;
      case ':': this.addToken(TokenType.COLON); break;
      case ',': this.addToken(TokenType.COMMA); break;

      // Dot or range
      case '.':
        if (this.match('.')) {
          this.addToken(TokenType.RANGE);
        } else {
          this.addToken(TokenType.DOT);
        }
        break;

      // Comparison operators
      case '>':
        if (this.match('=')) {
          this.addToken(TokenType.GTE);
        } else {
          this.addToken(TokenType.GT);
        }
        break;

      case '<':
        if (this.match('=')) {
          this.addToken(TokenType.LTE);
        } else {
          this.addToken(TokenType.LT);
        }
        break;

      case '=':
        if (this.match('=')) {
          this.addToken(TokenType.EQ);
        }
        break;

      case '!':
        if (this.match('=')) {
          this.addToken(TokenType.NEQ);
        }
        break;

      // Arrow
      case '-':
        if (this.match('>')) {
          this.addToken(TokenType.ARROW);
        } else if (this.isDigit(this.peek())) {
          // Negative number
          this.number();
        }
        break;

      // Comments
      case '/':
        if (this.match('/')) {
          // Single line comment
          while (this.peek() !== '\n' && !this.isAtEnd()) {
            this.advance();
          }
        } else if (this.match('*')) {
          // Multi-line comment
          this.multiLineComment();
        }
        break;

      // Whitespace
      case ' ':
      case '\r':
      case '\t':
        // Ignore whitespace
        break;

      case '\n':
        this.line++;
        this.column = 1;
        break;

      // String
      case '"':
        this.string();
        break;

      default:
        if (this.isDigit(char)) {
          this.number();
        } else if (this.isAlpha(char)) {
          this.identifier();
        } else {
          throw new ParseError(
            `Unexpected character: ${char}`,
            this.line,
            this.column
          );
        }
    }
  }

  private string(): void {
    const startLine = this.line;
    const startColumn = this.column;

    while (this.peek() !== '"' && !this.isAtEnd()) {
      if (this.peek() === '\n') {
        this.line++;
        this.column = 1;
      }
      if (this.peek() === '\\' && this.peekNext() === '"') {
        this.advance(); // Skip escape
      }
      this.advance();
    }

    if (this.isAtEnd()) {
      throw new ParseError('Unterminated string', startLine, startColumn);
    }

    this.advance(); // Closing "

    // Get string value without quotes
    const value = this.source.slice(this.start + 1, this.current - 1);
    this.addToken(TokenType.STRING, value);
  }

  private number(): void {
    while (this.isDigit(this.peek())) {
      this.advance();
    }

    // Decimal part
    if (this.peek() === '.' && this.isDigit(this.peekNext())) {
      this.advance(); // Consume '.'
      while (this.isDigit(this.peek())) {
        this.advance();
      }
    }

    // Scientific notation
    if (this.peek() === 'e' || this.peek() === 'E') {
      this.advance();
      if (this.peek() === '+' || this.peek() === '-') {
        this.advance();
      }
      while (this.isDigit(this.peek())) {
        this.advance();
      }
    }

    this.addToken(TokenType.NUMBER);
  }

  private identifier(): void {
    while (this.isAlphaNumeric(this.peek())) {
      this.advance();
    }

    const text = this.source.slice(this.start, this.current);
    const type = KEYWORDS[text.toLowerCase()] || TokenType.IDENTIFIER;

    // Handle probability literals like maybe(0.7)
    if (text.toLowerCase() === 'maybe' && this.peek() === '(') {
      this.addToken(TokenType.PROBABILITY, text);
      return;
    }

    this.addToken(type, text);
  }

  private multiLineComment(): void {
    let depth = 1;
    while (depth > 0 && !this.isAtEnd()) {
      if (this.peek() === '/' && this.peekNext() === '*') {
        this.advance();
        this.advance();
        depth++;
      } else if (this.peek() === '*' && this.peekNext() === '/') {
        this.advance();
        this.advance();
        depth--;
      } else {
        if (this.peek() === '\n') {
          this.line++;
          this.column = 1;
        }
        this.advance();
      }
    }
  }

  // Helper methods
  private isAtEnd(): boolean {
    return this.current >= this.source.length;
  }

  private advance(): string {
    const char = this.source[this.current];
    this.current++;
    this.column++;
    return char;
  }

  private peek(): string {
    if (this.isAtEnd()) return '\0';
    return this.source[this.current];
  }

  private peekNext(): string {
    if (this.current + 1 >= this.source.length) return '\0';
    return this.source[this.current + 1];
  }

  private match(expected: string): boolean {
    if (this.isAtEnd()) return false;
    if (this.source[this.current] !== expected) return false;
    this.current++;
    this.column++;
    return true;
  }

  private isDigit(char: string): boolean {
    return char >= '0' && char <= '9';
  }

  private isAlpha(char: string): boolean {
    return (
      (char >= 'a' && char <= 'z') ||
      (char >= 'A' && char <= 'Z') ||
      char === '_'
    );
  }

  private isAlphaNumeric(char: string): boolean {
    return this.isAlpha(char) || this.isDigit(char);
  }

  private addToken(type: TokenType, value?: string): void {
    const text = value ?? this.source.slice(this.start, this.current);
    this.tokens.push({
      type,
      value: text,
      line: this.line,
      column: this.column - text.length,
    });
  }
}
