/**
 * arith-expr — lightweight arithmetic expression parser/evaluator.
 *
 * Supports + - * / ^ (with standard precedence; ^ is right-associative),
 * parentheses, floating point numbers (including scientific notation), and
 * {variableName} references resolved through a pluggable VariableProvider.
 *
 * Grammar (precedence low -> high):
 *   expression := term (("+" | "-") term)*
 *   term       := unary (("*" | "/") unary)*
 *   unary      := ("+" | "-") unary | power
 *   power      := primary ("^" unary)?
 *   primary    := NUMBER | VARIABLE | "(" expression ")"
 *
 * `^` is right-associative (2^3^2 == 2^(3^2) == 512), and unary minus binds
 * looser than `^` on its operand, matching conventional math notation
 * (-2^2 == -4, not 4).
 *
 * Single-file edition — drop this anywhere in your project and import from
 * it directly. Everything below is self-contained; no external dependencies.
 */

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

/** Thrown for lexical or grammatical problems while parsing an expression string. */
export class ExpressionSyntaxError extends Error {
  public readonly position?: number;

  constructor(message: string, position?: number) {
    super(message);
    this.name = "ExpressionSyntaxError";
    this.position = position;
    Object.setPrototypeOf(this, ExpressionSyntaxError.prototype);
  }
}

/** Thrown for problems that occur while evaluating an already-parsed AST. */
export class EvaluationError extends Error {
  constructor(message: string) {
    super(message);
    this.name = "EvaluationError";
    Object.setPrototypeOf(this, EvaluationError.prototype);
  }
}

// ---------------------------------------------------------------------------
// AST
// ---------------------------------------------------------------------------

export type BinaryOperator = "+" | "-" | "*" | "/" | "^";
export type UnaryOperator = "+" | "-";

export interface NumberNode {
  kind: "Number";
  value: number;
}

export interface VariableNode {
  kind: "Variable";
  name: string;
}

export interface UnaryOpNode {
  kind: "UnaryOp";
  operator: UnaryOperator;
  operand: ASTNode;
}

export interface BinaryOpNode {
  kind: "BinaryOp";
  operator: BinaryOperator;
  left: ASTNode;
  right: ASTNode;
}

export type ASTNode = NumberNode | VariableNode | UnaryOpNode | BinaryOpNode;

// ---------------------------------------------------------------------------
// Lexer
// ---------------------------------------------------------------------------

export enum TokenType {
  Number = "Number",
  Plus = "Plus",
  Minus = "Minus",
  Star = "Star",
  Slash = "Slash",
  Caret = "Caret",
  LParen = "LParen",
  RParen = "RParen",
  Variable = "Variable",
  EOF = "EOF",
}

export interface Token {
  type: TokenType;
  /** Raw text for operators/parens; numeric literal text for Number; variable name for Variable. */
  value: string;
  /** Character offset in the source string where this token starts. */
  position: number;
}

// Matches integers, decimals, and an optional exponent: 3, 3.14, .5, 2.5e-3
const NUMBER_RE = /^(\d+\.\d+|\.\d+|\d+)([eE][+-]?\d+)?/;

const SINGLE_CHAR_TOKENS: Partial<Record<string, TokenType>> = {
  "+": TokenType.Plus,
  "-": TokenType.Minus,
  "*": TokenType.Star,
  "/": TokenType.Slash,
  "^": TokenType.Caret,
  "(": TokenType.LParen,
  ")": TokenType.RParen,
};

/**
 * Converts a raw expression string into a flat list of tokens, ending in EOF.
 * Throws ExpressionSyntaxError on unrecognized characters, malformed numbers,
 * or unterminated `{...}` variable references.
 */
export function tokenize(input: string): Token[] {
  const tokens: Token[] = [];
  const n = input.length;
  let i = 0;

  while (i < n) {
    const ch = input[i];

    if (ch === " " || ch === "\t" || ch === "\n" || ch === "\r") {
      i++;
      continue;
    }

    const simpleType = SINGLE_CHAR_TOKENS[ch];
    if (simpleType !== undefined) {
      tokens.push({ type: simpleType, value: ch, position: i });
      i++;
      continue;
    }

    if (ch === "{") {
      const start = i;
      let j = i + 1;
      while (j < n && input[j] !== "}") j++;
      if (j >= n) {
        throw new ExpressionSyntaxError(
          `Unterminated variable reference starting at position ${start}`,
          start
        );
      }
      const name = input.slice(i + 1, j).trim();
      if (name.length === 0) {
        throw new ExpressionSyntaxError(`Empty variable name at position ${start}`, start);
      }
      tokens.push({ type: TokenType.Variable, value: name, position: start });
      i = j + 1;
      continue;
    }

    if ((ch >= "0" && ch <= "9") || ch === ".") {
      const match = NUMBER_RE.exec(input.slice(i));
      if (!match) {
        throw new ExpressionSyntaxError(`Invalid number literal at position ${i}`, i);
      }
      tokens.push({ type: TokenType.Number, value: match[0], position: i });
      i += match[0].length;
      continue;
    }

    throw new ExpressionSyntaxError(`Unexpected character '${ch}' at position ${i}`, i);
  }

  tokens.push({ type: TokenType.EOF, value: "", position: n });
  return tokens;
}

// ---------------------------------------------------------------------------
// Parser
// ---------------------------------------------------------------------------

/**
 * Parses an arithmetic expression string into an AST.
 *
 * Supports +, -, *, /, ^ (right-associative), parentheses, floating point
 * numbers, and {variableName} references. Throws ExpressionSyntaxError on
 * malformed input (unexpected tokens, unbalanced parens, trailing input).
 */
export function parse(input: string): ASTNode {
  const tokens = tokenize(input);
  let pos = 0;

  const peek = (): Token => tokens[pos];
  const advance = (): Token => tokens[pos++];
  const check = (type: TokenType): boolean => peek().type === type;

  function expect(type: TokenType, message: string): Token {
    if (!check(type)) {
      const t = peek();
      const found = t.type === TokenType.EOF ? "end of input" : `'${t.value}'`;
      throw new ExpressionSyntaxError(`${message}, but found ${found} at position ${t.position}`, t.position);
    }
    return advance();
  }

  function parseExpression(): ASTNode {
    let node = parseTerm();
    while (check(TokenType.Plus) || check(TokenType.Minus)) {
      const opToken = advance();
      const operator: BinaryOperator = opToken.type === TokenType.Plus ? "+" : "-";
      node = { kind: "BinaryOp", operator, left: node, right: parseTerm() };
    }
    return node;
  }

  function parseTerm(): ASTNode {
    let node = parseUnary();
    while (check(TokenType.Star) || check(TokenType.Slash)) {
      const opToken = advance();
      const operator: BinaryOperator = opToken.type === TokenType.Star ? "*" : "/";
      node = { kind: "BinaryOp", operator, left: node, right: parseUnary() };
    }
    return node;
  }

  function parseUnary(): ASTNode {
    if (check(TokenType.Plus) || check(TokenType.Minus)) {
      const opToken = advance();
      return {
        kind: "UnaryOp",
        operator: opToken.type === TokenType.Plus ? "+" : "-",
        operand: parseUnary(),
      };
    }
    return parsePower();
  }

  function parsePower(): ASTNode {
    const base = parsePrimary();
    if (check(TokenType.Caret)) {
      advance();
      // Recursing through parseUnary (rather than parsePower) lets exponents
      // carry their own sign, e.g. 2^-2, while the chained parsePower call
      // inside that recursion still yields right-associativity for 2^3^2.
      const exponent = parseUnary();
      return { kind: "BinaryOp", operator: "^", left: base, right: exponent };
    }
    return base;
  }

  function parsePrimary(): ASTNode {
    const t = peek();

    if (t.type === TokenType.Number) {
      advance();
      return { kind: "Number", value: parseFloat(t.value) };
    }

    if (t.type === TokenType.Variable) {
      advance();
      return { kind: "Variable", name: t.value };
    }

    if (t.type === TokenType.LParen) {
      advance();
      const inner = parseExpression();
      expect(TokenType.RParen, "Expected closing parenthesis");
      return inner;
    }

    const found = t.type === TokenType.EOF ? "end of input" : `'${t.value}'`;
    throw new ExpressionSyntaxError(
      `Expected a number, variable, or '(' but found ${found} at position ${t.position}`,
      t.position
    );
  }

  const ast = parseExpression();
  expect(TokenType.EOF, "Unexpected trailing input");
  return ast;
}

// ---------------------------------------------------------------------------
// Evaluator
// ---------------------------------------------------------------------------

/**
 * The core interface for supplying variable values to an evaluation.
 * Implement this directly for custom sources (databases, caches, live
 * sensors, etc). For convenience, evaluate()/Expression#evaluate() also
 * accept a plain object, a Map, or a lookup function, and will normalize
 * them into this interface automatically (see toVariableProvider).
 */
export interface VariableProvider {
  get(name: string): number;
}

/** Anything that can be normalized into a VariableProvider. */
export type VariableSource =
  | VariableProvider
  | Record<string, number>
  | Map<string, number>
  | ((name: string) => number);

function isVariableProvider(source: VariableSource): source is VariableProvider {
  return (
    typeof source === "object" &&
    source !== null &&
    !(source instanceof Map) &&
    typeof (source as VariableProvider).get === "function"
  );
}

/** Normalizes any supported VariableSource into a VariableProvider. */
export function toVariableProvider(source: VariableSource): VariableProvider {
  if (typeof source === "function") {
    return { get: source };
  }
  if (source instanceof Map) {
    return {
      get(name: string): number {
        if (!source.has(name)) {
          throw new EvaluationError(`Missing value for variable '${name}'`);
        }
        return source.get(name) as number;
      },
    };
  }
  if (isVariableProvider(source)) {
    return source;
  }
  const record = source as Record<string, number>;
  return {
    get(name: string): number {
      if (!(name in record)) {
        throw new EvaluationError(`Missing value for variable '${name}'`);
      }
      return record[name];
    },
  };
}

/**
 * Evaluates an AST to a number. `variables` may be omitted only if the
 * expression contains no {variable} references.
 */
export function evaluate(node: ASTNode, variables?: VariableSource): number {
  const provider = variables !== undefined ? toVariableProvider(variables) : undefined;

  function evalNode(n: ASTNode): number {
    switch (n.kind) {
      case "Number":
        return n.value;

      case "Variable": {
        if (!provider) {
          throw new EvaluationError(
            `Expression references variable '${n.name}' but no variable source was supplied`
          );
        }
        return provider.get(n.name);
      }

      case "UnaryOp": {
        const value = evalNode(n.operand);
        return n.operator === "-" ? -value : value;
      }

      case "BinaryOp": {
        const left = evalNode(n.left);
        const right = evalNode(n.right);
        switch (n.operator) {
          case "+":
            return left + right;
          case "-":
            return left - right;
          case "*":
            return left * right;
          case "/":
            if (right === 0) {
              throw new EvaluationError("Division by zero");
            }
            return left / right;
          case "^":
            return Math.pow(left, right);
        }
        // Unreachable: the switch above covers every BinaryOperator.
        throw new EvaluationError(`Unknown operator '${String((n as { operator: unknown }).operator)}'`);
      }

      default:
        // Unreachable: the outer switch covers every ASTNode kind.
        throw new EvaluationError(`Unknown AST node: ${JSON.stringify(n)}`);
    }
  }

  return evalNode(node);
}

// ---------------------------------------------------------------------------
// Expression (parse once, evaluate many times)
// ---------------------------------------------------------------------------

/**
 * A parsed arithmetic expression, ready to be evaluated repeatedly
 * (e.g. against many different variable bindings) without re-parsing.
 *
 * Example:
 *   const expr = new Expression("2 * ({x} + 1) ^ 2 - {y}");
 *   expr.getVariableNames();        // ["x", "y"]
 *   expr.evaluate({ x: 3, y: 5 });  // 27
 */
export class Expression {
  public readonly source: string;
  public readonly ast: ASTNode;

  constructor(source: string) {
    this.source = source;
    this.ast = parse(source);
  }

  /** Evaluates this expression. Omit `variables` only if it has no {var} references. */
  evaluate(variables?: VariableSource): number {
    return evaluate(this.ast, variables);
  }

  /** Distinct variable names referenced by this expression, in order of first appearance. */
  getVariableNames(): string[] {
    const seen = new Set<string>();
    const names: string[] = [];

    const walk = (node: ASTNode): void => {
      switch (node.kind) {
        case "Variable":
          if (!seen.has(node.name)) {
            seen.add(node.name);
            names.push(node.name);
          }
          break;
        case "UnaryOp":
          walk(node.operand);
          break;
        case "BinaryOp":
          walk(node.left);
          walk(node.right);
          break;
        case "Number":
          break;
      }
    };

    walk(this.ast);
    return names;
  }

  toString(): string {
    return this.source;
  }
}

/** Convenience factory, equivalent to `new Expression(source)`. */
export function parseExpression(source: string): Expression {
  return new Expression(source);
}

/** One-shot parse + evaluate, for when you don't need to reuse the AST. */
export function evaluateExpression(source: string, variables?: VariableSource): number {
  return new Expression(source).evaluate(variables);
}
