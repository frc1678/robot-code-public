The EBNF grammar for Lemonscript expressions is below:

```
expression = prefix_expression [binary_expressions];
parenthesized_expression = "(" expression {"," expression} ")";

prefix_expression = [prefix_operator] postfix_expression;

binary_expression = binary_operator prefix_expression;
binary_expressions = binary_expression [binary_expressions];

// postfix_expression = primary_expression | postfix_expression postfix_operator;
postfix_expression = primary_expression {postfix_operator};

primary_expression = identifier | literal | parenthesized_expression;

prefix_operator = operator;
binary_operator = operator;
postfix_operator = operator;

operator = operator_character {operator_character};
operator_character = "/" | "*" | "=" | "-" | "+" | "!" | "%" | "<" | ">" | "&" | "|" | "^";

identifier = alpha {identifier_character};
identifier_character = alpha | digit;

literal = numeric_literal | boolean_literal;
boolean_literal = "true" | "false";
numeric_literal = ["-"] integer_literal | ["-"] floating_point_literal;

integer_literal = decimal_literal;
floating_point_literal = decimal_literal [decimal_fraction];
decimal_fraction = "." decimal_literal;
decimal_literal = digit {digit};

alpha = "a".."z" | "A".."Z";
digit = "0".."9";

```

The tokens are:
```
"(", ",", ")", "true", "false", digit, alpha, "/", "*", "=", "-", "+", "!", "%", "<", ">", "&", "|", "^"
```

The first sets are:
```
expression = prefix_expression
parenthesized_expression = "("

prefix_expression = prefix_operator

binary_expression = binary_operator
binary_expressions = binary_expression

postfix_expression = primary_expression
function_call_expression = postfix_expression

primary_expression = identifier | literal | parenthesized_expression

prefix_operator = operator
binary_operator = operator
postfix_operator = operator

operator = operator_character
operator_character = "/" | "*" | "=" | "-" | "+" | "!" | "%" | "<" | ">" | "&" | "|" | "^"

identifier = alpha
identifier_character = alpha | digit

literal = numeric_literal | boolean_literal
boolean_literal = "true" | "false"
numeric_literal = ["-"] | integer_literal | floating_point_literal

integer_literal = decimal_literal
floating_point_literal = decimal_literal
decimal_fraction = "."
decimal_literal = digit

alpha = "a".."z" | "A".."Z"
digit = "0".."9"

```