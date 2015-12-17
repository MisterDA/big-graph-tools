# 1 "src/basicLexer.mll"
 

  (** Basic Lexer for parsing ints, floats, words and strings.  

      '# ... \n' delimit comments.  Use [BasicLexer.int lex, BasicLexer.float,
      ...] when you expect the next token to be an [int, float, ...]. Use
      optional argument [~no_eof:true] to raise an error raser than
      [End_of_file] when no more token can be read. Use [BasicLexer.search_int
      lex, ...] when you want to skip anything until the next [int, ...].  *)

  type token = Int of int | Float of float 
               | String of string | Comment of string | Eof

  open Lexing


# 19 "src/basicLexer.ml"
let __ocaml_lex_tables = {
  Lexing.lex_base = 
   "\000\000\249\255\002\000\010\000\064\000\076\000\001\000\006\000\
    \008\000\015\000\009\000\015\000\016\000\254\255\253\255\091\000\
    \140\000\150\000";
  Lexing.lex_backtrk = 
   "\255\255\255\255\005\000\004\000\003\000\005\000\002\000\002\000\
    \005\000\000\000\001\000\001\000\255\255\255\255\255\255\005\000\
    \004\000\005\000";
  Lexing.lex_default = 
   "\002\000\000\000\002\000\002\000\002\000\002\000\255\255\255\255\
    \008\000\255\255\255\255\255\255\012\000\000\000\000\000\002\000\
    \002\000\002\000";
  Lexing.lex_trans = 
   "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\009\000\006\000\255\255\255\255\007\000\014\000\255\255\
    \014\000\012\000\010\000\255\255\255\255\011\000\013\000\255\255\
    \009\000\013\000\010\000\000\000\000\000\011\000\000\000\000\000\
    \009\000\000\000\255\255\008\000\000\000\000\000\000\000\000\000\
    \012\000\000\000\255\255\005\000\000\000\005\000\003\000\009\000\
    \004\000\004\000\004\000\004\000\004\000\004\000\004\000\004\000\
    \004\000\004\000\003\000\003\000\003\000\003\000\003\000\003\000\
    \003\000\003\000\003\000\003\000\000\000\000\000\000\000\000\000\
    \000\000\255\255\255\255\000\000\000\000\255\255\000\000\015\000\
    \000\000\000\000\000\000\000\000\000\000\255\255\255\255\000\000\
    \000\000\255\255\000\000\000\000\000\000\000\000\000\000\000\000\
    \255\255\000\000\000\000\000\000\255\255\255\255\000\000\000\000\
    \255\255\000\000\000\000\000\000\255\255\000\000\003\000\015\000\
    \004\000\004\000\004\000\004\000\004\000\004\000\004\000\004\000\
    \004\000\004\000\003\000\255\255\004\000\004\000\004\000\004\000\
    \004\000\004\000\004\000\004\000\004\000\004\000\017\000\000\000\
    \017\000\000\000\000\000\016\000\016\000\016\000\016\000\016\000\
    \016\000\016\000\016\000\016\000\016\000\255\255\255\255\000\000\
    \000\000\255\255\000\000\000\000\000\000\000\000\000\000\255\255\
    \255\255\000\000\000\000\255\255\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\255\255\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\255\255\000\000\
    \000\000\000\000\000\000\000\000\016\000\016\000\016\000\016\000\
    \016\000\016\000\016\000\016\000\016\000\016\000\016\000\016\000\
    \016\000\016\000\016\000\016\000\016\000\016\000\016\000\016\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \001\000\000\000\255\255\000\000\000\000\000\000\000\000\000\000\
    \255\255\000\000\255\255\000\000\000\000\000\000\000\000\000\000\
    \255\255\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \255\255\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\255\255\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\255\255\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\255\255\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\255\255";
  Lexing.lex_check = 
   "\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\000\000\000\000\002\000\002\000\000\000\006\000\002\000\
    \007\000\008\000\008\000\003\000\003\000\008\000\010\000\003\000\
    \009\000\011\000\012\000\255\255\255\255\012\000\255\255\255\255\
    \000\000\255\255\002\000\000\000\255\255\255\255\255\255\255\255\
    \008\000\255\255\003\000\000\000\255\255\000\000\000\000\009\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\003\000\003\000\003\000\003\000\003\000\003\000\
    \003\000\003\000\003\000\003\000\255\255\255\255\255\255\255\255\
    \255\255\004\000\004\000\255\255\255\255\004\000\255\255\003\000\
    \255\255\255\255\255\255\255\255\255\255\005\000\005\000\255\255\
    \255\255\005\000\255\255\255\255\255\255\255\255\255\255\255\255\
    \004\000\255\255\255\255\255\255\015\000\015\000\255\255\255\255\
    \015\000\255\255\255\255\255\255\005\000\255\255\004\000\003\000\
    \004\000\004\000\004\000\004\000\004\000\004\000\004\000\004\000\
    \004\000\004\000\005\000\015\000\005\000\005\000\005\000\005\000\
    \005\000\005\000\005\000\005\000\005\000\005\000\015\000\255\255\
    \015\000\255\255\255\255\015\000\015\000\015\000\015\000\015\000\
    \015\000\015\000\015\000\015\000\015\000\016\000\016\000\255\255\
    \255\255\016\000\255\255\255\255\255\255\255\255\255\255\017\000\
    \017\000\255\255\255\255\017\000\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\016\000\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\017\000\255\255\
    \255\255\255\255\255\255\255\255\016\000\016\000\016\000\016\000\
    \016\000\016\000\016\000\016\000\016\000\016\000\017\000\017\000\
    \017\000\017\000\017\000\017\000\017\000\017\000\017\000\017\000\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \000\000\255\255\002\000\255\255\255\255\255\255\255\255\255\255\
    \008\000\255\255\003\000\255\255\255\255\255\255\255\255\255\255\
    \012\000\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \004\000\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\005\000\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\015\000\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\016\000\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\017\000";
  Lexing.lex_base_code = 
   "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\001\000\004\000\000\000\000\000\
    \000\000\000\000";
  Lexing.lex_backtrk_code = 
   "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\004\000\004\000\000\000\000\000\000\000\000\000\
    \000\000\000\000";
  Lexing.lex_default_code = 
   "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \001\000\000\000\000\000\000\000\001\000\000\000\000\000\000\000\
    \000\000\000\000";
  Lexing.lex_trans_code = 
   "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\001\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\
    \000\000\000\000";
  Lexing.lex_check_code = 
   "\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\008\000\012\000\255\255\008\000\012\000\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\000\000\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\255\
    \008\000\012\000";
  Lexing.lex_code = 
   "\255\001\255\255\000\001\255";
}

let rec token lexbuf =
  lexbuf.Lexing.lex_mem <- Array.make 2 (-1) ;   __ocaml_lex_token_rec lexbuf 0
and __ocaml_lex_token_rec lexbuf __ocaml_lex_state =
  match Lexing.new_engine __ocaml_lex_tables __ocaml_lex_state lexbuf with
      | 0 ->
# 27 "src/basicLexer.mll"
    ( token lexbuf )
# 228 "src/basicLexer.ml"

  | 1 ->
let
# 29 "src/basicLexer.mll"
                           c
# 234 "src/basicLexer.ml"
= Lexing.sub_lexeme lexbuf (lexbuf.Lexing.lex_start_pos + 1) lexbuf.Lexing.lex_mem.(0) in
# 30 "src/basicLexer.mll"
    ( new_line lexbuf ; Comment c )
# 238 "src/basicLexer.ml"

  | 2 ->
# 33 "src/basicLexer.mll"
    ( new_line lexbuf ; token lexbuf )
# 243 "src/basicLexer.ml"

  | 3 ->
let
# 35 "src/basicLexer.mll"
                  w
# 249 "src/basicLexer.ml"
= Lexing.sub_lexeme lexbuf lexbuf.Lexing.lex_start_pos lexbuf.Lexing.lex_curr_pos in
# 36 "src/basicLexer.mll"
    ( Int (int_of_string w) )
# 253 "src/basicLexer.ml"

  | 4 ->
let
# 38 "src/basicLexer.mll"
                                                       w
# 259 "src/basicLexer.ml"
= Lexing.sub_lexeme lexbuf lexbuf.Lexing.lex_start_pos lexbuf.Lexing.lex_curr_pos in
# 39 "src/basicLexer.mll"
    ( Float (float_of_string w) )
# 263 "src/basicLexer.ml"

  | 5 ->
let
# 41 "src/basicLexer.mll"
               s
# 269 "src/basicLexer.ml"
= Lexing.sub_lexeme lexbuf lexbuf.Lexing.lex_start_pos lexbuf.Lexing.lex_curr_pos in
# 42 "src/basicLexer.mll"
    ( String s )
# 273 "src/basicLexer.ml"

  | 6 ->
# 45 "src/basicLexer.mll"
    ( raise End_of_file )
# 278 "src/basicLexer.ml"

  | 7 ->
# 47 "src/basicLexer.mll"
    ( (* empty token *) raise End_of_file )
# 283 "src/basicLexer.ml"

  | __ocaml_lex_state -> lexbuf.Lexing.refill_buff lexbuf; 
      __ocaml_lex_token_rec lexbuf __ocaml_lex_state

;;

# 49 "src/basicLexer.mll"
 

  let str_of_tok = function
    | Float f -> Printf.sprintf "float '%f'" f
    | Int i -> Printf.sprintf "int '%d'" i
    | String s -> Printf.sprintf "string '%s'" s
    | Comment s -> Printf.sprintf "comment '%s'" s
    | Eof -> "End_of_file"

  let error lex got expected =
    let pos = lex.lex_curr_p in
    let msg = 
      Printf.sprintf "BasicLexer: \
          got %s instead of %s%s at line %d, before character %d"
        (str_of_tok got) expected
        (if pos.pos_fname = "" then "" else " in " ^ pos.pos_fname)  
        pos.pos_lnum (pos.pos_cnum - pos.pos_bol) 
    in 
    invalid_arg msg
  
  let token ?(no_eof=false) expected lex =
    try 
      token lex 
    with End_of_file -> 
      if no_eof then error lex Eof expected else raise End_of_file


  let rec float ?(no_eof=false) ?(parse_comment=(fun _ -> ())) lex =
    match token ~no_eof "float" lex with
    | Float f -> f
    | Int i -> float_of_int i
    | String _ as tok | (Eof as tok) -> error lex tok "float"
    | Comment s -> parse_comment s ; float ~no_eof ~parse_comment lex

  let rec int ?(no_eof=false) ?(parse_comment=(fun _ -> ())) lex = 
    match token ~no_eof:no_eof "int" lex with
    | Int i -> i
    | Float _ as tok | (String _ as tok) | (Eof as tok) -> error lex tok "int"
    | Comment s -> parse_comment s ; int ~no_eof ~parse_comment lex

  let rec string ?(no_eof=false) ?(parse_comment=(fun _ -> ())) lex = 
    match token ~no_eof "string" lex with
    | Float f -> string_of_float f
    | Int i -> string_of_int i
    | String s -> s
    | Eof as tok -> error lex tok "string"
    | Comment s -> parse_comment s ; string ~no_eof ~parse_comment lex


  let rec search_float ?(no_eof=false) lex =
    match token ~no_eof "float" lex with
    | Float f -> f
    | Int i -> float_of_int i
    | _ -> search_float ~no_eof lex

  let rec search_int ?(no_eof=false) lex = 
    match token ~no_eof:no_eof "int" lex with
    | Int i -> i
    | _ -> search_int ~no_eof lex

  let rec search_string ?(no_eof=false) lex = 
    match token ~no_eof "string" lex with
    | Float f -> string_of_float f
    | Int i -> string_of_int i
    | String s -> s
    | _ -> search_string ~no_eof lex


  let token ?(no_eof=false) lex = token ~no_eof "token" lex


  let iter f lex =
    try while true do 
      f (token lex) 
    done with End_of_file ->
    ()

  let iter_int f lex =
    try while true do 
      f (int lex) 
    done with End_of_file ->
    ()


  let test () = 
    let lex = Lexing.from_channel stdin in
    iter (fun t -> Printf.printf "%s\n" (str_of_tok t) ; flush stdout) lex

  let unit () =
    (* let lex = Lexing.from_channel stdin in *)
    let lex = Lexing.from_string "lqkdj 123 i? 12ZER -.2e+127 -lsdkjf.sldkf" in
    iter (fun t -> Printf.printf "%s\n" (str_of_tok t) ; flush stdout) lex ;
    let lex = Lexing.from_string "123 4\n 5\n 00123 bug 34 56\n" in
    try
      iter_int (fun i -> Printf.printf "%d\n" i ; flush stdout) lex ;
      assert false ;
    with Invalid_argument _ ->
    ()



# 392 "src/basicLexer.ml"
