(* Print Variables appearing in expressions. *)

open Format
open Camlcoq
open Datatypes
open Values
open AST
open PrintAST
open Ctypes
open Ownership
open Cop
open PrintCsyntax
open Clight
open Str

(** temporary name **)

let temp_name (id: ident) = "$" ^ Z.to_string (Z.Zpos id)

(** type **)

let rec print_type p t =
  match t with
  | Tpointer (_, o, _) ->
    (match o with
    | OVar i -> fprintf p ": o_%d" i
    | _ -> fprintf p ": pointer")
  | Tarray (_, _, o, _) ->
    (match o with
    | OVar i -> fprintf p ":o_%d" i
    | _ -> fprintf p ": array")
  | _ -> ()

(** Expressions *)

let rec expr_var p (prec, e) =
  fprintf p "@[<hov 0>";
  begin match e with
  | Evar(id, ty) ->
    fprintf p "%s%a" (extern_atom id) print_type ty
  | Etempvar(id, ty) ->
    fprintf p "%s%a" (temp_name id) print_type ty
  | Ederef(a1, ty) ->
    fprintf p "*%a%a" expr_var (prec, a1) print_type ty
  | Efield(a1, f, ty) ->
    fprintf p "%a.%s %a" expr_var (prec, a1) (extern_atom f) print_type ty
  | Econst_int(n, Tint(I32, Unsigned, _)) ->
    (* fprintf p "%luU" (camlint_of_coqint n) *)
    fprintf p "const";
  | Econst_int(n, _) ->
    (* fprintf p "%ld" (camlint_of_coqint n) *)
    fprintf p "const";
  | Econst_float(f, _) ->
    (* fprintf p "%F" (camlfloat_of_coqfloat f) *)
    fprintf p "const";
  | Econst_single(f, _) ->
    (* fprintf p "%Ff" (camlfloat_of_coqfloat32 f) *)
    fprintf p "const";
  | Econst_long(n, Tlong(Unsigned, _)) ->
    (* fprintf p "%LuLLU" (camlint64_of_coqint n) *)
    fprintf p "const";
  | Econst_long(n, _) ->
    (* fprintf p "%LdLL" (camlint64_of_coqint n) *)
    fprintf p "const";
  | Eunop(Oabsfloat, a1, _) ->
    fprintf p "__builtin_fabs(%a)" expr_var (2, a1)
  | Eunop(op, a1, ty) ->
    fprintf p "[%s %a]" (name_unop op) expr_var (prec, a1)
  | Eaddrof(a1, ty) ->
    fprintf p "&%a" expr_var (prec, a1)
  | Ebinop(op, a1, a2, ty) ->
    fprintf p "[%a@ %s %a]"
      expr_var (prec, a1) (name_binop op) expr_var (prec, a2)
  | Ecast(a1, ty) ->
    fprintf p "(%s) %a" (name_type ty) expr_var (prec, a1)
  end;
  fprintf p "@]"

let print_expr_var p e = expr_var p (0, e)

let rec print_expr_var_list p (first, rl) =
  match rl with
  | [] -> ()
  | r :: rl ->
    if not first then fprintf p ",@ ";
    expr_var p (2, r);
    print_expr_var_list p (false, rl)

(** Statements **)

let rec print_stmt_var p s =
  match s with
  | Sskip ->
    fprintf p "/* skip */;"
  | Sassign (e1, e2) ->
    fprintf p "@[Assign: [%a, %a];@]" print_expr_var e1 print_expr_var e2
  | Sset(id, e2) ->
    fprintf p "@[Set: [%s, %a];@]" (temp_name id) print_expr_var e2
  | Scall(None, e1, el) ->
    fprintf p "@[<hv 2>Call: %a@,(@[<hov 0>%a@]);@]"
      print_expr_var e1
      print_expr_var_list (true, el)
  | Scall(Some id, e1, el) ->
    fprintf p "@[<hv 2>Call: [%s,%a@,(@[%a@])]@];"
      (temp_name id)
      print_expr_var e1
      print_expr_var_list (true, el)
  | Sbuiltin(None, ef, tyargs, el) ->
    fprintf p "@[builtin %s@,(@[<hov 0>%a@]);@]"
      (name_of_external ef)
      print_expr_var_list (true, el)
  | Sbuiltin(Some id, ef, tyargs, el) ->
    fprintf p "@[[%s, builtin %s@,(@[<hov 0>%a@])];@]"
      (temp_name id)
      (name_of_external ef)
      print_expr_var_list (true, el)
  | Ssequence(Sskip, s2) ->
    print_stmt_var p s2
  | Ssequence(s1, Sskip) ->
    print_stmt_var p s1
  | Ssequence(s1, s2) ->
    fprintf p "%a@\n %a" print_stmt_var s1 print_stmt_var s2
  | Sifthenelse(e, s1, Sskip) ->
    fprintf p "@[if (%a) {@ %a@;<0 -2>}@]"
      print_expr_var e
      print_stmt_var s1
  | Sifthenelse(e, Sskip, s2) ->
    fprintf p "@[if (! %a) {@ %a@;<0 -2>}@]"
      expr_var (15, e)
      print_stmt_var s2
  | Sifthenelse(e, s1, s2) ->
    fprintf p "@[if (%a) {@ %a@;<0 -2>} else {@ %a@;<0 -2>}@]"
      print_expr_var e
      print_stmt_var s1
      print_stmt_var s2
  | Sloop(s1, Sskip) ->
    fprintf p "@[while (1) {@ %a@;<0 -2>}@]"
      print_stmt_var s1
  | Sloop(s1, s2) ->
    fprintf p "@[for (@[<hv 0>;@ 1;@ %a) {@]@ %a@;<0 -2>}@]"
      print_stmt_var_for s2
      print_stmt_var s1
  | Sbreak ->
    fprintf p "break;"
  | Scontinue ->
    fprintf p "continue;"
  | Sswitch(e, cases) ->
    fprintf p "@[<v 0>switch (%a) {@ %a@;<0 -2>}@]"
      print_expr_var e
      print_cases_var cases
  | Sreturn None ->
    fprintf p "return;"
  | Sreturn (Some e) ->
    fprintf p "return [%a];" print_expr_var e
  | Slabel(lbl, s1) ->
    fprintf p "%s:@ %a" (extern_atom lbl) print_stmt_var s1
  | Sgoto lbl ->
    fprintf p "goto %s;" (extern_atom lbl)

and print_cases_var p cases =
  match cases with
  | LSnil ->
      ()
  | LScons(lbl, Sskip, rem) ->
      fprintf p "%a:@ %a"
              print_case_label lbl
              print_cases_var rem
  | LScons(lbl, s, rem) ->
      fprintf p "@[<v 2>%a:@ %a@]@ %a"
              print_case_label lbl
              print_stmt_var s
              print_cases_var rem

and print_case_label p = function
  | None -> fprintf p "default"
  | Some lbl -> fprintf p "case %ld" (camlint_of_coqint lbl)

and print_stmt_var_for p s =
  match s with
  | Sskip ->
      fprintf p "/* nothing */"
  | Sassign(e1, e2) ->
      fprintf p "@[<hv 2>%a = %a@]" print_expr_var e1 print_expr_var e2
  | Sset(id, e2) ->
      fprintf p "%s = %a" (temp_name id) print_expr_var e2
  | Ssequence(s1, s2) ->
      fprintf p "%a, %a" print_stmt_var_for s1 print_stmt_var_for s2
  | Scall(None, e1, el) ->
      fprintf p "@[<hv 2>%a@,(@[<hov 0>%a@])@]"
                print_expr_var e1
                print_expr_var_list (true, el)
  | Scall(Some id, e1, el) ->
      fprintf p "@[<hv 2>%s =@ %a@,(@[<hov 0>%a@])@]"
                (temp_name id)
                print_expr_var e1
                print_expr_var_list (true, el)
  | Sbuiltin(None, ef, tyargs, el) ->
      fprintf p "@[<hv 2>builtin %s@,(@[<hov 0>%a@]);@]"
                (name_of_external ef)
                print_expr_var_list (true, el)
  | Sbuiltin(Some id, ef, tyargs, el) ->
      fprintf p "@[<hv 2>%s =@ builtin %s@,(@[<hov 0>%a@]);@]"
                (temp_name id)
                (name_of_external ef)
                print_expr_var_list (true, el)
  | _ ->
      fprintf p "({ %a })" print_stmt_var s

let rec print_function_variables p vars =
  match vars with
  | [] -> ()
  | (id, ty) :: tl ->
    fprintf p "@[<h 2>%s%a@]@." (name_cdecl (extern_atom id) ty) print_type ty;
    print_function_variables p tl

let rec print_function_params p params =
  match params with
  | [] ->  ()
  | (id, ty) :: tl ->
    fprintf p "%s" (extern_atom id);
    (if (tl = [])
     then ()
     else fprintf p ",");
    print_function_params p tl

let print_function_var p id f =
  fprintf p "%s@[(" (extern_atom id);
  print_function_params p f.fn_params;
  fprintf p ")@] @[<hov 2>{@.";
  print_function_variables p f.fn_vars;
  fprintf p "@.";
  print_stmt_var p f.fn_body;
  fprintf p "@.}@]@."

let print_fundef_var p id fd =
  match fd with
  | External (_, _, _, _) ->
    ()
  | Internal f ->
    print_function_var p id f

let print_globdef_var p (id, gd) =
  match gd with
  | Gfun f -> print_fundef_var p id f
  | Gvar v -> print_globvar p id v

let print_program_var p prog =
  List.iter (print_globdef_var p) prog.prog_defs

let f prog =
  print_program_var std_formatter prog
