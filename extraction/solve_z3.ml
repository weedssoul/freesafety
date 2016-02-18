open Typing
open Ownership
open Format

let loc_tbl = Hashtbl.create 10

let next_id = ref 0

let fresh_constr =
  fun () ->
    let i = !next_id in
    incr next_id; i

let rec print_loc_id fmt (i1, s, i2) =
  fprintf fmt "@[c_%d -> %s: %d@]" i1 s i2


let rec print_ownership_z3 fmt o =
  match o with
  | OVar i -> fprintf fmt "o_%d" i
  | OConst i -> fprintf fmt "%d" i
  | OPlus (o1, o2) ->
    fprintf fmt
      "(+ @[%a@] @[%a@])"
      print_ownership_z3 o1 print_ownership_z3 o2

let print_constr_z3 fmt c =
  let id = fresh_constr() in
  match c with
  | Le (o1, o2, loc) ->
    Hashtbl.add loc_tbl id (List.hd loc);
    fprintf fmt "(assert (! (<= @[%a@] @[%a@]) :named c_%d))"
      print_ownership_z3 o1 print_ownership_z3 o2 id
  | Lt (o1, o2, loc) ->
    Hashtbl.add loc_tbl id (List.hd loc);
    fprintf fmt "(assert (! (< @[%a@] @[%a@]) :named c_%d))"
      print_ownership_z3 o1 print_ownership_z3 o2 id
  | Eq (o1, o2, loc) ->
    Hashtbl.add loc_tbl id (List.hd loc);
    fprintf fmt "(assert (! (= @[%a@] @[%a@]) :named c_%d))"
      print_ownership_z3 o1 print_ownership_z3 o2 id
  | Empty _ | Teq _ ->
    failwith "empty and teq are not be allowed!!"

let print_declare_z3 fmt oid =
  fprintf fmt
    "(declare-const o_%d Real) (assert (and (<= 0. o_%d) (<= o_%d 1.)))"
    oid oid oid

let rec search_hashtbl l r =
  match l with
  | [] -> List.sort_uniq compare r
  | hd :: tl ->
    let sl = Str.split (Str.regexp "_") hd in
    let id = int_of_string (List.nth sl 1) in
    let (s, i) = Hashtbl.find loc_tbl id in
    (* printf "%a@\n" print_loc_id (id, s, i); *)
    search_hashtbl tl (i :: r)

let solve c =
  let cmd_z3 = "./z3 -smt2 -in" in
  let (ch_z3_in, ch_z3_out) = Unix.open_process cmd_z3 in
  let f = formatter_of_out_channel ch_z3_out in
  (* let f = std_formatter in *)
  fprintf f "(set-option :produce-unsat-cores true)@.";
  fprintf f "(set-option :produce-models true)@.";
  let oid_list = oid_of_constrs c in
  List.iter (fun oid -> fprintf f "@[%a@]@." print_declare_z3 oid) oid_list;
  List.iter (fun c -> fprintf f "@[%a@]@." print_constr_z3 c) c;
  fprintf f "(check-sat)@.";
  (* fprintf f "(get-model)@."; *)
  fprintf f "(get-unsat-core)@.";
  fprintf f "(exit)@.";
  close_out ch_z3_out;
  let first_line = input_line ch_z3_in in
  printf "%s@\n" first_line;
  let second_line = input_line ch_z3_in in
  if (first_line <> "sat")
  then
    begin
      let unsat_core = String.sub second_line 1 (String.length second_line - 2) in
      (* printf "%s@\n" second_line; *)
      close_in ch_z3_in;
      let sl = Str.split (Str.regexp " ") unsat_core in
      let l = search_hashtbl sl [] in
      printf "-- slice %d --@\n" (List.length l);
      List.iter (fun i -> printf "%d " i) l;
      printf "@\n-- slice end --@\n";
    end
