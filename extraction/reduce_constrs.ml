open AST
open Clight
open Ctypes
open Camlcoq
open Typing
open Preproc_constrs
open Ownership
open Format

module StructUnion = Map.Make(String)
(* let struct_unions = ref StructUnion.empty *)
let struct_tbl = ref (Hashtbl.create 10)

let rec remove_teq c =
  match c with
  | [] -> []
  | hd :: tl ->
    match hd with
    | Le _ | Lt _ | Eq _ ->  hd :: remove_teq tl
    | Empty (id, i, loc) -> failwith "empty exists yet"
    | Teq (l1, l2, loc) -> remove_teq tl

let rec reduce c s h =
(* struct_unions := s; *)
  struct_tbl := h;
  let rec f l =
    match l with
    | [] -> []
    | hd :: tl ->
      match hd with
      | Le _ | Lt _ | Eq _ ->  hd :: f tl
      | Empty (id, i, loc) ->
        let s = Hashtbl.find !struct_tbl ((extern_atom id), i) in
        let empty = reduce_empty s loc in
        empty @ f tl
      | Teq (l1, l2, loc) -> let (c, l1, l2) = reduce_teq l1 l2 loc in
                        c @ f tl
  in
  let rec iter l =
    let new_l = uniq (f l) in
    (* printf "%a@\n" print_constrs new_l; *)
    if l = new_l
    then l
    else iter new_l
  in
  let new_c = iter c in
  let removed_c = remove_teq new_c in
  (* printf "---- reduced constrs ----@\n";
     printf "%a" print_constrs removed_c; *)
  removed_c
