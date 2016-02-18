open C
open AST
open Clight
open Ctypes
open Camlcoq
open Typing
open Ownership
open Format

let struct_tbl = ref (Hashtbl.create 10)

module StructUnion = Map.Make(String)
let struct_unions = ref StructUnion.empty

let rec add_expand_result id i t =
  Hashtbl.add !struct_tbl ((extern_atom id), i) t

let rec rename_same_comp_ptr (t: coq_type) (p: ident * int) : coq_type =
  match t with
  | Tvoid | Tint _ | Tlong _ | Tfloat _ | Tarray _ | Tunion _ -> t
  | Tpointer (t, o, a)  ->
    Tpointer (rename_same_comp_ptr t p, o, a)
  | Tfunction (tl, tl', t, c) ->
    Tfunction (tl, tl', t, c)
  | Tstruct (i, f, a) ->
    let new_f = rename_field_same_comp_ptr f p in
    Tstruct (i, new_f, a)
  | Tcomp_ptr (id, a, o, i) ->
    let (id', i') = p in
    Tcomp_ptr (id', a, o, i')
  | Tplus (l, o) ->
    (* TODO: modify *)
    Tplus (l, o)

and rename_field_same_comp_ptr (fl: fieldlist) (p: ident * int) : fieldlist =
  match fl with
  | Fnil -> Fnil
  | Fcons (id, ty, fl) ->
    let new_ty = rename_same_comp_ptr ty p in
    Fcons (id, new_ty, rename_field_same_comp_ptr fl p)

let rec ident_int_of_type (t: coq_type) : (ident * int) =
  match t with
  | Tpointer (t, o, a) ->
    ident_int_of_type t
  | Tarray (t, z, o, a) ->
    ident_int_of_type t
  | Tstruct(_, fl, _) | Tunion(_, fl, _) ->
    let rec iter fl =
      match fl with
      | Fnil -> failwith "no comp_ptr"
      | Fcons(_, t, fl) ->
        match t with
        | Tcomp_ptr (id, a, o, i) -> (id, i)
        | _ -> iter fl
    in
    iter fl
  | Tcomp_ptr (id, a, o, i) -> (id, i)
  | _ -> failwith "no comp_ptr"

let rec change_location c loc =
  match c with
  | [] -> []
  | hd :: tl ->
    let c' =
      match hd with
      | Le (o1, o2, l) -> Le (o1, o2, loc)
      | Lt (o1, o2, l) -> Lt (o1, o2, loc)
      | Eq (o1, o2, l) -> Eq (o1, o2, loc)
      | Empty (id, i, l) -> Empty (id, i, loc)
      | Teq (l1, l2, l) -> Teq (l1, l2, loc)
    in
    c' :: change_location tl loc

let rec reduce_empty t loc =
  match t with
  | Tpointer (t, o, _) ->
    let c = reduce_empty t loc in
    Eq (o, OConst 0, loc) :: c
  | Tcomp_ptr (id, a, o, i) ->
    [Eq (o, OConst 0, loc)]
  | Tstruct (id, fl, a) | Tunion (id, fl, a) ->
    let rec iter fl =
      match fl with
      | Fnil -> []
      | Fcons (id, t, fl) ->
        let c = reduce_empty t loc in
        c @ iter fl
    in
    iter fl
  | _ -> []

let rec preproc_empty (id: ident) (i: int) (loc: location list) : constr list =
  if Hashtbl.mem !struct_tbl ((extern_atom id), i)
  then
    let s = Hashtbl.find !struct_tbl ((extern_atom id), i) in
    let (id', i') = ident_int_of_type s in
    reduce_empty s loc @ preproc_empty id' i' loc
  else
    [Empty (id, i, loc)]

let rec expand_ident_int l =
  match l with
  | [] -> []
  | hd :: tl ->
    let (id, i) = hd in
    if Hashtbl.mem !struct_tbl ((extern_atom id), i)
    then
      let s = Hashtbl.find !struct_tbl ((extern_atom id), i) in
      s :: expand_ident_int tl
    else
      let s = StructUnion.find (extern_atom id) !struct_unions in
      let renamed_s = rename_all_comp_ptr (rename_all_ownership_type s) in
      (* add_expand_result id i renamed_s; *)
      renamed_s :: expand_ident_int tl

let rec add_list l =
  match l with
  | [] -> failwith "add_list: empty list"
  | [hd] -> hd
  | hd :: tl ->
    let (t, c) = add_type hd (add_list tl) in
    t

let rec reduce_teq l1 l2 loc =
  let el1 = expand_ident_int l1 in
  let el2 = expand_ident_int l2 in
  let l1' = List.map (fun x -> ident_int_of_type x) el1 in
  let l2' = List.map (fun x -> ident_int_of_type x) el2 in
  let t1 = add_list el1 in
  let t2 = add_list el2 in
  let c = eq_type t1 t2 in
  let c' = change_location c loc in
  (* change location *)
  (c', l1', l2')

let rec preproc_teq l1 l2 loc =
  (*
    First: Check whether every elements in l1 l2 appears in the domain of hashtbl.
    Second: If true
    then expand each elements in l1 l2 and
    apply First to expanded list l1'and l2',
    else return Teq(l1, l2).
  *)
  let b1 =
    List.for_all (fun x ->
      let (id, i) = x in Hashtbl.mem !struct_tbl ((extern_atom id), i)
    ) l1 in
  let b2 =
    List.for_all (fun x ->
      let (id, i) = x in Hashtbl.mem !struct_tbl ((extern_atom id), i)
    ) l2 in
  (* check empty l1 or l2 *)
  if b1 && b2
  then let (c, l3, l4) = reduce_teq l1 l2 loc in
       let (c', l3', l4') = preproc_teq l3 l4 loc in
       (c @ c', l3', l4')
  else ([Teq(l1, l2, loc)], l1, l2)

let add_same_struct_empty id i =
  if Hashtbl.mem !struct_tbl ((extern_atom id), i)
  then ()
  else
    let s = StructUnion.find (extern_atom id) !struct_unions in
    let renamed_s =
      rename_same_comp_ptr (rename_all_ownership_type s) (id, i) in
    add_expand_result id i renamed_s

let add_same_struct_teq l1 l2 =
  let rec f l =
    match l with
    | [] -> ()
    | hd :: tl ->
      let (id, i) = hd in
      if Hashtbl.mem !struct_tbl ((extern_atom id), i)
      then f tl
      else
        begin
          let s = StructUnion.find (extern_atom id) !struct_unions in
          let renamed_s =
            rename_same_comp_ptr (rename_all_ownership_type s) (id, i) in
          add_expand_result id i renamed_s;
          f tl
        end
  in
  f l1;
  f l2

let rec add_same_struct_constr c =
  (* structs that have not been added to hashtbl point to themselves *)
  match c with
  | [] -> ();
  | hd :: tl ->
    match hd with
    | Le _ | Lt _ | Eq _ -> add_same_struct_constr tl
    | Empty (id, i, loc) ->
      add_same_struct_empty id i;
      add_same_struct_constr tl
    | Teq (l1, l2, loc) ->
      add_same_struct_teq l1 l2;
      add_same_struct_constr tl

let add_same_struct_tbl tbl =
  (* values of hasthtbl that have not been added to hashtbl point to themselves *)
  Hashtbl.iter (fun (s, i) t ->
    let (id', i') = ident_int_of_type t in
    if Hashtbl.mem !struct_tbl ((extern_atom id'), i')
    then ()
    else
      begin
        let s = StructUnion.find (extern_atom id') !struct_unions in
        let renamed_s =
          rename_same_comp_ptr (rename_all_ownership_type s) (id', i') in
        add_expand_result id' i' renamed_s;
      end
  ) tbl

let rec preproc c s h =
  struct_tbl := h;
  struct_unions := s;
  let rec f l =
    match l with
    | [] -> []
    | hd :: tl ->
      match hd with
      | Le _ | Lt _ | Eq _ -> hd :: f tl
      | Empty (id, i, loc) -> let empty = preproc_empty id i loc in
                         empty @ f tl
      | Teq (l1, l2, loc) -> let (c, l1, l2) = preproc_teq l1 l2 loc in
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
  add_same_struct_constr new_c;
  add_same_struct_tbl !struct_tbl;
  (* printf "--- Hashtbl: %d --- @\n" (Hashtbl.length !struct_tbl);
  Hashtbl.iter (fun (s, i) t ->
    printf "%a@\n" print_hashtbl (s, i, t)) !struct_tbl;
  printf "---- preproc constr 1 ---- @\n";
  printf "%a@\n" print_constrs new_c; *)
  (new_c, !struct_unions, !struct_tbl)

