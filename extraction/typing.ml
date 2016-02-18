open C
open AST
open Clight
open Ctypes
open Ownership
open Type_env
open BinNums
open OUnit2
open Format
open Camlcoq
open C2C

type constr =
| Le of ownership * ownership * location list
| Lt of ownership * ownership * location list
| Eq of ownership * ownership * location list
| Empty of ident * int * location list
| Teq of ((ident * int) list) * ((ident * int) list) * location list

(** location **)
let start_loc = ref Cutil.no_loc
let current_loc = ref Cutil.no_loc
let end_loc = ref Cutil.no_loc

let update_loc s i = current_loc := (s, i)

let rec change_location c new_loc =
  match c with
  | [] -> []
  | hd :: tl ->
    match hd with
    | Le(o1, o2, loc) -> Le(o1, o2, [new_loc]) :: change_location tl new_loc
    | Lt(o1, o2, loc) -> Lt(o1, o2, [new_loc]) :: change_location tl new_loc
    | Eq(o1, o2, loc) -> Eq(o1, o2, [new_loc]) :: change_location tl new_loc
    | Empty(id, n, loc) -> Empty(id, n, [new_loc]) :: change_location tl new_loc
    | Teq(l1, l2, loc) -> Teq(l1, l2, [new_loc]) :: change_location tl new_loc

(** printer **)
let rec print_location fmt loc =
  let (s, i) = List.hd loc in
  fprintf fmt "#%d" i

let rec print_ctypes fmt ty =
  match ty with
  | Tint (_, _, _) -> fprintf fmt "int"
  | Tpointer (ty, o, _) ->
    fprintf fmt "@[%a@] ref (%a) pointer"
      print_ctypes ty
      print_ownership o
  | Tvoid -> fprintf fmt "void"
  | Tlong _ -> fprintf fmt "long"
  | Tfloat _ -> fprintf fmt "float"
  | Tarray (ty, _, o, _) ->
    fprintf fmt "@[%a@] array (%a)"
      print_ctypes ty
      print_ownership o
  | Tstruct (id, fl, attr) | Tunion (id, fl, attr) ->
        fprintf fmt "struct:@[{@\n @[%a@] }@]"
      print_fieldlist fl
  | Tfunction (tl, tl', ty, _) ->
    fprintf fmt "function (@[%a@] -> @[%a@]) : @[%a@] "
      print_typelist tl
      print_typelist tl'
      print_ctypes ty
  | Tcomp_ptr (id, a, o, i) ->
    fprintf fmt "comp_ptr %s %i ref (%a)"
      (extern_atom id) i print_ownership o
  | Tplus (l, o)->
    fprintf fmt "Tplus %a %a" print_tplus l print_ownership o
and print_fieldlist fmt fl =
  match fl with
  | Fnil -> fprintf fmt ""
  | Fcons (id, ty, fl) ->
    fprintf fmt "@[%a@]@\n @[%a@]"
      print_ctypes ty
      print_fieldlist fl
and print_typelist fmt tl =
  match tl with
  | Tnil -> fprintf fmt ""
  | Tcons (ty, tl) ->
    fprintf fmt "@[%a@], @[%a@]"
      print_ctypes ty
      print_typelist tl
and print_tplus fmt l =
  match l with
  | [] -> fprintf fmt ""
  | (id, i) :: tl ->
    fprintf fmt "@[%s %i; %a@] " (extern_atom id) i print_tplus tl
(*  | _ -> failwith "print_ctypes not implemented" *)

let rec print_constr fmt c =
  match c with
  | Le(o1, o2, loc) ->
    fprintf fmt "@[%a <= %a : %a@]@\n"
      print_ownership o1
      print_ownership o2
      print_location loc
  | Lt(o1, o2, loc) ->
    fprintf fmt "@[%a < %a : %a@]@\n"
      print_ownership o1
      print_ownership o2
      print_location loc
  | Eq(o1, o2, loc) ->
    fprintf fmt "@[%a = %a : %a@]@\n"
      print_ownership o1
      print_ownership o2
      print_location loc
  | Empty(id, i, loc) ->
    fprintf fmt "Empty[%s %d] : %a@\n"
      (extern_atom id)
      i
      print_location loc
  | Teq(l1, l2, loc) ->
    fprintf fmt "Teq[%a = %a] : %a@\n"
      print_teq l1
      print_teq l2
      print_location loc

and print_teq fmt l =
  match l with
  | [] -> fprintf fmt ""
  | (id, i) :: tl ->
    fprintf fmt "@[%s %i;%a@]" (extern_atom id) i print_tplus tl

let rec print_list ?(sep="\n") pp fmt l =
  match l with
  | [] -> ()
  | [hd] -> fprintf fmt "%a" pp hd
  | hd::tl ->
    fprintf fmt "%a%s%a" pp hd sep (print_list ~sep pp) tl

let print_constrs fmt c =
  print_list print_constr fmt c

let print_id_type fmt (id, ty) =
  fprintf fmt "@[%s: %a@]" (extern_atom id) print_ctypes ty

let print_env fmt env =
  print_list  print_id_type fmt env

let print_hashtbl fmt (s, i, t) =
  fprintf fmt "@[%s,%d: %a@]" s i print_ctypes t

let rec fresh_ovar_type (t: coq_type) =
  match t with
  | Tvoid | Tint _ | Tlong _ | Tfloat _ | Tplus _ |
    Tarray _ | Tunion _ | Tfunction _->
    t
  | Tpointer (t, o, a) ->
    Tpointer (fresh_ovar_type t, OVar (fresh_ovar()), a)
  | Tcomp_ptr (id, a, o, i) ->
    Tcomp_ptr (id, a, OVar (fresh_ovar()), fresh_int())
  | Tstruct (id, fl, a) ->
    let fl' = fresh_ovar_fieldlist fl in
    Tstruct (id, fl', a)
and fresh_ovar_fieldlist (fl: fieldlist) =
  match fl with
  | Fnil -> Fnil
  | Fcons (id, t, fl) ->
    let fl' = fresh_ovar_fieldlist fl in
    let new_t = fresh_ovar_type t in
    Fcons (id, new_t, fl')


(* all enviroment *)
let all_env = ref []

(* Collecting the names and fields of structs and unions *)
module StructUnion = Map.Make(String)
let struct_unions = ref StructUnion.empty

let struct_tbl = Hashtbl.create 10

let rec uniq l =
  List.sort_uniq compare l

let rec ident_int_of_type (t: coq_type) : (ident * int) list =
  match t with
  | Tpointer (t, o, a) ->
    ident_int_of_type t
  | Tarray (t, z, o, a) ->
    ident_int_of_type t
  | Tstruct(_, fl, _) | Tunion(_, fl, _) ->
    let rec iter fl =
      match fl with
      | Fnil -> []
      | Fcons(_, t, fl) ->
        (ident_int_of_type t) @ (iter fl)
    in
    iter fl
  | Tcomp_ptr (id, a, o, i) -> [(id, i)]
  | _ -> []

let rec collect_oid (o: ownership) : oid list =
  match o with
  | OVar i ->[i]
  | OConst f -> []
  | OPlus(o1, o2) -> collect_oid o1 @ collect_oid o2

let rec oid_of_ty (ty: coq_type) : oid list =
  match ty with
  | Tpointer (t, o, _) | Tarray (t, _, o, _) ->
    let ol = oid_of_ty t in
    let oid_list = collect_oid o in
    ol @ oid_list
  | Tfunction _ -> []
    (* failwith "oid_of_ty: Tfunction not implemented" *)
  | Tstruct (_, fl, _) | Tunion (_, fl, _) ->
    let rec iter fl =
      match fl with
      | Fnil -> []
      | Fcons(_, t, fl) ->
        (oid_of_ty t) @ (iter fl)
    in
    iter fl
  | Tcomp_ptr (id, a, o, i) ->
    let oid_list = collect_oid o in
    oid_list
  | _ -> []

let rec oid_of_constr (c: constr) : oid list =
  match c with
  | Le (o1, o2, loc) | Lt (o1, o2, loc) | Eq (o1, o2, loc) ->
    let ol1 = collect_oid o1 in
    let ol2 = collect_oid o2 in
    ol1 @ ol2
  | Empty _ | Teq _  -> []

let rec oid_of_constrs (cl: constr list) : oid list =
  let ol = List.fold_left
    (fun l c -> oid_of_constr c @ l) [] cl in
  uniq ol

let rec intersection_of_oid_lists (ol1: oid list) (ol2: oid list) (r: oid list)
    : oid list =
  match ol1 with
  | [] -> r
  | h :: t ->
      if (List.exists (fun x -> h = x) ol2) then
        intersection_of_oid_lists t ol2 (h :: r)
      else
        intersection_of_oid_lists t ol2 r

let ownership_of_constr (c: constr) : ownership list =
  match c with
  | Le (o1, o2, loc) | Lt (o1, o2, loc) | Eq (o1, o2, loc) ->
    [o1; o2]
  | Empty _ | Teq _  -> []

(** search ty recursively.
 ** If an ownership in ol appears in ty,
 ** rename the ownership to new one **)
let rec rename_type (ty: coq_type) (ol: oid list) : coq_type =
  match ty with
  | Tvoid -> Tvoid
  | Tint (i, s, a) -> Tint (i, s, a)
  | Tlong (s, a) -> Tlong (s, a)
  | Tfloat (f, a) -> Tfloat (f, a)
  | Tpointer (t, o, a)  ->
    let oid_list = collect_oid o in
    let intersection = intersection_of_oid_lists
      oid_list ol [] in
    let new_o =
      if ((List.length intersection) >= 1) then
        OVar (fresh_ovar ())
      else
        o
    in
    Tpointer (rename_type t ol, new_o, a)
  | Tarray (t, i, o, a) ->
    let oid_list = collect_oid o in
    let intersection = intersection_of_oid_lists
      oid_list ol [] in
    let new_o =
      if ((List.length intersection) >= 1) then
        OVar (fresh_ovar ())
      else
        o
    in
    Tarray (rename_type t ol, i, new_o, a)
  | Tfunction (tl, tl', t, c) ->
    Tfunction (tl, tl', t, c)
  | Tstruct (i, f, a) ->
    let new_f = rename_field f ol in
    Tstruct (i, new_f, a)
  | Tunion (i, f, a) ->
    let new_f = rename_field f ol in
    Tunion (i, new_f, a)
  | Tcomp_ptr (id, a, o, i) ->
    let oid_list = collect_oid o in
    let intersection = intersection_of_oid_lists
      oid_list ol [] in
    let new_o =
      if ((List.length intersection) >= 1) then
        OVar (fresh_ovar ())
      else
        o
    in
    Tcomp_ptr (id, a, new_o, i)
(* TO: modify *)
  | Tplus (l, o) ->
    Tplus (l, o)

and rename_field (f: fieldlist) (ol: oid list) : fieldlist =
  match f with
  | Fnil -> Fnil
  | Fcons (id, ty, fl) ->
    let new_ty = rename_type ty ol in
    Fcons (id, new_ty, rename_field fl ol)

let rec rename_comp_ptr (t: coq_type) (il: (ident * int) list)
    : coq_type =
  match t with
  | Tvoid | Tint _ | Tlong _ | Tfloat _ | Tarray _ | Tunion _ -> t
  | Tpointer (t, o, a)  ->
    Tpointer (rename_comp_ptr t il, o, a)
  | Tfunction (tl, tl', t, c) ->
    Tfunction (tl, tl', t, c)
  | Tstruct (i, f, a) ->
    let new_f = rename_field_comp_ptr f il in
    Tstruct (i, new_f, a)
  | Tcomp_ptr (id, a, o, i) ->
    if (List.mem (id,i) il)
    then
      make_comp_ptr id a o
    else
      t
  (* TO: modify *)
  | Tplus (l, o) ->
    Tplus (l, o)

and rename_field_comp_ptr (fl: fieldlist) (il: (ident * int) list) : fieldlist =
  match fl with
  | Fnil -> Fnil
  | Fcons (id, ty, fl) ->
    let new_ty = rename_comp_ptr ty il in
    Fcons (id, new_ty, rename_field_comp_ptr fl il)

let rec rename_all_comp_ptr (t: coq_type) : coq_type =
  let il = ident_int_of_type t in
  rename_comp_ptr t il

let rec rename_env env (ol: oid list) (il: (ident * int) list) =
  List.map
    (fun (id, t) ->
      let new_t =
        if ((List.length il) = 0)
        then rename_type t ol
        else
          let nt = rename_type t ol in
          rename_comp_ptr nt il
      in
      (id, new_t))
    env

let rec rename_all_ownership_type t =
  let oid = oid_of_ty t in
  rename_type t oid

let rec rename_all_ownership_env env =
  List.map
    (fun (id, t) ->
      (id, rename_all_ownership_type t)
    ) env

let rec add_expand_result id i t =
  Hashtbl.add struct_tbl ((extern_atom id), i) t

(* TODO: map to Tstruct *)
let rec expand_comp_ptr id i o =
  if Hashtbl.mem struct_tbl ((extern_atom id), i)
  then Hashtbl.find struct_tbl ((extern_atom id), i)
  else
    let s = StructUnion.find (extern_atom id) !struct_unions in
    let renamed_s = rename_all_comp_ptr (rename_all_ownership_type s) in
    add_expand_result id i renamed_s;
    renamed_s

let rec int_to_positive (i:int) : positive =
  if i = 0 then
    Coq_xH
  else if i > 0 then
    Coq_xI (int_to_positive (i - 1))
  else
    Coq_xO (int_to_positive (i + 1))

let rec positive_to_int (p:positive) : int =
  match p with
  | Coq_xH -> 0
  | Coq_xI p -> (positive_to_int p) + 1
  | Coq_xO p -> (positive_to_int p) - 1



let rec eq_ownership (o1: ownership) (o2: ownership) : constr list =
  if (compare o1 o2) = 0 then
    []
  else
    [Eq (o1, o2, [!current_loc])]

let writable_ownership (o: ownership) : constr =
  Eq (o, OConst 1, [!current_loc])

let readable_ownership (o: ownership) : constr =
  Lt (OConst 0, o, [!current_loc])

(* TODO: correct? *)
let rec empty_type (ty: coq_type) :constr list =
  match ty with
  | Tpointer (t, o, a) ->
    let c = empty_type t in
    Eq (o, OConst 0, [!current_loc]) :: c
  | Tstruct (id, fl, a) | Tunion (id, fl, a) ->
    empty_fieldlist fl
  | Tcomp_ptr (id, attr, o, i) ->
     Eq (o, OConst 0, [!current_loc]) :: [Empty (id, i, [!current_loc])]
  | _ -> []

and empty_fieldlist (fl: fieldlist) : constr list =
  match fl with
  | Fnil -> []
  | Fcons (id, t, fl) ->
    let c = empty_type t in
    c @ empty_fieldlist fl

let rec eq_type (ty1:coq_type) (ty2:coq_type) : constr list =
  match (ty1, ty2) with
  | Tvoid, ty | ty, Tvoid ->
    empty_type ty
  | ((Tint _ | Tlong _ | Tfloat _),
     ( Tint _ | Tlong _ | Tfloat _)) ->
    []
  | (Tpointer (t1, o1, _), Tpointer (t2, o2, _)) ->
    (eq_type t1 t2) @ (eq_ownership o1 o2)
  | Tstruct (id1, fl1, _), Tstruct (id2, fl2, _) ->
    eq_fieldlist fl1 fl2
  | ((Tfunction (tl1, tl1', ty1, cc1)), (Tfunction (tl2, tl2', ty2, cc2))) ->
    []
  | Tplus (l1, o1), Tplus (l2, o2) ->
    eq_ownership o1 o2 @ [Teq (l1, l2, [!current_loc])]
  | Tcomp_ptr (id1, a1, o1, i1), Tcomp_ptr (id2, a2, o2, i2) ->
    if compare id1 id2 = 0
    then (eq_ownership o1 o2) @ [Teq ([(id1, i1)], [(id2, i2)], [!current_loc])]
    else failwith "eq_type: Tcomp_ptr"
  | Tcomp_ptr (id, a, o, i), Tplus (l, o')
  | Tplus (l, o'), Tcomp_ptr (id, a, o, i)
    -> eq_ownership o o' @ [Teq ([id, i], l, [!current_loc])]
  | Tcomp_ptr (id, a, o, i) , Tpointer (t1, o1, _)
  | Tpointer (t1, o1, _) , Tcomp_ptr (id, a, o, i) ->
    let s = expand_comp_ptr id i o in
    eq_ownership o o1 @ eq_type s t1
  | Tcomp_ptr (id, a, o, i) , t | t, Tcomp_ptr (id, a, o, i) ->
    let s = expand_comp_ptr id i o in
    let pointer_s = Tpointer (s, o, noattr) in
   (* printf "s: %a@\n" print_ctypes s;
      printf "t: %a@\n" print_ctypes t; *)
    eq_type pointer_s t
  (* TODO:
     Tplus, Tplus ->
     Tpointer, Tcomp_ptr ->
     Tcomp_ptr, Tplus ->
  *)
  | _ ->
    (*    printf "t1: %a@\n" print_ctypes ty1;
          printf "t2: %a@\n" print_ctypes ty2; *)
    failwith "eq_type: not implemented."
and eq_fieldlist (fl1: fieldlist) (fl2: fieldlist) : constr list =
  match (fl1, fl2) with
  | (Fnil, Fnil) -> []
  | (Fcons (id1, ty1, fl1'), Fcons (id2, ty2, fl2')) ->
    if ((compare id1 id2) = 0) then
      (eq_type ty1 ty2) @ (eq_fieldlist fl1' fl2')
    else
      failwith "eq_fieldlist: not equal"
  | _ -> failwith "eq_fieldlist: not equal"

let rec add_type (ty1:coq_type) (ty2:coq_type) : (coq_type * constr list) =
  match (ty1, ty2) with
  | ((Tvoid | Tint _ | Tlong _ | Tfloat _ ),
     (Tvoid | Tint _ | Tlong _ | Tfloat _)) ->
    (ty1, [])
  | (Tpointer (t1, o1, a), Tpointer (t2, o2, _)) ->
    let (t, c) = add_type t1 t2 in
    (* printf "o1: %a@\n" print_ownership o1;
       printf "o2: %a@\n" print_ownership o2; *)
    (Tpointer (t, owner_plus o1 o2, a), c)
  | (Tarray (t1, i, o1, a), Tarray (t2, _, o2, _)) ->
    let (t, c) = add_type t1 t2 in
    (Tarray (t, i, owner_plus o1 o2, a), c)
  | (Tstruct (id1, fl1, a1), Tstruct (id2, fl2, a2)) ->
    if ((compare id1 id2) = 0)
    then let (fl, c) = add_fieldlist fl1 fl2 in
         (Tstruct (id1, fl, a1), c)
    else failwith "add_type: id1 and id2 are not same"
  | Tcomp_ptr (id1, a1, o1, i1), Tcomp_ptr (id2, a2, o2, i2) ->
    if ((compare id1 id2) = 0)
    then (Tplus ([(id1, i1); (id2, i2)], owner_plus o1 o2), [])
    else failwith "id1 and id2 not equal"
  | Tcomp_ptr (id, a, o, i), Tplus (((id',i')::l), o')
  | Tplus (((id',i')::l), o'), Tcomp_ptr (id, a, o, i)
    ->
    (* Do something? *)
    if compare id id' = 0 then (Tplus(((id',i')::(id,i)::l), owner_plus o o'), [])
    else failwith "Tcomp_ptr + tplus: id not equal."
  | Tplus(((id1,i1)::l1), o1), Tplus(((id2,i2)::l2), o2) ->
    if compare id1 id2 = 0
    then (Tplus(((id1,i1)::(id2,i2)::(l1@l2)), owner_plus o1 o2), [])
    else failwith "Tplus + Tplus: id not equal"
  | Tcomp_ptr (id, a, o, i), t | t, Tcomp_ptr (id, a, o, i) ->
    let s = expand_comp_ptr id i o in
    let pointer_s = Tpointer (s, o, noattr) in
    add_type pointer_s t
  (* Tplus, Tstruct -> all expand *)
  | _ ->
    printf "ty1: %a@\n" print_ctypes ty1;
    printf "ty2: %a@\n" print_ctypes ty2;
    failwith "add_type: not implemented."
and add_fieldlist fl1 fl2 =
  match (fl1, fl2) with
  | (Fnil, Fnil) -> (Fnil, [])
  | (Fcons(id1, ty1, fl1'), Fcons(id2, ty2, fl2')) ->
    if ((compare id1 id2) = 0)
    then
      let (t, c0) = add_type ty1 ty2 in
      let (f, c1) = add_fieldlist fl1' fl2' in
      (Fcons(id1, t, f), c0 @ c1)
    else
      failwith "add_fieldlist: not same"
  | _ -> failwith "add_fieldlist: not same"

let rec eq_env env1 env2 : constr list =
  let sort_env1 = List.sort compare env1 in
  let sort_env2 = List.sort compare env2 in
  let rec iter l1 l2 =
    match l1, l2 with
    | [], [] -> []
    | (id1, ty1) :: tl1, (id2, ty2) :: tl2 ->
      (*printf "ty1; %a" print_ctypes ty1;
      printf "ty2: %a@\n" print_ctypes ty2;*)
      if ((compare id1 id2) = 0) then
        let c = eq_type ty1 ty2 in
        c @ (iter tl1 tl2)
      else
        failwith "eq_env: not equal"
    | _ -> failwith "eq_env: can't be equal"
  in
  iter sort_env1 sort_env2

let rec eq_env_list env_list : constr list =
  match env_list with
  | [] -> []
  | [hd] -> []
  | hd1 :: hd2 :: tl ->
    let c = eq_env hd1 hd2 in
    c @ eq_env_list (hd2 :: tl)

let first_ownership_of_ty (ty: coq_type) : coq_type * ownership =
  match ty with
  | Tpointer (t, o, _) -> (t, o)
  | Tarray (t, _, o, _) -> (t, o)
  | _ ->
    (* printf "first_ownership: %a@\n" print_ctypes ty; *)
    failwith "first_ownership_of_ty: ty must be Tpointer or Tarray"

let rec field_name (fn: ident) (fl: fieldlist) : coq_type =
  match fl with
  | Fnil -> failwith "fl must be Fcons"
  | Fcons(id, ty, fl) ->
    if (id = fn) then
      ty
    else
      field_name fn fl

let rec convert_to_typelist (typel: coq_type list) : typelist =
  match typel with
  | [] -> Tnil
  | hd :: tl ->
    Tcons (hd, convert_to_typelist tl)

let rec convert_to_list_of_type (tl: typelist) : coq_type list =
  match tl with
  | Tnil -> []
  | Tcons (hd, tl) ->
    hd :: convert_to_list_of_type tl

let rec variables_of_expr (e: expr) : ident list =
  match e with
  | Econst_int _| Econst_float _ | Econst_single _| Econst_long _ -> []
  | Evar (id, ty) -> [id]
  | Etempvar (id, ty) -> [id]
  | Ederef (e, ty) ->
    variables_of_expr e
  | Eaddrof (e, ty) ->
    variables_of_expr e
  | Eunop (op, e, ty) ->
    variables_of_expr e
  | Ebinop (op, e1, e2, ty) ->
    variables_of_expr e1 @ variables_of_expr e2
  | Ecast (e, ty) ->
    variables_of_expr e
  | Efield (e, id, ty) ->
    variables_of_expr e


(** expr **)
let rec gen_constr_expr being_written e env : constr list * coq_type =
  match e with
  | Evar (id, _) | Etempvar (id, _) ->
    let ty = Type_env.find id env in
    ([], ty)
  | Ederef (a1, _) ->
    (* TO: correct? *)
    let (c, ty) = gen_constr_expr false a1 env in
    begin
      match ty with
      | Tpointer (t, o, _) ->
        if being_written then
          ((writable_ownership o) :: c, t)
        else
          ((readable_ownership o) :: c, t)
      | _ -> failwith "gen_constr_expr < Ederef:a type of Ederef must be Tpointer"
    end
  (** base type is Tvoid **)
  | Econst_int (_, _) | Econst_float (_, _)
  | Econst_single (_, _) | Econst_long (_, _) ->
    ([], Tvoid)
  (** pointer not considered **)
  | Eunop (_, e, _) ->
    let (c, ty) = gen_constr_expr false e env in
    (c, Tvoid)
  | Ebinop (_, e1, e2, _) ->
    let (c1, ty1) = gen_constr_expr false e1 env in
    let (c2, ty2) = gen_constr_expr false e2 env in
    (c1 @ c2, Tvoid)
  | Efield (a1, fn, _) ->
    let (c, ty) = gen_constr_expr false a1 env in
    begin
      match ty with
      | Tstruct (id, fl, _) | Tunion (id, fl, _) ->
          let ty = field_name fn fl in
          (c, ty)
      | _ -> failwith "type of Efiled must be Tstruct or Tunion"
    end
  | Eaddrof (_, _) | Ecast (_,_) ->
    failwith "gen_constr_expr: not_implemented"

(** stmt **)
let rec gen_constr_stmt s pre_env break_env continue_env fun_id =
  (* printf "%a@\n" print_env pre_env; *)
  (* let (path, loc) = !current_loc in
     printf "path: %s, loc: %i@\n" path loc; *)
  match s with
  | Sskip ->
    ([], pre_env)
  | Sassign (e1, e2) ->
    let (c1, ty1) = gen_constr_expr true e1 pre_env in
    let (c2, ty2) = gen_constr_expr false e2 pre_env in
(*    printf "ty1:%a@\n" print_ctypes ty1;
      printf "ty2:%a@\n" print_ctypes ty2;*)
    let ty1_is_empty = empty_type ty1 in
    (* printf "ty1_empty: %a@\n" print_constrs ty1_is_empty; *)
    let ol_of_ty1 = oid_of_ty ty1 in
    let ol_of_ty2 = oid_of_ty ty2 in
    (* TODO: should rename comp_ptr ? *)
    let il1 = ident_int_of_type ty1 in
    let il2 = ident_int_of_type ty2 in
    let new_env =
      rename_env pre_env (ol_of_ty1 @ ol_of_ty2) (il1 @ il2) in
    let (c3, ty3) = gen_constr_expr true e1 new_env in
    let (c4, ty4) = gen_constr_expr false e2 new_env in
(*    printf "ty3:%a@\n" print_ctypes ty3;
      printf "ty4:%a@\n" print_ctypes ty4; *)
    let (add_t, c5) = add_type ty3 ty4 in
(*   printf "add_t:%a@\n" print_ctypes add_t; *)
    let c6 = eq_type ty2 add_t in
    all_env := new_env :: !all_env;
    (c1 @ c2 @ ty1_is_empty @ c3 @ c4 @ c5 @ c6, new_env)
  | Sset (id, e2) ->
    (* TO: id (temp var) must be empty *)
    let t0 = Type_env.find id pre_env in
    let c0 = empty_type t0 in
    let (c1, t1) = gen_constr_expr false e2 pre_env in
    all_env := pre_env :: !all_env;
    (c0 @ c1, pre_env)
  | Ssequence (s1, s2) ->
    let (c1, env1) =
      gen_constr_stmt s1 pre_env break_env continue_env fun_id in
    let (c2, env2) =
      gen_constr_stmt s2 env1 break_env continue_env fun_id in
    (c1 @ c2, env2)
  | Sifthenelse (e, s1, s2) ->
    let (c1, ty1) = gen_constr_expr false e pre_env in
    let (c2, env2) =
      gen_constr_stmt s1 pre_env break_env continue_env fun_id in
    let (c3, env3) =
      gen_constr_stmt s2 pre_env break_env continue_env fun_id in
    let c4 = eq_env env2 env3 in
    (c1 @ c2 @ c3 @ c4, env3)
  | Sswitch (e, cases) ->
    assert (cases != LSnil);
    let renamed_env = rename_all_ownership_env pre_env in
    let (c1, env1) = gen_constr_expr false e pre_env in
    let rec iter l =
      match l with
      | LSnil -> []
      | LScons (_, s, ls) ->
        let (c, env) =
          gen_constr_stmt s pre_env renamed_env continue_env fun_id in
       (* printf "@\n ----- @\n";
          printf "@[%a@]" print_env env;
          printf "@\n ----- @\n"; *)
        (c, env) :: iter ls
    in
    let l = iter cases in
    let constr_list = List.fold_left
      (fun r (c, e) -> c @ r) [] l in
    let env_list = List.fold_left
      (fun r (c, e) -> e :: r) [] l in
    let c2 = eq_env_list (renamed_env :: env_list) in
    (constr_list @ c2 @ c1, List.hd env_list)
  | Sloop(s1, s2) ->
    let renamed_env =
      rename_all_ownership_env pre_env in
    let (c1, env1) =
      gen_constr_stmt s1 pre_env renamed_env pre_env fun_id in
    let (c2, env2) =
      gen_constr_stmt s2 env1 renamed_env pre_env fun_id in
    let c3 = eq_env pre_env env2 in
    (c1@c2@c3, renamed_env)
  | Sbreak ->
    let renamed_env =
      rename_all_ownership_env pre_env in
    let c0 = eq_env break_env pre_env in
    (c0, renamed_env)
  | Scontinue ->
    let renamed_env =
      rename_all_ownership_env pre_env in
    let c0 = eq_env continue_env pre_env in
    (c0, renamed_env)
(*
  | Slabel(lbl, s1) ->
    gen_constr_stmt s1 pre_env
  | Sgoto lbl ->
    ([], pre_env)
*)
  | Scall (None, Evar (id, _), [e]) when (extern_atom id) = "free" ->
    let (c0, t0) = gen_constr_expr false e pre_env in
    let (t1, o0) = first_ownership_of_ty t0 in
    let c1 = [writable_ownership o0] in
    let c2 = empty_type t1 in
    let ol = oid_of_ty t0 in
    let new_env = rename_env pre_env ol [] in
    let (c3, t2) = gen_constr_expr false e new_env in
    let c4 = empty_type t2 in
    all_env := new_env :: !all_env;
    (c0 @ c1 @ c2 @ c3 @ c4, new_env)
  | Scall (Some id0, Evar (id1, _), [e]) when (extern_atom id1) = "malloc" ->
    let (c0, t0) = gen_constr_expr false e pre_env in
    let t1 = Type_env.find id0 pre_env in
    let c1 = empty_type t1 in
    let ol = oid_of_ty t1 in
    let new_env = rename_env pre_env ol [] in
    let (c2, t2) = gen_constr_expr false e new_env in
    let t3 = Type_env.find id0 new_env in
    let (t4, o0) = first_ownership_of_ty t3 in
    let c3 = [writable_ownership o0] in
    let c4 = empty_type t4 in
    all_env := new_env :: !all_env;
    (c0 @ c1 @ c2 @ c3 @ c4, new_env)
  | Scall (None, Evar (id, _), [e1;e2]) when (extern_atom id) = "assert" ->
    let (c0, t0) = gen_constr_expr false e1 pre_env in
    let (c1, t1) = gen_constr_expr false e2 pre_env in
    let ol = (oid_of_ty t0) @ (oid_of_ty t1) in
    let new_env = rename_env pre_env ol [] in
    let (c2, t2) = gen_constr_expr false e1 new_env in
    let (c3, t3) = gen_constr_expr false e2 new_env in
    let (t4, c4) = add_type t0 t1 in
    let (t5, c5) = add_type t2 t3 in
    let c6 = eq_type t4 t5 in
    all_env := new_env :: !all_env;
    (c0@c1@c2@c3@c4@c5@c6, new_env)
  | Scall (None, Evar (id, _), [e])
      when (extern_atom id) = "assert_null" ->
    begin
      match e with
      | Evar _ ->
        let (c0, t0) = gen_constr_expr false e pre_env in
        let ol = oid_of_ty t0 in
        let il = ident_int_of_type t0 in
        let new_env = rename_env pre_env ol il in
        all_env := new_env :: !all_env;
        (c0, new_env)
      | _ ->
        failwith "assert_null: parameter must be variable"
    end
  | Scall (None, e1, el) ->
    let id_list =
      List.fold_left (fun a b ->
        variables_of_expr b @ a) [] el in
    let post_env =
      List.map
        (fun (id, ty) ->
          if (List.mem id id_list) then
            (id, rename_all_comp_ptr (rename_all_ownership_type ty))
          else
            (id, ty)
        ) pre_env in
    let (c1, ty1) = gen_constr_expr false e1 pre_env in
    let (c2, pretl, posttl) =
      List.fold_left (fun (cs, pretl, posttl) e ->
        let (c1, pret) = gen_constr_expr false e pre_env in
        let (c2, postt) = gen_constr_expr false e post_env in
        (c1 @ c2 @ cs, pret :: pretl, postt :: posttl)
      ) ([], [], []) el in
    let pretl = List.rev pretl in
    let posttl = List.rev posttl in
    begin
      match ty1 with
      | Tfunction (tl, tl', ty, _) ->
        let rec iter tl1 tl2 =
          match tl1, tl2 with
          | Tnil, [] -> []
          | Tcons (ty, tl1), hd :: tl2 ->
            let c = eq_type ty hd in
            c @ iter tl1 tl2
          | _ -> failwith "typelist not equal"
        in
        let c3 = iter tl pretl in
        let c4 = iter tl' posttl in
        let c5 = empty_type ty in
        all_env := post_env :: !all_env;
        (c1@c2@c3@c4@c5, post_env)
      | _ -> failwith "ty must be Tfunction"
    end
  | Scall (Some id, e1, el) ->
    (* (pre: id is empty)
       (post: type of id is return type of function *)
    let id_list =
      List.fold_left (fun a b ->
        variables_of_expr b @ a) [] el in
    let post_env =
      List.map
        (fun (id, ty) ->
          if (List.mem id id_list) then
            (id, rename_all_comp_ptr (rename_all_ownership_type ty))
          else
            (id, ty)
        ) pre_env in
    let t0 = Type_env.find id pre_env in
    let c0 = empty_type t0 in
    let ol = oid_of_ty t0 in
    let il = ident_int_of_type t0 in
    let post_env = rename_env post_env ol il in
    let (c1, t1) = gen_constr_expr false e1 pre_env in
    let (c2, pretl, posttl) =
      List.fold_left (fun (cs, pretl, posttl) e ->
        let (c1, pret) = gen_constr_expr false e pre_env in
        let (c2, postt) = gen_constr_expr false e post_env in
        (c1 @ c2 @ cs, pret :: pretl, postt :: posttl)
      ) ([], [], []) el in
    let pretl = List.rev pretl in
    let posttl = List.rev posttl in
    begin
      match t1 with
      | Tfunction (tl, tl', ty, _) ->
        let rec iter tl1 tl2 =
          match tl1, tl2 with
          | Tnil, [] -> []
          | Tcons (ty, tl1), hd :: tl2 ->
            let c = eq_type ty hd in
            c @ iter tl1 tl2
          | _ -> failwith "typelist not equal"
        in
        let c3 = iter tl pretl in
        let c4 = iter tl' posttl in
        let t2 = Type_env.find id post_env in
        let c5 = eq_type t2 ty in
        all_env := post_env :: !all_env;
        (c0 @ c1 @ c2 @ c3 @ c4 @ c5, post_env)
      | _ -> failwith "ty must be Tfunction"
    end
  | Sbuiltin (None, ef, tyargs, el) ->
    (match ef with
    | EF_annot (id, al) ->
      let s = extern_atom id in
      let sl = Str.split (Str.regexp ":") s in
      let path = List.nth sl 1 in
      let loc = int_of_string (List.nth sl 2) in
      if ((compare !start_loc ("", -1)) = 0)
      then
        begin
          start_loc := (path, (loc - 1));
          update_loc path loc;
          ([], pre_env)
        end
      else
        begin
          update_loc path loc;
          ([], pre_env)
        end
    | _ ->
      (* TO: imcomplete *)
      printf "builtin %a called @\n" print_external_function ef;
      ([], pre_env))
  | Sbuiltin (Some id, ef, tyargs, el) ->
    (* TO: imcomplete *)
    printf "builtin %a called @\n" print_external_function ef;
    ([], pre_env)
  | Sreturn None ->
    (* printf "return @\n"; *)
    ([], pre_env)
  | Sreturn (Some e) ->
    let (c0, t0) = gen_constr_expr false e pre_env in
    (* constr: pre_e = post_e + return type of function *)
    let ol = oid_of_ty t0 in
    let il = ident_int_of_type t0 in
    let new_env = rename_env pre_env ol il in
    let (c1, t1) = gen_constr_expr false e new_env in
    let t2 = Type_env.find fun_id new_env in
    let t3 =
      match t2 with
      | Tfunction (_, _, return, _) -> return
      | _ ->
        failwith "gen_constr_stmt > Sreturn: must be Tfunctin"
    in
    let (t4, c2) = add_type t1 t3 in
    let c3 = eq_type t0 t4 in
    (* printf "return @\n"; *)
    all_env := new_env :: !all_env;
    (c0 @ c1 @ c2 @ c3, new_env)
  | _ -> failwith
    "gen_constr_stmt: not implemented"

(** fun:
    {fn_params;
     fn_vars;
     fn_body;
     fn_return;
     fn_callconv;
     fn_temps }
**)
let gen_constr_fun id f c env =
  let fun_ty = Type_env.find id env in
  let post_l =
    match fun_ty with
    | Tfunction (_, post_tl, _, _) ->
       convert_to_list_of_type post_tl
    | _ ->
      failwith "not Tfunction"
  in
  let pre_params = f.fn_params in
  let post_params =
    let rec iter (l1:coq_type list) (l2:(ident * coq_type) list) =
      match l1, l2 with
      | [], [] -> []
      | hd1 :: tl1, hd2 :: tl2 ->
        let (id, t) = hd2 in
        (id, hd1) :: iter tl1 tl2
      | _ -> failwith "not same length l1 l2"
    in
    iter post_l pre_params
  in
  let env2 = List.fold_left
    (fun a (id, ty) ->
      add (id, ty) a ) env pre_params in
  let env3 = List.fold_left
    (fun a (id, ty) ->
      let new_t = rename_all_comp_ptr ty in
      add (id, new_t) a ) env2 f.fn_vars in
  let env4 = List.fold_left
    (fun a (id, ty) ->
      let new_t = rename_all_comp_ptr ty in
      add (id, new_t) a ) env3 f.fn_temps in
  all_env := env4 :: !all_env;
  let c0 = List.fold_left
    (fun c (id, ty) ->
      let c' = empty_type ty in
      c' @ c) [] f.fn_vars in
  let (c, env5) = gen_constr_stmt f.fn_body env4 [] [] id in
  let (path, c_loc) = !current_loc in
  let c0 = change_location c0 !start_loc in
  start_loc := Cutil.no_loc;
  end_loc := (path, (c_loc + 1));
  let c1 = List.fold_left
    (fun c (id, ty) ->
      let ty' = Type_env.find id env5 in
      let c' = eq_type ty ty' in
      c @ c') [] post_params in
  let c1 = change_location c1 !end_loc in
  let c2 =
    List.fold_left
      (fun a (id, ty) ->
        let ty0 = find id env5 in
        let c = empty_type ty0 in
        a @ c
      ) [] f.fn_vars
  in
  let c2 = change_location c2 !end_loc in
  let c3 = List.fold_left
    (fun a (id, ty) ->
      let ty0 = find id env5 in
      let c = empty_type ty0 in
      a @ c
    ) [] f.fn_temps in
  let c3 = change_location c3 !end_loc in
  (* TODO: consider global variables *)
  (c @ c0 @ c1 @ c2 @ c3, env5)

(** fundef: External | Internal **)
let gen_constr_fundef id fd c env =
  match fd with
  | External (ef, tl, ty, _) ->
    (* printf "External: %s@\n" (extern_atom id); *)
    (c, env)
  | Internal f ->
    (* printf "Internal: %s@\n" (extern_atom id); *)
    gen_constr_fun id f c env

(** prog_defs: Gfun | Gvar **)
let gen_constr_globdef (id, gd) c env =
  match gd with
  | Gfun f ->
    gen_constr_fundef id f c env
  | Gvar v ->
    (* TODO: OK? *)
    printf "Gvar: %s@\n" (extern_atom id);
    (c, env)

let declare_struct_or_union p name t =
  fprintf p "%s;@ @ " name

let print_struct_or_union p name t =
  let fld =
    match t with
    | Tstruct (id, fld, a) -> fld
    | Tunion (id, fld, a) -> fld
    | _ -> failwith "not struct"
  in
  fprintf p "@[<v 2>%s {" name;
  let rec print_fields = function
  | Fnil -> ()
  | Fcons(id, ty, rem) ->
      fprintf p "@ %a %s;" print_ctypes ty (extern_atom id);
      print_fields rem in
  print_fields fld;
  fprintf p "@;<0 -2>};@]@ @ "

(* Collecting the names and fields of structs and unions *)
let rec collect_type = function
  | Tvoid -> ()
  | Tint _ -> ()
  | Tfloat _ -> ()
  | Tlong _ -> ()
  | Tpointer(t, o, _) -> collect_type t
  | Tarray(t, _, o, _) -> collect_type t
  | Tfunction(args, _, res, _) -> collect_type_list args; collect_type res
  | Tstruct(id, fld, a) ->
      let s = extern_atom id in
      if not (StructUnion.mem s !struct_unions)
      then
        begin
          struct_unions :=
            StructUnion.add s (fresh_ovar_type (Tstruct(id, fld, a)))
            !struct_unions;
          collect_fields fld
        end
  | Tunion(id, fld, a) ->
      let s = extern_atom id in
      if not (StructUnion.mem s !struct_unions)
      then
        begin
          struct_unions :=
            StructUnion.add s (Tstruct(id, fld, a))
            !struct_unions;
          collect_fields fld
        end
  | Tcomp_ptr _ -> ()
  | Tplus _ -> ()
and collect_type_list = function
  | Tnil -> ()
  | Tcons(hd, tl) -> collect_type hd; collect_type_list tl
and collect_fields = function
  | Fnil -> ()
  | Fcons(id, hd, tl) -> collect_type hd; collect_fields tl
let rec collect_expr e =
  collect_type (typeof e);
  match e with
  | Econst_int _ | Econst_float _ | Econst_single _ | Econst_long _ -> ()
  | Evar _ | Etempvar _ -> ()
  | Ederef(r, _) -> collect_expr r
  | Eaddrof(l, _) -> collect_expr l
  | Eunop(_, r, _) -> collect_expr r
  | Ebinop(_, r1, r2, _) -> collect_expr r1; collect_expr r2
  | Ecast(r, _) -> collect_expr r
  | Efield(l, _, _) -> collect_expr l
let rec collect_stmt = function
  | Sskip -> ()
  | Sassign (e1, e2) -> collect_expr e1; collect_expr e2
  | Sset (id, e) -> collect_expr e;
  | Scall (id, e, el) -> collect_expr e;
    List.iter (fun e -> collect_expr e) el
  | Sbuiltin (id, ef, tl, el) -> collect_type_list tl;
    List.iter (fun e -> collect_expr e) el
  | Ssequence(s1, s2) -> collect_stmt s1; collect_stmt s2
  | Sifthenelse(e, s1, s2) -> collect_expr e; collect_stmt s1; collect_stmt s2
  | Sloop (s1, s2) -> collect_stmt s1; collect_stmt s2
  | Sbreak -> ()
  | Scontinue -> ()
  | Sswitch(e, cases) -> collect_expr e; collect_cases cases
  | Sreturn None -> ()
  | Sreturn (Some e) -> collect_expr e
  | Slabel(lbl, s) -> collect_stmt s
  | Sgoto lbl -> ()
and collect_cases = function
  | LSnil -> ()
  | LScons(lbl, s, rem) -> collect_stmt s; collect_cases rem
let collect_fun f =
  collect_type f.fn_return;
  List.iter (fun (id, ty) -> collect_type ty) f.fn_params;
  List.iter (fun (id, ty) -> collect_type ty) f.fn_vars;
  collect_stmt f.fn_body
let collect_fundef fd =
  match fd with
  | External(_, args, res, _) -> collect_type_list args; collect_type res
  | Internal f -> collect_fun f
let collect_globdef (id, gd) =
  match gd with
  | Gfun fd -> collect_fundef fd
  | Gvar v -> collect_type v.gvar_info
let collect_program p =
  List.iter collect_globdef p.prog_defs

(** add function types to type enviroment **)

let add_fundef fd =
  match fd with
  | External _ -> None
  | Internal f ->
    let pre_params = List.rev (f.fn_params) in
    let post_params =
      List.map
        (fun (id, ty) -> (id, rename_all_comp_ptr (rename_all_ownership_type ty)))
        pre_params
    in
    let pre_params_tl =
      convert_to_typelist (List.map (fun (id, ty) -> ty) pre_params) in
    let post_params_tl =
      convert_to_typelist (List.map (fun (id, ty) -> ty) post_params) in
    let return = f.fn_return in
    let callconv = f.fn_callconv in
    let fun_ty =
      Tfunction (pre_params_tl, post_params_tl, return, callconv) in
    Some (fun_ty)

let add_fundef_globdef (id, gd) =
  match gd with
  | Gfun fd -> let f = add_fundef fd in
               (id, f)
  | Gvar v -> (id, None)

let add_fundef_program p  =
  let rec iter l =
    match l with
    | [] -> []
    | hd :: tl ->
      let (id, f) = add_fundef_globdef hd in
      match f with
      | Some t -> add (id, t) (iter tl)
      | None -> iter tl
  in
  iter p.prog_defs

(** prog: {prog_defs, prog_main} **)
let gen_constr_prog p =
  struct_unions := StructUnion.empty;
  collect_program p;
  (* printf "--- struct --- @\n"; *)
  (* StructUnion.iter (print_struct_or_union std_formatter) !struct_unions; *)
  let e = add_fundef_program p in
  (* printf "---- fundef: %d ---- @\n" (List.length e);
     List.iter (fun e' -> printf "%a@\n" print_id_type e') e; *)
  let (constr, new_env) = List.fold_left
    (fun (c, e) x ->
      let (c0, e0) = gen_constr_globdef x c e in
      (c @ c0, e))
    ([], e) p.prog_defs in
  (* printf "--- all_env: %d --- @\n" (List.length !all_env);
  List.iter (fun e ->
    printf "------@\n";
    List.iter (fun e' -> printf "%a@\n" print_id_type e') e;
  ) (List.rev !all_env);
   printf "--- Hashtbl: %d --- @\n" (Hashtbl.length struct_tbl);
  Hashtbl.iter (fun (s, i) t ->
    printf "%a@\n" print_hashtbl (s, i, t)) struct_tbl;
    printf "--- consrs: %d --- @\n@[%a@]" (List.length constr)
    print_constrs constr; *)
  (constr, new_env, !struct_unions, struct_tbl)
