open Typing
open AST
open Cop
open Clight
open Ctypes
open Type_env
open Ownership
open BinNums
open Integers
open Floats
open OUnit2
open Format


let attr = { attr_volatile= true; attr_alignas= None }
let tint = Tint (I32, Signed, attr)
let id0 = int_to_positive 0
let id1 = int_to_positive 1
let id2 = int_to_positive 2
let id3 = int_to_positive 3
let id4 = int_to_positive 4
let id5 = int_to_positive 5
let id6 = int_to_positive 6
let id7 = int_to_positive 7
let phi0 = OVar (fresh_ovar())
let phi1 = OVar (fresh_ovar())
let phi2 = OVar (fresh_ovar())
let phi3 = OVar (fresh_ovar())
let phi4 = OVar (fresh_ovar())
let phi5 = OVar (fresh_ovar())
let phi6 = OVar (fresh_ovar())
let ty0 = Tpointer (Tvoid, phi0, attr)
let ty1' = Tpointer (tint, phi2, attr)
let ty1 = Tpointer (ty1', phi1, attr)
let field1 = Fcons (id3, tint, Fnil)
let field2 = Fcons (id4, ty0, field1)
let ty2 = Tstruct (id5, field2, attr)
let ty0_is_empty = [Eq (phi0, OConst (0.0))]
let ty1'_is_writable = [Eq (phi2, OConst (1.0))]
let ty1_is_writable = [Eq (phi1, OConst (1.0))]
let ty1_is_readable = [Lt (OConst (0.0), phi1)]
let oplus0 = OPlus (phi0, phi1)
let oplus1 = OPlus (phi2, phi3)
let oplus2 = OPlus (oplus0, oplus1)
let ty3 = Tpointer (Tvoid, oplus0, attr)
let ty4 = Tpointer (Tvoid, phi4, attr)
let ty5 = Tpointer (tint, phi6, attr)
let ty6 = Tpointer (ty5, phi5, attr)

let env = add (id7, ty1') (add (id6, tint) (add (id2, ty2) (add (id1, ty1) (add (id0, ty0) Type_env.empty)))) ;;

let env1 = add (id0, ty0) (add (id1, ty1') (add (id2, ty1) (add (id3, tint) Type_env.empty))) ;;

let env2 = add (id2, ty6) (add (id0, ty4) (add (id3, tint) (add (id1, ty5) Type_env.empty))) ;;


(** gen_constr_expr test **)
let test1 _ = assert_equal
  ([], ty0)
  (gen_constr_expr true (Evar(id0, Tvoid)) env) ;;

let test2 _ = assert_equal
  ([], ty0)
  (gen_constr_expr false (Evar(id0, Tvoid)) env) ;;

let test3 _ =
  let (c, ty) = gen_constr_expr true (Ederef(Evar (id1, Tvoid), Tvoid)) env in
 (* printf "%a @\n" print_ctypes ty;
    List.iter (fun x -> printf "%a @\n" print_constr x) c;
    List.iter (fun x -> printf "%a @\n" print_constr x) ty1_is_writable;
    printf "%a @\n" print_ctypes ty1';
 *)
  assert_equal
    (ty1_is_writable, ty1')
    (c, ty) ;;

let test4 _ =
    let (c, ty) = gen_constr_expr false (Ederef(Evar (id1, Tvoid), Tvoid)) env in
    assert_equal
      (ty1_is_readable, ty1')
      (c, ty) ;;

let test5 _ =
  (** All base type is Tvoid **)
  let (c, ty) = gen_constr_expr false (Econst_int (Int.intval Int.one, Tvoid)) env in
  assert_equal
    ([], Tvoid)
    (c, ty)

let test6 _ =
  let (c, ty) = gen_constr_expr false (Econst_float (Float.zero, Tvoid)) env in
  assert_equal
    ([], Tvoid)
    (c, ty)

let test7 _ =
    let (c, ty) = gen_constr_expr false (Econst_long (Int64.intval Int64.zero, Tvoid)) env in
  assert_equal
    ([], Tvoid)
    (c, ty)

let test8 _ =
  let (c, ty) = gen_constr_expr false
    (Eunop (Onotint,
            Econst_int (Int.intval Int.one, Tvoid),
            Tvoid)) env in
  assert_equal
    ([], Tvoid)
    (c, ty)

let test9 _ =
    let (c, ty) = gen_constr_expr false
    (Eunop (Oabsfloat,
            Econst_float (Float.zero, Tvoid),
            Tvoid)) env in
    assert_equal
      ([], Tvoid)
      (c, ty)

let test10 _ =
  let (c, ty) = gen_constr_expr false
    (Ebinop (Oadd,
             Econst_int (Int.intval Int.one, Tvoid),
             Econst_int (Int.intval Int.one, Tvoid),
             Tvoid)) env in
    assert_equal
      ([], Tvoid)
      (c, ty)

let test11 _ =
  (** id0 == *id1 **)
  let id0 = int_to_positive 0 in
  let id1 = int_to_positive 1 in
  let id6 = int_to_positive 6 in
  let phi0 = OVar (fresh_ovar()) in
  let phi1 = OVar (fresh_ovar()) in
  let phi2 = OVar (fresh_ovar()) in
  let ty0 = Tpointer (Tvoid, phi0, attr) in
  let ty1' = Tpointer (tint, phi2, attr) in
  let ty1 = Tpointer (ty1', phi1, attr) in
  let env = add (id6, tint) (add (id1, ty1) (add (id0, ty0) Type_env.empty)) in
  let e1 = Evar (id0, Tvoid) in
  let e2 = Ederef (Evar (id1, Tvoid), Tvoid) in
  let (c1, ty1) = gen_constr_expr false
    (Ebinop (Oeq, e1, e2, Tvoid)) env in
  let c2 = [Lt (OConst 0.0, phi1)] in
  assert_equal
    (c2, Tvoid)
    (c1, ty1)

(** field name test start **)

let test12 _ =
  let ty = field_name id3 field2 in
  assert_equal ty tint

let test13 _ =
  let ty = field_name id4 field2 in
  assert_equal ty ty0

(** field name test end **)

let test14 _ =
  let (c, ty) = gen_constr_expr false
    (Efield(Evar (id2, Tvoid), id3, Tvoid)) env in
  assert_equal
    ([], tint)
    (c, ty)

(** gen_constr_stmt test **)

let test15 _ =
  let (c, new_env) = gen_constr_stmt Sskip env env env id0 in
  assert_equal
    ([], env)
    (c, new_env)

let test16 _ =
  let e1 = Evar(id6, Tvoid) in
  let e2 = Econst_int(Int.intval Int.zero, Tvoid) in
  let (c, new_env) = gen_constr_stmt (Sassign(e1, e2)) env env env id0 in
  assert_equal
    ([], env)
    (c, new_env)

let test17 _ =
  let e1 = Ederef(Evar(id7, Tvoid), Tvoid) in
  let e2 = Econst_int(Int.intval Int.zero, Tvoid) in
  let (c1, new_env) = gen_constr_stmt (Sassign(e1, e2)) env env env id0 in
  let c2 = uniq c1 in
(*  printf "@\n---c----@\n";
    print_constrs std_formatter c2;
    printf "@\n---ty1---@\n";
    print_constrs std_formatter ty1'_is_writable;
*)
  assert_equal
    (ty1'_is_writable, env)
    (c2, new_env)

let test18 _ =
  (* *id1 = id0 *)
  let e1 = Ederef(Evar(id1, Tvoid), Tvoid) in
  let e2 = Evar(id0, Tvoid) in
  let (c1, new_env) = gen_constr_stmt (Sassign(e1, e2)) env env env id0 in
  let c2 = uniq c1 in
  let pre_id0 = find id0 env in
  let pre_id1 = find id1 env in
  let (t0, o0) = first_ownership_of_ty pre_id0 in
  let (t1, o1) = first_ownership_of_ty pre_id1 in
  let (t2, o2) = first_ownership_of_ty t1 in
  let new_id0 = find id0 new_env in
  let new_id1 = find id1 new_env in
  let (t3, o3) = first_ownership_of_ty new_id0 in
  let (t4, o4) = first_ownership_of_ty new_id1 in
  let (t5, o5) = first_ownership_of_ty t4 in
(*  printf "@\n----c2----@\n";
    print_constrs std_formatter c2;
*)
  let c3 =
    eq_ownership o1 (OConst(1.0)) @
    eq_ownership o2 (OConst(0.0)) @
    eq_ownership o0 (owner_plus o5 o3) @
    eq_ownership o4 o1 in
  let c4 = uniq c3 in
(*
  printf "@\n----c4----@\n";
  print_constrs std_formatter c4;
*)
  assert_equal c2 c4

(** collect_oid test start **)
let test19 _ =
  let ol = collect_oid phi0 in
  assert_equal ol [0]

let test20 _ =
  let ol = collect_oid oplus0 in
  assert_equal ol [0;1]

let test21 _ =
  let ol = collect_oid oplus2 in
  assert_equal ol [0;1;2;3]
(** collect_oid test end **)

(** intersection_of_oid_lists test start **)
let test22 _ =
  let ol1 = collect_oid phi0 in
  let ol = intersection_of_oid_lists ol1 [0] [] in
  assert_equal ol [0]

let test23 _ =
  let ol1 = collect_oid oplus2 in
  let ol = intersection_of_oid_lists ol1 [1;2] [] in
  assert_equal ol [2;1]
(** intersection_of_oid_lists test end **)

(** rename_type test start **)
let test24 _ =
  let new_t = reset(); rename_type ty0 [0] in
  assert_equal
    new_t
    (Tpointer (Tvoid, OVar 0, attr))

let test25 _ =
  let new_t = reset(); rename_type ty0 [3] in
  assert_equal
    new_t
    (Tpointer (Tvoid, phi0, attr))

let test26 _ =
  let new_t = reset(); rename_type ty3 [0] in
  assert_equal
    new_t
    (Tpointer (Tvoid, OVar 0, attr))

(** rename_type test end **)

(** gen_constr_stmt test **)

let test27 _ =
  let e1 = Ederef(Evar(id7, Tvoid), Tvoid) in
  let e2 = Econst_int(Int.intval Int.zero, Tvoid) in
  let s2 = Sassign(e1, e2) in
  let (c1, new_env) =
    gen_constr_stmt (Ssequence (Sskip, s2)) env env env id0 in
  let c2 = uniq c1 in
  assert_equal
    (ty1'_is_writable, env)
    (c2, new_env)

let test28 _ =
  let e1 = Ederef(Evar(id7, Tvoid), Tvoid) in
  let e2 = Econst_int(Int.intval Int.zero, Tvoid) in
  let s1 = Sassign(e1, e2) in
  let (c1, new_env) =
    gen_constr_stmt (Ssequence (s1, Sskip)) env env env id0 in
  let c2 = uniq c1 in
  assert_equal
    (ty1'_is_writable, env)
    (c2, new_env)

let test29 _ =
  let e1 = Evar(id6, Tvoid) in
  let e2 = Econst_int(Int.intval Int.zero, Tvoid) in
  let s1 = Sassign (e1, e2) in
  let e3 = Ederef(Evar(id7, Tvoid), Tvoid) in
  let e4 = Econst_int(Int.intval Int.zero, Tvoid) in
  let s2 = Sassign(e3, e4) in
  let (c1, new_env) =
    gen_constr_stmt (Ssequence (s1, s2)) env env env id0 in
  let c2 = uniq c1 in
  assert_equal
    (ty1'_is_writable, env)
    (c2, new_env)

(** eq_env test **)
let test30 _ =
(*  printf "@\n%a@\n" print_env env1; *)
  let c = eq_env env1 env1 in
  assert_equal [] c

let test31 _ =
  let c = eq_env env1 env2 in
  let c1 = eq_ownership phi0 phi4 @ eq_ownership phi1 phi5 @ eq_ownership phi2 phi6 in
  let c2 = uniq c in
  let c3 = uniq c1 in
  assert_equal c3 c2

(** eq_env test end **)

(** gen_constr_stmt test **)
let test32 _ =
(* pre_env = id6:int, id1:tint ref(phi3) ref(phi2),
   id0:void ref(phi1) *)
(* if (id0 = *id1) then
     id6 = int const
   else
     *id1 = id0
*)
(* constr: phi1 > 0, phi1 = 1, phi0 = phi3 + phi4, phi2 = 0, phi2 = phi3, phi0 = phi4 *)
(* then_env = pre_env *)
(* else_env = id6:int id1:tint ref(phi3) ref(phi1),
   id0:void ref(phi4) *)
  let id0 = int_to_positive 0 in
  let id1 = int_to_positive 1 in
  let id6 = int_to_positive 6 in
  let id7 = int_to_positive 7 in
  let phi0 = OVar (fresh_ovar()) in
  let phi1 = OVar (fresh_ovar()) in
  let phi2 = OVar (fresh_ovar()) in
  let phi3 = OVar (fresh_ovar()) in
  let ty0 = Tpointer (Tvoid, phi0, attr) in
  let ty1' = Tpointer (tint, phi2, attr) in
  let ty1 = Tpointer (ty1', phi1, attr) in
  let ty2 = Tpointer (Tvoid, phi3, attr) in
  let env = add (id7, ty2) (add (id6, tint) (add (id1, ty1) (add (id0, ty0) Type_env.empty))) in
  let e = Ebinop (Oeq, Evar (id0, Tvoid), Ederef (Evar (id1, Tvoid), Tvoid), Tvoid) in
  let e1 = Evar(id6, Tvoid) in
  let e2 = Econst_int(Int.intval Int.zero, Tvoid) in
  let s1 = Sassign (e1, e2) in
  let e3 = Ederef(Evar(id1, Tvoid), Tvoid) in
  let e4 = Evar(id0, Tvoid) in
  let s2 = Sassign (e3, e4) in
  let (c1, new_env) =
    gen_constr_stmt (Sifthenelse(e, s1, s2)) env env env id0 in
  let c2 = uniq c1 in
  (** id0 = *id1 **)
  let c3 = [Lt (OConst 0.0, phi1)] in
  let pre_id0 = find id0 env in
  let pre_id1 = find id1 env in
  let (t0, o0) = first_ownership_of_ty pre_id0 in
  let (t1, o1) = first_ownership_of_ty pre_id1 in
  let (t2, o2) = first_ownership_of_ty t1 in
  let new_id0 = find id0 new_env in
  let new_id1 = find id1 new_env in
  let (t3, o3) = first_ownership_of_ty new_id0 in
  let (t4, o4) = first_ownership_of_ty new_id1 in
  let (t5, o5) = first_ownership_of_ty t4 in
  (** *id1 = id0 **)
  let c5 =
    eq_ownership o1 (OConst(1.0)) @
    eq_ownership o2 (OConst(0.0)) @
    eq_ownership o0 (owner_plus o5 o3) in
  (** equal environment **)
  let c6 =
    eq_ownership o0 o3 @
    eq_ownership o1 o4 @
    eq_ownership o2 o5 in
  let c7 = uniq (c3 @ c5 @ c6) in
(*
  printf "@\n pre_env: %a" print_env env;
  printf "@\n new_env: %a" print_env new_env;
  printf "@\n c2: %a" print_constrs c2;
  printf "@\n c7: %a" print_constrs c7;
*)
  assert_equal c2 c7

(** eq_env_list test **)
let test33 _ =
  let id0 = int_to_positive 0 in
  let id1 = int_to_positive 1 in
  let phi0 = OVar (fresh_ovar()) in
  let phi1 = OVar (fresh_ovar()) in
  let phi2 = OVar (fresh_ovar()) in
  let phi3 = OVar (fresh_ovar()) in
  let phi4 = OVar (fresh_ovar()) in
  let phi5 = OVar (fresh_ovar()) in
  let phi6 = OVar (fresh_ovar()) in
  let phi7 = OVar (fresh_ovar()) in
  let phi8 = OVar (fresh_ovar()) in
  let ty0 = Tpointer (Tvoid, phi0, attr) in
  let ty1' = Tpointer (tint, phi2, attr) in
  let ty1 = Tpointer (ty1', phi1, attr) in
  let ty2 = Tpointer (Tvoid, phi3, attr) in
  let ty3' = Tpointer (tint, phi5, attr) in
  let ty3 = Tpointer (ty3', phi4, attr) in
  let ty4 = Tpointer (Tvoid, phi6, attr) in
  let ty5' = Tpointer (tint, phi8, attr) in
  let ty5 = Tpointer (ty5', phi7, attr) in
  let e0 = add (id1, ty1) (add (id0, ty0) Type_env.empty) in
  let e1 = add (id1, ty3) (add (id0, ty2) Type_env.empty) in
  let e2 = add (id1, ty5) (add (id0, ty4) Type_env.empty) in
  let e3 = [e0; e1; e2] in
  let c0 = eq_env_list e3 in
  let c1 = uniq c0 in
  let c2 =
    eq_ownership phi0 phi3 @
    eq_ownership phi1 phi4 @
    eq_ownership phi2 phi5 @
    eq_ownership phi3 phi6 @
    eq_ownership phi4 phi7 @
    eq_ownership phi5 phi8 in
  let c3 = uniq c2 in
  assert_equal c1 c3

(** gen_constr_stmt test **)
(** TODO: Sswitch test **)
(*let test34 _ =

  (** switch *id2
      case 1 : **id1 = int
      case 2 : *id1 = id0
      default : skip
  **)
  let _ = Ownership.reset() in
  let id0 = int_to_positive 0 in
  let id1 = int_to_positive 1 in
  let id2 = int_to_positive 2 in
  let phi0 = OVar (fresh_ovar()) in
  let phi1 = OVar (fresh_ovar()) in
  let phi2 = OVar (fresh_ovar()) in
  let phi3 = OVar (fresh_ovar()) in
  let ty0 = Tpointer (Tvoid, phi0, attr) in
  let ty1' = Tpointer (tint, phi2, attr) in
  let ty1 = Tpointer (ty1', phi1, attr) in
  let ty2 = Tpointer (tint, phi3, attr) in
  let env = add (id2, ty2) (add (id1, ty1) (add (id0, ty0) Type_env.empty)) in
  printf "@\npre_env:@\n%a" print_env env;
  let e = Ederef (Evar (id2, Tvoid), Tvoid) in
  let e1 = Econst_int(Int.intval Int.zero, Tvoid) in
  let s1 = Sassign
    (Ederef (Ederef (Evar (id1, Tvoid), Tvoid), Tvoid), e1) in
  let s2 = Sassign
    (Ederef (Evar (id1, Tvoid), Tvoid),
     Evar (id0, Tvoid)) in
  let ls = LScons ((Some (Zpos id1)), s1,
                   (LScons ((Some (Zpos id2)), s2,
                    (LScons (None, Sskip, LSnil)))))
  in
  let s = Sswitch (e,ls)  in
  let (c0, new_env) = gen_constr_stmt s env in
  let c1 = uniq c0 in
  (** *id2 **)
  let c2 = [readable_ownership phi3] in
  (** **id1 = int **)
  let c3 = [readable_ownership phi1] in
  let c4 = eq_ownership phi2 (OConst 1.0) in
  (** *id1 = id0 **)
  let c5 = eq_ownership phi1 (OConst 1.0) in
  let c6 = eq_ownership phi2 (OConst 0.0) in
  let c7 = eq_ownership phi0 (owner_plus (OVar 4) (OVar 5)) in
  (** eq_env **)
  let c8 =
    eq_ownership (OVar 2) (OVar 4) @
    eq_ownership (OVar 0) (OVar 5) in
  let c9 = uniq (c2@c3@c4@c5@c6@c7@c8) in
  printf "@\nc1:@\n%a" print_constrs c1;
  printf "@\nc9:@\n%a" print_constrs c9;
  assert_equal
    (c1, new_env)
    (c9, env)
*)


(** TODO: function test **)
(*
let test35 _ =
  (**
     ---
     pre_env:
     id0: void ref phi0
     id1: (int ref phi2) ref phi1
     id2: int ref phi3
     id3: function (void ref, int ref ref, int ref) ->
                   (void ref, int ref ref, int ref)
          : int
     ---
     fun (id0, id1, id2*)
  let id0 = int_to_positive 0 in
  let id1 = int_to_positive 1 in
  let id2 = int_to_positive 2 in
  let id3 = int_to_positive 3 in
  let phi0 = OVar (fresh_ovar()) in
  let phi1 = OVar (fresh_ovar()) in
  let phi2 = OVar (fresh_ovar()) in
  let phi3 = OVar (fresh_ovar()) in
  let phi4 = OVar (fresh_ovar()) in
  let phi5 = OVar (fresh_ovar()) in
  let phi6 = OVar (fresh_ovar()) in
  let phi7 = OVar (fresh_ovar()) in
  let phi8 = OVar (fresh_ovar()) in
  let phi9 = OVar (fresh_ovar()) in
  let phi10 = OVar (fresh_ovar()) in
  let phi11 = OVar (fresh_ovar()) in
  let tint = Tint (I32, Signed, attr) in
  let ty0 = Tpointer (Tvoid, phi0, attr) in
  let ty1' = Tpointer (tint, phi2, attr) in
  let ty1 = Tpointer (ty1', phi1, attr) in
  let ty2 = Tpointer (tint, phi3, attr) in
  let ty3 = Tpointer (Tvoid, phi4, attr) in
  let ty4' = Tpointer (tint, phi6, attr) in
  let ty4 = Tpointer (ty4', phi5, attr) in
  let ty5 = Tpointer (tint, phi7, attr) in
  let ty6 = Tpointer (Tvoid, phi8, attr) in
  let ty7' = Tpointer (tint, phi10, attr) in
  let ty7 = Tpointer (ty7', phi9, attr) in
  let ty8 = Tpointer (tint, phi11, attr) in
  let tl1 =
    Tcons (
  let ty3 = Tfunction
  let env = add (id2, ty2) (add (id1, ty1) (add (id0, ty0) Type_env.empty)) in
  let el =
    Evar (id0, Tvoid) ::
    Evar (id1, Tvoid) ::
    Ederef (Evar (id2, Tvoid), Tvoid) :: [] in
  let e =
*)





let suite =
  "suite">:::
    ["test1">:: test1;
     "test2">:: test2;
     "test3">:: test3;
     "test4">:: test4;
     "test5">:: test5;
     "test6">:: test6;
     "test7">:: test7;
     "test8">:: test8;
     "test9">:: test9;
     "test10">:: test10;
     "test11">:: test11;
     "test12">:: test12;
     "test13">:: test13;
     "test14">:: test14;
     "test15">:: test15;
     "test16">:: test16;
     "test17">:: test17;
     "test18">:: test18;
     "test19">:: test19;
     "test20">:: test20;
     "test21">:: test21;
     "test22">:: test22;
     "test23">:: test23;
     "test24">:: test24;
     "test25">:: test25;
     "test26">:: test26;
     "test27">:: test27;
     "test28">:: test28;
     "test29">:: test29;
     "test30">:: test30;
     "test31">:: test31;
     "test32">:: test32;
     "test33">:: test33;
(*   "test34">:: test34; *)
    ]
let _ =
  Printf.printf "Testing...\n";
  run_test_tt_main suite
