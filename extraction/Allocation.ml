open AST
open Archi
open BinNums
open BinPos
open Conventions
open Conventions1
open Coqlib
open Datatypes
open Errors
open FSetAVLplus
open Int0
open Integers
open Kildall
open LTL
open Lattice
open List0
open Locations
open Machregs
open Maps
open Op
open Ordered
open RTL
open RTLtyping
open Registers

type moves = (loc * loc) list

type block_shape =
| BSnop of moves * LTL.node
| BSmove of reg * reg * moves * LTL.node
| BSmakelong of reg * reg * reg * moves * LTL.node
| BSlowlong of reg * reg * moves * LTL.node
| BShighlong of reg * reg * moves * LTL.node
| BSop of operation * reg list * reg * moves * mreg list * mreg * moves
   * LTL.node
| BSopdead of operation * reg list * reg * moves * LTL.node
| BSload of memory_chunk * addressing * reg list * reg * moves * mreg list
   * mreg * moves * LTL.node
| BSloaddead of memory_chunk * addressing * reg list * reg * moves * LTL.node
| BSload2 of addressing * addressing * reg list * reg * moves * mreg list
   * mreg * moves * mreg list * mreg * moves * LTL.node
| BSload2_1 of addressing * reg list * reg * moves * mreg list * mreg * 
   moves * LTL.node
| BSload2_2 of addressing * addressing * reg list * reg * moves * mreg list
   * mreg * moves * LTL.node
| BSstore of memory_chunk * addressing * reg list * reg * moves * mreg list
   * mreg * LTL.node
| BSstore2 of addressing * addressing * reg list * reg * moves * mreg list
   * mreg * moves * mreg list * mreg * LTL.node
| BScall of signature * (reg, ident) sum * reg list * reg * moves
   * (mreg, ident) sum * moves * LTL.node
| BStailcall of signature * (reg, ident) sum * reg list * moves
   * (mreg, ident) sum
| BSbuiltin of external_function * reg list * reg * moves * mreg list
   * mreg list * moves * LTL.node
| BSannot of ident * annot_arg list * reg list * reg * moves * loc list
   * LTL.node
| BScond of condition * reg list * moves * mreg list * LTL.node * LTL.node
| BSjumptable of reg * moves * mreg * LTL.node list
| BSreturn of reg option * moves

(** val extract_moves : moves -> bblock -> moves * bblock **)

let rec extract_moves accu b = match b with
| [] -> ((rev accu), b)
| i :: b' ->
  (match i with
   | Lop (op, args, res0) ->
     (match is_move_operation op args with
      | Some arg -> extract_moves (((R arg), (R res0)) :: accu) b'
      | None -> ((rev accu), b))
   | Lgetstack (sl, ofs, ty, dst) ->
     extract_moves (((S (sl, ofs, ty)), (R dst)) :: accu) b'
   | Lsetstack (src, sl, ofs, ty) ->
     extract_moves (((R src), (S (sl, ofs, ty))) :: accu) b'
   | _ -> ((rev accu), b))

(** val check_succ : LTL.node -> bblock -> bool **)

let check_succ s = function
| [] -> false
| i :: l ->
  (match i with
   | Lbranch s' -> (fun x -> x) (peq s s')
   | _ -> false)

type operation_kind =
| Coq_operation_Omove of reg
| Coq_operation_Omakelong of reg * reg
| Coq_operation_Olowlong of reg
| Coq_operation_Ohighlong of reg
| Coq_operation_other of operation * reg list

(** val classify_operation : operation -> reg list -> operation_kind **)

let classify_operation op args =
  match op with
  | Omove ->
    let op0 = Omove in
    (match args with
     | [] -> Coq_operation_other (op0, [])
     | arg :: l ->
       (match l with
        | [] -> Coq_operation_Omove arg
        | r :: l0 -> Coq_operation_other (op0, (arg :: (r :: l0)))))
  | Omakelong ->
    let op0 = Omakelong in
    (match args with
     | [] -> Coq_operation_other (op0, [])
     | arg1 :: l ->
       (match l with
        | [] -> Coq_operation_other (op0, (arg1 :: []))
        | arg2 :: l0 ->
          (match l0 with
           | [] -> Coq_operation_Omakelong (arg1, arg2)
           | r :: l1 ->
             Coq_operation_other (op0, (arg1 :: (arg2 :: (r :: l1)))))))
  | Olowlong ->
    let op0 = Olowlong in
    (match args with
     | [] -> Coq_operation_other (op0, [])
     | arg :: l ->
       (match l with
        | [] -> Coq_operation_Olowlong arg
        | r :: l0 -> Coq_operation_other (op0, (arg :: (r :: l0)))))
  | Ohighlong ->
    let op0 = Ohighlong in
    (match args with
     | [] -> Coq_operation_other (op0, [])
     | arg :: l ->
       (match l with
        | [] -> Coq_operation_Ohighlong arg
        | r :: l0 -> Coq_operation_other (op0, (arg :: (r :: l0)))))
  | x -> Coq_operation_other (x, args)

(** val pair_instr_block : instruction -> bblock -> block_shape option **)

let pair_instr_block i b =
  match i with
  | Inop s ->
    let (mv, b1) = extract_moves [] b in
    if check_succ s b1 then Some (BSnop (mv, s)) else None
  | Iop (op, args, res0, s) ->
    (match classify_operation op args with
     | Coq_operation_Omove arg ->
       let (mv, b1) = extract_moves [] b in
       if check_succ s b1 then Some (BSmove (arg, res0, mv, s)) else None
     | Coq_operation_Omakelong (arg1, arg2) ->
       let (mv, b1) = extract_moves [] b in
       if check_succ s b1
       then Some (BSmakelong (arg1, arg2, res0, mv, s))
       else None
     | Coq_operation_Olowlong arg ->
       let (mv, b1) = extract_moves [] b in
       if check_succ s b1 then Some (BSlowlong (arg, res0, mv, s)) else None
     | Coq_operation_Ohighlong arg ->
       let (mv, b1) = extract_moves [] b in
       if check_succ s b1 then Some (BShighlong (arg, res0, mv, s)) else None
     | Coq_operation_other (op0, args0) ->
       let (mv1, b1) = extract_moves [] b in
       (match b1 with
        | [] ->
          if check_succ s b1
          then Some (BSopdead (op, args, res0, mv1, s))
          else None
        | i0 :: b2 ->
          (match i0 with
           | Lop (op', args', res') ->
             let (mv2, b3) = extract_moves [] b2 in
             if eq_operation op op'
             then if check_succ s b3
                  then Some (BSop (op, args, res0, mv1, args', res', mv2, s))
                  else None
             else None
           | _ ->
             if check_succ s b1
             then Some (BSopdead (op, args, res0, mv1, s))
             else None)))
  | Iload (chunk, addr, args, dst, s) ->
    let (mv1, b1) = extract_moves [] b in
    (match b1 with
     | [] ->
       if check_succ s b1
       then Some (BSloaddead (chunk, addr, args, dst, mv1, s))
       else None
     | i0 :: b2 ->
       (match i0 with
        | Lload (chunk', addr', args', dst') ->
          if chunk_eq chunk Mint64
          then if chunk_eq chunk' Mint32
               then let (mv2, b3) = extract_moves [] b2 in
                    (match b3 with
                     | [] ->
                       if check_succ s b3
                       then if eq_addressing addr addr'
                            then Some (BSload2_1 (addr, args, dst, mv1,
                                   args', dst', mv2, s))
                            else if option_eq eq_addressing
                                      (offset_addressing addr
                                        (Int.repr (Zpos (Coq_xO (Coq_xO
                                          Coq_xH))))) (Some addr')
                                 then Some (BSload2_2 (addr, addr', args,
                                        dst, mv1, args', dst', mv2, s))
                                 else None
                       else None
                     | i1 :: b4 ->
                       (match i1 with
                        | Lload (chunk'', addr'', args'', dst'') ->
                          let (mv3, b5) = extract_moves [] b4 in
                          if chunk_eq chunk'' Mint32
                          then if eq_addressing addr addr'
                               then if option_eq eq_addressing
                                         (offset_addressing addr
                                           (Int.repr (Zpos (Coq_xO (Coq_xO
                                             Coq_xH))))) (Some addr'')
                                    then if check_succ s b5
                                         then Some (BSload2 (addr, addr'',
                                                args, dst, mv1, args', dst',
                                                mv2, args'', dst'', mv3, s))
                                         else None
                                    else None
                               else None
                          else None
                        | _ ->
                          if check_succ s b3
                          then if eq_addressing addr addr'
                               then Some (BSload2_1 (addr, args, dst, mv1,
                                      args', dst', mv2, s))
                               else if option_eq eq_addressing
                                         (offset_addressing addr
                                           (Int.repr (Zpos (Coq_xO (Coq_xO
                                             Coq_xH))))) (Some addr')
                                    then Some (BSload2_2 (addr, addr', args,
                                           dst, mv1, args', dst', mv2, s))
                                    else None
                          else None))
               else None
          else let (mv2, b3) = extract_moves [] b2 in
               if chunk_eq chunk chunk'
               then if eq_addressing addr addr'
                    then if check_succ s b3
                         then Some (BSload (chunk, addr, args, dst, mv1,
                                args', dst', mv2, s))
                         else None
                    else None
               else None
        | _ ->
          if check_succ s b1
          then Some (BSloaddead (chunk, addr, args, dst, mv1, s))
          else None))
  | Istore (chunk, addr, args, src, s) ->
    let (mv1, b1) = extract_moves [] b in
    (match b1 with
     | [] -> None
     | i0 :: b2 ->
       (match i0 with
        | Lstore (chunk', addr', args', src') ->
          if chunk_eq chunk Mint64
          then let (mv2, b3) = extract_moves [] b2 in
               (match b3 with
                | [] -> None
                | i1 :: b4 ->
                  (match i1 with
                   | Lstore (chunk'', addr'', args'', src'') ->
                     if chunk_eq chunk' Mint32
                     then if chunk_eq chunk'' Mint32
                          then if eq_addressing addr addr'
                               then if option_eq eq_addressing
                                         (offset_addressing addr
                                           (Int.repr (Zpos (Coq_xO (Coq_xO
                                             Coq_xH))))) (Some addr'')
                                    then if check_succ s b4
                                         then Some (BSstore2 (addr, addr'',
                                                args, src, mv1, args', src',
                                                mv2, args'', src'', s))
                                         else None
                                    else None
                               else None
                          else None
                     else None
                   | _ -> None))
          else if chunk_eq chunk chunk'
               then if eq_addressing addr addr'
                    then if check_succ s b2
                         then Some (BSstore (chunk, addr, args, src, mv1,
                                args', src', s))
                         else None
                    else None
               else None
        | _ -> None))
  | Icall (sg, ros, args, res0, s) ->
    let (mv1, b1) = extract_moves [] b in
    (match b1 with
     | [] -> None
     | i0 :: b2 ->
       (match i0 with
        | Lcall (sg', ros') ->
          let (mv2, b3) = extract_moves [] b2 in
          if signature_eq sg sg'
          then if check_succ s b3
               then Some (BScall (sg, ros, args, res0, mv1, ros', mv2, s))
               else None
          else None
        | _ -> None))
  | Itailcall (sg, ros, args) ->
    let (mv1, b1) = extract_moves [] b in
    (match b1 with
     | [] -> None
     | i0 :: b2 ->
       (match i0 with
        | Ltailcall (sg', ros') ->
          if signature_eq sg sg'
          then Some (BStailcall (sg, ros, args, mv1, ros'))
          else None
        | _ -> None))
  | Ibuiltin (ef, args, res0, s) ->
    let (mv1, b1) = extract_moves [] b in
    (match b1 with
     | [] -> None
     | i0 :: b2 ->
       (match i0 with
        | Lbuiltin (ef', args', res') ->
          let (mv2, b3) = extract_moves [] b2 in
          if external_function_eq ef ef'
          then if check_succ s b3
               then Some (BSbuiltin (ef, args, res0, mv1, args', res', mv2,
                      s))
               else None
          else None
        | Lannot (ef', args') ->
          if external_function_eq ef ef'
          then if check_succ s b2
               then (match ef with
                     | EF_annot (txt, typ0) ->
                       Some (BSannot (txt, typ0, args, res0, mv1, args', s))
                     | _ -> None)
               else None
          else None
        | _ -> None))
  | Icond (cond, args, s1, s2) ->
    let (mv1, b1) = extract_moves [] b in
    (match b1 with
     | [] -> None
     | i0 :: b2 ->
       (match i0 with
        | Lcond (cond', args', s1', s2') ->
          if eq_condition cond cond'
          then if peq s1 s1'
               then if peq s2 s2'
                    then Some (BScond (cond, args, mv1, args', s1, s2))
                    else None
               else None
          else None
        | _ -> None))
  | Ijumptable (arg, tbl) ->
    let (mv1, b1) = extract_moves [] b in
    (match b1 with
     | [] -> None
     | i0 :: b2 ->
       (match i0 with
        | Ljumptable (arg', tbl') ->
          if list_eq_dec peq tbl tbl'
          then Some (BSjumptable (arg, mv1, arg', tbl))
          else None
        | _ -> None))
  | Ireturn arg ->
    let (mv1, b1) = extract_moves [] b in
    (match b1 with
     | [] -> None
     | i0 :: b2 ->
       (match i0 with
        | Lreturn -> Some (BSreturn (arg, mv1))
        | _ -> None))

(** val pair_codes :
    coq_function -> LTL.coq_function -> block_shape PTree.t **)

let pair_codes f1 f2 =
  PTree.combine (fun opti optb ->
    match opti with
    | Some i ->
      (match optb with
       | Some b -> pair_instr_block i b
       | None -> None)
    | None -> None) f1.fn_code f2.LTL.fn_code

(** val pair_entrypoints :
    coq_function -> LTL.coq_function -> moves option **)

let pair_entrypoints f1 f2 =
  match PTree.get f2.LTL.fn_entrypoint f2.LTL.fn_code with
  | Some b ->
    let (mv, b1) = extract_moves [] b in
    if check_succ f1.fn_entrypoint b1 then Some mv else None
  | None -> None

type equation_kind =
| Full
| Low
| High

type equation = { ekind : equation_kind; ereg : reg; eloc : loc }

(** val ekind : equation -> equation_kind **)

let ekind x = x.ekind

(** val ereg : equation -> reg **)

let ereg x = x.ereg

(** val eloc : equation -> loc **)

let eloc x = x.eloc

module IndexedEqKind = 
 struct 
  type t = equation_kind
  
  (** val index : t -> positive **)
  
  let index = function
  | Full -> Coq_xH
  | Low -> Coq_xO Coq_xH
  | High -> Coq_xI Coq_xH
  
  (** val eq : t -> t -> bool **)
  
  let eq x y =
    match x with
    | Full ->
      (match y with
       | Full -> true
       | _ -> false)
    | Low ->
      (match y with
       | Low -> true
       | _ -> false)
    | High ->
      (match y with
       | High -> true
       | _ -> false)
 end

module OrderedEqKind = OrderedIndexed(IndexedEqKind)

module OrderedEquation = 
 struct 
  type t = equation
  
  (** val compare : t -> t -> t OrderedType.coq_Compare **)
  
  let compare x y =
    let c = OrderedPositive.compare x.ereg y.ereg in
    (match c with
     | OrderedType.LT -> OrderedType.LT
     | OrderedType.EQ ->
       let c0 = OrderedLoc.compare x.eloc y.eloc in
       (match c0 with
        | OrderedType.LT -> OrderedType.LT
        | OrderedType.EQ ->
          let c1 = OrderedEqKind.compare x.ekind y.ekind in
          (match c1 with
           | OrderedType.LT -> OrderedType.LT
           | OrderedType.EQ -> OrderedType.EQ
           | OrderedType.GT -> OrderedType.GT)
        | OrderedType.GT -> OrderedType.GT)
     | OrderedType.GT -> OrderedType.GT)
  
  (** val eq_dec : t -> t -> bool **)
  
  let eq_dec x y =
    let { ekind = x0; ereg = x1; eloc = x2 } = x in
    let { ekind = ekind1; ereg = ereg1; eloc = eloc1 } = y in
    if IndexedEqKind.eq x0 ekind1
    then if peq x1 ereg1 then Loc.eq x2 eloc1 else false
    else false
 end

module OrderedEquation' = 
 struct 
  type t = equation
  
  (** val compare : t -> t -> t OrderedType.coq_Compare **)
  
  let compare x y =
    let c = OrderedLoc.compare x.eloc y.eloc in
    (match c with
     | OrderedType.LT -> OrderedType.LT
     | OrderedType.EQ ->
       let c0 = OrderedPositive.compare x.ereg y.ereg in
       (match c0 with
        | OrderedType.LT -> OrderedType.LT
        | OrderedType.EQ ->
          let c1 = OrderedEqKind.compare x.ekind y.ekind in
          (match c1 with
           | OrderedType.LT -> OrderedType.LT
           | OrderedType.EQ -> OrderedType.EQ
           | OrderedType.GT -> OrderedType.GT)
        | OrderedType.GT -> OrderedType.GT)
     | OrderedType.GT -> OrderedType.GT)
  
  (** val eq_dec : t -> t -> bool **)
  
  let eq_dec =
    OrderedEquation.eq_dec
 end

module EqSet = FSetAVLplus.Make(OrderedEquation)

module EqSet2 = FSetAVLplus.Make(OrderedEquation')

type eqs = { eqs1 : EqSet.t; eqs2 : EqSet2.t }

(** val eqs1 : eqs -> EqSet.t **)

let eqs1 x = x.eqs1

(** val eqs2 : eqs -> EqSet2.t **)

let eqs2 x = x.eqs2

(** val empty_eqs : eqs **)

let empty_eqs =
  { eqs1 = EqSet.empty; eqs2 = EqSet2.empty }

(** val add_equation : equation -> eqs -> eqs **)

let add_equation q e =
  { eqs1 = (EqSet.add q e.eqs1); eqs2 = (EqSet2.add q e.eqs2) }

(** val remove_equation : equation -> eqs -> eqs **)

let remove_equation q e =
  { eqs1 = (EqSet.remove q e.eqs1); eqs2 = (EqSet2.remove q e.eqs2) }

(** val select_reg_l : reg -> equation -> bool **)

let select_reg_l r q =
  Pos.leb r q.ereg

(** val select_reg_h : reg -> equation -> bool **)

let select_reg_h r q =
  Pos.leb q.ereg r

(** val reg_unconstrained : reg -> eqs -> bool **)

let reg_unconstrained r e =
  negb (EqSet.mem_between (select_reg_l r) (select_reg_h r) e.eqs1)

(** val select_loc_l : loc -> equation -> bool **)

let select_loc_l l =
  let lb = OrderedLoc.diff_low_bound l in
  (fun q ->
  match OrderedLoc.compare q.eloc lb with
  | OrderedType.LT -> false
  | _ -> true)

(** val select_loc_h : loc -> equation -> bool **)

let select_loc_h l =
  let lh = OrderedLoc.diff_high_bound l in
  (fun q ->
  match OrderedLoc.compare q.eloc lh with
  | OrderedType.GT -> false
  | _ -> true)

(** val loc_unconstrained : loc -> eqs -> bool **)

let loc_unconstrained l e =
  negb (EqSet2.mem_between (select_loc_l l) (select_loc_h l) e.eqs2)

(** val reg_loc_unconstrained : reg -> loc -> eqs -> bool **)

let reg_loc_unconstrained r l e =
  (&&) (reg_unconstrained r e) (loc_unconstrained l e)

(** val subst_reg : reg -> reg -> eqs -> eqs **)

let subst_reg r1 r2 e =
  EqSet.fold (fun q e0 ->
    add_equation { ekind = q.ekind; ereg = r2; eloc = q.eloc }
      (remove_equation q e0))
    (EqSet.elements_between (select_reg_l r1) (select_reg_h r1) e.eqs1) e

(** val subst_reg_kind :
    reg -> equation_kind -> reg -> equation_kind -> eqs -> eqs **)

let subst_reg_kind r1 k1 r2 k2 e =
  EqSet.fold (fun q e0 ->
    if IndexedEqKind.eq q.ekind k1
    then add_equation { ekind = k2; ereg = r2; eloc = q.eloc }
           (remove_equation q e0)
    else e0)
    (EqSet.elements_between (select_reg_l r1) (select_reg_h r1) e.eqs1) e

(** val subst_loc : loc -> loc -> eqs -> eqs option **)

let subst_loc l1 l2 e =
  EqSet2.fold (fun q opte ->
    match opte with
    | Some e0 ->
      if Loc.eq l1 q.eloc
      then Some
             (add_equation { ekind = q.ekind; ereg = q.ereg; eloc = l2 }
               (remove_equation q e0))
      else None
    | None -> None)
    (EqSet2.elements_between (select_loc_l l1) (select_loc_h l1) e.eqs2)
    (Some e)

(** val sel_type : equation_kind -> typ -> typ **)

let sel_type k ty =
  match k with
  | Full -> ty
  | _ -> Tint

(** val loc_type_compat : regenv -> loc -> eqs -> bool **)

let loc_type_compat env l e =
  EqSet2.for_all_between (fun q ->
    subtype (sel_type q.ekind (env q.ereg)) (Loc.coq_type l))
    (select_loc_l l) (select_loc_h l) e.eqs2

(** val add_equations : reg list -> mreg list -> eqs -> eqs option **)

let rec add_equations rl ml e =
  match rl with
  | [] ->
    (match ml with
     | [] -> Some e
     | m :: l -> None)
  | r1 :: rl0 ->
    (match ml with
     | [] -> None
     | m1 :: ml0 ->
       add_equations rl0 ml0
         (add_equation { ekind = Full; ereg = r1; eloc = (R m1) } e))

(** val add_equations_args :
    reg list -> typ list -> loc list -> eqs -> eqs option **)

let rec add_equations_args rl tyl ll e =
  match rl with
  | [] ->
    (match tyl with
     | [] ->
       (match ll with
        | [] -> Some e
        | l :: l0 -> None)
     | t0 :: l -> None)
  | r1 :: rl0 ->
    (match tyl with
     | [] -> None
     | t0 :: tyl0 ->
       (match t0 with
        | Tlong ->
          (match ll with
           | [] -> None
           | l1 :: l ->
             (match l with
              | [] -> None
              | l2 :: ll0 ->
                add_equations_args rl0 tyl0 ll0
                  (add_equation { ekind = Low; ereg = r1; eloc = l2 }
                    (add_equation { ekind = High; ereg = r1; eloc = l1 } e))))
        | Tany32 -> None
        | Tany64 -> None
        | _ ->
          (match ll with
           | [] -> None
           | l1 :: ll0 ->
             add_equations_args rl0 tyl0 ll0
               (add_equation { ekind = Full; ereg = r1; eloc = l1 } e))))

(** val add_equations_res :
    reg -> typ option -> loc list -> eqs -> eqs option **)

let add_equations_res r oty ll e =
  match oty with
  | Some t0 ->
    (match t0 with
     | Tlong ->
       (match ll with
        | [] -> None
        | l1 :: l ->
          (match l with
           | [] -> None
           | l2 :: l0 ->
             (match l0 with
              | [] ->
                Some
                  (add_equation { ekind = Low; ereg = r; eloc = l2 }
                    (add_equation { ekind = High; ereg = r; eloc = l1 } e))
              | l3 :: l4 -> None)))
     | _ ->
       (match ll with
        | [] -> None
        | l1 :: l ->
          (match l with
           | [] ->
             Some (add_equation { ekind = Full; ereg = r; eloc = l1 } e)
           | l0 :: l2 -> None)))
  | None ->
    (match ll with
     | [] -> None
     | l1 :: l ->
       (match l with
        | [] -> Some (add_equation { ekind = Full; ereg = r; eloc = l1 } e)
        | l0 :: l2 -> None))

(** val remove_equations_res :
    reg -> typ option -> loc list -> eqs -> eqs option **)

let remove_equations_res r oty ll e =
  match oty with
  | Some t0 ->
    (match t0 with
     | Tlong ->
       (match ll with
        | [] -> None
        | l1 :: l ->
          (match l with
           | [] -> None
           | l2 :: l0 ->
             (match l0 with
              | [] ->
                if Loc.diff_dec l2 l1
                then Some
                       (remove_equation { ekind = Low; ereg = r; eloc = l2 }
                         (remove_equation { ekind = High; ereg = r; eloc =
                           l1 } e))
                else None
              | l3 :: l4 -> None)))
     | _ ->
       (match ll with
        | [] -> None
        | l1 :: l ->
          (match l with
           | [] ->
             Some (remove_equation { ekind = Full; ereg = r; eloc = l1 } e)
           | l0 :: l2 -> None)))
  | None ->
    (match ll with
     | [] -> None
     | l1 :: l ->
       (match l with
        | [] ->
          Some (remove_equation { ekind = Full; ereg = r; eloc = l1 } e)
        | l0 :: l2 -> None))

(** val add_equation_ros :
    (reg, ident) sum -> (mreg, ident) sum -> eqs -> eqs option **)

let add_equation_ros ros ros' e =
  match ros with
  | Coq_inl r ->
    (match ros' with
     | Coq_inl mr ->
       Some (add_equation { ekind = Full; ereg = r; eloc = (R mr) } e)
     | Coq_inr i -> None)
  | Coq_inr id ->
    (match ros' with
     | Coq_inl m -> None
     | Coq_inr id' -> if ident_eq id id' then Some e else None)

(** val can_undef : mreg list -> eqs -> bool **)

let rec can_undef ml e =
  match ml with
  | [] -> true
  | m1 :: ml0 -> (&&) (loc_unconstrained (R m1) e) (can_undef ml0 e)

(** val can_undef_except : loc -> mreg list -> eqs -> bool **)

let rec can_undef_except l ml e =
  match ml with
  | [] -> true
  | m1 :: ml0 ->
    (&&) ((||) ((fun x -> x) (Loc.eq l (R m1))) (loc_unconstrained (R m1) e))
      (can_undef_except l ml0 e)

(** val no_caller_saves : eqs -> bool **)

let no_caller_saves e =
  EqSet.for_all (fun eq0 ->
    match eq0.eloc with
    | R r ->
      (||) ((fun x -> x) (zle Z0 (index_int_callee_save r)))
        ((fun x -> x) (zle Z0 (index_float_callee_save r)))
    | S (sl, pos, ty) ->
      (match sl with
       | Outgoing -> false
       | _ -> true)) e.eqs1

(** val compat_left : reg -> loc -> eqs -> bool **)

let compat_left r l e =
  EqSet.for_all_between (fun q ->
    match q.ekind with
    | Full -> (fun x -> x) (Loc.eq l q.eloc)
    | _ -> false) (select_reg_l r) (select_reg_h r) e.eqs1

(** val compat_left2 : reg -> loc -> loc -> eqs -> bool **)

let compat_left2 r l1 l2 e =
  EqSet.for_all_between (fun q ->
    match q.ekind with
    | Full -> false
    | Low -> (fun x -> x) (Loc.eq l2 q.eloc)
    | High -> (fun x -> x) (Loc.eq l1 q.eloc)) (select_reg_l r)
    (select_reg_h r) e.eqs1

(** val ros_compatible_tailcall : (mreg, ident) sum -> bool **)

let ros_compatible_tailcall = function
| Coq_inl r -> (fun x -> x) (in_dec mreg_eq r destroyed_at_call)
| Coq_inr id -> true

(** val destroyed_by_move : loc -> loc -> mreg list **)

let destroyed_by_move src dst =
  match src with
  | R r ->
    (match dst with
     | R r0 -> destroyed_by_op Omove
     | S (sl, ofs, ty) -> destroyed_by_setstack ty)
  | S (sl, ofs, ty) -> destroyed_by_getstack sl

(** val well_typed_move : regenv -> loc -> eqs -> bool **)

let well_typed_move env dst e =
  match dst with
  | R r -> true
  | S (sl, ofs, ty) -> loc_type_compat env dst e

(** val track_moves : regenv -> moves -> eqs -> eqs option **)

let rec track_moves env mv e =
  match mv with
  | [] -> Some e
  | p :: mv0 ->
    let (src, dst) = p in
    (match track_moves env mv0 e with
     | Some e1 ->
       if can_undef_except dst (destroyed_by_move src dst) e1
       then if well_typed_move env dst e1 then subst_loc dst src e1 else None
       else None
     | None -> None)

(** val transfer_use_def :
    reg list -> reg -> mreg list -> mreg -> mreg list -> eqs -> eqs option **)

let transfer_use_def args res0 args' res' undefs e =
  let e1 = remove_equation { ekind = Full; ereg = res0; eloc = (R res') } e
  in
  if reg_loc_unconstrained res0 (R res') e1
  then if can_undef undefs e1 then add_equations args args' e1 else None
  else None

(** val kind_first_word : equation_kind **)

let kind_first_word =
  if big_endian then High else Low

(** val kind_second_word : equation_kind **)

let kind_second_word =
  if big_endian then Low else High

(** val transfer_aux :
    coq_function -> regenv -> block_shape -> eqs -> eqs option **)

let transfer_aux f env shape e =
  match shape with
  | BSnop (mv, s) -> track_moves env mv e
  | BSmove (src, dst, mv, s) -> track_moves env mv (subst_reg dst src e)
  | BSmakelong (src1, src2, dst, mv, s) ->
    let e1 = subst_reg_kind dst High src1 Full e in
    let e2 = subst_reg_kind dst Low src2 Full e1 in
    if reg_unconstrained dst e2 then track_moves env mv e2 else None
  | BSlowlong (src, dst, mv, s) ->
    let e1 = subst_reg_kind dst Full src Low e in
    if reg_unconstrained dst e1 then track_moves env mv e1 else None
  | BShighlong (src, dst, mv, s) ->
    let e1 = subst_reg_kind dst Full src High e in
    if reg_unconstrained dst e1 then track_moves env mv e1 else None
  | BSop (op, args, res0, mv1, args', res', mv2, s) ->
    (match track_moves env mv2 e with
     | Some e1 ->
       (match transfer_use_def args res0 args' res' (destroyed_by_op op) e1 with
        | Some e2 -> track_moves env mv1 e2
        | None -> None)
     | None -> None)
  | BSopdead (op, args, res0, mv, s) ->
    if reg_unconstrained res0 e then track_moves env mv e else None
  | BSload (chunk, addr, args, dst, mv1, args', dst', mv2, s) ->
    (match track_moves env mv2 e with
     | Some e1 ->
       (match transfer_use_def args dst args' dst'
                (destroyed_by_load chunk addr) e1 with
        | Some e2 -> track_moves env mv1 e2
        | None -> None)
     | None -> None)
  | BSloaddead (chunk, addr, args, dst, mv, s) ->
    if reg_unconstrained dst e then track_moves env mv e else None
  | BSload2 (addr, addr', args, dst, mv1, args1', dst1', mv2, args2', dst2',
             mv3, s) ->
    (match track_moves env mv3 e with
     | Some e1 ->
       let e2 =
         remove_equation { ekind = kind_second_word; ereg = dst; eloc = (R
           dst2') } e1
       in
       if loc_unconstrained (R dst2') e2
       then if can_undef (destroyed_by_load Mint32 addr') e2
            then (match add_equations args args2' e2 with
                  | Some e3 ->
                    (match track_moves env mv2 e3 with
                     | Some e4 ->
                       let e5 =
                         remove_equation { ekind = kind_first_word; ereg =
                           dst; eloc = (R dst1') } e4
                       in
                       if loc_unconstrained (R dst1') e5
                       then if can_undef (destroyed_by_load Mint32 addr) e5
                            then if reg_unconstrained dst e5
                                 then (match add_equations args args1' e5 with
                                       | Some e6 -> track_moves env mv1 e6
                                       | None -> None)
                                 else None
                            else None
                       else None
                     | None -> None)
                  | None -> None)
            else None
       else None
     | None -> None)
  | BSload2_1 (addr, args, dst, mv1, args', dst', mv2, s) ->
    (match track_moves env mv2 e with
     | Some e1 ->
       let e2 =
         remove_equation { ekind = kind_first_word; ereg = dst; eloc = (R
           dst') } e1
       in
       if reg_loc_unconstrained dst (R dst') e2
       then if can_undef (destroyed_by_load Mint32 addr) e2
            then (match add_equations args args' e2 with
                  | Some e3 -> track_moves env mv1 e3
                  | None -> None)
            else None
       else None
     | None -> None)
  | BSload2_2 (addr, addr', args, dst, mv1, args', dst', mv2, s) ->
    (match track_moves env mv2 e with
     | Some e1 ->
       let e2 =
         remove_equation { ekind = kind_second_word; ereg = dst; eloc = (R
           dst') } e1
       in
       if reg_loc_unconstrained dst (R dst') e2
       then if can_undef (destroyed_by_load Mint32 addr') e2
            then (match add_equations args args' e2 with
                  | Some e3 -> track_moves env mv1 e3
                  | None -> None)
            else None
       else None
     | None -> None)
  | BSstore (chunk, addr, args, src, mv, args', src', s) ->
    if can_undef (destroyed_by_store chunk addr) e
    then (match add_equations (src :: args) (src' :: args') e with
          | Some e1 -> track_moves env mv e1
          | None -> None)
    else None
  | BSstore2 (addr, addr', args, src, mv1, args1', src1', mv2, args2', src2',
              s) ->
    if can_undef (destroyed_by_store Mint32 addr') e
    then (match add_equations args args2'
                  (add_equation { ekind = kind_second_word; ereg = src;
                    eloc = (R src2') } e) with
          | Some e1 ->
            (match track_moves env mv2 e1 with
             | Some e2 ->
               if can_undef (destroyed_by_store Mint32 addr) e2
               then (match add_equations args args1'
                             (add_equation { ekind = kind_first_word; ereg =
                               src; eloc = (R src1') } e2) with
                     | Some e3 -> track_moves env mv1 e3
                     | None -> None)
               else None
             | None -> None)
          | None -> None)
    else None
  | BScall (sg, ros, args, res0, mv1, ros', mv2, s) ->
    let args' = loc_arguments sg in
    let res' = map (fun x -> R x) (loc_result sg) in
    (match track_moves env mv2 e with
     | Some e1 ->
       (match remove_equations_res res0 sg.sig_res res' e1 with
        | Some e2 ->
          if forallb (fun l -> reg_loc_unconstrained res0 l e2) res'
          then if no_caller_saves e2
               then (match add_equation_ros ros ros' e2 with
                     | Some e3 ->
                       (match add_equations_args args sg.sig_args args' e3 with
                        | Some e4 -> track_moves env mv1 e4
                        | None -> None)
                     | None -> None)
               else None
          else None
        | None -> None)
     | None -> None)
  | BStailcall (sg, ros, args, mv1, ros') ->
    let args' = loc_arguments sg in
    if tailcall_is_possible sg
    then if opt_typ_eq sg.sig_res f.fn_sig.sig_res
         then if ros_compatible_tailcall ros'
              then (match add_equation_ros ros ros' empty_eqs with
                    | Some e1 ->
                      (match add_equations_args args sg.sig_args args' e1 with
                       | Some e2 -> track_moves env mv1 e2
                       | None -> None)
                    | None -> None)
              else None
         else None
    else None
  | BSbuiltin (ef, args, res0, mv1, args', res', mv2, s) ->
    (match track_moves env mv2 e with
     | Some e1 ->
       let args'0 = map (fun x -> R x) args' in
       let res'0 = map (fun x -> R x) res' in
       (match remove_equations_res res0 (ef_sig ef).sig_res res'0 e1 with
        | Some e2 ->
          if reg_unconstrained res0 e2
          then if forallb (fun l -> loc_unconstrained l e2) res'0
               then if can_undef (destroyed_by_builtin ef) e2
                    then (match add_equations_args args (ef_sig ef).sig_args
                                  args'0 e2 with
                          | Some e3 -> track_moves env mv1 e3
                          | None -> None)
                    else None
               else None
          else None
        | None -> None)
     | None -> None)
  | BSannot (txt, typ0, args, res0, mv1, args', s) ->
    (match add_equations_args args (annot_args_typ typ0) args' e with
     | Some e1 -> track_moves env mv1 e1
     | None -> None)
  | BScond (cond, args, mv, args', s1, s2) ->
    if can_undef (destroyed_by_cond cond) e
    then (match add_equations args args' e with
          | Some e1 -> track_moves env mv e1
          | None -> None)
    else None
  | BSjumptable (arg, mv, arg', tbl) ->
    if can_undef destroyed_by_jumptable e
    then track_moves env mv
           (add_equation { ekind = Full; ereg = arg; eloc = (R arg') } e)
    else None
  | BSreturn (arg0, mv) ->
    (match arg0 with
     | Some arg ->
       let arg' = map (fun x -> R x) (loc_result f.fn_sig) in
       (match add_equations_res arg f.fn_sig.sig_res arg' empty_eqs with
        | Some e1 -> track_moves env mv e1
        | None -> None)
     | None -> track_moves env mv empty_eqs)

(** val transfer :
    coq_function -> regenv -> block_shape PTree.t -> LTL.node -> eqs res ->
    eqs res **)

let transfer f env shapes pc after = match after with
| OK e ->
  (match PTree.get pc shapes with
   | Some shape ->
     (match transfer_aux f env shape e with
      | Some e' -> OK e'
      | None ->
        Error ((MSG ('A'::('t'::(' '::('P'::('C'::(' '::[]))))))) :: ((POS
          pc) :: ((MSG
          (':'::(' '::('i'::('n'::('v'::('a'::('l'::('i'::('d'::(' '::('r'::('e'::('g'::('i'::('s'::('t'::('e'::('r'::(' '::('a'::('l'::('l'::('o'::('c'::('a'::('t'::('i'::('o'::('n'::[])))))))))))))))))))))))))))))) :: []))))
   | None ->
     Error ((MSG ('A'::('t'::(' '::('P'::('C'::(' '::[]))))))) :: ((POS
       pc) :: ((MSG
       (':'::(' '::('u'::('n'::('m'::('a'::('t'::('c'::('h'::('e'::('d'::(' '::('b'::('l'::('o'::('c'::('k'::[])))))))))))))))))) :: []))))
| Error e -> after

module LEq = 
 struct 
  type t = eqs res
  
  (** val beq : t -> t -> bool **)
  
  let beq x y =
    match x with
    | OK a ->
      (match y with
       | OK b -> EqSet.equal a.eqs1 b.eqs1
       | Error e -> false)
    | Error e ->
      (match y with
       | OK e0 -> false
       | Error e0 -> true)
  
  (** val bot : t **)
  
  let bot =
    OK empty_eqs
  
  (** val lub : t -> t -> t **)
  
  let lub x y =
    match x with
    | OK a ->
      (match y with
       | OK b ->
         OK { eqs1 = (EqSet.union a.eqs1 b.eqs1); eqs2 =
           (EqSet2.union a.eqs2 b.eqs2) }
       | Error e -> y)
    | Error e -> x
 end

module DS = Backward_Dataflow_Solver(LEq)(NodeSetBackward)

(** val successors_block_shape : block_shape -> LTL.node list **)

let successors_block_shape = function
| BSnop (mv, s) -> s :: []
| BSmove (src, dst, mv, s) -> s :: []
| BSmakelong (src1, src2, dst, mv, s) -> s :: []
| BSlowlong (src, dst, mv, s) -> s :: []
| BShighlong (src, dst, mv, s) -> s :: []
| BSop (op, args, res0, mv1, args', res', mv2, s) -> s :: []
| BSopdead (op, args, res0, mv, s) -> s :: []
| BSload (chunk, addr, args, dst, mv1, args', dst', mv2, s) -> s :: []
| BSloaddead (chunk, addr, args, dst, mv, s) -> s :: []
| BSload2 (addr, addr', args, dst, mv1, args1', dst1', mv2, args2', dst2',
           mv3, s) ->
  s :: []
| BSload2_1 (addr, args, dst, mv1, args', dst', mv2, s) -> s :: []
| BSload2_2 (addr, addr', args, dst, mv1, args', dst', mv2, s) -> s :: []
| BSstore (chunk, addr, args, src, mv1, args', src', s) -> s :: []
| BSstore2 (addr, addr', args, src, mv1, args1', src1', mv2, args2', src2', s) ->
  s :: []
| BScall (sg, ros, args, res0, mv1, ros', mv2, s) -> s :: []
| BSbuiltin (ef, args, res0, mv1, args', res', mv2, s) -> s :: []
| BSannot (txt, typ0, args, res0, mv1, args', s) -> s :: []
| BScond (cond, args, mv, args', s1, s2) -> s1 :: (s2 :: [])
| BSjumptable (arg, mv, arg', tbl) -> tbl
| _ -> []

(** val analyze :
    coq_function -> regenv -> block_shape PTree.t -> DS.L.t PMap.t option **)

let analyze f env bsh =
  DS.fixpoint_allnodes bsh successors_block_shape (transfer f env bsh)

(** val compat_entry : reg list -> typ list -> loc list -> eqs -> bool **)

let rec compat_entry rparams tys lparams e =
  match rparams with
  | [] ->
    (match tys with
     | [] ->
       (match lparams with
        | [] -> true
        | l :: l0 -> false)
     | t0 :: l -> false)
  | r1 :: rl ->
    (match tys with
     | [] -> false
     | t0 :: tyl ->
       (match t0 with
        | Tlong ->
          (match lparams with
           | [] -> false
           | l1 :: l ->
             (match l with
              | [] -> false
              | l2 :: ll ->
                (&&) (compat_left2 r1 l1 l2 e) (compat_entry rl tyl ll e)))
        | Tany32 -> false
        | Tany64 -> false
        | _ ->
          (match lparams with
           | [] -> false
           | l1 :: ll ->
             (&&) (compat_left r1 l1 e) (compat_entry rl tyl ll e))))

(** val check_entrypoints_aux :
    coq_function -> LTL.coq_function -> regenv -> eqs -> unit option **)

let check_entrypoints_aux rtl ltl env e1 =
  match pair_entrypoints rtl ltl with
  | Some mv ->
    (match track_moves env mv e1 with
     | Some e2 ->
       if compat_entry rtl.fn_params rtl.fn_sig.sig_args
            (loc_parameters rtl.fn_sig) e2
       then if can_undef destroyed_at_function_entry e2
            then if zeq rtl.fn_stacksize ltl.LTL.fn_stacksize
                 then if signature_eq rtl.fn_sig ltl.LTL.fn_sig
                      then Some ()
                      else None
                 else None
            else None
       else None
     | None -> None)
  | None -> None

(** val check_entrypoints :
    coq_function -> LTL.coq_function -> regenv -> block_shape PTree.t ->
    LEq.t PMap.t -> unit res **)

let check_entrypoints rtl ltl env bsh a =
  match transfer rtl env bsh rtl.fn_entrypoint (PMap.get rtl.fn_entrypoint a) with
  | OK x ->
    (match check_entrypoints_aux rtl ltl env x with
     | Some u -> OK ()
     | None ->
       Error
         (msg
           ('i'::('n'::('v'::('a'::('l'::('i'::('d'::(' '::('r'::('e'::('g'::('i'::('s'::('t'::('e'::('r'::(' '::('a'::('l'::('l'::('o'::('c'::('a'::('t'::('i'::('o'::('n'::(' '::('a'::('t'::(' '::('e'::('n'::('t'::('r'::('y'::(' '::('p'::('o'::('i'::('n'::('t'::[]))))))))))))))))))))))))))))))))))))))))))))
  | Error msg0 -> Error msg0

(** val check_function :
    coq_function -> LTL.coq_function -> regenv -> unit res **)

let check_function rtl ltl env =
  let bsh = pair_codes rtl ltl in
  (match analyze rtl env bsh with
   | Some a -> check_entrypoints rtl ltl env bsh a
   | None ->
     Error
       (msg
         ('a'::('l'::('l'::('o'::('c'::('a'::('t'::('i'::('o'::('n'::(' '::('a'::('n'::('a'::('l'::('y'::('s'::('i'::('s'::(' '::('d'::('i'::('v'::('e'::('r'::('g'::('e'::('s'::[]))))))))))))))))))))))))))))))

(** val regalloc : coq_function -> LTL.coq_function res **)

let regalloc = Regalloc.regalloc

(** val transf_function : coq_function -> LTL.coq_function res **)

let transf_function f =
  match type_function f with
  | OK env ->
    (match regalloc f with
     | OK tf ->
       (match check_function f tf env with
        | OK x -> OK tf
        | Error msg0 -> Error msg0)
     | Error m -> Error m)
  | Error m -> Error m

(** val transf_fundef : fundef -> LTL.fundef res **)

let transf_fundef fd =
  transf_partial_fundef transf_function fd

(** val transf_program : program -> LTL.program res **)

let transf_program p =
  transform_partial_program transf_fundef p

