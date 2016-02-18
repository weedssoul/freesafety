open AST
open BinInt
open BinNums
open Coqlib
open Datatypes
open Floats
open Integers

type block = positive

(** val eq_block : positive -> positive -> bool **)

let eq_block =
  peq

type coq_val =
| Vundef
| Vint of Int.int
| Vlong of Int64.int
| Vfloat of float
| Vsingle of float32
| Vptr of block * Int.int

(** val coq_Vzero : coq_val **)

let coq_Vzero =
  Vint Int.zero

(** val coq_Vtrue : coq_val **)

let coq_Vtrue =
  Vint Int.one

(** val coq_Vfalse : coq_val **)

let coq_Vfalse =
  Vint Int.zero

module Val = 
 struct 
  (** val eq : coq_val -> coq_val -> bool **)
  
  let eq x y =
    match x with
    | Vundef ->
      (match y with
       | Vundef -> true
       | _ -> false)
    | Vint x0 ->
      (match y with
       | Vint i0 -> Int.eq_dec x0 i0
       | _ -> false)
    | Vlong x0 ->
      (match y with
       | Vlong i0 -> Int64.eq_dec x0 i0
       | _ -> false)
    | Vfloat x0 ->
      (match y with
       | Vfloat f0 -> Float.eq_dec x0 f0
       | _ -> false)
    | Vsingle x0 ->
      (match y with
       | Vsingle f0 -> Float32.eq_dec x0 f0
       | _ -> false)
    | Vptr (x0, x1) ->
      (match y with
       | Vptr (b0, i0) -> if eq_block x0 b0 then Int.eq_dec x1 i0 else false
       | _ -> false)
  
  (** val neg : coq_val -> coq_val **)
  
  let neg = function
  | Vint n -> Vint (Int.neg n)
  | _ -> Vundef
  
  (** val negf : coq_val -> coq_val **)
  
  let negf = function
  | Vfloat f -> Vfloat (Float.neg f)
  | _ -> Vundef
  
  (** val absf : coq_val -> coq_val **)
  
  let absf = function
  | Vfloat f -> Vfloat (Float.abs f)
  | _ -> Vundef
  
  (** val negfs : coq_val -> coq_val **)
  
  let negfs = function
  | Vsingle f -> Vsingle (Float32.neg f)
  | _ -> Vundef
  
  (** val absfs : coq_val -> coq_val **)
  
  let absfs = function
  | Vsingle f -> Vsingle (Float32.abs f)
  | _ -> Vundef
  
  (** val maketotal : coq_val option -> coq_val **)
  
  let maketotal = function
  | Some v -> v
  | None -> Vundef
  
  (** val intoffloat : coq_val -> coq_val option **)
  
  let intoffloat = function
  | Vfloat f -> option_map (fun x -> Vint x) (Float.to_int f)
  | _ -> None
  
  (** val intuoffloat : coq_val -> coq_val option **)
  
  let intuoffloat = function
  | Vfloat f -> option_map (fun x -> Vint x) (Float.to_intu f)
  | _ -> None
  
  (** val floatofint : coq_val -> coq_val option **)
  
  let floatofint = function
  | Vint n -> Some (Vfloat (Float.of_int n))
  | _ -> None
  
  (** val floatofintu : coq_val -> coq_val option **)
  
  let floatofintu = function
  | Vint n -> Some (Vfloat (Float.of_intu n))
  | _ -> None
  
  (** val intofsingle : coq_val -> coq_val option **)
  
  let intofsingle = function
  | Vsingle f -> option_map (fun x -> Vint x) (Float32.to_int f)
  | _ -> None
  
  (** val intuofsingle : coq_val -> coq_val option **)
  
  let intuofsingle = function
  | Vsingle f -> option_map (fun x -> Vint x) (Float32.to_intu f)
  | _ -> None
  
  (** val singleofint : coq_val -> coq_val option **)
  
  let singleofint = function
  | Vint n -> Some (Vsingle (Float32.of_int n))
  | _ -> None
  
  (** val singleofintu : coq_val -> coq_val option **)
  
  let singleofintu = function
  | Vint n -> Some (Vsingle (Float32.of_intu n))
  | _ -> None
  
  (** val negint : coq_val -> coq_val **)
  
  let negint = function
  | Vint n -> Vint (Int.neg n)
  | _ -> Vundef
  
  (** val notint : coq_val -> coq_val **)
  
  let notint = function
  | Vint n -> Vint (Int.not n)
  | _ -> Vundef
  
  (** val of_bool : bool -> coq_val **)
  
  let of_bool = function
  | true -> coq_Vtrue
  | false -> coq_Vfalse
  
  (** val boolval : coq_val -> coq_val **)
  
  let boolval = function
  | Vint n -> of_bool (negb (Int.eq n Int.zero))
  | Vptr (b, ofs) -> coq_Vtrue
  | _ -> Vundef
  
  (** val notbool : coq_val -> coq_val **)
  
  let notbool = function
  | Vint n -> of_bool (Int.eq n Int.zero)
  | Vptr (b, ofs) -> coq_Vfalse
  | _ -> Vundef
  
  (** val zero_ext : coq_Z -> coq_val -> coq_val **)
  
  let zero_ext nbits = function
  | Vint n -> Vint (Int.zero_ext nbits n)
  | _ -> Vundef
  
  (** val sign_ext : coq_Z -> coq_val -> coq_val **)
  
  let sign_ext nbits = function
  | Vint n -> Vint (Int.sign_ext nbits n)
  | _ -> Vundef
  
  (** val singleoffloat : coq_val -> coq_val **)
  
  let singleoffloat = function
  | Vfloat f -> Vsingle (Float.to_single f)
  | _ -> Vundef
  
  (** val floatofsingle : coq_val -> coq_val **)
  
  let floatofsingle = function
  | Vsingle f -> Vfloat (Float.of_single f)
  | _ -> Vundef
  
  (** val add : coq_val -> coq_val -> coq_val **)
  
  let add v1 v2 =
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 -> Vint (Int.add n1 n2)
       | Vptr (b2, ofs2) -> Vptr (b2, (Int.add ofs2 n1))
       | _ -> Vundef)
    | Vptr (b1, ofs1) ->
      (match v2 with
       | Vint n2 -> Vptr (b1, (Int.add ofs1 n2))
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val sub : coq_val -> coq_val -> coq_val **)
  
  let sub v1 v2 =
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 -> Vint (Int.sub n1 n2)
       | _ -> Vundef)
    | Vptr (b1, ofs1) ->
      (match v2 with
       | Vint n2 -> Vptr (b1, (Int.sub ofs1 n2))
       | Vptr (b2, ofs2) ->
         if eq_block b1 b2 then Vint (Int.sub ofs1 ofs2) else Vundef
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val mul : coq_val -> coq_val -> coq_val **)
  
  let mul v1 v2 =
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 -> Vint (Int.mul n1 n2)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val mulhs : coq_val -> coq_val -> coq_val **)
  
  let mulhs v1 v2 =
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 -> Vint (Int.mulhs n1 n2)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val mulhu : coq_val -> coq_val -> coq_val **)
  
  let mulhu v1 v2 =
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 -> Vint (Int.mulhu n1 n2)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val divs : coq_val -> coq_val -> coq_val option **)
  
  let divs v1 v2 =
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 ->
         if (||) (Int.eq n2 Int.zero)
              ((&&) (Int.eq n1 (Int.repr Int.min_signed))
                (Int.eq n2 Int.mone))
         then None
         else Some (Vint (Int.divs n1 n2))
       | _ -> None)
    | _ -> None
  
  (** val mods : coq_val -> coq_val -> coq_val option **)
  
  let mods v1 v2 =
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 ->
         if (||) (Int.eq n2 Int.zero)
              ((&&) (Int.eq n1 (Int.repr Int.min_signed))
                (Int.eq n2 Int.mone))
         then None
         else Some (Vint (Int.mods n1 n2))
       | _ -> None)
    | _ -> None
  
  (** val divu : coq_val -> coq_val -> coq_val option **)
  
  let divu v1 v2 =
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 ->
         if Int.eq n2 Int.zero then None else Some (Vint (Int.divu n1 n2))
       | _ -> None)
    | _ -> None
  
  (** val modu : coq_val -> coq_val -> coq_val option **)
  
  let modu v1 v2 =
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 ->
         if Int.eq n2 Int.zero then None else Some (Vint (Int.modu n1 n2))
       | _ -> None)
    | _ -> None
  
  (** val add_carry : coq_val -> coq_val -> coq_val -> coq_val **)
  
  let add_carry v1 v2 cin =
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 ->
         (match cin with
          | Vint c -> Vint (Int.add_carry n1 n2 c)
          | _ -> Vundef)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val sub_overflow : coq_val -> coq_val -> coq_val **)
  
  let sub_overflow v1 v2 =
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 -> Vint (Int.sub_overflow n1 n2 Int.zero)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val negative : coq_val -> coq_val **)
  
  let negative = function
  | Vint n -> Vint (Int.negative n)
  | _ -> Vundef
  
  (** val coq_and : coq_val -> coq_val -> coq_val **)
  
  let coq_and v1 v2 =
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 -> Vint (Int.coq_and n1 n2)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val coq_or : coq_val -> coq_val -> coq_val **)
  
  let coq_or v1 v2 =
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 -> Vint (Int.coq_or n1 n2)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val xor : coq_val -> coq_val -> coq_val **)
  
  let xor v1 v2 =
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 -> Vint (Int.xor n1 n2)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val shl : coq_val -> coq_val -> coq_val **)
  
  let shl v1 v2 =
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 ->
         if Int.ltu n2 Int.iwordsize then Vint (Int.shl n1 n2) else Vundef
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val shr : coq_val -> coq_val -> coq_val **)
  
  let shr v1 v2 =
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 ->
         if Int.ltu n2 Int.iwordsize then Vint (Int.shr n1 n2) else Vundef
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val shr_carry : coq_val -> coq_val -> coq_val **)
  
  let shr_carry v1 v2 =
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 ->
         if Int.ltu n2 Int.iwordsize
         then Vint (Int.shr_carry n1 n2)
         else Vundef
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val shrx : coq_val -> coq_val -> coq_val option **)
  
  let shrx v1 v2 =
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 ->
         if Int.ltu n2
              (Int.repr (Zpos (Coq_xI (Coq_xI (Coq_xI (Coq_xI Coq_xH))))))
         then Some (Vint (Int.shrx n1 n2))
         else None
       | _ -> None)
    | _ -> None
  
  (** val shru : coq_val -> coq_val -> coq_val **)
  
  let shru v1 v2 =
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 ->
         if Int.ltu n2 Int.iwordsize then Vint (Int.shru n1 n2) else Vundef
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val rolm : coq_val -> Int.int -> Int.int -> coq_val **)
  
  let rolm v amount mask =
    match v with
    | Vint n -> Vint (Int.rolm n amount mask)
    | _ -> Vundef
  
  (** val ror : coq_val -> coq_val -> coq_val **)
  
  let ror v1 v2 =
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 ->
         if Int.ltu n2 Int.iwordsize then Vint (Int.ror n1 n2) else Vundef
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val addf : coq_val -> coq_val -> coq_val **)
  
  let addf v1 v2 =
    match v1 with
    | Vfloat f1 ->
      (match v2 with
       | Vfloat f2 -> Vfloat (Float.add f1 f2)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val subf : coq_val -> coq_val -> coq_val **)
  
  let subf v1 v2 =
    match v1 with
    | Vfloat f1 ->
      (match v2 with
       | Vfloat f2 -> Vfloat (Float.sub f1 f2)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val mulf : coq_val -> coq_val -> coq_val **)
  
  let mulf v1 v2 =
    match v1 with
    | Vfloat f1 ->
      (match v2 with
       | Vfloat f2 -> Vfloat (Float.mul f1 f2)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val divf : coq_val -> coq_val -> coq_val **)
  
  let divf v1 v2 =
    match v1 with
    | Vfloat f1 ->
      (match v2 with
       | Vfloat f2 -> Vfloat (Float.div f1 f2)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val floatofwords : coq_val -> coq_val -> coq_val **)
  
  let floatofwords v1 v2 =
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 -> Vfloat (Float.from_words n1 n2)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val addfs : coq_val -> coq_val -> coq_val **)
  
  let addfs v1 v2 =
    match v1 with
    | Vsingle f1 ->
      (match v2 with
       | Vsingle f2 -> Vsingle (Float32.add f1 f2)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val subfs : coq_val -> coq_val -> coq_val **)
  
  let subfs v1 v2 =
    match v1 with
    | Vsingle f1 ->
      (match v2 with
       | Vsingle f2 -> Vsingle (Float32.sub f1 f2)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val mulfs : coq_val -> coq_val -> coq_val **)
  
  let mulfs v1 v2 =
    match v1 with
    | Vsingle f1 ->
      (match v2 with
       | Vsingle f2 -> Vsingle (Float32.mul f1 f2)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val divfs : coq_val -> coq_val -> coq_val **)
  
  let divfs v1 v2 =
    match v1 with
    | Vsingle f1 ->
      (match v2 with
       | Vsingle f2 -> Vsingle (Float32.div f1 f2)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val longofwords : coq_val -> coq_val -> coq_val **)
  
  let longofwords v1 v2 =
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 -> Vlong (Int64.ofwords n1 n2)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val loword : coq_val -> coq_val **)
  
  let loword = function
  | Vlong n -> Vint (Int64.loword n)
  | _ -> Vundef
  
  (** val hiword : coq_val -> coq_val **)
  
  let hiword = function
  | Vlong n -> Vint (Int64.hiword n)
  | _ -> Vundef
  
  (** val negl : coq_val -> coq_val **)
  
  let negl = function
  | Vlong n -> Vlong (Int64.neg n)
  | _ -> Vundef
  
  (** val notl : coq_val -> coq_val **)
  
  let notl = function
  | Vlong n -> Vlong (Int64.not n)
  | _ -> Vundef
  
  (** val longofint : coq_val -> coq_val **)
  
  let longofint = function
  | Vint n -> Vlong (Int64.repr (Int.signed n))
  | _ -> Vundef
  
  (** val longofintu : coq_val -> coq_val **)
  
  let longofintu = function
  | Vint n -> Vlong (Int64.repr (Int.unsigned n))
  | _ -> Vundef
  
  (** val longoffloat : coq_val -> coq_val option **)
  
  let longoffloat = function
  | Vfloat f -> option_map (fun x -> Vlong x) (Float.to_long f)
  | _ -> None
  
  (** val longuoffloat : coq_val -> coq_val option **)
  
  let longuoffloat = function
  | Vfloat f -> option_map (fun x -> Vlong x) (Float.to_longu f)
  | _ -> None
  
  (** val longofsingle : coq_val -> coq_val option **)
  
  let longofsingle = function
  | Vsingle f -> option_map (fun x -> Vlong x) (Float32.to_long f)
  | _ -> None
  
  (** val longuofsingle : coq_val -> coq_val option **)
  
  let longuofsingle = function
  | Vsingle f -> option_map (fun x -> Vlong x) (Float32.to_longu f)
  | _ -> None
  
  (** val floatoflong : coq_val -> coq_val option **)
  
  let floatoflong = function
  | Vlong n -> Some (Vfloat (Float.of_long n))
  | _ -> None
  
  (** val floatoflongu : coq_val -> coq_val option **)
  
  let floatoflongu = function
  | Vlong n -> Some (Vfloat (Float.of_longu n))
  | _ -> None
  
  (** val singleoflong : coq_val -> coq_val option **)
  
  let singleoflong = function
  | Vlong n -> Some (Vsingle (Float32.of_long n))
  | _ -> None
  
  (** val singleoflongu : coq_val -> coq_val option **)
  
  let singleoflongu = function
  | Vlong n -> Some (Vsingle (Float32.of_longu n))
  | _ -> None
  
  (** val addl : coq_val -> coq_val -> coq_val **)
  
  let addl v1 v2 =
    match v1 with
    | Vlong n1 ->
      (match v2 with
       | Vlong n2 -> Vlong (Int64.add n1 n2)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val subl : coq_val -> coq_val -> coq_val **)
  
  let subl v1 v2 =
    match v1 with
    | Vlong n1 ->
      (match v2 with
       | Vlong n2 -> Vlong (Int64.sub n1 n2)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val mull : coq_val -> coq_val -> coq_val **)
  
  let mull v1 v2 =
    match v1 with
    | Vlong n1 ->
      (match v2 with
       | Vlong n2 -> Vlong (Int64.mul n1 n2)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val mull' : coq_val -> coq_val -> coq_val **)
  
  let mull' v1 v2 =
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 -> Vlong (Int64.mul' n1 n2)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val divls : coq_val -> coq_val -> coq_val option **)
  
  let divls v1 v2 =
    match v1 with
    | Vlong n1 ->
      (match v2 with
       | Vlong n2 ->
         if (||) (Int64.eq n2 Int64.zero)
              ((&&) (Int64.eq n1 (Int64.repr Int64.min_signed))
                (Int64.eq n2 Int64.mone))
         then None
         else Some (Vlong (Int64.divs n1 n2))
       | _ -> None)
    | _ -> None
  
  (** val modls : coq_val -> coq_val -> coq_val option **)
  
  let modls v1 v2 =
    match v1 with
    | Vlong n1 ->
      (match v2 with
       | Vlong n2 ->
         if (||) (Int64.eq n2 Int64.zero)
              ((&&) (Int64.eq n1 (Int64.repr Int64.min_signed))
                (Int64.eq n2 Int64.mone))
         then None
         else Some (Vlong (Int64.mods n1 n2))
       | _ -> None)
    | _ -> None
  
  (** val divlu : coq_val -> coq_val -> coq_val option **)
  
  let divlu v1 v2 =
    match v1 with
    | Vlong n1 ->
      (match v2 with
       | Vlong n2 ->
         if Int64.eq n2 Int64.zero
         then None
         else Some (Vlong (Int64.divu n1 n2))
       | _ -> None)
    | _ -> None
  
  (** val modlu : coq_val -> coq_val -> coq_val option **)
  
  let modlu v1 v2 =
    match v1 with
    | Vlong n1 ->
      (match v2 with
       | Vlong n2 ->
         if Int64.eq n2 Int64.zero
         then None
         else Some (Vlong (Int64.modu n1 n2))
       | _ -> None)
    | _ -> None
  
  (** val andl : coq_val -> coq_val -> coq_val **)
  
  let andl v1 v2 =
    match v1 with
    | Vlong n1 ->
      (match v2 with
       | Vlong n2 -> Vlong (Int64.coq_and n1 n2)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val orl : coq_val -> coq_val -> coq_val **)
  
  let orl v1 v2 =
    match v1 with
    | Vlong n1 ->
      (match v2 with
       | Vlong n2 -> Vlong (Int64.coq_or n1 n2)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val xorl : coq_val -> coq_val -> coq_val **)
  
  let xorl v1 v2 =
    match v1 with
    | Vlong n1 ->
      (match v2 with
       | Vlong n2 -> Vlong (Int64.xor n1 n2)
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val shll : coq_val -> coq_val -> coq_val **)
  
  let shll v1 v2 =
    match v1 with
    | Vlong n1 ->
      (match v2 with
       | Vint n2 ->
         if Int.ltu n2 Int64.iwordsize'
         then Vlong (Int64.shl' n1 n2)
         else Vundef
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val shrl : coq_val -> coq_val -> coq_val **)
  
  let shrl v1 v2 =
    match v1 with
    | Vlong n1 ->
      (match v2 with
       | Vint n2 ->
         if Int.ltu n2 Int64.iwordsize'
         then Vlong (Int64.shr' n1 n2)
         else Vundef
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val shrlu : coq_val -> coq_val -> coq_val **)
  
  let shrlu v1 v2 =
    match v1 with
    | Vlong n1 ->
      (match v2 with
       | Vint n2 ->
         if Int.ltu n2 Int64.iwordsize'
         then Vlong (Int64.shru' n1 n2)
         else Vundef
       | _ -> Vundef)
    | _ -> Vundef
  
  (** val cmp_bool : comparison -> coq_val -> coq_val -> bool option **)
  
  let cmp_bool c v1 v2 =
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 -> Some (Int.cmp c n1 n2)
       | _ -> None)
    | _ -> None
  
  (** val cmp_different_blocks : comparison -> bool option **)
  
  let cmp_different_blocks = function
  | Ceq -> Some false
  | Cne -> Some true
  | _ -> None
  
  (** val cmpu_bool :
      (block -> coq_Z -> bool) -> comparison -> coq_val -> coq_val -> bool
      option **)
  
  let cmpu_bool valid_ptr =
    let weak_valid_ptr = fun b ofs ->
      (||) (valid_ptr b ofs) (valid_ptr b (Z.sub ofs (Zpos Coq_xH)))
    in
    (fun c v1 v2 ->
    match v1 with
    | Vint n1 ->
      (match v2 with
       | Vint n2 -> Some (Int.cmpu c n1 n2)
       | Vptr (b2, ofs2) ->
         if Int.eq n1 Int.zero then cmp_different_blocks c else None
       | _ -> None)
    | Vptr (b1, ofs1) ->
      (match v2 with
       | Vint n2 ->
         if Int.eq n2 Int.zero then cmp_different_blocks c else None
       | Vptr (b2, ofs2) ->
         if eq_block b1 b2
         then if (&&) (weak_valid_ptr b1 (Int.unsigned ofs1))
                   (weak_valid_ptr b2 (Int.unsigned ofs2))
              then Some (Int.cmpu c ofs1 ofs2)
              else None
         else if (&&) (valid_ptr b1 (Int.unsigned ofs1))
                   (valid_ptr b2 (Int.unsigned ofs2))
              then cmp_different_blocks c
              else None
       | _ -> None)
    | _ -> None)
  
  (** val cmpf_bool : comparison -> coq_val -> coq_val -> bool option **)
  
  let cmpf_bool c v1 v2 =
    match v1 with
    | Vfloat f1 ->
      (match v2 with
       | Vfloat f2 -> Some (Float.cmp c f1 f2)
       | _ -> None)
    | _ -> None
  
  (** val cmpfs_bool : comparison -> coq_val -> coq_val -> bool option **)
  
  let cmpfs_bool c v1 v2 =
    match v1 with
    | Vsingle f1 ->
      (match v2 with
       | Vsingle f2 -> Some (Float32.cmp c f1 f2)
       | _ -> None)
    | _ -> None
  
  (** val cmpl_bool : comparison -> coq_val -> coq_val -> bool option **)
  
  let cmpl_bool c v1 v2 =
    match v1 with
    | Vlong n1 ->
      (match v2 with
       | Vlong n2 -> Some (Int64.cmp c n1 n2)
       | _ -> None)
    | _ -> None
  
  (** val cmplu_bool : comparison -> coq_val -> coq_val -> bool option **)
  
  let cmplu_bool c v1 v2 =
    match v1 with
    | Vlong n1 ->
      (match v2 with
       | Vlong n2 -> Some (Int64.cmpu c n1 n2)
       | _ -> None)
    | _ -> None
  
  (** val of_optbool : bool option -> coq_val **)
  
  let of_optbool = function
  | Some b -> if b then coq_Vtrue else coq_Vfalse
  | None -> Vundef
  
  (** val cmp : comparison -> coq_val -> coq_val -> coq_val **)
  
  let cmp c v1 v2 =
    of_optbool (cmp_bool c v1 v2)
  
  (** val cmpu :
      (block -> coq_Z -> bool) -> comparison -> coq_val -> coq_val -> coq_val **)
  
  let cmpu valid_ptr c v1 v2 =
    of_optbool (cmpu_bool valid_ptr c v1 v2)
  
  (** val cmpf : comparison -> coq_val -> coq_val -> coq_val **)
  
  let cmpf c v1 v2 =
    of_optbool (cmpf_bool c v1 v2)
  
  (** val cmpfs : comparison -> coq_val -> coq_val -> coq_val **)
  
  let cmpfs c v1 v2 =
    of_optbool (cmpfs_bool c v1 v2)
  
  (** val cmpl : comparison -> coq_val -> coq_val -> coq_val option **)
  
  let cmpl c v1 v2 =
    option_map of_bool (cmpl_bool c v1 v2)
  
  (** val cmplu : comparison -> coq_val -> coq_val -> coq_val option **)
  
  let cmplu c v1 v2 =
    option_map of_bool (cmplu_bool c v1 v2)
  
  (** val maskzero_bool : coq_val -> Int.int -> bool option **)
  
  let maskzero_bool v mask =
    match v with
    | Vint n -> Some (Int.eq (Int.coq_and n mask) Int.zero)
    | _ -> None
  
  (** val load_result : memory_chunk -> coq_val -> coq_val **)
  
  let load_result chunk v =
    match chunk with
    | Mint8signed ->
      (match v with
       | Vint n ->
         Vint (Int.sign_ext (Zpos (Coq_xO (Coq_xO (Coq_xO Coq_xH)))) n)
       | _ -> Vundef)
    | Mint8unsigned ->
      (match v with
       | Vint n ->
         Vint (Int.zero_ext (Zpos (Coq_xO (Coq_xO (Coq_xO Coq_xH)))) n)
       | _ -> Vundef)
    | Mint16signed ->
      (match v with
       | Vint n ->
         Vint
           (Int.sign_ext (Zpos (Coq_xO (Coq_xO (Coq_xO (Coq_xO Coq_xH))))) n)
       | _ -> Vundef)
    | Mint16unsigned ->
      (match v with
       | Vint n ->
         Vint
           (Int.zero_ext (Zpos (Coq_xO (Coq_xO (Coq_xO (Coq_xO Coq_xH))))) n)
       | _ -> Vundef)
    | Mint32 ->
      (match v with
       | Vint n -> Vint n
       | Vptr (b, ofs) -> Vptr (b, ofs)
       | _ -> Vundef)
    | Mint64 ->
      (match v with
       | Vlong n -> Vlong n
       | _ -> Vundef)
    | Mfloat32 ->
      (match v with
       | Vsingle f -> Vsingle f
       | _ -> Vundef)
    | Mfloat64 ->
      (match v with
       | Vfloat f -> Vfloat f
       | _ -> Vundef)
    | Many32 ->
      (match v with
       | Vundef -> Vundef
       | Vlong i -> Vundef
       | Vfloat f -> Vundef
       | _ -> v)
    | Many64 -> v
 end

type meminj = block -> (block * coq_Z) option

