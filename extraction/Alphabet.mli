open Compare_dec
open Datatypes
open Int31

type 'a coq_Comparable =
  'a -> 'a -> comparison
  (* singleton inductive, whose constructor was Build_Comparable *)

val compare : 'a1 coq_Comparable -> 'a1 -> 'a1 -> comparison

val natComparable : nat coq_Comparable

val coq_PairComparable :
  'a1 coq_Comparable -> 'a2 coq_Comparable -> ('a1 * 'a2) coq_Comparable

val compare_eqb : 'a1 coq_Comparable -> 'a1 -> 'a1 -> bool

val compare_eqdec : 'a1 coq_Comparable -> 'a1 -> 'a1 -> bool

type 'a coq_Finite =
  'a list
  (* singleton inductive, whose constructor was Build_Finite *)

val all_list : 'a1 coq_Finite -> 'a1 list

type 'a coq_Alphabet = { coq_AlphabetComparable : 'a coq_Comparable;
                         coq_AlphabetFinite : 'a coq_Finite }

val coq_AlphabetComparable : 'a1 coq_Alphabet -> 'a1 coq_Comparable

val coq_AlphabetFinite : 'a1 coq_Alphabet -> 'a1 coq_Finite

type 'a coq_Numbered = { inj : ('a -> int); surj : (int -> 'a);
                         inj_bound : int }

val inj : 'a1 coq_Numbered -> 'a1 -> int

val surj : 'a1 coq_Numbered -> int -> 'a1

val inj_bound : 'a1 coq_Numbered -> int

val coq_NumberedAlphabet : 'a1 coq_Numbered -> 'a1 coq_Alphabet

module type ComparableM = 
 sig 
  type t 
  
  val tComparable : t coq_Comparable
 end

module OrderedTypeAlt_from_ComparableM : 
 functor (C:ComparableM) ->
 sig 
  type t = C.t
  
  val compare : t -> t -> comparison
 end

module OrderedType_from_ComparableM : 
 functor (C:ComparableM) ->
 sig 
  module Alt : 
   sig 
    type t = C.t
    
    val compare : t -> t -> comparison
   end
  
  type t = Alt.t
  
  val compare : Alt.t -> Alt.t -> Alt.t OrderedType.coq_Compare
  
  val eq_dec : Alt.t -> Alt.t -> bool
 end

