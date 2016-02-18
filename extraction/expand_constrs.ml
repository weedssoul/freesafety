open AST
open Clight
open Ctypes
open Camlcoq
open Typing
open Ownership
open Format


let rec expand c s h =
(*  struct_unions := s;
    struct_tbl := h; *)
  let rec f l =
    match l with
    | [] -> []
    | hd :: tl ->
      match hd with
      | Le _ | Lt _ | Eq _ ->  hd :: f tl
      | Empty (id, i) -> (* expand_empty id i *)
                         hd :: f tl
      | Teq (l1, l2) -> (* expand_teq l1 l2 *) hd :: f tl
  in
  f c
