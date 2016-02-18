open Format

type oid = int

type ownership =
| OVar of oid
| OConst of int
| OPlus of ownership * ownership

let rec print_ownership fmt o =
  match o with
  | OVar o -> fprintf fmt "ovar%d" o
  | OConst o -> fprintf fmt "%d" o
  | OPlus (o1, o2) -> fprintf fmt "@[%a@] + @[%a@]" print_ownership o1 print_ownership o2

let owner_plus o1 o2 =
  OPlus (o1, o2)

let next_id = ref 0

let fresh_ovar =
  fun () ->
    let i = !next_id in
    incr next_id; i

let reset () =
  next_id := 0

let empty () =
  OConst 0
