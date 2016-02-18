type oid = int

type ownership =
| OVar of oid
| OConst of int
| OPlus of ownership * ownership

val print_ownership: Format.formatter -> ownership -> unit

val owner_plus: ownership -> ownership -> ownership
val fresh_ovar: unit -> int
val reset: unit -> unit
val empty: unit -> ownership
