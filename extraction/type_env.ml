open AST

type 'a t = (ident * 'a) list

let empty = []

let add (id, ty) env = (id, ty) :: env
let find id env = List.assoc id env
