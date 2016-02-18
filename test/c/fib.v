Require Import Clightdefs.

Local Open Scope Z_scope.

Definition ___builtin_read32_reversed : ident := 32%positive.
Definition ___compcert_va_int32 : ident := 16%positive.
Definition _r : ident := 38%positive.
Definition ___builtin_fsqrt : ident := 24%positive.
Definition ___builtin_clz : ident := 22%positive.
Definition ___compcert_va_int64 : ident := 17%positive.
Definition ___builtin_memcpy_aligned : ident := 8%positive.
Definition ___builtin_subl : ident := 5%positive.
Definition ___builtin_va_start : ident := 12%positive.
Definition ___builtin_annot_intval : ident := 10%positive.
Definition ___builtin_negl : ident := 3%positive.
Definition ___builtin_write32_reversed : ident := 2%positive.
Definition ___builtin_write16_reversed : ident := 1%positive.
Definition _argc : ident := 36%positive.
Definition ___builtin_read16_reversed : ident := 31%positive.
Definition ___builtin_va_copy : ident := 14%positive.
Definition ___builtin_mull : ident := 6%positive.
Definition ___builtin_fmin : ident := 26%positive.
Definition ___builtin_bswap : ident := 19%positive.
Definition _main : ident := 39%positive.
Definition ___builtin_membar : ident := 11%positive.
Definition _argv : ident := 37%positive.
Definition ___builtin_addl : ident := 4%positive.
Definition ___builtin_fmsub : ident := 28%positive.
Definition ___builtin_fabs : ident := 7%positive.
Definition ___builtin_bswap16 : ident := 21%positive.
Definition ___compcert_va_float64 : ident := 18%positive.
Definition ___builtin_annot : ident := 9%positive.
Definition ___builtin_va_arg : ident := 13%positive.
Definition ___builtin_fmadd : ident := 27%positive.
Definition _atoi : ident := 33%positive.
Definition ___builtin_fmax : ident := 25%positive.
Definition ___builtin_va_end : ident := 15%positive.
Definition _n : ident := 34%positive.
Definition ___builtin_fnmadd : ident := 29%positive.
Definition _fib : ident := 35%positive.
Definition ___builtin_fnmsub : ident := 30%positive.
Definition ___builtin_ctz : ident := 23%positive.
Definition ___builtin_bswap32 : ident := 20%positive.
Definition _r' : ident := 43%positive.
Definition _n' : ident := 42%positive.


Definition f_fib := {|
  fn_return := tint;
  fn_callconv := cc_default;
  fn_params := ((_n, tint) :: nil);
  fn_vars := nil;
  fn_temps := ((41%positive, tint) :: (40%positive, tint) :: nil);
  fn_body :=
(Sifthenelse (Ebinop Olt (Etempvar _n tint) (Econst_int (Int.repr 2) tint)
               tint)
  (Sreturn (Some (Econst_int (Int.repr 1) tint)))
  (Ssequence
    (Ssequence
      (Scall (Some 40%positive)
        (Evar _fib (Tfunction (Tcons tint Tnil) tint cc_default))
        ((Ebinop Osub (Etempvar _n tint) (Econst_int (Int.repr 1) tint) tint) ::
         nil))
      (Scall (Some 41%positive)
        (Evar _fib (Tfunction (Tcons tint Tnil) tint cc_default))
        ((Ebinop Osub (Etempvar _n tint) (Econst_int (Int.repr 2) tint) tint) ::
         nil)))
    (Sreturn (Some (Ebinop Oadd (Etempvar 40%positive tint)
                     (Etempvar 41%positive tint) tint)))))
|}.

Definition f_main := {|
  fn_return := tint;
  fn_callconv := cc_default;
  fn_params := ((_argc, tint) :: (_argv, (tptr (tptr tschar))) :: nil);
  fn_vars := nil;
  fn_temps := ((_n, tint) :: (_r, tint) :: (_r', tint) :: (_n', tint) :: nil);
  fn_body :=
(Ssequence
  (Sifthenelse (Ebinop Oge (Etempvar _argc tint)
                 (Econst_int (Int.repr 2) tint) tint)
    (Ssequence
      (Scall (Some _n')
        (Evar _atoi (Tfunction (Tcons (tptr tschar) Tnil) tint cc_default))
        ((Ederef
           (Ebinop Oadd (Etempvar _argv (tptr (tptr tschar)))
             (Econst_int (Int.repr 1) tint) (tptr (tptr tschar)))
           (tptr tschar)) :: nil))
      (Sset _n (Etempvar _n' tint)))
    (Sset _n (Econst_int (Int.repr 36) tint)))
  (Ssequence
    (Ssequence
      (Scall (Some _r')
        (Evar _fib (Tfunction (Tcons tint Tnil) tint cc_default))
        ((Etempvar _n tint) :: nil))
      (Sset _r (Etempvar _r' tint)))
    (Sreturn (Some (Econst_int (Int.repr 0) tint)))))
|}.

Definition prog : Clight.program := {|
prog_defs :=
((___builtin_fabs,
   Gfun(External (EF_builtin ___builtin_fabs
                   (mksignature (AST.Tfloat :: nil) (Some AST.Tfloat)
                     cc_default)) (Tcons tdouble Tnil) tdouble cc_default)) ::
 (___builtin_memcpy_aligned,
   Gfun(External (EF_builtin ___builtin_memcpy_aligned
                   (mksignature
                     (AST.Tint :: AST.Tint :: AST.Tint :: AST.Tint :: nil)
                     None cc_default))
     (Tcons (tptr tvoid)
       (Tcons (tptr tvoid) (Tcons tuint (Tcons tuint Tnil)))) tvoid
     cc_default)) ::
 (___builtin_annot,
   Gfun(External (EF_builtin ___builtin_annot
                   (mksignature (AST.Tint :: nil) None
                     {|cc_vararg:=true; cc_structret:=false|}))
     (Tcons (tptr tschar) Tnil) tvoid
     {|cc_vararg:=true; cc_structret:=false|})) ::
 (___builtin_annot_intval,
   Gfun(External (EF_builtin ___builtin_annot_intval
                   (mksignature (AST.Tint :: AST.Tint :: nil) (Some AST.Tint)
                     cc_default)) (Tcons (tptr tschar) (Tcons tint Tnil))
     tint cc_default)) ::
 (___builtin_membar,
   Gfun(External (EF_builtin ___builtin_membar
                   (mksignature nil None cc_default)) Tnil tvoid cc_default)) ::
 (___builtin_va_start,
   Gfun(External (EF_builtin ___builtin_va_start
                   (mksignature (AST.Tint :: nil) None cc_default))
     (Tcons (tptr tvoid) Tnil) tvoid cc_default)) ::
 (___builtin_va_arg,
   Gfun(External (EF_builtin ___builtin_va_arg
                   (mksignature (AST.Tint :: AST.Tint :: nil) None
                     cc_default)) (Tcons (tptr tvoid) (Tcons tuint Tnil))
     tvoid cc_default)) ::
 (___builtin_va_copy,
   Gfun(External (EF_builtin ___builtin_va_copy
                   (mksignature (AST.Tint :: AST.Tint :: nil) None
                     cc_default))
     (Tcons (tptr tvoid) (Tcons (tptr tvoid) Tnil)) tvoid cc_default)) ::
 (___builtin_va_end,
   Gfun(External (EF_builtin ___builtin_va_end
                   (mksignature (AST.Tint :: nil) None cc_default))
     (Tcons (tptr tvoid) Tnil) tvoid cc_default)) ::
 (___compcert_va_int32,
   Gfun(External (EF_external ___compcert_va_int32
                   (mksignature (AST.Tint :: nil) (Some AST.Tint) cc_default))
     (Tcons (tptr tvoid) Tnil) tuint cc_default)) ::
 (___compcert_va_int64,
   Gfun(External (EF_external ___compcert_va_int64
                   (mksignature (AST.Tint :: nil) (Some AST.Tlong)
                     cc_default)) (Tcons (tptr tvoid) Tnil) tulong
     cc_default)) ::
 (___compcert_va_float64,
   Gfun(External (EF_external ___compcert_va_float64
                   (mksignature (AST.Tint :: nil) (Some AST.Tfloat)
                     cc_default)) (Tcons (tptr tvoid) Tnil) tdouble
     cc_default)) ::
 (___builtin_bswap,
   Gfun(External (EF_builtin ___builtin_bswap
                   (mksignature (AST.Tint :: nil) (Some AST.Tint) cc_default))
     (Tcons tuint Tnil) tuint cc_default)) ::
 (___builtin_bswap32,
   Gfun(External (EF_builtin ___builtin_bswap32
                   (mksignature (AST.Tint :: nil) (Some AST.Tint) cc_default))
     (Tcons tuint Tnil) tuint cc_default)) ::
 (___builtin_bswap16,
   Gfun(External (EF_builtin ___builtin_bswap16
                   (mksignature (AST.Tint :: nil) (Some AST.Tint) cc_default))
     (Tcons tushort Tnil) tushort cc_default)) ::
 (___builtin_clz,
   Gfun(External (EF_builtin ___builtin_clz
                   (mksignature (AST.Tint :: nil) (Some AST.Tint) cc_default))
     (Tcons tuint Tnil) tuint cc_default)) ::
 (___builtin_ctz,
   Gfun(External (EF_builtin ___builtin_ctz
                   (mksignature (AST.Tint :: nil) (Some AST.Tint) cc_default))
     (Tcons tuint Tnil) tuint cc_default)) ::
 (___builtin_fsqrt,
   Gfun(External (EF_builtin ___builtin_fsqrt
                   (mksignature (AST.Tfloat :: nil) (Some AST.Tfloat)
                     cc_default)) (Tcons tdouble Tnil) tdouble cc_default)) ::
 (___builtin_fmax,
   Gfun(External (EF_builtin ___builtin_fmax
                   (mksignature (AST.Tfloat :: AST.Tfloat :: nil)
                     (Some AST.Tfloat) cc_default))
     (Tcons tdouble (Tcons tdouble Tnil)) tdouble cc_default)) ::
 (___builtin_fmin,
   Gfun(External (EF_builtin ___builtin_fmin
                   (mksignature (AST.Tfloat :: AST.Tfloat :: nil)
                     (Some AST.Tfloat) cc_default))
     (Tcons tdouble (Tcons tdouble Tnil)) tdouble cc_default)) ::
 (___builtin_fmadd,
   Gfun(External (EF_builtin ___builtin_fmadd
                   (mksignature
                     (AST.Tfloat :: AST.Tfloat :: AST.Tfloat :: nil)
                     (Some AST.Tfloat) cc_default))
     (Tcons tdouble (Tcons tdouble (Tcons tdouble Tnil))) tdouble
     cc_default)) ::
 (___builtin_fmsub,
   Gfun(External (EF_builtin ___builtin_fmsub
                   (mksignature
                     (AST.Tfloat :: AST.Tfloat :: AST.Tfloat :: nil)
                     (Some AST.Tfloat) cc_default))
     (Tcons tdouble (Tcons tdouble (Tcons tdouble Tnil))) tdouble
     cc_default)) ::
 (___builtin_fnmadd,
   Gfun(External (EF_builtin ___builtin_fnmadd
                   (mksignature
                     (AST.Tfloat :: AST.Tfloat :: AST.Tfloat :: nil)
                     (Some AST.Tfloat) cc_default))
     (Tcons tdouble (Tcons tdouble (Tcons tdouble Tnil))) tdouble
     cc_default)) ::
 (___builtin_fnmsub,
   Gfun(External (EF_builtin ___builtin_fnmsub
                   (mksignature
                     (AST.Tfloat :: AST.Tfloat :: AST.Tfloat :: nil)
                     (Some AST.Tfloat) cc_default))
     (Tcons tdouble (Tcons tdouble (Tcons tdouble Tnil))) tdouble
     cc_default)) ::
 (___builtin_read16_reversed,
   Gfun(External (EF_builtin ___builtin_read16_reversed
                   (mksignature (AST.Tint :: nil) (Some AST.Tint) cc_default))
     (Tcons (tptr tushort) Tnil) tushort cc_default)) ::
 (___builtin_read32_reversed,
   Gfun(External (EF_builtin ___builtin_read32_reversed
                   (mksignature (AST.Tint :: nil) (Some AST.Tint) cc_default))
     (Tcons (tptr tuint) Tnil) tuint cc_default)) ::
 (___builtin_write16_reversed,
   Gfun(External (EF_builtin ___builtin_write16_reversed
                   (mksignature (AST.Tint :: AST.Tint :: nil) None
                     cc_default)) (Tcons (tptr tushort) (Tcons tushort Tnil))
     tvoid cc_default)) ::
 (___builtin_write32_reversed,
   Gfun(External (EF_builtin ___builtin_write32_reversed
                   (mksignature (AST.Tint :: AST.Tint :: nil) None
                     cc_default)) (Tcons (tptr tuint) (Tcons tuint Tnil))
     tvoid cc_default)) ::
 (_atoi,
   Gfun(External (EF_external _atoi
                   (mksignature (AST.Tint :: nil) (Some AST.Tint) cc_default))
     (Tcons (tptr tschar) Tnil) tint cc_default)) ::
 (_fib, Gfun(Internal f_fib)) :: (_main, Gfun(Internal f_main)) :: nil);
prog_main := _main
|}.

