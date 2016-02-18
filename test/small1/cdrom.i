





















struct sched_param {
 int sched_priority;
};




























typedef struct {
 unsigned long fds_bits [(1024/(8 * sizeof(unsigned long)))];
} __kernel_fd_set;


typedef void (*__kernel_sighandler_t)(int);


typedef int __kernel_key_t;
typedef int __kernel_mqd_t;



typedef unsigned long __kernel_ino_t;
typedef unsigned short __kernel_mode_t;
typedef unsigned short __kernel_nlink_t;
typedef long __kernel_off_t;
typedef int __kernel_pid_t;
typedef unsigned short __kernel_ipc_pid_t;
typedef unsigned short __kernel_uid_t;
typedef unsigned short __kernel_gid_t;
typedef unsigned int __kernel_size_t;
typedef int __kernel_ssize_t;
typedef int __kernel_ptrdiff_t;
typedef long __kernel_time_t;
typedef long __kernel_suseconds_t;
typedef long __kernel_clock_t;
typedef int __kernel_timer_t;
typedef int __kernel_clockid_t;
typedef int __kernel_daddr_t;
typedef char * __kernel_caddr_t;
typedef unsigned short __kernel_uid16_t;
typedef unsigned short __kernel_gid16_t;
typedef unsigned int __kernel_uid32_t;
typedef unsigned int __kernel_gid32_t;

typedef unsigned short __kernel_old_uid_t;
typedef unsigned short __kernel_old_gid_t;
typedef unsigned short __kernel_old_dev_t;


typedef long long __kernel_loff_t;


typedef struct {

 int val[2];



} __kernel_fsid_t;








typedef unsigned short umode_t;






typedef signed char __s8;
typedef unsigned char __u8;

typedef signed short __s16;
typedef unsigned short __u16;

typedef signed int __s32;
typedef unsigned int __u32;


typedef signed long long __s64;
typedef unsigned long long __u64;

typedef signed char s8;
typedef unsigned char u8;

typedef signed short s16;
typedef unsigned short u16;

typedef signed int s32;
typedef unsigned int u32;

typedef signed long long s64;
typedef unsigned long long u64;






typedef u32 dma_addr_t;

typedef u64 dma64_addr_t;


typedef u64 sector_t;




typedef u64 blkcnt_t;




typedef __u32 __kernel_dev_t;

typedef __kernel_fd_set fd_set;
typedef __kernel_dev_t dev_t;
typedef __kernel_ino_t ino_t;
typedef __kernel_mode_t mode_t;
typedef __kernel_nlink_t nlink_t;
typedef __kernel_off_t off_t;
typedef __kernel_pid_t pid_t;
typedef __kernel_daddr_t daddr_t;
typedef __kernel_key_t key_t;
typedef __kernel_suseconds_t suseconds_t;
typedef __kernel_timer_t timer_t;
typedef __kernel_clockid_t clockid_t;
typedef __kernel_mqd_t mqd_t;


typedef __kernel_uid32_t uid_t;
typedef __kernel_gid32_t gid_t;
typedef __kernel_uid16_t uid16_t;
typedef __kernel_gid16_t gid16_t;



typedef __kernel_old_uid_t old_uid_t;
typedef __kernel_old_gid_t old_gid_t;

typedef __kernel_loff_t loff_t;

typedef __kernel_size_t size_t;




typedef __kernel_ssize_t ssize_t;




typedef __kernel_ptrdiff_t ptrdiff_t;




typedef __kernel_time_t time_t;




typedef __kernel_clock_t clock_t;




typedef __kernel_caddr_t caddr_t;



typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;


typedef unsigned char unchar;
typedef unsigned short ushort;
typedef unsigned int uint;
typedef unsigned long ulong;




typedef __u8 u_int8_t;
typedef __s8 int8_t;
typedef __u16 u_int16_t;
typedef __s16 int16_t;
typedef __u32 u_int32_t;
typedef __s32 int32_t;



typedef __u8 uint8_t;
typedef __u16 uint16_t;
typedef __u32 uint32_t;


typedef __u64 uint64_t;
typedef __u64 u_int64_t;
typedef __s64 int64_t;

typedef __u16 __le16;
typedef __u16 __be16;
typedef __u32 __le32;
typedef __u32 __be32;

typedef __u64 __le64;
typedef __u64 __be64;



typedef unsigned gfp_t;


typedef u64 resource_size_t;






struct ustat {
 __kernel_daddr_t f_tfree;
 __kernel_ino_t f_tinode;
 char f_fname[6];
 char f_fpack[6];
};


typedef struct __user_cap_header_struct {
 __u32 version;
 int pid;
} *cap_user_header_t;

typedef struct __user_cap_data_struct {
        __u32 effective;
        __u32 permitted;
        __u32 inheritable;
} *cap_user_data_t;









struct restart_block {
 long (*fn)(struct restart_block *);
 unsigned long arg0, arg1, arg2, arg3;
};

extern long do_no_restart_syscall(struct restart_block *parm);







struct alt_instr {
 u8 *instr;
 u8 *replacement;
 u8 cpuid;
 u8 instrlen;
 u8 replacementlen;
 u8 pad;
};

extern void apply_alternatives(struct alt_instr *start, struct alt_instr *end);

struct module;







static inline __attribute__((always_inline)) void alternatives_smp_module_add(struct module *mod, char *name,
     void *locks, void *locks_end,
     void *text, void *text_end) {}
static inline __attribute__((always_inline)) void alternatives_smp_module_del(struct module *mod) {}
static inline __attribute__((always_inline)) void alternatives_smp_switch(int smp) {}


static inline __attribute__((always_inline)) void set_bit(int nr, volatile unsigned long * addr)
{
 __asm__ __volatile__( ""
  "btsl %1,%0"
  :"+m" ((*(volatile long *) addr))
  :"Ir" (nr));
}

static inline __attribute__((always_inline)) void __set_bit(int nr, volatile unsigned long * addr)
{
 __asm__(
  "btsl %1,%0"
  :"+m" ((*(volatile long *) addr))
  :"Ir" (nr));
}

static inline __attribute__((always_inline)) void clear_bit(int nr, volatile unsigned long * addr)
{
 __asm__ __volatile__( ""
  "btrl %1,%0"
  :"+m" ((*(volatile long *) addr))
  :"Ir" (nr));
}

static inline __attribute__((always_inline)) void __clear_bit(int nr, volatile unsigned long * addr)
{
 __asm__ __volatile__(
  "btrl %1,%0"
  :"+m" ((*(volatile long *) addr))
  :"Ir" (nr));
}

static inline __attribute__((always_inline)) void __change_bit(int nr, volatile unsigned long * addr)
{
 __asm__ __volatile__(
  "btcl %1,%0"
  :"+m" ((*(volatile long *) addr))
  :"Ir" (nr));
}

static inline __attribute__((always_inline)) void change_bit(int nr, volatile unsigned long * addr)
{
 __asm__ __volatile__( ""
  "btcl %1,%0"
  :"+m" ((*(volatile long *) addr))
  :"Ir" (nr));
}

static inline __attribute__((always_inline)) int test_and_set_bit(int nr, volatile unsigned long * addr)
{
 int oldbit;

 __asm__ __volatile__( ""
  "btsl %2,%1\n\tsbbl %0,%0"
  :"=r" (oldbit),"+m" ((*(volatile long *) addr))
  :"Ir" (nr) : "memory");
 return oldbit;
}

static inline __attribute__((always_inline)) int __test_and_set_bit(int nr, volatile unsigned long * addr)
{
 int oldbit;

 __asm__(
  "btsl %2,%1\n\tsbbl %0,%0"
  :"=r" (oldbit),"+m" ((*(volatile long *) addr))
  :"Ir" (nr));
 return oldbit;
}

static inline __attribute__((always_inline)) int test_and_clear_bit(int nr, volatile unsigned long * addr)
{
 int oldbit;

 __asm__ __volatile__( ""
  "btrl %2,%1\n\tsbbl %0,%0"
  :"=r" (oldbit),"+m" ((*(volatile long *) addr))
  :"Ir" (nr) : "memory");
 return oldbit;
}

static inline __attribute__((always_inline)) int __test_and_clear_bit(int nr, volatile unsigned long *addr)
{
 int oldbit;

 __asm__(
  "btrl %2,%1\n\tsbbl %0,%0"
  :"=r" (oldbit),"+m" ((*(volatile long *) addr))
  :"Ir" (nr));
 return oldbit;
}


static inline __attribute__((always_inline)) int __test_and_change_bit(int nr, volatile unsigned long *addr)
{
 int oldbit;

 __asm__ __volatile__(
  "btcl %2,%1\n\tsbbl %0,%0"
  :"=r" (oldbit),"+m" ((*(volatile long *) addr))
  :"Ir" (nr) : "memory");
 return oldbit;
}

static inline __attribute__((always_inline)) int test_and_change_bit(int nr, volatile unsigned long* addr)
{
 int oldbit;

 __asm__ __volatile__( ""
  "btcl %2,%1\n\tsbbl %0,%0"
  :"=r" (oldbit),"+m" ((*(volatile long *) addr))
  :"Ir" (nr) : "memory");
 return oldbit;
}

static inline __attribute__((always_inline)) __attribute__((always_inline)) int constant_test_bit(int nr, const volatile unsigned long *addr)
{
  return ((1UL << (nr & 31)) & (addr[nr >> 5])) != 0;
}

static inline __attribute__((always_inline)) int variable_test_bit(int nr, const volatile unsigned long * addr)
{
 int oldbit;

 __asm__ __volatile__(
  "btl %2,%1\n\tsbbl %0,%0"
  :"=r" (oldbit)
  :"m" ((*(volatile long *) addr)),"Ir" (nr));
 return oldbit;
}

static inline __attribute__((always_inline)) int find_first_zero_bit(const unsigned long *addr, unsigned size)
{
 int d0, d1, d2;
 int res;

 if (!size)
  return 0;

 __asm__ __volatile__(
  "movl $-1,%%eax\n\t"
  "xorl %%edx,%%edx\n\t"
  "repe; scasl\n\t"
  "je 1f\n\t"
  "xorl -4(%%edi),%%eax\n\t"
  "subl $4,%%edi\n\t"
  "bsfl %%eax,%%edx\n"
  "1:\tsubl %%ebx,%%edi\n\t"
  "shll $3,%%edi\n\t"
  "addl %%edi,%%edx"
  :"=d" (res), "=&c" (d0), "=&D" (d1), "=&a" (d2)
  :"1" ((size + 31) >> 5), "2" (addr), "b" (addr) : "memory");
 return res;
}







int find_next_zero_bit(const unsigned long *addr, int size, int offset);







static inline __attribute__((always_inline)) unsigned long __ffs(unsigned long word)
{
 __asm__("bsfl %1,%0"
  :"=r" (word)
  :"rm" (word));
 return word;
}

static inline __attribute__((always_inline)) unsigned find_first_bit(const unsigned long *addr, unsigned size)
{
 unsigned x = 0;

 while (x < size) {
  unsigned long val = *addr++;
  if (val)
   return __ffs(val) + x;
  x += (sizeof(*addr)<<3);
 }
 return x;
}







int find_next_bit(const unsigned long *addr, int size, int offset);







static inline __attribute__((always_inline)) unsigned long ffz(unsigned long word)
{
 __asm__("bsfl %1,%0"
  :"=r" (word)
  :"r" (~word));
 return word;
}





static inline __attribute__((always_inline)) int sched_find_first_bit(const unsigned long *b)
{







 if (__builtin_expect(!!(b[0]), 0))
  return __ffs(b[0]);
 if (__builtin_expect(!!(b[1]), 0))
  return __ffs(b[1]) + 32;
 if (__builtin_expect(!!(b[2]), 0))
  return __ffs(b[2]) + 64;
 if (b[3])
  return __ffs(b[3]) + 96;
 return __ffs(b[4]) + 128;



}


static inline __attribute__((always_inline)) int ffs(int x)
{
 int r;

 __asm__("bsfl %1,%0\n\t"
  "jnz 1f\n\t"
  "movl $-1,%0\n"
  "1:" : "=r" (r) : "rm" (x));
 return r+1;
}







static inline __attribute__((always_inline)) int fls(int x)
{
 int r;

 __asm__("bsrl %1,%0\n\t"
  "jnz 1f\n\t"
  "movl $-1,%0\n"
  "1:" : "=r" (r) : "rm" (x));
 return r+1;
}







extern unsigned int hweight32(unsigned int w);
extern unsigned int hweight16(unsigned int w);
extern unsigned int hweight8(unsigned int w);
extern unsigned long hweight64(__u64 w);










static inline __attribute__((always_inline)) int fls64(__u64 x)
{
 __u32 h = x >> 32;
 if (h)
  return fls(h) + 32;
 return fls(x);
}















static __inline__ __attribute__((always_inline)) __attribute__((__const__)) __u32 ___arch__swab32(__u32 x)
{

 __asm__("bswap %0" : "=r" (x) : "0" (x));







 return x;
}
// here
static __inline__ __attribute__((always_inline)) __attribute__((__const__)) __u64 ___arch__swab64(__u64 val)
{
 union {
  struct { __u32 a,b; } s;
  __u64 u;
 } v;
 v.u = val;

 asm("bswapl %0 ; bswapl %1 ; xchgl %0,%1"
     : "=r" (v.s.a), "=r" (v.s.b)
     : "0" (v.s.a), "1" (v.s.b));





 return v.u;
}





static __inline__ __attribute__((always_inline)) __attribute__((__const__)) __u16 __fswab16(__u16 x)
{
 return ({ __u16 __tmp = (x) ; ({ __u16 __x = (__tmp); ((__u16)( (((__u16)(__x) & (__u16)0x00ffU) << 8) | (((__u16)(__x) & (__u16)0xff00U) >> 8) )); }); });
}
static __inline__ __attribute__((always_inline)) __u16 __swab16p(const __u16 *x)
{
 return ({ __u16 __tmp = (*(x)) ; ({ __u16 __x = (__tmp); ((__u16)( (((__u16)(__x) & (__u16)0x00ffU) << 8) | (((__u16)(__x) & (__u16)0xff00U) >> 8) )); }); });
}
static __inline__ __attribute__((always_inline)) void __swab16s(__u16 *addr)
{
 do { *(addr) = ({ __u16 __tmp = (*((addr))) ; ({ __u16 __x = (__tmp); ((__u16)( (((__u16)(__x) & (__u16)0x00ffU) << 8) | (((__u16)(__x) & (__u16)0xff00U) >> 8) )); }); }); } while (0);
}

static __inline__ __attribute__((always_inline)) __attribute__((__const__)) __u32 __fswab32(__u32 x)
{
 return ___arch__swab32(x);
}
static __inline__ __attribute__((always_inline)) __u32 __swab32p(const __u32 *x)
{
 return ___arch__swab32(*(x));
}
static __inline__ __attribute__((always_inline)) void __swab32s(__u32 *addr)
{
 do { *(addr) = ___arch__swab32(*((addr))); } while (0);
}


static __inline__ __attribute__((always_inline)) __attribute__((__const__)) __u64 __fswab64(__u64 x)
{





 return ___arch__swab64(x);

}
static __inline__ __attribute__((always_inline)) __u64 __swab64p(const __u64 *x)
{
 return ___arch__swab64(*(x));
}
static __inline__ __attribute__((always_inline)) void __swab64s(__u64 *addr)
{
 do { *(addr) = ___arch__swab64(*((addr))); } while (0);
}


static inline __attribute__((always_inline)) __le64 __cpu_to_le64p(const __u64 *p)
{
 return ( __le64)*p;
}
static inline __attribute__((always_inline)) __u64 __le64_to_cpup(const __le64 *p)
{
 return ( __u64)*p;
}
static inline __attribute__((always_inline)) __le32 __cpu_to_le32p(const __u32 *p)
{
 return ( __le32)*p;
}
static inline __attribute__((always_inline)) __u32 __le32_to_cpup(const __le32 *p)
{
 return ( __u32)*p;
}
static inline __attribute__((always_inline)) __le16 __cpu_to_le16p(const __u16 *p)
{
 return ( __le16)*p;
}
static inline __attribute__((always_inline)) __u16 __le16_to_cpup(const __le16 *p)
{
 return ( __u16)*p;
}
static inline __attribute__((always_inline)) __be64 __cpu_to_be64p(const __u64 *p)
{
 return ( __be64)__swab64p(p);
}
static inline __attribute__((always_inline)) __u64 __be64_to_cpup(const __be64 *p)
{
 return __swab64p((__u64 *)p);
}
static inline __attribute__((always_inline)) __be32 __cpu_to_be32p(const __u32 *p)
{
 return ( __be32)__swab32p(p);
}
static inline __attribute__((always_inline)) __u32 __be32_to_cpup(const __be32 *p)
{
 return __swab32p((__u32 *)p);
}
static inline __attribute__((always_inline)) __be16 __cpu_to_be16p(const __u16 *p)
{
 return ( __be16)__swab16p(p);
}
static inline __attribute__((always_inline)) __u16 __be16_to_cpup(const __be16 *p)
{
 return __swab16p((__u16 *)p);
}



extern __u32 ntohl(__be32);
extern __be32 htonl(__u32);
extern __u16 ntohs(__be16);
extern __be16 htons(__u16);















static __inline__ __attribute__((always_inline)) int get_bitmask_order(unsigned int count)
{
 int order;

 order = fls(count);
 return order;
}

static __inline__ __attribute__((always_inline)) int get_count_order(unsigned int count)
{
 int order;

 order = fls(count) - 1;
 if (count & (count - 1))
  order++;
 return order;
}

static inline __attribute__((always_inline)) unsigned long hweight_long(unsigned long w)
{
 return sizeof(w) == 4 ? hweight32(w) : hweight64(w);
}







static inline __attribute__((always_inline)) __u32 rol32(__u32 word, unsigned int shift)
{
 return (word << shift) | (word >> (32 - shift));
}







static inline __attribute__((always_inline)) __u32 ror32(__u32 word, unsigned int shift)
{
 return (word >> shift) | (word << (32 - shift));
}

static inline __attribute__((always_inline)) unsigned fls_long(unsigned long l)
{
 if (sizeof(l) == 4)
  return fls(l);
 return fls64(l);
}





extern int nx_enabled;

typedef struct { unsigned long pte_low; } pte_t;
typedef struct { unsigned long pgd; } pgd_t;
typedef struct { unsigned long pgprot; } pgprot_t;

struct vm_area_struct;





extern unsigned int __VMALLOC_RESERVE;

extern int sysctl_legacy_va_layout;

extern int page_is_ram(unsigned long pagenr);





static __inline__ __attribute__((always_inline)) __attribute__((__const__)) int get_order(unsigned long size)
{
 int order;

 size = (size - 1) >> (12 - 1);
 order = -1;
 do {
  size >>= 1;
  order++;
 } while (size);
 return order;
}








struct vm86_regs {



 long ebx;
 long ecx;
 long edx;
 long esi;
 long edi;
 long ebp;
 long eax;
 long __null_ds;
 long __null_es;
 long __null_fs;
 long __null_gs;
 long orig_eax;
 long eip;
 unsigned short cs, __csh;
 long eflags;
 long esp;
 unsigned short ss, __ssh;



 unsigned short es, __esh;
 unsigned short ds, __dsh;
 unsigned short fs, __fsh;
 unsigned short gs, __gsh;
};

struct revectored_struct {
 unsigned long __map[8];
};

struct vm86_struct {
 struct vm86_regs regs;
 unsigned long flags;
 unsigned long screen_bitmap;
 unsigned long cpu_type;
 struct revectored_struct int_revectored;
 struct revectored_struct int21_revectored;
};






struct vm86plus_info_struct {
 unsigned long force_return_for_pic:1;
 unsigned long vm86dbg_active:1;
 unsigned long vm86dbg_TFpendig:1;
 unsigned long unused:28;
 unsigned long is_vm86pus:1;
 unsigned char vm86dbg_intxxtab[32];
};

struct vm86plus_struct {
 struct vm86_regs regs;
 unsigned long flags;
 unsigned long screen_bitmap;
 unsigned long cpu_type;
 struct revectored_struct int_revectored;
 struct revectored_struct int21_revectored;
 struct vm86plus_info_struct vm86plus;
};

struct kernel_vm86_regs {



 long ebx;
 long ecx;
 long edx;
 long esi;
 long edi;
 long ebp;
 long eax;
 long __null_ds;
 long __null_es;
 long orig_eax;
 long eip;
 unsigned short cs, __csh;
 long eflags;
 long esp;
 unsigned short ss, __ssh;



 unsigned short es, __esh;
 unsigned short ds, __dsh;
 unsigned short fs, __fsh;
 unsigned short gs, __gsh;
};

struct kernel_vm86_struct {
 struct kernel_vm86_regs regs;

 unsigned long flags;
 unsigned long screen_bitmap;
 unsigned long cpu_type;
 struct revectored_struct int_revectored;
 struct revectored_struct int21_revectored;
 struct vm86plus_info_struct vm86plus;
 struct pt_regs *regs32;

};



void handle_vm86_fault(struct kernel_vm86_regs *, long);
int handle_vm86_trap(struct kernel_vm86_regs *, long, int);

struct task_struct;
void release_vm86_irqs(struct task_struct *);







struct _fpreg {
 unsigned short significand[4];
 unsigned short exponent;
};

struct _fpxreg {
 unsigned short significand[4];
 unsigned short exponent;
 unsigned short padding[3];
};

struct _xmmreg {
 unsigned long element[4];
};

struct _fpstate {

 unsigned long cw;
 unsigned long sw;
 unsigned long tag;
 unsigned long ipoff;
 unsigned long cssel;
 unsigned long dataoff;
 unsigned long datasel;
 struct _fpreg _st[8];
 unsigned short status;
 unsigned short magic;


 unsigned long _fxsr_env[6];
 unsigned long mxcsr;
 unsigned long reserved;
 struct _fpxreg _fxsr_st[8];
 struct _xmmreg _xmm[8];
 unsigned long padding[56];
};



struct sigcontext {
 unsigned short gs, __gsh;
 unsigned short fs, __fsh;
 unsigned short es, __esh;
 unsigned short ds, __dsh;
 unsigned long edi;
 unsigned long esi;
 unsigned long ebp;
 unsigned long esp;
 unsigned long ebx;
 unsigned long edx;
 unsigned long ecx;
 unsigned long eax;
 unsigned long trapno;
 unsigned long err;
 unsigned long eip;
 unsigned short cs, __csh;
 unsigned long eflags;
 unsigned long esp_at_signal;
 unsigned short ss, __ssh;
 struct _fpstate * fpstate;
 unsigned long oldmask;
 unsigned long cr2;
};


int restore_i387_soft(void *s387, struct _fpstate *buf);
int save_i387_soft(void *s387, struct _fpstate *buf);





struct info {
 long ___orig_eip;
 long ___ebx;
 long ___ecx;
 long ___edx;
 long ___esi;
 long ___edi;
 long ___ebp;
 long ___eax;
 long ___ds;
 long ___es;
 long ___orig_eax;
 long ___eip;
 long ___cs;
 long ___eflags;
 long ___esp;
 long ___ss;
 long ___vm86_es;
 long ___vm86_ds;
 long ___vm86_fs;
 long ___vm86_gs;
};










static inline __attribute__((always_inline)) void wrmsrl (unsigned long msr, unsigned long long val)
{
 unsigned long lo, hi;
 lo = (unsigned long) val;
 hi = val >> 32;
 __asm__ __volatile__("wrmsr" : : "c" (msr), "a" (lo), "d" (hi));
}









typedef __builtin_va_list __gnuc_va_list;

typedef __gnuc_va_list va_list;



















extern const char linux_banner[];

extern int console_printk[];






struct completion;
struct pt_regs;
struct user;

extern int cond_resched(void);






  void __might_sleep(char *file, int line);

extern struct atomic_notifier_head panic_notifier_list;
extern long (*panic_blink)(long time);
 void panic(const char * fmt, ...)
 __attribute__ ((noreturn, format (printf, 1, 2)));
extern void oops_enter(void);
extern void oops_exit(void);
extern int oops_may_print(void);
__attribute__((regparm(3))) void do_exit(long error_code)
 __attribute__((noreturn));
 void complete_and_exit(struct completion *, long)
 __attribute__((noreturn));
extern unsigned long simple_strtoul(const char *,char **,unsigned int);
extern long simple_strtol(const char *,char **,unsigned int);
extern unsigned long long simple_strtoull(const char *,char **,unsigned int);
extern long long simple_strtoll(const char *,char **,unsigned int);
extern int sprintf(char * buf, const char * fmt, ...)
 __attribute__ ((format (printf, 2, 3)));
extern int vsprintf(char *buf, const char *, va_list)
 __attribute__ ((format (printf, 2, 0)));
extern int snprintf(char * buf, size_t size, const char * fmt, ...)
 __attribute__ ((format (printf, 3, 4)));
extern int vsnprintf(char *buf, size_t size, const char *fmt, va_list args)
 __attribute__ ((format (printf, 3, 0)));
extern int scnprintf(char * buf, size_t size, const char * fmt, ...)
 __attribute__ ((format (printf, 3, 4)));
extern int vscnprintf(char *buf, size_t size, const char *fmt, va_list args)
 __attribute__ ((format (printf, 3, 0)));
extern char *kasprintf(gfp_t gfp, const char *fmt, ...)
 __attribute__ ((format (printf, 2, 3)));

extern int sscanf(const char *, const char *, ...)
 __attribute__ ((format (scanf, 2, 3)));
extern int vsscanf(const char *, const char *, va_list)
 __attribute__ ((format (scanf, 2, 0)));

extern int get_option(char **str, int *pint);
extern char *get_options(const char *str, int nints, int *ints);
extern unsigned long long memparse(char *ptr, char **retptr);

extern int core_kernel_text(unsigned long addr);
extern int __kernel_text_address(unsigned long addr);
extern int kernel_text_address(unsigned long addr);
extern int session_of_pgrp(int pgrp);

extern void dump_thread(struct pt_regs *regs, struct user *dump);


 __attribute__((regparm(0))) int vprintk(const char *fmt, va_list args)
 __attribute__ ((format (printf, 1, 0)));
 __attribute__((regparm(0))) int printk(const char * fmt, ...)
 __attribute__ ((format (printf, 1, 2)));

unsigned long int_sqrt(unsigned long);

static inline __attribute__((always_inline)) int __attribute__((pure)) long_log2(unsigned long x)
{
 int r = 0;
 for (x >>= 1; x > 0; x >>= 1)
  r++;
 return r;
}

static inline __attribute__((always_inline)) unsigned long
__attribute__((__const__)) roundup_pow_of_two(unsigned long x)
{
 return 1UL << fls_long(x - 1);
}

extern int printk_ratelimit(void);
extern int __printk_ratelimit(int ratelimit_jiffies, int ratelimit_burst);

static inline __attribute__((always_inline)) void console_silent(void)
{
 (console_printk[0]) = 0;
}

static inline __attribute__((always_inline)) void console_verbose(void)
{
 if ((console_printk[0]))
  (console_printk[0]) = 15;
}

extern void bust_spinlocks(int yes);
extern int oops_in_progress;
extern int panic_timeout;
extern int panic_on_oops;
extern int tainted;
extern const char *print_tainted(void);
extern void add_taint(unsigned);


extern enum system_states {
 SYSTEM_BOOTING,
 SYSTEM_RUNNING,
 SYSTEM_HALT,
 SYSTEM_POWER_OFF,
 SYSTEM_RESTART,
 SYSTEM_SUSPEND_DISK,
} system_state;

extern void dump_stack(void);

struct sysinfo {
 long uptime;
 unsigned long loads[3];
 unsigned long totalram;
 unsigned long freeram;
 unsigned long sharedram;
 unsigned long bufferram;
 unsigned long totalswap;
 unsigned long freeswap;
 unsigned short procs;
 unsigned short pad;
 unsigned long totalhigh;
 unsigned long freehigh;
 unsigned int mem_unit;
 char _f[20-2*sizeof(long)-sizeof(int)];
};







struct task_struct;
extern struct task_struct * __switch_to(struct task_struct *prev, struct task_struct *next) __attribute__((regparm(3)));

static inline __attribute__((always_inline)) unsigned long get_limit(unsigned long segment)
{
 unsigned long __limit;
 __asm__("lsll %1,%0"
  :"=r" (__limit):"r" (segment));
 return __limit+1;
}







struct __xchg_dummy { unsigned long a[100]; };

static inline __attribute__((always_inline)) void __set_64bit (unsigned long long * ptr,
  unsigned int low, unsigned int high)
{
 __asm__ __volatile__ (
  "\n1:\t"
  "movl (%0), %%eax\n\t"
  "movl 4(%0), %%edx\n\t"
  "lock cmpxchg8b (%0)\n\t"
  "jnz 1b"
  :
  : "D"(ptr),
   "b"(low),
   "c"(high)
  : "ax","dx","memory");
}

static inline __attribute__((always_inline)) void __set_64bit_constant (unsigned long long *ptr,
       unsigned long long value)
{
 __set_64bit(ptr,(unsigned int)(value), (unsigned int)((value)>>32ULL));
}



static inline __attribute__((always_inline)) void __set_64bit_var (unsigned long long *ptr,
    unsigned long long value)
{
 __set_64bit(ptr,*(((unsigned int*)&(value))+0), *(((unsigned int*)&(value))+1));
}

static inline __attribute__((always_inline)) unsigned long __xchg(unsigned long x, volatile void * ptr, int size)
{
 switch (size) {
  case 1:
   __asm__ __volatile__("xchgb %b0,%1"
    :"=q" (x)
    :"m" (*((struct __xchg_dummy *)(ptr))), "0" (x)
    :"memory");
   break;
  case 2:
   __asm__ __volatile__("xchgw %w0,%1"
    :"=r" (x)
    :"m" (*((struct __xchg_dummy *)(ptr))), "0" (x)
    :"memory");
   break;
  case 4:
   __asm__ __volatile__("xchgl %0,%1"
    :"=r" (x)
    :"m" (*((struct __xchg_dummy *)(ptr))), "0" (x)
    :"memory");
   break;
 }
 return x;
}

static inline __attribute__((always_inline)) unsigned long __cmpxchg(volatile void *ptr, unsigned long old,
          unsigned long new, int size)
{
 unsigned long prev;
 switch (size) {
 case 1:
  __asm__ __volatile__("" "cmpxchgb %b1,%2"
         : "=a"(prev)
         : "q"(new), "m"(*((struct __xchg_dummy *)(ptr))), "0"(old)
         : "memory");
  return prev;
 case 2:
  __asm__ __volatile__("" "cmpxchgw %w1,%2"
         : "=a"(prev)
         : "r"(new), "m"(*((struct __xchg_dummy *)(ptr))), "0"(old)
         : "memory");
  return prev;
 case 4:
  __asm__ __volatile__("" "cmpxchgl %1,%2"
         : "=a"(prev)
         : "r"(new), "m"(*((struct __xchg_dummy *)(ptr))), "0"(old)
         : "memory");
  return prev;
 }
 return old;
}

static inline __attribute__((always_inline)) unsigned long long __cmpxchg64(volatile void *ptr, unsigned long long old,
          unsigned long long new)
{
 unsigned long long prev;
 __asm__ __volatile__("" "cmpxchg8b %3"
        : "=A"(prev)
        : "b"((unsigned long)new),
          "c"((unsigned long)(new >> 32)),
          "m"(*((struct __xchg_dummy *)(ptr))),
          "0"(old)
        : "memory");
 return prev;
}





static inline __attribute__((always_inline)) unsigned long __raw_local_save_flags(void)
{
 unsigned long flags;

 __asm__ __volatile__(
  "pushfl ; popl %0"
  : "=g" (flags)
  :
 );

 return flags;
}




static inline __attribute__((always_inline)) void raw_local_irq_restore(unsigned long flags)
{
 __asm__ __volatile__(
  "pushl %0 ; popfl"
  :
  :"g" (flags)
  :"memory", "cc"
 );
}

static inline __attribute__((always_inline)) void raw_local_irq_disable(void)
{
 __asm__ __volatile__("cli" : : : "memory");
}

static inline __attribute__((always_inline)) void raw_local_irq_enable(void)
{
 __asm__ __volatile__("sti" : : : "memory");
}





static inline __attribute__((always_inline)) void raw_safe_halt(void)
{
 __asm__ __volatile__("sti; hlt" : : : "memory");
}





static inline __attribute__((always_inline)) void halt(void)
{
 __asm__ __volatile__("hlt": : :"memory");
}

static inline __attribute__((always_inline)) int raw_irqs_disabled_flags(unsigned long flags)
{
 return !(flags & (1 << 9));
}

static inline __attribute__((always_inline)) int raw_irqs_disabled(void)
{
 unsigned long flags = __raw_local_save_flags();

 return raw_irqs_disabled_flags(flags);
}




static inline __attribute__((always_inline)) unsigned long __raw_local_irq_save(void)
{
 unsigned long flags = __raw_local_save_flags();

 raw_local_irq_disable();

 return flags;
}







void disable_hlt(void);
void enable_hlt(void);

extern int es7000_plat;
void cpu_idle_wait(void);





static inline __attribute__((always_inline)) void sched_cacheflush(void)
{
 __asm__ __volatile__ ("wbinvd": : :"memory");
}

extern unsigned long arch_align_stack(unsigned long sp);
extern void free_init_pages(char *what, unsigned long begin, unsigned long end);

void default_idle(void);






























extern char *strndup_user(const char *, long);






static inline __attribute__((always_inline)) char * strcpy(char * dest,const char *src)
{
int d0, d1, d2;
__asm__ __volatile__(
 "1:\tlodsb\n\t"
 "stosb\n\t"
 "testb %%al,%%al\n\t"
 "jne 1b"
 : "=&S" (d0), "=&D" (d1), "=&a" (d2)
 :"0" (src),"1" (dest) : "memory");
return dest;
}


static inline __attribute__((always_inline)) char * strncpy(char * dest,const char *src,size_t count)
{
int d0, d1, d2, d3;
__asm__ __volatile__(
 "1:\tdecl %2\n\t"
 "js 2f\n\t"
 "lodsb\n\t"
 "stosb\n\t"
 "testb %%al,%%al\n\t"
 "jne 1b\n\t"
 "rep\n\t"
 "stosb\n"
 "2:"
 : "=&S" (d0), "=&D" (d1), "=&c" (d2), "=&a" (d3)
 :"0" (src),"1" (dest),"2" (count) : "memory");
return dest;
}


static inline __attribute__((always_inline)) char * strcat(char * dest,const char * src)
{
int d0, d1, d2, d3;
__asm__ __volatile__(
 "repne\n\t"
 "scasb\n\t"
 "decl %1\n"
 "1:\tlodsb\n\t"
 "stosb\n\t"
 "testb %%al,%%al\n\t"
 "jne 1b"
 : "=&S" (d0), "=&D" (d1), "=&a" (d2), "=&c" (d3)
 : "0" (src), "1" (dest), "2" (0), "3" (0xffffffffu):"memory");
return dest;
}


static inline __attribute__((always_inline)) char * strncat(char * dest,const char * src,size_t count)
{
int d0, d1, d2, d3;
__asm__ __volatile__(
 "repne\n\t"
 "scasb\n\t"
 "decl %1\n\t"
 "movl %8,%3\n"
 "1:\tdecl %3\n\t"
 "js 2f\n\t"
 "lodsb\n\t"
 "stosb\n\t"
 "testb %%al,%%al\n\t"
 "jne 1b\n"
 "2:\txorl %2,%2\n\t"
 "stosb"
 : "=&S" (d0), "=&D" (d1), "=&a" (d2), "=&c" (d3)
 : "0" (src),"1" (dest),"2" (0),"3" (0xffffffffu), "g" (count)
 : "memory");
return dest;
}


static inline __attribute__((always_inline)) int strcmp(const char * cs,const char * ct)
{
int d0, d1;
register int __res;
__asm__ __volatile__(
 "1:\tlodsb\n\t"
 "scasb\n\t"
 "jne 2f\n\t"
 "testb %%al,%%al\n\t"
 "jne 1b\n\t"
 "xorl %%eax,%%eax\n\t"
 "jmp 3f\n"
 "2:\tsbbl %%eax,%%eax\n\t"
 "orb $1,%%al\n"
 "3:"
 :"=a" (__res), "=&S" (d0), "=&D" (d1)
 :"1" (cs),"2" (ct)
 :"memory");
return __res;
}


static inline __attribute__((always_inline)) int strncmp(const char * cs,const char * ct,size_t count)
{
register int __res;
int d0, d1, d2;
__asm__ __volatile__(
 "1:\tdecl %3\n\t"
 "js 2f\n\t"
 "lodsb\n\t"
 "scasb\n\t"
 "jne 3f\n\t"
 "testb %%al,%%al\n\t"
 "jne 1b\n"
 "2:\txorl %%eax,%%eax\n\t"
 "jmp 4f\n"
 "3:\tsbbl %%eax,%%eax\n\t"
 "orb $1,%%al\n"
 "4:"
 :"=a" (__res), "=&S" (d0), "=&D" (d1), "=&c" (d2)
 :"1" (cs),"2" (ct),"3" (count)
 :"memory");
return __res;
}


static inline __attribute__((always_inline)) char * strchr(const char * s, int c)
{
int d0;
register char * __res;
__asm__ __volatile__(
 "movb %%al,%%ah\n"
 "1:\tlodsb\n\t"
 "cmpb %%ah,%%al\n\t"
 "je 2f\n\t"
 "testb %%al,%%al\n\t"
 "jne 1b\n\t"
 "movl $1,%1\n"
 "2:\tmovl %1,%0\n\t"
 "decl %0"
 :"=a" (__res), "=&S" (d0)
 :"1" (s),"0" (c)
 :"memory");
return __res;
}


static inline __attribute__((always_inline)) char * strrchr(const char * s, int c)
{
int d0, d1;
register char * __res;
__asm__ __volatile__(
 "movb %%al,%%ah\n"
 "1:\tlodsb\n\t"
 "cmpb %%ah,%%al\n\t"
 "jne 2f\n\t"
 "leal -1(%%esi),%0\n"
 "2:\ttestb %%al,%%al\n\t"
 "jne 1b"
 :"=g" (__res), "=&S" (d0), "=&a" (d1)
 :"0" (0),"1" (s),"2" (c)
 :"memory");
return __res;
}


static inline __attribute__((always_inline)) size_t strlen(const char * s)
{
int d0;
register int __res;
__asm__ __volatile__(
 "repne\n\t"
 "scasb\n\t"
 "notl %0\n\t"
 "decl %0"
 :"=c" (__res), "=&D" (d0)
 :"1" (s),"a" (0), "0" (0xffffffffu)
 :"memory");
return __res;
}

static inline __attribute__((always_inline)) __attribute__((always_inline)) void * __memcpy(void * to, const void * from, size_t n)
{
int d0, d1, d2;
__asm__ __volatile__(
 "rep ; movsl\n\t"
 "movl %4,%%ecx\n\t"
 "andl $3,%%ecx\n\t"

 "jz 1f\n\t"

 "rep ; movsb\n\t"
 "1:"
 : "=&c" (d0), "=&D" (d1), "=&S" (d2)
 : "0" (n/4), "g" (n), "1" ((long) to), "2" ((long) from)
 : "memory");
return (to);
}



extern void * __constant_memcpy(void * to, const void * from, size_t n);



void *memmove(void * dest,const void * src, size_t n);




static inline __attribute__((always_inline)) void * memchr(const void * cs,int c,size_t count)
{
int d0;
register void * __res;
if (!count)
 return ((void *)0);
__asm__ __volatile__(
 "repne\n\t"
 "scasb\n\t"
 "je 1f\n\t"
 "movl $1,%0\n"
 "1:\tdecl %0"
 :"=D" (__res), "=&c" (d0)
 :"a" (c),"0" (cs),"1" (count)
 :"memory");
return __res;
}

static inline __attribute__((always_inline)) void * __memset_generic(void * s, char c,size_t count)
{
int d0, d1;
__asm__ __volatile__(
 "rep\n\t"
 "stosb"
 : "=&c" (d0), "=&D" (d1)
 :"a" (c),"1" (s),"0" (count)
 :"memory");
return s;
}

static inline __attribute__((always_inline)) __attribute__((always_inline)) void * __constant_c_memset(void * s, unsigned long c, size_t count)
{
int d0, d1;
__asm__ __volatile__(
 "rep ; stosl\n\t"
 "testb $2,%b3\n\t"
 "je 1f\n\t"
 "stosw\n"
 "1:\ttestb $1,%b3\n\t"
 "je 2f\n\t"
 "stosb\n"
 "2:"
 :"=&c" (d0), "=&D" (d1)
 :"a" (c), "q" (count), "0" (count/4), "1" ((long) s)
 :"memory");
return (s);
}



static inline __attribute__((always_inline)) size_t strnlen(const char * s, size_t count)
{
int d0;
register int __res;
__asm__ __volatile__(
 "movl %2,%0\n\t"
 "jmp 2f\n"
 "1:\tcmpb $0,(%0)\n\t"
 "je 3f\n\t"
 "incl %0\n"
 "2:\tdecl %1\n\t"
 "cmpl $-1,%1\n\t"
 "jne 1b\n"
 "3:\tsubl %2,%0"
 :"=a" (__res), "=&d" (d0)
 :"c" (s),"1" (count)
 :"memory");
return __res;
}




extern char *strstr(const char *cs, const char *ct);





static inline __attribute__((always_inline)) __attribute__((always_inline)) void * __constant_c_and_count_memset(void * s, unsigned long pattern, size_t count)
{
 switch (count) {
  case 0:
   return s;
  case 1:
   *(unsigned char *)s = pattern;
   return s;
  case 2:
   *(unsigned short *)s = pattern;
   return s;
  case 3:
   *(unsigned short *)s = pattern;
   *(2+(unsigned char *)s) = pattern;
   return s;
  case 4:
   *(unsigned long *)s = pattern;
   return s;
 }







{
 int d0, d1;
 switch (count % 4) {
  case 0: __asm__ __volatile__( "rep ; stosl" "" : "=&c" (d0), "=&D" (d1) : "a" (pattern),"0" (count/4),"1" ((long) s) : "memory"); return s;
  case 1: __asm__ __volatile__( "rep ; stosl" "\n\tstosb" : "=&c" (d0), "=&D" (d1) : "a" (pattern),"0" (count/4),"1" ((long) s) : "memory"); return s;
  case 2: __asm__ __volatile__( "rep ; stosl" "\n\tstosw" : "=&c" (d0), "=&D" (d1) : "a" (pattern),"0" (count/4),"1" ((long) s) : "memory"); return s;
  default: __asm__ __volatile__( "rep ; stosl" "\n\tstosw\n\tstosb" : "=&c" (d0), "=&D" (d1) : "a" (pattern),"0" (count/4),"1" ((long) s) : "memory"); return s;
 }
}


}

static inline __attribute__((always_inline)) void * memscan(void * addr, int c, size_t size)
{
 if (!size)
  return addr;
 __asm__("repnz; scasb\n\t"
  "jnz 1f\n\t"
  "dec %%edi\n"
  "1:"
  : "=D" (addr), "=c" (size)
  : "0" (addr), "1" (size), "a" (c)
  : "memory");
 return addr;
}


size_t strlcpy(char *, const char *, size_t);

extern size_t strlcat(char *, const char *, __kernel_size_t);

extern int strnicmp(const char *, const char *, __kernel_size_t);





extern char * strnchr(const char *, size_t, int);




extern char * strstrip(char *);

extern char * strpbrk(const char *,const char *);


extern char * strsep(char **,const char *);


extern __kernel_size_t strspn(const char *,const char *);


extern __kernel_size_t strcspn(const char *,const char *);

extern int __builtin_memcmp(const void *,const void *,__kernel_size_t);





extern char *kstrdup(const char *s, gfp_t gfp);


extern int __bitmap_empty(const unsigned long *bitmap, int bits);
extern int __bitmap_full(const unsigned long *bitmap, int bits);
extern int __bitmap_equal(const unsigned long *bitmap1,
                 const unsigned long *bitmap2, int bits);
extern void __bitmap_complement(unsigned long *dst, const unsigned long *src,
   int bits);
extern void __bitmap_shift_right(unsigned long *dst,
                        const unsigned long *src, int shift, int bits);
extern void __bitmap_shift_left(unsigned long *dst,
                        const unsigned long *src, int shift, int bits);
extern void __bitmap_and(unsigned long *dst, const unsigned long *bitmap1,
   const unsigned long *bitmap2, int bits);
extern void __bitmap_or(unsigned long *dst, const unsigned long *bitmap1,
   const unsigned long *bitmap2, int bits);
extern void __bitmap_xor(unsigned long *dst, const unsigned long *bitmap1,
   const unsigned long *bitmap2, int bits);
extern void __bitmap_andnot(unsigned long *dst, const unsigned long *bitmap1,
   const unsigned long *bitmap2, int bits);
extern int __bitmap_intersects(const unsigned long *bitmap1,
   const unsigned long *bitmap2, int bits);
extern int __bitmap_subset(const unsigned long *bitmap1,
   const unsigned long *bitmap2, int bits);
extern int __bitmap_weight(const unsigned long *bitmap, int bits);

extern int bitmap_scnprintf(char *buf, unsigned int len,
   const unsigned long *src, int nbits);
extern int bitmap_parse(const char *ubuf, unsigned int ulen,
   unsigned long *dst, int nbits);
extern int bitmap_scnlistprintf(char *buf, unsigned int len,
   const unsigned long *src, int nbits);
extern int bitmap_parselist(const char *buf, unsigned long *maskp,
   int nmaskbits);
extern void bitmap_remap(unsigned long *dst, const unsigned long *src,
  const unsigned long *old, const unsigned long *new, int bits);
extern int bitmap_bitremap(int oldbit,
  const unsigned long *old, const unsigned long *new, int bits);
extern int bitmap_find_free_region(unsigned long *bitmap, int bits, int order);
extern void bitmap_release_region(unsigned long *bitmap, int pos, int order);
extern int bitmap_allocate_region(unsigned long *bitmap, int pos, int order);







static inline __attribute__((always_inline)) void bitmap_zero(unsigned long *dst, int nbits)
{
 if (nbits <= 32)
  *dst = 0UL;
 else {
  int len = (((nbits)+32 -1)/32) * sizeof(unsigned long);
  (__builtin_constant_p(0) ? (__builtin_constant_p((len)) ? __constant_c_and_count_memset(((dst)),((0x01010101UL*(unsigned char)(0))),((len))) : __constant_c_memset(((dst)),((0x01010101UL*(unsigned char)(0))),((len)))) : (__builtin_constant_p((len)) ? __memset_generic((((dst))),(((0))),(((len)))) : __memset_generic(((dst)),((0)),((len)))));
 }
}

static inline __attribute__((always_inline)) void bitmap_fill(unsigned long *dst, int nbits)
{
 size_t nlongs = (((nbits)+32 -1)/32);
 if (nlongs > 1) {
  int len = (nlongs - 1) * sizeof(unsigned long);
  (__builtin_constant_p(0xff) ? (__builtin_constant_p((len)) ? __constant_c_and_count_memset(((dst)),((0x01010101UL*(unsigned char)(0xff))),((len))) : __constant_c_memset(((dst)),((0x01010101UL*(unsigned char)(0xff))),((len)))) : (__builtin_constant_p((len)) ? __memset_generic((((dst))),(((0xff))),(((len)))) : __memset_generic(((dst)),((0xff)),((len)))));
 }
 dst[nlongs - 1] = ( ((nbits) % 32) ? (1UL<<((nbits) % 32))-1 : ~0UL );
}

static inline __attribute__((always_inline)) void bitmap_copy(unsigned long *dst, const unsigned long *src,
   int nbits)
{
 if (nbits <= 32)
  *dst = *src;
 else {
  int len = (((nbits)+32 -1)/32) * sizeof(unsigned long);
  (__builtin_constant_p(len) ? __constant_memcpy((dst),(src),(len)) : __memcpy((dst),(src),(len)));
 }
}

static inline __attribute__((always_inline)) void bitmap_and(unsigned long *dst, const unsigned long *src1,
   const unsigned long *src2, int nbits)
{
 if (nbits <= 32)
  *dst = *src1 & *src2;
 else
  __bitmap_and(dst, src1, src2, nbits);
}

static inline __attribute__((always_inline)) void bitmap_or(unsigned long *dst, const unsigned long *src1,
   const unsigned long *src2, int nbits)
{
 if (nbits <= 32)
  *dst = *src1 | *src2;
 else
  __bitmap_or(dst, src1, src2, nbits);
}

static inline __attribute__((always_inline)) void bitmap_xor(unsigned long *dst, const unsigned long *src1,
   const unsigned long *src2, int nbits)
{
 if (nbits <= 32)
  *dst = *src1 ^ *src2;
 else
  __bitmap_xor(dst, src1, src2, nbits);
}

static inline __attribute__((always_inline)) void bitmap_andnot(unsigned long *dst, const unsigned long *src1,
   const unsigned long *src2, int nbits)
{
 if (nbits <= 32)
   *dst = *src1 & ~(*src2);
 else
   __bitmap_andnot(dst, src1, src2, nbits);
}

static inline __attribute__((always_inline)) void bitmap_complement(unsigned long *dst, const unsigned long *src,
   int nbits)
{
 if (nbits <= 32)
  *dst = ~(*src) & ( ((nbits) % 32) ? (1UL<<((nbits) % 32))-1 : ~0UL );
 else
  __bitmap_complement(dst, src, nbits);
}

static inline __attribute__((always_inline)) int bitmap_equal(const unsigned long *src1,
   const unsigned long *src2, int nbits)
{
 if (nbits <= 32)
  return ! ((*src1 ^ *src2) & ( ((nbits) % 32) ? (1UL<<((nbits) % 32))-1 : ~0UL ));
 else
  return __bitmap_equal(src1, src2, nbits);
}

static inline __attribute__((always_inline)) int bitmap_intersects(const unsigned long *src1,
   const unsigned long *src2, int nbits)
{
 if (nbits <= 32)
  return ((*src1 & *src2) & ( ((nbits) % 32) ? (1UL<<((nbits) % 32))-1 : ~0UL )) != 0;
 else
  return __bitmap_intersects(src1, src2, nbits);
}

static inline __attribute__((always_inline)) int bitmap_subset(const unsigned long *src1,
   const unsigned long *src2, int nbits)
{
 if (nbits <= 32)
  return ! ((*src1 & ~(*src2)) & ( ((nbits) % 32) ? (1UL<<((nbits) % 32))-1 : ~0UL ));
 else
  return __bitmap_subset(src1, src2, nbits);
}

static inline __attribute__((always_inline)) int bitmap_empty(const unsigned long *src, int nbits)
{
 if (nbits <= 32)
  return ! (*src & ( ((nbits) % 32) ? (1UL<<((nbits) % 32))-1 : ~0UL ));
 else
  return __bitmap_empty(src, nbits);
}

static inline __attribute__((always_inline)) int bitmap_full(const unsigned long *src, int nbits)
{
 if (nbits <= 32)
  return ! (~(*src) & ( ((nbits) % 32) ? (1UL<<((nbits) % 32))-1 : ~0UL ));
 else
  return __bitmap_full(src, nbits);
}

static inline __attribute__((always_inline)) int bitmap_weight(const unsigned long *src, int nbits)
{
 if (nbits <= 32)
  return hweight_long(*src & ( ((nbits) % 32) ? (1UL<<((nbits) % 32))-1 : ~0UL ));
 return __bitmap_weight(src, nbits);
}

static inline __attribute__((always_inline)) void bitmap_shift_right(unsigned long *dst,
   const unsigned long *src, int n, int nbits)
{
 if (nbits <= 32)
  *dst = *src >> n;
 else
  __bitmap_shift_right(dst, src, n, nbits);
}

static inline __attribute__((always_inline)) void bitmap_shift_left(unsigned long *dst,
   const unsigned long *src, int n, int nbits)
{
 if (nbits <= 32)
  *dst = (*src << n) & ( ((nbits) % 32) ? (1UL<<((nbits) % 32))-1 : ~0UL );
 else
  __bitmap_shift_left(dst, src, n, nbits);
}


typedef struct { unsigned long bits[(((1)+32 -1)/32)]; } cpumask_t;
extern cpumask_t _unused_cpumask_arg_;


static inline __attribute__((always_inline)) void __cpu_set(int cpu, volatile cpumask_t *dstp)
{
 set_bit(cpu, dstp->bits);
}


static inline __attribute__((always_inline)) void __cpu_clear(int cpu, volatile cpumask_t *dstp)
{
 clear_bit(cpu, dstp->bits);
}


static inline __attribute__((always_inline)) void __cpus_setall(cpumask_t *dstp, int nbits)
{
 bitmap_fill(dstp->bits, nbits);
}


/* static inline __attribute__((always_inline)) void __cpus_clear(cpumask_t *dstp, int nbits) */
/* { */
/*  bitmap_zero(dstp->bits, nbits); */
/* } */





/* static inline __attribute__((always_inline)) int __cpu_test_and_set(int cpu, cpumask_t *addr) */
/* { */
/*  return test_and_set_bit(cpu, addr->bits); */
/* } */


/* static inline __attribute__((always_inline)) void __cpus_and(cpumask_t *dstp, const cpumask_t *src1p, */
/*      const cpumask_t *src2p, int nbits) */
/* { */
/*  bitmap_and(dstp->bits, src1p->bits, src2p->bits, nbits); */
/* } */


/* static inline __attribute__((always_inline)) void __cpus_or(cpumask_t *dstp, const cpumask_t *src1p, */
/*      const cpumask_t *src2p, int nbits) */
/* { */
/*  bitmap_or(dstp->bits, src1p->bits, src2p->bits, nbits); */
/* } */


/* static inline __attribute__((always_inline)) void __cpus_xor(cpumask_t *dstp, const cpumask_t *src1p, */
/*      const cpumask_t *src2p, int nbits) */
/* { */
/*  bitmap_xor(dstp->bits, src1p->bits, src2p->bits, nbits); */
/* } */



/* static inline __attribute__((always_inline)) void __cpus_andnot(cpumask_t *dstp, const cpumask_t *src1p, */
/*      const cpumask_t *src2p, int nbits) */
/* { */
/*  bitmap_andnot(dstp->bits, src1p->bits, src2p->bits, nbits); */
/* } */


/* static inline __attribute__((always_inline)) void __cpus_complement(cpumask_t *dstp, */
/*      const cpumask_t *srcp, int nbits) */
/* { */
/*  bitmap_complement(dstp->bits, srcp->bits, nbits); */
/* } */


/* static inline __attribute__((always_inline)) int __cpus_equal(const cpumask_t *src1p, */
/*      const cpumask_t *src2p, int nbits) */
/* { */
/*  return bitmap_equal(src1p->bits, src2p->bits, nbits); */
/* } */


/* static inline __attribute__((always_inline)) int __cpus_intersects(const cpumask_t *src1p, */
/*      const cpumask_t *src2p, int nbits) */
/* { */
/*  return bitmap_intersects(src1p->bits, src2p->bits, nbits); */
/* } */


/* static inline __attribute__((always_inline)) int __cpus_subset(const cpumask_t *src1p, */
/*      const cpumask_t *src2p, int nbits) */
/* { */
/*  return bitmap_subset(src1p->bits, src2p->bits, nbits); */
/* } */


/* static inline __attribute__((always_inline)) int __cpus_empty(const cpumask_t *srcp, int nbits) */
/* { */
/*  return bitmap_empty(srcp->bits, nbits); */
/* } */


/* static inline __attribute__((always_inline)) int __cpus_full(const cpumask_t *srcp, int nbits) */
/* { */
/*  return bitmap_full(srcp->bits, nbits); */
/* } */


/* static inline __attribute__((always_inline)) int __cpus_weight(const cpumask_t *srcp, int nbits) */
/* { */
/*  return bitmap_weight(srcp->bits, nbits); */
/* } */



/* static inline __attribute__((always_inline)) void __cpus_shift_right(cpumask_t *dstp, */
/*      const cpumask_t *srcp, int n, int nbits) */
/* { */
/*  bitmap_shift_right(dstp->bits, srcp->bits, n, nbits); */
/* } */



/* static inline __attribute__((always_inline)) void __cpus_shift_left(cpumask_t *dstp, */
/*      const cpumask_t *srcp, int n, int nbits) */
/* { */
/*  bitmap_shift_left(dstp->bits, srcp->bits, n, nbits); */
/* } */

/* static inline __attribute__((always_inline)) int __cpumask_scnprintf(char *buf, int len, */
/*      const cpumask_t *srcp, int nbits) */
/* { */
/*  return bitmap_scnprintf(buf, len, srcp->bits, nbits); */
/* } */



/* static inline __attribute__((always_inline)) int __cpumask_parse(const char *buf, int len, */
/*      cpumask_t *dstp, int nbits) */
/* { */
/*  return bitmap_parse(buf, len, dstp->bits, nbits); */
/* } */



/* static inline __attribute__((always_inline)) int __cpulist_scnprintf(char *buf, int len, */
/*      const cpumask_t *srcp, int nbits) */
/* { */
/*  return bitmap_scnlistprintf(buf, len, srcp->bits, nbits); */
/* } */


/* static inline __attribute__((always_inline)) int __cpulist_parse(const char *buf, cpumask_t *dstp, int nbits) */
/* { */
/*  return bitmap_parselist(buf, dstp->bits, nbits); */
/* } */



/* static inline __attribute__((always_inline)) int __cpu_remap(int oldbit, */
/*   const cpumask_t *oldp, const cpumask_t *newp, int nbits) */
/* { */
/*  return bitmap_bitremap(oldbit, oldp->bits, newp->bits, nbits); */
/* } */



/* static inline __attribute__((always_inline)) void __cpus_remap(cpumask_t *dstp, const cpumask_t *srcp, */
/*   const cpumask_t *oldp, const cpumask_t *newp, int nbits) */
/* { */
/*  bitmap_remap(dstp->bits, srcp->bits, oldp->bits, newp->bits, nbits); */
/* } */

/* extern cpumask_t cpu_possible_map; */
/* extern cpumask_t cpu_online_map; */
/* extern cpumask_t cpu_present_map; */



/* extern int tsc_disable; */

/* struct desc_struct { */
/*  unsigned long a,b; */
/* }; */

/* struct cpuinfo_x86 { */
/*  __u8 x86; */
/*  __u8 x86_vendor; */
/*  __u8 x86_model; */
/*  __u8 x86_mask; */
/*  char wp_works_ok; */
/*  char hlt_works_ok; */
/*  char hard_math; */
/*  char rfu; */
/*         int cpuid_level; */
/*  unsigned long x86_capability[7]; */
/*  char x86_vendor_id[16]; */
/*  char x86_model_id[64]; */
/*  int x86_cache_size; */

/*  int x86_cache_alignment; */
/*  char fdiv_bug; */
/*  char f00f_bug; */
/*  char coma_bug; */
/*  char pad0; */
/*  int x86_power; */
/*  unsigned long loops_per_jiffy; */



/*  unsigned char x86_max_cores; */
/*  unsigned char apicid; */





/* } __attribute__((__aligned__((1 << (7))))); */

/* extern struct cpuinfo_x86 boot_cpu_data; */
/* extern struct cpuinfo_x86 new_cpu_data; */
/* extern struct tss_struct doublefault_tss; */
/* extern __typeof__(struct tss_struct) per_cpu__init_tss; */

/* extern int cpu_llc_id[1]; */
/* extern char ignore_fpu_irq; */

/* extern void identify_cpu(struct cpuinfo_x86 *); */
/* extern void print_cpu_info(struct cpuinfo_x86 *); */
/* extern unsigned int init_intel_cacheinfo(struct cpuinfo_x86 *c); */
/* extern unsigned short num_cache_leaves; */




/* static inline __attribute__((always_inline)) void detect_ht(struct cpuinfo_x86 *c) {} */

/* static inline __attribute__((always_inline)) void cpuid(unsigned int op, unsigned int *eax, unsigned int *ebx, unsigned int *ecx, unsigned int *edx) */
/* { */
/*  __asm__("cpuid" */
/*   : "=a" (*eax), */
/*     "=b" (*ebx), */
/*     "=c" (*ecx), */
/*     "=d" (*edx) */
/*   : "0" (op), "c"(0)); */
/* } */


/* static inline __attribute__((always_inline)) void cpuid_count(int op, int count, int *eax, int *ebx, int *ecx, */
/*          int *edx) */
/* { */
/*  __asm__("cpuid" */
/*   : "=a" (*eax), */
/*     "=b" (*ebx), */
/*     "=c" (*ecx), */
/*     "=d" (*edx) */
/*   : "0" (op), "c" (count)); */
/* } */




/* static inline __attribute__((always_inline)) unsigned int cpuid_eax(unsigned int op) */
/* { */
/*  unsigned int eax; */

/*  __asm__("cpuid" */
/*   : "=a" (eax) */
/*   : "0" (op) */
/*   : "bx", "cx", "dx"); */
/*  return eax; */
/* } */
/* static inline __attribute__((always_inline)) unsigned int cpuid_ebx(unsigned int op) */
/* { */
/*  unsigned int eax, ebx; */

/*  __asm__("cpuid" */
/*   : "=a" (eax), "=b" (ebx) */
/*   : "0" (op) */
/*   : "cx", "dx" ); */
/*  return ebx; */
/* } */
/* static inline __attribute__((always_inline)) unsigned int cpuid_ecx(unsigned int op) */
/* { */
/*  unsigned int eax, ecx; */

/*  __asm__("cpuid" */
/*   : "=a" (eax), "=c" (ecx) */
/*   : "0" (op) */
/*   : "bx", "dx" ); */
/*  return ecx; */
/* } */
/* static inline __attribute__((always_inline)) unsigned int cpuid_edx(unsigned int op) */
/* { */
/*  unsigned int eax, edx; */

/*  __asm__("cpuid" */
/*   : "=a" (eax), "=d" (edx) */
/*   : "0" (op) */
/*   : "bx", "cx"); */
/*  return edx; */
/* } */

/* extern unsigned long mmu_cr4_features; */

/* static inline __attribute__((always_inline)) void set_in_cr4 (unsigned long mask) */
/* { */
/*  unsigned cr4; */
/*  mmu_cr4_features |= mask; */
/*  cr4 = ({ unsigned int __dummy; __asm__( "movl %%cr4,%0\n\t" :"=r" (__dummy)); __dummy; }); */
/*  cr4 |= mask; */
/*  __asm__ __volatile__("movl %0,%%cr4": :"r" (cr4)); */
/* } */

/* static inline __attribute__((always_inline)) void clear_in_cr4 (unsigned long mask) */
/* { */
/*  unsigned cr4; */
/*  mmu_cr4_features &= ~mask; */
/*  cr4 = ({ unsigned int __dummy; __asm__( "movl %%cr4,%0\n\t" :"=r" (__dummy)); __dummy; }); */
/*  cr4 &= ~mask; */
/*  __asm__ __volatile__("movl %0,%%cr4": :"r" (cr4)); */
/* } */

/* static inline __attribute__((always_inline)) void sync_core(void) */
/* { */
/*  int tmp; */
/*  asm volatile("cpuid" : "=a" (tmp) : "0" (1) : "ebx","ecx","edx","memory"); */
/* } */

/* static inline __attribute__((always_inline)) void __monitor(const void *eax, unsigned long ecx, */
/*   unsigned long edx) */
/* { */

/*  asm volatile( */
/*   ".byte 0x0f,0x01,0xc8;" */
/*   : :"a" (eax), "c" (ecx), "d"(edx)); */
/* } */

/* static inline __attribute__((always_inline)) void __mwait(unsigned long eax, unsigned long ecx) */
/* { */

/*  asm volatile( */
/*   ".byte 0x0f,0x01,0xc9;" */
/*   : :"a" (eax), "c" (ecx)); */
/* } */



/* extern unsigned int machine_id; */
/* extern unsigned int machine_submodel_id; */
/* extern unsigned int BIOS_revision; */
/* extern unsigned int mca_pentium_flag; */


/* extern int bootloader_type; */

/* struct i387_fsave_struct { */
/*  long cwd; */
/*  long swd; */
/*  long twd; */
/*  long fip; */
/*  long fcs; */
/*  long foo; */
/*  long fos; */
/*  long st_space[20]; */
/*  long status; */
/* }; */

/* struct i387_fxsave_struct { */
/*  unsigned short cwd; */
/*  unsigned short swd; */
/*  unsigned short twd; */
/*  unsigned short fop; */
/*  long fip; */
/*  long fcs; */
/*  long foo; */
/*  long fos; */
/*  long mxcsr; */
/*  long mxcsr_mask; */
/*  long st_space[32]; */
/*  long xmm_space[32]; */
/*  long padding[56]; */
/* } __attribute__ ((aligned (16))); */

/* struct i387_soft_struct { */
/*  long cwd; */
/*  long swd; */
/*  long twd; */
/*  long fip; */
/*  long fcs; */
/*  long foo; */
/*  long fos; */
/*  long st_space[20]; */
/*  unsigned char ftop, changed, lookahead, no_update, rm, alimit; */
/*  struct info *info; */
/*  unsigned long entry_eip; */
/* }; */

/* union i387_union { */
/*  struct i387_fsave_struct fsave; */
/*  struct i387_fxsave_struct fxsave; */
/*  struct i387_soft_struct soft; */
/* }; */

/* typedef struct { */
/*  unsigned long seg; */
/* } mm_segment_t; */

/* struct thread_struct; */

/* struct tss_struct { */
/*  unsigned short back_link,__blh; */
/*  unsigned long esp0; */
/*  unsigned short ss0,__ss0h; */
/*  unsigned long esp1; */
/*  unsigned short ss1,__ss1h; */
/*  unsigned long esp2; */
/*  unsigned short ss2,__ss2h; */
/*  unsigned long __cr3; */
/*  unsigned long eip; */
/*  unsigned long eflags; */
/*  unsigned long eax,ecx,edx,ebx; */
/*  unsigned long esp; */
/*  unsigned long ebp; */
/*  unsigned long esi; */
/*  unsigned long edi; */
/*  unsigned short es, __esh; */
/*  unsigned short cs, __csh; */
/*  unsigned short ss, __ssh; */
/*  unsigned short ds, __dsh; */
/*  unsigned short fs, __fsh; */
/*  unsigned short gs, __gsh; */
/*  unsigned short ldt, __ldth; */
/*  unsigned short trace, io_bitmap_base; */






/*  unsigned long io_bitmap[((65536/8)/sizeof(long)) + 1]; */



/*  unsigned long io_bitmap_max; */
/*  struct thread_struct *io_bitmap_owner; */



/*  unsigned long __cacheline_filler[35]; */



/*  unsigned long stack[64]; */
/* } __attribute__((packed)); */



/* struct thread_struct { */

/*  struct desc_struct tls_array[3]; */
/*  unsigned long esp0; */
/*  unsigned long sysenter_cs; */
/*  unsigned long eip; */
/*  unsigned long esp; */
/*  unsigned long fs; */
/*  unsigned long gs; */

/*  unsigned long debugreg[8]; */

/*  unsigned long cr2, trap_no, error_code; */

/*  union i387_union i387; */

/*  struct vm86_struct * vm86_info; */
/*  unsigned long screen_bitmap; */
/*  unsigned long v86flags, v86mask, saved_esp0; */
/*  unsigned int saved_fs, saved_gs; */

/*  unsigned long *io_bitmap_ptr; */
/*   unsigned long iopl; */

/*  unsigned long io_bitmap_max; */
/* }; */

/* static inline __attribute__((always_inline)) void load_esp0(struct tss_struct *tss, struct thread_struct *thread) */
/* { */
/*  tss->esp0 = thread->esp0; */

/*  if (__builtin_expect(!!(tss->ss1 != thread->sysenter_cs), 0)) { */
/*   tss->ss1 = thread->sysenter_cs; */
/*   __asm__ __volatile__("wrmsr" : : "c" (0x174), "a" (thread->sysenter_cs), "d" (0)); */
/*  } */
/* } */

/* static inline __attribute__((always_inline)) void set_iopl_mask(unsigned mask) */
/* { */
/*  unsigned int reg; */
/*  __asm__ __volatile__ ("pushfl;" */
/*          "popl %0;" */
/*          "andl %1, %0;" */
/*          "orl %2, %0;" */
/*          "pushl %0;" */
/*          "popfl" */
/*     : "=&r" (reg) */
/*     : "i" (~0x00003000), "r" (mask)); */
/* } */


/* struct task_struct; */
/* struct mm_struct; */


/* extern void release_thread(struct task_struct *); */


/* extern void prepare_to_copy(struct task_struct *tsk); */




/* extern int kernel_thread(int (*fn)(void *), void * arg, unsigned long flags); */

/* extern unsigned long thread_saved_pc(struct task_struct *tsk); */
/* void show_trace(struct task_struct *task, struct pt_regs *regs, unsigned long *stack); */

/* unsigned long get_wchan(struct task_struct *p); */

/* struct microcode_header { */
/*  unsigned int hdrver; */
/*  unsigned int rev; */
/*  unsigned int date; */
/*  unsigned int sig; */
/*  unsigned int cksum; */
/*  unsigned int ldrver; */
/*  unsigned int pf; */
/*  unsigned int datasize; */
/*  unsigned int totalsize; */
/*  unsigned int reserved[3]; */
/* }; */

/* struct microcode { */
/*  struct microcode_header hdr; */
/*  unsigned int bits[0]; */
/* }; */

/* typedef struct microcode microcode_t; */
/* typedef struct microcode_header microcode_header_t; */


/* struct extended_signature { */
/*  unsigned int sig; */
/*  unsigned int pf; */
/*  unsigned int cksum; */
/* }; */

/* struct extended_sigtable { */
/*  unsigned int count; */
/*  unsigned int cksum; */
/*  unsigned int reserved[3]; */
/*  struct extended_signature sigs[0]; */
/* }; */


/* static inline __attribute__((always_inline)) void rep_nop(void) */
/* { */
/*  __asm__ __volatile__("rep;nop": : :"memory"); */
/* } */

/* static inline __attribute__((always_inline)) void prefetch(const void *x) */
/* { */
/*  asm volatile ("661:\n\t" ".byte 0x8d,0x74,0x26,0x00\n" "\n662:\n" ".section .altinstructions,\"a\"\n" "  .align 4\n" "  .long 661b\n" "  .long 663f\n" "  .byte %c0\n" "  .byte 662b-661b\n" "  .byte 664f-663f\n" ".previous\n" ".section .altinstr_replacement,\"ax\"\n" "663:\n\t" "prefetchnta (%1)" "\n664:\n" ".previous" :: "i" ((0*32+25)), "r" (x)); */



/* } */







/* static inline __attribute__((always_inline)) void prefetchw(const void *x) */
/* { */
/*  asm volatile ("661:\n\t" ".byte 0x8d,0x74,0x26,0x00\n" "\n662:\n" ".section .altinstructions,\"a\"\n" "  .align 4\n" "  .long 661b\n" "  .long 663f\n" "  .byte %c0\n" "  .byte 662b-661b\n" "  .byte 664f-663f\n" ".previous\n" ".section .altinstr_replacement,\"ax\"\n" "663:\n\t" "prefetchw (%1)" "\n664:\n" ".previous" :: "i" ((1*32+31)), "r" (x)); */



/* } */


/* extern void select_idle_routine(const struct cpuinfo_x86 *c); */



/* extern unsigned long boot_option_idle_override; */
/* extern void enable_sep_cpu(void); */
/* extern int sysenter_setup(void); */


/* struct thread_info { */
/*  struct task_struct *task; */
/*  struct exec_domain *exec_domain; */
/*  unsigned long flags; */
/*  unsigned long status; */
/*  __u32 cpu; */
/*  int preempt_count; */


/*  mm_segment_t addr_limit; */



/*  void *sysenter_return; */
/*  struct restart_block restart_block; */

/*  unsigned long previous_esp; */


/*  __u8 supervisor_stack[0]; */
/* }; */

/* register unsigned long current_stack_pointer asm("esp") __attribute__((__used__)); */

/* extern struct thread_info *current_thread_info(void); */



/* static inline __attribute__((always_inline)) void set_ti_thread_flag(struct thread_info *ti, int flag) */
/* { */
/*  set_bit(flag,&ti->flags); */
/* } */

/* static inline __attribute__((always_inline)) void clear_ti_thread_flag(struct thread_info *ti, int flag) */
/* { */
/*  clear_bit(flag,&ti->flags); */
/* } */

/* static inline __attribute__((always_inline)) int test_and_set_ti_thread_flag(struct thread_info *ti, int flag) */
/* { */
/*  return test_and_set_bit(flag,&ti->flags); */
/* } */

/* static inline __attribute__((always_inline)) int test_and_clear_ti_thread_flag(struct thread_info *ti, int flag) */
/* { */
/*  return test_and_clear_bit(flag,&ti->flags); */
/* } */

/* static inline __attribute__((always_inline)) int test_ti_thread_flag(struct thread_info *ti, int flag) */
/* { */
/*  return (__builtin_constant_p(flag) ? constant_test_bit((flag),(&ti->flags)) : variable_test_bit((flag),(&ti->flags))); */
/* } */
























/* static inline __attribute__((always_inline)) void prefetch_range(void *addr, size_t len) */
/* { */

/*  char *cp; */
/*  char *end = addr + len; */

/*  for (cp = addr; cp < end; cp += (4*(1 << (7)))) */
/*   prefetch(cp); */

/* } */


/* struct list_head { */
/*  struct list_head *next, *prev; */
/* }; */






/* static inline __attribute__((always_inline)) void INIT_LIST_HEAD(struct list_head *list) */
/* { */
/*  list->next = list; */
/*  list->prev = list; */
/* } */







/* static inline __attribute__((always_inline)) void __list_add(struct list_head *new, */
/*          struct list_head *prev, */
/*          struct list_head *next) */
/* { */
/*  next->prev = new; */
/*  new->next = next; */
/*  new->prev = prev; */
/*  prev->next = new; */
/* } */

/* static inline __attribute__((always_inline)) void list_add(struct list_head *new, struct list_head *head) */
/* { */
/*  __list_add(new, head, head->next); */
/* } */

/* static inline __attribute__((always_inline)) void list_add_tail(struct list_head *new, struct list_head *head) */
/* { */
/*  __list_add(new, head->prev, head); */
/* } */







/* static inline __attribute__((always_inline)) void __list_add_rcu(struct list_head * new, */
/*   struct list_head * prev, struct list_head * next) */
/* { */
/*  new->next = next; */
/*  new->prev = prev; */
/*  __asm__ __volatile__("": : :"memory"); */
/*  next->prev = new; */
/*  prev->next = new; */
/* } */

/* static inline __attribute__((always_inline)) void list_add_rcu(struct list_head *new, struct list_head *head) */
/* { */
/*  __list_add_rcu(new, head, head->next); */
/* } */

/* static inline __attribute__((always_inline)) void list_add_tail_rcu(struct list_head *new, */
/*      struct list_head *head) */
/* { */
/*  __list_add_rcu(new, head->prev, head); */
/* } */

/* static inline __attribute__((always_inline)) void __list_del(struct list_head * prev, struct list_head * next) */
/* { */
/*  next->prev = prev; */
/*  prev->next = next; */
/* } */







/* static inline __attribute__((always_inline)) void list_del(struct list_head *entry) */
/* { */
/*  __list_del(entry->prev, entry->next); */
/*  entry->next = ((void *) 0x00100100); */
/*  entry->prev = ((void *) 0x00200200); */
/* } */

/* static inline __attribute__((always_inline)) void list_del_rcu(struct list_head *entry) */
/* { */
/*  __list_del(entry->prev, entry->next); */
/*  entry->prev = ((void *) 0x00200200); */
/* } */







/* static inline __attribute__((always_inline)) void list_replace(struct list_head *old, */
/*     struct list_head *new) */
/* { */
/*  new->next = old->next; */
/*  new->next->prev = new; */
/*  new->prev = old->prev; */
/*  new->prev->next = new; */
/* } */

/* static inline __attribute__((always_inline)) void list_replace_init(struct list_head *old, */
/*      struct list_head *new) */
/* { */
/*  list_replace(old, new); */
/*  INIT_LIST_HEAD(old); */
/* } */

/* static inline __attribute__((always_inline)) void list_replace_rcu(struct list_head *old, */
/*     struct list_head *new) */
/* { */
/*  new->next = old->next; */
/*  new->prev = old->prev; */
/*  __asm__ __volatile__("": : :"memory"); */
/*  new->next->prev = new; */
/*  new->prev->next = new; */
/*  old->prev = ((void *) 0x00200200); */
/* } */





/* static inline __attribute__((always_inline)) void list_del_init(struct list_head *entry) */
/* { */
/*  __list_del(entry->prev, entry->next); */
/*  INIT_LIST_HEAD(entry); */
/* } */






/* static inline __attribute__((always_inline)) void list_move(struct list_head *list, struct list_head *head) */
/* { */
/*         __list_del(list->prev, list->next); */
/*         list_add(list, head); */
/* } */






/* static inline __attribute__((always_inline)) void list_move_tail(struct list_head *list, */
/*       struct list_head *head) */
/* { */
/*         __list_del(list->prev, list->next); */
/*         list_add_tail(list, head); */
/* } */






/* static inline __attribute__((always_inline)) int list_is_last(const struct list_head *list, */
/*     const struct list_head *head) */
/* { */
/*  return list->next == head; */
/* } */





/* static inline __attribute__((always_inline)) int list_empty(const struct list_head *head) */
/* { */
/*  return head->next == head; */
/* } */

/* static inline __attribute__((always_inline)) int list_empty_careful(const struct list_head *head) */
/* { */
/*  struct list_head *next = head->next; */
/*  return (next == head) && (next == head->prev); */
/* } */

/* static inline __attribute__((always_inline)) void __list_splice(struct list_head *list, */
/*      struct list_head *head) */
/* { */
/*  struct list_head *first = list->next; */
/*  struct list_head *last = list->prev; */
/*  struct list_head *at = head->next; */

/*  first->prev = head; */
/*  head->next = first; */

/*  last->next = at; */
/*  at->prev = last; */
/* } */






/* static inline __attribute__((always_inline)) void list_splice(struct list_head *list, struct list_head *head) */
/* { */
/*  if (!list_empty(list)) */
/*   __list_splice(list, head); */
/* } */

/* static inline __attribute__((always_inline)) void list_splice_init(struct list_head *list, */
/*         struct list_head *head) */
/* { */
/*  if (!list_empty(list)) { */
/*   __list_splice(list, head); */
/*   INIT_LIST_HEAD(list); */
/*  } */
/* } */

/* struct hlist_head { */
/*  struct hlist_node *first; */
/* }; */

/* struct hlist_node { */
/*  struct hlist_node *next, **pprev; */
/* }; */




/* static inline __attribute__((always_inline)) void INIT_HLIST_NODE(struct hlist_node *h) */
/* { */
/*  h->next = ((void *)0); */
/*  h->pprev = ((void *)0); */
/* } */

/* static inline __attribute__((always_inline)) int hlist_unhashed(const struct hlist_node *h) */
/* { */
/*  return !h->pprev; */
/* } */

/* static inline __attribute__((always_inline)) int hlist_empty(const struct hlist_head *h) */
/* { */
/*  return !h->first; */
/* } */

/* static inline __attribute__((always_inline)) void __hlist_del(struct hlist_node *n) */
/* { */
/*  struct hlist_node *next = n->next; */
/*  struct hlist_node **pprev = n->pprev; */
/*  *pprev = next; */
/*  if (next) */
/*   next->pprev = pprev; */
/* } */

/* static inline __attribute__((always_inline)) void hlist_del(struct hlist_node *n) */
/* { */
/*  __hlist_del(n); */
/*  n->next = ((void *) 0x00100100); */
/*  n->pprev = ((void *) 0x00200200); */
/* } */

/* static inline __attribute__((always_inline)) void hlist_del_rcu(struct hlist_node *n) */
/* { */
/*  __hlist_del(n); */
/*  n->pprev = ((void *) 0x00200200); */
/* } */

/* static inline __attribute__((always_inline)) void hlist_del_init(struct hlist_node *n) */
/* { */
/*  if (!hlist_unhashed(n)) { */
/*   __hlist_del(n); */
/*   INIT_HLIST_NODE(n); */
/*  } */
/* } */

/* static inline __attribute__((always_inline)) void hlist_replace_rcu(struct hlist_node *old, */
/*      struct hlist_node *new) */
/* { */
/*  struct hlist_node *next = old->next; */

/*  new->next = next; */
/*  new->pprev = old->pprev; */
/*  __asm__ __volatile__("": : :"memory"); */
/*  if (next) */
/*   new->next->pprev = &new->next; */
/*  *new->pprev = new; */
/*  old->pprev = ((void *) 0x00200200); */
/* } */

/* static inline __attribute__((always_inline)) void hlist_add_head(struct hlist_node *n, struct hlist_head *h) */
/* { */
/*  struct hlist_node *first = h->first; */
/*  n->next = first; */
/*  if (first) */
/*   first->pprev = &n->next; */
/*  h->first = n; */
/*  n->pprev = &h->first; */
/* } */

/* static inline __attribute__((always_inline)) void hlist_add_head_rcu(struct hlist_node *n, */
/*      struct hlist_head *h) */
/* { */
/*  struct hlist_node *first = h->first; */
/*  n->next = first; */
/*  n->pprev = &h->first; */
/*  __asm__ __volatile__("": : :"memory"); */
/*  if (first) */
/*   first->pprev = &n->next; */
/*  h->first = n; */
/* } */


/* static inline __attribute__((always_inline)) void hlist_add_before(struct hlist_node *n, */
/*      struct hlist_node *next) */
/* { */
/*  n->pprev = next->pprev; */
/*  n->next = next; */
/*  next->pprev = &n->next; */
/*  *(n->pprev) = n; */
/* } */

/* static inline __attribute__((always_inline)) void hlist_add_after(struct hlist_node *n, */
/*      struct hlist_node *next) */
/* { */
/*  next->next = n->next; */
/*  n->next = next; */
/*  next->pprev = &n->next; */

/*  if(next->next) */
/*   next->next->pprev = &next->next; */
/* } */

/* static inline __attribute__((always_inline)) void hlist_add_before_rcu(struct hlist_node *n, */
/*      struct hlist_node *next) */
/* { */
/*  n->pprev = next->pprev; */
/*  n->next = next; */
/*  __asm__ __volatile__("": : :"memory"); */
/*  next->pprev = &n->next; */
/*  *(n->pprev) = n; */
/* } */

/* static inline __attribute__((always_inline)) void hlist_add_after_rcu(struct hlist_node *prev, */
/*            struct hlist_node *n) */
/* { */
/*  n->next = prev->next; */
/*  n->pprev = &prev->next; */
/*  __asm__ __volatile__("": : :"memory"); */
/*  prev->next = n; */
/*  if (n->next) */
/*   n->next->pprev = &n->next; */
/* } */





/* struct task_struct; */

/* extern int debug_locks; */
/* extern int debug_locks_silent; */




/* extern int debug_locks_off(void); */

/* static inline __attribute__((always_inline)) void debug_show_all_locks(void) */
/* { */
/* } */

/* static inline __attribute__((always_inline)) void debug_show_held_locks(struct task_struct *task) */
/* { */
/* } */

/* static inline __attribute__((always_inline)) void */
/* debug_check_no_locks_freed(const void *from, unsigned long len) */
/* { */
/* } */

/* static inline __attribute__((always_inline)) void */
/* debug_check_no_locks_held(struct task_struct *task) */
/* { */
/* } */




/* static inline __attribute__((always_inline)) void lockdep_off(void) */
/* { */
/* } */

/* static inline __attribute__((always_inline)) void lockdep_on(void) */
/* { */
/* } */

/* static inline __attribute__((always_inline)) int lockdep_internal(void) */
/* { */
/*  return 0; */
/* } */

/* struct lock_class_key { }; */







/* typedef struct { */
/*  volatile unsigned int slock; */



/* } raw_spinlock_t; */

/* typedef struct { */




/* } raw_rwlock_t; */



/* typedef struct { */
/*  raw_spinlock_t raw_lock; */




/*  unsigned int magic, owner_cpu; */
/*  void *owner; */




/* } spinlock_t; */



/* typedef struct { */
/*  raw_rwlock_t raw_lock; */




/*  unsigned int magic, owner_cpu; */
/*  void *owner; */




/* } rwlock_t; */


/* extern int __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) generic__raw_read_trylock(raw_rwlock_t *lock); */









/* static inline __attribute__((always_inline)) void __raw_spin_lock(raw_spinlock_t *lock) */
/* { */
/*  lock->slock = 0; */
/* } */

/* static inline __attribute__((always_inline)) void */
/* __raw_spin_lock_flags(raw_spinlock_t *lock, unsigned long flags) */
/* { */
/*  do { do { (flags) = __raw_local_irq_save(); } while (0); do { } while (0); } while (0); */
/*  lock->slock = 0; */
/* } */

/* static inline __attribute__((always_inline)) int __raw_spin_trylock(raw_spinlock_t *lock) */
/* { */
/*  char oldval = lock->slock; */

/*  lock->slock = 0; */

/*  return oldval > 0; */
/* } */

/* static inline __attribute__((always_inline)) void __raw_spin_unlock(raw_spinlock_t *lock) */
/* { */
/*  lock->slock = 1; */
/* } */




/*   extern void __spin_lock_init(spinlock_t *lock, const char *name, */
/*           struct lock_class_key *key); */

/*   extern void __rwlock_init(rwlock_t *lock, const char *name, */
/*        struct lock_class_key *key); */



/* int in_lock_functions(unsigned long addr); */



/* void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _spin_lock(spinlock_t *lock) ; */
/* void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _spin_lock_nested(spinlock_t *lock, int subclass) */
/*        ; */
/* void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _read_lock(rwlock_t *lock) ; */
/* void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _write_lock(rwlock_t *lock) ; */
/* void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _spin_lock_bh(spinlock_t *lock) ; */
/* void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _read_lock_bh(rwlock_t *lock) ; */
/* void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _write_lock_bh(rwlock_t *lock) ; */
/* void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _spin_lock_irq(spinlock_t *lock) ; */
/* void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _read_lock_irq(rwlock_t *lock) ; */
/* void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _write_lock_irq(rwlock_t *lock) ; */
/* unsigned long __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _spin_lock_irqsave(spinlock_t *lock) */
/*        ; */
/* unsigned long __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _read_lock_irqsave(rwlock_t *lock) */
/*        ; */
/* unsigned long __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _write_lock_irqsave(rwlock_t *lock) */
/*        ; */
/* int __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _spin_trylock(spinlock_t *lock); */
/* int __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _read_trylock(rwlock_t *lock); */
/* int __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _write_trylock(rwlock_t *lock); */
/* int __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _spin_trylock_bh(spinlock_t *lock); */
/* void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _spin_unlock(spinlock_t *lock) ; */
/* void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _read_unlock(rwlock_t *lock) ; */
/* void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _write_unlock(rwlock_t *lock) ; */
/* void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _spin_unlock_bh(spinlock_t *lock) ; */
/* void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _read_unlock_bh(rwlock_t *lock) ; */
/* void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _write_unlock_bh(rwlock_t *lock) ; */
/* void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _spin_unlock_irq(spinlock_t *lock) ; */
/* void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _read_unlock_irq(rwlock_t *lock) ; */
/* void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _write_unlock_irq(rwlock_t *lock) ; */
/* void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _spin_unlock_irqrestore(spinlock_t *lock, unsigned long flags) */
/*        ; */
/* void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _read_unlock_irqrestore(rwlock_t *lock, unsigned long flags) */
/*        ; */
/* void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _write_unlock_irqrestore(rwlock_t *lock, unsigned long flags) */
/*        ; */






/*  extern void _raw_spin_lock(spinlock_t *lock); */

/*  extern int _raw_spin_trylock(spinlock_t *lock); */
/*  extern void _raw_spin_unlock(spinlock_t *lock); */
/*  extern void _raw_read_lock(rwlock_t *lock); */
/*  extern int _raw_read_trylock(rwlock_t *lock); */
/*  extern void _raw_read_unlock(rwlock_t *lock); */
/*  extern void _raw_write_lock(rwlock_t *lock); */
/*  extern int _raw_write_trylock(rwlock_t *lock); */
/*  extern void _raw_write_unlock(rwlock_t *lock); */



/* typedef struct { volatile int counter; } atomic_t; */

/* static __inline__ __attribute__((always_inline)) void atomic_add(int i, atomic_t *v) */
/* { */
/*  __asm__ __volatile__( */
/*   "" "addl %1,%0" */
/*   :"+m" (v->counter) */
/*   :"ir" (i)); */
/* } */

/* static __inline__ __attribute__((always_inline)) void atomic_sub(int i, atomic_t *v) */
/* { */
/*  __asm__ __volatile__( */
/*   "" "subl %1,%0" */
/*   :"+m" (v->counter) */
/*   :"ir" (i)); */
/* } */

/* static __inline__ __attribute__((always_inline)) int atomic_sub_and_test(int i, atomic_t *v) */
/* { */
/*  unsigned char c; */

/*  __asm__ __volatile__( */
/*   "" "subl %2,%0; sete %1" */
/*   :"+m" (v->counter), "=qm" (c) */
/*   :"ir" (i) : "memory"); */
/*  return c; */
/* } */







/* static __inline__ __attribute__((always_inline)) void atomic_inc(atomic_t *v) */
/* { */
/*  __asm__ __volatile__( */
/*   "" "incl %0" */
/*   :"+m" (v->counter)); */
/* } */







/* static __inline__ __attribute__((always_inline)) void atomic_dec(atomic_t *v) */
/* { */
/*  __asm__ __volatile__( */
/*   "" "decl %0" */
/*   :"+m" (v->counter)); */
/* } */

/* static __inline__ __attribute__((always_inline)) int atomic_dec_and_test(atomic_t *v) */
/* { */
/*  unsigned char c; */

/*  __asm__ __volatile__( */
/*   "" "decl %0; sete %1" */
/*   :"+m" (v->counter), "=qm" (c) */
/*   : : "memory"); */
/*  return c != 0; */
/* } */

/* static __inline__ __attribute__((always_inline)) int atomic_inc_and_test(atomic_t *v) */
/* { */
/*  unsigned char c; */

/*  __asm__ __volatile__( */
/*   "" "incl %0; sete %1" */
/*   :"+m" (v->counter), "=qm" (c) */
/*   : : "memory"); */
/*  return c != 0; */
/* } */

/* static __inline__ __attribute__((always_inline)) int atomic_add_negative(int i, atomic_t *v) */
/* { */
/*  unsigned char c; */

/*  __asm__ __volatile__( */
/*   "" "addl %2,%0; sets %1" */
/*   :"+m" (v->counter), "=qm" (c) */
/*   :"ir" (i) : "memory"); */
/*  return c; */
/* } */

/* static __inline__ __attribute__((always_inline)) int atomic_add_return(int i, atomic_t *v) */
/* { */
/*  int __i; */






/*  __i = i; */
/*  __asm__ __volatile__( */
/*   "" "xaddl %0, %1;" */
/*   :"=r"(i) */
/*   :"m"(v->counter), "0"(i)); */
/*  return i + __i; */

/* } */

/* static __inline__ __attribute__((always_inline)) int atomic_sub_return(int i, atomic_t *v) */
/* { */
/*  return atomic_add_return(-i,v); */
/* } */



/* typedef atomic_t atomic_long_t; */


/* static inline __attribute__((always_inline)) long atomic_long_read(atomic_long_t *l) */
/* { */
/*  atomic_t *v = (atomic_t *)l; */

/*  return (long)((v)->counter); */
/* } */

/* static inline __attribute__((always_inline)) void atomic_long_set(atomic_long_t *l, long i) */
/* { */
/*  atomic_t *v = (atomic_t *)l; */

/*  (((v)->counter) = (i)); */
/* } */

/* static inline __attribute__((always_inline)) void atomic_long_inc(atomic_long_t *l) */
/* { */
/*  atomic_t *v = (atomic_t *)l; */

/*  atomic_inc(v); */
/* } */

/* static inline __attribute__((always_inline)) void atomic_long_dec(atomic_long_t *l) */
/* { */
/*  atomic_t *v = (atomic_t *)l; */

/*  atomic_dec(v); */
/* } */

/* static inline __attribute__((always_inline)) void atomic_long_add(long i, atomic_long_t *l) */
/* { */
/*  atomic_t *v = (atomic_t *)l; */

/*  atomic_add(i, v); */
/* } */

/* static inline __attribute__((always_inline)) void atomic_long_sub(long i, atomic_long_t *l) */
/* { */
/*  atomic_t *v = (atomic_t *)l; */

/*  atomic_sub(i, v); */
/* } */







/* extern int _atomic_dec_and_lock(atomic_t *atomic, spinlock_t *lock); */







/* struct task_struct; */

/* static inline __attribute__((always_inline)) __attribute__((always_inline)) struct task_struct * get_current(void) */
/* { */
/*  return current_thread_info()->task; */
/* } */



/* extern struct task_struct *__vericon_dummy_current; */


/* typedef __u32 kernel_cap_t; */

/* extern kernel_cap_t cap_bset; */

/* static inline __attribute__((always_inline)) kernel_cap_t cap_combine(kernel_cap_t a, kernel_cap_t b) */
/* { */
/*      kernel_cap_t dest; */
/*      (dest) = (a) | (b); */
/*      return dest; */
/* } */

/* static inline __attribute__((always_inline)) kernel_cap_t cap_intersect(kernel_cap_t a, kernel_cap_t b) */
/* { */
/*      kernel_cap_t dest; */
/*      (dest) = (a) & (b); */
/*      return dest; */
/* } */

/* static inline __attribute__((always_inline)) kernel_cap_t cap_drop(kernel_cap_t a, kernel_cap_t drop) */
/* { */
/*      kernel_cap_t dest; */
/*      (dest) = (a) & ~(drop); */
/*      return dest; */
/* } */

/* static inline __attribute__((always_inline)) kernel_cap_t cap_invert(kernel_cap_t c) */
/* { */
/*      kernel_cap_t dest; */
/*      (dest) = ~(c); */
/*      return dest; */
/* } */

/* int capable(int cap); */
/* int __capable(struct task_struct *t, int cap); */















/* typedef struct { */
/*  unsigned sequence; */
/*  spinlock_t lock; */
/* } seqlock_t; */

/* static inline __attribute__((always_inline)) void write_seqlock(seqlock_t *sl) */
/* { */
/*  _spin_lock(&sl->lock); */
/*  ++sl->sequence; */
/*  __asm__ __volatile__("": : :"memory"); */
/* } */

/* static inline __attribute__((always_inline)) void write_sequnlock(seqlock_t *sl) */
/* { */
/*  __asm__ __volatile__("": : :"memory"); */
/*  sl->sequence++; */
/*  _spin_unlock(&sl->lock); */
/* } */

/* static inline __attribute__((always_inline)) int write_tryseqlock(seqlock_t *sl) */
/* { */
/*  int ret = (_spin_trylock(&sl->lock)); */

/*  if (ret) { */
/*   ++sl->sequence; */
/*   __asm__ __volatile__("": : :"memory"); */
/*  } */
/*  return ret; */
/* } */


/* static inline __attribute__((always_inline)) __attribute__((always_inline)) unsigned read_seqbegin(const seqlock_t *sl) */
/* { */
/*  unsigned ret = sl->sequence; */
/*  __asm__ __volatile__("": : :"memory"); */
/*  return ret; */
/* } */

/* static inline __attribute__((always_inline)) __attribute__((always_inline)) int read_seqretry(const seqlock_t *sl, unsigned iv) */
/* { */
/*  __asm__ __volatile__("": : :"memory"); */
/*  return (iv & 1) | (sl->sequence ^ iv); */
/* } */

/* typedef struct seqcount { */
/*  unsigned sequence; */
/* } seqcount_t; */





/* static inline __attribute__((always_inline)) unsigned read_seqcount_begin(const seqcount_t *s) */
/* { */
/*  unsigned ret = s->sequence; */
/*  __asm__ __volatile__("": : :"memory"); */
/*  return ret; */
/* } */






/* static inline __attribute__((always_inline)) int read_seqcount_retry(const seqcount_t *s, unsigned iv) */
/* { */
/*  __asm__ __volatile__("": : :"memory"); */
/*  return (iv & 1) | (s->sequence ^ iv); */
/* } */






/* static inline __attribute__((always_inline)) void write_seqcount_begin(seqcount_t *s) */
/* { */
/*  s->sequence++; */
/*  __asm__ __volatile__("": : :"memory"); */
/* } */

/* static inline __attribute__((always_inline)) void write_seqcount_end(seqcount_t *s) */
/* { */
/*  __asm__ __volatile__("": : :"memory"); */
/*  s->sequence++; */
/* } */





/* struct timespec { */
/*  time_t tv_sec; */
/*  long tv_nsec; */
/* }; */


/* struct timeval { */
/*  time_t tv_sec; */
/*  suseconds_t tv_usec; */
/* }; */

/* struct timezone { */
/*  int tz_minuteswest; */
/*  int tz_dsttime; */
/* }; */

/* static inline __attribute__((always_inline)) int timespec_equal(struct timespec *a, struct timespec *b) */
/* { */
/*  return (a->tv_sec == b->tv_sec) && (a->tv_nsec == b->tv_nsec); */
/* } */






/* static inline __attribute__((always_inline)) int timespec_compare(struct timespec *lhs, struct timespec *rhs) */
/* { */
/*  if (lhs->tv_sec < rhs->tv_sec) */
/*   return -1; */
/*  if (lhs->tv_sec > rhs->tv_sec) */
/*   return 1; */
/*  return lhs->tv_nsec - rhs->tv_nsec; */
/* } */

/* static inline __attribute__((always_inline)) int timeval_compare(struct timeval *lhs, struct timeval *rhs) */
/* { */
/*  if (lhs->tv_sec < rhs->tv_sec) */
/*   return -1; */
/*  if (lhs->tv_sec > rhs->tv_sec) */
/*   return 1; */
/*  return lhs->tv_usec - rhs->tv_usec; */
/* } */

/* extern unsigned long mktime(const unsigned int year, const unsigned int mon, */
/*        const unsigned int day, const unsigned int hour, */
/*        const unsigned int min, const unsigned int sec); */

/* extern void set_normalized_timespec(struct timespec *ts, time_t sec, long nsec); */




/* static inline __attribute__((always_inline)) struct timespec timespec_sub(struct timespec lhs, */
/*       struct timespec rhs) */
/* { */
/*  struct timespec ts_delta; */
/*  set_normalized_timespec(&ts_delta, lhs.tv_sec - rhs.tv_sec, */
/*     lhs.tv_nsec - rhs.tv_nsec); */
/*  return ts_delta; */
/* } */







/* extern struct timespec xtime; */
/* extern struct timespec wall_to_monotonic; */
/* extern seqlock_t xtime_lock; */

/* void timekeeping_init(void); */

/* static inline __attribute__((always_inline)) unsigned long get_seconds(void) */
/* { */
/*  return xtime.tv_sec; */
/* } */

/* struct timespec current_kernel_time(void); */




/* extern void do_gettimeofday(struct timeval *tv); */
/* extern int do_settimeofday(struct timespec *tv); */
/* extern int do_sys_settimeofday(struct timespec *tv, struct timezone *tz); */

/* extern long do_utimes(int dfd, char *filename, struct timeval *times); */
/* struct itimerval; */
/* extern int do_setitimer(int which, struct itimerval *value, */
/*    struct itimerval *ovalue); */
/* extern unsigned int alarm_setitimer(unsigned int seconds); */
/* extern int do_getitimer(int which, struct itimerval *value); */
/* extern void getnstimeofday(struct timespec *tv); */

/* extern struct timespec timespec_trunc(struct timespec t, unsigned gran); */
/* extern int timekeeping_is_continuous(void); */

/* static inline __attribute__((always_inline)) s64 timespec_to_ns(const struct timespec *ts) */
/* { */
/*  return ((s64) ts->tv_sec * 1000000000L) + ts->tv_nsec; */
/* } */

/* static inline __attribute__((always_inline)) s64 timeval_to_ns(const struct timeval *tv) */
/* { */
/*  return ((s64) tv->tv_sec * 1000000000L) + */
/*   tv->tv_usec * 1000L; */
/* } */







/* extern struct timespec ns_to_timespec(const s64 nsec); */







/* extern struct timeval ns_to_timeval(const s64 nsec); */






/* static inline __attribute__((always_inline)) void timespec_add_ns(struct timespec *a, u64 ns) */
/* { */
/*  ns += a->tv_nsec; */
/*  while(__builtin_expect(!!(ns >= 1000000000L), 0)) { */
/*   ns -= 1000000000L; */
/*   a->tv_sec++; */
/*  } */
/*  a->tv_nsec = ns; */
/* } */

/* struct itimerspec { */
/*  struct timespec it_interval; */
/*  struct timespec it_value; */
/* }; */

/* struct itimerval { */
/*  struct timeval it_interval; */
/*  struct timeval it_value; */
/* }; */


/* struct timex { */
/*  unsigned int modes; */
/*  long offset; */
/*  long freq; */
/*  long maxerror; */
/*  long esterror; */
/*  int status; */
/*  long constant; */
/*  long precision; */
/*  long tolerance; */


/*  struct timeval time; */
/*  long tick; */

/*  long ppsfreq; */
/*  long jitter; */
/*  int shift; */
/*  long stabil; */
/*  long jitcnt; */
/*  long calcnt; */
/*  long errcnt; */
/*  long stbcnt; */

/*  int :32; int :32; int :32; int :32; */
/*  int :32; int :32; int :32; int :32; */
/*  int :32; int :32; int :32; int :32; */
/* }; */















/* typedef unsigned long long cycles_t; */

/* extern unsigned int cpu_khz; */
/* extern unsigned int tsc_khz; */

/* static inline __attribute__((always_inline)) cycles_t get_cycles(void) */
/* { */
/*  unsigned long long ret = 0; */







/*  __asm__ __volatile__("rdtsc" : "=A" (ret)); */

/*  return ret; */
/* } */

/* extern void tsc_init(void); */
/* extern void mark_tsc_unstable(void); */


/* extern int read_current_timer(unsigned long *timer_value); */







/* extern unsigned long tick_usec; */
/* extern unsigned long tick_nsec; */
/* extern int tickadj; */




/* extern int time_state; */
/* extern int time_status; */
/* extern long time_offset; */
/* extern long time_constant; */
/* extern long time_tolerance; */
/* extern long time_precision; */
/* extern long time_maxerror; */
/* extern long time_esterror; */

/* extern long time_freq; */
/* extern long time_reftime; */

/* extern long time_adjust; */
/* extern long time_next_adjust; */






/* static inline __attribute__((always_inline)) void ntp_clear(void) */
/* { */
/*  time_adjust = 0; */
/*  time_status |= 0x0040; */
/*  time_maxerror = (512000L << 5); */
/*  time_esterror = (512000L << 5); */
/* } */





/* static inline __attribute__((always_inline)) int ntp_synced(void) */
/* { */
/*  return !(time_status & 0x0040); */
/* } */

/* static inline __attribute__((always_inline)) void */
/* time_interpolator_reset(void) */
/* { */
/* } */






/* extern u64 current_tick_length(void); */

/* extern int do_adjtimex(struct timex *); */












/* static inline __attribute__((always_inline)) long */
/* div_ll_X_l_rem(long long divs, long div, long *rem) */
/* { */
/*  long dum2; */
/*       __asm__("divl %2":"=a"(dum2), "=d"(*rem) */
/*       : "rm"(div), "A"(divs)); */

/*  return dum2; */

/* } */


/* static inline __attribute__((always_inline)) long div_long_long_rem_signed(const long long dividend, */
/*          const long divisor, long *remainder) */
/* { */
/*  long res; */

/*  if (__builtin_expect(!!(dividend < 0), 0)) { */
/*   res = -div_ll_X_l_rem(-dividend,divisor,remainder); */
/*   *remainder = -(*remainder); */
/*  } else */
/*   res = div_ll_X_l_rem(dividend,divisor,remainder); */

/*  return res; */
/* } */


/* extern u64 __attribute__((section(".data"))) jiffies_64; */
/* extern unsigned long volatile __attribute__((section(".data"))) jiffies; */


/* u64 get_jiffies_64(void); */

/* static inline __attribute__((always_inline)) unsigned int jiffies_to_msecs(const unsigned long j) */
/* { */

/*  return (1000L / 1000) * j; */





/* } */

/* static inline __attribute__((always_inline)) unsigned int jiffies_to_usecs(const unsigned long j) */
/* { */

/*  return (1000000L / 1000) * j; */





/* } */

/* static inline __attribute__((always_inline)) unsigned long msecs_to_jiffies(const unsigned int m) */
/* { */
/*  if (m > jiffies_to_msecs(((~0UL >> 1)-1))) */
/*   return ((~0UL >> 1)-1); */

/*  return (m + (1000L / 1000) - 1) / (1000L / 1000); */





/* } */

/* static inline __attribute__((always_inline)) unsigned long usecs_to_jiffies(const unsigned int u) */
/* { */
/*  if (u > jiffies_to_usecs(((~0UL >> 1)-1))) */
/*   return ((~0UL >> 1)-1); */

/*  return (u + (1000000L / 1000) - 1) / (1000000L / 1000); */





/* } */

/* static __inline__ __attribute__((always_inline)) unsigned long */
/* timespec_to_jiffies(const struct timespec *value) */
/* { */
/*  unsigned long sec = value->tv_sec; */
/*  long nsec = value->tv_nsec + (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))))) - 1; */

/*  if (sec >= (long)((u64)((u64)((~0UL >> 1)-1) * (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))))) / 1000000000L)){ */
/*   sec = (long)((u64)((u64)((~0UL >> 1)-1) * (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))))) / 1000000000L); */
/*   nsec = 0; */
/*  } */
/*  return (((u64)sec * ((unsigned long)((((u64)1000000000L << (32 - 10)) + (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))))) -1) / (u64)(( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))))))) + */
/*   (((u64)nsec * ((unsigned long)((((u64)1 << ((32 - 10) + 29)) + (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))))) -1) / (u64)(( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))))))) >> */
/*    (((32 - 10) + 29) - (32 - 10)))) >> (32 - 10); */

/* } */

/* static __inline__ __attribute__((always_inline)) void */
/* jiffies_to_timespec(const unsigned long jiffies, struct timespec *value) */
/* { */




/*  u64 nsec = (u64)jiffies * (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))))); */
/*  value->tv_sec = div_ll_X_l_rem(nsec,1000000000L,&value->tv_nsec); */
/* } */

/* static __inline__ __attribute__((always_inline)) unsigned long */
/* timeval_to_jiffies(const struct timeval *value) */
/* { */
/*  unsigned long sec = value->tv_sec; */
/*  long usec = value->tv_usec; */

/*  if (sec >= (long)((u64)((u64)((~0UL >> 1)-1) * (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))))) / 1000000000L)){ */
/*   sec = (long)((u64)((u64)((~0UL >> 1)-1) * (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))))) / 1000000000L); */
/*   usec = 0; */
/*  } */
/*  return (((u64)sec * ((unsigned long)((((u64)1000000000L << (32 - 10)) + (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))))) -1) / (u64)(( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))))))) + */
/*   (((u64)usec * ((unsigned long)((((u64)1000L << ((32 - 10) + 19)) + (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))))) -1) / (u64)(( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))))))) + (u64)(((u64)1 << ((32 - 10) + 19)) - 1)) >> */
/*    (((32 - 10) + 19) - (32 - 10)))) >> (32 - 10); */
/* } */

/* static __inline__ __attribute__((always_inline)) void */
/* jiffies_to_timeval(const unsigned long jiffies, struct timeval *value) */
/* { */




/*  u64 nsec = (u64)jiffies * (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))))); */
/*  long tv_usec; */

/*  value->tv_sec = div_ll_X_l_rem(nsec,1000000000L,&tv_usec); */
/*  tv_usec /= 1000L; */
/*  value->tv_usec = tv_usec; */
/* } */




/* static inline __attribute__((always_inline)) clock_t jiffies_to_clock_t(long x) */
/* { */



/*  u64 tmp = (u64)x * (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))))); */
/*  ({ unsigned long __upper, __low, __high, __mod, __base; __base = ((1000000000L / 100)); asm("":"=a" (__low), "=d" (__high):"A" (tmp)); __upper = __high; if (__high) { __upper = __high % (__base); __high = __high / (__base); } asm("divl %2":"=a" (__low), "=d" (__mod):"rm" (__base), "0" (__low), "1" (__upper)); asm("":"=A" (tmp):"a" (__low),"d" (__high)); __mod; }); */
/*  return (long)tmp; */

/* } */

/* static inline __attribute__((always_inline)) unsigned long clock_t_to_jiffies(unsigned long x) */
/* { */

/*  if (x >= ~0UL / (1000 / 100)) */
/*   return ~0UL; */
/*  return x * (1000 / 100); */

/* } */

/* static inline __attribute__((always_inline)) u64 jiffies_64_to_clock_t(u64 x) */
/* { */

/*  x *= (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))))); */
/*  ({ unsigned long __upper, __low, __high, __mod, __base; __base = ((1000000000L / 100)); asm("":"=a" (__low), "=d" (__high):"A" (x)); __upper = __high; if (__high) { __upper = __high % (__base); __high = __high / (__base); } asm("divl %2":"=a" (__low), "=d" (__mod):"rm" (__base), "0" (__low), "1" (__upper)); asm("":"=A" (x):"a" (__low),"d" (__high)); __mod; }); */

/*  return x; */
/* } */

/* static inline __attribute__((always_inline)) u64 nsec_to_clock_t(u64 x) */
/* { */

/*  ({ unsigned long __upper, __low, __high, __mod, __base; __base = ((1000000000L / 100)); asm("":"=a" (__low), "=d" (__high):"A" (x)); __upper = __high; if (__high) { __upper = __high % (__base); __high = __high / (__base); } asm("divl %2":"=a" (__low), "=d" (__mod):"rm" (__base), "0" (__low), "1" (__upper)); asm("":"=A" (x):"a" (__low),"d" (__high)); __mod; }); */

/*  return x; */
/* } */



/* struct rb_node */
/* { */
/*  unsigned long rb_parent_color; */


/*  struct rb_node *rb_right; */
/*  struct rb_node *rb_left; */
/* } __attribute__((aligned(sizeof(long)))); */


/* struct rb_root */
/* { */
/*  struct rb_node *rb_node; */
/* }; */

/* static inline __attribute__((always_inline)) void rb_set_parent(struct rb_node *rb, struct rb_node *p) */
/* { */
/*  rb->rb_parent_color = (rb->rb_parent_color & 3) | (unsigned long)p; */
/* } */
/* static inline __attribute__((always_inline)) void rb_set_color(struct rb_node *rb, int color) */
/* { */
/*  rb->rb_parent_color = (rb->rb_parent_color & ~1) | color; */
/* } */

/* extern void rb_insert_color(struct rb_node *, struct rb_root *); */
/* extern void rb_erase(struct rb_node *, struct rb_root *); */


/* extern struct rb_node *rb_next(struct rb_node *); */
/* extern struct rb_node *rb_prev(struct rb_node *); */
/* extern struct rb_node *rb_first(struct rb_root *); */
/* extern struct rb_node *rb_last(struct rb_root *); */


/* extern void rb_replace_node(struct rb_node *victim, struct rb_node *new, */
/*        struct rb_root *root); */

/* static inline __attribute__((always_inline)) void rb_link_node(struct rb_node * node, struct rb_node * parent, */
/*     struct rb_node ** rb_link) */
/* { */
/*  node->rb_parent_color = (unsigned long )parent; */
/*  node->rb_left = node->rb_right = ((void *)0); */

/*  *rb_link = node; */
/* } */

























/* typedef struct { unsigned long bits[((((1 << 0))+32 -1)/32)]; } nodemask_t; */
/* extern nodemask_t _unused_nodemask_arg_; */


/* static inline __attribute__((always_inline)) void __node_set(int node, volatile nodemask_t *dstp) */
/* { */
/*  set_bit(node, dstp->bits); */
/* } */


/* static inline __attribute__((always_inline)) void __node_clear(int node, volatile nodemask_t *dstp) */
/* { */
/*  clear_bit(node, dstp->bits); */
/* } */


/* static inline __attribute__((always_inline)) void __nodes_setall(nodemask_t *dstp, int nbits) */
/* { */
/*  bitmap_fill(dstp->bits, nbits); */
/* } */


/* static inline __attribute__((always_inline)) void __nodes_clear(nodemask_t *dstp, int nbits) */
/* { */
/*  bitmap_zero(dstp->bits, nbits); */
/* } */






/* static inline __attribute__((always_inline)) int __node_test_and_set(int node, nodemask_t *addr) */
/* { */
/*  return test_and_set_bit(node, addr->bits); */
/* } */



/* static inline __attribute__((always_inline)) void __nodes_and(nodemask_t *dstp, const nodemask_t *src1p, */
/*      const nodemask_t *src2p, int nbits) */
/* { */
/*  bitmap_and(dstp->bits, src1p->bits, src2p->bits, nbits); */
/* } */



/* static inline __attribute__((always_inline)) void __nodes_or(nodemask_t *dstp, const nodemask_t *src1p, */
/*      const nodemask_t *src2p, int nbits) */
/* { */
/*  bitmap_or(dstp->bits, src1p->bits, src2p->bits, nbits); */
/* } */



/* static inline __attribute__((always_inline)) void __nodes_xor(nodemask_t *dstp, const nodemask_t *src1p, */
/*      const nodemask_t *src2p, int nbits) */
/* { */
/*  bitmap_xor(dstp->bits, src1p->bits, src2p->bits, nbits); */
/* } */



/* static inline __attribute__((always_inline)) void __nodes_andnot(nodemask_t *dstp, const nodemask_t *src1p, */
/*      const nodemask_t *src2p, int nbits) */
/* { */
/*  bitmap_andnot(dstp->bits, src1p->bits, src2p->bits, nbits); */
/* } */



/* static inline __attribute__((always_inline)) void __nodes_complement(nodemask_t *dstp, */
/*      const nodemask_t *srcp, int nbits) */
/* { */
/*  bitmap_complement(dstp->bits, srcp->bits, nbits); */
/* } */



/* static inline __attribute__((always_inline)) int __nodes_equal(const nodemask_t *src1p, */
/*      const nodemask_t *src2p, int nbits) */
/* { */
/*  return bitmap_equal(src1p->bits, src2p->bits, nbits); */
/* } */



/* static inline __attribute__((always_inline)) int __nodes_intersects(const nodemask_t *src1p, */
/*      const nodemask_t *src2p, int nbits) */
/* { */
/*  return bitmap_intersects(src1p->bits, src2p->bits, nbits); */
/* } */



/* static inline __attribute__((always_inline)) int __nodes_subset(const nodemask_t *src1p, */
/*      const nodemask_t *src2p, int nbits) */
/* { */
/*  return bitmap_subset(src1p->bits, src2p->bits, nbits); */
/* } */


/* static inline __attribute__((always_inline)) int __nodes_empty(const nodemask_t *srcp, int nbits) */
/* { */
/*  return bitmap_empty(srcp->bits, nbits); */
/* } */


/* static inline __attribute__((always_inline)) int __nodes_full(const nodemask_t *srcp, int nbits) */
/* { */
/*  return bitmap_full(srcp->bits, nbits); */
/* } */


/* static inline __attribute__((always_inline)) int __nodes_weight(const nodemask_t *srcp, int nbits) */
/* { */
/*  return bitmap_weight(srcp->bits, nbits); */
/* } */



/* static inline __attribute__((always_inline)) void __nodes_shift_right(nodemask_t *dstp, */
/*      const nodemask_t *srcp, int n, int nbits) */
/* { */
/*  bitmap_shift_right(dstp->bits, srcp->bits, n, nbits); */
/* } */



/* static inline __attribute__((always_inline)) void __nodes_shift_left(nodemask_t *dstp, */
/*      const nodemask_t *srcp, int n, int nbits) */
/* { */
/*  bitmap_shift_left(dstp->bits, srcp->bits, n, nbits); */
/* } */





/* static inline __attribute__((always_inline)) int __first_node(const nodemask_t *srcp) */
/* { */
/*  return ({ int __x = ((1 << 0)); int __y = (find_first_bit(srcp->bits, (1 << 0))); __x < __y ? __x: __y; }); */
/* } */


/* static inline __attribute__((always_inline)) int __next_node(int n, const nodemask_t *srcp) */
/* { */
/*  return ({ int __x = ((1 << 0)); int __y = (find_next_bit(srcp->bits, (1 << 0), n+1)); __x < __y ? __x: __y; }); */
/* } */

/* static inline __attribute__((always_inline)) int __first_unset_node(const nodemask_t *maskp) */
/* { */
/*  return ({ int __x = ((1 << 0)); int __y = (find_first_zero_bit(maskp->bits, (1 << 0))); __x < __y ? __x: __y; }); */

/* } */

/* static inline __attribute__((always_inline)) int __nodemask_scnprintf(char *buf, int len, */
/*      const nodemask_t *srcp, int nbits) */
/* { */
/*  return bitmap_scnprintf(buf, len, srcp->bits, nbits); */
/* } */



/* static inline __attribute__((always_inline)) int __nodemask_parse(const char *buf, int len, */
/*      nodemask_t *dstp, int nbits) */
/* { */
/*  return bitmap_parse(buf, len, dstp->bits, nbits); */
/* } */



/* static inline __attribute__((always_inline)) int __nodelist_scnprintf(char *buf, int len, */
/*      const nodemask_t *srcp, int nbits) */
/* { */
/*  return bitmap_scnlistprintf(buf, len, srcp->bits, nbits); */
/* } */


/* static inline __attribute__((always_inline)) int __nodelist_parse(const char *buf, nodemask_t *dstp, int nbits) */
/* { */
/*  return bitmap_parselist(buf, dstp->bits, nbits); */
/* } */



/* static inline __attribute__((always_inline)) int __node_remap(int oldbit, */
/*   const nodemask_t *oldp, const nodemask_t *newp, int nbits) */
/* { */
/*  return bitmap_bitremap(oldbit, oldp->bits, newp->bits, nbits); */
/* } */



/* static inline __attribute__((always_inline)) void __nodes_remap(nodemask_t *dstp, const nodemask_t *srcp, */
/*   const nodemask_t *oldp, const nodemask_t *newp, int nbits) */
/* { */
/*  bitmap_remap(dstp->bits, srcp->bits, oldp->bits, newp->bits, nbits); */
/* } */

/* extern nodemask_t node_online_map; */
/* extern nodemask_t node_possible_map; */







/* typedef struct __wait_queue wait_queue_t; */
/* typedef int (*wait_queue_func_t)(wait_queue_t *wait, unsigned mode, int sync, void *key); */
/* int default_wake_function(wait_queue_t *wait, unsigned mode, int sync, void *key); */

/* struct __wait_queue { */
/*  unsigned int flags; */

/*  void *private; */
/*  wait_queue_func_t func; */
/*  struct list_head task_list; */
/* }; */

/* struct wait_bit_key { */
/*  void *flags; */
/*  int bit_nr; */
/* }; */

/* struct wait_bit_queue { */
/*  struct wait_bit_key key; */
/*  wait_queue_t wait; */
/* }; */

/* struct __wait_queue_head { */
/*  spinlock_t lock; */
/*  struct list_head task_list; */
/* }; */
/* typedef struct __wait_queue_head wait_queue_head_t; */

/* struct task_struct; */

/* extern void init_waitqueue_head(wait_queue_head_t *q); */

/* static inline __attribute__((always_inline)) void init_waitqueue_entry(wait_queue_t *q, struct task_struct *p) */
/* { */
/*  q->flags = 0; */
/*  q->private = p; */
/*  q->func = default_wake_function; */
/* } */

/* static inline __attribute__((always_inline)) void init_waitqueue_func_entry(wait_queue_t *q, */
/*      wait_queue_func_t func) */
/* { */
/*  q->flags = 0; */
/*  q->private = ((void *)0); */
/*  q->func = func; */
/* } */

/* static inline __attribute__((always_inline)) int waitqueue_active(wait_queue_head_t *q) */
/* { */
/*  return !list_empty(&q->task_list); */
/* } */

/* extern void add_wait_queue(wait_queue_head_t *q, wait_queue_t * wait) __attribute__((regparm(3))); */
/* extern void add_wait_queue_exclusive(wait_queue_head_t *q, wait_queue_t * wait) __attribute__((regparm(3))); */
/* extern void remove_wait_queue(wait_queue_head_t *q, wait_queue_t * wait) __attribute__((regparm(3))); */

/* static inline __attribute__((always_inline)) void __add_wait_queue(wait_queue_head_t *head, wait_queue_t *new) */
/* { */
/*  list_add(&new->task_list, &head->task_list); */
/* } */




/* static inline __attribute__((always_inline)) void __add_wait_queue_tail(wait_queue_head_t *head, */
/*       wait_queue_t *new) */
/* { */
/*  list_add_tail(&new->task_list, &head->task_list); */
/* } */

/* static inline __attribute__((always_inline)) void __remove_wait_queue(wait_queue_head_t *head, */
/*        wait_queue_t *old) */
/* { */
/*  list_del(&old->task_list); */
/* } */

/* void __wake_up(wait_queue_head_t *q, unsigned int mode, int nr, void *key) __attribute__((regparm(3))); */
/* extern void __wake_up_locked(wait_queue_head_t *q, unsigned int mode) __attribute__((regparm(3))); */
/* extern void __wake_up_sync(wait_queue_head_t *q, unsigned int mode, int nr) __attribute__((regparm(3))); */
/* void __wake_up_bit(wait_queue_head_t *, void *, int) __attribute__((regparm(3))); */
/* int __wait_on_bit(wait_queue_head_t *, struct wait_bit_queue *, int (*)(void *), unsigned) __attribute__((regparm(3))); */
/* int __wait_on_bit_lock(wait_queue_head_t *, struct wait_bit_queue *, int (*)(void *), unsigned) __attribute__((regparm(3))); */
/* void wake_up_bit(void *, int) __attribute__((regparm(3))); */
/* int out_of_line_wait_on_bit(void *, int, int (*)(void *), unsigned) __attribute__((regparm(3))); */
/* int out_of_line_wait_on_bit_lock(void *, int, int (*)(void *), unsigned) __attribute__((regparm(3))); */
/* wait_queue_head_t *bit_waitqueue(void *, int) __attribute__((regparm(3))); */

/* static inline __attribute__((always_inline)) void add_wait_queue_exclusive_locked(wait_queue_head_t *q, */
/*          wait_queue_t * wait) */
/* { */
/*  wait->flags |= 0x01; */
/*  __add_wait_queue_tail(q, wait); */
/* } */




/* static inline __attribute__((always_inline)) void remove_wait_queue_locked(wait_queue_head_t *q, */
/*          wait_queue_t * wait) */
/* { */
/*  __remove_wait_queue(q, wait); */
/* } */






/* extern void sleep_on(wait_queue_head_t *q) __attribute__((regparm(3))); */
/* extern long sleep_on_timeout(wait_queue_head_t *q, signed long timeout) __attribute__((regparm(3))); */

/* extern void interruptible_sleep_on(wait_queue_head_t *q) __attribute__((regparm(3))); */
/* extern long interruptible_sleep_on_timeout(wait_queue_head_t *q, signed long timeout) __attribute__((regparm(3))); */





/* void prepare_to_wait(wait_queue_head_t *q, wait_queue_t *wait, int state) __attribute__((regparm(3))); */

/* void prepare_to_wait_exclusive(wait_queue_head_t *q, wait_queue_t *wait, int state) __attribute__((regparm(3))); */

/* void finish_wait(wait_queue_head_t *q, wait_queue_t *wait) __attribute__((regparm(3))); */
/* int autoremove_wake_function(wait_queue_t *wait, unsigned mode, int sync, void *key); */
/* int wake_bit_function(wait_queue_t *wait, unsigned mode, int sync, void *key); */

/* static inline __attribute__((always_inline)) int wait_on_bit(void *word, int bit, */
/*     int (*action)(void *), unsigned mode) */
/* { */
/*  if (!(__builtin_constant_p(bit) ? constant_test_bit((bit),(word)) : variable_test_bit((bit),(word)))) */
/*   return 0; */
/*  return out_of_line_wait_on_bit(word, bit, action, mode); */
/* } */

/* static inline __attribute__((always_inline)) int wait_on_bit_lock(void *word, int bit, */
/*     int (*action)(void *), unsigned mode) */
/* { */
/*  if (!test_and_set_bit(bit, word)) */
/*   return 0; */
/*  return out_of_line_wait_on_bit_lock(word, bit, action, mode); */
/* } */



/* struct rw_semaphore; */






/* struct rwsem_waiter; */

/* extern struct rw_semaphore *rwsem_down_read_failed(struct rw_semaphore *sem) __attribute__((regparm(3))); */
/* extern struct rw_semaphore *rwsem_down_write_failed(struct rw_semaphore *sem) __attribute__((regparm(3))); */
/* extern struct rw_semaphore *rwsem_wake(struct rw_semaphore *) __attribute__((regparm(3))); */
/* extern struct rw_semaphore *rwsem_downgrade_wake(struct rw_semaphore *sem) __attribute__((regparm(3))); */




/* struct rw_semaphore { */
/*  signed long count; */






/*  spinlock_t wait_lock; */
/*  struct list_head wait_list; */



/* }; */

/* extern void __init_rwsem(struct rw_semaphore *sem, const char *name, */
/*     struct lock_class_key *key); */

/* static inline __attribute__((always_inline)) void __down_read(struct rw_semaphore *sem) */
/* { */
/*  __asm__ __volatile__( */
/*   "# beginning down_read\n\t" */
/* "" "  incl      (%%eax)\n\t" */
/*   "  js        2f\n\t" */
/*   "1:\n\t" */
/*   ".subsection 1\n\t" "" ".ifndef " ".text.lock.""cdrom" "\n\t" ".text.lock.""cdrom" ":\n\t" ".endif\n" */
/*   "2:\n\t" */
/*   "  pushl     %%ecx\n\t" */
/*   "  pushl     %%edx\n\t" */
/*   "  call      rwsem_down_read_failed\n\t" */
/*   "  popl      %%edx\n\t" */
/*   "  popl      %%ecx\n\t" */
/*   "  jmp       1b\n" */
/*   ".previous\n\t" */
/*   "# ending down_read\n\t" */
/*   : "+m" (sem->count) */
/*   : "a" (sem) */
/*   : "memory", "cc"); */
/* } */




/* static inline __attribute__((always_inline)) int __down_read_trylock(struct rw_semaphore *sem) */
/* { */
/*  __s32 result, tmp; */
/*  __asm__ __volatile__( */
/*   "# beginning __down_read_trylock\n\t" */
/*   "  movl      %0,%1\n\t" */
/*   "1:\n\t" */
/*   "  movl	     %1,%2\n\t" */
/*   "  addl      %3,%2\n\t" */
/*   "  jle	     2f\n\t" */
/* "" "  cmpxchgl  %2,%0\n\t" */
/*   "  jnz	     1b\n\t" */
/*   "2:\n\t" */
/*   "# ending __down_read_trylock\n\t" */
/*   : "+m" (sem->count), "=&a" (result), "=&r" (tmp) */
/*   : "i" (0x00000001) */
/*   : "memory", "cc"); */
/*  return result>=0 ? 1 : 0; */
/* } */




/* static inline __attribute__((always_inline)) void __down_write_nested(struct rw_semaphore *sem, int subclass) */
/* { */
/*  int tmp; */

/*  tmp = ((-0x00010000) + 0x00000001); */
/*  __asm__ __volatile__( */
/*   "# beginning down_write\n\t" */
/* "" "  xadd      %%edx,(%%eax)\n\t" */
/*   "  testl     %%edx,%%edx\n\t" */
/*   "  jnz       2f\n\t" */
/*   "1:\n\t" */
/*   ".subsection 1\n\t" "" ".ifndef " ".text.lock.""cdrom" "\n\t" ".text.lock.""cdrom" ":\n\t" ".endif\n" */
/*   "2:\n\t" */
/*   "  pushl     %%ecx\n\t" */
/*   "  call      rwsem_down_write_failed\n\t" */
/*   "  popl      %%ecx\n\t" */
/*   "  jmp       1b\n" */
/*   ".previous\n\t" */
/*   "# ending down_write" */
/*   : "+m" (sem->count), "=d" (tmp) */
/*   : "a" (sem), "1" (tmp) */
/*   : "memory", "cc"); */
/* } */

/* static inline __attribute__((always_inline)) void __down_write(struct rw_semaphore *sem) */
/* { */
/*  __down_write_nested(sem, 0); */
/* } */




/* static inline __attribute__((always_inline)) int __down_write_trylock(struct rw_semaphore *sem) */
/* { */
/*  signed long ret = ((__typeof__(*(&sem->count)))__cmpxchg((&sem->count),(unsigned long)(0x00000000), (unsigned long)(((-0x00010000) + 0x00000001)),sizeof(*(&sem->count)))); */


/*  if (ret == 0x00000000) */
/*   return 1; */
/*  return 0; */
/* } */




/* static inline __attribute__((always_inline)) void __up_read(struct rw_semaphore *sem) */
/* { */
/*  __s32 tmp = -0x00000001; */
/*  __asm__ __volatile__( */
/*   "# beginning __up_read\n\t" */
/* "" "  xadd      %%edx,(%%eax)\n\t" */
/*   "  js        2f\n\t" */
/*   "1:\n\t" */
/*   ".subsection 1\n\t" "" ".ifndef " ".text.lock.""cdrom" "\n\t" ".text.lock.""cdrom" ":\n\t" ".endif\n" */
/*   "2:\n\t" */
/*   "  decw      %%dx\n\t" */
/*   "  jnz       1b\n\t" */
/*   "  pushl     %%ecx\n\t" */
/*   "  call      rwsem_wake\n\t" */
/*   "  popl      %%ecx\n\t" */
/*   "  jmp       1b\n" */
/*   ".previous\n\t" */
/*   "# ending __up_read\n" */
/*   : "+m" (sem->count), "=d" (tmp) */
/*   : "a" (sem), "1" (tmp) */
/*   : "memory", "cc"); */
/* } */




/* static inline __attribute__((always_inline)) void __up_write(struct rw_semaphore *sem) */
/* { */
/*  __asm__ __volatile__( */
/*   "# beginning __up_write\n\t" */
/*   "  movl      %2,%%edx\n\t" */
/* "" "  xaddl     %%edx,(%%eax)\n\t" */
/*   "  jnz       2f\n\t" */
/*   "1:\n\t" */
/*   ".subsection 1\n\t" "" ".ifndef " ".text.lock.""cdrom" "\n\t" ".text.lock.""cdrom" ":\n\t" ".endif\n" */
/*   "2:\n\t" */
/*   "  decw      %%dx\n\t" */
/*   "  jnz       1b\n\t" */
/*   "  pushl     %%ecx\n\t" */
/*   "  call      rwsem_wake\n\t" */
/*   "  popl      %%ecx\n\t" */
/*   "  jmp       1b\n" */
/*   ".previous\n\t" */
/*   "# ending __up_write\n" */
/*   : "+m" (sem->count) */
/*   : "a" (sem), "i" (-((-0x00010000) + 0x00000001)) */
/*   : "memory", "cc", "edx"); */
/* } */




/* static inline __attribute__((always_inline)) void __downgrade_write(struct rw_semaphore *sem) */
/* { */
/*  __asm__ __volatile__( */
/*   "# beginning __downgrade_write\n\t" */
/* "" "  addl      %2,(%%eax)\n\t" */
/*   "  js        2f\n\t" */
/*   "1:\n\t" */
/*   ".subsection 1\n\t" "" ".ifndef " ".text.lock.""cdrom" "\n\t" ".text.lock.""cdrom" ":\n\t" ".endif\n" */
/*   "2:\n\t" */
/*   "  pushl     %%ecx\n\t" */
/*   "  pushl     %%edx\n\t" */
/*   "  call      rwsem_downgrade_wake\n\t" */
/*   "  popl      %%edx\n\t" */
/*   "  popl      %%ecx\n\t" */
/*   "  jmp       1b\n" */
/*   ".previous\n\t" */
/*   "# ending __downgrade_write\n" */
/*   : "+m" (sem->count) */
/*   : "a" (sem), "i" (-(-0x00010000)) */
/*   : "memory", "cc"); */
/* } */




/* static inline __attribute__((always_inline)) void rwsem_atomic_add(int delta, struct rw_semaphore *sem) */
/* { */
/*  __asm__ __volatile__( */
/* "" "addl %1,%0" */
/*   : "+m" (sem->count) */
/*   : "ir" (delta)); */
/* } */




/* static inline __attribute__((always_inline)) int rwsem_atomic_update(int delta, struct rw_semaphore *sem) */
/* { */
/*  int tmp = delta; */

/*  __asm__ __volatile__( */
/* "" "xadd %0,%1" */
/*   : "+r" (tmp), "+m" (sem->count) */
/*   : : "memory"); */

/*  return tmp+delta; */
/* } */

/* static inline __attribute__((always_inline)) int rwsem_is_locked(struct rw_semaphore *sem) */
/* { */
/*  return (sem->count != 0); */
/* } */






/* extern void down_read(struct rw_semaphore *sem); */




/* extern int down_read_trylock(struct rw_semaphore *sem); */




/* extern void down_write(struct rw_semaphore *sem); */




/* extern int down_write_trylock(struct rw_semaphore *sem); */




/* extern void up_read(struct rw_semaphore *sem); */




/* extern void up_write(struct rw_semaphore *sem); */




/* extern void downgrade_write(struct rw_semaphore *sem); */


/* struct semaphore { */
/*  atomic_t count; */
/*  int sleepers; */
/*  wait_queue_head_t wait; */
/* }; */

/* static inline __attribute__((always_inline)) void sema_init (struct semaphore *sem, int val) */
/* { */






/*  (((&sem->count)->counter) = (val)); */
/*  sem->sleepers = 0; */
/*  init_waitqueue_head(&sem->wait); */
/* } */

/* static inline __attribute__((always_inline)) void init_MUTEX (struct semaphore *sem) */
/* { */
/*  sema_init(sem, 1); */
/* } */

/* static inline __attribute__((always_inline)) void init_MUTEX_LOCKED (struct semaphore *sem) */
/* { */
/*  sema_init(sem, 0); */
/* } */

/* __attribute__((regparm(3))) void __down_failed(void ); */
/* __attribute__((regparm(3))) int __down_failed_interruptible(void ); */
/* __attribute__((regparm(3))) int __down_failed_trylock(void ); */
/* __attribute__((regparm(3))) void __up_wakeup(void ); */






/* static inline __attribute__((always_inline)) void down(struct semaphore * sem) */
/* { */
/*  do { __might_sleep("include/asm/semaphore.h", 99); cond_resched(); } while (0); */
/*  __asm__ __volatile__( */
/*   "# atomic down operation\n\t" */
/*   "" "decl %0\n\t" */
/*   "js 2f\n" */
/*   "1:\n" */
/*   ".subsection 1\n\t" "" ".ifndef " ".text.lock.""cdrom" "\n\t" ".text.lock.""cdrom" ":\n\t" ".endif\n" */
/*   "2:\tlea %0,%%eax\n\t" */
/*   "call __down_failed\n\t" */
/*   "jmp 1b\n" */
/*   ".previous\n\t" */
/*   :"+m" (sem->count) */
/*   : */
/*   :"memory","ax"); */
/* } */





/* static inline __attribute__((always_inline)) int down_interruptible(struct semaphore * sem) */
/* { */
/*  int result; */

/*  do { __might_sleep("include/asm/semaphore.h", 123); cond_resched(); } while (0); */
/*  __asm__ __volatile__( */
/*   "# atomic interruptible down operation\n\t" */
/*   "" "decl %1\n\t" */
/*   "js 2f\n\t" */
/*   "xorl %0,%0\n" */
/*   "1:\n" */
/*   ".subsection 1\n\t" "" ".ifndef " ".text.lock.""cdrom" "\n\t" ".text.lock.""cdrom" ":\n\t" ".endif\n" */
/*   "2:\tlea %1,%%eax\n\t" */
/*   "call __down_failed_interruptible\n\t" */
/*   "jmp 1b\n" */
/*   ".previous\n\t" */
/*   :"=a" (result), "+m" (sem->count) */
/*   : */
/*   :"memory"); */
/*  return result; */
/* } */





/* static inline __attribute__((always_inline)) int down_trylock(struct semaphore * sem) */
/* { */
/*  int result; */

/*  __asm__ __volatile__( */
/*   "# atomic interruptible down operation\n\t" */
/*   "" "decl %1\n\t" */
/*   "js 2f\n\t" */
/*   "xorl %0,%0\n" */
/*   "1:\n" */
/*   ".subsection 1\n\t" "" ".ifndef " ".text.lock.""cdrom" "\n\t" ".text.lock.""cdrom" ":\n\t" ".endif\n" */
/*   "2:\tlea %1,%%eax\n\t" */
/*   "call __down_failed_trylock\n\t" */
/*   "jmp 1b\n" */
/*   ".previous\n\t" */
/*   :"=a" (result), "+m" (sem->count) */
/*   : */
/*   :"memory"); */
/*  return result; */
/* } */







/* static inline __attribute__((always_inline)) void up(struct semaphore * sem) */
/* { */
/*  __asm__ __volatile__( */
/*   "# atomic up operation\n\t" */
/*   "" "incl %0\n\t" */
/*   "jle 2f\n" */
/*   "1:\n" */
/*   ".subsection 1\n\t" "" ".ifndef " ".text.lock.""cdrom" "\n\t" ".text.lock.""cdrom" ":\n\t" ".endif\n" */
/*   "2:\tlea %0,%%eax\n\t" */
/*   "call __up_wakeup\n\t" */
/*   "jmp 1b\n" */
/*   ".previous\n\t" */
/*   ".subsection 0\n" */
/*   :"+m" (sem->count) */
/*   : */
/*   :"memory","ax"); */
/* } */




/* struct pt_regs { */
/*  long ebx; */
/*  long ecx; */
/*  long edx; */
/*  long esi; */
/*  long edi; */
/*  long ebp; */
/*  long eax; */
/*  int xds; */
/*  int xes; */
/*  long orig_eax; */
/*  long eip; */
/*  int xcs; */
/*  long eflags; */
/*  long esp; */
/*  int xss; */
/* }; */

/* struct task_struct; */
/* extern void send_sigtrap(struct task_struct *tsk, struct pt_regs *regs, int error_code); */

/* static inline __attribute__((always_inline)) int user_mode(struct pt_regs *regs) */
/* { */
/*  return (regs->xcs & 3) != 0; */
/* } */
/* static inline __attribute__((always_inline)) int user_mode_vm(struct pt_regs *regs) */
/* { */
/*  return ((regs->xcs & 3) | (regs->eflags & 0x00020000)) != 0; */
/* } */



/* typedef struct { */
/*  int size; */
/*  struct semaphore sem; */
/*  void *ldt; */
/*  void *vdso; */
/* } mm_context_t; */












/* typedef unsigned long cputime_t; */

/* typedef u64 cputime64_t; */





/* extern void cpu_idle(void); */

/* static inline __attribute__((always_inline)) int up_smp_call_function(void) */
/* { */
/*  return 0; */
/* } */

/* static inline __attribute__((always_inline)) void smp_send_reschedule(int cpu) { } */

/* void smp_setup_processor_id(void); */







/* struct ipc_perm */
/* { */
/*  __kernel_key_t key; */
/*  __kernel_uid_t uid; */
/*  __kernel_gid_t gid; */
/*  __kernel_uid_t cuid; */
/*  __kernel_gid_t cgid; */
/*  __kernel_mode_t mode; */
/*  unsigned short seq; */
/* }; */




/* struct ipc64_perm */
/* { */
/*  __kernel_key_t key; */
/*  __kernel_uid32_t uid; */
/*  __kernel_gid32_t gid; */
/*  __kernel_uid32_t cuid; */
/*  __kernel_gid32_t cgid; */
/*  __kernel_mode_t mode; */
/*  unsigned short __pad1; */
/*  unsigned short seq; */
/*  unsigned short __pad2; */
/*  unsigned long __unused1; */
/*  unsigned long __unused2; */
/* }; */


/* struct kern_ipc_perm */
/* { */
/*  spinlock_t lock; */
/*  int deleted; */
/*  key_t key; */
/*  uid_t uid; */
/*  gid_t gid; */
/*  uid_t cuid; */
/*  gid_t cgid; */
/*  mode_t mode; */
/*  unsigned long seq; */
/*  void *security; */
/* }; */


/* struct semid_ds { */
/*  struct ipc_perm sem_perm; */
/*  __kernel_time_t sem_otime; */
/*  __kernel_time_t sem_ctime; */
/*  struct sem *sem_base; */
/*  struct sem_queue *sem_pending; */
/*  struct sem_queue **sem_pending_last; */
/*  struct sem_undo *undo; */
/*  unsigned short sem_nsems; */
/* }; */




/* struct semid64_ds { */
/*  struct ipc64_perm sem_perm; */
/*  __kernel_time_t sem_otime; */
/*  unsigned long __unused1; */
/*  __kernel_time_t sem_ctime; */
/*  unsigned long __unused2; */
/*  unsigned long sem_nsems; */
/*  unsigned long __unused3; */
/*  unsigned long __unused4; */
/* }; */



/* struct sembuf { */
/*  unsigned short sem_num; */
/*  short sem_op; */
/*  short sem_flg; */
/* }; */


/* union semun { */
/*  int val; */
/*  struct semid_ds *buf; */
/*  unsigned short *array; */
/*  struct seminfo *__buf; */
/*  void *__pad; */
/* }; */

/* struct seminfo { */
/*  int semmap; */
/*  int semmni; */
/*  int semmns; */
/*  int semmnu; */
/*  int semmsl; */
/*  int semopm; */
/*  int semume; */
/*  int semusz; */
/*  int semvmx; */
/*  int semaem; */
/* }; */

/* struct task_struct; */


/* struct sem { */
/*  int semval; */
/*  int sempid; */
/* }; */


/* struct sem_array { */
/*  struct kern_ipc_perm sem_perm; */
/*  int sem_id; */
/*  time_t sem_otime; */
/*  time_t sem_ctime; */
/*  struct sem *sem_base; */
/*  struct sem_queue *sem_pending; */
/*  struct sem_queue **sem_pending_last; */
/*  struct sem_undo *undo; */
/*  unsigned long sem_nsems; */
/* }; */


/* struct sem_queue { */
/*  struct sem_queue * next; */
/*  struct sem_queue ** prev; */
/*  struct task_struct* sleeper; */
/*  struct sem_undo * undo; */
/*  int pid; */
/*  int status; */
/*  struct sem_array * sma; */
/*  int id; */
/*  struct sembuf * sops; */
/*  int nsops; */
/*  int alter; */
/* }; */




/* struct sem_undo { */
/*  struct sem_undo * proc_next; */
/*  struct sem_undo * id_next; */
/*  int semid; */
/*  short * semadj; */
/* }; */




/* struct sem_undo_list { */
/*  atomic_t refcnt; */
/*  spinlock_t lock; */
/*  struct sem_undo *proc_list; */
/* }; */

/* struct sysv_sem { */
/*  struct sem_undo_list *undo_list; */
/* }; */



/* extern int copy_semundo(unsigned long clone_flags, struct task_struct *tsk); */
/* extern void exit_sem(struct task_struct *tsk); */







/* struct siginfo; */

/* typedef unsigned long old_sigset_t; */

/* typedef struct { */
/*  unsigned long sig[(64 / 32)]; */
/* } sigset_t; */



/* typedef void __signalfn_t(int); */
/* typedef __signalfn_t *__sighandler_t; */

/* typedef void __restorefn_t(void); */
/* typedef __restorefn_t *__sigrestore_t; */



/* struct old_sigaction { */
/*  __sighandler_t sa_handler; */
/*  old_sigset_t sa_mask; */
/*  unsigned long sa_flags; */
/*  __sigrestore_t sa_restorer; */
/* }; */

/* struct sigaction { */
/*  __sighandler_t sa_handler; */
/*  unsigned long sa_flags; */
/*  __sigrestore_t sa_restorer; */
/*  sigset_t sa_mask; */
/* }; */

/* struct k_sigaction { */
/*  struct sigaction sa; */
/* }; */

/* typedef struct sigaltstack { */
/*  void *ss_sp; */
/*  int ss_flags; */
/*  size_t ss_size; */
/* } stack_t; */

/* static __inline__ __attribute__((always_inline)) void __gen_sigaddset(sigset_t *set, int _sig) */
/* { */
/*  __asm__("btsl %1,%0" : "+m"(*set) : "Ir"(_sig - 1) : "cc"); */
/* } */

/* static __inline__ __attribute__((always_inline)) void __const_sigaddset(sigset_t *set, int _sig) */
/* { */
/*  unsigned long sig = _sig - 1; */
/*  set->sig[sig / 32] |= 1 << (sig % 32); */
/* } */







/* static __inline__ __attribute__((always_inline)) void __gen_sigdelset(sigset_t *set, int _sig) */
/* { */
/*  __asm__("btrl %1,%0" : "+m"(*set) : "Ir"(_sig - 1) : "cc"); */
/* } */

/* static __inline__ __attribute__((always_inline)) void __const_sigdelset(sigset_t *set, int _sig) */
/* { */
/*  unsigned long sig = _sig - 1; */
/*  set->sig[sig / 32] &= ~(1 << (sig % 32)); */
/* } */

/* static __inline__ __attribute__((always_inline)) int __const_sigismember(sigset_t *set, int _sig) */
/* { */
/*  unsigned long sig = _sig - 1; */
/*  return 1 & (set->sig[sig / 32] >> (sig % 32)); */
/* } */

/* static __inline__ __attribute__((always_inline)) int __gen_sigismember(sigset_t *set, int _sig) */
/* { */
/*  int ret; */
/*  __asm__("btl %2,%1\n\tsbbl %0,%0" */
/*   : "=r"(ret) : "m"(*set), "Ir"(_sig-1) : "cc"); */
/*  return ret; */
/* } */






/* static __inline__ __attribute__((always_inline)) int sigfindinword(unsigned long word) */
/* { */
/*  __asm__("bsfl %1,%0" : "=r"(word) : "rm"(word) : "cc"); */
/*  return word; */
/* } */

/* struct pt_regs; */












/* typedef union sigval { */
/*  int sival_int; */
/*  void *sival_ptr; */
/* } sigval_t; */

/* typedef struct siginfo { */
/*  int si_signo; */
/*  int si_errno; */
/*  int si_code; */

/*  union { */
/*   int _pad[((128 - (3 * sizeof(int))) / sizeof(int))]; */


/*   struct { */
/*    pid_t _pid; */
/*    uid_t _uid; */
/*   } _kill; */


/*   struct { */
/*    timer_t _tid; */
/*    int _overrun; */
/*    char _pad[sizeof( uid_t) - sizeof(int)]; */
/*    sigval_t _sigval; */
/*    int _sys_private; */
/*   } _timer; */


/*   struct { */
/*    pid_t _pid; */
/*    uid_t _uid; */
/*    sigval_t _sigval; */
/*   } _rt; */


/*   struct { */
/*    pid_t _pid; */
/*    uid_t _uid; */
/*    int _status; */
/*    clock_t _utime; */
/*    clock_t _stime; */
/*   } _sigchld; */


/*   struct { */
/*    void *_addr; */



/*   } _sigfault; */


/*   struct { */
/*    long _band; */
/*    int _fd; */
/*   } _sigpoll; */
/*  } _sifields; */
/* } siginfo_t; */

/* typedef struct sigevent { */
/*  sigval_t sigev_value; */
/*  int sigev_signo; */
/*  int sigev_notify; */
/*  union { */
/*   int _pad[((64 - (sizeof(int) * 2 + sizeof(sigval_t))) / sizeof(int))]; */
/*    int _tid; */

/*   struct { */
/*    void (*_function)(sigval_t); */
/*    void *_attribute; */
/*   } _sigev_thread; */
/*  } _sigev_un; */
/* } sigevent_t; */







/* struct siginfo; */
/* void do_schedule_next_timer(struct siginfo *info); */





/* static inline __attribute__((always_inline)) void copy_siginfo(struct siginfo *to, struct siginfo *from) */
/* { */
/*  if (from->si_code < 0) */
/*   (__builtin_constant_p(sizeof(*to)) ? __constant_memcpy((to),(from),(sizeof(*to))) : __memcpy((to),(from),(sizeof(*to)))); */
/*  else */

/*   (__builtin_constant_p((3 * sizeof(int)) + sizeof(from->_sifields._sigchld)) ? __constant_memcpy((to),(from),((3 * sizeof(int)) + sizeof(from->_sifields._sigchld))) : __memcpy((to),(from),((3 * sizeof(int)) + sizeof(from->_sifields._sigchld)))); */
/* } */



/* extern int copy_siginfo_to_user(struct siginfo *to, struct siginfo *from); */



/* struct sigqueue { */
/*  struct list_head list; */
/*  int flags; */
/*  siginfo_t info; */
/*  struct user_struct *user; */
/* }; */




/* struct sigpending { */
/*  struct list_head list; */
/*  sigset_t signal; */
/* }; */

/* static inline __attribute__((always_inline)) int sigisemptyset(sigset_t *set) */
/* { */
/*  extern void _NSIG_WORDS_is_unsupported_size(void); */
/*  switch ((64 / 32)) { */
/*  case 4: */
/*   return (set->sig[3] | set->sig[2] | */
/*    set->sig[1] | set->sig[0]) == 0; */
/*  case 2: */
/*   return (set->sig[1] | set->sig[0]) == 0; */
/*  case 1: */
/*   return set->sig[0] == 0; */
/*  default: */
/*   _NSIG_WORDS_is_unsupported_size(); */
/*   return 0; */
/*  } */
/* } */

/* static inline __attribute__((always_inline)) void sigorsets(sigset_t *r, const sigset_t *a, const sigset_t *b) { extern void _NSIG_WORDS_is_unsupported_size(void); unsigned long a0, a1, a2, a3, b0, b1, b2, b3; switch ((64 / 32)) { case 4: a3 = a->sig[3]; a2 = a->sig[2]; b3 = b->sig[3]; b2 = b->sig[2]; r->sig[3] = ((a3) | (b3)); r->sig[2] = ((a2) | (b2)); case 2: a1 = a->sig[1]; b1 = b->sig[1]; r->sig[1] = ((a1) | (b1)); case 1: a0 = a->sig[0]; b0 = b->sig[0]; r->sig[0] = ((a0) | (b0)); break; default: _NSIG_WORDS_is_unsupported_size(); } } */


/* static inline __attribute__((always_inline)) void sigandsets(sigset_t *r, const sigset_t *a, const sigset_t *b) { extern void _NSIG_WORDS_is_unsupported_size(void); unsigned long a0, a1, a2, a3, b0, b1, b2, b3; switch ((64 / 32)) { case 4: a3 = a->sig[3]; a2 = a->sig[2]; b3 = b->sig[3]; b2 = b->sig[2]; r->sig[3] = ((a3) & (b3)); r->sig[2] = ((a2) & (b2)); case 2: a1 = a->sig[1]; b1 = b->sig[1]; r->sig[1] = ((a1) & (b1)); case 1: a0 = a->sig[0]; b0 = b->sig[0]; r->sig[0] = ((a0) & (b0)); break; default: _NSIG_WORDS_is_unsupported_size(); } } */


/* static inline __attribute__((always_inline)) void signandsets(sigset_t *r, const sigset_t *a, const sigset_t *b) { extern void _NSIG_WORDS_is_unsupported_size(void); unsigned long a0, a1, a2, a3, b0, b1, b2, b3; switch ((64 / 32)) { case 4: a3 = a->sig[3]; a2 = a->sig[2]; b3 = b->sig[3]; b2 = b->sig[2]; r->sig[3] = ((a3) & ~(b3)); r->sig[2] = ((a2) & ~(b2)); case 2: a1 = a->sig[1]; b1 = b->sig[1]; r->sig[1] = ((a1) & ~(b1)); case 1: a0 = a->sig[0]; b0 = b->sig[0]; r->sig[0] = ((a0) & ~(b0)); break; default: _NSIG_WORDS_is_unsupported_size(); } } */

/* static inline __attribute__((always_inline)) void signotset(sigset_t *set) { extern void _NSIG_WORDS_is_unsupported_size(void); switch ((64 / 32)) { case 4: set->sig[3] = (~(set->sig[3])); set->sig[2] = (~(set->sig[2])); case 2: set->sig[1] = (~(set->sig[1])); case 1: set->sig[0] = (~(set->sig[0])); break; default: _NSIG_WORDS_is_unsupported_size(); } } */




/* static inline __attribute__((always_inline)) void sigemptyset(sigset_t *set) */
/* { */
/*  switch ((64 / 32)) { */
/*  default: */
/*   (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(sigset_t))) ? __constant_c_and_count_memset(((set)),((0x01010101UL*(unsigned char)(0))),((sizeof(sigset_t)))) : __constant_c_memset(((set)),((0x01010101UL*(unsigned char)(0))),((sizeof(sigset_t))))) : (__builtin_constant_p((sizeof(sigset_t))) ? __memset_generic((((set))),(((0))),(((sizeof(sigset_t))))) : __memset_generic(((set)),((0)),((sizeof(sigset_t)))))); */
/*   break; */
/*  case 2: set->sig[1] = 0; */
/*  case 1: set->sig[0] = 0; */
/*   break; */
/*  } */
/* } */

/* static inline __attribute__((always_inline)) void sigfillset(sigset_t *set) */
/* { */
/*  switch ((64 / 32)) { */
/*  default: */
/*   (__builtin_constant_p(-1) ? (__builtin_constant_p((sizeof(sigset_t))) ? __constant_c_and_count_memset(((set)),((0x01010101UL*(unsigned char)(-1))),((sizeof(sigset_t)))) : __constant_c_memset(((set)),((0x01010101UL*(unsigned char)(-1))),((sizeof(sigset_t))))) : (__builtin_constant_p((sizeof(sigset_t))) ? __memset_generic((((set))),(((-1))),(((sizeof(sigset_t))))) : __memset_generic(((set)),((-1)),((sizeof(sigset_t)))))); */
/*   break; */
/*  case 2: set->sig[1] = -1; */
/*  case 1: set->sig[0] = -1; */
/*   break; */
/*  } */
/* } */



/* static inline __attribute__((always_inline)) void sigaddsetmask(sigset_t *set, unsigned long mask) */
/* { */
/*  set->sig[0] |= mask; */
/* } */

/* static inline __attribute__((always_inline)) void sigdelsetmask(sigset_t *set, unsigned long mask) */
/* { */
/*  set->sig[0] &= ~mask; */
/* } */

/* static inline __attribute__((always_inline)) int sigtestsetmask(sigset_t *set, unsigned long mask) */
/* { */
/*  return (set->sig[0] & mask) != 0; */
/* } */

/* static inline __attribute__((always_inline)) void siginitset(sigset_t *set, unsigned long mask) */
/* { */
/*  set->sig[0] = mask; */
/*  switch ((64 / 32)) { */
/*  default: */
/*   (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(long)*((64 / 32)-1))) ? __constant_c_and_count_memset(((&set->sig[1])),((0x01010101UL*(unsigned char)(0))),((sizeof(long)*((64 / 32)-1)))) : __constant_c_memset(((&set->sig[1])),((0x01010101UL*(unsigned char)(0))),((sizeof(long)*((64 / 32)-1))))) : (__builtin_constant_p((sizeof(long)*((64 / 32)-1))) ? __memset_generic((((&set->sig[1]))),(((0))),(((sizeof(long)*((64 / 32)-1))))) : __memset_generic(((&set->sig[1])),((0)),((sizeof(long)*((64 / 32)-1)))))); */
/*   break; */
/*  case 2: set->sig[1] = 0; */
/*  case 1: ; */
/*  } */
/* } */

/* static inline __attribute__((always_inline)) void siginitsetinv(sigset_t *set, unsigned long mask) */
/* { */
/*  set->sig[0] = ~mask; */
/*  switch ((64 / 32)) { */
/*  default: */
/*   (__builtin_constant_p(-1) ? (__builtin_constant_p((sizeof(long)*((64 / 32)-1))) ? __constant_c_and_count_memset(((&set->sig[1])),((0x01010101UL*(unsigned char)(-1))),((sizeof(long)*((64 / 32)-1)))) : __constant_c_memset(((&set->sig[1])),((0x01010101UL*(unsigned char)(-1))),((sizeof(long)*((64 / 32)-1))))) : (__builtin_constant_p((sizeof(long)*((64 / 32)-1))) ? __memset_generic((((&set->sig[1]))),(((-1))),(((sizeof(long)*((64 / 32)-1))))) : __memset_generic(((&set->sig[1])),((-1)),((sizeof(long)*((64 / 32)-1)))))); */
/*   break; */
/*  case 2: set->sig[1] = -1; */
/*  case 1: ; */
/*  } */
/* } */



/* static inline __attribute__((always_inline)) void init_sigpending(struct sigpending *sig) */
/* { */
/*  sigemptyset(&sig->signal); */
/*  INIT_LIST_HEAD(&sig->list); */
/* } */

/* extern void flush_sigqueue(struct sigpending *queue); */


/* static inline __attribute__((always_inline)) int valid_signal(unsigned long sig) */
/* { */
/*  return sig <= 64 ? 1 : 0; */
/* } */

/* extern int group_send_sig_info(int sig, struct siginfo *info, struct task_struct *p); */
/* extern int __group_send_sig_info(int, struct siginfo *, struct task_struct *); */
/* extern long do_sigpending(void *, unsigned long); */
/* extern int sigprocmask(int, sigset_t *, sigset_t *); */

/* struct pt_regs; */
/* extern int get_signal_to_deliver(siginfo_t *info, struct k_sigaction *return_ka, struct pt_regs *regs, void *cookie); */







/* extern unsigned securebits; */





/* struct dentry; */
/* struct vfsmount; */

/* struct fs_struct { */
/*  atomic_t count; */
/*  rwlock_t lock; */
/*  int umask; */
/*  struct dentry * root, * pwd, * altroot; */
/*  struct vfsmount * rootmnt, * pwdmnt, * altrootmnt; */
/* }; */







/* extern void exit_fs(struct task_struct *); */
/* extern void set_fs_altroot(void); */
/* extern void set_fs_root(struct fs_struct *, struct vfsmount *, struct dentry *); */
/* extern void set_fs_pwd(struct fs_struct *, struct vfsmount *, struct dentry *); */
/* extern struct fs_struct *copy_fs_struct(struct fs_struct *); */
/* extern void put_fs_struct(struct fs_struct *); */




/* struct completion { */
/*  unsigned int done; */
/*  wait_queue_head_t wait; */
/* }; */

/* static inline __attribute__((always_inline)) void init_completion(struct completion *x) */
/* { */
/*  x->done = 0; */
/*  init_waitqueue_head(&x->wait); */
/* } */

/* extern void wait_for_completion(struct completion *) __attribute__((regparm(3))); */
/* extern int wait_for_completion_interruptible(struct completion *x) __attribute__((regparm(3))); */
/* extern unsigned long wait_for_completion_timeout(struct completion *x, unsigned long timeout) __attribute__((regparm(3))); */

/* extern unsigned long wait_for_completion_interruptible_timeout( struct completion *x, unsigned long timeout) __attribute__((regparm(3))); */


/* extern void complete(struct completion *) __attribute__((regparm(3))); */
/* extern void complete_all(struct completion *) __attribute__((regparm(3))); */













/* typedef struct kmem_cache kmem_cache_t; */









/* typedef int (*initcall_t)(void); */
/* typedef void (*exitcall_t)(void); */

/* extern initcall_t __con_initcall_start[], __con_initcall_end[]; */
/* extern initcall_t __security_initcall_start[], __security_initcall_end[]; */


/* extern char saved_command_line[]; */


/* extern void setup_arch(char **); */


/* struct free_area { */
/*  struct list_head free_list; */
/*  unsigned long nr_free; */
/* }; */

/* struct pglist_data; */

/* enum zone_stat_item { */
/*  NR_ANON_PAGES, */
/*  NR_FILE_MAPPED, */

/*  NR_FILE_PAGES, */
/*  NR_SLAB, */
/*  NR_PAGETABLE, */
/*  NR_FILE_DIRTY, */
/*  NR_WRITEBACK, */
/*  NR_UNSTABLE_NFS, */
/*  NR_BOUNCE, */

/*  NR_VM_ZONE_STAT_ITEMS }; */

/* struct per_cpu_pages { */
/*  int count; */
/*  int high; */
/*  int batch; */
/*  struct list_head list; */
/* }; */

/* struct per_cpu_pageset { */
/*  struct per_cpu_pages pcp[2]; */




/* } ; */

/* struct zone { */

/*  unsigned long free_pages; */
/*  unsigned long pages_min, pages_low, pages_high; */

/*  unsigned long lowmem_reserve[4]; */

/*  struct per_cpu_pageset pageset[1]; */




/*  spinlock_t lock; */




/*  struct free_area free_area[11]; */





/*  spinlock_t lru_lock; */
/*  struct list_head active_list; */
/*  struct list_head inactive_list; */
/*  unsigned long nr_scan_active; */
/*  unsigned long nr_scan_inactive; */
/*  unsigned long nr_active; */
/*  unsigned long nr_inactive; */
/*  unsigned long pages_scanned; */
/*  int all_unreclaimable; */


/*  atomic_t reclaim_in_progress; */


/*  atomic_long_t vm_stat[NR_VM_ZONE_STAT_ITEMS]; */

/*  int prev_priority; */




/*  wait_queue_head_t * wait_table; */
/*  unsigned long wait_table_hash_nr_entries; */
/*  unsigned long wait_table_bits; */




/*  struct pglist_data *zone_pgdat; */

/*  unsigned long zone_start_pfn; */

/*  unsigned long spanned_pages; */
/*  unsigned long present_pages; */




/*  char *name; */
/* } ; */

/* struct zonelist { */
/*  struct zone *zones[(1 << 0) * 4 + 1]; */
/* }; */

/* struct bootmem_data; */
/* typedef struct pglist_data { */
/*  struct zone node_zones[4]; */
/*  struct zonelist node_zonelists[((0x07 + 1) / 2 + 1)]; */
/*  int nr_zones; */

/*  struct page *node_mem_map; */

/*  struct bootmem_data *bdata; */

/*  unsigned long node_start_pfn; */
/*  unsigned long node_present_pages; */
/*  unsigned long node_spanned_pages; */

/*  int node_id; */
/*  wait_queue_head_t kswapd_wait; */
/*  struct task_struct *kswapd; */
/*  int kswapd_max_order; */
/* } pg_data_t; */













/* struct mutex { */

/*  atomic_t count; */
/*  spinlock_t wait_lock; */
/*  struct list_head wait_list; */

/* }; */





/* struct mutex_waiter { */
/*  struct list_head list; */
/*  struct task_struct *task; */




/* }; */

/* extern void __mutex_init(struct mutex *lock, const char *name, */
/*     struct lock_class_key *key); */







/* static inline __attribute__((always_inline)) int __attribute__((regparm(3))) mutex_is_locked(struct mutex *lock) */
/* { */
/*  return ((&lock->count)->counter) != 1; */
/* } */





/* extern void __attribute__((regparm(3))) mutex_lock(struct mutex *lock); */
/* extern int __attribute__((regparm(3))) mutex_lock_interruptible(struct mutex *lock); */

/* extern int __attribute__((regparm(3))) mutex_trylock(struct mutex *lock); */
/* extern void __attribute__((regparm(3))) mutex_unlock(struct mutex *lock); */


/* struct notifier_block { */
/*  int (*notifier_call)(struct notifier_block *, unsigned long, void *); */
/*  struct notifier_block *next; */
/*  int priority; */
/* }; */

/* struct atomic_notifier_head { */
/*  spinlock_t lock; */
/*  struct notifier_block *head; */
/* }; */

/* struct blocking_notifier_head { */
/*  struct rw_semaphore rwsem; */
/*  struct notifier_block *head; */
/* }; */

/* struct raw_notifier_head { */
/*  struct notifier_block *head; */
/* }; */

/* extern int atomic_notifier_chain_register(struct atomic_notifier_head *, */
/*   struct notifier_block *); */
/* extern int blocking_notifier_chain_register(struct blocking_notifier_head *, */
/*   struct notifier_block *); */
/* extern int raw_notifier_chain_register(struct raw_notifier_head *, */
/*   struct notifier_block *); */

/* extern int atomic_notifier_chain_unregister(struct atomic_notifier_head *, */
/*   struct notifier_block *); */
/* extern int blocking_notifier_chain_unregister(struct blocking_notifier_head *, */
/*   struct notifier_block *); */
/* extern int raw_notifier_chain_unregister(struct raw_notifier_head *, */
/*   struct notifier_block *); */

/* extern int atomic_notifier_call_chain(struct atomic_notifier_head *, */
/*   unsigned long val, void *v); */
/* extern int blocking_notifier_call_chain(struct blocking_notifier_head *, */
/*   unsigned long val, void *v); */
/* extern int raw_notifier_call_chain(struct raw_notifier_head *, */
/*   unsigned long val, void *v); */


/* struct page; */
/* struct zone; */
/* struct pglist_data; */

/* static inline __attribute__((always_inline)) void pgdat_resize_lock(struct pglist_data *p, unsigned long *f) {} */
/* static inline __attribute__((always_inline)) void pgdat_resize_unlock(struct pglist_data *p, unsigned long *f) {} */
/* static inline __attribute__((always_inline)) void pgdat_resize_init(struct pglist_data *pgdat) {} */

/* static inline __attribute__((always_inline)) unsigned zone_span_seqbegin(struct zone *zone) */
/* { */
/*  return 0; */
/* } */
/* static inline __attribute__((always_inline)) int zone_span_seqretry(struct zone *zone, unsigned iv) */
/* { */
/*  return 0; */
/* } */
/* static inline __attribute__((always_inline)) void zone_span_writelock(struct zone *zone) {} */
/* static inline __attribute__((always_inline)) void zone_span_writeunlock(struct zone *zone) {} */
/* static inline __attribute__((always_inline)) void zone_seqlock_init(struct zone *zone) {} */

/* static inline __attribute__((always_inline)) int mhp_notimplemented(const char *func) */
/* { */
/*  printk("<4>" "%s() called, with CONFIG_MEMORY_HOTPLUG disabled\n", func); */
/*  dump_stack(); */
/*  return -38; */
/* } */


/* static inline __attribute__((always_inline)) int __remove_pages(struct zone *zone, unsigned long start_pfn, */
/*  unsigned long nr_pages) */
/* { */
/*  printk("<4>" "%s() called, not yet supported\n", (__func__)); */
/*  dump_stack(); */
/*  return -38; */
/* } */

/* extern int add_memory(int nid, u64 start, u64 size); */
/* extern int arch_add_memory(int nid, u64 start, u64 size); */
/* extern int remove_memory(u64 start, u64 size); */


/* void __get_zone_counts(unsigned long *active, unsigned long *inactive, */
/*    unsigned long *free, struct pglist_data *pgdat); */
/* void get_zone_counts(unsigned long *active, unsigned long *inactive, */
/*    unsigned long *free); */
/* void build_all_zonelists(void); */
/* void wakeup_kswapd(struct zone *zone, int order); */
/* int zone_watermark_ok(struct zone *z, int order, unsigned long mark, */
/*   int classzone_idx, int alloc_flags); */

/* extern int init_currently_empty_zone(struct zone *zone, unsigned long start_pfn, */
/*          unsigned long size); */




/* static inline __attribute__((always_inline)) void memory_present(int nid, unsigned long start, unsigned long end) {} */

/* static inline __attribute__((always_inline)) int populated_zone(struct zone *zone) */
/* { */
/*  return (!!zone->present_pages); */
/* } */

/* static inline __attribute__((always_inline)) int is_highmem_idx(int idx) */
/* { */
/*  return (idx == 3); */
/* } */

/* static inline __attribute__((always_inline)) int is_normal_idx(int idx) */
/* { */
/*  return (idx == 2); */
/* } */







/* static inline __attribute__((always_inline)) int is_highmem(struct zone *zone) */
/* { */
/*  return zone == zone->zone_pgdat->node_zones + 3; */
/* } */

/* static inline __attribute__((always_inline)) int is_normal(struct zone *zone) */
/* { */
/*  return zone == zone->zone_pgdat->node_zones + 2; */
/* } */

/* static inline __attribute__((always_inline)) int is_dma32(struct zone *zone) */
/* { */
/*  return zone == zone->zone_pgdat->node_zones + 1; */
/* } */

/* static inline __attribute__((always_inline)) int is_dma(struct zone *zone) */
/* { */
/*  return zone == zone->zone_pgdat->node_zones + 0; */
/* } */


/* struct ctl_table; */
/* struct file; */
/* int min_free_kbytes_sysctl_handler(struct ctl_table *, int, struct file *, */
/*      void *, size_t *, loff_t *); */
/* extern int sysctl_lowmem_reserve_ratio[4 -1]; */
/* int lowmem_reserve_ratio_sysctl_handler(struct ctl_table *, int, struct file *, */
/*      void *, size_t *, loff_t *); */
/* int percpu_pagelist_fraction_sysctl_handler(struct ctl_table *, int, struct file *, */
/*      void *, size_t *, loff_t *); */
/* int sysctl_min_unmapped_ratio_sysctl_handler(struct ctl_table *, int, */
/*    struct file *, void *, size_t *, loff_t *); */
/* int sysctl_min_slab_ratio_sysctl_handler(struct ctl_table *, int, */
/*    struct file *, void *, size_t *, loff_t *); */










/* extern cpumask_t cpu_coregroup_map(int cpu); */









/* extern struct pglist_data contig_page_data; */

/* extern struct pglist_data *first_online_pgdat(void); */
/* extern struct pglist_data *next_online_pgdat(struct pglist_data *pgdat); */
/* extern struct zone *next_zone(struct zone *zone); */

/* void memory_present(int nid, unsigned long start, unsigned long end); */
/* unsigned long __attribute__ ((__section__ (".init.text"))) node_memmap_size_bytes(int, unsigned long, unsigned long); */




/* struct vm_area_struct; */

/* static inline __attribute__((always_inline)) int gfp_zone(gfp_t gfp) */
/* { */
/*  int zone = 0x07 & ( int) gfp; */
/*  do { if (__builtin_expect(!!((zone >= ((0x07 + 1) / 2 + 1))!=0), 0)) __asm__ __volatile__( "ud2\n" "\t.word %c0\n" "\t.long %c1\n" : : "i" (82), "i" ("include/linux/gfp.h")); } while(0); */
/*  return zone; */
/* } */

/* static inline __attribute__((always_inline)) void arch_free_page(struct page *page, int order) { } */


/* extern struct page * */
/* __alloc_pages(gfp_t, unsigned int, struct zonelist *) __attribute__((regparm(3))); */

/* static inline __attribute__((always_inline)) struct page *alloc_pages_node(int nid, gfp_t gfp_mask, */
/*       unsigned int order) */
/* { */
/*  if (__builtin_expect(!!(order >= 11), 0)) */
/*   return ((void *)0); */


/*  if (nid < 0) */
/*   nid = ((0)); */

/*  return __alloc_pages(gfp_mask, order, */
/*   (&contig_page_data)->node_zonelists + gfp_zone(gfp_mask)); */
/* } */

/* extern unsigned long __get_free_pages(gfp_t gfp_mask, unsigned int order) __attribute__((regparm(3))); */
/* extern unsigned long get_zeroed_page(gfp_t gfp_mask) __attribute__((regparm(3))); */







/* extern void __free_pages(struct page *page, unsigned int order) __attribute__((regparm(3))); */
/* extern void free_pages(unsigned long addr, unsigned int order) __attribute__((regparm(3))); */
/* extern void free_hot_page(struct page *page) __attribute__((regparm(3))); */
/* extern void free_cold_page(struct page *page) __attribute__((regparm(3))); */




/* void page_alloc_init(void); */



/* static inline __attribute__((always_inline)) void drain_node_pages(int node) { }; */


/* extern void __attribute__ ((__section__ (".init.text"))) kmem_cache_init(void); */

/* extern kmem_cache_t *kmem_cache_create(const char *, size_t, size_t, unsigned long, */
/*            void (*)(void *, kmem_cache_t *, unsigned long), */
/*            void (*)(void *, kmem_cache_t *, unsigned long)); */
/* extern int kmem_cache_destroy(kmem_cache_t *); */
/* extern int kmem_cache_shrink(kmem_cache_t *); */
/* extern void *kmem_cache_alloc(kmem_cache_t *, gfp_t); */
/* extern void *kmem_cache_zalloc(struct kmem_cache *, gfp_t); */
/* extern void kmem_cache_free(kmem_cache_t *, void *); */
/* extern unsigned int kmem_cache_size(kmem_cache_t *); */
/* extern const char *kmem_cache_name(kmem_cache_t *); */
/* extern kmem_cache_t *kmem_find_general_cachep(size_t size, gfp_t gfpflags); */


/* struct cache_sizes { */
/*  size_t cs_size; */
/*  kmem_cache_t *cs_cachep; */
/*  kmem_cache_t *cs_dmacachep; */
/* }; */
/* extern struct cache_sizes malloc_sizes[]; */

/* extern void *__kmalloc(size_t, gfp_t); */

/* static inline __attribute__((always_inline)) void *kmalloc(size_t size, gfp_t flags) */
/* { */
/*  if (__builtin_constant_p(size)) { */
/*   int i = 0; */







/*  if (size <= 32) goto found; else i++; */

/*  if (size <= 64) goto found; else i++; */



/*  if (size <= 128) goto found; else i++; */



/*  if (size <= 256) goto found; else i++; */
/*  if (size <= 512) goto found; else i++; */
/*  if (size <= 1024) goto found; else i++; */
/*  if (size <= 2048) goto found; else i++; */
/*  if (size <= 4096) goto found; else i++; */
/*  if (size <= 8192) goto found; else i++; */
/*  if (size <= 16384) goto found; else i++; */
/*  if (size <= 32768) goto found; else i++; */
/*  if (size <= 65536) goto found; else i++; */
/*  if (size <= 131072) goto found; else i++; */


/*   { */
/*    extern void __you_cannot_kmalloc_that_much(void); */
/*    __you_cannot_kmalloc_that_much(); */
/*   } */
/* found: */
/*   return kmem_cache_alloc((flags & (( gfp_t)0x01u)) ? */
/*    malloc_sizes[i].cs_dmacachep : */
/*    malloc_sizes[i].cs_cachep, flags); */
/*  } */
/*  return __kmalloc(size, flags); */
/* } */

/* extern void *__kzalloc(size_t, gfp_t); */






/* static inline __attribute__((always_inline)) void *kzalloc(size_t size, gfp_t flags) */
/* { */
/*  if (__builtin_constant_p(size)) { */
/*   int i = 0; */







/*  if (size <= 32) goto found; else i++; */

/*  if (size <= 64) goto found; else i++; */



/*  if (size <= 128) goto found; else i++; */



/*  if (size <= 256) goto found; else i++; */
/*  if (size <= 512) goto found; else i++; */
/*  if (size <= 1024) goto found; else i++; */
/*  if (size <= 2048) goto found; else i++; */
/*  if (size <= 4096) goto found; else i++; */
/*  if (size <= 8192) goto found; else i++; */
/*  if (size <= 16384) goto found; else i++; */
/*  if (size <= 32768) goto found; else i++; */
/*  if (size <= 65536) goto found; else i++; */
/*  if (size <= 131072) goto found; else i++; */


/*   { */
/*    extern void __you_cannot_kzalloc_that_much(void); */
/*    __you_cannot_kzalloc_that_much(); */
/*   } */
/* found: */
/*   return kmem_cache_zalloc((flags & (( gfp_t)0x01u)) ? */
/*    malloc_sizes[i].cs_dmacachep : */
/*    malloc_sizes[i].cs_cachep, flags); */
/*  } */
/*  return __kzalloc(size, flags); */
/* } */







/* static inline __attribute__((always_inline)) void *kcalloc(size_t n, size_t size, gfp_t flags) */
/* { */
/*  if (n != 0 && size > (~0UL) / n) */
/*   return ((void *)0); */
/*  return kzalloc(n * size, flags); */
/* } */

/* extern void kfree(const void *); */
/* extern unsigned int ksize(const void *); */
/* extern int slab_is_available(void); */





/* static inline __attribute__((always_inline)) void *kmem_cache_alloc_node(kmem_cache_t *cachep, gfp_t flags, int node) */
/* { */
/*  return kmem_cache_alloc(cachep, flags); */
/* } */
/* static inline __attribute__((always_inline)) void *kmalloc_node(size_t size, gfp_t flags, int node) */
/* { */
/*  return kmalloc(size, flags); */
/* } */


/* extern int kmem_cache_reap(int) __attribute__((regparm(3))); */
/* extern int kmem_ptr_validate(kmem_cache_t *cachep, void *ptr) __attribute__((regparm(3))); */

/* extern kmem_cache_t *vm_area_cachep; */
/* extern kmem_cache_t *names_cachep; */
/* extern kmem_cache_t *files_cachep; */
/* extern kmem_cache_t *filp_cachep; */
/* extern kmem_cache_t *fs_cachep; */
/* extern kmem_cache_t *sighand_cachep; */
/* extern kmem_cache_t *bio_cachep; */

/* extern atomic_t slab_reclaim_pages; */


/* static inline __attribute__((always_inline)) void *__alloc_percpu(size_t size) */
/* { */
/*  void *ret = kmalloc(size, ((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u))); */
/*  if (ret) */
/*   (__builtin_constant_p(0) ? (__builtin_constant_p((size)) ? __constant_c_and_count_memset(((ret)),((0x01010101UL*(unsigned char)(0))),((size))) : __constant_c_memset(((ret)),((0x01010101UL*(unsigned char)(0))),((size)))) : (__builtin_constant_p((size)) ? __memset_generic((((ret))),(((0))),(((size)))) : __memset_generic(((ret)),((0)),((size))))); */
/*  return ret; */
/* } */
/* static inline __attribute__((always_inline)) void free_percpu(const void *ptr) */
/* { */
/*  kfree(ptr); */
/* } */


/* struct rcu_head { */
/*  struct rcu_head *next; */
/*  void (*func)(struct rcu_head *head); */
/* }; */

/* struct rcu_ctrlblk { */
/*  long cur; */
/*  long completed; */
/*  int next_pending; */

/*  spinlock_t lock ; */
/*  cpumask_t cpumask; */

/* } ; */


/* static inline __attribute__((always_inline)) int rcu_batch_before(long a, long b) */
/* { */
/*         return (a - b) < 0; */
/* } */


/* static inline __attribute__((always_inline)) int rcu_batch_after(long a, long b) */
/* { */
/*         return (a - b) > 0; */
/* } */






/* struct rcu_data { */

/*  long quiescbatch; */
/*  int passed_quiesc; */
/*  int qs_pending; */


/*  long batch; */
/*  struct rcu_head *nxtlist; */
/*  struct rcu_head **nxttail; */
/*  long qlen; */
/*  struct rcu_head *curlist; */
/*  struct rcu_head **curtail; */
/*  struct rcu_head *donelist; */
/*  struct rcu_head **donetail; */
/*  long blimit; */
/*  int cpu; */
/*  struct rcu_head barrier; */



/* }; */

/* extern __typeof__(struct rcu_data) per_cpu__rcu_data; */
/* extern __typeof__(struct rcu_data) per_cpu__rcu_bh_data; */







/* static inline __attribute__((always_inline)) void rcu_qsctr_inc(int cpu) */
/* { */
/*  struct rcu_data *rdp = &(*((void)(cpu), &per_cpu__rcu_data)); */
/*  rdp->passed_quiesc = 1; */
/* } */
/* static inline __attribute__((always_inline)) void rcu_bh_qsctr_inc(int cpu) */
/* { */
/*  struct rcu_data *rdp = &(*((void)(cpu), &per_cpu__rcu_bh_data)); */
/*  rdp->passed_quiesc = 1; */
/* } */

/* extern int rcu_pending(int cpu); */
/* extern int rcu_needs_cpu(int cpu); */

/* extern void rcu_init(void); */
/* extern void rcu_check_callbacks(int cpu, int user); */
/* extern void rcu_restart_cpu(int cpu); */
/* extern long rcu_batches_completed(void); */
/* extern long rcu_batches_completed_bh(void); */


/* extern void call_rcu(struct rcu_head *head, void (*func)(struct rcu_head *head)) __attribute__((regparm(3))); */

/* extern void call_rcu_bh(struct rcu_head *head, void (*func)(struct rcu_head *head)) __attribute__((regparm(3))); */

/* extern void synchronize_rcu(void); */
/* void synchronize_idle(void); */
/* extern void rcu_barrier(void); */


/* enum pid_type */
/* { */
/*  PIDTYPE_PID, */
/*  PIDTYPE_PGID, */
/*  PIDTYPE_SID, */
/*  PIDTYPE_MAX */
/* }; */

/* struct pid */
/* { */
/*  atomic_t count; */

/*  int nr; */
/*  struct hlist_node pid_chain; */

/*  struct hlist_head tasks[PIDTYPE_MAX]; */
/*  struct rcu_head rcu; */
/* }; */

/* struct pid_link */
/* { */
/*  struct hlist_node node; */
/*  struct pid *pid; */
/* }; */

/* static inline __attribute__((always_inline)) struct pid *get_pid(struct pid *pid) */
/* { */
/*  if (pid) */
/*   atomic_inc(&pid->count); */
/*  return pid; */
/* } */

/* extern void put_pid(struct pid *pid) __attribute__((regparm(3))); */
/* extern struct task_struct *pid_task(struct pid *pid, enum pid_type) __attribute__((regparm(3))); */
/* extern struct task_struct *get_pid_task(struct pid *pid, enum pid_type) __attribute__((regparm(3))); */






/* extern int attach_pid(struct task_struct *task, enum pid_type type, int nr) __attribute__((regparm(3))); */


/* extern void detach_pid(struct task_struct *task, enum pid_type) __attribute__((regparm(3))); */





/* extern struct pid *find_pid(int nr) __attribute__((regparm(3))); */




/* extern struct pid *find_get_pid(int nr); */

/* extern struct pid *alloc_pid(void); */
/* extern void free_pid(struct pid *pid) __attribute__((regparm(3))); */





/* typedef struct { } seccomp_t; */



/* static inline __attribute__((always_inline)) int has_secure_computing(struct thread_info *ti) */
/* { */
/*  return 0; */
/* } */









/* struct robust_list { */
/*  struct robust_list *next; */
/* }; */

/* struct robust_list_head { */



/*  struct robust_list list; */







/*  long futex_offset; */

/*  struct robust_list *list_op_pending; */
/* }; */

/* long do_futex(u32 *uaddr, int op, u32 val, unsigned long timeout, */
/*        u32 *uaddr2, u32 val2, u32 val3); */

/* extern int */
/* handle_futex_death(u32 *uaddr, struct task_struct *curr, int pi); */


/* extern void exit_robust_list(struct task_struct *curr); */
/* extern void exit_pi_state_list(struct task_struct *curr); */





/* struct plist_head { */
/*  struct list_head prio_list; */
/*  struct list_head node_list; */



/* }; */

/* struct plist_node { */
/*  int prio; */
/*  struct plist_head plist; */
/* }; */

/* static inline __attribute__((always_inline)) void */
/* plist_head_init(struct plist_head *head, spinlock_t *lock) */
/* { */
/*  INIT_LIST_HEAD(&head->prio_list); */
/*  INIT_LIST_HEAD(&head->node_list); */



/* } */







/* static inline __attribute__((always_inline)) void plist_node_init(struct plist_node *node, int prio) */
/* { */
/*  node->prio = prio; */
/*  plist_head_init(&node->plist, ((void *)0)); */
/* } */

/* extern void plist_add(struct plist_node *node, struct plist_head *head); */
/* extern void plist_del(struct plist_node *node, struct plist_head *head); */

/* static inline __attribute__((always_inline)) int plist_head_empty(const struct plist_head *head) */
/* { */
/*  return list_empty(&head->node_list); */
/* } */






/* static inline __attribute__((always_inline)) int plist_node_empty(const struct plist_node *node) */
/* { */
/*  return plist_head_empty(&node->plist); */
/* } */

/* static inline __attribute__((always_inline)) struct plist_node* plist_first(const struct plist_head *head) */
/* { */
/*  return ({ const typeof( ((struct plist_node *)0)->plist.node_list ) *__mptr = (head->node_list.next); (struct plist_node *)( (char *)__mptr - __builtin_offsetof(struct plist_node,plist.node_list) );}); */

/* } */


/* struct rt_mutex { */
/*  spinlock_t wait_lock; */
/*  struct plist_head wait_list; */
/*  struct task_struct *owner; */






/* }; */

/* struct rt_mutex_waiter; */
/* struct hrtimer_sleeper; */






/*  static inline __attribute__((always_inline)) int rt_mutex_debug_check_no_locks_freed(const void *from, */
/*              unsigned long len) */
/*  { */
/*  return 0; */
/*  } */

/* static inline __attribute__((always_inline)) int rt_mutex_is_locked(struct rt_mutex *lock) */
/* { */
/*  return lock->owner != ((void *)0); */
/* } */

/* extern void __rt_mutex_init(struct rt_mutex *lock, const char *name); */
/* extern void rt_mutex_destroy(struct rt_mutex *lock); */

/* extern void rt_mutex_lock(struct rt_mutex *lock); */
/* extern int rt_mutex_lock_interruptible(struct rt_mutex *lock, */
/*       int detect_deadlock); */
/* extern int rt_mutex_timed_lock(struct rt_mutex *lock, */
/*      struct hrtimer_sleeper *timeout, */
/*      int detect_deadlock); */

/* extern int rt_mutex_trylock(struct rt_mutex *lock); */

/* extern void rt_mutex_unlock(struct rt_mutex *lock); */











/* struct task_struct; */

/* struct rusage { */
/*  struct timeval ru_utime; */
/*  struct timeval ru_stime; */
/*  long ru_maxrss; */
/*  long ru_ixrss; */
/*  long ru_idrss; */
/*  long ru_isrss; */
/*  long ru_minflt; */
/*  long ru_majflt; */
/*  long ru_nswap; */
/*  long ru_inblock; */
/*  long ru_oublock; */
/*  long ru_msgsnd; */
/*  long ru_msgrcv; */
/*  long ru_nsignals; */
/*  long ru_nvcsw; */
/*  long ru_nivcsw; */
/* }; */

/* struct rlimit { */
/*  unsigned long rlim_cur; */
/*  unsigned long rlim_max; */
/* }; */









/* int getrusage(struct task_struct *p, int who, struct rusage *ru); */









/* struct tvec_t_base_s; */

/* struct timer_list { */
/*  struct list_head entry; */
/*  unsigned long expires; */

/*  void (*function)(unsigned long); */
/*  unsigned long data; */

/*  struct tvec_t_base_s *base; */
/* }; */

/* extern struct tvec_t_base_s boot_tvec_bases; */

/* void __attribute__((regparm(3))) init_timer(struct timer_list * timer); */

/* static inline __attribute__((always_inline)) void setup_timer(struct timer_list * timer, */
/*     void (*function)(unsigned long), */
/*     unsigned long data) */
/* { */
/*  timer->function = function; */
/*  timer->data = data; */
/*  init_timer(timer); */
/* } */

/* static inline __attribute__((always_inline)) int timer_pending(const struct timer_list * timer) */
/* { */
/*  return timer->entry.next != ((void *)0); */
/* } */

/* extern void add_timer_on(struct timer_list *timer, int cpu); */
/* extern int del_timer(struct timer_list * timer); */
/* extern int __mod_timer(struct timer_list *timer, unsigned long expires); */
/* extern int mod_timer(struct timer_list *timer, unsigned long expires); */

/* extern unsigned long next_timer_interrupt(void); */

/* static inline __attribute__((always_inline)) void add_timer(struct timer_list *timer) */
/* { */
/*  do { if (__builtin_expect(!!((timer_pending(timer))!=0), 0)) __asm__ __volatile__( "ud2\n" "\t.word %c0\n" "\t.long %c1\n" : : "i" (82), "i" ("include/linux/timer.h")); } while(0); */
/*  __mod_timer(timer, timer->expires); */
/* } */

/* extern void init_timers(void); */
/* extern void run_local_timers(void); */
/* struct hrtimer; */
/* extern int it_real_fn(struct hrtimer *); */





/* typedef union { */
/*  s64 tv64; */

/* } ktime_t; */

/* static inline __attribute__((always_inline)) ktime_t ktime_set(const long secs, const unsigned long nsecs) */
/* { */




/*  return (ktime_t) { .tv64 = (s64)secs * 1000000000L + (s64)nsecs }; */
/* } */

/* static inline __attribute__((always_inline)) ktime_t timespec_to_ktime(struct timespec ts) */
/* { */
/*  return ktime_set(ts.tv_sec, ts.tv_nsec); */
/* } */


/* static inline __attribute__((always_inline)) ktime_t timeval_to_ktime(struct timeval tv) */
/* { */
/*  return ktime_set(tv.tv_sec, tv.tv_usec * 1000L); */
/* } */

/* extern void ktime_get_ts(struct timespec *ts); */








/* enum hrtimer_mode { */
/*  HRTIMER_ABS, */
/*  HRTIMER_REL, */
/* }; */

/* enum hrtimer_restart { */
/*  HRTIMER_NORESTART, */
/*  HRTIMER_RESTART, */
/* }; */



/* struct hrtimer_base; */

/* struct hrtimer { */
/*  struct rb_node node; */
/*  ktime_t expires; */
/*  int (*function)(struct hrtimer *); */
/*  struct hrtimer_base *base; */
/* }; */

/* struct hrtimer_sleeper { */
/*  struct hrtimer timer; */
/*  struct task_struct *task; */
/* }; */

/* struct hrtimer_base { */
/*  clockid_t index; */
/*  spinlock_t lock; */
/*  struct rb_root active; */
/*  struct rb_node *first; */
/*  ktime_t resolution; */
/*  ktime_t (*get_time)(void); */
/*  ktime_t (*get_softirq_time)(void); */
/*  struct hrtimer *curr_timer; */
/*  ktime_t softirq_time; */
/*  struct lock_class_key lock_key; */
/* }; */

/* extern void hrtimer_init(struct hrtimer *timer, clockid_t which_clock, */
/*     enum hrtimer_mode mode); */


/* extern int hrtimer_start(struct hrtimer *timer, ktime_t tim, */
/*     const enum hrtimer_mode mode); */
/* extern int hrtimer_cancel(struct hrtimer *timer); */
/* extern int hrtimer_try_to_cancel(struct hrtimer *timer); */




/* extern ktime_t hrtimer_get_remaining(const struct hrtimer *timer); */
/* extern int hrtimer_get_res(const clockid_t which_clock, struct timespec *tp); */





/* static inline __attribute__((always_inline)) int hrtimer_active(const struct hrtimer *timer) */
/* { */
/*  return ((struct rb_node *)((&timer->node)->rb_parent_color & ~3)) != &timer->node; */
/* } */


/* extern unsigned long */
/* hrtimer_forward(struct hrtimer *timer, ktime_t now, ktime_t interval); */


/* extern long hrtimer_nanosleep(struct timespec *rqtp, */
/*          struct timespec *rmtp, */
/*          const enum hrtimer_mode mode, */
/*          const clockid_t clockid); */

/* extern void hrtimer_init_sleeper(struct hrtimer_sleeper *sl, */
/*      struct task_struct *tsk); */


/* extern void hrtimer_run_queues(void); */


/* extern void __attribute__ ((__section__ (".init.text"))) hrtimers_init(void); */




/* struct exec_domain; */
/* struct futex_pi_state; */

/* extern unsigned long avenrun[]; */

/* extern unsigned long total_forks; */
/* extern int nr_threads; */
/* extern int last_pid; */
/* extern __typeof__(unsigned long) per_cpu__process_counts; */
/* extern int nr_processes(void); */
/* extern unsigned long nr_running(void); */
/* extern unsigned long nr_uninterruptible(void); */
/* extern unsigned long nr_active(void); */
/* extern unsigned long nr_iowait(void); */
/* extern unsigned long weighted_cpuload(const int cpu); */

/* extern rwlock_t tasklist_lock; */
/* extern spinlock_t mmlist_lock; */

/* struct task_struct; */

/* extern void sched_init(void); */
/* extern void sched_init_smp(void); */
/* extern void init_idle(struct task_struct *idle, int cpu); */

/* extern cpumask_t nohz_cpu_mask; */

/* extern void show_state(void); */
/* extern void show_regs(struct pt_regs *); */






/* extern void show_stack(struct task_struct *task, unsigned long *sp); */

/* void io_schedule(void); */
/* long io_schedule_timeout(long timeout); */

/* extern void cpu_init (void); */
/* extern void trap_init(void); */
/* extern void update_process_times(int user); */
/* extern void scheduler_tick(void); */


/* extern void softlockup_tick(void); */
/* extern void spawn_softlockup_task(void); */
/* extern void touch_softlockup_watchdog(void); */

/* extern int in_sched_functions(unsigned long addr); */


/* extern signed long schedule_timeout(signed long timeout) __attribute__((regparm(3))); */
/* extern signed long schedule_timeout_interruptible(signed long timeout); */
/* extern signed long schedule_timeout_uninterruptible(signed long timeout); */
/*  __attribute__((regparm(0))) void schedule(void); */

/* struct namespace; */




/* extern int sysctl_max_map_count; */








/* struct workqueue_struct; */

/* struct work_struct { */
/*  unsigned long pending; */
/*  struct list_head entry; */
/*  void (*func)(void *); */
/*  void *data; */
/*  void *wq_data; */
/*  struct timer_list timer; */
/* }; */

/* struct execute_work { */
/*  struct work_struct work; */
/* }; */

/* extern struct workqueue_struct *__create_workqueue(const char *name, */
/*           int singlethread); */



/* extern void destroy_workqueue(struct workqueue_struct *wq); */

/* extern int queue_work(struct workqueue_struct *wq, struct work_struct *work) __attribute__((regparm(3))); */
/* extern int queue_delayed_work(struct workqueue_struct *wq, struct work_struct *work, unsigned long delay) __attribute__((regparm(3))); */
/* extern int queue_delayed_work_on(int cpu, struct workqueue_struct *wq, */
/*  struct work_struct *work, unsigned long delay); */
/* extern void flush_workqueue(struct workqueue_struct *wq) __attribute__((regparm(3))); */

/* extern int schedule_work(struct work_struct *work) __attribute__((regparm(3))); */
/* extern int schedule_delayed_work(struct work_struct *work, unsigned long delay) __attribute__((regparm(3))); */

/* extern int schedule_delayed_work_on(int cpu, struct work_struct *work, unsigned long delay); */
/* extern int schedule_on_each_cpu(void (*func)(void *info), void *info); */
/* extern void flush_scheduled_work(void); */
/* extern int current_is_keventd(void); */
/* extern int keventd_up(void); */

/* extern void init_workqueues(void); */
/* void cancel_rearming_delayed_work(struct work_struct *work); */
/* void cancel_rearming_delayed_workqueue(struct workqueue_struct *, */
/*            struct work_struct *); */
/* int execute_in_process_context(void (*fn)(void *), void *, */
/*           struct execute_work *); */






/* static inline __attribute__((always_inline)) int cancel_delayed_work(struct work_struct *work) */
/* { */
/*  int ret; */

/*  ret = del_timer(&work->timer); */
/*  if (ret) */
/*   clear_bit(0, &work->pending); */
/*  return ret; */
/* } */



/* typedef unsigned long aio_context_t; */

/* enum { */
/*  IOCB_CMD_PREAD = 0, */
/*  IOCB_CMD_PWRITE = 1, */
/*  IOCB_CMD_FSYNC = 2, */
/*  IOCB_CMD_FDSYNC = 3, */




/*  IOCB_CMD_NOOP = 6, */
/* }; */


/* struct io_event { */
/*  __u64 data; */
/*  __u64 obj; */
/*  __s64 res; */
/*  __s64 res2; */
/* }; */

/* struct iocb { */

/*  __u64 aio_data; */
/*  __u32 aio_key, aio_reserved1; */



/*  __u16 aio_lio_opcode; */
/*  __s16 aio_reqprio; */
/*  __u32 aio_fildes; */

/*  __u64 aio_buf; */
/*  __u64 aio_nbytes; */
/*  __s64 aio_offset; */


/*  __u64 aio_reserved2; */
/*  __u64 aio_reserved3; */
/* }; */







/* struct kioctx; */

/* struct kiocb { */
/*  struct list_head ki_run_list; */
/*  long ki_flags; */
/*  int ki_users; */
/*  unsigned ki_key; */

/*  struct file *ki_filp; */
/*  struct kioctx *ki_ctx; */
/*  int (*ki_cancel)(struct kiocb *, struct io_event *); */
/*  ssize_t (*ki_retry)(struct kiocb *); */
/*  void (*ki_dtor)(struct kiocb *); */

/*  union { */
/*   void *user; */
/*   struct task_struct *tsk; */
/*  } ki_obj; */

/*  __u64 ki_user_data; */
/*  wait_queue_t ki_wait; */
/*  loff_t ki_pos; */

/*  void *private; */

/*  unsigned short ki_opcode; */
/*  size_t ki_nbytes; */
/*  char *ki_buf; */
/*  size_t ki_left; */
/*  long ki_retried; */
/*  long ki_kicked; */
/*  long ki_queued; */

/*  struct list_head ki_list; */

/* }; */

/* struct aio_ring { */
/*  unsigned id; */
/*  unsigned nr; */
/*  unsigned head; */
/*  unsigned tail; */

/*  unsigned magic; */
/*  unsigned compat_features; */
/*  unsigned incompat_features; */
/*  unsigned header_length; */


/*  struct io_event io_events[0]; */
/* }; */




/* struct aio_ring_info { */
/*  unsigned long mmap_base; */
/*  unsigned long mmap_size; */

/*  struct page **ring_pages; */
/*  spinlock_t ring_lock; */
/*  long nr_pages; */

/*  unsigned nr, tail; */

/*  struct page *internal_pages[8]; */
/* }; */

/* struct kioctx { */
/*  atomic_t users; */
/*  int dead; */
/*  struct mm_struct *mm; */


/*  unsigned long user_id; */
/*  struct kioctx *next; */

/*  wait_queue_head_t wait; */

/*  spinlock_t ctx_lock; */

/*  int reqs_active; */
/*  struct list_head active_reqs; */
/*  struct list_head run_list; */


/*  unsigned max_reqs; */

/*  struct aio_ring_info ring_info; */

/*  struct work_struct wq; */
/* }; */


/* extern unsigned aio_max_size; */

/* extern ssize_t wait_on_sync_kiocb(struct kiocb *iocb) __attribute__((regparm(3))); */
/* extern int aio_put_req(struct kiocb *iocb) __attribute__((regparm(3))); */
/* extern void kick_iocb(struct kiocb *iocb) __attribute__((regparm(3))); */
/* extern int aio_complete(struct kiocb *iocb, long res, long res2) __attribute__((regparm(3))); */
/* extern void __put_ioctx(struct kioctx *ctx) __attribute__((regparm(3))); */
/* struct mm_struct; */
/* extern void exit_aio(struct mm_struct *mm) __attribute__((regparm(3))); */
/* extern struct kioctx *lookup_ioctx(unsigned long ctx_id); */
/* extern int io_submit_one(struct kioctx *ctx, struct iocb *user_iocb, struct iocb *iocb) __attribute__((regparm(3))); */



/* struct kioctx *lookup_ioctx(unsigned long ctx_id); */
/* int io_submit_one(struct kioctx *ctx, struct iocb *user_iocb, struct iocb *iocb) __attribute__((regparm(3))); */

/* static inline __attribute__((always_inline)) struct kiocb *list_kiocb(struct list_head *h) */
/* { */
/*  return ({ const typeof( ((struct kiocb *)0)->ki_list ) *__mptr = (h); (struct kiocb *)( (char *)__mptr - __builtin_offsetof(struct kiocb,ki_list) );}); */
/* } */


/* extern unsigned long aio_nr; */
/* extern unsigned long aio_max_nr; */


/* extern unsigned long */
/* arch_get_unmapped_area(struct file *, unsigned long, unsigned long, */
/*          unsigned long, unsigned long); */
/* extern unsigned long */
/* arch_get_unmapped_area_topdown(struct file *filp, unsigned long addr, */
/*      unsigned long len, unsigned long pgoff, */
/*      unsigned long flags); */
/* extern void arch_unmap_area(struct mm_struct *, unsigned long); */
/* extern void arch_unmap_area_topdown(struct mm_struct *, unsigned long); */

/* typedef unsigned long mm_counter_t; */

/* struct mm_struct { */
/*  struct vm_area_struct * mmap; */
/*  struct rb_root mm_rb; */
/*  struct vm_area_struct * mmap_cache; */
/*  unsigned long (*get_unmapped_area) (struct file *filp, */
/*     unsigned long addr, unsigned long len, */
/*     unsigned long pgoff, unsigned long flags); */
/*  void (*unmap_area) (struct mm_struct *mm, unsigned long addr); */
/*  unsigned long mmap_base; */
/*  unsigned long task_size; */
/*  unsigned long cached_hole_size; */
/*  unsigned long free_area_cache; */
/*  pgd_t * pgd; */
/*  atomic_t mm_users; */
/*  atomic_t mm_count; */
/*  int map_count; */
/*  struct rw_semaphore mmap_sem; */
/*  spinlock_t page_table_lock; */

/*  struct list_head mmlist; */







/*  mm_counter_t _file_rss; */
/*  mm_counter_t _anon_rss; */

/*  unsigned long hiwater_rss; */
/*  unsigned long hiwater_vm; */

/*  unsigned long total_vm, locked_vm, shared_vm, exec_vm; */
/*  unsigned long stack_vm, reserved_vm, def_flags, nr_ptes; */
/*  unsigned long start_code, end_code, start_data, end_data; */
/*  unsigned long start_brk, brk, start_stack; */
/*  unsigned long arg_start, arg_end, env_start, env_end; */

/*  unsigned long saved_auxv[44]; */

/*  unsigned dumpable:2; */
/*  cpumask_t cpu_vm_mask; */


/*  mm_context_t context; */


/*  unsigned long swap_token_time; */
/*  char recent_pagein; */


/*  int core_waiters; */
/*  struct completion *core_startup_done, core_done; */


/*  rwlock_t ioctx_list_lock; */
/*  struct kioctx *ioctx_list; */
/* }; */

/* struct sighand_struct { */
/*  atomic_t count; */
/*  struct k_sigaction action[64]; */
/*  spinlock_t siglock; */
/* }; */

/* struct pacct_struct { */
/*  int ac_flag; */
/*  long ac_exitcode; */
/*  unsigned long ac_mem; */
/*  cputime_t ac_utime, ac_stime; */
/*  unsigned long ac_minflt, ac_majflt; */
/* }; */

/* struct signal_struct { */
/*  atomic_t count; */
/*  atomic_t live; */

/*  wait_queue_head_t wait_chldexit; */


/*  struct task_struct *curr_target; */


/*  struct sigpending shared_pending; */


/*  int group_exit_code; */





/*  struct task_struct *group_exit_task; */
/*  int notify_count; */


/*  int group_stop_count; */
/*  unsigned int flags; */


/*  struct list_head posix_timers; */


/*  struct hrtimer real_timer; */
/*  struct task_struct *tsk; */
/*  ktime_t it_real_incr; */


/*  cputime_t it_prof_expires, it_virt_expires; */
/*  cputime_t it_prof_incr, it_virt_incr; */


/*  pid_t pgrp; */
/*  pid_t tty_old_pgrp; */
/*  pid_t session; */

/*  int leader; */

/*  struct tty_struct *tty; */







/*  cputime_t utime, stime, cutime, cstime; */
/*  unsigned long nvcsw, nivcsw, cnvcsw, cnivcsw; */
/*  unsigned long min_flt, maj_flt, cmin_flt, cmaj_flt; */







/*  unsigned long long sched_time; */

/*  struct rlimit rlim[15]; */

/*  struct list_head cpu_timers[3]; */




/*  struct key *session_keyring; */
/*  struct key *process_keyring; */


/*  struct pacct_struct pacct; */


/*  spinlock_t stats_lock; */
/*  struct taskstats *stats; */

/* }; */

/* struct user_struct { */
/*  atomic_t __count; */
/*  atomic_t processes; */
/*  atomic_t files; */
/*  atomic_t sigpending; */

/*  atomic_t inotify_watches; */
/*  atomic_t inotify_devs; */


/*  unsigned long mq_bytes; */
/*  unsigned long locked_shm; */


/*  struct key *uid_keyring; */
/*  struct key *session_keyring; */



/*  struct list_head uidhash_list; */
/*  uid_t uid; */
/* }; */

/* extern struct user_struct *find_user(uid_t); */

/* extern struct user_struct root_user; */


/* struct backing_dev_info; */
/* struct reclaim_state; */


/* struct sched_info { */

/*  unsigned long cpu_time, */
/*    run_delay, */
/*    pcnt; */


/*  unsigned long last_arrival, */
/*    last_queued; */
/* }; */



/* extern struct file_operations proc_schedstat_operations; */



/* struct task_delay_info { */
/*  spinlock_t lock; */
/*  unsigned int flags; */

/*  struct timespec blkio_start, blkio_end; */
/*  u64 blkio_delay; */
/*  u64 swapin_delay; */
/*  u32 blkio_count; */

/*  u32 swapin_count; */

/* }; */


/* static inline __attribute__((always_inline)) int sched_info_on(void) */
/* { */

/*  return 1; */






/* } */

/* enum idle_type */
/* { */
/*  SCHED_IDLE, */
/*  NOT_IDLE, */
/*  NEWLY_IDLE, */
/*  MAX_IDLE_TYPES */
/* }; */

/* struct io_context; */
/* void exit_io_context(void); */
/* struct cpuset; */



/* struct group_info { */
/*  int ngroups; */
/*  atomic_t usage; */
/*  gid_t small_block[32]; */
/*  int nblocks; */
/*  gid_t *blocks[0]; */
/* }; */

/* extern struct group_info *groups_alloc(int gidsetsize); */
/* extern void groups_free(struct group_info *group_info); */
/* extern int set_current_groups(struct group_info *group_info); */
/* extern int groups_search(struct group_info *group_info, gid_t grp); */







/* static inline __attribute__((always_inline)) void prefetch_stack(struct task_struct *t) { } */


/* struct audit_context; */
/* struct mempolicy; */
/* struct pipe_inode_info; */

/* enum sleep_type { */
/*  SLEEP_NORMAL, */
/*  SLEEP_NONINTERACTIVE, */
/*  SLEEP_INTERACTIVE, */
/*  SLEEP_INTERRUPTED, */
/* }; */

/* struct prio_array; */

/* struct task_struct { */
/*  volatile long state; */
/*  struct thread_info *thread_info; */
/*  atomic_t usage; */
/*  unsigned long flags; */
/*  unsigned long ptrace; */

/*  int lock_depth; */






/*  int load_weight; */
/*  int prio, static_prio, normal_prio; */
/*  struct list_head run_list; */
/*  struct prio_array *array; */

/*  unsigned short ioprio; */
/*  unsigned int btrace_seq; */

/*  unsigned long sleep_avg; */
/*  unsigned long long timestamp, last_ran; */
/*  unsigned long long sched_time; */
/*  enum sleep_type sleep_type; */

/*  unsigned long policy; */
/*  cpumask_t cpus_allowed; */
/*  unsigned int time_slice, first_time_slice; */


/*  struct sched_info sched_info; */


/*  struct list_head tasks; */




/*  struct list_head ptrace_children; */
/*  struct list_head ptrace_list; */

/*  struct mm_struct *mm, *active_mm; */


/*  struct linux_binfmt *binfmt; */
/*  long exit_state; */
/*  int exit_code, exit_signal; */
/*  int pdeath_signal; */

/*  unsigned long personality; */
/*  unsigned did_exec:1; */
/*  pid_t pid; */
/*  pid_t tgid; */





/*  struct task_struct *real_parent; */
/*  struct task_struct *parent; */




/*  struct list_head children; */
/*  struct list_head sibling; */
/*  struct task_struct *group_leader; */


/*  struct pid_link pids[PIDTYPE_MAX]; */
/*  struct list_head thread_group; */

/*  struct completion *vfork_done; */
/*  int *set_child_tid; */
/*  int *clear_child_tid; */

/*  unsigned long rt_priority; */
/*  cputime_t utime, stime; */
/*  unsigned long nvcsw, nivcsw; */
/*  struct timespec start_time; */

/*  unsigned long min_flt, maj_flt; */

/*    cputime_t it_prof_expires, it_virt_expires; */
/*  unsigned long long it_sched_expires; */
/*  struct list_head cpu_timers[3]; */


/*  uid_t uid,euid,suid,fsuid; */
/*  gid_t gid,egid,sgid,fsgid; */
/*  struct group_info *group_info; */
/*  kernel_cap_t cap_effective, cap_inheritable, cap_permitted; */
/*  unsigned keep_capabilities:1; */
/*  struct user_struct *user; */

/*  struct key *request_key_auth; */
/*  struct key *thread_keyring; */
/*  unsigned char jit_keyring; */

/*  int oomkilladj; */
/*  char comm[16]; */




/*  int link_count, total_link_count; */

/*  struct sysv_sem sysvsem; */

/*  struct thread_struct thread; */

/*  struct fs_struct *fs; */

/*  struct files_struct *files; */

/*  struct namespace *namespace; */

/*  struct signal_struct *signal; */
/*  struct sighand_struct *sighand; */

/*  sigset_t blocked, real_blocked; */
/*  sigset_t saved_sigmask; */
/*  struct sigpending pending; */

/*  unsigned long sas_ss_sp; */
/*  size_t sas_ss_size; */
/*  int (*notifier)(void *priv); */
/*  void *notifier_data; */
/*  sigset_t *notifier_mask; */

/*  void *security; */
/*  struct audit_context *audit_context; */
/*  seccomp_t seccomp; */


/*     u32 parent_exec_id; */
/*     u32 self_exec_id; */

/*  spinlock_t alloc_lock; */


/*  spinlock_t pi_lock; */



/*  struct plist_head pi_waiters; */

/*  struct rt_mutex_waiter *pi_blocked_on; */

/*  void *journal_info; */


/*  struct reclaim_state *reclaim_state; */

/*  struct backing_dev_info *backing_dev_info; */

/*  struct io_context *io_context; */

/*  unsigned long ptrace_message; */
/*  siginfo_t *last_siginfo; */






/*  wait_queue_t *io_wait; */

/*  u64 rchar, wchar, syscr, syscw; */

/*  u64 acct_rss_mem1; */
/*  u64 acct_vm_mem1; */
/*  clock_t acct_stimexpd; */

/*  struct robust_list_head *robust_list; */



/*  struct list_head pi_state_list; */
/*  struct futex_pi_state *pi_state_cache; */

/*  atomic_t fs_excl; */
/*  struct rcu_head rcu; */




/*  struct pipe_inode_info *splice_pipe; */

/*  struct task_delay_info *delays; */

/* }; */

/* static inline __attribute__((always_inline)) pid_t process_group(struct task_struct *tsk) */
/* { */
/*  return tsk->signal->pgrp; */
/* } */

/* static inline __attribute__((always_inline)) int pid_alive(struct task_struct *p) */
/* { */
/*  return p->pids[PIDTYPE_PID].pid != ((void *)0); */
/* } */

/* extern void free_task(struct task_struct *tsk); */


/* extern void __put_task_struct(struct task_struct *t); */

/* static inline __attribute__((always_inline)) void put_task_struct(struct task_struct *t) */
/* { */
/*  if (atomic_dec_and_test(&t->usage)) */
/*   __put_task_struct(t); */
/* } */

/* static inline __attribute__((always_inline)) int set_cpus_allowed(struct task_struct *p, cpumask_t new_mask) */
/* { */
/*  if (!(__builtin_constant_p((0)) ? constant_test_bit(((0)),((new_mask).bits)) : variable_test_bit(((0)),((new_mask).bits)))) */
/*   return -22; */
/*  return 0; */
/* } */


/* extern unsigned long long sched_clock(void); */
/* extern unsigned long long */
/* current_sched_time(const struct task_struct *current_task); */

/* static inline __attribute__((always_inline)) void idle_task_exit(void) {} */


/* extern void sched_idle_next(void); */


/* extern int rt_mutex_getprio(struct task_struct *p); */
/* extern void rt_mutex_setprio(struct task_struct *p, int prio); */
/* extern void rt_mutex_adjust_pi(struct task_struct *p); */

/* extern void set_user_nice(struct task_struct *p, long nice); */
/* extern int task_prio(const struct task_struct *p); */
/* extern int task_nice(const struct task_struct *p); */
/* extern int can_nice(const struct task_struct *p, const int nice); */
/* extern int task_curr(const struct task_struct *p); */
/* extern int idle_cpu(int cpu); */
/* extern int sched_setscheduler(struct task_struct *, int, struct sched_param *); */
/* extern struct task_struct *idle_task(int cpu); */
/* extern struct task_struct *curr_task(int cpu); */
/* extern void set_curr_task(int cpu, struct task_struct *p); */

/* void yield(void); */




/* extern struct exec_domain default_exec_domain; */

/* union thread_union { */
/*  struct thread_info thread_info; */
/*  unsigned long stack[(4096)/sizeof(long)]; */
/* }; */


/* static inline __attribute__((always_inline)) int kstack_end(void *addr) */
/* { */



/*  return !(((unsigned long)addr+sizeof(void*)-1) & ((4096)-sizeof(void*))); */
/* } */


/* extern union thread_union init_thread_union; */
/* extern struct task_struct init_task; */

/* extern struct mm_struct init_mm; */


/* extern struct task_struct *find_task_by_pid_type(int type, int pid); */
/* extern void set_special_pids(pid_t session, pid_t pgrp); */
/* extern void __set_special_pids(pid_t session, pid_t pgrp); */


/* extern struct user_struct * alloc_uid(uid_t); */
/* static inline __attribute__((always_inline)) struct user_struct *get_uid(struct user_struct *u) */
/* { */
/*  atomic_inc(&u->__count); */
/*  return u; */
/* } */
/* extern void free_uid(struct user_struct *); */
/* extern void switch_uid(struct user_struct *); */



/* extern void do_timer(struct pt_regs *); */

/* extern int wake_up_state(struct task_struct * tsk, unsigned int state) __attribute__((regparm(3))); */
/* extern int wake_up_process(struct task_struct * tsk) __attribute__((regparm(3))); */
/* extern void wake_up_new_task(struct task_struct * tsk, unsigned long clone_flags) __attribute__((regparm(3))); */




/*  static inline __attribute__((always_inline)) void kick_process(struct task_struct *tsk) { } */

/* extern void sched_fork(struct task_struct * p, int clone_flags) __attribute__((regparm(3))); */
/* extern void sched_exit(struct task_struct * p) __attribute__((regparm(3))); */

/* extern int in_group_p(gid_t); */
/* extern int in_egroup_p(gid_t); */

/* extern void proc_caches_init(void); */
/* extern void flush_signals(struct task_struct *); */
/* extern void flush_signal_handlers(struct task_struct *, int force_default); */
/* extern int dequeue_signal(struct task_struct *tsk, sigset_t *mask, siginfo_t *info); */

/* static inline __attribute__((always_inline)) int dequeue_signal_lock(struct task_struct *tsk, sigset_t *mask, siginfo_t *info) */
/* { */
/*  unsigned long flags; */
/*  int ret; */

/*  flags = _spin_lock_irqsave(&tsk->sighand->siglock); */
/*  ret = dequeue_signal(tsk, mask, info); */
/*  _spin_unlock_irqrestore(&tsk->sighand->siglock, flags); */

/*  return ret; */
/* } */

/* extern void block_all_signals(int (*notifier)(void *priv), void *priv, */
/*          sigset_t *mask); */
/* extern void unblock_all_signals(void); */
/* extern void release_task(struct task_struct * p); */
/* extern int send_sig_info(int, struct siginfo *, struct task_struct *); */
/* extern int send_group_sig_info(int, struct siginfo *, struct task_struct *); */
/* extern int force_sigsegv(int, struct task_struct *); */
/* extern int force_sig_info(int, struct siginfo *, struct task_struct *); */
/* extern int __kill_pg_info(int sig, struct siginfo *info, pid_t pgrp); */
/* extern int kill_pg_info(int, struct siginfo *, pid_t); */
/* extern int kill_proc_info(int, struct siginfo *, pid_t); */
/* extern int kill_proc_info_as_uid(int, struct siginfo *, pid_t, uid_t, uid_t, u32); */
/* extern void do_notify_parent(struct task_struct *, int); */
/* extern void force_sig(int, struct task_struct *); */
/* extern void force_sig_specific(int, struct task_struct *); */
/* extern int send_sig(int, struct task_struct *, int); */
/* extern void zap_other_threads(struct task_struct *p); */
/* extern int kill_pg(pid_t, int, int); */
/* extern int kill_proc(pid_t, int, int); */
/* extern struct sigqueue *sigqueue_alloc(void); */
/* extern void sigqueue_free(struct sigqueue *); */
/* extern int send_sigqueue(int, struct sigqueue *, struct task_struct *); */
/* extern int send_group_sigqueue(int, struct sigqueue *, struct task_struct *); */
/* extern int do_sigaction(int, struct k_sigaction *, struct k_sigaction *); */
/* extern int do_sigaltstack(const stack_t *, stack_t *, unsigned long); */






/* static inline __attribute__((always_inline)) int is_si_special(const struct siginfo *info) */
/* { */
/*  return info <= ((struct siginfo *) 2); */
/* } */



/* static inline __attribute__((always_inline)) int on_sig_stack(unsigned long sp) */
/* { */
/*  return (sp - __vericon_dummy_current->sas_ss_sp < __vericon_dummy_current->sas_ss_size); */
/* } */

/* static inline __attribute__((always_inline)) int sas_ss_flags(unsigned long sp) */
/* { */
/*  return (__vericon_dummy_current->sas_ss_size == 0 ? 2 */
/*   : on_sig_stack(sp) ? 1 : 0); */
/* } */




/* extern struct mm_struct * mm_alloc(void); */


/* extern void __mmdrop(struct mm_struct *) __attribute__((regparm(3))); */
/* static inline __attribute__((always_inline)) void mmdrop(struct mm_struct * mm) */
/* { */
/*  if (atomic_dec_and_test(&mm->mm_count)) */
/*   __mmdrop(mm); */
/* } */


/* extern void mmput(struct mm_struct *); */

/* extern struct mm_struct *get_task_mm(struct task_struct *task); */

/* extern void mm_release(struct task_struct *, struct mm_struct *); */

/* extern int copy_thread(int, unsigned long, unsigned long, unsigned long, struct task_struct *, struct pt_regs *); */
/* extern void flush_thread(void); */
/* extern void exit_thread(void); */

/* extern void exit_files(struct task_struct *); */
/* extern void __cleanup_signal(struct signal_struct *); */
/* extern void __cleanup_sighand(struct sighand_struct *); */
/* extern void exit_itimers(struct signal_struct *); */

/* extern void do_group_exit(int); */

/* extern void daemonize(const char *, ...); */
/* extern int allow_signal(int); */
/* extern int disallow_signal(int); */
/* extern struct task_struct *child_reaper; */

/* extern int do_execve(char *, char * *, char * *, struct pt_regs *); */
/* extern long do_fork(unsigned long, unsigned long, struct pt_regs *, unsigned long, int *, int *); */
/* struct task_struct *fork_idle(int); */

/* extern void set_task_comm(struct task_struct *tsk, char *from); */
/* extern void get_task_comm(char *to, struct task_struct *tsk); */

/* static inline __attribute__((always_inline)) struct task_struct *next_thread(const struct task_struct *p) */
/* { */
/*  return ({ const typeof( ((struct task_struct *)0)->thread_group ) *__mptr = (({ typeof(p->thread_group.next) _________p1 = p->thread_group.next; do { } while(0); (_________p1); })); (struct task_struct *)( (char *)__mptr - __builtin_offsetof(struct task_struct,thread_group) );}); */

/* } */

/* static inline __attribute__((always_inline)) int thread_group_empty(struct task_struct *p) */
/* { */
/*  return list_empty(&p->thread_group); */
/* } */

/* static inline __attribute__((always_inline)) void task_lock(struct task_struct *p) */
/* { */
/*  _spin_lock(&p->alloc_lock); */
/* } */

/* static inline __attribute__((always_inline)) void task_unlock(struct task_struct *p) */
/* { */
/*  _spin_unlock(&p->alloc_lock); */
/* } */

/* extern struct sighand_struct *lock_task_sighand(struct task_struct *tsk, */
/*        unsigned long *flags); */

/* static inline __attribute__((always_inline)) void unlock_task_sighand(struct task_struct *tsk, */
/*       unsigned long *flags) */
/* { */
/*  _spin_unlock_irqrestore(&tsk->sighand->siglock, *flags); */
/* } */






/* static inline __attribute__((always_inline)) void setup_thread_stack(struct task_struct *p, struct task_struct *org) */
/* { */
/*  *(p)->thread_info = *(org)->thread_info; */
/*  (p)->thread_info->task = p; */
/* } */

/* static inline __attribute__((always_inline)) unsigned long *end_of_stack(struct task_struct *p) */
/* { */
/*  return (unsigned long *)(p->thread_info + 1); */
/* } */






/* static inline __attribute__((always_inline)) void set_tsk_thread_flag(struct task_struct *tsk, int flag) */
/* { */
/*  set_ti_thread_flag((tsk)->thread_info, flag); */
/* } */

/* static inline __attribute__((always_inline)) void clear_tsk_thread_flag(struct task_struct *tsk, int flag) */
/* { */
/*  clear_ti_thread_flag((tsk)->thread_info, flag); */
/* } */

/* static inline __attribute__((always_inline)) int test_and_set_tsk_thread_flag(struct task_struct *tsk, int flag) */
/* { */
/*  return test_and_set_ti_thread_flag((tsk)->thread_info, flag); */
/* } */

/* static inline __attribute__((always_inline)) int test_and_clear_tsk_thread_flag(struct task_struct *tsk, int flag) */
/* { */
/*  return test_and_clear_ti_thread_flag((tsk)->thread_info, flag); */
/* } */

/* static inline __attribute__((always_inline)) int test_tsk_thread_flag(struct task_struct *tsk, int flag) */
/* { */
/*  return test_ti_thread_flag((tsk)->thread_info, flag); */
/* } */

/* static inline __attribute__((always_inline)) void set_tsk_need_resched(struct task_struct *tsk) */
/* { */
/*  set_tsk_thread_flag(tsk,3); */
/* } */

/* static inline __attribute__((always_inline)) void clear_tsk_need_resched(struct task_struct *tsk) */
/* { */
/*  clear_tsk_thread_flag(tsk,3); */
/* } */

/* static inline __attribute__((always_inline)) int signal_pending(struct task_struct *p) */
/* { */
/*  return __builtin_expect(!!(test_tsk_thread_flag(p,2)), 0); */
/* } */

/* static inline __attribute__((always_inline)) int need_resched(void) */
/* { */
/*  return __builtin_expect(!!(test_ti_thread_flag(current_thread_info(), 3)), 0); */
/* } */

/* extern int cond_resched(void); */
/* extern int cond_resched_lock(spinlock_t * lock); */
/* extern int cond_resched_softirq(void); */

/* static inline __attribute__((always_inline)) int lock_need_resched(spinlock_t *lock) */
/* { */
/*  if (0 || need_resched()) */
/*   return 1; */
/*  return 0; */
/* } */





/* extern void recalc_sigpending_tsk(struct task_struct *t) __attribute__((regparm(3))); */
/* extern void recalc_sigpending(void); */

/* extern void signal_wake_up(struct task_struct *t, int resume_stopped); */

/* static inline __attribute__((always_inline)) unsigned int task_cpu(const struct task_struct *p) */
/* { */
/*  return 0; */
/* } */

/* static inline __attribute__((always_inline)) void set_task_cpu(struct task_struct *p, unsigned int cpu) */
/* { */
/* } */




/* extern void arch_pick_mmap_layout(struct mm_struct *mm); */

/* extern long sched_setaffinity(pid_t pid, cpumask_t new_mask); */
/* extern long sched_getaffinity(pid_t pid, cpumask_t *mask); */







/* struct kobject; */
/* struct module; */

/* struct attribute { */
/*  const char * name; */
/*  struct module * owner; */
/*  mode_t mode; */
/* }; */

/* struct attribute_group { */
/*  const char * name; */
/*  struct attribute ** attrs; */
/* }; */

/* struct vm_area_struct; */

/* struct bin_attribute { */
/*  struct attribute attr; */
/*  size_t size; */
/*  void *private; */
/*  ssize_t (*read)(struct kobject *, char *, loff_t, size_t); */
/*  ssize_t (*write)(struct kobject *, char *, loff_t, size_t); */
/*  int (*mmap)(struct kobject *, struct bin_attribute *attr, */
/*       struct vm_area_struct *vma); */
/* }; */

/* struct sysfs_ops { */
/*  ssize_t (*show)(struct kobject *, struct attribute *,char *); */
/*  ssize_t (*store)(struct kobject *,struct attribute *,const char *, size_t); */
/* }; */

/* struct sysfs_dirent { */
/*  atomic_t s_count; */
/*  struct list_head s_sibling; */
/*  struct list_head s_children; */
/*  void * s_element; */
/*  int s_type; */
/*  umode_t s_mode; */
/*  struct dentry * s_dentry; */
/*  struct iattr * s_iattr; */
/*  atomic_t s_event; */
/* }; */

/* extern int */
/* sysfs_create_dir(struct kobject *); */

/* extern void */
/* sysfs_remove_dir(struct kobject *); */

/* extern int */
/* sysfs_rename_dir(struct kobject *, const char *new_name); */

/* extern int */
/* sysfs_create_file(struct kobject *, const struct attribute *); */

/* extern int */
/* sysfs_update_file(struct kobject *, const struct attribute *); */

/* extern int */
/* sysfs_chmod_file(struct kobject *kobj, struct attribute *attr, mode_t mode); */

/* extern void */
/* sysfs_remove_file(struct kobject *, const struct attribute *); */

/* extern int */
/* sysfs_create_link(struct kobject * kobj, struct kobject * target, const char * name); */

/* extern void */
/* sysfs_remove_link(struct kobject *, const char * name); */

/* int sysfs_create_bin_file(struct kobject * kobj, struct bin_attribute * attr); */
/* int sysfs_remove_bin_file(struct kobject * kobj, struct bin_attribute * attr); */

/* int sysfs_create_group(struct kobject *, const struct attribute_group *); */
/* void sysfs_remove_group(struct kobject *, const struct attribute_group *); */
/* void sysfs_notify(struct kobject * k, char *dir, char *attr); */





/* struct kref { */
/*  atomic_t refcount; */
/* }; */

/* void kref_init(struct kref *kref); */
/* void kref_get(struct kref *kref); */
/* int kref_put(struct kref *kref, void (*release) (struct kref *kref)); */


/* extern char uevent_helper[]; */


/* extern u64 uevent_seqnum; */


/* typedef int kobject_action_t; */
/* enum kobject_action { */
/*  KOBJ_ADD = ( kobject_action_t) 0x01, */
/*  KOBJ_REMOVE = ( kobject_action_t) 0x02, */
/*  KOBJ_CHANGE = ( kobject_action_t) 0x03, */
/*  KOBJ_MOUNT = ( kobject_action_t) 0x04, */
/*  KOBJ_UMOUNT = ( kobject_action_t) 0x05, */
/*  KOBJ_OFFLINE = ( kobject_action_t) 0x06, */
/*  KOBJ_ONLINE = ( kobject_action_t) 0x07, */
/* }; */

/* struct kobject { */
/*  const char * k_name; */
/*  char name[20]; */
/*  struct kref kref; */
/*  struct list_head entry; */
/*  struct kobject * parent; */
/*  struct kset * kset; */
/*  struct kobj_type * ktype; */
/*  struct dentry * dentry; */
/*  wait_queue_head_t poll; */
/* }; */

/* extern int kobject_set_name(struct kobject *, const char *, ...) */
/*  __attribute__((format(printf,2,3))); */

/* static inline __attribute__((always_inline)) const char * kobject_name(const struct kobject * kobj) */
/* { */
/*  return kobj->k_name; */
/* } */

/* extern void kobject_init(struct kobject *); */
/* extern void kobject_cleanup(struct kobject *); */

/* extern int kobject_add(struct kobject *); */
/* extern void kobject_del(struct kobject *); */

/* extern int kobject_rename(struct kobject *, const char *new_name); */

/* extern int kobject_register(struct kobject *); */
/* extern void kobject_unregister(struct kobject *); */

/* extern struct kobject * kobject_get(struct kobject *); */
/* extern void kobject_put(struct kobject *); */

/* extern struct kobject *kobject_add_dir(struct kobject *, const char *); */

/* extern char * kobject_get_path(struct kobject *, gfp_t); */

/* struct kobj_type { */
/*  void (*release)(struct kobject *); */
/*  struct sysfs_ops * sysfs_ops; */
/*  struct attribute ** default_attrs; */
/* }; */

/* struct kset_uevent_ops { */
/*  int (*filter)(struct kset *kset, struct kobject *kobj); */
/*  const char *(*name)(struct kset *kset, struct kobject *kobj); */
/*  int (*uevent)(struct kset *kset, struct kobject *kobj, char **envp, */
/*    int num_envp, char *buffer, int buffer_size); */
/* }; */

/* struct kset { */
/*  struct subsystem * subsys; */
/*  struct kobj_type * ktype; */
/*  struct list_head list; */
/*  spinlock_t list_lock; */
/*  struct kobject kobj; */
/*  struct kset_uevent_ops * uevent_ops; */
/* }; */


/* extern void kset_init(struct kset * k); */
/* extern int kset_add(struct kset * k); */
/* extern int kset_register(struct kset * k); */
/* extern void kset_unregister(struct kset * k); */

/* static inline __attribute__((always_inline)) struct kset * to_kset(struct kobject * kobj) */
/* { */
/*  return kobj ? ({ const typeof( ((struct kset *)0)->kobj ) *__mptr = (kobj); (struct kset *)( (char *)__mptr - __builtin_offsetof(struct kset,kobj) );}) : ((void *)0); */
/* } */

/* static inline __attribute__((always_inline)) struct kset * kset_get(struct kset * k) */
/* { */
/*  return k ? to_kset(kobject_get(&k->kobj)) : ((void *)0); */
/* } */

/* static inline __attribute__((always_inline)) void kset_put(struct kset * k) */
/* { */
/*  kobject_put(&k->kobj); */
/* } */

/* static inline __attribute__((always_inline)) struct kobj_type * get_ktype(struct kobject * k) */
/* { */
/*  if (k->kset && k->kset->ktype) */
/*   return k->kset->ktype; */
/*  else */
/*   return k->ktype; */
/* } */

/* extern struct kobject * kset_find_obj(struct kset *, const char *); */

/* struct subsystem { */
/*  struct kset kset; */
/*  struct rw_semaphore rwsem; */
/* }; */

/* extern struct subsystem kernel_subsys; */

/* extern struct subsystem hypervisor_subsys; */

/* extern void subsystem_init(struct subsystem *); */
/* extern int subsystem_register(struct subsystem *); */
/* extern void subsystem_unregister(struct subsystem *); */

/* static inline __attribute__((always_inline)) struct subsystem * subsys_get(struct subsystem * s) */
/* { */
/*  return s ? ({ const typeof( ((struct subsystem *)0)->kset ) *__mptr = (kset_get(&s->kset)); (struct subsystem *)( (char *)__mptr - __builtin_offsetof(struct subsystem,kset) );}) : ((void *)0); */
/* } */

/* static inline __attribute__((always_inline)) void subsys_put(struct subsystem * s) */
/* { */
/*  kset_put(&s->kset); */
/* } */

/* struct subsys_attribute { */
/*  struct attribute attr; */
/*  ssize_t (*show)(struct subsystem *, char *); */
/*  ssize_t (*store)(struct subsystem *, const char *, size_t); */
/* }; */

/* extern int subsys_create_file(struct subsystem * , struct subsys_attribute *); */


/* void kobject_uevent(struct kobject *kobj, enum kobject_action action); */

/* int add_uevent_var(char **envp, int num_envp, int *cur_index, */
/*    char *buffer, int buffer_size, int *cur_len, */
/*    const char *format, ...) */
/*  __attribute__((format (printf, 7, 8))); */



/* typedef int pm_request_t; */

/* typedef int pm_dev_t; */

/* enum */
/* { */
/*  PM_SYS_UNKNOWN = 0x00000000, */
/*  PM_SYS_KBC = 0x41d00303, */
/*  PM_SYS_COM = 0x41d00500, */
/*  PM_SYS_IRDA = 0x41d00510, */
/*  PM_SYS_FDC = 0x41d00700, */
/*  PM_SYS_VGA = 0x41d00900, */
/*  PM_SYS_PCMCIA = 0x41d00e00, */
/* }; */

/* struct pm_dev; */

/* typedef int (*pm_callback)(struct pm_dev *dev, pm_request_t rqst, void *data); */




/* struct pm_dev */
/* { */
/*  pm_dev_t type; */
/*  unsigned long id; */
/*  pm_callback callback; */
/*  void *data; */

/*  unsigned long flags; */
/*  unsigned long state; */
/*  unsigned long prev_state; */

/*  struct list_head entry; */
/* }; */







/* extern void (*pm_idle)(void); */
/* extern void (*pm_power_off)(void); */

/* typedef int suspend_state_t; */







/* typedef int suspend_disk_method_t; */







/* struct pm_ops { */
/*  suspend_disk_method_t pm_disk_mode; */
/*  int (*valid)(suspend_state_t state); */
/*  int (*prepare)(suspend_state_t state); */
/*  int (*enter)(suspend_state_t state); */
/*  int (*finish)(suspend_state_t state); */
/* }; */

/* extern void pm_set_ops(struct pm_ops *); */
/* extern struct pm_ops *pm_ops; */
/* extern int pm_suspend(suspend_state_t state); */






/* struct device; */

/* typedef struct pm_message { */
/*  int event; */
/* } pm_message_t; */

/* struct dev_pm_info { */
/*  pm_message_t power_state; */
/*  unsigned can_wakeup:1; */

/*  unsigned should_wakeup:1; */
/*  pm_message_t prev_state; */
/*  void * saved_state; */
/*  struct device * pm_parent; */
/*  struct list_head entry; */

/* }; */

/* extern void device_pm_set_parent(struct device * dev, struct device * parent); */

/* extern int device_power_down(pm_message_t state); */
/* extern void device_power_up(void); */
/* extern void device_resume(void); */


/* extern suspend_disk_method_t pm_disk_mode; */

/* extern int device_suspend(pm_message_t state); */






/* extern int dpm_runtime_suspend(struct device *, pm_message_t); */
/* extern void dpm_runtime_resume(struct device *); */
/* extern void __suspend_report_result(const char *function, void *fn, int ret); */



/* struct sys_device; */

/* struct sysdev_class { */
/*  struct list_head drivers; */


/*  int (*shutdown)(struct sys_device *); */
/*  int (*suspend)(struct sys_device *, pm_message_t state); */
/*  int (*resume)(struct sys_device *); */
/*  struct kset kset; */
/* }; */

/* struct sysdev_class_attribute { */
/*  struct attribute attr; */
/*  ssize_t (*show)(struct sysdev_class *, char *); */
/*  ssize_t (*store)(struct sysdev_class *, const char *, size_t); */
/* }; */

/* extern int sysdev_class_register(struct sysdev_class *); */
/* extern void sysdev_class_unregister(struct sysdev_class *); */

/* extern int sysdev_class_create_file(struct sysdev_class *, */
/*  struct sysdev_class_attribute *); */
/* extern void sysdev_class_remove_file(struct sysdev_class *, */
/*  struct sysdev_class_attribute *); */




/* struct sysdev_driver { */
/*  struct list_head entry; */
/*  int (*add)(struct sys_device *); */
/*  int (*remove)(struct sys_device *); */
/*  int (*shutdown)(struct sys_device *); */
/*  int (*suspend)(struct sys_device *, pm_message_t state); */
/*  int (*resume)(struct sys_device *); */
/* }; */


/* extern int sysdev_driver_register(struct sysdev_class *, struct sysdev_driver *); */
/* extern void sysdev_driver_unregister(struct sysdev_class *, struct sysdev_driver *); */







/* struct sys_device { */
/*  u32 id; */
/*  struct sysdev_class * cls; */
/*  struct kobject kobj; */
/* }; */

/* extern int sysdev_register(struct sys_device *); */
/* extern void sysdev_unregister(struct sys_device *); */


/* struct sysdev_attribute { */
/*  struct attribute attr; */
/*  ssize_t (*show)(struct sys_device *, char *); */
/*  ssize_t (*store)(struct sys_device *, const char *, size_t); */
/* }; */

/* extern int sysdev_create_file(struct sys_device *, struct sysdev_attribute *); */
/* extern void sysdev_remove_file(struct sys_device *, struct sysdev_attribute *); */

/* extern int sched_mc_power_savings, sched_smt_power_savings; */
/* extern struct sysdev_attribute attr_sched_mc_power_savings, attr_sched_smt_power_savings; */
/* extern int sched_create_sysfs_power_savings_entries(struct sysdev_class *cls); */

/* extern void normalize_rt_tasks(void); */





/* static inline __attribute__((always_inline)) int frozen(struct task_struct *p) */
/* { */
/*  return p->flags & 0x00010000; */
/* } */




/* static inline __attribute__((always_inline)) int freezing(struct task_struct *p) */
/* { */
/*  return p->flags & 0x00004000; */
/* } */





/* static inline __attribute__((always_inline)) void freeze(struct task_struct *p) */
/* { */
/*  p->flags |= 0x00004000; */
/* } */




/* static inline __attribute__((always_inline)) void do_not_freeze(struct task_struct *p) */
/* { */
/*  p->flags &= ~0x00004000; */
/* } */




/* static inline __attribute__((always_inline)) int thaw_process(struct task_struct *p) */
/* { */
/*  if (frozen(p)) { */
/*   p->flags &= ~0x00010000; */
/*   wake_up_process(p); */
/*   return 1; */
/*  } */
/*  return 0; */
/* } */




/* static inline __attribute__((always_inline)) void frozen_process(struct task_struct *p) */
/* { */
/*  p->flags = (p->flags & ~0x00004000) | 0x00010000; */
/* } */

/* extern void refrigerator(void); */
/* extern int freeze_processes(void); */
/* extern void thaw_processes(void); */

/* static inline __attribute__((always_inline)) int try_to_freeze(void) */
/* { */
/*  if (freezing(__vericon_dummy_current)) { */
/*   refrigerator(); */
/*   return 1; */
/*  } else */
/*   return 0; */
/* } */













/* struct __old_kernel_stat { */
/*  unsigned short st_dev; */
/*  unsigned short st_ino; */
/*  unsigned short st_mode; */
/*  unsigned short st_nlink; */
/*  unsigned short st_uid; */
/*  unsigned short st_gid; */
/*  unsigned short st_rdev; */
/*  unsigned long st_size; */
/*  unsigned long st_atime; */
/*  unsigned long st_mtime; */
/*  unsigned long st_ctime; */
/* }; */

/* struct stat { */
/*  unsigned long st_dev; */
/*  unsigned long st_ino; */
/*  unsigned short st_mode; */
/*  unsigned short st_nlink; */
/*  unsigned short st_uid; */
/*  unsigned short st_gid; */
/*  unsigned long st_rdev; */
/*  unsigned long st_size; */
/*  unsigned long st_blksize; */
/*  unsigned long st_blocks; */
/*  unsigned long st_atime; */
/*  unsigned long st_atime_nsec; */
/*  unsigned long st_mtime; */
/*  unsigned long st_mtime_nsec; */
/*  unsigned long st_ctime; */
/*  unsigned long st_ctime_nsec; */
/*  unsigned long __unused4; */
/*  unsigned long __unused5; */
/* }; */




/* struct stat64 { */
/*  unsigned long long st_dev; */
/*  unsigned char __pad0[4]; */


/*  unsigned long __st_ino; */

/*  unsigned int st_mode; */
/*  unsigned int st_nlink; */

/*  unsigned long st_uid; */
/*  unsigned long st_gid; */

/*  unsigned long long st_rdev; */
/*  unsigned char __pad3[4]; */

/*  long long st_size; */
/*  unsigned long st_blksize; */

/*  unsigned long long st_blocks; */

/*  unsigned long st_atime; */
/*  unsigned long st_atime_nsec; */

/*  unsigned long st_mtime; */
/*  unsigned int st_mtime_nsec; */

/*  unsigned long st_ctime; */
/*  unsigned long st_ctime_nsec; */

/*  unsigned long long st_ino; */
/* }; */


/* struct kstat { */
/*  unsigned long ino; */
/*  dev_t dev; */
/*  umode_t mode; */
/*  unsigned int nlink; */
/*  uid_t uid; */
/*  gid_t gid; */
/*  dev_t rdev; */
/*  loff_t size; */
/*  struct timespec atime; */
/*  struct timespec mtime; */
/*  struct timespec ctime; */
/*  unsigned long blksize; */
/*  unsigned long long blocks; */
/* }; */





/* extern int request_module(const char * name, ...) __attribute__ ((format (printf, 1, 2))); */






/* struct key; */
/* extern int call_usermodehelper_keys(char *path, char *argv[], char *envp[], */
/*         struct key *session_keyring, int wait); */

/* static inline __attribute__((always_inline)) int */
/* call_usermodehelper(char *path, char **argv, char **envp, int wait) */
/* { */
/*  return call_usermodehelper_keys(path, argv, envp, ((void *)0), wait); */
/* } */

/* extern void usermodehelper_init(void); */













/* struct user_i387_struct { */
/*  long cwd; */
/*  long swd; */
/*  long twd; */
/*  long fip; */
/*  long fcs; */
/*  long foo; */
/*  long fos; */
/*  long st_space[20]; */
/* }; */

/* struct user_fxsr_struct { */
/*  unsigned short cwd; */
/*  unsigned short swd; */
/*  unsigned short twd; */
/*  unsigned short fop; */
/*  long fip; */
/*  long fcs; */
/*  long foo; */
/*  long fos; */
/*  long mxcsr; */
/*  long reserved; */
/*  long st_space[32]; */
/*  long xmm_space[32]; */
/*  long padding[56]; */
/* }; */







/* struct user_regs_struct { */
/*  long ebx, ecx, edx, esi, edi, ebp, eax; */
/*  unsigned short ds, __ds, es, __es; */
/*  unsigned short fs, __fs, gs, __gs; */
/*  long orig_eax, eip; */
/*  unsigned short cs, __cs; */
/*  long eflags, esp; */
/*  unsigned short ss, __ss; */
/* }; */




/* struct user{ */


/*   struct user_regs_struct regs; */

/*   int u_fpvalid; */

/*   struct user_i387_struct i387; */

/*   unsigned long int u_tsize; */
/*   unsigned long int u_dsize; */
/*   unsigned long int u_ssize; */
/*   unsigned long start_code; */
/*   unsigned long start_stack; */



/*   long int signal; */
/*   int reserved; */
/*   struct user_pt_regs * u_ar0; */

/*   struct user_i387_struct* u_fpstate; */
/*   unsigned long magic; */
/*   char u_comm[32]; */
/*   int u_debugreg[8]; */
/* }; */









/* struct oldold_utsname { */
/*  char sysname[9]; */
/*  char nodename[9]; */
/*  char release[9]; */
/*  char version[9]; */
/*  char machine[9]; */
/* }; */



/* struct old_utsname { */
/*  char sysname[65]; */
/*  char nodename[65]; */
/*  char release[65]; */
/*  char version[65]; */
/*  char machine[65]; */
/* }; */

/* struct new_utsname { */
/*  char sysname[65]; */
/*  char nodename[65]; */
/*  char release[65]; */
/*  char version[65]; */
/*  char machine[65]; */
/*  char domainname[65]; */
/* }; */

/* extern struct new_utsname system_utsname; */

/* extern struct rw_semaphore uts_sem; */


/* typedef unsigned long elf_greg_t; */


/* typedef elf_greg_t elf_gregset_t[(sizeof (struct user_regs_struct) / sizeof(elf_greg_t))]; */

/* typedef struct user_i387_struct elf_fpregset_t; */
/* typedef struct user_fxsr_struct elf_fpxregset_t; */







/* struct user_desc { */
/*  unsigned int entry_number; */
/*  unsigned long base_addr; */
/*  unsigned int limit; */
/*  unsigned int seg_32bit:1; */
/*  unsigned int contents:2; */
/*  unsigned int read_exec_only:1; */
/*  unsigned int limit_in_pages:1; */
/*  unsigned int seg_not_present:1; */
/*  unsigned int useable:1; */
/* }; */


/* extern struct desc_struct cpu_gdt_table[32]; */

/* extern __typeof__(unsigned char) per_cpu__cpu_16bit_stack[1024]; */

/* struct Xgt_desc_struct { */
/*  unsigned short size; */
/*  unsigned long address __attribute__((packed)); */
/*  unsigned short pad; */
/* } __attribute__ ((packed)); */

/* extern struct Xgt_desc_struct idt_descr; */
/* extern __typeof__(struct Xgt_desc_struct) per_cpu__cpu_gdt_descr; */


/* static inline __attribute__((always_inline)) struct desc_struct *get_cpu_gdt_table(unsigned int cpu) */
/* { */
/*  return (struct desc_struct *)(*((void)(cpu), &per_cpu__cpu_gdt_descr)).address; */
/* } */

/* extern struct desc_struct default_ldt[]; */
/* extern void set_intr_gate(unsigned int irq, void * addr); */

/* static inline __attribute__((always_inline)) void __set_tss_desc(unsigned int cpu, unsigned int entry, void *addr) */
/* { */
/*  __asm__ __volatile__ ("movw %w3,0(%2)\n\t" "movw %w1,2(%2)\n\t" "rorl $16,%1\n\t" "movb %b1,4(%2)\n\t" "movb %4,5(%2)\n\t" "movb $0,6(%2)\n\t" "movb %h1,7(%2)\n\t" "rorl $16,%1" : "=m"(*(&get_cpu_gdt_table(cpu)[entry])) : "q" ((int)addr), "r"(&get_cpu_gdt_table(cpu)[entry]), "ir"(__builtin_offsetof(struct tss_struct,__cacheline_filler) - 1), "i"(0x89)); */

/* } */



/* static inline __attribute__((always_inline)) void set_ldt_desc(unsigned int cpu, void *addr, unsigned int size) */
/* { */
/*  __asm__ __volatile__ ("movw %w3,0(%2)\n\t" "movw %w1,2(%2)\n\t" "rorl $16,%1\n\t" "movb %b1,4(%2)\n\t" "movb %4,5(%2)\n\t" "movb $0,6(%2)\n\t" "movb %h1,7(%2)\n\t" "rorl $16,%1" : "=m"(*(&get_cpu_gdt_table(cpu)[(12 + 5)])) : "q" ((int)addr), "r"(&get_cpu_gdt_table(cpu)[(12 + 5)]), "ir"(((size << 3)-1)), "i"(0x82)); */
/* } */

/* static inline __attribute__((always_inline)) void write_ldt_entry(void *ldt, int entry, __u32 entry_a, __u32 entry_b) */
/* { */
/*  __u32 *lp = (__u32 *)((char *)ldt + entry*8); */
/*  *lp = entry_a; */
/*  *(lp+1) = entry_b; */
/* } */





/* static inline __attribute__((always_inline)) void load_TLS(struct thread_struct *t, unsigned int cpu) */
/* { */

/*  get_cpu_gdt_table(cpu)[6 + 0] = t->tls_array[0]; get_cpu_gdt_table(cpu)[6 + 1] = t->tls_array[1]; get_cpu_gdt_table(cpu)[6 + 2] = t->tls_array[2]; */

/* } */

/* static inline __attribute__((always_inline)) void clear_LDT(void) */
/* { */
/*  int cpu = ({ do { } while (0); 0; }); */

/*  set_ldt_desc(cpu, &default_ldt[0], 5); */
/*  __asm__ __volatile__("lldt %w0"::"q" ((12 + 5)*8)); */
/*  do { } while (0); */
/* } */




/* static inline __attribute__((always_inline)) void load_LDT_nolock(mm_context_t *pc, int cpu) */
/* { */
/*  void *segments = pc->ldt; */
/*  int count = pc->size; */

/*  if (__builtin_expect(!!(!count), 1)) { */
/*   segments = &default_ldt[0]; */
/*   count = 5; */
/*  } */

/*  set_ldt_desc(cpu, segments, count); */
/*  __asm__ __volatile__("lldt %w0"::"q" ((12 + 5)*8)); */
/* } */

/* static inline __attribute__((always_inline)) void load_LDT(mm_context_t *pc) */
/* { */
/*  int cpu = ({ do { } while (0); 0; }); */
/*  load_LDT_nolock(pc, cpu); */
/*  do { } while (0); */
/* } */

/* static inline __attribute__((always_inline)) unsigned long get_desc_base(unsigned long *desc) */
/* { */
/*  unsigned long base; */
/*  base = ((desc[0] >> 16) & 0x0000ffff) | */
/*   ((desc[1] << 16) & 0x00ff0000) | */
/*   (desc[1] & 0xff000000); */
/*  return base; */
/* } */


/* struct task_struct; */

/* extern int dump_task_regs (struct task_struct *, elf_gregset_t *); */
/* extern int dump_task_fpu (struct task_struct *, elf_fpregset_t *); */
/* extern int dump_task_extended_fpu (struct task_struct *, struct user_fxsr_struct *); */

/* extern void __kernel_vsyscall; */




/* struct linux_binprm; */
/* extern int arch_setup_additional_pages(struct linux_binprm *bprm, */
/*                                        int executable_stack); */

/* extern unsigned int vdso_enabled; */


/* typedef __u32 Elf32_Addr; */
/* typedef __u16 Elf32_Half; */
/* typedef __u32 Elf32_Off; */
/* typedef __s32 Elf32_Sword; */
/* typedef __u32 Elf32_Word; */


/* typedef __u64 Elf64_Addr; */
/* typedef __u16 Elf64_Half; */
/* typedef __s16 Elf64_SHalf; */
/* typedef __u64 Elf64_Off; */
/* typedef __s32 Elf64_Sword; */
/* typedef __u32 Elf64_Word; */
/* typedef __u64 Elf64_Xword; */
/* typedef __s64 Elf64_Sxword; */

/* typedef struct dynamic{ */
/*   Elf32_Sword d_tag; */
/*   union{ */
/*     Elf32_Sword d_val; */
/*     Elf32_Addr d_ptr; */
/*   } d_un; */
/* } Elf32_Dyn; */

/* typedef struct { */
/*   Elf64_Sxword d_tag; */
/*   union { */
/*     Elf64_Xword d_val; */
/*     Elf64_Addr d_ptr; */
/*   } d_un; */
/* } Elf64_Dyn; */

/* typedef struct elf32_rel { */
/*   Elf32_Addr r_offset; */
/*   Elf32_Word r_info; */
/* } Elf32_Rel; */

/* typedef struct elf64_rel { */
/*   Elf64_Addr r_offset; */
/*   Elf64_Xword r_info; */
/* } Elf64_Rel; */

/* typedef struct elf32_rela{ */
/*   Elf32_Addr r_offset; */
/*   Elf32_Word r_info; */
/*   Elf32_Sword r_addend; */
/* } Elf32_Rela; */

/* typedef struct elf64_rela { */
/*   Elf64_Addr r_offset; */
/*   Elf64_Xword r_info; */
/*   Elf64_Sxword r_addend; */
/* } Elf64_Rela; */

/* typedef struct elf32_sym{ */
/*   Elf32_Word st_name; */
/*   Elf32_Addr st_value; */
/*   Elf32_Word st_size; */
/*   unsigned char st_info; */
/*   unsigned char st_other; */
/*   Elf32_Half st_shndx; */
/* } Elf32_Sym; */

/* typedef struct elf64_sym { */
/*   Elf64_Word st_name; */
/*   unsigned char st_info; */
/*   unsigned char st_other; */
/*   Elf64_Half st_shndx; */
/*   Elf64_Addr st_value; */
/*   Elf64_Xword st_size; */
/* } Elf64_Sym; */




/* typedef struct elf32_hdr{ */
/*   unsigned char e_ident[16]; */
/*   Elf32_Half e_type; */
/*   Elf32_Half e_machine; */
/*   Elf32_Word e_version; */
/*   Elf32_Addr e_entry; */
/*   Elf32_Off e_phoff; */
/*   Elf32_Off e_shoff; */
/*   Elf32_Word e_flags; */
/*   Elf32_Half e_ehsize; */
/*   Elf32_Half e_phentsize; */
/*   Elf32_Half e_phnum; */
/*   Elf32_Half e_shentsize; */
/*   Elf32_Half e_shnum; */
/*   Elf32_Half e_shstrndx; */
/* } Elf32_Ehdr; */

/* typedef struct elf64_hdr { */
/*   unsigned char e_ident[16]; */
/*   Elf64_Half e_type; */
/*   Elf64_Half e_machine; */
/*   Elf64_Word e_version; */
/*   Elf64_Addr e_entry; */
/*   Elf64_Off e_phoff; */
/*   Elf64_Off e_shoff; */
/*   Elf64_Word e_flags; */
/*   Elf64_Half e_ehsize; */
/*   Elf64_Half e_phentsize; */
/*   Elf64_Half e_phnum; */
/*   Elf64_Half e_shentsize; */
/*   Elf64_Half e_shnum; */
/*   Elf64_Half e_shstrndx; */
/* } Elf64_Ehdr; */







/* typedef struct elf32_phdr{ */
/*   Elf32_Word p_type; */
/*   Elf32_Off p_offset; */
/*   Elf32_Addr p_vaddr; */
/*   Elf32_Addr p_paddr; */
/*   Elf32_Word p_filesz; */
/*   Elf32_Word p_memsz; */
/*   Elf32_Word p_flags; */
/*   Elf32_Word p_align; */
/* } Elf32_Phdr; */

/* typedef struct elf64_phdr { */
/*   Elf64_Word p_type; */
/*   Elf64_Word p_flags; */
/*   Elf64_Off p_offset; */
/*   Elf64_Addr p_vaddr; */
/*   Elf64_Addr p_paddr; */
/*   Elf64_Xword p_filesz; */
/*   Elf64_Xword p_memsz; */
/*   Elf64_Xword p_align; */
/* } Elf64_Phdr; */

/* typedef struct { */
/*   Elf32_Word sh_name; */
/*   Elf32_Word sh_type; */
/*   Elf32_Word sh_flags; */
/*   Elf32_Addr sh_addr; */
/*   Elf32_Off sh_offset; */
/*   Elf32_Word sh_size; */
/*   Elf32_Word sh_link; */
/*   Elf32_Word sh_info; */
/*   Elf32_Word sh_addralign; */
/*   Elf32_Word sh_entsize; */
/* } Elf32_Shdr; */

/* typedef struct elf64_shdr { */
/*   Elf64_Word sh_name; */
/*   Elf64_Word sh_type; */
/*   Elf64_Xword sh_flags; */
/*   Elf64_Addr sh_addr; */
/*   Elf64_Off sh_offset; */
/*   Elf64_Xword sh_size; */
/*   Elf64_Word sh_link; */
/*   Elf64_Word sh_info; */
/*   Elf64_Xword sh_addralign; */
/*   Elf64_Xword sh_entsize; */
/* } Elf64_Shdr; */

/* typedef struct elf32_note { */
/*   Elf32_Word n_namesz; */
/*   Elf32_Word n_descsz; */
/*   Elf32_Word n_type; */
/* } Elf32_Nhdr; */


/* typedef struct elf64_note { */
/*   Elf64_Word n_namesz; */
/*   Elf64_Word n_descsz; */
/*   Elf64_Word n_type; */
/* } Elf64_Nhdr; */



/* extern Elf32_Dyn _DYNAMIC []; */





/* struct kernel_param; */


/* typedef int (*param_set_fn)(const char *val, struct kernel_param *kp); */

/* typedef int (*param_get_fn)(char *buffer, struct kernel_param *kp); */

/* struct kernel_param { */
/*  const char *name; */
/*  unsigned int perm; */
/*  param_set_fn set; */
/*  param_get_fn get; */
/*  void *arg; */
/* }; */


/* struct kparam_string { */
/*  unsigned int maxlen; */
/*  char *string; */
/* }; */


/* struct kparam_array */
/* { */
/*  unsigned int max; */
/*  unsigned int *num; */
/*  param_set_fn set; */
/*  param_get_fn get; */
/*  unsigned int elemsize; */
/*  void *elem; */
/* }; */

/* extern int parse_args(const char *name, */
/*         char *args, */
/*         struct kernel_param *params, */
/*         unsigned num, */
/*         int (*unknown)(char *param, char *val)); */







/* extern int param_set_byte(const char *val, struct kernel_param *kp); */
/* extern int param_get_byte(char *buffer, struct kernel_param *kp); */


/* extern int param_set_short(const char *val, struct kernel_param *kp); */
/* extern int param_get_short(char *buffer, struct kernel_param *kp); */


/* extern int param_set_ushort(const char *val, struct kernel_param *kp); */
/* extern int param_get_ushort(char *buffer, struct kernel_param *kp); */


/* extern int param_set_int(const char *val, struct kernel_param *kp); */
/* extern int param_get_int(char *buffer, struct kernel_param *kp); */


/* extern int param_set_uint(const char *val, struct kernel_param *kp); */
/* extern int param_get_uint(char *buffer, struct kernel_param *kp); */


/* extern int param_set_long(const char *val, struct kernel_param *kp); */
/* extern int param_get_long(char *buffer, struct kernel_param *kp); */


/* extern int param_set_ulong(const char *val, struct kernel_param *kp); */
/* extern int param_get_ulong(char *buffer, struct kernel_param *kp); */


/* extern int param_set_charp(const char *val, struct kernel_param *kp); */
/* extern int param_get_charp(char *buffer, struct kernel_param *kp); */


/* extern int param_set_bool(const char *val, struct kernel_param *kp); */
/* extern int param_get_bool(char *buffer, struct kernel_param *kp); */


/* extern int param_set_invbool(const char *val, struct kernel_param *kp); */
/* extern int param_get_invbool(char *buffer, struct kernel_param *kp); */

/* extern int param_array_set(const char *val, struct kernel_param *kp); */
/* extern int param_array_get(char *buffer, struct kernel_param *kp); */

/* extern int param_set_copystring(const char *val, struct kernel_param *kp); */
/* extern int param_get_string(char *buffer, struct kernel_param *kp); */



/* struct module; */

/* extern int module_param_sysfs_setup(struct module *mod, */
/*         struct kernel_param *kparam, */
/*         unsigned int num_params); */

/* extern void module_param_sysfs_remove(struct module *mod); */







/* typedef struct */
/* { */
/*  volatile long counter; */
/* } local_t; */






/* static __inline__ __attribute__((always_inline)) void local_inc(local_t *v) */
/* { */
/*  __asm__ __volatile__( */
/*   "incl %0" */
/*   :"+m" (v->counter)); */
/* } */

/* static __inline__ __attribute__((always_inline)) void local_dec(local_t *v) */
/* { */
/*  __asm__ __volatile__( */
/*   "decl %0" */
/*   :"+m" (v->counter)); */
/* } */

/* static __inline__ __attribute__((always_inline)) void local_add(long i, local_t *v) */
/* { */
/*  __asm__ __volatile__( */
/*   "addl %1,%0" */
/*   :"+m" (v->counter) */
/*   :"ir" (i)); */
/* } */

/* static __inline__ __attribute__((always_inline)) void local_sub(long i, local_t *v) */
/* { */
/*  __asm__ __volatile__( */
/*   "subl %1,%0" */
/*   :"+m" (v->counter) */
/*   :"ir" (i)); */
/* } */







/* struct mod_arch_specific */
/* { */
/* }; */


/* struct kernel_symbol */
/* { */
/*  unsigned long value; */
/*  const char *name; */
/* }; */

/* struct modversion_info */
/* { */
/*  unsigned long crc; */
/*  char name[(64 - sizeof(unsigned long))]; */
/* }; */

/* struct module; */

/* struct module_attribute { */
/*         struct attribute attr; */
/*         ssize_t (*show)(struct module_attribute *, struct module *, char *); */
/*         ssize_t (*store)(struct module_attribute *, struct module *, */
/*     const char *, size_t count); */
/*  void (*setup)(struct module *, const char *); */
/*  int (*test)(struct module *); */
/*  void (*free)(struct module *); */
/* }; */

/* struct module_kobject */
/* { */
/*  struct kobject kobj; */
/*  struct module *mod; */
/* }; */


/* extern int init_module(void); */
/* extern void cleanup_module(void); */


/* struct exception_table_entry; */

/* const struct exception_table_entry * */
/* search_extable(const struct exception_table_entry *first, */
/*         const struct exception_table_entry *last, */
/*         unsigned long value); */
/* void sort_extable(struct exception_table_entry *start, */
/*     struct exception_table_entry *finish); */
/* void sort_main_extable(void); */

/* extern struct subsystem module_subsys; */






/* extern struct module __this_module; */

/* const struct exception_table_entry *search_exception_tables(unsigned long add); */

/* struct notifier_block; */




/* void *__symbol_get(const char *symbol); */
/* void *__symbol_get_gpl(const char *symbol); */

/* struct module_ref */
/* { */
/*  local_t count; */
/* } __attribute__((__aligned__((1 << (7))))); */

/* enum module_state */
/* { */
/*  MODULE_STATE_LIVE, */
/*  MODULE_STATE_COMING, */
/*  MODULE_STATE_GOING, */
/* }; */



/* struct module_sect_attr */
/* { */
/*  struct module_attribute mattr; */
/*  char name[32]; */
/*  unsigned long address; */
/* }; */

/* struct module_sect_attrs */
/* { */
/*  struct attribute_group grp; */
/*  struct module_sect_attr attrs[0]; */
/* }; */

/* struct module_param_attrs; */

/* struct module */
/* { */
/*  enum module_state state; */


/*  struct list_head list; */


/*  char name[(64 - sizeof(unsigned long))]; */


/*  struct module_kobject mkobj; */
/*  struct module_param_attrs *param_attrs; */
/*  struct module_attribute *modinfo_attrs; */
/*  const char *version; */
/*  const char *srcversion; */


/*  const struct kernel_symbol *syms; */
/*  unsigned int num_syms; */
/*  const unsigned long *crcs; */


/*  const struct kernel_symbol *gpl_syms; */
/*  unsigned int num_gpl_syms; */
/*  const unsigned long *gpl_crcs; */


/*  const struct kernel_symbol *unused_syms; */
/*  unsigned int num_unused_syms; */
/*  const unsigned long *unused_crcs; */

/*  const struct kernel_symbol *unused_gpl_syms; */
/*  unsigned int num_unused_gpl_syms; */
/*  const unsigned long *unused_gpl_crcs; */


/*  const struct kernel_symbol *gpl_future_syms; */
/*  unsigned int num_gpl_future_syms; */
/*  const unsigned long *gpl_future_crcs; */


/*  unsigned int num_exentries; */
/*  const struct exception_table_entry *extable; */


/*  int (*init)(void); */


/*  void *module_init; */


/*  void *module_core; */


/*  unsigned long init_size, core_size; */


/*  unsigned long init_text_size, core_text_size; */


/*  void *unwind_info; */


/*  struct mod_arch_specific arch; */


/*  int unsafe; */


/*  int license_gplok; */



/*  struct module_ref ref[1]; */


/*  struct list_head modules_which_use_me; */


/*  struct task_struct *waiter; */


/*  void (*exit)(void); */




/*  Elf32_Sym *symtab; */
/*  unsigned long num_symtab; */
/*  char *strtab; */


/*  struct module_sect_attrs *sect_attrs; */



/*  void *percpu; */



/*  char *args; */
/* }; */




/* static inline __attribute__((always_inline)) int module_is_live(struct module *mod) */
/* { */
/*  return mod->state != MODULE_STATE_GOING; */
/* } */


/* struct module *module_text_address(unsigned long addr); */
/* struct module *__module_text_address(unsigned long addr); */
/* int is_module_address(unsigned long addr); */



/* struct module *module_get_kallsym(unsigned int symnum, unsigned long *value, */
/*     char *type, char *name, size_t namelen); */


/* unsigned long module_kallsyms_lookup_name(const char *name); */

/* int is_exported(const char *name, const struct module *mod); */

/* extern void __module_put_and_exit(struct module *mod, long code) */
/*  __attribute__((noreturn)); */



/* unsigned int module_refcount(struct module *mod); */
/* void __symbol_put(const char *symbol); */

/* void symbol_put_addr(void *addr); */



/* static inline __attribute__((always_inline)) void __module_get(struct module *module) */
/* { */
/*  if (module) { */
/*   do { if (__builtin_expect(!!((module_refcount(module) == 0)!=0), 0)) __asm__ __volatile__( "ud2\n" "\t.word %c0\n" "\t.long %c1\n" : : "i" (388), "i" ("include/linux/module.h")); } while(0); */
/*   local_inc(&module->ref[({ do { } while (0); 0; })].count); */
/*   do { } while (0); */
/*  } */
/* } */

/* static inline __attribute__((always_inline)) int try_module_get(struct module *module) */
/* { */
/*  int ret = 1; */

/*  if (module) { */
/*   unsigned int cpu = ({ do { } while (0); 0; }); */
/*   if (__builtin_expect(!!(module_is_live(module)), 1)) */
/*    local_inc(&module->ref[cpu].count); */
/*   else */
/*    ret = 0; */
/*   do { } while (0); */
/*  } */
/*  return ret; */
/* } */

/* static inline __attribute__((always_inline)) void module_put(struct module *module) */
/* { */
/*  if (module) { */
/*   unsigned int cpu = ({ do { } while (0); 0; }); */
/*   local_dec(&module->ref[cpu].count); */

/*   if (__builtin_expect(!!(!module_is_live(module)), 0)) */
/*    wake_up_process(module->waiter); */
/*   do { } while (0); */
/*  } */
/* } */

/* const char *module_address_lookup(unsigned long addr, */
/*       unsigned long *symbolsize, */
/*       unsigned long *offset, */
/*       char **modname); */


/* const struct exception_table_entry *search_module_extables(unsigned long addr); */

/* int register_module_notifier(struct notifier_block * nb); */
/* int unregister_module_notifier(struct notifier_block * nb); */

/* extern void print_modules(void); */

/* struct device_driver; */
/* void module_add_driver(struct module *, struct device_driver *); */
/* void module_remove_driver(struct device_driver *); */












/* extern unsigned int __invalid_size_argument_for_IOC; */




/* struct files_stat_struct { */
/*  int nr_files; */
/*  int nr_free_files; */
/*  int max_files; */
/* }; */
/* extern struct files_stat_struct files_stat; */
/* extern int get_max_files(void); */

/* struct inodes_stat_t { */
/*  int nr_inodes; */
/*  int nr_unused; */
/*  int dummy[5]; */
/* }; */
/* extern struct inodes_stat_t inodes_stat; */

/* extern int leases_enable, lease_break_time; */


/* extern int dir_notify_enable; */



/* static inline __attribute__((always_inline)) int old_valid_dev(dev_t dev) */
/* { */
/*  return ((unsigned int) ((dev) >> 20)) < 256 && ((unsigned int) ((dev) & ((1U << 20) - 1))) < 256; */
/* } */

/* static inline __attribute__((always_inline)) u16 old_encode_dev(dev_t dev) */
/* { */
/*  return (((unsigned int) ((dev) >> 20)) << 8) | ((unsigned int) ((dev) & ((1U << 20) - 1))); */
/* } */

/* static inline __attribute__((always_inline)) dev_t old_decode_dev(u16 val) */
/* { */
/*  return ((((val >> 8) & 255) << 20) | (val & 255)); */
/* } */

/* static inline __attribute__((always_inline)) int new_valid_dev(dev_t dev) */
/* { */
/*  return 1; */
/* } */

/* static inline __attribute__((always_inline)) u32 new_encode_dev(dev_t dev) */
/* { */
/*  unsigned major = ((unsigned int) ((dev) >> 20)); */
/*  unsigned minor = ((unsigned int) ((dev) & ((1U << 20) - 1))); */
/*  return (minor & 0xff) | (major << 8) | ((minor & ~0xff) << 12); */
/* } */

/* static inline __attribute__((always_inline)) dev_t new_decode_dev(u32 dev) */
/* { */
/*  unsigned major = (dev & 0xfff00) >> 8; */
/*  unsigned minor = (dev & 0xff) | ((dev >> 12) & 0xfff00); */
/*  return (((major) << 20) | (minor)); */
/* } */

/* static inline __attribute__((always_inline)) int huge_valid_dev(dev_t dev) */
/* { */
/*  return 1; */
/* } */

/* static inline __attribute__((always_inline)) u64 huge_encode_dev(dev_t dev) */
/* { */
/*  return new_encode_dev(dev); */
/* } */

/* static inline __attribute__((always_inline)) dev_t huge_decode_dev(u64 dev) */
/* { */
/*  return new_decode_dev(dev); */
/* } */

/* static inline __attribute__((always_inline)) int sysv_valid_dev(dev_t dev) */
/* { */
/*  return ((unsigned int) ((dev) >> 20)) < (1<<14) && ((unsigned int) ((dev) & ((1U << 20) - 1))) < (1<<18); */
/* } */

/* static inline __attribute__((always_inline)) u32 sysv_encode_dev(dev_t dev) */
/* { */
/*  return ((unsigned int) ((dev) & ((1U << 20) - 1))) | (((unsigned int) ((dev) >> 20)) << 18); */
/* } */

/* static inline __attribute__((always_inline)) unsigned sysv_major(u32 dev) */
/* { */
/*  return (dev >> 18) & 0x3fff; */
/* } */

/* static inline __attribute__((always_inline)) unsigned sysv_minor(u32 dev) */
/* { */
/*  return dev & 0x3ffff; */
/* } */



/* struct nameidata; */
/* struct vfsmount; */

/* struct qstr { */
/*  unsigned int hash; */
/*  unsigned int len; */
/*  const unsigned char *name; */
/* }; */

/* struct dentry_stat_t { */
/*  int nr_dentry; */
/*  int nr_unused; */
/*  int age_limit; */
/*  int want_pages; */
/*  int dummy[2]; */
/* }; */
/* extern struct dentry_stat_t dentry_stat; */






/* static inline __attribute__((always_inline)) unsigned long */
/* partial_name_hash(unsigned long c, unsigned long prevhash) */
/* { */
/*  return (prevhash + (c << 4) + (c >> 4)) * 11; */
/* } */





/* static inline __attribute__((always_inline)) unsigned long end_name_hash(unsigned long hash) */
/* { */
/*  return (unsigned int) hash; */
/* } */


/* static inline __attribute__((always_inline)) unsigned int */
/* full_name_hash(const unsigned char *name, unsigned int len) */
/* { */
/*  unsigned long hash = 0; */
/*  while (len--) */
/*   hash = partial_name_hash(*name++, hash); */
/*  return end_name_hash(hash); */
/* } */

/* struct dcookie_struct; */



/* struct dentry { */
/*  atomic_t d_count; */
/*  unsigned int d_flags; */
/*  spinlock_t d_lock; */
/*  struct inode *d_inode; */





/*  struct hlist_node d_hash; */
/*  struct dentry *d_parent; */
/*  struct qstr d_name; */

/*  struct list_head d_lru; */



/*  union { */
/*   struct list_head d_child; */
/*    struct rcu_head d_rcu; */
/*  } d_u; */
/*  struct list_head d_subdirs; */
/*  struct list_head d_alias; */
/*  unsigned long d_time; */
/*  struct dentry_operations *d_op; */
/*  struct super_block *d_sb; */
/*  void *d_fsdata; */

/*  struct dcookie_struct *d_cookie; */

/*  int d_mounted; */
/*  unsigned char d_iname[36]; */
/* }; */







/* enum dentry_d_lock_class */
/* { */
/*  DENTRY_D_LOCK_NORMAL, */
/*  DENTRY_D_LOCK_NESTED */
/* }; */

/* struct dentry_operations { */
/*  int (*d_revalidate)(struct dentry *, struct nameidata *); */
/*  int (*d_hash) (struct dentry *, struct qstr *); */
/*  int (*d_compare) (struct dentry *, struct qstr *, struct qstr *); */
/*  int (*d_delete)(struct dentry *); */
/*  void (*d_release)(struct dentry *); */
/*  void (*d_iput)(struct dentry *, struct inode *); */
/* }; */

/* extern spinlock_t dcache_lock; */

/* static inline __attribute__((always_inline)) void __d_drop(struct dentry *dentry) */
/* { */
/*  if (!(dentry->d_flags & 0x0010)) { */
/*   dentry->d_flags |= 0x0010; */
/*   hlist_del_rcu(&dentry->d_hash); */
/*  } */
/* } */

/* static inline __attribute__((always_inline)) void d_drop(struct dentry *dentry) */
/* { */
/*  _spin_lock(&dcache_lock); */
/*  _spin_lock(&dentry->d_lock); */
/*   __d_drop(dentry); */
/*  _spin_unlock(&dentry->d_lock); */
/*  _spin_unlock(&dcache_lock); */
/* } */

/* static inline __attribute__((always_inline)) int dname_external(struct dentry *dentry) */
/* { */
/*  return dentry->d_name.name != dentry->d_iname; */
/* } */




/* extern void d_instantiate(struct dentry *, struct inode *); */
/* extern struct dentry * d_instantiate_unique(struct dentry *, struct inode *); */
/* extern void d_delete(struct dentry *); */


/* extern struct dentry * d_alloc(struct dentry *, const struct qstr *); */
/* extern struct dentry * d_alloc_anon(struct inode *); */
/* extern struct dentry * d_splice_alias(struct inode *, struct dentry *); */
/* extern void shrink_dcache_sb(struct super_block *); */
/* extern void shrink_dcache_parent(struct dentry *); */
/* extern int d_invalidate(struct dentry *); */


/* extern struct dentry * d_alloc_root(struct inode *); */


/* extern void d_genocide(struct dentry *); */

/* extern struct dentry *d_find_alias(struct inode *); */
/* extern void d_prune_aliases(struct inode *); */


/* extern int have_submounts(struct dentry *); */




/* extern void d_rehash(struct dentry *); */

/* static inline __attribute__((always_inline)) void d_add(struct dentry *entry, struct inode *inode) */
/* { */
/*  d_instantiate(entry, inode); */
/*  d_rehash(entry); */
/* } */

/* static inline __attribute__((always_inline)) struct dentry *d_add_unique(struct dentry *entry, struct inode *inode) */
/* { */
/*  struct dentry *res; */

/*  res = d_instantiate_unique(entry, inode); */
/*  d_rehash(res != ((void *)0) ? res : entry); */
/*  return res; */
/* } */


/* extern void d_move(struct dentry *, struct dentry *); */


/* extern struct dentry * d_lookup(struct dentry *, struct qstr *); */
/* extern struct dentry * __d_lookup(struct dentry *, struct qstr *); */
/* extern struct dentry * d_hash_and_lookup(struct dentry *, struct qstr *); */


/* extern int d_validate(struct dentry *, struct dentry *); */

/* extern char * d_path(struct dentry *, struct vfsmount *, char *, int); */

/* static inline __attribute__((always_inline)) struct dentry *dget(struct dentry *dentry) */
/* { */
/*  if (dentry) { */
/*   do { if (__builtin_expect(!!((!((&dentry->d_count)->counter))!=0), 0)) __asm__ __volatile__( "ud2\n" "\t.word %c0\n" "\t.long %c1\n" : : "i" (314), "i" ("include/linux/dcache.h")); } while(0); */
/*   atomic_inc(&dentry->d_count); */
/*  } */
/*  return dentry; */
/* } */

/* extern struct dentry * dget_locked(struct dentry *); */

/* static inline __attribute__((always_inline)) int d_unhashed(struct dentry *dentry) */
/* { */
/*  return (dentry->d_flags & 0x0010); */
/* } */

/* static inline __attribute__((always_inline)) struct dentry *dget_parent(struct dentry *dentry) */
/* { */
/*  struct dentry *ret; */

/*  _spin_lock(&dentry->d_lock); */
/*  ret = dget(dentry->d_parent); */
/*  _spin_unlock(&dentry->d_lock); */
/*  return ret; */
/* } */

/* extern void dput(struct dentry *); */

/* static inline __attribute__((always_inline)) int d_mountpoint(struct dentry *dentry) */
/* { */
/*  return dentry->d_mounted; */
/* } */

/* extern struct vfsmount *lookup_mnt(struct vfsmount *, struct dentry *); */
/* extern struct vfsmount *__lookup_mnt(struct vfsmount *, struct dentry *, int); */
/* extern struct dentry *lookup_create(struct nameidata *nd, int is_dir); */

/* extern int sysctl_vfs_cache_pressure; */







/* struct radix_tree_root { */
/*  unsigned int height; */
/*  gfp_t gfp_mask; */
/*  struct radix_tree_node *rnode; */
/* }; */

/* int radix_tree_insert(struct radix_tree_root *, unsigned long, void *); */
/* void *radix_tree_lookup(struct radix_tree_root *, unsigned long); */
/* void **radix_tree_lookup_slot(struct radix_tree_root *, unsigned long); */
/* void *radix_tree_delete(struct radix_tree_root *, unsigned long); */
/* unsigned int */
/* radix_tree_gang_lookup(struct radix_tree_root *root, void **results, */
/*    unsigned long first_index, unsigned int max_items); */
/* int radix_tree_preload(gfp_t gfp_mask); */
/* void radix_tree_init(void); */
/* void *radix_tree_tag_set(struct radix_tree_root *root, */
/*    unsigned long index, unsigned int tag); */
/* void *radix_tree_tag_clear(struct radix_tree_root *root, */
/*    unsigned long index, unsigned int tag); */
/* int radix_tree_tag_get(struct radix_tree_root *root, */
/*    unsigned long index, unsigned int tag); */
/* unsigned int */
/* radix_tree_gang_lookup_tag(struct radix_tree_root *root, void **results, */
/*   unsigned long first_index, unsigned int max_items, */
/*   unsigned int tag); */
/* int radix_tree_tagged(struct radix_tree_root *root, unsigned int tag); */

/* static inline __attribute__((always_inline)) void radix_tree_preload_end(void) */
/* { */
/*  do { } while (0); */
/* } */



/* struct raw_prio_tree_node { */
/*  struct prio_tree_node *left; */
/*  struct prio_tree_node *right; */
/*  struct prio_tree_node *parent; */
/* }; */

/* struct prio_tree_node { */
/*  struct prio_tree_node *left; */
/*  struct prio_tree_node *right; */
/*  struct prio_tree_node *parent; */
/*  unsigned long start; */
/*  unsigned long last; */
/* }; */

/* struct prio_tree_root { */
/*  struct prio_tree_node *prio_tree_node; */
/*  unsigned short index_bits; */
/*  unsigned short raw; */




/* }; */

/* struct prio_tree_iter { */
/*  struct prio_tree_node *cur; */
/*  unsigned long mask; */
/*  unsigned long value; */
/*  int size_level; */

/*  struct prio_tree_root *root; */
/*  unsigned long r_index; */
/*  unsigned long h_index; */
/* }; */

/* static inline __attribute__((always_inline)) void prio_tree_iter_init(struct prio_tree_iter *iter, */
/*   struct prio_tree_root *root, unsigned long r_index, unsigned long h_index) */
/* { */
/*  iter->root = root; */
/*  iter->r_index = r_index; */
/*  iter->h_index = h_index; */
/*  iter->cur = ((void *)0); */
/* } */

/* static inline __attribute__((always_inline)) int prio_tree_empty(const struct prio_tree_root *root) */
/* { */
/*  return root->prio_tree_node == ((void *)0); */
/* } */

/* static inline __attribute__((always_inline)) int prio_tree_root(const struct prio_tree_node *node) */
/* { */
/*  return node->parent == node; */
/* } */

/* static inline __attribute__((always_inline)) int prio_tree_left_empty(const struct prio_tree_node *node) */
/* { */
/*  return node->left == node; */
/* } */

/* static inline __attribute__((always_inline)) int prio_tree_right_empty(const struct prio_tree_node *node) */
/* { */
/*  return node->right == node; */
/* } */


/* struct prio_tree_node *prio_tree_replace(struct prio_tree_root *root, */
/*                 struct prio_tree_node *old, struct prio_tree_node *node); */
/* struct prio_tree_node *prio_tree_insert(struct prio_tree_root *root, */
/*                 struct prio_tree_node *node); */
/* void prio_tree_remove(struct prio_tree_root *root, struct prio_tree_node *node); */
/* struct prio_tree_node *prio_tree_next(struct prio_tree_iter *iter); */


/* struct hd_geometry; */
/* struct iovec; */
/* struct nameidata; */
/* struct kiocb; */
/* struct pipe_inode_info; */
/* struct poll_table_struct; */
/* struct kstatfs; */
/* struct vm_area_struct; */
/* struct vfsmount; */

/* extern void __attribute__ ((__section__ (".init.text"))) inode_init(unsigned long); */
/* extern void __attribute__ ((__section__ (".init.text"))) inode_init_early(void); */
/* extern void __attribute__ ((__section__ (".init.text"))) mnt_init(unsigned long); */
/* extern void __attribute__ ((__section__ (".init.text"))) files_init(unsigned long); */

/* struct buffer_head; */
/* typedef int (get_block_t)(struct inode *inode, sector_t iblock, */
/*    struct buffer_head *bh_result, int create); */
/* typedef void (dio_iodone_t)(struct kiocb *iocb, loff_t offset, */
/*    ssize_t bytes, void *private); */

/* struct iattr { */
/*  unsigned int ia_valid; */
/*  umode_t ia_mode; */
/*  uid_t ia_uid; */
/*  gid_t ia_gid; */
/*  loff_t ia_size; */
/*  struct timespec ia_atime; */
/*  struct timespec ia_mtime; */
/*  struct timespec ia_ctime; */






/*  struct file *ia_file; */
/* }; */






/* typedef __kernel_uid32_t qid_t; */
/* typedef __u64 qsize_t; */

/* extern spinlock_t dq_data_lock; */

/* struct if_dqblk { */
/*  __u64 dqb_bhardlimit; */
/*  __u64 dqb_bsoftlimit; */
/*  __u64 dqb_curspace; */
/*  __u64 dqb_ihardlimit; */
/*  __u64 dqb_isoftlimit; */
/*  __u64 dqb_curinodes; */
/*  __u64 dqb_btime; */
/*  __u64 dqb_itime; */
/*  __u32 dqb_valid; */
/* }; */

/* struct if_dqinfo { */
/*  __u64 dqi_bgrace; */
/*  __u64 dqi_igrace; */
/*  __u32 dqi_flags; */
/*  __u32 dqi_valid; */
/* }; */







/* typedef struct fs_disk_quota { */
/*  __s8 d_version; */
/*  __s8 d_flags; */
/*  __u16 d_fieldmask; */
/*  __u32 d_id; */
/*  __u64 d_blk_hardlimit; */
/*  __u64 d_blk_softlimit; */
/*  __u64 d_ino_hardlimit; */
/*  __u64 d_ino_softlimit; */
/*  __u64 d_bcount; */
/*  __u64 d_icount; */
/*  __s32 d_itimer; */

/*  __s32 d_btimer; */
/*  __u16 d_iwarns; */
/*  __u16 d_bwarns; */
/*  __s32 d_padding2; */
/*  __u64 d_rtb_hardlimit; */
/*  __u64 d_rtb_softlimit; */
/*  __u64 d_rtbcount; */
/*  __s32 d_rtbtimer; */
/*  __u16 d_rtbwarns; */
/*  __s16 d_padding3; */
/*  char d_padding4[8]; */
/* } fs_disk_quota_t; */

/* typedef struct fs_qfilestat { */
/*  __u64 qfs_ino; */
/*  __u64 qfs_nblks; */
/*  __u32 qfs_nextents; */
/* } fs_qfilestat_t; */

/* typedef struct fs_quota_stat { */
/*  __s8 qs_version; */
/*  __u16 qs_flags; */
/*  __s8 qs_pad; */
/*  fs_qfilestat_t qs_uquota; */
/*  fs_qfilestat_t qs_gquota; */
/*  __u32 qs_incoredqs; */
/*  __s32 qs_btimelimit; */
/*  __s32 qs_itimelimit; */
/*  __s32 qs_rtbtimelimit; */
/*  __u16 qs_bwarnlimit; */
/*  __u16 qs_iwarnlimit; */
/* } fs_quota_stat_t; */



/* struct v1_mem_dqinfo { */
/* }; */



/* struct v2_mem_dqinfo { */
/*  unsigned int dqi_blocks; */
/*  unsigned int dqi_free_blk; */
/*  unsigned int dqi_free_entry; */
/* }; */


/* struct mem_dqblk { */
/*  __u32 dqb_bhardlimit; */
/*  __u32 dqb_bsoftlimit; */
/*  qsize_t dqb_curspace; */
/*  __u32 dqb_ihardlimit; */
/*  __u32 dqb_isoftlimit; */
/*  __u32 dqb_curinodes; */
/*  time_t dqb_btime; */
/*  time_t dqb_itime; */
/* }; */




/* struct quota_format_type; */

/* struct mem_dqinfo { */
/*  struct quota_format_type *dqi_format; */
/*  struct list_head dqi_dirty_list; */
/*  unsigned long dqi_flags; */
/*  unsigned int dqi_bgrace; */
/*  unsigned int dqi_igrace; */
/*  union { */
/*   struct v1_mem_dqinfo v1_i; */
/*   struct v2_mem_dqinfo v2_i; */
/*  } u; */
/* }; */

/* struct super_block; */





/* extern void mark_info_dirty(struct super_block *sb, int type); */







/* struct dqstats { */
/*  int lookups; */
/*  int drops; */
/*  int reads; */
/*  int writes; */
/*  int cache_hits; */
/*  int allocated_dquots; */
/*  int free_dquots; */
/*  int syncs; */
/* }; */

/* extern struct dqstats dqstats; */

/* struct dquot { */
/*  struct hlist_node dq_hash; */
/*  struct list_head dq_inuse; */
/*  struct list_head dq_free; */
/*  struct list_head dq_dirty; */
/*  struct mutex dq_lock; */
/*  atomic_t dq_count; */
/*  wait_queue_head_t dq_wait_unused; */
/*  struct super_block *dq_sb; */
/*  unsigned int dq_id; */
/*  loff_t dq_off; */
/*  unsigned long dq_flags; */
/*  short dq_type; */
/*  struct mem_dqblk dq_dqb; */
/* }; */







/* struct quota_format_ops { */
/*  int (*check_quota_file)(struct super_block *sb, int type); */
/*  int (*read_file_info)(struct super_block *sb, int type); */
/*  int (*write_file_info)(struct super_block *sb, int type); */
/*  int (*free_file_info)(struct super_block *sb, int type); */
/*  int (*read_dqblk)(struct dquot *dquot); */
/*  int (*commit_dqblk)(struct dquot *dquot); */
/*  int (*release_dqblk)(struct dquot *dquot); */
/* }; */


/* struct dquot_operations { */
/*  int (*initialize) (struct inode *, int); */
/*  int (*drop) (struct inode *); */
/*  int (*alloc_space) (struct inode *, qsize_t, int); */
/*  int (*alloc_inode) (const struct inode *, unsigned long); */
/*  int (*free_space) (struct inode *, qsize_t); */
/*  int (*free_inode) (const struct inode *, unsigned long); */
/*  int (*transfer) (struct inode *, struct iattr *); */
/*  int (*write_dquot) (struct dquot *); */
/*  int (*acquire_dquot) (struct dquot *); */
/*  int (*release_dquot) (struct dquot *); */
/*  int (*mark_dirty) (struct dquot *); */
/*  int (*write_info) (struct super_block *, int); */
/* }; */


/* struct quotactl_ops { */
/*  int (*quota_on)(struct super_block *, int, int, char *); */
/*  int (*quota_off)(struct super_block *, int); */
/*  int (*quota_sync)(struct super_block *, int); */
/*  int (*get_info)(struct super_block *, int, struct if_dqinfo *); */
/*  int (*set_info)(struct super_block *, int, struct if_dqinfo *); */
/*  int (*get_dqblk)(struct super_block *, int, qid_t, struct if_dqblk *); */
/*  int (*set_dqblk)(struct super_block *, int, qid_t, struct if_dqblk *); */
/*  int (*get_xstate)(struct super_block *, struct fs_quota_stat *); */
/*  int (*set_xstate)(struct super_block *, unsigned int, int); */
/*  int (*get_xquota)(struct super_block *, int, qid_t, struct fs_disk_quota *); */
/*  int (*set_xquota)(struct super_block *, int, qid_t, struct fs_disk_quota *); */
/* }; */

/* struct quota_format_type { */
/*  int qf_fmt_id; */
/*  struct quota_format_ops *qf_ops; */
/*  struct module *qf_owner; */
/*  struct quota_format_type *qf_next; */
/* }; */




/* struct quota_info { */
/*  unsigned int flags; */
/*  struct mutex dqio_mutex; */
/*  struct mutex dqonoff_mutex; */
/*  struct rw_semaphore dqptr_sem; */
/*  struct inode *files[2]; */
/*  struct mem_dqinfo info[2]; */
/*  struct quota_format_ops *ops[2]; */
/* }; */


/* int mark_dquot_dirty(struct dquot *dquot); */

/* int register_quota_format(struct quota_format_type *fmt); */
/* void unregister_quota_format(struct quota_format_type *fmt); */

/* struct quota_module_name { */
/*  int qm_fmt_id; */
/*  char *qm_mod_name; */
/* }; */


/* enum positive_aop_returns { */
/*  AOP_WRITEPAGE_ACTIVATE = 0x80000, */
/*  AOP_TRUNCATED_PAGE = 0x80001, */
/* }; */




/* struct page; */
/* struct address_space; */
/* struct writeback_control; */

/* struct address_space_operations { */
/*  int (*writepage)(struct page *page, struct writeback_control *wbc); */
/*  int (*readpage)(struct file *, struct page *); */
/*  void (*sync_page)(struct page *); */


/*  int (*writepages)(struct address_space *, struct writeback_control *); */


/*  int (*set_page_dirty)(struct page *page); */

/*  int (*readpages)(struct file *filp, struct address_space *mapping, */
/*    struct list_head *pages, unsigned nr_pages); */





/*  int (*prepare_write)(struct file *, struct page *, unsigned, unsigned); */
/*  int (*commit_write)(struct file *, struct page *, unsigned, unsigned); */

/*  sector_t (*bmap)(struct address_space *, sector_t); */
/*  void (*invalidatepage) (struct page *, unsigned long); */
/*  int (*releasepage) (struct page *, gfp_t); */
/*  ssize_t (*direct_IO)(int, struct kiocb *, const struct iovec *iov, */
/*    loff_t offset, unsigned long nr_segs); */
/*  struct page* (*get_xip_page)(struct address_space *, sector_t, */
/*    int); */

/*  int (*migratepage) (struct address_space *, */
/*    struct page *, struct page *); */
/* }; */

/* struct backing_dev_info; */
/* struct address_space { */
/*  struct inode *host; */
/*  struct radix_tree_root page_tree; */
/*  rwlock_t tree_lock; */
/*  unsigned int i_mmap_writable; */
/*  struct prio_tree_root i_mmap; */
/*  struct list_head i_mmap_nonlinear; */
/*  spinlock_t i_mmap_lock; */
/*  unsigned int truncate_count; */
/*  unsigned long nrpages; */
/*  unsigned long writeback_index; */
/*  const struct address_space_operations *a_ops; */
/*  unsigned long flags; */
/*  struct backing_dev_info *backing_dev_info; */
/*  spinlock_t private_lock; */
/*  struct list_head private_list; */
/*  struct address_space *assoc_mapping; */
/* } __attribute__((aligned(sizeof(long)))); */






/* struct block_device { */
/*  dev_t bd_dev; */
/*  struct inode * bd_inode; */
/*  int bd_openers; */
/*  struct mutex bd_mutex; */
/*  struct mutex bd_mount_mutex; */
/*  struct list_head bd_inodes; */
/*  void * bd_holder; */
/*  int bd_holders; */

/*  struct list_head bd_holder_list; */

/*  struct block_device * bd_contains; */
/*  unsigned bd_block_size; */
/*  struct hd_struct * bd_part; */

/*  unsigned bd_part_count; */
/*  int bd_invalidated; */
/*  struct gendisk * bd_disk; */
/*  struct list_head bd_list; */
/*  struct backing_dev_info *bd_inode_backing_dev_info; */






/*  unsigned long bd_private; */
/* }; */

/* enum bdev_bd_mutex_lock_class */
/* { */
/*  BD_MUTEX_NORMAL, */
/*  BD_MUTEX_WHOLE, */
/*  BD_MUTEX_PARTITION */
/* }; */

/* int mapping_tagged(struct address_space *mapping, int tag); */




/* static inline __attribute__((always_inline)) int mapping_mapped(struct address_space *mapping) */
/* { */
/*  return !prio_tree_empty(&mapping->i_mmap) || */
/*   !list_empty(&mapping->i_mmap_nonlinear); */
/* } */







/* static inline __attribute__((always_inline)) int mapping_writably_mapped(struct address_space *mapping) */
/* { */
/*  return mapping->i_mmap_writable != 0; */
/* } */

/* struct inode { */
/*  struct hlist_node i_hash; */
/*  struct list_head i_list; */
/*  struct list_head i_sb_list; */
/*  struct list_head i_dentry; */
/*  unsigned long i_ino; */
/*  atomic_t i_count; */
/*  umode_t i_mode; */
/*  unsigned int i_nlink; */
/*  uid_t i_uid; */
/*  gid_t i_gid; */
/*  dev_t i_rdev; */
/*  loff_t i_size; */
/*  struct timespec i_atime; */
/*  struct timespec i_mtime; */
/*  struct timespec i_ctime; */
/*  unsigned int i_blkbits; */
/*  unsigned long i_blksize; */
/*  unsigned long i_version; */
/*  blkcnt_t i_blocks; */
/*  unsigned short i_bytes; */
/*  spinlock_t i_lock; */
/*  struct mutex i_mutex; */
/*  struct rw_semaphore i_alloc_sem; */
/*  struct inode_operations *i_op; */
/*  const struct file_operations *i_fop; */
/*  struct super_block *i_sb; */
/*  struct file_lock *i_flock; */
/*  struct address_space *i_mapping; */
/*  struct address_space i_data; */

/*  struct dquot *i_dquot[2]; */


/*  struct list_head i_devices; */
/*  struct pipe_inode_info *i_pipe; */
/*  struct block_device *i_bdev; */
/*  struct cdev *i_cdev; */
/*  int i_cindex; */

/*  __u32 i_generation; */


/*  unsigned long i_dnotify_mask; */
/*  struct dnotify_struct *i_dnotify; */



/*  struct list_head inotify_watches; */
/*  struct mutex inotify_mutex; */


/*  unsigned long i_state; */
/*  unsigned long dirtied_when; */

/*  unsigned int i_flags; */

/*  atomic_t i_writecount; */
/*  void *i_security; */
/*  union { */
/*   void *generic_ip; */
/*  } u; */



/* }; */

/* enum inode_i_mutex_lock_class */
/* { */
/*  I_MUTEX_NORMAL, */
/*  I_MUTEX_PARENT, */
/*  I_MUTEX_CHILD, */
/*  I_MUTEX_XATTR, */
/*  I_MUTEX_QUOTA */
/* }; */

/* static inline __attribute__((always_inline)) loff_t i_size_read(struct inode *inode) */
/* { */

/*  return inode->i_size; */

/* } */


/* static inline __attribute__((always_inline)) void i_size_write(struct inode *inode, loff_t i_size) */
/* { */

/*  inode->i_size = i_size; */

/* } */

/* static inline __attribute__((always_inline)) unsigned iminor(struct inode *inode) */
/* { */
/*  return ((unsigned int) ((inode->i_rdev) & ((1U << 20) - 1))); */
/* } */

/* static inline __attribute__((always_inline)) unsigned imajor(struct inode *inode) */
/* { */
/*  return ((unsigned int) ((inode->i_rdev) >> 20)); */
/* } */

/* extern struct block_device *I_BDEV(struct inode *inode); */

/* struct fown_struct { */
/*  rwlock_t lock; */
/*  int pid; */
/*  uid_t uid, euid; */
/*  void *security; */
/*  int signum; */
/* }; */




/* struct file_ra_state { */
/*  unsigned long start; */
/*  unsigned long size; */
/*  unsigned long flags; */
/*  unsigned long cache_hit; */
/*  unsigned long prev_page; */
/*  unsigned long ahead_start; */
/*  unsigned long ahead_size; */
/*  unsigned long ra_pages; */
/*  unsigned long mmap_hit; */
/*  unsigned long mmap_miss; */
/* }; */



/* struct file { */




/*  union { */
/*   struct list_head fu_list; */
/*   struct rcu_head fu_rcuhead; */
/*  } f_u; */
/*  struct dentry *f_dentry; */
/*  struct vfsmount *f_vfsmnt; */
/*  const struct file_operations *f_op; */
/*  atomic_t f_count; */
/*  unsigned int f_flags; */
/*  mode_t f_mode; */
/*  loff_t f_pos; */
/*  struct fown_struct f_owner; */
/*  unsigned int f_uid, f_gid; */
/*  struct file_ra_state f_ra; */

/*  unsigned long f_version; */
/*  void *f_security; */


/*  void *private_data; */



/*  struct list_head f_ep_links; */
/*  spinlock_t f_ep_lock; */

/*  struct address_space *f_mapping; */
/* }; */
/* extern spinlock_t files_lock; */

/* typedef struct files_struct *fl_owner_t; */

/* struct file_lock_operations { */
/*  void (*fl_insert)(struct file_lock *); */
/*  void (*fl_remove)(struct file_lock *); */
/*  void (*fl_copy_lock)(struct file_lock *, struct file_lock *); */
/*  void (*fl_release_private)(struct file_lock *); */
/* }; */

/* struct lock_manager_operations { */
/*  int (*fl_compare_owner)(struct file_lock *, struct file_lock *); */
/*  void (*fl_notify)(struct file_lock *); */
/*  void (*fl_copy_lock)(struct file_lock *, struct file_lock *); */
/*  void (*fl_release_private)(struct file_lock *); */
/*  void (*fl_break)(struct file_lock *); */
/*  int (*fl_mylease)(struct file_lock *, struct file_lock *); */
/*  int (*fl_change)(struct file_lock **, int); */
/* }; */










/*  enum nfs_stat { */
/*  NFS_OK = 0, */
/*  NFSERR_PERM = 1, */
/*  NFSERR_NOENT = 2, */
/*  NFSERR_IO = 5, */
/*  NFSERR_NXIO = 6, */
/*  NFSERR_EAGAIN = 11, */
/*  NFSERR_ACCES = 13, */
/*  NFSERR_EXIST = 17, */
/*  NFSERR_XDEV = 18, */
/*  NFSERR_NODEV = 19, */
/*  NFSERR_NOTDIR = 20, */
/*  NFSERR_ISDIR = 21, */
/*  NFSERR_INVAL = 22, */
/*  NFSERR_FBIG = 27, */
/*  NFSERR_NOSPC = 28, */
/*  NFSERR_ROFS = 30, */
/*  NFSERR_MLINK = 31, */
/*  NFSERR_OPNOTSUPP = 45, */
/*  NFSERR_NAMETOOLONG = 63, */
/*  NFSERR_NOTEMPTY = 66, */
/*  NFSERR_DQUOT = 69, */
/*  NFSERR_STALE = 70, */
/*  NFSERR_REMOTE = 71, */
/*  NFSERR_WFLUSH = 99, */
/*  NFSERR_BADHANDLE = 10001, */
/*  NFSERR_NOT_SYNC = 10002, */
/*  NFSERR_BAD_COOKIE = 10003, */
/*  NFSERR_NOTSUPP = 10004, */
/*  NFSERR_TOOSMALL = 10005, */
/*  NFSERR_SERVERFAULT = 10006, */
/*  NFSERR_BADTYPE = 10007, */
/*  NFSERR_JUKEBOX = 10008, */
/*  NFSERR_SAME = 10009, */
/*  NFSERR_DENIED = 10010, */
/*  NFSERR_EXPIRED = 10011, */
/*  NFSERR_LOCKED = 10012, */
/*  NFSERR_GRACE = 10013, */
/*  NFSERR_FHEXPIRED = 10014, */
/*  NFSERR_SHARE_DENIED = 10015, */
/*  NFSERR_WRONGSEC = 10016, */
/*  NFSERR_CLID_INUSE = 10017, */
/*  NFSERR_RESOURCE = 10018, */
/*  NFSERR_MOVED = 10019, */
/*  NFSERR_NOFILEHANDLE = 10020, */
/*  NFSERR_MINOR_VERS_MISMATCH = 10021, */
/*  NFSERR_STALE_CLIENTID = 10022, */
/*  NFSERR_STALE_STATEID = 10023, */
/*  NFSERR_OLD_STATEID = 10024, */
/*  NFSERR_BAD_STATEID = 10025, */
/*  NFSERR_BAD_SEQID = 10026, */
/*  NFSERR_NOT_SAME = 10027, */
/*  NFSERR_LOCK_RANGE = 10028, */
/*  NFSERR_SYMLINK = 10029, */
/*  NFSERR_RESTOREFH = 10030, */
/*  NFSERR_LEASE_MOVED = 10031, */
/*  NFSERR_ATTRNOTSUPP = 10032, */
/*  NFSERR_NO_GRACE = 10033, */
/*  NFSERR_RECLAIM_BAD = 10034, */
/*  NFSERR_RECLAIM_CONFLICT = 10035, */
/*  NFSERR_BAD_XDR = 10036, */
/*  NFSERR_LOCKS_HELD = 10037, */
/*  NFSERR_OPENMODE = 10038, */
/*  NFSERR_BADOWNER = 10039, */
/*  NFSERR_BADCHAR = 10040, */
/*  NFSERR_BADNAME = 10041, */
/*  NFSERR_BAD_RANGE = 10042, */
/*  NFSERR_LOCK_NOTSUPP = 10043, */
/*  NFSERR_OP_ILLEGAL = 10044, */
/*  NFSERR_DEADLOCK = 10045, */
/*  NFSERR_FILE_OPEN = 10046, */
/*  NFSERR_ADMIN_REVOKED = 10047, */
/*  NFSERR_CB_PATH_DOWN = 10048, */
/*  NFSERR_REPLAY_ME = 10049 */
/* }; */



/* enum nfs_ftype { */
/*  NFNON = 0, */
/*  NFREG = 1, */
/*  NFDIR = 2, */
/*  NFBLK = 3, */
/*  NFCHR = 4, */
/*  NFLNK = 5, */
/*  NFSOCK = 6, */
/*  NFBAD = 7, */
/*  NFFIFO = 8 */
/* }; */




/* typedef u32 rpc_authflavor_t; */

/* enum rpc_auth_flavors { */
/*  RPC_AUTH_NULL = 0, */
/*  RPC_AUTH_UNIX = 1, */
/*  RPC_AUTH_SHORT = 2, */
/*  RPC_AUTH_DES = 3, */
/*  RPC_AUTH_KRB = 4, */
/*  RPC_AUTH_GSS = 6, */
/*  RPC_AUTH_MAXFLAVOR = 8, */

/*  RPC_AUTH_GSS_KRB5 = 390003, */
/*  RPC_AUTH_GSS_KRB5I = 390004, */
/*  RPC_AUTH_GSS_KRB5P = 390005, */
/*  RPC_AUTH_GSS_LKEY = 390006, */
/*  RPC_AUTH_GSS_LKEYI = 390007, */
/*  RPC_AUTH_GSS_LKEYP = 390008, */
/*  RPC_AUTH_GSS_SPKM = 390009, */
/*  RPC_AUTH_GSS_SPKMI = 390010, */
/*  RPC_AUTH_GSS_SPKMP = 390011, */
/* }; */

/* enum rpc_msg_type { */
/*  RPC_CALL = 0, */
/*  RPC_REPLY = 1 */
/* }; */

/* enum rpc_reply_stat { */
/*  RPC_MSG_ACCEPTED = 0, */
/*  RPC_MSG_DENIED = 1 */
/* }; */

/* enum rpc_accept_stat { */
/*  RPC_SUCCESS = 0, */
/*  RPC_PROG_UNAVAIL = 1, */
/*  RPC_PROG_MISMATCH = 2, */
/*  RPC_PROC_UNAVAIL = 3, */
/*  RPC_GARBAGE_ARGS = 4, */
/*  RPC_SYSTEM_ERR = 5 */
/* }; */

/* enum rpc_reject_stat { */
/*  RPC_MISMATCH = 0, */
/*  RPC_AUTH_ERROR = 1 */
/* }; */

/* enum rpc_auth_stat { */
/*  RPC_AUTH_OK = 0, */
/*  RPC_AUTH_BADCRED = 1, */
/*  RPC_AUTH_REJECTEDCRED = 2, */
/*  RPC_AUTH_BADVERF = 3, */
/*  RPC_AUTH_REJECTEDVERF = 4, */
/*  RPC_AUTH_TOOWEAK = 5, */

/*  RPCSEC_GSS_CREDPROBLEM = 13, */
/*  RPCSEC_GSS_CTXPROBLEM = 14 */
/* }; */

/* typedef u32 rpc_fraghdr; */







/* struct nfs_fh { */
/*  unsigned short size; */
/*  unsigned char data[128]; */
/* }; */





/* static inline __attribute__((always_inline)) int nfs_compare_fh(const struct nfs_fh *a, const struct nfs_fh *b) */
/* { */
/*  return a->size != b->size || __builtin_memcmp(a->data, b->data, a->size) != 0; */
/* } */

/* static inline __attribute__((always_inline)) void nfs_copy_fh(struct nfs_fh *target, const struct nfs_fh *source) */
/* { */
/*  target->size = source->size; */
/*  (__builtin_constant_p(source->size) ? __constant_memcpy((target->data),(source->data),(source->size)) : __memcpy((target->data),(source->data),(source->size))); */
/* } */

/* enum nfs3_stable_how { */
/*  NFS_UNSTABLE = 0, */
/*  NFS_DATA_SYNC = 1, */
/*  NFS_FILE_SYNC = 2 */
/* }; */


/* struct nlm_lockowner; */




/* struct nfs_lock_info { */
/*  u32 state; */
/*  struct nlm_lockowner *owner; */
/*  struct list_head list; */
/* }; */

/* struct nfs4_lock_state; */
/* struct nfs4_lock_info { */
/*  struct nfs4_lock_state *owner; */
/* }; */


/* struct file_lock { */
/*  struct file_lock *fl_next; */
/*  struct list_head fl_link; */
/*  struct list_head fl_block; */
/*  fl_owner_t fl_owner; */
/*  unsigned int fl_pid; */
/*  wait_queue_head_t fl_wait; */
/*  struct file *fl_file; */
/*  unsigned char fl_flags; */
/*  unsigned char fl_type; */
/*  loff_t fl_start; */
/*  loff_t fl_end; */

/*  struct fasync_struct * fl_fasync; */
/*  unsigned long fl_break_time; */

/*  struct file_lock_operations *fl_ops; */
/*  struct lock_manager_operations *fl_lmops; */
/*  union { */
/*   struct nfs_lock_info nfs_fl; */
/*   struct nfs4_lock_info nfs4_fl; */
/*  } fl_u; */
/* }; */








/* struct flock { */
/*  short l_type; */
/*  short l_whence; */
/*  off_t l_start; */
/*  off_t l_len; */
/*  pid_t l_pid; */

/* }; */

/* struct flock64 { */
/*  short l_type; */
/*  short l_whence; */
/*  loff_t l_start; */
/*  loff_t l_len; */
/*  pid_t l_pid; */

/* }; */




/* extern int fcntl_getlk(struct file *, struct flock *); */
/* extern int fcntl_setlk(unsigned int, struct file *, unsigned int, */
/*    struct flock *); */


/* extern int fcntl_getlk64(struct file *, struct flock64 *); */
/* extern int fcntl_setlk64(unsigned int, struct file *, unsigned int, */
/*    struct flock64 *); */


/* extern void send_sigio(struct fown_struct *fown, int fd, int band); */
/* extern int fcntl_setlease(unsigned int fd, struct file *filp, long arg); */
/* extern int fcntl_getlease(struct file *filp); */


/* extern int do_sync_file_range(struct file *file, loff_t offset, loff_t endbyte, */
/*    unsigned int flags); */


/* extern void locks_init_lock(struct file_lock *); */
/* extern void locks_copy_lock(struct file_lock *, struct file_lock *); */
/* extern void locks_remove_posix(struct file *, fl_owner_t); */
/* extern void locks_remove_flock(struct file *); */
/* extern int posix_test_lock(struct file *, struct file_lock *, struct file_lock *); */
/* extern int posix_lock_file_conf(struct file *, struct file_lock *, struct file_lock *); */
/* extern int posix_lock_file(struct file *, struct file_lock *); */
/* extern int posix_lock_file_wait(struct file *, struct file_lock *); */
/* extern int posix_unblock_lock(struct file *, struct file_lock *); */
/* extern int flock_lock_file_wait(struct file *filp, struct file_lock *fl); */
/* extern int __break_lease(struct inode *inode, unsigned int flags); */
/* extern void lease_get_mtime(struct inode *, struct timespec *time); */
/* extern int setlease(struct file *, long, struct file_lock **); */
/* extern int lease_modify(struct file_lock **, int); */
/* extern int lock_may_read(struct inode *, loff_t start, unsigned long count); */
/* extern int lock_may_write(struct inode *, loff_t start, unsigned long count); */

/* struct fasync_struct { */
/*  int magic; */
/*  int fa_fd; */
/*  struct fasync_struct *fa_next; */
/*  struct file *fa_file; */
/* }; */




/* extern int fasync_helper(int, struct file *, int, struct fasync_struct **); */

/* extern void kill_fasync(struct fasync_struct **, int, int); */

/* extern void __kill_fasync(struct fasync_struct *, int, int); */

/* extern int f_setown(struct file *filp, unsigned long arg, int force); */
/* extern void f_delown(struct file *filp); */
/* extern int send_sigurg(struct fown_struct *fown); */

/* extern struct list_head super_blocks; */
/* extern spinlock_t sb_lock; */



/* struct super_block { */
/*  struct list_head s_list; */
/*  dev_t s_dev; */
/*  unsigned long s_blocksize; */
/*  unsigned char s_blocksize_bits; */
/*  unsigned char s_dirt; */
/*  unsigned long long s_maxbytes; */
/*  struct file_system_type *s_type; */
/*  struct super_operations *s_op; */
/*  struct dquot_operations *dq_op; */
/*   struct quotactl_ops *s_qcop; */
/*  struct export_operations *s_export_op; */
/*  unsigned long s_flags; */
/*  unsigned long s_magic; */
/*  struct dentry *s_root; */
/*  struct rw_semaphore s_umount; */
/*  struct mutex s_lock; */
/*  int s_count; */
/*  int s_syncing; */
/*  int s_need_sync_fs; */
/*  atomic_t s_active; */
/*  void *s_security; */
/*  struct xattr_handler **s_xattr; */

/*  struct list_head s_inodes; */
/*  struct list_head s_dirty; */
/*  struct list_head s_io; */
/*  struct hlist_head s_anon; */
/*  struct list_head s_files; */

/*  struct block_device *s_bdev; */
/*  struct list_head s_instances; */
/*  struct quota_info s_dquot; */

/*  int s_frozen; */
/*  wait_queue_head_t s_wait_unfrozen; */

/*  char s_id[32]; */

/*  void *s_fs_info; */





/*  struct mutex s_vfs_rename_mutex; */



/*  u32 s_time_gran; */
/* }; */

/* extern struct timespec current_fs_time(struct super_block *sb); */




/* enum { */
/*  SB_UNFROZEN = 0, */
/*  SB_FREEZE_WRITE = 1, */
/*  SB_FREEZE_TRANS = 2, */
/* }; */




/* static inline __attribute__((always_inline)) void get_fs_excl(void) */
/* { */
/*  atomic_inc(&__vericon_dummy_current->fs_excl); */
/* } */

/* static inline __attribute__((always_inline)) void put_fs_excl(void) */
/* { */
/*  atomic_dec(&__vericon_dummy_current->fs_excl); */
/* } */

/* static inline __attribute__((always_inline)) int has_fs_excl(void) */
/* { */
/*  return ((&__vericon_dummy_current->fs_excl)->counter); */
/* } */





/* static inline __attribute__((always_inline)) void lock_super(struct super_block * sb) */
/* { */
/*  get_fs_excl(); */
/*  mutex_lock(&sb->s_lock); */
/* } */

/* static inline __attribute__((always_inline)) void unlock_super(struct super_block * sb) */
/* { */
/*  put_fs_excl(); */
/*  mutex_unlock(&sb->s_lock); */
/* } */




/* extern int vfs_permission(struct nameidata *, int); */
/* extern int vfs_create(struct inode *, struct dentry *, int, struct nameidata *); */
/* extern int vfs_mkdir(struct inode *, struct dentry *, int); */
/* extern int vfs_mknod(struct inode *, struct dentry *, int, dev_t); */
/* extern int vfs_symlink(struct inode *, struct dentry *, const char *, int); */
/* extern int vfs_link(struct dentry *, struct inode *, struct dentry *); */
/* extern int vfs_rmdir(struct inode *, struct dentry *); */
/* extern int vfs_unlink(struct inode *, struct dentry *); */
/* extern int vfs_rename(struct inode *, struct dentry *, struct inode *, struct dentry *); */




/* extern void dentry_unhash(struct dentry *dentry); */




/* extern int file_permission(struct file *, int); */

/* int generic_osync_inode(struct inode *, struct address_space *, int); */







/* typedef int (*filldir_t)(void *, const char *, int, loff_t, ino_t, unsigned); */

/* struct block_device_operations { */
/*  int (*open) (struct inode *, struct file *); */
/*  int (*release) (struct inode *, struct file *); */
/*  int (*ioctl) (struct inode *, struct file *, unsigned, unsigned long); */
/*  long (*unlocked_ioctl) (struct file *, unsigned, unsigned long); */
/*  long (*compat_ioctl) (struct file *, unsigned, unsigned long); */
/*  int (*direct_access) (struct block_device *, sector_t, unsigned long *); */
/*  int (*media_changed) (struct gendisk *); */
/*  int (*revalidate_disk) (struct gendisk *); */
/*  int (*getgeo)(struct block_device *, struct hd_geometry *); */
/*  struct module *owner; */
/* }; */

/* typedef struct { */
/*  size_t written; */
/*  size_t count; */
/*  union { */
/*   char * buf; */
/*   void *data; */
/*  } arg; */
/*  int error; */
/* } read_descriptor_t; */

/* typedef int (*read_actor_t)(read_descriptor_t *, struct page *, unsigned long, unsigned long); */

/* struct file_operations { */
/*  struct module *owner; */
/*  loff_t (*llseek) (struct file *, loff_t, int); */
/*  ssize_t (*read) (struct file *, char *, size_t, loff_t *); */
/*  ssize_t (*aio_read) (struct kiocb *, char *, size_t, loff_t); */
/*  ssize_t (*write) (struct file *, const char *, size_t, loff_t *); */
/*  ssize_t (*aio_write) (struct kiocb *, const char *, size_t, loff_t); */
/*  int (*readdir) (struct file *, void *, filldir_t); */
/*  unsigned int (*poll) (struct file *, struct poll_table_struct *); */
/*  int (*ioctl) (struct inode *, struct file *, unsigned int, unsigned long); */
/*  long (*unlocked_ioctl) (struct file *, unsigned int, unsigned long); */
/*  long (*compat_ioctl) (struct file *, unsigned int, unsigned long); */
/*  int (*mmap) (struct file *, struct vm_area_struct *); */
/*  int (*open) (struct inode *, struct file *); */
/*  int (*flush) (struct file *, fl_owner_t id); */
/*  int (*release) (struct inode *, struct file *); */
/*  int (*fsync) (struct file *, struct dentry *, int datasync); */
/*  int (*aio_fsync) (struct kiocb *, int datasync); */
/*  int (*fasync) (int, struct file *, int); */
/*  int (*lock) (struct file *, int, struct file_lock *); */
/*  ssize_t (*readv) (struct file *, const struct iovec *, unsigned long, loff_t *); */
/*  ssize_t (*writev) (struct file *, const struct iovec *, unsigned long, loff_t *); */
/*  ssize_t (*sendfile) (struct file *, loff_t *, size_t, read_actor_t, void *); */
/*  ssize_t (*sendpage) (struct file *, struct page *, int, size_t, loff_t *, int); */
/*  unsigned long (*get_unmapped_area)(struct file *, unsigned long, unsigned long, unsigned long, unsigned long); */
/*  int (*check_flags)(int); */
/*  int (*dir_notify)(struct file *filp, unsigned long arg); */
/*  int (*flock) (struct file *, int, struct file_lock *); */
/*  ssize_t (*splice_write)(struct pipe_inode_info *, struct file *, loff_t *, size_t, unsigned int); */
/*  ssize_t (*splice_read)(struct file *, loff_t *, struct pipe_inode_info *, size_t, unsigned int); */
/* }; */

/* struct inode_operations { */
/*  int (*create) (struct inode *,struct dentry *,int, struct nameidata *); */
/*  struct dentry * (*lookup) (struct inode *,struct dentry *, struct nameidata *); */
/*  int (*link) (struct dentry *,struct inode *,struct dentry *); */
/*  int (*unlink) (struct inode *,struct dentry *); */
/*  int (*symlink) (struct inode *,struct dentry *,const char *); */
/*  int (*mkdir) (struct inode *,struct dentry *,int); */
/*  int (*rmdir) (struct inode *,struct dentry *); */
/*  int (*mknod) (struct inode *,struct dentry *,int,dev_t); */
/*  int (*rename) (struct inode *, struct dentry *, */
/*    struct inode *, struct dentry *); */
/*  int (*readlink) (struct dentry *, char *,int); */
/*  void * (*follow_link) (struct dentry *, struct nameidata *); */
/*  void (*put_link) (struct dentry *, struct nameidata *, void *); */
/*  void (*truncate) (struct inode *); */
/*  int (*permission) (struct inode *, int, struct nameidata *); */
/*  int (*setattr) (struct dentry *, struct iattr *); */
/*  int (*getattr) (struct vfsmount *mnt, struct dentry *, struct kstat *); */
/*  int (*setxattr) (struct dentry *, const char *,const void *,size_t,int); */
/*  ssize_t (*getxattr) (struct dentry *, const char *, void *, size_t); */
/*  ssize_t (*listxattr) (struct dentry *, char *, size_t); */
/*  int (*removexattr) (struct dentry *, const char *); */
/*  void (*truncate_range)(struct inode *, loff_t, loff_t); */
/* }; */

/* struct seq_file; */

/* extern ssize_t vfs_read(struct file *, char *, size_t, loff_t *); */
/* extern ssize_t vfs_write(struct file *, const char *, size_t, loff_t *); */
/* extern ssize_t vfs_readv(struct file *, const struct iovec *, */
/*   unsigned long, loff_t *); */
/* extern ssize_t vfs_writev(struct file *, const struct iovec *, */
/*   unsigned long, loff_t *); */





/* struct super_operations { */
/*     struct inode *(*alloc_inode)(struct super_block *sb); */
/*  void (*destroy_inode)(struct inode *); */

/*  void (*read_inode) (struct inode *); */

/*     void (*dirty_inode) (struct inode *); */
/*  int (*write_inode) (struct inode *, int); */
/*  void (*put_inode) (struct inode *); */
/*  void (*drop_inode) (struct inode *); */
/*  void (*delete_inode) (struct inode *); */
/*  void (*put_super) (struct super_block *); */
/*  void (*write_super) (struct super_block *); */
/*  int (*sync_fs)(struct super_block *sb, int wait); */
/*  void (*write_super_lockfs) (struct super_block *); */
/*  void (*unlockfs) (struct super_block *); */
/*  int (*statfs) (struct dentry *, struct kstatfs *); */
/*  int (*remount_fs) (struct super_block *, int *, char *); */
/*  void (*clear_inode) (struct inode *); */
/*  void (*umount_begin) (struct vfsmount *, int); */

/*  int (*show_options)(struct seq_file *, struct vfsmount *); */
/*  int (*show_stats)(struct seq_file *, struct vfsmount *); */

/*  ssize_t (*quota_read)(struct super_block *, int, char *, size_t, loff_t); */
/*  ssize_t (*quota_write)(struct super_block *, int, const char *, size_t, loff_t); */
/* }; */

/* extern void __mark_inode_dirty(struct inode *, int); */
/* static inline __attribute__((always_inline)) void mark_inode_dirty(struct inode *inode) */
/* { */
/*  __mark_inode_dirty(inode, (1 | 2 | 4)); */
/* } */

/* static inline __attribute__((always_inline)) void mark_inode_dirty_sync(struct inode *inode) */
/* { */
/*  __mark_inode_dirty(inode, 1); */
/* } */

/* static inline __attribute__((always_inline)) void inode_inc_link_count(struct inode *inode) */
/* { */
/*  inode->i_nlink++; */
/*  mark_inode_dirty(inode); */
/* } */

/* static inline __attribute__((always_inline)) void inode_dec_link_count(struct inode *inode) */
/* { */
/*  inode->i_nlink--; */
/*  mark_inode_dirty(inode); */
/* } */

/* extern void touch_atime(struct vfsmount *mnt, struct dentry *dentry); */
/* static inline __attribute__((always_inline)) void file_accessed(struct file *file) */
/* { */
/*  if (!(file->f_flags & 01000000)) */
/*   touch_atime(file->f_vfsmnt, file->f_dentry); */
/* } */

/* int sync_inode(struct inode *inode, struct writeback_control *wbc); */

/* struct export_operations { */
/*  struct dentry *(*decode_fh)(struct super_block *sb, __u32 *fh, int fh_len, int fh_type, */
/*     int (*acceptable)(void *context, struct dentry *de), */
/*     void *context); */
/*  int (*encode_fh)(struct dentry *de, __u32 *fh, int *max_len, */
/*     int connectable); */


/*  int (*get_name)(struct dentry *parent, char *name, */
/*    struct dentry *child); */
/*  struct dentry * (*get_parent)(struct dentry *child); */
/*  struct dentry * (*get_dentry)(struct super_block *sb, void *inump); */


/*  struct dentry * (*find_exported_dentry)( */
/*   struct super_block *sb, void *obj, void *parent, */
/*   int (*acceptable)(void *context, struct dentry *de), */
/*   void *context); */


/* }; */

/* extern struct dentry * */
/* find_exported_dentry(struct super_block *sb, void *obj, void *parent, */
/*        int (*acceptable)(void *context, struct dentry *de), */
/*        void *context); */

/* struct file_system_type { */
/*  const char *name; */
/*  int fs_flags; */
/*  int (*get_sb) (struct file_system_type *, int, */
/*          const char *, void *, struct vfsmount *); */
/*  void (*kill_sb) (struct super_block *); */
/*  struct module *owner; */
/*  struct file_system_type * next; */
/*  struct list_head fs_supers; */
/*  struct lock_class_key s_lock_key; */
/*  struct lock_class_key s_umount_key; */
/* }; */

/* extern int get_sb_bdev(struct file_system_type *fs_type, */
/*  int flags, const char *dev_name, void *data, */
/*  int (*fill_super)(struct super_block *, void *, int), */
/*  struct vfsmount *mnt); */
/* extern int get_sb_single(struct file_system_type *fs_type, */
/*  int flags, void *data, */
/*  int (*fill_super)(struct super_block *, void *, int), */
/*  struct vfsmount *mnt); */
/* extern int get_sb_nodev(struct file_system_type *fs_type, */
/*  int flags, void *data, */
/*  int (*fill_super)(struct super_block *, void *, int), */
/*  struct vfsmount *mnt); */
/* void generic_shutdown_super(struct super_block *sb); */
/* void kill_block_super(struct super_block *sb); */
/* void kill_anon_super(struct super_block *sb); */
/* void kill_litter_super(struct super_block *sb); */
/* void deactivate_super(struct super_block *sb); */
/* int set_anon_super(struct super_block *s, void *data); */
/* struct super_block *sget(struct file_system_type *type, */
/*    int (*test)(struct super_block *,void *), */
/*    int (*set)(struct super_block *,void *), */
/*    void *data); */
/* extern int get_sb_pseudo(struct file_system_type *, char *, */
/*  struct super_operations *ops, unsigned long, */
/*  struct vfsmount *mnt); */
/* extern int simple_set_mnt(struct vfsmount *mnt, struct super_block *sb); */
/* int __put_super(struct super_block *sb); */
/* int __put_super_and_need_restart(struct super_block *sb); */
/* void unnamed_dev_init(void); */







/* extern int register_filesystem(struct file_system_type *); */
/* extern int unregister_filesystem(struct file_system_type *); */
/* extern struct vfsmount *kern_mount(struct file_system_type *); */
/* extern int may_umount_tree(struct vfsmount *); */
/* extern int may_umount(struct vfsmount *); */
/* extern void umount_tree(struct vfsmount *, int, struct list_head *); */
/* extern void release_mounts(struct list_head *); */
/* extern long do_mount(char *, char *, char *, unsigned long, void *); */
/* extern struct vfsmount *copy_tree(struct vfsmount *, struct dentry *, int); */
/* extern void mnt_set_mountpoint(struct vfsmount *, struct dentry *, */
/*       struct vfsmount *); */

/* extern int vfs_statfs(struct dentry *, struct kstatfs *); */


/* extern struct subsystem fs_subsys; */




/* extern int locks_mandatory_locked(struct inode *); */
/* extern int locks_mandatory_area(int, struct inode *, struct file *, loff_t, size_t); */

/* static inline __attribute__((always_inline)) int locks_verify_locked(struct inode *inode) */
/* { */
/*  if ((((inode)->i_sb->s_flags & (64)) && ((inode)->i_mode & (0002000 | 00010)) == 0002000)) */
/*   return locks_mandatory_locked(inode); */
/*  return 0; */
/* } */

/* extern int rw_verify_area(int, struct file *, loff_t *, size_t); */

/* static inline __attribute__((always_inline)) int locks_verify_truncate(struct inode *inode, */
/*         struct file *filp, */
/*         loff_t size) */
/* { */
/*  if (inode->i_flock && (((inode)->i_sb->s_flags & (64)) && ((inode)->i_mode & (0002000 | 00010)) == 0002000)) */
/*   return locks_mandatory_area( */
/*    2, inode, filp, */
/*    size < inode->i_size ? size : inode->i_size, */
/*    (size < inode->i_size ? inode->i_size - size */
/*     : size - inode->i_size) */
/*   ); */
/*  return 0; */
/* } */

/* static inline __attribute__((always_inline)) int break_lease(struct inode *inode, unsigned int mode) */
/* { */
/*  if (inode->i_flock) */
/*   return __break_lease(inode, mode); */
/*  return 0; */
/* } */



/* extern int do_truncate(struct dentry *, loff_t start, unsigned int time_attrs, */
/*          struct file *filp); */
/* extern long do_sys_open(int fdf, const char *filename, int flags, */
/*    int mode); */
/* extern struct file *filp_open(const char *, int, int); */
/* extern struct file * dentry_open(struct dentry *, struct vfsmount *, int); */
/* extern int filp_close(struct file *, fl_owner_t id); */
/* extern char * getname(const char *); */


/* extern void __attribute__ ((__section__ (".init.text"))) vfs_caches_init_early(void); */
/* extern void __attribute__ ((__section__ (".init.text"))) vfs_caches_init(unsigned long); */






/* extern void putname(const char *name); */


/* extern int register_blkdev(unsigned int, const char *); */
/* extern int unregister_blkdev(unsigned int, const char *); */
/* extern struct block_device *bdget(dev_t); */
/* extern void bd_set_size(struct block_device *, loff_t size); */
/* extern void bd_forget(struct inode *inode); */
/* extern void bdput(struct block_device *); */
/* extern struct block_device *open_by_devnum(dev_t, unsigned); */
/* extern struct block_device *open_partition_by_devnum(dev_t, unsigned); */
/* extern const struct file_operations def_blk_fops; */
/* extern const struct address_space_operations def_blk_aops; */
/* extern const struct file_operations def_chr_fops; */
/* extern const struct file_operations bad_sock_fops; */
/* extern const struct file_operations def_fifo_fops; */
/* extern int ioctl_by_bdev(struct block_device *, unsigned, unsigned long); */
/* extern int blkdev_ioctl(struct inode *, struct file *, unsigned, unsigned long); */
/* extern long compat_blkdev_ioctl(struct file *, unsigned, unsigned long); */
/* extern int blkdev_get(struct block_device *, mode_t, unsigned); */
/* extern int blkdev_put(struct block_device *); */
/* extern int blkdev_put_partition(struct block_device *); */
/* extern int bd_claim(struct block_device *, void *); */
/* extern void bd_release(struct block_device *); */

/* extern int bd_claim_by_disk(struct block_device *, void *, struct gendisk *); */
/* extern void bd_release_from_disk(struct block_device *, struct gendisk *); */







/* extern int alloc_chrdev_region(dev_t *, unsigned, unsigned, const char *); */
/* extern int register_chrdev_region(dev_t, unsigned, const char *); */
/* extern int register_chrdev(unsigned int, const char *, */
/*       const struct file_operations *); */
/* extern int unregister_chrdev(unsigned int, const char *); */
/* extern void unregister_chrdev_region(dev_t, unsigned); */
/* extern int chrdev_open(struct inode *, struct file *); */
/* extern void chrdev_show(struct seq_file *,off_t); */




/* extern const char *__bdevname(dev_t, char *buffer); */
/* extern const char *bdevname(struct block_device *bdev, char *buffer); */
/* extern struct block_device *lookup_bdev(const char *); */
/* extern struct block_device *open_bdev_excl(const char *, int, void *); */
/* extern void close_bdev_excl(struct block_device *); */
/* extern void blkdev_show(struct seq_file *,off_t); */

/* extern void init_special_inode(struct inode *, umode_t, dev_t); */


/* extern void make_bad_inode(struct inode *); */
/* extern int is_bad_inode(struct inode *); */

/* extern const struct file_operations read_fifo_fops; */
/* extern const struct file_operations write_fifo_fops; */
/* extern const struct file_operations rdwr_fifo_fops; */

/* extern int fs_may_remount_ro(struct super_block *); */

/* extern int check_disk_change(struct block_device *); */
/* extern int invalidate_inodes(struct super_block *); */
/* extern int __invalidate_device(struct block_device *); */
/* extern int invalidate_partition(struct gendisk *, int); */
/* unsigned long invalidate_mapping_pages(struct address_space *mapping, */
/*      unsigned long start, unsigned long end); */
/* unsigned long invalidate_inode_pages(struct address_space *mapping); */
/* static inline __attribute__((always_inline)) void invalidate_remote_inode(struct inode *inode) */
/* { */
/*  if ((((inode->i_mode) & 00170000) == 0100000) || (((inode->i_mode) & 00170000) == 0040000) || */
/*      (((inode->i_mode) & 00170000) == 0120000)) */
/*   invalidate_inode_pages(inode->i_mapping); */
/* } */
/* extern int invalidate_inode_pages2(struct address_space *mapping); */
/* extern int invalidate_inode_pages2_range(struct address_space *mapping, */
/*       unsigned long start, unsigned long end); */
/* extern int write_inode_now(struct inode *, int); */
/* extern int filemap_fdatawrite(struct address_space *); */
/* extern int filemap_flush(struct address_space *); */
/* extern int filemap_fdatawait(struct address_space *); */
/* extern int filemap_write_and_wait(struct address_space *mapping); */
/* extern int filemap_write_and_wait_range(struct address_space *mapping, */
/*             loff_t lstart, loff_t lend); */
/* extern int wait_on_page_writeback_range(struct address_space *mapping, */
/*     unsigned long start, unsigned long end); */
/* extern int __filemap_fdatawrite_range(struct address_space *mapping, */
/*     loff_t start, loff_t end, int sync_mode); */

/* extern long do_fsync(struct file *file, int datasync); */
/* extern void sync_supers(void); */
/* extern void sync_filesystems(int wait); */
/* extern void emergency_sync(void); */
/* extern void emergency_remount(void); */
/* extern int do_remount_sb(struct super_block *sb, int flags, */
/*     void *data, int force); */
/* extern sector_t bmap(struct inode *, sector_t); */
/* extern int notify_change(struct dentry *, struct iattr *); */
/* extern int permission(struct inode *, int, struct nameidata *); */
/* extern int generic_permission(struct inode *, int, */
/*   int (*check_acl)(struct inode *, int)); */

/* extern int get_write_access(struct inode *); */
/* extern int deny_write_access(struct file *); */
/* static inline __attribute__((always_inline)) void put_write_access(struct inode * inode) */
/* { */
/*  atomic_dec(&inode->i_writecount); */
/* } */
/* static inline __attribute__((always_inline)) void allow_write_access(struct file *file) */
/* { */
/*  if (file) */
/*   atomic_inc(&file->f_dentry->d_inode->i_writecount); */
/* } */
/* extern int do_pipe(int *); */

/* extern int open_namei(int dfd, const char *, int, int, struct nameidata *); */
/* extern int may_open(struct nameidata *, int, int); */

/* extern int kernel_read(struct file *, unsigned long, char *, unsigned long); */
/* extern struct file * open_exec(const char *); */


/* extern int is_subdir(struct dentry *, struct dentry *); */
/* extern ino_t find_inode_number(struct dentry *, struct qstr *); */



/* static inline __attribute__((always_inline)) void *ERR_PTR(long error) */
/* { */
/*  return (void *) error; */
/* } */

/* static inline __attribute__((always_inline)) long PTR_ERR(const void *ptr) */
/* { */
/*  return (long) ptr; */
/* } */

/* static inline __attribute__((always_inline)) long IS_ERR(const void *ptr) */
/* { */
/*  return __builtin_expect(!!(((unsigned long)ptr) >= (unsigned long)-4095), 0); */
/* } */



/* extern loff_t default_llseek(struct file *file, loff_t offset, int origin); */

/* extern loff_t vfs_llseek(struct file *file, loff_t offset, int origin); */

/* extern void inode_init_once(struct inode *); */
/* extern void iput(struct inode *); */
/* extern struct inode * igrab(struct inode *); */
/* extern ino_t iunique(struct super_block *, ino_t); */
/* extern int inode_needs_sync(struct inode *inode); */
/* extern void generic_delete_inode(struct inode *inode); */
/* extern void generic_drop_inode(struct inode *inode); */

/* extern struct inode *ilookup5_nowait(struct super_block *sb, */
/*   unsigned long hashval, int (*test)(struct inode *, void *), */
/*   void *data); */
/* extern struct inode *ilookup5(struct super_block *sb, unsigned long hashval, */
/*   int (*test)(struct inode *, void *), void *data); */
/* extern struct inode *ilookup(struct super_block *sb, unsigned long ino); */

/* extern struct inode * iget5_locked(struct super_block *, unsigned long, int (*test)(struct inode *, void *), int (*set)(struct inode *, void *), void *); */
/* extern struct inode * iget_locked(struct super_block *, unsigned long); */
/* extern void unlock_new_inode(struct inode *); */

/* static inline __attribute__((always_inline)) struct inode *iget(struct super_block *sb, unsigned long ino) */
/* { */
/*  struct inode *inode = iget_locked(sb, ino); */

/*  if (inode && (inode->i_state & 64)) { */
/*   sb->s_op->read_inode(inode); */
/*   unlock_new_inode(inode); */
/*  } */

/*  return inode; */
/* } */

/* extern void __iget(struct inode * inode); */
/* extern void clear_inode(struct inode *); */
/* extern void destroy_inode(struct inode *); */
/* extern struct inode *new_inode(struct super_block *); */
/* extern int remove_suid(struct dentry *); */
/* extern void remove_dquot_ref(struct super_block *, int, struct list_head *); */

/* extern void __insert_inode_hash(struct inode *, unsigned long hashval); */
/* extern void remove_inode_hash(struct inode *); */
/* static inline __attribute__((always_inline)) void insert_inode_hash(struct inode *inode) { */
/*  __insert_inode_hash(inode, inode->i_ino); */
/* } */

/* extern struct file * get_empty_filp(void); */
/* extern void file_move(struct file *f, struct list_head *list); */
/* extern void file_kill(struct file *f); */
/* struct bio; */
/* extern void submit_bio(int, struct bio *); */
/* extern int bdev_read_only(struct block_device *); */
/* extern int set_blocksize(struct block_device *, int); */
/* extern int sb_set_blocksize(struct super_block *, int); */
/* extern int sb_min_blocksize(struct super_block *, int); */

/* extern int generic_file_mmap(struct file *, struct vm_area_struct *); */
/* extern int generic_file_readonly_mmap(struct file *, struct vm_area_struct *); */
/* extern int file_read_actor(read_descriptor_t * desc, struct page *page, unsigned long offset, unsigned long size); */
/* extern int file_send_actor(read_descriptor_t * desc, struct page *page, unsigned long offset, unsigned long size); */
/* extern ssize_t generic_file_read(struct file *, char *, size_t, loff_t *); */
/* int generic_write_checks(struct file *file, loff_t *pos, size_t *count, int isblk); */
/* extern ssize_t generic_file_write(struct file *, const char *, size_t, loff_t *); */
/* extern ssize_t generic_file_aio_read(struct kiocb *, char *, size_t, loff_t); */
/* extern ssize_t __generic_file_aio_read(struct kiocb *, const struct iovec *, unsigned long, loff_t *); */
/* extern ssize_t generic_file_aio_write(struct kiocb *, const char *, size_t, loff_t); */
/* extern ssize_t generic_file_aio_write_nolock(struct kiocb *, const struct iovec *, */
/*   unsigned long, loff_t *); */
/* extern ssize_t generic_file_direct_write(struct kiocb *, const struct iovec *, */
/*   unsigned long *, loff_t, loff_t *, size_t, size_t); */
/* extern ssize_t generic_file_buffered_write(struct kiocb *, const struct iovec *, */
/*   unsigned long, loff_t, loff_t *, size_t, ssize_t); */
/* extern ssize_t do_sync_read(struct file *filp, char *buf, size_t len, loff_t *ppos); */
/* extern ssize_t do_sync_write(struct file *filp, const char *buf, size_t len, loff_t *ppos); */
/* ssize_t generic_file_write_nolock(struct file *file, const struct iovec *iov, */
/*     unsigned long nr_segs, loff_t *ppos); */
/* extern ssize_t generic_file_sendfile(struct file *, loff_t *, size_t, read_actor_t, void *); */
/* extern void do_generic_mapping_read(struct address_space *mapping, */
/*         struct file_ra_state *, struct file *, */
/*         loff_t *, read_descriptor_t *, read_actor_t); */


/* extern ssize_t generic_file_splice_read(struct file *, loff_t *, */
/*   struct pipe_inode_info *, size_t, unsigned int); */
/* extern ssize_t generic_file_splice_write(struct pipe_inode_info *, */
/*   struct file *, loff_t *, size_t, unsigned int); */
/* extern ssize_t generic_splice_sendpage(struct pipe_inode_info *pipe, */
/*   struct file *out, loff_t *, size_t len, unsigned int flags); */
/* extern long do_splice_direct(struct file *in, loff_t *ppos, struct file *out, */
/*   size_t len, unsigned int flags); */

/* extern void */
/* file_ra_state_init(struct file_ra_state *ra, struct address_space *mapping); */
/* extern ssize_t generic_file_readv(struct file *filp, const struct iovec *iov, */
/*  unsigned long nr_segs, loff_t *ppos); */
/* ssize_t generic_file_writev(struct file *filp, const struct iovec *iov, */
/*    unsigned long nr_segs, loff_t *ppos); */
/* extern loff_t no_llseek(struct file *file, loff_t offset, int origin); */
/* extern loff_t generic_file_llseek(struct file *file, loff_t offset, int origin); */
/* extern loff_t remote_llseek(struct file *file, loff_t offset, int origin); */
/* extern int generic_file_open(struct inode * inode, struct file * filp); */
/* extern int nonseekable_open(struct inode * inode, struct file * filp); */


/* extern ssize_t xip_file_read(struct file *filp, char *buf, size_t len, */
/*         loff_t *ppos); */
/* extern ssize_t xip_file_sendfile(struct file *in_file, loff_t *ppos, */
/*      size_t count, read_actor_t actor, */
/*      void *target); */
/* extern int xip_file_mmap(struct file * file, struct vm_area_struct * vma); */
/* extern ssize_t xip_file_write(struct file *filp, const char *buf, */
/*          size_t len, loff_t *ppos); */
/* extern int xip_truncate_page(struct address_space *mapping, loff_t from); */







/* static inline __attribute__((always_inline)) void do_generic_file_read(struct file * filp, loff_t *ppos, */
/*      read_descriptor_t * desc, */
/*      read_actor_t actor) */
/* { */
/*  do_generic_mapping_read(filp->f_mapping, */
/*     &filp->f_ra, */
/*     filp, */
/*     ppos, */
/*     desc, */
/*     actor); */
/* } */

/* ssize_t __blockdev_direct_IO(int rw, struct kiocb *iocb, struct inode *inode, */
/*  struct block_device *bdev, const struct iovec *iov, loff_t offset, */
/*  unsigned long nr_segs, get_block_t get_block, dio_iodone_t end_io, */
/*  int lock_type); */

/* enum { */
/*  DIO_LOCKING = 1, */
/*  DIO_NO_LOCKING, */
/*  DIO_OWN_LOCKING, */
/* }; */

/* static inline __attribute__((always_inline)) ssize_t blockdev_direct_IO(int rw, struct kiocb *iocb, */
/*  struct inode *inode, struct block_device *bdev, const struct iovec *iov, */
/*  loff_t offset, unsigned long nr_segs, get_block_t get_block, */
/*  dio_iodone_t end_io) */
/* { */
/*  return __blockdev_direct_IO(rw, iocb, inode, bdev, iov, offset, */
/*     nr_segs, get_block, end_io, DIO_LOCKING); */
/* } */

/* static inline __attribute__((always_inline)) ssize_t blockdev_direct_IO_no_locking(int rw, struct kiocb *iocb, */
/*  struct inode *inode, struct block_device *bdev, const struct iovec *iov, */
/*  loff_t offset, unsigned long nr_segs, get_block_t get_block, */
/*  dio_iodone_t end_io) */
/* { */
/*  return __blockdev_direct_IO(rw, iocb, inode, bdev, iov, offset, */
/*     nr_segs, get_block, end_io, DIO_NO_LOCKING); */
/* } */

/* static inline __attribute__((always_inline)) ssize_t blockdev_direct_IO_own_locking(int rw, struct kiocb *iocb, */
/*  struct inode *inode, struct block_device *bdev, const struct iovec *iov, */
/*  loff_t offset, unsigned long nr_segs, get_block_t get_block, */
/*  dio_iodone_t end_io) */
/* { */
/*  return __blockdev_direct_IO(rw, iocb, inode, bdev, iov, offset, */
/*     nr_segs, get_block, end_io, DIO_OWN_LOCKING); */
/* } */

/* extern const struct file_operations generic_ro_fops; */



/* extern int vfs_readlink(struct dentry *, char *, int, const char *); */
/* extern int vfs_follow_link(struct nameidata *, const char *); */
/* extern int page_readlink(struct dentry *, char *, int); */
/* extern void *page_follow_link_light(struct dentry *, struct nameidata *); */
/* extern void page_put_link(struct dentry *, struct nameidata *, void *); */
/* extern int __page_symlink(struct inode *inode, const char *symname, int len, */
/*   gfp_t gfp_mask); */
/* extern int page_symlink(struct inode *inode, const char *symname, int len); */
/* extern struct inode_operations page_symlink_inode_operations; */
/* extern int generic_readlink(struct dentry *, char *, int); */
/* extern void generic_fillattr(struct inode *, struct kstat *); */
/* extern int vfs_getattr(struct vfsmount *, struct dentry *, struct kstat *); */
/* void inode_add_bytes(struct inode *inode, loff_t bytes); */
/* void inode_sub_bytes(struct inode *inode, loff_t bytes); */
/* loff_t inode_get_bytes(struct inode *inode); */
/* void inode_set_bytes(struct inode *inode, loff_t bytes); */

/* extern int vfs_readdir(struct file *, filldir_t, void *); */

/* extern int vfs_stat(char *, struct kstat *); */
/* extern int vfs_lstat(char *, struct kstat *); */
/* extern int vfs_stat_fd(int dfd, char *, struct kstat *); */
/* extern int vfs_lstat_fd(int dfd, char *, struct kstat *); */
/* extern int vfs_fstat(unsigned int, struct kstat *); */

/* extern int vfs_ioctl(struct file *, unsigned int, unsigned int, unsigned long); */

/* extern struct file_system_type *get_fs_type(const char *name); */
/* extern struct super_block *get_super(struct block_device *); */
/* extern struct super_block *user_get_super(dev_t); */
/* extern void drop_super(struct super_block *sb); */

/* extern int dcache_dir_open(struct inode *, struct file *); */
/* extern int dcache_dir_close(struct inode *, struct file *); */
/* extern loff_t dcache_dir_lseek(struct file *, loff_t, int); */
/* extern int dcache_readdir(struct file *, void *, filldir_t); */
/* extern int simple_getattr(struct vfsmount *, struct dentry *, struct kstat *); */
/* extern int simple_statfs(struct dentry *, struct kstatfs *); */
/* extern int simple_link(struct dentry *, struct inode *, struct dentry *); */
/* extern int simple_unlink(struct inode *, struct dentry *); */
/* extern int simple_rmdir(struct inode *, struct dentry *); */
/* extern int simple_rename(struct inode *, struct dentry *, struct inode *, struct dentry *); */
/* extern int simple_sync_file(struct file *, struct dentry *, int); */
/* extern int simple_empty(struct dentry *); */
/* extern int simple_readpage(struct file *file, struct page *page); */
/* extern int simple_prepare_write(struct file *file, struct page *page, */
/*    unsigned offset, unsigned to); */
/* extern int simple_commit_write(struct file *file, struct page *page, */
/*     unsigned offset, unsigned to); */

/* extern struct dentry *simple_lookup(struct inode *, struct dentry *, struct nameidata *); */
/* extern ssize_t generic_read_dir(struct file *, char *, size_t, loff_t *); */
/* extern const struct file_operations simple_dir_operations; */
/* extern struct inode_operations simple_dir_inode_operations; */
/* struct tree_descr { char *name; const struct file_operations *ops; int mode; }; */
/* struct dentry *d_alloc_name(struct dentry *, const char *); */
/* extern int simple_fill_super(struct super_block *, int, struct tree_descr *); */
/* extern int simple_pin_fs(struct file_system_type *, struct vfsmount **mount, int *count); */
/* extern void simple_release_fs(struct vfsmount **mount, int *count); */

/* extern ssize_t simple_read_from_buffer(void *, size_t, loff_t *, const void *, size_t); */

/* extern int inode_change_ok(struct inode *, struct iattr *); */
/* extern int __attribute__((warn_unused_result)) inode_setattr(struct inode *, struct iattr *); */

/* extern void file_update_time(struct file *file); */

/* static inline __attribute__((always_inline)) ino_t parent_ino(struct dentry *dentry) */
/* { */
/*  ino_t res; */

/*  _spin_lock(&dentry->d_lock); */
/*  res = dentry->d_parent->d_inode->i_ino; */
/*  _spin_unlock(&dentry->d_lock); */
/*  return res; */
/* } */


/* extern int unshare_files(void); */







/* struct simple_transaction_argresp { */
/*  ssize_t size; */
/*  char data[0]; */
/* }; */



/* char *simple_transaction_get(struct file *file, const char *buf, */
/*     size_t size); */
/* ssize_t simple_transaction_read(struct file *file, char *buf, */
/*     size_t size, loff_t *pos); */
/* int simple_transaction_release(struct inode *inode, struct file *file); */

/* static inline __attribute__((always_inline)) void simple_transaction_set(struct file *file, size_t n) */
/* { */
/*  struct simple_transaction_argresp *ar = file->private_data; */

/*  do { if (__builtin_expect(!!((n > ((1UL << 12) - sizeof(struct simple_transaction_argresp)))!=0), 0)) __asm__ __volatile__( "ud2\n" "\t.word %c0\n" "\t.long %c1\n" : : "i" (1867), "i" ("include/linux/fs.h")); } while(0); */





/*  __asm__ __volatile__("": : :"memory"); */
/*  ar->size = n; */
/* } */

/* static inline __attribute__((always_inline)) void __attribute__((format(printf, 1, 2))) */
/* __simple_attr_check_format(const char *fmt, ...) */
/* { */

/* } */

/* int simple_attr_open(struct inode *inode, struct file *file, */
/*        u64 (*get)(void *), void (*set)(void *, u64), */
/*        const char *fmt); */
/* int simple_attr_close(struct inode *inode, struct file *file); */
/* ssize_t simple_attr_read(struct file *file, char *buf, */
/*     size_t len, loff_t *ppos); */
/* ssize_t simple_attr_write(struct file *file, const char *buf, */
/*      size_t len, loff_t *ppos); */



/* static inline __attribute__((always_inline)) char *alloc_secdata(void) */
/* { */
/*  return (char *)get_zeroed_page(((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u))); */
/* } */

/* static inline __attribute__((always_inline)) void free_secdata(void *secdata) */
/* { */
/*  free_pages(((unsigned long)secdata),0); */
/* } */












/* struct mempolicy; */
/* struct anon_vma; */


/* extern unsigned long max_mapnr; */


/* extern unsigned long num_physpages; */
/* extern void * high_memory; */
/* extern unsigned long vmalloc_earlyreserve; */
/* extern int page_cluster; */


/* extern int sysctl_legacy_va_layout; */














/* static inline __attribute__((always_inline)) int */
/* __acpi_acquire_global_lock (unsigned int *lock) */
/* { */
/*  unsigned int old, new, val; */
/*  do { */
/*   old = *lock; */
/*   new = (((old & ~0x3) + 2) + ((old >> 1) & 0x1)); */
/*   val = ((__typeof__(*(lock)))__cmpxchg((lock),(unsigned long)(old), (unsigned long)(new),sizeof(*(lock)))); */
/*  } while (__builtin_expect(!!(val != old), 0)); */
/*  return (new < 3) ? -1 : 0; */
/* } */

/* static inline __attribute__((always_inline)) int */
/* __acpi_release_global_lock (unsigned int *lock) */
/* { */
/*  unsigned int old, new, val; */
/*  do { */
/*   old = *lock; */
/*   new = old & ~0x3; */
/*   val = ((__typeof__(*(lock)))__cmpxchg((lock),(unsigned long)(old), (unsigned long)(new),sizeof(*(lock)))); */
/*  } while (__builtin_expect(!!(val != old), 0)); */
/*  return old & 0x1; */
/* } */

/* static inline __attribute__((always_inline)) void check_acpi_pci(void) { } */



/* extern int acpi_lapic; */
/* extern int acpi_ioapic; */
/* extern int acpi_noirq; */
/* extern int acpi_strict; */
/* extern int acpi_disabled; */
/* extern int acpi_ht; */
/* extern int acpi_pci_disabled; */
/* static inline __attribute__((always_inline)) void disable_acpi(void) */
/* { */
/*  acpi_disabled = 1; */
/*  acpi_ht = 0; */
/*  acpi_pci_disabled = 1; */
/*  acpi_noirq = 1; */
/* } */




/* extern int acpi_gsi_to_irq(u32 gsi, unsigned int *irq); */

/* static inline __attribute__((always_inline)) void disable_ioapic_setup(void) { } */


/* static inline __attribute__((always_inline)) void acpi_noirq_set(void) { acpi_noirq = 1; } */
/* static inline __attribute__((always_inline)) void acpi_disable_pci(void) */
/* { */
/*  acpi_pci_disabled = 1; */
/*  acpi_noirq_set(); */
/* } */
/* extern int acpi_irq_balance_set(char *str); */

/* extern int acpi_save_state_mem(void); */
/* extern void acpi_restore_state_mem(void); */

/* extern unsigned long acpi_wakeup_address; */


/* extern void acpi_reserve_bootmem(void); */



/* extern u8 x86_acpiid_to_apicid[]; */



/* struct local_apic { */

/*         struct { unsigned int __reserved[4]; } __reserved_01; */

/*         struct { unsigned int __reserved[4]; } __reserved_02; */

/*         struct { */
/*   unsigned int __reserved_1 : 24, */
/*    phys_apic_id : 4, */
/*    __reserved_2 : 4; */
/*   unsigned int __reserved[3]; */
/*  } id; */

/*         const */
/*  struct { */
/*   unsigned int version : 8, */
/*    __reserved_1 : 8, */
/*    max_lvt : 8, */
/*    __reserved_2 : 8; */
/*   unsigned int __reserved[3]; */
/*  } version; */

/*         struct { unsigned int __reserved[4]; } __reserved_03; */

/*         struct { unsigned int __reserved[4]; } __reserved_04; */

/*         struct { unsigned int __reserved[4]; } __reserved_05; */

/*         struct { unsigned int __reserved[4]; } __reserved_06; */

/*         struct { */
/*   unsigned int priority : 8, */
/*    __reserved_1 : 24; */
/*   unsigned int __reserved_2[3]; */
/*  } tpr; */

/*         const */
/*  struct { */
/*   unsigned int priority : 8, */
/*    __reserved_1 : 24; */
/*   unsigned int __reserved_2[3]; */
/*  } apr; */

/*         const */
/*  struct { */
/*   unsigned int priority : 8, */
/*    __reserved_1 : 24; */
/*   unsigned int __reserved_2[3]; */
/*  } ppr; */

/*         struct { */
/*   unsigned int eoi; */
/*   unsigned int __reserved[3]; */
/*  } eoi; */

/*         struct { unsigned int __reserved[4]; } __reserved_07; */

/*         struct { */
/*   unsigned int __reserved_1 : 24, */
/*    logical_dest : 8; */
/*   unsigned int __reserved_2[3]; */
/*  } ldr; */

/*         struct { */
/*   unsigned int __reserved_1 : 28, */
/*    model : 4; */
/*   unsigned int __reserved_2[3]; */
/*  } dfr; */

/*         struct { */
/*   unsigned int spurious_vector : 8, */
/*    apic_enabled : 1, */
/*    focus_cpu : 1, */
/*    __reserved_2 : 22; */
/*   unsigned int __reserved_3[3]; */
/*  } svr; */

/*         struct { */
/*          unsigned int bitfield; */
/*   unsigned int __reserved[3]; */
/*  } isr [8]; */

/*         struct { */
/*          unsigned int bitfield; */
/*   unsigned int __reserved[3]; */
/*  } tmr [8]; */

/*         struct { */
/*          unsigned int bitfield; */
/*   unsigned int __reserved[3]; */
/*  } irr [8]; */

/*         union { */
/*   struct { */
/*    unsigned int send_cs_error : 1, */
/*     receive_cs_error : 1, */
/*     send_accept_error : 1, */
/*     receive_accept_error : 1, */
/*     __reserved_1 : 1, */
/*     send_illegal_vector : 1, */
/*     receive_illegal_vector : 1, */
/*     illegal_register_address : 1, */
/*     __reserved_2 : 24; */
/*    unsigned int __reserved_3[3]; */
/*   } error_bits; */
/*   struct { */
/*    unsigned int errors; */
/*    unsigned int __reserved_3[3]; */
/*   } all_errors; */
/*  } esr; */

/*         struct { unsigned int __reserved[4]; } __reserved_08; */

/*         struct { unsigned int __reserved[4]; } __reserved_09; */

/*         struct { unsigned int __reserved[4]; } __reserved_10; */

/*         struct { unsigned int __reserved[4]; } __reserved_11; */

/*         struct { unsigned int __reserved[4]; } __reserved_12; */

/*         struct { unsigned int __reserved[4]; } __reserved_13; */

/*         struct { unsigned int __reserved[4]; } __reserved_14; */

/*         struct { */
/*   unsigned int vector : 8, */
/*    delivery_mode : 3, */
/*    destination_mode : 1, */
/*    delivery_status : 1, */
/*    __reserved_1 : 1, */
/*    level : 1, */
/*    trigger : 1, */
/*    __reserved_2 : 2, */
/*    shorthand : 2, */
/*    __reserved_3 : 12; */
/*   unsigned int __reserved_4[3]; */
/*  } icr1; */

/*         struct { */
/*   union { */
/*    unsigned int __reserved_1 : 24, */
/*     phys_dest : 4, */
/*     __reserved_2 : 4; */
/*    unsigned int __reserved_3 : 24, */
/*     logical_dest : 8; */
/*   } dest; */
/*   unsigned int __reserved_4[3]; */
/*  } icr2; */

/*         struct { */
/*   unsigned int vector : 8, */
/*    __reserved_1 : 4, */
/*    delivery_status : 1, */
/*    __reserved_2 : 3, */
/*    mask : 1, */
/*    timer_mode : 1, */
/*    __reserved_3 : 14; */
/*   unsigned int __reserved_4[3]; */
/*  } lvt_timer; */

/*         struct { */
/*   unsigned int vector : 8, */
/*    delivery_mode : 3, */
/*    __reserved_1 : 1, */
/*    delivery_status : 1, */
/*    __reserved_2 : 3, */
/*    mask : 1, */
/*    __reserved_3 : 15; */
/*   unsigned int __reserved_4[3]; */
/*  } lvt_thermal; */

/*         struct { */
/*   unsigned int vector : 8, */
/*    delivery_mode : 3, */
/*    __reserved_1 : 1, */
/*    delivery_status : 1, */
/*    __reserved_2 : 3, */
/*    mask : 1, */
/*    __reserved_3 : 15; */
/*   unsigned int __reserved_4[3]; */
/*  } lvt_pc; */

/*         struct { */
/*   unsigned int vector : 8, */
/*    delivery_mode : 3, */
/*    __reserved_1 : 1, */
/*    delivery_status : 1, */
/*    polarity : 1, */
/*    remote_irr : 1, */
/*    trigger : 1, */
/*    mask : 1, */
/*    __reserved_2 : 15; */
/*   unsigned int __reserved_3[3]; */
/*  } lvt_lint0; */

/*         struct { */
/*   unsigned int vector : 8, */
/*    delivery_mode : 3, */
/*    __reserved_1 : 1, */
/*    delivery_status : 1, */
/*    polarity : 1, */
/*    remote_irr : 1, */
/*    trigger : 1, */
/*    mask : 1, */
/*    __reserved_2 : 15; */
/*   unsigned int __reserved_3[3]; */
/*  } lvt_lint1; */

/*         struct { */
/*   unsigned int vector : 8, */
/*    __reserved_1 : 4, */
/*    delivery_status : 1, */
/*    __reserved_2 : 3, */
/*    mask : 1, */
/*    __reserved_3 : 15; */
/*   unsigned int __reserved_4[3]; */
/*  } lvt_error; */

/*         struct { */
/*   unsigned int initial_count; */
/*   unsigned int __reserved_2[3]; */
/*  } timer_icr; */

/*         const */
/*  struct { */
/*   unsigned int curr_count; */
/*   unsigned int __reserved_2[3]; */
/*  } timer_ccr; */

/*         struct { unsigned int __reserved[4]; } __reserved_16; */

/*         struct { unsigned int __reserved[4]; } __reserved_17; */

/*         struct { unsigned int __reserved[4]; } __reserved_18; */

/*         struct { unsigned int __reserved[4]; } __reserved_19; */

/*         struct { */
/*   unsigned int divisor : 4, */
/*    __reserved_1 : 28; */
/*   unsigned int __reserved_2[3]; */
/*  } timer_dcr; */

/*         struct { unsigned int __reserved[4]; } __reserved_20; */

/* } __attribute__ ((packed)); */






/* enum km_type { */
/* __KM_FENCE_0 , KM_BOUNCE_READ, */
/* __KM_FENCE_1 , KM_SKB_SUNRPC_DATA, */
/* __KM_FENCE_2 , KM_SKB_DATA_SOFTIRQ, */
/* __KM_FENCE_3 , KM_USER0, */
/* __KM_FENCE_4 , KM_USER1, */
/* __KM_FENCE_5 , KM_BIO_SRC_IRQ, */
/* __KM_FENCE_6 , KM_BIO_DST_IRQ, */
/* __KM_FENCE_7 , KM_PTE0, */
/* __KM_FENCE_8 , KM_PTE1, */
/* __KM_FENCE_9 , KM_IRQ0, */
/* __KM_FENCE_10 , KM_IRQ1, */
/* __KM_FENCE_11 , KM_SOFTIRQ0, */
/* __KM_FENCE_12 , KM_SOFTIRQ1, */
/* __KM_FENCE_13 , KM_TYPE_NR */
/* }; */


/* enum fixed_addresses { */
/*  FIX_HOLE, */
/*  FIX_VDSO, */

/*  FIX_KMAP_BEGIN, */
/*  FIX_KMAP_END = FIX_KMAP_BEGIN+(KM_TYPE_NR*1)-1, */


/*  FIX_ACPI_BEGIN, */
/*  FIX_ACPI_END = FIX_ACPI_BEGIN + 4 - 1, */


/*  FIX_PCIE_MCFG, */

/*  __end_of_permanent_fixed_addresses, */


/*  FIX_BTMAP_END = __end_of_permanent_fixed_addresses, */
/*  FIX_BTMAP_BEGIN = FIX_BTMAP_END + 16 - 1, */
/*  FIX_WP_TEST, */
/*  __end_of_fixed_addresses */
/* }; */

/* extern void __set_fixmap (enum fixed_addresses idx, */
/*      unsigned long phys, pgprot_t flags); */

/* extern void __this_fixmap_does_not_exist(void); */






/* static inline __attribute__((always_inline)) __attribute__((always_inline)) unsigned long fix_to_virt(const unsigned int idx) */
/* { */

/*  if (idx >= __end_of_fixed_addresses) */
/*   __this_fixmap_does_not_exist(); */

/*         return (((unsigned long)0xfffff000) - ((idx) << 12)); */
/* } */

/* static inline __attribute__((always_inline)) unsigned long virt_to_fix(const unsigned long vaddr) */
/* { */
/*  do { if (__builtin_expect(!!((vaddr >= ((unsigned long)0xfffff000) || vaddr < (((unsigned long)0xfffff000) - (__end_of_permanent_fixed_addresses << 12)))!=0), 0)) __asm__ __volatile__( "ud2\n" "\t.word %c0\n" "\t.long %c1\n" : : "i" (144), "i" ("include/asm/fixmap.h")); } while(0); */
/*  return ((((unsigned long)0xfffff000) - ((vaddr)&(~((1UL << 12)-1)))) >> 12); */
/* } */


/* struct mm_struct; */
/* struct vm_area_struct; */






/* extern unsigned long empty_zero_page[1024]; */
/* extern pgd_t swapper_pg_dir[1024]; */
/* extern kmem_cache_t *pgd_cache; */
/* extern kmem_cache_t *pmd_cache; */
/* extern spinlock_t pgd_lock; */
/* extern struct page *pgd_list; */

/* void pmd_ctor(void *, kmem_cache_t *, unsigned long); */
/* void pgd_ctor(void *, kmem_cache_t *, unsigned long); */
/* void pgd_dtor(void *, kmem_cache_t *, unsigned long); */
/* void pgtable_cache_init(void); */
/* void paging_init(void); */




/* extern unsigned long long __PAGE_KERNEL, __PAGE_KERNEL_EXEC; */

/* extern unsigned long pg0[]; */

/* static inline __attribute__((always_inline)) int pte_user(pte_t pte) { return (pte).pte_low & 0x004; } */
/* static inline __attribute__((always_inline)) int pte_read(pte_t pte) { return (pte).pte_low & 0x004; } */
/* static inline __attribute__((always_inline)) int pte_dirty(pte_t pte) { return (pte).pte_low & 0x040; } */
/* static inline __attribute__((always_inline)) int pte_young(pte_t pte) { return (pte).pte_low & 0x020; } */
/* static inline __attribute__((always_inline)) int pte_write(pte_t pte) { return (pte).pte_low & 0x002; } */
/* static inline __attribute__((always_inline)) int pte_huge(pte_t pte) { return (pte).pte_low & 0x080; } */




/* static inline __attribute__((always_inline)) int pte_file(pte_t pte) { return (pte).pte_low & 0x040; } */

/* static inline __attribute__((always_inline)) pte_t pte_rdprotect(pte_t pte) { (pte).pte_low &= ~0x004; return pte; } */
/* static inline __attribute__((always_inline)) pte_t pte_exprotect(pte_t pte) { (pte).pte_low &= ~0x004; return pte; } */
/* static inline __attribute__((always_inline)) pte_t pte_mkclean(pte_t pte) { (pte).pte_low &= ~0x040; return pte; } */
/* static inline __attribute__((always_inline)) pte_t pte_mkold(pte_t pte) { (pte).pte_low &= ~0x020; return pte; } */
/* static inline __attribute__((always_inline)) pte_t pte_wrprotect(pte_t pte) { (pte).pte_low &= ~0x002; return pte; } */
/* static inline __attribute__((always_inline)) pte_t pte_mkread(pte_t pte) { (pte).pte_low |= 0x004; return pte; } */
/* static inline __attribute__((always_inline)) pte_t pte_mkexec(pte_t pte) { (pte).pte_low |= 0x004; return pte; } */
/* static inline __attribute__((always_inline)) pte_t pte_mkdirty(pte_t pte) { (pte).pte_low |= 0x040; return pte; } */
/* static inline __attribute__((always_inline)) pte_t pte_mkyoung(pte_t pte) { (pte).pte_low |= 0x020; return pte; } */
/* static inline __attribute__((always_inline)) pte_t pte_mkwrite(pte_t pte) { (pte).pte_low |= 0x002; return pte; } */
/* static inline __attribute__((always_inline)) pte_t pte_mkhuge(pte_t pte) { (pte).pte_low |= 0x080; return pte; } */
















/* typedef struct { pgd_t pgd; } pud_t; */

/* static inline __attribute__((always_inline)) int pgd_none(pgd_t pgd) { return 0; } */
/* static inline __attribute__((always_inline)) int pgd_bad(pgd_t pgd) { return 0; } */
/* static inline __attribute__((always_inline)) int pgd_present(pgd_t pgd) { return 1; } */
/* static inline __attribute__((always_inline)) void pgd_clear(pgd_t *pgd) { } */

/* static inline __attribute__((always_inline)) pud_t * pud_offset(pgd_t * pgd, unsigned long address) */
/* { */
/*  return (pud_t *)pgd; */
/* } */


/* typedef struct { pud_t pud; } pmd_t; */

/* static inline __attribute__((always_inline)) int pud_none(pud_t pud) { return 0; } */
/* static inline __attribute__((always_inline)) int pud_bad(pud_t pud) { return 0; } */
/* static inline __attribute__((always_inline)) int pud_present(pud_t pud) { return 1; } */
/* static inline __attribute__((always_inline)) void pud_clear(pud_t *pud) { } */

/* static inline __attribute__((always_inline)) pmd_t * pmd_offset(pud_t * pud, unsigned long address) */
/* { */
/*  return (pmd_t *)pud; */
/* } */


/* static inline __attribute__((always_inline)) int pte_exec(pte_t pte) */
/* { */
/*  return pte_user(pte); */
/* } */




/* static inline __attribute__((always_inline)) int pte_exec_kernel(pte_t pte) */
/* { */
/*  return 1; */
/* } */

/* void vmalloc_sync_all(void); */



/* static inline __attribute__((always_inline)) int ptep_test_and_clear_dirty(struct vm_area_struct *vma, unsigned long addr, pte_t *ptep) */
/* { */
/*  if (!pte_dirty(*ptep)) */
/*   return 0; */
/*  return test_and_clear_bit(6, &ptep->pte_low); */
/* } */

/* static inline __attribute__((always_inline)) int ptep_test_and_clear_young(struct vm_area_struct *vma, unsigned long addr, pte_t *ptep) */
/* { */
/*  if (!pte_young(*ptep)) */
/*   return 0; */
/*  return test_and_clear_bit(5, &ptep->pte_low); */
/* } */

/* static inline __attribute__((always_inline)) pte_t ptep_get_and_clear_full(struct mm_struct *mm, unsigned long addr, pte_t *ptep, int full) */
/* { */
/*  pte_t pte; */
/*  if (full) { */
/*   pte = *ptep; */
/*   do { (*(ptep) = ((pte_t) { (0) } )); } while (0); */
/*  } else { */
/*   pte = ((pte_t) { (((__typeof__(*(&(ptep)->pte_low)))__xchg((unsigned long)(0),(&(ptep)->pte_low),sizeof(*(&(ptep)->pte_low))))) } ); */
/*  } */
/*  return pte; */
/* } */

/* static inline __attribute__((always_inline)) void ptep_set_wrprotect(struct mm_struct *mm, unsigned long addr, pte_t *ptep) */
/* { */
/*  clear_bit(1, &ptep->pte_low); */
/* } */

/* static inline __attribute__((always_inline)) void clone_pgd_range(pgd_t *dst, pgd_t *src, int count) */
/* { */
/*        (__builtin_constant_p(count * sizeof(pgd_t)) ? __constant_memcpy((dst),(src),(count * sizeof(pgd_t))) : __memcpy((dst),(src),(count * sizeof(pgd_t)))); */
/* } */

/* static inline __attribute__((always_inline)) pte_t pte_modify(pte_t pte, pgprot_t newprot) */
/* { */
/*  pte.pte_low &= ((~((1UL << 12)-1)) | 0x020 | 0x040); */
/*  pte.pte_low |= ((newprot).pgprot); */

/*  return pte; */
/* } */

/* extern pte_t *lookup_address(unsigned long address); */

/*  static inline __attribute__((always_inline)) int set_kernel_exec(unsigned long vaddr, int enable) { return 0;} */


/* extern void noexec_setup(const char *str); */



/* void pgd_clear_bad(pgd_t *); */
/* void pud_clear_bad(pud_t *); */
/* void pmd_clear_bad(pmd_t *); */

/* static inline __attribute__((always_inline)) int pgd_none_or_clear_bad(pgd_t *pgd) */
/* { */
/*  if (pgd_none(*pgd)) */
/*   return 1; */
/*  if (__builtin_expect(!!(pgd_bad(*pgd)), 0)) { */
/*   pgd_clear_bad(pgd); */
/*   return 1; */
/*  } */
/*  return 0; */
/* } */

/* static inline __attribute__((always_inline)) int pud_none_or_clear_bad(pud_t *pud) */
/* { */
/*  if (pud_none(*pud)) */
/*   return 1; */
/*  if (__builtin_expect(!!(pud_bad(*pud)), 0)) { */
/*   pud_clear_bad(pud); */
/*   return 1; */
/*  } */
/*  return 0; */
/* } */

/* static inline __attribute__((always_inline)) int pmd_none_or_clear_bad(pmd_t *pmd) */
/* { */
/*  if ((!(unsigned long)((((((*pmd).pud).pgd).pgd))))) */
/*   return 1; */
/*  if (__builtin_expect(!!(((((((((*pmd).pud).pgd).pgd))) & (~(~((1UL << 12)-1)) & ~0x004)) != (0x001 | 0x002 | 0x020 | 0x040))), 0)) { */
/*   pmd_clear_bad(pmd); */
/*   return 1; */
/*  } */
/*  return 0; */
/* } */



/* struct vm_area_struct { */
/*  struct mm_struct * vm_mm; */
/*  unsigned long vm_start; */
/*  unsigned long vm_end; */



/*  struct vm_area_struct *vm_next; */

/*  pgprot_t vm_page_prot; */
/*  unsigned long vm_flags; */

/*  struct rb_node vm_rb; */







/*  union { */
/*   struct { */
/*    struct list_head list; */
/*    void *parent; */
/*    struct vm_area_struct *head; */
/*   } vm_set; */

/*   struct raw_prio_tree_node prio_tree_node; */
/*  } shared; */







/*  struct list_head anon_vma_node; */
/*  struct anon_vma *anon_vma; */


/*  struct vm_operations_struct * vm_ops; */


/*  unsigned long vm_pgoff; */

/*  struct file * vm_file; */
/*  void * vm_private_data; */
/*  unsigned long vm_truncate_count; */







/* }; */






/* struct vm_list_struct { */
/*  struct vm_list_struct *next; */
/*  struct vm_area_struct *vma; */
/* }; */

/* extern pgprot_t protection_map[16]; */







/* struct vm_operations_struct { */
/*  void (*open)(struct vm_area_struct * area); */
/*  void (*close)(struct vm_area_struct * area); */
/*  struct page * (*nopage)(struct vm_area_struct * area, unsigned long address, int *type); */
/*  int (*populate)(struct vm_area_struct * area, unsigned long address, unsigned long len, pgprot_t prot, unsigned long pgoff, int nonblock); */



/*  int (*page_mkwrite)(struct vm_area_struct *vma, struct page *page); */







/* }; */

/* struct mmu_gather; */
/* struct inode; */







/* struct page { */
/*  unsigned long flags; */

/*  atomic_t _count; */
/*  atomic_t _mapcount; */



/*  union { */
/*      struct { */
/*   unsigned long private; */






/*   struct address_space *mapping; */






/*      }; */



/*  }; */
/*  unsigned long index; */
/*  struct list_head lru; */

/* }; */



/* struct page; */

/* int test_clear_page_dirty(struct page *page); */
/* int test_clear_page_writeback(struct page *page); */
/* int test_set_page_writeback(struct page *page); */

/* static inline __attribute__((always_inline)) void clear_page_dirty(struct page *page) */
/* { */
/*  test_clear_page_dirty(page); */
/* } */

/* static inline __attribute__((always_inline)) void set_page_writeback(struct page *page) */
/* { */
/*  test_set_page_writeback(page); */
/* } */


/* static inline __attribute__((always_inline)) int put_page_testzero(struct page *page) */
/* { */
/*  do { if (__builtin_expect(!!((((&page->_count)->counter) == 0)!=0), 0)) __asm__ __volatile__( "ud2\n" "\t.word %c0\n" "\t.long %c1\n" : : "i" (300), "i" ("include/linux/mm.h")); } while(0); */
/*  return atomic_dec_and_test(&page->_count); */
/* } */





/* static inline __attribute__((always_inline)) int get_page_unless_zero(struct page *page) */
/* { */
/*  return ({ int c, old; c = (((&page->_count))->counter); for (;;) { if (__builtin_expect(!!(c == (0)), 0)) break; old = ((int)((__typeof__(*(&((((&page->_count)))->counter))))__cmpxchg((&((((&page->_count)))->counter)),(unsigned long)(c), (unsigned long)(c + (1)),sizeof(*(&((((&page->_count)))->counter)))))); if (__builtin_expect(!!(old == c), 1)) break; c = old; } c != (0); }); */
/* } */

/* extern void __page_cache_release(struct page *) __attribute__((regparm(3))); */

/* static inline __attribute__((always_inline)) int page_count(struct page *page) */
/* { */
/*  if (__builtin_expect(!!((__builtin_constant_p(14) ? constant_test_bit((14),(&(page)->flags)) : variable_test_bit((14),(&(page)->flags)))), 0)) */
/*   page = (struct page *)((page)->private); */
/*  return ((&page->_count)->counter); */
/* } */

/* static inline __attribute__((always_inline)) void get_page(struct page *page) */
/* { */
/*  if (__builtin_expect(!!((__builtin_constant_p(14) ? constant_test_bit((14),(&(page)->flags)) : variable_test_bit((14),(&(page)->flags)))), 0)) */
/*   page = (struct page *)((page)->private); */
/*  atomic_inc(&page->_count); */
/* } */





/* static inline __attribute__((always_inline)) void init_page_count(struct page *page) */
/* { */
/*  (((&page->_count)->counter) = (1)); */
/* } */

/* void put_page(struct page *page); */
/* void put_pages_list(struct list_head *pages); */

/* void split_page(struct page *page, unsigned int order); */

/* static inline __attribute__((always_inline)) unsigned long page_zonenum(struct page *page) */
/* { */
/*  return (page->flags >> (((((sizeof(unsigned long)*8) - 0) - 0) - 2) * (2 != 0))) & ((1UL << 2) - 1); */
/* } */

/* struct zone; */
/* extern struct zone *zone_table[]; */

/* static inline __attribute__((always_inline)) int page_zone_id(struct page *page) */
/* { */
/*  return (page->flags >> (((((sizeof(unsigned long)*8) - 0) - 0) - 2) * (2 != 0))) & ((1UL << (0 + 2)) - 1); */
/* } */
/* static inline __attribute__((always_inline)) struct zone *page_zone(struct page *page) */
/* { */
/*  return zone_table[page_zone_id(page)]; */
/* } */

/* static inline __attribute__((always_inline)) unsigned long page_to_nid(struct page *page) */
/* { */
/*  if ((0 > 0 || 0 == 0)) */
/*   return (page->flags >> ((((sizeof(unsigned long)*8) - 0) - 0) * (0 != 0))) & ((1UL << 0) - 1); */
/*  else */
/*   return page_zone(page)->zone_pgdat->node_id; */
/* } */
/* static inline __attribute__((always_inline)) unsigned long page_to_section(struct page *page) */
/* { */
/*  return (page->flags >> (((sizeof(unsigned long)*8) - 0) * (0 != 0))) & ((1UL << 0) - 1); */
/* } */

/* static inline __attribute__((always_inline)) void set_page_zone(struct page *page, unsigned long zone) */
/* { */
/*  page->flags &= ~(((1UL << 2) - 1) << (((((sizeof(unsigned long)*8) - 0) - 0) - 2) * (2 != 0))); */
/*  page->flags |= (zone & ((1UL << 2) - 1)) << (((((sizeof(unsigned long)*8) - 0) - 0) - 2) * (2 != 0)); */
/* } */
/* static inline __attribute__((always_inline)) void set_page_node(struct page *page, unsigned long node) */
/* { */
/*  page->flags &= ~(((1UL << 0) - 1) << ((((sizeof(unsigned long)*8) - 0) - 0) * (0 != 0))); */
/*  page->flags |= (node & ((1UL << 0) - 1)) << ((((sizeof(unsigned long)*8) - 0) - 0) * (0 != 0)); */
/* } */
/* static inline __attribute__((always_inline)) void set_page_section(struct page *page, unsigned long section) */
/* { */
/*  page->flags &= ~(((1UL << 0) - 1) << (((sizeof(unsigned long)*8) - 0) * (0 != 0))); */
/*  page->flags |= (section & ((1UL << 0) - 1)) << (((sizeof(unsigned long)*8) - 0) * (0 != 0)); */
/* } */

/* static inline __attribute__((always_inline)) void set_page_links(struct page *page, unsigned long zone, */
/*  unsigned long node, unsigned long pfn) */
/* { */
/*  set_page_zone(page, zone); */
/*  set_page_node(page, node); */
/*  set_page_section(page, ((pfn) >> 0)); */
/* } */






/* enum vm_event_item { PGPGIN, PGPGOUT, PSWPIN, PSWPOUT, */
/*   PGALLOC_DMA, PGALLOC_DMA32, PGALLOC_NORMAL, PGALLOC_HIGH, */
/*   PGFREE, PGACTIVATE, PGDEACTIVATE, */
/*   PGFAULT, PGMAJFAULT, */
/*   PGREFILL_DMA, PGREFILL_DMA32, PGREFILL_NORMAL, PGREFILL_HIGH, */
/*   PGSTEAL_DMA, PGSTEAL_DMA32, PGSTEAL_NORMAL, PGSTEAL_HIGH, */
/*   PGSCAN_KSWAPD_DMA, PGSCAN_KSWAPD_DMA32, PGSCAN_KSWAPD_NORMAL, PGSCAN_KSWAPD_HIGH, */
/*   PGSCAN_DIRECT_DMA, PGSCAN_DIRECT_DMA32, PGSCAN_DIRECT_NORMAL, PGSCAN_DIRECT_HIGH, */
/*   PGINODESTEAL, SLABS_SCANNED, KSWAPD_STEAL, KSWAPD_INODESTEAL, */
/*   PAGEOUTRUN, ALLOCSTALL, PGROTATED, */
/*   NR_VM_EVENT_ITEMS */
/* }; */

/* struct vm_event_state { */
/*  unsigned long event[NR_VM_EVENT_ITEMS]; */
/* }; */

/* extern __typeof__(struct vm_event_state) per_cpu__vm_event_states; */

/* static inline __attribute__((always_inline)) void __count_vm_event(enum vm_event_item item) */
/* { */
/*  per_cpu__vm_event_states.event[item]++; */
/* } */

/* static inline __attribute__((always_inline)) void count_vm_event(enum vm_event_item item) */
/* { */
/*  (*({ do { } while (0); &per_cpu__vm_event_states; })).event[item]++; */
/*  do { } while (0); */
/* } */

/* static inline __attribute__((always_inline)) void __count_vm_events(enum vm_event_item item, long delta) */
/* { */
/*  per_cpu__vm_event_states.event[item] += delta; */
/* } */

/* static inline __attribute__((always_inline)) void count_vm_events(enum vm_event_item item, long delta) */
/* { */
/*  (*({ do { } while (0); &per_cpu__vm_event_states; })).event[item] += delta; */
/*  do { } while (0); */
/* } */

/* extern void all_vm_events(unsigned long *); */
/* extern void vm_events_fold_cpu(int cpu); */

/* extern atomic_long_t vm_stat[NR_VM_ZONE_STAT_ITEMS]; */

/* static inline __attribute__((always_inline)) void zone_page_state_add(long x, struct zone *zone, */
/*      enum zone_stat_item item) */
/* { */
/*  atomic_long_add(x, &zone->vm_stat[item]); */
/*  atomic_long_add(x, &vm_stat[item]); */
/* } */

/* static inline __attribute__((always_inline)) unsigned long global_page_state(enum zone_stat_item item) */
/* { */
/*  long x = atomic_long_read(&vm_stat[item]); */




/*  return x; */
/* } */

/* static inline __attribute__((always_inline)) unsigned long zone_page_state(struct zone *zone, */
/*      enum zone_stat_item item) */
/* { */
/*  long x = atomic_long_read(&zone->vm_stat[item]); */




/*  return x; */
/* } */

/* static inline __attribute__((always_inline)) void zap_zone_vm_stats(struct zone *zone) */
/* { */
/*  (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(zone->vm_stat))) ? __constant_c_and_count_memset(((zone->vm_stat)),((0x01010101UL*(unsigned char)(0))),((sizeof(zone->vm_stat)))) : __constant_c_memset(((zone->vm_stat)),((0x01010101UL*(unsigned char)(0))),((sizeof(zone->vm_stat))))) : (__builtin_constant_p((sizeof(zone->vm_stat))) ? __memset_generic((((zone->vm_stat))),(((0))),(((sizeof(zone->vm_stat))))) : __memset_generic(((zone->vm_stat)),((0)),((sizeof(zone->vm_stat)))))); */
/* } */

/* extern void inc_zone_state(struct zone *, enum zone_stat_item); */

/* static inline __attribute__((always_inline)) void __mod_zone_page_state(struct zone *zone, */
/*    enum zone_stat_item item, int delta) */
/* { */
/*  zone_page_state_add(delta, zone, item); */
/* } */

/* static inline __attribute__((always_inline)) void __inc_zone_state(struct zone *zone, enum zone_stat_item item) */
/* { */
/*  atomic_long_inc(&zone->vm_stat[item]); */
/*  atomic_long_inc(&vm_stat[item]); */
/* } */

/* static inline __attribute__((always_inline)) void __inc_zone_page_state(struct page *page, */
/*    enum zone_stat_item item) */
/* { */
/*  __inc_zone_state(page_zone(page), item); */
/* } */

/* static inline __attribute__((always_inline)) void __dec_zone_page_state(struct page *page, */
/*    enum zone_stat_item item) */
/* { */
/*  atomic_long_dec(&page_zone(page)->vm_stat[item]); */
/*  atomic_long_dec(&vm_stat[item]); */
/* } */

/* static inline __attribute__((always_inline)) void refresh_cpu_vm_stats(int cpu) { } */
/* static inline __attribute__((always_inline)) void refresh_vm_stats(void) { } */




/* extern struct page *mem_map; */


/* static inline __attribute__((always_inline)) __attribute__((always_inline)) void *lowmem_page_address(struct page *page) */
/* { */
/*  return ((void *)((unsigned long)(((unsigned long)((page) - mem_map) + (0UL)) << 12)+((unsigned long)((unsigned long)0xC0000000)))); */
/* } */

/* void *page_address(struct page *page); */
/* void set_page_address(struct page *page, void *virtual); */
/* void page_address_init(void); */

/* extern struct address_space swapper_space; */
/* static inline __attribute__((always_inline)) struct address_space *page_mapping(struct page *page) */
/* { */
/*  struct address_space *mapping = page->mapping; */

/*  if (__builtin_expect(!!((__builtin_constant_p(15) ? constant_test_bit((15),(&(page)->flags)) : variable_test_bit((15),(&(page)->flags)))), 0)) */
/*   mapping = &swapper_space; */
/*  else if (__builtin_expect(!!((unsigned long)mapping & 1), 0)) */
/*   mapping = ((void *)0); */
/*  return mapping; */
/* } */

/* static inline __attribute__((always_inline)) int PageAnon(struct page *page) */
/* { */
/*  return ((unsigned long)page->mapping & 1) != 0; */
/* } */





/* static inline __attribute__((always_inline)) unsigned long page_index(struct page *page) */
/* { */
/*  if (__builtin_expect(!!((__builtin_constant_p(15) ? constant_test_bit((15),(&(page)->flags)) : variable_test_bit((15),(&(page)->flags)))), 0)) */
/*   return ((page)->private); */
/*  return page->index; */
/* } */






/* static inline __attribute__((always_inline)) void reset_page_mapcount(struct page *page) */
/* { */
/*  (((&(page)->_mapcount)->counter) = (-1)); */
/* } */

/* static inline __attribute__((always_inline)) int page_mapcount(struct page *page) */
/* { */
/*  return ((&(page)->_mapcount)->counter) + 1; */
/* } */




/* static inline __attribute__((always_inline)) int page_mapped(struct page *page) */
/* { */
/*  return ((&(page)->_mapcount)->counter) >= 0; */
/* } */

/* extern void show_free_areas(void); */


/* struct page *shmem_nopage(struct vm_area_struct *vma, */
/*    unsigned long address, int *type); */
/* int shmem_set_policy(struct vm_area_struct *vma, struct mempolicy *new); */
/* struct mempolicy *shmem_get_policy(struct vm_area_struct *vma, */
/*      unsigned long addr); */
/* int shmem_lock(struct file *file, int lock, struct user_struct *user); */

/* struct file *shmem_file_setup(char *name, loff_t size, unsigned long flags); */
/* extern int shmem_mmap(struct file *file, struct vm_area_struct *vma); */

/* int shmem_zero_setup(struct vm_area_struct *); */

/* static inline __attribute__((always_inline)) int can_do_mlock(void) */
/* { */
/*  if (capable(14)) */
/*   return 1; */
/*  if (__vericon_dummy_current->signal->rlim[8].rlim_cur != 0) */
/*   return 1; */
/*  return 0; */
/* } */
/* extern int user_shm_lock(size_t, struct user_struct *); */
/* extern void user_shm_unlock(size_t, struct user_struct *); */




/* struct zap_details { */
/*  struct vm_area_struct *nonlinear_vma; */
/*  struct address_space *check_mapping; */
/*  unsigned long first_index; */
/*  unsigned long last_index; */
/*  spinlock_t *i_mmap_lock; */
/*  unsigned long truncate_count; */
/* }; */

/* struct page *vm_normal_page(struct vm_area_struct *, unsigned long, pte_t); */
/* unsigned long zap_page_range(struct vm_area_struct *vma, unsigned long address, */
/*   unsigned long size, struct zap_details *); */
/* unsigned long unmap_vmas(struct mmu_gather **tlb, */
/*   struct vm_area_struct *start_vma, unsigned long start_addr, */
/*   unsigned long end_addr, unsigned long *nr_accounted, */
/*   struct zap_details *); */
/* void free_pgd_range(struct mmu_gather **tlb, unsigned long addr, */
/*   unsigned long end, unsigned long floor, unsigned long ceiling); */
/* void free_pgtables(struct mmu_gather **tlb, struct vm_area_struct *start_vma, */
/*   unsigned long floor, unsigned long ceiling); */
/* int copy_page_range(struct mm_struct *dst, struct mm_struct *src, */
/*    struct vm_area_struct *vma); */
/* int zeromap_page_range(struct vm_area_struct *vma, unsigned long from, */
/*    unsigned long size, pgprot_t prot); */
/* void unmap_mapping_range(struct address_space *mapping, */
/*   loff_t const holebegin, loff_t const holelen, int even_cows); */

/* static inline __attribute__((always_inline)) void unmap_shared_mapping_range(struct address_space *mapping, */
/*   loff_t const holebegin, loff_t const holelen) */
/* { */
/*  unmap_mapping_range(mapping, holebegin, holelen, 0); */
/* } */

/* extern int vmtruncate(struct inode * inode, loff_t offset); */
/* extern int vmtruncate_range(struct inode * inode, loff_t offset, loff_t end); */
/* extern int install_page(struct mm_struct *mm, struct vm_area_struct *vma, unsigned long addr, struct page *page, pgprot_t prot); */
/* extern int install_file_pte(struct mm_struct *mm, struct vm_area_struct *vma, unsigned long addr, unsigned long pgoff, pgprot_t prot); */


/* extern int __handle_mm_fault(struct mm_struct *mm,struct vm_area_struct *vma, */
/*    unsigned long address, int write_access); */

/* static inline __attribute__((always_inline)) int handle_mm_fault(struct mm_struct *mm, */
/*    struct vm_area_struct *vma, unsigned long address, */
/*    int write_access) */
/* { */
/*  return __handle_mm_fault(mm, vma, address, write_access) & */
/*     (~0x10); */
/* } */

/* extern int make_pages_present(unsigned long addr, unsigned long end); */
/* extern int access_process_vm(struct task_struct *tsk, unsigned long addr, void *buf, int len, int write); */
/* void install_arg_page(struct vm_area_struct *, struct page *, unsigned long); */

/* int get_user_pages(struct task_struct *tsk, struct mm_struct *mm, unsigned long start, */
/*   int len, int write, int force, struct page **pages, struct vm_area_struct **vmas); */
/* void print_bad_pte(struct vm_area_struct *, pte_t, unsigned long); */

/* int __set_page_dirty_buffers(struct page *page); */
/* int __set_page_dirty_nobuffers(struct page *page); */
/* int redirty_page_for_writepage(struct writeback_control *wbc, */
/*     struct page *page); */
/* int set_page_dirty(struct page *page) __attribute__((regparm(3))); */
/* int set_page_dirty_lock(struct page *page); */
/* int clear_page_dirty_for_io(struct page *page); */

/* extern unsigned long do_mremap(unsigned long addr, */
/*           unsigned long old_len, unsigned long new_len, */
/*           unsigned long flags, unsigned long new_addr); */

/* typedef int (*shrinker_t)(int nr_to_scan, gfp_t gfp_mask); */







/* struct shrinker; */
/* extern struct shrinker *set_shrinker(int, shrinker_t); */
/* extern void remove_shrinker(struct shrinker *shrinker); */

/* extern pte_t *get_locked_pte(struct mm_struct *mm, unsigned long addr, spinlock_t **ptl) __attribute__((regparm(3))); */

/* int __pud_alloc(struct mm_struct *mm, pgd_t *pgd, unsigned long address); */
/* int __pmd_alloc(struct mm_struct *mm, pud_t *pud, unsigned long address); */
/* int __pte_alloc(struct mm_struct *mm, pmd_t *pmd, unsigned long address); */
/* int __pte_alloc_kernel(pmd_t *pmd, unsigned long address); */






/* static inline __attribute__((always_inline)) pud_t *pud_alloc(struct mm_struct *mm, pgd_t *pgd, unsigned long address) */
/* { */
/*  return (__builtin_expect(!!(pgd_none(*pgd)), 0) && __pud_alloc(mm, pgd, address))? */
/*   ((void *)0): pud_offset(pgd, address); */
/* } */

/* static inline __attribute__((always_inline)) pmd_t *pmd_alloc(struct mm_struct *mm, pud_t *pud, unsigned long address) */
/* { */
/*  return (__builtin_expect(!!(pud_none(*pud)), 0) && __pmd_alloc(mm, pud, address))? */
/*   ((void *)0): pmd_offset(pud, address); */
/* } */

/* extern void free_area_init(unsigned long * zones_size); */
/* extern void free_area_init_node(int nid, pg_data_t *pgdat, */
/*  unsigned long * zones_size, unsigned long zone_start_pfn, */
/*  unsigned long *zholes_size); */
/* extern void memmap_init_zone(unsigned long, int, unsigned long, unsigned long); */
/* extern void setup_per_zone_pages_min(void); */
/* extern void mem_init(void); */
/* extern void show_mem(void); */
/* extern void si_meminfo(struct sysinfo * val); */
/* extern void si_meminfo_node(struct sysinfo *val, int nid); */




/* static inline __attribute__((always_inline)) void setup_per_cpu_pageset(void) {} */



/* void vma_prio_tree_add(struct vm_area_struct *, struct vm_area_struct *old); */
/* void vma_prio_tree_insert(struct vm_area_struct *, struct prio_tree_root *); */
/* void vma_prio_tree_remove(struct vm_area_struct *, struct prio_tree_root *); */
/* struct vm_area_struct *vma_prio_tree_next(struct vm_area_struct *vma, */
/*  struct prio_tree_iter *iter); */





/* static inline __attribute__((always_inline)) void vma_nonlinear_insert(struct vm_area_struct *vma, */
/*      struct list_head *list) */
/* { */
/*  vma->shared.vm_set.parent = ((void *)0); */
/*  list_add_tail(&vma->shared.vm_set.list, list); */
/* } */


/* extern int __vm_enough_memory(long pages, int cap_sys_admin); */
/* extern void vma_adjust(struct vm_area_struct *vma, unsigned long start, */
/*  unsigned long end, unsigned long pgoff, struct vm_area_struct *insert); */
/* extern struct vm_area_struct *vma_merge(struct mm_struct *, */
/*  struct vm_area_struct *prev, unsigned long addr, unsigned long end, */
/*  unsigned long vm_flags, struct anon_vma *, struct file *, unsigned long, */
/*  struct mempolicy *); */
/* extern struct anon_vma *find_mergeable_anon_vma(struct vm_area_struct *); */
/* extern int split_vma(struct mm_struct *, */
/*  struct vm_area_struct *, unsigned long addr, int new_below); */
/* extern int insert_vm_struct(struct mm_struct *, struct vm_area_struct *); */
/* extern void __vma_link_rb(struct mm_struct *, struct vm_area_struct *, */
/*  struct rb_node **, struct rb_node *); */
/* extern void unlink_file_vma(struct vm_area_struct *); */
/* extern struct vm_area_struct *copy_vma(struct vm_area_struct **, */
/*  unsigned long addr, unsigned long len, unsigned long pgoff); */
/* extern void exit_mmap(struct mm_struct *); */
/* extern int may_expand_vm(struct mm_struct *mm, unsigned long npages); */

/* extern unsigned long get_unmapped_area(struct file *, unsigned long, unsigned long, unsigned long, unsigned long); */

/* extern unsigned long do_mmap_pgoff(struct file *file, unsigned long addr, */
/*  unsigned long len, unsigned long prot, */
/*  unsigned long flag, unsigned long pgoff); */

/* static inline __attribute__((always_inline)) unsigned long do_mmap(struct file *file, unsigned long addr, */
/*  unsigned long len, unsigned long prot, */
/*  unsigned long flag, unsigned long offset) */
/* { */
/*  unsigned long ret = -22; */
/*  if ((offset + (((len)+(1UL << 12)-1)&(~((1UL << 12)-1)))) < offset) */
/*   goto out; */
/*  if (!(offset & ~(~((1UL << 12)-1)))) */
/*   ret = do_mmap_pgoff(file, addr, len, prot, flag, offset >> 12); */
/* out: */
/*  return ret; */
/* } */

/* extern int do_munmap(struct mm_struct *, unsigned long, size_t); */

/* extern unsigned long do_brk(unsigned long, unsigned long); */


/* extern unsigned long page_unuse(struct page *); */
/* extern void truncate_inode_pages(struct address_space *, loff_t); */
/* extern void truncate_inode_pages_range(struct address_space *, */
/*            loff_t lstart, loff_t lend); */


/* extern struct page *filemap_nopage(struct vm_area_struct *, unsigned long, int *); */
/* extern int filemap_populate(struct vm_area_struct *, unsigned long, */
/*   unsigned long, pgprot_t, unsigned long, int); */


/* int write_one_page(struct page *page, int wait); */







/* int do_page_cache_readahead(struct address_space *mapping, struct file *filp, */
/*    unsigned long offset, unsigned long nr_to_read); */
/* int force_page_cache_readahead(struct address_space *mapping, struct file *filp, */
/*    unsigned long offset, unsigned long nr_to_read); */
/* unsigned long page_cache_readahead(struct address_space *mapping, */
/*      struct file_ra_state *ra, */
/*      struct file *filp, */
/*      unsigned long offset, */
/*      unsigned long size); */
/* void handle_ra_miss(struct address_space *mapping, */
/*       struct file_ra_state *ra, unsigned long offset); */
/* unsigned long max_sane_readahead(unsigned long nr); */


/* extern int expand_stack(struct vm_area_struct *vma, unsigned long address); */





/* extern struct vm_area_struct * find_vma(struct mm_struct * mm, unsigned long addr); */
/* extern struct vm_area_struct * find_vma_prev(struct mm_struct * mm, unsigned long addr, */
/*           struct vm_area_struct **pprev); */



/* static inline __attribute__((always_inline)) struct vm_area_struct * find_vma_intersection(struct mm_struct * mm, unsigned long start_addr, unsigned long end_addr) */
/* { */
/*  struct vm_area_struct * vma = find_vma(mm,start_addr); */

/*  if (vma && end_addr <= vma->vm_start) */
/*   vma = ((void *)0); */
/*  return vma; */
/* } */

/* static inline __attribute__((always_inline)) unsigned long vma_pages(struct vm_area_struct *vma) */
/* { */
/*  return (vma->vm_end - vma->vm_start) >> 12; */
/* } */

/* struct vm_area_struct *find_extend_vma(struct mm_struct *, unsigned long addr); */
/* struct page *vmalloc_to_page(void *addr); */
/* unsigned long vmalloc_to_pfn(void *addr); */
/* int remap_pfn_range(struct vm_area_struct *, unsigned long addr, */
/*    unsigned long pfn, unsigned long size, pgprot_t); */
/* int vm_insert_page(struct vm_area_struct *, unsigned long addr, struct page *); */

/* struct page *follow_page(struct vm_area_struct *, unsigned long address, */
/*    unsigned int foll_flags); */






/* void vm_stat_account(struct mm_struct *, unsigned long, struct file *, long); */

/* static inline __attribute__((always_inline)) void */
/* kernel_map_pages(struct page *page, int numpages, int enable) */
/* { */
/*  if (!is_highmem(page_zone(page)) && !enable) */
/*   debug_check_no_locks_freed(page_address(page), */
/*         numpages * (1UL << 12)); */
/* } */


/* extern struct vm_area_struct *get_gate_vma(struct task_struct *tsk); */

/* int in_gate_area_no_task(unsigned long addr); */
/* int in_gate_area(struct task_struct *task, unsigned long addr); */

/* int drop_caches_sysctl_handler(struct ctl_table *, int, struct file *, */
/*      void *, size_t *, loff_t *); */
/* unsigned long shrink_slab(unsigned long scanned, gfp_t gfp_mask, */
/*    unsigned long lru_pages); */
/* void drop_pagecache(void); */
/* void drop_slab(void); */




/* extern int randomize_va_space; */


/* const char *arch_vma_name(struct vm_area_struct *vma); */












/* void global_flush_tlb(void); */
/* int change_page_attr(struct page *page, int numpages, pgprot_t prot); */







/* void mark_rodata_ro(void); */



/* static inline __attribute__((always_inline)) void flush_anon_page(struct page *page, unsigned long vmaddr) */
/* { */
/* } */



/* static inline __attribute__((always_inline)) void flush_kernel_dcache_page(struct page *page) */
/* { */
/* } */










/* typedef int irqreturn_t; */

























/* static __inline__ __attribute__((always_inline)) int irq_canonicalize(int irq) */
/* { */
/*  return ((irq == 2) ? 9 : irq); */
/* } */






/*   extern void irq_ctx_init(int cpu); */
/*   extern void irq_ctx_exit(int cpu); */


/* struct proc_dir_entry; */

/* struct irq_chip { */
/*  const char *name; */
/*  unsigned int (*startup)(unsigned int irq); */
/*  void (*shutdown)(unsigned int irq); */
/*  void (*enable)(unsigned int irq); */
/*  void (*disable)(unsigned int irq); */

/*  void (*ack)(unsigned int irq); */
/*  void (*mask)(unsigned int irq); */
/*  void (*mask_ack)(unsigned int irq); */
/*  void (*unmask)(unsigned int irq); */
/*  void (*eoi)(unsigned int irq); */

/*  void (*end)(unsigned int irq); */
/*  void (*set_affinity)(unsigned int irq, cpumask_t dest); */
/*  int (*retrigger)(unsigned int irq); */
/*  int (*set_type)(unsigned int irq, unsigned int flow_type); */
/*  int (*set_wake)(unsigned int irq, unsigned int on); */

/*  const char *typename; */
/* }; */

/* struct irq_desc { */
/*  void __attribute__((regparm(3))) (*handle_irq)(unsigned int irq, */
/*            struct irq_desc *desc, */
/*            struct pt_regs *regs); */
/*  struct irq_chip *chip; */
/*  void *handler_data; */
/*  void *chip_data; */
/*  struct irqaction *action; */
/*  unsigned int status; */

/*  unsigned int depth; */
/*  unsigned int wake_depth; */
/*  unsigned int irq_count; */
/*  unsigned int irqs_unhandled; */
/*  spinlock_t lock; */

/*  struct proc_dir_entry *dir; */

/* } __attribute__((__aligned__((1 << (7))))); */

/* extern struct irq_desc irq_desc[16]; */





/* typedef struct irq_chip hw_irq_controller; */

/* typedef struct irq_desc irq_desc_t; */








/* struct proc_dir_entry; */
/* struct pt_regs; */
/* struct notifier_block; */


/* void __attribute__ ((__section__ (".init.text"))) profile_init(void); */
/* void profile_tick(int, struct pt_regs *); */
/* void profile_hit(int, void *); */

/* void create_prof_cpu_mask(struct proc_dir_entry *); */




/* enum profile_type { */
/*  PROFILE_TASK_EXIT, */
/*  PROFILE_MUNMAP */
/* }; */



/* struct task_struct; */
/* struct mm_struct; */


/* void profile_task_exit(struct task_struct * task); */




/* int profile_handoff_task(struct task_struct * task); */


/* void profile_munmap(unsigned long addr); */

/* int task_handoff_register(struct notifier_block * n); */
/* int task_handoff_unregister(struct notifier_block * n); */

/* int profile_event_register(enum profile_type, struct notifier_block * n); */
/* int profile_event_unregister(enum profile_type, struct notifier_block * n); */

/* int register_timer_hook(int (*hook)(struct pt_regs *)); */
/* void unregister_timer_hook(int (*hook)(struct pt_regs *)); */


/* extern int (*timer_hook)(struct pt_regs *); */

/* struct pt_regs; */














/* extern char _text[], _stext[], _etext[]; */
/* extern char _data[], _sdata[], _edata[]; */
/* extern char __bss_start[], __bss_stop[]; */
/* extern char __init_begin[], __init_end[]; */
/* extern char _sinittext[], _einittext[]; */
/* extern char _sextratext[] __attribute__((weak)); */
/* extern char _eextratext[] __attribute__((weak)); */
/* extern char _end[]; */
/* extern char __per_cpu_start[], __per_cpu_end[]; */
/* extern char __kprobes_text_start[], __kprobes_text_end[]; */
/* extern char __initdata_begin[], __initdata_end[]; */
/* extern char __start_rodata[], __end_rodata[]; */



/* struct irq_chip; */

/* extern u8 irq_vector[16]; */



/* extern void (*interrupt[16])(void); */

/* void disable_8259A_irq(unsigned int irq); */
/* void enable_8259A_irq(unsigned int irq); */
/* int i8259A_irq_pending(unsigned int irq); */
/* void make_8259A_irq(unsigned int irq); */
/* void init_8259A(int aeoi); */
/* void send_IPI_self(int vector) __attribute__((regparm(3))); */
/* void init_VISWS_APIC_irqs(void); */
/* void setup_IO_APIC(void); */
/* void disable_IO_APIC(void); */
/* void print_IO_APIC(void); */
/* int IO_APIC_get_PCI_irq_vector(int bus, int slot, int fn); */
/* void send_IPI(int dest, int vector); */
/* void setup_ioapic_dest(void); */

/* extern unsigned long io_apic_irqs; */

/* extern atomic_t irq_err_count; */
/* extern atomic_t irq_mis_count; */


/* extern int setup_irq(unsigned int irq, struct irqaction *new); */

/* static inline __attribute__((always_inline)) void set_native_irq_info(int irq, cpumask_t mask) */
/* { */
/* } */

/* static inline __attribute__((always_inline)) void set_balance_irq_affinity(unsigned int irq, cpumask_t mask) */
/* { */
/* } */





/* static inline __attribute__((always_inline)) int select_smp_affinity(unsigned int irq) */
/* { */
/*  return 1; */
/* } */


/* extern int no_irq_affinity; */


/* extern int handle_IRQ_event(unsigned int irq, struct pt_regs *regs, */
/*        struct irqaction *action); */





/* extern void __attribute__((regparm(3))) */
/* handle_level_irq(unsigned int irq, struct irq_desc *desc, struct pt_regs *regs); */
/* extern void __attribute__((regparm(3))) */
/* handle_fasteoi_irq(unsigned int irq, struct irq_desc *desc, */
/*     struct pt_regs *regs); */
/* extern void __attribute__((regparm(3))) */
/* handle_edge_irq(unsigned int irq, struct irq_desc *desc, struct pt_regs *regs); */
/* extern void __attribute__((regparm(3))) */
/* handle_simple_irq(unsigned int irq, struct irq_desc *desc, */
/*     struct pt_regs *regs); */
/* extern void __attribute__((regparm(3))) */
/* handle_percpu_irq(unsigned int irq, struct irq_desc *desc, */
/*     struct pt_regs *regs); */
/* extern void __attribute__((regparm(3))) */
/* handle_bad_irq(unsigned int irq, struct irq_desc *desc, struct pt_regs *regs); */





/* extern const char * */
/* handle_irq_name(void __attribute__((regparm(3))) (*handle)(unsigned int, struct irq_desc *, */
/*      struct pt_regs *)); */





/* extern __attribute__((regparm(3))) unsigned int __do_IRQ(unsigned int irq, struct pt_regs *regs); */







/* static inline __attribute__((always_inline)) void generic_handle_irq(unsigned int irq, struct pt_regs *regs) */
/* { */
/*  struct irq_desc *desc = irq_desc + irq; */

/*  if (__builtin_expect(!!(desc->handle_irq), 1)) */
/*   desc->handle_irq(irq, desc, regs); */
/*  else */
/*   __do_IRQ(irq, regs); */
/* } */


/* extern void note_interrupt(unsigned int irq, struct irq_desc *desc, */
/*       int action_ret, struct pt_regs *regs); */


/* void check_irq_resend(struct irq_desc *desc, unsigned int irq); */


/* extern void init_irq_proc(void); */


/* extern int noirqdebug_setup(char *str); */


/* extern int can_request_irq(unsigned int irq, unsigned long irqflags); */


/* extern struct irq_chip no_irq_chip; */
/* extern struct irq_chip dummy_irq_chip; */

/* extern void */
/* set_irq_chip_and_handler(unsigned int irq, struct irq_chip *chip, */
/*     void __attribute__((regparm(3))) (*handle)(unsigned int, */
/*        struct irq_desc *, */
/*        struct pt_regs *)); */
/* extern void */
/* __set_irq_handler(unsigned int irq, */
/*     void __attribute__((regparm(3))) (*handle)(unsigned int, struct irq_desc *, */
/*        struct pt_regs *), */
/*     int is_chained); */




/* static inline __attribute__((always_inline)) void */
/* set_irq_handler(unsigned int irq, */
/*   void __attribute__((regparm(3))) (*handle)(unsigned int, struct irq_desc *, */
/*      struct pt_regs *)) */
/* { */
/*  __set_irq_handler(irq, handle, 0); */
/* } */






/* static inline __attribute__((always_inline)) void */
/* set_irq_chained_handler(unsigned int irq, */
/*    void __attribute__((regparm(3))) (*handle)(unsigned int, struct irq_desc *, */
/*       struct pt_regs *)) */
/* { */
/*  __set_irq_handler(irq, handle, 1); */
/* } */



/* extern int set_irq_chip(unsigned int irq, struct irq_chip *chip); */
/* extern int set_irq_data(unsigned int irq, void *data); */
/* extern int set_irq_chip_data(unsigned int irq, void *data); */
/* extern int set_irq_type(unsigned int irq, unsigned int type); */


/* typedef struct { */
/*  unsigned int __softirq_pending; */
/*  unsigned long idle_timestamp; */
/*  unsigned int __nmi_count; */
/*  unsigned int apic_timer_irqs; */
/* } __attribute__((__aligned__((1 << (7))))) irq_cpustat_t; */

/* extern __typeof__(irq_cpustat_t) per_cpu__irq_stat; */
/* extern irq_cpustat_t irq_stat[]; */




/* void ack_bad_irq(unsigned int irq); */




/* struct task_struct; */


/* static inline __attribute__((always_inline)) void account_system_vtime(struct task_struct *tsk) */
/* { */
/* } */

/* extern void irq_exit(void); */


/* struct irqaction { */
/*  irqreturn_t (*handler)(int, void *, struct pt_regs *); */
/*  unsigned long flags; */
/*  cpumask_t mask; */
/*  const char *name; */
/*  void *dev_id; */
/*  struct irqaction *next; */
/*  int irq; */
/*  struct proc_dir_entry *dir; */
/* }; */

/* extern irqreturn_t no_action(int cpl, void *dev_id, struct pt_regs *regs); */
/* extern int request_irq(unsigned int, */
/*          irqreturn_t (*handler)(int, void *, struct pt_regs *), */
/*          unsigned long, const char *, void *); */
/* extern void free_irq(unsigned int, void *); */

/* extern void disable_irq_nosync(unsigned int irq); */
/* extern void disable_irq(unsigned int irq); */
/* extern void enable_irq(unsigned int irq); */

/* static inline __attribute__((always_inline)) void disable_irq_nosync_lockdep(unsigned int irq) */
/* { */
/*  disable_irq_nosync(irq); */



/* } */

/* static inline __attribute__((always_inline)) void disable_irq_lockdep(unsigned int irq) */
/* { */
/*  disable_irq(irq); */



/* } */

/* static inline __attribute__((always_inline)) void enable_irq_lockdep(unsigned int irq) */
/* { */



/*  enable_irq(irq); */
/* } */


/* extern int set_irq_wake(unsigned int irq, unsigned int on); */

/* static inline __attribute__((always_inline)) int enable_irq_wake(unsigned int irq) */
/* { */
/*  return set_irq_wake(irq, 1); */
/* } */

/* static inline __attribute__((always_inline)) int disable_irq_wake(unsigned int irq) */
/* { */
/*  return set_irq_wake(irq, 0); */
/* } */

/* static inline __attribute__((always_inline)) void __attribute__((deprecated)) cli(void) */
/* { */
/*  do { raw_local_irq_disable(); do { } while (0); } while (0); */
/* } */
/* static inline __attribute__((always_inline)) void __attribute__((deprecated)) sti(void) */
/* { */
/*  do { do { } while (0); raw_local_irq_enable(); } while (0); */
/* } */
/* static inline __attribute__((always_inline)) void __attribute__((deprecated)) save_flags(unsigned long *x) */
/* { */
/*  do { (*x) = __raw_local_save_flags(); } while (0); */
/* } */

/* static inline __attribute__((always_inline)) void __attribute__((deprecated)) restore_flags(unsigned long x) */
/* { */
/*  do { if (raw_irqs_disabled_flags(x)) { raw_local_irq_restore(x); do { } while (0); } else { do { } while (0); raw_local_irq_restore(x); } } while (0); */
/* } */

/* static inline __attribute__((always_inline)) void __attribute__((deprecated)) save_and_cli(unsigned long *x) */
/* { */
/*  do { do { (*x) = __raw_local_irq_save(); } while (0); do { } while (0); } while (0); */
/* } */



/* extern void local_bh_disable(void); */
/* extern void __local_bh_enable(void); */
/* extern void _local_bh_enable(void); */
/* extern void local_bh_enable(void); */
/* extern void local_bh_enable_ip(unsigned long ip); */







/* enum */
/* { */
/*  HI_SOFTIRQ=0, */
/*  TIMER_SOFTIRQ, */
/*  NET_TX_SOFTIRQ, */
/*  NET_RX_SOFTIRQ, */
/*  BLOCK_SOFTIRQ, */
/*  TASKLET_SOFTIRQ */
/* }; */





/* struct softirq_action */
/* { */
/*  void (*action)(struct softirq_action *); */
/*  void *data; */
/* }; */

/*  __attribute__((regparm(0))) void do_softirq(void); */
/* extern void open_softirq(int nr, void (*action)(struct softirq_action*), void *data); */
/* extern void softirq_init(void); */

/* extern void raise_softirq_irqoff(unsigned int nr) __attribute__((regparm(3))); */
/* extern void raise_softirq(unsigned int nr) __attribute__((regparm(3))); */

/* struct tasklet_struct */
/* { */
/*  struct tasklet_struct *next; */
/*  unsigned long state; */
/*  atomic_t count; */
/*  void (*func)(unsigned long); */
/*  unsigned long data; */
/* }; */

/* enum */
/* { */
/*  TASKLET_STATE_SCHED, */
/*  TASKLET_STATE_RUN */
/* }; */

/* extern void __tasklet_schedule(struct tasklet_struct *t) __attribute__((regparm(3))); */

/* static inline __attribute__((always_inline)) void tasklet_schedule(struct tasklet_struct *t) */
/* { */
/*  if (!test_and_set_bit(TASKLET_STATE_SCHED, &t->state)) */
/*   __tasklet_schedule(t); */
/* } */

/* extern void __tasklet_hi_schedule(struct tasklet_struct *t) __attribute__((regparm(3))); */

/* static inline __attribute__((always_inline)) void tasklet_hi_schedule(struct tasklet_struct *t) */
/* { */
/*  if (!test_and_set_bit(TASKLET_STATE_SCHED, &t->state)) */
/*   __tasklet_hi_schedule(t); */
/* } */


/* static inline __attribute__((always_inline)) void tasklet_disable_nosync(struct tasklet_struct *t) */
/* { */
/*  atomic_inc(&t->count); */
/*  __asm__ __volatile__("": : :"memory"); */
/* } */

/* static inline __attribute__((always_inline)) void tasklet_disable(struct tasklet_struct *t) */
/* { */
/*  tasklet_disable_nosync(t); */
/*  do { } while (0); */
/*  __asm__ __volatile__("": : :"memory"); */
/* } */

/* static inline __attribute__((always_inline)) void tasklet_enable(struct tasklet_struct *t) */
/* { */
/*  __asm__ __volatile__("": : :"memory"); */
/*  atomic_dec(&t->count); */
/* } */

/* static inline __attribute__((always_inline)) void tasklet_hi_enable(struct tasklet_struct *t) */
/* { */
/*  __asm__ __volatile__("": : :"memory"); */
/*  atomic_dec(&t->count); */
/* } */

/* extern void tasklet_kill(struct tasklet_struct *t); */
/* extern void tasklet_kill_immediate(struct tasklet_struct *t, unsigned int cpu); */
/* extern void tasklet_init(struct tasklet_struct *t, */
/*     void (*func)(unsigned long), unsigned long data); */

/* extern unsigned long probe_irq_on(void); */
/* extern int probe_irq_off(unsigned long); */
/* extern unsigned int probe_irq_mask(unsigned long); */





/* extern unsigned long pgkern_mask; */

/* static inline __attribute__((always_inline)) void flush_tlb_mm(struct mm_struct *mm) */
/* { */
/*  if (mm == __vericon_dummy_current->active_mm) */
/*   do { unsigned int tmpreg; __asm__ __volatile__( "movl %%cr3, %0;              \n" "movl %0, %%cr3;  # flush TLB \n" : "=r" (tmpreg) :: "memory"); } while (0); */
/* } */

/* static inline __attribute__((always_inline)) void flush_tlb_page(struct vm_area_struct *vma, */
/*  unsigned long addr) */
/* { */
/*  if (vma->vm_mm == __vericon_dummy_current->active_mm) */
/*   __asm__ __volatile__("invlpg %0": :"m" (*(char *) addr)); */
/* } */

/* static inline __attribute__((always_inline)) void flush_tlb_range(struct vm_area_struct *vma, */
/*  unsigned long start, unsigned long end) */
/* { */
/*  if (vma->vm_mm == __vericon_dummy_current->active_mm) */
/*   do { unsigned int tmpreg; __asm__ __volatile__( "movl %%cr3, %0;              \n" "movl %0, %%cr3;  # flush TLB \n" : "=r" (tmpreg) :: "memory"); } while (0); */
/* } */

/* static inline __attribute__((always_inline)) void flush_tlb_pgtables(struct mm_struct *mm, */
/*           unsigned long start, unsigned long end) */
/* { */

/* } */



/* extern unsigned long highstart_pfn, highend_pfn; */

/* extern pte_t *kmap_pte; */
/* extern pgprot_t kmap_prot; */
/* extern pte_t *pkmap_page_table; */

/* extern void * kmap_high(struct page *page) __attribute__((regparm(3))); */
/* extern void kunmap_high(struct page *page) __attribute__((regparm(3))); */

/* void *kmap(struct page *page); */
/* void kunmap(struct page *page); */
/* void *kmap_atomic(struct page *page, enum km_type type); */
/* void kunmap_atomic(void *kvaddr, enum km_type type); */
/* void *kmap_atomic_pfn(unsigned long pfn, enum km_type type); */
/* struct page *kmap_atomic_to_page(void *ptr); */



/* unsigned int nr_free_highpages(void); */

/* static inline __attribute__((always_inline)) void clear_user_highpage(struct page *page, unsigned long vaddr) */
/* { */
/*  void *addr = kmap_atomic(page, KM_USER0); */
/*  (__builtin_constant_p(0) ? (__builtin_constant_p(((1UL << 12))) ? __constant_c_and_count_memset((((void *)(addr))),((0x01010101UL*(unsigned char)(0))),(((1UL << 12)))) : __constant_c_memset((((void *)(addr))),((0x01010101UL*(unsigned char)(0))),(((1UL << 12))))) : (__builtin_constant_p(((1UL << 12))) ? __memset_generic(((((void *)(addr)))),(((0))),((((1UL << 12))))) : __memset_generic((((void *)(addr))),((0)),(((1UL << 12)))))); */
/*  kunmap_atomic(addr, KM_USER0); */

/*  __asm__ __volatile__("": : :"memory"); */
/* } */

/* static inline __attribute__((always_inline)) void clear_highpage(struct page *page) */
/* { */
/*  void *kaddr = kmap_atomic(page, KM_USER0); */
/*  (__builtin_constant_p(0) ? (__builtin_constant_p(((1UL << 12))) ? __constant_c_and_count_memset((((void *)(kaddr))),((0x01010101UL*(unsigned char)(0))),(((1UL << 12)))) : __constant_c_memset((((void *)(kaddr))),((0x01010101UL*(unsigned char)(0))),(((1UL << 12))))) : (__builtin_constant_p(((1UL << 12))) ? __memset_generic(((((void *)(kaddr)))),(((0))),((((1UL << 12))))) : __memset_generic((((void *)(kaddr))),((0)),(((1UL << 12)))))); */
/*  kunmap_atomic(kaddr, KM_USER0); */
/* } */




/* static inline __attribute__((always_inline)) void memclear_highpage_flush(struct page *page, unsigned int offset, unsigned int size) */
/* { */
/*  void *kaddr; */

/*  do { if (__builtin_expect(!!((offset + size > (1UL << 12))!=0), 0)) __asm__ __volatile__( "ud2\n" "\t.word %c0\n" "\t.long %c1\n" : : "i" (84), "i" ("include/linux/highmem.h")); } while(0); */

/*  kaddr = kmap_atomic(page, KM_USER0); */
/*  (__builtin_constant_p(0) ? (__builtin_constant_p((size)) ? __constant_c_and_count_memset((((char *)kaddr + offset)),((0x01010101UL*(unsigned char)(0))),((size))) : __constant_c_memset((((char *)kaddr + offset)),((0x01010101UL*(unsigned char)(0))),((size)))) : (__builtin_constant_p((size)) ? __memset_generic(((((char *)kaddr + offset))),(((0))),(((size)))) : __memset_generic((((char *)kaddr + offset)),((0)),((size))))); */
/*  do { } while (0); */
/*  kunmap_atomic(kaddr, KM_USER0); */
/* } */

/* static inline __attribute__((always_inline)) void copy_user_highpage(struct page *to, struct page *from, unsigned long vaddr) */
/* { */
/*  char *vfrom, *vto; */

/*  vfrom = kmap_atomic(from, KM_USER0); */
/*  vto = kmap_atomic(to, KM_USER1); */
/*  (__builtin_constant_p((1UL << 12)) ? __constant_memcpy(((void *)(vto)),((void *)(vfrom)),((1UL << 12))) : __memcpy(((void *)(vto)),((void *)(vfrom)),((1UL << 12)))); */
/*  kunmap_atomic(vfrom, KM_USER0); */
/*  kunmap_atomic(vto, KM_USER1); */

/*  __asm__ __volatile__("": : :"memory"); */
/* } */

/* static inline __attribute__((always_inline)) void copy_highpage(struct page *to, struct page *from) */
/* { */
/*  char *vfrom, *vto; */

/*  vfrom = kmap_atomic(from, KM_USER0); */
/*  vto = kmap_atomic(to, KM_USER1); */
/*  (__builtin_constant_p((1UL << 12)) ? __constant_memcpy(((void *)(vto)),((void *)(vfrom)),((1UL << 12))) : __memcpy(((void *)(vto)),((void *)(vfrom)),((1UL << 12)))); */
/*  kunmap_atomic(vfrom, KM_USER0); */
/*  kunmap_atomic(vto, KM_USER1); */
/* } */




/* extern struct movsl_mask { */
/*  int mask; */
/* } movsl_mask; */

/* struct exception_table_entry */
/* { */
/*  unsigned long insn, fixup; */
/* }; */

/* extern int fixup_exception(struct pt_regs *regs); */

/* extern void __get_user_1(void); */
/* extern void __get_user_2(void); */
/* extern void __get_user_4(void); */

/* extern void __put_user_bad(void); */





/* extern void __put_user_1(void); */
/* extern void __put_user_2(void); */
/* extern void __put_user_4(void); */
/* extern void __put_user_8(void); */

/* struct __large_struct { unsigned long buf[100]; }; */

/* extern long __get_user_bad(void); */

/* unsigned long __attribute__((warn_unused_result)) __copy_to_user_ll(void *to, */
/*     const void *from, unsigned long n); */
/* unsigned long __attribute__((warn_unused_result)) __copy_from_user_ll(void *to, */
/*     const void *from, unsigned long n); */
/* unsigned long __attribute__((warn_unused_result)) __copy_from_user_ll_nozero(void *to, */
/*     const void *from, unsigned long n); */
/* unsigned long __attribute__((warn_unused_result)) __copy_from_user_ll_nocache(void *to, */
/*     const void *from, unsigned long n); */
/* unsigned long __attribute__((warn_unused_result)) __copy_from_user_ll_nocache_nozero(void *to, */
/*     const void *from, unsigned long n); */

/* static inline __attribute__((always_inline)) __attribute__((always_inline)) unsigned long __attribute__((warn_unused_result)) */
/* __copy_to_user_inatomic(void *to, const void *from, unsigned long n) */
/* { */
/*  if (__builtin_constant_p(n)) { */
/*   unsigned long ret; */

/*   switch (n) { */
/*   case 1: */
/*    do { ret = 0; (void)0; switch (1) { case 1: __asm__ __volatile__( "1:	mov""b"" %""b""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret) : "iq" (*(u8 *)from), "m"((*(struct __large_struct *)((u8 *)to))), "i"(1), "0"(ret));break; case 2: __asm__ __volatile__( "1:	mov""w"" %""w""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret) : "ir" (*(u8 *)from), "m"((*(struct __large_struct *)((u8 *)to))), "i"(1), "0"(ret));break; case 4: __asm__ __volatile__( "1:	mov""l"" %""""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret) : "ir" (*(u8 *)from), "m"((*(struct __large_struct *)((u8 *)to))), "i"(1), "0"(ret)); break; case 8: __asm__ __volatile__( "1:	movl %%eax,0(%2)\n" "2:	movl %%edx,4(%2)\n" "3:\n" ".section .fixup,\"ax\"\n" "4:	movl %3,%0\n" "	jmp 3b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,4b\n" "	.long 2b,4b\n" ".previous" : "=r"(ret) : "A" ((__typeof__(*(u8 *)to))(*(u8 *)from)), "r" ((u8 *)to), "i"(-14), "0"(ret)); break; default: __put_user_bad(); } } while (0); */
/*    return ret; */
/*   case 2: */
/*    do { ret = 0; (void)0; switch (2) { case 1: __asm__ __volatile__( "1:	mov""b"" %""b""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret) : "iq" (*(u16 *)from), "m"((*(struct __large_struct *)((u16 *)to))), "i"(2), "0"(ret));break; case 2: __asm__ __volatile__( "1:	mov""w"" %""w""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret) : "ir" (*(u16 *)from), "m"((*(struct __large_struct *)((u16 *)to))), "i"(2), "0"(ret));break; case 4: __asm__ __volatile__( "1:	mov""l"" %""""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret) : "ir" (*(u16 *)from), "m"((*(struct __large_struct *)((u16 *)to))), "i"(2), "0"(ret)); break; case 8: __asm__ __volatile__( "1:	movl %%eax,0(%2)\n" "2:	movl %%edx,4(%2)\n" "3:\n" ".section .fixup,\"ax\"\n" "4:	movl %3,%0\n" "	jmp 3b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,4b\n" "	.long 2b,4b\n" ".previous" : "=r"(ret) : "A" ((__typeof__(*(u16 *)to))(*(u16 *)from)), "r" ((u16 *)to), "i"(-14), "0"(ret)); break; default: __put_user_bad(); } } while (0); */
/*    return ret; */
/*   case 4: */
/*    do { ret = 0; (void)0; switch (4) { case 1: __asm__ __volatile__( "1:	mov""b"" %""b""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret) : "iq" (*(u32 *)from), "m"((*(struct __large_struct *)((u32 *)to))), "i"(4), "0"(ret));break; case 2: __asm__ __volatile__( "1:	mov""w"" %""w""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret) : "ir" (*(u32 *)from), "m"((*(struct __large_struct *)((u32 *)to))), "i"(4), "0"(ret));break; case 4: __asm__ __volatile__( "1:	mov""l"" %""""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret) : "ir" (*(u32 *)from), "m"((*(struct __large_struct *)((u32 *)to))), "i"(4), "0"(ret)); break; case 8: __asm__ __volatile__( "1:	movl %%eax,0(%2)\n" "2:	movl %%edx,4(%2)\n" "3:\n" ".section .fixup,\"ax\"\n" "4:	movl %3,%0\n" "	jmp 3b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,4b\n" "	.long 2b,4b\n" ".previous" : "=r"(ret) : "A" ((__typeof__(*(u32 *)to))(*(u32 *)from)), "r" ((u32 *)to), "i"(-14), "0"(ret)); break; default: __put_user_bad(); } } while (0); */
/*    return ret; */
/*   } */
/*  } */
/*  return __copy_to_user_ll(to, from, n); */
/* } */

/* static inline __attribute__((always_inline)) __attribute__((always_inline)) unsigned long __attribute__((warn_unused_result)) */
/* __copy_to_user(void *to, const void *from, unsigned long n) */
/* { */
/*        do { __might_sleep("include/asm/uaccess.h", 445); cond_resched(); } while (0); */
/*        return __copy_to_user_inatomic(to, from, n); */
/* } */

/* static inline __attribute__((always_inline)) __attribute__((always_inline)) unsigned long */
/* __copy_from_user_inatomic(void *to, const void *from, unsigned long n) */
/* { */





/*  if (__builtin_constant_p(n)) { */
/*   unsigned long ret; */

/*   switch (n) { */
/*   case 1: */
/*    do { ret = 0; (void)0; switch (1) { case 1: __asm__ __volatile__( "1:	mov""b"" %2,%""b""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=q" (*(u8 *)to) : "m"((*(struct __large_struct *)(from))), "i"(1), "0"(ret));break; case 2: __asm__ __volatile__( "1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u8 *)to) : "m"((*(struct __large_struct *)(from))), "i"(1), "0"(ret));break; case 4: __asm__ __volatile__( "1:	mov""l"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""l"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u8 *)to) : "m"((*(struct __large_struct *)(from))), "i"(1), "0"(ret));break; default: (*(u8 *)to) = __get_user_bad(); } } while (0); */
/*    return ret; */
/*   case 2: */
/*    do { ret = 0; (void)0; switch (2) { case 1: __asm__ __volatile__( "1:	mov""b"" %2,%""b""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=q" (*(u16 *)to) : "m"((*(struct __large_struct *)(from))), "i"(2), "0"(ret));break; case 2: __asm__ __volatile__( "1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u16 *)to) : "m"((*(struct __large_struct *)(from))), "i"(2), "0"(ret));break; case 4: __asm__ __volatile__( "1:	mov""l"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""l"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u16 *)to) : "m"((*(struct __large_struct *)(from))), "i"(2), "0"(ret));break; default: (*(u16 *)to) = __get_user_bad(); } } while (0); */
/*    return ret; */
/*   case 4: */
/*    do { ret = 0; (void)0; switch (4) { case 1: __asm__ __volatile__( "1:	mov""b"" %2,%""b""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=q" (*(u32 *)to) : "m"((*(struct __large_struct *)(from))), "i"(4), "0"(ret));break; case 2: __asm__ __volatile__( "1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u32 *)to) : "m"((*(struct __large_struct *)(from))), "i"(4), "0"(ret));break; case 4: __asm__ __volatile__( "1:	mov""l"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""l"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u32 *)to) : "m"((*(struct __large_struct *)(from))), "i"(4), "0"(ret));break; default: (*(u32 *)to) = __get_user_bad(); } } while (0); */
/*    return ret; */
/*   } */
/*  } */
/*  return __copy_from_user_ll_nozero(to, from, n); */
/* } */
/* static inline __attribute__((always_inline)) __attribute__((always_inline)) unsigned long */
/* __copy_from_user(void *to, const void *from, unsigned long n) */
/* { */
/*  do { __might_sleep("include/asm/uaccess.h", 499); cond_resched(); } while (0); */
/*  if (__builtin_constant_p(n)) { */
/*   unsigned long ret; */

/*   switch (n) { */
/*   case 1: */
/*    do { ret = 0; (void)0; switch (1) { case 1: __asm__ __volatile__( "1:	mov""b"" %2,%""b""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=q" (*(u8 *)to) : "m"((*(struct __large_struct *)(from))), "i"(1), "0"(ret));break; case 2: __asm__ __volatile__( "1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u8 *)to) : "m"((*(struct __large_struct *)(from))), "i"(1), "0"(ret));break; case 4: __asm__ __volatile__( "1:	mov""l"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""l"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u8 *)to) : "m"((*(struct __large_struct *)(from))), "i"(1), "0"(ret));break; default: (*(u8 *)to) = __get_user_bad(); } } while (0); */
/*    return ret; */
/*   case 2: */
/*    do { ret = 0; (void)0; switch (2) { case 1: __asm__ __volatile__( "1:	mov""b"" %2,%""b""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=q" (*(u16 *)to) : "m"((*(struct __large_struct *)(from))), "i"(2), "0"(ret));break; case 2: __asm__ __volatile__( "1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u16 *)to) : "m"((*(struct __large_struct *)(from))), "i"(2), "0"(ret));break; case 4: __asm__ __volatile__( "1:	mov""l"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""l"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u16 *)to) : "m"((*(struct __large_struct *)(from))), "i"(2), "0"(ret));break; default: (*(u16 *)to) = __get_user_bad(); } } while (0); */
/*    return ret; */
/*   case 4: */
/*    do { ret = 0; (void)0; switch (4) { case 1: __asm__ __volatile__( "1:	mov""b"" %2,%""b""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=q" (*(u32 *)to) : "m"((*(struct __large_struct *)(from))), "i"(4), "0"(ret));break; case 2: __asm__ __volatile__( "1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u32 *)to) : "m"((*(struct __large_struct *)(from))), "i"(4), "0"(ret));break; case 4: __asm__ __volatile__( "1:	mov""l"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""l"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u32 *)to) : "m"((*(struct __large_struct *)(from))), "i"(4), "0"(ret));break; default: (*(u32 *)to) = __get_user_bad(); } } while (0); */
/*    return ret; */
/*   } */
/*  } */
/*  return __copy_from_user_ll(to, from, n); */
/* } */



/* static inline __attribute__((always_inline)) __attribute__((always_inline)) unsigned long __copy_from_user_nocache(void *to, */
/*     const void *from, unsigned long n) */
/* { */
/*  do { __might_sleep("include/asm/uaccess.h", 523); cond_resched(); } while (0); */
/*  if (__builtin_constant_p(n)) { */
/*   unsigned long ret; */

/*   switch (n) { */
/*   case 1: */
/*    do { ret = 0; (void)0; switch (1) { case 1: __asm__ __volatile__( "1:	mov""b"" %2,%""b""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=q" (*(u8 *)to) : "m"((*(struct __large_struct *)(from))), "i"(1), "0"(ret));break; case 2: __asm__ __volatile__( "1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u8 *)to) : "m"((*(struct __large_struct *)(from))), "i"(1), "0"(ret));break; case 4: __asm__ __volatile__( "1:	mov""l"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""l"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u8 *)to) : "m"((*(struct __large_struct *)(from))), "i"(1), "0"(ret));break; default: (*(u8 *)to) = __get_user_bad(); } } while (0); */
/*    return ret; */
/*   case 2: */
/*    do { ret = 0; (void)0; switch (2) { case 1: __asm__ __volatile__( "1:	mov""b"" %2,%""b""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=q" (*(u16 *)to) : "m"((*(struct __large_struct *)(from))), "i"(2), "0"(ret));break; case 2: __asm__ __volatile__( "1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u16 *)to) : "m"((*(struct __large_struct *)(from))), "i"(2), "0"(ret));break; case 4: __asm__ __volatile__( "1:	mov""l"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""l"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u16 *)to) : "m"((*(struct __large_struct *)(from))), "i"(2), "0"(ret));break; default: (*(u16 *)to) = __get_user_bad(); } } while (0); */
/*    return ret; */
/*   case 4: */
/*    do { ret = 0; (void)0; switch (4) { case 1: __asm__ __volatile__( "1:	mov""b"" %2,%""b""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=q" (*(u32 *)to) : "m"((*(struct __large_struct *)(from))), "i"(4), "0"(ret));break; case 2: __asm__ __volatile__( "1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u32 *)to) : "m"((*(struct __large_struct *)(from))), "i"(4), "0"(ret));break; case 4: __asm__ __volatile__( "1:	mov""l"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""l"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u32 *)to) : "m"((*(struct __large_struct *)(from))), "i"(4), "0"(ret));break; default: (*(u32 *)to) = __get_user_bad(); } } while (0); */
/*    return ret; */
/*   } */
/*  } */
/*  return __copy_from_user_ll_nocache(to, from, n); */
/* } */

/* static inline __attribute__((always_inline)) __attribute__((always_inline)) unsigned long */
/* __copy_from_user_inatomic_nocache(void *to, const void *from, unsigned long n) */
/* { */
/*        return __copy_from_user_ll_nocache_nozero(to, from, n); */
/* } */

/* unsigned long __attribute__((warn_unused_result)) copy_to_user(void *to, */
/*     const void *from, unsigned long n); */
/* unsigned long __attribute__((warn_unused_result)) copy_from_user(void *to, */
/*     const void *from, unsigned long n); */
/* long __attribute__((warn_unused_result)) strncpy_from_user(char *dst, const char *src, */
/*     long count); */
/* long __attribute__((warn_unused_result)) __strncpy_from_user(char *dst, */
/*     const char *src, long count); */

/* long strnlen_user(const char *str, long n); */
/* unsigned long __attribute__((warn_unused_result)) clear_user(void *mem, unsigned long len); */
/* unsigned long __attribute__((warn_unused_result)) __clear_user(void *mem, unsigned long len); */


/* static inline __attribute__((always_inline)) gfp_t mapping_gfp_mask(struct address_space * mapping) */
/* { */
/*  return ( gfp_t)mapping->flags & (( gfp_t)((1 << 20) - 1)); */
/* } */





/* static inline __attribute__((always_inline)) void mapping_set_gfp_mask(struct address_space *m, gfp_t mask) */
/* { */
/*  m->flags = (m->flags & ~( unsigned long)(( gfp_t)((1 << 20) - 1))) | */
/*     ( unsigned long)mask; */
/* } */

/* void release_pages(struct page **pages, int nr, int cold); */





/* static inline __attribute__((always_inline)) struct page *page_cache_alloc(struct address_space *x) */
/* { */
/*  return alloc_pages_node(((0)), mapping_gfp_mask(x), 0); */
/* } */

/* static inline __attribute__((always_inline)) struct page *page_cache_alloc_cold(struct address_space *x) */
/* { */
/*  return alloc_pages_node(((0)), mapping_gfp_mask(x)|(( gfp_t)0x100u), 0); */
/* } */


/* typedef int filler_t(void *, struct page *); */

/* extern struct page * find_get_page(struct address_space *mapping, */
/*     unsigned long index); */
/* extern struct page * find_lock_page(struct address_space *mapping, */
/*     unsigned long index); */
/* extern __attribute__((deprecated)) struct page * find_trylock_page( */
/*    struct address_space *mapping, unsigned long index); */
/* extern struct page * find_or_create_page(struct address_space *mapping, */
/*     unsigned long index, gfp_t gfp_mask); */
/* unsigned find_get_pages(struct address_space *mapping, unsigned long start, */
/*    unsigned int nr_pages, struct page **pages); */
/* unsigned find_get_pages_contig(struct address_space *mapping, unsigned long start, */
/*           unsigned int nr_pages, struct page **pages); */
/* unsigned find_get_pages_tag(struct address_space *mapping, unsigned long *index, */
/*    int tag, unsigned int nr_pages, struct page **pages); */




/* static inline __attribute__((always_inline)) struct page *grab_cache_page(struct address_space *mapping, unsigned long index) */
/* { */
/*  return find_or_create_page(mapping, index, mapping_gfp_mask(mapping)); */
/* } */

/* extern struct page * grab_cache_page_nowait(struct address_space *mapping, */
/*     unsigned long index); */
/* extern struct page * read_cache_page(struct address_space *mapping, */
/*     unsigned long index, filler_t *filler, */
/*     void *data); */
/* extern int read_cache_pages(struct address_space *mapping, */
/*   struct list_head *pages, filler_t *filler, void *data); */

/* static inline __attribute__((always_inline)) struct page *read_mapping_page(struct address_space *mapping, */
/*           unsigned long index, void *data) */
/* { */
/*  filler_t *filler = (filler_t *)mapping->a_ops->readpage; */
/*  return read_cache_page(mapping, index, filler, data); */
/* } */

/* int add_to_page_cache(struct page *page, struct address_space *mapping, */
/*     unsigned long index, gfp_t gfp_mask); */
/* int add_to_page_cache_lru(struct page *page, struct address_space *mapping, */
/*     unsigned long index, gfp_t gfp_mask); */
/* extern void remove_from_page_cache(struct page *page); */
/* extern void __remove_from_page_cache(struct page *page); */




/* static inline __attribute__((always_inline)) loff_t page_offset(struct page *page) */
/* { */
/*  return ((loff_t)page->index) << 12; */
/* } */

/* static inline __attribute__((always_inline)) unsigned long linear_page_index(struct vm_area_struct *vma, */
/*      unsigned long address) */
/* { */
/*  unsigned long pgoff = (address - vma->vm_start) >> 12; */
/*  pgoff += vma->vm_pgoff; */
/*  return pgoff >> (12 - 12); */
/* } */

/* extern void __lock_page(struct page *page) __attribute__((regparm(3))); */
/* extern void unlock_page(struct page *page) __attribute__((regparm(3))); */

/* static inline __attribute__((always_inline)) void lock_page(struct page *page) */
/* { */
/*  do { __might_sleep("include/linux/pagemap.h", 137); cond_resched(); } while (0); */
/*  if (test_and_set_bit(0, &(page)->flags)) */
/*   __lock_page(page); */
/* } */





/* extern void wait_on_page_bit(struct page *page, int bit_nr) __attribute__((regparm(3))); */

/* static inline __attribute__((always_inline)) void wait_on_page_locked(struct page *page) */
/* { */
/*  if ((__builtin_constant_p(0) ? constant_test_bit((0),(&(page)->flags)) : variable_test_bit((0),(&(page)->flags)))) */
/*   wait_on_page_bit(page, 0); */
/* } */




/* static inline __attribute__((always_inline)) void wait_on_page_writeback(struct page *page) */
/* { */
/*  if ((__builtin_constant_p(12) ? constant_test_bit((12),(&(page)->flags)) : variable_test_bit((12),(&(page)->flags)))) */
/*   wait_on_page_bit(page, 12); */
/* } */

/* extern void end_page_writeback(struct page *page); */







/* static inline __attribute__((always_inline)) int fault_in_pages_writeable(char *uaddr, int size) */
/* { */
/*  int ret; */





/*  ret = ({ long __pu_err; do { __pu_err = 0; (void)0; switch ((sizeof(*(uaddr)))) { case 1: __asm__ __volatile__( "1:	mov""b"" %""b""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(__pu_err) : "iq" (((__typeof__(*(uaddr)))(0))), "m"((*(struct __large_struct *)(((uaddr))))), "i"(-14), "0"(__pu_err));break; case 2: __asm__ __volatile__( "1:	mov""w"" %""w""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(__pu_err) : "ir" (((__typeof__(*(uaddr)))(0))), "m"((*(struct __large_struct *)(((uaddr))))), "i"(-14), "0"(__pu_err));break; case 4: __asm__ __volatile__( "1:	mov""l"" %""""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(__pu_err) : "ir" (((__typeof__(*(uaddr)))(0))), "m"((*(struct __large_struct *)(((uaddr))))), "i"(-14), "0"(__pu_err)); break; case 8: __asm__ __volatile__( "1:	movl %%eax,0(%2)\n" "2:	movl %%edx,4(%2)\n" "3:\n" ".section .fixup,\"ax\"\n" "4:	movl %3,%0\n" "	jmp 3b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,4b\n" "	.long 2b,4b\n" ".previous" : "=r"(__pu_err) : "A" ((__typeof__(*((uaddr))))(((__typeof__(*(uaddr)))(0)))), "r" (((uaddr))), "i"(-14), "0"(__pu_err)); break; default: __put_user_bad(); } } while (0); __pu_err; }); */
/*  if (ret == 0) { */
/*   char *end = uaddr + size - 1; */





/*   if (((unsigned long)uaddr & (~((1UL << 12)-1))) != */
/*     ((unsigned long)end & (~((1UL << 12)-1)))) */
/*     ret = ({ long __pu_err; do { __pu_err = 0; (void)0; switch ((sizeof(*(end)))) { case 1: __asm__ __volatile__( "1:	mov""b"" %""b""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(__pu_err) : "iq" (((__typeof__(*(end)))(0))), "m"((*(struct __large_struct *)(((end))))), "i"(-14), "0"(__pu_err));break; case 2: __asm__ __volatile__( "1:	mov""w"" %""w""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(__pu_err) : "ir" (((__typeof__(*(end)))(0))), "m"((*(struct __large_struct *)(((end))))), "i"(-14), "0"(__pu_err));break; case 4: __asm__ __volatile__( "1:	mov""l"" %""""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(__pu_err) : "ir" (((__typeof__(*(end)))(0))), "m"((*(struct __large_struct *)(((end))))), "i"(-14), "0"(__pu_err)); break; case 8: __asm__ __volatile__( "1:	movl %%eax,0(%2)\n" "2:	movl %%edx,4(%2)\n" "3:\n" ".section .fixup,\"ax\"\n" "4:	movl %3,%0\n" "	jmp 3b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,4b\n" "	.long 2b,4b\n" ".previous" : "=r"(__pu_err) : "A" ((__typeof__(*((end))))(((__typeof__(*(end)))(0)))), "r" (((end))), "i"(-14), "0"(__pu_err)); break; default: __put_user_bad(); } } while (0); __pu_err; }); */
/*  } */
/*  return ret; */
/* } */

/* static inline __attribute__((always_inline)) void fault_in_pages_readable(const char *uaddr, int size) */
/* { */
/*  volatile char c; */
/*  int ret; */

/*  ret = ({ long __gu_err; unsigned long __gu_val; do { __gu_err = 0; (void)0; switch ((sizeof(*(uaddr)))) { case 1: __asm__ __volatile__( "1:	mov""b"" %2,%""b""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(__gu_err), "=q" (__gu_val) : "m"((*(struct __large_struct *)(((uaddr))))), "i"(-14), "0"(__gu_err));break; case 2: __asm__ __volatile__( "1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(__gu_err), "=r" (__gu_val) : "m"((*(struct __large_struct *)(((uaddr))))), "i"(-14), "0"(__gu_err));break; case 4: __asm__ __volatile__( "1:	mov""l"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""l"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(__gu_err), "=r" (__gu_val) : "m"((*(struct __large_struct *)(((uaddr))))), "i"(-14), "0"(__gu_err));break; default: (__gu_val) = __get_user_bad(); } } while (0); ((c)) = (__typeof__(*((uaddr))))__gu_val; __gu_err; }); */
/*  if (ret == 0) { */
/*   const char *end = uaddr + size - 1; */

/*   if (((unsigned long)uaddr & (~((1UL << 12)-1))) != */
/*     ((unsigned long)end & (~((1UL << 12)-1)))) */
/*     ({ long __gu_err; unsigned long __gu_val; do { __gu_err = 0; (void)0; switch ((sizeof(*(end)))) { case 1: __asm__ __volatile__( "1:	mov""b"" %2,%""b""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(__gu_err), "=q" (__gu_val) : "m"((*(struct __large_struct *)(((end))))), "i"(-14), "0"(__gu_err));break; case 2: __asm__ __volatile__( "1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(__gu_err), "=r" (__gu_val) : "m"((*(struct __large_struct *)(((end))))), "i"(-14), "0"(__gu_err));break; case 4: __asm__ __volatile__( "1:	mov""l"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""l"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(__gu_err), "=r" (__gu_val) : "m"((*(struct __large_struct *)(((end))))), "i"(-14), "0"(__gu_err));break; default: (__gu_val) = __get_user_bad(); } } while (0); ((c)) = (__typeof__(*((end))))__gu_val; __gu_err; }); */
/*  } */
/* } */




/* enum bh_state_bits { */
/*  BH_Uptodate, */
/*  BH_Dirty, */
/*  BH_Lock, */
/*  BH_Req, */
/*  BH_Uptodate_Lock, */



/*  BH_Mapped, */
/*  BH_New, */
/*  BH_Async_Read, */
/*  BH_Async_Write, */
/*  BH_Delay, */
/*  BH_Boundary, */
/*  BH_Write_EIO, */
/*  BH_Ordered, */
/*  BH_Eopnotsupp, */

/*  BH_PrivateStart, */


/* }; */



/* struct page; */
/* struct buffer_head; */
/* struct address_space; */
/* typedef void (bh_end_io_t)(struct buffer_head *bh, int uptodate); */

/* struct buffer_head { */
/*  unsigned long b_state; */
/*  struct buffer_head *b_this_page; */
/*  struct page *b_page; */

/*  sector_t b_blocknr; */
/*  size_t b_size; */
/*  char *b_data; */

/*  struct block_device *b_bdev; */
/*  bh_end_io_t *b_end_io; */
/*   void *b_private; */
/*  struct list_head b_assoc_buffers; */
/*  atomic_t b_count; */
/* }; */

/* static inline __attribute__((always_inline)) void set_buffer_uptodate(struct buffer_head *bh) { set_bit(BH_Uptodate, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_uptodate(struct buffer_head *bh) { clear_bit(BH_Uptodate, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_uptodate(const struct buffer_head *bh) { return (__builtin_constant_p(BH_Uptodate) ? constant_test_bit((BH_Uptodate),(&(bh)->b_state)) : variable_test_bit((BH_Uptodate),(&(bh)->b_state))); } */
/* static inline __attribute__((always_inline)) void set_buffer_dirty(struct buffer_head *bh) { set_bit(BH_Dirty, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_dirty(struct buffer_head *bh) { clear_bit(BH_Dirty, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_dirty(const struct buffer_head *bh) { return (__builtin_constant_p(BH_Dirty) ? constant_test_bit((BH_Dirty),(&(bh)->b_state)) : variable_test_bit((BH_Dirty),(&(bh)->b_state))); } */
/* static inline __attribute__((always_inline)) int test_set_buffer_dirty(struct buffer_head *bh) { return test_and_set_bit(BH_Dirty, &(bh)->b_state); } static inline __attribute__((always_inline)) int test_clear_buffer_dirty(struct buffer_head *bh) { return test_and_clear_bit(BH_Dirty, &(bh)->b_state); } */
/* static inline __attribute__((always_inline)) void set_buffer_locked(struct buffer_head *bh) { set_bit(BH_Lock, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_locked(struct buffer_head *bh) { clear_bit(BH_Lock, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_locked(const struct buffer_head *bh) { return (__builtin_constant_p(BH_Lock) ? constant_test_bit((BH_Lock),(&(bh)->b_state)) : variable_test_bit((BH_Lock),(&(bh)->b_state))); } */
/* static inline __attribute__((always_inline)) int test_set_buffer_locked(struct buffer_head *bh) { return test_and_set_bit(BH_Lock, &(bh)->b_state); } static inline __attribute__((always_inline)) int test_clear_buffer_locked(struct buffer_head *bh) { return test_and_clear_bit(BH_Lock, &(bh)->b_state); } */
/* static inline __attribute__((always_inline)) void set_buffer_req(struct buffer_head *bh) { set_bit(BH_Req, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_req(struct buffer_head *bh) { clear_bit(BH_Req, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_req(const struct buffer_head *bh) { return (__builtin_constant_p(BH_Req) ? constant_test_bit((BH_Req),(&(bh)->b_state)) : variable_test_bit((BH_Req),(&(bh)->b_state))); } */
/* static inline __attribute__((always_inline)) int test_set_buffer_req(struct buffer_head *bh) { return test_and_set_bit(BH_Req, &(bh)->b_state); } static inline __attribute__((always_inline)) int test_clear_buffer_req(struct buffer_head *bh) { return test_and_clear_bit(BH_Req, &(bh)->b_state); } */
/* static inline __attribute__((always_inline)) void set_buffer_mapped(struct buffer_head *bh) { set_bit(BH_Mapped, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_mapped(struct buffer_head *bh) { clear_bit(BH_Mapped, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_mapped(const struct buffer_head *bh) { return (__builtin_constant_p(BH_Mapped) ? constant_test_bit((BH_Mapped),(&(bh)->b_state)) : variable_test_bit((BH_Mapped),(&(bh)->b_state))); } */
/* static inline __attribute__((always_inline)) void set_buffer_new(struct buffer_head *bh) { set_bit(BH_New, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_new(struct buffer_head *bh) { clear_bit(BH_New, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_new(const struct buffer_head *bh) { return (__builtin_constant_p(BH_New) ? constant_test_bit((BH_New),(&(bh)->b_state)) : variable_test_bit((BH_New),(&(bh)->b_state))); } */
/* static inline __attribute__((always_inline)) void set_buffer_async_read(struct buffer_head *bh) { set_bit(BH_Async_Read, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_async_read(struct buffer_head *bh) { clear_bit(BH_Async_Read, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_async_read(const struct buffer_head *bh) { return (__builtin_constant_p(BH_Async_Read) ? constant_test_bit((BH_Async_Read),(&(bh)->b_state)) : variable_test_bit((BH_Async_Read),(&(bh)->b_state))); } */
/* static inline __attribute__((always_inline)) void set_buffer_async_write(struct buffer_head *bh) { set_bit(BH_Async_Write, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_async_write(struct buffer_head *bh) { clear_bit(BH_Async_Write, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_async_write(const struct buffer_head *bh) { return (__builtin_constant_p(BH_Async_Write) ? constant_test_bit((BH_Async_Write),(&(bh)->b_state)) : variable_test_bit((BH_Async_Write),(&(bh)->b_state))); } */
/* static inline __attribute__((always_inline)) void set_buffer_delay(struct buffer_head *bh) { set_bit(BH_Delay, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_delay(struct buffer_head *bh) { clear_bit(BH_Delay, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_delay(const struct buffer_head *bh) { return (__builtin_constant_p(BH_Delay) ? constant_test_bit((BH_Delay),(&(bh)->b_state)) : variable_test_bit((BH_Delay),(&(bh)->b_state))); } */
/* static inline __attribute__((always_inline)) void set_buffer_boundary(struct buffer_head *bh) { set_bit(BH_Boundary, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_boundary(struct buffer_head *bh) { clear_bit(BH_Boundary, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_boundary(const struct buffer_head *bh) { return (__builtin_constant_p(BH_Boundary) ? constant_test_bit((BH_Boundary),(&(bh)->b_state)) : variable_test_bit((BH_Boundary),(&(bh)->b_state))); } */
/* static inline __attribute__((always_inline)) void set_buffer_write_io_error(struct buffer_head *bh) { set_bit(BH_Write_EIO, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_write_io_error(struct buffer_head *bh) { clear_bit(BH_Write_EIO, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_write_io_error(const struct buffer_head *bh) { return (__builtin_constant_p(BH_Write_EIO) ? constant_test_bit((BH_Write_EIO),(&(bh)->b_state)) : variable_test_bit((BH_Write_EIO),(&(bh)->b_state))); } */
/* static inline __attribute__((always_inline)) void set_buffer_ordered(struct buffer_head *bh) { set_bit(BH_Ordered, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_ordered(struct buffer_head *bh) { clear_bit(BH_Ordered, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_ordered(const struct buffer_head *bh) { return (__builtin_constant_p(BH_Ordered) ? constant_test_bit((BH_Ordered),(&(bh)->b_state)) : variable_test_bit((BH_Ordered),(&(bh)->b_state))); } */
/* static inline __attribute__((always_inline)) void set_buffer_eopnotsupp(struct buffer_head *bh) { set_bit(BH_Eopnotsupp, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_eopnotsupp(struct buffer_head *bh) { clear_bit(BH_Eopnotsupp, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_eopnotsupp(const struct buffer_head *bh) { return (__builtin_constant_p(BH_Eopnotsupp) ? constant_test_bit((BH_Eopnotsupp),(&(bh)->b_state)) : variable_test_bit((BH_Eopnotsupp),(&(bh)->b_state))); } */

/* void mark_buffer_dirty(struct buffer_head *bh) __attribute__((regparm(3))); */
/* void init_buffer(struct buffer_head *, bh_end_io_t *, void *); */
/* void set_bh_page(struct buffer_head *bh, */
/*   struct page *page, unsigned long offset); */
/* int try_to_free_buffers(struct page *); */
/* struct buffer_head *alloc_page_buffers(struct page *page, unsigned long size, */
/*   int retry); */
/* void create_empty_buffers(struct page *, unsigned long, */
/*    unsigned long b_state); */
/* void end_buffer_read_sync(struct buffer_head *bh, int uptodate); */
/* void end_buffer_write_sync(struct buffer_head *bh, int uptodate); */


/* void mark_buffer_dirty_inode(struct buffer_head *bh, struct inode *inode); */
/* int inode_has_buffers(struct inode *); */
/* void invalidate_inode_buffers(struct inode *); */
/* int remove_inode_buffers(struct inode *inode); */
/* int sync_mapping_buffers(struct address_space *mapping); */
/* void unmap_underlying_metadata(struct block_device *bdev, sector_t block); */

/* void mark_buffer_async_write(struct buffer_head *bh); */
/* void invalidate_bdev(struct block_device *, int); */
/* int sync_blockdev(struct block_device *bdev); */
/* void __wait_on_buffer(struct buffer_head *); */
/* wait_queue_head_t *bh_waitq_head(struct buffer_head *bh); */
/* int fsync_bdev(struct block_device *); */
/* struct super_block *freeze_bdev(struct block_device *); */
/* void thaw_bdev(struct block_device *, struct super_block *); */
/* int fsync_super(struct super_block *); */
/* int fsync_no_super(struct block_device *); */
/* struct buffer_head *__find_get_block(struct block_device *, sector_t, int); */
/* struct buffer_head * __getblk(struct block_device *, sector_t, int); */
/* void __brelse(struct buffer_head *); */
/* void __bforget(struct buffer_head *); */
/* void __breadahead(struct block_device *, sector_t block, int size); */
/* struct buffer_head *__bread(struct block_device *, sector_t block, int size); */
/* struct buffer_head *alloc_buffer_head(gfp_t gfp_flags); */
/* void free_buffer_head(struct buffer_head * bh); */
/* void unlock_buffer(struct buffer_head *bh) __attribute__((regparm(3))); */
/* void __lock_buffer(struct buffer_head *bh) __attribute__((regparm(3))); */
/* void ll_rw_block(int, int, struct buffer_head * bh[]); */
/* int sync_dirty_buffer(struct buffer_head *bh); */
/* int submit_bh(int, struct buffer_head *); */
/* void write_boundary_block(struct block_device *bdev, */
/*    sector_t bblock, unsigned blocksize); */

/* extern int buffer_heads_over_limit; */





/* int try_to_release_page(struct page * page, gfp_t gfp_mask); */
/* void block_invalidatepage(struct page *page, unsigned long offset); */
/* void do_invalidatepage(struct page *page, unsigned long offset); */
/* int block_write_full_page(struct page *page, get_block_t *get_block, */
/*     struct writeback_control *wbc); */
/* int block_read_full_page(struct page*, get_block_t*); */
/* int block_prepare_write(struct page*, unsigned, unsigned, get_block_t*); */
/* int cont_prepare_write(struct page*, unsigned, unsigned, get_block_t*, */
/*     loff_t *); */
/* int generic_cont_expand(struct inode *inode, loff_t size); */
/* int generic_cont_expand_simple(struct inode *inode, loff_t size); */
/* int block_commit_write(struct page *page, unsigned from, unsigned to); */
/* void block_sync_page(struct page *); */
/* sector_t generic_block_bmap(struct address_space *, sector_t, get_block_t *); */
/* int generic_commit_write(struct file *, struct page *, unsigned, unsigned); */
/* int block_truncate_page(struct address_space *, loff_t, get_block_t *); */
/* int file_fsync(struct file *, struct dentry *, int); */
/* int nobh_prepare_write(struct page*, unsigned, unsigned, get_block_t*); */
/* int nobh_commit_write(struct file *, struct page *, unsigned, unsigned); */
/* int nobh_truncate_page(struct address_space *, loff_t); */
/* int nobh_writepage(struct page *page, get_block_t *get_block, */
/*                         struct writeback_control *wbc); */

/* void buffer_init(void); */





/* static inline __attribute__((always_inline)) void attach_page_buffers(struct page *page, */
/*   struct buffer_head *head) */
/* { */
/*  get_page(page); */
/*  set_bit(11, &(page)->flags); */
/*  ((page)->private = ((unsigned long)head)); */
/* } */

/* static inline __attribute__((always_inline)) void get_bh(struct buffer_head *bh) */
/* { */
/*         atomic_inc(&bh->b_count); */
/* } */

/* static inline __attribute__((always_inline)) void put_bh(struct buffer_head *bh) */
/* { */
/*         __asm__ __volatile__("": : :"memory"); */
/*         atomic_dec(&bh->b_count); */
/* } */

/* static inline __attribute__((always_inline)) void brelse(struct buffer_head *bh) */
/* { */
/*  if (bh) */
/*   __brelse(bh); */
/* } */

/* static inline __attribute__((always_inline)) void bforget(struct buffer_head *bh) */
/* { */
/*  if (bh) */
/*   __bforget(bh); */
/* } */

/* static inline __attribute__((always_inline)) struct buffer_head * */
/* sb_bread(struct super_block *sb, sector_t block) */
/* { */
/*  return __bread(sb->s_bdev, block, sb->s_blocksize); */
/* } */

/* static inline __attribute__((always_inline)) void */
/* sb_breadahead(struct super_block *sb, sector_t block) */
/* { */
/*  __breadahead(sb->s_bdev, block, sb->s_blocksize); */
/* } */

/* static inline __attribute__((always_inline)) struct buffer_head * */
/* sb_getblk(struct super_block *sb, sector_t block) */
/* { */
/*  return __getblk(sb->s_bdev, block, sb->s_blocksize); */
/* } */

/* static inline __attribute__((always_inline)) struct buffer_head * */
/* sb_find_get_block(struct super_block *sb, sector_t block) */
/* { */
/*  return __find_get_block(sb->s_bdev, block, sb->s_blocksize); */
/* } */

/* static inline __attribute__((always_inline)) void */
/* map_bh(struct buffer_head *bh, struct super_block *sb, sector_t block) */
/* { */
/*  set_buffer_mapped(bh); */
/*  bh->b_bdev = sb->s_bdev; */
/*  bh->b_blocknr = block; */
/*  bh->b_size = sb->s_blocksize; */
/* } */






/* static inline __attribute__((always_inline)) void wait_on_buffer(struct buffer_head *bh) */
/* { */
/*  do { __might_sleep("include/linux/buffer_head.h", 293); cond_resched(); } while (0); */
/*  if (buffer_locked(bh) || ((&bh->b_count)->counter) == 0) */
/*   __wait_on_buffer(bh); */
/* } */

/* static inline __attribute__((always_inline)) void lock_buffer(struct buffer_head *bh) */
/* { */
/*  do { __might_sleep("include/linux/buffer_head.h", 300); cond_resched(); } while (0); */
/*  if (test_set_buffer_locked(bh)) */
/*   __lock_buffer(bh); */
/* } */










/* struct cdrom_msf0 */
/* { */
/*  __u8 minute; */
/*  __u8 second; */
/*  __u8 frame; */
/* }; */


/* union cdrom_addr */
/* { */
/*  struct cdrom_msf0 msf; */
/*  int lba; */
/* }; */


/* struct cdrom_msf */
/* { */
/*  __u8 cdmsf_min0; */
/*  __u8 cdmsf_sec0; */
/*  __u8 cdmsf_frame0; */
/*  __u8 cdmsf_min1; */
/*  __u8 cdmsf_sec1; */
/*  __u8 cdmsf_frame1; */
/* }; */


/* struct cdrom_ti */
/* { */
/*  __u8 cdti_trk0; */
/*  __u8 cdti_ind0; */
/*  __u8 cdti_trk1; */
/*  __u8 cdti_ind1; */
/* }; */


/* struct cdrom_tochdr */
/* { */
/*  __u8 cdth_trk0; */
/*  __u8 cdth_trk1; */
/* }; */


/* struct cdrom_volctrl */
/* { */
/*  __u8 channel0; */
/*  __u8 channel1; */
/*  __u8 channel2; */
/*  __u8 channel3; */
/* }; */


/* struct cdrom_subchnl */
/* { */
/*  __u8 cdsc_format; */
/*  __u8 cdsc_audiostatus; */
/*  __u8 cdsc_adr: 4; */
/*  __u8 cdsc_ctrl: 4; */
/*  __u8 cdsc_trk; */
/*  __u8 cdsc_ind; */
/*  union cdrom_addr cdsc_absaddr; */
/*  union cdrom_addr cdsc_reladdr; */
/* }; */



/* struct cdrom_tocentry */
/* { */
/*  __u8 cdte_track; */
/*  __u8 cdte_adr :4; */
/*  __u8 cdte_ctrl :4; */
/*  __u8 cdte_format; */
/*  union cdrom_addr cdte_addr; */
/*  __u8 cdte_datamode; */
/* }; */


/* struct cdrom_read */
/* { */
/*  int cdread_lba; */
/*  char *cdread_bufaddr; */
/*  int cdread_buflen; */
/* }; */


/* struct cdrom_read_audio */
/* { */
/*  union cdrom_addr addr; */
/*  __u8 addr_format; */
/*  int nframes; */
/*  __u8 *buf; */
/* }; */


/* struct cdrom_multisession */
/* { */
/*  union cdrom_addr addr; */


/*  __u8 xa_flag; */
/*  __u8 addr_format; */
/* }; */






/* struct cdrom_mcn */
/* { */
/*   __u8 medium_catalog_number[14]; */
/* }; */


/* struct cdrom_blk */
/* { */
/*  unsigned from; */
/*  unsigned short len; */
/* }; */

/* struct cdrom_generic_command */
/* { */
/*  unsigned char cmd[12]; */
/*  unsigned char *buffer; */
/*  unsigned int buflen; */
/*  int stat; */
/*  struct request_sense *sense; */
/*  unsigned char data_direction; */
/*  int quiet; */
/*  int timeout; */
/*  void *reserved[1]; */
/* }; */

/* struct dvd_layer { */
/*  __u8 book_version : 4; */
/*  __u8 book_type : 4; */
/*  __u8 min_rate : 4; */
/*  __u8 disc_size : 4; */
/*  __u8 layer_type : 4; */
/*  __u8 track_path : 1; */
/*  __u8 nlayers : 2; */
/*  __u8 track_density : 4; */
/*  __u8 linear_density : 4; */
/*  __u8 bca : 1; */
/*  __u32 start_sector; */
/*  __u32 end_sector; */
/*  __u32 end_sector_l0; */
/* }; */



/* struct dvd_physical { */
/*  __u8 type; */
/*  __u8 layer_num; */
/*  struct dvd_layer layer[4]; */
/* }; */

/* struct dvd_copyright { */
/*  __u8 type; */

/*  __u8 layer_num; */
/*  __u8 cpst; */
/*  __u8 rmi; */
/* }; */

/* struct dvd_disckey { */
/*  __u8 type; */

/*  unsigned agid : 2; */
/*  __u8 value[2048]; */
/* }; */

/* struct dvd_bca { */
/*  __u8 type; */

/*  int len; */
/*  __u8 value[188]; */
/* }; */

/* struct dvd_manufact { */
/*  __u8 type; */

/*  __u8 layer_num; */
/*  int len; */
/*  __u8 value[2048]; */
/* }; */

/* typedef union { */
/*  __u8 type; */

/*  struct dvd_physical physical; */
/*  struct dvd_copyright copyright; */
/*  struct dvd_disckey disckey; */
/*  struct dvd_bca bca; */
/*  struct dvd_manufact manufact; */
/* } dvd_struct; */

/* typedef __u8 dvd_key[5]; */
/* typedef __u8 dvd_challenge[10]; */

/* struct dvd_lu_send_agid { */
/*  __u8 type; */
/*  unsigned agid : 2; */
/* }; */

/* struct dvd_host_send_challenge { */
/*  __u8 type; */
/*  unsigned agid : 2; */

/*  dvd_challenge chal; */
/* }; */

/* struct dvd_send_key { */
/*  __u8 type; */
/*  unsigned agid : 2; */

/*  dvd_key key; */
/* }; */

/* struct dvd_lu_send_challenge { */
/*  __u8 type; */
/*  unsigned agid : 2; */

/*  dvd_challenge chal; */
/* }; */

/* struct dvd_lu_send_title_key { */
/*  __u8 type; */
/*  unsigned agid : 2; */

/*  dvd_key title_key; */
/*  int lba; */
/*  unsigned cpm : 1; */
/*  unsigned cp_sec : 1; */
/*  unsigned cgms : 2; */
/* }; */

/* struct dvd_lu_send_asf { */
/*  __u8 type; */
/*  unsigned agid : 2; */

/*  unsigned asf : 1; */
/* }; */

/* struct dvd_host_send_rpcstate { */
/*  __u8 type; */
/*  __u8 pdrc; */
/* }; */

/* struct dvd_lu_send_rpcstate { */
/*  __u8 type : 2; */
/*  __u8 vra : 3; */
/*  __u8 ucca : 3; */
/*  __u8 region_mask; */
/*  __u8 rpc_scheme; */
/* }; */

/* typedef union { */
/*  __u8 type; */

/*  struct dvd_lu_send_agid lsa; */
/*  struct dvd_host_send_challenge hsc; */
/*  struct dvd_send_key lsk; */
/*  struct dvd_lu_send_challenge lsc; */
/*  struct dvd_send_key hsk; */
/*  struct dvd_lu_send_title_key lstk; */
/*  struct dvd_lu_send_asf lsasf; */
/*  struct dvd_host_send_rpcstate hrpcs; */
/*  struct dvd_lu_send_rpcstate lrpcs; */
/* } dvd_authinfo; */

/* struct request_sense { */




/*  __u8 error_code : 7; */
/*  __u8 valid : 1; */

/*  __u8 segment_number; */






/*  __u8 sense_key : 4; */
/*  __u8 reserved2 : 1; */
/*  __u8 ili : 1; */
/*  __u8 reserved1 : 2; */

/*  __u8 information[4]; */
/*  __u8 add_sense_len; */
/*  __u8 command_info[4]; */
/*  __u8 asc; */
/*  __u8 ascq; */
/*  __u8 fruc; */
/*  __u8 sks[3]; */
/*  __u8 asb[46]; */
/* }; */

/* struct mrw_feature_desc { */
/*  __u16 feature_code; */






/*  __u8 curr : 1; */
/*  __u8 persistent : 1; */
/*  __u8 feature_version : 4; */
/*  __u8 reserved1 : 2; */

/*  __u8 add_len; */




/*  __u8 write : 1; */
/*  __u8 reserved2 : 7; */

/*  __u8 reserved3; */
/*  __u8 reserved4; */
/*  __u8 reserved5; */
/* }; */


/* struct rwrt_feature_desc { */
/*  __u16 feature_code; */






/*  __u8 curr : 1; */
/*  __u8 persistent : 1; */
/*  __u8 feature_version : 4; */
/*  __u8 reserved1 : 2; */

/*  __u8 add_len; */
/*  __u32 last_lba; */
/*  __u32 block_size; */
/*  __u16 blocking; */




/*  __u8 page_present : 1; */
/*  __u8 reserved2 : 7; */

/*  __u8 reserved3; */
/* }; */

/* typedef struct { */
/*  __u16 disc_information_length; */






/*         __u8 disc_status : 2; */
/*         __u8 border_status : 2; */
/*         __u8 erasable : 1; */
/*  __u8 reserved1 : 3; */



/*  __u8 n_first_track; */
/*  __u8 n_sessions_lsb; */
/*  __u8 first_track_lsb; */
/*  __u8 last_track_lsb; */

/*  __u8 mrw_status : 2; */
/*  __u8 dbit : 1; */
/*         __u8 reserved2 : 2; */
/*         __u8 uru : 1; */
/*         __u8 dbc_v : 1; */
/*  __u8 did_v : 1; */

/*  __u8 disc_type; */
/*  __u8 n_sessions_msb; */
/*  __u8 first_track_msb; */
/*  __u8 last_track_msb; */
/*  __u32 disc_id; */
/*  __u32 lead_in; */
/*  __u32 lead_out; */
/*  __u8 disc_bar_code[8]; */
/*  __u8 reserved3; */
/*  __u8 n_opc; */
/* } disc_information; */

/* typedef struct { */
/*  __u16 track_information_length; */
/*  __u8 track_lsb; */
/*  __u8 session_lsb; */
/*  __u8 reserved1; */

/*         __u8 track_mode : 4; */
/*         __u8 copy : 1; */
/*         __u8 damage : 1; */
/*  __u8 reserved2 : 2; */
/*  __u8 data_mode : 4; */
/*  __u8 fp : 1; */
/*  __u8 packet : 1; */
/*  __u8 blank : 1; */
/*  __u8 rt : 1; */
/*  __u8 nwa_v : 1; */
/*  __u8 lra_v : 1; */
/*  __u8 reserved3 : 6; */

/*  __u32 track_start; */
/*  __u32 next_writable; */
/*  __u32 free_blocks; */
/*  __u32 fixed_packet_size; */
/*  __u32 track_size; */
/*  __u32 last_rec_address; */
/* } track_information; */

/* struct feature_header { */
/*  __u32 data_len; */
/*  __u8 reserved1; */
/*  __u8 reserved2; */
/*  __u16 curr_profile; */
/* }; */

/* struct mode_page_header { */
/*  __u16 mode_data_length; */
/*  __u8 medium_type; */
/*  __u8 reserved1; */
/*  __u8 reserved2; */
/*  __u8 reserved3; */
/*  __u16 desc_length; */
/* }; */







/* struct resource { */
/*  resource_size_t start; */
/*  resource_size_t end; */
/*  const char *name; */
/*  unsigned long flags; */
/*  struct resource *parent, *sibling, *child; */
/* }; */

/* struct resource_list { */
/*  struct resource_list *next; */
/*  struct resource *res; */
/*  struct pci_dev *dev; */
/* }; */

/* extern struct resource ioport_resource; */
/* extern struct resource iomem_resource; */

/* extern int request_resource(struct resource *root, struct resource *new); */
/* extern struct resource * ____request_resource(struct resource *root, struct resource *new); */
/* extern int release_resource(struct resource *new); */
/* extern int insert_resource(struct resource *parent, struct resource *new); */
/* extern int allocate_resource(struct resource *root, struct resource *new, */
/*         resource_size_t size, resource_size_t min, */
/*         resource_size_t max, resource_size_t align, */
/*         void (*alignf)(void *, struct resource *, */
/*          resource_size_t, resource_size_t), */
/*         void *alignf_data); */
/* int adjust_resource(struct resource *res, resource_size_t start, */
/*       resource_size_t size); */


/* extern int find_next_system_ram(struct resource *res); */






/* extern struct resource * __request_region(struct resource *, */
/*      resource_size_t start, */
/*      resource_size_t n, const char *name); */






/* extern int __check_region(struct resource *, resource_size_t, resource_size_t); */
/* extern void __release_region(struct resource *, resource_size_t, */
/*     resource_size_t); */

/* static inline __attribute__((always_inline)) int __attribute__((deprecated)) check_region(resource_size_t s, */
/*       resource_size_t n) */
/* { */
/*  return __check_region(&ioport_resource, s, n); */
/* } */




/* struct klist_node; */
/* struct klist { */
/*  spinlock_t k_lock; */
/*  struct list_head k_list; */
/*  void (*get)(struct klist_node *); */
/*  void (*put)(struct klist_node *); */
/* }; */


/* extern void klist_init(struct klist * k, void (*get)(struct klist_node *), */
/*          void (*put)(struct klist_node *)); */

/* struct klist_node { */
/*  struct klist * n_klist; */
/*  struct list_head n_node; */
/*  struct kref n_ref; */
/*  struct completion n_removed; */
/* }; */

/* extern void klist_add_tail(struct klist_node * n, struct klist * k); */
/* extern void klist_add_head(struct klist_node * n, struct klist * k); */

/* extern void klist_del(struct klist_node * n); */
/* extern void klist_remove(struct klist_node * n); */

/* extern int klist_node_attached(struct klist_node * n); */


/* struct klist_iter { */
/*  struct klist * i_klist; */
/*  struct list_head * i_head; */
/*  struct klist_node * i_cur; */
/* }; */


/* extern void klist_iter_init(struct klist * k, struct klist_iter * i); */
/* extern void klist_iter_init_node(struct klist * k, struct klist_iter * i, */
/*      struct klist_node * n); */
/* extern void klist_iter_exit(struct klist_iter * i); */
/* extern struct klist_node * klist_next(struct klist_iter * i); */


/* struct device; */
/* struct device_driver; */
/* struct class; */
/* struct class_device; */

/* struct bus_type { */
/*  const char * name; */

/*  struct subsystem subsys; */
/*  struct kset drivers; */
/*  struct kset devices; */
/*  struct klist klist_devices; */
/*  struct klist klist_drivers; */

/*  struct bus_attribute * bus_attrs; */
/*  struct device_attribute * dev_attrs; */
/*  struct driver_attribute * drv_attrs; */

/*  int (*match)(struct device * dev, struct device_driver * drv); */
/*  int (*uevent)(struct device *dev, char **envp, */
/*       int num_envp, char *buffer, int buffer_size); */
/*  int (*probe)(struct device * dev); */
/*  int (*remove)(struct device * dev); */
/*  void (*shutdown)(struct device * dev); */
/*  int (*suspend)(struct device * dev, pm_message_t state); */
/*  int (*resume)(struct device * dev); */
/* }; */

/* extern int bus_register(struct bus_type * bus); */
/* extern void bus_unregister(struct bus_type * bus); */

/* extern void bus_rescan_devices(struct bus_type * bus); */



/* int bus_for_each_dev(struct bus_type * bus, struct device * start, void * data, */
/*        int (*fn)(struct device *, void *)); */
/* struct device * bus_find_device(struct bus_type *bus, struct device *start, */
/*     void *data, int (*match)(struct device *, void *)); */

/* int bus_for_each_drv(struct bus_type * bus, struct device_driver * start, */
/*        void * data, int (*fn)(struct device_driver *, void *)); */




/* struct bus_attribute { */
/*  struct attribute attr; */
/*  ssize_t (*show)(struct bus_type *, char * buf); */
/*  ssize_t (*store)(struct bus_type *, const char * buf, size_t count); */
/* }; */




/* extern int bus_create_file(struct bus_type *, struct bus_attribute *); */
/* extern void bus_remove_file(struct bus_type *, struct bus_attribute *); */

/* struct device_driver { */
/*  const char * name; */
/*  struct bus_type * bus; */

/*  struct completion unloaded; */
/*  struct kobject kobj; */
/*  struct klist klist_devices; */
/*  struct klist_node knode_bus; */

/*  struct module * owner; */

/*  int (*probe) (struct device * dev); */
/*  int (*remove) (struct device * dev); */
/*  void (*shutdown) (struct device * dev); */
/*  int (*suspend) (struct device * dev, pm_message_t state); */
/*  int (*resume) (struct device * dev); */
/* }; */


/* extern int driver_register(struct device_driver * drv); */
/* extern void driver_unregister(struct device_driver * drv); */

/* extern struct device_driver * get_driver(struct device_driver * drv); */
/* extern void put_driver(struct device_driver * drv); */
/* extern struct device_driver *driver_find(const char *name, struct bus_type *bus); */




/* struct driver_attribute { */
/*  struct attribute attr; */
/*  ssize_t (*show)(struct device_driver *, char * buf); */
/*  ssize_t (*store)(struct device_driver *, const char * buf, size_t count); */
/* }; */




/* extern int driver_create_file(struct device_driver *, struct driver_attribute *); */
/* extern void driver_remove_file(struct device_driver *, struct driver_attribute *); */

/* extern int driver_for_each_device(struct device_driver * drv, struct device * start, */
/*       void * data, int (*fn)(struct device *, void *)); */
/* struct device * driver_find_device(struct device_driver *drv, */
/*        struct device *start, void *data, */
/*        int (*match)(struct device *, void *)); */





/* struct class { */
/*  const char * name; */
/*  struct module * owner; */

/*  struct subsystem subsys; */
/*  struct list_head children; */
/*  struct list_head devices; */
/*  struct list_head interfaces; */
/*  struct semaphore sem; */

/*  struct class_attribute * class_attrs; */
/*  struct class_device_attribute * class_dev_attrs; */

/*  int (*uevent)(struct class_device *dev, char **envp, */
/*       int num_envp, char *buffer, int buffer_size); */

/*  void (*release)(struct class_device *dev); */
/*  void (*class_release)(struct class *class); */
/* }; */

/* extern int class_register(struct class *); */
/* extern void class_unregister(struct class *); */


/* struct class_attribute { */
/*  struct attribute attr; */
/*  ssize_t (*show)(struct class *, char * buf); */
/*  ssize_t (*store)(struct class *, const char * buf, size_t count); */
/* }; */




/* extern int class_create_file(struct class *, const struct class_attribute *); */
/* extern void class_remove_file(struct class *, const struct class_attribute *); */

/* struct class_device_attribute { */
/*  struct attribute attr; */
/*  ssize_t (*show)(struct class_device *, char * buf); */
/*  ssize_t (*store)(struct class_device *, const char * buf, size_t count); */
/* }; */





/* extern int class_device_create_file(struct class_device *, */
/*         const struct class_device_attribute *); */

/* struct class_device { */
/*  struct list_head node; */

/*  struct kobject kobj; */
/*  struct class * class; */
/*  dev_t devt; */
/*  struct class_device_attribute *devt_attr; */
/*  struct class_device_attribute uevent_attr; */
/*  struct device * dev; */
/*  void * class_data; */
/*  struct class_device *parent; */
/*  struct attribute_group ** groups; */

/*  void (*release)(struct class_device *dev); */
/*  int (*uevent)(struct class_device *dev, char **envp, */
/*       int num_envp, char *buffer, int buffer_size); */
/*  char class_id[20]; */
/* }; */

/* static inline __attribute__((always_inline)) void * */
/* class_get_devdata (struct class_device *dev) */
/* { */
/*  return dev->class_data; */
/* } */

/* static inline __attribute__((always_inline)) void */
/* class_set_devdata (struct class_device *dev, void *data) */
/* { */
/*  dev->class_data = data; */
/* } */


/* extern int class_device_register(struct class_device *); */
/* extern void class_device_unregister(struct class_device *); */
/* extern void class_device_initialize(struct class_device *); */
/* extern int class_device_add(struct class_device *); */
/* extern void class_device_del(struct class_device *); */

/* extern int class_device_rename(struct class_device *, char *); */

/* extern struct class_device * class_device_get(struct class_device *); */
/* extern void class_device_put(struct class_device *); */

/* extern void class_device_remove_file(struct class_device *, */
/*          const struct class_device_attribute *); */
/* extern int class_device_create_bin_file(struct class_device *, */
/*      struct bin_attribute *); */
/* extern void class_device_remove_bin_file(struct class_device *, */
/*       struct bin_attribute *); */

/* struct class_interface { */
/*  struct list_head node; */
/*  struct class *class; */

/*  int (*add) (struct class_device *, struct class_interface *); */
/*  void (*remove) (struct class_device *, struct class_interface *); */
/* }; */

/* extern int class_interface_register(struct class_interface *); */
/* extern void class_interface_unregister(struct class_interface *); */

/* extern struct class *class_create(struct module *owner, char *name); */
/* extern void class_destroy(struct class *cls); */
/* extern struct class_device *class_device_create(struct class *cls, */
/*       struct class_device *parent, */
/*       dev_t devt, */
/*       struct device *device, */
/*       char *fmt, ...) */
/*      __attribute__((format(printf,5,6))); */
/* extern void class_device_destroy(struct class *cls, dev_t devt); */



/* struct device_attribute { */
/*  struct attribute attr; */
/*  ssize_t (*show)(struct device *dev, struct device_attribute *attr, */
/*    char *buf); */
/*  ssize_t (*store)(struct device *dev, struct device_attribute *attr, */
/*     const char *buf, size_t count); */
/* }; */




/* extern int device_create_file(struct device *device, struct device_attribute * entry); */
/* extern void device_remove_file(struct device * dev, struct device_attribute * attr); */
/* struct device { */
/*  struct klist klist_children; */
/*  struct klist_node knode_parent; */
/*  struct klist_node knode_driver; */
/*  struct klist_node knode_bus; */
/*  struct device * parent; */

/*  struct kobject kobj; */
/*  char bus_id[20]; */
/*  struct device_attribute uevent_attr; */
/*  struct device_attribute *devt_attr; */

/*  struct semaphore sem; */



/*  struct bus_type * bus; */
/*  struct device_driver *driver; */

/*  void *driver_data; */
/*  void *platform_data; */

/*  void *firmware_data; */

/*  struct dev_pm_info power; */

/*  u64 *dma_mask; */
/*  u64 coherent_dma_mask; */





/*  struct list_head dma_pools; */

/*  struct dma_coherent_mem *dma_mem; */



/*  struct list_head node; */
/*  struct class *class; */
/*  dev_t devt; */

/*  void (*release)(struct device * dev); */
/* }; */

/* static inline __attribute__((always_inline)) void * */
/* dev_get_drvdata (struct device *dev) */
/* { */
/*  return dev->driver_data; */
/* } */

/* static inline __attribute__((always_inline)) void */
/* dev_set_drvdata (struct device *dev, void *data) */
/* { */
/*  dev->driver_data = data; */
/* } */

/* static inline __attribute__((always_inline)) int device_is_registered(struct device *dev) */
/* { */
/*  return klist_node_attached(&dev->knode_bus); */
/* } */




/* extern int device_register(struct device * dev); */
/* extern void device_unregister(struct device * dev); */
/* extern void device_initialize(struct device * dev); */
/* extern int device_add(struct device * dev); */
/* extern void device_del(struct device * dev); */
/* extern int device_for_each_child(struct device *, void *, */
/*        int (*fn)(struct device *, void *)); */





/* extern void device_bind_driver(struct device * dev); */
/* extern void device_release_driver(struct device * dev); */
/* extern int device_attach(struct device * dev); */
/* extern void driver_attach(struct device_driver * drv); */
/* extern void device_reprobe(struct device *dev); */




/* extern struct device *device_create(struct class *cls, struct device *parent, */
/*         dev_t devt, char *fmt, ...) */
/*         __attribute__((format(printf,4,5))); */
/* extern void device_destroy(struct class *cls, dev_t devt); */







/* extern int (*platform_notify)(struct device * dev); */

/* extern int (*platform_notify_remove)(struct device * dev); */






/* extern struct device * get_device(struct device * dev); */
/* extern void put_device(struct device * dev); */



/* extern void device_shutdown(void); */



/* extern int firmware_register(struct subsystem *); */
/* extern void firmware_unregister(struct subsystem *); */


/* extern const char *dev_driver_string(struct device *dev); */


/* struct packet_command */
/* { */
/*  unsigned char cmd[12]; */
/*  unsigned char *buffer; */
/*  unsigned int buflen; */
/*  int stat; */
/*  struct request_sense *sense; */
/*  unsigned char data_direction; */
/*  int quiet; */
/*  int timeout; */
/*  void *reserved[1]; */
/* }; */

/* struct cdrom_device_info { */
/*  struct cdrom_device_ops *ops; */
/*  struct cdrom_device_info *next; */
/*  struct gendisk *disk; */
/*  void *handle; */

/*  int mask; */
/*  int speed; */
/*  int capacity; */

/*  int options : 30; */
/*  unsigned mc_flags : 2; */
/*      int use_count; */
/*      char name[20]; */

/*         __u8 sanyo_slot : 2; */
/*         __u8 reserved : 6; */
/*  int cdda_method; */
/*  __u8 last_sense; */
/*  __u8 media_written; */
/*  unsigned short mmc3_profile; */
/*  int for_data; */
/*  int (*exit)(struct cdrom_device_info *); */
/*  int mrw_mode_page; */
/* }; */

/* struct cdrom_device_ops { */

/*  int (*open) (struct cdrom_device_info *, int); */
/*  void (*release) (struct cdrom_device_info *); */
/*  int (*drive_status) (struct cdrom_device_info *, int); */
/*  int (*media_changed) (struct cdrom_device_info *, int); */
/*  int (*tray_move) (struct cdrom_device_info *, int); */
/*  int (*lock_door) (struct cdrom_device_info *, int); */
/*  int (*select_speed) (struct cdrom_device_info *, int); */
/*  int (*select_disc) (struct cdrom_device_info *, int); */
/*  int (*get_last_session) (struct cdrom_device_info *, */
/*      struct cdrom_multisession *); */
/*  int (*get_mcn) (struct cdrom_device_info *, */
/*    struct cdrom_mcn *); */

/*  int (*reset) (struct cdrom_device_info *); */

/*  int (*audio_ioctl) (struct cdrom_device_info *,unsigned int, void *); */


/*  const int capability; */
/*  int n_minors; */

/*  int (*generic_packet) (struct cdrom_device_info *, */
/*           struct packet_command *); */
/* }; */


/* extern int cdrom_open(struct cdrom_device_info *cdi, struct inode *ip, */
/*    struct file *fp); */
/* extern int cdrom_release(struct cdrom_device_info *cdi, struct file *fp); */
/* extern int cdrom_ioctl(struct file *file, struct cdrom_device_info *cdi, */
/*   struct inode *ip, unsigned int cmd, unsigned long arg); */
/* extern int cdrom_media_changed(struct cdrom_device_info *); */

/* extern int register_cdrom(struct cdrom_device_info *cdi); */
/* extern int unregister_cdrom(struct cdrom_device_info *cdi); */

/* typedef struct { */
/*     int data; */
/*     int audio; */
/*     int cdi; */
/*     int xa; */
/*     long error; */
/* } tracktype; */

/* extern int cdrom_get_last_written(struct cdrom_device_info *cdi, long *last_written); */
/* extern int cdrom_number_of_slots(struct cdrom_device_info *cdi); */
/* extern int cdrom_mode_select(struct cdrom_device_info *cdi, */
/*         struct packet_command *cgc); */
/* extern int cdrom_mode_sense(struct cdrom_device_info *cdi, */
/*        struct packet_command *cgc, */
/*        int page_code, int page_control); */
/* extern void init_cdrom_command(struct packet_command *cgc, */
/*           void *buffer, int len, int type); */




/* struct cdrom_mechstat_header { */

/*  __u8 curslot : 5; */
/*  __u8 changer_state : 2; */
/*  __u8 fault : 1; */
/*  __u8 reserved1 : 4; */
/*  __u8 door_open : 1; */
/*  __u8 mech_state : 3; */

/*  __u8 curlba[3]; */
/*  __u8 nslots; */
/*  __u16 slot_tablelen; */
/* }; */

/* struct cdrom_slot { */





/*  __u8 change : 1; */
/*  __u8 reserved1 : 6; */
/*  __u8 disc_present : 1; */

/*  __u8 reserved2[3]; */
/* }; */

/* struct cdrom_changer_info { */
/*  struct cdrom_mechstat_header hdr; */
/*  struct cdrom_slot slots[256]; */
/* }; */

/* typedef enum { */
/*  mechtype_caddy = 0, */
/*  mechtype_tray = 1, */
/*  mechtype_popup = 2, */
/*  mechtype_individual_changer = 4, */
/*  mechtype_cartridge_changer = 5 */
/* } mechtype_t; */

/* typedef struct { */

/*  __u8 page_code : 6; */
/*  __u8 reserved1 : 1; */
/*  __u8 ps : 1; */
/*         __u8 page_length; */
/*         __u8 write_type : 4; */
/*  __u8 test_write : 1; */
/*  __u8 ls_v : 1; */
/*  __u8 bufe : 1; */
/*  __u8 reserved2 : 1; */
/*  __u8 track_mode : 4; */
/*  __u8 copy : 1; */
/*  __u8 fp : 1; */
/*  __u8 multi_session : 2; */
/*  __u8 data_block_type : 4; */
/*  __u8 reserved3 : 4; */

/*  __u8 link_size; */
/*  __u8 reserved4; */




/*  __u8 app_code : 6; */
/*  __u8 reserved5 : 2; */

/*  __u8 session_format; */
/*  __u8 reserved6; */
/*  __u32 packet_size; */
/*  __u16 audio_pause; */
/*  __u8 mcn[16]; */
/*  __u8 isrc[16]; */
/*  __u8 subhdr0; */
/*  __u8 subhdr1; */
/*  __u8 subhdr2; */
/*  __u8 subhdr3; */
/* } __attribute__((packed)) write_param_page; */

/* struct modesel_head */
/* { */
/*  __u8 reserved1; */
/*  __u8 medium; */
/*  __u8 reserved2; */
/*  __u8 block_desc_length; */
/*  __u8 density; */
/*  __u8 number_of_blocks_hi; */
/*  __u8 number_of_blocks_med; */
/*  __u8 number_of_blocks_lo; */
/*  __u8 reserved3; */
/*  __u8 block_length_hi; */
/*  __u8 block_length_med; */
/*  __u8 block_length_lo; */
/* }; */

/* typedef struct { */
/*  __u16 report_key_length; */
/*  __u8 reserved1; */
/*  __u8 reserved2; */





/*  __u8 ucca : 3; */
/*  __u8 vra : 3; */
/*  __u8 type_code : 2; */

/*  __u8 region_mask; */
/*  __u8 rpc_scheme; */
/*  __u8 reserved3; */
/* } rpc_state_t; */

/* struct event_header { */
/*  __u16 data_len; */





/*  __u8 notification_class : 3; */
/*  __u8 reserved1 : 4; */
/*  __u8 nea : 1; */

/*  __u8 supp_event_class; */
/* }; */

/* struct media_event_desc { */







/*  __u8 media_event_code : 4; */
/*  __u8 reserved1 : 4; */
/*  __u8 door_open : 1; */
/*  __u8 media_present : 1; */
/*  __u8 reserved2 : 6; */

/*  __u8 start_slot; */
/*  __u8 end_slot; */
/* }; */

/* extern int cdrom_get_media_event(struct cdrom_device_info *cdi, struct media_event_desc *med); */



/* struct file; */
/* struct completion; */






/* struct __sysctl_args { */
/*  int *name; */
/*  int nlen; */
/*  void *oldval; */
/*  size_t *oldlenp; */
/*  void *newval; */
/*  size_t newlen; */
/*  unsigned long __unused[4]; */
/* }; */

/* enum */
/* { */
/*  CTL_KERN=1, */
/*  CTL_VM=2, */
/*  CTL_NET=3, */

/*  CTL_FS=5, */
/*  CTL_DEBUG=6, */
/*  CTL_DEV=7, */
/*  CTL_BUS=8, */
/*  CTL_ABI=9, */
/*  CTL_CPU=10 */
/* }; */


/* enum */
/* { */
/*  CTL_BUS_ISA=1 */
/* }; */


/* enum */
/* { */
/*  INOTIFY_MAX_USER_INSTANCES=1, */
/*  INOTIFY_MAX_USER_WATCHES=2, */
/*  INOTIFY_MAX_QUEUED_EVENTS=3 */
/* }; */


/* enum */
/* { */
/*  KERN_OSTYPE=1, */
/*  KERN_OSRELEASE=2, */
/*  KERN_OSREV=3, */
/*  KERN_VERSION=4, */
/*  KERN_SECUREMASK=5, */
/*  KERN_PROF=6, */
/*  KERN_NODENAME=7, */
/*  KERN_DOMAINNAME=8, */

/*  KERN_CAP_BSET=14, */
/*  KERN_PANIC=15, */
/*  KERN_REALROOTDEV=16, */

/*  KERN_SPARC_REBOOT=21, */
/*  KERN_CTLALTDEL=22, */
/*  KERN_PRINTK=23, */
/*  KERN_NAMETRANS=24, */
/*  KERN_PPC_HTABRECLAIM=25, */
/*  KERN_PPC_ZEROPAGED=26, */
/*  KERN_PPC_POWERSAVE_NAP=27, */
/*  KERN_MODPROBE=28, */
/*  KERN_SG_BIG_BUFF=29, */
/*  KERN_ACCT=30, */
/*  KERN_PPC_L2CR=31, */

/*  KERN_RTSIGNR=32, */
/*  KERN_RTSIGMAX=33, */

/*  KERN_SHMMAX=34, */
/*  KERN_MSGMAX=35, */
/*  KERN_MSGMNB=36, */
/*  KERN_MSGPOOL=37, */
/*  KERN_SYSRQ=38, */
/*  KERN_MAX_THREADS=39, */
/*   KERN_RANDOM=40, */
/*   KERN_SHMALL=41, */
/*   KERN_MSGMNI=42, */
/*   KERN_SEM=43, */
/*   KERN_SPARC_STOP_A=44, */
/*   KERN_SHMMNI=45, */
/*  KERN_OVERFLOWUID=46, */
/*  KERN_OVERFLOWGID=47, */
/*  KERN_SHMPATH=48, */
/*  KERN_HOTPLUG=49, */
/*  KERN_IEEE_EMULATION_WARNINGS=50, */
/*  KERN_S390_USER_DEBUG_LOGGING=51, */
/*  KERN_CORE_USES_PID=52, */
/*  KERN_TAINTED=53, */
/*  KERN_CADPID=54, */
/*  KERN_PIDMAX=55, */
/*    KERN_CORE_PATTERN=56, */
/*  KERN_PANIC_ON_OOPS=57, */
/*  KERN_HPPA_PWRSW=58, */
/*  KERN_HPPA_UNALIGNED=59, */
/*  KERN_PRINTK_RATELIMIT=60, */
/*  KERN_PRINTK_RATELIMIT_BURST=61, */
/*  KERN_PTY=62, */
/*  KERN_NGROUPS_MAX=63, */
/*  KERN_SPARC_SCONS_PWROFF=64, */
/*  KERN_HZ_TIMER=65, */
/*  KERN_UNKNOWN_NMI_PANIC=66, */
/*  KERN_BOOTLOADER_TYPE=67, */
/*  KERN_RANDOMIZE=68, */
/*  KERN_SETUID_DUMPABLE=69, */
/*  KERN_SPIN_RETRY=70, */
/*  KERN_ACPI_VIDEO_FLAGS=71, */
/*  KERN_IA64_UNALIGNED=72, */
/*  KERN_COMPAT_LOG=73, */
/*  KERN_MAX_LOCK_DEPTH=74, */
/* }; */




/* enum */
/* { */
/*  VM_UNUSED1=1, */
/*  VM_UNUSED2=2, */
/*  VM_UNUSED3=3, */
/*  VM_UNUSED4=4, */
/*  VM_OVERCOMMIT_MEMORY=5, */
/*  VM_UNUSED5=6, */
/*  VM_UNUSED7=7, */
/*  VM_UNUSED8=8, */
/*  VM_UNUSED9=9, */
/*  VM_PAGE_CLUSTER=10, */
/*  VM_DIRTY_BACKGROUND=11, */
/*  VM_DIRTY_RATIO=12, */
/*  VM_DIRTY_WB_CS=13, */
/*  VM_DIRTY_EXPIRE_CS=14, */
/*  VM_NR_PDFLUSH_THREADS=15, */
/*  VM_OVERCOMMIT_RATIO=16, */
/*  VM_PAGEBUF=17, */
/*  VM_HUGETLB_PAGES=18, */
/*  VM_SWAPPINESS=19, */
/*  VM_LOWMEM_RESERVE_RATIO=20, */
/*  VM_MIN_FREE_KBYTES=21, */
/*  VM_MAX_MAP_COUNT=22, */
/*  VM_LAPTOP_MODE=23, */
/*  VM_BLOCK_DUMP=24, */
/*  VM_HUGETLB_GROUP=25, */
/*  VM_VFS_CACHE_PRESSURE=26, */
/*  VM_LEGACY_VA_LAYOUT=27, */
/*  VM_SWAP_TOKEN_TIMEOUT=28, */
/*  VM_DROP_PAGECACHE=29, */
/*  VM_PERCPU_PAGELIST_FRACTION=30, */
/*  VM_ZONE_RECLAIM_MODE=31, */
/*  VM_MIN_UNMAPPED=32, */
/*  VM_PANIC_ON_OOM=33, */
/*  VM_VDSO_ENABLED=34, */
/*  VM_MIN_SLAB=35, */
/* }; */



/* enum */
/* { */
/*  NET_CORE=1, */
/*  NET_ETHER=2, */
/*  NET_802=3, */
/*  NET_UNIX=4, */
/*  NET_IPV4=5, */
/*  NET_IPX=6, */
/*  NET_ATALK=7, */
/*  NET_NETROM=8, */
/*  NET_AX25=9, */
/*  NET_BRIDGE=10, */
/*  NET_ROSE=11, */
/*  NET_IPV6=12, */
/*  NET_X25=13, */
/*  NET_TR=14, */
/*  NET_DECNET=15, */
/*  NET_ECONET=16, */
/*  NET_SCTP=17, */
/*  NET_LLC=18, */
/*  NET_NETFILTER=19, */
/*  NET_DCCP=20, */
/* }; */


/* enum */
/* { */
/*  RANDOM_POOLSIZE=1, */
/*  RANDOM_ENTROPY_COUNT=2, */
/*  RANDOM_READ_THRESH=3, */
/*  RANDOM_WRITE_THRESH=4, */
/*  RANDOM_BOOT_ID=5, */
/*  RANDOM_UUID=6 */
/* }; */


/* enum */
/* { */
/*  PTY_MAX=1, */
/*  PTY_NR=2 */
/* }; */


/* enum */
/* { */
/*  BUS_ISA_MEM_BASE=1, */
/*  BUS_ISA_PORT_BASE=2, */
/*  BUS_ISA_PORT_SHIFT=3 */
/* }; */


/* enum */
/* { */
/*  NET_CORE_WMEM_MAX=1, */
/*  NET_CORE_RMEM_MAX=2, */
/*  NET_CORE_WMEM_DEFAULT=3, */
/*  NET_CORE_RMEM_DEFAULT=4, */

/*  NET_CORE_MAX_BACKLOG=6, */
/*  NET_CORE_FASTROUTE=7, */
/*  NET_CORE_MSG_COST=8, */
/*  NET_CORE_MSG_BURST=9, */
/*  NET_CORE_OPTMEM_MAX=10, */
/*  NET_CORE_HOT_LIST_LENGTH=11, */
/*  NET_CORE_DIVERT_VERSION=12, */
/*  NET_CORE_NO_CONG_THRESH=13, */
/*  NET_CORE_NO_CONG=14, */
/*  NET_CORE_LO_CONG=15, */
/*  NET_CORE_MOD_CONG=16, */
/*  NET_CORE_DEV_WEIGHT=17, */
/*  NET_CORE_SOMAXCONN=18, */
/*  NET_CORE_BUDGET=19, */
/*  NET_CORE_AEVENT_ETIME=20, */
/*  NET_CORE_AEVENT_RSEQTH=21, */
/* }; */







/* enum */
/* { */
/*  NET_UNIX_DESTROY_DELAY=1, */
/*  NET_UNIX_DELETE_DELAY=2, */
/*  NET_UNIX_MAX_DGRAM_QLEN=3, */
/* }; */


/* enum */
/* { */
/*  NET_NF_CONNTRACK_MAX=1, */
/*  NET_NF_CONNTRACK_TCP_TIMEOUT_SYN_SENT=2, */
/*  NET_NF_CONNTRACK_TCP_TIMEOUT_SYN_RECV=3, */
/*  NET_NF_CONNTRACK_TCP_TIMEOUT_ESTABLISHED=4, */
/*  NET_NF_CONNTRACK_TCP_TIMEOUT_FIN_WAIT=5, */
/*  NET_NF_CONNTRACK_TCP_TIMEOUT_CLOSE_WAIT=6, */
/*  NET_NF_CONNTRACK_TCP_TIMEOUT_LAST_ACK=7, */
/*  NET_NF_CONNTRACK_TCP_TIMEOUT_TIME_WAIT=8, */
/*  NET_NF_CONNTRACK_TCP_TIMEOUT_CLOSE=9, */
/*  NET_NF_CONNTRACK_UDP_TIMEOUT=10, */
/*  NET_NF_CONNTRACK_UDP_TIMEOUT_STREAM=11, */
/*  NET_NF_CONNTRACK_ICMP_TIMEOUT=12, */
/*  NET_NF_CONNTRACK_GENERIC_TIMEOUT=13, */
/*  NET_NF_CONNTRACK_BUCKETS=14, */
/*  NET_NF_CONNTRACK_LOG_INVALID=15, */
/*  NET_NF_CONNTRACK_TCP_TIMEOUT_MAX_RETRANS=16, */
/*  NET_NF_CONNTRACK_TCP_LOOSE=17, */
/*  NET_NF_CONNTRACK_TCP_BE_LIBERAL=18, */
/*  NET_NF_CONNTRACK_TCP_MAX_RETRANS=19, */
/*  NET_NF_CONNTRACK_SCTP_TIMEOUT_CLOSED=20, */
/*  NET_NF_CONNTRACK_SCTP_TIMEOUT_COOKIE_WAIT=21, */
/*  NET_NF_CONNTRACK_SCTP_TIMEOUT_COOKIE_ECHOED=22, */
/*  NET_NF_CONNTRACK_SCTP_TIMEOUT_ESTABLISHED=23, */
/*  NET_NF_CONNTRACK_SCTP_TIMEOUT_SHUTDOWN_SENT=24, */
/*  NET_NF_CONNTRACK_SCTP_TIMEOUT_SHUTDOWN_RECD=25, */
/*  NET_NF_CONNTRACK_SCTP_TIMEOUT_SHUTDOWN_ACK_SENT=26, */
/*  NET_NF_CONNTRACK_COUNT=27, */
/*  NET_NF_CONNTRACK_ICMPV6_TIMEOUT=28, */
/*  NET_NF_CONNTRACK_FRAG6_TIMEOUT=29, */
/*  NET_NF_CONNTRACK_FRAG6_LOW_THRESH=30, */
/*  NET_NF_CONNTRACK_FRAG6_HIGH_THRESH=31, */
/*  NET_NF_CONNTRACK_CHECKSUM=32, */
/* }; */


/* enum */
/* { */

/*  NET_IPV4_FORWARD=8, */
/*  NET_IPV4_DYNADDR=9, */

/*  NET_IPV4_CONF=16, */
/*  NET_IPV4_NEIGH=17, */
/*  NET_IPV4_ROUTE=18, */
/*  NET_IPV4_FIB_HASH=19, */
/*  NET_IPV4_NETFILTER=20, */

/*  NET_IPV4_TCP_TIMESTAMPS=33, */
/*  NET_IPV4_TCP_WINDOW_SCALING=34, */
/*  NET_IPV4_TCP_SACK=35, */
/*  NET_IPV4_TCP_RETRANS_COLLAPSE=36, */
/*  NET_IPV4_DEFAULT_TTL=37, */
/*  NET_IPV4_AUTOCONFIG=38, */
/*  NET_IPV4_NO_PMTU_DISC=39, */
/*  NET_IPV4_TCP_SYN_RETRIES=40, */
/*  NET_IPV4_IPFRAG_HIGH_THRESH=41, */
/*  NET_IPV4_IPFRAG_LOW_THRESH=42, */
/*  NET_IPV4_IPFRAG_TIME=43, */
/*  NET_IPV4_TCP_MAX_KA_PROBES=44, */
/*  NET_IPV4_TCP_KEEPALIVE_TIME=45, */
/*  NET_IPV4_TCP_KEEPALIVE_PROBES=46, */
/*  NET_IPV4_TCP_RETRIES1=47, */
/*  NET_IPV4_TCP_RETRIES2=48, */
/*  NET_IPV4_TCP_FIN_TIMEOUT=49, */
/*  NET_IPV4_IP_MASQ_DEBUG=50, */
/*  NET_TCP_SYNCOOKIES=51, */
/*  NET_TCP_STDURG=52, */
/*  NET_TCP_RFC1337=53, */
/*  NET_TCP_SYN_TAILDROP=54, */
/*  NET_TCP_MAX_SYN_BACKLOG=55, */
/*  NET_IPV4_LOCAL_PORT_RANGE=56, */
/*  NET_IPV4_ICMP_ECHO_IGNORE_ALL=57, */
/*  NET_IPV4_ICMP_ECHO_IGNORE_BROADCASTS=58, */
/*  NET_IPV4_ICMP_SOURCEQUENCH_RATE=59, */
/*  NET_IPV4_ICMP_DESTUNREACH_RATE=60, */
/*  NET_IPV4_ICMP_TIMEEXCEED_RATE=61, */
/*  NET_IPV4_ICMP_PARAMPROB_RATE=62, */
/*  NET_IPV4_ICMP_ECHOREPLY_RATE=63, */
/*  NET_IPV4_ICMP_IGNORE_BOGUS_ERROR_RESPONSES=64, */
/*  NET_IPV4_IGMP_MAX_MEMBERSHIPS=65, */
/*  NET_TCP_TW_RECYCLE=66, */
/*  NET_IPV4_ALWAYS_DEFRAG=67, */
/*  NET_IPV4_TCP_KEEPALIVE_INTVL=68, */
/*  NET_IPV4_INET_PEER_THRESHOLD=69, */
/*  NET_IPV4_INET_PEER_MINTTL=70, */
/*  NET_IPV4_INET_PEER_MAXTTL=71, */
/*  NET_IPV4_INET_PEER_GC_MINTIME=72, */
/*  NET_IPV4_INET_PEER_GC_MAXTIME=73, */
/*  NET_TCP_ORPHAN_RETRIES=74, */
/*  NET_TCP_ABORT_ON_OVERFLOW=75, */
/*  NET_TCP_SYNACK_RETRIES=76, */
/*  NET_TCP_MAX_ORPHANS=77, */
/*  NET_TCP_MAX_TW_BUCKETS=78, */
/*  NET_TCP_FACK=79, */
/*  NET_TCP_REORDERING=80, */
/*  NET_TCP_ECN=81, */
/*  NET_TCP_DSACK=82, */
/*  NET_TCP_MEM=83, */
/*  NET_TCP_WMEM=84, */
/*  NET_TCP_RMEM=85, */
/*  NET_TCP_APP_WIN=86, */
/*  NET_TCP_ADV_WIN_SCALE=87, */
/*  NET_IPV4_NONLOCAL_BIND=88, */
/*  NET_IPV4_ICMP_RATELIMIT=89, */
/*  NET_IPV4_ICMP_RATEMASK=90, */
/*  NET_TCP_TW_REUSE=91, */
/*  NET_TCP_FRTO=92, */
/*  NET_TCP_LOW_LATENCY=93, */
/*  NET_IPV4_IPFRAG_SECRET_INTERVAL=94, */
/*  NET_IPV4_IGMP_MAX_MSF=96, */
/*  NET_TCP_NO_METRICS_SAVE=97, */
/*  NET_TCP_DEFAULT_WIN_SCALE=105, */
/*  NET_TCP_MODERATE_RCVBUF=106, */
/*  NET_TCP_TSO_WIN_DIVISOR=107, */
/*  NET_TCP_BIC_BETA=108, */
/*  NET_IPV4_ICMP_ERRORS_USE_INBOUND_IFADDR=109, */
/*  NET_TCP_CONG_CONTROL=110, */
/*  NET_TCP_ABC=111, */
/*  NET_IPV4_IPFRAG_MAX_DIST=112, */
/*   NET_TCP_MTU_PROBING=113, */
/*  NET_TCP_BASE_MSS=114, */
/*  NET_IPV4_TCP_WORKAROUND_SIGNED_WINDOWS=115, */
/*  NET_TCP_DMA_COPYBREAK=116, */
/*  NET_TCP_SLOW_START_AFTER_IDLE=117, */
/* }; */

/* enum { */
/*  NET_IPV4_ROUTE_FLUSH=1, */
/*  NET_IPV4_ROUTE_MIN_DELAY=2, */
/*  NET_IPV4_ROUTE_MAX_DELAY=3, */
/*  NET_IPV4_ROUTE_GC_THRESH=4, */
/*  NET_IPV4_ROUTE_MAX_SIZE=5, */
/*  NET_IPV4_ROUTE_GC_MIN_INTERVAL=6, */
/*  NET_IPV4_ROUTE_GC_TIMEOUT=7, */
/*  NET_IPV4_ROUTE_GC_INTERVAL=8, */
/*  NET_IPV4_ROUTE_REDIRECT_LOAD=9, */
/*  NET_IPV4_ROUTE_REDIRECT_NUMBER=10, */
/*  NET_IPV4_ROUTE_REDIRECT_SILENCE=11, */
/*  NET_IPV4_ROUTE_ERROR_COST=12, */
/*  NET_IPV4_ROUTE_ERROR_BURST=13, */
/*  NET_IPV4_ROUTE_GC_ELASTICITY=14, */
/*  NET_IPV4_ROUTE_MTU_EXPIRES=15, */
/*  NET_IPV4_ROUTE_MIN_PMTU=16, */
/*  NET_IPV4_ROUTE_MIN_ADVMSS=17, */
/*  NET_IPV4_ROUTE_SECRET_INTERVAL=18, */
/*  NET_IPV4_ROUTE_GC_MIN_INTERVAL_MS=19, */
/* }; */

/* enum */
/* { */
/*  NET_PROTO_CONF_ALL=-2, */
/*  NET_PROTO_CONF_DEFAULT=-3 */


/* }; */

/* enum */
/* { */
/*  NET_IPV4_CONF_FORWARDING=1, */
/*  NET_IPV4_CONF_MC_FORWARDING=2, */
/*  NET_IPV4_CONF_PROXY_ARP=3, */
/*  NET_IPV4_CONF_ACCEPT_REDIRECTS=4, */
/*  NET_IPV4_CONF_SECURE_REDIRECTS=5, */
/*  NET_IPV4_CONF_SEND_REDIRECTS=6, */
/*  NET_IPV4_CONF_SHARED_MEDIA=7, */
/*  NET_IPV4_CONF_RP_FILTER=8, */
/*  NET_IPV4_CONF_ACCEPT_SOURCE_ROUTE=9, */
/*  NET_IPV4_CONF_BOOTP_RELAY=10, */
/*  NET_IPV4_CONF_LOG_MARTIANS=11, */
/*  NET_IPV4_CONF_TAG=12, */
/*  NET_IPV4_CONF_ARPFILTER=13, */
/*  NET_IPV4_CONF_MEDIUM_ID=14, */
/*  NET_IPV4_CONF_NOXFRM=15, */
/*  NET_IPV4_CONF_NOPOLICY=16, */
/*  NET_IPV4_CONF_FORCE_IGMP_VERSION=17, */
/*  NET_IPV4_CONF_ARP_ANNOUNCE=18, */
/*  NET_IPV4_CONF_ARP_IGNORE=19, */
/*  NET_IPV4_CONF_PROMOTE_SECONDARIES=20, */
/*  NET_IPV4_CONF_ARP_ACCEPT=21, */
/*  __NET_IPV4_CONF_MAX */
/* }; */


/* enum */
/* { */
/*  NET_IPV4_NF_CONNTRACK_MAX=1, */
/*  NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_SYN_SENT=2, */
/*  NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_SYN_RECV=3, */
/*  NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_ESTABLISHED=4, */
/*  NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_FIN_WAIT=5, */
/*  NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_CLOSE_WAIT=6, */
/*  NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_LAST_ACK=7, */
/*  NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_TIME_WAIT=8, */
/*  NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_CLOSE=9, */
/*  NET_IPV4_NF_CONNTRACK_UDP_TIMEOUT=10, */
/*  NET_IPV4_NF_CONNTRACK_UDP_TIMEOUT_STREAM=11, */
/*  NET_IPV4_NF_CONNTRACK_ICMP_TIMEOUT=12, */
/*  NET_IPV4_NF_CONNTRACK_GENERIC_TIMEOUT=13, */
/*  NET_IPV4_NF_CONNTRACK_BUCKETS=14, */
/*  NET_IPV4_NF_CONNTRACK_LOG_INVALID=15, */
/*  NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_MAX_RETRANS=16, */
/*  NET_IPV4_NF_CONNTRACK_TCP_LOOSE=17, */
/*  NET_IPV4_NF_CONNTRACK_TCP_BE_LIBERAL=18, */
/*  NET_IPV4_NF_CONNTRACK_TCP_MAX_RETRANS=19, */
/*   NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_CLOSED=20, */
/*   NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_COOKIE_WAIT=21, */
/*   NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_COOKIE_ECHOED=22, */
/*   NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_ESTABLISHED=23, */
/*   NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_SHUTDOWN_SENT=24, */
/*   NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_SHUTDOWN_RECD=25, */
/*   NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_SHUTDOWN_ACK_SENT=26, */
/*  NET_IPV4_NF_CONNTRACK_COUNT=27, */
/*  NET_IPV4_NF_CONNTRACK_CHECKSUM=28, */
/* }; */


/* enum { */
/*  NET_IPV6_CONF=16, */
/*  NET_IPV6_NEIGH=17, */
/*  NET_IPV6_ROUTE=18, */
/*  NET_IPV6_ICMP=19, */
/*  NET_IPV6_BINDV6ONLY=20, */
/*  NET_IPV6_IP6FRAG_HIGH_THRESH=21, */
/*  NET_IPV6_IP6FRAG_LOW_THRESH=22, */
/*  NET_IPV6_IP6FRAG_TIME=23, */
/*  NET_IPV6_IP6FRAG_SECRET_INTERVAL=24, */
/*  NET_IPV6_MLD_MAX_MSF=25, */
/* }; */

/* enum { */
/*  NET_IPV6_ROUTE_FLUSH=1, */
/*  NET_IPV6_ROUTE_GC_THRESH=2, */
/*  NET_IPV6_ROUTE_MAX_SIZE=3, */
/*  NET_IPV6_ROUTE_GC_MIN_INTERVAL=4, */
/*  NET_IPV6_ROUTE_GC_TIMEOUT=5, */
/*  NET_IPV6_ROUTE_GC_INTERVAL=6, */
/*  NET_IPV6_ROUTE_GC_ELASTICITY=7, */
/*  NET_IPV6_ROUTE_MTU_EXPIRES=8, */
/*  NET_IPV6_ROUTE_MIN_ADVMSS=9, */
/*  NET_IPV6_ROUTE_GC_MIN_INTERVAL_MS=10 */
/* }; */

/* enum { */
/*  NET_IPV6_FORWARDING=1, */
/*  NET_IPV6_HOP_LIMIT=2, */
/*  NET_IPV6_MTU=3, */
/*  NET_IPV6_ACCEPT_RA=4, */
/*  NET_IPV6_ACCEPT_REDIRECTS=5, */
/*  NET_IPV6_AUTOCONF=6, */
/*  NET_IPV6_DAD_TRANSMITS=7, */
/*  NET_IPV6_RTR_SOLICITS=8, */
/*  NET_IPV6_RTR_SOLICIT_INTERVAL=9, */
/*  NET_IPV6_RTR_SOLICIT_DELAY=10, */
/*  NET_IPV6_USE_TEMPADDR=11, */
/*  NET_IPV6_TEMP_VALID_LFT=12, */
/*  NET_IPV6_TEMP_PREFERED_LFT=13, */
/*  NET_IPV6_REGEN_MAX_RETRY=14, */
/*  NET_IPV6_MAX_DESYNC_FACTOR=15, */
/*  NET_IPV6_MAX_ADDRESSES=16, */
/*  NET_IPV6_FORCE_MLD_VERSION=17, */
/*  NET_IPV6_ACCEPT_RA_DEFRTR=18, */
/*  NET_IPV6_ACCEPT_RA_PINFO=19, */
/*  NET_IPV6_ACCEPT_RA_RTR_PREF=20, */
/*  NET_IPV6_RTR_PROBE_INTERVAL=21, */
/*  NET_IPV6_ACCEPT_RA_RT_INFO_MAX_PLEN=22, */
/*  __NET_IPV6_MAX */
/* }; */


/* enum { */
/*  NET_IPV6_ICMP_RATELIMIT=1 */
/* }; */


/* enum { */
/*  NET_NEIGH_MCAST_SOLICIT=1, */
/*  NET_NEIGH_UCAST_SOLICIT=2, */
/*  NET_NEIGH_APP_SOLICIT=3, */
/*  NET_NEIGH_RETRANS_TIME=4, */
/*  NET_NEIGH_REACHABLE_TIME=5, */
/*  NET_NEIGH_DELAY_PROBE_TIME=6, */
/*  NET_NEIGH_GC_STALE_TIME=7, */
/*  NET_NEIGH_UNRES_QLEN=8, */
/*  NET_NEIGH_PROXY_QLEN=9, */
/*  NET_NEIGH_ANYCAST_DELAY=10, */
/*  NET_NEIGH_PROXY_DELAY=11, */
/*  NET_NEIGH_LOCKTIME=12, */
/*  NET_NEIGH_GC_INTERVAL=13, */
/*  NET_NEIGH_GC_THRESH1=14, */
/*  NET_NEIGH_GC_THRESH2=15, */
/*  NET_NEIGH_GC_THRESH3=16, */
/*  NET_NEIGH_RETRANS_TIME_MS=17, */
/*  NET_NEIGH_REACHABLE_TIME_MS=18, */
/*  __NET_NEIGH_MAX */
/* }; */


/* enum { */
/*  NET_DCCP_DEFAULT=1, */
/* }; */


/* enum { */
/*  NET_DCCP_DEFAULT_SEQ_WINDOW = 1, */
/*  NET_DCCP_DEFAULT_RX_CCID = 2, */
/*  NET_DCCP_DEFAULT_TX_CCID = 3, */
/*  NET_DCCP_DEFAULT_ACK_RATIO = 4, */
/*  NET_DCCP_DEFAULT_SEND_ACKVEC = 5, */
/*  NET_DCCP_DEFAULT_SEND_NDP = 6, */
/* }; */


/* enum { */
/*  NET_IPX_PPROP_BROADCASTING=1, */
/*  NET_IPX_FORWARDING=2 */
/* }; */


/* enum { */
/*  NET_LLC2=1, */
/*  NET_LLC_STATION=2, */
/* }; */


/* enum { */
/*  NET_LLC2_TIMEOUT=1, */
/* }; */


/* enum { */
/*  NET_LLC_STATION_ACK_TIMEOUT=1, */
/* }; */


/* enum { */
/*  NET_LLC2_ACK_TIMEOUT=1, */
/*  NET_LLC2_P_TIMEOUT=2, */
/*  NET_LLC2_REJ_TIMEOUT=3, */
/*  NET_LLC2_BUSY_TIMEOUT=4, */
/* }; */


/* enum { */
/*  NET_ATALK_AARP_EXPIRY_TIME=1, */
/*  NET_ATALK_AARP_TICK_TIME=2, */
/*  NET_ATALK_AARP_RETRANSMIT_LIMIT=3, */
/*  NET_ATALK_AARP_RESOLVE_TIME=4 */
/* }; */



/* enum { */
/*  NET_NETROM_DEFAULT_PATH_QUALITY=1, */
/*  NET_NETROM_OBSOLESCENCE_COUNT_INITIALISER=2, */
/*  NET_NETROM_NETWORK_TTL_INITIALISER=3, */
/*  NET_NETROM_TRANSPORT_TIMEOUT=4, */
/*  NET_NETROM_TRANSPORT_MAXIMUM_TRIES=5, */
/*  NET_NETROM_TRANSPORT_ACKNOWLEDGE_DELAY=6, */
/*  NET_NETROM_TRANSPORT_BUSY_DELAY=7, */
/*  NET_NETROM_TRANSPORT_REQUESTED_WINDOW_SIZE=8, */
/*  NET_NETROM_TRANSPORT_NO_ACTIVITY_TIMEOUT=9, */
/*  NET_NETROM_ROUTING_CONTROL=10, */
/*  NET_NETROM_LINK_FAILS_COUNT=11, */
/*  NET_NETROM_RESET=12 */
/* }; */


/* enum { */
/*  NET_AX25_IP_DEFAULT_MODE=1, */
/*  NET_AX25_DEFAULT_MODE=2, */
/*  NET_AX25_BACKOFF_TYPE=3, */
/*  NET_AX25_CONNECT_MODE=4, */
/*  NET_AX25_STANDARD_WINDOW=5, */
/*  NET_AX25_EXTENDED_WINDOW=6, */
/*  NET_AX25_T1_TIMEOUT=7, */
/*  NET_AX25_T2_TIMEOUT=8, */
/*  NET_AX25_T3_TIMEOUT=9, */
/*  NET_AX25_IDLE_TIMEOUT=10, */
/*  NET_AX25_N2=11, */
/*  NET_AX25_PACLEN=12, */
/*  NET_AX25_PROTOCOL=13, */
/*  NET_AX25_DAMA_SLAVE_TIMEOUT=14 */
/* }; */


/* enum { */
/*  NET_ROSE_RESTART_REQUEST_TIMEOUT=1, */
/*  NET_ROSE_CALL_REQUEST_TIMEOUT=2, */
/*  NET_ROSE_RESET_REQUEST_TIMEOUT=3, */
/*  NET_ROSE_CLEAR_REQUEST_TIMEOUT=4, */
/*  NET_ROSE_ACK_HOLD_BACK_TIMEOUT=5, */
/*  NET_ROSE_ROUTING_CONTROL=6, */
/*  NET_ROSE_LINK_FAIL_TIMEOUT=7, */
/*  NET_ROSE_MAX_VCS=8, */
/*  NET_ROSE_WINDOW_SIZE=9, */
/*  NET_ROSE_NO_ACTIVITY_TIMEOUT=10 */
/* }; */


/* enum { */
/*  NET_X25_RESTART_REQUEST_TIMEOUT=1, */
/*  NET_X25_CALL_REQUEST_TIMEOUT=2, */
/*  NET_X25_RESET_REQUEST_TIMEOUT=3, */
/*  NET_X25_CLEAR_REQUEST_TIMEOUT=4, */
/*  NET_X25_ACK_HOLD_BACK_TIMEOUT=5 */
/* }; */


/* enum */
/* { */
/*  NET_TR_RIF_TIMEOUT=1 */
/* }; */


/* enum { */
/*  NET_DECNET_NODE_TYPE = 1, */
/*  NET_DECNET_NODE_ADDRESS = 2, */
/*  NET_DECNET_NODE_NAME = 3, */
/*  NET_DECNET_DEFAULT_DEVICE = 4, */
/*  NET_DECNET_TIME_WAIT = 5, */
/*  NET_DECNET_DN_COUNT = 6, */
/*  NET_DECNET_DI_COUNT = 7, */
/*  NET_DECNET_DR_COUNT = 8, */
/*  NET_DECNET_DST_GC_INTERVAL = 9, */
/*  NET_DECNET_CONF = 10, */
/*  NET_DECNET_NO_FC_MAX_CWND = 11, */
/*  NET_DECNET_MEM = 12, */
/*  NET_DECNET_RMEM = 13, */
/*  NET_DECNET_WMEM = 14, */
/*  NET_DECNET_DEBUG_LEVEL = 255 */
/* }; */


/* enum { */
/*  NET_DECNET_CONF_LOOPBACK = -2, */
/*  NET_DECNET_CONF_DDCMP = -3, */
/*  NET_DECNET_CONF_PPP = -4, */
/*  NET_DECNET_CONF_X25 = -5, */
/*  NET_DECNET_CONF_GRE = -6, */
/*  NET_DECNET_CONF_ETHER = -7 */


/* }; */


/* enum { */
/*  NET_DECNET_CONF_DEV_PRIORITY = 1, */
/*  NET_DECNET_CONF_DEV_T1 = 2, */
/*  NET_DECNET_CONF_DEV_T2 = 3, */
/*  NET_DECNET_CONF_DEV_T3 = 4, */
/*  NET_DECNET_CONF_DEV_FORWARDING = 5, */
/*  NET_DECNET_CONF_DEV_BLKSIZE = 6, */
/*  NET_DECNET_CONF_DEV_STATE = 7 */
/* }; */


/* enum { */
/*  NET_SCTP_RTO_INITIAL = 1, */
/*  NET_SCTP_RTO_MIN = 2, */
/*  NET_SCTP_RTO_MAX = 3, */
/*  NET_SCTP_RTO_ALPHA = 4, */
/*  NET_SCTP_RTO_BETA = 5, */
/*  NET_SCTP_VALID_COOKIE_LIFE = 6, */
/*  NET_SCTP_ASSOCIATION_MAX_RETRANS = 7, */
/*  NET_SCTP_PATH_MAX_RETRANS = 8, */
/*  NET_SCTP_MAX_INIT_RETRANSMITS = 9, */
/*  NET_SCTP_HB_INTERVAL = 10, */
/*  NET_SCTP_PRESERVE_ENABLE = 11, */
/*  NET_SCTP_MAX_BURST = 12, */
/*  NET_SCTP_ADDIP_ENABLE = 13, */
/*  NET_SCTP_PRSCTP_ENABLE = 14, */
/*  NET_SCTP_SNDBUF_POLICY = 15, */
/*  NET_SCTP_SACK_TIMEOUT = 16, */
/*  NET_SCTP_RCVBUF_POLICY = 17, */
/* }; */


/* enum { */
/*  NET_BRIDGE_NF_CALL_ARPTABLES = 1, */
/*  NET_BRIDGE_NF_CALL_IPTABLES = 2, */
/*  NET_BRIDGE_NF_CALL_IP6TABLES = 3, */
/*  NET_BRIDGE_NF_FILTER_VLAN_TAGGED = 4, */
/* }; */


/* enum */
/* { */
/*  FS_NRINODE=1, */
/*  FS_STATINODE=2, */
/*  FS_MAXINODE=3, */
/*  FS_NRDQUOT=4, */
/*  FS_MAXDQUOT=5, */
/*  FS_NRFILE=6, */
/*  FS_MAXFILE=7, */
/*  FS_DENTRY=8, */
/*  FS_NRSUPER=9, */
/*  FS_MAXSUPER=10, */
/*  FS_OVERFLOWUID=11, */
/*  FS_OVERFLOWGID=12, */
/*  FS_LEASES=13, */
/*  FS_DIR_NOTIFY=14, */
/*  FS_LEASE_TIME=15, */
/*  FS_DQSTATS=16, */
/*  FS_XFS=17, */
/*  FS_AIO_NR=18, */
/*  FS_AIO_MAX_NR=19, */
/*  FS_INOTIFY=20, */
/* }; */


/* enum { */
/*  FS_DQ_LOOKUPS = 1, */
/*  FS_DQ_DROPS = 2, */
/*  FS_DQ_READS = 3, */
/*  FS_DQ_WRITES = 4, */
/*  FS_DQ_CACHE_HITS = 5, */
/*  FS_DQ_ALLOCATED = 6, */
/*  FS_DQ_FREE = 7, */
/*  FS_DQ_SYNCS = 8, */
/*  FS_DQ_WARNINGS = 9, */
/* }; */




/* enum { */
/*  DEV_CDROM=1, */
/*  DEV_HWMON=2, */
/*  DEV_PARPORT=3, */
/*  DEV_RAID=4, */
/*  DEV_MAC_HID=5, */
/*  DEV_SCSI=6, */
/*  DEV_IPMI=7, */
/* }; */


/* enum { */
/*  DEV_CDROM_INFO=1, */
/*  DEV_CDROM_AUTOCLOSE=2, */
/*  DEV_CDROM_AUTOEJECT=3, */
/*  DEV_CDROM_DEBUG=4, */
/*  DEV_CDROM_LOCK=5, */
/*  DEV_CDROM_CHECK_MEDIA=6 */
/* }; */


/* enum { */
/*  DEV_PARPORT_DEFAULT=-3 */
/* }; */


/* enum { */
/*  DEV_RAID_SPEED_LIMIT_MIN=1, */
/*  DEV_RAID_SPEED_LIMIT_MAX=2 */
/* }; */


/* enum { */
/*  DEV_PARPORT_DEFAULT_TIMESLICE=1, */
/*  DEV_PARPORT_DEFAULT_SPINTIME=2 */
/* }; */


/* enum { */
/*  DEV_PARPORT_SPINTIME=1, */
/*  DEV_PARPORT_BASE_ADDR=2, */
/*  DEV_PARPORT_IRQ=3, */
/*  DEV_PARPORT_DMA=4, */
/*  DEV_PARPORT_MODES=5, */
/*  DEV_PARPORT_DEVICES=6, */
/*  DEV_PARPORT_AUTOPROBE=16 */
/* }; */


/* enum { */
/*  DEV_PARPORT_DEVICES_ACTIVE=-3, */
/* }; */


/* enum { */
/*  DEV_PARPORT_DEVICE_TIMESLICE=1, */
/* }; */


/* enum { */
/*  DEV_MAC_HID_KEYBOARD_SENDS_LINUX_KEYCODES=1, */
/*  DEV_MAC_HID_KEYBOARD_LOCK_KEYCODES=2, */
/*  DEV_MAC_HID_MOUSE_BUTTON_EMULATION=3, */
/*  DEV_MAC_HID_MOUSE_BUTTON2_KEYCODE=4, */
/*  DEV_MAC_HID_MOUSE_BUTTON3_KEYCODE=5, */
/*  DEV_MAC_HID_ADB_MOUSE_SENDS_KEYCODES=6 */
/* }; */


/* enum { */
/*  DEV_SCSI_LOGGING_LEVEL=1, */
/* }; */


/* enum { */
/*  DEV_IPMI_POWEROFF_POWERCYCLE=1, */
/* }; */


/* enum */
/* { */
/*  ABI_DEFHANDLER_COFF=1, */
/*  ABI_DEFHANDLER_ELF=2, */
/*  ABI_DEFHANDLER_LCALL7=3, */
/*  ABI_DEFHANDLER_LIBCSO=4, */
/*  ABI_TRACE=5, */
/*  ABI_FAKE_UTSNAME=6, */
/* }; */




/* extern void sysctl_init(void); */

/* typedef struct ctl_table ctl_table; */

/* typedef int ctl_handler (ctl_table *table, int *name, int nlen, */
/*     void *oldval, size_t *oldlenp, */
/*     void *newval, size_t newlen, */
/*     void **context); */

/* typedef int proc_handler (ctl_table *ctl, int write, struct file * filp, */
/*      void *buffer, size_t *lenp, loff_t *ppos); */

/* extern int proc_dostring(ctl_table *, int, struct file *, */
/*     void *, size_t *, loff_t *); */
/* extern int proc_dointvec(ctl_table *, int, struct file *, */
/*     void *, size_t *, loff_t *); */
/* extern int proc_dointvec_bset(ctl_table *, int, struct file *, */
/*          void *, size_t *, loff_t *); */
/* extern int proc_dointvec_minmax(ctl_table *, int, struct file *, */
/*     void *, size_t *, loff_t *); */
/* extern int proc_dointvec_jiffies(ctl_table *, int, struct file *, */
/*      void *, size_t *, loff_t *); */
/* extern int proc_dointvec_userhz_jiffies(ctl_table *, int, struct file *, */
/*      void *, size_t *, loff_t *); */
/* extern int proc_dointvec_ms_jiffies(ctl_table *, int, struct file *, */
/*         void *, size_t *, loff_t *); */
/* extern int proc_doulongvec_minmax(ctl_table *, int, struct file *, */
/*       void *, size_t *, loff_t *); */
/* extern int proc_doulongvec_ms_jiffies_minmax(ctl_table *table, int, */
/*           struct file *, void *, size_t *, loff_t *); */

/* extern int do_sysctl (int *name, int nlen, */
/*         void *oldval, size_t *oldlenp, */
/*         void *newval, size_t newlen); */

/* extern int do_sysctl_strategy (ctl_table *table, */
/*           int *name, int nlen, */
/*           void *oldval, size_t *oldlenp, */
/*           void *newval, size_t newlen, void ** context); */

/* extern ctl_handler sysctl_string; */
/* extern ctl_handler sysctl_intvec; */
/* extern ctl_handler sysctl_jiffies; */
/* extern ctl_handler sysctl_ms_jiffies; */

/* struct ctl_table */
/* { */
/*  int ctl_name; */
/*  const char *procname; */
/*  void *data; */
/*  int maxlen; */
/*  mode_t mode; */
/*  ctl_table *child; */
/*  proc_handler *proc_handler; */
/*  ctl_handler *strategy; */
/*  struct proc_dir_entry *de; */
/*  void *extra1; */
/*  void *extra2; */
/* }; */



/* struct ctl_table_header */
/* { */
/*  ctl_table *ctl_table; */
/*  struct list_head ctl_entry; */
/*  int used; */
/*  struct completion *unregistering; */
/* }; */

/* struct ctl_table_header * register_sysctl_table(ctl_table * table, */
/*       int insert_at_head); */
/* void unregister_sysctl_table(struct ctl_table_header * table); */



/* enum { */
/*  PROC_ROOT_INO = 1, */
/* }; */

/* typedef int (read_proc_t)(char *page, char **start, off_t off, */
/*      int count, int *eof, void *data); */
/* typedef int (write_proc_t)(struct file *file, const char *buffer, */
/*       unsigned long count, void *data); */
/* typedef int (get_info_t)(char *, char **, off_t, int); */

/* struct proc_dir_entry { */
/*  unsigned int low_ino; */
/*  unsigned short namelen; */
/*  const char *name; */
/*  mode_t mode; */
/*  nlink_t nlink; */
/*  uid_t uid; */
/*  gid_t gid; */
/*  loff_t size; */
/*  struct inode_operations * proc_iops; */
/*  const struct file_operations * proc_fops; */
/*  get_info_t *get_info; */
/*  struct module *owner; */
/*  struct proc_dir_entry *next, *parent, *subdir; */
/*  void *data; */
/*  read_proc_t *read_proc; */
/*  write_proc_t *write_proc; */
/*  atomic_t count; */
/*  int deleted; */
/*  void *set; */
/* }; */

/* struct kcore_list { */
/*  struct kcore_list *next; */
/*  unsigned long addr; */
/*  size_t size; */
/* }; */

/* struct vmcore { */
/*  struct list_head list; */
/*  unsigned long long paddr; */
/*  unsigned long long size; */
/*  loff_t offset; */
/* }; */



/* extern struct proc_dir_entry proc_root; */
/* extern struct proc_dir_entry *proc_root_fs; */
/* extern struct proc_dir_entry *proc_net; */
/* extern struct proc_dir_entry *proc_net_stat; */
/* extern struct proc_dir_entry *proc_bus; */
/* extern struct proc_dir_entry *proc_root_driver; */
/* extern struct proc_dir_entry *proc_root_kcore; */

/* extern spinlock_t proc_subdir_lock; */

/* extern void proc_root_init(void); */
/* extern void proc_misc_init(void); */

/* struct mm_struct; */

/* void proc_flush_task(struct task_struct *task); */
/* struct dentry *proc_pid_lookup(struct inode *dir, struct dentry * dentry, struct nameidata *); */
/* int proc_pid_readdir(struct file * filp, void * dirent, filldir_t filldir); */
/* unsigned long task_vsize(struct mm_struct *); */
/* int task_statm(struct mm_struct *, int *, int *, int *, int *); */
/* char *task_mem(struct mm_struct *, char *); */

/* extern struct proc_dir_entry *create_proc_entry(const char *name, mode_t mode, */
/*       struct proc_dir_entry *parent); */
/* extern void remove_proc_entry(const char *name, struct proc_dir_entry *parent); */

/* extern struct vfsmount *proc_mnt; */
/* extern int proc_fill_super(struct super_block *,void *,int); */
/* extern struct inode *proc_get_inode(struct super_block *, unsigned int, struct proc_dir_entry *); */

/* extern int proc_match(int, const char *,struct proc_dir_entry *); */

/* extern int proc_readdir(struct file *, void *, filldir_t); */
/* extern struct dentry *proc_lookup(struct inode *, struct dentry *, struct nameidata *); */

/* extern const struct file_operations proc_kcore_operations; */
/* extern const struct file_operations proc_kmsg_operations; */
/* extern const struct file_operations ppc_htab_operations; */




/* struct tty_driver; */
/* extern void proc_tty_init(void); */
/* extern void proc_tty_register_driver(struct tty_driver *driver); */
/* extern void proc_tty_unregister_driver(struct tty_driver *driver); */

/* extern struct proc_dir_entry *proc_symlink(const char *, */
/*   struct proc_dir_entry *, const char *); */
/* extern struct proc_dir_entry *proc_mkdir(const char *,struct proc_dir_entry *); */
/* extern struct proc_dir_entry *proc_mkdir_mode(const char *name, mode_t mode, */
/*    struct proc_dir_entry *parent); */

/* static inline __attribute__((always_inline)) struct proc_dir_entry *create_proc_read_entry(const char *name, */
/*  mode_t mode, struct proc_dir_entry *base, */
/*  read_proc_t *read_proc, void * data) */
/* { */
/*  struct proc_dir_entry *res=create_proc_entry(name,mode,base); */
/*  if (res) { */
/*   res->read_proc=read_proc; */
/*   res->data=data; */
/*  } */
/*  return res; */
/* } */

/* static inline __attribute__((always_inline)) struct proc_dir_entry *create_proc_info_entry(const char *name, */
/*  mode_t mode, struct proc_dir_entry *base, get_info_t *get_info) */
/* { */
/*  struct proc_dir_entry *res=create_proc_entry(name,mode,base); */
/*  if (res) res->get_info=get_info; */
/*  return res; */
/* } */

/* static inline __attribute__((always_inline)) struct proc_dir_entry *proc_net_create(const char *name, */
/*  mode_t mode, get_info_t *get_info) */
/* { */
/*  return create_proc_info_entry(name,mode,proc_net,get_info); */
/* } */

/* static inline __attribute__((always_inline)) struct proc_dir_entry *proc_net_fops_create(const char *name, */
/*  mode_t mode, const struct file_operations *fops) */
/* { */
/*  struct proc_dir_entry *res = create_proc_entry(name, mode, proc_net); */
/*  if (res) */
/*   res->proc_fops = fops; */
/*  return res; */
/* } */

/* static inline __attribute__((always_inline)) void proc_net_remove(const char *name) */
/* { */
/*  remove_proc_entry(name,proc_net); */
/* } */

/* extern void kclist_add(struct kcore_list *, void *, size_t); */


/* struct proc_inode { */
/*  struct pid *pid; */
/*  int fd; */
/*  union { */
/*   int (*proc_get_link)(struct inode *, struct dentry **, struct vfsmount **); */
/*   int (*proc_read)(struct task_struct *task, char *page); */
/*  } op; */
/*  struct proc_dir_entry *pde; */
/*  struct inode vfs_inode; */
/* }; */

/* static inline __attribute__((always_inline)) struct proc_inode *PROC_I(const struct inode *inode) */
/* { */
/*  return ({ const typeof( ((struct proc_inode *)0)->vfs_inode ) *__mptr = (inode); (struct proc_inode *)( (char *)__mptr - __builtin_offsetof(struct proc_inode,vfs_inode) );}); */
/* } */

/* static inline __attribute__((always_inline)) struct proc_dir_entry *PDE(const struct inode *inode) */
/* { */
/*  return PROC_I(inode)->pde; */
/* } */

/* struct proc_maps_private { */
/*  struct pid *pid; */
/*  struct task_struct *task; */
/*  struct vm_area_struct *tail_vma; */
/* }; */



/* struct blkpg_ioctl_arg { */
/*         int op; */
/*         int flags; */
/*         int datalen; */
/*         void *data; */
/* }; */

/* struct blkpg_partition { */
/*  long long start; */
/*  long long length; */
/*  int pno; */
/*  char devname[64]; */

/*  char volname[64]; */
/* }; */










/* enum { */


/*  DOS_EXTENDED_PARTITION = 5, */
/*  LINUX_EXTENDED_PARTITION = 0x85, */
/*  WIN98_EXTENDED_PARTITION = 0x0f, */

/*  LINUX_SWAP_PARTITION = 0x82, */
/*  LINUX_RAID_PARTITION = 0xfd, */

/*  SOLARIS_X86_PARTITION = LINUX_SWAP_PARTITION, */
/*  NEW_SOLARIS_X86_PARTITION = 0xbf, */

/*  DM6_AUX1PARTITION = 0x51, */
/*  DM6_AUX3PARTITION = 0x53, */
/*  DM6_PARTITION = 0x54, */
/*  EZD_PARTITION = 0x55, */

/*  FREEBSD_PARTITION = 0xa5, */
/*  OPENBSD_PARTITION = 0xa6, */
/*  NETBSD_PARTITION = 0xa9, */
/*  BSDI_PARTITION = 0xb7, */
/*  MINIX_PARTITION = 0x81, */
/*  UNIXWARE_PARTITION = 0x63, */
/* }; */

/* struct partition { */
/*  unsigned char boot_ind; */
/*  unsigned char head; */
/*  unsigned char sector; */
/*  unsigned char cyl; */
/*  unsigned char sys_ind; */
/*  unsigned char end_head; */
/*  unsigned char end_sector; */
/*  unsigned char end_cyl; */
/*  __le32 start_sect; */
/*  __le32 nr_sects; */
/* } __attribute__((packed)); */

/* struct hd_struct { */
/*  sector_t start_sect; */
/*  sector_t nr_sects; */
/*  struct kobject kobj; */
/*  struct kobject *holder_dir; */
/*  unsigned ios[2], sectors[2]; */
/*  int policy, partno; */
/* }; */







/* struct disk_stats { */
/*  unsigned long sectors[2]; */
/*  unsigned long ios[2]; */
/*  unsigned long merges[2]; */
/*  unsigned long ticks[2]; */
/*  unsigned long io_ticks; */
/*  unsigned long time_in_queue; */
/* }; */

/* struct gendisk { */
/*  int major; */
/*  int first_minor; */
/*  int minors; */

/*  char disk_name[32]; */
/*  struct hd_struct **part; */
/*  int part_uevent_suppress; */
/*  struct block_device_operations *fops; */
/*  struct request_queue *queue; */
/*  void *private_data; */
/*  sector_t capacity; */

/*  int flags; */
/*  struct device *driverfs_dev; */
/*  struct kobject kobj; */
/*  struct kobject *holder_dir; */
/*  struct kobject *slave_dir; */

/*  struct timer_rand_state *random; */
/*  int policy; */

/*  atomic_t sync_io; */
/*  unsigned long stamp; */
/*  int in_flight; */



/*  struct disk_stats dkstats; */

/* }; */


/* struct disk_attribute { */
/*  struct attribute attr; */
/*  ssize_t (*show)(struct gendisk *, char *); */
/*  ssize_t (*store)(struct gendisk *, const char *, size_t); */
/* }; */

/* static inline __attribute__((always_inline)) void disk_stat_set_all(struct gendisk *gendiskp, int value) { */
/*  (__builtin_constant_p(value) ? (__builtin_constant_p((sizeof (struct disk_stats))) ? __constant_c_and_count_memset(((&gendiskp->dkstats)),((0x01010101UL*(unsigned char)(value))),((sizeof (struct disk_stats)))) : __constant_c_memset(((&gendiskp->dkstats)),((0x01010101UL*(unsigned char)(value))),((sizeof (struct disk_stats))))) : (__builtin_constant_p((sizeof (struct disk_stats))) ? __memset_generic((((&gendiskp->dkstats))),(((value))),(((sizeof (struct disk_stats))))) : __memset_generic(((&gendiskp->dkstats)),((value)),((sizeof (struct disk_stats)))))); */
/* } */

/* static inline __attribute__((always_inline)) int init_disk_stats(struct gendisk *disk) */
/* { */
/*  return 1; */
/* } */

/* static inline __attribute__((always_inline)) void free_disk_stats(struct gendisk *disk) */
/* { */
/* } */



/* extern void disk_round_stats(struct gendisk *disk); */


/* extern int get_blkdev_list(char *, int); */
/* extern void add_disk(struct gendisk *disk); */
/* extern void del_gendisk(struct gendisk *gp); */
/* extern void unlink_gendisk(struct gendisk *gp); */
/* extern struct gendisk *get_gendisk(dev_t dev, int *part); */

/* extern void set_device_ro(struct block_device *bdev, int flag); */
/* extern void set_disk_ro(struct gendisk *disk, int flag); */


/* extern void add_disk_randomness(struct gendisk *disk); */
/* extern void rand_initialize_disk(struct gendisk *disk); */

/* static inline __attribute__((always_inline)) sector_t get_start_sect(struct block_device *bdev) */
/* { */
/*  return bdev->bd_contains == bdev ? 0 : bdev->bd_part->start_sect; */
/* } */
/* static inline __attribute__((always_inline)) sector_t get_capacity(struct gendisk *disk) */
/* { */
/*  return disk->capacity; */
/* } */
/* static inline __attribute__((always_inline)) void set_capacity(struct gendisk *disk, sector_t size) */
/* { */
/*  disk->capacity = size; */
/* } */

/* struct solaris_x86_slice { */
/*  __le16 s_tag; */
/*  __le16 s_flag; */
/*  __le32 s_start; */
/*  __le32 s_size; */
/* }; */

/* struct solaris_x86_vtoc { */
/*  unsigned int v_bootinfo[3]; */
/*  __le32 v_sanity; */
/*  __le32 v_version; */
/*  char v_volume[8]; */
/*  __le16 v_sectorsz; */
/*  __le16 v_nparts; */
/*  unsigned int v_reserved[10]; */
/*  struct solaris_x86_slice */
/*   v_slice[8]; */
/*  unsigned int timestamp[8]; */
/*  char v_asciilabel[128]; */
/* }; */

/* struct bsd_disklabel { */
/*  __le32 d_magic; */
/*  __s16 d_type; */
/*  __s16 d_subtype; */
/*  char d_typename[16]; */
/*  char d_packname[16]; */
/*  __u32 d_secsize; */
/*  __u32 d_nsectors; */
/*  __u32 d_ntracks; */
/*  __u32 d_ncylinders; */
/*  __u32 d_secpercyl; */
/*  __u32 d_secperunit; */
/*  __u16 d_sparespertrack; */
/*  __u16 d_sparespercyl; */
/*  __u32 d_acylinders; */
/*  __u16 d_rpm; */
/*  __u16 d_interleave; */
/*  __u16 d_trackskew; */
/*  __u16 d_cylskew; */
/*  __u32 d_headswitch; */
/*  __u32 d_trkseek; */
/*  __u32 d_flags; */

/*  __u32 d_drivedata[5]; */

/*  __u32 d_spare[5]; */
/*  __le32 d_magic2; */
/*  __le16 d_checksum; */


/*  __le16 d_npartitions; */
/*  __le32 d_bbsize; */
/*  __le32 d_sbsize; */
/*  struct bsd_partition { */
/*   __le32 p_size; */
/*   __le32 p_offset; */
/*   __le32 p_fsize; */
/*   __u8 p_fstype; */
/*   __u8 p_frag; */
/*   __le16 p_cpg; */
/*  } d_partitions[16]; */
/* }; */

/* struct unixware_slice { */
/*  __le16 s_label; */
/*  __le16 s_flags; */
/*  __le32 start_sect; */
/*  __le32 nr_sects; */
/* }; */

/* struct unixware_disklabel { */
/*  __le32 d_type; */
/*  __le32 d_magic; */
/*  __le32 d_version; */
/*  char d_serial[12]; */
/*  __le32 d_ncylinders; */
/*  __le32 d_ntracks; */
/*  __le32 d_nsectors; */
/*  __le32 d_secsize; */
/*  __le32 d_part_start; */
/*  __le32 d_unknown1[12]; */
/*   __le32 d_alt_tbl; */
/*   __le32 d_alt_len; */
/*   __le32 d_phys_cyl; */
/*   __le32 d_phys_trk; */
/*   __le32 d_phys_sec; */
/*   __le32 d_phys_bytes; */
/*   __le32 d_unknown2; */
/*  __le32 d_unknown3; */
/*  __le32 d_pad[8]; */

/*  struct unixware_vtoc { */
/*   __le32 v_magic; */
/*   __le32 v_version; */
/*   char v_name[8]; */
/*   __le16 v_nslices; */
/*   __le16 v_unknown1; */
/*   __le32 v_reserved[10]; */
/*   struct unixware_slice */
/*    v_slice[16]; */
/*  } vtoc; */

/* }; */

/* char *disk_name (struct gendisk *hd, int part, char *buf); */

/* extern int rescan_partitions(struct gendisk *disk, struct block_device *bdev); */
/* extern void add_partition(struct gendisk *, int, sector_t, sector_t); */
/* extern void delete_partition(struct gendisk *, int); */

/* extern struct gendisk *alloc_disk_node(int minors, int node_id); */
/* extern struct gendisk *alloc_disk(int minors); */
/* extern struct kobject *get_disk(struct gendisk *disk); */
/* extern void put_disk(struct gendisk *disk); */

/* extern void blk_register_region(dev_t dev, unsigned long range, */
/*    struct module *module, */
/*    struct kobject *(*probe)(dev_t, int *, void *), */
/*    int (*lock)(dev_t, void *), */
/*    void *data); */
/* extern void blk_unregister_region(dev_t dev, unsigned long range); */

/* static inline __attribute__((always_inline)) struct block_device *bdget_disk(struct gendisk *disk, int index) */
/* { */
/*  return bdget((((disk->major) << 20) | (disk->first_minor)) + index); */
/* } */







/* enum bdi_state { */
/*  BDI_pdflush, */
/*  BDI_write_congested, */
/*  BDI_read_congested, */
/*  BDI_unused, */
/* }; */

/* typedef int (congested_fn)(void *, int); */

/* struct backing_dev_info { */
/*  unsigned long ra_pages; */
/*  unsigned long state; */
/*  unsigned int capabilities; */
/*  congested_fn *congested_fn; */
/*  void *congested_data; */
/*  void (*unplug_io_fn)(struct backing_dev_info *, struct page *); */
/*  void *unplug_io_data; */
/* }; */

/* extern struct backing_dev_info default_backing_dev_info; */
/* void default_unplug_io_fn(struct backing_dev_info *bdi, struct page *page); */

/* int writeback_acquire(struct backing_dev_info *bdi); */
/* int writeback_in_progress(struct backing_dev_info *bdi); */
/* void writeback_release(struct backing_dev_info *bdi); */

/* static inline __attribute__((always_inline)) int bdi_congested(struct backing_dev_info *bdi, int bdi_bits) */
/* { */
/*  if (bdi->congested_fn) */
/*   return bdi->congested_fn(bdi->congested_data, bdi_bits); */
/*  return (bdi->state & bdi_bits); */
/* } */

/* static inline __attribute__((always_inline)) int bdi_read_congested(struct backing_dev_info *bdi) */
/* { */
/*  return bdi_congested(bdi, 1 << BDI_read_congested); */
/* } */

/* static inline __attribute__((always_inline)) int bdi_write_congested(struct backing_dev_info *bdi) */
/* { */
/*  return bdi_congested(bdi, 1 << BDI_write_congested); */
/* } */

/* static inline __attribute__((always_inline)) int bdi_rw_congested(struct backing_dev_info *bdi) */
/* { */
/*  return bdi_congested(bdi, (1 << BDI_read_congested)| */
/*       (1 << BDI_write_congested)); */
/* } */




/* struct kmem_cache; */

/* typedef void * (mempool_alloc_t)(gfp_t gfp_mask, void *pool_data); */
/* typedef void (mempool_free_t)(void *element, void *pool_data); */

/* typedef struct mempool_s { */
/*  spinlock_t lock; */
/*  int min_nr; */
/*  int curr_nr; */
/*  void **elements; */

/*  void *pool_data; */
/*  mempool_alloc_t *alloc; */
/*  mempool_free_t *free; */
/*  wait_queue_head_t wait; */
/* } mempool_t; */

/* extern mempool_t *mempool_create(int min_nr, mempool_alloc_t *alloc_fn, */
/*    mempool_free_t *free_fn, void *pool_data); */
/* extern mempool_t *mempool_create_node(int min_nr, mempool_alloc_t *alloc_fn, */
/*    mempool_free_t *free_fn, void *pool_data, int nid); */

/* extern int mempool_resize(mempool_t *pool, int new_min_nr, gfp_t gfp_mask); */
/* extern void mempool_destroy(mempool_t *pool); */
/* extern void * mempool_alloc(mempool_t *pool, gfp_t gfp_mask); */
/* extern void mempool_free(void *element, mempool_t *pool); */





/* void *mempool_alloc_slab(gfp_t gfp_mask, void *pool_data); */
/* void mempool_free_slab(void *element, void *pool_data); */
/* static inline __attribute__((always_inline)) mempool_t * */
/* mempool_create_slab_pool(int min_nr, struct kmem_cache *kc) */
/* { */
/*  return mempool_create(min_nr, mempool_alloc_slab, mempool_free_slab, */
/*          (void *) kc); */
/* } */





/* void *mempool_kmalloc(gfp_t gfp_mask, void *pool_data); */
/* void *mempool_kzalloc(gfp_t gfp_mask, void *pool_data); */
/* void mempool_kfree(void *element, void *pool_data); */
/* static inline __attribute__((always_inline)) mempool_t *mempool_create_kmalloc_pool(int min_nr, size_t size) */
/* { */
/*  return mempool_create(min_nr, mempool_kmalloc, mempool_kfree, */
/*          (void *) size); */
/* } */
/* static inline __attribute__((always_inline)) mempool_t *mempool_create_kzalloc_pool(int min_nr, size_t size) */
/* { */
/*  return mempool_create(min_nr, mempool_kzalloc, mempool_kfree, */
/*          (void *) size); */
/* } */





/* void *mempool_alloc_pages(gfp_t gfp_mask, void *pool_data); */
/* void mempool_free_pages(void *element, void *pool_data); */
/* static inline __attribute__((always_inline)) mempool_t *mempool_create_page_pool(int min_nr, int order) */
/* { */
/*  return mempool_create(min_nr, mempool_alloc_pages, mempool_free_pages, */
/*          (void *)(long)order); */
/* } */





/* enum { */
/*  IOPRIO_CLASS_NONE, */
/*  IOPRIO_CLASS_RT, */
/*  IOPRIO_CLASS_BE, */
/*  IOPRIO_CLASS_IDLE, */
/* }; */






/* enum { */
/*  IOPRIO_WHO_PROCESS = 1, */
/*  IOPRIO_WHO_PGRP, */
/*  IOPRIO_WHO_USER, */
/* }; */






/* static inline __attribute__((always_inline)) int task_ioprio(struct task_struct *task) */
/* { */
/*  do { if (__builtin_expect(!!((!((((task->ioprio)) >> (13)) != IOPRIO_CLASS_NONE))!=0), 0)) { printk("BUG: warning at %s:%d/%s()\n", "include/linux/ioprio.h", 50, (__func__)); dump_stack(); } } while (0); */
/*  return ((task->ioprio) & ((1UL << (13)) - 1)); */
/* } */

/* static inline __attribute__((always_inline)) int task_nice_ioprio(struct task_struct *task) */
/* { */
/*  return (task_nice(task) + 20) / 5; */
/* } */




/* extern int ioprio_best(unsigned short aprio, unsigned short bprio); */







/* extern unsigned int __attribute__((regparm(3))) ioread8(void *); */
/* extern unsigned int __attribute__((regparm(3))) ioread16(void *); */
/* extern unsigned int __attribute__((regparm(3))) ioread16be(void *); */
/* extern unsigned int __attribute__((regparm(3))) ioread32(void *); */
/* extern unsigned int __attribute__((regparm(3))) ioread32be(void *); */

/* extern void __attribute__((regparm(3))) iowrite8(u8, void *); */
/* extern void __attribute__((regparm(3))) iowrite16(u16, void *); */
/* extern void __attribute__((regparm(3))) iowrite16be(u16, void *); */
/* extern void __attribute__((regparm(3))) iowrite32(u32, void *); */
/* extern void __attribute__((regparm(3))) iowrite32be(u32, void *); */

/* extern void __attribute__((regparm(3))) ioread8_rep(void *port, void *buf, unsigned long count); */
/* extern void __attribute__((regparm(3))) ioread16_rep(void *port, void *buf, unsigned long count); */
/* extern void __attribute__((regparm(3))) ioread32_rep(void *port, void *buf, unsigned long count); */

/* extern void __attribute__((regparm(3))) iowrite8_rep(void *port, const void *buf, unsigned long count); */
/* extern void __attribute__((regparm(3))) iowrite16_rep(void *port, const void *buf, unsigned long count); */
/* extern void __attribute__((regparm(3))) iowrite32_rep(void *port, const void *buf, unsigned long count); */


/* extern void *ioport_map(unsigned long port, unsigned int nr); */
/* extern void ioport_unmap(void *); */


/* struct pci_dev; */
/* extern void *pci_iomap(struct pci_dev *dev, int bar, unsigned long max); */
/* extern void pci_iounmap(struct pci_dev *dev, void *); */









/* struct vm_area_struct; */

/* struct vm_struct { */
/*  void *addr; */
/*  unsigned long size; */
/*  unsigned long flags; */
/*  struct page **pages; */
/*  unsigned int nr_pages; */
/*  unsigned long phys_addr; */
/*  struct vm_struct *next; */
/* }; */




/* extern void *vmalloc(unsigned long size); */
/* extern void *vmalloc_user(unsigned long size); */
/* extern void *vmalloc_node(unsigned long size, int node); */
/* extern void *vmalloc_exec(unsigned long size); */
/* extern void *vmalloc_32(unsigned long size); */
/* extern void *vmalloc_32_user(unsigned long size); */
/* extern void *__vmalloc(unsigned long size, gfp_t gfp_mask, pgprot_t prot); */
/* extern void *__vmalloc_area(struct vm_struct *area, gfp_t gfp_mask, */
/*     pgprot_t prot); */
/* extern void *__vmalloc_node(unsigned long size, gfp_t gfp_mask, */
/*     pgprot_t prot, int node); */
/* extern void vfree(void *addr); */

/* extern void *vmap(struct page **pages, unsigned int count, */
/*    unsigned long flags, pgprot_t prot); */
/* extern void vunmap(void *addr); */

/* extern int remap_vmalloc_range(struct vm_area_struct *vma, void *addr, */
/*        unsigned long pgoff); */




/* extern struct vm_struct *get_vm_area(unsigned long size, unsigned long flags); */
/* extern struct vm_struct *__get_vm_area(unsigned long size, unsigned long flags, */
/*      unsigned long start, unsigned long end); */
/* extern struct vm_struct *get_vm_area_node(unsigned long size, */
/*      unsigned long flags, int node); */
/* extern struct vm_struct *remove_vm_area(void *addr); */
/* extern struct vm_struct *__remove_vm_area(void *addr); */
/* extern int map_vm_area(struct vm_struct *area, pgprot_t prot, */
/*    struct page ***pages); */
/* extern void unmap_vm_area(struct vm_struct *area); */




/* extern rwlock_t vmlist_lock; */
/* extern struct vm_struct *vmlist; */


/* static inline __attribute__((always_inline)) unsigned long virt_to_phys(volatile void * address) */
/* { */
/*  return ((unsigned long)(address)-((unsigned long)((unsigned long)0xC0000000))); */
/* } */

/* static inline __attribute__((always_inline)) void * phys_to_virt(unsigned long address) */
/* { */
/*  return ((void *)((unsigned long)(address)+((unsigned long)((unsigned long)0xC0000000)))); */
/* } */






/* extern void * __ioremap(unsigned long offset, unsigned long size, unsigned long flags); */

/* static inline __attribute__((always_inline)) void * ioremap(unsigned long offset, unsigned long size) */
/* { */
/*  return __ioremap(offset, size, 0); */
/* } */

/* extern void * ioremap_nocache(unsigned long offset, unsigned long size); */
/* extern void iounmap(volatile void *addr); */






/* extern void *bt_ioremap(unsigned long offset, unsigned long size); */
/* extern void bt_iounmap(void *addr, unsigned long size); */

/* static inline __attribute__((always_inline)) unsigned char readb(const volatile void *addr) */
/* { */
/*  return *(volatile unsigned char *) addr; */
/* } */
/* static inline __attribute__((always_inline)) unsigned short readw(const volatile void *addr) */
/* { */
/*  return *(volatile unsigned short *) addr; */
/* } */
/* static inline __attribute__((always_inline)) unsigned int readl(const volatile void *addr) */
/* { */
/*  return *(volatile unsigned int *) addr; */
/* } */







/* static inline __attribute__((always_inline)) void writeb(unsigned char b, volatile void *addr) */
/* { */
/*  *(volatile unsigned char *) addr = b; */
/* } */
/* static inline __attribute__((always_inline)) void writew(unsigned short b, volatile void *addr) */
/* { */
/*  *(volatile unsigned short *) addr = b; */
/* } */
/* static inline __attribute__((always_inline)) void writel(unsigned int b, volatile void *addr) */
/* { */
/*  *(volatile unsigned int *) addr = b; */
/* } */






/* static inline __attribute__((always_inline)) void memset_io(volatile void *addr, unsigned char val, int count) */
/* { */
/*  (__builtin_constant_p(val) ? (__builtin_constant_p((count)) ? __constant_c_and_count_memset((((void *) addr)),((0x01010101UL*(unsigned char)(val))),((count))) : __constant_c_memset((((void *) addr)),((0x01010101UL*(unsigned char)(val))),((count)))) : (__builtin_constant_p((count)) ? __memset_generic(((((void *) addr))),(((val))),(((count)))) : __memset_generic((((void *) addr)),((val)),((count))))); */
/* } */
/* static inline __attribute__((always_inline)) void memcpy_fromio(void *dst, const volatile void *src, int count) */
/* { */
/*  __memcpy(dst, (void *) src, count); */
/* } */
/* static inline __attribute__((always_inline)) void memcpy_toio(volatile void *dst, const void *src, int count) */
/* { */
/*  __memcpy((void *) dst, src, count); */
/* } */

/* static inline __attribute__((always_inline)) int check_signature(volatile void * io_addr, */
/*  const unsigned char *signature, int length) */
/* { */
/*  int retval = 0; */
/*  do { */
/*   if (readb(io_addr) != *signature) */
/*    goto out; */
/*   io_addr++; */
/*   signature++; */
/*   length--; */
/*  } while (length); */
/*  retval = 1; */
/* out: */
/*  return retval; */
/* } */

/* static inline __attribute__((always_inline)) void flush_write_buffers(void) */
/* { */
/*  __asm__ __volatile__ ("lock; addl $0,0(%%esp)": : :"memory"); */
/* } */

/* static inline __attribute__((always_inline)) void slow_down_io(void) { */
/*  __asm__ __volatile__( */
/*   "outb %%al,$0x80;" */



/*   : : ); */
/* } */

/* static inline __attribute__((always_inline)) void outb_local(unsigned char value, int port) { __asm__ __volatile__("out" "b" " %" "b" "0, %w1" : : "a"(value), "Nd"(port)); } static inline __attribute__((always_inline)) unsigned char inb_local(int port) { unsigned char value; __asm__ __volatile__("in" "b" " %w1, %" "b" "0" : "=a"(value) : "Nd"(port)); return value; } static inline __attribute__((always_inline)) void outb_local_p(unsigned char value, int port) { outb_local(value, port); slow_down_io(); } static inline __attribute__((always_inline)) unsigned char inb_local_p(int port) { unsigned char value = inb_local(port); slow_down_io(); return value; } static inline __attribute__((always_inline)) void outb(unsigned char value, int port) { outb_local(value, port); } static inline __attribute__((always_inline)) unsigned char inb(int port) { return inb_local(port); } static inline __attribute__((always_inline)) void outb_p(unsigned char value, int port) { outb(value, port); slow_down_io(); } static inline __attribute__((always_inline)) unsigned char inb_p(int port) { unsigned char value = inb(port); slow_down_io(); return value; } static inline __attribute__((always_inline)) void outsb(int port, const void *addr, unsigned long count) { __asm__ __volatile__("rep; outs" "b" : "+S"(addr), "+c"(count) : "d"(port)); } static inline __attribute__((always_inline)) void insb(int port, void *addr, unsigned long count) { __asm__ __volatile__("rep; ins" "b" : "+D"(addr), "+c"(count) : "d"(port)); } */
/* static inline __attribute__((always_inline)) void outw_local(unsigned short value, int port) { __asm__ __volatile__("out" "w" " %" "w" "0, %w1" : : "a"(value), "Nd"(port)); } static inline __attribute__((always_inline)) unsigned short inw_local(int port) { unsigned short value; __asm__ __volatile__("in" "w" " %w1, %" "w" "0" : "=a"(value) : "Nd"(port)); return value; } static inline __attribute__((always_inline)) void outw_local_p(unsigned short value, int port) { outw_local(value, port); slow_down_io(); } static inline __attribute__((always_inline)) unsigned short inw_local_p(int port) { unsigned short value = inw_local(port); slow_down_io(); return value; } static inline __attribute__((always_inline)) void outw(unsigned short value, int port) { outw_local(value, port); } static inline __attribute__((always_inline)) unsigned short inw(int port) { return inw_local(port); } static inline __attribute__((always_inline)) void outw_p(unsigned short value, int port) { outw(value, port); slow_down_io(); } static inline __attribute__((always_inline)) unsigned short inw_p(int port) { unsigned short value = inw(port); slow_down_io(); return value; } static inline __attribute__((always_inline)) void outsw(int port, const void *addr, unsigned long count) { __asm__ __volatile__("rep; outs" "w" : "+S"(addr), "+c"(count) : "d"(port)); } static inline __attribute__((always_inline)) void insw(int port, void *addr, unsigned long count) { __asm__ __volatile__("rep; ins" "w" : "+D"(addr), "+c"(count) : "d"(port)); } */
/* static inline __attribute__((always_inline)) void outl_local(unsigned int value, int port) { __asm__ __volatile__("out" "l" " %" "" "0, %w1" : : "a"(value), "Nd"(port)); } static inline __attribute__((always_inline)) unsigned int inl_local(int port) { unsigned int value; __asm__ __volatile__("in" "l" " %w1, %" "" "0" : "=a"(value) : "Nd"(port)); return value; } static inline __attribute__((always_inline)) void outl_local_p(unsigned int value, int port) { outl_local(value, port); slow_down_io(); } static inline __attribute__((always_inline)) unsigned int inl_local_p(int port) { unsigned int value = inl_local(port); slow_down_io(); return value; } static inline __attribute__((always_inline)) void outl(unsigned int value, int port) { outl_local(value, port); } static inline __attribute__((always_inline)) unsigned int inl(int port) { return inl_local(port); } static inline __attribute__((always_inline)) void outl_p(unsigned int value, int port) { outl(value, port); slow_down_io(); } static inline __attribute__((always_inline)) unsigned int inl_p(int port) { unsigned int value = inl(port); slow_down_io(); return value; } static inline __attribute__((always_inline)) void outsl(int port, const void *addr, unsigned long count) { __asm__ __volatile__("rep; outs" "l" : "+S"(addr), "+c"(count) : "d"(port)); } static inline __attribute__((always_inline)) void insl(int port, void *addr, unsigned long count) { __asm__ __volatile__("rep; ins" "l" : "+D"(addr), "+c"(count) : "d"(port)); } */


/* struct bio_vec { */
/*  struct page *bv_page; */
/*  unsigned int bv_len; */
/*  unsigned int bv_offset; */
/* }; */

/* struct bio_set; */
/* struct bio; */
/* typedef int (bio_end_io_t) (struct bio *, unsigned int, int); */
/* typedef void (bio_destructor_t) (struct bio *); */





/* struct bio { */
/*  sector_t bi_sector; */
/*  struct bio *bi_next; */
/*  struct block_device *bi_bdev; */
/*  unsigned long bi_flags; */
/*  unsigned long bi_rw; */



/*  unsigned short bi_vcnt; */
/*  unsigned short bi_idx; */




/*  unsigned short bi_phys_segments; */




/*  unsigned short bi_hw_segments; */

/*  unsigned int bi_size; */






/*  unsigned int bi_hw_front_size; */
/*  unsigned int bi_hw_back_size; */

/*  unsigned int bi_max_vecs; */

/*  struct bio_vec *bi_io_vec; */

/*  bio_end_io_t *bi_end_io; */
/*  atomic_t bi_cnt; */

/*  void *bi_private; */

/*  bio_destructor_t *bi_destructor; */
/* }; */

/* struct bio_pair { */
/*  struct bio bio1, bio2; */
/*  struct bio_vec bv1, bv2; */
/*  atomic_t cnt; */
/*  int error; */
/* }; */
/* extern struct bio_pair *bio_split(struct bio *bi, mempool_t *pool, */
/*       int first_sectors); */
/* extern mempool_t *bio_split_pool; */
/* extern void bio_pair_release(struct bio_pair *dbio); */

/* extern struct bio_set *bioset_create(int, int, int); */
/* extern void bioset_free(struct bio_set *); */

/* extern struct bio *bio_alloc(gfp_t, int); */
/* extern struct bio *bio_alloc_bioset(gfp_t, int, struct bio_set *); */
/* extern void bio_put(struct bio *); */
/* extern void bio_free(struct bio *, struct bio_set *); */

/* extern void bio_endio(struct bio *, unsigned int, int); */
/* struct request_queue; */
/* extern int bio_phys_segments(struct request_queue *, struct bio *); */
/* extern int bio_hw_segments(struct request_queue *, struct bio *); */

/* extern void __bio_clone(struct bio *, struct bio *); */
/* extern struct bio *bio_clone(struct bio *, gfp_t); */

/* extern void bio_init(struct bio *); */

/* extern int bio_add_page(struct bio *, struct page *, unsigned int,unsigned int); */
/* extern int bio_add_pc_page(struct request_queue *, struct bio *, struct page *, */
/*       unsigned int, unsigned int); */
/* extern int bio_get_nr_vecs(struct block_device *); */
/* extern struct bio *bio_map_user(struct request_queue *, struct block_device *, */
/*     unsigned long, unsigned int, int); */
/* struct sg_iovec; */
/* extern struct bio *bio_map_user_iov(struct request_queue *, */
/*         struct block_device *, */
/*         struct sg_iovec *, int, int); */
/* extern void bio_unmap_user(struct bio *); */
/* extern struct bio *bio_map_kern(struct request_queue *, void *, unsigned int, */
/*     gfp_t); */
/* extern void bio_set_pages_dirty(struct bio *bio); */
/* extern void bio_check_pages_dirty(struct bio *bio); */
/* extern struct bio *bio_copy_user(struct request_queue *, unsigned long, unsigned int, int); */
/* extern int bio_uncopy_user(struct bio *); */
/* void zero_fill_bio(struct bio *bio); */

/* static inline __attribute__((always_inline)) char *bvec_kmap_irq(struct bio_vec *bvec, unsigned long *flags) */
/* { */
/*  unsigned long addr; */





/*  do { do { (*flags) = __raw_local_irq_save(); } while (0); do { } while (0); } while (0); */
/*  addr = (unsigned long) kmap_atomic(bvec->bv_page, KM_BIO_SRC_IRQ); */

/*  do { if (__builtin_expect(!!((addr & ~(~((1UL << 12)-1)))!=0), 0)) __asm__ __volatile__( "ud2\n" "\t.word %c0\n" "\t.long %c1\n" : : "i" (331), "i" ("include/linux/bio.h")); } while(0); */

/*  return (char *) addr + bvec->bv_offset; */
/* } */

/* static inline __attribute__((always_inline)) void bvec_kunmap_irq(char *buffer, unsigned long *flags) */
/* { */
/*  unsigned long ptr = (unsigned long) buffer & (~((1UL << 12)-1)); */

/*  kunmap_atomic((void *) ptr, KM_BIO_SRC_IRQ); */
/*  do { if (raw_irqs_disabled_flags(*flags)) { raw_local_irq_restore(*flags); do { } while (0); } else { do { } while (0); raw_local_irq_restore(*flags); } } while (0); */
/* } */






/* static inline __attribute__((always_inline)) char *__bio_kmap_irq(struct bio *bio, unsigned short idx, */
/*        unsigned long *flags) */
/* { */
/*  return bvec_kmap_irq((&((bio)->bi_io_vec[(idx)])), flags); */
/* } */








/* struct scatterlist { */
/*     struct page *page; */
/*     unsigned int offset; */
/*     dma_addr_t dma_address; */
/*     unsigned int length; */
/* }; */


/* struct scsi_ioctl_command; */

/* struct request_queue; */
/* typedef struct request_queue request_queue_t; */
/* struct elevator_queue; */
/* typedef struct elevator_queue elevator_t; */
/* struct request_pm_state; */
/* struct blk_trace; */







/* struct as_io_context { */
/*  spinlock_t lock; */

/*  void (*dtor)(struct as_io_context *aic); */
/*  void (*exit)(struct as_io_context *aic); */

/*  unsigned long state; */
/*  atomic_t nr_queued; */
/*  atomic_t nr_dispatched; */



/*  unsigned long last_end_request; */
/*  unsigned long ttime_total; */
/*  unsigned long ttime_samples; */
/*  unsigned long ttime_mean; */

/*  unsigned int seek_samples; */
/*  sector_t last_request_pos; */
/*  u64 seek_total; */
/*  sector_t seek_mean; */
/* }; */

/* struct cfq_queue; */
/* struct cfq_io_context { */
/*  struct rb_node rb_node; */
/*  void *key; */

/*  struct cfq_queue *cfqq[2]; */

/*  struct io_context *ioc; */

/*  unsigned long last_end_request; */
/*  sector_t last_request_pos; */
/*   unsigned long last_queue; */

/*  unsigned long ttime_total; */
/*  unsigned long ttime_samples; */
/*  unsigned long ttime_mean; */

/*  unsigned int seek_samples; */
/*  u64 seek_total; */
/*  sector_t seek_mean; */

/*  struct list_head queue_list; */

/*  void (*dtor)(struct io_context *); */
/*  void (*exit)(struct io_context *); */
/* }; */






/* struct io_context { */
/*  atomic_t refcount; */
/*  struct task_struct *task; */

/*  int (*set_ioprio)(struct io_context *, unsigned int); */




/*  unsigned long last_waited; */
/*  int nr_batch_requests; */

/*  struct as_io_context *aic; */
/*  struct rb_root cic_root; */
/* }; */

/* void put_io_context(struct io_context *ioc); */
/* void exit_io_context(void); */
/* struct io_context *current_io_context(gfp_t gfp_flags); */
/* struct io_context *get_io_context(gfp_t gfp_flags); */
/* void copy_io_context(struct io_context **pdst, struct io_context **psrc); */
/* void swap_io_context(struct io_context **ioc1, struct io_context **ioc2); */

/* struct request; */
/* typedef void (rq_end_io_fn)(struct request *, int); */

/* struct request_list { */
/*  int count[2]; */
/*  int starved[2]; */
/*  int elvpriv; */
/*  mempool_t *rq_pool; */
/*  wait_queue_head_t wait[2]; */
/* }; */






/* struct request { */
/*  struct list_head queuelist; */
/*  struct list_head donelist; */

/*  unsigned long flags; */





/*  sector_t sector; */
/*  unsigned long nr_sectors; */

/*  unsigned int current_nr_sectors; */

/*  sector_t hard_sector; */
/*  unsigned long hard_nr_sectors; */

/*  unsigned int hard_cur_sectors; */

/*  struct bio *bio; */
/*  struct bio *biotail; */

/*  void *elevator_private; */
/*  void *completion_data; */

/*  int rq_status; */
/*  int errors; */
/*  struct gendisk *rq_disk; */
/*  unsigned long start_time; */




/*  unsigned short nr_phys_segments; */






/*  unsigned short nr_hw_segments; */

/*  unsigned short ioprio; */

/*  int tag; */

/*  int ref_count; */
/*  request_queue_t *q; */
/*  struct request_list *rl; */

/*  struct completion *waiting; */
/*  void *special; */
/*  char *buffer; */




/*  unsigned int cmd_len; */
/*  unsigned char cmd[16]; */

/*  unsigned int data_len; */
/*  unsigned int sense_len; */
/*  void *data; */
/*  void *sense; */

/*  unsigned int timeout; */
/*  int retries; */




/*  rq_end_io_fn *end_io; */
/*  void *end_io_data; */
/* }; */




/* enum rq_flag_bits { */
/*  __REQ_RW, */
/*  __REQ_FAILFAST, */
/*  __REQ_SORTED, */
/*  __REQ_SOFTBARRIER, */
/*  __REQ_HARDBARRIER, */
/*  __REQ_FUA, */
/*  __REQ_CMD, */
/*  __REQ_NOMERGE, */
/*  __REQ_STARTED, */
/*  __REQ_DONTPREP, */
/*  __REQ_QUEUED, */
/*  __REQ_ELVPRIV, */



/*  __REQ_PC, */
/*  __REQ_BLOCK_PC, */
/*  __REQ_SENSE, */

/*  __REQ_FAILED, */
/*  __REQ_QUIET, */
/*  __REQ_SPECIAL, */
/*  __REQ_DRIVE_CMD, */
/*  __REQ_DRIVE_TASK, */
/*  __REQ_DRIVE_TASKFILE, */
/*  __REQ_PREEMPT, */
/*  __REQ_PM_SUSPEND, */
/*  __REQ_PM_RESUME, */
/*  __REQ_PM_SHUTDOWN, */
/*  __REQ_ORDERED_COLOR, */
/*  __REQ_RW_SYNC, */
/*  __REQ_NR_BITS, */
/* }; */

/* struct request_pm_state */
/* { */

/*  int pm_step; */

/*  u32 pm_state; */
/*  void* data; */
/* }; */





/* typedef int (elevator_merge_fn) (request_queue_t *, struct request **, */
/*      struct bio *); */

/* typedef void (elevator_merge_req_fn) (request_queue_t *, struct request *, struct request *); */

/* typedef void (elevator_merged_fn) (request_queue_t *, struct request *); */

/* typedef int (elevator_dispatch_fn) (request_queue_t *, int); */

/* typedef void (elevator_add_req_fn) (request_queue_t *, struct request *); */
/* typedef int (elevator_queue_empty_fn) (request_queue_t *); */
/* typedef struct request *(elevator_request_list_fn) (request_queue_t *, struct request *); */
/* typedef void (elevator_completed_req_fn) (request_queue_t *, struct request *); */
/* typedef int (elevator_may_queue_fn) (request_queue_t *, int, struct bio *); */

/* typedef int (elevator_set_req_fn) (request_queue_t *, struct request *, struct bio *, gfp_t); */
/* typedef void (elevator_put_req_fn) (request_queue_t *, struct request *); */
/* typedef void (elevator_activate_req_fn) (request_queue_t *, struct request *); */
/* typedef void (elevator_deactivate_req_fn) (request_queue_t *, struct request *); */

/* typedef void *(elevator_init_fn) (request_queue_t *, elevator_t *); */
/* typedef void (elevator_exit_fn) (elevator_t *); */

/* struct elevator_ops */
/* { */
/*  elevator_merge_fn *elevator_merge_fn; */
/*  elevator_merged_fn *elevator_merged_fn; */
/*  elevator_merge_req_fn *elevator_merge_req_fn; */

/*  elevator_dispatch_fn *elevator_dispatch_fn; */
/*  elevator_add_req_fn *elevator_add_req_fn; */
/*  elevator_activate_req_fn *elevator_activate_req_fn; */
/*  elevator_deactivate_req_fn *elevator_deactivate_req_fn; */

/*  elevator_queue_empty_fn *elevator_queue_empty_fn; */
/*  elevator_completed_req_fn *elevator_completed_req_fn; */

/*  elevator_request_list_fn *elevator_former_req_fn; */
/*  elevator_request_list_fn *elevator_latter_req_fn; */

/*  elevator_set_req_fn *elevator_set_req_fn; */
/*  elevator_put_req_fn *elevator_put_req_fn; */

/*  elevator_may_queue_fn *elevator_may_queue_fn; */

/*  elevator_init_fn *elevator_init_fn; */
/*  elevator_exit_fn *elevator_exit_fn; */
/*  void (*trim)(struct io_context *); */
/* }; */



/* struct elv_fs_entry { */
/*  struct attribute attr; */
/*  ssize_t (*show)(elevator_t *, char *); */
/*  ssize_t (*store)(elevator_t *, const char *, size_t); */
/* }; */




/* struct elevator_type */
/* { */
/*  struct list_head list; */
/*  struct elevator_ops ops; */
/*  struct elevator_type *elevator_type; */
/*  struct elv_fs_entry *elevator_attrs; */
/*  char elevator_name[(16)]; */
/*  struct module *elevator_owner; */
/* }; */




/* struct elevator_queue */
/* { */
/*  struct elevator_ops *ops; */
/*  void *elevator_data; */
/*  struct kobject kobj; */
/*  struct elevator_type *elevator_type; */
/*  struct mutex sysfs_lock; */
/* }; */




/* extern void elv_dispatch_sort(request_queue_t *, struct request *); */
/* extern void elv_add_request(request_queue_t *, struct request *, int, int); */
/* extern void __elv_add_request(request_queue_t *, struct request *, int, int); */
/* extern void elv_insert(request_queue_t *, struct request *, int); */
/* extern int elv_merge(request_queue_t *, struct request **, struct bio *); */
/* extern void elv_merge_requests(request_queue_t *, struct request *, */
/*           struct request *); */
/* extern void elv_merged_request(request_queue_t *, struct request *); */
/* extern void elv_dequeue_request(request_queue_t *, struct request *); */
/* extern void elv_requeue_request(request_queue_t *, struct request *); */
/* extern int elv_queue_empty(request_queue_t *); */
/* extern struct request *elv_next_request(struct request_queue *q); */
/* extern struct request *elv_former_request(request_queue_t *, struct request *); */
/* extern struct request *elv_latter_request(request_queue_t *, struct request *); */
/* extern int elv_register_queue(request_queue_t *q); */
/* extern void elv_unregister_queue(request_queue_t *q); */
/* extern int elv_may_queue(request_queue_t *, int, struct bio *); */
/* extern void elv_completed_request(request_queue_t *, struct request *); */
/* extern int elv_set_request(request_queue_t *, struct request *, struct bio *, gfp_t); */
/* extern void elv_put_request(request_queue_t *, struct request *); */




/* extern int elv_register(struct elevator_type *); */
/* extern void elv_unregister(struct elevator_type *); */




/* extern ssize_t elv_iosched_show(request_queue_t *, char *); */
/* extern ssize_t elv_iosched_store(request_queue_t *, const char *, size_t); */

/* extern int elevator_init(request_queue_t *, char *); */
/* extern void elevator_exit(elevator_t *); */
/* extern int elv_rq_merge_ok(struct request *, struct bio *); */

/* enum { */
/*  ELV_MQUEUE_MAY, */
/*  ELV_MQUEUE_NO, */
/*  ELV_MQUEUE_MUST, */
/* }; */


/* typedef int (merge_request_fn) (request_queue_t *, struct request *, */
/*     struct bio *); */
/* typedef int (merge_requests_fn) (request_queue_t *, struct request *, */
/*      struct request *); */
/* typedef void (request_fn_proc) (request_queue_t *q); */
/* typedef int (make_request_fn) (request_queue_t *q, struct bio *bio); */
/* typedef int (prep_rq_fn) (request_queue_t *, struct request *); */
/* typedef void (unplug_fn) (request_queue_t *); */

/* struct bio_vec; */
/* typedef int (merge_bvec_fn) (request_queue_t *, struct bio *, struct bio_vec *); */
/* typedef void (activity_fn) (void *data, int rw); */
/* typedef int (issue_flush_fn) (request_queue_t *, struct gendisk *, sector_t *); */
/* typedef void (prepare_flush_fn) (request_queue_t *, struct request *); */
/* typedef void (softirq_done_fn)(struct request *); */

/* enum blk_queue_state { */
/*  Queue_down, */
/*  Queue_up, */
/* }; */

/* struct blk_queue_tag { */
/*  struct request **tag_index; */
/*  unsigned long *tag_map; */
/*  struct list_head busy_list; */
/*  int busy; */
/*  int max_depth; */
/*  int real_max_depth; */
/*  atomic_t refcnt; */
/* }; */

/* struct request_queue */
/* { */



/*  struct list_head queue_head; */
/*  struct request *last_merge; */
/*  elevator_t *elevator; */




/*  struct request_list rq; */

/*  request_fn_proc *request_fn; */
/*  merge_request_fn *back_merge_fn; */
/*  merge_request_fn *front_merge_fn; */
/*  merge_requests_fn *merge_requests_fn; */
/*  make_request_fn *make_request_fn; */
/*  prep_rq_fn *prep_rq_fn; */
/*  unplug_fn *unplug_fn; */
/*  merge_bvec_fn *merge_bvec_fn; */
/*  activity_fn *activity_fn; */
/*  issue_flush_fn *issue_flush_fn; */
/*  prepare_flush_fn *prepare_flush_fn; */
/*  softirq_done_fn *softirq_done_fn; */




/*  sector_t end_sector; */
/*  struct request *boundary_rq; */




/*  struct timer_list unplug_timer; */
/*  int unplug_thresh; */
/*  unsigned long unplug_delay; */
/*  struct work_struct unplug_work; */

/*  struct backing_dev_info backing_dev_info; */





/*  void *queuedata; */

/*  void *activity_data; */




/*  unsigned long bounce_pfn; */
/*  gfp_t bounce_gfp; */




/*  unsigned long queue_flags; */






/*  spinlock_t __queue_lock; */
/*  spinlock_t *queue_lock; */




/*  struct kobject kobj; */




/*  unsigned long nr_requests; */
/*  unsigned int nr_congestion_on; */
/*  unsigned int nr_congestion_off; */
/*  unsigned int nr_batching; */

/*  unsigned int max_sectors; */
/*  unsigned int max_hw_sectors; */
/*  unsigned short max_phys_segments; */
/*  unsigned short max_hw_segments; */
/*  unsigned short hardsect_size; */
/*  unsigned int max_segment_size; */

/*  unsigned long seg_boundary_mask; */
/*  unsigned int dma_alignment; */

/*  struct blk_queue_tag *queue_tags; */

/*  unsigned int nr_sorted; */
/*  unsigned int in_flight; */




/*  unsigned int sg_timeout; */
/*  unsigned int sg_reserved_size; */
/*  int node; */

/*  struct blk_trace *blk_trace; */




/*  unsigned int ordered, next_ordered, ordseq; */
/*  int orderr, ordcolor; */
/*  struct request pre_flush_rq, bar_rq, post_flush_rq; */
/*  struct request *orig_bar_rq; */
/*  unsigned int bi_size; */

/*  struct mutex sysfs_lock; */
/* }; */

/* enum { */

/*  QUEUE_ORDERED_NONE = 0x00, */
/*  QUEUE_ORDERED_DRAIN = 0x01, */
/*  QUEUE_ORDERED_TAG = 0x02, */

/*  QUEUE_ORDERED_PREFLUSH = 0x10, */
/*  QUEUE_ORDERED_POSTFLUSH = 0x20, */
/*  QUEUE_ORDERED_FUA = 0x40, */

/*  QUEUE_ORDERED_DRAIN_FLUSH = QUEUE_ORDERED_DRAIN | */
/*    QUEUE_ORDERED_PREFLUSH | QUEUE_ORDERED_POSTFLUSH, */
/*  QUEUE_ORDERED_DRAIN_FUA = QUEUE_ORDERED_DRAIN | */
/*    QUEUE_ORDERED_PREFLUSH | QUEUE_ORDERED_FUA, */
/*  QUEUE_ORDERED_TAG_FLUSH = QUEUE_ORDERED_TAG | */
/*    QUEUE_ORDERED_PREFLUSH | QUEUE_ORDERED_POSTFLUSH, */
/*  QUEUE_ORDERED_TAG_FUA = QUEUE_ORDERED_TAG | */
/*    QUEUE_ORDERED_PREFLUSH | QUEUE_ORDERED_FUA, */




/*  QUEUE_ORDSEQ_STARTED = 0x01, */
/*  QUEUE_ORDSEQ_DRAIN = 0x02, */
/*  QUEUE_ORDSEQ_PREFLUSH = 0x04, */
/*  QUEUE_ORDSEQ_BAR = 0x08, */
/*  QUEUE_ORDSEQ_POSTFLUSH = 0x10, */
/*  QUEUE_ORDSEQ_DONE = 0x20, */
/* }; */

/* static inline __attribute__((always_inline)) int blk_queue_full(struct request_queue *q, int rw) */
/* { */
/*  if (rw == 0) */
/*   return (__builtin_constant_p(3) ? constant_test_bit((3),(&q->queue_flags)) : variable_test_bit((3),(&q->queue_flags))); */
/*  return (__builtin_constant_p(4) ? constant_test_bit((4),(&q->queue_flags)) : variable_test_bit((4),(&q->queue_flags))); */
/* } */

/* static inline __attribute__((always_inline)) void blk_set_queue_full(struct request_queue *q, int rw) */
/* { */
/*  if (rw == 0) */
/*   set_bit(3, &q->queue_flags); */
/*  else */
/*   set_bit(4, &q->queue_flags); */
/* } */

/* static inline __attribute__((always_inline)) void blk_clear_queue_full(struct request_queue *q, int rw) */
/* { */
/*  if (rw == 0) */
/*   clear_bit(3, &q->queue_flags); */
/*  else */
/*   clear_bit(4, &q->queue_flags); */
/* } */

/* extern unsigned long blk_max_low_pfn, blk_max_pfn; */

/* extern int init_emergency_isa_pool(void); */
/* extern void blk_queue_bounce(request_queue_t *q, struct bio **bio); */

/* struct sec_size { */
/*  unsigned block_size; */
/*  unsigned block_size_bits; */
/* }; */

/* extern int blk_register_queue(struct gendisk *disk); */
/* extern void blk_unregister_queue(struct gendisk *disk); */
/* extern void register_disk(struct gendisk *dev); */
/* extern void generic_make_request(struct bio *bio); */
/* extern void blk_put_request(struct request *); */
/* extern void __blk_put_request(request_queue_t *, struct request *); */
/* extern void blk_end_sync_rq(struct request *rq, int error); */
/* extern struct request *blk_get_request(request_queue_t *, int, gfp_t); */
/* extern void blk_insert_request(request_queue_t *, struct request *, int, void *); */
/* extern void blk_requeue_request(request_queue_t *, struct request *); */
/* extern void blk_plug_device(request_queue_t *); */
/* extern int blk_remove_plug(request_queue_t *); */
/* extern void blk_recount_segments(request_queue_t *, struct bio *); */
/* extern int scsi_cmd_ioctl(struct file *, struct gendisk *, unsigned int, void *); */
/* extern int sg_scsi_ioctl(struct file *, struct request_queue *, */
/*   struct gendisk *, struct scsi_ioctl_command *); */
/* extern void blk_start_queue(request_queue_t *q); */
/* extern void blk_stop_queue(request_queue_t *q); */
/* extern void blk_sync_queue(struct request_queue *q); */
/* extern void __blk_stop_queue(request_queue_t *q); */
/* extern void blk_run_queue(request_queue_t *); */
/* extern void blk_queue_activity_fn(request_queue_t *, activity_fn *, void *); */
/* extern int blk_rq_map_user(request_queue_t *, struct request *, void *, unsigned int); */
/* extern int blk_rq_unmap_user(struct bio *, unsigned int); */
/* extern int blk_rq_map_kern(request_queue_t *, struct request *, void *, unsigned int, gfp_t); */
/* extern int blk_rq_map_user_iov(request_queue_t *, struct request *, struct sg_iovec *, int); */
/* extern int blk_execute_rq(request_queue_t *, struct gendisk *, */
/*      struct request *, int); */
/* extern void blk_execute_rq_nowait(request_queue_t *, struct gendisk *, */
/*       struct request *, int, rq_end_io_fn *); */

/* static inline __attribute__((always_inline)) request_queue_t *bdev_get_queue(struct block_device *bdev) */
/* { */
/*  return bdev->bd_disk->queue; */
/* } */

/* static inline __attribute__((always_inline)) void blk_run_backing_dev(struct backing_dev_info *bdi, */
/*            struct page *page) */
/* { */
/*  if (bdi && bdi->unplug_io_fn) */
/*   bdi->unplug_io_fn(bdi, page); */
/* } */

/* static inline __attribute__((always_inline)) void blk_run_address_space(struct address_space *mapping) */
/* { */
/*  if (mapping) */
/*   blk_run_backing_dev(mapping->backing_dev_info, ((void *)0)); */
/* } */

/* extern int end_that_request_first(struct request *, int, int); */
/* extern int end_that_request_chunk(struct request *, int, int); */
/* extern void end_that_request_last(struct request *, int); */
/* extern void end_request(struct request *req, int uptodate); */
/* extern void blk_complete_request(struct request *); */

/* static inline __attribute__((always_inline)) int rq_all_done(struct request *rq, unsigned int nr_bytes) */
/* { */
/*  if (((rq)->flags & (1 << __REQ_CMD))) */
/*   return (nr_bytes >= (rq->hard_nr_sectors << 9)); */
/*  else if (((rq)->flags & (1 << __REQ_BLOCK_PC))) */
/*   return nr_bytes >= rq->data_len; */

/*  return 0; */
/* } */

/* static inline __attribute__((always_inline)) void blkdev_dequeue_request(struct request *req) */
/* { */
/*  elv_dequeue_request(req->q, req); */
/* } */




/* static inline __attribute__((always_inline)) void elv_dispatch_add_tail(struct request_queue *q, */
/*       struct request *rq) */
/* { */
/*  if (q->last_merge == rq) */
/*   q->last_merge = ((void *)0); */
/*  q->nr_sorted--; */

/*  q->end_sector = ((rq)->sector + (rq)->nr_sectors); */
/*  q->boundary_rq = rq; */
/*  list_add_tail(&rq->queuelist, &q->queue_head); */
/* } */




/* extern request_queue_t *blk_init_queue_node(request_fn_proc *rfn, */
/*      spinlock_t *lock, int node_id); */
/* extern request_queue_t *blk_init_queue(request_fn_proc *, spinlock_t *); */
/* extern void blk_cleanup_queue(request_queue_t *); */
/* extern void blk_queue_make_request(request_queue_t *, make_request_fn *); */
/* extern void blk_queue_bounce_limit(request_queue_t *, u64); */
/* extern void blk_queue_max_sectors(request_queue_t *, unsigned int); */
/* extern void blk_queue_max_phys_segments(request_queue_t *, unsigned short); */
/* extern void blk_queue_max_hw_segments(request_queue_t *, unsigned short); */
/* extern void blk_queue_max_segment_size(request_queue_t *, unsigned int); */
/* extern void blk_queue_hardsect_size(request_queue_t *, unsigned short); */
/* extern void blk_queue_stack_limits(request_queue_t *t, request_queue_t *b); */
/* extern void blk_queue_segment_boundary(request_queue_t *, unsigned long); */
/* extern void blk_queue_prep_rq(request_queue_t *, prep_rq_fn *pfn); */
/* extern void blk_queue_merge_bvec(request_queue_t *, merge_bvec_fn *); */
/* extern void blk_queue_dma_alignment(request_queue_t *, int); */
/* extern void blk_queue_softirq_done(request_queue_t *, softirq_done_fn *); */
/* extern struct backing_dev_info *blk_get_backing_dev_info(struct block_device *bdev); */
/* extern int blk_queue_ordered(request_queue_t *, unsigned, prepare_flush_fn *); */
/* extern void blk_queue_issue_flush_fn(request_queue_t *, issue_flush_fn *); */
/* extern int blk_do_ordered(request_queue_t *, struct request **); */
/* extern unsigned blk_ordered_cur_seq(request_queue_t *); */
/* extern unsigned blk_ordered_req_seq(struct request *); */
/* extern void blk_ordered_complete_seq(request_queue_t *, unsigned, int); */

/* extern int blk_rq_map_sg(request_queue_t *, struct request *, struct scatterlist *); */
/* extern void blk_dump_rq_flags(struct request *, char *); */
/* extern void generic_unplug_device(request_queue_t *); */
/* extern void __generic_unplug_device(request_queue_t *); */
/* extern long nr_blockdev_pages(void); */

/* int blk_get_queue(request_queue_t *); */
/* request_queue_t *blk_alloc_queue(gfp_t); */
/* request_queue_t *blk_alloc_queue_node(gfp_t, int); */
/* extern void blk_put_queue(request_queue_t *); */







/* extern int blk_queue_start_tag(request_queue_t *, struct request *); */
/* extern struct request *blk_queue_find_tag(request_queue_t *, int); */
/* extern void blk_queue_end_tag(request_queue_t *, struct request *); */
/* extern int blk_queue_init_tags(request_queue_t *, int, struct blk_queue_tag *); */
/* extern void blk_queue_free_tags(request_queue_t *); */
/* extern int blk_queue_resize_tags(request_queue_t *, int); */
/* extern void blk_queue_invalidate_tags(request_queue_t *); */
/* extern long blk_congestion_wait(int rw, long timeout); */

/* extern void blk_rq_bio_prep(request_queue_t *, struct request *, struct bio *); */
/* extern int blkdev_issue_flush(struct block_device *, sector_t *); */

/* static inline __attribute__((always_inline)) int queue_hardsect_size(request_queue_t *q) */
/* { */
/*  int retval = 512; */

/*  if (q && q->hardsect_size) */
/*   retval = q->hardsect_size; */

/*  return retval; */
/* } */

/* static inline __attribute__((always_inline)) int bdev_hardsect_size(struct block_device *bdev) */
/* { */
/*  return queue_hardsect_size(bdev_get_queue(bdev)); */
/* } */

/* static inline __attribute__((always_inline)) int queue_dma_alignment(request_queue_t *q) */
/* { */
/*  int retval = 511; */

/*  if (q && q->dma_alignment) */
/*   retval = q->dma_alignment; */

/*  return retval; */
/* } */

/* static inline __attribute__((always_inline)) int bdev_dma_aligment(struct block_device *bdev) */
/* { */
/*  return queue_dma_alignment(bdev_get_queue(bdev)); */
/* } */





/* static inline __attribute__((always_inline)) unsigned int blksize_bits(unsigned int size) */
/* { */
/*  unsigned int bits = 8; */
/*  do { */
/*   bits++; */
/*   size >>= 1; */
/*  } while (size > 256); */
/*  return bits; */
/* } */

/* static inline __attribute__((always_inline)) unsigned int block_size(struct block_device *bdev) */
/* { */
/*  return bdev->bd_block_size; */
/* } */

/* typedef struct {struct page *v;} Sector; */

/* unsigned char *read_dev_sector(struct block_device *, sector_t, Sector *); */

/* static inline __attribute__((always_inline)) void put_dev_sector(Sector p) */
/* { */
/*  put_page(p.v); */
/* } */

/* struct work_struct; */
/* int kblockd_schedule_work(struct work_struct *work); */
/* void kblockd_flush(void); */







/* struct tms { */
/*  clock_t tms_utime; */
/*  clock_t tms_stime; */
/*  clock_t tms_cutime; */
/*  clock_t tms_cstime; */
/* }; */





/* static int debug; */

/* static int keeplocked; */

/* static int autoclose=1; */
/* static int autoeject; */
/* static int lockdoor = 1; */

/* static int check_media_type; */

/* static int mrw_format_restart = 1; */
/* static inline __attribute__((always_inline)) int *__check_debug(void) { return(&(debug)); }; static char __param_str_debug[] = "debug"; static struct kernel_param const __param_debug __attribute__((__used__)) __attribute__ ((unused,__section__ ("__param"),aligned(sizeof(void *)))) = { __param_str_debug, 0, param_set_bool, param_get_bool, &debug }; static const char __mod_debugtype298[] __attribute__((__used__)) __attribute__((section(".modinfo"),unused)) = "parmtype" "=" "debug" ":" "bool"; */
/* static inline __attribute__((always_inline)) int *__check_autoclose(void) { return(&(autoclose)); }; static char __param_str_autoclose[] = "autoclose"; static struct kernel_param const __param_autoclose __attribute__((__used__)) __attribute__ ((unused,__section__ ("__param"),aligned(sizeof(void *)))) = { __param_str_autoclose, 0, param_set_bool, param_get_bool, &autoclose }; static const char __mod_autoclosetype299[] __attribute__((__used__)) __attribute__((section(".modinfo"),unused)) = "parmtype" "=" "autoclose" ":" "bool"; */
/* static inline __attribute__((always_inline)) int *__check_autoeject(void) { return(&(autoeject)); }; static char __param_str_autoeject[] = "autoeject"; static struct kernel_param const __param_autoeject __attribute__((__used__)) __attribute__ ((unused,__section__ ("__param"),aligned(sizeof(void *)))) = { __param_str_autoeject, 0, param_set_bool, param_get_bool, &autoeject }; static const char __mod_autoejecttype300[] __attribute__((__used__)) __attribute__((section(".modinfo"),unused)) = "parmtype" "=" "autoeject" ":" "bool"; */
/* static inline __attribute__((always_inline)) int *__check_lockdoor(void) { return(&(lockdoor)); }; static char __param_str_lockdoor[] = "lockdoor"; static struct kernel_param const __param_lockdoor __attribute__((__used__)) __attribute__ ((unused,__section__ ("__param"),aligned(sizeof(void *)))) = { __param_str_lockdoor, 0, param_set_bool, param_get_bool, &lockdoor }; static const char __mod_lockdoortype301[] __attribute__((__used__)) __attribute__((section(".modinfo"),unused)) = "parmtype" "=" "lockdoor" ":" "bool"; */
/* static inline __attribute__((always_inline)) int *__check_check_media_type(void) { return(&(check_media_type)); }; static char __param_str_check_media_type[] = "check_media_type"; static struct kernel_param const __param_check_media_type __attribute__((__used__)) __attribute__ ((unused,__section__ ("__param"),aligned(sizeof(void *)))) = { __param_str_check_media_type, 0, param_set_bool, param_get_bool, &check_media_type }; static const char __mod_check_media_typetype302[] __attribute__((__used__)) __attribute__((section(".modinfo"),unused)) = "parmtype" "=" "check_media_type" ":" "bool"; */
/* static inline __attribute__((always_inline)) int *__check_mrw_format_restart(void) { return(&(mrw_format_restart)); }; static char __param_str_mrw_format_restart[] = "mrw_format_restart"; static struct kernel_param const __param_mrw_format_restart __attribute__((__used__)) __attribute__ ((unused,__section__ ("__param"),aligned(sizeof(void *)))) = { __param_str_mrw_format_restart, 0, param_set_bool, param_get_bool, &mrw_format_restart }; static const char __mod_mrw_format_restarttype303[] __attribute__((__used__)) __attribute__((section(".modinfo"),unused)) = "parmtype" "=" "mrw_format_restart" ":" "bool"; */

/* static spinlock_t cdrom_lock = (spinlock_t) { .raw_lock = { 1 }, .magic = 0xdead4ead, .owner = ((void *)-1L), .owner_cpu = -1, }; */

/* static const char *mrw_format_status[] = { */
/*  "not mrw", */
/*  "bgformat inactive", */
/*  "bgformat active", */
/*  "mrw complete", */
/* }; */

/* static const char *mrw_address_space[] = { "DMA", "GAA" }; */

/* static int open_for_data(struct cdrom_device_info * cdi); */
/* static int check_for_audio_disc(struct cdrom_device_info * cdi, */
/*     struct cdrom_device_ops * cdo); */
/* static void sanitize_format(union cdrom_addr *addr, */
/*   u_char * curr, u_char requested); */
/* static int mmc_ioctl(struct cdrom_device_info *cdi, unsigned int cmd, */
/*        unsigned long arg); */

/* int cdrom_get_last_written(struct cdrom_device_info *, long *); */
/* static int cdrom_get_next_writable(struct cdrom_device_info *, long *); */
/* static void cdrom_count_tracks(struct cdrom_device_info *, tracktype*); */

/* static int cdrom_mrw_exit(struct cdrom_device_info *cdi); */

/* static int cdrom_get_disc_info(struct cdrom_device_info *cdi, disc_information *di); */


/* static void cdrom_sysctl_register(void); */

/* static struct cdrom_device_info *topCdromPtr; */

/* static int cdrom_dummy_generic_packet(struct cdrom_device_info *cdi, */
/*           struct packet_command *cgc) */
/* { */
/*  if (cgc->sense) { */
/*   cgc->sense->sense_key = 0x05; */
/*   cgc->sense->asc = 0x20; */
/*   cgc->sense->ascq = 0x00; */
/*  } */

/*  cgc->stat = -5; */
/*  return -5; */
/* } */

/* int register_cdrom(struct cdrom_device_info *cdi) */
/* { */
/*  static char banner_printed; */
/*         struct cdrom_device_ops *cdo = cdi->ops; */
/*         int *change_capability = (int *)&cdo->capability; */

/*  if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "entering register_cdrom\n"); */

/*  if (cdo->open == ((void *)0) || cdo->release == ((void *)0)) */
/*   return -2; */
/*  if (!banner_printed) { */
/*   printk("<6>" "Uniform CD-ROM driver " "Revision: 3.20" "\n"); */
/*   banner_printed = 1; */

/*   cdrom_sysctl_register(); */

/*  } */

/*  if (cdo->drive_status == ((void *)0)) *change_capability &= ~(0x800); */
/*  if (cdo->media_changed == ((void *)0)) *change_capability &= ~(0x80); */
/*  if (cdo->tray_move == ((void *)0)) *change_capability &= ~(0x1 | 0x2); */
/*  if (cdo->lock_door == ((void *)0)) *change_capability &= ~(0x4); */
/*  if (cdo->select_speed == ((void *)0)) *change_capability &= ~(0x8); */
/*  if (cdo->get_last_session == ((void *)0)) *change_capability &= ~(0x20); */
/*  if (cdo->get_mcn == ((void *)0)) *change_capability &= ~(0x40); */
/*  if (cdo->reset == ((void *)0)) *change_capability &= ~(0x200); */
/*  if (cdo->audio_ioctl == ((void *)0)) *change_capability &= ~(0x100); */
/*  if (cdo->generic_packet == ((void *)0)) *change_capability &= ~(0x1000); */
/*  cdi->mc_flags = 0; */
/*  cdo->n_minors = 0; */
/*         cdi->options = 0x4; */

/*  if (autoclose==1 && (cdi->ops->capability & ~cdi->mask & (0x1))) */
/*   cdi->options |= (int) 0x1; */
/*  if (autoeject==1 && (cdi->ops->capability & ~cdi->mask & (0x2))) */
/*   cdi->options |= (int) 0x2; */
/*  if (lockdoor==1) */
/*   cdi->options |= (int) 0x8; */
/*  if (check_media_type==1) */
/*   cdi->options |= (int) 0x10; */

/*  if ((cdi->ops->capability & ~cdi->mask & (0x100000))) */
/*   cdi->exit = cdrom_mrw_exit; */

/*  if (cdi->disk) */
/*   cdi->cdda_method = 2; */
/*  else */
/*   cdi->cdda_method = 0; */

/*  if (!cdo->generic_packet) */
/*   cdo->generic_packet = cdrom_dummy_generic_packet; */

/*  if ((0x1 & 0x2) || debug==1 ) printk("<6>" "cdrom: " "drive \"/dev/%s\" registered\n", cdi->name); */
/*  _spin_lock(&cdrom_lock); */
/*  cdi->next = topCdromPtr; */
/*  topCdromPtr = cdi; */
/*  _spin_unlock(&cdrom_lock); */
/*  return 0; */
/* } */


/* int unregister_cdrom(struct cdrom_device_info *unreg) */
/* { */
/*  struct cdrom_device_info *cdi, *prev; */
/*  if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "entering unregister_cdrom\n"); */

/*  prev = ((void *)0); */
/*  _spin_lock(&cdrom_lock); */
/*  cdi = topCdromPtr; */
/*  while (cdi && cdi != unreg) { */
/*   prev = cdi; */
/*   cdi = cdi->next; */
/*  } */

/*  if (cdi == ((void *)0)) { */
/*   _spin_unlock(&cdrom_lock); */
/*   return -2; */
/*  } */
/*  if (prev) */
/*   prev->next = cdi->next; */
/*  else */
/*   topCdromPtr = cdi->next; */

/*  _spin_unlock(&cdrom_lock); */

/*  if (cdi->exit) */
/*    cdi->exit(cdi); */

/*  cdi->ops->n_minors--; */
/*  if ((0x1 & 0x2) || debug==1 ) printk("<6>" "cdrom: " "drive \"/dev/%s\" unregistered\n", cdi->name); */
/*  return 0; */
/* } */

/* extern int cdrom_get_media_event(struct cdrom_device_info *cdi, struct media_event_desc *med); */




/* static int cdrom_mrw_probe_pc(struct cdrom_device_info *cdi) */
/* { */
/*  struct packet_command cgc; */
/*  char buffer[16]; */

/*  init_cdrom_command(&cgc, buffer, sizeof(buffer), 2); */

/*  cgc.timeout = 1000; */
/*  cgc.quiet = 1; */

/*  if (!cdrom_mode_sense(cdi, &cgc, 0x03, 0)) { */
/*   cdi->mrw_mode_page = 0x03; */
/*   return 0; */
/*  } else if (!cdrom_mode_sense(cdi, &cgc, 0x2c, 0)) { */
/*   cdi->mrw_mode_page = 0x2c; */
/*   return 0; */
/*  } */

/*  return 1; */
/* } */

/* extern int cdrom_is_mrw(struct cdrom_device_info *cdi, int *write); */

/* static int cdrom_mrw_bgformat(struct cdrom_device_info *cdi, int cont) */
/* { */
/*  struct packet_command cgc; */
/*  unsigned char buffer[12]; */
/*  int ret; */

/*  printk("<6>" "cdrom: %sstarting format\n", cont ? "Re" : ""); */




/*  init_cdrom_command(&cgc, buffer, sizeof(buffer), 1); */
/*  cgc.cmd[0] = 0x04; */
/*  cgc.cmd[1] = (1 << 4) | 1; */

/*  cgc.timeout = 5 * 60 * 1000; */




/*  buffer[1] = 1 << 1; */
/*  buffer[3] = 8; */




/*  buffer[4] = 0xff; */
/*  buffer[5] = 0xff; */
/*  buffer[6] = 0xff; */
/*  buffer[7] = 0xff; */

/*  buffer[8] = 0x24 << 2; */
/*  buffer[11] = cont; */

/*  ret = cdi->ops->generic_packet(cdi, &cgc); */
/*  if (ret) */
/*   printk("<6>" "cdrom: bgformat failed\n"); */

/*  return ret; */
/* } */

/* static int cdrom_mrw_bgformat_susp(struct cdrom_device_info *cdi, int immed) */
/* { */
/*  struct packet_command cgc; */

/*  init_cdrom_command(&cgc, ((void *)0), 0, 3); */
/*  cgc.cmd[0] = 0x5b; */




/*  cgc.cmd[1] = !!immed; */
/*  cgc.cmd[2] = 1 << 1; */

/*  cgc.timeout = 5 * 60 * 1000; */

/*  return cdi->ops->generic_packet(cdi, &cgc); */
/* } */

/* static int cdrom_flush_cache(struct cdrom_device_info *cdi) */
/* { */
/*  struct packet_command cgc; */

/*  init_cdrom_command(&cgc, ((void *)0), 0, 3); */
/*  cgc.cmd[0] = 0x35; */

/*  cgc.timeout = 5 * 60 * 1000; */

/*  return cdi->ops->generic_packet(cdi, &cgc); */
/* } */

/* static int cdrom_mrw_exit(struct cdrom_device_info *cdi) */
/* { */
/*  disc_information di; */
/*  int ret; */

/*  ret = cdrom_get_disc_info(cdi, &di); */
/*  if (ret < 0 || ret < (int)__builtin_offsetof(typeof(di),disc_type)) */
/*   return 1; */

/*  ret = 0; */
/*  if (di.mrw_status == 2) { */
/*   printk("<6>" "cdrom: issuing MRW back ground " */
/*     "format suspend\n"); */
/*   ret = cdrom_mrw_bgformat_susp(cdi, 0); */
/*  } */

/*  if (!ret && cdi->media_written) */
/*   ret = cdrom_flush_cache(cdi); */

/*  return ret; */
/* } */

/* extern int cdrom_mrw_set_lba_space(struct cdrom_device_info *cdi, int space); */

/* static int cdrom_get_random_writable(struct cdrom_device_info *cdi, */
/*          struct rwrt_feature_desc *rfd) */
/* { */
/*  struct packet_command cgc; */
/*  char buffer[24]; */
/*  int ret; */

/*  init_cdrom_command(&cgc, buffer, sizeof(buffer), 2); */

/*  cgc.cmd[0] = 0x46; */
/*  cgc.cmd[3] = 0x0020; */
/*  cgc.cmd[8] = sizeof(buffer); */
/*  cgc.quiet = 1; */

/*  if ((ret = cdi->ops->generic_packet(cdi, &cgc))) */
/*   return ret; */

/*  (__builtin_constant_p(sizeof (*rfd)) ? __constant_memcpy((rfd),(&buffer[sizeof(struct feature_header)]),(sizeof (*rfd))) : __memcpy((rfd),(&buffer[sizeof(struct feature_header)]),(sizeof (*rfd)))); */
/*  return 0; */
/* } */

/* static int cdrom_has_defect_mgt(struct cdrom_device_info *cdi) */
/* { */
/*  struct packet_command cgc; */
/*  char buffer[16]; */
/*  __u16 *feature_code; */
/*  int ret; */

/*  init_cdrom_command(&cgc, buffer, sizeof(buffer), 2); */

/*  cgc.cmd[0] = 0x46; */
/*  cgc.cmd[3] = 0x0024; */
/*  cgc.cmd[8] = sizeof(buffer); */
/*  cgc.quiet = 1; */

/*  if ((ret = cdi->ops->generic_packet(cdi, &cgc))) */
/*   return ret; */

/*  feature_code = (__u16 *) &buffer[sizeof(struct feature_header)]; */
/*  if ((__builtin_constant_p((__u16)(( __u16)(__be16)(*feature_code))) ? ({ __u16 __x = ((( __u16)(__be16)(*feature_code))); ((__u16)( (((__u16)(__x) & (__u16)0x00ffU) << 8) | (((__u16)(__x) & (__u16)0xff00U) >> 8) )); }) : __fswab16((( __u16)(__be16)(*feature_code)))) == 0x0024) */
/*   return 0; */

/*  return 1; */
/* } */


/* static int cdrom_is_random_writable(struct cdrom_device_info *cdi, int *write) */
/* { */
/*  struct rwrt_feature_desc rfd; */
/*  int ret; */

/*  *write = 0; */

/*  if ((ret = cdrom_get_random_writable(cdi, &rfd))) */
/*   return ret; */

/*  if (0x0020 == (__builtin_constant_p((__u16)(( __u16)(__be16)(rfd.feature_code))) ? ({ __u16 __x = ((( __u16)(__be16)(rfd.feature_code))); ((__u16)( (((__u16)(__x) & (__u16)0x00ffU) << 8) | (((__u16)(__x) & (__u16)0xff00U) >> 8) )); }) : __fswab16((( __u16)(__be16)(rfd.feature_code))))) */
/*   *write = 1; */

/*  return 0; */
/* } */

/* static int cdrom_media_erasable(struct cdrom_device_info *cdi) */
/* { */
/*  disc_information di; */
/*  int ret; */

/*  ret = cdrom_get_disc_info(cdi, &di); */
/*  if (ret < 0 || ret < __builtin_offsetof(typeof(di),n_first_track)) */
/*   return -1; */

/*  return di.erasable; */
/* } */




/* static int cdrom_dvdram_open_write(struct cdrom_device_info *cdi) */
/* { */
/*  int ret = cdrom_media_erasable(cdi); */





/*  if (!ret) */
/*   return 1; */

/*  return 0; */
/* } */

/* static int cdrom_mrw_open_write(struct cdrom_device_info *cdi) */
/* { */
/*  disc_information di; */
/*  int ret; */




/*  if (cdrom_mrw_set_lba_space(cdi, 0)) { */
/*   printk("<3>" "cdrom: failed setting lba address space\n"); */
/*   return 1; */
/*  } */

/*  ret = cdrom_get_disc_info(cdi, &di); */
/*  if (ret < 0 || ret < __builtin_offsetof(typeof(di),disc_type)) */
/*   return 1; */

/*  if (!di.erasable) */
/*   return 1; */

/*  ret = 0; */
/*  printk("<6>" "cdrom open: mrw_status '%s'\n", */
/*    mrw_format_status[di.mrw_status]); */
/*  if (!di.mrw_status) */
/*   ret = 1; */
/*  else if (di.mrw_status == 1 && */
/*    mrw_format_restart) */
/*   ret = cdrom_mrw_bgformat(cdi, 1); */

/*  return ret; */
/* } */

/* static int mo_open_write(struct cdrom_device_info *cdi) */
/* { */
/*  struct packet_command cgc; */
/*  char buffer[255]; */
/*  int ret; */

/*  init_cdrom_command(&cgc, &buffer, 4, 2); */
/*  cgc.quiet = 1; */






/*  ret = cdrom_mode_sense(cdi, &cgc, 0x3f, 0); */
/*  if (ret) */
/*   ret = cdrom_mode_sense(cdi, &cgc, 0x00, 0); */
/*  if (ret) { */
/*   cgc.buflen = 255; */
/*   ret = cdrom_mode_sense(cdi, &cgc, 0x3f, 0); */
/*  } */


/*  if (ret) */
/*   return 0; */

/*  return buffer[3] & 0x80; */
/* } */

/* static int cdrom_ram_open_write(struct cdrom_device_info *cdi) */
/* { */
/*  struct rwrt_feature_desc rfd; */
/*  int ret; */

/*  if ((ret = cdrom_has_defect_mgt(cdi))) */
/*   return ret; */

/*  if ((ret = cdrom_get_random_writable(cdi, &rfd))) */
/*   return ret; */
/*  else if (0x0020 == (__builtin_constant_p((__u16)(( __u16)(__be16)(rfd.feature_code))) ? ({ __u16 __x = ((( __u16)(__be16)(rfd.feature_code))); ((__u16)( (((__u16)(__x) & (__u16)0x00ffU) << 8) | (((__u16)(__x) & (__u16)0xff00U) >> 8) )); }) : __fswab16((( __u16)(__be16)(rfd.feature_code))))) */
/*   ret = !rfd.curr; */

/*  if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "can open for random write\n"); */
/*  return ret; */
/* } */

/* static void cdrom_mmc3_profile(struct cdrom_device_info *cdi) */
/* { */
/*  struct packet_command cgc; */
/*  char buffer[32]; */
/*  int ret, mmc3_profile; */

/*  init_cdrom_command(&cgc, buffer, sizeof(buffer), 2); */

/*  cgc.cmd[0] = 0x46; */
/*  cgc.cmd[1] = 0; */
/*  cgc.cmd[2] = cgc.cmd[3] = 0; */
/*  cgc.cmd[8] = sizeof(buffer); */
/*  cgc.quiet = 1; */

/*  if ((ret = cdi->ops->generic_packet(cdi, &cgc))) */
/*   mmc3_profile = 0xffff; */
/*  else */
/*   mmc3_profile = (buffer[6] << 8) | buffer[7]; */

/*  cdi->mmc3_profile = mmc3_profile; */
/* } */

/* static int cdrom_is_dvd_rw(struct cdrom_device_info *cdi) */
/* { */
/*  switch (cdi->mmc3_profile) { */
/*  case 0x12: */
/*  case 0x1A: */
/*   return 0; */
/*  default: */
/*   return 1; */
/*  } */
/* } */




/* static int cdrom_open_write(struct cdrom_device_info *cdi) */
/* { */
/*  int mrw, mrw_write, ram_write; */
/*  int ret = 1; */

/*  mrw = 0; */
/*  if (!cdrom_is_mrw(cdi, &mrw_write)) */
/*   mrw = 1; */

/*  if ((cdi->ops->capability & ~cdi->mask & (0x40000))) */
/*   ram_write = 1; */
/*  else */
/*   (void) cdrom_is_random_writable(cdi, &ram_write); */

/*  if (mrw) */
/*   cdi->mask &= ~0x80000; */
/*  else */
/*   cdi->mask |= 0x80000; */

/*  if (mrw_write) */
/*   cdi->mask &= ~0x100000; */
/*  else */
/*   cdi->mask |= 0x100000; */

/*  if (ram_write) */
/*   cdi->mask &= ~0x200000; */
/*  else */
/*   cdi->mask |= 0x200000; */

/*  if ((cdi->ops->capability & ~cdi->mask & (0x100000))) */
/*   ret = cdrom_mrw_open_write(cdi); */
/*  else if ((cdi->ops->capability & ~cdi->mask & (0x20000))) */
/*   ret = cdrom_dvdram_open_write(cdi); */
/*   else if ((cdi->ops->capability & ~cdi->mask & (0x200000)) && */
/*     !(cdi->ops->capability & ~cdi->mask & (0x2000|0x4000|0x8000|0x10000|0x80000|0x40000))) */
/*    ret = cdrom_ram_open_write(cdi); */
/*  else if ((cdi->ops->capability & ~cdi->mask & (0x40000))) */
/*   ret = mo_open_write(cdi); */
/*  else if (!cdrom_is_dvd_rw(cdi)) */
/*   ret = 0; */

/*  return ret; */
/* } */

/* static void cdrom_dvd_rw_close_write(struct cdrom_device_info *cdi) */
/* { */
/*  struct packet_command cgc; */

/*  if (cdi->mmc3_profile != 0x1a) { */
/*   if ((0x1 & 0x10) || debug==1 ) printk("<6>" "cdrom: " "%s: No DVD+RW\n", cdi->name); */
/*   return; */
/*  } */

/*  if (!cdi->media_written) { */
/*   if ((0x1 & 0x10) || debug==1 ) printk("<6>" "cdrom: " "%s: DVD+RW media clean\n", cdi->name); */
/*   return; */
/*  } */

/*  printk("<6>" "cdrom: %s: dirty DVD+RW media, \"finalizing\"\n", */
/*         cdi->name); */

/*  init_cdrom_command(&cgc, ((void *)0), 0, 3); */
/*  cgc.cmd[0] = 0x35; */
/*  cgc.timeout = 30*1000; */
/*  cdi->ops->generic_packet(cdi, &cgc); */

/*  init_cdrom_command(&cgc, ((void *)0), 0, 3); */
/*  cgc.cmd[0] = 0x5b; */
/*  cgc.timeout = 3000*1000; */
/*  cgc.quiet = 1; */
/*  cdi->ops->generic_packet(cdi, &cgc); */

/*  init_cdrom_command(&cgc, ((void *)0), 0, 3); */
/*  cgc.cmd[0] = 0x5b; */
/*  cgc.cmd[2] = 2; */
/*  cgc.quiet = 1; */
/*  cgc.timeout = 3000*1000; */
/*  cdi->ops->generic_packet(cdi, &cgc); */

/*  cdi->media_written = 0; */
/* } */

/* static int cdrom_close_write(struct cdrom_device_info *cdi) */
/* { */



/*  return 0; */

/* } */

/* int cdrom_open(struct cdrom_device_info *cdi, struct inode *ip, struct file *fp) */
/* { */
/*  int ret; */

/*  if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "entering cdrom_open\n"); */



/*  cdi->use_count++; */
/*  if ((fp->f_flags & 00004000) && (cdi->options & 0x4)) { */
/*   ret = cdi->ops->open(cdi, 1); */
/*  } else { */
/*   ret = open_for_data(cdi); */
/*   if (ret) */
/*    goto err; */
/*   cdrom_mmc3_profile(cdi); */
/*   if (fp->f_mode & 2) { */
/*    ret = -30; */
/*    if (cdrom_open_write(cdi)) */
/*     goto err_release; */
/*    if (!(cdi->ops->capability & ~cdi->mask & (0x200000))) */
/*     goto err_release; */
/*    ret = 0; */
/*    cdi->media_written = 0; */
/*   } */
/*  } */

/*  if (ret) */
/*   goto err; */

/*  if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "Use count for \"/dev/%s\" now %d\n", cdi->name, cdi->use_count); */



/*  check_disk_change(ip->i_bdev); */
/*  return 0; */
/* err_release: */
/*  cdi->ops->release(cdi); */
/* err: */
/*  cdi->use_count--; */
/*  return ret; */
/* } */

/* static */
/* int open_for_data(struct cdrom_device_info * cdi) */
/* { */
/*  int ret; */
/*  struct cdrom_device_ops *cdo = cdi->ops; */
/*  tracktype tracks; */
/*  if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "entering open_for_data\n"); */


/*  if (cdo->drive_status != ((void *)0)) { */
/*   ret = cdo->drive_status(cdi, ((int) (~0U>>1))); */
/*   if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "drive_status=%d\n", ret); */
/*   if (ret == 2) { */
/*    if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "the tray is open...\n"); */

/*    if ((cdi->ops->capability & ~cdi->mask & (0x1)) && */
/*        cdi->options & 0x1) { */
/*     if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "trying to close the tray.\n"); */
/*     ret=cdo->tray_move(cdi,0); */
/*     if (ret) { */
/*      if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "bummer. tried to close the tray but failed.\n"); */





/*      ret=-123; */
/*      goto clean_up_and_return; */
/*     } */
/*    } else { */
/*     if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "bummer. this drive can't close the tray.\n"); */
/*     ret=-123; */
/*     goto clean_up_and_return; */
/*    } */

/*    ret = cdo->drive_status(cdi, ((int) (~0U>>1))); */
/*    if ((ret == 1) || (ret==2)) { */
/*     if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "bummer. the tray is still not closed.\n"); */
/*     if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "tray might not contain a medium.\n"); */
/*     ret=-123; */
/*     goto clean_up_and_return; */
/*    } */
/*    if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "the tray is now closed.\n"); */
/*   } */

/*   ret = cdo->drive_status(cdi, ((int) (~0U>>1))); */
/*   if (ret!=4) { */
/*    ret = -123; */
/*    goto clean_up_and_return; */
/*   } */
/*  } */
/*  cdrom_count_tracks(cdi, &tracks); */
/*  if (tracks.error == 1) { */
/*   if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "bummer. no disc.\n"); */
/*   ret=-123; */
/*   goto clean_up_and_return; */
/*  } */


/*  if (tracks.data==0) { */
/*   if (cdi->options & 0x10) { */


/*       if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "bummer. wrong media type.\n"); */
/*       if ((0x1 & 0x1) || debug==1 ) printk("<6>" "cdrom: " "pid %d must open device O_NONBLOCK!\n", (unsigned int)__vericon_dummy_current->pid); */

/*       ret=-124; */
/*       goto clean_up_and_return; */
/*   } */
/*   else { */
/*       if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "wrong media type, but CDO_CHECK_TYPE not set.\n"); */
/*   } */
/*  } */

/*  if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "all seems well, opening the device.\n"); */


/*  ret = cdo->open(cdi, 0); */
/*  if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "opening the device gave me %d.\n", ret); */



/*  if (ret) { */
/*   if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "open device failed.\n"); */
/*   goto clean_up_and_return; */
/*  } */
/*  if ((cdi->ops->capability & ~cdi->mask & (0x4)) && (cdi->options & 0x8)) { */
/*    cdo->lock_door(cdi, 1); */
/*    if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "door locked.\n"); */
/*  } */
/*  if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "device opened successfully.\n"); */
/*  return ret; */






/* clean_up_and_return: */
/*  if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "open failed.\n"); */
/*  if ((cdi->ops->capability & ~cdi->mask & (0x4)) && cdi->options & 0x8) { */
/*    cdo->lock_door(cdi, 0); */
/*    if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "door unlocked.\n"); */
/*  } */
/*  return ret; */
/* } */




/* int check_for_audio_disc(struct cdrom_device_info * cdi, */
/*     struct cdrom_device_ops * cdo) */
/* { */
/*         int ret; */
/*  tracktype tracks; */
/*  if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "entering check_for_audio_disc\n"); */
/*  if (!(cdi->options & 0x10)) */
/*   return 0; */
/*  if (cdo->drive_status != ((void *)0)) { */
/*   ret = cdo->drive_status(cdi, ((int) (~0U>>1))); */
/*   if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "drive_status=%d\n", ret); */
/*   if (ret == 2) { */
/*    if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "the tray is open...\n"); */

/*    if ((cdi->ops->capability & ~cdi->mask & (0x1)) && */
/*        cdi->options & 0x1) { */
/*     if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "trying to close the tray.\n"); */
/*     ret=cdo->tray_move(cdi,0); */
/*     if (ret) { */
/*      if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "bummer. tried to close tray but failed.\n"); */





/*      return -123; */
/*     } */
/*    } else { */
/*     if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "bummer. this driver can't close the tray.\n"); */
/*     return -123; */
/*    } */

/*    ret = cdo->drive_status(cdi, ((int) (~0U>>1))); */
/*    if ((ret == 1) || (ret==2)) { */
/*     if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "bummer. the tray is still not closed.\n"); */
/*     return -123; */
/*    } */
/*    if (ret!=4) { */
/*     if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "bummer. disc isn't ready.\n"); */
/*     return -5; */
/*    } */
/*    if ((0x1 & 0x8) || debug==1 ) printk("<6>" "cdrom: " "the tray is now closed.\n"); */
/*   } */
/*  } */
/*  cdrom_count_tracks(cdi, &tracks); */
/*  if (tracks.error) */
/*   return(tracks.error); */

/*  if (tracks.audio==0) */
/*   return -124; */

/*  return 0; */
/* } */


/* int cdrom_release(struct cdrom_device_info *cdi, struct file *fp) */
/* { */
/*  struct cdrom_device_ops *cdo = cdi->ops; */
/*  int opened_for_data; */

/*  if ((0x1 & 0x10) || debug==1 ) printk("<6>" "cdrom: " "entering cdrom_release\n"); */

/*  if (cdi->use_count > 0) */
/*   cdi->use_count--; */
/*  if (cdi->use_count == 0) */
/*   if ((0x1 & 0x10) || debug==1 ) printk("<6>" "cdrom: " "Use count for \"/dev/%s\" now zero\n", cdi->name); */
/*  if (cdi->use_count == 0) */
/*   cdrom_dvd_rw_close_write(cdi); */
/*  if (cdi->use_count == 0 && */
/*      (cdo->capability & 0x4) && !keeplocked) { */
/*   if ((0x1 & 0x10) || debug==1 ) printk("<6>" "cdrom: " "Unlocking door!\n"); */
/*   cdo->lock_door(cdi, 0); */
/*  } */
/*  opened_for_data = !(cdi->options & 0x4) || */
/*   !(fp && fp->f_flags & 00004000); */




/*  if ((cdi->ops->capability & ~cdi->mask & (0x200000)) && !cdi->use_count && cdi->for_data) */
/*   cdrom_close_write(cdi); */

/*  cdo->release(cdi); */
/*  if (cdi->use_count == 0) { */
/*   if (opened_for_data && */
/*       cdi->options & 0x2 && (cdi->ops->capability & ~cdi->mask & (0x2))) */
/*    cdo->tray_move(cdi, 1); */
/*  } */
/*  return 0; */
/* } */

/* static int cdrom_read_mech_status(struct cdrom_device_info *cdi, */
/*       struct cdrom_changer_info *buf) */
/* { */
/*  struct packet_command cgc; */
/*  struct cdrom_device_ops *cdo = cdi->ops; */
/*  int length; */






/*  if (cdi->sanyo_slot) { */
/*   buf->hdr.nslots = 3; */
/*   buf->hdr.curslot = cdi->sanyo_slot == 3 ? 0 : cdi->sanyo_slot; */
/*   for (length = 0; length < 3; length++) { */
/*    buf->slots[length].disc_present = 1; */
/*    buf->slots[length].change = 0; */
/*   } */
/*   return 0; */
/*  } */

/*  length = sizeof(struct cdrom_mechstat_header) + */
/*    cdi->capacity * sizeof(struct cdrom_slot); */

/*  init_cdrom_command(&cgc, buf, length, 2); */
/*  cgc.cmd[0] = 0xbd; */
/*  cgc.cmd[8] = (length >> 8) & 0xff; */
/*  cgc.cmd[9] = length & 0xff; */
/*  return cdo->generic_packet(cdi, &cgc); */
/* } */

/* static int cdrom_slot_status(struct cdrom_device_info *cdi, int slot) */
/* { */
/*  struct cdrom_changer_info *info; */
/*  int ret; */

/*  if ((0x1 & 0x40) || debug==1 ) printk("<6>" "cdrom: " "entering cdrom_slot_status()\n"); */
/*  if (cdi->sanyo_slot) */
/*   return 0; */

/*  info = kmalloc(sizeof(*info), ((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u))); */
/*  if (!info) */
/*   return -12; */

/*  if ((ret = cdrom_read_mech_status(cdi, info))) */
/*   goto out_free; */

/*  if (info->slots[slot].disc_present) */
/*   ret = 4; */
/*  else */
/*   ret = 1; */

/* out_free: */
/*  kfree(info); */
/*  return ret; */
/* } */




/* int cdrom_number_of_slots(struct cdrom_device_info *cdi) */
/* { */
/*  int status; */
/*  int nslots = 1; */
/*  struct cdrom_changer_info *info; */

/*  if ((0x1 & 0x40) || debug==1 ) printk("<6>" "cdrom: " "entering cdrom_number_of_slots()\n"); */

/*  cdi->capacity = 0; */

/*  info = kmalloc(sizeof(*info), ((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u))); */
/*  if (!info) */
/*   return -12; */

/*  if ((status = cdrom_read_mech_status(cdi, info)) == 0) */
/*   nslots = info->hdr.nslots; */

/*  kfree(info); */
/*  return nslots; */
/* } */


/* extern int cdrom_load_unload(struct cdrom_device_info *cdi, int slot); */

/* static int cdrom_select_disc(struct cdrom_device_info *cdi, int slot) */
/* { */
/*  struct cdrom_changer_info *info; */
/*  int curslot; */
/*  int ret; */

/*  if ((0x1 & 0x40) || debug==1 ) printk("<6>" "cdrom: " "entering cdrom_select_disc()\n"); */
/*  if (!(cdi->ops->capability & ~cdi->mask & (0x10))) */
/*   return -95; */

/*  (void) cdi->ops->media_changed(cdi, slot); */

/*  if (slot == ((int) (~0U>>1)-1)) { */

/*   cdi->mc_flags = 0x3; */
/*   return cdrom_load_unload(cdi, -1); */
/*  } */

/*  info = kmalloc(sizeof(*info), ((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u))); */
/*  if (!info) */
/*   return -12; */

/*  if ((ret = cdrom_read_mech_status(cdi, info))) { */
/*   kfree(info); */
/*   return ret; */
/*  } */

/*  curslot = info->hdr.curslot; */
/*  kfree(info); */

/*  if (cdi->use_count > 1 || keeplocked) { */
/*   if (slot == ((int) (~0U>>1))) { */
/*        return curslot; */
/*   } else { */
/*    return -16; */
/*   } */
/*  } */






/*  if (slot == ((int) (~0U>>1))) */
/*   slot = curslot; */


/*  cdi->mc_flags = 0x3; */
/*  if ((ret = cdrom_load_unload(cdi, slot))) */
/*   return ret; */

/*  return slot; */
/* } */







/* static */
/* int media_changed(struct cdrom_device_info *cdi, int queue) */
/* { */
/*  unsigned int mask = (1 << (queue & 1)); */
/*  int ret = !!(cdi->mc_flags & mask); */

/*  if (!(cdi->ops->capability & ~cdi->mask & (0x80))) */
/*      return ret; */

/*  if (cdi->ops->media_changed(cdi, ((int) (~0U>>1)))) { */
/*   cdi->mc_flags = 0x3; */
/*   ret |= 1; */
/*   cdi->media_written = 0; */
/*  } */
/*  cdi->mc_flags &= ~mask; */
/*  return ret; */
/* } */

/* int cdrom_media_changed(struct cdrom_device_info *cdi) */
/* { */



/*  if (cdi == ((void *)0) || cdi->ops->media_changed == ((void *)0)) */
/*   return 0; */
/*  if (!(cdi->ops->capability & ~cdi->mask & (0x80))) */
/*   return 0; */
/*  return media_changed(cdi, 0); */
/* } */


/* static void cdrom_count_tracks(struct cdrom_device_info *cdi, tracktype* tracks) */
/* { */
/*  struct cdrom_tochdr header; */
/*  struct cdrom_tocentry entry; */
/*  int ret, i; */
/*  tracks->data=0; */
/*  tracks->audio=0; */
/*  tracks->cdi=0; */
/*  tracks->xa=0; */
/*  tracks->error=0; */
/*  if ((0x1 & 0x20) || debug==1 ) printk("<6>" "cdrom: " "entering cdrom_count_tracks\n"); */
/*         if (!(cdi->ops->capability & ~cdi->mask & (0x100))) { */
/*                 tracks->error=0; */
/*                 return; */
/*         } */

/*  if ((ret = cdi->ops->audio_ioctl(cdi, 0x5305, &header))) { */
/*   if (ret == -123) */
/*    tracks->error = 1; */
/*   else */
/*    tracks->error = 0; */
/*   return; */
/*  } */

/*  entry.cdte_format = 0x02; */
/*  for (i = header.cdth_trk0; i <= header.cdth_trk1; i++) { */
/*   entry.cdte_track = i; */
/*   if (cdi->ops->audio_ioctl(cdi, 0x5306, &entry)) { */
/*    tracks->error=0; */
/*    return; */
/*   } */
/*   if (entry.cdte_ctrl & 0x04) { */
/*       if (entry.cdte_format == 0x10) */
/*    tracks->cdi++; */
/*       else if (entry.cdte_format == 0x20) */
/*    tracks->xa++; */
/*       else */
/*    tracks->data++; */
/*   } else */
/*       tracks->audio++; */
/*   if ((0x1 & 0x20) || debug==1 ) printk("<6>" "cdrom: " "track %d: format=%d, ctrl=%d\n", i, entry.cdte_format, entry.cdte_ctrl); */

/*  } */
/*  if ((0x1 & 0x20) || debug==1 ) printk("<6>" "cdrom: " "disc has %d tracks: %d=audio %d=data %d=Cd-I %d=XA\n", header.cdth_trk1, tracks->audio, tracks->data, tracks->cdi, tracks->xa); */


/* } */

/* static */
/* void sanitize_format(union cdrom_addr *addr, */
/*        u_char * curr, u_char requested) */
/* { */
/*  if (*curr == requested) */
/*   return; */
/*  if (requested == 0x01) { */
/*   addr->lba = (int) addr->msf.frame + */
/*    75 * (addr->msf.second - 2 + 60 * addr->msf.minute); */
/*  } else { */
/*   int lba = addr->lba; */
/*   addr->msf.frame = lba % 75; */
/*   lba /= 75; */
/*   lba += 2; */
/*   addr->msf.second = lba % 60; */
/*   addr->msf.minute = lba / 60; */
/*  } */
/*  *curr = requested; */
/* } */

/* void init_cdrom_command(struct packet_command *cgc, void *buf, int len, */
/*    int type) */
/* { */
/*  (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(struct packet_command))) ? __constant_c_and_count_memset(((cgc)),((0x01010101UL*(unsigned char)(0))),((sizeof(struct packet_command)))) : __constant_c_memset(((cgc)),((0x01010101UL*(unsigned char)(0))),((sizeof(struct packet_command))))) : (__builtin_constant_p((sizeof(struct packet_command))) ? __memset_generic((((cgc))),(((0))),(((sizeof(struct packet_command))))) : __memset_generic(((cgc)),((0)),((sizeof(struct packet_command)))))); */
/*  if (buf) */
/*   (__builtin_constant_p(0) ? (__builtin_constant_p((len)) ? __constant_c_and_count_memset(((buf)),((0x01010101UL*(unsigned char)(0))),((len))) : __constant_c_memset(((buf)),((0x01010101UL*(unsigned char)(0))),((len)))) : (__builtin_constant_p((len)) ? __memset_generic((((buf))),(((0))),(((len)))) : __memset_generic(((buf)),((0)),((len))))); */
/*  cgc->buffer = (char *) buf; */
/*  cgc->buflen = len; */
/*  cgc->data_direction = type; */
/*  cgc->timeout = 5*1000; */
/* } */






/* static void setup_report_key(struct packet_command *cgc, unsigned agid, unsigned type) */
/* { */
/*  cgc->cmd[0] = 0xa4; */
/*  cgc->cmd[10] = type | (agid << 6); */
/*  switch (type) { */
/*   case 0: case 8: case 5: { */
/*    cgc->buflen = 8; */
/*    break; */
/*   } */
/*   case 1: { */
/*    cgc->buflen = 16; */
/*    break; */
/*   } */
/*   case 2: case 4: { */
/*    cgc->buflen = 12; */
/*    break; */
/*   } */
/*  } */
/*  cgc->cmd[9] = cgc->buflen; */
/*  cgc->data_direction = 2; */
/* } */

/* static void setup_send_key(struct packet_command *cgc, unsigned agid, unsigned type) */
/* { */
/*  cgc->cmd[0] = 0xa3; */
/*  cgc->cmd[10] = type | (agid << 6); */
/*  switch (type) { */
/*   case 1: { */
/*    cgc->buflen = 16; */
/*    break; */
/*   } */
/*   case 3: { */
/*    cgc->buflen = 12; */
/*    break; */
/*   } */
/*   case 6: { */
/*    cgc->buflen = 8; */
/*    break; */
/*   } */
/*  } */
/*  cgc->cmd[9] = cgc->buflen; */
/*  cgc->data_direction = 1; */
/* } */

/* extern int dvd_do_auth(struct cdrom_device_info *cdi, dvd_authinfo *ai); */

/* static int dvd_read_physical(struct cdrom_device_info *cdi, dvd_struct *s) */
/* { */
/*  unsigned char buf[21], *base; */
/*  struct dvd_layer *layer; */
/*  struct packet_command cgc; */
/*  struct cdrom_device_ops *cdo = cdi->ops; */
/*  int ret, layer_num = s->physical.layer_num; */

/*  if (layer_num >= 4) */
/*   return -22; */

/*  init_cdrom_command(&cgc, buf, sizeof(buf), 2); */
/*  cgc.cmd[0] = 0xad; */
/*  cgc.cmd[6] = layer_num; */
/*  cgc.cmd[7] = s->type; */
/*  cgc.cmd[9] = cgc.buflen & 0xff; */




/*  cgc.quiet = 1; */

/*  if ((ret = cdo->generic_packet(cdi, &cgc))) */
/*   return ret; */

/*  base = &buf[4]; */
/*  layer = &s->physical.layer[layer_num]; */





/*  (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(*layer))) ? __constant_c_and_count_memset(((layer)),((0x01010101UL*(unsigned char)(0))),((sizeof(*layer)))) : __constant_c_memset(((layer)),((0x01010101UL*(unsigned char)(0))),((sizeof(*layer))))) : (__builtin_constant_p((sizeof(*layer))) ? __memset_generic((((layer))),(((0))),(((sizeof(*layer))))) : __memset_generic(((layer)),((0)),((sizeof(*layer)))))); */
/*  layer->book_version = base[0] & 0xf; */
/*  layer->book_type = base[0] >> 4; */
/*  layer->min_rate = base[1] & 0xf; */
/*  layer->disc_size = base[1] >> 4; */
/*  layer->layer_type = base[2] & 0xf; */
/*  layer->track_path = (base[2] >> 4) & 1; */
/*  layer->nlayers = (base[2] >> 5) & 3; */
/*  layer->track_density = base[3] & 0xf; */
/*  layer->linear_density = base[3] >> 4; */
/*  layer->start_sector = base[5] << 16 | base[6] << 8 | base[7]; */
/*  layer->end_sector = base[9] << 16 | base[10] << 8 | base[11]; */
/*  layer->end_sector_l0 = base[13] << 16 | base[14] << 8 | base[15]; */
/*  layer->bca = base[16] >> 7; */

/*  return 0; */
/* } */

/* static int dvd_read_copyright(struct cdrom_device_info *cdi, dvd_struct *s) */
/* { */
/*  int ret; */
/*  u_char buf[8]; */
/*  struct packet_command cgc; */
/*  struct cdrom_device_ops *cdo = cdi->ops; */

/*  init_cdrom_command(&cgc, buf, sizeof(buf), 2); */
/*  cgc.cmd[0] = 0xad; */
/*  cgc.cmd[6] = s->copyright.layer_num; */
/*  cgc.cmd[7] = s->type; */
/*  cgc.cmd[8] = cgc.buflen >> 8; */
/*  cgc.cmd[9] = cgc.buflen & 0xff; */

/*  if ((ret = cdo->generic_packet(cdi, &cgc))) */
/*   return ret; */

/*  s->copyright.cpst = buf[4]; */
/*  s->copyright.rmi = buf[5]; */

/*  return 0; */
/* } */

/* static int dvd_read_disckey(struct cdrom_device_info *cdi, dvd_struct *s) */
/* { */
/*  int ret, size; */
/*  u_char *buf; */
/*  struct packet_command cgc; */
/*  struct cdrom_device_ops *cdo = cdi->ops; */

/*  size = sizeof(s->disckey.value) + 4; */

/*  if ((buf = (u_char *) kmalloc(size, ((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u)))) == ((void *)0)) */
/*   return -12; */

/*  init_cdrom_command(&cgc, buf, size, 2); */
/*  cgc.cmd[0] = 0xad; */
/*  cgc.cmd[7] = s->type; */
/*  cgc.cmd[8] = size >> 8; */
/*  cgc.cmd[9] = size & 0xff; */
/*  cgc.cmd[10] = s->disckey.agid << 6; */

/*  if (!(ret = cdo->generic_packet(cdi, &cgc))) */
/*   (__builtin_constant_p(sizeof(s->disckey.value)) ? __constant_memcpy((s->disckey.value),(&buf[4]),(sizeof(s->disckey.value))) : __memcpy((s->disckey.value),(&buf[4]),(sizeof(s->disckey.value)))); */

/*  kfree(buf); */
/*  return ret; */
/* } */

/* static int dvd_read_bca(struct cdrom_device_info *cdi, dvd_struct *s) */
/* { */
/*  int ret; */
/*  u_char buf[4 + 188]; */
/*  struct packet_command cgc; */
/*  struct cdrom_device_ops *cdo = cdi->ops; */

/*  init_cdrom_command(&cgc, buf, sizeof(buf), 2); */
/*  cgc.cmd[0] = 0xad; */
/*  cgc.cmd[7] = s->type; */
/*  cgc.cmd[9] = cgc.buflen & 0xff; */

/*  if ((ret = cdo->generic_packet(cdi, &cgc))) */
/*   return ret; */

/*  s->bca.len = buf[0] << 8 | buf[1]; */
/*  if (s->bca.len < 12 || s->bca.len > 188) { */
/*   if ((0x1 & 0x1) || debug==1 ) printk("<6>" "cdrom: " "Received invalid BCA length (%d)\n", s->bca.len); */
/*   return -5; */
/*  } */
/*  (__builtin_constant_p(s->bca.len) ? __constant_memcpy((s->bca.value),(&buf[4]),(s->bca.len)) : __memcpy((s->bca.value),(&buf[4]),(s->bca.len))); */

/*  return 0; */
/* } */

/* static int dvd_read_manufact(struct cdrom_device_info *cdi, dvd_struct *s) */
/* { */
/*  int ret = 0, size; */
/*  u_char *buf; */
/*  struct packet_command cgc; */
/*  struct cdrom_device_ops *cdo = cdi->ops; */

/*  size = sizeof(s->manufact.value) + 4; */

/*  if ((buf = (u_char *) kmalloc(size, ((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u)))) == ((void *)0)) */
/*   return -12; */

/*  init_cdrom_command(&cgc, buf, size, 2); */
/*  cgc.cmd[0] = 0xad; */
/*  cgc.cmd[7] = s->type; */
/*  cgc.cmd[8] = size >> 8; */
/*  cgc.cmd[9] = size & 0xff; */

/*  if ((ret = cdo->generic_packet(cdi, &cgc))) { */
/*   kfree(buf); */
/*   return ret; */
/*  } */

/*  s->manufact.len = buf[0] << 8 | buf[1]; */
/*  if (s->manufact.len < 0 || s->manufact.len > 2048) { */
/*   if ((0x1 & 0x1) || debug==1 ) printk("<6>" "cdrom: " "Received invalid manufacture info length" " (%d)\n", s->manufact.len); */

/*   ret = -5; */
/*  } else { */
/*   (__builtin_constant_p(s->manufact.len) ? __constant_memcpy((s->manufact.value),(&buf[4]),(s->manufact.len)) : __memcpy((s->manufact.value),(&buf[4]),(s->manufact.len))); */
/*  } */

/*  kfree(buf); */
/*  return ret; */
/* } */

/* static int dvd_read_struct(struct cdrom_device_info *cdi, dvd_struct *s) */
/* { */
/*  switch (s->type) { */
/*  case 0x00: */
/*   return dvd_read_physical(cdi, s); */

/*  case 0x01: */
/*   return dvd_read_copyright(cdi, s); */

/*  case 0x02: */
/*   return dvd_read_disckey(cdi, s); */

/*  case 0x03: */
/*   return dvd_read_bca(cdi, s); */

/*  case 0x04: */
/*   return dvd_read_manufact(cdi, s); */

/*  default: */
/*   if ((0x1 & 0x1) || debug==1 ) printk("<6>" "cdrom: " ": Invalid DVD structure read requested (%d)\n", s->type); */

/*   return -22; */
/*  } */
/* } */

/* int cdrom_mode_sense(struct cdrom_device_info *cdi, */
/*        struct packet_command *cgc, */
/*        int page_code, int page_control) */
/* { */
/*  struct cdrom_device_ops *cdo = cdi->ops; */

/*  (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(cgc->cmd))) ? __constant_c_and_count_memset(((cgc->cmd)),((0x01010101UL*(unsigned char)(0))),((sizeof(cgc->cmd)))) : __constant_c_memset(((cgc->cmd)),((0x01010101UL*(unsigned char)(0))),((sizeof(cgc->cmd))))) : (__builtin_constant_p((sizeof(cgc->cmd))) ? __memset_generic((((cgc->cmd))),(((0))),(((sizeof(cgc->cmd))))) : __memset_generic(((cgc->cmd)),((0)),((sizeof(cgc->cmd)))))); */

/*  cgc->cmd[0] = 0x5a; */
/*  cgc->cmd[2] = page_code | (page_control << 6); */
/*  cgc->cmd[7] = cgc->buflen >> 8; */
/*  cgc->cmd[8] = cgc->buflen & 0xff; */
/*  cgc->data_direction = 2; */
/*  return cdo->generic_packet(cdi, cgc); */
/* } */

/* int cdrom_mode_select(struct cdrom_device_info *cdi, */
/*         struct packet_command *cgc) */
/* { */
/*  struct cdrom_device_ops *cdo = cdi->ops; */

/*  (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(cgc->cmd))) ? __constant_c_and_count_memset(((cgc->cmd)),((0x01010101UL*(unsigned char)(0))),((sizeof(cgc->cmd)))) : __constant_c_memset(((cgc->cmd)),((0x01010101UL*(unsigned char)(0))),((sizeof(cgc->cmd))))) : (__builtin_constant_p((sizeof(cgc->cmd))) ? __memset_generic((((cgc->cmd))),(((0))),(((sizeof(cgc->cmd))))) : __memset_generic(((cgc->cmd)),((0)),((sizeof(cgc->cmd)))))); */
/*  (__builtin_constant_p(0) ? (__builtin_constant_p((2)) ? __constant_c_and_count_memset(((cgc->buffer)),((0x01010101UL*(unsigned char)(0))),((2))) : __constant_c_memset(((cgc->buffer)),((0x01010101UL*(unsigned char)(0))),((2)))) : (__builtin_constant_p((2)) ? __memset_generic((((cgc->buffer))),(((0))),(((2)))) : __memset_generic(((cgc->buffer)),((0)),((2))))); */
/*  cgc->cmd[0] = 0x55; */
/*  cgc->cmd[1] = 0x10; */
/*  cgc->cmd[7] = cgc->buflen >> 8; */
/*  cgc->cmd[8] = cgc->buflen & 0xff; */
/*  cgc->data_direction = 1; */
/*  return cdo->generic_packet(cdi, cgc); */
/* } */

/* static int cdrom_read_subchannel(struct cdrom_device_info *cdi, */
/*      struct cdrom_subchnl *subchnl, int mcn) */
/* { */
/*  struct cdrom_device_ops *cdo = cdi->ops; */
/*  struct packet_command cgc; */
/*  char buffer[32]; */
/*  int ret; */

/*  init_cdrom_command(&cgc, buffer, 16, 2); */
/*  cgc.cmd[0] = 0x42; */
/*  cgc.cmd[1] = 2; */
/*  cgc.cmd[2] = 0x40; */
/*  cgc.cmd[3] = mcn ? 2 : 1; */
/*  cgc.cmd[8] = 16; */

/*  if ((ret = cdo->generic_packet(cdi, &cgc))) */
/*   return ret; */

/*  subchnl->cdsc_audiostatus = cgc.buffer[1]; */
/*  subchnl->cdsc_format = 0x02; */
/*  subchnl->cdsc_ctrl = cgc.buffer[5] & 0xf; */
/*  subchnl->cdsc_trk = cgc.buffer[6]; */
/*  subchnl->cdsc_ind = cgc.buffer[7]; */

/*  subchnl->cdsc_reladdr.msf.minute = cgc.buffer[13]; */
/*  subchnl->cdsc_reladdr.msf.second = cgc.buffer[14]; */
/*  subchnl->cdsc_reladdr.msf.frame = cgc.buffer[15]; */
/*  subchnl->cdsc_absaddr.msf.minute = cgc.buffer[9]; */
/*  subchnl->cdsc_absaddr.msf.second = cgc.buffer[10]; */
/*  subchnl->cdsc_absaddr.msf.frame = cgc.buffer[11]; */

/*  return 0; */
/* } */




/* static int cdrom_read_cd(struct cdrom_device_info *cdi, */
/*     struct packet_command *cgc, int lba, */
/*     int blocksize, int nblocks) */
/* { */
/*  struct cdrom_device_ops *cdo = cdi->ops; */

/*  (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(cgc->cmd))) ? __constant_c_and_count_memset(((&cgc->cmd)),((0x01010101UL*(unsigned char)(0))),((sizeof(cgc->cmd)))) : __constant_c_memset(((&cgc->cmd)),((0x01010101UL*(unsigned char)(0))),((sizeof(cgc->cmd))))) : (__builtin_constant_p((sizeof(cgc->cmd))) ? __memset_generic((((&cgc->cmd))),(((0))),(((sizeof(cgc->cmd))))) : __memset_generic(((&cgc->cmd)),((0)),((sizeof(cgc->cmd)))))); */
/*  cgc->cmd[0] = 0x28; */
/*  cgc->cmd[2] = (lba >> 24) & 0xff; */
/*  cgc->cmd[3] = (lba >> 16) & 0xff; */
/*  cgc->cmd[4] = (lba >> 8) & 0xff; */
/*  cgc->cmd[5] = lba & 0xff; */
/*  cgc->cmd[6] = (nblocks >> 16) & 0xff; */
/*  cgc->cmd[7] = (nblocks >> 8) & 0xff; */
/*  cgc->cmd[8] = nblocks & 0xff; */
/*  cgc->buflen = blocksize * nblocks; */
/*  return cdo->generic_packet(cdi, cgc); */
/* } */


/* static int cdrom_read_block(struct cdrom_device_info *cdi, */
/*        struct packet_command *cgc, */
/*        int lba, int nblocks, int format, int blksize) */
/* { */
/*  struct cdrom_device_ops *cdo = cdi->ops; */

/*  (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(cgc->cmd))) ? __constant_c_and_count_memset(((&cgc->cmd)),((0x01010101UL*(unsigned char)(0))),((sizeof(cgc->cmd)))) : __constant_c_memset(((&cgc->cmd)),((0x01010101UL*(unsigned char)(0))),((sizeof(cgc->cmd))))) : (__builtin_constant_p((sizeof(cgc->cmd))) ? __memset_generic((((&cgc->cmd))),(((0))),(((sizeof(cgc->cmd))))) : __memset_generic(((&cgc->cmd)),((0)),((sizeof(cgc->cmd)))))); */
/*  cgc->cmd[0] = 0xbe; */

/*  cgc->cmd[1] = format << 2; */

/*  cgc->cmd[2] = (lba >> 24) & 0xff; */
/*  cgc->cmd[3] = (lba >> 16) & 0xff; */
/*  cgc->cmd[4] = (lba >> 8) & 0xff; */
/*  cgc->cmd[5] = lba & 0xff; */

/*  cgc->cmd[6] = (nblocks >> 16) & 0xff; */
/*  cgc->cmd[7] = (nblocks >> 8) & 0xff; */
/*  cgc->cmd[8] = nblocks & 0xff; */
/*  cgc->buflen = blksize * nblocks; */


/*  switch (blksize) { */
/*  case (2352 -12 -4) : cgc->cmd[9] = 0x58; break; */
/*  case (2352 -12) : cgc->cmd[9] = 0x78; break; */
/*  case 2352 : cgc->cmd[9] = 0xf8; break; */
/*  default : cgc->cmd[9] = 0x10; */
/*  } */

/*  return cdo->generic_packet(cdi, cgc); */
/* } */

/* static int cdrom_read_cdda_old(struct cdrom_device_info *cdi, __u8 *ubuf, */
/*           int lba, int nframes) */
/* { */
/*  struct packet_command cgc; */
/*  int ret = 0; */
/*  int nr; */

/*  cdi->last_sense = 0; */

/*  (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(cgc))) ? __constant_c_and_count_memset(((&cgc)),((0x01010101UL*(unsigned char)(0))),((sizeof(cgc)))) : __constant_c_memset(((&cgc)),((0x01010101UL*(unsigned char)(0))),((sizeof(cgc))))) : (__builtin_constant_p((sizeof(cgc))) ? __memset_generic((((&cgc))),(((0))),(((sizeof(cgc))))) : __memset_generic(((&cgc)),((0)),((sizeof(cgc)))))); */




/*  nr = nframes; */
/*  do { */
/*   cgc.buffer = kmalloc(2352 * nr, ((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u))); */
/*   if (cgc.buffer) */
/*    break; */

/*   nr >>= 1; */
/*  } while (nr); */

/*  if (!nr) */
/*   return -12; */

/*  if (!(__builtin_expect(!!(({ unsigned long flag,sum; (void)0; asm("addl %3,%1 ; sbbl %0,%0; cmpl %1,%4; sbbl $0,%0" :"=&r" (flag), "=r" (sum) :"1" (ubuf),"g" ((int)(nframes * 2352)),"rm" (current_thread_info()->addr_limit.seg)); flag; }) == 0), 1))) { */
/*   ret = -14; */
/*   goto out; */
/*  } */

/*  cgc.data_direction = 2; */
/*  while (nframes > 0) { */
/*   if (nr > nframes) */
/*    nr = nframes; */

/*   ret = cdrom_read_block(cdi, &cgc, lba, nr, 1, 2352); */
/*   if (ret) */
/*    break; */
/*   if (__copy_to_user(ubuf, cgc.buffer, 2352 * nr)) { */
/*    ret = -14; */
/*    break; */
/*   } */
/*   ubuf += 2352 * nr; */
/*   nframes -= nr; */
/*   lba += nr; */
/*  } */
/* out: */
/*  kfree(cgc.buffer); */
/*  return ret; */
/* } */

/* static int cdrom_read_cdda_bpc(struct cdrom_device_info *cdi, __u8 *ubuf, */
/*           int lba, int nframes) */
/* { */
/*  request_queue_t *q = cdi->disk->queue; */
/*  struct request *rq; */
/*  struct bio *bio; */
/*  unsigned int len; */
/*  int nr, ret = 0; */

/*  if (!q) */
/*   return -6; */

/*  rq = blk_get_request(q, 0, ((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u))); */
/*  if (!rq) */
/*   return -12; */

/*  cdi->last_sense = 0; */

/*  while (nframes) { */
/*   nr = nframes; */
/*   if (cdi->cdda_method == 1) */
/*    nr = 1; */
/*   if (nr * 2352 > (q->max_sectors << 9)) */
/*    nr = (q->max_sectors << 9) / 2352; */

/*   len = nr * 2352; */

/*   ret = blk_rq_map_user(q, rq, ubuf, len); */
/*   if (ret) */
/*    break; */

/*   (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(rq->cmd))) ? __constant_c_and_count_memset(((rq->cmd)),((0x01010101UL*(unsigned char)(0))),((sizeof(rq->cmd)))) : __constant_c_memset(((rq->cmd)),((0x01010101UL*(unsigned char)(0))),((sizeof(rq->cmd))))) : (__builtin_constant_p((sizeof(rq->cmd))) ? __memset_generic((((rq->cmd))),(((0))),(((sizeof(rq->cmd))))) : __memset_generic(((rq->cmd)),((0)),((sizeof(rq->cmd)))))); */
/*   rq->cmd[0] = 0xbe; */
/*   rq->cmd[1] = 1 << 2; */
/*   rq->cmd[2] = (lba >> 24) & 0xff; */
/*   rq->cmd[3] = (lba >> 16) & 0xff; */
/*   rq->cmd[4] = (lba >> 8) & 0xff; */
/*   rq->cmd[5] = lba & 0xff; */
/*   rq->cmd[6] = (nr >> 16) & 0xff; */
/*   rq->cmd[7] = (nr >> 8) & 0xff; */
/*   rq->cmd[8] = nr & 0xff; */
/*   rq->cmd[9] = 0xf8; */

/*   rq->cmd_len = 12; */
/*   rq->flags |= (1 << __REQ_BLOCK_PC); */
/*   rq->timeout = 60 * 1000; */
/*   bio = rq->bio; */

/*   if (rq->bio) */
/*    blk_queue_bounce(q, &rq->bio); */

/*   if (blk_execute_rq(q, cdi->disk, rq, 0)) { */
/*    struct request_sense *s = rq->sense; */
/*    ret = -5; */
/*    cdi->last_sense = s->sense_key; */
/*   } */

/*   if (blk_rq_unmap_user(bio, len)) */
/*    ret = -14; */

/*   if (ret) */
/*    break; */

/*   nframes -= nr; */
/*   lba += nr; */
/*   ubuf += len; */
/*  } */

/*  blk_put_request(rq); */
/*  return ret; */
/* } */

/* static int cdrom_read_cdda(struct cdrom_device_info *cdi, __u8 *ubuf, */
/*       int lba, int nframes) */
/* { */
/*  int ret; */

/*  if (cdi->cdda_method == 0) */
/*   return cdrom_read_cdda_old(cdi, ubuf, lba, nframes); */

/* retry: */



/*  ret = cdrom_read_cdda_bpc(cdi, ubuf, lba, nframes); */
/*  if (!ret || ret != -5) */
/*   return ret; */





/*  if (cdi->cdda_method == 2 && nframes > 1) { */
/*   printk("cdrom: dropping to single frame dma\n"); */
/*   cdi->cdda_method = 1; */
/*   goto retry; */
/*  } */






/*  if (cdi->last_sense != 0x04 && cdi->last_sense != 0x0b) */
/*   return ret; */

/*  printk("cdrom: dropping to old style cdda (sense=%x)\n", cdi->last_sense); */
/*  cdi->cdda_method = 0; */
/*  return cdrom_read_cdda_old(cdi, ubuf, lba, nframes); */
/* } */

/* static int cdrom_ioctl_multisession(struct cdrom_device_info *cdi, */
/*   void *argp) */
/* { */
/*  struct cdrom_multisession ms_info; */
/*  u8 requested_format; */
/*  int ret; */

/*  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROMMULTISESSION\n"); */

/*  if (!(cdi->ops->capability & 0x20)) */
/*   return -38; */

/*  if (copy_from_user(&ms_info, argp, sizeof(ms_info))) */
/*   return -14; */

/*  requested_format = ms_info.addr_format; */
/*  if (requested_format != 0x02 && requested_format != 0x01) */
/*   return -22; */
/*  ms_info.addr_format = 0x01; */

/*  ret = cdi->ops->get_last_session(cdi, &ms_info); */
/*  if (ret) */
/*   return ret; */

/*  sanitize_format(&ms_info.addr, &ms_info.addr_format, requested_format); */

/*  if (copy_to_user(argp, &ms_info, sizeof(ms_info))) */
/*   return -14; */

/*  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "CDROMMULTISESSION successful\n"); */
/*  return 0; */
/* } */

/* static int cdrom_ioctl_eject(struct cdrom_device_info *cdi) */
/* { */
/*  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROMEJECT\n"); */

/*  if (!(cdi->ops->capability & ~cdi->mask & (0x2))) */
/*   return -38; */
/*  if (cdi->use_count != 1 || keeplocked) */
/*   return -16; */
/*  if ((cdi->ops->capability & ~cdi->mask & (0x4))) { */
/*   int ret = cdi->ops->lock_door(cdi, 0); */
/*   if (ret) */
/*    return ret; */
/*  } */

/*  return cdi->ops->tray_move(cdi, 1); */
/* } */

/* static int cdrom_ioctl_closetray(struct cdrom_device_info *cdi) */
/* { */
/*  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROMCLOSETRAY\n"); */

/*  if (!(cdi->ops->capability & ~cdi->mask & (0x1))) */
/*   return -38; */
/*  return cdi->ops->tray_move(cdi, 0); */
/* } */

/* static int cdrom_ioctl_eject_sw(struct cdrom_device_info *cdi, */
/*   unsigned long arg) */
/* { */
/*  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROMEJECT_SW\n"); */

/*  if (!(cdi->ops->capability & ~cdi->mask & (0x2))) */
/*   return -38; */
/*  if (keeplocked) */
/*   return -16; */

/*  cdi->options &= ~(0x1 | 0x2); */
/*  if (arg) */
/*   cdi->options |= 0x1 | 0x2; */
/*  return 0; */
/* } */

/* static int cdrom_ioctl_media_changed(struct cdrom_device_info *cdi, */
/*   unsigned long arg) */
/* { */
/*  struct cdrom_changer_info *info; */
/*  int ret; */

/*  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROM_MEDIA_CHANGED\n"); */

/*  if (!(cdi->ops->capability & ~cdi->mask & (0x80))) */
/*   return -38; */


/*  if (!(cdi->ops->capability & ~cdi->mask & (0x10)) || arg == ((int) (~0U>>1))) */
/*   return media_changed(cdi, 1); */

/*  if ((unsigned int)arg >= cdi->capacity) */
/*   return -22; */

/*  info = kmalloc(sizeof(*info), ((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u))); */
/*  if (!info) */
/*   return -12; */

/*  ret = cdrom_read_mech_status(cdi, info); */
/*  if (!ret) */
/*   ret = info->slots[arg].change; */
/*  kfree(info); */
/*  return ret; */
/* } */

/* static int cdrom_ioctl_set_options(struct cdrom_device_info *cdi, */
/*   unsigned long arg) */
/* { */
/*  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROM_SET_OPTIONS\n"); */





/*  switch (arg) { */
/*  case 0x4: */
/*  case 0x10: */
/*   break; */
/*  case 0x8: */
/*   if (!(cdi->ops->capability & ~cdi->mask & (0x4))) */
/*    return -38; */
/*   break; */
/*  case 0: */
/*   return cdi->options; */

/*  default: */
/*   if (!(cdi->ops->capability & ~cdi->mask & (arg))) */
/*    return -38; */
/*  } */
/*  cdi->options |= (int) arg; */
/*  return cdi->options; */
/* } */

/* static int cdrom_ioctl_clear_options(struct cdrom_device_info *cdi, */
/*   unsigned long arg) */
/* { */
/*  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROM_CLEAR_OPTIONS\n"); */

/*  cdi->options &= ~(int) arg; */
/*  return cdi->options; */
/* } */

/* static int cdrom_ioctl_select_speed(struct cdrom_device_info *cdi, */
/*   unsigned long arg) */
/* { */
/*  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROM_SELECT_SPEED\n"); */

/*  if (!(cdi->ops->capability & ~cdi->mask & (0x8))) */
/*   return -38; */
/*  return cdi->ops->select_speed(cdi, arg); */
/* } */

/* static int cdrom_ioctl_select_disc(struct cdrom_device_info *cdi, */
/*   unsigned long arg) */
/* { */
/*  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROM_SELECT_DISC\n"); */

/*  if (!(cdi->ops->capability & ~cdi->mask & (0x10))) */
/*   return -38; */

/*  if (arg != ((int) (~0U>>1)) && arg != ((int) (~0U>>1)-1)) { */
/*   if ((int)arg >= cdi->capacity) */
/*    return -22; */
/*  } */






/*  if (cdi->ops->select_disc) */
/*   return cdi->ops->select_disc(cdi, arg); */

/*  if ((0x1 & 0x40) || debug==1 ) printk("<6>" "cdrom: " "Using generic cdrom_select_disc()\n"); */
/*  return cdrom_select_disc(cdi, arg); */
/* } */

/* static int cdrom_ioctl_reset(struct cdrom_device_info *cdi, */
/*   struct block_device *bdev) */
/* { */
/*  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROM_RESET\n"); */

/*  if (!capable(21)) */
/*   return -13; */
/*  if (!(cdi->ops->capability & ~cdi->mask & (0x200))) */
/*   return -38; */
/*  invalidate_bdev(bdev, 0); */
/*  return cdi->ops->reset(cdi); */
/* } */

/* static int cdrom_ioctl_lock_door(struct cdrom_device_info *cdi, */
/*   unsigned long arg) */
/* { */
/*  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "%socking door.\n", arg ? "L" : "Unl"); */

/*  if (!(cdi->ops->capability & ~cdi->mask & (0x4))) */
/*   return -95; */

/*  keeplocked = arg ? 1 : 0; */





/*  if (cdi->use_count != 1 && !arg && !capable(21)) */
/*   return -16; */
/*  return cdi->ops->lock_door(cdi, arg); */
/* } */

/* static int cdrom_ioctl_debug(struct cdrom_device_info *cdi, */
/*   unsigned long arg) */
/* { */
/*  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "%sabling debug.\n", arg ? "En" : "Dis"); */

/*  if (!capable(21)) */
/*   return -13; */
/*  debug = arg ? 1 : 0; */
/*  return debug; */
/* } */

/* static int cdrom_ioctl_get_capability(struct cdrom_device_info *cdi) */
/* { */
/*  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROM_GET_CAPABILITY\n"); */
/*  return (cdi->ops->capability & ~cdi->mask); */
/* } */







/* static int cdrom_ioctl_get_mcn(struct cdrom_device_info *cdi, */
/*   void *argp) */
/* { */
/*  struct cdrom_mcn mcn; */
/*  int ret; */

/*  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROM_GET_MCN\n"); */

/*  if (!(cdi->ops->capability & 0x40)) */
/*   return -38; */
/*  ret = cdi->ops->get_mcn(cdi, &mcn); */
/*  if (ret) */
/*   return ret; */

/*  if (copy_to_user(argp, &mcn, sizeof(mcn))) */
/*   return -14; */
/*  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "CDROM_GET_MCN successful\n"); */
/*  return 0; */
/* } */

/* static int cdrom_ioctl_drive_status(struct cdrom_device_info *cdi, */
/*   unsigned long arg) */
/* { */
/*  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROM_DRIVE_STATUS\n"); */

/*  if (!(cdi->ops->capability & 0x800)) */
/*   return -38; */
/*  if (!(cdi->ops->capability & ~cdi->mask & (0x10)) || */
/*      (arg == ((int) (~0U>>1)) || arg == ((int) (~0U>>1)-1))) */
/*   return cdi->ops->drive_status(cdi, ((int) (~0U>>1))); */
/*  if (((int)arg >= cdi->capacity)) */
/*   return -22; */
/*  return cdrom_slot_status(cdi, arg); */
/* } */

/* static int cdrom_ioctl_disc_status(struct cdrom_device_info *cdi) */
/* { */
/*  tracktype tracks; */

/*  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROM_DISC_STATUS\n"); */

/*  cdrom_count_tracks(cdi, &tracks); */
/*  if (tracks.error) */
/*   return tracks.error; */


/*  if (tracks.audio > 0) { */
/*   if (!tracks.data && !tracks.cdi && !tracks.xa) */
/*    return 100; */
/*   else */
/*    return 105; */
/*  } */

/*  if (tracks.cdi > 0) */
/*   return 104; */
/*  if (tracks.xa > 0) */
/*   return 103; */
/*  if (tracks.data > 0) */
/*   return 101; */


/*  if ((0x1 & 0x1) || debug==1 ) printk("<6>" "cdrom: " "This disc doesn't have any tracks I recognize!\n"); */
/*  return 0; */
/* } */

/* static int cdrom_ioctl_changer_nslots(struct cdrom_device_info *cdi) */
/* { */
/*  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROM_CHANGER_NSLOTS\n"); */
/*  return cdi->capacity; */
/* } */

/* static int cdrom_ioctl_get_subchnl(struct cdrom_device_info *cdi, */
/*   void *argp) */
/* { */
/*  struct cdrom_subchnl q; */
/*  u8 requested, back; */
/*  int ret; */



/*  if (!(cdi->ops->capability & ~cdi->mask & (0x100))) */
/*   return -38; */
/*  if (copy_from_user(&q, argp, sizeof(q))) */
/*   return -14; */

/*  requested = q.cdsc_format; */
/*  if (requested != 0x02 && requested != 0x01) */
/*   return -22; */
/*  q.cdsc_format = 0x02; */

/*  ret = cdi->ops->audio_ioctl(cdi, 0x530b, &q); */
/*  if (ret) */
/*   return ret; */

/*  back = q.cdsc_format; */
/*  sanitize_format(&q.cdsc_absaddr, &back, requested); */
/*  sanitize_format(&q.cdsc_reladdr, &q.cdsc_format, requested); */

/*  if (copy_to_user(argp, &q, sizeof(q))) */
/*   return -14; */

/*  return 0; */
/* } */

/* static int cdrom_ioctl_read_tochdr(struct cdrom_device_info *cdi, */
/*   void *argp) */
/* { */
/*  struct cdrom_tochdr header; */
/*  int ret; */



/*  if (!(cdi->ops->capability & ~cdi->mask & (0x100))) */
/*   return -38; */
/*  if (copy_from_user(&header, argp, sizeof(header))) */
/*   return -14; */

/*  ret = cdi->ops->audio_ioctl(cdi, 0x5305, &header); */
/*  if (ret) */
/*   return ret; */

/*  if (copy_to_user(argp, &header, sizeof(header))) */
/*   return -14; */

/*  return 0; */
/* } */

/* static int cdrom_ioctl_read_tocentry(struct cdrom_device_info *cdi, */
/*   void *argp) */
/* { */
/*  struct cdrom_tocentry entry; */
/*  u8 requested_format; */
/*  int ret; */



/*  if (!(cdi->ops->capability & ~cdi->mask & (0x100))) */
/*   return -38; */
/*  if (copy_from_user(&entry, argp, sizeof(entry))) */
/*   return -14; */

/*  requested_format = entry.cdte_format; */
/*  if (requested_format != 0x02 && requested_format != 0x01) */
/*   return -22; */

/*  entry.cdte_format = 0x02; */
/*  ret = cdi->ops->audio_ioctl(cdi, 0x5306, &entry); */
/*  if (ret) */
/*   return ret; */
/*  sanitize_format(&entry.cdte_addr, &entry.cdte_format, requested_format); */

/*  if (copy_to_user(argp, &entry, sizeof(entry))) */
/*   return -14; */

/*  return 0; */
/* } */

/* static int cdrom_ioctl_play_msf(struct cdrom_device_info *cdi, */
/*   void *argp) */
/* { */
/*  struct cdrom_msf msf; */

/*  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROMPLAYMSF\n"); */

/*  if (!(cdi->ops->capability & ~cdi->mask & (0x100))) */
/*   return -38; */
/*  if (copy_from_user(&msf, argp, sizeof(msf))) */
/*   return -14; */
/*  return cdi->ops->audio_ioctl(cdi, 0x5303, &msf); */
/* } */

/* static int cdrom_ioctl_play_trkind(struct cdrom_device_info *cdi, */
/*   void *argp) */
/* { */
/*  struct cdrom_ti ti; */
/*  int ret; */

/*  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROMPLAYTRKIND\n"); */

/*  if (!(cdi->ops->capability & ~cdi->mask & (0x100))) */
/*   return -38; */
/*  if (copy_from_user(&ti, argp, sizeof(ti))) */
/*   return -14; */

/*  ret = check_for_audio_disc(cdi, cdi->ops); */
/*  if (ret) */
/*   return ret; */
/*  return cdi->ops->audio_ioctl(cdi, 0x5304, &ti); */
/* } */
/* static int cdrom_ioctl_volctrl(struct cdrom_device_info *cdi, */
/*   void *argp) */
/* { */
/*  struct cdrom_volctrl volume; */

/*  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROMVOLCTRL\n"); */

/*  if (!(cdi->ops->capability & ~cdi->mask & (0x100))) */
/*   return -38; */
/*  if (copy_from_user(&volume, argp, sizeof(volume))) */
/*   return -14; */
/*  return cdi->ops->audio_ioctl(cdi, 0x530a, &volume); */
/* } */

/* static int cdrom_ioctl_volread(struct cdrom_device_info *cdi, */
/*   void *argp) */
/* { */
/*  struct cdrom_volctrl volume; */
/*  int ret; */

/*  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROMVOLREAD\n"); */

/*  if (!(cdi->ops->capability & ~cdi->mask & (0x100))) */
/*   return -38; */

/*  ret = cdi->ops->audio_ioctl(cdi, 0x5313, &volume); */
/*  if (ret) */
/*   return ret; */

/*  if (copy_to_user(argp, &volume, sizeof(volume))) */
/*   return -14; */
/*  return 0; */
/* } */

/* static int cdrom_ioctl_audioctl(struct cdrom_device_info *cdi, */
/*   unsigned int cmd) */
/* { */
/*  int ret; */

/*  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "doing audio ioctl (start/stop/pause/resume)\n"); */

/*  if (!(cdi->ops->capability & ~cdi->mask & (0x100))) */
/*   return -38; */
/*  ret = check_for_audio_disc(cdi, cdi->ops); */
/*  if (ret) */
/*   return ret; */
/*  return cdi->ops->audio_ioctl(cdi, cmd, ((void *)0)); */
/* } */






/* int cdrom_ioctl(struct file * file, struct cdrom_device_info *cdi, */
/*   struct inode *ip, unsigned int cmd, unsigned long arg) */
/* { */
/*  void *argp = (void *)arg; */
/*  int ret; */




/*  ret = scsi_cmd_ioctl(file, ip->i_bdev->bd_disk, cmd, argp); */
/*  if (ret != -25) */
/*   return ret; */

/*  switch (cmd) { */
/*  case 0x5310: */
/*   return cdrom_ioctl_multisession(cdi, argp); */
/*  case 0x5309: */
/*   return cdrom_ioctl_eject(cdi); */
/*  case 0x5319: */
/*   return cdrom_ioctl_closetray(cdi); */
/*  case 0x530f: */
/*   return cdrom_ioctl_eject_sw(cdi, arg); */
/*  case 0x5325: */
/*   return cdrom_ioctl_media_changed(cdi, arg); */
/*  case 0x5320: */
/*   return cdrom_ioctl_set_options(cdi, arg); */
/*  case 0x5321: */
/*   return cdrom_ioctl_clear_options(cdi, arg); */
/*  case 0x5322: */
/*   return cdrom_ioctl_select_speed(cdi, arg); */
/*  case 0x5323: */
/*   return cdrom_ioctl_select_disc(cdi, arg); */
/*  case 0x5312: */
/*   return cdrom_ioctl_reset(cdi, ip->i_bdev); */
/*  case 0x5329: */
/*   return cdrom_ioctl_lock_door(cdi, arg); */
/*  case 0x5330: */
/*   return cdrom_ioctl_debug(cdi, arg); */
/*  case 0x5331: */
/*   return cdrom_ioctl_get_capability(cdi); */
/*  case 0x5311: */
/*   return cdrom_ioctl_get_mcn(cdi, argp); */
/*  case 0x5326: */
/*   return cdrom_ioctl_drive_status(cdi, arg); */
/*  case 0x5327: */
/*   return cdrom_ioctl_disc_status(cdi); */
/*  case 0x5328: */
/*   return cdrom_ioctl_changer_nslots(cdi); */
/*  } */







/*  if ((cdi->ops->capability & ~cdi->mask & (0x1000))) { */
/*   ret = mmc_ioctl(cdi, cmd, arg); */
/*   if (ret != -25) */
/*    return ret; */
/*  } */






/*  switch (cmd) { */
/*  case 0x530b: */
/*   return cdrom_ioctl_get_subchnl(cdi, argp); */
/*  case 0x5305: */
/*   return cdrom_ioctl_read_tochdr(cdi, argp); */
/*  case 0x5306: */
/*   return cdrom_ioctl_read_tocentry(cdi, argp); */
/*  case 0x5303: */
/*   return cdrom_ioctl_play_msf(cdi, argp); */
/*  case 0x5304: */
/*   return cdrom_ioctl_play_trkind(cdi, argp); */
/*  case 0x530a: */
/*   return cdrom_ioctl_volctrl(cdi, argp); */
/*  case 0x5313: */
/*   return cdrom_ioctl_volread(cdi, argp); */
/*  case 0x5308: */
/*  case 0x5307: */
/*  case 0x5301: */
/*  case 0x5302: */
/*   return cdrom_ioctl_audioctl(cdi, cmd); */
/*  } */

/*  return -38; */
/* } */

/* static inline __attribute__((always_inline)) */
/* int msf_to_lba(char m, char s, char f) */
/* { */
/*  return (((m * 60) + s) * 75 + f) - 150; */
/* } */




/* extern int cdrom_switch_blocksize(struct cdrom_device_info *cdi, int size); */

/* static int cdrom_get_track_info(struct cdrom_device_info *cdi, __u16 track, __u8 type, */
/*     track_information *ti) */
/* { */
/*  struct cdrom_device_ops *cdo = cdi->ops; */
/*  struct packet_command cgc; */
/*  int ret, buflen; */

/*  init_cdrom_command(&cgc, ti, 8, 2); */
/*  cgc.cmd[0] = 0x52; */
/*  cgc.cmd[1] = type & 3; */
/*  cgc.cmd[4] = (track & 0xff00) >> 8; */
/*  cgc.cmd[5] = track & 0xff; */
/*  cgc.cmd[8] = 8; */
/*  cgc.quiet = 1; */

/*  if ((ret = cdo->generic_packet(cdi, &cgc))) */
/*   return ret; */

/*  buflen = (__builtin_constant_p((__u16)(( __u16)(__be16)(ti->track_information_length))) ? ({ __u16 __x = ((( __u16)(__be16)(ti->track_information_length))); ((__u16)( (((__u16)(__x) & (__u16)0x00ffU) << 8) | (((__u16)(__x) & (__u16)0xff00U) >> 8) )); }) : __fswab16((( __u16)(__be16)(ti->track_information_length)))) + */
/*        sizeof(ti->track_information_length); */

/*  if (buflen > sizeof(track_information)) */
/*   buflen = sizeof(track_information); */

/*  cgc.cmd[8] = cgc.buflen = buflen; */
/*  if ((ret = cdo->generic_packet(cdi, &cgc))) */
/*   return ret; */


/*  return buflen; */
/* } */


/* static int cdrom_get_disc_info(struct cdrom_device_info *cdi, disc_information *di) */
/* { */
/*  struct cdrom_device_ops *cdo = cdi->ops; */
/*  struct packet_command cgc; */
/*  int ret, buflen; */


/*  init_cdrom_command(&cgc, di, sizeof(*di), 2); */
/*  cgc.cmd[0] = 0x51; */
/*  cgc.cmd[8] = cgc.buflen = 2; */
/*  cgc.quiet = 1; */

/*  if ((ret = cdo->generic_packet(cdi, &cgc))) */
/*   return ret; */




/*  buflen = (__builtin_constant_p((__u16)(( __u16)(__be16)(di->disc_information_length))) ? ({ __u16 __x = ((( __u16)(__be16)(di->disc_information_length))); ((__u16)( (((__u16)(__x) & (__u16)0x00ffU) << 8) | (((__u16)(__x) & (__u16)0xff00U) >> 8) )); }) : __fswab16((( __u16)(__be16)(di->disc_information_length)))) + */
/*        sizeof(di->disc_information_length); */

/*  if (buflen > sizeof(disc_information)) */
/*   buflen = sizeof(disc_information); */

/*  cgc.cmd[8] = cgc.buflen = buflen; */
/*  if ((ret = cdo->generic_packet(cdi, &cgc))) */
/*   return ret; */


/*  return buflen; */
/* } */



/* int cdrom_get_last_written(struct cdrom_device_info *cdi, long *last_written) */
/* { */
/*  struct cdrom_tocentry toc; */
/*  disc_information di; */
/*  track_information ti; */
/*  __u32 last_track; */
/*  int ret = -1, ti_size; */

/*  if (!(cdi->ops->capability & ~cdi->mask & (0x1000))) */
/*   goto use_toc; */

/*  ret = cdrom_get_disc_info(cdi, &di); */
/*  if (ret < (int)(__builtin_offsetof(typeof(di),last_track_lsb) */
/*    + sizeof(di.last_track_lsb))) */
/*   goto use_toc; */


/*  last_track = (di.last_track_msb << 8) | di.last_track_lsb; */
/*  ti_size = cdrom_get_track_info(cdi, last_track, 1, &ti); */
/*  if (ti_size < (int)__builtin_offsetof(typeof(ti),track_start)) */
/*   goto use_toc; */


/*  if (ti.blank) { */
/*   if (last_track==1) */
/*    goto use_toc; */
/*   last_track--; */
/*   ti_size = cdrom_get_track_info(cdi, last_track, 1, &ti); */
/*  } */

/*  if (ti_size < (int)(__builtin_offsetof(typeof(ti),track_size) */
/*     + sizeof(ti.track_size))) */
/*   goto use_toc; */


/*  if (ti.lra_v && ti_size >= (int)(__builtin_offsetof(typeof(ti),last_rec_address) */
/*     + sizeof(ti.last_rec_address))) { */
/*   *last_written = (__builtin_constant_p((__u32)(( __u32)(__be32)(ti.last_rec_address))) ? ({ __u32 __x = ((( __u32)(__be32)(ti.last_rec_address))); ((__u32)( (((__u32)(__x) & (__u32)0x000000ffUL) << 24) | (((__u32)(__x) & (__u32)0x0000ff00UL) << 8) | (((__u32)(__x) & (__u32)0x00ff0000UL) >> 8) | (((__u32)(__x) & (__u32)0xff000000UL) >> 24) )); }) : __fswab32((( __u32)(__be32)(ti.last_rec_address)))); */
/*  } else { */

/*   *last_written = (__builtin_constant_p((__u32)(( __u32)(__be32)(ti.track_start))) ? ({ __u32 __x = ((( __u32)(__be32)(ti.track_start))); ((__u32)( (((__u32)(__x) & (__u32)0x000000ffUL) << 24) | (((__u32)(__x) & (__u32)0x0000ff00UL) << 8) | (((__u32)(__x) & (__u32)0x00ff0000UL) >> 8) | (((__u32)(__x) & (__u32)0xff000000UL) >> 24) )); }) : __fswab32((( __u32)(__be32)(ti.track_start)))) + */
/*     (__builtin_constant_p((__u32)(( __u32)(__be32)(ti.track_size))) ? ({ __u32 __x = ((( __u32)(__be32)(ti.track_size))); ((__u32)( (((__u32)(__x) & (__u32)0x000000ffUL) << 24) | (((__u32)(__x) & (__u32)0x0000ff00UL) << 8) | (((__u32)(__x) & (__u32)0x00ff0000UL) >> 8) | (((__u32)(__x) & (__u32)0xff000000UL) >> 24) )); }) : __fswab32((( __u32)(__be32)(ti.track_size)))); */
/*   if (ti.free_blocks) */
/*    *last_written -= ((__builtin_constant_p((__u32)(( __u32)(__be32)(ti.free_blocks))) ? ({ __u32 __x = ((( __u32)(__be32)(ti.free_blocks))); ((__u32)( (((__u32)(__x) & (__u32)0x000000ffUL) << 24) | (((__u32)(__x) & (__u32)0x0000ff00UL) << 8) | (((__u32)(__x) & (__u32)0x00ff0000UL) >> 8) | (((__u32)(__x) & (__u32)0xff000000UL) >> 24) )); }) : __fswab32((( __u32)(__be32)(ti.free_blocks)))) + 7); */
/*  } */
/*  return 0; */





/* use_toc: */
/*  toc.cdte_format = 0x02; */
/*  toc.cdte_track = 0xAA; */
/*  if ((ret = cdi->ops->audio_ioctl(cdi, 0x5306, &toc))) */
/*   return ret; */
/*  sanitize_format(&toc.cdte_addr, &toc.cdte_format, 0x01); */
/*  *last_written = toc.cdte_addr.lba; */
/*  return 0; */
/* } */


/* static int cdrom_get_next_writable(struct cdrom_device_info *cdi, long *next_writable) */
/* { */
/*  disc_information di; */
/*  track_information ti; */
/*  __u16 last_track; */
/*  int ret, ti_size; */

/*  if (!(cdi->ops->capability & ~cdi->mask & (0x1000))) */
/*   goto use_last_written; */

/*  ret = cdrom_get_disc_info(cdi, &di); */
/*  if (ret < 0 || ret < __builtin_offsetof(typeof(di),last_track_lsb) */
/*     + sizeof(di.last_track_lsb)) */
/*   goto use_last_written; */


/*  last_track = (di.last_track_msb << 8) | di.last_track_lsb; */
/*  ti_size = cdrom_get_track_info(cdi, last_track, 1, &ti); */
/*  if (ti_size < 0 || ti_size < __builtin_offsetof(typeof(ti),track_start)) */
/*   goto use_last_written; */


/*  if (ti.blank) { */
/*   if (last_track == 1) */
/*    goto use_last_written; */
/*   last_track--; */
/*   ti_size = cdrom_get_track_info(cdi, last_track, 1, &ti); */
/*   if (ti_size < 0) */
/*    goto use_last_written; */
/*  } */


/*  if (ti.nwa_v && ti_size >= __builtin_offsetof(typeof(ti),next_writable) */
/*     + sizeof(ti.next_writable)) { */
/*   *next_writable = (__builtin_constant_p((__u32)(( __u32)(__be32)(ti.next_writable))) ? ({ __u32 __x = ((( __u32)(__be32)(ti.next_writable))); ((__u32)( (((__u32)(__x) & (__u32)0x000000ffUL) << 24) | (((__u32)(__x) & (__u32)0x0000ff00UL) << 8) | (((__u32)(__x) & (__u32)0x00ff0000UL) >> 8) | (((__u32)(__x) & (__u32)0xff000000UL) >> 24) )); }) : __fswab32((( __u32)(__be32)(ti.next_writable)))); */
/*   return 0; */
/*  } */

/* use_last_written: */
/*  if ((ret = cdrom_get_last_written(cdi, next_writable))) { */
/*   *next_writable = 0; */
/*   return ret; */
/*  } else { */
/*   *next_writable += 7; */
/*   return 0; */
/*  } */
/* } */

/* extern typeof(cdrom_get_last_written) cdrom_get_last_written; extern void *__crc_cdrom_get_last_written __attribute__((weak)); static const unsigned long __kcrctab_cdrom_get_last_written __attribute__((__used__)) __attribute__((section("__kcrctab" ""), unused)) = (unsigned long) &__crc_cdrom_get_last_written; static const char __kstrtab_cdrom_get_last_written[] __attribute__((section("__ksymtab_strings"))) = "" "cdrom_get_last_written"; static const struct kernel_symbol __ksymtab_cdrom_get_last_written __attribute__((__used__)) __attribute__((section("__ksymtab" ""), unused)) = { (unsigned long)&cdrom_get_last_written, __kstrtab_cdrom_get_last_written }; */
/* extern typeof(register_cdrom) register_cdrom; extern void *__crc_register_cdrom __attribute__((weak)); static const unsigned long __kcrctab_register_cdrom __attribute__((__used__)) __attribute__((section("__kcrctab" ""), unused)) = (unsigned long) &__crc_register_cdrom; static const char __kstrtab_register_cdrom[] __attribute__((section("__ksymtab_strings"))) = "" "register_cdrom"; static const struct kernel_symbol __ksymtab_register_cdrom __attribute__((__used__)) __attribute__((section("__ksymtab" ""), unused)) = { (unsigned long)&register_cdrom, __kstrtab_register_cdrom }; */
/* extern typeof(unregister_cdrom) unregister_cdrom; extern void *__crc_unregister_cdrom __attribute__((weak)); static const unsigned long __kcrctab_unregister_cdrom __attribute__((__used__)) __attribute__((section("__kcrctab" ""), unused)) = (unsigned long) &__crc_unregister_cdrom; static const char __kstrtab_unregister_cdrom[] __attribute__((section("__ksymtab_strings"))) = "" "unregister_cdrom"; static const struct kernel_symbol __ksymtab_unregister_cdrom __attribute__((__used__)) __attribute__((section("__ksymtab" ""), unused)) = { (unsigned long)&unregister_cdrom, __kstrtab_unregister_cdrom }; */
/* extern typeof(cdrom_open) cdrom_open; extern void *__crc_cdrom_open __attribute__((weak)); static const unsigned long __kcrctab_cdrom_open __attribute__((__used__)) __attribute__((section("__kcrctab" ""), unused)) = (unsigned long) &__crc_cdrom_open; static const char __kstrtab_cdrom_open[] __attribute__((section("__ksymtab_strings"))) = "" "cdrom_open"; static const struct kernel_symbol __ksymtab_cdrom_open __attribute__((__used__)) __attribute__((section("__ksymtab" ""), unused)) = { (unsigned long)&cdrom_open, __kstrtab_cdrom_open }; */
/* extern typeof(cdrom_release) cdrom_release; extern void *__crc_cdrom_release __attribute__((weak)); static const unsigned long __kcrctab_cdrom_release __attribute__((__used__)) __attribute__((section("__kcrctab" ""), unused)) = (unsigned long) &__crc_cdrom_release; static const char __kstrtab_cdrom_release[] __attribute__((section("__ksymtab_strings"))) = "" "cdrom_release"; static const struct kernel_symbol __ksymtab_cdrom_release __attribute__((__used__)) __attribute__((section("__ksymtab" ""), unused)) = { (unsigned long)&cdrom_release, __kstrtab_cdrom_release }; */
/* extern typeof(cdrom_ioctl) cdrom_ioctl; extern void *__crc_cdrom_ioctl __attribute__((weak)); static const unsigned long __kcrctab_cdrom_ioctl __attribute__((__used__)) __attribute__((section("__kcrctab" ""), unused)) = (unsigned long) &__crc_cdrom_ioctl; static const char __kstrtab_cdrom_ioctl[] __attribute__((section("__ksymtab_strings"))) = "" "cdrom_ioctl"; static const struct kernel_symbol __ksymtab_cdrom_ioctl __attribute__((__used__)) __attribute__((section("__ksymtab" ""), unused)) = { (unsigned long)&cdrom_ioctl, __kstrtab_cdrom_ioctl }; */
/* extern typeof(cdrom_media_changed) cdrom_media_changed; extern void *__crc_cdrom_media_changed __attribute__((weak)); static const unsigned long __kcrctab_cdrom_media_changed __attribute__((__used__)) __attribute__((section("__kcrctab" ""), unused)) = (unsigned long) &__crc_cdrom_media_changed; static const char __kstrtab_cdrom_media_changed[] __attribute__((section("__ksymtab_strings"))) = "" "cdrom_media_changed"; static const struct kernel_symbol __ksymtab_cdrom_media_changed __attribute__((__used__)) __attribute__((section("__ksymtab" ""), unused)) = { (unsigned long)&cdrom_media_changed, __kstrtab_cdrom_media_changed }; */
/* extern typeof(cdrom_number_of_slots) cdrom_number_of_slots; extern void *__crc_cdrom_number_of_slots __attribute__((weak)); static const unsigned long __kcrctab_cdrom_number_of_slots __attribute__((__used__)) __attribute__((section("__kcrctab" ""), unused)) = (unsigned long) &__crc_cdrom_number_of_slots; static const char __kstrtab_cdrom_number_of_slots[] __attribute__((section("__ksymtab_strings"))) = "" "cdrom_number_of_slots"; static const struct kernel_symbol __ksymtab_cdrom_number_of_slots __attribute__((__used__)) __attribute__((section("__ksymtab" ""), unused)) = { (unsigned long)&cdrom_number_of_slots, __kstrtab_cdrom_number_of_slots }; */
/* extern typeof(cdrom_mode_select) cdrom_mode_select; extern void *__crc_cdrom_mode_select __attribute__((weak)); static const unsigned long __kcrctab_cdrom_mode_select __attribute__((__used__)) __attribute__((section("__kcrctab" ""), unused)) = (unsigned long) &__crc_cdrom_mode_select; static const char __kstrtab_cdrom_mode_select[] __attribute__((section("__ksymtab_strings"))) = "" "cdrom_mode_select"; static const struct kernel_symbol __ksymtab_cdrom_mode_select __attribute__((__used__)) __attribute__((section("__ksymtab" ""), unused)) = { (unsigned long)&cdrom_mode_select, __kstrtab_cdrom_mode_select }; */
/* extern typeof(cdrom_mode_sense) cdrom_mode_sense; extern void *__crc_cdrom_mode_sense __attribute__((weak)); static const unsigned long __kcrctab_cdrom_mode_sense __attribute__((__used__)) __attribute__((section("__kcrctab" ""), unused)) = (unsigned long) &__crc_cdrom_mode_sense; static const char __kstrtab_cdrom_mode_sense[] __attribute__((section("__ksymtab_strings"))) = "" "cdrom_mode_sense"; static const struct kernel_symbol __ksymtab_cdrom_mode_sense __attribute__((__used__)) __attribute__((section("__ksymtab" ""), unused)) = { (unsigned long)&cdrom_mode_sense, __kstrtab_cdrom_mode_sense }; */
/* extern typeof(init_cdrom_command) init_cdrom_command; extern void *__crc_init_cdrom_command __attribute__((weak)); static const unsigned long __kcrctab_init_cdrom_command __attribute__((__used__)) __attribute__((section("__kcrctab" ""), unused)) = (unsigned long) &__crc_init_cdrom_command; static const char __kstrtab_init_cdrom_command[] __attribute__((section("__ksymtab_strings"))) = "" "init_cdrom_command"; static const struct kernel_symbol __ksymtab_init_cdrom_command __attribute__((__used__)) __attribute__((section("__ksymtab" ""), unused)) = { (unsigned long)&init_cdrom_command, __kstrtab_init_cdrom_command }; */
/* extern typeof(cdrom_get_media_event) cdrom_get_media_event; extern void *__crc_cdrom_get_media_event __attribute__((weak)); static const unsigned long __kcrctab_cdrom_get_media_event __attribute__((__used__)) __attribute__((section("__kcrctab" ""), unused)) = (unsigned long) &__crc_cdrom_get_media_event; static const char __kstrtab_cdrom_get_media_event[] __attribute__((section("__ksymtab_strings"))) = "" "cdrom_get_media_event"; static const struct kernel_symbol __ksymtab_cdrom_get_media_event __attribute__((__used__)) __attribute__((section("__ksymtab" ""), unused)) = { (unsigned long)&cdrom_get_media_event, __kstrtab_cdrom_get_media_event }; */





/* static struct cdrom_sysctl_settings { */
/*  char info[1000]; */
/*  int autoclose; */
/*  int autoeject; */
/*  int debug; */
/*  int lock; */
/*  int check; */
/* } cdrom_sysctl_settings; */

/* static int cdrom_sysctl_info(ctl_table *ctl, int write, struct file * filp, */
/*                            void *buffer, size_t *lenp, loff_t *ppos) */
/* { */
/*         int pos; */
/*  struct cdrom_device_info *cdi; */
/*  char *info = cdrom_sysctl_settings.info; */

/*  if (!*lenp || (*ppos && !write)) { */
/*   *lenp = 0; */
/*   return 0; */
/*  } */

/*  pos = sprintf(info, "CD-ROM information, " "Id: cdrom.c 3.20 2003/12/17" "\n"); */

/*  pos += sprintf(info+pos, "\ndrive name:\t"); */
/*  for (cdi=topCdromPtr;cdi!=((void *)0);cdi=cdi->next) */
/*      pos += sprintf(info+pos, "\t%s", cdi->name); */

/*  pos += sprintf(info+pos, "\ndrive speed:\t"); */
/*  for (cdi=topCdromPtr;cdi!=((void *)0);cdi=cdi->next) */
/*      pos += sprintf(info+pos, "\t%d", cdi->speed); */

/*  pos += sprintf(info+pos, "\ndrive # of slots:"); */
/*  for (cdi=topCdromPtr;cdi!=((void *)0);cdi=cdi->next) */
/*      pos += sprintf(info+pos, "\t%d", cdi->capacity); */

/*  pos += sprintf(info+pos, "\nCan close tray:\t"); */
/*  for (cdi=topCdromPtr;cdi!=((void *)0);cdi=cdi->next) */
/*      pos += sprintf(info+pos, "\t%d", (cdi->ops->capability & ~cdi->mask & (0x1)) != 0); */

/*  pos += sprintf(info+pos, "\nCan open tray:\t"); */
/*  for (cdi=topCdromPtr;cdi!=((void *)0);cdi=cdi->next) */
/*      pos += sprintf(info+pos, "\t%d", (cdi->ops->capability & ~cdi->mask & (0x2)) != 0); */

/*  pos += sprintf(info+pos, "\nCan lock tray:\t"); */
/*  for (cdi=topCdromPtr;cdi!=((void *)0);cdi=cdi->next) */
/*      pos += sprintf(info+pos, "\t%d", (cdi->ops->capability & ~cdi->mask & (0x4)) != 0); */

/*  pos += sprintf(info+pos, "\nCan change speed:"); */
/*  for (cdi=topCdromPtr;cdi!=((void *)0);cdi=cdi->next) */
/*      pos += sprintf(info+pos, "\t%d", (cdi->ops->capability & ~cdi->mask & (0x8)) != 0); */

/*  pos += sprintf(info+pos, "\nCan select disk:"); */
/*  for (cdi=topCdromPtr;cdi!=((void *)0);cdi=cdi->next) */
/*      pos += sprintf(info+pos, "\t%d", (cdi->ops->capability & ~cdi->mask & (0x10)) != 0); */

/*  pos += sprintf(info+pos, "\nCan read multisession:"); */
/*  for (cdi=topCdromPtr;cdi!=((void *)0);cdi=cdi->next) */
/*      pos += sprintf(info+pos, "\t%d", (cdi->ops->capability & ~cdi->mask & (0x20)) != 0); */

/*  pos += sprintf(info+pos, "\nCan read MCN:\t"); */
/*  for (cdi=topCdromPtr;cdi!=((void *)0);cdi=cdi->next) */
/*      pos += sprintf(info+pos, "\t%d", (cdi->ops->capability & ~cdi->mask & (0x40)) != 0); */

/*  pos += sprintf(info+pos, "\nReports media changed:"); */
/*  for (cdi=topCdromPtr;cdi!=((void *)0);cdi=cdi->next) */
/*      pos += sprintf(info+pos, "\t%d", (cdi->ops->capability & ~cdi->mask & (0x80)) != 0); */

/*  pos += sprintf(info+pos, "\nCan play audio:\t"); */
/*  for (cdi=topCdromPtr;cdi!=((void *)0);cdi=cdi->next) */
/*      pos += sprintf(info+pos, "\t%d", (cdi->ops->capability & ~cdi->mask & (0x100)) != 0); */

/*  pos += sprintf(info+pos, "\nCan write CD-R:\t"); */
/*  for (cdi=topCdromPtr;cdi!=((void *)0);cdi=cdi->next) */
/*      pos += sprintf(info+pos, "\t%d", (cdi->ops->capability & ~cdi->mask & (0x2000)) != 0); */

/*  pos += sprintf(info+pos, "\nCan write CD-RW:"); */
/*  for (cdi=topCdromPtr;cdi!=((void *)0);cdi=cdi->next) */
/*      pos += sprintf(info+pos, "\t%d", (cdi->ops->capability & ~cdi->mask & (0x4000)) != 0); */

/*  pos += sprintf(info+pos, "\nCan read DVD:\t"); */
/*  for (cdi=topCdromPtr;cdi!=((void *)0);cdi=cdi->next) */
/*      pos += sprintf(info+pos, "\t%d", (cdi->ops->capability & ~cdi->mask & (0x8000)) != 0); */

/*  pos += sprintf(info+pos, "\nCan write DVD-R:"); */
/*  for (cdi=topCdromPtr;cdi!=((void *)0);cdi=cdi->next) */
/*      pos += sprintf(info+pos, "\t%d", (cdi->ops->capability & ~cdi->mask & (0x10000)) != 0); */

/*  pos += sprintf(info+pos, "\nCan write DVD-RAM:"); */
/*  for (cdi=topCdromPtr;cdi!=((void *)0);cdi=cdi->next) */
/*      pos += sprintf(info+pos, "\t%d", (cdi->ops->capability & ~cdi->mask & (0x20000)) != 0); */

/*  pos += sprintf(info+pos, "\nCan read MRW:\t"); */
/*  for (cdi=topCdromPtr;cdi!=((void *)0);cdi=cdi->next) */
/*      pos += sprintf(info+pos, "\t%d", (cdi->ops->capability & ~cdi->mask & (0x80000)) != 0); */

/*  pos += sprintf(info+pos, "\nCan write MRW:\t"); */
/*  for (cdi=topCdromPtr;cdi!=((void *)0);cdi=cdi->next) */
/*      pos += sprintf(info+pos, "\t%d", (cdi->ops->capability & ~cdi->mask & (0x100000)) != 0); */

/*  pos += sprintf(info+pos, "\nCan write RAM:\t"); */
/*  for (cdi=topCdromPtr;cdi!=((void *)0);cdi=cdi->next) */
/*      pos += sprintf(info+pos, "\t%d", (cdi->ops->capability & ~cdi->mask & (0x200000)) != 0); */

/*  strcpy(info+pos,"\n\n"); */

/*         return proc_dostring(ctl, write, filp, buffer, lenp, ppos); */
/* } */





/* static void cdrom_update_settings(void) */
/* { */
/*  struct cdrom_device_info *cdi; */

/*  for (cdi = topCdromPtr; cdi != ((void *)0); cdi = cdi->next) { */
/*   if (autoclose && (cdi->ops->capability & ~cdi->mask & (0x1))) */
/*    cdi->options |= 0x1; */
/*   else if (!autoclose) */
/*    cdi->options &= ~0x1; */
/*   if (autoeject && (cdi->ops->capability & ~cdi->mask & (0x2))) */
/*    cdi->options |= 0x2; */
/*   else if (!autoeject) */
/*    cdi->options &= ~0x2; */
/*   if (lockdoor && (cdi->ops->capability & ~cdi->mask & (0x4))) */
/*    cdi->options |= 0x8; */
/*   else if (!lockdoor) */
/*    cdi->options &= ~0x8; */
/*   if (check_media_type) */
/*    cdi->options |= 0x10; */
/*   else */
/*    cdi->options &= ~0x10; */
/*  } */
/* } */

/* static int cdrom_sysctl_handler(ctl_table *ctl, int write, struct file * filp, */
/*     void *buffer, size_t *lenp, loff_t *ppos) */
/* { */
/*  int *valp = ctl->data; */
/*  int val = *valp; */
/*  int ret; */

/*  ret = proc_dointvec(ctl, write, filp, buffer, lenp, ppos); */

/*  if (write && *valp != val) { */


/*   if (*valp) */
/*    *valp = 1; */
/*   else */
/*    *valp = 0; */

/*   switch (ctl->ctl_name) { */
/*   case DEV_CDROM_AUTOCLOSE: { */
/*    if (valp == &cdrom_sysctl_settings.autoclose) */
/*     autoclose = cdrom_sysctl_settings.autoclose; */
/*    break; */
/*    } */
/*   case DEV_CDROM_AUTOEJECT: { */
/*    if (valp == &cdrom_sysctl_settings.autoeject) */
/*     autoeject = cdrom_sysctl_settings.autoeject; */
/*    break; */
/*    } */
/*   case DEV_CDROM_DEBUG: { */
/*    if (valp == &cdrom_sysctl_settings.debug) */
/*     debug = cdrom_sysctl_settings.debug; */
/*    break; */
/*    } */
/*   case DEV_CDROM_LOCK: { */
/*    if (valp == &cdrom_sysctl_settings.lock) */
/*     lockdoor = cdrom_sysctl_settings.lock; */
/*    break; */
/*    } */
/*   case DEV_CDROM_CHECK_MEDIA: { */
/*    if (valp == &cdrom_sysctl_settings.check) */
/*     check_media_type = cdrom_sysctl_settings.check; */
/*    break; */
/*    } */
/*   } */



/*   cdrom_update_settings(); */
/*  } */

/*         return ret; */
/* } */


/* static ctl_table cdrom_table[] = { */
/*  { */
/*   .ctl_name = DEV_CDROM_INFO, */
/*   .procname = "info", */
/*   .data = &cdrom_sysctl_settings.info, */
/*   .maxlen = 1000, */
/*   .mode = 0444, */
/*   .proc_handler = &cdrom_sysctl_info, */
/*  }, */
/*  { */
/*   .ctl_name = DEV_CDROM_AUTOCLOSE, */
/*   .procname = "autoclose", */
/*   .data = &cdrom_sysctl_settings.autoclose, */
/*   .maxlen = sizeof(int), */
/*   .mode = 0644, */
/*   .proc_handler = &cdrom_sysctl_handler, */
/*  }, */
/*  { */
/*   .ctl_name = DEV_CDROM_AUTOEJECT, */
/*   .procname = "autoeject", */
/*   .data = &cdrom_sysctl_settings.autoeject, */
/*   .maxlen = sizeof(int), */
/*   .mode = 0644, */
/*   .proc_handler = &cdrom_sysctl_handler, */
/*  }, */
/*  { */
/*   .ctl_name = DEV_CDROM_DEBUG, */
/*   .procname = "debug", */
/*   .data = &cdrom_sysctl_settings.debug, */
/*   .maxlen = sizeof(int), */
/*   .mode = 0644, */
/*   .proc_handler = &cdrom_sysctl_handler, */
/*  }, */
/*  { */
/*   .ctl_name = DEV_CDROM_LOCK, */
/*   .procname = "lock", */
/*   .data = &cdrom_sysctl_settings.lock, */
/*   .maxlen = sizeof(int), */
/*   .mode = 0644, */
/*   .proc_handler = &cdrom_sysctl_handler, */
/*  }, */
/*  { */
/*   .ctl_name = DEV_CDROM_CHECK_MEDIA, */
/*   .procname = "check_media", */
/*   .data = &cdrom_sysctl_settings.check, */
/*   .maxlen = sizeof(int), */
/*   .mode = 0644, */
/*   .proc_handler = &cdrom_sysctl_handler */
/*  }, */
/*  { .ctl_name = 0 } */
/* }; */

/* static ctl_table cdrom_cdrom_table[] = { */
/*  { */
/*   .ctl_name = DEV_CDROM, */
/*   .procname = "cdrom", */
/*   .maxlen = 0, */
/*   .mode = 0555, */
/*   .child = cdrom_table, */
/*  }, */
/*  { .ctl_name = 0 } */
/* }; */


/* static ctl_table cdrom_root_table[] = { */
/*  { */
/*   .ctl_name = CTL_DEV, */
/*   .procname = "dev", */
/*   .maxlen = 0, */
/*   .mode = 0555, */
/*   .child = cdrom_cdrom_table, */
/*  }, */
/*  { .ctl_name = 0 } */
/* }; */
/* static struct ctl_table_header *cdrom_sysctl_header; */

/* static void cdrom_sysctl_register(void) */
/* { */
/*  static int initialized; */

/*  if (initialized == 1) */
/*   return; */

/*  cdrom_sysctl_header = register_sysctl_table(cdrom_root_table, 1); */
/*  if (cdrom_root_table->ctl_name && cdrom_root_table->child->de) */
/*   cdrom_root_table->child->de->owner = (&__this_module); */


/*  cdrom_sysctl_settings.autoclose = autoclose; */
/*  cdrom_sysctl_settings.autoeject = autoeject; */
/*  cdrom_sysctl_settings.debug = debug; */
/*  cdrom_sysctl_settings.lock = lockdoor; */
/*  cdrom_sysctl_settings.check = check_media_type; */

/*  initialized = 1; */
/* } */

/* static void cdrom_sysctl_unregister(void) */
/* { */
/*  if (cdrom_sysctl_header) */
/*   unregister_sysctl_table(cdrom_sysctl_header); */
/* } */



/* static int __attribute__ ((__section__ (".init.text"))) cdrom_init(void) */
/* { */

/*  cdrom_sysctl_register(); */

/*  return 0; */
/* } */

/* static void __attribute__ ((__section__(".exit.text"))) cdrom_exit(void) */
/* { */
/*  printk("<6>" "Uniform CD-ROM driver unloaded\n"); */

/*  cdrom_sysctl_unregister(); */

/* } */

/* static inline __attribute__((always_inline)) initcall_t __inittest(void) { return cdrom_init; } int init_module(void) __attribute__((alias("cdrom_init")));; */
/* static inline __attribute__((always_inline)) exitcall_t __exittest(void) { return cdrom_exit; } void cleanup_module(void) __attribute__((alias("cdrom_exit")));; */
/* static const char __mod_license3593[] __attribute__((__used__)) __attribute__((section(".modinfo"),unused)) = "license" "=" "GPL"; */

/*
// [kohei]
static inline __attribute__((always_inline)) __attribute__((always_inline)) void * __constant_memcpy(void * to, const void * from, size_t n)
{
 long esi, edi;
 if (!n) return to;

 switch (n) {
  case 1: *(char*)to = *(char*)from; return to;
  case 2: *(short*)to = *(short*)from; return to;
  case 4: *(int*)to = *(int*)from; return to;

  case 3: *(short*)to = *(short*)from;
   *((char*)to+2) = *((char*)from+2); return to;
  case 5: *(int*)to = *(int*)from;
   *((char*)to+4) = *((char*)from+4); return to;
  case 6: *(int*)to = *(int*)from;
   *((short*)to+2) = *((short*)from+2); return to;
  case 8: *(int*)to = *(int*)from;
   *((int*)to+1) = *((int*)from+1); return to;

 }

 esi = (long) from;
 edi = (long) to;
 if (n >= 5*4) {

  int ecx;
  __asm__ __volatile__(
   "rep ; movsl"
   : "=&c" (ecx), "=&D" (edi), "=&S" (esi)
   : "0" (n/4), "1" (edi),"2" (esi)
   : "memory"
  );
 } else {

  if (n >= 4*4) __asm__ __volatile__("movsl"
   :"=&D"(edi),"=&S"(esi):"0"(edi),"1"(esi):"memory");
  if (n >= 3*4) __asm__ __volatile__("movsl"
   :"=&D"(edi),"=&S"(esi):"0"(edi),"1"(esi):"memory");
  if (n >= 2*4) __asm__ __volatile__("movsl"
   :"=&D"(edi),"=&S"(esi):"0"(edi),"1"(esi):"memory");
  if (n >= 1*4) __asm__ __volatile__("movsl"
   :"=&D"(edi),"=&S"(esi):"0"(edi),"1"(esi):"memory");
 }
 switch (n % 4) {

  case 0: return to;
  case 1: __asm__ __volatile__("movsb"
   :"=&D"(edi),"=&S"(esi):"0"(edi),"1"(esi):"memory");
   return to;
  case 2: __asm__ __volatile__("movsw"
   :"=&D"(edi),"=&S"(esi):"0"(edi),"1"(esi):"memory");
   return to;
  default: __asm__ __volatile__("movsw\n\tmovsb"
   :"=&D"(edi),"=&S"(esi):"0"(edi),"1"(esi):"memory");
   return to;
 }
}

// [kohei]
static inline __attribute__((always_inline)) struct thread_info *current_thread_info(void)
{
 return (struct thread_info *)(current_stack_pointer & ~((4096) - 1));
}

// [kohei]
int cdrom_get_media_event(struct cdrom_device_info *cdi,
     struct media_event_desc *med)
{
 struct packet_command cgc;
 unsigned char buffer[8];
 struct event_header *eh = (struct event_header *) buffer;

 init_cdrom_command(&cgc, buffer, sizeof(buffer), 2);
 cgc.cmd[0] = 0x4a;
 cgc.cmd[1] = 1;
 cgc.cmd[4] = 1 << 4;
 cgc.cmd[8] = sizeof(buffer);
 cgc.quiet = 1;

 if (cdi->ops->generic_packet(cdi, &cgc))
  return 1;

 if ((__builtin_constant_p((__u16)(( __u16)(__be16)(eh->data_len))) ? ({ __u16 __x = ((( __u16)(__be16)(eh->data_len))); ((__u16)( (((__u16)(__x) & (__u16)0x00ffU) << 8) | (((__u16)(__x) & (__u16)0xff00U) >> 8) )); }) : __fswab16((( __u16)(__be16)(eh->data_len)))) < sizeof(*med))
  return 1;

 if (eh->nea || eh->notification_class != 0x4)
  return 1;

 (__builtin_constant_p(sizeof(*med)) ? __constant_memcpy((med),(&buffer[sizeof(*eh)]),(sizeof(*med))) : __memcpy((med),(&buffer[sizeof(*eh)]),(sizeof(*med))));
 return 0;
}

// [kohei]
static int cdrom_is_mrw(struct cdrom_device_info *cdi, int *write)
{
 struct packet_command cgc;
 struct mrw_feature_desc *mfd;
 unsigned char buffer[16];
 int ret;

 *write = 0;

 init_cdrom_command(&cgc, buffer, sizeof(buffer), 2);

 cgc.cmd[0] = 0x46;
 cgc.cmd[3] = 0x0028;
 cgc.cmd[8] = sizeof(buffer);
 cgc.quiet = 1;

 if ((ret = cdi->ops->generic_packet(cdi, &cgc)))
  return ret;

 mfd = (struct mrw_feature_desc *)&buffer[sizeof(struct feature_header)];
 if ((__builtin_constant_p((__u16)(( __u16)(__be16)(mfd->feature_code))) ? ({ __u16 __x = ((( __u16)(__be16)(mfd->feature_code))); ((__u16)( (((__u16)(__x) & (__u16)0x00ffU) << 8) | (((__u16)(__x) & (__u16)0xff00U) >> 8) )); }) : __fswab16((( __u16)(__be16)(mfd->feature_code)))) != 0x0028)
  return 1;
 *write = mfd->write;

 if ((ret = cdrom_mrw_probe_pc(cdi))) {
  *write = 0;
  return ret;
 }

 return 0;
}

// [kohei]
static int cdrom_mrw_set_lba_space(struct cdrom_device_info *cdi, int space)
{
 struct packet_command cgc;
 struct mode_page_header *mph;
 char buffer[16];
 int ret, offset, size;

 init_cdrom_command(&cgc, buffer, sizeof(buffer), 2);

 cgc.buffer = buffer;
 cgc.buflen = sizeof(buffer);

 if ((ret = cdrom_mode_sense(cdi, &cgc, cdi->mrw_mode_page, 0)))
  return ret;

 mph = (struct mode_page_header *) buffer;
 offset = (__builtin_constant_p((__u16)(( __u16)(__be16)(mph->desc_length))) ? ({ __u16 __x = ((( __u16)(__be16)(mph->desc_length))); ((__u16)( (((__u16)(__x) & (__u16)0x00ffU) << 8) | (((__u16)(__x) & (__u16)0xff00U) >> 8) )); }) : __fswab16((( __u16)(__be16)(mph->desc_length))));
 size = (__builtin_constant_p((__u16)(( __u16)(__be16)(mph->mode_data_length))) ? ({ __u16 __x = ((( __u16)(__be16)(mph->mode_data_length))); ((__u16)( (((__u16)(__x) & (__u16)0x00ffU) << 8) | (((__u16)(__x) & (__u16)0xff00U) >> 8) )); }) : __fswab16((( __u16)(__be16)(mph->mode_data_length)))) + 2;

 buffer[offset + 3] = space;
 cgc.buflen = size;

 if ((ret = cdrom_mode_select(cdi, &cgc)))
  return ret;

 printk("<6>" "cdrom: %s: mrw address space %s selected\n", cdi->name, mrw_address_space[space]);
 return 0;
}

// [kohei]
static int cdrom_load_unload(struct cdrom_device_info *cdi, int slot)
{
 struct packet_command cgc;

 if ((0x1 & 0x40) || debug==1 ) printk("<6>" "cdrom: " "entering cdrom_load_unload()\n");
 if (cdi->sanyo_slot && slot < 0)
  return 0;

 init_cdrom_command(&cgc, ((void *)0), 0, 3);
 cgc.cmd[0] = 0xa6;
 cgc.cmd[4] = 2 + (slot >= 0);
 cgc.cmd[8] = slot;
 cgc.timeout = 60 * 1000;




 if (cdi->sanyo_slot && -1 < slot) {
  cgc.cmd[0] = 0x00;
  cgc.cmd[7] = slot;
  cgc.cmd[4] = cgc.cmd[8] = 0;
  cdi->sanyo_slot = slot ? slot : 3;
 }

 return cdi->ops->generic_packet(cdi, &cgc);
}

// [kohei]
static int dvd_do_auth(struct cdrom_device_info *cdi, dvd_authinfo *ai)
{
 int ret;
 u_char buf[20];
 struct packet_command cgc;
 struct cdrom_device_ops *cdo = cdi->ops;
 rpc_state_t rpc_state;

 (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(buf))) ? __constant_c_and_count_memset(((buf)),((0x01010101UL*(unsigned char)(0))),((sizeof(buf)))) : __constant_c_memset(((buf)),((0x01010101UL*(unsigned char)(0))),((sizeof(buf))))) : (__builtin_constant_p((sizeof(buf))) ? __memset_generic((((buf))),(((0))),(((sizeof(buf))))) : __memset_generic(((buf)),((0)),((sizeof(buf))))));
 init_cdrom_command(&cgc, buf, 0, 2);

 switch (ai->type) {

 case 0:
  if ((0x1 & 0x80) || debug==1 ) printk("<6>" "cdrom: " "entering DVD_LU_SEND_AGID\n");
  cgc.quiet = 1;
  setup_report_key(&cgc, ai->lsa.agid, 0);

  if ((ret = cdo->generic_packet(cdi, &cgc)))
   return ret;

  ai->lsa.agid = buf[7] >> 6;

  break;

 case 2:
  if ((0x1 & 0x80) || debug==1 ) printk("<6>" "cdrom: " "entering DVD_LU_SEND_KEY1\n");
  setup_report_key(&cgc, ai->lsk.agid, 2);

  if ((ret = cdo->generic_packet(cdi, &cgc)))
   return ret;

  (__builtin_constant_p(sizeof(dvd_key)) ? __constant_memcpy(((ai->lsk.key)),((&buf[4])),(sizeof(dvd_key))) : __memcpy(((ai->lsk.key)),((&buf[4])),(sizeof(dvd_key))));

  break;

 case 3:
  if ((0x1 & 0x80) || debug==1 ) printk("<6>" "cdrom: " "entering DVD_LU_SEND_CHALLENGE\n");
  setup_report_key(&cgc, ai->lsc.agid, 1);

  if ((ret = cdo->generic_packet(cdi, &cgc)))
   return ret;

  (__builtin_constant_p(sizeof(dvd_challenge)) ? __constant_memcpy(((ai->lsc.chal)),((&buf[4])),(sizeof(dvd_challenge))) : __memcpy(((ai->lsc.chal)),((&buf[4])),(sizeof(dvd_challenge))));

  break;


 case 7:
  if ((0x1 & 0x80) || debug==1 ) printk("<6>" "cdrom: " "entering DVD_LU_SEND_TITLE_KEY\n");
  cgc.quiet = 1;
  setup_report_key(&cgc, ai->lstk.agid, 4);
  cgc.cmd[5] = ai->lstk.lba;
  cgc.cmd[4] = ai->lstk.lba >> 8;
  cgc.cmd[3] = ai->lstk.lba >> 16;
  cgc.cmd[2] = ai->lstk.lba >> 24;

  if ((ret = cdo->generic_packet(cdi, &cgc)))
   return ret;

  ai->lstk.cpm = (buf[4] >> 7) & 1;
  ai->lstk.cp_sec = (buf[4] >> 6) & 1;
  ai->lstk.cgms = (buf[4] >> 4) & 3;
  (__builtin_constant_p(sizeof(dvd_key)) ? __constant_memcpy(((ai->lstk.title_key)),((&buf[5])),(sizeof(dvd_key))) : __memcpy(((ai->lstk.title_key)),((&buf[5])),(sizeof(dvd_key))));

  break;

 case 8:
  if ((0x1 & 0x80) || debug==1 ) printk("<6>" "cdrom: " "entering DVD_LU_SEND_ASF\n");
  setup_report_key(&cgc, ai->lsasf.agid, 5);

  if ((ret = cdo->generic_packet(cdi, &cgc)))
   return ret;

  ai->lsasf.asf = buf[7] & 1;
  break;


 case 1:
  if ((0x1 & 0x80) || debug==1 ) printk("<6>" "cdrom: " "entering DVD_HOST_SEND_CHALLENGE\n");
  setup_send_key(&cgc, ai->hsc.agid, 1);
  buf[1] = 0xe;
  (__builtin_constant_p(sizeof(dvd_challenge)) ? __constant_memcpy(((&buf[4])),((ai->hsc.chal)),(sizeof(dvd_challenge))) : __memcpy(((&buf[4])),((ai->hsc.chal)),(sizeof(dvd_challenge))));

  if ((ret = cdo->generic_packet(cdi, &cgc)))
   return ret;

  ai->type = 2;
  break;

 case 4:
  if ((0x1 & 0x80) || debug==1 ) printk("<6>" "cdrom: " "entering DVD_HOST_SEND_KEY2\n");
  setup_send_key(&cgc, ai->hsk.agid, 3);
  buf[1] = 0xa;
  (__builtin_constant_p(sizeof(dvd_key)) ? __constant_memcpy(((&buf[4])),((ai->hsk.key)),(sizeof(dvd_key))) : __memcpy(((&buf[4])),((ai->hsk.key)),(sizeof(dvd_key))));

  if ((ret = cdo->generic_packet(cdi, &cgc))) {
   ai->type = 6;
   return ret;
  }
  ai->type = 5;
  break;


 case 9:
  cgc.quiet = 1;
  if ((0x1 & 0x80) || debug==1 ) printk("<6>" "cdrom: " "entering DVD_INVALIDATE_AGID\n");
  setup_report_key(&cgc, ai->lsa.agid, 0x3f);
  if ((ret = cdo->generic_packet(cdi, &cgc)))
   return ret;
  break;


 case 10:
  if ((0x1 & 0x80) || debug==1 ) printk("<6>" "cdrom: " "entering DVD_LU_SEND_RPC_STATE\n");
  setup_report_key(&cgc, 0, 8);
  (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(rpc_state_t))) ? __constant_c_and_count_memset(((&rpc_state)),((0x01010101UL*(unsigned char)(0))),((sizeof(rpc_state_t)))) : __constant_c_memset(((&rpc_state)),((0x01010101UL*(unsigned char)(0))),((sizeof(rpc_state_t))))) : (__builtin_constant_p((sizeof(rpc_state_t))) ? __memset_generic((((&rpc_state))),(((0))),(((sizeof(rpc_state_t))))) : __memset_generic(((&rpc_state)),((0)),((sizeof(rpc_state_t))))));
  cgc.buffer = (char *) &rpc_state;

  if ((ret = cdo->generic_packet(cdi, &cgc)))
   return ret;

  ai->lrpcs.type = rpc_state.type_code;
  ai->lrpcs.vra = rpc_state.vra;
  ai->lrpcs.ucca = rpc_state.ucca;
  ai->lrpcs.region_mask = rpc_state.region_mask;
  ai->lrpcs.rpc_scheme = rpc_state.rpc_scheme;
  break;


 case 11:
  if ((0x1 & 0x80) || debug==1 ) printk("<6>" "cdrom: " "entering DVD_HOST_SEND_RPC_STATE\n");
  setup_send_key(&cgc, 0, 6);
  buf[1] = 6;
  buf[4] = ai->hrpcs.pdrc;

  if ((ret = cdo->generic_packet(cdi, &cgc)))
   return ret;
  break;

 default:
  if ((0x1 & 0x1) || debug==1 ) printk("<6>" "cdrom: " "Invalid DVD key ioctl (%d)\n", ai->type);
  return -25;
 }

 return 0;
}

// [kohei]
static int cdrom_switch_blocksize(struct cdrom_device_info *cdi, int size)
{
 struct cdrom_device_ops *cdo = cdi->ops;
 struct packet_command cgc;
 struct modesel_head mh;

 (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(mh))) ? __constant_c_and_count_memset(((&mh)),((0x01010101UL*(unsigned char)(0))),((sizeof(mh)))) : __constant_c_memset(((&mh)),((0x01010101UL*(unsigned char)(0))),((sizeof(mh))))) : (__builtin_constant_p((sizeof(mh))) ? __memset_generic((((&mh))),(((0))),(((sizeof(mh))))) : __memset_generic(((&mh)),((0)),((sizeof(mh))))));
 mh.block_desc_length = 0x08;
 mh.block_length_med = (size >> 8) & 0xff;
 mh.block_length_lo = size & 0xff;

 (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(cgc))) ? __constant_c_and_count_memset(((&cgc)),((0x01010101UL*(unsigned char)(0))),((sizeof(cgc)))) : __constant_c_memset(((&cgc)),((0x01010101UL*(unsigned char)(0))),((sizeof(cgc))))) : (__builtin_constant_p((sizeof(cgc))) ? __memset_generic((((&cgc))),(((0))),(((sizeof(cgc))))) : __memset_generic(((&cgc)),((0)),((sizeof(cgc))))));
 cgc.cmd[0] = 0x15;
 cgc.cmd[1] = 1 << 4;
 cgc.cmd[4] = 12;
 cgc.buflen = sizeof(mh);
 cgc.buffer = (char *) &mh;
 cgc.data_direction = 1;
 mh.block_desc_length = 0x08;
 mh.block_length_med = (size >> 8) & 0xff;
 mh.block_length_lo = size & 0xff;

 return cdo->generic_packet(cdi, &cgc);
}

extern int mmc_ioctl(struct cdrom_device_info *cdi, unsigned int cmd, unsigned long arg);
// [kohei]
static int mmc_ioctl(struct cdrom_device_info *cdi, unsigned int cmd,
       unsigned long arg)
{
 struct cdrom_device_ops *cdo = cdi->ops;
 struct packet_command cgc;
 struct request_sense sense;
 unsigned char buffer[32];
 int ret = 0;

 (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(cgc))) ? __constant_c_and_count_memset(((&cgc)),((0x01010101UL*(unsigned char)(0))),((sizeof(cgc)))) : __constant_c_memset(((&cgc)),((0x01010101UL*(unsigned char)(0))),((sizeof(cgc))))) : (__builtin_constant_p((sizeof(cgc))) ? __memset_generic((((&cgc))),(((0))),(((sizeof(cgc))))) : __memset_generic(((&cgc)),((0)),((sizeof(cgc))))));



 switch (cmd) {
 case 0x5314:
 case 0x530d:
 case 0x530c: {
  struct cdrom_msf msf;
  int blocksize = 0, format = 0, lba;

  switch (cmd) {
  case 0x5314:
   blocksize = 2352;
   break;
  case 0x530d:
   blocksize = 2048;
   format = 2;
   break;
  case 0x530c:
   blocksize = (2352 -12 -4);
   break;
  }
  if (copy_from_user(&(msf), (struct cdrom_msf *) (arg), sizeof (msf))) return -14;;
  lba = msf_to_lba(msf.cdmsf_min0,msf.cdmsf_sec0,msf.cdmsf_frame0);

  if (lba < 0)
   return -22;
  cgc.buffer = (char *) kmalloc(blocksize, ((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u)));
  if (cgc.buffer == ((void *)0))
   return -12;
  (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(sense))) ? __constant_c_and_count_memset(((&sense)),((0x01010101UL*(unsigned char)(0))),((sizeof(sense)))) : __constant_c_memset(((&sense)),((0x01010101UL*(unsigned char)(0))),((sizeof(sense))))) : (__builtin_constant_p((sizeof(sense))) ? __memset_generic((((&sense))),(((0))),(((sizeof(sense))))) : __memset_generic(((&sense)),((0)),((sizeof(sense))))));
  cgc.sense = &sense;
  cgc.data_direction = 2;
  ret = cdrom_read_block(cdi, &cgc, lba, 1, format, blocksize);
  if (ret && sense.sense_key==0x05 && sense.asc==0x20 && sense.ascq==0x00) {





   if ((ret = cdrom_switch_blocksize(cdi, blocksize))) {
    kfree(cgc.buffer);
    return ret;
   }
   cgc.sense = ((void *)0);
   ret = cdrom_read_cd(cdi, &cgc, lba, blocksize, 1);
   ret |= cdrom_switch_blocksize(cdi, blocksize);
  }
  if (!ret && copy_to_user((char *)arg, cgc.buffer, blocksize))
   ret = -14;
  kfree(cgc.buffer);
  return ret;
  }
 case 0x530e: {
  struct cdrom_read_audio ra;
  int lba;

  if (copy_from_user(&(ra), (struct cdrom_read_audio *) (arg), sizeof (ra))) return -14;;

  if (ra.addr_format == 0x02)
   lba = msf_to_lba(ra.addr.msf.minute,
      ra.addr.msf.second,
      ra.addr.msf.frame);
  else if (ra.addr_format == 0x01)
   lba = ra.addr.lba;
  else
   return -22;


  if (lba < 0 || ra.nframes <= 0 || ra.nframes > 75)
   return -22;

  return cdrom_read_cdda(cdi, ra.buf, lba, ra.nframes);
  }
 case 0x530b: {
  struct cdrom_subchnl q;
  u_char requested, back;
  if (copy_from_user(&(q), (struct cdrom_subchnl *) (arg), sizeof (q))) return -14;;
  requested = q.cdsc_format;
  if (!((requested == 0x02) ||
        (requested == 0x01)))
   return -22;
  q.cdsc_format = 0x02;
  if ((ret = cdrom_read_subchannel(cdi, &q, 0)))
   return ret;
  back = q.cdsc_format;
  sanitize_format(&q.cdsc_absaddr, &back, requested);
  sanitize_format(&q.cdsc_reladdr, &q.cdsc_format, requested);
  if (copy_to_user((struct cdrom_subchnl *) (arg), &(q), sizeof (q))) return -14;;

  return 0;
  }
 case 0x5303: {
  struct cdrom_msf msf;
  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROMPLAYMSF\n");
  if (copy_from_user(&(msf), (struct cdrom_msf *) (arg), sizeof (msf))) return -14;;
  cgc.cmd[0] = 0x47;
  cgc.cmd[3] = msf.cdmsf_min0;
  cgc.cmd[4] = msf.cdmsf_sec0;
  cgc.cmd[5] = msf.cdmsf_frame0;
  cgc.cmd[6] = msf.cdmsf_min1;
  cgc.cmd[7] = msf.cdmsf_sec1;
  cgc.cmd[8] = msf.cdmsf_frame1;
  cgc.data_direction = 3;
  return cdo->generic_packet(cdi, &cgc);
  }
 case 0x5317: {
  struct cdrom_blk blk;
  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROMPLAYBLK\n");
  if (copy_from_user(&(blk), (struct cdrom_blk *) (arg), sizeof (blk))) return -14;;
  cgc.cmd[0] = 0x45;
  cgc.cmd[2] = (blk.from >> 24) & 0xff;
  cgc.cmd[3] = (blk.from >> 16) & 0xff;
  cgc.cmd[4] = (blk.from >> 8) & 0xff;
  cgc.cmd[5] = blk.from & 0xff;
  cgc.cmd[7] = (blk.len >> 8) & 0xff;
  cgc.cmd[8] = blk.len & 0xff;
  cgc.data_direction = 3;
  return cdo->generic_packet(cdi, &cgc);
  }
 case 0x530a:
 case 0x5313: {
  struct cdrom_volctrl volctrl;
  char mask[sizeof(buffer)];
  unsigned short offset;

  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROMVOLUME\n");

  if (copy_from_user(&(volctrl), (struct cdrom_volctrl *) (arg), sizeof (volctrl))) return -14;;

  cgc.buffer = buffer;
  cgc.buflen = 24;
  if ((ret = cdrom_mode_sense(cdi, &cgc, 0x0e, 0)))
      return ret;





  offset = 8 + (__builtin_constant_p((__u16)(( __u16)(__be16)(*(unsigned short *)(buffer+6)))) ? ({ __u16 __x = ((( __u16)(__be16)(*(unsigned short *)(buffer+6)))); ((__u16)( (((__u16)(__x) & (__u16)0x00ffU) << 8) | (((__u16)(__x) & (__u16)0xff00U) >> 8) )); }) : __fswab16((( __u16)(__be16)(*(unsigned short *)(buffer+6)))));

  if (offset + 16 > sizeof(buffer))
   return -7;

  if (offset + 16 > cgc.buflen) {
   cgc.buflen = offset+16;
   ret = cdrom_mode_sense(cdi, &cgc,
      0x0e, 0);
   if (ret)
    return ret;
  }


  if ((buffer[offset] & 0x3f) != 0x0e ||
    buffer[offset+1] < 14)
   return -22;



  if (cmd == 0x5313) {
   volctrl.channel0 = buffer[offset+9];
   volctrl.channel1 = buffer[offset+11];
   volctrl.channel2 = buffer[offset+13];
   volctrl.channel3 = buffer[offset+15];
   if (copy_to_user((struct cdrom_volctrl *) (arg), &(volctrl), sizeof (volctrl))) return -14;;
   return 0;
  }


  cgc.buffer = mask;
  if ((ret = cdrom_mode_sense(cdi, &cgc,
    0x0e, 1)))
   return ret;

  buffer[offset+9] = volctrl.channel0 & mask[offset+9];
  buffer[offset+11] = volctrl.channel1 & mask[offset+11];
  buffer[offset+13] = volctrl.channel2 & mask[offset+13];
  buffer[offset+15] = volctrl.channel3 & mask[offset+15];


  cgc.buffer = buffer + offset - 8;
  (__builtin_constant_p(0) ? (__builtin_constant_p((8)) ? __constant_c_and_count_memset(((cgc.buffer)),((0x01010101UL*(unsigned char)(0))),((8))) : __constant_c_memset(((cgc.buffer)),((0x01010101UL*(unsigned char)(0))),((8)))) : (__builtin_constant_p((8)) ? __memset_generic((((cgc.buffer))),(((0))),(((8)))) : __memset_generic(((cgc.buffer)),((0)),((8)))));
  return cdrom_mode_select(cdi, &cgc);
  }

 case 0x5308:
 case 0x5307: {
  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROMSTART/CDROMSTOP\n");
  cgc.cmd[0] = 0x1b;
  cgc.cmd[1] = 1;
  cgc.cmd[4] = (cmd == 0x5308) ? 1 : 0;
  cgc.data_direction = 3;
  return cdo->generic_packet(cdi, &cgc);
  }

 case 0x5301:
 case 0x5302: {
  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROMPAUSE/CDROMRESUME\n");
  cgc.cmd[0] = 0x4b;
  cgc.cmd[8] = (cmd == 0x5302) ? 1 : 0;
  cgc.data_direction = 3;
  return cdo->generic_packet(cdi, &cgc);
  }

 case 0x5390: {
  dvd_struct *s;
  int size = sizeof(dvd_struct);
  if (!(cdi->ops->capability & ~cdi->mask & (0x8000)))
   return -38;
  if ((s = (dvd_struct *) kmalloc(size, ((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u)))) == ((void *)0))
   return -12;
  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering DVD_READ_STRUCT\n");
  if (copy_from_user(s, (dvd_struct *)arg, size)) {
   kfree(s);
   return -14;
  }
  if ((ret = dvd_read_struct(cdi, s))) {
   kfree(s);
   return ret;
  }
  if (copy_to_user((dvd_struct *)arg, s, size))
   ret = -14;
  kfree(s);
  return ret;
  }

 case 0x5392: {
  dvd_authinfo ai;
  if (!(cdi->ops->capability & ~cdi->mask & (0x8000)))
   return -38;
  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering DVD_AUTH\n");
  if (copy_from_user(&(ai), (dvd_authinfo *) (arg), sizeof (ai))) return -14;;
  if ((ret = dvd_do_auth (cdi, &ai)))
   return ret;
  if (copy_to_user((dvd_authinfo *) (arg), &(ai), sizeof (ai))) return -14;;
  return 0;
  }

 case 0x5394: {
  long next = 0;
  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROM_NEXT_WRITABLE\n");
  if ((ret = cdrom_get_next_writable(cdi, &next)))
   return ret;
  if (copy_to_user((long *) (arg), &(next), sizeof (next))) return -14;;
  return 0;
  }
 case 0x5395: {
  long last = 0;
  if ((0x1 & 0x4) || debug==1 ) printk("<6>" "cdrom: " "entering CDROM_LAST_WRITTEN\n");
  if ((ret = cdrom_get_last_written(cdi, &last)))
   return ret;
  if (copy_to_user((long *) (arg), &(last), sizeof (last))) return -14;;
  return 0;
  }
 }

 return -25;
}
*/
