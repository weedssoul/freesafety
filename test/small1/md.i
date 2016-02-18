





















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






typedef __signed__ char __s8;
typedef unsigned char __u8;

typedef __signed__ short __s16;
typedef unsigned short __u16;

typedef __signed__ int __s32;
typedef unsigned int __u32;


typedef __signed__ long long __s64;
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
/* [kohei]
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
*/
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


static inline __attribute__((always_inline)) void __cpus_clear(cpumask_t *dstp, int nbits)
{
 bitmap_zero(dstp->bits, nbits);
}





static inline __attribute__((always_inline)) int __cpu_test_and_set(int cpu, cpumask_t *addr)
{
 return test_and_set_bit(cpu, addr->bits);
}


static inline __attribute__((always_inline)) void __cpus_and(cpumask_t *dstp, const cpumask_t *src1p,
     const cpumask_t *src2p, int nbits)
{
 bitmap_and(dstp->bits, src1p->bits, src2p->bits, nbits);
}


static inline __attribute__((always_inline)) void __cpus_or(cpumask_t *dstp, const cpumask_t *src1p,
     const cpumask_t *src2p, int nbits)
{
 bitmap_or(dstp->bits, src1p->bits, src2p->bits, nbits);
}


static inline __attribute__((always_inline)) void __cpus_xor(cpumask_t *dstp, const cpumask_t *src1p,
     const cpumask_t *src2p, int nbits)
{
 bitmap_xor(dstp->bits, src1p->bits, src2p->bits, nbits);
}



static inline __attribute__((always_inline)) void __cpus_andnot(cpumask_t *dstp, const cpumask_t *src1p,
     const cpumask_t *src2p, int nbits)
{
 bitmap_andnot(dstp->bits, src1p->bits, src2p->bits, nbits);
}


static inline __attribute__((always_inline)) void __cpus_complement(cpumask_t *dstp,
     const cpumask_t *srcp, int nbits)
{
 bitmap_complement(dstp->bits, srcp->bits, nbits);
}


static inline __attribute__((always_inline)) int __cpus_equal(const cpumask_t *src1p,
     const cpumask_t *src2p, int nbits)
{
 return bitmap_equal(src1p->bits, src2p->bits, nbits);
}


static inline __attribute__((always_inline)) int __cpus_intersects(const cpumask_t *src1p,
     const cpumask_t *src2p, int nbits)
{
 return bitmap_intersects(src1p->bits, src2p->bits, nbits);
}


static inline __attribute__((always_inline)) int __cpus_subset(const cpumask_t *src1p,
     const cpumask_t *src2p, int nbits)
{
 return bitmap_subset(src1p->bits, src2p->bits, nbits);
}


static inline __attribute__((always_inline)) int __cpus_empty(const cpumask_t *srcp, int nbits)
{
 return bitmap_empty(srcp->bits, nbits);
}


static inline __attribute__((always_inline)) int __cpus_full(const cpumask_t *srcp, int nbits)
{
 return bitmap_full(srcp->bits, nbits);
}


static inline __attribute__((always_inline)) int __cpus_weight(const cpumask_t *srcp, int nbits)
{
 return bitmap_weight(srcp->bits, nbits);
}



static inline __attribute__((always_inline)) void __cpus_shift_right(cpumask_t *dstp,
     const cpumask_t *srcp, int n, int nbits)
{
 bitmap_shift_right(dstp->bits, srcp->bits, n, nbits);
}



static inline __attribute__((always_inline)) void __cpus_shift_left(cpumask_t *dstp,
     const cpumask_t *srcp, int n, int nbits)
{
 bitmap_shift_left(dstp->bits, srcp->bits, n, nbits);
}

static inline __attribute__((always_inline)) int __cpumask_scnprintf(char *buf, int len,
     const cpumask_t *srcp, int nbits)
{
 return bitmap_scnprintf(buf, len, srcp->bits, nbits);
}



static inline __attribute__((always_inline)) int __cpumask_parse(const char *buf, int len,
     cpumask_t *dstp, int nbits)
{
 return bitmap_parse(buf, len, dstp->bits, nbits);
}



static inline __attribute__((always_inline)) int __cpulist_scnprintf(char *buf, int len,
     const cpumask_t *srcp, int nbits)
{
 return bitmap_scnlistprintf(buf, len, srcp->bits, nbits);
}


static inline __attribute__((always_inline)) int __cpulist_parse(const char *buf, cpumask_t *dstp, int nbits)
{
 return bitmap_parselist(buf, dstp->bits, nbits);
}



static inline __attribute__((always_inline)) int __cpu_remap(int oldbit,
  const cpumask_t *oldp, const cpumask_t *newp, int nbits)
{
 return bitmap_bitremap(oldbit, oldp->bits, newp->bits, nbits);
}



static inline __attribute__((always_inline)) void __cpus_remap(cpumask_t *dstp, const cpumask_t *srcp,
  const cpumask_t *oldp, const cpumask_t *newp, int nbits)
{
 bitmap_remap(dstp->bits, srcp->bits, oldp->bits, newp->bits, nbits);
}

extern cpumask_t cpu_possible_map;
extern cpumask_t cpu_online_map;
extern cpumask_t cpu_present_map;



extern int tsc_disable;

struct desc_struct {
 unsigned long a,b;
};

struct cpuinfo_x86 {
 __u8 x86;
 __u8 x86_vendor;
 __u8 x86_model;
 __u8 x86_mask;
 char wp_works_ok;
 char hlt_works_ok;
 char hard_math;
 char rfu;
        int cpuid_level;
 unsigned long x86_capability[7];
 char x86_vendor_id[16];
 char x86_model_id[64];
 int x86_cache_size;

 int x86_cache_alignment;
 char fdiv_bug;
 char f00f_bug;
 char coma_bug;
 char pad0;
 int x86_power;
 unsigned long loops_per_jiffy;



 unsigned char x86_max_cores;
 unsigned char apicid;





} __attribute__((__aligned__((1 << (7)))));

extern struct cpuinfo_x86 boot_cpu_data;
extern struct cpuinfo_x86 new_cpu_data;
extern struct tss_struct doublefault_tss;
extern __typeof__(struct tss_struct) per_cpu__init_tss;

extern int cpu_llc_id[1];
extern char ignore_fpu_irq;

extern void identify_cpu(struct cpuinfo_x86 *);
extern void print_cpu_info(struct cpuinfo_x86 *);
extern unsigned int init_intel_cacheinfo(struct cpuinfo_x86 *c);
extern unsigned short num_cache_leaves;




static inline __attribute__((always_inline)) void detect_ht(struct cpuinfo_x86 *c) {}

static inline __attribute__((always_inline)) void cpuid(unsigned int op, unsigned int *eax, unsigned int *ebx, unsigned int *ecx, unsigned int *edx)
{
 __asm__("cpuid"
  : "=a" (*eax),
    "=b" (*ebx),
    "=c" (*ecx),
    "=d" (*edx)
  : "0" (op), "c"(0));
}


static inline __attribute__((always_inline)) void cpuid_count(int op, int count, int *eax, int *ebx, int *ecx,
         int *edx)
{
 __asm__("cpuid"
  : "=a" (*eax),
    "=b" (*ebx),
    "=c" (*ecx),
    "=d" (*edx)
  : "0" (op), "c" (count));
}




static inline __attribute__((always_inline)) unsigned int cpuid_eax(unsigned int op)
{
 unsigned int eax;

 __asm__("cpuid"
  : "=a" (eax)
  : "0" (op)
  : "bx", "cx", "dx");
 return eax;
}
static inline __attribute__((always_inline)) unsigned int cpuid_ebx(unsigned int op)
{
 unsigned int eax, ebx;

 __asm__("cpuid"
  : "=a" (eax), "=b" (ebx)
  : "0" (op)
  : "cx", "dx" );
 return ebx;
}
static inline __attribute__((always_inline)) unsigned int cpuid_ecx(unsigned int op)
{
 unsigned int eax, ecx;

 __asm__("cpuid"
  : "=a" (eax), "=c" (ecx)
  : "0" (op)
  : "bx", "dx" );
 return ecx;
}
static inline __attribute__((always_inline)) unsigned int cpuid_edx(unsigned int op)
{
 unsigned int eax, edx;

 __asm__("cpuid"
  : "=a" (eax), "=d" (edx)
  : "0" (op)
  : "bx", "cx");
 return edx;
}

extern unsigned long mmu_cr4_features;

static inline __attribute__((always_inline)) void set_in_cr4 (unsigned long mask)
{
 unsigned cr4;
 mmu_cr4_features |= mask;
 cr4 = ({ unsigned int __dummy; __asm__( "movl %%cr4,%0\n\t" :"=r" (__dummy)); __dummy; });
 cr4 |= mask;
 __asm__ __volatile__("movl %0,%%cr4": :"r" (cr4));
}

static inline __attribute__((always_inline)) void clear_in_cr4 (unsigned long mask)
{
 unsigned cr4;
 mmu_cr4_features &= ~mask;
 cr4 = ({ unsigned int __dummy; __asm__( "movl %%cr4,%0\n\t" :"=r" (__dummy)); __dummy; });
 cr4 &= ~mask;
 __asm__ __volatile__("movl %0,%%cr4": :"r" (cr4));
}

static inline __attribute__((always_inline)) void sync_core(void)
{
 int tmp;
 asm volatile("cpuid" : "=a" (tmp) : "0" (1) : "ebx","ecx","edx","memory");
}

static inline __attribute__((always_inline)) void __monitor(const void *eax, unsigned long ecx,
  unsigned long edx)
{

 asm volatile(
  ".byte 0x0f,0x01,0xc8;"
  : :"a" (eax), "c" (ecx), "d"(edx));
}

static inline __attribute__((always_inline)) void __mwait(unsigned long eax, unsigned long ecx)
{

 asm volatile(
  ".byte 0x0f,0x01,0xc9;"
  : :"a" (eax), "c" (ecx));
}



extern unsigned int machine_id;
extern unsigned int machine_submodel_id;
extern unsigned int BIOS_revision;
extern unsigned int mca_pentium_flag;


extern int bootloader_type;

struct i387_fsave_struct {
 long cwd;
 long swd;
 long twd;
 long fip;
 long fcs;
 long foo;
 long fos;
 long st_space[20];
 long status;
};

struct i387_fxsave_struct {
 unsigned short cwd;
 unsigned short swd;
 unsigned short twd;
 unsigned short fop;
 long fip;
 long fcs;
 long foo;
 long fos;
 long mxcsr;
 long mxcsr_mask;
 long st_space[32];
 long xmm_space[32];
 long padding[56];
} __attribute__ ((aligned (16)));

struct i387_soft_struct {
 long cwd;
 long swd;
 long twd;
 long fip;
 long fcs;
 long foo;
 long fos;
 long st_space[20];
 unsigned char ftop, changed, lookahead, no_update, rm, alimit;
 struct info *info;
 unsigned long entry_eip;
};

union i387_union {
 struct i387_fsave_struct fsave;
 struct i387_fxsave_struct fxsave;
 struct i387_soft_struct soft;
};

typedef struct {
 unsigned long seg;
} mm_segment_t;

struct thread_struct;

struct tss_struct {
 unsigned short back_link,__blh;
 unsigned long esp0;
 unsigned short ss0,__ss0h;
 unsigned long esp1;
 unsigned short ss1,__ss1h;
 unsigned long esp2;
 unsigned short ss2,__ss2h;
 unsigned long __cr3;
 unsigned long eip;
 unsigned long eflags;
 unsigned long eax,ecx,edx,ebx;
 unsigned long esp;
 unsigned long ebp;
 unsigned long esi;
 unsigned long edi;
 unsigned short es, __esh;
 unsigned short cs, __csh;
 unsigned short ss, __ssh;
 unsigned short ds, __dsh;
 unsigned short fs, __fsh;
 unsigned short gs, __gsh;
 unsigned short ldt, __ldth;
 unsigned short trace, io_bitmap_base;






 unsigned long io_bitmap[((65536/8)/sizeof(long)) + 1];



 unsigned long io_bitmap_max;
 struct thread_struct *io_bitmap_owner;



 unsigned long __cacheline_filler[35];



 unsigned long stack[64];
} __attribute__((packed));



struct thread_struct {

 struct desc_struct tls_array[3];
 unsigned long esp0;
 unsigned long sysenter_cs;
 unsigned long eip;
 unsigned long esp;
 unsigned long fs;
 unsigned long gs;

 unsigned long debugreg[8];

 unsigned long cr2, trap_no, error_code;

 union i387_union i387;

 struct vm86_struct * vm86_info;
 unsigned long screen_bitmap;
 unsigned long v86flags, v86mask, saved_esp0;
 unsigned int saved_fs, saved_gs;

 unsigned long *io_bitmap_ptr;
  unsigned long iopl;

 unsigned long io_bitmap_max;
};

static inline __attribute__((always_inline)) void load_esp0(struct tss_struct *tss, struct thread_struct *thread)
{
 tss->esp0 = thread->esp0;

 if (__builtin_expect(!!(tss->ss1 != thread->sysenter_cs), 0)) {
  tss->ss1 = thread->sysenter_cs;
  __asm__ __volatile__("wrmsr" : : "c" (0x174), "a" (thread->sysenter_cs), "d" (0));
 }
}

static inline __attribute__((always_inline)) void set_iopl_mask(unsigned mask)
{
 unsigned int reg;
 __asm__ __volatile__ ("pushfl;"
         "popl %0;"
         "andl %1, %0;"
         "orl %2, %0;"
         "pushl %0;"
         "popfl"
    : "=&r" (reg)
    : "i" (~0x00003000), "r" (mask));
}


struct task_struct;
struct mm_struct;


extern void release_thread(struct task_struct *);


extern void prepare_to_copy(struct task_struct *tsk);




extern int kernel_thread(int (*fn)(void *), void * arg, unsigned long flags);

extern unsigned long thread_saved_pc(struct task_struct *tsk);
void show_trace(struct task_struct *task, struct pt_regs *regs, unsigned long *stack);

unsigned long get_wchan(struct task_struct *p);

struct microcode_header {
 unsigned int hdrver;
 unsigned int rev;
 unsigned int date;
 unsigned int sig;
 unsigned int cksum;
 unsigned int ldrver;
 unsigned int pf;
 unsigned int datasize;
 unsigned int totalsize;
 unsigned int reserved[3];
};

struct microcode {
 struct microcode_header hdr;
 unsigned int bits[0];
};

typedef struct microcode microcode_t;
typedef struct microcode_header microcode_header_t;


struct extended_signature {
 unsigned int sig;
 unsigned int pf;
 unsigned int cksum;
};

struct extended_sigtable {
 unsigned int count;
 unsigned int cksum;
 unsigned int reserved[3];
 struct extended_signature sigs[0];
};


static inline __attribute__((always_inline)) void rep_nop(void)
{
 __asm__ __volatile__("rep;nop": : :"memory");
}

static inline __attribute__((always_inline)) void prefetch(const void *x)
{
 asm volatile ("661:\n\t" ".byte 0x8d,0x74,0x26,0x00\n" "\n662:\n" ".section .altinstructions,\"a\"\n" "  .align 4\n" "  .long 661b\n" "  .long 663f\n" "  .byte %c0\n" "  .byte 662b-661b\n" "  .byte 664f-663f\n" ".previous\n" ".section .altinstr_replacement,\"ax\"\n" "663:\n\t" "prefetchnta (%1)" "\n664:\n" ".previous" :: "i" ((0*32+25)), "r" (x));



}







static inline __attribute__((always_inline)) void prefetchw(const void *x)
{
 asm volatile ("661:\n\t" ".byte 0x8d,0x74,0x26,0x00\n" "\n662:\n" ".section .altinstructions,\"a\"\n" "  .align 4\n" "  .long 661b\n" "  .long 663f\n" "  .byte %c0\n" "  .byte 662b-661b\n" "  .byte 664f-663f\n" ".previous\n" ".section .altinstr_replacement,\"ax\"\n" "663:\n\t" "prefetchw (%1)" "\n664:\n" ".previous" :: "i" ((1*32+31)), "r" (x));



}


extern void select_idle_routine(const struct cpuinfo_x86 *c);



extern unsigned long boot_option_idle_override;
extern void enable_sep_cpu(void);
extern int sysenter_setup(void);


struct thread_info {
 struct task_struct *task;
 struct exec_domain *exec_domain;
 unsigned long flags;
 unsigned long status;
 __u32 cpu;
 int preempt_count;


 mm_segment_t addr_limit;



 void *sysenter_return;
 struct restart_block restart_block;

 unsigned long previous_esp;


 __u8 supervisor_stack[0];
};

register unsigned long current_stack_pointer asm("esp") __attribute__((__used__));


static inline __attribute__((always_inline)) struct thread_info *current_thread_info(void)
{
 return (struct thread_info *)(current_stack_pointer & ~((4096) - 1));
}


static inline __attribute__((always_inline)) void set_ti_thread_flag(struct thread_info *ti, int flag)
{
 set_bit(flag,&ti->flags);
}

static inline __attribute__((always_inline)) void clear_ti_thread_flag(struct thread_info *ti, int flag)
{
 clear_bit(flag,&ti->flags);
}

static inline __attribute__((always_inline)) int test_and_set_ti_thread_flag(struct thread_info *ti, int flag)
{
 return test_and_set_bit(flag,&ti->flags);
}

static inline __attribute__((always_inline)) int test_and_clear_ti_thread_flag(struct thread_info *ti, int flag)
{
 return test_and_clear_bit(flag,&ti->flags);
}

static inline __attribute__((always_inline)) int test_ti_thread_flag(struct thread_info *ti, int flag)
{
 return (__builtin_constant_p(flag) ? constant_test_bit((flag),(&ti->flags)) : variable_test_bit((flag),(&ti->flags)));
}
























static inline __attribute__((always_inline)) void prefetch_range(void *addr, size_t len)
{

 char *cp;
 char *end = addr + len;

 for (cp = addr; cp < end; cp += (4*(1 << (7))))
  prefetch(cp);

}


struct list_head {
 struct list_head *next, *prev;
};






static inline __attribute__((always_inline)) void INIT_LIST_HEAD(struct list_head *list)
{
 list->next = list;
 list->prev = list;
}







static inline __attribute__((always_inline)) void __list_add(struct list_head *new,
         struct list_head *prev,
         struct list_head *next)
{
 next->prev = new;
 new->next = next;
 new->prev = prev;
 prev->next = new;
}

static inline __attribute__((always_inline)) void list_add(struct list_head *new, struct list_head *head)
{
 __list_add(new, head, head->next);
}

static inline __attribute__((always_inline)) void list_add_tail(struct list_head *new, struct list_head *head)
{
 __list_add(new, head->prev, head);
}







static inline __attribute__((always_inline)) void __list_add_rcu(struct list_head * new,
  struct list_head * prev, struct list_head * next)
{
 new->next = next;
 new->prev = prev;
 __asm__ __volatile__("": : :"memory");
 next->prev = new;
 prev->next = new;
}

static inline __attribute__((always_inline)) void list_add_rcu(struct list_head *new, struct list_head *head)
{
 __list_add_rcu(new, head, head->next);
}

static inline __attribute__((always_inline)) void list_add_tail_rcu(struct list_head *new,
     struct list_head *head)
{
 __list_add_rcu(new, head->prev, head);
}

static inline __attribute__((always_inline)) void __list_del(struct list_head * prev, struct list_head * next)
{
 next->prev = prev;
 prev->next = next;
}





extern void list_del(struct list_head *entry);
/*
static inline __attribute__((always_inline)) void list_del(struct list_head *entry)
{
 __list_del(entry->prev, entry->next);
 entry->next = ((void *) 0x00100100);
 entry->prev = ((void *) 0x00200200);
}
*/
static inline __attribute__((always_inline)) void list_del_rcu(struct list_head *entry)
{
 __list_del(entry->prev, entry->next);
 entry->prev = ((void *) 0x00200200);
}







static inline __attribute__((always_inline)) void list_replace(struct list_head *old,
    struct list_head *new)
{
 new->next = old->next;
 new->next->prev = new;
 new->prev = old->prev;
 new->prev->next = new;
}

static inline __attribute__((always_inline)) void list_replace_init(struct list_head *old,
     struct list_head *new)
{
 list_replace(old, new);
 INIT_LIST_HEAD(old);
}

static inline __attribute__((always_inline)) void list_replace_rcu(struct list_head *old,
    struct list_head *new)
{
 new->next = old->next;
 new->prev = old->prev;
 __asm__ __volatile__("": : :"memory");
 new->next->prev = new;
 new->prev->next = new;
 old->prev = ((void *) 0x00200200);
}





static inline __attribute__((always_inline)) void list_del_init(struct list_head *entry)
{
 __list_del(entry->prev, entry->next);
 INIT_LIST_HEAD(entry);
}






static inline __attribute__((always_inline)) void list_move(struct list_head *list, struct list_head *head)
{
        __list_del(list->prev, list->next);
        list_add(list, head);
}






static inline __attribute__((always_inline)) void list_move_tail(struct list_head *list,
      struct list_head *head)
{
        __list_del(list->prev, list->next);
        list_add_tail(list, head);
}






static inline __attribute__((always_inline)) int list_is_last(const struct list_head *list,
    const struct list_head *head)
{
 return list->next == head;
}




extern int list_empty(const struct list_head *head);
/* [kohei]
static inline __attribute__((always_inline)) int list_empty(const struct list_head *head)
{
  return head->next == head;
}
*/
static inline __attribute__((always_inline)) int list_empty_careful(const struct list_head *head)
{
 struct list_head *next = head->next;
 return (next == head) && (next == head->prev);
}

static inline __attribute__((always_inline)) void __list_splice(struct list_head *list,
     struct list_head *head)
{
 struct list_head *first = list->next;
 struct list_head *last = list->prev;
 struct list_head *at = head->next;

 first->prev = head;
 head->next = first;

 last->next = at;
 at->prev = last;
}






static inline __attribute__((always_inline)) void list_splice(struct list_head *list, struct list_head *head)
{
 if (!list_empty(list))
  __list_splice(list, head);
}

static inline __attribute__((always_inline)) void list_splice_init(struct list_head *list,
        struct list_head *head)
{
 if (!list_empty(list)) {
  __list_splice(list, head);
  INIT_LIST_HEAD(list);
 }
}

struct hlist_head {
 struct hlist_node *first;
};

struct hlist_node {
 struct hlist_node *next, **pprev;
};




static inline __attribute__((always_inline)) void INIT_HLIST_NODE(struct hlist_node *h)
{
 h->next = ((void *)0);
 h->pprev = ((void *)0);
}

static inline __attribute__((always_inline)) int hlist_unhashed(const struct hlist_node *h)
{
 return !h->pprev;
}

static inline __attribute__((always_inline)) int hlist_empty(const struct hlist_head *h)
{
 return !h->first;
}

static inline __attribute__((always_inline)) void __hlist_del(struct hlist_node *n)
{
 struct hlist_node *next = n->next;
 struct hlist_node **pprev = n->pprev;
 *pprev = next;
 if (next)
  next->pprev = pprev;
}

static inline __attribute__((always_inline)) void hlist_del(struct hlist_node *n)
{
 __hlist_del(n);
 n->next = ((void *) 0x00100100);
 n->pprev = ((void *) 0x00200200);
}

static inline __attribute__((always_inline)) void hlist_del_rcu(struct hlist_node *n)
{
 __hlist_del(n);
 n->pprev = ((void *) 0x00200200);
}

static inline __attribute__((always_inline)) void hlist_del_init(struct hlist_node *n)
{
 if (!hlist_unhashed(n)) {
  __hlist_del(n);
  INIT_HLIST_NODE(n);
 }
}

static inline __attribute__((always_inline)) void hlist_replace_rcu(struct hlist_node *old,
     struct hlist_node *new)
{
 struct hlist_node *next = old->next;

 new->next = next;
 new->pprev = old->pprev;
 __asm__ __volatile__("": : :"memory");
 if (next)
  new->next->pprev = &new->next;
 *new->pprev = new;
 old->pprev = ((void *) 0x00200200);
}

static inline __attribute__((always_inline)) void hlist_add_head(struct hlist_node *n, struct hlist_head *h)
{
 struct hlist_node *first = h->first;
 n->next = first;
 if (first)
  first->pprev = &n->next;
 h->first = n;
 n->pprev = &h->first;
}

static inline __attribute__((always_inline)) void hlist_add_head_rcu(struct hlist_node *n,
     struct hlist_head *h)
{
 struct hlist_node *first = h->first;
 n->next = first;
 n->pprev = &h->first;
 __asm__ __volatile__("": : :"memory");
 if (first)
  first->pprev = &n->next;
 h->first = n;
}


static inline __attribute__((always_inline)) void hlist_add_before(struct hlist_node *n,
     struct hlist_node *next)
{
 n->pprev = next->pprev;
 n->next = next;
 next->pprev = &n->next;
 *(n->pprev) = n;
}

static inline __attribute__((always_inline)) void hlist_add_after(struct hlist_node *n,
     struct hlist_node *next)
{
 next->next = n->next;
 n->next = next;
 next->pprev = &n->next;

 if(next->next)
  next->next->pprev = &next->next;
}

static inline __attribute__((always_inline)) void hlist_add_before_rcu(struct hlist_node *n,
     struct hlist_node *next)
{
 n->pprev = next->pprev;
 n->next = next;
 __asm__ __volatile__("": : :"memory");
 next->pprev = &n->next;
 *(n->pprev) = n;
}

static inline __attribute__((always_inline)) void hlist_add_after_rcu(struct hlist_node *prev,
           struct hlist_node *n)
{
 n->next = prev->next;
 n->pprev = &prev->next;
 __asm__ __volatile__("": : :"memory");
 prev->next = n;
 if (n->next)
  n->next->pprev = &n->next;
}





struct task_struct;

extern int debug_locks;
extern int debug_locks_silent;




extern int debug_locks_off(void);

static inline __attribute__((always_inline)) void debug_show_all_locks(void)
{
}

static inline __attribute__((always_inline)) void debug_show_held_locks(struct task_struct *task)
{
}

static inline __attribute__((always_inline)) void
debug_check_no_locks_freed(const void *from, unsigned long len)
{
}

static inline __attribute__((always_inline)) void
debug_check_no_locks_held(struct task_struct *task)
{
}




static inline __attribute__((always_inline)) void lockdep_off(void)
{
}

static inline __attribute__((always_inline)) void lockdep_on(void)
{
}

static inline __attribute__((always_inline)) int lockdep_internal(void)
{
 return 0;
}

struct lock_class_key { };







typedef struct {
 volatile unsigned int slock;



} raw_spinlock_t;

typedef struct {




} raw_rwlock_t;



typedef struct {
 raw_spinlock_t raw_lock;




 unsigned int magic, owner_cpu;
 void *owner;




} spinlock_t;



typedef struct {
 raw_rwlock_t raw_lock;




 unsigned int magic, owner_cpu;
 void *owner;




} rwlock_t;


extern int __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) generic__raw_read_trylock(raw_rwlock_t *lock);









static inline __attribute__((always_inline)) void __raw_spin_lock(raw_spinlock_t *lock)
{
 lock->slock = 0;
}

static inline __attribute__((always_inline)) void
__raw_spin_lock_flags(raw_spinlock_t *lock, unsigned long flags)
{
 do { do { (flags) = __raw_local_irq_save(); } while (0); do { } while (0); } while (0);
 lock->slock = 0;
}

static inline __attribute__((always_inline)) int __raw_spin_trylock(raw_spinlock_t *lock)
{
 char oldval = lock->slock;

 lock->slock = 0;

 return oldval > 0;
}

static inline __attribute__((always_inline)) void __raw_spin_unlock(raw_spinlock_t *lock)
{
 lock->slock = 1;
}




  extern void __spin_lock_init(spinlock_t *lock, const char *name,
          struct lock_class_key *key);

  extern void __rwlock_init(rwlock_t *lock, const char *name,
       struct lock_class_key *key);



int in_lock_functions(unsigned long addr);



void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _spin_lock(spinlock_t *lock) ;
void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _spin_lock_nested(spinlock_t *lock, int subclass)
       ;
void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _read_lock(rwlock_t *lock) ;
void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _write_lock(rwlock_t *lock) ;
void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _spin_lock_bh(spinlock_t *lock) ;
void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _read_lock_bh(rwlock_t *lock) ;
void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _write_lock_bh(rwlock_t *lock) ;
void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _spin_lock_irq(spinlock_t *lock) ;
void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _read_lock_irq(rwlock_t *lock) ;
void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _write_lock_irq(rwlock_t *lock) ;
unsigned long __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _spin_lock_irqsave(spinlock_t *lock)
       ;
unsigned long __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _read_lock_irqsave(rwlock_t *lock)
       ;
unsigned long __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _write_lock_irqsave(rwlock_t *lock)
       ;
int __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _spin_trylock(spinlock_t *lock);
int __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _read_trylock(rwlock_t *lock);
int __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _write_trylock(rwlock_t *lock);
int __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _spin_trylock_bh(spinlock_t *lock);
void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _spin_unlock(spinlock_t *lock) ;
void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _read_unlock(rwlock_t *lock) ;
void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _write_unlock(rwlock_t *lock) ;
void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _spin_unlock_bh(spinlock_t *lock) ;
void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _read_unlock_bh(rwlock_t *lock) ;
void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _write_unlock_bh(rwlock_t *lock) ;
void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _spin_unlock_irq(spinlock_t *lock) ;
void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _read_unlock_irq(rwlock_t *lock) ;
void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _write_unlock_irq(rwlock_t *lock) ;
void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _spin_unlock_irqrestore(spinlock_t *lock, unsigned long flags)
       ;
void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _read_unlock_irqrestore(rwlock_t *lock, unsigned long flags)
       ;
void __attribute__((regparm(3))) __attribute__((section(".spinlock.text"))) _write_unlock_irqrestore(rwlock_t *lock, unsigned long flags)
       ;






 extern void _raw_spin_lock(spinlock_t *lock);

 extern int _raw_spin_trylock(spinlock_t *lock);
 extern void _raw_spin_unlock(spinlock_t *lock);
 extern void _raw_read_lock(rwlock_t *lock);
 extern int _raw_read_trylock(rwlock_t *lock);
 extern void _raw_read_unlock(rwlock_t *lock);
 extern void _raw_write_lock(rwlock_t *lock);
 extern int _raw_write_trylock(rwlock_t *lock);
 extern void _raw_write_unlock(rwlock_t *lock);



typedef struct { volatile int counter; } atomic_t;

static __inline__ __attribute__((always_inline)) void atomic_add(int i, atomic_t *v)
{
 __asm__ __volatile__(
  "" "addl %1,%0"
  :"+m" (v->counter)
  :"ir" (i));
}

static __inline__ __attribute__((always_inline)) void atomic_sub(int i, atomic_t *v)
{
 __asm__ __volatile__(
  "" "subl %1,%0"
  :"+m" (v->counter)
  :"ir" (i));
}

static __inline__ __attribute__((always_inline)) int atomic_sub_and_test(int i, atomic_t *v)
{
 unsigned char c;

 __asm__ __volatile__(
  "" "subl %2,%0; sete %1"
  :"+m" (v->counter), "=qm" (c)
  :"ir" (i) : "memory");
 return c;
}







static __inline__ __attribute__((always_inline)) void atomic_inc(atomic_t *v)
{
 __asm__ __volatile__(
  "" "incl %0"
  :"+m" (v->counter));
}







static __inline__ __attribute__((always_inline)) void atomic_dec(atomic_t *v)
{
 __asm__ __volatile__(
  "" "decl %0"
  :"+m" (v->counter));
}

static __inline__ __attribute__((always_inline)) int atomic_dec_and_test(atomic_t *v)
{
 unsigned char c;

 __asm__ __volatile__(
  "" "decl %0; sete %1"
  :"+m" (v->counter), "=qm" (c)
  : : "memory");
 return c != 0;
}

static __inline__ __attribute__((always_inline)) int atomic_inc_and_test(atomic_t *v)
{
 unsigned char c;

 __asm__ __volatile__(
  "" "incl %0; sete %1"
  :"+m" (v->counter), "=qm" (c)
  : : "memory");
 return c != 0;
}

static __inline__ __attribute__((always_inline)) int atomic_add_negative(int i, atomic_t *v)
{
 unsigned char c;

 __asm__ __volatile__(
  "" "addl %2,%0; sets %1"
  :"+m" (v->counter), "=qm" (c)
  :"ir" (i) : "memory");
 return c;
}

static __inline__ __attribute__((always_inline)) int atomic_add_return(int i, atomic_t *v)
{
 int __i;






 __i = i;
 __asm__ __volatile__(
  "" "xaddl %0, %1;"
  :"=r"(i)
  :"m"(v->counter), "0"(i));
 return i + __i;

}

static __inline__ __attribute__((always_inline)) int atomic_sub_return(int i, atomic_t *v)
{
 return atomic_add_return(-i,v);
}



typedef atomic_t atomic_long_t;


static inline __attribute__((always_inline)) long atomic_long_read(atomic_long_t *l)
{
 atomic_t *v = (atomic_t *)l;

 return (long)((v)->counter);
}

static inline __attribute__((always_inline)) void atomic_long_set(atomic_long_t *l, long i)
{
 atomic_t *v = (atomic_t *)l;

 (((v)->counter) = (i));
}

static inline __attribute__((always_inline)) void atomic_long_inc(atomic_long_t *l)
{
 atomic_t *v = (atomic_t *)l;

 atomic_inc(v);
}

static inline __attribute__((always_inline)) void atomic_long_dec(atomic_long_t *l)
{
 atomic_t *v = (atomic_t *)l;

 atomic_dec(v);
}

static inline __attribute__((always_inline)) void atomic_long_add(long i, atomic_long_t *l)
{
 atomic_t *v = (atomic_t *)l;

 atomic_add(i, v);
}

static inline __attribute__((always_inline)) void atomic_long_sub(long i, atomic_long_t *l)
{
 atomic_t *v = (atomic_t *)l;

 atomic_sub(i, v);
}







extern int _atomic_dec_and_lock(atomic_t *atomic, spinlock_t *lock);







struct task_struct;

static inline __attribute__((always_inline)) __attribute__((always_inline)) struct task_struct * get_current(void)
{
 return current_thread_info()->task;
}



extern struct task_struct *__vericon_dummy_current;


typedef __u32 kernel_cap_t;

extern kernel_cap_t cap_bset;

static inline __attribute__((always_inline)) kernel_cap_t cap_combine(kernel_cap_t a, kernel_cap_t b)
{
     kernel_cap_t dest;
     (dest) = (a) | (b);
     return dest;
}

static inline __attribute__((always_inline)) kernel_cap_t cap_intersect(kernel_cap_t a, kernel_cap_t b)
{
     kernel_cap_t dest;
     (dest) = (a) & (b);
     return dest;
}

static inline __attribute__((always_inline)) kernel_cap_t cap_drop(kernel_cap_t a, kernel_cap_t drop)
{
     kernel_cap_t dest;
     (dest) = (a) & ~(drop);
     return dest;
}

static inline __attribute__((always_inline)) kernel_cap_t cap_invert(kernel_cap_t c)
{
     kernel_cap_t dest;
     (dest) = ~(c);
     return dest;
}

int capable(int cap);
int __capable(struct task_struct *t, int cap);















typedef struct {
 unsigned sequence;
 spinlock_t lock;
} seqlock_t;

static inline __attribute__((always_inline)) void write_seqlock(seqlock_t *sl)
{
 _spin_lock(&sl->lock);
 ++sl->sequence;
 __asm__ __volatile__("": : :"memory");
}

static inline __attribute__((always_inline)) void write_sequnlock(seqlock_t *sl)
{
 __asm__ __volatile__("": : :"memory");
 sl->sequence++;
 _spin_unlock(&sl->lock);
}

static inline __attribute__((always_inline)) int write_tryseqlock(seqlock_t *sl)
{
 int ret = (_spin_trylock(&sl->lock));

 if (ret) {
  ++sl->sequence;
  __asm__ __volatile__("": : :"memory");
 }
 return ret;
}


static inline __attribute__((always_inline)) __attribute__((always_inline)) unsigned read_seqbegin(const seqlock_t *sl)
{
 unsigned ret = sl->sequence;
 __asm__ __volatile__("": : :"memory");
 return ret;
}

static inline __attribute__((always_inline)) __attribute__((always_inline)) int read_seqretry(const seqlock_t *sl, unsigned iv)
{
 __asm__ __volatile__("": : :"memory");
 return (iv & 1) | (sl->sequence ^ iv);
}

typedef struct seqcount {
 unsigned sequence;
} seqcount_t;





static inline __attribute__((always_inline)) unsigned read_seqcount_begin(const seqcount_t *s)
{
 unsigned ret = s->sequence;
 __asm__ __volatile__("": : :"memory");
 return ret;
}






static inline __attribute__((always_inline)) int read_seqcount_retry(const seqcount_t *s, unsigned iv)
{
 __asm__ __volatile__("": : :"memory");
 return (iv & 1) | (s->sequence ^ iv);
}






static inline __attribute__((always_inline)) void write_seqcount_begin(seqcount_t *s)
{
 s->sequence++;
 __asm__ __volatile__("": : :"memory");
}

static inline __attribute__((always_inline)) void write_seqcount_end(seqcount_t *s)
{
 __asm__ __volatile__("": : :"memory");
 s->sequence++;
}





struct timespec {
 time_t tv_sec;
 long tv_nsec;
};


struct timeval {
 time_t tv_sec;
 suseconds_t tv_usec;
};

struct timezone {
 int tz_minuteswest;
 int tz_dsttime;
};

static inline __attribute__((always_inline)) int timespec_equal(struct timespec *a, struct timespec *b)
{
 return (a->tv_sec == b->tv_sec) && (a->tv_nsec == b->tv_nsec);
}






static inline __attribute__((always_inline)) int timespec_compare(struct timespec *lhs, struct timespec *rhs)
{
 if (lhs->tv_sec < rhs->tv_sec)
  return -1;
 if (lhs->tv_sec > rhs->tv_sec)
  return 1;
 return lhs->tv_nsec - rhs->tv_nsec;
}

static inline __attribute__((always_inline)) int timeval_compare(struct timeval *lhs, struct timeval *rhs)
{
 if (lhs->tv_sec < rhs->tv_sec)
  return -1;
 if (lhs->tv_sec > rhs->tv_sec)
  return 1;
 return lhs->tv_usec - rhs->tv_usec;
}

extern unsigned long mktime(const unsigned int year, const unsigned int mon,
       const unsigned int day, const unsigned int hour,
       const unsigned int min, const unsigned int sec);

extern void set_normalized_timespec(struct timespec *ts, time_t sec, long nsec);




static inline __attribute__((always_inline)) struct timespec timespec_sub(struct timespec lhs,
      struct timespec rhs)
{
 struct timespec ts_delta;
 set_normalized_timespec(&ts_delta, lhs.tv_sec - rhs.tv_sec,
    lhs.tv_nsec - rhs.tv_nsec);
 return ts_delta;
}







extern struct timespec xtime;
extern struct timespec wall_to_monotonic;
extern seqlock_t xtime_lock;

void timekeeping_init(void);

static inline __attribute__((always_inline)) unsigned long get_seconds(void)
{
 return xtime.tv_sec;
}

struct timespec current_kernel_time(void);




extern void do_gettimeofday(struct timeval *tv);
extern int do_settimeofday(struct timespec *tv);
extern int do_sys_settimeofday(struct timespec *tv, struct timezone *tz);

extern long do_utimes(int dfd, char *filename, struct timeval *times);
struct itimerval;
extern int do_setitimer(int which, struct itimerval *value,
   struct itimerval *ovalue);
extern unsigned int alarm_setitimer(unsigned int seconds);
extern int do_getitimer(int which, struct itimerval *value);
extern void getnstimeofday(struct timespec *tv);

extern struct timespec timespec_trunc(struct timespec t, unsigned gran);
extern int timekeeping_is_continuous(void);

static inline __attribute__((always_inline)) s64 timespec_to_ns(const struct timespec *ts)
{
 return ((s64) ts->tv_sec * 1000000000L) + ts->tv_nsec;
}

static inline __attribute__((always_inline)) s64 timeval_to_ns(const struct timeval *tv)
{
 return ((s64) tv->tv_sec * 1000000000L) +
  tv->tv_usec * 1000L;
}







extern struct timespec ns_to_timespec(const s64 nsec);







extern struct timeval ns_to_timeval(const s64 nsec);






static inline __attribute__((always_inline)) void timespec_add_ns(struct timespec *a, u64 ns)
{
 ns += a->tv_nsec;
 while(__builtin_expect(!!(ns >= 1000000000L), 0)) {
  ns -= 1000000000L;
  a->tv_sec++;
 }
 a->tv_nsec = ns;
}

struct itimerspec {
 struct timespec it_interval;
 struct timespec it_value;
};

struct itimerval {
 struct timeval it_interval;
 struct timeval it_value;
};


struct timex {
 unsigned int modes;
 long offset;
 long freq;
 long maxerror;
 long esterror;
 int status;
 long constant;
 long precision;
 long tolerance;


 struct timeval time;
 long tick;

 long ppsfreq;
 long jitter;
 int shift;
 long stabil;
 long jitcnt;
 long calcnt;
 long errcnt;
 long stbcnt;

 int :32; int :32; int :32; int :32;
 int :32; int :32; int :32; int :32;
 int :32; int :32; int :32; int :32;
};















typedef unsigned long long cycles_t;

extern unsigned int cpu_khz;
extern unsigned int tsc_khz;

static inline __attribute__((always_inline)) cycles_t get_cycles(void)
{
 unsigned long long ret = 0;







 __asm__ __volatile__("rdtsc" : "=A" (ret));

 return ret;
}

extern void tsc_init(void);
extern void mark_tsc_unstable(void);


extern int read_current_timer(unsigned long *timer_value);







extern unsigned long tick_usec;
extern unsigned long tick_nsec;
extern int tickadj;




extern int time_state;
extern int time_status;
extern long time_offset;
extern long time_constant;
extern long time_tolerance;
extern long time_precision;
extern long time_maxerror;
extern long time_esterror;

extern long time_freq;
extern long time_reftime;

extern long time_adjust;
extern long time_next_adjust;






static inline __attribute__((always_inline)) void ntp_clear(void)
{
 time_adjust = 0;
 time_status |= 0x0040;
 time_maxerror = (512000L << 5);
 time_esterror = (512000L << 5);
}





static inline __attribute__((always_inline)) int ntp_synced(void)
{
 return !(time_status & 0x0040);
}

static inline __attribute__((always_inline)) void
time_interpolator_reset(void)
{
}






extern u64 current_tick_length(void);

extern int do_adjtimex(struct timex *);












static inline __attribute__((always_inline)) long
div_ll_X_l_rem(long long divs, long div, long *rem)
{
 long dum2;
      __asm__("divl %2":"=a"(dum2), "=d"(*rem)
      : "rm"(div), "A"(divs));

 return dum2;

}


static inline __attribute__((always_inline)) long div_long_long_rem_signed(const long long dividend,
         const long divisor, long *remainder)
{
 long res;

 if (__builtin_expect(!!(dividend < 0), 0)) {
  res = -div_ll_X_l_rem(-dividend,divisor,remainder);
  *remainder = -(*remainder);
 } else
  res = div_ll_X_l_rem(dividend,divisor,remainder);

 return res;
}


extern u64 __attribute__((section(".data"))) jiffies_64;
extern unsigned long volatile __attribute__((section(".data"))) jiffies;


u64 get_jiffies_64(void);

static inline __attribute__((always_inline)) unsigned int jiffies_to_msecs(const unsigned long j)
{

 return (1000L / 1000) * j;





}

static inline __attribute__((always_inline)) unsigned int jiffies_to_usecs(const unsigned long j)
{

 return (1000000L / 1000) * j;





}

static inline __attribute__((always_inline)) unsigned long msecs_to_jiffies(const unsigned int m)
{
 if (m > jiffies_to_msecs(((~0UL >> 1)-1)))
  return ((~0UL >> 1)-1);

 return (m + (1000L / 1000) - 1) / (1000L / 1000);





}

static inline __attribute__((always_inline)) unsigned long usecs_to_jiffies(const unsigned int u)
{
 if (u > jiffies_to_usecs(((~0UL >> 1)-1)))
  return ((~0UL >> 1)-1);

 return (u + (1000000L / 1000) - 1) / (1000000L / 1000);





}

static __inline__ __attribute__((always_inline)) unsigned long
timespec_to_jiffies(const struct timespec *value)
{
 unsigned long sec = value->tv_sec;
 long nsec = value->tv_nsec + (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))))) - 1;

 if (sec >= (long)((u64)((u64)((~0UL >> 1)-1) * (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))))) / 1000000000L)){
  sec = (long)((u64)((u64)((~0UL >> 1)-1) * (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))))) / 1000000000L);
  nsec = 0;
 }
 return (((u64)sec * ((unsigned long)((((u64)1000000000L << (32 - 10)) + (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))))) -1) / (u64)(( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))))))) +
  (((u64)nsec * ((unsigned long)((((u64)1 << ((32 - 10) + 29)) + (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))))) -1) / (u64)(( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))))))) >>
   (((32 - 10) + 29) - (32 - 10)))) >> (32 - 10);

}

static __inline__ __attribute__((always_inline)) void
jiffies_to_timespec(const unsigned long jiffies, struct timespec *value)
{




 u64 nsec = (u64)jiffies * (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))));
 value->tv_sec = div_ll_X_l_rem(nsec,1000000000L,&value->tv_nsec);
}

static __inline__ __attribute__((always_inline)) unsigned long
timeval_to_jiffies(const struct timeval *value)
{
 unsigned long sec = value->tv_sec;
 long usec = value->tv_usec;

 if (sec >= (long)((u64)((u64)((~0UL >> 1)-1) * (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))))) / 1000000000L)){
  sec = (long)((u64)((u64)((~0UL >> 1)-1) * (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))))) / 1000000000L);
  usec = 0;
 }
 return (((u64)sec * ((unsigned long)((((u64)1000000000L << (32 - 10)) + (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))))) -1) / (u64)(( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))))))) +
  (((u64)usec * ((unsigned long)((((u64)1000L << ((32 - 10) + 19)) + (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))))) -1) / (u64)(( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))))))) + (u64)(((u64)1 << ((32 - 10) + 19)) - 1)) >>
   (((32 - 10) + 19) - (32 - 10)))) >> (32 - 10);
}

static __inline__ __attribute__((always_inline)) void
jiffies_to_timeval(const unsigned long jiffies, struct timeval *value)
{




 u64 nsec = (u64)jiffies * (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))));
 long tv_usec;

 value->tv_sec = div_ll_X_l_rem(nsec,1000000000L,&tv_usec);
 tv_usec /= 1000L;
 value->tv_usec = tv_usec;
}




static inline __attribute__((always_inline)) clock_t jiffies_to_clock_t(long x)
{



 u64 tmp = (u64)x * (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))));
 ({ unsigned long __upper, __low, __high, __mod, __base; __base = ((1000000000L / 100)); asm("":"=a" (__low), "=d" (__high):"A" (tmp)); __upper = __high; if (__high) { __upper = __high % (__base); __high = __high / (__base); } asm("divl %2":"=a" (__low), "=d" (__mod):"rm" (__base), "0" (__low), "1" (__upper)); asm("":"=A" (tmp):"a" (__low),"d" (__high)); __mod; });
 return (long)tmp;

}

static inline __attribute__((always_inline)) unsigned long clock_t_to_jiffies(unsigned long x)
{

 if (x >= ~0UL / (1000 / 100))
  return ~0UL;
 return x * (1000 / 100);

}

static inline __attribute__((always_inline)) u64 jiffies_64_to_clock_t(u64 x)
{

 x *= (( (((1000000UL * 1000) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((((1000000UL * 1000) % ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))) << (8)) + ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000))))) / 2) / ((( (((1193182) / (((1193182 + 1000/2) / 1000))) << (8)) + ((((1193182) % (((1193182 + 1000/2) / 1000))) << (8)) + (((1193182 + 1000/2) / 1000)) / 2) / (((1193182 + 1000/2) / 1000)))))));
 ({ unsigned long __upper, __low, __high, __mod, __base; __base = ((1000000000L / 100)); asm("":"=a" (__low), "=d" (__high):"A" (x)); __upper = __high; if (__high) { __upper = __high % (__base); __high = __high / (__base); } asm("divl %2":"=a" (__low), "=d" (__mod):"rm" (__base), "0" (__low), "1" (__upper)); asm("":"=A" (x):"a" (__low),"d" (__high)); __mod; });

 return x;
}

static inline __attribute__((always_inline)) u64 nsec_to_clock_t(u64 x)
{

 ({ unsigned long __upper, __low, __high, __mod, __base; __base = ((1000000000L / 100)); asm("":"=a" (__low), "=d" (__high):"A" (x)); __upper = __high; if (__high) { __upper = __high % (__base); __high = __high / (__base); } asm("divl %2":"=a" (__low), "=d" (__mod):"rm" (__base), "0" (__low), "1" (__upper)); asm("":"=A" (x):"a" (__low),"d" (__high)); __mod; });

 return x;
}



struct rb_node
{
 unsigned long rb_parent_color;


 struct rb_node *rb_right;
 struct rb_node *rb_left;
} __attribute__((aligned(sizeof(long))));


struct rb_root
{
 struct rb_node *rb_node;
};

static inline __attribute__((always_inline)) void rb_set_parent(struct rb_node *rb, struct rb_node *p)
{
 rb->rb_parent_color = (rb->rb_parent_color & 3) | (unsigned long)p;
}
static inline __attribute__((always_inline)) void rb_set_color(struct rb_node *rb, int color)
{
 rb->rb_parent_color = (rb->rb_parent_color & ~1) | color;
}

extern void rb_insert_color(struct rb_node *, struct rb_root *);
extern void rb_erase(struct rb_node *, struct rb_root *);


extern struct rb_node *rb_next(struct rb_node *);
extern struct rb_node *rb_prev(struct rb_node *);
extern struct rb_node *rb_first(struct rb_root *);
extern struct rb_node *rb_last(struct rb_root *);


extern void rb_replace_node(struct rb_node *victim, struct rb_node *new,
       struct rb_root *root);

static inline __attribute__((always_inline)) void rb_link_node(struct rb_node * node, struct rb_node * parent,
    struct rb_node ** rb_link)
{
 node->rb_parent_color = (unsigned long )parent;
 node->rb_left = node->rb_right = ((void *)0);

 *rb_link = node;
}

























typedef struct { unsigned long bits[((((1 << 0))+32 -1)/32)]; } nodemask_t;
extern nodemask_t _unused_nodemask_arg_;


static inline __attribute__((always_inline)) void __node_set(int node, volatile nodemask_t *dstp)
{
 set_bit(node, dstp->bits);
}


static inline __attribute__((always_inline)) void __node_clear(int node, volatile nodemask_t *dstp)
{
 clear_bit(node, dstp->bits);
}


static inline __attribute__((always_inline)) void __nodes_setall(nodemask_t *dstp, int nbits)
{
 bitmap_fill(dstp->bits, nbits);
}


static inline __attribute__((always_inline)) void __nodes_clear(nodemask_t *dstp, int nbits)
{
 bitmap_zero(dstp->bits, nbits);
}






static inline __attribute__((always_inline)) int __node_test_and_set(int node, nodemask_t *addr)
{
 return test_and_set_bit(node, addr->bits);
}



static inline __attribute__((always_inline)) void __nodes_and(nodemask_t *dstp, const nodemask_t *src1p,
     const nodemask_t *src2p, int nbits)
{
 bitmap_and(dstp->bits, src1p->bits, src2p->bits, nbits);
}



static inline __attribute__((always_inline)) void __nodes_or(nodemask_t *dstp, const nodemask_t *src1p,
     const nodemask_t *src2p, int nbits)
{
 bitmap_or(dstp->bits, src1p->bits, src2p->bits, nbits);
}



static inline __attribute__((always_inline)) void __nodes_xor(nodemask_t *dstp, const nodemask_t *src1p,
     const nodemask_t *src2p, int nbits)
{
 bitmap_xor(dstp->bits, src1p->bits, src2p->bits, nbits);
}



static inline __attribute__((always_inline)) void __nodes_andnot(nodemask_t *dstp, const nodemask_t *src1p,
     const nodemask_t *src2p, int nbits)
{
 bitmap_andnot(dstp->bits, src1p->bits, src2p->bits, nbits);
}



static inline __attribute__((always_inline)) void __nodes_complement(nodemask_t *dstp,
     const nodemask_t *srcp, int nbits)
{
 bitmap_complement(dstp->bits, srcp->bits, nbits);
}



static inline __attribute__((always_inline)) int __nodes_equal(const nodemask_t *src1p,
     const nodemask_t *src2p, int nbits)
{
 return bitmap_equal(src1p->bits, src2p->bits, nbits);
}



static inline __attribute__((always_inline)) int __nodes_intersects(const nodemask_t *src1p,
     const nodemask_t *src2p, int nbits)
{
 return bitmap_intersects(src1p->bits, src2p->bits, nbits);
}



static inline __attribute__((always_inline)) int __nodes_subset(const nodemask_t *src1p,
     const nodemask_t *src2p, int nbits)
{
 return bitmap_subset(src1p->bits, src2p->bits, nbits);
}


static inline __attribute__((always_inline)) int __nodes_empty(const nodemask_t *srcp, int nbits)
{
 return bitmap_empty(srcp->bits, nbits);
}


static inline __attribute__((always_inline)) int __nodes_full(const nodemask_t *srcp, int nbits)
{
 return bitmap_full(srcp->bits, nbits);
}


static inline __attribute__((always_inline)) int __nodes_weight(const nodemask_t *srcp, int nbits)
{
 return bitmap_weight(srcp->bits, nbits);
}



static inline __attribute__((always_inline)) void __nodes_shift_right(nodemask_t *dstp,
     const nodemask_t *srcp, int n, int nbits)
{
 bitmap_shift_right(dstp->bits, srcp->bits, n, nbits);
}



static inline __attribute__((always_inline)) void __nodes_shift_left(nodemask_t *dstp,
     const nodemask_t *srcp, int n, int nbits)
{
 bitmap_shift_left(dstp->bits, srcp->bits, n, nbits);
}





static inline __attribute__((always_inline)) int __first_node(const nodemask_t *srcp)
{
 return ({ int __x = ((1 << 0)); int __y = (find_first_bit(srcp->bits, (1 << 0))); __x < __y ? __x: __y; });
}


static inline __attribute__((always_inline)) int __next_node(int n, const nodemask_t *srcp)
{
 return ({ int __x = ((1 << 0)); int __y = (find_next_bit(srcp->bits, (1 << 0), n+1)); __x < __y ? __x: __y; });
}

static inline __attribute__((always_inline)) int __first_unset_node(const nodemask_t *maskp)
{
 return ({ int __x = ((1 << 0)); int __y = (find_first_zero_bit(maskp->bits, (1 << 0))); __x < __y ? __x: __y; });

}

static inline __attribute__((always_inline)) int __nodemask_scnprintf(char *buf, int len,
     const nodemask_t *srcp, int nbits)
{
 return bitmap_scnprintf(buf, len, srcp->bits, nbits);
}



static inline __attribute__((always_inline)) int __nodemask_parse(const char *buf, int len,
     nodemask_t *dstp, int nbits)
{
 return bitmap_parse(buf, len, dstp->bits, nbits);
}



static inline __attribute__((always_inline)) int __nodelist_scnprintf(char *buf, int len,
     const nodemask_t *srcp, int nbits)
{
 return bitmap_scnlistprintf(buf, len, srcp->bits, nbits);
}


static inline __attribute__((always_inline)) int __nodelist_parse(const char *buf, nodemask_t *dstp, int nbits)
{
 return bitmap_parselist(buf, dstp->bits, nbits);
}



static inline __attribute__((always_inline)) int __node_remap(int oldbit,
  const nodemask_t *oldp, const nodemask_t *newp, int nbits)
{
 return bitmap_bitremap(oldbit, oldp->bits, newp->bits, nbits);
}



static inline __attribute__((always_inline)) void __nodes_remap(nodemask_t *dstp, const nodemask_t *srcp,
  const nodemask_t *oldp, const nodemask_t *newp, int nbits)
{
 bitmap_remap(dstp->bits, srcp->bits, oldp->bits, newp->bits, nbits);
}

extern nodemask_t node_online_map;
extern nodemask_t node_possible_map;







typedef struct __wait_queue wait_queue_t;
typedef int (*wait_queue_func_t)(wait_queue_t *wait, unsigned mode, int sync, void *key);
int default_wake_function(wait_queue_t *wait, unsigned mode, int sync, void *key);

struct __wait_queue {
 unsigned int flags;

 void *private;
 wait_queue_func_t func;
 struct list_head task_list;
};

struct wait_bit_key {
 void *flags;
 int bit_nr;
};

struct wait_bit_queue {
 struct wait_bit_key key;
 wait_queue_t wait;
};

struct __wait_queue_head {
 spinlock_t lock;
 struct list_head task_list;
};
typedef struct __wait_queue_head wait_queue_head_t;

struct task_struct;

extern void init_waitqueue_head(wait_queue_head_t *q);

static inline __attribute__((always_inline)) void init_waitqueue_entry(wait_queue_t *q, struct task_struct *p)
{
 q->flags = 0;
 q->private = p;
 q->func = default_wake_function;
}

static inline __attribute__((always_inline)) void init_waitqueue_func_entry(wait_queue_t *q,
     wait_queue_func_t func)
{
 q->flags = 0;
 q->private = ((void *)0);
 q->func = func;
}

static inline __attribute__((always_inline)) int waitqueue_active(wait_queue_head_t *q)
{
 return !list_empty(&q->task_list);
}

extern void add_wait_queue(wait_queue_head_t *q, wait_queue_t * wait) __attribute__((regparm(3)));
extern void add_wait_queue_exclusive(wait_queue_head_t *q, wait_queue_t * wait) __attribute__((regparm(3)));
extern void remove_wait_queue(wait_queue_head_t *q, wait_queue_t * wait) __attribute__((regparm(3)));

static inline __attribute__((always_inline)) void __add_wait_queue(wait_queue_head_t *head, wait_queue_t *new)
{
 list_add(&new->task_list, &head->task_list);
}




static inline __attribute__((always_inline)) void __add_wait_queue_tail(wait_queue_head_t *head,
      wait_queue_t *new)
{
 list_add_tail(&new->task_list, &head->task_list);
}

static inline __attribute__((always_inline)) void __remove_wait_queue(wait_queue_head_t *head,
       wait_queue_t *old)
{
 list_del(&old->task_list);
}

void __wake_up(wait_queue_head_t *q, unsigned int mode, int nr, void *key) __attribute__((regparm(3)));
extern void __wake_up_locked(wait_queue_head_t *q, unsigned int mode) __attribute__((regparm(3)));
extern void __wake_up_sync(wait_queue_head_t *q, unsigned int mode, int nr) __attribute__((regparm(3)));
void __wake_up_bit(wait_queue_head_t *, void *, int) __attribute__((regparm(3)));
int __wait_on_bit(wait_queue_head_t *, struct wait_bit_queue *, int (*)(void *), unsigned) __attribute__((regparm(3)));
int __wait_on_bit_lock(wait_queue_head_t *, struct wait_bit_queue *, int (*)(void *), unsigned) __attribute__((regparm(3)));
void wake_up_bit(void *, int) __attribute__((regparm(3)));
int out_of_line_wait_on_bit(void *, int, int (*)(void *), unsigned) __attribute__((regparm(3)));
int out_of_line_wait_on_bit_lock(void *, int, int (*)(void *), unsigned) __attribute__((regparm(3)));
wait_queue_head_t *bit_waitqueue(void *, int) __attribute__((regparm(3)));

static inline __attribute__((always_inline)) void add_wait_queue_exclusive_locked(wait_queue_head_t *q,
         wait_queue_t * wait)
{
 wait->flags |= 0x01;
 __add_wait_queue_tail(q, wait);
}




static inline __attribute__((always_inline)) void remove_wait_queue_locked(wait_queue_head_t *q,
         wait_queue_t * wait)
{
 __remove_wait_queue(q, wait);
}






extern void sleep_on(wait_queue_head_t *q) __attribute__((regparm(3)));
extern long sleep_on_timeout(wait_queue_head_t *q, signed long timeout) __attribute__((regparm(3)));

extern void interruptible_sleep_on(wait_queue_head_t *q) __attribute__((regparm(3)));
extern long interruptible_sleep_on_timeout(wait_queue_head_t *q, signed long timeout) __attribute__((regparm(3)));





void prepare_to_wait(wait_queue_head_t *q, wait_queue_t *wait, int state) __attribute__((regparm(3)));

void prepare_to_wait_exclusive(wait_queue_head_t *q, wait_queue_t *wait, int state) __attribute__((regparm(3)));

void finish_wait(wait_queue_head_t *q, wait_queue_t *wait) __attribute__((regparm(3)));
int autoremove_wake_function(wait_queue_t *wait, unsigned mode, int sync, void *key);
int wake_bit_function(wait_queue_t *wait, unsigned mode, int sync, void *key);

static inline __attribute__((always_inline)) int wait_on_bit(void *word, int bit,
    int (*action)(void *), unsigned mode)
{
 if (!(__builtin_constant_p(bit) ? constant_test_bit((bit),(word)) : variable_test_bit((bit),(word))))
  return 0;
 return out_of_line_wait_on_bit(word, bit, action, mode);
}

static inline __attribute__((always_inline)) int wait_on_bit_lock(void *word, int bit,
    int (*action)(void *), unsigned mode)
{
 if (!test_and_set_bit(bit, word))
  return 0;
 return out_of_line_wait_on_bit_lock(word, bit, action, mode);
}



struct rw_semaphore;






struct rwsem_waiter;

extern struct rw_semaphore *rwsem_down_read_failed(struct rw_semaphore *sem) __attribute__((regparm(3)));
extern struct rw_semaphore *rwsem_down_write_failed(struct rw_semaphore *sem) __attribute__((regparm(3)));
extern struct rw_semaphore *rwsem_wake(struct rw_semaphore *) __attribute__((regparm(3)));
extern struct rw_semaphore *rwsem_downgrade_wake(struct rw_semaphore *sem) __attribute__((regparm(3)));




struct rw_semaphore {
 signed long count;






 spinlock_t wait_lock;
 struct list_head wait_list;



};

extern void __init_rwsem(struct rw_semaphore *sem, const char *name,
    struct lock_class_key *key);

static inline __attribute__((always_inline)) void __down_read(struct rw_semaphore *sem)
{
 __asm__ __volatile__(
  "# beginning down_read\n\t"
"" "  incl      (%%eax)\n\t"
  "  js        2f\n\t"
  "1:\n\t"
  ".subsection 1\n\t" "" ".ifndef " ".text.lock.""md" "\n\t" ".text.lock.""md" ":\n\t" ".endif\n"
  "2:\n\t"
  "  pushl     %%ecx\n\t"
  "  pushl     %%edx\n\t"
  "  call      rwsem_down_read_failed\n\t"
  "  popl      %%edx\n\t"
  "  popl      %%ecx\n\t"
  "  jmp       1b\n"
  ".previous\n\t"
  "# ending down_read\n\t"
  : "+m" (sem->count)
  : "a" (sem)
  : "memory", "cc");
}




static inline __attribute__((always_inline)) int __down_read_trylock(struct rw_semaphore *sem)
{
 __s32 result, tmp;
 __asm__ __volatile__(
  "# beginning __down_read_trylock\n\t"
  "  movl      %0,%1\n\t"
  "1:\n\t"
  "  movl	     %1,%2\n\t"
  "  addl      %3,%2\n\t"
  "  jle	     2f\n\t"
"" "  cmpxchgl  %2,%0\n\t"
  "  jnz	     1b\n\t"
  "2:\n\t"
  "# ending __down_read_trylock\n\t"
  : "+m" (sem->count), "=&a" (result), "=&r" (tmp)
  : "i" (0x00000001)
  : "memory", "cc");
 return result>=0 ? 1 : 0;
}




static inline __attribute__((always_inline)) void __down_write_nested(struct rw_semaphore *sem, int subclass)
{
 int tmp;

 tmp = ((-0x00010000) + 0x00000001);
 __asm__ __volatile__(
  "# beginning down_write\n\t"
"" "  xadd      %%edx,(%%eax)\n\t"
  "  testl     %%edx,%%edx\n\t"
  "  jnz       2f\n\t"
  "1:\n\t"
  ".subsection 1\n\t" "" ".ifndef " ".text.lock.""md" "\n\t" ".text.lock.""md" ":\n\t" ".endif\n"
  "2:\n\t"
  "  pushl     %%ecx\n\t"
  "  call      rwsem_down_write_failed\n\t"
  "  popl      %%ecx\n\t"
  "  jmp       1b\n"
  ".previous\n\t"
  "# ending down_write"
  : "+m" (sem->count), "=d" (tmp)
  : "a" (sem), "1" (tmp)
  : "memory", "cc");
}

static inline __attribute__((always_inline)) void __down_write(struct rw_semaphore *sem)
{
 __down_write_nested(sem, 0);
}




static inline __attribute__((always_inline)) int __down_write_trylock(struct rw_semaphore *sem)
{
 signed long ret = ((__typeof__(*(&sem->count)))__cmpxchg((&sem->count),(unsigned long)(0x00000000), (unsigned long)(((-0x00010000) + 0x00000001)),sizeof(*(&sem->count))));


 if (ret == 0x00000000)
  return 1;
 return 0;
}




static inline __attribute__((always_inline)) void __up_read(struct rw_semaphore *sem)
{
 __s32 tmp = -0x00000001;
 __asm__ __volatile__(
  "# beginning __up_read\n\t"
"" "  xadd      %%edx,(%%eax)\n\t"
  "  js        2f\n\t"
  "1:\n\t"
  ".subsection 1\n\t" "" ".ifndef " ".text.lock.""md" "\n\t" ".text.lock.""md" ":\n\t" ".endif\n"
  "2:\n\t"
  "  decw      %%dx\n\t"
  "  jnz       1b\n\t"
  "  pushl     %%ecx\n\t"
  "  call      rwsem_wake\n\t"
  "  popl      %%ecx\n\t"
  "  jmp       1b\n"
  ".previous\n\t"
  "# ending __up_read\n"
  : "+m" (sem->count), "=d" (tmp)
  : "a" (sem), "1" (tmp)
  : "memory", "cc");
}




static inline __attribute__((always_inline)) void __up_write(struct rw_semaphore *sem)
{
 __asm__ __volatile__(
  "# beginning __up_write\n\t"
  "  movl      %2,%%edx\n\t"
"" "  xaddl     %%edx,(%%eax)\n\t"
  "  jnz       2f\n\t"
  "1:\n\t"
  ".subsection 1\n\t" "" ".ifndef " ".text.lock.""md" "\n\t" ".text.lock.""md" ":\n\t" ".endif\n"
  "2:\n\t"
  "  decw      %%dx\n\t"
  "  jnz       1b\n\t"
  "  pushl     %%ecx\n\t"
  "  call      rwsem_wake\n\t"
  "  popl      %%ecx\n\t"
  "  jmp       1b\n"
  ".previous\n\t"
  "# ending __up_write\n"
  : "+m" (sem->count)
  : "a" (sem), "i" (-((-0x00010000) + 0x00000001))
  : "memory", "cc", "edx");
}




static inline __attribute__((always_inline)) void __downgrade_write(struct rw_semaphore *sem)
{
 __asm__ __volatile__(
  "# beginning __downgrade_write\n\t"
"" "  addl      %2,(%%eax)\n\t"
  "  js        2f\n\t"
  "1:\n\t"
  ".subsection 1\n\t" "" ".ifndef " ".text.lock.""md" "\n\t" ".text.lock.""md" ":\n\t" ".endif\n"
  "2:\n\t"
  "  pushl     %%ecx\n\t"
  "  pushl     %%edx\n\t"
  "  call      rwsem_downgrade_wake\n\t"
  "  popl      %%edx\n\t"
  "  popl      %%ecx\n\t"
  "  jmp       1b\n"
  ".previous\n\t"
  "# ending __downgrade_write\n"
  : "+m" (sem->count)
  : "a" (sem), "i" (-(-0x00010000))
  : "memory", "cc");
}




static inline __attribute__((always_inline)) void rwsem_atomic_add(int delta, struct rw_semaphore *sem)
{
 __asm__ __volatile__(
"" "addl %1,%0"
  : "+m" (sem->count)
  : "ir" (delta));
}




static inline __attribute__((always_inline)) int rwsem_atomic_update(int delta, struct rw_semaphore *sem)
{
 int tmp = delta;

 __asm__ __volatile__(
"" "xadd %0,%1"
  : "+r" (tmp), "+m" (sem->count)
  : : "memory");

 return tmp+delta;
}

static inline __attribute__((always_inline)) int rwsem_is_locked(struct rw_semaphore *sem)
{
 return (sem->count != 0);
}






extern void down_read(struct rw_semaphore *sem);




extern int down_read_trylock(struct rw_semaphore *sem);




extern void down_write(struct rw_semaphore *sem);




extern int down_write_trylock(struct rw_semaphore *sem);




extern void up_read(struct rw_semaphore *sem);




extern void up_write(struct rw_semaphore *sem);




extern void downgrade_write(struct rw_semaphore *sem);


struct semaphore {
 atomic_t count;
 int sleepers;
 wait_queue_head_t wait;
};

static inline __attribute__((always_inline)) void sema_init (struct semaphore *sem, int val)
{






 (((&sem->count)->counter) = (val));
 sem->sleepers = 0;
 init_waitqueue_head(&sem->wait);
}

static inline __attribute__((always_inline)) void init_MUTEX (struct semaphore *sem)
{
 sema_init(sem, 1);
}

static inline __attribute__((always_inline)) void init_MUTEX_LOCKED (struct semaphore *sem)
{
 sema_init(sem, 0);
}

__attribute__((regparm(3))) void __down_failed(void );
__attribute__((regparm(3))) int __down_failed_interruptible(void );
__attribute__((regparm(3))) int __down_failed_trylock(void );
__attribute__((regparm(3))) void __up_wakeup(void );






static inline __attribute__((always_inline)) void down(struct semaphore * sem)
{
 do { __might_sleep("include/asm/semaphore.h", 99); cond_resched(); } while (0);
 __asm__ __volatile__(
  "# atomic down operation\n\t"
  "" "decl %0\n\t"
  "js 2f\n"
  "1:\n"
  ".subsection 1\n\t" "" ".ifndef " ".text.lock.""md" "\n\t" ".text.lock.""md" ":\n\t" ".endif\n"
  "2:\tlea %0,%%eax\n\t"
  "call __down_failed\n\t"
  "jmp 1b\n"
  ".previous\n\t"
  :"+m" (sem->count)
  :
  :"memory","ax");
}





static inline __attribute__((always_inline)) int down_interruptible(struct semaphore * sem)
{
 int result;

 do { __might_sleep("include/asm/semaphore.h", 123); cond_resched(); } while (0);
 __asm__ __volatile__(
  "# atomic interruptible down operation\n\t"
  "" "decl %1\n\t"
  "js 2f\n\t"
  "xorl %0,%0\n"
  "1:\n"
  ".subsection 1\n\t" "" ".ifndef " ".text.lock.""md" "\n\t" ".text.lock.""md" ":\n\t" ".endif\n"
  "2:\tlea %1,%%eax\n\t"
  "call __down_failed_interruptible\n\t"
  "jmp 1b\n"
  ".previous\n\t"
  :"=a" (result), "+m" (sem->count)
  :
  :"memory");
 return result;
}





static inline __attribute__((always_inline)) int down_trylock(struct semaphore * sem)
{
 int result;

 __asm__ __volatile__(
  "# atomic interruptible down operation\n\t"
  "" "decl %1\n\t"
  "js 2f\n\t"
  "xorl %0,%0\n"
  "1:\n"
  ".subsection 1\n\t" "" ".ifndef " ".text.lock.""md" "\n\t" ".text.lock.""md" ":\n\t" ".endif\n"
  "2:\tlea %1,%%eax\n\t"
  "call __down_failed_trylock\n\t"
  "jmp 1b\n"
  ".previous\n\t"
  :"=a" (result), "+m" (sem->count)
  :
  :"memory");
 return result;
}







static inline __attribute__((always_inline)) void up(struct semaphore * sem)
{
 __asm__ __volatile__(
  "# atomic up operation\n\t"
  "" "incl %0\n\t"
  "jle 2f\n"
  "1:\n"
  ".subsection 1\n\t" "" ".ifndef " ".text.lock.""md" "\n\t" ".text.lock.""md" ":\n\t" ".endif\n"
  "2:\tlea %0,%%eax\n\t"
  "call __up_wakeup\n\t"
  "jmp 1b\n"
  ".previous\n\t"
  ".subsection 0\n"
  :"+m" (sem->count)
  :
  :"memory","ax");
}




struct pt_regs {
 long ebx;
 long ecx;
 long edx;
 long esi;
 long edi;
 long ebp;
 long eax;
 int xds;
 int xes;
 long orig_eax;
 long eip;
 int xcs;
 long eflags;
 long esp;
 int xss;
};

struct task_struct;
extern void send_sigtrap(struct task_struct *tsk, struct pt_regs *regs, int error_code);

static inline __attribute__((always_inline)) int user_mode(struct pt_regs *regs)
{
 return (regs->xcs & 3) != 0;
}
static inline __attribute__((always_inline)) int user_mode_vm(struct pt_regs *regs)
{
 return ((regs->xcs & 3) | (regs->eflags & 0x00020000)) != 0;
}



typedef struct {
 int size;
 struct semaphore sem;
 void *ldt;
 void *vdso;
} mm_context_t;












typedef unsigned long cputime_t;

typedef u64 cputime64_t;





extern void cpu_idle(void);

static inline __attribute__((always_inline)) int up_smp_call_function(void)
{
 return 0;
}

static inline __attribute__((always_inline)) void smp_send_reschedule(int cpu) { }

void smp_setup_processor_id(void);







struct ipc_perm
{
 __kernel_key_t key;
 __kernel_uid_t uid;
 __kernel_gid_t gid;
 __kernel_uid_t cuid;
 __kernel_gid_t cgid;
 __kernel_mode_t mode;
 unsigned short seq;
};




struct ipc64_perm
{
 __kernel_key_t key;
 __kernel_uid32_t uid;
 __kernel_gid32_t gid;
 __kernel_uid32_t cuid;
 __kernel_gid32_t cgid;
 __kernel_mode_t mode;
 unsigned short __pad1;
 unsigned short seq;
 unsigned short __pad2;
 unsigned long __unused1;
 unsigned long __unused2;
};


struct kern_ipc_perm
{
 spinlock_t lock;
 int deleted;
 key_t key;
 uid_t uid;
 gid_t gid;
 uid_t cuid;
 gid_t cgid;
 mode_t mode;
 unsigned long seq;
 void *security;
};


struct semid_ds {
 struct ipc_perm sem_perm;
 __kernel_time_t sem_otime;
 __kernel_time_t sem_ctime;
 struct sem *sem_base;
 struct sem_queue *sem_pending;
 struct sem_queue **sem_pending_last;
 struct sem_undo *undo;
 unsigned short sem_nsems;
};




struct semid64_ds {
 struct ipc64_perm sem_perm;
 __kernel_time_t sem_otime;
 unsigned long __unused1;
 __kernel_time_t sem_ctime;
 unsigned long __unused2;
 unsigned long sem_nsems;
 unsigned long __unused3;
 unsigned long __unused4;
};



struct sembuf {
 unsigned short sem_num;
 short sem_op;
 short sem_flg;
};


union semun {
 int val;
 struct semid_ds *buf;
 unsigned short *array;
 struct seminfo *__buf;
 void *__pad;
};

struct seminfo {
 int semmap;
 int semmni;
 int semmns;
 int semmnu;
 int semmsl;
 int semopm;
 int semume;
 int semusz;
 int semvmx;
 int semaem;
};

struct task_struct;


struct sem {
 int semval;
 int sempid;
};


struct sem_array {
 struct kern_ipc_perm sem_perm;
 int sem_id;
 time_t sem_otime;
 time_t sem_ctime;
 struct sem *sem_base;
 struct sem_queue *sem_pending;
 struct sem_queue **sem_pending_last;
 struct sem_undo *undo;
 unsigned long sem_nsems;
};


struct sem_queue {
 struct sem_queue * next;
 struct sem_queue ** prev;
 struct task_struct* sleeper;
 struct sem_undo * undo;
 int pid;
 int status;
 struct sem_array * sma;
 int id;
 struct sembuf * sops;
 int nsops;
 int alter;
};




struct sem_undo {
 struct sem_undo * proc_next;
 struct sem_undo * id_next;
 int semid;
 short * semadj;
};




struct sem_undo_list {
 atomic_t refcnt;
 spinlock_t lock;
 struct sem_undo *proc_list;
};

struct sysv_sem {
 struct sem_undo_list *undo_list;
};



extern int copy_semundo(unsigned long clone_flags, struct task_struct *tsk);
extern void exit_sem(struct task_struct *tsk);







struct siginfo;

typedef unsigned long old_sigset_t;

typedef struct {
 unsigned long sig[(64 / 32)];
} sigset_t;



typedef void __signalfn_t(int);
typedef __signalfn_t *__sighandler_t;

typedef void __restorefn_t(void);
typedef __restorefn_t *__sigrestore_t;



struct old_sigaction {
 __sighandler_t sa_handler;
 old_sigset_t sa_mask;
 unsigned long sa_flags;
 __sigrestore_t sa_restorer;
};

struct sigaction {
 __sighandler_t sa_handler;
 unsigned long sa_flags;
 __sigrestore_t sa_restorer;
 sigset_t sa_mask;
};

struct k_sigaction {
 struct sigaction sa;
};

typedef struct sigaltstack {
 void *ss_sp;
 int ss_flags;
 size_t ss_size;
} stack_t;

static __inline__ __attribute__((always_inline)) void __gen_sigaddset(sigset_t *set, int _sig)
{
 __asm__("btsl %1,%0" : "+m"(*set) : "Ir"(_sig - 1) : "cc");
}

static __inline__ __attribute__((always_inline)) void __const_sigaddset(sigset_t *set, int _sig)
{
 unsigned long sig = _sig - 1;
 set->sig[sig / 32] |= 1 << (sig % 32);
}







static __inline__ __attribute__((always_inline)) void __gen_sigdelset(sigset_t *set, int _sig)
{
 __asm__("btrl %1,%0" : "+m"(*set) : "Ir"(_sig - 1) : "cc");
}

static __inline__ __attribute__((always_inline)) void __const_sigdelset(sigset_t *set, int _sig)
{
 unsigned long sig = _sig - 1;
 set->sig[sig / 32] &= ~(1 << (sig % 32));
}

static __inline__ __attribute__((always_inline)) int __const_sigismember(sigset_t *set, int _sig)
{
 unsigned long sig = _sig - 1;
 return 1 & (set->sig[sig / 32] >> (sig % 32));
}

static __inline__ __attribute__((always_inline)) int __gen_sigismember(sigset_t *set, int _sig)
{
 int ret;
 __asm__("btl %2,%1\n\tsbbl %0,%0"
  : "=r"(ret) : "m"(*set), "Ir"(_sig-1) : "cc");
 return ret;
}






static __inline__ __attribute__((always_inline)) int sigfindinword(unsigned long word)
{
 __asm__("bsfl %1,%0" : "=r"(word) : "rm"(word) : "cc");
 return word;
}

struct pt_regs;












typedef union sigval {
 int sival_int;
 void *sival_ptr;
} sigval_t;

typedef struct siginfo {
 int si_signo;
 int si_errno;
 int si_code;

 union {
  int _pad[((128 - (3 * sizeof(int))) / sizeof(int))];


  struct {
   pid_t _pid;
   uid_t _uid;
  } _kill;


  struct {
   timer_t _tid;
   int _overrun;
   char _pad[sizeof( uid_t) - sizeof(int)];
   sigval_t _sigval;
   int _sys_private;
  } _timer;


  struct {
   pid_t _pid;
   uid_t _uid;
   sigval_t _sigval;
  } _rt;


  struct {
   pid_t _pid;
   uid_t _uid;
   int _status;
   clock_t _utime;
   clock_t _stime;
  } _sigchld;


  struct {
   void *_addr;



  } _sigfault;


  struct {
   long _band;
   int _fd;
  } _sigpoll;
 } _sifields;
} siginfo_t;

typedef struct sigevent {
 sigval_t sigev_value;
 int sigev_signo;
 int sigev_notify;
 union {
  int _pad[((64 - (sizeof(int) * 2 + sizeof(sigval_t))) / sizeof(int))];
   int _tid;

  struct {
   void (*_function)(sigval_t);
   void *_attribute;
  } _sigev_thread;
 } _sigev_un;
} sigevent_t;







struct siginfo;
void do_schedule_next_timer(struct siginfo *info);





static inline __attribute__((always_inline)) void copy_siginfo(struct siginfo *to, struct siginfo *from)
{
 if (from->si_code < 0)
  (__builtin_constant_p(sizeof(*to)) ? __constant_memcpy((to),(from),(sizeof(*to))) : __memcpy((to),(from),(sizeof(*to))));
 else

  (__builtin_constant_p((3 * sizeof(int)) + sizeof(from->_sifields._sigchld)) ? __constant_memcpy((to),(from),((3 * sizeof(int)) + sizeof(from->_sifields._sigchld))) : __memcpy((to),(from),((3 * sizeof(int)) + sizeof(from->_sifields._sigchld))));
}



extern int copy_siginfo_to_user(struct siginfo *to, struct siginfo *from);



struct sigqueue {
 struct list_head list;
 int flags;
 siginfo_t info;
 struct user_struct *user;
};




struct sigpending {
 struct list_head list;
 sigset_t signal;
};

static inline __attribute__((always_inline)) int sigisemptyset(sigset_t *set)
{
 extern void _NSIG_WORDS_is_unsupported_size(void);
 switch ((64 / 32)) {
 case 4:
  return (set->sig[3] | set->sig[2] |
   set->sig[1] | set->sig[0]) == 0;
 case 2:
  return (set->sig[1] | set->sig[0]) == 0;
 case 1:
  return set->sig[0] == 0;
 default:
  _NSIG_WORDS_is_unsupported_size();
  return 0;
 }
}

static inline __attribute__((always_inline)) void sigorsets(sigset_t *r, const sigset_t *a, const sigset_t *b) { extern void _NSIG_WORDS_is_unsupported_size(void); unsigned long a0, a1, a2, a3, b0, b1, b2, b3; switch ((64 / 32)) { case 4: a3 = a->sig[3]; a2 = a->sig[2]; b3 = b->sig[3]; b2 = b->sig[2]; r->sig[3] = ((a3) | (b3)); r->sig[2] = ((a2) | (b2)); case 2: a1 = a->sig[1]; b1 = b->sig[1]; r->sig[1] = ((a1) | (b1)); case 1: a0 = a->sig[0]; b0 = b->sig[0]; r->sig[0] = ((a0) | (b0)); break; default: _NSIG_WORDS_is_unsupported_size(); } }


static inline __attribute__((always_inline)) void sigandsets(sigset_t *r, const sigset_t *a, const sigset_t *b) { extern void _NSIG_WORDS_is_unsupported_size(void); unsigned long a0, a1, a2, a3, b0, b1, b2, b3; switch ((64 / 32)) { case 4: a3 = a->sig[3]; a2 = a->sig[2]; b3 = b->sig[3]; b2 = b->sig[2]; r->sig[3] = ((a3) & (b3)); r->sig[2] = ((a2) & (b2)); case 2: a1 = a->sig[1]; b1 = b->sig[1]; r->sig[1] = ((a1) & (b1)); case 1: a0 = a->sig[0]; b0 = b->sig[0]; r->sig[0] = ((a0) & (b0)); break; default: _NSIG_WORDS_is_unsupported_size(); } }


static inline __attribute__((always_inline)) void signandsets(sigset_t *r, const sigset_t *a, const sigset_t *b) { extern void _NSIG_WORDS_is_unsupported_size(void); unsigned long a0, a1, a2, a3, b0, b1, b2, b3; switch ((64 / 32)) { case 4: a3 = a->sig[3]; a2 = a->sig[2]; b3 = b->sig[3]; b2 = b->sig[2]; r->sig[3] = ((a3) & ~(b3)); r->sig[2] = ((a2) & ~(b2)); case 2: a1 = a->sig[1]; b1 = b->sig[1]; r->sig[1] = ((a1) & ~(b1)); case 1: a0 = a->sig[0]; b0 = b->sig[0]; r->sig[0] = ((a0) & ~(b0)); break; default: _NSIG_WORDS_is_unsupported_size(); } }

static inline __attribute__((always_inline)) void signotset(sigset_t *set) { extern void _NSIG_WORDS_is_unsupported_size(void); switch ((64 / 32)) { case 4: set->sig[3] = (~(set->sig[3])); set->sig[2] = (~(set->sig[2])); case 2: set->sig[1] = (~(set->sig[1])); case 1: set->sig[0] = (~(set->sig[0])); break; default: _NSIG_WORDS_is_unsupported_size(); } }




static inline __attribute__((always_inline)) void sigemptyset(sigset_t *set)
{
 switch ((64 / 32)) {
 default:
  (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(sigset_t))) ? __constant_c_and_count_memset(((set)),((0x01010101UL*(unsigned char)(0))),((sizeof(sigset_t)))) : __constant_c_memset(((set)),((0x01010101UL*(unsigned char)(0))),((sizeof(sigset_t))))) : (__builtin_constant_p((sizeof(sigset_t))) ? __memset_generic((((set))),(((0))),(((sizeof(sigset_t))))) : __memset_generic(((set)),((0)),((sizeof(sigset_t))))));
  break;
 case 2: set->sig[1] = 0;
 case 1: set->sig[0] = 0;
  break;
 }
}

static inline __attribute__((always_inline)) void sigfillset(sigset_t *set)
{
 switch ((64 / 32)) {
 default:
  (__builtin_constant_p(-1) ? (__builtin_constant_p((sizeof(sigset_t))) ? __constant_c_and_count_memset(((set)),((0x01010101UL*(unsigned char)(-1))),((sizeof(sigset_t)))) : __constant_c_memset(((set)),((0x01010101UL*(unsigned char)(-1))),((sizeof(sigset_t))))) : (__builtin_constant_p((sizeof(sigset_t))) ? __memset_generic((((set))),(((-1))),(((sizeof(sigset_t))))) : __memset_generic(((set)),((-1)),((sizeof(sigset_t))))));
  break;
 case 2: set->sig[1] = -1;
 case 1: set->sig[0] = -1;
  break;
 }
}



static inline __attribute__((always_inline)) void sigaddsetmask(sigset_t *set, unsigned long mask)
{
 set->sig[0] |= mask;
}

static inline __attribute__((always_inline)) void sigdelsetmask(sigset_t *set, unsigned long mask)
{
 set->sig[0] &= ~mask;
}

static inline __attribute__((always_inline)) int sigtestsetmask(sigset_t *set, unsigned long mask)
{
 return (set->sig[0] & mask) != 0;
}

static inline __attribute__((always_inline)) void siginitset(sigset_t *set, unsigned long mask)
{
 set->sig[0] = mask;
 switch ((64 / 32)) {
 default:
  (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(long)*((64 / 32)-1))) ? __constant_c_and_count_memset(((&set->sig[1])),((0x01010101UL*(unsigned char)(0))),((sizeof(long)*((64 / 32)-1)))) : __constant_c_memset(((&set->sig[1])),((0x01010101UL*(unsigned char)(0))),((sizeof(long)*((64 / 32)-1))))) : (__builtin_constant_p((sizeof(long)*((64 / 32)-1))) ? __memset_generic((((&set->sig[1]))),(((0))),(((sizeof(long)*((64 / 32)-1))))) : __memset_generic(((&set->sig[1])),((0)),((sizeof(long)*((64 / 32)-1))))));
  break;
 case 2: set->sig[1] = 0;
 case 1: ;
 }
}

static inline __attribute__((always_inline)) void siginitsetinv(sigset_t *set, unsigned long mask)
{
 set->sig[0] = ~mask;
 switch ((64 / 32)) {
 default:
  (__builtin_constant_p(-1) ? (__builtin_constant_p((sizeof(long)*((64 / 32)-1))) ? __constant_c_and_count_memset(((&set->sig[1])),((0x01010101UL*(unsigned char)(-1))),((sizeof(long)*((64 / 32)-1)))) : __constant_c_memset(((&set->sig[1])),((0x01010101UL*(unsigned char)(-1))),((sizeof(long)*((64 / 32)-1))))) : (__builtin_constant_p((sizeof(long)*((64 / 32)-1))) ? __memset_generic((((&set->sig[1]))),(((-1))),(((sizeof(long)*((64 / 32)-1))))) : __memset_generic(((&set->sig[1])),((-1)),((sizeof(long)*((64 / 32)-1))))));
  break;
 case 2: set->sig[1] = -1;
 case 1: ;
 }
}



static inline __attribute__((always_inline)) void init_sigpending(struct sigpending *sig)
{
 sigemptyset(&sig->signal);
 INIT_LIST_HEAD(&sig->list);
}

extern void flush_sigqueue(struct sigpending *queue);


static inline __attribute__((always_inline)) int valid_signal(unsigned long sig)
{
 return sig <= 64 ? 1 : 0;
}

extern int group_send_sig_info(int sig, struct siginfo *info, struct task_struct *p);
extern int __group_send_sig_info(int, struct siginfo *, struct task_struct *);
extern long do_sigpending(void *, unsigned long);
extern int sigprocmask(int, sigset_t *, sigset_t *);

struct pt_regs;
extern int get_signal_to_deliver(siginfo_t *info, struct k_sigaction *return_ka, struct pt_regs *regs, void *cookie);







extern unsigned securebits;





struct dentry;
struct vfsmount;

struct fs_struct {
 atomic_t count;
 rwlock_t lock;
 int umask;
 struct dentry * root, * pwd, * altroot;
 struct vfsmount * rootmnt, * pwdmnt, * altrootmnt;
};







extern void exit_fs(struct task_struct *);
extern void set_fs_altroot(void);
extern void set_fs_root(struct fs_struct *, struct vfsmount *, struct dentry *);
extern void set_fs_pwd(struct fs_struct *, struct vfsmount *, struct dentry *);
extern struct fs_struct *copy_fs_struct(struct fs_struct *);
extern void put_fs_struct(struct fs_struct *);




struct completion {
 unsigned int done;
 wait_queue_head_t wait;
};

static inline __attribute__((always_inline)) void init_completion(struct completion *x)
{
 x->done = 0;
 init_waitqueue_head(&x->wait);
}

extern void wait_for_completion(struct completion *) __attribute__((regparm(3)));
extern int wait_for_completion_interruptible(struct completion *x) __attribute__((regparm(3)));
extern unsigned long wait_for_completion_timeout(struct completion *x, unsigned long timeout) __attribute__((regparm(3)));

extern unsigned long wait_for_completion_interruptible_timeout( struct completion *x, unsigned long timeout) __attribute__((regparm(3)));


extern void complete(struct completion *) __attribute__((regparm(3)));
extern void complete_all(struct completion *) __attribute__((regparm(3)));













typedef struct kmem_cache kmem_cache_t;









typedef int (*initcall_t)(void);
typedef void (*exitcall_t)(void);

extern initcall_t __con_initcall_start[], __con_initcall_end[];
extern initcall_t __security_initcall_start[], __security_initcall_end[];


extern char saved_command_line[];


extern void setup_arch(char **);

struct obs_kernel_param {
 const char *str;
 int (*setup_func)(char *);
 int early;
};

void __attribute__ ((__section__ (".init.text"))) parse_early_param(void);


struct free_area {
 struct list_head free_list;
 unsigned long nr_free;
};

struct pglist_data;

enum zone_stat_item {
 NR_ANON_PAGES,
 NR_FILE_MAPPED,

 NR_FILE_PAGES,
 NR_SLAB,
 NR_PAGETABLE,
 NR_FILE_DIRTY,
 NR_WRITEBACK,
 NR_UNSTABLE_NFS,
 NR_BOUNCE,

 NR_VM_ZONE_STAT_ITEMS };

struct per_cpu_pages {
 int count;
 int high;
 int batch;
 struct list_head list;
};

struct per_cpu_pageset {
 struct per_cpu_pages pcp[2];




} ;

struct zone {

 unsigned long free_pages;
 unsigned long pages_min, pages_low, pages_high;

 unsigned long lowmem_reserve[4];

 struct per_cpu_pageset pageset[1];




 spinlock_t lock;




 struct free_area free_area[11];





 spinlock_t lru_lock;
 struct list_head active_list;
 struct list_head inactive_list;
 unsigned long nr_scan_active;
 unsigned long nr_scan_inactive;
 unsigned long nr_active;
 unsigned long nr_inactive;
 unsigned long pages_scanned;
 int all_unreclaimable;


 atomic_t reclaim_in_progress;


 atomic_long_t vm_stat[NR_VM_ZONE_STAT_ITEMS];

 int prev_priority;




 wait_queue_head_t * wait_table;
 unsigned long wait_table_hash_nr_entries;
 unsigned long wait_table_bits;




 struct pglist_data *zone_pgdat;

 unsigned long zone_start_pfn;

 unsigned long spanned_pages;
 unsigned long present_pages;




 char *name;
} ;

struct zonelist {
 struct zone *zones[(1 << 0) * 4 + 1];
};

struct bootmem_data;
typedef struct pglist_data {
 struct zone node_zones[4];
 struct zonelist node_zonelists[((0x07 + 1) / 2 + 1)];
 int nr_zones;

 struct page *node_mem_map;

 struct bootmem_data *bdata;

 unsigned long node_start_pfn;
 unsigned long node_present_pages;
 unsigned long node_spanned_pages;

 int node_id;
 wait_queue_head_t kswapd_wait;
 struct task_struct *kswapd;
 int kswapd_max_order;
} pg_data_t;













struct mutex {

 atomic_t count;
 spinlock_t wait_lock;
 struct list_head wait_list;

};





struct mutex_waiter {
 struct list_head list;
 struct task_struct *task;




};

extern void __mutex_init(struct mutex *lock, const char *name,
    struct lock_class_key *key);







static inline __attribute__((always_inline)) int __attribute__((regparm(3))) mutex_is_locked(struct mutex *lock)
{
 return ((&lock->count)->counter) != 1;
}





extern void __attribute__((regparm(3))) mutex_lock(struct mutex *lock);
extern int __attribute__((regparm(3))) mutex_lock_interruptible(struct mutex *lock);

extern int __attribute__((regparm(3))) mutex_trylock(struct mutex *lock);
extern void __attribute__((regparm(3))) mutex_unlock(struct mutex *lock);


struct notifier_block {
 int (*notifier_call)(struct notifier_block *, unsigned long, void *);
 struct notifier_block *next;
 int priority;
};

struct atomic_notifier_head {
 spinlock_t lock;
 struct notifier_block *head;
};

struct blocking_notifier_head {
 struct rw_semaphore rwsem;
 struct notifier_block *head;
};

struct raw_notifier_head {
 struct notifier_block *head;
};

extern int atomic_notifier_chain_register(struct atomic_notifier_head *,
  struct notifier_block *);
extern int blocking_notifier_chain_register(struct blocking_notifier_head *,
  struct notifier_block *);
extern int raw_notifier_chain_register(struct raw_notifier_head *,
  struct notifier_block *);

extern int atomic_notifier_chain_unregister(struct atomic_notifier_head *,
  struct notifier_block *);
extern int blocking_notifier_chain_unregister(struct blocking_notifier_head *,
  struct notifier_block *);
extern int raw_notifier_chain_unregister(struct raw_notifier_head *,
  struct notifier_block *);

extern int atomic_notifier_call_chain(struct atomic_notifier_head *,
  unsigned long val, void *v);
extern int blocking_notifier_call_chain(struct blocking_notifier_head *,
  unsigned long val, void *v);
extern int raw_notifier_call_chain(struct raw_notifier_head *,
  unsigned long val, void *v);


struct page;
struct zone;
struct pglist_data;

static inline __attribute__((always_inline)) void pgdat_resize_lock(struct pglist_data *p, unsigned long *f) {}
static inline __attribute__((always_inline)) void pgdat_resize_unlock(struct pglist_data *p, unsigned long *f) {}
static inline __attribute__((always_inline)) void pgdat_resize_init(struct pglist_data *pgdat) {}

static inline __attribute__((always_inline)) unsigned zone_span_seqbegin(struct zone *zone)
{
 return 0;
}
static inline __attribute__((always_inline)) int zone_span_seqretry(struct zone *zone, unsigned iv)
{
 return 0;
}
static inline __attribute__((always_inline)) void zone_span_writelock(struct zone *zone) {}
static inline __attribute__((always_inline)) void zone_span_writeunlock(struct zone *zone) {}
static inline __attribute__((always_inline)) void zone_seqlock_init(struct zone *zone) {}

static inline __attribute__((always_inline)) int mhp_notimplemented(const char *func)
{
 printk("<4>" "%s() called, with CONFIG_MEMORY_HOTPLUG disabled\n", func);
 dump_stack();
 return -38;
}


static inline __attribute__((always_inline)) int __remove_pages(struct zone *zone, unsigned long start_pfn,
 unsigned long nr_pages)
{
 printk("<4>" "%s() called, not yet supported\n", (__func__));
 dump_stack();
 return -38;
}

extern int add_memory(int nid, u64 start, u64 size);
extern int arch_add_memory(int nid, u64 start, u64 size);
extern int remove_memory(u64 start, u64 size);


void __get_zone_counts(unsigned long *active, unsigned long *inactive,
   unsigned long *free, struct pglist_data *pgdat);
void get_zone_counts(unsigned long *active, unsigned long *inactive,
   unsigned long *free);
void build_all_zonelists(void);
void wakeup_kswapd(struct zone *zone, int order);
int zone_watermark_ok(struct zone *z, int order, unsigned long mark,
  int classzone_idx, int alloc_flags);

extern int init_currently_empty_zone(struct zone *zone, unsigned long start_pfn,
         unsigned long size);




static inline __attribute__((always_inline)) void memory_present(int nid, unsigned long start, unsigned long end) {}

static inline __attribute__((always_inline)) int populated_zone(struct zone *zone)
{
 return (!!zone->present_pages);
}

static inline __attribute__((always_inline)) int is_highmem_idx(int idx)
{
 return (idx == 3);
}

static inline __attribute__((always_inline)) int is_normal_idx(int idx)
{
 return (idx == 2);
}







static inline __attribute__((always_inline)) int is_highmem(struct zone *zone)
{
 return zone == zone->zone_pgdat->node_zones + 3;
}

static inline __attribute__((always_inline)) int is_normal(struct zone *zone)
{
 return zone == zone->zone_pgdat->node_zones + 2;
}

static inline __attribute__((always_inline)) int is_dma32(struct zone *zone)
{
 return zone == zone->zone_pgdat->node_zones + 1;
}

static inline __attribute__((always_inline)) int is_dma(struct zone *zone)
{
 return zone == zone->zone_pgdat->node_zones + 0;
}


struct ctl_table;
struct file;
int min_free_kbytes_sysctl_handler(struct ctl_table *, int, struct file *,
     void *, size_t *, loff_t *);
extern int sysctl_lowmem_reserve_ratio[4 -1];
int lowmem_reserve_ratio_sysctl_handler(struct ctl_table *, int, struct file *,
     void *, size_t *, loff_t *);
int percpu_pagelist_fraction_sysctl_handler(struct ctl_table *, int, struct file *,
     void *, size_t *, loff_t *);
int sysctl_min_unmapped_ratio_sysctl_handler(struct ctl_table *, int,
   struct file *, void *, size_t *, loff_t *);
int sysctl_min_slab_ratio_sysctl_handler(struct ctl_table *, int,
   struct file *, void *, size_t *, loff_t *);










extern cpumask_t cpu_coregroup_map(int cpu);









extern struct pglist_data contig_page_data;

extern struct pglist_data *first_online_pgdat(void);
extern struct pglist_data *next_online_pgdat(struct pglist_data *pgdat);
extern struct zone *next_zone(struct zone *zone);

void memory_present(int nid, unsigned long start, unsigned long end);
unsigned long __attribute__ ((__section__ (".init.text"))) node_memmap_size_bytes(int, unsigned long, unsigned long);




struct vm_area_struct;

static inline __attribute__((always_inline)) int gfp_zone(gfp_t gfp)
{
 int zone = 0x07 & ( int) gfp;
 do { if (__builtin_expect(!!((zone >= ((0x07 + 1) / 2 + 1))!=0), 0)) __asm__ __volatile__( "ud2\n" "\t.word %c0\n" "\t.long %c1\n" : : "i" (82), "i" ("include/linux/gfp.h")); } while(0);
 return zone;
}

static inline __attribute__((always_inline)) void arch_free_page(struct page *page, int order) { }


extern struct page *
__alloc_pages(gfp_t, unsigned int, struct zonelist *) __attribute__((regparm(3)));

static inline __attribute__((always_inline)) struct page *alloc_pages_node(int nid, gfp_t gfp_mask,
      unsigned int order)
{
 if (__builtin_expect(!!(order >= 11), 0))
  return ((void *)0);


 if (nid < 0)
  nid = ((0));

 return __alloc_pages(gfp_mask, order,
  (&contig_page_data)->node_zonelists + gfp_zone(gfp_mask));
}

extern unsigned long __get_free_pages(gfp_t gfp_mask, unsigned int order) __attribute__((regparm(3)));
extern unsigned long get_zeroed_page(gfp_t gfp_mask) __attribute__((regparm(3)));







extern void __free_pages(struct page *page, unsigned int order) __attribute__((regparm(3)));
extern void free_pages(unsigned long addr, unsigned int order) __attribute__((regparm(3)));
extern void free_hot_page(struct page *page) __attribute__((regparm(3)));
extern void free_cold_page(struct page *page) __attribute__((regparm(3)));




void page_alloc_init(void);



static inline __attribute__((always_inline)) void drain_node_pages(int node) { };


extern void __attribute__ ((__section__ (".init.text"))) kmem_cache_init(void);

extern kmem_cache_t *kmem_cache_create(const char *, size_t, size_t, unsigned long,
           void (*)(void *, kmem_cache_t *, unsigned long),
           void (*)(void *, kmem_cache_t *, unsigned long));
extern int kmem_cache_destroy(kmem_cache_t *);
extern int kmem_cache_shrink(kmem_cache_t *);
extern void *kmem_cache_alloc(kmem_cache_t *, gfp_t);
extern void *kmem_cache_zalloc(struct kmem_cache *, gfp_t);
extern void kmem_cache_free(kmem_cache_t *, void *);
extern unsigned int kmem_cache_size(kmem_cache_t *);
extern const char *kmem_cache_name(kmem_cache_t *);
extern kmem_cache_t *kmem_find_general_cachep(size_t size, gfp_t gfpflags);


struct cache_sizes {
 size_t cs_size;
 kmem_cache_t *cs_cachep;
 kmem_cache_t *cs_dmacachep;
};
extern struct cache_sizes malloc_sizes[];

extern void *__kmalloc(size_t, gfp_t);

static inline __attribute__((always_inline)) void *kmalloc(size_t size, gfp_t flags)
{
 if (__builtin_constant_p(size)) {
  int i = 0;







 if (size <= 32) goto found; else i++;

 if (size <= 64) goto found; else i++;



 if (size <= 128) goto found; else i++;



 if (size <= 256) goto found; else i++;
 if (size <= 512) goto found; else i++;
 if (size <= 1024) goto found; else i++;
 if (size <= 2048) goto found; else i++;
 if (size <= 4096) goto found; else i++;
 if (size <= 8192) goto found; else i++;
 if (size <= 16384) goto found; else i++;
 if (size <= 32768) goto found; else i++;
 if (size <= 65536) goto found; else i++;
 if (size <= 131072) goto found; else i++;


  {
   extern void __you_cannot_kmalloc_that_much(void);
   __you_cannot_kmalloc_that_much();
  }
found:
  return kmem_cache_alloc((flags & (( gfp_t)0x01u)) ?
   malloc_sizes[i].cs_dmacachep :
   malloc_sizes[i].cs_cachep, flags);
 }
 return __kmalloc(size, flags);
}

extern void *__kzalloc(size_t, gfp_t);






static inline __attribute__((always_inline)) void *kzalloc(size_t size, gfp_t flags)
{
 if (__builtin_constant_p(size)) {
  int i = 0;







 if (size <= 32) goto found; else i++;

 if (size <= 64) goto found; else i++;



 if (size <= 128) goto found; else i++;



 if (size <= 256) goto found; else i++;
 if (size <= 512) goto found; else i++;
 if (size <= 1024) goto found; else i++;
 if (size <= 2048) goto found; else i++;
 if (size <= 4096) goto found; else i++;
 if (size <= 8192) goto found; else i++;
 if (size <= 16384) goto found; else i++;
 if (size <= 32768) goto found; else i++;
 if (size <= 65536) goto found; else i++;
 if (size <= 131072) goto found; else i++;


  {
   extern void __you_cannot_kzalloc_that_much(void);
   __you_cannot_kzalloc_that_much();
  }
found:
  return kmem_cache_zalloc((flags & (( gfp_t)0x01u)) ?
   malloc_sizes[i].cs_dmacachep :
   malloc_sizes[i].cs_cachep, flags);
 }
 return __kzalloc(size, flags);
}







static inline __attribute__((always_inline)) void *kcalloc(size_t n, size_t size, gfp_t flags)
{
 if (n != 0 && size > (~0UL) / n)
  return ((void *)0);
 return kzalloc(n * size, flags);
}

extern void kfree(const void *);
extern unsigned int ksize(const void *);
extern int slab_is_available(void);





static inline __attribute__((always_inline)) void *kmem_cache_alloc_node(kmem_cache_t *cachep, gfp_t flags, int node)
{
 return kmem_cache_alloc(cachep, flags);
}
static inline __attribute__((always_inline)) void *kmalloc_node(size_t size, gfp_t flags, int node)
{
 return kmalloc(size, flags);
}


extern int kmem_cache_reap(int) __attribute__((regparm(3)));
extern int kmem_ptr_validate(kmem_cache_t *cachep, void *ptr) __attribute__((regparm(3)));

extern kmem_cache_t *vm_area_cachep;
extern kmem_cache_t *names_cachep;
extern kmem_cache_t *files_cachep;
extern kmem_cache_t *filp_cachep;
extern kmem_cache_t *fs_cachep;
extern kmem_cache_t *sighand_cachep;
extern kmem_cache_t *bio_cachep;

extern atomic_t slab_reclaim_pages;


static inline __attribute__((always_inline)) void *__alloc_percpu(size_t size)
{
 void *ret = kmalloc(size, ((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u)));
 if (ret)
  (__builtin_constant_p(0) ? (__builtin_constant_p((size)) ? __constant_c_and_count_memset(((ret)),((0x01010101UL*(unsigned char)(0))),((size))) : __constant_c_memset(((ret)),((0x01010101UL*(unsigned char)(0))),((size)))) : (__builtin_constant_p((size)) ? __memset_generic((((ret))),(((0))),(((size)))) : __memset_generic(((ret)),((0)),((size)))));
 return ret;
}
static inline __attribute__((always_inline)) void free_percpu(const void *ptr)
{
 kfree(ptr);
}


struct rcu_head {
 struct rcu_head *next;
 void (*func)(struct rcu_head *head);
};

struct rcu_ctrlblk {
 long cur;
 long completed;
 int next_pending;

 spinlock_t lock ;
 cpumask_t cpumask;

} ;


static inline __attribute__((always_inline)) int rcu_batch_before(long a, long b)
{
        return (a - b) < 0;
}


static inline __attribute__((always_inline)) int rcu_batch_after(long a, long b)
{
        return (a - b) > 0;
}






struct rcu_data {

 long quiescbatch;
 int passed_quiesc;
 int qs_pending;


 long batch;
 struct rcu_head *nxtlist;
 struct rcu_head **nxttail;
 long qlen;
 struct rcu_head *curlist;
 struct rcu_head **curtail;
 struct rcu_head *donelist;
 struct rcu_head **donetail;
 long blimit;
 int cpu;
 struct rcu_head barrier;



};

extern __typeof__(struct rcu_data) per_cpu__rcu_data;
extern __typeof__(struct rcu_data) per_cpu__rcu_bh_data;







static inline __attribute__((always_inline)) void rcu_qsctr_inc(int cpu)
{
 struct rcu_data *rdp = &(*((void)(cpu), &per_cpu__rcu_data));
 rdp->passed_quiesc = 1;
}
static inline __attribute__((always_inline)) void rcu_bh_qsctr_inc(int cpu)
{
 struct rcu_data *rdp = &(*((void)(cpu), &per_cpu__rcu_bh_data));
 rdp->passed_quiesc = 1;
}

extern int rcu_pending(int cpu);
extern int rcu_needs_cpu(int cpu);

extern void rcu_init(void);
extern void rcu_check_callbacks(int cpu, int user);
extern void rcu_restart_cpu(int cpu);
extern long rcu_batches_completed(void);
extern long rcu_batches_completed_bh(void);


extern void call_rcu(struct rcu_head *head, void (*func)(struct rcu_head *head)) __attribute__((regparm(3)));

extern void call_rcu_bh(struct rcu_head *head, void (*func)(struct rcu_head *head)) __attribute__((regparm(3)));

extern void synchronize_rcu(void);
void synchronize_idle(void);
extern void rcu_barrier(void);


enum pid_type
{
 PIDTYPE_PID,
 PIDTYPE_PGID,
 PIDTYPE_SID,
 PIDTYPE_MAX
};

struct pid
{
 atomic_t count;

 int nr;
 struct hlist_node pid_chain;

 struct hlist_head tasks[PIDTYPE_MAX];
 struct rcu_head rcu;
};

struct pid_link
{
 struct hlist_node node;
 struct pid *pid;
};

static inline __attribute__((always_inline)) struct pid *get_pid(struct pid *pid)
{
 if (pid)
  atomic_inc(&pid->count);
 return pid;
}

extern void put_pid(struct pid *pid) __attribute__((regparm(3)));
extern struct task_struct *pid_task(struct pid *pid, enum pid_type) __attribute__((regparm(3)));
extern struct task_struct *get_pid_task(struct pid *pid, enum pid_type) __attribute__((regparm(3)));






extern int attach_pid(struct task_struct *task, enum pid_type type, int nr) __attribute__((regparm(3)));


extern void detach_pid(struct task_struct *task, enum pid_type) __attribute__((regparm(3)));





extern struct pid *find_pid(int nr) __attribute__((regparm(3)));




extern struct pid *find_get_pid(int nr);

extern struct pid *alloc_pid(void);
extern void free_pid(struct pid *pid) __attribute__((regparm(3)));





typedef struct { } seccomp_t;



static inline __attribute__((always_inline)) int has_secure_computing(struct thread_info *ti)
{
 return 0;
}









struct robust_list {
 struct robust_list *next;
};

struct robust_list_head {



 struct robust_list list;







 long futex_offset;

 struct robust_list *list_op_pending;
};

long do_futex(u32 *uaddr, int op, u32 val, unsigned long timeout,
       u32 *uaddr2, u32 val2, u32 val3);

extern int
handle_futex_death(u32 *uaddr, struct task_struct *curr, int pi);


extern void exit_robust_list(struct task_struct *curr);
extern void exit_pi_state_list(struct task_struct *curr);





struct plist_head {
 struct list_head prio_list;
 struct list_head node_list;



};

struct plist_node {
 int prio;
 struct plist_head plist;
};

static inline __attribute__((always_inline)) void
plist_head_init(struct plist_head *head, spinlock_t *lock)
{
 INIT_LIST_HEAD(&head->prio_list);
 INIT_LIST_HEAD(&head->node_list);



}







static inline __attribute__((always_inline)) void plist_node_init(struct plist_node *node, int prio)
{
 node->prio = prio;
 plist_head_init(&node->plist, ((void *)0));
}

extern void plist_add(struct plist_node *node, struct plist_head *head);
extern void plist_del(struct plist_node *node, struct plist_head *head);

static inline __attribute__((always_inline)) int plist_head_empty(const struct plist_head *head)
{
 return list_empty(&head->node_list);
}






static inline __attribute__((always_inline)) int plist_node_empty(const struct plist_node *node)
{
 return plist_head_empty(&node->plist);
}

static inline __attribute__((always_inline)) struct plist_node* plist_first(const struct plist_head *head)
{
 return ({ const typeof( ((struct plist_node *)0)->plist.node_list ) *__mptr = (head->node_list.next); (struct plist_node *)( (char *)__mptr - __builtin_offsetof(struct plist_node,plist.node_list) );});

}


struct rt_mutex {
 spinlock_t wait_lock;
 struct plist_head wait_list;
 struct task_struct *owner;






};

struct rt_mutex_waiter;
struct hrtimer_sleeper;






 static inline __attribute__((always_inline)) int rt_mutex_debug_check_no_locks_freed(const void *from,
             unsigned long len)
 {
 return 0;
 }

static inline __attribute__((always_inline)) int rt_mutex_is_locked(struct rt_mutex *lock)
{
 return lock->owner != ((void *)0);
}

extern void __rt_mutex_init(struct rt_mutex *lock, const char *name);
extern void rt_mutex_destroy(struct rt_mutex *lock);

extern void rt_mutex_lock(struct rt_mutex *lock);
extern int rt_mutex_lock_interruptible(struct rt_mutex *lock,
      int detect_deadlock);
extern int rt_mutex_timed_lock(struct rt_mutex *lock,
     struct hrtimer_sleeper *timeout,
     int detect_deadlock);

extern int rt_mutex_trylock(struct rt_mutex *lock);

extern void rt_mutex_unlock(struct rt_mutex *lock);











struct task_struct;

struct rusage {
 struct timeval ru_utime;
 struct timeval ru_stime;
 long ru_maxrss;
 long ru_ixrss;
 long ru_idrss;
 long ru_isrss;
 long ru_minflt;
 long ru_majflt;
 long ru_nswap;
 long ru_inblock;
 long ru_oublock;
 long ru_msgsnd;
 long ru_msgrcv;
 long ru_nsignals;
 long ru_nvcsw;
 long ru_nivcsw;
};

struct rlimit {
 unsigned long rlim_cur;
 unsigned long rlim_max;
};









int getrusage(struct task_struct *p, int who, struct rusage *ru);









struct tvec_t_base_s;

struct timer_list {
 struct list_head entry;
 unsigned long expires;

 void (*function)(unsigned long);
 unsigned long data;

 struct tvec_t_base_s *base;
};

extern struct tvec_t_base_s boot_tvec_bases;

void __attribute__((regparm(3))) init_timer(struct timer_list * timer);

static inline __attribute__((always_inline)) void setup_timer(struct timer_list * timer,
    void (*function)(unsigned long),
    unsigned long data)
{
 timer->function = function;
 timer->data = data;
 init_timer(timer);
}

static inline __attribute__((always_inline)) int timer_pending(const struct timer_list * timer)
{
 return timer->entry.next != ((void *)0);
}

extern void add_timer_on(struct timer_list *timer, int cpu);
extern int del_timer(struct timer_list * timer);
extern int __mod_timer(struct timer_list *timer, unsigned long expires);
extern int mod_timer(struct timer_list *timer, unsigned long expires);

extern unsigned long next_timer_interrupt(void);

static inline __attribute__((always_inline)) void add_timer(struct timer_list *timer)
{
 do { if (__builtin_expect(!!((timer_pending(timer))!=0), 0)) __asm__ __volatile__( "ud2\n" "\t.word %c0\n" "\t.long %c1\n" : : "i" (82), "i" ("include/linux/timer.h")); } while(0);
 __mod_timer(timer, timer->expires);
}

extern void init_timers(void);
extern void run_local_timers(void);
struct hrtimer;
extern int it_real_fn(struct hrtimer *);





typedef union {
 s64 tv64;

} ktime_t;

static inline __attribute__((always_inline)) ktime_t ktime_set(const long secs, const unsigned long nsecs)
{




 return (ktime_t) { .tv64 = (s64)secs * 1000000000L + (s64)nsecs };
}

static inline __attribute__((always_inline)) ktime_t timespec_to_ktime(struct timespec ts)
{
 return ktime_set(ts.tv_sec, ts.tv_nsec);
}


static inline __attribute__((always_inline)) ktime_t timeval_to_ktime(struct timeval tv)
{
 return ktime_set(tv.tv_sec, tv.tv_usec * 1000L);
}

extern void ktime_get_ts(struct timespec *ts);








enum hrtimer_mode {
 HRTIMER_ABS,
 HRTIMER_REL,
};

enum hrtimer_restart {
 HRTIMER_NORESTART,
 HRTIMER_RESTART,
};



struct hrtimer_base;

struct hrtimer {
 struct rb_node node;
 ktime_t expires;
 int (*function)(struct hrtimer *);
 struct hrtimer_base *base;
};

struct hrtimer_sleeper {
 struct hrtimer timer;
 struct task_struct *task;
};

struct hrtimer_base {
 clockid_t index;
 spinlock_t lock;
 struct rb_root active;
 struct rb_node *first;
 ktime_t resolution;
 ktime_t (*get_time)(void);
 ktime_t (*get_softirq_time)(void);
 struct hrtimer *curr_timer;
 ktime_t softirq_time;
 struct lock_class_key lock_key;
};

extern void hrtimer_init(struct hrtimer *timer, clockid_t which_clock,
    enum hrtimer_mode mode);


extern int hrtimer_start(struct hrtimer *timer, ktime_t tim,
    const enum hrtimer_mode mode);
extern int hrtimer_cancel(struct hrtimer *timer);
extern int hrtimer_try_to_cancel(struct hrtimer *timer);




extern ktime_t hrtimer_get_remaining(const struct hrtimer *timer);
extern int hrtimer_get_res(const clockid_t which_clock, struct timespec *tp);





static inline __attribute__((always_inline)) int hrtimer_active(const struct hrtimer *timer)
{
 return ((struct rb_node *)((&timer->node)->rb_parent_color & ~3)) != &timer->node;
}


extern unsigned long
hrtimer_forward(struct hrtimer *timer, ktime_t now, ktime_t interval);


extern long hrtimer_nanosleep(struct timespec *rqtp,
         struct timespec *rmtp,
         const enum hrtimer_mode mode,
         const clockid_t clockid);

extern void hrtimer_init_sleeper(struct hrtimer_sleeper *sl,
     struct task_struct *tsk);


extern void hrtimer_run_queues(void);


extern void __attribute__ ((__section__ (".init.text"))) hrtimers_init(void);




struct exec_domain;
struct futex_pi_state;

extern unsigned long avenrun[];

extern unsigned long total_forks;
extern int nr_threads;
extern int last_pid;
extern __typeof__(unsigned long) per_cpu__process_counts;
extern int nr_processes(void);
extern unsigned long nr_running(void);
extern unsigned long nr_uninterruptible(void);
extern unsigned long nr_active(void);
extern unsigned long nr_iowait(void);
extern unsigned long weighted_cpuload(const int cpu);

extern rwlock_t tasklist_lock;
extern spinlock_t mmlist_lock;

struct task_struct;

extern void sched_init(void);
extern void sched_init_smp(void);
extern void init_idle(struct task_struct *idle, int cpu);

extern cpumask_t nohz_cpu_mask;

extern void show_state(void);
extern void show_regs(struct pt_regs *);






extern void show_stack(struct task_struct *task, unsigned long *sp);

void io_schedule(void);
long io_schedule_timeout(long timeout);

extern void cpu_init (void);
extern void trap_init(void);
extern void update_process_times(int user);
extern void scheduler_tick(void);


extern void softlockup_tick(void);
extern void spawn_softlockup_task(void);
extern void touch_softlockup_watchdog(void);

extern int in_sched_functions(unsigned long addr);


extern signed long schedule_timeout(signed long timeout) __attribute__((regparm(3)));
extern signed long schedule_timeout_interruptible(signed long timeout);
extern signed long schedule_timeout_uninterruptible(signed long timeout);
 __attribute__((regparm(0))) void schedule(void);

struct namespace;




extern int sysctl_max_map_count;








struct workqueue_struct;

struct work_struct {
 unsigned long pending;
 struct list_head entry;
 void (*func)(void *);
 void *data;
 void *wq_data;
 struct timer_list timer;
};

struct execute_work {
 struct work_struct work;
};

extern struct workqueue_struct *__create_workqueue(const char *name,
          int singlethread);



extern void destroy_workqueue(struct workqueue_struct *wq);

extern int queue_work(struct workqueue_struct *wq, struct work_struct *work) __attribute__((regparm(3)));
extern int queue_delayed_work(struct workqueue_struct *wq, struct work_struct *work, unsigned long delay) __attribute__((regparm(3)));
extern int queue_delayed_work_on(int cpu, struct workqueue_struct *wq,
 struct work_struct *work, unsigned long delay);
extern void flush_workqueue(struct workqueue_struct *wq) __attribute__((regparm(3)));

extern int schedule_work(struct work_struct *work) __attribute__((regparm(3)));
extern int schedule_delayed_work(struct work_struct *work, unsigned long delay) __attribute__((regparm(3)));

extern int schedule_delayed_work_on(int cpu, struct work_struct *work, unsigned long delay);
extern int schedule_on_each_cpu(void (*func)(void *info), void *info);
extern void flush_scheduled_work(void);
extern int current_is_keventd(void);
extern int keventd_up(void);

extern void init_workqueues(void);
void cancel_rearming_delayed_work(struct work_struct *work);
void cancel_rearming_delayed_workqueue(struct workqueue_struct *,
           struct work_struct *);
int execute_in_process_context(void (*fn)(void *), void *,
          struct execute_work *);






static inline __attribute__((always_inline)) int cancel_delayed_work(struct work_struct *work)
{
 int ret;

 ret = del_timer(&work->timer);
 if (ret)
  clear_bit(0, &work->pending);
 return ret;
}



typedef unsigned long aio_context_t;

enum {
 IOCB_CMD_PREAD = 0,
 IOCB_CMD_PWRITE = 1,
 IOCB_CMD_FSYNC = 2,
 IOCB_CMD_FDSYNC = 3,




 IOCB_CMD_NOOP = 6,
};


struct io_event {
 __u64 data;
 __u64 obj;
 __s64 res;
 __s64 res2;
};

struct iocb {

 __u64 aio_data;
 __u32 aio_key, aio_reserved1;



 __u16 aio_lio_opcode;
 __s16 aio_reqprio;
 __u32 aio_fildes;

 __u64 aio_buf;
 __u64 aio_nbytes;
 __s64 aio_offset;


 __u64 aio_reserved2;
 __u64 aio_reserved3;
};







struct kioctx;

struct kiocb {
 struct list_head ki_run_list;
 long ki_flags;
 int ki_users;
 unsigned ki_key;

 struct file *ki_filp;
 struct kioctx *ki_ctx;
 int (*ki_cancel)(struct kiocb *, struct io_event *);
 ssize_t (*ki_retry)(struct kiocb *);
 void (*ki_dtor)(struct kiocb *);

 union {
  void *user;
  struct task_struct *tsk;
 } ki_obj;

 __u64 ki_user_data;
 wait_queue_t ki_wait;
 loff_t ki_pos;

 void *private;

 unsigned short ki_opcode;
 size_t ki_nbytes;
 char *ki_buf;
 size_t ki_left;
 long ki_retried;
 long ki_kicked;
 long ki_queued;

 struct list_head ki_list;

};

struct aio_ring {
 unsigned id;
 unsigned nr;
 unsigned head;
 unsigned tail;

 unsigned magic;
 unsigned compat_features;
 unsigned incompat_features;
 unsigned header_length;


 struct io_event io_events[0];
};




struct aio_ring_info {
 unsigned long mmap_base;
 unsigned long mmap_size;

 struct page **ring_pages;
 spinlock_t ring_lock;
 long nr_pages;

 unsigned nr, tail;

 struct page *internal_pages[8];
};

struct kioctx {
 atomic_t users;
 int dead;
 struct mm_struct *mm;


 unsigned long user_id;
 struct kioctx *next;

 wait_queue_head_t wait;

 spinlock_t ctx_lock;

 int reqs_active;
 struct list_head active_reqs;
 struct list_head run_list;


 unsigned max_reqs;

 struct aio_ring_info ring_info;

 struct work_struct wq;
};


extern unsigned aio_max_size;

extern ssize_t wait_on_sync_kiocb(struct kiocb *iocb) __attribute__((regparm(3)));
extern int aio_put_req(struct kiocb *iocb) __attribute__((regparm(3)));
extern void kick_iocb(struct kiocb *iocb) __attribute__((regparm(3)));
extern int aio_complete(struct kiocb *iocb, long res, long res2) __attribute__((regparm(3)));
extern void __put_ioctx(struct kioctx *ctx) __attribute__((regparm(3)));
struct mm_struct;
extern void exit_aio(struct mm_struct *mm) __attribute__((regparm(3)));
extern struct kioctx *lookup_ioctx(unsigned long ctx_id);
extern int io_submit_one(struct kioctx *ctx, struct iocb *user_iocb, struct iocb *iocb) __attribute__((regparm(3)));



struct kioctx *lookup_ioctx(unsigned long ctx_id);
int io_submit_one(struct kioctx *ctx, struct iocb *user_iocb, struct iocb *iocb) __attribute__((regparm(3)));

static inline __attribute__((always_inline)) struct kiocb *list_kiocb(struct list_head *h)
{
 return ({ const typeof( ((struct kiocb *)0)->ki_list ) *__mptr = (h); (struct kiocb *)( (char *)__mptr - __builtin_offsetof(struct kiocb,ki_list) );});
}


extern unsigned long aio_nr;
extern unsigned long aio_max_nr;


extern unsigned long
arch_get_unmapped_area(struct file *, unsigned long, unsigned long,
         unsigned long, unsigned long);
extern unsigned long
arch_get_unmapped_area_topdown(struct file *filp, unsigned long addr,
     unsigned long len, unsigned long pgoff,
     unsigned long flags);
extern void arch_unmap_area(struct mm_struct *, unsigned long);
extern void arch_unmap_area_topdown(struct mm_struct *, unsigned long);

typedef unsigned long mm_counter_t;

struct mm_struct {
 struct vm_area_struct * mmap;
 struct rb_root mm_rb;
 struct vm_area_struct * mmap_cache;
 unsigned long (*get_unmapped_area) (struct file *filp,
    unsigned long addr, unsigned long len,
    unsigned long pgoff, unsigned long flags);
 void (*unmap_area) (struct mm_struct *mm, unsigned long addr);
 unsigned long mmap_base;
 unsigned long task_size;
 unsigned long cached_hole_size;
 unsigned long free_area_cache;
 pgd_t * pgd;
 atomic_t mm_users;
 atomic_t mm_count;
 int map_count;
 struct rw_semaphore mmap_sem;
 spinlock_t page_table_lock;

 struct list_head mmlist;







 mm_counter_t _file_rss;
 mm_counter_t _anon_rss;

 unsigned long hiwater_rss;
 unsigned long hiwater_vm;

 unsigned long total_vm, locked_vm, shared_vm, exec_vm;
 unsigned long stack_vm, reserved_vm, def_flags, nr_ptes;
 unsigned long start_code, end_code, start_data, end_data;
 unsigned long start_brk, brk, start_stack;
 unsigned long arg_start, arg_end, env_start, env_end;

 unsigned long saved_auxv[44];

 unsigned dumpable:2;
 cpumask_t cpu_vm_mask;


 mm_context_t context;


 unsigned long swap_token_time;
 char recent_pagein;


 int core_waiters;
 struct completion *core_startup_done, core_done;


 rwlock_t ioctx_list_lock;
 struct kioctx *ioctx_list;
};

struct sighand_struct {
 atomic_t count;
 struct k_sigaction action[64];
 spinlock_t siglock;
};

struct pacct_struct {
 int ac_flag;
 long ac_exitcode;
 unsigned long ac_mem;
 cputime_t ac_utime, ac_stime;
 unsigned long ac_minflt, ac_majflt;
};

struct signal_struct {
 atomic_t count;
 atomic_t live;

 wait_queue_head_t wait_chldexit;


 struct task_struct *curr_target;


 struct sigpending shared_pending;


 int group_exit_code;





 struct task_struct *group_exit_task;
 int notify_count;


 int group_stop_count;
 unsigned int flags;


 struct list_head posix_timers;


 struct hrtimer real_timer;
 struct task_struct *tsk;
 ktime_t it_real_incr;


 cputime_t it_prof_expires, it_virt_expires;
 cputime_t it_prof_incr, it_virt_incr;


 pid_t pgrp;
 pid_t tty_old_pgrp;
 pid_t session;

 int leader;

 struct tty_struct *tty;







 cputime_t utime, stime, cutime, cstime;
 unsigned long nvcsw, nivcsw, cnvcsw, cnivcsw;
 unsigned long min_flt, maj_flt, cmin_flt, cmaj_flt;







 unsigned long long sched_time;

 struct rlimit rlim[15];

 struct list_head cpu_timers[3];




 struct key *session_keyring;
 struct key *process_keyring;


 struct pacct_struct pacct;


 spinlock_t stats_lock;
 struct taskstats *stats;

};

struct user_struct {
 atomic_t __count;
 atomic_t processes;
 atomic_t files;
 atomic_t sigpending;

 atomic_t inotify_watches;
 atomic_t inotify_devs;


 unsigned long mq_bytes;
 unsigned long locked_shm;


 struct key *uid_keyring;
 struct key *session_keyring;



 struct list_head uidhash_list;
 uid_t uid;
};

extern struct user_struct *find_user(uid_t);

extern struct user_struct root_user;


struct backing_dev_info;
struct reclaim_state;


struct sched_info {

 unsigned long cpu_time,
   run_delay,
   pcnt;


 unsigned long last_arrival,
   last_queued;
};



extern struct file_operations proc_schedstat_operations;



struct task_delay_info {
 spinlock_t lock;
 unsigned int flags;

 struct timespec blkio_start, blkio_end;
 u64 blkio_delay;
 u64 swapin_delay;
 u32 blkio_count;

 u32 swapin_count;

};


static inline __attribute__((always_inline)) int sched_info_on(void)
{

 return 1;






}

enum idle_type
{
 SCHED_IDLE,
 NOT_IDLE,
 NEWLY_IDLE,
 MAX_IDLE_TYPES
};

struct io_context;
void exit_io_context(void);
struct cpuset;



struct group_info {
 int ngroups;
 atomic_t usage;
 gid_t small_block[32];
 int nblocks;
 gid_t *blocks[0];
};

extern struct group_info *groups_alloc(int gidsetsize);
extern void groups_free(struct group_info *group_info);
extern int set_current_groups(struct group_info *group_info);
extern int groups_search(struct group_info *group_info, gid_t grp);







static inline __attribute__((always_inline)) void prefetch_stack(struct task_struct *t) { }


struct audit_context;
struct mempolicy;
struct pipe_inode_info;

enum sleep_type {
 SLEEP_NORMAL,
 SLEEP_NONINTERACTIVE,
 SLEEP_INTERACTIVE,
 SLEEP_INTERRUPTED,
};

struct prio_array;

struct task_struct {
 volatile long state;
 struct thread_info *thread_info;
 atomic_t usage;
 unsigned long flags;
 unsigned long ptrace;

 int lock_depth;






 int load_weight;
 int prio, static_prio, normal_prio;
 struct list_head run_list;
 struct prio_array *array;

 unsigned short ioprio;
 unsigned int btrace_seq;

 unsigned long sleep_avg;
 unsigned long long timestamp, last_ran;
 unsigned long long sched_time;
 enum sleep_type sleep_type;

 unsigned long policy;
 cpumask_t cpus_allowed;
 unsigned int time_slice, first_time_slice;


 struct sched_info sched_info;


 struct list_head tasks;




 struct list_head ptrace_children;
 struct list_head ptrace_list;

 struct mm_struct *mm, *active_mm;


 struct linux_binfmt *binfmt;
 long exit_state;
 int exit_code, exit_signal;
 int pdeath_signal;

 unsigned long personality;
 unsigned did_exec:1;
 pid_t pid;
 pid_t tgid;





 struct task_struct *real_parent;
 struct task_struct *parent;




 struct list_head children;
 struct list_head sibling;
 struct task_struct *group_leader;


 struct pid_link pids[PIDTYPE_MAX];
 struct list_head thread_group;

 struct completion *vfork_done;
 int *set_child_tid;
 int *clear_child_tid;

 unsigned long rt_priority;
 cputime_t utime, stime;
 unsigned long nvcsw, nivcsw;
 struct timespec start_time;

 unsigned long min_flt, maj_flt;

   cputime_t it_prof_expires, it_virt_expires;
 unsigned long long it_sched_expires;
 struct list_head cpu_timers[3];


 uid_t uid,euid,suid,fsuid;
 gid_t gid,egid,sgid,fsgid;
 struct group_info *group_info;
 kernel_cap_t cap_effective, cap_inheritable, cap_permitted;
 unsigned keep_capabilities:1;
 struct user_struct *user;

 struct key *request_key_auth;
 struct key *thread_keyring;
 unsigned char jit_keyring;

 int oomkilladj;
 char comm[16];




 int link_count, total_link_count;

 struct sysv_sem sysvsem;

 struct thread_struct thread;

 struct fs_struct *fs;

 struct files_struct *files;

 struct namespace *namespace;

 struct signal_struct *signal;
 struct sighand_struct *sighand;

 sigset_t blocked, real_blocked;
 sigset_t saved_sigmask;
 struct sigpending pending;

 unsigned long sas_ss_sp;
 size_t sas_ss_size;
 int (*notifier)(void *priv);
 void *notifier_data;
 sigset_t *notifier_mask;

 void *security;
 struct audit_context *audit_context;
 seccomp_t seccomp;


    u32 parent_exec_id;
    u32 self_exec_id;

 spinlock_t alloc_lock;


 spinlock_t pi_lock;



 struct plist_head pi_waiters;

 struct rt_mutex_waiter *pi_blocked_on;

 void *journal_info;


 struct reclaim_state *reclaim_state;

 struct backing_dev_info *backing_dev_info;

 struct io_context *io_context;

 unsigned long ptrace_message;
 siginfo_t *last_siginfo;






 wait_queue_t *io_wait;

 u64 rchar, wchar, syscr, syscw;

 u64 acct_rss_mem1;
 u64 acct_vm_mem1;
 clock_t acct_stimexpd;

 struct robust_list_head *robust_list;



 struct list_head pi_state_list;
 struct futex_pi_state *pi_state_cache;

 atomic_t fs_excl;
 struct rcu_head rcu;




 struct pipe_inode_info *splice_pipe;

 struct task_delay_info *delays;

};

static inline __attribute__((always_inline)) pid_t process_group(struct task_struct *tsk)
{
 return tsk->signal->pgrp;
}

static inline __attribute__((always_inline)) int pid_alive(struct task_struct *p)
{
 return p->pids[PIDTYPE_PID].pid != ((void *)0);
}

extern void free_task(struct task_struct *tsk);


extern void __put_task_struct(struct task_struct *t);

static inline __attribute__((always_inline)) void put_task_struct(struct task_struct *t)
{
 if (atomic_dec_and_test(&t->usage))
  __put_task_struct(t);
}

static inline __attribute__((always_inline)) int set_cpus_allowed(struct task_struct *p, cpumask_t new_mask)
{
 if (!(__builtin_constant_p((0)) ? constant_test_bit(((0)),((new_mask).bits)) : variable_test_bit(((0)),((new_mask).bits))))
  return -22;
 return 0;
}


extern unsigned long long sched_clock(void);
extern unsigned long long
current_sched_time(const struct task_struct *current_task);

static inline __attribute__((always_inline)) void idle_task_exit(void) {}


extern void sched_idle_next(void);


extern int rt_mutex_getprio(struct task_struct *p);
extern void rt_mutex_setprio(struct task_struct *p, int prio);
extern void rt_mutex_adjust_pi(struct task_struct *p);

extern void set_user_nice(struct task_struct *p, long nice);
extern int task_prio(const struct task_struct *p);
extern int task_nice(const struct task_struct *p);
extern int can_nice(const struct task_struct *p, const int nice);
extern int task_curr(const struct task_struct *p);
extern int idle_cpu(int cpu);
extern int sched_setscheduler(struct task_struct *, int, struct sched_param *);
extern struct task_struct *idle_task(int cpu);
extern struct task_struct *curr_task(int cpu);
extern void set_curr_task(int cpu, struct task_struct *p);

void yield(void);




extern struct exec_domain default_exec_domain;

union thread_union {
 struct thread_info thread_info;
 unsigned long stack[(4096)/sizeof(long)];
};


static inline __attribute__((always_inline)) int kstack_end(void *addr)
{



 return !(((unsigned long)addr+sizeof(void*)-1) & ((4096)-sizeof(void*)));
}


extern union thread_union init_thread_union;
extern struct task_struct init_task;

extern struct mm_struct init_mm;


extern struct task_struct *find_task_by_pid_type(int type, int pid);
extern void set_special_pids(pid_t session, pid_t pgrp);
extern void __set_special_pids(pid_t session, pid_t pgrp);


extern struct user_struct * alloc_uid(uid_t);
static inline __attribute__((always_inline)) struct user_struct *get_uid(struct user_struct *u)
{
 atomic_inc(&u->__count);
 return u;
}
extern void free_uid(struct user_struct *);
extern void switch_uid(struct user_struct *);



extern void do_timer(struct pt_regs *);

extern int wake_up_state(struct task_struct * tsk, unsigned int state) __attribute__((regparm(3)));
extern int wake_up_process(struct task_struct * tsk) __attribute__((regparm(3)));
extern void wake_up_new_task(struct task_struct * tsk, unsigned long clone_flags) __attribute__((regparm(3)));




 static inline __attribute__((always_inline)) void kick_process(struct task_struct *tsk) { }

extern void sched_fork(struct task_struct * p, int clone_flags) __attribute__((regparm(3)));
extern void sched_exit(struct task_struct * p) __attribute__((regparm(3)));

extern int in_group_p(gid_t);
extern int in_egroup_p(gid_t);

extern void proc_caches_init(void);
extern void flush_signals(struct task_struct *);
extern void flush_signal_handlers(struct task_struct *, int force_default);
extern int dequeue_signal(struct task_struct *tsk, sigset_t *mask, siginfo_t *info);

static inline __attribute__((always_inline)) int dequeue_signal_lock(struct task_struct *tsk, sigset_t *mask, siginfo_t *info)
{
 unsigned long flags;
 int ret;

 flags = _spin_lock_irqsave(&tsk->sighand->siglock);
 ret = dequeue_signal(tsk, mask, info);
 _spin_unlock_irqrestore(&tsk->sighand->siglock, flags);

 return ret;
}

extern void block_all_signals(int (*notifier)(void *priv), void *priv,
         sigset_t *mask);
extern void unblock_all_signals(void);
extern void release_task(struct task_struct * p);
extern int send_sig_info(int, struct siginfo *, struct task_struct *);
extern int send_group_sig_info(int, struct siginfo *, struct task_struct *);
extern int force_sigsegv(int, struct task_struct *);
extern int force_sig_info(int, struct siginfo *, struct task_struct *);
extern int __kill_pg_info(int sig, struct siginfo *info, pid_t pgrp);
extern int kill_pg_info(int, struct siginfo *, pid_t);
extern int kill_proc_info(int, struct siginfo *, pid_t);
extern int kill_proc_info_as_uid(int, struct siginfo *, pid_t, uid_t, uid_t, u32);
extern void do_notify_parent(struct task_struct *, int);
extern void force_sig(int, struct task_struct *);
extern void force_sig_specific(int, struct task_struct *);
extern int send_sig(int, struct task_struct *, int);
extern void zap_other_threads(struct task_struct *p);
extern int kill_pg(pid_t, int, int);
extern int kill_proc(pid_t, int, int);
extern struct sigqueue *sigqueue_alloc(void);
extern void sigqueue_free(struct sigqueue *);
extern int send_sigqueue(int, struct sigqueue *, struct task_struct *);
extern int send_group_sigqueue(int, struct sigqueue *, struct task_struct *);
extern int do_sigaction(int, struct k_sigaction *, struct k_sigaction *);
extern int do_sigaltstack(const stack_t *, stack_t *, unsigned long);






static inline __attribute__((always_inline)) int is_si_special(const struct siginfo *info)
{
 return info <= ((struct siginfo *) 2);
}



static inline __attribute__((always_inline)) int on_sig_stack(unsigned long sp)
{
 return (sp - __vericon_dummy_current->sas_ss_sp < __vericon_dummy_current->sas_ss_size);
}

static inline __attribute__((always_inline)) int sas_ss_flags(unsigned long sp)
{
 return (__vericon_dummy_current->sas_ss_size == 0 ? 2
  : on_sig_stack(sp) ? 1 : 0);
}




extern struct mm_struct * mm_alloc(void);


extern void __mmdrop(struct mm_struct *) __attribute__((regparm(3)));
static inline __attribute__((always_inline)) void mmdrop(struct mm_struct * mm)
{
 if (atomic_dec_and_test(&mm->mm_count))
  __mmdrop(mm);
}


extern void mmput(struct mm_struct *);

extern struct mm_struct *get_task_mm(struct task_struct *task);

extern void mm_release(struct task_struct *, struct mm_struct *);

extern int copy_thread(int, unsigned long, unsigned long, unsigned long, struct task_struct *, struct pt_regs *);
extern void flush_thread(void);
extern void exit_thread(void);

extern void exit_files(struct task_struct *);
extern void __cleanup_signal(struct signal_struct *);
extern void __cleanup_sighand(struct sighand_struct *);
extern void exit_itimers(struct signal_struct *);

extern void do_group_exit(int);

extern void daemonize(const char *, ...);
extern int allow_signal(int);
extern int disallow_signal(int);
extern struct task_struct *child_reaper;

extern int do_execve(char *, char * *, char * *, struct pt_regs *);
extern long do_fork(unsigned long, unsigned long, struct pt_regs *, unsigned long, int *, int *);
struct task_struct *fork_idle(int);

extern void set_task_comm(struct task_struct *tsk, char *from);
extern void get_task_comm(char *to, struct task_struct *tsk);

static inline __attribute__((always_inline)) struct task_struct *next_thread(const struct task_struct *p)
{
 return ({ const typeof( ((struct task_struct *)0)->thread_group ) *__mptr = (({ typeof(p->thread_group.next) _________p1 = p->thread_group.next; do { } while(0); (_________p1); })); (struct task_struct *)( (char *)__mptr - __builtin_offsetof(struct task_struct,thread_group) );});

}

static inline __attribute__((always_inline)) int thread_group_empty(struct task_struct *p)
{
 return list_empty(&p->thread_group);
}

static inline __attribute__((always_inline)) void task_lock(struct task_struct *p)
{
 _spin_lock(&p->alloc_lock);
}

static inline __attribute__((always_inline)) void task_unlock(struct task_struct *p)
{
 _spin_unlock(&p->alloc_lock);
}

extern struct sighand_struct *lock_task_sighand(struct task_struct *tsk,
       unsigned long *flags);

static inline __attribute__((always_inline)) void unlock_task_sighand(struct task_struct *tsk,
      unsigned long *flags)
{
 _spin_unlock_irqrestore(&tsk->sighand->siglock, *flags);
}






static inline __attribute__((always_inline)) void setup_thread_stack(struct task_struct *p, struct task_struct *org)
{
 *(p)->thread_info = *(org)->thread_info;
 (p)->thread_info->task = p;
}

static inline __attribute__((always_inline)) unsigned long *end_of_stack(struct task_struct *p)
{
 return (unsigned long *)(p->thread_info + 1);
}






static inline __attribute__((always_inline)) void set_tsk_thread_flag(struct task_struct *tsk, int flag)
{
 set_ti_thread_flag((tsk)->thread_info, flag);
}

static inline __attribute__((always_inline)) void clear_tsk_thread_flag(struct task_struct *tsk, int flag)
{
 clear_ti_thread_flag((tsk)->thread_info, flag);
}

static inline __attribute__((always_inline)) int test_and_set_tsk_thread_flag(struct task_struct *tsk, int flag)
{
 return test_and_set_ti_thread_flag((tsk)->thread_info, flag);
}

static inline __attribute__((always_inline)) int test_and_clear_tsk_thread_flag(struct task_struct *tsk, int flag)
{
 return test_and_clear_ti_thread_flag((tsk)->thread_info, flag);
}

static inline __attribute__((always_inline)) int test_tsk_thread_flag(struct task_struct *tsk, int flag)
{
 return test_ti_thread_flag((tsk)->thread_info, flag);
}

static inline __attribute__((always_inline)) void set_tsk_need_resched(struct task_struct *tsk)
{
 set_tsk_thread_flag(tsk,3);
}

static inline __attribute__((always_inline)) void clear_tsk_need_resched(struct task_struct *tsk)
{
 clear_tsk_thread_flag(tsk,3);
}

static inline __attribute__((always_inline)) int signal_pending(struct task_struct *p)
{
 return __builtin_expect(!!(test_tsk_thread_flag(p,2)), 0);
}

static inline __attribute__((always_inline)) int need_resched(void)
{
 return __builtin_expect(!!(test_ti_thread_flag(current_thread_info(), 3)), 0);
}

extern int cond_resched(void);
extern int cond_resched_lock(spinlock_t * lock);
extern int cond_resched_softirq(void);

static inline __attribute__((always_inline)) int lock_need_resched(spinlock_t *lock)
{
 if (0 || need_resched())
  return 1;
 return 0;
}





extern void recalc_sigpending_tsk(struct task_struct *t) __attribute__((regparm(3)));
extern void recalc_sigpending(void);

extern void signal_wake_up(struct task_struct *t, int resume_stopped);

static inline __attribute__((always_inline)) unsigned int task_cpu(const struct task_struct *p)
{
 return 0;
}

static inline __attribute__((always_inline)) void set_task_cpu(struct task_struct *p, unsigned int cpu)
{
}




extern void arch_pick_mmap_layout(struct mm_struct *mm);

extern long sched_setaffinity(pid_t pid, cpumask_t new_mask);
extern long sched_getaffinity(pid_t pid, cpumask_t *mask);







struct kobject;
struct module;

struct attribute {
 const char * name;
 struct module * owner;
 mode_t mode;
};

struct attribute_group {
 const char * name;
 struct attribute ** attrs;
};

struct vm_area_struct;

struct bin_attribute {
 struct attribute attr;
 size_t size;
 void *private;
 ssize_t (*read)(struct kobject *, char *, loff_t, size_t);
 ssize_t (*write)(struct kobject *, char *, loff_t, size_t);
 int (*mmap)(struct kobject *, struct bin_attribute *attr,
      struct vm_area_struct *vma);
};

struct sysfs_ops {
 ssize_t (*show)(struct kobject *, struct attribute *,char *);
 ssize_t (*store)(struct kobject *,struct attribute *,const char *, size_t);
};

struct sysfs_dirent {
 atomic_t s_count;
 struct list_head s_sibling;
 struct list_head s_children;
 void * s_element;
 int s_type;
 umode_t s_mode;
 struct dentry * s_dentry;
 struct iattr * s_iattr;
 atomic_t s_event;
};

extern int
sysfs_create_dir(struct kobject *);

extern void
sysfs_remove_dir(struct kobject *);

extern int
sysfs_rename_dir(struct kobject *, const char *new_name);

extern int
sysfs_create_file(struct kobject *, const struct attribute *);

extern int
sysfs_update_file(struct kobject *, const struct attribute *);

extern int
sysfs_chmod_file(struct kobject *kobj, struct attribute *attr, mode_t mode);

extern void
sysfs_remove_file(struct kobject *, const struct attribute *);

extern int
sysfs_create_link(struct kobject * kobj, struct kobject * target, const char * name);

extern void
sysfs_remove_link(struct kobject *, const char * name);

int sysfs_create_bin_file(struct kobject * kobj, struct bin_attribute * attr);
int sysfs_remove_bin_file(struct kobject * kobj, struct bin_attribute * attr);

int sysfs_create_group(struct kobject *, const struct attribute_group *);
void sysfs_remove_group(struct kobject *, const struct attribute_group *);
void sysfs_notify(struct kobject * k, char *dir, char *attr);





struct kref {
 atomic_t refcount;
};

void kref_init(struct kref *kref);
void kref_get(struct kref *kref);
int kref_put(struct kref *kref, void (*release) (struct kref *kref));


extern char uevent_helper[];


extern u64 uevent_seqnum;


typedef int kobject_action_t;
enum kobject_action {
 KOBJ_ADD = ( kobject_action_t) 0x01,
 KOBJ_REMOVE = ( kobject_action_t) 0x02,
 KOBJ_CHANGE = ( kobject_action_t) 0x03,
 KOBJ_MOUNT = ( kobject_action_t) 0x04,
 KOBJ_UMOUNT = ( kobject_action_t) 0x05,
 KOBJ_OFFLINE = ( kobject_action_t) 0x06,
 KOBJ_ONLINE = ( kobject_action_t) 0x07,
};

struct kobject {
 const char * k_name;
 char name[20];
 struct kref kref;
 struct list_head entry;
 struct kobject * parent;
 struct kset * kset;
 struct kobj_type * ktype;
 struct dentry * dentry;
 wait_queue_head_t poll;
};

extern int kobject_set_name(struct kobject *, const char *, ...)
 __attribute__((format(printf,2,3)));

static inline __attribute__((always_inline)) const char * kobject_name(const struct kobject * kobj)
{
 return kobj->k_name;
}

extern void kobject_init(struct kobject *);
extern void kobject_cleanup(struct kobject *);

extern int kobject_add(struct kobject *);
extern void kobject_del(struct kobject *);

extern int kobject_rename(struct kobject *, const char *new_name);

extern int kobject_register(struct kobject *);
extern void kobject_unregister(struct kobject *);

extern struct kobject * kobject_get(struct kobject *);
extern void kobject_put(struct kobject *);

extern struct kobject *kobject_add_dir(struct kobject *, const char *);

extern char * kobject_get_path(struct kobject *, gfp_t);

struct kobj_type {
 void (*release)(struct kobject *);
 struct sysfs_ops * sysfs_ops;
 struct attribute ** default_attrs;
};

struct kset_uevent_ops {
 int (*filter)(struct kset *kset, struct kobject *kobj);
 const char *(*name)(struct kset *kset, struct kobject *kobj);
 int (*uevent)(struct kset *kset, struct kobject *kobj, char **envp,
   int num_envp, char *buffer, int buffer_size);
};

struct kset {
 struct subsystem * subsys;
 struct kobj_type * ktype;
 struct list_head list;
 spinlock_t list_lock;
 struct kobject kobj;
 struct kset_uevent_ops * uevent_ops;
};


extern void kset_init(struct kset * k);
extern int kset_add(struct kset * k);
extern int kset_register(struct kset * k);
extern void kset_unregister(struct kset * k);

static inline __attribute__((always_inline)) struct kset * to_kset(struct kobject * kobj)
{
 return kobj ? ({ const typeof( ((struct kset *)0)->kobj ) *__mptr = (kobj); (struct kset *)( (char *)__mptr - __builtin_offsetof(struct kset,kobj) );}) : ((void *)0);
}

static inline __attribute__((always_inline)) struct kset * kset_get(struct kset * k)
{
 return k ? to_kset(kobject_get(&k->kobj)) : ((void *)0);
}

static inline __attribute__((always_inline)) void kset_put(struct kset * k)
{
 kobject_put(&k->kobj);
}

static inline __attribute__((always_inline)) struct kobj_type * get_ktype(struct kobject * k)
{
 if (k->kset && k->kset->ktype)
  return k->kset->ktype;
 else
  return k->ktype;
}

extern struct kobject * kset_find_obj(struct kset *, const char *);

struct subsystem {
 struct kset kset;
 struct rw_semaphore rwsem;
};

extern struct subsystem kernel_subsys;

extern struct subsystem hypervisor_subsys;

extern void subsystem_init(struct subsystem *);
extern int subsystem_register(struct subsystem *);
extern void subsystem_unregister(struct subsystem *);

static inline __attribute__((always_inline)) struct subsystem * subsys_get(struct subsystem * s)
{
 return s ? ({ const typeof( ((struct subsystem *)0)->kset ) *__mptr = (kset_get(&s->kset)); (struct subsystem *)( (char *)__mptr - __builtin_offsetof(struct subsystem,kset) );}) : ((void *)0);
}

static inline __attribute__((always_inline)) void subsys_put(struct subsystem * s)
{
 kset_put(&s->kset);
}

struct subsys_attribute {
 struct attribute attr;
 ssize_t (*show)(struct subsystem *, char *);
 ssize_t (*store)(struct subsystem *, const char *, size_t);
};

extern int subsys_create_file(struct subsystem * , struct subsys_attribute *);


void kobject_uevent(struct kobject *kobj, enum kobject_action action);

int add_uevent_var(char **envp, int num_envp, int *cur_index,
   char *buffer, int buffer_size, int *cur_len,
   const char *format, ...)
 __attribute__((format (printf, 7, 8)));



typedef int pm_request_t;

typedef int pm_dev_t;

enum
{
 PM_SYS_UNKNOWN = 0x00000000,
 PM_SYS_KBC = 0x41d00303,
 PM_SYS_COM = 0x41d00500,
 PM_SYS_IRDA = 0x41d00510,
 PM_SYS_FDC = 0x41d00700,
 PM_SYS_VGA = 0x41d00900,
 PM_SYS_PCMCIA = 0x41d00e00,
};

struct pm_dev;

typedef int (*pm_callback)(struct pm_dev *dev, pm_request_t rqst, void *data);




struct pm_dev
{
 pm_dev_t type;
 unsigned long id;
 pm_callback callback;
 void *data;

 unsigned long flags;
 unsigned long state;
 unsigned long prev_state;

 struct list_head entry;
};







extern void (*pm_idle)(void);
extern void (*pm_power_off)(void);

typedef int suspend_state_t;







typedef int suspend_disk_method_t;







struct pm_ops {
 suspend_disk_method_t pm_disk_mode;
 int (*valid)(suspend_state_t state);
 int (*prepare)(suspend_state_t state);
 int (*enter)(suspend_state_t state);
 int (*finish)(suspend_state_t state);
};

extern void pm_set_ops(struct pm_ops *);
extern struct pm_ops *pm_ops;
extern int pm_suspend(suspend_state_t state);






struct device;

typedef struct pm_message {
 int event;
} pm_message_t;

struct dev_pm_info {
 pm_message_t power_state;
 unsigned can_wakeup:1;

 unsigned should_wakeup:1;
 pm_message_t prev_state;
 void * saved_state;
 struct device * pm_parent;
 struct list_head entry;

};

extern void device_pm_set_parent(struct device * dev, struct device * parent);

extern int device_power_down(pm_message_t state);
extern void device_power_up(void);
extern void device_resume(void);


extern suspend_disk_method_t pm_disk_mode;

extern int device_suspend(pm_message_t state);






extern int dpm_runtime_suspend(struct device *, pm_message_t);
extern void dpm_runtime_resume(struct device *);
extern void __suspend_report_result(const char *function, void *fn, int ret);



struct sys_device;

struct sysdev_class {
 struct list_head drivers;


 int (*shutdown)(struct sys_device *);
 int (*suspend)(struct sys_device *, pm_message_t state);
 int (*resume)(struct sys_device *);
 struct kset kset;
};

struct sysdev_class_attribute {
 struct attribute attr;
 ssize_t (*show)(struct sysdev_class *, char *);
 ssize_t (*store)(struct sysdev_class *, const char *, size_t);
};

extern int sysdev_class_register(struct sysdev_class *);
extern void sysdev_class_unregister(struct sysdev_class *);

extern int sysdev_class_create_file(struct sysdev_class *,
 struct sysdev_class_attribute *);
extern void sysdev_class_remove_file(struct sysdev_class *,
 struct sysdev_class_attribute *);




struct sysdev_driver {
 struct list_head entry;
 int (*add)(struct sys_device *);
 int (*remove)(struct sys_device *);
 int (*shutdown)(struct sys_device *);
 int (*suspend)(struct sys_device *, pm_message_t state);
 int (*resume)(struct sys_device *);
};


extern int sysdev_driver_register(struct sysdev_class *, struct sysdev_driver *);
extern void sysdev_driver_unregister(struct sysdev_class *, struct sysdev_driver *);







struct sys_device {
 u32 id;
 struct sysdev_class * cls;
 struct kobject kobj;
};

extern int sysdev_register(struct sys_device *);
extern void sysdev_unregister(struct sys_device *);


struct sysdev_attribute {
 struct attribute attr;
 ssize_t (*show)(struct sys_device *, char *);
 ssize_t (*store)(struct sys_device *, const char *, size_t);
};

extern int sysdev_create_file(struct sys_device *, struct sysdev_attribute *);
extern void sysdev_remove_file(struct sys_device *, struct sysdev_attribute *);

extern int sched_mc_power_savings, sched_smt_power_savings;
extern struct sysdev_attribute attr_sched_mc_power_savings, attr_sched_smt_power_savings;
extern int sched_create_sysfs_power_savings_entries(struct sysdev_class *cls);

extern void normalize_rt_tasks(void);





static inline __attribute__((always_inline)) int frozen(struct task_struct *p)
{
 return p->flags & 0x00010000;
}




static inline __attribute__((always_inline)) int freezing(struct task_struct *p)
{
 return p->flags & 0x00004000;
}





static inline __attribute__((always_inline)) void freeze(struct task_struct *p)
{
 p->flags |= 0x00004000;
}




static inline __attribute__((always_inline)) void do_not_freeze(struct task_struct *p)
{
 p->flags &= ~0x00004000;
}




static inline __attribute__((always_inline)) int thaw_process(struct task_struct *p)
{
 if (frozen(p)) {
  p->flags &= ~0x00010000;
  wake_up_process(p);
  return 1;
 }
 return 0;
}




static inline __attribute__((always_inline)) void frozen_process(struct task_struct *p)
{
 p->flags = (p->flags & ~0x00004000) | 0x00010000;
}

extern void refrigerator(void);
extern int freeze_processes(void);
extern void thaw_processes(void);

static inline __attribute__((always_inline)) int try_to_freeze(void)
{
 if (freezing(__vericon_dummy_current)) {
  refrigerator();
  return 1;
 } else
  return 0;
}













struct __old_kernel_stat {
 unsigned short st_dev;
 unsigned short st_ino;
 unsigned short st_mode;
 unsigned short st_nlink;
 unsigned short st_uid;
 unsigned short st_gid;
 unsigned short st_rdev;
 unsigned long st_size;
 unsigned long st_atime;
 unsigned long st_mtime;
 unsigned long st_ctime;
};

struct stat {
 unsigned long st_dev;
 unsigned long st_ino;
 unsigned short st_mode;
 unsigned short st_nlink;
 unsigned short st_uid;
 unsigned short st_gid;
 unsigned long st_rdev;
 unsigned long st_size;
 unsigned long st_blksize;
 unsigned long st_blocks;
 unsigned long st_atime;
 unsigned long st_atime_nsec;
 unsigned long st_mtime;
 unsigned long st_mtime_nsec;
 unsigned long st_ctime;
 unsigned long st_ctime_nsec;
 unsigned long __unused4;
 unsigned long __unused5;
};




struct stat64 {
 unsigned long long st_dev;
 unsigned char __pad0[4];


 unsigned long __st_ino;

 unsigned int st_mode;
 unsigned int st_nlink;

 unsigned long st_uid;
 unsigned long st_gid;

 unsigned long long st_rdev;
 unsigned char __pad3[4];

 long long st_size;
 unsigned long st_blksize;

 unsigned long long st_blocks;

 unsigned long st_atime;
 unsigned long st_atime_nsec;

 unsigned long st_mtime;
 unsigned int st_mtime_nsec;

 unsigned long st_ctime;
 unsigned long st_ctime_nsec;

 unsigned long long st_ino;
};


struct kstat {
 unsigned long ino;
 dev_t dev;
 umode_t mode;
 unsigned int nlink;
 uid_t uid;
 gid_t gid;
 dev_t rdev;
 loff_t size;
 struct timespec atime;
 struct timespec mtime;
 struct timespec ctime;
 unsigned long blksize;
 unsigned long long blocks;
};





extern int request_module(const char * name, ...) __attribute__ ((format (printf, 1, 2)));






struct key;
extern int call_usermodehelper_keys(char *path, char *argv[], char *envp[],
        struct key *session_keyring, int wait);

static inline __attribute__((always_inline)) int
call_usermodehelper(char *path, char **argv, char **envp, int wait)
{
 return call_usermodehelper_keys(path, argv, envp, ((void *)0), wait);
}

extern void usermodehelper_init(void);













struct user_i387_struct {
 long cwd;
 long swd;
 long twd;
 long fip;
 long fcs;
 long foo;
 long fos;
 long st_space[20];
};

struct user_fxsr_struct {
 unsigned short cwd;
 unsigned short swd;
 unsigned short twd;
 unsigned short fop;
 long fip;
 long fcs;
 long foo;
 long fos;
 long mxcsr;
 long reserved;
 long st_space[32];
 long xmm_space[32];
 long padding[56];
};







struct user_regs_struct {
 long ebx, ecx, edx, esi, edi, ebp, eax;
 unsigned short ds, __ds, es, __es;
 unsigned short fs, __fs, gs, __gs;
 long orig_eax, eip;
 unsigned short cs, __cs;
 long eflags, esp;
 unsigned short ss, __ss;
};




struct user{


  struct user_regs_struct regs;

  int u_fpvalid;

  struct user_i387_struct i387;

  unsigned long int u_tsize;
  unsigned long int u_dsize;
  unsigned long int u_ssize;
  unsigned long start_code;
  unsigned long start_stack;



  long int signal;
  int reserved;
  struct user_pt_regs * u_ar0;

  struct user_i387_struct* u_fpstate;
  unsigned long magic;
  char u_comm[32];
  int u_debugreg[8];
};









struct oldold_utsname {
 char sysname[9];
 char nodename[9];
 char release[9];
 char version[9];
 char machine[9];
};



struct old_utsname {
 char sysname[65];
 char nodename[65];
 char release[65];
 char version[65];
 char machine[65];
};

struct new_utsname {
 char sysname[65];
 char nodename[65];
 char release[65];
 char version[65];
 char machine[65];
 char domainname[65];
};

extern struct new_utsname system_utsname;

extern struct rw_semaphore uts_sem;


typedef unsigned long elf_greg_t;


typedef elf_greg_t elf_gregset_t[(sizeof (struct user_regs_struct) / sizeof(elf_greg_t))];

typedef struct user_i387_struct elf_fpregset_t;
typedef struct user_fxsr_struct elf_fpxregset_t;







struct user_desc {
 unsigned int entry_number;
 unsigned long base_addr;
 unsigned int limit;
 unsigned int seg_32bit:1;
 unsigned int contents:2;
 unsigned int read_exec_only:1;
 unsigned int limit_in_pages:1;
 unsigned int seg_not_present:1;
 unsigned int useable:1;
};


extern struct desc_struct cpu_gdt_table[32];

extern __typeof__(unsigned char) per_cpu__cpu_16bit_stack[1024];

struct Xgt_desc_struct {
 unsigned short size;
 unsigned long address __attribute__((packed));
 unsigned short pad;
} __attribute__ ((packed));

extern struct Xgt_desc_struct idt_descr;
extern __typeof__(struct Xgt_desc_struct) per_cpu__cpu_gdt_descr;


static inline __attribute__((always_inline)) struct desc_struct *get_cpu_gdt_table(unsigned int cpu)
{
 return (struct desc_struct *)(*((void)(cpu), &per_cpu__cpu_gdt_descr)).address;
}

extern struct desc_struct default_ldt[];
extern void set_intr_gate(unsigned int irq, void * addr);

static inline __attribute__((always_inline)) void __set_tss_desc(unsigned int cpu, unsigned int entry, void *addr)
{
 __asm__ __volatile__ ("movw %w3,0(%2)\n\t" "movw %w1,2(%2)\n\t" "rorl $16,%1\n\t" "movb %b1,4(%2)\n\t" "movb %4,5(%2)\n\t" "movb $0,6(%2)\n\t" "movb %h1,7(%2)\n\t" "rorl $16,%1" : "=m"(*(&get_cpu_gdt_table(cpu)[entry])) : "q" ((int)addr), "r"(&get_cpu_gdt_table(cpu)[entry]), "ir"(__builtin_offsetof(struct tss_struct,__cacheline_filler) - 1), "i"(0x89));

}



static inline __attribute__((always_inline)) void set_ldt_desc(unsigned int cpu, void *addr, unsigned int size)
{
 __asm__ __volatile__ ("movw %w3,0(%2)\n\t" "movw %w1,2(%2)\n\t" "rorl $16,%1\n\t" "movb %b1,4(%2)\n\t" "movb %4,5(%2)\n\t" "movb $0,6(%2)\n\t" "movb %h1,7(%2)\n\t" "rorl $16,%1" : "=m"(*(&get_cpu_gdt_table(cpu)[(12 + 5)])) : "q" ((int)addr), "r"(&get_cpu_gdt_table(cpu)[(12 + 5)]), "ir"(((size << 3)-1)), "i"(0x82));
}

static inline __attribute__((always_inline)) void write_ldt_entry(void *ldt, int entry, __u32 entry_a, __u32 entry_b)
{
 __u32 *lp = (__u32 *)((char *)ldt + entry*8);
 *lp = entry_a;
 *(lp+1) = entry_b;
}





static inline __attribute__((always_inline)) void load_TLS(struct thread_struct *t, unsigned int cpu)
{

 get_cpu_gdt_table(cpu)[6 + 0] = t->tls_array[0]; get_cpu_gdt_table(cpu)[6 + 1] = t->tls_array[1]; get_cpu_gdt_table(cpu)[6 + 2] = t->tls_array[2];

}

static inline __attribute__((always_inline)) void clear_LDT(void)
{
 int cpu = ({ do { } while (0); 0; });

 set_ldt_desc(cpu, &default_ldt[0], 5);
 __asm__ __volatile__("lldt %w0"::"q" ((12 + 5)*8));
 do { } while (0);
}




static inline __attribute__((always_inline)) void load_LDT_nolock(mm_context_t *pc, int cpu)
{
 void *segments = pc->ldt;
 int count = pc->size;

 if (__builtin_expect(!!(!count), 1)) {
  segments = &default_ldt[0];
  count = 5;
 }

 set_ldt_desc(cpu, segments, count);
 __asm__ __volatile__("lldt %w0"::"q" ((12 + 5)*8));
}

static inline __attribute__((always_inline)) void load_LDT(mm_context_t *pc)
{
 int cpu = ({ do { } while (0); 0; });
 load_LDT_nolock(pc, cpu);
 do { } while (0);
}

static inline __attribute__((always_inline)) unsigned long get_desc_base(unsigned long *desc)
{
 unsigned long base;
 base = ((desc[0] >> 16) & 0x0000ffff) |
  ((desc[1] << 16) & 0x00ff0000) |
  (desc[1] & 0xff000000);
 return base;
}


struct task_struct;

extern int dump_task_regs (struct task_struct *, elf_gregset_t *);
extern int dump_task_fpu (struct task_struct *, elf_fpregset_t *);
extern int dump_task_extended_fpu (struct task_struct *, struct user_fxsr_struct *);

extern void __kernel_vsyscall;




struct linux_binprm;
extern int arch_setup_additional_pages(struct linux_binprm *bprm,
                                       int executable_stack);

extern unsigned int vdso_enabled;


typedef __u32 Elf32_Addr;
typedef __u16 Elf32_Half;
typedef __u32 Elf32_Off;
typedef __s32 Elf32_Sword;
typedef __u32 Elf32_Word;


typedef __u64 Elf64_Addr;
typedef __u16 Elf64_Half;
typedef __s16 Elf64_SHalf;
typedef __u64 Elf64_Off;
typedef __s32 Elf64_Sword;
typedef __u32 Elf64_Word;
typedef __u64 Elf64_Xword;
typedef __s64 Elf64_Sxword;

typedef struct dynamic{
  Elf32_Sword d_tag;
  union{
    Elf32_Sword d_val;
    Elf32_Addr d_ptr;
  } d_un;
} Elf32_Dyn;

typedef struct {
  Elf64_Sxword d_tag;
  union {
    Elf64_Xword d_val;
    Elf64_Addr d_ptr;
  } d_un;
} Elf64_Dyn;

typedef struct elf32_rel {
  Elf32_Addr r_offset;
  Elf32_Word r_info;
} Elf32_Rel;

typedef struct elf64_rel {
  Elf64_Addr r_offset;
  Elf64_Xword r_info;
} Elf64_Rel;

typedef struct elf32_rela{
  Elf32_Addr r_offset;
  Elf32_Word r_info;
  Elf32_Sword r_addend;
} Elf32_Rela;

typedef struct elf64_rela {
  Elf64_Addr r_offset;
  Elf64_Xword r_info;
  Elf64_Sxword r_addend;
} Elf64_Rela;

typedef struct elf32_sym{
  Elf32_Word st_name;
  Elf32_Addr st_value;
  Elf32_Word st_size;
  unsigned char st_info;
  unsigned char st_other;
  Elf32_Half st_shndx;
} Elf32_Sym;

typedef struct elf64_sym {
  Elf64_Word st_name;
  unsigned char st_info;
  unsigned char st_other;
  Elf64_Half st_shndx;
  Elf64_Addr st_value;
  Elf64_Xword st_size;
} Elf64_Sym;




typedef struct elf32_hdr{
  unsigned char e_ident[16];
  Elf32_Half e_type;
  Elf32_Half e_machine;
  Elf32_Word e_version;
  Elf32_Addr e_entry;
  Elf32_Off e_phoff;
  Elf32_Off e_shoff;
  Elf32_Word e_flags;
  Elf32_Half e_ehsize;
  Elf32_Half e_phentsize;
  Elf32_Half e_phnum;
  Elf32_Half e_shentsize;
  Elf32_Half e_shnum;
  Elf32_Half e_shstrndx;
} Elf32_Ehdr;

typedef struct elf64_hdr {
  unsigned char e_ident[16];
  Elf64_Half e_type;
  Elf64_Half e_machine;
  Elf64_Word e_version;
  Elf64_Addr e_entry;
  Elf64_Off e_phoff;
  Elf64_Off e_shoff;
  Elf64_Word e_flags;
  Elf64_Half e_ehsize;
  Elf64_Half e_phentsize;
  Elf64_Half e_phnum;
  Elf64_Half e_shentsize;
  Elf64_Half e_shnum;
  Elf64_Half e_shstrndx;
} Elf64_Ehdr;







typedef struct elf32_phdr{
  Elf32_Word p_type;
  Elf32_Off p_offset;
  Elf32_Addr p_vaddr;
  Elf32_Addr p_paddr;
  Elf32_Word p_filesz;
  Elf32_Word p_memsz;
  Elf32_Word p_flags;
  Elf32_Word p_align;
} Elf32_Phdr;

typedef struct elf64_phdr {
  Elf64_Word p_type;
  Elf64_Word p_flags;
  Elf64_Off p_offset;
  Elf64_Addr p_vaddr;
  Elf64_Addr p_paddr;
  Elf64_Xword p_filesz;
  Elf64_Xword p_memsz;
  Elf64_Xword p_align;
} Elf64_Phdr;

typedef struct {
  Elf32_Word sh_name;
  Elf32_Word sh_type;
  Elf32_Word sh_flags;
  Elf32_Addr sh_addr;
  Elf32_Off sh_offset;
  Elf32_Word sh_size;
  Elf32_Word sh_link;
  Elf32_Word sh_info;
  Elf32_Word sh_addralign;
  Elf32_Word sh_entsize;
} Elf32_Shdr;

typedef struct elf64_shdr {
  Elf64_Word sh_name;
  Elf64_Word sh_type;
  Elf64_Xword sh_flags;
  Elf64_Addr sh_addr;
  Elf64_Off sh_offset;
  Elf64_Xword sh_size;
  Elf64_Word sh_link;
  Elf64_Word sh_info;
  Elf64_Xword sh_addralign;
  Elf64_Xword sh_entsize;
} Elf64_Shdr;

typedef struct elf32_note {
  Elf32_Word n_namesz;
  Elf32_Word n_descsz;
  Elf32_Word n_type;
} Elf32_Nhdr;


typedef struct elf64_note {
  Elf64_Word n_namesz;
  Elf64_Word n_descsz;
  Elf64_Word n_type;
} Elf64_Nhdr;



extern Elf32_Dyn _DYNAMIC [];





struct kernel_param;


typedef int (*param_set_fn)(const char *val, struct kernel_param *kp);

typedef int (*param_get_fn)(char *buffer, struct kernel_param *kp);

struct kernel_param {
 const char *name;
 unsigned int perm;
 param_set_fn set;
 param_get_fn get;
 void *arg;
};


struct kparam_string {
 unsigned int maxlen;
 char *string;
};


struct kparam_array
{
 unsigned int max;
 unsigned int *num;
 param_set_fn set;
 param_get_fn get;
 unsigned int elemsize;
 void *elem;
};

extern int parse_args(const char *name,
        char *args,
        struct kernel_param *params,
        unsigned num,
        int (*unknown)(char *param, char *val));







extern int param_set_byte(const char *val, struct kernel_param *kp);
extern int param_get_byte(char *buffer, struct kernel_param *kp);


extern int param_set_short(const char *val, struct kernel_param *kp);
extern int param_get_short(char *buffer, struct kernel_param *kp);


extern int param_set_ushort(const char *val, struct kernel_param *kp);
extern int param_get_ushort(char *buffer, struct kernel_param *kp);


extern int param_set_int(const char *val, struct kernel_param *kp);
extern int param_get_int(char *buffer, struct kernel_param *kp);


extern int param_set_uint(const char *val, struct kernel_param *kp);
extern int param_get_uint(char *buffer, struct kernel_param *kp);


extern int param_set_long(const char *val, struct kernel_param *kp);
extern int param_get_long(char *buffer, struct kernel_param *kp);


extern int param_set_ulong(const char *val, struct kernel_param *kp);
extern int param_get_ulong(char *buffer, struct kernel_param *kp);


extern int param_set_charp(const char *val, struct kernel_param *kp);
extern int param_get_charp(char *buffer, struct kernel_param *kp);


extern int param_set_bool(const char *val, struct kernel_param *kp);
extern int param_get_bool(char *buffer, struct kernel_param *kp);


extern int param_set_invbool(const char *val, struct kernel_param *kp);
extern int param_get_invbool(char *buffer, struct kernel_param *kp);

extern int param_array_set(const char *val, struct kernel_param *kp);
extern int param_array_get(char *buffer, struct kernel_param *kp);

extern int param_set_copystring(const char *val, struct kernel_param *kp);
extern int param_get_string(char *buffer, struct kernel_param *kp);



struct module;

extern int module_param_sysfs_setup(struct module *mod,
        struct kernel_param *kparam,
        unsigned int num_params);

extern void module_param_sysfs_remove(struct module *mod);







typedef struct
{
 volatile long counter;
} local_t;






static __inline__ __attribute__((always_inline)) void local_inc(local_t *v)
{
 __asm__ __volatile__(
  "incl %0"
  :"+m" (v->counter));
}

static __inline__ __attribute__((always_inline)) void local_dec(local_t *v)
{
 __asm__ __volatile__(
  "decl %0"
  :"+m" (v->counter));
}

static __inline__ __attribute__((always_inline)) void local_add(long i, local_t *v)
{
 __asm__ __volatile__(
  "addl %1,%0"
  :"+m" (v->counter)
  :"ir" (i));
}

static __inline__ __attribute__((always_inline)) void local_sub(long i, local_t *v)
{
 __asm__ __volatile__(
  "subl %1,%0"
  :"+m" (v->counter)
  :"ir" (i));
}







struct mod_arch_specific
{
};


struct kernel_symbol
{
 unsigned long value;
 const char *name;
};

struct modversion_info
{
 unsigned long crc;
 char name[(64 - sizeof(unsigned long))];
};

struct module;

struct module_attribute {
        struct attribute attr;
        ssize_t (*show)(struct module_attribute *, struct module *, char *);
        ssize_t (*store)(struct module_attribute *, struct module *,
    const char *, size_t count);
 void (*setup)(struct module *, const char *);
 int (*test)(struct module *);
 void (*free)(struct module *);
};

struct module_kobject
{
 struct kobject kobj;
 struct module *mod;
};


extern int init_module(void);
extern void cleanup_module(void);


struct exception_table_entry;

const struct exception_table_entry *
search_extable(const struct exception_table_entry *first,
        const struct exception_table_entry *last,
        unsigned long value);
void sort_extable(struct exception_table_entry *start,
    struct exception_table_entry *finish);
void sort_main_extable(void);

extern struct subsystem module_subsys;

const struct exception_table_entry *search_exception_tables(unsigned long add);

struct notifier_block;




void *__symbol_get(const char *symbol);
void *__symbol_get_gpl(const char *symbol);

struct module_ref
{
 local_t count;
} __attribute__((__aligned__((1 << (7)))));

enum module_state
{
 MODULE_STATE_LIVE,
 MODULE_STATE_COMING,
 MODULE_STATE_GOING,
};



struct module_sect_attr
{
 struct module_attribute mattr;
 char name[32];
 unsigned long address;
};

struct module_sect_attrs
{
 struct attribute_group grp;
 struct module_sect_attr attrs[0];
};

struct module_param_attrs;

struct module
{
 enum module_state state;


 struct list_head list;


 char name[(64 - sizeof(unsigned long))];


 struct module_kobject mkobj;
 struct module_param_attrs *param_attrs;
 struct module_attribute *modinfo_attrs;
 const char *version;
 const char *srcversion;


 const struct kernel_symbol *syms;
 unsigned int num_syms;
 const unsigned long *crcs;


 const struct kernel_symbol *gpl_syms;
 unsigned int num_gpl_syms;
 const unsigned long *gpl_crcs;


 const struct kernel_symbol *unused_syms;
 unsigned int num_unused_syms;
 const unsigned long *unused_crcs;

 const struct kernel_symbol *unused_gpl_syms;
 unsigned int num_unused_gpl_syms;
 const unsigned long *unused_gpl_crcs;


 const struct kernel_symbol *gpl_future_syms;
 unsigned int num_gpl_future_syms;
 const unsigned long *gpl_future_crcs;


 unsigned int num_exentries;
 const struct exception_table_entry *extable;


 int (*init)(void);


 void *module_init;


 void *module_core;


 unsigned long init_size, core_size;


 unsigned long init_text_size, core_text_size;


 void *unwind_info;


 struct mod_arch_specific arch;


 int unsafe;


 int license_gplok;



 struct module_ref ref[1];


 struct list_head modules_which_use_me;


 struct task_struct *waiter;


 void (*exit)(void);




 Elf32_Sym *symtab;
 unsigned long num_symtab;
 char *strtab;


 struct module_sect_attrs *sect_attrs;



 void *percpu;



 char *args;
};




static inline __attribute__((always_inline)) int module_is_live(struct module *mod)
{
 return mod->state != MODULE_STATE_GOING;
}


struct module *module_text_address(unsigned long addr);
struct module *__module_text_address(unsigned long addr);
int is_module_address(unsigned long addr);



struct module *module_get_kallsym(unsigned int symnum, unsigned long *value,
    char *type, char *name, size_t namelen);


unsigned long module_kallsyms_lookup_name(const char *name);

int is_exported(const char *name, const struct module *mod);

extern void __module_put_and_exit(struct module *mod, long code)
 __attribute__((noreturn));



unsigned int module_refcount(struct module *mod);
void __symbol_put(const char *symbol);

void symbol_put_addr(void *addr);



static inline __attribute__((always_inline)) void __module_get(struct module *module)
{
 if (module) {
  do { if (__builtin_expect(!!((module_refcount(module) == 0)!=0), 0)) __asm__ __volatile__( "ud2\n" "\t.word %c0\n" "\t.long %c1\n" : : "i" (388), "i" ("include/linux/module.h")); } while(0);
  local_inc(&module->ref[({ do { } while (0); 0; })].count);
  do { } while (0);
 }
}

static inline __attribute__((always_inline)) int try_module_get(struct module *module)
{
 int ret = 1;

 if (module) {
  unsigned int cpu = ({ do { } while (0); 0; });
  if (__builtin_expect(!!(module_is_live(module)), 1))
   local_inc(&module->ref[cpu].count);
  else
   ret = 0;
  do { } while (0);
 }
 return ret;
}

static inline __attribute__((always_inline)) void module_put(struct module *module)
{
 if (module) {
  unsigned int cpu = ({ do { } while (0); 0; });
  local_dec(&module->ref[cpu].count);

  if (__builtin_expect(!!(!module_is_live(module)), 0))
   wake_up_process(module->waiter);
  do { } while (0);
 }
}

const char *module_address_lookup(unsigned long addr,
      unsigned long *symbolsize,
      unsigned long *offset,
      char **modname);


const struct exception_table_entry *search_module_extables(unsigned long addr);

int register_module_notifier(struct notifier_block * nb);
int unregister_module_notifier(struct notifier_block * nb);

extern void print_modules(void);

struct device_driver;
void module_add_driver(struct module *, struct device_driver *);
void module_remove_driver(struct device_driver *);





extern void *ERR_PTR(long error);
/* [kohei]
static inline __attribute__((always_inline)) void *ERR_PTR(long error)
{
  return (void *) error;
}
*/

static inline __attribute__((always_inline)) long PTR_ERR(const void *ptr)
{
 return (long) ptr;
}

static inline __attribute__((always_inline)) long IS_ERR(const void *ptr)
{
 return __builtin_expect(!!(((unsigned long)ptr) >= (unsigned long)-4095), 0);
}



struct task_struct *kthread_create(int (*threadfn)(void *data),
       void *data,
       const char namefmt[], ...);

void kthread_bind(struct task_struct *k, unsigned int cpu);
int kthread_stop(struct task_struct *k);
int kthread_should_stop(void);












enum {


 DOS_EXTENDED_PARTITION = 5,
 LINUX_EXTENDED_PARTITION = 0x85,
 WIN98_EXTENDED_PARTITION = 0x0f,

 LINUX_SWAP_PARTITION = 0x82,
 LINUX_RAID_PARTITION = 0xfd,

 SOLARIS_X86_PARTITION = LINUX_SWAP_PARTITION,
 NEW_SOLARIS_X86_PARTITION = 0xbf,

 DM6_AUX1PARTITION = 0x51,
 DM6_AUX3PARTITION = 0x53,
 DM6_PARTITION = 0x54,
 EZD_PARTITION = 0x55,

 FREEBSD_PARTITION = 0xa5,
 OPENBSD_PARTITION = 0xa6,
 NETBSD_PARTITION = 0xa9,
 BSDI_PARTITION = 0xb7,
 MINIX_PARTITION = 0x81,
 UNIXWARE_PARTITION = 0x63,
};





struct resource {
 resource_size_t start;
 resource_size_t end;
 const char *name;
 unsigned long flags;
 struct resource *parent, *sibling, *child;
};

struct resource_list {
 struct resource_list *next;
 struct resource *res;
 struct pci_dev *dev;
};

extern struct resource ioport_resource;
extern struct resource iomem_resource;

extern int request_resource(struct resource *root, struct resource *new);
extern struct resource * ____request_resource(struct resource *root, struct resource *new);
extern int release_resource(struct resource *new);
extern int insert_resource(struct resource *parent, struct resource *new);
extern int allocate_resource(struct resource *root, struct resource *new,
        resource_size_t size, resource_size_t min,
        resource_size_t max, resource_size_t align,
        void (*alignf)(void *, struct resource *,
         resource_size_t, resource_size_t),
        void *alignf_data);
int adjust_resource(struct resource *res, resource_size_t start,
      resource_size_t size);


extern int find_next_system_ram(struct resource *res);






extern struct resource * __request_region(struct resource *,
     resource_size_t start,
     resource_size_t n, const char *name);






extern int __check_region(struct resource *, resource_size_t, resource_size_t);
extern void __release_region(struct resource *, resource_size_t,
    resource_size_t);

static inline __attribute__((always_inline)) int __attribute__((deprecated)) check_region(resource_size_t s,
      resource_size_t n)
{
 return __check_region(&ioport_resource, s, n);
}




struct klist_node;
struct klist {
 spinlock_t k_lock;
 struct list_head k_list;
 void (*get)(struct klist_node *);
 void (*put)(struct klist_node *);
};


extern void klist_init(struct klist * k, void (*get)(struct klist_node *),
         void (*put)(struct klist_node *));

struct klist_node {
 struct klist * n_klist;
 struct list_head n_node;
 struct kref n_ref;
 struct completion n_removed;
};

extern void klist_add_tail(struct klist_node * n, struct klist * k);
extern void klist_add_head(struct klist_node * n, struct klist * k);

extern void klist_del(struct klist_node * n);
extern void klist_remove(struct klist_node * n);

extern int klist_node_attached(struct klist_node * n);


struct klist_iter {
 struct klist * i_klist;
 struct list_head * i_head;
 struct klist_node * i_cur;
};


extern void klist_iter_init(struct klist * k, struct klist_iter * i);
extern void klist_iter_init_node(struct klist * k, struct klist_iter * i,
     struct klist_node * n);
extern void klist_iter_exit(struct klist_iter * i);
extern struct klist_node * klist_next(struct klist_iter * i);


struct device;
struct device_driver;
struct class;
struct class_device;

struct bus_type {
 const char * name;

 struct subsystem subsys;
 struct kset drivers;
 struct kset devices;
 struct klist klist_devices;
 struct klist klist_drivers;

 struct bus_attribute * bus_attrs;
 struct device_attribute * dev_attrs;
 struct driver_attribute * drv_attrs;

 int (*match)(struct device * dev, struct device_driver * drv);
 int (*uevent)(struct device *dev, char **envp,
      int num_envp, char *buffer, int buffer_size);
 int (*probe)(struct device * dev);
 int (*remove)(struct device * dev);
 void (*shutdown)(struct device * dev);
 int (*suspend)(struct device * dev, pm_message_t state);
 int (*resume)(struct device * dev);
};

extern int bus_register(struct bus_type * bus);
extern void bus_unregister(struct bus_type * bus);

extern void bus_rescan_devices(struct bus_type * bus);



int bus_for_each_dev(struct bus_type * bus, struct device * start, void * data,
       int (*fn)(struct device *, void *));
struct device * bus_find_device(struct bus_type *bus, struct device *start,
    void *data, int (*match)(struct device *, void *));

int bus_for_each_drv(struct bus_type * bus, struct device_driver * start,
       void * data, int (*fn)(struct device_driver *, void *));




struct bus_attribute {
 struct attribute attr;
 ssize_t (*show)(struct bus_type *, char * buf);
 ssize_t (*store)(struct bus_type *, const char * buf, size_t count);
};




extern int bus_create_file(struct bus_type *, struct bus_attribute *);
extern void bus_remove_file(struct bus_type *, struct bus_attribute *);

struct device_driver {
 const char * name;
 struct bus_type * bus;

 struct completion unloaded;
 struct kobject kobj;
 struct klist klist_devices;
 struct klist_node knode_bus;

 struct module * owner;

 int (*probe) (struct device * dev);
 int (*remove) (struct device * dev);
 void (*shutdown) (struct device * dev);
 int (*suspend) (struct device * dev, pm_message_t state);
 int (*resume) (struct device * dev);
};


extern int driver_register(struct device_driver * drv);
extern void driver_unregister(struct device_driver * drv);

extern struct device_driver * get_driver(struct device_driver * drv);
extern void put_driver(struct device_driver * drv);
extern struct device_driver *driver_find(const char *name, struct bus_type *bus);




struct driver_attribute {
 struct attribute attr;
 ssize_t (*show)(struct device_driver *, char * buf);
 ssize_t (*store)(struct device_driver *, const char * buf, size_t count);
};




extern int driver_create_file(struct device_driver *, struct driver_attribute *);
extern void driver_remove_file(struct device_driver *, struct driver_attribute *);

extern int driver_for_each_device(struct device_driver * drv, struct device * start,
      void * data, int (*fn)(struct device *, void *));
struct device * driver_find_device(struct device_driver *drv,
       struct device *start, void *data,
       int (*match)(struct device *, void *));





struct class {
 const char * name;
 struct module * owner;

 struct subsystem subsys;
 struct list_head children;
 struct list_head devices;
 struct list_head interfaces;
 struct semaphore sem;

 struct class_attribute * class_attrs;
 struct class_device_attribute * class_dev_attrs;

 int (*uevent)(struct class_device *dev, char **envp,
      int num_envp, char *buffer, int buffer_size);

 void (*release)(struct class_device *dev);
 void (*class_release)(struct class *class);
};

extern int class_register(struct class *);
extern void class_unregister(struct class *);


struct class_attribute {
 struct attribute attr;
 ssize_t (*show)(struct class *, char * buf);
 ssize_t (*store)(struct class *, const char * buf, size_t count);
};




extern int class_create_file(struct class *, const struct class_attribute *);
extern void class_remove_file(struct class *, const struct class_attribute *);

struct class_device_attribute {
 struct attribute attr;
 ssize_t (*show)(struct class_device *, char * buf);
 ssize_t (*store)(struct class_device *, const char * buf, size_t count);
};





extern int class_device_create_file(struct class_device *,
        const struct class_device_attribute *);

struct class_device {
 struct list_head node;

 struct kobject kobj;
 struct class * class;
 dev_t devt;
 struct class_device_attribute *devt_attr;
 struct class_device_attribute uevent_attr;
 struct device * dev;
 void * class_data;
 struct class_device *parent;
 struct attribute_group ** groups;

 void (*release)(struct class_device *dev);
 int (*uevent)(struct class_device *dev, char **envp,
      int num_envp, char *buffer, int buffer_size);
 char class_id[20];
};

static inline __attribute__((always_inline)) void *
class_get_devdata (struct class_device *dev)
{
 return dev->class_data;
}

static inline __attribute__((always_inline)) void
class_set_devdata (struct class_device *dev, void *data)
{
 dev->class_data = data;
}


extern int class_device_register(struct class_device *);
extern void class_device_unregister(struct class_device *);
extern void class_device_initialize(struct class_device *);
extern int class_device_add(struct class_device *);
extern void class_device_del(struct class_device *);

extern int class_device_rename(struct class_device *, char *);

extern struct class_device * class_device_get(struct class_device *);
extern void class_device_put(struct class_device *);

extern void class_device_remove_file(struct class_device *,
         const struct class_device_attribute *);
extern int class_device_create_bin_file(struct class_device *,
     struct bin_attribute *);
extern void class_device_remove_bin_file(struct class_device *,
      struct bin_attribute *);

struct class_interface {
 struct list_head node;
 struct class *class;

 int (*add) (struct class_device *, struct class_interface *);
 void (*remove) (struct class_device *, struct class_interface *);
};

extern int class_interface_register(struct class_interface *);
extern void class_interface_unregister(struct class_interface *);

extern struct class *class_create(struct module *owner, char *name);
extern void class_destroy(struct class *cls);
extern struct class_device *class_device_create(struct class *cls,
      struct class_device *parent,
      dev_t devt,
      struct device *device,
      char *fmt, ...)
     __attribute__((format(printf,5,6)));
extern void class_device_destroy(struct class *cls, dev_t devt);



struct device_attribute {
 struct attribute attr;
 ssize_t (*show)(struct device *dev, struct device_attribute *attr,
   char *buf);
 ssize_t (*store)(struct device *dev, struct device_attribute *attr,
    const char *buf, size_t count);
};




extern int device_create_file(struct device *device, struct device_attribute * entry);
extern void device_remove_file(struct device * dev, struct device_attribute * attr);
struct device {
 struct klist klist_children;
 struct klist_node knode_parent;
 struct klist_node knode_driver;
 struct klist_node knode_bus;
 struct device * parent;

 struct kobject kobj;
 char bus_id[20];
 struct device_attribute uevent_attr;
 struct device_attribute *devt_attr;

 struct semaphore sem;



 struct bus_type * bus;
 struct device_driver *driver;

 void *driver_data;
 void *platform_data;

 void *firmware_data;

 struct dev_pm_info power;

 u64 *dma_mask;
 u64 coherent_dma_mask;





 struct list_head dma_pools;

 struct dma_coherent_mem *dma_mem;



 struct list_head node;
 struct class *class;
 dev_t devt;

 void (*release)(struct device * dev);
};

static inline __attribute__((always_inline)) void *
dev_get_drvdata (struct device *dev)
{
 return dev->driver_data;
}

static inline __attribute__((always_inline)) void
dev_set_drvdata (struct device *dev, void *data)
{
 dev->driver_data = data;
}

static inline __attribute__((always_inline)) int device_is_registered(struct device *dev)
{
 return klist_node_attached(&dev->knode_bus);
}




extern int device_register(struct device * dev);
extern void device_unregister(struct device * dev);
extern void device_initialize(struct device * dev);
extern int device_add(struct device * dev);
extern void device_del(struct device * dev);
extern int device_for_each_child(struct device *, void *,
       int (*fn)(struct device *, void *));





extern void device_bind_driver(struct device * dev);
extern void device_release_driver(struct device * dev);
extern int device_attach(struct device * dev);
extern void driver_attach(struct device_driver * drv);
extern void device_reprobe(struct device *dev);




extern struct device *device_create(struct class *cls, struct device *parent,
        dev_t devt, char *fmt, ...)
        __attribute__((format(printf,4,5)));
extern void device_destroy(struct class *cls, dev_t devt);







extern int (*platform_notify)(struct device * dev);

extern int (*platform_notify_remove)(struct device * dev);






extern struct device * get_device(struct device * dev);
extern void put_device(struct device * dev);



extern void device_shutdown(void);



extern int firmware_register(struct subsystem *);
extern void firmware_unregister(struct subsystem *);


extern const char *dev_driver_string(struct device *dev);














extern unsigned int __invalid_size_argument_for_IOC;




struct files_stat_struct {
 int nr_files;
 int nr_free_files;
 int max_files;
};
extern struct files_stat_struct files_stat;
extern int get_max_files(void);

struct inodes_stat_t {
 int nr_inodes;
 int nr_unused;
 int dummy[5];
};
extern struct inodes_stat_t inodes_stat;

extern int leases_enable, lease_break_time;


extern int dir_notify_enable;



static inline __attribute__((always_inline)) int old_valid_dev(dev_t dev)
{
 return ((unsigned int) ((dev) >> 20)) < 256 && ((unsigned int) ((dev) & ((1U << 20) - 1))) < 256;
}

static inline __attribute__((always_inline)) u16 old_encode_dev(dev_t dev)
{
 return (((unsigned int) ((dev) >> 20)) << 8) | ((unsigned int) ((dev) & ((1U << 20) - 1)));
}

static inline __attribute__((always_inline)) dev_t old_decode_dev(u16 val)
{
 return ((((val >> 8) & 255) << 20) | (val & 255));
}

static inline __attribute__((always_inline)) int new_valid_dev(dev_t dev)
{
 return 1;
}

static inline __attribute__((always_inline)) u32 new_encode_dev(dev_t dev)
{
 unsigned major = ((unsigned int) ((dev) >> 20));
 unsigned minor = ((unsigned int) ((dev) & ((1U << 20) - 1)));
 return (minor & 0xff) | (major << 8) | ((minor & ~0xff) << 12);
}

static inline __attribute__((always_inline)) dev_t new_decode_dev(u32 dev)
{
 unsigned major = (dev & 0xfff00) >> 8;
 unsigned minor = (dev & 0xff) | ((dev >> 12) & 0xfff00);
 return (((major) << 20) | (minor));
}

static inline __attribute__((always_inline)) int huge_valid_dev(dev_t dev)
{
 return 1;
}

static inline __attribute__((always_inline)) u64 huge_encode_dev(dev_t dev)
{
 return new_encode_dev(dev);
}

static inline __attribute__((always_inline)) dev_t huge_decode_dev(u64 dev)
{
 return new_decode_dev(dev);
}

static inline __attribute__((always_inline)) int sysv_valid_dev(dev_t dev)
{
 return ((unsigned int) ((dev) >> 20)) < (1<<14) && ((unsigned int) ((dev) & ((1U << 20) - 1))) < (1<<18);
}

static inline __attribute__((always_inline)) u32 sysv_encode_dev(dev_t dev)
{
 return ((unsigned int) ((dev) & ((1U << 20) - 1))) | (((unsigned int) ((dev) >> 20)) << 18);
}

static inline __attribute__((always_inline)) unsigned sysv_major(u32 dev)
{
 return (dev >> 18) & 0x3fff;
}

static inline __attribute__((always_inline)) unsigned sysv_minor(u32 dev)
{
 return dev & 0x3ffff;
}



struct nameidata;
struct vfsmount;

struct qstr {
 unsigned int hash;
 unsigned int len;
 const unsigned char *name;
};

struct dentry_stat_t {
 int nr_dentry;
 int nr_unused;
 int age_limit;
 int want_pages;
 int dummy[2];
};
extern struct dentry_stat_t dentry_stat;






static inline __attribute__((always_inline)) unsigned long
partial_name_hash(unsigned long c, unsigned long prevhash)
{
 return (prevhash + (c << 4) + (c >> 4)) * 11;
}





static inline __attribute__((always_inline)) unsigned long end_name_hash(unsigned long hash)
{
 return (unsigned int) hash;
}


static inline __attribute__((always_inline)) unsigned int
full_name_hash(const unsigned char *name, unsigned int len)
{
 unsigned long hash = 0;
 while (len--)
  hash = partial_name_hash(*name++, hash);
 return end_name_hash(hash);
}

struct dcookie_struct;



struct dentry {
 atomic_t d_count;
 unsigned int d_flags;
 spinlock_t d_lock;
 struct inode *d_inode;





 struct hlist_node d_hash;
 struct dentry *d_parent;
 struct qstr d_name;

 struct list_head d_lru;



 union {
  struct list_head d_child;
   struct rcu_head d_rcu;
 } d_u;
 struct list_head d_subdirs;
 struct list_head d_alias;
 unsigned long d_time;
 struct dentry_operations *d_op;
 struct super_block *d_sb;
 void *d_fsdata;

 struct dcookie_struct *d_cookie;

 int d_mounted;
 unsigned char d_iname[36];
};







enum dentry_d_lock_class
{
 DENTRY_D_LOCK_NORMAL,
 DENTRY_D_LOCK_NESTED
};

struct dentry_operations {
 int (*d_revalidate)(struct dentry *, struct nameidata *);
 int (*d_hash) (struct dentry *, struct qstr *);
 int (*d_compare) (struct dentry *, struct qstr *, struct qstr *);
 int (*d_delete)(struct dentry *);
 void (*d_release)(struct dentry *);
 void (*d_iput)(struct dentry *, struct inode *);
};

extern spinlock_t dcache_lock;

static inline __attribute__((always_inline)) void __d_drop(struct dentry *dentry)
{
 if (!(dentry->d_flags & 0x0010)) {
  dentry->d_flags |= 0x0010;
  hlist_del_rcu(&dentry->d_hash);
 }
}

static inline __attribute__((always_inline)) void d_drop(struct dentry *dentry)
{
 _spin_lock(&dcache_lock);
 _spin_lock(&dentry->d_lock);
  __d_drop(dentry);
 _spin_unlock(&dentry->d_lock);
 _spin_unlock(&dcache_lock);
}

static inline __attribute__((always_inline)) int dname_external(struct dentry *dentry)
{
 return dentry->d_name.name != dentry->d_iname;
}




extern void d_instantiate(struct dentry *, struct inode *);
extern struct dentry * d_instantiate_unique(struct dentry *, struct inode *);
extern void d_delete(struct dentry *);


extern struct dentry * d_alloc(struct dentry *, const struct qstr *);
extern struct dentry * d_alloc_anon(struct inode *);
extern struct dentry * d_splice_alias(struct inode *, struct dentry *);
extern void shrink_dcache_sb(struct super_block *);
extern void shrink_dcache_parent(struct dentry *);
extern int d_invalidate(struct dentry *);


extern struct dentry * d_alloc_root(struct inode *);


extern void d_genocide(struct dentry *);

extern struct dentry *d_find_alias(struct inode *);
extern void d_prune_aliases(struct inode *);


extern int have_submounts(struct dentry *);




extern void d_rehash(struct dentry *);

static inline __attribute__((always_inline)) void d_add(struct dentry *entry, struct inode *inode)
{
 d_instantiate(entry, inode);
 d_rehash(entry);
}

static inline __attribute__((always_inline)) struct dentry *d_add_unique(struct dentry *entry, struct inode *inode)
{
 struct dentry *res;

 res = d_instantiate_unique(entry, inode);
 d_rehash(res != ((void *)0) ? res : entry);
 return res;
}


extern void d_move(struct dentry *, struct dentry *);


extern struct dentry * d_lookup(struct dentry *, struct qstr *);
extern struct dentry * __d_lookup(struct dentry *, struct qstr *);
extern struct dentry * d_hash_and_lookup(struct dentry *, struct qstr *);


extern int d_validate(struct dentry *, struct dentry *);

extern char * d_path(struct dentry *, struct vfsmount *, char *, int);

static inline __attribute__((always_inline)) struct dentry *dget(struct dentry *dentry)
{
 if (dentry) {
  do { if (__builtin_expect(!!((!((&dentry->d_count)->counter))!=0), 0)) __asm__ __volatile__( "ud2\n" "\t.word %c0\n" "\t.long %c1\n" : : "i" (314), "i" ("include/linux/dcache.h")); } while(0);
  atomic_inc(&dentry->d_count);
 }
 return dentry;
}

extern struct dentry * dget_locked(struct dentry *);

static inline __attribute__((always_inline)) int d_unhashed(struct dentry *dentry)
{
 return (dentry->d_flags & 0x0010);
}

static inline __attribute__((always_inline)) struct dentry *dget_parent(struct dentry *dentry)
{
 struct dentry *ret;

 _spin_lock(&dentry->d_lock);
 ret = dget(dentry->d_parent);
 _spin_unlock(&dentry->d_lock);
 return ret;
}

extern void dput(struct dentry *);

static inline __attribute__((always_inline)) int d_mountpoint(struct dentry *dentry)
{
 return dentry->d_mounted;
}

extern struct vfsmount *lookup_mnt(struct vfsmount *, struct dentry *);
extern struct vfsmount *__lookup_mnt(struct vfsmount *, struct dentry *, int);
extern struct dentry *lookup_create(struct nameidata *nd, int is_dir);

extern int sysctl_vfs_cache_pressure;







struct radix_tree_root {
 unsigned int height;
 gfp_t gfp_mask;
 struct radix_tree_node *rnode;
};

int radix_tree_insert(struct radix_tree_root *, unsigned long, void *);
void *radix_tree_lookup(struct radix_tree_root *, unsigned long);
void **radix_tree_lookup_slot(struct radix_tree_root *, unsigned long);
void *radix_tree_delete(struct radix_tree_root *, unsigned long);
unsigned int
radix_tree_gang_lookup(struct radix_tree_root *root, void **results,
   unsigned long first_index, unsigned int max_items);
int radix_tree_preload(gfp_t gfp_mask);
void radix_tree_init(void);
void *radix_tree_tag_set(struct radix_tree_root *root,
   unsigned long index, unsigned int tag);
void *radix_tree_tag_clear(struct radix_tree_root *root,
   unsigned long index, unsigned int tag);
int radix_tree_tag_get(struct radix_tree_root *root,
   unsigned long index, unsigned int tag);
unsigned int
radix_tree_gang_lookup_tag(struct radix_tree_root *root, void **results,
  unsigned long first_index, unsigned int max_items,
  unsigned int tag);
int radix_tree_tagged(struct radix_tree_root *root, unsigned int tag);

static inline __attribute__((always_inline)) void radix_tree_preload_end(void)
{
 do { } while (0);
}



struct raw_prio_tree_node {
 struct prio_tree_node *left;
 struct prio_tree_node *right;
 struct prio_tree_node *parent;
};

struct prio_tree_node {
 struct prio_tree_node *left;
 struct prio_tree_node *right;
 struct prio_tree_node *parent;
 unsigned long start;
 unsigned long last;
};

struct prio_tree_root {
 struct prio_tree_node *prio_tree_node;
 unsigned short index_bits;
 unsigned short raw;




};

struct prio_tree_iter {
 struct prio_tree_node *cur;
 unsigned long mask;
 unsigned long value;
 int size_level;

 struct prio_tree_root *root;
 unsigned long r_index;
 unsigned long h_index;
};

static inline __attribute__((always_inline)) void prio_tree_iter_init(struct prio_tree_iter *iter,
  struct prio_tree_root *root, unsigned long r_index, unsigned long h_index)
{
 iter->root = root;
 iter->r_index = r_index;
 iter->h_index = h_index;
 iter->cur = ((void *)0);
}

static inline __attribute__((always_inline)) int prio_tree_empty(const struct prio_tree_root *root)
{
 return root->prio_tree_node == ((void *)0);
}

static inline __attribute__((always_inline)) int prio_tree_root(const struct prio_tree_node *node)
{
 return node->parent == node;
}

static inline __attribute__((always_inline)) int prio_tree_left_empty(const struct prio_tree_node *node)
{
 return node->left == node;
}

static inline __attribute__((always_inline)) int prio_tree_right_empty(const struct prio_tree_node *node)
{
 return node->right == node;
}


struct prio_tree_node *prio_tree_replace(struct prio_tree_root *root,
                struct prio_tree_node *old, struct prio_tree_node *node);
struct prio_tree_node *prio_tree_insert(struct prio_tree_root *root,
                struct prio_tree_node *node);
void prio_tree_remove(struct prio_tree_root *root, struct prio_tree_node *node);
struct prio_tree_node *prio_tree_next(struct prio_tree_iter *iter);


struct hd_geometry;
struct iovec;
struct nameidata;
struct kiocb;
struct pipe_inode_info;
struct poll_table_struct;
struct kstatfs;
struct vm_area_struct;
struct vfsmount;

extern void __attribute__ ((__section__ (".init.text"))) inode_init(unsigned long);
extern void __attribute__ ((__section__ (".init.text"))) inode_init_early(void);
extern void __attribute__ ((__section__ (".init.text"))) mnt_init(unsigned long);
extern void __attribute__ ((__section__ (".init.text"))) files_init(unsigned long);

struct buffer_head;
typedef int (get_block_t)(struct inode *inode, sector_t iblock,
   struct buffer_head *bh_result, int create);
typedef void (dio_iodone_t)(struct kiocb *iocb, loff_t offset,
   ssize_t bytes, void *private);

struct iattr {
 unsigned int ia_valid;
 umode_t ia_mode;
 uid_t ia_uid;
 gid_t ia_gid;
 loff_t ia_size;
 struct timespec ia_atime;
 struct timespec ia_mtime;
 struct timespec ia_ctime;






 struct file *ia_file;
};






typedef __kernel_uid32_t qid_t;
typedef __u64 qsize_t;

extern spinlock_t dq_data_lock;

struct if_dqblk {
 __u64 dqb_bhardlimit;
 __u64 dqb_bsoftlimit;
 __u64 dqb_curspace;
 __u64 dqb_ihardlimit;
 __u64 dqb_isoftlimit;
 __u64 dqb_curinodes;
 __u64 dqb_btime;
 __u64 dqb_itime;
 __u32 dqb_valid;
};

struct if_dqinfo {
 __u64 dqi_bgrace;
 __u64 dqi_igrace;
 __u32 dqi_flags;
 __u32 dqi_valid;
};







typedef struct fs_disk_quota {
 __s8 d_version;
 __s8 d_flags;
 __u16 d_fieldmask;
 __u32 d_id;
 __u64 d_blk_hardlimit;
 __u64 d_blk_softlimit;
 __u64 d_ino_hardlimit;
 __u64 d_ino_softlimit;
 __u64 d_bcount;
 __u64 d_icount;
 __s32 d_itimer;

 __s32 d_btimer;
 __u16 d_iwarns;
 __u16 d_bwarns;
 __s32 d_padding2;
 __u64 d_rtb_hardlimit;
 __u64 d_rtb_softlimit;
 __u64 d_rtbcount;
 __s32 d_rtbtimer;
 __u16 d_rtbwarns;
 __s16 d_padding3;
 char d_padding4[8];
} fs_disk_quota_t;

typedef struct fs_qfilestat {
 __u64 qfs_ino;
 __u64 qfs_nblks;
 __u32 qfs_nextents;
} fs_qfilestat_t;

typedef struct fs_quota_stat {
 __s8 qs_version;
 __u16 qs_flags;
 __s8 qs_pad;
 fs_qfilestat_t qs_uquota;
 fs_qfilestat_t qs_gquota;
 __u32 qs_incoredqs;
 __s32 qs_btimelimit;
 __s32 qs_itimelimit;
 __s32 qs_rtbtimelimit;
 __u16 qs_bwarnlimit;
 __u16 qs_iwarnlimit;
} fs_quota_stat_t;



struct v1_mem_dqinfo {
};



struct v2_mem_dqinfo {
 unsigned int dqi_blocks;
 unsigned int dqi_free_blk;
 unsigned int dqi_free_entry;
};


struct mem_dqblk {
 __u32 dqb_bhardlimit;
 __u32 dqb_bsoftlimit;
 qsize_t dqb_curspace;
 __u32 dqb_ihardlimit;
 __u32 dqb_isoftlimit;
 __u32 dqb_curinodes;
 time_t dqb_btime;
 time_t dqb_itime;
};




struct quota_format_type;

struct mem_dqinfo {
 struct quota_format_type *dqi_format;
 struct list_head dqi_dirty_list;
 unsigned long dqi_flags;
 unsigned int dqi_bgrace;
 unsigned int dqi_igrace;
 union {
  struct v1_mem_dqinfo v1_i;
  struct v2_mem_dqinfo v2_i;
 } u;
};

struct super_block;





extern void mark_info_dirty(struct super_block *sb, int type);







struct dqstats {
 int lookups;
 int drops;
 int reads;
 int writes;
 int cache_hits;
 int allocated_dquots;
 int free_dquots;
 int syncs;
};

extern struct dqstats dqstats;

struct dquot {
 struct hlist_node dq_hash;
 struct list_head dq_inuse;
 struct list_head dq_free;
 struct list_head dq_dirty;
 struct mutex dq_lock;
 atomic_t dq_count;
 wait_queue_head_t dq_wait_unused;
 struct super_block *dq_sb;
 unsigned int dq_id;
 loff_t dq_off;
 unsigned long dq_flags;
 short dq_type;
 struct mem_dqblk dq_dqb;
};







struct quota_format_ops {
 int (*check_quota_file)(struct super_block *sb, int type);
 int (*read_file_info)(struct super_block *sb, int type);
 int (*write_file_info)(struct super_block *sb, int type);
 int (*free_file_info)(struct super_block *sb, int type);
 int (*read_dqblk)(struct dquot *dquot);
 int (*commit_dqblk)(struct dquot *dquot);
 int (*release_dqblk)(struct dquot *dquot);
};


struct dquot_operations {
 int (*initialize) (struct inode *, int);
 int (*drop) (struct inode *);
 int (*alloc_space) (struct inode *, qsize_t, int);
 int (*alloc_inode) (const struct inode *, unsigned long);
 int (*free_space) (struct inode *, qsize_t);
 int (*free_inode) (const struct inode *, unsigned long);
 int (*transfer) (struct inode *, struct iattr *);
 int (*write_dquot) (struct dquot *);
 int (*acquire_dquot) (struct dquot *);
 int (*release_dquot) (struct dquot *);
 int (*mark_dirty) (struct dquot *);
 int (*write_info) (struct super_block *, int);
};


struct quotactl_ops {
 int (*quota_on)(struct super_block *, int, int, char *);
 int (*quota_off)(struct super_block *, int);
 int (*quota_sync)(struct super_block *, int);
 int (*get_info)(struct super_block *, int, struct if_dqinfo *);
 int (*set_info)(struct super_block *, int, struct if_dqinfo *);
 int (*get_dqblk)(struct super_block *, int, qid_t, struct if_dqblk *);
 int (*set_dqblk)(struct super_block *, int, qid_t, struct if_dqblk *);
 int (*get_xstate)(struct super_block *, struct fs_quota_stat *);
 int (*set_xstate)(struct super_block *, unsigned int, int);
 int (*get_xquota)(struct super_block *, int, qid_t, struct fs_disk_quota *);
 int (*set_xquota)(struct super_block *, int, qid_t, struct fs_disk_quota *);
};

struct quota_format_type {
 int qf_fmt_id;
 struct quota_format_ops *qf_ops;
 struct module *qf_owner;
 struct quota_format_type *qf_next;
};




struct quota_info {
 unsigned int flags;
 struct mutex dqio_mutex;
 struct mutex dqonoff_mutex;
 struct rw_semaphore dqptr_sem;
 struct inode *files[2];
 struct mem_dqinfo info[2];
 struct quota_format_ops *ops[2];
};


int mark_dquot_dirty(struct dquot *dquot);

int register_quota_format(struct quota_format_type *fmt);
void unregister_quota_format(struct quota_format_type *fmt);

struct quota_module_name {
 int qm_fmt_id;
 char *qm_mod_name;
};


enum positive_aop_returns {
 AOP_WRITEPAGE_ACTIVATE = 0x80000,
 AOP_TRUNCATED_PAGE = 0x80001,
};




struct page;
struct address_space;
struct writeback_control;

struct address_space_operations {
 int (*writepage)(struct page *page, struct writeback_control *wbc);
 int (*readpage)(struct file *, struct page *);
 void (*sync_page)(struct page *);


 int (*writepages)(struct address_space *, struct writeback_control *);


 int (*set_page_dirty)(struct page *page);

 int (*readpages)(struct file *filp, struct address_space *mapping,
   struct list_head *pages, unsigned nr_pages);





 int (*prepare_write)(struct file *, struct page *, unsigned, unsigned);
 int (*commit_write)(struct file *, struct page *, unsigned, unsigned);

 sector_t (*bmap)(struct address_space *, sector_t);
 void (*invalidatepage) (struct page *, unsigned long);
 int (*releasepage) (struct page *, gfp_t);
 ssize_t (*direct_IO)(int, struct kiocb *, const struct iovec *iov,
   loff_t offset, unsigned long nr_segs);
 struct page* (*get_xip_page)(struct address_space *, sector_t,
   int);

 int (*migratepage) (struct address_space *,
   struct page *, struct page *);
};

struct backing_dev_info;
struct address_space {
 struct inode *host;
 struct radix_tree_root page_tree;
 rwlock_t tree_lock;
 unsigned int i_mmap_writable;
 struct prio_tree_root i_mmap;
 struct list_head i_mmap_nonlinear;
 spinlock_t i_mmap_lock;
 unsigned int truncate_count;
 unsigned long nrpages;
 unsigned long writeback_index;
 const struct address_space_operations *a_ops;
 unsigned long flags;
 struct backing_dev_info *backing_dev_info;
 spinlock_t private_lock;
 struct list_head private_list;
 struct address_space *assoc_mapping;
} __attribute__((aligned(sizeof(long))));






struct block_device {
 dev_t bd_dev;
 struct inode * bd_inode;
 int bd_openers;
 struct mutex bd_mutex;
 struct mutex bd_mount_mutex;
 struct list_head bd_inodes;
 void * bd_holder;
 int bd_holders;

 struct list_head bd_holder_list;

 struct block_device * bd_contains;
 unsigned bd_block_size;
 struct hd_struct * bd_part;

 unsigned bd_part_count;
 int bd_invalidated;
 struct gendisk * bd_disk;
 struct list_head bd_list;
 struct backing_dev_info *bd_inode_backing_dev_info;






 unsigned long bd_private;
};

enum bdev_bd_mutex_lock_class
{
 BD_MUTEX_NORMAL,
 BD_MUTEX_WHOLE,
 BD_MUTEX_PARTITION
};

int mapping_tagged(struct address_space *mapping, int tag);




static inline __attribute__((always_inline)) int mapping_mapped(struct address_space *mapping)
{
 return !prio_tree_empty(&mapping->i_mmap) ||
  !list_empty(&mapping->i_mmap_nonlinear);
}







static inline __attribute__((always_inline)) int mapping_writably_mapped(struct address_space *mapping)
{
 return mapping->i_mmap_writable != 0;
}

struct inode {
 struct hlist_node i_hash;
 struct list_head i_list;
 struct list_head i_sb_list;
 struct list_head i_dentry;
 unsigned long i_ino;
 atomic_t i_count;
 umode_t i_mode;
 unsigned int i_nlink;
 uid_t i_uid;
 gid_t i_gid;
 dev_t i_rdev;
 loff_t i_size;
 struct timespec i_atime;
 struct timespec i_mtime;
 struct timespec i_ctime;
 unsigned int i_blkbits;
 unsigned long i_blksize;
 unsigned long i_version;
 blkcnt_t i_blocks;
 unsigned short i_bytes;
 spinlock_t i_lock;
 struct mutex i_mutex;
 struct rw_semaphore i_alloc_sem;
 struct inode_operations *i_op;
 const struct file_operations *i_fop;
 struct super_block *i_sb;
 struct file_lock *i_flock;
 struct address_space *i_mapping;
 struct address_space i_data;

 struct dquot *i_dquot[2];


 struct list_head i_devices;
 struct pipe_inode_info *i_pipe;
 struct block_device *i_bdev;
 struct cdev *i_cdev;
 int i_cindex;

 __u32 i_generation;


 unsigned long i_dnotify_mask;
 struct dnotify_struct *i_dnotify;



 struct list_head inotify_watches;
 struct mutex inotify_mutex;


 unsigned long i_state;
 unsigned long dirtied_when;

 unsigned int i_flags;

 atomic_t i_writecount;
 void *i_security;
 union {
  void *generic_ip;
 } u;



};

enum inode_i_mutex_lock_class
{
 I_MUTEX_NORMAL,
 I_MUTEX_PARENT,
 I_MUTEX_CHILD,
 I_MUTEX_XATTR,
 I_MUTEX_QUOTA
};

static inline __attribute__((always_inline)) loff_t i_size_read(struct inode *inode)
{

 return inode->i_size;

}


static inline __attribute__((always_inline)) void i_size_write(struct inode *inode, loff_t i_size)
{

 inode->i_size = i_size;

}

static inline __attribute__((always_inline)) unsigned iminor(struct inode *inode)
{
 return ((unsigned int) ((inode->i_rdev) & ((1U << 20) - 1)));
}

static inline __attribute__((always_inline)) unsigned imajor(struct inode *inode)
{
 return ((unsigned int) ((inode->i_rdev) >> 20));
}

extern struct block_device *I_BDEV(struct inode *inode);

struct fown_struct {
 rwlock_t lock;
 int pid;
 uid_t uid, euid;
 void *security;
 int signum;
};




struct file_ra_state {
 unsigned long start;
 unsigned long size;
 unsigned long flags;
 unsigned long cache_hit;
 unsigned long prev_page;
 unsigned long ahead_start;
 unsigned long ahead_size;
 unsigned long ra_pages;
 unsigned long mmap_hit;
 unsigned long mmap_miss;
};



struct file {




 union {
  struct list_head fu_list;
  struct rcu_head fu_rcuhead;
 } f_u;
 struct dentry *f_dentry;
 struct vfsmount *f_vfsmnt;
 const struct file_operations *f_op;
 atomic_t f_count;
 unsigned int f_flags;
 mode_t f_mode;
 loff_t f_pos;
 struct fown_struct f_owner;
 unsigned int f_uid, f_gid;
 struct file_ra_state f_ra;

 unsigned long f_version;
 void *f_security;


 void *private_data;



 struct list_head f_ep_links;
 spinlock_t f_ep_lock;

 struct address_space *f_mapping;
};
extern spinlock_t files_lock;

typedef struct files_struct *fl_owner_t;

struct file_lock_operations {
 void (*fl_insert)(struct file_lock *);
 void (*fl_remove)(struct file_lock *);
 void (*fl_copy_lock)(struct file_lock *, struct file_lock *);
 void (*fl_release_private)(struct file_lock *);
};

struct lock_manager_operations {
 int (*fl_compare_owner)(struct file_lock *, struct file_lock *);
 void (*fl_notify)(struct file_lock *);
 void (*fl_copy_lock)(struct file_lock *, struct file_lock *);
 void (*fl_release_private)(struct file_lock *);
 void (*fl_break)(struct file_lock *);
 int (*fl_mylease)(struct file_lock *, struct file_lock *);
 int (*fl_change)(struct file_lock **, int);
};










 enum nfs_stat {
 NFS_OK = 0,
 NFSERR_PERM = 1,
 NFSERR_NOENT = 2,
 NFSERR_IO = 5,
 NFSERR_NXIO = 6,
 NFSERR_EAGAIN = 11,
 NFSERR_ACCES = 13,
 NFSERR_EXIST = 17,
 NFSERR_XDEV = 18,
 NFSERR_NODEV = 19,
 NFSERR_NOTDIR = 20,
 NFSERR_ISDIR = 21,
 NFSERR_INVAL = 22,
 NFSERR_FBIG = 27,
 NFSERR_NOSPC = 28,
 NFSERR_ROFS = 30,
 NFSERR_MLINK = 31,
 NFSERR_OPNOTSUPP = 45,
 NFSERR_NAMETOOLONG = 63,
 NFSERR_NOTEMPTY = 66,
 NFSERR_DQUOT = 69,
 NFSERR_STALE = 70,
 NFSERR_REMOTE = 71,
 NFSERR_WFLUSH = 99,
 NFSERR_BADHANDLE = 10001,
 NFSERR_NOT_SYNC = 10002,
 NFSERR_BAD_COOKIE = 10003,
 NFSERR_NOTSUPP = 10004,
 NFSERR_TOOSMALL = 10005,
 NFSERR_SERVERFAULT = 10006,
 NFSERR_BADTYPE = 10007,
 NFSERR_JUKEBOX = 10008,
 NFSERR_SAME = 10009,
 NFSERR_DENIED = 10010,
 NFSERR_EXPIRED = 10011,
 NFSERR_LOCKED = 10012,
 NFSERR_GRACE = 10013,
 NFSERR_FHEXPIRED = 10014,
 NFSERR_SHARE_DENIED = 10015,
 NFSERR_WRONGSEC = 10016,
 NFSERR_CLID_INUSE = 10017,
 NFSERR_RESOURCE = 10018,
 NFSERR_MOVED = 10019,
 NFSERR_NOFILEHANDLE = 10020,
 NFSERR_MINOR_VERS_MISMATCH = 10021,
 NFSERR_STALE_CLIENTID = 10022,
 NFSERR_STALE_STATEID = 10023,
 NFSERR_OLD_STATEID = 10024,
 NFSERR_BAD_STATEID = 10025,
 NFSERR_BAD_SEQID = 10026,
 NFSERR_NOT_SAME = 10027,
 NFSERR_LOCK_RANGE = 10028,
 NFSERR_SYMLINK = 10029,
 NFSERR_RESTOREFH = 10030,
 NFSERR_LEASE_MOVED = 10031,
 NFSERR_ATTRNOTSUPP = 10032,
 NFSERR_NO_GRACE = 10033,
 NFSERR_RECLAIM_BAD = 10034,
 NFSERR_RECLAIM_CONFLICT = 10035,
 NFSERR_BAD_XDR = 10036,
 NFSERR_LOCKS_HELD = 10037,
 NFSERR_OPENMODE = 10038,
 NFSERR_BADOWNER = 10039,
 NFSERR_BADCHAR = 10040,
 NFSERR_BADNAME = 10041,
 NFSERR_BAD_RANGE = 10042,
 NFSERR_LOCK_NOTSUPP = 10043,
 NFSERR_OP_ILLEGAL = 10044,
 NFSERR_DEADLOCK = 10045,
 NFSERR_FILE_OPEN = 10046,
 NFSERR_ADMIN_REVOKED = 10047,
 NFSERR_CB_PATH_DOWN = 10048,
 NFSERR_REPLAY_ME = 10049
};



enum nfs_ftype {
 NFNON = 0,
 NFREG = 1,
 NFDIR = 2,
 NFBLK = 3,
 NFCHR = 4,
 NFLNK = 5,
 NFSOCK = 6,
 NFBAD = 7,
 NFFIFO = 8
};




typedef u32 rpc_authflavor_t;

enum rpc_auth_flavors {
 RPC_AUTH_NULL = 0,
 RPC_AUTH_UNIX = 1,
 RPC_AUTH_SHORT = 2,
 RPC_AUTH_DES = 3,
 RPC_AUTH_KRB = 4,
 RPC_AUTH_GSS = 6,
 RPC_AUTH_MAXFLAVOR = 8,

 RPC_AUTH_GSS_KRB5 = 390003,
 RPC_AUTH_GSS_KRB5I = 390004,
 RPC_AUTH_GSS_KRB5P = 390005,
 RPC_AUTH_GSS_LKEY = 390006,
 RPC_AUTH_GSS_LKEYI = 390007,
 RPC_AUTH_GSS_LKEYP = 390008,
 RPC_AUTH_GSS_SPKM = 390009,
 RPC_AUTH_GSS_SPKMI = 390010,
 RPC_AUTH_GSS_SPKMP = 390011,
};

enum rpc_msg_type {
 RPC_CALL = 0,
 RPC_REPLY = 1
};

enum rpc_reply_stat {
 RPC_MSG_ACCEPTED = 0,
 RPC_MSG_DENIED = 1
};

enum rpc_accept_stat {
 RPC_SUCCESS = 0,
 RPC_PROG_UNAVAIL = 1,
 RPC_PROG_MISMATCH = 2,
 RPC_PROC_UNAVAIL = 3,
 RPC_GARBAGE_ARGS = 4,
 RPC_SYSTEM_ERR = 5
};

enum rpc_reject_stat {
 RPC_MISMATCH = 0,
 RPC_AUTH_ERROR = 1
};

enum rpc_auth_stat {
 RPC_AUTH_OK = 0,
 RPC_AUTH_BADCRED = 1,
 RPC_AUTH_REJECTEDCRED = 2,
 RPC_AUTH_BADVERF = 3,
 RPC_AUTH_REJECTEDVERF = 4,
 RPC_AUTH_TOOWEAK = 5,

 RPCSEC_GSS_CREDPROBLEM = 13,
 RPCSEC_GSS_CTXPROBLEM = 14
};

typedef u32 rpc_fraghdr;







struct nfs_fh {
 unsigned short size;
 unsigned char data[128];
};





static inline __attribute__((always_inline)) int nfs_compare_fh(const struct nfs_fh *a, const struct nfs_fh *b)
{
 return a->size != b->size || __builtin_memcmp(a->data, b->data, a->size) != 0;
}

static inline __attribute__((always_inline)) void nfs_copy_fh(struct nfs_fh *target, const struct nfs_fh *source)
{
 target->size = source->size;
 (__builtin_constant_p(source->size) ? __constant_memcpy((target->data),(source->data),(source->size)) : __memcpy((target->data),(source->data),(source->size)));
}

enum nfs3_stable_how {
 NFS_UNSTABLE = 0,
 NFS_DATA_SYNC = 1,
 NFS_FILE_SYNC = 2
};


struct nlm_lockowner;




struct nfs_lock_info {
 u32 state;
 struct nlm_lockowner *owner;
 struct list_head list;
};

struct nfs4_lock_state;
struct nfs4_lock_info {
 struct nfs4_lock_state *owner;
};


struct file_lock {
 struct file_lock *fl_next;
 struct list_head fl_link;
 struct list_head fl_block;
 fl_owner_t fl_owner;
 unsigned int fl_pid;
 wait_queue_head_t fl_wait;
 struct file *fl_file;
 unsigned char fl_flags;
 unsigned char fl_type;
 loff_t fl_start;
 loff_t fl_end;

 struct fasync_struct * fl_fasync;
 unsigned long fl_break_time;

 struct file_lock_operations *fl_ops;
 struct lock_manager_operations *fl_lmops;
 union {
  struct nfs_lock_info nfs_fl;
  struct nfs4_lock_info nfs4_fl;
 } fl_u;
};








struct flock {
 short l_type;
 short l_whence;
 off_t l_start;
 off_t l_len;
 pid_t l_pid;

};

struct flock64 {
 short l_type;
 short l_whence;
 loff_t l_start;
 loff_t l_len;
 pid_t l_pid;

};




extern int fcntl_getlk(struct file *, struct flock *);
extern int fcntl_setlk(unsigned int, struct file *, unsigned int,
   struct flock *);


extern int fcntl_getlk64(struct file *, struct flock64 *);
extern int fcntl_setlk64(unsigned int, struct file *, unsigned int,
   struct flock64 *);


extern void send_sigio(struct fown_struct *fown, int fd, int band);
extern int fcntl_setlease(unsigned int fd, struct file *filp, long arg);
extern int fcntl_getlease(struct file *filp);


extern int do_sync_file_range(struct file *file, loff_t offset, loff_t endbyte,
   unsigned int flags);


extern void locks_init_lock(struct file_lock *);
extern void locks_copy_lock(struct file_lock *, struct file_lock *);
extern void locks_remove_posix(struct file *, fl_owner_t);
extern void locks_remove_flock(struct file *);
extern int posix_test_lock(struct file *, struct file_lock *, struct file_lock *);
extern int posix_lock_file_conf(struct file *, struct file_lock *, struct file_lock *);
extern int posix_lock_file(struct file *, struct file_lock *);
extern int posix_lock_file_wait(struct file *, struct file_lock *);
extern int posix_unblock_lock(struct file *, struct file_lock *);
extern int flock_lock_file_wait(struct file *filp, struct file_lock *fl);
extern int __break_lease(struct inode *inode, unsigned int flags);
extern void lease_get_mtime(struct inode *, struct timespec *time);
extern int setlease(struct file *, long, struct file_lock **);
extern int lease_modify(struct file_lock **, int);
extern int lock_may_read(struct inode *, loff_t start, unsigned long count);
extern int lock_may_write(struct inode *, loff_t start, unsigned long count);

struct fasync_struct {
 int magic;
 int fa_fd;
 struct fasync_struct *fa_next;
 struct file *fa_file;
};




extern int fasync_helper(int, struct file *, int, struct fasync_struct **);

extern void kill_fasync(struct fasync_struct **, int, int);

extern void __kill_fasync(struct fasync_struct *, int, int);

extern int f_setown(struct file *filp, unsigned long arg, int force);
extern void f_delown(struct file *filp);
extern int send_sigurg(struct fown_struct *fown);

extern struct list_head super_blocks;
extern spinlock_t sb_lock;



struct super_block {
 struct list_head s_list;
 dev_t s_dev;
 unsigned long s_blocksize;
 unsigned char s_blocksize_bits;
 unsigned char s_dirt;
 unsigned long long s_maxbytes;
 struct file_system_type *s_type;
 struct super_operations *s_op;
 struct dquot_operations *dq_op;
  struct quotactl_ops *s_qcop;
 struct export_operations *s_export_op;
 unsigned long s_flags;
 unsigned long s_magic;
 struct dentry *s_root;
 struct rw_semaphore s_umount;
 struct mutex s_lock;
 int s_count;
 int s_syncing;
 int s_need_sync_fs;
 atomic_t s_active;
 void *s_security;
 struct xattr_handler **s_xattr;

 struct list_head s_inodes;
 struct list_head s_dirty;
 struct list_head s_io;
 struct hlist_head s_anon;
 struct list_head s_files;

 struct block_device *s_bdev;
 struct list_head s_instances;
 struct quota_info s_dquot;

 int s_frozen;
 wait_queue_head_t s_wait_unfrozen;

 char s_id[32];

 void *s_fs_info;





 struct mutex s_vfs_rename_mutex;



 u32 s_time_gran;
};

extern struct timespec current_fs_time(struct super_block *sb);




enum {
 SB_UNFROZEN = 0,
 SB_FREEZE_WRITE = 1,
 SB_FREEZE_TRANS = 2,
};




static inline __attribute__((always_inline)) void get_fs_excl(void)
{
 atomic_inc(&__vericon_dummy_current->fs_excl);
}

static inline __attribute__((always_inline)) void put_fs_excl(void)
{
 atomic_dec(&__vericon_dummy_current->fs_excl);
}

static inline __attribute__((always_inline)) int has_fs_excl(void)
{
 return ((&__vericon_dummy_current->fs_excl)->counter);
}





static inline __attribute__((always_inline)) void lock_super(struct super_block * sb)
{
 get_fs_excl();
 mutex_lock(&sb->s_lock);
}

static inline __attribute__((always_inline)) void unlock_super(struct super_block * sb)
{
 put_fs_excl();
 mutex_unlock(&sb->s_lock);
}




extern int vfs_permission(struct nameidata *, int);
extern int vfs_create(struct inode *, struct dentry *, int, struct nameidata *);
extern int vfs_mkdir(struct inode *, struct dentry *, int);
extern int vfs_mknod(struct inode *, struct dentry *, int, dev_t);
extern int vfs_symlink(struct inode *, struct dentry *, const char *, int);
extern int vfs_link(struct dentry *, struct inode *, struct dentry *);
extern int vfs_rmdir(struct inode *, struct dentry *);
extern int vfs_unlink(struct inode *, struct dentry *);
extern int vfs_rename(struct inode *, struct dentry *, struct inode *, struct dentry *);




extern void dentry_unhash(struct dentry *dentry);




extern int file_permission(struct file *, int);

int generic_osync_inode(struct inode *, struct address_space *, int);







typedef int (*filldir_t)(void *, const char *, int, loff_t, ino_t, unsigned);

struct block_device_operations {
 int (*open) (struct inode *, struct file *);
 int (*release) (struct inode *, struct file *);
 int (*ioctl) (struct inode *, struct file *, unsigned, unsigned long);
 long (*unlocked_ioctl) (struct file *, unsigned, unsigned long);
 long (*compat_ioctl) (struct file *, unsigned, unsigned long);
 int (*direct_access) (struct block_device *, sector_t, unsigned long *);
 int (*media_changed) (struct gendisk *);
 int (*revalidate_disk) (struct gendisk *);
 int (*getgeo)(struct block_device *, struct hd_geometry *);
 struct module *owner;
};

typedef struct {
 size_t written;
 size_t count;
 union {
  char * buf;
  void *data;
 } arg;
 int error;
} read_descriptor_t;

typedef int (*read_actor_t)(read_descriptor_t *, struct page *, unsigned long, unsigned long);

struct file_operations {
 struct module *owner;
 loff_t (*llseek) (struct file *, loff_t, int);
 ssize_t (*read) (struct file *, char *, size_t, loff_t *);
 ssize_t (*aio_read) (struct kiocb *, char *, size_t, loff_t);
 ssize_t (*write) (struct file *, const char *, size_t, loff_t *);
 ssize_t (*aio_write) (struct kiocb *, const char *, size_t, loff_t);
 int (*readdir) (struct file *, void *, filldir_t);
 unsigned int (*poll) (struct file *, struct poll_table_struct *);
 int (*ioctl) (struct inode *, struct file *, unsigned int, unsigned long);
 long (*unlocked_ioctl) (struct file *, unsigned int, unsigned long);
 long (*compat_ioctl) (struct file *, unsigned int, unsigned long);
 int (*mmap) (struct file *, struct vm_area_struct *);
 int (*open) (struct inode *, struct file *);
 int (*flush) (struct file *, fl_owner_t id);
 int (*release) (struct inode *, struct file *);
 int (*fsync) (struct file *, struct dentry *, int datasync);
 int (*aio_fsync) (struct kiocb *, int datasync);
 int (*fasync) (int, struct file *, int);
 int (*lock) (struct file *, int, struct file_lock *);
 ssize_t (*readv) (struct file *, const struct iovec *, unsigned long, loff_t *);
 ssize_t (*writev) (struct file *, const struct iovec *, unsigned long, loff_t *);
 ssize_t (*sendfile) (struct file *, loff_t *, size_t, read_actor_t, void *);
 ssize_t (*sendpage) (struct file *, struct page *, int, size_t, loff_t *, int);
 unsigned long (*get_unmapped_area)(struct file *, unsigned long, unsigned long, unsigned long, unsigned long);
 int (*check_flags)(int);
 int (*dir_notify)(struct file *filp, unsigned long arg);
 int (*flock) (struct file *, int, struct file_lock *);
 ssize_t (*splice_write)(struct pipe_inode_info *, struct file *, loff_t *, size_t, unsigned int);
 ssize_t (*splice_read)(struct file *, loff_t *, struct pipe_inode_info *, size_t, unsigned int);
};

struct inode_operations {
 int (*create) (struct inode *,struct dentry *,int, struct nameidata *);
 struct dentry * (*lookup) (struct inode *,struct dentry *, struct nameidata *);
 int (*link) (struct dentry *,struct inode *,struct dentry *);
 int (*unlink) (struct inode *,struct dentry *);
 int (*symlink) (struct inode *,struct dentry *,const char *);
 int (*mkdir) (struct inode *,struct dentry *,int);
 int (*rmdir) (struct inode *,struct dentry *);
 int (*mknod) (struct inode *,struct dentry *,int,dev_t);
 int (*rename) (struct inode *, struct dentry *,
   struct inode *, struct dentry *);
 int (*readlink) (struct dentry *, char *,int);
 void * (*follow_link) (struct dentry *, struct nameidata *);
 void (*put_link) (struct dentry *, struct nameidata *, void *);
 void (*truncate) (struct inode *);
 int (*permission) (struct inode *, int, struct nameidata *);
 int (*setattr) (struct dentry *, struct iattr *);
 int (*getattr) (struct vfsmount *mnt, struct dentry *, struct kstat *);
 int (*setxattr) (struct dentry *, const char *,const void *,size_t,int);
 ssize_t (*getxattr) (struct dentry *, const char *, void *, size_t);
 ssize_t (*listxattr) (struct dentry *, char *, size_t);
 int (*removexattr) (struct dentry *, const char *);
 void (*truncate_range)(struct inode *, loff_t, loff_t);
};

struct seq_file;

extern ssize_t vfs_read(struct file *, char *, size_t, loff_t *);
extern ssize_t vfs_write(struct file *, const char *, size_t, loff_t *);
extern ssize_t vfs_readv(struct file *, const struct iovec *,
  unsigned long, loff_t *);
extern ssize_t vfs_writev(struct file *, const struct iovec *,
  unsigned long, loff_t *);





struct super_operations {
    struct inode *(*alloc_inode)(struct super_block *sb);
 void (*destroy_inode)(struct inode *);

 void (*read_inode) (struct inode *);

    void (*dirty_inode) (struct inode *);
 int (*write_inode) (struct inode *, int);
 void (*put_inode) (struct inode *);
 void (*drop_inode) (struct inode *);
 void (*delete_inode) (struct inode *);
 void (*put_super) (struct super_block *);
 void (*write_super) (struct super_block *);
 int (*sync_fs)(struct super_block *sb, int wait);
 void (*write_super_lockfs) (struct super_block *);
 void (*unlockfs) (struct super_block *);
 int (*statfs) (struct dentry *, struct kstatfs *);
 int (*remount_fs) (struct super_block *, int *, char *);
 void (*clear_inode) (struct inode *);
 void (*umount_begin) (struct vfsmount *, int);

 int (*show_options)(struct seq_file *, struct vfsmount *);
 int (*show_stats)(struct seq_file *, struct vfsmount *);

 ssize_t (*quota_read)(struct super_block *, int, char *, size_t, loff_t);
 ssize_t (*quota_write)(struct super_block *, int, const char *, size_t, loff_t);
};

extern void __mark_inode_dirty(struct inode *, int);
static inline __attribute__((always_inline)) void mark_inode_dirty(struct inode *inode)
{
 __mark_inode_dirty(inode, (1 | 2 | 4));
}

static inline __attribute__((always_inline)) void mark_inode_dirty_sync(struct inode *inode)
{
 __mark_inode_dirty(inode, 1);
}

static inline __attribute__((always_inline)) void inode_inc_link_count(struct inode *inode)
{
 inode->i_nlink++;
 mark_inode_dirty(inode);
}

static inline __attribute__((always_inline)) void inode_dec_link_count(struct inode *inode)
{
 inode->i_nlink--;
 mark_inode_dirty(inode);
}

extern void touch_atime(struct vfsmount *mnt, struct dentry *dentry);
static inline __attribute__((always_inline)) void file_accessed(struct file *file)
{
 if (!(file->f_flags & 01000000))
  touch_atime(file->f_vfsmnt, file->f_dentry);
}

int sync_inode(struct inode *inode, struct writeback_control *wbc);

struct export_operations {
 struct dentry *(*decode_fh)(struct super_block *sb, __u32 *fh, int fh_len, int fh_type,
    int (*acceptable)(void *context, struct dentry *de),
    void *context);
 int (*encode_fh)(struct dentry *de, __u32 *fh, int *max_len,
    int connectable);


 int (*get_name)(struct dentry *parent, char *name,
   struct dentry *child);
 struct dentry * (*get_parent)(struct dentry *child);
 struct dentry * (*get_dentry)(struct super_block *sb, void *inump);


 struct dentry * (*find_exported_dentry)(
  struct super_block *sb, void *obj, void *parent,
  int (*acceptable)(void *context, struct dentry *de),
  void *context);


};

extern struct dentry *
find_exported_dentry(struct super_block *sb, void *obj, void *parent,
       int (*acceptable)(void *context, struct dentry *de),
       void *context);

struct file_system_type {
 const char *name;
 int fs_flags;
 int (*get_sb) (struct file_system_type *, int,
         const char *, void *, struct vfsmount *);
 void (*kill_sb) (struct super_block *);
 struct module *owner;
 struct file_system_type * next;
 struct list_head fs_supers;
 struct lock_class_key s_lock_key;
 struct lock_class_key s_umount_key;
};

extern int get_sb_bdev(struct file_system_type *fs_type,
 int flags, const char *dev_name, void *data,
 int (*fill_super)(struct super_block *, void *, int),
 struct vfsmount *mnt);
extern int get_sb_single(struct file_system_type *fs_type,
 int flags, void *data,
 int (*fill_super)(struct super_block *, void *, int),
 struct vfsmount *mnt);
extern int get_sb_nodev(struct file_system_type *fs_type,
 int flags, void *data,
 int (*fill_super)(struct super_block *, void *, int),
 struct vfsmount *mnt);
void generic_shutdown_super(struct super_block *sb);
void kill_block_super(struct super_block *sb);
void kill_anon_super(struct super_block *sb);
void kill_litter_super(struct super_block *sb);
void deactivate_super(struct super_block *sb);
int set_anon_super(struct super_block *s, void *data);
struct super_block *sget(struct file_system_type *type,
   int (*test)(struct super_block *,void *),
   int (*set)(struct super_block *,void *),
   void *data);
extern int get_sb_pseudo(struct file_system_type *, char *,
 struct super_operations *ops, unsigned long,
 struct vfsmount *mnt);
extern int simple_set_mnt(struct vfsmount *mnt, struct super_block *sb);
int __put_super(struct super_block *sb);
int __put_super_and_need_restart(struct super_block *sb);
void unnamed_dev_init(void);







extern int register_filesystem(struct file_system_type *);
extern int unregister_filesystem(struct file_system_type *);
extern struct vfsmount *kern_mount(struct file_system_type *);
extern int may_umount_tree(struct vfsmount *);
extern int may_umount(struct vfsmount *);
extern void umount_tree(struct vfsmount *, int, struct list_head *);
extern void release_mounts(struct list_head *);
extern long do_mount(char *, char *, char *, unsigned long, void *);
extern struct vfsmount *copy_tree(struct vfsmount *, struct dentry *, int);
extern void mnt_set_mountpoint(struct vfsmount *, struct dentry *,
      struct vfsmount *);

extern int vfs_statfs(struct dentry *, struct kstatfs *);


extern struct subsystem fs_subsys;




extern int locks_mandatory_locked(struct inode *);
extern int locks_mandatory_area(int, struct inode *, struct file *, loff_t, size_t);

static inline __attribute__((always_inline)) int locks_verify_locked(struct inode *inode)
{
 if ((((inode)->i_sb->s_flags & (64)) && ((inode)->i_mode & (0002000 | 00010)) == 0002000))
  return locks_mandatory_locked(inode);
 return 0;
}

extern int rw_verify_area(int, struct file *, loff_t *, size_t);

static inline __attribute__((always_inline)) int locks_verify_truncate(struct inode *inode,
        struct file *filp,
        loff_t size)
{
 if (inode->i_flock && (((inode)->i_sb->s_flags & (64)) && ((inode)->i_mode & (0002000 | 00010)) == 0002000))
  return locks_mandatory_area(
   2, inode, filp,
   size < inode->i_size ? size : inode->i_size,
   (size < inode->i_size ? inode->i_size - size
    : size - inode->i_size)
  );
 return 0;
}

static inline __attribute__((always_inline)) int break_lease(struct inode *inode, unsigned int mode)
{
 if (inode->i_flock)
  return __break_lease(inode, mode);
 return 0;
}



extern int do_truncate(struct dentry *, loff_t start, unsigned int time_attrs,
         struct file *filp);
extern long do_sys_open(int fdf, const char *filename, int flags,
   int mode);
extern struct file *filp_open(const char *, int, int);
extern struct file * dentry_open(struct dentry *, struct vfsmount *, int);
extern int filp_close(struct file *, fl_owner_t id);
extern char * getname(const char *);


extern void __attribute__ ((__section__ (".init.text"))) vfs_caches_init_early(void);
extern void __attribute__ ((__section__ (".init.text"))) vfs_caches_init(unsigned long);






extern void putname(const char *name);


extern int register_blkdev(unsigned int, const char *);
extern int unregister_blkdev(unsigned int, const char *);
extern struct block_device *bdget(dev_t);
extern void bd_set_size(struct block_device *, loff_t size);
extern void bd_forget(struct inode *inode);
extern void bdput(struct block_device *);
extern struct block_device *open_by_devnum(dev_t, unsigned);
extern struct block_device *open_partition_by_devnum(dev_t, unsigned);
extern const struct file_operations def_blk_fops;
extern const struct address_space_operations def_blk_aops;
extern const struct file_operations def_chr_fops;
extern const struct file_operations bad_sock_fops;
extern const struct file_operations def_fifo_fops;
extern int ioctl_by_bdev(struct block_device *, unsigned, unsigned long);
extern int blkdev_ioctl(struct inode *, struct file *, unsigned, unsigned long);
extern long compat_blkdev_ioctl(struct file *, unsigned, unsigned long);
extern int blkdev_get(struct block_device *, mode_t, unsigned);
extern int blkdev_put(struct block_device *);
extern int blkdev_put_partition(struct block_device *);
extern int bd_claim(struct block_device *, void *);
extern void bd_release(struct block_device *);

extern int bd_claim_by_disk(struct block_device *, void *, struct gendisk *);
extern void bd_release_from_disk(struct block_device *, struct gendisk *);







extern int alloc_chrdev_region(dev_t *, unsigned, unsigned, const char *);
extern int register_chrdev_region(dev_t, unsigned, const char *);
extern int register_chrdev(unsigned int, const char *,
      const struct file_operations *);
extern int unregister_chrdev(unsigned int, const char *);
extern void unregister_chrdev_region(dev_t, unsigned);
extern int chrdev_open(struct inode *, struct file *);
extern void chrdev_show(struct seq_file *,off_t);




extern const char *__bdevname(dev_t, char *buffer);
extern const char *bdevname(struct block_device *bdev, char *buffer);
extern struct block_device *lookup_bdev(const char *);
extern struct block_device *open_bdev_excl(const char *, int, void *);
extern void close_bdev_excl(struct block_device *);
extern void blkdev_show(struct seq_file *,off_t);

extern void init_special_inode(struct inode *, umode_t, dev_t);


extern void make_bad_inode(struct inode *);
extern int is_bad_inode(struct inode *);

extern const struct file_operations read_fifo_fops;
extern const struct file_operations write_fifo_fops;
extern const struct file_operations rdwr_fifo_fops;

extern int fs_may_remount_ro(struct super_block *);

extern int check_disk_change(struct block_device *);
extern int invalidate_inodes(struct super_block *);
extern int __invalidate_device(struct block_device *);
extern int invalidate_partition(struct gendisk *, int);
unsigned long invalidate_mapping_pages(struct address_space *mapping,
     unsigned long start, unsigned long end);
unsigned long invalidate_inode_pages(struct address_space *mapping);
static inline __attribute__((always_inline)) void invalidate_remote_inode(struct inode *inode)
{
 if ((((inode->i_mode) & 00170000) == 0100000) || (((inode->i_mode) & 00170000) == 0040000) ||
     (((inode->i_mode) & 00170000) == 0120000))
  invalidate_inode_pages(inode->i_mapping);
}
extern int invalidate_inode_pages2(struct address_space *mapping);
extern int invalidate_inode_pages2_range(struct address_space *mapping,
      unsigned long start, unsigned long end);
extern int write_inode_now(struct inode *, int);
extern int filemap_fdatawrite(struct address_space *);
extern int filemap_flush(struct address_space *);
extern int filemap_fdatawait(struct address_space *);
extern int filemap_write_and_wait(struct address_space *mapping);
extern int filemap_write_and_wait_range(struct address_space *mapping,
            loff_t lstart, loff_t lend);
extern int wait_on_page_writeback_range(struct address_space *mapping,
    unsigned long start, unsigned long end);
extern int __filemap_fdatawrite_range(struct address_space *mapping,
    loff_t start, loff_t end, int sync_mode);

extern long do_fsync(struct file *file, int datasync);
extern void sync_supers(void);
extern void sync_filesystems(int wait);
extern void emergency_sync(void);
extern void emergency_remount(void);
extern int do_remount_sb(struct super_block *sb, int flags,
    void *data, int force);
extern sector_t bmap(struct inode *, sector_t);
extern int notify_change(struct dentry *, struct iattr *);
extern int permission(struct inode *, int, struct nameidata *);
extern int generic_permission(struct inode *, int,
  int (*check_acl)(struct inode *, int));

extern int get_write_access(struct inode *);
extern int deny_write_access(struct file *);
static inline __attribute__((always_inline)) void put_write_access(struct inode * inode)
{
 atomic_dec(&inode->i_writecount);
}
static inline __attribute__((always_inline)) void allow_write_access(struct file *file)
{
 if (file)
  atomic_inc(&file->f_dentry->d_inode->i_writecount);
}
extern int do_pipe(int *);

extern int open_namei(int dfd, const char *, int, int, struct nameidata *);
extern int may_open(struct nameidata *, int, int);

extern int kernel_read(struct file *, unsigned long, char *, unsigned long);
extern struct file * open_exec(const char *);


extern int is_subdir(struct dentry *, struct dentry *);
extern ino_t find_inode_number(struct dentry *, struct qstr *);




extern loff_t default_llseek(struct file *file, loff_t offset, int origin);

extern loff_t vfs_llseek(struct file *file, loff_t offset, int origin);

extern void inode_init_once(struct inode *);
extern void iput(struct inode *);
extern struct inode * igrab(struct inode *);
extern ino_t iunique(struct super_block *, ino_t);
extern int inode_needs_sync(struct inode *inode);
extern void generic_delete_inode(struct inode *inode);
extern void generic_drop_inode(struct inode *inode);

extern struct inode *ilookup5_nowait(struct super_block *sb,
  unsigned long hashval, int (*test)(struct inode *, void *),
  void *data);
extern struct inode *ilookup5(struct super_block *sb, unsigned long hashval,
  int (*test)(struct inode *, void *), void *data);
extern struct inode *ilookup(struct super_block *sb, unsigned long ino);

extern struct inode * iget5_locked(struct super_block *, unsigned long, int (*test)(struct inode *, void *), int (*set)(struct inode *, void *), void *);
extern struct inode * iget_locked(struct super_block *, unsigned long);
extern void unlock_new_inode(struct inode *);

static inline __attribute__((always_inline)) struct inode *iget(struct super_block *sb, unsigned long ino)
{
 struct inode *inode = iget_locked(sb, ino);

 if (inode && (inode->i_state & 64)) {
  sb->s_op->read_inode(inode);
  unlock_new_inode(inode);
 }

 return inode;
}

extern void __iget(struct inode * inode);
extern void clear_inode(struct inode *);
extern void destroy_inode(struct inode *);
extern struct inode *new_inode(struct super_block *);
extern int remove_suid(struct dentry *);
extern void remove_dquot_ref(struct super_block *, int, struct list_head *);

extern void __insert_inode_hash(struct inode *, unsigned long hashval);
extern void remove_inode_hash(struct inode *);
static inline __attribute__((always_inline)) void insert_inode_hash(struct inode *inode) {
 __insert_inode_hash(inode, inode->i_ino);
}

extern struct file * get_empty_filp(void);
extern void file_move(struct file *f, struct list_head *list);
extern void file_kill(struct file *f);
struct bio;
extern void submit_bio(int, struct bio *);
extern int bdev_read_only(struct block_device *);
extern int set_blocksize(struct block_device *, int);
extern int sb_set_blocksize(struct super_block *, int);
extern int sb_min_blocksize(struct super_block *, int);

extern int generic_file_mmap(struct file *, struct vm_area_struct *);
extern int generic_file_readonly_mmap(struct file *, struct vm_area_struct *);
extern int file_read_actor(read_descriptor_t * desc, struct page *page, unsigned long offset, unsigned long size);
extern int file_send_actor(read_descriptor_t * desc, struct page *page, unsigned long offset, unsigned long size);
extern ssize_t generic_file_read(struct file *, char *, size_t, loff_t *);
int generic_write_checks(struct file *file, loff_t *pos, size_t *count, int isblk);
extern ssize_t generic_file_write(struct file *, const char *, size_t, loff_t *);
extern ssize_t generic_file_aio_read(struct kiocb *, char *, size_t, loff_t);
extern ssize_t __generic_file_aio_read(struct kiocb *, const struct iovec *, unsigned long, loff_t *);
extern ssize_t generic_file_aio_write(struct kiocb *, const char *, size_t, loff_t);
extern ssize_t generic_file_aio_write_nolock(struct kiocb *, const struct iovec *,
  unsigned long, loff_t *);
extern ssize_t generic_file_direct_write(struct kiocb *, const struct iovec *,
  unsigned long *, loff_t, loff_t *, size_t, size_t);
extern ssize_t generic_file_buffered_write(struct kiocb *, const struct iovec *,
  unsigned long, loff_t, loff_t *, size_t, ssize_t);
extern ssize_t do_sync_read(struct file *filp, char *buf, size_t len, loff_t *ppos);
extern ssize_t do_sync_write(struct file *filp, const char *buf, size_t len, loff_t *ppos);
ssize_t generic_file_write_nolock(struct file *file, const struct iovec *iov,
    unsigned long nr_segs, loff_t *ppos);
extern ssize_t generic_file_sendfile(struct file *, loff_t *, size_t, read_actor_t, void *);
extern void do_generic_mapping_read(struct address_space *mapping,
        struct file_ra_state *, struct file *,
        loff_t *, read_descriptor_t *, read_actor_t);


extern ssize_t generic_file_splice_read(struct file *, loff_t *,
  struct pipe_inode_info *, size_t, unsigned int);
extern ssize_t generic_file_splice_write(struct pipe_inode_info *,
  struct file *, loff_t *, size_t, unsigned int);
extern ssize_t generic_splice_sendpage(struct pipe_inode_info *pipe,
  struct file *out, loff_t *, size_t len, unsigned int flags);
extern long do_splice_direct(struct file *in, loff_t *ppos, struct file *out,
  size_t len, unsigned int flags);

extern void
file_ra_state_init(struct file_ra_state *ra, struct address_space *mapping);
extern ssize_t generic_file_readv(struct file *filp, const struct iovec *iov,
 unsigned long nr_segs, loff_t *ppos);
ssize_t generic_file_writev(struct file *filp, const struct iovec *iov,
   unsigned long nr_segs, loff_t *ppos);
extern loff_t no_llseek(struct file *file, loff_t offset, int origin);
extern loff_t generic_file_llseek(struct file *file, loff_t offset, int origin);
extern loff_t remote_llseek(struct file *file, loff_t offset, int origin);
extern int generic_file_open(struct inode * inode, struct file * filp);
extern int nonseekable_open(struct inode * inode, struct file * filp);


extern ssize_t xip_file_read(struct file *filp, char *buf, size_t len,
        loff_t *ppos);
extern ssize_t xip_file_sendfile(struct file *in_file, loff_t *ppos,
     size_t count, read_actor_t actor,
     void *target);
extern int xip_file_mmap(struct file * file, struct vm_area_struct * vma);
extern ssize_t xip_file_write(struct file *filp, const char *buf,
         size_t len, loff_t *ppos);
extern int xip_truncate_page(struct address_space *mapping, loff_t from);







static inline __attribute__((always_inline)) void do_generic_file_read(struct file * filp, loff_t *ppos,
     read_descriptor_t * desc,
     read_actor_t actor)
{
 do_generic_mapping_read(filp->f_mapping,
    &filp->f_ra,
    filp,
    ppos,
    desc,
    actor);
}

ssize_t __blockdev_direct_IO(int rw, struct kiocb *iocb, struct inode *inode,
 struct block_device *bdev, const struct iovec *iov, loff_t offset,
 unsigned long nr_segs, get_block_t get_block, dio_iodone_t end_io,
 int lock_type);

enum {
 DIO_LOCKING = 1,
 DIO_NO_LOCKING,
 DIO_OWN_LOCKING,
};

static inline __attribute__((always_inline)) ssize_t blockdev_direct_IO(int rw, struct kiocb *iocb,
 struct inode *inode, struct block_device *bdev, const struct iovec *iov,
 loff_t offset, unsigned long nr_segs, get_block_t get_block,
 dio_iodone_t end_io)
{
 return __blockdev_direct_IO(rw, iocb, inode, bdev, iov, offset,
    nr_segs, get_block, end_io, DIO_LOCKING);
}

static inline __attribute__((always_inline)) ssize_t blockdev_direct_IO_no_locking(int rw, struct kiocb *iocb,
 struct inode *inode, struct block_device *bdev, const struct iovec *iov,
 loff_t offset, unsigned long nr_segs, get_block_t get_block,
 dio_iodone_t end_io)
{
 return __blockdev_direct_IO(rw, iocb, inode, bdev, iov, offset,
    nr_segs, get_block, end_io, DIO_NO_LOCKING);
}

static inline __attribute__((always_inline)) ssize_t blockdev_direct_IO_own_locking(int rw, struct kiocb *iocb,
 struct inode *inode, struct block_device *bdev, const struct iovec *iov,
 loff_t offset, unsigned long nr_segs, get_block_t get_block,
 dio_iodone_t end_io)
{
 return __blockdev_direct_IO(rw, iocb, inode, bdev, iov, offset,
    nr_segs, get_block, end_io, DIO_OWN_LOCKING);
}

extern const struct file_operations generic_ro_fops;



extern int vfs_readlink(struct dentry *, char *, int, const char *);
extern int vfs_follow_link(struct nameidata *, const char *);
extern int page_readlink(struct dentry *, char *, int);
extern void *page_follow_link_light(struct dentry *, struct nameidata *);
extern void page_put_link(struct dentry *, struct nameidata *, void *);
extern int __page_symlink(struct inode *inode, const char *symname, int len,
  gfp_t gfp_mask);
extern int page_symlink(struct inode *inode, const char *symname, int len);
extern struct inode_operations page_symlink_inode_operations;
extern int generic_readlink(struct dentry *, char *, int);
extern void generic_fillattr(struct inode *, struct kstat *);
extern int vfs_getattr(struct vfsmount *, struct dentry *, struct kstat *);
void inode_add_bytes(struct inode *inode, loff_t bytes);
void inode_sub_bytes(struct inode *inode, loff_t bytes);
loff_t inode_get_bytes(struct inode *inode);
void inode_set_bytes(struct inode *inode, loff_t bytes);

extern int vfs_readdir(struct file *, filldir_t, void *);

extern int vfs_stat(char *, struct kstat *);
extern int vfs_lstat(char *, struct kstat *);
extern int vfs_stat_fd(int dfd, char *, struct kstat *);
extern int vfs_lstat_fd(int dfd, char *, struct kstat *);
extern int vfs_fstat(unsigned int, struct kstat *);

extern int vfs_ioctl(struct file *, unsigned int, unsigned int, unsigned long);

extern struct file_system_type *get_fs_type(const char *name);
extern struct super_block *get_super(struct block_device *);
extern struct super_block *user_get_super(dev_t);
extern void drop_super(struct super_block *sb);

extern int dcache_dir_open(struct inode *, struct file *);
extern int dcache_dir_close(struct inode *, struct file *);
extern loff_t dcache_dir_lseek(struct file *, loff_t, int);
extern int dcache_readdir(struct file *, void *, filldir_t);
extern int simple_getattr(struct vfsmount *, struct dentry *, struct kstat *);
extern int simple_statfs(struct dentry *, struct kstatfs *);
extern int simple_link(struct dentry *, struct inode *, struct dentry *);
extern int simple_unlink(struct inode *, struct dentry *);
extern int simple_rmdir(struct inode *, struct dentry *);
extern int simple_rename(struct inode *, struct dentry *, struct inode *, struct dentry *);
extern int simple_sync_file(struct file *, struct dentry *, int);
extern int simple_empty(struct dentry *);
extern int simple_readpage(struct file *file, struct page *page);
extern int simple_prepare_write(struct file *file, struct page *page,
   unsigned offset, unsigned to);
extern int simple_commit_write(struct file *file, struct page *page,
    unsigned offset, unsigned to);

extern struct dentry *simple_lookup(struct inode *, struct dentry *, struct nameidata *);
extern ssize_t generic_read_dir(struct file *, char *, size_t, loff_t *);
extern const struct file_operations simple_dir_operations;
extern struct inode_operations simple_dir_inode_operations;
struct tree_descr { char *name; const struct file_operations *ops; int mode; };
struct dentry *d_alloc_name(struct dentry *, const char *);
extern int simple_fill_super(struct super_block *, int, struct tree_descr *);
extern int simple_pin_fs(struct file_system_type *, struct vfsmount **mount, int *count);
extern void simple_release_fs(struct vfsmount **mount, int *count);

extern ssize_t simple_read_from_buffer(void *, size_t, loff_t *, const void *, size_t);

extern int inode_change_ok(struct inode *, struct iattr *);
extern int __attribute__((warn_unused_result)) inode_setattr(struct inode *, struct iattr *);

extern void file_update_time(struct file *file);

static inline __attribute__((always_inline)) ino_t parent_ino(struct dentry *dentry)
{
 ino_t res;

 _spin_lock(&dentry->d_lock);
 res = dentry->d_parent->d_inode->i_ino;
 _spin_unlock(&dentry->d_lock);
 return res;
}


extern int unshare_files(void);







struct simple_transaction_argresp {
 ssize_t size;
 char data[0];
};



char *simple_transaction_get(struct file *file, const char *buf,
    size_t size);
ssize_t simple_transaction_read(struct file *file, char *buf,
    size_t size, loff_t *pos);
int simple_transaction_release(struct inode *inode, struct file *file);

static inline __attribute__((always_inline)) void simple_transaction_set(struct file *file, size_t n)
{
 struct simple_transaction_argresp *ar = file->private_data;

 do { if (__builtin_expect(!!((n > ((1UL << 12) - sizeof(struct simple_transaction_argresp)))!=0), 0)) __asm__ __volatile__( "ud2\n" "\t.word %c0\n" "\t.long %c1\n" : : "i" (1867), "i" ("include/linux/fs.h")); } while(0);





 __asm__ __volatile__("": : :"memory");
 ar->size = n;
}

static inline __attribute__((always_inline)) void __attribute__((format(printf, 1, 2)))
__simple_attr_check_format(const char *fmt, ...)
{

}

int simple_attr_open(struct inode *inode, struct file *file,
       u64 (*get)(void *), void (*set)(void *, u64),
       const char *fmt);
int simple_attr_close(struct inode *inode, struct file *file);
ssize_t simple_attr_read(struct file *file, char *buf,
    size_t len, loff_t *ppos);
ssize_t simple_attr_write(struct file *file, const char *buf,
     size_t len, loff_t *ppos);



static inline __attribute__((always_inline)) char *alloc_secdata(void)
{
 return (char *)get_zeroed_page(((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u)));
}

static inline __attribute__((always_inline)) void free_secdata(void *secdata)
{
 free_pages(((unsigned long)secdata),0);
}


struct partition {
 unsigned char boot_ind;
 unsigned char head;
 unsigned char sector;
 unsigned char cyl;
 unsigned char sys_ind;
 unsigned char end_head;
 unsigned char end_sector;
 unsigned char end_cyl;
 __le32 start_sect;
 __le32 nr_sects;
} __attribute__((packed));

struct hd_struct {
 sector_t start_sect;
 sector_t nr_sects;
 struct kobject kobj;
 struct kobject *holder_dir;
 unsigned ios[2], sectors[2];
 int policy, partno;
};







struct disk_stats {
 unsigned long sectors[2];
 unsigned long ios[2];
 unsigned long merges[2];
 unsigned long ticks[2];
 unsigned long io_ticks;
 unsigned long time_in_queue;
};

struct gendisk {
 int major;
 int first_minor;
 int minors;

 char disk_name[32];
 struct hd_struct **part;
 int part_uevent_suppress;
 struct block_device_operations *fops;
 struct request_queue *queue;
 void *private_data;
 sector_t capacity;

 int flags;
 struct device *driverfs_dev;
 struct kobject kobj;
 struct kobject *holder_dir;
 struct kobject *slave_dir;

 struct timer_rand_state *random;
 int policy;

 atomic_t sync_io;
 unsigned long stamp;
 int in_flight;



 struct disk_stats dkstats;

};


struct disk_attribute {
 struct attribute attr;
 ssize_t (*show)(struct gendisk *, char *);
 ssize_t (*store)(struct gendisk *, const char *, size_t);
};

static inline __attribute__((always_inline)) void disk_stat_set_all(struct gendisk *gendiskp, int value) {
 (__builtin_constant_p(value) ? (__builtin_constant_p((sizeof (struct disk_stats))) ? __constant_c_and_count_memset(((&gendiskp->dkstats)),((0x01010101UL*(unsigned char)(value))),((sizeof (struct disk_stats)))) : __constant_c_memset(((&gendiskp->dkstats)),((0x01010101UL*(unsigned char)(value))),((sizeof (struct disk_stats))))) : (__builtin_constant_p((sizeof (struct disk_stats))) ? __memset_generic((((&gendiskp->dkstats))),(((value))),(((sizeof (struct disk_stats))))) : __memset_generic(((&gendiskp->dkstats)),((value)),((sizeof (struct disk_stats))))));
}

static inline __attribute__((always_inline)) int init_disk_stats(struct gendisk *disk)
{
 return 1;
}

static inline __attribute__((always_inline)) void free_disk_stats(struct gendisk *disk)
{
}



extern void disk_round_stats(struct gendisk *disk);


extern int get_blkdev_list(char *, int);
extern void add_disk(struct gendisk *disk);
extern void del_gendisk(struct gendisk *gp);
extern void unlink_gendisk(struct gendisk *gp);
extern struct gendisk *get_gendisk(dev_t dev, int *part);

extern void set_device_ro(struct block_device *bdev, int flag);
extern void set_disk_ro(struct gendisk *disk, int flag);


extern void add_disk_randomness(struct gendisk *disk);
extern void rand_initialize_disk(struct gendisk *disk);

static inline __attribute__((always_inline)) sector_t get_start_sect(struct block_device *bdev)
{
 return bdev->bd_contains == bdev ? 0 : bdev->bd_part->start_sect;
}
static inline __attribute__((always_inline)) sector_t get_capacity(struct gendisk *disk)
{
 return disk->capacity;
}
static inline __attribute__((always_inline)) void set_capacity(struct gendisk *disk, sector_t size)
{
 disk->capacity = size;
}

struct solaris_x86_slice {
 __le16 s_tag;
 __le16 s_flag;
 __le32 s_start;
 __le32 s_size;
};

struct solaris_x86_vtoc {
 unsigned int v_bootinfo[3];
 __le32 v_sanity;
 __le32 v_version;
 char v_volume[8];
 __le16 v_sectorsz;
 __le16 v_nparts;
 unsigned int v_reserved[10];
 struct solaris_x86_slice
  v_slice[8];
 unsigned int timestamp[8];
 char v_asciilabel[128];
};

struct bsd_disklabel {
 __le32 d_magic;
 __s16 d_type;
 __s16 d_subtype;
 char d_typename[16];
 char d_packname[16];
 __u32 d_secsize;
 __u32 d_nsectors;
 __u32 d_ntracks;
 __u32 d_ncylinders;
 __u32 d_secpercyl;
 __u32 d_secperunit;
 __u16 d_sparespertrack;
 __u16 d_sparespercyl;
 __u32 d_acylinders;
 __u16 d_rpm;
 __u16 d_interleave;
 __u16 d_trackskew;
 __u16 d_cylskew;
 __u32 d_headswitch;
 __u32 d_trkseek;
 __u32 d_flags;

 __u32 d_drivedata[5];

 __u32 d_spare[5];
 __le32 d_magic2;
 __le16 d_checksum;


 __le16 d_npartitions;
 __le32 d_bbsize;
 __le32 d_sbsize;
 struct bsd_partition {
  __le32 p_size;
  __le32 p_offset;
  __le32 p_fsize;
  __u8 p_fstype;
  __u8 p_frag;
  __le16 p_cpg;
 } d_partitions[16];
};

struct unixware_slice {
 __le16 s_label;
 __le16 s_flags;
 __le32 start_sect;
 __le32 nr_sects;
};

struct unixware_disklabel {
 __le32 d_type;
 __le32 d_magic;
 __le32 d_version;
 char d_serial[12];
 __le32 d_ncylinders;
 __le32 d_ntracks;
 __le32 d_nsectors;
 __le32 d_secsize;
 __le32 d_part_start;
 __le32 d_unknown1[12];
  __le32 d_alt_tbl;
  __le32 d_alt_len;
  __le32 d_phys_cyl;
  __le32 d_phys_trk;
  __le32 d_phys_sec;
  __le32 d_phys_bytes;
  __le32 d_unknown2;
 __le32 d_unknown3;
 __le32 d_pad[8];

 struct unixware_vtoc {
  __le32 v_magic;
  __le32 v_version;
  char v_name[8];
  __le16 v_nslices;
  __le16 v_unknown1;
  __le32 v_reserved[10];
  struct unixware_slice
   v_slice[16];
 } vtoc;

};

char *disk_name (struct gendisk *hd, int part, char *buf);

extern int rescan_partitions(struct gendisk *disk, struct block_device *bdev);
extern void add_partition(struct gendisk *, int, sector_t, sector_t);
extern void delete_partition(struct gendisk *, int);

extern struct gendisk *alloc_disk_node(int minors, int node_id);
extern struct gendisk *alloc_disk(int minors);
extern struct kobject *get_disk(struct gendisk *disk);
extern void put_disk(struct gendisk *disk);

extern void blk_register_region(dev_t dev, unsigned long range,
   struct module *module,
   struct kobject *(*probe)(dev_t, int *, void *),
   int (*lock)(dev_t, void *),
   void *data);
extern void blk_unregister_region(dev_t dev, unsigned long range);

static inline __attribute__((always_inline)) struct block_device *bdget_disk(struct gendisk *disk, int index)
{
 return bdget((((disk->major) << 20) | (disk->first_minor)) + index);
}













struct mempolicy;
struct anon_vma;


extern unsigned long max_mapnr;


extern unsigned long num_physpages;
extern void * high_memory;
extern unsigned long vmalloc_earlyreserve;
extern int page_cluster;


extern int sysctl_legacy_va_layout;














static inline __attribute__((always_inline)) int
__acpi_acquire_global_lock (unsigned int *lock)
{
 unsigned int old, new, val;
 do {
  old = *lock;
  new = (((old & ~0x3) + 2) + ((old >> 1) & 0x1));
  val = ((__typeof__(*(lock)))__cmpxchg((lock),(unsigned long)(old), (unsigned long)(new),sizeof(*(lock))));
 } while (__builtin_expect(!!(val != old), 0));
 return (new < 3) ? -1 : 0;
}

static inline __attribute__((always_inline)) int
__acpi_release_global_lock (unsigned int *lock)
{
 unsigned int old, new, val;
 do {
  old = *lock;
  new = old & ~0x3;
  val = ((__typeof__(*(lock)))__cmpxchg((lock),(unsigned long)(old), (unsigned long)(new),sizeof(*(lock))));
 } while (__builtin_expect(!!(val != old), 0));
 return old & 0x1;
}

static inline __attribute__((always_inline)) void check_acpi_pci(void) { }



extern int acpi_lapic;
extern int acpi_ioapic;
extern int acpi_noirq;
extern int acpi_strict;
extern int acpi_disabled;
extern int acpi_ht;
extern int acpi_pci_disabled;
static inline __attribute__((always_inline)) void disable_acpi(void)
{
 acpi_disabled = 1;
 acpi_ht = 0;
 acpi_pci_disabled = 1;
 acpi_noirq = 1;
}




extern int acpi_gsi_to_irq(u32 gsi, unsigned int *irq);

static inline __attribute__((always_inline)) void disable_ioapic_setup(void) { }


static inline __attribute__((always_inline)) void acpi_noirq_set(void) { acpi_noirq = 1; }
static inline __attribute__((always_inline)) void acpi_disable_pci(void)
{
 acpi_pci_disabled = 1;
 acpi_noirq_set();
}
extern int acpi_irq_balance_set(char *str);

extern int acpi_save_state_mem(void);
extern void acpi_restore_state_mem(void);

extern unsigned long acpi_wakeup_address;


extern void acpi_reserve_bootmem(void);



extern u8 x86_acpiid_to_apicid[];



struct local_apic {

        struct { unsigned int __reserved[4]; } __reserved_01;

        struct { unsigned int __reserved[4]; } __reserved_02;

        struct {
  unsigned int __reserved_1 : 24,
   phys_apic_id : 4,
   __reserved_2 : 4;
  unsigned int __reserved[3];
 } id;

        const
 struct {
  unsigned int version : 8,
   __reserved_1 : 8,
   max_lvt : 8,
   __reserved_2 : 8;
  unsigned int __reserved[3];
 } version;

        struct { unsigned int __reserved[4]; } __reserved_03;

        struct { unsigned int __reserved[4]; } __reserved_04;

        struct { unsigned int __reserved[4]; } __reserved_05;

        struct { unsigned int __reserved[4]; } __reserved_06;

        struct {
  unsigned int priority : 8,
   __reserved_1 : 24;
  unsigned int __reserved_2[3];
 } tpr;

        const
 struct {
  unsigned int priority : 8,
   __reserved_1 : 24;
  unsigned int __reserved_2[3];
 } apr;

        const
 struct {
  unsigned int priority : 8,
   __reserved_1 : 24;
  unsigned int __reserved_2[3];
 } ppr;

        struct {
  unsigned int eoi;
  unsigned int __reserved[3];
 } eoi;

        struct { unsigned int __reserved[4]; } __reserved_07;

        struct {
  unsigned int __reserved_1 : 24,
   logical_dest : 8;
  unsigned int __reserved_2[3];
 } ldr;

        struct {
  unsigned int __reserved_1 : 28,
   model : 4;
  unsigned int __reserved_2[3];
 } dfr;

        struct {
  unsigned int spurious_vector : 8,
   apic_enabled : 1,
   focus_cpu : 1,
   __reserved_2 : 22;
  unsigned int __reserved_3[3];
 } svr;

        struct {
         unsigned int bitfield;
  unsigned int __reserved[3];
 } isr [8];

        struct {
         unsigned int bitfield;
  unsigned int __reserved[3];
 } tmr [8];

        struct {
         unsigned int bitfield;
  unsigned int __reserved[3];
 } irr [8];

        union {
  struct {
   unsigned int send_cs_error : 1,
    receive_cs_error : 1,
    send_accept_error : 1,
    receive_accept_error : 1,
    __reserved_1 : 1,
    send_illegal_vector : 1,
    receive_illegal_vector : 1,
    illegal_register_address : 1,
    __reserved_2 : 24;
   unsigned int __reserved_3[3];
  } error_bits;
  struct {
   unsigned int errors;
   unsigned int __reserved_3[3];
  } all_errors;
 } esr;

        struct { unsigned int __reserved[4]; } __reserved_08;

        struct { unsigned int __reserved[4]; } __reserved_09;

        struct { unsigned int __reserved[4]; } __reserved_10;

        struct { unsigned int __reserved[4]; } __reserved_11;

        struct { unsigned int __reserved[4]; } __reserved_12;

        struct { unsigned int __reserved[4]; } __reserved_13;

        struct { unsigned int __reserved[4]; } __reserved_14;

        struct {
  unsigned int vector : 8,
   delivery_mode : 3,
   destination_mode : 1,
   delivery_status : 1,
   __reserved_1 : 1,
   level : 1,
   trigger : 1,
   __reserved_2 : 2,
   shorthand : 2,
   __reserved_3 : 12;
  unsigned int __reserved_4[3];
 } icr1;

        struct {
  union {
   unsigned int __reserved_1 : 24,
    phys_dest : 4,
    __reserved_2 : 4;
   unsigned int __reserved_3 : 24,
    logical_dest : 8;
  } dest;
  unsigned int __reserved_4[3];
 } icr2;

        struct {
  unsigned int vector : 8,
   __reserved_1 : 4,
   delivery_status : 1,
   __reserved_2 : 3,
   mask : 1,
   timer_mode : 1,
   __reserved_3 : 14;
  unsigned int __reserved_4[3];
 } lvt_timer;

        struct {
  unsigned int vector : 8,
   delivery_mode : 3,
   __reserved_1 : 1,
   delivery_status : 1,
   __reserved_2 : 3,
   mask : 1,
   __reserved_3 : 15;
  unsigned int __reserved_4[3];
 } lvt_thermal;

        struct {
  unsigned int vector : 8,
   delivery_mode : 3,
   __reserved_1 : 1,
   delivery_status : 1,
   __reserved_2 : 3,
   mask : 1,
   __reserved_3 : 15;
  unsigned int __reserved_4[3];
 } lvt_pc;

        struct {
  unsigned int vector : 8,
   delivery_mode : 3,
   __reserved_1 : 1,
   delivery_status : 1,
   polarity : 1,
   remote_irr : 1,
   trigger : 1,
   mask : 1,
   __reserved_2 : 15;
  unsigned int __reserved_3[3];
 } lvt_lint0;

        struct {
  unsigned int vector : 8,
   delivery_mode : 3,
   __reserved_1 : 1,
   delivery_status : 1,
   polarity : 1,
   remote_irr : 1,
   trigger : 1,
   mask : 1,
   __reserved_2 : 15;
  unsigned int __reserved_3[3];
 } lvt_lint1;

        struct {
  unsigned int vector : 8,
   __reserved_1 : 4,
   delivery_status : 1,
   __reserved_2 : 3,
   mask : 1,
   __reserved_3 : 15;
  unsigned int __reserved_4[3];
 } lvt_error;

        struct {
  unsigned int initial_count;
  unsigned int __reserved_2[3];
 } timer_icr;

        const
 struct {
  unsigned int curr_count;
  unsigned int __reserved_2[3];
 } timer_ccr;

        struct { unsigned int __reserved[4]; } __reserved_16;

        struct { unsigned int __reserved[4]; } __reserved_17;

        struct { unsigned int __reserved[4]; } __reserved_18;

        struct { unsigned int __reserved[4]; } __reserved_19;

        struct {
  unsigned int divisor : 4,
   __reserved_1 : 28;
  unsigned int __reserved_2[3];
 } timer_dcr;

        struct { unsigned int __reserved[4]; } __reserved_20;

} __attribute__ ((packed));






enum km_type {
__KM_FENCE_0 , KM_BOUNCE_READ,
__KM_FENCE_1 , KM_SKB_SUNRPC_DATA,
__KM_FENCE_2 , KM_SKB_DATA_SOFTIRQ,
__KM_FENCE_3 , KM_USER0,
__KM_FENCE_4 , KM_USER1,
__KM_FENCE_5 , KM_BIO_SRC_IRQ,
__KM_FENCE_6 , KM_BIO_DST_IRQ,
__KM_FENCE_7 , KM_PTE0,
__KM_FENCE_8 , KM_PTE1,
__KM_FENCE_9 , KM_IRQ0,
__KM_FENCE_10 , KM_IRQ1,
__KM_FENCE_11 , KM_SOFTIRQ0,
__KM_FENCE_12 , KM_SOFTIRQ1,
__KM_FENCE_13 , KM_TYPE_NR
};


enum fixed_addresses {
 FIX_HOLE,
 FIX_VDSO,

 FIX_KMAP_BEGIN,
 FIX_KMAP_END = FIX_KMAP_BEGIN+(KM_TYPE_NR*1)-1,


 FIX_ACPI_BEGIN,
 FIX_ACPI_END = FIX_ACPI_BEGIN + 4 - 1,


 FIX_PCIE_MCFG,

 __end_of_permanent_fixed_addresses,


 FIX_BTMAP_END = __end_of_permanent_fixed_addresses,
 FIX_BTMAP_BEGIN = FIX_BTMAP_END + 16 - 1,
 FIX_WP_TEST,
 __end_of_fixed_addresses
};

extern void __set_fixmap (enum fixed_addresses idx,
     unsigned long phys, pgprot_t flags);

extern void __this_fixmap_does_not_exist(void);






static inline __attribute__((always_inline)) __attribute__((always_inline)) unsigned long fix_to_virt(const unsigned int idx)
{

 if (idx >= __end_of_fixed_addresses)
  __this_fixmap_does_not_exist();

        return (((unsigned long)0xfffff000) - ((idx) << 12));
}

static inline __attribute__((always_inline)) unsigned long virt_to_fix(const unsigned long vaddr)
{
 do { if (__builtin_expect(!!((vaddr >= ((unsigned long)0xfffff000) || vaddr < (((unsigned long)0xfffff000) - (__end_of_permanent_fixed_addresses << 12)))!=0), 0)) __asm__ __volatile__( "ud2\n" "\t.word %c0\n" "\t.long %c1\n" : : "i" (144), "i" ("include/asm/fixmap.h")); } while(0);
 return ((((unsigned long)0xfffff000) - ((vaddr)&(~((1UL << 12)-1)))) >> 12);
}


struct mm_struct;
struct vm_area_struct;






extern unsigned long empty_zero_page[1024];
extern pgd_t swapper_pg_dir[1024];
extern kmem_cache_t *pgd_cache;
extern kmem_cache_t *pmd_cache;
extern spinlock_t pgd_lock;
extern struct page *pgd_list;

void pmd_ctor(void *, kmem_cache_t *, unsigned long);
void pgd_ctor(void *, kmem_cache_t *, unsigned long);
void pgd_dtor(void *, kmem_cache_t *, unsigned long);
void pgtable_cache_init(void);
void paging_init(void);




extern unsigned long long __PAGE_KERNEL, __PAGE_KERNEL_EXEC;

extern unsigned long pg0[];

static inline __attribute__((always_inline)) int pte_user(pte_t pte) { return (pte).pte_low & 0x004; }
static inline __attribute__((always_inline)) int pte_read(pte_t pte) { return (pte).pte_low & 0x004; }
static inline __attribute__((always_inline)) int pte_dirty(pte_t pte) { return (pte).pte_low & 0x040; }
static inline __attribute__((always_inline)) int pte_young(pte_t pte) { return (pte).pte_low & 0x020; }
static inline __attribute__((always_inline)) int pte_write(pte_t pte) { return (pte).pte_low & 0x002; }
static inline __attribute__((always_inline)) int pte_huge(pte_t pte) { return (pte).pte_low & 0x080; }




static inline __attribute__((always_inline)) int pte_file(pte_t pte) { return (pte).pte_low & 0x040; }

static inline __attribute__((always_inline)) pte_t pte_rdprotect(pte_t pte) { (pte).pte_low &= ~0x004; return pte; }
static inline __attribute__((always_inline)) pte_t pte_exprotect(pte_t pte) { (pte).pte_low &= ~0x004; return pte; }
static inline __attribute__((always_inline)) pte_t pte_mkclean(pte_t pte) { (pte).pte_low &= ~0x040; return pte; }
static inline __attribute__((always_inline)) pte_t pte_mkold(pte_t pte) { (pte).pte_low &= ~0x020; return pte; }
static inline __attribute__((always_inline)) pte_t pte_wrprotect(pte_t pte) { (pte).pte_low &= ~0x002; return pte; }
static inline __attribute__((always_inline)) pte_t pte_mkread(pte_t pte) { (pte).pte_low |= 0x004; return pte; }
static inline __attribute__((always_inline)) pte_t pte_mkexec(pte_t pte) { (pte).pte_low |= 0x004; return pte; }
static inline __attribute__((always_inline)) pte_t pte_mkdirty(pte_t pte) { (pte).pte_low |= 0x040; return pte; }
static inline __attribute__((always_inline)) pte_t pte_mkyoung(pte_t pte) { (pte).pte_low |= 0x020; return pte; }
static inline __attribute__((always_inline)) pte_t pte_mkwrite(pte_t pte) { (pte).pte_low |= 0x002; return pte; }
static inline __attribute__((always_inline)) pte_t pte_mkhuge(pte_t pte) { (pte).pte_low |= 0x080; return pte; }
















typedef struct { pgd_t pgd; } pud_t;

static inline __attribute__((always_inline)) int pgd_none(pgd_t pgd) { return 0; }
static inline __attribute__((always_inline)) int pgd_bad(pgd_t pgd) { return 0; }
static inline __attribute__((always_inline)) int pgd_present(pgd_t pgd) { return 1; }
static inline __attribute__((always_inline)) void pgd_clear(pgd_t *pgd) { }

static inline __attribute__((always_inline)) pud_t * pud_offset(pgd_t * pgd, unsigned long address)
{
 return (pud_t *)pgd;
}


typedef struct { pud_t pud; } pmd_t;

static inline __attribute__((always_inline)) int pud_none(pud_t pud) { return 0; }
static inline __attribute__((always_inline)) int pud_bad(pud_t pud) { return 0; }
static inline __attribute__((always_inline)) int pud_present(pud_t pud) { return 1; }
static inline __attribute__((always_inline)) void pud_clear(pud_t *pud) { }

static inline __attribute__((always_inline)) pmd_t * pmd_offset(pud_t * pud, unsigned long address)
{
 return (pmd_t *)pud;
}


static inline __attribute__((always_inline)) int pte_exec(pte_t pte)
{
 return pte_user(pte);
}




static inline __attribute__((always_inline)) int pte_exec_kernel(pte_t pte)
{
 return 1;
}

void vmalloc_sync_all(void);



static inline __attribute__((always_inline)) int ptep_test_and_clear_dirty(struct vm_area_struct *vma, unsigned long addr, pte_t *ptep)
{
 if (!pte_dirty(*ptep))
  return 0;
 return test_and_clear_bit(6, &ptep->pte_low);
}

static inline __attribute__((always_inline)) int ptep_test_and_clear_young(struct vm_area_struct *vma, unsigned long addr, pte_t *ptep)
{
 if (!pte_young(*ptep))
  return 0;
 return test_and_clear_bit(5, &ptep->pte_low);
}

static inline __attribute__((always_inline)) pte_t ptep_get_and_clear_full(struct mm_struct *mm, unsigned long addr, pte_t *ptep, int full)
{
 pte_t pte;
 if (full) {
  pte = *ptep;
  do { (*(ptep) = ((pte_t) { (0) } )); } while (0);
 } else {
  pte = ((pte_t) { (((__typeof__(*(&(ptep)->pte_low)))__xchg((unsigned long)(0),(&(ptep)->pte_low),sizeof(*(&(ptep)->pte_low))))) } );
 }
 return pte;
}

static inline __attribute__((always_inline)) void ptep_set_wrprotect(struct mm_struct *mm, unsigned long addr, pte_t *ptep)
{
 clear_bit(1, &ptep->pte_low);
}

static inline __attribute__((always_inline)) void clone_pgd_range(pgd_t *dst, pgd_t *src, int count)
{
       (__builtin_constant_p(count * sizeof(pgd_t)) ? __constant_memcpy((dst),(src),(count * sizeof(pgd_t))) : __memcpy((dst),(src),(count * sizeof(pgd_t))));
}

static inline __attribute__((always_inline)) pte_t pte_modify(pte_t pte, pgprot_t newprot)
{
 pte.pte_low &= ((~((1UL << 12)-1)) | 0x020 | 0x040);
 pte.pte_low |= ((newprot).pgprot);

 return pte;
}

extern pte_t *lookup_address(unsigned long address);

 static inline __attribute__((always_inline)) int set_kernel_exec(unsigned long vaddr, int enable) { return 0;}


extern void noexec_setup(const char *str);



void pgd_clear_bad(pgd_t *);
void pud_clear_bad(pud_t *);
void pmd_clear_bad(pmd_t *);

static inline __attribute__((always_inline)) int pgd_none_or_clear_bad(pgd_t *pgd)
{
 if (pgd_none(*pgd))
  return 1;
 if (__builtin_expect(!!(pgd_bad(*pgd)), 0)) {
  pgd_clear_bad(pgd);
  return 1;
 }
 return 0;
}

static inline __attribute__((always_inline)) int pud_none_or_clear_bad(pud_t *pud)
{
 if (pud_none(*pud))
  return 1;
 if (__builtin_expect(!!(pud_bad(*pud)), 0)) {
  pud_clear_bad(pud);
  return 1;
 }
 return 0;
}

static inline __attribute__((always_inline)) int pmd_none_or_clear_bad(pmd_t *pmd)
{
 if ((!(unsigned long)((((((*pmd).pud).pgd).pgd)))))
  return 1;
 if (__builtin_expect(!!(((((((((*pmd).pud).pgd).pgd))) & (~(~((1UL << 12)-1)) & ~0x004)) != (0x001 | 0x002 | 0x020 | 0x040))), 0)) {
  pmd_clear_bad(pmd);
  return 1;
 }
 return 0;
}



struct vm_area_struct {
 struct mm_struct * vm_mm;
 unsigned long vm_start;
 unsigned long vm_end;



 struct vm_area_struct *vm_next;

 pgprot_t vm_page_prot;
 unsigned long vm_flags;

 struct rb_node vm_rb;







 union {
  struct {
   struct list_head list;
   void *parent;
   struct vm_area_struct *head;
  } vm_set;

  struct raw_prio_tree_node prio_tree_node;
 } shared;







 struct list_head anon_vma_node;
 struct anon_vma *anon_vma;


 struct vm_operations_struct * vm_ops;


 unsigned long vm_pgoff;

 struct file * vm_file;
 void * vm_private_data;
 unsigned long vm_truncate_count;







};






struct vm_list_struct {
 struct vm_list_struct *next;
 struct vm_area_struct *vma;
};

extern pgprot_t protection_map[16];







struct vm_operations_struct {
 void (*open)(struct vm_area_struct * area);
 void (*close)(struct vm_area_struct * area);
 struct page * (*nopage)(struct vm_area_struct * area, unsigned long address, int *type);
 int (*populate)(struct vm_area_struct * area, unsigned long address, unsigned long len, pgprot_t prot, unsigned long pgoff, int nonblock);



 int (*page_mkwrite)(struct vm_area_struct *vma, struct page *page);







};

struct mmu_gather;
struct inode;







struct page {
 unsigned long flags;

 atomic_t _count;
 atomic_t _mapcount;



 union {
     struct {
  unsigned long private;






  struct address_space *mapping;






     };



 };
 unsigned long index;
 struct list_head lru;

};



struct page;

int test_clear_page_dirty(struct page *page);
int test_clear_page_writeback(struct page *page);
int test_set_page_writeback(struct page *page);

static inline __attribute__((always_inline)) void clear_page_dirty(struct page *page)
{
 test_clear_page_dirty(page);
}

static inline __attribute__((always_inline)) void set_page_writeback(struct page *page)
{
 test_set_page_writeback(page);
}


static inline __attribute__((always_inline)) int put_page_testzero(struct page *page)
{
 do { if (__builtin_expect(!!((((&page->_count)->counter) == 0)!=0), 0)) __asm__ __volatile__( "ud2\n" "\t.word %c0\n" "\t.long %c1\n" : : "i" (300), "i" ("include/linux/mm.h")); } while(0);
 return atomic_dec_and_test(&page->_count);
}





static inline __attribute__((always_inline)) int get_page_unless_zero(struct page *page)
{
 return ({ int c, old; c = (((&page->_count))->counter); for (;;) { if (__builtin_expect(!!(c == (0)), 0)) break; old = ((int)((__typeof__(*(&((((&page->_count)))->counter))))__cmpxchg((&((((&page->_count)))->counter)),(unsigned long)(c), (unsigned long)(c + (1)),sizeof(*(&((((&page->_count)))->counter)))))); if (__builtin_expect(!!(old == c), 1)) break; c = old; } c != (0); });
}

extern void __page_cache_release(struct page *) __attribute__((regparm(3)));

static inline __attribute__((always_inline)) int page_count(struct page *page)
{
 if (__builtin_expect(!!((__builtin_constant_p(14) ? constant_test_bit((14),(&(page)->flags)) : variable_test_bit((14),(&(page)->flags)))), 0))
  page = (struct page *)((page)->private);
 return ((&page->_count)->counter);
}

static inline __attribute__((always_inline)) void get_page(struct page *page)
{
 if (__builtin_expect(!!((__builtin_constant_p(14) ? constant_test_bit((14),(&(page)->flags)) : variable_test_bit((14),(&(page)->flags)))), 0))
  page = (struct page *)((page)->private);
 atomic_inc(&page->_count);
}





static inline __attribute__((always_inline)) void init_page_count(struct page *page)
{
 (((&page->_count)->counter) = (1));
}

void put_page(struct page *page);
void put_pages_list(struct list_head *pages);

void split_page(struct page *page, unsigned int order);

static inline __attribute__((always_inline)) unsigned long page_zonenum(struct page *page)
{
 return (page->flags >> (((((sizeof(unsigned long)*8) - 0) - 0) - 2) * (2 != 0))) & ((1UL << 2) - 1);
}

struct zone;
extern struct zone *zone_table[];

static inline __attribute__((always_inline)) int page_zone_id(struct page *page)
{
 return (page->flags >> (((((sizeof(unsigned long)*8) - 0) - 0) - 2) * (2 != 0))) & ((1UL << (0 + 2)) - 1);
}
static inline __attribute__((always_inline)) struct zone *page_zone(struct page *page)
{
 return zone_table[page_zone_id(page)];
}

static inline __attribute__((always_inline)) unsigned long page_to_nid(struct page *page)
{
 if ((0 > 0 || 0 == 0))
  return (page->flags >> ((((sizeof(unsigned long)*8) - 0) - 0) * (0 != 0))) & ((1UL << 0) - 1);
 else
  return page_zone(page)->zone_pgdat->node_id;
}
static inline __attribute__((always_inline)) unsigned long page_to_section(struct page *page)
{
 return (page->flags >> (((sizeof(unsigned long)*8) - 0) * (0 != 0))) & ((1UL << 0) - 1);
}

static inline __attribute__((always_inline)) void set_page_zone(struct page *page, unsigned long zone)
{
 page->flags &= ~(((1UL << 2) - 1) << (((((sizeof(unsigned long)*8) - 0) - 0) - 2) * (2 != 0)));
 page->flags |= (zone & ((1UL << 2) - 1)) << (((((sizeof(unsigned long)*8) - 0) - 0) - 2) * (2 != 0));
}
static inline __attribute__((always_inline)) void set_page_node(struct page *page, unsigned long node)
{
 page->flags &= ~(((1UL << 0) - 1) << ((((sizeof(unsigned long)*8) - 0) - 0) * (0 != 0)));
 page->flags |= (node & ((1UL << 0) - 1)) << ((((sizeof(unsigned long)*8) - 0) - 0) * (0 != 0));
}
static inline __attribute__((always_inline)) void set_page_section(struct page *page, unsigned long section)
{
 page->flags &= ~(((1UL << 0) - 1) << (((sizeof(unsigned long)*8) - 0) * (0 != 0)));
 page->flags |= (section & ((1UL << 0) - 1)) << (((sizeof(unsigned long)*8) - 0) * (0 != 0));
}

static inline __attribute__((always_inline)) void set_page_links(struct page *page, unsigned long zone,
 unsigned long node, unsigned long pfn)
{
 set_page_zone(page, zone);
 set_page_node(page, node);
 set_page_section(page, ((pfn) >> 0));
}






enum vm_event_item { PGPGIN, PGPGOUT, PSWPIN, PSWPOUT,
  PGALLOC_DMA, PGALLOC_DMA32, PGALLOC_NORMAL, PGALLOC_HIGH,
  PGFREE, PGACTIVATE, PGDEACTIVATE,
  PGFAULT, PGMAJFAULT,
  PGREFILL_DMA, PGREFILL_DMA32, PGREFILL_NORMAL, PGREFILL_HIGH,
  PGSTEAL_DMA, PGSTEAL_DMA32, PGSTEAL_NORMAL, PGSTEAL_HIGH,
  PGSCAN_KSWAPD_DMA, PGSCAN_KSWAPD_DMA32, PGSCAN_KSWAPD_NORMAL, PGSCAN_KSWAPD_HIGH,
  PGSCAN_DIRECT_DMA, PGSCAN_DIRECT_DMA32, PGSCAN_DIRECT_NORMAL, PGSCAN_DIRECT_HIGH,
  PGINODESTEAL, SLABS_SCANNED, KSWAPD_STEAL, KSWAPD_INODESTEAL,
  PAGEOUTRUN, ALLOCSTALL, PGROTATED,
  NR_VM_EVENT_ITEMS
};

struct vm_event_state {
 unsigned long event[NR_VM_EVENT_ITEMS];
};

extern __typeof__(struct vm_event_state) per_cpu__vm_event_states;

static inline __attribute__((always_inline)) void __count_vm_event(enum vm_event_item item)
{
 per_cpu__vm_event_states.event[item]++;
}

static inline __attribute__((always_inline)) void count_vm_event(enum vm_event_item item)
{
 (*({ do { } while (0); &per_cpu__vm_event_states; })).event[item]++;
 do { } while (0);
}

static inline __attribute__((always_inline)) void __count_vm_events(enum vm_event_item item, long delta)
{
 per_cpu__vm_event_states.event[item] += delta;
}

static inline __attribute__((always_inline)) void count_vm_events(enum vm_event_item item, long delta)
{
 (*({ do { } while (0); &per_cpu__vm_event_states; })).event[item] += delta;
 do { } while (0);
}

extern void all_vm_events(unsigned long *);
extern void vm_events_fold_cpu(int cpu);

extern atomic_long_t vm_stat[NR_VM_ZONE_STAT_ITEMS];

static inline __attribute__((always_inline)) void zone_page_state_add(long x, struct zone *zone,
     enum zone_stat_item item)
{
 atomic_long_add(x, &zone->vm_stat[item]);
 atomic_long_add(x, &vm_stat[item]);
}

static inline __attribute__((always_inline)) unsigned long global_page_state(enum zone_stat_item item)
{
 long x = atomic_long_read(&vm_stat[item]);




 return x;
}

static inline __attribute__((always_inline)) unsigned long zone_page_state(struct zone *zone,
     enum zone_stat_item item)
{
 long x = atomic_long_read(&zone->vm_stat[item]);




 return x;
}

static inline __attribute__((always_inline)) void zap_zone_vm_stats(struct zone *zone)
{
 (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(zone->vm_stat))) ? __constant_c_and_count_memset(((zone->vm_stat)),((0x01010101UL*(unsigned char)(0))),((sizeof(zone->vm_stat)))) : __constant_c_memset(((zone->vm_stat)),((0x01010101UL*(unsigned char)(0))),((sizeof(zone->vm_stat))))) : (__builtin_constant_p((sizeof(zone->vm_stat))) ? __memset_generic((((zone->vm_stat))),(((0))),(((sizeof(zone->vm_stat))))) : __memset_generic(((zone->vm_stat)),((0)),((sizeof(zone->vm_stat))))));
}

extern void inc_zone_state(struct zone *, enum zone_stat_item);

static inline __attribute__((always_inline)) void __mod_zone_page_state(struct zone *zone,
   enum zone_stat_item item, int delta)
{
 zone_page_state_add(delta, zone, item);
}

static inline __attribute__((always_inline)) void __inc_zone_state(struct zone *zone, enum zone_stat_item item)
{
 atomic_long_inc(&zone->vm_stat[item]);
 atomic_long_inc(&vm_stat[item]);
}

static inline __attribute__((always_inline)) void __inc_zone_page_state(struct page *page,
   enum zone_stat_item item)
{
 __inc_zone_state(page_zone(page), item);
}

static inline __attribute__((always_inline)) void __dec_zone_page_state(struct page *page,
   enum zone_stat_item item)
{
 atomic_long_dec(&page_zone(page)->vm_stat[item]);
 atomic_long_dec(&vm_stat[item]);
}

static inline __attribute__((always_inline)) void refresh_cpu_vm_stats(int cpu) { }
static inline __attribute__((always_inline)) void refresh_vm_stats(void) { }




extern struct page *mem_map;


static inline __attribute__((always_inline)) __attribute__((always_inline)) void *lowmem_page_address(struct page *page)
{
 return ((void *)((unsigned long)(((unsigned long)((page) - mem_map) + (0UL)) << 12)+((unsigned long)((unsigned long)0xC0000000))));
}

void *page_address(struct page *page);
void set_page_address(struct page *page, void *virtual);
void page_address_init(void);

extern struct address_space swapper_space;
static inline __attribute__((always_inline)) struct address_space *page_mapping(struct page *page)
{
 struct address_space *mapping = page->mapping;

 if (__builtin_expect(!!((__builtin_constant_p(15) ? constant_test_bit((15),(&(page)->flags)) : variable_test_bit((15),(&(page)->flags)))), 0))
  mapping = &swapper_space;
 else if (__builtin_expect(!!((unsigned long)mapping & 1), 0))
  mapping = ((void *)0);
 return mapping;
}

static inline __attribute__((always_inline)) int PageAnon(struct page *page)
{
 return ((unsigned long)page->mapping & 1) != 0;
}





static inline __attribute__((always_inline)) unsigned long page_index(struct page *page)
{
 if (__builtin_expect(!!((__builtin_constant_p(15) ? constant_test_bit((15),(&(page)->flags)) : variable_test_bit((15),(&(page)->flags)))), 0))
  return ((page)->private);
 return page->index;
}






static inline __attribute__((always_inline)) void reset_page_mapcount(struct page *page)
{
 (((&(page)->_mapcount)->counter) = (-1));
}

static inline __attribute__((always_inline)) int page_mapcount(struct page *page)
{
 return ((&(page)->_mapcount)->counter) + 1;
}




static inline __attribute__((always_inline)) int page_mapped(struct page *page)
{
 return ((&(page)->_mapcount)->counter) >= 0;
}

extern void show_free_areas(void);


struct page *shmem_nopage(struct vm_area_struct *vma,
   unsigned long address, int *type);
int shmem_set_policy(struct vm_area_struct *vma, struct mempolicy *new);
struct mempolicy *shmem_get_policy(struct vm_area_struct *vma,
     unsigned long addr);
int shmem_lock(struct file *file, int lock, struct user_struct *user);

struct file *shmem_file_setup(char *name, loff_t size, unsigned long flags);
extern int shmem_mmap(struct file *file, struct vm_area_struct *vma);

int shmem_zero_setup(struct vm_area_struct *);

static inline __attribute__((always_inline)) int can_do_mlock(void)
{
 if (capable(14))
  return 1;
 if (__vericon_dummy_current->signal->rlim[8].rlim_cur != 0)
  return 1;
 return 0;
}
extern int user_shm_lock(size_t, struct user_struct *);
extern void user_shm_unlock(size_t, struct user_struct *);




struct zap_details {
 struct vm_area_struct *nonlinear_vma;
 struct address_space *check_mapping;
 unsigned long first_index;
 unsigned long last_index;
 spinlock_t *i_mmap_lock;
 unsigned long truncate_count;
};

struct page *vm_normal_page(struct vm_area_struct *, unsigned long, pte_t);
unsigned long zap_page_range(struct vm_area_struct *vma, unsigned long address,
  unsigned long size, struct zap_details *);
unsigned long unmap_vmas(struct mmu_gather **tlb,
  struct vm_area_struct *start_vma, unsigned long start_addr,
  unsigned long end_addr, unsigned long *nr_accounted,
  struct zap_details *);
void free_pgd_range(struct mmu_gather **tlb, unsigned long addr,
  unsigned long end, unsigned long floor, unsigned long ceiling);
void free_pgtables(struct mmu_gather **tlb, struct vm_area_struct *start_vma,
  unsigned long floor, unsigned long ceiling);
int copy_page_range(struct mm_struct *dst, struct mm_struct *src,
   struct vm_area_struct *vma);
int zeromap_page_range(struct vm_area_struct *vma, unsigned long from,
   unsigned long size, pgprot_t prot);
void unmap_mapping_range(struct address_space *mapping,
  loff_t const holebegin, loff_t const holelen, int even_cows);

static inline __attribute__((always_inline)) void unmap_shared_mapping_range(struct address_space *mapping,
  loff_t const holebegin, loff_t const holelen)
{
 unmap_mapping_range(mapping, holebegin, holelen, 0);
}

extern int vmtruncate(struct inode * inode, loff_t offset);
extern int vmtruncate_range(struct inode * inode, loff_t offset, loff_t end);
extern int install_page(struct mm_struct *mm, struct vm_area_struct *vma, unsigned long addr, struct page *page, pgprot_t prot);
extern int install_file_pte(struct mm_struct *mm, struct vm_area_struct *vma, unsigned long addr, unsigned long pgoff, pgprot_t prot);


extern int __handle_mm_fault(struct mm_struct *mm,struct vm_area_struct *vma,
   unsigned long address, int write_access);

static inline __attribute__((always_inline)) int handle_mm_fault(struct mm_struct *mm,
   struct vm_area_struct *vma, unsigned long address,
   int write_access)
{
 return __handle_mm_fault(mm, vma, address, write_access) &
    (~0x10);
}

extern int make_pages_present(unsigned long addr, unsigned long end);
extern int access_process_vm(struct task_struct *tsk, unsigned long addr, void *buf, int len, int write);
void install_arg_page(struct vm_area_struct *, struct page *, unsigned long);

int get_user_pages(struct task_struct *tsk, struct mm_struct *mm, unsigned long start,
  int len, int write, int force, struct page **pages, struct vm_area_struct **vmas);
void print_bad_pte(struct vm_area_struct *, pte_t, unsigned long);

int __set_page_dirty_buffers(struct page *page);
int __set_page_dirty_nobuffers(struct page *page);
int redirty_page_for_writepage(struct writeback_control *wbc,
    struct page *page);
int set_page_dirty(struct page *page) __attribute__((regparm(3)));
int set_page_dirty_lock(struct page *page);
int clear_page_dirty_for_io(struct page *page);

extern unsigned long do_mremap(unsigned long addr,
          unsigned long old_len, unsigned long new_len,
          unsigned long flags, unsigned long new_addr);

typedef int (*shrinker_t)(int nr_to_scan, gfp_t gfp_mask);







struct shrinker;
extern struct shrinker *set_shrinker(int, shrinker_t);
extern void remove_shrinker(struct shrinker *shrinker);

extern pte_t *get_locked_pte(struct mm_struct *mm, unsigned long addr, spinlock_t **ptl) __attribute__((regparm(3)));

int __pud_alloc(struct mm_struct *mm, pgd_t *pgd, unsigned long address);
int __pmd_alloc(struct mm_struct *mm, pud_t *pud, unsigned long address);
int __pte_alloc(struct mm_struct *mm, pmd_t *pmd, unsigned long address);
int __pte_alloc_kernel(pmd_t *pmd, unsigned long address);






static inline __attribute__((always_inline)) pud_t *pud_alloc(struct mm_struct *mm, pgd_t *pgd, unsigned long address)
{
 return (__builtin_expect(!!(pgd_none(*pgd)), 0) && __pud_alloc(mm, pgd, address))?
  ((void *)0): pud_offset(pgd, address);
}

static inline __attribute__((always_inline)) pmd_t *pmd_alloc(struct mm_struct *mm, pud_t *pud, unsigned long address)
{
 return (__builtin_expect(!!(pud_none(*pud)), 0) && __pmd_alloc(mm, pud, address))?
  ((void *)0): pmd_offset(pud, address);
}

extern void free_area_init(unsigned long * zones_size);
extern void free_area_init_node(int nid, pg_data_t *pgdat,
 unsigned long * zones_size, unsigned long zone_start_pfn,
 unsigned long *zholes_size);
extern void memmap_init_zone(unsigned long, int, unsigned long, unsigned long);
extern void setup_per_zone_pages_min(void);
extern void mem_init(void);
extern void show_mem(void);
extern void si_meminfo(struct sysinfo * val);
extern void si_meminfo_node(struct sysinfo *val, int nid);




static inline __attribute__((always_inline)) void setup_per_cpu_pageset(void) {}



void vma_prio_tree_add(struct vm_area_struct *, struct vm_area_struct *old);
void vma_prio_tree_insert(struct vm_area_struct *, struct prio_tree_root *);
void vma_prio_tree_remove(struct vm_area_struct *, struct prio_tree_root *);
struct vm_area_struct *vma_prio_tree_next(struct vm_area_struct *vma,
 struct prio_tree_iter *iter);





static inline __attribute__((always_inline)) void vma_nonlinear_insert(struct vm_area_struct *vma,
     struct list_head *list)
{
 vma->shared.vm_set.parent = ((void *)0);
 list_add_tail(&vma->shared.vm_set.list, list);
}


extern int __vm_enough_memory(long pages, int cap_sys_admin);
extern void vma_adjust(struct vm_area_struct *vma, unsigned long start,
 unsigned long end, unsigned long pgoff, struct vm_area_struct *insert);
extern struct vm_area_struct *vma_merge(struct mm_struct *,
 struct vm_area_struct *prev, unsigned long addr, unsigned long end,
 unsigned long vm_flags, struct anon_vma *, struct file *, unsigned long,
 struct mempolicy *);
extern struct anon_vma *find_mergeable_anon_vma(struct vm_area_struct *);
extern int split_vma(struct mm_struct *,
 struct vm_area_struct *, unsigned long addr, int new_below);
extern int insert_vm_struct(struct mm_struct *, struct vm_area_struct *);
extern void __vma_link_rb(struct mm_struct *, struct vm_area_struct *,
 struct rb_node **, struct rb_node *);
extern void unlink_file_vma(struct vm_area_struct *);
extern struct vm_area_struct *copy_vma(struct vm_area_struct **,
 unsigned long addr, unsigned long len, unsigned long pgoff);
extern void exit_mmap(struct mm_struct *);
extern int may_expand_vm(struct mm_struct *mm, unsigned long npages);

extern unsigned long get_unmapped_area(struct file *, unsigned long, unsigned long, unsigned long, unsigned long);

extern unsigned long do_mmap_pgoff(struct file *file, unsigned long addr,
 unsigned long len, unsigned long prot,
 unsigned long flag, unsigned long pgoff);

static inline __attribute__((always_inline)) unsigned long do_mmap(struct file *file, unsigned long addr,
 unsigned long len, unsigned long prot,
 unsigned long flag, unsigned long offset)
{
 unsigned long ret = -22;
 if ((offset + (((len)+(1UL << 12)-1)&(~((1UL << 12)-1)))) < offset)
  goto out;
 if (!(offset & ~(~((1UL << 12)-1))))
  ret = do_mmap_pgoff(file, addr, len, prot, flag, offset >> 12);
out:
 return ret;
}

extern int do_munmap(struct mm_struct *, unsigned long, size_t);

extern unsigned long do_brk(unsigned long, unsigned long);


extern unsigned long page_unuse(struct page *);
extern void truncate_inode_pages(struct address_space *, loff_t);
extern void truncate_inode_pages_range(struct address_space *,
           loff_t lstart, loff_t lend);


extern struct page *filemap_nopage(struct vm_area_struct *, unsigned long, int *);
extern int filemap_populate(struct vm_area_struct *, unsigned long,
  unsigned long, pgprot_t, unsigned long, int);


int write_one_page(struct page *page, int wait);







int do_page_cache_readahead(struct address_space *mapping, struct file *filp,
   unsigned long offset, unsigned long nr_to_read);
int force_page_cache_readahead(struct address_space *mapping, struct file *filp,
   unsigned long offset, unsigned long nr_to_read);
unsigned long page_cache_readahead(struct address_space *mapping,
     struct file_ra_state *ra,
     struct file *filp,
     unsigned long offset,
     unsigned long size);
void handle_ra_miss(struct address_space *mapping,
      struct file_ra_state *ra, unsigned long offset);
unsigned long max_sane_readahead(unsigned long nr);


extern int expand_stack(struct vm_area_struct *vma, unsigned long address);





extern struct vm_area_struct * find_vma(struct mm_struct * mm, unsigned long addr);
extern struct vm_area_struct * find_vma_prev(struct mm_struct * mm, unsigned long addr,
          struct vm_area_struct **pprev);



static inline __attribute__((always_inline)) struct vm_area_struct * find_vma_intersection(struct mm_struct * mm, unsigned long start_addr, unsigned long end_addr)
{
 struct vm_area_struct * vma = find_vma(mm,start_addr);

 if (vma && end_addr <= vma->vm_start)
  vma = ((void *)0);
 return vma;
}

static inline __attribute__((always_inline)) unsigned long vma_pages(struct vm_area_struct *vma)
{
 return (vma->vm_end - vma->vm_start) >> 12;
}

struct vm_area_struct *find_extend_vma(struct mm_struct *, unsigned long addr);
struct page *vmalloc_to_page(void *addr);
unsigned long vmalloc_to_pfn(void *addr);
int remap_pfn_range(struct vm_area_struct *, unsigned long addr,
   unsigned long pfn, unsigned long size, pgprot_t);
int vm_insert_page(struct vm_area_struct *, unsigned long addr, struct page *);

struct page *follow_page(struct vm_area_struct *, unsigned long address,
   unsigned int foll_flags);






void vm_stat_account(struct mm_struct *, unsigned long, struct file *, long);

static inline __attribute__((always_inline)) void
kernel_map_pages(struct page *page, int numpages, int enable)
{
 if (!is_highmem(page_zone(page)) && !enable)
  debug_check_no_locks_freed(page_address(page),
        numpages * (1UL << 12));
}


extern struct vm_area_struct *get_gate_vma(struct task_struct *tsk);

int in_gate_area_no_task(unsigned long addr);
int in_gate_area(struct task_struct *task, unsigned long addr);

int drop_caches_sysctl_handler(struct ctl_table *, int, struct file *,
     void *, size_t *, loff_t *);
unsigned long shrink_slab(unsigned long scanned, gfp_t gfp_mask,
   unsigned long lru_pages);
void drop_pagecache(void);
void drop_slab(void);




extern int randomize_va_space;


const char *arch_vma_name(struct vm_area_struct *vma);












void global_flush_tlb(void);
int change_page_attr(struct page *page, int numpages, pgprot_t prot);







void mark_rodata_ro(void);



static inline __attribute__((always_inline)) void flush_anon_page(struct page *page, unsigned long vmaddr)
{
}



static inline __attribute__((always_inline)) void flush_kernel_dcache_page(struct page *page)
{
}










typedef int irqreturn_t;

























static __inline__ __attribute__((always_inline)) int irq_canonicalize(int irq)
{
 return ((irq == 2) ? 9 : irq);
}






  extern void irq_ctx_init(int cpu);
  extern void irq_ctx_exit(int cpu);


struct proc_dir_entry;

struct irq_chip {
 const char *name;
 unsigned int (*startup)(unsigned int irq);
 void (*shutdown)(unsigned int irq);
 void (*enable)(unsigned int irq);
 void (*disable)(unsigned int irq);

 void (*ack)(unsigned int irq);
 void (*mask)(unsigned int irq);
 void (*mask_ack)(unsigned int irq);
 void (*unmask)(unsigned int irq);
 void (*eoi)(unsigned int irq);

 void (*end)(unsigned int irq);
 void (*set_affinity)(unsigned int irq, cpumask_t dest);
 int (*retrigger)(unsigned int irq);
 int (*set_type)(unsigned int irq, unsigned int flow_type);
 int (*set_wake)(unsigned int irq, unsigned int on);

 const char *typename;
};

struct irq_desc {
 void __attribute__((regparm(3))) (*handle_irq)(unsigned int irq,
           struct irq_desc *desc,
           struct pt_regs *regs);
 struct irq_chip *chip;
 void *handler_data;
 void *chip_data;
 struct irqaction *action;
 unsigned int status;

 unsigned int depth;
 unsigned int wake_depth;
 unsigned int irq_count;
 unsigned int irqs_unhandled;
 spinlock_t lock;

 struct proc_dir_entry *dir;

} __attribute__((__aligned__((1 << (7)))));

extern struct irq_desc irq_desc[16];





typedef struct irq_chip hw_irq_controller;

typedef struct irq_desc irq_desc_t;








struct proc_dir_entry;
struct pt_regs;
struct notifier_block;


void __attribute__ ((__section__ (".init.text"))) profile_init(void);
void profile_tick(int, struct pt_regs *);
void profile_hit(int, void *);

void create_prof_cpu_mask(struct proc_dir_entry *);




enum profile_type {
 PROFILE_TASK_EXIT,
 PROFILE_MUNMAP
};



struct task_struct;
struct mm_struct;


void profile_task_exit(struct task_struct * task);




int profile_handoff_task(struct task_struct * task);


void profile_munmap(unsigned long addr);

int task_handoff_register(struct notifier_block * n);
int task_handoff_unregister(struct notifier_block * n);

int profile_event_register(enum profile_type, struct notifier_block * n);
int profile_event_unregister(enum profile_type, struct notifier_block * n);

int register_timer_hook(int (*hook)(struct pt_regs *));
void unregister_timer_hook(int (*hook)(struct pt_regs *));


extern int (*timer_hook)(struct pt_regs *);

struct pt_regs;














extern char _text[], _stext[], _etext[];
extern char _data[], _sdata[], _edata[];
extern char __bss_start[], __bss_stop[];
extern char __init_begin[], __init_end[];
extern char _sinittext[], _einittext[];
extern char _sextratext[] __attribute__((weak));
extern char _eextratext[] __attribute__((weak));
extern char _end[];
extern char __per_cpu_start[], __per_cpu_end[];
extern char __kprobes_text_start[], __kprobes_text_end[];
extern char __initdata_begin[], __initdata_end[];
extern char __start_rodata[], __end_rodata[];



struct irq_chip;

extern u8 irq_vector[16];



extern void (*interrupt[16])(void);

void disable_8259A_irq(unsigned int irq);
void enable_8259A_irq(unsigned int irq);
int i8259A_irq_pending(unsigned int irq);
void make_8259A_irq(unsigned int irq);
void init_8259A(int aeoi);
void send_IPI_self(int vector) __attribute__((regparm(3)));
void init_VISWS_APIC_irqs(void);
void setup_IO_APIC(void);
void disable_IO_APIC(void);
void print_IO_APIC(void);
int IO_APIC_get_PCI_irq_vector(int bus, int slot, int fn);
void send_IPI(int dest, int vector);
void setup_ioapic_dest(void);

extern unsigned long io_apic_irqs;

extern atomic_t irq_err_count;
extern atomic_t irq_mis_count;


extern int setup_irq(unsigned int irq, struct irqaction *new);

static inline __attribute__((always_inline)) void set_native_irq_info(int irq, cpumask_t mask)
{
}

static inline __attribute__((always_inline)) void set_balance_irq_affinity(unsigned int irq, cpumask_t mask)
{
}





static inline __attribute__((always_inline)) int select_smp_affinity(unsigned int irq)
{
 return 1;
}


extern int no_irq_affinity;


extern int handle_IRQ_event(unsigned int irq, struct pt_regs *regs,
       struct irqaction *action);





extern void __attribute__((regparm(3)))
handle_level_irq(unsigned int irq, struct irq_desc *desc, struct pt_regs *regs);
extern void __attribute__((regparm(3)))
handle_fasteoi_irq(unsigned int irq, struct irq_desc *desc,
    struct pt_regs *regs);
extern void __attribute__((regparm(3)))
handle_edge_irq(unsigned int irq, struct irq_desc *desc, struct pt_regs *regs);
extern void __attribute__((regparm(3)))
handle_simple_irq(unsigned int irq, struct irq_desc *desc,
    struct pt_regs *regs);
extern void __attribute__((regparm(3)))
handle_percpu_irq(unsigned int irq, struct irq_desc *desc,
    struct pt_regs *regs);
extern void __attribute__((regparm(3)))
handle_bad_irq(unsigned int irq, struct irq_desc *desc, struct pt_regs *regs);





extern const char *
handle_irq_name(void __attribute__((regparm(3))) (*handle)(unsigned int, struct irq_desc *,
     struct pt_regs *));





extern __attribute__((regparm(3))) unsigned int __do_IRQ(unsigned int irq, struct pt_regs *regs);







static inline __attribute__((always_inline)) void generic_handle_irq(unsigned int irq, struct pt_regs *regs)
{
 struct irq_desc *desc = irq_desc + irq;

 if (__builtin_expect(!!(desc->handle_irq), 1))
  desc->handle_irq(irq, desc, regs);
 else
  __do_IRQ(irq, regs);
}


extern void note_interrupt(unsigned int irq, struct irq_desc *desc,
      int action_ret, struct pt_regs *regs);


void check_irq_resend(struct irq_desc *desc, unsigned int irq);


extern void init_irq_proc(void);


extern int noirqdebug_setup(char *str);


extern int can_request_irq(unsigned int irq, unsigned long irqflags);


extern struct irq_chip no_irq_chip;
extern struct irq_chip dummy_irq_chip;

extern void
set_irq_chip_and_handler(unsigned int irq, struct irq_chip *chip,
    void __attribute__((regparm(3))) (*handle)(unsigned int,
       struct irq_desc *,
       struct pt_regs *));
extern void
__set_irq_handler(unsigned int irq,
    void __attribute__((regparm(3))) (*handle)(unsigned int, struct irq_desc *,
       struct pt_regs *),
    int is_chained);




static inline __attribute__((always_inline)) void
set_irq_handler(unsigned int irq,
  void __attribute__((regparm(3))) (*handle)(unsigned int, struct irq_desc *,
     struct pt_regs *))
{
 __set_irq_handler(irq, handle, 0);
}






static inline __attribute__((always_inline)) void
set_irq_chained_handler(unsigned int irq,
   void __attribute__((regparm(3))) (*handle)(unsigned int, struct irq_desc *,
      struct pt_regs *))
{
 __set_irq_handler(irq, handle, 1);
}



extern int set_irq_chip(unsigned int irq, struct irq_chip *chip);
extern int set_irq_data(unsigned int irq, void *data);
extern int set_irq_chip_data(unsigned int irq, void *data);
extern int set_irq_type(unsigned int irq, unsigned int type);


typedef struct {
 unsigned int __softirq_pending;
 unsigned long idle_timestamp;
 unsigned int __nmi_count;
 unsigned int apic_timer_irqs;
} __attribute__((__aligned__((1 << (7))))) irq_cpustat_t;

extern __typeof__(irq_cpustat_t) per_cpu__irq_stat;
extern irq_cpustat_t irq_stat[];




void ack_bad_irq(unsigned int irq);




struct task_struct;


static inline __attribute__((always_inline)) void account_system_vtime(struct task_struct *tsk)
{
}

extern void irq_exit(void);


struct irqaction {
 irqreturn_t (*handler)(int, void *, struct pt_regs *);
 unsigned long flags;
 cpumask_t mask;
 const char *name;
 void *dev_id;
 struct irqaction *next;
 int irq;
 struct proc_dir_entry *dir;
};

extern irqreturn_t no_action(int cpl, void *dev_id, struct pt_regs *regs);
extern int request_irq(unsigned int,
         irqreturn_t (*handler)(int, void *, struct pt_regs *),
         unsigned long, const char *, void *);
extern void free_irq(unsigned int, void *);

extern void disable_irq_nosync(unsigned int irq);
extern void disable_irq(unsigned int irq);
extern void enable_irq(unsigned int irq);

static inline __attribute__((always_inline)) void disable_irq_nosync_lockdep(unsigned int irq)
{
 disable_irq_nosync(irq);



}

static inline __attribute__((always_inline)) void disable_irq_lockdep(unsigned int irq)
{
 disable_irq(irq);



}

static inline __attribute__((always_inline)) void enable_irq_lockdep(unsigned int irq)
{



 enable_irq(irq);
}


extern int set_irq_wake(unsigned int irq, unsigned int on);

static inline __attribute__((always_inline)) int enable_irq_wake(unsigned int irq)
{
 return set_irq_wake(irq, 1);
}

static inline __attribute__((always_inline)) int disable_irq_wake(unsigned int irq)
{
 return set_irq_wake(irq, 0);
}

static inline __attribute__((always_inline)) void __attribute__((deprecated)) cli(void)
{
 do { raw_local_irq_disable(); do { } while (0); } while (0);
}
static inline __attribute__((always_inline)) void __attribute__((deprecated)) sti(void)
{
 do { do { } while (0); raw_local_irq_enable(); } while (0);
}
static inline __attribute__((always_inline)) void __attribute__((deprecated)) save_flags(unsigned long *x)
{
 do { (*x) = __raw_local_save_flags(); } while (0);
}

static inline __attribute__((always_inline)) void __attribute__((deprecated)) restore_flags(unsigned long x)
{
 do { if (raw_irqs_disabled_flags(x)) { raw_local_irq_restore(x); do { } while (0); } else { do { } while (0); raw_local_irq_restore(x); } } while (0);
}

static inline __attribute__((always_inline)) void __attribute__((deprecated)) save_and_cli(unsigned long *x)
{
 do { do { (*x) = __raw_local_irq_save(); } while (0); do { } while (0); } while (0);
}



extern void local_bh_disable(void);
extern void __local_bh_enable(void);
extern void _local_bh_enable(void);
extern void local_bh_enable(void);
extern void local_bh_enable_ip(unsigned long ip);







enum
{
 HI_SOFTIRQ=0,
 TIMER_SOFTIRQ,
 NET_TX_SOFTIRQ,
 NET_RX_SOFTIRQ,
 BLOCK_SOFTIRQ,
 TASKLET_SOFTIRQ
};





struct softirq_action
{
 void (*action)(struct softirq_action *);
 void *data;
};

 __attribute__((regparm(0))) void do_softirq(void);
extern void open_softirq(int nr, void (*action)(struct softirq_action*), void *data);
extern void softirq_init(void);

extern void raise_softirq_irqoff(unsigned int nr) __attribute__((regparm(3)));
extern void raise_softirq(unsigned int nr) __attribute__((regparm(3)));

struct tasklet_struct
{
 struct tasklet_struct *next;
 unsigned long state;
 atomic_t count;
 void (*func)(unsigned long);
 unsigned long data;
};

enum
{
 TASKLET_STATE_SCHED,
 TASKLET_STATE_RUN
};

extern void __tasklet_schedule(struct tasklet_struct *t) __attribute__((regparm(3)));

static inline __attribute__((always_inline)) void tasklet_schedule(struct tasklet_struct *t)
{
 if (!test_and_set_bit(TASKLET_STATE_SCHED, &t->state))
  __tasklet_schedule(t);
}

extern void __tasklet_hi_schedule(struct tasklet_struct *t) __attribute__((regparm(3)));

static inline __attribute__((always_inline)) void tasklet_hi_schedule(struct tasklet_struct *t)
{
 if (!test_and_set_bit(TASKLET_STATE_SCHED, &t->state))
  __tasklet_hi_schedule(t);
}


static inline __attribute__((always_inline)) void tasklet_disable_nosync(struct tasklet_struct *t)
{
 atomic_inc(&t->count);
 __asm__ __volatile__("": : :"memory");
}

static inline __attribute__((always_inline)) void tasklet_disable(struct tasklet_struct *t)
{
 tasklet_disable_nosync(t);
 do { } while (0);
 __asm__ __volatile__("": : :"memory");
}

static inline __attribute__((always_inline)) void tasklet_enable(struct tasklet_struct *t)
{
 __asm__ __volatile__("": : :"memory");
 atomic_dec(&t->count);
}

static inline __attribute__((always_inline)) void tasklet_hi_enable(struct tasklet_struct *t)
{
 __asm__ __volatile__("": : :"memory");
 atomic_dec(&t->count);
}

extern void tasklet_kill(struct tasklet_struct *t);
extern void tasklet_kill_immediate(struct tasklet_struct *t, unsigned int cpu);
extern void tasklet_init(struct tasklet_struct *t,
    void (*func)(unsigned long), unsigned long data);

extern unsigned long probe_irq_on(void);
extern int probe_irq_off(unsigned long);
extern unsigned int probe_irq_mask(unsigned long);





extern unsigned long pgkern_mask;

static inline __attribute__((always_inline)) void flush_tlb_mm(struct mm_struct *mm)
{
 if (mm == __vericon_dummy_current->active_mm)
  do { unsigned int tmpreg; __asm__ __volatile__( "movl %%cr3, %0;              \n" "movl %0, %%cr3;  # flush TLB \n" : "=r" (tmpreg) :: "memory"); } while (0);
}

static inline __attribute__((always_inline)) void flush_tlb_page(struct vm_area_struct *vma,
 unsigned long addr)
{
 if (vma->vm_mm == __vericon_dummy_current->active_mm)
  __asm__ __volatile__("invlpg %0": :"m" (*(char *) addr));
}

static inline __attribute__((always_inline)) void flush_tlb_range(struct vm_area_struct *vma,
 unsigned long start, unsigned long end)
{
 if (vma->vm_mm == __vericon_dummy_current->active_mm)
  do { unsigned int tmpreg; __asm__ __volatile__( "movl %%cr3, %0;              \n" "movl %0, %%cr3;  # flush TLB \n" : "=r" (tmpreg) :: "memory"); } while (0);
}

static inline __attribute__((always_inline)) void flush_tlb_pgtables(struct mm_struct *mm,
          unsigned long start, unsigned long end)
{

}



extern unsigned long highstart_pfn, highend_pfn;

extern pte_t *kmap_pte;
extern pgprot_t kmap_prot;
extern pte_t *pkmap_page_table;

extern void * kmap_high(struct page *page) __attribute__((regparm(3)));
extern void kunmap_high(struct page *page) __attribute__((regparm(3)));

void *kmap(struct page *page);
void kunmap(struct page *page);
void *kmap_atomic(struct page *page, enum km_type type);
void kunmap_atomic(void *kvaddr, enum km_type type);
void *kmap_atomic_pfn(unsigned long pfn, enum km_type type);
struct page *kmap_atomic_to_page(void *ptr);



unsigned int nr_free_highpages(void);

static inline __attribute__((always_inline)) void clear_user_highpage(struct page *page, unsigned long vaddr)
{
 void *addr = kmap_atomic(page, KM_USER0);
 (__builtin_constant_p(0) ? (__builtin_constant_p(((1UL << 12))) ? __constant_c_and_count_memset((((void *)(addr))),((0x01010101UL*(unsigned char)(0))),(((1UL << 12)))) : __constant_c_memset((((void *)(addr))),((0x01010101UL*(unsigned char)(0))),(((1UL << 12))))) : (__builtin_constant_p(((1UL << 12))) ? __memset_generic(((((void *)(addr)))),(((0))),((((1UL << 12))))) : __memset_generic((((void *)(addr))),((0)),(((1UL << 12))))));
 kunmap_atomic(addr, KM_USER0);

 __asm__ __volatile__("": : :"memory");
}

static inline __attribute__((always_inline)) void clear_highpage(struct page *page)
{
 void *kaddr = kmap_atomic(page, KM_USER0);
 (__builtin_constant_p(0) ? (__builtin_constant_p(((1UL << 12))) ? __constant_c_and_count_memset((((void *)(kaddr))),((0x01010101UL*(unsigned char)(0))),(((1UL << 12)))) : __constant_c_memset((((void *)(kaddr))),((0x01010101UL*(unsigned char)(0))),(((1UL << 12))))) : (__builtin_constant_p(((1UL << 12))) ? __memset_generic(((((void *)(kaddr)))),(((0))),((((1UL << 12))))) : __memset_generic((((void *)(kaddr))),((0)),(((1UL << 12))))));
 kunmap_atomic(kaddr, KM_USER0);
}




static inline __attribute__((always_inline)) void memclear_highpage_flush(struct page *page, unsigned int offset, unsigned int size)
{
 void *kaddr;

 do { if (__builtin_expect(!!((offset + size > (1UL << 12))!=0), 0)) __asm__ __volatile__( "ud2\n" "\t.word %c0\n" "\t.long %c1\n" : : "i" (84), "i" ("include/linux/highmem.h")); } while(0);

 kaddr = kmap_atomic(page, KM_USER0);
 (__builtin_constant_p(0) ? (__builtin_constant_p((size)) ? __constant_c_and_count_memset((((char *)kaddr + offset)),((0x01010101UL*(unsigned char)(0))),((size))) : __constant_c_memset((((char *)kaddr + offset)),((0x01010101UL*(unsigned char)(0))),((size)))) : (__builtin_constant_p((size)) ? __memset_generic(((((char *)kaddr + offset))),(((0))),(((size)))) : __memset_generic((((char *)kaddr + offset)),((0)),((size)))));
 do { } while (0);
 kunmap_atomic(kaddr, KM_USER0);
}

static inline __attribute__((always_inline)) void copy_user_highpage(struct page *to, struct page *from, unsigned long vaddr)
{
 char *vfrom, *vto;

 vfrom = kmap_atomic(from, KM_USER0);
 vto = kmap_atomic(to, KM_USER1);
 (__builtin_constant_p((1UL << 12)) ? __constant_memcpy(((void *)(vto)),((void *)(vfrom)),((1UL << 12))) : __memcpy(((void *)(vto)),((void *)(vfrom)),((1UL << 12))));
 kunmap_atomic(vfrom, KM_USER0);
 kunmap_atomic(vto, KM_USER1);

 __asm__ __volatile__("": : :"memory");
}

static inline __attribute__((always_inline)) void copy_highpage(struct page *to, struct page *from)
{
 char *vfrom, *vto;

 vfrom = kmap_atomic(from, KM_USER0);
 vto = kmap_atomic(to, KM_USER1);
 (__builtin_constant_p((1UL << 12)) ? __constant_memcpy(((void *)(vto)),((void *)(vfrom)),((1UL << 12))) : __memcpy(((void *)(vto)),((void *)(vfrom)),((1UL << 12))));
 kunmap_atomic(vfrom, KM_USER0);
 kunmap_atomic(vto, KM_USER1);
}




extern struct movsl_mask {
 int mask;
} movsl_mask;

struct exception_table_entry
{
 unsigned long insn, fixup;
};

extern int fixup_exception(struct pt_regs *regs);

extern void __get_user_1(void);
extern void __get_user_2(void);
extern void __get_user_4(void);

extern void __put_user_bad(void);





extern void __put_user_1(void);
extern void __put_user_2(void);
extern void __put_user_4(void);
extern void __put_user_8(void);

struct __large_struct { unsigned long buf[100]; };

extern long __get_user_bad(void);

unsigned long __attribute__((warn_unused_result)) __copy_to_user_ll(void *to,
    const void *from, unsigned long n);
unsigned long __attribute__((warn_unused_result)) __copy_from_user_ll(void *to,
    const void *from, unsigned long n);
unsigned long __attribute__((warn_unused_result)) __copy_from_user_ll_nozero(void *to,
    const void *from, unsigned long n);
unsigned long __attribute__((warn_unused_result)) __copy_from_user_ll_nocache(void *to,
    const void *from, unsigned long n);
unsigned long __attribute__((warn_unused_result)) __copy_from_user_ll_nocache_nozero(void *to,
    const void *from, unsigned long n);

static inline __attribute__((always_inline)) __attribute__((always_inline)) unsigned long __attribute__((warn_unused_result))
__copy_to_user_inatomic(void *to, const void *from, unsigned long n)
{
 if (__builtin_constant_p(n)) {
  unsigned long ret;

  switch (n) {
  case 1:
   do { ret = 0; (void)0; switch (1) { case 1: __asm__ __volatile__( "1:	mov""b"" %""b""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret) : "iq" (*(u8 *)from), "m"((*(struct __large_struct *)((u8 *)to))), "i"(1), "0"(ret));break; case 2: __asm__ __volatile__( "1:	mov""w"" %""w""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret) : "ir" (*(u8 *)from), "m"((*(struct __large_struct *)((u8 *)to))), "i"(1), "0"(ret));break; case 4: __asm__ __volatile__( "1:	mov""l"" %""""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret) : "ir" (*(u8 *)from), "m"((*(struct __large_struct *)((u8 *)to))), "i"(1), "0"(ret)); break; case 8: __asm__ __volatile__( "1:	movl %%eax,0(%2)\n" "2:	movl %%edx,4(%2)\n" "3:\n" ".section .fixup,\"ax\"\n" "4:	movl %3,%0\n" "	jmp 3b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,4b\n" "	.long 2b,4b\n" ".previous" : "=r"(ret) : "A" ((__typeof__(*(u8 *)to))(*(u8 *)from)), "r" ((u8 *)to), "i"(-14), "0"(ret)); break; default: __put_user_bad(); } } while (0);
   return ret;
  case 2:
   do { ret = 0; (void)0; switch (2) { case 1: __asm__ __volatile__( "1:	mov""b"" %""b""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret) : "iq" (*(u16 *)from), "m"((*(struct __large_struct *)((u16 *)to))), "i"(2), "0"(ret));break; case 2: __asm__ __volatile__( "1:	mov""w"" %""w""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret) : "ir" (*(u16 *)from), "m"((*(struct __large_struct *)((u16 *)to))), "i"(2), "0"(ret));break; case 4: __asm__ __volatile__( "1:	mov""l"" %""""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret) : "ir" (*(u16 *)from), "m"((*(struct __large_struct *)((u16 *)to))), "i"(2), "0"(ret)); break; case 8: __asm__ __volatile__( "1:	movl %%eax,0(%2)\n" "2:	movl %%edx,4(%2)\n" "3:\n" ".section .fixup,\"ax\"\n" "4:	movl %3,%0\n" "	jmp 3b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,4b\n" "	.long 2b,4b\n" ".previous" : "=r"(ret) : "A" ((__typeof__(*(u16 *)to))(*(u16 *)from)), "r" ((u16 *)to), "i"(-14), "0"(ret)); break; default: __put_user_bad(); } } while (0);
   return ret;
  case 4:
   do { ret = 0; (void)0; switch (4) { case 1: __asm__ __volatile__( "1:	mov""b"" %""b""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret) : "iq" (*(u32 *)from), "m"((*(struct __large_struct *)((u32 *)to))), "i"(4), "0"(ret));break; case 2: __asm__ __volatile__( "1:	mov""w"" %""w""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret) : "ir" (*(u32 *)from), "m"((*(struct __large_struct *)((u32 *)to))), "i"(4), "0"(ret));break; case 4: __asm__ __volatile__( "1:	mov""l"" %""""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret) : "ir" (*(u32 *)from), "m"((*(struct __large_struct *)((u32 *)to))), "i"(4), "0"(ret)); break; case 8: __asm__ __volatile__( "1:	movl %%eax,0(%2)\n" "2:	movl %%edx,4(%2)\n" "3:\n" ".section .fixup,\"ax\"\n" "4:	movl %3,%0\n" "	jmp 3b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,4b\n" "	.long 2b,4b\n" ".previous" : "=r"(ret) : "A" ((__typeof__(*(u32 *)to))(*(u32 *)from)), "r" ((u32 *)to), "i"(-14), "0"(ret)); break; default: __put_user_bad(); } } while (0);
   return ret;
  }
 }
 return __copy_to_user_ll(to, from, n);
}

static inline __attribute__((always_inline)) __attribute__((always_inline)) unsigned long __attribute__((warn_unused_result))
__copy_to_user(void *to, const void *from, unsigned long n)
{
       do { __might_sleep("include/asm/uaccess.h", 445); cond_resched(); } while (0);
       return __copy_to_user_inatomic(to, from, n);
}

static inline __attribute__((always_inline)) __attribute__((always_inline)) unsigned long
__copy_from_user_inatomic(void *to, const void *from, unsigned long n)
{





 if (__builtin_constant_p(n)) {
  unsigned long ret;

  switch (n) {
  case 1:
   do { ret = 0; (void)0; switch (1) { case 1: __asm__ __volatile__( "1:	mov""b"" %2,%""b""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=q" (*(u8 *)to) : "m"((*(struct __large_struct *)(from))), "i"(1), "0"(ret));break; case 2: __asm__ __volatile__( "1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u8 *)to) : "m"((*(struct __large_struct *)(from))), "i"(1), "0"(ret));break; case 4: __asm__ __volatile__( "1:	mov""l"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""l"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u8 *)to) : "m"((*(struct __large_struct *)(from))), "i"(1), "0"(ret));break; default: (*(u8 *)to) = __get_user_bad(); } } while (0);
   return ret;
  case 2:
   do { ret = 0; (void)0; switch (2) { case 1: __asm__ __volatile__( "1:	mov""b"" %2,%""b""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=q" (*(u16 *)to) : "m"((*(struct __large_struct *)(from))), "i"(2), "0"(ret));break; case 2: __asm__ __volatile__( "1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u16 *)to) : "m"((*(struct __large_struct *)(from))), "i"(2), "0"(ret));break; case 4: __asm__ __volatile__( "1:	mov""l"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""l"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u16 *)to) : "m"((*(struct __large_struct *)(from))), "i"(2), "0"(ret));break; default: (*(u16 *)to) = __get_user_bad(); } } while (0);
   return ret;
  case 4:
   do { ret = 0; (void)0; switch (4) { case 1: __asm__ __volatile__( "1:	mov""b"" %2,%""b""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=q" (*(u32 *)to) : "m"((*(struct __large_struct *)(from))), "i"(4), "0"(ret));break; case 2: __asm__ __volatile__( "1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u32 *)to) : "m"((*(struct __large_struct *)(from))), "i"(4), "0"(ret));break; case 4: __asm__ __volatile__( "1:	mov""l"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""l"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u32 *)to) : "m"((*(struct __large_struct *)(from))), "i"(4), "0"(ret));break; default: (*(u32 *)to) = __get_user_bad(); } } while (0);
   return ret;
  }
 }
 return __copy_from_user_ll_nozero(to, from, n);
}
static inline __attribute__((always_inline)) __attribute__((always_inline)) unsigned long
__copy_from_user(void *to, const void *from, unsigned long n)
{
 do { __might_sleep("include/asm/uaccess.h", 499); cond_resched(); } while (0);
 if (__builtin_constant_p(n)) {
  unsigned long ret;

  switch (n) {
  case 1:
   do { ret = 0; (void)0; switch (1) { case 1: __asm__ __volatile__( "1:	mov""b"" %2,%""b""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=q" (*(u8 *)to) : "m"((*(struct __large_struct *)(from))), "i"(1), "0"(ret));break; case 2: __asm__ __volatile__( "1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u8 *)to) : "m"((*(struct __large_struct *)(from))), "i"(1), "0"(ret));break; case 4: __asm__ __volatile__( "1:	mov""l"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""l"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u8 *)to) : "m"((*(struct __large_struct *)(from))), "i"(1), "0"(ret));break; default: (*(u8 *)to) = __get_user_bad(); } } while (0);
   return ret;
  case 2:
   do { ret = 0; (void)0; switch (2) { case 1: __asm__ __volatile__( "1:	mov""b"" %2,%""b""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=q" (*(u16 *)to) : "m"((*(struct __large_struct *)(from))), "i"(2), "0"(ret));break; case 2: __asm__ __volatile__( "1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u16 *)to) : "m"((*(struct __large_struct *)(from))), "i"(2), "0"(ret));break; case 4: __asm__ __volatile__( "1:	mov""l"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""l"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u16 *)to) : "m"((*(struct __large_struct *)(from))), "i"(2), "0"(ret));break; default: (*(u16 *)to) = __get_user_bad(); } } while (0);
   return ret;
  case 4:
   do { ret = 0; (void)0; switch (4) { case 1: __asm__ __volatile__( "1:	mov""b"" %2,%""b""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=q" (*(u32 *)to) : "m"((*(struct __large_struct *)(from))), "i"(4), "0"(ret));break; case 2: __asm__ __volatile__( "1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u32 *)to) : "m"((*(struct __large_struct *)(from))), "i"(4), "0"(ret));break; case 4: __asm__ __volatile__( "1:	mov""l"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""l"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u32 *)to) : "m"((*(struct __large_struct *)(from))), "i"(4), "0"(ret));break; default: (*(u32 *)to) = __get_user_bad(); } } while (0);
   return ret;
  }
 }
 return __copy_from_user_ll(to, from, n);
}



static inline __attribute__((always_inline)) __attribute__((always_inline)) unsigned long __copy_from_user_nocache(void *to,
    const void *from, unsigned long n)
{
 do { __might_sleep("include/asm/uaccess.h", 523); cond_resched(); } while (0);
 if (__builtin_constant_p(n)) {
  unsigned long ret;

  switch (n) {
  case 1:
   do { ret = 0; (void)0; switch (1) { case 1: __asm__ __volatile__( "1:	mov""b"" %2,%""b""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=q" (*(u8 *)to) : "m"((*(struct __large_struct *)(from))), "i"(1), "0"(ret));break; case 2: __asm__ __volatile__( "1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u8 *)to) : "m"((*(struct __large_struct *)(from))), "i"(1), "0"(ret));break; case 4: __asm__ __volatile__( "1:	mov""l"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""l"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u8 *)to) : "m"((*(struct __large_struct *)(from))), "i"(1), "0"(ret));break; default: (*(u8 *)to) = __get_user_bad(); } } while (0);
   return ret;
  case 2:
   do { ret = 0; (void)0; switch (2) { case 1: __asm__ __volatile__( "1:	mov""b"" %2,%""b""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=q" (*(u16 *)to) : "m"((*(struct __large_struct *)(from))), "i"(2), "0"(ret));break; case 2: __asm__ __volatile__( "1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u16 *)to) : "m"((*(struct __large_struct *)(from))), "i"(2), "0"(ret));break; case 4: __asm__ __volatile__( "1:	mov""l"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""l"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u16 *)to) : "m"((*(struct __large_struct *)(from))), "i"(2), "0"(ret));break; default: (*(u16 *)to) = __get_user_bad(); } } while (0);
   return ret;
  case 4:
   do { ret = 0; (void)0; switch (4) { case 1: __asm__ __volatile__( "1:	mov""b"" %2,%""b""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=q" (*(u32 *)to) : "m"((*(struct __large_struct *)(from))), "i"(4), "0"(ret));break; case 2: __asm__ __volatile__( "1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u32 *)to) : "m"((*(struct __large_struct *)(from))), "i"(4), "0"(ret));break; case 4: __asm__ __volatile__( "1:	mov""l"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""l"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(ret), "=r" (*(u32 *)to) : "m"((*(struct __large_struct *)(from))), "i"(4), "0"(ret));break; default: (*(u32 *)to) = __get_user_bad(); } } while (0);
   return ret;
  }
 }
 return __copy_from_user_ll_nocache(to, from, n);
}

static inline __attribute__((always_inline)) __attribute__((always_inline)) unsigned long
__copy_from_user_inatomic_nocache(void *to, const void *from, unsigned long n)
{
       return __copy_from_user_ll_nocache_nozero(to, from, n);
}

unsigned long __attribute__((warn_unused_result)) copy_to_user(void *to,
    const void *from, unsigned long n);
unsigned long __attribute__((warn_unused_result)) copy_from_user(void *to,
    const void *from, unsigned long n);
long __attribute__((warn_unused_result)) strncpy_from_user(char *dst, const char *src,
    long count);
long __attribute__((warn_unused_result)) __strncpy_from_user(char *dst,
    const char *src, long count);

long strnlen_user(const char *str, long n);
unsigned long __attribute__((warn_unused_result)) clear_user(void *mem, unsigned long len);
unsigned long __attribute__((warn_unused_result)) __clear_user(void *mem, unsigned long len);


static inline __attribute__((always_inline)) gfp_t mapping_gfp_mask(struct address_space * mapping)
{
 return ( gfp_t)mapping->flags & (( gfp_t)((1 << 20) - 1));
}





static inline __attribute__((always_inline)) void mapping_set_gfp_mask(struct address_space *m, gfp_t mask)
{
 m->flags = (m->flags & ~( unsigned long)(( gfp_t)((1 << 20) - 1))) |
    ( unsigned long)mask;
}

void release_pages(struct page **pages, int nr, int cold);





static inline __attribute__((always_inline)) struct page *page_cache_alloc(struct address_space *x)
{
 return alloc_pages_node(((0)), mapping_gfp_mask(x), 0);
}

static inline __attribute__((always_inline)) struct page *page_cache_alloc_cold(struct address_space *x)
{
 return alloc_pages_node(((0)), mapping_gfp_mask(x)|(( gfp_t)0x100u), 0);
}


typedef int filler_t(void *, struct page *);

extern struct page * find_get_page(struct address_space *mapping,
    unsigned long index);
extern struct page * find_lock_page(struct address_space *mapping,
    unsigned long index);
extern struct page * find_trylock_page(
   struct address_space *mapping, unsigned long index);
extern struct page * find_or_create_page(struct address_space *mapping,
    unsigned long index, gfp_t gfp_mask);
unsigned find_get_pages(struct address_space *mapping, unsigned long start,
   unsigned int nr_pages, struct page **pages);
unsigned find_get_pages_contig(struct address_space *mapping, unsigned long start,
          unsigned int nr_pages, struct page **pages);
unsigned find_get_pages_tag(struct address_space *mapping, unsigned long *index,
   int tag, unsigned int nr_pages, struct page **pages);




static inline __attribute__((always_inline)) struct page *grab_cache_page(struct address_space *mapping, unsigned long index)
{
 return find_or_create_page(mapping, index, mapping_gfp_mask(mapping));
}

extern struct page * grab_cache_page_nowait(struct address_space *mapping,
    unsigned long index);
extern struct page * read_cache_page(struct address_space *mapping,
    unsigned long index, filler_t *filler,
    void *data);
extern int read_cache_pages(struct address_space *mapping,
  struct list_head *pages, filler_t *filler, void *data);

static inline __attribute__((always_inline)) struct page *read_mapping_page(struct address_space *mapping,
          unsigned long index, void *data)
{
 filler_t *filler = (filler_t *)mapping->a_ops->readpage;
 return read_cache_page(mapping, index, filler, data);
}

int add_to_page_cache(struct page *page, struct address_space *mapping,
    unsigned long index, gfp_t gfp_mask);
int add_to_page_cache_lru(struct page *page, struct address_space *mapping,
    unsigned long index, gfp_t gfp_mask);
extern void remove_from_page_cache(struct page *page);
extern void __remove_from_page_cache(struct page *page);




static inline __attribute__((always_inline)) loff_t page_offset(struct page *page)
{
 return ((loff_t)page->index) << 12;
}

static inline __attribute__((always_inline)) unsigned long linear_page_index(struct vm_area_struct *vma,
     unsigned long address)
{
 unsigned long pgoff = (address - vma->vm_start) >> 12;
 pgoff += vma->vm_pgoff;
 return pgoff >> (12 - 12);
}

extern void __lock_page(struct page *page) __attribute__((regparm(3)));
extern void unlock_page(struct page *page) __attribute__((regparm(3)));

static inline __attribute__((always_inline)) void lock_page(struct page *page)
{
 do { __might_sleep("include/linux/pagemap.h", 137); cond_resched(); } while (0);
 if (test_and_set_bit(0, &(page)->flags))
  __lock_page(page);
}





extern void wait_on_page_bit(struct page *page, int bit_nr) __attribute__((regparm(3)));

static inline __attribute__((always_inline)) void wait_on_page_locked(struct page *page)
{
 if ((__builtin_constant_p(0) ? constant_test_bit((0),(&(page)->flags)) : variable_test_bit((0),(&(page)->flags))))
  wait_on_page_bit(page, 0);
}




static inline __attribute__((always_inline)) void wait_on_page_writeback(struct page *page)
{
 if ((__builtin_constant_p(12) ? constant_test_bit((12),(&(page)->flags)) : variable_test_bit((12),(&(page)->flags))))
  wait_on_page_bit(page, 12);
}

extern void end_page_writeback(struct page *page);







static inline __attribute__((always_inline)) int fault_in_pages_writeable(char *uaddr, int size)
{
 int ret;





 ret = ({ long __pu_err; do { __pu_err = 0; (void)0; switch ((sizeof(*(uaddr)))) { case 1: __asm__ __volatile__( "1:	mov""b"" %""b""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(__pu_err) : "iq" (((__typeof__(*(uaddr)))(0))), "m"((*(struct __large_struct *)(((uaddr))))), "i"(-14), "0"(__pu_err));break; case 2: __asm__ __volatile__( "1:	mov""w"" %""w""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(__pu_err) : "ir" (((__typeof__(*(uaddr)))(0))), "m"((*(struct __large_struct *)(((uaddr))))), "i"(-14), "0"(__pu_err));break; case 4: __asm__ __volatile__( "1:	mov""l"" %""""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(__pu_err) : "ir" (((__typeof__(*(uaddr)))(0))), "m"((*(struct __large_struct *)(((uaddr))))), "i"(-14), "0"(__pu_err)); break; case 8: __asm__ __volatile__( "1:	movl %%eax,0(%2)\n" "2:	movl %%edx,4(%2)\n" "3:\n" ".section .fixup,\"ax\"\n" "4:	movl %3,%0\n" "	jmp 3b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,4b\n" "	.long 2b,4b\n" ".previous" : "=r"(__pu_err) : "A" ((__typeof__(*((uaddr))))(((__typeof__(*(uaddr)))(0)))), "r" (((uaddr))), "i"(-14), "0"(__pu_err)); break; default: __put_user_bad(); } } while (0); __pu_err; });
 if (ret == 0) {
  char *end = uaddr + size - 1;





  if (((unsigned long)uaddr & (~((1UL << 12)-1))) !=
    ((unsigned long)end & (~((1UL << 12)-1))))
    ret = ({ long __pu_err; do { __pu_err = 0; (void)0; switch ((sizeof(*(end)))) { case 1: __asm__ __volatile__( "1:	mov""b"" %""b""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(__pu_err) : "iq" (((__typeof__(*(end)))(0))), "m"((*(struct __large_struct *)(((end))))), "i"(-14), "0"(__pu_err));break; case 2: __asm__ __volatile__( "1:	mov""w"" %""w""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(__pu_err) : "ir" (((__typeof__(*(end)))(0))), "m"((*(struct __large_struct *)(((end))))), "i"(-14), "0"(__pu_err));break; case 4: __asm__ __volatile__( "1:	mov""l"" %""""1,%2\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(__pu_err) : "ir" (((__typeof__(*(end)))(0))), "m"((*(struct __large_struct *)(((end))))), "i"(-14), "0"(__pu_err)); break; case 8: __asm__ __volatile__( "1:	movl %%eax,0(%2)\n" "2:	movl %%edx,4(%2)\n" "3:\n" ".section .fixup,\"ax\"\n" "4:	movl %3,%0\n" "	jmp 3b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,4b\n" "	.long 2b,4b\n" ".previous" : "=r"(__pu_err) : "A" ((__typeof__(*((end))))(((__typeof__(*(end)))(0)))), "r" (((end))), "i"(-14), "0"(__pu_err)); break; default: __put_user_bad(); } } while (0); __pu_err; });
 }
 return ret;
}

static inline __attribute__((always_inline)) void fault_in_pages_readable(const char *uaddr, int size)
{
 volatile char c;
 int ret;

 ret = ({ long __gu_err; unsigned long __gu_val; do { __gu_err = 0; (void)0; switch ((sizeof(*(uaddr)))) { case 1: __asm__ __volatile__( "1:	mov""b"" %2,%""b""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(__gu_err), "=q" (__gu_val) : "m"((*(struct __large_struct *)(((uaddr))))), "i"(-14), "0"(__gu_err));break; case 2: __asm__ __volatile__( "1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(__gu_err), "=r" (__gu_val) : "m"((*(struct __large_struct *)(((uaddr))))), "i"(-14), "0"(__gu_err));break; case 4: __asm__ __volatile__( "1:	mov""l"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""l"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(__gu_err), "=r" (__gu_val) : "m"((*(struct __large_struct *)(((uaddr))))), "i"(-14), "0"(__gu_err));break; default: (__gu_val) = __get_user_bad(); } } while (0); ((c)) = (__typeof__(*((uaddr))))__gu_val; __gu_err; });
 if (ret == 0) {
  const char *end = uaddr + size - 1;

  if (((unsigned long)uaddr & (~((1UL << 12)-1))) !=
    ((unsigned long)end & (~((1UL << 12)-1))))
    ({ long __gu_err; unsigned long __gu_val; do { __gu_err = 0; (void)0; switch ((sizeof(*(end)))) { case 1: __asm__ __volatile__( "1:	mov""b"" %2,%""b""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(__gu_err), "=q" (__gu_val) : "m"((*(struct __large_struct *)(((end))))), "i"(-14), "0"(__gu_err));break; case 2: __asm__ __volatile__( "1:	mov""w"" %2,%""w""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(__gu_err), "=r" (__gu_val) : "m"((*(struct __large_struct *)(((end))))), "i"(-14), "0"(__gu_err));break; case 4: __asm__ __volatile__( "1:	mov""l"" %2,%""""1\n" "2:\n" ".section .fixup,\"ax\"\n" "3:	movl %3,%0\n" "	xor""l"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" ".section __ex_table,\"a\"\n" "	.align 4\n" "	.long 1b,3b\n" ".previous" : "=r"(__gu_err), "=r" (__gu_val) : "m"((*(struct __large_struct *)(((end))))), "i"(-14), "0"(__gu_err));break; default: (__gu_val) = __get_user_bad(); } } while (0); ((c)) = (__typeof__(*((end))))__gu_val; __gu_err; });
 }
}



enum bdi_state {
 BDI_pdflush,
 BDI_write_congested,
 BDI_read_congested,
 BDI_unused,
};

typedef int (congested_fn)(void *, int);

struct backing_dev_info {
 unsigned long ra_pages;
 unsigned long state;
 unsigned int capabilities;
 congested_fn *congested_fn;
 void *congested_data;
 void (*unplug_io_fn)(struct backing_dev_info *, struct page *);
 void *unplug_io_data;
};

extern struct backing_dev_info default_backing_dev_info;
void default_unplug_io_fn(struct backing_dev_info *bdi, struct page *page);

int writeback_acquire(struct backing_dev_info *bdi);
int writeback_in_progress(struct backing_dev_info *bdi);
void writeback_release(struct backing_dev_info *bdi);

static inline __attribute__((always_inline)) int bdi_congested(struct backing_dev_info *bdi, int bdi_bits)
{
 if (bdi->congested_fn)
  return bdi->congested_fn(bdi->congested_data, bdi_bits);
 return (bdi->state & bdi_bits);
}

static inline __attribute__((always_inline)) int bdi_read_congested(struct backing_dev_info *bdi)
{
 return bdi_congested(bdi, 1 << BDI_read_congested);
}

static inline __attribute__((always_inline)) int bdi_write_congested(struct backing_dev_info *bdi)
{
 return bdi_congested(bdi, 1 << BDI_write_congested);
}

static inline __attribute__((always_inline)) int bdi_rw_congested(struct backing_dev_info *bdi)
{
 return bdi_congested(bdi, (1 << BDI_read_congested)|
      (1 << BDI_write_congested));
}




struct kmem_cache;

typedef void * (mempool_alloc_t)(gfp_t gfp_mask, void *pool_data);
typedef void (mempool_free_t)(void *element, void *pool_data);

typedef struct mempool_s {
 spinlock_t lock;
 int min_nr;
 int curr_nr;
 void **elements;

 void *pool_data;
 mempool_alloc_t *alloc;
 mempool_free_t *free;
 wait_queue_head_t wait;
} mempool_t;

extern mempool_t *mempool_create(int min_nr, mempool_alloc_t *alloc_fn,
   mempool_free_t *free_fn, void *pool_data);
extern mempool_t *mempool_create_node(int min_nr, mempool_alloc_t *alloc_fn,
   mempool_free_t *free_fn, void *pool_data, int nid);

extern int mempool_resize(mempool_t *pool, int new_min_nr, gfp_t gfp_mask);
extern void mempool_destroy(mempool_t *pool);
extern void * mempool_alloc(mempool_t *pool, gfp_t gfp_mask);
extern void mempool_free(void *element, mempool_t *pool);





void *mempool_alloc_slab(gfp_t gfp_mask, void *pool_data);
void mempool_free_slab(void *element, void *pool_data);
static inline __attribute__((always_inline)) mempool_t *
mempool_create_slab_pool(int min_nr, struct kmem_cache *kc)
{
 return mempool_create(min_nr, mempool_alloc_slab, mempool_free_slab,
         (void *) kc);
}





void *mempool_kmalloc(gfp_t gfp_mask, void *pool_data);
void *mempool_kzalloc(gfp_t gfp_mask, void *pool_data);
void mempool_kfree(void *element, void *pool_data);
static inline __attribute__((always_inline)) mempool_t *mempool_create_kmalloc_pool(int min_nr, size_t size)
{
 return mempool_create(min_nr, mempool_kmalloc, mempool_kfree,
         (void *) size);
}
static inline __attribute__((always_inline)) mempool_t *mempool_create_kzalloc_pool(int min_nr, size_t size)
{
 return mempool_create(min_nr, mempool_kzalloc, mempool_kfree,
         (void *) size);
}





void *mempool_alloc_pages(gfp_t gfp_mask, void *pool_data);
void mempool_free_pages(void *element, void *pool_data);
static inline __attribute__((always_inline)) mempool_t *mempool_create_page_pool(int min_nr, int order)
{
 return mempool_create(min_nr, mempool_alloc_pages, mempool_free_pages,
         (void *)(long)order);
}





enum {
 IOPRIO_CLASS_NONE,
 IOPRIO_CLASS_RT,
 IOPRIO_CLASS_BE,
 IOPRIO_CLASS_IDLE,
};






enum {
 IOPRIO_WHO_PROCESS = 1,
 IOPRIO_WHO_PGRP,
 IOPRIO_WHO_USER,
};






static inline __attribute__((always_inline)) int task_ioprio(struct task_struct *task)
{
 do { if (__builtin_expect(!!((!((((task->ioprio)) >> (13)) != IOPRIO_CLASS_NONE))!=0), 0)) { printk("BUG: warning at %s:%d/%s()\n", "include/linux/ioprio.h", 50, (__func__)); dump_stack(); } } while (0);
 return ((task->ioprio) & ((1UL << (13)) - 1));
}

static inline __attribute__((always_inline)) int task_nice_ioprio(struct task_struct *task)
{
 return (task_nice(task) + 20) / 5;
}




extern int ioprio_best(unsigned short aprio, unsigned short bprio);







extern unsigned int __attribute__((regparm(3))) ioread8(void *);
extern unsigned int __attribute__((regparm(3))) ioread16(void *);
extern unsigned int __attribute__((regparm(3))) ioread16be(void *);
extern unsigned int __attribute__((regparm(3))) ioread32(void *);
extern unsigned int __attribute__((regparm(3))) ioread32be(void *);

extern void __attribute__((regparm(3))) iowrite8(u8, void *);
extern void __attribute__((regparm(3))) iowrite16(u16, void *);
extern void __attribute__((regparm(3))) iowrite16be(u16, void *);
extern void __attribute__((regparm(3))) iowrite32(u32, void *);
extern void __attribute__((regparm(3))) iowrite32be(u32, void *);

extern void __attribute__((regparm(3))) ioread8_rep(void *port, void *buf, unsigned long count);
extern void __attribute__((regparm(3))) ioread16_rep(void *port, void *buf, unsigned long count);
extern void __attribute__((regparm(3))) ioread32_rep(void *port, void *buf, unsigned long count);

extern void __attribute__((regparm(3))) iowrite8_rep(void *port, const void *buf, unsigned long count);
extern void __attribute__((regparm(3))) iowrite16_rep(void *port, const void *buf, unsigned long count);
extern void __attribute__((regparm(3))) iowrite32_rep(void *port, const void *buf, unsigned long count);


extern void *ioport_map(unsigned long port, unsigned int nr);
extern void ioport_unmap(void *);


struct pci_dev;
extern void *pci_iomap(struct pci_dev *dev, int bar, unsigned long max);
extern void pci_iounmap(struct pci_dev *dev, void *);









struct vm_area_struct;

struct vm_struct {
 void *addr;
 unsigned long size;
 unsigned long flags;
 struct page **pages;
 unsigned int nr_pages;
 unsigned long phys_addr;
 struct vm_struct *next;
};




extern void *vmalloc(unsigned long size);
extern void *vmalloc_user(unsigned long size);
extern void *vmalloc_node(unsigned long size, int node);
extern void *vmalloc_exec(unsigned long size);
extern void *vmalloc_32(unsigned long size);
extern void *vmalloc_32_user(unsigned long size);
extern void *__vmalloc(unsigned long size, gfp_t gfp_mask, pgprot_t prot);
extern void *__vmalloc_area(struct vm_struct *area, gfp_t gfp_mask,
    pgprot_t prot);
extern void *__vmalloc_node(unsigned long size, gfp_t gfp_mask,
    pgprot_t prot, int node);
extern void vfree(void *addr);

extern void *vmap(struct page **pages, unsigned int count,
   unsigned long flags, pgprot_t prot);
extern void vunmap(void *addr);

extern int remap_vmalloc_range(struct vm_area_struct *vma, void *addr,
       unsigned long pgoff);




extern struct vm_struct *get_vm_area(unsigned long size, unsigned long flags);
extern struct vm_struct *__get_vm_area(unsigned long size, unsigned long flags,
     unsigned long start, unsigned long end);
extern struct vm_struct *get_vm_area_node(unsigned long size,
     unsigned long flags, int node);
extern struct vm_struct *remove_vm_area(void *addr);
extern struct vm_struct *__remove_vm_area(void *addr);
extern int map_vm_area(struct vm_struct *area, pgprot_t prot,
   struct page ***pages);
extern void unmap_vm_area(struct vm_struct *area);




extern rwlock_t vmlist_lock;
extern struct vm_struct *vmlist;


static inline __attribute__((always_inline)) unsigned long virt_to_phys(volatile void * address)
{
 return ((unsigned long)(address)-((unsigned long)((unsigned long)0xC0000000)));
}

static inline __attribute__((always_inline)) void * phys_to_virt(unsigned long address)
{
 return ((void *)((unsigned long)(address)+((unsigned long)((unsigned long)0xC0000000))));
}






extern void * __ioremap(unsigned long offset, unsigned long size, unsigned long flags);

static inline __attribute__((always_inline)) void * ioremap(unsigned long offset, unsigned long size)
{
 return __ioremap(offset, size, 0);
}

extern void * ioremap_nocache(unsigned long offset, unsigned long size);
extern void iounmap(volatile void *addr);






extern void *bt_ioremap(unsigned long offset, unsigned long size);
extern void bt_iounmap(void *addr, unsigned long size);

static inline __attribute__((always_inline)) unsigned char readb(const volatile void *addr)
{
 return *(volatile unsigned char *) addr;
}
static inline __attribute__((always_inline)) unsigned short readw(const volatile void *addr)
{
 return *(volatile unsigned short *) addr;
}
static inline __attribute__((always_inline)) unsigned int readl(const volatile void *addr)
{
 return *(volatile unsigned int *) addr;
}







static inline __attribute__((always_inline)) void writeb(unsigned char b, volatile void *addr)
{
 *(volatile unsigned char *) addr = b;
}
static inline __attribute__((always_inline)) void writew(unsigned short b, volatile void *addr)
{
 *(volatile unsigned short *) addr = b;
}
static inline __attribute__((always_inline)) void writel(unsigned int b, volatile void *addr)
{
 *(volatile unsigned int *) addr = b;
}






static inline __attribute__((always_inline)) void memset_io(volatile void *addr, unsigned char val, int count)
{
 (__builtin_constant_p(val) ? (__builtin_constant_p((count)) ? __constant_c_and_count_memset((((void *) addr)),((0x01010101UL*(unsigned char)(val))),((count))) : __constant_c_memset((((void *) addr)),((0x01010101UL*(unsigned char)(val))),((count)))) : (__builtin_constant_p((count)) ? __memset_generic(((((void *) addr))),(((val))),(((count)))) : __memset_generic((((void *) addr)),((val)),((count)))));
}
static inline __attribute__((always_inline)) void memcpy_fromio(void *dst, const volatile void *src, int count)
{
 __memcpy(dst, (void *) src, count);
}
static inline __attribute__((always_inline)) void memcpy_toio(volatile void *dst, const void *src, int count)
{
 __memcpy((void *) dst, src, count);
}

static inline __attribute__((always_inline)) int check_signature(volatile void * io_addr,
 const unsigned char *signature, int length)
{
 int retval = 0;
 do {
  if (readb(io_addr) != *signature)
   goto out;
  io_addr++;
  signature++;
  length--;
 } while (length);
 retval = 1;
out:
 return retval;
}

static inline __attribute__((always_inline)) void flush_write_buffers(void)
{
 __asm__ __volatile__ ("lock; addl $0,0(%%esp)": : :"memory");
}

static inline __attribute__((always_inline)) void slow_down_io(void) {
 __asm__ __volatile__(
  "outb %%al,$0x80;"



  : : );
}

static inline __attribute__((always_inline)) void outb_local(unsigned char value, int port) { __asm__ __volatile__("out" "b" " %" "b" "0, %w1" : : "a"(value), "Nd"(port)); } static inline __attribute__((always_inline)) unsigned char inb_local(int port) { unsigned char value; __asm__ __volatile__("in" "b" " %w1, %" "b" "0" : "=a"(value) : "Nd"(port)); return value; } static inline __attribute__((always_inline)) void outb_local_p(unsigned char value, int port) { outb_local(value, port); slow_down_io(); } static inline __attribute__((always_inline)) unsigned char inb_local_p(int port) { unsigned char value = inb_local(port); slow_down_io(); return value; } static inline __attribute__((always_inline)) void outb(unsigned char value, int port) { outb_local(value, port); } static inline __attribute__((always_inline)) unsigned char inb(int port) { return inb_local(port); } static inline __attribute__((always_inline)) void outb_p(unsigned char value, int port) { outb(value, port); slow_down_io(); } static inline __attribute__((always_inline)) unsigned char inb_p(int port) { unsigned char value = inb(port); slow_down_io(); return value; } static inline __attribute__((always_inline)) void outsb(int port, const void *addr, unsigned long count) { __asm__ __volatile__("rep; outs" "b" : "+S"(addr), "+c"(count) : "d"(port)); } static inline __attribute__((always_inline)) void insb(int port, void *addr, unsigned long count) { __asm__ __volatile__("rep; ins" "b" : "+D"(addr), "+c"(count) : "d"(port)); }
static inline __attribute__((always_inline)) void outw_local(unsigned short value, int port) { __asm__ __volatile__("out" "w" " %" "w" "0, %w1" : : "a"(value), "Nd"(port)); } static inline __attribute__((always_inline)) unsigned short inw_local(int port) { unsigned short value; __asm__ __volatile__("in" "w" " %w1, %" "w" "0" : "=a"(value) : "Nd"(port)); return value; } static inline __attribute__((always_inline)) void outw_local_p(unsigned short value, int port) { outw_local(value, port); slow_down_io(); } static inline __attribute__((always_inline)) unsigned short inw_local_p(int port) { unsigned short value = inw_local(port); slow_down_io(); return value; } static inline __attribute__((always_inline)) void outw(unsigned short value, int port) { outw_local(value, port); } static inline __attribute__((always_inline)) unsigned short inw(int port) { return inw_local(port); } static inline __attribute__((always_inline)) void outw_p(unsigned short value, int port) { outw(value, port); slow_down_io(); } static inline __attribute__((always_inline)) unsigned short inw_p(int port) { unsigned short value = inw(port); slow_down_io(); return value; } static inline __attribute__((always_inline)) void outsw(int port, const void *addr, unsigned long count) { __asm__ __volatile__("rep; outs" "w" : "+S"(addr), "+c"(count) : "d"(port)); } static inline __attribute__((always_inline)) void insw(int port, void *addr, unsigned long count) { __asm__ __volatile__("rep; ins" "w" : "+D"(addr), "+c"(count) : "d"(port)); }
static inline __attribute__((always_inline)) void outl_local(unsigned int value, int port) { __asm__ __volatile__("out" "l" " %" "" "0, %w1" : : "a"(value), "Nd"(port)); } static inline __attribute__((always_inline)) unsigned int inl_local(int port) { unsigned int value; __asm__ __volatile__("in" "l" " %w1, %" "" "0" : "=a"(value) : "Nd"(port)); return value; } static inline __attribute__((always_inline)) void outl_local_p(unsigned int value, int port) { outl_local(value, port); slow_down_io(); } static inline __attribute__((always_inline)) unsigned int inl_local_p(int port) { unsigned int value = inl_local(port); slow_down_io(); return value; } static inline __attribute__((always_inline)) void outl(unsigned int value, int port) { outl_local(value, port); } static inline __attribute__((always_inline)) unsigned int inl(int port) { return inl_local(port); } static inline __attribute__((always_inline)) void outl_p(unsigned int value, int port) { outl(value, port); slow_down_io(); } static inline __attribute__((always_inline)) unsigned int inl_p(int port) { unsigned int value = inl(port); slow_down_io(); return value; } static inline __attribute__((always_inline)) void outsl(int port, const void *addr, unsigned long count) { __asm__ __volatile__("rep; outs" "l" : "+S"(addr), "+c"(count) : "d"(port)); } static inline __attribute__((always_inline)) void insl(int port, void *addr, unsigned long count) { __asm__ __volatile__("rep; ins" "l" : "+D"(addr), "+c"(count) : "d"(port)); }


struct bio_vec {
 struct page *bv_page;
 unsigned int bv_len;
 unsigned int bv_offset;
};

struct bio_set;
struct bio;
typedef int (bio_end_io_t) (struct bio *, unsigned int, int);
typedef void (bio_destructor_t) (struct bio *);





struct bio {
 sector_t bi_sector;
 struct bio *bi_next;
 struct block_device *bi_bdev;
 unsigned long bi_flags;
 unsigned long bi_rw;



 unsigned short bi_vcnt;
 unsigned short bi_idx;




 unsigned short bi_phys_segments;




 unsigned short bi_hw_segments;

 unsigned int bi_size;






 unsigned int bi_hw_front_size;
 unsigned int bi_hw_back_size;

 unsigned int bi_max_vecs;

 struct bio_vec *bi_io_vec;

 bio_end_io_t *bi_end_io;
 atomic_t bi_cnt;

 void *bi_private;

 bio_destructor_t *bi_destructor;
};

struct bio_pair {
 struct bio bio1, bio2;
 struct bio_vec bv1, bv2;
 atomic_t cnt;
 int error;
};
extern struct bio_pair *bio_split(struct bio *bi, mempool_t *pool,
      int first_sectors);
extern mempool_t *bio_split_pool;
extern void bio_pair_release(struct bio_pair *dbio);

extern struct bio_set *bioset_create(int, int, int);
extern void bioset_free(struct bio_set *);

extern struct bio *bio_alloc(gfp_t, int);
extern struct bio *bio_alloc_bioset(gfp_t, int, struct bio_set *);
extern void bio_put(struct bio *);
extern void bio_free(struct bio *, struct bio_set *);

extern void bio_endio(struct bio *, unsigned int, int);
struct request_queue;
extern int bio_phys_segments(struct request_queue *, struct bio *);
extern int bio_hw_segments(struct request_queue *, struct bio *);

extern void __bio_clone(struct bio *, struct bio *);
extern struct bio *bio_clone(struct bio *, gfp_t);

extern void bio_init(struct bio *);

extern int bio_add_page(struct bio *, struct page *, unsigned int,unsigned int);
extern int bio_add_pc_page(struct request_queue *, struct bio *, struct page *,
      unsigned int, unsigned int);
extern int bio_get_nr_vecs(struct block_device *);
extern struct bio *bio_map_user(struct request_queue *, struct block_device *,
    unsigned long, unsigned int, int);
struct sg_iovec;
extern struct bio *bio_map_user_iov(struct request_queue *,
        struct block_device *,
        struct sg_iovec *, int, int);
extern void bio_unmap_user(struct bio *);
extern struct bio *bio_map_kern(struct request_queue *, void *, unsigned int,
    gfp_t);
extern void bio_set_pages_dirty(struct bio *bio);
extern void bio_check_pages_dirty(struct bio *bio);
extern struct bio *bio_copy_user(struct request_queue *, unsigned long, unsigned int, int);
extern int bio_uncopy_user(struct bio *);
void zero_fill_bio(struct bio *bio);

static inline __attribute__((always_inline)) char *bvec_kmap_irq(struct bio_vec *bvec, unsigned long *flags)
{
 unsigned long addr;





 do { do { (*flags) = __raw_local_irq_save(); } while (0); do { } while (0); } while (0);
 addr = (unsigned long) kmap_atomic(bvec->bv_page, KM_BIO_SRC_IRQ);

 do { if (__builtin_expect(!!((addr & ~(~((1UL << 12)-1)))!=0), 0)) __asm__ __volatile__( "ud2\n" "\t.word %c0\n" "\t.long %c1\n" : : "i" (331), "i" ("include/linux/bio.h")); } while(0);

 return (char *) addr + bvec->bv_offset;
}

static inline __attribute__((always_inline)) void bvec_kunmap_irq(char *buffer, unsigned long *flags)
{
 unsigned long ptr = (unsigned long) buffer & (~((1UL << 12)-1));

 kunmap_atomic((void *) ptr, KM_BIO_SRC_IRQ);
 do { if (raw_irqs_disabled_flags(*flags)) { raw_local_irq_restore(*flags); do { } while (0); } else { do { } while (0); raw_local_irq_restore(*flags); } } while (0);
}






static inline __attribute__((always_inline)) char *__bio_kmap_irq(struct bio *bio, unsigned short idx,
       unsigned long *flags)
{
 return bvec_kmap_irq((&((bio)->bi_io_vec[(idx)])), flags);
}








struct scatterlist {
    struct page *page;
    unsigned int offset;
    dma_addr_t dma_address;
    unsigned int length;
};


struct scsi_ioctl_command;

struct request_queue;
typedef struct request_queue request_queue_t;
struct elevator_queue;
typedef struct elevator_queue elevator_t;
struct request_pm_state;
struct blk_trace;







struct as_io_context {
 spinlock_t lock;

 void (*dtor)(struct as_io_context *aic);
 void (*exit)(struct as_io_context *aic);

 unsigned long state;
 atomic_t nr_queued;
 atomic_t nr_dispatched;



 unsigned long last_end_request;
 unsigned long ttime_total;
 unsigned long ttime_samples;
 unsigned long ttime_mean;

 unsigned int seek_samples;
 sector_t last_request_pos;
 u64 seek_total;
 sector_t seek_mean;
};

struct cfq_queue;
struct cfq_io_context {
 struct rb_node rb_node;
 void *key;

 struct cfq_queue *cfqq[2];

 struct io_context *ioc;

 unsigned long last_end_request;
 sector_t last_request_pos;
  unsigned long last_queue;

 unsigned long ttime_total;
 unsigned long ttime_samples;
 unsigned long ttime_mean;

 unsigned int seek_samples;
 u64 seek_total;
 sector_t seek_mean;

 struct list_head queue_list;

 void (*dtor)(struct io_context *);
 void (*exit)(struct io_context *);
};






struct io_context {
 atomic_t refcount;
 struct task_struct *task;

 int (*set_ioprio)(struct io_context *, unsigned int);




 unsigned long last_waited;
 int nr_batch_requests;

 struct as_io_context *aic;
 struct rb_root cic_root;
};

void put_io_context(struct io_context *ioc);
void exit_io_context(void);
struct io_context *current_io_context(gfp_t gfp_flags);
struct io_context *get_io_context(gfp_t gfp_flags);
void copy_io_context(struct io_context **pdst, struct io_context **psrc);
void swap_io_context(struct io_context **ioc1, struct io_context **ioc2);

struct request;
typedef void (rq_end_io_fn)(struct request *, int);

struct request_list {
 int count[2];
 int starved[2];
 int elvpriv;
 mempool_t *rq_pool;
 wait_queue_head_t wait[2];
};






struct request {
 struct list_head queuelist;
 struct list_head donelist;

 unsigned long flags;





 sector_t sector;
 unsigned long nr_sectors;

 unsigned int current_nr_sectors;

 sector_t hard_sector;
 unsigned long hard_nr_sectors;

 unsigned int hard_cur_sectors;

 struct bio *bio;
 struct bio *biotail;

 void *elevator_private;
 void *completion_data;

 int rq_status;
 int errors;
 struct gendisk *rq_disk;
 unsigned long start_time;




 unsigned short nr_phys_segments;






 unsigned short nr_hw_segments;

 unsigned short ioprio;

 int tag;

 int ref_count;
 request_queue_t *q;
 struct request_list *rl;

 struct completion *waiting;
 void *special;
 char *buffer;




 unsigned int cmd_len;
 unsigned char cmd[16];

 unsigned int data_len;
 unsigned int sense_len;
 void *data;
 void *sense;

 unsigned int timeout;
 int retries;




 rq_end_io_fn *end_io;
 void *end_io_data;
};




enum rq_flag_bits {
 __REQ_RW,
 __REQ_FAILFAST,
 __REQ_SORTED,
 __REQ_SOFTBARRIER,
 __REQ_HARDBARRIER,
 __REQ_FUA,
 __REQ_CMD,
 __REQ_NOMERGE,
 __REQ_STARTED,
 __REQ_DONTPREP,
 __REQ_QUEUED,
 __REQ_ELVPRIV,



 __REQ_PC,
 __REQ_BLOCK_PC,
 __REQ_SENSE,

 __REQ_FAILED,
 __REQ_QUIET,
 __REQ_SPECIAL,
 __REQ_DRIVE_CMD,
 __REQ_DRIVE_TASK,
 __REQ_DRIVE_TASKFILE,
 __REQ_PREEMPT,
 __REQ_PM_SUSPEND,
 __REQ_PM_RESUME,
 __REQ_PM_SHUTDOWN,
 __REQ_ORDERED_COLOR,
 __REQ_RW_SYNC,
 __REQ_NR_BITS,
};

struct request_pm_state
{

 int pm_step;

 u32 pm_state;
 void* data;
};





typedef int (elevator_merge_fn) (request_queue_t *, struct request **,
     struct bio *);

typedef void (elevator_merge_req_fn) (request_queue_t *, struct request *, struct request *);

typedef void (elevator_merged_fn) (request_queue_t *, struct request *);

typedef int (elevator_dispatch_fn) (request_queue_t *, int);

typedef void (elevator_add_req_fn) (request_queue_t *, struct request *);
typedef int (elevator_queue_empty_fn) (request_queue_t *);
typedef struct request *(elevator_request_list_fn) (request_queue_t *, struct request *);
typedef void (elevator_completed_req_fn) (request_queue_t *, struct request *);
typedef int (elevator_may_queue_fn) (request_queue_t *, int, struct bio *);

typedef int (elevator_set_req_fn) (request_queue_t *, struct request *, struct bio *, gfp_t);
typedef void (elevator_put_req_fn) (request_queue_t *, struct request *);
typedef void (elevator_activate_req_fn) (request_queue_t *, struct request *);
typedef void (elevator_deactivate_req_fn) (request_queue_t *, struct request *);

typedef void *(elevator_init_fn) (request_queue_t *, elevator_t *);
typedef void (elevator_exit_fn) (elevator_t *);

struct elevator_ops
{
 elevator_merge_fn *elevator_merge_fn;
 elevator_merged_fn *elevator_merged_fn;
 elevator_merge_req_fn *elevator_merge_req_fn;

 elevator_dispatch_fn *elevator_dispatch_fn;
 elevator_add_req_fn *elevator_add_req_fn;
 elevator_activate_req_fn *elevator_activate_req_fn;
 elevator_deactivate_req_fn *elevator_deactivate_req_fn;

 elevator_queue_empty_fn *elevator_queue_empty_fn;
 elevator_completed_req_fn *elevator_completed_req_fn;

 elevator_request_list_fn *elevator_former_req_fn;
 elevator_request_list_fn *elevator_latter_req_fn;

 elevator_set_req_fn *elevator_set_req_fn;
 elevator_put_req_fn *elevator_put_req_fn;

 elevator_may_queue_fn *elevator_may_queue_fn;

 elevator_init_fn *elevator_init_fn;
 elevator_exit_fn *elevator_exit_fn;
 void (*trim)(struct io_context *);
};



struct elv_fs_entry {
 struct attribute attr;
 ssize_t (*show)(elevator_t *, char *);
 ssize_t (*store)(elevator_t *, const char *, size_t);
};




struct elevator_type
{
 struct list_head list;
 struct elevator_ops ops;
 struct elevator_type *elevator_type;
 struct elv_fs_entry *elevator_attrs;
 char elevator_name[(16)];
 struct module *elevator_owner;
};




struct elevator_queue
{
 struct elevator_ops *ops;
 void *elevator_data;
 struct kobject kobj;
 struct elevator_type *elevator_type;
 struct mutex sysfs_lock;
};




extern void elv_dispatch_sort(request_queue_t *, struct request *);
extern void elv_add_request(request_queue_t *, struct request *, int, int);
extern void __elv_add_request(request_queue_t *, struct request *, int, int);
extern void elv_insert(request_queue_t *, struct request *, int);
extern int elv_merge(request_queue_t *, struct request **, struct bio *);
extern void elv_merge_requests(request_queue_t *, struct request *,
          struct request *);
extern void elv_merged_request(request_queue_t *, struct request *);
extern void elv_dequeue_request(request_queue_t *, struct request *);
extern void elv_requeue_request(request_queue_t *, struct request *);
extern int elv_queue_empty(request_queue_t *);
extern struct request *elv_next_request(struct request_queue *q);
extern struct request *elv_former_request(request_queue_t *, struct request *);
extern struct request *elv_latter_request(request_queue_t *, struct request *);
extern int elv_register_queue(request_queue_t *q);
extern void elv_unregister_queue(request_queue_t *q);
extern int elv_may_queue(request_queue_t *, int, struct bio *);
extern void elv_completed_request(request_queue_t *, struct request *);
extern int elv_set_request(request_queue_t *, struct request *, struct bio *, gfp_t);
extern void elv_put_request(request_queue_t *, struct request *);




extern int elv_register(struct elevator_type *);
extern void elv_unregister(struct elevator_type *);




extern ssize_t elv_iosched_show(request_queue_t *, char *);
extern ssize_t elv_iosched_store(request_queue_t *, const char *, size_t);

extern int elevator_init(request_queue_t *, char *);
extern void elevator_exit(elevator_t *);
extern int elv_rq_merge_ok(struct request *, struct bio *);

enum {
 ELV_MQUEUE_MAY,
 ELV_MQUEUE_NO,
 ELV_MQUEUE_MUST,
};


typedef int (merge_request_fn) (request_queue_t *, struct request *,
    struct bio *);
typedef int (merge_requests_fn) (request_queue_t *, struct request *,
     struct request *);
typedef void (request_fn_proc) (request_queue_t *q);
typedef int (make_request_fn) (request_queue_t *q, struct bio *bio);
typedef int (prep_rq_fn) (request_queue_t *, struct request *);
typedef void (unplug_fn) (request_queue_t *);

struct bio_vec;
typedef int (merge_bvec_fn) (request_queue_t *, struct bio *, struct bio_vec *);
typedef void (activity_fn) (void *data, int rw);
typedef int (issue_flush_fn) (request_queue_t *, struct gendisk *, sector_t *);
typedef void (prepare_flush_fn) (request_queue_t *, struct request *);
typedef void (softirq_done_fn)(struct request *);

enum blk_queue_state {
 Queue_down,
 Queue_up,
};

struct blk_queue_tag {
 struct request **tag_index;
 unsigned long *tag_map;
 struct list_head busy_list;
 int busy;
 int max_depth;
 int real_max_depth;
 atomic_t refcnt;
};

struct request_queue
{



 struct list_head queue_head;
 struct request *last_merge;
 elevator_t *elevator;




 struct request_list rq;

 request_fn_proc *request_fn;
 merge_request_fn *back_merge_fn;
 merge_request_fn *front_merge_fn;
 merge_requests_fn *merge_requests_fn;
 make_request_fn *make_request_fn;
 prep_rq_fn *prep_rq_fn;
 unplug_fn *unplug_fn;
 merge_bvec_fn *merge_bvec_fn;
 activity_fn *activity_fn;
 issue_flush_fn *issue_flush_fn;
 prepare_flush_fn *prepare_flush_fn;
 softirq_done_fn *softirq_done_fn;




 sector_t end_sector;
 struct request *boundary_rq;




 struct timer_list unplug_timer;
 int unplug_thresh;
 unsigned long unplug_delay;
 struct work_struct unplug_work;

 struct backing_dev_info backing_dev_info;





 void *queuedata;

 void *activity_data;




 unsigned long bounce_pfn;
 gfp_t bounce_gfp;




 unsigned long queue_flags;






 spinlock_t __queue_lock;
 spinlock_t *queue_lock;




 struct kobject kobj;




 unsigned long nr_requests;
 unsigned int nr_congestion_on;
 unsigned int nr_congestion_off;
 unsigned int nr_batching;

 unsigned int max_sectors;
 unsigned int max_hw_sectors;
 unsigned short max_phys_segments;
 unsigned short max_hw_segments;
 unsigned short hardsect_size;
 unsigned int max_segment_size;

 unsigned long seg_boundary_mask;
 unsigned int dma_alignment;

 struct blk_queue_tag *queue_tags;

 unsigned int nr_sorted;
 unsigned int in_flight;




 unsigned int sg_timeout;
 unsigned int sg_reserved_size;
 int node;

 struct blk_trace *blk_trace;




 unsigned int ordered, next_ordered, ordseq;
 int orderr, ordcolor;
 struct request pre_flush_rq, bar_rq, post_flush_rq;
 struct request *orig_bar_rq;
 unsigned int bi_size;

 struct mutex sysfs_lock;
};

enum {

 QUEUE_ORDERED_NONE = 0x00,
 QUEUE_ORDERED_DRAIN = 0x01,
 QUEUE_ORDERED_TAG = 0x02,

 QUEUE_ORDERED_PREFLUSH = 0x10,
 QUEUE_ORDERED_POSTFLUSH = 0x20,
 QUEUE_ORDERED_FUA = 0x40,

 QUEUE_ORDERED_DRAIN_FLUSH = QUEUE_ORDERED_DRAIN |
   QUEUE_ORDERED_PREFLUSH | QUEUE_ORDERED_POSTFLUSH,
 QUEUE_ORDERED_DRAIN_FUA = QUEUE_ORDERED_DRAIN |
   QUEUE_ORDERED_PREFLUSH | QUEUE_ORDERED_FUA,
 QUEUE_ORDERED_TAG_FLUSH = QUEUE_ORDERED_TAG |
   QUEUE_ORDERED_PREFLUSH | QUEUE_ORDERED_POSTFLUSH,
 QUEUE_ORDERED_TAG_FUA = QUEUE_ORDERED_TAG |
   QUEUE_ORDERED_PREFLUSH | QUEUE_ORDERED_FUA,




 QUEUE_ORDSEQ_STARTED = 0x01,
 QUEUE_ORDSEQ_DRAIN = 0x02,
 QUEUE_ORDSEQ_PREFLUSH = 0x04,
 QUEUE_ORDSEQ_BAR = 0x08,
 QUEUE_ORDSEQ_POSTFLUSH = 0x10,
 QUEUE_ORDSEQ_DONE = 0x20,
};

static inline __attribute__((always_inline)) int blk_queue_full(struct request_queue *q, int rw)
{
 if (rw == 0)
  return (__builtin_constant_p(3) ? constant_test_bit((3),(&q->queue_flags)) : variable_test_bit((3),(&q->queue_flags)));
 return (__builtin_constant_p(4) ? constant_test_bit((4),(&q->queue_flags)) : variable_test_bit((4),(&q->queue_flags)));
}

static inline __attribute__((always_inline)) void blk_set_queue_full(struct request_queue *q, int rw)
{
 if (rw == 0)
  set_bit(3, &q->queue_flags);
 else
  set_bit(4, &q->queue_flags);
}

static inline __attribute__((always_inline)) void blk_clear_queue_full(struct request_queue *q, int rw)
{
 if (rw == 0)
  clear_bit(3, &q->queue_flags);
 else
  clear_bit(4, &q->queue_flags);
}

extern unsigned long blk_max_low_pfn, blk_max_pfn;

extern int init_emergency_isa_pool(void);
extern void blk_queue_bounce(request_queue_t *q, struct bio **bio);

struct sec_size {
 unsigned block_size;
 unsigned block_size_bits;
};

extern int blk_register_queue(struct gendisk *disk);
extern void blk_unregister_queue(struct gendisk *disk);
extern void register_disk(struct gendisk *dev);
extern void generic_make_request(struct bio *bio);
extern void blk_put_request(struct request *);
extern void __blk_put_request(request_queue_t *, struct request *);
extern void blk_end_sync_rq(struct request *rq, int error);
extern struct request *blk_get_request(request_queue_t *, int, gfp_t);
extern void blk_insert_request(request_queue_t *, struct request *, int, void *);
extern void blk_requeue_request(request_queue_t *, struct request *);
extern void blk_plug_device(request_queue_t *);
extern int blk_remove_plug(request_queue_t *);
extern void blk_recount_segments(request_queue_t *, struct bio *);
extern int scsi_cmd_ioctl(struct file *, struct gendisk *, unsigned int, void *);
extern int sg_scsi_ioctl(struct file *, struct request_queue *,
  struct gendisk *, struct scsi_ioctl_command *);
extern void blk_start_queue(request_queue_t *q);
extern void blk_stop_queue(request_queue_t *q);
extern void blk_sync_queue(struct request_queue *q);
extern void __blk_stop_queue(request_queue_t *q);
extern void blk_run_queue(request_queue_t *);
extern void blk_queue_activity_fn(request_queue_t *, activity_fn *, void *);
extern int blk_rq_map_user(request_queue_t *, struct request *, void *, unsigned int);
extern int blk_rq_unmap_user(struct bio *, unsigned int);
extern int blk_rq_map_kern(request_queue_t *, struct request *, void *, unsigned int, gfp_t);
extern int blk_rq_map_user_iov(request_queue_t *, struct request *, struct sg_iovec *, int);
extern int blk_execute_rq(request_queue_t *, struct gendisk *,
     struct request *, int);
extern void blk_execute_rq_nowait(request_queue_t *, struct gendisk *,
      struct request *, int, rq_end_io_fn *);

static inline __attribute__((always_inline)) request_queue_t *bdev_get_queue(struct block_device *bdev)
{
 return bdev->bd_disk->queue;
}

static inline __attribute__((always_inline)) void blk_run_backing_dev(struct backing_dev_info *bdi,
           struct page *page)
{
 if (bdi && bdi->unplug_io_fn)
  bdi->unplug_io_fn(bdi, page);
}

static inline __attribute__((always_inline)) void blk_run_address_space(struct address_space *mapping)
{
 if (mapping)
  blk_run_backing_dev(mapping->backing_dev_info, ((void *)0));
}

extern int end_that_request_first(struct request *, int, int);
extern int end_that_request_chunk(struct request *, int, int);
extern void end_that_request_last(struct request *, int);
extern void end_request(struct request *req, int uptodate);
extern void blk_complete_request(struct request *);

static inline __attribute__((always_inline)) int rq_all_done(struct request *rq, unsigned int nr_bytes)
{
 if (((rq)->flags & (1 << __REQ_CMD)))
  return (nr_bytes >= (rq->hard_nr_sectors << 9));
 else if (((rq)->flags & (1 << __REQ_BLOCK_PC)))
  return nr_bytes >= rq->data_len;

 return 0;
}

static inline __attribute__((always_inline)) void blkdev_dequeue_request(struct request *req)
{
 elv_dequeue_request(req->q, req);
}




static inline __attribute__((always_inline)) void elv_dispatch_add_tail(struct request_queue *q,
      struct request *rq)
{
 if (q->last_merge == rq)
  q->last_merge = ((void *)0);
 q->nr_sorted--;

 q->end_sector = ((rq)->sector + (rq)->nr_sectors);
 q->boundary_rq = rq;
 list_add_tail(&rq->queuelist, &q->queue_head);
}




extern request_queue_t *blk_init_queue_node(request_fn_proc *rfn,
     spinlock_t *lock, int node_id);
extern request_queue_t *blk_init_queue(request_fn_proc *, spinlock_t *);
extern void blk_cleanup_queue(request_queue_t *);
extern void blk_queue_make_request(request_queue_t *, make_request_fn *);
extern void blk_queue_bounce_limit(request_queue_t *, u64);
extern void blk_queue_max_sectors(request_queue_t *, unsigned int);
extern void blk_queue_max_phys_segments(request_queue_t *, unsigned short);
extern void blk_queue_max_hw_segments(request_queue_t *, unsigned short);
extern void blk_queue_max_segment_size(request_queue_t *, unsigned int);
extern void blk_queue_hardsect_size(request_queue_t *, unsigned short);
extern void blk_queue_stack_limits(request_queue_t *t, request_queue_t *b);
extern void blk_queue_segment_boundary(request_queue_t *, unsigned long);
extern void blk_queue_prep_rq(request_queue_t *, prep_rq_fn *pfn);
extern void blk_queue_merge_bvec(request_queue_t *, merge_bvec_fn *);
extern void blk_queue_dma_alignment(request_queue_t *, int);
extern void blk_queue_softirq_done(request_queue_t *, softirq_done_fn *);
extern struct backing_dev_info *blk_get_backing_dev_info(struct block_device *bdev);
extern int blk_queue_ordered(request_queue_t *, unsigned, prepare_flush_fn *);
extern void blk_queue_issue_flush_fn(request_queue_t *, issue_flush_fn *);
extern int blk_do_ordered(request_queue_t *, struct request **);
extern unsigned blk_ordered_cur_seq(request_queue_t *);
extern unsigned blk_ordered_req_seq(struct request *);
extern void blk_ordered_complete_seq(request_queue_t *, unsigned, int);

extern int blk_rq_map_sg(request_queue_t *, struct request *, struct scatterlist *);
extern void blk_dump_rq_flags(struct request *, char *);
extern void generic_unplug_device(request_queue_t *);
extern void __generic_unplug_device(request_queue_t *);
extern long nr_blockdev_pages(void);

int blk_get_queue(request_queue_t *);
request_queue_t *blk_alloc_queue(gfp_t);
request_queue_t *blk_alloc_queue_node(gfp_t, int);
extern void blk_put_queue(request_queue_t *);







extern int blk_queue_start_tag(request_queue_t *, struct request *);
extern struct request *blk_queue_find_tag(request_queue_t *, int);
extern void blk_queue_end_tag(request_queue_t *, struct request *);
extern int blk_queue_init_tags(request_queue_t *, int, struct blk_queue_tag *);
extern void blk_queue_free_tags(request_queue_t *);
extern int blk_queue_resize_tags(request_queue_t *, int);
extern void blk_queue_invalidate_tags(request_queue_t *);
extern long blk_congestion_wait(int rw, long timeout);

extern void blk_rq_bio_prep(request_queue_t *, struct request *, struct bio *);
extern int blkdev_issue_flush(struct block_device *, sector_t *);

static inline __attribute__((always_inline)) int queue_hardsect_size(request_queue_t *q)
{
 int retval = 512;

 if (q && q->hardsect_size)
  retval = q->hardsect_size;

 return retval;
}

static inline __attribute__((always_inline)) int bdev_hardsect_size(struct block_device *bdev)
{
 return queue_hardsect_size(bdev_get_queue(bdev));
}

static inline __attribute__((always_inline)) int queue_dma_alignment(request_queue_t *q)
{
 int retval = 511;

 if (q && q->dma_alignment)
  retval = q->dma_alignment;

 return retval;
}

static inline __attribute__((always_inline)) int bdev_dma_aligment(struct block_device *bdev)
{
 return queue_dma_alignment(bdev_get_queue(bdev));
}





static inline __attribute__((always_inline)) unsigned int blksize_bits(unsigned int size)
{
 unsigned int bits = 8;
 do {
  bits++;
  size >>= 1;
 } while (size > 256);
 return bits;
}

static inline __attribute__((always_inline)) unsigned int block_size(struct block_device *bdev)
{
 return bdev->bd_block_size;
}

typedef struct {struct page *v;} Sector;

unsigned char *read_dev_sector(struct block_device *, sector_t, Sector *);

static inline __attribute__((always_inline)) void put_dev_sector(Sector p)
{
 put_page(p.v);
}

struct work_struct;
int kblockd_schedule_work(struct work_struct *work);
void kblockd_flush(void);














enum {

 ATA_MAX_DEVICES = 2,
 ATA_MAX_PRD = 256,
 ATA_SECT_SIZE = 512,

 ATA_ID_WORDS = 256,
 ATA_ID_SERNO_OFS = 10,
 ATA_ID_FW_REV_OFS = 23,
 ATA_ID_PROD_OFS = 27,
 ATA_ID_OLD_PIO_MODES = 51,
 ATA_ID_FIELD_VALID = 53,
 ATA_ID_MWDMA_MODES = 63,
 ATA_ID_PIO_MODES = 64,
 ATA_ID_EIDE_DMA_MIN = 65,
 ATA_ID_EIDE_PIO = 67,
 ATA_ID_EIDE_PIO_IORDY = 68,
 ATA_ID_UDMA_MODES = 88,
 ATA_ID_MAJOR_VER = 80,
 ATA_ID_PIO4 = (1 << 1),

 ATA_PCI_CTL_OFS = 2,
 ATA_SERNO_LEN = 20,
 ATA_UDMA0 = (1 << 0),
 ATA_UDMA1 = ATA_UDMA0 | (1 << 1),
 ATA_UDMA2 = ATA_UDMA1 | (1 << 2),
 ATA_UDMA3 = ATA_UDMA2 | (1 << 3),
 ATA_UDMA4 = ATA_UDMA3 | (1 << 4),
 ATA_UDMA5 = ATA_UDMA4 | (1 << 5),
 ATA_UDMA6 = ATA_UDMA5 | (1 << 6),
 ATA_UDMA7 = ATA_UDMA6 | (1 << 7),


 ATA_UDMA_MASK_40C = ATA_UDMA2,


 ATA_PRD_SZ = 8,
 ATA_PRD_TBL_SZ = (ATA_MAX_PRD * ATA_PRD_SZ),
 ATA_PRD_EOT = (1 << 31),

 ATA_DMA_TABLE_OFS = 4,
 ATA_DMA_STATUS = 2,
 ATA_DMA_CMD = 0,
 ATA_DMA_WR = (1 << 3),
 ATA_DMA_START = (1 << 0),
 ATA_DMA_INTR = (1 << 2),
 ATA_DMA_ERR = (1 << 1),
 ATA_DMA_ACTIVE = (1 << 0),


 ATA_HOB = (1 << 7),
 ATA_NIEN = (1 << 1),
 ATA_LBA = (1 << 6),
 ATA_DEV1 = (1 << 4),
 ATA_DEVICE_OBS = (1 << 7) | (1 << 5),
 ATA_DEVCTL_OBS = (1 << 3),
 ATA_BUSY = (1 << 7),
 ATA_DRDY = (1 << 6),
 ATA_DF = (1 << 5),
 ATA_DRQ = (1 << 3),
 ATA_ERR = (1 << 0),
 ATA_SRST = (1 << 2),
 ATA_ICRC = (1 << 7),
 ATA_UNC = (1 << 6),
 ATA_IDNF = (1 << 4),
 ATA_ABORTED = (1 << 2),


 ATA_REG_DATA = 0x00,
 ATA_REG_ERR = 0x01,
 ATA_REG_NSECT = 0x02,
 ATA_REG_LBAL = 0x03,
 ATA_REG_LBAM = 0x04,
 ATA_REG_LBAH = 0x05,
 ATA_REG_DEVICE = 0x06,
 ATA_REG_STATUS = 0x07,

 ATA_REG_FEATURE = ATA_REG_ERR,
 ATA_REG_CMD = ATA_REG_STATUS,
 ATA_REG_BYTEL = ATA_REG_LBAM,
 ATA_REG_BYTEH = ATA_REG_LBAH,
 ATA_REG_DEVSEL = ATA_REG_DEVICE,
 ATA_REG_IRQ = ATA_REG_NSECT,


 ATA_CMD_CHK_POWER = 0xE5,
 ATA_CMD_STANDBY = 0xE2,
 ATA_CMD_IDLE = 0xE3,
 ATA_CMD_EDD = 0x90,
 ATA_CMD_FLUSH = 0xE7,
 ATA_CMD_FLUSH_EXT = 0xEA,
 ATA_CMD_ID_ATA = 0xEC,
 ATA_CMD_ID_ATAPI = 0xA1,
 ATA_CMD_READ = 0xC8,
 ATA_CMD_READ_EXT = 0x25,
 ATA_CMD_WRITE = 0xCA,
 ATA_CMD_WRITE_EXT = 0x35,
 ATA_CMD_WRITE_FUA_EXT = 0x3D,
 ATA_CMD_FPDMA_READ = 0x60,
 ATA_CMD_FPDMA_WRITE = 0x61,
 ATA_CMD_PIO_READ = 0x20,
 ATA_CMD_PIO_READ_EXT = 0x24,
 ATA_CMD_PIO_WRITE = 0x30,
 ATA_CMD_PIO_WRITE_EXT = 0x34,
 ATA_CMD_READ_MULTI = 0xC4,
 ATA_CMD_READ_MULTI_EXT = 0x29,
 ATA_CMD_WRITE_MULTI = 0xC5,
 ATA_CMD_WRITE_MULTI_EXT = 0x39,
 ATA_CMD_WRITE_MULTI_FUA_EXT = 0xCE,
 ATA_CMD_SET_FEATURES = 0xEF,
 ATA_CMD_PACKET = 0xA0,
 ATA_CMD_VERIFY = 0x40,
 ATA_CMD_VERIFY_EXT = 0x42,
  ATA_CMD_STANDBYNOW1 = 0xE0,
  ATA_CMD_IDLEIMMEDIATE = 0xE1,
 ATA_CMD_INIT_DEV_PARAMS = 0x91,
 ATA_CMD_READ_NATIVE_MAX = 0xF8,
 ATA_CMD_READ_NATIVE_MAX_EXT = 0x27,
 ATA_CMD_READ_LOG_EXT = 0x2f,


 ATA_LOG_SATA_NCQ = 0x10,


 SETFEATURES_XFER = 0x03,
 XFER_UDMA_7 = 0x47,
 XFER_UDMA_6 = 0x46,
 XFER_UDMA_5 = 0x45,
 XFER_UDMA_4 = 0x44,
 XFER_UDMA_3 = 0x43,
 XFER_UDMA_2 = 0x42,
 XFER_UDMA_1 = 0x41,
 XFER_UDMA_0 = 0x40,
 XFER_MW_DMA_2 = 0x22,
 XFER_MW_DMA_1 = 0x21,
 XFER_MW_DMA_0 = 0x20,
 XFER_SW_DMA_2 = 0x12,
 XFER_SW_DMA_1 = 0x11,
 XFER_SW_DMA_0 = 0x10,
 XFER_PIO_4 = 0x0C,
 XFER_PIO_3 = 0x0B,
 XFER_PIO_2 = 0x0A,
 XFER_PIO_1 = 0x09,
 XFER_PIO_0 = 0x08,
 XFER_PIO_SLOW = 0x00,

 SETFEATURES_WC_ON = 0x02,
 SETFEATURES_WC_OFF = 0x82,


 ATAPI_PKT_DMA = (1 << 0),
 ATAPI_DMADIR = (1 << 2),

 ATAPI_CDB_LEN = 16,


 ATA_CBL_NONE = 0,
 ATA_CBL_PATA40 = 1,
 ATA_CBL_PATA80 = 2,
 ATA_CBL_PATA_UNK = 3,
 ATA_CBL_SATA = 4,


 SCR_STATUS = 0,
 SCR_ERROR = 1,
 SCR_CONTROL = 2,
 SCR_ACTIVE = 3,
 SCR_NOTIFICATION = 4,


 SERR_DATA_RECOVERED = (1 << 0),
 SERR_COMM_RECOVERED = (1 << 1),
 SERR_DATA = (1 << 8),
 SERR_PERSISTENT = (1 << 9),
 SERR_PROTOCOL = (1 << 10),
 SERR_INTERNAL = (1 << 11),
 SERR_PHYRDY_CHG = (1 << 16),
 SERR_DEV_XCHG = (1 << 26),


 ATA_TFLAG_LBA48 = (1 << 0),
 ATA_TFLAG_ISADDR = (1 << 1),
 ATA_TFLAG_DEVICE = (1 << 2),
 ATA_TFLAG_WRITE = (1 << 3),
 ATA_TFLAG_LBA = (1 << 4),
 ATA_TFLAG_FUA = (1 << 5),
 ATA_TFLAG_POLLING = (1 << 6),
};

enum ata_tf_protocols {

 ATA_PROT_UNKNOWN,
 ATA_PROT_NODATA,
 ATA_PROT_PIO,
 ATA_PROT_DMA,
 ATA_PROT_NCQ,
 ATA_PROT_ATAPI,
 ATA_PROT_ATAPI_NODATA,
 ATA_PROT_ATAPI_DMA,
};

enum ata_ioctls {
 ATA_IOC_GET_IO32 = 0x309,
 ATA_IOC_SET_IO32 = 0x324,
};



struct ata_prd {
 u32 addr;
 u32 flags_len;
};

struct ata_taskfile {
 unsigned long flags;
 u8 protocol;

 u8 ctl;

 u8 hob_feature;
 u8 hob_nsect;
 u8 hob_lbal;
 u8 hob_lbam;
 u8 hob_lbah;

 u8 feature;
 u8 nsect;
 u8 lbal;
 u8 lbam;
 u8 lbah;

 u8 device;

 u8 command;
};

static inline __attribute__((always_inline)) unsigned int ata_id_major_version(const u16 *id)
{
 unsigned int mver;

 for (mver = 14; mver >= 1; mver--)
  if (id[ATA_ID_MAJOR_VER] & (1 << mver))
   break;
 return mver;
}

static inline __attribute__((always_inline)) int ata_id_current_chs_valid(const u16 *id)
{



 return (id[53] & 0x01) &&
  id[54] &&
  id[55] &&
  id[55] <= 16 &&
  id[56];
}

static inline __attribute__((always_inline)) int atapi_cdb_len(const u16 *dev_id)
{
 u16 tmp = dev_id[0] & 0x3;
 switch (tmp) {
 case 0: return 12;
 case 1: return 16;
 default: return -1;
 }
}

static inline __attribute__((always_inline)) int is_atapi_taskfile(const struct ata_taskfile *tf)
{
 return (tf->protocol == ATA_PROT_ATAPI) ||
        (tf->protocol == ATA_PROT_ATAPI_NODATA) ||
        (tf->protocol == ATA_PROT_ATAPI_DMA);
}

static inline __attribute__((always_inline)) int is_multi_taskfile(struct ata_taskfile *tf)
{
 return (tf->command == ATA_CMD_READ_MULTI) ||
        (tf->command == ATA_CMD_WRITE_MULTI) ||
        (tf->command == ATA_CMD_READ_MULTI_EXT) ||
        (tf->command == ATA_CMD_WRITE_MULTI_EXT) ||
        (tf->command == ATA_CMD_WRITE_MULTI_FUA_EXT);
}

static inline __attribute__((always_inline)) int ata_ok(u8 status)
{
 return ((status & (ATA_BUSY | ATA_DRDY | ATA_DF | ATA_DRQ | ATA_ERR))
   == ATA_DRDY);
}

static inline __attribute__((always_inline)) int lba_28_ok(u64 block, u32 n_block)
{

 return ((block + n_block - 1) < ((u64)1 << 28)) && (n_block <= 256);
}

static inline __attribute__((always_inline)) int lba_48_ok(u64 block, u32 n_block)
{

 return ((block + n_block - 1) < ((u64)1 << 48)) && (n_block <= 65536);
}


typedef unsigned char task_ioreg_t;
typedef unsigned long sata_ioreg_t;

typedef union ide_reg_valid_s {
 unsigned all : 16;
 struct {
  unsigned data : 1;
  unsigned error_feature : 1;
  unsigned sector : 1;
  unsigned nsector : 1;
  unsigned lcyl : 1;
  unsigned hcyl : 1;
  unsigned select : 1;
  unsigned status_command : 1;

  unsigned data_hob : 1;
  unsigned error_feature_hob : 1;
  unsigned sector_hob : 1;
  unsigned nsector_hob : 1;
  unsigned lcyl_hob : 1;
  unsigned hcyl_hob : 1;
  unsigned select_hob : 1;
  unsigned control_hob : 1;
 } b;
} ide_reg_valid_t;

typedef struct ide_task_request_s {
 task_ioreg_t io_ports[8];
 task_ioreg_t hob_ports[8];
 ide_reg_valid_t out_flags;
 ide_reg_valid_t in_flags;
 int data_phase;
 int req_cmd;
 unsigned long out_size;
 unsigned long in_size;
} ide_task_request_t;

typedef struct ide_ioctl_request_s {
 ide_task_request_t *task_request;
 unsigned char *out_buffer;
 unsigned char *in_buffer;
} ide_ioctl_request_t;

struct hd_drive_cmd_hdr {
 task_ioreg_t command;
 task_ioreg_t sector_number;
 task_ioreg_t feature;
 task_ioreg_t sector_count;
};

typedef struct hd_drive_task_hdr {
 task_ioreg_t data;
 task_ioreg_t feature;
 task_ioreg_t sector_count;
 task_ioreg_t sector_number;
 task_ioreg_t low_cylinder;
 task_ioreg_t high_cylinder;
 task_ioreg_t device_head;
 task_ioreg_t command;
} task_struct_t;

typedef struct hd_drive_hob_hdr {
 task_ioreg_t data;
 task_ioreg_t feature;
 task_ioreg_t sector_count;
 task_ioreg_t sector_number;
 task_ioreg_t low_cylinder;
 task_ioreg_t high_cylinder;
 task_ioreg_t device_head;
 task_ioreg_t control;
} hob_struct_t;

struct hd_geometry {
      unsigned char heads;
      unsigned char sectors;
      unsigned short cylinders;
      unsigned long start;
};

enum {
 BUSSTATE_OFF = 0,
 BUSSTATE_ON,
 BUSSTATE_TRISTATE
};

struct hd_driveid {
 unsigned short config;
 unsigned short cyls;
 unsigned short reserved2;
 unsigned short heads;
 unsigned short track_bytes;
 unsigned short sector_bytes;
 unsigned short sectors;
 unsigned short vendor0;
 unsigned short vendor1;
 unsigned short vendor2;
 unsigned char serial_no[20];
 unsigned short buf_type;
 unsigned short buf_size;


 unsigned short ecc_bytes;
 unsigned char fw_rev[8];
 unsigned char model[40];
 unsigned char max_multsect;
 unsigned char vendor3;
 unsigned short dword_io;
 unsigned char vendor4;
 unsigned char capability;





 unsigned short reserved50;
 unsigned char vendor5;
 unsigned char tPIO;
 unsigned char vendor6;
 unsigned char tDMA;
 unsigned short field_valid;




 unsigned short cur_cyls;
 unsigned short cur_heads;
 unsigned short cur_sectors;
 unsigned short cur_capacity0;
 unsigned short cur_capacity1;
 unsigned char multsect;
 unsigned char multsect_valid;
 unsigned int lba_capacity;
 unsigned short dma_1word;
 unsigned short dma_mword;
 unsigned short eide_pio_modes;
 unsigned short eide_dma_min;
 unsigned short eide_dma_time;
 unsigned short eide_pio;
 unsigned short eide_pio_iordy;
 unsigned short words69_70[2];



 unsigned short words71_74[4];


 unsigned short queue_depth;



 unsigned short words76_79[4];
 unsigned short major_rev_num;
 unsigned short minor_rev_num;
 unsigned short command_set_1;

 unsigned short command_set_2;

 unsigned short cfsse;

 unsigned short cfs_enable_1;

 unsigned short cfs_enable_2;

 unsigned short csf_default;

 unsigned short dma_ultra;
 unsigned short trseuc;
 unsigned short trsEuc;
 unsigned short CurAPMvalues;
 unsigned short mprc;
 unsigned short hw_config;

 unsigned short acoustic;



 unsigned short msrqs;
 unsigned short sxfert;
 unsigned short sal;
 unsigned int spg;
 unsigned long long lba_capacity_2;
 unsigned short words104_125[22];
 unsigned short last_lun;
 unsigned short word127;







 unsigned short dlf;

 unsigned short csfo;







 unsigned short words130_155[26];
 unsigned short word156;
 unsigned short words157_159[3];
 unsigned short cfa_power;






 unsigned short words161_175[15];
 unsigned short words176_205[30];
 unsigned short words206_254[49];
 unsigned short integrity_word;



};



enum {
 PROC_ROOT_INO = 1,
};

typedef int (read_proc_t)(char *page, char **start, off_t off,
     int count, int *eof, void *data);
typedef int (write_proc_t)(struct file *file, const char *buffer,
      unsigned long count, void *data);
typedef int (get_info_t)(char *, char **, off_t, int);

struct proc_dir_entry {
 unsigned int low_ino;
 unsigned short namelen;
 const char *name;
 mode_t mode;
 nlink_t nlink;
 uid_t uid;
 gid_t gid;
 loff_t size;
 struct inode_operations * proc_iops;
 const struct file_operations * proc_fops;
 get_info_t *get_info;
 struct module *owner;
 struct proc_dir_entry *next, *parent, *subdir;
 void *data;
 read_proc_t *read_proc;
 write_proc_t *write_proc;
 atomic_t count;
 int deleted;
 void *set;
};

struct kcore_list {
 struct kcore_list *next;
 unsigned long addr;
 size_t size;
};

struct vmcore {
 struct list_head list;
 unsigned long long paddr;
 unsigned long long size;
 loff_t offset;
};



extern struct proc_dir_entry proc_root;
extern struct proc_dir_entry *proc_root_fs;
extern struct proc_dir_entry *proc_net;
extern struct proc_dir_entry *proc_net_stat;
extern struct proc_dir_entry *proc_bus;
extern struct proc_dir_entry *proc_root_driver;
extern struct proc_dir_entry *proc_root_kcore;

extern spinlock_t proc_subdir_lock;

extern void proc_root_init(void);
extern void proc_misc_init(void);

struct mm_struct;

void proc_flush_task(struct task_struct *task);
struct dentry *proc_pid_lookup(struct inode *dir, struct dentry * dentry, struct nameidata *);
int proc_pid_readdir(struct file * filp, void * dirent, filldir_t filldir);
unsigned long task_vsize(struct mm_struct *);
int task_statm(struct mm_struct *, int *, int *, int *, int *);
char *task_mem(struct mm_struct *, char *);

extern struct proc_dir_entry *create_proc_entry(const char *name, mode_t mode,
      struct proc_dir_entry *parent);
extern void remove_proc_entry(const char *name, struct proc_dir_entry *parent);

extern struct vfsmount *proc_mnt;
extern int proc_fill_super(struct super_block *,void *,int);
extern struct inode *proc_get_inode(struct super_block *, unsigned int, struct proc_dir_entry *);

extern int proc_match(int, const char *,struct proc_dir_entry *);

extern int proc_readdir(struct file *, void *, filldir_t);
extern struct dentry *proc_lookup(struct inode *, struct dentry *, struct nameidata *);

extern const struct file_operations proc_kcore_operations;
extern const struct file_operations proc_kmsg_operations;
extern const struct file_operations ppc_htab_operations;




struct tty_driver;
extern void proc_tty_init(void);
extern void proc_tty_register_driver(struct tty_driver *driver);
extern void proc_tty_unregister_driver(struct tty_driver *driver);

extern struct proc_dir_entry *proc_symlink(const char *,
  struct proc_dir_entry *, const char *);
extern struct proc_dir_entry *proc_mkdir(const char *,struct proc_dir_entry *);
extern struct proc_dir_entry *proc_mkdir_mode(const char *name, mode_t mode,
   struct proc_dir_entry *parent);

static inline __attribute__((always_inline)) struct proc_dir_entry *create_proc_read_entry(const char *name,
 mode_t mode, struct proc_dir_entry *base,
 read_proc_t *read_proc, void * data)
{
 struct proc_dir_entry *res=create_proc_entry(name,mode,base);
 if (res) {
  res->read_proc=read_proc;
  res->data=data;
 }
 return res;
}

static inline __attribute__((always_inline)) struct proc_dir_entry *create_proc_info_entry(const char *name,
 mode_t mode, struct proc_dir_entry *base, get_info_t *get_info)
{
 struct proc_dir_entry *res=create_proc_entry(name,mode,base);
 if (res) res->get_info=get_info;
 return res;
}

static inline __attribute__((always_inline)) struct proc_dir_entry *proc_net_create(const char *name,
 mode_t mode, get_info_t *get_info)
{
 return create_proc_info_entry(name,mode,proc_net,get_info);
}

static inline __attribute__((always_inline)) struct proc_dir_entry *proc_net_fops_create(const char *name,
 mode_t mode, const struct file_operations *fops)
{
 struct proc_dir_entry *res = create_proc_entry(name, mode, proc_net);
 if (res)
  res->proc_fops = fops;
 return res;
}

static inline __attribute__((always_inline)) void proc_net_remove(const char *name)
{
 remove_proc_entry(name,proc_net);
}

extern void kclist_add(struct kcore_list *, void *, size_t);


struct proc_inode {
 struct pid *pid;
 int fd;
 union {
  int (*proc_get_link)(struct inode *, struct dentry **, struct vfsmount **);
  int (*proc_read)(struct task_struct *task, char *page);
 } op;
 struct proc_dir_entry *pde;
 struct inode vfs_inode;
};

static inline __attribute__((always_inline)) struct proc_inode *PROC_I(const struct inode *inode)
{
 return ({ const typeof( ((struct proc_inode *)0)->vfs_inode ) *__mptr = (inode); (struct proc_inode *)( (char *)__mptr - __builtin_offsetof(struct proc_inode,vfs_inode) );});
}

static inline __attribute__((always_inline)) struct proc_dir_entry *PDE(const struct inode *inode)
{
 return PROC_I(inode)->pde;
}

struct proc_maps_private {
 struct pid *pid;
 struct task_struct *task;
 struct vm_area_struct *tail_vma;
};



struct seq_operations;
struct file;
struct vfsmount;
struct dentry;
struct inode;

struct seq_file {
 char *buf;
 size_t size;
 size_t from;
 size_t count;
 loff_t index;
 loff_t version;
 struct mutex lock;
 struct seq_operations *op;
 void *private;
};

struct seq_operations {
 void * (*start) (struct seq_file *m, loff_t *pos);
 void (*stop) (struct seq_file *m, void *v);
 void * (*next) (struct seq_file *m, void *v, loff_t *pos);
 int (*show) (struct seq_file *m, void *v);
};

int seq_open(struct file *, struct seq_operations *);
ssize_t seq_read(struct file *, char *, size_t, loff_t *);
loff_t seq_lseek(struct file *, loff_t, int);
int seq_release(struct inode *, struct file *);
int seq_escape(struct seq_file *, const char *, const char *);
int seq_putc(struct seq_file *m, char c);
int seq_puts(struct seq_file *m, const char *s);

int seq_printf(struct seq_file *, const char *, ...)
 __attribute__ ((format (printf,2,3)));

int seq_path(struct seq_file *, struct vfsmount *, struct dentry *, char *);

int single_open(struct file *, int (*)(struct seq_file *, void *), void *);
int single_release(struct inode *, struct file *);
int seq_release_private(struct inode *, struct file *);




extern unsigned long loops_per_jiffy;



extern void __bad_udelay(void);
extern void __bad_ndelay(void);

extern void __udelay(unsigned long usecs);
extern void __ndelay(unsigned long nsecs);
extern void __const_udelay(unsigned long usecs);
extern void __delay(unsigned long loops);

void use_tsc_delay(void);


void calibrate_delay(void);
void msleep(unsigned int msecs);
unsigned long msleep_interruptible(unsigned int msecs);

static inline __attribute__((always_inline)) void ssleep(unsigned int seconds)
{
 msleep(seconds * 1000);
}









struct in6_addr
{
 union
 {
  __u8 u6_addr8[16];
  __u16 u6_addr16[8];
  __u32 u6_addr32[4];
 } in6_u;



};

extern const struct in6_addr in6addr_loopback;


struct sockaddr_in6 {
 unsigned short int sin6_family;
 __u16 sin6_port;
 __u32 sin6_flowinfo;
 struct in6_addr sin6_addr;
 __u32 sin6_scope_id;
};

struct ipv6_mreq {

 struct in6_addr ipv6mr_multiaddr;


 int ipv6mr_ifindex;
};



struct in6_flowlabel_req
{
 struct in6_addr flr_dst;
 __u32 flr_label;
 __u8 flr_action;
 __u8 flr_share;
 __u16 flr_flags;
 __u16 flr_expires;
 __u16 flr_linger;
 __u32 __flr_pad;

};


 __attribute__((regparm(0))) unsigned int csum_partial(const unsigned char * buff, int len, unsigned int sum);

 __attribute__((regparm(0))) unsigned int csum_partial_copy_generic(const unsigned char *src, unsigned char *dst,
        int len, int sum, int *src_err_ptr, int *dst_err_ptr);

static __inline__ __attribute__((always_inline))
unsigned int csum_partial_copy_nocheck (const unsigned char *src, unsigned char *dst,
     int len, int sum)
{
 return csum_partial_copy_generic ( src, dst, len, sum, ((void *)0), ((void *)0));
}

static __inline__ __attribute__((always_inline))
unsigned int csum_partial_copy_from_user(const unsigned char *src, unsigned char *dst,
      int len, int sum, int *err_ptr)
{
 do { __might_sleep("include/asm/checksum.h", 51); cond_resched(); } while (0);
 return csum_partial_copy_generic(( unsigned char *)src, dst,
     len, sum, err_ptr, ((void *)0));
}

static inline __attribute__((always_inline)) unsigned short ip_fast_csum(unsigned char * iph,
       unsigned int ihl)
{
 unsigned int sum;

 __asm__ __volatile__(
     "movl (%1), %0	;\n"
     "subl $4, %2	;\n"
     "jbe 2f		;\n"
     "addl 4(%1), %0	;\n"
     "adcl 8(%1), %0	;\n"
     "adcl 12(%1), %0	;\n"
"1:	    adcl 16(%1), %0	;\n"
     "lea 4(%1), %1	;\n"
     "decl %2		;\n"
     "jne 1b		;\n"
     "adcl $0, %0	;\n"
     "movl %0, %2	;\n"
     "shrl $16, %0	;\n"
     "addw %w2, %w0	;\n"
     "adcl $0, %0	;\n"
     "notl %0		;\n"
"2:				;\n"



 : "=r" (sum), "=r" (iph), "=r" (ihl)
 : "1" (iph), "2" (ihl)
 : "memory");
 return(sum);
}





static inline __attribute__((always_inline)) unsigned int csum_fold(unsigned int sum)
{
 __asm__(
  "addl %1, %0		;\n"
  "adcl $0xffff, %0	;\n"
  : "=r" (sum)
  : "r" (sum << 16), "0" (sum & 0xffff0000)
 );
 return (~sum) >> 16;
}

static inline __attribute__((always_inline)) unsigned long csum_tcpudp_nofold(unsigned long saddr,
         unsigned long daddr,
         unsigned short len,
         unsigned short proto,
         unsigned int sum)
{
    __asm__(
 "addl %1, %0	;\n"
 "adcl %2, %0	;\n"
 "adcl %3, %0	;\n"
 "adcl $0, %0	;\n"
 : "=r" (sum)
 : "g" (daddr), "g"(saddr), "g"(((__builtin_constant_p((__u16)(( __u16)(__be16)(len))) ? ({ __u16 __x = ((( __u16)(__be16)(len))); ((__u16)( (((__u16)(__x) & (__u16)0x00ffU) << 8) | (((__u16)(__x) & (__u16)0xff00U) >> 8) )); }) : __fswab16((( __u16)(__be16)(len))))<<16)+proto*256), "0"(sum));
    return sum;
}





static inline __attribute__((always_inline)) unsigned short int csum_tcpudp_magic(unsigned long saddr,
         unsigned long daddr,
         unsigned short len,
         unsigned short proto,
         unsigned int sum)
{
 return csum_fold(csum_tcpudp_nofold(saddr,daddr,len,proto,sum));
}






static inline __attribute__((always_inline)) unsigned short ip_compute_csum(unsigned char * buff, int len)
{
    return csum_fold (csum_partial(buff, len, 0));
}


static __inline__ __attribute__((always_inline)) unsigned short int csum_ipv6_magic(struct in6_addr *saddr,
           struct in6_addr *daddr,
           __u32 len,
           unsigned short proto,
           unsigned int sum)
{
 __asm__(
  "addl 0(%1), %0		;\n"
  "adcl 4(%1), %0		;\n"
  "adcl 8(%1), %0		;\n"
  "adcl 12(%1), %0	;\n"
  "adcl 0(%2), %0		;\n"
  "adcl 4(%2), %0		;\n"
  "adcl 8(%2), %0		;\n"
  "adcl 12(%2), %0	;\n"
  "adcl %3, %0		;\n"
  "adcl %4, %0		;\n"
  "adcl $0, %0		;\n"
  : "=&r" (sum)
  : "r" (saddr), "r" (daddr),
    "r"((( __be32)(__builtin_constant_p((__u32)((len))) ? ({ __u32 __x = (((len))); ((__u32)( (((__u32)(__x) & (__u32)0x000000ffUL) << 24) | (((__u32)(__x) & (__u32)0x0000ff00UL) << 8) | (((__u32)(__x) & (__u32)0x00ff0000UL) >> 8) | (((__u32)(__x) & (__u32)0xff000000UL) >> 24) )); }) : __fswab32(((len)))))), "r"((( __be32)(__builtin_constant_p((__u32)((proto))) ? ({ __u32 __x = (((proto))); ((__u32)( (((__u32)(__x) & (__u32)0x000000ffUL) << 24) | (((__u32)(__x) & (__u32)0x0000ff00UL) << 8) | (((__u32)(__x) & (__u32)0x00ff0000UL) >> 8) | (((__u32)(__x) & (__u32)0xff000000UL) >> 24) )); }) : __fswab32(((proto)))))), "0"(sum));

 return csum_fold(sum);
}





static __inline__ __attribute__((always_inline)) unsigned int csum_and_copy_to_user(const unsigned char *src,
           unsigned char *dst,
           int len, int sum,
           int *err_ptr)
{
 do { __might_sleep("include/asm/checksum.h", 184); cond_resched(); } while (0);
 if ((__builtin_expect(!!(({ unsigned long flag,sum; (void)0; asm("addl %3,%1 ; sbbl %0,%0; cmpl %1,%4; sbbl $0,%0" :"=&r" (flag), "=r" (sum) :"1" (dst),"g" ((int)(len)),"rm" (current_thread_info()->addr_limit.seg)); flag; }) == 0), 1)))
  return csum_partial_copy_generic(src, ( unsigned char *)dst, len, sum, ((void *)0), err_ptr);

 if (len)
  *err_ptr = -14;

 return -1;
}



static inline __attribute__((always_inline))
unsigned int csum_and_copy_from_user (const unsigned char *src, unsigned char *dst,
          int len, int sum, int *err_ptr)
{
 if ((__builtin_expect(!!(({ unsigned long flag,sum; (void)0; asm("addl %3,%1 ; sbbl %0,%0; cmpl %1,%4; sbbl $0,%0" :"=&r" (flag), "=r" (sum) :"1" (src),"g" ((int)(len)),"rm" (current_thread_info()->addr_limit.seg)); flag; }) == 0), 1)))
  return csum_partial_copy_from_user(src, dst, len, sum, err_ptr);

 if (len)
  *err_ptr = -14;

 return sum;
}

static inline __attribute__((always_inline)) unsigned int csum_add(unsigned int csum, unsigned int addend)
{
 csum += addend;
 return csum + (csum < addend);
}

static inline __attribute__((always_inline)) unsigned int csum_sub(unsigned int csum, unsigned int addend)
{
 return csum_add(csum, ~addend);
}

static inline __attribute__((always_inline)) unsigned int
csum_block_add(unsigned int csum, unsigned int csum2, int offset)
{
 if (offset&1)
  csum2 = ((csum2&0xFF00FF)<<8)+((csum2>>8)&0xFF00FF);
 return csum_add(csum, csum2);
}

static inline __attribute__((always_inline)) unsigned int
csum_block_sub(unsigned int csum, unsigned int csum2, int offset)
{
 if (offset&1)
  csum2 = ((csum2&0xFF00FF)<<8)+((csum2>>8)&0xFF00FF);
 return csum_sub(csum, csum2);
}



struct rand_pool_info {
 int entropy_count;
 int buf_size;
 __u32 buf[0];
};





extern void rand_initialize_irq(int irq);

extern void add_input_randomness(unsigned int type, unsigned int code,
     unsigned int value);
extern void add_interrupt_randomness(int irq);

extern void get_random_bytes(void *buf, int nbytes);
void generate_random_uuid(unsigned char uuid_out[16]);

extern __u32 secure_ip_id(__u32 daddr);
extern u32 secure_ipv4_port_ephemeral(__u32 saddr, __u32 daddr, __u16 dport);
extern u32 secure_ipv6_port_ephemeral(const __u32 *saddr, const __u32 *daddr,
          __u16 dport);
extern __u32 secure_tcp_sequence_number(__u32 saddr, __u32 daddr,
     __u16 sport, __u16 dport);
extern __u32 secure_tcpv6_sequence_number(__u32 *saddr, __u32 *daddr,
       __u16 sport, __u16 dport);
extern u64 secure_dccp_sequence_number(__u32 saddr, __u32 daddr,
           __u16 sport, __u16 dport);


extern struct file_operations random_fops, urandom_fops;


unsigned int get_random_int(void);
unsigned long randomize_range(unsigned long start, unsigned long end, unsigned long len);



struct cpu_usage_stat {
 cputime64_t user;
 cputime64_t nice;
 cputime64_t system;
 cputime64_t softirq;
 cputime64_t irq;
 cputime64_t idle;
 cputime64_t iowait;
 cputime64_t steal;
};

struct kernel_stat {
 struct cpu_usage_stat cpustat;
 unsigned int irqs[16];
};

extern __typeof__(struct kernel_stat) per_cpu__kstat;





extern unsigned long long nr_context_switches(void);




static inline __attribute__((always_inline)) int kstat_irqs(int irq)
{
 int cpu, sum = 0;

 for (((cpu)) = 0; ((cpu)) < 1; ((cpu))++, (void)cpu_possible_map)
  sum += (*((void)(cpu), &per_cpu__kstat)).irqs[irq];

 return sum;
}

extern void account_user_time(struct task_struct *, cputime_t);
extern void account_system_time(struct task_struct *, int, cputime_t);
extern void account_steal_time(struct task_struct *, cputime_t);







extern int register_reboot_notifier(struct notifier_block *);
extern int unregister_reboot_notifier(struct notifier_block *);






extern void machine_restart(char *cmd);
extern void machine_halt(void);
extern void machine_power_off(void);

extern void machine_shutdown(void);
struct pt_regs;
extern void machine_crash_shutdown(struct pt_regs *);





extern void kernel_shutdown_prepare(enum system_states state);

extern void kernel_restart(char *cmd);
extern void kernel_halt(void);
extern void kernel_power_off(void);

void ctrl_alt_del(void);





extern void emergency_restart(void);




extern void machine_emergency_restart(void);





struct blkpg_ioctl_arg {
        int op;
        int flags;
        int datalen;
        void *data;
};

struct blkpg_partition {
 long long start;
 long long length;
 int pno;
 char devname[64];

 char volname[64];
};




typedef struct mdp_device_descriptor_s {
 __u32 number;
 __u32 major;
 __u32 minor;
 __u32 raid_disk;
 __u32 state;
 __u32 reserved[32 - 5];
} mdp_disk_t;

typedef struct mdp_superblock_s {



 __u32 md_magic;
 __u32 major_version;
 __u32 minor_version;
 __u32 patch_version;
 __u32 gvalid_words;
 __u32 set_uuid0;
 __u32 ctime;
 __u32 level;
 __u32 size;
 __u32 nr_disks;
 __u32 raid_disks;
 __u32 md_minor;
 __u32 not_persistent;
 __u32 set_uuid1;
 __u32 set_uuid2;
 __u32 set_uuid3;
 __u32 gstate_creserved[32 - 16];




 __u32 utime;
 __u32 state;
 __u32 active_disks;
 __u32 working_disks;
 __u32 failed_disks;
 __u32 spare_disks;
 __u32 sb_csum;






 __u32 events_lo;
 __u32 events_hi;
 __u32 cp_events_lo;
 __u32 cp_events_hi;

 __u32 recovery_cp;

 __u64 reshape_position;
 __u32 new_level;
 __u32 delta_disks;
 __u32 new_layout;
 __u32 new_chunk;
 __u32 gstate_sreserved[32 - 18];




 __u32 layout;
 __u32 chunk_size;
 __u32 root_pv;
 __u32 root_block;
 __u32 pstate_reserved[64 - 4];




 mdp_disk_t disks[27];




 __u32 reserved[(1024 - (32 + 32) - 64 - (27*32) - 32)];




 mdp_disk_t this_disk;

} mdp_super_t;

static inline __attribute__((always_inline)) __u64 md_event(mdp_super_t *sb) {
 __u64 ev = sb->events_hi;
 return (ev<<32)| sb->events_lo;
}

struct mdp_superblock_1 {

 __u32 magic;
 __u32 major_version;
 __u32 feature_map;
 __u32 pad0;

 __u8 set_uuid[16];
 char set_name[32];

 __u64 ctime;
 __u32 level;
 __u32 layout;
 __u64 size;

 __u32 chunksize;
 __u32 raid_disks;
 __u32 bitmap_offset;





 __u32 new_level;
 __u64 reshape_position;
 __u32 delta_disks;
 __u32 new_layout;
 __u32 new_chunk;
 __u8 pad1[128-124];


 __u64 data_offset;
 __u64 data_size;
 __u64 super_offset;
 __u64 recovery_offset;
 __u32 dev_number;
 __u32 cnt_corrected_read;
 __u8 device_uuid[16];
 __u8 devflags;

 __u8 pad2[64-57];


 __u64 utime;
 __u64 events;
 __u64 resync_offset;
 __u32 sb_csum;
 __u32 max_dev;
 __u8 pad3[64-32];







 __u16 dev_roles[0];
};



typedef struct mdu_version_s {
 int major;
 int minor;
 int patchlevel;
} mdu_version_t;

typedef struct mdu_array_info_s {



 int major_version;
 int minor_version;
 int patch_version;
 int ctime;
 int level;
 int size;
 int nr_disks;
 int raid_disks;
 int md_minor;
 int not_persistent;




 int utime;
 int state;
 int active_disks;
 int working_disks;
 int failed_disks;
 int spare_disks;




 int layout;
 int chunk_size;

} mdu_array_info_t;

typedef struct mdu_disk_info_s {



 int number;
 int major;
 int minor;
 int raid_disk;
 int state;

} mdu_disk_info_t;

typedef struct mdu_start_info_s {



 int major;
 int minor;
 int raid_disk;
 int state;

} mdu_start_info_t;

typedef struct mdu_bitmap_file_s
{
 char pathname[4096];
} mdu_bitmap_file_t;

typedef struct mdu_param_s
{
 int personality;
 int chunk_size;
 int max_fault;
} mdu_param_t;





struct bio_list {
 struct bio *head;
 struct bio *tail;
};

static inline __attribute__((always_inline)) void bio_list_init(struct bio_list *bl)
{
 bl->head = bl->tail = ((void *)0);
}

static inline __attribute__((always_inline)) void bio_list_add(struct bio_list *bl, struct bio *bio)
{
 bio->bi_next = ((void *)0);

 if (bl->tail)
  bl->tail->bi_next = bio;
 else
  bl->head = bio;

 bl->tail = bio;
}

static inline __attribute__((always_inline)) void bio_list_merge(struct bio_list *bl, struct bio_list *bl2)
{
 if (!bl2->head)
  return;

 if (bl->tail)
  bl->tail->bi_next = bl2->head;
 else
  bl->head = bl2->head;

 bl->tail = bl2->tail;
}

static inline __attribute__((always_inline)) struct bio *bio_list_pop(struct bio_list *bl)
{
 struct bio *bio = bl->head;

 if (bio) {
  bl->head = bl->head->bi_next;
  if (!bl->head)
   bl->tail = ((void *)0);

  bio->bi_next = ((void *)0);
 }

 return bio;
}

static inline __attribute__((always_inline)) struct bio *bio_list_get(struct bio_list *bl)
{
 struct bio *bio = bl->head;

 bl->head = bl->tail = ((void *)0);

 return bio;
}


typedef struct mddev_s mddev_t;
typedef struct mdk_rdev_s mdk_rdev_t;

struct mdk_rdev_s
{
 struct list_head same_set;

 sector_t size;
 mddev_t *mddev;
 unsigned long last_events;

 struct block_device *bdev;

 struct page *sb_page;
 int sb_loaded;
 __u64 sb_events;
 sector_t data_offset;
 sector_t sb_offset;
 int sb_size;
 int preferred_minor;

 struct kobject kobj;

 unsigned long flags;





 int desc_nr;
 int raid_disk;
 int saved_raid_disk;



 sector_t recovery_offset;




 atomic_t nr_pending;



 atomic_t read_errors;


 atomic_t corrected_errors;



};

struct mddev_s
{
 void *private;
 struct mdk_personality *pers;
 dev_t unit;
 int md_minor;
 struct list_head disks;
 int sb_dirty;
 int ro;

 struct gendisk *gendisk;

 struct kobject kobj;


 int major_version,
     minor_version,
     patch_version;
 int persistent;
 int chunk_size;
 time_t ctime, utime;
 int level, layout;
 char clevel[16];
 int raid_disks;
 int max_disks;
 sector_t size;
 sector_t array_size;
 __u64 events;

 char uuid[16];






 sector_t reshape_position;
 int delta_disks, new_level, new_layout, new_chunk;

 struct mdk_thread_s *thread;
 struct mdk_thread_s *sync_thread;
 sector_t curr_resync;
 unsigned long resync_mark;
 sector_t resync_mark_cnt;
 sector_t curr_mark_cnt;

 sector_t resync_max_sectors;

 sector_t resync_mismatches;




 sector_t suspend_lo;
 sector_t suspend_hi;

 int sync_speed_min;
 int sync_speed_max;

 int ok_start_degraded;

 unsigned long recovery;

 int in_sync;
 struct mutex reconfig_mutex;
 atomic_t active;

 int changed;
 int degraded;


 int barriers_work;



 struct bio *biolist;



 atomic_t recovery_active;
 wait_queue_head_t recovery_wait;
 sector_t recovery_cp;

 spinlock_t write_lock;
 wait_queue_head_t sb_wait;
 atomic_t pending_writes;

 unsigned int safemode;


 unsigned int safemode_delay;
 struct timer_list safemode_timer;
 atomic_t writes_pending;
 request_queue_t *queue;

 atomic_t write_behind;
 unsigned int max_write_behind;

 struct bitmap *bitmap;
 struct file *bitmap_file;
 long bitmap_offset;



 long default_bitmap_offset;




 struct list_head all_mddevs;
};


static inline __attribute__((always_inline)) void rdev_dec_pending(mdk_rdev_t *rdev, mddev_t *mddev)
{
 int faulty = (__builtin_constant_p(1) ? constant_test_bit((1),(&rdev->flags)) : variable_test_bit((1),(&rdev->flags)));
 if (atomic_dec_and_test(&rdev->nr_pending) && faulty)
  set_bit(5, &mddev->recovery);
}

static inline __attribute__((always_inline)) void md_sync_acct(struct block_device *bdev, unsigned long nr_sectors)
{
        atomic_add(nr_sectors, &bdev->bd_contains->bd_disk->sync_io);
}

struct mdk_personality
{
 char *name;
 int level;
 struct list_head list;
 struct module *owner;
 int (*make_request)(request_queue_t *q, struct bio *bio);
 int (*run)(mddev_t *mddev);
 int (*stop)(mddev_t *mddev);
 void (*status)(struct seq_file *seq, mddev_t *mddev);



 void (*error_handler)(mddev_t *mddev, mdk_rdev_t *rdev);
 int (*hot_add_disk) (mddev_t *mddev, mdk_rdev_t *rdev);
 int (*hot_remove_disk) (mddev_t *mddev, int number);
 int (*spare_active) (mddev_t *mddev);
 sector_t (*sync_request)(mddev_t *mddev, sector_t sector_nr, int *skipped, int go_faster);
 int (*resize) (mddev_t *mddev, sector_t sectors);
 int (*check_reshape) (mddev_t *mddev);
 int (*start_reshape) (mddev_t *mddev);
 int (*reconfig) (mddev_t *mddev, int layout, int chunk_size);





 void (*quiesce) (mddev_t *mddev, int state);
};


struct md_sysfs_entry {
 struct attribute attr;
 ssize_t (*show)(mddev_t *, char *);
 ssize_t (*store)(mddev_t *, const char *, size_t);
};


static inline __attribute__((always_inline)) char * mdname (mddev_t * mddev)
{
 return mddev->gendisk ? mddev->gendisk->disk_name : "mdX";
}

typedef struct mdk_thread_s {
 void (*run) (mddev_t *mddev);
 mddev_t *mddev;
 wait_queue_head_t wqueue;
 unsigned long flags;
 struct task_struct *tsk;
 unsigned long timeout;
} mdk_thread_t;

static inline __attribute__((always_inline)) void safe_put_page(struct page *p)
{
 if (p) put_page(p);
}


extern int register_md_personality (struct mdk_personality *p);
extern int unregister_md_personality (struct mdk_personality *p);
extern mdk_thread_t * md_register_thread (void (*run) (mddev_t *mddev),
    mddev_t *mddev, const char *name);
extern void md_unregister_thread (mdk_thread_t *thread);
extern void md_wakeup_thread(mdk_thread_t *thread);
extern void md_check_recovery(mddev_t *mddev);
extern void md_write_start(mddev_t *mddev, struct bio *bi);
extern void md_write_end(mddev_t *mddev);
extern void md_handle_safemode(mddev_t *mddev);
extern void md_done_sync(mddev_t *mddev, int blocks, int ok);
extern void md_error (mddev_t *mddev, mdk_rdev_t *rdev);
extern void md_unplug_mddev(mddev_t *mddev);

extern void md_super_write(mddev_t *mddev, mdk_rdev_t *rdev,
      sector_t sector, int size, struct page *page);
extern void md_super_wait(mddev_t *mddev);
extern int sync_page_io(struct block_device *bdev, sector_t sector, int size,
   struct page *page, int rw);
extern void md_do_sync(mddev_t *mddev);
extern void md_new_event(mddev_t *mddev);

extern void md_update_sb(mddev_t * mddev);



typedef __u16 bitmap_counter_t;

enum bitmap_state {
 BITMAP_ACTIVE = 0x001,
 BITMAP_STALE = 0x002,
 BITMAP_WRITE_ERROR = 0x004,
 BITMAP_HOSTENDIAN = 0x8000,
};


typedef struct bitmap_super_s {
 __u32 magic;
 __u32 version;
 __u8 uuid[16];
 __u64 events;
 __u64 events_cleared;
 __u64 sync_size;
 __u32 state;
 __u32 chunksize;
 __u32 daemon_sleep;
 __u32 write_behind;

 __u8 pad[256 - 64];
} bitmap_super_t;

struct bitmap_page {



 char *map;




 unsigned int hijacked:1;



 unsigned int count:31;
};


struct page_list {
 struct list_head list;
 struct page *page;
};


struct bitmap {
 struct bitmap_page *bp;
 unsigned long pages;
 unsigned long missing_pages;

 mddev_t *mddev;

 int counter_bits;


 unsigned long chunksize;
 unsigned long chunkshift;
 unsigned long chunks;






 unsigned long syncchunk;

 __u64 events_cleared;


 spinlock_t lock;

 long offset;
 struct file *file;
 struct page *sb_page;
 struct page **filemap;
 unsigned long *filemap_attr;
 unsigned long file_pages;

 unsigned long flags;

 unsigned long max_write_behind;
 atomic_t behind_writes;





 unsigned long daemon_lastrun;
 unsigned long daemon_sleep;

 atomic_t pending_writes;
 wait_queue_head_t write_wait;

};




int bitmap_create(mddev_t *mddev);
void bitmap_flush(mddev_t *mddev);
void bitmap_destroy(mddev_t *mddev);
int bitmap_active(struct bitmap *bitmap);

char *file_path(struct file *file, char *buf, int count);
void bitmap_print_sb(struct bitmap *bitmap);
int bitmap_update_sb(struct bitmap *bitmap);

int bitmap_setallbits(struct bitmap *bitmap);
void bitmap_write_all(struct bitmap *bitmap);


int bitmap_startwrite(struct bitmap *bitmap, sector_t offset,
   unsigned long sectors, int behind);
void bitmap_endwrite(struct bitmap *bitmap, sector_t offset,
   unsigned long sectors, int success, int behind);
int bitmap_start_sync(struct bitmap *bitmap, sector_t offset, int *blocks, int degraded);
void bitmap_end_sync(struct bitmap *bitmap, sector_t offset, int *blocks, int aborted);
void bitmap_close_sync(struct bitmap *bitmap);

int bitmap_unplug(struct bitmap *bitmap);
int bitmap_daemon_work(struct bitmap *bitmap);



struct file;
struct completion;






struct __sysctl_args {
 int *name;
 int nlen;
 void *oldval;
 size_t *oldlenp;
 void *newval;
 size_t newlen;
 unsigned long __unused[4];
};

enum
{
 CTL_KERN=1,
 CTL_VM=2,
 CTL_NET=3,

 CTL_FS=5,
 CTL_DEBUG=6,
 CTL_DEV=7,
 CTL_BUS=8,
 CTL_ABI=9,
 CTL_CPU=10
};


enum
{
 CTL_BUS_ISA=1
};


enum
{
 INOTIFY_MAX_USER_INSTANCES=1,
 INOTIFY_MAX_USER_WATCHES=2,
 INOTIFY_MAX_QUEUED_EVENTS=3
};


enum
{
 KERN_OSTYPE=1,
 KERN_OSRELEASE=2,
 KERN_OSREV=3,
 KERN_VERSION=4,
 KERN_SECUREMASK=5,
 KERN_PROF=6,
 KERN_NODENAME=7,
 KERN_DOMAINNAME=8,

 KERN_CAP_BSET=14,
 KERN_PANIC=15,
 KERN_REALROOTDEV=16,

 KERN_SPARC_REBOOT=21,
 KERN_CTLALTDEL=22,
 KERN_PRINTK=23,
 KERN_NAMETRANS=24,
 KERN_PPC_HTABRECLAIM=25,
 KERN_PPC_ZEROPAGED=26,
 KERN_PPC_POWERSAVE_NAP=27,
 KERN_MODPROBE=28,
 KERN_SG_BIG_BUFF=29,
 KERN_ACCT=30,
 KERN_PPC_L2CR=31,

 KERN_RTSIGNR=32,
 KERN_RTSIGMAX=33,

 KERN_SHMMAX=34,
 KERN_MSGMAX=35,
 KERN_MSGMNB=36,
 KERN_MSGPOOL=37,
 KERN_SYSRQ=38,
 KERN_MAX_THREADS=39,
  KERN_RANDOM=40,
  KERN_SHMALL=41,
  KERN_MSGMNI=42,
  KERN_SEM=43,
  KERN_SPARC_STOP_A=44,
  KERN_SHMMNI=45,
 KERN_OVERFLOWUID=46,
 KERN_OVERFLOWGID=47,
 KERN_SHMPATH=48,
 KERN_HOTPLUG=49,
 KERN_IEEE_EMULATION_WARNINGS=50,
 KERN_S390_USER_DEBUG_LOGGING=51,
 KERN_CORE_USES_PID=52,
 KERN_TAINTED=53,
 KERN_CADPID=54,
 KERN_PIDMAX=55,
   KERN_CORE_PATTERN=56,
 KERN_PANIC_ON_OOPS=57,
 KERN_HPPA_PWRSW=58,
 KERN_HPPA_UNALIGNED=59,
 KERN_PRINTK_RATELIMIT=60,
 KERN_PRINTK_RATELIMIT_BURST=61,
 KERN_PTY=62,
 KERN_NGROUPS_MAX=63,
 KERN_SPARC_SCONS_PWROFF=64,
 KERN_HZ_TIMER=65,
 KERN_UNKNOWN_NMI_PANIC=66,
 KERN_BOOTLOADER_TYPE=67,
 KERN_RANDOMIZE=68,
 KERN_SETUID_DUMPABLE=69,
 KERN_SPIN_RETRY=70,
 KERN_ACPI_VIDEO_FLAGS=71,
 KERN_IA64_UNALIGNED=72,
 KERN_COMPAT_LOG=73,
 KERN_MAX_LOCK_DEPTH=74,
};




enum
{
 VM_UNUSED1=1,
 VM_UNUSED2=2,
 VM_UNUSED3=3,
 VM_UNUSED4=4,
 VM_OVERCOMMIT_MEMORY=5,
 VM_UNUSED5=6,
 VM_UNUSED7=7,
 VM_UNUSED8=8,
 VM_UNUSED9=9,
 VM_PAGE_CLUSTER=10,
 VM_DIRTY_BACKGROUND=11,
 VM_DIRTY_RATIO=12,
 VM_DIRTY_WB_CS=13,
 VM_DIRTY_EXPIRE_CS=14,
 VM_NR_PDFLUSH_THREADS=15,
 VM_OVERCOMMIT_RATIO=16,
 VM_PAGEBUF=17,
 VM_HUGETLB_PAGES=18,
 VM_SWAPPINESS=19,
 VM_LOWMEM_RESERVE_RATIO=20,
 VM_MIN_FREE_KBYTES=21,
 VM_MAX_MAP_COUNT=22,
 VM_LAPTOP_MODE=23,
 VM_BLOCK_DUMP=24,
 VM_HUGETLB_GROUP=25,
 VM_VFS_CACHE_PRESSURE=26,
 VM_LEGACY_VA_LAYOUT=27,
 VM_SWAP_TOKEN_TIMEOUT=28,
 VM_DROP_PAGECACHE=29,
 VM_PERCPU_PAGELIST_FRACTION=30,
 VM_ZONE_RECLAIM_MODE=31,
 VM_MIN_UNMAPPED=32,
 VM_PANIC_ON_OOM=33,
 VM_VDSO_ENABLED=34,
 VM_MIN_SLAB=35,
};



enum
{
 NET_CORE=1,
 NET_ETHER=2,
 NET_802=3,
 NET_UNIX=4,
 NET_IPV4=5,
 NET_IPX=6,
 NET_ATALK=7,
 NET_NETROM=8,
 NET_AX25=9,
 NET_BRIDGE=10,
 NET_ROSE=11,
 NET_IPV6=12,
 NET_X25=13,
 NET_TR=14,
 NET_DECNET=15,
 NET_ECONET=16,
 NET_SCTP=17,
 NET_LLC=18,
 NET_NETFILTER=19,
 NET_DCCP=20,
};


enum
{
 RANDOM_POOLSIZE=1,
 RANDOM_ENTROPY_COUNT=2,
 RANDOM_READ_THRESH=3,
 RANDOM_WRITE_THRESH=4,
 RANDOM_BOOT_ID=5,
 RANDOM_UUID=6
};


enum
{
 PTY_MAX=1,
 PTY_NR=2
};


enum
{
 BUS_ISA_MEM_BASE=1,
 BUS_ISA_PORT_BASE=2,
 BUS_ISA_PORT_SHIFT=3
};


enum
{
 NET_CORE_WMEM_MAX=1,
 NET_CORE_RMEM_MAX=2,
 NET_CORE_WMEM_DEFAULT=3,
 NET_CORE_RMEM_DEFAULT=4,

 NET_CORE_MAX_BACKLOG=6,
 NET_CORE_FASTROUTE=7,
 NET_CORE_MSG_COST=8,
 NET_CORE_MSG_BURST=9,
 NET_CORE_OPTMEM_MAX=10,
 NET_CORE_HOT_LIST_LENGTH=11,
 NET_CORE_DIVERT_VERSION=12,
 NET_CORE_NO_CONG_THRESH=13,
 NET_CORE_NO_CONG=14,
 NET_CORE_LO_CONG=15,
 NET_CORE_MOD_CONG=16,
 NET_CORE_DEV_WEIGHT=17,
 NET_CORE_SOMAXCONN=18,
 NET_CORE_BUDGET=19,
 NET_CORE_AEVENT_ETIME=20,
 NET_CORE_AEVENT_RSEQTH=21,
};







enum
{
 NET_UNIX_DESTROY_DELAY=1,
 NET_UNIX_DELETE_DELAY=2,
 NET_UNIX_MAX_DGRAM_QLEN=3,
};


enum
{
 NET_NF_CONNTRACK_MAX=1,
 NET_NF_CONNTRACK_TCP_TIMEOUT_SYN_SENT=2,
 NET_NF_CONNTRACK_TCP_TIMEOUT_SYN_RECV=3,
 NET_NF_CONNTRACK_TCP_TIMEOUT_ESTABLISHED=4,
 NET_NF_CONNTRACK_TCP_TIMEOUT_FIN_WAIT=5,
 NET_NF_CONNTRACK_TCP_TIMEOUT_CLOSE_WAIT=6,
 NET_NF_CONNTRACK_TCP_TIMEOUT_LAST_ACK=7,
 NET_NF_CONNTRACK_TCP_TIMEOUT_TIME_WAIT=8,
 NET_NF_CONNTRACK_TCP_TIMEOUT_CLOSE=9,
 NET_NF_CONNTRACK_UDP_TIMEOUT=10,
 NET_NF_CONNTRACK_UDP_TIMEOUT_STREAM=11,
 NET_NF_CONNTRACK_ICMP_TIMEOUT=12,
 NET_NF_CONNTRACK_GENERIC_TIMEOUT=13,
 NET_NF_CONNTRACK_BUCKETS=14,
 NET_NF_CONNTRACK_LOG_INVALID=15,
 NET_NF_CONNTRACK_TCP_TIMEOUT_MAX_RETRANS=16,
 NET_NF_CONNTRACK_TCP_LOOSE=17,
 NET_NF_CONNTRACK_TCP_BE_LIBERAL=18,
 NET_NF_CONNTRACK_TCP_MAX_RETRANS=19,
 NET_NF_CONNTRACK_SCTP_TIMEOUT_CLOSED=20,
 NET_NF_CONNTRACK_SCTP_TIMEOUT_COOKIE_WAIT=21,
 NET_NF_CONNTRACK_SCTP_TIMEOUT_COOKIE_ECHOED=22,
 NET_NF_CONNTRACK_SCTP_TIMEOUT_ESTABLISHED=23,
 NET_NF_CONNTRACK_SCTP_TIMEOUT_SHUTDOWN_SENT=24,
 NET_NF_CONNTRACK_SCTP_TIMEOUT_SHUTDOWN_RECD=25,
 NET_NF_CONNTRACK_SCTP_TIMEOUT_SHUTDOWN_ACK_SENT=26,
 NET_NF_CONNTRACK_COUNT=27,
 NET_NF_CONNTRACK_ICMPV6_TIMEOUT=28,
 NET_NF_CONNTRACK_FRAG6_TIMEOUT=29,
 NET_NF_CONNTRACK_FRAG6_LOW_THRESH=30,
 NET_NF_CONNTRACK_FRAG6_HIGH_THRESH=31,
 NET_NF_CONNTRACK_CHECKSUM=32,
};


enum
{

 NET_IPV4_FORWARD=8,
 NET_IPV4_DYNADDR=9,

 NET_IPV4_CONF=16,
 NET_IPV4_NEIGH=17,
 NET_IPV4_ROUTE=18,
 NET_IPV4_FIB_HASH=19,
 NET_IPV4_NETFILTER=20,

 NET_IPV4_TCP_TIMESTAMPS=33,
 NET_IPV4_TCP_WINDOW_SCALING=34,
 NET_IPV4_TCP_SACK=35,
 NET_IPV4_TCP_RETRANS_COLLAPSE=36,
 NET_IPV4_DEFAULT_TTL=37,
 NET_IPV4_AUTOCONFIG=38,
 NET_IPV4_NO_PMTU_DISC=39,
 NET_IPV4_TCP_SYN_RETRIES=40,
 NET_IPV4_IPFRAG_HIGH_THRESH=41,
 NET_IPV4_IPFRAG_LOW_THRESH=42,
 NET_IPV4_IPFRAG_TIME=43,
 NET_IPV4_TCP_MAX_KA_PROBES=44,
 NET_IPV4_TCP_KEEPALIVE_TIME=45,
 NET_IPV4_TCP_KEEPALIVE_PROBES=46,
 NET_IPV4_TCP_RETRIES1=47,
 NET_IPV4_TCP_RETRIES2=48,
 NET_IPV4_TCP_FIN_TIMEOUT=49,
 NET_IPV4_IP_MASQ_DEBUG=50,
 NET_TCP_SYNCOOKIES=51,
 NET_TCP_STDURG=52,
 NET_TCP_RFC1337=53,
 NET_TCP_SYN_TAILDROP=54,
 NET_TCP_MAX_SYN_BACKLOG=55,
 NET_IPV4_LOCAL_PORT_RANGE=56,
 NET_IPV4_ICMP_ECHO_IGNORE_ALL=57,
 NET_IPV4_ICMP_ECHO_IGNORE_BROADCASTS=58,
 NET_IPV4_ICMP_SOURCEQUENCH_RATE=59,
 NET_IPV4_ICMP_DESTUNREACH_RATE=60,
 NET_IPV4_ICMP_TIMEEXCEED_RATE=61,
 NET_IPV4_ICMP_PARAMPROB_RATE=62,
 NET_IPV4_ICMP_ECHOREPLY_RATE=63,
 NET_IPV4_ICMP_IGNORE_BOGUS_ERROR_RESPONSES=64,
 NET_IPV4_IGMP_MAX_MEMBERSHIPS=65,
 NET_TCP_TW_RECYCLE=66,
 NET_IPV4_ALWAYS_DEFRAG=67,
 NET_IPV4_TCP_KEEPALIVE_INTVL=68,
 NET_IPV4_INET_PEER_THRESHOLD=69,
 NET_IPV4_INET_PEER_MINTTL=70,
 NET_IPV4_INET_PEER_MAXTTL=71,
 NET_IPV4_INET_PEER_GC_MINTIME=72,
 NET_IPV4_INET_PEER_GC_MAXTIME=73,
 NET_TCP_ORPHAN_RETRIES=74,
 NET_TCP_ABORT_ON_OVERFLOW=75,
 NET_TCP_SYNACK_RETRIES=76,
 NET_TCP_MAX_ORPHANS=77,
 NET_TCP_MAX_TW_BUCKETS=78,
 NET_TCP_FACK=79,
 NET_TCP_REORDERING=80,
 NET_TCP_ECN=81,
 NET_TCP_DSACK=82,
 NET_TCP_MEM=83,
 NET_TCP_WMEM=84,
 NET_TCP_RMEM=85,
 NET_TCP_APP_WIN=86,
 NET_TCP_ADV_WIN_SCALE=87,
 NET_IPV4_NONLOCAL_BIND=88,
 NET_IPV4_ICMP_RATELIMIT=89,
 NET_IPV4_ICMP_RATEMASK=90,
 NET_TCP_TW_REUSE=91,
 NET_TCP_FRTO=92,
 NET_TCP_LOW_LATENCY=93,
 NET_IPV4_IPFRAG_SECRET_INTERVAL=94,
 NET_IPV4_IGMP_MAX_MSF=96,
 NET_TCP_NO_METRICS_SAVE=97,
 NET_TCP_DEFAULT_WIN_SCALE=105,
 NET_TCP_MODERATE_RCVBUF=106,
 NET_TCP_TSO_WIN_DIVISOR=107,
 NET_TCP_BIC_BETA=108,
 NET_IPV4_ICMP_ERRORS_USE_INBOUND_IFADDR=109,
 NET_TCP_CONG_CONTROL=110,
 NET_TCP_ABC=111,
 NET_IPV4_IPFRAG_MAX_DIST=112,
  NET_TCP_MTU_PROBING=113,
 NET_TCP_BASE_MSS=114,
 NET_IPV4_TCP_WORKAROUND_SIGNED_WINDOWS=115,
 NET_TCP_DMA_COPYBREAK=116,
 NET_TCP_SLOW_START_AFTER_IDLE=117,
};

enum {
 NET_IPV4_ROUTE_FLUSH=1,
 NET_IPV4_ROUTE_MIN_DELAY=2,
 NET_IPV4_ROUTE_MAX_DELAY=3,
 NET_IPV4_ROUTE_GC_THRESH=4,
 NET_IPV4_ROUTE_MAX_SIZE=5,
 NET_IPV4_ROUTE_GC_MIN_INTERVAL=6,
 NET_IPV4_ROUTE_GC_TIMEOUT=7,
 NET_IPV4_ROUTE_GC_INTERVAL=8,
 NET_IPV4_ROUTE_REDIRECT_LOAD=9,
 NET_IPV4_ROUTE_REDIRECT_NUMBER=10,
 NET_IPV4_ROUTE_REDIRECT_SILENCE=11,
 NET_IPV4_ROUTE_ERROR_COST=12,
 NET_IPV4_ROUTE_ERROR_BURST=13,
 NET_IPV4_ROUTE_GC_ELASTICITY=14,
 NET_IPV4_ROUTE_MTU_EXPIRES=15,
 NET_IPV4_ROUTE_MIN_PMTU=16,
 NET_IPV4_ROUTE_MIN_ADVMSS=17,
 NET_IPV4_ROUTE_SECRET_INTERVAL=18,
 NET_IPV4_ROUTE_GC_MIN_INTERVAL_MS=19,
};

enum
{
 NET_PROTO_CONF_ALL=-2,
 NET_PROTO_CONF_DEFAULT=-3


};

enum
{
 NET_IPV4_CONF_FORWARDING=1,
 NET_IPV4_CONF_MC_FORWARDING=2,
 NET_IPV4_CONF_PROXY_ARP=3,
 NET_IPV4_CONF_ACCEPT_REDIRECTS=4,
 NET_IPV4_CONF_SECURE_REDIRECTS=5,
 NET_IPV4_CONF_SEND_REDIRECTS=6,
 NET_IPV4_CONF_SHARED_MEDIA=7,
 NET_IPV4_CONF_RP_FILTER=8,
 NET_IPV4_CONF_ACCEPT_SOURCE_ROUTE=9,
 NET_IPV4_CONF_BOOTP_RELAY=10,
 NET_IPV4_CONF_LOG_MARTIANS=11,
 NET_IPV4_CONF_TAG=12,
 NET_IPV4_CONF_ARPFILTER=13,
 NET_IPV4_CONF_MEDIUM_ID=14,
 NET_IPV4_CONF_NOXFRM=15,
 NET_IPV4_CONF_NOPOLICY=16,
 NET_IPV4_CONF_FORCE_IGMP_VERSION=17,
 NET_IPV4_CONF_ARP_ANNOUNCE=18,
 NET_IPV4_CONF_ARP_IGNORE=19,
 NET_IPV4_CONF_PROMOTE_SECONDARIES=20,
 NET_IPV4_CONF_ARP_ACCEPT=21,
 __NET_IPV4_CONF_MAX
};


enum
{
 NET_IPV4_NF_CONNTRACK_MAX=1,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_SYN_SENT=2,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_SYN_RECV=3,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_ESTABLISHED=4,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_FIN_WAIT=5,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_CLOSE_WAIT=6,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_LAST_ACK=7,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_TIME_WAIT=8,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_CLOSE=9,
 NET_IPV4_NF_CONNTRACK_UDP_TIMEOUT=10,
 NET_IPV4_NF_CONNTRACK_UDP_TIMEOUT_STREAM=11,
 NET_IPV4_NF_CONNTRACK_ICMP_TIMEOUT=12,
 NET_IPV4_NF_CONNTRACK_GENERIC_TIMEOUT=13,
 NET_IPV4_NF_CONNTRACK_BUCKETS=14,
 NET_IPV4_NF_CONNTRACK_LOG_INVALID=15,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_MAX_RETRANS=16,
 NET_IPV4_NF_CONNTRACK_TCP_LOOSE=17,
 NET_IPV4_NF_CONNTRACK_TCP_BE_LIBERAL=18,
 NET_IPV4_NF_CONNTRACK_TCP_MAX_RETRANS=19,
  NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_CLOSED=20,
  NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_COOKIE_WAIT=21,
  NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_COOKIE_ECHOED=22,
  NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_ESTABLISHED=23,
  NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_SHUTDOWN_SENT=24,
  NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_SHUTDOWN_RECD=25,
  NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_SHUTDOWN_ACK_SENT=26,
 NET_IPV4_NF_CONNTRACK_COUNT=27,
 NET_IPV4_NF_CONNTRACK_CHECKSUM=28,
};


enum {
 NET_IPV6_CONF=16,
 NET_IPV6_NEIGH=17,
 NET_IPV6_ROUTE=18,
 NET_IPV6_ICMP=19,
 NET_IPV6_BINDV6ONLY=20,
 NET_IPV6_IP6FRAG_HIGH_THRESH=21,
 NET_IPV6_IP6FRAG_LOW_THRESH=22,
 NET_IPV6_IP6FRAG_TIME=23,
 NET_IPV6_IP6FRAG_SECRET_INTERVAL=24,
 NET_IPV6_MLD_MAX_MSF=25,
};

enum {
 NET_IPV6_ROUTE_FLUSH=1,
 NET_IPV6_ROUTE_GC_THRESH=2,
 NET_IPV6_ROUTE_MAX_SIZE=3,
 NET_IPV6_ROUTE_GC_MIN_INTERVAL=4,
 NET_IPV6_ROUTE_GC_TIMEOUT=5,
 NET_IPV6_ROUTE_GC_INTERVAL=6,
 NET_IPV6_ROUTE_GC_ELASTICITY=7,
 NET_IPV6_ROUTE_MTU_EXPIRES=8,
 NET_IPV6_ROUTE_MIN_ADVMSS=9,
 NET_IPV6_ROUTE_GC_MIN_INTERVAL_MS=10
};

enum {
 NET_IPV6_FORWARDING=1,
 NET_IPV6_HOP_LIMIT=2,
 NET_IPV6_MTU=3,
 NET_IPV6_ACCEPT_RA=4,
 NET_IPV6_ACCEPT_REDIRECTS=5,
 NET_IPV6_AUTOCONF=6,
 NET_IPV6_DAD_TRANSMITS=7,
 NET_IPV6_RTR_SOLICITS=8,
 NET_IPV6_RTR_SOLICIT_INTERVAL=9,
 NET_IPV6_RTR_SOLICIT_DELAY=10,
 NET_IPV6_USE_TEMPADDR=11,
 NET_IPV6_TEMP_VALID_LFT=12,
 NET_IPV6_TEMP_PREFERED_LFT=13,
 NET_IPV6_REGEN_MAX_RETRY=14,
 NET_IPV6_MAX_DESYNC_FACTOR=15,
 NET_IPV6_MAX_ADDRESSES=16,
 NET_IPV6_FORCE_MLD_VERSION=17,
 NET_IPV6_ACCEPT_RA_DEFRTR=18,
 NET_IPV6_ACCEPT_RA_PINFO=19,
 NET_IPV6_ACCEPT_RA_RTR_PREF=20,
 NET_IPV6_RTR_PROBE_INTERVAL=21,
 NET_IPV6_ACCEPT_RA_RT_INFO_MAX_PLEN=22,
 __NET_IPV6_MAX
};


enum {
 NET_IPV6_ICMP_RATELIMIT=1
};


enum {
 NET_NEIGH_MCAST_SOLICIT=1,
 NET_NEIGH_UCAST_SOLICIT=2,
 NET_NEIGH_APP_SOLICIT=3,
 NET_NEIGH_RETRANS_TIME=4,
 NET_NEIGH_REACHABLE_TIME=5,
 NET_NEIGH_DELAY_PROBE_TIME=6,
 NET_NEIGH_GC_STALE_TIME=7,
 NET_NEIGH_UNRES_QLEN=8,
 NET_NEIGH_PROXY_QLEN=9,
 NET_NEIGH_ANYCAST_DELAY=10,
 NET_NEIGH_PROXY_DELAY=11,
 NET_NEIGH_LOCKTIME=12,
 NET_NEIGH_GC_INTERVAL=13,
 NET_NEIGH_GC_THRESH1=14,
 NET_NEIGH_GC_THRESH2=15,
 NET_NEIGH_GC_THRESH3=16,
 NET_NEIGH_RETRANS_TIME_MS=17,
 NET_NEIGH_REACHABLE_TIME_MS=18,
 __NET_NEIGH_MAX
};


enum {
 NET_DCCP_DEFAULT=1,
};


enum {
 NET_DCCP_DEFAULT_SEQ_WINDOW = 1,
 NET_DCCP_DEFAULT_RX_CCID = 2,
 NET_DCCP_DEFAULT_TX_CCID = 3,
 NET_DCCP_DEFAULT_ACK_RATIO = 4,
 NET_DCCP_DEFAULT_SEND_ACKVEC = 5,
 NET_DCCP_DEFAULT_SEND_NDP = 6,
};


enum {
 NET_IPX_PPROP_BROADCASTING=1,
 NET_IPX_FORWARDING=2
};


enum {
 NET_LLC2=1,
 NET_LLC_STATION=2,
};


enum {
 NET_LLC2_TIMEOUT=1,
};


enum {
 NET_LLC_STATION_ACK_TIMEOUT=1,
};


enum {
 NET_LLC2_ACK_TIMEOUT=1,
 NET_LLC2_P_TIMEOUT=2,
 NET_LLC2_REJ_TIMEOUT=3,
 NET_LLC2_BUSY_TIMEOUT=4,
};


enum {
 NET_ATALK_AARP_EXPIRY_TIME=1,
 NET_ATALK_AARP_TICK_TIME=2,
 NET_ATALK_AARP_RETRANSMIT_LIMIT=3,
 NET_ATALK_AARP_RESOLVE_TIME=4
};



enum {
 NET_NETROM_DEFAULT_PATH_QUALITY=1,
 NET_NETROM_OBSOLESCENCE_COUNT_INITIALISER=2,
 NET_NETROM_NETWORK_TTL_INITIALISER=3,
 NET_NETROM_TRANSPORT_TIMEOUT=4,
 NET_NETROM_TRANSPORT_MAXIMUM_TRIES=5,
 NET_NETROM_TRANSPORT_ACKNOWLEDGE_DELAY=6,
 NET_NETROM_TRANSPORT_BUSY_DELAY=7,
 NET_NETROM_TRANSPORT_REQUESTED_WINDOW_SIZE=8,
 NET_NETROM_TRANSPORT_NO_ACTIVITY_TIMEOUT=9,
 NET_NETROM_ROUTING_CONTROL=10,
 NET_NETROM_LINK_FAILS_COUNT=11,
 NET_NETROM_RESET=12
};


enum {
 NET_AX25_IP_DEFAULT_MODE=1,
 NET_AX25_DEFAULT_MODE=2,
 NET_AX25_BACKOFF_TYPE=3,
 NET_AX25_CONNECT_MODE=4,
 NET_AX25_STANDARD_WINDOW=5,
 NET_AX25_EXTENDED_WINDOW=6,
 NET_AX25_T1_TIMEOUT=7,
 NET_AX25_T2_TIMEOUT=8,
 NET_AX25_T3_TIMEOUT=9,
 NET_AX25_IDLE_TIMEOUT=10,
 NET_AX25_N2=11,
 NET_AX25_PACLEN=12,
 NET_AX25_PROTOCOL=13,
 NET_AX25_DAMA_SLAVE_TIMEOUT=14
};


enum {
 NET_ROSE_RESTART_REQUEST_TIMEOUT=1,
 NET_ROSE_CALL_REQUEST_TIMEOUT=2,
 NET_ROSE_RESET_REQUEST_TIMEOUT=3,
 NET_ROSE_CLEAR_REQUEST_TIMEOUT=4,
 NET_ROSE_ACK_HOLD_BACK_TIMEOUT=5,
 NET_ROSE_ROUTING_CONTROL=6,
 NET_ROSE_LINK_FAIL_TIMEOUT=7,
 NET_ROSE_MAX_VCS=8,
 NET_ROSE_WINDOW_SIZE=9,
 NET_ROSE_NO_ACTIVITY_TIMEOUT=10
};


enum {
 NET_X25_RESTART_REQUEST_TIMEOUT=1,
 NET_X25_CALL_REQUEST_TIMEOUT=2,
 NET_X25_RESET_REQUEST_TIMEOUT=3,
 NET_X25_CLEAR_REQUEST_TIMEOUT=4,
 NET_X25_ACK_HOLD_BACK_TIMEOUT=5
};


enum
{
 NET_TR_RIF_TIMEOUT=1
};


enum {
 NET_DECNET_NODE_TYPE = 1,
 NET_DECNET_NODE_ADDRESS = 2,
 NET_DECNET_NODE_NAME = 3,
 NET_DECNET_DEFAULT_DEVICE = 4,
 NET_DECNET_TIME_WAIT = 5,
 NET_DECNET_DN_COUNT = 6,
 NET_DECNET_DI_COUNT = 7,
 NET_DECNET_DR_COUNT = 8,
 NET_DECNET_DST_GC_INTERVAL = 9,
 NET_DECNET_CONF = 10,
 NET_DECNET_NO_FC_MAX_CWND = 11,
 NET_DECNET_MEM = 12,
 NET_DECNET_RMEM = 13,
 NET_DECNET_WMEM = 14,
 NET_DECNET_DEBUG_LEVEL = 255
};


enum {
 NET_DECNET_CONF_LOOPBACK = -2,
 NET_DECNET_CONF_DDCMP = -3,
 NET_DECNET_CONF_PPP = -4,
 NET_DECNET_CONF_X25 = -5,
 NET_DECNET_CONF_GRE = -6,
 NET_DECNET_CONF_ETHER = -7


};


enum {
 NET_DECNET_CONF_DEV_PRIORITY = 1,
 NET_DECNET_CONF_DEV_T1 = 2,
 NET_DECNET_CONF_DEV_T2 = 3,
 NET_DECNET_CONF_DEV_T3 = 4,
 NET_DECNET_CONF_DEV_FORWARDING = 5,
 NET_DECNET_CONF_DEV_BLKSIZE = 6,
 NET_DECNET_CONF_DEV_STATE = 7
};


enum {
 NET_SCTP_RTO_INITIAL = 1,
 NET_SCTP_RTO_MIN = 2,
 NET_SCTP_RTO_MAX = 3,
 NET_SCTP_RTO_ALPHA = 4,
 NET_SCTP_RTO_BETA = 5,
 NET_SCTP_VALID_COOKIE_LIFE = 6,
 NET_SCTP_ASSOCIATION_MAX_RETRANS = 7,
 NET_SCTP_PATH_MAX_RETRANS = 8,
 NET_SCTP_MAX_INIT_RETRANSMITS = 9,
 NET_SCTP_HB_INTERVAL = 10,
 NET_SCTP_PRESERVE_ENABLE = 11,
 NET_SCTP_MAX_BURST = 12,
 NET_SCTP_ADDIP_ENABLE = 13,
 NET_SCTP_PRSCTP_ENABLE = 14,
 NET_SCTP_SNDBUF_POLICY = 15,
 NET_SCTP_SACK_TIMEOUT = 16,
 NET_SCTP_RCVBUF_POLICY = 17,
};


enum {
 NET_BRIDGE_NF_CALL_ARPTABLES = 1,
 NET_BRIDGE_NF_CALL_IPTABLES = 2,
 NET_BRIDGE_NF_CALL_IP6TABLES = 3,
 NET_BRIDGE_NF_FILTER_VLAN_TAGGED = 4,
};


enum
{
 FS_NRINODE=1,
 FS_STATINODE=2,
 FS_MAXINODE=3,
 FS_NRDQUOT=4,
 FS_MAXDQUOT=5,
 FS_NRFILE=6,
 FS_MAXFILE=7,
 FS_DENTRY=8,
 FS_NRSUPER=9,
 FS_MAXSUPER=10,
 FS_OVERFLOWUID=11,
 FS_OVERFLOWGID=12,
 FS_LEASES=13,
 FS_DIR_NOTIFY=14,
 FS_LEASE_TIME=15,
 FS_DQSTATS=16,
 FS_XFS=17,
 FS_AIO_NR=18,
 FS_AIO_MAX_NR=19,
 FS_INOTIFY=20,
};


enum {
 FS_DQ_LOOKUPS = 1,
 FS_DQ_DROPS = 2,
 FS_DQ_READS = 3,
 FS_DQ_WRITES = 4,
 FS_DQ_CACHE_HITS = 5,
 FS_DQ_ALLOCATED = 6,
 FS_DQ_FREE = 7,
 FS_DQ_SYNCS = 8,
 FS_DQ_WARNINGS = 9,
};




enum {
 DEV_CDROM=1,
 DEV_HWMON=2,
 DEV_PARPORT=3,
 DEV_RAID=4,
 DEV_MAC_HID=5,
 DEV_SCSI=6,
 DEV_IPMI=7,
};


enum {
 DEV_CDROM_INFO=1,
 DEV_CDROM_AUTOCLOSE=2,
 DEV_CDROM_AUTOEJECT=3,
 DEV_CDROM_DEBUG=4,
 DEV_CDROM_LOCK=5,
 DEV_CDROM_CHECK_MEDIA=6
};


enum {
 DEV_PARPORT_DEFAULT=-3
};


enum {
 DEV_RAID_SPEED_LIMIT_MIN=1,
 DEV_RAID_SPEED_LIMIT_MAX=2
};


enum {
 DEV_PARPORT_DEFAULT_TIMESLICE=1,
 DEV_PARPORT_DEFAULT_SPINTIME=2
};


enum {
 DEV_PARPORT_SPINTIME=1,
 DEV_PARPORT_BASE_ADDR=2,
 DEV_PARPORT_IRQ=3,
 DEV_PARPORT_DMA=4,
 DEV_PARPORT_MODES=5,
 DEV_PARPORT_DEVICES=6,
 DEV_PARPORT_AUTOPROBE=16
};


enum {
 DEV_PARPORT_DEVICES_ACTIVE=-3,
};


enum {
 DEV_PARPORT_DEVICE_TIMESLICE=1,
};


enum {
 DEV_MAC_HID_KEYBOARD_SENDS_LINUX_KEYCODES=1,
 DEV_MAC_HID_KEYBOARD_LOCK_KEYCODES=2,
 DEV_MAC_HID_MOUSE_BUTTON_EMULATION=3,
 DEV_MAC_HID_MOUSE_BUTTON2_KEYCODE=4,
 DEV_MAC_HID_MOUSE_BUTTON3_KEYCODE=5,
 DEV_MAC_HID_ADB_MOUSE_SENDS_KEYCODES=6
};


enum {
 DEV_SCSI_LOGGING_LEVEL=1,
};


enum {
 DEV_IPMI_POWEROFF_POWERCYCLE=1,
};


enum
{
 ABI_DEFHANDLER_COFF=1,
 ABI_DEFHANDLER_ELF=2,
 ABI_DEFHANDLER_LCALL7=3,
 ABI_DEFHANDLER_LIBCSO=4,
 ABI_TRACE=5,
 ABI_FAKE_UTSNAME=6,
};




extern void sysctl_init(void);

typedef struct ctl_table ctl_table;

typedef int ctl_handler (ctl_table *table, int *name, int nlen,
    void *oldval, size_t *oldlenp,
    void *newval, size_t newlen,
    void **context);

typedef int proc_handler (ctl_table *ctl, int write, struct file * filp,
     void *buffer, size_t *lenp, loff_t *ppos);

extern int proc_dostring(ctl_table *, int, struct file *,
    void *, size_t *, loff_t *);
extern int proc_dointvec(ctl_table *, int, struct file *,
    void *, size_t *, loff_t *);
extern int proc_dointvec_bset(ctl_table *, int, struct file *,
         void *, size_t *, loff_t *);
extern int proc_dointvec_minmax(ctl_table *, int, struct file *,
    void *, size_t *, loff_t *);
extern int proc_dointvec_jiffies(ctl_table *, int, struct file *,
     void *, size_t *, loff_t *);
extern int proc_dointvec_userhz_jiffies(ctl_table *, int, struct file *,
     void *, size_t *, loff_t *);
extern int proc_dointvec_ms_jiffies(ctl_table *, int, struct file *,
        void *, size_t *, loff_t *);
extern int proc_doulongvec_minmax(ctl_table *, int, struct file *,
      void *, size_t *, loff_t *);
extern int proc_doulongvec_ms_jiffies_minmax(ctl_table *table, int,
          struct file *, void *, size_t *, loff_t *);

extern int do_sysctl (int *name, int nlen,
        void *oldval, size_t *oldlenp,
        void *newval, size_t newlen);

extern int do_sysctl_strategy (ctl_table *table,
          int *name, int nlen,
          void *oldval, size_t *oldlenp,
          void *newval, size_t newlen, void ** context);

extern ctl_handler sysctl_string;
extern ctl_handler sysctl_intvec;
extern ctl_handler sysctl_jiffies;
extern ctl_handler sysctl_ms_jiffies;

struct ctl_table
{
 int ctl_name;
 const char *procname;
 void *data;
 int maxlen;
 mode_t mode;
 ctl_table *child;
 proc_handler *proc_handler;
 ctl_handler *strategy;
 struct proc_dir_entry *de;
 void *extra1;
 void *extra2;
};



struct ctl_table_header
{
 ctl_table *ctl_table;
 struct list_head ctl_entry;
 int used;
 struct completion *unregistering;
};

struct ctl_table_header * register_sysctl_table(ctl_table * table,
      int insert_at_head);
void unregister_sysctl_table(struct ctl_table_header * table);



enum bh_state_bits {
 BH_Uptodate,
 BH_Dirty,
 BH_Lock,
 BH_Req,
 BH_Uptodate_Lock,



 BH_Mapped,
 BH_New,
 BH_Async_Read,
 BH_Async_Write,
 BH_Delay,
 BH_Boundary,
 BH_Write_EIO,
 BH_Ordered,
 BH_Eopnotsupp,

 BH_PrivateStart,


};



struct page;
struct buffer_head;
struct address_space;
typedef void (bh_end_io_t)(struct buffer_head *bh, int uptodate);

struct buffer_head {
 unsigned long b_state;
 struct buffer_head *b_this_page;
 struct page *b_page;

 sector_t b_blocknr;
 size_t b_size;
 char *b_data;

 struct block_device *b_bdev;
 bh_end_io_t *b_end_io;
  void *b_private;
 struct list_head b_assoc_buffers;
 atomic_t b_count;
};

static inline __attribute__((always_inline)) void set_buffer_uptodate(struct buffer_head *bh) { set_bit(BH_Uptodate, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_uptodate(struct buffer_head *bh) { clear_bit(BH_Uptodate, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_uptodate(const struct buffer_head *bh) { return (__builtin_constant_p(BH_Uptodate) ? constant_test_bit((BH_Uptodate),(&(bh)->b_state)) : variable_test_bit((BH_Uptodate),(&(bh)->b_state))); }
static inline __attribute__((always_inline)) void set_buffer_dirty(struct buffer_head *bh) { set_bit(BH_Dirty, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_dirty(struct buffer_head *bh) { clear_bit(BH_Dirty, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_dirty(const struct buffer_head *bh) { return (__builtin_constant_p(BH_Dirty) ? constant_test_bit((BH_Dirty),(&(bh)->b_state)) : variable_test_bit((BH_Dirty),(&(bh)->b_state))); }
static inline __attribute__((always_inline)) int test_set_buffer_dirty(struct buffer_head *bh) { return test_and_set_bit(BH_Dirty, &(bh)->b_state); } static inline __attribute__((always_inline)) int test_clear_buffer_dirty(struct buffer_head *bh) { return test_and_clear_bit(BH_Dirty, &(bh)->b_state); }
static inline __attribute__((always_inline)) void set_buffer_locked(struct buffer_head *bh) { set_bit(BH_Lock, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_locked(struct buffer_head *bh) { clear_bit(BH_Lock, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_locked(const struct buffer_head *bh) { return (__builtin_constant_p(BH_Lock) ? constant_test_bit((BH_Lock),(&(bh)->b_state)) : variable_test_bit((BH_Lock),(&(bh)->b_state))); }
static inline __attribute__((always_inline)) int test_set_buffer_locked(struct buffer_head *bh) { return test_and_set_bit(BH_Lock, &(bh)->b_state); } static inline __attribute__((always_inline)) int test_clear_buffer_locked(struct buffer_head *bh) { return test_and_clear_bit(BH_Lock, &(bh)->b_state); }
static inline __attribute__((always_inline)) void set_buffer_req(struct buffer_head *bh) { set_bit(BH_Req, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_req(struct buffer_head *bh) { clear_bit(BH_Req, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_req(const struct buffer_head *bh) { return (__builtin_constant_p(BH_Req) ? constant_test_bit((BH_Req),(&(bh)->b_state)) : variable_test_bit((BH_Req),(&(bh)->b_state))); }
static inline __attribute__((always_inline)) int test_set_buffer_req(struct buffer_head *bh) { return test_and_set_bit(BH_Req, &(bh)->b_state); } static inline __attribute__((always_inline)) int test_clear_buffer_req(struct buffer_head *bh) { return test_and_clear_bit(BH_Req, &(bh)->b_state); }
static inline __attribute__((always_inline)) void set_buffer_mapped(struct buffer_head *bh) { set_bit(BH_Mapped, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_mapped(struct buffer_head *bh) { clear_bit(BH_Mapped, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_mapped(const struct buffer_head *bh) { return (__builtin_constant_p(BH_Mapped) ? constant_test_bit((BH_Mapped),(&(bh)->b_state)) : variable_test_bit((BH_Mapped),(&(bh)->b_state))); }
static inline __attribute__((always_inline)) void set_buffer_new(struct buffer_head *bh) { set_bit(BH_New, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_new(struct buffer_head *bh) { clear_bit(BH_New, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_new(const struct buffer_head *bh) { return (__builtin_constant_p(BH_New) ? constant_test_bit((BH_New),(&(bh)->b_state)) : variable_test_bit((BH_New),(&(bh)->b_state))); }
static inline __attribute__((always_inline)) void set_buffer_async_read(struct buffer_head *bh) { set_bit(BH_Async_Read, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_async_read(struct buffer_head *bh) { clear_bit(BH_Async_Read, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_async_read(const struct buffer_head *bh) { return (__builtin_constant_p(BH_Async_Read) ? constant_test_bit((BH_Async_Read),(&(bh)->b_state)) : variable_test_bit((BH_Async_Read),(&(bh)->b_state))); }
static inline __attribute__((always_inline)) void set_buffer_async_write(struct buffer_head *bh) { set_bit(BH_Async_Write, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_async_write(struct buffer_head *bh) { clear_bit(BH_Async_Write, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_async_write(const struct buffer_head *bh) { return (__builtin_constant_p(BH_Async_Write) ? constant_test_bit((BH_Async_Write),(&(bh)->b_state)) : variable_test_bit((BH_Async_Write),(&(bh)->b_state))); }
static inline __attribute__((always_inline)) void set_buffer_delay(struct buffer_head *bh) { set_bit(BH_Delay, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_delay(struct buffer_head *bh) { clear_bit(BH_Delay, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_delay(const struct buffer_head *bh) { return (__builtin_constant_p(BH_Delay) ? constant_test_bit((BH_Delay),(&(bh)->b_state)) : variable_test_bit((BH_Delay),(&(bh)->b_state))); }
static inline __attribute__((always_inline)) void set_buffer_boundary(struct buffer_head *bh) { set_bit(BH_Boundary, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_boundary(struct buffer_head *bh) { clear_bit(BH_Boundary, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_boundary(const struct buffer_head *bh) { return (__builtin_constant_p(BH_Boundary) ? constant_test_bit((BH_Boundary),(&(bh)->b_state)) : variable_test_bit((BH_Boundary),(&(bh)->b_state))); }
static inline __attribute__((always_inline)) void set_buffer_write_io_error(struct buffer_head *bh) { set_bit(BH_Write_EIO, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_write_io_error(struct buffer_head *bh) { clear_bit(BH_Write_EIO, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_write_io_error(const struct buffer_head *bh) { return (__builtin_constant_p(BH_Write_EIO) ? constant_test_bit((BH_Write_EIO),(&(bh)->b_state)) : variable_test_bit((BH_Write_EIO),(&(bh)->b_state))); }
static inline __attribute__((always_inline)) void set_buffer_ordered(struct buffer_head *bh) { set_bit(BH_Ordered, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_ordered(struct buffer_head *bh) { clear_bit(BH_Ordered, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_ordered(const struct buffer_head *bh) { return (__builtin_constant_p(BH_Ordered) ? constant_test_bit((BH_Ordered),(&(bh)->b_state)) : variable_test_bit((BH_Ordered),(&(bh)->b_state))); }
static inline __attribute__((always_inline)) void set_buffer_eopnotsupp(struct buffer_head *bh) { set_bit(BH_Eopnotsupp, &(bh)->b_state); } static inline __attribute__((always_inline)) void clear_buffer_eopnotsupp(struct buffer_head *bh) { clear_bit(BH_Eopnotsupp, &(bh)->b_state); } static inline __attribute__((always_inline)) int buffer_eopnotsupp(const struct buffer_head *bh) { return (__builtin_constant_p(BH_Eopnotsupp) ? constant_test_bit((BH_Eopnotsupp),(&(bh)->b_state)) : variable_test_bit((BH_Eopnotsupp),(&(bh)->b_state))); }

void mark_buffer_dirty(struct buffer_head *bh) __attribute__((regparm(3)));
void init_buffer(struct buffer_head *, bh_end_io_t *, void *);
void set_bh_page(struct buffer_head *bh,
  struct page *page, unsigned long offset);
int try_to_free_buffers(struct page *);
struct buffer_head *alloc_page_buffers(struct page *page, unsigned long size,
  int retry);
void create_empty_buffers(struct page *, unsigned long,
   unsigned long b_state);
void end_buffer_read_sync(struct buffer_head *bh, int uptodate);
void end_buffer_write_sync(struct buffer_head *bh, int uptodate);


void mark_buffer_dirty_inode(struct buffer_head *bh, struct inode *inode);
int inode_has_buffers(struct inode *);
void invalidate_inode_buffers(struct inode *);
int remove_inode_buffers(struct inode *inode);
int sync_mapping_buffers(struct address_space *mapping);
void unmap_underlying_metadata(struct block_device *bdev, sector_t block);

void mark_buffer_async_write(struct buffer_head *bh);
void invalidate_bdev(struct block_device *, int);
int sync_blockdev(struct block_device *bdev);
void __wait_on_buffer(struct buffer_head *);
wait_queue_head_t *bh_waitq_head(struct buffer_head *bh);
int fsync_bdev(struct block_device *);
struct super_block *freeze_bdev(struct block_device *);
void thaw_bdev(struct block_device *, struct super_block *);
int fsync_super(struct super_block *);
int fsync_no_super(struct block_device *);
struct buffer_head *__find_get_block(struct block_device *, sector_t, int);
struct buffer_head * __getblk(struct block_device *, sector_t, int);
void __brelse(struct buffer_head *);
void __bforget(struct buffer_head *);
void __breadahead(struct block_device *, sector_t block, int size);
struct buffer_head *__bread(struct block_device *, sector_t block, int size);
struct buffer_head *alloc_buffer_head(gfp_t gfp_flags);
void free_buffer_head(struct buffer_head * bh);
void unlock_buffer(struct buffer_head *bh) __attribute__((regparm(3)));
void __lock_buffer(struct buffer_head *bh) __attribute__((regparm(3)));
void ll_rw_block(int, int, struct buffer_head * bh[]);
int sync_dirty_buffer(struct buffer_head *bh);
int submit_bh(int, struct buffer_head *);
void write_boundary_block(struct block_device *bdev,
   sector_t bblock, unsigned blocksize);

extern int buffer_heads_over_limit;





int try_to_release_page(struct page * page, gfp_t gfp_mask);
void block_invalidatepage(struct page *page, unsigned long offset);
void do_invalidatepage(struct page *page, unsigned long offset);
int block_write_full_page(struct page *page, get_block_t *get_block,
    struct writeback_control *wbc);
int block_read_full_page(struct page*, get_block_t*);
int block_prepare_write(struct page*, unsigned, unsigned, get_block_t*);
int cont_prepare_write(struct page*, unsigned, unsigned, get_block_t*,
    loff_t *);
int generic_cont_expand(struct inode *inode, loff_t size);
int generic_cont_expand_simple(struct inode *inode, loff_t size);
int block_commit_write(struct page *page, unsigned from, unsigned to);
void block_sync_page(struct page *);
sector_t generic_block_bmap(struct address_space *, sector_t, get_block_t *);
int generic_commit_write(struct file *, struct page *, unsigned, unsigned);
int block_truncate_page(struct address_space *, loff_t, get_block_t *);
int file_fsync(struct file *, struct dentry *, int);
int nobh_prepare_write(struct page*, unsigned, unsigned, get_block_t*);
int nobh_commit_write(struct file *, struct page *, unsigned, unsigned);
int nobh_truncate_page(struct address_space *, loff_t);
int nobh_writepage(struct page *page, get_block_t *get_block,
                        struct writeback_control *wbc);

void buffer_init(void);





static inline __attribute__((always_inline)) void attach_page_buffers(struct page *page,
  struct buffer_head *head)
{
 get_page(page);
 set_bit(11, &(page)->flags);
 ((page)->private = ((unsigned long)head));
}

static inline __attribute__((always_inline)) void get_bh(struct buffer_head *bh)
{
        atomic_inc(&bh->b_count);
}

static inline __attribute__((always_inline)) void put_bh(struct buffer_head *bh)
{
        __asm__ __volatile__("": : :"memory");
        atomic_dec(&bh->b_count);
}

static inline __attribute__((always_inline)) void brelse(struct buffer_head *bh)
{
 if (bh)
  __brelse(bh);
}

static inline __attribute__((always_inline)) void bforget(struct buffer_head *bh)
{
 if (bh)
  __bforget(bh);
}

static inline __attribute__((always_inline)) struct buffer_head *
sb_bread(struct super_block *sb, sector_t block)
{
 return __bread(sb->s_bdev, block, sb->s_blocksize);
}

static inline __attribute__((always_inline)) void
sb_breadahead(struct super_block *sb, sector_t block)
{
 __breadahead(sb->s_bdev, block, sb->s_blocksize);
}

static inline __attribute__((always_inline)) struct buffer_head *
sb_getblk(struct super_block *sb, sector_t block)
{
 return __getblk(sb->s_bdev, block, sb->s_blocksize);
}

static inline __attribute__((always_inline)) struct buffer_head *
sb_find_get_block(struct super_block *sb, sector_t block)
{
 return __find_get_block(sb->s_bdev, block, sb->s_blocksize);
}

static inline __attribute__((always_inline)) void
map_bh(struct buffer_head *bh, struct super_block *sb, sector_t block)
{
 set_buffer_mapped(bh);
 bh->b_bdev = sb->s_bdev;
 bh->b_blocknr = block;
 bh->b_size = sb->s_blocksize;
}






static inline __attribute__((always_inline)) void wait_on_buffer(struct buffer_head *bh)
{
 do { __might_sleep("include/linux/buffer_head.h", 293); cond_resched(); } while (0);
 if (buffer_locked(bh) || ((&bh->b_count)->counter) == 0)
  __wait_on_buffer(bh);
}

static inline __attribute__((always_inline)) void lock_buffer(struct buffer_head *bh)
{
 do { __might_sleep("include/linux/buffer_head.h", 300); cond_resched(); } while (0);
 if (test_set_buffer_locked(bh))
  __lock_buffer(bh);
}















extern void mxcsr_feature_mask_init(void);
extern void init_fpu(struct task_struct *);

extern void kernel_fpu_begin(void);

static inline __attribute__((always_inline)) void __save_init_fpu( struct task_struct *tsk )
{

        asm volatile ("661:\n\t" "fnsave %1 ;fwait;" ".byte 0x90\n" ".byte 0x8d,0xb4,0x26,0x00,0x00,0x00,0x00\n" ".byte 0x8d,0x74,0x26,0x00\n" "\n662:\n" ".section .altinstructions,\"a\"\n" "  .align 4\n" "  .long 661b\n" "  .long 663f\n" "  .byte %c0\n" "  .byte 662b-661b\n" "  .byte 664f-663f\n" ".previous\n" ".section .altinstr_replacement,\"ax\"\n" "663:\n\t" "fxsave %1\n" "bt $7,%2 ; jnc 1f ; fnclex\n1:" "\n664:\n" ".previous" :: "i" ((0*32+24)), "m" (tsk->thread.i387.fxsave), "m" (tsk->thread.i387.fxsave.swd) : "memory");

 asm volatile ("661:\n\t" ".byte 0x90\n" ".byte 0x8d,0xb4,0x26,0x00,0x00,0x00,0x00\n" ".byte 0x89,0xf6\n" "\n662:\n" ".section .altinstructions,\"a\"\n" "  .align 4\n" "  .long 661b\n" "  .long 663f\n" "  .byte %c0\n" "  .byte 662b-661b\n" "  .byte 664f-663f\n" ".previous\n" ".section .altinstr_replacement,\"ax\"\n" "663:\n\t" "emms\n\t" "fildl %1" "\n664:\n" ".previous" :: "i" ((3*32+10)), "m" (((*((void)(0), &per_cpu__kstat)).cpustat.user)));






 (tsk)->thread_info->status &= ~0x0001;
}

static inline __attribute__((always_inline)) void save_init_fpu( struct task_struct *tsk )
{
 do { } while (0);
 __save_init_fpu(tsk);
 __asm__ __volatile__("movl %0,%%cr0": :"r" (8 | ({ unsigned int __dummy; __asm__ __volatile__( "movl %%cr0,%0\n\t" :"=r" (__dummy)); __dummy; })));
 do { } while (0);
}

extern unsigned short get_fpu_cwd( struct task_struct *tsk );
extern unsigned short get_fpu_swd( struct task_struct *tsk );
extern unsigned short get_fpu_mxcsr( struct task_struct *tsk );




extern int save_i387( struct _fpstate *buf );
extern int restore_i387( struct _fpstate *buf );




extern int get_fpregs( struct user_i387_struct *buf,
         struct task_struct *tsk );
extern int set_fpregs( struct task_struct *tsk,
         struct user_i387_struct *buf );

extern int get_fpxregs( struct user_fxsr_struct *buf,
   struct task_struct *tsk );
extern int set_fpxregs( struct task_struct *tsk,
   struct user_fxsr_struct *buf );




extern int dump_fpu( struct pt_regs *regs,
       struct user_i387_struct *fpu );


static inline __attribute__((always_inline)) int
arch_prepare_suspend(void)
{



 if (!(__builtin_constant_p((0*32+ 3)) ? constant_test_bit(((0*32+ 3)),(boot_cpu_data.x86_capability)) : variable_test_bit(((0*32+ 3)),(boot_cpu_data.x86_capability)))) {
  printk("<3>" "PSE is required for swsusp.\n");
  return -1;
 }
 return 0;
}


struct saved_context {
   u16 es, fs, gs, ss;
 unsigned long cr0, cr2, cr3, cr4;
 u16 gdt_pad;
 u16 gdt_limit;
 unsigned long gdt_base;
 u16 idt_pad;
 u16 idt_limit;
 unsigned long idt_base;
 u16 ldt;
 u16 tss;
 unsigned long tr;
 unsigned long safety;
 unsigned long return_address;
} __attribute__((packed));


extern unsigned long saved_eip;
extern unsigned long saved_esp;
extern unsigned long saved_ebp;
extern unsigned long saved_ebx;
extern unsigned long saved_esi;
extern unsigned long saved_edi;

static inline __attribute__((always_inline)) void acpi_save_register_state(unsigned long return_point)
{
 saved_eip = return_point;
 asm volatile ("movl %%esp,%0" : "=m" (saved_esp));
 asm volatile ("movl %%ebp,%0" : "=m" (saved_ebp));
 asm volatile ("movl %%ebx,%0" : "=m" (saved_ebx));
 asm volatile ("movl %%edi,%0" : "=m" (saved_edi));
 asm volatile ("movl %%esi,%0" : "=m" (saved_esi));
}




extern int acpi_save_state_mem(void);




static inline __attribute__((always_inline)) int current_is_kswapd(void)
{
 return __vericon_dummy_current->flags & 0x00040000;
}

union swap_header {
 struct {
  char reserved[(1UL << 12) - 10];
  char magic[10];
 } magic;
 struct {
  char bootbits[1024];
  __u32 version;
  __u32 last_page;
  __u32 nr_badpages;
  unsigned char sws_uuid[16];
  unsigned char sws_volume[16];
  __u32 padding[117];
  __u32 badpages[1];
 } info;
};





typedef struct {
 unsigned long val;
} swp_entry_t;





struct reclaim_state {
 unsigned long reclaimed_slab;
};



struct address_space;
struct sysinfo;
struct writeback_control;
struct zone;

struct swap_extent {
 struct list_head list;
 unsigned long start_page;
 unsigned long nr_pages;
 sector_t start_block;
};

enum {
 SWP_USED = (1 << 0),
 SWP_WRITEOK = (1 << 1),
 SWP_ACTIVE = (SWP_USED | SWP_WRITEOK),

 SWP_SCANNING = (1 << 8),
};

struct swap_info_struct {
 unsigned int flags;
 int prio;
 struct file *swap_file;
 struct block_device *bdev;
 struct list_head extent_list;
 struct swap_extent *curr_swap_extent;
 unsigned old_block_size;
 unsigned short * swap_map;
 unsigned int lowest_bit;
 unsigned int highest_bit;
 unsigned int cluster_next;
 unsigned int cluster_nr;
 unsigned int pages;
 unsigned int max;
 unsigned int inuse_pages;
 int next;
};

struct swap_list_t {
 int head;
 int next;
};





extern void out_of_memory(struct zonelist *zonelist, gfp_t gfp_mask, int order);


extern void swapin_readahead(swp_entry_t, unsigned long, struct vm_area_struct *);


extern unsigned long totalram_pages;
extern unsigned long totalhigh_pages;
extern unsigned long totalreserve_pages;
extern long nr_swap_pages;
extern unsigned int nr_free_pages(void);
extern unsigned int nr_free_pages_pgdat(pg_data_t *pgdat);
extern unsigned int nr_free_buffer_pages(void);
extern unsigned int nr_free_pagecache_pages(void);


extern void lru_cache_add(struct page *) __attribute__((regparm(3)));
extern void lru_cache_add_active(struct page *) __attribute__((regparm(3)));
extern void activate_page(struct page *) __attribute__((regparm(3)));
extern void mark_page_accessed(struct page *) __attribute__((regparm(3)));
extern void lru_add_drain(void);
extern int lru_add_drain_all(void);
extern int rotate_reclaimable_page(struct page *page);
extern void swap_setup(void);


extern unsigned long try_to_free_pages(struct zone **, gfp_t);
extern unsigned long shrink_all_memory(unsigned long nr_pages);
extern int vm_swappiness;
extern int remove_mapping(struct address_space *mapping, struct page *page);
extern long vm_total_pages;

static inline __attribute__((always_inline)) int zone_reclaim(struct zone *z, gfp_t mask, unsigned int order)
{
 return 0;
}


extern int kswapd_run(int nid);



extern int shmem_unuse(swp_entry_t entry, struct page *page);


extern void swap_unplug_io_fn(struct backing_dev_info *, struct page *);



extern int swap_readpage(struct file *, struct page *);
extern int swap_writepage(struct page *page, struct writeback_control *wbc);
extern int rw_swap_page_sync(int, swp_entry_t, struct page *);


extern struct address_space swapper_space;

extern void show_swap_cache_info(void);
extern int add_to_swap(struct page *, gfp_t);
extern void __delete_from_swap_cache(struct page *);
extern void delete_from_swap_cache(struct page *);
extern int move_to_swap_cache(struct page *, swp_entry_t);
extern int move_from_swap_cache(struct page *, unsigned long,
  struct address_space *);
extern void free_page_and_swap_cache(struct page *);
extern void free_pages_and_swap_cache(struct page **, int);
extern struct page * lookup_swap_cache(swp_entry_t);
extern struct page * read_swap_cache_async(swp_entry_t, struct vm_area_struct *vma,
        unsigned long addr);

extern long total_swap_pages;
extern unsigned int nr_swapfiles;
extern void si_swapinfo(struct sysinfo *);
extern swp_entry_t get_swap_page(void);
extern swp_entry_t get_swap_page_of_type(int);
extern int swap_duplicate(swp_entry_t);
extern int valid_swaphandles(swp_entry_t, unsigned long *);
extern void swap_free(swp_entry_t);
extern void free_swap_and_cache(swp_entry_t);
extern int swap_type_of(dev_t);
extern unsigned int count_swap_pages(int, int);
extern sector_t map_swap_page(struct swap_info_struct *, unsigned long);
extern struct swap_info_struct *get_swap_info_struct(unsigned);
extern int can_share_swap_page(struct page *);
extern int remove_exclusive_swap_page(struct page *);
struct backing_dev_info;

extern spinlock_t swap_lock;


extern struct mm_struct * swap_token_mm;
extern unsigned long swap_token_default_timeout;
extern void grab_swap_token(void);
extern void __put_swap_token(struct mm_struct *);

static inline __attribute__((always_inline)) int has_swap_token(struct mm_struct *mm)
{
 return (mm == swap_token_mm);
}

static inline __attribute__((always_inline)) void put_swap_token(struct mm_struct *mm)
{
 if (has_swap_token(mm))
  __put_swap_token(mm);
}

static inline __attribute__((always_inline)) void disable_swap_token(void)
{
 put_swap_token(swap_token_mm);
}






typedef struct pbe {
 unsigned long address;
 unsigned long orig_address;
 struct pbe *next;
} suspend_pagedir_t;

extern dev_t swsusp_resume_device;


extern int shrink_mem(void);


extern void drain_local_pages(void);
extern void mark_free_pages(struct zone *zone);



extern int software_suspend(void);


extern int pm_prepare_console(void);
extern void pm_restore_console(void);

static inline __attribute__((always_inline)) void disable_nonboot_cpus(void) {}
static inline __attribute__((always_inline)) void enable_nonboot_cpus(void) {}


void save_processor_state(void);
void restore_processor_state(void);
struct saved_context;
void __save_processor_state(struct saved_context *ctxt);
void __restore_processor_state(struct saved_context *ctxt);
unsigned long get_safe_page(gfp_t gfp_mask);







struct pollfd {
 int fd;
 short events;
 short revents;
};


struct poll_table_struct;




typedef void (*poll_queue_proc)(struct file *, wait_queue_head_t *, struct poll_table_struct *);

typedef struct poll_table_struct {
 poll_queue_proc qproc;
} poll_table;

static inline __attribute__((always_inline)) void poll_wait(struct file * filp, wait_queue_head_t * wait_address, poll_table *p)
{
 if (p && wait_address)
  p->qproc(filp, wait_address, p);
}

static inline __attribute__((always_inline)) void init_poll_funcptr(poll_table *pt, poll_queue_proc qproc)
{
 pt->qproc = qproc;
}

struct poll_table_entry {
 struct file * filp;
 wait_queue_t wait;
 wait_queue_head_t * wait_address;
};




struct poll_wqueues {
 poll_table pt;
 struct poll_table_page * table;
 int error;
 int inline_index;
 struct poll_table_entry inline_entries[((832 - 256) / sizeof(struct poll_table_entry))];
};

extern void poll_initwait(struct poll_wqueues *pwq);
extern void poll_freewait(struct poll_wqueues *pwq);





typedef struct {
 unsigned long *in, *out, *ex;
 unsigned long *res_in, *res_out, *res_ex;
} fd_set_bits;

static inline __attribute__((always_inline))
int get_fd_set(unsigned long nr, void *ufdset, unsigned long *fdset)
{
 nr = ((((nr)+(8*sizeof(long))-1)/(8*sizeof(long)))*sizeof(long));
 if (ufdset)
  return copy_from_user(fdset, ufdset, nr) ? -14 : 0;

 (__builtin_constant_p(0) ? (__builtin_constant_p((nr)) ? __constant_c_and_count_memset(((fdset)),((0x01010101UL*(unsigned char)(0))),((nr))) : __constant_c_memset(((fdset)),((0x01010101UL*(unsigned char)(0))),((nr)))) : (__builtin_constant_p((nr)) ? __memset_generic((((fdset))),(((0))),(((nr)))) : __memset_generic(((fdset)),((0)),((nr)))));
 return 0;
}

static inline __attribute__((always_inline)) unsigned long __attribute__((warn_unused_result))
set_fd_set(unsigned long nr, void *ufdset, unsigned long *fdset)
{
 if (ufdset)
  return __copy_to_user(ufdset, fdset, ((((nr)+(8*sizeof(long))-1)/(8*sizeof(long)))*sizeof(long)));
 return 0;
}

static inline __attribute__((always_inline))
void zero_fd_set(unsigned long nr, unsigned long *fdset)
{
 (__builtin_constant_p(0) ? (__builtin_constant_p((((((nr)+(8*sizeof(long))-1)/(8*sizeof(long)))*sizeof(long)))) ? __constant_c_and_count_memset(((fdset)),((0x01010101UL*(unsigned char)(0))),((((((nr)+(8*sizeof(long))-1)/(8*sizeof(long)))*sizeof(long))))) : __constant_c_memset(((fdset)),((0x01010101UL*(unsigned char)(0))),((((((nr)+(8*sizeof(long))-1)/(8*sizeof(long)))*sizeof(long)))))) : (__builtin_constant_p((((((nr)+(8*sizeof(long))-1)/(8*sizeof(long)))*sizeof(long)))) ? __memset_generic((((fdset))),(((0))),(((((((nr)+(8*sizeof(long))-1)/(8*sizeof(long)))*sizeof(long)))))) : __memset_generic(((fdset)),((0)),((((((nr)+(8*sizeof(long))-1)/(8*sizeof(long)))*sizeof(long)))))));
}



extern int do_select(int n, fd_set_bits *fds, s64 *timeout);
extern int do_sys_poll(struct pollfd * ufds, unsigned int nfds,
         s64 *timeout);




extern unsigned char _ctype[];

static inline __attribute__((always_inline)) unsigned char __tolower(unsigned char c)
{
 if ((((_ctype[(int)(unsigned char)(c)])&(0x01)) != 0))
  c -= 'A'-'a';
 return c;
}

static inline __attribute__((always_inline)) unsigned char __toupper(unsigned char c)
{
 if ((((_ctype[(int)(unsigned char)(c)])&(0x02)) != 0))
  c -= 'a'-'A';
 return c;
}






struct embedded_fd_set {
 unsigned long fds_bits[1];
};






struct fdtable {
 unsigned int max_fds;
 int max_fdset;
 struct file ** fd;
 fd_set *close_on_exec;
 fd_set *open_fds;
 struct rcu_head rcu;
 struct files_struct *free_files;
 struct fdtable *next;
};




struct files_struct {



 atomic_t count;
 struct fdtable *fdt;
 struct fdtable fdtab;



 spinlock_t file_lock ;
 int next_fd;
 struct embedded_fd_set close_on_exec_init;
 struct embedded_fd_set open_fds_init;
 struct file * fd_array[32];
};



extern void __fput(struct file *) __attribute__((regparm(3)));
extern void fput(struct file *) __attribute__((regparm(3)));

static inline __attribute__((always_inline)) void fput_light(struct file *file, int fput_needed)
{
 if (__builtin_expect(!!(fput_needed), 0))
  fput(file);
}

extern struct file * fget(unsigned int fd) __attribute__((regparm(3)));
extern struct file * fget_light(unsigned int fd, int *fput_needed) __attribute__((regparm(3)));
extern void set_close_on_exec(unsigned int fd, int flag) __attribute__((regparm(3)));
extern void put_filp(struct file *);
extern int get_unused_fd(void);
extern void put_unused_fd(unsigned int fd) __attribute__((regparm(3)));
struct kmem_cache;

extern struct file ** alloc_fd_array(int);
extern void free_fd_array(struct file **, int);

extern fd_set *alloc_fdset(int);
extern void free_fdset(fd_set *, int);

extern int expand_files(struct files_struct *, int nr);
extern void free_fdtable(struct fdtable *fdt);
extern void __attribute__ ((__section__ (".init.text"))) files_defer_init(void);

static inline __attribute__((always_inline)) struct file * fcheck_files(struct files_struct *files, unsigned int fd)
{
 struct file * file = ((void *)0);
 struct fdtable *fdt = (({ typeof((files)->fdt) _________p1 = (files)->fdt; do { } while(0); (_________p1); }));

 if (fd < fdt->max_fds)
  file = ({ typeof(fdt->fd[fd]) _________p1 = fdt->fd[fd]; do { } while(0); (_________p1); });
 return file;
}






extern void fd_install(unsigned int fd, struct file * file) __attribute__((regparm(3)));

struct task_struct;

struct files_struct *get_files_struct(struct task_struct *);
void put_files_struct(struct files_struct *fs) __attribute__((regparm(3)));









static void autostart_arrays (int part);


static struct list_head pers_list = { &(pers_list), &(pers_list) };
static spinlock_t pers_lock = (spinlock_t) { .raw_lock = { 1 }, .magic = 0xdead4ead, .owner = ((void *)-1L), .owner_cpu = -1, };

static void md_print_devices(void);

static int sysctl_speed_limit_min = 1000;
static int sysctl_speed_limit_max = 200000;
static inline __attribute__((always_inline)) int speed_min(mddev_t *mddev)
{
 return mddev->sync_speed_min ?
  mddev->sync_speed_min : sysctl_speed_limit_min;
}

static inline __attribute__((always_inline)) int speed_max(mddev_t *mddev)
{
 return mddev->sync_speed_max ?
  mddev->sync_speed_max : sysctl_speed_limit_max;
}

static struct ctl_table_header *raid_table_header;

static ctl_table raid_table[] = {
 {
  .ctl_name = DEV_RAID_SPEED_LIMIT_MIN,
  .procname = "speed_limit_min",
  .data = &sysctl_speed_limit_min,
  .maxlen = sizeof(int),
  .mode = (00400|00040|00004)|00200,
  .proc_handler = &proc_dointvec,
 },
 {
  .ctl_name = DEV_RAID_SPEED_LIMIT_MAX,
  .procname = "speed_limit_max",
  .data = &sysctl_speed_limit_max,
  .maxlen = sizeof(int),
  .mode = (00400|00040|00004)|00200,
  .proc_handler = &proc_dointvec,
 },
 { .ctl_name = 0 }
};

static ctl_table raid_dir_table[] = {
 {
  .ctl_name = DEV_RAID,
  .procname = "raid",
  .maxlen = 0,
  .mode = (00400|00040|00004)|(00100|00010|00001),
  .child = raid_table,
 },
 { .ctl_name = 0 }
};

static ctl_table raid_root_table[] = {
 {
  .ctl_name = CTL_DEV,
  .procname = "dev",
  .maxlen = 0,
  .mode = 0555,
  .child = raid_dir_table,
 },
 { .ctl_name = 0 }
};

static struct block_device_operations md_fops;

static int start_readonly;

static wait_queue_head_t md_event_waiters = { .lock = (spinlock_t) { .raw_lock = { 1 }, .magic = 0xdead4ead, .owner = ((void *)-1L), .owner_cpu = -1, }, .task_list = { &(md_event_waiters).task_list, &(md_event_waiters).task_list } };
static atomic_t md_event_count;
void md_new_event(mddev_t *mddev)
{
 atomic_inc(&md_event_count);
 __wake_up(&md_event_waiters, 2 | 1, 1, ((void *)0));
 sysfs_notify(&mddev->kobj, ((void *)0), "sync_action");
}
extern typeof(md_new_event) md_new_event; extern void *__crc_md_new_event __attribute__((weak)); static const unsigned long __kcrctab_md_new_event __attribute__((__used__)) __attribute__((section("__kcrctab" "_gpl"), unused)) = (unsigned long) &__crc_md_new_event; static const char __kstrtab_md_new_event[] __attribute__((section("__ksymtab_strings"))) = "" "md_new_event"; static const struct kernel_symbol __ksymtab_md_new_event __attribute__((__used__)) __attribute__((section("__ksymtab" "_gpl"), unused)) = { (unsigned long)&md_new_event, __kstrtab_md_new_event };




static void md_new_event_inintr(mddev_t *mddev)
{
 atomic_inc(&md_event_count);
 __wake_up(&md_event_waiters, 2 | 1, 1, ((void *)0));
}





static struct list_head all_mddevs = { &(all_mddevs), &(all_mddevs) };
static spinlock_t all_mddevs_lock = (spinlock_t) { .raw_lock = { 1 }, .magic = 0xdead4ead, .owner = ((void *)-1L), .owner_cpu = -1, };

static int md_fail_request (request_queue_t *q, struct bio *bio)
{
 bio_endio((bio), (bio->bi_size), -5);
 return 0;
}

static inline __attribute__((always_inline)) mddev_t *mddev_get(mddev_t *mddev)
{
 atomic_inc(&mddev->active);
 return mddev;
}

static void mddev_put(mddev_t *mddev)
{
 if (!(_atomic_dec_and_lock(&mddev->active, &all_mddevs_lock)))
  return;
 if (!mddev->raid_disks && list_empty(&mddev->disks)) {
  list_del(&mddev->all_mddevs);
  _spin_unlock(&all_mddevs_lock);
  blk_cleanup_queue(mddev->queue);
  kobject_unregister(&mddev->kobj);
 } else
  _spin_unlock(&all_mddevs_lock);
}

extern mddev_t * mddev_find(dev_t unit);
/* [kohei]
static mddev_t * mddev_find(dev_t unit)
{
 mddev_t *mddev, *new = ((void *)0);

 retry:
 _spin_lock(&all_mddevs_lock);
 for (mddev = ({ const typeof( ((typeof(*mddev) *)0)->all_mddevs ) *__mptr = ((&all_mddevs)->next); (typeof(*mddev) *)( (char *)__mptr - __builtin_offsetof(typeof(*mddev),all_mddevs) );}); prefetch(mddev->all_mddevs.next), &mddev->all_mddevs != (&all_mddevs); mddev = ({ const typeof( ((typeof(*mddev) *)0)->all_mddevs ) *__mptr = (mddev->all_mddevs.next); (typeof(*mddev) *)( (char *)__mptr - __builtin_offsetof(typeof(*mddev),all_mddevs) );}))
  if (mddev->unit == unit) {
   mddev_get(mddev);
   _spin_unlock(&all_mddevs_lock);
   kfree(new);
   return mddev;
  }

 if (new) {
  list_add(&new->all_mddevs, &all_mddevs);
  _spin_unlock(&all_mddevs_lock);
  return new;
 }
 _spin_unlock(&all_mddevs_lock);

 new = kzalloc(sizeof(*new), ((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u)));
 if (!new)
  return ((void *)0);

 new->unit = unit;
 if (((unsigned int) ((unit) >> 20)) == 9)
  new->md_minor = ((unsigned int) ((unit) & ((1U << 20) - 1)));
 else
  new->md_minor = ((unsigned int) ((unit) & ((1U << 20) - 1))) >> 6;

 do { static struct lock_class_key __key; __mutex_init((&new->reconfig_mutex), "&new->reconfig_mutex", &__key); } while (0);
 INIT_LIST_HEAD(&new->disks);
 INIT_LIST_HEAD(&new->all_mddevs);
 init_timer(&new->safemode_timer);
 (((&new->active)->counter) = (1));
 do { static struct lock_class_key __key; __spin_lock_init((&new->write_lock), "&new->write_lock", &__key); } while (0);
 init_waitqueue_head(&new->sb_wait);

 new->queue = blk_alloc_queue(((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u)));
 if (!new->queue) {
  kfree(new);
  return ((void *)0);
 }
 set_bit(0, &new->queue->queue_flags);

 blk_queue_make_request(new->queue, md_fail_request);

 goto retry;
}
*/

static inline __attribute__((always_inline)) int mddev_lock(mddev_t * mddev)
{
 return mutex_lock_interruptible(&mddev->reconfig_mutex);
}

static inline __attribute__((always_inline)) int mddev_trylock(mddev_t * mddev)
{
 return mutex_trylock(&mddev->reconfig_mutex);
}

static inline __attribute__((always_inline)) void mddev_unlock(mddev_t * mddev)
{
 mutex_unlock(&mddev->reconfig_mutex);

 md_wakeup_thread(mddev->thread);
}

extern mdk_rdev_t * find_rdev_nr(mddev_t *mddev, int nr);
/* [kohei]
static mdk_rdev_t * find_rdev_nr(mddev_t *mddev, int nr)
{
 mdk_rdev_t * rdev;
 struct list_head *tmp;

 for ((tmp) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp) = (tmp)->next, (tmp)->prev != &((mddev)->disks) ; ) {
  if (rdev->desc_nr == nr)
   return rdev;
 }
 return ((void *)0);
}
*/

extern mdk_rdev_t * find_rdev(mddev_t * mddev, dev_t dev);
/* [kohei]
static mdk_rdev_t * find_rdev(mddev_t * mddev, dev_t dev)
{
 struct list_head *tmp;
 mdk_rdev_t *rdev;

 for ((tmp) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp) = (tmp)->next, (tmp)->prev != &((mddev)->disks) ; ) {
  if (rdev->bdev->bd_dev == dev)
   return rdev;
 }
 return ((void *)0);
}
*/

extern struct mdk_personality *find_pers(int level, char *clevel);
/* [kohei]
static struct mdk_personality *find_pers(int level, char *clevel)
{
 struct mdk_personality *pers;
 for (pers = ({ const typeof( ((typeof(*pers) *)0)->list ) *__mptr = ((&pers_list)->next); (typeof(*pers) *)( (char *)__mptr - __builtin_offsetof(typeof(*pers),list) );}); prefetch(pers->list.next), &pers->list != (&pers_list); pers = ({ const typeof( ((typeof(*pers) *)0)->list ) *__mptr = (pers->list.next); (typeof(*pers) *)( (char *)__mptr - __builtin_offsetof(typeof(*pers),list) );})) {
  if (level != (-1000000) && pers->level == level)
   return pers;
  if (strcmp(pers->name, clevel)==0)
   return pers;
 }
 return ((void *)0);
}
*/
static inline __attribute__((always_inline)) sector_t calc_dev_sboffset(struct block_device *bdev)
{
 sector_t size = bdev->bd_inode->i_size >> 10;
 return ((size & ~(((64 * 1024) / (1<<10)) - 1)) - ((64 * 1024) / (1<<10)));
}

static sector_t calc_dev_size(mdk_rdev_t *rdev, unsigned chunk_size)
{
 sector_t size;

 size = rdev->sb_offset;

 if (chunk_size)
  size &= ~((sector_t)chunk_size/1024 - 1);
 return size;
}

static int alloc_disk_sb(mdk_rdev_t * rdev)
{
 if (rdev->sb_page)
  { printk("md: bug in file %s, line %d\n", "drivers/md/md.c", 362); md_print_devices(); };

 rdev->sb_page = alloc_pages_node(((0)), ((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u)), 0);
 if (!rdev->sb_page) {
  printk("<1>" "md: out of memory.\n");
  return -22;
 }

 return 0;
}

static void free_disk_sb(mdk_rdev_t * rdev)
{
 if (rdev->sb_page) {
  put_page(rdev->sb_page);
  rdev->sb_loaded = 0;
  rdev->sb_page = ((void *)0);
  rdev->sb_offset = 0;
  rdev->size = 0;
 }
}


static int super_written(struct bio *bio, unsigned int bytes_done, int error)
{
 mdk_rdev_t *rdev = bio->bi_private;
 mddev_t *mddev = rdev->mddev;
 if (bio->bi_size)
  return 1;

 if (error || !(__builtin_constant_p(0) ? constant_test_bit((0),(&bio->bi_flags)) : variable_test_bit((0),(&bio->bi_flags))))
  md_error(mddev, rdev);

 if (atomic_dec_and_test(&mddev->pending_writes))
  __wake_up(&mddev->sb_wait, 2 | 1, 1, ((void *)0));
 bio_put(bio);
 return 0;
}

static int super_written_barrier(struct bio *bio, unsigned int bytes_done, int error)
{
 struct bio *bio2 = bio->bi_private;
 mdk_rdev_t *rdev = bio2->bi_private;
 mddev_t *mddev = rdev->mddev;
 if (bio->bi_size)
  return 1;

 if (!(__builtin_constant_p(0) ? constant_test_bit((0),(&bio->bi_flags)) : variable_test_bit((0),(&bio->bi_flags))) &&
     error == -95) {
  unsigned long flags;

  set_bit(5, &rdev->flags);
  mddev->barriers_work = 0;
  flags = _spin_lock_irqsave(&mddev->write_lock);
  bio2->bi_next = mddev->biolist;
  mddev->biolist = bio2;
  _spin_unlock_irqrestore(&mddev->write_lock, flags);
  __wake_up(&mddev->sb_wait, 2 | 1, 1, ((void *)0));
  bio_put(bio);
  return 0;
 }
 bio_put(bio2);
 bio->bi_private = rdev;
 return super_written(bio, bytes_done, error);
}

void md_super_write(mddev_t *mddev, mdk_rdev_t *rdev,
     sector_t sector, int size, struct page *page)
{

 struct bio *bio = bio_alloc(((( gfp_t)0x10u)), 1);
 int rw = (1<<0) | (1<<4);

 bio->bi_bdev = rdev->bdev;
 bio->bi_sector = sector;
 bio_add_page(bio, page, size, 0);
 bio->bi_private = rdev;
 bio->bi_end_io = super_written;
 bio->bi_rw = rw;

 atomic_inc(&mddev->pending_writes);
 if (!(__builtin_constant_p(5) ? constant_test_bit((5),(&rdev->flags)) : variable_test_bit((5),(&rdev->flags)))) {
  struct bio *rbio;
  rw |= (1<<2);
  rbio = bio_clone(bio, ((( gfp_t)0x10u)));
  rbio->bi_private = bio;
  rbio->bi_end_io = super_written_barrier;
  submit_bio(rw, rbio);
 } else
  submit_bio(rw, bio);
}

void md_super_wait(mddev_t *mddev)
{



 wait_queue_t wq = { .private = __vericon_dummy_current, .func = autoremove_wake_function, .task_list = { &((wq).task_list), &((wq).task_list) }, };
 for(;;) {
  prepare_to_wait(&mddev->sb_wait, &wq, 2);
  if (((&mddev->pending_writes)->counter)==0)
   break;
  while (mddev->biolist) {
   struct bio *bio;
   _spin_lock_irq(&mddev->write_lock);
   bio = mddev->biolist;
   mddev->biolist = bio->bi_next ;
   bio->bi_next = ((void *)0);
   _spin_unlock_irq(&mddev->write_lock);
   submit_bio(bio->bi_rw, bio);
  }
  schedule();
 }
 finish_wait(&mddev->sb_wait, &wq);
}

static int bi_complete(struct bio *bio, unsigned int bytes_done, int error)
{
 if (bio->bi_size)
  return 1;

 complete((struct completion*)bio->bi_private);
 return 0;
}

int sync_page_io(struct block_device *bdev, sector_t sector, int size,
     struct page *page, int rw)
{
 struct bio *bio = bio_alloc(((( gfp_t)0x10u)), 1);
 struct completion event;
 int ret;

 rw |= (1 << 4);

 bio->bi_bdev = bdev;
 bio->bi_sector = sector;
 bio_add_page(bio, page, size, 0);
 init_completion(&event);
 bio->bi_private = &event;
 bio->bi_end_io = bi_complete;
 submit_bio(rw, bio);
 wait_for_completion(&event);

 ret = (__builtin_constant_p(0) ? constant_test_bit((0),(&bio->bi_flags)) : variable_test_bit((0),(&bio->bi_flags)));
 bio_put(bio);
 return ret;
}
extern typeof(sync_page_io) sync_page_io; extern void *__crc_sync_page_io __attribute__((weak)); static const unsigned long __kcrctab_sync_page_io __attribute__((__used__)) __attribute__((section("__kcrctab" "_gpl"), unused)) = (unsigned long) &__crc_sync_page_io; static const char __kstrtab_sync_page_io[] __attribute__((section("__ksymtab_strings"))) = "" "sync_page_io"; static const struct kernel_symbol __ksymtab_sync_page_io __attribute__((__used__)) __attribute__((section("__ksymtab" "_gpl"), unused)) = { (unsigned long)&sync_page_io, __kstrtab_sync_page_io };

static int read_disk_sb(mdk_rdev_t * rdev, int size)
{
 char b[32];
 if (!rdev->sb_page) {
  { printk("md: bug in file %s, line %d\n", "drivers/md/md.c", 523); md_print_devices(); };
  return -22;
 }
 if (rdev->sb_loaded)
  return 0;


 if (!sync_page_io(rdev->bdev, rdev->sb_offset<<1, size, rdev->sb_page, 0))
  goto fail;
 rdev->sb_loaded = 1;
 return 0;

fail:
 printk("<4>" "md: disabled device %s, could not read superblock.\n",
  bdevname(rdev->bdev,b));
 return -22;
}

static int uuid_equal(mdp_super_t *sb1, mdp_super_t *sb2)
{
 if ( (sb1->set_uuid0 == sb2->set_uuid0) &&
  (sb1->set_uuid1 == sb2->set_uuid1) &&
  (sb1->set_uuid2 == sb2->set_uuid2) &&
  (sb1->set_uuid3 == sb2->set_uuid3))

  return 1;

 return 0;
}


static int sb_equal(mdp_super_t *sb1, mdp_super_t *sb2)
{
 int ret;
 mdp_super_t *tmp1, *tmp2;

 tmp1 = kmalloc(sizeof(*tmp1),((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u)));
 tmp2 = kmalloc(sizeof(*tmp2),((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u)));

 if (!tmp1 || !tmp2) {
  ret = 0;
  printk("<6>" "md.c: sb1 is not equal to sb2!\n");
  goto abort;
 }

 *tmp1 = *sb1;
 *tmp2 = *sb2;




 tmp1->nr_disks = 0;
 tmp2->nr_disks = 0;

 if (__builtin_memcmp(tmp1, tmp2, 32 * 4))
  ret = 0;
 else
  ret = 1;

abort:
 kfree(tmp1);
 kfree(tmp2);
 return ret;
}

extern unsigned int calc_sb_csum(mdp_super_t * sb);
/* [kohei]
static unsigned int calc_sb_csum(mdp_super_t * sb)
{
 unsigned int disk_csum, csum;

 disk_csum = sb->sb_csum;
 sb->sb_csum = 0;
 csum = csum_partial((void *)sb, 4096, 0);
 sb->sb_csum = disk_csum;
 return csum;
}
*/

struct super_type {
 char *name;
 struct module *owner;
 int (*load_super)(mdk_rdev_t *rdev, mdk_rdev_t *refdev, int minor_version);
 int (*validate_super)(mddev_t *mddev, mdk_rdev_t *rdev);
 void (*sync_super)(mddev_t *mddev, mdk_rdev_t *rdev);
};




static int super_90_load(mdk_rdev_t *rdev, mdk_rdev_t *refdev, int minor_version)
{
 char b[32], b2[32];
 mdp_super_t *sb;
 int ret;
 sector_t sb_offset;







 sb_offset = calc_dev_sboffset(rdev->bdev);
 rdev->sb_offset = sb_offset;

 ret = read_disk_sb(rdev, 4096);
 if (ret) return ret;

 ret = -22;

 bdevname(rdev->bdev, b);
 sb = (mdp_super_t*)page_address(rdev->sb_page);

 if (sb->md_magic != 0xa92b4efc) {
  printk("<3>" "md: invalid raid superblock magic on %s\n",
         b);
  goto abort;
 }

 if (sb->major_version != 0 ||
     sb->minor_version < 90 ||
     sb->minor_version > 91) {
  printk("<4>" "Bad version number %d.%d on %s\n",
   sb->major_version, sb->minor_version,
   b);
  goto abort;
 }

 if (sb->raid_disks <= 0)
  goto abort;

 if (csum_fold(calc_sb_csum(sb)) != csum_fold(sb->sb_csum)) {
  printk("<4>" "md: invalid superblock checksum on %s\n",
   b);
  goto abort;
 }

 rdev->preferred_minor = sb->md_minor;
 rdev->data_offset = 0;
 rdev->sb_size = 4096;

 if (sb->level == (-4))
  rdev->desc_nr = -1;
 else
  rdev->desc_nr = sb->this_disk.number;

 if (refdev == 0)
  ret = 1;
 else {
  __u64 ev1, ev2;
  mdp_super_t *refsb = (mdp_super_t*)page_address(refdev->sb_page);
  if (!uuid_equal(refsb, sb)) {
   printk("<4>" "md: %s has different UUID to %s\n",
    b, bdevname(refdev->bdev,b2));
   goto abort;
  }
  if (!sb_equal(refsb, sb)) {
   printk("<4>" "md: %s has same UUID"
          " but different superblock to %s\n",
          b, bdevname(refdev->bdev, b2));
   goto abort;
  }
  ev1 = md_event(sb);
  ev2 = md_event(refsb);
  if (ev1 > ev2)
   ret = 1;
  else
   ret = 0;
 }
 rdev->size = calc_dev_size(rdev, sb->chunk_size);

 if (rdev->size < sb->size && sb->level > 1)

  ret = -22;

 abort:
 return ret;
}




static int super_90_validate(mddev_t *mddev, mdk_rdev_t *rdev)
{
 mdp_disk_t *desc;
 mdp_super_t *sb = (mdp_super_t *)page_address(rdev->sb_page);
 __u64 ev1 = md_event(sb);

 rdev->raid_disk = -1;
 rdev->flags = 0;
 if (mddev->raid_disks == 0) {
  mddev->major_version = 0;
  mddev->minor_version = sb->minor_version;
  mddev->patch_version = sb->patch_version;
  mddev->persistent = ! sb->not_persistent;
  mddev->chunk_size = sb->chunk_size;
  mddev->ctime = sb->ctime;
  mddev->utime = sb->utime;
  mddev->level = sb->level;
  mddev->clevel[0] = 0;
  mddev->layout = sb->layout;
  mddev->raid_disks = sb->raid_disks;
  mddev->size = sb->size;
  mddev->events = ev1;
  mddev->bitmap_offset = 0;
  mddev->default_bitmap_offset = 4096 >> 9;

  if (mddev->minor_version >= 91) {
   mddev->reshape_position = sb->reshape_position;
   mddev->delta_disks = sb->delta_disks;
   mddev->new_level = sb->new_level;
   mddev->new_layout = sb->new_layout;
   mddev->new_chunk = sb->new_chunk;
  } else {
   mddev->reshape_position = (~(sector_t)0);
   mddev->delta_disks = 0;
   mddev->new_level = mddev->level;
   mddev->new_layout = mddev->layout;
   mddev->new_chunk = mddev->chunk_size;
  }

  if (sb->state & (1<<0))
   mddev->recovery_cp = (~(sector_t)0);
  else {
   if (sb->events_hi == sb->cp_events_hi &&
    sb->events_lo == sb->cp_events_lo) {
    mddev->recovery_cp = sb->recovery_cp;
   } else
    mddev->recovery_cp = 0;
  }

  (__builtin_constant_p(4) ? __constant_memcpy((mddev->uuid+0),(&sb->set_uuid0),(4)) : __memcpy((mddev->uuid+0),(&sb->set_uuid0),(4)));
  (__builtin_constant_p(4) ? __constant_memcpy((mddev->uuid+4),(&sb->set_uuid1),(4)) : __memcpy((mddev->uuid+4),(&sb->set_uuid1),(4)));
  (__builtin_constant_p(4) ? __constant_memcpy((mddev->uuid+8),(&sb->set_uuid2),(4)) : __memcpy((mddev->uuid+8),(&sb->set_uuid2),(4)));
  (__builtin_constant_p(4) ? __constant_memcpy((mddev->uuid+12),(&sb->set_uuid3),(4)) : __memcpy((mddev->uuid+12),(&sb->set_uuid3),(4)));

  mddev->max_disks = 27;

  if (sb->state & (1<<8) &&
      mddev->bitmap_file == ((void *)0)) {
   if (mddev->level != 1 && mddev->level != 4
       && mddev->level != 5 && mddev->level != 6
       && mddev->level != 10) {

    printk("<4>" "md: bitmaps not supported for this level.\n");
    return -22;
   }
   mddev->bitmap_offset = mddev->default_bitmap_offset;
  }

 } else if (mddev->pers == ((void *)0)) {

  ++ev1;
  if (ev1 < mddev->events)
   return -22;
 } else if (mddev->bitmap) {



  if (ev1 < mddev->bitmap->events_cleared)
   return 0;
 } else {
  if (ev1 < mddev->events)

   return 0;
 }

 if (mddev->level != (-4)) {
  desc = sb->disks + rdev->desc_nr;

  if (desc->state & (1<<0))
   set_bit(1, &rdev->flags);
  else if (desc->state & (1<<2) ) {

   set_bit(2, &rdev->flags);
   rdev->raid_disk = desc->raid_disk;
  }
  if (desc->state & (1<<9))
   set_bit(4, &rdev->flags);
 } else
  set_bit(2, &rdev->flags);
 return 0;
}


extern void super_90_sync(mddev_t *mddev, mdk_rdev_t *rdev);
/* [kohei]
static void super_90_sync(mddev_t *mddev, mdk_rdev_t *rdev)
{
 mdp_super_t *sb;
 struct list_head *tmp;
 mdk_rdev_t *rdev2;
 int next_spare = mddev->raid_disks;

 int i;
 int active=0, working=0,failed=0,spare=0,nr_disks=0;

 rdev->sb_size = 4096;

 sb = (mdp_super_t*)page_address(rdev->sb_page);

 (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(*sb))) ? __constant_c_and_count_memset(((sb)),((0x01010101UL*(unsigned char)(0))),((sizeof(*sb)))) : __constant_c_memset(((sb)),((0x01010101UL*(unsigned char)(0))),((sizeof(*sb))))) : (__builtin_constant_p((sizeof(*sb))) ? __memset_generic((((sb))),(((0))),(((sizeof(*sb))))) : __memset_generic(((sb)),((0)),((sizeof(*sb))))));

 sb->md_magic = 0xa92b4efc;
 sb->major_version = mddev->major_version;
 sb->patch_version = mddev->patch_version;
 sb->gvalid_words = 0;
 (__builtin_constant_p(4) ? __constant_memcpy((&sb->set_uuid0),(mddev->uuid+0),(4)) : __memcpy((&sb->set_uuid0),(mddev->uuid+0),(4)));
 (__builtin_constant_p(4) ? __constant_memcpy((&sb->set_uuid1),(mddev->uuid+4),(4)) : __memcpy((&sb->set_uuid1),(mddev->uuid+4),(4)));
 (__builtin_constant_p(4) ? __constant_memcpy((&sb->set_uuid2),(mddev->uuid+8),(4)) : __memcpy((&sb->set_uuid2),(mddev->uuid+8),(4)));
 (__builtin_constant_p(4) ? __constant_memcpy((&sb->set_uuid3),(mddev->uuid+12),(4)) : __memcpy((&sb->set_uuid3),(mddev->uuid+12),(4)));

 sb->ctime = mddev->ctime;
 sb->level = mddev->level;
 sb->size = mddev->size;
 sb->raid_disks = mddev->raid_disks;
 sb->md_minor = mddev->md_minor;
 sb->not_persistent = !mddev->persistent;
 sb->utime = mddev->utime;
 sb->state = 0;
 sb->events_hi = (mddev->events>>32);
 sb->events_lo = (u32)mddev->events;

 if (mddev->reshape_position == (~(sector_t)0))
  sb->minor_version = 90;
 else {
  sb->minor_version = 91;
  sb->reshape_position = mddev->reshape_position;
  sb->new_level = mddev->new_level;
  sb->delta_disks = mddev->delta_disks;
  sb->new_layout = mddev->new_layout;
  sb->new_chunk = mddev->new_chunk;
 }
 mddev->minor_version = sb->minor_version;
 if (mddev->in_sync)
 {
  sb->recovery_cp = mddev->recovery_cp;
  sb->cp_events_hi = (mddev->events>>32);
  sb->cp_events_lo = (u32)mddev->events;
  if (mddev->recovery_cp == (~(sector_t)0))
   sb->state = (1<< 0);
 } else
  sb->recovery_cp = 0;

 sb->layout = mddev->layout;
 sb->chunk_size = mddev->chunk_size;

 if (mddev->bitmap && mddev->bitmap_file == ((void *)0))
  sb->state |= (1<<8);

 sb->disks[0].state = (1<<3);
 for ((tmp) = ((mddev)->disks).next; (rdev2) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp) = (tmp)->next, (tmp)->prev != &((mddev)->disks) ; ) {
  mdp_disk_t *d;
  int desc_nr;
  if (rdev2->raid_disk >= 0 && (__builtin_constant_p(2) ? constant_test_bit((2),(&rdev2->flags)) : variable_test_bit((2),(&rdev2->flags)))
      && !(__builtin_constant_p(1) ? constant_test_bit((1),(&rdev2->flags)) : variable_test_bit((1),(&rdev2->flags))))
   desc_nr = rdev2->raid_disk;
  else
   desc_nr = next_spare++;
  rdev2->desc_nr = desc_nr;
  d = &sb->disks[rdev2->desc_nr];
  nr_disks++;
  d->number = rdev2->desc_nr;
  d->major = ((unsigned int) ((rdev2->bdev->bd_dev) >> 20));
  d->minor = ((unsigned int) ((rdev2->bdev->bd_dev) & ((1U << 20) - 1)));
  if (rdev2->raid_disk >= 0 && (__builtin_constant_p(2) ? constant_test_bit((2),(&rdev2->flags)) : variable_test_bit((2),(&rdev2->flags)))
      && !(__builtin_constant_p(1) ? constant_test_bit((1),(&rdev2->flags)) : variable_test_bit((1),(&rdev2->flags))))
   d->raid_disk = rdev2->raid_disk;
  else
   d->raid_disk = rdev2->desc_nr;
  if ((__builtin_constant_p(1) ? constant_test_bit((1),(&rdev2->flags)) : variable_test_bit((1),(&rdev2->flags))))
   d->state = (1<<0);
  else if ((__builtin_constant_p(2) ? constant_test_bit((2),(&rdev2->flags)) : variable_test_bit((2),(&rdev2->flags)))) {
   d->state = (1<<1);
   d->state |= (1<<2);
   active++;
   working++;
  } else {
   d->state = 0;
   spare++;
   working++;
  }
  if ((__builtin_constant_p(4) ? constant_test_bit((4),(&rdev2->flags)) : variable_test_bit((4),(&rdev2->flags))))
   d->state |= (1<<9);
 }

 for (i=0 ; i < mddev->raid_disks ; i++) {
  mdp_disk_t *d = &sb->disks[i];
  if (d->state == 0 && d->number == 0) {
   d->number = i;
   d->raid_disk = i;
   d->state = (1<<3);
   d->state |= (1<<0);
   failed++;
  }
 }
 sb->nr_disks = nr_disks;
 sb->active_disks = active;
 sb->working_disks = working;
 sb->failed_disks = failed;
 sb->spare_disks = spare;

 sb->this_disk = sb->disks[rdev->desc_nr];
 sb->sb_csum = calc_sb_csum(sb);
}
*/


extern unsigned int calc_sb_1_csum(struct mdp_superblock_1 * sb);
/* [kohei]
static unsigned int calc_sb_1_csum(struct mdp_superblock_1 * sb)
{
 unsigned int disk_csum, csum;
 unsigned long long newcsum;
 int size = 256 + (( __u32)(__le32)(sb->max_dev))*2;
 unsigned int *isuper = (unsigned int*)sb;
 int i;

 disk_csum = sb->sb_csum;
 sb->sb_csum = 0;
 newcsum = 0;
 for (i=0; size>=4; size -= 4 )
  newcsum += (( __u32)(__le32)(*isuper++));

 if (size == 2)
  newcsum += (( __u16)(__le16)(*(unsigned short*) isuper));

 csum = (newcsum & 0xffffffff) + (newcsum >> 32);
 sb->sb_csum = disk_csum;
 return (( __le32)(__u32)(csum));
}
*/

static int super_1_load(mdk_rdev_t *rdev, mdk_rdev_t *refdev, int minor_version)
{
 struct mdp_superblock_1 *sb;
 int ret;
 sector_t sb_offset;
 char b[32], b2[32];
 int bmask;

 switch(minor_version) {
 case 0:
  sb_offset = rdev->bdev->bd_inode->i_size >> 9;
  sb_offset -= 8*2;
  sb_offset &= ~(sector_t)(4*2-1);

  sb_offset /= 2;
  break;
 case 1:
  sb_offset = 0;
  break;
 case 2:
  sb_offset = 4;
  break;
 default:
  return -22;
 }
 rdev->sb_offset = sb_offset;




 ret = read_disk_sb(rdev, 4096);
 if (ret) return ret;


 sb = (struct mdp_superblock_1*)page_address(rdev->sb_page);

 if (sb->magic != (( __le32)(__u32)(0xa92b4efc)) ||
     sb->major_version != (( __le32)(__u32)(1)) ||
     (( __u32)(__le32)(sb->max_dev)) > (4096-256)/2 ||
     (( __u64)(__le64)(sb->super_offset)) != (rdev->sb_offset<<1) ||
     ((( __u32)(__le32)(sb->feature_map)) & ~(1|2|4)) != 0)
  return -22;

 if (calc_sb_1_csum(sb) != sb->sb_csum) {
  printk("md: invalid superblock checksum on %s\n",
   bdevname(rdev->bdev,b));
  return -22;
 }
 if ((( __u64)(__le64)(sb->data_size)) < 10) {
  printk("md: data_size too small on %s\n",
         bdevname(rdev->bdev,b));
  return -22;
 }
 rdev->preferred_minor = 0xffff;
 rdev->data_offset = (( __u64)(__le64)(sb->data_offset));
 (((&rdev->corrected_errors)->counter) = ((( __u32)(__le32)(sb->cnt_corrected_read))));

 rdev->sb_size = (( __u32)(__le32)(sb->max_dev)) * 2 + 256;
 bmask = queue_hardsect_size(rdev->bdev->bd_disk->queue)-1;
 if (rdev->sb_size & bmask)
  rdev-> sb_size = (rdev->sb_size | bmask)+1;

 if (sb->level == (( __le32)(__u32)((-4))))
  rdev->desc_nr = -1;
 else
  rdev->desc_nr = (( __u32)(__le32)(sb->dev_number));

 if (refdev == 0)
  ret = 1;
 else {
  __u64 ev1, ev2;
  struct mdp_superblock_1 *refsb =
   (struct mdp_superblock_1*)page_address(refdev->sb_page);

  if (__builtin_memcmp(sb->set_uuid, refsb->set_uuid, 16) != 0 ||
      sb->level != refsb->level ||
      sb->layout != refsb->layout ||
      sb->chunksize != refsb->chunksize) {
   printk("<4>" "md: %s has strangely different"
    " superblock to %s\n",
    bdevname(rdev->bdev,b),
    bdevname(refdev->bdev,b2));
   return -22;
  }
  ev1 = (( __u64)(__le64)(sb->events));
  ev2 = (( __u64)(__le64)(refsb->events));

  if (ev1 > ev2)
   ret = 1;
  else
   ret = 0;
 }
 if (minor_version)
  rdev->size = ((rdev->bdev->bd_inode->i_size>>9) - (( __u64)(__le64)(sb->data_offset))) / 2;
 else
  rdev->size = rdev->sb_offset;
 if (rdev->size < (( __u64)(__le64)(sb->data_size))/2)
  return -22;
 rdev->size = (( __u64)(__le64)(sb->data_size))/2;
 if ((( __u32)(__le32)(sb->chunksize)))
  rdev->size &= ~((sector_t)(( __u32)(__le32)(sb->chunksize))/2 - 1);

 if ((( __u32)(__le32)(sb->size)) > rdev->size*2)
  return -22;
 return ret;
}

static int super_1_validate(mddev_t *mddev, mdk_rdev_t *rdev)
{
 struct mdp_superblock_1 *sb = (struct mdp_superblock_1*)page_address(rdev->sb_page);
 __u64 ev1 = (( __u64)(__le64)(sb->events));

 rdev->raid_disk = -1;
 rdev->flags = 0;
 if (mddev->raid_disks == 0) {
  mddev->major_version = 1;
  mddev->patch_version = 0;
  mddev->persistent = 1;
  mddev->chunk_size = (( __u32)(__le32)(sb->chunksize)) << 9;
  mddev->ctime = (( __u64)(__le64)(sb->ctime)) & ((1ULL << 32)-1);
  mddev->utime = (( __u64)(__le64)(sb->utime)) & ((1ULL << 32)-1);
  mddev->level = (( __u32)(__le32)(sb->level));
  mddev->clevel[0] = 0;
  mddev->layout = (( __u32)(__le32)(sb->layout));
  mddev->raid_disks = (( __u32)(__le32)(sb->raid_disks));
  mddev->size = (( __u64)(__le64)(sb->size))/2;
  mddev->events = ev1;
  mddev->bitmap_offset = 0;
  mddev->default_bitmap_offset = 1024 >> 9;

  mddev->recovery_cp = (( __u64)(__le64)(sb->resync_offset));
  (__builtin_constant_p(16) ? __constant_memcpy((mddev->uuid),(sb->set_uuid),(16)) : __memcpy((mddev->uuid),(sb->set_uuid),(16)));

  mddev->max_disks = (4096-256)/2;

  if (((( __u32)(__le32)(sb->feature_map)) & 1) &&
      mddev->bitmap_file == ((void *)0) ) {
   if (mddev->level != 1 && mddev->level != 5 && mddev->level != 6
       && mddev->level != 10) {
    printk("<4>" "md: bitmaps not supported for this level.\n");
    return -22;
   }
   mddev->bitmap_offset = (__s32)(( __u32)(__le32)(sb->bitmap_offset));
  }
  if (((( __u32)(__le32)(sb->feature_map)) & 4)) {
   mddev->reshape_position = (( __u64)(__le64)(sb->reshape_position));
   mddev->delta_disks = (( __u32)(__le32)(sb->delta_disks));
   mddev->new_level = (( __u32)(__le32)(sb->new_level));
   mddev->new_layout = (( __u32)(__le32)(sb->new_layout));
   mddev->new_chunk = (( __u32)(__le32)(sb->new_chunk))<<9;
  } else {
   mddev->reshape_position = (~(sector_t)0);
   mddev->delta_disks = 0;
   mddev->new_level = mddev->level;
   mddev->new_layout = mddev->layout;
   mddev->new_chunk = mddev->chunk_size;
  }

 } else if (mddev->pers == ((void *)0)) {

  ++ev1;
  if (ev1 < mddev->events)
   return -22;
 } else if (mddev->bitmap) {



  if (ev1 < mddev->bitmap->events_cleared)
   return 0;
 } else {
  if (ev1 < mddev->events)

   return 0;
 }
 if (mddev->level != (-4)) {
  int role;
  role = (( __u16)(__le16)(sb->dev_roles[rdev->desc_nr]));
  switch(role) {
  case 0xffff:
   break;
  case 0xfffe:
   set_bit(1, &rdev->flags);
   break;
  default:
   if (((( __u32)(__le32)(sb->feature_map)) &
        2))
    rdev->recovery_offset = (( __u64)(__le64)(sb->recovery_offset));
   else
    set_bit(2, &rdev->flags);
   rdev->raid_disk = role;
   break;
  }
  if (sb->devflags & 1)
   set_bit(4, &rdev->flags);
 } else
  set_bit(2, &rdev->flags);

 return 0;
}


extern void super_1_sync(mddev_t *mddev, mdk_rdev_t *rdev);
/* [kohei]
static void super_1_sync(mddev_t *mddev, mdk_rdev_t *rdev)
{
 struct mdp_superblock_1 *sb;
 struct list_head *tmp;
 mdk_rdev_t *rdev2;
 int max_dev, i;


 sb = (struct mdp_superblock_1*)page_address(rdev->sb_page);

 sb->feature_map = 0;
 sb->pad0 = 0;
 sb->recovery_offset = (( __le64)(__u64)(0));
 (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(sb->pad1))) ? __constant_c_and_count_memset(((sb->pad1)),((0x01010101UL*(unsigned char)(0))),((sizeof(sb->pad1)))) : __constant_c_memset(((sb->pad1)),((0x01010101UL*(unsigned char)(0))),((sizeof(sb->pad1))))) : (__builtin_constant_p((sizeof(sb->pad1))) ? __memset_generic((((sb->pad1))),(((0))),(((sizeof(sb->pad1))))) : __memset_generic(((sb->pad1)),((0)),((sizeof(sb->pad1))))));
 (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(sb->pad2))) ? __constant_c_and_count_memset(((sb->pad2)),((0x01010101UL*(unsigned char)(0))),((sizeof(sb->pad2)))) : __constant_c_memset(((sb->pad2)),((0x01010101UL*(unsigned char)(0))),((sizeof(sb->pad2))))) : (__builtin_constant_p((sizeof(sb->pad2))) ? __memset_generic((((sb->pad2))),(((0))),(((sizeof(sb->pad2))))) : __memset_generic(((sb->pad2)),((0)),((sizeof(sb->pad2))))));
 (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(sb->pad3))) ? __constant_c_and_count_memset(((sb->pad3)),((0x01010101UL*(unsigned char)(0))),((sizeof(sb->pad3)))) : __constant_c_memset(((sb->pad3)),((0x01010101UL*(unsigned char)(0))),((sizeof(sb->pad3))))) : (__builtin_constant_p((sizeof(sb->pad3))) ? __memset_generic((((sb->pad3))),(((0))),(((sizeof(sb->pad3))))) : __memset_generic(((sb->pad3)),((0)),((sizeof(sb->pad3))))));

 sb->utime = (( __le64)(__u64)((__u64)mddev->utime));
 sb->events = (( __le64)(__u64)(mddev->events));
 if (mddev->in_sync)
  sb->resync_offset = (( __le64)(__u64)(mddev->recovery_cp));
 else
  sb->resync_offset = (( __le64)(__u64)(0));

 sb->cnt_corrected_read = ((&rdev->corrected_errors)->counter);

 sb->raid_disks = (( __le32)(__u32)(mddev->raid_disks));
 sb->size = (( __le64)(__u64)(mddev->size<<1));

 if (mddev->bitmap && mddev->bitmap_file == ((void *)0)) {
  sb->bitmap_offset = (( __le32)(__u32)((__u32)mddev->bitmap_offset));
  sb->feature_map = (( __le32)(__u32)(1));
 }

 if (rdev->raid_disk >= 0 &&
     !(__builtin_constant_p(2) ? constant_test_bit((2),(&rdev->flags)) : variable_test_bit((2),(&rdev->flags))) &&
     rdev->recovery_offset > 0) {
  sb->feature_map |= (( __le32)(__u32)(2));
  sb->recovery_offset = (( __le64)(__u64)(rdev->recovery_offset));
 }

 if (mddev->reshape_position != (~(sector_t)0)) {
  sb->feature_map |= (( __le32)(__u32)(4));
  sb->reshape_position = (( __le64)(__u64)(mddev->reshape_position));
  sb->new_layout = (( __le32)(__u32)(mddev->new_layout));
  sb->delta_disks = (( __le32)(__u32)(mddev->delta_disks));
  sb->new_level = (( __le32)(__u32)(mddev->new_level));
  sb->new_chunk = (( __le32)(__u32)(mddev->new_chunk>>9));
 }

 max_dev = 0;
 for ((tmp) = ((mddev)->disks).next; (rdev2) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp) = (tmp)->next, (tmp)->prev != &((mddev)->disks) ; )
  if (rdev2->desc_nr+1 > max_dev)
   max_dev = rdev2->desc_nr+1;

 sb->max_dev = (( __le32)(__u32)(max_dev));
 for (i=0; i<max_dev;i++)
  sb->dev_roles[i] = (( __le16)(__u16)(0xfffe));

 for ((tmp) = ((mddev)->disks).next; (rdev2) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp) = (tmp)->next, (tmp)->prev != &((mddev)->disks) ; ) {
  i = rdev2->desc_nr;
  if ((__builtin_constant_p(1) ? constant_test_bit((1),(&rdev2->flags)) : variable_test_bit((1),(&rdev2->flags))))
   sb->dev_roles[i] = (( __le16)(__u16)(0xfffe));
  else if ((__builtin_constant_p(2) ? constant_test_bit((2),(&rdev2->flags)) : variable_test_bit((2),(&rdev2->flags))))
   sb->dev_roles[i] = (( __le16)(__u16)(rdev2->raid_disk));
  else if (rdev2->raid_disk >= 0 && rdev2->recovery_offset > 0)
   sb->dev_roles[i] = (( __le16)(__u16)(rdev2->raid_disk));
  else
   sb->dev_roles[i] = (( __le16)(__u16)(0xffff));
 }

 sb->sb_csum = calc_sb_1_csum(sb);
}
*/

static struct super_type super_types[] = {
 [0] = {
  .name = "0.90.0",
  .owner = ((struct module *)0),
  .load_super = super_90_load,
  .validate_super = super_90_validate,
  .sync_super = super_90_sync,
 },
 [1] = {
  .name = "md-1",
  .owner = ((struct module *)0),
  .load_super = super_1_load,
  .validate_super = super_1_validate,
  .sync_super = super_1_sync,
 },
};

extern mdk_rdev_t * match_dev_unit(mddev_t *mddev, mdk_rdev_t *dev);
/* [kohei]
static mdk_rdev_t * match_dev_unit(mddev_t *mddev, mdk_rdev_t *dev)
{
 struct list_head *tmp;
 mdk_rdev_t *rdev;

 for ((tmp) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp) = (tmp)->next, (tmp)->prev != &((mddev)->disks) ; )
  if (rdev->bdev->bd_contains == dev->bdev->bd_contains)
   return rdev;

 return ((void *)0);
}
*/

/* [kohei]
static int match_mddev_units(mddev_t *mddev1, mddev_t *mddev2)
{
 struct list_head *tmp;
 mdk_rdev_t *rdev;

 for ((tmp) = ((mddev1)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp) = (tmp)->next, (tmp)->prev != &((mddev1)->disks) ; )
  if (match_dev_unit(mddev2, rdev))
   return 1;

 return 0;
}
*/

static struct list_head pending_raid_disks = { &(pending_raid_disks), &(pending_raid_disks) };

static int bind_rdev_to_array(mdk_rdev_t * rdev, mddev_t * mddev)
{
 mdk_rdev_t *same_pdev;
 char b[32], b2[32];
 struct kobject *ko;
 char *s;

 if (rdev->mddev) {
  { printk("md: bug in file %s, line %d\n", "drivers/md/md.c", 1329); md_print_devices(); };
  return -22;
 }

 if (rdev->size && (mddev->size == 0 || rdev->size < mddev->size)) {
  if (mddev->pers)

   return -28;
  else
   mddev->size = rdev->size;
 }
 same_pdev = match_dev_unit(mddev, rdev);
 if (same_pdev)
  printk("<4>"
   "%s: WARNING: %s appears to be on the same physical"
    " disk as %s. True\n     protection against single-disk"
   " failure might be compromised.\n",
   mdname(mddev), bdevname(rdev->bdev,b),
   bdevname(same_pdev->bdev,b2));





 if (rdev->desc_nr < 0) {
  int choice = 0;
  if (mddev->pers) choice = mddev->raid_disks;
  while (find_rdev_nr(mddev, choice))
   choice++;
  rdev->desc_nr = choice;
 } else {
  if (find_rdev_nr(mddev, rdev->desc_nr))
   return -16;
 }
 bdevname(rdev->bdev,b);
 if (kobject_set_name(&rdev->kobj, "dev-%s", b) < 0)
  return -12;
 while ( (s=strchr(rdev->kobj.k_name, '/')) != ((void *)0))
  *s = '!';

 list_add(&rdev->same_set, &mddev->disks);
 rdev->mddev = mddev;
 printk("<6>" "md: bind<%s>\n", b);

 rdev->kobj.parent = &mddev->kobj;
 kobject_add(&rdev->kobj);

 if (rdev->bdev->bd_part)
  ko = &rdev->bdev->bd_part->kobj;
 else
  ko = &rdev->bdev->bd_disk->kobj;
 sysfs_create_link(&rdev->kobj, ko, "block");
 bd_claim_by_disk(rdev->bdev, rdev, mddev->gendisk);
 return 0;
}

static void unbind_rdev_from_array(mdk_rdev_t * rdev)
{
 char b[32];
 if (!rdev->mddev) {
  { printk("md: bug in file %s, line %d\n", "drivers/md/md.c", 1389); md_print_devices(); };
  return;
 }
 bd_release_from_disk(rdev->bdev, rdev->mddev->gendisk);
 list_del_init(&rdev->same_set);
 printk("<6>" "md: unbind<%s>\n", bdevname(rdev->bdev,b));
 rdev->mddev = ((void *)0);
 sysfs_remove_link(&rdev->kobj, "block");
 kobject_del(&rdev->kobj);
}






static int lock_rdev(mdk_rdev_t *rdev, dev_t dev)
{
 int err = 0;
 struct block_device *bdev;
 char b[32];

 bdev = open_partition_by_devnum(dev, 1|2);
 if (IS_ERR(bdev)) {
  printk("<3>" "md: could not open %s.\n",
   __bdevname(dev, b));
  return PTR_ERR(bdev);
 }
 err = bd_claim(bdev, rdev);
 if (err) {
  printk("<3>" "md: could not bd_claim %s.\n",
   bdevname(bdev, b));
  blkdev_put_partition(bdev);
  return err;
 }
 rdev->bdev = bdev;
 return err;
}

static void unlock_rdev(mdk_rdev_t *rdev)
{
 struct block_device *bdev = rdev->bdev;
 rdev->bdev = ((void *)0);
 if (!bdev)
  { printk("md: bug in file %s, line %d\n", "drivers/md/md.c", 1433); md_print_devices(); };
 bd_release(bdev);
 blkdev_put_partition(bdev);
}

void md_autodetect_dev(dev_t dev);

static void export_rdev(mdk_rdev_t * rdev)
{
 char b[32];
 printk("<6>" "md: export_rdev(%s)\n",
  bdevname(rdev->bdev,b));
 if (rdev->mddev)
  { printk("md: bug in file %s, line %d\n", "drivers/md/md.c", 1446); md_print_devices(); };
 free_disk_sb(rdev);
 list_del_init(&rdev->same_set);

 md_autodetect_dev(rdev->bdev->bd_dev);

 unlock_rdev(rdev);
 kobject_put(&rdev->kobj);
}

static void kick_rdev_from_array(mdk_rdev_t * rdev)
{
 unbind_rdev_from_array(rdev);
 export_rdev(rdev);
}

extern void export_array(mddev_t *mddev);
/* [kohei]
static void export_array(mddev_t *mddev)
{
 struct list_head *tmp;
 mdk_rdev_t *rdev;

 for ((tmp) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp) = (tmp)->next, (tmp)->prev != &((mddev)->disks) ; ) {
  if (!rdev->mddev) {
   { printk("md: bug in file %s, line %d\n", "drivers/md/md.c", 1469); md_print_devices(); };
   continue;
  }
  kick_rdev_from_array(rdev);
 }
 if (!list_empty(&mddev->disks))
  { printk("md: bug in file %s, line %d\n", "drivers/md/md.c", 1475); md_print_devices(); };
 mddev->raid_disks = 0;
 mddev->major_version = 0;
}
*/

static void print_desc(mdp_disk_t *desc)
{
 printk(" DISK<N:%d,(%d,%d),R:%d,S:%d>\n", desc->number,
  desc->major,desc->minor,desc->raid_disk,desc->state);
}

static void print_sb(mdp_super_t *sb)
{
 int i;

 printk("<6>"
  "md:  SB: (V:%d.%d.%d) ID:<%08x.%08x.%08x.%08x> CT:%08x\n",
  sb->major_version, sb->minor_version, sb->patch_version,
  sb->set_uuid0, sb->set_uuid1, sb->set_uuid2, sb->set_uuid3,
  sb->ctime);
 printk("<6>" "md:     L%d S%08d ND:%d RD:%d md%d LO:%d CS:%d\n",
  sb->level, sb->size, sb->nr_disks, sb->raid_disks,
  sb->md_minor, sb->layout, sb->chunk_size);
 printk("<6>" "md:     UT:%08x ST:%d AD:%d WD:%d"
  " FD:%d SD:%d CSUM:%08x E:%08lx\n",
  sb->utime, sb->state, sb->active_disks, sb->working_disks,
  sb->failed_disks, sb->spare_disks,
  sb->sb_csum, (unsigned long)sb->events_lo);

 printk("<6>");
 for (i = 0; i < 27; i++) {
  mdp_disk_t *desc;

  desc = sb->disks + i;
  if (desc->number || desc->major || desc->minor ||
      desc->raid_disk || (desc->state && (desc->state != 4))) {
   printk("     D %2d: ", i);
   print_desc(desc);
  }
 }
 printk("<6>" "md:     THIS: ");
 print_desc(&sb->this_disk);

}

static void print_rdev(mdk_rdev_t *rdev)
{
 char b[32];
 printk("<6>" "md: rdev %s, SZ:%08llu F:%d S:%d DN:%u\n",
  bdevname(rdev->bdev,b), (unsigned long long)rdev->size,
         (__builtin_constant_p(1) ? constant_test_bit((1),(&rdev->flags)) : variable_test_bit((1),(&rdev->flags))), (__builtin_constant_p(2) ? constant_test_bit((2),(&rdev->flags)) : variable_test_bit((2),(&rdev->flags))),
         rdev->desc_nr);
 if (rdev->sb_loaded) {
  printk("<6>" "md: rdev superblock:\n");
  print_sb((mdp_super_t*)page_address(rdev->sb_page));
 } else
  printk("<6>" "md: no rdev superblock!\n");
}

extern void md_print_devices(void);
/* [kohei]
static void md_print_devices(void)
{
 struct list_head *tmp, *tmp2;
 mdk_rdev_t *rdev;
 mddev_t *mddev;
 char b[32];

 printk("\n");
 printk("md:	**********************************\n");
 printk("md:	* <COMPLETE RAID STATE PRINTOUT> *\n");
 printk("md:	**********************************\n");
 for (({ _spin_lock(&all_mddevs_lock); tmp = all_mddevs.next; mddev = ((void *)0);}); ({ if (tmp != &all_mddevs) mddev_get(({ const typeof( ((mddev_t *)0)->all_mddevs ) *__mptr = (tmp); (mddev_t *)( (char *)__mptr - __builtin_offsetof(mddev_t,all_mddevs) );})); _spin_unlock(&all_mddevs_lock); if (mddev) mddev_put(mddev); mddev = ({ const typeof( ((mddev_t *)0)->all_mddevs ) *__mptr = (tmp); (mddev_t *)( (char *)__mptr - __builtin_offsetof(mddev_t,all_mddevs) );}); tmp != &all_mddevs;}); ({ _spin_lock(&all_mddevs_lock); tmp = tmp->next;}) ) {

  if (mddev->bitmap)
   bitmap_print_sb(mddev->bitmap);
  else
   printk("%s: ", mdname(mddev));
  for ((tmp2) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp2)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp2) = (tmp2)->next, (tmp2)->prev != &((mddev)->disks) ; )
   printk("<%s>", bdevname(rdev->bdev,b));
  printk("\n");

  for ((tmp2) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp2)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp2) = (tmp2)->next, (tmp2)->prev != &((mddev)->disks) ; )
   print_rdev(rdev);
 }
 printk("md:	**********************************\n");
 printk("\n");
}
*/


extern void sync_sbs(mddev_t * mddev, int nospares);
/*
static void sync_sbs(mddev_t * mddev, int nospares)
{






 mdk_rdev_t *rdev;
 struct list_head *tmp;

 for ((tmp) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp) = (tmp)->next, (tmp)->prev != &((mddev)->disks) ; ) {
  if (rdev->sb_events == mddev->events ||
      (nospares &&
       rdev->raid_disk < 0 &&
       (rdev->sb_events&1)==0 &&
       rdev->sb_events+1 == mddev->events)) {

   rdev->sb_loaded = 2;
  } else {
   super_types[mddev->major_version].
    sync_super(mddev, rdev);
   rdev->sb_loaded = 1;
  }
 }
}
*/

extern void md_update_sb(mddev_t * mddev);
/*
void md_update_sb(mddev_t * mddev)
{
 int err;
 struct list_head *tmp;
 mdk_rdev_t *rdev;
 int sync_req;
 int nospares = 0;

repeat:
 _spin_lock_irq(&mddev->write_lock);

 if (mddev->degraded && mddev->sb_dirty == 3)

  mddev->sb_dirty = 1;

 sync_req = mddev->in_sync;
 mddev->utime = get_seconds();
 if (mddev->sb_dirty == 3)




  nospares = 1;



 if (mddev->sb_dirty == 3
     && (mddev->in_sync && mddev->recovery_cp == (~(sector_t)0))
     && (mddev->events & 1))
  mddev->events--;
 else {

  mddev->events ++;
  if (!mddev->in_sync || mddev->recovery_cp != (~(sector_t)0)) {

   if ((mddev->events&1)==0) {
    mddev->events++;
    nospares = 0;
   }
  } else {

   if ((mddev->events&1)) {
    mddev->events++;
    nospares = 0;
   }
  }
 }

 if (!mddev->events) {





  { printk("md: bug in file %s, line %d\n", "drivers/md/md.c", 1652); md_print_devices(); };
  mddev->events --;
 }
 mddev->sb_dirty = 2;
 sync_sbs(mddev, nospares);





 if (!mddev->persistent) {
  mddev->sb_dirty = 0;
  _spin_unlock_irq(&mddev->write_lock);
  __wake_up(&mddev->sb_wait, 2 | 1, 1, ((void *)0));
  return;
 }
 _spin_unlock_irq(&mddev->write_lock);

 ((void)(0 && printk("<6>" "md: updating %s RAID superblock on device (in sync %d)\n", mdname(mddev),mddev->in_sync)));



 err = bitmap_update_sb(mddev->bitmap);
 for ((tmp) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp) = (tmp)->next, (tmp)->prev != &((mddev)->disks) ; ) {
  char b[32];
  ((void)(0 && printk("<6>" "md: ")));
  if (rdev->sb_loaded != 1)
   continue;
  if ((__builtin_constant_p(1) ? constant_test_bit((1),(&rdev->flags)) : variable_test_bit((1),(&rdev->flags))))
   ((void)(0 && printk("(skipping faulty ")));

  ((void)(0 && printk("%s ", bdevname(rdev->bdev,b))));
  if (!(__builtin_constant_p(1) ? constant_test_bit((1),(&rdev->flags)) : variable_test_bit((1),(&rdev->flags)))) {
   md_super_write(mddev,rdev,
           rdev->sb_offset<<1, rdev->sb_size,
           rdev->sb_page);
   ((void)(0 && printk("<6>" "(write) %s's sb offset: %llu\n", bdevname(rdev->bdev,b), (unsigned long long)rdev->sb_offset)));


   rdev->sb_events = mddev->events;

  } else
   ((void)(0 && printk(")\n")));
  if (mddev->level == (-4))

   break;
 }
 md_super_wait(mddev);


 _spin_lock_irq(&mddev->write_lock);
 if (mddev->in_sync != sync_req|| mddev->sb_dirty == 1) {

  _spin_unlock_irq(&mddev->write_lock);
  goto repeat;
 }
 mddev->sb_dirty = 0;
 _spin_unlock_irq(&mddev->write_lock);
 __wake_up(&mddev->sb_wait, 2 | 1, 1, ((void *)0));

}
*/

extern typeof(md_update_sb) md_update_sb; extern void *__crc_md_update_sb __attribute__((weak)); static const unsigned long __kcrctab_md_update_sb __attribute__((__used__)) __attribute__((section("__kcrctab" "_gpl"), unused)) = (unsigned long) &__crc_md_update_sb; static const char __kstrtab_md_update_sb[] __attribute__((section("__ksymtab_strings"))) = "" "md_update_sb"; static const struct kernel_symbol __ksymtab_md_update_sb __attribute__((__used__)) __attribute__((section("__ksymtab" "_gpl"), unused)) = { (unsigned long)&md_update_sb, __kstrtab_md_update_sb };




static int cmd_match(const char *cmd, const char *str)
{




 while (*cmd && *str && *cmd == *str) {
  cmd++;
  str++;
 }
 if (*cmd == '\n')
  cmd++;
 if (*str || *cmd)
  return 0;
 return 1;
}

struct rdev_sysfs_entry {
 struct attribute attr;
 ssize_t (*show)(mdk_rdev_t *, char *);
 ssize_t (*store)(mdk_rdev_t *, const char *, size_t);
};

static ssize_t
state_show(mdk_rdev_t *rdev, char *page)
{
 char *sep = "";
 int len=0;

 if ((__builtin_constant_p(1) ? constant_test_bit((1),(&rdev->flags)) : variable_test_bit((1),(&rdev->flags)))) {
  len+= sprintf(page+len, "%sfaulty",sep);
  sep = ",";
 }
 if ((__builtin_constant_p(2) ? constant_test_bit((2),(&rdev->flags)) : variable_test_bit((2),(&rdev->flags)))) {
  len += sprintf(page+len, "%sin_sync",sep);
  sep = ",";
 }
 if ((__builtin_constant_p(4) ? constant_test_bit((4),(&rdev->flags)) : variable_test_bit((4),(&rdev->flags)))) {
  len += sprintf(page+len, "%swrite_mostly",sep);
  sep = ",";
 }
 if (!(__builtin_constant_p(1) ? constant_test_bit((1),(&rdev->flags)) : variable_test_bit((1),(&rdev->flags))) &&
     !(__builtin_constant_p(2) ? constant_test_bit((2),(&rdev->flags)) : variable_test_bit((2),(&rdev->flags)))) {
  len += sprintf(page+len, "%sspare", sep);
  sep = ",";
 }
 return len+sprintf(page+len, "\n");
}

static ssize_t
state_store(mdk_rdev_t *rdev, const char *buf, size_t len)
{






 int err = -22;
 if (cmd_match(buf, "faulty") && rdev->mddev->pers) {
  md_error(rdev->mddev, rdev);
  err = 0;
 } else if (cmd_match(buf, "remove")) {
  if (rdev->raid_disk >= 0)
   err = -16;
  else {
   mddev_t *mddev = rdev->mddev;
   kick_rdev_from_array(rdev);
   md_update_sb(mddev);
   md_new_event(mddev);
   err = 0;
  }
 } else if (cmd_match(buf, "writemostly")) {
  set_bit(4, &rdev->flags);
  err = 0;
 } else if (cmd_match(buf, "-writemostly")) {
  clear_bit(4, &rdev->flags);
  err = 0;
 }
 return err ? err : len;
}
static struct rdev_sysfs_entry rdev_state =
{ .attr = {.name = "state", .mode = (00400|00040|00004)|00200, .owner = ((struct module *)0) }, .show = state_show, .store = state_store, };

static ssize_t
super_show(mdk_rdev_t *rdev, char *page)
{
 if (rdev->sb_loaded && rdev->sb_size) {
  (__builtin_constant_p(rdev->sb_size) ? __constant_memcpy((page),(page_address(rdev->sb_page)),(rdev->sb_size)) : __memcpy((page),(page_address(rdev->sb_page)),(rdev->sb_size)));
  return rdev->sb_size;
 } else
  return 0;
}
static struct rdev_sysfs_entry rdev_super = { .attr = { .name = "super", .mode = 0444, .owner = ((struct module *)0) }, .show = super_show, };

static ssize_t
errors_show(mdk_rdev_t *rdev, char *page)
{
 return sprintf(page, "%d\n", ((&rdev->corrected_errors)->counter));
}

static ssize_t
errors_store(mdk_rdev_t *rdev, const char *buf, size_t len)
{
 char *e;
 unsigned long n = simple_strtoul(buf, &e, 10);
 if (*buf && (*e == 0 || *e == '\n')) {
  (((&rdev->corrected_errors)->counter) = (n));
  return len;
 }
 return -22;
}
static struct rdev_sysfs_entry rdev_errors =
{ .attr = {.name = "errors", .mode = (00400|00040|00004)|00200, .owner = ((struct module *)0) }, .show = errors_show, .store = errors_store, };

static ssize_t
slot_show(mdk_rdev_t *rdev, char *page)
{
 if (rdev->raid_disk < 0)
  return sprintf(page, "none\n");
 else
  return sprintf(page, "%d\n", rdev->raid_disk);
}

static ssize_t
slot_store(mdk_rdev_t *rdev, const char *buf, size_t len)
{
 char *e;
 int slot = simple_strtoul(buf, &e, 10);
 if (strncmp(buf, "none", 4)==0)
  slot = -1;
 else if (e==buf || (*e && *e!= '\n'))
  return -22;
 if (rdev->mddev->pers)

  return -16;
 if (slot >= rdev->mddev->raid_disks)
  return -28;
 rdev->raid_disk = slot;

 rdev->flags = 0;
 set_bit(2, &rdev->flags);
 return len;
}


static struct rdev_sysfs_entry rdev_slot =
{ .attr = {.name = "slot", .mode = (00400|00040|00004)|00200, .owner = ((struct module *)0) }, .show = slot_show, .store = slot_store, };

static ssize_t
offset_show(mdk_rdev_t *rdev, char *page)
{
 return sprintf(page, "%llu\n", (unsigned long long)rdev->data_offset);
}

static ssize_t
offset_store(mdk_rdev_t *rdev, const char *buf, size_t len)
{
 char *e;
 unsigned long long offset = simple_strtoull(buf, &e, 10);
 if (e==buf || (*e && *e != '\n'))
  return -22;
 if (rdev->mddev->pers)
  return -16;
 rdev->data_offset = offset;
 return len;
}

static struct rdev_sysfs_entry rdev_offset =
{ .attr = {.name = "offset", .mode = (00400|00040|00004)|00200, .owner = ((struct module *)0) }, .show = offset_show, .store = offset_store, };

static ssize_t
rdev_size_show(mdk_rdev_t *rdev, char *page)
{
 return sprintf(page, "%llu\n", (unsigned long long)rdev->size);
}

static ssize_t
rdev_size_store(mdk_rdev_t *rdev, const char *buf, size_t len)
{
 char *e;
 unsigned long long size = simple_strtoull(buf, &e, 10);
 if (e==buf || (*e && *e != '\n'))
  return -22;
 if (rdev->mddev->pers)
  return -16;
 rdev->size = size;
 if (size < rdev->mddev->size || rdev->mddev->size == 0)
  rdev->mddev->size = size;
 return len;
}

static struct rdev_sysfs_entry rdev_size =
{ .attr = {.name = "size", .mode = (00400|00040|00004)|00200, .owner = ((struct module *)0) }, .show = rdev_size_show, .store = rdev_size_store, };

static struct attribute *rdev_default_attrs[] = {
 &rdev_state.attr,
 &rdev_super.attr,
 &rdev_errors.attr,
 &rdev_slot.attr,
 &rdev_offset.attr,
 &rdev_size.attr,
 ((void *)0),
};

extern ssize_t rdev_attr_show(struct kobject *kobj, struct attribute *attr, char *page);
/* [kohei]
static ssize_t
rdev_attr_show(struct kobject *kobj, struct attribute *attr, char *page)
{
 struct rdev_sysfs_entry *entry = ({ const typeof( ((struct rdev_sysfs_entry *)0)->attr ) *__mptr = (attr); (struct rdev_sysfs_entry *)( (char *)__mptr - __builtin_offsetof(struct rdev_sysfs_entry,attr) );});
 mdk_rdev_t *rdev = ({ const typeof( ((mdk_rdev_t *)0)->kobj ) *__mptr = (kobj); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,kobj) );});

 if (!entry->show)
  return -5;
 return entry->show(rdev, page);
}
*/

extern ssize_t rdev_attr_store(struct kobject *kobj, struct attribute *attr, const char *page, size_t length);
/* [kohei]
static ssize_t
rdev_attr_store(struct kobject *kobj, struct attribute *attr,
       const char *page, size_t length)
{
 struct rdev_sysfs_entry *entry = ({ const typeof( ((struct rdev_sysfs_entry *)0)->attr ) *__mptr = (attr); (struct rdev_sysfs_entry *)( (char *)__mptr - __builtin_offsetof(struct rdev_sysfs_entry,attr) );});
 mdk_rdev_t *rdev = ({ const typeof( ((mdk_rdev_t *)0)->kobj ) *__mptr = (kobj); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,kobj) );});

 if (!entry->store)
  return -5;
 if (!capable(21))
  return -13;
 return entry->store(rdev, page, length);
}
*/

extern void rdev_free(struct kobject *ko);
/* [kohei]
static void rdev_free(struct kobject *ko)
{
 mdk_rdev_t *rdev = ({ const typeof( ((mdk_rdev_t *)0)->kobj ) *__mptr = (ko); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,kobj) );});
 kfree(rdev);
}
*/
static struct sysfs_ops rdev_sysfs_ops = {
 .show = rdev_attr_show,
 .store = rdev_attr_store,
};
static struct kobj_type rdev_ktype = {
 .release = rdev_free,
 .sysfs_ops = &rdev_sysfs_ops,
 .default_attrs = rdev_default_attrs,
};

static mdk_rdev_t *md_import_device(dev_t newdev, int super_format, int super_minor)
{
 char b[32];
 int err;
 mdk_rdev_t *rdev;
 sector_t size;

 rdev = kzalloc(sizeof(*rdev), ((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u)));
 if (!rdev) {
  printk("<3>" "md: could not alloc mem for new device!\n");
  return ERR_PTR(-12);
 }

 if ((err = alloc_disk_sb(rdev)))
  goto abort_free;

 err = lock_rdev(rdev, newdev);
 if (err)
  goto abort_free;

 rdev->kobj.parent = ((void *)0);
 rdev->kobj.ktype = &rdev_ktype;
 kobject_init(&rdev->kobj);

 rdev->desc_nr = -1;
 rdev->saved_raid_disk = -1;
 rdev->flags = 0;
 rdev->data_offset = 0;
 rdev->sb_events = 0;
 (((&rdev->nr_pending)->counter) = (0));
 (((&rdev->read_errors)->counter) = (0));
 (((&rdev->corrected_errors)->counter) = (0));

 size = rdev->bdev->bd_inode->i_size >> 10;
 if (!size) {
  printk("<4>"
   "md: %s has zero or unknown size, marking faulty!\n",
   bdevname(rdev->bdev,b));
  err = -22;
  goto abort_free;
 }

 if (super_format >= 0) {
  err = super_types[super_format].
   load_super(rdev, ((void *)0), super_minor);
  if (err == -22) {
   printk("<4>"
    "md: %s has invalid sb, not importing!\n",
    bdevname(rdev->bdev,b));
   goto abort_free;
  }
  if (err < 0) {
   printk("<4>"
    "md: could not read %s's sb, not importing!\n",
    bdevname(rdev->bdev,b));
   goto abort_free;
  }
 }
 INIT_LIST_HEAD(&rdev->same_set);

 return rdev;

abort_free:
 if (rdev->sb_page) {
  if (rdev->bdev)
   unlock_rdev(rdev);
  free_disk_sb(rdev);
 }
 kfree(rdev);
 return ERR_PTR(err);
}





extern void analyze_sbs(mddev_t * mddev);
/* [kohei]
static void analyze_sbs(mddev_t * mddev)
{
 int i;
 struct list_head *tmp;
 mdk_rdev_t *rdev, *freshest;
 char b[32];

 freshest = ((void *)0);
 for ((tmp) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp) = (tmp)->next, (tmp)->prev != &((mddev)->disks) ; )
  switch (super_types[mddev->major_version].
   load_super(rdev, freshest, mddev->minor_version)) {
  case 1:
   freshest = rdev;
   break;
  case 0:
   break;
  default:
   printk( "<3>" "md: fatal superblock inconsistency in %s"

    " -- removing from array\n",
    bdevname(rdev->bdev,b));
   kick_rdev_from_array(rdev);
  }


 super_types[mddev->major_version].
  validate_super(mddev, freshest);

 i = 0;
 for ((tmp) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp) = (tmp)->next, (tmp)->prev != &((mddev)->disks) ; ) {
  if (rdev != freshest)
   if (super_types[mddev->major_version].
       validate_super(mddev, rdev)) {
    printk("<4>" "md: kicking non-fresh %s"
     " from array!\n",
     bdevname(rdev->bdev,b));
    kick_rdev_from_array(rdev);
    continue;
   }
  if (mddev->level == (-4)) {
   rdev->desc_nr = i++;
   rdev->raid_disk = rdev->desc_nr;
   set_bit(2, &rdev->flags);
  }
 }



 if (mddev->recovery_cp != (~(sector_t)0) &&
     mddev->level >= 1)
  printk("<3>" "md: %s: raid array is not clean"
         " -- starting background reconstruction\n",
         mdname(mddev));

}
*/

static ssize_t
safe_delay_show(mddev_t *mddev, char *page)
{
 int msec = (mddev->safemode_delay*1000)/1000;
 return sprintf(page, "%d.%03d\n", msec/1000, msec%1000);
}
static ssize_t
safe_delay_store(mddev_t *mddev, const char *cbuf, size_t len)
{
 int scale=1;
 int dot=0;
 int i;
 unsigned long msec;
 char buf[30];
 char *e;

 if (len >= sizeof(buf))
  return -22;
 strlcpy(buf, cbuf, len);
 buf[len] = 0;
 for (i=0; i<len; i++) {
  if (dot) {
   if ((((_ctype[(int)(unsigned char)(buf[i])])&(0x04)) != 0)) {
    buf[i-1] = buf[i];
    scale *= 10;
   }
   buf[i] = 0;
  } else if (buf[i] == '.') {
   dot=1;
   buf[i] = 0;
  }
 }
 msec = simple_strtoul(buf, &e, 10);
 if (e == buf || (*e && *e != '\n'))
  return -22;
 msec = (msec * 1000) / scale;
 if (msec == 0)
  mddev->safemode_delay = 0;
 else {
  mddev->safemode_delay = (msec*1000)/1000;
  if (mddev->safemode_delay == 0)
   mddev->safemode_delay = 1;
 }
 return len;
}
static struct md_sysfs_entry md_safe_delay =
{ .attr = {.name = "safe_mode_delay", .mode = (00400|00040|00004)|00200, .owner = ((struct module *)0) }, .show = safe_delay_show, .store = safe_delay_store, };

static ssize_t
level_show(mddev_t *mddev, char *page)
{
 struct mdk_personality *p = mddev->pers;
 if (p)
  return sprintf(page, "%s\n", p->name);
 else if (mddev->clevel[0])
  return sprintf(page, "%s\n", mddev->clevel);
 else if (mddev->level != (-1000000))
  return sprintf(page, "%d\n", mddev->level);
 else
  return 0;
}

static ssize_t
level_store(mddev_t *mddev, const char *buf, size_t len)
{
 int rv = len;
 if (mddev->pers)
  return -16;
 if (len == 0)
  return 0;
 if (len >= sizeof(mddev->clevel))
  return -28;
 strncpy(mddev->clevel, buf, len);
 if (mddev->clevel[len-1] == '\n')
  len--;
 mddev->clevel[len] = 0;
 mddev->level = (-1000000);
 return rv;
}

static struct md_sysfs_entry md_level =
{ .attr = {.name = "level", .mode = (00400|00040|00004)|00200, .owner = ((struct module *)0) }, .show = level_show, .store = level_store, };


static ssize_t
layout_show(mddev_t *mddev, char *page)
{

 return sprintf(page, "%d\n", mddev->layout);
}

static ssize_t
layout_store(mddev_t *mddev, const char *buf, size_t len)
{
 char *e;
 unsigned long n = simple_strtoul(buf, &e, 10);
 if (mddev->pers)
  return -16;

 if (!*buf || (*e && *e != '\n'))
  return -22;

 mddev->layout = n;
 return len;
}
static struct md_sysfs_entry md_layout =
{ .attr = {.name = "layout", .mode = (00400|00040|00004)|00200, .owner = ((struct module *)0) }, .show = layout_show, .store = layout_store, };


static ssize_t
raid_disks_show(mddev_t *mddev, char *page)
{
 if (mddev->raid_disks == 0)
  return 0;
 return sprintf(page, "%d\n", mddev->raid_disks);
}

static int update_raid_disks(mddev_t *mddev, int raid_disks);

static ssize_t
raid_disks_store(mddev_t *mddev, const char *buf, size_t len)
{

 char *e;
 int rv = 0;
 unsigned long n = simple_strtoul(buf, &e, 10);

 if (!*buf || (*e && *e != '\n'))
  return -22;

 if (mddev->pers)
  rv = update_raid_disks(mddev, n);
 else
  mddev->raid_disks = n;
 return rv ? rv : len;
}
static struct md_sysfs_entry md_raid_disks =
{ .attr = {.name = "raid_disks", .mode = (00400|00040|00004)|00200, .owner = ((struct module *)0) }, .show = raid_disks_show, .store = raid_disks_store, };

static ssize_t
chunk_size_show(mddev_t *mddev, char *page)
{
 return sprintf(page, "%d\n", mddev->chunk_size);
}

static ssize_t
chunk_size_store(mddev_t *mddev, const char *buf, size_t len)
{

 char *e;
 unsigned long n = simple_strtoul(buf, &e, 10);

 if (mddev->pers)
  return -16;
 if (!*buf || (*e && *e != '\n'))
  return -22;

 mddev->chunk_size = n;
 return len;
}
static struct md_sysfs_entry md_chunk_size =
{ .attr = {.name = "chunk_size", .mode = (00400|00040|00004)|00200, .owner = ((struct module *)0) }, .show = chunk_size_show, .store = chunk_size_store, };

static ssize_t
resync_start_show(mddev_t *mddev, char *page)
{
 return sprintf(page, "%llu\n", (unsigned long long)mddev->recovery_cp);
}

static ssize_t
resync_start_store(mddev_t *mddev, const char *buf, size_t len)
{

 char *e;
 unsigned long long n = simple_strtoull(buf, &e, 10);

 if (mddev->pers)
  return -16;
 if (!*buf || (*e && *e != '\n'))
  return -22;

 mddev->recovery_cp = n;
 return len;
}
static struct md_sysfs_entry md_resync_start =
{ .attr = {.name = "resync_start", .mode = (00400|00040|00004)|00200, .owner = ((struct module *)0) }, .show = resync_start_show, .store = resync_start_store, };

enum array_state { clear, inactive, suspended, readonly, read_auto, clean, active,
     write_pending, active_idle, bad_word};
static char *array_states[] = {
 "clear", "inactive", "suspended", "readonly", "read-auto", "clean", "active",
 "write-pending", "active-idle", ((void *)0) };

static int match_word(const char *word, char **list)
{
 int n;
 for (n=0; list[n]; n++)
  if (cmd_match(word, list[n]))
   break;
 return n;
}

static ssize_t
array_state_show(mddev_t *mddev, char *page)
{
 enum array_state st = inactive;

 if (mddev->pers)
  switch(mddev->ro) {
  case 1:
   st = readonly;
   break;
  case 2:
   st = read_auto;
   break;
  case 0:
   if (mddev->in_sync)
    st = clean;
   else if (mddev->safemode)
    st = active_idle;
   else
    st = active;
  }
 else {
  if (list_empty(&mddev->disks) &&
      mddev->raid_disks == 0 &&
      mddev->size == 0)
   st = clear;
  else
   st = inactive;
 }
 return sprintf(page, "%s\n", array_states[st]);
}

static int do_md_stop(mddev_t * mddev, int ro);
static int do_md_run(mddev_t * mddev);
static int restart_array(mddev_t *mddev);

static ssize_t
array_state_store(mddev_t *mddev, const char *buf, size_t len)
{
 int err = -22;
 enum array_state st = match_word(buf, array_states);
 switch(st) {
 case bad_word:
  break;
 case clear:

  if (mddev->pers) {
   if (((&mddev->active)->counter) > 1)
    return -16;
   err = do_md_stop(mddev, 0);
  }
  break;
 case inactive:

  if (mddev->pers) {
   if (((&mddev->active)->counter) > 1)
    return -16;
   err = do_md_stop(mddev, 2);
  }
  break;
 case suspended:
  break;
 case readonly:
  if (mddev->pers)
   err = do_md_stop(mddev, 1);
  else {
   mddev->ro = 1;
   err = do_md_run(mddev);
  }
  break;
 case read_auto:

  if (mddev->pers) {
   err = do_md_stop(mddev, 1);
   if (err == 0)
    mddev->ro = 2;
  } else {
   mddev->ro = 2;
   err = do_md_run(mddev);
  }
  break;
 case clean:
  if (mddev->pers) {
   restart_array(mddev);
   _spin_lock_irq(&mddev->write_lock);
   if (((&mddev->writes_pending)->counter) == 0) {
    mddev->in_sync = 1;
    mddev->sb_dirty = 1;
   }
   _spin_unlock_irq(&mddev->write_lock);
  } else {
   mddev->ro = 0;
   mddev->recovery_cp = (~(sector_t)0);
   err = do_md_run(mddev);
  }
  break;
 case active:
  if (mddev->pers) {
   restart_array(mddev);
   mddev->sb_dirty = 0;
   __wake_up(&mddev->sb_wait, 2 | 1, 1, ((void *)0));
   err = 0;
  } else {
   mddev->ro = 0;
   err = do_md_run(mddev);
  }
  break;
 case write_pending:
 case active_idle:

  break;
 }
 if (err)
  return err;
 else
  return len;
}
static struct md_sysfs_entry md_array_state =
{ .attr = {.name = "array_state", .mode = (00400|00040|00004)|00200, .owner = ((struct module *)0) }, .show = array_state_show, .store = array_state_store, };

static ssize_t
null_show(mddev_t *mddev, char *page)
{
 return -22;
}

extern ssize_t new_dev_store(mddev_t *mddev, const char *buf, size_t len);
/* [kohei]
static ssize_t
new_dev_store(mddev_t *mddev, const char *buf, size_t len)
{







 char *e;
 int major = simple_strtoul(buf, &e, 10);
 int minor;
 dev_t dev;
 mdk_rdev_t *rdev;
 int err;

 if (!*buf || *e != ':' || !e[1] || e[1] == '\n')
  return -22;
 minor = simple_strtoul(e+1, &e, 10);
 if (*e && *e != '\n')
  return -22;
 dev = (((major) << 20) | (minor));
 if (major != ((unsigned int) ((dev) >> 20)) ||
     minor != ((unsigned int) ((dev) & ((1U << 20) - 1))))
  return -75;


 if (mddev->persistent) {
  rdev = md_import_device(dev, mddev->major_version,
     mddev->minor_version);
  if (!IS_ERR(rdev) && !list_empty(&mddev->disks)) {
   mdk_rdev_t *rdev0 = ({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = (mddev->disks.next); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );});

   err = super_types[mddev->major_version]
    .load_super(rdev, rdev0, mddev->minor_version);
   if (err < 0)
    goto out;
  }
 } else
  rdev = md_import_device(dev, -1, -1);

 if (IS_ERR(rdev))
  return PTR_ERR(rdev);
 err = bind_rdev_to_array(rdev, mddev);
 out:
 if (err)
  export_rdev(rdev);
 return err ? err : len;
}
*/

static struct md_sysfs_entry md_new_device =
{ .attr = {.name = "new_dev", .mode = 00200, .owner = ((struct module *)0) }, .show = null_show, .store = new_dev_store, };

static ssize_t
size_show(mddev_t *mddev, char *page)
{
 return sprintf(page, "%llu\n", (unsigned long long)mddev->size);
}

static int update_size(mddev_t *mddev, unsigned long size);

static ssize_t
size_store(mddev_t *mddev, const char *buf, size_t len)
{




 char *e;
 int err = 0;
 unsigned long long size = simple_strtoull(buf, &e, 10);
 if (!*buf || *buf == '\n' ||
     (*e && *e != '\n'))
  return -22;

 if (mddev->pers) {
  err = update_size(mddev, size);
  md_update_sb(mddev);
 } else {
  if (mddev->size == 0 ||
      mddev->size > size)
   mddev->size = size;
  else
   err = -28;
 }
 return err ? err : len;
}

static struct md_sysfs_entry md_size =
{ .attr = {.name = "component_size", .mode = (00400|00040|00004)|00200, .owner = ((struct module *)0) }, .show = size_show, .store = size_store, };






static ssize_t
metadata_show(mddev_t *mddev, char *page)
{
 if (mddev->persistent)
  return sprintf(page, "%d.%d\n",
          mddev->major_version, mddev->minor_version);
 else
  return sprintf(page, "none\n");
}

static ssize_t
metadata_store(mddev_t *mddev, const char *buf, size_t len)
{
 int major, minor;
 char *e;
 if (!list_empty(&mddev->disks))
  return -16;

 if (cmd_match(buf, "none")) {
  mddev->persistent = 0;
  mddev->major_version = 0;
  mddev->minor_version = 90;
  return len;
 }
 major = simple_strtoul(buf, &e, 10);
 if (e==buf || *e != '.')
  return -22;
 buf = e+1;
 minor = simple_strtoul(buf, &e, 10);
 if (e==buf || *e != '\n')
  return -22;
 if (major >= sizeof(super_types)/sizeof(super_types[0]) ||
     super_types[major].name == ((void *)0))
  return -2;
 mddev->major_version = major;
 mddev->minor_version = minor;
 mddev->persistent = 1;
 return len;
}

static struct md_sysfs_entry md_metadata =
{ .attr = {.name = "metadata_version", .mode = (00400|00040|00004)|00200, .owner = ((struct module *)0) }, .show = metadata_show, .store = metadata_store, };

static ssize_t
action_show(mddev_t *mddev, char *page)
{
 char *type = "idle";
 if ((__builtin_constant_p(0) ? constant_test_bit((0),(&mddev->recovery)) : variable_test_bit((0),(&mddev->recovery))) ||
     (__builtin_constant_p(5) ? constant_test_bit((5),(&mddev->recovery)) : variable_test_bit((5),(&mddev->recovery)))) {
  if ((__builtin_constant_p(8) ? constant_test_bit((8),(&mddev->recovery)) : variable_test_bit((8),(&mddev->recovery))))
   type = "reshape";
  else if ((__builtin_constant_p(1) ? constant_test_bit((1),(&mddev->recovery)) : variable_test_bit((1),(&mddev->recovery)))) {
   if (!(__builtin_constant_p(6) ? constant_test_bit((6),(&mddev->recovery)) : variable_test_bit((6),(&mddev->recovery))))
    type = "resync";
   else if ((__builtin_constant_p(7) ? constant_test_bit((7),(&mddev->recovery)) : variable_test_bit((7),(&mddev->recovery))))
    type = "check";
   else
    type = "repair";
  } else
   type = "recover";
 }
 return sprintf(page, "%s\n", type);
}

static ssize_t
action_store(mddev_t *mddev, const char *page, size_t len)
{
 if (!mddev->pers || !mddev->pers->sync_request)
  return -22;

 if (cmd_match(page, "idle")) {
  if (mddev->sync_thread) {
   set_bit(3, &mddev->recovery);
   md_unregister_thread(mddev->sync_thread);
   mddev->sync_thread = ((void *)0);
   mddev->recovery = 0;
  }
 } else if ((__builtin_constant_p(0) ? constant_test_bit((0),(&mddev->recovery)) : variable_test_bit((0),(&mddev->recovery))) ||
     (__builtin_constant_p(5) ? constant_test_bit((5),(&mddev->recovery)) : variable_test_bit((5),(&mddev->recovery))))
  return -16;
 else if (cmd_match(page, "resync") || cmd_match(page, "recover"))
  set_bit(5, &mddev->recovery);
 else if (cmd_match(page, "reshape")) {
  int err;
  if (mddev->pers->start_reshape == ((void *)0))
   return -22;
  err = mddev->pers->start_reshape(mddev);
  if (err)
   return err;
 } else {
  if (cmd_match(page, "check"))
   set_bit(7, &mddev->recovery);
  else if (!cmd_match(page, "repair"))
   return -22;
  set_bit(6, &mddev->recovery);
  set_bit(1, &mddev->recovery);
 }
 set_bit(5, &mddev->recovery);
 md_wakeup_thread(mddev->thread);
 return len;
}

static ssize_t
mismatch_cnt_show(mddev_t *mddev, char *page)
{
 return sprintf(page, "%llu\n",
         (unsigned long long) mddev->resync_mismatches);
}

static struct md_sysfs_entry md_scan_mode =
{ .attr = {.name = "sync_action", .mode = (00400|00040|00004)|00200, .owner = ((struct module *)0) }, .show = action_show, .store = action_store, };


static struct md_sysfs_entry md_mismatches = { .attr = { .name = "mismatch_cnt", .mode = 0444, .owner = ((struct module *)0) }, .show = mismatch_cnt_show, };

static ssize_t
sync_min_show(mddev_t *mddev, char *page)
{
 return sprintf(page, "%d (%s)\n", speed_min(mddev),
         mddev->sync_speed_min ? "local": "system");
}

static ssize_t
sync_min_store(mddev_t *mddev, const char *buf, size_t len)
{
 int min;
 char *e;
 if (strncmp(buf, "system", 6)==0) {
  mddev->sync_speed_min = 0;
  return len;
 }
 min = simple_strtoul(buf, &e, 10);
 if (buf == e || (*e && *e != '\n') || min <= 0)
  return -22;
 mddev->sync_speed_min = min;
 return len;
}

static struct md_sysfs_entry md_sync_min =
{ .attr = {.name = "sync_speed_min", .mode = (00400|00040|00004)|00200, .owner = ((struct module *)0) }, .show = sync_min_show, .store = sync_min_store, };

static ssize_t
sync_max_show(mddev_t *mddev, char *page)
{
 return sprintf(page, "%d (%s)\n", speed_max(mddev),
         mddev->sync_speed_max ? "local": "system");
}

static ssize_t
sync_max_store(mddev_t *mddev, const char *buf, size_t len)
{
 int max;
 char *e;
 if (strncmp(buf, "system", 6)==0) {
  mddev->sync_speed_max = 0;
  return len;
 }
 max = simple_strtoul(buf, &e, 10);
 if (buf == e || (*e && *e != '\n') || max <= 0)
  return -22;
 mddev->sync_speed_max = max;
 return len;
}

static struct md_sysfs_entry md_sync_max =
{ .attr = {.name = "sync_speed_max", .mode = (00400|00040|00004)|00200, .owner = ((struct module *)0) }, .show = sync_max_show, .store = sync_max_store, };


static ssize_t
sync_speed_show(mddev_t *mddev, char *page)
{
 unsigned long resync, dt, db;
 resync = (mddev->curr_mark_cnt - ((&mddev->recovery_active)->counter));
 dt = ((jiffies - mddev->resync_mark) / 1000);
 if (!dt) dt++;
 db = resync - (mddev->resync_mark_cnt);
 return sprintf(page, "%ld\n", db/dt/2);
}

static struct md_sysfs_entry md_sync_speed = { .attr = { .name = "sync_speed", .mode = 0444, .owner = ((struct module *)0) }, .show = sync_speed_show, };

static ssize_t
sync_completed_show(mddev_t *mddev, char *page)
{
 unsigned long max_blocks, resync;

 if ((__builtin_constant_p(1) ? constant_test_bit((1),(&mddev->recovery)) : variable_test_bit((1),(&mddev->recovery))))
  max_blocks = mddev->resync_max_sectors;
 else
  max_blocks = mddev->size << 1;

 resync = (mddev->curr_resync - ((&mddev->recovery_active)->counter));
 return sprintf(page, "%lu / %lu\n", resync, max_blocks);
}

static struct md_sysfs_entry md_sync_completed = { .attr = { .name = "sync_completed", .mode = 0444, .owner = ((struct module *)0) }, .show = sync_completed_show, };

static ssize_t
suspend_lo_show(mddev_t *mddev, char *page)
{
 return sprintf(page, "%llu\n", (unsigned long long)mddev->suspend_lo);
}

static ssize_t
suspend_lo_store(mddev_t *mddev, const char *buf, size_t len)
{
 char *e;
 unsigned long long new = simple_strtoull(buf, &e, 10);

 if (mddev->pers->quiesce == ((void *)0))
  return -22;
 if (buf == e || (*e && *e != '\n'))
  return -22;
 if (new >= mddev->suspend_hi ||
     (new > mddev->suspend_lo && new < mddev->suspend_hi)) {
  mddev->suspend_lo = new;
  mddev->pers->quiesce(mddev, 2);
  return len;
 } else
  return -22;
}
static struct md_sysfs_entry md_suspend_lo =
{ .attr = {.name = "suspend_lo", .mode = (00400|00040|00004)|00200, .owner = ((struct module *)0) }, .show = suspend_lo_show, .store = suspend_lo_store, };


static ssize_t
suspend_hi_show(mddev_t *mddev, char *page)
{
 return sprintf(page, "%llu\n", (unsigned long long)mddev->suspend_hi);
}

static ssize_t
suspend_hi_store(mddev_t *mddev, const char *buf, size_t len)
{
 char *e;
 unsigned long long new = simple_strtoull(buf, &e, 10);

 if (mddev->pers->quiesce == ((void *)0))
  return -22;
 if (buf == e || (*e && *e != '\n'))
  return -22;
 if ((new <= mddev->suspend_lo && mddev->suspend_lo >= mddev->suspend_hi) ||
     (new > mddev->suspend_lo && new > mddev->suspend_hi)) {
  mddev->suspend_hi = new;
  mddev->pers->quiesce(mddev, 1);
  mddev->pers->quiesce(mddev, 0);
  return len;
 } else
  return -22;
}
static struct md_sysfs_entry md_suspend_hi =
{ .attr = {.name = "suspend_hi", .mode = (00400|00040|00004)|00200, .owner = ((struct module *)0) }, .show = suspend_hi_show, .store = suspend_hi_store, };


static struct attribute *md_default_attrs[] = {
 &md_level.attr,
 &md_layout.attr,
 &md_raid_disks.attr,
 &md_chunk_size.attr,
 &md_size.attr,
 &md_resync_start.attr,
 &md_metadata.attr,
 &md_new_device.attr,
 &md_safe_delay.attr,
 &md_array_state.attr,
 ((void *)0),
};

static struct attribute *md_redundancy_attrs[] = {
 &md_scan_mode.attr,
 &md_mismatches.attr,
 &md_sync_min.attr,
 &md_sync_max.attr,
 &md_sync_speed.attr,
 &md_sync_completed.attr,
 &md_suspend_lo.attr,
 &md_suspend_hi.attr,
 ((void *)0),
};
static struct attribute_group md_redundancy_group = {
 .name = ((void *)0),
 .attrs = md_redundancy_attrs,
};

extern ssize_t md_attr_show(struct kobject *kobj, struct attribute *attr, char *page);
/* [kohei]
static ssize_t
md_attr_show(struct kobject *kobj, struct attribute *attr, char *page)
{
 struct md_sysfs_entry *entry = ({ const typeof( ((struct md_sysfs_entry *)0)->attr ) *__mptr = (attr); (struct md_sysfs_entry *)( (char *)__mptr - __builtin_offsetof(struct md_sysfs_entry,attr) );});
 mddev_t *mddev = ({ const typeof( ((struct mddev_s *)0)->kobj ) *__mptr = (kobj); (struct mddev_s *)( (char *)__mptr - __builtin_offsetof(struct mddev_s,kobj) );});
 ssize_t rv;

 if (!entry->show)
  return -5;
 rv = mddev_lock(mddev);
 if (!rv) {
  rv = entry->show(mddev, page);
  mddev_unlock(mddev);
 }
 return rv;
}
*/

extern ssize_t md_attr_store(struct kobject *kobj, struct attribute *attr, const char *page, size_t length);
/* [kohei]
static ssize_t
md_attr_store(struct kobject *kobj, struct attribute *attr,
       const char *page, size_t length)
{
 struct md_sysfs_entry *entry = ({ const typeof( ((struct md_sysfs_entry *)0)->attr ) *__mptr = (attr); (struct md_sysfs_entry *)( (char *)__mptr - __builtin_offsetof(struct md_sysfs_entry,attr) );});
 mddev_t *mddev = ({ const typeof( ((struct mddev_s *)0)->kobj ) *__mptr = (kobj); (struct mddev_s *)( (char *)__mptr - __builtin_offsetof(struct mddev_s,kobj) );});
 ssize_t rv;

 if (!entry->store)
  return -5;
 if (!capable(21))
  return -13;
 rv = mddev_lock(mddev);
 if (!rv) {
  rv = entry->store(mddev, page, length);
  mddev_unlock(mddev);
 }
 return rv;
}
*/

extern void md_free(struct kobject *ko);
/* [kohei]
static void md_free(struct kobject *ko)
{
 mddev_t *mddev = ({ const typeof( ((mddev_t *)0)->kobj ) *__mptr = (ko); (mddev_t *)( (char *)__mptr - __builtin_offsetof(mddev_t,kobj) );});
 kfree(mddev);
}
*/
static struct sysfs_ops md_sysfs_ops = {
 .show = md_attr_show,
 .store = md_attr_store,
};
static struct kobj_type md_ktype = {
 .release = md_free,
 .sysfs_ops = &md_sysfs_ops,
 .default_attrs = md_default_attrs,
};

int mdp_major = 0;

static struct kobject *md_probe(dev_t dev, int *part, void *data)
{
 static struct mutex disks_mutex = { .count = { (1) } , .wait_lock = (spinlock_t) { .raw_lock = { 1 }, .magic = 0xdead4ead, .owner = ((void *)-1L), .owner_cpu = -1, } , .wait_list = { &(disks_mutex.wait_list), &(disks_mutex.wait_list) } };
 mddev_t *mddev = mddev_find(dev);
 struct gendisk *disk;
 int partitioned = (((unsigned int) ((dev) >> 20)) != 9);
 int shift = partitioned ? 6 : 0;
 int unit = ((unsigned int) ((dev) & ((1U << 20) - 1))) >> shift;

 if (!mddev)
  return ((void *)0);

 mutex_lock(&disks_mutex);
 if (mddev->gendisk) {
  mutex_unlock(&disks_mutex);
  mddev_put(mddev);
  return ((void *)0);
 }
 disk = alloc_disk(1 << shift);
 if (!disk) {
  mutex_unlock(&disks_mutex);
  mddev_put(mddev);
  return ((void *)0);
 }
 disk->major = ((unsigned int) ((dev) >> 20));
 disk->first_minor = unit << shift;
 if (partitioned)
  sprintf(disk->disk_name, "md_d%d", unit);
 else
  sprintf(disk->disk_name, "md%d", unit);
 disk->fops = &md_fops;
 disk->private_data = mddev;
 disk->queue = mddev->queue;
 add_disk(disk);
 mddev->gendisk = disk;
 mutex_unlock(&disks_mutex);
 mddev->kobj.parent = &disk->kobj;
 mddev->kobj.k_name = ((void *)0);
 snprintf(mddev->kobj.name, 20, "%s", "md");
 mddev->kobj.ktype = &md_ktype;
 kobject_register(&mddev->kobj);
 return ((void *)0);
}

extern void md_safemode_timeout(unsigned long data);
/* [kohei]
static void md_safemode_timeout(unsigned long data)
{
 mddev_t *mddev = (mddev_t *) data;

 mddev->safemode = 1;
 md_wakeup_thread(mddev->thread);
}
*/

static int start_dirty_degraded;

extern int do_md_run(mddev_t * mddev);
/* [kohei]
static int do_md_run(mddev_t * mddev)
{
 int err;
 int chunk_size;
 struct list_head *tmp;
 mdk_rdev_t *rdev;
 struct gendisk *disk;
 struct mdk_personality *pers;
 char b[32];

 if (list_empty(&mddev->disks))

  return -22;

 if (mddev->pers)
  return -16;




 if (!mddev->raid_disks)
  analyze_sbs(mddev);

 chunk_size = mddev->chunk_size;

 if (chunk_size) {
  if (chunk_size > (1<<30)) {
   printk("<3>" "too big chunk_size: %d > %d\n",
    chunk_size, (1<<30));
   return -22;
  }



  if ( (1 << ffz(~chunk_size)) != chunk_size) {
   printk("<3>" "chunk_size of %d not valid\n", chunk_size);
   return -22;
  }
  if (chunk_size < (1UL << 12)) {
   printk("<3>" "too small chunk_size: %d < %ld\n",
    chunk_size, (1UL << 12));
   return -22;
  }


  for ((tmp) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp) = (tmp)->next, (tmp)->prev != &((mddev)->disks) ; ) {
   if ((__builtin_constant_p(1) ? constant_test_bit((1),(&rdev->flags)) : variable_test_bit((1),(&rdev->flags))))
    continue;
   if (rdev->size < chunk_size / 1024) {
    printk("<4>"
     "md: Dev %s smaller than chunk_size:"
     " %lluk < %dk\n",
     bdevname(rdev->bdev,b),
     (unsigned long long)rdev->size,
     chunk_size / 1024);
    return -22;
   }
  }
 }


 if (mddev->level != (-1000000))
  request_module("md-level-%d", mddev->level);
 else if (mddev->clevel[0])
  request_module("md-%s", mddev->clevel);

 for ((tmp) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp) = (tmp)->next, (tmp)->prev != &((mddev)->disks) ; ) {
  if ((__builtin_constant_p(1) ? constant_test_bit((1),(&rdev->flags)) : variable_test_bit((1),(&rdev->flags))))
   continue;
  sync_blockdev(rdev->bdev);
  invalidate_bdev(rdev->bdev, 0);
 }

 md_probe(mddev->unit, ((void *)0), ((void *)0));
 disk = mddev->gendisk;
 if (!disk)
  return -12;

 _spin_lock(&pers_lock);
 pers = find_pers(mddev->level, mddev->clevel);
 if (!pers || !try_module_get(pers->owner)) {
  _spin_unlock(&pers_lock);
  if (mddev->level != (-1000000))
   printk("<4>" "md: personality for level %d is not loaded!\n",
          mddev->level);
  else
   printk("<4>" "md: personality for level %s is not loaded!\n",
          mddev->clevel);
  return -22;
 }
 mddev->pers = pers;
 _spin_unlock(&pers_lock);
 mddev->level = pers->level;
 strlcpy(mddev->clevel, pers->name, sizeof(mddev->clevel));

 if (mddev->reshape_position != (~(sector_t)0) &&
     pers->start_reshape == ((void *)0)) {

  mddev->pers = ((void *)0);
  module_put(pers->owner);
  return -22;
 }

 mddev->recovery = 0;
 mddev->resync_max_sectors = mddev->size << 1;
 mddev->barriers_work = 1;
 mddev->ok_start_degraded = start_dirty_degraded;

 if (start_readonly)
  mddev->ro = 2;

 err = mddev->pers->run(mddev);
 if (!err && mddev->pers->sync_request) {
  err = bitmap_create(mddev);
  if (err) {
   printk("<3>" "%s: failed to create bitmap (%d)\n",
          mdname(mddev), err);
   mddev->pers->stop(mddev);
  }
 }
 if (err) {
  printk("<3>" "md: pers->run() failed ...\n");
  module_put(mddev->pers->owner);
  mddev->pers = ((void *)0);
  bitmap_destroy(mddev);
  return err;
 }
 if (mddev->pers->sync_request)
  sysfs_create_group(&mddev->kobj, &md_redundancy_group);
 else if (mddev->ro == 2)
  mddev->ro = 0;

  (((&mddev->writes_pending)->counter) = (0));
 mddev->safemode = 0;
 mddev->safemode_timer.function = md_safemode_timeout;
 mddev->safemode_timer.data = (unsigned long) mddev;
 mddev->safemode_delay = (200 * 1000)/1000 +1;
 mddev->in_sync = 1;

 for ((tmp) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp) = (tmp)->next, (tmp)->prev != &((mddev)->disks) ; )
  if (rdev->raid_disk >= 0) {
   char nm[20];
   sprintf(nm, "rd%d", rdev->raid_disk);
   sysfs_create_link(&mddev->kobj, &rdev->kobj, nm);
  }

 set_bit(5, &mddev->recovery);

 if (mddev->sb_dirty)
  md_update_sb(mddev);

 set_capacity(disk, mddev->array_size<<1);

 mddev->queue->queuedata = mddev;
 mddev->queue->make_request_fn = mddev->pers->make_request;





 if (mddev->degraded && !mddev->sync_thread) {
  struct list_head *rtmp;
  int spares = 0;
  for ((rtmp) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((rtmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (rtmp) = (rtmp)->next, (rtmp)->prev != &((mddev)->disks) ; )
   if (rdev->raid_disk >= 0 &&
       !(__builtin_constant_p(2) ? constant_test_bit((2),(&rdev->flags)) : variable_test_bit((2),(&rdev->flags))) &&
       !(__builtin_constant_p(1) ? constant_test_bit((1),(&rdev->flags)) : variable_test_bit((1),(&rdev->flags))))

    spares++;
  if (spares && mddev->pers->sync_request) {
   mddev->recovery = 0;
   set_bit(0, &mddev->recovery);
   mddev->sync_thread = md_register_thread(md_do_sync,
        mddev,
        "%s_resync");
   if (!mddev->sync_thread) {
    printk("<3>" "%s: could not start resync"
           " thread...\n",
           mdname(mddev));

    mddev->recovery = 0;
   }
  }
 }
 md_wakeup_thread(mddev->thread);
 md_wakeup_thread(mddev->sync_thread);

 mddev->changed = 1;
 md_new_event(mddev);
 return 0;
}
*/

static int restart_array(mddev_t *mddev)
{
 struct gendisk *disk = mddev->gendisk;
 int err;




 err = -6;
 if (list_empty(&mddev->disks))
  goto out;

 if (mddev->pers) {
  err = -16;
  if (!mddev->ro)
   goto out;

  mddev->safemode = 0;
  mddev->ro = 0;
  set_disk_ro(disk, 0);

  printk("<6>" "md: %s switched to read-write mode.\n",
   mdname(mddev));



  set_bit(5, &mddev->recovery);
  md_wakeup_thread(mddev->thread);
  md_wakeup_thread(mddev->sync_thread);
  err = 0;
 } else
  err = -22;

out:
 return err;
}



static int deny_bitmap_write_access(struct file * file)
{
 struct inode *inode = file->f_mapping->host;

 _spin_lock(&inode->i_lock);
 if (((&inode->i_writecount)->counter) > 1) {
  _spin_unlock(&inode->i_lock);
  return -26;
 }
 (((&inode->i_writecount)->counter) = (-1));
 _spin_unlock(&inode->i_lock);

 return 0;
}

static void restore_bitmap_write_access(struct file *file)
{
 struct inode *inode = file->f_mapping->host;

 _spin_lock(&inode->i_lock);
 (((&inode->i_writecount)->counter) = (1));
 _spin_unlock(&inode->i_lock);
}




extern int do_md_stop(mddev_t * mddev, int mode);
/* [kohei]
static int do_md_stop(mddev_t * mddev, int mode)
{
 int err = 0;
 struct gendisk *disk = mddev->gendisk;

 if (mddev->pers) {
  if (((&mddev->active)->counter)>2) {
   printk("md: %s still in use.\n",mdname(mddev));
   return -16;
  }

  if (mddev->sync_thread) {
   set_bit(9, &mddev->recovery);
   set_bit(3, &mddev->recovery);
   md_unregister_thread(mddev->sync_thread);
   mddev->sync_thread = ((void *)0);
  }

  del_timer(&mddev->safemode_timer);

  invalidate_partition(disk, 0);

  switch(mode) {
  case 1:
   err = -6;
   if (mddev->ro==1)
    goto out;
   mddev->ro = 1;
   break;
  case 0:
  case 2:
   bitmap_flush(mddev);
   md_super_wait(mddev);
   if (mddev->ro)
    set_disk_ro(disk, 0);
   blk_queue_make_request(mddev->queue, md_fail_request);
   mddev->pers->stop(mddev);
   if (mddev->pers->sync_request)
    sysfs_remove_group(&mddev->kobj, &md_redundancy_group);

   module_put(mddev->pers->owner);
   mddev->pers = ((void *)0);
   if (mddev->ro)
    mddev->ro = 0;
  }
  if (!mddev->in_sync || mddev->sb_dirty) {

   mddev->in_sync = 1;
   md_update_sb(mddev);
  }
  if (mode == 1)
   set_disk_ro(disk, 1);
  clear_bit(9, &mddev->recovery);
 }




 if (mode == 0) {
  mdk_rdev_t *rdev;
  struct list_head *tmp;
  struct gendisk *disk;
  printk("<6>" "md: %s stopped.\n", mdname(mddev));

  bitmap_destroy(mddev);
  if (mddev->bitmap_file) {
   restore_bitmap_write_access(mddev->bitmap_file);
   fput(mddev->bitmap_file);
   mddev->bitmap_file = ((void *)0);
  }
  mddev->bitmap_offset = 0;

  for ((tmp) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp) = (tmp)->next, (tmp)->prev != &((mddev)->disks) ; )
   if (rdev->raid_disk >= 0) {
    char nm[20];
    sprintf(nm, "rd%d", rdev->raid_disk);
    sysfs_remove_link(&mddev->kobj, nm);
   }

  export_array(mddev);

  mddev->array_size = 0;
  mddev->size = 0;
  mddev->raid_disks = 0;
  mddev->recovery_cp = 0;

  disk = mddev->gendisk;
  if (disk)
   set_capacity(disk, 0);
  mddev->changed = 1;
 } else if (mddev->pers)
  printk("<6>" "md: %s switched to read-only mode.\n",
   mdname(mddev));
 err = 0;
 md_new_event(mddev);
out:
 return err;
}
*/

extern void autorun_array(mddev_t *mddev);
/* [kohei]
static void autorun_array(mddev_t *mddev)
{
 mdk_rdev_t *rdev;
 struct list_head *tmp;
 int err;

 if (list_empty(&mddev->disks))
  return;

 printk("<6>" "md: running: ");

 for ((tmp) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp) = (tmp)->next, (tmp)->prev != &((mddev)->disks) ; ) {
  char b[32];
  printk("<%s>", bdevname(rdev->bdev,b));
 }
 printk("\n");

 err = do_md_run (mddev);
 if (err) {
  printk("<4>" "md: do_md_run() returned %d\n", err);
  do_md_stop (mddev, 0);
 }
}
*/

extern void autorun_devices(int part);
/* [kohei]
static void autorun_devices(int part)
{
 struct list_head *tmp;
 mdk_rdev_t *rdev0, *rdev;
 mddev_t *mddev;
 char b[32];

 printk("<6>" "md: autorun ...\n");
 while (!list_empty(&pending_raid_disks)) {
  dev_t dev;
  struct list_head candidates = { &(candidates), &(candidates) };
  rdev0 = ({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = (pending_raid_disks.next); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );});


  printk("<6>" "md: considering %s ...\n",
   bdevname(rdev0->bdev,b));
  INIT_LIST_HEAD(&candidates);
  for ((tmp) = (pending_raid_disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp) = (tmp)->next, (tmp)->prev != &(pending_raid_disks) ; )
   if (super_90_load(rdev, rdev0, 0) >= 0) {
    printk("<6>" "md:  adding %s ...\n",
     bdevname(rdev->bdev,b));
    list_move(&rdev->same_set, &candidates);
   }





  if (rdev0->preferred_minor < 0 || rdev0->preferred_minor >= 256) {
   printk("<6>" "md: unit number in %s is bad: %d\n",
          bdevname(rdev0->bdev, b), rdev0->preferred_minor);
   break;
  }
  if (part)
   dev = (((mdp_major) << 20) | (rdev0->preferred_minor << 6));

  else
   dev = (((9) << 20) | (rdev0->preferred_minor));

  md_probe(dev, ((void *)0), ((void *)0));
  mddev = mddev_find(dev);
  if (!mddev) {
   printk("<3>"
    "md: cannot allocate memory for md drive.\n");
   break;
  }
  if (mddev_lock(mddev))
   printk("<4>" "md: %s locked, cannot run\n",
          mdname(mddev));
  else if (mddev->raid_disks || mddev->major_version
    || !list_empty(&mddev->disks)) {
   printk("<4>"
    "md: %s already running, cannot run %s\n",
    mdname(mddev), bdevname(rdev0->bdev,b));
   mddev_unlock(mddev);
  } else {
   printk("<6>" "md: created %s\n", mdname(mddev));
   for ((tmp) = (candidates).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp) = (tmp)->next, (tmp)->prev != &(candidates) ; ) {
    list_del_init(&rdev->same_set);
    if (bind_rdev_to_array(rdev, mddev))
     export_rdev(rdev);
   }
   autorun_array(mddev);
   mddev_unlock(mddev);
  }



  for ((tmp) = (candidates).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp) = (tmp)->next, (tmp)->prev != &(candidates) ; )
   export_rdev(rdev);
  mddev_put(mddev);
 }
 printk("<6>" "md: ... autorun DONE.\n");
}
*/





static int autostart_array(dev_t startdev)
{
 char b[32];
 int err = -22, i;
 mdp_super_t *sb = ((void *)0);
 mdk_rdev_t *start_rdev = ((void *)0), *rdev;

 start_rdev = md_import_device(startdev, 0, 0);
 if (IS_ERR(start_rdev))
  return err;



 sb = (mdp_super_t*)page_address(start_rdev->sb_page);
 if (sb->major_version != 0 ||
     sb->minor_version != 90 ) {
  printk("<4>" "md: can only autostart 0.90.0 arrays\n");
  export_rdev(start_rdev);
  return err;
 }

 if ((__builtin_constant_p(1) ? constant_test_bit((1),(&start_rdev->flags)) : variable_test_bit((1),(&start_rdev->flags)))) {
  printk("<4>"
   "md: can not autostart based on faulty %s!\n",
   bdevname(start_rdev->bdev,b));
  export_rdev(start_rdev);
  return err;
 }
 list_add(&start_rdev->same_set, &pending_raid_disks);

 for (i = 0; i < 27; i++) {
  mdp_disk_t *desc = sb->disks + i;
  dev_t dev = (((desc->major) << 20) | (desc->minor));

  if (!dev)
   continue;
  if (dev == startdev)
   continue;
  if (((unsigned int) ((dev) >> 20)) != desc->major || ((unsigned int) ((dev) & ((1U << 20) - 1))) != desc->minor)
   continue;
  rdev = md_import_device(dev, 0, 0);
  if (IS_ERR(rdev))
   continue;

  list_add(&rdev->same_set, &pending_raid_disks);
 }




 autorun_devices(0);
 return 0;

}


static int get_version(void * arg)
{
 mdu_version_t ver;

 ver.major = 0;
 ver.minor = 90;
 ver.patchlevel = 3;

 if (copy_to_user(arg, &ver, sizeof(ver)))
  return -14;

 return 0;
}

extern int get_array_info(mddev_t * mddev, void * arg);
/* [kohei]
static int get_array_info(mddev_t * mddev, void * arg)
{
 mdu_array_info_t info;
 int nr,working,active,failed,spare;
 mdk_rdev_t *rdev;
 struct list_head *tmp;

 nr=working=active=failed=spare=0;
 for ((tmp) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp) = (tmp)->next, (tmp)->prev != &((mddev)->disks) ; ) {
  nr++;
  if ((__builtin_constant_p(1) ? constant_test_bit((1),(&rdev->flags)) : variable_test_bit((1),(&rdev->flags))))
   failed++;
  else {
   working++;
   if ((__builtin_constant_p(2) ? constant_test_bit((2),(&rdev->flags)) : variable_test_bit((2),(&rdev->flags))))
    active++;
   else
    spare++;
  }
 }

 info.major_version = mddev->major_version;
 info.minor_version = mddev->minor_version;
 info.patch_version = 3;
 info.ctime = mddev->ctime;
 info.level = mddev->level;
 info.size = mddev->size;
 if (info.size != mddev->size)
  info.size = -1;
 info.nr_disks = nr;
 info.raid_disks = mddev->raid_disks;
 info.md_minor = mddev->md_minor;
 info.not_persistent= !mddev->persistent;

 info.utime = mddev->utime;
 info.state = 0;
 if (mddev->in_sync)
  info.state = (1<<0);
 if (mddev->bitmap && mddev->bitmap_offset)
  info.state = (1<<8);
 info.active_disks = active;
 info.working_disks = working;
 info.failed_disks = failed;
 info.spare_disks = spare;

 info.layout = mddev->layout;
 info.chunk_size = mddev->chunk_size;

 if (copy_to_user(arg, &info, sizeof(info)))
  return -14;

 return 0;
}
*/

static int get_bitmap_file(mddev_t * mddev, void * arg)
{
 mdu_bitmap_file_t *file = ((void *)0);
 char *ptr, *buf = ((void *)0);
 int err = -12;

 file = kmalloc(sizeof(*file), ((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u)));
 if (!file)
  goto out;


 if (!mddev->bitmap || !mddev->bitmap->file) {
  file->pathname[0] = '\0';
  goto copy_out;
 }

 buf = kmalloc(sizeof(file->pathname), ((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u)));
 if (!buf)
  goto out;

 ptr = file_path(mddev->bitmap->file, buf, sizeof(file->pathname));
 if (!ptr)
  goto out;

 strcpy(file->pathname, ptr);

copy_out:
 err = 0;
 if (copy_to_user(arg, file, sizeof(*file)))
  err = -14;
out:
 kfree(buf);
 kfree(file);
 return err;
}

static int get_disk_info(mddev_t * mddev, void * arg)
{
 mdu_disk_info_t info;
 unsigned int nr;
 mdk_rdev_t *rdev;

 if (copy_from_user(&info, arg, sizeof(info)))
  return -14;

 nr = info.number;

 rdev = find_rdev_nr(mddev, nr);
 if (rdev) {
  info.major = ((unsigned int) ((rdev->bdev->bd_dev) >> 20));
  info.minor = ((unsigned int) ((rdev->bdev->bd_dev) & ((1U << 20) - 1)));
  info.raid_disk = rdev->raid_disk;
  info.state = 0;
  if ((__builtin_constant_p(1) ? constant_test_bit((1),(&rdev->flags)) : variable_test_bit((1),(&rdev->flags))))
   info.state |= (1<<0);
  else if ((__builtin_constant_p(2) ? constant_test_bit((2),(&rdev->flags)) : variable_test_bit((2),(&rdev->flags)))) {
   info.state |= (1<<1);
   info.state |= (1<<2);
  }
  if ((__builtin_constant_p(4) ? constant_test_bit((4),(&rdev->flags)) : variable_test_bit((4),(&rdev->flags))))
   info.state |= (1<<9);
 } else {
  info.major = info.minor = 0;
  info.raid_disk = -1;
  info.state = (1<<3);
 }

 if (copy_to_user(arg, &info, sizeof(info)))
  return -14;

 return 0;
}

extern int add_new_disk(mddev_t * mddev, mdu_disk_info_t *info);
/* [kohei]
static int add_new_disk(mddev_t * mddev, mdu_disk_info_t *info)
{
 char b[32], b2[32];
 mdk_rdev_t *rdev;
 dev_t dev = (((info->major) << 20) | (info->minor));

 if (info->major != ((unsigned int) ((dev) >> 20)) || info->minor != ((unsigned int) ((dev) & ((1U << 20) - 1))))
  return -75;

 if (!mddev->raid_disks) {
  int err;

  rdev = md_import_device(dev, mddev->major_version, mddev->minor_version);
  if (IS_ERR(rdev)) {
   printk("<4>"
    "md: md_import_device returned %ld\n",
    PTR_ERR(rdev));
   return PTR_ERR(rdev);
  }
  if (!list_empty(&mddev->disks)) {
   mdk_rdev_t *rdev0 = ({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = (mddev->disks.next); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );});

   int err = super_types[mddev->major_version]
    .load_super(rdev, rdev0, mddev->minor_version);
   if (err < 0) {
    printk("<4>"
     "md: %s has different UUID to %s\n",
     bdevname(rdev->bdev,b),
     bdevname(rdev0->bdev,b2));
    export_rdev(rdev);
    return -22;
   }
  }
  err = bind_rdev_to_array(rdev, mddev);
  if (err)
   export_rdev(rdev);
  return err;
 }






 if (mddev->pers) {
  int err;
  if (!mddev->pers->hot_add_disk) {
   printk("<4>"
    "%s: personality does not support diskops!\n",
          mdname(mddev));
   return -22;
  }
  if (mddev->persistent)
   rdev = md_import_device(dev, mddev->major_version,
      mddev->minor_version);
  else
   rdev = md_import_device(dev, -1, -1);
  if (IS_ERR(rdev)) {
   printk("<4>"
    "md: md_import_device returned %ld\n",
    PTR_ERR(rdev));
   return PTR_ERR(rdev);
  }

  if (!mddev->persistent) {
   if (info->state & (1<<2) &&
       info->raid_disk < mddev->raid_disks)
    rdev->raid_disk = info->raid_disk;
   else
    rdev->raid_disk = -1;
  } else
   super_types[mddev->major_version].
    validate_super(mddev, rdev);
  rdev->saved_raid_disk = rdev->raid_disk;

  clear_bit(2, &rdev->flags);
  if (info->state & (1<<9))
   set_bit(4, &rdev->flags);

  rdev->raid_disk = -1;
  err = bind_rdev_to_array(rdev, mddev);
  if (!err && !mddev->pers->hot_remove_disk) {




   super_types[mddev->major_version].
    validate_super(mddev, rdev);
   err = mddev->pers->hot_add_disk(mddev, rdev);
   if (err)
    unbind_rdev_from_array(rdev);
  }
  if (err)
   export_rdev(rdev);

  set_bit(5, &mddev->recovery);
  md_wakeup_thread(mddev->thread);
  return err;
 }




 if (mddev->major_version != 0) {
  printk("<4>" "%s: ADD_NEW_DISK not supported\n",
         mdname(mddev));
  return -22;
 }

 if (!(info->state & (1<<0))) {
  int err;
  rdev = md_import_device (dev, -1, 0);
  if (IS_ERR(rdev)) {
   printk("<4>"
    "md: error, md_import_device() returned %ld\n",
    PTR_ERR(rdev));
   return PTR_ERR(rdev);
  }
  rdev->desc_nr = info->number;
  if (info->raid_disk < mddev->raid_disks)
   rdev->raid_disk = info->raid_disk;
  else
   rdev->raid_disk = -1;

  rdev->flags = 0;

  if (rdev->raid_disk < mddev->raid_disks)
   if (info->state & (1<<2))
    set_bit(2, &rdev->flags);

  if (info->state & (1<<9))
   set_bit(4, &rdev->flags);

  if (!mddev->persistent) {
   printk("<6>" "md: nonpersistent superblock ...\n");
   rdev->sb_offset = rdev->bdev->bd_inode->i_size >> 10;
  } else
   rdev->sb_offset = calc_dev_sboffset(rdev->bdev);
  rdev->size = calc_dev_size(rdev, mddev->chunk_size);

  err = bind_rdev_to_array(rdev, mddev);
  if (err) {
   export_rdev(rdev);
   return err;
  }
 }

 return 0;
}
*/

static int hot_remove_disk(mddev_t * mddev, dev_t dev)
{
 char b[32];
 mdk_rdev_t *rdev;

 if (!mddev->pers)
  return -19;

 rdev = find_rdev(mddev, dev);
 if (!rdev)
  return -6;

 if (rdev->raid_disk >= 0)
  goto busy;

 kick_rdev_from_array(rdev);
 md_update_sb(mddev);
 md_new_event(mddev);

 return 0;
busy:
 printk("<4>" "md: cannot remove active disk %s from %s ... \n",
  bdevname(rdev->bdev,b), mdname(mddev));
 return -16;
}

static int hot_add_disk(mddev_t * mddev, dev_t dev)
{
 char b[32];
 int err;
 unsigned int size;
 mdk_rdev_t *rdev;

 if (!mddev->pers)
  return -19;

 if (mddev->major_version != 0) {
  printk("<4>" "%s: HOT_ADD may only be used with"
   " version-0 superblocks.\n",
   mdname(mddev));
  return -22;
 }
 if (!mddev->pers->hot_add_disk) {
  printk("<4>"
   "%s: personality does not support diskops!\n",
   mdname(mddev));
  return -22;
 }

 rdev = md_import_device (dev, -1, 0);
 if (IS_ERR(rdev)) {
  printk("<4>"
   "md: error, md_import_device() returned %ld\n",
   PTR_ERR(rdev));
  return -22;
 }

 if (mddev->persistent)
  rdev->sb_offset = calc_dev_sboffset(rdev->bdev);
 else
  rdev->sb_offset =
   rdev->bdev->bd_inode->i_size >> 10;

 size = calc_dev_size(rdev, mddev->chunk_size);
 rdev->size = size;

 if ((__builtin_constant_p(1) ? constant_test_bit((1),(&rdev->flags)) : variable_test_bit((1),(&rdev->flags)))) {
  printk("<4>"
   "md: can not hot-add faulty %s disk to %s!\n",
   bdevname(rdev->bdev,b), mdname(mddev));
  err = -22;
  goto abort_export;
 }
 clear_bit(2, &rdev->flags);
 rdev->desc_nr = -1;
 rdev->saved_raid_disk = -1;
 err = bind_rdev_to_array(rdev, mddev);
 if (err)
  goto abort_export;






 if (rdev->desc_nr == mddev->max_disks) {
  printk("<4>" "%s: can not hot-add to full array!\n",
   mdname(mddev));
  err = -16;
  goto abort_unbind_export;
 }

 rdev->raid_disk = -1;

 md_update_sb(mddev);





 set_bit(5, &mddev->recovery);
 md_wakeup_thread(mddev->thread);
 md_new_event(mddev);
 return 0;

abort_unbind_export:
 unbind_rdev_from_array(rdev);

abort_export:
 export_rdev(rdev);
 return err;
}

static int set_bitmap_file(mddev_t *mddev, int fd)
{
 int err;

 if (mddev->pers) {
  if (!mddev->pers->quiesce)
   return -16;
  if (mddev->recovery || mddev->sync_thread)
   return -16;

 }


 if (fd >= 0) {
  if (mddev->bitmap)
   return -17;
  mddev->bitmap_file = fget(fd);

  if (mddev->bitmap_file == ((void *)0)) {
   printk("<3>" "%s: error: failed to get bitmap file\n",
          mdname(mddev));
   return -9;
  }

  err = deny_bitmap_write_access(mddev->bitmap_file);
  if (err) {
   printk("<3>" "%s: error: bitmap file is already in use\n",
          mdname(mddev));
   fput(mddev->bitmap_file);
   mddev->bitmap_file = ((void *)0);
   return err;
  }
  mddev->bitmap_offset = 0;
 } else if (mddev->bitmap == ((void *)0))
  return -2;
 err = 0;
 if (mddev->pers) {
  mddev->pers->quiesce(mddev, 1);
  if (fd >= 0)
   err = bitmap_create(mddev);
  if (fd < 0 || err) {
   bitmap_destroy(mddev);
   fd = -1;
  }
  mddev->pers->quiesce(mddev, 0);
 }
 if (fd < 0) {
  if (mddev->bitmap_file) {
   restore_bitmap_write_access(mddev->bitmap_file);
   fput(mddev->bitmap_file);
  }
  mddev->bitmap_file = ((void *)0);
 }

 return err;
}

static int set_array_info(mddev_t * mddev, mdu_array_info_t *info)
{

 if (info->raid_disks == 0) {

  if (info->major_version < 0 ||
      info->major_version >= sizeof(super_types)/sizeof(super_types[0]) ||
      super_types[info->major_version].name == ((void *)0)) {

   printk("<6>"
    "md: superblock version %d not known\n",
    info->major_version);
   return -22;
  }
  mddev->major_version = info->major_version;
  mddev->minor_version = info->minor_version;
  mddev->patch_version = info->patch_version;
  return 0;
 }
 mddev->major_version = 0;
 mddev->minor_version = 90;
 mddev->patch_version = 3;
 mddev->ctime = get_seconds();

 mddev->level = info->level;
 mddev->clevel[0] = 0;
 mddev->size = info->size;
 mddev->raid_disks = info->raid_disks;



 if (info->state & (1<<0))
  mddev->recovery_cp = (~(sector_t)0);
 else
  mddev->recovery_cp = 0;
 mddev->persistent = ! info->not_persistent;

 mddev->layout = info->layout;
 mddev->chunk_size = info->chunk_size;

 mddev->max_disks = 27;

 mddev->sb_dirty = 1;

 mddev->default_bitmap_offset = 4096 >> 9;
 mddev->bitmap_offset = 0;

 mddev->reshape_position = (~(sector_t)0);




 get_random_bytes(mddev->uuid, 16);

 mddev->new_level = mddev->level;
 mddev->new_chunk = mddev->chunk_size;
 mddev->new_layout = mddev->layout;
 mddev->delta_disks = 0;

 return 0;
}

extern int update_size(mddev_t *mddev, unsigned long size);
/* [kohei]
static int update_size(mddev_t *mddev, unsigned long size)
{
 mdk_rdev_t * rdev;
 int rv;
 struct list_head *tmp;
 int fit = (size == 0);

 if (mddev->pers->resize == ((void *)0))
  return -22;

 if (mddev->sync_thread)
  return -16;
 for ((tmp) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp) = (tmp)->next, (tmp)->prev != &((mddev)->disks) ; ) {
  sector_t avail;
  if (rdev->sb_offset > rdev->data_offset)
   avail = (rdev->sb_offset*2) - rdev->data_offset;
  else
   avail = get_capacity(rdev->bdev->bd_disk)
    - rdev->data_offset;
  if (fit && (size == 0 || size > avail/2))
   size = avail/2;
  if (avail < ((sector_t)size << 1))
   return -28;
 }
 rv = mddev->pers->resize(mddev, (sector_t)size *2);
 if (!rv) {
  struct block_device *bdev;

  bdev = bdget_disk(mddev->gendisk, 0);
  if (bdev) {
   mutex_lock(&bdev->bd_inode->i_mutex);
   i_size_write(bdev->bd_inode, (loff_t)mddev->array_size << 10);
   mutex_unlock(&bdev->bd_inode->i_mutex);
   bdput(bdev);
  }
 }
 return rv;
}
*/

static int update_raid_disks(mddev_t *mddev, int raid_disks)
{
 int rv;

 if (mddev->pers->check_reshape == ((void *)0))
  return -22;
 if (raid_disks <= 0 ||
     raid_disks >= mddev->max_disks)
  return -22;
 if (mddev->sync_thread || mddev->reshape_position != (~(sector_t)0))
  return -16;
 mddev->delta_disks = raid_disks - mddev->raid_disks;

 rv = mddev->pers->check_reshape(mddev);
 return rv;
}

static int update_array_info(mddev_t *mddev, mdu_array_info_t *info)
{
 int rv = 0;
 int cnt = 0;
 int state = 0;


 if (mddev->bitmap && mddev->bitmap_offset)
  state |= (1 << 8);

 if (mddev->major_version != info->major_version ||
     mddev->minor_version != info->minor_version ||

     mddev->ctime != info->ctime ||
     mddev->level != info->level ||

     !mddev->persistent != info->not_persistent||
     mddev->chunk_size != info->chunk_size ||

     ((state^info->state) & 0xfffffe00)
  )
  return -22;

 if (info->size >= 0 && mddev->size != info->size) cnt++;
 if (mddev->raid_disks != info->raid_disks) cnt++;
 if (mddev->layout != info->layout) cnt++;
 if ((state ^ info->state) & (1<<8)) cnt++;
 if (cnt == 0) return 0;
 if (cnt > 1) return -22;

 if (mddev->layout != info->layout) {




  if (mddev->pers->reconfig == ((void *)0))
   return -22;
  else
   return mddev->pers->reconfig(mddev, info->layout, -1);
 }
 if (info->size >= 0 && mddev->size != info->size)
  rv = update_size(mddev, info->size);

 if (mddev->raid_disks != info->raid_disks)
  rv = update_raid_disks(mddev, info->raid_disks);

 if ((state ^ info->state) & (1<<8)) {
  if (mddev->pers->quiesce == ((void *)0))
   return -22;
  if (mddev->recovery || mddev->sync_thread)
   return -16;
  if (info->state & (1<<8)) {

   if (mddev->bitmap)
    return -17;
   if (mddev->default_bitmap_offset == 0)
    return -22;
   mddev->bitmap_offset = mddev->default_bitmap_offset;
   mddev->pers->quiesce(mddev, 1);
   rv = bitmap_create(mddev);
   if (rv)
    bitmap_destroy(mddev);
   mddev->pers->quiesce(mddev, 0);
  } else {

   if (!mddev->bitmap)
    return -2;
   if (mddev->bitmap->file)
    return -22;
   mddev->pers->quiesce(mddev, 1);
   bitmap_destroy(mddev);
   mddev->pers->quiesce(mddev, 0);
   mddev->bitmap_offset = 0;
  }
 }
 md_update_sb(mddev);
 return rv;
}

static int set_disk_faulty(mddev_t *mddev, dev_t dev)
{
 mdk_rdev_t *rdev;

 if (mddev->pers == ((void *)0))
  return -19;

 rdev = find_rdev(mddev, dev);
 if (!rdev)
  return -19;

 md_error(mddev, rdev);
 return 0;
}

static int md_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
 mddev_t *mddev = bdev->bd_disk->private_data;

 geo->heads = 2;
 geo->sectors = 4;
 geo->cylinders = get_capacity(mddev->gendisk) / 8;
 return 0;
}

static int md_ioctl(struct inode *inode, struct file *file,
   unsigned int cmd, unsigned long arg)
{
 int err = 0;
 void *argp = (void *)arg;
 mddev_t *mddev = ((void *)0);

 if (!capable(21))
  return -13;





 switch (cmd)
 {
  case (((2U) << (((0 +8)+8)+14)) | (((9)) << (0 +8)) | (((0x10)) << 0) | (((((sizeof(mdu_version_t) == sizeof(mdu_version_t[1]) && sizeof(mdu_version_t) < (1 << 14)) ? sizeof(mdu_version_t) : __invalid_size_argument_for_IOC))) << ((0 +8)+8))):
   err = get_version(argp);
   goto done;

  case (((0U) << (((0 +8)+8)+14)) | (((9)) << (0 +8)) | (((0x13)) << 0) | ((0) << ((0 +8)+8))):
   err = 0;
   md_print_devices();
   goto done;


  case (((0U) << (((0 +8)+8)+14)) | (((9)) << (0 +8)) | (((0x14)) << 0) | ((0) << ((0 +8)+8))):
   err = 0;
   autostart_arrays(arg);
   goto done;

  default:;
 }





 mddev = inode->i_bdev->bd_disk->private_data;

 if (!mddev) {
  __asm__ __volatile__( "ud2\n" "\t.word %c0\n" "\t.long %c1\n" : : "i" (4260), "i" ("drivers/md/md.c"));
  goto abort;
 }


 if (cmd == (((0U) << (((0 +8)+8)+14)) | (((9)) << (0 +8)) | (((0x31)) << 0) | ((0) << ((0 +8)+8)))) {



  static int cnt = 3;
  if (cnt > 0 ) {
   printk("<4>"
          "md: %s(pid %d) used deprecated START_ARRAY ioctl. "
          "This will not be supported beyond July 2006\n",
          __vericon_dummy_current->comm, __vericon_dummy_current->pid);
   cnt--;
  }
  err = autostart_array(new_decode_dev(arg));
  if (err) {
   printk("<4>" "md: autostart failed!\n");
   goto abort;
  }
  goto done;
 }

 err = mddev_lock(mddev);
 if (err) {
  printk("<6>"
   "md: ioctl lock interrupted, reason %d, cmd %d\n",
   err, cmd);
  goto abort;
 }

 switch (cmd)
 {
  case (((1U) << (((0 +8)+8)+14)) | (((9)) << (0 +8)) | (((0x23)) << 0) | (((((sizeof(mdu_array_info_t) == sizeof(mdu_array_info_t[1]) && sizeof(mdu_array_info_t) < (1 << 14)) ? sizeof(mdu_array_info_t) : __invalid_size_argument_for_IOC))) << ((0 +8)+8))):
   {
    mdu_array_info_t info;
    if (!arg)
     (__builtin_constant_p(0) ? (__builtin_constant_p((sizeof(info))) ? __constant_c_and_count_memset(((&info)),((0x01010101UL*(unsigned char)(0))),((sizeof(info)))) : __constant_c_memset(((&info)),((0x01010101UL*(unsigned char)(0))),((sizeof(info))))) : (__builtin_constant_p((sizeof(info))) ? __memset_generic((((&info))),(((0))),(((sizeof(info))))) : __memset_generic(((&info)),((0)),((sizeof(info))))));
    else if (copy_from_user(&info, argp, sizeof(info))) {
     err = -14;
     goto abort_unlock;
    }
    if (mddev->pers) {
     err = update_array_info(mddev, &info);
     if (err) {
      printk("<4>" "md: couldn't update"
             " array info. %d\n", err);
      goto abort_unlock;
     }
     goto done_unlock;
    }
    if (!list_empty(&mddev->disks)) {
     printk("<4>"
            "md: array %s already has disks!\n",
            mdname(mddev));
     err = -16;
     goto abort_unlock;
    }
    if (mddev->raid_disks) {
     printk("<4>"
            "md: array %s already initialised!\n",
            mdname(mddev));
     err = -16;
     goto abort_unlock;
    }
    err = set_array_info(mddev, &info);
    if (err) {
     printk("<4>" "md: couldn't set"
            " array info. %d\n", err);
     goto abort_unlock;
    }
   }
   goto done_unlock;

  default:;
 }






 if (!mddev->raid_disks && cmd != (((1U) << (((0 +8)+8)+14)) | (((9)) << (0 +8)) | (((0x21)) << 0) | (((((sizeof(mdu_disk_info_t) == sizeof(mdu_disk_info_t[1]) && sizeof(mdu_disk_info_t) < (1 << 14)) ? sizeof(mdu_disk_info_t) : __invalid_size_argument_for_IOC))) << ((0 +8)+8))) && cmd != (((0U) << (((0 +8)+8)+14)) | (((9)) << (0 +8)) | (((0x32)) << 0) | ((0) << ((0 +8)+8)))
   && cmd != (((1U) << (((0 +8)+8)+14)) | (((9)) << (0 +8)) | (((0x30)) << 0) | (((((sizeof(mdu_param_t) == sizeof(mdu_param_t[1]) && sizeof(mdu_param_t) < (1 << 14)) ? sizeof(mdu_param_t) : __invalid_size_argument_for_IOC))) << ((0 +8)+8))) && cmd != (((1U) << (((0 +8)+8)+14)) | (((9)) << (0 +8)) | (((0x2b)) << 0) | (((((sizeof(int) == sizeof(int[1]) && sizeof(int) < (1 << 14)) ? sizeof(int) : __invalid_size_argument_for_IOC))) << ((0 +8)+8)))) {
  err = -19;
  goto abort_unlock;
 }




 switch (cmd)
 {
  case (((2U) << (((0 +8)+8)+14)) | (((9)) << (0 +8)) | (((0x11)) << 0) | (((((sizeof(mdu_array_info_t) == sizeof(mdu_array_info_t[1]) && sizeof(mdu_array_info_t) < (1 << 14)) ? sizeof(mdu_array_info_t) : __invalid_size_argument_for_IOC))) << ((0 +8)+8))):
   err = get_array_info(mddev, argp);
   goto done_unlock;

  case (((2U) << (((0 +8)+8)+14)) | (((9)) << (0 +8)) | (((0x15)) << 0) | (((((sizeof(mdu_bitmap_file_t) == sizeof(mdu_bitmap_file_t[1]) && sizeof(mdu_bitmap_file_t) < (1 << 14)) ? sizeof(mdu_bitmap_file_t) : __invalid_size_argument_for_IOC))) << ((0 +8)+8))):
   err = get_bitmap_file(mddev, argp);
   goto done_unlock;

  case (((2U) << (((0 +8)+8)+14)) | (((9)) << (0 +8)) | (((0x12)) << 0) | (((((sizeof(mdu_disk_info_t) == sizeof(mdu_disk_info_t[1]) && sizeof(mdu_disk_info_t) < (1 << 14)) ? sizeof(mdu_disk_info_t) : __invalid_size_argument_for_IOC))) << ((0 +8)+8))):
   err = get_disk_info(mddev, argp);
   goto done_unlock;

  case (((0U) << (((0 +8)+8)+14)) | (((9)) << (0 +8)) | (((0x34)) << 0) | ((0) << ((0 +8)+8))):
   err = restart_array(mddev);
   goto done_unlock;

  case (((0U) << (((0 +8)+8)+14)) | (((9)) << (0 +8)) | (((0x32)) << 0) | ((0) << ((0 +8)+8))):
   err = do_md_stop (mddev, 0);
   goto done_unlock;

  case (((0U) << (((0 +8)+8)+14)) | (((9)) << (0 +8)) | (((0x33)) << 0) | ((0) << ((0 +8)+8))):
   err = do_md_stop (mddev, 1);
   goto done_unlock;







 }

 if ((((cmd) >> (0 +8)) & ((1 << 8)-1)) == 9 &&
     mddev->ro && mddev->pers) {
  if (mddev->ro == 2) {
   mddev->ro = 0;
  set_bit(5, &mddev->recovery);
  md_wakeup_thread(mddev->thread);

  } else {
   err = -30;
   goto abort_unlock;
  }
 }

 switch (cmd)
 {
  case (((1U) << (((0 +8)+8)+14)) | (((9)) << (0 +8)) | (((0x21)) << 0) | (((((sizeof(mdu_disk_info_t) == sizeof(mdu_disk_info_t[1]) && sizeof(mdu_disk_info_t) < (1 << 14)) ? sizeof(mdu_disk_info_t) : __invalid_size_argument_for_IOC))) << ((0 +8)+8))):
  {
   mdu_disk_info_t info;
   if (copy_from_user(&info, argp, sizeof(info)))
    err = -14;
   else
    err = add_new_disk(mddev, &info);
   goto done_unlock;
  }

  case (((0U) << (((0 +8)+8)+14)) | (((9)) << (0 +8)) | (((0x22)) << 0) | ((0) << ((0 +8)+8))):
   err = hot_remove_disk(mddev, new_decode_dev(arg));
   goto done_unlock;

  case (((0U) << (((0 +8)+8)+14)) | (((9)) << (0 +8)) | (((0x28)) << 0) | ((0) << ((0 +8)+8))):
   err = hot_add_disk(mddev, new_decode_dev(arg));
   goto done_unlock;

  case (((0U) << (((0 +8)+8)+14)) | (((9)) << (0 +8)) | (((0x29)) << 0) | ((0) << ((0 +8)+8))):
   err = set_disk_faulty(mddev, new_decode_dev(arg));
   goto done_unlock;

  case (((1U) << (((0 +8)+8)+14)) | (((9)) << (0 +8)) | (((0x30)) << 0) | (((((sizeof(mdu_param_t) == sizeof(mdu_param_t[1]) && sizeof(mdu_param_t) < (1 << 14)) ? sizeof(mdu_param_t) : __invalid_size_argument_for_IOC))) << ((0 +8)+8))):
   err = do_md_run (mddev);
   goto done_unlock;

  case (((1U) << (((0 +8)+8)+14)) | (((9)) << (0 +8)) | (((0x2b)) << 0) | (((((sizeof(int) == sizeof(int[1]) && sizeof(int) < (1 << 14)) ? sizeof(int) : __invalid_size_argument_for_IOC))) << ((0 +8)+8))):
   err = set_bitmap_file(mddev, (int)arg);
   goto done_unlock;

  default:
   err = -22;
   goto abort_unlock;
 }

done_unlock:
abort_unlock:
 mddev_unlock(mddev);

 return err;
done:
 if (err)
  { printk("md: bug in file %s, line %d\n", "drivers/md/md.c", 4451); md_print_devices(); };
abort:
 return err;
}

static int md_open(struct inode *inode, struct file *file)
{




 mddev_t *mddev = inode->i_bdev->bd_disk->private_data;
 int err;

 if ((err = mddev_lock(mddev)))
  goto out;

 err = 0;
 mddev_get(mddev);
 mddev_unlock(mddev);

 check_disk_change(inode->i_bdev);
 out:
 return err;
}

static int md_release(struct inode *inode, struct file * file)
{
  mddev_t *mddev = inode->i_bdev->bd_disk->private_data;

 if (!mddev)
  __asm__ __volatile__( "ud2\n" "\t.word %c0\n" "\t.long %c1\n" : : "i" (4482), "i" ("drivers/md/md.c"));
 mddev_put(mddev);

 return 0;
}

static int md_media_changed(struct gendisk *disk)
{
 mddev_t *mddev = disk->private_data;

 return mddev->changed;
}

static int md_revalidate(struct gendisk *disk)
{
 mddev_t *mddev = disk->private_data;

 mddev->changed = 0;
 return 0;
}
static struct block_device_operations md_fops =
{
 .owner = ((struct module *)0),
 .open = md_open,
 .release = md_release,
 .ioctl = md_ioctl,
 .getgeo = md_getgeo,
 .media_changed = md_media_changed,
 .revalidate_disk= md_revalidate,
};

static int md_thread(void * arg)
{
 mdk_thread_t *thread = arg;

 allow_signal(9);
 while (!kthread_should_stop()) {






  if (signal_pending(__vericon_dummy_current))
   flush_signals(__vericon_dummy_current);

  ({ long __ret = thread->timeout; if (!((__builtin_constant_p(0) ? constant_test_bit((0),(&thread->flags)) : variable_test_bit((0),(&thread->flags))) || kthread_should_stop())) do { wait_queue_t __wait = { .private = __vericon_dummy_current, .func = autoremove_wake_function, .task_list = { &((__wait).task_list), &((__wait).task_list) }, }; for (;;) { prepare_to_wait(&thread->wqueue, &__wait, 1); if ((__builtin_constant_p(0) ? constant_test_bit((0),(&thread->flags)) : variable_test_bit((0),(&thread->flags))) || kthread_should_stop()) break; if (!signal_pending(__vericon_dummy_current)) { __ret = schedule_timeout(__ret); if (!__ret) break; continue; } __ret = -512; break; } finish_wait(&thread->wqueue, &__wait); } while (0); __ret; });




  try_to_freeze();

  clear_bit(0, &thread->flags);

  thread->run(thread->mddev);
 }

 return 0;
}

void md_wakeup_thread(mdk_thread_t *thread)
{
 if (thread) {
  ((void)(0 && printk("md: waking up MD thread %s.\n", thread->tsk->comm)));
  set_bit(0, &thread->flags);
  __wake_up(&thread->wqueue, 2 | 1, 1, ((void *)0));
 }
}

extern mdk_thread_t *md_register_thread(void (*run) (mddev_t *), mddev_t *mddev, const char *name);
/* [kohei]
mdk_thread_t *md_register_thread(void (*run) (mddev_t *), mddev_t *mddev,
     const char *name)
{
 mdk_thread_t *thread;

 thread = kzalloc(sizeof(mdk_thread_t), ((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u)));
 if (!thread)
  return ((void *)0);

 init_waitqueue_head(&thread->wqueue);

 thread->run = run;
 thread->mddev = mddev;
 thread->timeout = ((long)(~0UL>>1));
 thread->tsk = ({ struct task_struct *__k = kthread_create(md_thread, thread, name, mdname(thread->mddev)); if (!IS_ERR(__k)) wake_up_process(__k); __k; });
 if (IS_ERR(thread->tsk)) {
  kfree(thread);
  return ((void *)0);
 }
 return thread;
}
*/
void md_unregister_thread(mdk_thread_t *thread)
{
 ((void)(0 && printk("interrupting MD-thread pid %d\n", thread->tsk->pid)));

 kthread_stop(thread->tsk);
 kfree(thread);
}

void md_error(mddev_t *mddev, mdk_rdev_t *rdev)
{
 if (!mddev) {
  { printk("md: bug in file %s, line %d\n", "drivers/md/md.c", 4597); md_print_devices(); };
  return;
 }

 if (!rdev || (__builtin_constant_p(1) ? constant_test_bit((1),(&rdev->flags)) : variable_test_bit((1),(&rdev->flags))))
  return;







 if (!mddev->pers)
  return;
 if (!mddev->pers->error_handler)
  return;
 mddev->pers->error_handler(mddev,rdev);
 set_bit(3, &mddev->recovery);
 set_bit(5, &mddev->recovery);
 md_wakeup_thread(mddev->thread);
 md_new_event_inintr(mddev);
}


extern void status_unused(struct seq_file *seq);
/* [kohei]
static void status_unused(struct seq_file *seq)
{
 int i = 0;
 mdk_rdev_t *rdev;
 struct list_head *tmp;

 seq_printf(seq, "unused devices: ");

 for ((tmp) = (pending_raid_disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp) = (tmp)->next, (tmp)->prev != &(pending_raid_disks) ; ) {
  char b[32];
  i++;
  seq_printf(seq, "%s ",
         bdevname(rdev->bdev,b));
 }
 if (!i)
  seq_printf(seq, "<none>");

 seq_printf(seq, "\n");
}
*/

static void status_resync(struct seq_file *seq, mddev_t * mddev)
{
 sector_t max_blocks, resync, res;
 unsigned long dt, db, rt;
 int scale;
 unsigned int per_milli;

 resync = (mddev->curr_resync - ((&mddev->recovery_active)->counter))/2;

 if ((__builtin_constant_p(1) ? constant_test_bit((1),(&mddev->recovery)) : variable_test_bit((1),(&mddev->recovery))))
  max_blocks = mddev->resync_max_sectors >> 1;
 else
  max_blocks = mddev->size;




 if (!max_blocks) {
  { printk("md: bug in file %s, line %d\n", "drivers/md/md.c", 4662); md_print_devices(); };
  return;
 }





 scale = 10;
 if (sizeof(sector_t) > sizeof(unsigned long)) {
  while ( max_blocks/2 > (1ULL<<(scale+32)))
   scale++;
 }
 res = (resync>>scale)*1000;
 ({ unsigned long __upper, __low, __high, __mod, __base; __base = ((u32)((max_blocks>>scale)+1)); asm("":"=a" (__low), "=d" (__high):"A" (res)); __upper = __high; if (__high) { __upper = __high % (__base); __high = __high / (__base); } asm("divl %2":"=a" (__low), "=d" (__mod):"rm" (__base), "0" (__low), "1" (__upper)); asm("":"=A" (res):"a" (__low),"d" (__high)); __mod; });

 per_milli = res;
 {
  int i, x = per_milli/50, y = 20-x;
  seq_printf(seq, "[");
  for (i = 0; i < x; i++)
   seq_printf(seq, "=");
  seq_printf(seq, ">");
  for (i = 0; i < y; i++)
   seq_printf(seq, ".");
  seq_printf(seq, "] ");
 }
 seq_printf(seq, " %s =%3u.%u%% (%llu/%llu)",
     ((__builtin_constant_p(8) ? constant_test_bit((8),(&mddev->recovery)) : variable_test_bit((8),(&mddev->recovery)))?
      "reshape" :
        ((__builtin_constant_p(1) ? constant_test_bit((1),(&mddev->recovery)) : variable_test_bit((1),(&mddev->recovery))) ?
         "resync" : "recovery")),
        per_milli/10, per_milli % 10,
     (unsigned long long) resync,
     (unsigned long long) max_blocks);

 dt = ((jiffies - mddev->resync_mark) / 1000);
 if (!dt) dt++;
 db = (mddev->curr_mark_cnt - ((&mddev->recovery_active)->counter))
  - mddev->resync_mark_cnt;
 rt = (dt * ((unsigned long)(max_blocks-resync) / (db/2/100+1)))/100;

 seq_printf(seq, " finish=%lu.%lumin", rt / 60, (rt % 60)/6);

 seq_printf(seq, " speed=%ldK/sec", db/2/dt);
}

extern void *md_seq_start(struct seq_file *seq, loff_t *pos);
/* [kohei]
static void *md_seq_start(struct seq_file *seq, loff_t *pos)
{
 struct list_head *tmp;
 loff_t l = *pos;
 mddev_t *mddev;

 if (l >= 0x10000)
  return ((void *)0);
 if (!l--)

  return (void*)1;

 _spin_lock(&all_mddevs_lock);
 for (tmp = (&all_mddevs)->next; prefetch(tmp->next), tmp != (&all_mddevs); tmp = tmp->next)
  if (!l--) {
    mddev = ({ const typeof( ((mddev_t *)0)->all_mddevs ) *__mptr = (tmp); (mddev_t *)( (char *)__mptr - __builtin_offsetof(mddev_t,all_mddevs) );});
   mddev_get(mddev);
   _spin_unlock(&all_mddevs_lock);
   return mddev;
  }
 _spin_unlock(&all_mddevs_lock);
 if (!l--)
  return (void*)2;
 return ((void *)0);
}
*/

extern void *md_seq_next(struct seq_file *seq, void *v, loff_t *pos);
/* [kohei]
static void *md_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
 struct list_head *tmp;
 mddev_t *next_mddev, *mddev = v;

 ++*pos;
 if (v == (void*)2)
  return ((void *)0);

 _spin_lock(&all_mddevs_lock);
 if (v == (void*)1)
  tmp = all_mddevs.next;
 else
  tmp = mddev->all_mddevs.next;
 if (tmp != &all_mddevs)
   next_mddev = mddev_get(({ const typeof( ((mddev_t *)0)->all_mddevs ) *__mptr = (tmp); (mddev_t *)( (char *)__mptr - __builtin_offsetof(mddev_t,all_mddevs) );}));
 else {
  next_mddev = (void*)2;
  *pos = 0x10000;
 }
 _spin_unlock(&all_mddevs_lock);

 if (v != (void*)1)
  mddev_put(mddev);
 return next_mddev;

}
*/

static void md_seq_stop(struct seq_file *seq, void *v)
{
 mddev_t *mddev = v;

 if (mddev && v != (void*)1 && v != (void*)2)
  mddev_put(mddev);
}

struct mdstat_info {
 int event;
};

extern int md_seq_show(struct seq_file *seq, void *v);
/* [kohei]
static int md_seq_show(struct seq_file *seq, void *v)
{
 mddev_t *mddev = v;
 sector_t size;
 struct list_head *tmp2;
 mdk_rdev_t *rdev;
 struct mdstat_info *mi = seq->private;
 struct bitmap *bitmap;

 if (v == (void*)1) {
  struct mdk_personality *pers;
  seq_printf(seq, "Personalities : ");
  _spin_lock(&pers_lock);
  for (pers = ({ const typeof( ((typeof(*pers) *)0)->list ) *__mptr = ((&pers_list)->next); (typeof(*pers) *)( (char *)__mptr - __builtin_offsetof(typeof(*pers),list) );}); prefetch(pers->list.next), &pers->list != (&pers_list); pers = ({ const typeof( ((typeof(*pers) *)0)->list ) *__mptr = (pers->list.next); (typeof(*pers) *)( (char *)__mptr - __builtin_offsetof(typeof(*pers),list) );}))
   seq_printf(seq, "[%s] ", pers->name);

  _spin_unlock(&pers_lock);
  seq_printf(seq, "\n");
  mi->event = ((&md_event_count)->counter);
  return 0;
 }
 if (v == (void*)2) {
  status_unused(seq);
  return 0;
 }

 if (mddev_lock(mddev) < 0)
  return -4;

 if (mddev->pers || mddev->raid_disks || !list_empty(&mddev->disks)) {
  seq_printf(seq, "%s : %sactive", mdname(mddev),
      mddev->pers ? "" : "in");
  if (mddev->pers) {
   if (mddev->ro==1)
    seq_printf(seq, " (read-only)");
   if (mddev->ro==2)
    seq_printf(seq, "(auto-read-only)");
   seq_printf(seq, " %s", mddev->pers->name);
  }

  size = 0;
  for ((tmp2) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp2)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp2) = (tmp2)->next, (tmp2)->prev != &((mddev)->disks) ; ) {
   char b[32];
   seq_printf(seq, " %s[%d]",
    bdevname(rdev->bdev,b), rdev->desc_nr);
   if ((__builtin_constant_p(4) ? constant_test_bit((4),(&rdev->flags)) : variable_test_bit((4),(&rdev->flags))))
    seq_printf(seq, "(W)");
   if ((__builtin_constant_p(1) ? constant_test_bit((1),(&rdev->flags)) : variable_test_bit((1),(&rdev->flags)))) {
    seq_printf(seq, "(F)");
    continue;
   } else if (rdev->raid_disk < 0)
    seq_printf(seq, "(S)");
   size += rdev->size;
  }

  if (!list_empty(&mddev->disks)) {
   if (mddev->pers)
    seq_printf(seq, "\n      %llu blocks",
     (unsigned long long)mddev->array_size);
   else
    seq_printf(seq, "\n      %llu blocks",
     (unsigned long long)size);
  }
  if (mddev->persistent) {
   if (mddev->major_version != 0 ||
       mddev->minor_version != 90) {
    seq_printf(seq," super %d.%d",
        mddev->major_version,
        mddev->minor_version);
   }
  } else
   seq_printf(seq, " super non-persistent");

  if (mddev->pers) {
   mddev->pers->status (seq, mddev);
    seq_printf(seq, "\n      ");
   if (mddev->pers->sync_request) {
    if (mddev->curr_resync > 2) {
     status_resync (seq, mddev);
     seq_printf(seq, "\n      ");
    } else if (mddev->curr_resync == 1 || mddev->curr_resync == 2)
     seq_printf(seq, "\tresync=DELAYED\n      ");
    else if (mddev->recovery_cp < (~(sector_t)0))
     seq_printf(seq, "\tresync=PENDING\n      ");
   }
  } else
   seq_printf(seq, "\n       ");

  if ((bitmap = mddev->bitmap)) {
   unsigned long chunk_kb;
   unsigned long flags;
   flags = _spin_lock_irqsave(&bitmap->lock);
   chunk_kb = bitmap->chunksize >> 10;
   seq_printf(seq, "bitmap: %lu/%lu pages [%luKB], "
    "%lu%s chunk",
    bitmap->pages - bitmap->missing_pages,
    bitmap->pages,
    (bitmap->pages - bitmap->missing_pages)
     << (12 - 10),
    chunk_kb ? chunk_kb : bitmap->chunksize,
    chunk_kb ? "KB" : "B");
   if (bitmap->file) {
    seq_printf(seq, ", file: ");
    seq_path(seq, bitmap->file->f_vfsmnt,
      bitmap->file->f_dentry," \t\n");
   }

   seq_printf(seq, "\n");
   _spin_unlock_irqrestore(&bitmap->lock, flags);
  }

  seq_printf(seq, "\n");
 }
 mddev_unlock(mddev);

 return 0;
}
*/

static struct seq_operations md_seq_ops = {
 .start = md_seq_start,
 .next = md_seq_next,
 .stop = md_seq_stop,
 .show = md_seq_show,
};

// [kohei] Leak?
static int md_seq_open(struct inode *inode, struct file *file)
{
 int error;
 struct mdstat_info *mi = kmalloc(sizeof(*mi), ((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u)));
 if (mi == ((void *)0)) {
   _fst_assume_null(mi);
   return -12;
 }

 error = seq_open(file, &md_seq_ops);
 if (error) {
   kfree(mi);
 } else {
   struct seq_file *p = file->private_data;
   p->private = mi;
   mi->event = ((&md_event_count)->counter);
 }
 return error;
}

static int md_seq_release(struct inode *inode, struct file *file)
{
 struct seq_file *m = file->private_data;
 struct mdstat_info *mi = m->private;
 m->private = ((void *)0);
 kfree(mi);
 return seq_release(inode, file);
}

static unsigned int mdstat_poll(struct file *filp, poll_table *wait)
{
 struct seq_file *m = filp->private_data;
 struct mdstat_info *mi = m->private;
 int mask;

 poll_wait(filp, &md_event_waiters, wait);


 mask = 0x0001 | 0x0040;

 if (mi->event != ((&md_event_count)->counter))
  mask |= 0x0008 | 0x0002;
 return mask;
}

static struct file_operations md_seq_fops = {
 .open = md_seq_open,
 .read = seq_read,
 .llseek = seq_lseek,
 .release = md_seq_release,
 .poll = mdstat_poll,
};

int register_md_personality(struct mdk_personality *p)
{
 _spin_lock(&pers_lock);
 list_add_tail(&p->list, &pers_list);
 printk("<6>" "md: %s personality registered for level %d\n", p->name, p->level);
 _spin_unlock(&pers_lock);
 return 0;
}

int unregister_md_personality(struct mdk_personality *p)
{
 printk("<6>" "md: %s personality unregistered\n", p->name);
 _spin_lock(&pers_lock);
 list_del_init(&p->list);
 _spin_unlock(&pers_lock);
 return 0;
}

extern int is_mddev_idle(mddev_t *mddev);
/* [kohei]
static int is_mddev_idle(mddev_t *mddev)
{
 mdk_rdev_t * rdev;
 struct list_head *tmp;
 int idle;
 unsigned long curr_events;

 idle = 1;
 for ((tmp) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((tmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (tmp) = (tmp)->next, (tmp)->prev != &((mddev)->disks) ; ) {
  struct gendisk *disk = rdev->bdev->bd_contains->bd_disk;
  curr_events = (disk->dkstats.sectors[0]) +
    (disk->dkstats.sectors[1]) -
    ((&disk->sync_io)->counter);

  if ((curr_events - rdev->last_events + 4096) > 8192) {
   rdev->last_events = curr_events;
   idle = 0;
  }
 }
 return idle;
}
*/
void md_done_sync(mddev_t *mddev, int blocks, int ok)
{

 atomic_sub(blocks, &mddev->recovery_active);
 __wake_up(&mddev->recovery_wait, 2 | 1, 1, ((void *)0));
 if (!ok) {
  set_bit(2, &mddev->recovery);
  md_wakeup_thread(mddev->thread);

 }
}







void md_write_start(mddev_t *mddev, struct bio *bi)
{
 if (((bi)->bi_rw & 1) != 1)
  return;

 do { if (__builtin_expect(!!((mddev->ro == 1)!=0), 0)) __asm__ __volatile__( "ud2\n" "\t.word %c0\n" "\t.long %c1\n" : : "i" (5035), "i" ("drivers/md/md.c")); } while(0);
 if (mddev->ro == 2) {

  mddev->ro = 0;
  set_bit(5, &mddev->recovery);
  md_wakeup_thread(mddev->thread);
 }
 atomic_inc(&mddev->writes_pending);
 if (mddev->in_sync) {
  _spin_lock_irq(&mddev->write_lock);
  if (mddev->in_sync) {
   mddev->in_sync = 0;
   mddev->sb_dirty = 3;
   md_wakeup_thread(mddev->thread);
  }
  _spin_unlock_irq(&mddev->write_lock);
 }
 do { if (mddev->sb_dirty==0) break; do { wait_queue_t __wait = { .private = __vericon_dummy_current, .func = autoremove_wake_function, .task_list = { &((__wait).task_list), &((__wait).task_list) }, }; for (;;) { prepare_to_wait(&mddev->sb_wait, &__wait, 2); if (mddev->sb_dirty==0) break; schedule(); } finish_wait(&mddev->sb_wait, &__wait); } while (0); } while (0);
}

void md_write_end(mddev_t *mddev)
{
 if (atomic_dec_and_test(&mddev->writes_pending)) {
  if (mddev->safemode == 2)
   md_wakeup_thread(mddev->thread);
  else if (mddev->safemode_delay)
   mod_timer(&mddev->safemode_timer, jiffies + mddev->safemode_delay);
 }
}

static wait_queue_head_t resync_wait = { .lock = (spinlock_t) { .raw_lock = { 1 }, .magic = 0xdead4ead, .owner = ((void *)-1L), .owner_cpu = -1, }, .task_list = { &(resync_wait).task_list, &(resync_wait).task_list } };


extern void md_do_sync(mddev_t *mddev);
/* [kohei]
void md_do_sync(mddev_t *mddev)
{
 mddev_t *mddev2;
 unsigned int currspeed = 0,
   window;
 sector_t max_sectors,j, io_sectors;
 unsigned long mark[10];
 sector_t mark_cnt[10];
 int last_mark,m;
 struct list_head *tmp;
 sector_t last_check;
 int skipped = 0;
 struct list_head *rtmp;
 mdk_rdev_t *rdev;


 if ((__builtin_constant_p(4) ? constant_test_bit((4),(&mddev->recovery)) : variable_test_bit((4),(&mddev->recovery))))
  return;
 if (mddev->ro)
  return;

 do {
  mddev->curr_resync = 2;

 try_again:
  if (kthread_should_stop()) {
   set_bit(3, &mddev->recovery);
   goto skip;
  }
  for (({ _spin_lock(&all_mddevs_lock); tmp = all_mddevs.next; mddev2 = ((void *)0);}); ({ if (tmp != &all_mddevs) mddev_get(({ const typeof( ((mddev_t *)0)->all_mddevs ) *__mptr = (tmp); (mddev_t *)( (char *)__mptr - __builtin_offsetof(mddev_t,all_mddevs) );})); _spin_unlock(&all_mddevs_lock); if (mddev2) mddev_put(mddev2); mddev2 = ({ const typeof( ((mddev_t *)0)->all_mddevs ) *__mptr = (tmp); (mddev_t *)( (char *)__mptr - __builtin_offsetof(mddev_t,all_mddevs) );}); tmp != &all_mddevs;}); ({ _spin_lock(&all_mddevs_lock); tmp = tmp->next;}) ) {
   if (mddev2 == mddev)
    continue;
   if (mddev2->curr_resync &&
       match_mddev_units(mddev,mddev2)) {
    wait_queue_t wq = { .private = __vericon_dummy_current, .func = autoremove_wake_function, .task_list = { &((wq).task_list), &((wq).task_list) }, };
    if (mddev < mddev2 && mddev->curr_resync == 2) {

     mddev->curr_resync = 1;
     __wake_up(&resync_wait, 2 | 1, 1, ((void *)0));
    }
    if (mddev > mddev2 && mddev->curr_resync == 1)



     continue;
    prepare_to_wait(&resync_wait, &wq, 2);
    if (!kthread_should_stop() &&
        mddev2->curr_resync >= mddev->curr_resync) {
     printk("<6>" "md: delaying resync of %s"
            " until %s has finished resync (they"
            " share one or more physical units)\n",
            mdname(mddev), mdname(mddev2));
     mddev_put(mddev2);
     schedule();
     finish_wait(&resync_wait, &wq);
     goto try_again;
    }
    finish_wait(&resync_wait, &wq);
   }
  }
 } while (mddev->curr_resync < 2);

 j = 0;
 if ((__builtin_constant_p(1) ? constant_test_bit((1),(&mddev->recovery)) : variable_test_bit((1),(&mddev->recovery)))) {



  max_sectors = mddev->resync_max_sectors;
  mddev->resync_mismatches = 0;

  if (!mddev->bitmap &&
      !(__builtin_constant_p(6) ? constant_test_bit((6),(&mddev->recovery)) : variable_test_bit((6),(&mddev->recovery))))
   j = mddev->recovery_cp;
 } else if ((__builtin_constant_p(8) ? constant_test_bit((8),(&mddev->recovery)) : variable_test_bit((8),(&mddev->recovery))))
  max_sectors = mddev->size << 1;
 else {

  max_sectors = mddev->size << 1;
  j = (~(sector_t)0);
  for ((rtmp) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((rtmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (rtmp) = (rtmp)->next, (rtmp)->prev != &((mddev)->disks) ; )
   if (rdev->raid_disk >= 0 &&
       !(__builtin_constant_p(1) ? constant_test_bit((1),(&rdev->flags)) : variable_test_bit((1),(&rdev->flags))) &&
       !(__builtin_constant_p(2) ? constant_test_bit((2),(&rdev->flags)) : variable_test_bit((2),(&rdev->flags))) &&
       rdev->recovery_offset < j)
    j = rdev->recovery_offset;
 }

 printk("<6>" "md: syncing RAID array %s\n", mdname(mddev));
 printk("<6>" "md: minimum _guaranteed_ reconstruction speed:"
  " %d KB/sec/disc.\n", speed_min(mddev));
 printk("<6>" "md: using maximum available idle IO bandwidth "
        "(but not more than %d KB/sec) for reconstruction.\n",
        speed_max(mddev));

 is_mddev_idle(mddev);

 io_sectors = 0;
 for (m = 0; m < 10; m++) {
  mark[m] = jiffies;
  mark_cnt[m] = io_sectors;
 }
 last_mark = 0;
 mddev->resync_mark = mark[last_mark];
 mddev->resync_mark_cnt = mark_cnt[last_mark];




 window = 32*((1UL << 12)/512);
 printk("<6>" "md: using %dk window, over a total of %llu blocks.\n",
  window/2,(unsigned long long) max_sectors/2);

 (((&mddev->recovery_active)->counter) = (0));
 init_waitqueue_head(&mddev->recovery_wait);
 last_check = 0;

 if (j>2) {
  printk("<6>"
   "md: resuming recovery of %s from checkpoint.\n",
   mdname(mddev));
  mddev->curr_resync = j;
 }

 while (j < max_sectors) {
  sector_t sectors;

  skipped = 0;
  sectors = mddev->pers->sync_request(mddev, j, &skipped,
         currspeed < speed_min(mddev));
  if (sectors == 0) {
   set_bit(2, &mddev->recovery);
   goto out;
  }

  if (!skipped) {
   io_sectors += sectors;
   atomic_add(sectors, &mddev->recovery_active);
  }

  j += sectors;
  if (j>1) mddev->curr_resync = j;
  mddev->curr_mark_cnt = io_sectors;
  if (last_check == 0)



   md_new_event(mddev);

  if (last_check + window > io_sectors || j == max_sectors)
   continue;

  last_check = io_sectors;

  if ((__builtin_constant_p(3) ? constant_test_bit((3),(&mddev->recovery)) : variable_test_bit((3),(&mddev->recovery))) ||
      (__builtin_constant_p(2) ? constant_test_bit((2),(&mddev->recovery)) : variable_test_bit((2),(&mddev->recovery))))
   break;

 repeat:
  if ((({ unsigned long __dummy; typeof(jiffies) __dummy2; (void)(&__dummy == &__dummy2); 1; }) && ({ unsigned long __dummy; typeof(mark[last_mark] + (3*1000)) __dummy2; (void)(&__dummy == &__dummy2); 1; }) && ((long)(jiffies) - (long)(mark[last_mark] + (3*1000)) >= 0))) {

   int next = (last_mark+1) % 10;

   mddev->resync_mark = mark[next];
   mddev->resync_mark_cnt = mark_cnt[next];
   mark[next] = jiffies;
   mark_cnt[next] = io_sectors - ((&mddev->recovery_active)->counter);
   last_mark = next;
  }


  if (kthread_should_stop()) {



   printk("<6>"
    "md: md_do_sync() got signal ... exiting\n");
   set_bit(3, &mddev->recovery);
   goto out;
  }

  mddev->queue->unplug_fn(mddev->queue);
  cond_resched();

  currspeed = ((unsigned long)(io_sectors-mddev->resync_mark_cnt))/2
   /((jiffies-mddev->resync_mark)/1000 +1) +1;

  if (currspeed > speed_min(mddev)) {
   if ((currspeed > speed_max(mddev)) ||
     !is_mddev_idle(mddev)) {
    msleep(500);
    goto repeat;
   }
  }
 }
 printk("<6>" "md: %s: sync done.\n",mdname(mddev));



 out:
 mddev->queue->unplug_fn(mddev->queue);

 do { if (!((&mddev->recovery_active)->counter)) break; do { wait_queue_t __wait = { .private = __vericon_dummy_current, .func = autoremove_wake_function, .task_list = { &((__wait).task_list), &((__wait).task_list) }, }; for (;;) { prepare_to_wait(&mddev->recovery_wait, &__wait, 2); if (!((&mddev->recovery_active)->counter)) break; schedule(); } finish_wait(&mddev->recovery_wait, &__wait); } while (0); } while (0);


 mddev->pers->sync_request(mddev, max_sectors, &skipped, 1);

 if (!(__builtin_constant_p(2) ? constant_test_bit((2),(&mddev->recovery)) : variable_test_bit((2),(&mddev->recovery))) &&
     (__builtin_constant_p(1) ? constant_test_bit((1),(&mddev->recovery)) : variable_test_bit((1),(&mddev->recovery))) &&
     !(__builtin_constant_p(7) ? constant_test_bit((7),(&mddev->recovery)) : variable_test_bit((7),(&mddev->recovery))) &&
     mddev->curr_resync > 2) {
  if ((__builtin_constant_p(1) ? constant_test_bit((1),(&mddev->recovery)) : variable_test_bit((1),(&mddev->recovery)))) {
   if ((__builtin_constant_p(3) ? constant_test_bit((3),(&mddev->recovery)) : variable_test_bit((3),(&mddev->recovery)))) {
    if (mddev->curr_resync >= mddev->recovery_cp) {
     printk("<6>"
            "md: checkpointing recovery of %s.\n",
            mdname(mddev));
     mddev->recovery_cp = mddev->curr_resync;
    }
   } else
    mddev->recovery_cp = (~(sector_t)0);
  } else {
   if (!(__builtin_constant_p(3) ? constant_test_bit((3),(&mddev->recovery)) : variable_test_bit((3),(&mddev->recovery))))
    mddev->curr_resync = (~(sector_t)0);
   for ((rtmp) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((rtmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (rtmp) = (rtmp)->next, (rtmp)->prev != &((mddev)->disks) ; )
    if (rdev->raid_disk >= 0 &&
        !(__builtin_constant_p(1) ? constant_test_bit((1),(&rdev->flags)) : variable_test_bit((1),(&rdev->flags))) &&
        !(__builtin_constant_p(2) ? constant_test_bit((2),(&rdev->flags)) : variable_test_bit((2),(&rdev->flags))) &&
        rdev->recovery_offset < mddev->curr_resync)
     rdev->recovery_offset = mddev->curr_resync;
   mddev->sb_dirty = 1;
  }
 }

 skip:
 mddev->curr_resync = 0;
 __wake_up(&resync_wait, 2 | 1, 1, ((void *)0));
 set_bit(4, &mddev->recovery);
 md_wakeup_thread(mddev->thread);
}
*/
extern typeof(md_do_sync) md_do_sync; extern void *__crc_md_do_sync __attribute__((weak)); static const unsigned long __kcrctab_md_do_sync __attribute__((__used__)) __attribute__((section("__kcrctab" "_gpl"), unused)) = (unsigned long) &__crc_md_do_sync; static const char __kstrtab_md_do_sync[] __attribute__((section("__ksymtab_strings"))) = "" "md_do_sync"; static const struct kernel_symbol __ksymtab_md_do_sync __attribute__((__used__)) __attribute__((section("__ksymtab" "_gpl"), unused)) = { (unsigned long)&md_do_sync, __kstrtab_md_do_sync };

extern void md_check_recovery(mddev_t *mddev);
/* [kohei]
void md_check_recovery(mddev_t *mddev)
{
 mdk_rdev_t *rdev;
 struct list_head *rtmp;


 if (mddev->bitmap)
  bitmap_daemon_work(mddev->bitmap);

 if (mddev->ro)
  return;

 if (signal_pending(__vericon_dummy_current)) {
  if (mddev->pers->sync_request) {
   printk("<6>" "md: %s in immediate safe mode\n",
          mdname(mddev));
   mddev->safemode = 2;
  }
  flush_signals(__vericon_dummy_current);
 }

 if ( ! (
  mddev->sb_dirty ||
  (__builtin_constant_p(5) ? constant_test_bit((5),(&mddev->recovery)) : variable_test_bit((5),(&mddev->recovery))) ||
  (__builtin_constant_p(4) ? constant_test_bit((4),(&mddev->recovery)) : variable_test_bit((4),(&mddev->recovery))) ||
  (mddev->safemode == 1) ||
  (mddev->safemode == 2 && ! ((&mddev->writes_pending)->counter)
   && !mddev->in_sync && mddev->recovery_cp == (~(sector_t)0))
  ))
  return;

 if (mddev_trylock(mddev)) {
  int spares =0;

  _spin_lock_irq(&mddev->write_lock);
  if (mddev->safemode && !((&mddev->writes_pending)->counter) &&
      !mddev->in_sync && mddev->recovery_cp == (~(sector_t)0)) {
   mddev->in_sync = 1;
   mddev->sb_dirty = 3;
  }
  if (mddev->safemode == 1)
   mddev->safemode = 0;
  _spin_unlock_irq(&mddev->write_lock);

  if (mddev->sb_dirty)
   md_update_sb(mddev);


  if ((__builtin_constant_p(0) ? constant_test_bit((0),(&mddev->recovery)) : variable_test_bit((0),(&mddev->recovery))) &&
      !(__builtin_constant_p(4) ? constant_test_bit((4),(&mddev->recovery)) : variable_test_bit((4),(&mddev->recovery)))) {

   clear_bit(5, &mddev->recovery);
   goto unlock;
  }
  if (mddev->sync_thread) {

   md_unregister_thread(mddev->sync_thread);
   mddev->sync_thread = ((void *)0);
   if (!(__builtin_constant_p(2) ? constant_test_bit((2),(&mddev->recovery)) : variable_test_bit((2),(&mddev->recovery))) &&
       !(__builtin_constant_p(3) ? constant_test_bit((3),(&mddev->recovery)) : variable_test_bit((3),(&mddev->recovery)))) {


    mddev->pers->spare_active(mddev);
   }
   md_update_sb(mddev);




   if (!mddev->degraded)
    for ((rtmp) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((rtmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (rtmp) = (rtmp)->next, (rtmp)->prev != &((mddev)->disks) ; )
     rdev->saved_raid_disk = -1;

   mddev->recovery = 0;

   set_bit(5, &mddev->recovery);
   md_new_event(mddev);
   goto unlock;
  }



  clear_bit(5, &mddev->recovery);
  clear_bit(2, &mddev->recovery);
  clear_bit(3, &mddev->recovery);
  clear_bit(4, &mddev->recovery);

  if ((__builtin_constant_p(9) ? constant_test_bit((9),(&mddev->recovery)) : variable_test_bit((9),(&mddev->recovery))))
   goto unlock;






  for ((rtmp) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((rtmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (rtmp) = (rtmp)->next, (rtmp)->prev != &((mddev)->disks) ; )
   if (rdev->raid_disk >= 0 &&
       ((__builtin_constant_p(1) ? constant_test_bit((1),(&rdev->flags)) : variable_test_bit((1),(&rdev->flags))) || ! (__builtin_constant_p(2) ? constant_test_bit((2),(&rdev->flags)) : variable_test_bit((2),(&rdev->flags)))) &&
       ((&rdev->nr_pending)->counter)==0) {
    if (mddev->pers->hot_remove_disk(mddev, rdev->raid_disk)==0) {
     char nm[20];
     sprintf(nm,"rd%d", rdev->raid_disk);
     sysfs_remove_link(&mddev->kobj, nm);
     rdev->raid_disk = -1;
    }
   }

  if (mddev->degraded) {
   for ((rtmp) = ((mddev)->disks).next; (rdev) = (({ const typeof( ((mdk_rdev_t *)0)->same_set ) *__mptr = ((rtmp)); (mdk_rdev_t *)( (char *)__mptr - __builtin_offsetof(mdk_rdev_t,same_set) );})), (rtmp) = (rtmp)->next, (rtmp)->prev != &((mddev)->disks) ; )
    if (rdev->raid_disk < 0
        && !(__builtin_constant_p(1) ? constant_test_bit((1),(&rdev->flags)) : variable_test_bit((1),(&rdev->flags)))) {
     rdev->recovery_offset = 0;
     if (mddev->pers->hot_add_disk(mddev,rdev)) {
      char nm[20];
      sprintf(nm, "rd%d", rdev->raid_disk);
      sysfs_create_link(&mddev->kobj, &rdev->kobj, nm);
      spares++;
      md_new_event(mddev);
     } else
      break;
    }
  }

  if (spares) {
   clear_bit(1, &mddev->recovery);
   clear_bit(7, &mddev->recovery);
  } else if (mddev->recovery_cp < (~(sector_t)0)) {
   set_bit(1, &mddev->recovery);
  } else if (!(__builtin_constant_p(1) ? constant_test_bit((1),(&mddev->recovery)) : variable_test_bit((1),(&mddev->recovery))))

   goto unlock;

  if (mddev->pers->sync_request) {
   set_bit(0, &mddev->recovery);
   if (spares && mddev->bitmap && ! mddev->bitmap->file) {




    bitmap_write_all(mddev->bitmap);
   }
   mddev->sync_thread = md_register_thread(md_do_sync,
        mddev,
        "%s_resync");
   if (!mddev->sync_thread) {
    printk("<3>" "%s: could not start resync"
     " thread...\n",
     mdname(mddev));

    mddev->recovery = 0;
   } else
    md_wakeup_thread(mddev->sync_thread);
   md_new_event(mddev);
  }
 unlock:
  mddev_unlock(mddev);
 }
}
*/

extern int md_notify_reboot(struct notifier_block *this, unsigned long code, void *x);
/* [kohei]
static int md_notify_reboot(struct notifier_block *this,
       unsigned long code, void *x)
{
 struct list_head *tmp;
 mddev_t *mddev;

 if ((code == 0x0001) || (code == 0x0002) || (code == 0x0003)) {

  printk("<6>" "md: stopping all md devices.\n");

  for (({ _spin_lock(&all_mddevs_lock); tmp = all_mddevs.next; mddev = ((void *)0);}); ({ if (tmp != &all_mddevs) mddev_get(({ const typeof( ((mddev_t *)0)->all_mddevs ) *__mptr = (tmp); (mddev_t *)( (char *)__mptr - __builtin_offsetof(mddev_t,all_mddevs) );})); _spin_unlock(&all_mddevs_lock); if (mddev) mddev_put(mddev); mddev = ({ const typeof( ((mddev_t *)0)->all_mddevs ) *__mptr = (tmp); (mddev_t *)( (char *)__mptr - __builtin_offsetof(mddev_t,all_mddevs) );}); tmp != &all_mddevs;}); ({ _spin_lock(&all_mddevs_lock); tmp = tmp->next;}) )
   if (mddev_trylock(mddev)) {
    do_md_stop (mddev, 1);
    mddev_unlock(mddev);
   }






  ( (__builtin_constant_p(1000*1) && (1000*1)<=5) ? (__builtin_constant_p((1000*1)*1000) ? (((1000*1)*1000) > 20000 ? __bad_udelay() : __const_udelay(((1000*1)*1000) * 0x10c7ul)) : __udelay((1000*1)*1000)) : ({unsigned long __ms=(1000*1); while (__ms--) (__builtin_constant_p(1000) ? ((1000) > 20000 ? __bad_udelay() : __const_udelay((1000) * 0x10c7ul)) : __udelay(1000));}));
 }
 return 0x0000;
}
*/
static struct notifier_block md_notifier = {
 .notifier_call = md_notify_reboot,
 .next = ((void *)0),
 .priority = ((int)(~0U>>1)),
};

static void md_geninit(void)
{
 struct proc_dir_entry *p;

 ((void)(0 && printk("md: sizeof(mdp_super_t) = %d\n", (int)sizeof(mdp_super_t))));

 p = create_proc_entry("mdstat", (00400|00040|00004), ((void *)0));
 if (p)
  p->proc_fops = &md_seq_fops;
}

static int __attribute__ ((__section__ (".init.text"))) md_init(void)
{
 printk("<6>" "md: md driver %d.%d.%d MAX_MD_DEVS=%d,"
   " MD_SB_DISKS=%d\n",
   0, 90,
   3, 256, 27);
 printk("<6>" "md: bitmap version %d.%d\n", 4,
   39);

 if (register_blkdev(9, "md"))
  return -1;
 if ((mdp_major=register_blkdev(0, "mdp"))<=0) {
  unregister_blkdev(9, "md");
  return -1;
 }
 blk_register_region((((9) << 20) | (0)), 256, ((struct module *)0),
    md_probe, ((void *)0), ((void *)0));
 blk_register_region((((mdp_major) << 20) | (0)), 256<<6, ((struct module *)0),
       md_probe, ((void *)0), ((void *)0));

 register_reboot_notifier(&md_notifier);
 raid_table_header = register_sysctl_table(raid_root_table, 1);

 md_geninit();
 return (0);
}

static dev_t detected_devices[128];
static int dev_cnt;

void md_autodetect_dev(dev_t dev)
{
 if (dev_cnt >= 0 && dev_cnt < 127)
  detected_devices[dev_cnt++] = dev;
}


static void autostart_arrays(int part)
{
 mdk_rdev_t *rdev;
 int i;

 printk("<6>" "md: Autodetecting RAID arrays.\n");

 for (i = 0; i < dev_cnt; i++) {
  dev_t dev = detected_devices[i];

  rdev = md_import_device(dev,0, 0);
  if (IS_ERR(rdev))
   continue;

  if ((__builtin_constant_p(1) ? constant_test_bit((1),(&rdev->flags)) : variable_test_bit((1),(&rdev->flags)))) {
   { printk("md: bug in file %s, line %d\n", "drivers/md/md.c", 5618); md_print_devices(); };
   continue;
  }
  list_add(&rdev->same_set, &pending_raid_disks);
 }
 dev_cnt = 0;

 autorun_devices(part);
}


extern __attribute__((__used__)) __attribute__ ((__section__(".exit.text"))) void md_exit(void);
/* [kohei]
static __attribute__((__used__)) __attribute__ ((__section__(".exit.text"))) void md_exit(void)
{
 mddev_t *mddev;
 struct list_head *tmp;

 blk_unregister_region((((9) << 20) | (0)), 256);
 blk_unregister_region((((mdp_major) << 20) | (0)), 256 << 6);

 unregister_blkdev(9,"md");
 unregister_blkdev(mdp_major, "mdp");
 unregister_reboot_notifier(&md_notifier);
 unregister_sysctl_table(raid_table_header);
 remove_proc_entry("mdstat", ((void *)0));
 for (({ _spin_lock(&all_mddevs_lock); tmp = all_mddevs.next; mddev = ((void *)0);}); ({ if (tmp != &all_mddevs) mddev_get(({ const typeof( ((mddev_t *)0)->all_mddevs ) *__mptr = (tmp); (mddev_t *)( (char *)__mptr - __builtin_offsetof(mddev_t,all_mddevs) );})); _spin_unlock(&all_mddevs_lock); if (mddev) mddev_put(mddev); mddev = ({ const typeof( ((mddev_t *)0)->all_mddevs ) *__mptr = (tmp); (mddev_t *)( (char *)__mptr - __builtin_offsetof(mddev_t,all_mddevs) );}); tmp != &all_mddevs;}); ({ _spin_lock(&all_mddevs_lock); tmp = tmp->next;}) ) {
  struct gendisk *disk = mddev->gendisk;
  if (!disk)
   continue;
  export_array(mddev);
  del_gendisk(disk);
  put_disk(disk);
  mddev->gendisk = ((void *)0);
  mddev_put(mddev);
 }
}
*/
static initcall_t __initcall_md_init __attribute__((__used__)) __attribute__((__section__(".initcall" "6" ".init"))) = md_init;
static exitcall_t __exitcall_md_exit __attribute__((__used__)) __attribute__ ((__section__ (".exitcall.exit"))) = md_exit;

static int get_ro(char *buffer, struct kernel_param *kp)
{
 return sprintf(buffer, "%d", start_readonly);
}
static int set_ro(const char *val, struct kernel_param *kp)
{
 char *e;
 int num = simple_strtoul(val, &e, 10);
 if (*val && (*e == '\0' || *e == '\n')) {
  start_readonly = num;
  return 0;
 }
 return -22;
}

static char __param_str_start_ro[] = "md_mod" "." "start_ro"; static struct kernel_param const __param_start_ro __attribute__((__used__)) __attribute__ ((unused,__section__ ("__param"),aligned(sizeof(void *)))) = { __param_str_start_ro, 00400|00200, set_ro, get_ro, ((void *)0) };
static inline __attribute__((always_inline)) int *__check_start_dirty_degraded(void) { return(&(start_dirty_degraded)); }; static char __param_str_start_dirty_degraded[] = "md_mod" "." "start_dirty_degraded"; static struct kernel_param const __param_start_dirty_degraded __attribute__((__used__)) __attribute__ ((unused,__section__ ("__param"),aligned(sizeof(void *)))) = { __param_str_start_dirty_degraded, (00400|00040|00004)|00200, param_set_int, param_get_int, &start_dirty_degraded }; ;


extern typeof(register_md_personality) register_md_personality; extern void *__crc_register_md_personality __attribute__((weak)); static const unsigned long __kcrctab_register_md_personality __attribute__((__used__)) __attribute__((section("__kcrctab" ""), unused)) = (unsigned long) &__crc_register_md_personality; static const char __kstrtab_register_md_personality[] __attribute__((section("__ksymtab_strings"))) = "" "register_md_personality"; static const struct kernel_symbol __ksymtab_register_md_personality __attribute__((__used__)) __attribute__((section("__ksymtab" ""), unused)) = { (unsigned long)&register_md_personality, __kstrtab_register_md_personality };
extern typeof(unregister_md_personality) unregister_md_personality; extern void *__crc_unregister_md_personality __attribute__((weak)); static const unsigned long __kcrctab_unregister_md_personality __attribute__((__used__)) __attribute__((section("__kcrctab" ""), unused)) = (unsigned long) &__crc_unregister_md_personality; static const char __kstrtab_unregister_md_personality[] __attribute__((section("__ksymtab_strings"))) = "" "unregister_md_personality"; static const struct kernel_symbol __ksymtab_unregister_md_personality __attribute__((__used__)) __attribute__((section("__ksymtab" ""), unused)) = { (unsigned long)&unregister_md_personality, __kstrtab_unregister_md_personality };
extern typeof(md_error) md_error; extern void *__crc_md_error __attribute__((weak)); static const unsigned long __kcrctab_md_error __attribute__((__used__)) __attribute__((section("__kcrctab" ""), unused)) = (unsigned long) &__crc_md_error; static const char __kstrtab_md_error[] __attribute__((section("__ksymtab_strings"))) = "" "md_error"; static const struct kernel_symbol __ksymtab_md_error __attribute__((__used__)) __attribute__((section("__ksymtab" ""), unused)) = { (unsigned long)&md_error, __kstrtab_md_error };
extern typeof(md_done_sync) md_done_sync; extern void *__crc_md_done_sync __attribute__((weak)); static const unsigned long __kcrctab_md_done_sync __attribute__((__used__)) __attribute__((section("__kcrctab" ""), unused)) = (unsigned long) &__crc_md_done_sync; static const char __kstrtab_md_done_sync[] __attribute__((section("__ksymtab_strings"))) = "" "md_done_sync"; static const struct kernel_symbol __ksymtab_md_done_sync __attribute__((__used__)) __attribute__((section("__ksymtab" ""), unused)) = { (unsigned long)&md_done_sync, __kstrtab_md_done_sync };
extern typeof(md_write_start) md_write_start; extern void *__crc_md_write_start __attribute__((weak)); static const unsigned long __kcrctab_md_write_start __attribute__((__used__)) __attribute__((section("__kcrctab" ""), unused)) = (unsigned long) &__crc_md_write_start; static const char __kstrtab_md_write_start[] __attribute__((section("__ksymtab_strings"))) = "" "md_write_start"; static const struct kernel_symbol __ksymtab_md_write_start __attribute__((__used__)) __attribute__((section("__ksymtab" ""), unused)) = { (unsigned long)&md_write_start, __kstrtab_md_write_start };
extern typeof(md_write_end) md_write_end; extern void *__crc_md_write_end __attribute__((weak)); static const unsigned long __kcrctab_md_write_end __attribute__((__used__)) __attribute__((section("__kcrctab" ""), unused)) = (unsigned long) &__crc_md_write_end; static const char __kstrtab_md_write_end[] __attribute__((section("__ksymtab_strings"))) = "" "md_write_end"; static const struct kernel_symbol __ksymtab_md_write_end __attribute__((__used__)) __attribute__((section("__ksymtab" ""), unused)) = { (unsigned long)&md_write_end, __kstrtab_md_write_end };
extern typeof(md_register_thread) md_register_thread; extern void *__crc_md_register_thread __attribute__((weak)); static const unsigned long __kcrctab_md_register_thread __attribute__((__used__)) __attribute__((section("__kcrctab" ""), unused)) = (unsigned long) &__crc_md_register_thread; static const char __kstrtab_md_register_thread[] __attribute__((section("__ksymtab_strings"))) = "" "md_register_thread"; static const struct kernel_symbol __ksymtab_md_register_thread __attribute__((__used__)) __attribute__((section("__ksymtab" ""), unused)) = { (unsigned long)&md_register_thread, __kstrtab_md_register_thread };
extern typeof(md_unregister_thread) md_unregister_thread; extern void *__crc_md_unregister_thread __attribute__((weak)); static const unsigned long __kcrctab_md_unregister_thread __attribute__((__used__)) __attribute__((section("__kcrctab" ""), unused)) = (unsigned long) &__crc_md_unregister_thread; static const char __kstrtab_md_unregister_thread[] __attribute__((section("__ksymtab_strings"))) = "" "md_unregister_thread"; static const struct kernel_symbol __ksymtab_md_unregister_thread __attribute__((__used__)) __attribute__((section("__ksymtab" ""), unused)) = { (unsigned long)&md_unregister_thread, __kstrtab_md_unregister_thread };
extern typeof(md_wakeup_thread) md_wakeup_thread; extern void *__crc_md_wakeup_thread __attribute__((weak)); static const unsigned long __kcrctab_md_wakeup_thread __attribute__((__used__)) __attribute__((section("__kcrctab" ""), unused)) = (unsigned long) &__crc_md_wakeup_thread; static const char __kstrtab_md_wakeup_thread[] __attribute__((section("__ksymtab_strings"))) = "" "md_wakeup_thread"; static const struct kernel_symbol __ksymtab_md_wakeup_thread __attribute__((__used__)) __attribute__((section("__ksymtab" ""), unused)) = { (unsigned long)&md_wakeup_thread, __kstrtab_md_wakeup_thread };
extern typeof(md_check_recovery) md_check_recovery; extern void *__crc_md_check_recovery __attribute__((weak)); static const unsigned long __kcrctab_md_check_recovery __attribute__((__used__)) __attribute__((section("__kcrctab" ""), unused)) = (unsigned long) &__crc_md_check_recovery; static const char __kstrtab_md_check_recovery[] __attribute__((section("__ksymtab_strings"))) = "" "md_check_recovery"; static const struct kernel_symbol __ksymtab_md_check_recovery __attribute__((__used__)) __attribute__((section("__ksymtab" ""), unused)) = { (unsigned long)&md_check_recovery, __kstrtab_md_check_recovery };
;
;
;
