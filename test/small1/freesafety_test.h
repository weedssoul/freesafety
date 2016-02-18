
#define NULL '\0'

struct dl_node {
  struct dl_node *next;
  struct dl_node *prev;
};

struct bt_node {
  struct bt_node *right;
  struct bt_node *left;
};

struct ll_node {
  struct ll_node *next;
};

extern void *malloc(unsigned int size);
extern void free(void *p);
extern void _fst_assume_null(void *p);
extern void _fst_assert_eq(void *p1, void *p2);
extern void _fst_assert_empty(void *p);
extern void _fst_cast(void *p);
extern int random(void);
extern void skip();
