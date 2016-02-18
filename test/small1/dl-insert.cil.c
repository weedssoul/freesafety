/* Generated by CIL v. 1.3.6 */
/* print_CIL_Input is true */

//#line  4 "test/small1/freesafety_test.h"
struct dl_node {
   struct dl_node *next ;
   struct dl_node *prev ;
};
//#line  18
extern void malloc(void **__fst_ret , unsigned int size ) ;
//#line  19
extern void free(void *p ) ;
//#line  20
extern void _fst_assume_null(void *p ) ;
//#line  21
extern void _fst_assert_eq(void *p1 , void *p2 ) ;
//#line  4 "test/small1/dl-insert.c"
void insert(struct dl_node *elm , struct dl_node *l ) 
{ 

  {
//#line  12
  elm->prev = l;
//#line  17
  elm->next = l->next;
//#line  22
  l->next = elm;
//#line  23
  (elm->next)->prev = elm;
//#line  28
  _fst_assert_eq((void *)l->next, (void *)elm);
//#line  33
  _fst_assert_eq((void *)l, (void *)elm->prev);
//#line  36
  return;
}
}
//#line  38 "test/small1/dl-insert.c"
void free_list(struct dl_node *list ) 
{ struct dl_node *p ;
  struct dl_node *tmp ;

  {
//#line  39
  p = (struct dl_node *)'\000';
//#line  40
  tmp = (struct dl_node *)'\000';
//#line  45
  p = list->next;
//#line  49
  while ((unsigned int )p != (unsigned int )((struct dl_node *)0)) {
//#line  53
    tmp = p->next;
//#line  57
    free((void *)p);
//#line  61
    p = tmp;
  }
//#line  66
  _fst_assert_eq((void *)p, (void *)tmp);
//#line  70
  _fst_assume_null((void *)p);
//#line  71
  _fst_assert_eq((void *)p, (void *)tmp);
//#line  75
  p = list->prev;
//#line  79
  while ((unsigned int )p != (unsigned int )((struct dl_node *)0)) {
//#line  83
    tmp = p->prev;
//#line  87
    free((void *)p);
//#line  91
    p = tmp;
  }
//#line  96
  _fst_assert_eq((void *)p, (void *)tmp);
//#line  100
  _fst_assume_null((void *)p);
//#line  101
  _fst_assert_eq((void *)p, (void *)tmp);
//#line  105
  free((void *)list);
//#line  109
  return;
}
}
//#line  112 "test/small1/dl-insert.c"
void make_list(struct dl_node **__fst_ret , unsigned int n ) 
{ struct dl_node *ret ;
  struct dl_node *tmp ;
  void *tmp___0 ;
  void **__vericon_ptr_tmp___0 ;

  {
//#line  112
  __vericon_ptr_tmp___0 = & tmp___0;
//#line  113
  ret = (struct dl_node *)'\000';
//#line  114
  tmp = (struct dl_node *)'\000';
//#line  117
  while (n > 0U) {
//#line  120
    malloc(__vericon_ptr_tmp___0, sizeof(struct dl_node ));
//#line  120
    tmp = (struct dl_node *)*__vericon_ptr_tmp___0;
//#line  123
    tmp->next = ret;
//#line  126
    ret->prev = tmp;
//#line  127
    _fst_assert_eq((void *)tmp->next, (void *)ret);
//#line  130
    ret = tmp;
  }
//#line  136
  ret->prev = (struct dl_node *)'\000';
  {
//#line  139
  *__fst_ret = ret;
//#line  139
  return;
  }
}
}
//#line  142 "test/small1/dl-insert.c"
void main(int *__fst_ret , int argc , char **argv ) 
{ struct dl_node *l ;
  struct dl_node **__vericon_ptr_l ;

  {
//#line  142
  __vericon_ptr_l = & l;
//#line  146
  make_list(__vericon_ptr_l, 5U);
//#line  149
  free_list(*__vericon_ptr_l);
  {
//#line  151
  *__fst_ret = 0;
//#line  151
  return;
  }
}
}