/* Generated by CIL v. 1.3.6 */
/* print_CIL_Input is true */

//#line  14 "test/small1/freesafety_test.h"
struct ll_node {
   struct ll_node *next ;
};
//#line  18
extern void malloc(void **__fst_ret , unsigned int size ) ;
//#line  19
extern void free(void *p ) ;
//#line  20
extern void _fst_assume_null(void *p ) ;
//#line  4 "test/small1/ll-reverse.c"
void reverse(struct ll_node **__fst_ret , struct ll_node *l ) 
{ struct ll_node *p ;
  struct ll_node *prev ;
  struct ll_node *next ;

  {
//#line  5
  p = l;
//#line  6
  prev = (struct ll_node *)'\000';
//#line  7
  next = (struct ll_node *)'\000';
//#line  8
  while ((unsigned int )p != (unsigned int )((struct ll_node *)0)) {
//#line  9
    next = p->next;
//#line  10
    p->next = prev;
//#line  11
    prev = p;
//#line  12
    p = next;
  }
//#line  14
  _fst_assume_null((void *)p);
  {
//#line  15
  *__fst_ret = prev;
//#line  15
  return;
  }
}
}
//#line  18 "test/small1/ll-reverse.c"
void make_list(struct ll_node **__fst_ret , unsigned int n ) 
{ unsigned int i ;
  struct ll_node *ret ;
  struct ll_node *tmp ;
  void *tmp___0 ;
  void **__vericon_ptr_tmp___0 ;

  {
//#line  18
  __vericon_ptr_tmp___0 = & tmp___0;
//#line  20
  i = 0U;
//#line  21
  ret = (struct ll_node *)'\000';
//#line  22
  tmp = (struct ll_node *)'\000';
//#line  23
  i = 0U;
//#line  23
  while (i < n) {
//#line  24
    malloc(__vericon_ptr_tmp___0, sizeof(struct ll_node ));
//#line  24
    tmp = (struct ll_node *)*__vericon_ptr_tmp___0;
//#line  25
    tmp->next = ret;
//#line  26
    ret = tmp;
//#line  23
    i ++;
  }
  {
//#line  28
  *__fst_ret = ret;
//#line  28
  return;
  }
}
}
//#line  31 "test/small1/ll-reverse.c"
void free_list(struct ll_node *l ) 
{ struct ll_node *p ;
  struct ll_node *tmp ;

  {
//#line  33
  p = l;
//#line  34
  while ((unsigned int )p != (unsigned int )((struct ll_node *)0)) {
//#line  35
    tmp = p->next;
//#line  36
    free((void *)p);
//#line  37
    p = tmp;
  }
//#line  39
  _fst_assume_null((void *)p);
//#line  40
  return;
}
}
//#line  42 "test/small1/ll-reverse.c"
void main(int *__fst_ret , int argc , char **argv ) 
{ struct ll_node *tmp ;
  struct ll_node *tmp___0 ;
  struct ll_node **__vericon_ptr_tmp ;
  struct ll_node **__vericon_ptr_tmp___0 ;

  {
//#line  42
  __vericon_ptr_tmp = & tmp;
//#line  42
  __vericon_ptr_tmp___0 = & tmp___0;
//#line  44
  make_list(__vericon_ptr_tmp, 5U);
//#line  44
  reverse(__vericon_ptr_tmp___0, *__vericon_ptr_tmp);
//#line  44
  free_list(*__vericon_ptr_tmp___0);
  {
//#line  45
  *__fst_ret = 0;
//#line  45
  return;
  }
}
}