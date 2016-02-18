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
//#line  23
extern void random(int *__fst_ret ) ;
//#line  4 "test/small1/ll-search.c"
void search(struct ll_node **__fst_ret , struct ll_node *l ) 
{ struct ll_node *p ;
  int tmp ;
  int *__vericon_ptr_tmp ;

  {
//#line  4
  __vericon_ptr_tmp = & tmp;
//#line  6
  p = (struct ll_node *)'\000';
//#line  7
  while (1) {
//#line  8
    random(__vericon_ptr_tmp);
//#line  8
    if (*__vericon_ptr_tmp) {
//#line  9
      p = l->next;
//#line  10
      l->next = p->next;
//#line  11
      break;
    }
  }
  {
//#line  14
  *__fst_ret = p;
//#line  14
  return;
  }
}
}
//#line  17 "test/small1/ll-search.c"
void make_list(struct ll_node **__fst_ret , unsigned int n ) 
{ unsigned int i ;
  struct ll_node *ret ;
  struct ll_node *tmp ;
  void *tmp___0 ;
  void **__vericon_ptr_tmp___0 ;

  {
//#line  17
  __vericon_ptr_tmp___0 = & tmp___0;
//#line  19
  i = 0U;
//#line  20
  ret = (struct ll_node *)'\000';
//#line  21
  tmp = (struct ll_node *)'\000';
//#line  22
  i = 0U;
//#line  22
  while (i < n) {
//#line  23
    malloc(__vericon_ptr_tmp___0, sizeof(struct ll_node ));
//#line  23
    tmp = (struct ll_node *)*__vericon_ptr_tmp___0;
//#line  24
    tmp->next = ret;
//#line  25
    ret = tmp;
//#line  22
    i ++;
  }
  {
//#line  27
  *__fst_ret = ret;
//#line  27
  return;
  }
}
}
//#line  30 "test/small1/ll-search.c"
void free_list(struct ll_node *l ) 
{ struct ll_node *p ;
  struct ll_node *tmp ;

  {
//#line  32
  p = l;
//#line  33
  while ((unsigned int )p != (unsigned int )((struct ll_node *)0)) {
//#line  34
    tmp = p->next;
//#line  35
    free((void *)p);
//#line  36
    p = tmp;
  }
//#line  38
  _fst_assume_null((void *)p);
//#line  39
  return;
}
}
//#line  41 "test/small1/ll-search.c"
void main(int *__fst_ret , int argc , char **argv ) 
{ struct ll_node *elm ;
  struct ll_node *l ;
  struct ll_node **__vericon_ptr_elm ;
  struct ll_node **__vericon_ptr_l ;

  {
//#line  41
  __vericon_ptr_elm = & elm;
//#line  41
  __vericon_ptr_l = & l;
//#line  44
  make_list(__vericon_ptr_l, 5U);
//#line  45
  search(__vericon_ptr_elm, *__vericon_ptr_l);
//#line  46
  free((void *)*__vericon_ptr_elm);
//#line  47
  free_list(*__vericon_ptr_l);
  {
//#line  48
  *__fst_ret = 0;
//#line  48
  return;
  }
}
}