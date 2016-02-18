
#include "freesafety_test.h"

void insert(struct dl_node *elm, struct dl_node *l) {
  /* a = { next : a ref(1, 1); prev : _ ref(0, 0) }
     b = { next : _ ref(0, 0); prev : b ref(1, 1) } */

  /*
    elm : { next : _ ref(0, 0); prev : _ ref(0, 0) } ref(1, 1)
    l : { next : a ref(1, 1); prev : b ref(1, 1) } ref(1, 1)
  */
  elm->prev = l;
  /*
    elm : { next : _ ref(0, 0); prev : b ref(1, 1) } ref(1, 1)
    l : { next : a ref(1, 1); prev : _ ref(0, 0) } ref(1, 1)
  */
  elm->next = l->next;
  /*
    elm : { next : a ref(1, 1); prev : b ref(1, 1) } ref(1, 1)
    l : { next : _ ref(0, 0); prev : _ ref(0, 0) } ref(1, 1)
  */
  l->next = elm;
  elm->next->prev = elm;
  /*
    elm : { next : _ ref(0, 0); prev : b ref(1, 1) } ref(0, 0)
    l : { next : a ref(1, 1); prev : _ ref(0, 0) } ref(1, 1)
  */
  _fst_assert_eq(l->next, elm);
  /*
    elm : { next : _ ref(0, 0); prev : b ref(1, 1) } ref(0, 0)
    l : { next : a ref(1, 1); prev : _ ref(0, 0) } ref(1, 1)
  */
  _fst_assert_eq(l, elm->prev);

  //_fst_assert_empty(elm);
}

void free_list(struct dl_node *list) {
  struct dl_node *p = '\0';
  struct dl_node *tmp = '\0';

  // list : { prev : a ref(1, 1); next : b ref(1, 1) } ref(1, 1)
  // p : top
  // tmp : top
  p = list->next;
  // list : { prev : a ref(1, 1); next : top ref(0, 0) } ref(1, 1)
  // p : { prev : top ref(0, 0); next : b ref(1, 1) } ref(1, 1)
  // tmp : top
  while(p != '\0') {
    // list : { prev : a ref(1, 1); next : top ref(0, 0) } ref(1, 1)
    // p : { prev : top ref(0, 0); next : b ref(1, 1) } ref(1, 1)
    // tmp : top
    tmp = p->next;
    // list : { prev : a ref(1, 1); next : top ref(0, 0) } ref(1, 1)
    // p : { prev : top ref(0, 0); next : top ref(0, 0) } ref(1, 1)
    // tmp : { prev : top ref(0, 0); next : b ref(1, 1) } ref(1, 1)
    free(p);
    // list : { prev : a ref(1, 1); next : top ref(0, 0) } ref(1, 1)
    // p : { prev : top ref(0, 0); next : top ref(0, 0) } ref(0, 0)
    // tmp : { prev : top ref(0, 0); next : b ref(1, 1) } ref(1, 1)
    p = tmp;
    // list : { prev : a ref(1, 1); next : top ref(0, 0) } ref(1, 1)
    // p : { prev : top ref(0, 0); next : b ref(1, 1) } ref(1, 1)
    // tmp : top
  }
  _fst_assert_eq(p, tmp);
  // list : { prev : a ref(1, 1); next : top ref(0, 0) } ref(1, 1)
  // p : { prev : top ref(0, 0); next : b ref(1, 1) } ref(1, 1)
  // tmp : top
  _fst_assume_null(p);
  _fst_assert_eq(p, tmp);
  // list : { prev : a ref(1, 1); next : top ref(0, 0) } ref(1, 1)
  // p : top
  // tmp : top
  p = list->prev;
  // list : { prev : top ref(0, 0); next : top ref(0, 0) } ref(1, 1)
  // p : { prev : a ref(1, 1); next : top ref(0, 0) } ref(1, 1)
  // tmp : top
  while(p != '\0') {
    // list : { prev : top ref(0, 0); next : top ref(0, 0) } ref(1, 1)
    // p : { prev : a ref(1, 1); next : top ref(0, 0) } ref(1, 1)
    // tmp : top
    tmp = p->prev;
    // list : { prev : top ref(0, 0); next : top ref(0, 0) } ref(1, 1)
    // p : { prev : top ref(0, 0); next : top ref(0, 0) } ref(1, 1)
    // tmp : { prev : a ref(1, 1); next : top ref(0, 0) } ref(1, 1)
    free(p);
    // list : { prev : top ref(0, 0); next : top ref(0, 0) } ref(1, 1)
    // p : { prev : top ref(0, 0); next : top ref(0, 0) } ref(0, 0)
    // tmp : { prev : a ref(1, 1); next : top ref(0, 0) } ref(1, 1)
    p = tmp;
    // list : { prev : top ref(0, 0); next : top ref(0, 0) } ref(1, 1)
    // p : { prev : a ref(1, 1); next : top ref(0, 0) } ref(1, 1)
    // tmp : top
  }
  _fst_assert_eq(p, tmp);
  // list : { prev : top ref(0, 0); next : top ref(0, 0) } ref(1, 1)
  // p : { prev : a ref(1, 1); next : top ref(0, 0) } ref(1, 1)
  // tmp : top
  _fst_assume_null(p);
  _fst_assert_eq(p, tmp);
  // list : { prev : top ref(0, 0); next : top ref(0, 0) } ref(1, 1)
  // p : { prev : top ref(0, 0); next : top ref(0, 0) } ref(0, 0)
  // tmp : top
  free(list);
  // list : { prev : top ref(0, 0); next : top ref(0, 0) } ref(0, 0)
  // p : { prev : top ref(0, 0); next : top ref(0, 0) } ref(0, 0)
  // tmp : top
  return;
}

struct dl_node *make_list(unsigned int n) {
  struct dl_node *ret = '\0';
  struct dl_node *tmp = '\0';
  // tmp : { prev : top ref(0, 0); next : top ref(0, 0) } ref(0, 0)
  // ret : { prev : top ref(0, 0); next : b ref(1, 1) } ref(1, 1)
  while (n > 0) {
    // tmp : { prev : top ref(0, 0); next : top ref(0, 0) } ref(0, 0)
    // ret : { prev : top ref(0, 0); next : b ref(1, 1) } ref(1, 1)
    tmp = malloc(sizeof(struct dl_node));
    // tmp : { prev : top ref(0, 0); next : top ref(0, 0) } ref(1, 1)
    // ret : { prev : top ref(0, 0); next : b ref(1, 1) } ref(1, 1)
    tmp->next = ret;
    // tmp : { prev : top ref(0, 0); next : b ref(1, 1) } ref(1, 1)
    // ret : { prev : top ref(0, 0); next : top ref(0, 0) } ref(0, 0)
    ret->prev = tmp;
    _fst_assert_eq(tmp->next, ret);
    // tmp : { prev : top ref(0, 0); next : b ref(1, 1) } ref(1, 1)
    // ret : { prev : top ref(0, 0); next : top ref(0, 0) } ref(0, 0)
    ret = tmp;
    // tmp : { prev : top ref(0, 0); next : top ref(0, 0) } ref(0, 0)
    // ret : { prev : top ref(0, 0); next : b ref(1, 1) } ref(1, 1)
  }
  // tmp : { prev : top ref(0, 0); next : top ref(0, 0) } ref(0, 0)
  // ret : { prev : top ref(0, 0); next : b ref(1, 1) } ref(1, 1)
  ret->prev = '\0';
  // tmp : { prev : top ref(0, 0); next : top ref(0, 0) } ref(0, 0)
  // ret : { prev : a ref(1, 1); next : b ref(1, 1) } ref(1, 1)
  return ret;
}

int main(int argc, char **argv)
{
  struct dl_node *l;
  struct dl_node *elm;
  l = make_list(5);
  // elm = (struct dl_node *)malloc(sizeof(struct dl_node));
  insert(elm, l);
  free_list(l);
  // free(elm);
  return 0;
}

