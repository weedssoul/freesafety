
#include "freesafety_test.h"

struct ll_node *merge(struct ll_node *l1, struct ll_node *l2) {
  if (l1 == NULL) {
    _fst_assume_null(l1);
    return l2;
  } else if (l2 == NULL) {
    _fst_assume_null(l2);
    return(l1);
  } else if (random()) {
    l1->next = merge(l1->next, l2);
    return l1;
  } else {
    l2->next = merge(l1, l2->next);
    return l2;
  }
}

struct ll_node *make_list(unsigned int n)
{
  unsigned int i = 0;
  struct ll_node *ret = NULL;
  struct ll_node *tmp = NULL;
  for (i = 0; i < n; ++i) {
    tmp = (struct ll_node *)malloc(sizeof(struct ll_node));
    tmp->next = ret;
    ret = tmp;
  }
  return ret;
}

void free_list(struct ll_node *l) {
  struct ll_node *p, *tmp;
  p = l;
  while (p != NULL) {
    tmp = p->next;
    free(p);
    p = tmp;
  }
  _fst_assume_null(p);
}

int main(int argc, char **argv)
{
  free_list(merge(make_list(5), make_list(5)));
  return 0;
}
