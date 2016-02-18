
#include "freesafety_test.h"

struct ll_node *search(struct ll_node *l)
{
  struct ll_node *p = NULL;
  while (1) {
    if (random()) {
      p = l->next;
      l->next = p->next;
      break;
    }
  }
  return p;
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
  struct ll_node *elm, *l;
  l = make_list(5);
  elm = search(l);
  free(elm);
  free_list(l);
  return 0;
}
