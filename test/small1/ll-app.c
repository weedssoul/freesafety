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

struct ll_node *malloc(unsigned int size);
void free(void *p);
extern void assert_null(void *p);
extern int random(void);

struct ll_node *append(struct ll_node *l1, struct ll_node *l2) {
  struct ll_node *ret;
  if (l1 == NULL) {
    assert_null(l1);
    return l2;
  } else {
    l1->next = append(l1->next, l2);
    return l1;
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
  assert_null(p);
}

int main(int argc, char **argv)
{
  free_list(append(make_list(5), make_list(5)));
  return 0;
}
