
struct node {
  struct node *prev;
  struct node *next;
};

struct node *malloc(unsigned int size);
void free(struct node *p);
void _fst_assume_null(void *p);

struct node *make_list(unsigned int n) {
  struct node *ret = '\0';
  struct node *tmp = '\0';

  /* a = { next : a ref(1, 1); prev : _ } ref(1, 1)
     b = { next : _; prev : b ref(1, 1) } ref(1, 1) */

  /* ret : { next : a ref(1, 1); prev : _ } ref(1, 1); 
     tmp : _ */
  while (n > 0) {
    /* ret : { next : a ref(1, 1); prev : _ } ref(1, 1); 
       tmp : _ */
    tmp = malloc(sizeof(struct node));
    /* ret : { next : a ref(1, 1); prev : _ } ref(1, 1); 
       tmp : { next : _; prev : _ } ref(1, 1) */
    tmp->next = ret;
    /* ret : { next : _; prev : _} ref(0, 0)
       tmp : { next : a ref(1, 1); prev : _ } ref(1, 1) */
    ret->prev = tmp;
    /* ret : { next : _; prev : _} ref(0, 0)
       tmp : { next : a ref(1, 1); prev : _ } ref(1, 1) */
    ret = tmp;
    /* ret : { next : a ref(1, 1); prev : _ } ref(1, 1) 
       tmp : _ */
  }

  /* ret : { next : a ref(1, 1); prev : _ } ref(1, 1); 
     tmp : _ */
  if (ret == '\0') {
    _fst_assume_null(ret);
    ret = '\0';
  } else {
    ret->prev = '\0';
  }
  /* ret : { next : a ref(1, 1); prev : b ref(1, 1) } ref(1, 1); 
     tmp : _ */
  return ret;
}

void free_list(struct node *list) {
  struct node *p;
  struct node *tmp;

  /* list : { prev : a ref(1, 1); next : b ref(1, 1) } ref(1, 1)
     p : _
     tmp : _ */
  p = list->next;
  /* list : { prev : a ref(1, 1); next : _ } ref(1, 1)
     p : b ref(1, 1)
     tmp : _ */
  while(p != '\0') {
    /* list : { prev : a ref(1, 1); next : _ } ref(1, 1)
       p : { prev : _; next : b ref(1, 1) } ref(1, 1)
       tmp : _ */
    tmp = p->next;
    /* list : { prev : a ref(1, 1); next : _ } ref(1, 1)
       p : { prev : _; next : _ } ref(1, 1)
       tmp : b ref(1, 1) */
    free(p);
    /* list : { prev : a ref(1, 1); next : _ } ref(1, 1)
       p : { prev : _; next : _ } ref(0, 0)
       tmp : b ref(1, 1) */
    p = tmp;
    /* list : { prev : a ref(1, 1); next : _ } ref(1, 1)
       p : { prev : _; next : b ref(1, 1) } ref(1, 1)
       tmp : _ */
  }
  /* [XXX] p should be top here (p == '\0') */
  /* list : { prev : a ref(1 (='o219), 1); next : _ } ref(1, 1)
     p : _
     tmp : _ */
  _fst_assume_null(p);
  p = list->prev;
  /* list : { prev : _; next : _ } ref(1, 1)
     p : { prev : a ref(1, 1); next _ } ref(1, 1)
     tmp : _ */
  while(p != '\0') { /* break --> free(p) = 'o145 */
    /* list : { prev : _; next : _ } ref(1, 1)
       p : { prev : a ref(1, 1); next _ } ref(1 (='o667), 1)
       tmp : _ */
    tmp = p->prev;
    /* list : { prev : _; next : _ } ref(1, 1)
       p : { prev : _; next _ } ref(1 (='o595), 1)
       tmp : a ref(1, 1) */
    free(p);
    /* list : { prev : _; next : _ } ref(1, 1)
       p : { prev : _; next _ } ref(0, 0)
       tmp : a ref(1, 1) */
    p = tmp;
    /* list : { prev : _; next : _ } ref(1, 1)
       p : { prev : a ref(1, 1); next _ } ref(1, 1)
       tmp : _ */
  }
  _fst_assume_null(p);
  /* [XXX] p should be top here */
  /* list : { prev : _; next : _ } ref(1, 1)
     p : _
     tmp : _ */
  free(list);
  /* list : _
     p : _
     tmp : _ */
  return;
}

struct node *take_middle(struct node *p, unsigned int n)
{
  struct node *tmp = p;

  while (n > 0) {
    tmp = tmp->next;
    n--;
  }

  return tmp;
}

int main(int argc, char **argv)
{
  free_list(take_middle(make_list(5), 3));
  return 0;
}
