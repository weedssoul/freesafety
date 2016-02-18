# 1 "test/small1/dl-delete.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 170 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "test/small1/dl-delete.c" 2

# 1 "test/small1/freesafety_test.h" 1



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
extern int random(void);
# 3 "test/small1/dl-delete.c" 2

struct dl_node *delete(struct dl_node *l) {
  struct dl_node *ret = '\0';





  if (random()) {




    ret = l->next;




    l->next = ret->next;




    l->next->prev = l;







    return ret;
  } else {





    l->next->prev = l;




    ret = delete(l->next);







    return ret;
  }
}

void free_list(struct dl_node *list) {
  struct dl_node *p = '\0';
  struct dl_node *tmp = '\0';




  p = list->next;



  while(p != '\0') {



    tmp = p->next;



    free(p);



    p = tmp;



  }
  _fst_assert_eq(p, tmp);



  _fst_assume_null(p);
  _fst_assert_eq(p, tmp);



  p = list->prev;



  while(p != '\0') {



    tmp = p->prev;



    free(p);



    p = tmp;



  }
  _fst_assert_eq(p, tmp);



  _fst_assume_null(p);
  _fst_assert_eq(p, tmp);



  free(list);



  return;
}

struct dl_node *make_list(unsigned int n) {
  struct dl_node *ret = '\0';
  struct dl_node *tmp = '\0';


  while (n > 0) {


    tmp = malloc(sizeof(struct dl_node));


    tmp->next = ret;


    ret->prev = tmp;


    _fst_assert_eq(tmp->next, ret);
    ret = tmp;


  }
  _fst_assert_eq(ret, tmp);


  ret->prev = '\0';


  return ret;
}

int main(int argc, char **argv)
{
  struct dl_node *l;
  struct dl_node *elm;
  l = make_list(5);
  elm = delete(l);
  free(elm);

  return 0;
}
