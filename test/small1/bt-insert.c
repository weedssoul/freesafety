
#include "freesafety_test.h"

void insert(struct bt_node *elm, struct bt_node *p)
{
  /*
    elm : { left : _ ref(0, 0); right : _ ref(0, 0) } ref(1, 1)
    p : { left : a ref(1, 1); right : a ref(1, 1) } ref(1, 1)
  */
  if (random()) {
    insert(elm, p->left);
  } else if (random()) {
    insert(elm, p->right);
  } else {
    /*
      elm : { left : _ ref(0, 0); right : _ ref(0, 0) } ref(1, 1)
      p : { left : a ref(1, 1); right : a ref(1, 1) } ref(1, 1)
    */
    elm->left = p->left;
    elm->right = NULL;
    /*
      elm : { left : a ref(1, 1); right : a ref(1, 1) } ref(1, 1)
      p : { left : _ ref(0, 0); right : a ref(1, 1) } ref(1, 1)
    */
    p->left = elm;
  }
}

void free_tree(struct bt_node *p)
{
  free_tree(p->right);
  free_tree(p->left);
  free(p);
}

struct bt_node *make_tree() 
{
  struct bt_node *ret = NULL;

  if (random()) {
    return NULL;
  } else {
    ret = (struct bt_node *)malloc(sizeof(struct bt_node));
    ret->right = make_tree();
    ret->left = make_tree();
    return ret;
  }
}

int main(int argc, char **argv)
{
  struct bt_node *tree = make_tree();
  insert((struct bt_node *)malloc(sizeof(struct bt_node)), tree);
  free_tree(tree);
  return 0;
}
