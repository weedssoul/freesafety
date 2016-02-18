
struct node *malloc(unsigned int size);
void free(struct node *p);

struct node {
  struct node *next;
};

/*
struct node *make_list(unsigned int n)
{
  struct node *head;
  unsigned int i;
  struct node *tmpnode;

  for (i = 0; i < n; ++i) {
    tmpnode = malloc(sizeof(struct node));
    tmpnode->next = head;
    head = tmpnode;
  }

  return head;
}

/*
void free_list(struct node *head) {
  struct node *tmp;

  while (head != '\0') {
    tmp = head->next;
    free(head);
    head = tmp;
  }
}
*/

/*
int main(int argc, char **argv)
{
  free_list(make_list(5));
  return 0;
}

*/

int main(int argc, char **argv)
{
  struct node p;
  p.next = malloc(sizeof(struct node));
  free(p.next);
  return 0;
}
