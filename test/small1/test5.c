
struct node *malloc(unsigned int size);
void free(struct node *p);

struct node {
  struct node *next;
};

struct node *make_list(unsigned int n)
{
  unsigned int i;
  struct node *head = '\0';
  struct node *prehead = '\0';
  
  /*
    head : _
    prehead : { next : a } ref(1, 1)
   */
  for (i = 0; i < n; ++i) {
    /*
      head : _
      prehead : { next : a } ref(1, 1)
    */
    head = malloc(sizeof(struct node));
    /* free(head) = 1 */
    /*
      head : { next : _ } ref(1, 1)
      prehead : { next : a } ref(1, 1)
    */
    head->next = prehead;
    /* free(head) = 1 */
    /*
      head : { next : a } ref(1, 1)
      prehead : _
    */
    prehead = head;
    /* free(prehead) + free(head) = 1 */
    /*
      head : _
      prehead : { next : a } ref(1, 1)
    */
  }

  /*
    head : _
    prehead : { next : a } ref(1, 1)
  */
  head = prehead;
  /*
    head : { next : a } ref(1, 1)
    prehead : _
  */
  return head;
}

void free_list(struct node *head) {
  struct node *tmp;

  while (1) {
    if (head == '\0') {
      break;
    }
    tmp = head->next;
    free(head);
    head = tmp;
  }
  return;
}

int main(int argc, char **argv)
{
  free_list(make_list(5));
  return 0;
}
