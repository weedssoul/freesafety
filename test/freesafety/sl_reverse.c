struct list {
  struct list *next;
};

struct list *malloc(unsigned int size);
void free(void *p);
void assert_null(void *p);

struct list *reverse(struct list *l) {
  struct list *temp;
  struct list *prev;
  struct list *next;

  temp = l;
  assert_null(prev);

  while (temp != '\0') {
      next = temp->next;
      temp->next = prev;
      prev = temp;
      temp = next;
  }

  assert_null(temp);

  return prev;
}

struct list *make_list(unsigned int n) {
    struct list *ret;

    if (n == 0) {
        assert_null (ret);
        return ret;
    } else {
        ret = malloc(sizeof (struct list));
        ret->next = make_list(n-1);
        return ret;
    }
}

void free_list(struct list *l) {
    struct list *p;

    if (l == '\0') {
        assert_null(l);
        return;
    } else {
        p = l->next;
        free_list(p);
        free(l);
    }
}

int main()
{
    struct list *l;

    l = reverse(make_list(5));
    free_list(l);

    return 0;
}
