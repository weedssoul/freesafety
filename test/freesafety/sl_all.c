struct list {
  struct list *next;
};

struct list *malloc(unsigned int size);
void free(void *p);
void assert_null(void *p);

struct list *append(struct list *l1, struct list *l2) {
  struct list *ret;

  if (l1 == '\0') {
    assert_null(l1);
    return l2;
  } else {
    l1->next = append(l1->next, l2);
    return l1;
  }
}

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

struct list *merge(struct list *l1, struct list *l2) {
    int x;

    if (l1 == '\0') {
        assert_null(l1);
        return l2;
    } else if (l2 == '\0') {
        assert_null(l2);
        return(l1);
    } else if (x) {
        l1->next = merge(l1->next, l2);
        return l1;
    } else {
        l2->next = merge(l1, l2->next);
        return l2;
    }
}

struct list *search(struct list *l)
{
    int x;
    struct list *p;

    while (1) {
        if (x) {
            p = l->next;
            l->next = p->next;
            break;
        }
    }
    return p;
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
        // free(l);
    }
}

int main()
{
    struct list *l1;
    struct list *l2;
    struct list *l3;

    l1 = append(make_list(5), make_list(5));
    l2 = reverse(l1);
    l3 = merge(l2, make_list(5));

    free_list(l3);

    return 0;
}
