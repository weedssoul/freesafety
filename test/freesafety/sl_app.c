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

    l = append(make_list(5), make_list(5));
    free_list(l);

    return 0;
}
