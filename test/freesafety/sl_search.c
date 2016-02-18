struct list {
  struct list *next;
};

struct list *malloc(unsigned int size);
void free(void *p);
void assert_null(void *p);

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
        free(l);
    }
}

int main()
{
    struct list *elm, *l;

    l = make_list(5);
    elm = search(l);
    free(elm);
    free_list(l);

    return 0;
}
