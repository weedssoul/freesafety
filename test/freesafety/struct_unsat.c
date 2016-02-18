struct list{
    struct list *next;
    int e;
};

struct list *malloc(unsigned int size);
void free(void*);
void assert_null(void*);
struct list *f(struct list*);
struct list *g(struct list*);
struct list *h(struct list*);

struct list *f(struct list *l) {
    struct list *s;
    struct list *s1;

    s = l->next;
    s1 = g(s);
    free(l);
    return s1;
}

struct list *g(struct list *l) {
    struct list *s;
    struct list *s1;

    s = l->next;
    s1 = h(s);
    free(l);
    return s1;
}

struct list *h(struct list *l) {
    // struct list *s;

    // s = l->next;
    return l;
}

void free_all_list (struct list *l) {
    struct list *p;

    if (l == '\0') {
        assert_null(l);
        return;
    } else {
        p = l->next;
        free_all_list(p);
        free(l);
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

int main() {

    struct list *l;
    struct list *l1;

    l = make_list(3);
    l1 = f(l);
    // free_all_list(l1);

    return 0;
}
