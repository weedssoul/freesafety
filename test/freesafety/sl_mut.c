struct list{
    struct list *next;
    int e;
};

struct list *malloc(unsigned int size);
void free(void*);
void free_alllist_odd(struct list*);
void free_alllist_even(struct list*);
void assert_null(void*);


void free_alllist_even (struct list *l) {
    struct list *p;

    if (l == '\0') {
        assert_null(l);
        return;
    } else {
        p = l->next;
        free_alllist_odd(p);
        free(l);
    }
}

void free_alllist_odd (struct list *l) {
    struct list *p;

    if (l == '\0') {
        assert_null(l);
        return;
    } else {
        p = l->next;
        free_alllist_even(p);
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

    l = make_list(3);
    free_alllist_even (l);

    return 0;
}
