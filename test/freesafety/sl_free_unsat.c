struct list{
    struct list *next;
    int e;
};

struct list *malloc(unsigned int size);
void free(void*);
void assert_null(void*);

void free_all_list (struct list *l) {
    struct list *p;

    if (l == '\0') {
        assert_null(l);
        return;
    } else {
        p = l->next;
        free_all_list(p);
        // free(l);
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
    free_all_list(l);

    return 0;
}
