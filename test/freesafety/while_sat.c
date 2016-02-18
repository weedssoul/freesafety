struct list{
    struct list *next;
    int e;
};

struct list *malloc(unsigned int size);
void free(void*);
void assert_null(void*);

void free_all_list (struct list *l) {
    struct list *p;
    struct list *tmp;

    p = l;
    while(1) {
        if (p == '\0') {
            break;
        }
        tmp = p->next;
        free(p);
        p = tmp;
    }
    assert_null(p);
}

struct list *make_list(unsigned int n) {
    unsigned int i;
    struct list *ret;
    struct list *tmp;

    assert_null(ret);
    assert_null(tmp);
    i = 0;
    while(1) {
        tmp = malloc(sizeof (struct list));
        tmp->next = ret;
        ret = tmp;
        i = i + 1;
    }

    return ret;
}

int main() {

    struct list *l;

    l = make_list(3);
    // free_all_list(l);

    return 0;
}
