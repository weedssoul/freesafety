struct list{
    struct list *next;
    int e;
};

struct list *malloc(unsigned int size);
void free(void*);
void free_alllist(struct list*);

void free_alllist (struct list *l) {
    struct list *x;
    struct list *y;
    int i = 0;

    if (i > 0) {
        x = l -> next;
        free_alllist(x);
        free(l);
    }
    else {
        x = l -> next;
        y = x -> next;
        free_alllist(y);
        free(x);
        free(l);
    }
}



int main() {

    struct list *l1;
    struct list *l2;
    struct list *l3;

    l1 = malloc(sizeof (struct list));
    l2 = malloc(sizeof (struct list));
    l3 = malloc(sizeof (struct list));
    l1->next = l2;
    l2->next = l3;
    free_alllist(l1);

    return 0;
}
