struct list{
    struct list *next;
    int e;
};

struct list *malloc(unsigned int size);
void free(void*);

int main() {
    struct list *l;
    int x = 0;

    while (x > 0) {
        l = malloc(sizeof (struct list));
//        free(l);
    }

    return 0;
}
