struct list {
    struct list *next;
    int n;
};

struct list *malloc(unsigned int size);
void free(struct list*);

int main()
{
    struct list *l;

    l = malloc(sizeof(struct list));
    free(l);

    return 0;
}
