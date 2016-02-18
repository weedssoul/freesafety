void *malloc(unsigned int size);
void free(void*);

void myfree(void *p) {
    free(p);
};

char *mymalloc(unsigned int s) {
    char *y;

    y = malloc(s);
    return y;
};

void main() {
    char *x;

    x = mymalloc(1);
//    myfree(x);

    return;
}
