char *malloc(unsigned int size);
void free(char *p);
int gx;
int *gy = &gx;
int main(int argc, char **argv)
{
    char *s;
    char **t;
    char **z;
    char *y;
    y = malloc(sizeof(char));
    s = y;
    z = &y;
    t = &s;
    /* free(*z); */
    return 0;
}
