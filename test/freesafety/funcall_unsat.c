void *malloc(unsigned int size);
void free(void*);
void assert(char*, char*);

void myfree(void *p) {
    // free(p);
  return;
};

int main()
{
  char *y;
  char *z;
  char p;
  char q;

  y = malloc(1);
  z = y;
  p = *y;
  q = *z;
  assert(y,z);
  myfree(z);

  return 0;
}
