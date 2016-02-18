
char *malloc(unsigned int size);
void free(char *p);
char *p;

void myfree(char *l) {
  free(l);
  /* free(p); */
  return;
}

char *mymalloc(unsigned int size) {
  return malloc(size);
  /* p = malloc(size); */
}

int main() {
  /* p = mymalloc(sizeof(char)); */
  char *tmp;
  tmp = mymalloc(sizeof(char));
  myfree(tmp);
  return 0;
}
