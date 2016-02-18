void *malloc(unsigned int size);
void free(void*);

int main()
{
  char *x;

  x = malloc(1);
  free(x);

  return 0;
}
