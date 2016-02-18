void *malloc(unsigned int size);
void free(void*);

int main()
{
  char *x;
  char *y;

  x = malloc(1);
  y = x;
//  free(x);

  return 0;
}
