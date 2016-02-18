
char *malloc(unsigned int size);
void free(char *p);

unsigned int cond;

int main(int argc, char **argv)
{
  char *p = malloc(sizeof(char));
  if (cond) {
    free(p);
  } else {
    ;
  }
}

