char *malloc(unsigned int size);
void free(char *p);
void _fst_cast(char *p);

int gx;
int *gy = &gx;

int main(int argc, char **argv)
{
  char **z;
  char *y;

  /*
    z : S ref(0, 0) ref(0, 0)
    y : S ref(0, 0)
   */
  y = malloc(sizeof(char));
  /*
    z : S ref(0, 0) ref(0, 0)
    y : S ref(1, 1)
   */
  z = &y;
  /*
    z : S ref(1, 1) ref(0, 1)
    y : S ref(0, 0)
   */
  // free(*z);
  /*
    z : S ref(0, 0) ref(0, 1)
    y : S ref(0, 0)
   */

  return 0;
}
