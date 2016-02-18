void *malloc(unsigned int size);
void free(void*);
void assert(char*, char*);

int main()
{
  int *x;
  int *y;
  int p;
  int q;

  /*
    env:
      temp: void ref (ovar2)
      x: int ref (ovar0)
      y: int ref (ovar1)
      p: int
      q: int
    constr:
      []
  */
  x = malloc(sizeof(int));

  /*
    env:
      temp: void ref (ovar3)
      x: int ref (ovar0)
      y; int ref (ovar1)
      p: int
      q: int
    constr:
      [ovar2 = 0.0;
       ovar3 = 1.0]
  */

  /*
    env:
      temp: void ref (ovar4)
      x: int ref (ovar5)
      y: int ref (ovar1)
      p: int
      q: int
    constr:
      [ovar0 = 0.0;
       ovar3 = ovar4 + ovar5]
  */
  y = x;
  /*
    env:
      temp: void ref (ovar4)
      x: int ref (ovar6)
      y: int ref (ovar7)
      p: int
      q: int
    constr:
      [ovar1 = 0.0;
       ovar5 = ovar6 + ovar7]
  */
  p = *x;
  /*
    env: same
    constr:
      [0.0 < ovar6]
  */
  q = *y;
  /*
    env: same
    constr:
      [0.0 < ovar7]
  */
//  assert(x,y);
  /*
    env:
      temp: void ref (ovar4)
      x: int ref (ovar8)
      y: int ref (ovar9)
      p: int
      q: int
    constr:
      [ovar6 + ovar7 = ovar8 + ovar9]
  */
  free(x);
  /*
    env:
      temp: void ref (ovar4)
      x: int ref (ovar10)
      y: int ref (ovar9)
      p: int
      q: int
    constr:
      [ovar8 = 1.0;
       ovar10 = 0.0]
  */

  return 0;

  /*
    env: same
    constr:
      [ovar4 = 0.0;
       ovar10 = 0.0;
       ovar9 = 0.0]
  */
}
