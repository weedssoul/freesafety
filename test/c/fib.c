/* #include <stdlib.h>
#include <stdio.h>
#include <sys/_types.h> */

int atoi(char *v);

int fib(int n)
{
  if (n < 2)
    return 1;
  else
    return fib(n-1) + fib(n-2);
}

int main(int argc, char ** argv)
{
  int n, r;
  if (argc >= 2) n = atoi(argv[1]); else n = 36;
  r = fib(n);
  // printf("fib(%d) = %d\n", n, r);
  return 0;
}
