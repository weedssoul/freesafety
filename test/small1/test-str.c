extern void *malloc(unsigned int size);
extern void free(void *p);
extern void _fst_assume_null(void *p);
extern void _fst_assert_eq(void *p1, void *p2);
extern void _fst_assert_sameregion(void *p1, void *p2);
extern void _fst_assert_empty(void *p);
extern void _fst_assert_writecap(void *p, double w);
extern void _fst_assert_freeob(void *p, double w);

typedef int ptrdiff_t;
typedef unsigned int size_t;
typedef char reg_char;
typedef unsigned char byte;
typedef unsigned chartype;
extern char *__strerror_r (int __errnum, char *__buf, size_t __buflen);
extern char * strchr (const char *s, int c_in);

enum
  {
    __LC_CTYPE = 0,
    __LC_NUMERIC = 1,
    __LC_TIME = 2,
    __LC_COLLATE = 3,
    __LC_MONETARY = 4,
    __LC_MESSAGES = 5,
    __LC_ALL = 6,
    __LC_PAPER = 7,
    __LC_NAME = 8,
    __LC_ADDRESS = 9,
    __LC_TELEPHONE = 10,
    __LC_MEASUREMENT = 11,
    __LC_IDENTIFICATION = 12
  };
extern int errno;
static char *buf;
extern const char _libc_intl_domainname[];
extern char *dcgettext (__const char *__domainname,
			__const char *__msgid, int __category)
  __attribute__ ((__nothrow__)) __attribute__ ((__format_arg__ (2)));
extern char *__dcgettext (__const char *__domainname,
			  __const char *__msgid, int __category)
  __attribute__ ((__nothrow__)) __attribute__ ((__format_arg__ (2)));
extern void *__rawmemchr (__const void *__s, int __c)
     __attribute__ ((__pure__));

char *
strcat (dest, src)
     char *dest;
     const char *src;
{
  char *s1 = dest;
  const char *s2 = src;
  char c;
  char tmp;
  do {
    c = *s1;
    s1 = s1 + 1;
  } while (c != '\0');

  s1 -= 2;

  do
    {
      // [kohei:rewrite] c = *s2++;
      c = *s2;
      s2++;
      *++s1 = c;
    }
  while (c != '\0');
  _fst_assert_sameregion(src, s1);
  _fst_assert_sameregion(dest, s2);
  return dest;
}

int
strcmp (p1, p2)
     const char *p1;
     const char *p2;
{
  register const unsigned char *s1 = (const unsigned char *) p1;
  register const unsigned char *s2 = (const unsigned char *) p2;
  unsigned char c1, c2;
  
  do
    {
      // [kohei:rewrite]
      // c1 = (unsigned char) *s1++;
      // c2 = (unsigned char) *s2++;
      c1 = (unsigned char) *s1;
      s1++;
      c2 = (unsigned char) *s2;
      s2++;
      if (c1 == '\0') {
	_fst_assert_sameregion(p1, s1);
	_fst_assert_sameregion(p2, s2);
	return c1 - c2;
      }
    }
  while (c1 == c2);
  _fst_assert_sameregion(p1, s1);
  _fst_assert_sameregion(p2, s2);
  return c1 - c2;
}

char *
strcpy (dest, src)
     char *dest;
     const char *src;
{
  // src : S ref(1, 1)
  // dest : S ref(1, 1);
  char c;
  char * s = (char *) (src);
  const ptrdiff_t off = (dest) - s - 1;
  size_t n;
  char *tmp;

  // src : S ref(1, 0)
  // dest : S ref(1, 1);
  // s : S ref(0, 1)
  do
    {
      // [kohei:rewrite]
      c = *s;
      s++;
      s[off] = c;
    }
  while (c != '\0');

  // src : S ref(1, 0)
  // dest : S ref(1, 1);
  // s : S ref(0, 1)

  n = s - src;
  (void) (src + n);
  (void) (dest + n);
  _fst_assert_sameregion(src, s);
  return dest;
}

size_t
strlen (str)
const char *str;
{
  const char *char_ptr;
  const unsigned long int *longword_ptr;
  unsigned long int longword, magic_bits, himagic, lomagic;
  char *tmp;
  size_t ret;


  for (char_ptr = str; ((unsigned long int) char_ptr
			& (sizeof (longword) - 1)) != 0;
       ++char_ptr)
    if (*char_ptr == '\0') {
      _fst_assert_sameregion(str, char_ptr);
      return char_ptr - str;
    }



  longword_ptr = (unsigned long int *) char_ptr;
  magic_bits = 0x7efefeffL;
  himagic = 0x80808080L;
  lomagic = 0x01010101L;
  if (sizeof (longword) > 4)
    {


      magic_bits = ((0x7efefefeL << 16) << 16) | 0xfefefeffL;
      himagic = ((himagic << 16) << 16) | himagic;
      lomagic = ((lomagic << 16) << 16) | lomagic;
    }
  if (sizeof (longword) > 8)
    abort ();




  for (;;)
    {
      longword = *longword_ptr;
      longword_ptr++;
      // longword = *longword_ptr++;

      if (
	  ((longword - lomagic) & himagic)

	  != 0)
	{



	  const char *cp = (const char *) (longword_ptr - 1);

	  if (cp[0] == 0) {
	    _fst_assert_sameregion(str, longword_ptr);
	    return cp - str;
	  }
	  if (cp[1] == 0) {
	    _fst_assert_sameregion(str, longword_ptr);
	    return cp - str + 1;
	  }
	  if (cp[2] == 0) {
	    _fst_assert_sameregion(str, longword_ptr);
	    return cp - str + 2;
	  }
	  if (cp[3] == 0) {
	    _fst_assert_sameregion(str, longword_ptr);
	    return cp - str + 3;
	  }
	  if (sizeof (longword) > 4)
	    {
	      if (cp[4] == 0) {
		_fst_assert_sameregion(str, longword_ptr);
		return cp - str + 4;
	      }
	      if (cp[5] == 0) {
		_fst_assert_sameregion(str, longword_ptr);
		return cp - str + 5;
	      }
	      if (cp[6] == 0) {
		_fst_assert_sameregion(str, longword_ptr);
		return cp - str + 6;
	      if (cp[7] == 0)
		_fst_assert_sameregion(str, longword_ptr);
		return cp - str + 7;
	      }
	    }
	}
    }
}

char *
strncpy (s1, s2, n)
     char *s1;
     const char *s2;
     size_t n;
{
  char c;
  char *s = s1;

  --s1;
  
  if (n >= 4)
    {
      size_t n4 = n >> 2;

      for (;;)
	{
	  c = *s2;
	  s2++;
	  *++s1 = c;
	  if (c == '\0')
	    break;
	  c = *s2;
	  s2++;
	  *++s1 = c;
	  if (c == '\0')
	    break;
	  c = *s2;
	  s2++;
	  *++s1 = c;
	  if (c == '\0')
	    break;
	  c = *s2;
	  s2++;
	  *++s1 = c;
	  if (c == '\0')
	    break;
	  if (--n4 == 0)
	    goto last_chars;
	}
      n = n - (s1 - s) - 1;
      if (n == 0) {
	_fst_assert_sameregion(s1, s);
	return s;
      }
      goto zero_fill;
    }

 last_chars:
  n &= 3;
  if (n == 0) {
    _fst_assert_sameregion(s1, s);
    return s;
  }

  do
    {
      c = *s2;
      s2++;
      *++s1 = c;
      if (--n == 0) {
	_fst_assert_sameregion(s1, s);
	return s;
      }
    }
  while (c != '\0');

 zero_fill:
  do
    *++s1 = '\0';
  while (--n > 0);
  
  _fst_assert_sameregion(s1, s);
  return s;
}

char *
strncat (s1, s2, n)
     char *s1;
     const char *s2;
     size_t n;
{
  reg_char c;
  char *s = s1;


  do {
    c = *s1;
    s1++;
  } while (c != '\0');

  s1 -= 2;

  if (n >= 4)
    {
      size_t n4 = n >> 2;
      do
	{
	  c = *s2;
	  s2++;
	  *++s1 = c;
	  if (c == '\0') {
	    _fst_assert_sameregion(s1, s);
	    return s;
	  }
	  c = *s2;
	  s2++;
	  *++s1 = c;
	  if (c == '\0') {
	    _fst_assert_sameregion(s1, s);
	    return s;
	  }
	  c = *s2;
	  s2++;
	  *++s1 = c;
	  if (c == '\0') {
	    _fst_assert_sameregion(s1, s);
	    return s;
	  }
	  c = *s2;
	  s2++;
	  *++s1 = c;
	  if (c == '\0') {
	    _fst_assert_sameregion(s1, s);
	    return s;
	  }
	} while (--n4 > 0);
      n &= 3;
    }

  while (n > 0)
    {
      c = *s2;
      s2++;
      *++s1 = c;
      if (c == '\0') {
	_fst_assert_sameregion(s1, s);
	return s;
      }
      n--;
    }

  if (c != '\0')
    *++s1 = '\0';
  _fst_assert_sameregion(s1, s);
  return s;
}

int
strncmp (s1, s2, n)
     const char *s1;
     const char *s2;
     size_t n;
{
  unsigned char c1 = '\0';
  unsigned char c2 = '\0';

  if (n >= 4)
    {
      size_t n4 = n >> 2;
      do
	{
	  c1 = (unsigned char) *s1;
	  s1++;
	  c2 = (unsigned char) *s2;
	  s2++;
	  if (c1 == '\0' || c1 != c2)
	    return c1 - c2;
	  c1 = (unsigned char) *s1;
	  s1++;
	  c2 = (unsigned char) *s2;
	  s2++;
	  if (c1 == '\0' || c1 != c2)
	    return c1 - c2;
	  c1 = (unsigned char) *s1;
	  s1++;
	  c2 = (unsigned char) *s2;
	  s2++;
	  if (c1 == '\0' || c1 != c2)
	    return c1 - c2;
	  c1 = (unsigned char) *s1;
	  s1++;
	  c2 = (unsigned char) *s2;
	  s2++;
	  if (c1 == '\0' || c1 != c2)
	    return c1 - c2;
	} while (--n4 > 0);
      n &= 3;
    }

  while (n > 0)
    {
      c1 = (unsigned char) *s1;
      s1++;
      c2 = (unsigned char) *s2;
      s2++;
      if (c1 == '\0' || c1 != c2)
	return c1 - c2;
      n--;
    }

  return c1 - c2;
}

size_t
strspn (s, accept)
     const char *s;
     const char *accept;
{
  const char *p;
  const char *a;
  size_t count = 0;

  for (p = s; *p != '\0'; ++p)
    {
      for (a = accept; *a != '\0'; ++a)
	if (*p == *a)
	  break;
      if (*a == '\0') {
	_fst_assert_sameregion(s, p);
	_fst_assert_sameregion(accept, a);
	return count;
      }
      else
	++count;
    }

  _fst_assert_sameregion(s, p);
  _fst_assert_sameregion(accept, a);
  return count;
}

void *
__memchr (s, c_in, n)
     const void * s;
     int c_in;
     size_t n;
{
  const unsigned char *char_ptr;
  const unsigned long int *longword_ptr;
  unsigned long int tmp;
  unsigned long int longword, magic_bits, charmask;
  unsigned char c;

  c = (unsigned char) c_in;



  for (char_ptr = (const unsigned char *) s;
       n > 0 && ((unsigned long int) char_ptr
		 & (sizeof (longword) - 1)) != 0;
       --n, ++char_ptr)
    if (*char_ptr == c) {
      _fst_assert_sameregion(s, char_ptr);
      return (void *) char_ptr;
    }




  longword_ptr = (unsigned long int *) char_ptr;

  if (sizeof (longword) != 4 && sizeof (longword) != 8)
    abort ();


  magic_bits = 0x7efefeff;





  charmask = c | (c << 8);
  charmask |= charmask << 16;







  while (n >= sizeof (longword))
    {

    // [kohei:rewrite]
    // longword = *longword_ptr++ ^ charmask;
      tmp = *longword_ptr;
      longword_ptr++;
      longword = tmp ^ charmask;


      if ((((longword + magic_bits)


	    ^ ~longword)




	   & ~magic_bits) != 0)
	{



	  const unsigned char *cp = (const unsigned char *) (longword_ptr - 1);

	  if (cp[0] == c) {
	    _fst_assert_sameregion(s, longword_ptr);
	    return (void *) cp;
	  }
	  if (cp[1] == c) {
	    _fst_assert_sameregion(s, longword_ptr);
	    return (void *) &cp[1];
	  }
	  if (cp[2] == c) {
	    _fst_assert_sameregion(s, longword_ptr);
	    return (void *) &cp[2];
	  }
	  if (cp[3] == c) {
	    _fst_assert_sameregion(s, longword_ptr);
	    return (void *) &cp[3];
	  }

	}

      n -= sizeof (longword);
    }

  char_ptr = (const unsigned char *) longword_ptr;
  while (n-- > 0)
    {
      if (*char_ptr == c) {
	_fst_assert_sameregion(s, char_ptr);
	return (void *) char_ptr;
      }
      else
	++char_ptr;
    }
  _fst_assert_sameregion(s, char_ptr);
  return 0;
}

/*
char *
strchr (s, c_in)
     const char *s;
     int c_in;
{
  const unsigned char *char_ptr;
  const unsigned long int *longword_ptr;
  unsigned long int longword, magic_bits, charmask;
  unsigned char c;
  unsigned long int *tmp;

  c = (unsigned char) c_in;

  

  for (char_ptr = (const unsigned char *) s;
       ((unsigned long int) char_ptr & (sizeof (longword) - 1)) != 0;
       ++char_ptr)
    if (*char_ptr == c) {
      // _fst_assert_sameregion(s, char_ptr);
      return (void *) char_ptr;
    }
    else if (*char_ptr == '\0') {
      _fst_assert_sameregion(s, char_ptr);
      return ((void *)0);
    }




  longword_ptr = (unsigned long int *) char_ptr;
  switch (sizeof (longword))
    {
    case 4: magic_bits = 0x7efefeffL; break;
    case 8: magic_bits = ((0x7efefefeL << 16) << 16) | 0xfefefeffL; break;
    default:
      abort ();
    }


  charmask = c | (c << 8);
  charmask |= charmask << 16;
  if (sizeof (longword) > 4)

    charmask |= (charmask << 16) << 16;
  if (sizeof (longword) > 8)
    abort ();




  for (;;)
    {
      longword = *longword_ptr;
      longword_ptr++;


      if ((((longword + magic_bits)
	    ^ ~longword)
	   & ~magic_bits) != 0 ||
	  ((((longword ^ charmask) + magic_bits) ^ ~(longword ^ charmask))
	   & ~magic_bits) != 0)
	{

	  const unsigned char *cp = (const unsigned char *) (longword_ptr - 1);

	  if (*cp == c) {
	    _fst_assert_sameregion(s, cp);
	    return (char *) cp;
	  }
	  else if (*cp == '\0') {
	    _fst_assert_sameregion(s, cp);
	    return ((void *)0);
	  }
	  if (*++cp == c) {
	    _fst_assert_sameregion(s, cp);
	    return (char *) cp;
	  }
	  else if (*cp == '\0') {
	    _fst_assert_sameregion(s, cp);
	    return ((void *)0);
	  }
	  if (*++cp == c) {
	    _fst_assert_sameregion(s, cp);
	    return (char *) cp;
	  }
	  else if (*cp == '\0') {
	    _fst_assert_sameregion(s, cp);
	    return ((void *)0);
	  }
	  if (*++cp == c) {
	    _fst_assert_sameregion(s, cp);
	    return (char *) cp;
	  }
	  else if (*cp == '\0') {
	    _fst_assert_sameregion(s, cp);
	    return ((void *)0);
	  }
	  if (sizeof (longword) > 4)
	    {
	      if (*++cp == c) {
		_fst_assert_sameregion(s, cp);
		return (char *) cp; // write(cp)
	      }
	      else if (*cp == '\0') {
		_fst_assert_sameregion(s, cp);
		return ((void *)0);
	      }
	      if (*++cp == c) {
		_fst_assert_sameregion(s, cp);
		return (char *) cp;
	      }
	      else if (*cp == '\0') {
		_fst_assert_sameregion(s, cp);
		return ((void *)0);
	      }
	      if (*++cp == c) {
		_fst_assert_sameregion(s, cp); // write(cp)
		return (char *) cp;
	      }
	      else if (*cp == '\0') {
		_fst_assert_sameregion(s, cp); // write(cp)
		return ((void *)0);
	      }
	      if (*++cp == c) {
		_fst_assert_sameregion(s, cp); // write(cp)
		return (char *) cp;
	      }
	      else if (*cp == '\0') {
		_fst_assert_sameregion(s, cp);
		return ((void *)0);
	      }
	    }
	  _fst_assert_sameregion(longword_ptr, cp);
	}
      
    }
  // _fst_assert_sameregion(s, longword_ptr);
  return ((void *)0);
}
*/

size_t
strcspn (s, reject)
     const char *s;
     const char *reject;
{
  size_t count = 0;
  char *tmp;
  char retval;

  while (*s != '\0') {
    tmp = s;
    s++;
    retval = index(reject, *tmp);
    _fst_assert_sameregion(s, tmp);
    if (retval == '\0')
      ++count;
    else
      return count;
  }
  return count;
}

char *
strpbrk (s, accept)
     const char *s;
     const char *accept;
{
  while (*s != '\0')
    {
      const char *a = accept;
      while (*a != '\0')
	if (*a++ == *s) {
	  _fst_assert_sameregion(accept, a);
	  return (char *) s;
	}
      ++s;
    }
  
  return '\0';
}

char *
strrchr (const char *s, int c)
{
  register const char *found, *p;
  c = (unsigned char) c;

  if (c == '\0') {
    return strchr (s, '\0');
  }

  {
    // s : S ref(1, 1)
    // tmps : S ref(0, 0)
    char *tmps = s;
    // s : S ref(1, 0)
    // tmps : S ref(0, 1)
    found = ((void *)0);
    // s : S ref(1, 0)
    // tmps : S ref(0, 1)
    // found : S ref(0, 1)
    while ((p = strchr (tmps, c)) != ((void *)0))
      {
	found = p;
	tmps = p + 1;
      }
    _fst_assert_sameregion(found, p);
    return (char *) found;
  }
}

char *
strstr (phaystack, pneedle)
     const char *phaystack;
     const char *pneedle;
{
  const unsigned char *haystack, *needle;
  chartype b;
  const unsigned char *rneedle;

  haystack = (const unsigned char *) phaystack;
  needle = pneedle;
  if ((b = *needle))
    {
      chartype c;
      haystack--;

      {
	chartype a;
	do 
	  if (!(a = *++haystack)) {
	    _fst_assert_sameregion(pneedle, needle);
	    goto ret0;
	  }
	while (a != b);
      }

      if (!(c = *++needle)) {
	// write(pneedle) = 2541
	_fst_assert_sameregion(pneedle, needle);
	goto foundneedle;
      }
      ++needle;
      goto jin;

      for (;;)
	{
	  {
	    chartype a;
	    if (0)
	    jin:{
		if ((a = *++haystack) == c)
		  goto crest;
	      }
	    else
	      a = *++haystack;
	    do
	      {
		for (; a != b; a = *++haystack)
		  {
		    if (!a) {
		      _fst_assert_sameregion(pneedle, needle);
		      goto ret0;
		    }
		    if ((a = *++haystack) == b) {
		      break;
		    }
		    if (!a) {
		      _fst_assert_sameregion(pneedle, needle);
		      goto ret0;
		    }
		  }
	      }
	    while ((a = *++haystack) != c);
	  }
	crest:
	  {
	    chartype a;
	    {
	      const unsigned char *rhaystack;
	      rhaystack = haystack + 1;
	      haystack--;
	      rneedle = needle;
	      if (*rhaystack == (a = *rneedle))
		do
		  {
		    if (!a) {
		      _fst_assert_sameregion(needle, rneedle);
		      _fst_assert_sameregion(pneedle, needle);
		      _fst_assert_sameregion(haystack, rhaystack);
		      goto foundneedle;
		    }
		    _fst_assert_sameregion(needle, rneedle);
		    needle++;
		    a = *needle;
		    _fst_assert_sameregion(rneedle, needle);
		    if (*++rhaystack != a) {
		      break;
		    }
		    if (!a) {
		      _fst_assert_sameregion(needle, rneedle);
		      _fst_assert_sameregion(pneedle, needle);
		      _fst_assert_sameregion(haystack, rhaystack);
		      goto foundneedle;
		    }
		    _fst_assert_sameregion(needle, rneedle);
		    needle++;
		    a = *needle;
		    _fst_assert_sameregion(rneedle, needle);
		  }
		while (*++rhaystack == a);
	      needle = rneedle;
	      _fst_assert_sameregion(needle, rneedle);
	      _fst_assert_sameregion(haystack, rhaystack);
	    }
	    if (!a) {
	      _fst_assert_sameregion(pneedle, needle);
	      break;
	    }
	  }
	}
    }
  _fst_assert_sameregion(pneedle, needle);
 foundneedle:
  return (char *) haystack;
 ret0:
  return 0;
}


/*
int main(int argc, char **argv)
{
  char *p1, *p2, *p3;
  p1 = (char *)malloc(sizeof("Hello!"));
  p2 = (char *)malloc(sizeof("How are you?"));
  p3 = strstr(p1, p2);
  *p3 = 'd';
  _fst_assert_sameregion(p1, p3);
  free(p1);
  free(p2);
  return 0;
}
*/

/*
static char *olds;
char *
strtok (s, delim)
     char *s;
     const char *delim;
{
  char *token;
  char *tmps = s;

  if (tmps == ((void *)0)) {
    _fst_assume_null(tmps);
    tmps = olds;
  }

  tmps += strspn (tmps, delim);
  if (*tmps == '\0') {
    olds = tmps;
    _fst_assert_sameregion(s, tmps);
    return ((void *)0);
  }

  token = tmps;
  tmps = strpbrk (token, delim);
  if (tmps == ((void *)0)) {
    _fst_assume_null(tmps);
    olds = __rawmemchr (token, '\0');
  } else {
    *tmps = '\0';
    olds = tmps + 1;
  }
  _fst_assert_sameregion(token, olds);
  _fst_assert_sameregion(token, tmps);
  _fst_assert_sameregion(s, tmps);
  return token;
}

size_t
strxfrm (char *dest, const char *src, size_t n)
{
  return __strxfrm_l (dest, src, n, ((__locale_t) (__libc_tsd_LOCALE_data)));
}
*/

/*
  int
  strcoll (s1, s2)
  const char *s1;
  const char *s2;
  {
  return __strcoll_l (s1, s2, ((__locale_t) (__libc_tsd_LOCALE_data)));
  }
*/

/*
void *
memset (dstpp, c, len)
     void *dstpp;
     int c;
     size_t len;
{
  long int dstp = (long int) dstpp;

  if (len >= 8)
    {
      size_t xlen;
      unsigned long int cccc;

      cccc = (unsigned char) c;
      cccc |= cccc << 8;
      cccc |= cccc << 16;
      if ((sizeof(unsigned long int)) > 4)

	cccc |= (cccc << 16) << 16;



      while (dstp % (sizeof(unsigned long int)) != 0)
	{
	  ((byte *) dstp)[0] = c;
	  dstp += 1;
	  len -= 1;
	}


      xlen = len / ((sizeof(unsigned long int)) * 8);
      while (xlen > 0)
	{
	  ((unsigned long int *) dstp)[0] = cccc;
	  ((unsigned long int *) dstp)[1] = cccc;
	  ((unsigned long int *) dstp)[2] = cccc;
	  ((unsigned long int *) dstp)[3] = cccc;
	  ((unsigned long int *) dstp)[4] = cccc;
	  ((unsigned long int *) dstp)[5] = cccc;
	  ((unsigned long int *) dstp)[6] = cccc;
	  ((unsigned long int *) dstp)[7] = cccc;
	  dstp += 8 * (sizeof(unsigned long int));
	  xlen -= 1;
	}
      len %= (sizeof(unsigned long int)) * 8;


      xlen = len / (sizeof(unsigned long int));
      while (xlen > 0)
	{
	  ((unsigned long int *) dstp)[0] = cccc;
	  dstp += (sizeof(unsigned long int));
	  xlen -= 1;
	}
      len %= (sizeof(unsigned long int));
    }


  while (len > 0)
    {
      ((byte *) dstp)[0] = c;
      dstp += 1;
      len -= 1;
    }

  return dstpp;
}

  void *
  memmove (dest, src, len)
  void *dest;
  const void *src;
  size_t len;
  {
  unsigned long int dstp = (long int) dest;
  unsigned long int srcp = (long int) src;



  if (dstp - srcp >= len)
  {



  if (len >= 16)
  {

  len -= (-dstp) % (sizeof(unsigned long int));
  do { size_t __nbytes = ((-dstp) % (sizeof(unsigned long int))); while (__nbytes > 0) { byte __x = ((byte *) srcp)[0]; srcp += 1; __nbytes -= 1; ((byte *) dstp)[0] = __x; dstp += 1; } } while (0);




  ;






  do { if (srcp % (sizeof(unsigned long int)) == 0) _wordcopy_fwd_aligned (dstp, srcp, (len) / (sizeof(unsigned long int))); else _wordcopy_fwd_dest_aligned (dstp, srcp, (len) / (sizeof(unsigned long int))); srcp += (len) & -(sizeof(unsigned long int)); dstp += (len) & -(sizeof(unsigned long int)); (len) = (len) % (sizeof(unsigned long int)); } while (0);


  }


  do { size_t __nbytes = (len); while (__nbytes > 0) { byte __x = ((byte *) srcp)[0]; srcp += 1; __nbytes -= 1; ((byte *) dstp)[0] = __x; dstp += 1; } } while (0);
  }
  else
  {

  srcp += len;
  dstp += len;


  if (len >= 16)
  {

  len -= dstp % (sizeof(unsigned long int));
  do { size_t __nbytes = (dstp % (sizeof(unsigned long int))); while (__nbytes > 0) { byte __x; srcp -= 1; __x = ((byte *) srcp)[0]; dstp -= 1; __nbytes -= 1; ((byte *) dstp)[0] = __x; } } while (0);






  do { if (srcp % (sizeof(unsigned long int)) == 0) _wordcopy_bwd_aligned (dstp, srcp, (len) / (sizeof(unsigned long int))); else _wordcopy_bwd_dest_aligned (dstp, srcp, (len) / (sizeof(unsigned long int))); srcp -= (len) & -(sizeof(unsigned long int)); dstp -= (len) & -(sizeof(unsigned long int)); (len) = (len) % (sizeof(unsigned long int)); } while (0);


  }


  do { size_t __nbytes = (len); while (__nbytes > 0) { byte __x; srcp -= 1; __x = ((byte *) srcp)[0]; dstp -= 1; __nbytes -= 1; ((byte *) dstp)[0] = __x; } } while (0);
  }

  return (dest);
  }
*/


/*
  void *
  memcpy (dstpp, srcpp, len)
  void *dstpp;
  const void *srcpp;
  size_t len;
  {
  unsigned long int dstp = (long int) dstpp;
  unsigned long int srcp = (long int) srcpp;




  if (len >= 16)
  {

  len -= (-dstp) % (sizeof(unsigned long int));
  do { size_t __nbytes = ((-dstp) % (sizeof(unsigned long int))); while (__nbytes > 0) { byte __x = ((byte *) srcp)[0]; srcp += 1; __nbytes -= 1; ((byte *) dstp)[0] = __x; dstp += 1; } } while (0);




  ;





  do { if (srcp % (sizeof(unsigned long int)) == 0) _wordcopy_fwd_aligned (dstp, srcp, (len) / (sizeof(unsigned long int))); else _wordcopy_fwd_dest_aligned (dstp, srcp, (len) / (sizeof(unsigned long int))); srcp += (len) & -(sizeof(unsigned long int)); dstp += (len) & -(sizeof(unsigned long int)); (len) = (len) % (sizeof(unsigned long int)); } while (0);


  }


  do { size_t __nbytes = (len); while (__nbytes > 0) { byte __x = ((byte *) srcp)[0]; srcp += 1; __nbytes -= 1; ((byte *) dstp)[0] = __x; dstp += 1; } } while (0);

  return dstpp;
  }
*/

/*
  int
  memcmp (s1, s2, len)
  const void * s1;
  const void * s2;
  size_t len;
  {
  unsigned long int a0;
  unsigned long int b0;
  long int srcp1 = (long int) s1;
  long int srcp2 = (long int) s2;
  unsigned long int res;

  if (len >= 16)
  {


  while (srcp2 % (sizeof(unsigned long int)) != 0)
  {
  a0 = ((byte *) srcp1)[0];
  b0 = ((byte *) srcp2)[0];
  srcp1 += 1;
  srcp2 += 1;
  res = a0 - b0;
  if (res != 0)
  return res;
  len -= 1;
  }





  if (srcp1 % (sizeof(unsigned long int)) == 0)
  res = memcmp_common_alignment (srcp1, srcp2, len / (sizeof(unsigned long int)));
  else
  res = memcmp_not_common_alignment (srcp1, srcp2, len / (sizeof(unsigned long int)));
  if (res != 0)
  return res;


  srcp1 += len & -(sizeof(unsigned long int));
  srcp2 += len & -(sizeof(unsigned long int));
  len %= (sizeof(unsigned long int));
  }


  while (len != 0)
  {
  a0 = ((byte *) srcp1)[0];
  b0 = ((byte *) srcp2)[0];
  srcp1 += 1;
  srcp2 += 1;
  res = a0 - b0;
  if (res != 0)
  return res;
  len -= 1;
  }

  return 0;
  }

char *
strerror (errnum)
     int errnum;
{
  char *ret = __strerror_r (errnum, ((void *)0), 0);
  int saved_errno;

  // [kohei:rewrite]
  // if (__builtin_expect (ret != ((void *)0), 1))
  if (ret != '\0')
  return ret;
  saved_errno = errno;
  if (buf == '\0') {
  _fst_assume_null(buf);
    buf = malloc (1024);
    }
  errno = (saved_errno);
  if (buf == '\0') {
  _fst_assume_null(buf);
  return __dcgettext (_libc_intl_domainname, "Unknown error", __LC_MESSAGES);
  }
  return __strerror_r (errnum, buf, 1024);
}

*/
