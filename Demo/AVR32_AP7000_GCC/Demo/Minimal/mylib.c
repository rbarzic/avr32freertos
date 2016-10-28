
/* From gcc */
/* Public domain. */


#include <stddef.h>
void *
memset (void *dest, int val, size_t len)
{
unsigned char *ptr = dest;
while (len-- > 0)
*ptr++ = val;
return dest;
}



void *
memcpy (void *dest, const void *src, size_t len)
{
char *d = dest;
const char *s = src;
while (len--)
*d++ = *s++;
return dest;
}


/* from oskit */

char *
strncpy(char *to, const char *from, size_t count)
{
register char *ret = to;
while (count > 0) {
count--;
if ((*to++ = *from++) == '\0')
break;
}
while (count > 0) {
count--;
*to++ = '\0';
}
return ret;
}


char *
strcat(char *s, const char *add)
{
register char *ret = s;
while (*s) s++;
while ((*s++ = *add++) != 0);
return ret;
}

char *
strcpy(char *to, const char *from)
{
register char *ret = to;
while ((*to++ = *from++) != 0);
return ret;
}
