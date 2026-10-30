#include <stddef.h>
/* 功能：计算NUL结尾串长度；s不可空；返回字节数。 */
size_t strlen(const char *s){size_t n=0;while(s[n])n++;return n;}
/* 功能：比较完整字符串；a/b不可空；返回字节差，0表示相同。 */
int strcmp(const char *a,const char *b)
{while(*a&&*a==*b){a++;b++;}return (unsigned char)*a-(unsigned char)*b;}
/* 功能：最多比较n字节；a/b不可空，n可0；返回0相同，非0不同。 */
int strncmp(const char *a,const char *b,size_t n)
{while(n){if(*a!=*b)return (unsigned char)*a-(unsigned char)*b;if(!*a)return 0;a++;b++;n--;}return 0;}
