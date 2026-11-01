#include <stddef.h>
/* 功能：裸机memset；dst不可空（n=0除外），n字节；返回原dst。不依赖堆或OS。 */
void *memset(void *dst,int value,size_t n)
{
    unsigned char *p=dst;
    for(size_t i=0;i<n;i++) p[i]=(unsigned char)value;
    return dst;
}
/* 功能：复制不重叠的n字节；指针不可空（n=0除外）；返回dst。 */
void *memcpy(void *dst,const void *src,size_t n)
{
    unsigned char *d=dst;const unsigned char *s=src;
    for(size_t i=0;i<n;i++) d[i]=s[i];
    return dst;
}
