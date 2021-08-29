#include "hex2bin.h"

unsigned char ChartoByte(char c)
{
    if(c-'a'>=0 ) return(c-'a'+10);
    else if(c-'A'>=0 ) return(c-'A'+10);
    else return(c-'0');
}

unsigned char Char2toByte(char* s)
{
    return (ChartoByte(*s)*16+ChartoByte(*(s+1)));
}
