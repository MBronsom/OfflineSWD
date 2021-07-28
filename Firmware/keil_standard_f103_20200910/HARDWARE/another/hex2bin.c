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
 
char HexFormatUncode(unsigned char *File)
{
	  FIL fp_hex,fp_bin;
    u8 a,b = 0;
	  char buff[64] = "" ;

    unsigned short offset = 0;  //0~65535
    unsigned char type = 0;
    unsigned char i = 0;
	  f_open(&fp_hex, (const TCHAR*)File,FA_READ);
	  f_open(&fp_bin, (const TCHAR*)"write.bin",FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		u8 WaitTips[] = "...";
		OLED_ShowString(45,2,(u8*)"INVER",1,1);
    while(f_gets(buff,64,&fp_hex)!=NULL){
		a++;
		if(a==100){
			OLED_ShowChar(75+b*6,2,WaitTips[b],1);
			b++;
			if(b>3){ 
				b=0;
				OLED_ShowString(75,2,(u8*)"     ",1,1);
			}
			a=0;
		}
 
    if(buff[0] != ':') return 0;
    else
    {
        offset=Char2toByte(&buff[3])*256+Char2toByte(&buff[5]);
        type=Char2toByte(&buff[7]);
        if(type==0)
        {
            f_lseek(&fp_bin,offset);
            for(i=0; i<strlen(buff); i++)
                f_putc(Char2toByte(&buff[9+2*i]), &fp_bin);
        }

    }
	}
	OLED_ShowString(45,2,(u8*)"          ",1,1);
	f_close(&fp_bin);
	f_close(&fp_hex);
	return 1;
}
