#ifndef _UTILITY_H
#define _UTILITY_H



void get_device_id(uint8_t device_id[8]);
unsigned char  CheckSum(unsigned char *uBuff, unsigned char uBuffLen);
unsigned char XOR_Cal(unsigned char *uBuf,unsigned char num);
unsigned char HexToASCII(unsigned char data_hex);
void Ascii2Hex(unsigned char *hexBuf, char *ascBuf, unsigned char hexbufLen);
unsigned int  Ascii2Dec(char *ascBuf, unsigned char num);
void encrypt(uint32_t*v,uint32_t*k);
void decrypt(uint32_t*v,uint32_t*k);

#endif

