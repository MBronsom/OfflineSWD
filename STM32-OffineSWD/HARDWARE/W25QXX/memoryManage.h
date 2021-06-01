/**
 *  �Զ����ļ�����ϵͳ���պ�����ϳ��õ��ļ�ϵͳ��������ʾҪ��emWinһ��
 * 
 */


#ifndef __MEMORYMANAGE_H
#define __MEMORYMANAGE_H			    


#include "stm32f10x.h"

#pragma pack (1)  // ע�⣺���ڶ������⣬����ʹ�������ʽ����������������ṹ�ϲ���Ӧ����
                  // ���ڶ��룬�պ�����´ο�����ʱ�����������Ʊ���Э�飬�����Ĳ�����ʹЧ�ʽ��ͣ�
typedef union
{
	struct
	{
		u8 Buf[256];
		u8 DownloaderNumbers[256];  // ����ָʾ��������ش���������Ƶ���ĸ�д��
		u16 pWrite;  // ����д��ָ��
		u16 pRead;   // ��������ָ��
	};
	struct
	{
		u8 Status;
		u16 ChipTypePrefix;
		u16 ChipTypeSuffix;
		u32 ProgramStartAddress;
		u32 ProgramSaveAtW25QAddress;
		u32 ProgramSize;
		u8 ProgramResetWay;
		u8 ChipEraseWay;
		u8 FlashCheckWay;
		u32 ProgrammeTimes;
		u16 SumValueOfProgram;
		u16 SumValueOfProgramAESSecure;
		u32 AlreadyProgrammedCarrayBit;
		u8 SWDLowSpeedEnable;
		u8 Res5;
		char FileName[34];  // �ļ�����
		u8 OptionBytesFunction;
		u8 OptionByte[20];
		
		u8 STM8_CFG;
		u32 ProgramStartAddress_STM8;
    u16 SumValueOfProgram_STM8;
    u16 SumValueOfProgramAESSecure_STM8;
		u32 ProgramSize_STM8;
		
		u8 RollingCodeFunction;
		u32 RollingCodeStartAddress;
		u32 RollingCodeStartValue;
		u32 RollingCodeStepValue;
		u8 RollingCodeEndianMode;
		u8 Res6;
		u8 Res7;
		u8 Res8;
		u8 Res9[9];
		u8 Res10[127];
		u8 SumValue;
	};
}_FileInformation;
#pragma pack ( )  // ȡ�����뷽ʽ

#pragma pack (1)
typedef union
{
	struct
	{
		u8 Buf[40];
	};
	struct
	{
		u8 toUploadHeard0; // 0xAA
		u8 toUploadHeard1; // 0x55
		u8 functionCode;   // 0x01
		u8 functionStatus; // 0x01
		u16 fileNumbers;
		u32 RemainingSize;
		u32 RemainingContinuousMaxSize;
		u8 radomCode[8];
		u16 bootloaderVersion;
		u16 APPVersion;
		u16 HardWareVersion;
		u8 SWDFunction;
		u8 SWINFunction;
		u8 Res[8];
		u8 sum;
		u8 toUploaderTail; // �ļ�β��
	};
}_UpStatusInfo ;
#pragma pack () // ȡ�����뷽ʽ

#pragma pack (1)
typedef union
{
	struct
	{
		u8 Buf[40];
	};
	struct
	{
		u8 toUploadHeard0; // 0xAA
		u8 toUploadHeard1; // 0x55
		u8 functionCode;   // 0x01
		u8 functionFileInfo;//0x02
		u16 fileNO;        // �ļ����
		u16 ChipTypePrefix ;
		u16 ChipTypeSuffix;
		char fileName[20]; // ���ڱ��룬ע���к���֮�֣���ʱֻ����20���ַ��������պ��ٶ���
		u8 Res[8];
		u8 sum;
		u8 toUploaderTail; // �ļ�β��
	};
}_UpFileInfo;
#pragma pack ( )

typedef struct
{
	u32 RemainingSize;  // ��λ���ֽڣ�����8M�Ĵ洢����˵��������ֵΪ��0x7F0000��64*127KB��
	u32 RemainingContinuousMaxSize;  // �����Ĵ洢�е����ֵ������8M�Ĵ洢����˵��������ֵΪ��0x7F0000�����������������ȣ�����Ϊ�����Ż������ݣ�
	u16 FlieNumbers;
}_W25QFlashInformation;

typedef struct
{
	u8 databuf4K[0x1000];  // ������С�Ŀ�Ϊ4KB���ֲ��������ܶ��������飬����ʹ����������õ�4K�����Ĺ���ʹ�á�ע�⣺��Ҫ����ϣ���������
	u32 pWrite;
	u32 pRead;
}_W25Q4KBuf;  // ����һ��4K��������

//typedef union // �ýṹ��Ѱַ
//{
//	u8 Index256[256][256];	// ����������������д��
//	u8 Index512[128][512];
//	u8 Index4K[16][4096];
//}_W25QXX_FlieDensity;

//#define W25QXX_FlieDensityAddress(x)    (u32)(((_W25QXX_FlieDensity *) (x*0xFFFF)))  // �����ʹ��ֻ��Ϊ���ҵ���ַ��������Ϊ�˵�ַ�е����ݣ�������ʵ�ΪMCUƬ�ڵ�Flash������ʹ��ǿ��ת��
extern _W25QFlashInformation W25QFlashInformation;
extern _UpStatusInfo UpStatusInfo;
extern _UpFileInfo UpFileInfo;
extern _FileInformation filex;  // ����һ���ļ�
extern _FileInformation fileTemp; // ����ļ�����ʱ��Ӧ�ã���ʵ������ļ���ʱ��Ҳ����ʱ��Ӧ�õģ�������ʱ��ȴ�ǲ����ģ� 
extern _W25Q4KBuf W25Q4KBuf;

u32 W25QXX_FlieDensityAddress( u8 block,u8 sector,u16 page );
u16 GetSumOf16Bit( u8* DataBuf,u32 Length );
u8 deleteFile( u8 fileType,u16 fileIndex );
void W25QFlashProgramFileInformation( void );
void arrangeTheProgramFile( void );
void memoryManageInit( void );

#endif















