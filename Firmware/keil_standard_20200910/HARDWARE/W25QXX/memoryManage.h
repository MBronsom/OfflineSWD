/**
 *  自定义文件管理系统，日后必须上常用的文件系统，就像显示要用emWin一样
 * 
 */


#ifndef __MEMORYMANAGE_H
#define __MEMORYMANAGE_H			    


#include "stm32f10x.h"

#pragma pack (1)  // 注意：由于对齐问题，必须使用这个方式，否则下面的两个结构上不对应！！
                  // 关于对齐，日后进行下次开发的时候，务必重新设计报文协议，这样的操作会使效率降低！
typedef union
{
	struct
	{
		u8 Buf[256];
		u8 DownloaderNumbers[256];  // 用来指示程序的下载次数，不被频繁的改写！
		u16 pWrite;  // 用来写的指针
		u16 pRead;   // 用来读的指针
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
		char FileName[34];  // 文件名称
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
#pragma pack ( )  // 取消对齐方式

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
		u8 toUploaderTail; // 文件尾巴
	};
}_UpStatusInfo ;
#pragma pack () // 取消对齐方式

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
		u16 fileNO;        // 文件编号
		u16 ChipTypePrefix ;
		u16 ChipTypeSuffix;
		char fileName[20]; // 对于编码，注意有汉字之分，暂时只允许20个字符，其它日后再定！
		u8 Res[8];
		u8 sum;
		u8 toUploaderTail; // 文件尾巴
	};
}_UpFileInfo;
#pragma pack ( )

typedef struct
{
	u32 RemainingSize;  // 单位：字节，对于8M的存储器来说，这个最大值为：0x7F0000（64*127KB）
	u32 RemainingContinuousMaxSize;  // 连续的存储中的最大值，对于8M的存储器来说，这个最大值为：0x7F0000。如果和上面两个相等，则认为是最优化的数据！
	u16 FlieNumbers;
}_W25QFlashInformation;

typedef struct
{
	u8 databuf4K[0x1000];  // 擦除最小的块为4KB，局部变量不能定义大的数组，所以使用这个用于用到4K扇区的管理使用。注意：不要被打断，否则会出错！
	u32 pWrite;
	u32 pRead;
}_W25Q4KBuf;  // 定义一个4K缓冲区！

//typedef union // 用结构体寻址
//{
//	u8 Index256[256][256];	// 可正常读出，不可写入
//	u8 Index512[128][512];
//	u8 Index4K[16][4096];
//}_W25QXX_FlieDensity;

//#define W25QXX_FlieDensityAddress(x)    (u32)(((_W25QXX_FlieDensity *) (x*0xFFFF)))  // 这个的使用只是为了找到地址，而不是为了地址中的数据，否则访问的为MCU片内的Flash！所以使用强制转换
extern _W25QFlashInformation W25QFlashInformation;
extern _UpStatusInfo UpStatusInfo;
extern _UpFileInfo UpFileInfo;
extern _FileInformation filex;  // 定义一个文件
extern _FileInformation fileTemp; // 这个文件是临时的应用！其实上面的文件有时候也是临时的应用的，但是有时候却是不够的！ 
extern _W25Q4KBuf W25Q4KBuf;

u32 W25QXX_FlieDensityAddress( u8 block,u8 sector,u16 page );
u16 GetSumOf16Bit( u8* DataBuf,u32 Length );
u8 deleteFile( u8 fileType,u16 fileIndex );
void W25QFlashProgramFileInformation( void );
void arrangeTheProgramFile( void );
void memoryManageInit( void );

#endif















