/**
 * 依托对于W25Q的操作，来达到存储管理的目的，主要是针对自己制定的协议，但是日后还是要用文件管理系统，这样会更容易上手，本次主要是为了加速开发进度！
 * 
 */

#include "memoryManage.h" 
#include "w25qxx.h" 
#include "ForSampleMain.h"  // 求和函数
#include "STM32_CRC.H"  

_W25QFlashInformation W25QFlashInformation;
_W25Q4KBuf W25Q4KBuf;
_UpStatusInfo UpStatusInfo ;
_UpFileInfo UpFileInfo;
_FileInformation filex;  // 定义一个文件
_FileInformation fileTemp;  // 定义一个临时文件
/**
 *  @B 这个函数专用于程序区的和校验！长度长，返回值为u16形式！
 *
 */
u16 GetSumOf16Bit( u8* DataBuf,u32 Length )
{
	u16 Sum = 0;
	for( u32 i=0; i< Length; i++ )
	{
		Sum += DataBuf[i];
	}
	return Sum;
}

/**
 *  @B 根据W25Q的属性，返回Block[128个]，Sector[128 * 16 = 2048个]，Page[128 * 16 * 16 = 32768个]所在地址值，
 */
 
u32 W25QXX_FlieDensityAddress( u8 block,u8 sector,u16 page )
{
	if( ( block >= 128 ) && ( sector >= 16 ) && ( page >= 256 ) )
	{
		return 0xFFFFFFFF;  // 出错，W25Q64文件不可能这样的。
	}
	u32 address = block * 0x10000 + sector * 0x1000 + page * 0x100;
	return address;
}

/**
 *  @B 找存储程序文件的最大长度，和剩余文件大小
 *
 */
void W25QFlashProgramFileInformation( void )
{
//	u32 maxSize = 0;   // 找出最大值，显然要作为对比！
//	W25QFlashInformation.RemainingSize = 0;
//	W25QFlashInformation.RemainingContinuousMaxSize = 0;
//	W25QXX_Read( W25Q4KBuf.databuf4K , W25QXX_FlieDensityAddress(0,15,0) , 2032); // 读出这个扇区的数据
//	for( u16 i=0;i<2032;i++ )
//	{
//		if( W25Q4KBuf.databuf4K[i] == 0xFF ) // 表示空闲(只有两种状态：一个空闲，一个使用！)
//		{
//			if( i==0 )
//			{
//				W25QFlashInformation.RemainingSize = 4096;
//				W25QFlashInformation.RemainingContinuousMaxSize = 4096;
//			}
//			else
//			{
//				W25QFlashInformation.RemainingSize += 4096;
//				if( W25Q4KBuf.databuf4K[i-1] == 0xFF )
//				{
//					W25QFlashInformation.RemainingContinuousMaxSize += 4096;
//				}
//				else
//				{
//					W25QFlashInformation.RemainingContinuousMaxSize = 4096;
//				}
//			}
//		}
//		else  // 表示不空闲
//		{
//			if( W25QFlashInformation.RemainingContinuousMaxSize > maxSize)
//			{
//				maxSize = W25QFlashInformation.RemainingContinuousMaxSize;  // 则之前找到的文件大小
//			}
//			W25QFlashInformation.RemainingContinuousMaxSize = 0;  // 清零
//		}
//	}
//	if( maxSize > W25QFlashInformation.RemainingContinuousMaxSize )
//		W25QFlashInformation.RemainingContinuousMaxSize = maxSize;  // 
//	
//	{ // 下面为找出文件总数目的函数，最多为120个文件。
//		W25QFlashInformation.FlieNumbers = 0;
//		for( u8 i = 0;i<120;i++ )
//		{
//			W25QXX_Read( filex.Buf,i*512,sizeof(filex.Buf));
//			if( filex.Status != 0x55 )  // 找到第一个不为文件的，则认为结束！
//			{
////				W25QFlashInformation.FlieNumbers = i;  // 这个为文件数量！
////				break;
//			}
//			else
//			{
//				W25QFlashInformation.FlieNumbers ++;  // 文件数量+1
//			}
//		}
//	}

{ // 新文件管理信息系统
	u32 maxSize = 0;   // 找出最大值，显然要作为对比！
	W25QFlashInformation.RemainingSize = 0;
	W25QFlashInformation.RemainingContinuousMaxSize = 0;
	W25QXX_Read( W25Q4KBuf.databuf4K , W25QXX_FlieDensityAddress(15,0,0) , 1792); // 读出这个扇区的数据
	for( u16 i=0;i<1792;i++ )
	{
		if( W25Q4KBuf.databuf4K[i] == 0xFF ) // 表示空闲(只有两种状态：一个空闲，一个使用！)
		{
			if( i==0 )
			{
				W25QFlashInformation.RemainingSize = 4096;
				W25QFlashInformation.RemainingContinuousMaxSize = 4096;
			}
			else
			{
				W25QFlashInformation.RemainingSize += 4096;
				if( W25Q4KBuf.databuf4K[i-1] == 0xFF )
				{
					W25QFlashInformation.RemainingContinuousMaxSize += 4096;
				}
				else
				{
					W25QFlashInformation.RemainingContinuousMaxSize = 4096;
				}
			}
		}
		else  // 表示不空闲
		{
			if( W25QFlashInformation.RemainingContinuousMaxSize > maxSize)
			{
				maxSize = W25QFlashInformation.RemainingContinuousMaxSize;  // 则之前找到的文件大小
			}
			W25QFlashInformation.RemainingContinuousMaxSize = 0;  // 清零
		}
	}
	if( maxSize > W25QFlashInformation.RemainingContinuousMaxSize )
		W25QFlashInformation.RemainingContinuousMaxSize = maxSize;  // 
	
	{ // 下面为找出文件总数目的函数，最多为100个文件。
		W25QFlashInformation.FlieNumbers = 0;
		for( u8 i = 0;i<100;i++ )
		{
			W25QXX_Read( filex.Buf,W25QXX_FlieDensityAddress( i/8,(i%8)*2,0),sizeof(filex.Buf));
			if( filex.Status != 0x55 )  // 找到第一个不为文件的，则认为结束！
			{
			}
			else
			{
				W25QFlashInformation.FlieNumbers ++;  // 文件数量+1
			}
		}
	}
}

}


/**
 *  @B 删除文件操作，参数：删除文件的编号是从：1开始的，不是从0开始，这个要注意！删除文件：是将删除区域的文件清空，同时后面的文件向前移动
       但是没有管理是不是正确的文件或者当前编号有没有文件。删除文件包括指示区不文件存储区！（当然：只要删除文件指示区就可以了！）
 *  // 根据测试发现：操作一次时间需要500ms左右()
 *  // 当：fileType为0x00时，表示删除单个文件，fileIndex表示文件编号。当fileType为0x01，表示全部删除！
 *  @PARA fileIndex = [1-120] // 文件编号从1-120
 *  @WARNING 删除文件，只是移动信息部分！包括已经下载次数！！也就是移动512个字节！！
 */
u8 deleteFile( u8 fileType,u16 fileIndex )
{
//	if( fileType == 0x01 )
//	{
//		for( u8 i=0;i<16;i++ ) // 删除所有的文件！
//		{
//			W25QXX_Erase_Sector(i*0x1000);  // 清除64K文件区，包括指示区！
//		}
//	}
//	else if( fileType == 0x00 )
//	{
//			if( fileIndex < 1 || fileIndex >120 )
//			{
//				return FAIL;  //文件不在范围内！
//			}
//			for( u8 i=0;i<15;i++ )  // 找64KB中前60KB中存储在哪个Sector
//			{
////				if( i == fileIndex / 8)
//				if( ( (fileIndex-1)*512 < ((i+1)*0x1000) ) &&  ( (fileIndex-1)*512 >= (i*0x1000) ) ) // 如果满足，则在第i块！
//				{
//					W25QXX_Read( W25Q4KBuf.databuf4K , i * 0x1000 , 0x1000 ); // 先把所在文件编号的最小擦除块sector读出来，放入databuf4K中！
//					W25QXX_Erase_Sector( i * 0x1000 ); // 擦除这个扇区
//					{ // 操作databuf4K，搬移缓存中数据
//						for( u16 j=0;j<0x1000;j++ )
//						{
//							if( (j>=(((fileIndex-1)%8)*512)) && ( j < (((fileIndex-1)%8)*512+512) ) ) // 操作的在文件范围之内
//							{
//								// 不需要操作
//							}
//							else if( (j<(((fileIndex-1)%8)*512)) )  // 在文件范围之下
//							{
//								 // 保持，不需要操作
//							}
//							else if( j >= (((fileIndex-1)%8)*512+512) )// 在文件范围之上
//							{
//								W25Q4KBuf.databuf4K[j-512] = W25Q4KBuf.databuf4K[j];  // 把前面的512处的字节覆盖
//							}
//						}
//						W25QXX_Write_NoCheck( W25Q4KBuf.databuf4K,i*0x1000,4096-512 );  // 更新这一个Flash，因为已经删除了一个，所以要少512字节，也就是这个文件下，只写7个文件。
//						{ // 把剩下的存储往前面移动！
//							i++; // 后面的页
//							while( i< 15 ) // 仍然在范围之内，需要搬移数据！
//							{
//								W25QXX_Read( W25Q4KBuf.databuf4K , i * 0x1000 , 0x1000 ); // 读出这个扇区的数据
//								W25QXX_Erase_Sector(i * 0x1000); // 擦除这个扇区的数据
//								W25QXX_Write_NoCheck( W25Q4KBuf.databuf4K,i*0x1000 - 512,4096 );  // 更新，此时要从上一个扇区开始更新，可以地址-512就可以，然后再写4K数据
//								i++; // 警告：当把i++写到while( i++ <15){}会出现错误，(感觉就是i+1后面有文件没有跟上后面)难道是先执行判断后就执行了++，不应该是执行完{}后再++么？（应该是判断完就执行了++程序！这个要注意！）
//							};
//						}
//					}
//					break;  // 退出循环
//				}
//				else {};
//			}
//			
//		}
//		else
//		{
//			return FAIL;
//		}
//	// 此时文件结束！
//		return SUCCESS;
		
		{ // 新删除文件操作
			if( fileType == 0x01 )
			{
				for( u8 i=0;i<16;i++ ) // 删除所有的文件:使用了前面的16个Block(1MB的空间)
				{
					W25QXX_Erase_Block(W25QXX_FlieDensityAddress(i,0,0));  // 清除64K文件区，包括指示区！
				}
			}
			else if( fileType == 0x00 )
			{
				if( fileIndex < 1 || fileIndex >100 )
				{
					return FAIL;  //文件不在范围内！
				}
				else
				{  // 删除文件，同时重新排列文件
					{ // 先删除文件
						W25QXX_Erase_Sector( W25QXX_FlieDensityAddress( (fileIndex-1)/8,((fileIndex-1)%8)*2  ,0 ) );  // 删除文件信息
						W25QXX_Erase_Sector( W25QXX_FlieDensityAddress( (fileIndex-1)/8,((fileIndex-1)%8)*2+1,0 ) );  // 删除下载次数指示
					}
					{  // 把后面的文档向前推进，也就是执行，读写擦除-读写擦除操作
						for( u8 i = fileIndex+1,j = 0;i<=100;i++ )  
						{  
							j = i-1;  // 表示上一个文件
							W25QXX_Read( W25Q4KBuf.databuf4K , W25QXX_FlieDensityAddress( (i-1)/8,((i-1)%8)*2  ,0 ) , 0x1000 );  // 读文件指示区
							W25QXX_Write_NoCheck( W25Q4KBuf.databuf4K, W25QXX_FlieDensityAddress( (j-1)/8,((j-1)%8)*2  ,0 ),0x1000 );  // 写到上一个文件指示
							W25QXX_Erase_Sector( W25QXX_FlieDensityAddress( (i-1)/8,((i-1)%8)*2  ,0 ) );  // 删除刚刚的读的这个
							
							W25QXX_Read( W25Q4KBuf.databuf4K , W25QXX_FlieDensityAddress( (i-1)/8,((i-1)%8)*2+1  ,0 ) , 0x1000 );  // 读文件下载次数区
							W25QXX_Write_NoCheck( W25Q4KBuf.databuf4K, W25QXX_FlieDensityAddress( (j-1)/8,((j-1)%8)*2+1  ,0 ),0x1000 );  // 写到上一个文件下载次数区
							W25QXX_Erase_Sector( W25QXX_FlieDensityAddress( (i-1)/8,((i-1)%8)*2+1 ,0 ) );  // 删除刚刚的读的这个
						}
					}
				}
			}
			else
			{
				return FAIL;
			}
			// 此时文件结束！
				return SUCCESS;
		}
}


/**
 *  @B 这个函数用来查找正确的文件然后标记对应的程序指示区！
 *
 */
void CheckFileIndexArea( )
{ // 标记有用的程序区！通过：查找正确的文件信息，从而标注正确的文件信息。
//	memset( W25Q4KBuf.databuf4K,0xFF,sizeof(W25Q4KBuf.databuf4K) );
//	u16 startIndex = 0; // 显然要找出数组的对应起始地址
//	u16 size = 0;       // 有多少个数组
//	for( u8 i = 0;i<120;i++ )
//	{
//		W25QXX_Read( filex.Buf,i*512,sizeof(filex.Buf));
//		if( ( filex.Status == 0x55 ) && ( GetSum(filex.Buf,sizeof(filex.Buf) - 1) == filex.SumValue ) ) // 表示这是一个正常的文件而且校验和正确！
//		{
//			startIndex = filex.ProgramSaveAtW25QAddress/0x1000 - 16;  // 4K地址起始地址为0x1000整数倍。减去16就是其对应的数组的下标！
//			if( filex.ProgramSize % 0x1000 == 0 )
//			{
//				size = filex.ProgramSize / 0x1000;
//			}
//			else
//				size = filex.ProgramSize / 0x1000 + 1;
//			for( u16 j=0;j<size;j++ )
//			{
//				W25Q4KBuf.databuf4K[startIndex+j] = 0x55; // 表示存储的文件有效果。
//			}
//		}
//		else
//			break;  // 能出循环，只要查到不是0x55，则认为后面的都不是正常的文件！
//	}
//	W25QXX_Erase_Sector(15*0x1000); // 擦除扇区
//	W25QXX_Write_NoCheck( W25Q4KBuf.databuf4K,15*0x1000,16*127 );  // 重新更新位置！
	{
		memset( W25Q4KBuf.databuf4K,0xFF,sizeof(W25Q4KBuf.databuf4K) );
		u16 startIndex = 0; // 显然要找出数组的对应起始地址
		u16 size = 0;       // 有多少个数组
		for( u8 i = 0;i<100;i++ )
		{
			W25QXX_Read( filex.Buf,W25QXX_FlieDensityAddress( i/8,(i%8)*2 ,0 ),sizeof(filex.Buf));  // 读取文件信息
			if( ( filex.Status == 0x55 ) && ( GetSum(filex.Buf,sizeof(filex.Buf) - 1) == filex.SumValue ) ) // 表示这是一个正常的文件而且校验和正确！
			{
				startIndex = filex.ProgramSaveAtW25QAddress/0x1000 - 256;  // 4K地址起始地址为0x1000整数倍。减去1024/4就是其对应的数组的下标！
				if( filex.ProgramSize % 0x1000 == 0 )
				{
					size = filex.ProgramSize / 0x1000;
				}
				else
					size = filex.ProgramSize / 0x1000 + 1;
				for( u16 j=0;j<size;j++ )
				{
					W25Q4KBuf.databuf4K[startIndex+j] = 0x55; // 表示存储的文件有效果。
				}
			}
			else
				break;  // 能出循环，只要查到不是0x55，则认为后面的都不是正常的文件！
		}
		W25QXX_Erase_Sector(W25QXX_FlieDensityAddress(15,0,0)); // 擦除扇区
		W25QXX_Write_NoCheck( W25Q4KBuf.databuf4K,W25QXX_FlieDensityAddress(15,0,0),16*127 );  // 重新更新位置！
	}
}

/**
 *  @B 整理程序区的文件！一定时间后，程序存储区的文件会变得比较乱，能连续的文件，不能发挥其最大能力。整理文件，同理整理出错的文件。包括校验不正确的文件。
 *  @way ：这个关联的比较多！文件信息256里面的起始地址，指示区中的指示，还有就是文件区！
 *  整理文件的目的是：整理删除错误的文件（文件信息和校验不正确！）然后整理并删除程序区AES加密值不正确的文件（CRC不正确，则删除文件信息）。最后重新排布文件（整理文件关联信息！）。达到最优化！
 *  这些其中的逻辑关系比较复杂。
 *  
 */

void arrangeTheProgramFile( )
{
	{ // 第一步，整理出错的文件：本次的出错文件为：信息和检验错误。然后删除。
		for( u16 j = 0;j<100; j++ ) // 最多只能存储120个文件
		{
			for( u16 i = 0;i<100; i++ ) // 最多只能存储120个文件
			{
				W25QXX_Read( filex.Buf,W25QXX_FlieDensityAddress( i/8,(i%8)*2,0 ),sizeof(filex.Buf) );  // 读出这个程序！
				if( ( filex.Buf[0] == 0x55 ) && ( GetSum( filex.Buf,sizeof(filex.Buf)-1 ) != filex.Buf[255] )   ) // 使用中的文件求和校验不正确，查找到一个错误文件！
				{
					deleteFile( 0,i + 1 ); // 显然，这个文件需要删除！
					break;  // 跳出循环，也就是每次只能处理一个文件。最多也就是120文件，所以外面用了一个循环。当然这个不是最优解，不过确定能解决问题！
				}
				if( filex.Buf[0] != 0x55 )  // 文件为0XFF，表示发现了一个没有使用的，日后也就不存在出错！
				{
					break;
				}
			}
		}
	} // 经过了这一步，我们认为：其中留存的都是正确的文件。
	{ // 检查AES值对应的和值，是不是正确！不正确，则删除！
		// 警告：因为AES和出现问题，而且本版本中没有使用AES加密，所以最后的判断，只要使用和校验就可以了！
		for( u16 j = 0;j<120; j++ ) // 因为可能存在120个坏文件，而每次只处理一个文件，所以用两重循环
		{
			for( u16 i = 0;i<120; i++ ) // 最多只能存储120个文件
			{
				u16 size = 0; // 程序可能会存在多个4K区，这个表示有多少个4K区！
				u16 sum = 0;  // 和校验值，每一次的查找都要重新初始化，防止被累加！
				
				W25QXX_Read( filex.Buf,W25QXX_FlieDensityAddress( i/8,(i%8)*2,0 ),sizeof(filex.Buf) );  // 读出这个程序！
				if( filex.Status == 0x55 )  // 判断文件，经过上面的判断，这个时候显然已经是正确的文件了。
				{
					if( filex.ProgramSize % 0x1000 == 0 )
					{
						size = filex.ProgramSize / 0x1000;
					}
					else
						size = filex.ProgramSize / 0x1000 + 1;
					for( u16 k = 0;k<size; k++ )  // 经过上面的判断，程序区最少为1个（当然：程序区必须存在程序！）
					{
						W25QXX_Read( W25Q4KBuf.databuf4K , filex.ProgramSaveAtW25QAddress + k*0x1000, 0x1000 ); // 先读出文件
						if( ( k == size - 1 ) && ( filex.ProgramSize % 0x1000 != 0 ))  // 最后一个程序区，要防止这个程序区正好4K大小！虽然概率小，但是仍然存在这样的问题的。
							sum += GetSumOf16Bit( W25Q4KBuf.databuf4K,filex.ProgramSize%0x1000 );
						else
							sum += GetSumOf16Bit( W25Q4KBuf.databuf4K,0x1000 );
					}
					if( sum != filex.SumValueOfProgram )  // 表明文件和检验出错！这个文件要删除
					{
						deleteFile( 0, i + 1 ); // 显然，这个文件需要删除！
						break;  // 跳出循环，也就是每次只能处理一个文件。最多也就是120文件，所以外面用了一个循环。当然这个不是最优解，不过确定能解决问题！
					}
				}
				else // 一旦发现不满足，则立刻退出！
					break;
			}
		}
	} // 经过上面的程序，文件信息区已经保证完好
	{  // 整理对应的指示区
		CheckFileIndexArea( );
	}
	{ // 更改映射表：如果发现不是连续的存储，则开始更新！
		// 显然需要知道之前的关系，然后再来搬移。
		// 注意：也一定要更改和值的关系。
		u16 size = 0;       // 有多少个数组
		u16 startIndexChange = 0; // 需要移动到的位置指示。从0开始（也就是程序区的第一个4K区域！）。
		u32 oldProgramSaveAtW25QAddress = 0;  // 搬移前程序存储的地址信息~
		for( u16 i = 0;i<100; i++ ) // 最多只能存储100个文件
		{
			W25QXX_Read( filex.Buf,W25QXX_FlieDensityAddress( i/8,(i%8)*2,0 ),sizeof(filex.Buf) );  // 读出这个程序！
// 下载指示区不需要更新到映射表中！
//			W25QXX_Read( filex.DownloaderNumbers,i*512+sizeof(filex.Buf),sizeof(filex.DownloaderNumbers) ); // 读取下载指示区
			if( filex.Status == 0x55 ) // 表示不是正确的文件！只有正确文件才可以处理！
			{
				if( filex.ProgramSaveAtW25QAddress != ( startIndexChange*0x1000 + 0x100000 ) ) // 判断文件是否在最连续的存储区中！
				{ // 文件不在连续的存储区中！需要进行文件搬移。
					{ // 更新文件信息中的地址内容，同时更新和校验！
						oldProgramSaveAtW25QAddress = filex.ProgramSaveAtW25QAddress;    // 
						filex.ProgramSaveAtW25QAddress = startIndexChange*0x1000 + 0x100000; // 这个为连续应该有的地址。
						{
							filex.SumValue = GetSum(filex.Buf,sizeof(filex.Buf)-1); // 也就是更新和filex.buf(255);
						}
						{  // 重新更新文件
							W25QXX_Write(filex.Buf,W25QXX_FlieDensityAddress( i/8,(i%8)*2,0 ),sizeof(filex.Buf));
						}
//						W25QXX_Read( W25Q4KBuf.databuf4K , i/8 * 0x1000 , 0x1000 ); // 先读出所在文件的4K缓存区
//						for( u16 j=0;j<sizeof(filex.Buf);j++ ) // 填充文件信息
//						{
//							W25Q4KBuf.databuf4K[i%8 * 512 + j] = filex.Buf[j];
//							W25Q4KBuf.databuf4K[i%8 * 512 + j + 256 ] = filex.DownloaderNumbers[j];
//						}
//						W25QXX_Erase_Sector( i/8 * 0x1000 ); // 删除这个4K文件区域！
//						W25QXX_Write( W25Q4KBuf.databuf4K,i/8 * 0x1000,0x1000 );  // 更新4K区域！
					}
					{ // 搬移程序的4K区域
						if( filex.ProgramSize % 0x1000 == 0 ) // 找出文件有多少个4K区域~
						{
							size = filex.ProgramSize / 0x1000;
						}
						else
							size = filex.ProgramSize / 0x1000 + 1;
						
						for( u16 m=0; m<size; m++ ) // 开始搬移区域！
						{
							W25QXX_Read( W25Q4KBuf.databuf4K , oldProgramSaveAtW25QAddress + m * 0x1000 , 0x1000 ); // 读出老地址所在文件区域
							W25QXX_Erase_Sector( filex.ProgramSaveAtW25QAddress + m * 0x1000 ); // 清除新地址所在的文件区域
							W25QXX_Write( W25Q4KBuf.databuf4K,filex.ProgramSaveAtW25QAddress + m * 0x1000,0x1000 );  // 写新文件到缓存区域
							W25QXX_Erase_Sector( oldProgramSaveAtW25QAddress + m * 0x1000 ); // 清除旧地址所在的文件区域
						}
					}
				}
				else
				{
				}
				{ // 不管文件的有没有再连续的存储区中，经过一个循环后，最优程序地址都应该改变。
					if( filex.ProgramSize % 0x1000 == 0 ) 
						{
							startIndexChange += filex.ProgramSize / 0x1000;
						}
						else
							startIndexChange += filex.ProgramSize / 0x1000 + 1;
				}
			}
			else
			{
				break;  // 当发现不是正确的文件后，则认为没有文件需要整理了~
			}
		}
	}
	{  // 两次整理对应的指示区
		CheckFileIndexArea( );
	}
}


/**
 *  @B 初始化文件信息~暂时不知道
 *
 */
void memoryManageInit( void )
{
	memset( &UpStatusInfo,0,sizeof(_UpFileInfo) );
	memset( &UpFileInfo,0,sizeof(_UpFileInfo) );
	
	UpStatusInfo.toUploadHeard0 = 0xAA;
	UpStatusInfo.toUploadHeard1 = 0x55;
	UpStatusInfo.functionStatus = 0x01;
	UpStatusInfo.functionCode = 0x01;
	UpStatusInfo.toUploaderTail = 0x88;
	
	UpFileInfo.toUploadHeard0 = 0xAA;
	UpFileInfo.toUploadHeard1 = 0x55;
	UpFileInfo.functionCode = 0x01;
	UpFileInfo.functionFileInfo = 0x02;
	UpFileInfo.toUploaderTail = 0x88;
}


