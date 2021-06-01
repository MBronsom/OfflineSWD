/**
 * ���ж���W25Q�Ĳ��������ﵽ�洢�����Ŀ�ģ���Ҫ������Լ��ƶ���Э�飬�����պ���Ҫ���ļ�����ϵͳ����������������֣�������Ҫ��Ϊ�˼��ٿ������ȣ�
 * 
 */

#include "memoryManage.h" 
#include "w25qxx.h" 
#include "ForSampleMain.h"  // ��ͺ���
#include "STM32_CRC.H"  

_W25QFlashInformation W25QFlashInformation;
_W25Q4KBuf W25Q4KBuf;
_UpStatusInfo UpStatusInfo ;
_UpFileInfo UpFileInfo;
_FileInformation filex;  // ����һ���ļ�
_FileInformation fileTemp;  // ����һ����ʱ�ļ�
/**
 *  @B �������ר���ڳ������ĺ�У�飡���ȳ�������ֵΪu16��ʽ��
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
 *  @B ����W25Q�����ԣ�����Block[128��]��Sector[128 * 16 = 2048��]��Page[128 * 16 * 16 = 32768��]���ڵ�ֵַ��
 */
 
u32 W25QXX_FlieDensityAddress( u8 block,u8 sector,u16 page )
{
	if( ( block >= 128 ) && ( sector >= 16 ) && ( page >= 256 ) )
	{
		return 0xFFFFFFFF;  // ����W25Q64�ļ������������ġ�
	}
	u32 address = block * 0x10000 + sector * 0x1000 + page * 0x100;
	return address;
}

/**
 *  @B �Ҵ洢�����ļ�����󳤶ȣ���ʣ���ļ���С
 *
 */
void W25QFlashProgramFileInformation( void )
{
//	u32 maxSize = 0;   // �ҳ����ֵ����ȻҪ��Ϊ�Աȣ�
//	W25QFlashInformation.RemainingSize = 0;
//	W25QFlashInformation.RemainingContinuousMaxSize = 0;
//	W25QXX_Read( W25Q4KBuf.databuf4K , W25QXX_FlieDensityAddress(0,15,0) , 2032); // �����������������
//	for( u16 i=0;i<2032;i++ )
//	{
//		if( W25Q4KBuf.databuf4K[i] == 0xFF ) // ��ʾ����(ֻ������״̬��һ�����У�һ��ʹ�ã�)
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
//		else  // ��ʾ������
//		{
//			if( W25QFlashInformation.RemainingContinuousMaxSize > maxSize)
//			{
//				maxSize = W25QFlashInformation.RemainingContinuousMaxSize;  // ��֮ǰ�ҵ����ļ���С
//			}
//			W25QFlashInformation.RemainingContinuousMaxSize = 0;  // ����
//		}
//	}
//	if( maxSize > W25QFlashInformation.RemainingContinuousMaxSize )
//		W25QFlashInformation.RemainingContinuousMaxSize = maxSize;  // 
//	
//	{ // ����Ϊ�ҳ��ļ�����Ŀ�ĺ��������Ϊ120���ļ���
//		W25QFlashInformation.FlieNumbers = 0;
//		for( u8 i = 0;i<120;i++ )
//		{
//			W25QXX_Read( filex.Buf,i*512,sizeof(filex.Buf));
//			if( filex.Status != 0x55 )  // �ҵ���һ����Ϊ�ļ��ģ�����Ϊ������
//			{
////				W25QFlashInformation.FlieNumbers = i;  // ���Ϊ�ļ�������
////				break;
//			}
//			else
//			{
//				W25QFlashInformation.FlieNumbers ++;  // �ļ�����+1
//			}
//		}
//	}

{ // ���ļ�������Ϣϵͳ
	u32 maxSize = 0;   // �ҳ����ֵ����ȻҪ��Ϊ�Աȣ�
	W25QFlashInformation.RemainingSize = 0;
	W25QFlashInformation.RemainingContinuousMaxSize = 0;
	W25QXX_Read( W25Q4KBuf.databuf4K , W25QXX_FlieDensityAddress(15,0,0) , 1792); // �����������������
	for( u16 i=0;i<1792;i++ )
	{
		if( W25Q4KBuf.databuf4K[i] == 0xFF ) // ��ʾ����(ֻ������״̬��һ�����У�һ��ʹ�ã�)
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
		else  // ��ʾ������
		{
			if( W25QFlashInformation.RemainingContinuousMaxSize > maxSize)
			{
				maxSize = W25QFlashInformation.RemainingContinuousMaxSize;  // ��֮ǰ�ҵ����ļ���С
			}
			W25QFlashInformation.RemainingContinuousMaxSize = 0;  // ����
		}
	}
	if( maxSize > W25QFlashInformation.RemainingContinuousMaxSize )
		W25QFlashInformation.RemainingContinuousMaxSize = maxSize;  // 
	
	{ // ����Ϊ�ҳ��ļ�����Ŀ�ĺ��������Ϊ100���ļ���
		W25QFlashInformation.FlieNumbers = 0;
		for( u8 i = 0;i<100;i++ )
		{
			W25QXX_Read( filex.Buf,W25QXX_FlieDensityAddress( i/8,(i%8)*2,0),sizeof(filex.Buf));
			if( filex.Status != 0x55 )  // �ҵ���һ����Ϊ�ļ��ģ�����Ϊ������
			{
			}
			else
			{
				W25QFlashInformation.FlieNumbers ++;  // �ļ�����+1
			}
		}
	}
}

}


/**
 *  @B ɾ���ļ�������������ɾ���ļ��ı���Ǵӣ�1��ʼ�ģ����Ǵ�0��ʼ�����Ҫע�⣡ɾ���ļ����ǽ�ɾ��������ļ���գ�ͬʱ������ļ���ǰ�ƶ�
       ����û�й����ǲ�����ȷ���ļ����ߵ�ǰ�����û���ļ���ɾ���ļ�����ָʾ�����ļ��洢��������Ȼ��ֻҪɾ���ļ�ָʾ���Ϳ����ˣ���
 *  // ���ݲ��Է��֣�����һ��ʱ����Ҫ500ms����()
 *  // ����fileTypeΪ0x00ʱ����ʾɾ�������ļ���fileIndex��ʾ�ļ���š���fileTypeΪ0x01����ʾȫ��ɾ����
 *  @PARA fileIndex = [1-120] // �ļ���Ŵ�1-120
 *  @WARNING ɾ���ļ���ֻ���ƶ���Ϣ���֣������Ѿ����ش�������Ҳ�����ƶ�512���ֽڣ���
 */
u8 deleteFile( u8 fileType,u16 fileIndex )
{
//	if( fileType == 0x01 )
//	{
//		for( u8 i=0;i<16;i++ ) // ɾ�����е��ļ���
//		{
//			W25QXX_Erase_Sector(i*0x1000);  // ���64K�ļ���������ָʾ����
//		}
//	}
//	else if( fileType == 0x00 )
//	{
//			if( fileIndex < 1 || fileIndex >120 )
//			{
//				return FAIL;  //�ļ����ڷ�Χ�ڣ�
//			}
//			for( u8 i=0;i<15;i++ )  // ��64KB��ǰ60KB�д洢���ĸ�Sector
//			{
////				if( i == fileIndex / 8)
//				if( ( (fileIndex-1)*512 < ((i+1)*0x1000) ) &&  ( (fileIndex-1)*512 >= (i*0x1000) ) ) // ������㣬���ڵ�i�飡
//				{
//					W25QXX_Read( W25Q4KBuf.databuf4K , i * 0x1000 , 0x1000 ); // �Ȱ������ļ���ŵ���С������sector������������databuf4K�У�
//					W25QXX_Erase_Sector( i * 0x1000 ); // �����������
//					{ // ����databuf4K�����ƻ���������
//						for( u16 j=0;j<0x1000;j++ )
//						{
//							if( (j>=(((fileIndex-1)%8)*512)) && ( j < (((fileIndex-1)%8)*512+512) ) ) // ���������ļ���Χ֮��
//							{
//								// ����Ҫ����
//							}
//							else if( (j<(((fileIndex-1)%8)*512)) )  // ���ļ���Χ֮��
//							{
//								 // ���֣�����Ҫ����
//							}
//							else if( j >= (((fileIndex-1)%8)*512+512) )// ���ļ���Χ֮��
//							{
//								W25Q4KBuf.databuf4K[j-512] = W25Q4KBuf.databuf4K[j];  // ��ǰ���512�����ֽڸ���
//							}
//						}
//						W25QXX_Write_NoCheck( W25Q4KBuf.databuf4K,i*0x1000,4096-512 );  // ������һ��Flash����Ϊ�Ѿ�ɾ����һ��������Ҫ��512�ֽڣ�Ҳ��������ļ��£�ֻд7���ļ���
//						{ // ��ʣ�µĴ洢��ǰ���ƶ���
//							i++; // �����ҳ
//							while( i< 15 ) // ��Ȼ�ڷ�Χ֮�ڣ���Ҫ�������ݣ�
//							{
//								W25QXX_Read( W25Q4KBuf.databuf4K , i * 0x1000 , 0x1000 ); // �����������������
//								W25QXX_Erase_Sector(i * 0x1000); // �����������������
//								W25QXX_Write_NoCheck( W25Q4KBuf.databuf4K,i*0x1000 - 512,4096 );  // ���£���ʱҪ����һ��������ʼ���£����Ե�ַ-512�Ϳ��ԣ�Ȼ����д4K����
//								i++; // ���棺����i++д��while( i++ <15){}����ִ���(�о�����i+1�������ļ�û�и��Ϻ���)�ѵ�����ִ���жϺ��ִ����++����Ӧ����ִ����{}����++ô����Ӧ�����ж����ִ����++�������Ҫע�⣡��
//							};
//						}
//					}
//					break;  // �˳�ѭ��
//				}
//				else {};
//			}
//			
//		}
//		else
//		{
//			return FAIL;
//		}
//	// ��ʱ�ļ�������
//		return SUCCESS;
		
		{ // ��ɾ���ļ�����
			if( fileType == 0x01 )
			{
				for( u8 i=0;i<16;i++ ) // ɾ�����е��ļ�:ʹ����ǰ���16��Block(1MB�Ŀռ�)
				{
					W25QXX_Erase_Block(W25QXX_FlieDensityAddress(i,0,0));  // ���64K�ļ���������ָʾ����
				}
			}
			else if( fileType == 0x00 )
			{
				if( fileIndex < 1 || fileIndex >100 )
				{
					return FAIL;  //�ļ����ڷ�Χ�ڣ�
				}
				else
				{  // ɾ���ļ���ͬʱ���������ļ�
					{ // ��ɾ���ļ�
						W25QXX_Erase_Sector( W25QXX_FlieDensityAddress( (fileIndex-1)/8,((fileIndex-1)%8)*2  ,0 ) );  // ɾ���ļ���Ϣ
						W25QXX_Erase_Sector( W25QXX_FlieDensityAddress( (fileIndex-1)/8,((fileIndex-1)%8)*2+1,0 ) );  // ɾ�����ش���ָʾ
					}
					{  // �Ѻ�����ĵ���ǰ�ƽ���Ҳ����ִ�У���д����-��д��������
						for( u8 i = fileIndex+1,j = 0;i<=100;i++ )  
						{  
							j = i-1;  // ��ʾ��һ���ļ�
							W25QXX_Read( W25Q4KBuf.databuf4K , W25QXX_FlieDensityAddress( (i-1)/8,((i-1)%8)*2  ,0 ) , 0x1000 );  // ���ļ�ָʾ��
							W25QXX_Write_NoCheck( W25Q4KBuf.databuf4K, W25QXX_FlieDensityAddress( (j-1)/8,((j-1)%8)*2  ,0 ),0x1000 );  // д����һ���ļ�ָʾ
							W25QXX_Erase_Sector( W25QXX_FlieDensityAddress( (i-1)/8,((i-1)%8)*2  ,0 ) );  // ɾ���ոյĶ������
							
							W25QXX_Read( W25Q4KBuf.databuf4K , W25QXX_FlieDensityAddress( (i-1)/8,((i-1)%8)*2+1  ,0 ) , 0x1000 );  // ���ļ����ش�����
							W25QXX_Write_NoCheck( W25Q4KBuf.databuf4K, W25QXX_FlieDensityAddress( (j-1)/8,((j-1)%8)*2+1  ,0 ),0x1000 );  // д����һ���ļ����ش�����
							W25QXX_Erase_Sector( W25QXX_FlieDensityAddress( (i-1)/8,((i-1)%8)*2+1 ,0 ) );  // ɾ���ոյĶ������
						}
					}
				}
			}
			else
			{
				return FAIL;
			}
			// ��ʱ�ļ�������
				return SUCCESS;
		}
}


/**
 *  @B �����������������ȷ���ļ�Ȼ���Ƕ�Ӧ�ĳ���ָʾ����
 *
 */
void CheckFileIndexArea( )
{ // ������õĳ�������ͨ����������ȷ���ļ���Ϣ���Ӷ���ע��ȷ���ļ���Ϣ��
//	memset( W25Q4KBuf.databuf4K,0xFF,sizeof(W25Q4KBuf.databuf4K) );
//	u16 startIndex = 0; // ��ȻҪ�ҳ�����Ķ�Ӧ��ʼ��ַ
//	u16 size = 0;       // �ж��ٸ�����
//	for( u8 i = 0;i<120;i++ )
//	{
//		W25QXX_Read( filex.Buf,i*512,sizeof(filex.Buf));
//		if( ( filex.Status == 0x55 ) && ( GetSum(filex.Buf,sizeof(filex.Buf) - 1) == filex.SumValue ) ) // ��ʾ����һ���������ļ�����У�����ȷ��
//		{
//			startIndex = filex.ProgramSaveAtW25QAddress/0x1000 - 16;  // 4K��ַ��ʼ��ַΪ0x1000����������ȥ16�������Ӧ��������±꣡
//			if( filex.ProgramSize % 0x1000 == 0 )
//			{
//				size = filex.ProgramSize / 0x1000;
//			}
//			else
//				size = filex.ProgramSize / 0x1000 + 1;
//			for( u16 j=0;j<size;j++ )
//			{
//				W25Q4KBuf.databuf4K[startIndex+j] = 0x55; // ��ʾ�洢���ļ���Ч����
//			}
//		}
//		else
//			break;  // �ܳ�ѭ����ֻҪ�鵽����0x55������Ϊ����Ķ������������ļ���
//	}
//	W25QXX_Erase_Sector(15*0x1000); // ��������
//	W25QXX_Write_NoCheck( W25Q4KBuf.databuf4K,15*0x1000,16*127 );  // ���¸���λ�ã�
	{
		memset( W25Q4KBuf.databuf4K,0xFF,sizeof(W25Q4KBuf.databuf4K) );
		u16 startIndex = 0; // ��ȻҪ�ҳ�����Ķ�Ӧ��ʼ��ַ
		u16 size = 0;       // �ж��ٸ�����
		for( u8 i = 0;i<100;i++ )
		{
			W25QXX_Read( filex.Buf,W25QXX_FlieDensityAddress( i/8,(i%8)*2 ,0 ),sizeof(filex.Buf));  // ��ȡ�ļ���Ϣ
			if( ( filex.Status == 0x55 ) && ( GetSum(filex.Buf,sizeof(filex.Buf) - 1) == filex.SumValue ) ) // ��ʾ����һ���������ļ�����У�����ȷ��
			{
				startIndex = filex.ProgramSaveAtW25QAddress/0x1000 - 256;  // 4K��ַ��ʼ��ַΪ0x1000����������ȥ1024/4�������Ӧ��������±꣡
				if( filex.ProgramSize % 0x1000 == 0 )
				{
					size = filex.ProgramSize / 0x1000;
				}
				else
					size = filex.ProgramSize / 0x1000 + 1;
				for( u16 j=0;j<size;j++ )
				{
					W25Q4KBuf.databuf4K[startIndex+j] = 0x55; // ��ʾ�洢���ļ���Ч����
				}
			}
			else
				break;  // �ܳ�ѭ����ֻҪ�鵽����0x55������Ϊ����Ķ������������ļ���
		}
		W25QXX_Erase_Sector(W25QXX_FlieDensityAddress(15,0,0)); // ��������
		W25QXX_Write_NoCheck( W25Q4KBuf.databuf4K,W25QXX_FlieDensityAddress(15,0,0),16*127 );  // ���¸���λ�ã�
	}
}

/**
 *  @B ������������ļ���һ��ʱ��󣬳���洢�����ļ����ñȽ��ң����������ļ������ܷ�������������������ļ���ͬ�����������ļ�������У�鲻��ȷ���ļ���
 *  @way ����������ıȽ϶࣡�ļ���Ϣ256�������ʼ��ַ��ָʾ���е�ָʾ�����о����ļ�����
 *  �����ļ���Ŀ���ǣ�����ɾ��������ļ����ļ���Ϣ��У�鲻��ȷ����Ȼ������ɾ��������AES����ֵ����ȷ���ļ���CRC����ȷ����ɾ���ļ���Ϣ������������Ų��ļ��������ļ�������Ϣ�������ﵽ���Ż���
 *  ��Щ���е��߼���ϵ�Ƚϸ��ӡ�
 *  
 */

void arrangeTheProgramFile( )
{
	{ // ��һ�������������ļ������εĳ����ļ�Ϊ����Ϣ�ͼ������Ȼ��ɾ����
		for( u16 j = 0;j<100; j++ ) // ���ֻ�ܴ洢120���ļ�
		{
			for( u16 i = 0;i<100; i++ ) // ���ֻ�ܴ洢120���ļ�
			{
				W25QXX_Read( filex.Buf,W25QXX_FlieDensityAddress( i/8,(i%8)*2,0 ),sizeof(filex.Buf) );  // �����������
				if( ( filex.Buf[0] == 0x55 ) && ( GetSum( filex.Buf,sizeof(filex.Buf)-1 ) != filex.Buf[255] )   ) // ʹ���е��ļ����У�鲻��ȷ�����ҵ�һ�������ļ���
				{
					deleteFile( 0,i + 1 ); // ��Ȼ������ļ���Ҫɾ����
					break;  // ����ѭ����Ҳ����ÿ��ֻ�ܴ���һ���ļ������Ҳ����120�ļ���������������һ��ѭ������Ȼ����������Ž⣬����ȷ���ܽ�����⣡
				}
				if( filex.Buf[0] != 0x55 )  // �ļ�Ϊ0XFF����ʾ������һ��û��ʹ�õģ��պ�Ҳ�Ͳ����ڳ���
				{
					break;
				}
			}
		}
	} // ��������һ����������Ϊ����������Ķ�����ȷ���ļ���
	{ // ���AESֵ��Ӧ�ĺ�ֵ���ǲ�����ȷ������ȷ����ɾ����
		// ���棺��ΪAES�ͳ������⣬���ұ��汾��û��ʹ��AES���ܣ����������жϣ�ֻҪʹ�ú�У��Ϳ����ˣ�
		for( u16 j = 0;j<120; j++ ) // ��Ϊ���ܴ���120�����ļ�����ÿ��ֻ����һ���ļ�������������ѭ��
		{
			for( u16 i = 0;i<120; i++ ) // ���ֻ�ܴ洢120���ļ�
			{
				u16 size = 0; // ������ܻ���ڶ��4K���������ʾ�ж��ٸ�4K����
				u16 sum = 0;  // ��У��ֵ��ÿһ�εĲ��Ҷ�Ҫ���³�ʼ������ֹ���ۼӣ�
				
				W25QXX_Read( filex.Buf,W25QXX_FlieDensityAddress( i/8,(i%8)*2,0 ),sizeof(filex.Buf) );  // �����������
				if( filex.Status == 0x55 )  // �ж��ļ�������������жϣ����ʱ����Ȼ�Ѿ�����ȷ���ļ��ˡ�
				{
					if( filex.ProgramSize % 0x1000 == 0 )
					{
						size = filex.ProgramSize / 0x1000;
					}
					else
						size = filex.ProgramSize / 0x1000 + 1;
					for( u16 k = 0;k<size; k++ )  // ����������жϣ�����������Ϊ1������Ȼ��������������ڳ��򣡣�
					{
						W25QXX_Read( W25Q4KBuf.databuf4K , filex.ProgramSaveAtW25QAddress + k*0x1000, 0x1000 ); // �ȶ����ļ�
						if( ( k == size - 1 ) && ( filex.ProgramSize % 0x1000 != 0 ))  // ���һ����������Ҫ��ֹ�������������4K��С����Ȼ����С��������Ȼ��������������ġ�
							sum += GetSumOf16Bit( W25Q4KBuf.databuf4K,filex.ProgramSize%0x1000 );
						else
							sum += GetSumOf16Bit( W25Q4KBuf.databuf4K,0x1000 );
					}
					if( sum != filex.SumValueOfProgram )  // �����ļ��ͼ����������ļ�Ҫɾ��
					{
						deleteFile( 0, i + 1 ); // ��Ȼ������ļ���Ҫɾ����
						break;  // ����ѭ����Ҳ����ÿ��ֻ�ܴ���һ���ļ������Ҳ����120�ļ���������������һ��ѭ������Ȼ����������Ž⣬����ȷ���ܽ�����⣡
					}
				}
				else // һ�����ֲ����㣬�������˳���
					break;
			}
		}
	} // ��������ĳ����ļ���Ϣ���Ѿ���֤���
	{  // �����Ӧ��ָʾ��
		CheckFileIndexArea( );
	}
	{ // ����ӳ���������ֲ��������Ĵ洢����ʼ���£�
		// ��Ȼ��Ҫ֪��֮ǰ�Ĺ�ϵ��Ȼ���������ơ�
		// ע�⣺Ҳһ��Ҫ���ĺ�ֵ�Ĺ�ϵ��
		u16 size = 0;       // �ж��ٸ�����
		u16 startIndexChange = 0; // ��Ҫ�ƶ�����λ��ָʾ����0��ʼ��Ҳ���ǳ������ĵ�һ��4K���򣡣���
		u32 oldProgramSaveAtW25QAddress = 0;  // ����ǰ����洢�ĵ�ַ��Ϣ~
		for( u16 i = 0;i<100; i++ ) // ���ֻ�ܴ洢100���ļ�
		{
			W25QXX_Read( filex.Buf,W25QXX_FlieDensityAddress( i/8,(i%8)*2,0 ),sizeof(filex.Buf) );  // �����������
// ����ָʾ������Ҫ���µ�ӳ����У�
//			W25QXX_Read( filex.DownloaderNumbers,i*512+sizeof(filex.Buf),sizeof(filex.DownloaderNumbers) ); // ��ȡ����ָʾ��
			if( filex.Status == 0x55 ) // ��ʾ������ȷ���ļ���ֻ����ȷ�ļ��ſ��Դ���
			{
				if( filex.ProgramSaveAtW25QAddress != ( startIndexChange*0x1000 + 0x100000 ) ) // �ж��ļ��Ƿ����������Ĵ洢���У�
				{ // �ļ����������Ĵ洢���У���Ҫ�����ļ����ơ�
					{ // �����ļ���Ϣ�еĵ�ַ���ݣ�ͬʱ���º�У�飡
						oldProgramSaveAtW25QAddress = filex.ProgramSaveAtW25QAddress;    // 
						filex.ProgramSaveAtW25QAddress = startIndexChange*0x1000 + 0x100000; // ���Ϊ����Ӧ���еĵ�ַ��
						{
							filex.SumValue = GetSum(filex.Buf,sizeof(filex.Buf)-1); // Ҳ���Ǹ��º�filex.buf(255);
						}
						{  // ���¸����ļ�
							W25QXX_Write(filex.Buf,W25QXX_FlieDensityAddress( i/8,(i%8)*2,0 ),sizeof(filex.Buf));
						}
//						W25QXX_Read( W25Q4KBuf.databuf4K , i/8 * 0x1000 , 0x1000 ); // �ȶ��������ļ���4K������
//						for( u16 j=0;j<sizeof(filex.Buf);j++ ) // ����ļ���Ϣ
//						{
//							W25Q4KBuf.databuf4K[i%8 * 512 + j] = filex.Buf[j];
//							W25Q4KBuf.databuf4K[i%8 * 512 + j + 256 ] = filex.DownloaderNumbers[j];
//						}
//						W25QXX_Erase_Sector( i/8 * 0x1000 ); // ɾ�����4K�ļ�����
//						W25QXX_Write( W25Q4KBuf.databuf4K,i/8 * 0x1000,0x1000 );  // ����4K����
					}
					{ // ���Ƴ����4K����
						if( filex.ProgramSize % 0x1000 == 0 ) // �ҳ��ļ��ж��ٸ�4K����~
						{
							size = filex.ProgramSize / 0x1000;
						}
						else
							size = filex.ProgramSize / 0x1000 + 1;
						
						for( u16 m=0; m<size; m++ ) // ��ʼ��������
						{
							W25QXX_Read( W25Q4KBuf.databuf4K , oldProgramSaveAtW25QAddress + m * 0x1000 , 0x1000 ); // �����ϵ�ַ�����ļ�����
							W25QXX_Erase_Sector( filex.ProgramSaveAtW25QAddress + m * 0x1000 ); // ����µ�ַ���ڵ��ļ�����
							W25QXX_Write( W25Q4KBuf.databuf4K,filex.ProgramSaveAtW25QAddress + m * 0x1000,0x1000 );  // д���ļ�����������
							W25QXX_Erase_Sector( oldProgramSaveAtW25QAddress + m * 0x1000 ); // ����ɵ�ַ���ڵ��ļ�����
						}
					}
				}
				else
				{
				}
				{ // �����ļ�����û���������Ĵ洢���У�����һ��ѭ�������ų����ַ��Ӧ�øı䡣
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
				break;  // �����ֲ�����ȷ���ļ�������Ϊû���ļ���Ҫ������~
			}
		}
	}
	{  // ���������Ӧ��ָʾ��
		CheckFileIndexArea( );
	}
}


/**
 *  @B ��ʼ���ļ���Ϣ~��ʱ��֪��
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


