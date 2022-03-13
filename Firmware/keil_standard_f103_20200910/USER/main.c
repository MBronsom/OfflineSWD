#include "main.h"

/***********************文件系统使用定义************************/
FIL fnew;													/* file objects */
FILINFO FileInfo = {0};
DIR DirInfo;
FATFS fs;													/* Work area (file system object) for logical drives */
FRESULT Res; 
UINT br, bw;            					/* File R/W count */

TCHAR lFileName[255] = {0};


/***********************变量定义************************/
char rData[1024] = "";
u8 readflag = 1;
u32 addr = 0;
u32 i = 0;
u32 select = 0;
u32 fileCount;
u32 fileChangeCount;
u8 breakDebug = 0;
u8 debugMode = 0;
u8 menuType = MAIN_WINDOW;
char* debugBaud[9] = {"4800","9600","14400","19200","38400","57600","115200","256000","921600"};
u8 debugSelect = 1; //默认选择波特率为9600
uint16_t bytesread;
u8 Logo[] = "BRONSON";

/***********************主函数************************/
 int main(void)
 {
	 Init_device(); //初始化设备
	 Draw_Logo(); //绘制LOGO
	 Draw_Menu(); //绘制菜单
	 while(1);
}
 
/***********************初始化设备************************/
void Init_device()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //优先级调整
	delay_init(); //初始化延时
	Key_Init(); //按键初始化
	W25QXX_Init(); //初始化Flash芯片
	f_mount(0,&fs); //初始化文件系统
	OLED_Init(); //初始化OLED  
	OLED_Clear(); //清空OLED屏幕
	LED_Init();
	if(Scan_Key() == 1 ){ //按住SELTCT开机进入CMSIS-DAP模式
		Draw_Logo(); //绘制LOGO
		OLED_DrawBMP(0,0,34,34,USBLogo); //绘制图标
		OLED_ShowString(38,1,(u8*)"DAP Connect",1,0); //绘制提示词
		Init_DAPUSB(); //初始化DAP
		Do_DAPUSB(); //DAP循环
	}
	else{
		Set_System(); //设置USB系统
	  Set_USBClock(); //设置USB时钟
	  USB_Interrupts_Config(); //设置USB中断
	  USB_Init(); //初始化USB
	}
}

/***********************绘制LOGO************************/
void Draw_Logo(){
	 u8 i;
	 for(i=0;i<7;i++){
     OLED_ShowChar(34+i*8,1,Logo[i],1);
	   delay_ms(15);
	 }
	 delay_ms(1000);
	 OLED_Clear();
}

/***********************绘制菜单************************/
void Draw_Menu(){
	if(bDeviceState != UNCONNECTED){
		OLED_DrawBMP(0,0,34,34,USBLogo);
		OLED_ShowString(38,1,(u8*)"USB Connect",1,0);
	}
  else{
		OLED_DrawBMP(0,0,33,33,FlashLogo);
		//自动烧写模式（将命名为AUTO.bin的文件放入根目录）
		if(f_open(&fnew, (const TCHAR*)"AUTO.bin",FA_READ ) == FR_OK){ 
		  OLED_ShowString(45,-1,(u8*)"AUTO FLASH",1,0);
		  while(!FLASH_SWD((u8*)"AUTO.bin")){
		    u8 WaitTips[] = "...";
			  OLED_ShowString(45,1,(u8*)"          ",1,0);
			  OLED_ShowString(45,2,(u8*)"WAIT",1,1);
			  for(i=0;i<3;i++){
          OLED_ShowChar(69+i*6,2,WaitTips[i],1);
	        delay_ms(200);
	      }
			  OLED_ShowString(45,2,(u8*)"       ",1,1);
		  }
			OLED_ShowString(98,2,(u8*)"BACK",1,1);
			select = 0;
			while(1)
			{
				if(Scan_Key() == 1){
					select ++;
					if(select == 1) {
						OLED_ShowString(98,2,(u8*)"    ",1,1);
						OLED_ShowString(98,2,(u8*)"BACK",0,1);
					}
					else{
						OLED_ShowString(98,2,(u8*)"    ",1,1);
						OLED_ShowString(98,2,(u8*)"BACK",1,1);
					}
					if(select == 2) select = 0;
				}
				if(Scan_Key() == 2){
					if(select == 1) break;
				}
			}
			OLED_Clear();
		  OLED_DrawBMP(0,0,33,33,FlashLogo);
		}
		Draw_Main(); //进入主菜单
	}
}

/***********************绘制主菜单************************/
void Draw_Main(){
	FileInfo.lfname = lFileName;
	FileInfo.lfsize = 255;
	
	f_unlink("0:/write.bin");
	if(f_opendir(&DirInfo,(const TCHAR*)"0:") == FR_OK)/* 打开文件夹目录成功，目录信息已经在dir结构体中保存 */
  {
      if(f_readdir(&DirInfo, &FileInfo) == FR_OK)  /* 读文件信息到文件状态结构体中 */
      {
				f_readdir(&DirInfo, &FileInfo);
				int fileLen = strlen(FileInfo.lfname);
				if(fileLen != 0 || strlen(FileInfo.fname)){
					if(fileLen < 7) OLED_ShowString(45,1,(u8*)FileInfo.fname,1,1);
					else{
						char fileShortName[7] = {0};
						strncpy(fileShortName, FileInfo.lfname, 7);
						OLED_ShowString(45,1,(u8*)fileShortName,1,1);
					}
				}
				else OLED_ShowString(45,1,(u8*)"NULL",1,1);
			}
	}		
	OLED_ShowString(45,-1,(u8*)"SELECT HEX",1,0);
	OLED_ShowString(45,0,(u8*)"----------",1,0);
	OLED_ShowString(110,1,(u8*)">>",1,1);
	OLED_ShowString(92,2,(u8*)"FLASH",1,1);
	select = 0;
	fileCount = 0;
	fileChangeCount = 0;
	while(1)
	{
		if(menuType == MAIN_WINDOW && select == 1 && strlen(FileInfo.lfname) > 7){
			char fileShortName[7] = {0};
			strncpy(fileShortName, &FileInfo.lfname[fileCount], 7);
			OLED_ShowString(45,1,(u8*)fileShortName,0,1);
			fileChangeCount ++;
			if(fileCount == 0 && fileChangeCount % 2000 == 0) fileCount ++;
			if(fileChangeCount % 500 == 0 && fileCount < strlen(FileInfo.lfname)-7 && fileCount != 0) fileCount ++;
			if(fileCount == strlen(FileInfo.lfname)-7 && fileChangeCount % 5000 == 0) fileCount = 0;
		}
		if(Scan_Key() == 1){
			select ++;
			if(select == 4) select = 0;
			switch(select){
				case 0:
				  if(strlen(FileInfo.lfname) != 0){
						char fileShortName[7] = {0};
						strncpy(fileShortName, FileInfo.lfname, 7);
						OLED_ShowString(45,1,(u8*)fileShortName,1,1);
					}
					else if(strlen(FileInfo.fname) != 0) OLED_ShowString(45,1,FileInfo.fname,1,1);
				  else OLED_ShowString(45,1,(u8*)"NULL",1,1);
					OLED_ShowString(110,1,(u8*)">>",1,1);
				  OLED_ShowString(92,2,(u8*)"FLASH",1,1);
				  break;
				case 1:
					fileCount = 0;
				  fileChangeCount = 0;
				  if(strlen(FileInfo.lfname) != 0){
						char fileShortName[7] = {0};
						strncpy(fileShortName, FileInfo.lfname, 7);
						OLED_ShowString(45,1,(u8*)fileShortName,0,1);
					}
					else if(strlen(FileInfo.fname) != 0) OLED_ShowString(45,1,FileInfo.fname,0,1);
				  else OLED_ShowString(45,1,(u8*)"NULL",0,1);
					OLED_ShowString(110,1,(u8*)">>",1,1);
				  OLED_ShowString(92,2,(u8*)"FLASH",1,1);
				  break;
				case 2:
				  if(strlen(FileInfo.lfname) != 0){
						char fileShortName[7] = {0};
						strncpy(fileShortName, FileInfo.lfname, 7);
						OLED_ShowString(45,1,(u8*)fileShortName,1,1);
					}
					else if(strlen(FileInfo.fname) != 0) OLED_ShowString(45,1,FileInfo.fname,1,1);
				  else OLED_ShowString(45,1,(u8*)"NULL",1,1);
					OLED_ShowString(110,1,(u8*)">>",0,1);
				  OLED_ShowString(92,2,(u8*)"FLASH",1,1);
				  break;
				case 3:
				  if(strlen(FileInfo.lfname) != 0){
						char fileShortName[7] = {0};
						strncpy(fileShortName, FileInfo.lfname, 7);
						OLED_ShowString(45,1,(u8*)fileShortName,1,1);
					}
					else if(strlen(FileInfo.fname) != 0) OLED_ShowString(45,1,FileInfo.fname,1,1);
				  else OLED_ShowString(45,1,(u8*)"NULL",1,1);
					OLED_ShowString(110,1,(u8*)">>",1,1);
				  OLED_ShowString(92,2,(u8*)"FLASH",0,1);
				  break;
			}
		}
		if(Scan_Key() == 2){
			switch(select){
				case 1: //选择文件
					fileCount = 0;
				  fileChangeCount = 0;
					if(strlen(FileInfo.lfname) != 0 || strlen(FileInfo.fname) != 0){
						f_readdir(&DirInfo, &FileInfo);
						OLED_ShowString(45,1,(u8*)"          ",1, 1);
						if(strlen(FileInfo.lfname) < 7) OLED_ShowString(45,1,(u8*)FileInfo.fname,0,1);
						else{
							char fileShortName[7] = {0};
							strncpy(fileShortName, FileInfo.lfname, 7);
							OLED_ShowString(45,1,(u8*)fileShortName,0,1);
						}
						OLED_ShowString(110,1,(u8*)">>",1,1);
						if(!FileInfo.lfname[0] || !FileInfo.fname[0]){
							f_opendir(&DirInfo,(const TCHAR*)"0:");
							f_readdir(&DirInfo, &FileInfo);
							f_readdir(&DirInfo, &FileInfo);
							OLED_ShowString(45,1,(u8*)"          ",1,1);
							if(strlen(FileInfo.lfname) < 7) OLED_ShowString(45,1,(u8*)FileInfo.fname,0,1);
							else{
								char fileShortName[7] = {0};
								strncpy(fileShortName, FileInfo.lfname, 7);
								OLED_ShowString(45,1,(u8*)fileShortName,0,1);
							}
							OLED_ShowString(110,1,(u8*)">>",1,1);
						}
				  }
					break;
				case 2: //进入调试模式
					select = 0;
				  menuType = DEBUG_WINDOW;
					OLED_Clear();
				  OLED_DrawBMP(0,0,32,32,DebugLogo);
				  OLED_ShowString(45,-1,(u8*)"DEBUG MODE",1,0);
					OLED_ShowString(45,0,(u8*)"----------",1,0);
					OLED_ShowString(45,1,(u8*)debugBaud[debugSelect],1,1);
				  OLED_ShowString(110,1,(u8*)">>",1,1);
				  OLED_ShowString(92,2,(u8*)"ENTER",1,1);
				  while(1){
						if(Scan_Key() == 1){
							select ++;
							if(select == 4) select = 0;
							switch(select){
								case 0:
									OLED_ShowString(45,1,(u8*)"      ",1,1);
									OLED_ShowString(45,1,(u8*)debugBaud[debugSelect],1,1);
									OLED_ShowString(110,1,(u8*)">>",1,1);
								  OLED_ShowString(92,2,(u8*)"ENTER",1,1);
									break;
								case 1:
									OLED_ShowString(45,1,(u8*)"      ",1,1);
									OLED_ShowString(45,1,(u8*)debugBaud[debugSelect],0,1);
									OLED_ShowString(110,1,(u8*)">>",1,1);
								  OLED_ShowString(92,2,(u8*)"ENTER",1,1);
									break;
								case 2:
									OLED_ShowString(45,1,(u8*)"      ",1,1);
									OLED_ShowString(45,1,(u8*)debugBaud[debugSelect],1,1);
									OLED_ShowString(110,1,(u8*)">>",0,1);
								  OLED_ShowString(92,2,(u8*)"ENTER",1,1);
									break;
								case 3:
									OLED_ShowString(45,1,(u8*)"      ",1,1);
									OLED_ShowString(45,1,(u8*)debugBaud[debugSelect],1,1);
									OLED_ShowString(110,1,(u8*)">>",1,1);
								  OLED_ShowString(92,2,(u8*)"ENTER",0,1);
									break;
							}
							delay_ms(100);
						}
						
						if(Scan_Key() == 2){
							switch(select){
								  case 1: //修改调试模式波特率
										debugSelect++;
									  if(debugSelect == 9) debugSelect = 0;
										OLED_ShowString(45,1,(u8*)"      ",0,1);
										OLED_ShowString(45,1,(u8*)debugBaud[debugSelect],0,1);
									  switch(debugSelect){
											case 0:
											  usart_config(4800);
												break;
											case 1:
												usart_config(9600);
												break;
											case 2:
												usart_config(14400);
												break;
											case 3:
												usart_config(19200);
												break;
											case 4:
												usart_config(38400);
												break;
											case 5:
												usart_config(57600);
												break;
											case 6:
												usart_config(115200);
												break;
											case 7:
												usart_config(256000);
												break;
											case 8:
												usart_config(921600);
												break;
										}
										break;
									case 2: //进入下一页面
										menuType = MAIN_WINDOW;
										OLED_Clear();
										OLED_DrawBMP(0,0,33,33,FlashLogo);
										if(f_opendir(&DirInfo,(const TCHAR*)"0:") == FR_OK)/* 打开文件夹目录成功，目录信息已经在dir结构体中保存 */
										{
											if(f_readdir(&DirInfo, &FileInfo) == FR_OK)  /* 读文件信息到文件状态结构体中 */
											{
												f_readdir(&DirInfo, &FileInfo);
												int fileLen = strlen(FileInfo.lfname);
												if(fileLen != 0 || strlen(FileInfo.fname)){
													if(fileLen < 7) OLED_ShowString(45,1,(u8*)FileInfo.fname,1,1);
													else{
														char fileShortName[7] = {0};
														strncpy(fileShortName, FileInfo.lfname, 7);
														OLED_ShowString(45,1,(u8*)fileShortName,1,1);
													}
												}
												else OLED_ShowString(45,1,(u8*)"NULL",1,1);
											}
										}		
										OLED_ShowString(45,-1,(u8*)"SELECT HEX",1,0);
										OLED_ShowString(45,0,(u8*)"----------",1,0);
										OLED_ShowString(110,1,(u8*)">>",1,1);
										OLED_ShowString(92,2,(u8*)"FLASH",1,1);
										select = 0;
										breakDebug = 1;
										break;
									case 3: //确认进入调试模式
										select = 0;
									  menuType = DEBUGING_WINDOW;
									  debugMode = 1;
										OLED_Clear();
										OLED_ShowString(98,2,(u8*)"BACK",1,1);
									  while(1)
										{
											if(Scan_Key() == 1){
												select ++;
												if(select == 1) {
													OLED_ShowString(98,2,(u8*)"    ",1,1);
													OLED_ShowString(98,2,(u8*)"BACK",0,1);
												}
												else{
													OLED_ShowString(98,2,(u8*)"    ",1,1);
													OLED_ShowString(98,2,(u8*)"BACK",1,1);
												}
												if(select == 2) select = 0;
											}
											if(Scan_Key() == 2){
												if(select == 1) break;
											}
										}
										menuType = DEBUG_WINDOW;
										select = 0;
										debugMode = 0;
										OLED_Clear();
										OLED_DrawBMP(0,0,32,32,DebugLogo);
										OLED_ShowString(45,-1,(u8*)"DEBUG MODE",1,0);
										OLED_ShowString(45,0,(u8*)"----------",1,0);
										OLED_ShowString(45,1,(u8*)debugBaud[debugSelect],1,1);
										OLED_ShowString(110,1,(u8*)">>",1,1);
										OLED_ShowString(92,2,(u8*)"ENTER",1,1);
										break;
								}
						}
						if(breakDebug){
							breakDebug = 0;
							break;
						}
					}
					break;
				case 3: //进行烧写
					if(strstr(FileInfo.lfname,".hex")) { //HEX文件烧写模式
						menuType = HEX_WINDOW;
						OLED_Clear();
		        OLED_DrawBMP(0,0,33,33,FlashLogo);
						if(f_open(&fnew, (const TCHAR*)FileInfo.lfname,FA_READ ) == FR_OK){
		          OLED_ShowString(45,-1,(u8*)"HEX  FLASH",1,0);
		          while(!FLASH_HEX_SWD((u8*)FileInfo.lfname)){
		            u8 WaitTips[] = "...";
			          OLED_ShowString(45,1,(u8*)"          ",1,0);
			          OLED_ShowString(45,2,(u8*)"WAIT",1,1);
			          for(i=0;i<3;i++){
                  OLED_ShowChar(69+i*6,2,WaitTips[i],1);
	                delay_ms(200);
	              }
			          OLED_ShowString(45,2,(u8*)"       ",1,1);
		          }
			        OLED_ShowString(98,2,(u8*)"BACK",1,1);
			        select = 0;
			        while(1)
			        {
				        if(Scan_Key() == 1){
					        select ++;
					        if(select == 1) {
						        OLED_ShowString(98,2,(u8*)"    ",1,1);
						        OLED_ShowString(98,2,(u8*)"BACK",0,1);
					        }
					        else{
					        	OLED_ShowString(98,2,(u8*)"    ",1,1);
						        OLED_ShowString(98,2,(u8*)"BACK",1,1);
					        }
					         if(select == 2) select = 0;
				        }
				        if(Scan_Key() == 2){
					        if(select == 1) break;
				        }
			        }
			        OLED_Clear();
		          OLED_DrawBMP(0,0,33,33,FlashLogo);
							if(f_opendir(&DirInfo,(const TCHAR*)"0:") == FR_OK)/* 打开文件夹目录成功，目录信息已经在dir结构体中保存 */
              {
                if(f_readdir(&DirInfo, &FileInfo) == FR_OK)  /* 读文件信息到文件状态结构体中 */
								{
									f_readdir(&DirInfo, &FileInfo);
									int fileLen = strlen(FileInfo.lfname);
									if(fileLen != 0 || strlen(FileInfo.fname)){
										if(fileLen < 7) OLED_ShowString(45,1,(u8*)FileInfo.fname,1,1);
										else{
											char fileShortName[7] = {0};
											strncpy(fileShortName, FileInfo.lfname, 7);
											OLED_ShowString(45,1,(u8*)fileShortName,1,1);
										}
									}
									else OLED_ShowString(45,1,(u8*)"NULL",1,1);
								}
	            }		
							menuType = MAIN_WINDOW;
	            OLED_ShowString(45,-1,(u8*)"SELECT HEX",1,0);
	            OLED_ShowString(45,0,(u8*)"----------",1,0);
	            OLED_ShowString(110,1,(u8*)">>",1,1);
	            OLED_ShowString(92,2,(u8*)"FLASH",1,1);
	            select = 0;
		        }
					}
					else if(strstr(FileInfo.fname,".hex")) { //短HEX文件烧写模式
						menuType = HEX_WINDOW;
						OLED_Clear();
		        OLED_DrawBMP(0,0,33,33,FlashLogo);
						if(f_open(&fnew, (const TCHAR*)FileInfo.fname,FA_READ ) == FR_OK){
		          OLED_ShowString(45,-1,(u8*)"HEX  FLASH",1,0);
		          while(!FLASH_HEX_SWD((u8*)FileInfo.fname)){
		            u8 WaitTips[] = "...";
			          OLED_ShowString(45,1,(u8*)"          ",1,0);
			          OLED_ShowString(45,2,(u8*)"WAIT",1,1);
			          for(i=0;i<3;i++){
                  OLED_ShowChar(69+i*6,2,WaitTips[i],1);
	                delay_ms(200);
	              }
			          OLED_ShowString(45,2,(u8*)"       ",1,1);
		          }
			        OLED_ShowString(98,2,(u8*)"BACK",1,1);
			        select = 0;
			        while(1)
			        {
				        if(Scan_Key() == 1){
					        select ++;
					        if(select == 1) {
						        OLED_ShowString(98,2,(u8*)"    ",1,1);
						        OLED_ShowString(98,2,(u8*)"BACK",0,1);
					        }
					        else{
					        	OLED_ShowString(98,2,(u8*)"    ",1,1);
						        OLED_ShowString(98,2,(u8*)"BACK",1,1);
					        }
					         if(select == 2) select = 0;
				        }
				        if(Scan_Key() == 2){
					        if(select == 1) break;
				        }
			        }
			        OLED_Clear();
		          OLED_DrawBMP(0,0,33,33,FlashLogo);
							if(f_opendir(&DirInfo,(const TCHAR*)"0:") == FR_OK)/* 打开文件夹目录成功，目录信息已经在dir结构体中保存 */
              {
                if(f_readdir(&DirInfo, &FileInfo) == FR_OK)  /* 读文件信息到文件状态结构体中 */
								{
									f_readdir(&DirInfo, &FileInfo);
									int fileLen = strlen(FileInfo.lfname);
									if(fileLen != 0 || strlen(FileInfo.fname)){
										if(fileLen < 7) OLED_ShowString(45,1,(u8*)FileInfo.fname,1,1);
										else{
											char fileShortName[7] = {0};
											strncpy(fileShortName, FileInfo.lfname, 7);
											OLED_ShowString(45,1,(u8*)fileShortName,1,1);
										}
									}
									else OLED_ShowString(45,1,(u8*)"NULL",1,1);
								}
	            }		
							menuType = MAIN_WINDOW;
	            OLED_ShowString(45,-1,(u8*)"SELECT HEX",1,0);
	            OLED_ShowString(45,0,(u8*)"----------",1,0);
	            OLED_ShowString(110,1,(u8*)">>",1,1);
	            OLED_ShowString(92,2,(u8*)"FLASH",1,1);
	            select = 0;
		        }
					}
				  else if(strstr(FileInfo.lfname,".bin")) { //BIN文件烧写模式
						menuType = BIN_WINDOW;
						OLED_Clear();
		        OLED_DrawBMP(0,0,33,33,FlashLogo);
						if(f_open(&fnew, (const TCHAR*)FileInfo.lfname,FA_READ ) == FR_OK){
		          OLED_ShowString(45,-1,(u8*)"BIN  FLASH",1,0);
		          while(!FLASH_SWD((u8*)FileInfo.lfname)){
		            u8 WaitTips[] = "...";
			          OLED_ShowString(45,1,(u8*)"          ",1,0);
			          OLED_ShowString(45,2,(u8*)"WAIT",1,1);
			          for(i=0;i<3;i++){
                  OLED_ShowChar(69+i*6,2,WaitTips[i],1);
	                delay_ms(200);
	              }
			          OLED_ShowString(45,2,(u8*)"       ",1,1);
		          }
			        OLED_ShowString(98,2,(u8*)"BACK",1,1);
			        select = 0;
			        while(1)
			        {
				        if(Scan_Key() == 1){
					        select ++;
					        if(select == 1) {
						        OLED_ShowString(98,2,(u8*)"    ",1,1);
						        OLED_ShowString(98,2,(u8*)"BACK",0,1);
					        }
					        else{
					        	OLED_ShowString(98,2,(u8*)"    ",1,1);
						        OLED_ShowString(98,2,(u8*)"BACK",1,1);
					        }
					         if(select == 2) select = 0;
				        }
				        if(Scan_Key() == 2){
					        if(select == 1) break;
				        }
			        }
			        OLED_Clear();
		          OLED_DrawBMP(0,0,33,33,FlashLogo);
							if(f_opendir(&DirInfo,(const TCHAR*)"0:") == FR_OK)/* 打开文件夹目录成功，目录信息已经在dir结构体中保存 */
              {
                if(f_readdir(&DirInfo, &FileInfo) == FR_OK)  /* 读文件信息到文件状态结构体中 */
								{
									f_readdir(&DirInfo, &FileInfo);
									int fileLen = strlen(FileInfo.lfname);
									if(fileLen != 0 || strlen(FileInfo.fname)){
										if(fileLen < 7) OLED_ShowString(45,1,(u8*)FileInfo.fname,1,1);
										else{
											char fileShortName[7] = {0};
											strncpy(fileShortName, FileInfo.lfname, 7);
											OLED_ShowString(45,1,(u8*)fileShortName,1,1);
										}
									}
									else OLED_ShowString(45,1,(u8*)"NULL",1,1);
								}
	            }		
							menuType = MAIN_WINDOW;
	            OLED_ShowString(45,-1,(u8*)"SELECT HEX",1,0);
	            OLED_ShowString(45,0,(u8*)"----------",1,0);
	            OLED_ShowString(110,1,(u8*)">>",1,1);
	            OLED_ShowString(92,2,(u8*)"FLASH",1,1);
	            select = 0;
		        }
					}
					else if(strstr(FileInfo.fname,".bin")) { //短BIN文件烧写模式
						menuType = BIN_WINDOW;
						OLED_Clear();
		        OLED_DrawBMP(0,0,33,33,FlashLogo);
						if(f_open(&fnew, (const TCHAR*)FileInfo.fname,FA_READ ) == FR_OK){
		          OLED_ShowString(45,-1,(u8*)"BIN  FLASH",1,0);
		          while(!FLASH_SWD((u8*)FileInfo.fname)){
		            u8 WaitTips[] = "...";
			          OLED_ShowString(45,1,(u8*)"          ",1,0);
			          OLED_ShowString(45,2,(u8*)"WAIT",1,1);
			          for(i=0;i<3;i++){
                  OLED_ShowChar(69+i*6,2,WaitTips[i],1);
	                delay_ms(200);
	              }
			          OLED_ShowString(45,2,(u8*)"       ",1,1);
		          }
			        OLED_ShowString(98,2,(u8*)"BACK",1,1);
			        select = 0;
			        while(1)
			        {
				        if(Scan_Key() == 1){
					        select ++;
					        if(select == 1) {
						        OLED_ShowString(98,2,(u8*)"    ",1,1);
						        OLED_ShowString(98,2,(u8*)"BACK",0,1);
					        }
					        else{
					        	OLED_ShowString(98,2,(u8*)"    ",1,1);
						        OLED_ShowString(98,2,(u8*)"BACK",1,1);
					        }
					         if(select == 2) select = 0;
				        }
				        if(Scan_Key() == 2){
					        if(select == 1) break;
				        }
			        }
			        OLED_Clear();
		          OLED_DrawBMP(0,0,33,33,FlashLogo);
							if(f_opendir(&DirInfo,(const TCHAR*)"0:") == FR_OK)/* 打开文件夹目录成功，目录信息已经在dir结构体中保存 */
              {
                if(f_readdir(&DirInfo, &FileInfo) == FR_OK)  /* 读文件信息到文件状态结构体中 */
								{
									f_readdir(&DirInfo, &FileInfo);
									int fileLen = strlen(FileInfo.lfname);
									if(fileLen != 0 || strlen(FileInfo.fname)){
										if(fileLen < 7) OLED_ShowString(45,1,(u8*)FileInfo.fname,1,1);
										else{
											char fileShortName[7] = {0};
											strncpy(fileShortName, FileInfo.lfname, 7);
											OLED_ShowString(45,1,(u8*)fileShortName,1,1);
										}
									}
									else OLED_ShowString(45,1,(u8*)"NULL",1,1);
								}
	            }		
							menuType = MAIN_WINDOW;
	            OLED_ShowString(45,-1,(u8*)"SELECT HEX",1,0);
	            OLED_ShowString(45,0,(u8*)"----------",1,0);
	            OLED_ShowString(110,1,(u8*)">>",1,1);
	            OLED_ShowString(92,2,(u8*)"FLASH",1,1);
	            select = 0;
		        }
					}
					break;
			}
		}
	}
}

/***********************离线SWD烧写************************/
u8 FLASH_SWD(u8 *File)
{
	Res = f_open(&fnew, (const TCHAR *)File, FA_READ);
	if (Res != FR_OK)
	{
		return 0;
	}
	readflag = 1;
	addr = 0;
	if (!swd_init_debug())
	{
		f_close(&fnew);
		return 0;
	}
	if (target_opt_init() != ERROR_SUCCESS)
	{
		f_close(&fnew);
		return 0;
	}
	if (target_opt_erase_chip() != ERROR_SUCCESS)
	{
		f_close(&fnew);
		return 0;
	}

	target_opt_uninit();
	if (!swd_init_debug())
	{
		return 0;
	}
	if (target_flash_init(0x08000000) != ERROR_SUCCESS)
	{
		target_flash_uninit();
		return 0;
	}
	if (target_flash_erase_chip() != ERROR_SUCCESS)
	{
		f_close(&fnew);
		return 0;
	}
	while (1)
	{
		Res = f_read(&fnew, rData, 1024, (void *)&bytesread);
		if ((Res != FR_OK) || (bytesread == 0))
		{
			break;
		}
		if (target_flash_program_page(0x08000000 + addr, (u8 *)&rData[0], 1024) != ERROR_SUCCESS)
		{
			f_close(&fnew);
			return 0;
		}
		u32 progess = (((double)addr / f_size(&fnew)) * 100);
		if (progess % 10 == 0)
		{
			char temp[16];
			memset(temp, '=', progess % 10);
			temp[progess % 10 + 1] = 0;
			OLED_ShowString(45, 1, temp, 1, 0);
			snprintf(temp, 15, "%d%%", progess % 10);
			OLED_ShowString(45, 2, temp, 1, 0);

			DEBUG_LED = !DEBUG_LED;
		}
		addr += 1024;
	}
	f_close(&fnew);
	if (!swd_init_debug())
	{
		return 0;
	}
	DEBUG_LED = 1;
	swd_set_target_reset(0); //复位运行
	delay_ms(100);
	OLED_ShowString(45, 1, (u8 *)"==========", 1, 0);
	OLED_ShowString(45, 2, (u8 *)"DONE", 1, 0);
	return 1;
}


u8 FLASH_HEX_SWD(u8 *File){
	char hexread[64];
	uint32_t nowSize = 0;
	Res = f_open(&fnew, (const TCHAR *)File, FA_READ);
	if (Res != FR_OK)
	{
		return 0;
	}
	//Res OK
	addr = 0;
	if (!swd_init_debug())
	{
		f_close(&fnew);
		return 0;
	}
	if (target_opt_init() != ERROR_SUCCESS)
	{
		f_close(&fnew);
		return 0;
	}
	if (target_opt_erase_chip() != ERROR_SUCCESS)
	{
		f_close(&fnew);
		return 0;
	}

	target_opt_uninit();
	if (!swd_init_debug())
	{
		f_close(&fnew);
		return 0;
	}
	if (target_flash_init(0x08000000) != ERROR_SUCCESS)
	{
		target_flash_uninit();
		f_close(&fnew);
		return 0;
	}
	if (target_flash_erase_chip() != ERROR_SUCCESS)
	{
		f_close(&fnew);
		return 0;
	}
	int add_off = 0;
	char head_data[16] = {0};
	while (f_gets(hexread, 64, &fnew) != NULL)
	{
		if (hexread[0] != ':')
		{
			return -2;
			f_close(&fnew);
		}
		else
		{
			nowSize += strlen(hexread);
			uint64_t offset = Char2toByte(&hexread[3]) * 256 + Char2toByte(&hexread[5]) + add_off; //偏移量
			uint8_t type = Char2toByte(&hexread[7]);											   //类型
			uint8_t datalen = Char2toByte(&hexread[1]);											   //当前行数据长度

			if (type == 4) //当类型为4时添加偏移量
				add_off = Char2toByte(&hexread[11]) * 0x10000;
			else if (type == 0) //当类型为1时为数据
			{
				if (offset == 0 || offset % 1024 != 0)
				{
					for (i = 0; i < datalen; i++)
						rData[i + (offset - ((offset / 1024) * 1024))] = Char2toByte(&hexread[9 + 2 * i]);
				}
				else if (offset % 1024 == 0)
				{
					for (i = 0; i < datalen; i++)
						head_data[i] = Char2toByte(&hexread[9 + 2 * i]);
				}
				if (offset >= 1024 && offset % 1024 == 0)
				{
					if (target_flash_program_page(0x08000000 + addr, (u8 *)&rData[0], 1024) != ERROR_SUCCESS)
					{
						f_close(&fnew);
						return 0;
					}
					u32 progess = (((double)nowSize / f_size(&fnew)) * 100);
					if (progess % 10 == 0)
					{
						char temp[16];
						memset(temp, '=', progess % 10);
						temp[progess % 10 + 1] = 0;
						OLED_ShowString(45, 1, temp, 1, 0);
						snprintf(temp, 15, "%d%%", progess % 10);
						OLED_ShowString(45, 2, temp, 1, 0);

						DEBUG_LED = !DEBUG_LED;
					}

					memset(rData, 0xFF, sizeof(rData));
					for (i = 0; i < datalen; i++)
						rData[i] = head_data[i];

					addr += 1024;
				}
			}
		}
	}
	f_close(&fnew);
	if (target_flash_program_page(0x08000000 + addr, (u8 *)&rData[0], 1024) != ERROR_SUCCESS)
		return 0;
	if (!swd_init_debug())
	{
		return 0;
	}
	DEBUG_LED = 1;
	swd_set_target_reset(0); //复位运行
	delay_ms(100);
	OLED_ShowString(45, 1, (u8 *)"==========", 1, 0);
	OLED_ShowString(45, 2, (u8 *)"DONE", 1, 0);
	return 1;
}
/***********************按键监测************************/
u8 Scan_Key(){
	if(!SELECT){
		delay_ms(35);
		if(!SELECT){
			while(SELECT == !SELECT);
			delay_ms(35);
			return 1;
		}
	}
	if(!OK){
		delay_ms(35);
		if(!OK){
			while(OK == !OK);
			delay_ms(35);
			return 2;
		}
	}
	return 0;
}


