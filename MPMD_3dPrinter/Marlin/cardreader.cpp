
#include "Marlin.h"
#include "cardreader.h"
#include <strings.h>

#if ENABLED(SDSUPPORT)
static unsigned char readBuff[512];

CardReader::CardReader()
{
   uint32_t loop;
  
   filesize = 0;
   sdpos = 0;
   sdprinting = false;
   updateLCD = false;
   cardOK = false;
   rootIsOpened = false;
   saving = false;
   logging = false;
   workDirDepth = 0;
   file_subcall_ctr=0;
   cardReaderInitialized = false;
   isBinaryMode = false;
   pReadEnd = readBuff+512;
   pRead = pReadEnd;

   curDir = NULL;
   diveDirName = NULL;
   filenameIsDir = false;
   lsAction = LS_SerialPrint;
   nrFiles = 0;

   memset(workDirParents, 0, sizeof(workDirParents));
   for(loop = 0; loop < SD_PROCEDURE_DEPTH; loop++)
   { 
     fileOpened[loop] = 0; 
   }   

   memset(&fileSystem, 0, sizeof(FATFS));

   // Variable dedicated to autostart on SD
   autostart_stilltocheck=true; //the SD start is delayed, because otherwise the serial cannot answer fast enough to make contact with the host software.
   autostart_atmillis=0;

   lastnr=0;
  //power to SD reader
  #if SDPOWER > -1
    SET_OUTPUT(SDPOWER); 
    WRITE(SDPOWER,HIGH);
  #endif //SDPOWER
  
  autostart_atmillis=millis()+5000;

  // Activate the SD card detection
}

void CardReader::initsd()
{
  for(int i=0;i<2;i++) //Attempt two times in case card need released first
  {
	  cardOK = false;

	  // Set SD path
	  strcpy( SDPath, "/");
	  FRESULT code = FR_OK;
	  uint8_t buff[80]="";
	  if (rootIsOpened == false)
	  {
		//BSP_SD_DetectInit();

		if (BSP_SD_IsDetected() == 0)
		{
		  SERIAL_ECHO_START;
		  SERIAL_ECHOLNPGM(MSG_SD_INIT_FAIL);
		}
		else if ((code = f_mount(&fileSystem, (TCHAR const*)SDPath, 0)) != FR_OK)
		{
		  SERIAL_ERROR_START;
		  SERIAL_ERRORLNPGM(MSG_SD_VOL_INIT_FAIL);
		  sprintf((char *)buff,"code=%d\n",code);
		  MYSERIAL.print((char *)buff);
		  release();
		  return;//Previously uninitialized, don't try again
		}
		else if ((code = f_opendir(&root,SDPath)) != FR_OK)
		{
		  SERIAL_ERROR_START;
		  SERIAL_ERRORLNPGM(MSG_SD_OPENROOT_FAIL);
		  sprintf((char *)buff,"code=%d\n",code);
		  MYSERIAL.print((char *)buff);
		  release();
		  return;//Previously uninitialized, don't try again
		}
		else
		{
		  cardOK = true;
		  rootIsOpened = true;
		  cardReaderInitialized = true;
		  SERIAL_ECHO_START;
		  SERIAL_ECHOLNPGM(MSG_SD_CARD_OK);
		}
		workDir=root;
		curDir=&root;
	  }
	  else
	  {
		DIR testroot;
		FILINFO testnfo;
		FRESULT res = f_opendir(&testroot,SDPath);
//		res = f_readdir(&testroot,&testnfo);
//		uint8_t code = BSP_SD_GetStatus();
		if(res!=FR_OK/* || code!= BSP_SD_OK*/) {
			  SERIAL_ERROR_START;
			  SERIAL_ERRORLNPGM(MSG_SD_OPENROOT_FAIL);
			  sprintf((char *)buff,"code=%d\n",res);
			  MYSERIAL.print((char *)buff);
			  release(); //test failed, loop again after releasing
		}
		else {
			f_closedir(&testroot);
			cardOK = true;
		}
	  }
	  if(cardOK)
		return; //Success, don't loop again
  }
}



char *createFilename(char *buffer,FILINFO *pEntry) //buffer>12characters
{
  char *pos=buffer;

  for (uint8_t i = 0; i < 11; i++) 
  {
    if (pEntry->fname[i] == ' ')continue;
    if (i == 8) 
    {
      *pos++='.';
    }
    *pos++= pEntry->fname[i];
  }
  *pos++=0;
  return buffer;
}

void CardReader::lsDive(const char *prepend, DIR *parent, const char * const match/*=NULL*/)
{
  FILINFO entry;
  uint8_t cnt=0;
 
  if(!cardOK)
	  return;

  //while (f_readdir(parent,&entry) == FR_OK) 
  while ((f_readdir(parent,&entry) == FR_OK)&&(entry.fname[0] != '\0')) 
  {
    if ((entry.fattrib & AM_DIR) && (lsAction != LS_Count) && (lsAction != LS_GetFilename)) // hence LS_SerialPrint
    {
      DIR subDir;
      char path[MAXPATHNAMELENGTH];
      char lfilename[FILENAME_LENGTH];
      strcpy(lfilename,entry.altname);
      path[0]=0;
      if(strlen(prepend)==0) //avoid leading / if already in prepend
      {
       strcat(path,"/");
      }
      strcat(path,prepend);
      strcat(path,lfilename);
      
      if (f_opendir(&subDir,path) != FR_OK) 
      {
        if(lsAction==LS_SerialPrint)
        {
          SERIAL_ECHO_START;
          SERIAL_ECHOLN(MSG_SD_CANT_OPEN_SUBDIR);
          SERIAL_ECHOLN(path);
        }
      }
	  //Do not include hidden folders in the listing
      else if(entry.fname[0]!='.' && (entry.fattrib & (AM_HID | AM_SYS)) == 0)
      {
        strcat(path,"/");
        lsDive(path,&subDir);  
        //close done automatically by destructor of SdFile
      }
    }
    else
    {
      char fn0 = entry.fname[0];

      filenameIsDir=(entry.fattrib & AM_DIR) ;

      if (fn0 == 0X00) break; //DIR_NAME_FREE 0X00

      if (fn0 == 0XE5 || fn0 == '.' || fn0 == '_') continue;  //DIR_NAME_DELETED 0XE5
      
      if(!filenameIsDir)
      {
        char *ptr = strchr(entry.fname, '.');
        if(ptr==NULL)
        {
          continue;
        }
      }

      if(lsAction==LS_SerialPrint)
      {
        char pathAndFilename[LONG_FILENAME_LENGTH] = "\n";
        strcat(pathAndFilename, prepend);
        strcat(pathAndFilename, entry.fname);
        strcat(pathAndFilename, "\n");
        SERIAL_PROTOCOL_N((uint8_t *)pathAndFilename, strlen(pathAndFilename));
      }
      else if(lsAction==LS_Count)
      {
        nrFiles++;
      } 
      else if(lsAction==LS_GetFilename)
      {
    	strcpy(filename,entry.fname);
    	strcpy(longFilename, prepend);
    	strcat(longFilename, entry.fname);
        if (match != NULL) {
          if (strcasecmp(match, entry.fname) == 0) return;
        }
        else if (cnt == nrFiles) return;
        cnt++;
      }
    }
  }
}


bool CardReader::testPath( char *name, char **fname)
{
	DIR myDir;
	curDir=&root;
	char *dirname_start,*dirname_end;

	if( !cardOK)
		return false;

	if(name[0]=='/')
	{
		dirname_start=strchr(name,'/')+1;
	    while(dirname_start>0)
	    {
	    	dirname_end=strchr(dirname_start,'/');
	    	//SERIAL_ECHO("start:");SERIAL_ECHOLN((int)(dirname_start-name));
	    	//SERIAL_ECHO("end  :");SERIAL_ECHOLN((int)(dirname_end-name));
	    	if(dirname_end>0 && dirname_end>dirname_start)
	    	{
	    		char subdirname[13];
	    		strncpy(subdirname, dirname_start, dirname_end-dirname_start);
	    		subdirname[dirname_end-dirname_start]=0;
	    		SERIAL_ECHOLN(subdirname);
	    		if (f_opendir(&myDir,subdirname) != FR_OK)
	    		{
	    			SERIAL_PROTOCOLPGM("open failed, File: ");
	    			SERIAL_PROTOCOL(subdirname);
	    			SERIAL_PROTOCOLLNPGM(".");
	    			return false;
	    		}
	    		else
	    		{
	    			//SERIAL_ECHOLN("dive ok");
	    		}

	    		curDir=&myDir;
	    		dirname_start=dirname_end+1;
	    	}
	    	else // the reminder after all /fsa/fdsa/ is the filename
	    	{
	    		*fname=dirname_start;
	    		//SERIAL_ECHOLN("remaider");
	    		//SERIAL_ECHOLN(fname);
	    		break;
	    	}
	    }
	}
	else //relative path
	{
		curDir=&workDir;
		*fname = name;
	}

	return true;
}


void CardReader::ls() 
{
	initsd(); //test card
	if(!cardOK)
		return;
	lsAction=LS_SerialPrint;
 /*   --  BDI : Quel int�ret?
  if(lsAction==LS_Count)
  nrFiles=0;
*/
	FRESULT res = f_readdir(&root,0);
	if(res==FR_OK)
		lsDive("",&root);
	else
		release();
}



void CardReader::setroot()
{
  /*if(!workDir.openRoot(&volume))
  {
    SERIAL_ECHOLNPGM(MSG_SD_WORKDIR_FAIL);
  }*/
  workDir=root;
  curDir=&root;
}

void CardReader::release()
{
  sdprinting = false;
  updateLCD = false;
  cardOK = false;
  cardReaderInitialized = false;
  rootIsOpened = false;
  disk_deinitialize(fileSystem.drv);
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM(MSG_SD_INIT_FAIL);
  if(isBinaryMode)
	  flush_buff();
}

void CardReader::startFileprint()
{
  initsd(); //test card
  if(!cardOK)
	  return;
//	if(sdpos!=0)
//	{
//	    enqueue_and_echo_command_now("G91");
//	    enqueue_and_echo_commands_P(PSTR("G1 Z-50 S1"));
//	    enqueue_and_echo_commands_P(PSTR("G90"));
//	}
#if ENABLED(MALYAN_LCD)
#if ENABLED(SD_SETTINGS)
  const char upper_config_file_name[] = UPPER_CONFIG_FILE_NAME;
  updateLCD = ( strncmp(longFilename,
		  	  upper_config_file_name,
			  sizeof(upper_config_file_name))!=0);
  if(updateLCD)
#else
  updateLCD = true;
#endif// ENABLED(SD_SETTINGS)
	lcd_setstatuspgm(PSTR(MSG_BUILD));
#endif// ENABLED(MALYAN_LCD)
  sdprinting = true;
}

void CardReader::pauseSDPrint()
{
  if(sdprinting)
  {
    sdprinting = false;
    //TODO: add behavior to correctly raise the print head but currently causes deadlock
//    enqueue_and_echo_command_now("M91");
//    enqueue_and_echo_commands_P(PSTR("G1 Z50 S1"));
  }
}

void CardReader::stopSDPrint() {
	  if(sdprinting)
	  {
	    sdprinting = false;
	    //TODO: add behavior to correctly raise the print head but currently causes deadlock
	//    enqueue_and_echo_command_now("M91");
	//    enqueue_and_echo_commands_P(PSTR("G1 Z50 S1"));
	    closefile();
	  }
}


void CardReader::openLogFile(char* name)
{
  initsd(); //test card
  if(!cardOK)
	  return;
  logging = true;
  openFile(name, false);
}

void CardReader::getAbsFilename(char *t)
{
#if 0
  FILINFO entry;
  uint8_t cnt=0;
  *t='/';t++;cnt++;
  
  /*
  for(uint8_t i=0;i<workDirDepth;i++)
  {
    FILINFO entry;
    get_fileinfo(workDirParents[i], &entry);
    strncpy(t, entry.fname, strlen(entry.fname));
      
    while(*t!=0 && cnt< MAXPATHNAMELENGTH) 
    {t++;cnt++;}  //crawl counter forward.
  }
  */
  if (f_getcwd (t, MAXPATHNAMELENGTH) == FR_OK)
  {
    cnt = strlen(t);
    get_fileinfo(file, &entry);
    if (cnt < MAXPATHNAMELENGTH-13)
    {
      strcat(t, entry.fname);
    }
    t[0]=0;
  }
  else
  {
    t[0]=0;
  }
#endif
    strncpy( t, longFilename, strlen(longFilename));
}

void CardReader::openFile(char* name,bool read, bool replace_current/*=true*/)
{
  initsd(); //test card
  if(!cardOK)
    return;
   
  // Test if a file or subfile (file_subcall_ctrl>0) is already opened.
  // In this case, two choices:
  //    - In case replace_current=true, the current file is closed and file_subcall_ctr is not incremented.
  //    - In case replace_current=false, the position is saved, the current file is closed and file_subcall_ctr is incremented.
  if(fileOpened[file_subcall_ctr])
  {
    if(!replace_current)
    {
    	if((int)file_subcall_ctr>(int)SD_PROCEDURE_DEPTH-1)
    	{
    		SERIAL_ERROR_START;
    		SERIAL_ERRORPGM("trying to call sub-gcode files with too many levels. MAX level is:");
    		SERIAL_ERRORLN(SD_PROCEDURE_DEPTH);
    		kill(PSTR(MSG_KILLED));
    		return;
    	}
     
    	SERIAL_ECHO_START;
    	SERIAL_ECHOPGM("SUBROUTINE CALL target:\"");
    	SERIAL_ECHO(name);
    	SERIAL_ECHOPGM("\" parent:\"");
     
    	//store current filename and position
    	getAbsFilename(filenames[file_subcall_ctr]);
     
    	SERIAL_ECHO(filenames[file_subcall_ctr]);
    	SERIAL_ECHOPGM("\" pos");
    	SERIAL_ECHOLN(sdpos);
    	filespos[file_subcall_ctr]=sdpos;
    	fileOpened[file_subcall_ctr] = 0;
    	file_subcall_ctr++;
    }
    else
    {
     SERIAL_ECHO_START;
     SERIAL_ECHOPGM("Now doing file: ");
     SERIAL_ECHOLN(name);
     fileOpened[file_subcall_ctr] = 0;
    }
    
    f_close(&file);
    
  }

  //
  // Opening fresh file
  //
  else
  {
    file_subcall_ctr=0; //resetting procedure depth in case user cancels print while in procedure
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("Now fresh file: ");
    SERIAL_ECHOLN(name);
  }
  sdprinting = false;

#if 0
  DIR myDir;
  curDir=&root;
  char *fname=name;
  char *dirname_start,*dirname_end;
  
  if(name[0]=='/')  // Case of absolute path
  {
    dirname_start=strchr(name,'/')+1;
    while(dirname_start>0)
    {
      dirname_end=strchr(dirname_start,'/');
      //SERIAL_ECHO("start:");SERIAL_ECHOLN((int)(dirname_start-name));
      //SERIAL_ECHO("end  :");SERIAL_ECHOLN((int)(dirname_end-name));
      if(dirname_end>0 && dirname_end>dirname_start)
      {
        char subdirname[13];
        strncpy(subdirname, dirname_start, dirname_end-dirname_start);
        subdirname[dirname_end-dirname_start]=0;
        SERIAL_ECHOLN(subdirname);
       if (f_opendir(&myDir,subdirname) != FR_OK)
        {
          SERIAL_PROTOCOLPGM(MSG_SD_OPEN_FILE_FAIL);
          SERIAL_PROTOCOL(subdirname);
          SERIAL_PROTOCOLLNPGM(".");
          return;
        }
        else
        {
          //SERIAL_ECHOLN("dive ok");
        }
          
        curDir=&myDir;
        dirname_start=dirname_end+1;
      }
      else // the reminder after all /fsa/fdsa/ is the filename
      {
        fname=dirname_start;
        //SERIAL_ECHOLN("remaider");
        //SERIAL_ECHOLN(fname);
        break;
      }
      
    }
  }
  else //relative path
  {
    curDir=&workDir;
  }
#endif

  char *fname=name;


  if( !testPath(name, &fname))
	  return;
  
  strncpy(longFilename, fname, strlen(fname));
  
  if(read)
  {
	  if (f_open(&file, name, FA_OPEN_EXISTING | FA_READ) == FR_OK)
    {
      fileOpened[file_subcall_ctr] = 1;
      filesize = f_size(&file);
      SERIAL_PROTOCOLPGM(MSG_SD_FILE_OPENED);
      SERIAL_PROTOCOL(fname);
      SERIAL_PROTOCOLPGM(MSG_SD_SIZE);
      SERIAL_PROTOCOLLN(filesize);
      sdpos = 0;
      
      SERIAL_PROTOCOLLNPGM(MSG_SD_FILE_SELECTED);
      getfilename(0, fname);
#if ENABLED(MALYAN_LCD)//TODO: it seems like Malyan ignores this, I thought it might put it on the status bar, but apparently not
      lcd_setstatuspgm(PSTR(MSG_PRINTFILE));
      lcd_setstatus(longFilename[0] ? longFilename : fname);
      lcd_setstatuspgm(PSTR("}"));
#else
      lcd_setstatus(longFilename[0] ? longFilename : fname);
#endif
    }
    else
    {
      SERIAL_PROTOCOLPGM(MSG_SD_OPEN_FILE_FAIL);
      SERIAL_PROTOCOL(fname);
      SERIAL_PROTOCOLLNPGM(".");
    }
  }
  else
  { //write
    if (f_open(&file, fname, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
    {
      SERIAL_PROTOCOLPGM(MSG_SD_OPEN_FILE_FAIL);
      SERIAL_PROTOCOL(fname);
      SERIAL_PROTOCOLLNPGM(".");
    }
    else
    {
      sdpos = 0;
      saving = true;
      SERIAL_PROTOCOLPGM(MSG_SD_WRITE_TO_FILE);
      SERIAL_PROTOCOLLN(name);
	  release();
#if DISABLED(MALYAN_LCD)
      lcd_setstatus(fname);
#endif
    }
  }
  if(isBinaryMode)
	flush_buff();
}

void CardReader::openAndPrintFile(const char *name) {
//  sdpos = 0;
  char cmd[4 + strlen(name) + 1]; // Room for "M23 ", filename, and null
  sprintf_P(cmd, PSTR("M23 %s"), name);
  for (char *c = &cmd[4]; *c; c++) *c = tolower(*c);
  enqueue_and_echo_command_now(cmd);
  enqueue_and_echo_commands_P(PSTR("M24"));
}

void CardReader::removeFile(char* name)
{
	char *fname=name;
	initsd();
	if(!cardOK)
		return;

	if( !testPath( name, &fname))
		return;

	if( !strncmp(name, filename, strlen(name)))
	{
		fileOpened[file_subcall_ctr] = 0;
		f_close(&file);
		sdprinting = false;
	}

	if( f_unlink (fname) == FR_OK)
	{
		SERIAL_PROTOCOLPGM("File deleted:");
		SERIAL_PROTOCOLLN(fname);
		sdpos = 0;
		if(isBinaryMode)
			flush_buff();
	}
	else
	{
		SERIAL_PROTOCOLPGM("Deletion failed, File: ");
		SERIAL_PROTOCOL(fname);
		SERIAL_PROTOCOLLNPGM(".");
	}
  
}

void CardReader::getStatus()
{
  if(cardOK){
    SERIAL_PROTOCOLPGM(MSG_SD_PRINTING_BYTE);
    SERIAL_PROTOCOL(sdpos);
    SERIAL_PROTOCOLPGM("/");
    SERIAL_PROTOCOLLN(filesize);
  }
  else{
    SERIAL_PROTOCOLLNPGM(MSG_SD_NOT_PRINTING);
  }
}

void CardReader::flush_buff() {
	pReadEnd = readBuff+512;
	pRead = pReadEnd;
}

int CardReader::read_buff(unsigned char *buf,uint32_t len)
{
	unsigned int bytesCopied = 0;
    unsigned int bytesRemaining = len;
	unsigned int bytesRead = -1;
	FRESULT readStatus;

	while(bytesCopied<len && bytesRead!=0) {
		if(pRead>=pReadEnd) {
			BSP_LED_On(LED_RED);
			BSP_LED_On(LED_GREEN);
			BSP_LED_On(LED_BLUE);
			readStatus = f_read(&file, readBuff, 512, &bytesRead);
		    if(readStatus != FR_OK)
		    {
			  SERIAL_ERROR_START;
			  SERIAL_ERRORLNPGM(MSG_SD_ERR_READ);
		    }
			BSP_LED_Off(LED_RED);
			BSP_LED_Off(LED_GREEN);
			BSP_LED_Off(LED_BLUE);
			pRead = readBuff;
			pReadEnd = readBuff +bytesRead;
			sdpos += bytesRead;
		}
		int bytesAvailable = pReadEnd - pRead;
		if(bytesAvailable>0) {
			unsigned int bytestoCopy = min(bytesAvailable,(int)bytesRemaining);
			memcpy(buf,pRead,bytestoCopy);
			bytesCopied+=bytestoCopy;
            pRead+=bytestoCopy;
            buf+=bytestoCopy;
            bytesRemaining-=bytestoCopy;
		}
	}
    return bytesCopied;
}

void CardReader::push_read_buff(int len)
{
    if((pRead-len)<readBuff) {
    	unsigned int bytesRead;
    	FSIZE_t curpos = f_tell(&file);
        f_lseek(&file,curpos-((pReadEnd)-pRead+len));
        sdpos-=((pReadEnd)-pRead+len);
        FRESULT readStatus = f_read(&file, readBuff, 512, &bytesRead);
	    if(readStatus != FR_OK)
	    {
		  SERIAL_ERROR_START;
		  SERIAL_ERRORLNPGM(MSG_SD_ERR_READ);
	    }
        // printf("Rewinding %ld Reading %d bytes, got %d bytes\n",pRead-readBuff-len,512,bytesRead);
        pRead = readBuff;
        pReadEnd = readBuff+bytesRead;
		sdpos += bytesRead;
    }
    else
        pRead = pRead-len;
}

void CardReader::write_buff(unsigned char *buf,uint32_t len)
{
  if(len==512)
	  BSP_LED_On(LED_RED);
  BSP_LED_On(LED_GREEN);
  BSP_LED_On(LED_BLUE);
  unsigned int bytesWritten;
  FRESULT writeStatus;

  writeStatus = f_write(&file, buf, len, &bytesWritten);
  sdpos+=bytesWritten;
  if( 	(writeStatus != FR_OK) ||
		(bytesWritten != len))
  {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_SD_ERR_WRITE_TO_FILE);
  }
  BSP_LED_Off(LED_GREEN);
  BSP_LED_Off(LED_BLUE);
  BSP_LED_Off(LED_RED);
}

void CardReader::write_command(char *buf)
{
  unsigned int lastBufferEntry;
  FRESULT writeStatus;
  char* begin = buf;
  char* npos = 0;
  char* end = buf + strlen(buf) - 1;

  if((npos = strchr(buf, 'N')) != NULL)
  {
    begin = strchr(npos, ' ') + 1;
    end = strchr(npos, '*') - 1;
  }
  end[1] = '\r';
  end[2] = '\n';
  
  writeStatus = f_write(&file, begin, &(end[2]) - begin + 1, &lastBufferEntry);
  if( 	(writeStatus != FR_OK) ||
		(lastBufferEntry != (unsigned int)(&(end[2]) - begin + 1)))
  {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_SD_ERR_WRITE_TO_FILE);
  }
}


void CardReader::checkautostart(bool force)
{
  if(!force)
  {
    if(!autostart_stilltocheck)
      return;
    if(autostart_atmillis>millis())
      return;
  }
  autostart_stilltocheck=false;

  initsd();
  if(!cardOK) //fail
	  return;
  
  char autoname[30];
  sprintf(autoname, PSTR("auto%i.g"), lastnr);
  for(int8_t i=0;i<(int8_t)strlen(autoname);i++)
    autoname[i]=tolower(autoname[i]);
  
  bool found=false;
 
  if (f_readdir(&root, 0) == FR_OK)    /* Rewind directory object */
  {
    FILINFO entry;
     
    while ((f_readdir(&root,&entry) == FR_OK)&&(entry.fname[0] != '\0')) 
	  {
      for(int8_t i=0;i<(int8_t)strlen((char*)entry.fname);i++)
      entry.fname[i]=tolower(entry.fname[i]);
      //Serial.print((char*)p.name);
      //Serial.print(" ");
      //Serial.println(autoname);
      if(entry.fname[9]!='~') //skip safety copies
      if(strncmp((char*)entry.fname,autoname,5)==0)
      {
        char cmd[30];

        sprintf(cmd, PSTR("M23 %s"), autoname);
        enqueue_and_echo_command(cmd);
        enqueue_and_echo_commands_P(PSTR("M24"));
        found=true;
      }
    }
  }
  else
	  release();
  if(!found)
    lastnr=-1;
  else
    lastnr++;
}

void CardReader::closefile(bool store_location)
{
  f_sync(&file);
  fileOpened[file_subcall_ctr] = 0;
  f_close(&file);
  saving = false; 
  logging = false;
  sdpos = 0;
  updateLCD = false;
  if(isBinaryMode)
	  flush_buff();
  if(store_location)
  {
    //future: store printer state, filename and position for continuing a stopped print
    // so one can unplug the printer and continue printing the next day.
    
  }

  
}

void CardReader::getfilename(uint16_t nr, const char * const match/*=NULL*/)
{
  curDir=&workDir;
  lsAction=LS_GetFilename;
  nrFiles=nr;
  f_readdir(curDir, 0); // rewind current directory
  lsDive("",curDir,match);
}

uint16_t CardReader::getnrfilenames()
{
  initsd();
  if(!cardOK)
	return 0;
  curDir=&workDir;
  lsAction=LS_Count;
  nrFiles=0;
  f_readdir(curDir, 0); // rewind current directory
  lsDive("",curDir);
  //SERIAL_ECHOLN(nrFiles);
  return nrFiles;
}

void CardReader::chdir(const char * relpath)
{
	DIR newDir;
	initsd(); //test card
	if(!cardOK)
		return;
  
	if (f_opendir(&newDir,relpath) != FR_OK)
	{
		SERIAL_ECHO_START;
		SERIAL_ECHOPGM(MSG_SD_CANT_ENTER_SUBDIR);
		SERIAL_ECHOLN(relpath);
		return;
	}
	else
	{
		if (workDirDepth < MAX_DIR_DEPTH) {
			for (int d = ++workDirDepth; d--;)
				workDirParents[d+1] = workDirParents[d];
			workDirParents[0]=*curDir;
		}
		workDir = newDir;
		curDir = &workDir;
	}
}

void CardReader::updir()
{
  if(workDirDepth > 0)
  {
    --workDirDepth;
    workDir = workDirParents[0];
    for (int d = 0; d < workDirDepth; d++)
      workDirParents[d] = workDirParents[d+1];
  }
}

uint16_t CardReader::get_num_Files() {
  return getnrfilenames();
}


void CardReader::printingHasFinished()
{
    stepper.synchronize();
    if(file_subcall_ctr>0 && SD_PROCEDURE_DEPTH>1) //heading up to a parent file that called current as a procedure.
    {
      fileOpened[file_subcall_ctr] = 0;
      f_close(&file);
      file_subcall_ctr--;
      openFile(filenames[file_subcall_ctr],true,true);
      setIndex(filespos[file_subcall_ctr]);
      startFileprint();
    }
    else
    {
      // quickStop();   -- BDI : no more present in new version
      fileOpened[file_subcall_ctr] = 0;
      f_close(&file);
      sdprinting = false;
      if(SD_FINISHED_STEPPERRELEASE)
      {
          //finishAndDisableSteppers();
          enqueue_and_echo_commands_P(PSTR(SD_FINISHED_RELEASECOMMAND));
      }
      // autotempShutdown();  -- BDI : no more present in new version
#if defined (SD_SETTINGS)      
      if (strcasecmp(longFilename, CONFIG_FILE_NAME)==0)
      {
	  Config_PrintSettings(false);
      }
#endif
    }
}


#endif //SDSUPPORT
