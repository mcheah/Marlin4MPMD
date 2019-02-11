/**
  ******************************************************************************
  * @file    cardreader.h
  * @author
  * @version
  * @date    July 7, 2015
  * @brief   Header for cardreader module of Marlin 3D printer
  * @note
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CARDREADER_H
#define CARDREADER_H

#if ENABLED(SDSUPPORT)

#include "Marlin.h"
#include "ultralcd.h"
#include "stepper.h"
#include "temperature.h"
#include "language.h"
#include "configuration_store.h"
#include "diskio.h"

#include "ff.h"  //for FATFS

#include <ctype.h> //for call to tolower function


/* Exported Constants --------------------------------------------------------*/

/** @defgroup cardreader_Exported_Constants
  * @{
  */

// Maximum directory depth
#define MAX_DIR_DEPTH (1)

#if _USE_LFN != 0
// We're going to "cheat" and assume that Path+LFN is less than MAX_LFN
// We'll need to add protection to truncate strcat calls to do this.
#define MAXPATHNAMELENGTH		(_MAX_LFN)
#define FILENAME_LENGTH 		(_MAX_LFN)
#define LONG_FILENAME_LENGTH 	(_MAX_LFN)
#else
// Maximum length of path
#define MAXPATHNAMELENGTH (13*MAX_DIR_DEPTH+MAX_DIR_DEPTH+1)

// Maximum number of files displayed
#define MAX_FILES (24) //MAX_FILES*LONG_FILENAME_LENGTH < 4096

// Number of UTF-16 characters per entry
#define FILENAME_LENGTH (13)

// Number of VFAT entries used. Every entry has 13 UTF-16 characters
//#define MAX_VFAT_ENTRIES (2)

// Total size of the buffer used to store the long filenames
//#define LONG_FILENAME_LENGTH (FILENAME_LENGTH*MAX_VFAT_ENTRIES+1)
#define LONG_FILENAME_LENGTH (FILENAME_LENGTH+MAXPATHNAMELENGTH)
#endif

// Number of possible sub-function call (call of a G file by another one)
#define SD_PROCEDURE_DEPTH (1)

/**
  * @}
  */



/* Exported Types  -------------------------------------------------------*/

/** @defgroup cardreader_Exported_Types
  * @{
  */


/**
  * @}
  */

/** \class CardReader
 * Object used to manage SD card.
 */
class CardReader
{
public:

	// Public varaibles
	//------------------

	bool saving;
	bool logging;
	bool sdprinting ;
	bool updateLCD;

	bool cardReaderInitialized;
	bool cardOK ;
	bool rootIsOpened;
#if _USE_LFN != 0
	char filename[LONG_FILENAME_LENGTH+1];
#else
	char filename[13];
#endif
	char longFilename[LONG_FILENAME_LENGTH+1];
	bool filenameIsDir;
	int lastnr; //last number of the autostart;

	unsigned long autostart_atmillis;

	// Public methods
	//----------------

	/**
	 * \fn CardReader()
	 * \brief Constructor of the class CardReader, used to initialized internal variables.
	 */
	CardReader();
  

	/**
	 * \fn void initsd()
	 * \brief Function used at startup to init and mount the SD card.
	 */
	void initsd();


	/**
	 * \fn void write_command(char *buf)
	 * \brief Function used to write a command, stored in buf, in the current open file.
	 *
	 * \param buf  Pointer to a char buffer.
	 */
	void write_command(char *buf);

	/**
	 * \fn void checkautostart(bool x)
	 * \brief Try to find an autostart file (file with name auto[0-9].g) on the sd card.
	 * \brief If present, files are executed in a row.
	 *  in the directory, execute it..
	 *	If not already done, this method will performed the tnitialisaiton of the sd card.
	 *
	 * \param force  Boolean that indicate if the autostart check shall be forced or not.
	 *
	 */
	void checkautostart(bool x);

	/**
	 * \fn void openFile(char* name,bool read,bool replace_current=true)
	 * \brief Function used to write a command, stored in buf, in the current open file.
	 *
	 * \param name  Name of the file to open.
	 * \param read  Indicates if the file shall be or not opened in read only mode..
	 * \param replace_current To indicate if the file indicated as parameter shall be used in replacement of the current one.
	 *                        Otherwize, it will be a sub call.
	 *                        Default value: True
	 */
	void openFile(char* name,bool read,bool replace_current=true);

	/**
	 * \fn void openLogFile(char* name)
	 * \brief Function used to create and open a log file in write mode, with the name specified as parameter.
	 *
	 * \param name  Name of the logfile.
	 */

	/**
	 * \fn void openAndPrintFile(char* name)
	 * \brief Function used to open a file and start a print
	 *
	 * \param name  Name of the file to open.
	 */
	void openAndPrintFile(const char *name);

	void openLogFile(char* name);

	/**
	 * \fn void removeFile(char* name)
	 * \brief Function used to remove a file from the SD card.
	 *
	 * \param name  Name of the file to remove.
	 */
	void removeFile(char* name);

	/**
	 * \fn void closefile(bool store_location=false)
	 * \brief Function used to close a file and so, to stop the printing. The user can save the location in the file, in order to continue later the print action from this position.
	 *
	 * \param store_location	If true, the printer state (filename + position) are saved for continuing later the print action.
	 */
	void closefile(bool store_location=false);

	/**
	 * \fn void release()
	 * \brief Function used to release the SD card, in order to extract it.
	 */
	void release();

	/**
	 * \fn void startFileprint()
	 * \brief Function used to start the printing action.
	 */
	void startFileprint();

	/**
	 * \fn void pauseSDPrint()
	 * \brief Function used to pause the printing action.
	 */
	void pauseSDPrint();

	/**
	 * \fn void getStatus()
	 * \brief Function used to status of the printing action.
	 */
	void getStatus();

	uint16_t get_num_Files();
	void printingHasFinished();

	void getfilename(uint16_t nr, const char* const match=NULL);
	uint16_t getnrfilenames();
  
	void getAbsFilename(char *t);
  

	void ls();
	void chdir(const char * relpath);
	void updir();
	void setroot();

	FORCE_INLINE uint32_t fileLength() { return filesize; }
	FORCE_INLINE bool isFileOpen() { return (bool)file.obj.fs; }
	FORCE_INLINE bool eof() { return sdpos>=filesize ;};
	FORCE_INLINE int16_t get() {
		BYTE readByte;
		UINT rc;
		if (f_read(&file, &readByte, 1, &rc) != FR_OK)                     {
			readByte = -1;
		}
		else
		{
			sdpos += rc;
		}
		return (int16_t) readByte;
	};
	FORCE_INLINE void setIndex(long index) {sdpos = index;f_lseek(&file, index);};
	FORCE_INLINE uint8_t percentDone(){
		if(!isFileOpen())
			return 0;
		if(filesize)
			if(sdpos>=filesize)
				return 100;
			else
				return sdpos/((filesize+99)/100);
		else return 0;
	};

#if (!MB(STM_3DPRINT))
	FORCE_INLINE char* getWorkDirName(){workDir.getFilename(filename);return filename;};
#endif
  
private:

	// Private variables
	//-------------------

	uint16_t workDirDepth;
	DIR root, *curDir, workDir, workDirParents[MAX_DIR_DEPTH];
	FIL file;  // Current file
	FATFS fileSystem;
	char SDPath[4]; /* SD card logical drive path */
	uint8_t fileOpened[SD_PROCEDURE_DEPTH];
	uint8_t file_subcall_ctr;
	uint32_t filespos[SD_PROCEDURE_DEPTH];
	char filenames[SD_PROCEDURE_DEPTH][MAXPATHNAMELENGTH];
	uint32_t filesize;
	//int16_t n;
	uint32_t sdpos ;

	// This variable is used to determine if the autostart have to be check or not in the function checkautostart()
	bool autostart_stilltocheck; //the sd start is delayed, because otherwise the serial cannot answer fast enought to make contact with the hostsoftware.

	LsAction lsAction; //stored for recursion.
	int16_t nrFiles; //counter for the files in the current directory and recycled as position counter for getting the nrFiles'th name in the directory.
	char* diveDirName;

	// Private methods
	//-----------------

	void lsDive(const char *prepend, DIR *parent, const char * const match=NULL);
	bool testPath( char *name, char **fname);


};


	extern CardReader card;
	extern CardReader *p_card;
	#define IS_SD_PRINTING (card.sdprinting)

#if (SDCARDDETECT > -1)
# ifdef SDCARDDETECTINVERTED 
#  define IS_SD_INSERTED (READ(SDCARDDETECT)!=0)
# else
#  define IS_SD_INSERTED (READ(SDCARDDETECT)==0)
# endif //SDCARDTETECTINVERTED
#else
//If we don't have a card detect line, aways asume the card is inserted
# define IS_SD_INSERTED true
#endif


#endif

#endif
