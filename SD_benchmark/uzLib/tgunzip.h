/**
  ******************************************************************************
  * @file    tgunzip.h
  * @author  MCheah
  * @version
  * @date    Sept 30, 2018
  * @brief   Header for embedded uZunzip
  * @note
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#ifndef __TGUNZIP_H
#define __TGUNZIP_H

int read_byte(struct uzlib_uncomp *uncomp);
int decompress_gzip(char *fname_in, char *fname_out);

#endif //
