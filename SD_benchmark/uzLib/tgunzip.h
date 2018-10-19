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
#ifdef   __cplusplus
extern "C"
{
#endif
#define BLAHH
int read_byte(struct uzlib_uncomp *uncomp);
int decompress_gzip(char *fname_in, char *fname_out);
#ifdef   __cplusplus
}
#endif
#endif //
