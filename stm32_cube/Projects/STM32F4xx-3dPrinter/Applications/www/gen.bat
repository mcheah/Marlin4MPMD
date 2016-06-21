@echo off
cls
echo *************************************
echo * Script to create an image file
echo * for AT+S.HTTPDFSUPDATE command
echo * 
echo * only for internal use - made in SPG
echo *
echo *
echo * USAGE: create a dir named "pages",
echo * and put your html code inside that
echo *************************************


cd pages
If exist outfile.img del outfile.img

..\httpd_gen.exe outfile.img *
echo *
echo *
echo *
echo * DONE!
echo * 
echo * Press a key to exit...
pause>nul