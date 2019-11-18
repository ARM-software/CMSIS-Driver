:: Batch file for generating CMSIS-Driver pack
:: This batch file uses:
::    7-Zip for packaging
:: The generated pack and pdsc file are placed in folder %RELEASE_PATH% (../Local_Release)
@ECHO off

SETLOCAL

:: Tool path for zipping tool 7-Zip
SET ZIPPATH=C:\Program Files\7-Zip

:: These settings should be passed on to subprocesses as well
SET PATH=%ZIPPATH%;%DOXYGENPATH%;%MSCGENPATH%;%PATH%

:: Pack Path (where generated pack is stored)
SET RELEASE_PATH=..\Local_Release

:: Pack Vendor:
SET PACK_VENDOR=ARM

:: Pack Name:
SET PACK_NAME=CMSIS-Driver

:: Folder for Pack
SET PACK_FOLDER_LIST=Config Documentation ETH Flash I2C NAND SPI WiFi

:: Files in base folder for Pack
SET PACK_FILE_LIST=%PACK_VENDOR%.%PACK_NAME%.pdsc README.md LICENSE.txt

:: !!!!!!!!!!!!!!!!!
:: DO NOT EDIT BELOW
:: !!!!!!!!!!!!!!!!! 

:: Remove previous build
IF EXIST %RELEASE_PATH% (
  ECHO removing %RELEASE_PATH%
  RMDIR /Q /S  %RELEASE_PATH%
)

:: Create build output directory
MKDIR %RELEASE_PATH%

:: build doxygen documentation
PUSHD ..\DoxyGen
  call genDoc.bat
POPD  

:: copy file list for base folder
FOR %%A IN (%PACK_FILE_LIST%) DO (
  COPY .\..\%%A %RELEASE_PATH%\
)

:: copy directory list of pack
FOR %%A IN (%PACK_FOLDER_LIST%) DO (
  XCOPY /Q /S /Y .\..\%%A %RELEASE_PATH%\%%A\
)

:: Checking 
:: Silencing warnings that are irrelevant in the context (M324, M382, M363)
Win32\PackChk.exe %RELEASE_PATH%\%PACK_VENDOR%.%PACK_NAME%.pdsc -n %RELEASE_PATH%\PackName.txt -x M353 -x M364 -x M324 -x M382 -x M363 -x M362

:: --Check if PackChk.exe has completed successfully
IF %errorlevel% neq 0 GOTO ErrPackChk

:: Packing 
PUSHD %RELEASE_PATH%

:: -- Pipe Pack's Name into Variable
SET /P PackName=<PackName.txt
DEL /Q PackName.txt

:: Pack files
ECHO Creating pack file ...
7z.exe a %PackName% -tzip > ..\zip.log
ECHO Packaging complete
POPD
GOTO End

:ErrPackChk
ECHO PackChk.exe has encountered an error!
EXIT /b

:End
ECHO Removing temporary files and folders
:: remove files from local release base folder
FOR %%A IN (%PACK_FILE_LIST%) DO (
  DEL /Q /F %RELEASE_PATH%\%%A 
)

:: remove all directories from list from %RELEASE_PATH%
FOR %%A IN (%PACK_FOLDER_LIST%) DO (
  RMDIR /Q /S  %RELEASE_PATH%\%%A
)

:: remove intermediate Documentation directory from root
RMDIR /Q /S ..\Documentation

DEL ..\zip.log

ECHO gen_pack.bat completed successfully
