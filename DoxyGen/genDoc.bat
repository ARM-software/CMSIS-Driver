@echo off

REM ====================================================================================
REM Batch file for generating
REM
REM Author  : 
REM Date    : 12th March 2018
REM Version : 1.0
REM Company : Arm 
REM
REM 
REM Command syntax: genDoc.bat
REM
REM  Version: 1.0 Initial Version.
REM ====================================================================================

SETLOCAL ENABLEEXTENSIONS ENABLEDELAYEDEXPANSION

REM -- Delete previous generated HTML files ---------------------
  ECHO.
  ECHO Delete previous generated HTML files

REM -- Remove generated doxygen files ---------------------
IF EXIST ..\Documentation (
  PUSHD ..\Documentation
    RMDIR /S /Q General
  POPD
) ELSE (
  MKDIR ..\Documentation
)

REM -- Generate New HTML Files ---------------------
  ECHO.
  ECHO Generate New HTML Files

PUSHD General
CALL doxygen_general.bat
POPD

:END
  ECHO.
REM done
