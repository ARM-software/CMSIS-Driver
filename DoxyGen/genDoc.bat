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
PUSHD ..\Documentation
RMDIR /S /Q General
POPD

REM -- Generate New HTML Files ---------------------
  ECHO.
  ECHO Generate New HTML Files

pushd General
CALL doxygen_general.bat
popd

:END
  ECHO.
REM done
