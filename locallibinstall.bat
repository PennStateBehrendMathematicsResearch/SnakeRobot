@ECHO OFF
SETLOCAL ENABLEDELAYEDEXPANSION

SET copyDirName=SnakeRobotShared


SET copyDir="%CD%\%copyDirName%"

SET tempFile=%TEMP%\libinstalltemp.txt

ECHO Finding Arduino install...

(reg query "HKLM\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall" /s | findstr /R UninstallString.*Arduino) > "%tempFile%"

IF %ERRORLEVEL% NEQ 1 (
	CALL :errorCheck Querying/parsing uninstall registry key failed.
)

FOR /F delims^=^"^ tokens^=2 %%G IN (%tempFile%) DO (
	SET arduinoUninstallerPath=%%G
)

(reg query "HKLM\SOFTWARE\Wow6432Node\Microsoft\Windows\CurrentVersion\Uninstall" /s | findstr /R UninstallString.*Arduino) > "%tempFile%"

IF %ERRORLEVEL% NEQ 1 (
	CALL :errorCheck Querying/parsing WOW64 uninstall registry key failed.
)

FOR /F delims^=^"^ tokens^=2 %%G IN (%tempFile%) DO (
	SET arduinoUninstallerPath=%%G
)

CD /D "%arduinoUninstallerPath%\.."
CALL :errorCheck Navigating to Arduino install directory failed.

ECHO Finding Arduino library path...

arduino_debug.exe --get-pref "sketchbook.path" > "%tempFile%"
CALL :errorCheck Querying Arduino for library path failed.

FOR /F "delims=" %%G IN (%tempFile%) DO (
	SET lastLine=%%G
)

SET destinationDir="%lastLine%\libraries\%copyDirName%"

ECHO Removing existing library...

DEL %destinationDir%
CALL :errorCheck Removing existing library files failed.

ECHO Copying library...

MKDIR %destinationDir%

XCOPY /E /Y %copyDir% %destinationDir%
CALL :errorCheck Copying library files failed.

ECHO Operation completed

PAUSE
EXIT

:errorCheck
	IF %ERRORLEVEL% NEQ 0 (
		ECHO Error: %* 1>&2
		PAUSE
		EXIT
	) ELSE (
		EXIT /B
	)