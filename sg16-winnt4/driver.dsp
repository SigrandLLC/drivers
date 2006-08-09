# Microsoft Developer Studio Project File - Name="driver" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Application" 0x0101

CFG=driver - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "driver.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "driver.mak" CFG="driver - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "driver - Win32 Release" (based on "Win32 (x86) Application")
!MESSAGE "driver - Win32 Debug" (based on "Win32 (x86) Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "driver - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "driver\Release"
# PROP Intermediate_Dir "driver\Release"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /D "_MBCS" /YX /FD /c
# ADD CPP /nologo /Gz /W4 /WX /Ox /Ot /Oa /Ow /Og /Oi /I "$(win2kddk)\inc\ddk" /I "$(win2kddk)\inc" /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /D "_MBCS" /FR /Yu"common.h" /FD /c
# SUBTRACT CPP /Os
# ADD BASE MTL /nologo /D "NDEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "NDEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x419 /d "NDEBUG"
# ADD RSC /l 0x409 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:windows /machine:I386
# ADD LINK32 ndis.lib ntoskrnl.lib hal.lib /nologo /map /machine:I386 /nodefaultlib /out:"driver\Release/sg16.sys" /libpath:"$(win2kddk)\lib" /subsystem:native /driver
# SUBTRACT LINK32 /pdb:none

!ELSEIF  "$(CFG)" == "driver - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "driver\Debug"
# PROP Intermediate_Dir "driver\Debug"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /D "_MBCS" /YX /FD /GZ /c
# ADD CPP /nologo /Gz /W4 /WX /Zi /Od /I "$(win2kddk)\inc\ddk" /I "$(win2kddk)\inc" /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /D "_MBCS" /FR /Yu"precomp.h" /FD /c
# ADD BASE MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x419 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:windows /debug /machine:I386 /pdbtype:sept
# ADD LINK32 ndis.lib ntoskrnl.lib hal.lib /nologo /incremental:no /map /debug /machine:I386 /nodefaultlib /out:"driver\Debug/sbni16.sys" /libpath:"$(win2kddk)\lib" /subsystem:native /driver
# SUBTRACT LINK32 /verbose /pdb:none

!ENDIF 

# Begin Target

# Name "driver - Win32 Release"
# Name "driver - Win32 Debug"
# Begin Source File

SOURCE=.\handler.c
# End Source File
# Begin Source File

SOURCE=.\init.c
# End Source File
# Begin Source File

SOURCE=.\precomp.c
# End Source File
# Begin Source File

SOURCE=.\precomp.h
# End Source File
# Begin Source File

SOURCE=.\queries.c
# End Source File
# Begin Source File

SOURCE=.\routines.c
# End Source File
# Begin Source File

SOURCE=.\sbni16.c
# End Source File
# Begin Source File

SOURCE=.\sbni16.rc
# End Source File
# Begin Source File

SOURCE=.\sbni16hw.h
# End Source File
# Begin Source File

SOURCE=.\sbni16sw.h
# End Source File
# End Target
# End Project
