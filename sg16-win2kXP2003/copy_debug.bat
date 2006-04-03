@echo off
copy driver\debug\sbni16.sys %systemroot%\system32\drivers
call lddbginf.bat
sync
