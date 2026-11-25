@echo off
rem Build the browser release and publish it to the web server (default ssh host "jp").
rem   tools\deploy_web.cmd [ssh-host] [--keep N] [--no-build] [--dry-run]
rem Windows cmd / PowerShell launcher for tools\deploy_web.py (needs Python 3 and the OpenSSH client).
setlocal
where py >nul 2>nul
if %errorlevel%==0 (
  py -3 "%~dp0deploy_web.py" %*
) else (
  python "%~dp0deploy_web.py" %*
)
exit /b %errorlevel%
