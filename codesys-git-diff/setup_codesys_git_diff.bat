@echo off
REM =============================================================================
REM  setup_codesys_git_diff.bat
REM  One-time setup: registers the CodeSys XML normalizer as a git diff driver.
REM
REM  Run this script ONCE from any location.  It writes to your global
REM  ~/.gitconfig so every repo on this machine benefits automatically.
REM
REM  Requirements:
REM    - Git for Windows  (git must be on PATH)
REM    - Python 3.7+      (python must be on PATH)
REM
REM  Usage:
REM    setup_codesys_git_diff.bat             <- global config (all repos)
REM    setup_codesys_git_diff.bat --repo      <- current repo only
REM    setup_codesys_git_diff.bat --uninstall <- remove the driver
REM =============================================================================

setlocal enabledelayedexpansion

REM --- Resolve the directory this script lives in ---
set "SCRIPT_DIR=%~dp0"
REM Remove trailing backslash
if "%SCRIPT_DIR:~-1%"=="\" set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"

set "NORMALIZE_SCRIPT=%SCRIPT_DIR%\codesys_xml_normalize.py"
set "GITATTRIBUTES_TEMPLATE=%SCRIPT_DIR%\.gitattributes.template"

REM --- Parse arguments ---
set "SCOPE=--global"
set "UNINSTALL=0"

:parse_args
if "%~1"=="" goto :check_deps
if /i "%~1"=="--repo"      set "SCOPE=--local"  & shift & goto :parse_args
if /i "%~1"=="--global"    set "SCOPE=--global" & shift & goto :parse_args
if /i "%~1"=="--uninstall" set "UNINSTALL=1"    & shift & goto :parse_args
shift
goto :parse_args

REM --- Uninstall path ---
:check_uninstall
if "%UNINSTALL%"=="1" (
    echo Removing CodeSys XML diff driver from git config [%SCOPE%]...
    git config %SCOPE% --unset diff.codesys-xml.textconv  2>nul
    git config %SCOPE% --unset diff.codesys-xml.cachetextconv 2>nul
    echo Done.  You can also remove lines from .gitattributes manually.
    goto :eof
)

REM --- Dependency checks ---
:check_deps
echo ============================================================
echo  CodeSys XML Git Diff Setup
echo ============================================================
echo.

REM Check git
git --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] git not found on PATH.
    echo         Install Git for Windows: https://git-scm.com/download/win
    pause
    exit /b 1
)
for /f "tokens=*" %%v in ('git --version') do echo [OK] %%v

REM Check python
python --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] python not found on PATH.
    echo         Install Python 3: https://www.python.org/downloads/
    pause
    exit /b 1
)
for /f "tokens=*" %%v in ('python --version') do echo [OK] %%v

REM Check the normalizer script exists
if not exist "%NORMALIZE_SCRIPT%" (
    echo [ERROR] Cannot find normalizer script at:
    echo         %NORMALIZE_SCRIPT%
    echo         Make sure codesys_xml_normalize.py is in the same folder as this .bat
    pause
    exit /b 1
)
echo [OK] Normalizer script found: %NORMALIZE_SCRIPT%
echo.

REM --- Register the git diff driver ---
echo Configuring git diff driver [scope: %SCOPE%]...
echo.

REM Use forward slashes for the path (git prefers them on Windows)
set "NORMALIZE_SCRIPT_FWD=%NORMALIZE_SCRIPT:\=/%"

git config %SCOPE% diff.codesys-xml.textconv "python \"%NORMALIZE_SCRIPT_FWD%\""
git config %SCOPE% diff.codesys-xml.cachetextconv true

if errorlevel 1 (
    echo [ERROR] Failed to write git config.
    if "%SCOPE%"=="--local" echo         Are you inside a git repository?
    pause
    exit /b 1
)

echo [OK] Registered diff driver:
echo      diff.codesys-xml.textconv  = python "%NORMALIZE_SCRIPT_FWD%"
echo      diff.codesys-xml.cachetextconv = true
echo.

REM --- .gitattributes guidance ---
echo ============================================================
echo  NEXT STEP: add a .gitattributes file to your repository
echo ============================================================
echo.
echo  Copy the template from this folder:
echo    %GITATTRIBUTES_TEMPLATE%
echo  to your repo root as ".gitattributes", then commit it.
echo.
echo  Or add these lines to your existing .gitattributes:
echo.
echo    # CodeSys PLCopen XML exports
echo    *.xml     diff=codesys-xml
echo    *.export  diff=codesys-xml
echo    *.expx    diff=codesys-xml
echo.
echo  TIP: If your repo also contains non-CodeSys XML files, be more
echo  specific (e.g., use a subfolder pattern like src/*.xml).
echo.
echo ============================================================
echo  Verify the setup with:
echo    git diff HEAD~1 HEAD -- yourfile.xml
echo ============================================================
echo.
pause
endlocal
