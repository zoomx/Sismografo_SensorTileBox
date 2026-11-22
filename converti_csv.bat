@echo off
rem Trascina uno o piu' file CSV del sismografo su questo file:
rem crea NOME_tempo.csv e NOME_tempo.ods (l'ODS richiede:  pip install odfpy)
rem Opzioni utili da aggiungere nella riga di comando python qui sotto:
rem   --mg  (colonne in mg)   --decimali 5   --local (ora locale)   --sep ";"
setlocal

where py >nul 2>nul
if %errorlevel%==0 (set "PY=py") else (set "PY=python")

if "%~1"=="" (
  echo Trascina un file CSV su questo file .bat
  pause
  exit /b 1
)

:prossimo
if "%~1"=="" goto fine
echo.
echo === %~nx1 ===
%PY% "%~dp0csv_con_tempo.py" "%~1" --ods
shift
goto prossimo

:fine
echo.
pause
