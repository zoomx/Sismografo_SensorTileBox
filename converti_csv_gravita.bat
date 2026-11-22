@echo off
rem Trascina uno o piu' file CSV del sismografo su questo file:
rem crea NOME_tempo.csv (come prima) e NOME_tempo_netto.ods (accelerazione senza gravita'
rem e foglio "Orientamento"). L'ODS richiede:  pip install odfpy
rem Opzioni utili da aggiungere alla riga python qui sotto:
rem   --riferimento pre|post|tutto   --finestra 150   --soglia 100   --decimali 5   --local
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
%PY% "%~dp0csv_con_gravita.py" "%~1"
shift
goto prossimo

:fine
echo.
pause
