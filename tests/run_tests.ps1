param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
python "$PSScriptRoot/run_tests.py" --compiler $Compiler
if ($LASTEXITCODE -ne 0) { throw 'Tests failed' }
