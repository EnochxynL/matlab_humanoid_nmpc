param([Parameter(Mandatory=$true)][string]$Directory)
$ErrorActionPreference='Stop'
$root=Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$objdump=Join-Path $root 'native/build/windows/toolchains/mingw64/bin/objdump.exe'
$records=@()
$missing=@()
foreach($binary in Get-ChildItem -LiteralPath $Directory -File | Where-Object { $_.Extension -in @('.dll','.mexw64','.exe') }) {
  $imports=@(& $objdump -p $binary.FullName | ForEach-Object {
    if($_ -match 'DLL Name:\s*(\S+)'){ $matches[1] }
  })
  if($LASTEXITCODE -ne 0){throw "Cannot inspect $($binary.FullName)"}
  foreach($name in $imports){
    if($name -match '^(api-ms-|ext-ms-)'){continue}
    if(!(Test-Path (Join-Path $Directory $name)) -and !(Test-Path (Join-Path "$env:WINDIR/System32" $name))){
      # MATLAB SDK imports are supplied by MATLAB, not redistributed here.
      if($name -notmatch '^(libmex|libmx|libmat|libMatlabDataArray|libMatlabEngine)\.dll$'){
        $missing += "$($binary.Name): $name"
      }
    }
  }
  $records += [pscustomobject]@{binary=$binary.Name;imports=$imports}
}
$records | ConvertTo-Json -Depth 4 | Set-Content -Encoding UTF8 (Join-Path $Directory 'runtime-imports.json')
if($missing.Count){throw ($missing -join "`n")}
Write-Output "Runtime imports resolved for $($records.Count) binaries (MATLAB SDK supplied by MATLAB)."
