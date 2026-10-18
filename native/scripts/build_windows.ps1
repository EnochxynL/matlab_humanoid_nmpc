param(
  [ValidateSet('Platform','Core','All')][string]$Stage = 'All',
  [string]$Conan = "$env:LOCALAPPDATA\Conan\conan\conan.exe",
  [string]$CMake,
  [string]$Ninja,
  [string]$MatlabRoot = 'C:\Program Files\MATLAB\R2025b',
  [ValidatePattern('^[D-Z]$')][string]$CacheDrive = 'G',
  [switch]$WithoutMatlab
)
$ErrorActionPreference = 'Stop'
$native = Split-Path $PSScriptRoot -Parent
$root = Split-Path $native -Parent
$build = Join-Path $native 'build/windows'
$coreBuild = Join-Path $build 'native-core'
$mingw = Join-Path $build 'toolchains/mingw64'
$drivePrefix = "$CacheDrive`:"
$mapping = (& subst.exe) | Where-Object { $_ -like "$drivePrefix\:*" }
if ($mapping) {
  $target = ($mapping -split ' => ',2)[1]
  if ($target.TrimEnd('\') -ine $build.Replace('/','\').TrimEnd('\')) { throw "$drivePrefix is mapped elsewhere" }
} elseif (Test-Path "$drivePrefix\") {
  throw "$drivePrefix is already in use"
} else {
  & subst.exe $drivePrefix $build
  if ($LASTEXITCODE -ne 0) { throw 'Cannot create short Conan cache mapping' }
}
if (!(Test-Path "$mingw/bin/g++.exe")) { throw "Missing isolated MinGW 8.1 toolchain: $mingw" }
if (!(Test-Path $Conan)) { $Conan = (Get-Command conan -ErrorAction Stop).Source }
if (!$CMake) {
  $candidates = @(Get-ChildItem "$build/conan-home/p/cmake*/p/bin/cmake.exe" -ErrorAction SilentlyContinue)
  foreach ($candidate in $candidates) {
    if ((& $candidate.FullName --version | Select-Object -First 1) -match 'version 3\.31\.') {
      $CMake = $candidate.FullName; break
    }
  }
}
if (!$CMake) { $CMake = (Get-Command cmake -ErrorAction Stop).Source }
if (!$Ninja) {
  $candidate = Get-ChildItem "$build/conan-home/p/ninja*/p/bin/ninja.exe" -ErrorAction SilentlyContinue | Select-Object -First 1
  if ($candidate) { $Ninja = $candidate.FullName }
  else { $Ninja = (Get-Command ninja -ErrorAction Stop).Source }
}
function Invoke-Checked([string]$Program, [string[]]$Arguments) {
  # Windows PowerShell 5 treats ordinary native stderr as ErrorRecord objects.
  $savedPreference = $ErrorActionPreference
  try {
    $ErrorActionPreference = 'Continue'
    & $Program @Arguments
    $nativeExit = $LASTEXITCODE
  } finally {
    $ErrorActionPreference = $savedPreference
  }
  if ($nativeExit -ne 0) { throw "$Program failed with exit code $nativeExit" }
}
$previousPath = $env:PATH
$previousHome = $env:CONAN_HOME
$previousMirrors = $env:G1_RECIPE_SOURCE_ROOT
Push-Location $root
try {
  # Keep this mapping for generated CMake package paths; reruns verify its target.
  $env:CONAN_HOME = "$drivePrefix\conan-home"
  $env:PATH = "$mingw/bin;$(Split-Path $CMake);$(Split-Path $Ninja);$previousPath"
  $mirrorRoot = Join-Path $native 'third_party'
  if (@('blasfeo','hpipm','ocs2' | Where-Object { !(Test-Path "$mirrorRoot/$_/.git") }).Count -eq 0) {
    $env:G1_RECIPE_SOURCE_ROOT = $mirrorRoot
  } else {
    $env:G1_RECIPE_SOURCE_ROOT = $null
  }
  $profile = 'native/conan/profiles/mingw81'
  & {
    foreach ($dependency in @('blasfeo','hpipm','ocs2')) {
      Invoke-Checked $Conan @('export',"native/conan/recipes/$dependency")
    }
    Invoke-Checked $Conan @('export','native/conan/recipes/coal','--version=3.0.3')
    Invoke-Checked $Conan @('export','native/conan/recipes/pinocchio','--version=4.1.0')
    $coreLockArguments = @()
    if (Test-Path 'native/conan.lock') {
      $coreLockArguments = @('--lockfile=native/conan.lock')
    }
    Invoke-Checked $Conan (@('install','native/conanfile.txt','-pr:h',$profile,'-pr:b',$profile,
      '-of',"$build/native-core-deps",'--build=missing',
      '--deployer=native/conan/deploy_runtime.py',"--deployer-folder=$coreBuild/lib") + $coreLockArguments)
    Copy-Item "$mingw/bin/libgcc_s_seh-1.dll","$mingw/bin/libstdc++-6.dll","$mingw/bin/libwinpthread-1.dll","$mingw/bin/libgomp-1.dll" "$coreBuild/lib"
    if ($coreLockArguments.Count -eq 0) {
      Invoke-Checked $Conan @('lock','create','native/conanfile.txt','-pr:h',$profile,'-pr:b',$profile,
        '--lockfile-out=native/conan.lock')
    }
  }
  if ($Stage -in @('Platform','All')) {
    Invoke-Checked $CMake @('-S','native/tests/windows','-B',"$build/native-platform",'-G','Ninja',
      "-DCMAKE_TOOLCHAIN_FILE=$build/native-core-deps/conan_toolchain.cmake","-DCMAKE_C_COMPILER=$mingw/bin/gcc.exe","-DCMAKE_CXX_COMPILER=$mingw/bin/g++.exe",'-DCMAKE_BUILD_TYPE=Release')
    Invoke-Checked $CMake @('--build',"$build/native-platform")
    Invoke-Checked (Join-Path (Split-Path $CMake) 'ctest.exe') @('--test-dir',"$build/native-platform",'--output-on-failure')
  }
  if ($Stage -in @('Core','All')) {
    $matlab = if ($WithoutMatlab) { 'OFF' } else { 'ON' }
    Invoke-Checked $CMake @('-S','native','-B',"$coreBuild",'-G','Ninja',
      "-DCMAKE_TOOLCHAIN_FILE=$build/native-core-deps/conan_toolchain.cmake",'-DCMAKE_BUILD_TYPE=Release',
      '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG',"-DG1_BUILD_MATLAB=$matlab",'-DG1_BUILD_TESTS=ON',"-DMatlab_ROOT_DIR=$MatlabRoot")
    # Pinocchio/CppAD template units are memory-heavy on this 16 GB host.
    Invoke-Checked $CMake @('--build',"$coreBuild",'--parallel','2')
    Invoke-Checked $CMake @('--install',"$coreBuild",'--prefix',"$root/install/windows")
    Copy-Item "$coreBuild/lib/*.dll" "$root/install/windows/lib" -Force
    Copy-Item "$coreBuild/lib/conan-runtime.json" "$root/install/windows/lib" -Force
    & "$root/native/scripts/verify_runtime.ps1" -Directory "$coreBuild/lib"
    Invoke-Checked "$coreBuild/lib/g1_windows_ocs2_ad_smoke.exe" @("$build/ocs2 ad integration",'create')
    Invoke-Checked "$coreBuild/lib/g1_windows_ocs2_ad_smoke.exe" @("$build/ocs2 ad integration",'reload')
  }
} finally {
  Pop-Location
  $env:PATH = $previousPath
  $env:CONAN_HOME = $previousHome
  $env:G1_RECIPE_SOURCE_ROOT = $previousMirrors
}
