param(
  [ValidateSet('Platform','Core','All')][string]$Stage = 'All',
  [string]$Conan = "$env:LOCALAPPDATA\Conan\conan\conan.exe",
  [string]$CMake,
  [string]$Ninja,
  [string]$MatlabRoot = 'C:\Program Files\MATLAB\R2025b',
  [ValidatePattern('^[D-Z]$')][string]$CacheDrive = 'G',
  [string]$InstallPrefix,
  [switch]$WithoutMatlab
)
$ErrorActionPreference = 'Stop'
$native = Split-Path $PSScriptRoot -Parent
$root = Split-Path $native -Parent
$build = Join-Path $native 'build/windows'
$cc = (Get-Command gcc -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source
$cxx = (Get-Command g++ -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source
if ((Split-Path $cc) -ine (Split-Path $cxx)) { throw 'gcc and g++ must come from the same toolchain on PATH' }
$compilerBin = Split-Path $cc
$compilerVersion = (& $cc '-dumpfullversion').Trim()
$compilerMajor = $compilerVersion.Split('.')[0]
$target = (& $cc '-dumpmachine').Trim()
$crtMacros = '' | & $cc '-dM' '-E' '-include' '_mingw.h' '-xc' '-'
if ($LASTEXITCODE -ne 0 -or $target -ne 'x86_64-w64-mingw32' -or !($crtMacros -match '^#define _UCRT')) {
  throw 'Run with the native x64 MSYS2 UCRT64 gcc/g++ on PATH'
}
$toolchainTag = "gcc$compilerMajor-ucrt64"
$toolchainBuild = Join-Path $build $toolchainTag
$coreBuild = Join-Path $toolchainBuild 'core'
if (!$InstallPrefix) { $InstallPrefix = Join-Path $root 'install/windows' }
Write-Output "Compiler: $cxx ($compilerVersion, $toolchainTag)"
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
Push-Location $root
try {
  # Keep this mapping for generated CMake package paths; reruns verify its target.
  $env:CONAN_HOME = "$drivePrefix\conan-home"
  $env:PATH = "$compilerBin;$(Split-Path $CMake);$(Split-Path $Ninja);$previousPath"
  $profile = 'native/conan/profiles/gcc-ucrt64'
  $buildProfile = 'native/conan/profiles/gcc'
  & {
    foreach ($dependency in @('blasfeo','hpipm','ocs2','urdfdom')) {
      Invoke-Checked $Conan @('export',"native/conan/recipes/$dependency")
    }
    Invoke-Checked $Conan @('export','native/conan/recipes/coal','--version=3.0.3')
    Invoke-Checked $Conan @('export','native/conan/recipes/pinocchio','--version=4.1.0')
    $coreLockArguments = @()
    if (Test-Path 'native/conan.lock') {
      $coreLockArguments = @('--lockfile=native/conan.lock')
    }
    Invoke-Checked $Conan (@('install','native/conanfile.txt','-pr:h',$profile,'-pr:b',$buildProfile,
      '-of',"$toolchainBuild/deps",'--build=missing',
      '--deployer=native/conan/deploy_runtime.py',"--deployer-folder=$coreBuild/lib") + $coreLockArguments)
    foreach ($runtime in @('libgcc_s_seh-1.dll','libstdc++-6.dll','libwinpthread-1.dll','libgomp-1.dll')) {
      Copy-Item (Join-Path $compilerBin $runtime) "$coreBuild/lib"
    }
    $metadata = @{ compiler='gcc'; version=$compilerVersion; crt='ucrt64'; tag=$toolchainTag } | ConvertTo-Json
    [IO.File]::WriteAllText("$coreBuild/lib/compiler-info.json", $metadata, (New-Object Text.UTF8Encoding($false)))
    if ($coreLockArguments.Count -eq 0) {
      Invoke-Checked $Conan @('lock','create','native/conanfile.txt','-pr:h',$profile,'-pr:b',$buildProfile,
        '--lockfile-out=native/conan.lock')
    }
  }
  if ($Stage -in @('Platform','All')) {
    Invoke-Checked $CMake @('-S','native/tests/windows','-B',"$toolchainBuild/platform",'-G','Ninja',
      "-DCMAKE_TOOLCHAIN_FILE=$toolchainBuild/deps/conan_toolchain.cmake",'-DCMAKE_BUILD_TYPE=Release')
    Invoke-Checked $CMake @('--build',"$toolchainBuild/platform")
    Invoke-Checked (Join-Path (Split-Path $CMake) 'ctest.exe') @('--test-dir',"$toolchainBuild/platform",'--output-on-failure')
  }
  if ($Stage -in @('Core','All')) {
    $matlab = if ($WithoutMatlab) { 'OFF' } else { 'ON' }
    Invoke-Checked $CMake @('-S','native','-B',"$coreBuild",'-G','Ninja',
      "-DCMAKE_TOOLCHAIN_FILE=$toolchainBuild/deps/conan_toolchain.cmake",'-DCMAKE_BUILD_TYPE=Release',
      '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG',"-DG1_BUILD_MATLAB=$matlab",'-DG1_BUILD_TESTS=ON',"-DMatlab_ROOT_DIR=$MatlabRoot")
    # Pinocchio/CppAD template units are memory-heavy on this 16 GB host.
    Invoke-Checked $CMake @('--build',"$coreBuild",'--parallel','2')
    Invoke-Checked $CMake @('--install',"$coreBuild",'--prefix',"$InstallPrefix")
    Copy-Item "$coreBuild/lib/*.dll" "$InstallPrefix/lib" -Force
    Copy-Item "$coreBuild/lib/conan-runtime.json","$coreBuild/lib/compiler-info.json" "$InstallPrefix/lib" -Force
    & "$root/native/scripts/verify_runtime.ps1" -Directory "$coreBuild/lib"
    Invoke-Checked "$coreBuild/lib/g1_windows_ocs2_ad_smoke.exe" @("$toolchainBuild/ocs2 ad integration",'create')
    Invoke-Checked "$coreBuild/lib/g1_windows_ocs2_ad_smoke.exe" @("$toolchainBuild/ocs2 ad integration",'reload')
  }
} finally {
  Pop-Location
  $env:PATH = $previousPath
  $env:CONAN_HOME = $previousHome
}
