# ---- build_app.ps1 -----
# Build OpenVSLAM

Param([parameter(mandatory=$true)][string]$installDir)
$installDir = Resolve-Path $installDir

$buildTarget = "Visual Studio 15 2017"
$platform = "x64"
$buildType = "Release"

# create path variables
$lapackLibrary = (Join-Path $installDir "\lib\liblapack.lib").Replace("\", "/")
$blasLibrary = (Join-Path $installDir "\lib\libblas.lib").Replace("\", "/")
$eigen3Dir = (Join-Path $installDir "\share\eigen3\cmake").Replace("\", "/")
$opencvDir = (Join-Path $installDir "\x64\vc16\lib").Replace("\", "/")
$g2oDir = (Join-Path $installDir "\lib\cmake\g2o").Replace("\", "/")
$dbow2Dir = (Join-Path $installDir "\lib\cmake\DBoW2").Replace("\", "/")
$yamlCppDir = (Join-Path $installDir "\CMake").Replace("\", "/")
$pangolinDir = (Join-Path $installDir "\lib\cmake\Pangolin").Replace("\", "/")
$sioclientDir = (Join-Path $installDir "\lib\cmake\sioclient").Replace("\", "/")
$protocExecutable = (Join-Path $installDir "\bin\protoc.exe").Replace("\", "/")
$protobufIncludeDir = (Join-Path $installDir "\include").Replace("\", "/")
$protobufLibrary = (Join-Path $installDir "\lib\libprotobuf.lib").Replace("\", "/")
$cxsparseIncludeDir = (Join-Path $installDir "\include\suitesparse").Replace("\", "/")
$cxsparseLibrary = (Join-Path $installDir "\lib\libcxsparse.lib").Replace("\", "/")
$suitesparseIncludeDir = (Join-Path $installDir "\include\suitesparse").Replace("\", "/")
$suitesparseLibraryDir = (Join-Path $installDir "\lib").Replace("\", "/")

# build
New-Item "build" -ItemType Directory -ErrorAction SilentlyContinue
Set-Location -Path "build"
cmake .. -G $buildTarget -A $platform `
  -DEigen3_DIR="$eigen3Dir" `
  -DOpenCV_DIR="$opencvDir" `
  -Dg2o_DIR="$g2oDir" `
  -DDBoW2_DIR="$dbow2Dir" `
  -Dyaml-cpp_DIR="$yamlCppDir" `
  -DPangolin_DIR="$pangolinDir" `
  -Dsioclient_DIR="$sioclientDir" `
  -DPROTOBUF_PROTOC_EXECUTABLE="$protocExecutable" `
  -DProtobuf_INCLUDE_DIR="$protobufIncludeDir" `
  -DProtobuf_LIBRARIES="$protobufLibrary" `
  -DLAPACK_LIBRARIES="$lapackLibrary" `
  -DBLAS_LIBRARIES="$blasLibrary" `
  -DCXSPARSE_INCLUDE_DIR="$cxsparseIncludeDir" `
  -DCXSPARSE_LIBRARY="$cxsparseLibrary" `
  -DSUITESPARSE_CHECK_INCLUDE_DIRS="$suitesparseIncludeDir" `
  -DSUITESPARSE_CHECK_LIBRARY_DIRS="$suitesparseLibraryDir" `
  -DBUILD_SHARED_LIBS=OFF `
  -DUSE_OPENMP=OFF `
  -DUSE_PANGOLIN_VIEWER=ON `
  -DUSE_SOCKET_PUBLISHER=ON
MSBuild.exe ALL_BUILD.vcxproj /p:Configuration=$buildType /p:ExceptionHandling=Sync /p:MultiProcessorCompilation=true
