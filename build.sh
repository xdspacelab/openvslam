echo "Building openVSLAM"

mkdir -p build
cd build

# -DCMAKE_TOOLCHAIN_FILE=/home/jedsadakorn/git/vcpkg/scripts/buildsystems/vcpkg.cmake \

cmake \
    -DBUILD_WITH_MARCH_NATIVE=ON \
    -DUSE_PANGOLIN_VIEWER=ON \
    -DUSE_SOCKET_PUBLISHER=OFF \
    -DUSE_STACK_TRACE_LOGGER=ON \
    -DBOW_FRAMEWORK=DBoW2 \
    -DBUILD_TESTS=ON \
    ..

make -j8

