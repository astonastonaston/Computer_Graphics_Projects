#! /bin/bash
sudo apt install libpcl-dev
mkdir -p build
cd build
cmake ../
cmake --build .
./meshedit ${PWD}/../ply/tt.ply 1 1
# After running that, you just need to press "P" to reconstruct the mesh