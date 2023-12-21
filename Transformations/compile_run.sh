mkdir build;
cd build;
cmake ../;
cmake --build .;
cp ../config.ini ./;
./main -o result.png;
./main;
