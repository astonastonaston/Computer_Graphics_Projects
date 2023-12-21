cmake .;
cmake --build .;
./pathtracer -t 8 -s 4 -l 64 -m 5 dae/sky/bunny_unlit.dae -e exr/field.exr;