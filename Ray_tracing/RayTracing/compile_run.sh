cmake .;
cmake --build .;
./pathtracer -r 2400 1800 -f CBspheres.png dae/sky/CBspheres_lambertian.dae;