 rmdir  /s /q .\build\
 mkdir build
 cmake -G "Visual Studio 17 2022" -B "build" .
 cmake --build ./build
