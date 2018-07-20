
# See libuavcan/CMakelists and matrix/Cmakelists for examples
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-error=float-equal -Wno-error=double-promotion")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=float-equal -Wno-error=double-promotion")

# From: https://stackoverflow.com/questions/2110795/how-to-use-all-c-files-in-a-directory-with-the-cmake-build-system
file(GLOB SRCS ./*.cpp ./*/*.cpp ./*/*/*.cpp ./*/*/*/*.cpp ./*.c ./*/*.c ./*/*/*.c ./*/*/*/*.c)

px4_add_library(simulink_codegen 
    ${SRCS}
)
