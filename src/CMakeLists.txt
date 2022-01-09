# Generic test that uses conan libs

add_executable(pwmfan pwmfan.cpp)

if(NOT TARGET pigpio::pigpio)
    add_library(pigpio::pigpio SHARED IMPORTED)
    set_target_properties(pigpio::pigpio PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "/usr/include"
        IMPORTED_CONFIGURATIONS RELEASE
        IMPORTED_LOCATION_RELEASE "/usr/lib/libpigpio.so.1"
        IMPORTED_SONAME_RELEASE "libpigpio.so.1"
        INTERFACE_LINK_LIBRARIES "pthread;rt"
        )
endif()

find_package(fmt REQUIRED)
# find_package(pigpio REQUIRED)

target_link_libraries(
    pwmfan
    PRIVATE project_options
            project_warnings
            pigpio::pigpio
            pthread
            rt
            fmt::fmt)
