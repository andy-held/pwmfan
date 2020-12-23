# Generic test that uses conan libs

add_executable(pwmfan pwmfan.cpp)

if(NOT TARGET wiringPi)
    add_library(wiringPi SHARED IMPORTED)
    set_target_properties(wiringPi PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "/usr/include"
        IMPORTED_CONFIGURATIONS RELEASE
        IMPORTED_LOCATION_RELEASE "/usr/lib/libwiringPi.so.2.52"
        IMPORTED_SONAME_RELEASE "libwiringPi.so.2.52"
        INTERFACE_LINK_LIBRARIES "pthread;crypt;rt"
        )
endif()

find_package(docopt COMPONENTS CXX REQUIRED)
find_package(fmt REQUIRED)

target_link_libraries(
    pwmfan
    PRIVATE project_options
            project_warnings
            wiringPi
            docopt
            fmt::fmt)
