# Generic test that uses conan libs

add_executable(pwmfan pwmfan.cpp)

if(NOT TARGET wiringPi)
    add_library(wiringPi SHARED IMPORTED)
    set_target_properties(wiringPi PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SYSROOT}/include"
        IMPORTED_CONFIGURATIONS RELEASE
        IMPORTED_LOCATION_RELEASE "${CMAKE_SYSROOT}/lib/libwiringPi.so.2.50"
        IMPORTED_SONAME_RELEASE "libwiringPi.so.2.50"
        INTERFACE_LINK_LIBRARIES "pthread;crypt;rt"
        )
endif()

target_link_libraries(
    pwmfan
    PRIVATE project_options
            project_warnings
            wiringPi)