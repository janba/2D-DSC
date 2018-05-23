project(2D-DSC)


if (APPLE)
    # TODO for Mac
endif (APPLE)

if (UNIX)
    # TODO for UNix
endif (UNIX)
if(WIN32)
    # TODO for Windows
endif()


aux_source_directory(./src SRC)

include_directories(../GEL/src/GEL)

add_library(2D-DSC SHARED ${SRC})