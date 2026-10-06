cmake_minimum_required(VERSION 3.20)
if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()
project(Boeing747Sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(QT6_ROOT "" CACHE PATH "Qt 6 installation prefix")
if(QT6_ROOT)
    list(PREPEND CMAKE_PREFIX_PATH "${QT6_ROOT}")
endif()

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Charts)

add_executable(Boeing747Sim
    main.cpp
    SimulationUI.cpp
    SimulationUI.h
    PhysicsEngine.h
)

set_target_properties(Boeing747Sim PROPERTIES
    CXX_EXTENSIONS OFF
    AUTOMOC ON
    AUTOUIC ON
)

target_link_libraries(Boeing747Sim PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::Charts
)

if(WIN32)
    if(NOT TARGET Qt6::windeployqt)
        message(FATAL_ERROR "Qt's windeployqt tool is required to deploy Boeing747Sim on Windows.")
    endif()

    add_custom_command(TARGET Boeing747Sim POST_BUILD
        COMMAND "$<TARGET_FILE:Qt6::windeployqt>"
            "$<IF:$<CONFIG:Debug>,--debug,--release>"
            "$<TARGET_FILE:Boeing747Sim>"
        COMMENT "Deploying Qt runtime dependencies"
        VERBATIM
    )
endif()

if(MSVC)
    target_compile_options(Boeing747Sim PRIVATE /W4 /WX /permissive-)
else()
    target_compile_options(Boeing747Sim PRIVATE -Wall -Wextra -Werror -pedantic)
endif()