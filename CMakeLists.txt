# HMS_BH17XX_DRIVER/CMakeLists.txt

set(HMS_BH17XX_DRIVER_VERSION 1.0.0)

set(HMS_BH17XX_IS_STM32 FALSE)
if(CMAKE_SYSTEM_NAME STREQUAL "Generic" OR
   CMAKE_C_COMPILER_TARGET MATCHES "arm-none-eabi" OR
   CMAKE_CXX_COMPILER_TARGET MATCHES "arm-none-eabi" OR
   CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|ARM|armv)")
  set(HMS_BH17XX_IS_STM32 TRUE)
endif()

# Check if we're building with Zephyr
if(DEFINED ZEPHYR_BASE)
    zephyr_include_directories(include)
    zephyr_library_sources(src/HMS_BH17XX_DRIVER.cpp)

# Check if we're building with ESP-IDF
elseif(IDF_PROJECT)
    idf_component_register(
        SRCS "src/HMS_BH17XX_DRIVER.cpp"
        INCLUDE_DIRS "include"
    )

# STM32 / generic embedded CMake project
elseif(HMS_BH17XX_IS_STM32)
    add_library(HMS_BH17XX_DRIVER STATIC
        src/HMS_BH17XX_DRIVER.cpp
    )
    target_include_directories(HMS_BH17XX_DRIVER PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_compile_features(HMS_BH17XX_DRIVER PUBLIC cxx_std_17)

# Desktop / other CMake project
else()
    add_library(HMS_BH17XX_DRIVER INTERFACE)
    target_include_directories(HMS_BH17XX_DRIVER INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_compile_features(HMS_BH17XX_DRIVER INTERFACE cxx_std_17)
endif()