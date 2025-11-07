
# components/your_component/CMakeLists.txt
file(GLOB SOURCES "src/*.cpp" "src/**/*.cpp")
file(GLOB HEADERS "include/*.h" "include/**/*.h")

idf_component_register(
    SRCS ${SOURCES}          # 自动包含所有.cpp文件
    INCLUDE_DIRS "include"   # 自动导出include/下的头文件
    REQUIRES
     driver
     esp_timer
     esp_adc
     freertos 
)