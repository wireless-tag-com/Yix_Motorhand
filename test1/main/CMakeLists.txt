idf_component_register(
    SRCS "main.cpp"  
    INCLUDE_DIRS "."  
    
    "${CMAKE_CURRENT_SOURCE_DIR}/../components/core/include"
    REQUIRES 
        core               
        driver              
        freertos            
        esp_timer           
)


target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)

# 添加编译宏定义
# target_compile_definitions(${COMPONENT_LIB} PRIVATE DEBUG_MODE)