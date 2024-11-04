include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_INCLUDE_PATH})
link_directories(${CMAKE_PREFIX_PATH}/lib)
link_directories(${CMAKE_PREFIX_PATH}/debug/lib)

add_executable (quizRansac ransac2d.cpp ../../render/render.cpp)
target_link_libraries (quizRansac 
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES} 
    spdlog::spdlog
    fmt::fmt
)




