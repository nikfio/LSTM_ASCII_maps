# adding all libraries needed

add_library (Scenario Scenario.cpp)

add_library (Supervisor Supervisor.cpp)

add_library (RRT RRT.cpp)

target_link_libraries(Scenario -lboost_system -lboost_filesystem)

target_link_libraries(RRT Scenario Supervisor)
# making sure compiler finds libraries

# target_include_directories (Map PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_include_directories (Scenario PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_include_directories (Supervisor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_include_directories (RRT PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
