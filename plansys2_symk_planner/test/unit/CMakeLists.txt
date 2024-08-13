ament_add_gtest(symk_test symk_test.cpp)
target_link_libraries(symk_test ${PROJECT_NAME} dl)
target_compile_definitions(symk_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
