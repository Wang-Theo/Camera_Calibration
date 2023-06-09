add_executable(cam_publish test_cam_publish.cpp)

target_link_libraries(cam_publish
   cam_cali
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${CERES_LIBRARIES}
)