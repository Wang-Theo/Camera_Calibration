add_executable(cam_publish test_cam_publish.cpp)
target_link_libraries(cam_publish
   cam_cali
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${CERES_LIBRARIES}
)

add_executable(cam_calibrate test_cam_calibrate.cpp)
target_link_libraries(cam_calibrate
   cam_cali
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${CERES_LIBRARIES}
)

add_executable(line_extract test_line_extract.cpp)
target_link_libraries(line_extract
   cam_cali
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${CERES_LIBRARIES}
)