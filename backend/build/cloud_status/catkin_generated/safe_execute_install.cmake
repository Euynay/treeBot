execute_process(COMMAND "/home/wzq/my_ws/build/cloud_status/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/wzq/my_ws/build/cloud_status/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
