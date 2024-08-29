cmake_minimum_required(VERSION 2.8.3)
project(tiago_controller_configuration_mujoco)
find_package(catkin REQUIRED)
catkin_package()

foreach(dir config launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

