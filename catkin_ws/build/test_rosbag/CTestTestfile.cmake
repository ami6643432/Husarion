# CMake generated Testfile for 
# Source directory: /home/husarion/catkin_ws/src/ros_comm/test/test_rosbag
# Build directory: /home/husarion/catkin_ws/build/test_rosbag
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_test_rosbag_nosetests_test.test_bag.py "/home/husarion/catkin_ws/build/test_rosbag/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/test_rosbag/test_results/test_rosbag/nosetests-test.test_bag.py.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/husarion/catkin_ws/build/test_rosbag/test_results/test_rosbag" "/usr/bin/nosetests-2.7 -P --process-timeout=60 /home/husarion/catkin_ws/src/ros_comm/test/test_rosbag/test/test_bag.py --with-xunit --xunit-file=/home/husarion/catkin_ws/build/test_rosbag/test_results/test_rosbag/nosetests-test.test_bag.py.xml")
add_test(_ctest_test_rosbag_gtest_test_bag "/home/husarion/catkin_ws/build/test_rosbag/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/test_rosbag/test_results/test_rosbag/gtest-test_bag.xml" "--return-code" "/home/husarion/catkin_ws/devel/.private/test_rosbag/lib/test_rosbag/test_bag --gtest_output=xml:/home/husarion/catkin_ws/build/test_rosbag/test_results/test_rosbag/gtest-test_bag.xml")
add_test(_ctest_test_rosbag_rostest_test_play_play.test "/home/husarion/catkin_ws/build/test_rosbag/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/test_rosbag/test_results/test_rosbag/rostest-test_play_play.xml" "--return-code" "/usr/bin/python2 /home/husarion/catkin_ws/src/ros_comm/tools/rostest/scripts/rostest --pkgdir=/home/husarion/catkin_ws/src/ros_comm/test/test_rosbag --package=test_rosbag --results-filename test_play_play.xml --results-base-dir \"/home/husarion/catkin_ws/build/test_rosbag/test_results\" /home/husarion/catkin_ws/build/test_rosbag/test/play_play.test ")
add_test(_ctest_test_rosbag_rostest_test_rosbag_play.test "/home/husarion/catkin_ws/build/test_rosbag/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/test_rosbag/test_results/test_rosbag/rostest-test_rosbag_play.xml" "--return-code" "/usr/bin/python2 /home/husarion/catkin_ws/src/ros_comm/tools/rostest/scripts/rostest --pkgdir=/home/husarion/catkin_ws/src/ros_comm/test/test_rosbag --package=test_rosbag --results-filename test_rosbag_play.xml --results-base-dir \"/home/husarion/catkin_ws/build/test_rosbag/test_results\" /home/husarion/catkin_ws/build/test_rosbag/test/rosbag_play.test ")
add_test(_ctest_test_rosbag_rostest_test_latched_pub.test "/home/husarion/catkin_ws/build/test_rosbag/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/test_rosbag/test_results/test_rosbag/rostest-test_latched_pub.xml" "--return-code" "/usr/bin/python2 /home/husarion/catkin_ws/src/ros_comm/tools/rostest/scripts/rostest --pkgdir=/home/husarion/catkin_ws/src/ros_comm/test/test_rosbag --package=test_rosbag --results-filename test_latched_pub.xml --results-base-dir \"/home/husarion/catkin_ws/build/test_rosbag/test_results\" /home/husarion/catkin_ws/src/ros_comm/test/test_rosbag/test/latched_pub.test ")
add_test(_ctest_test_rosbag_rostest_test_latched_sub.test "/home/husarion/catkin_ws/build/test_rosbag/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/test_rosbag/test_results/test_rosbag/rostest-test_latched_sub.xml" "--return-code" "/usr/bin/python2 /home/husarion/catkin_ws/src/ros_comm/tools/rostest/scripts/rostest --pkgdir=/home/husarion/catkin_ws/src/ros_comm/test/test_rosbag --package=test_rosbag --results-filename test_latched_sub.xml --results-base-dir \"/home/husarion/catkin_ws/build/test_rosbag/test_results\" /home/husarion/catkin_ws/build/test_rosbag/test/latched_sub.test ")
add_test(_ctest_test_rosbag_rostest_test_record_two_publishers.test "/home/husarion/catkin_ws/build/test_rosbag/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/test_rosbag/test_results/test_rosbag/rostest-test_record_two_publishers.xml" "--return-code" "/usr/bin/python2 /home/husarion/catkin_ws/src/ros_comm/tools/rostest/scripts/rostest --pkgdir=/home/husarion/catkin_ws/src/ros_comm/test/test_rosbag --package=test_rosbag --results-filename test_record_two_publishers.xml --results-base-dir \"/home/husarion/catkin_ws/build/test_rosbag/test_results\" /home/husarion/catkin_ws/src/ros_comm/test/test_rosbag/test/record_two_publishers.test ")
add_test(_ctest_test_rosbag_rostest_test_record_one_publisher_two_topics.test "/home/husarion/catkin_ws/build/test_rosbag/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/test_rosbag/test_results/test_rosbag/rostest-test_record_one_publisher_two_topics.xml" "--return-code" "/usr/bin/python2 /home/husarion/catkin_ws/src/ros_comm/tools/rostest/scripts/rostest --pkgdir=/home/husarion/catkin_ws/src/ros_comm/test/test_rosbag --package=test_rosbag --results-filename test_record_one_publisher_two_topics.xml --results-base-dir \"/home/husarion/catkin_ws/build/test_rosbag/test_results\" /home/husarion/catkin_ws/src/ros_comm/test/test_rosbag/test/record_one_publisher_two_topics.test ")
add_test(_ctest_test_rosbag_rostest_test_record_sigint_cleanup.test "/home/husarion/catkin_ws/build/test_rosbag/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/test_rosbag/test_results/test_rosbag/rostest-test_record_sigint_cleanup.xml" "--return-code" "/usr/bin/python2 /home/husarion/catkin_ws/src/ros_comm/tools/rostest/scripts/rostest --pkgdir=/home/husarion/catkin_ws/src/ros_comm/test/test_rosbag --package=test_rosbag --results-filename test_record_sigint_cleanup.xml --results-base-dir \"/home/husarion/catkin_ws/build/test_rosbag/test_results\" /home/husarion/catkin_ws/src/ros_comm/test/test_rosbag/test/record_sigint_cleanup.test ")
add_test(_ctest_test_rosbag_rostest_test_record_sigterm_cleanup.test "/home/husarion/catkin_ws/build/test_rosbag/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/husarion/catkin_ws/build/test_rosbag/test_results/test_rosbag/rostest-test_record_sigterm_cleanup.xml" "--return-code" "/usr/bin/python2 /home/husarion/catkin_ws/src/ros_comm/tools/rostest/scripts/rostest --pkgdir=/home/husarion/catkin_ws/src/ros_comm/test/test_rosbag --package=test_rosbag --results-filename test_record_sigterm_cleanup.xml --results-base-dir \"/home/husarion/catkin_ws/build/test_rosbag/test_results\" /home/husarion/catkin_ws/src/ros_comm/test/test_rosbag/test/record_sigterm_cleanup.test ")
subdirs("gtest")
subdirs("bag_migration_tests")
