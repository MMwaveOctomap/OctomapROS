# CMake generated Testfile for 
# Source directory: /home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing
# Build directory: /home/zgh/workspace/octomap_src/octomap-devel/cmake-build-debug/octomap/src/testing
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(MathVector "/home/zgh/workspace/octomap_src/octomap-devel/bin/unit_tests" "MathVector")
set_tests_properties(MathVector PROPERTIES  _BACKTRACE_TRIPLES "/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;35;ADD_TEST;/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;0;")
add_test(MathPose "/home/zgh/workspace/octomap_src/octomap-devel/bin/unit_tests" "MathPose")
set_tests_properties(MathPose PROPERTIES  _BACKTRACE_TRIPLES "/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;36;ADD_TEST;/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;0;")
add_test(InsertRay "/home/zgh/workspace/octomap_src/octomap-devel/bin/unit_tests" "InsertRay")
set_tests_properties(InsertRay PROPERTIES  _BACKTRACE_TRIPLES "/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;37;ADD_TEST;/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;0;")
add_test(InsertScan "/home/zgh/workspace/octomap_src/octomap-devel/bin/unit_tests" "InsertScan")
set_tests_properties(InsertScan PROPERTIES  _BACKTRACE_TRIPLES "/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;38;ADD_TEST;/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;0;")
add_test(ReadGraph "/home/zgh/workspace/octomap_src/octomap-devel/bin/unit_tests" "ReadGraph")
set_tests_properties(ReadGraph PROPERTIES  _BACKTRACE_TRIPLES "/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;39;ADD_TEST;/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;0;")
add_test(StampedTree "/home/zgh/workspace/octomap_src/octomap-devel/bin/unit_tests" "StampedTree")
set_tests_properties(StampedTree PROPERTIES  _BACKTRACE_TRIPLES "/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;40;ADD_TEST;/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;0;")
add_test(OcTreeKey "/home/zgh/workspace/octomap_src/octomap-devel/bin/unit_tests" "OcTreeKey")
set_tests_properties(OcTreeKey PROPERTIES  _BACKTRACE_TRIPLES "/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;41;ADD_TEST;/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;0;")
add_test(test_scans "/home/zgh/workspace/octomap_src/octomap-devel/bin/test_scans" "/home/zgh/workspace/octomap_src/octomap-devel/octomap/share/data/spherical_scan.graph")
set_tests_properties(test_scans PROPERTIES  _BACKTRACE_TRIPLES "/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;42;ADD_TEST;/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;0;")
add_test(test_raycasting "/home/zgh/workspace/octomap_src/octomap-devel/bin/test_raycasting")
set_tests_properties(test_raycasting PROPERTIES  _BACKTRACE_TRIPLES "/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;43;ADD_TEST;/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;0;")
add_test(test_io "/home/zgh/workspace/octomap_src/octomap-devel/bin/test_io" "/home/zgh/workspace/octomap_src/octomap-devel/octomap/share/data/geb079.bt")
set_tests_properties(test_io PROPERTIES  _BACKTRACE_TRIPLES "/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;44;ADD_TEST;/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;0;")
add_test(test_pruning "/home/zgh/workspace/octomap_src/octomap-devel/bin/test_pruning")
set_tests_properties(test_pruning PROPERTIES  _BACKTRACE_TRIPLES "/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;45;ADD_TEST;/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;0;")
add_test(test_iterators "/home/zgh/workspace/octomap_src/octomap-devel/bin/test_iterators" "/home/zgh/workspace/octomap_src/octomap-devel/octomap/share/data/geb079.bt")
set_tests_properties(test_iterators PROPERTIES  _BACKTRACE_TRIPLES "/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;46;ADD_TEST;/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;0;")
add_test(test_mapcollection "/home/zgh/workspace/octomap_src/octomap-devel/bin/test_mapcollection" "/home/zgh/workspace/octomap_src/octomap-devel/octomap/share/data/mapcoll.txt")
set_tests_properties(test_mapcollection PROPERTIES  _BACKTRACE_TRIPLES "/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;47;ADD_TEST;/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;0;")
add_test(test_color_tree "/home/zgh/workspace/octomap_src/octomap-devel/bin/test_color_tree")
set_tests_properties(test_color_tree PROPERTIES  _BACKTRACE_TRIPLES "/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;48;ADD_TEST;/home/zgh/workspace/octomap_src/octomap-devel/octomap/src/testing/CMakeLists.txt;0;")
