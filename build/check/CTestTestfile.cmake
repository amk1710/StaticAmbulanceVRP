# CMake generated Testfile for 
# Source directory: /mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/check
# Build directory: /mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/build/check
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(examples-StaticAmbulanceVRP-build "/usr/bin/cmake" "--build" "/mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/build" "--config" "" "--target" "StaticAmbulanceVRP")
set_tests_properties(examples-StaticAmbulanceVRP-build PROPERTIES  RESOURCE_LOCK "libscip" _BACKTRACE_TRIPLES "/mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/check/CMakeLists.txt;16;add_test;/mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/check/CMakeLists.txt;0;")
add_test(examples-StaticAmbulanceVRP-u20_00 "/mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/build/StaticAmbulanceVRP" "-f" "/mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/check/../data/u20_00.bpa" "-o" "9" "9")
set_tests_properties(examples-StaticAmbulanceVRP-u20_00 PROPERTIES  DEPENDS "examples-StaticAmbulanceVRP-build" PASS_REGULAR_EXPRESSION "Validation         : Success" _BACKTRACE_TRIPLES "/mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/check/CMakeLists.txt;37;add_test;/mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/check/CMakeLists.txt;0;")
add_test(examples-StaticAmbulanceVRP-u40_00 "/mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/build/StaticAmbulanceVRP" "-f" "/mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/check/../data/u40_00.bpa" "-o" "17" "17")
set_tests_properties(examples-StaticAmbulanceVRP-u40_00 PROPERTIES  DEPENDS "examples-StaticAmbulanceVRP-build" PASS_REGULAR_EXPRESSION "Validation         : Success" _BACKTRACE_TRIPLES "/mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/check/CMakeLists.txt;37;add_test;/mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/check/CMakeLists.txt;0;")
add_test(examples-StaticAmbulanceVRP-u60_00 "/mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/build/StaticAmbulanceVRP" "-f" "/mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/check/../data/u60_00.bpa" "-o" "27" "27")
set_tests_properties(examples-StaticAmbulanceVRP-u60_00 PROPERTIES  DEPENDS "examples-StaticAmbulanceVRP-build" PASS_REGULAR_EXPRESSION "Validation         : Success" _BACKTRACE_TRIPLES "/mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/check/CMakeLists.txt;37;add_test;/mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/check/CMakeLists.txt;0;")
