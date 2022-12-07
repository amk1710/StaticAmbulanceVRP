include(CTest)

#
# instances of varying size in the StaticAmbulanceVRP data set.
#
set(instances
    "u20_00\;9"
    "u40_00\;17"
    "u60_00\;27"
#    "u80_00\;33"
#    "u100_00\;40"
#    "u120_00\;48"
# the remaining 3 from this set take too long.
)

add_test(NAME examples-StaticAmbulanceVRP-build
        COMMAND "${CMAKE_COMMAND}" --build "${CMAKE_BINARY_DIR}" --config $<CONFIG> --target StaticAmbulanceVRP
        )
#
# avoid that several build jobs try to concurrently build the SCIP library
# note that this ressource lock name is not the actual libscip target
#
set_tests_properties(examples-StaticAmbulanceVRP-build
                    PROPERTIES
                        RESOURCE_LOCK libscip
                    )

#
# add a test for every instance
#
foreach(instance ${instances})
    list(GET instance 0 basename)
    list(GET instance 1 optval)
    #
    # call the StaticAmbulanceVRP binary and validate the solve with the given objective value
    #
    add_test(NAME "examples-StaticAmbulanceVRP-${basename}"
            COMMAND $<TARGET_FILE:StaticAmbulanceVRP> -f ${CMAKE_CURRENT_SOURCE_DIR}/../data/${basename}.bpa -o ${optval} ${optval}
            )
    set_tests_properties("examples-StaticAmbulanceVRP-${basename}"
                        PROPERTIES
                            PASS_REGULAR_EXPRESSION "Validation         : Success"
                            DEPENDS examples-StaticAmbulanceVRP-build
                        )
    if(WIN32)
        # on windows we need to execute the application and examples executables from the directory containing the libscip.dll,
        # on other systems this directory does not exist.
        set_tests_properties("examples-StaticAmbulanceVRP-${basename}"
                PROPERTIES
                    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin/$<CONFIG>
            )
    endif()
endforeach()
