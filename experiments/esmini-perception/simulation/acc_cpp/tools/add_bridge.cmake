# Author: Zhuo Ma
# Add ACC observation integration and shared perception; upstream ACC stays intact.
function(accsim_add_native_bridge)
    if(NOT TARGET esminiLib)
        message(FATAL_ERROR "esminiLib target missing in pinned upstream source")
    endif()
    target_sources(esminiLib PRIVATE
        "${ACCSIM_BRIDGE_ROOT}/src/controllers/acc.cpp"
        "${ACCSIM_BRIDGE_ROOT}/src/perception.cpp")
    target_include_directories(esminiLib PRIVATE "${ACCSIM_BRIDGE_ROOT}/include")
endfunction()
cmake_language(DEFER CALL accsim_add_native_bridge)
