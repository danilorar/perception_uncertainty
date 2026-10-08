# Author: Zhuo Ma
# Inject only the adapter; upstream ControllerACC.cpp/hpp stay byte-for-byte intact.
function(accsim_add_native_bridge)
    if(NOT TARGET esminiLib)
        message(FATAL_ERROR "esminiLib target missing in pinned upstream source")
    endif()
    target_sources(esminiLib PRIVATE "${ACCSIM_BRIDGE_ROOT}/src/native_acc_bridge.cpp"
        "${ACCSIM_BRIDGE_ROOT}/src/native_aeb_bridge.cpp")
    target_include_directories(esminiLib PRIVATE "${ACCSIM_BRIDGE_ROOT}/include")
endfunction()
cmake_language(DEFER CALL accsim_add_native_bridge)
