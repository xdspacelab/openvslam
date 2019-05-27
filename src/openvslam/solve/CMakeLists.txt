target_sources(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/common.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/homography_solver.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/fundamental_solver.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/essential_solver.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/pnp_solver.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/sim3_solver.cc)
