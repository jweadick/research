add_artery_feature(police ClearLaneService.cc PoliceService.cc)
generate_opp_message(MotiveReq.msg TARGET police DIRECTORY police_msgs)
generate_opp_message(MathCar.msg TARGET police DIRECTORY mathCar_msgs)
add_opp_run(highway_police NED_FOLDERS ${CMAKE_CURRENT_SOURCE_DIR})
