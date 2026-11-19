#include "car_mission/core.hpp"
#include <cassert>
#include <iostream>
int main(){
 using car_mission::Inputs;using car_mission::classify;
 Inputs i;assert(classify(i)=="WAIT_LINK");
 i.ever_link=true;assert(classify(i)=="LINK_LOST");
 i.link_alive=true;assert(classify(i)=="WAIT_MISSION");
 i.mission_fresh=true;assert(classify(i)=="UNKNOWN_MODE");
 i.mission_semantics_known=true;assert(classify(i)=="READY");
 i.race_state=1;assert(classify(i)=="RUNNING");
 i.race_state=2;assert(classify(i)=="ROUTE_DONE");
 i.plan_done=true;assert(classify(i)=="COMPLETE");
 i.mechanism_fault=true;assert(classify(i)=="FAULT");
 i.mechanism_fault=false;i.race_state=4;assert(classify(i)=="ABORTED");
 i.race_state=1;i.link_alive=false;assert(classify(i)=="LINK_LOST");
 std::cout<<"PASS: mission classification and route/full-plan distinction\n";
}
