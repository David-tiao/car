#pragma once
// Pure deterministic state classification, reused by ROS2 node and g++ unit test.
#include <string>
namespace car_mission {
struct Inputs{
 bool ever_link=false, link_alive=false,mission_fresh=false,mission_semantics_known=false;
 bool plan_done=false,mission_fault=false,mechanism_fault=false;
 unsigned race_state=0;
};
inline std::string classify(const Inputs&i){
 if(!i.ever_link)return "WAIT_LINK";
 if(!i.link_alive)return "LINK_LOST";
 if(!i.mission_fresh)return "WAIT_MISSION";
 if(!i.mission_semantics_known)return "UNKNOWN_MODE";
 if(i.mission_fault||i.mechanism_fault||i.race_state==3)return "FAULT";
 if(i.race_state==4)return "ABORTED";
 if(i.plan_done && i.race_state==2)return "COMPLETE";
 if(i.race_state==1)return "RUNNING";
 if(i.race_state==2)return "ROUTE_DONE"; // RACE_DONE != full plan completion
 return "READY";
}
}
