#ifndef BDI_COMMUNICATIONS_H_
#define BDI_COMMUNICATIONS_H_

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"

namespace BDICommunications
{   
    /*
        Response message to find out the result of a CHECK belief op
        among different agents
    */
    typedef struct{
        ros2_bdi_interfaces::msg::Belief belief;
        bool accepted; // queried agent has confirmed that info is obtainable by requesting agent
        bool found; // belief has been found within the belief set of the queried agent
    } CheckBeliefResult;


    /*
        Response message to find out the result of a CHECK desire op
        among different agents
    */
    typedef struct{
        ros2_bdi_interfaces::msg::Desire desire;
        bool accepted;// queried agent has confirmed that info is obtainable by requesting agent
        bool found;// desire has been found within the belief set of the queried agent
    } CheckDesireResult;


    // upd operation requested among diff. agents wrt. their belief/desire sets
    typedef enum {ADD, DEL} UpdOperation;

    /*
        Response message to find out the result of a WRITE belief op
        among different agents
    */
    typedef struct{
        ros2_bdi_interfaces::msg::Belief belief;// belief to be added/modified/deleted
        UpdOperation op;//ADD, DEL
        bool accepted;// queried agent has confirmed that requested upd. has been accepted
        bool performed;// requested upd. has been effectively performed
    } UpdBeliefResult;


    /*
        Response message to find out the result of a WRITE desire op
        among different agents
    */
    typedef struct{
        ros2_bdi_interfaces::msg::Desire desire;// desire to be added/deleted
        UpdOperation op;//ADD, DEL
        bool accepted;// queried agent has confirmed that requested upd. has been accepted
        bool performed;// requested upd. has been effectively performed
    } UpdDesireResult;
};

#endif  // BDI_COMMUNICATIONS_H_