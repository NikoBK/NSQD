#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

// Client logic states (Finite State Machine)
#define GROUNDED_STATE 0
#define ARMED_STATE 1
#define HOVER_STATE 2
#define INITIALISE_ENROUTE_STATE 3
#define ENROUTE_STATE 4
#define ENROUTE_TURN_STATE 5
#define ENROUTE_STOPPED_STATE 6
#define PUBLISH_ANGLE_STATE 7
#define START_TEST_STATE 8
#define RUNNING_TEST_STATE 9
#define STOP_TEST_STATE 10

// Message IDs
#define ERROR_MSG 0
#define UPDATE_MSG 1
#define SET_AUTH_MSG 2
#define ARM_MSG 3
#define TAKEOFF_MSG 4
#define LAND_MSG 5
#define SET_PID_MSG 6
#define SET_RPYTFF_MSG 7
#define STOP_TEST_MSG 8

// MISC
#define REFRESH_RATE_HZ 50
#define SERVER_PORT 8888

// constants.hpp
#endif