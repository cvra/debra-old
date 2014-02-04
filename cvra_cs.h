/**
 @file cvra_cs.c
 @author Antoine Albertelli
 @date 19th September 2009
 @todo Deplacer toutes les constantes comme TRACK_MM dans strat.c:initRobot().
*/

#ifndef CVRA_CS_H
#define CVRA_CS_H

#include <aversive.h>
#include <aversive/queue.h>
#include <2wheels/trajectory_manager.h>
#include <2wheels/robot_system.h>
#include <2wheels/position_manager.h>
#include <control_system_manager.h>
#include <cvra_beacon.h>
#include <pid.h>
#include <quadramp.h>
#include <blocking_detection_manager.h>

#include <cvra_adc.h>

#include <obstacle_avoidance.h>

#include "arm.h"
#include "strat.h"
#include "cvra_param_robot.h"



/**Distance between wheels */
#define TRACK_MM 271.8

/** Wheel diameter */
#define WHEEL_DIAM_MM 50.0

/** Number of ipulsions on a rotation of the wheel */
#define IMP_PER_TURN 32000.0

/** Frequency of the regulation loop (in Hz) */
#define ASSERV_FREQUENCY 100

/**
 @brief Type of the regulators

 This enum holds the type of regulators : Angle and distance, distance only,
 angle only, etc...
 @note Some values are unused :
 - BOARD_MODE_INDEPENDENT
 - BOARD_MODE_FREE
 - BOARD_MODE_OTHER
 */
enum board_mode_t {
    BOARD_MODE_ANGLE_DISTANCE, ///< Angle & Distance regulated
    BOARD_MODE_ANGLE_ONLY,     ///< Angle regulated only
    BOARD_MODE_DISTANCE_ONLY,  ///< Distance regulated only
    BOARD_MODE_INDEPENDENT,    ///< 3 independent axis
    BOARD_MODE_FREE,           ///< No control system
    BOARD_MODE_SET_PWM,
};

/**
 @brief Trajectory type

 This enum is used to store informations about the trajectory type, like "x,y
 forward only" or "angle only", etc...

 */
enum trajectory_type_t {
    TRAJECTORY_TYPE_XY_ALL,						///< Chemin direct, en avant ou en arriere
    TRAJECTORY_TYPE_XY_FORWARD,					///< Chemin direct, en avant
    TRAJECTORY_TYPE_XY_BACKWARD,				///< Chemin direct, en arriere
    TRAJECTORY_TYPE_ANGLE_ONLY,					///< Orientation vers un angle
    TRAJECTORY_TYPE_DISTANCE_FORWARD_ONLY,		///< Marche avant
    TRAJECTORY_TYPE_DISTANCE_BACKWARD_ONLY,		///< Marche arriere
    TRAJECTORY_TYPE_XY_AVOID,					///< Trajectoire d'evitement, en marche avant
};

/**
 @brief contains all global vars.

 This structure contains all vars that should be global. This is a clean way to
 group all vars in one place. It also serve as a namespace.
 */
struct _rob {
    uint8_t verbosity_level;				///< @deprecated Contient le niveau de debug du robot.

    struct robot_system rs;                 ///< Robot system (angle & distance).
    struct robot_position pos;              ///< Position manager.
    struct cs angle_cs;                     ///< Control system manager for angle.
    struct cs distance_cs;                  ///< Control system manager for distance.
    struct pid_filter angle_pid;            ///< Angle PID filter.
    struct pid_filter distance_pid;         ///< Distance PID filter.
    struct quadramp_filter angle_qr;        ///< Angle quadramp
    struct quadramp_filter distance_qr;     ///< Distance quadramps.
    struct trajectory traj;                 ///< Trivial trajectory manager.
    struct blocking_detection angle_bd;     ///< Angle blocking detection manager.
    struct blocking_detection distance_bd;  ///< Distance blocking detection manager.

    volatile cvra_beacon_t beacon;

    enum board_mode_t mode;                 ///< The current board mode. @deprecated


    uint8_t is_aligning:1;                  ///< =1 if the robot is aligning on border

    uint8_t error_dump_enabled:1;           ///< =1 if infos should be dumped
    uint8_t is_blocked:1;                   ///< =1 if the robot got blocked
    uint8_t avoiding_enabled:1;
    uint8_t askLog;

    arm_t left_arm;					///< Structure representant le bras gauche.
    arm_t right_arm;					///< Structure representant le bras droit.

};


/**
 @brief Contient toutes les variables globales.

 Cette structure sert en quelque sorte de namespace pour les variables globales,
 qui peuvent ensuite etre accedees en faisant robot.color par exemple.
 */
extern struct _rob robot;

/**
 @brief Inits all the regulation related modules.

 This function inits and setups the following modules :
 - robot_system
 - encoders_cvra
 - position_manager
 - control_system_manager
 - pid
 - quadramp (acceleration and speed ramps)
 - trajectory_manager
 - blocking_detection_manager
 - couple_limiter
 */
void cvra_cs_init(void);

#endif /* CVRA_CS_H */
