/** @file arm.h
 * @author Antoine Albertelli, CVRA
 * @date 2013
 * @brief Toutes les fonctions liees a la nouvelle version des bras pour la coupe 2013.
 * @sa doc/APi Bras.odt
 */
#ifndef _ARM_H_
#define _ARM_H_

#include <control_system_manager.h>
#include <pid.h>
#include <blocking_detection_manager.h>
#include <vect2.h>
#include <polygon.h>

#include "keyframe.h"

/** @brief A forbidden volume for the arm.
 *
 * This structure contains all the required informations to create forbidden
 * volumes that the arms cannot go into. Those volumes are defined as right
 * prisms who point in the z-axis direction.
 */
typedef struct {
    arm_coordinate_t coordinate_system; /**< Coordinate system of the obstacle. */
    poly_t base; /**< The base polygon of the prism. */
    int height;  /**< The height of the polygon, in mm. */
} arm_obstacle_t;

/** @brief The shoulder position.
 *
 * This enum is used to define a shoulder mode for the robot, as in "The shoulder
 * should be in front of the hand or behind it ?".
 */
typedef enum {
    SHOULDER_FRONT,
    SHOULDER_BACK,
} shoulder_mode_t;

/** Struct holding everything that is related to a single arm.
 * \image html arm_change_coordinate.png "The offset_* parameters."
 * @todo Are the blocking managers really useful ? To be tested.
 * */
typedef struct {
    vect2_cart offset_xy; /**< Offset vector between center of robot and shoulder. */
    float offset_rotation; /**< Rotation between the robot base and shoulder in rad. */

    /* Control systems */
    struct cs z_axis_cs;    /**< Control System of Z axis. */
    struct cs shoulder_cs;  /**< Control System of the soulder. */
    struct cs elbow_cs;     /**< Control System of the elbow */

    struct pid_filter z_axis_pid;   /**< Z axis PID */
    struct pid_filter shoulder_pid; /**< Shoulder PID. */
    struct pid_filter elbow_pid;    /**< Elbow PID. */

    struct blocking_detection z_axis_bd;    /**< Z axis blocking detector. */
    struct blocking_detection shoulder_bd;  /**< Shoulder blocking detector. */
    struct blocking_detection elbow_bd;     /**< Elbow blocking detector. */

    /* Physical parameters. */
    int32_t z_axis_imp_per_mm;      /**< Z axis encoder impulsion per mm. */
    int32_t shoulder_imp_per_rad;   /**< Shoulder impulsions per rad. */
    int32_t elbow_imp_per_rad;      /**< Elbow impulsions per rad. */
    float length[2];                  /**< Length of the 2 arms elements. */

    /* Path informations */
    arm_trajectory_t trajectory;    /**< Current trajectory of the arm. */
    int32_t last_loop;              /**< Timestamp of the last loop execution, in us since boot. */

    shoulder_mode_t shoulder_mode;
} arm_t;

/** @brief Highlevel init of the arm.
 *
 * This function does all the high level init of the arm.
 */
void arm_highlevel_init(void);

/** Init all the regulators of a given arm.
 * @param [in, out] arm The arm to initialize.
 */
void arm_init(arm_t *arm);

/** Executes a trajectory on given arm.
 *
 * @param [in] arm The arm to move.
 * @param [in] traj An arm_trajectory_t with appropriately filled keyframes.
 *
 * @note Keyframes are supposed sorted from oldest to newest.
 *
 * @note The arm module keeps its own copy of the trajectory, so the caller
 * can free its own copy.
 *
 * @warning No verification on the trajectory is done prior to moving. Therefore
 * violent and uncontrolled movements can happen in case of bad prepared data.
 */
void arm_execute_movement(arm_t *arm, arm_trajectory_t *traj);

/** Manages an arm.
 *
 * This function takes care of an arm management. Its tasks are:
 * - Computing current point, using a linear interpolation of the trajectory.
 * - Computing inverse kinematics.
 * - Send the computed values to the control system.
 *
 * @param [in] a A pointer to an arm_t structure which was cast to a void *
 * to be compatible with the scheduler.
 *
 * @note This function should be called at a fixed rate, even if it integrates
 * some lag compensation
 *
 * @note This function doesn't do any regulation. You have to call arm_manage_cs
 * for the regulation to happen.
 *
 * @note If the current time is before the first point or after the last point,
 * the trajectory is clamped.
 *
 * @todo Handle all the different modes for the trajectories (table, etc...).
 * For now, only movement relatives to the arm base are supported.
 */
void arm_manage(void *a);

/** Manages an arm regulation.
 * @param [in] a A pointer to an arm_t structure, cast to void * for scheduler compatibility.
 *
 * @note arm_manage must be called to update control system consigns.
 * @note If there is no trajectory on the arm, regulation is disabled.
 */
void arm_manage_cs(void *a);

/** Tells if a trajectory is finished.
 *
 * This function tells us if an arm's trajectory is finished.
 *
 * @param [in] arm The arm pointer.
 * @returns 1 if the trajectory is finished, 0 otherwise.
 */
int arm_trajectory_finished(arm_t *arm);

/** Gets an arm position.
 * @param [in] arm The arm whom position we want.
 * @param [out] x,y,z The adresses where to stock the position.
 *
 * @note If one of the pointer is NULL, then the corresponding coordinate will
 * be discarded. Ex : arm_get_position(a, &x, NULL, NULL); will only get the X
 * coordinate.
 *
 * @todo Implement it.
 */
void arm_get_position(arm_t *arm, float *x, float *y, float *z);

/** Deletes a trajectory and cutoffs the motors.
 *
 * @param [in] arm The arm instance to shutdown.
 *
 * @warning This doesnt stop the arm, it justs cuts the power on the motors.
 *
 * @note calling arm_execute_movement power backs the arms.
 */
void arm_shutdown(arm_t *arm);

/** @brief Switches coordinates system.
 *
 * This function takes a point in any coordinate system and translates it to the
 * arm coordinate system for execution.
 *
 * \image html arm_change_coordinate.png "The different coordinate systems for the arms."
 *
 * @param [in] arm The instance of an arm.
 * @param [in] x,y The point in any coordinate system.
 * @param [in] system The coordinate system of the point.
 * @param [out] arm_x, arm_y The point in arm coordinates.
 */
void arm_change_coordinate_system(arm_t *arm, float x, float y,
             arm_coordinate_t system, float *arm_x, float *arm_y);


/** Tells the arms to assume they are at a known position. */
void arm_calibrate(void);


#endif
