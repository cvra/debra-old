/** Registers all the command in a single file. */
#include <aversive.h>
#include <cvra_servo.h>
#include <aversive/error.h>
#include <2wheels/trajectory_manager_utils.h>
#include <string.h>
#include <uptime.h>
#include <cvra_dc.h>
#include <commandline.h>
#include <fcntl.h>
#include <cvra_pio.h>
#include <stdio.h>

#include "cvra_cs.h"
#include "arm_interpolators.h"
#include "arm.h"
#include "hardware.h"
#include "strat_utils.h"
#include <math.h>

static int calibration_done = 0;

/** Shutdowns the given arm. */
void cmd_arm_shutdown(int argc, char **argv)
{
    if(argc < 2) {
        arm_shutdown(&robot.right_arm);
        arm_shutdown(&robot.left_arm);
    } else {
        if(!strcmp(argv[1], "left"))
            arm_shutdown(&robot.left_arm);

        if(!strcmp(argv[1], "right"))
            arm_shutdown(&robot.right_arm);
    }
}

/** starts the strategy. */
void cmd_start()
{
#if 0
    if(!calibration_done) {
        printf("Niveau calibration des bras, ca se passe comment?\n");
        return;
    }
#endif

    printf("Pull starter to start the robot.");
    cvra_wait_starter_pull();
    strat_begin();
    printf("Match done. Hope you enjoyed it !\n");
}

/** Positions the robot at the beginning of a match. */
void cmd_autopos(int argc, char **argv)
{
    if(argc < 2) {
        printf("usage : %s [blue|red]\n", argv[0]);
        return;
    }

    if(!strcmp("red", argv[1]))
        strat.color = RED;
    if(!strcmp("blue", argv[1]))
        strat.color = BLUE;

    strat_autopos(170, 1255, COLOR_A(0), 119);

    trajectory_set_speed(&robot.traj, speed_mm2imp(&robot.traj, 700), speed_rd2imp(&robot.traj, 4.85) ); /* distance, angle */

    bd_set_thresholds(&robot.distance_bd,  7200, 1);
    bd_set_thresholds(&robot.angle_bd,  6000, 1);
}

/** Writes to a specific PWM. */
void cmd_pwm(int argc, char **argv)
{
    if(argc == 3) {
        printf("Putting channel %d = %d\n", atoi(argv[1]), atoi(argv[2]));
#ifdef COMPILE_ON_ROBOT
        cvra_dc_set_pwm(HEXMOTORCONTROLLER_BASE, atoi(argv[1]), atoi(argv[2]));
#endif
    }
}

/** Tests all 6 motors channel. */
void cmd_motor_test(int argc, char **argv)
{
    int *device = HEXMOTORCONTROLLER_BASE;
    int encoders_values[6];
    int i;
    int value = 500;


    if (argc > 1)
        if (!strcmp(argv[1], "arms"))
            device = ARMSMOTORCONTROLLER_BASE;
        else
            device = HEXMOTORCONTROLLER_BASE;

    if (argc > 2)
        value = atoi(argv[2]);

    for (i = 0; i < 6; i++) {
        encoders_values[i] = cvra_dc_get_encoder(device, i);
        cvra_dc_set_pwm(device, i, value);
    }

    strat_wait_ms(2000);

    printf("test results\n------------\n");

    for (i = 0; i < 6; i++) {
        if (encoders_values[i] == cvra_dc_get_encoder(device, i))
            printf("Channel %d FAIL.\n");
        else
            printf("Channel %d OK.\n");

        cvra_dc_set_pwm(device, i, 0);
    }

}

/** Gets the encoder values. */
void cmd_encoders(void)
{
#ifdef COMPILE_ON_ROBOT
    int i;
    for(i=0;i<6;i++)
        printf("%d;", cvra_dc_get_encoder(HEXMOTORCONTROLLER_BASE, i));
#endif
    printf("\n");
}

/** Setups PID. */
void cmd_pid(int argc, char **argv)
{
    if(argc < 2) {
        /* Show current gains. */
        printf("Distance : \tKp=%d\tGi=%d\tGd=%d\n", pid_get_gain_P(&robot.distance_pid),
                                          pid_get_gain_I(&robot.distance_pid),
                                          pid_get_gain_D(&robot.distance_pid));

        printf("Angle : \tKp=%d\tGi=%d\tGd=%d\n", pid_get_gain_P(&robot.angle_pid),
                                       pid_get_gain_I(&robot.angle_pid),
                                       pid_get_gain_D(&robot.angle_pid));

    } else if(argc < 5) {
            printf("usage: %s pid_name P I D\n", argv[0]);
    } else {
        struct pid_filter *pid;

        if(!strcmp(argv[1], "distance")) {
            pid =  &robot.distance_pid;
        } else if(!strcmp(argv[1], "angle")) {
            pid =  &robot.angle_pid;
        } else {
            printf("Unknown PID name : %s\n", argv[1]);
            return;
        }

        /** @todo We should be more cautious when handling user input. */
        pid_set_gains(pid, atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
    }
}

/** Gets or sets position. */
void cmd_position(int argc, char **argv)
{
    if(argc < 2) {
        /* Position get */
        printf("Position : (%d;%d;%d)\n", position_get_x_s16(&robot.pos), position_get_y_s16(&robot.pos), position_get_a_deg_s16(&robot.pos));
    } else {
        if(argc < 4) {
            printf("Usage : %s x y a_deg\n", argv[0]);
        } else {
            position_set(&robot.pos, atoi(argv[1]), atoi(argv[2]), atoi(argv[3]));
        }
    }
}


/** Show PID error. */
void cmd_error_calibrate(int argc, char **argv)
{
    int32_t start_time = uptime_get();
    int32_t time = uptime_get();
    if(argc < 2)
        return;

    trajectory_a_rel(&robot.traj, atof(argv[1]));

    while(!trajectory_finished(&robot.traj)) {
        time = uptime_get();
        printf("%d;%d\n", (time-start_time)/1000, cs_get_error(&robot.angle_cs));
        while(uptime_get() < time + 10000);
    }
}

/** Sets or gets right wheel gain. */
void cmd_right_gain(int argc, char **argv)
{
    /** @todo We should be more cautious when handling user input. */
    if(argc >= 3) {
        double a, b;
        a = atof(argv[1]);
        b = atof(argv[2]);
        rs_set_right_ext_encoder(&robot.rs, cvra_dc_get_encoder5, HEXMOTORCONTROLLER_BASE, -a);
        rs_set_left_ext_encoder(&robot.rs, cvra_dc_get_encoder0, HEXMOTORCONTROLLER_BASE, b);
    }
    printf("Right = %.4f Left = %.4f\n", robot.rs.right_ext_gain, robot.rs.left_ext_gain);
}

/** Lists all available commands. */
void cmd_help(void)
{
    int i;
    extern command_t commands_list[];
    for(i=0;commands_list[i].f!= NULL;i++) {
        printf("%s\t", commands_list[i].name);
        if(i > 0 && i%4 == 0)
            printf("\n");
    }
    printf("\n");
}

/** Gets error. */
void cmd_error_get(void)
{
    printf("angle %d\n distance %d\n", cs_get_error(&robot.angle_cs), cs_get_error(&robot.distance_cs));

}

/** Puts all PWM to max and measure the position. */
void cmd_acceleration_calibrate(void)
{
    printf("Press a key to go. Regulation task should be off.\n");
    getchar();

    int32_t start_time = uptime_get();
    int32_t time;


    rs_set_angle(&robot.rs, 475);

    /* Print it for 1s. */
    while((time = uptime_get()) < start_time + 2 * 1000000) {
        /* Dumps every 10 ms. */
        printf("%d;%d\n", (uptime_get() - start_time) / 1000, rs_get_angle(&robot.rs));
        while(uptime_get() < time + 10000);
        rs_update(&robot.rs);
    }

    rs_set_angle(&robot.rs, 0);
    return;
}


/** Dumps an error curve. */
void cmd_error_dump(int argc, char **argv)
{
    if(argc < 2)
        return;
    trajectory_a_rel(&robot.traj, atoi(argv[1]));

    int32_t start_time = uptime_get();
    int32_t time;
    int32_t previous_pos = rs_get_angle(&robot.rs);
    /* Print it for 5s. */
    while((time = uptime_get()) < start_time + 3* 1000000) {
        /* Dumps every 10 ms. */
        printf("%d;%d;%d\n", (uptime_get() - start_time) / 1000, (int)robot.angle_qr.previous_var, (int)rs_get_ext_angle(&robot.rs)-previous_pos);
        previous_pos = rs_get_ext_angle(&robot.rs);
        while(uptime_get() < time + 10000);
    }
}

/** Goes forward by a given distance. */
void cmd_forward(int argc, char **argv)
{
    if(argc < 2) {
        printf("Usage : %s distance_mm\n", argv[0]);
        return;
    }

    cvra_wait_starter_pull();
    trajectory_d_rel(&robot.traj, atoi(argv[1]));
}


/** Turns of a given angle. */
void cmd_turn(int argc, char **argv)
{
    cvra_wait_starter_pull();
    if(argc == 2)
        trajectory_a_rel(&robot.traj, atoi(argv[1]));
}

/** Test UART. */
void cmd_test_uart(int argc, char **argv)
{
    char path[20];
    FILE *file;
    if (argc < 2) return;
    sprintf(path, "/dev/%s", argv[1]);
    printf("Trying to open %s...", path);
    file = fopen(path, "w");
    if(file == NULL) {
        printf("[KO]\n");
        return;
    }


    printf("[OK]\n");

//    char data[256];
 //   fgets(data, 256, file);

    fprintf(file, "Hello world !\n");

    fclose(file);
}

/** Reads the robot_system state. */
void cmd_rs(void)
{
    int32_t start_time = uptime_get();
    int32_t time;
    /* Print it for 5s. */
    while((time = uptime_get()) < start_time + 5* 1000000) {
        printf("%d;%d;%d\n",(time-start_time)/1000, rs_get_ext_angle(&robot.rs), rs_get_ext_distance(&robot.rs));
        while(uptime_get() < time + 10000);
    }
}

/** Goes to a given (x,y) point. */
void cmd_goto(int argc, char **argv)
{
    if(argc < 3) {
        printf("Usage %s x y\n", argv[0]);
        return;
    }

    cvra_wait_starter_pull();
    strat_goto_avoid(atoi(argv[1]), atoi(argv[2]), END_TRAJ);
}

/** Puts the robot to a certain mode. */
void cmd_mode(int argc, char **argv)
{
    if(argc != 2)
        return;

    if(!strcmp("angle", argv[1]))
        robot.mode = BOARD_MODE_ANGLE_ONLY;
    if(!strcmp("distance", argv[1]))
        robot.mode = BOARD_MODE_DISTANCE_ONLY;
    if(!strcmp("off", argv[1]))
        robot.mode = BOARD_MODE_FREE;
    if(!strcmp("all", argv[1]))
        robot.mode = BOARD_MODE_ANGLE_DISTANCE;

    trajectory_hardstop(&robot.traj);
}

void cmd_shoulder_mode(int argc, char **argv)
{
    if(argc < 2) {
        printf("usage %s back|front\n", argv[0]);
        return;
    }

    if(!strcmp(argv[1], "front")) {
        robot.left_arm.shoulder_mode = SHOULDER_FRONT;
        robot.right_arm.shoulder_mode = SHOULDER_FRONT;
    } else {
        robot.left_arm.shoulder_mode = SHOULDER_BACK;
        robot.right_arm.shoulder_mode = SHOULDER_BACK;
    }

}


void cmd_do_stack(int argc, char **argv)
{
    float x, y, z;
    arm_t *arm = &robot.left_arm;

    arm_trajectory_t traj;
    arm_trajectory_init(&traj);
    arm_get_position(arm, &x, &y, &z);
    arm_interpolator_append_point(&traj, x, y, z, COORDINATE_ARM, 1.); // duration not used
    arm_interpolator_append_point(&traj, 200, 120, z, COORDINATE_TABLE, 1.);
    arm_interpolator_append_point(&traj, 200, 120, 17, COORDINATE_TABLE, 1.);
    arm_execute_movement(arm, &traj);
    while(!arm_trajectory_finished(arm));
    left_pump(1);
    getchar();


    arm_trajectory_init(&traj);
    arm_interpolator_append_point(&traj, 200, 120, 17, COORDINATE_TABLE, 5.); // duration not used
    arm_interpolator_append_point(&traj, 200, 120, 27, COORDINATE_TABLE, 5.); // duration not used
    arm_interpolator_append_point(&traj, 200, 200, 27, COORDINATE_TABLE, 4.);
    arm_interpolator_append_point(&traj, 200, 200, 17, COORDINATE_TABLE, 4.);
    arm_execute_movement(arm, &traj);
    while(!arm_trajectory_finished(arm));
    getchar();
    left_pump(-1);

    arm_trajectory_init(&traj);
    arm_interpolator_append_point(&traj, 200, 200, 17, COORDINATE_TABLE, 4.);
    arm_interpolator_append_point(&traj, 200, 200, z, COORDINATE_TABLE, 4.);
    arm_interpolator_append_point(&traj, 200, 120, z, COORDINATE_TABLE, 4.);
    arm_execute_movement(arm, &traj);
    while(!arm_trajectory_finished(arm));
    left_pump(-1);
    getchar();



    arm_trajectory_init(&traj);
    arm_get_position(arm, &x, &y, &z);
    arm_interpolator_append_point(&traj, x, y, z, COORDINATE_ARM, 1.); // duration not used
    arm_interpolator_append_point(&traj, 200, 120, z, COORDINATE_TABLE, 1.);
    arm_interpolator_append_point(&traj, 200, 120, 17, COORDINATE_TABLE, 1.);
    arm_execute_movement(arm, &traj);
    while(!arm_trajectory_finished(arm));
    left_pump(1);
    getchar();

    arm_trajectory_init(&traj);
    arm_interpolator_append_point(&traj, 200, 120, 17, COORDINATE_TABLE, 5.); // duration not used
    arm_interpolator_append_point(&traj, 200, 120, 193, COORDINATE_TABLE, 4.);
    arm_interpolator_append_point(&traj, 200, 200, 193, COORDINATE_TABLE, 4.);
    arm_interpolator_append_point(&traj, 200, 200, 183, COORDINATE_TABLE, 4.);
    arm_execute_movement(arm, &traj);

    while(!arm_trajectory_finished(arm));

    left_pump(0);
}

/** Places the arm. */
void cmd_place_arms(int argc, char **argv)
{
    if(argc != 2) {
        printf("usage : %s [blue|red]\n", argv[0]);
        return;
    }
    arm_trajectory_t traj;
    float x, y, z;

    arm_get_position(&robot.left_arm, &x, &y, &z);
    arm_trajectory_init(&traj);
    arm_interpolator_append_point(&traj, x, y, z, COORDINATE_ARM, 9.); // duration not used
    arm_interpolator_append_point(&traj, 0, -200, 197, COORDINATE_ARM, 3.);
    arm_interpolator_append_point(&traj, 200, 10, 197, COORDINATE_ARM, 3.);
//    arm_interpolator_append_point(&traj, 200, 10, 60, COORDINATE_ARM, 3.);
    arm_execute_movement(&robot.left_arm, &traj);


    arm_get_position(&robot.right_arm, &x, &y, &z);
    arm_trajectory_init(&traj);
    arm_interpolator_append_point(&traj, x, y, z, COORDINATE_ARM, 9.); // duration not used
    arm_interpolator_append_point(&traj, 0, 200, 197, COORDINATE_ARM, 3.);
    arm_interpolator_append_point(&traj, 200, 10, 197, COORDINATE_ARM, 3.);
    arm_execute_movement(&robot.right_arm, &traj);

    while(!arm_trajectory_finished(&robot.left_arm) && !arm_trajectory_finished(&robot.right_arm));
}

void cmd_demo(void)
{
    trajectory_d_rel(&robot.traj, 1300);
    wait_traj_end(END_TRAJ);
    trajectory_a_rel(&robot.traj, 180);
    wait_traj_end(END_TRAJ);

    trajectory_d_rel(&robot.traj, 1300);
    wait_traj_end(END_TRAJ);
    trajectory_a_rel(&robot.traj, -180);
}

void cmd_arm_pos()
{
    float x, y, z;


    arm_get_position(&robot.left_arm, &x, &y, &z);
    printf("left : x=%.1f, y=%.1f, z=%.1f\n", x, y, z);

    arm_get_position(&robot.right_arm, &x, &y, &z);
    printf("right : x=%.1f, y=%.1f, z=%.1f\n", x, y, z);
}

void cmd_servo(int argc, char **argv)
{
    if(argc < 3) {
        printf("usage : %s channel value\n", argv[0]);
        return;
    }

    printf("Setting %d to %d\n", atoi(argv[1]), atoi(argv[2]));

    IOWR(SERVOS_BASE, atoi(argv[1]), atoi(argv[2]));
}

void cmd_arm_goto(int argc, char **argv)
{
    arm_t *arm;
    arm_trajectory_t traj;

    float start[3], end[3];
    if(argc < 6) {
        printf("usage : %s [left#right] [arm#robot#table] x y z\n", argv[0]);
        return;
    }

    if(!strcmp("left", argv[1]))
        arm = &robot.left_arm;
    else
        arm = &robot.right_arm;


    arm_get_position(arm, &start[0],&start[1],&start[2]);

    printf("start %.1f %.1f %.1f\n", start[0], start[1],start[2]);

    arm_coordinate_t system;
    if(!strcmp("arm", argv[2]))
        system = COORDINATE_ARM;
    if(!strcmp("table", argv[2]))
        system = COORDINATE_TABLE;
    if(!strcmp("robot", argv[2]))
        system = COORDINATE_ROBOT;

    end[0] = (float)atoi(argv[3]);
    end[1] = (float)atoi(argv[4]);
    end[2] = (float)atoi(argv[5]);

    arm_trajectory_init(&traj);
    arm_interpolator_append_point(&traj, start[0], start[1], start[2], COORDINATE_ARM, 9.);
    arm_interpolator_append_point(&traj, end[0], end[1], end[2], system, 9.);
    arm_execute_movement(arm, &traj);
}

void cmd_arm_pid(int argc, char **argv)
{
    if(argc < 6)
        return;

    arm_t *arm;
    struct pid_filter *pid;

    if(!strcmp("left", argv[1]))
        arm = &robot.left_arm;
    if(!strcmp("right", argv[1]))
        arm = &robot.right_arm;

    if(!strcmp("shoulder", argv[2]))
        pid = &arm->shoulder_pid;
    if(!strcmp("elbow", argv[2]))
        pid = &arm->elbow_pid;
    if(!strcmp("z_axis", argv[2]))
        pid = &arm->z_axis_pid;

    pid_set_gains(pid, atoi(argv[3]), atoi(argv[4]), atoi(argv[5]));

    printf("Left arm :\n");
    printf("Z axis : %d %d %d\n",
            pid_get_gain_P(&robot.left_arm.z_axis_pid),
            pid_get_gain_I(&robot.left_arm.z_axis_pid),
            pid_get_gain_D(&robot.left_arm.z_axis_pid));
    printf("Shoulder : %d %d %d\n", pid_get_gain_P(&robot.left_arm.shoulder_pid),
            pid_get_gain_I(&robot.left_arm.shoulder_pid),
            pid_get_gain_D(&robot.left_arm.z_axis_pid));
    printf("Elbow : %d %d %d\n", pid_get_gain_P(&robot.left_arm.elbow_pid),
            pid_get_gain_I(&robot.left_arm.shoulder_pid),
            pid_get_gain_D(&robot.left_arm.z_axis_pid));


    printf("Right arm :\n");
    printf("Z axis : %d %d %d\n",
            pid_get_gain_P(&robot.right_arm.z_axis_pid),
            pid_get_gain_I(&robot.right_arm.z_axis_pid),
            pid_get_gain_D(&robot.right_arm.z_axis_pid));
    printf("Shoulder : %d %d %d\n", pid_get_gain_P(&robot.right_arm.shoulder_pid),
            pid_get_gain_I(&robot.right_arm.shoulder_pid),
            pid_get_gain_D(&robot.right_arm.z_axis_pid));
    printf("Elbow : %d %d %d\n", pid_get_gain_P(&robot.right_arm.elbow_pid),
            pid_get_gain_I(&robot.right_arm.shoulder_pid),
            pid_get_gain_D(&robot.right_arm.z_axis_pid));
}

void cmd_calibrate_arm()
{
    int start_time = uptime_get();
    printf("Place the arms in position, then wait 3 sec\n");

    while(uptime_get() < start_time + 3 * 1000000);

    arm_calibrate();
    calibration_done = 1;
}

void cmd_show_currents()
{
    int i=0;
    for(i=0;i<6;i++)
        printf("%d : %d\n", i, cvra_dc_get_current(ARMSMOTORCONTROLLER_BASE, i));

}

void cmd_pio_read(void)
{
    printf("pio : 0x%X\n", IORD(PIO_BASE, 0));

}

void cmd_pio_write(int argc, char **argv)
{
    if(argc != 2)
        return;
    if(argv[1][0] == '1') {
        IOWR(PIO_BASE, 0, 1 << 9);
    }
    else {
        IOWR(PIO_BASE, 0, 0x0000);
    }
}

void cmd_beacon(void)
{
    int i;
    int32_t start_time = uptime_get();

    while(uptime_get() - start_time < 30000000) {
        if(robot.beacon.nb_beacon != 0) {
            trajectory_a_rel(&robot.traj, robot.beacon.beacon[0].direction);
        }

        for(i=0;i<robot.beacon.nb_beacon;i++) {
            printf("Opp %d angle = %.1f distance = %.1f\n", i, robot.beacon.beacon[i].direction,
                    robot.beacon.beacon[i].distance);
        }
    }
}


void cmd_circle(int argc, char **argv)
{
    int radius, angle;
    if (argc < 3) {
        printf("usage : %s radius angle\n", argv[0]);
        return;
    }

    radius = atoi(argv[1]);
    angle = atoi(argv[2]);

    cvra_wait_starter_pull();

    /* Starts the circular trajectory. */
    trajectory_circle_rel(&robot.traj, 0, radius, radius, angle, FORWARD|TRIGO);

    /* XXX What is the stopping condition. */
    while (position_get_a_deg_s16(&robot.pos) >= 0);
    while (position_get_a_deg_s16(&robot.pos) <= 0);
    trajectory_hardstop(&robot.traj);
}

void cmd_calage_test(int argc, char **argv)
{
    int32_t start_angle;

    bd_set_thresholds(&robot.distance_bd,  5000, 2);

    trajectory_set_speed(&robot.traj, 100, 100);
    robot.mode = BOARD_MODE_DISTANCE_ONLY;

    // On recule jusqu'a  qu'on ait touche un mur
    trajectory_d_rel(&robot.traj, (double) -2000);

    while(!bd_get(&robot.distance_bd));
    trajectory_hardstop(&robot.traj);
    bd_reset(&robot.distance_bd);
    bd_reset(&robot.angle_bd);
    robot.mode = BOARD_MODE_ANGLE_DISTANCE;

    start_angle = rs_get_angle(&robot.rs);

    trajectory_d_rel(&robot.traj, 10);
    while(!trajectory_finished(&robot.traj));

    cvra_wait_starter_pull();


    robot.mode = BOARD_MODE_DISTANCE_ONLY;
    // On recule jusqu'a  qu'on ait touche un mur
    trajectory_d_rel(&robot.traj, (double) -2000);
    while(!bd_get(&robot.distance_bd));
    trajectory_hardstop(&robot.traj);
    bd_reset(&robot.distance_bd);
    bd_reset(&robot.angle_bd);

    start_angle -= rs_get_angle(&robot.rs);
    printf("delta [pulse] : %d\n", (int)start_angle);
    printf("delta [deg] : %f\n", DEG(pos_imp2rd(&robot.traj, start_angle)));
    printf("decalage sur 3m : %f\n", sin(pos_imp2rd(&robot.traj, start_angle)) * 3000);

}

void cmd_degage_arms(void)
{
}

void cmd_angle_calibrate(int argc, char **argv)
{
    int32_t start_angle, delta_angle;
    float factor;

    int count;
    if(argc < 2)
        count = 1;
    else
        count = atoi(argv[1]);


    trajectory_set_acc(&robot.traj, acc_mm2imp(&robot.traj, 160), acc_rd2imp(&robot.traj, 1.94));
    trajectory_set_speed(&robot.traj, 200, 200);


    cvra_wait_starter_pull();

    bd_set_thresholds(&robot.distance_bd,  5000, 2);

    robot.mode = BOARD_MODE_DISTANCE_ONLY;
    // On recule jusqu'a  qu'on ait touche un mur
    trajectory_d_rel(&robot.traj, (double) -2000);
    while(!bd_get(&robot.distance_bd));
    trajectory_hardstop(&robot.traj);
    bd_reset(&robot.distance_bd);
    bd_reset(&robot.angle_bd);
    robot.mode = BOARD_MODE_ANGLE_DISTANCE;
    start_angle = rs_get_angle(&robot.rs);

    trajectory_d_rel(&robot.traj, 100);
    while(!trajectory_finished(&robot.traj));

    trajectory_a_rel(&robot.traj, count*360);
    while(!trajectory_finished(&robot.traj));


    trajectory_d_rel(&robot.traj, -50);
    while(!trajectory_finished(&robot.traj));

    robot.mode = BOARD_MODE_DISTANCE_ONLY;
    // On recule jusqu'a  qu'on ait touche un mur
    trajectory_d_rel(&robot.traj, (double) -2000);
    while(!bd_get(&robot.distance_bd));
    trajectory_hardstop(&robot.traj);
    bd_reset(&robot.distance_bd);
    bd_reset(&robot.angle_bd);
    robot.mode = BOARD_MODE_ANGLE_DISTANCE;

    delta_angle = rs_get_angle(&robot.rs) - start_angle;
    delta_angle -= pos_rd2imp(&robot.traj, RAD(360*count));

    // if factor > 0, then the robot turns too much
    factor = (float)delta_angle / (float)(pos_rd2imp(&robot.traj,RAD(360*count)));
    factor = (1.+factor)*robot.pos.phys.track_mm;

    printf("delta [pulse] : %d\n", delta_angle);
    printf("delta [deg] : %.3f\n", DEG(pos_imp2rd(&robot.traj, delta_angle)));
    printf("Suggested track : %.8f [mm]\n", factor);
    printf("Old track : %.8f [mm]\n", robot.pos.phys.track_mm);
    printf("apply ? [Yn]\n");
    if(getchar()=='n')
        return;
    robot.pos.phys.track_mm = factor;
}

void cmd_wheel_calibrate(int argc, char **argv)
{
    int32_t start_angle, start_distance;
    int32_t delta_angle, delta_distance;
    float factor, left_gain, right_gain;

    int count;
    if(argc < 2)
        count = 1;
    else
        count = atoi(argv[1]);


    trajectory_set_acc(&robot.traj, acc_mm2imp(&robot.traj, 160), acc_rd2imp(&robot.traj, 1.94));

    cvra_wait_starter_pull();

    bd_set_thresholds(&robot.distance_bd,  5000, 2);
    trajectory_set_speed(&robot.traj, 200, 200);

    robot.mode = BOARD_MODE_DISTANCE_ONLY;

    // On recule jusqu'a  qu'on ait touche un mur
    trajectory_d_rel(&robot.traj, (double) -2000);
    while(!bd_get(&robot.distance_bd));
    trajectory_hardstop(&robot.traj);
    bd_reset(&robot.distance_bd);
    bd_reset(&robot.angle_bd);
    robot.mode = BOARD_MODE_ANGLE_DISTANCE;

    start_angle = rs_get_angle(&robot.rs);
    start_distance = rs_get_distance(&robot.rs);


    trajectory_d_rel(&robot.traj, 1200);
    while(!trajectory_finished(&robot.traj));
    trajectory_a_rel(&robot.traj, 180);
    while(!trajectory_finished(&robot.traj));
    trajectory_d_rel(&robot.traj, 1150);
    while(!trajectory_finished(&robot.traj));
    trajectory_a_rel(&robot.traj, -180);
    while(!trajectory_finished(&robot.traj));

    while(--count) {
        WARNING(0, "%d left !", count);
        trajectory_d_rel(&robot.traj, 1150);
        while(!trajectory_finished(&robot.traj));
        trajectory_a_rel(&robot.traj, 180);
        while(!trajectory_finished(&robot.traj));
        trajectory_d_rel(&robot.traj, 1150);
        while(!trajectory_finished(&robot.traj));
        trajectory_a_rel(&robot.traj, -180);
        while(!trajectory_finished(&robot.traj));
    }


    trajectory_d_rel(&robot.traj, -25);
    while(!trajectory_finished(&robot.traj));


    robot.mode = BOARD_MODE_DISTANCE_ONLY;
    // On recule jusqu'a  qu'on ait touche un mur
    trajectory_d_rel(&robot.traj, (double) -2000);
    while(!bd_get(&robot.distance_bd));
    trajectory_hardstop(&robot.traj);
    bd_reset(&robot.distance_bd);
    bd_reset(&robot.angle_bd);
    robot.mode = BOARD_MODE_ANGLE_DISTANCE;

    delta_angle = start_angle-rs_get_angle(&robot.rs);
    delta_distance = start_distance-rs_get_distance(&robot.rs);

    factor = (float)(delta_angle)/(float)(delta_distance);
    left_gain = (1. + factor) * robot.rs.left_ext_gain;
    right_gain = (1. - factor) * robot.rs.right_ext_gain;

    printf("angle difference : %f\n", DEG(pos_imp2rd(&robot.traj, delta_angle)));
    printf("suggested factors :\n");
    printf("left : %.8f (old gain was %f)\n", left_gain, robot.rs.left_ext_gain);
    printf("right : %.8f (old gain was %f)\n", right_gain, robot.rs.right_ext_gain);

    printf("apply those ? [Yn]\n");
    if(getchar()=='n')
        return;


    rs_set_left_ext_encoder(&robot.rs, cvra_dc_get_encoder0, HEXMOTORCONTROLLER_BASE,left_gain);
    rs_set_right_ext_encoder(&robot.rs, cvra_dc_get_encoder5, HEXMOTORCONTROLLER_BASE,right_gain);
}


void cmd_calibrate_cale(void)
{
    trajectory_set_acc(&robot.traj, acc_mm2imp(&robot.traj, 160), acc_rd2imp(&robot.traj, 1.94));

    cvra_wait_starter_pull();

    bd_set_thresholds(&robot.distance_bd,  5000, 2);
    trajectory_set_speed(&robot.traj, 200, 200);

    robot.mode = BOARD_MODE_DISTANCE_ONLY;
    // On recule jusqu'a  qu'on ait touche un mur
    trajectory_d_rel(&robot.traj, (double) -2000);
    while(!bd_get(&robot.distance_bd));
    trajectory_hardstop(&robot.traj);
    bd_reset(&robot.distance_bd);
    bd_reset(&robot.angle_bd);
    robot.mode = BOARD_MODE_ANGLE_DISTANCE;

    position_set(&robot.pos, 0., 0., 0.);
    trajectory_d_rel(&robot.traj, 50);
    while(!trajectory_finished(&robot.traj));
   // trajectory_a_rel(&robot.traj, 180);
    while(!trajectory_finished(&robot.traj));

    trajectory_goto_xy_abs(&robot.traj, 1500, 0);
    while(!trajectory_finished(&robot.traj));
  //  trajectory_a_rel(&robot.traj, -180);
}


/** An array of all the commands. Sort them by order of completion. */
command_t commands_list[] = {

//    COMMAND("acc_calibrate", cmd_acceleration_calibrate),
    COMMAND("place_arms", cmd_place_arms),
    COMMAND("test_argv",test_func),
    COMMAND("autopos", cmd_autopos),
    COMMAND("arm_shutdown",cmd_arm_shutdown),
    COMMAND("arm_pid", cmd_arm_pid),
    COMMAND("pio_write", cmd_pio_write),
    COMMAND("start",cmd_start),
    COMMAND("pio_read", cmd_pio_read),
    COMMAND("beacon", cmd_beacon),
    COMMAND("currents", cmd_show_currents),
    COMMAND("pid", cmd_pid),
    COMMAND("pwm", cmd_pwm),
    COMMAND("get_error", cmd_error_get),
    COMMAND("choc", cmd_error_calibrate),
    COMMAND("encoders", cmd_encoders),
    COMMAND("position", cmd_position),
    COMMAND("shoulder", cmd_shoulder_mode),
    COMMAND("forward", cmd_forward),
    COMMAND("servo", cmd_servo),
    COMMAND("correction", cmd_right_gain),
    COMMAND("autotest", cmd_motor_test),
    COMMAND("wheel_calibrate", cmd_wheel_calibrate),
    COMMAND("angle_calibrate", cmd_angle_calibrate),
    COMMAND("error", cmd_error_dump),
    COMMAND("help", cmd_help),
    COMMAND("turn", cmd_turn),
    COMMAND("rs", cmd_rs),
    COMMAND("goto", cmd_goto),
    COMMAND("arm_goto", cmd_arm_goto),
    COMMAND("demo", cmd_demo),
    COMMAND("calibrate_arm", cmd_calibrate_arm),
    COMMAND("cale", cmd_calibrate_cale),
    COMMAND("mode", cmd_mode),
    COMMAND("stack", cmd_do_stack),
    COMMAND("arm_pos", cmd_arm_pos),
    COMMAND("circle", cmd_circle),
    COMMAND("uart", cmd_test_uart),
    COMMAND("calage", cmd_calage_test),
    COMMAND("degage", cmd_degage_arms),
    COMMAND("none",NULL), /* must be last. */
};




