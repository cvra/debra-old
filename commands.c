/** Registers all the command in a single file. */
#include <aversive.h>
#include <cvra_servo.h>
#include <aversive/error.h>
#include <2wheels/trajectory_manager_utils.h>
#include <string.h>
#include <uptime.h>
#include <cvra_dc.h>
#include <commandline.h>

#include "adresses.h"
#include "cvra_cs.h"
#include "arm_interpolators.h"
#include "arm.h"

/** Prints all args, then exits. */
void test_func(int argc, char **argv) {
    int i;
    for(i=0;i<argc;i++)
        printf("argv[%d] = \"%s\"\n", i, argv[i]);
}

/** Shutdowns the given arm. */
void cmd_arm_shutdown(int argc, char **argv) {
    if(argc < 2) {
        arm_shutdown(&robot.right_arm);
        arm_shutdown(&robot.left_arm);
    }
    else {
        if(!strcmp(argv[1], "left"))
            arm_shutdown(&robot.left_arm);

        if(!strcmp(argv[1], "right"))
            arm_shutdown(&robot.right_arm);
    }
}

/** resets the robot. */
void cmd_reset(void) {
    reset();
}

/** starts the strategy. */
void cmd_start() {
    printf("Press a key to start the robot.\n");
    getchar();
    strat_begin();
    printf("Match done. Hope you enjoyed it !\n");
}

/** Positions the robot at the beginning of a match. */
void cmd_autopos(int argc, char **argv) {
    if(argc < 2) {
        printf("usage : %s [blue|red]\n", argv[0]);
        return;
    }

    if(!strcmp("red", argv[1])) strat.color = RED;
    if(!strcmp("blue", argv[1])) strat.color = BLUE;

    strat_autopos(230, 1255, COLOR_A(-10), 119);

	trajectory_set_speed(&robot.traj, speed_mm2imp(&robot.traj, 700), speed_rd2imp(&robot.traj, 2*M_PI)); 

	bd_set_thresholds(&robot.distance_bd,  3000, 1);
	bd_set_thresholds(&robot.angle_bd,  1200, 1);

    // XXX
    return;
    arm_trajectory_t left_traj, right_traj;
    float x, y,z ;

    arm_get_position(&robot.left_arm, &x, &y, &z);

    arm_trajectory_init(&left_traj);
    arm_interpolator_append_point(&left_traj, x, y, z, COORDINATE_ARM, 9.); // duration not used 
    arm_interpolator_append_point(&left_traj, 100., -100., 197, COORDINATE_ARM, 1.);
    arm_interpolator_append_point(&left_traj, 100., -100., 100, COORDINATE_ARM, 2.);

    arm_trajectory_init(&right_traj);
    arm_get_position(&robot.right_arm, &x, &y, &z);

    arm_interpolator_append_point(&right_traj, x, y, z, COORDINATE_ARM, 9.); // duration not used 
    arm_interpolator_append_point(&right_traj, 100., 100., 197, COORDINATE_ARM, 1.); 
    arm_interpolator_append_point(&right_traj, 100., -100., 197, COORDINATE_ARM, 1.);
//    arm_interpolator_append_point(&right_traj, -150., -50., 197, COORDINATE_ARM, 1.);
    
    printf("length : %d\n", (int)right_traj.frame_count);

    int i;
    printf("now = %d\n", (int)uptime_get());
    for(i=0;i<right_traj.frame_count;i++) {
        printf("frame %d\t (%.1f;%.1f;%.1f)\t date = %d\n", i, right_traj.frames[i].position[0],right_traj.frames[i].position[1],right_traj.frames[i].position[2], (int)right_traj.frames[i].date);

    } 

    printf("God bless this poor robot... executing trajectory.\n");
   // arm_execute_movement(&robot.left_arm, &left_traj);
   //
   // robot.right_arm.shoulder_mode = SHOULDER_FRONT;
   // arm_execute_movement(&robot.right_arm, &right_traj);
}


/** Grabs a glass. */
void cmd_grab(int argc, char **argv) {

    arm_trajectory_t traj;
    if(argc < 3)
        return;

    float start[3], end[3];

    start[0] = (float)atoi(argv[1]);
    start[1] = (float)atoi(argv[2]);
    start[2] = 265.;

    end[0] = (float)atoi(argv[1]);
    end[1] = (float)atoi(argv[2]);
    end[2] = 100.;

    /* Descend. */
    //arm_interpolator_linear_motion(&traj, start, end, 3.);
    //arm_execute_movement(&robot.left_arm, &traj);
    while(!arm_trajectory_finished(&robot.left_arm)); 

//    cvra_dc_set_pwm1(HEXMOTORCONTROLLER_BASE, 400);

    /* Remonte */
//    arm_interpolator_linear_motion(&traj, end, start, 3.);
    arm_execute_movement(&robot.left_arm, &traj);
    while(!arm_trajectory_finished(&robot.left_arm)); 

    /* Bouge vers l'arriere. */
    end[0]  = 55;
    end[1] = 55;
    end[2] = 265.;

 //   arm_interpolator_linear_motion(&traj, start, end, 3.);
    arm_execute_movement(&robot.left_arm, &traj);
    while(!arm_trajectory_finished(&robot.left_arm)); 
//    cvra_dc_set_pwm1(HEXMOTORCONTROLLER_BASE, 0);

}

/** Writes to a specific PWM. */
void cmd_pwm(int argc, char **argv) {
    if(argc == 3) {
        printf("Putting channel %d = %d\n", atoi(argv[1]), atoi(argv[2]));
#ifdef COMPILE_ON_ROBOT
        cvra_dc_set_pwm(HEXMOTORCONTROLLER_BASE, atoi(argv[1]), atoi(argv[2]));
#endif
    }
}

/** Gets the encoder values. */
void cmd_encoders(void) {
#ifdef COMPILE_ON_ROBOT
    int i;
    for(i=0;i<6;i++)
        printf("%d;", cvra_dc_get_encoder(HEXMOTORCONTROLLER_BASE, i));
#endif
    printf("\n");
}

/** Setups PID. */
void cmd_pid(int argc, char **argv) {
    if(argc < 2) {
        /* Show current gains. */
        printf("Distance : \tKp=%d\tGi=%d\tGd=%d\n", pid_get_gain_P(&robot.distance_pid), 
                                          pid_get_gain_I(&robot.distance_pid),
                                          pid_get_gain_D(&robot.distance_pid));

        printf("Angle : \tKp=%d\tGi=%d\tGd=%d\n", pid_get_gain_P(&robot.angle_pid), 
                                       pid_get_gain_I(&robot.angle_pid),
                                       pid_get_gain_D(&robot.angle_pid));

    }
    else if(argc < 5) {
            printf("usage: %s pid_name P I D\n", argv[0]);
    } 
    else {
        struct pid_filter *pid;

        if(!strcmp(argv[1], "distance")) pid =  &robot.distance_pid;
        else if(!strcmp(argv[1], "angle")) pid =  &robot.angle_pid;
        else {
            printf("Unknown PID name : %s\n", argv[1]);
            return;
        }

        /** @todo We should be more cautious when handling user input. */
        pid_set_gains(pid, atoi(argv[2]), atoi(argv[3]), atoi(argv[4])); 
    }
}

/** Gets or sets position. */
void cmd_position(int argc, char **argv) {
    if(argc < 2) {
        /* Position get */
        printf("Position : (%d;%d;%d)\n", position_get_x_s16(&robot.pos), position_get_y_s16(&robot.pos), position_get_a_deg_s16(&robot.pos)); 
    }
    else {
        if(argc < 4) {
            printf("Usage : %s x y a_deg\n", argv[0]); 
        }
        else {
            position_set(&robot.pos, atoi(argv[1]), atoi(argv[2]), atoi(argv[3]));
        }
    }
}


/** Show PID error. */
void cmd_error_calibrate(int argc, char **argv) {
	int32_t start_time = uptime_get();
	int32_t time = uptime_get();
	if(argc < 2) return;

	trajectory_a_rel(&robot.traj, atof(argv[1]));

	while(!trajectory_finished(&robot.traj)) {
		time = uptime_get();
		printf("%d;%d\n", (time-start_time)/1000, cs_get_error(&robot.angle_cs));
		while(uptime_get() < time + 10000);
	}
}

/** Setups arm shoulder mode. */
void cmd_arm_shoulder_mode(int argc, char **argv) {
    if(argc < 2)
        return;

    if(!strcmp("front", argv[1])) {
        robot.left_arm.shoulder_mode = SHOULDER_FRONT;
        robot.right_arm.shoulder_mode = SHOULDER_FRONT;
    }
    else {
        robot.left_arm.shoulder_mode = SHOULDER_BACK;
        robot.right_arm.shoulder_mode = SHOULDER_BACK;
    }

}

/** Sets or gets right wheel gain. */
void cmd_right_gain(int argc, char **argv) {
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
void cmd_help(void) {
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
void cmd_error_get(void) {
    printf("angle %d\n distance %d\n", cs_get_error(&robot.angle_cs), cs_get_error(&robot.distance_cs));

}

/** Dumps an error curve. */
void cmd_error_dump(int argc, char **argv) {
    if(argc < 2) return;
    trajectory_a_rel(&robot.traj, atoi(argv[1]));

    int32_t start_time = uptime_get();
    int32_t time;
    int32_t previous_pos = rs_get_angle(&robot.rs);
    int left_current_max =0 , right_current_max = 0;
    /* Print it for 5s. */
    while((time = uptime_get()) < start_time + 3* 1000000) {
        /* Dumps every 10 ms. */
        printf("%d;%d;%d\n", (uptime_get() - start_time) / 1000, (int)robot.angle_qr.previous_var, (int)rs_get_ext_angle(&robot.rs)-previous_pos);
        previous_pos = rs_get_ext_angle(&robot.rs);
        if(cvra_dc_get_current(HEXMOTORCONTROLLER_BASE, 4) > left_current_max)
        	left_current_max = cvra_dc_get_current(HEXMOTORCONTROLLER_BASE, 4);

        if(cvra_dc_get_current(HEXMOTORCONTROLLER_BASE, 2) > right_current_max)
        	right_current_max = cvra_dc_get_current(HEXMOTORCONTROLLER_BASE, 2);
        while(uptime_get() < time + 10000);
    }

    printf("max current = %d;%d\n", left_current_max, right_current_max);
}

/** Goes forward by a given distance. */
void cmd_forward(int argc, char **argv) {
    if(argc < 2) {
        printf("Usage : %s distance_mm\n", argv[0]);
        return;
    }

    trajectory_d_rel(&robot.traj, atoi(argv[1])); 
}

/** Turns of a given angle. */
void cmd_turn(int argc, char **argv) {
    if(argc == 2)
        trajectory_a_rel(&robot.traj, atoi(argv[1]));

}

/** Reads the robot_system state. */
void cmd_rs(void) {
    int32_t start_time = uptime_get();
    int32_t time;
    /* Print it for 5s. */
    while((time = uptime_get()) < start_time + 5* 1000000) { 
        printf("%d;%d;%d\n",(time-start_time)/1000, rs_get_ext_angle(&robot.rs), rs_get_ext_distance(&robot.rs));
        while(uptime_get() < time + 10000);
    }
}

/** Goes to a given (x,y) point. */
void cmd_goto(int argc, char **argv) {
    if(argc < 3) {
        printf("Usage %s x y\n", argv[0]);
        return;
    }
    trajectory_goto_forward_xy_abs(&robot.traj, atoi(argv[1]), COLOR_Y(atoi(argv[2])));
}

/** Puts the robot to a certain mode. */
void cmd_mode(int argc, char **argv) {
    if(argc != 2) return;

    if(!strcmp("angle", argv[1])) robot.mode = BOARD_MODE_ANGLE_ONLY;
    if(!strcmp("distance", argv[1])) robot.mode = BOARD_MODE_DISTANCE_ONLY;
    if(!strcmp("off", argv[1])) robot.mode = BOARD_MODE_FREE;
    if(!strcmp("all", argv[1])) robot.mode = BOARD_MODE_ANGLE_DISTANCE;

    trajectory_hardstop(&robot.traj);
}

void cmd_demo(void) {
    trajectory_d_rel(&robot.traj, 1300);
    wait_traj_end(END_TRAJ);
    trajectory_a_rel(&robot.traj, 180);
    wait_traj_end(END_TRAJ);

    trajectory_d_rel(&robot.traj, 1300);
    wait_traj_end(END_TRAJ);
    trajectory_a_rel(&robot.traj, -180);
}

void cmd_arm_pos() {
    printf("Left : %.1f mm %.1f deg %.1f deg\n",  
            (double)cvra_dc_get_encoder(ARMSMOTORCONTROLLER_BASE, 1) / robot.left_arm.z_axis_imp_per_mm,
            (double)cvra_dc_get_encoder(ARMSMOTORCONTROLLER_BASE, 0) / robot.left_arm.shoulder_imp_per_rad * 180./M_PI ,
            (double)cvra_dc_get_encoder(ARMSMOTORCONTROLLER_BASE, 2) / robot.left_arm.elbow_imp_per_rad * 180./M_PI);

    printf("right : %.1f mm %.1f deg %.1f deg\n",  
            (double)cvra_dc_get_encoder(ARMSMOTORCONTROLLER_BASE, 4) / robot.right_arm.z_axis_imp_per_mm,
            (double)cvra_dc_get_encoder(ARMSMOTORCONTROLLER_BASE, 5) / robot.right_arm.shoulder_imp_per_rad * 180./M_PI ,
            (double)cvra_dc_get_encoder(ARMSMOTORCONTROLLER_BASE, 3) / robot.right_arm.elbow_imp_per_rad * 180./M_PI);
    printf("\n");
}

void cmd_servo(int argc, char **argv) {
    if(argc < 3) {
        printf("usage : %s channel value\n", argv[0]);
        return;
    }

    printf("Setting %d to %d\n", atoi(argv[1]), atoi(argv[2]));

    IOWR(SERVOS_BASE, atoi(argv[1]), atoi(argv[2]));
}

void cmd_arm_goto(int argc, char **argv) {
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
    if(!strcmp("arm", argv[2])) system = COORDINATE_ARM;
    if(!strcmp("table", argv[2])) system = COORDINATE_TABLE;
    if(!strcmp("robot", argv[2])) system = COORDINATE_ROBOT;

    end[0] = (float)atoi(argv[3]);
    end[1] = (float)atoi(argv[4]);
    end[2] = (float)atoi(argv[5]);

    arm_trajectory_init(&traj);
    arm_interpolator_append_point(&traj, start[0], start[1], start[2], COORDINATE_ARM, 9.);
    arm_interpolator_append_point(&traj, end[0], end[1], end[2], system, 9.);
    arm_execute_movement(arm, &traj);

    return;

    int32_t time = uptime_get();
    int32_t start_time = time;

    while(uptime_get() < start_time + 2 * 1000000) {
        time = uptime_get();
        printf("%d;%d;%d;%d\n", (time-start_time)/1000, cs_get_filtered_feedback(&arm->z_axis_cs), cs_get_filtered_consign(&arm->z_axis_cs), cvra_dc_get_current(ARMSMOTORCONTROLLER_BASE, 1)); 
        while(uptime_get() < time + 10000);
    }

}

void cmd_arm_pid(int argc, char **argv) {
    if(argc < 6) return;

    arm_t *arm;
    struct pid_filter *pid;
    
    if(!strcmp("left", argv[1])) arm = &robot.left_arm;
    if(!strcmp("right", argv[1])) arm = &robot.right_arm;

    if(!strcmp("shoulder", argv[2])) pid = &arm->shoulder_pid;
    if(!strcmp("elbow", argv[2])) pid = &arm->elbow_pid;
    if(!strcmp("z_axis", argv[2])) pid = &arm->z_axis_pid;

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

void cmd_calibrate_arm() {
    printf("Place the arms in position, then press a key\n");
    getchar();

    arm_calibrate();
}

void cmd_show_currents() {
    int i=0;
    for(i=0;i<6;i++)
        printf("%d : %d\n", i, cvra_dc_get_current(ARMSMOTORCONTROLLER_BASE, i));

}


/** An array of all the commands. */
command_t commands_list[] = {
    COMMAND("test_argv",test_func),
    COMMAND("arm_shutdown",cmd_arm_shutdown),
    COMMAND("arm_pid", cmd_arm_pid),
//    COMMAND("reset", cmd_reset),
    COMMAND("start",cmd_start),
    COMMAND("autopos", cmd_autopos),

    COMMAND("currents", cmd_show_currents),
    COMMAND("pid", cmd_pid), 
    COMMAND("pwm", cmd_pwm),
    COMMAND("get_error", cmd_error_get),
    COMMAND("choc", cmd_error_calibrate),
    COMMAND("encoders", cmd_encoders),
    COMMAND("position", cmd_position),
    COMMAND("grab", cmd_grab),
    COMMAND("shoulder_mode", cmd_arm_shoulder_mode),
    COMMAND("forward", cmd_forward),
    COMMAND("servo", cmd_servo),
    COMMAND("correction", cmd_right_gain),
    COMMAND("error", cmd_error_dump),
    COMMAND("help", cmd_help),
    COMMAND("turn", cmd_turn),
    COMMAND("rs", cmd_rs),
    COMMAND("goto", cmd_goto),
    COMMAND("arm_goto", cmd_arm_goto),
    COMMAND("demo", cmd_demo),
    COMMAND("calibrate_arm", cmd_calibrate_arm),
    COMMAND("mode", cmd_mode),
    COMMAND("arm_pos", cmd_arm_pos),
    COMMAND("none",NULL), /* must be last. */
};
 



