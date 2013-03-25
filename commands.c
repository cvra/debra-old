/** Registers all the command in a single file. */
#include <aversive.h>
#include "adresses.h"
#include <commandline.h>
#include <string.h>
#include "cvra_cs.h"
#include <uptime.h>
#include <cvra_dc.h>

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

/** Dumps an error curve. */
void cmd_error_dump(int argc, char **argv) {
    if(argc < 2) return;
    trajectory_d_rel(&robot.traj, atoi(argv[1]));

    int32_t start_time = uptime_get();
    int32_t time;
    /* Print it for 5s. */
    while((time = uptime_get()) < start_time + 5* 1000000) { 
        /* Dumps every 10 ms. */
        printf("%d;%d\n", uptime_get() / 1000, cs_get_error(&robot.distance_cs));
        while(uptime_get() < time + 10000);
    }
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
    trajectory_goto_forward_xy_abs(&robot.traj, atoi(argv[1]), atoi(argv[2]));
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


/** An array of all the commands. */
command_t commands_list[] = {
    COMMAND("test_argv",test_func),
    COMMAND("arm_shutdown",cmd_arm_shutdown),
//    COMMAND("reset", cmd_reset),
    COMMAND("start",cmd_start),
    COMMAND("pid", cmd_pid), 
    COMMAND("pwm", cmd_pwm),
    COMMAND("encoders", cmd_encoders),
    COMMAND("position", cmd_position),
    COMMAND("forward", cmd_forward),
    COMMAND("correction", cmd_right_gain),
    COMMAND("error", cmd_error_dump),
    COMMAND("help", cmd_help),
    COMMAND("turn", cmd_turn),
    COMMAND("rs", cmd_rs),
    COMMAND("goto", cmd_goto),
    COMMAND("demo", cmd_demo),
    COMMAND("mode", cmd_mode),
    COMMAND("none",NULL), /* must be last. */
};
 



