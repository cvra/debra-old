#include <commandline.h>
#include <string.h>
#include "cvra_cs.h"
#include <uptime.h>

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
    if(argc == 2)
        robot.rs.right_ext_gain = atof(argv[1]);
    printf("Right wheel gain = %.4f\n", robot.rs.right_ext_gain);
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
void cmd_error_dump(void) {
    trajectory_d_rel(&robot.traj, 200);
    while(!trajectory_finished(&robot.traj)) {
        /* Dumps every 10 ms. */
        if(uptime_get() % 10000 == 0) {
            printf("%d;%d\n", uptime_get() / 1000, cs_get_error(&robot.distance_cs));
        }
    }
}

/** An array of all the commands. */
command_t commands_list[] = {
    COMMAND("test_argv",test_func),
    COMMAND("arm_shutdown",cmd_arm_shutdown),
    COMMAND("reset", cmd_reset),
    COMMAND("start",cmd_start),
    COMMAND("pid", cmd_pid), 
    COMMAND("position", cmd_position),
    COMMAND("right_gain", cmd_right_gain),
    COMMAND("error", cmd_error_dump),
    COMMAND("help", cmd_help),
    COMMAND("none",NULL), /* must be last. */
};




