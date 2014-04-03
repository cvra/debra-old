#include <platform.h>
#include <stdio.h>
#include <uptime.h>
#include "lua/lua.h"
#include "lua/lauxlib.h"
#include "lua/lualib.h"
#include "cvra_cs.h"

int cmd_pio_read(lua_State *l)
{
    lua_pushnumber(l, IORD(PIO_BASE, 0));
    return 1;
}

int cmd_uptime_get(lua_State *l)
{
    lua_pushnumber(l, uptime_get());
    return 1;
}

int cmd_bluetooth_send(lua_State *l)
{
    if (lua_gettop(l) < 1)
        return 0;

    FILE *f = fopen("/dev/comDEBUG", "w");

    fprintf(f, "%s\n", lua_tostring(l, -1));
    fclose(f);

    return 0;
}

int cmd_mode(lua_State *l)
{
    char *mode;

    if (lua_gettop(l) < 1)
        return 0;

    mode = lua_tostring(l, -1);

    if(!strcmp("angle", mode))
        robot.mode = BOARD_MODE_ANGLE_ONLY;
    if(!strcmp("distance", mode))
        robot.mode = BOARD_MODE_DISTANCE_ONLY;
    if(!strcmp("off", mode))
        robot.mode = BOARD_MODE_FREE;
    if(!strcmp("all", mode))
        robot.mode = BOARD_MODE_ANGLE_DISTANCE;

    trajectory_hardstop(&robot.traj);

    return 0;
}

int cmd_position_get(lua_State *l)
{
    lua_pushnumber(l, position_get_x_float(&robot.pos));
    lua_pushnumber(l, position_get_y_float(&robot.pos));
    lua_pushnumber(l, position_get_a_rad_float(&robot.pos));
    return 3;
}

int cmd_encoders_get(lua_State *l)
{
    int32_t *adress;
    int channel;

    if (lua_gettop(l) < 2)
        return 0;

    adress = lua_touserdata(l, -2);
    channel = lua_tointeger(l, -1);

    lua_pushnumber(l, cvra_dc_get_encoder(adress, channel));

    return 1;
}

int cmd_forward(lua_State *l)
{
    int distance;
    if (lua_gettop(l) < 1)
        return 0;

    distance = lua_tointeger(l, -1);

    trajectory_d_rel(&robot.traj, distance);

    return 0; 
}


void commands_register(lua_State *l)
{
    lua_pushcfunction(l, cmd_pio_read);
    lua_setglobal(l, "pio_get");

    lua_pushcfunction(l, cmd_uptime_get);
    lua_setglobal(l, "uptime_get");

    lua_pushcfunction(l, cmd_position_get);
    lua_setglobal(l, "position_get");

    lua_pushcfunction(l, cmd_bluetooth_send);
    lua_setglobal(l, "log");

    lua_pushcfunction(l, cmd_mode);
    lua_setglobal(l, "mode");

    lua_pushcfunction(l, cmd_forward);
    lua_setglobal(l, "forward");

    lua_pushcfunction(l, cmd_encoders_get);
    lua_setglobal(l, "encoder_get");

    lua_pushlightuserdata(l, HEXMOTORCONTROLLER_BASE);
    lua_setglobal(l, "hexmotor");

    lua_pushlightuserdata(l, ARMSMOTORCONTROLLER_BASE);
    lua_setglobal(l, "armmotor");
}

