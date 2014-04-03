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

int cmd_position_get(lua_State *l)
{
    lua_pushnumber(l, position_get_x_float(&robot.pos));
    lua_pushnumber(l, position_get_y_float(&robot.pos));
    lua_pushnumber(l, position_get_a_rad_float(&robot.pos));
    return 3;
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
}

