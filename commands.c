#include <platform.h>
#include "lua/lua.h"
#include "lua/lauxlib.h"
#include "lua/lualib.h"

int cmd_pio_read(lua_State *l)
{
    lua_pushnumber(l, IORD(PIO_BASE, 0));
    return 1;
}


void commands_register(lua_State *l)
{
    lua_pushcfunction(l, cmd_pio_read);
    lua_setglobal(l, "pio_get");

}




