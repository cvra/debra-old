#include <lwip/sockets.h>
#include <lwip/api.h>
#include <lwip/inet.h>

#ifdef __unix__
#include <stdlib.h>
#include <string.h>
#endif

#include "lua/lua.h"
#include "lua/lauxlib.h"
#include "lua/lualib.h"

#define MAX_COMMAND_LEN 200
#define PROMPT ">> "

// simple example of function binding from C to lua
int mysum(lua_State *l)
{
    if (lua_gettop(l) != 2 || !lua_isnumber(l, -1) || !lua_isnumber(l, -2))
        return 0;

    double a, b;

    // first parameters are top of stack
    a = lua_tonumber(l, -1);
    b = lua_tonumber(l, -2);

    lua_pushnumber(l, a+b);

    return 1; // return value count
}

int print_func(lua_State *l)
{
    struct netconn *conn;
    const char *to_print;

    if (lua_gettop(l) == 0)
        return 0;

    lua_getglobal(l, "__conn");
    conn = lua_touserdata(l, -1);
    lua_pop(l, 1);

    if (lua_isstring(l, -1)) {
        to_print = lua_tostring(l, -1);
        netconn_write(conn, to_print, strlen(to_print), NETCONN_COPY);
        netconn_write(conn, "\r\n", 2, NETCONN_COPY);
    } else {
        const char *error = "ERROR : cannot print that\r\n";
        netconn_write(conn, error, strlen(error), NETCONN_COPY);
    }
    return 0;
}

void lua_do_rom_script(char *buffer, int size)
{
    lua_State *l;
    unsigned char *command;
    int i;

    command = malloc(size + 1);

    l = luaL_newstate();
    luaL_openlibs(l);
    commands_register(l);

    for (i=0;i<size;i++) {
        command[i] = buffer[i];
    }
    command[i] = 0;

    luaL_dostring(l, command);

    free(command);
}

void lua_do_settings(void)
{
    extern unsigned char settings_lua[];
    extern long int settings_lua_size;

    lua_do_rom_script(settings_lua, settings_lua_size);
}

void serve_conn(struct netconn *conn)
{

    struct netbuf *buf;
    void *data;
    u16_t len;

    err_t err;
	lua_State *l;
    char command[MAX_COMMAND_LEN], real_command[MAX_COMMAND_LEN];
    int ret;

    l = luaL_newstate();
    luaL_openlibs(l);

    lua_pushlightuserdata(l, conn);
    lua_setglobal(l, "__conn");

    lua_pushcfunction(l, print_func);
    lua_setglobal(l, "print");

    commands_register(l);

    netconn_write(conn, PROMPT, strlen(PROMPT), NETCONN_COPY);
    while((err = netconn_recv(conn, &buf)) == ERR_OK) {
        do {
            /* Copies the buffer data into a string. */
            netbuf_data(buf, &data, &len);
            strncpy(command, data, len); // XXX buffer overflow
            command[len] = 0;

            sprintf(real_command, "print(%s)", command); // XXX buffer overflow

            luaL_loadstring(l, real_command);
            ret = lua_pcall(l, 0, LUA_MULTRET, 0);
            if (ret) {
                lua_pop(l, 1);
                luaL_loadstring(l, command);
                ret = lua_pcall(l, 0, LUA_MULTRET, 0);
                if (ret) {
                    snprintf(command, MAX_COMMAND_LEN, "ERROR %d : %s\n", ret, lua_tostring(l, -1));
                    netconn_write(conn, command, strlen(command), NETCONN_COPY);

                    lua_pop(l, 1);
                }
            }
            netconn_write(conn, PROMPT, strlen(PROMPT), NETCONN_COPY);
        } while (netbuf_next(buf) >= 0);
        netbuf_delete(buf);
    }

    lua_close(l);
}

static void luaconsole_thread(void *arg)
{
    struct netconn *conn, *newconn;
    err_t err;
    LWIP_UNUSED_ARG(arg);


    /* Create a new connection identifier. */
    conn = netconn_new(NETCONN_TCP);

    /* Bind connection to well known port number 7. */
    netconn_bind(conn, NULL, 1235);

    /* Tell connection to go into listening mode. */
    netconn_listen(conn);
        printf("started\n");

    while(1) {
        /* Grab new connection. */
        err = netconn_accept(conn, &newconn);
        printf("accepted\n");
        if(err == ERR_OK) {
            serve_conn(newconn);

            netconn_close(newconn);
            netconn_delete(newconn);
        }
    }
}

void luaconsole_init(void)
{
    sys_thread_new("luaconsole",luaconsole_thread , NULL, DEFAULT_THREAD_STACKSIZE, 32);
}
