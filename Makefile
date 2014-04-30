PREFIX = nios2-elf

CC = @${PREFIX}-gcc

# Yeah I know...
LD = @${PREFIX}-g++
CP = @${PREFIX}-objcopy
OD = @${PREFIX}-objdump



# Project name (W/O .c extension eg. "main")
PROJECT_NAME = debra

PROJECT_ROOT = ../../debra
ALT_LIBRARY_ROOT_DIR = ../Debra_bsp

include $(ALT_LIBRARY_ROOT_DIR)/public.mk

INCLUDE_DIRS := ../../debra/modules/include/
INCLUDE_DIRS += ../../debra/modules/modules/blocking_detection_manager/
INCLUDE_DIRS += ../../debra/modules/modules/cirbuf/
INCLUDE_DIRS += ../../debra/modules/modules/commandline/
INCLUDE_DIRS += ../../debra/modules/modules/control_system_manager/
INCLUDE_DIRS += ../../debra/modules/modules/cvra_adc/
INCLUDE_DIRS += ../../debra/modules/modules/cvra_beacon/
INCLUDE_DIRS += ../../debra/modules/modules/cvra_dc/
INCLUDE_DIRS += ../../debra/modules/modules/cvra_logger/
INCLUDE_DIRS += ../../debra/modules/modules/cvra_servo/
INCLUDE_DIRS += ../../debra/modules/modules/dual_quadramp/
INCLUDE_DIRS += ../../debra/modules/modules/error/
INCLUDE_DIRS += ../../debra/modules/modules/math/fast_math/
INCLUDE_DIRS += ../../debra/modules/modules/math/geometry/
INCLUDE_DIRS += ../../debra/modules/modules/math/geometry/
INCLUDE_DIRS += ../../debra/modules/modules/math/geometry/
INCLUDE_DIRS += ../../debra/modules/modules/math/geometry/
INCLUDE_DIRS += ../../debra/modules/modules/math/vect2/
INCLUDE_DIRS += ../../debra/modules/modules/obstacle_avoidance/
INCLUDE_DIRS += ../../debra/modules/modules/parse/
INCLUDE_DIRS += ../../debra/modules/modules/parse/
INCLUDE_DIRS += ../../debra/modules/modules/parse/
INCLUDE_DIRS += ../../debra/modules/modules/pid/
INCLUDE_DIRS += ../../debra/modules/modules/position_manager/
INCLUDE_DIRS += ../../debra/modules/modules/quadramp/
INCLUDE_DIRS += ../../debra/modules/modules/ramp/
INCLUDE_DIRS += ../../debra/modules/modules/rdline/
INCLUDE_DIRS += ../../debra/modules/modules/robot_system/
INCLUDE_DIRS += ../../debra/modules/modules/robot_system/
INCLUDE_DIRS += ../../debra/modules/modules/trajectory_manager/
INCLUDE_DIRS += ../../debra/modules/modules/trajectory_manager/
INCLUDE_DIRS += ../../debra/modules/modules/trajectory_manager/
INCLUDE_DIRS += ../../debra/modules/modules/uptime/
INCLUDE_DIRS += ../../debra/modules/modules/vt100/
INCLUDE_DIRS += ../../debra/modules/modules/platform/
INCLUDE_DIRS += ../../debra/lwip/src/include/
INCLUDE_DIRS += ../../debra/lwip/src/include/ipv4
INCLUDE_DIRS += ../../debra/lwip/src/include/ipv6
INCLUDE_DIRS += ../../debra/lwip/src/contrib/ports/ucos-ii/include/
INCLUDE_DIRS += ../../debra/lwip/src/netif/
INCLUDE_DIRS += ../../debra

SRC = $(wildcard $(PROJECT_ROOT)/*.c)
SRC += $(wildcard $(PROJECT_ROOT)/lua/*.c)

SRC += ../../debra/modules/modules/blocking_detection_manager/blocking_detection_manager.c
SRC += ../../debra/modules/modules/control_system_manager/control_system_manager.c
SRC += ../../debra/modules/modules/cvra_adc/cvra_adc.c
SRC += ../../debra/modules/modules/cvra_beacon/cvra_beacon.c
SRC += ../../debra/modules/modules/cvra_dc/cvra_dc.c
SRC += ../../debra/modules/modules/error/error.c
SRC += ../../debra/modules/modules/math/fast_math/fast_math.c
SRC += ../../debra/modules/modules/math/geometry/circles.c
SRC += ../../debra/modules/modules/math/geometry/lines.c
SRC += ../../debra/modules/modules/math/geometry/polygon.c
SRC += ../../debra/modules/modules/math/geometry/vect_base.c
SRC += ../../debra/modules/modules/math/vect2/vect2.c
SRC += ../../debra/modules/modules/obstacle_avoidance/obstacle_avoidance.c
SRC += ../../debra/modules/modules/pid/pid.c
SRC += ../../debra/modules/modules/platform/platform_nios2.c
SRC += ../../debra/modules/modules/position_manager/2wheels/position_manager.c
SRC += ../../debra/modules/modules/quadramp/quadramp.c
SRC += ../../debra/modules/modules/ramp/ramp.c
SRC += ../../debra/modules/modules/robot_system/2wheels/angle_distance.c
SRC += ../../debra/modules/modules/robot_system/2wheels/robot_system.c
SRC += ../../debra/modules/modules/trajectory_manager/2wheels/trajectory_manager.c
SRC += ../../debra/modules/modules/trajectory_manager/2wheels/trajectory_manager_core.c
SRC += ../../debra/modules/modules/trajectory_manager/2wheels/trajectory_manager_utils.c
SRC += ../../debra/modules/modules/uptime/uptime.c
SRC += ../../debra/modules/modules/vt100/vt100.c
SRC += ../../debra/lwip/src/api/api_lib.c
SRC += ../../debra/lwip/src/api/api_msg.c
SRC += ../../debra/lwip/src/api/err.c
SRC += ../../debra/lwip/src/api/netbuf.c
SRC += ../../debra/lwip/src/api/netdb.c
SRC += ../../debra/lwip/src/api/netifapi.c
SRC += ../../debra/lwip/src/api/sockets.c
SRC += ../../debra/lwip/src/api/tcpip.c
SRC += ../../debra/lwip/src/contrib/ports/ucos-ii/lib.c
SRC += ../../debra/lwip/src/contrib/ports/ucos-ii/sys_arch.c
SRC += ../../debra/lwip/src/core/def.c
SRC += ../../debra/lwip/src/core/dhcp.c
SRC += ../../debra/lwip/src/core/dns.c
SRC += ../../debra/lwip/src/core/inet_chksum.c
SRC += ../../debra/lwip/src/core/init.c
SRC += ../../debra/lwip/src/core/ipv4/autoip.c
SRC += ../../debra/lwip/src/core/ipv4/icmp.c
SRC += ../../debra/lwip/src/core/ipv4/igmp.c
SRC += ../../debra/lwip/src/core/ipv4/ip4.c
SRC += ../../debra/lwip/src/core/ipv4/ip4_addr.c
SRC += ../../debra/lwip/src/core/ipv4/ip_frag.c
SRC += ../../debra/lwip/src/core/ipv6/dhcp6.c
SRC += ../../debra/lwip/src/core/ipv6/ethip6.c
SRC += ../../debra/lwip/src/core/ipv6/icmp6.c
SRC += ../../debra/lwip/src/core/ipv6/inet6.c
SRC += ../../debra/lwip/src/core/ipv6/ip6.c
SRC += ../../debra/lwip/src/core/ipv6/ip6_addr.c
SRC += ../../debra/lwip/src/core/ipv6/ip6_frag.c
SRC += ../../debra/lwip/src/core/ipv6/mld6.c
SRC += ../../debra/lwip/src/core/ipv6/nd6.c
SRC += ../../debra/lwip/src/core/mem.c
SRC += ../../debra/lwip/src/core/memp.c
SRC += ../../debra/lwip/src/core/netif.c
SRC += ../../debra/lwip/src/core/pbuf.c
SRC += ../../debra/lwip/src/core/raw.c
SRC += ../../debra/lwip/src/core/snmp/asn1_dec.c
SRC += ../../debra/lwip/src/core/snmp/asn1_enc.c
SRC += ../../debra/lwip/src/core/snmp/mib2.c
SRC += ../../debra/lwip/src/core/snmp/mib_structs.c
SRC += ../../debra/lwip/src/core/snmp/msg_in.c
SRC += ../../debra/lwip/src/core/snmp/msg_out.c
SRC += ../../debra/lwip/src/core/stats.c
SRC += ../../debra/lwip/src/core/sys.c
SRC += ../../debra/lwip/src/core/tcp.c
SRC += ../../debra/lwip/src/core/tcp_in.c
SRC += ../../debra/lwip/src/core/tcp_out.c
SRC += ../../debra/lwip/src/core/timers.c
SRC += ../../debra/lwip/src/core/udp.c
SRC += ../../debra/lwip/src/netif/etharp.c
SRC += ../../debra/lwip/src/netif/ethernetif.c
SRC += ../../debra/lwip/src/netif/slipif.c


INCLUDE_DIRS := $(addprefix -I, $(ALT_INCLUDE_DIRS) $(INCLUDE_DIRS)) 
CFLAGS = -c -g $(INCLUDE_DIRS) -DCOMPILE_ON_ROBOT $(ALT_CFLAGS) $(ALT_CPPFLAGS) -Wall

APP_LIB_DIRS := $(addprefix -L, $(ALT_LIBRARY_DIRS))

LFLAGS  = --gc-sections

ELF = $(PROJECT_NAME).elf



OBJS = $(SRC:.c=.o)

#==============================================================================
#                      Rules to make the target
#==============================================================================

#make all rule
all: $(ELF)

$(ELF): $(OBJS)
	@echo "Linking..."
	$(LD) -o test.elf $(OBJS) -lm $(APP_LIB_DIRS) -lucosii_bsp -T $(BSP_LINKER_SCRIPT) -msys-crt0='$(BSP_CRT0)'
	@echo "Patching elf..."
	@nios2-elf-insert test.elf $(ELF_PATCH_FLAG)

# Rule to load the project to the board
load: $(ELF)
	@nios2-download --go --cpu_name=$(CPU_NAME) $(SOPC_SYSID_FLAG) test.elf

%.o: %.c
	@echo
	@echo Compiling $<...
	$(CC) -c $(CFLAGS) ${<} -o ${@}

# make clean rule
clean:
	rm $(OBJS) $(ELF)

