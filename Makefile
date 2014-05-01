PREFIX = nios2-elf

CC = @${PREFIX}-gcc

# Yeah I know...
LD = @${PREFIX}-g++
CP = @${PREFIX}-objcopy
OD = @${PREFIX}-objdump



# Project name (W/O .c extension eg. "main")
PROJECT_NAME = debra

PROJECT_ROOT = .
ALT_LIBRARY_ROOT_DIR = ../project_files/Debra_bsp


INCLUDE_DIRS := $(PROJECT_ROOT)/modules/include/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/blocking_detection_manager/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/cirbuf/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/control_system_manager/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/cvra_adc/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/cvra_beacon/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/cvra_dc/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/cvra_logger/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/cvra_servo/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/dual_quadramp/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/error/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/math/fast_math/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/math/geometry/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/math/geometry/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/math/geometry/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/math/geometry/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/math/vect2/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/obstacle_avoidance/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/parse/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/parse/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/parse/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/pid/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/position_manager/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/quadramp/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/ramp/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/rdline/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/robot_system/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/robot_system/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/trajectory_manager/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/trajectory_manager/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/trajectory_manager/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/uptime/
INCLUDE_DIRS += $(PROJECT_ROOT)/modules/modules/platform/
INCLUDE_DIRS += $(PROJECT_ROOT)/lwip/src/include/
INCLUDE_DIRS += $(PROJECT_ROOT)/lwip/src/include/ipv4
INCLUDE_DIRS += $(PROJECT_ROOT)/lwip/src/include/ipv6
INCLUDE_DIRS += $(PROJECT_ROOT)/lwip/src/contrib/ports/ucos-ii/include/
INCLUDE_DIRS += $(PROJECT_ROOT)/lwip/src/netif/
INCLUDE_DIRS += $(PROJECT_ROOT)

SRC = $(wildcard $(PROJECT_ROOT)/*.c)
SRC += $(wildcard $(PROJECT_ROOT)/lua/*.c)

SRC += $(PROJECT_ROOT)/modules/modules/blocking_detection_manager/blocking_detection_manager.c
SRC += $(PROJECT_ROOT)/modules/modules/control_system_manager/control_system_manager.c
SRC += $(PROJECT_ROOT)/modules/modules/cvra_adc/cvra_adc.c
SRC += $(PROJECT_ROOT)/modules/modules/cvra_beacon/cvra_beacon.c
SRC += $(PROJECT_ROOT)/modules/modules/cvra_dc/cvra_dc.c
SRC += $(PROJECT_ROOT)/modules/modules/error/error.c
SRC += $(PROJECT_ROOT)/modules/modules/math/fast_math/fast_math.c
SRC += $(PROJECT_ROOT)/modules/modules/math/geometry/circles.c
SRC += $(PROJECT_ROOT)/modules/modules/math/geometry/lines.c
SRC += $(PROJECT_ROOT)/modules/modules/math/geometry/polygon.c
SRC += $(PROJECT_ROOT)/modules/modules/math/geometry/vect_base.c
SRC += $(PROJECT_ROOT)/modules/modules/math/vect2/vect2.c
SRC += $(PROJECT_ROOT)/modules/modules/obstacle_avoidance/obstacle_avoidance.c
SRC += $(PROJECT_ROOT)/modules/modules/pid/pid.c
SRC += $(PROJECT_ROOT)/modules/modules/platform/platform_nios2.c
SRC += $(PROJECT_ROOT)/modules/modules/position_manager/2wheels/position_manager.c
SRC += $(PROJECT_ROOT)/modules/modules/quadramp/quadramp.c
SRC += $(PROJECT_ROOT)/modules/modules/ramp/ramp.c
SRC += $(PROJECT_ROOT)/modules/modules/robot_system/2wheels/angle_distance.c
SRC += $(PROJECT_ROOT)/modules/modules/robot_system/2wheels/robot_system.c
SRC += $(PROJECT_ROOT)/modules/modules/trajectory_manager/2wheels/trajectory_manager.c
SRC += $(PROJECT_ROOT)/modules/modules/trajectory_manager/2wheels/trajectory_manager_core.c
SRC += $(PROJECT_ROOT)/modules/modules/trajectory_manager/2wheels/trajectory_manager_utils.c
SRC += $(PROJECT_ROOT)/modules/modules/uptime/uptime.c
SRC += $(PROJECT_ROOT)/lwip/src/api/api_lib.c
SRC += $(PROJECT_ROOT)/lwip/src/api/api_msg.c
SRC += $(PROJECT_ROOT)/lwip/src/api/err.c
SRC += $(PROJECT_ROOT)/lwip/src/api/netbuf.c
SRC += $(PROJECT_ROOT)/lwip/src/api/netdb.c
SRC += $(PROJECT_ROOT)/lwip/src/api/netifapi.c
SRC += $(PROJECT_ROOT)/lwip/src/api/sockets.c
SRC += $(PROJECT_ROOT)/lwip/src/api/tcpip.c
SRC += $(PROJECT_ROOT)/lwip/src/contrib/ports/ucos-ii/lib.c
SRC += $(PROJECT_ROOT)/lwip/src/contrib/ports/ucos-ii/sys_arch.c
SRC += $(PROJECT_ROOT)/lwip/src/core/def.c
SRC += $(PROJECT_ROOT)/lwip/src/core/dhcp.c
SRC += $(PROJECT_ROOT)/lwip/src/core/dns.c
SRC += $(PROJECT_ROOT)/lwip/src/core/inet_chksum.c
SRC += $(PROJECT_ROOT)/lwip/src/core/init.c
SRC += $(PROJECT_ROOT)/lwip/src/core/ipv4/autoip.c
SRC += $(PROJECT_ROOT)/lwip/src/core/ipv4/icmp.c
SRC += $(PROJECT_ROOT)/lwip/src/core/ipv4/igmp.c
SRC += $(PROJECT_ROOT)/lwip/src/core/ipv4/ip4.c
SRC += $(PROJECT_ROOT)/lwip/src/core/ipv4/ip4_addr.c
SRC += $(PROJECT_ROOT)/lwip/src/core/ipv4/ip_frag.c
SRC += $(PROJECT_ROOT)/lwip/src/core/ipv6/dhcp6.c
SRC += $(PROJECT_ROOT)/lwip/src/core/ipv6/ethip6.c
SRC += $(PROJECT_ROOT)/lwip/src/core/ipv6/icmp6.c
SRC += $(PROJECT_ROOT)/lwip/src/core/ipv6/inet6.c
SRC += $(PROJECT_ROOT)/lwip/src/core/ipv6/ip6.c
SRC += $(PROJECT_ROOT)/lwip/src/core/ipv6/ip6_addr.c
SRC += $(PROJECT_ROOT)/lwip/src/core/ipv6/ip6_frag.c
SRC += $(PROJECT_ROOT)/lwip/src/core/ipv6/mld6.c
SRC += $(PROJECT_ROOT)/lwip/src/core/ipv6/nd6.c
SRC += $(PROJECT_ROOT)/lwip/src/core/mem.c
SRC += $(PROJECT_ROOT)/lwip/src/core/memp.c
SRC += $(PROJECT_ROOT)/lwip/src/core/netif.c
SRC += $(PROJECT_ROOT)/lwip/src/core/pbuf.c
SRC += $(PROJECT_ROOT)/lwip/src/core/raw.c
SRC += $(PROJECT_ROOT)/lwip/src/core/snmp/asn1_dec.c
SRC += $(PROJECT_ROOT)/lwip/src/core/snmp/asn1_enc.c
SRC += $(PROJECT_ROOT)/lwip/src/core/snmp/mib2.c
SRC += $(PROJECT_ROOT)/lwip/src/core/snmp/mib_structs.c
SRC += $(PROJECT_ROOT)/lwip/src/core/snmp/msg_in.c
SRC += $(PROJECT_ROOT)/lwip/src/core/snmp/msg_out.c
SRC += $(PROJECT_ROOT)/lwip/src/core/stats.c
SRC += $(PROJECT_ROOT)/lwip/src/core/sys.c
SRC += $(PROJECT_ROOT)/lwip/src/core/tcp.c
SRC += $(PROJECT_ROOT)/lwip/src/core/tcp_in.c
SRC += $(PROJECT_ROOT)/lwip/src/core/tcp_out.c
SRC += $(PROJECT_ROOT)/lwip/src/core/timers.c
SRC += $(PROJECT_ROOT)/lwip/src/core/udp.c
SRC += $(PROJECT_ROOT)/lwip/src/netif/etharp.c
SRC += $(PROJECT_ROOT)/lwip/src/netif/ethernetif.c
SRC += $(PROJECT_ROOT)/lwip/src/netif/slipif.c

include $(ALT_LIBRARY_ROOT_DIR)/public.mk

INCLUDE_DIRS := $(addprefix -I, $(ALT_INCLUDE_DIRS) $(INCLUDE_DIRS)) 
CFLAGS = -c -g $(INCLUDE_DIRS) -DCOMPILE_ON_ROBOT $(ALT_CFLAGS) $(ALT_CPPFLAGS) -Wall
CFLAGS += -MD

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
	@rm -rf $(OBJS) $(ELF) $(OBJS:.o=.d)

-include $(OBJS:.o=.d)

