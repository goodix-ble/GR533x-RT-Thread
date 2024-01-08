import os

# toolchains options
ARCH='arm'
CPU='cortex-m4'
CROSS_TOOL='keil'
DEBUG = False

# if os.getenv('RTT_CC'):
#     CROSS_TOOL = os.getenv('RTT_CC')

# cross_tool provides the cross compiler
# EXEC_PATH is the compiler execute path, for example, CodeSourcery, Keil MDK, IAR
print('CROSS_TOOL:' + CROSS_TOOL )

if CROSS_TOOL == 'keil':
    PLATFORM    = 'armcc'
    EXEC_PATH   = 'C:/Keil_v5'
elif CROSS_TOOL == 'iar':
    print('================ERROR============================')
    print('Not support iar yet!')
    print('=================================================')
    exit(0)
elif CROSS_TOOL == 'gcc':
    print('================ERROR============================')
    print('Not support gcc yet!')
    print('=================================================')
    exit(0)


if os.path.exists(EXEC_PATH):
    pass
elif os.getenv('RTT_EXEC_PATH'):
    EXEC_PATH = os.getenv('RTT_EXEC_PATH')

if PLATFORM == 'armcc':
    # toolchains
    CC = 'armcc'
    AS = 'armasm'
    AR = 'armar'
    LINK = 'armlink'
    TARGET_EXT = 'axf'

    DEVICE = ' --cpu Cortex-M4.fp.sp'
    CFLAGS = '-c ' + DEVICE + ' --apcs=interwork --split_sections'
    CFLAGS += ' --c99'
    AFLAGS = DEVICE + ' --apcs=interwork '

    LFLAGS = DEVICE + ' --scatter "../../../sdk/GR533x/platform/soc/linker/keil/flash_scatter_common.sct" --info sizes --info totals --info unused --info veneers --list build/rtthread.map --strict'
    CFLAGS += ' -I' + EXEC_PATH + '/ARM/ARMCC/include'
    LFLAGS += ' --libpath=' + EXEC_PATH + '/ARM/ARMCC/lib'
    LFLAGS += ' ../../../sdk/GR533x/platform/soc/linker/keil/ble_sdk.lib '
    LFLAGS += ' ../../../sdk/GR533x/platform/soc/linker/keil/rom_symbol.txt '

    CFLAGS += ' -DENV_USE_RT_THREAD '
    AFLAGS += ' --pd "ENV_USE_RT_THREAD SETA 1" '
    
    # Enable Microlib
    # CFLAGS += ' -D__MICROLIB '
    # AFLAGS += ' --pd "__MICROLIB SETA 1" '
    LFLAGS += ' --library_type=standardlib '
    EXEC_PATH += '/ARM/ARMCC/bin/'

    if DEBUG == True:
        CFLAGS += ' -g -O0'
        AFLAGS += ' -g'
    else:
        CFLAGS += ' -O1'

    POST_ACTION = 'fromelf --bin $TARGET --output build/rtthread.bin \nfromelf.exe --text -c --output build/rtthread.s $TARGET \nfromelf -z $TARGET '

elif PLATFORM == 'gcc':
    print('================ERROR============================')
    print('Not support gcc yet!')
    print('=================================================')
    exit(0)

def dist_handle(BSP_ROOT, dist_dir):
    import sys
    cwd_path = os.getcwd()
    sys.path.append(os.path.join(os.path.dirname(BSP_ROOT), 'tools'))
    from sdk_dist import dist_do_building
    dist_do_building(BSP_ROOT, dist_dir)
