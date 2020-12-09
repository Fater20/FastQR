#!/bin/bash


# This file is used to determine the addresses for all the buffers for the demo

# Begining of RAM
BASE_ADDR=`echo $((0x80000000))`


##################################################################
echo ""
echo "/* Video Frame Buffers */"
##################################################################
END_ADDR=`echo $(( $BASE_ADDR + $((800*480)) - 1 ))`
printf       '#define CAMERA_FB_ADDR 0x%X           /* 0x%X  ---------------*/\n' $BASE_ADDR $BASE_ADDR
printf '					    /*             | CLUT8        */\n'
printf '#define CAMERA_FB_SIZE (800*480)            /* 0x%X  ---------------*/\n' $END_ADDR
BASE_ADDR=`echo $(( $END_ADDR + 1 ))`

END_ADDR=`echo $(( $BASE_ADDR + $((800*480/2)) - 1 ))`
printf       '#define OVERLAY_FB_ADDR 0x%X          /* 0x%X  ---------------*/\n' $BASE_ADDR $BASE_ADDR
printf '					    /*             | CLUT4        */\n'
printf '#define OVERLAY_FB_SIZE (800*480/2)         /* 0x%X  ---------------*/\n' $END_ADDR
BASE_ADDR=`echo $(( $END_ADDR + 1 ))`


## Round up to 4K boundary
#BASE_ADDR=`echo $(( $BASE_ADDR + 0x600 ))`

##################################################################
echo ""
echo "/* Input image to DRP */"
##################################################################
END_ADDR=`echo $(( $BASE_ADDR + $((800*480)) - 1 ))`
printf       '#define DRP_IN_RAM 0x%X               /* 0x%X  ---------------*/\n' $BASE_ADDR $BASE_ADDR
printf '					    /*             | Bayer        */\n'
printf '#define DRP_IN_RAM_SIZE (800*480)           /* 0x%X  ---------------*/\n' $END_ADDR
BASE_ADDR=`echo $(( $END_ADDR + 1 ))`

## Round up to 4K boundary
#BASE_ADDR=`echo $(( $BASE_ADDR + 0x400 ))`


##################################################################
echo ""
echo "/* DRP work Buffers */"
##################################################################

END_ADDR=`echo $(( $BASE_ADDR + $((800*480)) - 1 ))`
printf       '#define DRP_OUT_RAM 0x%X              /* 0x%X  ---------------*/\n' $BASE_ADDR $BASE_ADDR
printf '					    /*             | DRP data     */\n'
printf '#define DRP_OUT_RAM_SIZE (800*480)          /* 0x%X  ---------------*/\n' $END_ADDR
BASE_ADDR=`echo $(( $END_ADDR + 1 ))`

END_ADDR=`echo $(( $BASE_ADDR + $((800*480)) - 1 ))`
printf       '#define DRP_WORK_RAM 0x%X             /* 0x%X  ---------------*/\n' $BASE_ADDR $BASE_ADDR
printf '					    /*             | DRP data     */\n'
printf '#define DRP_WORK_RAM_SIZE (800*480)         /* 0x%X  ---------------*/\n' $END_ADDR
BASE_ADDR=`echo $(( $END_ADDR + 1 ))`

END_ADDR=`echo $(( $BASE_ADDR + $((800*480)) - 1 ))`
printf       '#define CANNY_HYST_RAM 0x%X           /* 0x%X  ---------------*/\n' $BASE_ADDR $BASE_ADDR
printf '					    /*             | DRP data     */\n'
printf '#define CANNY_HYST_RAM_SIZE (800*480)       /* 0x%X  ---------------*/\n' $END_ADDR
BASE_ADDR=`echo $(( $END_ADDR + 1 ))`

END_ADDR=`echo $(( $BASE_ADDR + $((800*(480+3)*2)) - 1 ))`
printf       '#define CANNY_WORK_RAM 0x%X           /* 0x%X  ---------------*/\n' $BASE_ADDR $BASE_ADDR
printf '					    /*             | DRP data     */\n'
printf '#define CANNY_WORK_RAM_SIZE (800*(480+3)*2) /* 0x%X  ---------------*/\n' $END_ADDR
BASE_ADDR=`echo $(( $END_ADDR + 1 ))`


##################################################################
echo ""
echo "/* Camera Capture Buffers */"
##################################################################

#MIPI must be on 128 byte address boundary
BASE_ADDR=`echo $(( $BASE_ADDR + 0x40 ))`

END_ADDR=`echo $(( $BASE_ADDR + $((800*480)) - 1 ))`
printf       '#define CEU_BUFF_ADDR 0x%X            /* 0x%X  ---------------*/\n' $BASE_ADDR $BASE_ADDR
printf '					    /*             | YCbCr422     */\n'
printf '#define CEU_BUFF_SIZE (800*480*2)           /* 0x%X  ---------------*/\n' $END_ADDR
#BASE_ADDR=`echo $(( $END_ADDR + 1 ))`
#CEU and MIPI will use the same buffer area

END_ADDR=`echo $(( $BASE_ADDR + $((800*480)) - 1 ))`
printf       '#define MIPI_BUFF_A_ADDR 0x%X         /* 0x%X  ---------------*/\n' $BASE_ADDR $BASE_ADDR
printf '					    /*             | RAW8         */\n'
printf '#define MIPI_BUFF_A_SIZE (800*480)          /* 0x%X  ---------------*/\n' $END_ADDR
BASE_ADDR=`echo $(( $END_ADDR + 1 ))`

END_ADDR=`echo $(( $BASE_ADDR + $((800*480)) - 1 ))`
printf       '#define MIPI_BUFF_B_ADDR 0x%X         /* 0x%X  ---------------*/\n' $BASE_ADDR $BASE_ADDR
printf '					    /*             | RAW8         */\n'
printf '#define MIPI_BUFF_A_SIZE (800*480)          /* 0x%X  ---------------*/\n' $END_ADDR
BASE_ADDR=`echo $(( $END_ADDR + 1 ))`




