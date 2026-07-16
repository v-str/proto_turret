#!/bin/bash
set -e

export BASE_PATH=/project/$MICROROS_LIBRARY_FOLDER

cd /uros_ws

source /opt/ros/$ROS_DISTRO/setup.bash
source install/local_setup.bash

if [ ! -d "firmware/mcu_ws/ros2" ]; then
    echo ""
    echo "╔══════════════════════════════════════════════════════════╗"
    echo "║         Первый запуск — скачивание зависимостей         ║"
    echo "╚══════════════════════════════════════════════════════════╝"
    echo ""

    apt install -y gcc-arm-none-eabi

    ros2 run micro_ros_setup create_firmware_ws.sh generate_lib

    ######## Adding extra packages ########
    pushd firmware/mcu_ws > /dev/null

        # Workaround: Copy just tf2_msgs
        git clone -b kilted https://github.com/ros2/geometry2
        cp -R geometry2/tf2_msgs ros2/tf2_msgs
        rm -rf geometry2

        # Import user defined packages
        mkdir extra_packages
        pushd extra_packages > /dev/null
            USER_CUSTOM_PACKAGES_DIR=$BASE_PATH/../../microros_component/extra_packages
            if [ -d "$USER_CUSTOM_PACKAGES_DIR" ]; then
                cp -R $USER_CUSTOM_PACKAGES_DIR/* .
            fi
            if [ -f $USER_CUSTOM_PACKAGES_DIR/extra_packages.repos ]; then
                vcs import --input $USER_CUSTOM_PACKAGES_DIR/extra_packages.repos
            fi
            cp -R $BASE_PATH/library_generation/extra_packages/* .
            vcs import --input extra_packages.repos
        popd > /dev/null

    popd > /dev/null
else
    echo "=== Зависимости уже скачаны, пропускаем ==="
fi

######## Patch UXR config ########
# Increase session connection interval to 60s so it doesn't re-establish
# during the micro-ROS initialization sequence (rclc_support_init, etc.)
# The default 1s interval causes constant session re-establishment which
# breaks entity registration — the client returns OBJK_INVALID on GET_INFO.
UCFG_IN="firmware/mcu_ws/eProsima/Micro-XRCE-DDS-Client/include/uxr/client/config.h.in"
if [ -f "$UCFG_IN" ]; then
    echo "Patching $UCFG_IN ..."
    sed -i 's/@UCLIENT_MIN_SESSION_CONNECTION_INTERVAL@/60000/' "$UCFG_IN"
fi
UCFG="firmware/mcu_ws/eProsima/Micro-XRCE-DDS-Client/include/uxr/client/config.h"
if [ -f "$UCFG" ]; then
    echo "Patching $UCFG ..."
    sed -i 's/#define UXR_CONFIG_MIN_SESSION_CONNECTION_INTERVAL[[:space:]]*1000/#define UXR_CONFIG_MIN_SESSION_CONNECTION_INTERVAL    60000/' "$UCFG"
fi

######## Trying to retrieve CFLAGS ########
pushd /project > /dev/null
export RET_CFLAGS=$(make print_cflags)
RET_CODE=$?

if [ $RET_CODE = "0" ]; then
    echo "Found CFLAGS:"
    echo "-------------"
    echo $RET_CFLAGS
    echo "-------------"
    read -p "Do you want to continue with them? (y/n)" -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]
    then
        echo "Continuing..."
    else
        echo "Aborting"
        exit 0;
    fi
else
    echo "Please read README.md to update your Makefile"
    exit 1;
fi
popd > /dev/null

######## Build  ########
export TOOLCHAIN_PREFIX=/usr/bin/arm-none-eabi-

echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║              ФЛАГИ СБОРКИ (toolchain.cmake)              ║"
echo "╠══════════════════════════════════════════════════════════╣"
echo "║  RET_CFLAGS = ${RET_CFLAGS:-<не заданы>}"
echo "║  TOOLCHAIN  = $TOOLCHAIN_PREFIX"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

ros2 run micro_ros_setup build_firmware.sh $BASE_PATH/library_generation/toolchain.cmake $BASE_PATH/library_generation/colcon.meta

find firmware/build/include/ -name "*.c"  -delete
rm -rf $BASE_PATH/libmicroros
mkdir -p $BASE_PATH/libmicroros/microros_include
cp -R firmware/build/include/* $BASE_PATH/libmicroros/microros_include/
cp -R firmware/build/libmicroros.a $BASE_PATH/libmicroros/libmicroros.a

######## Fix include paths  ########
pushd firmware/mcu_ws > /dev/null
    INCLUDE_ROS2_PACKAGES=$(colcon list | awk '{print $1}' | awk -v d=" " '{s=(NR==1?s:s d)$0}END{print s}')
popd > /dev/null

for var in ${INCLUDE_ROS2_PACKAGES}; do
    if [ -d "$BASE_PATH/libmicroros/microros_include/${var}/${var}" ]; then
        rsync -r $BASE_PATH/libmicroros/microros_include/${var}/${var}/* $BASE_PATH/libmicroros/microros_include/${var}
        rm -rf $BASE_PATH/libmicroros/microros_include/${var}/${var}
    fi
done

######## Generate extra files ########
find firmware/mcu_ws/ros2 \( -name "*.srv" -o -name "*.msg" -o -name "*.action" \) | awk -F"/" '{print $(NF-2)"/"$NF}' > $BASE_PATH/libmicroros/available_ros2_types
find firmware/mcu_ws/extra_packages \( -name "*.srv" -o -name "*.msg" -o -name "*.action" \) | awk -F"/" '{print $(NF-2)"/"$NF}' >> $BASE_PATH/libmicroros/available_ros2_types

cd firmware
echo "" > $BASE_PATH/libmicroros/built_packages
for f in $(find $(pwd) -name .git -type d); do pushd $f > /dev/null; echo $(git config --get remote.origin.url) $(git rev-parse HEAD) >> $BASE_PATH/libmicroros/built_packages; popd > /dev/null; done;

######## Fix permissions ########
sudo chmod -R 777 $BASE_PATH/libmicroros/
sudo chmod -R 777 $BASE_PATH/libmicroros/microros_include/
sudo chmod -R 777 $BASE_PATH/libmicroros/libmicroros.a
