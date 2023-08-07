#!/usr/bin/env sh

# Check if MUJOCO_BUILD_DIR exists
if [ -z "$MUJOCO_BUILD_DIR" ]; then
    echo "MUJOCO_BUILD_DIR is unset." >&2
    exit 1
else
    echo "MUJOCO_BUILD_DIR is set to: $MUJOCO_BUILD_DIR"
    
    SETUP_PATH="$CATKIN_DEVEL_DIR/setup.sh"
    
    if ! echo "$PATH" | grep -q "$MUJOCO_BUILD_DIR/bin"; then
        export PATH=$PATH:$MUJOCO_BUILD_DIR/bin
    fi
    
    PATH_TO_ADD="if ! echo \"\$PATH\" | grep -q \"$MUJOCO_BUILD_DIR/bin\"; then\n  export PATH=\$PATH:$MUJOCO_BUILD_DIR/bin\nfi"
    
    # Check if the line already exists in the file
    if ! grep -Fxq "$PATH_TO_ADD" "$SETUP_PATH"; then
        # Add the line to the file
        echo "$PATH_TO_ADD" >> $SETUP_PATH
    fi
fi