# select a ros1 workspace

# --- Only alter this value ---|
#
DESIRED_NUMBER=4             # |
#              ^^
# -----------------------------|

rws_list=(
    # $HOME/catkin_ws/src                   # X do not use anymore, is under root!, mobipick labs workspace (built in with docker image)
    $HOME/ros_ws/clean_mobipick_labs_ws/src # 0 non-modified mobipick labs clean ws
    $HOME/ros_ws/mobipick_labs_ws/src       # 1, mobipick labs
    $HOME/ros_ws/rae_upom_mobipick_ws/src   # *2, RAE + UPOM + Mobipick (extends #1)
    $HOME/ros_ws/common_tools_ws/src        # *3, adds TTS and other things (extends #0)
    $HOME/ros_ws/gpt_ws/src                 # *4, adds GPT capabilities (extends #3)
)

# set this env, only if it does not previouly exist
export ROS_WORKSPACE=${ROS_WORKSPACE:=${rws_list[$DESIRED_NUMBER]}}
