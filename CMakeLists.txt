
# ZMK Trackball Angle Correction Module

# Original input processor
if(CONFIG_ZMK_INPUT_PROCESSOR_ANGLE)
    target_sources(app PRIVATE
        src/input_processor_angle.c
    )
endif()

# Virtual input device for full XY rotation
if(CONFIG_ZMK_INPUT_PROCESSOR_ANGLE)
    target_sources(app PRIVATE
        src/angle_input.c
    )
endif()
