
if(CONFIG_ZMK_INPUT_PROCESSOR_ANGLE)
    target_sources(app PRIVATE
        src/input_processor_angle.c
    )
endif()
