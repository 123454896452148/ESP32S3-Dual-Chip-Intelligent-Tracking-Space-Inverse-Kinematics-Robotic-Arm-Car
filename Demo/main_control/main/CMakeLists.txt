idf_component_register(
    SRCS
        main.cc
        car_control.cc
        track_sensor.cc
        ultrasonic.cc
        uart_comm.cc
        pid_controller.cc

    INCLUDE_DIRS "."
)