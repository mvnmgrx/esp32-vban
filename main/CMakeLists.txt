idf_component_register(SRCS "main.c" "vban/vban_frame.c" "vban/vban_ringbuf.c" "vban/stream.c" "vban/stream_ctrl.c" "interfaces/ethernet.c"
                       INCLUDE_DIRS "." "./vban" "./interfaces")
