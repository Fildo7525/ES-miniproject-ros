set(SRC
	xnn_inference.h
	xnn_inference_hw.h
	xnn_inference.c
	xnn_inference_linux.c
)

add_library(drivers ${SRC})

