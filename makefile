all: main    source  dxlhandler
	g++ lib/main.o lib/source.o  lib/DynamixelHandler.o  -o bin/robotpuppetry -L/usr/local/lib -L/usr/lib/x86_64-linux-gnu -lopencv_stitching -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_dnn_objdetect -lopencv_dnn_superres -lopencv_dpm -lopencv_highgui -lopencv_face -lopencv_freetype -lopencv_fuzzy -lopencv_hdf -lopencv_hfs -lopencv_img_hash -lopencv_line_descriptor -lopencv_quality -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_shape -lopencv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_superres -lopencv_optflow -lopencv_surface_matching -lopencv_tracking -lopencv_datasets -lopencv_text -lopencv_dnn -lopencv_plot -lopencv_ml -lopencv_videostab -lopencv_videoio -lopencv_viz -lopencv_ximgproc -lopencv_video -lopencv_xobjdetect -lopencv_objdetect -lopencv_calib3d -lopencv_imgcodecs -lopencv_features2d -lopencv_flann -lopencv_xphoto -lopencv_photo -lopencv_imgproc -lopencv_core -lpthread -lX11 -ldxl_x64_cpp -lrt
	
source: src/source.cpp
	g++  -c src/source.cpp -O3  -msse4 -o lib/source.o -I/usr/include/dlib-19.23 
	
main: src/main.cpp
	g++  -c src/main.cpp -O3 -msse4 -o lib/main.o -I./include -I/usr/include/opencv4 -I/usr/include/dlib-19.23
	
dxlhandler: src/DynamixelHandler.cpp
	g++ -c src/DynamixelHandler.cpp -O3 -msse4 -o lib/DynamixelHandler.o -I./include
	
clean:
	rm lib/*.o
	rm bin/*