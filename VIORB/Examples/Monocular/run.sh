cd ../../build
make -j4
cd ../Examples/Monocular
#./euro_imu ../../Vocabulary/ORBvoc.bin ../../config/euroc.yaml /home/aayush/workspace/sandbox/mav0/cam0/data ../../../../mav0/cam0/data.csv ../../../../mav0/imu0/data.csv
./euro_imu ../../Vocabulary/ORBvoc.bin ../../config/euroc.yaml ../../datasets/V1_01_easy/mav0/cam0/data ../../datasets/V1_01_easy/mav0/cam0/data.csv ../../datasets/V1_01_easy/mav0/imu0/data.csv 50
