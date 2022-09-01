#!/bin/sh

/system/bin/opencv_bmcv conv dog.jpg
cmp dog.conv.png conv_1_0.png || echo -e "\033[31m BMCV ERR \033[0m"

/system/bin/opencv_bmcv size dog.jpg
cmp dog.size.png size_1_0.png || echo -e "\033[31m BMCV ERR \033[0m"

/system/bin/opencv_bmcv image dog.jpg
cmp dog.image.jpg image_0.jpg || echo -e "\033[31m BMCV ERR \033[0m"
cmp dog.image.png image_0.png || echo -e "\033[31m BMCV ERR \033[0m"
