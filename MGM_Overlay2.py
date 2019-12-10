#Displays the disparity of zedcamera images with search range of 90 

from pynq import Overlay
from pynq import Xlnk
import numpy as np
import cv2
import struct
import sys
import time
import struct
import cffi

IMG_WIDTH         =640
IMG_HEIGHT        =480
FILTER_SIZE     =9
FILTER_OFFS     =(int)(FILTER_SIZE/2)
SECTIONS        =9
SECTION_HEIGHT  =(int)((IMG_HEIGHT-2*FILTER_OFFS)/SECTIONS)
DISP_IMG_HEIGHT =SECTIONS*SECTION_HEIGHT
BYTES_PER_PIXEL =1
TOTAL_BYTES     =DISP_IMG_HEIGHT*IMG_WIDTH*BYTES_PER_PIXEL
ADDRESS_OFFSET  =int(TOTAL_BYTES/SECTIONS)
image_size = int(IMG_WIDTH*IMG_HEIGHT)
ZED_IMAGE_WIDTH   =1344
ZED_IMAGE_WIDTH_2 = int(ZED_IMAGE_WIDTH/2)
ZED_IMAGE_HEIGHT  =376

#overlay = Overlay('/home/xilinx/sgm_pynq_ver/census_mgm_multi_bit/design_1.bit')
overlay = Overlay('./Bitstream/9Sections/design_1.bit')
overlay
ffi = cffi.FFI() #Being used to convert FPGA accessable memory to np array
capl = cv2.VideoCapture(0)

#image = cv2.imread('/home/xilinx/sgm_pynq_ver/test_images/right_image32.jpg', 0);
imagel = np.zeros((ZED_IMAGE_HEIGHT,ZED_IMAGE_WIDTH),dtype=np.ubyte)
imager = np.zeros((ZED_IMAGE_HEIGHT,ZED_IMAGE_WIDTH),dtype=np.ubyte)
rectified_left = np.zeros((ZED_IMAGE_HEIGHT,ZED_IMAGE_WIDTH),dtype=np.ubyte)
rectified_right = np.zeros((ZED_IMAGE_HEIGHT,ZED_IMAGE_WIDTH),dtype=np.ubyte)
buffer_left = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)
buffer_right = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)

# imagel[0:ZED_IMAGE_HEIGHT,0:640] = image[0:ZED_IMAGE_HEIGHT,0:640]
# imager[0:ZED_IMAGE_HEIGHT,0:640] = image[0:ZED_IMAGE_HEIGHT,672:1312]

fs_left = cv2.FileStorage("./zed_left_calibration.yml", cv2.FILE_STORAGE_READ)
fs_right = cv2.FileStorage("./zed_right_calibration.yml", cv2.FILE_STORAGE_READ)
left_rmap0 = fs_left.getNode("rmap0").mat()
left_rmap1 = fs_left.getNode("rmap1").mat()
right_rmap0 = fs_right.getNode("rmap0").mat()
right_rmap1 = fs_right.getNode("rmap1").mat()



print(overlay.ip_dict.keys())

SGM_GreyCost_0 = overlay.SGM_GreyCost_0
SGM_GreyCost_1 = overlay.SGM_GreyCost_1
SGM_GreyCost_2 = overlay.SGM_GreyCost_2
SGM_GreyCost_3 = overlay.SGM_GreyCost_3
SGM_GreyCost_4 = overlay.SGM_GreyCost_4
SGM_GreyCost_5 = overlay.SGM_GreyCost_5
SGM_GreyCost_6 = overlay.SGM_GreyCost_6
SGM_GreyCost_7 = overlay.SGM_GreyCost_7
#SGM_GreyCost_8 = overlay.SGM_GreyCost_8
median_filter = overlay.noise_filter_0

#sgm offsets
inL_offs = SGM_GreyCost_0.register_map.inL.address
inR_offs = SGM_GreyCost_0.register_map.inR.address
outD_offs = SGM_GreyCost_0.register_map.outD.address
CTRL_reg_offset = SGM_GreyCost_0.register_map.CTRL.address

#filter offsets
filter_inp = median_filter.register_map.in_r.address
filter_out  = median_filter.register_map.out_r.address

xlnk = Xlnk()

BufferSize = 0x0A00000
Image_buf =  xlnk.cma_alloc(BufferSize, data_type = "unsigned char")
Img_py_buffer = ffi.buffer(Image_buf,BufferSize)  #converts c data object to python memory readable object

Image_buf_phy_addr = xlnk.cma_get_phy_addr(Image_buf)

#Mapping the left and right images to the buffer space
LeftImg = np.frombuffer(Img_py_buffer, dtype=np.ubyte, \
	count = image_size,offset = 0*image_size).reshape((IMG_HEIGHT,IMG_WIDTH))
RightImg = np.frombuffer(Img_py_buffer, dtype=np.ubyte, \
	count = image_size,offset = 1*image_size).reshape((IMG_HEIGHT,IMG_WIDTH))
#Mapping the dispartiy images to the buffer space
disp_im_buffer = np.frombuffer(Img_py_buffer, dtype=np.ubyte, \
	count = image_size,offset = 2*image_size).reshape((IMG_HEIGHT,IMG_WIDTH))
filter_out_buffer = np.frombuffer(Img_py_buffer, dtype=np.ubyte, \
	count = image_size,offset = 3*image_size).reshape((IMG_HEIGHT,IMG_WIDTH))

SGM_GreyCost_0.write(inL_offs,Image_buf_phy_addr+0*image_size)
SGM_GreyCost_0.write(inR_offs,Image_buf_phy_addr+1*image_size)
SGM_GreyCost_0.write(outD_offs,Image_buf_phy_addr+2*image_size)
SGM_GreyCost_1.write(inL_offs,Image_buf_phy_addr+0*image_size + ADDRESS_OFFSET)
SGM_GreyCost_1.write(inR_offs,Image_buf_phy_addr+1*image_size + ADDRESS_OFFSET)
SGM_GreyCost_1.write(outD_offs,Image_buf_phy_addr+2*image_size + ADDRESS_OFFSET)
SGM_GreyCost_2.write(inL_offs,Image_buf_phy_addr+0*image_size + 2*ADDRESS_OFFSET)
SGM_GreyCost_2.write(inR_offs,Image_buf_phy_addr+1*image_size + 2*ADDRESS_OFFSET)
SGM_GreyCost_2.write(outD_offs,Image_buf_phy_addr+2*image_size + 2*ADDRESS_OFFSET)
SGM_GreyCost_3.write(inL_offs,Image_buf_phy_addr+0*image_size + 3*ADDRESS_OFFSET)
SGM_GreyCost_3.write(inR_offs,Image_buf_phy_addr+1*image_size + 3*ADDRESS_OFFSET)
SGM_GreyCost_3.write(outD_offs,Image_buf_phy_addr+2*image_size + 3*ADDRESS_OFFSET)
SGM_GreyCost_4.write(inL_offs,Image_buf_phy_addr+0*image_size + 4*ADDRESS_OFFSET)
SGM_GreyCost_4.write(inR_offs,Image_buf_phy_addr+1*image_size + 4*ADDRESS_OFFSET)
SGM_GreyCost_4.write(outD_offs,Image_buf_phy_addr+2*image_size + 4*ADDRESS_OFFSET)
SGM_GreyCost_5.write(inL_offs,Image_buf_phy_addr+0*image_size + 5*ADDRESS_OFFSET)
SGM_GreyCost_5.write(inR_offs,Image_buf_phy_addr+1*image_size + 5*ADDRESS_OFFSET)
SGM_GreyCost_5.write(outD_offs,Image_buf_phy_addr+2*image_size + 5*ADDRESS_OFFSET)
SGM_GreyCost_6.write(inL_offs,Image_buf_phy_addr+0*image_size + 6*ADDRESS_OFFSET)
SGM_GreyCost_6.write(inR_offs,Image_buf_phy_addr+1*image_size + 6*ADDRESS_OFFSET)
SGM_GreyCost_6.write(outD_offs,Image_buf_phy_addr+2*image_size + 6*ADDRESS_OFFSET)
SGM_GreyCost_7.write(inL_offs,Image_buf_phy_addr+0*image_size + 7*ADDRESS_OFFSET)
SGM_GreyCost_7.write(inR_offs,Image_buf_phy_addr+1*image_size + 7*ADDRESS_OFFSET)
SGM_GreyCost_7.write(outD_offs,Image_buf_phy_addr+2*image_size + 7*ADDRESS_OFFSET)
# SGM_GreyCost_8.write(inL_offs,Image_buf_phy_addr+0*image_size + 8*ADDRESS_OFFSET)
# SGM_GreyCost_8.write(inR_offs,Image_buf_phy_addr+1*image_size + 8*ADDRESS_OFFSET)
# SGM_GreyCost_8.write(outD_offs,Image_buf_phy_addr+2*image_size + 8*ADDRESS_OFFSET)
median_filter.write(filter_inp,Image_buf_phy_addr+2*image_size)
median_filter.write(filter_out,Image_buf_phy_addr+3*image_size)


for i in range(30):
	ret, framel = capl.read()



#SGM_GreyCost_8.write(CTRL_reg_offset,0b10000001)
SGM_GreyCost_7.write(CTRL_reg_offset,0b10000001)
SGM_GreyCost_6.write(CTRL_reg_offset,0b10000001)
SGM_GreyCost_5.write(CTRL_reg_offset,0b10000001)
SGM_GreyCost_4.write(CTRL_reg_offset,0b10000001)
SGM_GreyCost_3.write(CTRL_reg_offset,0b10000001)
SGM_GreyCost_2.write(CTRL_reg_offset,0b10000001)
SGM_GreyCost_1.write(CTRL_reg_offset,0b10000001)
SGM_GreyCost_0.write(CTRL_reg_offset,0b10000001)
median_filter.write(CTRL_reg_offset,0b10000001)

count = 0
trigger = True
while(count < 5000 and trigger):
	try:
		#print(count)
		count = count +1
		ret, framel = capl.read()
		image = cv2.cvtColor(framel, cv2.COLOR_BGR2GRAY)
		imagel[0:ZED_IMAGE_HEIGHT,0:ZED_IMAGE_WIDTH_2] = image[0:ZED_IMAGE_HEIGHT,0:ZED_IMAGE_WIDTH_2]
		imager[0:ZED_IMAGE_HEIGHT,0:ZED_IMAGE_WIDTH_2] = image[0:ZED_IMAGE_HEIGHT,ZED_IMAGE_WIDTH_2:ZED_IMAGE_WIDTH]
		rectified_left = cv2.remap (imagel, left_rmap0, left_rmap1, cv2.INTER_LINEAR)
		rectified_right = cv2.remap (imager, right_rmap0, right_rmap1, cv2.INTER_LINEAR)
		buffer_left[0:ZED_IMAGE_HEIGHT,0:640] = rectified_left[0:ZED_IMAGE_HEIGHT,0:IMG_WIDTH]
		buffer_right[0:ZED_IMAGE_HEIGHT,0:640] = rectified_right[0:ZED_IMAGE_HEIGHT,0:IMG_WIDTH]
		np.copyto(LeftImg,buffer_left)
		np.copyto(RightImg,buffer_right)
		cv2.imshow('disparity',disp_im_buffer)
		#cv2.imshow("filtered",filter_out_buffer)
		cv2.waitKey(1)
	except:
		trigger = False


#SGM_GreyCost_8.write(CTRL_reg_offset,0b00000000)
SGM_GreyCost_7.write(CTRL_reg_offset,0b00000000)
SGM_GreyCost_6.write(CTRL_reg_offset,0b00000000)
SGM_GreyCost_5.write(CTRL_reg_offset,0b00000000)
SGM_GreyCost_4.write(CTRL_reg_offset,0b00000000)
SGM_GreyCost_3.write(CTRL_reg_offset,0b00000000)
SGM_GreyCost_2.write(CTRL_reg_offset,0b00000000)
SGM_GreyCost_1.write(CTRL_reg_offset,0b00000000)
SGM_GreyCost_0.write(CTRL_reg_offset,0b00000000)


cv2.imwrite('/home/xilinx/MGM_Ultra96_ver/output_images/raw_iml_buffer.png',buffer_left)
cv2.imwrite('/home/xilinx/MGM_Ultra96_ver/output_images/raw_imr_buffer.png',buffer_right)
cv2.imwrite('/home/xilinx/MGM_Ultra96_ver/output_images/rec_iml_buffer.png',rectified_left)
cv2.imwrite('/home/xilinx/MGM_Ultra96_ver/output_images/rec_imr_buffer.png',rectified_right)
np.copyto(buffer_left,disp_median_buff)
cv2.imwrite('/home/xilinx/MGM_Ultra96_ver/output_images/disp_im_buffer.png',buffer_left)


capl.release()
xlnk.cma_free(Image_buf)



