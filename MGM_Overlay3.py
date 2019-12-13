#Added a grace offset between sections to remove the discontinuities with median filter to the output 
#Transmitting the output through a socket to be read by octomap for further processing.
from pynq import Overlay
from pynq import Xlnk
import numpy as np
import cv2
import struct
import sys
import time
import struct
import cffi
import subprocess 
import os 
import re 


IMG_WIDTH         =640
IMG_HEIGHT        =480
FILTER_SIZE     =11
FILTER_OFFS     =(int)(FILTER_SIZE/2)
SECTIONS        =5
SECTION_GRACE	=10
SECTION_HEIGHT  =(int)((IMG_HEIGHT-2*FILTER_OFFS)/SECTIONS + SECTION_GRACE)
DISP_IMG_HEIGHT =SECTIONS*SECTION_HEIGHT
BYTES_PER_PIXEL =1
TOTAL_BYTES     =DISP_IMG_HEIGHT*IMG_WIDTH*BYTES_PER_PIXEL
ADDRESS_OFFSET  =int(TOTAL_BYTES/SECTIONS)
SECTION_HEIGHT_2  =(int)((IMG_HEIGHT-2*FILTER_OFFS)/SECTIONS)
DISP_IMG_HEIGHT_2 =SECTIONS*SECTION_HEIGHT_2
TOTAL_BYTES_2     =DISP_IMG_HEIGHT_2*IMG_WIDTH*BYTES_PER_PIXEL
ADDRESS_OFFSET_2  =int(TOTAL_BYTES_2/SECTIONS)
image_size = int(IMG_WIDTH*IMG_HEIGHT)


def execute_octomap(location): 
  
	# create a pipe to a child process 
	#data, temp = os.pipe() 
  
	# write to STDIN as a byte object(covert string 
	# to bytes with encoding utf8) 
	#os.write(temp, bytes("5 10\n", "utf-8")); 
	#os.close(temp)
	print(location)
	# store output of the program as a byte string in s 
	# s = subprocess.check_output("../octomap/octomap/bin/shared_mem_to_bt "+location, stdin = data, shell = True) 
	# stream = os.popen("../octomap/octomap/bin/shared_mem_to_bt "+location)
	# output = stream.read()
	# print(output)
	os.system("../octomap/octomap/bin/shared_mem_to_bt "+location)
	# decode s to a normal string 
	#print(s.decode("utf-8")) 
	#print(s)

if __name__ == "__main__":
	#overlay = Overlay('/home/xilinx/sgm_pynq_ver/census_mgm_multi_bit/design_1.bit')
	overlay = Overlay('./Bitstream/5Sections/design_1.bit')
	overlay
	ffi = cffi.FFI() #Being used to convert FPGA accessable memory to np array
	capl = cv2.VideoCapture(0)
	capr = cv2.VideoCapture(1)

	# imagel = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)
	# imager = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)
	# rectified_left = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)
	# rectified_right = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)
	# buffer_left = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)
	# buffer_right = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)
	# disp_median_buff = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)

	fs_left = cv2.FileStorage("./elp_left_calibration.yml", cv2.FILE_STORAGE_READ)
	fs_right = cv2.FileStorage("./elp_right_calibration.yml", cv2.FILE_STORAGE_READ)
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
		count = image_size+4*SECTION_GRACE*IMG_WIDTH,offset = 2*image_size).reshape((IMG_HEIGHT+4*SECTION_GRACE,IMG_WIDTH))
	filter_inp_buffer = np.frombuffer(Img_py_buffer, dtype=np.ubyte, \
		count = image_size,offset = 3*image_size+4*SECTION_GRACE*IMG_WIDTH).reshape((IMG_HEIGHT,IMG_WIDTH))
	filter_out_buffer = np.frombuffer(Img_py_buffer, dtype=np.ubyte, \
		count = image_size,offset = 4*image_size+4*SECTION_GRACE*IMG_WIDTH).reshape((IMG_HEIGHT,IMG_WIDTH))

	SGM_GreyCost_0.write(inL_offs,Image_buf_phy_addr+0*image_size)
	SGM_GreyCost_0.write(inR_offs,Image_buf_phy_addr+1*image_size)
	SGM_GreyCost_0.write(outD_offs,Image_buf_phy_addr+2*image_size)
	SGM_GreyCost_1.write(inL_offs,Image_buf_phy_addr+0*image_size + ADDRESS_OFFSET_2)
	SGM_GreyCost_1.write(inR_offs,Image_buf_phy_addr+1*image_size + ADDRESS_OFFSET_2)
	SGM_GreyCost_1.write(outD_offs,Image_buf_phy_addr+2*image_size + ADDRESS_OFFSET)
	SGM_GreyCost_2.write(inL_offs,Image_buf_phy_addr+0*image_size + 2*ADDRESS_OFFSET_2)
	SGM_GreyCost_2.write(inR_offs,Image_buf_phy_addr+1*image_size + 2*ADDRESS_OFFSET_2)
	SGM_GreyCost_2.write(outD_offs,Image_buf_phy_addr+2*image_size + 2*ADDRESS_OFFSET)
	SGM_GreyCost_3.write(inL_offs,Image_buf_phy_addr+0*image_size + 3*ADDRESS_OFFSET_2)
	SGM_GreyCost_3.write(inR_offs,Image_buf_phy_addr+1*image_size + 3*ADDRESS_OFFSET_2)
	SGM_GreyCost_3.write(outD_offs,Image_buf_phy_addr+2*image_size + 3*ADDRESS_OFFSET)
	SGM_GreyCost_4.write(inL_offs,Image_buf_phy_addr+0*image_size + 4*ADDRESS_OFFSET_2)
	SGM_GreyCost_4.write(inR_offs,Image_buf_phy_addr+1*image_size + 4*ADDRESS_OFFSET_2)
	SGM_GreyCost_4.write(outD_offs,Image_buf_phy_addr+2*image_size + 4*ADDRESS_OFFSET)
	median_filter.write(filter_inp,Image_buf_phy_addr+3*image_size+4*SECTION_GRACE*IMG_WIDTH)
	median_filter.write(filter_out, Image_buf_phy_addr+4*image_size+4*SECTION_GRACE*IMG_WIDTH)

	for i in range(30):
		ret, framel = capl.read()



	SGM_GreyCost_4.write(CTRL_reg_offset,0b10000001)
	SGM_GreyCost_3.write(CTRL_reg_offset,0b10000001)
	SGM_GreyCost_2.write(CTRL_reg_offset,0b10000001)
	SGM_GreyCost_1.write(CTRL_reg_offset,0b10000001)
	SGM_GreyCost_0.write(CTRL_reg_offset,0b10000001)
	median_filter.write(CTRL_reg_offset,0b10000001)

	count = 0
	trigger = True
	while(count < 5000 and trigger == True):
		try:		
			#print(count)
			count = count +1
			ret, framel = capr.read()
			ret, framer = capl.read()
			imagel = cv2.cvtColor(framel, cv2.COLOR_BGR2GRAY)
			imager = cv2.cvtColor(framer, cv2.COLOR_BGR2GRAY)
			rectified_left = cv2.remap (imagel, left_rmap0, left_rmap1, cv2.INTER_LINEAR)
			rectified_right = cv2.remap (imager, right_rmap0, right_rmap1, cv2.INTER_LINEAR)
			np.copyto(LeftImg,rectified_left)
			np.copyto(RightImg,rectified_right)
			disp_median_buff = np.row_stack((disp_im_buffer[0:SECTION_HEIGHT,0:640],\
				disp_im_buffer[SECTION_HEIGHT+SECTION_GRACE:2*SECTION_HEIGHT,0:640],\
				disp_im_buffer[SECTION_HEIGHT*2+SECTION_GRACE:3*SECTION_HEIGHT,0:640],\
				disp_im_buffer[SECTION_HEIGHT*3+SECTION_GRACE:4*SECTION_HEIGHT,0:640],\
				disp_im_buffer[SECTION_HEIGHT*4+SECTION_GRACE:5*SECTION_HEIGHT+4*SECTION_GRACE,0:640]))
			#np.copyto(disp_median_buff,disp_im_buffer)
			#median =  cv2.medianBlur(disp_median_buff,3)
			#cv2.imshow('disparity',median)
			np.copyto(filter_inp_buffer,disp_median_buff)
			#cv2.imshow('disparity',imagel)
			#cv2.imshow('disparity2',rectified_left)
			#execute_octomap(str(Image_buf_phy_addr + 4*image_size + 4*SECTION_GRACE*IMG_WIDTH))
			#jet_color = cv2.applyColorMap(filter_out_buffer*2,cv2.COLORMAP_JET)
			cv2.imshow("filtered",filter_out_buffer)
			cv2.waitKey(1)
		except:
			trigger = False


	SGM_GreyCost_4.write(CTRL_reg_offset,0b00000000)
	SGM_GreyCost_3.write(CTRL_reg_offset,0b00000000)
	SGM_GreyCost_2.write(CTRL_reg_offset,0b00000000)
	SGM_GreyCost_1.write(CTRL_reg_offset,0b00000000)
	SGM_GreyCost_0.write(CTRL_reg_offset,0b00000000)
	median_filter.write(CTRL_reg_offset,0b00000000)


	cv2.imwrite('./output_images/raw_iml_buffer.png',imagel)
	cv2.imwrite('./output_images/raw_imr_buffer.png',imager)
	cv2.imwrite('./output_images/rec_iml_buffer.png',rectified_left)
	cv2.imwrite('./output_images/rec_imr_buffer.png',rectified_right)
	np.copyto(imagel,filter_out_buffer)
	cv2.imwrite('./output_images/disp_im_buffer.png',imagel)


	capl.release()
	xlnk.cma_free(Image_buf)



