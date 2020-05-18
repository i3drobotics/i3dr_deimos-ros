/* This file is from the uvc_cam package by Morgan Quigley */
#include <cstring>
#include <string>
#include <cstdio>
#include <stdexcept>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <libv4l2.h>
#include <glib.h>
#include <iostream>
#include <limits>
#include "uvc_cam/uvc_cam.h"

using std::string;
using namespace uvc_cam;


	static void
enumerate_menu (int device_file_h_,
		struct v4l2_queryctrl &queryctrl,
		struct v4l2_querymenu &querymenu)
{
	printf ("  Menu items:\n");

	memset (&querymenu, 0, sizeof (querymenu));
	querymenu.id = queryctrl.id;

	for (querymenu.index = queryctrl.minimum;
			querymenu.index <= queryctrl.maximum;
			querymenu.index++) {
		if (0 == ioctl (device_file_h_, VIDIOC_QUERYMENU, &querymenu)) {
			printf ("  %s\n", querymenu.name);
		}
	}
}


	Cam::Cam(const char *_device, mode_t _mode, int _width, int _height, int _fps)
: mode_(_mode), device_(_device),
	motion_threshold_luminance_(100), motion_threshold_count(-1),
	width_(_width), height_(_height), fps_(_fps), rgb_frame_(NULL)
{
	//enumerate();

	GetListofDeviceseCon();

	index = ( _device[10] ) - 48;

//	printf("opening %s\n", _device);

	if ((device_file_h_ = open(_device, O_RDWR)) == -1)
	{
		throw std::runtime_error("couldn't open " + device_);
	}

	DeviceInfo = DeviceInstances->listVidDevices[index].bus_info;

	memset(&format_, 0, sizeof(v4l2_format));
	memset(&capability_, 0, sizeof(v4l2_capability));

	if (ioctl(device_file_h_, VIDIOC_QUERYCAP, &capability_) < 0)
		throw std::runtime_error("couldn't query " + device_);
	if (!(capability_.capabilities & V4L2_CAP_VIDEO_CAPTURE))
		throw std::runtime_error(device_ + " does not support capture");
	if (!(capability_.capabilities & V4L2_CAP_STREAMING))
		throw std::runtime_error(device_ + " does not support streaming");

	// enumerate formats
	v4l2_fmtdesc format_desc;
	memset(&format_desc, 0, sizeof(format_desc));
	format_desc.index = 0;
	format_desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int ret;

//	printf("## FORMATS: ##\n");

	while ((ret = ioctl(device_file_h_, VIDIOC_ENUM_FMT, &format_desc)) == 0)
	{
/*
		printf("pixfmt %d = '%4s' desc = '%s'\n",
				format_desc.index, (char *)&format_desc.pixelformat, format_desc.description);
*/
		format_desc.index++;

		// enumerate frame sizes
		v4l2_frmsizeenum fsize;
		fsize.index = 0;
		fsize.pixel_format = format_desc.pixelformat;
		while ((ret = ioctl(device_file_h_, VIDIOC_ENUM_FRAMESIZES, &fsize)) == 0)
		{
			fsize.index++;
			if (fsize.type == V4L2_FRMSIZE_TYPE_DISCRETE)
			{
//				printf("  discrete: %ux%u:   ",fsize.discrete.width, fsize.discrete.height);

				// enumerate frame rates
				v4l2_frmivalenum fival;
				fival.index = 0;
				fival.pixel_format = format_desc.pixelformat;
				fival.width = fsize.discrete.width;
				fival.height = fsize.discrete.height;
/*
				while ((ret = ioctl(device_file_h_, VIDIOC_ENUM_FRAMEINTERVALS, &fival)) == 0)
				{
					fival.index++;
					if (fival.type == V4L2_FRMIVAL_TYPE_DISCRETE)
					{
						printf("%u/%u ",fival.discrete.numerator, fival.discrete.denominator);
					}
					else
						printf("I only handle discrete frame intervals...\n");
				}
				printf("\n");
*/
			}
			else if (fsize.type == V4L2_FRMSIZE_TYPE_CONTINUOUS)
			{
				printf("  continuous: %ux%u to %ux%u\n",
						fsize.stepwise.min_width, fsize.stepwise.min_height,
						fsize.stepwise.max_width, fsize.stepwise.max_height);
			}
			else if (fsize.type == V4L2_FRMSIZE_TYPE_STEPWISE)
			{
				printf("  stepwise: %ux%u to %ux%u step %ux%u\n",
						fsize.stepwise.min_width,  fsize.stepwise.min_height,
						fsize.stepwise.max_width,  fsize.stepwise.max_height,
						fsize.stepwise.step_width, fsize.stepwise.step_height);
			}
			else
			{
				printf("  fsize.type not supported: %d\n", fsize.type);
			}
		}
	}

	if (errno != EINVAL)
		throw std::runtime_error("error enumerating frame formats");

	///////////////////////////////////////////

	struct v4l2_queryctrl queryctrl;
	struct v4l2_querymenu querymenu;

	memset (&queryctrl, 0, sizeof (queryctrl));

//	printf("## CONTROLS: ##\n");

	for (queryctrl.id = V4L2_CID_BASE;
			queryctrl.id < V4L2_CID_LASTP1;
			queryctrl.id++) {
		if (0 == ioctl (device_file_h_, VIDIOC_QUERYCTRL, &queryctrl)) {
			if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
				continue;

//			printf ("Control '%s'\n", queryctrl.name);

			//if (queryctrl.type == V4L2_CTRL_TYPE_MENU)
			//	enumerate_menu (device_file_h_,queryctrl,querymenu);
		} else {
			if (errno == EINVAL)
				continue;

			perror ("VIDIOC_QUERYCTRL");
			//exit (EXIT_FAILURE);
		}
	}

	for (queryctrl.id = V4L2_CID_PRIVATE_BASE;;
			queryctrl.id++) {
		if (0 == ioctl (device_file_h_, VIDIOC_QUERYCTRL, &queryctrl)) {
			if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
				continue;

//			printf ("Control '%s'\n", queryctrl.name);

			//if (queryctrl.type == V4L2_CTRL_TYPE_MENU)
			//enumerate_menu (device_file_h_,queryctrl,querymenu);
		} else {
			if (errno == EINVAL)
				break;

			perror ("VIDIOC_QUERYCTRL");
			//exit (EXIT_FAILURE);
		}
	}

	//////////////////////////////////////////

	format_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	format_.fmt.pix.width = width_;
	format_.fmt.pix.height = height_;

	if (mode_ == MODE_RGB || mode_ == MODE_YUYV) // we'll convert later
		format_.fmt.pix.pixelformat = 'Y' | ('U' << 8) | ('Y' << 16) | ('V' << 24);
	else if (mode_ == MODE_BAYER)
		format_.fmt.pix.pixelformat = 'B' | ('A' << 8) | ('8' << 16) | ('1' << 24);
	else
		format_.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;

	format_.fmt.pix.field = V4L2_FIELD_ANY;

	if ((ret = ioctl(device_file_h_, VIDIOC_S_FMT, &format_)) < 0)
		throw std::runtime_error("couldn't set format");

	if (format_.fmt.pix.width != width_ || format_.fmt.pix.height != height_)
		throw std::runtime_error("pixel format unavailable");

	stream_parm_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	stream_parm_.parm.capture.timeperframe.numerator = 1;
	stream_parm_.parm.capture.timeperframe.denominator = fps_;
	if ((ret = ioctl(device_file_h_, VIDIOC_S_PARM, &stream_parm_)) < 0)
		throw std::runtime_error("unable to set framerate");
	//v4l2_queryctrl queryctrl;
	memset(&queryctrl, 0, sizeof(queryctrl));

//	printf("## CONTROLS: ##\n");

	uint32_t i = V4L2_CID_BASE;
	while (i != V4L2_CID_LAST_EXTCTR)
	{
		queryctrl.id = i;
		if ((ret = ioctl(device_file_h_, VIDIOC_QUERYCTRL, &queryctrl)) == 0 &&
				!(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED))
		{
/*
			const char *ctrl_type = NULL;
			if (queryctrl.type == V4L2_CTRL_TYPE_INTEGER)
				ctrl_type = "int";
			else if (queryctrl.type == V4L2_CTRL_TYPE_BOOLEAN)
				ctrl_type = "bool";
			else if (queryctrl.type == V4L2_CTRL_TYPE_BUTTON)
				ctrl_type = "button";
			else if (queryctrl.type == V4L2_CTRL_TYPE_MENU)
				ctrl_type = "menu";
			printf("  %s (%s, %d, id = %x): %d to %d (%d)\n",
					ctrl_type,
					queryctrl.name, queryctrl.flags, queryctrl.id,
					queryctrl.minimum, queryctrl.maximum, queryctrl.step);
	
			if (queryctrl.type == V4L2_CTRL_TYPE_MENU)
			{
				v4l2_querymenu querymenu;
				memset(&querymenu, 0, sizeof(querymenu));
				querymenu.id = queryctrl.id;
				querymenu.index = 0;
				while (ioctl(device_file_h_, VIDIOC_QUERYMENU, &querymenu) == 0)
				{
					printf("    %d: %s\n", querymenu.index, querymenu.name);
					querymenu.index++;
				}
			}
*/
		}
		else if (errno != EINVAL)
			throw std::runtime_error("couldn't query control");
		i++;
		if (i == V4L2_CID_LAST_NEW)
			i = V4L2_CID_CAMERA_CLASS_BASE_NEW;
		else if (i == V4L2_CID_CAMERA_CLASS_LAST)
			i = V4L2_CID_PRIVATE_BASE_OLD;
		else if (i == V4L2_CID_PRIVATE_LAST)
			i = V4L2_CID_BASE_EXTCTR;
	}

	try
	{
		// the commented labels correspond to the controls in guvcview and uvcdynctrl

		//set_control(0x009a0901, 0); // exposure, auto (0 = auto, 1 = manual)
		//set_control(0x00980900, 4); // brightness


		//set_control(V4L2_CID_EXPOSURE_AUTO_NEW, 2);
		//set_control(10094851, 1); // Exposure, Auto Priority
		//set_control(10094849, 1); // Exposure, Auto
		//set_control(168062321, 0); //Disable video processing
		//set_control(0x9a9010, 100);
		//set_control(V4L2_CID_EXPOSURE_ABSOLUTE_NEW, 300);
		//set_control(V4L2_CID_BRIGHTNESS, 140);
		//set_control(V4L2_CID_CONTRAST, 40);
		//set_control(V4L2_CID_WHITE_BALANCE_TEMP_AUTO_OLD, 0);
		//set_control(V4L2_CID_WHITE_BALANCE_TEMPERATURE_NEW, 0);
		//set_control(9963776, 128); //Brightness
		//set_control(9963777, 32); //Contrast
		//set_control(9963788, 0); // White Balance Temperature, Auto
		//set_control(9963802, 5984); // White Balance Temperature
		//set_control(9963800, 2);  // power line frequency to 60 hz
		//set_control(9963795, 200); // Gain
		//set_control(9963803, 224); // Sharpness
		//set_control(9963804, 1); //Backlight Compensation
		//set_control(10094850, 250); // Exposure (Absolute)
		//set_control(168062212, 16); //Focus (absolute)
		//set_control(168062213, 3); //LED1 Mode
		//set_control(168062214, 0); //LED1 Frequency
		//set_control(9963778, 32); // Saturation
	}
	catch (std::runtime_error &ex)
	{

		printf("ERROR: could not set some settings.  \n %s \n", ex.what());
	}

	/*
	   v4l2_jpegcompression v4l2_jpeg;
	   if (ioctl(fd, VIDIOC_G_JPEGCOMP, &v4l2_jpeg) < 0)
	   throw std::runtime_error("no jpeg compression iface exposed");
	   printf("jpeg quality: %d\n", v4l2_jpeg.quality);
	 */

	memset(&request_buffers_, 0, sizeof(request_buffers_));
	request_buffers_.count = NUM_BUFFERS;
	request_buffers_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	request_buffers_.memory = V4L2_MEMORY_MMAP;
	if (ioctl(device_file_h_, VIDIOC_REQBUFS, &request_buffers_) < 0)
		throw std::runtime_error("unable to allocate buffers");
	for (unsigned i = 0; i < NUM_BUFFERS; i++)
	{
		memset(&buffer_, 0, sizeof(buffer_));
		buffer_.index = i;
		buffer_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buffer_.flags = V4L2_BUF_FLAG_TIMECODE;
		buffer_.timecode = time_code_;
		buffer_.timestamp.tv_sec = 0;
		buffer_.timestamp.tv_usec = 0;
		buffer_.memory = V4L2_MEMORY_MMAP;
		if (ioctl(device_file_h_, VIDIOC_QUERYBUF, &buffer_) < 0)
			throw std::runtime_error("unable to query buffer");
		if (buffer_.length <= 0)
			throw std::runtime_error("buffer length is bogus");
		buffer_mem_[i] = mmap(0, buffer_.length, PROT_READ, MAP_SHARED, device_file_h_, buffer_.m.offset);
		//printf("buf length = %d at %x\n", buf.length, mem[i]);
		if (buffer_mem_[i] == MAP_FAILED)
			throw std::runtime_error("couldn't map buffer");
	}
	buffer_length_ = buffer_.length;
	for (unsigned i = 0; i < NUM_BUFFERS; i++)
	{
		memset(&buffer_, 0, sizeof(buffer_));
		buffer_.index = i;
		buffer_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buffer_.flags = V4L2_BUF_FLAG_TIMECODE;
		buffer_.timecode = time_code_;
		buffer_.timestamp.tv_sec = 0;
		buffer_.timestamp.tv_usec = 0;
		buffer_.memory = V4L2_MEMORY_MMAP;
		if (ioctl(device_file_h_, VIDIOC_QBUF, &buffer_) < 0)
			throw std::runtime_error("unable to queue buffer");
	}
	int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(device_file_h_, VIDIOC_STREAMON, &type) < 0)
		throw std::runtime_error("unable to start capture");
	rgb_frame_ = new unsigned char[width_ * height_ * 3];
	last_yuv_frame_ = new unsigned char[width_ * height_ * 2];
	y16_frame_ = new unsigned char[width_ * height_ * 2];
	left_frame_ = new unsigned char[width_ * height_];
	right_frame_ = new unsigned char[width_ * height_];
	concat_frame_ = new unsigned char[width_ * height_ * 2];


	// initialize see3cam extension unit
	//  InitExtensionUnit( (const char*)capability_.bus_info );
	if(IsStereoDeviceAvail(DeviceInstances->listVidDevices[index].product))
	{
		IsStereo = true;
	}
	else
	{
		IsStereo = false;
	}
	if (true == IsStereo)
	{
		if(!InitExtensionUnit(DeviceInfo))
		{			
			cout << "InitCamera : Extension Unit Initialisation Failed\n";
		}
	}

	//  EnableTriggerMode();
}

BOOL Cam::IsStereoDeviceAvail(char *pid)
{
	if(strcmp(pid, See3CAM_STEREO) == 0)
		return TRUE;
	else
		return FALSE;
}

void Cam::showFirmwareVersion()
{
	if ( true == ReadFirmwareVersion( &MajorVersion, &MinorVersion1, &MinorVersion2, &MinorVersion3 ))
	{
		printf ("\nfirmwareversion of the camera is %d : %d : %d : %d\n", MajorVersion, MinorVersion1, MinorVersion2, MinorVersion3);
	}
	else
	{
		printf ("failed to read firmware version\n");
	}
}	

Cam::~Cam()
{
	// stop stream
	int type = V4L2_BUF_TYPE_VIDEO_CAPTURE, ret;
	if ((ret = ioctl(device_file_h_, VIDIOC_STREAMOFF, &type)) < 0)
		perror("VIDIOC_STREAMOFF");
	for (unsigned i = 0; i < NUM_BUFFERS; i++)
		if (munmap(buffer_mem_[i], buffer_length_) < 0)
			perror("failed to unmap buffer");
	close(device_file_h_);
	if (rgb_frame_)
	{
		delete[] rgb_frame_;
		delete[] last_yuv_frame_;
	}
	last_yuv_frame_ = rgb_frame_ = NULL;

	DeinitExtensionUnit();
	//  UninitExtensionUnit();
}


void Cam::enumerate()
{
	string v4l_path = "/sys/class/video4linux";
	DIR *d = opendir(v4l_path.c_str());
	if (!d)
		throw std::runtime_error("couldn't open " + v4l_path);
	struct dirent *ent, *ent2, *ent3;
	int fd, ret;
	struct v4l2_capability v4l2_cap;
	while ((ent = readdir(d)) != NULL)
	{
		if (strncmp(ent->d_name, "video", 5))
			continue; // ignore anything not starting with "video"
		string dev_name = string("/dev/") + string(ent->d_name);
		printf("enumerating %s ...\n", dev_name.c_str());
		if ((fd = open(dev_name.c_str(), O_RDWR)) == -1)
			throw std::runtime_error("couldn't open " + dev_name + "  perhaps the " +
					"permissions are not set correctly?");
		if ((ret = ioctl(fd, VIDIOC_QUERYCAP, &v4l2_cap)) < 0)
			throw std::runtime_error("couldn't query " + dev_name);
		printf("name = [%s]\n", v4l2_cap.card);
		printf("driver = [%s]\n", v4l2_cap.driver);
		printf("location = [%s]\n", v4l2_cap.bus_info);
		close(fd);
		string v4l_dev_path = v4l_path + string("/") + string(ent->d_name) +
			string("/device");
		// my kernel is using /sys/class/video4linux/videoN/device/inputX/id
		DIR *d2 = opendir(v4l_dev_path.c_str());
		if (!d2)
			throw std::runtime_error("couldn't open " + v4l_dev_path);
		string input_dir;
		while ((ent2 = readdir(d2)) != NULL)
		{
			if (strncmp(ent2->d_name, "input", 5))
				continue; // ignore anything not beginning with "input"

			DIR *input = opendir((v4l_dev_path + string("/") + string(ent2->d_name)).c_str());
			bool output_set = false;
			while ((ent3 = readdir(input)) != NULL)
			{
				if (!strncmp(ent3->d_name, "input", 5))
				{
					input_dir = (string("input/") + string(ent3->d_name )).c_str();
					output_set = true;
					break;
				}
			}
			if (!output_set)
				input_dir = ent2->d_name;
			break;
		}
		printf("input_dir: %s",(v4l_dev_path + string("/") + input_dir).c_str());
		closedir(d2);
		if (!input_dir.length())
			throw std::runtime_error("couldn't find input dir in " + v4l_dev_path);
		string vid_fname = v4l_dev_path + string("/") + input_dir +
			string("/id/vendor");
		string pid_fname = v4l_dev_path + string("/") + input_dir +
			string("/id/product");
		string ver_fname = v4l_dev_path + string("/") + input_dir +
			string("/id/version");
		char vid[5], pid[5], ver[5];
		FILE *vid_fp = fopen(vid_fname.c_str(), "r");
		if (!vid_fp)
			throw std::runtime_error("couldn't open " + vid_fname);
		if (!fgets(vid, sizeof(vid), vid_fp))
			throw std::runtime_error("couldn't read VID from " + vid_fname);
		fclose(vid_fp);
		vid[4] = 0;
		printf("vid = [%s]\n", vid);
		FILE *pid_fp = fopen(pid_fname.c_str(), "r");
		if (!pid_fp)
			throw std::runtime_error("couldn't open " + pid_fname);
		if (!fgets(pid, sizeof(pid), pid_fp))
			throw std::runtime_error("couldn't read PID from " + pid_fname);
		fclose(pid_fp);
		printf("pid = [%s]\n", pid);
		FILE *ver_fp = fopen(ver_fname.c_str(), "r");
		if (!ver_fp)
			throw std::runtime_error("couldn't open " + ver_fname);
		if (!fgets(ver, sizeof(ver), ver_fp))
			throw std::runtime_error("couldn't read version from " + ver_fname);
		fclose(ver_fp);
		printf("ver = [%s]\n", ver);
	}
	closedir(d);

}

// saturate input into [0, 255]
inline unsigned char sat(float f)
{
	return (unsigned char)( f >= 255 ? 255 : (f < 0 ? 0 : f));
}

int Cam::grabStereo(unsigned char **frame, uint32_t &bytes_used, unsigned char **left_frame = NULL, unsigned char **right_frame = NULL, unsigned char **concat_frame = NULL)
{
	*frame = NULL;
	int ret = 0;
	fd_set rdset;
	timeval timeout;
	FD_ZERO(&rdset);
	FD_SET(device_file_h_, &rdset);
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;
	bytes_used = 0;
	ret = select(device_file_h_ + 1, &rdset, NULL, NULL, &timeout);
	if (ret == 0)
	{
		printf("select timeout in grab\n");
		return -1;
	}
	else if (ret < 0)
	{
		perror("couldn't grab image");
		return -1;
	}
	if (!FD_ISSET(device_file_h_, &rdset))
		return -1;
	memset(&buffer_, 0, sizeof(buffer_));
	buffer_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buffer_.memory = V4L2_MEMORY_MMAP;
	if (ioctl(device_file_h_, VIDIOC_DQBUF, &buffer_) < 0)
		throw std::runtime_error("couldn't dequeue buffer");
	bytes_used = buffer_.bytesused;

	if (mode_ == MODE_Y16)
	{
	   		
   	  unsigned char *py16 = (unsigned char *)buffer_mem_[buffer_.index];
//   	 unsigned int var_ = 0;
  	  for (unsigned int i = 0; i < width_ * height_ * 2; i ++)
      { 
    	if (i % 2 == 0)
    	{	
    		concat_frame_ [ (i/(2*width_)) * width_ + (i / 2) ] = py16 [i];
    		left_frame_ [i / 2 ] = py16 [i];
    	}
    	else
    	{
    		concat_frame_ [ (i/(2*width_)) * width_ + (i / 2) + width_ ] = py16 [i];
    		right_frame_ [ i / 2 ] = py16 [i];
    	}
      }
      
      (*concat_frame) = concat_frame_;
      (*frame) = py16;
      (*right_frame) = right_frame_;
      (*left_frame) = left_frame_;
      
	}
  return buffer_.index;
}


int Cam::grab(unsigned char **frame, uint32_t &bytes_used)
{
	*frame = NULL;
	int ret = 0;
	fd_set rdset;
	timeval timeout;
	FD_ZERO(&rdset);
	FD_SET(device_file_h_, &rdset);
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;
	bytes_used = 0;
	ret = select(device_file_h_ + 1, &rdset, NULL, NULL, &timeout);
	if (ret == 0)
	{
		printf("select timeout in grab\n");
		return -1;
	}
	else if (ret < 0)
	{
		perror("couldn't grab image");
		return -1;
	}
	if (!FD_ISSET(device_file_h_, &rdset))
		return -1;
	memset(&buffer_, 0, sizeof(buffer_));
	buffer_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buffer_.memory = V4L2_MEMORY_MMAP;
	if (ioctl(device_file_h_, VIDIOC_DQBUF, &buffer_) < 0)
		throw std::runtime_error("couldn't dequeue buffer");
	bytes_used = buffer_.bytesused;
	if (mode_ == MODE_RGB)
	{
		int num_pixels_different = 0; // just look at the Y channel
		unsigned char *pyuv = (unsigned char *)buffer_mem_[buffer_.index];
		// yuyv is 2 bytes per pixel. step through every pixel pair.
		unsigned char *prgb = rgb_frame_;
		unsigned char *pyuv_last = last_yuv_frame_;
		for (unsigned i = 0; i < width_ * height_ * 2; i += 4)
		{
			*prgb++ = sat(pyuv[i]+1.402f  *(pyuv[i+3]-128));
			*prgb++ = sat(pyuv[i]-0.34414f*(pyuv[i+1]-128)-0.71414f*(pyuv[i+3]-128));
			*prgb++ = sat(pyuv[i]+1.772f  *(pyuv[i+1]-128));
			*prgb++ = sat(pyuv[i+2]+1.402f*(pyuv[i+3]-128));
			*prgb++ = sat(pyuv[i+2]-0.34414f*(pyuv[i+1]-128)-0.71414f*(pyuv[i+3]-128));
			*prgb++ = sat(pyuv[i+2]+1.772f*(pyuv[i+1]-128));
			if ((int)pyuv[i] - (int)pyuv_last[i] > motion_threshold_luminance_ ||
					(int)pyuv_last[i] - (int)pyuv[i] > motion_threshold_luminance_)
				num_pixels_different++;
			if ((int)pyuv[i+2] - (int)pyuv_last[i+2] > motion_threshold_luminance_ ||
					(int)pyuv_last[i+2] - (int)pyuv[i+2] > motion_threshold_luminance_)
				num_pixels_different++;

			// this gives bgr images...
			/*
			 *prgb++ = sat(pyuv[i]+1.772f  *(pyuv[i+1]-128));
			 *prgb++ = sat(pyuv[i]-0.34414f*(pyuv[i+1]-128)-0.71414f*(pyuv[i+3]-128));
			 *prgb++ = sat(pyuv[i]+1.402f  *(pyuv[i+3]-128));
			 *prgb++ = sat(pyuv[i+2]+1.772f*(pyuv[i+1]-128));
			 *prgb++ = sat(pyuv[i+2]-0.34414f*(pyuv[i+1]-128)-0.71414f*(pyuv[i+3]-128));
			 *prgb++ = sat(pyuv[i+2]+1.402f*(pyuv[i+3]-128));
			 */
		}
		memcpy(last_yuv_frame_, pyuv, width_ * height_ * 2);
		if (num_pixels_different > motion_threshold_count) // default: always true
			*frame = rgb_frame_;
		else
		{
			*frame = NULL; // not enough luminance change
			release(buffer_.index); // let go of this image
		}
	}
	else if (mode_ == MODE_YUYV)
	{
		*frame = (uint8_t *)buffer_mem_[buffer_.index];
	}
	else // mode == MODE_JPEG
	{
		//if (bytes_used > 100)
		*frame = (unsigned char *)buffer_mem_[buffer_.index];
	}
	return buffer_.index;
}

void Cam::release(unsigned buf_idx)
{
	if (buf_idx < NUM_BUFFERS)
		if (ioctl(device_file_h_, VIDIOC_QBUF, &buffer_) < 0)
			throw std::runtime_error("couldn't requeue buffer");
}

int xioctl(int fd, int IOCTL_X, void *arg)
{
	const int IOCTL_RETRY = 5;
	int ret = 0;
	int tries= IOCTL_RETRY;
	do
	{
		ret = ioctl(fd, IOCTL_X, arg);
	}
	while (ret && tries-- &&
			((errno == EINTR) || (errno == EAGAIN) || (errno == ETIMEDOUT)));

	if (ret && (tries <= 0)) printf("ioctl (%i) retried %i times - giving up: %s)\n", IOCTL_X, IOCTL_RETRY, strerror(errno));

	return (ret);
}
int Cam::get_control(uint32_t id, int *value)
{
	v4l2_control c;
	c.id = id;

	// get ctrl name
	struct v4l2_queryctrl queryctrl;
	memset (&queryctrl, 0, sizeof (queryctrl));
	queryctrl.id = id;
	if (0 == ioctl (device_file_h_, VIDIOC_QUERYCTRL, &queryctrl))
	{
		if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
		{
			printf ("Control '%s' is disabled.\n", queryctrl.name);
			return false;
		}
	}
	else
	{
		printf("Control #%d does not exist.\n", id);
		return false;
	}

	if (ioctl(device_file_h_, VIDIOC_G_CTRL, &c) == 0)
	{
		(*value) = c.value;
		return true;
	}
	else
	{
		printf ("failed to get the current value of %s\n", queryctrl.name);
	}
	return false;
}

int Cam::set_control(uint32_t id, int val)
{
	v4l2_control c;
	c.id = id;
	// get ctrl name
	struct v4l2_queryctrl queryctrl;
	memset (&queryctrl, 0, sizeof (queryctrl));
	queryctrl.id = id;
	if (0 == ioctl (device_file_h_, VIDIOC_QUERYCTRL, &queryctrl))
	{
		if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
		{
			printf ("Control '%s' is disabled.\n", queryctrl.name);
			return false;
		}
	}
	else
	{
		printf("Control #%d does not exist.\n", id);
		return false;
	}
	if ( val < queryctrl.minimum || val > queryctrl.maximum )
	{
		printf ("Value of cntrol #%s is out of range, Select value between %d and %d\n", queryctrl.name, queryctrl.minimum, queryctrl.maximum );
		return false;
	}

	if (ioctl(device_file_h_, VIDIOC_G_CTRL, &c) == 0)
	{
		printf("Current value of %s is %d\n", queryctrl.name, c.value);
	}

//	printf("Setting control '%s' from %d to %d\n", queryctrl.name, c.value, val);

	c.value = val;
	if (xioctl(device_file_h_, VIDIOC_S_CTRL, &c) < 0)
	{
		printf("unable to set control '%s'!\n", queryctrl.name);
	}
	return true;

}

void Cam::set_motion_thresholds(int lum, int count)
{
	motion_threshold_luminance_ = lum;
	motion_threshold_count = count;
}

int Cam::GetListofDeviceseCon(void)
{
	struct udev_enumerate *enumerate;
	struct udev_list_entry *devices, *dev_list_entry;
	struct udev_device *dev;

	int num_dev = 0;
	int fd = 0;
	struct v4l2_capability v4l2_cap;
	struct udev *udev = udev_new();

	if (!udev)
	{
		/*use fall through method (sysfs)*/
		g_print("Can't create udev...using sysfs method\n");
	}

	DeviceInstances = NULL;
	DeviceInstances = g_new0( LDevices, 1);
	DeviceInstances->listVidDevices = NULL;

	/* Create a list of the devices in the 'v4l2' subsystem. */
	enumerate = udev_enumerate_new(udev);
	udev_enumerate_add_match_subsystem(enumerate, "video4linux");
	udev_enumerate_scan_devices(enumerate);
	devices = udev_enumerate_get_list_entry(enumerate);
	/* For each item enumerated, print out its information.
	   udev_list_entry_foreach is a macro which expands to
	   a loop. The loop will be executed for each member in
	   devices, setting dev_list_entry to a list entry
	   which contains the device's path in /sys. */
	udev_list_entry_foreach(dev_list_entry, devices)
	{
		const char *path;

		/* Get the filename of the /sys entry for the device
		   and create a udev_device object (dev) representing it */
		path = udev_list_entry_get_name(dev_list_entry);
		dev = udev_device_new_from_syspath(udev, path);

		/* usb_device_get_devnode() returns the path to the device node
		   itself in /dev. */
		const gchar *v4l2_device = udev_device_get_devnode(dev);
		/* open the device and query the capabilities */
		//#if 0
		if ((fd = v4l2_open(v4l2_device, O_RDWR | O_NONBLOCK, 0)) < 0)
			//   		        if ((fd = open(v4l2_device, O_RDWR | O_NONBLOCK, 0)) < 0)

		{
			g_printerr("ERROR opening V4L2 interface for %s\n", v4l2_device);
			v4l2_close(fd);
			//		close(fd);
			continue; /*next dir entry*/
		}

		if (xioctl(fd, VIDIOC_QUERYCAP, &v4l2_cap) < 0)
		{
			perror("VIDIOC_QUERYCAP error");
			g_printerr("   couldn't query device %s\n", v4l2_device);
			v4l2_close(fd);
			//		close (fd);
			continue; /*next dir entry*/
		}
		v4l2_close(fd);
		//		close (fd);
		//#endif
		num_dev++;
		/* Update the device list*/
		DeviceInstances->listVidDevices = g_renew(VidDevice,
				DeviceInstances->listVidDevices,
				num_dev);
		DeviceInstances->listVidDevices[num_dev-1].device = g_strdup(v4l2_device);
		DeviceInstances->listVidDevices[num_dev-1].deviceID = atoi(DeviceInstances->listVidDevices[num_dev-1].device+10);
		DeviceInstances->listVidDevices[num_dev-1].friendlyname = g_strdup((gchar *) v4l2_cap.card);
		DeviceInstances->listVidDevices[num_dev-1].bus_info = g_strdup((gchar *) v4l2_cap.bus_info);

		/* The device pointed to by dev contains information about
		   the v4l2 device. In order to get information about the
		   USB device, get the parent device with the
		   subsystem/devtype pair of "usb"/"usb_device". This will
		   be several levels up the tree, but the function will find
		   it.*/
		dev = udev_device_get_parent_with_subsystem_devtype(
				dev,
				"usb",
				"usb_device");
		if (!dev)
		{
			cout << "Unable to find parent usb device.";
			continue;
		}

		/* From here, we can call get_sysattr_value() for each file
		   in the device's /sys entry. The strings passed into these
		   functions (idProduct, idVendor, etc.) correspond
		   directly to the files in the directory which represents
		   the USB device. Note that USB strings are Unicode, UCS2
		   encoded, but the strings returned from
		   udev_device_get_sysattr_value() are UTF-8 encoded. */

		DeviceInstances->listVidDevices[num_dev-1].vendor = g_strdup((gchar*)udev_device_get_sysattr_value(dev, "idVendor"));
		DeviceInstances->listVidDevices[num_dev-1].product =  g_strdup((gchar*)udev_device_get_sysattr_value(dev, "idProduct"));

		udev_device_unref(dev);
	}
	/* Free the enumerator object */
	udev_enumerate_unref(enumerate);

	DeviceInstances->num_devices = num_dev;
	return(num_dev);
}

