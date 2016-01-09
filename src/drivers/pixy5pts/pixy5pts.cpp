/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file pixy5Pts.cpp
 * @author Michael Göttlicher
 *
 * Driver for an IR-Lock and Pixy vision sensor connected via I2C. Up to five
 * points will be advertised over uorb
 *
 * Created on: Dez 12, 2015
 **/

#include <fcntl.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <math.h>

#include <drivers/boards/px4fmu-v2/board_config.h>
#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_pixy5pts.h>
#include <drivers/drv_hrt.h>

#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/camera_norm_coords.h>

/** Configuration Constants **/
#define PIXY5PTS_I2C_BUS			PX4_I2C_BUS_EXPANSION
#define PIXY5PTS_I2C_ADDRESS		0x54 /** 7-bit address (non shifted) **/
#define PIXY5PTS_CONVERSION_INTERVAL_US	20000U /** us = 20ms = 50Hz **/

#define PIXY5PTS_SYNC			0xAA55
#define PIXY5PTS_RESYNC		0x5500
#define PIXY5PTS_ADJUST		0xAA

#define PIXY5PTS_CENTER_X				160.44f			// the x-axis center pixel position
#define PIXY5PTS_CENTER_Y				100.75f		// the y-axis center pixel position
#define PIXY5PTS_FOCAL_X                323.70584f       // focal length in x-direction
#define PIXY5PTS_FOCAL_Y                324.26201f       // focal length in y-direction
#define PIXY5PTS_P1                     0.5084f          // coefficient for undistortion for 3rd degree
#define PIXY5PTS_P3                     0.9970f          // coefficient for undistortion for 1st degree

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class PIXY5PTS : public device::I2C
{
public:
    PIXY5PTS(int bus = PIXY5PTS_I2C_BUS, int address = PIXY5PTS_I2C_ADDRESS);
    virtual ~PIXY5PTS();

	virtual int init();
	virtual int probe();
	virtual int info();
	virtual int test();
    virtual int test_pts();

	virtual ssize_t read(struct file *filp, char *buffer, size_t buflen);

private:

	/** start periodic reads from sensor **/
	void 		start();

	/** stop periodic reads from sensor **/
	void 		stop();

	/** static function that is called by worker queue, arg will be pointer to instance of this class **/
	static void	cycle_trampoline(void *arg);

	/** read from device and schedule next read **/
	void		cycle();

	/** low level communication with sensor **/
	int 		read_device();
	bool 		sync_device();
	int 		read_device_word(uint16_t *word);
    int 		read_device_block(struct pixy5pts_s *block);

	/** internal variables **/
	ringbuffer::RingBuffer *_reports;
	bool _sensor_ok;
	work_s _work;
	uint32_t _read_failures;

    orb_advert_t _camera_norm_coords_topic;
};

/** global pointer for single PIXY5PTS sensor **/
namespace
{
PIXY5PTS *g_pixy5pts = nullptr;
}

void pixy5pts_usage();

extern "C" __EXPORT int pixy5pts_main(int argc, char *argv[]);

/** constructor **/
PIXY5PTS::PIXY5PTS(int bus, int address) :
    I2C("pixy5pts", PIXY5PTS0_DEVICE_PATH, bus, address, 400000),
	_reports(nullptr),
	_sensor_ok(false),
    _read_failures(0),
    _camera_norm_coords_topic(nullptr)
{
	memset(&_work, 0, sizeof(_work));
}

/** destructor **/
PIXY5PTS::~PIXY5PTS()
{
	stop();

	/** clear reports queue **/
	if (_reports != nullptr) {
		delete _reports;
	}
}

/** initialise driver to communicate with sensor **/
int PIXY5PTS::init()
{
	/** initialise I2C bus **/
	int ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	/** allocate buffer storing values read from sensor **/
    _reports = new ringbuffer::RingBuffer(PIXY5PTS_OBJECTS_MAX, sizeof(struct pixy5pts_s));

	if (_reports == nullptr) {
		return ENOTTY;

    } else {
        _sensor_ok = true;
        /** start work queue **/
        start();
        return OK;
    }
}

/** probe the device is on the I2C bus **/
int PIXY5PTS::probe()
{
	/*
     * PIXY5PTS defaults to sending 0x00 when there is no block
	 * data to return, so really all we can do is check to make
	 * sure a transfer completes successfully.
	 **/
	uint8_t byte;

	if (transfer(nullptr, 0, &byte, 1) != OK) {
		return -EIO;
	}

	return OK;
}

/** display driver info **/
int PIXY5PTS::info()
{
    if (g_pixy5pts == nullptr) {
        errx(1, "pixy5pts device driver is not running");
	}

	/** display reports in queue **/
	if (_sensor_ok) {
		_reports->print_info("report queue: ");
		warnx("read errors:%lu", (unsigned long)_read_failures);

	} else {
		warnx("sensor is not healthy");
	}

	return OK;
}

/** test driver **/
int PIXY5PTS::test()
{
	/** exit immediately if driver not running **/
    if (g_pixy5pts == nullptr) {
        errx(1, "pixy5pts device driver is not running");
	}

	/** exit immediately if sensor is not healty **/
	if (!_sensor_ok) {
		errx(1, "sensor is not healthy");
	}

	/** instructions to user **/
	warnx("searching for object for 10 seconds");

	/** read from sensor for 10 seconds **/
    struct pixy5pts_s obj_report;
	uint64_t start_time = hrt_absolute_time();

	while ((hrt_absolute_time() - start_time) < 10000000) {

		/** output all objects found **/
		while (_reports->count() > 0) {
			_reports->get(&obj_report);
			warnx("sig:%d x:%4.3f y:%4.3f width:%4.3f height:%4.3f",
			      (int)obj_report.target_num,
			      (double)obj_report.angle_x,
			      (double)obj_report.angle_y,
			      (double)obj_report.size_x,
			      (double)obj_report.size_y);
		}

		/** sleep for 0.05 seconds **/
		usleep(50000);
	}

	return OK;
}

int PIXY5PTS::test_pts()
{
    /** exit immediately if driver not running **/
    if (g_pixy5pts == nullptr) {
        errx(1, "pixy5pts device driver is not running");
    }

    /** exit immediately if sensor is not healty **/
    if (!_sensor_ok) {
        errx(1, "sensor is not healthy");
    }

    /** instructions to user **/
    warnx("sending test points");

    struct camera_norm_coords_s report;
    report.timestamp = hrt_absolute_time();
    report.count = 4;
    report.x_coord[0] = 107.2309;
    report.y_coord[0] = 66.4704;
    report.x_coord[1] = 168.1143;
    report.y_coord[1] = 65.2582;
    report.x_coord[2] = 105.6128;
    report.y_coord[2] = 127.8905;
    report.x_coord[3] = 53.3100;
    report.y_coord[3] = 68.4937;

    //add all new objects to topic
    for( unsigned i=0; i < 4; ++i )
    {
        //convert to ned
        float x_norm = (report.x_coord[i] - PIXY5PTS_CENTER_X) / PIXY5PTS_FOCAL_X;
        float y_norm = (report.y_coord[i] - PIXY5PTS_CENTER_Y) / PIXY5PTS_FOCAL_Y;
        //conversion to polar coordinates
        float phi = atan2f(y_norm, x_norm);
        float r_dist_sq = x_norm * x_norm + y_norm * y_norm;
        float r_dist = sqrtf(r_dist_sq);
        float r_undist = PIXY5PTS_P1 * r_dist_sq * r_dist + PIXY5PTS_P3 * r_dist;

        //set to report
        report.x_coord[i] = r_undist * cosf(phi);
        report.y_coord[i] = r_undist * sinf(phi);
    }

    //send new report over uorb
    if (_camera_norm_coords_topic == nullptr) {
        _camera_norm_coords_topic = orb_advertise(ORB_ID(camera_norm_coords), &report);

    } else {
        /* publish it */
        orb_publish(ORB_ID(camera_norm_coords), _camera_norm_coords_topic, &report);
    }


    return OK;
}

/** start periodic reads from sensor **/
void PIXY5PTS::start()
{
	/** flush ring and reset state machine **/
	_reports->flush();

	/** start work queue cycle **/
    work_queue(HPWORK, &_work, (worker_t)&PIXY5PTS::cycle_trampoline, this, 1);
}

/** stop periodic reads from sensor **/
void PIXY5PTS::stop()
{
	work_cancel(HPWORK, &_work);
}

void PIXY5PTS::cycle_trampoline(void *arg)
{
    PIXY5PTS *device = (PIXY5PTS *)arg;

    /** check global pixy5pts reference and cycle **/
    if (g_pixy5pts != nullptr) {
		device->cycle();
	}
}

void PIXY5PTS::cycle()
{
	/** ignoring failure, if we do, we will be back again right away... **/
	read_device();

	/** schedule the next cycle **/
    work_queue(HPWORK, &_work, (worker_t)&PIXY5PTS::cycle_trampoline, this, USEC2TICK(PIXY5PTS_CONVERSION_INTERVAL_US));
}

//this method is only called from ardupilot application design
ssize_t PIXY5PTS::read(struct file *filp, char *buffer, size_t buflen)
{
    unsigned count = buflen / sizeof(struct pixy5pts_s);
    struct pixy5pts_s *rbuf = reinterpret_cast<struct pixy5pts_s *>(buffer);
	int ret = 0;

	if (count < 1) {
		return -ENOSPC;
	}

	/** try to read **/
	while (count--) {
		if (_reports->get(rbuf)) {
			ret += sizeof(*rbuf);
			++rbuf;
		}
	}

	return ret ? ret : -EAGAIN;

	return ret;
}

/** sync device to ensure reading starts at new frame*/
bool PIXY5PTS::sync_device()
{
	uint8_t sync_byte;
	uint16_t sync_word;

	if (read_device_word(&sync_word) != OK) {
		return false;
	}

    if (sync_word == PIXY5PTS_RESYNC) {
		transfer(nullptr, 0, &sync_byte, 1);

        if (sync_byte == PIXY5PTS_ADJUST) {
			return true;
		}

    } else if (sync_word == PIXY5PTS_SYNC) {
		return true;
	}

	return false;
}

/** read all available frames from sensor **/
int PIXY5PTS::read_device()
{
	/** if we sync, then we are starting a new frame, else fail **/
	if (!sync_device()) {
		return -ENOTTY;
	}

	/** now read blocks until sync stops, first flush stale queue data **/
	_reports->flush();
	int num_objects = 0;

    //get all new objects from pixy
    //todo: what about sending 5 points from pixy in one frame
    struct pixy5pts_s block;
    while (sync_device() && (num_objects < PIXY5PTS_OBJECTS_MAX)) {

		if (read_device_block(&block) != OK) {
			break;
		}

		_reports->force(&block);
        num_objects++;
	}

    //iterate over new buffer objects if there are any
    if(num_objects > 0)
    {
        struct camera_norm_coords_s report;
        //struct camera_norm_coords_s report;
        report.timestamp = hrt_absolute_time();
        report.count = num_objects;

//        //debug output
//        warnx("new block with %d points, time %lld", _reports->count(), report.timestamp);

        unsigned count = 0;
        //add all new objects to topic
        while (_reports->count() > 0) {
            _reports->get(&block);

//            warnx("x:%4.3f y:%4.3f",
//                  (double)block.angle_x,
//                  (double)block.angle_y);

            //convert to ned
            float x_norm = (block.angle_x - PIXY5PTS_CENTER_X) / PIXY5PTS_FOCAL_X;
            float y_norm = (block.angle_y - PIXY5PTS_CENTER_Y) / PIXY5PTS_FOCAL_Y;
            //conversion to polar coordinates
            float phi = atan2f(y_norm, x_norm);
            float r_dist_sq = x_norm * x_norm + y_norm * y_norm;
            float r_dist = sqrtf(r_dist_sq);
            float r_undist = PIXY5PTS_P1 * r_dist_sq * r_dist + PIXY5PTS_P3 * r_dist;

            //set to report
            float x = r_undist * cosf(phi);
            float y = r_undist * sinf(phi);

            //rotate camera about -90 degree around z (x becomes -y and y becomes x)
            //x becomes -y
            report.x_coord[count] = -y;
            //y becomes x
            report.y_coord[count] = x;

            //counter increment
            count++;
        }

        //send new report over uorb
        if (_camera_norm_coords_topic == nullptr) {
            _camera_norm_coords_topic = orb_advertise(ORB_ID(camera_norm_coords), &report);

        } else {
            /* publish it */
            orb_publish(ORB_ID(camera_norm_coords), _camera_norm_coords_topic, &report);
        }

//        //debug output
//        warnx("new report with %d points, time %lld", report.count, report.timestamp);
//        for(unsigned i=0; i<num_objects; ++i)
//        {
//            warnx("x:%4.3f y:%4.3f",
//                  (double)report.x_coord[i],
//                  (double)report.y_coord[i]);
//        }
    }

    return OK;
}

/** read a word (two bytes) from sensor **/
int PIXY5PTS::read_device_word(uint16_t *word)
{
	uint8_t bytes[2];
	memset(bytes, 0, sizeof bytes);

	int status = transfer(nullptr, 0, &bytes[0], 2);
	*word = bytes[1] << 8 | bytes[0];

	return status;
}

/** read a single block (a full frame) from sensor **/
int PIXY5PTS::read_device_block(struct pixy5pts_s *block)
{
	uint8_t bytes[12];
	memset(bytes, 0, sizeof bytes);

	int status = transfer(nullptr, 0, &bytes[0], 12);
	uint16_t checksum = bytes[1] << 8 | bytes[0];
	uint16_t target_num = bytes[3] << 8 | bytes[2];
	uint16_t pixel_x = bytes[5] << 8 | bytes[4];
	uint16_t pixel_y = bytes[7] << 8 | bytes[6];
	uint16_t pixel_size_x = bytes[9] << 8 | bytes[8];
	uint16_t pixel_size_y = bytes[11] << 8 | bytes[10];

	/** crc check **/
	if (target_num + pixel_x + pixel_y + pixel_size_x + pixel_size_y != checksum) {
		_read_failures++;
		return -EIO;
	}

//	/** convert to angles **/
//	block->target_num = target_num;
//    block->angle_x = (((float)(pixel_x - PIXY5PTS_CENTER_X)) / PIXY5PTS_PIXELS_PER_RADIAN_X);
//    block->angle_y = (((float)(pixel_y - PIXY5PTS_CENTER_Y)) / PIXY5PTS_PIXELS_PER_RADIAN_Y);
//    block->size_x = pixel_size_x / PIXY5PTS_PIXELS_PER_RADIAN_X;
//    block->size_y = pixel_size_y / PIXY5PTS_PIXELS_PER_RADIAN_Y;

//	block->timestamp = hrt_absolute_time();

    block->target_num = target_num;
    block->angle_x = (float)pixel_x;
    block->angle_y = (float)pixel_y;
    block->size_x = (float)pixel_size_x;
    block->size_y = (float)pixel_size_y;
    block->timestamp = hrt_absolute_time();

	return status;
}

void pixy5pts_usage()
{
    warnx("missing command: try 'start', 'stop', 'info', 'test', 'testpts' ");
	warnx("options:");
    warnx("    -b i2cbus (%d)", PIXY5PTS_I2C_BUS);
}

int pixy5pts_main(int argc, char *argv[])
{
    int i2cdevice = PIXY5PTS_I2C_BUS;

    /* jump over start/off/etc and look at options first **/
	if (getopt(argc, argv, "b:") != EOF) {
		i2cdevice = (int)strtol(optarg, NULL, 0);
	}

	if (optind >= argc) {
        pixy5pts_usage();
		exit(1);
	}

	const char *command = argv[optind];

	/** start driver **/
	if (!strcmp(command, "start")) {
        /* test nullpointer if already initialized */
        if (g_pixy5pts != nullptr) {
			errx(1, "driver has already been started");
		}

		/** instantiate global instance **/
        g_pixy5pts = new PIXY5PTS(i2cdevice, PIXY5PTS_I2C_ADDRESS);

        if (g_pixy5pts == nullptr) {
			errx(1, "failed to allocated memory for driver");
		}

		/** initialise global instance **/
        if (g_pixy5pts->init() != OK) {
            PIXY5PTS *tmp_pixy5pts = g_pixy5pts;
            g_pixy5pts = nullptr;
            delete tmp_pixy5pts;
			errx(1, "failed to initialize device, stopping driver");
		}

		exit(0);
	}

	/** need the driver past this point **/
    if (g_pixy5pts == nullptr) {
		warnx("not started");
        pixy5pts_usage();
		exit(1);
	}

	/** stop the driver **/
	if (!strcmp(command, "stop")) {
        PIXY5PTS *tmp_pixy5pts = g_pixy5pts;
        g_pixy5pts = nullptr;
        delete tmp_pixy5pts;
        warnx("pixy5pts stopped");
		exit(OK);
	}

	/** Print driver information **/
	if (!strcmp(command, "info")) {
        g_pixy5pts->info();
		exit(OK);
	}

	/** test driver **/
	if (!strcmp(command, "test")) {
        g_pixy5pts->test();
		exit(OK);
	}

    /** send test points **/
    if (!strcmp(command, "testpts")) {
        g_pixy5pts->test_pts();
        exit(OK);
    }

	/** display usage info **/
    pixy5pts_usage();
	exit(0);
}
