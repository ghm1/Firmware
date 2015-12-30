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

#include <drivers/boards/px4fmu-v2/board_config.h>
#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_pixy5pts.h>
#include <drivers/drv_hrt.h>

#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/camera_pixy5pts.h>

/** Configuration Constants **/
#define PIXY5PTS_I2C_BUS			PX4_I2C_BUS_EXPANSION
#define PIXY5PTS_I2C_ADDRESS		0x54 /** 7-bit address (non shifted) **/
#define PIXY5PTS_CONVERSION_INTERVAL_US	20000U /** us = 20ms = 50Hz **/

#define PIXY5PTS_SYNC			0xAA55
#define PIXY5PTS_RESYNC		0x5500
#define PIXY5PTS_ADJUST		0xAA

#define PIXY5PTS_CENTER_X				159			// the x-axis center pixel position
#define PIXY5PTS_CENTER_Y				99			// the y-axis center pixel position
#define PIXY5PTS_PIXELS_PER_RADIAN_X	307.9075f	// x-axis pixel to radian scaler assuming 60deg FOV on x-axis
#define PIXY5PTS_PIXELS_PER_RADIAN_Y	326.4713f	// y-axis pixel to radian scaler assuming 35deg FOV on y-axis

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

    orb_advert_t _camera_pixy5pts_topic;
};

/** global pointer for single PIXY5PTS sensor **/
namespace
{
PIXY5PTS *g_irlock = nullptr;
}

void irlock_usage();

extern "C" __EXPORT int pixy5pts_main(int argc, char *argv[]);

/** constructor **/
PIXY5PTS::PIXY5PTS(int bus, int address) :
    I2C("irlock", PIXY5PTS0_DEVICE_PATH, bus, address, 400000),
	_reports(nullptr),
	_sensor_ok(false),
    _read_failures(0),
    _camera_pixy5pts_topic(nullptr)
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
	if (g_irlock == nullptr) {
		errx(1, "irlock device driver is not running");
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
	if (g_irlock == nullptr) {
		errx(1, "irlock device driver is not running");
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

	/** check global irlock reference and cycle **/
	if (g_irlock != nullptr) {
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
        struct camera_pixy5pts_s report;
        //struct camera_pixy5pts_s report;
        report.timestamp = hrt_absolute_time();
        report.count = num_objects;

        //add all new objects to topic
        while (_reports->count() > 0) {
            _reports->get(&block);

            unsigned count = 0;
            //convert to ned
            float xCoord = block.angle_x;
            float yCoord = block.angle_y;

            //set to report
            report.x_coord[count] = xCoord;
            report.y_coord[count] = yCoord;

            //counter increment
            count++;
        }

        //send new report over uorb
        if (_camera_pixy5pts_topic == nullptr) {
            _camera_pixy5pts_topic = orb_advertise(ORB_ID(camera_pixy5pts), &report);

        } else {
            /* publish it */
            orb_publish(ORB_ID(camera_pixy5pts), _camera_pixy5pts_topic, &report);
        }

        //debug output
        warnx("new report with %d points, time %lld", report.count, report.timestamp);
        for(unsigned i=0; i<num_objects; ++i)
        {
            warnx("x:%4.3f y:%4.3f",
                  (double)report.x_coord[i],
                  (double)report.y_coord[i]);
        }
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

void irlock_usage()
{
	warnx("missing command: try 'start', 'stop', 'info', 'test'");
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
		irlock_usage();
		exit(1);
	}

	const char *command = argv[optind];

	/** start driver **/
	if (!strcmp(command, "start")) {
        /* test nullpointer if already initialized */
		if (g_irlock != nullptr) {
			errx(1, "driver has already been started");
		}

		/** instantiate global instance **/
        g_irlock = new PIXY5PTS(i2cdevice, PIXY5PTS_I2C_ADDRESS);

		if (g_irlock == nullptr) {
			errx(1, "failed to allocated memory for driver");
		}

		/** initialise global instance **/
		if (g_irlock->init() != OK) {
            PIXY5PTS *tmp_irlock = g_irlock;
			g_irlock = nullptr;
			delete tmp_irlock;
			errx(1, "failed to initialize device, stopping driver");
		}

		exit(0);
	}

	/** need the driver past this point **/
	if (g_irlock == nullptr) {
		warnx("not started");
		irlock_usage();
		exit(1);
	}

	/** stop the driver **/
	if (!strcmp(command, "stop")) {
        PIXY5PTS *tmp_irlock = g_irlock;
		g_irlock = nullptr;
		delete tmp_irlock;
		warnx("irlock stopped");
		exit(OK);
	}

	/** Print driver information **/
	if (!strcmp(command, "info")) {
		g_irlock->info();
		exit(OK);
	}

	/** test driver **/
	if (!strcmp(command, "test")) {
		g_irlock->test();
		exit(OK);
	}

	/** display usage info **/
	irlock_usage();
	exit(0);
}
