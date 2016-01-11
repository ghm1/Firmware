#ifndef PIXY_CAM_SIM_HPP
#define PIXY_CAM_SIM_HPP

#include <uORB/topics/pixy_cam_pts.h>
#include <uORB/topics/camera_norm_coords.h>
#include <mathlib/mathlib.h>
#include <vector>

namespace pixy_cam_sim
{

    class PixyCamSim
    {
    public:
        /**
         * Constructor
         */
        PixyCamSim();

        /**
         * Destructor, also kills task.
         */
        ~PixyCamSim();

        /**
         * Start task.
         *
         * @return		OK on success.
         */
        int		start();

    private:
        bool		_task_should_exit;		/**< if true, task should exit */
        int         _control_task;			/**< task handle for task */

        int		_pixy_cam_pts_sub;
        //outgoing topic
        orb_advert_t _camera_norm_coords_pub;

        //incoming points
        struct pixy_cam_pts_s           _pixy_cam_pts_s;
        //outgoing points
        struct camera_norm_coords_s     _camera_norm_coords_s;

        /**
         * Shim for calling task_main from task_create.
         */
        static void	task_main_trampoline(int argc, char *argv[]);

        /**
         * Main sensor collection task.
         */
        void		task_main();

        /**
         * Check for changes in subscribed topics.
         */
        void		poll_subscriptions();

        /**
         * Subscribe to topics.
         */
        void        make_subscriptions();

    };


    extern PixyCamSim	*pixy_instance;
}

#endif // PIXY_CAM_SIM_HPP
