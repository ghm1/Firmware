#ifndef TARGETLANDPOSESTIMATOR_HPP
#define TARGETLANDPOSESTIMATOR_HPP

#include <mathlib/mathlib.h>
#include <vector>

#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/camera_norm_coords.h>
#include <uORB/topics/target_land_position.h>
#include <uORB/topics/target_land_offset.h>
#include <uORB/topics/home_position.h>

#include <systemlib/param/param.h>

#include <geo/geo.h>

namespace target_land_pos_estimator
{

    class TargetLandPosEstimator
    {
    public:
        struct Target
        {
            math::Vector<3> L;
            math::Vector<3> R;
            math::Vector<3> M;
            math::Vector<3> F;
            float distM;
            float distF;
            float distLR;
            bool valid;

            Target() :
                distM(-1),
                distF(-1),
                distLR(-1),
                valid(false)
            {}
        };

        /**
         * Constructor
         */
        TargetLandPosEstimator();

        /**
         * Destructor, also kills task.
         */
        ~TargetLandPosEstimator();

        /**
         * Start task.
         *
         * @return		OK on success.
         */
        int		start();

        /**
         * Test scenario
         *
         * @return		OK on success.
         */
        bool    test1();
        bool    test2();
        bool    test3();

    private:
        bool		_task_should_exit;		/**< if true, task should exit */
        int         _control_task;			/**< task handle for task */

        int		_local_pos_sub;			/**< vehicle local position */
        int     _camera_norm_coords_sub;
        int     _ctrl_state_sub;

        orb_advert_t _target_land_position_pub;
        orb_advert_t _target_land_offset_pub;

        struct vehicle_local_position_s			_local_pos;		/**< vehicle local position */
        struct camera_norm_coords_s                _camera_norm_coords;
        struct control_state_s                  _ctrl_state;
        struct target_land_position_s           _target_land_position;
        struct target_land_offset_s             _target_land_offset;

        struct Target                           _target;
        bool                                    _test1;              //function test active (only debug)
        bool                                    _test2;              //function test active (only debug)
        bool                                    _test3;              //function test active (only debug)
        std::vector<math::Vector<3>>            _rotPts;            //vector of points after rotation
        std::vector<Target>                     _targetCandidates;  //vector of target candidates
        math::Vector<3>                         _shift_xyz;
        math::Vector<3>                         _targetPosGlobal;

        struct map_projection_reference_s _ref_pos;

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

        void        calculateTargetToCameraShift();
        void        identifyTargetPoints();
        float       ptDistance(const math::Vector<3> &pt1, const math::Vector<3> &pt2);
        void        findTargetCandidate(const math::Vector<3>& L1, const math::Vector<3>& L2,
                                                  const math::Vector<3>& P1, const math::Vector<3>& P2 );

        static bool        targetCompare(const Target& lhs, const Target& rhs);

        //control::BlockParamFloat _param_target_pts_dist_l_to_r;
        param_t     _target_pts_dist_handle;
        float       _target_pts_dist;
    };


    extern TargetLandPosEstimator	*instance;
}

#endif // TARGETLANDPOSESTIMATOR_HPP
