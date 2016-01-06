#ifndef TARGETSHIFTESTIMATOR_HPP
#define TARGETSHIFTESTIMATOR_HPP

#include <mathlib/mathlib.h>
#include <vector>

#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/camera_pixy5pts.h>

namespace shift_estimator
{

    class TargetShiftEstimator
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
        TargetShiftEstimator();

        /**
         * Destructor, also kills task.
         */
        ~TargetShiftEstimator();

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
        bool    test();


    private:
        bool		_task_should_exit;		/**< if true, task should exit */
        int         _control_task;			/**< task handle for task */

        int		_local_pos_sub;			/**< vehicle local position */
        int     _pixy5pts_sub;
        int     _ctrl_state_sub;

        struct vehicle_local_position_s			_local_pos;		/**< vehicle local position */
        struct camera_pixy5pts_s                _pixy5pts;
        struct control_state_s                  _ctrl_state;

        struct Target                           _target;
        bool                                    _test;              //function test active
        std::vector<math::Vector<3>>            _rotPts;            //vector of points after rotation
        std::vector<Target>                     _targetCandidates;  //vector of target candidates
        math::Vector<3>                         _shift_xyz;

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

    };


    extern TargetShiftEstimator	*instance;
}

#endif // TARGETSHIFTESTIMATOR_HPP
