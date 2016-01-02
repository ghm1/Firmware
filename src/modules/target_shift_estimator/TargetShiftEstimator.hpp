#ifndef TARGETSHIFTESTIMATOR_HPP
#define TARGETSHIFTESTIMATOR_HPP

namespace shift_estimator
{

    class TargetShiftEstimator
    {
    public:
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

    private:
        bool		_task_should_exit;		/**< if true, task should exit */
        int         _control_task;			/**< task handle for task */

        /**
         * Shim for calling task_main from task_create.
         */
        static void	task_main_trampoline(int argc, char *argv[]);

        /**
         * Main sensor collection task.
         */
        void		task_main();
    };


    extern TargetShiftEstimator	*instance;
}

#endif // TARGETSHIFTESTIMATOR_HPP
