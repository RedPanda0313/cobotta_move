#ifndef KDLCHAINIKSOLVERPOS_MY_HPP
#define KDLCHAINIKSOLVERPOS_MY_HPP
#include "kdl/chainiksolver.hpp"
#include "kdl/chainfksolver.hpp"
#include "std_msgs/Float64MultiArray.h"
namespace KDL {
        class ChainIkSolverPos_MY : public ChainIkSolverPos
    {
        public:
            static const int E_IKSOLVER_FAILED = -100;
            ChainIkSolverPos_MY(const Chain &chain, ChainFkSolverPos &fksolver, ChainIkSolverVel &iksolver, unsigned int maxiter = 100, double eps = 1e-6);
            ~ChainIkSolverPos_MY();
            virtual int CartToJnt(const JntArray &q_init, const Frame &p_in, JntArray &q_out);
            virtual const char *strError(const int error) const;
            bool converged();
            void set_limit(std_msgs::Float64MultiArray& min,std_msgs::Float64MultiArray& max);;

        private:
            const Chain chain;
            ChainIkSolverVel &iksolver;
            ChainFkSolverPos &fksolver;
            JntArray delta_q;
            JntArray q_min;
            JntArray q_max;
            JntArray q_over;
            JntArray q_under;
            Frame f;
            Twist delta_twist;
            unsigned int maxiter;
            double eps;
            bool has_limit = false;
            bool limit_over = false;
               };
}
#endif