#include "kdl/chainiksolverpos_my.hpp"
#include <iostream>
namespace KDL
{

    ChainIkSolverPos_MY::ChainIkSolverPos_MY(const Chain &_chain, ChainFkSolverPos &_fksolver, ChainIkSolverVel &_iksolver,
                                             unsigned int _maxiter, double _eps) : chain(_chain), iksolver(_iksolver), fksolver(_fksolver), delta_q(_chain.getNrOfJoints()),
                                                                                   maxiter(_maxiter), eps(_eps)
    {
    }
    int ChainIkSolverPos_MY::CartToJnt(const JntArray &q_init, const Frame &p_in, JntArray &q_out)
    {

        has_limit = false;
        q_out = q_init;
        unsigned int i = 0;
        fksolver.JntToCart(q_out, f);
        delta_twist = diff(f, p_in);
        int rc = iksolver.CartToJnt(q_out, delta_twist, delta_q);
         //std::cout <<"twist "<< delta_twist.rot.data[0] << " " << delta_twist.rot.data[1] << " " << delta_twist.rot.data[2] << std::endl;
        /*if (Equal(delta_twist, Twist::Zero(), eps)){
            // no change
            return (i);
        }*/
        // std::cout << delta_q.data[0] << ","<<delta_q.data[1] <<","<<delta_q.data[2] <<","<<delta_q.data[3] <<","<<delta_q.data[4] << ","<<delta_q.data[5] <<std::endl;

        int size = delta_q.data.size();
        if (has_limit)
        {
            for (int j = 0; j < size; j++)
            {
                q_over.data[j] = q_max.data[j] - q_out.data[j];
                q_under.data[j] = q_out.data[j] - q_min.data[j];
                if (q_over.data[j] < 0)
                {
                    limit_over = true;
                    q_out.data[j] = q_max.data[j];
                }
                if (q_under.data[j] < 0)
                {
                    limit_over = true;
                    q_out.data[j] = q_min.data[j];
                }
            }
        }
        /*if (!limit_over && Equal(delta_twist, Twist::Zero(), eps))
        {
            // no change
            return (i);
        }*/
        if(!limit_over&&converged()){
        return (i);

        } 
        for (i = 1; i < maxiter; i++)
        {

            // we chose to continue if the child solver returned a positive
            // "error", which may simply indicate a degraded solution
            /*if (Equal(delta_twist, Twist::Zero(), eps)){
                // converged, but possibly with a degraded solution
                return (rc > E_NOERROR ? E_DEGRADED : i);
                }*/
            limit_over = false;
            if (has_limit)
            {
                size = delta_q.data.size();
                for (int j = 0; j < size; j++)
                {
                    q_over.data[j] = q_max.data[j] - q_out.data[j];
                    q_under.data[j] = q_out.data[j] - q_min.data[j];
                    if (q_over.data[j] < 0)
                    {
                        limit_over = true;
                        q_out.data[j] = q_max.data[j];
                    }
                    if (q_under.data[j] < 0)
                    {
                        limit_over = true;
                        q_out.data[j] = q_min.data[j];
                    }
                }
            }
            fksolver.JntToCart(q_out, f);
            delta_twist = diff(f, p_in);
            rc = iksolver.CartToJnt(q_out, delta_twist, delta_q);
            if (E_NOERROR > rc)
            {
                return (error = E_IKSOLVER_FAILED);
            }
            if(converged()){
            //std::cout <<i<< "here" << std::endl;
                return (limit_over ? i + 10000 : i);
            }
            /*if (Equal(delta_twist, Twist::Zero(), eps) && !limit_over)
            {
                // converged, but possibly with a degraded solution
                int ret = (limit_over ? i + 10000 : i);
                //std::cout << ret << std::endl;
                return (rc > E_NOERROR ? E_DEGRADED : ret);
            }*/
            // limit_over = false;
            //  Multiply(delta_q, 0.05, delta_q);
//            std::cout << delta_q.data[0] << ","<<delta_q.data[1] <<","<<delta_q.data[2] <<","<<delta_q.data[3] <<","<<delta_q.data[4] << ","<<delta_q.data[5] <<std::endl;

            Multiply(delta_q, 1.0, delta_q);
            //Multiply(delta_q, (maxiter-i+1)/maxiter, delta_q);
            Add(delta_q, q_out, q_out);
        }
        if (has_limit)
        {
            size = delta_q.data.size();
            for (int j = 0; j < size; j++)
            {
                q_over.data[j] = q_max.data[j] - q_out.data[j];
                q_under.data[j] = q_out.data[j] - q_min.data[j];
                if (q_over.data[j] < 0)
                {
                    limit_over = true;
                    q_out.data[j] = q_max.data[j];
                }
                if (q_under.data[j] < 0)
                {
                    limit_over = true;
                    q_out.data[j] = q_min.data[j];
                }
            }
        }
        fksolver.JntToCart(q_out, f);
        delta_twist = diff(f, p_in);
        rc = iksolver.CartToJnt(q_out, delta_twist, delta_q);
        if (E_NOERROR > rc)
        {
            return (error = E_IKSOLVER_FAILED);
        }
        if(converged()){
            return (limit_over ? i + 10000 : i);
        }
        /*if (Equal(delta_twist, Twist::Zero(), eps))
        {
            // converged, but possibly with a degraded solution
            int ret = (limit_over ? i + 10000 : i);
            std::cout << ret << std::endl;
            return (rc > E_NOERROR ? E_DEGRADED : ret);
        }*/
        // std::cout << delta_q.data[0] << ","<<delta_q.data[1] <<","<<delta_q.data[2] <<","<<delta_q.data[3] <<","<<delta_q.data[4] << ","<<delta_q.data[5] <<std::endl;

        return (error = E_NO_CONVERGE); // failed to converge
    }
    ChainIkSolverPos_MY::~ChainIkSolverPos_MY()
    {
    }
    const char *ChainIkSolverPos_MY::strError(const int error) const
    {
        if (E_IKSOLVER_FAILED == error)
            return "Child IK solver failed";
        else
            return SolverI::strError(error);
    }
    bool ChainIkSolverPos_MY::converged()
    {
        int size = delta_q.data.size();
        double sum = 0;
        for (int i = 0; i < size; i++)
        {
            sum += sqrt(delta_q.data[i] * delta_q.data[i]);
        }
        sum /= size;
        return (sum < eps ? true : false);
    }
    void ChainIkSolverPos_MY::set_limit(std_msgs::Float64MultiArray &min, std_msgs::Float64MultiArray &max)
    {
        int size = min.data.size();
        q_max.resize(size);
        q_min.resize(size);
        q_over.resize(size);
        q_under.resize(size);
        has_limit = true;
        for (int i = 0; i < size; i++)
        {
            q_min.data[i] = min.data[i];
            q_max.data[i] = max.data[i];
            // std::cout << q_min.data[i] << " " << q_max.data[i] << std::endl;
        }
    }
}