// Based on work by Franz Pucher, Diffbot, (2020), GitHub repository, https://github.com/fjp/diffbot
#include "hoverboard_driver/pid.hpp"

PID::PID(double p, double i, double d, double i_max, double i_min, bool antiwindup, double out_max, double out_min)
    : control_toolbox::Pid()
{
    f_ = 0.0;
    initPid(p, i, d, i_max, i_min, antiwindup);
    error_ = 0.0;

    out_min_ = out_min;
    out_max_ = out_max;
}

void PID::init(double f, double p, double i, double d, double i_max, double i_min, bool antiwindup, double out_max, double out_min)
{
    RCLCPP_INFO(rclcpp::get_logger("hoverboard_driver"), "Initialize PID");
    f_ = f;
    initPid(p, i, d, i_max, i_min, antiwindup);
    error_ = 0.0;

    out_min_ = out_min;
    out_max_ = out_max;

    Gains gains = getGains();
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("hoverboard_driver"), "Initialized PID: F=" << f << ", P=" << gains.p_gain_ << ", I=" << gains.i_gain_ << ", D=" << gains.d_gain_ << ", out_min=" << out_min_ << ", out_max=" << out_max_);
}

double PID::operator()(const double &measured_value, const double &setpoint, const rclcpp::Duration &dt)
{
    // Compute error terms
    error_ = setpoint - measured_value;
    // rclcpp::Clock clock; // NOTA: Puede mamar
    // RCLCPP_DEBUG_STREAM_THROTTLE(
    //     node_->get_logger(), clock, 1000000000, "Error: " << error_);

    // Reset the i_error in case the p_error and the setpoint is zero
    // Otherwise there will always be a constant i_error_ that won't vanish
    if (0.0 == setpoint && 0.0 == error_)
    {
	// reset() will reset
	// p_error_last_ = 0.0;
	// p_error_ = 0.0;
	// i_error_ = 0.0;
	// d_error_ = 0.0;
	// cmd_ = 0.0;
	reset();
    }

    // Use control_toolbox::Pid::computeCommand()
    double output = computeCommand(error_, dt.nanoseconds());
    // RCLCPP_DEBUG_STREAM_THROTTLE(node_->get_logger(), clock, dt.nanoseconds(),
    //     "PID computed command: " << output);

    // Compute final output including feed forward term
    output = f_ * setpoint + output;
    //output = clamp(output, out_min_, out_max_);

    return output;
}

void PID::getParameters(double &f, double &p, double &i, double &d, double &i_max, double &i_min)
{
    bool antiwindup;
    getParameters(f, p, i, d, i_max, i_min, antiwindup);
}

void PID::getParameters(double &f, double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
{
    f = f_;
    // Call getGains from control_toolbox
    getGains(p, i, d, i_max, i_min, antiwindup);
}

void PID::setParameters(double f, double p, double i, double d, double i_max, double i_min, bool antiwindup)
{
    f_ = f;
    setGains(p, i, d, i_max, i_min, antiwindup);

    Gains gains = getGains();
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("hoverboard_driver"), "Update PID Gains: F=" << f << ", P=" << gains.p_gain_ << ", I=" << gains.i_gain_ << ", D=" << gains.d_gain_ << ", out_min=" << out_min_ << ", out_max=" << out_max_);
}

void PID::setOutputLimits(double output_max, double output_min)
{
    out_max_ = output_max;
    out_min_ = output_min;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("hoverboard_driver"), "Update PID output limits: lower=" << out_min_ << ", upper=" << out_max_);
}


double PID::clamp(const double& value, const double& lower_limit, const double& upper_limit)
{
    rclcpp::Clock clock;
    if (value > upper_limit){
        RCLCPP_DEBUG_STREAM_THROTTLE(
            rclcpp::get_logger("hoverboard_driver"), clock, 1000000000,
            "Clamp " << value << " to upper limit " << upper_limit);
        return upper_limit;
    }else if (value < lower_limit){
        RCLCPP_DEBUG_STREAM_THROTTLE(
            rclcpp::get_logger("hoverboard_driver"), clock, 1000000000,
            "Clamp " << value << " to lower limit " << upper_limit);
        return lower_limit;
    }
    return value;
}