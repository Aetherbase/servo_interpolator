#pragma once
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"

template <typename angle_t,typename pwm_t>
struct ServoInterpolatorConf{
    using conf_angle_t = angle_t;
    using conf_pwm_t = pwm_t;
    pwm_t pwmExtremeRight,pwmExtremeLeft,pwmCenter;
    angle_t angleExtremeRight,angleExtremeLeft,angleCenter;
};
const std::string pwm_param_extreme_left = "extreme_left";
const std::string pwm_param_extreme_right = "extreme_right";
const std::string pwm_param_center = "center";

template <typename ServoInterpolatorConf_t,typename coeff_t = double>
struct ServoInterpolator{
    using angle_t = typename ServoInterpolatorConf_t::conf_angle_t;
    using pwm_t = typename ServoInterpolatorConf_t::conf_pwm_t;
    enum class Pos{
        EXTREME_RIGHT=0,
        EXTREME_LEFT=1,
        CENTER=2
    };
    
    inline ServoInterpolator( const ServoInterpolatorConf_t& _conf) :
    conf(_conf)
    {
        updateCoeffs(); 
    };
    template<Pos pos>
    inline void setAngleAtPos(angle_t angle){
        angle_t* const angles = &conf.angleExtremeRight;
        angles[static_cast<uint32_t>(pos)]=angle;
        updateCoeffs();
    }
    template<Pos pos>
    inline void getAngleAtPos(angle_t& angle) const{
        const angle_t* const angles = &conf.angleExtremeRight;
        angle=angles[static_cast<uint32_t>(pos)];
    }
    inline void setAngles(angle_t max, angle_t min , angle_t center){
        conf.angleExtremeRight = max;
        conf.angleExtremeLeft = min;
        conf.angleCenter = center;
        updateCoeffs();
    }
    inline void getAngles(angle_t& max, angle_t& min , angle_t& center) const{
        max = conf.angleExtremeRight;
        min = conf.angleExtremeLeft;
        center = conf.angleCenter;
    }
    inline void getPwms(pwm_t& max, pwm_t& min , pwm_t& center) const{
        max = conf.pwmExtremeRight;
        min = conf.pwmExtremeLeft;
        center = conf.pwmCenter;
    }
    inline void setPwms(pwm_t max, pwm_t min , pwm_t center){
        conf.pwmExtremeRight = max;
        conf.pwmExtremeLeft = min;
        conf.pwmCenter = center;
        updateCoeffs();
    }
    template<Pos pos>
    inline void setPwmAtPos(pwm_t pwm){
        angle_t* const pwms = &conf.pwmExtremeRight;
        pwms[static_cast<uint32_t>(pos)]=pwm;
        updateCoeffs();
    }
    template<Pos pos>
    inline void getPwmAtPos(pwm_t& pwm) const{
        const pwm_t* const pwms = &conf.pwmExtremeRight;
        pwm=pwms[static_cast<uint32_t>(pos)];
    }
    inline void interpolate(angle_t angle, pwm_t& pwm) const{
        pwm = coeff_x2*angle*angle + coeff_x*angle + coeff_c;
    }
    private:
    inline void updateCoeffs(){
        angle_t ld((conf.angleExtremeLeft-conf.angleExtremeRight)*(conf.angleExtremeLeft-conf.angleCenter));
        angle_t rd((conf.angleExtremeRight-conf.angleExtremeLeft)*(conf.angleExtremeRight-conf.angleCenter));
        angle_t cd((conf.angleCenter-conf.angleExtremeRight)*(conf.angleCenter-conf.angleExtremeLeft));
        coeff_x2=(conf.pwmExtremeRight/rd) + (conf.pwmExtremeLeft/ld) + (conf.pwmCenter/cd);
        coeff_x=-(((conf.pwmCenter+conf.pwmExtremeLeft)/rd) + ((conf.pwmExtremeRight+conf.pwmExtremeLeft)/cd) + \
        ((conf.pwmExtremeRight+conf.pwmCenter)/ld));
        coeff_c=((conf.pwmCenter*conf.pwmExtremeLeft)/rd) + ((conf.pwmExtremeRight*conf.pwmExtremeLeft)/cd) + \
        ((conf.pwmExtremeRight*conf.pwmCenter)/ld);
    }
    ServoInterpolatorConf_t conf;
    coeff_t coeff_x2,coeff_x,coeff_c;
};

struct ServoInterpolatorRosConf
{
    const char * pwm_topic, *angle_topic;
    uint32_t pwm_qs = 100, angle_qs = 100;
    const char* pwm_param;
    const char* stop_updates_param;

};

template <typename PwmMsg_t, typename AngleMsg_t>
struct ServoInterpolatorRos
{
    using _Type = ServoInterpolatorRos< PwmMsg_t, AngleMsg_t>;
    using ServoInterpolatorConf_t =  ServoInterpolatorConf< typename AngleMsg_t::_data_type,typename PwmMsg_t::_data_type>;
    using AngleCallbackType = const typename AngleMsg_t::ConstPtr;

    ServoInterpolatorRos(ros::NodeHandle &_nh, ServoInterpolatorRosConf& conf,\
    ServoInterpolatorConf_t& interpolator_conf) :\
    nh(_nh),interpolator(interpolator_conf),enable_updates(true),
    pwm_param(conf.pwm_param),stop_updates_param(conf.stop_updates_param)
    {
        pwm_pub = _nh.advertise<PwmMsg_t>(conf.pwm_topic,conf.pwm_qs);
        angle_sub = _nh.subscribe(conf.angle_topic,conf.angle_qs,&_Type::interpolatorCallback,this);
    }
    private:
    void updateInterpolator(){
        if(enable_updates){
            std::map<std::string,int> param_dict;
            if(nh.getParam(pwm_param,param_dict))
                interpolator.setPwms(param_dict[pwm_param_extreme_right],param_dict[pwm_param_extreme_left],param_dict[pwm_param_center]);
            bool stop_updates;
            if(nh.getParam(stop_updates_param,stop_updates))
                enable_updates = !stop_updates;
        }
    }
    void interpolatorCallback(AngleCallbackType& angle_msg){
        updateInterpolator();
        interpolator.interpolate(angle_msg.get()->data,pwm_msg.data);
    }
    ros::NodeHandle& nh;
    PwmMsg_t pwm_msg;
    ros::Subscriber angle_sub;
    ros::Publisher pwm_pub;
    ServoInterpolator<ServoInterpolatorConf_t,double> interpolator;
    const std::string pwm_param,stop_updates_param;
    bool enable_updates;
};