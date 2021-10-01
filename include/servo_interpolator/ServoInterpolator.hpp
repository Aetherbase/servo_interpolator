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
    using typename angle_t = ServoInterpolatorConf_t::conf_angle_t;
    using typename pwm_t = ServoInterpolatorConf_t::conf_pwm_t;
    enum class Pos{
        EXTREME_RIGHT=0,
        EXTREME_LEFT=1,
        CENTER=2
    };
    
    inline ServoInterpolator( const ServoInterpolatorConf<angle_t,pwm_t>& _conf) :
    {
        memcpy(&conf,&_conf,sizeof(conf));
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
    ServoInterpolatorConf conf;
    coeff_t coeff_x2,coeff_x,coeff_c;
};

struct ServoInterpolatorRosConf
{
    const char * pwm_topic, angle_topic;
    uint32_t pwm_qs = 100, angle_qs = 100;
    const char* pwm_param;
    const char* stop_updates_param;

};

template <typename PwmMsg_t, typename AngleMsg_t>
struct ServoInterpolatorRos
{
    using ServoInterpolatorConf_t = ServoInterpolatorConf<AngleMsg_t::_data_type,PwmMsg_t::_data_type>;
    ServoInterpolatorRos(ros::NodeHandle &_nh, ServoInterpolatorRosConf& conf,\
    ServoInterpolatorConf_t& interpolator_conf) :\
    nh(_nh),interpolator(interpolator_conf),enable_updates(true),
    l_extreme_param(conf.l_extreme_param),r_extreme_param(conf.r_extreme_param),c_param(conf.c_param),
    stop_updates_param(conf.stop_updates_param)
    {
        pwm_pub = _nh.advertise<PwmMsg_t>(conf.pwm_topic,conf.pwm_qs);
        angle_sub = _nh.subscribe<AngleMsg_t,ServoInterpolatorRos>(conf.angle_topic,conf.angle_qs,&interpolatorCallback,this);
    }
    private:
    // void interpolatorCallback(const AngleMsg_t::ConstPtr& msg){
    //     pwm_msg.data = 
    // }
    void updateInterpolator(){
        if(enable_updates){
            std::map<std::string,int> param_dict;
            if(nh.getParam(pwm_param,param_dict))
                interpolator.setPwms(param_dict[pwm_param_extreme_right],param_dict[pwm_param_extreme_left],param_dict[pwm_param_center])
            bool stop_updates;
            if(nh.getParam(stop_updates_param,&stop_updates))
                enable_updates = !stop_updates;
        }
    }
    void interpolatorCallback(const AngleMsg_t::ConstPtr& angle_msg){
        updateInterpolator();
        PwmMsg_t::_data_type& pwm_data = pwm_msg.get()->data;
        interpolator.interpolate(angle_msg.data,pwm_data);
    }
    ros::NodeHandle& nh;
    PwmMsg_t pwm_msg;
    ros::Subscriber angle_sub;
    ros::Publisher pwm_pub;
    ServoInterpolator<ServoInterpolatorConf_t,double> interpolator;
    const std::string pwm_param;
    bool enable_updates;
};
using i_t = ServoInterpolatorConf<std_msgs::Int8::_data_type,std_msgs::Int16::_data_type>;
i_t conf;
ServoInterpolator <i_t>interpolator(conf);
ServoInterpolatorRosConf confr;
ros::NodeHandle nh;
ServoInterpolatorRos <std_msgs::Int16,std_msgs::Int8> iros(nh,confr,conf);