//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2022, AMBF
    (https://github.com/WPI-AIM/ambf)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <amunawar@jhu.edu>
    \author    Adnan Munawar
*/
//==============================================================================

// To silence warnings on MacOS
#define GL_SILENCE_DEPRECATION
#include <afFramework.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace ambf;

class afCameraHMD : public afObjectPlugin
{
public:
    afCameraHMD();
    virtual int init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs) override;
    virtual void graphicsUpdate() override;
    virtual void physicsUpdate(double dt) override;
    virtual void reset() override;
    virtual bool close() override;

    void updateHMDParams();

    void makeFullScreen();
    // Initialization methods
    string read_rostopic_from_config(const afBaseObjectAttribsPtr a_objectAttribs);
    void initilize_ros_subscribers(const afBaseObjectAttribsPtr a_objectAttribs);
    void set_window_size_to_pub_resolution(const afBaseObjectAttribsPtr a_objectAttribs);
    void load_bg_quad_shaders();
    void create_screen_filling_quad();

    // ROS attributes and callbacks
    ros::NodeHandle *ros_node_handle;
    ros::Subscriber img_subscriber;
    ros::Subscriber ar_activate_subscriber;
    void ar_activate_callback(const std_msgs::Bool::ConstPtr &msg);
    void left_img_callback(const sensor_msgs::ImageConstPtr &msg);
    cv_bridge::CvImagePtr img_ptr = nullptr;
    void process_and_set_ros_texture();

    cTexture2dPtr ros_texture;

protected:
    string g_current_filepath;
    afCameraPtr m_camera;
    cMesh *m_screen_filling_quad;
    bool activate_ar = true;
    int m_width;
    int m_height;
    cShaderProgramPtr m_shaderPgm;

    cWorld *m_back_layer_world;
    cWorld *empty_world;
    cWorld *ar_world;
};

AF_REGISTER_OBJECT_PLUGIN(afCameraHMD)
