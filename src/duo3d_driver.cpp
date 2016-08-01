///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2016, Code laboratories, Inc.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>

// Config parameters
#include <duo3d_driver/Duo3DConfig.h>

// Include Dense3DMT
#include <Dense3DMT.h>

using namespace std;
using namespace cv;

#define NODE_NAME   "duo3d"

namespace duo3d_driver
{
// topic items
enum { LEFT, RIGHT, RGB, DEPTH, POINT_CLOUD, IMU, TEMP, ITEM_COUNT };
const vector<string> prefix =
{
    "left", "right", "rgb", "depth", "point_cloud", "imu", "temperature"
};

// parameter names
const vector<string> topic_param_name =
{
    prefix[LEFT] + "_topic",
    prefix[RIGHT] + "_topic",
    prefix[RGB] + "_topic",
    prefix[DEPTH] + "_topic",
    prefix[POINT_CLOUD] + "_topic",
    prefix[IMU] + "_topic",
    prefix[TEMP] + "_topic"
};
const vector<string> cam_info_topic_param_name =
{
    prefix[LEFT] + "_cam_info_topic",
    prefix[RIGHT] + "_cam_info_topic",
    prefix[RGB] + "_cam_info_topic",
    prefix[DEPTH] + "_cam_info_topic"
};
const vector<string> frame_id_param_name =
{
    prefix[LEFT] + "_frame_id",
    prefix[RIGHT] + "_frame_id",
    prefix[RGB] + "_frame_id",
    prefix[DEPTH] + "_frame_id",
    prefix[POINT_CLOUD] + "_frame_id",
    prefix[IMU] + "_frame_id",
    prefix[TEMP] + "_frame_id"
};

// parameter default values
vector<string> topic_name =
{
    prefix[LEFT] + "/image_rect",
    prefix[RIGHT] + "/image_rect",
    prefix[RGB] + "/image_rect",
    prefix[DEPTH] + "/image_raw",
    prefix[POINT_CLOUD] + "/image_raw",
    prefix[IMU] + "/data_raw",
    prefix[TEMP]
};
vector<string> cam_info_topic_name =
{
    prefix[LEFT] + "/camera_info",
    prefix[RIGHT] + "/camera_info",
    prefix[RGB] + "/camera_info",
    prefix[DEPTH] + "/camera_info"
};
vector<string> frame_id_name =
{
    string(NODE_NAME) + "/camera_frame",      // LEFT
    string(NODE_NAME) + "/camera_frame",      // RIGHT
    string(NODE_NAME) + "/camera_frame",      // RGB
    string(NODE_NAME) + "/camera_frame",      // DEPTH
    string(NODE_NAME) + "/camera_frame",      // POINT_CLOUD
    string(NODE_NAME) + "/imu_frame",         // IMU
    string(NODE_NAME) + "/temperature_frame"  // TEMP
};

// DUO3DDriver class
class DUO3DDriver
{
    Dense3DMTInstance _dense3dInstance;
    string _dense3d_license;

    ros::NodeHandle _nh;
    // Dynamic reconfigure server
    dynamic_reconfigure::Server<Duo3DConfig> _server;

    // Camera Parameters
    float _frame_rate;
    vector<int> _image_size;

    double _start_time;
    uint32_t _frame_num;

    // Dense3D
    Mat _colorLut;

    // Image publishers
    image_transport::Publisher _pub_image[ITEM_COUNT-1];
    // Camera info publishers
    ros::Publisher _pub_cam_info[ITEM_COUNT-1];
    // Camera info messages
    sensor_msgs::CameraInfo _msg_cam_info[ITEM_COUNT-1];
    // Point cloud publisher
    ros::Publisher _pub_point_cloud;
    // IMU publisher
    ros::Publisher _pub_imu;
    // Temperature publisher
    ros::Publisher _pub_temperature;

    // Gyroscope offset calibration
    int _num_samples;
    double _gyro_offset[3];

public:
    DUO3DDriver()
        : _dense3dInstance(NULL),
          _dense3d_license("XXXXX-XXXXX-XXXXX-XXXXX-XXXXX"),
          _nh(NODE_NAME),
          _frame_rate(30),
          _image_size({640, 480})
	{
        // Build color lookup table for depth display
        _colorLut = Mat(Size(256, 1), CV_8UC3);
        for(int i = 0; i < 256; i++)
            _colorLut.at<Vec3b>(i) = (i==0) ? Vec3b(0, 0, 0) : HSV2RGB(i/256.0f, 1, 1);

        getParams();

        image_transport::ImageTransport itrans(_nh);
        for(int i = 0; i < topic_name.size(); i++)
        {
            if(i == POINT_CLOUD)
                _pub_point_cloud = _nh.advertise<sensor_msgs::PointCloud2>(topic_name[i], 16);
            else if(i == IMU)
                _pub_imu = _nh.advertise<sensor_msgs::Imu>(topic_name[i], 100);
            else if(i == TEMP)
                _pub_temperature = _nh.advertise<sensor_msgs::Temperature>(topic_name[i], 100);
            else
                _pub_image[i] = itrans.advertise(topic_name[i], 16);
        }
        for(int i = 0; i < cam_info_topic_name.size(); i++)
            _pub_cam_info[i] = _nh.advertise<sensor_msgs::CameraInfo>(cam_info_topic_name[i], 1);
    }
    ~DUO3DDriver()
    {
        closeDense3D();
    }

    void run()
    {
        try
        {
            if(!openDense3D()) return;

            fillCameraInfo();

            _server.setCallback(boost::bind(&DUO3DDriver::dynamicCallback, this, _1, _2));

            _frame_num = 0;   // reset frame number
            _num_samples = 0;

            if(!Dense3DStart(_dense3dInstance,
                            [](const PDense3DFrame pFrame, void *pUserData)
                            {
                                if(!ros::isShuttingDown())
                                    ((DUO3DDriver*)pUserData)->dense3dCallback(pFrame);
                            }, this))
            {
                ROS_ERROR("Could not start DUO camera");
                return;
            }
            ros::spin();
        }
        catch(...)
        {
            ros::shutdown();
        }
    }
protected:
    int width() { return _image_size[0]; }
    int height() { return _image_size[1]; }
    double fps() { return _frame_rate; }
    Vec3b HSV2RGB(float hue, float sat, float val)
    {
        float x, y, z;

        if(hue == 1) hue = 0;
        else         hue *= 6;

        int i = static_cast<int>(floorf(hue));
        float f = hue - i;
        float p = val * (1 - sat);
        float q = val * (1 - (sat * f));
        float t = val * (1 - (sat * (1 - f)));

        switch(i)
        {
            case 0: x = val; y = t; z = p; break;
            case 1: x = q; y = val; z = p; break;
            case 2: x = p; y = val; z = t; break;
            case 3: x = p; y = q; z = val; break;
            case 4: x = t; y = p; z = val; break;
            case 5: x = val; y = p; z = q; break;
        }
        return Vec3b((uchar)(z * 255), (uchar)(y * 255), (uchar)(x * 255));
    }
    void getParams()
    {
        ros::NodeHandle nh("~");
        nh.getParam("frame_rate", _frame_rate);
        nh.getParam("image_size", _image_size);
        nh.getParam("dense3d_license", _dense3d_license);

        for(int i = 0; i < topic_param_name.size(); i++)
            nh.getParam(topic_param_name[i], topic_name[i]);
        for(int i = 0; i < cam_info_topic_param_name.size(); i++)
            nh.getParam(cam_info_topic_param_name[i], cam_info_topic_name[i]);
        for(int i = 0; i < frame_id_param_name.size(); i++)
            nh.getParam(frame_id_param_name[i], frame_id_name[i]);
    }

    void dynamicCallback(Duo3DConfig &config, uint32_t level)
    {
        if(!_dense3dInstance) return;
        DUOInstance duo = GetDUOInstance(_dense3dInstance);
        // Set DUO parameters
        if(duo)
        {
            SetDUOGain(duo, config.gain);
            SetDUOExposure(duo, config.exposure);
            SetDUOAutoExposure(duo, config.auto_exposure);
            SetDUOCameraSwap(duo, config.camera_swap);
            SetDUOHFlip(duo, config.horizontal_flip);
            SetDUOVFlip(duo, config.vertical_flip);
            SetDUOLedPWM(duo, config.led);
            SetDUOIMURange(duo, config.accel_range, config.gyro_range);
            SetDUOIMURate(duo, config.imu_rate);
        }
        // Set Dense3D parameters
        Dense3DParams params;
        params.mode = config.processing_mode;
        params.scale = config.image_scale;
        params.numDisparities = config.num_disparities;
        params.sadWindowSize = config.sad_window_size;
        params.preFilterCap = config.pre_filter_cap;
        params.uniqenessRatio = config.uniqueness_ratio;
        params.speckleWindowSize = config.speckle_window_size;
        params.speckleRange = config.speckle_range;
        SetDense3Params(_dense3dInstance, params);
    }

    void dense3dCallback(const PDense3DFrame pFrame)
    {
        bool needDense3d = (_pub_image[DEPTH].getNumSubscribers() > 0) ||
                           (_pub_point_cloud.getNumSubscribers() > 0);
        // Enable Dense3d processing
        SetDense3DProcessing(_dense3dInstance, needDense3d);

        // Set the start time
        if(_frame_num++ == 0) _start_time = ros::Time::now().toSec();

        Size size(pFrame->duoFrame->width, pFrame->duoFrame->height);

        // Create Mat for left and right images
        Mat left(size, CV_8UC1, pFrame->duoFrame->leftData);
        Mat right(size, CV_8UC1, pFrame->duoFrame->rightData);

        for(int i = 0; i < ITEM_COUNT; i++)
        {
            std_msgs::Header header;
            header.stamp = ros::Time(_start_time + (double)pFrame->duoFrame->timeStamp / 10000.0);
            header.frame_id = frame_id_name[i];

            if((i == LEFT) && (_pub_image[i].getNumSubscribers() > 0))
            {
                _pub_image[i].publish(cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, left).toImageMsg());
                _msg_cam_info[i].header = header;
                _pub_cam_info[i].publish(_msg_cam_info[i]);
            }
            if((i == RIGHT) && (_pub_image[i].getNumSubscribers() > 0))
            {
                _pub_image[i].publish(cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, right).toImageMsg());
                _msg_cam_info[i].header = header;
                _pub_cam_info[i].publish(_msg_cam_info[i]);
            }
            if((i == RGB) && (_pub_image[i].getNumSubscribers() > 0))
            {
                cv::Mat rgb(size, CV_8UC3);
                uint8_t *src = left.data;
                uint8_t *dst = rgb.data;
                // Convert gray image to RGB
                for(int j = 0; j < left.total(); j += 8)
                {
                    *dst++ = *src; *dst++ = *src; *dst++ = *src++;
                    *dst++ = *src; *dst++ = *src; *dst++ = *src++;
                    *dst++ = *src; *dst++ = *src; *dst++ = *src++;
                    *dst++ = *src; *dst++ = *src; *dst++ = *src++;
                    *dst++ = *src; *dst++ = *src; *dst++ = *src++;
                    *dst++ = *src; *dst++ = *src; *dst++ = *src++;
                    *dst++ = *src; *dst++ = *src; *dst++ = *src++;
                    *dst++ = *src; *dst++ = *src; *dst++ = *src++;
                }
                _pub_image[i].publish(cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, rgb).toImageMsg());
                _msg_cam_info[i].header = header;
                _pub_cam_info[i].publish(_msg_cam_info[i]);
            }
            if((i == DEPTH) && pFrame->dense3dDataValid)
            {
                Mat disp8;
                Mat(size, CV_32FC1, pFrame->disparityData).convertTo(disp8, CV_8UC1, 255.0/(pFrame->dense3dParams.numDisparities*16.0));
                Mat rgbDepth;
                cvtColor(disp8, rgbDepth, cv::COLOR_GRAY2BGR);
                LUT(rgbDepth, _colorLut, rgbDepth);
                _pub_image[i].publish(cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, rgbDepth).toImageMsg());
                _msg_cam_info[i].header = header;
                _pub_cam_info[i].publish(_msg_cam_info[i]);
            }
            if((i == POINT_CLOUD) && pFrame->dense3dDataValid)
            {
                Mat depth3d = Mat(size, CV_32FC3, pFrame->depthData);
                pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
                point_cloud.width = width();
                point_cloud.height = height();
                uint8_t *color = left.data;
                PDense3DDepth depth = (PDense3DDepth)depth3d.data;
                for(int j = 0; j < left.total(); j++)
                {
                    if(depth[j].z >= 10000.0f) continue;
                    pcl::PointXYZRGB p;
                    p.x = depth[j].x * 0.001f;
                    p.y = depth[j].y * 0.001f;
                    p.z = depth[j].z * 0.001f;
                    uint32_t rgb = ((uint32_t)color[j] << 16 | (uint32_t)color[j] << 8 | (uint32_t)color[j]);
                    p.rgb = *reinterpret_cast<float*>(&rgb);
                    point_cloud.push_back(p);
                }
                sensor_msgs::PointCloud2 output;
                pcl::toROSMsg(point_cloud, output);
                output.header = header;
                _pub_point_cloud.publish(output);
            }
            if((i == IMU) && pFrame->duoFrame->IMUPresent && (_pub_imu.getNumSubscribers() > 0))
            {
                sensor_msgs::Imu imu_msg;
                for(int j = 0; j < pFrame->duoFrame->IMUSamples; j++)
                {
                    // Calibrate gyroscope offsets for 100 samples
                    if(_num_samples < 100)
                    {
                        if(_num_samples == 0)
                            _gyro_offset[0] = _gyro_offset[1] = _gyro_offset[2] = 0;
                        _gyro_offset[0] += pFrame->duoFrame->IMUData[j].gyroData[0];
                        _gyro_offset[1] += pFrame->duoFrame->IMUData[j].gyroData[1];
                        _gyro_offset[2] += pFrame->duoFrame->IMUData[j].gyroData[2];
                    }
                    else if(_num_samples == 100)
                    {
                        _gyro_offset[0] /= 100.0f;
                        _gyro_offset[1] /= 100.0f;
                        _gyro_offset[2] /= 100.0f;
                        ROS_INFO("Calculated gyroscope offets [%g, %g, %g]",
                                 _gyro_offset[0],
                                 _gyro_offset[1],
                                 _gyro_offset[2]);
                    }
                    else
                    {
                        // Adjust timestamp
                        header.stamp = ros::Time(_start_time + (double)pFrame->duoFrame->IMUData[j].timeStamp / 10000.0);
                        imu_msg.header = header;
                        // Accelerations should be in m/s^2
                        imu_msg.linear_acceleration.x = pFrame->duoFrame->IMUData[j].accelData[0] * 9.81;
                        imu_msg.linear_acceleration.y = -pFrame->duoFrame->IMUData[j].accelData[1] * 9.81;
                        imu_msg.linear_acceleration.z = -pFrame->duoFrame->IMUData[j].accelData[2] * 9.81;
                        // Angular velocity should be in rad/sec
                        imu_msg.angular_velocity.x = DEG2RAD(pFrame->duoFrame->IMUData[j].gyroData[0] - _gyro_offset[0]);
                        imu_msg.angular_velocity.y = -DEG2RAD(pFrame->duoFrame->IMUData[j].gyroData[1] - _gyro_offset[1]);
                        imu_msg.angular_velocity.z = -DEG2RAD(pFrame->duoFrame->IMUData[j].gyroData[2] - _gyro_offset[2]);
                        _pub_imu.publish(imu_msg);
                    }
                    if(_num_samples < 101) _num_samples++;
                }
            }
            if((i == TEMP) && pFrame->duoFrame->IMUPresent && (_pub_temperature.getNumSubscribers() > 0))
            {
                sensor_msgs::Temperature temp_msg;
                for(int j = 0; j < pFrame->duoFrame->IMUSamples; j++)
                {
                    header.stamp = ros::Time(_start_time + (double)pFrame->duoFrame->IMUData[j].timeStamp / 10000.0);
                    temp_msg.header = header;
                    temp_msg.temperature = pFrame->duoFrame->IMUData[j].tempData;
                    _pub_temperature.publish(temp_msg);
                }
            }
        }
    }

    bool fillCameraInfo()
    {
        if(!_dense3dInstance) return false;
        DUOInstance duo = GetDUOInstance(_dense3dInstance);
        if(!duo) return false;
        DUO_STEREO stereo;
        if(!GetDUOStereoParameters(duo, &stereo))
        {
            ROS_ERROR("Could not get DUO camera calibration data");
            return false;
        }
        for(int i = 0; i < ITEM_COUNT-1; i++)
        {
            _msg_cam_info[i].width = width();
            _msg_cam_info[i].height = height();
            _msg_cam_info[i].distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
            _msg_cam_info[i].D.resize(5, 0.0);
            _msg_cam_info[i].K.fill(0.0);
            _msg_cam_info[i].K[0] = stereo.P1[0]; // fx
            _msg_cam_info[i].K[2] = stereo.P1[2]; // cx
            _msg_cam_info[i].K[4] = stereo.P1[4]; // fy
            _msg_cam_info[i].K[6] = stereo.P1[6]; // cy
            _msg_cam_info[i].K[8] = 1.0;
            _msg_cam_info[i].R.fill(0.0);
            _msg_cam_info[i].P.fill(0.0);
            _msg_cam_info[i].P[0] = stereo.P1[0]; // fx
            _msg_cam_info[i].P[2] = stereo.P1[2]; // cx
            _msg_cam_info[i].P[5] = stereo.P1[5]; // fy
            _msg_cam_info[i].P[6] = stereo.P1[6]; // cy
            _msg_cam_info[i].P[10] = 1.0;
            if(i == RIGHT)
                _msg_cam_info[i].P[3] = stereo.P2[3] / 1000.0;  // (fx * baseline) / 1000
        }
        return true;
    }

    bool openDense3D()
    {
        // Find the optimal sensor binning parameters for given (width, height)
        // This maximizes sensor imaging area for given resolution
        int binning = DUO_BIN_NONE;
        if(width() <= 752/4)        binning += DUO_BIN_HORIZONTAL4;
        else if(width() <= 752/2)   binning += DUO_BIN_HORIZONTAL2;
        if(height() <= 480/4)       binning += DUO_BIN_VERTICAL4;
        else if(height() <= 480/2)  binning += DUO_BIN_VERTICAL2;

        DUOResolutionInfo ri;
        if(!EnumerateDUOResolutions(&ri, 1, width(), height(), binning, fps()))
        {
            ROS_ERROR("Invalid DUO camera resolution");
            return false;
        }
        ROS_INFO("DUO resolution [%d x %d] @ %f fps", width(), height(), fps());

        if(!Dense3DOpen(&_dense3dInstance))
        {
            ROS_ERROR("Could not open Dense3D library");
            return false;
        }
        if(!SetDense3DLicense(_dense3dInstance, _dense3d_license.c_str()))
        {
            ROS_ERROR("Invalid or missing Dense3D license. To get your license visit https://duo3d.com/account");
            return false;
        }
        // Set the image size
        if(!SetDense3DImageInfo(_dense3dInstance, width(), height(), fps()))
        {
            ROS_ERROR("Invalid image size");
            return false;
        }
        return true;
    }
    void closeDense3D()
    {
        if(_dense3dInstance)
        {
            Dense3DStop(_dense3dInstance);
            Dense3DClose(_dense3dInstance);
            _dense3dInstance = NULL;
        }
    }
};
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    duo3d_driver::DUO3DDriver duo;
    duo.run();
	return 0;
}
