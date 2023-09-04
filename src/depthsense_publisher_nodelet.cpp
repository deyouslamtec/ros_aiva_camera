#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <depthsense_publisher/DepthsensePublisherConfig.h>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/highgui/highgui.hpp>

#include "libdepthsense/ds.hpp"
#include "libdepthsense/dsutils.h"
#include "libdepthsense/dsutils.hpp"
#include <boost/thread/thread.hpp>

namespace depthsense_publisher
{

#define DEV_MAX_NUM 2

class DepthsensePublisherNodelet : public nodelet::Nodelet
{
    typedef dynamic_reconfigure::Server<depthsense_publisher::DepthsensePublisherConfig> ReconfigureServer;
    boost::shared_ptr<ReconfigureServer> dyn_srv;

    boost::shared_ptr<ros::NodeHandle> nh, pnh;
    boost::thread capture_thread;

    DepthsensePublisherConfig config;

    int camera_count;

    pcl::PointCloud<pcl::PointXYZ> cloud[DEV_MAX_NUM];

    static int img3d_to_PointCloud(pcl::PointCloud<pcl::PointXYZ> &_cloud, cv::Mat &img3d, cv::Mat &rgb_depth)
    {
        _cloud.points.clear();

        for (int y = 0; y < img3d.rows; y++)
        {
            for (int x = 0; x < img3d.cols; x++)
            {
            cv::Vec3f point = img3d.at<cv::Vec3f>(y, x);
            cv::Vec3b rgb_ptr = rgb_depth.at<cv::Vec3b>(y, x);

            pcl::PointXYZ p;
            p.x = -point[0];
            p.y = -point[1];
            p.z =  point[2];
            //p.rgba = ((uint32_t)rgb_ptr[2] << 16) | ((uint32_t)rgb_ptr[1] << 8) | (uint32_t)rgb_ptr[0];
            uint32_t rgb = ((uint32_t)rgb_ptr[2] << 16) | ((uint32_t)rgb_ptr[1] << 8) | (uint32_t)rgb_ptr[0];

            _cloud.points.push_back(p);
            }
        }

        return 0;
    }

    virtual void configCallback(DepthsensePublisherConfig &new_config, uint32_t level)
    {
        config = new_config;

        // show current configuration
        NODELET_INFO_STREAM("Camera name: " << new_config.camera_name);
        NODELET_INFO_STREAM("Publishing with frame_id: " << new_config.frame_id);
        NODELET_INFO_STREAM("distance_max: " << new_config.distance_max);
    }

    virtual int get_depthsense_data() try
    {
        int color_w = 0;
        int color_h = 0;
        int ir_w = 640;
        int ir_h = 400;
        bool is_valid_param = false;

        DepthsensePublisherConfig last_config = config;

        ros::Publisher pub_pcl[DEV_MAX_NUM];
        ros::Publisher pub_ir0[DEV_MAX_NUM];
        ros::Publisher pub_ir1[DEV_MAX_NUM];
        ros::Publisher pub_image[DEV_MAX_NUM];
        ros::Publisher pub_depth[DEV_MAX_NUM];

        pub_pcl[0] = nh->advertise<sensor_msgs::PointCloud2>("points", 1);
        pub_image[0] = nh->advertise<sensor_msgs::Image>("image_raw", 1);
        pub_depth[0] = nh->advertise<sensor_msgs::Image>("depth_raw", 1);

        ds::context ctx;

        while (1) {
            int camera_count;
            do {
                ctx.refresh_devices();
                camera_count = ctx.get_camera_count();
                cv::waitKey(1000);
                fprintf(stdout, "No camera plugged? Waiting...\n");
            } while (camera_count < 1);

            std::cout << "camera_count is " << camera_count << std::endl;

            ds::camera cam = ctx.get_camera(0);

            cam.enable_stream(DS_STREAM_DEPTH,      ir_w, ir_h, DS_FORMAT_Z16, 30);
            cam.enable_stream(DS_STREAM_INFRARED,   ir_w, ir_h, DS_FORMAT_Y8,  30);
            cam.enable_stream(DS_STREAM_INFRARED_2, ir_w, ir_h, DS_FORMAT_Y8,  30);

            if (color_w != 0 && color_h != 0) {
                cam.enable_stream(DS_STREAM_COLOR, color_w, color_h, DS_FORMAT_RGB8, 30);
            }

            int w, h;

            cam.start_capture();

            w = cam.get_stream_width(DS_STREAM_DEPTH);
            h = cam.get_stream_height(DS_STREAM_DEPTH);

            //uint8_t depth_level = DS_DEPTH_LEVEL_MEDIUM;
            uint8_t depth_level = DS_DEPTH_LEVEL_OPTIMIZED;
            //uint8_t depth_level = DS_DEPTH_LEVEL_HIGH;
            cam.set_option(DS_OPTION_DEPTH_CONTROL_PRESET, &depth_level, sizeof(depth_level));
            uint8_t ae_enable = 1;
            cam.set_option(DS_OPTION_LR_AUTO_EXPOSURE_ENABLED, &ae_enable, sizeof(ae_enable));

            uint8_t spot_filt_en = 1;
            cam.set_option(DS_OPTION_SPOT_FILT_ENABLED, &spot_filt_en, sizeof(spot_filt_en));

            uint8_t pyr_depth_en = 1;
            cam.set_option(DS_OPTION_PYR_DEPTH_ENABLED, &pyr_depth_en, sizeof(pyr_depth_en));

            uint32_t min_depth_mm = 80;         // 80   mm
            uint32_t max_depth_mm = 10000;      // 10000 mm
            cam.set_option(DS_OPTION_DEPTH_CLAMP_MIN, &min_depth_mm, sizeof(min_depth_mm));
            cam.set_option(DS_OPTION_DEPTH_CLAMP_MAX, &max_depth_mm, sizeof(max_depth_mm));

            uint8_t emitter_en = 1;
            cam.set_option(DS_OPTION_EMITTER_ENABLED, &emitter_en, sizeof(emitter_en));

            ds::intrinsics intr = cam.get_stream_intrinsics(DS_STREAM_DEPTH);
            std::cout << "intr: cx " << intr.principal_point[0] << ", cy " << intr.principal_point[1] << std::endl;
            std::cout << "intr: fx " << intr.focal_length[0] << ", fy " << intr.focal_length[1] << std::endl;

            ds::extrinsics extr = cam.get_stream_extrinsics(DS_STREAM_INFRARED, DS_STREAM_INFRARED_2);
            std::cout << "T is " << extr.translation[0] << std::endl;

            float hfov, vfov;
            hfov = ds_compute_fov(intr.image_size[0], intr.focal_length[0], intr.principal_point[0]);
            vfov = ds_compute_fov(intr.image_size[1], intr.focal_length[1], intr.principal_point[1]);

            fprintf(stdout, "depth stream: w is %d, h is %d\n", w, h);
            fprintf(stdout, "HFOV %f, VFOV %f.\n", hfov, vfov);

            sensor_msgs::PointCloud2 msg_point;
            sensor_msgs::Image msg_image;
            sensor_msgs::Image msg_depth;
            std_msgs::Header header;

            while (1) try {
                cam.wait_all_streams();

                ds::Mat depth = cam.get_image_mat(DS_STREAM_DEPTH);

                ds::Mat color_depth;
                ds_colorize_depth(color_depth, depth);

                cv::Mat cv_color_depth(color_depth.rows, color_depth.cols, CV_8UC3, color_depth.data);


                ds::Mat img3d;
                ds_repoject_image_to_3d(img3d, depth, intr, false);

                cv::Mat cv_img3d(img3d.rows, img3d.cols, CV_32FC3, img3d.data);

                const void *ir0_ptr = cam.get_image_pixels(DS_STREAM_INFRARED);
                cv::Mat ir0 = cv::Mat(h, w, CV_8UC1, (uint8_t*)ir0_ptr);

                // const void *ir1_ptr = cam.get_image_pixels(DS_STREAM_INFRARED_2);
                // cv::Mat ir1 = cv::Mat(h, w, CV_8UC1, (uint8_t*)ir1_ptr);

                img3d_to_PointCloud(cloud[0], cv_img3d, cv_color_depth);
                // cv::imshow("color_depth", cv_color_depth);
                // cv::imshow("ir0", ir0);
                // cv::imshow("ir1", ir1);

                pcl::toROSMsg(cloud[0], msg_point);
                msg_point.header.stamp = ros::Time::now();
                msg_point.header.frame_id = config.frame_id;
                pub_pcl[0].publish(msg_point);

                cv_bridge::CvImagePtr cv_depth_image = boost::make_shared<cv_bridge::CvImage>(header, "bgr8", cv_color_depth);
                msg_depth = *(cv_depth_image->toImageMsg());
                msg_depth.header.stamp = ros::Time::now();
                pub_depth[0].publish(msg_depth);

                cv_bridge::CvImagePtr cv_ir_image = boost::make_shared<cv_bridge::CvImage>(header, "mono8", ir0);
                msg_image = *(cv_ir_image->toImageMsg());
                msg_image.header.stamp = ros::Time::now();
                pub_image[0].publish(msg_image);

                if (color_w != 0 && color_h != 0) {
                    ds::Mat color = cam.get_image_mat(DS_STREAM_COLOR);
                    cv::Mat cv_rgb_color = cv::Mat(color.rows, color.cols, CV_8UC3, color.data);
                    cv::Mat cv_bgr_color;
                    cv::cvtColor(cv_rgb_color, cv_bgr_color, cv::COLOR_RGB2BGR);
                    cv::imshow("color", cv_bgr_color);
                }

                char key = cv::waitKey(1);
                    if (key == 'q') {
                    goto out;
                }
            }
            catch (const ds::camera_disconnected_error &e) {
                std::cerr << e.what() << std::endl;
                ctx.refresh_devices();
                break;
            }
        }

        out:    
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    virtual void onInit()
    {
        nh.reset(new ros::NodeHandle(getNodeHandle()));
        pnh.reset(new ros::NodeHandle(getPrivateNodeHandle()));

        // set parameters from dynamic reconfigure server
        dyn_srv = boost::make_shared<ReconfigureServer>(*pnh);
        auto f = boost::bind(&DepthsensePublisherNodelet::configCallback, this, _1, _2);
        dyn_srv->setCallback(f);

        capture_thread = boost::thread(boost::bind(&DepthsensePublisherNodelet::get_depthsense_data, this));
        capture_thread.detach();
    }
};

} // namespace depthsense_publisher

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(depthsense_publisher::DepthsensePublisherNodelet, nodelet::Nodelet)
