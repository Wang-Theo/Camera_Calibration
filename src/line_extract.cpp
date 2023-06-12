// Jun 12 created by Wang Renjie //

#include "cam_cali/line_extract.hpp"

namespace calibration {

void LineExtract::LineExtractSubscriber(ros::NodeHandle nh)
{
    subscriber_image1 = new message_filters::Subscriber<sensor_msgs::Image>(nh, img1_topic.c_str(), 1);
    subscriber_image2 = new message_filters::Subscriber<sensor_msgs::Image>(nh, img2_topic.c_str(), 1);

    sync_ = new  message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *subscriber_image1, *subscriber_image2);
    sync_->registerCallback(boost::bind(&LineExtract::ImageCallback, this, _1, _2));

    pub_ = nh.advertise<sensor_msgs::Image>("/line_extract", 1);

    ros::spin();  
}

void LineExtract::ImageCallback(const sensor_msgs::ImageConstPtr& msg_cam1, const sensor_msgs::ImageConstPtr& msg_cam2){
    std::cout<<"----import images from two cameras---"<<std::endl;
    cv::Mat image1 = cv_bridge::toCvShare(msg_cam1, "mono8")->image;
    cv::Mat image2 = cv_bridge::toCvShare(msg_cam2, "mono8")->image;

    if(image1.data==NULL||image2.data==NULL)
    {
        std::cout<<"no image received"<<std::endl;
    }

    // std::vector<cv::line_descriptor::KeyLine> keylines1,keylines2;
    // LineExtract line_fearture;
    // cv::Mat out_img = line_fearture.ExtractLineSegment(image1,image2,keylines1,keylines2);

    cv::Mat out_img = LineExtract::FastLineDetector(image2, image1);

    sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", out_img).toImageMsg();
    pub_.publish(out_msg);
}

cv::Mat LineExtract::FastLineDetector(cv::Mat image1, cv::Mat image2){
    int    length_threshold    = 10;  
    float  distance_threshold  = 1.41421356f;  
    double canny_th1           = 50.0;  
    double canny_th2           = 50.0;  
    int    canny_aperture_size = 3;  
    bool   do_merge            = false;  
    cv::Ptr<cv::ximgproc::FastLineDetector> fld = cv::ximgproc::createFastLineDetector(  
            length_threshold,  
            distance_threshold,   
            canny_th1,   
            canny_th2,   
            canny_aperture_size,  
            do_merge);  
    std::vector<cv::Vec4f> lines_fld;
    for(int run_count = 0; run_count < 10; run_count++) {  
        // Detect the lines with FLD  
        lines_fld.clear();
        fld->detect(image1, lines_fld);  
    }  
    // Show found lines with FLD  
    cv::Mat line_image_fld(image1);  
    fld->drawSegments(line_image_fld, lines_fld);  
    return line_image_fld;
}

cv::Mat LineExtract::ExtractLineSegment(const cv::Mat &image1, const cv::Mat &image2, 
                                        std::vector<cv::line_descriptor::KeyLine> &keylines1,
                                        std::vector<cv::line_descriptor::KeyLine> &keylines2)
{
    cv::Mat mLdesc,mLdesc2;

    std::vector<std::vector<cv::DMatch>> lmatches;

    cv::Ptr<cv::line_descriptor::BinaryDescriptor> lbd = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();
    cv::Ptr<cv::line_descriptor::LSDDetector> lsd = cv::line_descriptor::LSDDetector::createLSDDetector();

    std::cout<<"extract lsd line segments"<<std::endl;
    lsd->detect(image1, keylines1, 1.2,1);
    lsd->detect(image2, keylines2, 1.2,1);
    int lsdNFeatures = 50;
    std::cout<<"filter lines"<<std::endl;
    if(keylines1.size()>lsdNFeatures)
    {
        std::sort(keylines1.begin(), keylines1.end(), LineExtract::sort_lines_by_response());
        keylines1.resize(lsdNFeatures);
        for( int i=0; i<lsdNFeatures; i++)
            keylines1[i].class_id = i;
    }
    if(keylines2.size()>lsdNFeatures)
    {
        std::sort(keylines2.begin(), keylines2.end(), LineExtract::sort_lines_by_response());
        keylines2.resize(lsdNFeatures);
        for(int i=0; i<lsdNFeatures; i++)
            keylines2[i].class_id = i;
    }
    std::cout<<"lbd describle"<<std::endl;
    lbd->compute(image1, keylines1, mLdesc);
    lbd->compute(image2, keylines2,mLdesc2);//计算特征线段的描述子
    cv::BFMatcher* bfm = new cv::BFMatcher(cv::NORM_HAMMING, false);
    bfm->knnMatch(mLdesc, mLdesc2, lmatches, 2);
    std::vector<cv::DMatch> matches;
    for(size_t i=0;i<lmatches.size();i++)
    {
        const cv::DMatch& bestMatch = lmatches[i][0];
        const cv::DMatch& betterMatch = lmatches[i][1];
        float  distanceRatio = bestMatch.distance / betterMatch.distance;
        if (distanceRatio < 0.75)
            matches.push_back(bestMatch);
    }

    cv::Mat out_img_;
    std::vector<char> mask( lmatches.size(), 1 );
    drawLineMatches( image1, keylines1, image2, keylines2, matches, out_img_, cv::Scalar::all( -1 ), cv::Scalar::all( -1 ), mask,
                     cv::line_descriptor::DrawLinesMatchesFlags::DEFAULT );

    return out_img_;
}

} // namespace calibration