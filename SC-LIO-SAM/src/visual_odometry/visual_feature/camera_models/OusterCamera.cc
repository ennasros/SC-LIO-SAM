#include "OusterCamera.h"

#include <cmath>
#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "gpl.h"


// #include <iostream>
namespace camodocal
{

OusterCamera::Parameters::Parameters()
 : Camera::Parameters(OUSTER)
 , m_fov(0.0)
{

}

OusterCamera::Parameters::Parameters(const std::string& cameraName,
                                      int w, int h, double fov)
 : Camera::Parameters(OUSTER, cameraName, w, h)
 , m_fov(fov)
{
}

double&
OusterCamera::Parameters::fov(void)
{
    return m_fov;
}

double
OusterCamera::Parameters::fov(void) const
{
    return m_fov;
}

bool
OusterCamera::Parameters::readFromYamlFile(const std::string& filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        return false;
    }

    if (!fs["model_type"].isNone())
    {
        std::string sModelType;
        fs["model_type"] >> sModelType;

        if (sModelType.compare("OUSTER") != 0)
        {
            return false;
        }
    }

    m_modelType = OUSTER;
    fs["camera_name"] >> m_cameraName;
    m_imageWidth = static_cast<int>(fs["image_width"]);
    m_imageHeight = static_cast<int>(fs["image_height"]);
    m_fov = static_cast<double>(fs["field_of_view"]);
    
    return true;
}

void
OusterCamera::Parameters::writeToYamlFile(const std::string& filename) const
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    fs << "model_type" << "OUSTER";
    fs << "camera_name" << m_cameraName;
    fs << "image_width" << m_imageWidth;
    fs << "image_height" << m_imageHeight;
    fs << "field_of_view" << m_fov;

    fs.release();
}

OusterCamera::Parameters&
OusterCamera::Parameters::operator=(const OusterCamera::Parameters& other)
{
    if (this != &other)
    {
        m_modelType = other.m_modelType;
        m_cameraName = other.m_cameraName;
        m_imageWidth = other.m_imageWidth;
        m_imageHeight = other.m_imageHeight;
        m_fov = other.m_fov;

    }

    return *this;
}

std::ostream&
operator<< (std::ostream& out, const OusterCamera::Parameters& params)
{
    out << "Camera Parameters:" << std::endl;
    out << "    model_type "  << "OUSTER" << std::endl;
    out << "   camera_name "  << params.m_cameraName << std::endl;
    out << "   image_width "  << params.m_imageWidth << std::endl;
    out << "  image_height "  << params.m_imageHeight << std::endl;
    out << "  field_of_view " << params.m_fov << std::endl;

    return out;
}

OusterCamera::OusterCamera()
 : m_fov(45)
{

}

OusterCamera::OusterCamera(const std::string& cameraName,
                             int imageWidth, int imageHeight, double fieldOfView)
 : mParameters(cameraName, imageWidth, imageHeight, fieldOfView)
{
}

OusterCamera::OusterCamera(const OusterCamera::Parameters& params)
 : mParameters(params)
{
}

Camera::ModelType
OusterCamera::modelType(void) const
{
    return mParameters.modelType();
}

const std::string&
OusterCamera::cameraName(void) const
{
    return mParameters.cameraName();
}

int
OusterCamera::imageWidth(void) const
{
    return mParameters.imageWidth();
}

int
OusterCamera::imageHeight(void) const
{
    return mParameters.imageHeight();
}

void
OusterCamera::estimateIntrinsics(const cv::Size& boardSize,
                                  const std::vector< std::vector<cv::Point3f> >& objectPoints,
                                  const std::vector< std::vector<cv::Point2f> >& imagePoints)
{
    // not implemented
}

/**
 * \brief Lifts a point from the image plane to the unit sphere
 *
 * \param p image coordinates
 * \param P coordinates of the point on the sphere
 */
void
OusterCamera::liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const
{
    liftProjective(p, P);
    P.normalize();
}

/**
 * \brief Lifts a point from the image plane to its projective ray
 *
 * \param p image coordinates
 * \param P coordinates of the projective ray
 */
void
OusterCamera::liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const
{
    // spherical coordinates
    double r = 1.0;
    double theta = 360.0/mParameters.imageWidth() * p(0) - 180;
    double phi = m_fov/2 - m_fov/mParameters.imageHeight() * p(1); 

    // std::cout << "p2d = " << p.transpose() << ", theta = " << theta << ", phi = " << phi << std::endl;

    // coordinate transform
    theta = 90.0 - theta;
    phi = 90.0 - phi;

    theta *= M_PI / 180.0;
    phi *= M_PI / 180.0;

    double y = r * sin(phi) * cos(theta);
    double x = r * sin(phi) * sin(theta);
    double z = r * cos(phi);

    // // Obtain a projective ray
    P << x, y, z;

    // std::cout << "p3d = " << P.transpose() << std::endl;
}


/**
 * \brief Project a 3D point (\a x,\a y,\a z) to the image plane in (\a u,\a v)
 *
 * \param P 3D point coordinates
 * \param p return value, contains the image point coordinates
 */
void
OusterCamera::spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const
{
    double row_angle = atan2(-P(2), sqrt(P(0) * P(0) + P(1) * P(1))) * 180.0 / M_PI + m_fov/2;
    double col_angle = atan2(P(1), P(0)) * 180.0 / M_PI + 180.0;
    double x = col_angle/360.0*imageWidth();
    double y = row_angle/m_fov*imageHeight();
    p << x, y;
}

/**
 * \brief Projects an undistorted 2D point p_u to the image plane
 *
 * \param p_u 2D point coordinates
 * \return image point coordinates
 */
void
OusterCamera::undistToPlane(const Eigen::Vector2d& p_u, Eigen::Vector2d& p) const
{
    // not implemented
}

// /**
//  * \brief Apply distortion to input point (from the normalised plane)
//  *
//  * \param p_u undistorted coordinates of point on the normalised plane
//  * \return to obtain the distorted point: p_d = p_u + d_u
//  */
// void
// OusterCamera::distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const
// {
//     // not implemented
// }
// 
// /**
//  * \brief Apply distortion to input point (from the normalised plane)
//  *        and calculate Jacobian
//  *
//  * \param p_u undistorted coordinates of point on the normalised plane
//  * \return to obtain the distorted point: p_d = p_u + d_u
//  */
// void
// OusterCamera::distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u,
//                           Eigen::Matrix2d& J) const
// {
//     // not implemented
// }

// void
// OusterCamera::initUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale) const
// {
//     // not implemented
// }

cv::Mat
OusterCamera::initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2,
                                       float fx, float fy,
                                       cv::Size imageSize,
                                       float cx, float cy,
                                       cv::Mat rmat) const
{
    // if (imageSize == cv::Size(0, 0))
    // {
    //     imageSize = cv::Size(mParameters.imageWidth(), mParameters.imageHeight());
    // }

    // cv::Mat mapX = cv::Mat::zeros(imageSize.height, imageSize.width, CV_32F);
    // cv::Mat mapY = cv::Mat::zeros(imageSize.height, imageSize.width, CV_32F);

    // Eigen::Matrix3f R, R_inv;
    // cv::cv2eigen(rmat, R);
    // R_inv = R.inverse();

    // // assume no skew
    // Eigen::Matrix3f K_rect;

    // if (cx == -1.0f || cy == -1.0f)
    // {
    //     K_rect << fx, 0, imageSize.width / 2,
    //               0, fy, imageSize.height / 2,
    //               0, 0, 1;
    // }
    // else
    // {
    //     K_rect << fx, 0, cx,
    //               0, fy, cy,
    //               0, 0, 1;
    // }

    // if (fx == -1.0f || fy == -1.0f)
    // {
    //     K_rect(0,0) = 370;//mParameters.fx();
    //     K_rect(1,1) = 370;//mParameters.fy();
    // }

    // Eigen::Matrix3f K_rect_inv = K_rect.inverse();

    // for (int v = 0; v < imageSize.height; ++v)
    // {
    //     for (int u = 0; u < imageSize.width; ++u)
    //     {
    //         Eigen::Vector3f xo;
    //         xo << u, v, 1;

    //         Eigen::Vector3f uo = R_inv * K_rect_inv * xo;

    //         Eigen::Vector2d p;
    //         spaceToPlane(uo.cast<double>(), p);

    //         mapX.at<float>(v,u) = p(0);
    //         mapY.at<float>(v,u) = p(1);
    //     }
    // }

    // cv::convertMaps(mapX, mapY, map1, map2, CV_32FC1, false);

    // cv::Mat K_rect_cv;
    // cv::eigen2cv(K_rect, K_rect_cv);
    // return K_rect_cv;
}

int
OusterCamera::parameterCount(void) const
{
    return 1;
}

const OusterCamera::Parameters&
OusterCamera::getParameters(void) const
{
    return mParameters;
}

void
OusterCamera::setParameters(const OusterCamera::Parameters& parameters)
{
    mParameters = parameters;
}

void
OusterCamera::readParameters(const std::vector<double>& parameterVec)
{
    if ((int)parameterVec.size() != parameterCount())
    {
        return;
    }

    Parameters params = getParameters();

    params.fov() = parameterVec.at(0);

    setParameters(params);
}

void
OusterCamera::writeParameters(std::vector<double>& parameterVec) const
{
    parameterVec.resize(parameterCount());
    parameterVec.at(0) = mParameters.fov();
}

void
OusterCamera::writeParametersToYamlFile(const std::string& filename) const
{
    mParameters.writeToYamlFile(filename);
}

std::string
OusterCamera::parametersToString(void) const
{
    std::ostringstream oss;
    oss << mParameters;

    return oss.str();
}

}
