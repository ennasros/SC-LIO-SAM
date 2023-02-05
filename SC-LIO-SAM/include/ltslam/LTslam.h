#pragma once

#include "lio_sam/cloud_info.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam/nonlinear/ISAM2.h>

#include <optional>

#include "RosParamServer.h"
#include "BetweenFactorWithAnchoring.h"
#include "Session.h"

using namespace gtsam;
using namespace LTslamParam;

class LTslam : public RosParamServer
{
public:
    tf::TransformBroadcaster tfMap2Odom;
    tf::Transform previous_to_current;

    // Sessions 
    static inline int num_sessions = 1; // session index should start from 1

    // const static inline int kSessionStartIdxOffset = 1000000; // int max 2147483647 so ok.

    Sessions sessions_;
    Session prev_session_;
    SessionsDict sessions_dict_;

    ros::Subscriber subCloud;
    ros::Publisher pubPreviousCloud;

    // Pose graph 
    
    const int target_sess_idx = 1; // means the centralt session. recommend to use 1 for it (because for the stable indexing for anchor node index) 
    const int source_sess_idx = 2; // TODO: generalize this function, change this sess_idx pair for arbitary pair (e.g., [2,7], [1,5])

    std::vector<std::pair<int, int>> SCLoopIdxPairs_;
    std::vector<std::pair<int, int>> RSLoopIdxPairs_;

    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    const gtsam::Pose3 poseOrigin;

    noiseModel::Diagonal::shared_ptr priorNoise;
    noiseModel::Diagonal::shared_ptr odomNoise;
    noiseModel::Diagonal::shared_ptr loopNoise;
    noiseModel::Diagonal::shared_ptr largeNoise;
    noiseModel::Base::shared_ptr robustNoise;

    std::mutex mtx;
    const int numberOfCores = 8;


public:
    LTslam();
    ~LTslam();

    void keyframeHandler(const sensor_msgs::PointCloud2ConstPtr& keyframe);
    // void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr& msgIn);
    void laserCloudInfoHandler(const sensor_msgs::PointCloud2::ConstPtr& msgIn);


    void run( void );

    void initNoiseConstants();
    void initOptimizer();

    // void loadAllSessions();
    void loadPrevSession();

    friend int genGlobalNodeIdx(const int&, const int&);
    friend int genAnchorNodeIdx(const int&);

    void addAllSessionsToGraph();
    void initTrajectoryByAnchoring(const Session& _sess);
    void addSessionToCentralGraph(const Session& _sess);

    void detectInterSessionSCloops();
    void detectInterSessionRSloops();
    int detectPreviousSessionSCloops(pcl::PointCloud<PointType>::Ptr laserCloudRawDS);

    void addSCloops();
    std::vector<std::pair<int, int>> equisampleElements(const std::vector<std::pair<int, int>>& _input_pair, float _gap, int _num);

    double calcInformationGainBtnTwoNodes(const int loop_idx_target_session, const int loop_idx_source_session);

    void findNearestRSLoopsTargetNodeIdx();
    bool addRSloops();
    
    void addFactorsWrtInfoGain();

    void optimizeMultisesseionGraph(bool _toOpt);
    void updateSessionsPoses();

    std::optional<gtsam::Pose3> doICPVirtualRelative(Session& target_sess, Session& source_sess, 
                        const int& loop_idx_target_session, const int& loop_idx_source_session);
    std::optional<gtsam::Pose3> doPreviousICPVirtualRelative(
                        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud, 
                        const int& loop_idx_prev_session);
    std::optional<gtsam::Pose3> doICPGlobalRelative(Session& target_sess, Session& source_sess, 
                        const int& loop_idx_target_session, const int& loop_idx_source_session);
    std::optional<gtsam::Pose3> doPreviousICPGlobalRelative(
                        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud, 
                        const int& loop_idx_prev_session);

    // saver 
    gtsam::Pose3 getPoseOfIsamUsingKey(Key _key);
    void writeAllSessionsTrajectories(std::string _postfix);
    
}; // LTslam