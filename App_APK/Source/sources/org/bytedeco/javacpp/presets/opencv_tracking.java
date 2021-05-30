package org.bytedeco.javacpp.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.Info;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(inherit = {opencv_plot.class, opencv_video.class, opencv_dnn.class}, target = "org.bytedeco.javacpp.opencv_tracking", value = {@Platform(include = {"<opencv2/tracking.hpp>", "<opencv2/tracking/feature.hpp>", "<opencv2/tracking/kalman_filters.hpp>", "<opencv2/tracking/onlineMIL.hpp>", "<opencv2/tracking/onlineBoosting.hpp>", "<opencv2/tracking/tldDataset.hpp>", "<opencv2/tracking/tracker.hpp>"}, link = {"opencv_tracking@.4.0"}), @Platform(preload = {"libopencv_tracking"}, value = {"ios"}), @Platform(link = {"opencv_tracking401"}, value = {"windows"})})
public class opencv_tracking implements InfoMapper {
    public void map(InfoMap infoMap) {
        infoMap.put(new Info().javaText("import org.bytedeco.javacpp.annotation.Index;")).put(new Info("override").annotations(new String[0])).put(new Info("cv::Ptr<cv::Tracker>").annotations("@Ptr").pointerTypes("Tracker")).put(new Info("cv::Ptr<cv::TrackerFeature>").annotations("@Ptr").pointerTypes("TrackerFeature")).put(new Info("cv::Ptr<cv::TrackerTargetState>").annotations("@Ptr").pointerTypes("TrackerTargetState")).put(new Info("cv::Ptr<cv::TrackerSamplerAlgorithm>").annotations("@Ptr").pointerTypes("TrackerSamplerAlgorithm")).put(new Info("std::vector<cv::Ptr<cv::Tracker> >").pointerTypes("TrackerVector").define()).put(new Info("std::vector<cv::ConfidenceMap>").pointerTypes("ConfidenceMapVector").define()).put(new Info("std::vector<std::pair<cv::Ptr<cv::TrackerTargetState>,float> >").pointerTypes("ConfidenceMap").define()).put(new Info("std::vector<std::pair<cv::String,cv::Ptr<cv::TrackerFeature> > >").pointerTypes("StringTrackerFeaturePairVector").define()).put(new Info("std::vector<std::pair<cv::String,cv::Ptr<cv::TrackerSamplerAlgorithm> > >").pointerTypes("StringTrackerSamplerAlgorithmPairVector").define()).put(new Info("std::vector<cv::Ptr<cv::TrackerTargetState> >").pointerTypes("Trajectory").define()).put(new Info("cv::CvHaarEvaluator::setWinSize").annotations("@Function")).put(new Info("cv::TrackerMIL", "cv::TrackerBoosting", "cv::TrackerMedianFlow", "cv::TrackerTLD", "cv::TrackerGOTURN", "cv::TrackerMOSSE").purify());
    }
}
