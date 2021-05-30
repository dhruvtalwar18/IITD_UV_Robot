package org.bytedeco.javacpp.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.Info;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(inherit = {opencv_objdetect.class, opencv_optflow.class}, target = "org.bytedeco.javacpp.opencv_superres", value = {@Platform(include = {"<opencv2/superres.hpp>", "<opencv2/superres/optical_flow.hpp>"}, link = {"opencv_superres@.4.0"}, not = {"ios"}, preload = {"opencv_cuda@.4.0", "opencv_cudacodec@.4.0", "opencv_cudaarithm@.4.0", "opencv_cudafilters@.4.0", "opencv_cudaimgproc@.4.0", "opencv_cudafeatures2d@.4.0", "opencv_cudalegacy@.4.0", "opencv_cudaoptflow@.4.0", "opencv_cudawarping@.4.0"}), @Platform(link = {"opencv_superres401"}, preload = {"opencv_cuda401", "opencv_cudacodec401", "opencv_cudaarithm401", "opencv_cudafilters401", "opencv_cudaimgproc401", "opencv_cudafeatures2d401", "opencv_cudalegacy401", "opencv_cudaoptflow401", "opencv_cudawarping401"}, value = {"windows"})})
public class opencv_superres implements InfoMapper {
    public void map(InfoMap infoMap) {
        infoMap.put(new Info("override").annotations(new String[0])).put(new Info("cv::superres::FarnebackOpticalFlow").pointerTypes("SuperResFarnebackOpticalFlow")).put(new Info("cv::superres::DualTVL1OpticalFlow").pointerTypes("SuperResDualTVL1OpticalFlow"));
    }
}
