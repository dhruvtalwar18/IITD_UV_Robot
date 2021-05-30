package org.bytedeco.javacpp.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.Info;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(inherit = {opencv_cudafilters.class}, target = "org.bytedeco.javacpp.opencv_cudaimgproc", value = {@Platform(extension = {"-gpu"}, include = {"<opencv2/cudaimgproc.hpp>"}, link = {"opencv_cudaimgproc@.4.0"}), @Platform(extension = {"-gpu"}, link = {"opencv_cudaimgproc401"}, value = {"windows"})})
public class opencv_cudaimgproc implements InfoMapper {
    public void map(InfoMap infoMap) {
        infoMap.put(new Info("cv::cuda::CLAHE").pointerTypes("CudaCLAHE"));
    }
}
