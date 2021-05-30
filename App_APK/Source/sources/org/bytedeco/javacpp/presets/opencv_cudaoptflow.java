package org.bytedeco.javacpp.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(inherit = {opencv_calib3d.class, opencv_objdetect.class, opencv_optflow.class, opencv_video.class, opencv_videoio.class, opencv_cudaimgproc.class, opencv_cudawarping.class}, target = "org.bytedeco.javacpp.opencv_cudaoptflow", value = {@Platform(extension = {"-gpu"}, include = {"<opencv2/cudaoptflow.hpp>"}, link = {"opencv_cudaoptflow@.4.0"}, preload = {"opencv_cudalegacy@.4.0"}), @Platform(extension = {"-gpu"}, link = {"opencv_cudaoptflow401"}, preload = {"opencv_cudalegacy401"}, value = {"windows"})})
public class opencv_cudaoptflow implements InfoMapper {
    public void map(InfoMap infoMap) {
    }
}
