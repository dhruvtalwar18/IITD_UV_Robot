package org.bytedeco.javacpp.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(inherit = {opencv_imgproc.class}, target = "org.bytedeco.javacpp.opencv_photo", value = {@Platform(include = {"<opencv2/photo.hpp>", "<opencv2/photo/cuda.hpp>"}, link = {"opencv_photo@.4.0"}, preload = {"opencv_cuda@.4.0", "opencv_cudaarithm@.4.0", "opencv_cudafilters@.4.0", "opencv_cudaimgproc@.4.0"}), @Platform(preload = {"libopencv_photo"}, value = {"ios"}), @Platform(link = {"opencv_photo401"}, preload = {"opencv_cuda401", "opencv_cudaarithm401", "opencv_cudafilters401", "opencv_cudaimgproc401"}, value = {"windows"})})
public class opencv_photo implements InfoMapper {
    public void map(InfoMap infoMap) {
    }
}
