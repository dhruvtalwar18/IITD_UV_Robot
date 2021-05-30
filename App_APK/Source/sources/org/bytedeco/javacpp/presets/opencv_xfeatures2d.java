package org.bytedeco.javacpp.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.Info;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(inherit = {opencv_ml.class, opencv_shape.class}, target = "org.bytedeco.javacpp.opencv_xfeatures2d", value = {@Platform(include = {"<opencv2/xfeatures2d.hpp>", "<opencv2/xfeatures2d/nonfree.hpp>"}, link = {"opencv_xfeatures2d@.4.0"}, preload = {"opencv_cuda@.4.0", "opencv_cudaarithm@.4.0"}), @Platform(preload = {"libopencv_xfeatures2d"}, value = {"ios"}), @Platform(link = {"opencv_xfeatures2d401"}, preload = {"opencv_cuda401", "opencv_cudaarithm401"}, value = {"windows"})})
public class opencv_xfeatures2d implements InfoMapper {
    public void map(InfoMap infoMap) {
        infoMap.put(new Info("cv::Matx23f").cast().pointerTypes("FloatPointer"));
    }
}
