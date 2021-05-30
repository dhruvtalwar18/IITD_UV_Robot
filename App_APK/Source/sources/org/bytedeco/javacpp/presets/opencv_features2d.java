package org.bytedeco.javacpp.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(inherit = {opencv_highgui.class, opencv_flann.class}, target = "org.bytedeco.javacpp.opencv_features2d", value = {@Platform(include = {"<opencv2/features2d.hpp>"}, link = {"opencv_features2d@.4.0"}), @Platform(preload = {"libopencv_features2d"}, value = {"ios"}), @Platform(link = {"opencv_features2d401"}, value = {"windows"})})
public class opencv_features2d implements InfoMapper {
    public void map(InfoMap infoMap) {
    }
}
