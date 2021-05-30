package org.bytedeco.javacpp.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(inherit = {opencv_features2d.class}, target = "org.bytedeco.javacpp.opencv_saliency", value = {@Platform(include = {"<opencv2/saliency.hpp>", "<opencv2/saliency/saliencyBaseClasses.hpp>", "<opencv2/saliency/saliencySpecializedClasses.hpp>"}, link = {"opencv_saliency@.4.0"}), @Platform(preload = {"libopencv_saliency"}, value = {"ios"}), @Platform(link = {"opencv_saliency401"}, value = {"windows"})})
public class opencv_saliency implements InfoMapper {
    public void map(InfoMap infoMap) {
    }
}
