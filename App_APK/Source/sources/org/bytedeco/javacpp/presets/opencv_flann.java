package org.bytedeco.javacpp.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.Info;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(inherit = {opencv_core.class}, target = "org.bytedeco.javacpp.opencv_flann", value = {@Platform(include = {"<opencv2/flann/defines.h>", "<opencv2/flann/miniflann.hpp>"}, link = {"opencv_flann@.4.0"}), @Platform(preload = {"libopencv_flann"}, value = {"ios"}), @Platform(link = {"opencv_flann401"}, value = {"windows"})})
public class opencv_flann implements InfoMapper {
    public void map(InfoMap infoMap) {
        infoMap.put(new Info("FLANN_EXPORT", "FLANN_DEPRECATED").cppTypes(new String[0]));
    }
}
