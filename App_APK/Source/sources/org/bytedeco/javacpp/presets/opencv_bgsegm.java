package org.bytedeco.javacpp.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(inherit = {opencv_video.class}, target = "org.bytedeco.javacpp.opencv_bgsegm", value = {@Platform(include = {"<opencv2/bgsegm.hpp>"}, link = {"opencv_bgsegm@.4.0"}), @Platform(preload = {"libopencv_bgsegm"}, value = {"ios"}), @Platform(link = {"opencv_bgsegm401"}, value = {"windows"})})
public class opencv_bgsegm implements InfoMapper {
    public void map(InfoMap infoMap) {
    }
}
