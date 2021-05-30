package org.bytedeco.javacpp.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(inherit = {opencv_ximgproc.class, opencv_video.class}, target = "org.bytedeco.javacpp.opencv_optflow", value = {@Platform(include = {"<opencv2/optflow.hpp>", "<opencv2/optflow/motempl.hpp>"}, link = {"opencv_optflow@.4.0"}), @Platform(preload = {"libopencv_optflow"}, value = {"ios"}), @Platform(link = {"opencv_optflow401"}, value = {"windows"})})
public class opencv_optflow implements InfoMapper {
    public void map(InfoMap infoMap) {
    }
}
