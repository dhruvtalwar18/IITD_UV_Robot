package org.bytedeco.javacpp.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(inherit = {opencv_imgproc.class}, target = "org.bytedeco.javacpp.opencv_phase_unwrapping", value = {@Platform(include = {"<opencv2/phase_unwrapping.hpp>", "<opencv2/phase_unwrapping/phase_unwrapping.hpp>", "<opencv2/phase_unwrapping/histogramphaseunwrapping.hpp>"}, link = {"opencv_phase_unwrapping@.4.0"}), @Platform(preload = {"libopencv_phase_unwrapping"}, value = {"ios"}), @Platform(link = {"opencv_phase_unwrapping401"}, value = {"windows"})})
public class opencv_phase_unwrapping implements InfoMapper {
    public void map(InfoMap infoMap) {
    }
}
