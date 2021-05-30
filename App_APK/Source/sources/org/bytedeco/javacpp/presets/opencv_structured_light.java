package org.bytedeco.javacpp.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(inherit = {opencv_phase_unwrapping.class, opencv_calib3d.class}, target = "org.bytedeco.javacpp.opencv_structured_light", value = {@Platform(include = {"<opencv2/structured_light.hpp>", "<opencv2/structured_light/structured_light.hpp>", "<opencv2/structured_light/graycodepattern.hpp>", "<opencv2/structured_light/sinusoidalpattern.hpp>"}, link = {"opencv_structured_light@.4.0"}), @Platform(preload = {"libopencv_structured_light"}, value = {"ios"}), @Platform(link = {"opencv_structured_light401"}, value = {"windows"})})
public class opencv_structured_light implements InfoMapper {
    public void map(InfoMap infoMap) {
    }
}
