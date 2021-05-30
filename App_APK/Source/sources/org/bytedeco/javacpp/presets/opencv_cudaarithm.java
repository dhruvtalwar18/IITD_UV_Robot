package org.bytedeco.javacpp.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(inherit = {opencv_core.class}, target = "org.bytedeco.javacpp.opencv_cudaarithm", value = {@Platform(extension = {"-gpu"}, include = {"<opencv2/cudaarithm.hpp>"}, link = {"opencv_cudaarithm@.4.0"}), @Platform(extension = {"-gpu"}, link = {"opencv_cudaarithm401"}, value = {"windows"})})
public class opencv_cudaarithm implements InfoMapper {
    public void map(InfoMap infoMap) {
    }
}
