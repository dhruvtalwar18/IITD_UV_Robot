package org.bytedeco.javacpp.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(inherit = {opencv_imgproc.class, opencv_cudaarithm.class}, target = "org.bytedeco.javacpp.opencv_cudafilters", value = {@Platform(extension = {"-gpu"}, include = {"<opencv2/cudafilters.hpp>"}, link = {"opencv_cudafilters@.4.0"}), @Platform(extension = {"-gpu"}, link = {"opencv_cudafilters401"}, value = {"windows"})})
public class opencv_cudafilters implements InfoMapper {
    public void map(InfoMap infoMap) {
    }
}
