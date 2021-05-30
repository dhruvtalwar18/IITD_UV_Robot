package org.bytedeco.javacpp.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(inherit = {opencv_imgproc.class}, target = "org.bytedeco.javacpp.opencv_cudawarping", value = {@Platform(extension = {"-gpu"}, include = {"<opencv2/cudawarping.hpp>"}, link = {"opencv_cudawarping@.4.0"}), @Platform(extension = {"-gpu"}, link = {"opencv_cudawarping401"}, value = {"windows"})})
public class opencv_cudawarping implements InfoMapper {
    public void map(InfoMap infoMap) {
    }
}
