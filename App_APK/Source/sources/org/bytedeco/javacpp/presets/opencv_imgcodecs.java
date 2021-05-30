package org.bytedeco.javacpp.presets;

import org.apache.xmlrpc.serializer.I4Serializer;
import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.Info;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(helper = "org.bytedeco.javacpp.helper.opencv_imgcodecs", inherit = {opencv_imgproc.class}, target = "org.bytedeco.javacpp.opencv_imgcodecs", value = {@Platform(include = {"<opencv2/imgcodecs.hpp>"}, link = {"opencv_imgcodecs@.4.0"}), @Platform(preload = {"libopencv_imgcodecs"}, value = {"ios"}), @Platform(link = {"opencv_imgcodecs401"}, value = {"windows"})})
public class opencv_imgcodecs implements InfoMapper {
    public void map(InfoMap infoMap) {
        infoMap.put(new Info("cvvLoadImage").cppTypes("IplImage*", "const char*")).put(new Info("cvvSaveImage").cppTypes(I4Serializer.INT_TAG, "const char*", "CvArr*", "int*")).put(new Info("cvvConvertImage").cppTypes("void", "CvArr*", "CvArr*", I4Serializer.INT_TAG));
    }
}
