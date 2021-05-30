package org.bytedeco.javacpp.presets;

import org.apache.xmlrpc.serializer.DoubleSerializer;
import org.apache.xmlrpc.serializer.I4Serializer;
import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.Info;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(inherit = {opencv_imgcodecs.class}, target = "org.bytedeco.javacpp.opencv_videoio", value = {@Platform(include = {"<opencv2/videoio.hpp>"}, link = {"opencv_videoio@.4.0"}), @Platform(preload = {"native_camera_r2.2.0", "native_camera_r2.3.4", "native_camera_r3.0.1", "native_camera_r4.0.0", "native_camera_r4.0.3", "native_camera_r4.1.1", "native_camera_r4.2.0", "native_camera_r4.3.0", "native_camera_r4.4.0"}, value = {"android"}), @Platform(preload = {"libopencv_videoio"}, value = {"ios"}), @Platform(link = {"opencv_videoio401"}, preload = {"opencv_ffmpeg401", "opencv_ffmpeg401_64"}, value = {"windows"})})
public class opencv_videoio implements InfoMapper {
    public void map(InfoMap infoMap) {
        infoMap.put(new Info("CV_FOURCC_DEFAULT").cppTypes(I4Serializer.INT_TAG)).put(new Info("cvCaptureFromFile", "cvCaptureFromAVI").cppTypes("CvCapture*", "const char*")).put(new Info("cvCaptureFromCAM").cppTypes("CvCapture*", I4Serializer.INT_TAG)).put(new Info("cvCreateAVIWriter").cppTypes("CvVideoWriter*", "const char*", I4Serializer.INT_TAG, DoubleSerializer.DOUBLE_TAG, "CvSize", I4Serializer.INT_TAG)).put(new Info("cvWriteToAVI").cppTypes(I4Serializer.INT_TAG, "CvVideoWriter*", "IplImage*")).put(new Info("cv::DefaultDeleter<CvCapture>").pointerTypes("CvCaptureDefaultDeleter")).put(new Info("cv::DefaultDeleter<CvVideoWriter>").pointerTypes("CvVideoWriterDefaultDeleter"));
    }
}
