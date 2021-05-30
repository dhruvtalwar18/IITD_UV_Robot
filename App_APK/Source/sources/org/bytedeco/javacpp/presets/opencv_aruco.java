package org.bytedeco.javacpp.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.Info;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(inherit = {opencv_calib3d.class}, target = "org.bytedeco.javacpp.opencv_aruco", value = {@Platform(include = {"<opencv2/aruco/dictionary.hpp>", "<opencv2/aruco.hpp>", "<opencv2/aruco/charuco.hpp>"}, link = {"opencv_aruco@.4.0"}), @Platform(preload = {"libopencv_aruco"}, value = {"ios"}), @Platform(link = {"opencv_aruco401"}, value = {"windows"})})
public class opencv_aruco implements InfoMapper {
    public void map(InfoMap infoMap) {
        infoMap.put(new Info("cv::aruco::_drawPlanarBoardImpl").skip());
    }
}
