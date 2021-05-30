package org.bytedeco.javacpp.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.Info;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(helper = "org.bytedeco.javacpp.helper.opencv_ml", inherit = {opencv_core.class}, target = "org.bytedeco.javacpp.opencv_ml", value = {@Platform(include = {"<opencv2/ml.hpp>"}, link = {"opencv_ml@.4.0"}), @Platform(preload = {"libopencv_ml"}, value = {"ios"}), @Platform(link = {"opencv_ml401"}, value = {"windows"})})
public class opencv_ml implements InfoMapper {
    public void map(InfoMap infoMap) {
        infoMap.put(new Info("CV_DOXYGEN").define(false)).put(new Info("cv::ml::StatModel").base("AbstractStatModel")).put(new Info("cv::ml::SVM::getDefaultGrid").javaNames("getDefaultGrid")).put(new Info("cv::ml::randGaussMixture").skip());
    }
}
