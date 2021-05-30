package org.bytedeco.javacpp.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.Info;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(inherit = {opencv_dnn.class, opencv_features2d.class, opencv_ml.class}, target = "org.bytedeco.javacpp.opencv_text", value = {@Platform(include = {"<opencv2/text.hpp>", "<opencv2/text/erfilter.hpp>", "<opencv2/text/ocr.hpp>", "opencv2/text/textDetector.hpp"}, link = {"opencv_text@.4.0"}), @Platform(preload = {"libopencv_text"}, value = {"ios"}), @Platform(link = {"opencv_text401"}, value = {"windows"})})
public class opencv_text implements InfoMapper {
    public void map(InfoMap infoMap) {
        infoMap.put(new Info().javaText("import org.bytedeco.javacpp.annotation.Index;")).put(new Info("std::deque<int>").pointerTypes("IntDeque").define()).put(new Info("std::vector<cv::text::ERStat>").pointerTypes("ERStatVector").define()).put(new Info("std::vector<std::vector<cv::text::ERStat> >").pointerTypes("ERStatVectorVector").define()).put(new Info("std::vector<double>").pointerTypes("DoubleVector").define()).put(new Info("std::vector<std::string>").pointerTypes("StdStringVector").define()).put(new Info("std::vector<cv::Vec2i>").pointerTypes("PointVector").cast()).put(new Info("std::vector<std::vector<cv::Vec2i> >").pointerTypes("PointVectorVector").cast()).put(new Info("cv::text::OCRBeamSearchDecoder::create").skipDefaults());
    }
}
