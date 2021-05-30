package org.bytedeco.javacpp.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.Info;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(inherit = {opencv_objdetect.class, opencv_optflow.class, opencv_photo.class}, target = "org.bytedeco.javacpp.opencv_videostab", value = {@Platform(include = {"<opencv2/videostab/frame_source.hpp>", "<opencv2/videostab/log.hpp>", "<opencv2/videostab/fast_marching.hpp>", "<opencv2/videostab/optical_flow.hpp>", "<opencv2/videostab/motion_core.hpp>", "<opencv2/videostab/outlier_rejection.hpp>", "<opencv2/videostab/global_motion.hpp>", "<opencv2/videostab/motion_stabilizing.hpp>", "<opencv2/videostab/inpainting.hpp>", "<opencv2/videostab/deblurring.hpp>", "<opencv2/videostab/wobble_suppression.hpp>", "<opencv2/videostab/stabilizer.hpp>", "<opencv2/videostab/ring_buffer.hpp>", "<opencv2/videostab.hpp>"}, link = {"opencv_videostab@.4.0"}, preload = {"opencv_cuda@.4.0", "opencv_cudaarithm@.4.0", "opencv_cudafilters@.4.0", "opencv_cudaimgproc@.4.0", "opencv_cudafeatures2d@.4.0", "opencv_cudalegacy@.4.0", "opencv_cudaoptflow@.4.0", "opencv_cudawarping@.4.0"}), @Platform(preload = {"libopencv_videostab"}, value = {"ios"}), @Platform(link = {"opencv_videostab401"}, preload = {"opencv_cuda401", "opencv_cudaarithm401", "opencv_cudafilters401", "opencv_cudaimgproc401", "opencv_cudafeatures2d401", "opencv_cudalegacy401", "opencv_cudaoptflow401", "opencv_cudawarping401"}, value = {"windows"})})
public class opencv_videostab implements InfoMapper {
    public void map(InfoMap infoMap) {
        infoMap.put(new Info("override").annotations(new String[0])).put(new Info("cv::videostab::IFrameSource").virtualize());
    }
}
