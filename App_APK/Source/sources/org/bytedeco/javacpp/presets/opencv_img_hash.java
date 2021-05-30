package org.bytedeco.javacpp.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(inherit = {opencv_imgproc.class}, target = "org.bytedeco.javacpp.opencv_img_hash", value = {@Platform(include = {"<opencv2/img_hash.hpp>", "<opencv2/img_hash/img_hash_base.hpp>", "<opencv2/img_hash/average_hash.hpp>", "<opencv2/img_hash/block_mean_hash.hpp>", "<opencv2/img_hash/color_moment_hash.hpp>", "<opencv2/img_hash/marr_hildreth_hash.hpp>", "<opencv2/img_hash/phash.hpp>", "<opencv2/img_hash/radial_variance_hash.hpp>"}, link = {"opencv_img_hash@.4.0"}), @Platform(preload = {"libopencv_img_hash"}, value = {"ios"}), @Platform(link = {"opencv_img_hash401"}, value = {"windows"})})
public class opencv_img_hash implements InfoMapper {
    public void map(InfoMap infoMap) {
    }
}
