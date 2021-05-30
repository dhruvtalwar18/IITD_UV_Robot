package org.bytedeco.javacpp.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(inherit = {opencv_photo.class}, target = "org.bytedeco.javacpp.opencv_xphoto", value = {@Platform(include = {"<opencv2/xphoto.hpp>", "<opencv2/xphoto/inpainting.hpp>", "<opencv2/xphoto/white_balance.hpp>", "<opencv2/xphoto/dct_image_denoising.hpp>", "<opencv2/xphoto/bm3d_image_denoising.hpp>", "<opencv2/xphoto/oilpainting.hpp>", "<opencv2/xphoto/tonemap.hpp>"}, link = {"opencv_xphoto@.4.0"}), @Platform(preload = {"libopencv_xphoto"}, value = {"ios"}), @Platform(link = {"opencv_xphoto401"}, value = {"windows"})})
public class opencv_xphoto implements InfoMapper {
    public void map(InfoMap infoMap) {
    }
}
