package org.bytedeco.javacpp;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;

@Properties(inherit = {opencv_aruco.class, opencv_bgsegm.class, opencv_bioinspired.class, opencv_face.class, opencv_img_hash.class, opencv_structured_light.class, opencv_text.class, opencv_tracking.class, opencv_xfeatures2d.class, opencv_ximgproc.class, opencv_xphoto.class}, value = {@Platform(preload = {"opencv_cuda@.4.0", "opencv_cudaarithm@.4.0", "opencv_cudafilters@.4.0", "opencv_cudaimgproc@.4.0", "opencv_java"}), @Platform(preload = {"libopencv_java"}, value = {"ios"}), @Platform(preload = {"opencv_cuda400", "opencv_cudaarithm400", "opencv_cudafilters400", "opencv_cudaimgproc400", "opencv_java"}, value = {"windows"})})
public class opencv_java {
    static {
        Loader.load();
    }
}
