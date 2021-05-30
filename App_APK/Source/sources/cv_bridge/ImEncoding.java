package cv_bridge;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Vector;
import org.bytedeco.javacpp.opencv_core;
import org.xbill.DNS.TTL;
import sensor_msgs.ImageEncodings;

/* compiled from: Encoding */
class ImEncoding {
    protected static final int SAME_FORMAT = -1;

    ImEncoding() {
    }

    static int getCvType(String enc) throws Exception {
        String encoding = enc.toLowerCase();
        if (encoding.equals(ImageEncodings.BGR8)) {
            return opencv_core.CV_8UC3;
        }
        if (encoding.equals(ImageEncodings.MONO8)) {
            return opencv_core.CV_8UC1;
        }
        if (encoding.equals(ImageEncodings.RGB8)) {
            return opencv_core.CV_8UC3;
        }
        if (encoding.equals(ImageEncodings.MONO16)) {
            return opencv_core.CV_16UC1;
        }
        if (encoding.equals(ImageEncodings.BGR16)) {
            return opencv_core.CV_16UC3;
        }
        if (encoding.equals(ImageEncodings.RGB16)) {
            return opencv_core.CV_16UC3;
        }
        if (encoding.equals(ImageEncodings.BGRA8)) {
            return opencv_core.CV_8UC4;
        }
        if (encoding.equals(ImageEncodings.RGBA8)) {
            return opencv_core.CV_8UC4;
        }
        if (encoding.equals(ImageEncodings.BGRA16)) {
            return opencv_core.CV_16UC4;
        }
        if (encoding.equals(ImageEncodings.RGBA16)) {
            return opencv_core.CV_16UC4;
        }
        if (encoding.equals(ImageEncodings.BAYER_RGGB8)) {
            return opencv_core.CV_8UC1;
        }
        if (encoding.equals(ImageEncodings.BAYER_BGGR8)) {
            return opencv_core.CV_8UC1;
        }
        if (encoding.equals(ImageEncodings.BAYER_GBRG8)) {
            return opencv_core.CV_8UC1;
        }
        if (encoding.equals(ImageEncodings.BAYER_GRBG8)) {
            return opencv_core.CV_8UC1;
        }
        if (encoding.equals(ImageEncodings.BAYER_RGGB16)) {
            return opencv_core.CV_16UC1;
        }
        if (encoding.equals(ImageEncodings.BAYER_BGGR16)) {
            return opencv_core.CV_16UC1;
        }
        if (encoding.equals(ImageEncodings.BAYER_GBRG16)) {
            return opencv_core.CV_16UC1;
        }
        if (encoding.equals(ImageEncodings.BAYER_GRBG16)) {
            return opencv_core.CV_16UC1;
        }
        if (encoding.equals(ImageEncodings.YUV422)) {
            return opencv_core.CV_8UC2;
        }
        String encoding2 = encoding.toUpperCase();
        if (encoding2.equals(ImageEncodings.TYPE_8UC1)) {
            return opencv_core.CV_8UC1;
        }
        if (encoding2.equals(ImageEncodings.TYPE_8UC2)) {
            return opencv_core.CV_8UC2;
        }
        if (encoding2.equals(ImageEncodings.TYPE_8UC3)) {
            return opencv_core.CV_8UC3;
        }
        if (encoding2.equals(ImageEncodings.TYPE_8UC4)) {
            return opencv_core.CV_8UC4;
        }
        if (encoding2.equals(ImageEncodings.TYPE_8SC1)) {
            return opencv_core.CV_8SC1;
        }
        if (encoding2.equals(ImageEncodings.TYPE_8SC2)) {
            return opencv_core.CV_8SC2;
        }
        if (encoding2.equals(ImageEncodings.TYPE_8SC3)) {
            return opencv_core.CV_8SC3;
        }
        if (encoding2.equals(ImageEncodings.TYPE_8SC4)) {
            return opencv_core.CV_8SC4;
        }
        if (encoding2.equals(ImageEncodings.TYPE_16UC1)) {
            return opencv_core.CV_16UC1;
        }
        if (encoding2.equals(ImageEncodings.TYPE_16UC2)) {
            return opencv_core.CV_16UC2;
        }
        if (encoding2.equals(ImageEncodings.TYPE_16UC3)) {
            return opencv_core.CV_16UC3;
        }
        if (encoding2.equals(ImageEncodings.TYPE_16UC4)) {
            return opencv_core.CV_16UC4;
        }
        if (encoding2.equals(ImageEncodings.TYPE_16SC1)) {
            return opencv_core.CV_16SC1;
        }
        if (encoding2.equals(ImageEncodings.TYPE_16SC2)) {
            return opencv_core.CV_16SC2;
        }
        if (encoding2.equals(ImageEncodings.TYPE_16SC3)) {
            return opencv_core.CV_16SC3;
        }
        if (encoding2.equals(ImageEncodings.TYPE_16SC4)) {
            return opencv_core.CV_16SC4;
        }
        if (encoding2.equals(ImageEncodings.TYPE_32SC1)) {
            return opencv_core.CV_32SC1;
        }
        if (encoding2.equals(ImageEncodings.TYPE_32SC2)) {
            return opencv_core.CV_32SC2;
        }
        if (encoding2.equals(ImageEncodings.TYPE_32SC3)) {
            return opencv_core.CV_32SC3;
        }
        if (encoding2.equals(ImageEncodings.TYPE_32SC4)) {
            return opencv_core.CV_32SC4;
        }
        if (encoding2.equals(ImageEncodings.TYPE_32FC1)) {
            return opencv_core.CV_32FC1;
        }
        if (encoding2.equals(ImageEncodings.TYPE_32FC2)) {
            return opencv_core.CV_32FC2;
        }
        if (encoding2.equals(ImageEncodings.TYPE_32FC3)) {
            return opencv_core.CV_32FC3;
        }
        if (encoding2.equals(ImageEncodings.TYPE_32FC4)) {
            return opencv_core.CV_32FC4;
        }
        if (encoding2.equals(ImageEncodings.TYPE_64FC1)) {
            return opencv_core.CV_64FC1;
        }
        if (encoding2.equals(ImageEncodings.TYPE_64FC2)) {
            return opencv_core.CV_64FC2;
        }
        if (encoding2.equals(ImageEncodings.TYPE_64FC3)) {
            return opencv_core.CV_64FC3;
        }
        if (encoding2.equals(ImageEncodings.TYPE_64FC4)) {
            return opencv_core.CV_64FC4;
        }
        throw new Exception("Unrecognized image encoding [" + encoding2 + "]");
    }

    protected static int safeLongToInt(long l) {
        if (l >= -2147483648L && l <= TTL.MAX_VALUE) {
            return (int) l;
        }
        throw new IllegalArgumentException(l + " cannot be cast to int without changing its value.");
    }

    protected static Map<Pair<Encoding, Encoding>, Vector<Integer>> getConversionCodes() {
        Map<Pair<Encoding, Encoding>, Vector<Integer>> res = new HashMap<>();
        for (int i = 0; i <= 5; i++) {
            res.put(new Pair(Encoding.valueOf(i), Encoding.valueOf(i)), new Vector(Arrays.asList(new Integer[]{-1})));
        }
        res.put(new Pair(Encoding.GRAY, Encoding.RGB), new Vector(Arrays.asList(new Integer[]{8})));
        res.put(new Pair(Encoding.GRAY, Encoding.BGR), new Vector(Arrays.asList(new Integer[]{8})));
        res.put(new Pair(Encoding.GRAY, Encoding.RGBA), new Vector(Arrays.asList(new Integer[]{9})));
        res.put(new Pair(Encoding.GRAY, Encoding.BGRA), new Vector(Arrays.asList(new Integer[]{9})));
        res.put(new Pair(Encoding.RGB, Encoding.GRAY), new Vector(Arrays.asList(new Integer[]{7})));
        res.put(new Pair(Encoding.RGB, Encoding.BGR), new Vector(Arrays.asList(new Integer[]{4})));
        res.put(new Pair(Encoding.RGB, Encoding.RGBA), new Vector(Arrays.asList(new Integer[]{0})));
        res.put(new Pair(Encoding.RGB, Encoding.BGRA), new Vector(Arrays.asList(new Integer[]{2})));
        res.put(new Pair(Encoding.BGR, Encoding.GRAY), new Vector(Arrays.asList(new Integer[]{6})));
        res.put(new Pair(Encoding.BGR, Encoding.RGB), new Vector(Arrays.asList(new Integer[]{4})));
        res.put(new Pair(Encoding.BGR, Encoding.RGBA), new Vector(Arrays.asList(new Integer[]{2})));
        res.put(new Pair(Encoding.BGR, Encoding.BGRA), new Vector(Arrays.asList(new Integer[]{0})));
        res.put(new Pair(Encoding.RGBA, Encoding.GRAY), new Vector(Arrays.asList(new Integer[]{11})));
        res.put(new Pair(Encoding.RGBA, Encoding.RGB), new Vector(Arrays.asList(new Integer[]{1})));
        res.put(new Pair(Encoding.RGBA, Encoding.BGR), new Vector(Arrays.asList(new Integer[]{3})));
        res.put(new Pair(Encoding.RGBA, Encoding.BGRA), new Vector(Arrays.asList(new Integer[]{5})));
        res.put(new Pair(Encoding.BGRA, Encoding.GRAY), new Vector(Arrays.asList(new Integer[]{10})));
        res.put(new Pair(Encoding.BGRA, Encoding.RGB), new Vector(Arrays.asList(new Integer[]{3})));
        res.put(new Pair(Encoding.BGRA, Encoding.BGR), new Vector(Arrays.asList(new Integer[]{1})));
        res.put(new Pair(Encoding.BGRA, Encoding.RGBA), new Vector(Arrays.asList(new Integer[]{5})));
        res.put(new Pair(Encoding.YUV422, Encoding.GRAY), new Vector(Arrays.asList(new Integer[]{123})));
        res.put(new Pair(Encoding.YUV422, Encoding.RGB), new Vector(Arrays.asList(new Integer[]{107})));
        res.put(new Pair(Encoding.YUV422, Encoding.BGR), new Vector(Arrays.asList(new Integer[]{108})));
        res.put(new Pair(Encoding.YUV422, Encoding.RGBA), new Vector(Arrays.asList(new Integer[]{111})));
        res.put(new Pair(Encoding.YUV422, Encoding.BGRA), new Vector(Arrays.asList(new Integer[]{112})));
        res.put(new Pair(Encoding.BAYER_RGGB, Encoding.GRAY), new Vector(Arrays.asList(new Integer[]{86})));
        res.put(new Pair(Encoding.BAYER_RGGB, Encoding.RGB), new Vector(Arrays.asList(new Integer[]{48})));
        res.put(new Pair(Encoding.BAYER_RGGB, Encoding.BGR), new Vector(Arrays.asList(new Integer[]{46})));
        res.put(new Pair(Encoding.BAYER_BGGR, Encoding.GRAY), new Vector(Arrays.asList(new Integer[]{88})));
        res.put(new Pair(Encoding.BAYER_BGGR, Encoding.RGB), new Vector(Arrays.asList(new Integer[]{46})));
        res.put(new Pair(Encoding.BAYER_BGGR, Encoding.BGR), new Vector(Arrays.asList(new Integer[]{48})));
        res.put(new Pair(Encoding.BAYER_GBRG, Encoding.GRAY), new Vector(Arrays.asList(new Integer[]{89})));
        res.put(new Pair(Encoding.BAYER_GBRG, Encoding.RGB), new Vector(Arrays.asList(new Integer[]{47})));
        res.put(new Pair(Encoding.BAYER_GBRG, Encoding.BGR), new Vector(Arrays.asList(new Integer[]{49})));
        res.put(new Pair(Encoding.BAYER_GRBG, Encoding.GRAY), new Vector(Arrays.asList(new Integer[]{87})));
        res.put(new Pair(Encoding.BAYER_GRBG, Encoding.RGB), new Vector(Arrays.asList(new Integer[]{49})));
        res.put(new Pair(Encoding.BAYER_GRBG, Encoding.BGR), new Vector(Arrays.asList(new Integer[]{47})));
        return res;
    }

    protected static Encoding getEncoding(String encoding) {
        String lEncoding = encoding.toLowerCase();
        if (lEncoding.equals(ImageEncodings.MONO8)) {
            return Encoding.GRAY;
        }
        if (lEncoding.equals(ImageEncodings.BGR8)) {
            return Encoding.BGR;
        }
        if (lEncoding.equals(ImageEncodings.RGB8)) {
            return Encoding.RGB;
        }
        if (lEncoding.equals(ImageEncodings.BGRA8)) {
            return Encoding.BGRA;
        }
        if (lEncoding.equals(ImageEncodings.RGBA8)) {
            return Encoding.RGBA;
        }
        if (lEncoding.equals(ImageEncodings.YUV422)) {
            return Encoding.YUV422;
        }
        if (lEncoding.equals(ImageEncodings.BAYER_RGGB8)) {
            return Encoding.BAYER_RGGB;
        }
        if (lEncoding.equals(ImageEncodings.BAYER_BGGR8)) {
            return Encoding.BAYER_BGGR;
        }
        if (lEncoding.equals(ImageEncodings.BAYER_GBRG8)) {
            return Encoding.BAYER_GBRG;
        }
        if (lEncoding.equals(ImageEncodings.BAYER_GRBG8)) {
            return Encoding.BAYER_GRBG;
        }
        return Encoding.INVALID;
    }

    protected static Vector<Integer> getConversionCode(String src_encoding, String dst_encoding) throws Exception {
        Encoding src_encode = getEncoding(src_encoding);
        Encoding dst_encode = getEncoding(dst_encoding);
        boolean is_num_channels_the_same = false;
        boolean is_src_color_format = ImageEncodings.isColor(src_encoding) || ImageEncodings.isMono(src_encoding) || ImageEncodings.isBayer(src_encoding) || src_encoding.toLowerCase().equals(ImageEncodings.YUV422);
        boolean is_dst_color_format = ImageEncodings.isColor(dst_encoding) || ImageEncodings.isMono(dst_encoding) || ImageEncodings.isBayer(dst_encoding) || dst_encoding.toLowerCase().equals(ImageEncodings.YUV422);
        if (ImageEncodings.numChannels(src_encoding) == ImageEncodings.numChannels(dst_encoding)) {
            is_num_channels_the_same = true;
        }
        if (!is_src_color_format) {
            if (is_dst_color_format) {
                throw new Exception("[" + src_encoding + "] is not a color format. but [" + dst_encoding + "] is. The conversion does not make sense");
            } else if (is_num_channels_the_same) {
                return new Vector<>(1, -1);
            } else {
                throw new Exception("[" + src_encoding + "] and [" + dst_encoding + "] do not have the same number of channel");
            }
        } else if (is_dst_color_format) {
            Vector<Integer> res = getConversionCodes().get(new Pair<>(src_encode, dst_encode));
            if (res != null) {
                if (!(ImageEncodings.bitDepth(src_encoding) == ImageEncodings.bitDepth(dst_encoding) || getEncoding(src_encoding) == getEncoding(dst_encoding))) {
                    res.add(-1);
                }
                return res;
            }
            throw new Exception("Unsupported conversion from [" + src_encoding + "] to [" + dst_encoding + "]");
        } else if (is_num_channels_the_same) {
            return new Vector<>(1, -1);
        } else {
            throw new Exception("[" + src_encoding + "] is a color format but [" + dst_encoding + "] is not so they must have the same OpenCV type, CV_8UC3, CV16UC1 ....");
        }
    }
}
