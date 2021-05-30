package sensor_msgs;

public class ImageEncodings {
    public static final String BAYER_BGGR16 = "bayer_bggr16";
    public static final String BAYER_BGGR8 = "bayer_bggr8";
    public static final String BAYER_GBRG16 = "bayer_gbrg16";
    public static final String BAYER_GBRG8 = "bayer_gbrg8";
    public static final String BAYER_GRBG16 = "bayer_grbg16";
    public static final String BAYER_GRBG8 = "bayer_grbg8";
    public static final String BAYER_RGGB16 = "bayer_rggb16";
    public static final String BAYER_RGGB8 = "bayer_rggb8";
    public static final String BGR16 = "bgr16";
    public static final String BGR8 = "bgr8";
    public static final String BGRA16 = "bgra16";
    public static final String BGRA8 = "bgra8";
    public static final String MONO16 = "mono16";
    public static final String MONO8 = "mono8";
    public static final String RGB16 = "rgb16";
    public static final String RGB8 = "rgb8";
    public static final String RGBA16 = "rgba16";
    public static final String RGBA8 = "rgba8";
    public static final String TYPE_16SC1 = "16SC1";
    public static final String TYPE_16SC2 = "16SC2";
    public static final String TYPE_16SC3 = "16SC3";
    public static final String TYPE_16SC4 = "16SC4";
    public static final String TYPE_16UC1 = "16UC1";
    public static final String TYPE_16UC2 = "16UC2";
    public static final String TYPE_16UC3 = "16UC3";
    public static final String TYPE_16UC4 = "16UC4";
    public static final String TYPE_32FC1 = "32FC1";
    public static final String TYPE_32FC2 = "32FC2";
    public static final String TYPE_32FC3 = "32FC3";
    public static final String TYPE_32FC4 = "32FC4";
    public static final String TYPE_32SC1 = "32SC1";
    public static final String TYPE_32SC2 = "32SC2";
    public static final String TYPE_32SC3 = "32SC3";
    public static final String TYPE_32SC4 = "32SC4";
    public static final String TYPE_64FC1 = "64FC1";
    public static final String TYPE_64FC2 = "64FC2";
    public static final String TYPE_64FC3 = "64FC3";
    public static final String TYPE_64FC4 = "64FC4";
    public static final String TYPE_8SC1 = "8SC1";
    public static final String TYPE_8SC2 = "8SC2";
    public static final String TYPE_8SC3 = "8SC3";
    public static final String TYPE_8SC4 = "8SC4";
    public static final String TYPE_8UC1 = "8UC1";
    public static final String TYPE_8UC2 = "8UC2";
    public static final String TYPE_8UC3 = "8UC3";
    public static final String TYPE_8UC4 = "8UC4";
    public static final String YUV422 = "yuv422";

    public static boolean isColor(String encoding) {
        String lEncoding = encoding.toLowerCase();
        return lEncoding.equals(RGB8) || lEncoding.equals(BGR8) || lEncoding.equals(RGBA8) || lEncoding.equals(BGRA8) || lEncoding.equals(RGB16) || lEncoding.equals(BGR16) || lEncoding.equals(RGBA16) || lEncoding.equals(BGRA16);
    }

    public static boolean isMono(String encoding) {
        String lEncoding = encoding.toLowerCase();
        return lEncoding.equals(MONO8) || lEncoding.equals(MONO16);
    }

    public static boolean isBayer(String encoding) {
        String lEncoding = encoding.toLowerCase();
        return lEncoding.equals(BAYER_RGGB8) || lEncoding.equals(BAYER_BGGR8) || lEncoding.equals(BAYER_GBRG8) || lEncoding.equals(BAYER_GRBG8) || lEncoding.equals(BAYER_RGGB16) || lEncoding.equals(BAYER_BGGR16) || lEncoding.equals(BAYER_GBRG16) || lEncoding.equals(BAYER_GRBG16);
    }

    public static boolean hasAlpha(String encoding) {
        String lEncoding = encoding.toLowerCase();
        return lEncoding.equals(RGBA8) || lEncoding.equals(BGRA8) || lEncoding.equals(RGBA16) || lEncoding.equals(BGRA16);
    }

    public static int numChannels(String enc) throws Exception {
        String encoding = enc.toLowerCase();
        if (encoding.equals(MONO8) || encoding.equals(MONO16)) {
            return 1;
        }
        if (encoding.equals(BGR8) || encoding.equals(RGB8) || encoding.equals(BGR16) || encoding.equals(RGB16)) {
            return 3;
        }
        if (encoding.equals(BGRA8) || encoding.equals(RGBA8) || encoding.equals(BGRA16) || encoding.equals(RGBA16)) {
            return 4;
        }
        if (encoding.equals(BAYER_RGGB8) || encoding.equals(BAYER_BGGR8) || encoding.equals(BAYER_GBRG8) || encoding.equals(BAYER_GRBG8) || encoding.equals(BAYER_RGGB16) || encoding.equals(BAYER_BGGR16) || encoding.equals(BAYER_GBRG16) || encoding.equals(BAYER_GRBG16)) {
            return 1;
        }
        if (encoding.equals(YUV422)) {
            return 2;
        }
        String encoding2 = encoding.toUpperCase();
        if (encoding2.equals(TYPE_8UC1) || encoding2.equals(TYPE_8SC1) || encoding2.equals(TYPE_16UC1) || encoding2.equals(TYPE_16SC1) || encoding2.equals(TYPE_32SC1) || encoding2.equals(TYPE_32FC1) || encoding2.equals(TYPE_64FC1)) {
            return 1;
        }
        if (encoding2.equals(TYPE_8UC2) || encoding2.equals(TYPE_8SC2) || encoding2.equals(TYPE_16UC2) || encoding2.equals(TYPE_16SC2) || encoding2.equals(TYPE_32SC2) || encoding2.equals(TYPE_32FC2) || encoding2.equals(TYPE_64FC2)) {
            return 2;
        }
        if (encoding2.equals(TYPE_8UC3) || encoding2.equals(TYPE_8SC3) || encoding2.equals(TYPE_16UC3) || encoding2.equals(TYPE_16SC3) || encoding2.equals(TYPE_32SC3) || encoding2.equals(TYPE_32FC3) || encoding2.equals(TYPE_64FC3)) {
            return 3;
        }
        if (encoding2.equals(TYPE_8UC4) || encoding2.equals(TYPE_8SC4) || encoding2.equals(TYPE_16UC4) || encoding2.equals(TYPE_16SC4) || encoding2.equals(TYPE_32SC4) || encoding2.equals(TYPE_32FC4) || encoding2.equals(TYPE_64FC4)) {
            return 4;
        }
        throw new Exception("Unknown encoding " + encoding2);
    }

    public static int bitDepth(String enc) throws Exception {
        String encoding = enc.toLowerCase();
        if (encoding.equals(MONO16)) {
            return 16;
        }
        if (encoding.equals(MONO8) || encoding.equals(BGR8) || encoding.equals(RGB8) || encoding.equals(BGRA8) || encoding.equals(RGBA8) || encoding.equals(BAYER_RGGB8) || encoding.equals(BAYER_BGGR8) || encoding.equals(BAYER_GBRG8) || encoding.equals(BAYER_GRBG8)) {
            return 8;
        }
        if (encoding.equals(MONO16) || encoding.equals(BGR16) || encoding.equals(RGB16) || encoding.equals(BGRA16) || encoding.equals(RGBA16) || encoding.equals(BAYER_RGGB16) || encoding.equals(BAYER_BGGR16) || encoding.equals(BAYER_GBRG16) || encoding.equals(BAYER_GRBG16)) {
            return 16;
        }
        if (encoding.equals(YUV422)) {
            return 8;
        }
        String encoding2 = encoding.toUpperCase();
        if (encoding2.equals(TYPE_8UC1) || encoding2.equals(TYPE_8UC2) || encoding2.equals(TYPE_8UC3) || encoding2.equals(TYPE_8UC4) || encoding2.equals(TYPE_8SC1) || encoding2.equals(TYPE_8SC2) || encoding2.equals(TYPE_8SC3) || encoding2.equals(TYPE_8SC4)) {
            return 8;
        }
        if (encoding2.equals(TYPE_16UC1) || encoding2.equals(TYPE_16UC2) || encoding2.equals(TYPE_16UC3) || encoding2.equals(TYPE_16UC4) || encoding2.equals(TYPE_16SC1) || encoding2.equals(TYPE_16SC2) || encoding2.equals(TYPE_16SC3) || encoding2.equals(TYPE_16SC4)) {
            return 16;
        }
        if (encoding2.equals(TYPE_32SC1) || encoding2.equals(TYPE_32SC2) || encoding2.equals(TYPE_32SC3) || encoding2.equals(TYPE_32SC4) || encoding2.equals(TYPE_32FC1) || encoding2.equals(TYPE_32FC2) || encoding2.equals(TYPE_32FC3) || encoding2.equals(TYPE_32FC4)) {
            return 32;
        }
        if (encoding2.equals(TYPE_64FC1) || encoding2.equals(TYPE_64FC2) || encoding2.equals(TYPE_64FC3) || encoding2.equals(TYPE_64FC4)) {
            return 64;
        }
        throw new Exception("Unknown encoding " + encoding2);
    }
}
