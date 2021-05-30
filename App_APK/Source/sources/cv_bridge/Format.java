package cv_bridge;

import java.util.HashMap;
import java.util.Map;

public enum Format {
    JPG("jpg"),
    JPEG("jpeg"),
    JPE("jpe"),
    PNG("png"),
    BMP("bmp"),
    DIB("dib"),
    PPM("ppm"),
    PGM("pgm"),
    PBM("pbm");
    
    private static Map<Format, String> map;
    protected String strFormat;

    static {
        int i;
        map = new HashMap();
        for (Format format : values()) {
            map.put(format, format.strFormat);
        }
    }

    private Format(String strFormat2) {
        this.strFormat = strFormat2;
    }

    public static String valueOf(Format format) {
        return map.get(format);
    }

    public static String getExtension(Format format) {
        return ".".concat(map.get(format));
    }
}
