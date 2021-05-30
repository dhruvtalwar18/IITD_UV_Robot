package cv_bridge;

import java.util.HashMap;
import java.util.Map;

public enum Encoding {
    INVALID(-1),
    GRAY(0),
    RGB(1),
    BGR(2),
    RGBA(3),
    BGRA(4),
    YUV422(5),
    BAYER_RGGB(6),
    BAYER_BGGR(7),
    BAYER_GBRG(8),
    BAYER_GRBG(9);
    
    private static Map<Integer, Encoding> map;
    protected int encodingNumber;

    static {
        int i;
        map = new HashMap();
        for (Encoding encoding : values()) {
            map.put(Integer.valueOf(encoding.encodingNumber), encoding);
        }
    }

    private Encoding(int encodingNumber2) {
        this.encodingNumber = encodingNumber2;
    }

    public static Encoding valueOf(int encodingNumber2) {
        return map.get(Integer.valueOf(encodingNumber2));
    }
}
