package org.bytedeco.javacpp;

import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.StdVector;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_imgcodecs extends org.bytedeco.javacpp.helper.opencv_imgcodecs {
    public static final int IMREAD_ANYCOLOR = 4;
    public static final int IMREAD_ANYDEPTH = 2;
    public static final int IMREAD_COLOR = 1;
    public static final int IMREAD_GRAYSCALE = 0;
    public static final int IMREAD_IGNORE_ORIENTATION = 128;
    public static final int IMREAD_LOAD_GDAL = 8;
    public static final int IMREAD_REDUCED_COLOR_2 = 17;
    public static final int IMREAD_REDUCED_COLOR_4 = 33;
    public static final int IMREAD_REDUCED_COLOR_8 = 65;
    public static final int IMREAD_REDUCED_GRAYSCALE_2 = 16;
    public static final int IMREAD_REDUCED_GRAYSCALE_4 = 32;
    public static final int IMREAD_REDUCED_GRAYSCALE_8 = 64;
    public static final int IMREAD_UNCHANGED = -1;
    public static final int IMWRITE_EXR_TYPE = 48;
    public static final int IMWRITE_EXR_TYPE_FLOAT = 2;
    public static final int IMWRITE_EXR_TYPE_HALF = 1;
    public static final int IMWRITE_JPEG_CHROMA_QUALITY = 6;
    public static final int IMWRITE_JPEG_LUMA_QUALITY = 5;
    public static final int IMWRITE_JPEG_OPTIMIZE = 3;
    public static final int IMWRITE_JPEG_PROGRESSIVE = 2;
    public static final int IMWRITE_JPEG_QUALITY = 1;
    public static final int IMWRITE_JPEG_RST_INTERVAL = 4;
    public static final int IMWRITE_PAM_FORMAT_BLACKANDWHITE = 1;
    public static final int IMWRITE_PAM_FORMAT_GRAYSCALE = 2;
    public static final int IMWRITE_PAM_FORMAT_GRAYSCALE_ALPHA = 3;
    public static final int IMWRITE_PAM_FORMAT_NULL = 0;
    public static final int IMWRITE_PAM_FORMAT_RGB = 4;
    public static final int IMWRITE_PAM_FORMAT_RGB_ALPHA = 5;
    public static final int IMWRITE_PAM_TUPLETYPE = 128;
    public static final int IMWRITE_PNG_BILEVEL = 18;
    public static final int IMWRITE_PNG_COMPRESSION = 16;
    public static final int IMWRITE_PNG_STRATEGY = 17;
    public static final int IMWRITE_PNG_STRATEGY_DEFAULT = 0;
    public static final int IMWRITE_PNG_STRATEGY_FILTERED = 1;
    public static final int IMWRITE_PNG_STRATEGY_FIXED = 4;
    public static final int IMWRITE_PNG_STRATEGY_HUFFMAN_ONLY = 2;
    public static final int IMWRITE_PNG_STRATEGY_RLE = 3;
    public static final int IMWRITE_PXM_BINARY = 32;
    public static final int IMWRITE_TIFF_RESUNIT = 256;
    public static final int IMWRITE_TIFF_XDPI = 257;
    public static final int IMWRITE_TIFF_YDPI = 258;
    public static final int IMWRITE_WEBP_QUALITY = 64;

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean haveImageReader(@opencv_core.Str String str);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean haveImageReader(@opencv_core.Str BytePointer bytePointer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean haveImageWriter(@opencv_core.Str String str);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean haveImageWriter(@opencv_core.Str BytePointer bytePointer);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat imdecode(@ByVal opencv_core.GpuMat gpuMat, int i);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat imdecode(@ByVal opencv_core.GpuMat gpuMat, int i, opencv_core.Mat mat);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat imdecode(@ByVal opencv_core.Mat mat, int i);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat imdecode(@ByVal opencv_core.Mat mat, int i, opencv_core.Mat mat2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat imdecode(@ByVal opencv_core.UMat uMat, int i);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat imdecode(@ByVal opencv_core.UMat uMat, int i, opencv_core.Mat mat);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str String str, @ByVal opencv_core.GpuMat gpuMat, @Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str String str, @ByVal opencv_core.GpuMat gpuMat, @Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer, @StdVector IntBuffer intBuffer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str String str, @ByVal opencv_core.GpuMat gpuMat, @Cast({"uchar*"}) @StdVector BytePointer bytePointer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str String str, @ByVal opencv_core.GpuMat gpuMat, @Cast({"uchar*"}) @StdVector BytePointer bytePointer, @StdVector IntPointer intPointer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str String str, @ByVal opencv_core.GpuMat gpuMat, @Cast({"uchar*"}) @StdVector byte[] bArr);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str String str, @ByVal opencv_core.GpuMat gpuMat, @Cast({"uchar*"}) @StdVector byte[] bArr, @StdVector int[] iArr);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str String str, @ByVal opencv_core.Mat mat, @Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str String str, @ByVal opencv_core.Mat mat, @Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer, @StdVector IntBuffer intBuffer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str String str, @ByVal opencv_core.Mat mat, @Cast({"uchar*"}) @StdVector BytePointer bytePointer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str String str, @ByVal opencv_core.Mat mat, @Cast({"uchar*"}) @StdVector BytePointer bytePointer, @StdVector IntPointer intPointer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str String str, @ByVal opencv_core.Mat mat, @Cast({"uchar*"}) @StdVector byte[] bArr);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str String str, @ByVal opencv_core.Mat mat, @Cast({"uchar*"}) @StdVector byte[] bArr, @StdVector int[] iArr);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str String str, @ByVal opencv_core.UMat uMat, @Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str String str, @ByVal opencv_core.UMat uMat, @Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer, @StdVector IntBuffer intBuffer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str String str, @ByVal opencv_core.UMat uMat, @Cast({"uchar*"}) @StdVector BytePointer bytePointer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str String str, @ByVal opencv_core.UMat uMat, @Cast({"uchar*"}) @StdVector BytePointer bytePointer, @StdVector IntPointer intPointer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str String str, @ByVal opencv_core.UMat uMat, @Cast({"uchar*"}) @StdVector byte[] bArr);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str String str, @ByVal opencv_core.UMat uMat, @Cast({"uchar*"}) @StdVector byte[] bArr, @StdVector int[] iArr);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.GpuMat gpuMat, @Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.GpuMat gpuMat, @Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer, @StdVector IntBuffer intBuffer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.GpuMat gpuMat, @Cast({"uchar*"}) @StdVector BytePointer bytePointer2);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.GpuMat gpuMat, @Cast({"uchar*"}) @StdVector BytePointer bytePointer2, @StdVector IntPointer intPointer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.GpuMat gpuMat, @Cast({"uchar*"}) @StdVector byte[] bArr);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.GpuMat gpuMat, @Cast({"uchar*"}) @StdVector byte[] bArr, @StdVector int[] iArr);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.Mat mat, @Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.Mat mat, @Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer, @StdVector IntBuffer intBuffer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.Mat mat, @Cast({"uchar*"}) @StdVector BytePointer bytePointer2);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.Mat mat, @Cast({"uchar*"}) @StdVector BytePointer bytePointer2, @StdVector IntPointer intPointer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.Mat mat, @Cast({"uchar*"}) @StdVector byte[] bArr);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.Mat mat, @Cast({"uchar*"}) @StdVector byte[] bArr, @StdVector int[] iArr);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.UMat uMat, @Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.UMat uMat, @Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer, @StdVector IntBuffer intBuffer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.UMat uMat, @Cast({"uchar*"}) @StdVector BytePointer bytePointer2);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.UMat uMat, @Cast({"uchar*"}) @StdVector BytePointer bytePointer2, @StdVector IntPointer intPointer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.UMat uMat, @Cast({"uchar*"}) @StdVector byte[] bArr);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imencode(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.UMat uMat, @Cast({"uchar*"}) @StdVector byte[] bArr, @StdVector int[] iArr);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat imread(@opencv_core.Str String str);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat imread(@opencv_core.Str String str, int i);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat imread(@opencv_core.Str BytePointer bytePointer);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat imread(@opencv_core.Str BytePointer bytePointer, int i);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imreadmulti(@opencv_core.Str String str, @ByRef opencv_core.MatVector matVector);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imreadmulti(@opencv_core.Str String str, @ByRef opencv_core.MatVector matVector, int i);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imreadmulti(@opencv_core.Str BytePointer bytePointer, @ByRef opencv_core.MatVector matVector);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imreadmulti(@opencv_core.Str BytePointer bytePointer, @ByRef opencv_core.MatVector matVector, int i);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imwrite(@opencv_core.Str String str, @ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imwrite(@opencv_core.Str String str, @ByVal opencv_core.GpuMat gpuMat, @StdVector IntBuffer intBuffer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imwrite(@opencv_core.Str String str, @ByVal opencv_core.GpuMat gpuMat, @StdVector IntPointer intPointer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imwrite(@opencv_core.Str String str, @ByVal opencv_core.GpuMat gpuMat, @StdVector int[] iArr);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imwrite(@opencv_core.Str String str, @ByVal opencv_core.Mat mat);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imwrite(@opencv_core.Str String str, @ByVal opencv_core.Mat mat, @StdVector IntBuffer intBuffer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imwrite(@opencv_core.Str String str, @ByVal opencv_core.Mat mat, @StdVector IntPointer intPointer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imwrite(@opencv_core.Str String str, @ByVal opencv_core.Mat mat, @StdVector int[] iArr);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imwrite(@opencv_core.Str String str, @ByVal opencv_core.UMat uMat);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imwrite(@opencv_core.Str String str, @ByVal opencv_core.UMat uMat, @StdVector IntBuffer intBuffer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imwrite(@opencv_core.Str String str, @ByVal opencv_core.UMat uMat, @StdVector IntPointer intPointer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imwrite(@opencv_core.Str String str, @ByVal opencv_core.UMat uMat, @StdVector int[] iArr);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imwrite(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imwrite(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.GpuMat gpuMat, @StdVector IntBuffer intBuffer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imwrite(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.GpuMat gpuMat, @StdVector IntPointer intPointer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imwrite(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.GpuMat gpuMat, @StdVector int[] iArr);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imwrite(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.Mat mat);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imwrite(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.Mat mat, @StdVector IntBuffer intBuffer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imwrite(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.Mat mat, @StdVector IntPointer intPointer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imwrite(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.Mat mat, @StdVector int[] iArr);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imwrite(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.UMat uMat);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imwrite(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.UMat uMat, @StdVector IntBuffer intBuffer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imwrite(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.UMat uMat, @StdVector IntPointer intPointer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean imwrite(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.UMat uMat, @StdVector int[] iArr);

    static {
        Loader.load();
    }
}
