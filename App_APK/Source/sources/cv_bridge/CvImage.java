package cv_bridge;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Vector;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.opencv_imgcodecs;
import org.bytedeco.javacpp.opencv_imgproc;
import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.ros.internal.message.MessageBuffers;
import sensor_msgs.CompressedImage;
import sensor_msgs.Image;
import sensor_msgs.ImageEncodings;
import std_msgs.Header;

public class CvImage {
    protected static final String TAG = "cv_bridge::CvImage";
    public String encoding = "";
    public Header header;
    public opencv_core.Mat image = new opencv_core.Mat();

    protected CvImage() {
    }

    public CvImage(Header header2, String encoding2) {
        this.header = header2;
        this.encoding = encoding2.toUpperCase();
        this.image = new opencv_core.Mat();
    }

    public CvImage(Header header2, String encoding2, opencv_core.Mat image2) {
        this.header = header2;
        this.encoding = encoding2.toUpperCase();
        this.image = image2;
    }

    public final Image toImageMsg(Image ros_image) throws IOException {
        ros_image.setHeader(this.header);
        ros_image.setEncoding(this.encoding.toLowerCase());
        ros_image.setWidth(this.image.cols());
        ros_image.setHeight(this.image.rows());
        ros_image.setStep(this.image.arrayStep());
        ChannelBufferOutputStream stream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
        byte[] imageInBytes = new byte[this.image.arraySize()];
        ((ByteBuffer) this.image.createBuffer()).get(imageInBytes);
        stream.write(imageInBytes);
        ros_image.setData(stream.buffer());
        return ros_image;
    }

    public final CompressedImage toCompressedImageMsg(CompressedImage ros_image, Format dst_format) throws Exception {
        opencv_core.Mat image2;
        ros_image.setHeader(this.header);
        if (!this.encoding.equals(ImageEncodings.BGR8)) {
            image2 = cvtColor(this, ImageEncodings.BGR8).image;
        } else {
            image2 = this.image;
        }
        BytePointer buf = new BytePointer();
        ros_image.setFormat(Format.valueOf(dst_format));
        opencv_imgcodecs.imencode(Format.getExtension(dst_format), image2, buf);
        ChannelBufferOutputStream stream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
        byte[] outputBuffer = new byte[((int) buf.capacity())];
        buf.get(outputBuffer);
        stream.write(outputBuffer);
        ros_image.setData(stream.buffer());
        return ros_image;
    }

    public static CvImage toCvCopy(Image source) throws Exception {
        return toCvCopyImpl(matFromImage(source), source.getHeader(), source.getEncoding(), "");
    }

    public static CvImage toCvCopy(Image source, String dst_encoding) throws Exception {
        return toCvCopyImpl(matFromImage(source), source.getHeader(), source.getEncoding(), dst_encoding);
    }

    public static CvImage toCvCopy(CompressedImage source) throws Exception {
        return toCvCopyImpl(matFromImage(source), source.getHeader(), ImageEncodings.BGR8, "");
    }

    public static CvImage toCvCopy(CompressedImage source, String dst_encoding) throws Exception {
        return toCvCopyImpl(matFromImage(source), source.getHeader(), ImageEncodings.BGR8, dst_encoding);
    }

    public static CvImage cvtColor(CvImage source, String encoding2) throws Exception {
        return toCvCopyImpl(source.image, source.header, source.encoding, encoding2);
    }

    protected static CvImage toCvCopyImpl(opencv_core.Mat source, Header src_header, String src_encoding, String dst_encoding) throws Exception {
        String str = src_encoding;
        String str2 = dst_encoding;
        CvImage cvImage = new CvImage();
        cvImage.header = src_header;
        if (dst_encoding.isEmpty() || str2.equals(str)) {
            cvImage.encoding = src_encoding;
            source.copyTo(cvImage.image);
        } else {
            Vector<Integer> conversion_codes = ImEncoding.getConversionCode(src_encoding, dst_encoding);
            opencv_core.Mat image2 = new opencv_core.Mat();
            opencv_core.Mat image1 = source;
            int i = 0;
            while (i < conversion_codes.size()) {
                int conversion_code = conversion_codes.get(i).intValue();
                if (conversion_code == -1) {
                    int src_depth = ImageEncodings.bitDepth(src_encoding);
                    int dst_depth = ImageEncodings.bitDepth(dst_encoding);
                    int image2_type = opencv_core.CV_MAKETYPE(opencv_core.CV_MAT_DEPTH(ImEncoding.getCvType(dst_encoding)), image1.channels());
                    if (src_depth == 8 && dst_depth == 16) {
                        int i2 = image2_type;
                        int i3 = dst_depth;
                        int i4 = conversion_code;
                        int i5 = src_depth;
                        image1.convertTo(image2, image2_type, 257.0d, (double) 0);
                    } else {
                        int image2_type2 = image2_type;
                        int dst_depth2 = dst_depth;
                        int i6 = conversion_code;
                        if (src_depth == 16 && dst_depth2 == 8) {
                            image1.convertTo(image2, image2_type2, 0.0038910505836575876d, (double) 0);
                        } else {
                            image1.convertTo(image2, image2_type2);
                        }
                    }
                } else {
                    opencv_imgproc.cvtColor(image1, image2, conversion_codes.get(0).intValue());
                }
                image1 = image2;
                i++;
                String str3 = src_encoding;
                Header header2 = src_header;
            }
            cvImage.image = image2;
            cvImage.encoding = str2;
            String str4 = src_encoding;
            opencv_core.Mat mat = source;
        }
        return cvImage;
    }

    protected static opencv_core.Mat matFromImage(Image source) throws Exception {
        byte[] imageInBytes = source.getData().array();
        byte[] imageInBytes2 = Arrays.copyOfRange(imageInBytes, source.getData().arrayOffset(), imageInBytes.length);
        return new opencv_core.Mat(source.getHeight(), source.getWidth(), ImEncoding.getCvType(source.getEncoding().toUpperCase())).data(new BytePointer(imageInBytes2));
    }

    protected static opencv_core.Mat matFromImage(CompressedImage source) throws Exception {
        byte[] imageInBytes = source.getData().array();
        byte[] imageInBytes2 = Arrays.copyOfRange(imageInBytes, source.getData().arrayOffset(), imageInBytes.length);
        return opencv_imgcodecs.imdecode(new opencv_core.Mat(1, imageInBytes2.length, opencv_core.CV_8UC1).data(new BytePointer(imageInBytes2)), 4);
    }
}
