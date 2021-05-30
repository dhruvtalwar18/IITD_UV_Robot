package org.ros.android.view.camera;

import android.graphics.Rect;
import android.graphics.YuvImage;
import android.hardware.Camera;
import com.google.common.base.Preconditions;
import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.ros.internal.message.MessageBuffers;
import org.ros.message.Time;
import org.ros.namespace.NameResolver;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import sensor_msgs.CameraInfo;
import sensor_msgs.CompressedImage;

class CompressedImagePublisher implements RawImageListener {
    private final Publisher<CameraInfo> cameraInfoPublisher;
    private final ConnectedNode connectedNode;
    private final Publisher<CompressedImage> imagePublisher;
    private byte[] rawImageBuffer;
    private Camera.Size rawImageSize;
    private Rect rect;
    private ChannelBufferOutputStream stream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
    private YuvImage yuvImage;

    public CompressedImagePublisher(ConnectedNode connectedNode2) {
        this.connectedNode = connectedNode2;
        NameResolver resolver = connectedNode2.getResolver().newChild("camera");
        this.imagePublisher = connectedNode2.newPublisher(resolver.resolve("image/compressed"), CompressedImage._TYPE);
        this.cameraInfoPublisher = connectedNode2.newPublisher(resolver.resolve("camera_info"), CameraInfo._TYPE);
    }

    public void onNewRawImage(byte[] data, Camera.Size size) {
        Preconditions.checkNotNull(data);
        Preconditions.checkNotNull(size);
        if (data != this.rawImageBuffer || !size.equals(this.rawImageSize)) {
            this.rawImageBuffer = data;
            this.rawImageSize = size;
            this.yuvImage = new YuvImage(this.rawImageBuffer, 17, size.width, size.height, (int[]) null);
            this.rect = new Rect(0, 0, size.width, size.height);
        }
        Time currentTime = this.connectedNode.getCurrentTime();
        CompressedImage image = this.imagePublisher.newMessage();
        image.setFormat("jpeg");
        image.getHeader().setStamp(currentTime);
        image.getHeader().setFrameId("camera");
        Preconditions.checkState(this.yuvImage.compressToJpeg(this.rect, 20, this.stream));
        image.setData(this.stream.buffer().copy());
        this.stream.buffer().clear();
        this.imagePublisher.publish(image);
        CameraInfo cameraInfo = this.cameraInfoPublisher.newMessage();
        cameraInfo.getHeader().setStamp(currentTime);
        cameraInfo.getHeader().setFrameId("camera");
        cameraInfo.setWidth(size.width);
        cameraInfo.setHeight(size.height);
        this.cameraInfoPublisher.publish(cameraInfo);
    }
}
