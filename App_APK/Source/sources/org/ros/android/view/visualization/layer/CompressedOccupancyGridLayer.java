package org.ros.android.view.visualization.layer;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import com.google.common.base.Preconditions;
import javax.microedition.khronos.opengles.GL10;
import nav_msgs.OccupancyGrid;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.android.view.visualization.TextureBitmap;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.rosjava_geometry.Transform;

public class CompressedOccupancyGridLayer extends SubscriberLayer<OccupancyGrid> implements TfLayer {
    private static final int COLOR_FREE = -7500403;
    private static final int COLOR_OCCUPIED = -536870913;
    private static final int COLOR_UNKNOWN = -16777216;
    private GraphName frame;
    private boolean ready;
    private final TextureBitmap textureBitmap;

    public CompressedOccupancyGridLayer(String topic) {
        this(GraphName.of(topic));
    }

    public CompressedOccupancyGridLayer(GraphName topic) {
        super(topic, OccupancyGrid._TYPE);
        this.textureBitmap = new TextureBitmap();
        this.ready = false;
    }

    public void draw(VisualizationView view, GL10 gl) {
        if (this.ready) {
            this.textureBitmap.draw(view, gl);
        }
    }

    public GraphName getFrame() {
        return this.frame;
    }

    public void onStart(VisualizationView view, ConnectedNode connectedNode) {
        super.onStart(view, connectedNode);
        getSubscriber().addMessageListener(new MessageListener<OccupancyGrid>() {
            public void onNewMessage(OccupancyGrid message) {
                CompressedOccupancyGridLayer.this.update(message);
            }
        });
    }

    /* access modifiers changed from: package-private */
    public void update(OccupancyGrid message) {
        ChannelBuffer buffer = message.getData();
        Bitmap bitmap = BitmapFactory.decodeByteArray(buffer.array(), buffer.arrayOffset(), buffer.readableBytes());
        int stride = bitmap.getWidth();
        int height = bitmap.getHeight();
        Preconditions.checkArgument(stride <= 1024);
        Preconditions.checkArgument(height <= 1024);
        int[] pixels = new int[(stride * height)];
        bitmap.getPixels(pixels, 0, stride, 0, 0, stride, height);
        for (int i = 0; i < pixels.length; i++) {
            if (pixels[i] == -1) {
                pixels[i] = -16777216;
            } else if (pixels[i] == -16777216) {
                pixels[i] = COLOR_FREE;
            } else {
                pixels[i] = COLOR_OCCUPIED;
            }
        }
        this.textureBitmap.updateFromPixelArray(pixels, stride, message.getInfo().getResolution(), Transform.fromPoseMessage(message.getInfo().getOrigin()), -16777216);
        this.frame = GraphName.of(message.getHeader().getFrameId());
        this.ready = true;
    }
}
