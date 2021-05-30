package org.ros.android.view.visualization.layer;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import java.util.List;
import javax.microedition.khronos.opengles.GL10;
import nav_msgs.OccupancyGrid;
import org.bytedeco.javacpp.opencv_stitching;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.android.view.visualization.TextureBitmap;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.internal.message.MessageBuffers;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

public class OccupancyGridLayer extends SubscriberLayer<OccupancyGrid> implements TfLayer {
    private static final int COLOR_FREE = -1;
    private static final int COLOR_OCCUPIED = -15658735;
    private static final int COLOR_TRANSPARENT = 0;
    private static final int COLOR_UNKNOWN = -2236963;
    private GraphName frame;
    private GL10 previousGl;
    private boolean ready;
    private final List<Tile> tiles;

    private class Tile {
        private Transform origin;
        private final ChannelBuffer pixelBuffer = MessageBuffers.dynamicBuffer();
        private boolean ready;
        private final float resolution;
        private int stride;
        private final TextureBitmap textureBitmap = new TextureBitmap();

        public Tile(float resolution2) {
            this.resolution = resolution2;
            this.ready = false;
        }

        public void draw(VisualizationView view, GL10 gl) {
            if (this.ready) {
                this.textureBitmap.draw(view, gl);
            }
        }

        public void clearHandle() {
            this.textureBitmap.clearHandle();
        }

        public void writeInt(int value) {
            this.pixelBuffer.writeInt(value);
        }

        public void update() {
            Preconditions.checkNotNull(this.origin);
            Preconditions.checkNotNull(Integer.valueOf(this.stride));
            this.textureBitmap.updateFromPixelBuffer(this.pixelBuffer, this.stride, this.resolution, this.origin, 0);
            this.pixelBuffer.clear();
            this.ready = true;
        }

        public void setOrigin(Transform origin2) {
            this.origin = origin2;
        }

        public void setStride(int stride2) {
            this.stride = stride2;
        }
    }

    public OccupancyGridLayer(String topic) {
        this(GraphName.of(topic));
    }

    public OccupancyGridLayer(GraphName topic) {
        super(topic, OccupancyGrid._TYPE);
        this.tiles = Lists.newCopyOnWriteArrayList();
        this.ready = false;
    }

    public void draw(VisualizationView view, GL10 gl) {
        if (this.previousGl != gl) {
            for (Tile tile : this.tiles) {
                tile.clearHandle();
            }
            this.previousGl = gl;
        }
        if (this.ready) {
            for (Tile tile2 : this.tiles) {
                tile2.draw(view, gl);
            }
        }
    }

    public GraphName getFrame() {
        return this.frame;
    }

    public void onStart(VisualizationView view, ConnectedNode connectedNode) {
        super.onStart(view, connectedNode);
        this.previousGl = null;
        getSubscriber().addMessageListener(new MessageListener<OccupancyGrid>() {
            public void onNewMessage(OccupancyGrid message) {
                OccupancyGridLayer.this.update(message);
            }
        });
    }

    /* access modifiers changed from: private */
    public void update(OccupancyGrid message) {
        float resolution = message.getInfo().getResolution();
        int width = message.getInfo().getWidth();
        int height = message.getInfo().getHeight();
        float f = 1024.0f;
        int numTilesWide = (int) Math.ceil((double) (((float) width) / 1024.0f));
        int numTilesHigh = (int) Math.ceil((double) (((float) height) / 1024.0f));
        int numTiles = numTilesWide * numTilesHigh;
        Transform origin = Transform.fromPoseMessage(message.getInfo().getOrigin());
        while (this.tiles.size() < numTiles) {
            this.tiles.add(new Tile(resolution));
        }
        int y = 0;
        while (y < numTilesHigh) {
            int x = 0;
            while (x < numTilesWide) {
                int tileIndex = (y * numTilesWide) + x;
                int numTilesHigh2 = numTilesHigh;
                int y2 = y;
                Vector3 vector3 = r15;
                Vector3 vector32 = new Vector3((double) (((float) x) * resolution * f), (double) (((float) y) * resolution * 1024.0f), opencv_stitching.Stitcher.ORIG_RESOL);
                this.tiles.get(tileIndex).setOrigin(origin.multiply(new Transform(vector3, Quaternion.identity())));
                if (x < numTilesWide - 1) {
                    this.tiles.get(tileIndex).setStride(1024);
                } else {
                    this.tiles.get(tileIndex).setStride(width % 1024);
                }
                x++;
                numTilesHigh = numTilesHigh2;
                y = y2;
                f = 1024.0f;
            }
            y++;
            f = 1024.0f;
        }
        int x2 = 0;
        int y3 = 0;
        ChannelBuffer buffer = message.getData();
        while (true) {
            boolean z = true;
            if (!buffer.readable()) {
                break;
            }
            if (y3 >= height) {
                z = false;
            }
            Preconditions.checkState(z);
            int tileIndex2 = ((y3 / 1024) * numTilesWide) + (x2 / 1024);
            byte pixel = buffer.readByte();
            if (pixel == -1) {
                this.tiles.get(tileIndex2).writeInt(COLOR_UNKNOWN);
            } else if (pixel < 50) {
                this.tiles.get(tileIndex2).writeInt(-1);
            } else {
                this.tiles.get(tileIndex2).writeInt(COLOR_OCCUPIED);
            }
            x2++;
            if (x2 == width) {
                x2 = 0;
                y3++;
            }
        }
        for (Tile tile : this.tiles) {
            tile.update();
        }
        this.frame = GraphName.of(message.getHeader().getFrameId());
        this.ready = true;
    }
}
