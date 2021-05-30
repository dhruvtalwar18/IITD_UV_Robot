package org.ros.android;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import org.jboss.netty.buffer.ChannelBuffer;
import sensor_msgs.CompressedImage;

public class BitmapFromCompressedImage implements MessageCallable<Bitmap, CompressedImage> {
    public Bitmap call(CompressedImage message) {
        ChannelBuffer buffer = message.getData();
        return BitmapFactory.decodeByteArray(buffer.array(), buffer.arrayOffset(), buffer.readableBytes());
    }
}
