package org.ros.android;

import android.graphics.Bitmap;
import android.graphics.Color;
import com.google.common.base.Preconditions;
import org.jboss.netty.buffer.ChannelBuffer;
import sensor_msgs.Image;
import sensor_msgs.ImageEncodings;
import sensor_msgs.NavSatStatus;

public class BitmapFromImage implements MessageCallable<Bitmap, Image> {
    public Bitmap call(Image message) {
        Preconditions.checkArgument(message.getEncoding().equals(ImageEncodings.RGB8));
        Bitmap bitmap = Bitmap.createBitmap(message.getWidth(), message.getHeight(), Bitmap.Config.ARGB_8888);
        for (int x = 0; x < message.getWidth(); x++) {
            for (int y = 0; y < message.getHeight(); y++) {
                ChannelBuffer data = message.getData();
                bitmap.setPixel(x, y, Color.argb(255, data.getByte((message.getStep() * y) + (x * 3)) & NavSatStatus.STATUS_NO_FIX, data.getByte((message.getStep() * y) + (x * 3) + 1) & NavSatStatus.STATUS_NO_FIX, data.getByte((message.getStep() * y) + (x * 3) + 2) & NavSatStatus.STATUS_NO_FIX));
            }
        }
        return bitmap;
    }
}
