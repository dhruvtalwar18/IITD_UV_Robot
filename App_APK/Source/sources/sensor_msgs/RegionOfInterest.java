package sensor_msgs;

import org.ros.internal.message.Message;

public interface RegionOfInterest extends Message {
    public static final String _DEFINITION = "# This message is used to specify a region of interest within an image.\n#\n# When used to specify the ROI setting of the camera when the image was\n# taken, the height and width fields should either match the height and\n# width fields for the associated image; or height = width = 0\n# indicates that the full resolution image was captured.\n\nuint32 x_offset  # Leftmost pixel of the ROI\n                 # (0 if the ROI includes the left edge of the image)\nuint32 y_offset  # Topmost pixel of the ROI\n                 # (0 if the ROI includes the top edge of the image)\nuint32 height    # Height of ROI\nuint32 width     # Width of ROI\n\n# True if a distinct rectified ROI should be calculated from the \"raw\"\n# ROI in this message. Typically this should be False if the full image\n# is captured (ROI not used), and True if a subwindow is captured (ROI\n# used).\nbool do_rectify\n";
    public static final String _TYPE = "sensor_msgs/RegionOfInterest";

    boolean getDoRectify();

    int getHeight();

    int getWidth();

    int getXOffset();

    int getYOffset();

    void setDoRectify(boolean z);

    void setHeight(int i);

    void setWidth(int i);

    void setXOffset(int i);

    void setYOffset(int i);
}
