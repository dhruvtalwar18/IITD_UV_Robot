package org.opencv.highgui;

import java.awt.Dimension;
import javax.swing.JFrame;
import javax.swing.JLabel;
import org.bytedeco.javacpp.opencv_stitching;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public final class ImageWindow {
    public static final int WINDOW_AUTOSIZE = 1;
    public static final int WINDOW_NORMAL = 0;
    public Boolean alreadyUsed = false;
    public int flag;
    public JFrame frame = null;
    public int height = -1;
    public Mat img = null;
    public Boolean imgToBeResized = false;
    public JLabel lbl = null;
    public String name;
    public Boolean positionToBeChanged = false;
    public int width = -1;
    public Boolean windowToBeResized = false;
    public int x = -1;
    public int y = -1;

    public ImageWindow(String name2, Mat img2) {
        this.name = name2;
        this.img = img2;
        this.flag = 0;
    }

    public ImageWindow(String name2, int flag2) {
        this.name = name2;
        this.flag = flag2;
    }

    public static Size keepAspectRatioSize(int original_width, int original_height, int bound_width, int bound_height) {
        int new_width = original_width;
        int new_height = original_height;
        if (original_width > bound_width) {
            new_width = bound_width;
            new_height = (new_width * original_height) / original_width;
        }
        if (new_height > bound_height) {
            new_height = bound_height;
            new_width = (new_height * original_width) / original_height;
        }
        return new Size((double) new_width, (double) new_height);
    }

    public void setMat(Mat img2) {
        this.img = img2;
        this.alreadyUsed = false;
        if (this.imgToBeResized.booleanValue()) {
            resizeImage();
            this.imgToBeResized = false;
        }
    }

    public void setFrameLabelVisible(JFrame frame2, JLabel lbl2) {
        this.frame = frame2;
        this.lbl = lbl2;
        if (this.windowToBeResized.booleanValue()) {
            lbl2.setPreferredSize(new Dimension(this.width, this.height));
            this.windowToBeResized = false;
        }
        if (this.positionToBeChanged.booleanValue()) {
            frame2.setLocation(this.x, this.y);
            this.positionToBeChanged = false;
        }
        frame2.add(lbl2);
        frame2.pack();
        frame2.setVisible(true);
    }

    public void setNewDimension(int width2, int height2) {
        if (this.width != width2 || this.height != height2) {
            this.width = width2;
            this.height = height2;
            if (this.img != null) {
                resizeImage();
            } else {
                this.imgToBeResized = true;
            }
            if (this.lbl != null) {
                this.lbl.setPreferredSize(new Dimension(width2, height2));
            } else {
                this.windowToBeResized = true;
            }
        }
    }

    public void setNewPosition(int x2, int y2) {
        if (this.x != x2 || this.y != y2) {
            this.x = x2;
            this.y = y2;
            if (this.frame != null) {
                this.frame.setLocation(x2, y2);
            } else {
                this.positionToBeChanged = true;
            }
        }
    }

    private void resizeImage() {
        if (this.flag == 0) {
            Imgproc.resize(this.img, this.img, keepAspectRatioSize(this.img.width(), this.img.height(), this.width, this.height), opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL, 5);
        }
    }
}
