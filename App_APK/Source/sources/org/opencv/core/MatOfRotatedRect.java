package org.opencv.core;

import java.util.Arrays;
import java.util.List;

public class MatOfRotatedRect extends Mat {
    private static final int _channels = 5;
    private static final int _depth = 5;

    public MatOfRotatedRect() {
    }

    protected MatOfRotatedRect(long addr) {
        super(addr);
        if (!empty() && checkVector(5, 5) < 0) {
            throw new IllegalArgumentException("Incompatible Mat");
        }
    }

    public static MatOfRotatedRect fromNativeAddr(long addr) {
        return new MatOfRotatedRect(addr);
    }

    public MatOfRotatedRect(Mat m) {
        super(m, Range.all());
        if (!empty() && checkVector(5, 5) < 0) {
            throw new IllegalArgumentException("Incompatible Mat");
        }
    }

    public MatOfRotatedRect(RotatedRect... a) {
        fromArray(a);
    }

    public void alloc(int elemNumber) {
        if (elemNumber > 0) {
            super.create(elemNumber, 1, CvType.makeType(5, 5));
        }
    }

    public void fromArray(RotatedRect... a) {
        if (a != null && a.length != 0) {
            int num = a.length;
            alloc(num);
            float[] buff = new float[(num * 5)];
            for (int i = 0; i < num; i++) {
                RotatedRect r = a[i];
                buff[(i * 5) + 0] = (float) r.center.x;
                buff[(i * 5) + 1] = (float) r.center.y;
                buff[(i * 5) + 2] = (float) r.size.width;
                buff[(i * 5) + 3] = (float) r.size.height;
                buff[(i * 5) + 4] = (float) r.angle;
            }
            put(0, 0, buff);
        }
    }

    public RotatedRect[] toArray() {
        int num = (int) total();
        RotatedRect[] a = new RotatedRect[num];
        if (num == 0) {
            return a;
        }
        float[] buff = new float[5];
        for (int i = 0; i < num; i++) {
            get(i, 0, buff);
            a[i] = new RotatedRect(new Point((double) buff[0], (double) buff[1]), new Size((double) buff[2], (double) buff[3]), (double) buff[4]);
        }
        return a;
    }

    public void fromList(List<RotatedRect> lr) {
        fromArray((RotatedRect[]) lr.toArray(new RotatedRect[0]));
    }

    public List<RotatedRect> toList() {
        return Arrays.asList(toArray());
    }
}
