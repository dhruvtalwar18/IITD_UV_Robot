package org.opencv.core;

import java.util.Arrays;
import java.util.List;

public class MatOfRect2d extends Mat {
    private static final int _channels = 4;
    private static final int _depth = 6;

    public MatOfRect2d() {
    }

    protected MatOfRect2d(long addr) {
        super(addr);
        if (!empty() && checkVector(4, 6) < 0) {
            throw new IllegalArgumentException("Incompatible Mat");
        }
    }

    public static MatOfRect2d fromNativeAddr(long addr) {
        return new MatOfRect2d(addr);
    }

    public MatOfRect2d(Mat m) {
        super(m, Range.all());
        if (!empty() && checkVector(4, 6) < 0) {
            throw new IllegalArgumentException("Incompatible Mat");
        }
    }

    public MatOfRect2d(Rect2d... a) {
        fromArray(a);
    }

    public void alloc(int elemNumber) {
        if (elemNumber > 0) {
            super.create(elemNumber, 1, CvType.makeType(6, 4));
        }
    }

    public void fromArray(Rect2d... a) {
        if (a != null && a.length != 0) {
            int num = a.length;
            alloc(num);
            double[] buff = new double[(num * 4)];
            for (int i = 0; i < num; i++) {
                Rect2d r = a[i];
                buff[(i * 4) + 0] = r.x;
                buff[(i * 4) + 1] = r.y;
                buff[(i * 4) + 2] = r.width;
                buff[(i * 4) + 3] = r.height;
            }
            put(0, 0, buff);
        }
    }

    public Rect2d[] toArray() {
        int num = (int) total();
        Rect2d[] a = new Rect2d[num];
        if (num == 0) {
            return a;
        }
        double[] buff = new double[(num * 4)];
        get(0, 0, buff);
        for (int i = 0; i < num; i++) {
            a[i] = new Rect2d(buff[i * 4], buff[(i * 4) + 1], buff[(i * 4) + 2], buff[(i * 4) + 3]);
        }
        return a;
    }

    public void fromList(List<Rect2d> lr) {
        fromArray((Rect2d[]) lr.toArray(new Rect2d[0]));
    }

    public List<Rect2d> toList() {
        return Arrays.asList(toArray());
    }
}
