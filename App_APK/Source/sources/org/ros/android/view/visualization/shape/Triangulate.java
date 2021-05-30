package org.ros.android.view.visualization.shape;

import java.util.List;

public class Triangulate {
    private static final float EPSILON = 1.0E-9f;

    public static class Point {
        private final float x;
        private final float y;

        public Point(float x2, float y2) {
            this.x = x2;
            this.y = y2;
        }

        public float x() {
            return this.x;
        }

        public float y() {
            return this.y;
        }
    }

    public static boolean process(Point[] contour, List<Point> result) {
        Point[] pointArr = contour;
        List<Point> list = result;
        int n = pointArr.length;
        if (n < 3) {
            return false;
        }
        int[] V = new int[n];
        if (0.0f < area(contour)) {
            for (int v = 0; v < n; v++) {
                V[v] = v;
            }
        } else {
            for (int v2 = 0; v2 < n; v2++) {
                V[v2] = (n - 1) - v2;
            }
        }
        int v3 = n;
        int b = v3 * 2;
        int v4 = v3 - 1;
        int nv = v3;
        int m = 0;
        while (true) {
            int v5 = v4;
            if (nv <= 2) {
                return true;
            }
            int count = b - 1;
            if (b <= 0) {
                return false;
            }
            int u = v5;
            if (nv <= u) {
                u = 0;
            }
            int u2 = u;
            int v6 = u2 + 1;
            if (nv <= v6) {
                v6 = 0;
            }
            int v7 = v6;
            int w = v7 + 1;
            if (nv <= w) {
                w = 0;
            }
            int w2 = w;
            if (snip(contour, u2, v7, w2, nv, V)) {
                int a = V[u2];
                int b2 = V[v7];
                int c = V[w2];
                list.add(pointArr[a]);
                list.add(pointArr[b2]);
                list.add(pointArr[c]);
                m++;
                int s = v7;
                for (int t = v7 + 1; t < nv; t++) {
                    V[s] = V[t];
                    s++;
                }
                nv--;
                b = nv * 2;
            } else {
                b = count;
            }
            v4 = v7;
        }
    }

    public static float area(Point[] contour) {
        int n = contour.length;
        float A = 0.0f;
        int p = n - 1;
        for (int q = 0; q < n; q++) {
            A += (contour[p].x() * contour[q].y()) - (contour[q].x() * contour[p].y());
            p = q;
        }
        return 0.5f * A;
    }

    public static boolean isInsideTriangle(float Ax, float Ay, float Bx, float By, float Cx, float Cy, float Px, float Py) {
        return ((Cx - Bx) * (Py - By)) - ((Cy - By) * (Px - Bx)) >= 0.0f && ((Ax - Cx) * (Py - Cy)) - ((Ay - Cy) * (Px - Cx)) >= 0.0f && ((Bx - Ax) * (Py - Ay)) - ((By - Ay) * (Px - Ax)) >= 0.0f;
    }

    private static boolean snip(Point[] contour, int u, int v, int w, int n, int[] V) {
        int p;
        int i = u;
        int i2 = v;
        int i3 = w;
        float Ax = contour[V[i]].x();
        float Ay = contour[V[i]].y();
        float Bx = contour[V[i2]].x();
        float By = contour[V[i2]].y();
        float Cx = contour[V[i3]].x();
        float Cy = contour[V[i3]].y();
        if (EPSILON > ((Bx - Ax) * (Cy - Ay)) - ((By - Ay) * (Cx - Ax))) {
            return false;
        }
        int p2 = 0;
        while (true) {
            int p3 = p2;
            if (p3 >= n) {
                return true;
            }
            if (p3 == i || p3 == i2) {
                p = p3;
            } else if (p3 == i3) {
                p = p3;
            } else {
                p = p3;
                if (isInsideTriangle(Ax, Ay, Bx, By, Cx, Cy, contour[V[p3]].x(), contour[V[p3]].y())) {
                    return false;
                }
            }
            p2 = p + 1;
        }
    }
}
