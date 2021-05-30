package org.opencv.highgui;

import java.awt.Image;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;
import java.io.PrintStream;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;
import org.opencv.core.Mat;

public final class HighGui {
    public static final int WINDOW_AUTOSIZE = 1;
    public static final int WINDOW_NORMAL = 0;
    public static CountDownLatch latch = new CountDownLatch(1);
    public static int n_closed_windows = 0;
    public static int pressedKey = -1;
    public static Map<String, ImageWindow> windows = new HashMap();

    public static void namedWindow(String winname) {
        namedWindow(winname, 1);
    }

    public static void namedWindow(String winname, int flag) {
        ImageWindow newWin = new ImageWindow(winname, flag);
        if (windows.get(winname) == null) {
            windows.put(winname, newWin);
        }
    }

    public static void imshow(String winname, Mat img) {
        if (img.empty()) {
            System.err.println("Error: Empty image in imshow");
            System.exit(-1);
            return;
        }
        ImageWindow tmpWindow = windows.get(winname);
        if (tmpWindow == null) {
            windows.put(winname, new ImageWindow(winname, img));
            return;
        }
        tmpWindow.setMat(img);
    }

    public static Image toBufferedImage(Mat m) {
        int type = 10;
        if (m.channels() > 1) {
            type = 5;
        }
        byte[] b = new byte[(m.channels() * m.cols() * m.rows())];
        m.get(0, 0, b);
        BufferedImage image = new BufferedImage(m.cols(), m.rows(), type);
        System.arraycopy(b, 0, image.getRaster().getDataBuffer().getData(), 0, b.length);
        return image;
    }

    public static JFrame createJFrame(String title, int flag) {
        JFrame frame = new JFrame(title);
        frame.addWindowListener(new WindowAdapter() {
            public void windowClosing(WindowEvent windowEvent) {
                HighGui.n_closed_windows++;
                if (HighGui.n_closed_windows == HighGui.windows.size()) {
                    HighGui.latch.countDown();
                }
            }
        });
        frame.addKeyListener(new KeyListener() {
            public void keyTyped(KeyEvent e) {
            }

            public void keyReleased(KeyEvent e) {
            }

            public void keyPressed(KeyEvent e) {
                HighGui.pressedKey = e.getKeyCode();
                HighGui.latch.countDown();
            }
        });
        if (flag == 1) {
            frame.setResizable(false);
        }
        return frame;
    }

    public static void waitKey() {
        waitKey(0);
    }

    public static int waitKey(int delay) {
        latch = new CountDownLatch(1);
        n_closed_windows = 0;
        pressedKey = -1;
        if (windows.isEmpty()) {
            System.err.println("Error: waitKey must be used after an imshow");
            System.exit(-1);
        }
        Iterator<Map.Entry<String, ImageWindow>> iter = windows.entrySet().iterator();
        while (iter.hasNext()) {
            ImageWindow win = iter.next().getValue();
            if (win.alreadyUsed.booleanValue()) {
                iter.remove();
                win.frame.dispose();
            }
        }
        for (ImageWindow win2 : windows.values()) {
            if (win2.img != null) {
                ImageIcon icon = new ImageIcon(toBufferedImage(win2.img));
                if (win2.lbl == null) {
                    win2.setFrameLabelVisible(createJFrame(win2.name, win2.flag), new JLabel(icon));
                } else {
                    win2.lbl.setIcon(icon);
                }
            } else {
                PrintStream printStream = System.err;
                printStream.println("Error: no imshow associated with namedWindow: \"" + win2.name + "\"");
                System.exit(-1);
            }
        }
        if (delay == 0) {
            try {
                latch.await();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } else {
            latch.await((long) delay, TimeUnit.MILLISECONDS);
        }
        for (ImageWindow win3 : windows.values()) {
            win3.alreadyUsed = true;
        }
        return pressedKey;
    }

    public static void destroyWindow(String winname) {
        if (windows.get(winname) != null) {
            windows.remove(winname);
        }
    }

    public static void destroyAllWindows() {
        windows.clear();
    }

    public static void resizeWindow(String winname, int width, int height) {
        ImageWindow tmpWin = windows.get(winname);
        if (tmpWin != null) {
            tmpWin.setNewDimension(width, height);
        }
    }

    public static void moveWindow(String winname, int x, int y) {
        ImageWindow tmpWin = windows.get(winname);
        if (tmpWin != null) {
            tmpWin.setNewPosition(x, y);
        }
    }
}
