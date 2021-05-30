package org.apache.commons.lang.exception;

import java.io.PrintStream;
import java.io.PrintWriter;
import java.io.Serializable;
import java.io.StringWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;

public class NestableDelegate implements Serializable {
    private static final transient String MUST_BE_THROWABLE = "The Nestable implementation passed to the NestableDelegate(Nestable) constructor must extend java.lang.Throwable";
    static /* synthetic */ Class class$org$apache$commons$lang$exception$Nestable = null;
    public static boolean matchSubclasses = true;
    private static final long serialVersionUID = 1;
    public static boolean topDown = true;
    public static boolean trimStackFrames = true;
    private Throwable nestable = null;

    public NestableDelegate(Nestable nestable2) {
        if (nestable2 instanceof Throwable) {
            this.nestable = (Throwable) nestable2;
            return;
        }
        throw new IllegalArgumentException(MUST_BE_THROWABLE);
    }

    public String getMessage(int index) {
        Class cls;
        Throwable t = getThrowable(index);
        if (class$org$apache$commons$lang$exception$Nestable == null) {
            cls = class$("org.apache.commons.lang.exception.Nestable");
            class$org$apache$commons$lang$exception$Nestable = cls;
        } else {
            cls = class$org$apache$commons$lang$exception$Nestable;
        }
        if (cls.isInstance(t)) {
            return ((Nestable) t).getMessage(0);
        }
        return t.getMessage();
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError(x1.getMessage());
        }
    }

    public String getMessage(String baseMsg) {
        Throwable nestedCause = ExceptionUtils.getCause(this.nestable);
        String causeMsg = nestedCause == null ? null : nestedCause.getMessage();
        if (nestedCause == null || causeMsg == null) {
            return baseMsg;
        }
        if (baseMsg == null) {
            return causeMsg;
        }
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append(baseMsg);
        stringBuffer.append(": ");
        stringBuffer.append(causeMsg);
        return stringBuffer.toString();
    }

    public String[] getMessages() {
        Class cls;
        Throwable[] throwables = getThrowables();
        String[] msgs = new String[throwables.length];
        for (int i = 0; i < throwables.length; i++) {
            if (class$org$apache$commons$lang$exception$Nestable == null) {
                cls = class$("org.apache.commons.lang.exception.Nestable");
                class$org$apache$commons$lang$exception$Nestable = cls;
            } else {
                cls = class$org$apache$commons$lang$exception$Nestable;
            }
            msgs[i] = cls.isInstance(throwables[i]) ? ((Nestable) throwables[i]).getMessage(0) : throwables[i].getMessage();
        }
        return msgs;
    }

    public Throwable getThrowable(int index) {
        if (index == 0) {
            return this.nestable;
        }
        return getThrowables()[index];
    }

    public int getThrowableCount() {
        return ExceptionUtils.getThrowableCount(this.nestable);
    }

    public Throwable[] getThrowables() {
        return ExceptionUtils.getThrowables(this.nestable);
    }

    public int indexOfThrowable(Class type, int fromIndex) {
        if (type == null) {
            return -1;
        }
        if (fromIndex >= 0) {
            Throwable[] throwables = ExceptionUtils.getThrowables(this.nestable);
            if (fromIndex < throwables.length) {
                if (matchSubclasses) {
                    for (int i = fromIndex; i < throwables.length; i++) {
                        if (type.isAssignableFrom(throwables[i].getClass())) {
                            return i;
                        }
                    }
                } else {
                    for (int i2 = fromIndex; i2 < throwables.length; i2++) {
                        if (type.equals(throwables[i2].getClass())) {
                            return i2;
                        }
                    }
                }
                return -1;
            }
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("The start index was out of bounds: ");
            stringBuffer.append(fromIndex);
            stringBuffer.append(" >= ");
            stringBuffer.append(throwables.length);
            throw new IndexOutOfBoundsException(stringBuffer.toString());
        }
        StringBuffer stringBuffer2 = new StringBuffer();
        stringBuffer2.append("The start index was out of bounds: ");
        stringBuffer2.append(fromIndex);
        throw new IndexOutOfBoundsException(stringBuffer2.toString());
    }

    public void printStackTrace() {
        printStackTrace(System.err);
    }

    public void printStackTrace(PrintStream out) {
        synchronized (out) {
            PrintWriter pw = new PrintWriter(out, false);
            printStackTrace(pw);
            pw.flush();
        }
    }

    public void printStackTrace(PrintWriter out) {
        Throwable throwable = this.nestable;
        if (!ExceptionUtils.isThrowableNested()) {
            List stacks = new ArrayList();
            while (throwable != null) {
                stacks.add(getStackFrames(throwable));
                throwable = ExceptionUtils.getCause(throwable);
            }
            String separatorLine = "Caused by: ";
            if (!topDown) {
                separatorLine = "Rethrown as: ";
                Collections.reverse(stacks);
            }
            if (trimStackFrames) {
                trimStackFrames(stacks);
            }
            synchronized (out) {
                Iterator iter = stacks.iterator();
                while (iter.hasNext()) {
                    for (String println : (String[]) iter.next()) {
                        out.println(println);
                    }
                    if (iter.hasNext() != 0) {
                        out.print(separatorLine);
                    }
                }
            }
        } else if (throwable instanceof Nestable) {
            ((Nestable) throwable).printPartialStackTrace(out);
        } else {
            throwable.printStackTrace(out);
        }
    }

    /* access modifiers changed from: protected */
    public String[] getStackFrames(Throwable t) {
        StringWriter sw = new StringWriter();
        PrintWriter pw = new PrintWriter(sw, true);
        if (t instanceof Nestable) {
            ((Nestable) t).printPartialStackTrace(pw);
        } else {
            t.printStackTrace(pw);
        }
        return ExceptionUtils.getStackFrames(sw.getBuffer().toString());
    }

    /* access modifiers changed from: protected */
    public void trimStackFrames(List stacks) {
        for (int i = stacks.size() - 1; i > 0; i--) {
            String[] curr = (String[]) stacks.get(i);
            List currList = new ArrayList(Arrays.asList(curr));
            ExceptionUtils.removeCommonFrames(currList, new ArrayList(Arrays.asList((String[]) stacks.get(i - 1))));
            int trimmed = curr.length - currList.size();
            if (trimmed > 0) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("\t... ");
                stringBuffer.append(trimmed);
                stringBuffer.append(" more");
                currList.add(stringBuffer.toString());
                stacks.set(i, currList.toArray(new String[currList.size()]));
            }
        }
    }
}
