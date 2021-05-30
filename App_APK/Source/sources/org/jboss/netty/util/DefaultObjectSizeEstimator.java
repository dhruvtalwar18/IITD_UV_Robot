package org.jboss.netty.util;

import java.lang.reflect.Field;
import java.nio.ByteBuffer;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.ConcurrentMap;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.channel.MessageEvent;
import org.jboss.netty.util.internal.ConcurrentIdentityWeakKeyHashMap;

public class DefaultObjectSizeEstimator implements ObjectSizeEstimator {
    private final ConcurrentMap<Class<?>, Integer> class2size = new ConcurrentIdentityWeakKeyHashMap();

    public DefaultObjectSizeEstimator() {
        this.class2size.put(Boolean.TYPE, 4);
        this.class2size.put(Byte.TYPE, 1);
        this.class2size.put(Character.TYPE, 2);
        this.class2size.put(Integer.TYPE, 4);
        this.class2size.put(Short.TYPE, 2);
        this.class2size.put(Long.TYPE, 8);
        this.class2size.put(Float.TYPE, 4);
        this.class2size.put(Double.TYPE, 8);
        this.class2size.put(Void.TYPE, 0);
    }

    public int estimateSize(Object o) {
        if (o == null) {
            return 8;
        }
        int answer = estimateSize(o.getClass(), (Set<Class<?>>) null) + 8;
        if (o instanceof EstimatableObjectWrapper) {
            answer += estimateSize(((EstimatableObjectWrapper) o).unwrap());
        } else if (o instanceof MessageEvent) {
            answer += estimateSize(((MessageEvent) o).getMessage());
        } else if (o instanceof ChannelBuffer) {
            answer += ((ChannelBuffer) o).capacity();
        } else if (o instanceof byte[]) {
            answer += ((byte[]) o).length;
        } else if (o instanceof ByteBuffer) {
            answer += ((ByteBuffer) o).remaining();
        } else if (o instanceof CharSequence) {
            answer += ((CharSequence) o).length() << 1;
        } else if (o instanceof Iterable) {
            for (Object m : (Iterable) o) {
                answer += estimateSize(m);
            }
        }
        return align(answer);
    }

    private int estimateSize(Class<?> clazz, Set<Class<?>> visitedClasses) {
        Integer objectSize = (Integer) this.class2size.get(clazz);
        if (objectSize != null) {
            return objectSize.intValue();
        }
        if (visitedClasses == null) {
            visitedClasses = new HashSet<>();
        } else if (visitedClasses.contains(clazz)) {
            return 0;
        }
        visitedClasses.add(clazz);
        int answer = 8;
        Class<?> c = clazz;
        while (c != null) {
            int answer2 = answer;
            for (Field f : c.getDeclaredFields()) {
                if ((f.getModifiers() & 8) == 0) {
                    answer2 += estimateSize(f.getType(), visitedClasses);
                }
            }
            c = c.getSuperclass();
            answer = answer2;
        }
        visitedClasses.remove(clazz);
        int answer3 = align(answer);
        this.class2size.putIfAbsent(clazz, Integer.valueOf(answer3));
        return answer3;
    }

    private static int align(int size) {
        int r = size % 8;
        if (r != 0) {
            return size + (8 - r);
        }
        return size;
    }
}
