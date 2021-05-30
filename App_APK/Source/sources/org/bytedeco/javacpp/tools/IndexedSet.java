package org.bytedeco.javacpp.tools;

import java.util.Iterator;
import java.util.LinkedHashMap;

class IndexedSet<E> extends LinkedHashMap<E, Integer> implements Iterable<E> {
    IndexedSet() {
    }

    public int index(E e) {
        Integer i = (Integer) get(e);
        if (i == null) {
            Integer valueOf = Integer.valueOf(size());
            i = valueOf;
            put(e, valueOf);
        }
        return i.intValue();
    }

    public Iterator<E> iterator() {
        return keySet().iterator();
    }
}
