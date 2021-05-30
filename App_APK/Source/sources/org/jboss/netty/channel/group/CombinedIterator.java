package org.jboss.netty.channel.group;

import java.util.Iterator;
import java.util.NoSuchElementException;
import org.apache.xmlrpc.serializer.I1Serializer;
import org.apache.xmlrpc.serializer.I2Serializer;

final class CombinedIterator<E> implements Iterator<E> {
    private Iterator<E> currentIterator;
    private final Iterator<E> i1;
    private final Iterator<E> i2;

    CombinedIterator(Iterator<E> i12, Iterator<E> i22) {
        if (i12 == null) {
            throw new NullPointerException(I1Serializer.I1_TAG);
        } else if (i22 != null) {
            this.i1 = i12;
            this.i2 = i22;
            this.currentIterator = i12;
        } else {
            throw new NullPointerException(I2Serializer.I2_TAG);
        }
    }

    public boolean hasNext() {
        if (this.currentIterator.hasNext()) {
            return true;
        }
        if (this.currentIterator != this.i1) {
            return false;
        }
        this.currentIterator = this.i2;
        return hasNext();
    }

    public E next() {
        try {
            return this.currentIterator.next();
        } catch (NoSuchElementException e) {
            if (this.currentIterator == this.i1) {
                this.currentIterator = this.i2;
                return next();
            }
            throw e;
        }
    }

    public void remove() {
        this.currentIterator.remove();
    }
}
