package org.ros.concurrent;

import java.util.Iterator;
import java.util.NoSuchElementException;

public class CircularBlockingDeque<T> implements Iterable<T> {
    /* access modifiers changed from: private */
    public final T[] deque;
    /* access modifiers changed from: private */
    public int length;
    /* access modifiers changed from: private */
    public final int limit;
    private final Object mutex = new Object();
    /* access modifiers changed from: private */
    public int start;

    public CircularBlockingDeque(int capacity) {
        this.deque = (Object[]) new Object[capacity];
        this.limit = capacity;
        this.start = 0;
        this.length = 0;
    }

    public boolean addLast(T entry) {
        synchronized (this.mutex) {
            this.deque[(this.start + this.length) % this.limit] = entry;
            if (this.length == this.limit) {
                this.start = (this.start + 1) % this.limit;
            } else {
                this.length++;
            }
            this.mutex.notify();
        }
        return true;
    }

    public boolean addFirst(T entry) {
        synchronized (this.mutex) {
            if (this.start - 1 < 0) {
                this.start = this.limit - 1;
            } else {
                this.start--;
            }
            this.deque[this.start] = entry;
            if (this.length < this.limit) {
                this.length++;
            }
            this.mutex.notify();
        }
        return true;
    }

    public T takeFirst() throws InterruptedException {
        T entry;
        synchronized (this.mutex) {
            while (this.length <= 0) {
                this.mutex.wait();
            }
            entry = this.deque[this.start];
            this.start = (this.start + 1) % this.limit;
            this.length--;
        }
        return entry;
    }

    public T peekFirst() {
        synchronized (this.mutex) {
            if (this.length <= 0) {
                return null;
            }
            T t = this.deque[this.start];
            return t;
        }
    }

    public T takeLast() throws InterruptedException {
        T entry;
        synchronized (this.mutex) {
            while (this.length <= 0) {
                this.mutex.wait();
            }
            entry = this.deque[((this.start + this.length) - 1) % this.limit];
            this.length--;
        }
        return entry;
    }

    public T peekLast() {
        synchronized (this.mutex) {
            if (this.length <= 0) {
                return null;
            }
            T t = this.deque[((this.start + this.length) - 1) % this.limit];
            return t;
        }
    }

    public boolean isEmpty() {
        return this.length == 0;
    }

    public Iterator<T> iterator() {
        return new Iterator<T>() {
            int offset = 0;

            public boolean hasNext() {
                return this.offset < CircularBlockingDeque.this.length;
            }

            public T next() {
                if (this.offset != CircularBlockingDeque.this.length) {
                    T entry = CircularBlockingDeque.this.deque[(CircularBlockingDeque.this.start + this.offset) % CircularBlockingDeque.this.limit];
                    this.offset++;
                    return entry;
                }
                throw new NoSuchElementException();
            }

            public void remove() {
                throw new UnsupportedOperationException();
            }
        };
    }
}
