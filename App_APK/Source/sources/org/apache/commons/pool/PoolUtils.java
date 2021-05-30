package org.apache.commons.pool;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Timer;
import java.util.TimerTask;

public final class PoolUtils {
    private static Timer MIN_IDLE_TIMER;

    public static void checkRethrow(Throwable t) {
        if (t instanceof ThreadDeath) {
            throw ((ThreadDeath) t);
        } else if (t instanceof VirtualMachineError) {
            throw ((VirtualMachineError) t);
        }
    }

    public static <V> PoolableObjectFactory<V> adapt(KeyedPoolableObjectFactory<Object, V> keyedFactory) throws IllegalArgumentException {
        return adapt(keyedFactory, new Object());
    }

    public static <K, V> PoolableObjectFactory<V> adapt(KeyedPoolableObjectFactory<K, V> keyedFactory, K key) throws IllegalArgumentException {
        return new PoolableObjectFactoryAdaptor(keyedFactory, key);
    }

    public static <K, V> KeyedPoolableObjectFactory<K, V> adapt(PoolableObjectFactory<V> factory) throws IllegalArgumentException {
        return new KeyedPoolableObjectFactoryAdaptor(factory);
    }

    public static <V> ObjectPool<V> adapt(KeyedObjectPool<Object, V> keyedPool) throws IllegalArgumentException {
        return adapt(keyedPool, new Object());
    }

    public static <V> ObjectPool<V> adapt(KeyedObjectPool<Object, V> keyedPool, Object key) throws IllegalArgumentException {
        return new ObjectPoolAdaptor(keyedPool, key);
    }

    public static <K, V> KeyedObjectPool<K, V> adapt(ObjectPool<V> pool) throws IllegalArgumentException {
        return new KeyedObjectPoolAdaptor(pool);
    }

    public static <T> ObjectPool<T> checkedPool(ObjectPool<T> pool, Class<T> type) {
        if (pool == null) {
            throw new IllegalArgumentException("pool must not be null.");
        } else if (type != null) {
            return new CheckedObjectPool(pool, type);
        } else {
            throw new IllegalArgumentException("type must not be null.");
        }
    }

    public static <K, V> KeyedObjectPool<K, V> checkedPool(KeyedObjectPool<K, V> keyedPool, Class<V> type) {
        if (keyedPool == null) {
            throw new IllegalArgumentException("keyedPool must not be null.");
        } else if (type != null) {
            return new CheckedKeyedObjectPool(keyedPool, type);
        } else {
            throw new IllegalArgumentException("type must not be null.");
        }
    }

    public static <T> TimerTask checkMinIdle(ObjectPool<T> pool, int minIdle, long period) throws IllegalArgumentException {
        if (pool == null) {
            throw new IllegalArgumentException("keyedPool must not be null.");
        } else if (minIdle >= 0) {
            ObjectPoolMinIdleTimerTask objectPoolMinIdleTimerTask = new ObjectPoolMinIdleTimerTask(pool, minIdle);
            getMinIdleTimer().schedule(objectPoolMinIdleTimerTask, 0, period);
            return objectPoolMinIdleTimerTask;
        } else {
            throw new IllegalArgumentException("minIdle must be non-negative.");
        }
    }

    public static <K, V> TimerTask checkMinIdle(KeyedObjectPool<K, V> keyedPool, K key, int minIdle, long period) throws IllegalArgumentException {
        if (keyedPool == null) {
            throw new IllegalArgumentException("keyedPool must not be null.");
        } else if (key == null) {
            throw new IllegalArgumentException("key must not be null.");
        } else if (minIdle >= 0) {
            KeyedObjectPoolMinIdleTimerTask keyedObjectPoolMinIdleTimerTask = new KeyedObjectPoolMinIdleTimerTask(keyedPool, key, minIdle);
            getMinIdleTimer().schedule(keyedObjectPoolMinIdleTimerTask, 0, period);
            return keyedObjectPoolMinIdleTimerTask;
        } else {
            throw new IllegalArgumentException("minIdle must be non-negative.");
        }
    }

    public static <K, V> Map<K, TimerTask> checkMinIdle(KeyedObjectPool<K, V> keyedPool, Collection<? extends K> keys, int minIdle, long period) throws IllegalArgumentException {
        if (keys != null) {
            Map<K, TimerTask> tasks = new HashMap<>(keys.size());
            for (K key : keys) {
                tasks.put(key, checkMinIdle(keyedPool, key, minIdle, period));
            }
            return tasks;
        }
        throw new IllegalArgumentException("keys must not be null.");
    }

    public static <T> void prefill(ObjectPool<T> pool, int count) throws Exception, IllegalArgumentException {
        if (pool != null) {
            for (int i = 0; i < count; i++) {
                pool.addObject();
            }
            return;
        }
        throw new IllegalArgumentException("pool must not be null.");
    }

    public static <K, V> void prefill(KeyedObjectPool<K, V> keyedPool, K key, int count) throws Exception, IllegalArgumentException {
        if (keyedPool == null) {
            throw new IllegalArgumentException("keyedPool must not be null.");
        } else if (key != null) {
            for (int i = 0; i < count; i++) {
                keyedPool.addObject(key);
            }
        } else {
            throw new IllegalArgumentException("key must not be null.");
        }
    }

    public static <K, V> void prefill(KeyedObjectPool<K, V> keyedPool, Collection<? extends K> keys, int count) throws Exception, IllegalArgumentException {
        if (keys != null) {
            for (Object prefill : keys) {
                prefill(keyedPool, prefill, count);
            }
            return;
        }
        throw new IllegalArgumentException("keys must not be null.");
    }

    public static <T> ObjectPool<T> synchronizedPool(ObjectPool<T> pool) {
        if (pool != null) {
            return new SynchronizedObjectPool(pool);
        }
        throw new IllegalArgumentException("pool must not be null.");
    }

    public static <K, V> KeyedObjectPool<K, V> synchronizedPool(KeyedObjectPool<K, V> keyedPool) {
        if (keyedPool != null) {
            return new SynchronizedKeyedObjectPool(keyedPool);
        }
        throw new IllegalArgumentException("keyedPool must not be null.");
    }

    public static <T> PoolableObjectFactory<T> synchronizedPoolableFactory(PoolableObjectFactory<T> factory) {
        return new SynchronizedPoolableObjectFactory(factory);
    }

    public static <K, V> KeyedPoolableObjectFactory<K, V> synchronizedPoolableFactory(KeyedPoolableObjectFactory<K, V> keyedFactory) {
        return new SynchronizedKeyedPoolableObjectFactory(keyedFactory);
    }

    public static <T> ObjectPool<T> erodingPool(ObjectPool<T> pool) {
        return erodingPool(pool, 1.0f);
    }

    public static <T> ObjectPool<T> erodingPool(ObjectPool<T> pool, float factor) {
        if (pool == null) {
            throw new IllegalArgumentException("pool must not be null.");
        } else if (factor > 0.0f) {
            return new ErodingObjectPool(pool, factor);
        } else {
            throw new IllegalArgumentException("factor must be positive.");
        }
    }

    public static <K, V> KeyedObjectPool<K, V> erodingPool(KeyedObjectPool<K, V> keyedPool) {
        return erodingPool(keyedPool, 1.0f);
    }

    public static <K, V> KeyedObjectPool<K, V> erodingPool(KeyedObjectPool<K, V> keyedPool, float factor) {
        return erodingPool(keyedPool, factor, false);
    }

    public static <K, V> KeyedObjectPool<K, V> erodingPool(KeyedObjectPool<K, V> keyedPool, float factor, boolean perKey) {
        if (keyedPool == null) {
            throw new IllegalArgumentException("keyedPool must not be null.");
        } else if (factor <= 0.0f) {
            throw new IllegalArgumentException("factor must be positive.");
        } else if (perKey) {
            return new ErodingPerKeyKeyedObjectPool(keyedPool, factor);
        } else {
            return new ErodingKeyedObjectPool(keyedPool, factor);
        }
    }

    private static synchronized Timer getMinIdleTimer() {
        Timer timer;
        synchronized (PoolUtils.class) {
            if (MIN_IDLE_TIMER == null) {
                MIN_IDLE_TIMER = new Timer(true);
            }
            timer = MIN_IDLE_TIMER;
        }
        return timer;
    }

    private static class PoolableObjectFactoryAdaptor<K, V> implements PoolableObjectFactory<V> {
        private final K key;
        private final KeyedPoolableObjectFactory<K, V> keyedFactory;

        PoolableObjectFactoryAdaptor(KeyedPoolableObjectFactory<K, V> keyedFactory2, K key2) throws IllegalArgumentException {
            if (keyedFactory2 == null) {
                throw new IllegalArgumentException("keyedFactory must not be null.");
            } else if (key2 != null) {
                this.keyedFactory = keyedFactory2;
                this.key = key2;
            } else {
                throw new IllegalArgumentException("key must not be null.");
            }
        }

        public V makeObject() throws Exception {
            return this.keyedFactory.makeObject(this.key);
        }

        public void destroyObject(V obj) throws Exception {
            this.keyedFactory.destroyObject(this.key, obj);
        }

        public boolean validateObject(V obj) {
            return this.keyedFactory.validateObject(this.key, obj);
        }

        public void activateObject(V obj) throws Exception {
            this.keyedFactory.activateObject(this.key, obj);
        }

        public void passivateObject(V obj) throws Exception {
            this.keyedFactory.passivateObject(this.key, obj);
        }

        public String toString() {
            StringBuffer sb = new StringBuffer();
            sb.append("PoolableObjectFactoryAdaptor");
            sb.append("{key=");
            sb.append(this.key);
            sb.append(", keyedFactory=");
            sb.append(this.keyedFactory);
            sb.append('}');
            return sb.toString();
        }
    }

    private static class KeyedPoolableObjectFactoryAdaptor<K, V> implements KeyedPoolableObjectFactory<K, V> {
        private final PoolableObjectFactory<V> factory;

        KeyedPoolableObjectFactoryAdaptor(PoolableObjectFactory<V> factory2) throws IllegalArgumentException {
            if (factory2 != null) {
                this.factory = factory2;
                return;
            }
            throw new IllegalArgumentException("factory must not be null.");
        }

        public V makeObject(K k) throws Exception {
            return this.factory.makeObject();
        }

        public void destroyObject(K k, V obj) throws Exception {
            this.factory.destroyObject(obj);
        }

        public boolean validateObject(K k, V obj) {
            return this.factory.validateObject(obj);
        }

        public void activateObject(K k, V obj) throws Exception {
            this.factory.activateObject(obj);
        }

        public void passivateObject(K k, V obj) throws Exception {
            this.factory.passivateObject(obj);
        }

        public String toString() {
            StringBuffer sb = new StringBuffer();
            sb.append("KeyedPoolableObjectFactoryAdaptor");
            sb.append("{factory=");
            sb.append(this.factory);
            sb.append('}');
            return sb.toString();
        }
    }

    private static class ObjectPoolAdaptor<V> implements ObjectPool<V> {
        private final Object key;
        private final KeyedObjectPool<Object, V> keyedPool;

        ObjectPoolAdaptor(KeyedObjectPool<Object, V> keyedPool2, Object key2) throws IllegalArgumentException {
            if (keyedPool2 == null) {
                throw new IllegalArgumentException("keyedPool must not be null.");
            } else if (key2 != null) {
                this.keyedPool = keyedPool2;
                this.key = key2;
            } else {
                throw new IllegalArgumentException("key must not be null.");
            }
        }

        public V borrowObject() throws Exception, NoSuchElementException, IllegalStateException {
            return this.keyedPool.borrowObject(this.key);
        }

        public void returnObject(V obj) {
            try {
                this.keyedPool.returnObject(this.key, obj);
            } catch (Exception e) {
            }
        }

        public void invalidateObject(V obj) {
            try {
                this.keyedPool.invalidateObject(this.key, obj);
            } catch (Exception e) {
            }
        }

        public void addObject() throws Exception, IllegalStateException {
            this.keyedPool.addObject(this.key);
        }

        public int getNumIdle() throws UnsupportedOperationException {
            return this.keyedPool.getNumIdle(this.key);
        }

        public int getNumActive() throws UnsupportedOperationException {
            return this.keyedPool.getNumActive(this.key);
        }

        public void clear() throws Exception, UnsupportedOperationException {
            this.keyedPool.clear();
        }

        public void close() {
            try {
                this.keyedPool.close();
            } catch (Exception e) {
            }
        }

        @Deprecated
        public void setFactory(PoolableObjectFactory<V> factory) throws IllegalStateException, UnsupportedOperationException {
            this.keyedPool.setFactory(PoolUtils.adapt(factory));
        }

        public String toString() {
            StringBuffer sb = new StringBuffer();
            sb.append("ObjectPoolAdaptor");
            sb.append("{key=");
            sb.append(this.key);
            sb.append(", keyedPool=");
            sb.append(this.keyedPool);
            sb.append('}');
            return sb.toString();
        }
    }

    private static class KeyedObjectPoolAdaptor<K, V> implements KeyedObjectPool<K, V> {
        private final ObjectPool<V> pool;

        KeyedObjectPoolAdaptor(ObjectPool<V> pool2) throws IllegalArgumentException {
            if (pool2 != null) {
                this.pool = pool2;
                return;
            }
            throw new IllegalArgumentException("pool must not be null.");
        }

        public V borrowObject(K k) throws Exception, NoSuchElementException, IllegalStateException {
            return this.pool.borrowObject();
        }

        public void returnObject(K k, V obj) {
            try {
                this.pool.returnObject(obj);
            } catch (Exception e) {
            }
        }

        public void invalidateObject(K k, V obj) {
            try {
                this.pool.invalidateObject(obj);
            } catch (Exception e) {
            }
        }

        public void addObject(K k) throws Exception, IllegalStateException {
            this.pool.addObject();
        }

        public int getNumIdle(K k) throws UnsupportedOperationException {
            return this.pool.getNumIdle();
        }

        public int getNumActive(K k) throws UnsupportedOperationException {
            return this.pool.getNumActive();
        }

        public int getNumIdle() throws UnsupportedOperationException {
            return this.pool.getNumIdle();
        }

        public int getNumActive() throws UnsupportedOperationException {
            return this.pool.getNumActive();
        }

        public void clear() throws Exception, UnsupportedOperationException {
            this.pool.clear();
        }

        public void clear(K k) throws Exception, UnsupportedOperationException {
            this.pool.clear();
        }

        public void close() {
            try {
                this.pool.close();
            } catch (Exception e) {
            }
        }

        @Deprecated
        public void setFactory(KeyedPoolableObjectFactory<K, V> factory) throws IllegalStateException, UnsupportedOperationException {
            this.pool.setFactory(PoolUtils.adapt(factory));
        }

        public String toString() {
            StringBuffer sb = new StringBuffer();
            sb.append("KeyedObjectPoolAdaptor");
            sb.append("{pool=");
            sb.append(this.pool);
            sb.append('}');
            return sb.toString();
        }
    }

    private static class CheckedObjectPool<T> implements ObjectPool<T> {
        private final ObjectPool<T> pool;
        private final Class<T> type;

        CheckedObjectPool(ObjectPool<T> pool2, Class<T> type2) {
            if (pool2 == null) {
                throw new IllegalArgumentException("pool must not be null.");
            } else if (type2 != null) {
                this.pool = pool2;
                this.type = type2;
            } else {
                throw new IllegalArgumentException("type must not be null.");
            }
        }

        public T borrowObject() throws Exception, NoSuchElementException, IllegalStateException {
            T obj = this.pool.borrowObject();
            if (this.type.isInstance(obj)) {
                return obj;
            }
            throw new ClassCastException("Borrowed object is not of type: " + this.type.getName() + " was: " + obj);
        }

        public void returnObject(T obj) {
            if (this.type.isInstance(obj)) {
                try {
                    this.pool.returnObject(obj);
                } catch (Exception e) {
                }
            } else {
                throw new ClassCastException("Returned object is not of type: " + this.type.getName() + " was: " + obj);
            }
        }

        public void invalidateObject(T obj) {
            if (this.type.isInstance(obj)) {
                try {
                    this.pool.invalidateObject(obj);
                } catch (Exception e) {
                }
            } else {
                throw new ClassCastException("Invalidated object is not of type: " + this.type.getName() + " was: " + obj);
            }
        }

        public void addObject() throws Exception, IllegalStateException, UnsupportedOperationException {
            this.pool.addObject();
        }

        public int getNumIdle() throws UnsupportedOperationException {
            return this.pool.getNumIdle();
        }

        public int getNumActive() throws UnsupportedOperationException {
            return this.pool.getNumActive();
        }

        public void clear() throws Exception, UnsupportedOperationException {
            this.pool.clear();
        }

        public void close() {
            try {
                this.pool.close();
            } catch (Exception e) {
            }
        }

        @Deprecated
        public void setFactory(PoolableObjectFactory<T> factory) throws IllegalStateException, UnsupportedOperationException {
            this.pool.setFactory(factory);
        }

        public String toString() {
            StringBuffer sb = new StringBuffer();
            sb.append("CheckedObjectPool");
            sb.append("{type=");
            sb.append(this.type);
            sb.append(", pool=");
            sb.append(this.pool);
            sb.append('}');
            return sb.toString();
        }
    }

    private static class CheckedKeyedObjectPool<K, V> implements KeyedObjectPool<K, V> {
        private final KeyedObjectPool<K, V> keyedPool;
        private final Class<V> type;

        CheckedKeyedObjectPool(KeyedObjectPool<K, V> keyedPool2, Class<V> type2) {
            if (keyedPool2 == null) {
                throw new IllegalArgumentException("keyedPool must not be null.");
            } else if (type2 != null) {
                this.keyedPool = keyedPool2;
                this.type = type2;
            } else {
                throw new IllegalArgumentException("type must not be null.");
            }
        }

        public V borrowObject(K key) throws Exception, NoSuchElementException, IllegalStateException {
            V obj = this.keyedPool.borrowObject(key);
            if (this.type.isInstance(obj)) {
                return obj;
            }
            throw new ClassCastException("Borrowed object for key: " + key + " is not of type: " + this.type.getName() + " was: " + obj);
        }

        public void returnObject(K key, V obj) {
            if (this.type.isInstance(obj)) {
                try {
                    this.keyedPool.returnObject(key, obj);
                } catch (Exception e) {
                }
            } else {
                throw new ClassCastException("Returned object for key: " + key + " is not of type: " + this.type.getName() + " was: " + obj);
            }
        }

        public void invalidateObject(K key, V obj) {
            if (this.type.isInstance(obj)) {
                try {
                    this.keyedPool.invalidateObject(key, obj);
                } catch (Exception e) {
                }
            } else {
                throw new ClassCastException("Invalidated object for key: " + key + " is not of type: " + this.type.getName() + " was: " + obj);
            }
        }

        public void addObject(K key) throws Exception, IllegalStateException, UnsupportedOperationException {
            this.keyedPool.addObject(key);
        }

        public int getNumIdle(K key) throws UnsupportedOperationException {
            return this.keyedPool.getNumIdle(key);
        }

        public int getNumActive(K key) throws UnsupportedOperationException {
            return this.keyedPool.getNumActive(key);
        }

        public int getNumIdle() throws UnsupportedOperationException {
            return this.keyedPool.getNumIdle();
        }

        public int getNumActive() throws UnsupportedOperationException {
            return this.keyedPool.getNumActive();
        }

        public void clear() throws Exception, UnsupportedOperationException {
            this.keyedPool.clear();
        }

        public void clear(K key) throws Exception, UnsupportedOperationException {
            this.keyedPool.clear(key);
        }

        public void close() {
            try {
                this.keyedPool.close();
            } catch (Exception e) {
            }
        }

        @Deprecated
        public void setFactory(KeyedPoolableObjectFactory<K, V> factory) throws IllegalStateException, UnsupportedOperationException {
            this.keyedPool.setFactory(factory);
        }

        public String toString() {
            StringBuffer sb = new StringBuffer();
            sb.append("CheckedKeyedObjectPool");
            sb.append("{type=");
            sb.append(this.type);
            sb.append(", keyedPool=");
            sb.append(this.keyedPool);
            sb.append('}');
            return sb.toString();
        }
    }

    private static class ObjectPoolMinIdleTimerTask<T> extends TimerTask {
        private final int minIdle;
        private final ObjectPool<T> pool;

        ObjectPoolMinIdleTimerTask(ObjectPool<T> pool2, int minIdle2) throws IllegalArgumentException {
            if (pool2 != null) {
                this.pool = pool2;
                this.minIdle = minIdle2;
                return;
            }
            throw new IllegalArgumentException("pool must not be null.");
        }

        public void run() {
            try {
                if (this.pool.getNumIdle() < this.minIdle) {
                    this.pool.addObject();
                }
                if (1 != 0) {
                    return;
                }
            } catch (Exception e) {
                cancel();
                if (0 != 0) {
                    return;
                }
            } catch (Throwable th) {
                if (0 == 0) {
                    cancel();
                }
                throw th;
            }
            cancel();
        }

        public String toString() {
            StringBuffer sb = new StringBuffer();
            sb.append("ObjectPoolMinIdleTimerTask");
            sb.append("{minIdle=");
            sb.append(this.minIdle);
            sb.append(", pool=");
            sb.append(this.pool);
            sb.append('}');
            return sb.toString();
        }
    }

    private static class KeyedObjectPoolMinIdleTimerTask<K, V> extends TimerTask {
        private final K key;
        private final KeyedObjectPool<K, V> keyedPool;
        private final int minIdle;

        KeyedObjectPoolMinIdleTimerTask(KeyedObjectPool<K, V> keyedPool2, K key2, int minIdle2) throws IllegalArgumentException {
            if (keyedPool2 != null) {
                this.keyedPool = keyedPool2;
                this.key = key2;
                this.minIdle = minIdle2;
                return;
            }
            throw new IllegalArgumentException("keyedPool must not be null.");
        }

        public void run() {
            try {
                if (this.keyedPool.getNumIdle(this.key) < this.minIdle) {
                    this.keyedPool.addObject(this.key);
                }
                if (1 != 0) {
                    return;
                }
            } catch (Exception e) {
                cancel();
                if (0 != 0) {
                    return;
                }
            } catch (Throwable th) {
                if (0 == 0) {
                    cancel();
                }
                throw th;
            }
            cancel();
        }

        public String toString() {
            StringBuffer sb = new StringBuffer();
            sb.append("KeyedObjectPoolMinIdleTimerTask");
            sb.append("{minIdle=");
            sb.append(this.minIdle);
            sb.append(", key=");
            sb.append(this.key);
            sb.append(", keyedPool=");
            sb.append(this.keyedPool);
            sb.append('}');
            return sb.toString();
        }
    }

    private static class SynchronizedObjectPool<T> implements ObjectPool<T> {
        private final Object lock;
        private final ObjectPool<T> pool;

        SynchronizedObjectPool(ObjectPool<T> pool2) throws IllegalArgumentException {
            if (pool2 != null) {
                this.pool = pool2;
                this.lock = new Object();
                return;
            }
            throw new IllegalArgumentException("pool must not be null.");
        }

        public T borrowObject() throws Exception, NoSuchElementException, IllegalStateException {
            T borrowObject;
            synchronized (this.lock) {
                borrowObject = this.pool.borrowObject();
            }
            return borrowObject;
        }

        public void returnObject(T obj) {
            synchronized (this.lock) {
                try {
                    this.pool.returnObject(obj);
                } catch (Exception e) {
                }
            }
        }

        public void invalidateObject(T obj) {
            synchronized (this.lock) {
                try {
                    this.pool.invalidateObject(obj);
                } catch (Exception e) {
                }
            }
        }

        public void addObject() throws Exception, IllegalStateException, UnsupportedOperationException {
            synchronized (this.lock) {
                this.pool.addObject();
            }
        }

        public int getNumIdle() throws UnsupportedOperationException {
            int numIdle;
            synchronized (this.lock) {
                numIdle = this.pool.getNumIdle();
            }
            return numIdle;
        }

        public int getNumActive() throws UnsupportedOperationException {
            int numActive;
            synchronized (this.lock) {
                numActive = this.pool.getNumActive();
            }
            return numActive;
        }

        public void clear() throws Exception, UnsupportedOperationException {
            synchronized (this.lock) {
                this.pool.clear();
            }
        }

        public void close() {
            try {
                synchronized (this.lock) {
                    this.pool.close();
                }
            } catch (Exception e) {
            }
        }

        @Deprecated
        public void setFactory(PoolableObjectFactory<T> factory) throws IllegalStateException, UnsupportedOperationException {
            synchronized (this.lock) {
                this.pool.setFactory(factory);
            }
        }

        public String toString() {
            StringBuffer sb = new StringBuffer();
            sb.append("SynchronizedObjectPool");
            sb.append("{pool=");
            sb.append(this.pool);
            sb.append('}');
            return sb.toString();
        }
    }

    private static class SynchronizedKeyedObjectPool<K, V> implements KeyedObjectPool<K, V> {
        private final KeyedObjectPool<K, V> keyedPool;
        private final Object lock;

        SynchronizedKeyedObjectPool(KeyedObjectPool<K, V> keyedPool2) throws IllegalArgumentException {
            if (keyedPool2 != null) {
                this.keyedPool = keyedPool2;
                this.lock = new Object();
                return;
            }
            throw new IllegalArgumentException("keyedPool must not be null.");
        }

        public V borrowObject(K key) throws Exception, NoSuchElementException, IllegalStateException {
            V borrowObject;
            synchronized (this.lock) {
                borrowObject = this.keyedPool.borrowObject(key);
            }
            return borrowObject;
        }

        public void returnObject(K key, V obj) {
            synchronized (this.lock) {
                try {
                    this.keyedPool.returnObject(key, obj);
                } catch (Exception e) {
                }
            }
        }

        public void invalidateObject(K key, V obj) {
            synchronized (this.lock) {
                try {
                    this.keyedPool.invalidateObject(key, obj);
                } catch (Exception e) {
                }
            }
        }

        public void addObject(K key) throws Exception, IllegalStateException, UnsupportedOperationException {
            synchronized (this.lock) {
                this.keyedPool.addObject(key);
            }
        }

        public int getNumIdle(K key) throws UnsupportedOperationException {
            int numIdle;
            synchronized (this.lock) {
                numIdle = this.keyedPool.getNumIdle(key);
            }
            return numIdle;
        }

        public int getNumActive(K key) throws UnsupportedOperationException {
            int numActive;
            synchronized (this.lock) {
                numActive = this.keyedPool.getNumActive(key);
            }
            return numActive;
        }

        public int getNumIdle() throws UnsupportedOperationException {
            int numIdle;
            synchronized (this.lock) {
                numIdle = this.keyedPool.getNumIdle();
            }
            return numIdle;
        }

        public int getNumActive() throws UnsupportedOperationException {
            int numActive;
            synchronized (this.lock) {
                numActive = this.keyedPool.getNumActive();
            }
            return numActive;
        }

        public void clear() throws Exception, UnsupportedOperationException {
            synchronized (this.lock) {
                this.keyedPool.clear();
            }
        }

        public void clear(K key) throws Exception, UnsupportedOperationException {
            synchronized (this.lock) {
                this.keyedPool.clear(key);
            }
        }

        public void close() {
            try {
                synchronized (this.lock) {
                    this.keyedPool.close();
                }
            } catch (Exception e) {
            }
        }

        @Deprecated
        public void setFactory(KeyedPoolableObjectFactory<K, V> factory) throws IllegalStateException, UnsupportedOperationException {
            synchronized (this.lock) {
                this.keyedPool.setFactory(factory);
            }
        }

        public String toString() {
            StringBuffer sb = new StringBuffer();
            sb.append("SynchronizedKeyedObjectPool");
            sb.append("{keyedPool=");
            sb.append(this.keyedPool);
            sb.append('}');
            return sb.toString();
        }
    }

    private static class SynchronizedPoolableObjectFactory<T> implements PoolableObjectFactory<T> {
        private final PoolableObjectFactory<T> factory;
        private final Object lock;

        SynchronizedPoolableObjectFactory(PoolableObjectFactory<T> factory2) throws IllegalArgumentException {
            if (factory2 != null) {
                this.factory = factory2;
                this.lock = new Object();
                return;
            }
            throw new IllegalArgumentException("factory must not be null.");
        }

        public T makeObject() throws Exception {
            T makeObject;
            synchronized (this.lock) {
                makeObject = this.factory.makeObject();
            }
            return makeObject;
        }

        public void destroyObject(T obj) throws Exception {
            synchronized (this.lock) {
                this.factory.destroyObject(obj);
            }
        }

        public boolean validateObject(T obj) {
            boolean validateObject;
            synchronized (this.lock) {
                validateObject = this.factory.validateObject(obj);
            }
            return validateObject;
        }

        public void activateObject(T obj) throws Exception {
            synchronized (this.lock) {
                this.factory.activateObject(obj);
            }
        }

        public void passivateObject(T obj) throws Exception {
            synchronized (this.lock) {
                this.factory.passivateObject(obj);
            }
        }

        public String toString() {
            StringBuffer sb = new StringBuffer();
            sb.append("SynchronizedPoolableObjectFactory");
            sb.append("{factory=");
            sb.append(this.factory);
            sb.append('}');
            return sb.toString();
        }
    }

    private static class SynchronizedKeyedPoolableObjectFactory<K, V> implements KeyedPoolableObjectFactory<K, V> {
        private final KeyedPoolableObjectFactory<K, V> keyedFactory;
        private final Object lock;

        SynchronizedKeyedPoolableObjectFactory(KeyedPoolableObjectFactory<K, V> keyedFactory2) throws IllegalArgumentException {
            if (keyedFactory2 != null) {
                this.keyedFactory = keyedFactory2;
                this.lock = new Object();
                return;
            }
            throw new IllegalArgumentException("keyedFactory must not be null.");
        }

        public V makeObject(K key) throws Exception {
            V makeObject;
            synchronized (this.lock) {
                makeObject = this.keyedFactory.makeObject(key);
            }
            return makeObject;
        }

        public void destroyObject(K key, V obj) throws Exception {
            synchronized (this.lock) {
                this.keyedFactory.destroyObject(key, obj);
            }
        }

        public boolean validateObject(K key, V obj) {
            boolean validateObject;
            synchronized (this.lock) {
                validateObject = this.keyedFactory.validateObject(key, obj);
            }
            return validateObject;
        }

        public void activateObject(K key, V obj) throws Exception {
            synchronized (this.lock) {
                this.keyedFactory.activateObject(key, obj);
            }
        }

        public void passivateObject(K key, V obj) throws Exception {
            synchronized (this.lock) {
                this.keyedFactory.passivateObject(key, obj);
            }
        }

        public String toString() {
            StringBuffer sb = new StringBuffer();
            sb.append("SynchronizedKeyedPoolableObjectFactory");
            sb.append("{keyedFactory=");
            sb.append(this.keyedFactory);
            sb.append('}');
            return sb.toString();
        }
    }

    private static class ErodingFactor {
        private final float factor;
        private volatile transient int idleHighWaterMark = 1;
        private volatile transient long nextShrink;

        public ErodingFactor(float factor2) {
            this.factor = factor2;
            this.nextShrink = System.currentTimeMillis() + ((long) (900000.0f * factor2));
        }

        public void update(int numIdle) {
            update(System.currentTimeMillis(), numIdle);
        }

        public void update(long now, int numIdle) {
            int idle = Math.max(0, numIdle);
            this.idleHighWaterMark = Math.max(idle, this.idleHighWaterMark);
            this.nextShrink = ((long) (60000.0f * (((-14.0f / ((float) this.idleHighWaterMark)) * ((float) idle)) + 15.0f) * this.factor)) + now;
        }

        public long getNextShrink() {
            return this.nextShrink;
        }

        public String toString() {
            return "ErodingFactor{factor=" + this.factor + ", idleHighWaterMark=" + this.idleHighWaterMark + '}';
        }
    }

    private static class ErodingObjectPool<T> implements ObjectPool<T> {
        private final ErodingFactor factor;
        private final ObjectPool<T> pool;

        public ErodingObjectPool(ObjectPool<T> pool2, float factor2) {
            this.pool = pool2;
            this.factor = new ErodingFactor(factor2);
        }

        public T borrowObject() throws Exception, NoSuchElementException, IllegalStateException {
            return this.pool.borrowObject();
        }

        public void returnObject(T obj) {
            boolean discard = false;
            long now = System.currentTimeMillis();
            synchronized (this.pool) {
                if (this.factor.getNextShrink() < now) {
                    int numIdle = this.pool.getNumIdle();
                    if (numIdle > 0) {
                        discard = true;
                    }
                    this.factor.update(now, numIdle);
                }
            }
            if (discard) {
                try {
                    this.pool.invalidateObject(obj);
                } catch (Exception e) {
                }
            } else {
                this.pool.returnObject(obj);
            }
        }

        public void invalidateObject(T obj) {
            try {
                this.pool.invalidateObject(obj);
            } catch (Exception e) {
            }
        }

        public void addObject() throws Exception, IllegalStateException, UnsupportedOperationException {
            this.pool.addObject();
        }

        public int getNumIdle() throws UnsupportedOperationException {
            return this.pool.getNumIdle();
        }

        public int getNumActive() throws UnsupportedOperationException {
            return this.pool.getNumActive();
        }

        public void clear() throws Exception, UnsupportedOperationException {
            this.pool.clear();
        }

        public void close() {
            try {
                this.pool.close();
            } catch (Exception e) {
            }
        }

        @Deprecated
        public void setFactory(PoolableObjectFactory<T> factory) throws IllegalStateException, UnsupportedOperationException {
            this.pool.setFactory(factory);
        }

        public String toString() {
            return "ErodingObjectPool{factor=" + this.factor + ", pool=" + this.pool + '}';
        }
    }

    private static class ErodingKeyedObjectPool<K, V> implements KeyedObjectPool<K, V> {
        private final ErodingFactor erodingFactor;
        private final KeyedObjectPool<K, V> keyedPool;

        public ErodingKeyedObjectPool(KeyedObjectPool<K, V> keyedPool2, float factor) {
            this(keyedPool2, new ErodingFactor(factor));
        }

        protected ErodingKeyedObjectPool(KeyedObjectPool<K, V> keyedPool2, ErodingFactor erodingFactor2) {
            if (keyedPool2 != null) {
                this.keyedPool = keyedPool2;
                this.erodingFactor = erodingFactor2;
                return;
            }
            throw new IllegalArgumentException("keyedPool must not be null.");
        }

        public V borrowObject(K key) throws Exception, NoSuchElementException, IllegalStateException {
            return this.keyedPool.borrowObject(key);
        }

        public void returnObject(K key, V obj) throws Exception {
            boolean discard = false;
            long now = System.currentTimeMillis();
            ErodingFactor factor = getErodingFactor(key);
            synchronized (this.keyedPool) {
                if (factor.getNextShrink() < now) {
                    int numIdle = numIdle(key);
                    if (numIdle > 0) {
                        discard = true;
                    }
                    factor.update(now, numIdle);
                }
            }
            if (discard) {
                try {
                    this.keyedPool.invalidateObject(key, obj);
                } catch (Exception e) {
                }
            } else {
                this.keyedPool.returnObject(key, obj);
            }
        }

        /* access modifiers changed from: protected */
        public int numIdle(K k) {
            return getKeyedPool().getNumIdle();
        }

        /* access modifiers changed from: protected */
        public ErodingFactor getErodingFactor(K k) {
            return this.erodingFactor;
        }

        public void invalidateObject(K key, V obj) {
            try {
                this.keyedPool.invalidateObject(key, obj);
            } catch (Exception e) {
            }
        }

        public void addObject(K key) throws Exception, IllegalStateException, UnsupportedOperationException {
            this.keyedPool.addObject(key);
        }

        public int getNumIdle() throws UnsupportedOperationException {
            return this.keyedPool.getNumIdle();
        }

        public int getNumIdle(K key) throws UnsupportedOperationException {
            return this.keyedPool.getNumIdle(key);
        }

        public int getNumActive() throws UnsupportedOperationException {
            return this.keyedPool.getNumActive();
        }

        public int getNumActive(K key) throws UnsupportedOperationException {
            return this.keyedPool.getNumActive(key);
        }

        public void clear() throws Exception, UnsupportedOperationException {
            this.keyedPool.clear();
        }

        public void clear(K key) throws Exception, UnsupportedOperationException {
            this.keyedPool.clear(key);
        }

        public void close() {
            try {
                this.keyedPool.close();
            } catch (Exception e) {
            }
        }

        @Deprecated
        public void setFactory(KeyedPoolableObjectFactory<K, V> factory) throws IllegalStateException, UnsupportedOperationException {
            this.keyedPool.setFactory(factory);
        }

        /* access modifiers changed from: protected */
        public KeyedObjectPool<K, V> getKeyedPool() {
            return this.keyedPool;
        }

        public String toString() {
            return "ErodingKeyedObjectPool{erodingFactor=" + this.erodingFactor + ", keyedPool=" + this.keyedPool + '}';
        }
    }

    private static class ErodingPerKeyKeyedObjectPool<K, V> extends ErodingKeyedObjectPool<K, V> {
        private final float factor;
        private final Map<K, ErodingFactor> factors = Collections.synchronizedMap(new HashMap());

        public ErodingPerKeyKeyedObjectPool(KeyedObjectPool<K, V> keyedPool, float factor2) {
            super(keyedPool, (ErodingFactor) null);
            this.factor = factor2;
        }

        /* access modifiers changed from: protected */
        public int numIdle(K key) {
            return getKeyedPool().getNumIdle(key);
        }

        /* access modifiers changed from: protected */
        public ErodingFactor getErodingFactor(K key) {
            ErodingFactor factor2 = this.factors.get(key);
            if (factor2 != null) {
                return factor2;
            }
            ErodingFactor factor3 = new ErodingFactor(this.factor);
            this.factors.put(key, factor3);
            return factor3;
        }

        public String toString() {
            return "ErodingPerKeyKeyedObjectPool{factor=" + this.factor + ", keyedPool=" + getKeyedPool() + '}';
        }
    }
}
