package org.ros.rosjava_geometry;

import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Preconditions;
import com.google.common.collect.Maps;
import geometry_msgs.TransformStamped;
import java.util.Iterator;
import java.util.Map;
import org.ros.concurrent.CircularBlockingDeque;
import org.ros.message.Time;
import org.ros.namespace.GraphName;

public class FrameTransformTree {
    private static final int TRANSFORM_QUEUE_CAPACITY = 16;
    private final Object mutex = new Object();
    private final Map<GraphName, CircularBlockingDeque<LazyFrameTransform>> transforms = Maps.newConcurrentMap();

    public void update(TransformStamped transformStamped) {
        Preconditions.checkNotNull(transformStamped);
        add(GraphName.of(transformStamped.getChildFrameId()), new LazyFrameTransform(transformStamped));
    }

    /* access modifiers changed from: package-private */
    @VisibleForTesting
    public void update(FrameTransform frameTransform) {
        Preconditions.checkNotNull(frameTransform);
        add(frameTransform.getSourceFrame(), new LazyFrameTransform(frameTransform));
    }

    private void add(GraphName source, LazyFrameTransform lazyFrameTransform) {
        GraphName relativeSource = source.toRelative();
        if (!this.transforms.containsKey(relativeSource)) {
            this.transforms.put(relativeSource, new CircularBlockingDeque(16));
        }
        synchronized (this.mutex) {
            this.transforms.get(relativeSource).addFirst(lazyFrameTransform);
        }
    }

    public FrameTransform lookUp(GraphName source) {
        Preconditions.checkNotNull(source);
        return getLatest(source.toRelative());
    }

    private FrameTransform getLatest(GraphName source) {
        LazyFrameTransform lazyFrameTransform;
        CircularBlockingDeque<LazyFrameTransform> deque = this.transforms.get(source);
        if (deque == null || (lazyFrameTransform = deque.peekFirst()) == null) {
            return null;
        }
        return lazyFrameTransform.get();
    }

    public FrameTransform get(String source) {
        Preconditions.checkNotNull(source);
        return lookUp(GraphName.of(source));
    }

    public FrameTransform lookUp(GraphName source, Time time) {
        Preconditions.checkNotNull(source);
        Preconditions.checkNotNull(time);
        return get(source, time);
    }

    private FrameTransform get(GraphName source, Time time) {
        CircularBlockingDeque<LazyFrameTransform> deque = this.transforms.get(source);
        if (deque == null) {
            return null;
        }
        LazyFrameTransform result = null;
        synchronized (this.mutex) {
            long offset = 0;
            Iterator<LazyFrameTransform> it = deque.iterator();
            while (it.hasNext()) {
                LazyFrameTransform lazyFrameTransform = it.next();
                if (result == null) {
                    result = lazyFrameTransform;
                    offset = Math.abs(time.subtract(result.get().getTime()).totalNsecs());
                } else {
                    long newOffset = Math.abs(time.subtract(lazyFrameTransform.get().getTime()).totalNsecs());
                    if (newOffset < offset) {
                        result = lazyFrameTransform;
                        offset = newOffset;
                    }
                }
            }
        }
        if (result == null) {
            return null;
        }
        return result.get();
    }

    public FrameTransform get(String source, Time time) {
        Preconditions.checkNotNull(source);
        return lookUp(GraphName.of(source), time);
    }

    public FrameTransform transform(GraphName source, GraphName target) {
        Preconditions.checkNotNull(source);
        Preconditions.checkNotNull(target);
        GraphName relativeSource = source.toRelative();
        GraphName relativeTarget = target.toRelative();
        if (relativeSource.equals(relativeTarget)) {
            return new FrameTransform(Transform.identity(), relativeSource, relativeTarget, (Time) null);
        }
        FrameTransform sourceToRoot = transformToRoot(relativeSource);
        FrameTransform targetToRoot = transformToRoot(relativeTarget);
        if (sourceToRoot == null && targetToRoot == null) {
            return null;
        }
        if (sourceToRoot == null) {
            if (targetToRoot.getTargetFrame().equals(relativeSource)) {
                return targetToRoot.invert();
            }
            return null;
        } else if (targetToRoot == null) {
            if (sourceToRoot.getTargetFrame().equals(relativeTarget)) {
                return sourceToRoot;
            }
            return null;
        } else if (sourceToRoot.getTargetFrame().equals(targetToRoot.getTargetFrame())) {
            return new FrameTransform(targetToRoot.getTransform().invert().multiply(sourceToRoot.getTransform()), relativeSource, relativeTarget, sourceToRoot.getTime());
        } else {
            return null;
        }
    }

    public FrameTransform transform(String source, String target) {
        Preconditions.checkNotNull(source);
        Preconditions.checkNotNull(target);
        return transform(GraphName.of(source), GraphName.of(target));
    }

    /* access modifiers changed from: package-private */
    @VisibleForTesting
    public FrameTransform transformToRoot(GraphName source) {
        FrameTransform result = getLatest(source);
        if (result == null) {
            return null;
        }
        while (true) {
            FrameTransform resultToParent = lookUp(result.getTargetFrame(), result.getTime());
            if (resultToParent == null) {
                return result;
            }
            result = new FrameTransform(resultToParent.getTransform().multiply(result.getTransform()), source, resultToParent.getTargetFrame(), result.getTime());
        }
    }
}
