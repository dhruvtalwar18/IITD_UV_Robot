package com.google.common.eventbus;

import com.google.common.annotations.Beta;
import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Supplier;
import com.google.common.base.Throwables;
import com.google.common.cache.CacheBuilder;
import com.google.common.cache.CacheLoader;
import com.google.common.cache.LoadingCache;
import com.google.common.collect.Lists;
import com.google.common.collect.Multimaps;
import com.google.common.collect.SetMultimap;
import com.google.common.collect.Sets;
import java.lang.reflect.InvocationTargetException;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.CopyOnWriteArraySet;
import java.util.concurrent.ExecutionException;
import java.util.logging.Level;
import java.util.logging.Logger;
import org.apache.commons.httpclient.cookie.CookiePolicy;

@Beta
public class EventBus {
    private final ThreadLocal<ConcurrentLinkedQueue<EventWithHandler>> eventsToDispatch;
    private final HandlerFindingStrategy finder;
    private LoadingCache<Class<?>, Set<Class<?>>> flattenHierarchyCache;
    private final SetMultimap<Class<?>, EventHandler> handlersByType;
    private final ThreadLocal<Boolean> isDispatching;
    private final Logger logger;

    public EventBus() {
        this(CookiePolicy.DEFAULT);
    }

    public EventBus(String identifier) {
        this.handlersByType = Multimaps.newSetMultimap(new ConcurrentHashMap(), new Supplier<Set<EventHandler>>() {
            public Set<EventHandler> get() {
                return new CopyOnWriteArraySet();
            }
        });
        this.finder = new AnnotatedHandlerFinder();
        this.eventsToDispatch = new ThreadLocal<ConcurrentLinkedQueue<EventWithHandler>>() {
            /* access modifiers changed from: protected */
            public ConcurrentLinkedQueue<EventWithHandler> initialValue() {
                return new ConcurrentLinkedQueue<>();
            }
        };
        this.isDispatching = new ThreadLocal<Boolean>() {
            /* access modifiers changed from: protected */
            public Boolean initialValue() {
                return false;
            }
        };
        this.flattenHierarchyCache = CacheBuilder.newBuilder().weakKeys().build(new CacheLoader<Class<?>, Set<Class<?>>>() {
            public Set<Class<?>> load(Class<?> concreteClass) throws Exception {
                List<Class<?>> parents = Lists.newLinkedList();
                Set<Class<?>> classes = Sets.newHashSet();
                parents.add(concreteClass);
                while (!parents.isEmpty()) {
                    Class<?> clazz = parents.remove(0);
                    classes.add(clazz);
                    Class<? super Object> superclass = clazz.getSuperclass();
                    if (superclass != null) {
                        parents.add(superclass);
                    }
                    for (Class<?> iface : clazz.getInterfaces()) {
                        parents.add(iface);
                    }
                }
                return classes;
            }
        });
        this.logger = Logger.getLogger(EventBus.class.getName() + "." + identifier);
    }

    public void register(Object object) {
        this.handlersByType.putAll(this.finder.findAllHandlers(object));
    }

    public void unregister(Object object) {
        for (Map.Entry<Class<?>, Collection<EventHandler>> entry : this.finder.findAllHandlers(object).asMap().entrySet()) {
            Set<EventHandler> currentHandlers = getHandlersForEventType(entry.getKey());
            Collection<EventHandler> eventMethodsInListener = entry.getValue();
            if (currentHandlers == null || !currentHandlers.containsAll(entry.getValue())) {
                throw new IllegalArgumentException("missing event handler for an annotated method. Is " + object + " registered?");
            }
            currentHandlers.removeAll(eventMethodsInListener);
        }
    }

    public void post(Object event) {
        boolean dispatched = false;
        for (Class<?> eventType : flattenHierarchy(event.getClass())) {
            Set<EventHandler> wrappers = getHandlersForEventType(eventType);
            if (wrappers != null && !wrappers.isEmpty()) {
                dispatched = true;
                for (EventHandler wrapper : wrappers) {
                    enqueueEvent(event, wrapper);
                }
            }
        }
        if (!dispatched && !(event instanceof DeadEvent)) {
            post(new DeadEvent(this, event));
        }
        dispatchQueuedEvents();
    }

    /* access modifiers changed from: protected */
    public void enqueueEvent(Object event, EventHandler handler) {
        this.eventsToDispatch.get().offer(new EventWithHandler(event, handler));
    }

    /* access modifiers changed from: protected */
    public void dispatchQueuedEvents() {
        if (!this.isDispatching.get().booleanValue()) {
            this.isDispatching.set(true);
            while (true) {
                try {
                    EventWithHandler eventWithHandler = (EventWithHandler) this.eventsToDispatch.get().poll();
                    if (eventWithHandler != null) {
                        dispatch(eventWithHandler.event, eventWithHandler.handler);
                    } else {
                        return;
                    }
                } finally {
                    this.isDispatching.set(false);
                }
            }
        }
    }

    /* access modifiers changed from: protected */
    public void dispatch(Object event, EventHandler wrapper) {
        try {
            wrapper.handleEvent(event);
        } catch (InvocationTargetException e) {
            Logger logger2 = this.logger;
            Level level = Level.SEVERE;
            logger2.log(level, "Could not dispatch event: " + event + " to handler " + wrapper, e);
        }
    }

    /* access modifiers changed from: package-private */
    public Set<EventHandler> getHandlersForEventType(Class<?> type) {
        return this.handlersByType.get(type);
    }

    /* access modifiers changed from: protected */
    public Set<EventHandler> newHandlerSet() {
        return new CopyOnWriteArraySet();
    }

    /* access modifiers changed from: package-private */
    @VisibleForTesting
    public Set<Class<?>> flattenHierarchy(Class<?> concreteClass) {
        try {
            return this.flattenHierarchyCache.get(concreteClass);
        } catch (ExecutionException e) {
            throw Throwables.propagate(e.getCause());
        }
    }

    static class EventWithHandler {
        final Object event;
        final EventHandler handler;

        public EventWithHandler(Object event2, EventHandler handler2) {
            this.event = event2;
            this.handler = handler2;
        }
    }
}
