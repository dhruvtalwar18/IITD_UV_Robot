package com.google.common.eventbus;

import com.google.common.collect.Multimap;

interface HandlerFindingStrategy {
    Multimap<Class<?>, EventHandler> findAllHandlers(Object obj);
}
