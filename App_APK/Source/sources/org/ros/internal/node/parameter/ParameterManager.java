package org.ros.internal.node.parameter;

import com.google.common.collect.Maps;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import org.ros.concurrent.ListenerGroup;
import org.ros.concurrent.SignalRunnable;
import org.ros.namespace.GraphName;
import org.ros.node.parameter.ParameterListener;

public class ParameterManager {
    private final ExecutorService executorService;
    private final Map<GraphName, ListenerGroup<ParameterListener>> listeners = Maps.newHashMap();

    public ParameterManager(ExecutorService executorService2) {
        this.executorService = executorService2;
    }

    public void addListener(GraphName parameterName, ParameterListener listener) {
        synchronized (this.listeners) {
            if (!this.listeners.containsKey(parameterName)) {
                this.listeners.put(parameterName, new ListenerGroup(this.executorService));
            }
            this.listeners.get(parameterName).add(listener);
        }
    }

    public int updateParameter(GraphName parameterName, final Object value) {
        int numberOfListeners = 0;
        synchronized (this.listeners) {
            if (this.listeners.containsKey(parameterName)) {
                ListenerGroup<ParameterListener> listenerCollection = this.listeners.get(parameterName);
                numberOfListeners = listenerCollection.size();
                listenerCollection.signal(new SignalRunnable<ParameterListener>() {
                    public void run(ParameterListener listener) {
                        listener.onNewValue(value);
                    }
                });
            }
        }
        return numberOfListeners;
    }
}
