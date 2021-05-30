package org.ros.internal.node.parameter;

import com.google.common.base.Preconditions;
import java.net.URI;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import org.ros.exception.ParameterClassCastException;
import org.ros.exception.ParameterNotFoundException;
import org.ros.internal.node.client.ParameterClient;
import org.ros.internal.node.response.Response;
import org.ros.internal.node.response.StatusCode;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.parameter.ParameterListener;
import org.ros.node.parameter.ParameterTree;

public class DefaultParameterTree implements ParameterTree {
    private final ParameterClient parameterClient;
    private final ParameterManager parameterManager;
    private final NameResolver resolver;

    public static DefaultParameterTree newFromNodeIdentifier(NodeIdentifier nodeIdentifier, URI masterUri, NameResolver resolver2, ParameterManager parameterManager2) {
        return new DefaultParameterTree(new ParameterClient(nodeIdentifier, masterUri), parameterManager2, resolver2);
    }

    private DefaultParameterTree(ParameterClient parameterClient2, ParameterManager parameterManager2, NameResolver resolver2) {
        this.parameterClient = parameterClient2;
        this.parameterManager = parameterManager2;
        this.resolver = resolver2;
    }

    public boolean has(GraphName name) {
        return this.parameterClient.hasParam(this.resolver.resolve(name)).getResult().booleanValue();
    }

    public boolean has(String name) {
        return has(GraphName.of(name));
    }

    public void delete(GraphName name) {
        this.parameterClient.deleteParam(this.resolver.resolve(name));
    }

    public void delete(String name) {
        delete(GraphName.of(name));
    }

    public GraphName search(GraphName name) {
        Response<GraphName> response = this.parameterClient.searchParam(this.resolver.resolve(name));
        if (response.getStatusCode() == StatusCode.SUCCESS) {
            return response.getResult();
        }
        return null;
    }

    public GraphName search(String name) {
        return search(GraphName.of(name));
    }

    public List<GraphName> getNames() {
        return this.parameterClient.getParamNames().getResult();
    }

    public void addParameterListener(GraphName name, ParameterListener listener) {
        this.parameterManager.addListener(name, listener);
        this.parameterClient.subscribeParam(name);
    }

    public void addParameterListener(String name, ParameterListener listener) {
        addParameterListener(GraphName.of(name), listener);
    }

    public void set(GraphName name, boolean value) {
        this.parameterClient.setParam(this.resolver.resolve(name), Boolean.valueOf(value));
    }

    public void set(String name, boolean value) {
        set(GraphName.of(name), value);
    }

    public void set(GraphName name, int value) {
        this.parameterClient.setParam(this.resolver.resolve(name), Integer.valueOf(value));
    }

    public void set(String name, int value) {
        set(GraphName.of(name), value);
    }

    public void set(GraphName name, double value) {
        this.parameterClient.setParam(this.resolver.resolve(name), Double.valueOf(value));
    }

    public void set(String name, double value) {
        set(GraphName.of(name), value);
    }

    public void set(GraphName name, String value) {
        this.parameterClient.setParam(this.resolver.resolve(name), value);
    }

    public void set(String name, String value) {
        set(GraphName.of(name), value);
    }

    public void set(GraphName name, List<?> value) {
        this.parameterClient.setParam(this.resolver.resolve(name), value);
    }

    public void set(String name, List<?> value) {
        set(GraphName.of(name), value);
    }

    public void set(GraphName name, Map<?, ?> value) {
        this.parameterClient.setParam(this.resolver.resolve(name), value);
    }

    public void set(String name, Map<?, ?> value) {
        set(GraphName.of(name), value);
    }

    private <T> T get(GraphName name, Class<T> type) {
        Response<Object> response = this.parameterClient.getParam(this.resolver.resolve(name));
        try {
            if (response.getStatusCode() == StatusCode.SUCCESS) {
                return type.cast(response.getResult());
            }
            throw new ParameterNotFoundException("Parameter does not exist: " + name);
        } catch (ClassCastException e) {
            throw new ParameterClassCastException("Cannot cast parameter to: " + type.getName(), e);
        }
    }

    private <T> T get(GraphName name, T defaultValue) {
        Preconditions.checkNotNull(defaultValue);
        Response<Object> response = this.parameterClient.getParam(this.resolver.resolve(name));
        if (response.getStatusCode() != StatusCode.SUCCESS) {
            return defaultValue;
        }
        try {
            return defaultValue.getClass().cast(response.getResult());
        } catch (ClassCastException e) {
            throw new ParameterClassCastException("Cannot cast parameter to: " + defaultValue.getClass().getName(), e);
        }
    }

    public boolean getBoolean(GraphName name) {
        return ((Boolean) get(name, Boolean.class)).booleanValue();
    }

    public boolean getBoolean(String name) {
        return getBoolean(GraphName.of(name));
    }

    public boolean getBoolean(GraphName name, boolean defaultValue) {
        return ((Boolean) get(name, Boolean.valueOf(defaultValue))).booleanValue();
    }

    public boolean getBoolean(String name, boolean defaultValue) {
        return getBoolean(GraphName.of(name), defaultValue);
    }

    public int getInteger(GraphName name) {
        return ((Integer) get(name, Integer.class)).intValue();
    }

    public int getInteger(String name) {
        return getInteger(GraphName.of(name));
    }

    public int getInteger(GraphName name, int defaultValue) {
        return ((Integer) get(name, Integer.valueOf(defaultValue))).intValue();
    }

    public int getInteger(String name, int defaultValue) {
        return getInteger(GraphName.of(name), defaultValue);
    }

    public double getDouble(GraphName name) {
        return ((Double) get(name, Double.class)).doubleValue();
    }

    public double getDouble(String name) {
        return getDouble(GraphName.of(name));
    }

    public double getDouble(GraphName name, double defaultValue) {
        return ((Double) get(name, Double.valueOf(defaultValue))).doubleValue();
    }

    public double getDouble(String name, double defaultValue) {
        return getDouble(GraphName.of(name), defaultValue);
    }

    public String getString(GraphName name) {
        return (String) get(name, String.class);
    }

    public String getString(String name) {
        return (String) get(GraphName.of(name), String.class);
    }

    public String getString(GraphName name, String defaultValue) {
        return (String) get(name, defaultValue);
    }

    public String getString(String name, String defaultValue) {
        return getString(GraphName.of(name), defaultValue);
    }

    public List<?> getList(GraphName name) {
        return Arrays.asList((Object[]) get(name, Object[].class));
    }

    public List<?> getList(String name) {
        return getList(GraphName.of(name));
    }

    public List<?> getList(GraphName name, List<?> defaultValue) {
        return Arrays.asList((Object[]) get(name, defaultValue.toArray()));
    }

    public List<?> getList(String name, List<?> defaultValue) {
        return getList(GraphName.of(name), defaultValue);
    }

    public Map<?, ?> getMap(GraphName name) {
        return (Map) get(name, Map.class);
    }

    public Map<?, ?> getMap(String name) {
        return getMap(GraphName.of(name));
    }

    public Map<?, ?> getMap(GraphName name, Map<?, ?> defaultValue) {
        return (Map) get(name, defaultValue);
    }

    public Map<?, ?> getMap(String name, Map<?, ?> defaultValue) {
        return getMap(GraphName.of(name), defaultValue);
    }
}
