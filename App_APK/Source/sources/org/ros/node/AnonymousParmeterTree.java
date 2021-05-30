package org.ros.node;

import java.net.URI;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import org.ros.internal.node.parameter.DefaultParameterTree;
import org.ros.internal.node.parameter.ParameterManager;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.parameter.ParameterListener;
import org.ros.node.parameter.ParameterTree;
import rocon_std_msgs.Connection;

public class AnonymousParmeterTree implements ParameterTree {
    private ParameterTree parameterTree;

    public AnonymousParmeterTree(URI masterUri) {
        this.parameterTree = DefaultParameterTree.newFromNodeIdentifier(new NodeIdentifier(GraphName.of(Connection.INVALID), (URI) null), masterUri, NameResolver.newRoot(), (ParameterManager) null);
    }

    public boolean getBoolean(GraphName name) {
        return this.parameterTree.getBoolean(name);
    }

    public boolean getBoolean(String name) {
        return this.parameterTree.getBoolean(name);
    }

    public boolean getBoolean(GraphName name, boolean defaultValue) {
        return this.parameterTree.getBoolean(name, defaultValue);
    }

    public boolean getBoolean(String name, boolean defaultValue) {
        return this.parameterTree.getBoolean(name, defaultValue);
    }

    public int getInteger(GraphName name) {
        return this.parameterTree.getInteger(name);
    }

    public int getInteger(String name) {
        return this.parameterTree.getInteger(name);
    }

    public int getInteger(GraphName name, int defaultValue) {
        return this.parameterTree.getInteger(name, defaultValue);
    }

    public int getInteger(String name, int defaultValue) {
        return this.parameterTree.getInteger(name, defaultValue);
    }

    public double getDouble(GraphName name) {
        return this.parameterTree.getDouble(name);
    }

    public double getDouble(String name) {
        return this.parameterTree.getDouble(name);
    }

    public double getDouble(GraphName name, double defaultValue) {
        return this.parameterTree.getDouble(name, defaultValue);
    }

    public double getDouble(String name, double defaultValue) {
        return this.parameterTree.getDouble(name, defaultValue);
    }

    public String getString(GraphName name) {
        return this.parameterTree.getString(name);
    }

    public String getString(String name) {
        return this.parameterTree.getString(name);
    }

    public String getString(GraphName name, String defaultValue) {
        return this.parameterTree.getString(name, defaultValue);
    }

    public String getString(String name, String defaultValue) {
        return this.parameterTree.getString(name, defaultValue);
    }

    public List<?> getList(GraphName name) {
        return this.parameterTree.getList(name);
    }

    public List<?> getList(String name) {
        return this.parameterTree.getList(name);
    }

    public List<?> getList(GraphName name, List<?> defaultValue) {
        return this.parameterTree.getList(name, defaultValue);
    }

    public List<?> getList(String name, List<?> defaultValue) {
        return this.parameterTree.getList(name, defaultValue);
    }

    public Map<?, ?> getMap(GraphName name) {
        return this.parameterTree.getMap(name);
    }

    public Map<?, ?> getMap(String name) {
        return this.parameterTree.getMap(name);
    }

    public Map<?, ?> getMap(GraphName name, Map<?, ?> defaultValue) {
        return this.parameterTree.getMap(name, defaultValue);
    }

    public Map<?, ?> getMap(String name, Map<?, ?> defaultValue) {
        return this.parameterTree.getMap(name, defaultValue);
    }

    public void set(GraphName name, boolean value) {
        this.parameterTree.set(name, value);
    }

    public void set(String name, boolean value) {
        this.parameterTree.set(name, value);
    }

    public void set(GraphName name, int value) {
        this.parameterTree.set(name, value);
    }

    public void set(String name, int value) {
        this.parameterTree.set(name, value);
    }

    public void set(GraphName name, double value) {
        this.parameterTree.set(name, value);
    }

    public void set(String name, double value) {
        this.parameterTree.set(name, value);
    }

    public void set(GraphName name, String value) {
        this.parameterTree.set(name, value);
    }

    public void set(String name, String value) {
        this.parameterTree.set(name, value);
    }

    public void set(GraphName name, List<?> value) {
        this.parameterTree.set(name, value);
    }

    public void set(String name, List<?> value) {
        this.parameterTree.set(name, value);
    }

    public void set(GraphName name, Map<?, ?> value) {
        this.parameterTree.set(name, value);
    }

    public void set(String name, Map<?, ?> value) {
        this.parameterTree.set(name, value);
    }

    public boolean has(GraphName name) {
        return this.parameterTree.has(name);
    }

    public boolean has(String name) {
        return this.parameterTree.has(name);
    }

    public void delete(GraphName name) {
        this.parameterTree.delete(name);
    }

    public void delete(String name) {
        this.parameterTree.delete(name);
    }

    public GraphName search(GraphName name) {
        return this.parameterTree.search(name);
    }

    public GraphName search(String name) {
        return this.parameterTree.search(name);
    }

    public Collection<GraphName> getNames() {
        return this.parameterTree.getNames();
    }

    public void addParameterListener(GraphName name, ParameterListener listener) {
        throw new UnsupportedOperationException();
    }

    public void addParameterListener(String name, ParameterListener listener) {
        throw new UnsupportedOperationException();
    }
}
