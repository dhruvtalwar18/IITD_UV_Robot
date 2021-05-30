package org.ros.node.parameter;

import java.util.Collection;
import java.util.List;
import java.util.Map;
import org.ros.namespace.GraphName;

public interface ParameterTree {
    void addParameterListener(String str, ParameterListener parameterListener);

    void addParameterListener(GraphName graphName, ParameterListener parameterListener);

    void delete(String str);

    void delete(GraphName graphName);

    boolean getBoolean(String str);

    boolean getBoolean(String str, boolean z);

    boolean getBoolean(GraphName graphName);

    boolean getBoolean(GraphName graphName, boolean z);

    double getDouble(String str);

    double getDouble(String str, double d);

    double getDouble(GraphName graphName);

    double getDouble(GraphName graphName, double d);

    int getInteger(String str);

    int getInteger(String str, int i);

    int getInteger(GraphName graphName);

    int getInteger(GraphName graphName, int i);

    List<?> getList(String str);

    List<?> getList(String str, List<?> list);

    List<?> getList(GraphName graphName);

    List<?> getList(GraphName graphName, List<?> list);

    Map<?, ?> getMap(String str);

    Map<?, ?> getMap(String str, Map<?, ?> map);

    Map<?, ?> getMap(GraphName graphName);

    Map<?, ?> getMap(GraphName graphName, Map<?, ?> map);

    Collection<GraphName> getNames();

    String getString(String str);

    String getString(String str, String str2);

    String getString(GraphName graphName);

    String getString(GraphName graphName, String str);

    boolean has(String str);

    boolean has(GraphName graphName);

    GraphName search(String str);

    GraphName search(GraphName graphName);

    void set(String str, double d);

    void set(String str, int i);

    void set(String str, String str2);

    void set(String str, List<?> list);

    void set(String str, Map<?, ?> map);

    void set(String str, boolean z);

    void set(GraphName graphName, double d);

    void set(GraphName graphName, int i);

    void set(GraphName graphName, String str);

    void set(GraphName graphName, List<?> list);

    void set(GraphName graphName, Map<?, ?> map);

    void set(GraphName graphName, boolean z);
}
