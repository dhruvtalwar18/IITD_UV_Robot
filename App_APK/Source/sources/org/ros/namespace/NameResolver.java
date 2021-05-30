package org.ros.namespace;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import org.ros.exception.RosRuntimeException;

public class NameResolver {
    private final GraphName namespace;
    private final Map<GraphName, GraphName> remappings;

    public static NameResolver newFromNamespace(GraphName namespace2) {
        return new NameResolver(namespace2, new HashMap());
    }

    public static NameResolver newFromNamespace(String namespace2) {
        return newFromNamespace(GraphName.of(namespace2));
    }

    public static NameResolver newRoot() {
        return newFromNamespace(GraphName.root());
    }

    public static NameResolver newRootFromRemappings(Map<GraphName, GraphName> remappings2) {
        return new NameResolver(GraphName.root(), remappings2);
    }

    public static NameResolver newFromNamespaceAndRemappings(String namespace2, Map<GraphName, GraphName> remappings2) {
        return new NameResolver(GraphName.of(namespace2), remappings2);
    }

    public NameResolver(GraphName namespace2, Map<GraphName, GraphName> remappings2) {
        this.remappings = Collections.unmodifiableMap(remappings2);
        this.namespace = namespace2;
    }

    public GraphName getNamespace() {
        return this.namespace;
    }

    public GraphName resolve(GraphName namespace2, GraphName name) {
        GraphName remappedNamespace = lookUpRemapping(namespace2);
        if (remappedNamespace.isGlobal()) {
            GraphName remappedName = lookUpRemapping(name);
            if (remappedName.isGlobal()) {
                return remappedName;
            }
            if (remappedName.isRelative()) {
                return remappedNamespace.join(remappedName);
            }
            if (remappedName.isPrivate()) {
                throw new RosRuntimeException("Cannot resolve ~private names in arbitrary namespaces.");
            }
            throw new RosRuntimeException("Unable to resolve graph name: " + name);
        }
        throw new IllegalArgumentException(String.format("Namespace %s (remapped from %s) must be global.", new Object[]{remappedNamespace, namespace2}));
    }

    public GraphName resolve(String namespace2, String name) {
        return resolve(GraphName.of(namespace2), GraphName.of(name));
    }

    public GraphName resolve(GraphName namespace2, String name) {
        return resolve(namespace2, GraphName.of(name));
    }

    public GraphName resolve(String namespace2, GraphName name) {
        return resolve(GraphName.of(namespace2), name);
    }

    public GraphName resolve(GraphName name) {
        return resolve(this.namespace, name);
    }

    public GraphName resolve(String name) {
        return resolve(GraphName.of(name));
    }

    public Map<GraphName, GraphName> getRemappings() {
        return this.remappings;
    }

    public NameResolver newChild(GraphName namespace2) {
        return new NameResolver(resolve(namespace2), this.remappings);
    }

    public NameResolver newChild(String namespace2) {
        return newChild(GraphName.of(namespace2));
    }

    /* access modifiers changed from: protected */
    public GraphName lookUpRemapping(GraphName name) {
        GraphName remappedName = name;
        if (this.remappings.containsKey(name)) {
            return this.remappings.get(name);
        }
        return remappedName;
    }
}
