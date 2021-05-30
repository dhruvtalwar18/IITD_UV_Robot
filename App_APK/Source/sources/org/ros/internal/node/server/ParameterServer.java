package org.ros.internal.node.server;

import com.google.common.base.Preconditions;
import com.google.common.collect.HashMultimap;
import com.google.common.collect.Maps;
import com.google.common.collect.Multimap;
import com.google.common.collect.Multimaps;
import com.google.common.collect.Sets;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Stack;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.internal.node.client.SlaveClient;
import org.ros.namespace.GraphName;

public class ParameterServer {
    private static final Log log = LogFactory.getLog(ParameterServer.class);
    private final GraphName masterName = GraphName.of("/master");
    private final Multimap<GraphName, NodeIdentifier> subscribers = Multimaps.synchronizedMultimap(HashMultimap.create());
    private final Map<String, Object> tree = Maps.newConcurrentMap();

    private interface Updater {
        void update(SlaveClient slaveClient);
    }

    public void subscribe(GraphName name, NodeIdentifier nodeIdentifier) {
        this.subscribers.put(name, nodeIdentifier);
    }

    private Stack<String> getGraphNameParts(GraphName name) {
        Stack<String> parts = new Stack<>();
        for (GraphName tip = name; !tip.isRoot(); tip = tip.getParent()) {
            parts.add(tip.getBasename().toString());
        }
        return parts;
    }

    public Object get(GraphName name) {
        Preconditions.checkArgument(name.isGlobal());
        Stack<String> parts = getGraphNameParts(name);
        Object possibleSubtree = this.tree;
        while (!parts.empty() && possibleSubtree != null) {
            if (!(possibleSubtree instanceof Map)) {
                return null;
            }
            possibleSubtree = ((Map) possibleSubtree).get(parts.pop());
        }
        return possibleSubtree;
    }

    private void setValue(GraphName name, Object value) {
        Preconditions.checkArgument(name.isGlobal());
        Stack<String> parts = getGraphNameParts(name);
        Map<String, Object> subtree = this.tree;
        while (!parts.empty()) {
            String part = parts.pop();
            if (parts.empty()) {
                subtree.put(part, value);
            } else if (!subtree.containsKey(part) || !(subtree.get(part) instanceof Map)) {
                Map<String, Object> newSubtree = Maps.newHashMap();
                subtree.put(part, newSubtree);
                subtree = newSubtree;
            } else {
                subtree = (Map) subtree.get(part);
            }
        }
    }

    private <T> void update(GraphName name, T value, Updater updater) {
        setValue(name, value);
        synchronized (this.subscribers) {
            for (NodeIdentifier nodeIdentifier : this.subscribers.get(name)) {
                try {
                    updater.update(new SlaveClient(this.masterName, nodeIdentifier.getUri()));
                } catch (Exception e) {
                    log.error(e);
                }
            }
        }
    }

    public void set(final GraphName name, final boolean value) {
        update(name, Boolean.valueOf(value), new Updater() {
            public void update(SlaveClient client) {
                client.paramUpdate(name, value);
            }
        });
    }

    public void set(final GraphName name, final int value) {
        update(name, Integer.valueOf(value), new Updater() {
            public void update(SlaveClient client) {
                client.paramUpdate(name, value);
            }
        });
    }

    public void set(final GraphName name, final double value) {
        update(name, Double.valueOf(value), new Updater() {
            public void update(SlaveClient client) {
                client.paramUpdate(name, value);
            }
        });
    }

    public void set(final GraphName name, final String value) {
        update(name, value, new Updater() {
            public void update(SlaveClient client) {
                client.paramUpdate(name, value);
            }
        });
    }

    public void set(final GraphName name, final List<?> value) {
        update(name, value, new Updater() {
            public void update(SlaveClient client) {
                client.paramUpdate(name, (List<?>) value);
            }
        });
    }

    public void set(final GraphName name, final Map<?, ?> value) {
        update(name, value, new Updater() {
            public void update(SlaveClient client) {
                client.paramUpdate(name, (Map<?, ?>) value);
            }
        });
    }

    public void delete(GraphName name) {
        Preconditions.checkArgument(name.isGlobal());
        Stack<String> parts = getGraphNameParts(name);
        Map<String, Object> subtree = this.tree;
        while (!parts.empty() && subtree.containsKey(parts.peek())) {
            String part = parts.pop();
            if (parts.empty()) {
                subtree.remove(part);
            } else {
                subtree = subtree.get(part);
            }
        }
    }

    public Object search(GraphName namespace, GraphName name) {
        GraphName search = namespace;
        GraphName result = search.join(name.toRelative());
        if (has(result)) {
            return result;
        }
        while (!search.isRoot()) {
            search = search.getParent();
            GraphName result2 = search.join(name.toRelative());
            if (has(result2)) {
                return result2;
            }
        }
        return null;
    }

    public boolean has(GraphName name) {
        Preconditions.checkArgument(name.isGlobal());
        Stack<String> parts = getGraphNameParts(name);
        Map<String, Object> subtree = this.tree;
        while (!parts.empty() && subtree.containsKey(parts.peek())) {
            String part = parts.pop();
            if (!parts.empty()) {
                subtree = (Map) subtree.get(part);
            }
        }
        return parts.empty();
    }

    private Set<GraphName> getSubtreeNames(GraphName parent, Map<String, Object> subtree, Set<GraphName> names) {
        for (String name : subtree.keySet()) {
            Object possibleSubtree = subtree.get(name);
            if (possibleSubtree instanceof Map) {
                names.addAll(getSubtreeNames(parent.join(GraphName.of(name)), (Map) possibleSubtree, names));
            } else {
                names.add(parent.join(GraphName.of(name)));
            }
        }
        return names;
    }

    public Collection<GraphName> getNames() {
        return getSubtreeNames(GraphName.root(), this.tree, Sets.newHashSet());
    }
}
