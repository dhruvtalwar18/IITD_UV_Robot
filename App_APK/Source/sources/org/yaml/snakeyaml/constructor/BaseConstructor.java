package org.yaml.snakeyaml.constructor;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collection;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.yaml.snakeyaml.composer.Composer;
import org.yaml.snakeyaml.error.Mark;
import org.yaml.snakeyaml.error.YAMLException;
import org.yaml.snakeyaml.introspector.PropertyUtils;
import org.yaml.snakeyaml.nodes.MappingNode;
import org.yaml.snakeyaml.nodes.Node;
import org.yaml.snakeyaml.nodes.NodeId;
import org.yaml.snakeyaml.nodes.NodeTuple;
import org.yaml.snakeyaml.nodes.ScalarNode;
import org.yaml.snakeyaml.nodes.SequenceNode;
import org.yaml.snakeyaml.nodes.Tag;

public abstract class BaseConstructor {
    private Composer composer;
    private final Map<Node, Object> constructedObjects = new HashMap();
    private boolean explicitPropertyUtils = false;
    private final ArrayList<RecursiveTuple<Map<Object, Object>, RecursiveTuple<Object, Object>>> maps2fill = new ArrayList<>();
    private PropertyUtils propertyUtils;
    private final Set<Node> recursiveObjects = new HashSet();
    protected Tag rootTag = null;
    private final ArrayList<RecursiveTuple<Set<Object>, Object>> sets2fill = new ArrayList<>();
    protected final Map<NodeId, Construct> yamlClassConstructors = new EnumMap(NodeId.class);
    protected final Map<Tag, Construct> yamlConstructors = new HashMap();
    protected final Map<String, Construct> yamlMultiConstructors = new HashMap();

    public void setComposer(Composer composer2) {
        this.composer = composer2;
    }

    public boolean checkData() {
        return this.composer.checkNode();
    }

    public Object getData() {
        this.composer.checkNode();
        Node node = this.composer.getNode();
        if (this.rootTag != null) {
            node.setTag(this.rootTag);
        }
        return constructDocument(node);
    }

    public Object getSingleData(Class<?> type) {
        Node node = this.composer.getSingleNode();
        if (node == null) {
            return null;
        }
        if (Object.class != type) {
            node.setTag(new Tag((Class<? extends Object>) type));
        } else if (this.rootTag != null) {
            node.setTag(this.rootTag);
        }
        return constructDocument(node);
    }

    private Object constructDocument(Node node) {
        Object data = constructObject(node);
        fillRecursive();
        this.constructedObjects.clear();
        this.recursiveObjects.clear();
        return data;
    }

    private void fillRecursive() {
        if (!this.maps2fill.isEmpty()) {
            Iterator i$ = this.maps2fill.iterator();
            while (i$.hasNext()) {
                RecursiveTuple<Map<Object, Object>, RecursiveTuple<Object, Object>> entry = i$.next();
                RecursiveTuple<Object, Object> key_value = entry._2();
                entry._1().put(key_value._1(), key_value._2());
            }
            this.maps2fill.clear();
        }
        if (!this.sets2fill.isEmpty()) {
            Iterator i$2 = this.sets2fill.iterator();
            while (i$2.hasNext()) {
                RecursiveTuple<Set<Object>, Object> value = i$2.next();
                value._1().add(value._2());
            }
            this.sets2fill.clear();
        }
    }

    /* access modifiers changed from: protected */
    public Object constructObject(Node node) {
        if (this.constructedObjects.containsKey(node)) {
            return this.constructedObjects.get(node);
        }
        if (!this.recursiveObjects.contains(node)) {
            this.recursiveObjects.add(node);
            Construct constructor = getConstructor(node);
            Object data = constructor.construct(node);
            this.constructedObjects.put(node, data);
            this.recursiveObjects.remove(node);
            if (node.isTwoStepsConstruction()) {
                constructor.construct2ndStep(node, data);
            }
            return data;
        }
        throw new ConstructorException((String) null, (Mark) null, "found unconstructable recursive node", node.getStartMark());
    }

    /* access modifiers changed from: protected */
    public Construct getConstructor(Node node) {
        if (node.useClassConstructor()) {
            return this.yamlClassConstructors.get(node.getNodeId());
        }
        Construct constructor = this.yamlConstructors.get(node.getTag());
        if (constructor != null) {
            return constructor;
        }
        for (String prefix : this.yamlMultiConstructors.keySet()) {
            if (node.getTag().startsWith(prefix)) {
                return this.yamlMultiConstructors.get(prefix);
            }
        }
        return this.yamlConstructors.get((Object) null);
    }

    /* access modifiers changed from: protected */
    public Object constructScalar(ScalarNode node) {
        return node.getValue();
    }

    /* access modifiers changed from: protected */
    public List<Object> createDefaultList(int initSize) {
        return new ArrayList(initSize);
    }

    /* access modifiers changed from: protected */
    public Set<Object> createDefaultSet(int initSize) {
        return new LinkedHashSet(initSize);
    }

    /* access modifiers changed from: protected */
    public <T> T[] createArray(Class<T> type, int size) {
        return (Object[]) Array.newInstance(type.getComponentType(), size);
    }

    /* access modifiers changed from: protected */
    public List<? extends Object> constructSequence(SequenceNode node) {
        List<Object> result;
        if (!List.class.isAssignableFrom(node.getType()) || node.getType().isInterface()) {
            result = createDefaultList(node.getValue().size());
        } else {
            try {
                result = (List) node.getType().newInstance();
            } catch (Exception e) {
                throw new YAMLException((Throwable) e);
            }
        }
        constructSequenceStep2(node, result);
        return result;
    }

    /* access modifiers changed from: protected */
    public Set<? extends Object> constructSet(SequenceNode node) {
        Set<Object> result;
        if (!node.getType().isInterface()) {
            try {
                result = (Set) node.getType().newInstance();
            } catch (Exception e) {
                throw new YAMLException((Throwable) e);
            }
        } else {
            result = createDefaultSet(node.getValue().size());
        }
        constructSequenceStep2(node, result);
        return result;
    }

    /* access modifiers changed from: protected */
    public Object constructArray(SequenceNode node) {
        return constructArrayStep2(node, createArray(node.getType(), node.getValue().size()));
    }

    /* access modifiers changed from: protected */
    public void constructSequenceStep2(SequenceNode node, Collection<Object> collection) {
        for (Node child : node.getValue()) {
            collection.add(constructObject(child));
        }
    }

    /* access modifiers changed from: protected */
    public Object constructArrayStep2(SequenceNode node, Object array) {
        int index = 0;
        for (Node child : node.getValue()) {
            Array.set(array, index, constructObject(child));
            index++;
        }
        return array;
    }

    /* access modifiers changed from: protected */
    public Map<Object, Object> createDefaultMap() {
        return new LinkedHashMap();
    }

    /* access modifiers changed from: protected */
    public Set<Object> createDefaultSet() {
        return new LinkedHashSet();
    }

    /* access modifiers changed from: protected */
    public Set<Object> constructSet(MappingNode node) {
        Set<Object> set = createDefaultSet();
        constructSet2ndStep(node, set);
        return set;
    }

    /* access modifiers changed from: protected */
    public Map<Object, Object> constructMapping(MappingNode node) {
        Map<Object, Object> mapping = createDefaultMap();
        constructMapping2ndStep(node, mapping);
        return mapping;
    }

    /* access modifiers changed from: protected */
    public void constructMapping2ndStep(MappingNode node, Map<Object, Object> mapping) {
        for (NodeTuple tuple : node.getValue()) {
            Node keyNode = tuple.getKeyNode();
            Node valueNode = tuple.getValueNode();
            Object key = constructObject(keyNode);
            if (key != null) {
                try {
                    key.hashCode();
                } catch (Exception e) {
                    Mark startMark = node.getStartMark();
                    throw new ConstructorException("while constructing a mapping", startMark, "found unacceptable key " + key, tuple.getKeyNode().getStartMark(), e);
                }
            }
            Object value = constructObject(valueNode);
            if (keyNode.isTwoStepsConstruction()) {
                this.maps2fill.add(0, new RecursiveTuple(mapping, new RecursiveTuple(key, value)));
            } else {
                mapping.put(key, value);
            }
        }
    }

    /* access modifiers changed from: protected */
    public void constructSet2ndStep(MappingNode node, Set<Object> set) {
        for (NodeTuple tuple : node.getValue()) {
            Node keyNode = tuple.getKeyNode();
            Object key = constructObject(keyNode);
            if (key != null) {
                try {
                    key.hashCode();
                } catch (Exception e) {
                    Mark startMark = node.getStartMark();
                    throw new ConstructorException("while constructing a Set", startMark, "found unacceptable key " + key, tuple.getKeyNode().getStartMark(), e);
                }
            }
            if (keyNode.isTwoStepsConstruction()) {
                this.sets2fill.add(0, new RecursiveTuple(set, key));
            } else {
                set.add(key);
            }
        }
    }

    public void setPropertyUtils(PropertyUtils propertyUtils2) {
        this.propertyUtils = propertyUtils2;
        this.explicitPropertyUtils = true;
    }

    public final PropertyUtils getPropertyUtils() {
        if (this.propertyUtils == null) {
            this.propertyUtils = new PropertyUtils();
        }
        return this.propertyUtils;
    }

    private static class RecursiveTuple<T, K> {
        private final T _1;
        private final K _2;

        public RecursiveTuple(T _12, K _22) {
            this._1 = _12;
            this._2 = _22;
        }

        public K _2() {
            return this._2;
        }

        public T _1() {
            return this._1;
        }
    }

    public final boolean isExplicitPropertyUtils() {
        return this.explicitPropertyUtils;
    }
}
