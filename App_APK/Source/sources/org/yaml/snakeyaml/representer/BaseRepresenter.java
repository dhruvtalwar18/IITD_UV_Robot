package org.yaml.snakeyaml.representer;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.yaml.snakeyaml.DumperOptions;
import org.yaml.snakeyaml.error.Mark;
import org.yaml.snakeyaml.error.YAMLException;
import org.yaml.snakeyaml.introspector.PropertyUtils;
import org.yaml.snakeyaml.nodes.AnchorNode;
import org.yaml.snakeyaml.nodes.MappingNode;
import org.yaml.snakeyaml.nodes.Node;
import org.yaml.snakeyaml.nodes.NodeTuple;
import org.yaml.snakeyaml.nodes.ScalarNode;
import org.yaml.snakeyaml.nodes.SequenceNode;
import org.yaml.snakeyaml.nodes.Tag;

public abstract class BaseRepresenter {
    protected DumperOptions.FlowStyle defaultFlowStyle = DumperOptions.FlowStyle.AUTO;
    protected Character defaultScalarStyle;
    private boolean explicitPropertyUtils = false;
    protected final Map<Class<?>, Represent> multiRepresenters = new LinkedHashMap();
    protected Represent nullRepresenter;
    protected Object objectToRepresent;
    private PropertyUtils propertyUtils;
    protected final Map<Object, Node> representedObjects = new IdentityHashMap<Object, Node>() {
        private static final long serialVersionUID = -5576159264232131854L;

        public Node put(Object key, Node value) {
            return (Node) super.put(key, new AnchorNode(value));
        }
    };
    protected final Map<Class<?>, Represent> representers = new HashMap();

    public Node represent(Object data) {
        Node node = representData(data);
        this.representedObjects.clear();
        this.objectToRepresent = null;
        return node;
    }

    /* access modifiers changed from: protected */
    public final Node representData(Object data) {
        this.objectToRepresent = data;
        if (this.representedObjects.containsKey(this.objectToRepresent)) {
            return this.representedObjects.get(this.objectToRepresent);
        }
        if (data == null) {
            return this.nullRepresenter.representData((Object) null);
        }
        Class<?> clazz = data.getClass();
        if (this.representers.containsKey(clazz)) {
            return this.representers.get(clazz).representData(data);
        }
        for (Class<?> repr : this.multiRepresenters.keySet()) {
            if (repr.isInstance(data)) {
                return this.multiRepresenters.get(repr).representData(data);
            }
        }
        if (clazz.isArray()) {
            throw new YAMLException("Arrays of primitives are not fully supported.");
        } else if (this.multiRepresenters.containsKey((Object) null)) {
            return this.multiRepresenters.get((Object) null).representData(data);
        } else {
            return this.representers.get((Object) null).representData(data);
        }
    }

    /* access modifiers changed from: protected */
    public Node representScalar(Tag tag, String value, Character style) {
        if (style == null) {
            style = this.defaultScalarStyle;
        }
        return new ScalarNode(tag, value, (Mark) null, (Mark) null, style);
    }

    /* access modifiers changed from: protected */
    public Node representScalar(Tag tag, String value) {
        return representScalar(tag, value, (Character) null);
    }

    /* access modifiers changed from: protected */
    public Node representSequence(Tag tag, Iterable<? extends Object> sequence, Boolean flowStyle) {
        int size = 10;
        if (sequence instanceof List) {
            size = ((List) sequence).size();
        }
        List<Node> value = new ArrayList<>(size);
        SequenceNode node = new SequenceNode(tag, value, flowStyle);
        this.representedObjects.put(this.objectToRepresent, node);
        boolean bestStyle = true;
        for (Object item : sequence) {
            Node nodeItem = representData(item);
            if (!(nodeItem instanceof ScalarNode) || ((ScalarNode) nodeItem).getStyle() != null) {
                bestStyle = false;
            }
            value.add(nodeItem);
        }
        if (flowStyle == null) {
            if (this.defaultFlowStyle != DumperOptions.FlowStyle.AUTO) {
                node.setFlowStyle(this.defaultFlowStyle.getStyleBoolean());
            } else {
                node.setFlowStyle(Boolean.valueOf(bestStyle));
            }
        }
        return node;
    }

    /* access modifiers changed from: protected */
    public Node representMapping(Tag tag, Map<? extends Object, Object> mapping, Boolean flowStyle) {
        List<NodeTuple> value = new ArrayList<>(mapping.size());
        MappingNode node = new MappingNode(tag, value, flowStyle);
        this.representedObjects.put(this.objectToRepresent, node);
        boolean bestStyle = true;
        for (Map.Entry<? extends Object, Object> entry : mapping.entrySet()) {
            Node nodeKey = representData(entry.getKey());
            Node nodeValue = representData(entry.getValue());
            if (!(nodeKey instanceof ScalarNode) || ((ScalarNode) nodeKey).getStyle() != null) {
                bestStyle = false;
            }
            if (!(nodeValue instanceof ScalarNode) || ((ScalarNode) nodeValue).getStyle() != null) {
                bestStyle = false;
            }
            value.add(new NodeTuple(nodeKey, nodeValue));
        }
        if (flowStyle == null) {
            if (this.defaultFlowStyle != DumperOptions.FlowStyle.AUTO) {
                node.setFlowStyle(this.defaultFlowStyle.getStyleBoolean());
            } else {
                node.setFlowStyle(Boolean.valueOf(bestStyle));
            }
        }
        return node;
    }

    public void setDefaultScalarStyle(DumperOptions.ScalarStyle defaultStyle) {
        this.defaultScalarStyle = defaultStyle.getChar();
    }

    public void setDefaultFlowStyle(DumperOptions.FlowStyle defaultFlowStyle2) {
        this.defaultFlowStyle = defaultFlowStyle2;
    }

    public DumperOptions.FlowStyle getDefaultFlowStyle() {
        return this.defaultFlowStyle;
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

    public final boolean isExplicitPropertyUtils() {
        return this.explicitPropertyUtils;
    }
}
