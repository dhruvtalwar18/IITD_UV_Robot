package org.yaml.snakeyaml.representer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TimeZone;
import org.yaml.snakeyaml.DumperOptions;
import org.yaml.snakeyaml.introspector.Property;
import org.yaml.snakeyaml.nodes.MappingNode;
import org.yaml.snakeyaml.nodes.Node;
import org.yaml.snakeyaml.nodes.NodeId;
import org.yaml.snakeyaml.nodes.NodeTuple;
import org.yaml.snakeyaml.nodes.ScalarNode;
import org.yaml.snakeyaml.nodes.SequenceNode;
import org.yaml.snakeyaml.nodes.Tag;

public class Representer extends SafeRepresenter {
    public /* bridge */ /* synthetic */ Tag addClassTag(Class x0, String x1) {
        return super.addClassTag((Class<? extends Object>) x0, x1);
    }

    public /* bridge */ /* synthetic */ Tag addClassTag(Class x0, Tag x1) {
        return super.addClassTag((Class<? extends Object>) x0, x1);
    }

    public /* bridge */ /* synthetic */ TimeZone getTimeZone() {
        return super.getTimeZone();
    }

    public /* bridge */ /* synthetic */ void setTimeZone(TimeZone x0) {
        super.setTimeZone(x0);
    }

    public Representer() {
        this.representers.put((Object) null, new RepresentJavaBean());
    }

    protected class RepresentJavaBean implements Represent {
        protected RepresentJavaBean() {
        }

        public Node representData(Object data) {
            return Representer.this.representJavaBean(Representer.this.getProperties(data.getClass()), data);
        }
    }

    /* access modifiers changed from: protected */
    public MappingNode representJavaBean(Set<Property> properties, Object javaBean) {
        List<NodeTuple> value = new ArrayList<>(properties.size());
        Tag customTag = (Tag) this.classTags.get(javaBean.getClass());
        MappingNode node = new MappingNode(customTag != null ? customTag : new Tag((Class<? extends Object>) javaBean.getClass()), value, (Boolean) null);
        this.representedObjects.put(javaBean, node);
        boolean bestStyle = true;
        for (Property property : properties) {
            Object memberValue = property.get(javaBean);
            NodeTuple tuple = representJavaBeanProperty(javaBean, property, memberValue, memberValue == null ? null : (Tag) this.classTags.get(memberValue.getClass()));
            if (tuple != null) {
                if (((ScalarNode) tuple.getKeyNode()).getStyle() != null) {
                    bestStyle = false;
                }
                Node nodeValue = tuple.getValueNode();
                if (!(nodeValue instanceof ScalarNode) || ((ScalarNode) nodeValue).getStyle() != null) {
                    bestStyle = false;
                }
                value.add(tuple);
            }
        }
        if (this.defaultFlowStyle != DumperOptions.FlowStyle.AUTO) {
            node.setFlowStyle(this.defaultFlowStyle.getStyleBoolean());
        } else {
            node.setFlowStyle(Boolean.valueOf(bestStyle));
        }
        return node;
    }

    /* access modifiers changed from: protected */
    public NodeTuple representJavaBeanProperty(Object javaBean, Property property, Object propertyValue, Tag customTag) {
        ScalarNode nodeKey = (ScalarNode) representData(property.getName());
        boolean hasAlias = this.representedObjects.containsKey(propertyValue);
        Node nodeValue = representData(propertyValue);
        if (propertyValue != null && !hasAlias) {
            NodeId nodeId = nodeValue.getNodeId();
            if (customTag == null) {
                if (nodeId != NodeId.scalar) {
                    if (nodeId == NodeId.mapping && property.getType() == propertyValue.getClass() && !(propertyValue instanceof Map) && !nodeValue.getTag().equals(Tag.SET)) {
                        nodeValue.setTag(Tag.MAP);
                    }
                    checkGlobalTag(property, nodeValue, propertyValue);
                } else if (propertyValue instanceof Enum) {
                    nodeValue.setTag(Tag.STR);
                }
            }
        }
        return new NodeTuple(nodeKey, nodeValue);
    }

    /* access modifiers changed from: protected */
    public void checkGlobalTag(Property property, Node node, Object object) {
        Iterable<Object> memberList;
        Class<? extends Object>[] arguments = property.getActualTypeArguments();
        if (arguments == null) {
            return;
        }
        if (node.getNodeId() == NodeId.sequence) {
            Class<? extends Object> t = arguments[0];
            SequenceNode snode = (SequenceNode) node;
            if (object.getClass().isArray()) {
                memberList = Arrays.asList((Object[]) object);
            } else {
                memberList = (Iterable) object;
            }
            Iterator<Object> iter = memberList.iterator();
            for (Node childNode : snode.getValue()) {
                Object member = iter.next();
                if (member != null && t.equals(member.getClass()) && childNode.getNodeId() == NodeId.mapping) {
                    childNode.setTag(Tag.MAP);
                }
            }
        } else if (object instanceof Set) {
            Class<? extends Object> t2 = arguments[0];
            Iterator<NodeTuple> iter2 = ((MappingNode) node).getValue().iterator();
            for (Object member2 : (Set) object) {
                Node keyNode = iter2.next().getKeyNode();
                if (t2.equals(member2.getClass()) && keyNode.getNodeId() == NodeId.mapping) {
                    keyNode.setTag(Tag.MAP);
                }
            }
        } else if (object instanceof Map) {
            Class<? extends Object> keyType = arguments[0];
            Class<? extends Object> valueType = arguments[1];
            for (NodeTuple tuple : ((MappingNode) node).getValue()) {
                resetTag(keyType, tuple.getKeyNode());
                resetTag(valueType, tuple.getValueNode());
            }
        }
    }

    private void resetTag(Class<? extends Object> type, Node node) {
        if (!node.getTag().matches(type)) {
            return;
        }
        if (Enum.class.isAssignableFrom(type)) {
            node.setTag(Tag.STR);
        } else {
            node.setTag(Tag.MAP);
        }
    }

    /* access modifiers changed from: protected */
    public Set<Property> getProperties(Class<? extends Object> type) {
        return getPropertyUtils().getProperties(type);
    }
}
