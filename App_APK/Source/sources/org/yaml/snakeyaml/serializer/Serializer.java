package org.yaml.snakeyaml.serializer;

import java.io.IOException;
import java.text.NumberFormat;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import org.yaml.snakeyaml.DumperOptions;
import org.yaml.snakeyaml.emitter.Emitable;
import org.yaml.snakeyaml.error.Mark;
import org.yaml.snakeyaml.events.AliasEvent;
import org.yaml.snakeyaml.events.DocumentEndEvent;
import org.yaml.snakeyaml.events.DocumentStartEvent;
import org.yaml.snakeyaml.events.ImplicitTuple;
import org.yaml.snakeyaml.events.MappingEndEvent;
import org.yaml.snakeyaml.events.MappingStartEvent;
import org.yaml.snakeyaml.events.ScalarEvent;
import org.yaml.snakeyaml.events.SequenceEndEvent;
import org.yaml.snakeyaml.events.SequenceStartEvent;
import org.yaml.snakeyaml.events.StreamEndEvent;
import org.yaml.snakeyaml.events.StreamStartEvent;
import org.yaml.snakeyaml.nodes.AnchorNode;
import org.yaml.snakeyaml.nodes.CollectionNode;
import org.yaml.snakeyaml.nodes.MappingNode;
import org.yaml.snakeyaml.nodes.Node;
import org.yaml.snakeyaml.nodes.NodeId;
import org.yaml.snakeyaml.nodes.NodeTuple;
import org.yaml.snakeyaml.nodes.ScalarNode;
import org.yaml.snakeyaml.nodes.SequenceNode;
import org.yaml.snakeyaml.nodes.Tag;
import org.yaml.snakeyaml.resolver.Resolver;

public final class Serializer {
    private Map<Node, String> anchors;
    private Boolean closed;
    private final Emitable emitter;
    private boolean explicitEnd;
    private Tag explicitRoot;
    private boolean explicitStart;
    private int lastAnchorId;
    private final Resolver resolver;
    private Set<Node> serializedNodes;
    private Map<String, String> useTags;
    private Integer[] useVersion;

    public Serializer(Emitable emitter2, Resolver resolver2, DumperOptions opts, Tag rootTag) {
        this.emitter = emitter2;
        this.resolver = resolver2;
        this.explicitStart = opts.isExplicitStart();
        this.explicitEnd = opts.isExplicitEnd();
        if (opts.getVersion() != null) {
            this.useVersion = opts.getVersion().getArray();
        }
        this.useTags = opts.getTags();
        this.serializedNodes = new HashSet();
        this.anchors = new HashMap();
        this.lastAnchorId = 0;
        this.closed = null;
        this.explicitRoot = rootTag;
    }

    public void open() throws IOException {
        if (this.closed == null) {
            this.emitter.emit(new StreamStartEvent((Mark) null, (Mark) null));
            this.closed = Boolean.FALSE;
        } else if (Boolean.TRUE.equals(this.closed)) {
            throw new SerializerException("serializer is closed");
        } else {
            throw new SerializerException("serializer is already opened");
        }
    }

    public void close() throws IOException {
        if (this.closed == null) {
            throw new SerializerException("serializer is not opened");
        } else if (!Boolean.TRUE.equals(this.closed)) {
            this.emitter.emit(new StreamEndEvent((Mark) null, (Mark) null));
            this.closed = Boolean.TRUE;
        }
    }

    public void serialize(Node node) throws IOException {
        if (this.closed == null) {
            throw new SerializerException("serializer is not opened");
        } else if (!this.closed.booleanValue()) {
            this.emitter.emit(new DocumentStartEvent((Mark) null, (Mark) null, this.explicitStart, this.useVersion, this.useTags));
            anchorNode(node);
            if (this.explicitRoot != null) {
                node.setTag(this.explicitRoot);
            }
            serializeNode(node, (Node) null);
            this.emitter.emit(new DocumentEndEvent((Mark) null, (Mark) null, this.explicitEnd));
            this.serializedNodes.clear();
            this.anchors.clear();
            this.lastAnchorId = 0;
        } else {
            throw new SerializerException("serializer is closed");
        }
    }

    private void anchorNode(Node node) {
        if (node.getNodeId() == NodeId.anchor) {
            node = ((AnchorNode) node).getRealNode();
        }
        if (!this.anchors.containsKey(node)) {
            this.anchors.put(node, (Object) null);
            switch (node.getNodeId()) {
                case sequence:
                    for (Node item : ((SequenceNode) node).getValue()) {
                        anchorNode(item);
                    }
                    return;
                case mapping:
                    for (NodeTuple object : ((MappingNode) node).getValue()) {
                        Node key = object.getKeyNode();
                        Node value = object.getValueNode();
                        anchorNode(key);
                        anchorNode(value);
                    }
                    return;
                default:
                    return;
            }
        } else if (this.anchors.get(node) == null) {
            this.anchors.put(node, generateAnchor());
        }
    }

    private String generateAnchor() {
        this.lastAnchorId++;
        NumberFormat format = NumberFormat.getNumberInstance();
        format.setMinimumIntegerDigits(3);
        format.setGroupingUsed(false);
        return "id" + format.format((long) this.lastAnchorId);
    }

    private void serializeNode(Node node, Node parent) throws IOException {
        if (node.getNodeId() == NodeId.anchor) {
            node = ((AnchorNode) node).getRealNode();
        }
        String tAlias = this.anchors.get(node);
        if (this.serializedNodes.contains(node)) {
            this.emitter.emit(new AliasEvent(tAlias, (Mark) null, (Mark) null));
            return;
        }
        this.serializedNodes.add(node);
        int i = AnonymousClass1.$SwitchMap$org$yaml$snakeyaml$nodes$NodeId[node.getNodeId().ordinal()];
        if (i == 1) {
            SequenceNode seqNode = (SequenceNode) node;
            this.emitter.emit(new SequenceStartEvent(tAlias, node.getTag().getValue(), node.getTag().equals(this.resolver.resolve(NodeId.sequence, (String) null, true)), (Mark) null, (Mark) null, seqNode.getFlowStyle()));
            int indexCounter = 0;
            for (Node item : seqNode.getValue()) {
                serializeNode(item, node);
                indexCounter++;
            }
            this.emitter.emit(new SequenceEndEvent((Mark) null, (Mark) null));
        } else if (i != 3) {
            this.emitter.emit(new MappingStartEvent(tAlias, node.getTag().getValue(), node.getTag().equals(this.resolver.resolve(NodeId.mapping, (String) null, true)), (Mark) null, (Mark) null, ((CollectionNode) node).getFlowStyle()));
            MappingNode mnode = (MappingNode) node;
            for (NodeTuple row : mnode.getValue()) {
                Node key = row.getKeyNode();
                Node value = row.getValueNode();
                serializeNode(key, mnode);
                serializeNode(value, mnode);
            }
            this.emitter.emit(new MappingEndEvent((Mark) null, (Mark) null));
        } else {
            ScalarNode scalarNode = (ScalarNode) node;
            String str = tAlias;
            this.emitter.emit(new ScalarEvent(str, node.getTag().getValue(), new ImplicitTuple(node.getTag().equals(this.resolver.resolve(NodeId.scalar, scalarNode.getValue(), true)), node.getTag().equals(this.resolver.resolve(NodeId.scalar, scalarNode.getValue(), false))), scalarNode.getValue(), (Mark) null, (Mark) null, scalarNode.getStyle()));
        }
    }
}
