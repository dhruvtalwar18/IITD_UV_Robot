package org.yaml.snakeyaml.composer;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.yaml.snakeyaml.error.Mark;
import org.yaml.snakeyaml.events.AliasEvent;
import org.yaml.snakeyaml.events.Event;
import org.yaml.snakeyaml.events.MappingStartEvent;
import org.yaml.snakeyaml.events.NodeEvent;
import org.yaml.snakeyaml.events.ScalarEvent;
import org.yaml.snakeyaml.events.SequenceStartEvent;
import org.yaml.snakeyaml.nodes.MappingNode;
import org.yaml.snakeyaml.nodes.Node;
import org.yaml.snakeyaml.nodes.NodeId;
import org.yaml.snakeyaml.nodes.NodeTuple;
import org.yaml.snakeyaml.nodes.ScalarNode;
import org.yaml.snakeyaml.nodes.SequenceNode;
import org.yaml.snakeyaml.nodes.Tag;
import org.yaml.snakeyaml.parser.Parser;
import org.yaml.snakeyaml.resolver.Resolver;

public class Composer {
    private final Map<String, Node> anchors = new HashMap();
    private final Parser parser;
    private final Set<Node> recursiveNodes = new HashSet();
    private final Resolver resolver;

    public Composer(Parser parser2, Resolver resolver2) {
        this.parser = parser2;
        this.resolver = resolver2;
    }

    public boolean checkNode() {
        if (this.parser.checkEvent(Event.ID.StreamStart)) {
            this.parser.getEvent();
        }
        return !this.parser.checkEvent(Event.ID.StreamEnd);
    }

    public Node getNode() {
        if (!this.parser.checkEvent(Event.ID.StreamEnd)) {
            return composeDocument();
        }
        return null;
    }

    public Node getSingleNode() {
        this.parser.getEvent();
        Node document = null;
        if (!this.parser.checkEvent(Event.ID.StreamEnd)) {
            document = composeDocument();
        }
        if (this.parser.checkEvent(Event.ID.StreamEnd)) {
            this.parser.getEvent();
            return document;
        }
        throw new ComposerException("expected a single document in the stream", document.getStartMark(), "but found another document", this.parser.getEvent().getStartMark());
    }

    private Node composeDocument() {
        this.parser.getEvent();
        Node node = composeNode((Node) null);
        this.parser.getEvent();
        this.anchors.clear();
        this.recursiveNodes.clear();
        return node;
    }

    private Node composeNode(Node parent) {
        Node node;
        this.recursiveNodes.add(parent);
        if (this.parser.checkEvent(Event.ID.Alias)) {
            AliasEvent event = (AliasEvent) this.parser.getEvent();
            String anchor = event.getAnchor();
            if (this.anchors.containsKey(anchor)) {
                Node result = this.anchors.get(anchor);
                if (this.recursiveNodes.remove(result)) {
                    result.setTwoStepsConstruction(true);
                }
                return result;
            }
            throw new ComposerException((String) null, (Mark) null, "found undefined alias " + anchor, event.getStartMark());
        }
        NodeEvent event2 = (NodeEvent) this.parser.peekEvent();
        String anchor2 = event2.getAnchor();
        if (anchor2 == null || !this.anchors.containsKey(anchor2)) {
            if (this.parser.checkEvent(Event.ID.Scalar)) {
                node = composeScalarNode(anchor2);
            } else if (this.parser.checkEvent(Event.ID.SequenceStart)) {
                node = composeSequenceNode(anchor2);
            } else {
                node = composeMappingNode(anchor2);
            }
            this.recursiveNodes.remove(parent);
            return node;
        }
        throw new ComposerException("found duplicate anchor " + anchor2 + "; first occurence", this.anchors.get(anchor2).getStartMark(), "second occurence", event2.getStartMark());
    }

    private Node composeScalarNode(String anchor) {
        Tag nodeTag;
        ScalarEvent ev = (ScalarEvent) this.parser.getEvent();
        String tag = ev.getTag();
        boolean resolved = false;
        if (tag == null || tag.equals("!")) {
            nodeTag = this.resolver.resolve(NodeId.scalar, ev.getValue(), ev.getImplicit().canOmitTagInPlainScalar());
            resolved = true;
        } else {
            nodeTag = new Tag(tag);
        }
        ScalarNode scalarNode = new ScalarNode(nodeTag, resolved, ev.getValue(), ev.getStartMark(), ev.getEndMark(), ev.getStyle());
        if (anchor != null) {
            this.anchors.put(anchor, scalarNode);
        }
        return scalarNode;
    }

    private Node composeSequenceNode(String anchor) {
        Tag nodeTag;
        SequenceStartEvent startEvent = (SequenceStartEvent) this.parser.getEvent();
        String tag = startEvent.getTag();
        boolean resolved = false;
        if (tag == null || tag.equals("!")) {
            nodeTag = this.resolver.resolve(NodeId.sequence, (String) null, startEvent.getImplicit());
            resolved = true;
        } else {
            nodeTag = new Tag(tag);
        }
        Tag nodeTag2 = nodeTag;
        ArrayList<Node> children = new ArrayList<>();
        SequenceNode node = new SequenceNode(nodeTag2, resolved, children, startEvent.getStartMark(), (Mark) null, startEvent.getFlowStyle());
        if (anchor != null) {
            this.anchors.put(anchor, node);
        }
        int index = 0;
        while (!this.parser.checkEvent(Event.ID.SequenceEnd)) {
            children.add(composeNode(node));
            index++;
        }
        node.setEndMark(this.parser.getEvent().getEndMark());
        return node;
    }

    private Node composeMappingNode(String anchor) {
        Tag nodeTag;
        MappingStartEvent startEvent = (MappingStartEvent) this.parser.getEvent();
        String tag = startEvent.getTag();
        boolean resolved = false;
        if (tag == null || tag.equals("!")) {
            nodeTag = this.resolver.resolve(NodeId.mapping, (String) null, startEvent.getImplicit());
            resolved = true;
        } else {
            nodeTag = new Tag(tag);
        }
        Tag nodeTag2 = nodeTag;
        List<NodeTuple> children = new ArrayList<>();
        MappingNode node = new MappingNode(nodeTag2, resolved, children, startEvent.getStartMark(), (Mark) null, startEvent.getFlowStyle());
        if (anchor != null) {
            this.anchors.put(anchor, node);
        }
        while (!this.parser.checkEvent(Event.ID.MappingEnd)) {
            Node itemKey = composeNode(node);
            if (itemKey.getTag().equals(Tag.MERGE)) {
                node.setMerged(true);
            } else if (itemKey.getTag().equals(Tag.VALUE)) {
                itemKey.setTag(Tag.STR);
            }
            children.add(new NodeTuple(itemKey, composeNode(node)));
        }
        node.setEndMark(this.parser.getEvent().getEndMark());
        return node;
    }
}
