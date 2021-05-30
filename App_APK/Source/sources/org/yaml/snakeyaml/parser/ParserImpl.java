package org.yaml.snakeyaml.parser;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.yaml.snakeyaml.error.Mark;
import org.yaml.snakeyaml.events.AliasEvent;
import org.yaml.snakeyaml.events.DocumentEndEvent;
import org.yaml.snakeyaml.events.DocumentStartEvent;
import org.yaml.snakeyaml.events.Event;
import org.yaml.snakeyaml.events.ImplicitTuple;
import org.yaml.snakeyaml.events.MappingEndEvent;
import org.yaml.snakeyaml.events.MappingStartEvent;
import org.yaml.snakeyaml.events.ScalarEvent;
import org.yaml.snakeyaml.events.SequenceEndEvent;
import org.yaml.snakeyaml.events.SequenceStartEvent;
import org.yaml.snakeyaml.events.StreamStartEvent;
import org.yaml.snakeyaml.nodes.Tag;
import org.yaml.snakeyaml.reader.StreamReader;
import org.yaml.snakeyaml.scanner.Scanner;
import org.yaml.snakeyaml.scanner.ScannerImpl;
import org.yaml.snakeyaml.tokens.AliasToken;
import org.yaml.snakeyaml.tokens.AnchorToken;
import org.yaml.snakeyaml.tokens.BlockEntryToken;
import org.yaml.snakeyaml.tokens.DirectiveToken;
import org.yaml.snakeyaml.tokens.ScalarToken;
import org.yaml.snakeyaml.tokens.StreamStartToken;
import org.yaml.snakeyaml.tokens.TagToken;
import org.yaml.snakeyaml.tokens.TagTuple;
import org.yaml.snakeyaml.tokens.Token;
import org.yaml.snakeyaml.util.ArrayStack;

public final class ParserImpl implements Parser {
    /* access modifiers changed from: private */
    public static final Map<String, String> DEFAULT_TAGS = new HashMap();
    private Event currentEvent = null;
    /* access modifiers changed from: private */
    public final ArrayStack<Mark> marks = new ArrayStack<>(10);
    /* access modifiers changed from: private */
    public final Scanner scanner;
    /* access modifiers changed from: private */
    public Production state = new ParseStreamStart();
    /* access modifiers changed from: private */
    public final ArrayStack<Production> states = new ArrayStack<>(100);
    /* access modifiers changed from: private */
    public Map<String, String> tagHandles = new HashMap();
    private List<Integer> yamlVersion = null;

    static {
        DEFAULT_TAGS.put("!", "!");
        DEFAULT_TAGS.put("!!", Tag.PREFIX);
    }

    public ParserImpl(StreamReader reader) {
        this.scanner = new ScannerImpl(reader);
    }

    public boolean checkEvent(Event.ID choices) {
        peekEvent();
        if (this.currentEvent == null || !this.currentEvent.is(choices)) {
            return false;
        }
        return true;
    }

    public Event peekEvent() {
        if (this.currentEvent == null && this.state != null) {
            this.currentEvent = this.state.produce();
        }
        return this.currentEvent;
    }

    public Event getEvent() {
        peekEvent();
        Event value = this.currentEvent;
        this.currentEvent = null;
        return value;
    }

    private class ParseStreamStart implements Production {
        private ParseStreamStart() {
        }

        public Event produce() {
            StreamStartToken token = (StreamStartToken) ParserImpl.this.scanner.getToken();
            Event event = new StreamStartEvent(token.getStartMark(), token.getEndMark());
            Production unused = ParserImpl.this.state = new ParseImplicitDocumentStart();
            return event;
        }
    }

    private class ParseImplicitDocumentStart implements Production {
        private ParseImplicitDocumentStart() {
        }

        public Event produce() {
            if (ParserImpl.this.scanner.checkToken(Token.ID.Directive, Token.ID.DocumentStart, Token.ID.StreamEnd)) {
                return new ParseDocumentStart().produce();
            }
            Map unused = ParserImpl.this.tagHandles = ParserImpl.DEFAULT_TAGS;
            Mark startMark = ParserImpl.this.scanner.peekToken().getStartMark();
            Event event = new DocumentStartEvent(startMark, startMark, false, (Integer[]) null, (Map<String, String>) null);
            ParserImpl.this.states.push(new ParseDocumentEnd());
            Production unused2 = ParserImpl.this.state = new ParseBlockNode();
            return event;
        }
    }

    private class ParseDocumentStart implements Production {
        private ParseDocumentStart() {
        }

        /* JADX WARNING: type inference failed for: r5v18, types: [java.lang.Object[]] */
        /* JADX WARNING: Multi-variable type inference failed */
        /* Code decompiled incorrectly, please refer to instructions dump. */
        public org.yaml.snakeyaml.events.Event produce() {
            /*
                r14 = this;
            L_0x0000:
                org.yaml.snakeyaml.parser.ParserImpl r0 = org.yaml.snakeyaml.parser.ParserImpl.this
                org.yaml.snakeyaml.scanner.Scanner r0 = r0.scanner
                r1 = 1
                org.yaml.snakeyaml.tokens.Token$ID[] r2 = new org.yaml.snakeyaml.tokens.Token.ID[r1]
                org.yaml.snakeyaml.tokens.Token$ID r3 = org.yaml.snakeyaml.tokens.Token.ID.DocumentEnd
                r4 = 0
                r2[r4] = r3
                boolean r0 = r0.checkToken(r2)
                if (r0 == 0) goto L_0x001e
                org.yaml.snakeyaml.parser.ParserImpl r0 = org.yaml.snakeyaml.parser.ParserImpl.this
                org.yaml.snakeyaml.scanner.Scanner r0 = r0.scanner
                r0.getToken()
                goto L_0x0000
            L_0x001e:
                org.yaml.snakeyaml.parser.ParserImpl r0 = org.yaml.snakeyaml.parser.ParserImpl.this
                org.yaml.snakeyaml.scanner.Scanner r0 = r0.scanner
                org.yaml.snakeyaml.tokens.Token$ID[] r2 = new org.yaml.snakeyaml.tokens.Token.ID[r1]
                org.yaml.snakeyaml.tokens.Token$ID r3 = org.yaml.snakeyaml.tokens.Token.ID.StreamEnd
                r2[r4] = r3
                boolean r0 = r0.checkToken(r2)
                r2 = 0
                if (r0 != 0) goto L_0x00dc
                org.yaml.snakeyaml.parser.ParserImpl r0 = org.yaml.snakeyaml.parser.ParserImpl.this
                org.yaml.snakeyaml.scanner.Scanner r0 = r0.scanner
                org.yaml.snakeyaml.tokens.Token r0 = r0.peekToken()
                org.yaml.snakeyaml.error.Mark r3 = r0.getStartMark()
                org.yaml.snakeyaml.parser.ParserImpl r5 = org.yaml.snakeyaml.parser.ParserImpl.this
                java.util.List r11 = r5.processDirectives()
                java.lang.Object r5 = r11.get(r4)
                r12 = r5
                java.util.List r12 = (java.util.List) r12
                java.lang.Object r5 = r11.get(r1)
                r13 = r5
                java.util.Map r13 = (java.util.Map) r13
                org.yaml.snakeyaml.parser.ParserImpl r5 = org.yaml.snakeyaml.parser.ParserImpl.this
                org.yaml.snakeyaml.scanner.Scanner r5 = r5.scanner
                org.yaml.snakeyaml.tokens.Token$ID[] r1 = new org.yaml.snakeyaml.tokens.Token.ID[r1]
                org.yaml.snakeyaml.tokens.Token$ID r6 = org.yaml.snakeyaml.tokens.Token.ID.DocumentStart
                r1[r4] = r6
                boolean r1 = r5.checkToken(r1)
                if (r1 == 0) goto L_0x00a9
                org.yaml.snakeyaml.parser.ParserImpl r1 = org.yaml.snakeyaml.parser.ParserImpl.this
                org.yaml.snakeyaml.scanner.Scanner r1 = r1.scanner
                org.yaml.snakeyaml.tokens.Token r0 = r1.getToken()
                org.yaml.snakeyaml.error.Mark r1 = r0.getEndMark()
                if (r12 == 0) goto L_0x0081
                r4 = 2
                java.lang.Integer[] r4 = new java.lang.Integer[r4]
                java.lang.Object[] r5 = r12.toArray(r4)
                r4 = r5
                java.lang.Integer[] r4 = (java.lang.Integer[]) r4
                r9 = r4
                goto L_0x0082
            L_0x0081:
                r9 = r2
            L_0x0082:
                org.yaml.snakeyaml.events.DocumentStartEvent r4 = new org.yaml.snakeyaml.events.DocumentStartEvent
                r8 = 1
                r5 = r4
                r6 = r3
                r7 = r1
                r10 = r13
                r5.<init>(r6, r7, r8, r9, r10)
                org.yaml.snakeyaml.parser.ParserImpl r5 = org.yaml.snakeyaml.parser.ParserImpl.this
                org.yaml.snakeyaml.util.ArrayStack r5 = r5.states
                org.yaml.snakeyaml.parser.ParserImpl$ParseDocumentEnd r6 = new org.yaml.snakeyaml.parser.ParserImpl$ParseDocumentEnd
                org.yaml.snakeyaml.parser.ParserImpl r7 = org.yaml.snakeyaml.parser.ParserImpl.this
                r6.<init>()
                r5.push(r6)
                org.yaml.snakeyaml.parser.ParserImpl r5 = org.yaml.snakeyaml.parser.ParserImpl.this
                org.yaml.snakeyaml.parser.ParserImpl$ParseDocumentContent r6 = new org.yaml.snakeyaml.parser.ParserImpl$ParseDocumentContent
                org.yaml.snakeyaml.parser.ParserImpl r7 = org.yaml.snakeyaml.parser.ParserImpl.this
                r6.<init>()
                org.yaml.snakeyaml.parser.Production unused = r5.state = r6
                goto L_0x0113
            L_0x00a9:
                org.yaml.snakeyaml.parser.ParserException r1 = new org.yaml.snakeyaml.parser.ParserException
                java.lang.StringBuilder r4 = new java.lang.StringBuilder
                r4.<init>()
                java.lang.String r5 = "expected '<document start>', but found "
                r4.append(r5)
                org.yaml.snakeyaml.parser.ParserImpl r5 = org.yaml.snakeyaml.parser.ParserImpl.this
                org.yaml.snakeyaml.scanner.Scanner r5 = r5.scanner
                org.yaml.snakeyaml.tokens.Token r5 = r5.peekToken()
                org.yaml.snakeyaml.tokens.Token$ID r5 = r5.getTokenId()
                r4.append(r5)
                java.lang.String r4 = r4.toString()
                org.yaml.snakeyaml.parser.ParserImpl r5 = org.yaml.snakeyaml.parser.ParserImpl.this
                org.yaml.snakeyaml.scanner.Scanner r5 = r5.scanner
                org.yaml.snakeyaml.tokens.Token r5 = r5.peekToken()
                org.yaml.snakeyaml.error.Mark r5 = r5.getStartMark()
                r1.<init>(r2, r2, r4, r5)
                throw r1
            L_0x00dc:
                org.yaml.snakeyaml.parser.ParserImpl r0 = org.yaml.snakeyaml.parser.ParserImpl.this
                org.yaml.snakeyaml.scanner.Scanner r0 = r0.scanner
                org.yaml.snakeyaml.tokens.Token r0 = r0.getToken()
                org.yaml.snakeyaml.tokens.StreamEndToken r0 = (org.yaml.snakeyaml.tokens.StreamEndToken) r0
                org.yaml.snakeyaml.events.StreamEndEvent r1 = new org.yaml.snakeyaml.events.StreamEndEvent
                org.yaml.snakeyaml.error.Mark r3 = r0.getStartMark()
                org.yaml.snakeyaml.error.Mark r4 = r0.getEndMark()
                r1.<init>(r3, r4)
                r4 = r1
                org.yaml.snakeyaml.parser.ParserImpl r1 = org.yaml.snakeyaml.parser.ParserImpl.this
                org.yaml.snakeyaml.util.ArrayStack r1 = r1.states
                boolean r1 = r1.isEmpty()
                if (r1 == 0) goto L_0x0131
                org.yaml.snakeyaml.parser.ParserImpl r1 = org.yaml.snakeyaml.parser.ParserImpl.this
                org.yaml.snakeyaml.util.ArrayStack r1 = r1.marks
                boolean r1 = r1.isEmpty()
                if (r1 == 0) goto L_0x0114
                org.yaml.snakeyaml.parser.ParserImpl r1 = org.yaml.snakeyaml.parser.ParserImpl.this
                org.yaml.snakeyaml.parser.Production unused = r1.state = r2
            L_0x0113:
                return r4
            L_0x0114:
                org.yaml.snakeyaml.error.YAMLException r1 = new org.yaml.snakeyaml.error.YAMLException
                java.lang.StringBuilder r2 = new java.lang.StringBuilder
                r2.<init>()
                java.lang.String r3 = "Unexpected end of stream. Marks left: "
                r2.append(r3)
                org.yaml.snakeyaml.parser.ParserImpl r3 = org.yaml.snakeyaml.parser.ParserImpl.this
                org.yaml.snakeyaml.util.ArrayStack r3 = r3.marks
                r2.append(r3)
                java.lang.String r2 = r2.toString()
                r1.<init>((java.lang.String) r2)
                throw r1
            L_0x0131:
                org.yaml.snakeyaml.error.YAMLException r1 = new org.yaml.snakeyaml.error.YAMLException
                java.lang.StringBuilder r2 = new java.lang.StringBuilder
                r2.<init>()
                java.lang.String r3 = "Unexpected end of stream. States left: "
                r2.append(r3)
                org.yaml.snakeyaml.parser.ParserImpl r3 = org.yaml.snakeyaml.parser.ParserImpl.this
                org.yaml.snakeyaml.util.ArrayStack r3 = r3.states
                r2.append(r3)
                java.lang.String r2 = r2.toString()
                r1.<init>((java.lang.String) r2)
                throw r1
            */
            throw new UnsupportedOperationException("Method not decompiled: org.yaml.snakeyaml.parser.ParserImpl.ParseDocumentStart.produce():org.yaml.snakeyaml.events.Event");
        }
    }

    private class ParseDocumentEnd implements Production {
        private ParseDocumentEnd() {
        }

        public Event produce() {
            Mark startMark = ParserImpl.this.scanner.peekToken().getStartMark();
            Mark endMark = startMark;
            boolean explicit = false;
            if (ParserImpl.this.scanner.checkToken(Token.ID.DocumentEnd)) {
                endMark = ParserImpl.this.scanner.getToken().getEndMark();
                explicit = true;
            }
            Event event = new DocumentEndEvent(startMark, endMark, explicit);
            Production unused = ParserImpl.this.state = new ParseDocumentStart();
            return event;
        }
    }

    private class ParseDocumentContent implements Production {
        private ParseDocumentContent() {
        }

        public Event produce() {
            if (!ParserImpl.this.scanner.checkToken(Token.ID.Directive, Token.ID.DocumentStart, Token.ID.DocumentEnd, Token.ID.StreamEnd)) {
                return new ParseBlockNode().produce();
            }
            Event event = ParserImpl.this.processEmptyScalar(ParserImpl.this.scanner.peekToken().getStartMark());
            Production unused = ParserImpl.this.state = (Production) ParserImpl.this.states.pop();
            return event;
        }
    }

    /* access modifiers changed from: private */
    public List<Object> processDirectives() {
        this.yamlVersion = null;
        this.tagHandles = new HashMap();
        while (true) {
            if (this.scanner.checkToken(Token.ID.Directive)) {
                DirectiveToken token = (DirectiveToken) this.scanner.getToken();
                if (token.getName().equals("YAML")) {
                    if (this.yamlVersion != null) {
                        throw new ParserException((String) null, (Mark) null, "found duplicate YAML directive", token.getStartMark());
                    } else if (token.getValue().get(0).intValue() == 1) {
                        this.yamlVersion = token.getValue();
                    } else {
                        throw new ParserException((String) null, (Mark) null, "found incompatible YAML document (version 1.* is required)", token.getStartMark());
                    }
                } else if (token.getName().equals("TAG")) {
                    List<String> value = token.getValue();
                    String handle = value.get(0);
                    String prefix = value.get(1);
                    if (!this.tagHandles.containsKey(handle)) {
                        this.tagHandles.put(handle, prefix);
                    } else {
                        throw new ParserException((String) null, (Mark) null, "duplicate tag handle " + handle, token.getStartMark());
                    }
                } else {
                    continue;
                }
            } else {
                List<Object> value2 = new ArrayList<>(2);
                value2.add(this.yamlVersion);
                if (!this.tagHandles.isEmpty()) {
                    value2.add(new HashMap(this.tagHandles));
                } else {
                    value2.add(new HashMap());
                }
                for (String key : DEFAULT_TAGS.keySet()) {
                    if (!this.tagHandles.containsKey(key)) {
                        this.tagHandles.put(key, DEFAULT_TAGS.get(key));
                    }
                }
                return value2;
            }
        }
    }

    private class ParseBlockNode implements Production {
        private ParseBlockNode() {
        }

        public Event produce() {
            return ParserImpl.this.parseNode(true, false);
        }
    }

    /* access modifiers changed from: private */
    public Event parseFlowNode() {
        return parseNode(false, false);
    }

    /* access modifiers changed from: private */
    public Event parseBlockNodeOrIndentlessSequence() {
        return parseNode(true, true);
    }

    /* access modifiers changed from: private */
    public Event parseNode(boolean block, boolean indentlessSequence) {
        String node;
        ImplicitTuple implicitValues;
        Mark startMark = null;
        Mark endMark = null;
        Mark tagMark = null;
        if (this.scanner.checkToken(Token.ID.Alias)) {
            AliasToken token = (AliasToken) this.scanner.getToken();
            Event event = new AliasEvent(token.getValue(), token.getStartMark(), token.getEndMark());
            this.state = this.states.pop();
            return event;
        }
        String anchor = null;
        TagTuple tagTokenTag = null;
        if (this.scanner.checkToken(Token.ID.Anchor)) {
            AnchorToken token2 = (AnchorToken) this.scanner.getToken();
            startMark = token2.getStartMark();
            endMark = token2.getEndMark();
            anchor = token2.getValue();
            if (this.scanner.checkToken(Token.ID.Tag)) {
                TagToken tagToken = (TagToken) this.scanner.getToken();
                tagMark = tagToken.getStartMark();
                endMark = tagToken.getEndMark();
                tagTokenTag = tagToken.getValue();
            }
        } else {
            if (this.scanner.checkToken(Token.ID.Tag)) {
                TagToken tagToken2 = (TagToken) this.scanner.getToken();
                startMark = tagToken2.getStartMark();
                tagMark = startMark;
                endMark = tagToken2.getEndMark();
                tagTokenTag = tagToken2.getValue();
                if (this.scanner.checkToken(Token.ID.Anchor)) {
                    AnchorToken token3 = (AnchorToken) this.scanner.getToken();
                    endMark = token3.getEndMark();
                    anchor = token3.getValue();
                }
            }
        }
        String tag = null;
        if (tagTokenTag != null) {
            String handle = tagTokenTag.getHandle();
            String suffix = tagTokenTag.getSuffix();
            if (handle == null) {
                tag = suffix;
            } else if (this.tagHandles.containsKey(handle)) {
                tag = this.tagHandles.get(handle) + suffix;
            } else {
                throw new ParserException("while parsing a node", startMark, "found undefined tag handle " + handle, tagMark);
            }
        }
        if (startMark == null) {
            startMark = this.scanner.peekToken().getStartMark();
            endMark = startMark;
        }
        boolean implicit = tag == null || tag.equals("!");
        if (indentlessSequence) {
            if (this.scanner.checkToken(Token.ID.BlockEntry)) {
                boolean z = implicit;
                SequenceStartEvent sequenceStartEvent = new SequenceStartEvent(anchor, tag, implicit, startMark, this.scanner.peekToken().getEndMark(), Boolean.FALSE);
                this.state = new ParseIndentlessSequenceEntry();
                return sequenceStartEvent;
            }
        }
        boolean implicit2 = implicit;
        if (this.scanner.checkToken(Token.ID.Scalar)) {
            ScalarToken token4 = (ScalarToken) this.scanner.getToken();
            Mark endMark2 = token4.getEndMark();
            if ((token4.getPlain() && tag == null) || "!".equals(tag)) {
                implicitValues = new ImplicitTuple(true, false);
            } else if (tag == null) {
                implicitValues = new ImplicitTuple(false, true);
            } else {
                implicitValues = new ImplicitTuple(false, false);
            }
            ScalarEvent scalarEvent = new ScalarEvent(anchor, tag, implicitValues, token4.getValue(), startMark, endMark2, Character.valueOf(token4.getStyle()));
            this.state = this.states.pop();
            return scalarEvent;
        }
        if (this.scanner.checkToken(Token.ID.FlowSequenceStart)) {
            SequenceStartEvent sequenceStartEvent2 = new SequenceStartEvent(anchor, tag, implicit2, startMark, this.scanner.peekToken().getEndMark(), Boolean.TRUE);
            this.state = new ParseFlowSequenceFirstEntry();
            return sequenceStartEvent2;
        }
        if (this.scanner.checkToken(Token.ID.FlowMappingStart)) {
            MappingStartEvent mappingStartEvent = new MappingStartEvent(anchor, tag, implicit2, startMark, this.scanner.peekToken().getEndMark(), Boolean.TRUE);
            this.state = new ParseFlowMappingFirstKey();
            return mappingStartEvent;
        }
        if (block) {
            if (this.scanner.checkToken(Token.ID.BlockSequenceStart)) {
                SequenceStartEvent sequenceStartEvent3 = new SequenceStartEvent(anchor, tag, implicit2, startMark, this.scanner.peekToken().getStartMark(), Boolean.FALSE);
                this.state = new ParseBlockSequenceFirstEntry();
                return sequenceStartEvent3;
            }
        }
        if (block) {
            if (this.scanner.checkToken(Token.ID.BlockMappingStart)) {
                MappingStartEvent mappingStartEvent2 = new MappingStartEvent(anchor, tag, implicit2, startMark, this.scanner.peekToken().getStartMark(), Boolean.FALSE);
                this.state = new ParseBlockMappingFirstKey();
                return mappingStartEvent2;
            }
        }
        if (anchor == null && tag == null) {
            if (block) {
                node = "block";
            } else {
                node = "flow";
            }
            Token token5 = this.scanner.peekToken();
            throw new ParserException("while parsing a " + node + " node", startMark, "expected the node content, but found " + token5.getTokenId(), token5.getStartMark());
        }
        boolean implicit3 = implicit2;
        boolean z2 = implicit3;
        ScalarEvent scalarEvent2 = new ScalarEvent(anchor, tag, new ImplicitTuple(implicit3, false), "", startMark, endMark, 0);
        this.state = this.states.pop();
        return scalarEvent2;
    }

    private class ParseBlockSequenceFirstEntry implements Production {
        private ParseBlockSequenceFirstEntry() {
        }

        public Event produce() {
            ParserImpl.this.marks.push(ParserImpl.this.scanner.getToken().getStartMark());
            return new ParseBlockSequenceEntry().produce();
        }
    }

    private class ParseBlockSequenceEntry implements Production {
        private ParseBlockSequenceEntry() {
        }

        public Event produce() {
            if (ParserImpl.this.scanner.checkToken(Token.ID.BlockEntry)) {
                BlockEntryToken token = (BlockEntryToken) ParserImpl.this.scanner.getToken();
                if (!ParserImpl.this.scanner.checkToken(Token.ID.BlockEntry, Token.ID.BlockEnd)) {
                    ParserImpl.this.states.push(new ParseBlockSequenceEntry());
                    return new ParseBlockNode().produce();
                }
                Production unused = ParserImpl.this.state = new ParseBlockSequenceEntry();
                return ParserImpl.this.processEmptyScalar(token.getEndMark());
            }
            if (ParserImpl.this.scanner.checkToken(Token.ID.BlockEnd)) {
                Token token2 = ParserImpl.this.scanner.getToken();
                Event event = new SequenceEndEvent(token2.getStartMark(), token2.getEndMark());
                Production unused2 = ParserImpl.this.state = (Production) ParserImpl.this.states.pop();
                ParserImpl.this.marks.pop();
                return event;
            }
            Token token3 = ParserImpl.this.scanner.peekToken();
            throw new ParserException("while parsing a block collection", (Mark) ParserImpl.this.marks.pop(), "expected <block end>, but found " + token3.getTokenId(), token3.getStartMark());
        }
    }

    private class ParseIndentlessSequenceEntry implements Production {
        private ParseIndentlessSequenceEntry() {
        }

        public Event produce() {
            if (ParserImpl.this.scanner.checkToken(Token.ID.BlockEntry)) {
                Token token = ParserImpl.this.scanner.getToken();
                if (!ParserImpl.this.scanner.checkToken(Token.ID.BlockEntry, Token.ID.Key, Token.ID.Value, Token.ID.BlockEnd)) {
                    ParserImpl.this.states.push(new ParseIndentlessSequenceEntry());
                    return new ParseBlockNode().produce();
                }
                Production unused = ParserImpl.this.state = new ParseIndentlessSequenceEntry();
                return ParserImpl.this.processEmptyScalar(token.getEndMark());
            }
            Token token2 = ParserImpl.this.scanner.peekToken();
            Event event = new SequenceEndEvent(token2.getStartMark(), token2.getEndMark());
            Production unused2 = ParserImpl.this.state = (Production) ParserImpl.this.states.pop();
            return event;
        }
    }

    private class ParseBlockMappingFirstKey implements Production {
        private ParseBlockMappingFirstKey() {
        }

        public Event produce() {
            ParserImpl.this.marks.push(ParserImpl.this.scanner.getToken().getStartMark());
            return new ParseBlockMappingKey().produce();
        }
    }

    private class ParseBlockMappingKey implements Production {
        private ParseBlockMappingKey() {
        }

        public Event produce() {
            if (ParserImpl.this.scanner.checkToken(Token.ID.Key)) {
                Token token = ParserImpl.this.scanner.getToken();
                if (!ParserImpl.this.scanner.checkToken(Token.ID.Key, Token.ID.Value, Token.ID.BlockEnd)) {
                    ParserImpl.this.states.push(new ParseBlockMappingValue());
                    return ParserImpl.this.parseBlockNodeOrIndentlessSequence();
                }
                Production unused = ParserImpl.this.state = new ParseBlockMappingValue();
                return ParserImpl.this.processEmptyScalar(token.getEndMark());
            }
            if (ParserImpl.this.scanner.checkToken(Token.ID.BlockEnd)) {
                Token token2 = ParserImpl.this.scanner.getToken();
                Event event = new MappingEndEvent(token2.getStartMark(), token2.getEndMark());
                Production unused2 = ParserImpl.this.state = (Production) ParserImpl.this.states.pop();
                ParserImpl.this.marks.pop();
                return event;
            }
            Token token3 = ParserImpl.this.scanner.peekToken();
            throw new ParserException("while parsing a block mapping", (Mark) ParserImpl.this.marks.pop(), "expected <block end>, but found " + token3.getTokenId(), token3.getStartMark());
        }
    }

    private class ParseBlockMappingValue implements Production {
        private ParseBlockMappingValue() {
        }

        public Event produce() {
            if (ParserImpl.this.scanner.checkToken(Token.ID.Value)) {
                Token token = ParserImpl.this.scanner.getToken();
                if (!ParserImpl.this.scanner.checkToken(Token.ID.Key, Token.ID.Value, Token.ID.BlockEnd)) {
                    ParserImpl.this.states.push(new ParseBlockMappingKey());
                    return ParserImpl.this.parseBlockNodeOrIndentlessSequence();
                }
                Production unused = ParserImpl.this.state = new ParseBlockMappingKey();
                return ParserImpl.this.processEmptyScalar(token.getEndMark());
            }
            Production unused2 = ParserImpl.this.state = new ParseBlockMappingKey();
            return ParserImpl.this.processEmptyScalar(ParserImpl.this.scanner.peekToken().getStartMark());
        }
    }

    private class ParseFlowSequenceFirstEntry implements Production {
        private ParseFlowSequenceFirstEntry() {
        }

        public Event produce() {
            ParserImpl.this.marks.push(ParserImpl.this.scanner.getToken().getStartMark());
            return new ParseFlowSequenceEntry(true).produce();
        }
    }

    private class ParseFlowSequenceEntry implements Production {
        private boolean first = false;

        public ParseFlowSequenceEntry(boolean first2) {
            this.first = first2;
        }

        public Event produce() {
            if (!ParserImpl.this.scanner.checkToken(Token.ID.FlowSequenceEnd)) {
                if (!this.first) {
                    if (ParserImpl.this.scanner.checkToken(Token.ID.FlowEntry)) {
                        ParserImpl.this.scanner.getToken();
                    } else {
                        Token token = ParserImpl.this.scanner.peekToken();
                        throw new ParserException("while parsing a flow sequence", (Mark) ParserImpl.this.marks.pop(), "expected ',' or ']', but got " + token.getTokenId(), token.getStartMark());
                    }
                }
                if (ParserImpl.this.scanner.checkToken(Token.ID.Key)) {
                    Token token2 = ParserImpl.this.scanner.peekToken();
                    Event event = new MappingStartEvent((String) null, (String) null, true, token2.getStartMark(), token2.getEndMark(), Boolean.TRUE);
                    Production unused = ParserImpl.this.state = new ParseFlowSequenceEntryMappingKey();
                    return event;
                }
                if (!ParserImpl.this.scanner.checkToken(Token.ID.FlowSequenceEnd)) {
                    ParserImpl.this.states.push(new ParseFlowSequenceEntry(false));
                    return ParserImpl.this.parseFlowNode();
                }
            }
            Token token3 = ParserImpl.this.scanner.getToken();
            Event event2 = new SequenceEndEvent(token3.getStartMark(), token3.getEndMark());
            Production unused2 = ParserImpl.this.state = (Production) ParserImpl.this.states.pop();
            ParserImpl.this.marks.pop();
            return event2;
        }
    }

    private class ParseFlowSequenceEntryMappingKey implements Production {
        private ParseFlowSequenceEntryMappingKey() {
        }

        public Event produce() {
            Token token = ParserImpl.this.scanner.getToken();
            if (!ParserImpl.this.scanner.checkToken(Token.ID.Value, Token.ID.FlowEntry, Token.ID.FlowSequenceEnd)) {
                ParserImpl.this.states.push(new ParseFlowSequenceEntryMappingValue());
                return ParserImpl.this.parseFlowNode();
            }
            Production unused = ParserImpl.this.state = new ParseFlowSequenceEntryMappingValue();
            return ParserImpl.this.processEmptyScalar(token.getEndMark());
        }
    }

    private class ParseFlowSequenceEntryMappingValue implements Production {
        private ParseFlowSequenceEntryMappingValue() {
        }

        public Event produce() {
            if (ParserImpl.this.scanner.checkToken(Token.ID.Value)) {
                Token token = ParserImpl.this.scanner.getToken();
                if (!ParserImpl.this.scanner.checkToken(Token.ID.FlowEntry, Token.ID.FlowSequenceEnd)) {
                    ParserImpl.this.states.push(new ParseFlowSequenceEntryMappingEnd());
                    return ParserImpl.this.parseFlowNode();
                }
                Production unused = ParserImpl.this.state = new ParseFlowSequenceEntryMappingEnd();
                return ParserImpl.this.processEmptyScalar(token.getEndMark());
            }
            Production unused2 = ParserImpl.this.state = new ParseFlowSequenceEntryMappingEnd();
            return ParserImpl.this.processEmptyScalar(ParserImpl.this.scanner.peekToken().getStartMark());
        }
    }

    private class ParseFlowSequenceEntryMappingEnd implements Production {
        private ParseFlowSequenceEntryMappingEnd() {
        }

        public Event produce() {
            Production unused = ParserImpl.this.state = new ParseFlowSequenceEntry(false);
            Token token = ParserImpl.this.scanner.peekToken();
            return new MappingEndEvent(token.getStartMark(), token.getEndMark());
        }
    }

    private class ParseFlowMappingFirstKey implements Production {
        private ParseFlowMappingFirstKey() {
        }

        public Event produce() {
            ParserImpl.this.marks.push(ParserImpl.this.scanner.getToken().getStartMark());
            return new ParseFlowMappingKey(true).produce();
        }
    }

    private class ParseFlowMappingKey implements Production {
        private boolean first = false;

        public ParseFlowMappingKey(boolean first2) {
            this.first = first2;
        }

        public Event produce() {
            if (!ParserImpl.this.scanner.checkToken(Token.ID.FlowMappingEnd)) {
                if (!this.first) {
                    if (ParserImpl.this.scanner.checkToken(Token.ID.FlowEntry)) {
                        ParserImpl.this.scanner.getToken();
                    } else {
                        Token token = ParserImpl.this.scanner.peekToken();
                        throw new ParserException("while parsing a flow mapping", (Mark) ParserImpl.this.marks.pop(), "expected ',' or '}', but got " + token.getTokenId(), token.getStartMark());
                    }
                }
                if (ParserImpl.this.scanner.checkToken(Token.ID.Key)) {
                    Token token2 = ParserImpl.this.scanner.getToken();
                    if (!ParserImpl.this.scanner.checkToken(Token.ID.Value, Token.ID.FlowEntry, Token.ID.FlowMappingEnd)) {
                        ParserImpl.this.states.push(new ParseFlowMappingValue());
                        return ParserImpl.this.parseFlowNode();
                    }
                    Production unused = ParserImpl.this.state = new ParseFlowMappingValue();
                    return ParserImpl.this.processEmptyScalar(token2.getEndMark());
                }
                if (!ParserImpl.this.scanner.checkToken(Token.ID.FlowMappingEnd)) {
                    ParserImpl.this.states.push(new ParseFlowMappingEmptyValue());
                    return ParserImpl.this.parseFlowNode();
                }
            }
            Token token3 = ParserImpl.this.scanner.getToken();
            Event event = new MappingEndEvent(token3.getStartMark(), token3.getEndMark());
            Production unused2 = ParserImpl.this.state = (Production) ParserImpl.this.states.pop();
            ParserImpl.this.marks.pop();
            return event;
        }
    }

    private class ParseFlowMappingValue implements Production {
        private ParseFlowMappingValue() {
        }

        public Event produce() {
            if (ParserImpl.this.scanner.checkToken(Token.ID.Value)) {
                Token token = ParserImpl.this.scanner.getToken();
                if (!ParserImpl.this.scanner.checkToken(Token.ID.FlowEntry, Token.ID.FlowMappingEnd)) {
                    ParserImpl.this.states.push(new ParseFlowMappingKey(false));
                    return ParserImpl.this.parseFlowNode();
                }
                Production unused = ParserImpl.this.state = new ParseFlowMappingKey(false);
                return ParserImpl.this.processEmptyScalar(token.getEndMark());
            }
            Production unused2 = ParserImpl.this.state = new ParseFlowMappingKey(false);
            return ParserImpl.this.processEmptyScalar(ParserImpl.this.scanner.peekToken().getStartMark());
        }
    }

    private class ParseFlowMappingEmptyValue implements Production {
        private ParseFlowMappingEmptyValue() {
        }

        public Event produce() {
            Production unused = ParserImpl.this.state = new ParseFlowMappingKey(false);
            return ParserImpl.this.processEmptyScalar(ParserImpl.this.scanner.peekToken().getStartMark());
        }
    }

    /* access modifiers changed from: private */
    public Event processEmptyScalar(Mark mark) {
        return new ScalarEvent((String) null, (String) null, new ImplicitTuple(true, false), "", mark, mark, 0);
    }
}
