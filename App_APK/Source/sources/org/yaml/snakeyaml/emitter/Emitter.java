package org.yaml.snakeyaml.emitter;

import java.io.IOException;
import java.io.Writer;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Queue;
import java.util.TreeSet;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.regex.Pattern;
import org.apache.commons.io.IOUtils;
import org.apache.commons.lang.CharUtils;
import org.yaml.snakeyaml.DumperOptions;
import org.yaml.snakeyaml.error.YAMLException;
import org.yaml.snakeyaml.events.AliasEvent;
import org.yaml.snakeyaml.events.CollectionEndEvent;
import org.yaml.snakeyaml.events.CollectionStartEvent;
import org.yaml.snakeyaml.events.DocumentEndEvent;
import org.yaml.snakeyaml.events.DocumentStartEvent;
import org.yaml.snakeyaml.events.Event;
import org.yaml.snakeyaml.events.MappingEndEvent;
import org.yaml.snakeyaml.events.MappingStartEvent;
import org.yaml.snakeyaml.events.NodeEvent;
import org.yaml.snakeyaml.events.ScalarEvent;
import org.yaml.snakeyaml.events.SequenceEndEvent;
import org.yaml.snakeyaml.events.SequenceStartEvent;
import org.yaml.snakeyaml.events.StreamEndEvent;
import org.yaml.snakeyaml.events.StreamStartEvent;
import org.yaml.snakeyaml.nodes.Tag;
import org.yaml.snakeyaml.scanner.Constant;
import org.yaml.snakeyaml.util.ArrayStack;

public final class Emitter implements Emitable {
    private static final Pattern ANCHOR_FORMAT = Pattern.compile("^[-_\\w]*$");
    /* access modifiers changed from: private */
    public static final Map<String, String> DEFAULT_TAG_PREFIXES = new LinkedHashMap();
    private static final Map<Character, String> ESCAPE_REPLACEMENTS = new HashMap();
    private static final Pattern HANDLE_FORMAT = Pattern.compile("^![-_\\w]*!$");
    public static final int MAX_INDENT = 10;
    public static final int MIN_INDENT = 1;
    private static final char[] SPACE = {' '};
    private boolean allowUnicode;
    private ScalarAnalysis analysis;
    private int bestIndent;
    private char[] bestLineBreak;
    /* access modifiers changed from: private */
    public int bestWidth;
    /* access modifiers changed from: private */
    public Boolean canonical;
    /* access modifiers changed from: private */
    public int column = 0;
    /* access modifiers changed from: private */
    public Event event = null;
    private final Queue<Event> events = new ArrayBlockingQueue(100);
    private int flowLevel = 0;
    /* access modifiers changed from: private */
    public Integer indent = null;
    private boolean indention = true;
    /* access modifiers changed from: private */
    public final ArrayStack<Integer> indents = new ArrayStack<>(10);
    private boolean mappingContext = false;
    /* access modifiers changed from: private */
    public boolean openEnded = false;
    private DumperOptions options;
    private String preparedAnchor;
    private String preparedTag;
    /* access modifiers changed from: private */
    public Boolean prettyFlow;
    private boolean rootContext;
    private boolean simpleKeyContext = false;
    /* access modifiers changed from: private */
    public EmitterState state = new ExpectStreamStart();
    /* access modifiers changed from: private */
    public final ArrayStack<EmitterState> states = new ArrayStack<>(100);
    private final Writer stream;
    private Character style;
    /* access modifiers changed from: private */
    public Map<String, String> tagPrefixes;
    private boolean whitespace = true;

    static /* synthetic */ int access$2010(Emitter x0) {
        int i = x0.flowLevel;
        x0.flowLevel = i - 1;
        return i;
    }

    static {
        ESCAPE_REPLACEMENTS.put(0, "0");
        ESCAPE_REPLACEMENTS.put(7, "a");
        ESCAPE_REPLACEMENTS.put(8, "b");
        ESCAPE_REPLACEMENTS.put(9, "t");
        ESCAPE_REPLACEMENTS.put(10, "n");
        ESCAPE_REPLACEMENTS.put(11, "v");
        ESCAPE_REPLACEMENTS.put(12, "f");
        ESCAPE_REPLACEMENTS.put(Character.valueOf(CharUtils.CR), "r");
        ESCAPE_REPLACEMENTS.put(27, "e");
        ESCAPE_REPLACEMENTS.put('\"', "\"");
        ESCAPE_REPLACEMENTS.put(Character.valueOf(IOUtils.DIR_SEPARATOR_WINDOWS), "\\");
        ESCAPE_REPLACEMENTS.put(133, "N");
        ESCAPE_REPLACEMENTS.put(160, "_");
        ESCAPE_REPLACEMENTS.put(8232, "L");
        ESCAPE_REPLACEMENTS.put(8233, "P");
        DEFAULT_TAG_PREFIXES.put("!", "!");
        DEFAULT_TAG_PREFIXES.put(Tag.PREFIX, "!!");
    }

    public Emitter(Writer stream2, DumperOptions opts) {
        this.stream = stream2;
        this.canonical = Boolean.valueOf(opts.isCanonical());
        this.prettyFlow = Boolean.valueOf(opts.isPrettyFlow());
        this.allowUnicode = opts.isAllowUnicode();
        this.bestIndent = 2;
        if (opts.getIndent() > 1 && opts.getIndent() < 10) {
            this.bestIndent = opts.getIndent();
        }
        this.bestWidth = 80;
        if (opts.getWidth() > this.bestIndent * 2) {
            this.bestWidth = opts.getWidth();
        }
        this.bestLineBreak = opts.getLineBreak().getString().toCharArray();
        this.tagPrefixes = new LinkedHashMap();
        this.preparedAnchor = null;
        this.preparedTag = null;
        this.analysis = null;
        this.style = null;
        this.options = opts;
    }

    public void emit(Event event2) throws IOException {
        this.events.add(event2);
        while (!needMoreEvents()) {
            this.event = this.events.poll();
            this.state.expect();
            this.event = null;
        }
    }

    private boolean needMoreEvents() {
        if (this.events.isEmpty()) {
            return true;
        }
        Event event2 = this.events.peek();
        if (event2 instanceof DocumentStartEvent) {
            return needEvents(1);
        }
        if (event2 instanceof SequenceStartEvent) {
            return needEvents(2);
        }
        if (event2 instanceof MappingStartEvent) {
            return needEvents(3);
        }
        return false;
    }

    private boolean needEvents(int count) {
        int level = 0;
        Iterator<Event> iter = this.events.iterator();
        iter.next();
        while (iter.hasNext()) {
            Event event2 = iter.next();
            if ((event2 instanceof DocumentStartEvent) || (event2 instanceof CollectionStartEvent)) {
                level++;
                continue;
            } else if ((event2 instanceof DocumentEndEvent) || (event2 instanceof CollectionEndEvent)) {
                level--;
                continue;
            } else if (event2 instanceof StreamEndEvent) {
                level = -1;
                continue;
            } else {
                continue;
            }
            if (level < 0) {
                return false;
            }
        }
        if (this.events.size() < count + 1) {
            return true;
        }
        return false;
    }

    private void increaseIndent(boolean flow, boolean indentless) {
        this.indents.push(this.indent);
        if (this.indent == null) {
            if (flow) {
                this.indent = Integer.valueOf(this.bestIndent);
            } else {
                this.indent = 0;
            }
        } else if (!indentless) {
            this.indent = Integer.valueOf(this.indent.intValue() + this.bestIndent);
        }
    }

    private class ExpectStreamStart implements EmitterState {
        private ExpectStreamStart() {
        }

        public void expect() throws IOException {
            if (Emitter.this.event instanceof StreamStartEvent) {
                Emitter.this.writeStreamStart();
                EmitterState unused = Emitter.this.state = new ExpectFirstDocumentStart();
                return;
            }
            throw new EmitterException("expected StreamStartEvent, but got " + Emitter.this.event);
        }
    }

    private class ExpectNothing implements EmitterState {
        private ExpectNothing() {
        }

        public void expect() throws IOException {
            throw new EmitterException("expecting nothing, but got " + Emitter.this.event);
        }
    }

    private class ExpectFirstDocumentStart implements EmitterState {
        private ExpectFirstDocumentStart() {
        }

        public void expect() throws IOException {
            new ExpectDocumentStart(true).expect();
        }
    }

    private class ExpectDocumentStart implements EmitterState {
        private boolean first;

        public ExpectDocumentStart(boolean first2) {
            this.first = first2;
        }

        public void expect() throws IOException {
            if (Emitter.this.event instanceof DocumentStartEvent) {
                DocumentStartEvent ev = (DocumentStartEvent) Emitter.this.event;
                if (!(ev.getVersion() == null && ev.getTags() == null) && Emitter.this.openEnded) {
                    Emitter.this.writeIndicator("...", true, false, false);
                    Emitter.this.writeIndent();
                }
                if (ev.getVersion() != null) {
                    Emitter.this.writeVersionDirective(Emitter.this.prepareVersion(ev.getVersion()));
                }
                Map unused = Emitter.this.tagPrefixes = new LinkedHashMap(Emitter.DEFAULT_TAG_PREFIXES);
                if (ev.getTags() != null) {
                    for (String handle : new TreeSet<>(ev.getTags().keySet())) {
                        String prefix = ev.getTags().get(handle);
                        Emitter.this.tagPrefixes.put(prefix, handle);
                        Emitter.this.writeTagDirective(Emitter.this.prepareTagHandle(handle), Emitter.this.prepareTagPrefix(prefix));
                    }
                }
                if (!(this.first && !ev.getExplicit() && !Emitter.this.canonical.booleanValue() && ev.getVersion() == null && ev.getTags() == null && !Emitter.this.checkEmptyDocument())) {
                    Emitter.this.writeIndent();
                    Emitter.this.writeIndicator("---", true, false, false);
                    if (Emitter.this.canonical.booleanValue()) {
                        Emitter.this.writeIndent();
                    }
                }
                EmitterState unused2 = Emitter.this.state = new ExpectDocumentRoot();
            } else if (Emitter.this.event instanceof StreamEndEvent) {
                Emitter.this.writeStreamEnd();
                EmitterState unused3 = Emitter.this.state = new ExpectNothing();
            } else {
                throw new EmitterException("expected DocumentStartEvent, but got " + Emitter.this.event);
            }
        }
    }

    private class ExpectDocumentEnd implements EmitterState {
        private ExpectDocumentEnd() {
        }

        public void expect() throws IOException {
            if (Emitter.this.event instanceof DocumentEndEvent) {
                Emitter.this.writeIndent();
                if (((DocumentEndEvent) Emitter.this.event).getExplicit()) {
                    Emitter.this.writeIndicator("...", true, false, false);
                    Emitter.this.writeIndent();
                }
                Emitter.this.flushStream();
                EmitterState unused = Emitter.this.state = new ExpectDocumentStart(false);
                return;
            }
            throw new EmitterException("expected DocumentEndEvent, but got " + Emitter.this.event);
        }
    }

    private class ExpectDocumentRoot implements EmitterState {
        private ExpectDocumentRoot() {
        }

        public void expect() throws IOException {
            Emitter.this.states.push(new ExpectDocumentEnd());
            Emitter.this.expectNode(true, false, false);
        }
    }

    /* access modifiers changed from: private */
    public void expectNode(boolean root, boolean mapping, boolean simpleKey) throws IOException {
        this.rootContext = root;
        this.mappingContext = mapping;
        this.simpleKeyContext = simpleKey;
        if (this.event instanceof AliasEvent) {
            expectAlias();
        } else if ((this.event instanceof ScalarEvent) || (this.event instanceof CollectionStartEvent)) {
            processAnchor("&");
            processTag();
            if (this.event instanceof ScalarEvent) {
                expectScalar();
            } else if (this.event instanceof SequenceStartEvent) {
                if (this.flowLevel != 0 || this.canonical.booleanValue() || ((SequenceStartEvent) this.event).getFlowStyle().booleanValue() || checkEmptySequence()) {
                    expectFlowSequence();
                } else {
                    expectBlockSequence();
                }
            } else if (this.flowLevel != 0 || this.canonical.booleanValue() || ((MappingStartEvent) this.event).getFlowStyle().booleanValue() || checkEmptyMapping()) {
                expectFlowMapping();
            } else {
                expectBlockMapping();
            }
        } else {
            throw new EmitterException("expected NodeEvent, but got " + this.event);
        }
    }

    private void expectAlias() throws IOException {
        if (((NodeEvent) this.event).getAnchor() != null) {
            processAnchor("*");
            this.state = this.states.pop();
            return;
        }
        throw new EmitterException("anchor is not specified for alias");
    }

    private void expectScalar() throws IOException {
        increaseIndent(true, false);
        processScalar();
        this.indent = this.indents.pop();
        this.state = this.states.pop();
    }

    private void expectFlowSequence() throws IOException {
        writeIndicator("[", true, true, false);
        this.flowLevel++;
        increaseIndent(true, false);
        if (this.prettyFlow.booleanValue()) {
            writeIndent();
        }
        this.state = new ExpectFirstFlowSequenceItem();
    }

    private class ExpectFirstFlowSequenceItem implements EmitterState {
        private ExpectFirstFlowSequenceItem() {
        }

        public void expect() throws IOException {
            if (Emitter.this.event instanceof SequenceEndEvent) {
                Integer unused = Emitter.this.indent = (Integer) Emitter.this.indents.pop();
                Emitter.access$2010(Emitter.this);
                Emitter.this.writeIndicator("]", false, false, false);
                EmitterState unused2 = Emitter.this.state = (EmitterState) Emitter.this.states.pop();
                return;
            }
            if (Emitter.this.canonical.booleanValue() || Emitter.this.column > Emitter.this.bestWidth || Emitter.this.prettyFlow.booleanValue()) {
                Emitter.this.writeIndent();
            }
            Emitter.this.states.push(new ExpectFlowSequenceItem());
            Emitter.this.expectNode(false, false, false);
        }
    }

    private class ExpectFlowSequenceItem implements EmitterState {
        private ExpectFlowSequenceItem() {
        }

        public void expect() throws IOException {
            if (Emitter.this.event instanceof SequenceEndEvent) {
                Integer unused = Emitter.this.indent = (Integer) Emitter.this.indents.pop();
                Emitter.access$2010(Emitter.this);
                if (Emitter.this.canonical.booleanValue()) {
                    Emitter.this.writeIndicator(",", false, false, false);
                    Emitter.this.writeIndent();
                }
                Emitter.this.writeIndicator("]", false, false, false);
                if (Emitter.this.prettyFlow.booleanValue()) {
                    Emitter.this.writeIndent();
                }
                EmitterState unused2 = Emitter.this.state = (EmitterState) Emitter.this.states.pop();
                return;
            }
            Emitter.this.writeIndicator(",", false, false, false);
            if (Emitter.this.canonical.booleanValue() || Emitter.this.column > Emitter.this.bestWidth || Emitter.this.prettyFlow.booleanValue()) {
                Emitter.this.writeIndent();
            }
            Emitter.this.states.push(new ExpectFlowSequenceItem());
            Emitter.this.expectNode(false, false, false);
        }
    }

    private void expectFlowMapping() throws IOException {
        writeIndicator("{", true, true, false);
        this.flowLevel++;
        increaseIndent(true, false);
        if (this.prettyFlow.booleanValue()) {
            writeIndent();
        }
        this.state = new ExpectFirstFlowMappingKey();
    }

    private class ExpectFirstFlowMappingKey implements EmitterState {
        private ExpectFirstFlowMappingKey() {
        }

        public void expect() throws IOException {
            if (Emitter.this.event instanceof MappingEndEvent) {
                Integer unused = Emitter.this.indent = (Integer) Emitter.this.indents.pop();
                Emitter.access$2010(Emitter.this);
                Emitter.this.writeIndicator("}", false, false, false);
                EmitterState unused2 = Emitter.this.state = (EmitterState) Emitter.this.states.pop();
                return;
            }
            if (Emitter.this.canonical.booleanValue() || Emitter.this.column > Emitter.this.bestWidth || Emitter.this.prettyFlow.booleanValue()) {
                Emitter.this.writeIndent();
            }
            if (Emitter.this.canonical.booleanValue() || !Emitter.this.checkSimpleKey()) {
                Emitter.this.writeIndicator("?", true, false, false);
                Emitter.this.states.push(new ExpectFlowMappingValue());
                Emitter.this.expectNode(false, true, false);
                return;
            }
            Emitter.this.states.push(new ExpectFlowMappingSimpleValue());
            Emitter.this.expectNode(false, true, true);
        }
    }

    private class ExpectFlowMappingKey implements EmitterState {
        private ExpectFlowMappingKey() {
        }

        public void expect() throws IOException {
            if (Emitter.this.event instanceof MappingEndEvent) {
                Integer unused = Emitter.this.indent = (Integer) Emitter.this.indents.pop();
                Emitter.access$2010(Emitter.this);
                if (Emitter.this.canonical.booleanValue()) {
                    Emitter.this.writeIndicator(",", false, false, false);
                    Emitter.this.writeIndent();
                }
                if (Emitter.this.prettyFlow.booleanValue()) {
                    Emitter.this.writeIndent();
                }
                Emitter.this.writeIndicator("}", false, false, false);
                EmitterState unused2 = Emitter.this.state = (EmitterState) Emitter.this.states.pop();
                return;
            }
            Emitter.this.writeIndicator(",", false, false, false);
            if (Emitter.this.canonical.booleanValue() || Emitter.this.column > Emitter.this.bestWidth || Emitter.this.prettyFlow.booleanValue()) {
                Emitter.this.writeIndent();
            }
            if (Emitter.this.canonical.booleanValue() || !Emitter.this.checkSimpleKey()) {
                Emitter.this.writeIndicator("?", true, false, false);
                Emitter.this.states.push(new ExpectFlowMappingValue());
                Emitter.this.expectNode(false, true, false);
                return;
            }
            Emitter.this.states.push(new ExpectFlowMappingSimpleValue());
            Emitter.this.expectNode(false, true, true);
        }
    }

    private class ExpectFlowMappingSimpleValue implements EmitterState {
        private ExpectFlowMappingSimpleValue() {
        }

        public void expect() throws IOException {
            Emitter.this.writeIndicator(":", false, false, false);
            Emitter.this.states.push(new ExpectFlowMappingKey());
            Emitter.this.expectNode(false, true, false);
        }
    }

    private class ExpectFlowMappingValue implements EmitterState {
        private ExpectFlowMappingValue() {
        }

        public void expect() throws IOException {
            if (Emitter.this.canonical.booleanValue() || Emitter.this.column > Emitter.this.bestWidth || Emitter.this.prettyFlow.booleanValue()) {
                Emitter.this.writeIndent();
            }
            Emitter.this.writeIndicator(":", true, false, false);
            Emitter.this.states.push(new ExpectFlowMappingKey());
            Emitter.this.expectNode(false, true, false);
        }
    }

    private void expectBlockSequence() throws IOException {
        increaseIndent(false, this.mappingContext && !this.indention);
        this.state = new ExpectFirstBlockSequenceItem();
    }

    private class ExpectFirstBlockSequenceItem implements EmitterState {
        private ExpectFirstBlockSequenceItem() {
        }

        public void expect() throws IOException {
            new ExpectBlockSequenceItem(true).expect();
        }
    }

    private class ExpectBlockSequenceItem implements EmitterState {
        private boolean first;

        public ExpectBlockSequenceItem(boolean first2) {
            this.first = first2;
        }

        public void expect() throws IOException {
            if (this.first || !(Emitter.this.event instanceof SequenceEndEvent)) {
                Emitter.this.writeIndent();
                Emitter.this.writeIndicator("-", true, false, true);
                Emitter.this.states.push(new ExpectBlockSequenceItem(false));
                Emitter.this.expectNode(false, false, false);
                return;
            }
            Integer unused = Emitter.this.indent = (Integer) Emitter.this.indents.pop();
            EmitterState unused2 = Emitter.this.state = (EmitterState) Emitter.this.states.pop();
        }
    }

    private void expectBlockMapping() throws IOException {
        increaseIndent(false, false);
        this.state = new ExpectFirstBlockMappingKey();
    }

    private class ExpectFirstBlockMappingKey implements EmitterState {
        private ExpectFirstBlockMappingKey() {
        }

        public void expect() throws IOException {
            new ExpectBlockMappingKey(true).expect();
        }
    }

    private class ExpectBlockMappingKey implements EmitterState {
        private boolean first;

        public ExpectBlockMappingKey(boolean first2) {
            this.first = first2;
        }

        public void expect() throws IOException {
            if (this.first || !(Emitter.this.event instanceof MappingEndEvent)) {
                Emitter.this.writeIndent();
                if (Emitter.this.checkSimpleKey()) {
                    Emitter.this.states.push(new ExpectBlockMappingSimpleValue());
                    Emitter.this.expectNode(false, true, true);
                    return;
                }
                Emitter.this.writeIndicator("?", true, false, true);
                Emitter.this.states.push(new ExpectBlockMappingValue());
                Emitter.this.expectNode(false, true, false);
                return;
            }
            Integer unused = Emitter.this.indent = (Integer) Emitter.this.indents.pop();
            EmitterState unused2 = Emitter.this.state = (EmitterState) Emitter.this.states.pop();
        }
    }

    private class ExpectBlockMappingSimpleValue implements EmitterState {
        private ExpectBlockMappingSimpleValue() {
        }

        public void expect() throws IOException {
            Emitter.this.writeIndicator(":", false, false, false);
            Emitter.this.states.push(new ExpectBlockMappingKey(false));
            Emitter.this.expectNode(false, true, false);
        }
    }

    private class ExpectBlockMappingValue implements EmitterState {
        private ExpectBlockMappingValue() {
        }

        public void expect() throws IOException {
            Emitter.this.writeIndent();
            Emitter.this.writeIndicator(":", true, false, true);
            Emitter.this.states.push(new ExpectBlockMappingKey(false));
            Emitter.this.expectNode(false, true, false);
        }
    }

    private boolean checkEmptySequence() {
        return (this.event instanceof SequenceStartEvent) && !this.events.isEmpty() && (this.events.peek() instanceof SequenceEndEvent);
    }

    private boolean checkEmptyMapping() {
        return (this.event instanceof MappingStartEvent) && !this.events.isEmpty() && (this.events.peek() instanceof MappingEndEvent);
    }

    /* access modifiers changed from: private */
    public boolean checkEmptyDocument() {
        if (!(this.event instanceof DocumentStartEvent) || this.events.isEmpty()) {
            return false;
        }
        Event event2 = this.events.peek();
        if (!(event2 instanceof ScalarEvent)) {
            return false;
        }
        ScalarEvent e = (ScalarEvent) event2;
        if (e.getAnchor() == null && e.getTag() == null && e.getImplicit() != null && e.getValue().length() == 0) {
            return true;
        }
        return false;
    }

    /* access modifiers changed from: private */
    public boolean checkSimpleKey() {
        int length = 0;
        if ((this.event instanceof NodeEvent) && ((NodeEvent) this.event).getAnchor() != null) {
            if (this.preparedAnchor == null) {
                this.preparedAnchor = prepareAnchor(((NodeEvent) this.event).getAnchor());
            }
            length = 0 + this.preparedAnchor.length();
        }
        String tag = null;
        if (this.event instanceof ScalarEvent) {
            tag = ((ScalarEvent) this.event).getTag();
        } else if (this.event instanceof CollectionStartEvent) {
            tag = ((CollectionStartEvent) this.event).getTag();
        }
        if (tag != null) {
            if (this.preparedTag == null) {
                this.preparedTag = prepareTag(tag);
            }
            length += this.preparedTag.length();
        }
        if (this.event instanceof ScalarEvent) {
            if (this.analysis == null) {
                this.analysis = analyzeScalar(((ScalarEvent) this.event).getValue());
            }
            length += this.analysis.scalar.length();
        }
        return length < 128 && ((this.event instanceof AliasEvent) || (((this.event instanceof ScalarEvent) && !this.analysis.empty && !this.analysis.multiline) || checkEmptySequence() || checkEmptyMapping()));
    }

    private void processAnchor(String indicator) throws IOException {
        NodeEvent ev = (NodeEvent) this.event;
        if (ev.getAnchor() == null) {
            this.preparedAnchor = null;
            return;
        }
        if (this.preparedAnchor == null) {
            this.preparedAnchor = prepareAnchor(ev.getAnchor());
        }
        writeIndicator(indicator + this.preparedAnchor, true, false, false);
        this.preparedAnchor = null;
    }

    private void processTag() throws IOException {
        String tag;
        if (this.event instanceof ScalarEvent) {
            ScalarEvent ev = (ScalarEvent) this.event;
            tag = ev.getTag();
            if (this.style == null) {
                this.style = chooseScalarStyle();
            }
            if ((!this.canonical.booleanValue() || tag == null) && ((this.style == null && ev.getImplicit().canOmitTagInPlainScalar()) || (this.style != null && ev.getImplicit().canOmitTagInNonPlainScalar()))) {
                this.preparedTag = null;
                return;
            } else if (ev.getImplicit().canOmitTagInPlainScalar() && tag == null) {
                tag = "!";
                this.preparedTag = null;
            }
        } else {
            CollectionStartEvent ev2 = (CollectionStartEvent) this.event;
            tag = ev2.getTag();
            if ((!this.canonical.booleanValue() || tag == null) && ev2.getImplicit()) {
                this.preparedTag = null;
                return;
            }
        }
        if (tag != null) {
            if (this.preparedTag == null) {
                this.preparedTag = prepareTag(tag);
            }
            writeIndicator(this.preparedTag, true, false, false);
            this.preparedTag = null;
            return;
        }
        throw new EmitterException("tag is not specified");
    }

    private Character chooseScalarStyle() {
        ScalarEvent ev = (ScalarEvent) this.event;
        if (this.analysis == null) {
            this.analysis = analyzeScalar(ev.getValue());
        }
        if ((ev.getStyle() != null && ev.getStyle().charValue() == '\"') || this.canonical.booleanValue()) {
            return '\"';
        }
        if (ev.getStyle() == null && ev.getImplicit().canOmitTagInPlainScalar() && (!this.simpleKeyContext || (!this.analysis.empty && !this.analysis.multiline))) {
            if (this.flowLevel != 0 && this.analysis.allowFlowPlain) {
                return null;
            }
            if (this.flowLevel == 0 && this.analysis.allowBlockPlain) {
                return null;
            }
        }
        if (ev.getStyle() != null && ((ev.getStyle().charValue() == '|' || ev.getStyle().charValue() == '>') && this.flowLevel == 0 && !this.simpleKeyContext && this.analysis.allowBlock)) {
            return ev.getStyle();
        }
        if ((ev.getStyle() == null || ev.getStyle().charValue() == '\'') && this.analysis.allowSingleQuoted && (!this.simpleKeyContext || !this.analysis.multiline)) {
            return '\'';
        }
        return '\"';
    }

    private void processScalar() throws IOException {
        ScalarEvent ev = (ScalarEvent) this.event;
        if (this.analysis == null) {
            this.analysis = analyzeScalar(ev.getValue());
        }
        if (this.style == null) {
            this.style = chooseScalarStyle();
        }
        this.style = this.options.calculateScalarStyle(this.analysis, DumperOptions.ScalarStyle.createStyle(this.style)).getChar();
        boolean split = !this.simpleKeyContext;
        if (this.style == null) {
            writePlain(this.analysis.scalar, split);
        } else {
            char charValue = this.style.charValue();
            if (charValue == '\"') {
                writeDoubleQuoted(this.analysis.scalar, split);
            } else if (charValue == '\'') {
                writeSingleQuoted(this.analysis.scalar, split);
            } else if (charValue == '>') {
                writeFolded(this.analysis.scalar);
            } else if (charValue == '|') {
                writeLiteral(this.analysis.scalar);
            } else {
                throw new YAMLException("Unexpected style: " + this.style);
            }
        }
        this.analysis = null;
        this.style = null;
    }

    /* access modifiers changed from: private */
    public String prepareVersion(Integer[] version) {
        Integer major = version[0];
        Integer minor = version[1];
        if (major.intValue() == 1) {
            return major.toString() + "." + minor.toString();
        }
        throw new EmitterException("unsupported YAML version: " + version[0] + "." + version[1]);
    }

    /* access modifiers changed from: private */
    public String prepareTagHandle(String handle) {
        if (handle.length() == 0) {
            throw new EmitterException("tag handle must not be empty");
        } else if (handle.charAt(0) != '!' || handle.charAt(handle.length() - 1) != '!') {
            throw new EmitterException("tag handle must start and end with '!': " + handle);
        } else if ("!".equals(handle) || HANDLE_FORMAT.matcher(handle).matches()) {
            return handle;
        } else {
            throw new EmitterException("invalid character in the tag handle: " + handle);
        }
    }

    /* access modifiers changed from: private */
    public String prepareTagPrefix(String prefix) {
        if (prefix.length() != 0) {
            StringBuilder chunks = new StringBuilder();
            int end = 0;
            if (prefix.charAt(0) == '!') {
                end = 1;
            }
            while (end < prefix.length()) {
                end++;
            }
            if (0 < end) {
                chunks.append(prefix.substring(0, end));
            }
            return chunks.toString();
        }
        throw new EmitterException("tag prefix must not be empty");
    }

    private String prepareTag(String tag) {
        if (tag.length() == 0) {
            throw new EmitterException("tag must not be empty");
        } else if ("!".equals(tag)) {
            return tag;
        } else {
            String handle = null;
            String suffix = tag;
            for (String prefix : this.tagPrefixes.keySet()) {
                if (tag.startsWith(prefix) && ("!".equals(prefix) || prefix.length() < tag.length())) {
                    handle = prefix;
                }
            }
            if (handle != null) {
                suffix = tag.substring(handle.length());
                handle = this.tagPrefixes.get(handle);
            }
            int end = suffix.length();
            String suffixText = end > 0 ? suffix.substring(0, end) : "";
            if (handle != null) {
                return handle + suffixText;
            }
            return "!<" + suffixText + ">";
        }
    }

    static String prepareAnchor(String anchor) {
        if (anchor.length() == 0) {
            throw new EmitterException("anchor must not be empty");
        } else if (ANCHOR_FORMAT.matcher(anchor).matches()) {
            return anchor;
        } else {
            throw new EmitterException("invalid character in the anchor: " + anchor);
        }
    }

    /* JADX WARNING: Removed duplicated region for block: B:47:0x00b5  */
    /* JADX WARNING: Removed duplicated region for block: B:50:0x00bd A[ADDED_TO_REGION] */
    /* JADX WARNING: Removed duplicated region for block: B:72:0x00f1  */
    /* JADX WARNING: Removed duplicated region for block: B:80:0x0103  */
    /* JADX WARNING: Removed duplicated region for block: B:92:0x0128 A[ADDED_TO_REGION] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private org.yaml.snakeyaml.emitter.ScalarAnalysis analyzeScalar(java.lang.String r30) {
        /*
            r29 = this;
            r8 = r30
            int r0 = r30.length()
            if (r0 != 0) goto L_0x0017
            org.yaml.snakeyaml.emitter.ScalarAnalysis r9 = new org.yaml.snakeyaml.emitter.ScalarAnalysis
            r2 = 1
            r3 = 0
            r4 = 0
            r5 = 1
            r6 = 1
            r7 = 0
            r0 = r9
            r1 = r30
            r0.<init>(r1, r2, r3, r4, r5, r6, r7)
            return r9
        L_0x0017:
            r0 = 0
            r1 = 0
            r2 = 0
            r3 = 0
            r4 = 0
            r5 = 0
            r6 = 0
            r7 = 0
            r9 = 0
            r10 = 0
            java.lang.String r11 = "---"
            boolean r11 = r8.startsWith(r11)
            if (r11 != 0) goto L_0x0031
            java.lang.String r11 = "..."
            boolean r11 = r8.startsWith(r11)
            if (r11 == 0) goto L_0x0033
        L_0x0031:
            r0 = 1
            r1 = 1
        L_0x0033:
            r11 = 1
            int r12 = r30.length()
            r13 = 0
            r14 = 1
            if (r12 == r14) goto L_0x004b
            org.yaml.snakeyaml.scanner.Constant r12 = org.yaml.snakeyaml.scanner.Constant.NULL_BL_T_LINEBR
            char r15 = r8.charAt(r14)
            boolean r12 = r12.has(r15)
            if (r12 == 0) goto L_0x0049
            goto L_0x004b
        L_0x0049:
            r12 = 0
            goto L_0x004c
        L_0x004b:
            r12 = 1
        L_0x004c:
            r15 = 0
            r16 = 0
            r22 = r2
            r21 = r3
            r17 = r7
            r18 = r9
            r20 = r10
            r19 = r11
            r23 = r15
            r24 = r16
            r11 = r1
            r9 = r4
            r10 = r5
            r15 = r6
            r16 = r12
            r12 = r0
            r0 = 0
        L_0x0067:
            r7 = r0
            int r0 = r30.length()
            if (r7 >= r0) goto L_0x0152
            char r0 = r8.charAt(r7)
            r1 = 58
            r2 = -1
            if (r7 != 0) goto L_0x0094
            java.lang.String r3 = "#,[]{}&*!|>'\"%@`"
            int r3 = r3.indexOf(r0)
            if (r3 == r2) goto L_0x0081
            r11 = 1
            r12 = 1
        L_0x0081:
            r2 = 63
            if (r0 == r2) goto L_0x0087
            if (r0 != r1) goto L_0x008b
        L_0x0087:
            r11 = 1
            if (r16 == 0) goto L_0x008b
            r12 = 1
        L_0x008b:
            r1 = 45
            if (r0 != r1) goto L_0x00ad
            if (r16 == 0) goto L_0x00ad
            r1 = 1
            r2 = 1
            goto L_0x00ab
        L_0x0094:
            java.lang.String r3 = ",?[]{}"
            int r3 = r3.indexOf(r0)
            if (r3 == r2) goto L_0x009d
            r11 = 1
        L_0x009d:
            if (r0 != r1) goto L_0x00a3
            r11 = 1
            if (r16 == 0) goto L_0x00a3
            r12 = 1
        L_0x00a3:
            r1 = 35
            if (r0 != r1) goto L_0x00ad
            if (r19 == 0) goto L_0x00ad
            r1 = 1
            r2 = 1
        L_0x00ab:
            r11 = r1
            r12 = r2
        L_0x00ad:
            org.yaml.snakeyaml.scanner.Constant r1 = org.yaml.snakeyaml.scanner.Constant.LINEBR
            boolean r1 = r1.has(r0)
            if (r1 == 0) goto L_0x00b7
            r22 = 1
        L_0x00b7:
            r2 = 10
            r3 = 32
            if (r0 == r2) goto L_0x00ed
            if (r3 > r0) goto L_0x00c3
            r2 = 126(0x7e, float:1.77E-43)
            if (r0 <= r2) goto L_0x00ed
        L_0x00c3:
            r2 = 133(0x85, float:1.86E-43)
            if (r0 == r2) goto L_0x00da
            r2 = 160(0xa0, float:2.24E-43)
            if (r2 > r0) goto L_0x00d0
            r2 = 55295(0xd7ff, float:7.7485E-41)
            if (r0 <= r2) goto L_0x00da
        L_0x00d0:
            r2 = 57344(0xe000, float:8.0356E-41)
            if (r2 > r0) goto L_0x00e7
            r2 = 65533(0xfffd, float:9.1831E-41)
            if (r0 > r2) goto L_0x00e7
        L_0x00da:
            r2 = 65279(0xfeff, float:9.1475E-41)
            if (r0 == r2) goto L_0x00e7
            r6 = r29
            boolean r2 = r6.allowUnicode
            if (r2 != 0) goto L_0x00ef
            r2 = 1
            goto L_0x00ea
        L_0x00e7:
            r6 = r29
            r2 = 1
        L_0x00ea:
            r21 = r2
            goto L_0x00ef
        L_0x00ed:
            r6 = r29
        L_0x00ef:
            if (r0 != r3) goto L_0x0103
            if (r7 != 0) goto L_0x00f4
            r9 = 1
        L_0x00f4:
            int r2 = r30.length()
            int r2 = r2 - r14
            if (r7 != r2) goto L_0x00fc
            r15 = 1
        L_0x00fc:
            if (r24 == 0) goto L_0x0100
            r18 = 1
        L_0x0100:
            r2 = 1
            r3 = 0
            goto L_0x011a
        L_0x0103:
            if (r1 == 0) goto L_0x0118
            if (r7 != 0) goto L_0x0108
            r10 = 1
        L_0x0108:
            int r2 = r30.length()
            int r2 = r2 - r14
            if (r7 != r2) goto L_0x0111
            r17 = 1
        L_0x0111:
            if (r23 == 0) goto L_0x0115
            r20 = 1
        L_0x0115:
            r2 = 0
            r3 = 1
            goto L_0x011a
        L_0x0118:
            r2 = 0
            r3 = 0
        L_0x011a:
            r23 = r2
            r24 = r3
            int r2 = r7 + 1
            org.yaml.snakeyaml.scanner.Constant r3 = org.yaml.snakeyaml.scanner.Constant.NULL_BL_T
            boolean r3 = r3.has(r0)
            if (r3 != 0) goto L_0x012d
            if (r1 == 0) goto L_0x012b
            goto L_0x012d
        L_0x012b:
            r3 = 0
            goto L_0x012e
        L_0x012d:
            r3 = 1
        L_0x012e:
            r19 = r3
            int r3 = r2 + 1
            int r4 = r30.length()
            if (r3 >= r4) goto L_0x014b
            org.yaml.snakeyaml.scanner.Constant r3 = org.yaml.snakeyaml.scanner.Constant.NULL_BL_T
            int r4 = r2 + 1
            char r4 = r8.charAt(r4)
            boolean r3 = r3.has(r4)
            if (r3 != 0) goto L_0x014b
            if (r1 == 0) goto L_0x0149
            goto L_0x014b
        L_0x0149:
            r3 = 0
            goto L_0x014c
        L_0x014b:
            r3 = 1
        L_0x014c:
            r16 = r3
            r0 = r2
            goto L_0x0067
        L_0x0152:
            r6 = r29
            r0 = 1
            r1 = 1
            r2 = 1
            r3 = 1
            if (r9 != 0) goto L_0x0160
            if (r10 != 0) goto L_0x0160
            if (r15 != 0) goto L_0x0160
            if (r17 == 0) goto L_0x0162
        L_0x0160:
            r1 = r13
            r0 = r13
        L_0x0162:
            if (r15 == 0) goto L_0x0165
            r3 = 0
        L_0x0165:
            if (r18 == 0) goto L_0x016a
            r2 = r13
            r1 = r13
            r0 = r13
        L_0x016a:
            if (r20 != 0) goto L_0x0172
            if (r21 == 0) goto L_0x016f
            goto L_0x0172
        L_0x016f:
            r13 = r2
        L_0x0170:
            r14 = r3
            goto L_0x0177
        L_0x0172:
            r3 = r13
            r2 = r13
            r1 = r13
            r0 = r13
            goto L_0x0170
        L_0x0177:
            if (r22 == 0) goto L_0x017a
            r0 = 0
        L_0x017a:
            if (r11 == 0) goto L_0x017d
            r0 = 0
        L_0x017d:
            r25 = r0
            if (r12 == 0) goto L_0x0185
            r0 = 0
            r26 = r0
            goto L_0x0187
        L_0x0185:
            r26 = r1
        L_0x0187:
            org.yaml.snakeyaml.emitter.ScalarAnalysis r27 = new org.yaml.snakeyaml.emitter.ScalarAnalysis
            r2 = 0
            r0 = r27
            r1 = r30
            r3 = r22
            r4 = r25
            r5 = r26
            r6 = r13
            r28 = r7
            r7 = r14
            r0.<init>(r1, r2, r3, r4, r5, r6, r7)
            return r27
        */
        throw new UnsupportedOperationException("Method not decompiled: org.yaml.snakeyaml.emitter.Emitter.analyzeScalar(java.lang.String):org.yaml.snakeyaml.emitter.ScalarAnalysis");
    }

    /* access modifiers changed from: package-private */
    public void flushStream() throws IOException {
        this.stream.flush();
    }

    /* access modifiers changed from: package-private */
    public void writeStreamStart() {
    }

    /* access modifiers changed from: package-private */
    public void writeStreamEnd() throws IOException {
        flushStream();
    }

    /* access modifiers changed from: package-private */
    public void writeIndicator(String indicator, boolean needWhitespace, boolean whitespace2, boolean indentation) throws IOException {
        boolean z = true;
        if (!this.whitespace && needWhitespace) {
            this.column++;
            this.stream.write(SPACE);
        }
        this.whitespace = whitespace2;
        if (!this.indention || !indentation) {
            z = false;
        }
        this.indention = z;
        this.column += indicator.length();
        this.openEnded = false;
        this.stream.write(indicator);
    }

    /* access modifiers changed from: package-private */
    public void writeIndent() throws IOException {
        int indent2;
        if (this.indent != null) {
            indent2 = this.indent.intValue();
        } else {
            indent2 = 0;
        }
        if (!this.indention || this.column > indent2 || (this.column == indent2 && !this.whitespace)) {
            writeLineBreak((String) null);
        }
        if (this.column < indent2) {
            this.whitespace = true;
            char[] data = new char[(indent2 - this.column)];
            for (int i = 0; i < data.length; i++) {
                data[i] = ' ';
            }
            this.column = indent2;
            this.stream.write(data);
        }
    }

    private void writeLineBreak(String data) throws IOException {
        this.whitespace = true;
        this.indention = true;
        this.column = 0;
        if (data == null) {
            this.stream.write(this.bestLineBreak);
        } else {
            this.stream.write(data);
        }
    }

    /* access modifiers changed from: package-private */
    public void writeVersionDirective(String versionText) throws IOException {
        this.stream.write("%YAML ");
        this.stream.write(versionText);
        writeLineBreak((String) null);
    }

    /* access modifiers changed from: package-private */
    public void writeTagDirective(String handleText, String prefixText) throws IOException {
        this.stream.write("%TAG ");
        this.stream.write(handleText);
        this.stream.write(SPACE);
        this.stream.write(prefixText);
        writeLineBreak((String) null);
    }

    private void writeSingleQuoted(String text, boolean split) throws IOException {
        String str = text;
        writeIndicator("'", true, false, false);
        int start = 0;
        boolean breaks = false;
        boolean spaces = false;
        for (int end = 0; end <= text.length(); end++) {
            char ch = 0;
            if (end < text.length()) {
                ch = str.charAt(end);
            }
            if (spaces) {
                if (ch == 0 || ch != ' ') {
                    if (start + 1 != end || this.column <= this.bestWidth || !split || start == 0 || end == text.length()) {
                        int len = end - start;
                        this.column += len;
                        this.stream.write(str, start, len);
                    } else {
                        writeIndent();
                    }
                    start = end;
                }
            } else if (breaks) {
                if (ch == 0 || Constant.LINEBR.hasNo(ch)) {
                    String str2 = null;
                    if (str.charAt(start) == 10) {
                        writeLineBreak((String) null);
                    }
                    char[] arr$ = str.substring(start, end).toCharArray();
                    int len$ = arr$.length;
                    int i$ = 0;
                    while (true) {
                        int i$2 = i$;
                        if (i$2 >= len$) {
                            break;
                        }
                        char br = arr$[i$2];
                        if (br == 10) {
                            writeLineBreak(str2);
                        } else {
                            writeLineBreak(String.valueOf(br));
                        }
                        i$ = i$2 + 1;
                        str2 = null;
                    }
                    writeIndent();
                    start = end;
                }
            } else if (Constant.LINEBR.has(ch, "\u0000 '") && start < end) {
                int len2 = end - start;
                this.column += len2;
                this.stream.write(str, start, len2);
                start = end;
            }
            if (ch == '\'') {
                this.column += 2;
                this.stream.write("''");
                start = end + 1;
            }
            if (ch != 0) {
                spaces = ch == ' ';
                breaks = Constant.LINEBR.has(ch);
            }
        }
        writeIndicator("'", false, false, false);
    }

    private void writeDoubleQuoted(String text, boolean split) throws IOException {
        String data;
        String data2;
        writeIndicator("\"", true, false, false);
        int start = 0;
        for (int end = 0; end <= text.length(); end++) {
            Character ch = null;
            if (end < text.length()) {
                ch = Character.valueOf(text.charAt(end));
            }
            if (ch == null || "\"\\  ﻿".indexOf(ch.charValue()) != -1 || ' ' > ch.charValue() || ch.charValue() > '~') {
                if (start < end) {
                    int len = end - start;
                    this.column += len;
                    this.stream.write(text, start, len);
                    start = end;
                }
                if (ch != null) {
                    if (ESCAPE_REPLACEMENTS.containsKey(ch)) {
                        data2 = "\\" + ESCAPE_REPLACEMENTS.get(ch);
                    } else if (this.allowUnicode) {
                        data2 = String.valueOf(ch);
                    } else if (ch.charValue() <= 255) {
                        String s = "0" + Integer.toString(ch.charValue(), 16);
                        data2 = "\\x" + s.substring(s.length() - 2);
                    } else {
                        String s2 = "000" + Integer.toString(ch.charValue(), 16);
                        data2 = "\\u" + s2.substring(s2.length() - 4);
                    }
                    this.column += data2.length();
                    this.stream.write(data2);
                    start = end + 1;
                }
            }
            if (end > 0 && end < text.length() - 1 && ((ch.charValue() == ' ' || start >= end) && this.column + (end - start) > this.bestWidth && split)) {
                if (start >= end) {
                    data = "\\";
                } else {
                    data = text.substring(start, end) + "\\";
                }
                if (start < end) {
                    start = end;
                }
                this.column += data.length();
                this.stream.write(data);
                writeIndent();
                this.whitespace = false;
                this.indention = false;
                if (text.charAt(start) == ' ') {
                    this.column += "\\".length();
                    this.stream.write("\\");
                }
            }
        }
        writeIndicator("\"", false, false, false);
    }

    private String determineBlockHints(String text) {
        StringBuilder hints = new StringBuilder();
        if (Constant.LINEBR.has(text.charAt(0), " ")) {
            hints.append(this.bestIndent);
        }
        if (Constant.LINEBR.hasNo(text.charAt(text.length() - 1))) {
            hints.append("-");
        } else if (text.length() == 1 || Constant.LINEBR.has(text.charAt(text.length() - 2))) {
            hints.append("+");
        }
        return hints.toString();
    }

    /* access modifiers changed from: package-private */
    /* JADX WARNING: Removed duplicated region for block: B:55:0x00e6  */
    /* JADX WARNING: Removed duplicated region for block: B:64:0x00f5 A[SYNTHETIC] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public void writeFolded(java.lang.String r19) throws java.io.IOException {
        /*
            r18 = this;
            r0 = r18
            r1 = r19
            java.lang.String r2 = r18.determineBlockHints(r19)
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            java.lang.String r4 = ">"
            r3.append(r4)
            r3.append(r2)
            java.lang.String r3 = r3.toString()
            r4 = 0
            r5 = 1
            r0.writeIndicator(r3, r5, r4, r4)
            int r3 = r2.length()
            if (r3 <= 0) goto L_0x0033
            int r3 = r2.length()
            int r3 = r3 - r5
            char r3 = r2.charAt(r3)
            r6 = 43
            if (r3 != r6) goto L_0x0033
            r0.openEnded = r5
        L_0x0033:
            r3 = 0
            r0.writeLineBreak(r3)
            r6 = 1
            r7 = 0
            r8 = 1
            r9 = 0
            r10 = r9
            r9 = r8
            r8 = r7
            r7 = r6
            r6 = 0
        L_0x0040:
            int r11 = r19.length()
            if (r6 > r11) goto L_0x00fc
            r11 = 0
            int r12 = r19.length()
            if (r6 >= r12) goto L_0x0051
            char r11 = r1.charAt(r6)
        L_0x0051:
            r12 = 32
            if (r9 == 0) goto L_0x00a5
            if (r11 == 0) goto L_0x005f
            org.yaml.snakeyaml.scanner.Constant r13 = org.yaml.snakeyaml.scanner.Constant.LINEBR
            boolean r13 = r13.hasNo(r11)
            if (r13 == 0) goto L_0x00e4
        L_0x005f:
            r13 = 10
            if (r7 != 0) goto L_0x0070
            if (r11 == 0) goto L_0x0070
            if (r11 == r12) goto L_0x0070
            char r14 = r1.charAt(r10)
            if (r14 != r13) goto L_0x0070
            r0.writeLineBreak(r3)
        L_0x0070:
            if (r11 != r12) goto L_0x0074
            r14 = 1
            goto L_0x0075
        L_0x0074:
            r14 = 0
        L_0x0075:
            r7 = r14
            java.lang.String r14 = r1.substring(r10, r6)
            char[] r15 = r14.toCharArray()
            int r4 = r15.length
            r16 = 0
        L_0x0081:
            r17 = r16
            r5 = r17
            if (r5 >= r4) goto L_0x009e
            char r12 = r15[r5]
            if (r12 != r13) goto L_0x008f
            r0.writeLineBreak(r3)
            goto L_0x0096
        L_0x008f:
            java.lang.String r13 = java.lang.String.valueOf(r12)
            r0.writeLineBreak(r13)
        L_0x0096:
            int r16 = r5 + 1
            r5 = 1
            r12 = 32
            r13 = 10
            goto L_0x0081
        L_0x009e:
            if (r11 == 0) goto L_0x00a3
            r18.writeIndent()
        L_0x00a3:
            r4 = r6
            goto L_0x00e3
        L_0x00a5:
            if (r8 == 0) goto L_0x00c7
            r4 = 32
            if (r11 == r4) goto L_0x00e4
            int r4 = r10 + 1
            if (r4 != r6) goto L_0x00b9
            int r4 = r0.column
            int r5 = r0.bestWidth
            if (r4 <= r5) goto L_0x00b9
            r18.writeIndent()
            goto L_0x00c5
        L_0x00b9:
            int r4 = r6 - r10
            int r5 = r0.column
            int r5 = r5 + r4
            r0.column = r5
            java.io.Writer r5 = r0.stream
            r5.write(r1, r10, r4)
        L_0x00c5:
            r4 = r6
            goto L_0x00e3
        L_0x00c7:
            org.yaml.snakeyaml.scanner.Constant r4 = org.yaml.snakeyaml.scanner.Constant.LINEBR
            java.lang.String r5 = "\u0000 "
            boolean r4 = r4.has(r11, r5)
            if (r4 == 0) goto L_0x00e4
            int r4 = r6 - r10
            int r5 = r0.column
            int r5 = r5 + r4
            r0.column = r5
            java.io.Writer r5 = r0.stream
            r5.write(r1, r10, r4)
            if (r11 != 0) goto L_0x00e2
            r0.writeLineBreak(r3)
        L_0x00e2:
            r4 = r6
        L_0x00e3:
            r10 = r4
        L_0x00e4:
            if (r11 == 0) goto L_0x00f5
            org.yaml.snakeyaml.scanner.Constant r4 = org.yaml.snakeyaml.scanner.Constant.LINEBR
            boolean r4 = r4.has(r11)
            r5 = 32
            if (r11 != r5) goto L_0x00f2
            r5 = 1
            goto L_0x00f3
        L_0x00f2:
            r5 = 0
        L_0x00f3:
            r9 = r4
            r8 = r5
        L_0x00f5:
            int r6 = r6 + 1
            r4 = 0
            r5 = 1
            goto L_0x0040
        L_0x00fc:
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.yaml.snakeyaml.emitter.Emitter.writeFolded(java.lang.String):void");
    }

    /* access modifiers changed from: package-private */
    public void writeLiteral(String text) throws IOException {
        String hints = determineBlockHints(text);
        writeIndicator("|" + hints, true, false, false);
        if (hints.length() > 0 && hints.charAt(hints.length() - 1) == '+') {
            this.openEnded = true;
        }
        writeLineBreak((String) null);
        int start = 0;
        boolean breaks = true;
        for (int end = 0; end <= text.length(); end++) {
            char ch = 0;
            if (end < text.length()) {
                ch = text.charAt(end);
            }
            if (breaks) {
                if (ch == 0 || Constant.LINEBR.hasNo(ch)) {
                    for (char br : text.substring(start, end).toCharArray()) {
                        if (br == 10) {
                            writeLineBreak((String) null);
                        } else {
                            writeLineBreak(String.valueOf(br));
                        }
                    }
                    if (ch != 0) {
                        writeIndent();
                    }
                    start = end;
                }
            } else if (ch == 0 || Constant.LINEBR.has(ch)) {
                this.stream.write(text, start, end - start);
                if (ch == 0) {
                    writeLineBreak((String) null);
                }
                start = end;
            }
            if (ch != 0) {
                breaks = Constant.LINEBR.has(ch);
            }
        }
    }

    /* access modifiers changed from: package-private */
    public void writePlain(String text, boolean split) throws IOException {
        int len;
        String str = text;
        if (this.rootContext) {
            this.openEnded = true;
        }
        if (text.length() != 0) {
            if (!this.whitespace) {
                this.column++;
                this.stream.write(SPACE);
            }
            this.whitespace = false;
            this.indention = false;
            int start = 0;
            boolean breaks = false;
            boolean spaces = false;
            for (int end = 0; end <= text.length(); end++) {
                char ch = 0;
                if (end < text.length()) {
                    ch = str.charAt(end);
                }
                if (!spaces) {
                    if (breaks) {
                        if (Constant.LINEBR.hasNo(ch)) {
                            String str2 = null;
                            if (str.charAt(start) == 10) {
                                writeLineBreak((String) null);
                            }
                            char[] arr$ = str.substring(start, end).toCharArray();
                            int len$ = arr$.length;
                            int i$ = 0;
                            while (true) {
                                int i$2 = i$;
                                if (i$2 >= len$) {
                                    break;
                                }
                                char br = arr$[i$2];
                                if (br == 10) {
                                    writeLineBreak(str2);
                                } else {
                                    writeLineBreak(String.valueOf(br));
                                }
                                i$ = i$2 + 1;
                                str2 = null;
                            }
                            writeIndent();
                            this.whitespace = false;
                            this.indention = false;
                            len = end;
                        }
                    } else if (ch == 0 || Constant.LINEBR.has(ch)) {
                        int len2 = end - start;
                        this.column += len2;
                        this.stream.write(str, start, len2);
                        len = end;
                    }
                    start = len;
                } else if (ch != ' ') {
                    if (start + 1 != end || this.column <= this.bestWidth || !split) {
                        int len3 = end - start;
                        this.column += len3;
                        this.stream.write(str, start, len3);
                    } else {
                        writeIndent();
                        this.whitespace = false;
                        this.indention = false;
                    }
                    start = end;
                }
                if (ch != 0) {
                    boolean spaces2 = ch == ' ';
                    breaks = Constant.LINEBR.has(ch);
                    spaces = spaces2;
                }
            }
        }
    }
}
