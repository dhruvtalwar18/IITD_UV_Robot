package org.yaml.snakeyaml.scanner;

import java.nio.ByteBuffer;
import java.nio.charset.CharacterCodingException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Pattern;
import org.apache.commons.io.IOUtils;
import org.yaml.snakeyaml.error.Mark;
import org.yaml.snakeyaml.error.YAMLException;
import org.yaml.snakeyaml.reader.StreamReader;
import org.yaml.snakeyaml.tokens.AliasToken;
import org.yaml.snakeyaml.tokens.AnchorToken;
import org.yaml.snakeyaml.tokens.BlockEndToken;
import org.yaml.snakeyaml.tokens.BlockEntryToken;
import org.yaml.snakeyaml.tokens.BlockMappingStartToken;
import org.yaml.snakeyaml.tokens.BlockSequenceStartToken;
import org.yaml.snakeyaml.tokens.DirectiveToken;
import org.yaml.snakeyaml.tokens.DocumentEndToken;
import org.yaml.snakeyaml.tokens.DocumentStartToken;
import org.yaml.snakeyaml.tokens.FlowEntryToken;
import org.yaml.snakeyaml.tokens.FlowMappingEndToken;
import org.yaml.snakeyaml.tokens.FlowMappingStartToken;
import org.yaml.snakeyaml.tokens.FlowSequenceEndToken;
import org.yaml.snakeyaml.tokens.FlowSequenceStartToken;
import org.yaml.snakeyaml.tokens.KeyToken;
import org.yaml.snakeyaml.tokens.ScalarToken;
import org.yaml.snakeyaml.tokens.StreamEndToken;
import org.yaml.snakeyaml.tokens.StreamStartToken;
import org.yaml.snakeyaml.tokens.TagToken;
import org.yaml.snakeyaml.tokens.TagTuple;
import org.yaml.snakeyaml.tokens.Token;
import org.yaml.snakeyaml.tokens.ValueToken;
import org.yaml.snakeyaml.util.ArrayStack;
import org.yaml.snakeyaml.util.UriEncoder;

public final class ScannerImpl implements Scanner {
    public static final Map<Character, Integer> ESCAPE_CODES = new HashMap();
    public static final Map<Character, String> ESCAPE_REPLACEMENTS = new HashMap();
    private static final Pattern NOT_HEXA = Pattern.compile("[^0-9A-Fa-f]");
    private boolean allowSimpleKey = true;
    private boolean done = false;
    private int flowLevel = 0;
    private int indent = -1;
    private ArrayStack<Integer> indents;
    private Map<Integer, SimpleKey> possibleSimpleKeys;
    private final StreamReader reader;
    private List<Token> tokens;
    private int tokensTaken = 0;

    static {
        ESCAPE_REPLACEMENTS.put('0', "\u0000");
        ESCAPE_REPLACEMENTS.put('a', "\u0007");
        ESCAPE_REPLACEMENTS.put('b', "\b");
        ESCAPE_REPLACEMENTS.put('t', "\t");
        ESCAPE_REPLACEMENTS.put('n', "\n");
        ESCAPE_REPLACEMENTS.put('v', "\u000b");
        ESCAPE_REPLACEMENTS.put('f', "\f");
        ESCAPE_REPLACEMENTS.put('r', "\r");
        ESCAPE_REPLACEMENTS.put('e', "\u001b");
        ESCAPE_REPLACEMENTS.put(' ', " ");
        ESCAPE_REPLACEMENTS.put('\"', "\"");
        ESCAPE_REPLACEMENTS.put(Character.valueOf(IOUtils.DIR_SEPARATOR_WINDOWS), "\\");
        ESCAPE_REPLACEMENTS.put('N', "");
        ESCAPE_REPLACEMENTS.put('_', " ");
        ESCAPE_REPLACEMENTS.put('L', " ");
        ESCAPE_REPLACEMENTS.put('P', " ");
        ESCAPE_CODES.put('x', 2);
        ESCAPE_CODES.put('u', 4);
        ESCAPE_CODES.put('U', 8);
    }

    public ScannerImpl(StreamReader reader2) {
        this.reader = reader2;
        this.tokens = new ArrayList(100);
        this.indents = new ArrayStack<>(10);
        this.possibleSimpleKeys = new LinkedHashMap();
        fetchStreamStart();
    }

    public boolean checkToken(Token.ID... choices) {
        while (needMoreTokens()) {
            fetchMoreTokens();
        }
        if (!this.tokens.isEmpty()) {
            if (choices.length == 0) {
                return true;
            }
            Token.ID first = this.tokens.get(0).getTokenId();
            for (Token.ID id : choices) {
                if (first == id) {
                    return true;
                }
            }
        }
        return false;
    }

    public Token peekToken() {
        while (needMoreTokens()) {
            fetchMoreTokens();
        }
        return this.tokens.get(0);
    }

    public Token getToken() {
        if (this.tokens.isEmpty()) {
            return null;
        }
        this.tokensTaken++;
        return this.tokens.remove(0);
    }

    private boolean needMoreTokens() {
        if (this.done) {
            return false;
        }
        if (this.tokens.isEmpty()) {
            return true;
        }
        stalePossibleSimpleKeys();
        if (nextPossibleSimpleKey() == this.tokensTaken) {
            return true;
        }
        return false;
    }

    private void fetchMoreTokens() {
        scanToNextToken();
        stalePossibleSimpleKeys();
        unwindIndent(this.reader.getColumn());
        char ch = this.reader.peek();
        switch (ch) {
            case 0:
                fetchStreamEnd();
                return;
            case '!':
                fetchTag();
                return;
            case '\"':
                fetchDouble();
                return;
            case '%':
                if (checkDirective()) {
                    fetchDirective();
                    return;
                }
                break;
            case '&':
                fetchAnchor();
                return;
            case '\'':
                fetchSingle();
                return;
            case '*':
                fetchAlias();
                return;
            case ',':
                fetchFlowEntry();
                return;
            case '-':
                if (checkDocumentStart()) {
                    fetchDocumentStart();
                    return;
                } else if (checkBlockEntry()) {
                    fetchBlockEntry();
                    return;
                }
                break;
            case '.':
                if (checkDocumentEnd()) {
                    fetchDocumentEnd();
                    return;
                }
                break;
            case ':':
                if (checkValue()) {
                    fetchValue();
                    return;
                }
                break;
            case '>':
                if (this.flowLevel == 0) {
                    fetchFolded();
                    return;
                }
                break;
            case '?':
                if (checkKey()) {
                    fetchKey();
                    return;
                }
                break;
            case '[':
                fetchFlowSequenceStart();
                return;
            case ']':
                fetchFlowSequenceEnd();
                return;
            case '{':
                fetchFlowMappingStart();
                return;
            case '|':
                if (this.flowLevel == 0) {
                    fetchLiteral();
                    return;
                }
                break;
            case '}':
                fetchFlowMappingEnd();
                return;
        }
        if (checkPlain()) {
            fetchPlain();
            return;
        }
        String chRepresentation = String.valueOf(ch);
        Iterator i$ = ESCAPE_REPLACEMENTS.keySet().iterator();
        while (true) {
            if (i$.hasNext()) {
                Character s = i$.next();
                if (ESCAPE_REPLACEMENTS.get(s).equals(chRepresentation)) {
                    chRepresentation = "\\" + s;
                }
            }
        }
        throw new ScannerException("while scanning for the next token", (Mark) null, "found character " + ch + "'" + chRepresentation + "' that cannot start any token", this.reader.getMark());
    }

    private int nextPossibleSimpleKey() {
        if (!this.possibleSimpleKeys.isEmpty()) {
            return this.possibleSimpleKeys.values().iterator().next().getTokenNumber();
        }
        return -1;
    }

    private void stalePossibleSimpleKeys() {
        if (!this.possibleSimpleKeys.isEmpty()) {
            Iterator<SimpleKey> iterator = this.possibleSimpleKeys.values().iterator();
            while (iterator.hasNext()) {
                SimpleKey key = iterator.next();
                if (key.getLine() != this.reader.getLine() || this.reader.getIndex() - key.getIndex() > 1024) {
                    if (!key.isRequired()) {
                        iterator.remove();
                    } else {
                        throw new ScannerException("while scanning a simple key", key.getMark(), "could not found expected ':'", this.reader.getMark());
                    }
                }
            }
        }
    }

    private void savePossibleSimpleKey() {
        boolean required = this.flowLevel == 0 && this.indent == this.reader.getColumn();
        if (!this.allowSimpleKey && required) {
            throw new YAMLException("A simple key is required only if it is the first token in the current line");
        } else if (this.allowSimpleKey) {
            removePossibleSimpleKey();
            this.possibleSimpleKeys.put(Integer.valueOf(this.flowLevel), new SimpleKey(this.tokensTaken + this.tokens.size(), required, this.reader.getIndex(), this.reader.getLine(), this.reader.getColumn(), this.reader.getMark()));
        }
    }

    private void removePossibleSimpleKey() {
        SimpleKey key = this.possibleSimpleKeys.remove(Integer.valueOf(this.flowLevel));
        if (key != null && key.isRequired()) {
            throw new ScannerException("while scanning a simple key", key.getMark(), "could not found expected ':'", this.reader.getMark());
        }
    }

    private void unwindIndent(int col) {
        if (this.flowLevel == 0) {
            while (this.indent > col) {
                Mark mark = this.reader.getMark();
                this.indent = this.indents.pop().intValue();
                this.tokens.add(new BlockEndToken(mark, mark));
            }
        }
    }

    private boolean addIndent(int column) {
        if (this.indent >= column) {
            return false;
        }
        this.indents.push(Integer.valueOf(this.indent));
        this.indent = column;
        return true;
    }

    private void fetchStreamStart() {
        Mark mark = this.reader.getMark();
        this.tokens.add(new StreamStartToken(mark, mark));
    }

    private void fetchStreamEnd() {
        unwindIndent(-1);
        removePossibleSimpleKey();
        this.allowSimpleKey = false;
        this.possibleSimpleKeys.clear();
        Mark mark = this.reader.getMark();
        this.tokens.add(new StreamEndToken(mark, mark));
        this.done = true;
    }

    private void fetchDirective() {
        unwindIndent(-1);
        removePossibleSimpleKey();
        this.allowSimpleKey = false;
        this.tokens.add(scanDirective());
    }

    private void fetchDocumentStart() {
        fetchDocumentIndicator(true);
    }

    private void fetchDocumentEnd() {
        fetchDocumentIndicator(false);
    }

    private void fetchDocumentIndicator(boolean isDocumentStart) {
        Token token;
        unwindIndent(-1);
        removePossibleSimpleKey();
        this.allowSimpleKey = false;
        Mark startMark = this.reader.getMark();
        this.reader.forward(3);
        Mark endMark = this.reader.getMark();
        if (isDocumentStart) {
            token = new DocumentStartToken(startMark, endMark);
        } else {
            token = new DocumentEndToken(startMark, endMark);
        }
        this.tokens.add(token);
    }

    private void fetchFlowSequenceStart() {
        fetchFlowCollectionStart(false);
    }

    private void fetchFlowMappingStart() {
        fetchFlowCollectionStart(true);
    }

    private void fetchFlowCollectionStart(boolean isMappingStart) {
        Token token;
        savePossibleSimpleKey();
        this.flowLevel++;
        this.allowSimpleKey = true;
        Mark startMark = this.reader.getMark();
        this.reader.forward(1);
        Mark endMark = this.reader.getMark();
        if (isMappingStart) {
            token = new FlowMappingStartToken(startMark, endMark);
        } else {
            token = new FlowSequenceStartToken(startMark, endMark);
        }
        this.tokens.add(token);
    }

    private void fetchFlowSequenceEnd() {
        fetchFlowCollectionEnd(false);
    }

    private void fetchFlowMappingEnd() {
        fetchFlowCollectionEnd(true);
    }

    private void fetchFlowCollectionEnd(boolean isMappingEnd) {
        Token token;
        removePossibleSimpleKey();
        this.flowLevel--;
        this.allowSimpleKey = false;
        Mark startMark = this.reader.getMark();
        this.reader.forward();
        Mark endMark = this.reader.getMark();
        if (isMappingEnd) {
            token = new FlowMappingEndToken(startMark, endMark);
        } else {
            token = new FlowSequenceEndToken(startMark, endMark);
        }
        this.tokens.add(token);
    }

    private void fetchFlowEntry() {
        this.allowSimpleKey = true;
        removePossibleSimpleKey();
        Mark startMark = this.reader.getMark();
        this.reader.forward();
        this.tokens.add(new FlowEntryToken(startMark, this.reader.getMark()));
    }

    private void fetchBlockEntry() {
        if (this.flowLevel == 0) {
            if (!this.allowSimpleKey) {
                throw new ScannerException((String) null, (Mark) null, "sequence entries are not allowed here", this.reader.getMark());
            } else if (addIndent(this.reader.getColumn())) {
                Mark mark = this.reader.getMark();
                this.tokens.add(new BlockSequenceStartToken(mark, mark));
            }
        }
        this.allowSimpleKey = true;
        removePossibleSimpleKey();
        Mark startMark = this.reader.getMark();
        this.reader.forward();
        this.tokens.add(new BlockEntryToken(startMark, this.reader.getMark()));
    }

    private void fetchKey() {
        if (this.flowLevel == 0) {
            if (!this.allowSimpleKey) {
                throw new ScannerException((String) null, (Mark) null, "mapping keys are not allowed here", this.reader.getMark());
            } else if (addIndent(this.reader.getColumn())) {
                Mark mark = this.reader.getMark();
                this.tokens.add(new BlockMappingStartToken(mark, mark));
            }
        }
        this.allowSimpleKey = this.flowLevel == 0;
        removePossibleSimpleKey();
        Mark startMark = this.reader.getMark();
        this.reader.forward();
        this.tokens.add(new KeyToken(startMark, this.reader.getMark()));
    }

    private void fetchValue() {
        SimpleKey key = this.possibleSimpleKeys.remove(Integer.valueOf(this.flowLevel));
        boolean z = false;
        if (key != null) {
            this.tokens.add(key.getTokenNumber() - this.tokensTaken, new KeyToken(key.getMark(), key.getMark()));
            if (this.flowLevel == 0 && addIndent(key.getColumn())) {
                this.tokens.add(key.getTokenNumber() - this.tokensTaken, new BlockMappingStartToken(key.getMark(), key.getMark()));
            }
            this.allowSimpleKey = false;
        } else if (this.flowLevel != 0 || this.allowSimpleKey) {
            if (this.flowLevel == 0 && addIndent(this.reader.getColumn())) {
                Mark mark = this.reader.getMark();
                this.tokens.add(new BlockMappingStartToken(mark, mark));
            }
            if (this.flowLevel == 0) {
                z = true;
            }
            this.allowSimpleKey = z;
            removePossibleSimpleKey();
        } else {
            throw new ScannerException((String) null, (Mark) null, "mapping values are not allowed here", this.reader.getMark());
        }
        Mark startMark = this.reader.getMark();
        this.reader.forward();
        this.tokens.add(new ValueToken(startMark, this.reader.getMark()));
    }

    private void fetchAlias() {
        savePossibleSimpleKey();
        this.allowSimpleKey = false;
        this.tokens.add(scanAnchor(false));
    }

    private void fetchAnchor() {
        savePossibleSimpleKey();
        this.allowSimpleKey = false;
        this.tokens.add(scanAnchor(true));
    }

    private void fetchTag() {
        savePossibleSimpleKey();
        this.allowSimpleKey = false;
        this.tokens.add(scanTag());
    }

    private void fetchLiteral() {
        fetchBlockScalar('|');
    }

    private void fetchFolded() {
        fetchBlockScalar('>');
    }

    private void fetchBlockScalar(char style) {
        this.allowSimpleKey = true;
        removePossibleSimpleKey();
        this.tokens.add(scanBlockScalar(style));
    }

    private void fetchSingle() {
        fetchFlowScalar('\'');
    }

    private void fetchDouble() {
        fetchFlowScalar('\"');
    }

    private void fetchFlowScalar(char style) {
        savePossibleSimpleKey();
        this.allowSimpleKey = false;
        this.tokens.add(scanFlowScalar(style));
    }

    private void fetchPlain() {
        savePossibleSimpleKey();
        this.allowSimpleKey = false;
        this.tokens.add(scanPlain());
    }

    private boolean checkDirective() {
        return this.reader.getColumn() == 0;
    }

    private boolean checkDocumentStart() {
        if (this.reader.getColumn() != 0 || !"---".equals(this.reader.prefix(3)) || !Constant.NULL_BL_T_LINEBR.has(this.reader.peek(3))) {
            return false;
        }
        return true;
    }

    private boolean checkDocumentEnd() {
        if (this.reader.getColumn() != 0 || !"...".equals(this.reader.prefix(3)) || !Constant.NULL_BL_T_LINEBR.has(this.reader.peek(3))) {
            return false;
        }
        return true;
    }

    private boolean checkBlockEntry() {
        return Constant.NULL_BL_T_LINEBR.has(this.reader.peek(1));
    }

    private boolean checkKey() {
        if (this.flowLevel != 0) {
            return true;
        }
        return Constant.NULL_BL_T_LINEBR.has(this.reader.peek(1));
    }

    private boolean checkValue() {
        if (this.flowLevel != 0) {
            return true;
        }
        return Constant.NULL_BL_T_LINEBR.has(this.reader.peek(1));
    }

    private boolean checkPlain() {
        char ch = this.reader.peek();
        if (Constant.NULL_BL_T_LINEBR.hasNo(ch, "-?:,[]{}#&*!|>'\"%@`")) {
            return true;
        }
        if (Constant.NULL_BL_T_LINEBR.hasNo(this.reader.peek(1))) {
            if (ch != '-') {
                return this.flowLevel == 0 && "?:".indexOf(ch) != -1;
            }
            return true;
        }
    }

    private void scanToNextToken() {
        if (this.reader.getIndex() == 0 && this.reader.peek() == 65279) {
            this.reader.forward();
        }
        boolean found = false;
        while (!found) {
            int ff = 0;
            while (this.reader.peek(ff) == ' ') {
                ff++;
            }
            if (ff > 0) {
                this.reader.forward(ff);
            }
            if (this.reader.peek() == '#') {
                int ff2 = 0;
                while (Constant.NULL_OR_LINEBR.hasNo(this.reader.peek(ff2))) {
                    ff2++;
                }
                if (ff2 > 0) {
                    this.reader.forward(ff2);
                }
            }
            if (scanLineBreak().length() == 0) {
                found = true;
            } else if (this.flowLevel == 0) {
                this.allowSimpleKey = true;
            }
        }
    }

    private Token scanDirective() {
        Mark endMark;
        Mark startMark = this.reader.getMark();
        this.reader.forward();
        String name = scanDirectiveName(startMark);
        List list = null;
        if ("YAML".equals(name)) {
            list = scanYamlDirectiveValue(startMark);
            endMark = this.reader.getMark();
        } else if ("TAG".equals(name)) {
            list = scanTagDirectiveValue(startMark);
            endMark = this.reader.getMark();
        } else {
            endMark = this.reader.getMark();
            int ff = 0;
            while (Constant.NULL_OR_LINEBR.hasNo(this.reader.peek(ff))) {
                ff++;
            }
            if (ff > 0) {
                this.reader.forward(ff);
            }
        }
        scanDirectiveIgnoredLine(startMark);
        return new DirectiveToken(name, list, startMark, endMark);
    }

    private String scanDirectiveName(Mark startMark) {
        int length = 0;
        char ch = this.reader.peek(0);
        while (Constant.ALPHA.has(ch)) {
            length++;
            ch = this.reader.peek(length);
        }
        if (length != 0) {
            String value = this.reader.prefixForward(length);
            char ch2 = this.reader.peek();
            if (!Constant.NULL_BL_LINEBR.hasNo(ch2)) {
                return value;
            }
            throw new ScannerException("while scanning a directive", startMark, "expected alphabetic or numeric character, but found " + ch2 + "(" + ch2 + ")", this.reader.getMark());
        }
        throw new ScannerException("while scanning a directive", startMark, "expected alphabetic or numeric character, but found " + ch + "(" + ch + ")", this.reader.getMark());
    }

    private List<Integer> scanYamlDirectiveValue(Mark startMark) {
        while (this.reader.peek() == ' ') {
            this.reader.forward();
        }
        Integer major = scanYamlDirectiveNumber(startMark);
        if (this.reader.peek() == '.') {
            this.reader.forward();
            Integer minor = scanYamlDirectiveNumber(startMark);
            if (!Constant.NULL_BL_LINEBR.hasNo(this.reader.peek())) {
                List<Integer> result = new ArrayList<>(2);
                result.add(major);
                result.add(minor);
                return result;
            }
            throw new ScannerException("while scanning a directive", startMark, "expected a digit or ' ', but found " + this.reader.peek() + "(" + this.reader.peek() + ")", this.reader.getMark());
        }
        throw new ScannerException("while scanning a directive", startMark, "expected a digit or '.', but found " + this.reader.peek() + "(" + this.reader.peek() + ")", this.reader.getMark());
    }

    private Integer scanYamlDirectiveNumber(Mark startMark) {
        char ch = this.reader.peek();
        if (Character.isDigit(ch)) {
            int length = 0;
            while (Character.isDigit(this.reader.peek(length))) {
                length++;
            }
            return Integer.valueOf(Integer.parseInt(this.reader.prefixForward(length)));
        }
        throw new ScannerException("while scanning a directive", startMark, "expected a digit, but found " + ch + "(" + ch + ")", this.reader.getMark());
    }

    private List<String> scanTagDirectiveValue(Mark startMark) {
        while (this.reader.peek() == ' ') {
            this.reader.forward();
        }
        String handle = scanTagDirectiveHandle(startMark);
        while (this.reader.peek() == ' ') {
            this.reader.forward();
        }
        String prefix = scanTagDirectivePrefix(startMark);
        List<String> result = new ArrayList<>(2);
        result.add(handle);
        result.add(prefix);
        return result;
    }

    private String scanTagDirectiveHandle(Mark startMark) {
        String value = scanTagHandle("directive", startMark);
        char ch = this.reader.peek();
        if (ch == ' ') {
            return value;
        }
        throw new ScannerException("while scanning a directive", startMark, "expected ' ', but found " + this.reader.peek() + "(" + ch + ")", this.reader.getMark());
    }

    private String scanTagDirectivePrefix(Mark startMark) {
        String value = scanTagUri("directive", startMark);
        if (!Constant.NULL_BL_LINEBR.hasNo(this.reader.peek())) {
            return value;
        }
        throw new ScannerException("while scanning a directive", startMark, "expected ' ', but found " + this.reader.peek() + "(" + this.reader.peek() + ")", this.reader.getMark());
    }

    private String scanDirectiveIgnoredLine(Mark startMark) {
        int ff = 0;
        while (this.reader.peek(ff) == ' ') {
            ff++;
        }
        if (ff > 0) {
            this.reader.forward(ff);
        }
        if (this.reader.peek() == '#') {
            int ff2 = 0;
            while (Constant.NULL_OR_LINEBR.hasNo(this.reader.peek(ff2))) {
                ff2++;
            }
            this.reader.forward(ff2);
        }
        char ch = this.reader.peek();
        String lineBreak = scanLineBreak();
        if (lineBreak.length() != 0 || ch == 0) {
            return lineBreak;
        }
        throw new ScannerException("while scanning a directive", startMark, "expected a comment or a line break, but found " + ch + "(" + ch + ")", this.reader.getMark());
    }

    private Token scanAnchor(boolean isAnchor) {
        Mark startMark = this.reader.getMark();
        String name = this.reader.peek() == '*' ? "alias" : "anchor";
        this.reader.forward();
        int length = 0;
        char ch = this.reader.peek(0);
        while (Constant.ALPHA.has(ch)) {
            length++;
            ch = this.reader.peek(length);
        }
        if (length != 0) {
            String value = this.reader.prefixForward(length);
            char ch2 = this.reader.peek();
            if (!Constant.NULL_BL_T_LINEBR.hasNo(ch2, "?:,]}%@`")) {
                Mark endMark = this.reader.getMark();
                if (isAnchor) {
                    return new AnchorToken(value, startMark, endMark);
                }
                return new AliasToken(value, startMark, endMark);
            }
            throw new ScannerException("while scanning an " + name, startMark, "expected alphabetic or numeric character, but found " + ch2 + "(" + this.reader.peek() + ")", this.reader.getMark());
        }
        throw new ScannerException("while scanning an " + name, startMark, "expected alphabetic or numeric character, but found but found " + ch, this.reader.getMark());
    }

    private Token scanTag() {
        String suffix;
        Mark startMark = this.reader.getMark();
        char ch = this.reader.peek(1);
        String handle = null;
        if (ch == '<') {
            this.reader.forward(2);
            suffix = scanTagUri("tag", startMark);
            if (this.reader.peek() == '>') {
                this.reader.forward();
            } else {
                throw new ScannerException("while scanning a tag", startMark, "expected '>', but found '" + this.reader.peek() + "' (" + this.reader.peek() + ")", this.reader.getMark());
            }
        } else if (Constant.NULL_BL_T_LINEBR.has(ch)) {
            suffix = "!";
            this.reader.forward();
        } else {
            int length = 1;
            boolean useHandle = false;
            while (true) {
                if (!Constant.NULL_BL_LINEBR.hasNo(ch)) {
                    break;
                } else if (ch == '!') {
                    useHandle = true;
                    break;
                } else {
                    length++;
                    ch = this.reader.peek(length);
                }
            }
            if (useHandle) {
                handle = scanTagHandle("tag", startMark);
            } else {
                handle = "!";
                this.reader.forward();
            }
            suffix = scanTagUri("tag", startMark);
        }
        char ch2 = this.reader.peek();
        if (!Constant.NULL_BL_LINEBR.hasNo(ch2)) {
            return new TagToken(new TagTuple(handle, suffix), startMark, this.reader.getMark());
        }
        throw new ScannerException("while scanning a tag", startMark, "expected ' ', but found '" + ch2 + "' (" + ch2 + ")", this.reader.getMark());
    }

    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r1v7, resolved type: java.lang.Object[]} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r8v4, resolved type: java.lang.Object[]} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r15v14, resolved type: java.lang.Object[]} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r15v17, resolved type: java.lang.Object[]} */
    /* JADX WARNING: type inference failed for: r19v1 */
    /* JADX WARNING: type inference failed for: r19v2 */
    /* JADX WARNING: type inference failed for: r15v13 */
    /* JADX WARNING: Multi-variable type inference failed */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private org.yaml.snakeyaml.tokens.Token scanBlockScalar(char r22) {
        /*
            r21 = this;
            r0 = r21
            r1 = 0
            r2 = 62
            r9 = r22
            if (r9 != r2) goto L_0x000b
            r2 = 1
            goto L_0x000c
        L_0x000b:
            r2 = 0
        L_0x000c:
            java.lang.StringBuilder r3 = new java.lang.StringBuilder
            r3.<init>()
            r10 = r3
            org.yaml.snakeyaml.reader.StreamReader r3 = r0.reader
            org.yaml.snakeyaml.error.Mark r11 = r3.getMark()
            org.yaml.snakeyaml.reader.StreamReader r3 = r0.reader
            r3.forward()
            org.yaml.snakeyaml.scanner.ScannerImpl$Chomping r12 = r0.scanBlockScalarIndicators(r11)
            int r13 = r12.getIncrement()
            r0.scanBlockScalarIgnoredLine(r11)
            int r3 = r0.indent
            r4 = 1
            int r3 = r3 + r4
            if (r3 >= r4) goto L_0x002f
            r3 = 1
        L_0x002f:
            r14 = r3
            r3 = 0
            r5 = 0
            r6 = 0
            r7 = -1
            if (r13 != r7) goto L_0x0055
            java.lang.Object[] r8 = r21.scanBlockScalarIndentation()
            r15 = r8[r1]
            r3 = r15
            java.lang.String r3 = (java.lang.String) r3
            r15 = r8[r4]
            java.lang.Integer r15 = (java.lang.Integer) r15
            int r5 = r15.intValue()
            r15 = 2
            r15 = r8[r15]
            org.yaml.snakeyaml.error.Mark r15 = (org.yaml.snakeyaml.error.Mark) r15
            int r6 = java.lang.Math.max(r14, r5)
        L_0x0051:
            r16 = r5
            r8 = r6
            goto L_0x0067
        L_0x0055:
            int r8 = r14 + r13
            int r6 = r8 + -1
            java.lang.Object[] r8 = r0.scanBlockScalarBreaks(r6)
            r15 = r8[r1]
            r3 = r15
            java.lang.String r3 = (java.lang.String) r3
            r15 = r8[r4]
            org.yaml.snakeyaml.error.Mark r15 = (org.yaml.snakeyaml.error.Mark) r15
            goto L_0x0051
        L_0x0067:
            r5 = r15
            java.lang.String r6 = ""
        L_0x006a:
            org.yaml.snakeyaml.reader.StreamReader r15 = r0.reader
            int r15 = r15.getColumn()
            if (r15 != r8) goto L_0x0106
            org.yaml.snakeyaml.reader.StreamReader r15 = r0.reader
            char r15 = r15.peek()
            if (r15 == 0) goto L_0x0106
            r10.append(r3)
            java.lang.String r15 = " \t"
            org.yaml.snakeyaml.reader.StreamReader r4 = r0.reader
            char r4 = r4.peek()
            int r4 = r15.indexOf(r4)
            if (r4 != r7) goto L_0x008d
            r4 = 1
            goto L_0x008e
        L_0x008d:
            r4 = 0
        L_0x008e:
            r15 = 0
        L_0x008f:
            org.yaml.snakeyaml.scanner.Constant r7 = org.yaml.snakeyaml.scanner.Constant.NULL_OR_LINEBR
            org.yaml.snakeyaml.reader.StreamReader r1 = r0.reader
            char r1 = r1.peek(r15)
            boolean r1 = r7.hasNo(r1)
            if (r1 == 0) goto L_0x00a2
            int r15 = r15 + 1
            r1 = 0
            r7 = -1
            goto L_0x008f
        L_0x00a2:
            org.yaml.snakeyaml.reader.StreamReader r1 = r0.reader
            java.lang.String r1 = r1.prefixForward(r15)
            r10.append(r1)
            java.lang.String r6 = r21.scanLineBreak()
            java.lang.Object[] r1 = r0.scanBlockScalarBreaks(r8)
            r7 = 0
            r19 = r1[r7]
            r3 = r19
            java.lang.String r3 = (java.lang.String) r3
            r17 = 1
            r19 = r1[r17]
            r5 = r19
            org.yaml.snakeyaml.error.Mark r5 = (org.yaml.snakeyaml.error.Mark) r5
            org.yaml.snakeyaml.reader.StreamReader r7 = r0.reader
            int r7 = r7.getColumn()
            if (r7 != r8) goto L_0x0106
            org.yaml.snakeyaml.reader.StreamReader r7 = r0.reader
            char r7 = r7.peek()
            if (r7 == 0) goto L_0x0106
            if (r2 == 0) goto L_0x00fb
            java.lang.String r7 = "\n"
            boolean r7 = r7.equals(r6)
            if (r7 == 0) goto L_0x00fb
            if (r4 == 0) goto L_0x00fb
            java.lang.String r7 = " \t"
            r20 = r1
            org.yaml.snakeyaml.reader.StreamReader r1 = r0.reader
            char r1 = r1.peek()
            int r1 = r7.indexOf(r1)
            r7 = -1
            if (r1 != r7) goto L_0x00fe
            int r1 = r3.length()
            if (r1 != 0) goto L_0x0101
            java.lang.String r1 = " "
            r10.append(r1)
            goto L_0x0101
        L_0x00fb:
            r20 = r1
            r7 = -1
        L_0x00fe:
            r10.append(r6)
        L_0x0101:
            r1 = 0
            r4 = 1
            goto L_0x006a
        L_0x0106:
            r15 = r3
            r17 = r5
            r1 = r6
            boolean r3 = r12.chompTailIsNotFalse()
            if (r3 == 0) goto L_0x0113
            r10.append(r1)
        L_0x0113:
            boolean r3 = r12.chompTailIsTrue()
            if (r3 == 0) goto L_0x011c
            r10.append(r15)
        L_0x011c:
            org.yaml.snakeyaml.tokens.ScalarToken r18 = new org.yaml.snakeyaml.tokens.ScalarToken
            java.lang.String r4 = r10.toString()
            r5 = 0
            r3 = r18
            r6 = r11
            r7 = r17
            r19 = r8
            r8 = r22
            r3.<init>(r4, r5, r6, r7, r8)
            return r18
        */
        throw new UnsupportedOperationException("Method not decompiled: org.yaml.snakeyaml.scanner.ScannerImpl.scanBlockScalar(char):org.yaml.snakeyaml.tokens.Token");
    }

    private Chomping scanBlockScalarIndicators(Mark startMark) {
        Boolean chomping;
        Boolean chomping2 = null;
        int increment = -1;
        char ch = this.reader.peek();
        if (ch == '-' || ch == '+') {
            if (ch == '+') {
                chomping = Boolean.TRUE;
            } else {
                chomping = Boolean.FALSE;
            }
            this.reader.forward();
            char ch2 = this.reader.peek();
            if (Character.isDigit(ch2)) {
                increment = Integer.parseInt(String.valueOf(ch2));
                if (increment != 0) {
                    this.reader.forward();
                } else {
                    throw new ScannerException("while scanning a block scalar", startMark, "expected indentation indicator in the range 1-9, but found 0", this.reader.getMark());
                }
            }
        } else if (Character.isDigit(ch)) {
            increment = Integer.parseInt(String.valueOf(ch));
            if (increment != 0) {
                this.reader.forward();
                char ch3 = this.reader.peek();
                if (ch3 == '-' || ch3 == '+') {
                    if (ch3 == '+') {
                        chomping2 = Boolean.TRUE;
                    } else {
                        chomping2 = Boolean.FALSE;
                    }
                    this.reader.forward();
                }
            } else {
                throw new ScannerException("while scanning a block scalar", startMark, "expected indentation indicator in the range 1-9, but found 0", this.reader.getMark());
            }
        }
        char ch4 = this.reader.peek();
        if (!Constant.NULL_BL_LINEBR.hasNo(ch4)) {
            return new Chomping(chomping2, increment);
        }
        throw new ScannerException("while scanning a block scalar", startMark, "expected chomping or indentation indicators, but found " + ch4, this.reader.getMark());
    }

    private String scanBlockScalarIgnoredLine(Mark startMark) {
        int ff = 0;
        while (this.reader.peek(ff) == ' ') {
            ff++;
        }
        if (ff > 0) {
            this.reader.forward(ff);
        }
        if (this.reader.peek() == '#') {
            int ff2 = 0;
            while (Constant.NULL_OR_LINEBR.hasNo(this.reader.peek(ff2))) {
                ff2++;
            }
            if (ff2 > 0) {
                this.reader.forward(ff2);
            }
        }
        char ch = this.reader.peek();
        String lineBreak = scanLineBreak();
        if (lineBreak.length() != 0 || ch == 0) {
            return lineBreak;
        }
        throw new ScannerException("while scanning a block scalar", startMark, "expected a comment or a line break, but found " + ch, this.reader.getMark());
    }

    private Object[] scanBlockScalarIndentation() {
        StringBuilder chunks = new StringBuilder();
        int maxIndent = 0;
        Mark endMark = this.reader.getMark();
        while (Constant.LINEBR.has(this.reader.peek(), " \r")) {
            if (this.reader.peek() != ' ') {
                chunks.append(scanLineBreak());
                endMark = this.reader.getMark();
            } else {
                this.reader.forward();
                if (this.reader.getColumn() > maxIndent) {
                    maxIndent = this.reader.getColumn();
                }
            }
        }
        return new Object[]{chunks.toString(), Integer.valueOf(maxIndent), endMark};
    }

    private Object[] scanBlockScalarBreaks(int indent2) {
        StringBuilder chunks = new StringBuilder();
        Mark endMark = this.reader.getMark();
        int ff = 0;
        for (int col = this.reader.getColumn(); col < indent2 && this.reader.peek(ff) == ' '; col++) {
            ff++;
        }
        if (ff > 0) {
            this.reader.forward(ff);
        }
        while (true) {
            String scanLineBreak = scanLineBreak();
            String lineBreak = scanLineBreak;
            if (scanLineBreak.length() != 0) {
                chunks.append(lineBreak);
                endMark = this.reader.getMark();
                int ff2 = 0;
                for (int col2 = this.reader.getColumn(); col2 < indent2 && this.reader.peek(ff2) == ' '; col2++) {
                    ff2++;
                }
                if (ff2 > 0) {
                    this.reader.forward(ff2);
                }
            } else {
                return new Object[]{chunks.toString(), endMark};
            }
        }
    }

    private Token scanFlowScalar(char style) {
        boolean _double;
        if (style == '\"') {
            _double = true;
        } else {
            _double = false;
        }
        StringBuilder chunks = new StringBuilder();
        Mark startMark = this.reader.getMark();
        char quote = this.reader.peek();
        this.reader.forward();
        chunks.append(scanFlowScalarNonSpaces(_double, startMark));
        while (this.reader.peek() != quote) {
            chunks.append(scanFlowScalarSpaces(startMark));
            chunks.append(scanFlowScalarNonSpaces(_double, startMark));
        }
        this.reader.forward();
        return new ScalarToken(chunks.toString(), false, startMark, this.reader.getMark(), style);
    }

    private String scanFlowScalarNonSpaces(boolean doubleQuoted, Mark startMark) {
        StringBuilder chunks = new StringBuilder();
        while (true) {
            int length = 0;
            while (Constant.NULL_BL_T_LINEBR.hasNo(this.reader.peek(length), "'\"\\")) {
                length++;
            }
            if (length != 0) {
                chunks.append(this.reader.prefixForward(length));
            }
            char ch = this.reader.peek();
            if (!doubleQuoted && ch == '\'' && this.reader.peek(1) == '\'') {
                chunks.append("'");
                this.reader.forward(2);
            } else if ((doubleQuoted && ch == '\'') || (!doubleQuoted && "\"\\".indexOf(ch) != -1)) {
                chunks.append(ch);
                this.reader.forward();
            } else if (doubleQuoted && ch == '\\') {
                this.reader.forward();
                char ch2 = this.reader.peek();
                if (ESCAPE_REPLACEMENTS.containsKey(Character.valueOf(ch2))) {
                    chunks.append(ESCAPE_REPLACEMENTS.get(Character.valueOf(ch2)));
                    this.reader.forward();
                } else if (ESCAPE_CODES.containsKey(Character.valueOf(ch2))) {
                    int length2 = ESCAPE_CODES.get(Character.valueOf(ch2)).intValue();
                    this.reader.forward();
                    String hex = this.reader.prefix(length2);
                    if (!NOT_HEXA.matcher(hex).find()) {
                        chunks.append(new String(Character.toChars(Integer.parseInt(hex, 16))));
                        this.reader.forward(length2);
                    } else {
                        throw new ScannerException("while scanning a double-quoted scalar", startMark, "expected escape sequence of " + length2 + " hexadecimal numbers, but found: " + hex, this.reader.getMark());
                    }
                } else if (scanLineBreak().length() != 0) {
                    chunks.append(scanFlowScalarBreaks(startMark));
                } else {
                    throw new ScannerException("while scanning a double-quoted scalar", startMark, "found unknown escape character " + ch2 + "(" + ch2 + ")", this.reader.getMark());
                }
            }
        }
        return chunks.toString();
    }

    private String scanFlowScalarSpaces(Mark startMark) {
        StringBuilder chunks = new StringBuilder();
        int length = 0;
        while (" \t".indexOf(this.reader.peek(length)) != -1) {
            length++;
        }
        String whitespaces = this.reader.prefixForward(length);
        if (this.reader.peek() != 0) {
            String lineBreak = scanLineBreak();
            if (lineBreak.length() != 0) {
                String breaks = scanFlowScalarBreaks(startMark);
                if (!"\n".equals(lineBreak)) {
                    chunks.append(lineBreak);
                } else if (breaks.length() == 0) {
                    chunks.append(" ");
                }
                chunks.append(breaks);
            } else {
                chunks.append(whitespaces);
            }
            return chunks.toString();
        }
        throw new ScannerException("while scanning a quoted scalar", startMark, "found unexpected end of stream", this.reader.getMark());
    }

    private String scanFlowScalarBreaks(Mark startMark) {
        StringBuilder chunks = new StringBuilder();
        while (true) {
            String prefix = this.reader.prefix(3);
            if (("---".equals(prefix) || "...".equals(prefix)) && Constant.NULL_BL_T_LINEBR.has(this.reader.peek(3))) {
                throw new ScannerException("while scanning a quoted scalar", startMark, "found unexpected document separator", this.reader.getMark());
            }
            while (" \t".indexOf(this.reader.peek()) != -1) {
                this.reader.forward();
            }
            String lineBreak = scanLineBreak();
            if (lineBreak.length() == 0) {
                return chunks.toString();
            }
            chunks.append(lineBreak);
        }
    }

    /* JADX WARNING: Code restructure failed: missing block: B:24:0x008f, code lost:
        r3 = r10;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private org.yaml.snakeyaml.tokens.Token scanPlain() {
        /*
            r14 = this;
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            org.yaml.snakeyaml.reader.StreamReader r1 = r14.reader
            org.yaml.snakeyaml.error.Mark r1 = r1.getMark()
            r2 = r1
            int r3 = r14.indent
            r4 = 1
            int r8 = r3 + 1
            java.lang.String r3 = ""
            r9 = r2
        L_0x0014:
            r10 = r3
            r2 = 0
            org.yaml.snakeyaml.reader.StreamReader r3 = r14.reader
            char r3 = r3.peek()
            r5 = 35
            if (r3 != r5) goto L_0x0022
            goto L_0x008f
        L_0x0022:
            r11 = r2
        L_0x0023:
            org.yaml.snakeyaml.reader.StreamReader r2 = r14.reader
            char r12 = r2.peek(r11)
            org.yaml.snakeyaml.scanner.Constant r2 = org.yaml.snakeyaml.scanner.Constant.NULL_BL_T_LINEBR
            boolean r2 = r2.has(r12)
            r3 = 58
            if (r2 != 0) goto L_0x005a
            int r2 = r14.flowLevel
            if (r2 != 0) goto L_0x0049
            if (r12 != r3) goto L_0x0049
            org.yaml.snakeyaml.scanner.Constant r2 = org.yaml.snakeyaml.scanner.Constant.NULL_BL_T_LINEBR
            org.yaml.snakeyaml.reader.StreamReader r6 = r14.reader
            int r7 = r11 + 1
            char r6 = r6.peek(r7)
            boolean r2 = r2.has(r6)
            if (r2 != 0) goto L_0x005a
        L_0x0049:
            int r2 = r14.flowLevel
            if (r2 == 0) goto L_0x0057
            java.lang.String r2 = ",:?[]{}"
            int r2 = r2.indexOf(r12)
            r6 = -1
            if (r2 == r6) goto L_0x0057
            goto L_0x005a
        L_0x0057:
            int r11 = r11 + 1
            goto L_0x0023
        L_0x005a:
            int r2 = r14.flowLevel
            if (r2 == 0) goto L_0x008c
            if (r12 != r3) goto L_0x008c
            org.yaml.snakeyaml.scanner.Constant r2 = org.yaml.snakeyaml.scanner.Constant.NULL_BL_T_LINEBR
            org.yaml.snakeyaml.reader.StreamReader r3 = r14.reader
            int r6 = r11 + 1
            char r3 = r3.peek(r6)
            java.lang.String r6 = ",[]{}"
            boolean r2 = r2.hasNo(r3, r6)
            if (r2 != 0) goto L_0x0073
            goto L_0x008c
        L_0x0073:
            org.yaml.snakeyaml.reader.StreamReader r2 = r14.reader
            r2.forward(r11)
            org.yaml.snakeyaml.scanner.ScannerException r13 = new org.yaml.snakeyaml.scanner.ScannerException
            org.yaml.snakeyaml.reader.StreamReader r2 = r14.reader
            org.yaml.snakeyaml.error.Mark r6 = r2.getMark()
            java.lang.String r3 = "while scanning a plain scalar"
            java.lang.String r5 = "found unexpected ':'"
            java.lang.String r7 = "Please check http://pyyaml.org/wiki/YAMLColonInFlowContext for details."
            r2 = r13
            r4 = r1
            r2.<init>(r3, r4, r5, r6, r7)
            throw r13
        L_0x008c:
            if (r11 != 0) goto L_0x0091
        L_0x008f:
            r3 = r10
            goto L_0x00c7
        L_0x0091:
            r2 = 0
            r14.allowSimpleKey = r2
            r0.append(r10)
            org.yaml.snakeyaml.reader.StreamReader r2 = r14.reader
            java.lang.String r2 = r2.prefixForward(r11)
            r0.append(r2)
            org.yaml.snakeyaml.reader.StreamReader r2 = r14.reader
            org.yaml.snakeyaml.error.Mark r9 = r2.getMark()
            java.lang.String r3 = r14.scanPlainSpaces()
            int r2 = r3.length()
            if (r2 == 0) goto L_0x00c7
            org.yaml.snakeyaml.reader.StreamReader r2 = r14.reader
            char r2 = r2.peek()
            if (r2 == r5) goto L_0x00c7
            int r2 = r14.flowLevel
            if (r2 != 0) goto L_0x00c5
            org.yaml.snakeyaml.reader.StreamReader r2 = r14.reader
            int r2 = r2.getColumn()
            if (r2 >= r8) goto L_0x00c5
            goto L_0x00c7
        L_0x00c5:
            goto L_0x0014
        L_0x00c7:
            org.yaml.snakeyaml.tokens.ScalarToken r2 = new org.yaml.snakeyaml.tokens.ScalarToken
            java.lang.String r5 = r0.toString()
            r2.<init>(r5, r1, r9, r4)
            return r2
        */
        throw new UnsupportedOperationException("Method not decompiled: org.yaml.snakeyaml.scanner.ScannerImpl.scanPlain():org.yaml.snakeyaml.tokens.Token");
    }

    private String scanPlainSpaces() {
        int length = 0;
        while (true) {
            if (this.reader.peek(length) != ' ' && this.reader.peek(length) != 9) {
                break;
            }
            length++;
        }
        String whitespaces = this.reader.prefixForward(length);
        String lineBreak = scanLineBreak();
        if (lineBreak.length() == 0) {
            return whitespaces;
        }
        this.allowSimpleKey = true;
        String prefix = this.reader.prefix(3);
        if ("---".equals(prefix)) {
            return "";
        }
        if ("...".equals(prefix) && Constant.NULL_BL_T_LINEBR.has(this.reader.peek(3))) {
            return "";
        }
        StringBuilder breaks = new StringBuilder();
        while (true) {
            if (this.reader.peek() == ' ') {
                this.reader.forward();
            } else {
                String lb = scanLineBreak();
                if (lb.length() != 0) {
                    breaks.append(lb);
                    String prefix2 = this.reader.prefix(3);
                    if ("---".equals(prefix2)) {
                        return "";
                    }
                    if ("...".equals(prefix2) && Constant.NULL_BL_T_LINEBR.has(this.reader.peek(3))) {
                        return "";
                    }
                } else if (!"\n".equals(lineBreak)) {
                    return lineBreak + breaks;
                } else if (breaks.length() == 0) {
                    return " ";
                } else {
                    return breaks.toString();
                }
            }
        }
    }

    private String scanTagHandle(String name, Mark startMark) {
        char ch = this.reader.peek();
        if (ch == '!') {
            int length = 1;
            char ch2 = this.reader.peek(1);
            if (ch2 != ' ') {
                while (Constant.ALPHA.has(ch2)) {
                    length++;
                    ch2 = this.reader.peek(length);
                }
                if (ch2 == '!') {
                    length++;
                } else {
                    this.reader.forward(length);
                    throw new ScannerException("while scanning a " + name, startMark, "expected '!', but found " + ch2 + "(" + ch2 + ")", this.reader.getMark());
                }
            }
            return this.reader.prefixForward(length);
        }
        throw new ScannerException("while scanning a " + name, startMark, "expected '!', but found " + ch + "(" + ch + ")", this.reader.getMark());
    }

    private String scanTagUri(String name, Mark startMark) {
        StringBuilder chunks = new StringBuilder();
        int length = 0;
        char ch = this.reader.peek(0);
        while (Constant.URI_CHARS.has(ch)) {
            if (ch == '%') {
                chunks.append(this.reader.prefixForward(length));
                length = 0;
                chunks.append(scanUriEscapes(name, startMark));
            } else {
                length++;
            }
            ch = this.reader.peek(length);
        }
        if (length != 0) {
            chunks.append(this.reader.prefixForward(length));
        }
        if (chunks.length() != 0) {
            return chunks.toString();
        }
        throw new ScannerException("while scanning a " + name, startMark, "expected URI, but found " + ch + "(" + ch + ")", this.reader.getMark());
    }

    private String scanUriEscapes(String name, Mark startMark) {
        int length = 1;
        while (this.reader.peek(length * 3) == '%') {
            length++;
        }
        Mark beginningMark = this.reader.getMark();
        ByteBuffer buff = ByteBuffer.allocate(length);
        while (this.reader.peek() == '%') {
            this.reader.forward();
            try {
                buff.put((byte) Integer.parseInt(this.reader.prefix(2), 16));
                this.reader.forward(2);
            } catch (NumberFormatException e) {
                throw new ScannerException("while scanning a " + name, startMark, "expected URI escape sequence of 2 hexadecimal numbers, but found " + this.reader.peek() + "(" + this.reader.peek() + ") and " + this.reader.peek(1) + "(" + this.reader.peek(1) + ")", this.reader.getMark());
            }
        }
        buff.flip();
        try {
            return UriEncoder.decode(buff);
        } catch (CharacterCodingException e2) {
            throw new ScannerException("while scanning a " + name, startMark, "expected URI in UTF-8: " + e2.getMessage(), beginningMark);
        }
    }

    private String scanLineBreak() {
        char ch = this.reader.peek();
        if (ch == 13 || ch == 10 || ch == 133) {
            if (ch == 13 && 10 == this.reader.peek(1)) {
                this.reader.forward(2);
                return "\n";
            }
            this.reader.forward();
            return "\n";
        } else if (ch != 8232 && ch != 8233) {
            return "";
        } else {
            this.reader.forward();
            return String.valueOf(ch);
        }
    }

    private static class Chomping {
        private final int increment;
        private final Boolean value;

        public Chomping(Boolean value2, int increment2) {
            this.value = value2;
            this.increment = increment2;
        }

        public boolean chompTailIsNotFalse() {
            return this.value == null || this.value.booleanValue();
        }

        public boolean chompTailIsTrue() {
            return this.value != null && this.value.booleanValue();
        }

        public int getIncrement() {
            return this.increment;
        }
    }
}
