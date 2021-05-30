package org.xbill.DNS;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import org.apache.commons.httpclient.cookie.CookieSpec;
import org.xbill.DNS.Tokenizer;

public class Master {
    private int currentDClass;
    private long currentTTL;
    private int currentType;
    private long defaultTTL;
    private File file;
    private Generator generator;
    private List generators;
    private Master included;
    private Record last;
    private boolean needSOATTL;
    private boolean noExpandGenerate;
    private Name origin;
    private Tokenizer st;

    Master(File file2, Name origin2, long initialTTL) throws IOException {
        this.last = null;
        this.included = null;
        if (origin2 == null || origin2.isAbsolute()) {
            this.file = file2;
            this.st = new Tokenizer(file2);
            this.origin = origin2;
            this.defaultTTL = initialTTL;
            return;
        }
        throw new RelativeNameException(origin2);
    }

    public Master(String filename, Name origin2, long ttl) throws IOException {
        this(new File(filename), origin2, ttl);
    }

    public Master(String filename, Name origin2) throws IOException {
        this(new File(filename), origin2, -1);
    }

    public Master(String filename) throws IOException {
        this(new File(filename), (Name) null, -1);
    }

    public Master(InputStream in, Name origin2, long ttl) {
        this.last = null;
        this.included = null;
        if (origin2 == null || origin2.isAbsolute()) {
            this.st = new Tokenizer(in);
            this.origin = origin2;
            this.defaultTTL = ttl;
            return;
        }
        throw new RelativeNameException(origin2);
    }

    public Master(InputStream in, Name origin2) {
        this(in, origin2, -1);
    }

    public Master(InputStream in) {
        this(in, (Name) null, -1);
    }

    private Name parseName(String s, Name origin2) throws TextParseException {
        try {
            return Name.fromString(s, origin2);
        } catch (TextParseException e) {
            throw this.st.exception(e.getMessage());
        }
    }

    private void parseTTLClassAndType() throws IOException {
        boolean seen_class = false;
        String s = this.st.getString();
        int value = DClass.value(s);
        this.currentDClass = value;
        if (value >= 0) {
            s = this.st.getString();
            seen_class = true;
        }
        this.currentTTL = -1;
        try {
            this.currentTTL = TTL.parseTTL(s);
            s = this.st.getString();
        } catch (NumberFormatException e) {
            if (this.defaultTTL >= 0) {
                this.currentTTL = this.defaultTTL;
            } else if (this.last != null) {
                this.currentTTL = this.last.getTTL();
            }
        }
        if (!seen_class) {
            int value2 = DClass.value(s);
            this.currentDClass = value2;
            if (value2 >= 0) {
                s = this.st.getString();
            } else {
                this.currentDClass = 1;
            }
        }
        int value3 = Type.value(s);
        this.currentType = value3;
        if (value3 < 0) {
            Tokenizer tokenizer = this.st;
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Invalid type '");
            stringBuffer.append(s);
            stringBuffer.append("'");
            throw tokenizer.exception(stringBuffer.toString());
        } else if (this.currentTTL >= 0) {
        } else {
            if (this.currentType == 6) {
                this.needSOATTL = true;
                this.currentTTL = 0;
                return;
            }
            throw this.st.exception("missing TTL");
        }
    }

    private long parseUInt32(String s) {
        if (!Character.isDigit(s.charAt(0))) {
            return -1;
        }
        try {
            long l = Long.parseLong(s);
            if (l < 0 || l > 4294967295L) {
                return -1;
            }
            return l;
        } catch (NumberFormatException e) {
            return -1;
        }
    }

    private void startGenerate() throws IOException {
        long step;
        String s = this.st.getIdentifier();
        int n = s.indexOf("-");
        if (n >= 0) {
            String startstr = s.substring(0, n);
            String endstr = s.substring(n + 1);
            String stepstr = null;
            int n2 = endstr.indexOf(CookieSpec.PATH_DELIM);
            if (n2 >= 0) {
                stepstr = endstr.substring(n2 + 1);
                endstr = endstr.substring(0, n2);
            }
            long start = parseUInt32(startstr);
            long end = parseUInt32(endstr);
            if (stepstr != null) {
                step = parseUInt32(stepstr);
            } else {
                step = 1;
            }
            long step2 = step;
            if (start < 0 || end < 0 || start > end || step2 <= 0) {
                Tokenizer tokenizer = this.st;
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Invalid $GENERATE range specifier: ");
                stringBuffer.append(s);
                throw tokenizer.exception(stringBuffer.toString());
            }
            String nameSpec = this.st.getIdentifier();
            parseTTLClassAndType();
            if (Generator.supportedType(this.currentType)) {
                String rdataSpec = this.st.getIdentifier();
                this.st.getEOL();
                this.st.unget();
                int i = this.currentType;
                long j = step2;
                int i2 = i;
                String str = nameSpec;
                int i3 = n2;
                Generator generator2 = r7;
                int i4 = i2;
                Generator generator3 = new Generator(start, end, j, str, i4, this.currentDClass, this.currentTTL, rdataSpec, this.origin);
                this.generator = generator2;
                if (this.generators == null) {
                    this.generators = new ArrayList(1);
                }
                this.generators.add(this.generator);
                return;
            }
            Tokenizer tokenizer2 = this.st;
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append("$GENERATE does not support ");
            stringBuffer2.append(Type.string(this.currentType));
            stringBuffer2.append(" records");
            throw tokenizer2.exception(stringBuffer2.toString());
        }
        Tokenizer tokenizer3 = this.st;
        StringBuffer stringBuffer3 = new StringBuffer();
        stringBuffer3.append("Invalid $GENERATE range specifier: ");
        stringBuffer3.append(s);
        throw tokenizer3.exception(stringBuffer3.toString());
    }

    private void endGenerate() throws IOException {
        this.st.getEOL();
        this.generator = null;
    }

    private Record nextGenerated() throws IOException {
        try {
            return this.generator.nextRecord();
        } catch (Tokenizer.TokenizerException e) {
            Tokenizer tokenizer = this.st;
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Parsing $GENERATE: ");
            stringBuffer.append(e.getBaseMessage());
            throw tokenizer.exception(stringBuffer.toString());
        } catch (TextParseException e2) {
            Tokenizer tokenizer2 = this.st;
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append("Parsing $GENERATE: ");
            stringBuffer2.append(e2.getMessage());
            throw tokenizer2.exception(stringBuffer2.toString());
        }
    }

    public Record _nextRecord() throws IOException {
        Name name;
        File newfile;
        if (this.included != null) {
            Record rec = this.included.nextRecord();
            if (rec != null) {
                return rec;
            }
            this.included = null;
        }
        if (this.generator != null) {
            Record rec2 = nextGenerated();
            if (rec2 != null) {
                return rec2;
            }
            endGenerate();
        }
        while (true) {
            Tokenizer.Token token = this.st.get(true, false);
            if (token.type == 2) {
                Tokenizer.Token next = this.st.get();
                if (next.type != 1) {
                    if (next.type == 0) {
                        return null;
                    }
                    this.st.unget();
                    if (this.last != null) {
                        name = this.last.getName();
                    } else {
                        throw this.st.exception("no owner");
                    }
                }
            } else if (token.type == 1) {
                continue;
            } else if (token.type == 0) {
                return null;
            } else {
                if (token.value.charAt(0) == '$') {
                    String s = token.value;
                    if (s.equalsIgnoreCase("$ORIGIN")) {
                        this.origin = this.st.getName(Name.root);
                        this.st.getEOL();
                    } else if (s.equalsIgnoreCase("$TTL")) {
                        this.defaultTTL = this.st.getTTL();
                        this.st.getEOL();
                    } else if (s.equalsIgnoreCase("$INCLUDE")) {
                        String filename = this.st.getString();
                        if (this.file != null) {
                            newfile = new File(this.file.getParent(), filename);
                        } else {
                            newfile = new File(filename);
                        }
                        Name incorigin = this.origin;
                        Tokenizer.Token token2 = this.st.get();
                        if (token2.isString()) {
                            incorigin = parseName(token2.value, Name.root);
                            this.st.getEOL();
                        }
                        this.included = new Master(newfile, incorigin, this.defaultTTL);
                        return nextRecord();
                    } else if (!s.equalsIgnoreCase("$GENERATE")) {
                        Tokenizer tokenizer = this.st;
                        StringBuffer stringBuffer = new StringBuffer();
                        stringBuffer.append("Invalid directive: ");
                        stringBuffer.append(s);
                        throw tokenizer.exception(stringBuffer.toString());
                    } else if (this.generator == null) {
                        startGenerate();
                        if (!this.noExpandGenerate) {
                            return nextGenerated();
                        }
                        endGenerate();
                    } else {
                        throw new IllegalStateException("cannot nest $GENERATE");
                    }
                } else {
                    Name name2 = parseName(token.value, this.origin);
                    if (this.last == null || !name2.equals(this.last.getName())) {
                        name = name2;
                    } else {
                        name = this.last.getName();
                    }
                }
            }
        }
        parseTTLClassAndType();
        this.last = Record.fromString(name, this.currentType, this.currentDClass, this.currentTTL, this.st, this.origin);
        if (this.needSOATTL) {
            long ttl = ((SOARecord) this.last).getMinimum();
            this.last.setTTL(ttl);
            this.defaultTTL = ttl;
            this.needSOATTL = false;
        }
        return this.last;
    }

    public Record nextRecord() throws IOException {
        Record rec = null;
        try {
            rec = _nextRecord();
            return rec;
        } finally {
            if (rec == null) {
                this.st.close();
            }
        }
    }

    public void expandGenerate(boolean wantExpand) {
        this.noExpandGenerate = !wantExpand;
    }

    public Iterator generators() {
        if (this.generators != null) {
            return Collections.unmodifiableList(this.generators).iterator();
        }
        return Collections.EMPTY_LIST.iterator();
    }

    /* access modifiers changed from: protected */
    public void finalize() {
        this.st.close();
    }
}
