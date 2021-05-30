package org.bytedeco.javacpp.tools;

import java.io.BufferedReader;
import java.io.Closeable;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.Reader;
import java.io.StringReader;
import java.util.ArrayList;

class Tokenizer implements Closeable {
    StringBuilder buffer;
    File file;
    int lastChar;
    int lineNumber;
    String lineSeparator;
    Reader reader;
    String text;

    Tokenizer(Reader reader2, File file2, int lineNumber2) {
        this.file = null;
        this.text = null;
        this.reader = null;
        this.lineSeparator = null;
        this.lastChar = -1;
        this.lineNumber = 1;
        this.buffer = new StringBuilder();
        this.reader = reader2;
        this.file = file2;
        this.lineNumber = lineNumber2;
    }

    Tokenizer(String text2, File file2, int lineNumber2) {
        this.file = null;
        this.text = null;
        this.reader = null;
        this.lineSeparator = null;
        this.lastChar = -1;
        this.lineNumber = 1;
        this.buffer = new StringBuilder();
        this.text = text2;
        this.reader = new StringReader(text2);
        this.file = file2;
        this.lineNumber = lineNumber2;
    }

    Tokenizer(File file2) throws IOException {
        this(file2, (String) null);
    }

    Tokenizer(File file2, String encoding) throws IOException {
        this.file = null;
        this.text = null;
        this.reader = null;
        this.lineSeparator = null;
        this.lastChar = -1;
        this.lineNumber = 1;
        this.buffer = new StringBuilder();
        this.file = file2;
        FileInputStream fis = new FileInputStream(file2);
        this.reader = new BufferedReader(encoding != null ? new InputStreamReader(fis, encoding) : new InputStreamReader(fis));
    }

    public void filterLines(String[] patterns, boolean skip) throws IOException {
        if (patterns != null) {
            StringBuilder lines = new StringBuilder();
            BufferedReader lineReader = this.reader instanceof BufferedReader ? (BufferedReader) this.reader : new BufferedReader(this.reader);
            while (true) {
                String readLine = lineReader.readLine();
                String line = readLine;
                if (readLine != null) {
                    int i = 0;
                    while (i < patterns.length && !line.matches(patterns[i])) {
                        i += 2;
                    }
                    if (i < patterns.length) {
                        if (!skip) {
                            lines.append(line + "\n");
                        }
                        while (i + 1 < patterns.length) {
                            String readLine2 = lineReader.readLine();
                            String line2 = readLine2;
                            if (readLine2 != null) {
                                if (!skip) {
                                    lines.append(line2 + "\n");
                                }
                                if (line2.matches(patterns[i + 1])) {
                                    break;
                                }
                            } else {
                                break;
                            }
                        }
                    } else if (skip) {
                        lines.append(line + "\n");
                    }
                } else {
                    this.reader.close();
                    this.reader = new StringReader(lines.toString());
                    return;
                }
            }
        }
    }

    public void close() throws IOException {
        this.reader.close();
    }

    /* access modifiers changed from: package-private */
    public int readChar() throws IOException {
        int c2 = -1;
        if (this.lastChar != -1) {
            int c = this.lastChar;
            this.lastChar = -1;
            return c;
        }
        int c3 = this.reader.read();
        if (c3 != 13 && c3 != 10) {
            return c3;
        }
        if (this.text == null) {
            this.lineNumber++;
        }
        if (c3 == 13) {
            c2 = this.reader.read();
        }
        if (this.lineSeparator == null) {
            this.lineSeparator = (c3 == 13 && c2 == 10) ? "\r\n" : c3 == 13 ? "\r" : "\n";
        }
        if (c2 != 10) {
            this.lastChar = c2;
        }
        return 10;
    }

    /* JADX WARNING: Code restructure failed: missing block: B:142:0x0243, code lost:
        if (r6 != 120) goto L_0x0253;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public org.bytedeco.javacpp.tools.Token nextToken() throws java.io.IOException {
        /*
            r19 = this;
            r1 = r19
            org.bytedeco.javacpp.tools.Token r0 = new org.bytedeco.javacpp.tools.Token
            r0.<init>()
            r2 = r0
            int r0 = r19.readChar()
            java.lang.StringBuilder r3 = r1.buffer
            r4 = 0
            r3.setLength(r4)
            boolean r3 = java.lang.Character.isWhitespace(r0)
            r5 = -1
            if (r3 == 0) goto L_0x0033
            java.lang.StringBuilder r3 = r1.buffer
            char r6 = (char) r0
            r3.append(r6)
        L_0x001f:
            int r3 = r19.readChar()
            r0 = r3
            if (r3 == r5) goto L_0x0033
            boolean r3 = java.lang.Character.isWhitespace(r0)
            if (r3 == 0) goto L_0x0033
            java.lang.StringBuilder r3 = r1.buffer
            char r6 = (char) r0
            r3.append(r6)
            goto L_0x001f
        L_0x0033:
            java.io.File r3 = r1.file
            r2.file = r3
            java.lang.String r3 = r1.text
            r2.text = r3
            int r3 = r1.lineNumber
            r2.lineNumber = r3
            java.lang.StringBuilder r3 = r1.buffer
            java.lang.String r3 = r3.toString()
            r2.spacing = r3
            java.lang.StringBuilder r3 = r1.buffer
            r3.setLength(r4)
            boolean r3 = java.lang.Character.isLetter(r0)
            if (r3 != 0) goto L_0x02e4
            r3 = 95
            if (r0 != r3) goto L_0x0058
            goto L_0x02e4
        L_0x0058:
            boolean r3 = java.lang.Character.isDigit(r0)
            r6 = 6
            r7 = 1
            r8 = 46
            if (r3 != 0) goto L_0x01b2
            if (r0 == r8) goto L_0x01b2
            r3 = 45
            if (r0 == r3) goto L_0x01b2
            r3 = 43
            if (r0 != r3) goto L_0x006e
            goto L_0x01b2
        L_0x006e:
            r3 = 92
            r8 = 39
            if (r0 != r8) goto L_0x00a6
            r2.type = r7
            java.lang.StringBuilder r4 = r1.buffer
            r4.append(r8)
        L_0x007b:
            int r4 = r19.readChar()
            r0 = r4
            if (r4 == r5) goto L_0x0097
            if (r0 == r8) goto L_0x0097
            java.lang.StringBuilder r4 = r1.buffer
            char r6 = (char) r0
            r4.append(r6)
            if (r0 != r3) goto L_0x007b
            int r0 = r19.readChar()
            java.lang.StringBuilder r4 = r1.buffer
            char r6 = (char) r0
            r4.append(r6)
            goto L_0x007b
        L_0x0097:
            java.lang.StringBuilder r3 = r1.buffer
            r3.append(r8)
            java.lang.StringBuilder r3 = r1.buffer
            java.lang.String r3 = r3.toString()
            r2.value = r3
            goto L_0x0316
        L_0x00a6:
            r7 = 34
            if (r0 != r7) goto L_0x00dd
            r4 = 3
            r2.type = r4
            java.lang.StringBuilder r4 = r1.buffer
            r4.append(r7)
        L_0x00b2:
            int r4 = r19.readChar()
            r0 = r4
            if (r4 == r5) goto L_0x00ce
            if (r0 == r7) goto L_0x00ce
            java.lang.StringBuilder r4 = r1.buffer
            char r6 = (char) r0
            r4.append(r6)
            if (r0 != r3) goto L_0x00b2
            int r0 = r19.readChar()
            java.lang.StringBuilder r4 = r1.buffer
            char r6 = (char) r0
            r4.append(r6)
            goto L_0x00b2
        L_0x00ce:
            java.lang.StringBuilder r3 = r1.buffer
            r3.append(r7)
            java.lang.StringBuilder r3 = r1.buffer
            java.lang.String r3 = r3.toString()
            r2.value = r3
            goto L_0x0316
        L_0x00dd:
            r7 = 4
            r8 = 47
            if (r0 != r8) goto L_0x0150
            int r0 = r19.readChar()
            if (r0 != r8) goto L_0x0114
            r2.type = r7
            java.lang.StringBuilder r6 = r1.buffer
            r6.append(r8)
            r6.append(r8)
        L_0x00f3:
            int r6 = r19.readChar()
            r0 = r6
            if (r6 == r5) goto L_0x0108
            if (r4 == r3) goto L_0x0100
            r6 = 10
            if (r0 == r6) goto L_0x0108
        L_0x0100:
            java.lang.StringBuilder r6 = r1.buffer
            char r7 = (char) r0
            r6.append(r7)
            r4 = r0
            goto L_0x00f3
        L_0x0108:
            java.lang.StringBuilder r3 = r1.buffer
            java.lang.String r3 = r3.toString()
            r2.value = r3
            r1.lastChar = r0
            goto L_0x0316
        L_0x0114:
            r3 = 42
            if (r0 != r3) goto L_0x014a
            r2.type = r7
            java.lang.StringBuilder r3 = r1.buffer
            r3.append(r8)
            r6 = 42
            r3.append(r6)
        L_0x0125:
            r3 = r4
            int r4 = r19.readChar()
            r0 = r4
            if (r4 == r5) goto L_0x013b
            r4 = 42
            if (r3 != r4) goto L_0x0133
            if (r0 == r8) goto L_0x013b
        L_0x0133:
            java.lang.StringBuilder r4 = r1.buffer
            char r6 = (char) r0
            r4.append(r6)
            r4 = r0
            goto L_0x0125
        L_0x013b:
            java.lang.StringBuilder r4 = r1.buffer
            r4.append(r8)
            java.lang.StringBuilder r4 = r1.buffer
            java.lang.String r4 = r4.toString()
            r2.value = r4
            goto L_0x0316
        L_0x014a:
            r1.lastChar = r0
            r2.type = r8
            goto L_0x0316
        L_0x0150:
            r4 = 58
            if (r0 != r4) goto L_0x0169
            int r3 = r19.readChar()
            r4 = 58
            if (r3 != r4) goto L_0x0163
            r2.type = r6
            java.lang.String r4 = "::"
            r2.value = r4
            goto L_0x0167
        L_0x0163:
            r2.type = r0
            r1.lastChar = r3
        L_0x0167:
            goto L_0x0316
        L_0x0169:
            r4 = 38
            if (r0 != r4) goto L_0x0182
            int r3 = r19.readChar()
            r4 = 38
            if (r3 != r4) goto L_0x017c
            r2.type = r6
            java.lang.String r4 = "&&"
            r2.value = r4
            goto L_0x0180
        L_0x017c:
            r2.type = r0
            r1.lastChar = r3
        L_0x0180:
            goto L_0x0316
        L_0x0182:
            r4 = 35
            if (r0 != r4) goto L_0x019b
            int r3 = r19.readChar()
            r4 = 35
            if (r3 != r4) goto L_0x0195
            r2.type = r6
            java.lang.String r4 = "##"
            r2.value = r4
            goto L_0x0199
        L_0x0195:
            r2.type = r0
            r1.lastChar = r3
        L_0x0199:
            goto L_0x0316
        L_0x019b:
            if (r0 != r3) goto L_0x01ae
            int r3 = r19.readChar()
            r4 = 10
            if (r3 != r4) goto L_0x01ac
            r2.type = r7
            java.lang.String r4 = "\n"
            r2.value = r4
            return r2
        L_0x01ac:
            r1.lastChar = r3
        L_0x01ae:
            r2.type = r0
            goto L_0x0316
        L_0x01b2:
            if (r0 != r8) goto L_0x01cc
            int r3 = r19.readChar()
            if (r3 != r8) goto L_0x01ca
            int r9 = r19.readChar()
            if (r9 != r8) goto L_0x01c7
            r2.type = r6
            java.lang.String r4 = "..."
            r2.value = r4
            return r2
        L_0x01c7:
            r1.lastChar = r9
            goto L_0x01cc
        L_0x01ca:
            r1.lastChar = r3
        L_0x01cc:
            r3 = 2
            if (r0 != r8) goto L_0x01d1
            r6 = 2
            goto L_0x01d2
        L_0x01d1:
            r6 = 1
        L_0x01d2:
            r2.type = r6
            java.lang.StringBuilder r6 = r1.buffer
            char r9 = (char) r0
            r6.append(r9)
            r6 = 0
            r9 = 0
            r10 = 0
            r11 = 0
            r12 = r10
            r10 = r9
            r9 = r6
            r6 = r0
            r0 = 0
        L_0x01e3:
            r13 = r0
            int r0 = r19.readChar()
            r6 = r0
            r14 = 76
            if (r0 == r5) goto L_0x0267
            boolean r0 = java.lang.Character.isDigit(r6)
            r15 = 117(0x75, float:1.64E-43)
            r4 = 108(0x6c, float:1.51E-43)
            r5 = 85
            if (r0 != 0) goto L_0x022b
            if (r6 == r8) goto L_0x022b
            r0 = 45
            if (r6 == r0) goto L_0x022b
            r0 = 43
            if (r6 == r0) goto L_0x022b
            r0 = 97
            if (r6 < r0) goto L_0x020b
            r0 = 102(0x66, float:1.43E-43)
            if (r6 <= r0) goto L_0x022b
        L_0x020b:
            r0 = 105(0x69, float:1.47E-43)
            if (r6 == r0) goto L_0x022b
            if (r6 == r4) goto L_0x022b
            if (r6 == r15) goto L_0x022b
            r0 = 120(0x78, float:1.68E-43)
            if (r6 == r0) goto L_0x022b
            r0 = 65
            if (r6 < r0) goto L_0x021f
            r0 = 70
            if (r6 <= r0) goto L_0x022b
        L_0x021f:
            r0 = 73
            if (r6 == r0) goto L_0x022b
            if (r6 == r14) goto L_0x022b
            if (r6 == r5) goto L_0x022b
            r0 = 88
            if (r6 != r0) goto L_0x0267
        L_0x022b:
            if (r6 == r8) goto L_0x0251
            r0 = 69
            if (r6 == r0) goto L_0x024e
            if (r6 == r14) goto L_0x024b
            if (r6 == r5) goto L_0x0248
            r0 = 88
            if (r6 == r0) goto L_0x0246
            r0 = 101(0x65, float:1.42E-43)
            if (r6 == r0) goto L_0x024e
            if (r6 == r4) goto L_0x024b
            if (r6 == r15) goto L_0x0248
            r0 = 120(0x78, float:1.68E-43)
            if (r6 == r0) goto L_0x0246
            goto L_0x0253
        L_0x0246:
            r0 = 1
            goto L_0x0254
        L_0x0248:
            r0 = 1
            r11 = r0
            goto L_0x0253
        L_0x024b:
            r0 = 1
            r12 = r0
            goto L_0x0253
        L_0x024e:
            r0 = 1
            r10 = r0
            goto L_0x0253
        L_0x0251:
            r2.type = r3
        L_0x0253:
            r0 = r13
        L_0x0254:
            if (r6 == r4) goto L_0x0262
            if (r6 == r14) goto L_0x0262
            if (r6 == r15) goto L_0x0262
            if (r6 == r5) goto L_0x0262
            java.lang.StringBuilder r4 = r1.buffer
            char r5 = (char) r6
            r4.append(r5)
        L_0x0262:
            r9 = r6
            r4 = 0
            r5 = -1
            goto L_0x01e3
        L_0x0267:
            if (r13 != 0) goto L_0x0275
            if (r10 != 0) goto L_0x0273
            r0 = 102(0x66, float:1.43E-43)
            if (r9 == r0) goto L_0x0273
            r0 = 70
            if (r9 != r0) goto L_0x0275
        L_0x0273:
            r2.type = r3
        L_0x0275:
            int r0 = r2.type
            if (r0 != r7) goto L_0x02ac
            if (r12 != 0) goto L_0x02ac
            java.lang.StringBuilder r0 = r1.buffer     // Catch:{ NumberFormatException -> 0x02a0 }
            java.lang.String r0 = r0.toString()     // Catch:{ NumberFormatException -> 0x02a0 }
            java.lang.Long r0 = java.lang.Long.decode(r0)     // Catch:{ NumberFormatException -> 0x02a0 }
            long r3 = r0.longValue()     // Catch:{ NumberFormatException -> 0x02a0 }
            r0 = 32
            long r3 = r3 >> r0
            r17 = 0
            int r0 = (r3 > r17 ? 1 : (r3 == r17 ? 0 : -1))
            if (r0 == 0) goto L_0x029b
            r17 = -1
            int r0 = (r3 > r17 ? 1 : (r3 == r17 ? 0 : -1))
            if (r0 == 0) goto L_0x029b
            r16 = 1
            goto L_0x029d
        L_0x029b:
            r16 = 0
        L_0x029d:
            r12 = r16
            goto L_0x02ac
        L_0x02a0:
            r0 = move-exception
            java.lang.StringBuilder r3 = r1.buffer
            int r3 = r3.length()
            r4 = 16
            if (r3 < r4) goto L_0x02ac
            r12 = 1
        L_0x02ac:
            java.lang.StringBuilder r0 = r1.buffer
            java.lang.String r0 = r0.toString()
            java.lang.String r3 = "i64"
            boolean r0 = r0.endsWith(r3)
            if (r0 == 0) goto L_0x02c8
            java.lang.StringBuilder r0 = r1.buffer
            java.lang.StringBuilder r3 = r1.buffer
            int r3 = r3.length()
            int r3 = r3 + -3
            r0.setLength(r3)
            r12 = 1
        L_0x02c8:
            int r0 = r2.type
            if (r0 != r7) goto L_0x02d7
            if (r12 != 0) goto L_0x02d2
            if (r11 == 0) goto L_0x02d7
            if (r13 != 0) goto L_0x02d7
        L_0x02d2:
            java.lang.StringBuilder r0 = r1.buffer
            r0.append(r14)
        L_0x02d7:
            java.lang.StringBuilder r0 = r1.buffer
            java.lang.String r0 = r0.toString()
            r2.value = r0
            r1.lastChar = r6
            r0 = r6
            goto L_0x0316
        L_0x02e4:
            r3 = 5
            r2.type = r3
            java.lang.StringBuilder r3 = r1.buffer
            char r4 = (char) r0
            r3.append(r4)
        L_0x02ed:
            int r3 = r19.readChar()
            r0 = r3
            r4 = -1
            if (r3 == r4) goto L_0x030c
            boolean r3 = java.lang.Character.isDigit(r0)
            if (r3 != 0) goto L_0x0305
            boolean r3 = java.lang.Character.isLetter(r0)
            if (r3 != 0) goto L_0x0305
            r3 = 95
            if (r0 != r3) goto L_0x030c
        L_0x0305:
            java.lang.StringBuilder r3 = r1.buffer
            char r5 = (char) r0
            r3.append(r5)
            goto L_0x02ed
        L_0x030c:
            java.lang.StringBuilder r3 = r1.buffer
            java.lang.String r3 = r3.toString()
            r2.value = r3
            r1.lastChar = r0
        L_0x0316:
            return r2
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.Tokenizer.nextToken():org.bytedeco.javacpp.tools.Token");
    }

    /* access modifiers changed from: package-private */
    public Token[] tokenize() {
        ArrayList<Token> tokens = new ArrayList<>();
        while (true) {
            try {
                Token nextToken = nextToken();
                Token token = nextToken;
                if (nextToken.isEmpty()) {
                    return (Token[]) tokens.toArray(new Token[tokens.size()]);
                }
                tokens.add(token);
            } catch (IOException ex) {
                throw new RuntimeException(ex);
            }
        }
    }
}
