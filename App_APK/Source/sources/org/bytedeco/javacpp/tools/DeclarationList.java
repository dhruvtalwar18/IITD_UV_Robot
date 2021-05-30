package org.bytedeco.javacpp.tools;

import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;

class DeclarationList extends ArrayList<Declaration> {
    Context context = null;
    ListIterator<Info> infoIterator = null;
    InfoMap infoMap = null;
    DeclarationList inherited = null;
    String spacing = null;
    TemplateMap templateMap = null;

    DeclarationList() {
    }

    DeclarationList(DeclarationList inherited2) {
        this.inherited = inherited2;
    }

    /* access modifiers changed from: package-private */
    /* JADX WARNING: Code restructure failed: missing block: B:14:0x0046, code lost:
        r3 = move-exception;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:18:0x004a, code lost:
        if (r2 != null) goto L_0x004c;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:20:?, code lost:
        r1.close();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:21:0x0050, code lost:
        r4 = move-exception;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:22:0x0051, code lost:
        r2.addSuppressed(r4);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:23:0x0055, code lost:
        r1.close();
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public java.lang.String rescan(java.lang.String r6) {
        /*
            r5 = this;
            java.lang.String r0 = r5.spacing
            if (r0 != 0) goto L_0x0005
            return r6
        L_0x0005:
            java.lang.String r0 = ""
            java.util.Scanner r1 = new java.util.Scanner
            r1.<init>(r6)
            r2 = 0
        L_0x000d:
            boolean r3 = r1.hasNextLine()     // Catch:{ Throwable -> 0x0048 }
            if (r3 == 0) goto L_0x0042
            java.lang.StringBuilder r3 = new java.lang.StringBuilder     // Catch:{ Throwable -> 0x0048 }
            r3.<init>()     // Catch:{ Throwable -> 0x0048 }
            r3.append(r0)     // Catch:{ Throwable -> 0x0048 }
            java.lang.String r4 = r5.spacing     // Catch:{ Throwable -> 0x0048 }
            r3.append(r4)     // Catch:{ Throwable -> 0x0048 }
            java.lang.String r4 = r1.nextLine()     // Catch:{ Throwable -> 0x0048 }
            r3.append(r4)     // Catch:{ Throwable -> 0x0048 }
            java.lang.String r3 = r3.toString()     // Catch:{ Throwable -> 0x0048 }
            r0 = r3
            java.lang.String r3 = r5.spacing     // Catch:{ Throwable -> 0x0048 }
            r4 = 10
            int r3 = r3.lastIndexOf(r4)     // Catch:{ Throwable -> 0x0048 }
            if (r3 < 0) goto L_0x003d
            java.lang.String r4 = r5.spacing     // Catch:{ Throwable -> 0x0048 }
            java.lang.String r4 = r4.substring(r3)     // Catch:{ Throwable -> 0x0048 }
            goto L_0x003f
        L_0x003d:
            java.lang.String r4 = "\n"
        L_0x003f:
            r5.spacing = r4     // Catch:{ Throwable -> 0x0048 }
            goto L_0x000d
        L_0x0042:
            r1.close()
            return r0
        L_0x0046:
            r3 = move-exception
            goto L_0x004a
        L_0x0048:
            r2 = move-exception
            throw r2     // Catch:{ all -> 0x0046 }
        L_0x004a:
            if (r2 == 0) goto L_0x0055
            r1.close()     // Catch:{ Throwable -> 0x0050 }
            goto L_0x0058
        L_0x0050:
            r4 = move-exception
            r2.addSuppressed(r4)
            goto L_0x0058
        L_0x0055:
            r1.close()
        L_0x0058:
            throw r3
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.DeclarationList.rescan(java.lang.String):java.lang.String");
    }

    public boolean add(Declaration decl) {
        Info info;
        boolean add = true;
        if (!(this.templateMap == null || !this.templateMap.empty() || (decl.type == null && decl.declarator == null))) {
            if (this.infoIterator == null) {
                TemplateMap templateMap2 = this.templateMap;
                Type type = decl.type;
                templateMap2.type = type;
                Type type2 = type;
                TemplateMap templateMap3 = this.templateMap;
                Declarator declarator = decl.declarator;
                templateMap3.declarator = declarator;
                Declarator dcl = declarator;
                List<Info> infoList = this.infoMap.get(dcl != null ? dcl.cppName : type2.cppName);
                boolean hasJavaName = false;
                for (Info info2 : infoList) {
                    hasJavaName |= info2.javaNames != null && info2.javaNames.length > 0;
                }
                if (!decl.function || hasJavaName) {
                    this.infoIterator = infoList.size() > 0 ? infoList.listIterator() : null;
                }
            }
            add = false;
        }
        if (decl.declarator != null && decl.declarator.type != null) {
            Info info3 = this.infoMap.getFirst(decl.declarator.type.cppName);
            if (info3 != null && info3.skip && info3.valueTypes == null && info3.pointerTypes == null) {
                add = false;
            } else if (decl.declarator.parameters != null) {
                Declarator[] declaratorArr = decl.declarator.parameters.declarators;
                int length = declaratorArr.length;
                Info info4 = info3;
                int i = 0;
                while (true) {
                    if (i < length) {
                        Declarator d = declaratorArr[i];
                        if (d != null && d.type != null && (info = this.infoMap.getFirst(d.type.cppName)) != null && info.skip && info.valueTypes == null && info.pointerTypes == null) {
                            add = false;
                            break;
                        }
                        i++;
                    } else {
                        break;
                    }
                }
            }
        }
        if (decl.type != null && decl.type.javaName.equals("Pointer")) {
            add = false;
        }
        if (!add) {
            return false;
        }
        List<Declaration> stack = new ArrayList<>();
        ListIterator<Declaration> it = stack.listIterator();
        it.add(decl);
        it.previous();
        while (it.hasNext()) {
            Declarator dcl2 = it.next().declarator;
            if (!(dcl2 == null || dcl2.definition == null)) {
                it.add(dcl2.definition);
                it.previous();
            }
            if (!(dcl2 == null || dcl2.parameters == null || dcl2.parameters.declarators == null)) {
                for (Declarator d2 : dcl2.parameters.declarators) {
                    if (!(d2 == null || d2.definition == null)) {
                        it.add(d2.definition);
                        it.previous();
                    }
                }
            }
        }
        boolean add2 = false;
        while (!stack.isEmpty()) {
            Declaration decl2 = stack.remove(stack.size() - 1);
            if (this.context != null) {
                decl2.inaccessible = this.context.inaccessible && (!this.context.virtualize || decl2.declarator == null || decl2.declarator.type == null || !decl2.declarator.type.virtual);
            }
            if (decl2.text.length() == 0) {
                decl2.inaccessible = true;
            }
            ListIterator<Declaration> it2 = listIterator();
            boolean found = false;
            while (it2.hasNext()) {
                Declaration d3 = it2.next();
                if (d3 != null && d3.signature != null && d3.signature.length() > 0 && d3.signature.equals(decl2.signature)) {
                    if ((!d3.constMember || decl2.constMember) && ((!d3.inaccessible || decl2.inaccessible) && (!d3.incomplete || decl2.incomplete))) {
                        found = true;
                    } else {
                        it2.remove();
                    }
                }
            }
            if (this.inherited != null) {
                ListIterator<Declaration> it3 = this.inherited.listIterator();
                while (true) {
                    if (!it3.hasNext()) {
                        break;
                    }
                    Declaration d4 = it3.next();
                    if (d4.signature.length() > 0 && d4.signature.equals(decl2.signature) && !d4.incomplete && decl2.incomplete) {
                        found = true;
                        break;
                    }
                }
            }
            if (!found) {
                decl2.text = rescan(decl2.text);
                super.add(decl2);
                add2 = true;
            }
        }
        return add2;
    }
}
