package org.bytedeco.javacpp.tools;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import org.apache.xmlrpc.serializer.BooleanSerializer;
import org.apache.xmlrpc.serializer.DoubleSerializer;
import org.apache.xmlrpc.serializer.FloatSerializer;
import org.apache.xmlrpc.serializer.I4Serializer;

public class InfoMap extends HashMap<String, List<Info>> {
    static final InfoMap defaults = new InfoMap((InfoMap) null).put(new Info("basic/containers").cppTypes("std::bitset", "std::deque", "std::list", "std::map", "std::queue", "std::set", "std::stack", "std::vector", "std::valarray", "std::pair", "std::forward_list", "std::priority_queue", "std::unordered_map", "std::unordered_set")).put(new Info("basic/types").cppTypes("signed", "unsigned", "char", "short", I4Serializer.INT_TAG, "long", "bool", FloatSerializer.FLOAT_TAG, DoubleSerializer.DOUBLE_TAG, "_Bool", "_Complex", "_Imaginary", "complex", "imaginary")).put(new Info("noexcept").annotations("@NoException")).put(new Info("__COUNTER__").cppText("#define __COUNTER__ 0")).put(new Info(" __attribute__", "__declspec", "static_assert").annotations(new String[0]).skip()).put(new Info("void").valueTypes("void").pointerTypes("Pointer")).put(new Info("std::nullptr_t").valueTypes("Pointer").pointerTypes("PointerPointer")).put(new Info("FILE", "time_t", "va_list", "std::exception", "std::istream", "std::ostream", "std::iostream", "std::ifstream", "std::ofstream", "std::fstream").cast().pointerTypes("Pointer")).put(new Info("int8_t", "__int8", "jbyte", "signed char").valueTypes("byte").pointerTypes("BytePointer", "ByteBuffer", "byte[]")).put(new Info("uint8_t", "unsigned __int8", "char", "unsigned char").cast().valueTypes("byte").pointerTypes("BytePointer", "ByteBuffer", "byte[]")).put(new Info("int16_t", "__int16", "jshort", "short", "signed short", "short int", "signed short int").valueTypes("short").pointerTypes("ShortPointer", "ShortBuffer", "short[]")).put(new Info("uint16_t", "unsigned __int16", "unsigned short", "unsigned short int").cast().valueTypes("short").pointerTypes("ShortPointer", "ShortBuffer", "short[]")).put(new Info("int32_t", "__int32", "jint", I4Serializer.INT_TAG, "signed int", "signed").valueTypes(I4Serializer.INT_TAG).pointerTypes("IntPointer", "IntBuffer", "int[]")).put(new Info("uint32_t", "unsigned __int32", "unsigned int", "unsigned").cast().valueTypes(I4Serializer.INT_TAG).pointerTypes("IntPointer", "IntBuffer", "int[]")).put(new Info("jlong", "long long", "signed long long", "long long int", "signed long long int").valueTypes("long").pointerTypes("LongPointer", "LongBuffer", "long[]")).put(new Info("int64_t", "__int64", "uint64_t", "unsigned __int64", "unsigned long long", "unsigned long long int").cast().valueTypes("long").pointerTypes("LongPointer", "LongBuffer", "long[]")).put(new Info("long", "signed long", "long int", "signed long int").valueTypes("long").pointerTypes("CLongPointer")).put(new Info("unsigned long", "unsigned long int").cast().valueTypes("long").pointerTypes("CLongPointer")).put(new Info("size_t", "ptrdiff_t", "intptr_t", "uintptr_t", "off_t").cast().valueTypes("long").pointerTypes("SizeTPointer")).put(new Info(FloatSerializer.FLOAT_TAG, "jfloat").valueTypes(FloatSerializer.FLOAT_TAG).pointerTypes("FloatPointer", "FloatBuffer", "float[]")).put(new Info(DoubleSerializer.DOUBLE_TAG, "jdouble").valueTypes(DoubleSerializer.DOUBLE_TAG).pointerTypes("DoublePointer", "DoubleBuffer", "double[]")).put(new Info("long double").cast().valueTypes(DoubleSerializer.DOUBLE_TAG).pointerTypes("Pointer")).put(new Info("std::complex<float>", "float _Complex", "float _Imaginary", "float complex", "float imaginary").cast().pointerTypes("FloatPointer", "FloatBuffer", "float[]")).put(new Info("std::complex<double>", "double _Complex", "double _Imaginary", "double complex", "double imaginary").cast().pointerTypes("DoublePointer", "DoubleBuffer", "double[]")).put(new Info("_Bool", "bool", "jboolean").cast().valueTypes(BooleanSerializer.BOOLEAN_TAG).pointerTypes("BoolPointer", "boolean[]")).put(new Info("wchar_t", "WCHAR").cast().valueTypes("char", I4Serializer.INT_TAG).pointerTypes("CharPointer", "IntPointer")).put(new Info("const char").valueTypes("byte").pointerTypes("@Cast(\"const char*\") BytePointer", "String")).put(new Info("boost::shared_ptr", "std::shared_ptr").annotations("@SharedPtr")).put(new Info("boost::movelib::unique_ptr", "std::unique_ptr").annotations("@UniquePtr")).put(new Info("std::string").annotations("@StdString").valueTypes("BytePointer", "String").pointerTypes("BytePointer")).put(new Info("std::wstring").annotations("@StdWString").valueTypes("CharPointer", "IntPointer").pointerTypes("CharPointer", "IntPointer")).put(new Info("std::vector").annotations("@StdVector")).put(new Info("abstract").javaNames("_abstract")).put(new Info(BooleanSerializer.BOOLEAN_TAG).javaNames("_boolean")).put(new Info("byte").javaNames("_byte")).put(new Info("extends").javaNames("_extends")).put(new Info("finally").javaNames("_finally")).put(new Info("implements").javaNames("_implements")).put(new Info("import").javaNames("_import")).put(new Info("instanceof").javaNames("_instanceof")).put(new Info("native").javaNames("_native")).put(new Info("package").javaNames("_package")).put(new Info("super").javaNames("_super")).put(new Info("synchronized").javaNames("_synchronized")).put(new Info("transient").javaNames("_transient")).put(new Info("operator ->").javaNames("access")).put(new Info("operator ()").javaNames("apply")).put(new Info("operator []").javaNames("get")).put(new Info("operator =").javaNames("put")).put(new Info("operator +").javaNames("add")).put(new Info("operator -").javaNames("subtract")).put(new Info("operator *").javaNames("multiply")).put(new Info("operator /").javaNames("divide")).put(new Info("operator %").javaNames("mod")).put(new Info("operator ++").javaNames("increment")).put(new Info("operator --").javaNames("decrement")).put(new Info("operator ==").javaNames("equals")).put(new Info("operator !=").javaNames("notEquals")).put(new Info("operator <").javaNames("lessThan")).put(new Info("operator >").javaNames("greaterThan")).put(new Info("operator <=").javaNames("lessThanEquals")).put(new Info("operator >=").javaNames("greaterThanEquals")).put(new Info("operator !").javaNames("not")).put(new Info("operator &&").javaNames("and")).put(new Info("operator ||").javaNames("or")).put(new Info("operator &").javaNames("and")).put(new Info("operator |").javaNames("or")).put(new Info("operator ^").javaNames("xor")).put(new Info("operator ~").javaNames("not")).put(new Info("operator <<").javaNames("shiftLeft")).put(new Info("operator >>").javaNames("shiftRight")).put(new Info("operator +=").javaNames("addPut")).put(new Info("operator -=").javaNames("subtractPut")).put(new Info("operator *=").javaNames("multiplyPut")).put(new Info("operator /=").javaNames("dividePut")).put(new Info("operator %=").javaNames("modPut")).put(new Info("operator &=").javaNames("andPut")).put(new Info("operator |=").javaNames("orPut")).put(new Info("operator ^=").javaNames("xorPut")).put(new Info("operator <<=").javaNames("shiftLeftPut")).put(new Info("operator >>=").javaNames("shiftRightPut")).put(new Info("operator new").javaNames("_new")).put(new Info("operator delete").javaNames("_delete")).put(new Info("getClass").javaNames("_getClass")).put(new Info("notify").javaNames("_notify")).put(new Info("notifyAll").javaNames("_notifyAll")).put(new Info("wait").javaNames("_wait")).put(new Info("allocate").javaNames("_allocate")).put(new Info("close").javaNames("_close")).put(new Info("deallocate").javaNames("_deallocate")).put(new Info("address").javaNames("_address")).put(new Info("position").javaNames("_position")).put(new Info("limit").javaNames("_limit")).put(new Info("capacity").javaNames("_capacity")).put(new Info("fill").javaNames("_fill")).put(new Info("zero").javaNames("_zero"));
    InfoMap parent;

    public InfoMap() {
        this.parent = null;
        this.parent = defaults;
    }

    public InfoMap(InfoMap parent2) {
        this.parent = null;
        this.parent = parent2;
    }

    /* access modifiers changed from: package-private */
    public String normalize(String name, boolean unconst, boolean untemplate) {
        int i;
        String name2 = name;
        if (name2 == null || name.length() == 0 || name2.startsWith("basic/")) {
            return name2;
        }
        boolean simpleType = true;
        Token[] tokens = new Tokenizer(name2, (File) null, 0).tokenize();
        int n = tokens.length;
        Info info = getFirst("basic/types");
        String[] basicTypes = info != null ? info.cppTypes : new String[0];
        Arrays.sort(basicTypes);
        String prefix = null;
        boolean foundConst = false;
        int i2 = 0;
        while (true) {
            i = 1;
            if (i2 >= n) {
                break;
            }
            if (tokens[i2].match(Token.CONST, Token.CONSTEXPR)) {
                foundConst = true;
                for (int j = i2 + 1; j < n; j++) {
                    tokens[j - 1] = tokens[j];
                }
                i2--;
                n--;
            } else {
                if (tokens[i2].match(Token.CLASS, Token.STRUCT, Token.UNION)) {
                    prefix = tokens[i2].value;
                    for (int j2 = i2 + 1; j2 < n; j2++) {
                        tokens[j2 - 1] = tokens[j2];
                    }
                    i2--;
                    n--;
                } else if (Arrays.binarySearch(basicTypes, tokens[i2].value) < 0) {
                    simpleType = false;
                    break;
                }
            }
            i2++;
        }
        if (simpleType) {
            Arrays.sort(tokens, 0, n);
            StringBuilder sb = new StringBuilder();
            sb.append(foundConst ? "const " : "");
            sb.append(tokens[0].value);
            name2 = sb.toString();
            while (true) {
                int i3 = i;
                if (i3 >= n) {
                    break;
                }
                name2 = name2 + " " + tokens[i3].value;
                i = i3 + 1;
            }
        } else if (untemplate) {
            int template = -1;
            int count = 0;
            for (int i4 = 0; i4 < n; i4++) {
                if (tokens[i4].match('<')) {
                    if (count == 0) {
                        template = i4;
                    }
                    count++;
                } else {
                    if (tokens[i4].match('>') && count - 1 == 0 && i4 + 1 != n) {
                        template = -1;
                    }
                }
            }
            int i5 = 0;
            if (template >= 0) {
                String name3 = foundConst ? "const " : "";
                while (true) {
                    int i6 = i5;
                    if (i6 >= template) {
                        break;
                    }
                    name3 = name2 + tokens[i6].value;
                    i5 = i6 + 1;
                }
            }
        }
        if (unconst && foundConst) {
            name2 = name2.substring(name2.indexOf("const") + 5);
        }
        if (prefix != null) {
            name2 = name2.substring(name2.indexOf(prefix) + prefix.length());
        }
        return name2.trim();
    }

    public boolean containsKey(Object key) {
        return super.containsKey(key) || (this.parent != null && this.parent.containsKey(key));
    }

    public List<Info> get(String cppName) {
        return get(cppName, true);
    }

    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r2v6, resolved type: java.util.ArrayList} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r2v7, resolved type: java.util.ArrayList} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r4v5, resolved type: java.lang.Object} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r2v8, resolved type: java.util.List<org.bytedeco.javacpp.tools.Info>} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r5v2, resolved type: java.lang.Object} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r2v10, resolved type: java.util.List<org.bytedeco.javacpp.tools.Info>} */
    /* JADX WARNING: Multi-variable type inference failed */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public java.util.List<org.bytedeco.javacpp.tools.Info> get(java.lang.String r7, boolean r8) {
        /*
            r6 = this;
            r0 = 0
            java.lang.String r1 = r6.normalize(r7, r0, r0)
            java.lang.Object r2 = super.get(r1)
            java.util.List r2 = (java.util.List) r2
            r3 = 0
            r4 = 1
            if (r2 != 0) goto L_0x001a
            java.lang.String r1 = r6.normalize(r7, r4, r0)
            java.lang.Object r5 = super.get(r1)
            r2 = r5
            java.util.List r2 = (java.util.List) r2
        L_0x001a:
            if (r2 != 0) goto L_0x002a
            if (r8 == 0) goto L_0x002a
            java.lang.String r1 = r6.normalize(r7, r4, r4)
            java.lang.Object r4 = super.get(r1)
            r2 = r4
            java.util.List r2 = (java.util.List) r2
            r3 = 1
        L_0x002a:
            if (r2 != 0) goto L_0x0032
            java.util.ArrayList r4 = new java.util.ArrayList
            r4.<init>()
            r2 = r4
        L_0x0032:
            org.bytedeco.javacpp.tools.InfoMap r4 = r6.parent
            if (r4 == 0) goto L_0x0053
            org.bytedeco.javacpp.tools.InfoMap r4 = r6.parent
            java.util.List r4 = r4.get((java.lang.String) r7, (boolean) r8)
            if (r4 == 0) goto L_0x0053
            int r5 = r4.size()
            if (r5 <= 0) goto L_0x0053
            java.util.ArrayList r5 = new java.util.ArrayList
            r5.<init>(r2)
            r2 = r5
            if (r3 == 0) goto L_0x0050
            r2.addAll(r0, r4)
            goto L_0x0053
        L_0x0050:
            r2.addAll(r4)
        L_0x0053:
            return r2
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.InfoMap.get(java.lang.String, boolean):java.util.List");
    }

    public Info get(int index, String cppName) {
        return get(index, cppName, true);
    }

    public Info get(int index, String cppName, boolean partial) {
        List<Info> infoList = get(cppName, partial);
        if (infoList.size() > 0) {
            return infoList.get(index);
        }
        return null;
    }

    public Info getFirst(String cppName) {
        return getFirst(cppName, true);
    }

    public Info getFirst(String cppName, boolean partial) {
        List<Info> infoList = get(cppName, partial);
        if (infoList.size() > 0) {
            return infoList.get(0);
        }
        return null;
    }

    public InfoMap put(int index, Info info) {
        for (String cppName : info.cppNames != null ? info.cppNames : new String[]{null}) {
            for (String key : new String[]{normalize(cppName, false, false), normalize(cppName, false, true)}) {
                List<Info> infoList = (List) super.get(key);
                if (infoList == null) {
                    List<Info> arrayList = new ArrayList<>();
                    infoList = arrayList;
                    super.put(key, arrayList);
                }
                if (!infoList.contains(info)) {
                    if (index != -1) {
                        infoList.add(index, info);
                    } else {
                        infoList.add(info);
                    }
                }
            }
        }
        return this;
    }

    public InfoMap put(Info info) {
        return put(-1, info);
    }

    public InfoMap putFirst(Info info) {
        return put(0, info);
    }
}
