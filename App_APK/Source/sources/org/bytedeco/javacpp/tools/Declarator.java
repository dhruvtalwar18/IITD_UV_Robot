package org.bytedeco.javacpp.tools;

class Declarator {
    boolean constPointer = false;
    String cppName = "";
    Declaration definition = null;
    int indices = 0;
    int indirections = 0;
    int infoNumber = 0;
    String javaName = "";
    boolean operator = false;
    Parameters parameters = null;
    boolean reference = false;
    String signature = "";
    Type type = null;

    Declarator() {
    }
}
