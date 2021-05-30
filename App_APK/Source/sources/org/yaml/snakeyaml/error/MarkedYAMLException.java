package org.yaml.snakeyaml.error;

public class MarkedYAMLException extends YAMLException {
    private static final long serialVersionUID = -9119388488683035101L;
    private String context;
    private Mark contextMark;
    private String note;
    private String problem;
    private Mark problemMark;

    protected MarkedYAMLException(String context2, Mark contextMark2, String problem2, Mark problemMark2, String note2) {
        this(context2, contextMark2, problem2, problemMark2, note2, (Throwable) null);
    }

    protected MarkedYAMLException(String context2, Mark contextMark2, String problem2, Mark problemMark2, String note2, Throwable cause) {
        super(context2 + "; " + problem2, cause);
        this.context = context2;
        this.contextMark = contextMark2;
        this.problem = problem2;
        this.problemMark = problemMark2;
        this.note = note2;
    }

    protected MarkedYAMLException(String context2, Mark contextMark2, String problem2, Mark problemMark2) {
        this(context2, contextMark2, problem2, problemMark2, (String) null, (Throwable) null);
    }

    protected MarkedYAMLException(String context2, Mark contextMark2, String problem2, Mark problemMark2, Throwable cause) {
        this(context2, contextMark2, problem2, problemMark2, (String) null, cause);
    }

    public String toString() {
        StringBuilder lines = new StringBuilder();
        if (this.context != null) {
            lines.append(this.context);
            lines.append("\n");
        }
        if (this.contextMark != null && (this.problem == null || this.problemMark == null || this.contextMark.getName().equals(this.problemMark.getName()) || this.contextMark.getLine() != this.problemMark.getLine() || this.contextMark.getColumn() != this.problemMark.getColumn())) {
            lines.append(this.contextMark.toString());
            lines.append("\n");
        }
        if (this.problem != null) {
            lines.append(this.problem);
            lines.append("\n");
        }
        if (this.problemMark != null) {
            lines.append(this.problemMark.toString());
            lines.append("\n");
        }
        if (this.note != null) {
            lines.append(this.note);
            lines.append("\n");
        }
        return lines.toString();
    }

    public String getContext() {
        return this.context;
    }

    public Mark getContextMark() {
        return this.contextMark;
    }

    public String getProblem() {
        return this.problem;
    }

    public Mark getProblemMark() {
        return this.problemMark;
    }
}
