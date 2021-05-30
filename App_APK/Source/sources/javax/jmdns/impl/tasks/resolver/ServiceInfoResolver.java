package javax.jmdns.impl.tasks.resolver;

import java.io.IOException;
import javax.jmdns.impl.DNSOutgoing;
import javax.jmdns.impl.DNSQuestion;
import javax.jmdns.impl.DNSRecord;
import javax.jmdns.impl.JmDNSImpl;
import javax.jmdns.impl.ServiceInfoImpl;
import javax.jmdns.impl.constants.DNSRecordClass;
import javax.jmdns.impl.constants.DNSRecordType;

public class ServiceInfoResolver extends DNSResolverTask {
    private final ServiceInfoImpl _info;

    public ServiceInfoResolver(JmDNSImpl jmDNSImpl, ServiceInfoImpl info) {
        super(jmDNSImpl);
        this._info = info;
        info.setDns(getDns());
        getDns().addListener(info, DNSQuestion.newQuestion(info.getQualifiedName(), DNSRecordType.TYPE_ANY, DNSRecordClass.CLASS_IN, false));
    }

    public String getName() {
        StringBuilder sb = new StringBuilder();
        sb.append("ServiceInfoResolver(");
        sb.append(getDns() != null ? getDns().getName() : "");
        sb.append(")");
        return sb.toString();
    }

    public boolean cancel() {
        boolean result = super.cancel();
        if (!this._info.isPersistent()) {
            getDns().removeListener(this._info);
        }
        return result;
    }

    /* access modifiers changed from: protected */
    public DNSOutgoing addAnswers(DNSOutgoing out) throws IOException {
        DNSOutgoing newOut = out;
        if (this._info.hasData()) {
            return newOut;
        }
        long now = System.currentTimeMillis();
        DNSOutgoing newOut2 = addAnswer(addAnswer(newOut, (DNSRecord) getDns().getCache().getDNSEntry(this._info.getQualifiedName(), DNSRecordType.TYPE_SRV, DNSRecordClass.CLASS_IN), now), (DNSRecord) getDns().getCache().getDNSEntry(this._info.getQualifiedName(), DNSRecordType.TYPE_TXT, DNSRecordClass.CLASS_IN), now);
        if (this._info.getServer().length() > 0) {
            return addAnswer(addAnswer(newOut2, (DNSRecord) getDns().getCache().getDNSEntry(this._info.getServer(), DNSRecordType.TYPE_A, DNSRecordClass.CLASS_IN), now), (DNSRecord) getDns().getCache().getDNSEntry(this._info.getServer(), DNSRecordType.TYPE_AAAA, DNSRecordClass.CLASS_IN), now);
        }
        return newOut2;
    }

    /* access modifiers changed from: protected */
    public DNSOutgoing addQuestions(DNSOutgoing out) throws IOException {
        DNSOutgoing newOut = out;
        if (this._info.hasData()) {
            return newOut;
        }
        DNSOutgoing newOut2 = addQuestion(addQuestion(newOut, DNSQuestion.newQuestion(this._info.getQualifiedName(), DNSRecordType.TYPE_SRV, DNSRecordClass.CLASS_IN, false)), DNSQuestion.newQuestion(this._info.getQualifiedName(), DNSRecordType.TYPE_TXT, DNSRecordClass.CLASS_IN, false));
        if (this._info.getServer().length() > 0) {
            return addQuestion(addQuestion(newOut2, DNSQuestion.newQuestion(this._info.getServer(), DNSRecordType.TYPE_A, DNSRecordClass.CLASS_IN, false)), DNSQuestion.newQuestion(this._info.getServer(), DNSRecordType.TYPE_AAAA, DNSRecordClass.CLASS_IN, false));
        }
        return newOut2;
    }

    /* access modifiers changed from: protected */
    public String description() {
        StringBuilder sb = new StringBuilder();
        sb.append("querying service info: ");
        sb.append(this._info != null ? this._info.getQualifiedName() : "null");
        return sb.toString();
    }
}
