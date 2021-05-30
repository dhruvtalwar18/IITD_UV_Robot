package javax.jmdns.impl.tasks.resolver;

import java.io.IOException;
import javax.jmdns.ServiceInfo;
import javax.jmdns.impl.DNSOutgoing;
import javax.jmdns.impl.DNSQuestion;
import javax.jmdns.impl.DNSRecord;
import javax.jmdns.impl.JmDNSImpl;
import javax.jmdns.impl.constants.DNSConstants;
import javax.jmdns.impl.constants.DNSRecordClass;
import javax.jmdns.impl.constants.DNSRecordType;

public class ServiceResolver extends DNSResolverTask {
    private final String _type;

    public ServiceResolver(JmDNSImpl jmDNSImpl, String type) {
        super(jmDNSImpl);
        this._type = type;
    }

    public String getName() {
        StringBuilder sb = new StringBuilder();
        sb.append("ServiceResolver(");
        sb.append(getDns() != null ? getDns().getName() : "");
        sb.append(")");
        return sb.toString();
    }

    /* access modifiers changed from: protected */
    public DNSOutgoing addAnswers(DNSOutgoing out) throws IOException {
        DNSOutgoing newOut = out;
        long now = System.currentTimeMillis();
        for (ServiceInfo info : getDns().getServices().values()) {
            newOut = addAnswer(newOut, (DNSRecord) new DNSRecord.Pointer(info.getType(), DNSRecordClass.CLASS_IN, false, DNSConstants.DNS_TTL, info.getQualifiedName()), now);
        }
        return newOut;
    }

    /* access modifiers changed from: protected */
    public DNSOutgoing addQuestions(DNSOutgoing out) throws IOException {
        return addQuestion(out, DNSQuestion.newQuestion(this._type, DNSRecordType.TYPE_PTR, DNSRecordClass.CLASS_IN, false));
    }

    /* access modifiers changed from: protected */
    public String description() {
        return "querying service";
    }
}
