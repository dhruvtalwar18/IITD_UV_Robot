package org.ros.android.view;

import android.content.Context;
import android.content.res.Resources;
import android.graphics.drawable.Drawable;
import android.util.AttributeSet;
import android.widget.Button;
import android.widget.TableLayout;
import diagnostic_msgs.DiagnosticArray;
import diagnostic_msgs.DiagnosticStatus;
import java.util.List;
import org.ros.android.android_core_components.R;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;

public class DiagnosticsArrayView extends TableLayout implements NodeMain {
    private static final String DIAGNOSTICS_AGGREGATOR_TOPIC = "/diagnostics_agg";
    private static final int STALE = 3;
    /* access modifiers changed from: private */
    public Drawable errorDrawable;
    /* access modifiers changed from: private */
    public Drawable okDrawable;
    /* access modifiers changed from: private */
    public Drawable staleDrawable;
    /* access modifiers changed from: private */
    public Drawable warningDrawable;

    public DiagnosticsArrayView(Context context) {
        super(context);
        init();
    }

    public DiagnosticsArrayView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    private void init() {
        Resources resources = getResources();
        this.errorDrawable = resources.getDrawable(R.drawable.error);
        this.warningDrawable = resources.getDrawable(R.drawable.warn);
        this.okDrawable = resources.getDrawable(R.drawable.ok);
        this.staleDrawable = resources.getDrawable(R.drawable.stale);
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("android_core_components/diagnostics_array_view");
    }

    public void onStart(ConnectedNode connectedNode) {
        connectedNode.newSubscriber(DIAGNOSTICS_AGGREGATOR_TOPIC, DiagnosticArray._TYPE).addMessageListener(new MessageListener<DiagnosticArray>() {
            public void onNewMessage(DiagnosticArray message) {
                final List<DiagnosticStatus> diagnosticStatusMessages = message.getStatus();
                DiagnosticsArrayView.this.post(new Runnable() {
                    public void run() {
                        DiagnosticsArrayView.this.removeAllViews();
                        for (DiagnosticStatus diagnosticStatusMessage : diagnosticStatusMessages) {
                            Button button = new Button(DiagnosticsArrayView.this.getContext());
                            button.setText(diagnosticStatusMessage.getName());
                            if (diagnosticStatusMessage.getLevel() == 3) {
                                button.setCompoundDrawablesWithIntrinsicBounds(DiagnosticsArrayView.this.staleDrawable, (Drawable) null, (Drawable) null, (Drawable) null);
                            } else if (diagnosticStatusMessage.getLevel() == 2) {
                                button.setCompoundDrawablesWithIntrinsicBounds(DiagnosticsArrayView.this.errorDrawable, (Drawable) null, (Drawable) null, (Drawable) null);
                            } else if (diagnosticStatusMessage.getLevel() == 1) {
                                button.setCompoundDrawablesWithIntrinsicBounds(DiagnosticsArrayView.this.warningDrawable, (Drawable) null, (Drawable) null, (Drawable) null);
                            } else {
                                button.setCompoundDrawablesWithIntrinsicBounds(DiagnosticsArrayView.this.okDrawable, (Drawable) null, (Drawable) null, (Drawable) null);
                            }
                            DiagnosticsArrayView.this.addView(button);
                        }
                    }
                });
                DiagnosticsArrayView.this.postInvalidate();
            }
        });
    }

    public void onError(Node node, Throwable throwable) {
    }

    public void onShutdown(Node node) {
    }

    public void onShutdownComplete(Node node) {
    }
}
