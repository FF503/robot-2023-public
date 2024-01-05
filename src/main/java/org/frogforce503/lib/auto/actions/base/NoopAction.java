package org.frogforce503.lib.auto.actions.base;

public class NoopAction extends InlineAction {
    public NoopAction() {
        super(() -> {});
    }
}
