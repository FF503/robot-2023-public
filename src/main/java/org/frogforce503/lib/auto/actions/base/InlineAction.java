package org.frogforce503.lib.auto.actions.base;

public class InlineAction implements Action {

    private final Runnable func;
    public InlineAction(Runnable lambda) {
        this.func = lambda;
    }


    @Override
    public void start() {
        this.func.run();
    }

    @Override
    public void update() {  }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() { }
    
}
