package org.frogforce503.lib.auto.actions.base;

import java.util.concurrent.locks.Condition;
import java.util.function.BooleanSupplier;

public class ConditionalAction implements Action {

    private final BooleanSupplier condition;
    private final Action ifTrue, ifFalse;

    private boolean result = false;
    
    public ConditionalAction(BooleanSupplier condition, Action ifTrue, Action ifFalse) {
        this.condition = condition;
        this.ifTrue = ifTrue;
        this.ifFalse = ifFalse;
    }

    public ConditionalAction(BooleanSupplier condition, Action ifTrue) {
        this(condition, ifTrue, new NoopAction());
    }


    @Override
    public void start() {
        this.result = condition.getAsBoolean();

        if (this.result)
            ifTrue.start();
        else
            ifFalse.start();
    }

    @Override
    public void update() { 
        if (this.result)
            ifTrue.update();
        else
            ifFalse.update();
    }

    @Override
    public boolean isFinished() {
        return this.result ? ifTrue.isFinished() : ifFalse.isFinished();
    }

    @Override
    public void done() { }
    
}