package org.firstinspires.ftc.teamcode;

/**
 * Created by USER on 10/21/2017.
 */

public abstract class ButtonEvent {
    public BUTTON button;

    public ButtonEvent(BUTTON button) {
        this.button = button;
    }

    void onDown() {}
    void onUp() {}

    void whileDown() {}
    void whileUp() {}
}
