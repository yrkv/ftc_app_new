package org.firstinspires.ftc.teamcode;

/**
 * Created by USER on 10/21/2017.
 */

public abstract class ButtonEvent {
    public Button button;

    public ButtonEvent(Button button) {
        this.button = button;
    }

    void onDown() {}
    void onUp() {}

    void whileDown() {}
    void whileUp() {}
}
