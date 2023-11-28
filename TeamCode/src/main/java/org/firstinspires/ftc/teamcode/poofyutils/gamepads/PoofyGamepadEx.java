package org.firstinspires.ftc.teamcode.poofyutils.gamepads;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.poofyutils.gamepads.readers.ButtonReader;
import org.firstinspires.ftc.teamcode.poofyutils.gamepads.readers.GamepadButton;
import org.firstinspires.ftc.teamcode.poofyutils.gamepads.readers.TriggerReader;

import java.util.HashMap;

public class PoofyGamepadEx {

    public Gamepad gamepad;

    private final HashMap<GamepadKeys.Button, ButtonReader> buttonReaders;
    private final HashMap<GamepadKeys.Button, GamepadButton> gamepadButtons;

    private final HashMap<GamepadKeys.Trigger, TriggerReader> triggerReaders;
    private final HashMap<GamepadKeys.Trigger, GamepadTrigger> gamepadTriggers;

    private final GamepadKeys.Button[] buttons = {
            GamepadKeys.Button.Y, GamepadKeys.Button.X, GamepadKeys.Button.A, GamepadKeys.Button.B, GamepadKeys.Button.LEFT_BUMPER, GamepadKeys.Button.RIGHT_BUMPER, GamepadKeys.Button.BACK,
            GamepadKeys.Button.START, GamepadKeys.Button.DPAD_UP, GamepadKeys.Button.DPAD_DOWN, GamepadKeys.Button.DPAD_LEFT, GamepadKeys.Button.DPAD_RIGHT,
            GamepadKeys.Button.TOUCHPAD, GamepadKeys.Button.TOUCHPAD_FINGER_1, GamepadKeys.Button.TOUCHPAD_FINGER_2,
            GamepadKeys.Button.LEFT_STICK_BUTTON, GamepadKeys.Button.RIGHT_STICK_BUTTON
    };

    private final GamepadKeys.Trigger[] triggers = {
            GamepadKeys.Trigger.LEFT_TRIGGER, GamepadKeys.Trigger.RIGHT_TRIGGER
    };

    public PoofyGamepadEx(Gamepad gamepad) {
        this.gamepad = gamepad;
        buttonReaders = new HashMap<>();
        gamepadButtons = new HashMap<>();
        triggerReaders = new HashMap<>();
        gamepadTriggers = new HashMap<>();
        for (GamepadKeys.Button button : buttons) {
            buttonReaders.put(button, new ButtonReader(this, button));
            gamepadButtons.put(button, new GamepadButton(this, button));
        }
        for (GamepadKeys.Trigger trigger : triggers) {
            triggerReaders.put(trigger, new TriggerReader(this, trigger));
            gamepadTriggers.put(trigger, new GamepadTrigger(this, trigger));
        }
    }

    public boolean getButton(GamepadKeys.Button button) {
        boolean buttonValue = false;
        switch (button) {
            case A:
                buttonValue = gamepad.a;
                break;
            case B:
                buttonValue = gamepad.b;
                break;
            case X:
                buttonValue = gamepad.x;
                break;
            case Y:
                buttonValue = gamepad.y;
                break;
            case LEFT_BUMPER:
                buttonValue = gamepad.left_bumper;
                break;
            case RIGHT_BUMPER:
                buttonValue = gamepad.right_bumper;
                break;
            case DPAD_UP:
                buttonValue = gamepad.dpad_up;
                break;
            case DPAD_DOWN:
                buttonValue = gamepad.dpad_down;
                break;
            case DPAD_LEFT:
                buttonValue = gamepad.dpad_left;
                break;
            case DPAD_RIGHT:
                buttonValue = gamepad.dpad_right;
                break;
            case BACK:
                buttonValue = gamepad.back;
                break;
            case START:
                buttonValue = gamepad.start;
                break;
            case LEFT_STICK_BUTTON:
                buttonValue = gamepad.left_stick_button;
                break;
            case RIGHT_STICK_BUTTON:
                buttonValue = gamepad.right_stick_button;
                break;
            case TOUCHPAD:
                buttonValue = gamepad.touchpad;
                break;
            case TOUCHPAD_FINGER_1:
                buttonValue = gamepad.touchpad_finger_1;
                break;
            case TOUCHPAD_FINGER_2:
                buttonValue = gamepad.touchpad_finger_2;
                break;
            default:
                buttonValue = false;
                break;
        }
        return buttonValue;
    }

    public double getTrigger(GamepadKeys.Trigger trigger) {
        double triggerValue = 0;
        switch (trigger) {
            case LEFT_TRIGGER:
                triggerValue = gamepad.left_trigger;
                break;
            case RIGHT_TRIGGER:
                triggerValue = gamepad.right_trigger;
                break;
            default:
                break;
        }
        return triggerValue;
    }

    public double getLeftY() {
        return -gamepad.left_stick_y;
    }

    public double getRightY() {
        return gamepad.right_stick_y;
    }

    public double getLeftX() {
        return gamepad.left_stick_x;
    }

    public double getRightX() {
        return gamepad.right_stick_x;
    }

    public double getTouchX() {
        return gamepad.touchpad_finger_1_x;
    }

    public double getTouchY() {
        return gamepad.touchpad_finger_1_y;
    }

    public boolean wasJustPressed(GamepadKeys.Button button) {
        return buttonReaders.get(button).wasJustPressed();
    }

    public boolean wasJustReleased(GamepadKeys.Button button) {
        return buttonReaders.get(button).wasJustReleased();
    }

    public void readButtons() {
        for (GamepadKeys.Button button : buttons) {
            buttonReaders.get(button).readValue();
        }
    }

    public boolean isDown(GamepadKeys.Button button) {
        return buttonReaders.get(button).isDown();
    }

    public boolean stateJustChanged(GamepadKeys.Button button) {
        return buttonReaders.get(button).stateJustChanged();
    }

    public GamepadButton getGamepadButton(GamepadKeys.Button button) {
        return gamepadButtons.get(button);
    }

    public boolean wasJustPressed(com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger trigger) {
        return triggerReaders.get(trigger).wasJustPressed();
    }


    public boolean wasJustReleased(com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger trigger) {
        return triggerReaders.get(trigger).wasJustReleased();
    }


    public void readTriggers() {
        for (GamepadKeys.Trigger trigger : triggers) {
            triggerReaders.get(trigger).readValue();
        }
    }


    public boolean isDown(GamepadKeys.Trigger trigger) {
        return triggerReaders.get(trigger).isDown();
    }


    public boolean stateJustChanged(GamepadKeys.Trigger trigger) {
        return triggerReaders.get(trigger).stateJustChanged();
    }


    public GamepadTrigger getGamepadTrigger(GamepadKeys.Trigger trigger) {
        return gamepadTriggers.get(trigger);
    }

}
