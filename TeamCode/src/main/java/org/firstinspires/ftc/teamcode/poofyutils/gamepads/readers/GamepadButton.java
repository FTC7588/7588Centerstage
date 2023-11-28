package org.firstinspires.ftc.teamcode.poofyutils.gamepads.readers;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.button.Button;
import org.firstinspires.ftc.teamcode.poofyutils.gamepads.GamepadKeys;

import org.firstinspires.ftc.teamcode.poofyutils.gamepads.PoofyGamepadEx;

public class GamepadButton extends Button {

    private final PoofyGamepadEx m_gamepad;
    private final GamepadKeys.Button[] m_buttons;

    /**
     * Creates a gamepad button for triggering commands.
     *
     * @param gamepad the gamepad with the buttons
     * @param buttons the specified buttons
     */
    public GamepadButton(PoofyGamepadEx gamepad, @NonNull GamepadKeys.Button... buttons) {
        m_gamepad = gamepad;
        m_buttons = buttons;
    }

    /**
     * Gets the value of the joystick button.
     *
     * @return The value of the joystick button
     */
    @Override
    public boolean get() {
        boolean res = true;
        for (GamepadKeys.Button button : m_buttons)
            res = res && m_gamepad.getButton(button);
        return res;
    }

}
