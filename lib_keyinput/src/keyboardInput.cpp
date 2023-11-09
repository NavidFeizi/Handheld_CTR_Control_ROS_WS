
#include "keyboardInput.hpp"

KeyboardInput::KeyboardInput(std::vector<double> &target, std::vector<double> &coefficients, int *count)
    : target(target), coefficients(coefficients), shiftQPressed(false)
{
    if (count != NULL)
    {
        counter = count;
    }
    display = XOpenDisplay(NULL);
    lastPressTime = std::chrono::high_resolution_clock::now();
}

KeyboardInput::~KeyboardInput()
{
    // if (display)
    // {
    //     XCloseDisplay(display);
    // }

    if (inputThread.joinable())
    {
        inputThread.join();
    }
}

void KeyboardInput::startInputThread()
{
    if (display)
    {
        inputThread = std::thread(&KeyboardInput::inputLoop, this);
    }
}

void KeyboardInput::reset()
{
    target = {0.0, 0.0, 0.0, 0.0};
}

bool KeyboardInput::exit()
{
    return shiftQPressed;
}

void KeyboardInput::inputLoop()
{
    if (!display)
    {
        return;
    }

    Window root = DefaultRootWindow(display);
    XEvent event;

    XGrabKeyboard(display, root, True, GrabModeAsync, GrabModeAsync, CurrentTime);

    while (true)
    {
        XNextEvent(display, &event);
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto timeDiff = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastPressTime);

        if (timeDiff.count() >= 50)
        {

            if (event.type == KeyPress)
            {
                KeySym key = XKeycodeToKeysym(display, event.xkey.keycode, 0);

                if (key == XK_Right)
                {
                    target[0] += coefficients[0];
                }
                else if (key == XK_Left)
                {
                    target[0] -= coefficients[0];
                }
                else if (key == XK_Up)
                {
                    target[1] += coefficients[1];
                }
                else if (key == XK_Down)
                {
                    target[1] -= coefficients[1];
                }
                else if (key == XK_d)
                {
                    target[2] += coefficients[2];
                }
                else if (key == XK_a)
                {
                    target[2] -= coefficients[2];
                }
                else if (key == XK_w)
                {
                    target[3] += coefficients[3];
                }
                else if (key == XK_s)
                {
                    target[3] -= coefficients[3];
                }
                else if (key == XK_0)
                {
                    target = {0.0, 0.0, 0.0, 0.0};
                }
                else if (key == XK_KP_Subtract && counter != NULL)
                {
                    *counter -= 5;
                }
                else if (key == XK_KP_Add && counter != NULL)
                {
                    *counter += 5;
                }
                else if (key == XK_q || key == XK_Q)
                {
                    if (event.xkey.state & ShiftMask)
                    {
                        // Shift+Q is pressed
                        shiftQPressed = true;
                        if (display)
                        {
                            XCloseDisplay(display);
                        }
                        break; // Exit the input loop
                    }
                }
                lastPressTime = std::chrono::high_resolution_clock::now();
            }
        }
    }

    XUngrabKeyboard(display, CurrentTime);
}
