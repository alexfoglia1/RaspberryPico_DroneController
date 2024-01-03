#include "Joystick.h"


Joystick::Joystick()
{
    js = nullptr;
    act_state = js_thread_state_t::IDLE;
    _ctrlPacket.r2_axis = 0;
    _ctrlPacket.r3_haxis = 0;
    _ctrlPacket.r3_vaxis = 0;
    _l3_vaxis = 0;

    JOY_DEAD_CENTER_ZONE = 2000;
    R3_HORIZONTAL_AXIS = 2;
    R3_VERTICAL_AXIS = 3;
    L3_VERTICAL_AXIS = 1;
    R2_AXIS = 7;
    CROSS_BUTTON = 0;
    SQUARE_BUTTON = 2;
    CIRCLE_BUTTON = 1;
    TRIANGLE_BUTTON = 3;
    L1_BUTTON = 9;
    UP_ARROW = 11;
    RIGHT_ARROW = 14;
    DOWN_ARROW = 12;
    LEFT_ARROW = 13;
}


int Joystick::remapJsValue(int value, int min, int max)
{
    int min_js_axis_value = -32768;
    int max_js_axis_value = 32767;

    return mapValue(value, min_js_axis_value, max_js_axis_value, min, max);
}


int Joystick::mapValue(int value, int from_min, int from_max, int to_min, int to_max)
{
    int from_span = from_max - from_min;
    double from_percentage = ((double)value - (double)from_min) / (double)from_span;

    int to_span = to_max - to_min;

    return to_min + (int)(from_percentage * to_span);
}


Sint16 Joystick::deadZone(int raw_value)
{
    if (fabsf(raw_value) < JOY_DEAD_CENTER_ZONE)
    {
        return 0;
    }
    else
    {
        int min_js_axis_value = -32768;
        int max_js_axis_value = 32767;
        int js_axis_span = max_js_axis_value - min_js_axis_value;

        int js_axis_span_minus_dcz = js_axis_span - 2 * JOY_DEAD_CENTER_ZONE;

        if (raw_value < 0)
        {
            int raw_value_plus_dcz = raw_value + JOY_DEAD_CENTER_ZONE;
            return Sint16(mapValue(raw_value_plus_dcz, min_js_axis_value + JOY_DEAD_CENTER_ZONE, 0, min_js_axis_value, 0));
        }
        else
        {
            int raw_value_minus_dcz = raw_value - JOY_DEAD_CENTER_ZONE;
            return Sint16(mapValue(raw_value_minus_dcz, 0, max_js_axis_value - JOY_DEAD_CENTER_ZONE, 0, max_js_axis_value));
        }
    }
}


bool Joystick::initJoystick()
{
    if (SDL_Init(SDL_INIT_JOYSTICK) == 0)
    {
        SDL_JoystickEventState(SDL_ENABLE);
        js = SDL_JoystickOpen(0);
    }

    return js != nullptr;
}


void Joystick::checkJsEvent(SDL_Event event)
{
    Sint16 axisRaw = 0;
    switch (event.type)
    {
        case SDL_JOYAXISMOTION:
        {
            if (R2_AXIS == event.jaxis.axis)
            {
                axisRaw = event.jaxis.value;
                _ctrlPacket.r2_axis = deadZone(event.jaxis.value);
            }
            else if (R3_VERTICAL_AXIS == event.jaxis.axis)
            {
                axisRaw = event.jaxis.value;
                _ctrlPacket.r3_vaxis = deadZone(event.jaxis.value);
            }
            else if (R3_HORIZONTAL_AXIS == event.jaxis.axis)
            {
                axisRaw = event.jaxis.value;
                _ctrlPacket.r3_haxis = deadZone(event.jaxis.value);
            }
            else if (L3_VERTICAL_AXIS == event.jaxis.axis)
            {
                axisRaw = event.jaxis.value;
                _l3_vaxis = deadZone(event.jaxis.value);
            }

            break;
        }
        case  SDL_JOYBUTTONDOWN:
        {
            if (event.jbutton.button == CIRCLE_BUTTON) emit jsBtnPressed(js_button::CIRCLE);
            else if (event.jbutton.button == SQUARE_BUTTON) emit jsBtnPressed(js_button::SQUARE);
            else if (event.jbutton.button == CROSS_BUTTON) emit jsBtnPressed(js_button::CROSS);
            else if (event.jbutton.button == TRIANGLE_BUTTON) emit jsBtnPressed(js_button::TRIANGLE);
            else if (event.jbutton.button == UP_ARROW) emit jsBtnPressed(js_button::UP);
            else if (event.jbutton.button == RIGHT_ARROW) emit jsBtnPressed(js_button::RIGHT);
            else if (event.jbutton.button == DOWN_ARROW) emit jsBtnPressed(js_button::DOWN);
            else if (event.jbutton.button == LEFT_ARROW) emit jsBtnPressed(js_button::LEFT);
            else if (event.jbutton.button == L1_BUTTON) emit jsBtnPressed(js_button::L1);
            break;
        }

        default:
            break;

    }

}

void Joystick::run()
{
    js_thread_state_t  __act_state__ = js_thread_state_t::IDLE;
    setPriority(QThread::LowestPriority);

    while (__act_state__ != js_thread_state_t::EXIT)
    {
        switch (__act_state__)
        {
            case js_thread_state_t::IDLE:
            {
                _ctrlPacket.r2_axis = 0;
                _ctrlPacket.r3_haxis = 0;
                _ctrlPacket.r3_vaxis = 0;
                _l3_vaxis = 0;

                if (initJoystick())
                {
                    __act_state__ = js_thread_state_t::OPERATIVE;
                    emit jsConnected(true);
                }
            }
            break;
            case js_thread_state_t::OPERATIVE:
            {
                SDL_Event event;
                if (SDL_NumJoysticks() == 0)
                {
                    __act_state__ = js_thread_state_t::IDLE;
                    emit jsConnected(false);
                }
                else
                {
                    while (SDL_PollEvent(&event))
                    {
                        Sint16 r2_axis = _ctrlPacket.r2_axis;
                        Sint16 r3_haxis = _ctrlPacket.r3_haxis;
                        Sint16 r3_vaxis = _ctrlPacket.r3_vaxis;
                        Sint16 l3_vaxis = _l3_vaxis;

                        checkJsEvent(event);

                        if (_ctrlPacket.r2_axis != r2_axis || _ctrlPacket.r3_haxis != r3_haxis || _ctrlPacket.r3_vaxis != r3_vaxis)
                        {
                            emit jsControl(_ctrlPacket);
                        }

                        if (_l3_vaxis != 0)
                        {
                            emit jsAxisMoved(js_axis::L3_Y_AXIS, _l3_vaxis);
                        }
                    }
                }

            }
            break;
            case js_thread_state_t::EXIT:
                break;
            default:
                break;
        }
    }

    emit jsThreadExit();
}