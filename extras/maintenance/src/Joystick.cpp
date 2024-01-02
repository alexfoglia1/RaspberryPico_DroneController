#include "Joystick.h"


Joystick::Joystick()
{
    js = nullptr;
    act_state = js_thread_state_t::IDLE;
    _ctrlPacket.l3_vaxis = 0;
    _ctrlPacket.r3_haxis = 0;
    _ctrlPacket.r3_vaxis = 0;

    JOY_DEAD_CENTER_ZONE = 2000;
    R3_HORIZONTAL_AXIS = 2;
    R3_VERTICAL_AXIS = 3;
    L3_VERTICAL_AXIS = 1;
    CROSS_BUTTON = 0;
    SQUARE_BUTTON = 2;
    CIRCLE_BUTTON = 1;
    TRIANGLE_BUTTON = 3;
}


void Joystick::updateState(js_thread_state_t newState)
{
    statesMutex.lock();
    act_state = newState;
    statesMutex.unlock();
}


Joystick::js_thread_state_t Joystick::acquireState()
{
    statesMutex.lock();
    js_thread_state_t __act_state__ = act_state;
    statesMutex.unlock();

    return __act_state__;
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
            if (L3_VERTICAL_AXIS == event.jaxis.axis)
            {
                axisRaw = event.jaxis.value;
                _ctrlPacket.l3_vaxis = deadZone(event.jaxis.value);
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

            break;
        }
        case  SDL_JOYBUTTONUP:
        {
            if (event.jbutton.button == CIRCLE_BUTTON) emit jsBtnPressed(js_button::CIRCLE);
            else if (event.jbutton.button == SQUARE_BUTTON) emit jsBtnPressed(js_button::SQUARE);
            else if (event.jbutton.button == CROSS_BUTTON) emit jsBtnPressed(js_button::CROSS);
            else if (event.jbutton.button == TRIANGLE_BUTTON) emit jsBtnPressed(js_button::TRIANGLE);

            break;
        }
        case SDL_JOYHATMOTION:
        {
            switch(event.jhat.value)
            {

            case SDL_HAT_UP: emit jsBtnPressed(js_button::UP); break;

            case SDL_HAT_LEFT: emit jsBtnPressed(js_button::LEFT); break;

            case SDL_HAT_RIGHT: emit jsBtnPressed(js_button::RIGHT); break;

            case SDL_HAT_DOWN: emit jsBtnPressed(js_button::DOWN); break;
            }

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
        __act_state__ = acquireState();

        switch (act_state)
        {
            case js_thread_state_t::IDLE:
            {
                _ctrlPacket.l3_vaxis = 0;
                _ctrlPacket.r3_haxis = 0;
                _ctrlPacket.r3_vaxis = 0;

                if (initJoystick())
                {
                    updateState(js_thread_state_t::OPERATIVE);
                    emit jsConnected(true);
                }
                else
                {
                    emit jsConnected(false);
                }
            }
            break;
            case js_thread_state_t::OPERATIVE:
            {
                SDL_Event event;
                if (SDL_NumJoysticks() == 0)
                {
                    updateState(js_thread_state_t::IDLE);
                }
                else
                {
                    while (SDL_PollEvent(&event))
                    {
                        Sint16 l3_vaxis = _ctrlPacket.l3_vaxis;
                        Sint16 r3_haxis = _ctrlPacket.r3_haxis;
                        Sint16 r3_vaxis = _ctrlPacket.r3_vaxis;

                        checkJsEvent(event);

                        if (_ctrlPacket.l3_vaxis != l3_vaxis || _ctrlPacket.r3_haxis != r3_haxis || _ctrlPacket.r3_vaxis != r3_vaxis)
                        {
                            emit jsControl(_ctrlPacket);
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
        msleep(10);
    }

    emit jsThreadExit();
}

void Joystick::onApplicationQuit()
{
    updateState(js_thread_state_t::EXIT);
}
