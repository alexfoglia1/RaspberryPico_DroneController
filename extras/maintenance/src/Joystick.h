#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <fcntl.h>
#include <stdio.h>

#ifndef WIN32
#include <unistd.h>
#endif

#include <stdint.h>
#include <limits>
#include <string.h>
#include <QThread>
#include <SDL.h>
#include <QMutex>


typedef struct
{
    Sint16 l3_vaxis;
    Sint16 r3_haxis;
    Sint16 r3_vaxis;
} js_control_packet;


enum class js_button : quint8
{
    CROSS,
    SQUARE,
    CIRCLE,
    TRIANGLE,
    UP,
    DOWN,
    LEFT,
    RIGHT
};

class Joystick : public QThread
{
    Q_OBJECT

public:

    enum class js_thread_state_t
    {
        IDLE,
        OPERATIVE,
        EXIT
    };

    Joystick();
    void updateState(js_thread_state_t newState);
    js_thread_state_t acquireState();
    int remapJsValue(int value, int min, int max);

public slots:
    void onApplicationQuit();

signals:
    void jsConnected(bool connected);
    void jsBtnPressed(js_button btn);
    void jsControl(js_control_packet controlPacket);
    void jsThreadExit();

protected:
    void run() override;

private:
    int JOY_DEAD_CENTER_ZONE;
    int L3_VERTICAL_AXIS;
    int R3_HORIZONTAL_AXIS;
    int R3_VERTICAL_AXIS;
    int CROSS_BUTTON;
    int SQUARE_BUTTON;
    int CIRCLE_BUTTON;
    int TRIANGLE_BUTTON;

    SDL_Joystick *js;
    js_control_packet _ctrlPacket;
    js_thread_state_t act_state;
    QMutex statesMutex;

    void checkJsEvent(SDL_Event event);
    bool initJoystick();
    Sint16 deadZone(int raw_value);
    int mapValue(int value, int from_min, int from_max, int to_min, int to_max);
};


Q_DECLARE_METATYPE(js_control_packet);
Q_DECLARE_METATYPE(js_button);

#endif //JOYSTICK_H
