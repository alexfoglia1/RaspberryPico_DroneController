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
    Sint16 r2_axis;
    Sint16 r3_haxis;
    Sint16 r3_vaxis;
} js_control_packet;


enum class js_button : quint8
{
    CROSS,
    SQUARE,
    CIRCLE,
    TRIANGLE,
    L1,
    UP,
    DOWN,
    LEFT,
    RIGHT
};

enum class js_axis : quint8
{
    L2_AXIS,
    L3_X_AXIS,
    L3_Y_AXIS,
    R2_AXIS,
    R3_X_AXIS,
    R3_Y_AXIS
};

class Joystick : public QThread
{
    Q_OBJECT

public:
    static int remapJsValue(int value, int min, int max);
    enum class js_thread_state_t
    {
        IDLE,
        OPERATIVE,
        EXIT
    };

    Joystick();

signals:
    void jsConnected(bool connected);
    void jsBtnPressed(js_button btn);
    void jsAxisMoved(js_axis, qint16 val);
    void jsControl(js_control_packet controlPacket);
    void jsThreadExit();

protected:
    void run() override;

private:
    static int mapValue(int value, int from_min, int from_max, int to_min, int to_max);

    int JOY_DEAD_CENTER_ZONE;
    int L3_VERTICAL_AXIS;
    int R3_HORIZONTAL_AXIS;
    int R3_VERTICAL_AXIS;
    int R2_AXIS;
    int CROSS_BUTTON;
    int SQUARE_BUTTON;
    int CIRCLE_BUTTON;
    int TRIANGLE_BUTTON;
    int L1_BUTTON;
    int UP_ARROW;
    int RIGHT_ARROW;
    int DOWN_ARROW;
    int LEFT_ARROW;

    SDL_Joystick *js;
    js_control_packet _ctrlPacket;
    js_thread_state_t act_state;
    Sint16 _l3_vaxis;

    void checkJsEvent(SDL_Event event);
    bool initJoystick();
    Sint16 deadZone(int raw_value);
};


Q_DECLARE_METATYPE(js_control_packet);
Q_DECLARE_METATYPE(js_button);
Q_DECLARE_METATYPE(js_axis);

#endif //JOYSTICK_H
