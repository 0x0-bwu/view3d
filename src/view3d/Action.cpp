#include "Action.h"
using namespace view;
void ActionBinding::SetMouseBinding(Qt::KeyboardModifiers modifiers, Qt::MouseButton btn, MouseAction action)
{
    SetMouseBinding(Qt::Key(0), modifiers, btn, action);
}

void ActionBinding::SetMouseBinding(Qt::Key key, Qt::KeyboardModifiers modifiers,
                                    Qt::MouseButton btn, MouseAction action)
{
    MouseBinding mb(modifiers, btn, key);
    if(action == MouseAction::NoAction)
        mouseBindings.remove(mb);
    else
        mouseBindings.insert(mb, action);

    ClickBinding cb(modifiers, btn, Qt::NoButton, false, key);
    clickBindings.remove(cb);
}

void ActionBinding::SetMouseBinding(Qt::Key key, Qt::KeyboardModifiers modifiers,
                                    Qt::MouseButton currBtn, ClickAction action,
                                    bool dc, Qt::MouseButton prevBtn)

{
    ClickBinding cb(modifiers, currBtn, prevBtn, dc, key);
    if(action == ClickAction::NoAction)
        clickBindings.remove(cb);
    else
        clickBindings.insert(cb, action);

    if((!dc) && (prevBtn == Qt::NoButton)){
        MouseBinding mb(modifiers, currBtn, key);
        mouseBindings.remove(mb);
    }
}

void ActionBinding::SetWheelBinding(Qt::KeyboardModifiers modifiers, MouseAction action)
{
    this->SetWheelBinding(Qt::Key(0), modifiers, action);
}

void ActionBinding::SetWheelBinding(Qt::Key key, Qt::KeyboardModifiers modifiers, MouseAction action)
{
    WheelBinding wb(modifiers, key);
    if(action == MouseAction::NoAction)
        wheelBindings.remove(wb);
    else
        wheelBindings.insert(wb, action);
}

void ActionBinding::SetShortcut(KeyBoardAction action, Qt::Key key)
{
    keyboardBindings[action] = key;
}

