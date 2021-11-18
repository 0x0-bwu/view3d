#ifndef VIEW_ACTION_H
#define VIEW_ACTION_H
#include "Config.h"
#include <QObject>
#include <QMap>
namespace view {

enum class ClickAction { NoAction, Select };

enum class MouseAction { NoAction,
                         Rotate,
                         Zoom,
                         Translate,
                         ScreenTranslate
                       };

enum class KeyBoardAction { ShowAxis,
                            ShowGrid,
                            ShowInfo,
                            FitAll,
                            ViewTop,
                            ViewBot,
                            ViewFront,
                            ViewEnd,
                            ViewLeft,
                            ViewRight,
                            NextAction,
                            PrevAction,
                            TestAction,
                            UpdateAction
                          };

struct MouseBinding
{
    const Qt::KeyboardModifiers modifiers;
    const Qt::MouseButton button;
    const Qt::Key key;

    MouseBinding(Qt::KeyboardModifiers m, Qt::MouseButton b, Qt::Key k) : modifiers(m), button(b), key(k) {}

    bool operator< (const MouseBinding & mb) const {
      if (key != mb.key)
        return key < mb.key;
      if (modifiers != mb.modifiers)
        return modifiers < mb.modifiers;
      return button < mb.button;
    }

};

struct WheelBinding
{
    const Qt::KeyboardModifiers modifiers;
    const Qt::Key key;

    WheelBinding(Qt::KeyboardModifiers m, Qt::Key k) : modifiers(m), key(k) {}

    bool operator< (const WheelBinding & wb) const {
      if (key != wb.key)
        return key < wb.key;
      return modifiers < wb.modifiers;
    }
};

struct ClickBinding
{
    const Qt::KeyboardModifiers modifiers;
    const Qt::MouseButton currBtn;
    const Qt::MouseButton prevBtn;
    const bool doubleClick;
    const Qt::Key key;

    ClickBinding(Qt::KeyboardModifiers m, Qt::MouseButton curr, Qt::MouseButton prev, bool dc, Qt::Key k)
        : modifiers(m), currBtn(curr), prevBtn(prev), doubleClick(dc), key(k) {}

    bool operator<(const ClickBinding & cb) const {
      if (key != cb.key)
        return key < cb.key;
      if (prevBtn != cb.prevBtn)
        return prevBtn < cb.prevBtn;
      if (modifiers != cb.modifiers)
        return modifiers < cb.modifiers;
      if (currBtn != cb.currBtn)
        return currBtn < cb.currBtn;
      return doubleClick != cb.doubleClick;
    }
};

struct ActionBinding
{
    QMap<MouseBinding, MouseAction> mouseBindings;
    QMap<WheelBinding, MouseAction> wheelBindings;
    QMap<ClickBinding, ClickAction> clickBindings;
    QMap<KeyBoardAction,Qt::Key> keyboardBindings;

    void SetMouseBinding(Qt::KeyboardModifiers modifiers, Qt::MouseButton btn, MouseAction action);

    void SetMouseBinding(Qt::Key key, Qt::KeyboardModifiers modifiers, Qt::MouseButton btn, MouseAction action);

    void SetMouseBinding(Qt::Key key, Qt::KeyboardModifiers modifiers,
                         Qt::MouseButton currBtn, ClickAction action,
                         bool dc = false, Qt::MouseButton prevBtn = Qt::NoButton);

    void SetWheelBinding(Qt::KeyboardModifiers modifiers, MouseAction action);

    void SetWheelBinding(Qt::Key key, Qt::KeyboardModifiers modifiers, MouseAction action);

    void SetShortcut(KeyBoardAction action, Qt::Key key);
};
}//namespace view
#endif//VIEW_ACTION_H
