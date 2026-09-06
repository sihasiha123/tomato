/****************************************************************************
** Meta object code from reading C++ file 'multi_navi_goal_panel.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../../../src/putn-main/src/putn/rviz_multi_goals_manager_plugins/src/multi_navi_goal_panel.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'multi_navi_goal_panel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rviz_multi_goals_manager_plugins__MultiNaviGoalsPanel_t {
    QByteArrayData data[12];
    char stringdata0[178];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz_multi_goals_manager_plugins__MultiNaviGoalsPanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz_multi_goals_manager_plugins__MultiNaviGoalsPanel_t qt_meta_stringdata_rviz_multi_goals_manager_plugins__MultiNaviGoalsPanel = {
    {
QT_MOC_LITERAL(0, 0, 53), // "rviz_multi_goals_manager_plug..."
QT_MOC_LITERAL(1, 54, 13), // "setMaxNumGoal"
QT_MOC_LITERAL(2, 68, 0), // ""
QT_MOC_LITERAL(3, 69, 10), // "maxNumGoal"
QT_MOC_LITERAL(4, 80, 11), // "changeEvent"
QT_MOC_LITERAL(5, 92, 13), // "initPoseTable"
QT_MOC_LITERAL(6, 106, 15), // "updatePoseTable"
QT_MOC_LITERAL(7, 122, 9), // "startNavi"
QT_MOC_LITERAL(8, 132, 10), // "cancelNavi"
QT_MOC_LITERAL(9, 143, 13), // "addManualGoal"
QT_MOC_LITERAL(10, 157, 10), // "checkCycle"
QT_MOC_LITERAL(11, 168, 9) // "startSpin"

    },
    "rviz_multi_goals_manager_plugins::MultiNaviGoalsPanel\0"
    "setMaxNumGoal\0\0maxNumGoal\0changeEvent\0"
    "initPoseTable\0updatePoseTable\0startNavi\0"
    "cancelNavi\0addManualGoal\0checkCycle\0"
    "startSpin"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz_multi_goals_manager_plugins__MultiNaviGoalsPanel[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   59,    2, 0x0a /* Public */,
       4,    0,   62,    2, 0x09 /* Protected */,
       5,    0,   63,    2, 0x09 /* Protected */,
       6,    0,   64,    2, 0x09 /* Protected */,
       7,    0,   65,    2, 0x09 /* Protected */,
       8,    0,   66,    2, 0x09 /* Protected */,
       9,    0,   67,    2, 0x09 /* Protected */,
      10,    0,   68,    2, 0x09 /* Protected */,
      11,    0,   69,    2, 0x09 /* Protected */,

 // slots: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void rviz_multi_goals_manager_plugins::MultiNaviGoalsPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MultiNaviGoalsPanel *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->setMaxNumGoal((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 1: _t->changeEvent(); break;
        case 2: _t->initPoseTable(); break;
        case 3: _t->updatePoseTable(); break;
        case 4: _t->startNavi(); break;
        case 5: _t->cancelNavi(); break;
        case 6: _t->addManualGoal(); break;
        case 7: _t->checkCycle(); break;
        case 8: _t->startSpin(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject rviz_multi_goals_manager_plugins::MultiNaviGoalsPanel::staticMetaObject = { {
    &rviz::Panel::staticMetaObject,
    qt_meta_stringdata_rviz_multi_goals_manager_plugins__MultiNaviGoalsPanel.data,
    qt_meta_data_rviz_multi_goals_manager_plugins__MultiNaviGoalsPanel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *rviz_multi_goals_manager_plugins::MultiNaviGoalsPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz_multi_goals_manager_plugins::MultiNaviGoalsPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rviz_multi_goals_manager_plugins__MultiNaviGoalsPanel.stringdata0))
        return static_cast<void*>(this);
    return rviz::Panel::qt_metacast(_clname);
}

int rviz_multi_goals_manager_plugins::MultiNaviGoalsPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz::Panel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 9)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 9;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
