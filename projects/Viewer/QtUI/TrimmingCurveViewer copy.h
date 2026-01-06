#pragma once
#include <fstream>
#include "LFPoint.h"


#if defined(__APPLE__)
#include <QtWidgets/qwidget.h>
#include <qpainter.h>
#include <QtWidgets/qapplication.h>
#include <QtWidgets/qwidget.h>
#include <QtWidgets/qtoolbar.h>
#include <QMouseEvent>
#include <QtWidgets/qmainwindow.h>
#include <QtWidgets/qaction.h>
#include <qstring.h>
#include <QtWidgets/qspinbox.h>
#include <QtWidgets/qfiledialog.h>
#include <QtWidgets/qcheckbox.h>
#include <QtWidgets/qlabel.h>
#endif

#if defined(_WIN32) || defined(_WIN64)
#include <QtWidgets>
#include <QToolBar>
#include <qmainwindow.h>
#include <QAction>
#include <qcheckbox.h>
#include <QString>
#include <qwidget.h>
#include <QMouseEvent>
#include <QPainter>
#endif
class TrimManager;
class Patch;


class TrimmingCurveViewer: public QWidget
{
    Q_OBJECT
private:
    bool m_isNull{true};
    TrimManager* m_trimManager{nullptr};

    bool m_if_dragging = false;
    bool m_ifPress = false;
    float m_timeDuration = 0.0;

    int m_screen_w = 0 , m_screen_h = 0;

    float m_size_frac; //图像的长宽比，用于调整窗口

    Patch* m_patchNow{nullptr};
public:
    bool m_showCurve = true;
    bool m_showCover = false;
    bool m_showKdNode = false;
    bool m_showBDB1 = false;
    bool m_showClamp = false;
    bool m_showSample = false;
    int m_patchId = 0;
    int m_curveMode = 0;

    void m_switchShowSample(bool flag);

    void m_switchShowCover(bool flag);

    void m_switchShowCurve(bool flag);

    void m_switchShowKdNode(bool flag);

    void m_switchShowBDB1(bool flag);

    void m_switchShowClamp(bool flag);

    void m_switchPatch(int id);

    void m_switchCurveMode(int id);

    TrimmingCurveViewer(QMainWindow* parent = nullptr, int w = 700, int h = 700);

    ~TrimmingCurveViewer();

    QPointF m_getOffset2Main();
private:
    QSize minimumSizeHint() const override;

    QSize sizeHint() const override;

    void m_paint_general();

    void m_paint_test();

    void paintEvent(QPaintEvent* event) override;

    void act_drawCurve(QPainter &pen);
    void act_drawCurve(QPainter &pen, int index, vector<Point> &curve, double u0, double v0, double w, double h);
};

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:

    void* m_mainWidget = nullptr;
    QToolBar* m_toolBar = nullptr;

    MainWindow(int w = 1000, int h = 1000);

    ~MainWindow()
    {

    }

    QSpinBox* m_pathcidSpinBox = nullptr;
    QComboBox* m_combox_curvemode = nullptr;

    void set_mainWidget(int w, int h);

    void m_switchPath();
    void m_switchCurveMode();
};


const int color_pannel[24] =
{
     56,   222,    68,
    20,    22,    38,
   113,   102,    35,
    28,    67,   222,
   146,   205,   148,
     2,   111,   141,
   198,   233,    37,
   209,    47,   218
};