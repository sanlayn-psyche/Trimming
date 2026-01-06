#pragma once
#include <fstream>

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

    float m_size_frac; //图像的长宽比，用于调整窗口

    Patch* m_patchNow{nullptr};
    std::ifstream m_ifs;


public:
    
    bool m_showCurve = true;
    bool m_showCover = false;
    bool m_showKdNode = false;
    bool m_showBDB1 = false;
    bool m_showClamp = false;
    bool m_showSample = false;
    int m_patchId = 0;

    void m_switchShowSample(bool flag);
    
    void m_switchShowCover(bool flag);

    void m_switchShowCurve(bool flag);
   
    void m_switchShowKdNode(bool flag);
   
    void m_switchShowBDB1(bool flag);

    void m_switchShowClamp(bool flag);

    void m_switchPatch(int id);

    TrimmingCurveViewer(QMainWindow* parent = nullptr, int w = 700, int h = 700);

    ~TrimmingCurveViewer();

    QPointF m_getOffset2Main();
private:
    QSize minimumSizeHint() const override;
      
    QSize sizeHint() const override;
    
    void m_paint_general();
    
    void m_paint_test();

    void paintEvent(QPaintEvent* event) override;
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

    void set_mainWidget(int w, int h);

    void m_switchPath();
};
