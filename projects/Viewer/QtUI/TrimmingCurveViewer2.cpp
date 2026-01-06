#pragma once

#include "TrimmingCurveViewer2.h"
#include "Patch.h"
#include "NurbsCurve.h"
#include "TrimLoop.h"
#include "SubCurve.h"
#include "TrimManager.h"
#include "output.h"
#include "SpaceNode.h"
#include "CurveSet.h"

void TrimmingCurveViewer::m_switchShowSample(bool flag)
{
    m_showSample = flag;
    update();
}

void TrimmingCurveViewer::m_switchShowCover(bool flag)
{
    m_showCover = flag;
    update();
}

void TrimmingCurveViewer::m_switchShowCurve(bool flag)
{
    m_showCurve = flag;
    update();
}

void TrimmingCurveViewer::m_switchShowKdNode(bool flag)
{
    m_showKdNode = flag;
    update();
}

void TrimmingCurveViewer::m_switchShowBDB1(bool flag)
{
    m_showBDB1 = flag;
    update();

}

void TrimmingCurveViewer::m_switchShowClamp(bool flag)
{
    m_showClamp = flag;
    update();
}


void TrimmingCurveViewer::m_switchPatch(int id)
{
    m_patchId = id;
    update();
}

void TrimmingCurveViewer::m_switchCurveMode(int id)
{
    m_curveMode = id;
    update();
}

void MainWindow::set_mainWidget(int w, int h)
{
    TrimmingCurveViewer* widget = new TrimmingCurveViewer(this, w, h);
    m_mainWidget = widget;
    widget->show();
    setCentralWidget((QWidget*)widget);

    QCheckBox* checkbox1 = new QCheckBox(tr("Show Cover"), m_toolBar);
    connect(checkbox1, &QAbstractButton::toggled, (TrimmingCurveViewer*)m_mainWidget, &TrimmingCurveViewer::m_switchShowCover);
    m_toolBar->addWidget(checkbox1);

    m_toolBar->addSeparator();
    m_combox_curvemode = new QComboBox(m_toolBar);
    m_combox_curvemode->addItem(tr("NURBS"), 0);
    m_combox_curvemode->addItem(tr("Bezier"), 1);
    m_combox_curvemode->addItem(tr("Mono"), 2);
    m_combox_curvemode->addItem(tr("None"), 3);
    QLabel* combox_label = new QLabel(tr("Curve Mode: "));
    combox_label->setBuddy(m_combox_curvemode);
    m_toolBar->addWidget(combox_label);
    m_toolBar->addWidget(m_combox_curvemode);
    connect(m_combox_curvemode, &QComboBox::activated, this, &MainWindow::m_switchCurveMode);


    m_toolBar->addSeparator();
    QCheckBox* checkbox3 = new QCheckBox(tr("Show Split"), m_toolBar);
    connect(checkbox3, &QAbstractButton::toggled, (TrimmingCurveViewer*)m_mainWidget, &TrimmingCurveViewer::m_switchShowKdNode);
    m_toolBar->addWidget(checkbox3);

    m_toolBar->addSeparator();
    QCheckBox* checkbox6 = new QCheckBox(tr("Show Sample"), m_toolBar);
    connect(checkbox6, &QAbstractButton::toggled, (TrimmingCurveViewer*)m_mainWidget, &TrimmingCurveViewer::m_switchShowSample);
    m_toolBar->addWidget(checkbox6);


    m_toolBar->addSeparator();
    m_pathcidSpinBox = new QSpinBox;
    m_pathcidSpinBox->setRange(0, 100000);
    m_pathcidSpinBox->setSpecialValueText(tr("0"));
    m_pathcidSpinBox->setFixedWidth(50);
    connect(m_pathcidSpinBox, &QSpinBox::valueChanged, this, &MainWindow::m_switchPath);
    QLabel *pathlabel = new QLabel(tr("Patch Id"));
    pathlabel->setBuddy(m_pathcidSpinBox);
    m_toolBar->addWidget(pathlabel);
    m_toolBar->addWidget(m_pathcidSpinBox);
}


void MainWindow::m_switchPath()
{
    ((TrimmingCurveViewer*)m_mainWidget)->m_switchPatch(m_pathcidSpinBox->value());
}

void MainWindow::m_switchCurveMode()
{
    ((TrimmingCurveViewer*)m_mainWidget)->m_switchCurveMode(m_combox_curvemode->currentIndex());
}

MainWindow::MainWindow(int w, int h):
    QMainWindow()
{
    setFixedSize(w, h);

    m_toolBar = addToolBar(QString("config"));
    set_mainWidget(w, h);
    /*
    QAction* open = new QAction(QString("open"), toolBar);
    connect(open, &QAction::triggered, this, &Window::m_open);
    toolBar->addAction(open);
    */
}

QPointF TrimmingCurveViewer::m_getOffset2Main()
{
    return geometry().topLeft() + parentWidget()->geometry().topLeft();
}

TrimmingCurveViewer::~TrimmingCurveViewer()
{

}

QSize TrimmingCurveViewer::minimumSizeHint() const
{
    return QSize(1000, 1000);
}

QSize TrimmingCurveViewer::sizeHint() const
{
    return QSize(1000, 1000);
}

void TrimmingCurveViewer::m_paint_test()
{

}

void TrimmingCurveViewer::paintEvent(QPaintEvent* event)
{
    auto s = clock();
    m_paint_general();
    auto e = clock();
    double t = static_cast<double>(e - s) / static_cast<double>(CLOCKS_PER_SEC);
}

void TrimmingCurveViewer::act_drawCurve(QPainter& pen)
{
    int indx = 0;
    double u0 = m_patchNow->m_frame.get_edge(0);
    double v0 = m_patchNow->m_frame.get_edge(2);
    double w = m_patchNow->m_frame.get_size(0);
    double h = m_patchNow->m_frame.get_size(1);

    if (m_curveMode == 0)
    {
        for (auto lf : m_patchNow->m_loops)
        {
            for (auto cv = lf->m_curves.begin(); cv != lf->m_curves.end(); cv++)
            {
                vector<Point> res = (*cv)->get_evaluate();
                act_drawCurve(pen, indx++, res, u0, v0, w, h);
            }
        }
    }
    if (m_curveMode == 1)
    {
        for (auto lf : m_patchNow->m_loops)
        {
            for (auto cv : lf->m_curves)
            {
                for (size_t i = 1; i < cv->m_curve->m_spans.size(); i++)
                {
                    vector<Point> res = cv->m_curve->get_evaluate(cv->m_curve->m_spans[i - 1], cv->m_curve->m_spans[i]);
                    act_drawCurve(pen, indx++, res, u0, v0, w, h);
                }
            }
        }
    }
    if (m_curveMode == 2)
    {
        for (auto lf : m_patchNow->m_loops)
        {
            for (auto cv : lf->m_curves)
            {
                for (size_t i = 1; i < cv->m_curve->m_monoParas.size(); i++)
                {
                    vector<Point> res = cv->m_curve->get_evaluate(cv->m_curve->m_monoParas[i - 1], cv->m_curve->m_monoParas[i]);
                    act_drawCurve(pen, indx++, res, u0, v0, w, h);
                }
            }
        }
    }
}


void TrimmingCurveViewer::act_drawCurve(QPainter& pen, int index, vector<Point> &curve, double u0, double v0, double w, double h)
{
    index = index % 8;
    pen.setPen(QPen(QColor(color_pannel[index * 3], color_pannel[index * 3 + 1], color_pannel[index * 3 + 2]), 1.2));
    pen.save();
    for (auto k = curve.begin() + 1; k != curve.end(); k++)
    {
        pen.drawLine(
            m_screen_w * (k->get_cord(0) - u0) / w,
            m_screen_h * (k->get_cord(1) - v0) / h,
            m_screen_w * ((k - 1)->get_cord(0) - u0) / w,
            m_screen_h * ((k - 1)->get_cord(1) - v0) / h);
    }
}

TrimmingCurveViewer::TrimmingCurveViewer(QMainWindow* parent, int w, int h):
    QWidget(parent)
{
    setBaseSize(w, h);
    m_if_dragging = false;
    setFixedSize(w, h);
    m_screen_h = h;
    m_screen_w = w;
    m_trimManager = new TrimManager();
    m_trimManager->init_resolve();
    m_patchId = m_trimManager->m_startId;
}

void TrimmingCurveViewer::m_paint_general()
{
    QPainter painter;
    painter.begin(this);

    painter.translate(0, 0);

    QRectF r = geometry();
    int w = r.width();
    int h = r.height();
    float coverValue = 0.0;

    vector<Point> P;
    if (m_patchNow == nullptr || m_patchNow->m_patchId != m_patchId)
    {
        delete m_patchNow;
        m_patchNow = new Patch();
        m_patchNow->m_patchId = m_patchId;
        m_trimManager->act_resolve(m_patchId, m_patchNow);
    }
    if (m_patchNow == nullptr || !m_patchNow->m_initialed)
    {
        return;
    }
    Frame fram = m_patchNow->m_frame;

    // 绘制覆盖度
    if (m_showCover)
    {
        for (int i = 0; i < w; i++)
        {
            for (int j = 0; j < h; j++)
            {
                float pos[2] = { static_cast<float>((float(i) / float(w)) * fram.get_size(0) + fram.get_edge(0)),
                                static_cast<float>((float(j) / float(h)) * fram.get_size(1) + fram.get_edge(2)) };

                coverValue = m_patchNow->get_coverage_cstyle(pos);
                //coverValue = m_patchNow->get_coverage(pos[0], pos[1]);

                if (coverValue > 0)
                {
                    painter.setPen(QColor(0, 0, 0));
                }
                else if (coverValue < 0)
                {
                    painter.setPen(QColor(255, 255, 255));
                }
                else
                {
                    painter.setPen(QColor(255, 0, 255));
                }

                painter.save();
                painter.drawPoint(i, j);
            }
        }

    }

    // 绘制kd-node
    if (m_showKdNode)
    {

        painter.setPen(QPen(Qt::blue, 1.0));
        painter.save();
        vector<double> spn;
        m_patchNow->get_splitLines(spn);
        for (int i = 0; i < spn.size() / 4; i++)
        {
            painter.drawLine(w * (spn[i * 4] - fram.get_edge(0)) / fram.get_size(0), h * (spn[i * 4 + 1] - fram.get_edge(2)) / fram.get_size(1),
                w * (spn[i * 4 + 2] - fram.get_edge(0)) / fram.get_size(0), h * (spn[i * 4 + 3] - fram.get_edge(2)) / fram.get_size(1));

        }
    }

  

    // 绘制曲线
    if (m_showCurve)
    {
        act_drawCurve(painter);
    }
    if (m_showSample)
    {
        painter.setPen(QPen(Qt::blue, 1.0));
        painter.save();
        vector<Point> seg;
        m_patchNow->get_sample(seg);
        for (auto s = seg.begin(); s != seg.end(); s += 2)
        {
            painter.drawLine(
                w * (s->get_cord(0) - fram.get_edge(0)) / fram.get_size(0),
                h * (s->get_cord(1) - fram.get_edge(2)) / fram.get_size(1),
                w * ((s + 1)->get_cord(0) - fram.get_edge(0)) / fram.get_size(0),
                h * ((s + 1)->get_cord(1) - fram.get_edge(2)) / fram.get_size(1));
        }
    }
    // 绘制曲线段的bouding box
    if (m_showBDB1)
    {
        painter.setPen(QPen(Qt::green));

        painter.save();
        vector<double> cbd;

        m_patchNow->get_curveBoud(cbd);

        for (int i = 0; i < cbd.size()/4; i++)
        {
            painter.drawRect(
                w* (cbd[i * 4] - fram.get_edge(0)) / fram.get_size(0),
                h* (cbd[i * 4 + 2] - fram.get_edge(2)) / fram.get_size(1),
                w* (cbd[i * 4 + 1] - cbd[i * 4]) / fram.get_size(0),
                h* (cbd[i * 4 + 3] - cbd[i * 4 + 2]) / fram.get_size(1));
        }
    }

    // 绘制曲线段的clamp
    if (m_showClamp)
    {
        painter.setPen(QPen(Qt::blue));
        painter.save();
        vector<double> clamp;
        m_patchNow->get_curveParabox(clamp);

        for (int i = 0; i < clamp.size() / 8; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                painter.drawLine(
                    w * (clamp[i * 8 + j * 2] - fram.get_edge(0)) / fram.get_size(0),
                    h * (clamp[i * 8 + j * 2 + 1] - fram.get_edge(2)) / fram.get_size(1),
                    w * (clamp[i * 8 + j * 2 + 2] - fram.get_edge(0)) / fram.get_size(0),
                    h * (clamp[i * 8 + j * 2 + 3] - fram.get_edge(2)) / fram.get_size(1)
                    );
            }
            painter.drawLine(
                w* (clamp[i * 8 + 6] - fram.get_edge(0)) / fram.get_size(0),
                h* (clamp[i * 8 + 7] - fram.get_edge(2)) / fram.get_size(1),
                w* (clamp[i * 8] - fram.get_edge(0)) / fram.get_size(0),
                h* (clamp[i * 8 + 1] - fram.get_edge(2)) / fram.get_size(1)
                );


        }
    }

    painter.restore();
    painter.end();
}



