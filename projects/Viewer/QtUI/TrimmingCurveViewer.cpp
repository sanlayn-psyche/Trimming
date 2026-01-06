#pragma once

#include "TrimmingCurveViewer.h"
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

void MainWindow::set_mainWidget(int w, int h)
{
    TrimmingCurveViewer* widget = new TrimmingCurveViewer(this, w, h);
    m_mainWidget = widget;
    widget->show();
    setCentralWidget((QWidget*)widget);

    
    QCheckBox* checkbox1 = new QCheckBox(tr("show cover"), m_toolBar);
    connect(checkbox1, &QAbstractButton::toggled, (TrimmingCurveViewer*)m_mainWidget, &TrimmingCurveViewer::m_switchShowCover);
    m_toolBar->addWidget(checkbox1);

    QCheckBox* checkbox2 = new QCheckBox(tr("show curve"), m_toolBar);
    checkbox2->setChecked(true);
    connect(checkbox2, &QAbstractButton::toggled, (TrimmingCurveViewer*)m_mainWidget, &TrimmingCurveViewer::m_switchShowCurve);
    m_toolBar->addWidget(checkbox2);

    QCheckBox* checkbox3 = new QCheckBox(tr("show kdNode"), m_toolBar);
    connect(checkbox3, &QAbstractButton::toggled, (TrimmingCurveViewer*)m_mainWidget, &TrimmingCurveViewer::m_switchShowKdNode);
    m_toolBar->addWidget(checkbox3);
    
    QCheckBox* checkbox4 = new QCheckBox(tr("Show BDB"), m_toolBar);
    connect(checkbox4, &QAbstractButton::toggled, (TrimmingCurveViewer*)m_mainWidget, &TrimmingCurveViewer::m_switchShowBDB1);
    m_toolBar->addWidget(checkbox4);
    
    QCheckBox* checkbox5 = new QCheckBox(tr("Show Clamp"), m_toolBar);
    connect(checkbox5, &QAbstractButton::toggled, (TrimmingCurveViewer*)m_mainWidget, &TrimmingCurveViewer::m_switchShowClamp);
    m_toolBar->addWidget(checkbox5);
    
    QCheckBox* checkbox6 = new QCheckBox(tr("Show Sample"), m_toolBar);
    connect(checkbox6, &QAbstractButton::toggled, (TrimmingCurveViewer*)m_mainWidget, &TrimmingCurveViewer::m_switchShowSample);
    m_toolBar->addWidget(checkbox6);

    m_toolBar->addSeparator();

    m_pathcidSpinBox = new QSpinBox;
    m_pathcidSpinBox->setRange(0, 100000);
    m_pathcidSpinBox->setSpecialValueText(tr("0"));
    m_pathcidSpinBox->setFixedWidth(50);
    connect(m_pathcidSpinBox, &QSpinBox::valueChanged, this, &MainWindow::m_switchPath);
    QLabel* pathlabel = new QLabel(tr("Patch &Id"));
    pathlabel->setBuddy(m_pathcidSpinBox);

    m_toolBar->addWidget(pathlabel);
    m_toolBar->addWidget(m_pathcidSpinBox);
}


void MainWindow::m_switchPath()
{
    ((TrimmingCurveViewer*)m_mainWidget)->m_switchPatch(m_pathcidSpinBox->value());
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

TrimmingCurveViewer::TrimmingCurveViewer(QMainWindow* parent, int w, int h):
    QWidget(parent)
{
    setBaseSize(w, h);
    m_if_dragging = false;
    setFixedSize(w, h);
    m_trimManager = new TrimManager("D:/Project/NurbsViwer/Resource/model/gtr/model_info.json");
    m_trimManager->init_resolve();
    m_ifs.open(m_trimManager->m_inputRoot);
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
        try
        {
            m_patchNow->init_load(m_ifs, m_trimManager->m_offsetTable[m_patchId], m_trimManager->m_ifBinary);
            //throw lf_exception_unexcepted_loops(&m_patchNow->m_loops);

            m_patchNow->init_preprocess();
            m_patchNow->act_generateForest(m_patchNow->m_root);
            m_patchNow->act_postprocess();
            m_patchNow->act_generateSearchData_Forest();

  /*          float pos[2] = { 0.744998, 0.231553 };
            m_patchNow->get_coverage_cstyle(pos);*/
        }
        catch (const lf_exception& error)
        {
            std::cout << error.what() << std::endl;
#ifdef _DEBUG
            error.act_output();
#endif // _DEBUG
            m_showCover = false;
            m_showKdNode = false;
            m_showBDB1 = false;
            m_showSample = false;
            m_showClamp = false;
            m_showCurve = true;
        }
    }
    Frame fram = m_patchNow->m_frame;

    // 绘制覆盖度
    if (m_showCover)
    {
        for (int i = 0; i < w; i++)
        {
            for (int j = 0; j < h; j++) 
            {
                float pos[2] = {static_cast<float>((float(i) / float(w)) * fram.get_size(0) + fram.get_edge(0)),
                    static_cast<float>((float(j) / float(h))* fram.get_size(1) + fram.get_edge(2))};
                
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
        
        painter.setPen(QPen(Qt::blue, 1.2));
        painter.save();
        vector<double> spn;
        m_patchNow->get_splitLines(spn);
        for (int i = 0; i < spn.size()/4; i++)
        {
            painter.drawLine(w * (spn[i * 4] - fram.get_edge(0)) / fram.get_size(0), h * (spn[i * 4 + 1] - fram.get_edge(2)) / fram.get_size(1),
                w * (spn[i * 4 + 2] - fram.get_edge(0)) / fram.get_size(0), h * (spn[i * 4 + 3] - fram.get_edge(2)) / fram.get_size(1));

        }
    }
    
    if (m_showSample)
    {
        painter.setPen(QPen(Qt::blue, 1.2));
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
        }    }

    // 绘制曲线
    if (m_showCurve)
    {
        painter.setPen(QPen(Qt::red, 0.6));
        painter.save();
  
        for (auto lf: m_patchNow->m_loops)
        {
            for (auto cv = lf->m_curves.begin(); cv != lf->m_curves.end(); cv++)
            {
                vector<Point> res = (*cv)->get_evaluate();
                
                for (auto k = res.begin() + 1; k != res.end(); k++)
                {
                    painter.drawLine(
                        w * (k->get_cord(0) - fram.get_edge(0)) / fram.get_size(0),
                        h * (k->get_cord(1) - fram.get_edge(2)) / fram.get_size(1),
                        w * ((k - 1)->get_cord(0) - fram.get_edge(0)) / fram.get_size(0),
                        h * ((k - 1)->get_cord(1) - fram.get_edge(2)) / fram.get_size(1));
                }
            }
             
                 
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



