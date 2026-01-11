#include "ViewerController.h"
#include "Patch.h"
#include "NurbsCurve.h"
#include "TrimLoop.h"
#include "SubCurve.h"
#include "TrimManager.h"
#include "output.h"
#include "SpaceNode.h"
#include "CurveSet.h"
#include <QTime>

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

ViewerController::ViewerController(QQuickItem *parent) : QQuickPaintedItem(parent) {

    std::string config = ProjectPath + "/config.json";
    m_trimManager = new TrimManager(config.c_str());
    // Ensure we can initialize even if config is relative
    try {
        m_trimManager->init_resolve();
        m_patchId = m_trimManager->m_startId;
        m_trimManager->m_patch_prop.m_id = m_patchId;
        m_patchNow = new Patch(m_trimManager->m_patch_prop);
        m_trimManager->act_resolve(m_patchId, m_patchNow);
    } catch (...) {
        qDebug() << "Failed to initialize TrimManager or resolve initial patch";
    }
}

ViewerController::~ViewerController() {
    if (m_trimManager) delete m_trimManager;
    if (m_patchNow) delete m_patchNow;
}

void ViewerController::performTrimming() {
    // Placeholder as logical trimming is mostly visualization in this viewer
    qDebug() << "Perform Trimming Clicked";
}

// Property Setters
void ViewerController::setShowCurve(bool v) {
    if (m_showCurve == v) return;
    m_showCurve = v;
    emit showCurveChanged();
    update();
}

void ViewerController::setShowCover(bool v) {
    if (m_showCover == v) return;
    m_showCover = v;
    emit showCoverChanged();
    update();
}

void ViewerController::setShowKdNode(bool v) {
    if (m_showKdNode == v) return;
    m_showKdNode = v;
    emit showKdNodeChanged();
    update();
}

void ViewerController::setShowBDB1(bool v) {
    if (m_showBDB1 == v) return;
    m_showBDB1 = v;
    emit showBDB1Changed();
    update();
}

void ViewerController::setShowClamp(bool v) {
    if (m_showClamp == v) return;
    m_showClamp = v;
    emit showClampChanged();
    update();
}

void ViewerController::setShowSample(bool v) {
    if (m_showSample == v) return;
    m_showSample = v;
    emit showSampleChanged();
    update();
}

void ViewerController::setPatchId(int v) {
    if (m_patchId == v) return;
    m_patchId = v;
    
    // Logic to reload patch
    if (m_trimManager) {
        if (m_patchNow) delete m_patchNow;
        m_trimManager->m_patch_prop.m_id = m_patchId;
        m_patchNow = new Patch(m_trimManager->m_patch_prop);
        try {
            m_trimManager->act_resolve(m_patchId, m_patchNow);
        } catch(...) {
             qDebug() << "Error resolving patch" << m_patchId;
        }
    }

    emit patchIdChanged();
    update();
}

void ViewerController::setCurveMode(int v) {
    if (m_curveMode == v) return;
    m_curveMode = v;
    emit curveModeChanged();
    update();
}


void ViewerController::paint(QPainter* painter)
{
    QRectF r = boundingRect();
    int w = r.width();
    int h = r.height();
    float coverValue = 0.0;

    if (!m_patchNow || !m_patchNow->m_initialed) return;

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

                if (coverValue > 0)
                {
                    painter->setPen(QColor(0, 0, 0));
                }
                else if (coverValue < 0)
                {
                    painter->setPen(QColor(255, 255, 255));
                }
                else
                {
                    painter->setPen(QColor(255, 0, 255));
                }

                painter->drawPoint(i, j);
            }
        }

    }

    // 绘制kd-node
    if (m_showKdNode)
    {
        painter->setPen(QPen(Qt::blue, 1.0));
        painter->save();
        vector<double> spn;
        m_patchNow->get_splitLines(spn);
        for (int i = 0; i < spn.size() / 4; i++)
        {
            painter->drawLine(w * (spn[i * 4] - fram.get_edge(0)) / fram.get_size(0), h * (spn[i * 4 + 1] - fram.get_edge(2)) / fram.get_size(1),
                w * (spn[i * 4 + 2] - fram.get_edge(0)) / fram.get_size(0), h * (spn[i * 4 + 3] - fram.get_edge(2)) / fram.get_size(1));

        }
        painter->restore();
    }

    // 绘制曲线
    if (m_showCurve)
    {
        act_drawCurve(*painter);
    }
    
    if (m_showSample)
    {
        painter->setPen(QPen(Qt::blue, 1.0));
        painter->save();
        vector<Point> seg;
        m_patchNow->get_sample(seg);
        for (auto s = seg.begin(); s != seg.end(); s += 2)
        {
            painter->drawLine(
                w * (s->get_cord(0) - fram.get_edge(0)) / fram.get_size(0),
                h * (s->get_cord(1) - fram.get_edge(2)) / fram.get_size(1),
                w * ((s + 1)->get_cord(0) - fram.get_edge(0)) / fram.get_size(0),
                h * ((s + 1)->get_cord(1) - fram.get_edge(2)) / fram.get_size(1));
        }
        painter->restore();
    }

    // 绘制曲线段的bouding box
    if (m_showBDB1)
    {
        painter->setPen(QPen(Qt::green));

        painter->save();
        vector<double> cbd;

        m_patchNow->get_curveBoud(cbd);

        for (int i = 0; i < cbd.size()/4; i++)
        {
            painter->drawRect(
                w* (cbd[i * 4] - fram.get_edge(0)) / fram.get_size(0),
                h* (cbd[i * 4 + 2] - fram.get_edge(2)) / fram.get_size(1),
                w* (cbd[i * 4 + 1] - cbd[i * 4]) / fram.get_size(0),
                h* (cbd[i * 4 + 3] - cbd[i * 4 + 2]) / fram.get_size(1));
        }
        painter->restore();
    }

    // 绘制曲线段的clamp
    if (m_showClamp)
    {
        painter->setPen(QPen(Qt::blue));
        painter->save();
        vector<double> clamp;
        m_patchNow->get_curveParabox(clamp);

        for (int i = 0; i < clamp.size() / 8; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                painter->drawLine(
                    w * (clamp[i * 8 + j * 2] - fram.get_edge(0)) / fram.get_size(0),
                    h * (clamp[i * 8 + j * 2 + 1] - fram.get_edge(2)) / fram.get_size(1),
                    w * (clamp[i * 8 + j * 2 + 2] - fram.get_edge(0)) / fram.get_size(0),
                    h * (clamp[i * 8 + j * 2 + 3] - fram.get_edge(2)) / fram.get_size(1)
                    );
            }
            painter->drawLine(
                w* (clamp[i * 8 + 6] - fram.get_edge(0)) / fram.get_size(0),
                h* (clamp[i * 8 + 7] - fram.get_edge(2)) / fram.get_size(1),
                w* (clamp[i * 8] - fram.get_edge(0)) / fram.get_size(0),
                h* (clamp[i * 8 + 1] - fram.get_edge(2)) / fram.get_size(1)
                );
        }
        painter->restore();
    }
}

void ViewerController::act_drawCurve(QPainter& pen)
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
                vector<Point> res = (*cv)->get_evaluateAll();
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
                for (size_t i = 1; i < cv->m_spans.size(); i++)
                {
                  
                    vector<Point> res = cv->get_evaluate(cv->m_spans[i - 1], cv->m_spans[i]);
                    //ot::print(res, OUTPUT_ROOT + "/data/bezier_"+ std::to_string(indx) + ".txt");
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
                for (size_t i = 1; i < cv->m_monoParas.size(); i++)
                {
                    vector<Point> res = cv->get_evaluate(cv->m_monoParas[i - 1], cv->m_monoParas[i]);
                    act_drawCurve(pen, indx++, res, u0, v0, w, h);
                }
            }
        }
    }
}


void ViewerController::act_drawCurve(QPainter& pen, int index, vector<Point> &curve, double u0, double v0, double w, double h)
{
    QRectF r = boundingRect();
    int sw = r.width();
    int sh = r.height();

    index = index % 8;
    pen.setPen(QPen(QColor(color_pannel[index * 3], color_pannel[index * 3 + 1], color_pannel[index * 3 + 2]), 1.2));
    // pen.save(); // Not necessary to save here if we don't change global state drastically
    for (auto k = curve.begin() + 1; k != curve.end(); k++)
    {
        pen.drawLine(
            sw * (k->get_cord(0) - u0) / w,
            sh * (k->get_cord(1) - v0) / h,
            sw * ((k - 1)->get_cord(0) - u0) / w,
            sh * ((k - 1)->get_cord(1) - v0) / h);
    }
}
