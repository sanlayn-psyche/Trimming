#pragma once

#include <QQuickPaintedItem>
#include <QPainter>
#include "TrimManager.h"
#include "Patch.h"

const std::string ProjectPath {RootPath};

class ViewerController : public QQuickPaintedItem {
  Q_OBJECT
  Q_PROPERTY(bool showCurve READ showCurve WRITE setShowCurve NOTIFY showCurveChanged)
  Q_PROPERTY(bool showCover READ showCover WRITE setShowCover NOTIFY showCoverChanged)
  Q_PROPERTY(bool showKdNode READ showKdNode WRITE setShowKdNode NOTIFY showKdNodeChanged)
  Q_PROPERTY(bool showBDB1 READ showBDB1 WRITE setShowBDB1 NOTIFY showBDB1Changed)
  Q_PROPERTY(bool showClamp READ showClamp WRITE setShowClamp NOTIFY showClampChanged)
  Q_PROPERTY(bool showSample READ showSample WRITE setShowSample NOTIFY showSampleChanged)
  Q_PROPERTY(int patchId READ patchId WRITE setPatchId NOTIFY patchIdChanged)
  Q_PROPERTY(int curveMode READ curveMode WRITE setCurveMode NOTIFY curveModeChanged)
  QML_ELEMENT

public:
  explicit ViewerController(QQuickItem *parent = nullptr);
  ~ViewerController() override;

  void paint(QPainter *painter) override;

  // Property Getters
  bool showCurve() const { return m_showCurve; }
  bool showCover() const { return m_showCover; }
  bool showKdNode() const { return m_showKdNode; }
  bool showBDB1() const { return m_showBDB1; }
  bool showClamp() const { return m_showClamp; }
  bool showSample() const { return m_showSample; }
  int patchId() const { return m_patchId; }
  int curveMode() const { return m_curveMode; }

  // Property Setters
  void setShowCurve(bool v);
  void setShowCover(bool v);
  void setShowKdNode(bool v);
  void setShowBDB1(bool v);
  void setShowClamp(bool v);
  void setShowSample(bool v);
  void setPatchId(int v);
  void setCurveMode(int v);

  Q_INVOKABLE void performTrimming();

signals:
  void showCurveChanged();
  void showCoverChanged();
  void showKdNodeChanged();
  void showBDB1Changed();
  void showClampChanged();
  void showSampleChanged();
  void patchIdChanged();
  void curveModeChanged();

private:
  TrimManager* m_trimManager{nullptr};
  Patch* m_patchNow{nullptr};

  bool m_showCurve = true;
  bool m_showCover = false;
  bool m_showKdNode = false;
  bool m_showBDB1 = false;
  bool m_showClamp = false;
  bool m_showSample = false;
  int m_patchId = 0;
  int m_curveMode = 0;

  void act_drawCurve(QPainter &pen);
  void act_drawCurve(QPainter &pen, int index, std::vector<Point> &curve, double u0, double v0, double w, double h);
};
