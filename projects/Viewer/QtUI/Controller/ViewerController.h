#pragma once

#include <QDebug>
#include <QObject>
#include <QString>


class ViewerController : public QObject {
  Q_OBJECT
  Q_PROPERTY(
      QString statusMessage READ statusMessage NOTIFY statusMessageChanged)
  QML_ELEMENT

public:
  explicit ViewerController(QObject *parent = nullptr);

  QString statusMessage() const;

  Q_INVOKABLE void openFile(const QString &path);
  Q_INVOKABLE void performTrimming();

signals:
  void statusMessageChanged();

public slots:
  void setStatusMessage(const QString &msg);

private:
  QString m_statusMessage;
};
