#include "ViewerController.h"

ViewerController::ViewerController(QObject *parent)
    : QObject(parent), m_statusMessage("Ready") {}

QString ViewerController::statusMessage() const { return m_statusMessage; }

void ViewerController::setStatusMessage(const QString &msg) {
  if (m_statusMessage != msg) {
    m_statusMessage = msg;
    emit statusMessageChanged();
  }
}

void ViewerController::openFile(const QString &path) {
  qDebug() << "Opening file:" << path;
  setStatusMessage("Opened: " + path);
  // TODO: Call Trimming lib interface
}

void ViewerController::performTrimming() {
  qDebug() << "Performing trimming operation...";
  setStatusMessage("Trimming in progress...");
  // TODO: Call Trimming lib interface
}
