#include <QtGlobal>
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
#error "This application requires Qt 6.0 or higher."
#endif

#include <QDebug>
#include <QGuiApplication>
#include <QQmlApplicationEngine>

int main(int argc, char *argv[]) {
  QGuiApplication app(argc, argv);

  QQmlApplicationEngine engine;
  engine.loadFromModule("ViewerModule", "Main");

  return app.exec();
}
