#include <QGuiApplication>
#include <QQmlApplicationEngine>

int main(int argc, char *argv[]) {
    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;
    
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [](QObject *obj, const QUrl &objUrl) {
        if (!obj)
            QCoreApplication::exit(-1);
    }, Qt::QueuedConnection);

    engine.loadFromModule("ViewerModule", "Main");

    return app.exec();
}
