#include "PointCloud.h"
#define QT_NO_QTENTRYPOINT

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);

    // スタイルの適用（qrcに登録済み）
    QFile file("style.qss");
    if (file.open(QFile::ReadOnly | QFile::Text)) {
        QTextStream stream(&file);
        QString style = stream.readAll();
        app.setStyleSheet(style);
    }

    PointCloud window;
    window.showNormal();
    window.setMinimumSize(1000, 618);

    return app.exec();
}
