#include "mainwindow.h"

#include <QApplication>

#include <open3d/geometry/PointCloud.h>
#include <open3d/utility/Logging.h>

Q_DECLARE_METATYPE(std::shared_ptr<open3d::geometry::PointCloud>)

int main(int argc, char *argv[])
{
    qRegisterMetaType<std::shared_ptr<open3d::geometry::PointCloud>>(
        "std::shared_ptr<open3d::geometry::PointCloud>");

    open3d::utility::SetVerbosityLevel(
        open3d::utility::VerbosityLevel::Warning);

    QApplication a(argc, argv);
    QApplication::setApplicationName("PointCloudProcessor");
    QApplication::setApplicationVersion("0.1.0");
    QApplication::setOrganizationName("PCP");

    MainWindow w;
    w.resize(1280, 800);
    w.setWindowTitle("PointCloudProcessor - 2.5D Point Cloud Tool");
    w.show();
    return QApplication::exec();
}