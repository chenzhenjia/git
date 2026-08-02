#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <memory>

#include <QMainWindow>
#include <QProgressDialog>
#include <QTimer>

#include "core/Types.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class QThread;
class LoadWorker;
class PointCloudViewerWidget;
class MmapPlyReader;
class PointOctree;
namespace open3d { namespace geometry { class PointCloud; } }

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    void onLoadPly();
    void onPreviewReady(std::shared_ptr<open3d::geometry::PointCloud> cloud);
    void onFullCloudReady(std::shared_ptr<open3d::geometry::PointCloud> cloud);
    void onStatsReady(const CloudStats &stats);
    void onLoadFailed(const QString &errorMsg);

    // slider
    void onSliderTimerTick();
    void onSliderValueChanged(int sliderValue);
    void onAdaptiveClicked();

private:
    void initWorker();
    void cleanupWorker();
    void startLoad(const QString &path);
    int  logSliderToPoints(int sv) const;
    int  pointsToLogSlider(int pts) const;
    void applyRenderCount(int count);
    void applySse(float sse);

    Ui::MainWindow *ui;
    QThread         *m_loadThread = nullptr;
    LoadWorker      *m_loadWorker = nullptr;
    QProgressDialog  *m_progressDialog = nullptr;
    PointCloudViewerWidget *m_viewer = nullptr;

    QTimer m_sliderTimer;
    int    m_pendingSliderVal = 500;
    bool   m_sliderDirty = false;

    std::shared_ptr<MmapPlyReader> m_mmapReader;
    std::shared_ptr<PointOctree>   m_octree;
    bool m_useMmap = false;

    std::shared_ptr<open3d::geometry::PointCloud> m_cloud;
    CloudStats m_stats;
    int m_totalPoints = 0;
};
#endif