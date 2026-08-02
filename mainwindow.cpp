#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <cmath>

#include <QCloseEvent>
#include <QDebug>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QStyle>
#include <QThread>

#include <open3d/geometry/PointCloud.h>

#include "core/MmapPlyReader.h"
#include "core/Octree.h"
#include "ui/PointCloudViewerWidget.h"
#include "workers/LoadWorker.h"

static QByteArray peekFileHeader(const QString &path, int nBytes = 1024) {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) return {};
    return f.read(nBytes);
}

MainWindow::MainWindow(QWidget *parent):QMainWindow(parent),ui(new Ui::MainWindow){
    ui->setupUi(this);m_viewer=ui->viewerWidget;
    ui->sidePanel->setStyleSheet(
        "QWidget#sidePanel{background:#f4f5f7;border-left:1px solid #d8dce3;}"
        "QLabel#panelTitle,QLabel#displaySection{color:#1f2937;font-weight:600;font-size:12px;letter-spacing:1px;}"
        "QPushButton#loadPlyButton{background:#2563eb;color:white;border:none;border-radius:6px;padding:0 14px;font-size:13px;font-weight:600;min-height:36px;}"
        "QPushButton#loadPlyButton:hover{background:#1d4ed8;}"
        "QPushButton#loadPlyButton:pressed{background:#1e40af;}"
        "QPushButton#adaptiveButton{background:#e5e7eb;color:#374151;border:none;border-radius:6px;padding:0 10px;font-size:12px;min-height:30px;}"
        "QPushButton#adaptiveButton:hover{background:#d1d5db;}"
        "QLabel#pointCountLabel{color:#374151;font-size:12px;}"
        "QLabel#vramLabel{color:#6b7280;font-size:11px;}");
    ui->loadPlyButton->setIcon(style()->standardIcon(QStyle::SP_DialogOpenButton));
    ui->loadPlyButton->setIconSize(QSize(18,18));
    connect(ui->loadPlyButton,&QPushButton::clicked,this,&MainWindow::onLoadPly);
    connect(ui->adaptiveButton,&QPushButton::clicked,this,&MainWindow::onAdaptiveClicked);

    m_sliderTimer.setInterval(30);m_sliderTimer.setSingleShot(true);
    connect(&m_sliderTimer,&QTimer::timeout,this,&MainWindow::onSliderTimerTick);
    connect(ui->pointSlider,&QSlider::valueChanged,this,&MainWindow::onSliderValueChanged);

    connect(m_viewer,&PointCloudViewerWidget::vramUsageChanged,this,[this](int64_t mb){
        ui->vramLabel->setText(QString("VRAM: %1 MB").arg(mb));
        ui->vramLabel->setStyleSheet(mb>5800?"color:#dc2626;font-size:11px;font-weight:600;":"color:#6b7280;font-size:11px;");
    });

    initWorker();setWindowTitle("PointCloudProcessor");
}
MainWindow::~MainWindow(){cleanupWorker();delete ui;}

void MainWindow::initWorker(){
    m_loadThread=new QThread(this);m_loadWorker=new LoadWorker;
    m_loadWorker->moveToThread(m_loadThread);
    connect(m_loadWorker,&LoadWorker::loadStarted,this,[this](){if(m_progressDialog){m_progressDialog->setValue(0);m_progressDialog->show();}});
    connect(m_loadWorker,&LoadWorker::previewReady,this,&MainWindow::onPreviewReady);
    connect(m_loadWorker,&LoadWorker::fullCloudReady,this,&MainWindow::onFullCloudReady);
    connect(m_loadWorker,&LoadWorker::statsReady,this,&MainWindow::onStatsReady);
    connect(m_loadWorker,&LoadWorker::progressUpdated,this,[this](int p){if(m_progressDialog)m_progressDialog->setValue(p);});
    connect(m_loadWorker,&LoadWorker::loadFailed,this,&MainWindow::onLoadFailed);
    connect(m_loadWorker,&LoadWorker::mmapReady,this,[this](auto reader,auto octree,auto stats){
        if(m_progressDialog){m_progressDialog->close();m_progressDialog->deleteLater();m_progressDialog=nullptr;}
        m_mmapReader=reader;m_octree=octree;m_useMmap=true;m_stats=stats;
        m_totalPoints=static_cast<int>(stats.total_points);
        ui->sliderMaxLabel->setText("SSE");ui->sliderMinLabel->setText("0.01");
        m_viewer->setMmapSource(reader,octree,stats);
        applySse(0.5f);
        statusBar()->showMessage(QString("Mmap mode: %L1 points, %2 MB").arg(stats.total_points).arg(stats.file_size_bytes/(1024*1024)));
    });
    connect(m_loadThread,&QThread::finished,m_loadWorker,&QObject::deleteLater);
    m_loadThread->start();
}
void MainWindow::cleanupWorker(){if(m_loadWorker)m_loadWorker->cancel();if(m_loadThread){m_loadThread->quit();m_loadThread->wait(5000);}}

void MainWindow::onLoadPly(){
    QString path=QFileDialog::getOpenFileName(this,"Load Point Cloud",QString(),"Point Cloud (*.ply *.pcd *.xyz *.pts);;All (*)");
    if(path.isEmpty())return;startLoad(path);
}
void MainWindow::startLoad(const QString &path){
    delete m_progressDialog;
    m_progressDialog=new QProgressDialog("Loading...","Cancel",0,100,this);
    m_progressDialog->setWindowModality(Qt::WindowModal);m_progressDialog->setMinimumDuration(0);m_progressDialog->setAutoClose(false);
    connect(m_progressDialog,&QProgressDialog::canceled,this,[this](){if(m_loadWorker)m_loadWorker->cancel();});

    LoadConfig cfg;cfg.preview_target_points=50000;cfg.auto_preview_threshold=500000;

    // Format detection using QFile (supports Windows Chinese paths)
    QByteArray headerBytes=peekFileHeader(path);
    std::string peek=headerBytes.toStdString();
    bool isBinary=(peek.find("binary_little_endian")!=std::string::npos||
                   peek.find("binary_big_endian")!=std::string::npos);
    bool isAscii=(peek.find("format ascii")!=std::string::npos);
    if(!isBinary&&!isAscii){
        qWarning()<<"[MainWindow] Cannot determine PLY format, defaulting to Open3D";
    }

    // File size via QFileInfo (safe for Chinese paths)
    QFileInfo fi(path);
    if(!fi.exists()){onLoadFailed("File not found: "+path);return;}
    qint64 fsz=fi.size();

    if(isBinary&&fsz>cfg.mmap_threshold_bytes){
        qDebug()<<"[MainWindow] binary large file, routing to mmap:"<<fsz/1024/1024<<"MB";
        QMetaObject::invokeMethod(m_loadWorker,[this,path,cfg](){m_loadWorker->startMmapLoad(path,cfg);},Qt::QueuedConnection);
        return;
    }
    qDebug()<<"[MainWindow] Routing to Open3D path";
    QMetaObject::invokeMethod(m_loadWorker,[this,path,cfg](){m_loadWorker->startLoad(path,cfg);},Qt::QueuedConnection);
}

void MainWindow::onPreviewReady(std::shared_ptr<open3d::geometry::PointCloud> c){
    m_cloud=c;m_useMmap=false;m_totalPoints=static_cast<int>(c->points_.size());
    m_viewer->setPointCloud(c);m_viewer->setRenderCount(std::min(m_totalPoints,50000));
    applyRenderCount(std::min(m_totalPoints,50000));
    statusBar()->showMessage("Preview loaded (full cloud loading...)");
}
void MainWindow::onFullCloudReady(std::shared_ptr<open3d::geometry::PointCloud> c){
    m_cloud=c;m_useMmap=false;m_mmapReader.reset();m_octree.reset();
    m_totalPoints=static_cast<int>(c->points_.size());
    ui->sliderMinLabel->setText("1K");
    ui->sliderMaxLabel->setText(m_totalPoints>=1000000?QString("%1M").arg(m_totalPoints/1e6,0,'f',1):QString("%1K").arg(m_totalPoints/1000));
    m_viewer->setPointCloud(c);
    applyRenderCount(std::min(m_totalPoints,50000));
    statusBar()->showMessage(QString("Loaded %L1 points").arg(m_totalPoints));
}
void MainWindow::onStatsReady(const CloudStats &s){m_stats=s;}
void MainWindow::onLoadFailed(const QString &e){
    if(m_progressDialog){m_progressDialog->close();m_progressDialog->deleteLater();m_progressDialog=nullptr;}
    QMessageBox::critical(this,"Load Failed",e);
}

int MainWindow::logSliderToPoints(int sv)const{
    if(m_totalPoints<=1000)return m_totalPoints;
    double t=std::clamp(sv/1000.,0.,1.),lo=log10(1000.),hi=log10((double)m_totalPoints);
    return static_cast<int>(round(pow(10.,lo+t*(hi-lo))));
}
int MainWindow::pointsToLogSlider(int pts)const{
    if(m_totalPoints<=1000)return 0;
    double lo=log10(1000.),hi=log10((double)m_totalPoints),t=(log10((double)pts)-lo)/(hi-lo);
    return static_cast<int>(round(t*1000.));
}
void MainWindow::onSliderValueChanged(int sv){m_pendingSliderVal=sv;if(!m_sliderDirty){m_sliderDirty=true;m_sliderTimer.start();}}
void MainWindow::onSliderTimerTick(){
    m_sliderDirty=false;
    if(m_useMmap){applySse(0.01f+(m_pendingSliderVal/1000.f)*9.99f);}
    else{applyRenderCount(logSliderToPoints(m_pendingSliderVal));}
}
void MainWindow::applyRenderCount(int count){
    count=std::clamp(count,1000,m_totalPoints);m_viewer->setRenderCount(count);
    ui->pointCountLabel->setText(QString("Points: %L1 / %L2").arg(count).arg(m_totalPoints));
}
void MainWindow::applySse(float sse){
    m_viewer->setSseThreshold(sse);
    int rend=m_viewer->renderedPoints();
    ui->pointCountLabel->setText(QString("SSE: %1 px | ~%L2 pts").arg(sse,0,'f',2).arg(rend));
}
void MainWindow::onAdaptiveClicked(){
    if(m_useMmap){
        float w=m_viewer->width(),h=m_viewer->height(),sse=std::clamp(sqrtf(w*w+h*h)/100.f,.1f,3.f);
        int sv=static_cast<int>((sse-.01f)/9.99f*1000.f);ui->pointSlider->setValue(sv);applySse(sse);
    }else{
        int adaptive=std::clamp(static_cast<int>(m_viewer->width()*m_viewer->height()*.8),1000,m_totalPoints);
        ui->pointSlider->setValue(pointsToLogSlider(adaptive));applyRenderCount(adaptive);
    }
}