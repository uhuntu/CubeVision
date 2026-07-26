#include <QApplication>
#include "MainWindow.h"
#include <QFile>
#include <QDateTime>
#include <QDebug>
#include <QStandardPaths>
#include <QDir>

static QFile *g_logFile=nullptr;

static void logMessageHandler(QtMsgType type,const QMessageLogContext &context,const QString &msg){
    if(!g_logFile || !g_logFile->isOpen())
        return;
    const QString level=[type]{
        switch(type){
        case QtDebugMsg: return QStringLiteral("DBG");
        case QtInfoMsg: return QStringLiteral("INF");
        case QtWarningMsg: return QStringLiteral("WRN");
        case QtCriticalMsg: return QStringLiteral("ERR");
        case QtFatalMsg: return QStringLiteral("FTL");
        }
        return QStringLiteral("???");
    }();
    const QString line=QStringLiteral("%1 [%2] %3\n")
        .arg(QDateTime::currentDateTime().toString(Qt::ISODateWithMs))
        .arg(level)
        .arg(msg);
    g_logFile->write(line.toUtf8());
    g_logFile->flush();
}

int main(int argc,char *argv[]){
    const QString logDir=QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    QDir().mkpath(logDir);
    g_logFile=new QFile(logDir+"/CubeVision.log");
    if(g_logFile->open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text))
        qInstallMessageHandler(logMessageHandler);
    QApplication app(argc,argv);
    MainWindow w;
    w.show();
    const int ret=app.exec();
    delete g_logFile;
    return ret;
}
