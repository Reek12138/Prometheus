#ifndef XSUGV_XSUGVCENTERPANEL_H
#define XSUGV_XSUGVCENTERPANEL_H

#include <QLabel>
#include <QScrollArea>
#include <QUdpSocket>
#include <QTimer>

#include <memory>
#include <ros/ros.h>
#include <rviz/panel.h>

#include "xsugv_team_center/rviz_plugins/item_widget.h"

namespace xsugv {

class XSUGVCenterPanel : public rviz::Panel 
{
    Q_OBJECT

public:
    XSUGVCenterPanel(QWidget *parent = nullptr);
    ~XSUGVCenterPanel();

signals:
    void mapLoadState(const QString &ip, int state, const QString &message);

private slots:
    void onUdp();
    void onTimer();
    void updateInfo();
    void addItem(const xsugv_msgs::XSUGVPartner &partner_data, const QString &ip, int port);
    void removeItem(quint32 id);
    ItemWidget *findItem(quint32 id);

private:
    void loadMap(const QString &mapFile, const QStringList &ipList);
    void setStaticMap(std::shared_ptr<QStringList> ips, int index, QString mapFile);

private:
    QUdpSocket mUdp;
    QTimer mTimer;
    QLabel *mInfoLabel;
    QScrollArea *mScrollArea;
    std::shared_ptr<ros::NodeHandle> mNodeHandle;
    std::shared_ptr<ros::Publisher> mPartnerMarkersPub;
    QByteArray mBuffer;
};

}

#endif // XSUGV_XSUGVCENTERPANEL_H
