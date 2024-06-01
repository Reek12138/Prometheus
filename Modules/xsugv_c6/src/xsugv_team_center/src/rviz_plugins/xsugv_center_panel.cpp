#include "xsugv_team_center/rviz_plugins/xsugv_center_panel.h"
#include <QtConcurrent/QtConcurrent>
#include <QNetworkInterface>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QToolButton>
#include <QFileDialog>
#include <QDateTime>
#include <QComboBox>
#include <QDebug>
#include <QMenu>

#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/MarkerArray.h>

#include "xsugv_team_center/RosMasterRelayComm.h"

namespace xsugv {

#define PARTNER_GROUP_IP    "224.0.2.100"
#define PARTNER_GROUP_PORT  0x4336
#define PARTNER_BUFFER_SIZE 4096
#define PARTNER_MAGIC       0x58535853

static QFrame *createHLine(QWidget *parent)
{
    QFrame *line = new QFrame(parent);
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    return line;
}

XSUGVCenterPanel::XSUGVCenterPanel(QWidget *parent)
    : rviz::Panel(parent)
{
    mBuffer.resize(PARTNER_BUFFER_SIZE);

    QVBoxLayout *vLayout = new QVBoxLayout(this);
    QHBoxLayout *hLayout = new QHBoxLayout();
    QLabel *networkLabel = new QLabel("选择网络", this);
    QFrame *line = createHLine(this);
    QComboBox *networks = new QComboBox(this);
    QToolButton *btnLoadMap = new QToolButton(this);

    btnLoadMap->setText("切换地图");
    btnLoadMap->setMenu(new QMenu(btnLoadMap));
    btnLoadMap->setPopupMode(QToolButton::MenuButtonPopup);

    mNodeHandle = std::make_shared<ros::NodeHandle>(std::string());
    mPartnerMarkersPub = std::make_shared<ros::Publisher>(mNodeHandle->advertise<visualization_msgs::MarkerArray>("partner_markers", 1));

    // 加载静态地图方法
    auto doLoadMap = [this](const QString &filePath) {
        QStringList ips;
        foreach (ItemWidget *item, mScrollArea->widget()->findChildren<ItemWidget *>()) {
            ips.append(item->property("ip").toString());
        }
        loadMap(filePath, ips);
    };
    // 打开文件选择对话框, 选择地图加载
    connect(btnLoadMap, &QToolButton::clicked, this, [this, doLoadMap](){
        QString filePath = QFileDialog::getOpenFileName(this, "选择地图", "", "*.yaml");
        if (!filePath.isEmpty()) {
            doLoadMap(filePath);
        }
    });
    // 选择~/.ros/maps目录下地图文件快捷加载
    foreach (const QFileInfo &file_info, QDir(QDir::homePath() + "/.ros/maps").entryInfoList({"*.yaml"})) {
        QString filePath = file_info.filePath();
        btnLoadMap->menu()->addAction(file_info.baseName(), [doLoadMap, filePath](){
            doLoadMap(filePath);
        });
    }
    // 选择网卡进行组播通信, 接收所有车辆数据
    connect(networks, static_cast<void(QComboBox::*)(const QString&)>(&QComboBox::currentIndexChanged), this, [this, networkLabel](const QString &index){
        QString name = mUdp.property("iface").toString();
        if (!name.isEmpty()) {
            mUdp.leaveMulticastGroup(QHostAddress(PARTNER_GROUP_IP), QNetworkInterface::interfaceFromName(name));
            mUdp.setProperty("iface", QVariant());
            networkLabel->setStyleSheet("color: black;");
        }
        if (mUdp.joinMulticastGroup(QHostAddress(PARTNER_GROUP_IP), QNetworkInterface::interfaceFromName(index))) {
            mUdp.setProperty("iface", index);
            networkLabel->setStyleSheet("color: blue;");
        } else {
            networkLabel->setStyleSheet("color: red;");
        }
    });

    mInfoLabel = new QLabel(this);
    mScrollArea = new QScrollArea(this);
    mScrollArea->setWidget(new QWidget(mScrollArea));
    mScrollArea->widget()->setLayout(new QVBoxLayout());
    mScrollArea->setFrameShape(QFrame::NoFrame);
    mScrollArea->setWidgetResizable(true);
    mInfoLabel->setAlignment(Qt::AlignCenter);

    hLayout->addWidget(networkLabel);
    hLayout->addSpacing(4);
    hLayout->addWidget(networks);
    hLayout->addWidget(mInfoLabel, 1);
    hLayout->addWidget(btnLoadMap);
    vLayout->addLayout(hLayout);
    vLayout->addSpacing(6);
    vLayout->addWidget(line);
    vLayout->addWidget(mScrollArea);
    vLayout->setSpacing(0);

    mUdp.bind(QHostAddress::AnyIPv4, PARTNER_GROUP_PORT, QUdpSocket::ShareAddress);
    connect(&mUdp, &QUdpSocket::readyRead, this, &XSUGVCenterPanel::onUdp);

    connect(&mTimer, &QTimer::timeout, this, &XSUGVCenterPanel::onTimer);
    mTimer.start(200);

    foreach (const QNetworkInterface &iface, QNetworkInterface::allInterfaces()) {
        networks->addItem(iface.name());
        if (iface.name() != "lo" && networks->currentText() == "lo") {
            networks->setCurrentText(iface.name());
        }
    }

    vLayout = static_cast<QVBoxLayout *>(mScrollArea->widget()->layout());
    vLayout->addStretch(1);
    vLayout->setSpacing(0);
    vLayout->setMargin(0);
    updateInfo();

    connect(this, &XSUGVCenterPanel::mapLoadState, this, [this](const QString &ip, int state, const QString &message){
        foreach (ItemWidget *item, mScrollArea->widget()->findChildren<ItemWidget *>()) {
            if (item->ip() == ip) {
                item->setMapLoadState(state, message);
                break;
            }
        }
        if (state == 2 || state == -2) {
            setProperty("busy", false);
        }
    });
}

XSUGVCenterPanel::~XSUGVCenterPanel()
{
    ros::shutdown();
}

void XSUGVCenterPanel::onUdp()
{
    if (mUdp.hasPendingDatagrams()) {
        quint16 port;
        QHostAddress address;
        qint64 ret = mUdp.readDatagram(mBuffer.data(), mBuffer.size(), &address, &port);
        if (ret > 0) {
            xsugv_msgs::XSUGVPartner partner_data;
            partner_data.magic = 0;
            ros::serialization::IStream stream((uint8_t *)mBuffer.data(), ret);
            try {
                ros::serialization::deserialize(stream, partner_data);
            } catch(ros::serialization::StreamOverrunException e) {
                ROS_WARN("deserialize error: size=%lld", ret);
                return;
            }
            if (partner_data.magic == PARTNER_MAGIC) {
                ItemWidget *item = findItem(partner_data.id);
                if (item) {
                    item->setPartnerData(partner_data);
                    item->updateTimestamp();
                } else {
                    addItem(partner_data, address.toString(), port);
                }
            }
        }
    }
}

void XSUGVCenterPanel::onTimer()
{
    QList<qint32> ids;
    qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    visualization_msgs::MarkerArray array;

    foreach (ItemWidget *item, mScrollArea->widget()->findChildren<ItemWidget *>()) {
        const QString mesh;// = "file:///data/ros/xsugv_c6/src/xsugv_description/meshes/c6_colors.dae";
        bool remove = false;
        if (item->isTimeout()) {
            ids.append(item->id());
            remove = true;
        }
        auto markers = item->getMarkers(remove, mesh, 0.23);
        for (auto it=markers.begin(); it!=markers.end(); ++it) {
            array.markers.push_back(*it);
        }
    }
    for (int i=0; i<ids.size(); i++) {
        removeItem(ids.at(i));
    }
    foreach (ItemWidget *item, mScrollArea->widget()->findChildren<ItemWidget *>()) {
        double dt = (nowMs - item->timestamp()) * 0.001;
        item->setStatus(QString::asprintf("%.1fs", dt), dt < 5 ? "color:green;" : "color:red;");
    }
    if (array.markers.size() > 0) {
        mPartnerMarkersPub->publish(array);
    }
}

void XSUGVCenterPanel::updateInfo()
{
    mInfoLabel->setText(tr("在线车辆: %1").arg(mScrollArea->widget()->findChildren<ItemWidget *>().count()));
}

void XSUGVCenterPanel::addItem(const xsugv_msgs::XSUGVPartner &partner_data, const QString &ip, int port)
{
    ItemWidget *item = findItem(partner_data.id);
    if (item) {
        item->setPartnerData(partner_data);
    } else {
        QVBoxLayout *vLayout = static_cast<QVBoxLayout *>(mScrollArea->widget()->layout());
        int index = vLayout->count() > 0 ? vLayout->count() - 1 : 0;
        QFrame *line = createHLine(mScrollArea->widget());
        ItemWidget *item = new ItemWidget(partner_data, ip, port, mScrollArea->widget());
        line->setObjectName(QString("Line%1").arg(partner_data.id));
        item->setObjectName(QString("Item%1").arg(partner_data.id));
        vLayout->insertWidget(index, line);
        vLayout->insertWidget(index, item);
        updateInfo();

        connect(item, &ItemWidget::click, this, [this](ItemWidget *item){
            if (property("busy").toBool()) {
                ROS_WARN("Busy");
                return;
            }
            setProperty("busy", true);
            foreach (ItemWidget *i, mScrollArea->widget()->findChildren<ItemWidget *>()) {
                i->setSelected(i == item);
            }
            QtConcurrent::run([this, item](){
                xsugv_team_center::RosMasterRelayCommRequest request;
                xsugv_team_center::RosMasterRelayCommResponse response;
                request.action = "connect";
                request.ip = item->ip().toStdString();
                emit mapLoadState(item->ip(), 0, "获取数据..");
                if (ros::service::call("/relay/comm", request, response) && response.code == 0) {
                    emit mapLoadState(item->ip(), 2, "获取成功");
                } else {
                    emit mapLoadState(item->ip(), -2, "获取失败");
                }
            });
        });
    }
}

void XSUGVCenterPanel::removeItem(quint32 id)
{
    ItemWidget *item = findItem(id);
    QFrame *line = mScrollArea->widget()->findChild<QFrame *>(QString("Line%1").arg(id));
    if (item) delete item;
    if (line) delete line;
    updateInfo();
}

ItemWidget *XSUGVCenterPanel::findItem(quint32 id)
{
    return mScrollArea->widget()->findChild<ItemWidget *>(QString("Item%1").arg(id));
}

void XSUGVCenterPanel::setStaticMap(std::shared_ptr<QStringList> ips, int index, QString mapFile)
{
    if (ips && index < ips->count()) {
        QtConcurrent::run([this](std::shared_ptr<QStringList> ips, int index, QString mapFile){
            xsugv_team_center::RosMasterRelayCommRequest request;
            xsugv_team_center::RosMasterRelayCommResponse response;
            request.action = "load_map";
            request.path = mapFile.toStdString();
            request.ip = ips->at(index).toStdString();
            emit mapLoadState(ips->at(index), 0, "地图切换..");
            if (ros::service::call("/relay/comm", request, response) && response.code == 0) {
                emit mapLoadState(ips->at(index), 1, "地图切换成功");
            } else {
                emit mapLoadState(ips->at(index), -1, "地图切换失败");
            }
            setStaticMap(ips, index + 1, mapFile);
        }, ips, index, mapFile);
    }
}

void XSUGVCenterPanel::loadMap(const QString &mapFile, const QStringList &ipList)
{
    std::shared_ptr<QStringList> ips = std::make_shared<QStringList>(ipList);
    setStaticMap(ips, 0, mapFile);
}

PLUGINLIB_EXPORT_CLASS(xsugv::XSUGVCenterPanel, rviz::Panel)

} // namespace xsugv

