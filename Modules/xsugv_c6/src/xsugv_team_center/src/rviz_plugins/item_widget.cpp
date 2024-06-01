#include "xsugv_team_center/rviz_plugins/item_widget.h"
#include <QStyleOption>
#include <QGridLayout>
#include <QPainter>
#include <QEvent>
#include <QStyle>
#include <QDateTime>
#include <QDebug>

#include <tf/tf.h>

namespace xsugv {

ItemWidget::ItemWidget(const xsugv_msgs::XSUGVPartner &partner_data, const QString &ip, int port, QWidget *parent)
    : QWidget{parent},
      mPartnerData(partner_data),
      mTimestampMs(0)
{
    QGridLayout *gridLayout = new QGridLayout(this);

    mNameLabel = new QLabel(mPartnerData.name.c_str(), this);
    mAddressLabel = new QLabel(QString::asprintf("%s:%d", ip.toStdString().c_str(), port), this);
    mStatusLabel = new QLabel("-", this);
    mSocLabel = new QLabel(this);
    gridLayout->addWidget(mNameLabel, 0, 0);
    gridLayout->addWidget(mAddressLabel, 1, 0);
    gridLayout->addWidget(mStatusLabel, 1, 1);
    gridLayout->addWidget(mSocLabel, 0, 1);
    gridLayout->setMargin(8);

    mStatusLabel->setAlignment(Qt::AlignRight);
    mSocLabel->setAlignment(Qt::AlignRight);
    setSelected(false);

    setProperty("ip", ip);

    updateTimestamp();
    updateInfo();

    setProperty("item", true);
    installEventFilter(this);
}

void ItemWidget::setSelected(bool selected)
{
    setProperty("selected", selected);
    mNameLabel->setStyleSheet(QString("font-size: 20px; font-weight: %1; color: %2").arg(selected ? "bold" : "normal").arg(selected ? "blue" : "black"));
}

void ItemWidget::setMapLoadState(int state, const QString &message)
{
    mStatusLabel->setText(message);
    if (state == 0) {
        mStatusLabel->setProperty("waiting", true);
        mStatusLabel->setStyleSheet("");
    } else {
        mStatusLabel->setProperty("waiting", false);
        mStatusLabel->setProperty("time", QTime::currentTime().msecsSinceStartOfDay());
        if (state == 1 && property("selected").toBool()) {
            emit click(this);
        }
    }
}

bool ItemWidget::eventFilter(QObject *watched, QEvent *event)
{
    if (watched == this) {
        auto setBg = [this](int a) { setStyleSheet(QString("QWidget[item=\"true\"]{background-color:rgba(0,0,0,%1);}").arg(a)); };
        if (event->type() == QEvent::Enter) {
            setCursor(Qt::PointingHandCursor);
            setBg(20);
        } else if (event->type() == QEvent::Leave) {
            setCursor(Qt::ArrowCursor);
            setBg(0);
        } else if (event->type() == QEvent::MouseButtonPress) {
            setBg(40);
            emit click(this);
        } else if (event->type() == QEvent::MouseButtonRelease) {
            setBg(20);
        } else {
            return false;
        }
        return true;
    }
    return false;
}

void ItemWidget::paintEvent(QPaintEvent *)
{
    QStyleOption opt;
    opt.init(this);
    QPainter p(this);
    style()->drawPrimitive(QStyle::PE_Widget, &opt, &p, this);
}

quint32 ItemWidget::id() const
{
    return mPartnerData.id;
}

qint64 ItemWidget::timestamp() const
{
    return mTimestampMs;
}

QString ItemWidget::ip() const
{
    return property("ip").toString();
}

bool ItemWidget::isTimeout() const
{
    return QDateTime::currentMSecsSinceEpoch() - mTimestampMs > 10000;
}

void ItemWidget::updateTimestamp()
{
    mTimestampMs = QDateTime::currentMSecsSinceEpoch();
}

void ItemWidget::updateInfo()
{
    mSocLabel->setText(QString::asprintf("%s | %.1fkm/h | %.1f%%",
                                         mPartnerData.mode == 3 ? "Auto" : mPartnerData.mode == 2 ? "Remote" : mPartnerData.mode == 1 ? "Joy" : "None",
                                         mPartnerData.linear_velocity * 3.6, mPartnerData.battery));
}

ItemWidget& ItemWidget::setStatus(const QString &status, const QString &qss)
{
    if (!mStatusLabel->property("waiting").toBool() && QTime::currentTime().msecsSinceStartOfDay() - mStatusLabel->property("time").toInt() > 2000) {
        mStatusLabel->setText(status);
        if (!qss.isEmpty()) {
            mStatusLabel->setStyleSheet(qss);
        }
    }
    return *this;
}

ItemWidget& ItemWidget::setPartnerData(const xsugv_msgs::XSUGVPartner &partner_data)
{
    mPartnerData = partner_data;
    updateInfo();
    return *this;
}

bool ItemWidget::isMe(quint32 id) const
{
    return mPartnerData.id == id;
}

std::vector<visualization_msgs::Marker> ItemWidget::getMarkers(bool remove, const QString &mesh_path, double z) const
{
    bool lost = QDateTime::currentMSecsSinceEpoch() - mTimestampMs > 5000;
    std::vector<visualization_msgs::Marker> markers;
    visualization_msgs::Marker marker;

    if (mesh_path.isEmpty()) {
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.action = remove ? visualization_msgs::Marker::DELETE : visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.lifetime = ros::Duration();
        marker.id = mPartnerData.id;

        marker.color.r = lost ? 1 : 0;
        marker.color.g = lost ? 0 : 1;
        marker.color.b = 0;
        marker.color.a = 1;

        marker.scale.x = 0.6; // mPartnerData.left_size + mPartnerData.right_size;
        marker.scale.y = 0.2; // mPartnerData.front_size + mPartnerData.back_size;
        marker.scale.z = 0.2; // mPartnerData.height;

        marker.pose = mPartnerData.pose;
        marker.pose.position.z = marker.scale.z / 2;

        markers.push_back(marker);

        marker.id = 0x00010000 + marker.id;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.text = mPartnerData.name.c_str();
        marker.pose.position.z += 0.2;

        markers.push_back(marker);
    } else {
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.action = remove ? visualization_msgs::Marker::DELETE : visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.lifetime = ros::Duration();
        marker.id = mPartnerData.id;
        marker.mesh_use_embedded_materials = true;
        marker.mesh_resource = mesh_path.toStdString();

        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;

        marker.pose = mPartnerData.pose;
        marker.pose.position.z = z;

        markers.push_back(marker);

        marker.id = 0x00010000 + marker.id;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.text = mPartnerData.name.c_str();
        marker.pose.position.z += 0.2;
        marker.color.r = lost ? 1 : 0;
        marker.color.g = lost ? 0 : 1;
        marker.color.b = 0;
        marker.color.a = 1;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        markers.push_back(marker);
    }

    return markers;
}

} // namespace xsugv

