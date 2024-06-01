#ifndef XSUGV_ITEMWIDGET_H
#define XSUGV_ITEMWIDGET_H

#include <QLabel>
#include <visualization_msgs/Marker.h>
#include "xsugv_msgs/XSUGVPartner.h"

namespace xsugv {

class ItemWidget : public QWidget
{
    Q_OBJECT
public:
    explicit ItemWidget(const xsugv_msgs::XSUGVPartner &partner_data, const QString &ip, int port, QWidget *parent = nullptr);

    ItemWidget& setStatus(const QString &status, const QString &qss="");
    ItemWidget& setPartnerData(const xsugv_msgs::XSUGVPartner &partner_data);
    bool isMe(quint32 id) const;
    bool isTimeout() const;
    void updateTimestamp();
    quint32 id() const;
    qint64 timestamp() const;
    QString ip() const;

    std::vector<visualization_msgs::Marker> getMarkers(bool remove, const QString &mesh_path = "", double z = 1.0) const;

signals:
    void click(ItemWidget *item);

public slots:
    void setSelected(bool selected);
    void setMapLoadState(int state, const QString &message);

protected:
    void paintEvent(QPaintEvent *);
    bool eventFilter(QObject *watched, QEvent *event);

private:
    void updateInfo();

private:
    QLabel *mNameLabel;
    QLabel *mAddressLabel;
    QLabel *mStatusLabel;
    QLabel *mSocLabel;
    qint64 mTimestampMs;
    xsugv_msgs::XSUGVPartner mPartnerData;
};

} // namespace xsugv

#endif // XSUGV_ITEMWIDGET_H
